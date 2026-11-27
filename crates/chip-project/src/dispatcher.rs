use std::{
    collections::{BTreeMap, HashMap, HashSet},
    path::Path,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

use chip_cad::CadDocument;
use chip_cam::{
    CamError, FixtureKind, GeneratedToolpath, Operation, OperationGenRequest, OperationRotaryMode,
    OperationStatus, Setup, SharedAvoidRegion, UpstreamSweptInput,
    generate_operation_toolpath_with_fixtures, normalize_indexed_angle_deg, resolve_setup_fixtures,
};
use chip_math::DisplayUnit;
use chip_sim::{SimComputeBackendChoice, TriDexelStock};
use chip_sketch::SketchLayer;
use chip_tools::{Tool, resolve_machine_definition};
use crossbeam_channel::{Receiver, Sender};
use rayon::prelude::*;

use crate::{
    cache::{
        OperationInputHash, ToolpathCache, compute_in_setup_carryover_digest,
        compute_operation_input_hash, compute_setup_carryover_digest,
        op_needs_in_setup_rotary_carryover, resolve_upstream_rest_op_indices,
    },
    document::ProjectDocument,
    pipeline::{
        SetupSimRequest, compute_in_setup_carryover_stock, compute_setup_carryover_stock,
        resolve_setup_kinematics,
    },
};

/// Formats a wall-clock execution [`Duration`] into a concise human-readable string
/// (`142ms`, `1.42s`, or `2m 05.3s`) for Activity Log messages.
#[must_use]
pub fn format_exec_duration(elapsed: Duration) -> String {
    let total_ms = elapsed.as_millis();
    if total_ms < 1_000 {
        return format!("{total_ms}ms");
    }
    let secs = elapsed.as_secs_f64();
    if secs < 59.995 {
        return format!("{secs:.2}s");
    }
    let total_tenths = (secs * 10.0).round() as u64;
    let mins = total_tenths / 600;
    let rem_tenths = total_tenths % 600;
    let secs_whole = rem_tenths / 10;
    let secs_frac = rem_tenths % 10;
    format!("{mins}m {secs_whole:02}.{secs_frac}s")
}

/// Event emitted from background `rayon` workers to the UI / coordinator thread (`01-architecture.md`).
#[derive(Debug, Clone)]
pub enum DispatcherEvent {
    Started {
        op_id: String,
        input_hash: OperationInputHash,
    },
    Completed {
        op_id: String,
        input_hash: OperationInputHash,
        toolpath: Arc<GeneratedToolpath>,
        elapsed: Duration,
    },
    Cancelled {
        op_id: String,
        input_hash: OperationInputHash,
        elapsed: Duration,
    },
    Failed {
        op_id: String,
        input_hash: OperationInputHash,
        message: String,
        elapsed: Duration,
    },
}

#[derive(Clone)]
struct QueuedOpTask {
    setup: Setup,
    sketches: Arc<Vec<SketchLayer>>,
    shared_avoid_regions: Arc<Vec<SharedAvoidRegion>>,
    fixture_docs: Arc<BTreeMap<String, Arc<CadDocument>>>,
    machine_z_travel_mm: Option<f64>,
    op: Operation,
    tool: Option<Tool>,
    clearing_tool: Option<Tool>,
    display_unit: DisplayUnit,
    is_first_tool_in_setup: bool,
    upstream_rest: Vec<UpstreamSweptInput>,
    carryover_stock: Option<Arc<TriDexelStock>>,
    input_hash: OperationInputHash,
    cancel_token: Arc<AtomicBool>,
}

enum UpstreamReadiness {
    Ready(Vec<UpstreamSweptInput>),
    Waiting,
    FailedUpstream { op_name: String, message: String },
}

enum CarryoverReadiness {
    None,
    Ready(Arc<TriDexelStock>),
    NeedCompute { digest: u64 },
    WaitingUpstreamOps,
    FailedUpstream(String),
}

fn collect_transitive_carryover_setups(
    project: &ProjectDocument,
    target_setup_idx: usize,
) -> HashSet<usize> {
    let mut visited = HashSet::new();
    let mut cur = target_setup_idx;
    while let Some(setup) = project.setups.get(cur)
        && let Some(ref src_id) = setup.carryover_from_setup_id
        && let Some(src_idx) = project.setups[..cur].iter().position(|s| &s.id == src_id)
    {
        if !visited.insert(src_idx) {
            break;
        }
        cur = src_idx;
    }
    visited
}

fn check_carryover_readiness(
    project: &ProjectDocument,
    cache: &ToolpathCache,
    setup_idx: usize,
    cad_revision: u64,
    carryover_cache: &HashMap<(usize, u64), Arc<TriDexelStock>>,
    carryover_errors: &HashMap<(usize, u64), String>,
) -> CarryoverReadiness {
    let Some(setup) = project.setups.get(setup_idx) else {
        return CarryoverReadiness::None;
    };
    let Some(ref src_id) = setup.carryover_from_setup_id else {
        return CarryoverReadiness::None;
    };
    let Some(src_idx) = project.setups[..setup_idx]
        .iter()
        .position(|s| &s.id == src_id)
    else {
        return CarryoverReadiness::FailedUpstream(format!(
            "Carryover source setup '{src_id}' is missing or not ordered before '{}'",
            setup.name
        ));
    };

    // Ensure all transitive upstream carryover setups have clean toolpaths first
    let mut check_setups = vec![src_idx];
    check_setups.extend(collect_transitive_carryover_setups(project, src_idx));
    for up_idx in check_setups {
        let up_setup = &project.setups[up_idx];
        for (o_idx, op) in up_setup.operations.iter().enumerate() {
            if op.suppressed {
                continue;
            }
            let Some(h) = compute_operation_input_hash(project, up_idx, o_idx, cad_revision) else {
                return CarryoverReadiness::WaitingUpstreamOps;
            };
            if let Some(err) = cache.get_error(&op.id, h) {
                return CarryoverReadiness::FailedUpstream(format!(
                    "Carryover source setup '{}' op '{}' failed: {err}",
                    up_setup.name, op.name
                ));
            }
            if cache.get(&op.id, h).is_none() {
                return CarryoverReadiness::WaitingUpstreamOps;
            }
        }
    }

    let digest = compute_setup_carryover_digest(project, src_idx, cad_revision);
    if let Some(td) = carryover_cache.get(&(setup_idx, digest)) {
        return CarryoverReadiness::Ready(Arc::clone(td));
    }
    if let Some(err) = carryover_errors.get(&(setup_idx, digest)) {
        return CarryoverReadiness::FailedUpstream(err.clone());
    }
    CarryoverReadiness::NeedCompute { digest }
}

fn check_upstream_readiness(
    project: &ProjectDocument,
    cache: &ToolpathCache,
    setup_idx: usize,
    op_idx: usize,
    cad_revision: u64,
) -> UpstreamReadiness {
    let Some(setup) = project.setups.get(setup_idx) else {
        return UpstreamReadiness::Ready(Vec::new());
    };
    let upstream_indices = resolve_upstream_rest_op_indices(setup, op_idx);
    if upstream_indices.is_empty() {
        return UpstreamReadiness::Ready(Vec::new());
    }

    let mut inputs = Vec::with_capacity(upstream_indices.len());
    for u_idx in upstream_indices {
        let u_op = &setup.operations[u_idx];
        let Some(u_hash) = compute_operation_input_hash(project, setup_idx, u_idx, cad_revision)
        else {
            return UpstreamReadiness::Waiting;
        };
        if let Some(err_msg) = cache.get_error(&u_op.id, u_hash) {
            return UpstreamReadiness::FailedUpstream {
                op_name: u_op.name.clone(),
                message: err_msg.to_string(),
            };
        }
        let Some(u_tp) = cache.get(&u_op.id, u_hash) else {
            return UpstreamReadiness::Waiting;
        };
        let (tool_d, corner_r) = u_op
            .tool_id
            .as_ref()
            .and_then(|tid| project.tool(tid))
            .map(|t| {
                (
                    t.geometry.cutting_diameter_mm(),
                    t.geometry.corner_radius_mm(),
                )
            })
            .unwrap_or((0.0, 0.0));

        let (indexed_angle_deg, is_rotary_native) = if u_op.kind.is_rotary_native() {
            (None, true)
        } else {
            match u_op.rotary {
                OperationRotaryMode::None => (None, false),
                OperationRotaryMode::Indexed3Plus1 { angle_deg } => {
                    (Some(normalize_indexed_angle_deg(angle_deg)), false)
                }
                OperationRotaryMode::WrapCylinder { .. } => (None, true),
            }
        };

        inputs.push(UpstreamSweptInput {
            op_id: u_op.id.clone(),
            tool_diameter_mm: tool_d,
            corner_radius_mm: corner_r,
            toolpath: u_tp,
            indexed_angle_deg,
            is_rotary_native,
        });
    }
    UpstreamReadiness::Ready(inputs)
}

/// Non-blocking background toolpath generation coordinator backed by `rayon` (`01-architecture.md` & `05-operations.md`).
///
/// Supports topological wave execution for `RestMachiningMode::FromPrecedingOperations`: downstream
/// Rest operations wait in `pending_wave` until their upstream roughing toolpaths complete, then
/// automatically spawn with exact 2D swept coverage inputs.
type CarryoverMsg = (u64, usize, u64, Result<Arc<TriDexelStock>, String>);

/// Live batch progress snapshot for in-flight background toolpath generation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GenerationProgressSnapshot {
    /// Number of operations in the active generation batch that have finished.
    pub completed: usize,
    /// Total number of operations queued in the active generation batch.
    pub total: usize,
    /// Normalized completion fraction in `[0.0, 1.0]`.
    pub fraction: f32,
}

impl GenerationProgressSnapshot {
    /// Returns completion percentage rounded to the nearest integer in `0..=100`.
    #[must_use]
    pub fn percent_rounded(&self) -> u8 {
        (self.fraction.clamp(0.0, 1.0) * 100.0).round() as u8
    }
}

pub struct JobDispatcher {
    tx: Sender<(u64, DispatcherEvent)>,
    rx: Receiver<(u64, DispatcherEvent)>,
    carryover_tx: Sender<CarryoverMsg>,
    carryover_rx: Receiver<CarryoverMsg>,
    epoch: u64,
    in_flight: HashMap<String, (OperationInputHash, Arc<AtomicBool>)>,
    pending_wave: HashSet<String>,
    batch_ops: HashSet<String>,
    batch_started_at: Option<Instant>,
    batch_succeeded: usize,
    batch_failed: usize,
    carryover_cache: HashMap<(usize, u64), Arc<TriDexelStock>>,
    carryover_errors: HashMap<(usize, u64), String>,
    in_flight_carryover: HashSet<(usize, u64)>,
    active_cad: Option<(Arc<CadDocument>, u64)>,
    active_fixture_docs: Arc<BTreeMap<String, Arc<CadDocument>>>,
    active_machine_z_travel_mm: Option<f64>,
    wakeup_cb: Option<Arc<dyn Fn() + Send + Sync>>,
}

impl Default for JobDispatcher {
    fn default() -> Self {
        Self::new(None)
    }
}

impl JobDispatcher {
    pub fn new(wakeup_cb: Option<Arc<dyn Fn() + Send + Sync>>) -> Self {
        let (tx, rx) = crossbeam_channel::unbounded();
        let (carryover_tx, carryover_rx) = crossbeam_channel::unbounded();
        Self {
            tx,
            rx,
            carryover_tx,
            carryover_rx,
            epoch: 0,
            in_flight: HashMap::new(),
            pending_wave: HashSet::new(),
            batch_ops: HashSet::new(),
            batch_started_at: None,
            batch_succeeded: 0,
            batch_failed: 0,
            carryover_cache: HashMap::new(),
            carryover_errors: HashMap::new(),
            in_flight_carryover: HashSet::new(),
            active_cad: None,
            active_fixture_docs: Arc::new(BTreeMap::new()),
            active_machine_z_travel_mm: None,
            wakeup_cb,
        }
    }

    /// Updates the preloaded `ModelRole::Fixture` CAD documents and optional active machine Z travel
    /// envelope (`mm`) used by background toolpath generation for fixture-aware safe-rapid routing.
    pub fn set_workholding_context(
        &mut self,
        fixture_docs: BTreeMap<String, Arc<CadDocument>>,
        machine_z_travel_mm: Option<f64>,
    ) {
        self.active_fixture_docs = Arc::new(fixture_docs);
        self.active_machine_z_travel_mm = machine_z_travel_mm;
    }

    pub fn is_busy(&self) -> bool {
        !self.in_flight.is_empty()
            || !self.pending_wave.is_empty()
            || !self.in_flight_carryover.is_empty()
    }

    pub fn in_flight_count(&self) -> usize {
        self.in_flight.len() + self.pending_wave.len()
    }

    /// Returns wall-clock elapsed duration for the active generation batch, or `None` when idle.
    #[must_use]
    pub fn batch_elapsed(&self) -> Option<Duration> {
        self.batch_started_at.map(|t0| t0.elapsed())
    }

    /// Returns live batch completion progress while toolpath generation is running, or `None` when idle.
    #[must_use]
    pub fn batch_progress(&self) -> Option<GenerationProgressSnapshot> {
        if !self.is_busy() {
            return None;
        }
        let remaining = self.in_flight_count();
        let total = self.batch_ops.len().max(remaining);
        let completed = total.saturating_sub(remaining);
        let fraction = if total == 0 {
            0.0
        } else {
            (completed as f32 / total as f32).clamp(0.0, 1.0)
        };
        Some(GenerationProgressSnapshot {
            completed,
            total,
            fraction,
        })
    }

    pub fn in_flight_ids(&self) -> HashSet<String> {
        let mut ids: HashSet<String> = self.in_flight.keys().cloned().collect();
        ids.extend(self.pending_wave.iter().cloned());
        ids
    }

    /// Signals cancellation for all in-flight and queued wave jobs (`Esc`).
    pub fn cancel_all(&mut self) -> usize {
        let count = self.in_flight.len() + self.pending_wave.len();
        self.pending_wave.clear();
        self.batch_ops.clear();
        self.batch_started_at = None;
        self.batch_succeeded = 0;
        self.batch_failed = 0;
        self.in_flight_carryover.clear();
        for (_, (_, token)) in self.in_flight.drain() {
            token.store(true, Ordering::Relaxed);
        }
        count
    }

    /// Cancels all in-flight and queued wave jobs, drops the active CAD reference, advances the
    /// dispatcher epoch so stale worker completions are ignored, and drains pending channel events.
    pub fn reset(&mut self) {
        self.cancel_all();
        self.carryover_cache.clear();
        self.carryover_errors.clear();
        self.active_cad = None;
        self.active_fixture_docs = Arc::new(BTreeMap::new());
        self.active_machine_z_travel_mm = None;
        self.epoch = self.epoch.wrapping_add(1);
        while self.rx.try_recv().is_ok() {}
        while self.carryover_rx.try_recv().is_ok() {}
    }

    /// Cancels any in-flight job whose `OperationInputHash` no longer matches `project`
    /// (including downstream Rest operations when an upstream operation is edited!).
    pub fn cancel_stale_jobs(&mut self, project: &ProjectDocument, cad_revision: u64) {
        let mut current_hashes = HashMap::new();
        for s_idx in 0..project.setups.len() {
            for o_idx in 0..project.setups[s_idx].operations.len() {
                if let Some(h) = compute_operation_input_hash(project, s_idx, o_idx, cad_revision) {
                    current_hashes.insert(project.setups[s_idx].operations[o_idx].id.clone(), h);
                }
            }
        }

        let batch_ops = &mut self.batch_ops;
        self.in_flight.retain(|op_id, (job_hash, token)| {
            let still_matches = current_hashes.get(op_id) == Some(job_hash);
            if !still_matches {
                token.store(true, Ordering::Relaxed);
                batch_ops.remove(op_id);
            }
            still_matches
        });
        self.pending_wave.retain(|op_id| {
            let keep = current_hashes.contains_key(op_id);
            if !keep {
                batch_ops.remove(op_id);
            }
            keep
        });
        if !self.is_busy() {
            self.batch_ops.clear();
            self.batch_started_at = None;
            self.batch_succeeded = 0;
            self.batch_failed = 0;
        }
    }

    /// Queues dirty (or explicitly targeted) non-suppressed operations onto the background `rayon` pool,
    /// deferring downstream `FromPrecedingOperations` Rest tasks into `pending_wave` when upstream
    /// roughing operations are still generating.
    pub fn dispatch_operations(
        &mut self,
        project: &mut ProjectDocument,
        cad_doc: Arc<CadDocument>,
        cad_revision: u64,
        cache: &ToolpathCache,
        only_setup_idx: Option<usize>,
        only_op_id: Option<&str>,
    ) -> usize {
        if !self.is_busy() {
            self.batch_ops.clear();
            self.batch_started_at = None;
            self.batch_succeeded = 0;
            self.batch_failed = 0;
        }
        self.active_cad = Some((Arc::clone(&cad_doc), cad_revision));
        let shared_sketches = Arc::new(project.sketches.clone());
        let shared_avoid_regions = Arc::new(project.avoid_regions.clone());
        let mut tasks = Vec::new();
        let mut queued_count = 0usize;

        // If targeting a specific setup (or operation) whose Setup carries over stock from an
        // earlier Setup, include its transitive upstream carryover Setups so they generate first!
        let mut forced_carryover_setups: HashSet<usize> = HashSet::new();
        if let Some(target_s) = only_setup_idx {
            forced_carryover_setups.extend(collect_transitive_carryover_setups(project, target_s));
        } else if let Some(target_id) = only_op_id
            && let Some(target_s) = project
                .setups
                .iter()
                .position(|s| s.operations.iter().any(|o| o.id == target_id))
        {
            forced_carryover_setups.extend(collect_transitive_carryover_setups(project, target_s));
        }

        for s_idx in 0..project.setups.len() {
            let is_forced_carryover_setup = forced_carryover_setups.contains(&s_idx);
            if let Some(target_s) = only_setup_idx
                && s_idx != target_s
                && !is_forced_carryover_setup
            {
                continue;
            }
            // If `only_op_id` targets a downstream Rest operation whose upstream dependencies are Dirty,
            // automatically ensure those upstream dependencies are queued first!
            let forced_upstream: HashSet<usize> = if let Some(target_id) = only_op_id {
                if let Some(t_idx) = project.setups[s_idx]
                    .operations
                    .iter()
                    .position(|o| o.id == target_id)
                {
                    collect_transitive_upstream_indices(&project.setups[s_idx], t_idx)
                } else {
                    HashSet::new()
                }
            } else {
                HashSet::new()
            };

            let mut seen_tool = false;
            let op_count = project.setups[s_idx].operations.len();
            for o_idx in 0..op_count {
                let is_suppressed = project.setups[s_idx].operations[o_idx].suppressed;
                if is_suppressed {
                    continue;
                }
                let is_first_tool = !seen_tool;
                if project.setups[s_idx].operations[o_idx].tool_id.is_some() {
                    seen_tool = true;
                }

                let op_id = project.setups[s_idx].operations[o_idx].id.clone();
                if !is_forced_carryover_setup
                    && let Some(target_id) = only_op_id
                    && op_id != target_id
                    && !forced_upstream.contains(&o_idx)
                {
                    continue;
                }

                let Some(input_hash) =
                    compute_operation_input_hash(project, s_idx, o_idx, cad_revision)
                else {
                    continue;
                };

                // Skip if already cached Clean (unless explicitly forced via `only_op_id == Some(op_id)`)
                let explicitly_forced = only_op_id == Some(op_id.as_str());
                if !explicitly_forced && cache.get(&op_id, input_hash).is_some() {
                    project.setups[s_idx].operations[o_idx].runtime_status = OperationStatus::Clean;
                    continue;
                }

                // Cancel any previous in-flight run for `op_id`
                if let Some((_, old_tok)) = self.in_flight.remove(&op_id) {
                    old_tok.store(true, Ordering::Relaxed);
                }

                let carryover_state = check_carryover_readiness(
                    project,
                    cache,
                    s_idx,
                    cad_revision,
                    &self.carryover_cache,
                    &self.carryover_errors,
                );
                let carryover_stock = match carryover_state {
                    CarryoverReadiness::None => None,
                    CarryoverReadiness::Ready(td) => Some(td),
                    CarryoverReadiness::NeedCompute { digest } => {
                        self.spawn_carryover_compute(
                            project,
                            &cad_doc,
                            cad_revision,
                            cache,
                            s_idx,
                            digest,
                        );
                        self.batch_ops.insert(op_id.clone());
                        self.pending_wave.insert(op_id);
                        project.setups[s_idx].operations[o_idx].runtime_status =
                            OperationStatus::Generating;
                        queued_count += 1;
                        continue;
                    }
                    CarryoverReadiness::WaitingUpstreamOps
                    | CarryoverReadiness::FailedUpstream(_) => {
                        self.batch_ops.insert(op_id.clone());
                        self.pending_wave.insert(op_id);
                        project.setups[s_idx].operations[o_idx].runtime_status =
                            OperationStatus::Generating;
                        queued_count += 1;
                        continue;
                    }
                };

                match check_upstream_readiness(project, cache, s_idx, o_idx, cad_revision) {
                    UpstreamReadiness::Ready(upstream_rest) => {
                        let effective_carryover =
                            if op_needs_in_setup_rotary_carryover(&project.setups[s_idx], o_idx) {
                                let in_digest = compute_in_setup_carryover_digest(
                                    project,
                                    s_idx,
                                    o_idx,
                                    cad_revision,
                                );
                                if let Some(td) = self.carryover_cache.get(&(s_idx, in_digest)) {
                                    Some(Arc::clone(td))
                                } else if self.carryover_errors.contains_key(&(s_idx, in_digest)) {
                                    self.batch_ops.insert(op_id.clone());
                                    self.pending_wave.insert(op_id);
                                    project.setups[s_idx].operations[o_idx].runtime_status =
                                        OperationStatus::Generating;
                                    queued_count += 1;
                                    continue;
                                } else {
                                    let upstream_indices = resolve_upstream_rest_op_indices(
                                        &project.setups[s_idx],
                                        o_idx,
                                    );
                                    self.spawn_in_setup_carryover_compute(
                                        project,
                                        &cad_doc,
                                        cad_revision,
                                        cache,
                                        s_idx,
                                        upstream_indices,
                                        in_digest,
                                    );
                                    self.batch_ops.insert(op_id.clone());
                                    self.pending_wave.insert(op_id);
                                    project.setups[s_idx].operations[o_idx].runtime_status =
                                        OperationStatus::Generating;
                                    queued_count += 1;
                                    continue;
                                }
                            } else {
                                carryover_stock
                            };

                        self.pending_wave.remove(&op_id);
                        let cancel_token = Arc::new(AtomicBool::new(false));
                        self.batch_ops.insert(op_id.clone());
                        self.in_flight
                            .insert(op_id.clone(), (input_hash, Arc::clone(&cancel_token)));
                        project.setups[s_idx].operations[o_idx].runtime_status =
                            OperationStatus::Generating;

                        let tool = project.setups[s_idx].operations[o_idx]
                            .tool_id
                            .as_ref()
                            .and_then(|tid| project.tool(tid).cloned());
                        let clearing_tool = project.setups[s_idx].operations[o_idx]
                            .clearing_tool_id()
                            .and_then(|tid| project.tool(tid).cloned());

                        let machine_z = self.active_machine_z_travel_mm.or_else(|| {
                            resolve_machine_definition(project.effective_machine_id(), None)
                                .ok()
                                .map(|m| m.travel_mm.z_mm)
                        });
                        tasks.push(QueuedOpTask {
                            setup: project.setups[s_idx].clone(),
                            sketches: Arc::clone(&shared_sketches),
                            shared_avoid_regions: Arc::clone(&shared_avoid_regions),
                            fixture_docs: Arc::clone(&self.active_fixture_docs),
                            machine_z_travel_mm: machine_z,
                            op: project.setups[s_idx].operations[o_idx].clone(),
                            tool,
                            clearing_tool,
                            display_unit: project.display_units,
                            is_first_tool_in_setup: is_first_tool,
                            upstream_rest,
                            carryover_stock: effective_carryover,
                            input_hash,
                            cancel_token,
                        });
                        queued_count += 1;
                    }
                    UpstreamReadiness::Waiting | UpstreamReadiness::FailedUpstream { .. } => {
                        self.batch_ops.insert(op_id.clone());
                        self.pending_wave.insert(op_id);
                        project.setups[s_idx].operations[o_idx].runtime_status =
                            OperationStatus::Generating;
                        queued_count += 1;
                    }
                }
            }
        }

        if queued_count > 0 && self.batch_started_at.is_none() {
            self.batch_started_at = Some(Instant::now());
        }
        self.spawn_rayon_tasks(&cad_doc, tasks);
        queued_count
    }

    fn spawn_carryover_compute(
        &mut self,
        project: &ProjectDocument,
        cad_doc: &Arc<CadDocument>,
        cad_revision: u64,
        cache: &ToolpathCache,
        setup_idx: usize,
        digest: u64,
    ) {
        if !self.in_flight_carryover.insert((setup_idx, digest)) {
            return;
        }
        let epoch = self.epoch;
        let tx = self.carryover_tx.clone();
        let wakeup = self.wakeup_cb.clone();
        let proj_owned = project.clone();
        let cache_owned = cache.clone();
        let cad_owned = Arc::clone(cad_doc);

        rayon::spawn(move || {
            let mut req = SetupSimRequest::new(
                &proj_owned,
                setup_idx,
                &cad_owned,
                cad_revision,
                &cache_owned,
            );
            req.backend_choice = SimComputeBackendChoice::Auto;
            let res = compute_setup_carryover_stock(&req).and_then(|opt| {
                opt.map(|c| Arc::new(c.stock))
                    .ok_or_else(|| "carryover stock not configured".to_string())
            });
            let _ = tx.send((epoch, setup_idx, digest, res));
            if let Some(cb) = &wakeup {
                cb();
            }
        });
    }

    #[allow(clippy::too_many_arguments)]
    fn spawn_in_setup_carryover_compute(
        &mut self,
        project: &ProjectDocument,
        cad_doc: &Arc<CadDocument>,
        cad_revision: u64,
        cache: &ToolpathCache,
        setup_idx: usize,
        upstream_indices: Vec<usize>,
        digest: u64,
    ) {
        if !self.in_flight_carryover.insert((setup_idx, digest)) {
            return;
        }
        let epoch = self.epoch;
        let tx = self.carryover_tx.clone();
        let wakeup = self.wakeup_cb.clone();
        let proj_owned = project.clone();
        let cache_owned = cache.clone();
        let cad_owned = Arc::clone(cad_doc);

        rayon::spawn(move || {
            let mut req = SetupSimRequest::new(
                &proj_owned,
                setup_idx,
                &cad_owned,
                cad_revision,
                &cache_owned,
            );
            req.backend_choice = SimComputeBackendChoice::Auto;
            let res = compute_in_setup_carryover_stock(&req, &upstream_indices).map(Arc::new);
            let _ = tx.send((epoch, setup_idx, digest, res));
            if let Some(cb) = &wakeup {
                cb();
            }
        });
    }

    fn spawn_rayon_tasks(&self, cad_doc: &Arc<CadDocument>, tasks: Vec<QueuedOpTask>) {
        let epoch = self.epoch;
        for task in tasks {
            let tx = self.tx.clone();
            let wakeup = self.wakeup_cb.clone();
            let cad = Arc::clone(cad_doc);

            rayon::spawn(move || {
                let op_id = task.op.id.clone();
                let started = Instant::now();
                let _ = tx.send((
                    epoch,
                    DispatcherEvent::Started {
                        op_id: op_id.clone(),
                        input_hash: task.input_hash,
                    },
                ));

                if task.cancel_token.load(Ordering::Relaxed) {
                    let _ = tx.send((
                        epoch,
                        DispatcherEvent::Cancelled {
                            op_id,
                            input_hash: task.input_hash,
                            elapsed: started.elapsed(),
                        },
                    ));
                    if let Some(cb) = &wakeup {
                        cb();
                    }
                    return;
                }

                let res = execute_single_task(&cad, &task);
                let elapsed = started.elapsed();
                match res {
                    Ok(tp) => {
                        let _ = tx.send((
                            epoch,
                            DispatcherEvent::Completed {
                                op_id,
                                input_hash: task.input_hash,
                                toolpath: Arc::new(tp),
                                elapsed,
                            },
                        ));
                    }
                    Err(CamError::Cancelled) => {
                        let _ = tx.send((
                            epoch,
                            DispatcherEvent::Cancelled {
                                op_id,
                                input_hash: task.input_hash,
                                elapsed,
                            },
                        ));
                    }
                    Err(err) => {
                        let _ = tx.send((
                            epoch,
                            DispatcherEvent::Failed {
                                op_id,
                                input_hash: task.input_hash,
                                message: err.to_string(),
                                elapsed,
                            },
                        ));
                    }
                }

                if let Some(cb) = &wakeup {
                    cb();
                }
            });
        }
    }

    /// Drains all pending [`DispatcherEvent`]s, updating `cache`, advancing any ready downstream
    /// `pending_wave` Rest Machining operations, and reconciling `project` statuses.
    pub fn poll_events(
        &mut self,
        project: &mut ProjectDocument,
        cache: &mut ToolpathCache,
        cad_revision: u64,
    ) -> Vec<String> {
        let mut logs = Vec::new();
        while let Ok((ev_epoch, ev)) = self.rx.try_recv() {
            if ev_epoch != self.epoch {
                continue;
            }
            match ev {
                DispatcherEvent::Started { .. } => {}
                DispatcherEvent::Completed {
                    op_id,
                    input_hash,
                    toolpath,
                    elapsed,
                } => {
                    let matches_inflight = self
                        .in_flight
                        .get(&op_id)
                        .is_some_and(|(h, _)| *h == input_hash);
                    if matches_inflight {
                        self.in_flight.remove(&op_id);
                    }
                    self.batch_succeeded += 1;
                    let lift_suffix = if toolpath.rapid_lifts_inserted > 0 {
                        format!(
                            ", {} fixture rapid lift(s) up to Z={:+.1} mm",
                            toolpath.rapid_lifts_inserted, toolpath.max_lifted_z_wcs
                        )
                    } else {
                        String::new()
                    };
                    let wall = format_exec_duration(elapsed);
                    logs.push(format!(
                        "Generated '{}' in {wall} ({} passes, est. cycle {:.1}s, {} G2/G3 arcs{})",
                        op_id,
                        toolpath.stats.pass_count,
                        toolpath.stats.total_est_time_sec,
                        toolpath.stats.arc_move_count,
                        lift_suffix,
                    ));
                    cache.insert(op_id, input_hash, toolpath);
                }
                DispatcherEvent::Cancelled {
                    op_id,
                    input_hash,
                    elapsed,
                } => {
                    let matches_inflight = self
                        .in_flight
                        .get(&op_id)
                        .is_some_and(|(h, _)| *h == input_hash);
                    if matches_inflight {
                        self.in_flight.remove(&op_id);
                    }
                    let wall = format_exec_duration(elapsed);
                    logs.push(format!("Cancelled generation for '{op_id}' after {wall}"));
                }
                DispatcherEvent::Failed {
                    op_id,
                    input_hash,
                    message,
                    elapsed,
                } => {
                    let matches_inflight = self
                        .in_flight
                        .get(&op_id)
                        .is_some_and(|(h, _)| *h == input_hash);
                    if matches_inflight {
                        self.in_flight.remove(&op_id);
                    }
                    self.batch_failed += 1;
                    let wall = format_exec_duration(elapsed);
                    logs.push(format!("Failed '{op_id}' after {wall}: {message}"));
                    cache.record_error(op_id, input_hash, message);
                }
            }
        }

        // Drain background carryover Tri-Dexel computation results
        while let Ok((ev_epoch, s_idx, digest, res)) = self.carryover_rx.try_recv() {
            if ev_epoch != self.epoch {
                continue;
            }
            self.in_flight_carryover.remove(&(s_idx, digest));
            match res {
                Ok(td) => {
                    self.carryover_errors.remove(&(s_idx, digest));
                    self.carryover_cache.insert((s_idx, digest), td);
                }
                Err(msg) => {
                    logs.push(format!(
                        "Error computing carryover stock for setup #{s_idx}: {msg}"
                    ));
                    self.carryover_errors.insert((s_idx, digest), msg);
                }
            }
        }

        // Advance topological DAG waves for any `pending_wave` operations whose upstream deps finished!
        if !self.pending_wave.is_empty()
            && let Some((cad_doc, saved_rev)) = self.active_cad.clone()
            && saved_rev == cad_revision
        {
            let shared_sketches = Arc::new(project.sketches.clone());
            let shared_avoid_regions = Arc::new(project.avoid_regions.clone());
            let mut next_tasks = Vec::new();
            let mut to_spawn_carryover: Vec<(usize, u64)> = Vec::new();
            let mut to_spawn_in_setup_carryover: Vec<(usize, Vec<usize>, u64)> = Vec::new();
            for s_idx in 0..project.setups.len() {
                let mut seen_tool = false;
                for o_idx in 0..project.setups[s_idx].operations.len() {
                    if project.setups[s_idx].operations[o_idx].suppressed {
                        continue;
                    }
                    let is_first_tool = !seen_tool;
                    if project.setups[s_idx].operations[o_idx].tool_id.is_some() {
                        seen_tool = true;
                    }
                    let op_id = project.setups[s_idx].operations[o_idx].id.clone();
                    if !self.pending_wave.contains(&op_id) {
                        continue;
                    }
                    let Some(input_hash) =
                        compute_operation_input_hash(project, s_idx, o_idx, cad_revision)
                    else {
                        self.pending_wave.remove(&op_id);
                        continue;
                    };

                    let carryover_stock = match check_carryover_readiness(
                        project,
                        cache,
                        s_idx,
                        cad_revision,
                        &self.carryover_cache,
                        &self.carryover_errors,
                    ) {
                        CarryoverReadiness::None => None,
                        CarryoverReadiness::Ready(td) => Some(td),
                        CarryoverReadiness::NeedCompute { digest } => {
                            to_spawn_carryover.push((s_idx, digest));
                            continue;
                        }
                        CarryoverReadiness::WaitingUpstreamOps => {
                            if self.in_flight.is_empty()
                                && self.in_flight_carryover.is_empty()
                                && next_tasks.is_empty()
                                && to_spawn_carryover.is_empty()
                                && to_spawn_in_setup_carryover.is_empty()
                            {
                                self.pending_wave.remove(&op_id);
                            }
                            continue;
                        }
                        CarryoverReadiness::FailedUpstream(err) => {
                            self.pending_wave.remove(&op_id);
                            self.batch_failed += 1;
                            logs.push(format!("Error generating '{op_id}': {err}"));
                            cache.record_error(op_id, input_hash, err);
                            continue;
                        }
                    };

                    match check_upstream_readiness(project, cache, s_idx, o_idx, cad_revision) {
                        UpstreamReadiness::Ready(upstream_rest) => {
                            let effective_carryover = if op_needs_in_setup_rotary_carryover(
                                &project.setups[s_idx],
                                o_idx,
                            ) {
                                let in_digest = compute_in_setup_carryover_digest(
                                    project,
                                    s_idx,
                                    o_idx,
                                    cad_revision,
                                );
                                if let Some(td) = self.carryover_cache.get(&(s_idx, in_digest)) {
                                    Some(Arc::clone(td))
                                } else if let Some(err) =
                                    self.carryover_errors.get(&(s_idx, in_digest)).cloned()
                                {
                                    self.pending_wave.remove(&op_id);
                                    self.batch_failed += 1;
                                    logs.push(format!("Error generating '{op_id}': {err}"));
                                    cache.record_error(op_id, input_hash, err);
                                    continue;
                                } else {
                                    let upstream_indices = resolve_upstream_rest_op_indices(
                                        &project.setups[s_idx],
                                        o_idx,
                                    );
                                    to_spawn_in_setup_carryover.push((
                                        s_idx,
                                        upstream_indices,
                                        in_digest,
                                    ));
                                    continue;
                                }
                            } else {
                                carryover_stock
                            };

                            self.pending_wave.remove(&op_id);
                            let cancel_token = Arc::new(AtomicBool::new(false));
                            self.in_flight
                                .insert(op_id.clone(), (input_hash, Arc::clone(&cancel_token)));
                            let tool = project.setups[s_idx].operations[o_idx]
                                .tool_id
                                .as_ref()
                                .and_then(|tid| project.tool(tid).cloned());
                            let clearing_tool = project.setups[s_idx].operations[o_idx]
                                .clearing_tool_id()
                                .and_then(|tid| project.tool(tid).cloned());
                            let machine_z = self.active_machine_z_travel_mm.or_else(|| {
                                resolve_machine_definition(project.effective_machine_id(), None)
                                    .ok()
                                    .map(|m| m.travel_mm.z_mm)
                            });
                            next_tasks.push(QueuedOpTask {
                                setup: project.setups[s_idx].clone(),
                                sketches: Arc::clone(&shared_sketches),
                                shared_avoid_regions: Arc::clone(&shared_avoid_regions),
                                fixture_docs: Arc::clone(&self.active_fixture_docs),
                                machine_z_travel_mm: machine_z,
                                op: project.setups[s_idx].operations[o_idx].clone(),
                                tool,
                                clearing_tool,
                                display_unit: project.display_units,
                                is_first_tool_in_setup: is_first_tool,
                                upstream_rest,
                                carryover_stock: effective_carryover,
                                input_hash,
                                cancel_token,
                            });
                        }
                        UpstreamReadiness::FailedUpstream { op_name, message } => {
                            self.pending_wave.remove(&op_id);
                            self.batch_failed += 1;
                            let err = format!(
                                "Blocked by failed upstream Rest Machining operation '{op_name}': {message}"
                            );
                            logs.push(format!("Error generating '{op_id}': {err}"));
                            cache.record_error(op_id, input_hash, err);
                        }
                        UpstreamReadiness::Waiting => {
                            // If nothing is left in-flight at all, an upstream dependency was cancelled or missing
                            if self.in_flight.is_empty()
                                && self.in_flight_carryover.is_empty()
                                && next_tasks.is_empty()
                                && to_spawn_carryover.is_empty()
                                && to_spawn_in_setup_carryover.is_empty()
                            {
                                self.pending_wave.remove(&op_id);
                            }
                        }
                    }
                }
            }
            for (s_idx, digest) in to_spawn_carryover {
                self.spawn_carryover_compute(project, &cad_doc, cad_revision, cache, s_idx, digest);
            }
            for (s_idx, upstream_indices, digest) in to_spawn_in_setup_carryover {
                self.spawn_in_setup_carryover_compute(
                    project,
                    &cad_doc,
                    cad_revision,
                    cache,
                    s_idx,
                    upstream_indices,
                    digest,
                );
            }
            if !next_tasks.is_empty() {
                self.spawn_rayon_tasks(&cad_doc, next_tasks);
            }
        }

        if !self.is_busy() {
            let batch_total = self.batch_ops.len();
            if let Some(batch_start) = self.batch_started_at.take()
                && batch_total > 1
                && (self.batch_succeeded > 0 || self.batch_failed > 0)
            {
                let wall = format_exec_duration(batch_start.elapsed());
                if self.batch_failed == 0 {
                    logs.push(format!(
                        "Toolpath generation complete ({} operations) in {wall}",
                        self.batch_succeeded
                    ));
                } else {
                    logs.push(format!(
                        "Toolpath generation finished ({} succeeded, {} failed) in {wall}",
                        self.batch_succeeded, self.batch_failed
                    ));
                }
            }
            self.batch_ops.clear();
            self.batch_succeeded = 0;
            self.batch_failed = 0;
        }
        let inflight_set = self.in_flight_ids();
        cache.reconcile_project_statuses(project, cad_revision, &inflight_set);
        logs
    }
}

fn collect_transitive_upstream_indices(setup: &Setup, target_idx: usize) -> HashSet<usize> {
    let mut visited = HashSet::new();
    let mut stack = resolve_upstream_rest_op_indices(setup, target_idx);
    while let Some(u) = stack.pop() {
        if visited.insert(u) {
            stack.extend(resolve_upstream_rest_op_indices(setup, u));
        }
    }
    visited
}

fn execute_single_task(
    cad: &CadDocument,
    task: &QueuedOpTask,
) -> Result<GeneratedToolpath, CamError> {
    let resolved_setup =
        task.setup
            .resolve_with_sketches(cad, &task.sketches, task.display_unit)?;
    let tool = task.tool.as_ref().ok_or_else(|| CamError::MissingTool {
        op_name: task.op.name.clone(),
    })?;
    let needs_fixtures = task.setup.fixtures.iter().any(|f| f.enabled)
        && (task.setup.auto_avoid_fixtures_on_rapids
            || task
                .op
                .avoid
                .references_fixtures(&task.shared_avoid_regions));
    let resolved_fixtures = if needs_fixtures {
        resolve_setup_fixtures(&task.setup, &resolved_setup, task.display_unit, |id| {
            task.fixture_docs.get(id).map(Arc::as_ref)
        })?
    } else {
        Vec::new()
    };
    generate_operation_toolpath_with_fixtures(
        &OperationGenRequest {
            doc: cad,
            sketches: &task.sketches,
            setup: &resolved_setup,
            op: &task.op,
            tool,
            clearing_tool: task.clearing_tool.as_ref(),
            display_unit: task.display_unit,
            is_first_tool_in_setup: task.is_first_tool_in_setup,
            upstream_rest: &task.upstream_rest,
            carryover_stock: task.carryover_stock.clone(),
            fixtures: &resolved_fixtures,
            shared_avoid_regions: &task.shared_avoid_regions,
            cancel: &task.cancel_token,
        },
        &resolved_fixtures,
        task.setup.auto_avoid_fixtures_on_rapids,
        task.machine_z_travel_mm,
    )
}

struct WaveTaskOutcome {
    coord: (usize, usize),
    scope: String,
    op_id: String,
    hash: OperationInputHash,
    res: Result<GeneratedToolpath, CamError>,
}

/// Summary of a synchronous batch generation run (used by headless `chip-cli` and integration tests).
#[derive(Debug, Clone, Default)]
pub struct BatchGenerationReport {
    pub generated: Vec<(String, String, Arc<GeneratedToolpath>)>,
    pub errors: Vec<(String, String)>,
}

impl BatchGenerationReport {
    pub fn is_ok(&self) -> bool {
        self.errors.is_empty()
    }
}

/// Generates all non-suppressed operations across `project` in topological DAG waves,
/// executing independent operations within each wave concurrently on `rayon`.
pub fn generate_project_blocking(
    project: &mut ProjectDocument,
    cad_doc: &CadDocument,
    cache: &mut ToolpathCache,
    cad_revision: u64,
) -> BatchGenerationReport {
    generate_project_blocking_in_dir(project, cad_doc, None, cache, cad_revision)
}

/// Generates all non-suppressed operations across `project` in topological DAG waves, loading any
/// `ImportedModel` fixture CAD documents and custom machine travel limits from `project_dir`.
pub fn generate_project_blocking_in_dir(
    project: &mut ProjectDocument,
    cad_doc: &CadDocument,
    project_dir: Option<&Path>,
    cache: &mut ToolpathCache,
    cad_revision: u64,
) -> BatchGenerationReport {
    project.reevaluate_sketches(cad_doc, None);
    let mut report = BatchGenerationReport::default();
    let total_ops: usize = project
        .setups
        .iter()
        .map(|s| s.operations.iter().filter(|o| !o.suppressed).count())
        .sum();
    if total_ops == 0 {
        cache.reconcile_project_statuses(project, cad_revision, &HashSet::new());
        return report;
    }

    let needs_imported_fixtures = project.setups.iter().any(|s| {
        let setup_uses_fixtures = s.auto_avoid_fixtures_on_rapids
            || s.operations
                .iter()
                .any(|o| !o.suppressed && o.avoid.references_fixtures(&project.avoid_regions));
        setup_uses_fixtures
            && s.fixtures
                .iter()
                .any(|f| f.enabled && matches!(f.kind, FixtureKind::ImportedModel { .. }))
    });
    let shared_fixture_docs: Arc<BTreeMap<String, Arc<CadDocument>>> = if needs_imported_fixtures {
        Arc::new(
            project
                .load_fixture_cad_documents(project_dir)
                .into_iter()
                .map(|(k, v)| (k, Arc::new(v)))
                .collect(),
        )
    } else {
        Arc::new(BTreeMap::new())
    };
    let machine_z_travel_mm =
        resolve_machine_definition(project.effective_machine_id(), project_dir)
            .ok()
            .map(|m| m.travel_mm.z_mm);

    let mut processed: HashSet<(usize, usize)> = HashSet::new();
    let shared_sketches = Arc::new(project.sketches.clone());
    let shared_avoid_regions = Arc::new(project.avoid_regions.clone());
    let mut carryover_cache: HashMap<(usize, u64), Arc<TriDexelStock>> = HashMap::new();
    let mut carryover_errors: HashMap<(usize, u64), String> = HashMap::new();

    for _wave in 0..=(total_ops + project.setups.len()) {
        if processed.len() >= total_ops {
            break;
        }
        let mut wave_tasks: Vec<((usize, usize), String, QueuedOpTask)> = Vec::new();
        let mut blocked_this_pass = false;

        for s_idx in 0..project.setups.len() {
            let setup_name = project.setups[s_idx].name.clone();
            let mut seen_tool = false;
            for o_idx in 0..project.setups[s_idx].operations.len() {
                let op = &project.setups[s_idx].operations[o_idx];
                if op.suppressed {
                    continue;
                }
                let is_first = !seen_tool;
                if op.tool_id.is_some() {
                    seen_tool = true;
                }
                if processed.contains(&(s_idx, o_idx)) {
                    continue;
                }
                let Some(hash) = compute_operation_input_hash(project, s_idx, o_idx, cad_revision)
                else {
                    processed.insert((s_idx, o_idx));
                    continue;
                };

                // Reuse resident cached Clean toolpath if already up-to-date
                if let Some(cached_tp) = cache.get(&op.id, hash) {
                    processed.insert((s_idx, o_idx));
                    report.generated.push((
                        format!("Setup '{}' -> Op '{}'", setup_name, op.name),
                        op.id.clone(),
                        cached_tp,
                    ));
                    blocked_this_pass = true;
                    continue;
                }

                let scope = format!("Setup '{}' -> Op '{}'", setup_name, op.name);
                let carryover_stock = match check_carryover_readiness(
                    project,
                    cache,
                    s_idx,
                    cad_revision,
                    &carryover_cache,
                    &carryover_errors,
                ) {
                    CarryoverReadiness::None => None,
                    CarryoverReadiness::Ready(td) => Some(td),
                    CarryoverReadiness::NeedCompute { digest } => {
                        let mut sim_req =
                            SetupSimRequest::new(project, s_idx, cad_doc, cad_revision, cache);
                        sim_req.project_dir = project_dir;
                        sim_req.kinematics = resolve_setup_kinematics(project, s_idx, project_dir);
                        sim_req.backend_choice = SimComputeBackendChoice::Auto;
                        match compute_setup_carryover_stock(&sim_req) {
                            Ok(Some(res)) => {
                                let arc_td = Arc::new(res.stock);
                                carryover_cache.insert((s_idx, digest), Arc::clone(&arc_td));
                                Some(arc_td)
                            }
                            Ok(None) => None,
                            Err(err) => {
                                carryover_errors.insert((s_idx, digest), err.clone());
                                processed.insert((s_idx, o_idx));
                                cache.record_error(op.id.clone(), hash, err.clone());
                                report.errors.push((scope, err));
                                blocked_this_pass = true;
                                continue;
                            }
                        }
                    }
                    CarryoverReadiness::WaitingUpstreamOps => continue,
                    CarryoverReadiness::FailedUpstream(err) => {
                        processed.insert((s_idx, o_idx));
                        cache.record_error(op.id.clone(), hash, err.clone());
                        report.errors.push((scope, err));
                        blocked_this_pass = true;
                        continue;
                    }
                };

                match check_upstream_readiness(project, cache, s_idx, o_idx, cad_revision) {
                    UpstreamReadiness::Ready(upstream_rest) => {
                        let effective_carryover =
                            if op_needs_in_setup_rotary_carryover(&project.setups[s_idx], o_idx) {
                                let in_digest = compute_in_setup_carryover_digest(
                                    project,
                                    s_idx,
                                    o_idx,
                                    cad_revision,
                                );
                                if let Some(td) = carryover_cache.get(&(s_idx, in_digest)) {
                                    Some(Arc::clone(td))
                                } else if let Some(err) =
                                    carryover_errors.get(&(s_idx, in_digest)).cloned()
                                {
                                    processed.insert((s_idx, o_idx));
                                    cache.record_error(op.id.clone(), hash, err.clone());
                                    report.errors.push((scope, err));
                                    blocked_this_pass = true;
                                    continue;
                                } else {
                                    let upstream_indices = resolve_upstream_rest_op_indices(
                                        &project.setups[s_idx],
                                        o_idx,
                                    );
                                    let mut sim_req = SetupSimRequest::new(
                                        project,
                                        s_idx,
                                        cad_doc,
                                        cad_revision,
                                        cache,
                                    );
                                    sim_req.project_dir = project_dir;
                                    sim_req.kinematics =
                                        resolve_setup_kinematics(project, s_idx, project_dir);
                                    sim_req.backend_choice = SimComputeBackendChoice::Auto;
                                    match compute_in_setup_carryover_stock(
                                        &sim_req,
                                        &upstream_indices,
                                    ) {
                                        Ok(td) => {
                                            let arc_td = Arc::new(td);
                                            carryover_cache
                                                .insert((s_idx, in_digest), Arc::clone(&arc_td));
                                            Some(arc_td)
                                        }
                                        Err(err) => {
                                            carryover_errors
                                                .insert((s_idx, in_digest), err.clone());
                                            processed.insert((s_idx, o_idx));
                                            cache.record_error(op.id.clone(), hash, err.clone());
                                            report.errors.push((scope, err));
                                            blocked_this_pass = true;
                                            continue;
                                        }
                                    }
                                }
                            } else {
                                carryover_stock
                            };

                        let tool = op
                            .tool_id
                            .as_ref()
                            .and_then(|tid| project.tool(tid).cloned());
                        let clearing_tool = op
                            .clearing_tool_id()
                            .and_then(|tid| project.tool(tid).cloned());
                        wave_tasks.push((
                            (s_idx, o_idx),
                            scope,
                            QueuedOpTask {
                                setup: project.setups[s_idx].clone(),
                                sketches: Arc::clone(&shared_sketches),
                                shared_avoid_regions: Arc::clone(&shared_avoid_regions),
                                fixture_docs: Arc::clone(&shared_fixture_docs),
                                machine_z_travel_mm,
                                op: op.clone(),
                                tool,
                                clearing_tool,
                                display_unit: project.display_units,
                                is_first_tool_in_setup: is_first,
                                upstream_rest,
                                carryover_stock: effective_carryover,
                                input_hash: hash,
                                cancel_token: Arc::new(AtomicBool::new(false)),
                            },
                        ));
                    }
                    UpstreamReadiness::FailedUpstream { op_name, message } => {
                        processed.insert((s_idx, o_idx));
                        let err = format!(
                            "Blocked by failed upstream Rest Machining operation '{op_name}': {message}"
                        );
                        cache.record_error(op.id.clone(), hash, err.clone());
                        report.errors.push((scope, err));
                        blocked_this_pass = true;
                    }
                    UpstreamReadiness::Waiting => {}
                }
            }
        }

        if wave_tasks.is_empty() {
            if !blocked_this_pass {
                break;
            }
            continue;
        }

        let results: Vec<WaveTaskOutcome> = wave_tasks
            .into_par_iter()
            .map(|(coord, scope, task)| {
                let op_id = task.op.id.clone();
                let hash = task.input_hash;
                let res = execute_single_task(cad_doc, &task);
                WaveTaskOutcome {
                    coord,
                    scope,
                    op_id,
                    hash,
                    res,
                }
            })
            .collect();

        for WaveTaskOutcome {
            coord,
            scope,
            op_id,
            hash,
            res,
        } in results
        {
            processed.insert(coord);
            match res {
                Ok(tp) => {
                    let arc_tp = Arc::new(tp);
                    cache.insert(op_id.clone(), hash, Arc::clone(&arc_tp));
                    report.generated.push((scope, op_id, arc_tp));
                }
                Err(err) => {
                    let msg = err.to_string();
                    cache.record_error(op_id, hash, msg.clone());
                    report.errors.push((scope, msg));
                }
            }
        }
    }

    cache.reconcile_project_statuses(project, cad_revision, &HashSet::new());
    report
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use chip_cam::ToolpathBuilder;

    #[test]
    fn reset_discards_stale_events_from_previous_epoch() {
        let mut dispatcher = JobDispatcher::new(None);
        let mut project = ProjectDocument::new("Next Project");
        let mut cache = ToolpathCache::new(16);

        let stale_epoch = dispatcher.epoch;
        let stale_hash = OperationInputHash([0xAB; 32]);
        let stale_tp = Arc::new(ToolpathBuilder::new("stale_op").finish());

        // Simulate a project switch via reset() before a stale background worker posts its result.
        dispatcher.reset();
        assert!(!dispatcher.is_busy());
        assert!(dispatcher.active_cad.is_none());

        let _ = dispatcher.tx.send((
            stale_epoch,
            DispatcherEvent::Completed {
                op_id: "stale_op".to_string(),
                input_hash: stale_hash,
                toolpath: stale_tp,
                elapsed: Duration::from_millis(42),
            },
        ));
        let _ = dispatcher.tx.send((
            stale_epoch,
            DispatcherEvent::Failed {
                op_id: "stale_err".to_string(),
                input_hash: stale_hash,
                message: "stale failure".to_string(),
                elapsed: Duration::from_millis(17),
            },
        ));

        let logs = dispatcher.poll_events(&mut project, &mut cache, 1);
        assert!(logs.is_empty(), "stale events must not log: {logs:?}");
        assert!(
            cache.get("stale_op", stale_hash).is_none(),
            "stale Completed event must not populate ToolpathCache"
        );
        assert!(
            cache.get_error("stale_err", stale_hash).is_none(),
            "stale Failed event must not populate ToolpathCache errors"
        );
    }

    #[test]
    fn batch_progress_tracks_completions_and_clears_when_idle_or_cancelled() {
        let mut dispatcher = JobDispatcher::new(None);
        let mut project = ProjectDocument::new("Batch Progress Test");
        let mut cache = ToolpathCache::new(16);

        assert!(dispatcher.batch_progress().is_none());

        let hash_a = OperationInputHash([0x11; 32]);
        let hash_b = OperationInputHash([0x22; 32]);
        dispatcher.batch_ops.insert("op_a".to_string());
        dispatcher.batch_ops.insert("op_b".to_string());
        dispatcher.in_flight.insert(
            "op_a".to_string(),
            (hash_a, Arc::new(AtomicBool::new(false))),
        );
        dispatcher.pending_wave.insert("op_b".to_string());

        let p0 = dispatcher.batch_progress().unwrap();
        assert_eq!(p0.completed, 0);
        assert_eq!(p0.total, 2);
        assert_eq!(p0.percent_rounded(), 0);

        let _ = dispatcher.tx.send((
            dispatcher.epoch,
            DispatcherEvent::Completed {
                op_id: "op_a".to_string(),
                input_hash: hash_a,
                toolpath: Arc::new(ToolpathBuilder::new("op_a").finish()),
                elapsed: Duration::from_millis(25),
            },
        ));
        let _ = dispatcher.poll_events(&mut project, &mut cache, 1);

        let p1 = dispatcher.batch_progress().unwrap();
        assert_eq!(p1.completed, 1);
        assert_eq!(p1.total, 2);
        assert_eq!(p1.percent_rounded(), 50);

        let cancelled = dispatcher.cancel_all();
        assert_eq!(cancelled, 1);
        assert!(dispatcher.batch_progress().is_none());
        assert!(dispatcher.batch_ops.is_empty());

        // Also verify full completion clears batch_progress to None.
        dispatcher.batch_ops.insert("op_c".to_string());
        dispatcher.in_flight.insert(
            "op_c".to_string(),
            (hash_b, Arc::new(AtomicBool::new(false))),
        );
        assert_eq!(dispatcher.batch_progress().unwrap().total, 1);
        let _ = dispatcher.tx.send((
            dispatcher.epoch,
            DispatcherEvent::Completed {
                op_id: "op_c".to_string(),
                input_hash: hash_b,
                toolpath: Arc::new(ToolpathBuilder::new("op_c").finish()),
                elapsed: Duration::from_millis(15),
            },
        ));
        let _ = dispatcher.poll_events(&mut project, &mut cache, 1);
        assert!(dispatcher.batch_progress().is_none());
        assert!(dispatcher.batch_ops.is_empty());
    }

    #[test]
    fn format_exec_duration_formats_milliseconds_seconds_and_minutes() {
        assert_eq!(format_exec_duration(Duration::ZERO), "0ms");
        assert_eq!(format_exec_duration(Duration::from_millis(142)), "142ms");
        assert_eq!(format_exec_duration(Duration::from_millis(999)), "999ms");
        assert_eq!(format_exec_duration(Duration::from_millis(1_000)), "1.00s");
        assert_eq!(format_exec_duration(Duration::from_millis(1_420)), "1.42s");
        assert_eq!(
            format_exec_duration(Duration::from_millis(59_994)),
            "59.99s"
        );
        assert_eq!(
            format_exec_duration(Duration::from_millis(59_995)),
            "1m 00.0s"
        );
        assert_eq!(
            format_exec_duration(Duration::from_millis(125_300)),
            "2m 05.3s"
        );
        assert_eq!(
            format_exec_duration(Duration::from_millis(119_990)),
            "2m 00.0s"
        );
    }

    #[test]
    fn poll_events_logs_wall_clock_execution_time_for_completed_failed_cancelled_and_batch() {
        let mut dispatcher = JobDispatcher::new(None);
        let mut project = ProjectDocument::new("Exec Time Log Test");
        let mut cache = ToolpathCache::new(16);

        let hash_a = OperationInputHash([0x01; 32]);
        let hash_b = OperationInputHash([0x02; 32]);
        let hash_c = OperationInputHash([0x03; 32]);
        dispatcher.batch_started_at = Some(Instant::now());
        dispatcher.batch_ops.insert("op_pocket".to_string());
        dispatcher.batch_ops.insert("op_contour".to_string());
        dispatcher.batch_ops.insert("op_drill".to_string());
        dispatcher.in_flight.insert(
            "op_pocket".to_string(),
            (hash_a, Arc::new(AtomicBool::new(false))),
        );
        dispatcher.in_flight.insert(
            "op_contour".to_string(),
            (hash_b, Arc::new(AtomicBool::new(false))),
        );
        dispatcher.in_flight.insert(
            "op_drill".to_string(),
            (hash_c, Arc::new(AtomicBool::new(false))),
        );

        let _ = dispatcher.tx.send((
            dispatcher.epoch,
            DispatcherEvent::Completed {
                op_id: "op_pocket".to_string(),
                input_hash: hash_a,
                toolpath: Arc::new(ToolpathBuilder::new("op_pocket").finish()),
                elapsed: Duration::from_millis(142),
            },
        ));
        let _ = dispatcher.tx.send((
            dispatcher.epoch,
            DispatcherEvent::Failed {
                op_id: "op_contour".to_string(),
                input_hash: hash_b,
                message: "invalid tool".to_string(),
                elapsed: Duration::from_millis(1_420),
            },
        ));
        let _ = dispatcher.tx.send((
            dispatcher.epoch,
            DispatcherEvent::Cancelled {
                op_id: "op_drill".to_string(),
                input_hash: hash_c,
                elapsed: Duration::from_millis(125_300),
            },
        ));

        let logs = dispatcher.poll_events(&mut project, &mut cache, 1);
        assert_eq!(
            logs.len(),
            4,
            "expected 3 op logs + 1 batch summary: {logs:?}"
        );
        assert!(
            logs[0].contains("Generated 'op_pocket' in 142ms")
                && logs[0].contains("est. cycle 0.0s"),
            "unexpected completed log: {}",
            logs[0]
        );
        assert_eq!(logs[1], "Failed 'op_contour' after 1.42s: invalid tool");
        assert_eq!(
            logs[2],
            "Cancelled generation for 'op_drill' after 2m 05.3s"
        );
        assert!(
            logs[3].starts_with("Toolpath generation finished (1 succeeded, 1 failed) in "),
            "unexpected batch summary: {}",
            logs[3]
        );
        assert!(dispatcher.batch_elapsed().is_none());
    }
}
