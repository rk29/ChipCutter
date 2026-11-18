use std::{
    collections::{HashMap, HashSet},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
};

use chip_cad::CadDocument;
use chip_cam::{
    CamError, GeneratedToolpath, Operation, OperationGenRequest, OperationStatus, Setup,
    UpstreamSweptInput, generate_operation_toolpath_with_rest,
};
use chip_math::DisplayUnit;
use chip_sketch::SketchLayer;
use chip_tools::{CutterGeometry, Tool};
use crossbeam_channel::{Receiver, Sender};
use rayon::prelude::*;

use crate::{
    cache::{
        OperationInputHash, ToolpathCache, compute_operation_input_hash,
        resolve_upstream_rest_op_indices,
    },
    document::ProjectDocument,
};

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
    },
    Cancelled {
        op_id: String,
        input_hash: OperationInputHash,
    },
    Failed {
        op_id: String,
        input_hash: OperationInputHash,
        message: String,
    },
}

#[derive(Clone)]
struct QueuedOpTask {
    setup: Setup,
    sketches: Arc<Vec<SketchLayer>>,
    op: Operation,
    tool: Option<Tool>,
    clearing_tool: Option<Tool>,
    display_unit: DisplayUnit,
    is_first_tool_in_setup: bool,
    upstream_rest: Vec<UpstreamSweptInput>,
    input_hash: OperationInputHash,
    cancel_token: Arc<AtomicBool>,
}

enum UpstreamReadiness {
    Ready(Vec<UpstreamSweptInput>),
    Waiting,
    FailedUpstream { op_name: String, message: String },
}

fn cutter_corner_radius_mm(geom: &CutterGeometry) -> f64 {
    match *geom {
        CutterGeometry::BullNoseEndmill {
            corner_radius_mm, ..
        }
        | CutterGeometry::FaceMill {
            corner_radius_mm, ..
        } => corner_radius_mm,
        CutterGeometry::BallEndmill { diameter_mm, .. } => 0.5 * diameter_mm,
        _ => 0.0,
    }
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
                    cutter_corner_radius_mm(&t.geometry),
                )
            })
            .unwrap_or((0.0, 0.0));

        inputs.push(UpstreamSweptInput {
            op_id: u_op.id.clone(),
            tool_diameter_mm: tool_d,
            corner_radius_mm: corner_r,
            toolpath: u_tp,
        });
    }
    UpstreamReadiness::Ready(inputs)
}

/// Non-blocking background toolpath generation coordinator backed by `rayon` (`01-architecture.md` & `05-operations.md`).
///
/// Supports topological wave execution for `RestMachiningMode::FromPrecedingOperations`: downstream
/// Rest operations wait in `pending_wave` until their upstream roughing toolpaths complete, then
/// automatically spawn with exact 2D swept coverage inputs.
pub struct JobDispatcher {
    tx: Sender<DispatcherEvent>,
    rx: Receiver<DispatcherEvent>,
    in_flight: HashMap<String, (OperationInputHash, Arc<AtomicBool>)>,
    pending_wave: HashSet<String>,
    active_cad: Option<(Arc<CadDocument>, u64)>,
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
        Self {
            tx,
            rx,
            in_flight: HashMap::new(),
            pending_wave: HashSet::new(),
            active_cad: None,
            wakeup_cb,
        }
    }

    pub fn is_busy(&self) -> bool {
        !self.in_flight.is_empty() || !self.pending_wave.is_empty()
    }

    pub fn in_flight_count(&self) -> usize {
        self.in_flight.len() + self.pending_wave.len()
    }

    pub fn in_flight_ids(&self) -> HashSet<String> {
        let mut ids: HashSet<String> = self.in_flight.keys().cloned().collect();
        ids.extend(self.pending_wave.iter().cloned());
        ids
    }

    /// Signals cancellation for a single operation if currently generating or waiting in a DAG wave.
    pub fn cancel_operation(&mut self, op_id: &str) -> bool {
        let removed_pending = self.pending_wave.remove(op_id);
        if let Some((_, token)) = self.in_flight.remove(op_id) {
            token.store(true, Ordering::Relaxed);
            true
        } else {
            removed_pending
        }
    }

    /// Signals cancellation for all in-flight and queued wave jobs (`Esc`).
    pub fn cancel_all(&mut self) -> usize {
        let count = self.in_flight.len() + self.pending_wave.len();
        self.pending_wave.clear();
        for (_, (_, token)) in self.in_flight.drain() {
            token.store(true, Ordering::Relaxed);
        }
        count
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

        self.in_flight.retain(|op_id, (job_hash, token)| {
            let still_matches = current_hashes.get(op_id) == Some(job_hash);
            if !still_matches {
                token.store(true, Ordering::Relaxed);
            }
            still_matches
        });
        self.pending_wave
            .retain(|op_id| current_hashes.contains_key(op_id));
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
        self.active_cad = Some((Arc::clone(&cad_doc), cad_revision));
        let shared_sketches = Arc::new(project.sketches.clone());
        let mut tasks = Vec::new();
        let mut queued_count = 0usize;

        for s_idx in 0..project.setups.len() {
            if let Some(target_s) = only_setup_idx
                && s_idx != target_s
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
                if let Some(target_id) = only_op_id
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

                match check_upstream_readiness(project, cache, s_idx, o_idx, cad_revision) {
                    UpstreamReadiness::Ready(upstream_rest) => {
                        self.pending_wave.remove(&op_id);
                        let cancel_token = Arc::new(AtomicBool::new(false));
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

                        tasks.push(QueuedOpTask {
                            setup: project.setups[s_idx].clone(),
                            sketches: Arc::clone(&shared_sketches),
                            op: project.setups[s_idx].operations[o_idx].clone(),
                            tool,
                            clearing_tool,
                            display_unit: project.display_units,
                            is_first_tool_in_setup: is_first_tool,
                            upstream_rest,
                            input_hash,
                            cancel_token,
                        });
                        queued_count += 1;
                    }
                    UpstreamReadiness::Waiting | UpstreamReadiness::FailedUpstream { .. } => {
                        self.pending_wave.insert(op_id);
                        project.setups[s_idx].operations[o_idx].runtime_status =
                            OperationStatus::Generating;
                        queued_count += 1;
                    }
                }
            }
        }

        self.spawn_rayon_tasks(&cad_doc, tasks);
        queued_count
    }

    fn spawn_rayon_tasks(&self, cad_doc: &Arc<CadDocument>, tasks: Vec<QueuedOpTask>) {
        for task in tasks {
            let tx = self.tx.clone();
            let wakeup = self.wakeup_cb.clone();
            let cad = Arc::clone(cad_doc);

            rayon::spawn(move || {
                let op_id = task.op.id.clone();
                let _ = tx.send(DispatcherEvent::Started {
                    op_id: op_id.clone(),
                    input_hash: task.input_hash,
                });

                if task.cancel_token.load(Ordering::Relaxed) {
                    let _ = tx.send(DispatcherEvent::Cancelled {
                        op_id,
                        input_hash: task.input_hash,
                    });
                    if let Some(cb) = &wakeup {
                        cb();
                    }
                    return;
                }

                let res = execute_single_task(&cad, &task);
                match res {
                    Ok(tp) => {
                        let _ = tx.send(DispatcherEvent::Completed {
                            op_id,
                            input_hash: task.input_hash,
                            toolpath: Arc::new(tp),
                        });
                    }
                    Err(CamError::Cancelled) => {
                        let _ = tx.send(DispatcherEvent::Cancelled {
                            op_id,
                            input_hash: task.input_hash,
                        });
                    }
                    Err(err) => {
                        let _ = tx.send(DispatcherEvent::Failed {
                            op_id,
                            input_hash: task.input_hash,
                            message: err.to_string(),
                        });
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
        while let Ok(ev) = self.rx.try_recv() {
            match ev {
                DispatcherEvent::Started { .. } => {}
                DispatcherEvent::Completed {
                    op_id,
                    input_hash,
                    toolpath,
                } => {
                    let matches_inflight = self
                        .in_flight
                        .get(&op_id)
                        .is_some_and(|(h, _)| *h == input_hash);
                    if matches_inflight {
                        self.in_flight.remove(&op_id);
                    }
                    logs.push(format!(
                        "Generated '{}' ({} passes, {:.1}s est., {} G2/G3 arcs)",
                        op_id,
                        toolpath.stats.pass_count,
                        toolpath.stats.total_est_time_sec,
                        toolpath.stats.arc_move_count
                    ));
                    cache.insert(op_id, input_hash, toolpath);
                }
                DispatcherEvent::Cancelled { op_id, input_hash } => {
                    let matches_inflight = self
                        .in_flight
                        .get(&op_id)
                        .is_some_and(|(h, _)| *h == input_hash);
                    if matches_inflight {
                        self.in_flight.remove(&op_id);
                    }
                    logs.push(format!("Cancelled generation for '{op_id}'"));
                }
                DispatcherEvent::Failed {
                    op_id,
                    input_hash,
                    message,
                } => {
                    let matches_inflight = self
                        .in_flight
                        .get(&op_id)
                        .is_some_and(|(h, _)| *h == input_hash);
                    if matches_inflight {
                        self.in_flight.remove(&op_id);
                    }
                    logs.push(format!("Error generating '{op_id}': {message}"));
                    cache.record_error(op_id, input_hash, message);
                }
            }
        }

        // Advance topological DAG waves for any `pending_wave` operations whose upstream deps finished!
        if !self.pending_wave.is_empty()
            && let Some((cad_doc, saved_rev)) = self.active_cad.clone()
            && saved_rev == cad_revision
        {
            let shared_sketches = Arc::new(project.sketches.clone());
            let mut next_tasks = Vec::new();
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

                    match check_upstream_readiness(project, cache, s_idx, o_idx, cad_revision) {
                        UpstreamReadiness::Ready(upstream_rest) => {
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
                            next_tasks.push(QueuedOpTask {
                                setup: project.setups[s_idx].clone(),
                                sketches: Arc::clone(&shared_sketches),
                                op: project.setups[s_idx].operations[o_idx].clone(),
                                tool,
                                clearing_tool,
                                display_unit: project.display_units,
                                is_first_tool_in_setup: is_first_tool,
                                upstream_rest,
                                input_hash,
                                cancel_token,
                            });
                        }
                        UpstreamReadiness::FailedUpstream { op_name, message } => {
                            self.pending_wave.remove(&op_id);
                            let err = format!(
                                "Blocked by failed upstream Rest Machining operation '{op_name}': {message}"
                            );
                            logs.push(format!("Error generating '{op_id}': {err}"));
                            cache.record_error(op_id, input_hash, err);
                        }
                        UpstreamReadiness::Waiting => {
                            // If nothing is left in-flight at all, an upstream dependency was cancelled or missing
                            if self.in_flight.is_empty() && next_tasks.is_empty() {
                                self.pending_wave.remove(&op_id);
                            }
                        }
                    }
                }
            }
            if !next_tasks.is_empty() {
                self.spawn_rayon_tasks(&cad_doc, next_tasks);
            }
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
    generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: cad,
        sketches: &task.sketches,
        setup: &resolved_setup,
        op: &task.op,
        tool,
        clearing_tool: task.clearing_tool.as_ref(),
        display_unit: task.display_unit,
        is_first_tool_in_setup: task.is_first_tool_in_setup,
        upstream_rest: &task.upstream_rest,
        cancel: &task.cancel_token,
    })
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

    let mut processed: HashSet<(usize, usize)> = HashSet::new();
    let shared_sketches = Arc::new(project.sketches.clone());

    for _wave in 0..=total_ops {
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
                match check_upstream_readiness(project, cache, s_idx, o_idx, cad_revision) {
                    UpstreamReadiness::Ready(upstream_rest) => {
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
                                op: op.clone(),
                                tool,
                                clearing_tool,
                                display_unit: project.display_units,
                                is_first_tool_in_setup: is_first,
                                upstream_rest,
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
