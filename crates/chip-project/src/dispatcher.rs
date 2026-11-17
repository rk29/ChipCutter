use std::{
    collections::{HashMap, HashSet},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
};

use chip_cad::CadDocument;
use chip_cam::{
    CamError, GeneratedToolpath, Operation, OperationStatus, Setup, generate_operation_toolpath,
};
use chip_math::DisplayUnit;
use chip_tools::Tool;
use crossbeam_channel::{Receiver, Sender};
use rayon::prelude::*;

use crate::{
    cache::{OperationInputHash, ToolpathCache, compute_operation_input_hash},
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
    op: Operation,
    tool: Option<Tool>,
    display_unit: DisplayUnit,
    is_first_tool_in_setup: bool,
    input_hash: OperationInputHash,
    cancel_token: Arc<AtomicBool>,
}

/// Non-blocking background toolpath generation coordinator backed by `rayon` (`01-architecture.md` & `05-operations.md`).
pub struct JobDispatcher {
    tx: Sender<DispatcherEvent>,
    rx: Receiver<DispatcherEvent>,
    in_flight: HashMap<String, (OperationInputHash, Arc<AtomicBool>)>,
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
            wakeup_cb,
        }
    }

    pub fn is_busy(&self) -> bool {
        !self.in_flight.is_empty()
    }

    pub fn in_flight_count(&self) -> usize {
        self.in_flight.len()
    }

    pub fn in_flight_ids(&self) -> HashSet<String> {
        self.in_flight.keys().cloned().collect()
    }

    /// Signals cancellation for a single operation if currently generating.
    pub fn cancel_operation(&mut self, op_id: &str) -> bool {
        if let Some((_, token)) = self.in_flight.remove(op_id) {
            token.store(true, Ordering::Relaxed);
            true
        } else {
            false
        }
    }

    /// Signals cancellation for all in-flight background generation jobs (`Esc`).
    pub fn cancel_all(&mut self) -> usize {
        let count = self.in_flight.len();
        for (_, (_, token)) in self.in_flight.drain() {
            token.store(true, Ordering::Relaxed);
        }
        count
    }

    /// Cancels any in-flight job whose `OperationInputHash` no longer matches `project`
    /// (for example, when the user edits an operation mid-generation!).
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
    }

    /// Queues dirty (or explicitly targeted) non-suppressed operations onto the background `rayon` pool.
    /// Returns the number of operations spawned.
    pub fn dispatch_operations(
        &mut self,
        project: &mut ProjectDocument,
        cad_doc: Arc<CadDocument>,
        cad_revision: u64,
        cache: &ToolpathCache,
        only_setup_idx: Option<usize>,
        only_op_id: Option<&str>,
    ) -> usize {
        let mut tasks = Vec::new();

        for s_idx in 0..project.setups.len() {
            if let Some(target_s) = only_setup_idx
                && s_idx != target_s
            {
                continue;
            }
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
                {
                    continue;
                }

                let Some(input_hash) =
                    compute_operation_input_hash(project, s_idx, o_idx, cad_revision)
                else {
                    continue;
                };

                // Skip if already cached Clean (unless explicitly forced via `only_op_id`)
                if only_op_id.is_none() && cache.get(&op_id, input_hash).is_some() {
                    project.setups[s_idx].operations[o_idx].runtime_status = OperationStatus::Clean;
                    continue;
                }

                // Cancel any previous in-flight run for `op_id`
                if let Some((_, old_tok)) = self.in_flight.remove(&op_id) {
                    old_tok.store(true, Ordering::Relaxed);
                }

                let cancel_token = Arc::new(AtomicBool::new(false));
                self.in_flight
                    .insert(op_id.clone(), (input_hash, Arc::clone(&cancel_token)));
                project.setups[s_idx].operations[o_idx].runtime_status =
                    OperationStatus::Generating;

                let tool = project.setups[s_idx].operations[o_idx]
                    .tool_id
                    .as_ref()
                    .and_then(|tid| project.tool(tid).cloned());

                tasks.push(QueuedOpTask {
                    setup: project.setups[s_idx].clone(),
                    op: project.setups[s_idx].operations[o_idx].clone(),
                    tool,
                    display_unit: project.display_units,
                    is_first_tool_in_setup: is_first_tool,
                    input_hash,
                    cancel_token,
                });
            }
        }

        let spawned_count = tasks.len();
        for task in tasks {
            let tx = self.tx.clone();
            let wakeup = self.wakeup_cb.clone();
            let cad = Arc::clone(&cad_doc);

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

        spawned_count
    }

    /// Drains all pending [`DispatcherEvent`]s, updating `cache` and `project` statuses.
    /// Returns human-readable log lines for any completed/failed events.
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

        let inflight_set = self.in_flight_ids();
        cache.reconcile_project_statuses(project, cad_revision, &inflight_set);
        logs
    }
}

fn execute_single_task(
    cad: &CadDocument,
    task: &QueuedOpTask,
) -> Result<GeneratedToolpath, CamError> {
    let resolved_setup = task.setup.resolve_immutable(cad, task.display_unit)?;
    let tool = task.tool.as_ref().ok_or_else(|| CamError::MissingTool {
        op_name: task.op.name.clone(),
    })?;
    generate_operation_toolpath(
        cad,
        &resolved_setup,
        &task.op,
        tool,
        task.display_unit,
        task.is_first_tool_in_setup,
        &task.cancel_token,
    )
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

/// Generates all non-suppressed operations across `project` in parallel on `rayon`.
pub fn generate_project_blocking(
    project: &mut ProjectDocument,
    cad_doc: &CadDocument,
    cache: &mut ToolpathCache,
    cad_revision: u64,
) -> BatchGenerationReport {
    let mut tasks: Vec<(String, QueuedOpTask)> = Vec::new();
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
            let Some(hash) = compute_operation_input_hash(project, s_idx, o_idx, cad_revision)
            else {
                continue;
            };
            let tool = op
                .tool_id
                .as_ref()
                .and_then(|tid| project.tool(tid).cloned());
            tasks.push((
                format!("Setup '{}' -> Op '{}'", setup_name, op.name),
                QueuedOpTask {
                    setup: project.setups[s_idx].clone(),
                    op: op.clone(),
                    tool,
                    display_unit: project.display_units,
                    is_first_tool_in_setup: is_first,
                    input_hash: hash,
                    cancel_token: Arc::new(AtomicBool::new(false)),
                },
            ));
        }
    }

    let results: Vec<(
        String,
        String,
        OperationInputHash,
        Result<GeneratedToolpath, CamError>,
    )> = tasks
        .into_par_iter()
        .map(|(scope, task)| {
            let op_id = task.op.id.clone();
            let hash = task.input_hash;
            let res = execute_single_task(cad_doc, &task);
            (scope, op_id, hash, res)
        })
        .collect();

    let mut report = BatchGenerationReport::default();
    for (scope, op_id, hash, res) in results {
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

    cache.reconcile_project_statuses(project, cad_revision, &HashSet::new());
    report
}
