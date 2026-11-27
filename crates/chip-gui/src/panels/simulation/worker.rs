use std::{
    collections::BTreeMap,
    sync::{Arc, Condvar, Mutex, atomic::AtomicBool},
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use chip_cad::CadDocument;
use chip_cam::{FixtureKind, GeneratedToolpath, Operation, resolve_setup_fixtures};
use chip_project::{
    AllSetupsSimRequest, SetupSimRequest, collect_ready_setup_operations, format_exec_duration,
    resolve_setup_kinematics, simulate_all_setups, simulate_project_setup,
};
#[cfg(feature = "gpu-compute")]
use chip_sim::{GpuCheckpointCache, GpuSimContext, WgpuComputeBackend};
use chip_sim::{
    ScrubSnapshot, SetupSimulationInput, SetupSimulationResult, SimEngineKind, SimError,
    SimMeshOptions, SimOperationInput, SimProgressTracker, SimStockColorMode, SimStockMesh,
    SimulatedStock, SimulatedStockPlaybackExt, StockPlaybackCursor, format_cycle_time_hms,
    run_setup_simulation,
};
use chip_tools::Tool;
use crossbeam_channel::{Receiver, Sender, unbounded};

use super::{
    InFlightSim, MultiSetupSimEntry, MultiSetupSimSummary, SimTriggerArgs, SimulationPanelState,
};

struct OwnedSimOp {
    op: Operation,
    tool: Tool,
    toolpath: Arc<GeneratedToolpath>,
}

pub(super) enum SimWorkerMsg {
    Finished {
        run_id: u64,
        setup_idx: usize,
        signature: u64,
        engine: SimEngineKind,
        elapsed: Duration,
        result: Result<Arc<SetupSimulationResult>, String>,
    },
    AllSetupsFinished {
        run_id: u64,
        active_setup_idx: usize,
        active_signature: u64,
        engine: SimEngineKind,
        resolution: chip_sim::SimResolution,
        elapsed: Duration,
        result: Result<Vec<MultiSetupSimEntry>, String>,
    },
}

/// Visual shading metadata used to detect when color-mode changes require re-tessellation
/// even if the underlying stock solid has not changed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct StockMeshMeta {
    pub(super) run_id: u64,
    pub(super) result_ptr: usize,
    pub(super) color_by_operation: bool,
    pub(super) heatmap_active: bool,
    pub(super) mesh_skinning: bool,
    pub(super) tol_key: u32,
    pub(super) scale_key: u32,
}

/// Full submission deduplication key so idle frames never enqueue duplicate remesh work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct StockRemeshKey {
    pub(super) meta: StockMeshMeta,
    pub(super) seg_idx: usize,
    pub(super) seg_u_bits: u64,
    pub(super) time_bits: u64,
}

/// Request queued onto [`StockRemeshWorker`] to synchronize the worker-owned [`SimulatedStock`]
/// to a timeline scrub snapshot and extract a fresh [`SimStockMesh`] off the `egui` UI thread.
pub(super) struct StockRemeshRequest {
    pub(super) request_id: u64,
    pub(super) run_id: u64,
    pub(super) result_ptr: usize,
    pub(super) sim_result: Arc<SetupSimulationResult>,
    pub(super) target_time_sec: f64,
    pub(super) scrub_snapshot: Option<ScrubSnapshot>,
    pub(super) color_mode: SimStockColorMode,
    pub(super) mesh_skinning: bool,
    pub(super) meta: StockMeshMeta,
    pub(super) egui_ctx: Option<egui::Context>,
}

/// Payload produced by [`StockRemeshWorker`] (CPU indexed mesh or flat GPU stock snapshot uploaded
/// strictly on the main UI thread so background threads never touch `eframe`'s presentation device).
pub(super) enum StockRemeshPayload {
    CpuMesh(Arc<SimStockMesh>),
    #[cfg(feature = "gpu-compute")]
    GpuStockSnapshot(Arc<SimulatedStock>),
}

/// Completed remesh response produced by [`StockRemeshWorker`].
pub(super) struct StockRemeshOutput {
    pub(super) request_id: u64,
    pub(super) run_id: u64,
    pub(super) result_ptr: usize,
    pub(super) meta: StockMeshMeta,
    /// `Some(payload)` when stock geometry or shading mode changed; `None` when the requested
    /// timestamp required no stock mutation or color-mode remesh (e.g. air rapid / rotary slew).
    pub(super) payload: Option<StockRemeshPayload>,
}

#[derive(Default)]
struct RemeshWorkerState {
    working_stock_identity: Option<(u64, usize)>,
    working_stock_cursor: Option<StockPlaybackCursor>,
    working_stock: Option<SimulatedStock>,
    #[cfg(feature = "gpu-compute")]
    compute_gpu_ctx: Option<Arc<GpuSimContext>>,
    #[cfg(feature = "gpu-compute")]
    gpu_backend: Option<WgpuComputeBackend>,
    #[cfg(feature = "gpu-compute")]
    gpu_checkpoints: GpuCheckpointCache,
    last_meshed_meta: Option<StockMeshMeta>,
    has_emitted_mesh: bool,
    processed_requests: u64,
}

impl RemeshWorkerState {
    fn reset(&mut self) {
        self.working_stock_identity = None;
        self.working_stock_cursor = None;
        self.working_stock = None;
        #[cfg(feature = "gpu-compute")]
        {
            self.gpu_backend = None;
            self.gpu_checkpoints.clear();
        }
        self.last_meshed_meta = None;
        self.has_emitted_mesh = false;
    }

    fn process(&mut self, req: &StockRemeshRequest) -> StockRemeshOutput {
        self.processed_requests += 1;
        let run_identity = (req.run_id, req.result_ptr);

        #[cfg(feature = "gpu-compute")]
        if req.sim_result.active_backend.is_gpu {
            if self.compute_gpu_ctx.is_none() {
                self.compute_gpu_ctx = GpuSimContext::shared_headless();
            }
            if let Some(ctx) = self.compute_gpu_ctx.as_ref().map(Arc::clone) {
                if self.working_stock_identity != Some(run_identity) {
                    self.working_stock_identity = Some(run_identity);
                    self.working_stock = None;
                    let total_spans = req.sim_result.timeline.motion_spans.len();
                    self.working_stock_cursor = Some(StockPlaybackCursor {
                        seg_idx: total_spans,
                        seg_u: 0.0,
                    });
                    let backend = WgpuComputeBackend::new(
                        ctx,
                        req.sim_result.final_stock.clone(),
                        req.sim_result.target_tridexel_stock.clone(),
                    );
                    self.gpu_checkpoints.clear();
                    let _ = self
                        .gpu_checkpoints
                        .capture_from_backend(&backend, total_spans);
                    self.gpu_backend = Some(backend);
                    self.last_meshed_meta = None;
                    self.has_emitted_mesh = false;
                }

                let stock_changed = if let (Some(backend), Some(snap)) =
                    (self.gpu_backend.as_mut(), req.scrub_snapshot.as_ref())
                {
                    req.sim_result.sync_gpu_backend_to_scrub_snapshot(
                        backend,
                        &mut self.gpu_checkpoints,
                        &mut self.working_stock_cursor,
                        snap,
                    )
                } else if let Some(backend) = self.gpu_backend.as_mut() {
                    req.sim_result.sync_gpu_backend_to_time(
                        backend,
                        &mut self.gpu_checkpoints,
                        &mut self.working_stock_cursor,
                        req.target_time_sec,
                    )
                } else {
                    false
                };

                let needs_publish = stock_changed || !self.has_emitted_mesh;
                let payload = if needs_publish {
                    let snapshot = self
                        .gpu_backend
                        .as_ref()
                        .map(|b| b.with_synced_cpu_stock(|s| Arc::new(s.clone())));
                    self.last_meshed_meta = Some(req.meta);
                    self.has_emitted_mesh = true;
                    snapshot.map(StockRemeshPayload::GpuStockSnapshot)
                } else {
                    None
                };

                return StockRemeshOutput {
                    request_id: req.request_id,
                    run_id: req.run_id,
                    result_ptr: req.result_ptr,
                    meta: req.meta,
                    payload,
                };
            }
        }

        if self.working_stock_identity != Some(run_identity) {
            self.working_stock_identity = Some(run_identity);
            #[cfg(feature = "gpu-compute")]
            {
                self.gpu_backend = None;
                self.gpu_checkpoints.clear();
            }
            self.working_stock_cursor = Some(StockPlaybackCursor {
                seg_idx: req.sim_result.timeline.motion_spans.len(),
                seg_u: 0.0,
            });
            self.working_stock = Some(req.sim_result.final_stock.clone());
            self.last_meshed_meta = None;
            self.has_emitted_mesh = false;
        }

        let stock_changed = if let (Some(work), Some(snap)) =
            (self.working_stock.as_mut(), req.scrub_snapshot.as_ref())
        {
            work.sync_to_scrub_snapshot(&mut self.working_stock_cursor, &req.sim_result, snap)
        } else if let Some(work) = self.working_stock.as_mut() {
            work.sync_to_time(
                &mut self.working_stock_cursor,
                &req.sim_result,
                req.target_time_sec,
            )
        } else {
            false
        };

        let needs_remesh =
            stock_changed || !self.has_emitted_mesh || self.last_meshed_meta != Some(req.meta);

        let payload = if needs_remesh {
            let extracted =
                self.working_stock
                    .as_ref()
                    .map_or_else(SimStockMesh::default, |work| {
                        req.sim_result.extract_stock_mesh_with_options(
                            work,
                            SimMeshOptions::new(req.color_mode, req.mesh_skinning),
                        )
                    });
            self.last_meshed_meta = Some(req.meta);
            self.has_emitted_mesh = true;
            Some(StockRemeshPayload::CpuMesh(Arc::new(extracted)))
        } else {
            None
        };

        StockRemeshOutput {
            request_id: req.request_id,
            run_id: req.run_id,
            result_ptr: req.result_ptr,
            meta: req.meta,
            payload,
        }
    }
}

struct SharedSlot {
    pending: Option<StockRemeshRequest>,
    reset_requested: bool,
    busy: bool,
    completed_request_id: u64,
    shutdown: bool,
}

#[cfg(test)]
type BeforeProcessHook = Arc<dyn Fn(u64) + Send + Sync>;

struct SharedRemeshState {
    slot: Mutex<SharedSlot>,
    wake_cv: Condvar,
    idle_cv: Condvar,
    state: Mutex<RemeshWorkerState>,
    #[cfg(test)]
    before_process_hook: Mutex<Option<BeforeProcessHook>>,
}

/// Dedicated background worker owning the mutable playback [`SimulatedStock`] and performing
/// off-thread scrub synchronization and mesh extraction with latest-wins request coalescing.
///
/// Rapid scrubber drags overwrite any unstarted pending request so intermediate stale frames
/// are skipped without blocking the `egui` UI thread, while [`Self::wait_idle`] guarantees
/// 100% deterministic synchronous completion for headless `--capture-*` screenshots.
pub(super) struct StockRemeshWorker {
    shared: Arc<SharedRemeshState>,
    out_tx: Sender<StockRemeshOutput>,
    out_rx: Receiver<StockRemeshOutput>,
    thread: Option<JoinHandle<()>>,
}

impl StockRemeshWorker {
    pub(super) fn new() -> Self {
        let (out_tx, out_rx) = unbounded();
        let shared = Arc::new(SharedRemeshState {
            slot: Mutex::new(SharedSlot {
                pending: None,
                reset_requested: false,
                busy: false,
                completed_request_id: 0,
                shutdown: false,
            }),
            wake_cv: Condvar::new(),
            idle_cv: Condvar::new(),
            state: Mutex::new(RemeshWorkerState::default()),
            #[cfg(test)]
            before_process_hook: Mutex::new(None),
        });
        Self {
            shared,
            out_tx,
            out_rx,
            thread: None,
        }
    }

    fn ensure_started(&mut self) -> bool {
        if self.thread.is_some() {
            return true;
        }
        let shared = Arc::clone(&self.shared);
        let out_tx = self.out_tx.clone();
        match thread::Builder::new()
            .name("chip-stock-remesh".to_string())
            .spawn(move || remesh_worker_loop(shared, out_tx))
        {
            Ok(handle) => {
                self.thread = Some(handle);
                true
            }
            Err(_) => false,
        }
    }

    /// Enqueues `req` non-blockingly, replacing any older unstarted pending request (coalescing).
    pub(super) fn submit(&mut self, req: StockRemeshRequest) {
        if !self.ensure_started() {
            // Graceful fallback if OS refuses thread creation: run inline without panicking.
            let output = {
                let mut st = self.shared.state.lock().unwrap_or_else(|e| e.into_inner());
                st.process(&req)
            };
            let req_id = output.request_id;
            let _ = self.out_tx.send(output);
            let mut guard = self.shared.slot.lock().unwrap_or_else(|e| e.into_inner());
            guard.completed_request_id = guard.completed_request_id.max(req_id);
            self.shared.idle_cv.notify_all();
            return;
        }

        let mut guard = self.shared.slot.lock().unwrap_or_else(|e| e.into_inner());
        guard.pending = Some(req);
        self.shared.wake_cv.notify_one();
    }

    /// Discards any pending scrub request and resets the worker's cached `SimulatedStock`.
    pub(super) fn reset(&mut self) {
        while self.out_rx.try_recv().is_ok() {}
        if self.thread.is_none() {
            let mut st = self.shared.state.lock().unwrap_or_else(|e| e.into_inner());
            st.reset();
            let mut guard = self.shared.slot.lock().unwrap_or_else(|e| e.into_inner());
            guard.pending = None;
            guard.reset_requested = false;
            guard.busy = false;
            return;
        }

        let mut guard = self.shared.slot.lock().unwrap_or_else(|e| e.into_inner());
        guard.pending = None;
        guard.reset_requested = true;
        self.shared.wake_cv.notify_one();
    }

    pub(super) fn is_busy(&self) -> bool {
        let guard = self.shared.slot.lock().unwrap_or_else(|e| e.into_inner());
        guard.busy || guard.pending.is_some() || guard.reset_requested
    }

    pub(super) fn try_recv(&self) -> Option<StockRemeshOutput> {
        self.out_rx.try_recv().ok()
    }

    /// Blocks until the worker finishes all pending resets and remesh requests.
    /// Used exclusively for deterministic headless `--capture-*` rendering and tests.
    pub(super) fn wait_idle(&self) {
        let mut guard = self.shared.slot.lock().unwrap_or_else(|e| e.into_inner());
        while (guard.busy || guard.pending.is_some() || guard.reset_requested) && !guard.shutdown {
            guard = self
                .shared
                .idle_cv
                .wait(guard)
                .unwrap_or_else(|e| e.into_inner());
        }
    }

    #[cfg(all(test, feature = "gpu-compute"))]
    pub(super) fn compute_gpu_ctx_ptr(&self) -> Option<*const GpuSimContext> {
        self.wait_idle();
        let st = self.shared.state.lock().unwrap_or_else(|e| e.into_inner());
        st.compute_gpu_ctx.as_ref().map(Arc::as_ptr)
    }

    #[cfg(all(test, feature = "gpu-compute"))]
    pub(super) fn processed_request_count(&self) -> u64 {
        self.wait_idle();
        let st = self.shared.state.lock().unwrap_or_else(|e| e.into_inner());
        st.processed_requests
    }

    #[cfg(test)]
    pub(super) fn set_before_process_hook(&self, hook: Option<BeforeProcessHook>) {
        let mut guard = self
            .shared
            .before_process_hook
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        *guard = hook;
    }

    #[cfg(test)]
    pub(super) fn inspect_working_stock<R>(
        &self,
        f: impl FnOnce(Option<&SimulatedStock>) -> R,
    ) -> R {
        self.wait_idle();
        let st = self.shared.state.lock().unwrap_or_else(|e| e.into_inner());
        #[cfg(feature = "gpu-compute")]
        if let Some(backend) = &st.gpu_backend {
            return backend.with_synced_cpu_stock(|s| f(Some(s)));
        }
        f(st.working_stock.as_ref())
    }
}

impl Drop for StockRemeshWorker {
    fn drop(&mut self) {
        {
            let mut guard = self.shared.slot.lock().unwrap_or_else(|e| e.into_inner());
            guard.shutdown = true;
            guard.pending = None;
            self.shared.wake_cv.notify_all();
            self.shared.idle_cv.notify_all();
        }
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
    }
}

fn remesh_worker_loop(shared: Arc<SharedRemeshState>, out_tx: Sender<StockRemeshOutput>) {
    loop {
        let (do_reset, req) = {
            let mut guard = shared.slot.lock().unwrap_or_else(|e| e.into_inner());
            loop {
                if guard.shutdown {
                    guard.busy = false;
                    shared.idle_cv.notify_all();
                    return;
                }
                if guard.reset_requested || guard.pending.is_some() {
                    let r = guard.reset_requested;
                    guard.reset_requested = false;
                    let next_req = guard.pending.take();
                    guard.busy = true;
                    break (r, next_req);
                }
                guard.busy = false;
                shared.idle_cv.notify_all();
                guard = shared
                    .wake_cv
                    .wait(guard)
                    .unwrap_or_else(|e| e.into_inner());
            }
        };

        if do_reset {
            let mut st = shared.state.lock().unwrap_or_else(|e| e.into_inner());
            st.reset();
        }

        if let Some(req) = req {
            #[cfg(test)]
            {
                let hook = shared
                    .before_process_hook
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .clone();
                if let Some(h) = hook {
                    h(req.request_id);
                }
            }

            let output = {
                let mut st = shared.state.lock().unwrap_or_else(|e| e.into_inner());
                st.process(&req)
            };
            let req_id = output.request_id;
            let egui_ctx = req.egui_ctx.clone();
            let _ = out_tx.send(output);
            if let Some(ctx) = egui_ctx {
                ctx.request_repaint();
            }

            let mut guard = shared.slot.lock().unwrap_or_else(|e| e.into_inner());
            guard.completed_request_id = guard.completed_request_id.max(req_id);
            if guard.pending.is_none() && !guard.reset_requested {
                guard.busy = false;
            }
            shared.idle_cv.notify_all();
        } else {
            let mut guard = shared.slot.lock().unwrap_or_else(|e| e.into_inner());
            if guard.pending.is_none() && !guard.reset_requested {
                guard.busy = false;
                shared.idle_cv.notify_all();
            }
        }
    }
}

impl SimulationPanelState {
    /// Spawns a non-blocking `rayon` background job to run stock simulation & 5-rule crash checks.
    pub fn trigger_simulation(&mut self, args: SimTriggerArgs<'_>) {
        let SimTriggerArgs {
            project,
            setup_idx,
            cad_doc,
            cad_revision,
            cache,
            project_dir,
            egui_ctx,
            logs,
        } = args;
        self.repaint_ctx = Some(egui_ctx.clone());
        let Some(setup) = project.setups.get(setup_idx) else {
            return;
        };
        let resolved =
            match setup.resolve_with_sketches(&cad_doc, &project.sketches, project.display_units) {
                Ok(r) => r,
                Err(e) => {
                    logs.push(format!("Cannot simulate Setup '{}': {e}", setup.name));
                    return;
                }
            };

        let ready = collect_ready_setup_operations(project, setup_idx, cache, cad_revision);
        let owned_ops: Vec<OwnedSimOp> = ready
            .into_iter()
            .map(|r| OwnedSimOp {
                op: r.op.clone(),
                tool: r.tool.clone(),
                toolpath: r.toolpath,
            })
            .collect();

        if owned_ops.is_empty() {
            logs.push(format!(
                "Cannot simulate Setup '{}': generate clean toolpaths first (Ctrl+G).",
                setup.name
            ));
            return;
        }

        self.cancel_in_flight();
        let run_id = self.next_run_id;
        self.next_run_id += 1;
        let cancel_flag = Arc::new(AtomicBool::new(false));
        let progress_repaint_ctx = egui_ctx.clone();
        let progress_tracker = Arc::new(SimProgressTracker::with_callback(Arc::new(move || {
            progress_repaint_ctx.request_repaint();
        })));
        self.in_flight = Some(InFlightSim {
            run_id,
            started_at: Instant::now(),
            cancel: Arc::clone(&cancel_flag),
            progress: Arc::clone(&progress_tracker),
        });

        let signature = self.compute_setup_sim_signature(project, setup_idx, cache, cad_revision);
        let engine = self.engine;
        let backend_choice = self.backend_choice;
        let resolution = self.resolution;
        let verify_config = self.verify_config;
        let target_bodies = setup.target_bodies.clone();
        let setup_for_fixtures = setup.clone();
        let display_units = project.display_units;
        let needs_imported_fixtures = setup
            .fixtures
            .iter()
            .any(|f| f.enabled && matches!(f.kind, FixtureKind::ImportedModel { .. }));
        let fixture_docs: BTreeMap<String, CadDocument> = if needs_imported_fixtures {
            project.load_fixture_cad_documents(project_dir)
        } else {
            BTreeMap::new()
        };
        let kin = resolve_setup_kinematics(project, setup_idx, project_dir);

        let has_carryover = setup.carryover_from_setup_id.is_some();
        let carryover_bundle = if has_carryover {
            Some((
                project.clone(),
                cache.clone(),
                project_dir.map(std::path::Path::to_path_buf),
            ))
        } else {
            None
        };

        logs.push(format!(
            "Simulating Setup '{}' via {} [{}] at {} resolution on background worker…",
            setup.name,
            engine.label(),
            backend_choice.label(),
            resolution.label_for_engine(engine)
        ));

        let tx = self.tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            if let Some((proj_owned, cache_owned, proj_dir_owned)) = carryover_bundle {
                // Multi-setup carryover path: resolves upstream Setup chain on this background
                // thread using isolated headless GPU compute (`GpuSimContext::shared_headless()`),
                // NEVER touching `eframe`'s presentation device!
                let res = simulate_project_setup(&SetupSimRequest {
                    project: &proj_owned,
                    setup_idx,
                    cad_doc: &cad_doc,
                    cad_revision,
                    cache: &cache_owned,
                    project_dir: proj_dir_owned.as_deref(),
                    kinematics: kin,
                    resolution,
                    engine,
                    backend_choice,
                    verify_config,
                    cancel: Some(&cancel_flag),
                    progress: Some(progress_tracker.full_scope()),
                });
                let mapped = match res {
                    Ok(sim) => Ok(Arc::new(sim)),
                    Err(e) if e.contains("cancelled") => return,
                    Err(e) => Err(e),
                };
                let _ = tx.send(SimWorkerMsg::Finished {
                    run_id,
                    setup_idx,
                    signature,
                    engine,
                    elapsed: t0.elapsed(),
                    result: mapped,
                });
                egui_ctx.request_repaint();
                return;
            }

            let refs: Vec<SimOperationInput<'_>> = owned_ops
                .iter()
                .map(|item| {
                    SimOperationInput::from_operation(&item.op, &item.tool, item.toolpath.as_ref())
                })
                .collect();

            let resolved_fixtures = if setup_for_fixtures.fixtures.iter().any(|f| f.enabled) {
                match resolve_setup_fixtures(&setup_for_fixtures, &resolved, display_units, |id| {
                    fixture_docs.get(id)
                }) {
                    Ok(f) => f,
                    Err(e) => {
                        let _ = tx.send(SimWorkerMsg::Finished {
                            run_id,
                            setup_idx,
                            signature,
                            engine,
                            elapsed: t0.elapsed(),
                            result: Err(e.to_string()),
                        });
                        egui_ctx.request_repaint();
                        return;
                    }
                }
            } else {
                Vec::new()
            };

            let mut input = SetupSimulationInput::new(&cad_doc, &resolved, &refs);
            input.target_bodies = &target_bodies;
            input.fixtures = &resolved_fixtures;
            input.resolution = resolution;
            input.engine = engine;
            input.backend_choice = backend_choice;
            input.kinematics = kin;
            input.verify_config = verify_config;
            input.progress = Some(progress_tracker.full_scope());
            input.cancel = Some(&cancel_flag);
            // Always run full-setup background GPU simulation on an isolated headless compute
            // `wgpu::Device` (`GpuSimContext::shared_headless()`), NEVER on `eframe`'s presentation
            // device—otherwise resizing the window mid-simulation causes `Surface::configure` to
            // fail with `ConfigureSurfaceError::GpuWaitTimeout`.
            let res = run_setup_simulation(&input);
            let mapped = match res {
                Ok(sim) => Ok(Arc::new(sim)),
                Err(SimError::Cancelled) => return,
                Err(e) => Err(e.to_string()),
            };
            let _ = tx.send(SimWorkerMsg::Finished {
                run_id,
                setup_idx,
                signature,
                engine,
                elapsed: t0.elapsed(),
                result: mapped,
            });
            egui_ctx.request_repaint();
        });
    }

    /// Spawns a non-blocking `rayon` background job to simulate every Setup in `project`
    /// sequentially (reusing Tri-Dexel carryover stock) and capture per-setup + cumulative
    /// end-of-job metrology (`M89`).
    pub fn trigger_simulate_all_setups(&mut self, args: SimTriggerArgs<'_>) {
        let SimTriggerArgs {
            project,
            setup_idx,
            cad_doc,
            cad_revision,
            cache,
            project_dir,
            egui_ctx,
            logs,
        } = args;
        self.repaint_ctx = Some(egui_ctx.clone());
        if project.setups.is_empty() {
            return;
        }
        let any_ready = (0..project.setups.len())
            .any(|si| !collect_ready_setup_operations(project, si, cache, cad_revision).is_empty());
        if !any_ready {
            logs.push(
                "Cannot simulate all Setups: generate clean toolpaths first (Ctrl+G).".to_string(),
            );
            return;
        }

        self.cancel_in_flight();
        let run_id = self.next_run_id;
        self.next_run_id += 1;
        let cancel_flag = Arc::new(AtomicBool::new(false));
        let progress_repaint_ctx = egui_ctx.clone();
        let progress_tracker = Arc::new(SimProgressTracker::with_callback(Arc::new(move || {
            progress_repaint_ctx.request_repaint();
        })));
        self.in_flight = Some(InFlightSim {
            run_id,
            started_at: Instant::now(),
            cancel: Arc::clone(&cancel_flag),
            progress: Arc::clone(&progress_tracker),
        });

        let active_signature =
            self.compute_setup_sim_signature(project, setup_idx, cache, cad_revision);
        let engine = self.engine;
        let backend_choice = self.backend_choice;
        let resolution = self.resolution;
        let verify_config = self.verify_config;
        let proj_owned = project.clone();
        let cache_owned = cache.clone();
        let proj_dir_owned = project_dir.map(std::path::Path::to_path_buf);

        logs.push(format!(
            "Simulating all {} Setup(s) via {} [{}] at {} resolution on background worker...",
            proj_owned.setups.len(),
            engine.label(),
            backend_choice.label(),
            resolution.label_for_engine(engine),
        ));

        let tx = self.tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let res = simulate_all_setups(&AllSetupsSimRequest {
                project: &proj_owned,
                cad_doc: &cad_doc,
                cad_revision,
                cache: &cache_owned,
                project_dir: proj_dir_owned.as_deref(),
                resolution,
                engine,
                backend_choice,
                verify_config,
                cancel: Some(&cancel_flag),
                progress: Some(progress_tracker.full_scope()),
            });
            let mapped = match res {
                Ok(sims) => Ok(proj_owned
                    .setups
                    .iter()
                    .enumerate()
                    .zip(sims)
                    .map(|((idx, s), sim)| MultiSetupSimEntry {
                        setup_idx: idx,
                        setup_id: s.id.clone(),
                        setup_name: s.name.clone(),
                        result: Arc::new(sim),
                    })
                    .collect()),
                Err(e) if e.contains("cancelled") => return,
                Err(e) => Err(e),
            };
            let _ = tx.send(SimWorkerMsg::AllSetupsFinished {
                run_id,
                active_setup_idx: setup_idx,
                active_signature,
                engine,
                resolution,
                elapsed: t0.elapsed(),
                result: mapped,
            });
            egui_ctx.request_repaint();
        });
    }

    /// Polls background simulation and stock-remesh completion messages without blocking.
    pub fn poll_worker(&mut self, logs: &mut Vec<String>) -> bool {
        let mut completed_new = false;
        while let Ok(msg) = self.rx.try_recv() {
            match msg {
                SimWorkerMsg::Finished {
                    run_id,
                    setup_idx,
                    signature,
                    engine,
                    elapsed,
                    result,
                } => {
                    if self
                        .in_flight
                        .as_ref()
                        .is_some_and(|job| job.run_id == run_id)
                    {
                        self.in_flight = None;
                    } else {
                        continue;
                    }
                    let wall = format_exec_duration(elapsed);
                    match result {
                        Ok(sim) => {
                            let crit = sim.critical_error_count();
                            let warn = sim.warning_count();
                            let backend_tag = if sim.active_backend.is_gpu {
                                format!(
                                    "GPU/{} · {}",
                                    sim.active_backend.backend_api, sim.active_backend.adapter_name
                                )
                            } else {
                                "CPU Exact".to_string()
                            };
                            logs.push(format!(
                                "Simulation complete in {wall} ({}, {backend_tag}) — Cycle {}, Removed {:.0} mm^3 ({crit} critical error(s), {warn} warning(s)).",
                                sim.engine.label(),
                                format_cycle_time_hms(sim.timeline.total_time_sec),
                                sim.removed_volume_mm3
                            ));
                            self.current_time_sec = sim.timeline.total_time_sec;
                            self.is_playing = false;
                            self.active_run_id = run_id;
                            self.active_setup_idx = setup_idx;
                            self.active_signature = signature;
                            self.active_result = Some(sim);
                            self.clear_stock_display_cache();
                            completed_new = true;
                        }
                        Err(err) => {
                            logs.push(format!(
                                "ERROR during {} simulation after {wall}: {err}",
                                engine.label()
                            ));
                        }
                    }
                }
                SimWorkerMsg::AllSetupsFinished {
                    run_id,
                    active_setup_idx,
                    active_signature,
                    engine,
                    resolution,
                    elapsed,
                    result,
                } => {
                    if self
                        .in_flight
                        .as_ref()
                        .is_some_and(|job| job.run_id == run_id)
                    {
                        self.in_flight = None;
                    } else {
                        continue;
                    }
                    let wall = format_exec_duration(elapsed);
                    match result {
                        Ok(entries) => {
                            let total_crit: usize = entries
                                .iter()
                                .map(|e| e.result.critical_error_count())
                                .sum();
                            let total_warn: usize =
                                entries.iter().map(|e| e.result.warning_count()).sum();
                            let total_time: f64 = entries
                                .iter()
                                .map(|e| e.result.timeline.total_time_sec)
                                .sum();
                            let tol_mm = self.heatmap_tolerance_mm.clamp(1e-4, 50.0);
                            let cumulative_error_report = entries.iter().rev().find_map(|e| {
                                let base = e.result.error_report.as_ref()?;
                                if (base.tolerance_mm - tol_mm).abs() <= 1e-6 {
                                    Some(base.clone())
                                } else {
                                    e.result
                                        .compute_error_report_at_tolerance(tol_mm)
                                        .or_else(|| Some(base.clone()))
                                }
                            });
                            logs.push(format!(
                                "Simulated all {} Setup(s) in {wall} ({}) — Total Cycle {}, {total_crit} critical error(s), {total_warn} warning(s).",
                                entries.len(),
                                engine.label(),
                                format_cycle_time_hms(total_time),
                            ));
                            if let Some(active_entry) = entries
                                .iter()
                                .find(|e| e.setup_idx == active_setup_idx)
                                .or_else(|| entries.last())
                            {
                                self.current_time_sec = active_entry.result.timeline.total_time_sec;
                                self.is_playing = false;
                                self.active_run_id = run_id;
                                self.active_setup_idx = active_entry.setup_idx;
                                self.active_signature = active_signature;
                                self.active_result = Some(Arc::clone(&active_entry.result));
                                self.clear_stock_display_cache();
                            }
                            self.multi_setup_summary = Some(MultiSetupSimSummary {
                                engine,
                                resolution,
                                elapsed,
                                setups: entries,
                                cumulative_error_report,
                            });
                            completed_new = true;
                        }
                        Err(err) => {
                            logs.push(format!(
                                "ERROR during multi-setup {} simulation after {wall}: {err}",
                                engine.label()
                            ));
                        }
                    }
                }
            }
        }
        let _ = self.poll_remesh_outputs();
        completed_new
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::sync::{Arc, Barrier, Mutex};

    use chip_cad::CadDocument;
    use chip_cam::{
        BoxAnchor27, MotionRole, Setup, StockConfig, ToolpathBuilder, WcsOriginMode, WorkOffsetSlot,
    };
    use chip_math::{DVec3, DisplayUnit, ParamLength};
    use chip_project::ProjectDocument;
    use chip_sim::{
        MachineKinematicsConfig, SetupSimulationInput, SimOperationInput, SimResolution,
        SimVerificationConfig, run_setup_simulation,
    };
    use chip_tools::default_starter_tool_library;

    use crate::{
        panels::simulation::{SimulationPanelState, worker::SimWorkerMsg},
        viewport::SimStockRenderSource,
    };

    fn build_slot_sim() -> (ProjectDocument, Arc<chip_sim::SetupSimulationResult>) {
        let mut project = ProjectDocument::new("Remesh Coalesce Test");
        let lib = default_starter_tool_library();
        project.tools = lib.tools.clone();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s1", "Setup 1", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(100.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::CenterZ,
        };
        let doc = CadDocument::empty();
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let mut b = ToolpathBuilder::new("op_slot");
        b.rapid_to(DVec3::new(-40.0, 0.0, 10.0));
        b.linear_to(DVec3::new(-40.0, 0.0, -5.0), 300.0, MotionRole::Plunge);
        b.linear_to(DVec3::new(40.0, 0.0, -5.0), 600.0, MotionRole::Cut);
        b.rapid_to(DVec3::new(40.0, 0.0, 10.0));
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_slot",
                op_name: "Long Slot Cut",
                tool: flat_6mm,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        })
        .unwrap();

        (project, Arc::new(sim))
    }

    #[test]
    fn background_remesh_coalesces_rapid_scrubs_skips_stale_frames_and_advances_monotonically() {
        let (project, sim) = build_slot_sim();
        let cut_span = &sim.timeline.motion_spans[1];
        let cut_start = cut_span.t_start_sec;
        let cut_dur = cut_span.t_end_sec - cut_start;
        assert!(cut_dur > 4.0);

        let mut panel = SimulationPanelState {
            active_run_id: 1,
            active_setup_idx: 0,
            active_result: Some(Arc::clone(&sim)),
            ..Default::default()
        };

        let processed_ids = Arc::new(Mutex::new(Vec::<u64>::new()));
        let req1_entered = Arc::new(Barrier::new(2));
        let req1_release = Arc::new(Barrier::new(2));
        {
            let processed_ids = Arc::clone(&processed_ids);
            let req1_entered = Arc::clone(&req1_entered);
            let req1_release = Arc::clone(&req1_release);
            panel
                .remesh_worker
                .set_before_process_hook(Some(Arc::new(move |req_id| {
                    processed_ids.lock().unwrap().push(req_id);
                    if req_id == 1 {
                        req1_entered.wait();
                        req1_release.wait();
                    }
                })));
        }

        // Frame 1: starts request #1 on the background worker and holds it at the barrier.
        panel.current_time_sec = cut_start + cut_dur * 0.10;
        let vis_initial = panel.resolve_viewport_visuals(&project, 0);
        assert!(vis_initial.stock.is_none());
        req1_entered.wait();

        // While request #1 is busy, simulate 3 rapid scrubber drags (requests #2, #3, #4).
        // Requests #2 and #3 must be coalesced away so only #4 runs after #1!
        for frac in [0.30_f64, 0.55, 0.90] {
            panel.current_time_sec = cut_start + cut_dur * frac;
            let _ = panel.resolve_viewport_visuals(&project, 0);
        }

        // Release request #1 and wait for the background worker to settle.
        req1_release.wait();
        panel.flush_remesh_sync();

        let executed = processed_ids.lock().unwrap().clone();
        assert_eq!(
            executed,
            vec![1, 4],
            "intermediate scrub requests #2 and #3 must be coalesced away (executed={executed:?})"
        );

        let vis_final = panel.resolve_viewport_visuals(&project, 0);
        let SimStockRenderSource::CpuMesh {
            revision: rev_final,
            mesh: mesh_final,
            ..
        } = vis_final
            .stock
            .expect("coalesced background remesh must publish SimStockRenderSource::CpuMesh")
        else {
            panic!("expected SimStockRenderSource::CpuMesh");
        };
        assert!(
            rev_final >= 2,
            "revisions must increment monotonically across completed remeshes (got {rev_final})"
        );
        assert!(!mesh_final.vertices.is_empty());

        // Verify the worker stock actually reflects the latest scrub timestamp (frac = 0.90),
        // not the stale intermediate frames!
        let (_, _, tip_wcs) = vis_final.cutter.expect("cutter present");
        panel.remesh_worker.inspect_working_stock(|stock| {
            let hf = stock.expect("working stock present");
            let z_at_tip = hf.sample_stock_z(tip_wcs.x, 0.0).unwrap();
            assert!(
                (z_at_tip - (-5.0)).abs() < 0.05,
                "stock at latest scrub tip x={:.2} must be machined to -5.0, got {z_at_tip:.2}",
                tip_wcs.x
            );
        });

        // A follow-up color-mode toggle produces a strictly higher revision.
        panel.color_by_operation = false;
        let vis_metal = panel.resolve_viewport_visuals_sync(&project, 0);
        let rev_metal = vis_metal.stock.expect("stock render source").revision();
        assert!(
            rev_metal > rev_final,
            "color mode remesh must advance revision monotonically ({rev_metal} > {rev_final})"
        );
    }

    #[test]
    fn reset_for_new_project_clears_visuals_and_ignores_stale_worker_results() {
        let (project, sim) = build_slot_sim();

        let mut panel = SimulationPanelState {
            active_run_id: 7,
            active_setup_idx: 0,
            active_result: Some(Arc::clone(&sim)),
            is_playing: true,
            current_time_sec: 1.0,
            ..Default::default()
        };
        let before = panel.resolve_viewport_visuals_sync(&project, 0);
        assert!(before.stock.is_some());

        panel.reset_for_new_project();
        assert!(panel.active_result.is_none());
        assert!(!panel.is_playing);
        assert_eq!(panel.current_time_sec, 0.0);
        let after = panel.resolve_viewport_visuals_sync(&project, 0);
        assert!(after.stock.is_none());
        assert!(after.cutter.is_none());

        // A stale worker completion arriving after reset_for_new_project must be ignored.
        let _ = panel.tx.send(SimWorkerMsg::Finished {
            run_id: 7,
            setup_idx: 0,
            signature: 42,
            engine: chip_sim::SimEngineKind::Heightmap,
            elapsed: std::time::Duration::from_millis(5),
            result: Ok(sim),
        });
        let mut logs = Vec::new();
        assert!(!panel.poll_worker(&mut logs));
        assert!(panel.active_result.is_none());
        assert!(logs.is_empty());
    }

    #[test]
    fn in_flight_progress_reports_live_updates_and_clears_on_cancel_or_finish() {
        use std::{
            sync::atomic::AtomicBool,
            time::{Duration, Instant},
        };

        use chip_sim::{SimEngineKind, SimProgressStage, SimProgressTracker};

        use crate::panels::simulation::InFlightSim;

        let (_project, sim) = build_slot_sim();
        let mut panel = SimulationPanelState::default();
        assert!(panel.in_flight_progress().is_none());
        assert!(panel.cancel_in_flight().is_none());

        let tracker = Arc::new(SimProgressTracker::new());
        panel.in_flight = Some(InFlightSim {
            run_id: 9,
            started_at: Instant::now(),
            cancel: Arc::new(AtomicBool::new(false)),
            progress: Arc::clone(&tracker),
        });
        assert!(panel.is_busy());
        assert_eq!(panel.in_flight_progress().unwrap().percent_rounded(), 0);

        tracker
            .full_scope()
            .report(0.42, SimProgressStage::Sweeping);
        let snap = panel.in_flight_progress().unwrap();
        assert_eq!(snap.percent_rounded(), 42);
        assert_eq!(snap.stage, SimProgressStage::Sweeping);

        // Cancelling clears progress immediately and reports elapsed duration.
        assert!(panel.cancel_in_flight().is_some());
        assert!(!panel.is_busy());
        assert!(panel.in_flight_progress().is_none());

        // Completing a run via worker channel clears in-flight progress cleanly.
        let tracker2 = Arc::new(SimProgressTracker::new());
        panel.in_flight = Some(InFlightSim {
            run_id: 10,
            started_at: Instant::now(),
            cancel: Arc::new(AtomicBool::new(false)),
            progress: Arc::clone(&tracker2),
        });
        tracker2
            .full_scope()
            .report(1.0, SimProgressStage::Complete);
        let _ = panel.tx.send(SimWorkerMsg::Finished {
            run_id: 10,
            setup_idx: 0,
            signature: 99,
            engine: SimEngineKind::Heightmap,
            elapsed: Duration::from_millis(12),
            result: Ok(sim),
        });
        let mut logs = Vec::new();
        assert!(panel.poll_worker(&mut logs));
        assert!(!panel.is_busy());
        assert!(panel.in_flight_progress().is_none());
        assert_eq!(logs.len(), 1);
        assert!(
            logs[0].starts_with("Simulation complete in 12ms (Heightmap (Fast), CPU Exact)"),
            "unexpected completion log: {}",
            logs[0]
        );
    }

    #[test]
    fn poll_worker_logs_formatted_wall_clock_time_for_completion_and_failure() {
        use std::{
            sync::atomic::AtomicBool,
            time::{Duration, Instant},
        };

        use chip_sim::{SimEngineKind, SimProgressTracker};

        use crate::panels::simulation::InFlightSim;

        let (_project, sim) = build_slot_sim();
        // Multi-second completion log formatting
        let mut panel = SimulationPanelState {
            in_flight: Some(InFlightSim {
                run_id: 1,
                started_at: Instant::now(),
                cancel: Arc::new(AtomicBool::new(false)),
                progress: Arc::new(SimProgressTracker::new()),
            }),
            ..Default::default()
        };
        let _ = panel.tx.send(SimWorkerMsg::Finished {
            run_id: 1,
            setup_idx: 0,
            signature: 11,
            engine: SimEngineKind::Heightmap,
            elapsed: Duration::from_millis(2_340),
            result: Ok(sim),
        });
        let mut logs = Vec::new();
        assert!(panel.poll_worker(&mut logs));
        assert_eq!(logs.len(), 1);
        assert!(
            logs[0]
                .starts_with("Simulation complete in 2.34s (Heightmap (Fast), CPU Exact) — Cycle "),
            "unexpected completion log: {}",
            logs[0]
        );
        assert!(logs[0].contains("critical error(s)"));
        assert!(logs[0].contains("warning(s)"));

        // Failure log formatting preserves engine label and elapsed execution time
        panel.in_flight = Some(InFlightSim {
            run_id: 2,
            started_at: Instant::now(),
            cancel: Arc::new(AtomicBool::new(false)),
            progress: Arc::new(SimProgressTracker::new()),
        });
        let _ = panel.tx.send(SimWorkerMsg::Finished {
            run_id: 2,
            setup_idx: 0,
            signature: 12,
            engine: SimEngineKind::TriDexel,
            elapsed: Duration::from_millis(415),
            result: Err("invalid fixture geometry".to_string()),
        });
        assert!(!panel.poll_worker(&mut logs));
        assert_eq!(logs.len(), 2);
        assert_eq!(
            logs[1],
            "ERROR during Tri-Dexel (Volumetric 3D) simulation after 415ms: invalid fixture geometry"
        );
    }
}
