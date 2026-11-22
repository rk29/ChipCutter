use std::{
    sync::{Arc, Condvar, Mutex, atomic::AtomicBool},
    thread::{self, JoinHandle},
    time::Instant,
};

use chip_cam::{GeneratedToolpath, Operation};
use chip_project::{collect_ready_setup_operations, resolve_kinematics_for_key};
use chip_sim::{
    ScrubSnapshot, SetupSimulationInput, SetupSimulationResult, SimError, SimOperationInput,
    SimStockColorMode, SimStockMesh, SimVerificationConfig, SimulatedStock,
    SimulatedStockPlaybackExt, StockPlaybackCursor, format_cycle_time_hms,
    run_setup_simulation_with_engine,
};
use chip_tools::Tool;
use crossbeam_channel::{Receiver, Sender, unbounded};

use super::{SimTriggerArgs, SimulationPanelState};

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
        elapsed_ms: u128,
        result: Result<Arc<SetupSimulationResult>, String>,
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
    pub(super) meta: StockMeshMeta,
    pub(super) egui_ctx: Option<egui::Context>,
}

/// Completed remesh response produced by [`StockRemeshWorker`].
pub(super) struct StockRemeshOutput {
    pub(super) request_id: u64,
    pub(super) run_id: u64,
    pub(super) result_ptr: usize,
    pub(super) meta: StockMeshMeta,
    /// `Some(mesh)` when stock geometry or shading mode changed; `None` when the requested
    /// timestamp required no stock mutation or color-mode remesh (e.g. air rapid / rotary slew).
    pub(super) mesh: Option<Arc<SimStockMesh>>,
}

#[derive(Default)]
struct RemeshWorkerState {
    working_stock_identity: Option<(u64, usize)>,
    working_stock_cursor: Option<StockPlaybackCursor>,
    working_stock: Option<SimulatedStock>,
    last_meshed_meta: Option<StockMeshMeta>,
    has_emitted_mesh: bool,
}

impl RemeshWorkerState {
    fn reset(&mut self) {
        self.working_stock_identity = None;
        self.working_stock_cursor = None;
        self.working_stock = None;
        self.last_meshed_meta = None;
        self.has_emitted_mesh = false;
    }

    fn process(&mut self, req: &StockRemeshRequest) -> StockRemeshOutput {
        let run_identity = (req.run_id, req.result_ptr);
        if self.working_stock_identity != Some(run_identity) {
            self.working_stock_identity = Some(run_identity);
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

        let mesh = if needs_remesh {
            let extracted = self
                .working_stock
                .as_ref()
                .map_or_else(SimStockMesh::default, |work| {
                    req.sim_result.extract_stock_mesh(work, req.color_mode)
                });
            self.last_meshed_meta = Some(req.meta);
            self.has_emitted_mesh = true;
            Some(Arc::new(extracted))
        } else {
            None
        };

        StockRemeshOutput {
            request_id: req.request_id,
            run_id: req.run_id,
            result_ptr: req.result_ptr,
            meta: req.meta,
            mesh,
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
        self.in_flight = Some((run_id, Arc::clone(&cancel_flag)));

        let signature = self.compute_setup_sim_signature(project, setup_idx, cache, cad_revision);
        let engine = self.engine;
        let resolution = self.resolution;
        let target_bodies = setup.target_bodies.clone();
        let kin = if let Some(mid) = project.machine_id.as_deref()
            && let Ok(machine) = chip_tools::resolve_machine_definition(mid, project_dir)
        {
            chip_sim::MachineKinematicsConfig::from_machine_spec(&machine.kinematics)
        } else {
            resolve_kinematics_for_key(&setup.post_profile, project_dir)
        };

        logs.push(format!(
            "Simulating Setup '{}' via {} at {} resolution on background worker…",
            setup.name,
            engine.label(),
            resolution.label_for_engine(engine)
        ));

        let tx = self.tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let refs: Vec<SimOperationInput<'_>> = owned_ops
                .iter()
                .map(|item| {
                    SimOperationInput::from_operation(&item.op, &item.tool, item.toolpath.as_ref())
                })
                .collect();

            let res = run_setup_simulation_with_engine(
                &SetupSimulationInput {
                    doc: &cad_doc,
                    setup: &resolved,
                    target_bodies: &target_bodies,
                    operations: &refs,
                    resolution,
                    kinematics: kin,
                    verify_config: SimVerificationConfig::default(),
                    cancel: Some(&cancel_flag),
                },
                engine,
            );
            let mapped = match res {
                Ok(sim) => Ok(Arc::new(sim)),
                Err(SimError::Cancelled) => return,
                Err(e) => Err(e.to_string()),
            };
            let _ = tx.send(SimWorkerMsg::Finished {
                run_id,
                setup_idx,
                signature,
                elapsed_ms: t0.elapsed().as_millis(),
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
                    elapsed_ms,
                    result,
                } => {
                    if self.in_flight.as_ref().is_some_and(|(id, _)| *id == run_id) {
                        self.in_flight = None;
                    } else {
                        continue;
                    }
                    match result {
                        Ok(sim) => {
                            let crit = sim.critical_error_count();
                            let warn = sim.warning_count();
                            logs.push(format!(
                                "Simulation completed in {elapsed_ms} ms — Cycle {}, Removed {:.0} mm^3 ({crit} critical error(s), {warn} warning(s)).",
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
                            logs.push(format!("ERROR during heightmap simulation: {err}"));
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
            cancel: None,
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
            .expect("coalesced background remesh must publish SimStockRenderSource::CpuMesh");
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
            elapsed_ms: 5,
            result: Ok(sim),
        });
        let mut logs = Vec::new();
        assert!(!panel.poll_worker(&mut logs));
        assert!(panel.active_result.is_none());
        assert!(logs.is_empty());
    }
}
