mod metrology_ui;
mod playback;
mod ui;
mod worker;

use std::{
    hash::{Hash as _, Hasher as _},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
};

use chip_cad::CadDocument;
use chip_math::DVec3;
use chip_project::{ProjectDocument, ToolpathCache};
#[cfg(feature = "gpu-compute")]
use chip_sim::{GpuSimContext, GpuStockRenderHandle};
use chip_sim::{
    SetupSimulationResult, SimEngineKind, SimResolution, SimStockColorMode, SimStockMesh,
    SimToolMesh, StockErrorReport,
};
use chip_tools::ToolId;
use crossbeam_channel::{Receiver, Sender, unbounded};

use crate::viewport::SimStockRenderSource;

use self::worker::{SimWorkerMsg, StockMeshMeta, StockRemeshKey, StockRemeshWorker};

/// Interactive UI & background worker state for `[Simulation Timeline]` (`06-simulation.md`).
pub struct SimulationPanelState {
    pub engine: SimEngineKind,
    pub resolution: SimResolution,
    pub show_machined_stock: bool,
    pub color_by_operation: bool,
    pub show_error_heatmap: bool,
    pub heatmap_tolerance_mm: f64,
    pub heatmap_max_scale_mm: f64,
    pub show_cutter_assembly: bool,
    pub trail_up_to_cutter: bool,
    pub scrub_by_distance: bool,
    pub is_playing: bool,
    pub speed_multiplier: f64,
    pub current_time_sec: f64,

    next_run_id: u64,
    in_flight: Option<(u64, Arc<AtomicBool>)>,
    tx: Sender<SimWorkerMsg>,
    rx: Receiver<SimWorkerMsg>,

    active_run_id: u64,
    pub(crate) active_setup_idx: usize,
    active_signature: u64,
    pub active_result: Option<Arc<SetupSimulationResult>>,

    remesh_worker: StockRemeshWorker,
    next_remesh_request_id: u64,
    min_accept_remesh_request_id: u64,
    last_applied_remesh_request_id: u64,
    last_submitted_remesh_key: Option<StockRemeshKey>,
    cached_stock_run_identity: Option<(u64, usize)>,
    repaint_ctx: Option<egui::Context>,

    cached_error_report_key: Option<(u64, u32)>,
    cached_error_report: Option<StockErrorReport>,

    cached_stock_meta: Option<StockMeshMeta>,
    cached_stock_rev: u64,
    cached_stock_mesh: Option<Arc<SimStockMesh>>,
    #[cfg(feature = "gpu-compute")]
    gpu_ctx: Option<Arc<GpuSimContext>>,
    #[cfg(feature = "gpu-compute")]
    cached_gpu_handle: Option<Arc<GpuStockRenderHandle>>,

    cached_tool_key: Option<(ToolId, bool)>,
    cached_tool_rev: u64,
    cached_tool_mesh: Option<Arc<SimToolMesh>>,
    next_gpu_mesh_rev: u64,
}

/// Arguments for spawning a background stock simulation run.
pub struct SimTriggerArgs<'a> {
    pub project: &'a ProjectDocument,
    pub setup_idx: usize,
    pub cad_doc: Arc<CadDocument>,
    pub cad_revision: u64,
    pub cache: &'a ToolpathCache,
    pub project_dir: Option<&'a std::path::Path>,
    pub egui_ctx: egui::Context,
    pub logs: &'a mut Vec<String>,
}

/// Pass #6 GPU stock render source, revolved cutter assembly, and trail segment cap returned by
/// [`SimulationPanelState::resolve_viewport_visuals`] /
/// [`SimulationPanelState::resolve_viewport_visuals_sync`].
pub struct SimViewportVisuals {
    pub stock: Option<SimStockRenderSource>,
    pub cutter: Option<(u64, Arc<SimToolMesh>, DVec3)>,
    pub trail_limit: Option<usize>,
    pub rotary_deg: f64,
}

/// Context references passed to [`SimulationPanelState::show_ui`].
pub struct SimPanelCtx<'a> {
    pub project: &'a ProjectDocument,
    pub setup_idx: usize,
    pub cad_doc: Option<&'a Arc<CadDocument>>,
    pub cad_revision: u64,
    pub cache: &'a ToolpathCache,
    pub project_dir: Option<&'a std::path::Path>,
    pub logs: &'a mut Vec<String>,
}

/// Actions emitted by `[Simulation Timeline]` back to `ChipCutterApp`.
#[derive(Debug, Clone)]
pub enum SimulationUiAction {
    FocusDiagnosticsTab,
    CenterCameraWcs(DVec3),
}

impl Default for SimulationPanelState {
    fn default() -> Self {
        let (tx, rx) = unbounded();
        Self {
            engine: SimEngineKind::Heightmap,
            resolution: SimResolution::Standard,
            show_machined_stock: true,
            color_by_operation: true,
            show_error_heatmap: false,
            heatmap_tolerance_mm: 0.05,
            heatmap_max_scale_mm: 1.0,
            show_cutter_assembly: true,
            trail_up_to_cutter: false,
            scrub_by_distance: false,
            is_playing: false,
            speed_multiplier: 1.0,
            current_time_sec: 0.0,
            next_run_id: 1,
            in_flight: None,
            tx,
            rx,
            active_run_id: 0,
            active_setup_idx: 0,
            active_signature: 0,
            active_result: None,
            remesh_worker: StockRemeshWorker::new(),
            next_remesh_request_id: 1,
            min_accept_remesh_request_id: 1,
            last_applied_remesh_request_id: 0,
            last_submitted_remesh_key: None,
            cached_stock_run_identity: None,
            repaint_ctx: None,
            cached_error_report_key: None,
            cached_error_report: None,
            cached_stock_meta: None,
            cached_stock_rev: 0,
            cached_stock_mesh: None,
            #[cfg(feature = "gpu-compute")]
            gpu_ctx: None,
            #[cfg(feature = "gpu-compute")]
            cached_gpu_handle: None,
            cached_tool_key: None,
            cached_tool_rev: 0,
            cached_tool_mesh: None,
            next_gpu_mesh_rev: 1,
        }
    }
}

impl SimulationPanelState {
    pub fn with_engine(engine: SimEngineKind) -> Self {
        Self {
            engine,
            ..Self::default()
        }
    }

    #[cfg(feature = "gpu-compute")]
    pub fn with_engine_and_gpu(engine: SimEngineKind, gpu_ctx: Option<Arc<GpuSimContext>>) -> Self {
        let mut s = Self::with_engine(engine);
        s.set_gpu_context(gpu_ctx);
        s
    }

    #[cfg(feature = "gpu-compute")]
    pub fn set_gpu_context(&mut self, gpu_ctx: Option<Arc<GpuSimContext>>) {
        self.remesh_worker.set_gpu_context(gpu_ctx.clone());
        self.gpu_ctx = gpu_ctx;
    }

    #[inline]
    pub(crate) fn is_gpu_direct_active(&self) -> bool {
        #[cfg(feature = "gpu-compute")]
        {
            self.gpu_ctx.is_some()
                && self
                    .active_result
                    .as_ref()
                    .is_some_and(|r| r.active_backend.is_gpu)
        }
        #[cfg(not(feature = "gpu-compute"))]
        {
            false
        }
    }

    pub fn is_busy(&self) -> bool {
        self.in_flight.is_some()
    }

    /// Returns `true` while the background stock remesh worker has an active or queued scrub job.
    pub fn is_remeshing(&self) -> bool {
        self.remesh_worker.is_busy()
    }

    pub fn cancel_in_flight(&mut self) {
        if let Some((_, flag)) = self.in_flight.take() {
            flag.store(true, Ordering::Relaxed);
        }
    }

    /// Cancels any in-flight simulation worker and clears all cached simulation results,
    /// machined stock meshes, playback state, and cutter visuals when switching projects or models.
    pub fn reset_for_new_project(&mut self) {
        self.cancel_in_flight();
        while self.rx.try_recv().is_ok() {}
        self.is_playing = false;
        self.current_time_sec = 0.0;
        self.active_run_id = 0;
        self.active_setup_idx = 0;
        self.active_signature = 0;
        self.active_result = None;
        self.clear_stock_display_cache();
        self.cached_tool_key = None;
        self.cached_tool_mesh = None;
    }

    /// Computes a deterministic signature of `(setup_idx, resolution, clean toolpath hashes)`
    /// so we know immediately whether `active_result` matches the current Setup toolpaths!
    pub fn compute_setup_sim_signature(
        &self,
        project: &ProjectDocument,
        setup_idx: usize,
        cache: &ToolpathCache,
        cad_revision: u64,
    ) -> u64 {
        let mut h = std::collections::hash_map::DefaultHasher::new();
        cad_revision.hash(&mut h);
        setup_idx.hash(&mut h);
        self.engine.hash(&mut h);
        self.resolution.hash(&mut h);
        if let Some(setup) = project.setups.get(setup_idx) {
            setup.post_profile.hash(&mut h);
            for (o_idx, op) in setup.operations.iter().enumerate() {
                op.suppressed.hash(&mut h);
                if !op.suppressed
                    && let Some((hash_key, _)) =
                        cache.get_for_operation(project, setup_idx, o_idx, cad_revision)
                {
                    hash_key.0.hash(&mut h);
                }
            }
        }
        h.finish()
    }

    pub fn is_verified_up_to_date(
        &self,
        project: &ProjectDocument,
        setup_idx: usize,
        cache: &ToolpathCache,
        cad_revision: u64,
    ) -> bool {
        self.active_result.is_some()
            && self.active_setup_idx == setup_idx
            && self.active_signature
                == self.compute_setup_sim_signature(project, setup_idx, cache, cad_revision)
    }

    fn active_run_identity(&self) -> Option<(u64, usize)> {
        self.active_result
            .as_ref()
            .map(|res| (self.active_run_id, Arc::as_ptr(res) as usize))
    }

    fn clear_stock_display_cache(&mut self) {
        self.remesh_worker.reset();
        self.min_accept_remesh_request_id = self.next_remesh_request_id;
        self.last_applied_remesh_request_id = 0;
        self.last_submitted_remesh_key = None;
        self.cached_stock_run_identity = None;
        self.cached_error_report_key = None;
        self.cached_error_report = None;
        self.cached_stock_meta = None;
        self.cached_stock_mesh = None;
        #[cfg(feature = "gpu-compute")]
        {
            self.cached_gpu_handle = None;
        }
    }

    /// Returns the active [`StockErrorReport`] evaluated at `self.heatmap_tolerance_mm`,
    /// caching recomputation when the user drags the tolerance selector.
    pub fn active_error_report(&mut self) -> Option<&StockErrorReport> {
        let res = self.active_result.as_ref()?;
        let base = res.error_report.as_ref()?;
        let tol_mm = self.heatmap_tolerance_mm.clamp(1e-4, 50.0);
        if (base.tolerance_mm - tol_mm).abs() <= 1e-6 {
            return Some(base);
        }
        let tol_key = (tol_mm * 10_000.0).round() as u32;
        let key = (self.active_run_id, tol_key);
        if self.cached_error_report_key != Some(key) {
            self.cached_error_report = res.compute_error_report_at_tolerance(tol_mm);
            self.cached_error_report_key = Some(key);
        }
        self.cached_error_report.as_ref().or(Some(base))
    }

    fn active_stock_color_mode(&self) -> SimStockColorMode {
        // On CPU, defer expensive per-vertex heatmap shading during live animation playback;
        // on GPU direct rendering, Error Heatmap runs in < 0.1 ms in WGSL so it stays instant.
        if self.show_error_heatmap && (!self.is_playing || self.is_gpu_direct_active()) {
            let tol = self.heatmap_tolerance_mm.clamp(1e-4, 50.0);
            let max_scale = self.heatmap_max_scale_mm.max(tol * 2.0);
            SimStockColorMode::ErrorHeatmap {
                tolerance_mm: tol,
                max_scale_mm: max_scale,
            }
        } else if self.color_by_operation {
            SimStockColorMode::ByOperation
        } else {
            SimStockColorMode::Metallic
        }
    }
}
