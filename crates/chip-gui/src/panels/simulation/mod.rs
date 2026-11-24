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
    time::{Duration, Instant},
};

use chip_cad::CadDocument;
use chip_math::DVec3;
use chip_project::{ProjectDocument, ToolpathCache};
use chip_sim::{
    ActiveSimBackendInfo, SetupSimulationResult, SimComputeBackendChoice, SimDiagnostic,
    SimEngineKind, SimProgressSnapshot, SimProgressTracker, SimResolution, SimStockColorMode,
    SimStockMesh, SimTimingBreakdown, SimToolMesh, SimVerificationConfig, StockErrorReport,
    StockModelKind, format_cycle_time_hms,
};
#[cfg(feature = "gpu-compute")]
use chip_sim::{GpuSimContext, GpuStockRenderHandle};
use chip_tools::ToolId;
use crossbeam_channel::{Receiver, Sender, unbounded};
use serde::Serialize;

use crate::viewport::SimStockRenderSource;

use self::worker::{SimWorkerMsg, StockMeshMeta, StockRemeshKey, StockRemeshWorker};

struct InFlightSim {
    run_id: u64,
    started_at: Instant,
    cancel: Arc<AtomicBool>,
    progress: Arc<SimProgressTracker>,
}

/// Per-setup entry captured by a background **Simulate All Setups** run (`M89`).
#[derive(Debug, Clone)]
pub struct MultiSetupSimEntry {
    pub setup_idx: usize,
    pub setup_id: String,
    pub setup_name: String,
    pub result: Arc<SetupSimulationResult>,
}

/// Completed multi-setup simulation & verification chain summary (`M89`).
#[derive(Debug, Clone)]
pub struct MultiSetupSimSummary {
    pub engine: SimEngineKind,
    pub resolution: SimResolution,
    pub elapsed: Duration,
    pub setups: Vec<MultiSetupSimEntry>,
    pub cumulative_error_report: Option<StockErrorReport>,
}

/// Structured per-setup JSON metrology & crash verification payload exported by
/// **Export Metrology JSON...** (`M89`).
#[derive(Debug, Serialize)]
pub struct SetupMetrologyReportJson {
    pub setup_id: String,
    pub setup_name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub carried_over_from_setup: Option<String>,
    pub engine: SimEngineKind,
    pub stock_kind: String,
    pub resolution: SimResolution,
    pub backend: ActiveSimBackendInfo,
    pub timing: SimTimingBreakdown,
    pub total_time_sec: f64,
    pub total_time_hms: String,
    pub cutting_time_sec: f64,
    pub rapid_time_sec: f64,
    pub tool_change_time_sec: f64,
    pub initial_stock_volume_mm3: f64,
    pub remaining_stock_volume_mm3: f64,
    pub removed_volume_mm3: f64,
    pub critical_error_count: usize,
    pub warning_count: usize,
    pub diagnostics: Vec<SimDiagnostic>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error_report: Option<StockErrorReport>,
}

/// Top-level JSON metrology report emitted by **Export Metrology JSON...** (`M89`).
#[derive(Debug, Serialize)]
pub struct SimMetrologyReportJson {
    pub project_name: String,
    pub passed: bool,
    pub heatmap_tolerance_mm: f64,
    pub heatmap_max_scale_mm: f64,
    pub verify_config: SimVerificationConfig,
    pub setups: Vec<SetupMetrologyReportJson>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cumulative_error_report: Option<StockErrorReport>,
}

/// Interactive UI & background worker state for `[Simulation Timeline]` (`06-simulation.md`).
pub struct SimulationPanelState {
    pub engine: SimEngineKind,
    pub backend_choice: SimComputeBackendChoice,
    pub resolution: SimResolution,
    pub mesh_skinning: bool,
    pub show_machined_stock: bool,
    pub color_by_operation: bool,
    pub show_error_heatmap: bool,
    pub heatmap_tolerance_mm: f64,
    pub heatmap_max_scale_mm: f64,
    pub verify_config: SimVerificationConfig,
    pub show_cutter_assembly: bool,
    pub trail_up_to_cutter: bool,
    pub scrub_by_distance: bool,
    pub is_playing: bool,
    pub speed_multiplier: f64,
    pub current_time_sec: f64,

    next_run_id: u64,
    in_flight: Option<InFlightSim>,
    tx: Sender<SimWorkerMsg>,
    rx: Receiver<SimWorkerMsg>,

    active_run_id: u64,
    pub(crate) active_setup_idx: usize,
    active_signature: u64,
    pub active_result: Option<Arc<SetupSimulationResult>>,
    pub multi_setup_summary: Option<MultiSetupSimSummary>,

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
    OpenSimulationSettings,
}

impl Default for SimulationPanelState {
    fn default() -> Self {
        let (tx, rx) = unbounded();
        Self {
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Auto,
            resolution: SimResolution::Standard,
            mesh_skinning: true,
            show_machined_stock: true,
            color_by_operation: true,
            show_error_heatmap: false,
            heatmap_tolerance_mm: 0.05,
            heatmap_max_scale_mm: 1.0,
            verify_config: SimVerificationConfig::default(),
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
            multi_setup_summary: None,
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

    pub fn with_prefs(engine: SimEngineKind, backend_choice: SimComputeBackendChoice) -> Self {
        Self {
            engine,
            backend_choice,
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
    pub fn with_prefs_and_gpu(
        engine: SimEngineKind,
        backend_choice: SimComputeBackendChoice,
        gpu_ctx: Option<Arc<GpuSimContext>>,
    ) -> Self {
        let mut s = Self::with_prefs(engine, backend_choice);
        s.set_gpu_context(gpu_ctx);
        s
    }

    #[cfg(feature = "gpu-compute")]
    pub fn set_gpu_context(&mut self, gpu_ctx: Option<Arc<GpuSimContext>>) {
        // Keep `eframe`'s presentation `wgpu::Device` strictly main-thread-only: never hand it to
        // background worker threads, because `wgpu-core`'s `Surface::configure` requires the
        // presentation device queue to be completely empty on window resize (otherwise concurrent
        // background `queue.submit` calls fail with `ConfigureSurfaceError::GpuWaitTimeout`).
        self.gpu_ctx = gpu_ctx;
    }

    #[inline]
    pub fn has_gpu_context(&self) -> bool {
        #[cfg(feature = "gpu-compute")]
        {
            self.gpu_ctx.is_some()
        }
        #[cfg(not(feature = "gpu-compute"))]
        {
            false
        }
    }

    #[inline]
    pub fn gpu_adapter_info(&self) -> Option<&ActiveSimBackendInfo> {
        #[cfg(feature = "gpu-compute")]
        {
            self.gpu_ctx.as_ref().map(|ctx| ctx.adapter_info())
        }
        #[cfg(not(feature = "gpu-compute"))]
        {
            None
        }
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

    /// Returns live background simulation progress (`0.0..=1.0`, `0.0..=100.0%`, stage) while running.
    pub fn in_flight_progress(&self) -> Option<SimProgressSnapshot> {
        self.in_flight.as_ref().map(|job| job.progress.snapshot())
    }

    /// Returns `true` while the background stock remesh worker has an active or queued scrub job.
    pub fn is_remeshing(&self) -> bool {
        self.remesh_worker.is_busy()
    }

    pub fn cancel_in_flight(&mut self) -> Option<Duration> {
        if let Some(job) = self.in_flight.take() {
            job.cancel.store(true, Ordering::Relaxed);
            Some(job.started_at.elapsed())
        } else {
            None
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
        self.multi_setup_summary = None;
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
        self.backend_choice.hash(&mut h);
        self.resolution.hash(&mut h);
        self.verify_config.gouge_tol_mm.to_bits().hash(&mut h);
        self.verify_config.rapid_crash_tol_mm.to_bits().hash(&mut h);
        self.verify_config
            .radial_wall_skin_mm
            .to_bits()
            .hash(&mut h);
        if let Some(setup) = project.setups.get(setup_idx) {
            setup.post_profile.hash(&mut h);
            setup.auto_avoid_fixtures_on_rapids.hash(&mut h);
            setup.carryover_from_setup_id.hash(&mut h);
            if let Some(ref src_id) = setup.carryover_from_setup_id
                && let Some(src_idx) = project.setups[..setup_idx]
                    .iter()
                    .position(|s| &s.id == src_id)
            {
                chip_project::compute_setup_carryover_digest(project, src_idx, cad_revision)
                    .hash(&mut h);
            }
            for fix in &setup.fixtures {
                if fix.enabled {
                    fix.id.hash(&mut h);
                    fix.clearance_margin.value_mm.to_bits().hash(&mut h);
                    fix.allowed_overcut.value_mm.to_bits().hash(&mut h);
                }
            }
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

    /// Serializes the active single-setup simulation or completed multi-setup batch summary
    /// into a structured pretty-printed JSON metrology & crash report (`M89`).
    pub fn build_metrology_report_json(
        &mut self,
        project: &ProjectDocument,
    ) -> Result<String, String> {
        let tol_mm = self.heatmap_tolerance_mm.clamp(1e-4, 50.0);
        let max_scale_mm = self.heatmap_max_scale_mm.clamp(0.05, 20.0);

        let build_setup_entry = |setup_id: &str,
                                 setup_name: &str,
                                 sim: &SetupSimulationResult|
         -> SetupMetrologyReportJson {
            let err_report = sim.error_report.as_ref().map(|base| {
                if (base.tolerance_mm - tol_mm).abs() <= 1e-6 {
                    base.clone()
                } else {
                    sim.compute_error_report_at_tolerance(tol_mm)
                        .unwrap_or_else(|| base.clone())
                }
            });
            let stock_kind = match sim.stock_kind() {
                StockModelKind::Heightfield25D => "heightfield",
                StockModelKind::TriDexel3D => "tri_dexel",
                StockModelKind::Cylindrical4Axis(_) => "cylindrical",
            };
            SetupMetrologyReportJson {
                setup_id: setup_id.to_string(),
                setup_name: setup_name.to_string(),
                carried_over_from_setup: sim.carried_over_from_setup.clone(),
                engine: sim.engine,
                stock_kind: stock_kind.to_string(),
                resolution: sim.resolution,
                backend: sim.active_backend.clone(),
                timing: sim.timing,
                total_time_sec: sim.timeline.total_time_sec,
                total_time_hms: format_cycle_time_hms(sim.timeline.total_time_sec),
                cutting_time_sec: sim.timeline.total_cutting_time_sec,
                rapid_time_sec: sim.timeline.total_rapid_time_sec,
                tool_change_time_sec: sim.timeline.total_tool_change_time_sec,
                initial_stock_volume_mm3: sim.initial_stock_volume_mm3,
                remaining_stock_volume_mm3: sim.remaining_stock_volume_mm3,
                removed_volume_mm3: sim.removed_volume_mm3,
                critical_error_count: sim.critical_error_count(),
                warning_count: sim.warning_count(),
                diagnostics: sim.diagnostics.clone(),
                error_report: err_report,
            }
        };

        let (setups, cumulative_error_report) = if let Some(summary) = &self.multi_setup_summary {
            let entries: Vec<SetupMetrologyReportJson> = summary
                .setups
                .iter()
                .map(|s| build_setup_entry(&s.setup_id, &s.setup_name, s.result.as_ref()))
                .collect();
            let cumulative = entries.iter().rev().find_map(|e| e.error_report.clone());
            (entries, cumulative)
        } else if let Some(res) = self.active_result.clone() {
            let (sid, sname) = project.setups.get(self.active_setup_idx).map_or_else(
                || ("setup_1".to_string(), "Active Setup".to_string()),
                |s| (s.id.clone(), s.name.clone()),
            );
            let entry = build_setup_entry(&sid, &sname, res.as_ref());
            let cum = entry.error_report.clone();
            (vec![entry], cum)
        } else {
            return Err("No simulation result available to export.".to_string());
        };

        let passed = setups.iter().all(|s| s.critical_error_count == 0);
        let report = SimMetrologyReportJson {
            project_name: project.project_name.clone(),
            passed,
            heatmap_tolerance_mm: tol_mm,
            heatmap_max_scale_mm: max_scale_mm,
            verify_config: self.verify_config,
            setups,
            cumulative_error_report,
        };
        serde_json::to_string_pretty(&report).map_err(|e| format!("JSON serialization error: {e}"))
    }

    /// Prompts for a `.json` destination path and writes the metrology & crash verification report.
    pub fn export_metrology_json_dialog(
        &mut self,
        project: &ProjectDocument,
        logs: &mut Vec<String>,
    ) {
        let json = match self.build_metrology_report_json(project) {
            Ok(s) => s,
            Err(err) => {
                logs.push(format!("ERROR exporting metrology JSON: {err}"));
                return;
            }
        };
        let safe_stem: String = project
            .project_name
            .chars()
            .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
            .collect();
        let default_name = if safe_stem.trim_matches('_').is_empty() {
            "simulation_metrology.json".to_string()
        } else {
            format!("{}_metrology.json", safe_stem.trim_matches('_'))
        };
        if let Some(dest) = rfd::FileDialog::new()
            .add_filter("Simulation Metrology JSON", &["json"])
            .set_file_name(default_name)
            .save_file()
        {
            match std::fs::write(&dest, json) {
                Ok(()) => logs.push(format!(
                    "Exported simulation metrology JSON report to '{}'.",
                    dest.display()
                )),
                Err(e) => logs.push(format!(
                    "ERROR writing metrology JSON '{}': {e}",
                    dest.display()
                )),
            }
        }
    }
}
