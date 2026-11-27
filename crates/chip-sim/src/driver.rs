//! Unified setup simulation orchestrator, [`SetupSimulationResult`], single polymorphic
//! [`run_setup_simulation_with_backend`] driver, and single [`SimulatedStock`] playback cursor
//! state machine covering `Heightfield`, `TriDexel`, and 4th-axis `Cylindrical` engines.

#[cfg(feature = "gpu-compute")]
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;

use chip_cad::{BodyId, CadDocument};
use chip_cam::{
    ResolvedFixtureSolid, ResolvedSetup, SegmentMotion3D, ToolAssemblyPart, ToolAssemblyProfile,
    check_segments_fixture_collisions_par,
};
use chip_stock::{
    ActiveSimBackendInfo, CpuExactBackend, CylindricalStock, HeightfieldStock,
    OperationErrorSnapshot, RadialCutterProfile, RotaryStockSpec, SimComputeBackendChoice,
    SimEngineKind, SimError, SimMeshOptions, SimResolution, SimStockBackend, SimStockColorMode,
    SimStockMesh, SimulatedStock, StockCheckpointSnapshot, StockErrorReport, StockModelKind,
    StockPlaybackCursor, StockVerifyTolerances, SweepChord, SweepToolSpec, TriDexelStock,
};
#[cfg(feature = "gpu-compute")]
use chip_stock::{GpuCheckpointCache, GpuSimContext, WgpuComputeBackend};
use chip_tools::CutterGeometry;

use crate::{
    cad_adapter::{
        extract_cylindrical_target_mesh_from_cad, extract_heightfield_target_from_cad,
        extract_tridexel_target_from_cad, rotary_stock_spec_from_cam, stock_shape_spec_from_cam,
    },
    progress::{SimProgressScope, SimProgressStage},
    timeline::{MachineKinematicsConfig, ScrubSnapshot, SimOperationInput, SimulationTimeline},
    verifier::{SimDiagnostic, SimIssueKind, SimSeverity, SimVerificationConfig},
    verify_common::{
        RuleViolationHit, ViolationBatchCoalescer, evaluate_vertical_plunge_violation,
        push_segment_sweep_chords, slice_segment_range,
    },
};

/// Borrowed input bundle passed to [`run_setup_simulation`] and [`run_setup_simulation_with_backend`].
#[derive(Clone, Copy)]
pub struct SetupSimulationInput<'a> {
    pub doc: &'a CadDocument,
    pub setup: &'a ResolvedSetup,
    pub target_bodies: &'a [BodyId],
    pub fixtures: &'a [ResolvedFixtureSolid],
    pub operations: &'a [SimOperationInput<'a>],
    pub resolution: SimResolution,
    pub engine: SimEngineKind,
    pub backend_choice: SimComputeBackendChoice,
    pub kinematics: MachineKinematicsConfig,
    pub verify_config: SimVerificationConfig,
    /// Optional carried-over volumetric stock transferred from a preceding Setup (`M62`).
    /// When `Some`, Setup simulation starts from this machined solid instead of a fresh raw blank.
    pub initial_stock: Option<&'a TriDexelStock>,
    /// Optional original raw blank volume (`mm³`) from the root Setup of a carryover chain,
    /// preserved for cumulative `removed_volume_mm3` and `StockErrorReport` efficiency metrology.
    pub initial_raw_volume_mm3: Option<f64>,
    /// Optional display name of the preceding Setup that produced `initial_stock`.
    pub carried_over_from_setup: Option<&'a str>,
    /// Optional explicit shared GPU compute context (`None` uses `GpuSimContext::shared_headless()`
    /// when `backend_choice != SimComputeBackendChoice::Cpu`).
    #[cfg(feature = "gpu-compute")]
    pub gpu_ctx: Option<&'a Arc<GpuSimContext>>,
    /// Optional live completion progress reporter (`0%`–`100%`).
    pub progress: Option<SimProgressScope<'a>>,
    pub cancel: Option<&'a AtomicBool>,
}

impl<'a> SetupSimulationInput<'a> {
    /// Creates a [`SetupSimulationInput`] with default `Heightmap` engine, exact `Cpu` backend,
    /// `Standard` resolution, and default kinematics & verification tolerances.
    pub fn new(
        doc: &'a CadDocument,
        setup: &'a ResolvedSetup,
        operations: &'a [SimOperationInput<'a>],
    ) -> Self {
        Self {
            doc,
            setup,
            target_bodies: &[],
            fixtures: &[],
            operations,
            resolution: SimResolution::Standard,
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Cpu,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            #[cfg(feature = "gpu-compute")]
            gpu_ctx: None,
            progress: None,
            cancel: None,
        }
    }

    #[inline]
    pub const fn with_engine(mut self, engine: SimEngineKind) -> Self {
        self.engine = engine;
        self
    }

    #[inline]
    pub const fn with_backend_choice(mut self, backend_choice: SimComputeBackendChoice) -> Self {
        self.backend_choice = backend_choice;
        self
    }

    #[inline]
    pub const fn with_resolution(mut self, resolution: SimResolution) -> Self {
        self.resolution = resolution;
        self
    }

    #[inline]
    pub const fn with_kinematics(mut self, kinematics: MachineKinematicsConfig) -> Self {
        self.kinematics = kinematics;
        self
    }

    #[inline]
    pub const fn with_verify_config(mut self, verify_config: SimVerificationConfig) -> Self {
        self.verify_config = verify_config;
        self
    }

    #[inline]
    pub const fn with_target_bodies(mut self, target_bodies: &'a [BodyId]) -> Self {
        self.target_bodies = target_bodies;
        self
    }

    #[inline]
    pub const fn with_fixtures(mut self, fixtures: &'a [ResolvedFixtureSolid]) -> Self {
        self.fixtures = fixtures;
        self
    }

    #[inline]
    pub const fn with_progress(mut self, progress: Option<SimProgressScope<'a>>) -> Self {
        self.progress = progress;
        self
    }

    #[inline]
    pub const fn with_cancel(mut self, cancel: Option<&'a AtomicBool>) -> Self {
        self.cancel = cancel;
        self
    }

    #[cfg(feature = "gpu-compute")]
    #[inline]
    pub const fn with_gpu_ctx(mut self, gpu_ctx: Option<&'a Arc<GpuSimContext>>) -> Self {
        self.gpu_ctx = gpu_ctx;
        self
    }
}

/// Structured per-stage wall-clock timing & segment throughput metadata for a simulation run.
#[derive(Debug, Clone, Copy, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SimTimingBreakdown {
    pub target_setup_ms: f64,
    pub sweep_ms: f64,
    pub metrics_ms: f64,
    pub total_ms: f64,
    pub total_segments: usize,
    pub segments_per_sec: f64,
}

/// Complete output of simulating a Setup (`timeline`, unified `final_stock`, unified `checkpoints`,
/// `diagnostics`, quantitative `error_report`, and per-stage `timing`).
#[derive(Debug, Clone)]
pub struct SetupSimulationResult {
    pub engine: SimEngineKind,
    pub resolution: SimResolution,
    pub active_backend: ActiveSimBackendInfo,
    pub timing: SimTimingBreakdown,
    pub timeline: SimulationTimeline,
    pub initial_stock_volume_mm3: f64,
    pub remaining_stock_volume_mm3: f64,
    pub removed_volume_mm3: f64,
    /// Unified polymorphic final simulated stock (`Heightfield`, `TriDexel`, or `Cylindrical`).
    pub final_stock: SimulatedStock,
    /// Optional aligned 3D target solid when running the volumetric `TriDexel` engine.
    pub target_tridexel_stock: Option<TriDexelStock>,
    /// Unified periodic stock checkpoints for `< 4 ms` interactive timeline scrubbing.
    pub checkpoints: Vec<StockCheckpointSnapshot>,
    /// Per-operation cutter specs captured during simulation for exact interactive timeline
    /// reconstruction even if project operations are later edited.
    pub operation_profiles: Vec<SweepToolSpec>,
    pub diagnostics: Vec<SimDiagnostic>,
    /// Quantitative stock-vs-target error volume & surface deviation metrics (when 3D target model exists).
    pub error_report: Option<StockErrorReport>,
    /// Display name of the preceding Setup from which starting stock was carried over (`None` for raw blank).
    pub carried_over_from_setup: Option<String>,
}

impl SetupSimulationResult {
    #[inline]
    pub fn stock_kind(&self) -> StockModelKind {
        self.final_stock.kind()
    }

    #[inline]
    pub const fn is_cylindrical(&self) -> bool {
        self.final_stock.is_cylindrical()
    }

    #[inline]
    pub const fn rotary_spec(&self) -> Option<RotaryStockSpec> {
        self.final_stock.rotary_spec()
    }

    #[inline]
    pub const fn is_rotary(&self) -> bool {
        self.final_stock.is_rotary()
    }

    /// Computes the 4x4 WCS model matrix rotating simulated rotary stock by `rotary_deg`
    /// around its configured rotary axis and center (identity for 3-axis stock or zero angle).
    pub fn stock_model_wcs_transform(&self, rotary_deg: f64) -> chip_math::DMat4 {
        if let Some(rot) = self.rotary_spec()
            && rotary_deg.abs() > 1e-6
        {
            let angle_rad = (rotary_deg * rot.direction_sign).to_radians();
            chip_math::DMat4::from_translation(rot.center_wcs)
                * chip_math::DMat4::from_axis_angle(rot.axis_unit(), angle_rad)
                * chip_math::DMat4::from_translation(-rot.center_wcs)
        } else {
            chip_math::DMat4::IDENTITY
        }
    }

    #[inline]
    pub fn as_heightfield(&self) -> Option<&HeightfieldStock> {
        self.final_stock.as_heightfield()
    }

    #[inline]
    pub fn as_tridexel(&self) -> Option<&TriDexelStock> {
        self.final_stock.as_tridexel()
    }

    #[inline]
    pub fn as_cylindrical(&self) -> Option<&CylindricalStock> {
        self.final_stock.as_cylindrical()
    }

    pub fn critical_error_count(&self) -> usize {
        self.diagnostics
            .iter()
            .filter(|d| d.severity == SimSeverity::CriticalError)
            .count()
    }

    pub fn warning_count(&self) -> usize {
        self.diagnostics
            .iter()
            .filter(|d| d.severity == SimSeverity::Warning)
            .count()
    }

    pub fn has_critical_errors(&self) -> bool {
        self.critical_error_count() > 0
    }

    /// Recomputes [`StockErrorReport`] for an arbitrary `tolerance_mm` band (`in_tolerance_surface_pct`)
    /// without re-running the toolpath sweep.
    pub fn compute_error_report_at_tolerance(&self, tolerance_mm: f64) -> Option<StockErrorReport> {
        let per_op = self
            .error_report
            .as_ref()
            .map_or_else(Vec::new, |r| r.per_operation.clone());
        self.final_stock.compute_error_report(
            self.target_tridexel_stock.as_ref(),
            self.initial_stock_volume_mm3,
            tolerance_mm,
            per_op,
        )
    }

    /// Tessellates `stock` into a GPU-ready [`SimStockMesh`] using this result's target solid (when needed
    /// for `SimStockColorMode::ErrorHeatmap`).
    pub fn extract_stock_mesh(
        &self,
        stock: &SimulatedStock,
        color_mode: SimStockColorMode,
    ) -> SimStockMesh {
        self.extract_stock_mesh_with_options(stock, SimMeshOptions::from(color_mode))
    }

    /// Tessellates `stock` into a GPU-ready [`SimStockMesh`] using visual [`SimMeshOptions`]
    /// (`color_mode` + optional crease-preserving `mesh_skinning`).
    pub fn extract_stock_mesh_with_options(
        &self,
        stock: &SimulatedStock,
        options: SimMeshOptions,
    ) -> SimStockMesh {
        stock.extract_stock_mesh_with_options(self.target_tridexel_stock.as_ref(), options)
    }

    /// Reconstructs the exact [`SimulatedStock`] state at `time_sec` by restoring the nearest
    /// preceding [`StockCheckpointSnapshot`] and sweeping only the short tail delta up to `time_sec`.
    pub fn reconstruct_stock_at_time(&self, time_sec: f64) -> SimulatedStock {
        let mut stock = self.final_stock.clone();
        let mut cursor = None;
        self.sync_stock_to_time(&mut stock, &mut cursor, time_sec);
        stock
    }

    /// Synchronizes a reusable working [`SimulatedStock`] to the exact timeline state at `time_sec`.
    /// Returns `true` if `stock` mutated (indicating any cached display mesh should be re-tessellated).
    pub fn sync_stock_to_time(
        &self,
        stock: &mut SimulatedStock,
        cursor: &mut Option<StockPlaybackCursor>,
        time_sec: f64,
    ) -> bool {
        let total_spans = self.timeline.motion_spans.len();
        let target = self.timeline.sample_at_time(time_sec).map_or(
            StockPlaybackCursor {
                seg_idx: total_spans,
                seg_u: 0.0,
            },
            |s| target_cursor_from_snap(&s, total_spans),
        );
        self.sync_stock_to_target_cursor(stock, cursor, target)
    }

    /// Synchronizes a reusable working [`SimulatedStock`] to an already-sampled [`ScrubSnapshot`].
    /// Returns `true` if `stock` mutated (indicating any cached display mesh should be re-tessellated).
    pub fn sync_stock_to_scrub_snapshot(
        &self,
        stock: &mut SimulatedStock,
        cursor: &mut Option<StockPlaybackCursor>,
        snap: &ScrubSnapshot,
    ) -> bool {
        let target = target_cursor_from_snap(snap, self.timeline.motion_spans.len());
        self.sync_stock_to_target_cursor(stock, cursor, target)
    }

    fn sync_stock_to_target_cursor(
        &self,
        stock: &mut SimulatedStock,
        cursor: &mut Option<StockPlaybackCursor>,
        target: StockPlaybackCursor,
    ) -> bool {
        let total_spans = self.timeline.motion_spans.len();
        let spans = &self.timeline.motion_spans;
        let default_tol = StockVerifyTolerances::default();
        stock.sync_to_cursor(
            cursor,
            target,
            total_spans,
            &self.final_stock,
            &self.checkpoints,
            &self.operation_profiles,
            |seg_idx, u_start, u_end, out| {
                let Some(span) = spans.get(seg_idx) else {
                    return;
                };
                if u_start <= 1e-6 && u_end >= 1.0 - 1e-6 {
                    push_segment_sweep_chords(
                        &span.segment,
                        span.op_idx as u32,
                        span.op_idx,
                        default_tol,
                        out,
                    );
                } else if let Some(sub_seg) = slice_segment_range(&span.segment, u_start, u_end) {
                    push_segment_sweep_chords(
                        &sub_seg,
                        span.op_idx as u32,
                        span.op_idx,
                        default_tol,
                        out,
                    );
                }
            },
        )
    }

    /// Synchronizes a GPU-resident [`WgpuComputeBackend`] to an already-sampled [`ScrubSnapshot`]
    /// using the VRAM-bounded [`GpuCheckpointCache`] (`copy_buffer_to_buffer` restore + GPU tail
    /// delta sweep), returning `true` if GPU stock buffers mutated.
    #[cfg(feature = "gpu-compute")]
    pub fn sync_gpu_backend_to_scrub_snapshot(
        &self,
        backend: &mut WgpuComputeBackend,
        gpu_checkpoints: &mut GpuCheckpointCache,
        cursor: &mut Option<StockPlaybackCursor>,
        snap: &ScrubSnapshot,
    ) -> bool {
        let target = target_cursor_from_snap(snap, self.timeline.motion_spans.len());
        self.sync_gpu_backend_to_target_cursor(backend, gpu_checkpoints, cursor, target)
    }

    /// Synchronizes a GPU-resident [`WgpuComputeBackend`] to `time_sec` using the VRAM-bounded
    /// [`GpuCheckpointCache`], returning `true` if GPU stock buffers mutated.
    #[cfg(feature = "gpu-compute")]
    pub fn sync_gpu_backend_to_time(
        &self,
        backend: &mut WgpuComputeBackend,
        gpu_checkpoints: &mut GpuCheckpointCache,
        cursor: &mut Option<StockPlaybackCursor>,
        time_sec: f64,
    ) -> bool {
        let total_spans = self.timeline.motion_spans.len();
        let target = self.timeline.sample_at_time(time_sec).map_or(
            StockPlaybackCursor {
                seg_idx: total_spans,
                seg_u: 0.0,
            },
            |s| target_cursor_from_snap(&s, total_spans),
        );
        self.sync_gpu_backend_to_target_cursor(backend, gpu_checkpoints, cursor, target)
    }

    #[cfg(feature = "gpu-compute")]
    fn sync_gpu_backend_to_target_cursor(
        &self,
        backend: &mut WgpuComputeBackend,
        gpu_checkpoints: &mut GpuCheckpointCache,
        cursor: &mut Option<StockPlaybackCursor>,
        target: StockPlaybackCursor,
    ) -> bool {
        if let Some(cur) = *cursor
            && cur.seg_idx == target.seg_idx
            && (target.seg_u - cur.seg_u).abs() <= 1e-6
        {
            return false;
        }

        let total_spans = self.timeline.motion_spans.len();
        if target.seg_idx >= total_spans {
            if !gpu_checkpoints.restore_into_backend(backend, total_spans, None) {
                backend.copy_state_from_cpu_stock(&self.final_stock);
                let _ = gpu_checkpoints.capture_from_backend(backend, total_spans);
            }
            *cursor = Some(StockPlaybackCursor {
                seg_idx: total_spans,
                seg_u: 0.0,
            });
            return true;
        }

        let cp = self
            .checkpoints
            .iter()
            .rfind(|c| c.completed_segments() <= target.seg_idx);

        let (mut cur_seg, cur_u, mut changed) = if let Some(cur) = *cursor
            && (target.seg_idx > cur.seg_idx
                || (target.seg_idx == cur.seg_idx && target.seg_u >= cur.seg_u))
        {
            if let Some(cp) = cp
                && cp.completed_segments() > cur.seg_idx
            {
                let _ = gpu_checkpoints.restore_into_backend(
                    backend,
                    cp.completed_segments(),
                    Some(cp),
                );
                (cp.completed_segments(), 0.0, true)
            } else {
                (cur.seg_idx, cur.seg_u, false)
            }
        } else if let Some(cp) = cp {
            let _ =
                gpu_checkpoints.restore_into_backend(backend, cp.completed_segments(), Some(cp));
            (cp.completed_segments(), 0.0, true)
        } else {
            backend.copy_state_from_cpu_stock(&self.final_stock);
            (0, 0.0, true)
        };

        let spans = &self.timeline.motion_spans;
        let default_tol = StockVerifyTolerances::default();
        let mut delta_chords: Vec<SweepChord> = Vec::new();
        let emit_chords = |seg_idx: usize, u_start: f64, u_end: f64, out: &mut Vec<SweepChord>| {
            let Some(span) = spans.get(seg_idx) else {
                return;
            };
            if u_start <= 1e-6 && u_end >= 1.0 - 1e-6 {
                push_segment_sweep_chords(
                    &span.segment,
                    span.op_idx as u32,
                    span.op_idx,
                    default_tol,
                    out,
                );
            } else if let Some(sub_seg) = slice_segment_range(&span.segment, u_start, u_end) {
                push_segment_sweep_chords(
                    &sub_seg,
                    span.op_idx as u32,
                    span.op_idx,
                    default_tol,
                    out,
                );
            }
        };

        if cur_seg == target.seg_idx {
            if target.seg_u > cur_u + 1e-6 {
                emit_chords(cur_seg, cur_u, target.seg_u, &mut delta_chords);
            }
        } else {
            if cur_u > 1e-6 {
                emit_chords(cur_seg, cur_u, 1.0, &mut delta_chords);
                cur_seg += 1;
            }
            for seg_idx in cur_seg..target.seg_idx {
                emit_chords(seg_idx, 0.0, 1.0, &mut delta_chords);
            }
            if target.seg_u > 1e-6 {
                emit_chords(target.seg_idx, 0.0, target.seg_u, &mut delta_chords);
            }
        }

        if !delta_chords.is_empty() {
            let out = backend.sweep_batch(&delta_chords, &self.operation_profiles, false);
            changed |= out.mutated;
        }

        *cursor = Some(target);
        changed
    }
}

/// Extension trait adding ergonomic playback cursor sync directly onto [`SimulatedStock`] for
/// [`SetupSimulationResult`] timelines across all 3 volumetric stock engines.
pub trait SimulatedStockPlaybackExt {
    fn sync_to_scrub_snapshot(
        &mut self,
        cursor: &mut Option<StockPlaybackCursor>,
        result: &SetupSimulationResult,
        snap: &ScrubSnapshot,
    ) -> bool;

    fn sync_to_time(
        &mut self,
        cursor: &mut Option<StockPlaybackCursor>,
        result: &SetupSimulationResult,
        time_sec: f64,
    ) -> bool;
}

impl SimulatedStockPlaybackExt for SimulatedStock {
    fn sync_to_scrub_snapshot(
        &mut self,
        cursor: &mut Option<StockPlaybackCursor>,
        result: &SetupSimulationResult,
        snap: &ScrubSnapshot,
    ) -> bool {
        result.sync_stock_to_scrub_snapshot(self, cursor, snap)
    }

    fn sync_to_time(
        &mut self,
        cursor: &mut Option<StockPlaybackCursor>,
        result: &SetupSimulationResult,
        time_sec: f64,
    ) -> bool {
        result.sync_stock_to_time(self, cursor, time_sec)
    }
}

pub(crate) fn target_cursor_from_snap(
    snap: &ScrubSnapshot,
    total_spans: usize,
) -> StockPlaybackCursor {
    if snap.completed_segments >= total_spans {
        return StockPlaybackCursor {
            seg_idx: total_spans,
            seg_u: 0.0,
        };
    }
    let u = snap.seg_progress_t.clamp(0.0, 1.0);
    if u >= 1.0 - 1e-9 {
        StockPlaybackCursor {
            seg_idx: (snap.global_seg_idx + 1).min(total_spans),
            seg_u: 0.0,
        }
    } else if u <= 1e-6 {
        StockPlaybackCursor {
            seg_idx: snap.global_seg_idx.min(total_spans),
            seg_u: 0.0,
        }
    } else {
        StockPlaybackCursor {
            seg_idx: snap.global_seg_idx,
            seg_u: u,
        }
    }
}

/// Executes volumetric stock removal (`Heightmap`, `TriDexel`, or 4th-axis `Cylindrical`),
/// periodic checkpoint capture, and 6-rule crash/gouge/fixture verification across all active
/// Setup operations using the engine, backend choice, GPU context, and progress scope configured
/// on [`SetupSimulationInput`] (`06-simulation.md`).
pub fn run_setup_simulation(
    input: &SetupSimulationInput<'_>,
) -> Result<SetupSimulationResult, SimError> {
    let t_total = Instant::now();
    let t_setup = Instant::now();

    #[cfg(feature = "gpu-compute")]
    {
        let active_ctx = match input.backend_choice {
            SimComputeBackendChoice::Cpu => None,
            SimComputeBackendChoice::Auto | SimComputeBackendChoice::Gpu => input
                .gpu_ctx
                .cloned()
                .or_else(GpuSimContext::shared_headless),
        };
        let (effective_engine, stock, target_td) = prepare_initial_stock_with_gpu(
            input,
            input.engine,
            active_ctx.as_deref(),
            input.progress,
        )?;

        let mut res = if let Some(ctx) = active_ctx {
            let backend = WgpuComputeBackend::new(ctx, stock, target_td);
            let target_setup_ms = t_setup.elapsed().as_secs_f64() * 1000.0;
            let mut out = run_setup_simulation_with_backend(input, effective_engine, backend)?;
            finalize_timing(&mut out.timing, target_setup_ms, t_total);
            out
        } else {
            let backend = CpuExactBackend::new(stock, target_td);
            let target_setup_ms = t_setup.elapsed().as_secs_f64() * 1000.0;
            let mut out = run_setup_simulation_with_backend(input, effective_engine, backend)?;
            finalize_timing(&mut out.timing, target_setup_ms, t_total);
            out
        };
        res.timing.total_ms = t_total.elapsed().as_secs_f64() * 1000.0;
        res.timing.segments_per_sec =
            compute_segments_per_sec(res.timing.total_segments, res.timing.total_ms);
        Ok(res)
    }
    #[cfg(not(feature = "gpu-compute"))]
    {
        let (effective_engine, stock, target_td) =
            prepare_initial_stock(input, input.engine, input.progress)?;
        let backend = CpuExactBackend::new(stock, target_td);
        let target_setup_ms = t_setup.elapsed().as_secs_f64() * 1000.0;
        let mut res = run_setup_simulation_with_backend(input, effective_engine, backend)?;
        finalize_timing(&mut res.timing, target_setup_ms, t_total);
        Ok(res)
    }
}

#[inline]
fn compute_segments_per_sec(total_segments: usize, total_ms: f64) -> f64 {
    if total_ms > 1e-6 {
        (total_segments as f64) / (total_ms / 1000.0)
    } else {
        0.0
    }
}

#[inline]
fn finalize_timing(timing: &mut SimTimingBreakdown, target_setup_ms: f64, t_total: Instant) {
    timing.target_setup_ms = target_setup_ms;
    timing.total_ms = t_total.elapsed().as_secs_f64() * 1000.0;
    timing.segments_per_sec = compute_segments_per_sec(timing.total_segments, timing.total_ms);
}

#[cfg(not(feature = "gpu-compute"))]
fn prepare_initial_stock(
    input: &SetupSimulationInput<'_>,
    engine: SimEngineKind,
    progress: Option<SimProgressScope<'_>>,
) -> Result<(SimEngineKind, SimulatedStock, Option<TriDexelStock>), SimError> {
    if let Some(p) = progress {
        p.report(0.0, SimProgressStage::PreparingStock);
    }
    check_cancelled(input.cancel)?;
    let stock_spec = stock_shape_spec_from_cam(&input.setup.stock);

    if let Some(rotary) = input.setup.rotary
        && engine != SimEngineKind::TriDexel
    {
        let rotary_spec = rotary_stock_spec_from_cam(rotary);
        let mut cyl =
            CylindricalStock::from_stock_spec(&stock_spec, rotary_spec, input.resolution)?;
        if let Some(p) = progress {
            p.report(0.03, SimProgressStage::PreparingStock);
        }
        check_cancelled(input.cancel)?;
        if let Some(target_mesh) =
            extract_cylindrical_target_mesh_from_cad(input.doc, input.setup, input.target_bodies)
        {
            cyl.rasterize_target_mesh(&target_mesh);
        }
        if let Some(p) = progress {
            p.report(0.08, SimProgressStage::PreparingStock);
        }
        Ok((
            SimEngineKind::Heightmap,
            SimulatedStock::Cylindrical(cyl),
            None,
        ))
    } else if engine == SimEngineKind::TriDexel {
        let mut td = if let Some(carried) = input.initial_stock {
            carried.clone()
        } else {
            TriDexelStock::from_stock_spec(&stock_spec, input.resolution)?
        };
        td.rotary = input.setup.rotary.map(rotary_stock_spec_from_cam);
        if let Some(p) = progress {
            p.report(0.03, SimProgressStage::PreparingStock);
        }
        check_cancelled(input.cancel)?;
        let target_td =
            extract_tridexel_target_from_cad(input.doc, input.setup, input.target_bodies, &td);
        if let Some(p) = progress {
            p.report(0.08, SimProgressStage::PreparingStock);
        }
        Ok((
            SimEngineKind::TriDexel,
            SimulatedStock::TriDexel(Box::new(td)),
            target_td,
        ))
    } else {
        let mut hf = if let Some(carried) = input.initial_stock {
            HeightfieldStock::from_tridexel_projection(carried)
        } else {
            HeightfieldStock::from_stock_spec(&stock_spec, input.resolution)?
        };
        if let Some(p) = progress {
            p.report(0.03, SimProgressStage::PreparingStock);
        }
        check_cancelled(input.cancel)?;
        if input.doc.has_3d_geometry() {
            let (horiz_up, tris) =
                extract_heightfield_target_from_cad(input.doc, input.setup, input.target_bodies);
            hf.rasterize_target(&horiz_up, &tris);
        }
        if let Some(p) = progress {
            p.report(0.08, SimProgressStage::PreparingStock);
        }
        Ok((
            SimEngineKind::Heightmap,
            SimulatedStock::Heightfield(hf),
            None,
        ))
    }
}

#[cfg(feature = "gpu-compute")]
fn prepare_initial_stock_with_gpu(
    input: &SetupSimulationInput<'_>,
    engine: SimEngineKind,
    gpu_ctx: Option<&GpuSimContext>,
    progress: Option<SimProgressScope<'_>>,
) -> Result<(SimEngineKind, SimulatedStock, Option<TriDexelStock>), SimError> {
    if let Some(p) = progress {
        p.report(0.0, SimProgressStage::PreparingStock);
    }
    check_cancelled(input.cancel)?;
    let stock_spec = stock_shape_spec_from_cam(&input.setup.stock);

    if let Some(rotary) = input.setup.rotary
        && engine != SimEngineKind::TriDexel
    {
        let rotary_spec = rotary_stock_spec_from_cam(rotary);
        let mut cyl =
            CylindricalStock::from_stock_spec(&stock_spec, rotary_spec, input.resolution)?;
        if let Some(p) = progress {
            p.report(0.03, SimProgressStage::PreparingStock);
        }
        check_cancelled(input.cancel)?;
        if let Some(target_mesh) =
            extract_cylindrical_target_mesh_from_cad(input.doc, input.setup, input.target_bodies)
        {
            if let Some(ctx) = gpu_ctx {
                ctx.rasterize_cylindrical_target_mesh(&mut cyl, &target_mesh);
            } else {
                cyl.rasterize_target_mesh(&target_mesh);
            }
        }
        if let Some(p) = progress {
            p.report(0.08, SimProgressStage::PreparingStock);
        }
        Ok((
            SimEngineKind::Heightmap,
            SimulatedStock::Cylindrical(cyl),
            None,
        ))
    } else if engine == SimEngineKind::TriDexel {
        let mut td = if let Some(carried) = input.initial_stock {
            carried.clone()
        } else {
            TriDexelStock::from_stock_spec(&stock_spec, input.resolution)?
        };
        td.rotary = input.setup.rotary.map(rotary_stock_spec_from_cam);
        if let Some(p) = progress {
            p.report(0.03, SimProgressStage::PreparingStock);
        }
        check_cancelled(input.cancel)?;
        let target_td = if let Some(ctx) = gpu_ctx
            && input.doc.has_3d_geometry()
            && input.doc.faces.is_empty()
        {
            extract_cylindrical_target_mesh_from_cad(input.doc, input.setup, input.target_bodies)
                .map(|mesh| ctx.rasterize_tridexel_target_mesh(&td, &mesh))
        } else {
            extract_tridexel_target_from_cad(input.doc, input.setup, input.target_bodies, &td)
        };
        if let Some(p) = progress {
            p.report(0.08, SimProgressStage::PreparingStock);
        }
        Ok((
            SimEngineKind::TriDexel,
            SimulatedStock::TriDexel(Box::new(td)),
            target_td,
        ))
    } else {
        let mut hf = if let Some(carried) = input.initial_stock {
            HeightfieldStock::from_tridexel_projection(carried)
        } else {
            HeightfieldStock::from_stock_spec(&stock_spec, input.resolution)?
        };
        if let Some(p) = progress {
            p.report(0.03, SimProgressStage::PreparingStock);
        }
        check_cancelled(input.cancel)?;
        if input.doc.has_3d_geometry() {
            if let Some(ctx) = gpu_ctx
                && input.doc.faces.is_empty()
                && let Some(target_mesh) = extract_cylindrical_target_mesh_from_cad(
                    input.doc,
                    input.setup,
                    input.target_bodies,
                )
            {
                ctx.rasterize_heightfield_target_mesh(&mut hf, &target_mesh);
            } else {
                let (horiz_up, tris) = extract_heightfield_target_from_cad(
                    input.doc,
                    input.setup,
                    input.target_bodies,
                );
                hf.rasterize_target(&horiz_up, &tris);
            }
        }
        if let Some(p) = progress {
            p.report(0.08, SimProgressStage::PreparingStock);
        }
        Ok((
            SimEngineKind::Heightmap,
            SimulatedStock::Heightfield(hf),
            None,
        ))
    }
}

/// Single unified polymorphic setup simulation loop driving any [`SimStockBackend`] with optional
/// live completion progress reporting from `input.progress` (`0%`–`100%`).
pub fn run_setup_simulation_with_backend<B: SimStockBackend>(
    input: &SetupSimulationInput<'_>,
    engine: SimEngineKind,
    mut backend: B,
) -> Result<SetupSimulationResult, SimError> {
    let progress = input.progress;
    let t_driver_start = Instant::now();
    if let Some(p) = progress {
        p.report(0.05, SimProgressStage::PreparingStock);
    }
    check_cancelled(input.cancel)?;
    let timeline = SimulationTimeline::from_operations(input.operations, input.kinematics);
    let starting_stock_volume_mm3 = backend.compute_volume_mm3();
    let initial_stock_volume_mm3 = input
        .initial_raw_volume_mm3
        .unwrap_or(starting_stock_volume_mm3);

    let total_spans = timeline.motion_spans.len();
    let checkpoint_stride = backend.kind().checkpoint_stride(total_spans);
    let mut checkpoints = vec![backend.capture_checkpoint(0)];

    let operation_profiles: Vec<SweepToolSpec> = input
        .operations
        .iter()
        .map(|op| SweepToolSpec {
            profile: RadialCutterProfile::from_tool(op.tool),
            is_flat_endmill: matches!(op.tool.geometry, CutterGeometry::FlatEndmill { .. }),
        })
        .collect();

    let mut coalescer = ViolationBatchCoalescer::default();
    let mut per_op_errors: Vec<OperationErrorSnapshot> = Vec::new();
    let mut prev_vol = starting_stock_volume_mm3;
    let is_gpu_backend = backend.backend_info().is_gpu;
    const GPU_SWEEP_BATCH_MAX_SPANS: usize = 128;
    const GPU_SWEEP_BATCH_MAX_CHORDS: usize = 256;
    let mut span_chords: Vec<SweepChord> = Vec::with_capacity(if is_gpu_backend {
        GPU_SWEEP_BATCH_MAX_CHORDS + 16
    } else {
        16
    });
    let mut batch_chord_spans: Vec<usize> = Vec::with_capacity(if is_gpu_backend {
        GPU_SWEEP_BATCH_MAX_CHORDS + 16
    } else {
        0
    });
    let mut batch_span_count: usize = 0;

    if let Some(p) = progress {
        p.report(0.10, SimProgressStage::Sweeping);
    }
    let mut last_sweep_permille: u32 = 100;
    let t_sweep = Instant::now();
    for (span_idx, span) in timeline.motion_spans.iter().enumerate() {
        check_cancelled(input.cancel)?;

        let op_in = &input.operations[span.op_idx];
        let tool_spec = &operation_profiles[span.op_idx];
        let mut op_verify = input.verify_config;
        op_verify.gouge_tol_mm += op_in.intentional_edge_break_mm.max(0.0);

        let is_vertical_plunge = !matches!(span.segment.motion, SegmentMotion3D::Rapid)
            && (span.segment.end_wcs.x - span.segment.start_wcs.x)
                .hypot(span.segment.end_wcs.y - span.segment.start_wcs.y)
                <= 1e-4
            && span.segment.end_wcs.z < span.segment.start_wcs.z - 1e-4;

        // Flush any buffered GPU sweep batch before evaluating Rule 5 vertical plunge depth so the
        // stock height query sees exact up-to-the-segment material removal.
        if is_gpu_backend && is_vertical_plunge && !span_chords.is_empty() {
            let sweep_out = backend.sweep_batch(&span_chords, &operation_profiles, true);
            if !sweep_out.violations.is_empty() {
                let fallback_idx = batch_chord_spans.last().copied().unwrap_or(span_idx);
                coalescer.push_batch_violations(&sweep_out.violations, |src_chord_idx| {
                    let hit_idx = batch_chord_spans
                        .get(src_chord_idx as usize)
                        .copied()
                        .unwrap_or(fallback_idx);
                    let s = &timeline.motion_spans[hit_idx];
                    let op = &input.operations[s.op_idx];
                    let prof = &operation_profiles[s.op_idx].profile;
                    (
                        s.global_seg_idx,
                        s.op_idx,
                        op.op_id,
                        op.op_name,
                        op.tool.tool_number,
                        s.t_start_sec,
                        prof,
                    )
                });
            }
            span_chords.clear();
            batch_chord_spans.clear();
            batch_span_count = 0;
        }

        // Rule 5: Vertical Plunge check (shared across 3-axis Heightfield & Tri-Dexel; 0.0 on Cylindrical)
        if is_vertical_plunge
            && let Some(hit) = evaluate_vertical_plunge_violation(
                span,
                op_in.op_id,
                op_in.op_name,
                op_in.tool.tool_number,
                &tool_spec.profile,
                &op_verify,
                |x, y, z_start, z_end, thread_check_r| {
                    backend.query_vertical_plunge_depth(x, y, z_start, z_end, thread_check_r)
                },
            )
        {
            coalescer.push(hit);
        }

        let completed = span_idx + 1;
        let is_op_end = timeline.operations[span.op_idx].end_global_seg == completed;
        let need_checkpoint =
            (completed.is_multiple_of(checkpoint_stride) || is_op_end) && completed < total_spans;

        if is_gpu_backend {
            let chord_start = span_chords.len();
            push_segment_sweep_chords(
                &span.segment,
                span.op_idx as u32,
                span.op_idx,
                StockVerifyTolerances {
                    gouge_tol_mm: op_verify.gouge_tol_mm,
                    rapid_crash_tol_mm: op_verify.rapid_crash_tol_mm,
                    radial_wall_skin_mm: op_verify.radial_wall_skin_mm,
                },
                &mut span_chords,
            );
            batch_chord_spans.resize(span_chords.len(), span_idx);
            if span_chords.len() > chord_start {
                batch_span_count += 1;
            }

            let must_flush = is_op_end
                || need_checkpoint
                || completed == total_spans
                || batch_span_count >= GPU_SWEEP_BATCH_MAX_SPANS
                || span_chords.len() >= GPU_SWEEP_BATCH_MAX_CHORDS;
            if must_flush && !span_chords.is_empty() {
                let sweep_out = backend.sweep_batch(&span_chords, &operation_profiles, true);
                if !sweep_out.violations.is_empty() {
                    let fallback_idx = batch_chord_spans.last().copied().unwrap_or(span_idx);
                    coalescer.push_batch_violations(&sweep_out.violations, |src_chord_idx| {
                        let hit_idx = batch_chord_spans
                            .get(src_chord_idx as usize)
                            .copied()
                            .unwrap_or(fallback_idx);
                        let s = &timeline.motion_spans[hit_idx];
                        let op = &input.operations[s.op_idx];
                        let prof = &operation_profiles[s.op_idx].profile;
                        (
                            s.global_seg_idx,
                            s.op_idx,
                            op.op_id,
                            op.op_name,
                            op.tool.tool_number,
                            s.t_start_sec,
                            prof,
                        )
                    });
                }
                span_chords.clear();
                batch_chord_spans.clear();
                batch_span_count = 0;
            }
        } else {
            span_chords.clear();
            push_segment_sweep_chords(
                &span.segment,
                span.op_idx as u32,
                span.op_idx,
                StockVerifyTolerances {
                    gouge_tol_mm: op_verify.gouge_tol_mm,
                    rapid_crash_tol_mm: op_verify.rapid_crash_tol_mm,
                    radial_wall_skin_mm: op_verify.radial_wall_skin_mm,
                },
                &mut span_chords,
            );

            let sweep_out = backend.sweep_batch(&span_chords, &operation_profiles, true);
            coalescer.push_span_violations(
                &sweep_out.violations,
                span.global_seg_idx,
                span.op_idx,
                op_in.op_id,
                op_in.op_name,
                op_in.tool.tool_number,
                span.t_start_sec,
                &tool_spec.profile,
            );
        }

        if is_op_end {
            let cur_vol = backend.compute_volume_mm3();
            let op_removed = (prev_vol - cur_vol).max(0.0);
            prev_vol = cur_vol;
            if let Some((ex_v, g_v, rms)) = backend.compute_quick_error_summary() {
                per_op_errors.push(OperationErrorSnapshot {
                    op_idx: span.op_idx,
                    op_id: op_in.op_id.to_string(),
                    op_name: op_in.op_name.to_string(),
                    removed_volume_mm3: op_removed,
                    excess_volume_mm3: ex_v,
                    gouge_volume_mm3: g_v,
                    total_error_volume_mm3: ex_v + g_v,
                    rms_error_mm: rms,
                });
            }
        }
        if need_checkpoint {
            checkpoints.push(backend.capture_checkpoint(completed));
        }
        if let Some(p) = progress {
            let local_permille =
                100 + ((completed as u64).saturating_mul(820) / (total_spans.max(1) as u64)) as u32;
            if local_permille > last_sweep_permille {
                last_sweep_permille = local_permille;
                p.report(
                    f64::from(local_permille) / 1000.0,
                    SimProgressStage::Sweeping,
                );
            }
        }
    }
    let sweep_ms = t_sweep.elapsed().as_secs_f64() * 1000.0;
    if let Some(p) = progress {
        p.report(0.92, SimProgressStage::Sweeping);
    }

    // Rule 6: Full-assembly 3D workholding fixture collision verification (`M61`).
    // Runs in parallel over the shared `timeline.motion_spans` chord stream with 0.0 ms overhead
    // when `input.fixtures.is_empty()`, guaranteeing 100% identical diagnostics across CPU & GPU
    // backends and across Heightfield, Tri-Dexel, and Cylindrical engines!
    if !input.fixtures.is_empty() && !timeline.motion_spans.is_empty() {
        check_cancelled(input.cancel)?;
        if let Some(p) = progress {
            p.report(0.92, SimProgressStage::CheckingFixtures);
        }
        let assembly_profiles: Vec<ToolAssemblyProfile> = input
            .operations
            .iter()
            .map(|op| ToolAssemblyProfile::from_tool(op.tool))
            .collect();
        let span_items: Vec<(usize, usize, chip_cam::ToolpathSegment3D)> = timeline
            .motion_spans
            .iter()
            .enumerate()
            .map(|(idx, s)| (idx, s.op_idx, s.segment))
            .collect();
        let raw_hits = check_segments_fixture_collisions_par(
            &span_items,
            &assembly_profiles,
            input.fixtures,
            false,
        );
        for (span_pos, hit) in raw_hits {
            let span = &timeline.motion_spans[span_pos];
            let op_in = &input.operations[span.op_idx];
            let is_rapid = matches!(span.segment.motion, SegmentMotion3D::Rapid);
            let action_verb = match (is_rapid, hit.contact_part) {
                (true, ToolAssemblyPart::Flute) => "G0 rapid move collided with",
                (true, ToolAssemblyPart::Shank) => "Non-cutting shank rapid collided with",
                (true, ToolAssemblyPart::Holder) => "Holder/collet rapid collided with",
                (false, ToolAssemblyPart::Flute) => "Cutter flutes cut into",
                (false, ToolAssemblyPart::Shank) => "Non-cutting shank collided with",
                (false, ToolAssemblyPart::Holder) => "Holder/collet collided with",
            };
            let detail = format!(
                "{action_verb} '{}' by {:.2} mm at WCS ({:.2}, {:.2}, {:.2}).",
                hit.fixture_name,
                hit.penetration_mm,
                hit.hit_point_wcs.x,
                hit.hit_point_wcs.y,
                hit.hit_point_wcs.z,
            );
            coalescer.push(RuleViolationHit {
                severity: SimIssueKind::FixtureCollision.default_severity(),
                kind: SimIssueKind::FixtureCollision,
                span_idx: span.global_seg_idx,
                op_idx: span.op_idx,
                op_id: op_in.op_id.to_string(),
                op_name: op_in.op_name.to_string(),
                tool_number: op_in.tool.tool_number,
                time_sec: span.t_start_sec,
                location_wcs: hit.hit_point_wcs,
                excess_mm: hit.penetration_mm,
                detail,
                fixture_id: Some(hit.fixture_id),
                fixture_name: Some(hit.fixture_name),
            });
        }
    }

    if let Some(p) = progress {
        p.report(0.96, SimProgressStage::ComputingMetrics);
    }
    let t_metrics = Instant::now();
    let remaining_stock_volume_mm3 = backend.compute_volume_mm3();
    let removed_volume_mm3 = (initial_stock_volume_mm3 - remaining_stock_volume_mm3).max(0.0);
    let error_report = backend.compute_error_report(initial_stock_volume_mm3, 0.05, per_op_errors);
    let metrics_ms = t_metrics.elapsed().as_secs_f64() * 1000.0;
    let active_backend = backend.backend_info();
    let (final_stock, target_tridexel_stock) = backend.into_parts();
    let diagnostics = coalescer.into_diagnostics();
    let total_ms = t_driver_start.elapsed().as_secs_f64() * 1000.0;
    let timing = SimTimingBreakdown {
        target_setup_ms: 0.0,
        sweep_ms,
        metrics_ms,
        total_ms,
        total_segments: total_spans,
        segments_per_sec: compute_segments_per_sec(total_spans, total_ms),
    };
    if let Some(p) = progress {
        p.report(1.0, SimProgressStage::Complete);
    }

    Ok(SetupSimulationResult {
        engine,
        resolution: input.resolution,
        active_backend,
        timing,
        timeline,
        initial_stock_volume_mm3,
        remaining_stock_volume_mm3,
        removed_volume_mm3,
        final_stock,
        target_tridexel_stock,
        checkpoints,
        operation_profiles,
        diagnostics,
        error_report,
        carried_over_from_setup: input.carried_over_from_setup.map(ToString::to_string),
    })
}

#[inline]
fn check_cancelled(cancel: Option<&AtomicBool>) -> Result<(), SimError> {
    if cancel.is_some_and(|flag| flag.load(Ordering::Relaxed)) {
        return Err(SimError::Cancelled);
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::f64::consts::PI;

    use chip_cad::{
        CadDocument, CadKernel, MeshImportOptions, MonstertruckKernel, load_mesh_cad_document,
        sample_machined_plate_step, sample_rotary_shaft_stl_bytes,
    };
    use chip_cam::{
        AxisAnchor, BoxAnchor27, ClEvent, MotionRole, RotaryAxisKind, RotaryDriveMode, Setup,
        StockAxis, StockConfig, ToolpathBuilder, WcsOriginMode, WorkOffsetSlot, XyArcMove,
    };
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec2, DVec3, DisplayUnit, ParamLength, approx_eq};
    use chip_stock::{
        DexelSpan, RAW_STOCK_SIDE_COLOR, RAW_STOCK_TOP_COLOR, UNCUT_OP_IDX,
        deviation_to_heatmap_rgba, subtract_dexel_interval, tessellate_cylindrical_stock_mesh,
        tessellate_cylindrical_stock_mesh_colored, tessellate_tridexel_stock_mesh,
    };
    use chip_tools::default_starter_tool_library;

    use crate::{SimIssueKind, SimVerificationConfig};

    use super::*;

    #[test]
    fn tridexel_slot_cut_undercut_split_and_watertight_meshing_are_accurate() {
        let lib = default_starter_tool_library();
        let flat_6 = &lib.tools[0];

        let mut setup = Setup::new("s1", "Slot Test", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(100.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::default(),
        };
        let empty_doc = CadDocument::empty();
        let resolved = setup.resolve(&empty_doc, DisplayUnit::Millimeter).unwrap();

        let mut b = ToolpathBuilder::new("op_slot");
        b.push_event(ClEvent::SpindleStart {
            rpm: 12_000.0,
            direction: chip_tools::SpindleDirection::Cw,
        });
        b.rapid_to(DVec3::new(-10.0, 0.0, 5.0));
        b.linear_to(DVec3::new(-10.0, 0.0, -5.0), 300.0, MotionRole::Plunge);
        b.linear_to(DVec3::new(10.0, 0.0, -5.0), 1000.0, MotionRole::Cut);
        b.rapid_to(DVec3::new(10.0, 0.0, 5.0));
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &empty_doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_slot",
                op_name: "Slot",
                tool: flat_6,
                fallback_spindle_rpm: 12_000.0,
                toolpath: &tp,
                intentional_edge_break_mm: 0.0,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::TriDexel,
            ..SetupSimulationInput::new(&empty_doc, &resolved, &[])
        })
        .unwrap();

        assert_eq!(sim.engine, SimEngineKind::TriDexel);
        assert!(!sim.has_critical_errors(), "diags: {:?}", sim.diagnostics);

        let expected_removed = (20.0 * 6.0 + PI * 9.0) * 5.0;
        let rel_err = (sim.removed_volume_mm3 - expected_removed).abs() / expected_removed;
        assert!(
            rel_err < 0.035,
            "Standard removed vol {:.2} vs expected {:.2} (err {:.2}%)",
            sim.removed_volume_mm3,
            expected_removed,
            rel_err * 100.0
        );

        let sim_fine = run_setup_simulation(&SetupSimulationInput {
            doc: &empty_doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_slot",
                op_name: "Slot",
                tool: flat_6,
                fallback_spindle_rpm: 12_000.0,
                toolpath: &tp,
                intentional_edge_break_mm: 0.0,
            }],
            resolution: SimResolution::Fine,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::TriDexel,
            ..SetupSimulationInput::new(&empty_doc, &resolved, &[])
        })
        .unwrap();
        let fine_err = (sim_fine.removed_volume_mm3 - expected_removed).abs() / expected_removed;
        assert!(
            fine_err < 0.015,
            "Fine removed vol {:.2} vs expected {:.2} (err {:.2}%)",
            sim_fine.removed_volume_mm3,
            expected_removed,
            fine_err * 100.0
        );

        let td = sim.as_tridexel().unwrap();
        let mesh = tessellate_tridexel_stock_mesh(td, true);
        assert!(mesh.vertices.len() > 100);
        assert_eq!(mesh.indices.len() % 3, 0);

        let mut ray = vec![DexelSpan::new(-20.0, 0.0, UNCUT_OP_IDX, UNCUT_OP_IDX)];
        let rem = subtract_dexel_interval(&mut ray, -12.0, -8.0, 0);
        assert!((rem - 4.0).abs() < 1e-12);
        assert_eq!(ray.len(), 2, "undercut must split span into 2 intervals");
        assert!((ray[0].max_mm - (-12.0)).abs() < 1e-12);
        assert!((ray[1].min_mm - (-8.0)).abs() < 1e-12);
    }

    #[test]
    fn tridexel_fault_injection_catches_rapid_crash_and_vertical_wall_target_gouge() {
        let step = sample_machined_plate_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("plate.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let lib = default_starter_tool_library();
        let flat_6 = &lib.tools[0];

        let mut setup = Setup::new("s1", "Fault Test", WorkOffsetSlot::G54);
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let maabb = resolved.model_aabb_wcs;
        let center = maabb.center();

        let mut b = ToolpathBuilder::new("op_bad");
        b.push_event(ClEvent::SpindleStart {
            rpm: 12_000.0,
            direction: chip_tools::SpindleDirection::Cw,
        });
        b.rapid_to(DVec3::new(center.x, center.y, maabb.max.z + 15.0));
        b.rapid_to(DVec3::new(center.x, center.y, maabb.max.z - 5.0));
        b.linear_to(
            DVec3::new(center.x + 15.0, center.y, maabb.max.z - 5.0),
            800.0,
            MotionRole::Cut,
        );
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_bad",
                op_name: "Bad Cut",
                tool: flat_6,
                fallback_spindle_rpm: 12_000.0,
                toolpath: &tp,
                intentional_edge_break_mm: 0.0,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::TriDexel,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        })
        .unwrap();

        assert!(sim.has_critical_errors());
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::RapidIntoStock),
            "expected RapidIntoStock diagnostic: {:?}",
            sim.diagnostics
        );
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::TargetModelGouge),
            "expected TargetModelGouge diagnostic: {:?}",
            sim.diagnostics
        );
    }

    #[test]
    fn cylindrical_stock_turning_volume_and_chucked_square_bar_are_exact() {
        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_rot", "Rotary A", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27 {
                x: AxisAnchor::Center,
                y: AxisAnchor::Center,
                z: AxisAnchor::Center,
            },
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(44.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
        setup.rotary.axis = RotaryAxisKind::XAxis;

        let doc = CadDocument::empty();
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        assert!(resolved.rotary.is_some());

        let mut b = ToolpathBuilder::new("op_turn");
        b.rapid_to(DVec3::new(-10.0, 0.0, 30.0));
        b.linear_to(DVec3::new(-10.0, 0.0, 18.0), 400.0, MotionRole::Plunge);
        b.rotary_linear_to(
            DVec3::new(-10.0, 0.0, 18.0),
            360.0,
            800.0,
            18.0,
            MotionRole::Cut,
        );
        let n_turns = 10usize;
        let steps = n_turns * 36;
        for s in 1..=steps {
            let frac = s as f64 / steps as f64;
            let x = -10.0 + 20.0 * frac;
            let deg = 360.0 + 360.0 * n_turns as f64 * frac;
            b.rotary_linear_to(DVec3::new(x, 0.0, 18.0), deg, 800.0, 18.0, MotionRole::Cut);
        }
        b.rotary_linear_to(
            DVec3::new(10.0, 0.0, 18.0),
            360.0 * (n_turns as f64 + 2.0),
            800.0,
            18.0,
            MotionRole::FinishCut,
        );
        b.rapid_to(DVec3::new(10.0, 0.0, 30.0));
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_turn",
                op_name: "Rotary Turn Groove",
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

        assert_eq!(
            sim.critical_error_count(),
            0,
            "Clean rotary turning groove should have 0 crashes, got: {:#?}",
            sim.diagnostics
        );
        let exact_v = PI * (22.0_f64.powi(2) - 18.0_f64.powi(2)) * 26.0;
        let err_pct = ((sim.removed_volume_mm3 - exact_v).abs() / exact_v) * 100.0;
        assert!(
            err_pct < 3.0,
            "Removed volume {:.1} mm³ should match analytical {:.1} mm³ within 3% (err={err_pct:.2}%)",
            sim.removed_volume_mm3,
            exact_v
        );

        let cyl_stock = sim.as_cylindrical().unwrap();
        let mesh = tessellate_cylindrical_stock_mesh(cyl_stock, true, 45.0);
        assert!(mesh.vertices.len() > 1_000);
        assert_eq!(mesh.indices.len() % 3, 0);

        let mut sq_setup = Setup::new("s_sq", "Square Bar", WorkOffsetSlot::G54);
        sq_setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27 {
                x: AxisAnchor::Center,
                y: AxisAnchor::Center,
                z: AxisAnchor::Center,
            },
        };
        sq_setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(120.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(100.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::CenterZ,
        };
        sq_setup.rotary.mode = RotaryDriveMode::Indexed3Plus1;
        sq_setup.rotary.axis = RotaryAxisKind::XAxis;
        let sq_res = sq_setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let sq_cyl = CylindricalStock::from_stock_spec(
            &stock_shape_spec_from_cam(&sq_res.stock),
            rotary_stock_spec_from_cam(sq_res.rotary.unwrap()),
            SimResolution::Standard,
        )
        .unwrap();
        assert!(
            approx_eq(sq_cyl.initial_volume_mm3(), 1_200_000.0, 2_000.0),
            "Square bar polar volume should be ~1,200,000 mm³, got {:.1}",
            sq_cyl.initial_volume_mm3()
        );
    }

    #[test]
    fn fault_injection_catches_rotary_rapid_crash_target_gouge_and_chuck_collision() {
        let stl = sample_rotary_shaft_stl_bytes();
        let doc = load_mesh_cad_document("rotary_shaft.stl", &stl, &MeshImportOptions::default())
            .unwrap();
        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_shaft", "Rotary Shaft", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27 {
                x: AxisAnchor::Center,
                y: AxisAnchor::Center,
                z: AxisAnchor::Center,
            },
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(44.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup.rotary.axis = RotaryAxisKind::XAxis;
        setup.rotary.chuck_length = ParamLength::from_mm(20.0);
        setup.rotary.chuck_diameter = ParamLength::from_mm(65.0);
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let mut b = ToolpathBuilder::new("op_fault");
        b.rapid_to(DVec3::new(0.0, 0.0, 35.0));
        b.rapid_to(DVec3::new(0.0, 0.0, 17.0));
        b.linear_to(DVec3::new(0.0, 0.0, 10.0), 400.0, MotionRole::Cut);
        b.rapid_to(DVec3::new(0.0, 0.0, 35.0));
        b.rapid_to(DVec3::new(-48.0, 0.0, 25.0));
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_fault",
                op_name: "Fault Injection",
                tool: flat_6mm,
                fallback_spindle_rpm: 10_000.0,
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

        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::RapidIntoStock),
            "Must detect rotary RapidIntoStock, got: {:#?}",
            sim.diagnostics
        );
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::TargetModelGouge),
            "Must detect rotary TargetModelGouge on shaft STL, got: {:#?}",
            sim.diagnostics
        );
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::HolderCollision),
            "Must detect rotary chuck HolderCollision, got: {:#?}",
            sim.diagnostics
        );
    }

    #[test]
    fn off_centerline_simultaneous_4axis_fault_injection_catches_flank_gouge_and_holder_crash() {
        let stl = chip_cad::sample_helical_flute_shaft_stl_bytes();
        let doc = load_mesh_cad_document(
            "helical_flute_shaft.stl",
            &stl,
            &MeshImportOptions::default(),
        )
        .unwrap();
        assert!(doc.meshes[0].diagnostics.is_watertight);
        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_helical", "Helical Shaft", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(42.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup.rotary.axis = RotaryAxisKind::XAxis;
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let mut b = ToolpathBuilder::new("op_4ax_fault");
        b.rotary_rapid_to(DVec3::new(0.0, 8.0, 35.0), 0.0);
        b.rotary_linear_to(DVec3::new(0.0, 8.0, 9.0), 0.0, 400.0, 12.0, MotionRole::Cut);
        b.rotary_linear_to(
            DVec3::new(10.0, 8.0, 9.0),
            45.0,
            400.0,
            12.0,
            MotionRole::Cut,
        );
        b.rotary_rapid_to(DVec3::new(10.0, 8.0, 35.0), 45.0);
        b.rotary_rapid_to(DVec3::new(-32.0, -7.0, 35.0), 45.0);
        b.rotary_linear_to(
            DVec3::new(-32.0, -7.0, -25.0),
            45.0,
            300.0,
            20.0,
            MotionRole::Plunge,
        );
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_4ax_fault",
                op_name: "Off-Centerline Faults",
                tool: flat_6mm,
                fallback_spindle_rpm: 10_000.0,
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

        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::TargetModelGouge),
            "Off-centerline 4-axis verifier must catch helical flank gouge, got: {:#?}",
            sim.diagnostics
        );
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::HolderCollision),
            "Off-centerline 4-axis verifier must catch deep holder crash, got: {:#?}",
            sim.diagnostics
        );
    }

    #[test]
    fn rotary_setup_simulation_populates_error_report_per_op_and_tolerance_recompute() {
        let stl = sample_rotary_shaft_stl_bytes();
        let doc = load_mesh_cad_document("rotary_shaft.stl", &stl, &MeshImportOptions::default())
            .unwrap();
        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_shaft_err", "Rotary Shaft Error", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(44.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup.rotary.axis = RotaryAxisKind::XAxis;
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let mut b = ToolpathBuilder::new("op_turn");
        b.rotary_rapid_to(DVec3::new(-10.0, 0.0, 30.0), 0.0);
        b.rotary_linear_to(
            DVec3::new(-10.0, 0.0, 19.0),
            0.0,
            400.0,
            19.0,
            MotionRole::Plunge,
        );
        b.rotary_linear_to(
            DVec3::new(10.0, 0.0, 19.0),
            360.0,
            800.0,
            19.0,
            MotionRole::Cut,
        );
        b.rotary_rapid_to(DVec3::new(10.0, 0.0, 30.0), 360.0);
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_turn",
                op_name: "Rotary Rough",
                tool: flat_6mm,
                fallback_spindle_rpm: 10_000.0,
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
            engine: SimEngineKind::TriDexel,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        })
        .unwrap();

        let rep = sim
            .error_report
            .as_ref()
            .expect("4th-axis rotary simulation with 3D target must populate error_report");
        assert!(rep.target_volume_mm3 > 10_000.0);
        assert!(rep.sample_count > 1_000);
        assert!(rep.rms_error_mm.is_finite() && rep.rms_error_mm >= 0.0);
        assert!(rep.total_error_volume_mm3 > 0.0);
        assert_eq!(rep.per_operation.len(), 1);
        assert_eq!(rep.per_operation[0].op_name, "Rotary Rough");

        let tight = sim
            .compute_error_report_at_tolerance(0.01)
            .expect("recompute_at_tolerance must succeed for cylindrical stock");
        let loose = sim
            .compute_error_report_at_tolerance(5.0)
            .expect("recompute_at_tolerance must succeed for cylindrical stock");
        assert!(
            loose.in_tolerance_surface_pct >= tight.in_tolerance_surface_pct,
            "loose tol ({:.2}%) should be >= tight tol ({:.2}%)",
            loose.in_tolerance_surface_pct,
            tight.in_tolerance_surface_pct
        );
    }

    #[test]
    fn cylindrical_error_heatmap_tessellation_colors_in_tol_excess_and_gouge_bands() {
        let mut setup = Setup::new("s_hm", "Rotary Heatmap", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(40.0),
            length: ParamLength::from_mm(60.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup.rotary.axis = RotaryAxisKind::XAxis;

        let doc = CadDocument::empty();
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let mut stock = CylindricalStock::from_stock_spec(
            &stock_shape_spec_from_cam(&resolved.stock),
            rotary_stock_spec_from_cam(resolved.rotary.unwrap()),
            SimResolution::Standard,
        )
        .unwrap();

        let nu = stock.nu;
        let nt = stock.ntheta;
        for iu in 0..nu {
            for it in 0..nt {
                let idx = iu * nt + it;
                stock.r_target[idx] = 18.0_f32;
                stock.r_stock[idx] = if iu < nu / 3 {
                    19.0_f32
                } else if iu < (2 * nu) / 3 {
                    18.0_f32
                } else {
                    17.0_f32
                };
            }
        }

        let hm_mesh = tessellate_cylindrical_stock_mesh_colored(
            &stock,
            SimStockColorMode::ErrorHeatmap {
                tolerance_mm: 0.05,
                max_scale_mm: 1.0,
            },
            0.0,
        );
        let green = deviation_to_heatmap_rgba(0.0, 0.05, 1.0);
        let blue = deviation_to_heatmap_rgba(1.0, 0.05, 1.0);
        let red = deviation_to_heatmap_rgba(-1.0, 0.05, 1.0);
        let close = |a: [f32; 4], b: [f32; 4]| {
            (a[0] - b[0]).abs() < 1e-3 && (a[1] - b[1]).abs() < 1e-3 && (a[2] - b[2]).abs() < 1e-3
        };
        assert!(hm_mesh.vertices.iter().any(|v| close(v.color_rgba, green)));
        assert!(hm_mesh.vertices.iter().any(|v| close(v.color_rgba, blue)));
        assert!(hm_mesh.vertices.iter().any(|v| close(v.color_rgba, red)));

        let metal_mesh =
            tessellate_cylindrical_stock_mesh_colored(&stock, SimStockColorMode::Metallic, 0.0);
        assert!(metal_mesh.vertices.iter().all(|v| {
            close(v.color_rgba, RAW_STOCK_TOP_COLOR) || close(v.color_rgba, RAW_STOCK_SIDE_COLOR)
        }));
    }

    #[test]
    fn unified_playback_cursor_matches_full_reconstruction_across_all_three_stock_engines() {
        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];
        let doc = CadDocument::empty();

        // 1. 3-axis Box Setup (tested on both Heightfield and TriDexel)
        let mut setup_3ax = Setup::new("s_3ax", "3-Axis Scrub", WorkOffsetSlot::G54);
        setup_3ax.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup_3ax.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(100.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::CenterZ,
        };
        let resolved_3ax = setup_3ax.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let mut b3 = ToolpathBuilder::new("op_3ax");
        b3.rapid_to(DVec3::new(-25.0, 0.0, 10.0));
        b3.linear_to(DVec3::new(-25.0, 0.0, -5.0), 300.0, MotionRole::Plunge);
        b3.linear_to(DVec3::new(25.0, 0.0, -5.0), 600.0, MotionRole::Cut);
        b3.arc_xy_to(XyArcMove {
            end_wcs: DVec3::new(-25.0, 0.0, -5.0),
            center_xy: DVec2::ZERO,
            radius_mm: 25.0,
            ccw: true,
            turns: 0,
            feed_mm_min: 600.0,
            role: MotionRole::Cut,
        });
        b3.rapid_to(DVec3::new(-25.0, 0.0, 10.0));
        let tp3 = b3.finish();

        // 2. 4-axis Rotary Cylinder Setup (Cylindrical engine)
        let mut setup_4ax = Setup::new("s_4ax", "4-Axis Scrub", WorkOffsetSlot::G54);
        setup_4ax.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup_4ax.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(40.0),
            length: ParamLength::from_mm(60.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup_4ax.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup_4ax.rotary.axis = RotaryAxisKind::XAxis;
        let resolved_4ax = setup_4ax.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let mut b4 = ToolpathBuilder::new("op_4ax");
        b4.rotary_rapid_to(DVec3::new(-20.0, 0.0, 26.0), 0.0);
        b4.rotary_linear_to(
            DVec3::new(-20.0, 0.0, 16.0),
            0.0,
            400.0,
            16.0,
            MotionRole::Plunge,
        );
        b4.rotary_linear_to(
            DVec3::new(20.0, 0.0, 16.0),
            180.0,
            600.0,
            16.0,
            MotionRole::Cut,
        );
        b4.rotary_rapid_to(DVec3::new(20.0, 0.0, 26.0), 180.0);
        let tp4 = b4.finish();

        let sim_hf = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved_3ax,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_3ax",
                op_name: "3-Axis Cut",
                tool: flat_6mm,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp3,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::Heightmap,
            ..SetupSimulationInput::new(&doc, &resolved_3ax, &[])
        })
        .unwrap();

        let sim_td = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved_3ax,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_3ax",
                op_name: "3-Axis Cut",
                tool: flat_6mm,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp3,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::TriDexel,
            ..SetupSimulationInput::new(&doc, &resolved_3ax, &[])
        })
        .unwrap();

        let sim_cyl = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved_4ax,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_4ax",
                op_name: "4-Axis Cut",
                tool: flat_6mm,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp4,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            ..SetupSimulationInput::new(&doc, &resolved_4ax, &[])
        })
        .unwrap();

        assert!(sim_hf.as_heightfield().is_some());
        assert!(sim_td.as_tridexel().is_some());
        assert!(sim_cyl.as_cylindrical().is_some());

        for (label, sim) in [
            ("Heightfield", &sim_hf),
            ("TriDexel", &sim_td),
            ("Cylindrical", &sim_cyl),
        ] {
            let mut working = sim.final_stock.clone();
            let mut cursor: Option<StockPlaybackCursor> = None;
            let total_t = sim.timeline.total_time_sec;
            let fracs = [0.0, 0.20, 0.45, 0.70, 0.90, 0.30, 0.60, 1.0];

            for frac in fracs {
                let t = total_t * frac;
                let snap = sim.timeline.sample_at_time(t).unwrap();
                let _ = working.sync_to_scrub_snapshot(&mut cursor, sim, &snap);
                let reference = sim.reconstruct_stock_at_time(t);

                let vol_work = working.compute_volume_mm3();
                let vol_ref = reference.compute_volume_mm3();
                let tol = (sim.initial_stock_volume_mm3 * 1e-4).max(1.0);
                assert!(
                    (vol_work - vol_ref).abs() <= tol,
                    "{label} incremental scrub volume diverged at frac={frac}: work={vol_work:.3}, ref={vol_ref:.3}"
                );
            }
        }
    }

    #[cfg(feature = "gpu-compute")]
    #[test]
    fn gpu_vs_cpu_heightfield_end_to_end_parity_all_rules_and_cutters() {
        let step = sample_machined_plate_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("plate.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let lib = default_starter_tool_library();
        let flat_6 = &lib.tools[0];
        let vbit = lib
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
            .unwrap();
        let face = lib
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::FaceMill { .. }))
            .unwrap();

        let mut setup = Setup::new("s_hf_parity", "HF Parity", WorkOffsetSlot::G54);
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let maabb = resolved.model_aabb_wcs;
        let c = maabb.center();

        // Op 0: Clean sloped ramp + XY arc + deliberate rapid crash + target gouge
        let mut b0 = ToolpathBuilder::new("op_flat_fault");
        b0.rapid_to(DVec3::new(c.x - 12.0, c.y, maabb.max.z + 10.0));
        b0.linear_to(
            DVec3::new(c.x - 12.0, c.y, maabb.max.z - 1.5),
            300.0,
            MotionRole::Plunge,
        );
        b0.linear_to(
            DVec3::new(c.x, c.y + 4.0, maabb.max.z - 3.0),
            800.0,
            MotionRole::Cut,
        );
        b0.arc_xy_to(XyArcMove {
            end_wcs: DVec3::new(c.x + 10.0, c.y + 4.0, maabb.max.z - 3.0),
            center_xy: DVec2::new(c.x + 5.0, c.y + 4.0),
            radius_mm: 5.0,
            ccw: true,
            turns: 0,
            feed_mm_min: 700.0,
            role: MotionRole::Cut,
        });
        // Rapid crash + target gouge
        b0.rapid_to(DVec3::new(c.x, c.y - 6.0, maabb.max.z - 6.0));
        b0.rapid_to(DVec3::new(c.x, c.y - 6.0, maabb.max.z + 10.0));
        let tp0 = b0.finish();

        // Op 1: V-bit sloped climb
        let mut b1 = ToolpathBuilder::new("op_vbit");
        b1.rapid_to(DVec3::new(c.x - 8.0, c.y + 8.0, maabb.max.z + 10.0));
        b1.linear_to(
            DVec3::new(c.x - 8.0, c.y + 8.0, maabb.max.z - 3.5),
            400.0,
            MotionRole::Plunge,
        );
        b1.linear_to(
            DVec3::new(c.x + 8.0, c.y + 12.0, maabb.max.z - 0.5),
            600.0,
            MotionRole::Cut,
        );
        b1.rapid_to(DVec3::new(c.x + 8.0, c.y + 12.0, maabb.max.z + 10.0));
        let tp1 = b1.finish();

        // Op 2: Non-center-cutting FaceMill vertical plunge (Rule 5 check!)
        let mut b2 = ToolpathBuilder::new("op_face_plunge");
        b2.rapid_to(DVec3::new(c.x + 14.0, c.y - 10.0, maabb.max.z + 8.0));
        b2.linear_to(
            DVec3::new(c.x + 14.0, c.y - 10.0, maabb.max.z - 2.0),
            200.0,
            MotionRole::Plunge,
        );
        b2.rapid_to(DVec3::new(c.x + 14.0, c.y - 10.0, maabb.max.z + 8.0));
        let tp2 = b2.finish();

        let ops = [
            SimOperationInput {
                op_id: "op_flat_fault",
                op_name: "Flat Ramp & Fault",
                tool: flat_6,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp0,
            },
            SimOperationInput {
                op_id: "op_vbit",
                op_name: "V-Carve Climb",
                tool: vbit,
                fallback_spindle_rpm: 14_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp1,
            },
            SimOperationInput {
                op_id: "op_face_plunge",
                op_name: "Face Plunge Rule 5",
                tool: face,
                fallback_spindle_rpm: 8_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp2,
            },
        ];

        let input = SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &ops,
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        };

        let sim_cpu = run_setup_simulation(&SetupSimulationInput {
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Cpu,
            ..input
        })
        .unwrap();
        let sim_gpu = run_setup_simulation(&SetupSimulationInput {
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Gpu,
            ..input
        })
        .unwrap();

        assert!(!sim_cpu.active_backend.is_gpu);
        assert!(sim_gpu.active_backend.is_gpu);

        let vol_err = (sim_gpu.remaining_stock_volume_mm3 - sim_cpu.remaining_stock_volume_mm3)
            .abs()
            / sim_cpu.remaining_stock_volume_mm3.max(1.0);
        assert!(
            vol_err < 0.0005,
            "Heightfield E2E volume parity failed: cpu={:.3}, gpu={:.3}, err={:.5}%",
            sim_cpu.remaining_stock_volume_mm3,
            sim_gpu.remaining_stock_volume_mm3,
            vol_err * 100.0
        );

        let hf_cpu = sim_cpu.as_heightfield().unwrap();
        let hf_gpu = sim_gpu.as_heightfield().unwrap();
        let mut max_delta = 0.0_f64;
        let mut worst = (0usize, 0usize, 0.0_f32, 0.0_f32, 0u32, 0u32);
        for (idx, (zc, zg)) in hf_cpu.z_stock.iter().zip(hf_gpu.z_stock.iter()).enumerate() {
            assert_eq!(zc.is_finite(), zg.is_finite());
            if zc.is_finite() {
                let d = (f64::from(*zg) - f64::from(*zc)).abs();
                if d > max_delta {
                    max_delta = d;
                    worst = (
                        idx % hf_cpu.nx,
                        idx / hf_cpu.nx,
                        *zc,
                        *zg,
                        hf_cpu.last_op_idx[idx],
                        hf_gpu.last_op_idx[idx],
                    );
                }
            }
        }
        assert!(
            max_delta <= 2e-3,
            "Heightfield E2E max cell delta {max_delta:.6} mm > 2e-3 mm at ({}, {}) wcs=({:.4},{:.4}) origin=({:.4},{:.4},{:.4}) d=({:.5},{:.5}) c=({:.4},{:.4},{:.4}) maxz={:.4}: cpu={} (op {}), gpu={} (op {})",
            worst.0,
            worst.1,
            hf_cpu.origin_wcs.x + (worst.0 as f64 + 0.5) * hf_cpu.dx_mm,
            hf_cpu.origin_wcs.y + (worst.1 as f64 + 0.5) * hf_cpu.dy_mm,
            hf_cpu.origin_wcs.x,
            hf_cpu.origin_wcs.y,
            hf_cpu.origin_wcs.z,
            hf_cpu.dx_mm,
            hf_cpu.dy_mm,
            c.x,
            c.y,
            c.z,
            maabb.max.z,
            worst.2,
            worst.4,
            worst.3,
            worst.5
        );

        // Identical Rule 1-5 diagnostic counts & severities
        assert_eq!(
            sim_cpu.diagnostics.len(),
            sim_gpu.diagnostics.len(),
            "Diagnostic count mismatch:\nCPU: {:#?}\nGPU: {:#?}",
            sim_cpu.diagnostics,
            sim_gpu.diagnostics
        );
        for (dc, dg) in sim_cpu.diagnostics.iter().zip(sim_gpu.diagnostics.iter()) {
            assert_eq!(dc.kind, dg.kind);
            assert_eq!(dc.severity, dg.severity);
            assert_eq!(dc.op_idx, dg.op_idx);
        }
        assert!(
            sim_gpu
                .diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::UnsafeVerticalPlunge),
            "Rule 5 must fire identically on GPU backend"
        );
    }

    #[cfg(feature = "gpu-compute")]
    #[test]
    fn gpu_vs_cpu_cylindrical_end_to_end_parity_rotary_shaft_wrapped_and_4axis_sim() {
        let lib = default_starter_tool_library();
        let flat_6 = &lib.tools[0];
        let vbit = lib
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
            .unwrap();

        // Case 1: Rotary shaft turning + wrapped V-engraving + rapid/gouge/chuck faults
        let stl_shaft = sample_rotary_shaft_stl_bytes();
        let doc_shaft = load_mesh_cad_document(
            "rotary_shaft.stl",
            &stl_shaft,
            &MeshImportOptions::default(),
        )
        .unwrap();
        let mut setup_shaft = Setup::new("s_shaft_gpu", "Rotary Shaft GPU", WorkOffsetSlot::G54);
        setup_shaft.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup_shaft.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(44.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup_shaft.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
        setup_shaft.rotary.axis = RotaryAxisKind::XAxis;
        setup_shaft.rotary.chuck_length = ParamLength::from_mm(20.0);
        setup_shaft.rotary.chuck_diameter = ParamLength::from_mm(65.0);
        let res_shaft = setup_shaft
            .resolve(&doc_shaft, DisplayUnit::Millimeter)
            .unwrap();

        let mut bt = ToolpathBuilder::new("op_turn_wrap");
        bt.rotary_rapid_to(DVec3::new(-12.0, 0.0, 32.0), 0.0);
        bt.rotary_linear_to(
            DVec3::new(-12.0, 0.0, 19.0),
            0.0,
            400.0,
            19.0,
            MotionRole::Plunge,
        );
        bt.rotary_linear_to(
            DVec3::new(12.0, 0.0, 19.0),
            360.0,
            800.0,
            19.0,
            MotionRole::Cut,
        );
        // Rapid crash + target gouge + chuck collision
        bt.rotary_rapid_to(DVec3::new(0.0, 0.0, 11.0), 360.0);
        bt.rotary_rapid_to(DVec3::new(0.0, 0.0, 35.0), 360.0);
        bt.rotary_rapid_to(DVec3::new(-46.0, 0.0, 24.0), 360.0);
        let tp_turn = bt.finish();

        // Wrapped V-bit engraving groove around cylinder
        let mut bw = ToolpathBuilder::new("op_wrapped");
        bw.rotary_rapid_to(DVec3::new(-8.0, 0.0, 26.0), 45.0);
        bw.rotary_linear_to(
            DVec3::new(-8.0, 0.0, 17.5),
            45.0,
            350.0,
            17.5,
            MotionRole::Plunge,
        );
        bw.rotary_linear_to(
            DVec3::new(8.0, 0.0, 17.5),
            225.0,
            600.0,
            17.5,
            MotionRole::Cut,
        );
        bw.rotary_rapid_to(DVec3::new(8.0, 0.0, 26.0), 225.0);
        let tp_wrap = bw.finish();

        let ops_shaft = [
            SimOperationInput {
                op_id: "op_turn_wrap",
                op_name: "Rotary Turn & Faults",
                tool: flat_6,
                fallback_spindle_rpm: 10_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp_turn,
            },
            SimOperationInput {
                op_id: "op_wrapped",
                op_name: "Wrapped V-Engrave",
                tool: vbit,
                fallback_spindle_rpm: 14_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp_wrap,
            },
        ];
        let in_shaft = SetupSimulationInput {
            doc: &doc_shaft,
            setup: &res_shaft,
            target_bodies: &[],
            fixtures: &[],
            operations: &ops_shaft,
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            ..SetupSimulationInput::new(&doc_shaft, &res_shaft, &[])
        };
        let shaft_cpu = run_setup_simulation(&SetupSimulationInput {
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Cpu,
            ..in_shaft
        })
        .unwrap();
        let shaft_gpu = run_setup_simulation(&SetupSimulationInput {
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Gpu,
            ..in_shaft
        })
        .unwrap();
        assert!(shaft_gpu.active_backend.is_gpu);

        let shaft_vol_err =
            (shaft_gpu.remaining_stock_volume_mm3 - shaft_cpu.remaining_stock_volume_mm3).abs()
                / shaft_cpu.remaining_stock_volume_mm3.max(1.0);
        assert!(
            shaft_vol_err < 0.001,
            "Rotary shaft volume parity failed: cpu={:.3}, gpu={:.3}, err={:.5}%",
            shaft_cpu.remaining_stock_volume_mm3,
            shaft_gpu.remaining_stock_volume_mm3,
            shaft_vol_err * 100.0
        );
        let mut max_d_shaft = 0.0_f64;
        let mut worst_shaft = (0usize, 0usize, 0.0_f32, 0.0_f32, 0u32, 0u32);
        let sc = shaft_cpu.as_cylindrical().unwrap();
        let sg = shaft_gpu.as_cylindrical().unwrap();
        for (idx, (a, b)) in sc.r_stock.iter().zip(sg.r_stock.iter()).enumerate() {
            let d = (f64::from(*a) - f64::from(*b)).abs();
            if d > max_d_shaft {
                max_d_shaft = d;
                worst_shaft = (
                    idx / sc.ntheta,
                    idx % sc.ntheta,
                    *a,
                    *b,
                    sc.last_op_idx[idx],
                    sg.last_op_idx[idx],
                );
            }
        }
        assert!(
            max_d_shaft <= 5e-3,
            "Rotary shaft max cell delta {max_d_shaft:.6} mm > 5e-3 mm at (iu={}, it={}): cpu={} (op {}), gpu={} (op {})",
            worst_shaft.0,
            worst_shaft.1,
            worst_shaft.2,
            worst_shaft.4,
            worst_shaft.3,
            worst_shaft.5
        );
        assert_eq!(shaft_cpu.diagnostics.len(), shaft_gpu.diagnostics.len());
        for (dc, dg) in shaft_cpu
            .diagnostics
            .iter()
            .zip(shaft_gpu.diagnostics.iter())
        {
            assert_eq!(dc.kind, dg.kind);
            assert_eq!(dc.severity, dg.severity);
        }

        // Case 2: Simultaneous 4-axis off-centerline helical flute shaft + flank gouge & holder crash
        let stl_helical = chip_cad::sample_helical_flute_shaft_stl_bytes();
        let doc_helical = load_mesh_cad_document(
            "helical_flute_shaft.stl",
            &stl_helical,
            &MeshImportOptions::default(),
        )
        .unwrap();
        let mut setup_4ax = Setup::new("s_4ax_gpu", "Helical 4-Axis GPU", WorkOffsetSlot::G54);
        setup_4ax.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup_4ax.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(42.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup_4ax.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup_4ax.rotary.axis = RotaryAxisKind::XAxis;
        let res_4ax = setup_4ax
            .resolve(&doc_helical, DisplayUnit::Millimeter)
            .unwrap();

        let mut b4 = ToolpathBuilder::new("op_4ax_helical");
        b4.rotary_rapid_to(DVec3::new(-10.0, 7.5, 34.0), 0.0);
        b4.rotary_linear_to(
            DVec3::new(-10.0, 7.5, 10.0),
            0.0,
            400.0,
            12.0,
            MotionRole::Cut,
        );
        b4.rotary_linear_to(
            DVec3::new(12.0, 7.5, 10.0),
            60.0,
            400.0,
            12.0,
            MotionRole::Cut,
        );
        b4.rotary_rapid_to(DVec3::new(12.0, 7.5, 34.0), 60.0);
        b4.rotary_rapid_to(DVec3::new(-30.0, -7.0, 34.0), 60.0);
        b4.rotary_linear_to(
            DVec3::new(-30.0, -7.0, -24.0),
            60.0,
            300.0,
            20.0,
            MotionRole::Plunge,
        );
        let tp_4ax = b4.finish();

        let in_4ax = SetupSimulationInput {
            doc: &doc_helical,
            setup: &res_4ax,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_4ax_helical",
                op_name: "4-Axis Helical Flute",
                tool: flat_6,
                fallback_spindle_rpm: 10_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp_4ax,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            ..SetupSimulationInput::new(&doc_helical, &res_4ax, &[])
        };
        let sim4_cpu = run_setup_simulation(&SetupSimulationInput {
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Cpu,
            ..in_4ax
        })
        .unwrap();
        let sim4_gpu = run_setup_simulation(&SetupSimulationInput {
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Gpu,
            ..in_4ax
        })
        .unwrap();
        assert!(sim4_gpu.active_backend.is_gpu);

        let vol4_err = (sim4_gpu.remaining_stock_volume_mm3 - sim4_cpu.remaining_stock_volume_mm3)
            .abs()
            / sim4_cpu.remaining_stock_volume_mm3.max(1.0);
        assert!(
            vol4_err < 0.001,
            "4-axis helical volume parity failed: cpu={:.3}, gpu={:.3}, err={:.5}%",
            sim4_cpu.remaining_stock_volume_mm3,
            sim4_gpu.remaining_stock_volume_mm3,
            vol4_err * 100.0
        );
        let max_d_4ax = sim4_cpu
            .as_cylindrical()
            .unwrap()
            .r_stock
            .iter()
            .zip(sim4_gpu.as_cylindrical().unwrap().r_stock.iter())
            .map(|(a, b)| (f64::from(*a) - f64::from(*b)).abs())
            .fold(0.0_f64, f64::max);
        assert!(
            max_d_4ax <= 5e-3,
            "4-axis helical max cell delta {max_d_4ax:.6} mm > 5e-3 mm"
        );
        assert_eq!(
            sim4_cpu.diagnostics.len(),
            sim4_gpu.diagnostics.len(),
            "4-axis diag mismatch:\nCPU: {:#?}\nGPU: {:#?}",
            sim4_cpu.diagnostics,
            sim4_gpu.diagnostics
        );
        for (dc, dg) in sim4_cpu.diagnostics.iter().zip(sim4_gpu.diagnostics.iter()) {
            assert_eq!(dc.kind, dg.kind);
            assert_eq!(dc.severity, dg.severity);
        }

        // Case 3: TriDexel now executes end-to-end on GPU under Auto (Milestone 53)
        let empty_doc = CadDocument::empty();
        let mut setup_td = Setup::new("s_td_gpu", "TriDexel GPU Auto", WorkOffsetSlot::G54);
        setup_td.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup_td.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(100.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::default(),
        };
        let res_td = setup_td
            .resolve(&empty_doc, DisplayUnit::Millimeter)
            .unwrap();
        let sim_td = run_setup_simulation(&SetupSimulationInput {
            doc: &empty_doc,
            setup: &res_td,
            target_bodies: &[],
            fixtures: &[],
            operations: &[],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::TriDexel,
            backend_choice: SimComputeBackendChoice::Auto,
            ..SetupSimulationInput::new(&empty_doc, &res_td, &[])
        })
        .unwrap();
        assert!(sim_td.active_backend.is_gpu);
    }

    fn make_test_box_mesh_body(
        id: BodyId,
        name: &str,
        min: DVec3,
        max: DVec3,
    ) -> chip_cad::TriangleMeshBody {
        let vertices_mm = vec![
            DVec3::new(min.x, min.y, min.z),
            DVec3::new(max.x, min.y, min.z),
            DVec3::new(max.x, max.y, min.z),
            DVec3::new(min.x, max.y, min.z),
            DVec3::new(min.x, min.y, max.z),
            DVec3::new(max.x, min.y, max.z),
            DVec3::new(max.x, max.y, max.z),
            DVec3::new(min.x, max.y, max.z),
        ];
        let raw_tris: [[u32; 3]; 12] = [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [3, 7, 6],
            [3, 6, 2],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ];
        let triangles = raw_tris
            .into_iter()
            .map(|indices| {
                let v0 = vertices_mm[indices[0] as usize];
                let v1 = vertices_mm[indices[1] as usize];
                let v2 = vertices_mm[indices[2] as usize];
                chip_cad::MeshTriangle {
                    indices,
                    normal: (v1 - v0).cross(v2 - v0).normalize_or_zero(),
                    aabb: chip_math::Aabb3::from_points([v0, v1, v2]),
                }
            })
            .collect();
        let size = max - min;
        chip_cad::TriangleMeshBody {
            id,
            name: name.to_string(),
            format: chip_cad::MeshFormat::ThreeMf,
            detected_unit: Some(chip_cad::MeshUnitKind::Millimeter),
            applied_unit: chip_cad::MeshUnitKind::Millimeter,
            vertices_mm,
            triangles,
            aabb_mm: chip_math::Aabb3::new(min, max),
            diagnostics: chip_cad::MeshDiagnostics {
                raw_triangle_count: 12,
                valid_triangle_count: 12,
                degenerate_triangles_removed: 0,
                welded_vertex_count: 8,
                open_boundary_edges: 0,
                non_manifold_edges: 0,
                is_watertight: true,
                signed_volume_mm3: size.x * size.y * size.z,
            },
        }
    }

    #[cfg(feature = "gpu-compute")]
    fn make_test_sculpted_dome_mesh_body(id: BodyId) -> chip_cad::TriangleMeshBody {
        // Watertight 40 x 40 mm base block (Z = -12) topped with a smooth cosine dome (Z in [-6, -1])
        let n = 16usize;
        let x0 = -20.0;
        let y0 = -20.0;
        let step = 40.0 / n as f64;
        let z_bot = -12.0;
        let mut verts: Vec<DVec3> = Vec::new();
        let top_idx = |i: usize, j: usize| -> u32 { (j * (n + 1) + i) as u32 };
        let bot_base = ((n + 1) * (n + 1)) as u32;
        let bot_idx = |i: usize, j: usize| -> u32 { bot_base + (j * (n + 1) + i) as u32 };

        for j in 0..=n {
            let y = y0 + j as f64 * step;
            for i in 0..=n {
                let x = x0 + i as f64 * step;
                let r = (x * x + y * y).sqrt();
                let dome = if r < 16.0 {
                    5.0 * (0.5 * (1.0 + (PI * r / 16.0).cos()))
                } else {
                    0.0
                };
                verts.push(DVec3::new(x, y, -6.0 + dome));
            }
        }
        for j in 0..=n {
            let y = y0 + j as f64 * step;
            for i in 0..=n {
                let x = x0 + i as f64 * step;
                verts.push(DVec3::new(x, y, z_bot));
            }
        }

        let mut tris: Vec<[u32; 3]> = Vec::new();
        for j in 0..n {
            for i in 0..n {
                let a = top_idx(i, j);
                let b = top_idx(i + 1, j);
                let c = top_idx(i + 1, j + 1);
                let d = top_idx(i, j + 1);
                tris.push([a, b, c]);
                tris.push([a, c, d]);

                let ba = bot_idx(i, j);
                let bb = bot_idx(i + 1, j);
                let bc = bot_idx(i + 1, j + 1);
                let bd = bot_idx(i, j + 1);
                tris.push([ba, bc, bb]);
                tris.push([ba, bd, bc]);
            }
        }
        // Perimeter walls
        for i in 0..n {
            // y = y0 (-Y face)
            tris.push([bot_idx(i, 0), bot_idx(i + 1, 0), top_idx(i + 1, 0)]);
            tris.push([bot_idx(i, 0), top_idx(i + 1, 0), top_idx(i, 0)]);
            // y = +20 (+Y face)
            tris.push([bot_idx(i + 1, n), bot_idx(i, n), top_idx(i, n)]);
            tris.push([bot_idx(i + 1, n), top_idx(i, n), top_idx(i + 1, n)]);
        }
        for j in 0..n {
            // x = x0 (-X face)
            tris.push([bot_idx(0, j + 1), bot_idx(0, j), top_idx(0, j)]);
            tris.push([bot_idx(0, j + 1), top_idx(0, j), top_idx(0, j + 1)]);
            // x = +20 (+X face)
            tris.push([bot_idx(n, j), bot_idx(n, j + 1), top_idx(n, j + 1)]);
            tris.push([bot_idx(n, j), top_idx(n, j + 1), top_idx(n, j)]);
        }

        let triangles = tris
            .into_iter()
            .map(|indices| {
                let v0 = verts[indices[0] as usize];
                let v1 = verts[indices[1] as usize];
                let v2 = verts[indices[2] as usize];
                chip_cad::MeshTriangle {
                    indices,
                    normal: (v1 - v0).cross(v2 - v0).normalize_or_zero(),
                    aabb: chip_math::Aabb3::from_points([v0, v1, v2]),
                }
            })
            .collect();
        let aabb = chip_math::Aabb3::new(
            DVec3::new(-20.0, -20.0, z_bot),
            DVec3::new(20.0, 20.0, -1.0),
        );
        chip_cad::TriangleMeshBody {
            id,
            name: "Sculpted Dome".to_string(),
            format: chip_cad::MeshFormat::ThreeMf,
            detected_unit: Some(chip_cad::MeshUnitKind::Millimeter),
            applied_unit: chip_cad::MeshUnitKind::Millimeter,
            vertices_mm: verts,
            triangles,
            aabb_mm: aabb,
            diagnostics: chip_cad::MeshDiagnostics {
                raw_triangle_count: 0,
                valid_triangle_count: 0,
                degenerate_triangles_removed: 0,
                welded_vertex_count: 0,
                open_boundary_edges: 0,
                non_manifold_edges: 0,
                is_watertight: true,
                signed_volume_mm3: 0.0,
            },
        }
    }

    #[cfg(feature = "gpu-compute")]
    #[test]
    fn gpu_vs_cpu_tridexel_end_to_end_parity_prismatic_sculpted_and_threadmill() {
        let lib = default_starter_tool_library();
        let flat_6 = &lib.tools[0];
        let ball = lib
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::BallEndmill { .. }))
            .unwrap();
        let face = lib
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::FaceMill { .. }))
            .unwrap();

        // === Case 1: 2.5D Prismatic STEP plate + Rule 1/2/5 faults ===
        {
            let step = sample_machined_plate_step().unwrap();
            let doc = MonstertruckKernel
                .load_step_bytes("plate.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
                .unwrap();
            let mut setup = Setup::new("s_td_prism", "Prismatic TriDexel", WorkOffsetSlot::G54);
            let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
            let maabb = resolved.model_aabb_wcs;
            let c = maabb.center();

            let mut b0 = ToolpathBuilder::new("op_prism");
            b0.rapid_to(DVec3::new(c.x - 15.0, c.y - 8.0, maabb.max.z + 10.0));
            b0.linear_to(
                DVec3::new(c.x - 15.0, c.y - 8.0, maabb.max.z - 2.0),
                300.0,
                MotionRole::Plunge,
            );
            b0.linear_to(
                DVec3::new(c.x + 5.0, c.y - 8.0, maabb.max.z - 4.0),
                800.0,
                MotionRole::Cut,
            );
            b0.arc_xy_to(XyArcMove {
                end_wcs: DVec3::new(c.x + 5.0, c.y + 8.0, maabb.max.z - 4.0),
                center_xy: DVec2::new(c.x + 5.0, c.y),
                radius_mm: 8.0,
                ccw: true,
                turns: 0,
                feed_mm_min: 800.0,
                role: MotionRole::Cut,
            });
            // Deliberate rapid crash + target gouge
            b0.rapid_to(DVec3::new(c.x, c.y, maabb.max.z - 5.0));
            b0.rapid_to(DVec3::new(c.x, c.y, maabb.max.z + 10.0));
            let tp0 = b0.finish();

            // Non-center-cutting FaceMill plunge (Rule 5)
            let mut b1 = ToolpathBuilder::new("op_face_r5");
            b1.rapid_to(DVec3::new(c.x + 12.0, c.y - 10.0, maabb.max.z + 8.0));
            b1.linear_to(
                DVec3::new(c.x + 12.0, c.y - 10.0, maabb.max.z - 2.0),
                200.0,
                MotionRole::Plunge,
            );
            b1.rapid_to(DVec3::new(c.x + 12.0, c.y - 10.0, maabb.max.z + 8.0));
            let tp1 = b1.finish();

            let ops = [
                SimOperationInput {
                    op_id: "op_prism",
                    op_name: "Prismatic Slot & Fault",
                    tool: flat_6,
                    fallback_spindle_rpm: 12_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp0,
                },
                SimOperationInput {
                    op_id: "op_face_r5",
                    op_name: "Face Plunge Rule 5",
                    tool: face,
                    fallback_spindle_rpm: 8_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp1,
                },
            ];
            let input = SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                fixtures: &[],
                operations: &ops,
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                initial_stock: None,
                initial_raw_volume_mm3: None,
                carried_over_from_setup: None,
                cancel: None,
                ..SetupSimulationInput::new(&doc, &resolved, &[])
            };
            let sim_cpu = run_setup_simulation(&SetupSimulationInput {
                engine: SimEngineKind::TriDexel,
                backend_choice: SimComputeBackendChoice::Cpu,
                ..input
            })
            .unwrap();
            let sim_gpu = run_setup_simulation(&SetupSimulationInput {
                engine: SimEngineKind::TriDexel,
                backend_choice: SimComputeBackendChoice::Gpu,
                ..input
            })
            .unwrap();
            assert!(!sim_cpu.active_backend.is_gpu);
            assert!(sim_gpu.active_backend.is_gpu);

            let vol_err = (sim_gpu.remaining_stock_volume_mm3 - sim_cpu.remaining_stock_volume_mm3)
                .abs()
                / sim_cpu.remaining_stock_volume_mm3.max(1.0);
            assert!(
                vol_err < 0.001,
                "Prismatic TriDexel vol err {:.4}% > 0.1%",
                vol_err * 100.0
            );

            let rc = sim_cpu.error_report.as_ref().unwrap();
            let rg = sim_gpu.error_report.as_ref().unwrap();
            assert!(
                (rg.rms_error_mm - rc.rms_error_mm).abs() < 0.005,
                "Prismatic TriDexel RMS mismatch: cpu={:.5}, gpu={:.5}",
                rc.rms_error_mm,
                rg.rms_error_mm
            );
            assert!(
                (rg.mean_abs_error_mm - rc.mean_abs_error_mm).abs() < 0.005,
                "Prismatic TriDexel MAE mismatch: cpu={:.5}, gpu={:.5}",
                rc.mean_abs_error_mm,
                rg.mean_abs_error_mm
            );
            assert!((rg.in_tolerance_surface_pct - rc.in_tolerance_surface_pct).abs() < 0.5);
            assert_eq!(
                sim_cpu.diagnostics.len(),
                sim_gpu.diagnostics.len(),
                "Prismatic TriDexel diag mismatch:\nCPU: {:#?}\nGPU: {:#?}",
                sim_cpu.diagnostics,
                sim_gpu.diagnostics
            );
            for (dc, dg) in sim_cpu.diagnostics.iter().zip(sim_gpu.diagnostics.iter()) {
                assert_eq!(dc.kind, dg.kind);
                assert_eq!(dc.severity, dg.severity);
                assert_eq!(dc.op_idx, dg.op_idx);
            }
        }

        // === Case 2: 3-axis sculpted dome mesh with BallEndmill 3D finishing passes ===
        {
            let dome = make_test_sculpted_dome_mesh_body(BodyId(0));
            let mut doc = CadDocument::empty();
            doc.source_name = "dome.stl".to_string();
            doc.aabb_mm = dome.aabb_mm;
            doc.meshes = vec![dome];

            let mut setup = Setup::new("s_td_dome", "Sculpted Dome", WorkOffsetSlot::G54);
            setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
                anchor: BoxAnchor27::TOP_CENTER,
            };
            setup.stock = StockConfig::FixedBox {
                width_x: ParamLength::from_mm(44.0),
                depth_y: ParamLength::from_mm(44.0),
                height_z: ParamLength::from_mm(14.0),
                xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
                z_align: chip_cam::FixedBoxZAlign::default(),
            };
            let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

            let mut bd = ToolpathBuilder::new("op_dome_3d");
            bd.rapid_to(DVec3::new(-18.0, -14.0, 10.0));
            for row in 0..8 {
                let y = -14.0 + row as f64 * 4.0;
                let dir = if row % 2 == 0 { 1.0 } else { -1.0 };
                for step in 0..=12 {
                    let u = step as f64 / 12.0;
                    let x = if dir > 0.0 {
                        -18.0 + 36.0 * u
                    } else {
                        18.0 - 36.0 * u
                    };
                    let r = (x * x + y * y).sqrt();
                    let dome_z = if r < 16.0 {
                        -5.0 + 5.0 * (0.5 * (1.0 + (PI * r / 16.0).cos()))
                    } else {
                        -5.0
                    };
                    bd.linear_to(DVec3::new(x, y, dome_z), 1200.0, MotionRole::FinishCut);
                }
            }
            bd.rapid_to(DVec3::new(18.0, 14.0, 10.0));
            let tp_dome = bd.finish();

            let ops = [SimOperationInput {
                op_id: "op_dome_3d",
                op_name: "3D Ball Parallel",
                tool: ball,
                fallback_spindle_rpm: 16_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp_dome,
            }];
            let input = SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                fixtures: &[],
                operations: &ops,
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                initial_stock: None,
                initial_raw_volume_mm3: None,
                carried_over_from_setup: None,
                cancel: None,
                ..SetupSimulationInput::new(&doc, &resolved, &[])
            };
            let sim_cpu = run_setup_simulation(&SetupSimulationInput {
                engine: SimEngineKind::TriDexel,
                backend_choice: SimComputeBackendChoice::Cpu,
                ..input
            })
            .unwrap();
            let sim_gpu = run_setup_simulation(&SetupSimulationInput {
                engine: SimEngineKind::TriDexel,
                backend_choice: SimComputeBackendChoice::Gpu,
                ..input
            })
            .unwrap();
            assert!(sim_gpu.active_backend.is_gpu);

            let vol_err = (sim_gpu.remaining_stock_volume_mm3 - sim_cpu.remaining_stock_volume_mm3)
                .abs()
                / sim_cpu.remaining_stock_volume_mm3.max(1.0);
            assert!(
                vol_err < 0.001,
                "Sculpted dome TriDexel vol err {:.4}% > 0.1%",
                vol_err * 100.0
            );
            let rc = sim_cpu.error_report.as_ref().unwrap();
            let rg = sim_gpu.error_report.as_ref().unwrap();
            assert!(
                (rg.rms_error_mm - rc.rms_error_mm).abs() < 0.005,
                "Sculpted dome RMS mismatch: cpu={:.5}, gpu={:.5}",
                rc.rms_error_mm,
                rg.rms_error_mm
            );
            assert!(
                (rg.mean_abs_error_mm - rc.mean_abs_error_mm).abs() < 0.005,
                "Sculpted dome MAE mismatch: cpu={:.5}, gpu={:.5}",
                rc.mean_abs_error_mm,
                rg.mean_abs_error_mm
            );
        }

        // === Case 3: Helical ThreadMill undercut (true 3D multi-span internal threads) ===
        {
            let empty_doc = CadDocument::empty();
            let mut setup = Setup::new("s_td_tm", "ThreadMill Helical", WorkOffsetSlot::G54);
            setup.wcs.origin = WcsOriginMode::StockBoxPoint {
                anchor: BoxAnchor27::TOP_CENTER,
            };
            setup.stock = StockConfig::FixedBox {
                width_x: ParamLength::from_mm(100.0),
                depth_y: ParamLength::from_mm(100.0),
                height_z: ParamLength::from_mm(20.0),
                xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
                z_align: chip_cam::FixedBoxZAlign::default(),
            };
            let resolved = setup.resolve(&empty_doc, DisplayUnit::Millimeter).unwrap();

            // Pre-drill pilot bore with flat endmill, then cut helical multi-tooth internal thread
            let mut bp = ToolpathBuilder::new("op_pilot");
            bp.rapid_to(DVec3::new(0.0, 0.0, 6.0));
            bp.linear_to(DVec3::new(0.0, 0.0, -16.0), 300.0, MotionRole::Plunge);
            // Circular bore of R=1.5 with 6mm flat endmill -> 9mm pilot bore
            for k in 0..=24 {
                let ang = (k as f64 / 24.0) * std::f64::consts::TAU;
                bp.linear_to(
                    DVec3::new(1.5 * ang.cos(), 1.5 * ang.sin(), -16.0),
                    600.0,
                    MotionRole::Cut,
                );
            }
            bp.rapid_to(DVec3::new(0.0, 0.0, 6.0));
            let tp_pilot = bp.finish();

            let tm_tool = chip_tools::Tool {
                id: chip_tools::ToolId("tm_m12".to_string()),
                tool_number: 9,
                length_offset_number: 9,
                diameter_offset_number: 9,
                name: "M12 Multi-Form ThreadMill".to_string(),
                vendor: None,
                product_id: None,
                native_unit: DisplayUnit::Millimeter,
                spindle_direction: chip_tools::SpindleDirection::Cw,
                default_coolant: chip_tools::CoolantMode::Flood,
                stickout_length_mm: 24.0,
                geometry: CutterGeometry::ThreadMill {
                    diameter_mm: 7.5,
                    neck_diameter_mm: 4.0,
                    neck_length_mm: 18.0,
                    pitch_mm: 1.75,
                    teeth_count: 5,
                    thread_angle_deg: 60.0,
                    flute_length_mm: 9.0,
                    overall_length_mm: 55.0,
                    shank_diameter_mm: 6.0,
                    flutes: 3,
                },
                presets: Vec::new(),
            };

            let mut bt = ToolpathBuilder::new("op_thread");
            bt.rapid_to(DVec3::new(0.0, 0.0, 6.0));
            bt.linear_to(DVec3::new(0.0, 0.0, -14.0), 400.0, MotionRole::Plunge);
            let r_hel = 2.25; // cuts to radius 2.25 + 3.75 = 6.0 mm (M12 major radius)
            bt.linear_to(DVec3::new(r_hel, 0.0, -14.0), 300.0, MotionRole::LeadIn);
            let steps = 36usize;
            for k in 1..=steps {
                let u = k as f64 / steps as f64;
                let ang = u * std::f64::consts::TAU;
                let z = -14.0 + 1.75 * u;
                bt.linear_to(
                    DVec3::new(r_hel * ang.cos(), r_hel * ang.sin(), z),
                    500.0,
                    MotionRole::Cut,
                );
            }
            bt.linear_to(DVec3::new(0.0, 0.0, -12.25), 400.0, MotionRole::LeadOut);
            bt.rapid_to(DVec3::new(0.0, 0.0, 6.0));
            let tp_thread = bt.finish();

            let ops = [
                SimOperationInput {
                    op_id: "op_pilot",
                    op_name: "Pilot Bore",
                    tool: flat_6,
                    fallback_spindle_rpm: 12_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp_pilot,
                },
                SimOperationInput {
                    op_id: "op_thread",
                    op_name: "Helical ThreadMill",
                    tool: &tm_tool,
                    fallback_spindle_rpm: 8_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp_thread,
                },
            ];
            let input = SetupSimulationInput {
                doc: &empty_doc,
                setup: &resolved,
                target_bodies: &[],
                fixtures: &[],
                operations: &ops,
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                initial_stock: None,
                initial_raw_volume_mm3: None,
                carried_over_from_setup: None,
                cancel: None,
                ..SetupSimulationInput::new(&empty_doc, &resolved, &[])
            };
            let sim_cpu = run_setup_simulation(&SetupSimulationInput {
                engine: SimEngineKind::TriDexel,
                backend_choice: SimComputeBackendChoice::Cpu,
                ..input
            })
            .unwrap();
            let sim_gpu = run_setup_simulation(&SetupSimulationInput {
                engine: SimEngineKind::TriDexel,
                backend_choice: SimComputeBackendChoice::Gpu,
                ..input
            })
            .unwrap();
            assert!(sim_gpu.active_backend.is_gpu);

            let vol_err = (sim_gpu.remaining_stock_volume_mm3 - sim_cpu.remaining_stock_volume_mm3)
                .abs()
                / sim_cpu.remaining_stock_volume_mm3.max(1.0);
            assert!(
                vol_err < 0.001,
                "ThreadMill helical TriDexel vol err {:.4}% > 0.1% (cpu={:.2}, gpu={:.2})",
                vol_err * 100.0,
                sim_cpu.remaining_stock_volume_mm3,
                sim_gpu.remaining_stock_volume_mm3
            );
            assert_eq!(sim_cpu.diagnostics.len(), sim_gpu.diagnostics.len());
        }
    }

    #[cfg(feature = "gpu-compute")]
    #[test]
    fn gpu_bvh_multibody_stl_target_bodies_filter_matches_cpu_tridexel_and_cylindrical() {
        // Two disjoint 20 x 20 x 10 mm mesh bodies inside the same CadDocument (including
        // MESH_FACE_SENTINEL pick_triangles to guard against multi-body bleed-through).
        let mesh_left = make_test_box_mesh_body(
            BodyId(0),
            "Left Part",
            DVec3::new(0.0, 0.0, -10.0),
            DVec3::new(20.0, 20.0, 0.0),
        );
        let mesh_right = make_test_box_mesh_body(
            BodyId(1),
            "Right Part",
            DVec3::new(30.0, 0.0, -10.0),
            DVec3::new(50.0, 20.0, 0.0),
        );

        let mut render_mesh = chip_cad::build_render_mesh_for_triangle_mesh(
            &mesh_left,
            35.0,
            chip_cad::MESH_FACE_SENTINEL,
        );
        let rm_right = chip_cad::build_render_mesh_for_triangle_mesh(
            &mesh_right,
            35.0,
            chip_cad::MESH_FACE_SENTINEL,
        );
        let v_off = render_mesh.vertices.len() as u32;
        render_mesh.vertices.extend(rm_right.vertices);
        render_mesh
            .indices
            .extend(rm_right.indices.into_iter().map(|i| i + v_off));
        render_mesh.pick_triangles.extend(rm_right.pick_triangles);

        let full_aabb =
            chip_math::Aabb3::new(DVec3::new(0.0, 0.0, -10.0), DVec3::new(50.0, 20.0, 0.0));
        let mut doc = CadDocument::empty();
        doc.source_name = "two_bodies.3mf".to_string();
        doc.meshes = vec![mesh_left, mesh_right];
        doc.aabb_mm = full_aabb;
        doc.render_mesh = render_mesh;

        let mut setup = Setup::new("s_mb_gpu", "MultiBody GPU BVH", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27 {
                x: AxisAnchor::Min,
                y: AxisAnchor::Min,
                z: AxisAnchor::Max,
            },
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(54.0),
            depth_y: ParamLength::from_mm(24.0),
            height_z: ParamLength::from_mm(14.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::default(),
        };
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let input_right = SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[BodyId(1)],
            fixtures: &[],
            operations: &[],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        };
        let cpu_td = run_setup_simulation(&SetupSimulationInput {
            engine: SimEngineKind::TriDexel,
            backend_choice: SimComputeBackendChoice::Cpu,
            ..input_right
        })
        .unwrap();
        let gpu_td = run_setup_simulation(&SetupSimulationInput {
            engine: SimEngineKind::TriDexel,
            backend_choice: SimComputeBackendChoice::Gpu,
            ..input_right
        })
        .unwrap();
        assert!(gpu_td.active_backend.is_gpu);

        let tgt_cpu = cpu_td.target_tridexel_stock.as_ref().unwrap();
        let tgt_gpu = gpu_td.target_tridexel_stock.as_ref().unwrap();
        assert!(
            (tgt_gpu.compute_volume_mm3() - 4_000.0).abs() < 100.0,
            "GPU BVH filtered BodyId(1) target vol expected ~4000, got {:.1}",
            tgt_gpu.compute_volume_mm3()
        );
        assert!(
            (tgt_gpu.compute_volume_mm3() - tgt_cpu.compute_volume_mm3()).abs() < 50.0,
            "GPU vs CPU filtered BodyId(1) target vol mismatch: gpu={:.1}, cpu={:.1}",
            tgt_gpu.compute_volume_mm3(),
            tgt_cpu.compute_volume_mm3()
        );
        assert!(
            tgt_gpu.sample_top_z(10.0, 10.0).is_none(),
            "excluded BodyId(0) must NOT appear in GPU BVH TriDexel target"
        );
        assert!(
            tgt_gpu.sample_top_z(40.0, 10.0).is_some(),
            "selected BodyId(1) must appear in GPU BVH TriDexel target"
        );

        let rep_cpu = cpu_td.error_report.as_ref().unwrap();
        let rep_gpu = gpu_td.error_report.as_ref().unwrap();
        assert!(
            (rep_gpu.target_volume_mm3 - rep_cpu.target_volume_mm3).abs() < 50.0,
            "GPU vs CPU report target vol mismatch"
        );
        assert!(
            (rep_gpu.excess_volume_mm3 - rep_cpu.excess_volume_mm3).abs() < 50.0,
            "GPU vs CPU report excess vol mismatch"
        );

        // Also check Cylindrical GPU BVH target rasterization honors target_bodies filter!
        let mut setup_cyl = Setup::new("s_mb_cyl", "MultiBody Cyl", WorkOffsetSlot::G54);
        setup_cyl.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup_cyl.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(40.0),
            length: ParamLength::from_mm(60.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup_cyl.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
        setup_cyl.rotary.axis = RotaryAxisKind::XAxis;
        let res_cyl = setup_cyl.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let in_cyl = SetupSimulationInput {
            doc: &doc,
            setup: &res_cyl,
            target_bodies: &[BodyId(1)],
            fixtures: &[],
            operations: &[],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            ..SetupSimulationInput::new(&doc, &res_cyl, &[])
        };
        let cpu_cyl = run_setup_simulation(&SetupSimulationInput {
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Cpu,
            ..in_cyl
        })
        .unwrap();
        let gpu_cyl = run_setup_simulation(&SetupSimulationInput {
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Gpu,
            ..in_cyl
        })
        .unwrap();
        assert!(gpu_cyl.active_backend.is_gpu);
        let cc = cpu_cyl.as_cylindrical().unwrap();
        let cg = gpu_cyl.as_cylindrical().unwrap();
        let mut max_rt_diff = 0.0_f64;
        let mut hit_count = 0usize;
        for (a, b) in cc.r_target.iter().zip(cg.r_target.iter()) {
            assert_eq!(*a >= 0.0, *b >= 0.0);
            if *a >= 0.0 {
                hit_count += 1;
                max_rt_diff = max_rt_diff.max((f64::from(*a) - f64::from(*b)).abs());
            }
        }
        assert!(hit_count > 100);
        assert!(
            max_rt_diff < 5e-3,
            "Multi-body Cylindrical GPU BVH r_target max diff {max_rt_diff:.5} mm"
        );
    }

    #[test]
    fn rotary_4axis_sim_and_2linear_quality_oracle_zero_staircase_bias_and_gouge_free() {
        use std::sync::atomic::AtomicBool;

        use chip_cad::sample_helical_flute_shaft_stl_bytes;
        use chip_cam::{
            Operation, OperationGenRequest, OperationKind, Rotary4AxisPattern,
            Rotary4AxisSimParams, RotaryPatternMode, RotaryTiltMode, StrategyParams,
            generate_operation_toolpath_with_rest,
        };
        use chip_tools::CutterGeometry;

        let lib = default_starter_tool_library();
        let ball_6 = lib
            .tools
            .iter()
            .find(|t| {
                matches!(
                    t.geometry,
                    CutterGeometry::BallEndmill { diameter_mm, .. } if approx_eq(diameter_mm, 6.0, 0.5)
                )
            })
            .expect("6mm ball endmill in starter library");
        let cancel = AtomicBool::new(false);

        // === 1. Simultaneous 4-Axis Helical Flute (`Rotary4AxisSim`, Auto-Tilt + Slope-Compensated Cusp) ===
        {
            let stl = sample_helical_flute_shaft_stl_bytes();
            let doc =
                load_mesh_cad_document("helical_flute.stl", &stl, &MeshImportOptions::default())
                    .unwrap();

            let mut setup = Setup::new("s_m56_4ax", "M56 4-Axis Quality", WorkOffsetSlot::G54);
            setup.wcs.origin = WcsOriginMode::StockBoxPoint {
                anchor: BoxAnchor27::VOLUMETRIC_CENTER,
            };
            setup.stock = StockConfig::FixedCylinder {
                axis: StockAxis::X,
                diameter: ParamLength::from_mm(42.0),
                length: ParamLength::from_mm(80.0),
                axial_top_offset: ParamLength::from_mm(0.0),
            };
            setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
            setup.rotary.axis = RotaryAxisKind::XAxis;
            setup.rotary.chuck_length = ParamLength::from_mm(20.0);
            setup.rotary.chuck_diameter = ParamLength::from_mm(65.0);
            let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

            let mut op = Operation::new(
                "op_4ax_fin",
                "4-Axis Helical Finish (Cusp)",
                OperationKind::Rotary4AxisSim,
            );
            op.tool_id = Some(ball_6.id.clone());
            op.passes.multiple_depths = false;
            op.strategy = Some(StrategyParams::Rotary4AxisSim(Rotary4AxisSimParams {
                pattern: Rotary4AxisPattern::SimultaneousSpiral,
                tilt_mode: RotaryTiltMode::SurfaceNormalAutoTilt,
                max_tilt_deg: 18.0,
                tilt_smoothing_window: 3,
                use_cusp_height: true,
                cusp_height: ParamLength::from_mm(0.015),
                stepover: ParamLength::from_mm(1.5),
                axial_boundary: chip_cam::RotaryAxialBoundaryConfig {
                    axial_margin_start: ParamLength::from_mm(11.0),
                    axial_margin_end: ParamLength::from_mm(11.0),
                    ..Default::default()
                },
                ..Default::default()
            }));

            let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
                doc: &doc,
                sketches: &[],
                setup: &resolved,
                op: &op,
                tool: ball_6,
                clearing_tool: None,
                display_unit: DisplayUnit::Millimeter,
                is_first_tool_in_setup: true,
                upstream_rest: &[],
                carryover_stock: None,
                fixtures: &[],
                shared_avoid_regions: &[],
                cancel: &cancel,
            })
            .unwrap();

            let ops = [SimOperationInput {
                op_id: &op.id,
                op_name: &op.name,
                tool: ball_6,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }];
            let input = SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                fixtures: &[],
                operations: &ops,
                resolution: SimResolution::Fine,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                initial_stock: None,
                initial_raw_volume_mm3: None,
                carried_over_from_setup: None,
                cancel: None,
                ..SetupSimulationInput::new(&doc, &resolved, &[])
            };
            let sim = run_setup_simulation(&SetupSimulationInput {
                engine: SimEngineKind::Heightmap,
                backend_choice: SimComputeBackendChoice::Auto,
                ..input
            })
            .unwrap();
            assert_eq!(
                sim.critical_error_count(),
                0,
                "Rotary4AxisSim must have zero critical errors: {:#?}",
                sim.diagnostics
            );

            // Evaluate machined-zone (`u ∈ [-27.5, +27.5]`) surface accuracy directly on CylindricalStock
            let cyl = sim.as_cylindrical().unwrap();
            let mut sum_sq = 0.0_f64;
            let mut sum_signed = 0.0_f64;
            let mut max_gouge = 0.0_f64;
            let mut in_tol = 0usize;
            let mut total = 0usize;
            for iu in 0..cyl.nu {
                let u = cyl.u_min + (iu as f64 + 0.5) * cyl.du;
                if !(-27.5..=27.5).contains(&u) {
                    continue;
                }
                for it in 0..cyl.ntheta {
                    let idx = iu * cyl.ntheta + it;
                    let rt = f64::from(cyl.r_target[idx]);
                    if rt < 0.0 {
                        continue;
                    }
                    let dev = f64::from(cyl.r_stock[idx]) - rt;
                    sum_sq += dev * dev;
                    sum_signed += dev;
                    if dev < 0.0 {
                        max_gouge = max_gouge.max(-dev);
                    }
                    if dev.abs() <= 0.05 {
                        in_tol += 1;
                    }
                    total += 1;
                }
            }
            assert!(total > 10_000);
            let rms = (sum_sq / total as f64).sqrt();
            let bias = sum_signed / total as f64;
            let in_tol_pct = 100.0 * (in_tol as f64) / (total as f64);
            assert!(
                rms <= 0.08,
                "Rotary4AxisSim machined-zone RMS expected <= 0.08 mm, got {rms:.4} mm"
            );
            assert!(
                bias.abs() < 0.03,
                "Rotary4AxisSim staircase bias expected |bias| < 0.03 mm, got {bias:+.4} mm"
            );
            assert!(
                max_gouge <= 0.015,
                "Rotary4AxisSim max_gouge expected <= 0.015 mm, got {max_gouge:.4} mm"
            );
            assert!(
                in_tol_pct >= 85.0,
                "Rotary4AxisSim machined-zone In-Tol (±0.05) expected >= 85%, got {in_tol_pct:.1}%"
            );
        }

        // === 2. 2-Linear + Rotary 3-Lobe Cam & Collar (`Rotary2Linear`, Slope-Compensated Cusp) ===
        {
            let stl = sample_rotary_shaft_stl_bytes();
            let doc =
                load_mesh_cad_document("rotary_shaft.stl", &stl, &MeshImportOptions::default())
                    .unwrap();

            let mut setup = Setup::new("s_m56_2lin", "M56 2-Linear Quality", WorkOffsetSlot::G54);
            setup.wcs.origin = WcsOriginMode::StockBoxPoint {
                anchor: BoxAnchor27::VOLUMETRIC_CENTER,
            };
            setup.stock = StockConfig::FixedCylinder {
                axis: StockAxis::X,
                diameter: ParamLength::from_mm(42.0),
                length: ParamLength::from_mm(80.0),
                axial_top_offset: ParamLength::from_mm(0.0),
            };
            setup.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
            setup.rotary.axis = RotaryAxisKind::XAxis;
            setup.rotary.chuck_length = ParamLength::from_mm(20.0);
            setup.rotary.chuck_diameter = ParamLength::from_mm(65.0);
            let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

            let mut op = Operation::new(
                "op_2lin_fin",
                "Rotary 2-Linear Finish (Cusp)",
                OperationKind::Rotary2Linear,
            );
            op.tool_id = Some(ball_6.id.clone());
            op.passes.multiple_depths = false;
            if let StrategyParams::Rotary2Linear(p) = op.strategy_mut() {
                p.pattern = RotaryPatternMode::ContinuousSpiral;
                p.use_cusp_height = true;
                p.cusp_height = ParamLength::from_mm(0.015);
                p.stepover = ParamLength::from_mm(1.5);
                p.axial_boundary.axial_margin_start = ParamLength::from_mm(34.0);
                p.axial_boundary.axial_margin_end = ParamLength::from_mm(8.0);
            }

            let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
                doc: &doc,
                sketches: &[],
                setup: &resolved,
                op: &op,
                tool: ball_6,
                clearing_tool: None,
                display_unit: DisplayUnit::Millimeter,
                is_first_tool_in_setup: true,
                upstream_rest: &[],
                carryover_stock: None,
                fixtures: &[],
                shared_avoid_regions: &[],
                cancel: &cancel,
            })
            .unwrap();

            let ops = [SimOperationInput {
                op_id: &op.id,
                op_name: &op.name,
                tool: ball_6,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }];
            let input = SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                fixtures: &[],
                operations: &ops,
                resolution: SimResolution::Fine,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                initial_stock: None,
                initial_raw_volume_mm3: None,
                carried_over_from_setup: None,
                cancel: None,
                ..SetupSimulationInput::new(&doc, &resolved, &[])
            };
            let sim = run_setup_simulation(&SetupSimulationInput {
                engine: SimEngineKind::Heightmap,
                backend_choice: SimComputeBackendChoice::Auto,
                ..input
            })
            .unwrap();
            assert_eq!(
                sim.critical_error_count(),
                0,
                "Rotary2Linear must have zero critical errors: {:#?}",
                sim.diagnostics
            );

            // Evaluate machined collar + 3-lobe cam (clear of unavoidable R=3 mm ball corner fillets at x=-2.5/+7.5)
            let cyl = sim.as_cylindrical().unwrap();
            let mut sum_sq = 0.0_f64;
            let mut sum_signed = 0.0_f64;
            let mut max_gouge = 0.0_f64;
            let mut in_tol = 0usize;
            let mut total = 0usize;
            for iu in 0..cyl.nu {
                let u = cyl.u_min + (iu as f64 + 0.5) * cyl.du;
                // Sample across the turned collar floor (`[0.0, 5.0]`) + 3-lobe cam (`[14.0, 26.0]`),
                // clear of unavoidable R=3 mm ball-radius corner fillets at x=-2.5, +7.5, +12.0, +28.0
                if !((0.0..=5.0).contains(&u) || (14.0..=26.0).contains(&u)) {
                    continue;
                }
                for it in 0..cyl.ntheta {
                    let idx = iu * cyl.ntheta + it;
                    let rt = f64::from(cyl.r_target[idx]);
                    if rt < 0.0 {
                        continue;
                    }
                    let dev = f64::from(cyl.r_stock[idx]) - rt;
                    sum_sq += dev * dev;
                    sum_signed += dev;
                    if dev < 0.0 {
                        max_gouge = max_gouge.max(-dev);
                    }
                    if dev.abs() <= 0.05 {
                        in_tol += 1;
                    }
                    total += 1;
                }
            }
            assert!(total > 10_000);
            let rms = (sum_sq / total as f64).sqrt();
            let bias = sum_signed / total as f64;
            let in_tol_pct = 100.0 * (in_tol as f64) / (total as f64);
            assert!(
                rms <= 0.05,
                "Rotary2Linear machined-zone RMS expected <= 0.05 mm, got {rms:.4} mm"
            );
            assert!(
                bias.abs() < 0.02,
                "Rotary2Linear staircase bias expected |bias| < 0.02 mm, got {bias:+.4} mm"
            );
            assert!(
                max_gouge <= 0.015,
                "Rotary2Linear max_gouge expected <= 0.015 mm, got {max_gouge:.4} mm"
            );
            assert!(
                in_tol_pct >= 90.0,
                "Rotary2Linear machined-zone In-Tol (±0.05) expected >= 90%, got {in_tol_pct:.1}%"
            );
        }
    }

    #[test]
    fn indexed_step_cylindrical_target_metrics_non_zero_rms_and_cpu_gpu_parity() {
        use std::sync::atomic::AtomicBool;

        use chip_cad::{SurfaceKind, tutorial_indexed_4side_block_step};
        use chip_cam::{
            FixedBoxXyAlign, FixedBoxZAlign, GeometrySelectionItem, Operation, OperationGenRequest,
            OperationKind, OperationRotaryMode, compute_index_angle_for_normal,
            generate_operation_toolpath_with_rest,
        };

        let step = tutorial_indexed_4side_block_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes(
                "11-indexed-multiface-block.step",
                step.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .unwrap();

        let mut setup = Setup::new("s_idx_step", "Indexed 3+1 STEP", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(90.0),
            depth_y: ParamLength::from_mm(36.0),
            height_z: ParamLength::from_mm(36.0),
            xy_align: FixedBoxXyAlign::CenterXy,
            z_align: FixedBoxZAlign::CenterZ,
        };
        setup.rotary.mode = RotaryDriveMode::Indexed3Plus1;
        setup.rotary.axis = RotaryAxisKind::XAxis;
        setup.rotary.clearance_radial_offset = ParamLength::from_mm(10.0);
        setup.rotary.chuck_length = ParamLength::from_mm(20.0);
        setup.rotary.chuck_diameter = ParamLength::from_mm(65.0);
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let rot = resolved.rotary.unwrap();

        let lib = default_starter_tool_library();
        let flat_6 = &lib.tools[0];
        let cancel = AtomicBool::new(false);

        // Generate all 5 indexed planar face operations (0°, 45°, 90°, 180°, 270°)
        let mut generated = Vec::new();
        for face in &doc.faces {
            let SurfaceKind::Plane { normal, .. } = face.surface else {
                continue;
            };
            let n_wcs = resolved.frame.world_to_local_dir(normal);
            let Some(angle_deg) = compute_index_angle_for_normal(n_wcs, &rot) else {
                continue;
            };
            let Some(binding) = doc.bind_face(face.id) else {
                continue;
            };
            let mut op = Operation::new(
                format!("op_idx_{}", angle_deg.round() as i32),
                format!("Indexed {angle_deg:.0}°"),
                OperationKind::Pocket2D,
            );
            op.tool_id = Some(flat_6.id.clone());
            op.selections
                .push(GeometrySelectionItem::PlanarFace { binding });
            op.passes.multiple_depths = true;
            op.passes.max_stepdown = ParamLength::from_mm(2.5);
            op.rotary = OperationRotaryMode::Indexed3Plus1 { angle_deg };
            let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
                doc: &doc,
                sketches: &[],
                setup: &resolved,
                op: &op,
                tool: flat_6,
                clearing_tool: None,
                display_unit: DisplayUnit::Millimeter,
                is_first_tool_in_setup: generated.is_empty(),
                upstream_rest: &[],
                carryover_stock: None,
                fixtures: &[],
                shared_avoid_regions: &[],
                cancel: &cancel,
            })
            .unwrap();
            generated.push((op, tp));
        }
        assert_eq!(generated.len(), 5, "expected 5 indexed planar faces");

        let sim_ops: Vec<SimOperationInput<'_>> = generated
            .iter()
            .map(|(op, tp)| SimOperationInput {
                op_id: &op.id,
                op_name: &op.name,
                tool: flat_6,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: tp,
            })
            .collect();

        let input = SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &sim_ops,
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        };

        let cpu = run_setup_simulation(&SetupSimulationInput {
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Cpu,
            ..input
        })
        .unwrap();
        assert_eq!(cpu.critical_error_count(), 0);
        let rc = cpu
            .error_report
            .as_ref()
            .expect("11-indexed-step must produce a Cylindrical StockErrorReport");
        assert!(
            rc.rms_error_mm > 5.0,
            "11-indexed-step RMS must reflect uncut bar ends (non-zero), got {:.4}",
            rc.rms_error_mm
        );
        assert!(
            rc.max_excess_mm > 20.0,
            "max_excess must reflect 36x36 bar stub corner, got {:.3}",
            rc.max_excess_mm
        );
        assert!(
            rc.max_gouge_mm < 0.01,
            "indexed STEP faces must be gouge-free, got {:.5}",
            rc.max_gouge_mm
        );
        assert!(
            (70.0..=85.0).contains(&rc.in_tolerance_surface_pct),
            "expected ~77.7% in-tolerance surface (70/90 mm machined), got {:.2}%",
            rc.in_tolerance_surface_pct
        );

        #[cfg(feature = "gpu-compute")]
        if GpuSimContext::shared_headless().is_some() {
            let gpu = run_setup_simulation(&SetupSimulationInput {
                engine: SimEngineKind::Heightmap,
                backend_choice: SimComputeBackendChoice::Gpu,
                ..input
            })
            .unwrap();
            assert!(gpu.active_backend.is_gpu);
            assert_eq!(gpu.critical_error_count(), 0);
            let rg = gpu.error_report.as_ref().unwrap();
            let vol_err = (gpu.remaining_stock_volume_mm3 - cpu.remaining_stock_volume_mm3).abs()
                / cpu.remaining_stock_volume_mm3.max(1.0);
            assert!(vol_err < 0.001, "vol parity err {:.4}%", vol_err * 100.0);
            assert!(
                (rg.rms_error_mm - rc.rms_error_mm).abs() < 0.01,
                "RMS parity cpu={:.4} gpu={:.4}",
                rc.rms_error_mm,
                rg.rms_error_mm
            );
            assert!(
                (rg.mean_abs_error_mm - rc.mean_abs_error_mm).abs() < 0.01,
                "MAE parity cpu={:.4} gpu={:.4}",
                rc.mean_abs_error_mm,
                rg.mean_abs_error_mm
            );
            assert!(
                (rg.max_excess_mm - rc.max_excess_mm).abs() < 0.01,
                "max_excess parity cpu={:.4} gpu={:.4}",
                rc.max_excess_mm,
                rg.max_excess_mm
            );
            assert!(
                (rg.in_tolerance_surface_pct - rc.in_tolerance_surface_pct).abs() < 0.5,
                "in_tol parity cpu={:.2}% gpu={:.2}%",
                rc.in_tolerance_surface_pct,
                rg.in_tolerance_surface_pct
            );
        }
    }

    #[test]
    fn rotary_tridexel_indexed_below_equator_removes_all_cut_material_and_matches_cpu_gpu() {
        let stl = sample_rotary_shaft_stl_bytes();
        let doc = load_mesh_cad_document("rotary_shaft.stl", &stl, &MeshImportOptions::default())
            .unwrap();
        let lib = default_starter_tool_library();
        let flat_3mm = lib
            .tools
            .iter()
            .find(|t| {
                matches!(
                    t.geometry,
                    CutterGeometry::FlatEndmill { diameter_mm, .. } if diameter_mm <= 4.0
                )
            })
            .unwrap_or(&lib.tools[0]);

        let mut setup = Setup::new("s_rot_td", "Rotary Indexed Tri-Dexel", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(42.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup.rotary.axis = RotaryAxisKind::XAxis;
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        // Indexed 3+1 cuts on 0°, 90°, 180°, 270° including a multi-stepdown A=0° off-centerline
        // roughing slot extending below Z = 0 (Z = +6 -> 0 -> -5 -> -10.5 at Y = -17.0 mm, leaving
        // outer rim intact while cutting cleanly through the bottom of the cylinder).
        let mut b = ToolpathBuilder::new("op_idx_rough");
        b.rotary_rapid_to(DVec3::new(-15.0, -17.0, 28.0), 0.0);
        for &z_lvl in &[6.0_f64, 0.0, -5.0, -10.5] {
            b.rotary_linear_to(
                DVec3::new(-15.0, -17.0, z_lvl),
                0.0,
                300.0,
                21.0,
                MotionRole::Plunge,
            );
            b.rotary_linear_to(
                DVec3::new(15.0, -17.0, z_lvl),
                0.0,
                600.0,
                21.0,
                MotionRole::Cut,
            );
            b.rotary_rapid_to(DVec3::new(15.0, -17.0, 28.0), 0.0);
            b.rotary_rapid_to(DVec3::new(-15.0, -17.0, 28.0), 0.0);
        }
        for &ang in &[90.0_f64, 180.0, 270.0] {
            b.rotary_rapid_to(DVec3::new(-12.0, 0.0, 28.0), ang);
            b.rotary_linear_to(
                DVec3::new(-12.0, 0.0, 16.5),
                ang,
                300.0,
                16.5,
                MotionRole::Plunge,
            );
            b.rotary_linear_to(
                DVec3::new(12.0, 0.0, 16.5),
                ang,
                600.0,
                16.5,
                MotionRole::Cut,
            );
            b.rotary_rapid_to(DVec3::new(12.0, 0.0, 28.0), ang);
        }
        let tp = b.finish();

        let input = SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_idx_rough",
                op_name: "Indexed Adaptive Rough",
                tool: flat_3mm,
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
        };

        let cpu = run_setup_simulation(&SetupSimulationInput {
            engine: SimEngineKind::TriDexel,
            backend_choice: SimComputeBackendChoice::Cpu,
            ..input
        })
        .unwrap();
        assert_eq!(cpu.engine, SimEngineKind::TriDexel);
        assert!(cpu.is_rotary());
        let td_cpu = cpu
            .as_tridexel()
            .expect("SimEngineKind::TriDexel on rotary setup must produce TriDexelStock");
        assert!(
            td_cpu
                .sample_top_z(0.0, -17.0)
                .is_some_and(|z| (z - (-10.5)).abs() < 0.05),
            "deep A=0° cut below equator must remove material all the way to Z=-10.5 at (0, -17), got {:?}",
            td_cpu.sample_top_z(0.0, -17.0)
        );
        assert!(
            td_cpu.sample_top_z(0.0, -18.3).is_none(),
            "deep A=0° cut to Z=-10.5 must break cleanly through cylinder bottom at (0, -18.3), got {:?}",
            td_cpu.sample_top_z(0.0, -18.3)
        );
        assert!(
            td_cpu.sample_top_z(0.0, -20.0).is_some_and(|z| z > 5.0),
            "outer cylindrical rim outside cutter width must remain intact"
        );
        assert!(
            (2_800.0..=3_600.0).contains(&cpu.removed_volume_mm3),
            "expected ~3,196 mm^3 removed volume across 4 indexed sides, got {:.1}",
            cpu.removed_volume_mm3
        );
        let rc = cpu
            .error_report
            .as_ref()
            .expect("rotary Tri-Dexel must populate StockErrorReport");
        assert!(rc.rms_error_mm.is_finite() && rc.target_volume_mm3 > 10_000.0);

        // Verify incremental playback cursor sync matches final stock cleanly
        let mid_stock = cpu.reconstruct_stock_at_time(cpu.timeline.total_time_sec * 0.5);
        assert!(mid_stock.is_rotary());
        assert!(
            mid_stock.compute_volume_mm3() < cpu.initial_stock_volume_mm3
                && mid_stock.compute_volume_mm3() >= cpu.remaining_stock_volume_mm3 - 1.0
        );

        #[cfg(feature = "gpu-compute")]
        if GpuSimContext::shared_headless().is_some() {
            let gpu = run_setup_simulation(&SetupSimulationInput {
                engine: SimEngineKind::TriDexel,
                backend_choice: SimComputeBackendChoice::Gpu,
                ..input
            })
            .unwrap();
            assert!(gpu.active_backend.is_gpu);
            assert_eq!(gpu.engine, SimEngineKind::TriDexel);
            assert!(gpu.is_rotary());
            let td_gpu = gpu.as_tridexel().expect("expected GPU TriDexelStock");
            assert!(
                td_gpu
                    .sample_top_z(0.0, -17.0)
                    .is_some_and(|z| (z - (-10.5)).abs() < 0.05)
            );
            assert!(td_gpu.sample_top_z(0.0, -18.3).is_none());
            let vol_err = (gpu.remaining_stock_volume_mm3 - cpu.remaining_stock_volume_mm3).abs()
                / cpu.remaining_stock_volume_mm3.max(1.0);
            assert!(
                vol_err < 0.002,
                "rotary Tri-Dexel CPU vs GPU volume parity err {:.4}%",
                vol_err * 100.0
            );
            let rg = gpu.error_report.as_ref().unwrap();
            assert!(
                (rg.rms_error_mm - rc.rms_error_mm).abs() < 0.05,
                "rotary Tri-Dexel RMS parity cpu={:.4} gpu={:.4}",
                rc.rms_error_mm,
                rg.rms_error_mm
            );
        }
    }

    #[test]
    fn simulation_reports_monotonic_percent_complete_and_honors_cancellation() {
        use std::sync::{Arc, Mutex, atomic::AtomicBool};

        use crate::progress::{SimProgressStage, SimProgressTracker};

        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_prog", "Progress Setup", WorkOffsetSlot::G54);
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

        let mut b = ToolpathBuilder::new("op_prog");
        b.rapid_to(DVec3::new(-30.0, -25.0, 8.0));
        b.linear_to(DVec3::new(-30.0, -25.0, -3.0), 300.0, MotionRole::Plunge);
        for row in 0..20 {
            let y = -25.0 + (row as f64) * 2.5;
            let x_end = if row % 2 == 0 { 30.0 } else { -30.0 };
            b.linear_to(DVec3::new(x_end, y, -3.0), 800.0, MotionRole::Cut);
            if row < 19 {
                b.linear_to(DVec3::new(x_end, y + 2.5, -3.0), 800.0, MotionRole::Cut);
            }
        }
        b.rapid_to(DVec3::new(30.0, 25.0, 8.0));
        let tp = b.finish();

        let observed = Arc::new(Mutex::new(Vec::<(u8, SimProgressStage)>::new()));
        let observed_cb = Arc::clone(&observed);
        let tracker_slot: Arc<Mutex<Option<Arc<SimProgressTracker>>>> = Arc::new(Mutex::new(None));
        let tracker_slot_cb = Arc::clone(&tracker_slot);
        let tracker = Arc::new(SimProgressTracker::with_callback(Arc::new(move || {
            if let Some(t) = tracker_slot_cb.lock().unwrap().as_ref() {
                let snap = t.snapshot();
                observed_cb
                    .lock()
                    .unwrap()
                    .push((snap.percent_rounded(), snap.stage));
            }
        })));
        *tracker_slot.lock().unwrap() = Some(Arc::clone(&tracker));

        let input = SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_prog",
                op_name: "Progress Raster",
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
        };

        let _res = run_setup_simulation(&SetupSimulationInput {
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Cpu,
            progress: Some(tracker.full_scope()),
            ..input
        })
        .unwrap();

        let final_snap = tracker.snapshot();
        assert_eq!(final_snap.percent_rounded(), 100);
        assert_eq!(final_snap.stage, SimProgressStage::Complete);

        let log = observed.lock().unwrap();
        assert!(
            log.len() >= 5,
            "expected multiple intermediate progress updates, got {log:?}"
        );
        assert!(
            log.iter()
                .any(|(pct, st)| *pct > 10 && *pct < 95 && *st == SimProgressStage::Sweeping),
            "expected mid-sweep percentage updates in (10, 95), got {log:?}"
        );
        for w in log.windows(2) {
            assert!(
                w[1].0 >= w[0].0,
                "progress percentage must be monotonic, saw regression {} -> {}",
                w[0].0,
                w[1].0
            );
        }

        // Pre-cancelled run must return Cancelled without reporting 100% completion.
        let cancel = AtomicBool::new(true);
        let cancel_tracker = SimProgressTracker::new();
        let cancel_input = SetupSimulationInput {
            cancel: Some(&cancel),
            ..input
        };
        let err = run_setup_simulation(&SetupSimulationInput {
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Cpu,
            progress: Some(cancel_tracker.full_scope()),
            ..cancel_input
        })
        .unwrap_err();
        assert!(matches!(err, SimError::Cancelled));
        assert!(cancel_tracker.snapshot().percent_rounded() < 100);
    }

    #[cfg(feature = "gpu-compute")]
    #[test]
    fn batched_gpu_sweep_matches_cpu_volume_and_exact_violation_segment_on_long_path() {
        let Some(_) = GpuSimContext::shared_headless() else {
            return;
        };

        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];
        let mut setup = Setup::new("s_batch_gpu", "Batched GPU Setup", WorkOffsetSlot::G54);
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

        // Build >300 segments spanning multiple GPU sweep batches, with a deliberate G0 rapid crash
        // into uncut stock at a known mid-batch segment index.
        let mut b = ToolpathBuilder::new("op_batch");
        b.rapid_to(DVec3::new(-40.0, -30.0, 10.0));
        b.linear_to(DVec3::new(-40.0, -30.0, -2.0), 400.0, MotionRole::Plunge);
        for row in 0..160 {
            let y = -30.0 + (row as f64) * 0.35;
            let x_a = if row % 2 == 0 { -40.0 } else { 40.0 };
            let x_b = -x_a;
            b.linear_to(DVec3::new(x_a, y, -2.0), 1200.0, MotionRole::Cut);
            b.linear_to(DVec3::new(x_b, y, -2.0), 1200.0, MotionRole::Cut);
            if row == 110 {
                // Deliberate G0 rapid plunge/drag through uncut stock on the far side
                b.rapid_to(DVec3::new(x_b, 35.0, -4.0));
                b.rapid_to(DVec3::new(x_b, y, 5.0));
                b.linear_to(DVec3::new(x_b, y, -2.0), 400.0, MotionRole::Plunge);
            }
        }
        b.rapid_to(DVec3::new(0.0, 0.0, 10.0));
        let tp = b.finish();
        assert!(tp.segments_wcs.len() > 300);

        let input = SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_batch",
                op_name: "Batched Roughing",
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
        };

        for engine in [SimEngineKind::Heightmap, SimEngineKind::TriDexel] {
            let cpu = run_setup_simulation(&SetupSimulationInput {
                engine,
                backend_choice: SimComputeBackendChoice::Cpu,
                ..input
            })
            .unwrap();
            let gpu = run_setup_simulation(&SetupSimulationInput {
                engine,
                backend_choice: SimComputeBackendChoice::Gpu,
                ..input
            })
            .unwrap();
            assert!(gpu.active_backend.is_gpu);

            let vol_err = (gpu.remaining_stock_volume_mm3 - cpu.remaining_stock_volume_mm3).abs()
                / cpu.remaining_stock_volume_mm3.max(1.0);
            assert!(
                vol_err < 0.002,
                "{engine:?} batched GPU vs CPU volume err {:.4}%",
                vol_err * 100.0
            );

            let cpu_rapid = cpu
                .diagnostics
                .iter()
                .find(|d| d.kind == SimIssueKind::RapidIntoStock)
                .expect("CPU must catch deliberate G0 rapid crash");
            let gpu_rapid = gpu
                .diagnostics
                .iter()
                .find(|d| d.kind == SimIssueKind::RapidIntoStock)
                .expect("batched GPU must catch deliberate G0 rapid crash");
            assert_eq!(
                gpu_rapid.global_seg_idx, cpu_rapid.global_seg_idx,
                "{engine:?} batched GPU must attribute rapid crash to exact offending segment"
            );
        }
    }

    #[test]
    fn setup_simulation_mesh_skinning_toggle_leaves_metrology_bit_identical() {
        let target_mesh = make_test_box_mesh_body(
            BodyId(0),
            "Target Block",
            DVec3::new(-20.0, -20.0, -14.0),
            DVec3::new(20.0, 20.0, -2.0),
        );
        let render_mesh = chip_cad::build_render_mesh_for_triangle_mesh(
            &target_mesh,
            35.0,
            chip_cad::MESH_FACE_SENTINEL,
        );
        let mut doc = CadDocument::empty();
        doc.source_name = "skinning_target.3mf".to_string();
        doc.meshes = vec![target_mesh];
        doc.aabb_mm = chip_math::Aabb3::new(
            DVec3::new(-20.0, -20.0, -14.0),
            DVec3::new(20.0, 20.0, -2.0),
        );
        doc.render_mesh = render_mesh;
        let box_body = BodyId(0);

        let mut setup = Setup::new(
            "s_skin_metrology",
            "Skinning Metrology Setup",
            WorkOffsetSlot::G54,
        );
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(50.0),
            depth_y: ParamLength::from_mm(50.0),
            height_z: ParamLength::from_mm(16.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::CenterZ,
        };
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let ball_8mm = chip_tools::Tool {
            id: chip_tools::ToolId("ball_8".to_string()),
            tool_number: 1,
            length_offset_number: 1,
            diameter_offset_number: 1,
            name: "8mm Ball".to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: chip_tools::SpindleDirection::Cw,
            default_coolant: chip_tools::CoolantMode::Mist,
            stickout_length_mm: 28.0,
            geometry: CutterGeometry::BallEndmill {
                diameter_mm: 8.0,
                flute_length_mm: 14.0,
                overall_length_mm: 50.0,
                shank_diameter_mm: 8.0,
                flutes: 2,
            },
            presets: Vec::new(),
        };

        let mut b = ToolpathBuilder::new("op_ball");
        b.rapid_to(DVec3::new(-18.0, -10.0, 5.0));
        b.linear_to(DVec3::new(-18.0, -10.0, -2.5), 400.0, MotionRole::Plunge);
        b.linear_to(DVec3::new(18.0, 10.0, -4.5), 1200.0, MotionRole::Cut);
        b.rapid_to(DVec3::new(18.0, 10.0, 5.0));
        let tp = b.finish();

        let input = SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[box_body],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_ball",
                op_name: "Ball Groove",
                tool: &ball_8mm,
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
        };

        for engine in [SimEngineKind::Heightmap, SimEngineKind::TriDexel] {
            let res = run_setup_simulation(&SetupSimulationInput { engine, ..input }).unwrap();
            let rep_before = res.compute_error_report_at_tolerance(0.05).unwrap();
            let mesh_crisp = res.extract_stock_mesh_with_options(
                &res.final_stock,
                SimMeshOptions::new(SimStockColorMode::ByOperation, false),
            );
            let mesh_skinned = res.extract_stock_mesh_with_options(
                &res.final_stock,
                SimMeshOptions::new(SimStockColorMode::ByOperation, true),
            );
            let rep_after = res.compute_error_report_at_tolerance(0.05).unwrap();

            assert!(!mesh_crisp.vertices.is_empty());
            assert!(!mesh_skinned.vertices.is_empty());
            assert_eq!(rep_before, rep_after);
            assert_eq!(
                res.error_report
                    .as_ref()
                    .unwrap()
                    .excess_volume_mm3
                    .to_bits(),
                rep_after.excess_volume_mm3.to_bits()
            );
            assert_eq!(
                res.error_report
                    .as_ref()
                    .unwrap()
                    .gouge_volume_mm3
                    .to_bits(),
                rep_after.gouge_volume_mm3.to_bits()
            );
        }
    }
}
