//! Simulation output container ([`SetupSimulationResult`]), timing breakdown, and unified
//! CPU/GPU playback cursor synchronization across `Heightfield`, `TriDexel`, and `Cylindrical`.

use chip_stock::{
    ActiveSimBackendInfo, CylindricalStock, HeightfieldStock, RotaryStockSpec, SimEngineKind,
    SimMeshOptions, SimResolution, SimStockColorMode, SimStockMesh, SimulatedStock,
    StockCheckpointSnapshot, StockErrorReport, StockModelKind, StockPlaybackCursor,
    StockVerifyTolerances, SweepChord, SweepToolSpec, TriDexelStock,
};
#[cfg(feature = "gpu-compute")]
use chip_stock::{GpuCheckpointCache, SimStockBackend as _, WgpuComputeBackend};

use crate::{
    timeline::{ScrubSnapshot, SimulationTimeline},
    verifier::{SimDiagnostic, SimSeverity},
    verify_common::{push_segment_sweep_chords, slice_segment_range},
};

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
        let target = self.target_cursor_for_time(time_sec);
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
        let target = self.target_cursor_for_time(time_sec);
        self.sync_gpu_backend_to_target_cursor(backend, gpu_checkpoints, cursor, target)
    }

    fn target_cursor_for_time(&self, time_sec: f64) -> StockPlaybackCursor {
        let total_spans = self.timeline.motion_spans.len();
        self.timeline.sample_at_time(time_sec).map_or(
            StockPlaybackCursor {
                seg_idx: total_spans,
                seg_u: 0.0,
            },
            |s| target_cursor_from_snap(&s, total_spans),
        )
    }

    fn emit_span_chords(
        &self,
        seg_idx: usize,
        u_start: f64,
        u_end: f64,
        out: &mut Vec<SweepChord>,
    ) {
        let Some(span) = self.timeline.motion_spans.get(seg_idx) else {
            return;
        };
        let default_tol = StockVerifyTolerances::default();
        if u_start <= 1e-6 && u_end >= 1.0 - 1e-6 {
            push_segment_sweep_chords(
                &span.segment,
                span.op_idx as u32,
                span.op_idx,
                default_tol,
                out,
            );
        } else if let Some(sub_seg) = slice_segment_range(&span.segment, u_start, u_end) {
            push_segment_sweep_chords(&sub_seg, span.op_idx as u32, span.op_idx, default_tol, out);
        }
    }

    /// Unified CPU/GPU playback cursor state machine: short-circuits unchanged cursors,
    /// jumps directly to `final_stock` at end of timeline, selects the nearest checkpoint
    /// when scrubbing backward or jumping ahead, and sweeps only the minimal tail delta.
    fn sync_playback_target<T: PlaybackSyncTarget>(
        &self,
        sink: &mut T,
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
            sink.jump_to_end(total_spans, &self.final_stock);
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
                sink.restore_checkpoint(cp);
                (cp.completed_segments(), 0.0, true)
            } else {
                (cur.seg_idx, cur.seg_u, false)
            }
        } else if let Some(cp) = cp {
            sink.restore_checkpoint(cp);
            (cp.completed_segments(), 0.0, true)
        } else {
            sink.fallback_final(&self.final_stock);
            (0, 0.0, true)
        };

        let mut delta_chords: Vec<SweepChord> = Vec::new();
        if cur_seg == target.seg_idx {
            if target.seg_u > cur_u + 1e-6 {
                self.emit_span_chords(cur_seg, cur_u, target.seg_u, &mut delta_chords);
            }
        } else {
            if cur_u > 1e-6 {
                self.emit_span_chords(cur_seg, cur_u, 1.0, &mut delta_chords);
                cur_seg += 1;
            }
            for seg_idx in cur_seg..target.seg_idx {
                self.emit_span_chords(seg_idx, 0.0, 1.0, &mut delta_chords);
            }
            if target.seg_u > 1e-6 {
                self.emit_span_chords(target.seg_idx, 0.0, target.seg_u, &mut delta_chords);
            }
        }

        if !delta_chords.is_empty() {
            changed |= sink.sweep_delta(&delta_chords, &self.operation_profiles);
        }

        *cursor = Some(target);
        changed
    }

    fn sync_stock_to_target_cursor(
        &self,
        stock: &mut SimulatedStock,
        cursor: &mut Option<StockPlaybackCursor>,
        target: StockPlaybackCursor,
    ) -> bool {
        self.sync_playback_target(stock, cursor, target)
    }

    #[cfg(feature = "gpu-compute")]
    fn sync_gpu_backend_to_target_cursor(
        &self,
        backend: &mut WgpuComputeBackend,
        gpu_checkpoints: &mut GpuCheckpointCache,
        cursor: &mut Option<StockPlaybackCursor>,
        target: StockPlaybackCursor,
    ) -> bool {
        let mut sink = GpuPlaybackSink {
            backend,
            gpu_checkpoints,
        };
        self.sync_playback_target(&mut sink, cursor, target)
    }
}

trait PlaybackSyncTarget {
    fn jump_to_end(&mut self, total_spans: usize, final_stock: &SimulatedStock);
    fn restore_checkpoint(&mut self, cp: &StockCheckpointSnapshot);
    fn fallback_final(&mut self, final_stock: &SimulatedStock);
    fn sweep_delta(&mut self, chords: &[SweepChord], profiles: &[SweepToolSpec]) -> bool;
}

impl PlaybackSyncTarget for SimulatedStock {
    fn jump_to_end(&mut self, _total_spans: usize, final_stock: &SimulatedStock) {
        self.copy_state_from(final_stock);
    }

    fn restore_checkpoint(&mut self, cp: &StockCheckpointSnapshot) {
        self.reset_from_checkpoint(cp);
    }

    fn fallback_final(&mut self, final_stock: &SimulatedStock) {
        self.copy_state_from(final_stock);
    }

    fn sweep_delta(&mut self, chords: &[SweepChord], profiles: &[SweepToolSpec]) -> bool {
        self.sweep_chords(chords, profiles, None, false).mutated
    }
}

#[cfg(feature = "gpu-compute")]
struct GpuPlaybackSink<'a> {
    backend: &'a mut WgpuComputeBackend,
    gpu_checkpoints: &'a mut GpuCheckpointCache,
}

#[cfg(feature = "gpu-compute")]
impl PlaybackSyncTarget for GpuPlaybackSink<'_> {
    fn jump_to_end(&mut self, total_spans: usize, final_stock: &SimulatedStock) {
        if !self
            .gpu_checkpoints
            .restore_into_backend(self.backend, total_spans, None)
        {
            self.backend.copy_state_from_cpu_stock(final_stock);
            let _ = self
                .gpu_checkpoints
                .capture_from_backend(self.backend, total_spans);
        }
    }

    fn restore_checkpoint(&mut self, cp: &StockCheckpointSnapshot) {
        let _ = self.gpu_checkpoints.restore_into_backend(
            self.backend,
            cp.completed_segments(),
            Some(cp),
        );
    }

    fn fallback_final(&mut self, final_stock: &SimulatedStock) {
        self.backend.copy_state_from_cpu_stock(final_stock);
    }

    fn sweep_delta(&mut self, chords: &[SweepChord], profiles: &[SweepToolSpec]) -> bool {
        self.backend.sweep_batch(chords, profiles, false).mutated
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
