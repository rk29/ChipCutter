//! Unified polymorphic volumetric stock representation (`SimulatedStock`), checkpoint snapshot
//! (`StockCheckpointSnapshot`), playback cursor state machine, and `SimStockBackend` trait
//! (`CpuExactBackend` default; future `WgpuComputeBackend` plugs in behind a feature flag).

use serde::{Deserialize, Serialize};

use crate::{
    cylindrical::{CylindricalStock, tessellate_cylindrical_stock_mesh_colored},
    heightfield::{HeightfieldStock, tessellate_stock_mesh_colored},
    metrics::{
        OperationErrorSnapshot, SimStockColorMode, StockErrorReport,
        compute_cylindrical_error_report, compute_cylindrical_quick_error_summary,
        compute_heightmap_error_report, compute_heightmap_quick_error_summary,
        compute_tridexel_error_report, compute_tridexel_quick_error_summary,
    },
    tridexel::{TriDexelStock, tessellate_tridexel_stock_mesh_colored},
    types::{
        ActiveSimBackendInfo, BatchSweepOutput, RotaryStockSpec, SimStockMesh, StockPlaybackCursor,
        SweepChord, SweepToolSpec,
    },
};

/// Concrete volumetric stock representation selected for a simulation run.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum StockModelKind {
    Heightfield25D,
    Cylindrical4Axis(RotaryStockSpec),
    TriDexel3D,
}

impl StockModelKind {
    /// Recommended periodic checkpoint stride (`motion_spans` between snapshots) balancing
    /// RAM footprint against `< 4 ms` interactive timeline scrubbing.
    pub const fn checkpoint_stride(self, total_spans: usize) -> usize {
        match self {
            Self::Heightfield25D => {
                let s = total_spans / 20;
                if s < 128 { 128 } else { s }
            }
            Self::TriDexel3D => {
                let s = total_spans / 10;
                if s < 120 { 120 } else { s }
            }
            Self::Cylindrical4Axis(_) => {
                let s = total_spans / 24;
                if s < 16 { 16 } else { s }
            }
        }
    }
}

/// Unified compact stock checkpoint snapshot captured during simulation for `< 4 ms` scrubbing.
#[derive(Debug, Clone)]
pub enum StockCheckpointSnapshot {
    Heightfield {
        completed_segments: usize,
        z_stock: Vec<f32>,
        last_op_idx: Vec<u32>,
    },
    TriDexel {
        completed_segments: usize,
        stock: Box<TriDexelStock>,
    },
    Cylindrical {
        completed_segments: usize,
        r_stock: Vec<f32>,
        last_op_idx: Vec<u32>,
    },
}

impl StockCheckpointSnapshot {
    #[inline]
    pub const fn completed_segments(&self) -> usize {
        match self {
            Self::Heightfield {
                completed_segments, ..
            }
            | Self::TriDexel {
                completed_segments, ..
            }
            | Self::Cylindrical {
                completed_segments, ..
            } => *completed_segments,
        }
    }
}

/// Unified polymorphic simulated stock solid across all three volumetric engines.
#[derive(Debug, Clone)]
pub enum SimulatedStock {
    Heightfield(HeightfieldStock),
    TriDexel(Box<TriDexelStock>),
    Cylindrical(CylindricalStock),
}

impl SimulatedStock {
    #[inline]
    pub fn kind(&self) -> StockModelKind {
        match self {
            Self::Heightfield(_) => StockModelKind::Heightfield25D,
            Self::TriDexel(_) => StockModelKind::TriDexel3D,
            Self::Cylindrical(c) => StockModelKind::Cylindrical4Axis(c.rotary),
        }
    }

    #[inline]
    pub fn as_heightfield(&self) -> Option<&HeightfieldStock> {
        match self {
            Self::Heightfield(hf) => Some(hf),
            _ => None,
        }
    }

    #[inline]
    pub fn as_heightfield_mut(&mut self) -> Option<&mut HeightfieldStock> {
        match self {
            Self::Heightfield(hf) => Some(hf),
            _ => None,
        }
    }

    #[inline]
    pub fn as_tridexel(&self) -> Option<&TriDexelStock> {
        match self {
            Self::TriDexel(td) => Some(td),
            _ => None,
        }
    }

    #[inline]
    pub fn as_tridexel_mut(&mut self) -> Option<&mut TriDexelStock> {
        match self {
            Self::TriDexel(td) => Some(td),
            _ => None,
        }
    }

    #[inline]
    pub fn as_cylindrical(&self) -> Option<&CylindricalStock> {
        match self {
            Self::Cylindrical(cyl) => Some(cyl),
            _ => None,
        }
    }

    #[inline]
    pub fn as_cylindrical_mut(&mut self) -> Option<&mut CylindricalStock> {
        match self {
            Self::Cylindrical(cyl) => Some(cyl),
            _ => None,
        }
    }

    #[inline]
    pub const fn is_cylindrical(&self) -> bool {
        matches!(self, Self::Cylindrical(_))
    }

    /// Computes total remaining solid stock volume (`mm³`).
    pub fn compute_volume_mm3(&self) -> f64 {
        match self {
            Self::Heightfield(hf) => hf.compute_volume_mm3(),
            Self::TriDexel(td) => td.compute_volume_mm3(),
            Self::Cylindrical(cyl) => cyl.current_volume_mm3(),
        }
    }

    /// Samples the top remaining stock Z height (`mm` in WCS) at `(x_wcs, y_wcs)` for 3-axis
    /// Heightfield or Tri-Dexel stocks (`None` if empty or Cylindrical).
    pub fn sample_stock_z(&self, x_wcs: f64, y_wcs: f64) -> Option<f64> {
        match self {
            Self::Heightfield(hf) => hf.sample_stock_z(x_wcs, y_wcs),
            Self::TriDexel(td) => td.sample_top_z(x_wcs, y_wcs),
            Self::Cylindrical(_) => None,
        }
    }

    /// Evaluates Rule 5 solid axial penetration depth (`mm`) for a vertical downward plunge
    /// from `z_start_wcs` to `z_end_wcs` at `(x_wcs, y_wcs)`.
    pub fn query_vertical_plunge_depth(
        &self,
        x_wcs: f64,
        y_wcs: f64,
        z_start_wcs: f64,
        z_end_wcs: f64,
        thread_check_radius_mm: Option<f64>,
    ) -> f64 {
        match self {
            Self::Heightfield(hf) => {
                let stock_z = if let Some(check_r) = thread_check_radius_mm {
                    hf.sample_max_stock_z_in_disk(x_wcs, y_wcs, check_r)
                } else {
                    hf.sample_stock_z(x_wcs, y_wcs)
                };
                stock_z.map_or(0.0, |sz| sz.min(z_start_wcs) - z_end_wcs)
            }
            Self::TriDexel(td) => {
                if let Some(check_r) = thread_check_radius_mm {
                    td.query_vertical_plunge_overlap(x_wcs, y_wcs, check_r, z_end_wcs, z_start_wcs)
                } else {
                    td.sample_top_z(x_wcs, y_wcs)
                        .map_or(0.0, |top_z| top_z.min(z_start_wcs) - z_end_wcs)
                }
            }
            Self::Cylindrical(_) => 0.0,
        }
    }

    /// Captures a compact checkpoint snapshot at `completed_segments`.
    pub fn capture_checkpoint(&self, completed_segments: usize) -> StockCheckpointSnapshot {
        match self {
            Self::Heightfield(hf) => StockCheckpointSnapshot::Heightfield {
                completed_segments,
                z_stock: hf.z_stock.clone(),
                last_op_idx: hf.last_op_idx.clone(),
            },
            Self::TriDexel(td) => StockCheckpointSnapshot::TriDexel {
                completed_segments,
                stock: td.clone(),
            },
            Self::Cylindrical(cyl) => StockCheckpointSnapshot::Cylindrical {
                completed_segments,
                r_stock: cyl.r_stock.clone(),
                last_op_idx: cyl.last_op_idx.clone(),
            },
        }
    }

    /// Restores mutable stock buffers from `cp` (reusing existing `Vec` capacity where possible).
    pub fn reset_from_checkpoint(&mut self, cp: &StockCheckpointSnapshot) {
        match (self, cp) {
            (
                Self::Heightfield(hf),
                StockCheckpointSnapshot::Heightfield {
                    z_stock,
                    last_op_idx,
                    ..
                },
            ) => {
                hf.z_stock.clone_from(z_stock);
                hf.last_op_idx.clone_from(last_op_idx);
            }
            (Self::TriDexel(td), StockCheckpointSnapshot::TriDexel { stock, .. }) => {
                td.clone_from(stock);
            }
            (
                Self::Cylindrical(cyl),
                StockCheckpointSnapshot::Cylindrical {
                    r_stock,
                    last_op_idx,
                    ..
                },
            ) => {
                cyl.r_stock.clone_from(r_stock);
                cyl.last_op_idx.clone_from(last_op_idx);
            }
            _ => {}
        }
    }

    /// Copies mutable stock state from `other` (e.g. jumping directly to final stock at end of timeline).
    pub fn copy_state_from(&mut self, other: &Self) {
        match (self, other) {
            (Self::Heightfield(dst), Self::Heightfield(src)) => {
                dst.z_stock.clone_from(&src.z_stock);
                dst.last_op_idx.clone_from(&src.last_op_idx);
            }
            (Self::TriDexel(dst), Self::TriDexel(src)) => {
                dst.clone_from(src);
            }
            (Self::Cylindrical(dst), Self::Cylindrical(src)) => {
                dst.r_stock.clone_from(&src.r_stock);
                dst.last_op_idx.clone_from(&src.last_op_idx);
            }
            _ => {}
        }
    }

    /// Sweeps a batch of [`SweepChord`] primitives across the active stock solid.
    pub fn sweep_chords(
        &mut self,
        chords: &[SweepChord],
        tools: &[SweepToolSpec],
        target_tridexel: Option<&TriDexelStock>,
        verify: bool,
    ) -> BatchSweepOutput {
        match self {
            Self::Heightfield(hf) => hf.sweep_chords(chords, tools, verify),
            Self::TriDexel(td) => td.sweep_chords(chords, tools, target_tridexel, verify),
            Self::Cylindrical(cyl) => cyl.sweep_chords(chords, tools, verify),
        }
    }

    /// Extracts a 3D GPU-ready [`SimStockMesh`] (always in stock-local 0° frame for Cylindrical so
    /// the GPU rotates the stock cleanly via the `stock_model` uniform).
    pub fn extract_stock_mesh(
        &self,
        target_tridexel: Option<&TriDexelStock>,
        color_mode: SimStockColorMode,
    ) -> SimStockMesh {
        match self {
            Self::Heightfield(hf) => tessellate_stock_mesh_colored(hf, color_mode),
            Self::TriDexel(td) => {
                tessellate_tridexel_stock_mesh_colored(td, target_tridexel, color_mode)
            }
            Self::Cylindrical(cyl) => {
                tessellate_cylindrical_stock_mesh_colored(cyl, color_mode, 0.0)
            }
        }
    }

    /// Computes full quantitative stock-vs-target [`StockErrorReport`] at `tolerance_mm`.
    pub fn compute_error_report(
        &self,
        target_tridexel: Option<&TriDexelStock>,
        initial_stock_vol_mm3: f64,
        tolerance_mm: f64,
        per_operation: Vec<OperationErrorSnapshot>,
    ) -> Option<StockErrorReport> {
        match self {
            Self::Heightfield(hf) => compute_heightmap_error_report(
                hf,
                initial_stock_vol_mm3,
                tolerance_mm,
                per_operation,
            ),
            Self::TriDexel(td) => target_tridexel.map(|tgt| {
                compute_tridexel_error_report(
                    td,
                    tgt,
                    initial_stock_vol_mm3,
                    tolerance_mm,
                    per_operation,
                )
            }),
            Self::Cylindrical(cyl) => compute_cylindrical_error_report(
                cyl,
                initial_stock_vol_mm3,
                tolerance_mm,
                per_operation,
            ),
        }
    }

    /// Fast per-operation summary `(excess_vol, gouge_vol, rms_mm)` captured at operation boundaries.
    pub fn compute_quick_error_summary(
        &self,
        target_tridexel: Option<&TriDexelStock>,
    ) -> Option<(f64, f64, f64)> {
        match self {
            Self::Heightfield(hf) => compute_heightmap_quick_error_summary(hf),
            Self::TriDexel(td) => {
                target_tridexel.map(|tgt| compute_tridexel_quick_error_summary(td, tgt))
            }
            Self::Cylindrical(cyl) => compute_cylindrical_quick_error_summary(cyl),
        }
    }

    /// Unified incremental playback & checkpoint-restore cursor state machine across
    /// `Heightfield`, `TriDexel`, and `Cylindrical` stocks.
    ///
    /// `emit_chords(seg_idx, u_start, u_end, out)` appends the sweep chords covering
    /// `[u_start, u_end]` of motion span `seg_idx`.
    ///
    /// Returns `true` if `self` mutated (indicating any cached display mesh should be re-tessellated).
    #[allow(clippy::too_many_arguments)]
    pub fn sync_to_cursor<F>(
        &mut self,
        cursor: &mut Option<StockPlaybackCursor>,
        target: StockPlaybackCursor,
        total_spans: usize,
        final_stock: &Self,
        checkpoints: &[StockCheckpointSnapshot],
        tools: &[SweepToolSpec],
        mut emit_chords: F,
    ) -> bool
    where
        F: FnMut(usize, f64, f64, &mut Vec<SweepChord>),
    {
        if let Some(cur) = *cursor
            && cur.seg_idx == target.seg_idx
            && (target.seg_u - cur.seg_u).abs() <= 1e-6
        {
            return false;
        }

        if target.seg_idx >= total_spans {
            self.copy_state_from(final_stock);
            *cursor = Some(StockPlaybackCursor {
                seg_idx: total_spans,
                seg_u: 0.0,
            });
            return true;
        }

        let cp = checkpoints
            .iter()
            .rfind(|c| c.completed_segments() <= target.seg_idx);

        let (mut cur_seg, cur_u, mut changed) = if let Some(cur) = *cursor
            && (target.seg_idx > cur.seg_idx
                || (target.seg_idx == cur.seg_idx && target.seg_u >= cur.seg_u))
        {
            if let Some(cp) = cp
                && cp.completed_segments() > cur.seg_idx
            {
                self.reset_from_checkpoint(cp);
                (cp.completed_segments(), 0.0, true)
            } else {
                (cur.seg_idx, cur.seg_u, false)
            }
        } else if let Some(cp) = cp {
            self.reset_from_checkpoint(cp);
            (cp.completed_segments(), 0.0, true)
        } else {
            self.copy_state_from(final_stock);
            (0, 0.0, true)
        };

        let mut delta_chords: Vec<SweepChord> = Vec::new();
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
            let out = self.sweep_chords(&delta_chords, tools, None, false);
            changed |= out.mutated;
        }

        *cursor = Some(target);
        changed
    }
}

/// Polymorphic volumetric stock simulation backend trait (`CpuExactBackend` default;
/// future `WgpuComputeBackend` plugs in behind `#[cfg(feature = "gpu-compute")]`).
pub trait SimStockBackend {
    fn backend_info(&self) -> ActiveSimBackendInfo {
        ActiveSimBackendInfo::cpu()
    }

    fn kind(&self) -> StockModelKind;

    fn capture_checkpoint(&self, completed_segments: usize) -> StockCheckpointSnapshot;

    fn reset_from_checkpoint(&mut self, cp: &StockCheckpointSnapshot);

    fn sweep_batch(
        &mut self,
        chords: &[SweepChord],
        profiles: &[SweepToolSpec],
        verify: bool,
    ) -> BatchSweepOutput;

    fn query_vertical_plunge_depth(
        &self,
        x_wcs: f64,
        y_wcs: f64,
        z_start_wcs: f64,
        z_end_wcs: f64,
        thread_check_radius_mm: Option<f64>,
    ) -> f64;

    fn compute_volume_mm3(&self) -> f64;

    fn compute_quick_error_summary(&self) -> Option<(f64, f64, f64)>;

    fn compute_error_report(
        &self,
        initial_stock_vol_mm3: f64,
        tolerance_mm: f64,
        per_operation: Vec<OperationErrorSnapshot>,
    ) -> Option<StockErrorReport>;

    fn extract_stock_mesh(&self, color_mode: SimStockColorMode) -> SimStockMesh;

    fn into_parts(self) -> (SimulatedStock, Option<TriDexelStock>);
}

/// Exact CPU + Rayon volumetric stock simulation backend covering `HeightfieldStock`,
/// `TriDexelStock`, and 4th-axis `CylindricalStock`.
#[derive(Debug, Clone)]
pub struct CpuExactBackend {
    pub stock: SimulatedStock,
    pub target_tridexel: Option<TriDexelStock>,
}

impl CpuExactBackend {
    pub fn new(stock: SimulatedStock, target_tridexel: Option<TriDexelStock>) -> Self {
        Self {
            stock,
            target_tridexel,
        }
    }
}

impl SimStockBackend for CpuExactBackend {
    fn kind(&self) -> StockModelKind {
        self.stock.kind()
    }

    fn capture_checkpoint(&self, completed_segments: usize) -> StockCheckpointSnapshot {
        self.stock.capture_checkpoint(completed_segments)
    }

    fn reset_from_checkpoint(&mut self, cp: &StockCheckpointSnapshot) {
        self.stock.reset_from_checkpoint(cp);
    }

    fn sweep_batch(
        &mut self,
        chords: &[SweepChord],
        profiles: &[SweepToolSpec],
        verify: bool,
    ) -> BatchSweepOutput {
        self.stock
            .sweep_chords(chords, profiles, self.target_tridexel.as_ref(), verify)
    }

    fn query_vertical_plunge_depth(
        &self,
        x_wcs: f64,
        y_wcs: f64,
        z_start_wcs: f64,
        z_end_wcs: f64,
        thread_check_radius_mm: Option<f64>,
    ) -> f64 {
        self.stock.query_vertical_plunge_depth(
            x_wcs,
            y_wcs,
            z_start_wcs,
            z_end_wcs,
            thread_check_radius_mm,
        )
    }

    fn compute_volume_mm3(&self) -> f64 {
        self.stock.compute_volume_mm3()
    }

    fn compute_quick_error_summary(&self) -> Option<(f64, f64, f64)> {
        self.stock
            .compute_quick_error_summary(self.target_tridexel.as_ref())
    }

    fn compute_error_report(
        &self,
        initial_stock_vol_mm3: f64,
        tolerance_mm: f64,
        per_operation: Vec<OperationErrorSnapshot>,
    ) -> Option<StockErrorReport> {
        self.stock.compute_error_report(
            self.target_tridexel.as_ref(),
            initial_stock_vol_mm3,
            tolerance_mm,
            per_operation,
        )
    }

    fn extract_stock_mesh(&self, color_mode: SimStockColorMode) -> SimStockMesh {
        self.stock
            .extract_stock_mesh(self.target_tridexel.as_ref(), color_mode)
    }

    fn into_parts(self) -> (SimulatedStock, Option<TriDexelStock>) {
        (self.stock, self.target_tridexel)
    }
}
