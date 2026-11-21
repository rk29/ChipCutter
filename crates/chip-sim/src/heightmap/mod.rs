mod grid;
mod mesher;
mod sweeper;
mod target_rasterizer;

pub(crate) use grid::slice_segment_range;
pub use grid::{HeightfieldStock, UNCUT_OP_IDX};
pub use mesher::{OPERATION_STOCK_PALETTE, tessellate_stock_mesh, tessellate_stock_mesh_colored};
pub use sweeper::{run_setup_simulation, run_setup_simulation_with_engine};

use std::sync::atomic::AtomicBool;

use chip_cad::{BodyId, CadDocument};
use chip_cam::ResolvedSetup;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    cutter_profile::{RadialCutterProfile, SimColoredVertex},
    cylindrical::{CylindricalCheckpoint, CylindricalStock},
    metrics::StockErrorReport,
    timeline::{MachineKinematicsConfig, ScrubSnapshot, SimOperationInput, SimulationTimeline},
    tridexel::{SimEngineKind, TriDexelCheckpoint, TriDexelStock},
    verifier::{SimDiagnostic, SimSeverity, SimVerificationConfig},
};

/// Errors raised during heightmap simulation.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum SimError {
    #[error("simulation cancelled")]
    Cancelled,
    #[error("resolved stock bounding box is invalid or degenerate")]
    InvalidStockBounds,
}

/// Configurable Z-dexel grid resolution (`06-simulation.md` §3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimResolution {
    /// `~512 × 512` cells (instant interactive updates).
    #[default]
    Standard,
    /// `~1024 × 1024` cells (high-precision verification).
    Fine,
    /// `~2048 × 2048` cells (ultra-fine inspection).
    Ultra,
}

impl SimResolution {
    pub const ALL: [Self; 3] = [Self::Standard, Self::Fine, Self::Ultra];

    pub const fn max_grid_dim(self) -> usize {
        match self {
            Self::Standard => 512,
            Self::Fine => 1024,
            Self::Ultra => 2048,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::Standard => "Standard (512)",
            Self::Fine => "Fine (1024)",
            Self::Ultra => "Ultra (2048)",
        }
    }
}

/// Borrowed input bundle passed to [`run_setup_simulation`].
#[derive(Clone, Copy)]
pub struct SetupSimulationInput<'a> {
    pub doc: &'a CadDocument,
    pub setup: &'a ResolvedSetup,
    pub target_bodies: &'a [BodyId],
    pub operations: &'a [SimOperationInput<'a>],
    pub resolution: SimResolution,
    pub kinematics: MachineKinematicsConfig,
    pub verify_config: SimVerificationConfig,
    pub cancel: Option<&'a AtomicBool>,
}

/// Triangulated 3D machined stock mesh (`pos_mm`, `normal`, `color_rgba` in Setup WCS).
#[derive(Debug, Clone, Default)]
pub struct SimStockMesh {
    pub vertices: Vec<SimColoredVertex>,
    pub indices: Vec<u32>,
}

/// Snapshot of `z_stock` and `last_op_idx` after completing `completed_segments` motion spans,
/// enabling `< 4 ms` interactive timeline scrubbing to any timestamp.
#[derive(Debug, Clone)]
pub struct StockCheckpoint {
    pub completed_segments: usize,
    pub z_stock: Vec<f32>,
    pub last_op_idx: Vec<u16>,
}

/// Tracks the exact timeline position `(seg_idx, seg_u)` of an incrementally synchronized
/// [`HeightfieldStock`] or [`CylindricalStock`]: all segments `0..seg_idx` are fully swept,
/// plus the prefix `[0.0 .. seg_u]` of segment `seg_idx`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StockPlaybackCursor {
    pub seg_idx: usize,
    pub seg_u: f64,
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

/// Complete output of simulating a Setup (`timeline`, `final_stock`, `checkpoints`, `diagnostics`).
#[derive(Debug, Clone)]
pub struct SetupSimulationResult {
    pub engine: SimEngineKind,
    pub resolution: SimResolution,
    pub timeline: SimulationTimeline,
    pub initial_stock_volume_mm3: f64,
    pub remaining_stock_volume_mm3: f64,
    pub removed_volume_mm3: f64,
    pub final_stock: HeightfieldStock,
    pub final_cylindrical_stock: Option<CylindricalStock>,
    pub final_tridexel_stock: Option<TriDexelStock>,
    pub target_tridexel_stock: Option<TriDexelStock>,
    pub checkpoints: Vec<StockCheckpoint>,
    pub cylindrical_checkpoints: Vec<CylindricalCheckpoint>,
    pub tridexel_checkpoints: Vec<TriDexelCheckpoint>,
    /// Per-operation `(RadialCutterProfile, is_flat_endmill)` captured during simulation for exact
    /// interactive timeline reconstruction even if project operations are later edited.
    pub operation_profiles: Vec<(RadialCutterProfile, bool)>,
    pub diagnostics: Vec<SimDiagnostic>,
    /// Quantitative stock-vs-target error volume & surface deviation metrics (when 3D target model exists).
    pub error_report: Option<StockErrorReport>,
}

impl SetupSimulationResult {
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
        if let (Some(td_stock), Some(td_target)) = (
            self.final_tridexel_stock.as_ref(),
            self.target_tridexel_stock.as_ref(),
        ) {
            Some(crate::metrics::compute_tridexel_error_report(
                td_stock,
                td_target,
                self.initial_stock_volume_mm3,
                tolerance_mm,
                per_op,
            ))
        } else {
            crate::metrics::compute_heightmap_error_report(
                &self.final_stock,
                self.initial_stock_volume_mm3,
                tolerance_mm,
                per_op,
            )
        }
    }
}
