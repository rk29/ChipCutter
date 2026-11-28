//! Pure volumetric stock material-removal kernels (`chip-stock`).
//!
//! Provides UI-agnostic, CAD/CAM-agnostic volumetric stock representations and sweep/meshing/
//! metrology kernels with GPU-ready flat storage-buffer layouts:
//! - 2.5D Z-Dexel heightfield (`HeightfieldStock`)
//! - 3D axis-generic orthogonal Tri-Dexel (`TriDexelStock`) with flat fixed-`K` span pools
//! - 4th-axis polar cylindrical stock (`CylindricalStock`)
//! - Unified polymorphic `SimulatedStock`, `StockCheckpointSnapshot`, and `SimStockBackend` trait
//!
//! Zero dependency on `chip-cad`, `chip-cam`, `egui`, or `wgpu`.

#![deny(unsafe_code)]

pub mod backend;
pub mod cylindrical;
#[cfg(feature = "gpu-compute")]
pub mod gpu;
pub mod heightfield;
#[cfg(test)]
mod mesh_skinning_tests;
pub mod metrics;
pub mod tridexel;
pub mod types;
pub mod visual;

pub use backend::{
    CpuExactBackend, SimStockBackend, SimulatedStock, StockCheckpointSnapshot, StockModelKind,
};
pub use cylindrical::{
    CylindricalStock, tessellate_cylindrical_stock_mesh, tessellate_cylindrical_stock_mesh_colored,
    tessellate_cylindrical_stock_mesh_with_options,
};
#[cfg(feature = "gpu-compute")]
pub use gpu::{
    DEFAULT_GPU_CHECKPOINT_BUDGET_BYTES, GpuCheckpointCache, GpuCylindricalBuffers,
    GpuErrorHistogram, GpuHeightfieldBuffers, GpuMeshBvh, GpuSimContext, GpuStockBufferSnapshot,
    GpuTriDexelBuffers, TriDexelOverflowData, WgpuComputeBackend,
    build_tridexel_overflow_supplement_mesh,
};
pub use heightfield::{
    HeightfieldStock, tessellate_stock_mesh, tessellate_stock_mesh_colored,
    tessellate_stock_mesh_with_options,
};
pub use metrics::{
    OperationErrorSnapshot, StockErrorReport, compute_cylindrical_error_report,
    compute_cylindrical_quick_error_summary, compute_heightmap_error_report,
    compute_heightmap_quick_error_summary, compute_tridexel_error_report,
    compute_tridexel_quick_error_summary,
};
pub use tridexel::{
    DexelPlane, DexelSpan, FlatDexelSpan, INLINE_DEXEL_SLOTS, RaySpans, TriDexelStock,
    pack_span_ops, query_overlap_length, query_target_gouge_depth, spans_contain_coord,
    spans_difference_length, subtract_dexel_interval, tessellate_tridexel_stock_mesh,
    tessellate_tridexel_stock_mesh_colored, tessellate_tridexel_stock_mesh_with_options,
    unpack_max_op, unpack_min_op,
};
pub use types::{
    ActiveSimBackendInfo, AnalyticHorizFace, Axis3, BatchSweepOutput, RadialCutterProfile,
    RotaryStockSpec, SimColoredVertex, SimComputeBackendChoice, SimEngineKind, SimError,
    SimMeshOptions, SimResolution, SimStockMesh, SpanRuleViolations, StockPlaybackCursor,
    StockShapeSpec, StockVerifyTolerances, SweepChord, SweepMotion, SweepToolSpec, TargetTriangle,
    UNCUT_OP_IDX,
};
pub use visual::{
    MACHINED_METAL_COLOR, OPERATION_STOCK_PALETTE, RAW_STOCK_SIDE_COLOR, RAW_STOCK_TOP_COLOR,
    SimStockColorMode, deviation_to_heatmap_rgba, op_color,
};
