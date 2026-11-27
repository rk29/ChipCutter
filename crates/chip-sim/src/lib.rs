//! Simulation orchestrator, trapezoidal cycle-time timeline, 3D cutter assembly mesh builder,
//! CAD target adapter, and 5-rule crash & gouge verifier (`chip-sim`).
//!
//! Pure volumetric stock kernels (`HeightfieldStock`, `TriDexelStock`, `CylindricalStock`,
//! `SimulatedStock`, `SimStockBackend`, meshing, and metrology) live in `chip-stock` and are
//! re-exported here so downstream callers (`chip-project`, `chip-cli`, `chip-gui`) remain stable.

#![deny(unsafe_code)]

pub mod cad_adapter;
pub mod cutter_profile;
pub mod driver;
pub mod progress;
pub mod timeline;
pub mod verifier;
pub(crate) mod verify_common;

pub use cad_adapter::{
    axis3_from_rotary_axis, axis3_from_stock_axis, extract_cylindrical_target_mesh_from_cad,
    extract_heightfield_target_from_cad, extract_tridexel_target_from_cad,
    rotary_stock_spec_from_cam, stock_shape_spec_from_cam,
};
pub use chip_stock::{
    ActiveSimBackendInfo, AnalyticHorizFace, Axis3, BatchSweepOutput, CpuExactBackend,
    CylindricalStock, DexelPlane, DexelSpan, FlatDexelSpan, HeightfieldStock, INLINE_DEXEL_SLOTS,
    MACHINED_METAL_COLOR, OPERATION_STOCK_PALETTE, OperationErrorSnapshot, RAW_STOCK_SIDE_COLOR,
    RAW_STOCK_TOP_COLOR, RadialCutterProfile, RaySpans, RotaryStockSpec, SimColoredVertex,
    SimComputeBackendChoice, SimEngineKind, SimError, SimMeshOptions, SimResolution,
    SimStockBackend, SimStockColorMode, SimStockMesh, SimulatedStock, SpanRuleViolations,
    StockCheckpointSnapshot, StockErrorReport, StockModelKind, StockPlaybackCursor, StockShapeSpec,
    StockVerifyTolerances, SweepChord, SweepMotion, SweepToolSpec, TargetTriangle, TriDexelStock,
    UNCUT_OP_IDX, compute_cylindrical_error_report, compute_heightmap_error_report,
    compute_tridexel_error_report, deviation_to_heatmap_rgba, op_color, pack_span_ops,
    query_overlap_length, query_target_gouge_depth, spans_contain_coord, spans_difference_length,
    subtract_dexel_interval, tessellate_cylindrical_stock_mesh,
    tessellate_cylindrical_stock_mesh_colored, tessellate_cylindrical_stock_mesh_with_options,
    tessellate_stock_mesh, tessellate_stock_mesh_colored, tessellate_stock_mesh_with_options,
    tessellate_tridexel_stock_mesh, tessellate_tridexel_stock_mesh_colored,
    tessellate_tridexel_stock_mesh_with_options, unpack_max_op, unpack_min_op,
};
#[cfg(feature = "gpu-compute")]
pub use chip_stock::{
    DEFAULT_GPU_CHECKPOINT_BUDGET_BYTES, GpuCheckpointCache, GpuCylindricalRenderData,
    GpuHeightfieldRenderData, GpuSimContext, GpuStockRenderHandle, GpuTriDexelRenderData,
    MAX_TRIDEXEL_EMIT_QUADS, PackedTriDexelQuad, TRIDEXEL_DUAL_OFFSET_PACK_SCALE,
    TRIDEXEL_PACKED_QUAD_BYTES, WgpuComputeBackend, oct_decode_unit_normal, oct_encode_unit_normal,
    pack_oct_normal_pair_12bit, unpack_oct_normal_12bit,
};
pub use cutter_profile::{SimToolMesh, build_cutter_assembly_mesh};
pub use driver::{
    SetupSimulationInput, SetupSimulationResult, SimTimingBreakdown, SimulatedStockPlaybackExt,
    run_setup_simulation, run_setup_simulation_with_backend,
};
pub use progress::{SimProgressScope, SimProgressSnapshot, SimProgressStage, SimProgressTracker};
pub use timeline::{
    MachineKinematicsConfig, MotionSpan, OperationTimelineSummary, ScrubSnapshot,
    SimOperationInput, SimulationTimeline, ToolChangeSnapshot, format_cycle_time_hms,
};
pub use verifier::{SimDiagnostic, SimIssueKind, SimSeverity, SimVerificationConfig};
