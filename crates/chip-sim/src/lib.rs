//! Simulation orchestrator, trapezoidal cycle-time timeline, CAD target adapter, and 5-rule
//! crash & gouge verifier (`chip-sim`).
//!
//! Pure volumetric stock kernels (`HeightfieldStock`, `TriDexelStock`, `CylindricalStock`,
//! `SimulatedStock`, `SimStockBackend`, meshing, and metrology) live in `chip-stock` (re-exported
//! as [`chip_stock`] alongside core high-level simulation facade types).

#![deny(unsafe_code)]

pub mod cad_adapter;
pub mod driver;
pub mod progress;
pub mod timeline;
pub mod verifier;
pub(crate) mod verify_common;

pub use chip_stock;

pub use cad_adapter::{
    axis3_from_rotary_axis, axis3_from_stock_axis, extract_cylindrical_target_mesh_from_cad,
    extract_heightfield_target_from_cad, extract_tridexel_target_from_cad,
    rotary_stock_spec_from_cam, stock_shape_spec_from_cam,
};
pub use chip_stock::{
    ActiveSimBackendInfo, Axis3, CylindricalStock, HeightfieldStock, OperationErrorSnapshot,
    RadialCutterProfile, RotaryStockSpec, SimColoredVertex, SimComputeBackendChoice, SimEngineKind,
    SimError, SimMeshOptions, SimResolution, SimStockBackend, SimStockColorMode, SimStockMesh,
    SimulatedStock, StockCheckpointSnapshot, StockErrorReport, StockModelKind, StockPlaybackCursor,
    StockShapeSpec, StockVerifyTolerances, SweepChord, SweepMotion, SweepToolSpec, TriDexelStock,
};
#[cfg(feature = "gpu-compute")]
pub use chip_stock::{GpuSimContext, GpuStockBufferSnapshot, WgpuComputeBackend};
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
