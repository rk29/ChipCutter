//! 2.5D Z-Dexel heightmap stock material removal, trapezoidal cycle-time timeline,
//! 3D cutter/stock mesh builders, and 5-rule crash & gouge verifier (`chip-sim`).

pub mod cutter_profile;
pub mod cylindrical;
pub mod heightmap;
pub(crate) mod mesher_common;
pub mod metrics;
pub mod timeline;
pub mod tridexel;
pub mod verifier;
pub(crate) mod verify_common;

pub use cutter_profile::{
    RadialCutterProfile, SimColoredVertex, SimToolMesh, ToolMeshZone, build_cutter_assembly_mesh,
};
pub use cylindrical::{
    CylindricalCheckpoint, CylindricalStock, run_cylindrical_setup_simulation,
    tessellate_cylindrical_stock_mesh, tessellate_cylindrical_stock_mesh_colored,
};
pub use heightmap::{
    HeightfieldStock, OPERATION_STOCK_PALETTE, SetupSimulationInput, SetupSimulationResult,
    SimError, SimResolution, SimStockMesh, StockCheckpoint, StockPlaybackCursor, UNCUT_OP_IDX,
    run_setup_simulation, run_setup_simulation_with_engine, tessellate_stock_mesh,
    tessellate_stock_mesh_colored,
};
pub use metrics::{
    OperationErrorSnapshot, SimStockColorMode, StockErrorReport, compute_cylindrical_error_report,
    compute_heightmap_error_report, compute_tridexel_error_report, deviation_to_heatmap_rgba,
};
pub use timeline::{
    MachineKinematicsConfig, MotionSpan, OperationTimelineSummary, ScrubSnapshot,
    SimOperationInput, SimulationTimeline, ToolChangeSnapshot, format_cycle_time_hms,
};
pub use tridexel::{
    DexelPlane, DexelSpan, SimEngineKind, TriDexelCheckpoint, TriDexelStock,
    run_tridexel_setup_simulation, tessellate_tridexel_stock_mesh,
    tessellate_tridexel_stock_mesh_colored,
};
pub use verifier::{SimDiagnostic, SimIssueKind, SimSeverity, SimVerificationConfig};
