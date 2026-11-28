//! Optional `wgpu` GPU compute volumetric stock simulation backend (`#[cfg(feature = "gpu-compute")]`).

pub mod backend;
pub mod bvh;
pub mod checkpoints;
pub mod context;
pub mod cylindrical;
pub mod heightfield;
pub mod metrics;
pub mod snapshot;
pub mod tridexel;
pub mod types;

pub use backend::WgpuComputeBackend;
pub use bvh::GpuMeshBvh;
pub use checkpoints::{DEFAULT_GPU_CHECKPOINT_BUDGET_BYTES, GpuCheckpointCache};
pub use context::GpuSimContext;
pub use metrics::GpuErrorHistogram;
pub use snapshot::{
    GpuCylindricalBuffers, GpuHeightfieldBuffers, GpuStockBufferSnapshot, GpuTriDexelBuffers,
    TriDexelOverflowData,
};
pub use tridexel::build_tridexel_overflow_supplement_mesh;
pub use types::{
    BvhDispatchUniforms, CylindricalUniforms, GPU_METRICS_HIST_BINS, GPU_METRICS_MAX_WORKGROUPS,
    GPU_TILE_SIZE, GpuMetricsPartial, GpuRotaryPose, GpuSweepChord, GpuToolProfile,
    GpuViolationAccum, GpuViolationSlot, HeightfieldUniforms, MetricsUniforms,
    TriDexelPlaneUniforms,
};
