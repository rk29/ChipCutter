//! UI-agnostic CAD & B-Rep kernel layer for ChipCutter (`chip-cad`).
//!
//! Isolates `monstertruck` behind [`CadKernel`] and exposes canonical `f64` millimeter
//! B-Rep topology (`BodyId`, `FaceId`, `LoopId`, `EdgeId`), analytic geometry classification,
//! `GeometricSignature`s for associative STEP reload, per-entity tessellations, and spatial
//! ray-picking.

pub mod fixtures;
pub mod kernel;
pub mod model;
pub mod monstertruck_kernel;
pub mod picking;

pub use fixtures::{SampleStepError, sample_machined_plate_step};
pub use kernel::{CadError, CadKernel};
pub use model::{
    BRepEdge, BRepFace, BRepLoop, BodyId, CadDocument, CurveKind, EdgeId, EdgePolyline,
    EntitySignatureKind, FaceId, FaceTriangle, GeometricSignature, LoopId, OrientedEdgeUse,
    RenderMesh, RenderVertex, SolidBody, StepUnitKind, SurfaceKind,
};
pub use monstertruck_kernel::MonstertruckKernel;
pub use picking::{CadPicker, PickFilter, PickHit, PickTarget};
