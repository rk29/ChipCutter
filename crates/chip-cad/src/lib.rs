//! UI-agnostic CAD & B-Rep kernel layer for ChipCutter (`chip-cad`).
//!
//! Isolates `monstertruck` behind [`CadKernel`] and exposes canonical `f64` millimeter
//! B-Rep topology (`BodyId`, `FaceId`, `LoopId`, `EdgeId`), analytic geometry classification,
//! `GeometricSignature`s for associative STEP reload, welded vertex-edge topology graphs,
//! open-chain pathfinding, assistive hole/coplanar queries, per-entity tessellations, and
//! spatial ray-picking.

pub mod fixtures;
pub mod kernel;
pub mod model;
pub mod monstertruck_kernel;
pub mod picking;
pub mod topology;

pub use fixtures::{
    SampleStepError, sample_cam_torture_block_step, sample_machined_plate_step,
    sample_machined_plate_step_inches, sample_multi_feature_bracket_step,
};
pub use kernel::{CadError, CadKernel};
pub use model::{
    BRepEdge, BRepFace, BRepLoop, BodyId, CadDocument, CurveKind, EdgeId, EdgePolyline,
    EntitySignatureKind, FaceId, FaceTriangle, GeometricSignature, LoopId, OrientedEdgeUse,
    RenderMesh, RenderVertex, SelectionBinding, SolidBody, StepUnitKind, SurfaceKind,
};
pub use monstertruck_kernel::MonstertruckKernel;
pub use picking::{CadPicker, PickFilter, PickHit, PickTarget};
pub use topology::{
    CylindricalHoleSpan, EdgeEndpointInfo, LogicalCylindricalHole, ModeledChamferStrip,
    PlanarFaceLoopClassification, TopologyGraph, VertexId, analyze_cylindrical_hole,
    classify_planar_face_loops, expand_coaxial_hole_faces, extract_modeled_chamfer_strip,
    find_coplanar_faces, find_same_diameter_holes, group_coaxial_holes,
    highest_adjacent_lower_floor_proj, highest_adjacent_wall_top,
};
