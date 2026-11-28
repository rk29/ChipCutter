//! UI-agnostic CAD & B-Rep kernel layer for ChipCutter (`chip-cad`).
//!
//! Isolates `monstertruck` behind [`CadKernel`] and exposes canonical `f64` millimeter
//! B-Rep topology (`BodyId`, `FaceId`, `LoopId`, `EdgeId`), analytic geometry classification,
//! `GeometricSignature`s for associative STEP reload, welded vertex-edge topology graphs,
//! open-chain pathfinding, assistive hole/coplanar queries, per-entity tessellations, and
//! spatial ray-picking.

pub mod kernel;
pub mod loader;
pub mod measurement;
pub mod mesh;
pub mod model;
pub mod monstertruck_kernel;
pub mod picking;
pub mod rematch;
mod step_periodic;
pub mod topology;
pub mod transform;

pub use kernel::{CadError, CadKernel};
pub use loader::{
    is_mesh_path, load_model_bytes_cad_doc, load_model_entry_cad_doc,
    load_model_entry_cad_doc_transformed,
};
pub use measurement::{
    MeasureSnapTarget, MeasurementResult, measure_pair, measure_single,
    snap_brep_edge_to_measure_target, snap_brep_face_to_measure_target,
    snap_brep_pick_to_measure_target, snap_mesh_hit_to_measure_target,
};
pub use mesh::{
    MeshDiagnostics, MeshFormat, MeshImportOptions, MeshTriangle, MeshUnitKind, TriangleMeshBody,
    build_render_mesh_for_triangle_mesh, encode_binary_stl, is_mesh_extension, load_mesh_bytes,
    load_mesh_cad_document,
};
pub use model::{
    BRepEdge, BRepFace, BRepLoop, BodyId, CadDocument, CurveKind, EdgeId, EdgePolyline,
    EntitySignatureKind, FaceId, FaceTriangle, GeometricSignature, LoopId, MESH_FACE_SENTINEL,
    OrientedEdgeUse, RenderMesh, RenderVertex, SelectionBinding, SolidBody, StepUnitKind,
    SurfaceKind,
};
pub use monstertruck_kernel::MonstertruckKernel;
pub use picking::{CadPicker, PickFilter, PickHit, PickTarget};
pub use rematch::{RematchOutcome, rematch_edge, rematch_face, rematch_loop};
pub use topology::{
    CylindricalHoleSpan, EdgeEndpointInfo, LogicalCylindricalHole, TopologyGraph, VertexId,
    analyze_cylindrical_hole, expand_coaxial_hole_faces, find_coplanar_faces,
    find_same_diameter_holes, group_coaxial_holes,
};
pub use transform::{
    MAX_MODEL_SCALE, MIN_MODEL_SCALE, ModelTransform, rotation_matrix_to_xyz_euler_deg,
};
