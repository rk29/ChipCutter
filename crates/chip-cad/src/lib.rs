//! UI-agnostic CAD & B-Rep kernel layer for ChipCutter (`chip-cad`).
//!
//! Isolates `monstertruck` behind [`CadKernel`] and exposes canonical `f64` millimeter
//! B-Rep topology (`BodyId`, `FaceId`, `LoopId`, `EdgeId`), analytic geometry classification,
//! `GeometricSignature`s for associative STEP reload, welded vertex-edge topology graphs,
//! open-chain pathfinding, assistive hole/coplanar queries, per-entity tessellations, and
//! spatial ray-picking.

pub mod fixtures;
pub mod kernel;
pub mod mesh;
pub mod model;
pub mod monstertruck_kernel;
pub mod picking;
pub mod rematch;
pub mod topology;
pub mod transform;

pub use fixtures::{
    SampleStepError, sample_ambiguous_twin_hole_step, sample_cam_torture_block_step,
    sample_machined_plate_step, sample_machined_plate_step_inches,
    sample_multi_feature_bracket_step, sample_revised_part_pair_step,
    tutorial_adaptive_pocket_step, tutorial_chamfer_engrave_block_step,
    tutorial_contour_plate_step, tutorial_drill_bore_plate_step, tutorial_facing_billet_step,
    tutorial_indexed_4side_block_step, tutorial_pocket_block_step,
};
pub use kernel::{CadError, CadKernel};
pub use mesh::{
    MeshBvh, MeshDiagnostics, MeshFormat, MeshImportOptions, MeshRayHit, MeshTriangle,
    MeshUnitKind, TriangleMeshBody, build_render_mesh_for_triangle_mesh, is_mesh_extension,
    load_mesh_bytes, load_mesh_cad_document, sample_box_3mf_bytes,
    sample_helical_flute_shaft_stl_bytes, sample_relief_wave_obj_text,
    sample_rotary_shaft_stl_bytes, sample_sculpted_dome_stl_bytes, tutorial_yup_bracket_stl_bytes,
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
    CylindricalHoleSpan, EdgeEndpointInfo, LogicalCylindricalHole, ModeledChamferStrip,
    PlanarFaceLoopClassification, TopologyGraph, VertexId, analyze_cylindrical_hole,
    classify_planar_face_loops, expand_coaxial_hole_faces, extract_modeled_chamfer_strip,
    find_coplanar_faces, find_same_diameter_holes, group_coaxial_holes,
    highest_adjacent_lower_floor_proj, highest_adjacent_wall_top,
};
pub use transform::{
    MAX_MODEL_SCALE, MIN_MODEL_SCALE, ModelTransform, rotation_matrix_to_xyz_euler_deg,
};
