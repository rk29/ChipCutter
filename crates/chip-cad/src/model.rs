use chip_bvh::TriMesh3D;
use chip_math::{Aabb3, DVec3, RigidFrame3};
use serde::{Deserialize, Serialize};

use crate::mesh::{TriangleMeshBody, build_render_mesh_for_triangle_mesh};

/// Sentinel [`FaceId`] attached to [`RenderVertex::face_id`] for imported triangle meshes
/// (distinct from `u32::MAX` "no hovered face" and valid B-Rep face indices).
pub const MESH_FACE_SENTINEL: FaceId = FaceId(u32::MAX - 1);

/// Identifier of a solid or shell body within a [`CadDocument`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct BodyId(pub u32);

/// Identifier of a topological B-Rep face across a [`CadDocument`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct FaceId(pub u32);

/// Identifier of a closed or open edge loop / wire across a [`CadDocument`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct LoopId(pub u32);

/// Identifier of a topological B-Rep edge across a [`CadDocument`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct EdgeId(pub u32);

/// Original length unit declared inside an imported STEP file prior to `f64` mm normalization.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum StepUnitKind {
    Millimeter,
    Centimeter,
    Meter,
    Inch,
    CustomScaleToMm(f64),
}

impl StepUnitKind {
    pub fn scale_to_mm(self) -> f64 {
        match self {
            Self::Millimeter => 1.0,
            Self::Centimeter => 10.0,
            Self::Meter => 1000.0,
            Self::Inch => 25.4,
            Self::CustomScaleToMm(s) => s,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Millimeter => "MILLIMETRE (1.0 mm)",
            Self::Centimeter => "CENTIMETRE (10.0 mm)",
            Self::Meter => "METRE (1000.0 mm)",
            Self::Inch => "INCH (25.4 mm)",
            Self::CustomScaleToMm(_) => "CUSTOM",
        }
    }
}

/// Analytic or freeform classification of a B-Rep face in canonical `f64` millimeters.
/// `normal` on [`SurfaceKind::Plane`] always reflects the outward solid face orientation.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum SurfaceKind {
    Plane {
        origin_mm: DVec3,
        normal: DVec3,
    },
    Cylinder {
        axis_origin_mm: DVec3,
        axis_dir: DVec3,
        radius_mm: f64,
        /// `true` when the face normal points inward toward the cylinder axis (a hole / bore).
        is_internal: bool,
    },
    Cone {
        apex_mm: DVec3,
        axis_dir: DVec3,
        half_angle_rad: f64,
    },
    Torus {
        center_mm: DVec3,
        axis_dir: DVec3,
        major_radius_mm: f64,
        minor_radius_mm: f64,
    },
    OtherNurbs,
}

impl SurfaceKind {
    pub fn short_label(&self) -> &'static str {
        match self {
            Self::Plane { .. } => "Plane",
            Self::Cylinder {
                is_internal: true, ..
            } => "Cylindrical Hole (Internal)",
            Self::Cylinder {
                is_internal: false, ..
            } => "Cylindrical Boss/Fillet (External)",
            Self::Cone { .. } => "Cone",
            Self::Torus { .. } => "Torus",
            Self::OtherNurbs => "Freeform / NURBS",
        }
    }
}

/// Analytic or freeform classification of a 3D B-Rep edge curve in canonical `f64` millimeters.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum CurveKind {
    Line {
        start_mm: DVec3,
        end_mm: DVec3,
    },
    CircleArc {
        center_mm: DVec3,
        normal: DVec3,
        radius_mm: f64,
        start_mm: DVec3,
        end_mm: DVec3,
        is_full_circle: bool,
    },
    EllipseArc,
    OtherNurbs,
}

impl CurveKind {
    pub fn short_label(&self) -> &'static str {
        match self {
            Self::Line { .. } => "Line",
            Self::CircleArc {
                is_full_circle: true,
                ..
            } => "Full Circle",
            Self::CircleArc {
                is_full_circle: false,
                ..
            } => "Circular Arc",
            Self::EllipseArc => "Ellipse Arc",
            Self::OtherNurbs => "Spline / NURBS Curve",
        }
    }
}

/// Entity category stored inside a [`GeometricSignature`] for CAD reload re-matching.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EntitySignatureKind {
    PlanarFace,
    InternalCylinderFace,
    ExternalCylinderFace,
    ConicalFace,
    OtherFace,
    LineEdge,
    CircleEdge,
    OtherEdge,
    EdgeLoop,
}

/// Topology-independent geometric fingerprint used to re-bind selections on STEP reload.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GeometricSignature {
    pub kind: EntitySignatureKind,
    pub normal_or_axis: DVec3,
    pub centroid_mm: DVec3,
    /// Characteristic scalar: area (`mm²`) for planar/general faces, radius (`mm`) for cylinders/arcs,
    /// or chord/arc length (`mm`) for linear/loop entities.
    pub characteristic_measure: f64,
    pub aabb_mm: Aabb3,
}

/// Persistent CAD entity reference combining last-known topology index and [`GeometricSignature`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SelectionBinding {
    pub body_id: BodyId,
    pub last_index: u32,
    pub signature: GeometricSignature,
}

/// Directed use of an [`EdgeId`] inside a [`BRepLoop`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct OrientedEdgeUse {
    pub edge_id: EdgeId,
    pub forward: bool,
}

/// Boundary edge loop (outer face perimeter or inner island/hole wire).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BRepLoop {
    pub id: LoopId,
    pub face_id: FaceId,
    pub is_outer: bool,
    pub edges: Vec<OrientedEdgeUse>,
    pub signature: GeometricSignature,
}

/// Topological B-Rep face with exact `f64` analytic metadata.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BRepFace {
    pub id: FaceId,
    pub body_id: BodyId,
    pub surface: SurfaceKind,
    pub area_mm2: f64,
    pub aabb_mm: Aabb3,
    pub outer_loop: Option<LoopId>,
    pub inner_loops: Vec<LoopId>,
    pub signature: GeometricSignature,
}

/// Topological B-Rep edge shared by one or more faces.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BRepEdge {
    pub id: EdgeId,
    pub body_id: BodyId,
    pub curve: CurveKind,
    pub length_mm: f64,
    pub aabb_mm: Aabb3,
    pub adjacent_faces: Vec<FaceId>,
    pub loops: Vec<LoopId>,
    pub signature: GeometricSignature,
}

/// Solid or closed shell body inside a [`CadDocument`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolidBody {
    pub id: BodyId,
    pub name: String,
    pub faces: Vec<FaceId>,
    pub edges: Vec<EdgeId>,
    pub aabb_mm: Aabb3,
}

/// Vertex of the viewport tessellation mesh (`f32` positions/normals + `FaceId` for GPU highlighting).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RenderVertex {
    pub position_mm: [f32; 3],
    pub normal: [f32; 3],
    pub face_id: u32,
}

/// Exact `f64` triangle retained on CPU for sub-millimeter ray-picking.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceTriangle {
    pub face_id: FaceId,
    pub v0_mm: DVec3,
    pub v1_mm: DVec3,
    pub v2_mm: DVec3,
}

/// Discretized 3D polyline for a single topological [`EdgeId`].
#[derive(Debug, Clone, PartialEq)]
pub struct EdgePolyline {
    pub edge_id: EdgeId,
    pub points_mm: Vec<DVec3>,
}

/// Combined tessellation output for viewport rendering and CPU ray-picking.
#[derive(Debug, Clone, Default)]
pub struct RenderMesh {
    pub vertices: Vec<RenderVertex>,
    pub indices: Vec<u32>,
    pub pick_triangles: Vec<FaceTriangle>,
    pub edge_polylines: Vec<EdgePolyline>,
}

/// Complete imported CAD document normalized to canonical `f64` millimeters.
#[derive(Debug, Clone)]
pub struct CadDocument {
    pub source_name: String,
    pub detected_unit: StepUnitKind,
    pub bodies: Vec<SolidBody>,
    pub meshes: Vec<TriangleMeshBody>,
    pub faces: Vec<BRepFace>,
    pub loops: Vec<BRepLoop>,
    pub edges: Vec<BRepEdge>,
    pub aabb_mm: Aabb3,
    pub render_mesh: RenderMesh,
}

impl CadDocument {
    /// Returns an empty canonical `CadDocument` with zero 3D solid or mesh bodies (used for Pure-2D projects).
    pub fn empty() -> Self {
        Self {
            source_name: String::new(),
            detected_unit: StepUnitKind::Millimeter,
            bodies: Vec::new(),
            meshes: Vec::new(),
            faces: Vec::new(),
            loops: Vec::new(),
            edges: Vec::new(),
            aabb_mm: Aabb3::EMPTY,
            render_mesh: RenderMesh {
                vertices: Vec::new(),
                indices: Vec::new(),
                pick_triangles: Vec::new(),
                edge_polylines: Vec::new(),
            },
        }
    }

    /// Constructs a [`CadDocument`] wrapping a single imported [`TriangleMeshBody`] (`STL`/`OBJ`/`3MF`)
    /// with crease-angle viewport shading (`35°`) and unified `aabb_mm`.
    pub fn from_mesh(source_name: String, mesh: TriangleMeshBody) -> Self {
        let detected_unit = mesh.applied_unit.to_step_unit_kind();
        let aabb_mm = mesh.aabb_mm;
        let render_mesh = build_render_mesh_for_triangle_mesh(&mesh, 35.0, MESH_FACE_SENTINEL);
        Self {
            source_name,
            detected_unit,
            bodies: Vec::new(),
            meshes: vec![mesh],
            faces: Vec::new(),
            loops: Vec::new(),
            edges: Vec::new(),
            aabb_mm,
            render_mesh,
        }
    }

    /// Returns `true` if this document holds at least one 3D B-Rep solid body or imported triangle mesh.
    #[inline]
    pub fn has_3d_geometry(&self) -> bool {
        !self.bodies.is_empty() || !self.meshes.is_empty()
    }

    #[inline]
    pub fn face(&self, id: FaceId) -> Option<&BRepFace> {
        self.faces.get(id.0 as usize)
    }

    #[inline]
    pub fn edge(&self, id: EdgeId) -> Option<&BRepEdge> {
        self.edges.get(id.0 as usize)
    }

    #[inline]
    pub fn brep_loop(&self, id: LoopId) -> Option<&BRepLoop> {
        self.loops.get(id.0 as usize)
    }

    /// Creates a persistent [`SelectionBinding`] for a [`FaceId`].
    pub fn bind_face(&self, id: FaceId) -> Option<SelectionBinding> {
        let f = self.face(id)?;
        Some(SelectionBinding {
            body_id: f.body_id,
            last_index: id.0,
            signature: f.signature.clone(),
        })
    }

    /// Creates a persistent [`SelectionBinding`] for an [`EdgeId`].
    pub fn bind_edge(&self, id: EdgeId) -> Option<SelectionBinding> {
        let e = self.edge(id)?;
        Some(SelectionBinding {
            body_id: e.body_id,
            last_index: id.0,
            signature: e.signature.clone(),
        })
    }

    /// Creates a persistent [`SelectionBinding`] for a [`LoopId`].
    pub fn bind_loop(&self, id: LoopId) -> Option<SelectionBinding> {
        let lp = self.brep_loop(id)?;
        let face = self.face(lp.face_id)?;
        Some(SelectionBinding {
            body_id: face.body_id,
            last_index: id.0,
            signature: lp.signature.clone(),
        })
    }

    /// Expands an [`EdgeId`] to all edges belonging to its primary boundary loop.
    pub fn expand_primary_loop(&self, edge_id: EdgeId) -> Vec<EdgeId> {
        let Some(edge) = self.edge(edge_id) else {
            return vec![edge_id];
        };
        if let Some(&loop_id) = edge.loops.first()
            && let Some(lp) = self.brep_loop(loop_id)
        {
            return lp.edges.iter().map(|u| u.edge_id).collect();
        }
        vec![edge_id]
    }

    /// Builds a unified WCS-space [`chip_bvh::TriMesh3D`] from all targeted bodies (`self.meshes`
    /// and/or `self.render_mesh.pick_triangles` for STEP B-Rep solids) transformed into `setup_frame`.
    pub fn build_wcs_tri_mesh(
        &self,
        target_bodies: &[BodyId],
        setup_frame: &RigidFrame3,
    ) -> TriMesh3D {
        let include_body = |bid: BodyId| target_bodies.is_empty() || target_bodies.contains(&bid);

        let mut vertices: Vec<DVec3> = Vec::new();
        let mut indexed_faces: Vec<([u32; 3], u32)> = Vec::new();

        // 1. Imported triangle meshes (`STL` / `OBJ` / `3MF`)
        for mesh in &self.meshes {
            if !include_body(mesh.id) {
                continue;
            }
            let Ok(base) = u32::try_from(vertices.len()) else {
                break;
            };
            vertices.extend(
                mesh.vertices_mm
                    .iter()
                    .map(|&v| setup_frame.world_to_local_point(v)),
            );
            for tri in &mesh.triangles {
                indexed_faces.push((
                    [
                        base + tri.indices[0],
                        base + tri.indices[1],
                        base + tri.indices[2],
                    ],
                    MESH_FACE_SENTINEL.0,
                ));
            }
        }

        // 2. STEP B-Rep `pick_triangles` (exact `f64` surface tessellation)
        for tri in &self.render_mesh.pick_triangles {
            let Some(face) = self.face(tri.face_id) else {
                continue;
            };
            if !include_body(face.body_id) {
                continue;
            }
            let Ok(base) = u32::try_from(vertices.len()) else {
                break;
            };
            vertices.push(setup_frame.world_to_local_point(tri.v0_mm));
            vertices.push(setup_frame.world_to_local_point(tri.v1_mm));
            vertices.push(setup_frame.world_to_local_point(tri.v2_mm));
            indexed_faces.push(([base, base + 1, base + 2], tri.face_id.0));
        }

        TriMesh3D::from_indexed_with_faces(vertices, &indexed_faces)
    }
}
