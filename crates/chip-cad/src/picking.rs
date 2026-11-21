use chip_bvh::{BvhSegment, SegmentBvh, TriMesh3D, ray_triangle_intersect};
use chip_math::{DVec3, Ray3};

use crate::model::{CadDocument, EdgeId, FaceId, LoopId, SurfaceKind};

/// Filter controlling what B-Rep entity classes are selectable in the 3D viewport.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PickFilter {
    #[default]
    Any,
    FacesOnly,
    EdgesOnly,
    EdgeLoops,
    CylindricalHolesOnly,
}

impl PickFilter {
    pub const ALL: [Self; 5] = [
        Self::Any,
        Self::FacesOnly,
        Self::EdgesOnly,
        Self::EdgeLoops,
        Self::CylindricalHolesOnly,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Any => "Any (Face + Edge)",
            Self::FacesOnly => "Faces",
            Self::EdgesOnly => "Single Edges",
            Self::EdgeLoops => "Edge Loops",
            Self::CylindricalHolesOnly => "Cylindrical Holes",
        }
    }
}

/// Specific topological entity hit by a 3D viewport raycast.
#[derive(Debug, Clone, PartialEq)]
pub enum PickTarget {
    Face(FaceId),
    Edge(EdgeId),
    Loop {
        loop_id: LoopId,
        seed_edge: EdgeId,
        edges: Vec<EdgeId>,
    },
}

/// Result of a spatial pick query in canonical `f64` millimeters.
#[derive(Debug, Clone, PartialEq)]
pub struct PickHit {
    pub target: PickTarget,
    pub point_mm: DVec3,
    pub ray_t: f64,
}

/// Flat-BVH spatial acceleration structure for `O(log N)` sub-millisecond hover and click picking.
#[derive(Debug, Clone, Default)]
pub struct CadPicker {
    tri_mesh: TriMesh3D,
    seg_bvh: SegmentBvh,
}

impl CadPicker {
    /// Builds flat SAH BVH spatial pick indexes from an imported [`CadDocument`].
    pub fn build(doc: &CadDocument) -> Self {
        let raw_tris: Vec<([DVec3; 3], u32)> = doc
            .render_mesh
            .pick_triangles
            .iter()
            .map(|tri| ([tri.v0_mm, tri.v1_mm, tri.v2_mm], tri.face_id.0))
            .collect();

        let seg_cap: usize = doc
            .render_mesh
            .edge_polylines
            .iter()
            .map(|p| p.points_mm.len().saturating_sub(1))
            .sum();
        let mut segments = Vec::with_capacity(seg_cap);
        for poly in &doc.render_mesh.edge_polylines {
            for w in poly.points_mm.windows(2) {
                segments.push(BvhSegment {
                    a: w[0],
                    b: w[1],
                    user_id: poly.edge_id.0,
                });
            }
        }

        Self {
            tri_mesh: TriMesh3D::from_raw_triangles_with_faces(&raw_tris),
            seg_bvh: SegmentBvh::build(segments),
        }
    }

    /// Casts a 3D picking ray (`f64` mm) against the B-Rep model in `O(log N)` time.
    ///
    /// `mm_per_pixel` gives the world-space scale at the camera target so thin topological edges
    /// have a crisp ~6-pixel hover radius and win cleanly over the underlying face without picking
    /// occluded back-side edges!
    pub fn pick(
        &self,
        doc: &CadDocument,
        ray: &Ray3,
        mm_per_pixel: f64,
        filter: PickFilter,
    ) -> Option<PickHit> {
        let edge_radius_mm = (mm_per_pixel * 6.5).max(0.15);

        // 1. Closest face triangle intersection (also serves as depth occlusion barrier for edges!)
        let mut closest_any_face_t = f64::INFINITY;
        let mut best_filtered_face: Option<(FaceId, f64)> = None;

        match filter {
            PickFilter::CylindricalHolesOnly => {
                if let Some((t, face_id)) = self.tri_mesh.bvh.traverse_ray_closest(
                    ray.origin,
                    ray.dir,
                    f64::INFINITY,
                    |tri_idx, best_t| {
                        let tri = self.tri_mesh.triangles.get(tri_idx)?;
                        let fid = FaceId(tri.face_id);
                        let is_hole = doc.face(fid).is_some_and(|f| {
                            matches!(
                                f.surface,
                                SurfaceKind::Cylinder {
                                    is_internal: true,
                                    ..
                                }
                            )
                        });
                        if !is_hole {
                            return None;
                        }
                        let (v0, v1, v2) = self.tri_mesh.triangle_vertices(tri_idx)?;
                        let (t, _, _) =
                            ray_triangle_intersect(ray.origin, ray.dir, v0, v1, v2, best_t)?;
                        Some((t, fid))
                    },
                ) {
                    best_filtered_face = Some((face_id, t));
                }
            }
            PickFilter::EdgesOnly | PickFilter::EdgeLoops => {
                if let Some(hit) = self.tri_mesh.raycast(ray.origin, ray.dir, f64::INFINITY) {
                    closest_any_face_t = hit.t;
                }
            }
            PickFilter::Any | PickFilter::FacesOnly => {
                if doc.faces.is_empty() {
                    if let Some(hit) = self.tri_mesh.raycast(ray.origin, ray.dir, f64::INFINITY) {
                        closest_any_face_t = hit.t;
                    }
                } else if let Some((t, face_id)) = self.tri_mesh.bvh.traverse_ray_closest(
                    ray.origin,
                    ray.dir,
                    f64::INFINITY,
                    |tri_idx, best_t| {
                        let tri = self.tri_mesh.triangles.get(tri_idx)?;
                        let (v0, v1, v2) = self.tri_mesh.triangle_vertices(tri_idx)?;
                        let (t, _, _) =
                            ray_triangle_intersect(ray.origin, ray.dir, v0, v1, v2, best_t)?;
                        if t < closest_any_face_t {
                            closest_any_face_t = t;
                        }
                        let fid = FaceId(tri.face_id);
                        if doc.face(fid).is_some() {
                            Some((t, fid))
                        } else {
                            None
                        }
                    },
                ) {
                    best_filtered_face = Some((face_id, t));
                }
            }
        }

        // 2. Closest unoccluded edge segment within screen-space pick radius
        let check_edges = matches!(
            filter,
            PickFilter::Any | PickFilter::EdgesOnly | PickFilter::EdgeLoops
        );
        let occlusion_limit = if filter == PickFilter::EdgesOnly || filter == PickFilter::EdgeLoops
        {
            // Still block back-of-part edges occluded by solid faces, with generous crease tolerance
            closest_any_face_t + edge_radius_mm * 1.5
        } else {
            closest_any_face_t + edge_radius_mm * 1.25
        };

        let best_edge = if check_edges {
            self.seg_bvh.pick_ray(ray, edge_radius_mm, occlusion_limit)
        } else {
            None
        };

        if let Some(edge_hit) = best_edge {
            let edge_id = EdgeId(edge_hit.user_id);
            let target = if filter == PickFilter::EdgeLoops {
                let edge_rec = doc.edge(edge_id)?;
                if let Some(&loop_id) = edge_rec.loops.first() {
                    let edges = doc.expand_primary_loop(edge_id);
                    PickTarget::Loop {
                        loop_id,
                        seed_edge: edge_id,
                        edges,
                    }
                } else {
                    PickTarget::Edge(edge_id)
                }
            } else {
                PickTarget::Edge(edge_id)
            };
            return Some(PickHit {
                target,
                point_mm: edge_hit.hit.point_on_segment,
                ray_t: edge_hit.hit.ray_t,
            });
        }

        if let Some((face_id, ray_t)) = best_filtered_face {
            return Some(PickHit {
                target: PickTarget::Face(face_id),
                point_mm: ray.at(ray_t),
                ray_t,
            });
        }

        None
    }

    /// Casts a ray against both B-Rep faces and triangle meshes (`render_mesh.pick_triangles`)
    /// in `O(log N)` time and returns `(hit_point_mm, outward_normal)` for the closest surface
    /// intersection.
    ///
    /// For planar B-Rep faces, the exact analytic plane normal is returned; for mesh triangles
    /// (STL/OBJ/3MF) or curved faces, the triangle geometric normal is returned.
    pub fn pick_surface_normal(&self, doc: &CadDocument, ray: &Ray3) -> Option<(DVec3, DVec3)> {
        let hit = self.tri_mesh.raycast(ray.origin, ray.dir, f64::INFINITY)?;
        let face_id = FaceId(hit.face_id);
        let normal = if let Some(face) = doc.face(face_id) {
            match face.surface {
                SurfaceKind::Plane { normal, .. } => normal.normalize_or_zero(),
                _ => hit.normal,
            }
        } else {
            hit.normal
        };
        if normal.length_squared() < 1e-12 {
            return None;
        }
        Some((hit.point, normal))
    }
}
