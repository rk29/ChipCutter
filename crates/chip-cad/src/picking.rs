use chip_math::{DVec3, Ray3};
use rstar::{AABB, RTree, RTreeObject};

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

#[derive(Debug, Clone)]
struct TriEntry {
    face_id: FaceId,
    v0: DVec3,
    v1: DVec3,
    v2: DVec3,
    envelope: AABB<[f64; 3]>,
}

impl RTreeObject for TriEntry {
    type Envelope = AABB<[f64; 3]>;
    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}

#[derive(Debug, Clone)]
struct SegEntry {
    edge_id: EdgeId,
    a: DVec3,
    b: DVec3,
    envelope: AABB<[f64; 3]>,
}

impl RTreeObject for SegEntry {
    type Envelope = AABB<[f64; 3]>;
    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}

/// R-tree spatial acceleration structure for sub-millisecond hover and click picking.
#[derive(Debug, Clone, Default)]
pub struct CadPicker {
    tri_tree: RTree<TriEntry>,
    seg_tree: RTree<SegEntry>,
}

impl CadPicker {
    /// Builds spatial pick trees from an imported [`CadDocument`].
    pub fn build(doc: &CadDocument) -> Self {
        let tri_entries: Vec<TriEntry> = doc
            .render_mesh
            .pick_triangles
            .iter()
            .map(|tri| {
                let min = tri.v0_mm.min(tri.v1_mm).min(tri.v2_mm);
                let max = tri.v0_mm.max(tri.v1_mm).max(tri.v2_mm);
                TriEntry {
                    face_id: tri.face_id,
                    v0: tri.v0_mm,
                    v1: tri.v1_mm,
                    v2: tri.v2_mm,
                    envelope: AABB::from_corners([min.x, min.y, min.z], [max.x, max.y, max.z]),
                }
            })
            .collect();

        let mut seg_entries = Vec::new();
        for poly in &doc.render_mesh.edge_polylines {
            for w in poly.points_mm.windows(2) {
                let a = w[0];
                let b = w[1];
                let min = a.min(b);
                let max = a.max(b);
                seg_entries.push(SegEntry {
                    edge_id: poly.edge_id,
                    a,
                    b,
                    envelope: AABB::from_corners([min.x, min.y, min.z], [max.x, max.y, max.z]),
                });
            }
        }

        Self {
            tri_tree: RTree::bulk_load(tri_entries),
            seg_tree: RTree::bulk_load(seg_entries),
        }
    }

    /// Casts a 3D picking ray (`f64` mm) against the B-Rep model.
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

        for tri in self.tri_tree.iter() {
            let min = DVec3::from(tri.envelope.lower());
            let max = DVec3::from(tri.envelope.upper());
            let aabb = chip_math::Aabb3::new(min, max);
            let Some(box_t) = ray.intersect_aabb(&aabb, 1e-4) else {
                continue;
            };
            if box_t > closest_any_face_t && best_filtered_face.is_some_and(|(_, ft)| box_t > ft) {
                continue;
            }
            if let Some(t) = ray.intersect_triangle(tri.v0, tri.v1, tri.v2) {
                if t < closest_any_face_t {
                    closest_any_face_t = t;
                }
                let matches_face_filter = match filter {
                    PickFilter::Any | PickFilter::FacesOnly => doc.face(tri.face_id).is_some(),
                    PickFilter::CylindricalHolesOnly => doc.face(tri.face_id).is_some_and(|f| {
                        matches!(
                            f.surface,
                            SurfaceKind::Cylinder {
                                is_internal: true,
                                ..
                            }
                        )
                    }),
                    PickFilter::EdgesOnly | PickFilter::EdgeLoops => false,
                };
                if matches_face_filter && best_filtered_face.is_none_or(|(_, best_t)| t < best_t) {
                    best_filtered_face = Some((tri.face_id, t));
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

        let mut best_edge: Option<(EdgeId, f64, f64, DVec3)> = None; // (id, score, ray_t, pt)
        if check_edges {
            for seg in self.seg_tree.iter() {
                let min = DVec3::from(seg.envelope.lower());
                let max = DVec3::from(seg.envelope.upper());
                let aabb = chip_math::Aabb3::new(min, max);
                let Some(box_t) = ray.intersect_aabb(&aabb, edge_radius_mm) else {
                    continue;
                };
                if box_t > occlusion_limit {
                    continue;
                }
                let hit = ray.closest_to_segment(seg.a, seg.b);
                if hit.distance_mm <= edge_radius_mm && hit.ray_t <= occlusion_limit {
                    // Prefer closer screen-space hit + shallower ray depth
                    let score = hit.distance_mm + hit.ray_t * 1e-3;
                    if best_edge.is_none_or(|(_, best_s, _, _)| score < best_s) {
                        best_edge = Some((seg.edge_id, score, hit.ray_t, hit.point_on_segment));
                    }
                }
            }
        }

        if let Some((edge_id, _, ray_t, point_mm)) = best_edge {
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
                point_mm,
                ray_t,
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
    /// and returns `(hit_point_mm, outward_normal)` for the closest surface intersection.
    ///
    /// For planar B-Rep faces, the exact analytic plane normal is returned; for mesh triangles
    /// (STL/OBJ/3MF) or curved faces, the triangle geometric normal is returned.
    pub fn pick_surface_normal(&self, doc: &CadDocument, ray: &Ray3) -> Option<(DVec3, DVec3)> {
        let mut best: Option<(&TriEntry, f64)> = None;
        for tri in self.tri_tree.iter() {
            let min = DVec3::from(tri.envelope.lower());
            let max = DVec3::from(tri.envelope.upper());
            let aabb = chip_math::Aabb3::new(min, max);
            let Some(box_t) = ray.intersect_aabb(&aabb, 1e-4) else {
                continue;
            };
            if best.is_some_and(|(_, best_t)| box_t > best_t) {
                continue;
            }
            if let Some(t) = ray.intersect_triangle(tri.v0, tri.v1, tri.v2)
                && best.is_none_or(|(_, best_t)| t < best_t)
            {
                best = Some((tri, t));
            }
        }
        let (tri, ray_t) = best?;
        let hit_pt = ray.at(ray_t);
        let normal = if let Some(face) = doc.face(tri.face_id) {
            match face.surface {
                SurfaceKind::Plane { normal, .. } => normal.normalize_or_zero(),
                _ => (tri.v1 - tri.v0).cross(tri.v2 - tri.v0).normalize_or_zero(),
            }
        } else {
            (tri.v1 - tri.v0).cross(tri.v2 - tri.v0).normalize_or_zero()
        };
        if normal.length_squared() < 1e-12 {
            return None;
        }
        Some((hit_pt, normal))
    }
}
