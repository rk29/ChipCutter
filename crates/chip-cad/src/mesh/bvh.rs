use chip_math::{Aabb3, DVec3, Ray3};
use rstar::{AABB, RTree, RTreeObject};

use crate::mesh::MeshTriangle;

/// Exact `f64` ray-triangle intersection result against a [`MeshBvh`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeshRayHit {
    pub triangle_index: usize,
    pub point_mm: DVec3,
    pub normal: DVec3,
    pub ray_t: f64,
}

#[derive(Debug, Clone, Copy)]
struct MeshTriEntry {
    tri_index: usize,
    envelope: AABB<[f64; 3]>,
}

impl RTreeObject for MeshTriEntry {
    type Envelope = AABB<[f64; 3]>;

    #[inline]
    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}

/// R-tree / BVH spatial acceleration structure over canonical `f64` [`MeshTriangle`]s.
///
/// Supports sub-millisecond XY cylinder queries (Drop-Cutter), Z-slab queries (Push-Cutter /
/// Waterline slicing), AABB queries, and exact Möller–Trumbore raycasting.
#[derive(Debug, Clone)]
pub struct MeshBvh {
    tree: RTree<MeshTriEntry>,
    bounds: Aabb3,
}

impl Default for MeshBvh {
    fn default() -> Self {
        Self {
            tree: RTree::new(),
            bounds: Aabb3::EMPTY,
        }
    }
}

impl MeshBvh {
    /// Bulk-loads an R-tree spatial index from `triangles`.
    pub fn build(triangles: &[MeshTriangle]) -> Self {
        let mut bounds = Aabb3::EMPTY;
        let entries: Vec<MeshTriEntry> = triangles
            .iter()
            .enumerate()
            .filter(|(_, t)| t.aabb.is_valid())
            .map(|(tri_index, t)| {
                bounds = bounds.union(&t.aabb);
                MeshTriEntry {
                    tri_index,
                    envelope: AABB::from_corners(
                        [t.aabb.min.x, t.aabb.min.y, t.aabb.min.z],
                        [t.aabb.max.x, t.aabb.max.y, t.aabb.max.z],
                    ),
                }
            })
            .collect();
        Self {
            tree: RTree::bulk_load(entries),
            bounds,
        }
    }

    #[inline]
    pub fn bounds(&self) -> Aabb3 {
        self.bounds
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.tree.size() == 0
    }

    /// Returns candidate triangle indices whose XY bounding box overlaps the circle `(cx, cy, radius)`.
    pub fn query_xy_circle(&self, cx: f64, cy: f64, radius: f64) -> Vec<usize> {
        let mut out = Vec::new();
        self.query_xy_circle_into(cx, cy, radius, &mut out);
        out
    }

    /// Appends candidate triangle indices overlapping the vertical cylinder `(cx, cy, radius)` into `out`.
    pub fn query_xy_circle_into(&self, cx: f64, cy: f64, radius: f64, out: &mut Vec<usize>) {
        out.clear();
        if !self.bounds.is_valid() || !cx.is_finite() || !cy.is_finite() || !radius.is_finite() {
            return;
        }
        let r = radius.max(0.0);
        let env = AABB::from_corners(
            [cx - r, cy - r, self.bounds.min.z - 1.0],
            [cx + r, cy + r, self.bounds.max.z + 1.0],
        );
        for entry in self.tree.locate_in_envelope_intersecting(&env) {
            out.push(entry.tri_index);
        }
    }

    /// Returns candidate triangle indices overlapping the horizontal Z slab `[z_min, z_max]`.
    pub fn query_z_slab(&self, z_min: f64, z_max: f64) -> Vec<usize> {
        if !self.bounds.is_valid() || !z_min.is_finite() || !z_max.is_finite() {
            return Vec::new();
        }
        let lo = z_min.min(z_max);
        let hi = z_min.max(z_max);
        let env = AABB::from_corners(
            [self.bounds.min.x - 1.0, self.bounds.min.y - 1.0, lo],
            [self.bounds.max.x + 1.0, self.bounds.max.y + 1.0, hi],
        );
        self.tree
            .locate_in_envelope_intersecting(&env)
            .map(|e| e.tri_index)
            .collect()
    }

    /// Returns candidate triangle indices intersecting `aabb`.
    pub fn query_aabb(&self, aabb: &Aabb3) -> Vec<usize> {
        if !self.bounds.is_valid() || !aabb.is_valid() {
            return Vec::new();
        }
        let env = AABB::from_corners(
            [aabb.min.x, aabb.min.y, aabb.min.z],
            [aabb.max.x, aabb.max.y, aabb.max.z],
        );
        self.tree
            .locate_in_envelope_intersecting(&env)
            .map(|e| e.tri_index)
            .collect()
    }

    /// Casts an exact `f64` ray against the indexed triangles, returning the closest forward hit.
    pub fn raycast(
        &self,
        vertices_mm: &[DVec3],
        triangles: &[MeshTriangle],
        ray: &Ray3,
    ) -> Option<MeshRayHit> {
        if !self.bounds.is_valid() {
            return None;
        }
        ray.intersect_aabb(&self.bounds, 1e-4)?;

        let mut best: Option<MeshRayHit> = None;
        for entry in self.tree.iter() {
            let tri = &triangles[entry.tri_index];
            let Some(box_t) = ray.intersect_aabb(&tri.aabb, 1e-4) else {
                continue;
            };
            if let Some(cur) = best
                && box_t > cur.ray_t
            {
                continue;
            }
            let [i0, i1, i2] = tri.indices;
            let v0 = vertices_mm[i0 as usize];
            let v1 = vertices_mm[i1 as usize];
            let v2 = vertices_mm[i2 as usize];
            if let Some(t) = ray.intersect_triangle(v0, v1, v2)
                && best.is_none_or(|cur| t < cur.ray_t)
            {
                best = Some(MeshRayHit {
                    triangle_index: entry.tri_index,
                    point_mm: ray.at(t),
                    normal: tri.normal,
                    ray_t: t,
                });
            }
        }
        best
    }
}
