use chip_bvh::{FlatBvh, ray_triangle_intersect};
use chip_math::{Aabb3, DVec3, Ray3};

use crate::mesh::MeshTriangle;

/// Exact `f64` ray-triangle intersection result against a [`MeshBvh`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeshRayHit {
    pub triangle_index: usize,
    pub point_mm: DVec3,
    pub normal: DVec3,
    pub ray_t: f64,
}

/// Flat binned-SAH BVH spatial acceleration structure over canonical `f64` [`MeshTriangle`]s.
///
/// Backed by [`chip_bvh::FlatBvh`] for true `O(log N)` XY cylinder queries (Drop-Cutter),
/// Z-slab queries (Push-Cutter / Waterline slicing), AABB queries, and front-to-back exact
/// `f64` Möller–Trumbore raycasting.
#[derive(Debug, Clone, Default)]
pub struct MeshBvh {
    bvh: FlatBvh,
}

impl MeshBvh {
    /// Builds a flat binned-SAH BVH spatial index from `triangles`.
    pub fn build(triangles: &[MeshTriangle]) -> Self {
        let aabbs: Vec<Aabb3> = triangles.iter().map(|t| t.aabb).collect();
        Self {
            bvh: FlatBvh::build(&aabbs),
        }
    }

    /// Returns the underlying GPU-compatible [`FlatBvh`].
    #[inline]
    pub fn flat_bvh(&self) -> &FlatBvh {
        &self.bvh
    }

    #[inline]
    pub fn bounds(&self) -> Aabb3 {
        self.bvh.bounds()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.bvh.is_empty()
    }

    /// Returns candidate triangle indices whose XY bounding box overlaps the circle `(cx, cy, radius)`.
    pub fn query_xy_circle(&self, cx: f64, cy: f64, radius: f64) -> Vec<usize> {
        self.bvh.query_xy_circle(cx, cy, radius)
    }

    /// Appends candidate triangle indices overlapping the vertical cylinder `(cx, cy, radius)` into `out`.
    pub fn query_xy_circle_into(&self, cx: f64, cy: f64, radius: f64, out: &mut Vec<usize>) {
        self.bvh.query_xy_circle_into(cx, cy, radius, out);
    }

    /// Returns candidate triangle indices overlapping the horizontal Z slab `[z_min, z_max]`.
    pub fn query_z_slab(&self, z_min: f64, z_max: f64) -> Vec<usize> {
        self.bvh.query_z_slab(z_min, z_max)
    }

    /// Appends candidate triangle indices overlapping the horizontal Z slab `[z_min, z_max]` into `out`.
    pub fn query_z_slab_into(&self, z_min: f64, z_max: f64, out: &mut Vec<usize>) {
        self.bvh.query_z_slab_into(z_min, z_max, out);
    }

    /// Returns candidate triangle indices intersecting `aabb`.
    pub fn query_aabb(&self, aabb: &Aabb3) -> Vec<usize> {
        self.bvh.query_aabb(aabb)
    }

    /// Casts an exact `f64` ray against the indexed triangles in `O(log N)` front-to-back order,
    /// returning the closest forward hit.
    pub fn raycast(
        &self,
        vertices_mm: &[DVec3],
        triangles: &[MeshTriangle],
        ray: &Ray3,
    ) -> Option<MeshRayHit> {
        self.bvh
            .traverse_ray_closest(ray.origin, ray.dir, f64::INFINITY, |tri_idx, best_t| {
                let tri = triangles.get(tri_idx)?;
                let [i0, i1, i2] = tri.indices;
                let v0 = *vertices_mm.get(i0 as usize)?;
                let v1 = *vertices_mm.get(i1 as usize)?;
                let v2 = *vertices_mm.get(i2 as usize)?;
                let (t, _, _) = ray_triangle_intersect(ray.origin, ray.dir, v0, v1, v2, best_t)?;
                Some((
                    t,
                    MeshRayHit {
                        triangle_index: tri_idx,
                        point_mm: ray.at(t),
                        normal: tri.normal,
                        ray_t: t,
                    },
                ))
            })
            .map(|(_, hit)| hit)
    }
}
