//! Canonical `f64` 3D triangle mesh (`TriMesh3D`) accelerated by a binned-SAH [`FlatBvh`].

use chip_math::{Aabb3, DVec3};
use rayon::prelude::*;

use crate::bvh::{
    ClosestPointHit, FlatBvh, FlatBvhNode, GpuTriangle, RayHit, aabb_overlaps_f64,
    closest_point_on_triangle, is_finite_valid_aabb, ray_triangle_intersect,
    xy_circle_intersects_aabb_f64, xy_circle_intersects_node, z_slab_overlaps_node,
};

/// Single canonical `f64` triangle stored inside a [`TriMesh3D`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tri3D {
    pub indices: [u32; 3],
    pub face_id: u32,
    pub normal: DVec3,
    pub aabb: Aabb3,
}

/// Exact `f64` 3D triangle mesh paired with a GPU-compatible flat binned-SAH [`FlatBvh`].
///
/// CPU queries (`raycast`, `raycast_all_hits`, `closest_point`, `query_xy_circle_into`,
/// `query_z_slab_into`) operate on exact `f64` vertex coordinates, while [`Self::nodes`] and
/// [`Self::gpu_triangles`] expose zero-padding `bytemuck::Pod` buffers for GPU compute shaders.
#[derive(Debug, Clone)]
pub struct TriMesh3D {
    pub vertices: Vec<DVec3>,
    pub triangles: Vec<Tri3D>,
    pub bounds: Aabb3,
    pub bvh: FlatBvh,
}

impl Default for TriMesh3D {
    fn default() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
            bounds: Aabb3::EMPTY,
            bvh: FlatBvh::default(),
        }
    }
}

impl AsRef<TriMesh3D> for TriMesh3D {
    #[inline]
    fn as_ref(&self) -> &Self {
        self
    }
}

impl TriMesh3D {
    /// Builds a [`TriMesh3D`] from indexed vertices and triangle index triples (`face_id = 0`).
    pub fn from_indexed(vertices: Vec<DVec3>, indices: &[[u32; 3]]) -> Self {
        let mut triangles = Vec::with_capacity(indices.len());
        let mut prim_aabbs = Vec::with_capacity(indices.len());
        let mut bounds = Aabb3::EMPTY;

        for &[i0, i1, i2] in indices {
            let (Some(&v0), Some(&v1), Some(&v2)) = (
                vertices.get(i0 as usize),
                vertices.get(i1 as usize),
                vertices.get(i2 as usize),
            ) else {
                continue;
            };
            if !v0.is_finite() || !v1.is_finite() || !v2.is_finite() {
                continue;
            }
            let normal = (v1 - v0).cross(v2 - v0).normalize_or_zero();
            let aabb = Aabb3::new(v0.min(v1).min(v2), v0.max(v1).max(v2));
            bounds = bounds.union(&aabb);
            prim_aabbs.push(aabb);
            triangles.push(Tri3D {
                indices: [i0, i1, i2],
                face_id: 0,
                normal,
                aabb,
            });
        }

        let bvh = FlatBvh::build(&prim_aabbs);
        Self {
            vertices,
            triangles,
            bounds,
            bvh,
        }
    }

    /// Builds a [`TriMesh3D`] from indexed vertices and `(indices, face_id)` pairs.
    pub fn from_indexed_with_faces(
        vertices: Vec<DVec3>,
        indexed_faces: &[([u32; 3], u32)],
    ) -> Self {
        let mut triangles = Vec::with_capacity(indexed_faces.len());
        let mut prim_aabbs = Vec::with_capacity(indexed_faces.len());
        let mut bounds = Aabb3::EMPTY;

        for &([i0, i1, i2], face_id) in indexed_faces {
            let (Some(&v0), Some(&v1), Some(&v2)) = (
                vertices.get(i0 as usize),
                vertices.get(i1 as usize),
                vertices.get(i2 as usize),
            ) else {
                continue;
            };
            if !v0.is_finite() || !v1.is_finite() || !v2.is_finite() {
                continue;
            }
            let normal = (v1 - v0).cross(v2 - v0).normalize_or_zero();
            let aabb = Aabb3::new(v0.min(v1).min(v2), v0.max(v1).max(v2));
            bounds = bounds.union(&aabb);
            prim_aabbs.push(aabb);
            triangles.push(Tri3D {
                indices: [i0, i1, i2],
                face_id,
                normal,
                aabb,
            });
        }

        let bvh = FlatBvh::build(&prim_aabbs);
        Self {
            vertices,
            triangles,
            bounds,
            bvh,
        }
    }

    /// Builds a [`TriMesh3D`] from unindexed raw triangle triples (`face_id = 0`).
    pub fn from_raw_triangles(raw_tris: &[[DVec3; 3]]) -> Self {
        let mut vertices = Vec::with_capacity(raw_tris.len() * 3);
        let mut triangles = Vec::with_capacity(raw_tris.len());
        let mut prim_aabbs = Vec::with_capacity(raw_tris.len());
        let mut bounds = Aabb3::EMPTY;

        for &[v0, v1, v2] in raw_tris {
            if !v0.is_finite() || !v1.is_finite() || !v2.is_finite() {
                continue;
            }
            let Ok(base) = u32::try_from(vertices.len()) else {
                break;
            };
            vertices.push(v0);
            vertices.push(v1);
            vertices.push(v2);
            let normal = (v1 - v0).cross(v2 - v0).normalize_or_zero();
            let aabb = Aabb3::new(v0.min(v1).min(v2), v0.max(v1).max(v2));
            bounds = bounds.union(&aabb);
            prim_aabbs.push(aabb);
            triangles.push(Tri3D {
                indices: [base, base + 1, base + 2],
                face_id: 0,
                normal,
                aabb,
            });
        }

        let bvh = FlatBvh::build(&prim_aabbs);
        Self {
            vertices,
            triangles,
            bounds,
            bvh,
        }
    }

    /// Builds a [`TriMesh3D`] from unindexed `(triangle, face_id)` pairs.
    pub fn from_raw_triangles_with_faces(raw_tris: &[([DVec3; 3], u32)]) -> Self {
        let mut vertices = Vec::with_capacity(raw_tris.len() * 3);
        let mut triangles = Vec::with_capacity(raw_tris.len());
        let mut prim_aabbs = Vec::with_capacity(raw_tris.len());
        let mut bounds = Aabb3::EMPTY;

        for &([v0, v1, v2], face_id) in raw_tris {
            if !v0.is_finite() || !v1.is_finite() || !v2.is_finite() {
                continue;
            }
            let Ok(base) = u32::try_from(vertices.len()) else {
                break;
            };
            vertices.push(v0);
            vertices.push(v1);
            vertices.push(v2);
            let normal = (v1 - v0).cross(v2 - v0).normalize_or_zero();
            let aabb = Aabb3::new(v0.min(v1).min(v2), v0.max(v1).max(v2));
            bounds = bounds.union(&aabb);
            prim_aabbs.push(aabb);
            triangles.push(Tri3D {
                indices: [base, base + 1, base + 2],
                face_id,
                normal,
                aabb,
            });
        }

        let bvh = FlatBvh::build(&prim_aabbs);
        Self {
            vertices,
            triangles,
            bounds,
            bvh,
        }
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.triangles.len()
    }

    #[inline]
    pub fn bounds(&self) -> Aabb3 {
        self.bounds
    }

    /// Returns the flat 32-byte GPU-compatible BVH node array.
    #[inline]
    pub fn nodes(&self) -> &[FlatBvhNode] {
        self.bvh.nodes()
    }

    /// Returns the BVH leaf primitive index array mapping leaf slots to `self.triangles`.
    #[inline]
    pub fn prim_indices(&self) -> &[u32] {
        self.bvh.prim_indices()
    }

    /// Packs triangles in BVH `prim_indices` order into 48-byte [`GpuTriangle`] records ready
    /// for direct WGSL storage buffer upload (`gpu_triangles[node.left_or_first + i]`).
    pub fn gpu_triangles(&self) -> Vec<GpuTriangle> {
        let mut out = Vec::with_capacity(self.bvh.prim_indices().len());
        for &tri_idx_u32 in self.bvh.prim_indices() {
            let Some(tri) = self.triangles.get(tri_idx_u32 as usize) else {
                continue;
            };
            let (Some(&v0), Some(&v1), Some(&v2)) = (
                self.vertices.get(tri.indices[0] as usize),
                self.vertices.get(tri.indices[1] as usize),
                self.vertices.get(tri.indices[2] as usize),
            ) else {
                continue;
            };
            out.push(GpuTriangle {
                v0: [v0.x as f32, v0.y as f32, v0.z as f32],
                face_id: tri.face_id,
                v1: [v1.x as f32, v1.y as f32, v1.z as f32],
                flags: 0,
                v2: [v2.x as f32, v2.y as f32, v2.z as f32],
                prim_id: tri_idx_u32,
            });
        }
        out
    }

    /// Returns triangle vertices `(v0, v1, v2)` for triangle `tri_index`.
    #[inline]
    pub fn triangle_vertices(&self, tri_index: usize) -> Option<(DVec3, DVec3, DVec3)> {
        let tri = self.triangles.get(tri_index)?;
        let v0 = *self.vertices.get(tri.indices[0] as usize)?;
        let v1 = *self.vertices.get(tri.indices[1] as usize)?;
        let v2 = *self.vertices.get(tri.indices[2] as usize)?;
        Some((v0, v1, v2))
    }

    /// Casts an exact `f64` ray `(origin, dir)` up to `max_t`, returning the closest hit in
    /// `O(log N)` time with front-to-back ordered child traversal.
    pub fn raycast(&self, origin: DVec3, dir: DVec3, max_t: f64) -> Option<RayHit> {
        self.bvh
            .traverse_ray_closest(origin, dir, max_t, |tri_idx, best_t| {
                let tri = self.triangles.get(tri_idx)?;
                let v0 = *self.vertices.get(tri.indices[0] as usize)?;
                let v1 = *self.vertices.get(tri.indices[1] as usize)?;
                let v2 = *self.vertices.get(tri.indices[2] as usize)?;
                let (t, u, v) = ray_triangle_intersect(origin, dir, v0, v1, v2, best_t)?;
                Some((
                    t,
                    RayHit {
                        tri_index: tri_idx,
                        face_id: tri.face_id,
                        t,
                        point: origin + dir * t,
                        normal: tri.normal,
                        bary_u: u,
                        bary_v: v,
                    },
                ))
            })
            .map(|(_, hit)| hit)
    }

    /// Finds all exact `f64` intersections along `(origin, dir)` within `[0.0, max_t]`, writing
    /// hits sorted by ascending `t` into `out` (suitable for even-odd Tri-Dexel voxelization).
    pub fn raycast_all_hits(&self, origin: DVec3, dir: DVec3, max_t: f64, out: &mut Vec<RayHit>) {
        out.clear();
        self.bvh.traverse_ray_all(origin, dir, max_t, |tri_idx| {
            let Some(tri) = self.triangles.get(tri_idx) else {
                return;
            };
            let (Some(&v0), Some(&v1), Some(&v2)) = (
                self.vertices.get(tri.indices[0] as usize),
                self.vertices.get(tri.indices[1] as usize),
                self.vertices.get(tri.indices[2] as usize),
            ) else {
                return;
            };
            if let Some((t, u, v)) = ray_triangle_intersect(origin, dir, v0, v1, v2, max_t) {
                out.push(RayHit {
                    tri_index: tri_idx,
                    face_id: tri.face_id,
                    t,
                    point: origin + dir * t,
                    normal: tri.normal,
                    bary_u: u,
                    bary_v: v,
                });
            }
        });
        out.sort_by(|a, b| {
            a.t.total_cmp(&b.t)
                .then_with(|| a.tri_index.cmp(&b.tri_index))
        });
    }

    /// Finds the exact `f64` closest point on the mesh to `point` in `O(log N)` time using
    /// nearest-child priority AABB pruning.
    pub fn closest_point(&self, point: DVec3) -> Option<ClosestPointHit> {
        self.bvh
            .traverse_closest_point(point, |tri_idx, _best_d2| {
                let tri = self.triangles.get(tri_idx)?;
                let v0 = *self.vertices.get(tri.indices[0] as usize)?;
                let v1 = *self.vertices.get(tri.indices[1] as usize)?;
                let v2 = *self.vertices.get(tri.indices[2] as usize)?;
                let (q, d2, u, v) = closest_point_on_triangle(point, v0, v1, v2);
                Some((
                    d2,
                    ClosestPointHit {
                        tri_index: tri_idx,
                        face_id: tri.face_id,
                        point: q,
                        normal: tri.normal,
                        distance: d2.sqrt(),
                        bary_u: u,
                        bary_v: v,
                    },
                ))
            })
            .map(|(_, hit)| hit)
    }

    /// Appends all triangle indices whose exact `f64` XY AABB overlaps the circle `(cx, cy, radius)`
    /// into `out` (Drop-Cutter spatial filter).
    pub fn query_xy_circle_into(&self, cx: f64, cy: f64, radius: f64, out: &mut Vec<usize>) {
        out.clear();
        let nodes = self.bvh.nodes();
        if nodes.is_empty() || !cx.is_finite() || !cy.is_finite() || !radius.is_finite() {
            return;
        }
        let r = radius.max(0.0);
        let r_sq = r * r;
        if !xy_circle_intersects_node(&nodes[0], cx, cy, r_sq) {
            return;
        }

        let prims = self.bvh.prim_indices();
        let mut stack = [0u32; 64];
        let mut sp = 1usize;
        stack[0] = 0;

        while sp > 0 {
            sp -= 1;
            let node = &nodes[stack[sp] as usize];
            if node.is_leaf() {
                let first = node.left_or_first as usize;
                let end = first + node.tri_count as usize;
                for &pi in &prims[first..end] {
                    let ti = pi as usize;
                    if let Some(tri) = self.triangles.get(ti)
                        && xy_circle_intersects_aabb_f64(&tri.aabb, cx, cy, r_sq)
                    {
                        out.push(ti);
                    }
                }
            } else {
                let left_idx = node.left_or_first;
                let right_idx = left_idx + 1;
                if xy_circle_intersects_node(&nodes[right_idx as usize], cx, cy, r_sq)
                    && sp < stack.len()
                {
                    stack[sp] = right_idx;
                    sp += 1;
                }
                if xy_circle_intersects_node(&nodes[left_idx as usize], cx, cy, r_sq)
                    && sp < stack.len()
                {
                    stack[sp] = left_idx;
                    sp += 1;
                }
            }
        }
    }

    /// Appends all triangle indices whose exact `f64` Z interval overlaps `[z_min, z_max]` into
    /// `out` (Push-Cutter / Waterline spatial filter).
    pub fn query_z_slab_into(&self, z_min: f64, z_max: f64, out: &mut Vec<usize>) {
        out.clear();
        let nodes = self.bvh.nodes();
        if nodes.is_empty() || !z_min.is_finite() || !z_max.is_finite() {
            return;
        }
        let lo = z_min.min(z_max);
        let hi = z_min.max(z_max);
        if !z_slab_overlaps_node(&nodes[0], lo, hi) {
            return;
        }

        let prims = self.bvh.prim_indices();
        let mut stack = [0u32; 64];
        let mut sp = 1usize;
        stack[0] = 0;

        while sp > 0 {
            sp -= 1;
            let node = &nodes[stack[sp] as usize];
            if node.is_leaf() {
                let first = node.left_or_first as usize;
                let end = first + node.tri_count as usize;
                for &pi in &prims[first..end] {
                    let ti = pi as usize;
                    if let Some(tri) = self.triangles.get(ti)
                        && tri.aabb.min.z <= hi
                        && tri.aabb.max.z >= lo
                    {
                        out.push(ti);
                    }
                }
            } else {
                let left_idx = node.left_or_first;
                let right_idx = left_idx + 1;
                if z_slab_overlaps_node(&nodes[right_idx as usize], lo, hi) && sp < stack.len() {
                    stack[sp] = right_idx;
                    sp += 1;
                }
                if z_slab_overlaps_node(&nodes[left_idx as usize], lo, hi) && sp < stack.len() {
                    stack[sp] = left_idx;
                    sp += 1;
                }
            }
        }
    }

    /// Appends all triangle indices whose exact `f64` AABB intersects `aabb` into `out`.
    pub fn query_aabb_into(&self, aabb: &Aabb3, out: &mut Vec<usize>) {
        out.clear();
        if self.bvh.is_empty() || !is_finite_valid_aabb(aabb) {
            return;
        }
        let mut candidates = Vec::new();
        self.bvh.query_aabb_into(aabb, &mut candidates);
        for ti in candidates {
            if let Some(tri) = self.triangles.get(ti)
                && aabb_overlaps_f64(&tri.aabb, aabb)
            {
                out.push(ti);
            }
        }
    }

    /// Evaluates a batch of `(origin, dir, max_t)` raycasts in parallel using `rayon`.
    pub fn raycast_batch_par(&self, rays: &[(DVec3, DVec3, f64)]) -> Vec<Option<RayHit>> {
        rays.par_iter()
            .map(|&(origin, dir, max_t)| self.raycast(origin, dir, max_t))
            .collect()
    }

    /// Evaluates a batch of closest-point queries in parallel using `rayon`.
    pub fn closest_points_batch_par(&self, points: &[DVec3]) -> Vec<Option<ClosestPointHit>> {
        points
            .par_iter()
            .map(|&pt| self.closest_point(pt))
            .collect()
    }
}
