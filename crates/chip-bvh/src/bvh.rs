//! GPU-compatible 32-byte flat binary BVH (`FlatBvhNode`, `GpuTriangle`, `FlatBvh`)
//! with binned-SAH construction and exact `f64` leaf primitive primitives.

use chip_math::{Aabb3, DVec3};

/// Maximum number of primitives in a leaf node before forcing a split.
const MAX_LEAF_PRIMS: usize = 4;
/// Maximum number of primitives allowed in an SAH-rejected leaf.
const MAX_SAH_LEAF_PRIMS: usize = 16;
/// Maximum tree depth (guarantees a 64-entry traversal stack can never overflow).
const MAX_TREE_DEPTH: usize = 60;
/// Number of spatial SAH bins per axis during construction.
const NUM_SAH_BINS: usize = 12;

/// GPU-uploadable 32-byte flat binary BVH node (`#[repr(C)]`, zero padding).
///
/// Matches the standard WGSL storage-buffer layout:
/// - `tri_count == 0`: internal node with children `left = left_or_first` and `right = left + 1`.
/// - `tri_count > 0`: leaf node referencing `prim_indices[left_or_first .. left_or_first + tri_count]`.
///
/// The `f32` AABB bounds are conservatively rounded outward from canonical `f64` bounds so slab
/// pruning never misses an exact `f64` intersection.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct FlatBvhNode {
    pub aabb_min: [f32; 3],
    pub left_or_first: u32,
    pub aabb_max: [f32; 3],
    pub tri_count: u32,
}

impl FlatBvhNode {
    #[inline]
    pub const fn is_leaf(&self) -> bool {
        self.tri_count > 0
    }
}

/// GPU-uploadable 48-byte flat triangle (`#[repr(C)]`, 16-byte aligned WGSL fields).
///
/// Ordered to match `FlatBvh::prim_indices()` when produced by [`crate::TriMesh3D::gpu_triangles`],
/// allowing WGSL compute shaders to index `gpu_triangles[node.left_or_first + i]` directly.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuTriangle {
    pub v0: [f32; 3],
    pub face_id: u32,
    pub v1: [f32; 3],
    pub flags: u32,
    pub v2: [f32; 3],
    pub prim_id: u32,
}

/// Exact `f64` ray-triangle intersection hit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHit {
    pub tri_index: usize,
    pub face_id: u32,
    pub t: f64,
    pub point: DVec3,
    pub normal: DVec3,
    pub bary_u: f64,
    pub bary_v: f64,
}

/// Exact `f64` closest-point query hit against a triangle mesh.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClosestPointHit {
    pub tri_index: usize,
    pub face_id: u32,
    pub point: DVec3,
    pub normal: DVec3,
    pub distance: f64,
    pub bary_u: f64,
    pub bary_v: f64,
}

/// Flat-array binned-SAH binary BVH over arbitrary indexed 3D primitives.
///
/// Internal nodes store conservatively padded `f32` bounds (`FlatBvhNode`, 32 bytes each) so the
/// node array can be uploaded directly to GPU storage buffers while CPU queries remain exact `f64`.
#[derive(Debug, Clone)]
pub struct FlatBvh {
    nodes: Vec<FlatBvhNode>,
    prim_indices: Vec<u32>,
    bounds: Aabb3,
}

impl Default for FlatBvh {
    fn default() -> Self {
        Self {
            nodes: Vec::new(),
            prim_indices: Vec::new(),
            bounds: Aabb3::EMPTY,
        }
    }
}

impl FlatBvh {
    /// Builds a flat binned-SAH binary BVH from per-primitive canonical `f64` AABBs.
    ///
    /// Invalid or non-finite AABBs are silently ignored.
    pub fn build(prim_aabbs: &[Aabb3]) -> Self {
        let mut prim_indices: Vec<u32> = Vec::with_capacity(prim_aabbs.len());
        let mut bounds = Aabb3::EMPTY;
        for (idx, aabb) in prim_aabbs.iter().enumerate() {
            if is_finite_valid_aabb(aabb)
                && let Ok(idx_u32) = u32::try_from(idx)
            {
                prim_indices.push(idx_u32);
                bounds = bounds.union(aabb);
            }
        }

        if prim_indices.is_empty() {
            return Self::default();
        }

        let centroids: Vec<DVec3> = prim_aabbs
            .iter()
            .map(|aabb| {
                if is_finite_valid_aabb(aabb) {
                    0.5 * (aabb.min + aabb.max)
                } else {
                    DVec3::ZERO
                }
            })
            .collect();

        let estimated_nodes = prim_indices.len().saturating_mul(2).max(1);
        let mut nodes = Vec::with_capacity(estimated_nodes);
        nodes.push(FlatBvhNode {
            aabb_min: [0.0; 3],
            left_or_first: 0,
            aabb_max: [0.0; 3],
            tri_count: 0,
        });

        let total = prim_indices.len();
        build_recursive(
            0,
            0,
            total,
            0,
            prim_aabbs,
            &centroids,
            &mut prim_indices,
            &mut nodes,
        );

        Self {
            nodes,
            prim_indices,
            bounds,
        }
    }

    #[inline]
    pub fn nodes(&self) -> &[FlatBvhNode] {
        &self.nodes
    }

    #[inline]
    pub fn prim_indices(&self) -> &[u32] {
        &self.prim_indices
    }

    #[inline]
    pub fn bounds(&self) -> Aabb3 {
        self.bounds
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.prim_indices.is_empty()
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.prim_indices.len()
    }

    /// Appends candidate primitive indices whose leaf AABBs intersect `aabb` into `out`.
    pub fn query_aabb_into(&self, aabb: &Aabb3, out: &mut Vec<usize>) {
        out.clear();
        if self.nodes.is_empty() || !is_finite_valid_aabb(aabb) {
            return;
        }
        if !aabb_overlaps_f32(&self.nodes[0], aabb) {
            return;
        }

        let mut stack = [0u32; 64];
        let mut sp = 1usize;
        stack[0] = 0;

        while sp > 0 {
            sp -= 1;
            let node = &self.nodes[stack[sp] as usize];
            if node.is_leaf() {
                let first = node.left_or_first as usize;
                let end = first + node.tri_count as usize;
                for &pi in &self.prim_indices[first..end] {
                    out.push(pi as usize);
                }
            } else {
                let left_idx = node.left_or_first;
                let right_idx = left_idx + 1;
                if aabb_overlaps_f32(&self.nodes[right_idx as usize], aabb) && sp < stack.len() {
                    stack[sp] = right_idx;
                    sp += 1;
                }
                if aabb_overlaps_f32(&self.nodes[left_idx as usize], aabb) && sp < stack.len() {
                    stack[sp] = left_idx;
                    sp += 1;
                }
            }
        }
    }

    /// Returns candidate primitive indices whose leaf AABBs intersect `aabb`.
    pub fn query_aabb(&self, aabb: &Aabb3) -> Vec<usize> {
        let mut out = Vec::new();
        self.query_aabb_into(aabb, &mut out);
        out
    }

    /// Appends candidate primitive indices whose leaf XY AABBs overlap the vertical cylinder
    /// centered at `(cx, cy)` with radius `radius` into `out`.
    pub fn query_xy_circle_into(&self, cx: f64, cy: f64, radius: f64, out: &mut Vec<usize>) {
        out.clear();
        if self.nodes.is_empty() || !cx.is_finite() || !cy.is_finite() || !radius.is_finite() {
            return;
        }
        let r = radius.max(0.0);
        let r_sq = r * r;
        if !xy_circle_intersects_node(&self.nodes[0], cx, cy, r_sq) {
            return;
        }

        let mut stack = [0u32; 64];
        let mut sp = 1usize;
        stack[0] = 0;

        while sp > 0 {
            sp -= 1;
            let node = &self.nodes[stack[sp] as usize];
            if node.is_leaf() {
                let first = node.left_or_first as usize;
                let end = first + node.tri_count as usize;
                for &pi in &self.prim_indices[first..end] {
                    out.push(pi as usize);
                }
            } else {
                let left_idx = node.left_or_first;
                let right_idx = left_idx + 1;
                if xy_circle_intersects_node(&self.nodes[right_idx as usize], cx, cy, r_sq)
                    && sp < stack.len()
                {
                    stack[sp] = right_idx;
                    sp += 1;
                }
                if xy_circle_intersects_node(&self.nodes[left_idx as usize], cx, cy, r_sq)
                    && sp < stack.len()
                {
                    stack[sp] = left_idx;
                    sp += 1;
                }
            }
        }
    }

    /// Returns candidate primitive indices whose leaf XY AABBs overlap `(cx, cy, radius)`.
    pub fn query_xy_circle(&self, cx: f64, cy: f64, radius: f64) -> Vec<usize> {
        let mut out = Vec::new();
        self.query_xy_circle_into(cx, cy, radius, &mut out);
        out
    }

    /// Appends candidate primitive indices whose leaf Z intervals overlap `[z_min, z_max]` into `out`.
    pub fn query_z_slab_into(&self, z_min: f64, z_max: f64, out: &mut Vec<usize>) {
        out.clear();
        if self.nodes.is_empty() || !z_min.is_finite() || !z_max.is_finite() {
            return;
        }
        let lo = z_min.min(z_max);
        let hi = z_min.max(z_max);
        if !z_slab_overlaps_node(&self.nodes[0], lo, hi) {
            return;
        }

        let mut stack = [0u32; 64];
        let mut sp = 1usize;
        stack[0] = 0;

        while sp > 0 {
            sp -= 1;
            let node = &self.nodes[stack[sp] as usize];
            if node.is_leaf() {
                let first = node.left_or_first as usize;
                let end = first + node.tri_count as usize;
                for &pi in &self.prim_indices[first..end] {
                    out.push(pi as usize);
                }
            } else {
                let left_idx = node.left_or_first;
                let right_idx = left_idx + 1;
                if z_slab_overlaps_node(&self.nodes[right_idx as usize], lo, hi) && sp < stack.len()
                {
                    stack[sp] = right_idx;
                    sp += 1;
                }
                if z_slab_overlaps_node(&self.nodes[left_idx as usize], lo, hi) && sp < stack.len()
                {
                    stack[sp] = left_idx;
                    sp += 1;
                }
            }
        }
        out.sort_unstable();
    }

    /// Returns candidate primitive indices whose leaf Z intervals overlap `[z_min, z_max]`.
    pub fn query_z_slab(&self, z_min: f64, z_max: f64) -> Vec<usize> {
        let mut out = Vec::new();
        self.query_z_slab_into(z_min, z_max, &mut out);
        out
    }

    /// Traverses the BVH front-to-back along `(origin, dir)` up to `max_t`, invoking
    /// `intersect_leaf(prim_index, current_best_t)` on leaf primitives and tightening `best_t`
    /// whenever a closer hit is found.
    pub fn traverse_ray_closest<T, F>(
        &self,
        origin: DVec3,
        dir: DVec3,
        max_t: f64,
        mut intersect_leaf: F,
    ) -> Option<(f64, T)>
    where
        F: FnMut(usize, f64) -> Option<(f64, T)>,
    {
        if self.nodes.is_empty()
            || !origin.is_finite()
            || !dir.is_finite()
            || max_t.is_nan()
            || max_t < 0.0
            || dir.length_squared() < 1e-24
        {
            return None;
        }

        let inv_dir = DVec3::new(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);
        let root = &self.nodes[0];
        let root_t = intersect_aabb_slab(origin, inv_dir, root.aabb_min, root.aabb_max, max_t)?;

        let mut best_t = max_t;
        let mut best_val: Option<T> = None;

        let mut stack: [(u32, f64); 64] = [(0, 0.0); 64];
        let mut sp = 1usize;
        stack[0] = (0, root_t);

        while sp > 0 {
            sp -= 1;
            let (node_idx, node_t) = stack[sp];
            if node_t > best_t {
                continue;
            }
            let node = &self.nodes[node_idx as usize];
            if node.is_leaf() {
                let first = node.left_or_first as usize;
                let end = first + node.tri_count as usize;
                for &pi in &self.prim_indices[first..end] {
                    if let Some((t, val)) = intersect_leaf(pi as usize, best_t)
                        && t >= 0.0
                        && t <= best_t
                        && (best_val.is_none() || t < best_t)
                    {
                        best_t = t;
                        best_val = Some(val);
                    }
                }
            } else {
                let left_idx = node.left_or_first;
                let right_idx = left_idx + 1;
                let left = &self.nodes[left_idx as usize];
                let right = &self.nodes[right_idx as usize];

                let t_left =
                    intersect_aabb_slab(origin, inv_dir, left.aabb_min, left.aabb_max, best_t);
                let t_right =
                    intersect_aabb_slab(origin, inv_dir, right.aabb_min, right.aabb_max, best_t);

                match (t_left, t_right) {
                    (Some(tl), Some(tr)) => {
                        if sp + 2 <= stack.len() {
                            if tl <= tr {
                                stack[sp] = (right_idx, tr);
                                stack[sp + 1] = (left_idx, tl);
                            } else {
                                stack[sp] = (left_idx, tl);
                                stack[sp + 1] = (right_idx, tr);
                            }
                            sp += 2;
                        }
                    }
                    (Some(tl), None) => {
                        if sp < stack.len() {
                            stack[sp] = (left_idx, tl);
                            sp += 1;
                        }
                    }
                    (None, Some(tr)) => {
                        if sp < stack.len() {
                            stack[sp] = (right_idx, tr);
                            sp += 1;
                        }
                    }
                    (None, None) => {}
                }
            }
        }

        best_val.map(|v| (best_t, v))
    }

    /// Traverses all leaves intersected by `(origin, dir)` within `[0.0, max_t]`, invoking
    /// `visit_leaf(prim_index)` for every candidate primitive.
    pub fn traverse_ray_all<F>(&self, origin: DVec3, dir: DVec3, max_t: f64, mut visit_leaf: F)
    where
        F: FnMut(usize),
    {
        if self.nodes.is_empty()
            || !origin.is_finite()
            || !dir.is_finite()
            || max_t.is_nan()
            || max_t < 0.0
            || dir.length_squared() < 1e-24
        {
            return;
        }

        let inv_dir = DVec3::new(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);
        let root = &self.nodes[0];
        if intersect_aabb_slab(origin, inv_dir, root.aabb_min, root.aabb_max, max_t).is_none() {
            return;
        }

        let mut stack = [0u32; 64];
        let mut sp = 1usize;
        stack[0] = 0;

        while sp > 0 {
            sp -= 1;
            let node = &self.nodes[stack[sp] as usize];
            if node.is_leaf() {
                let first = node.left_or_first as usize;
                let end = first + node.tri_count as usize;
                for &pi in &self.prim_indices[first..end] {
                    visit_leaf(pi as usize);
                }
            } else {
                let left_idx = node.left_or_first;
                let right_idx = left_idx + 1;
                let left = &self.nodes[left_idx as usize];
                let right = &self.nodes[right_idx as usize];
                if intersect_aabb_slab(origin, inv_dir, right.aabb_min, right.aabb_max, max_t)
                    .is_some()
                    && sp < stack.len()
                {
                    stack[sp] = right_idx;
                    sp += 1;
                }
                if intersect_aabb_slab(origin, inv_dir, left.aabb_min, left.aabb_max, max_t)
                    .is_some()
                    && sp < stack.len()
                {
                    stack[sp] = left_idx;
                    sp += 1;
                }
            }
        }
    }

    /// Traverses the BVH along `(origin, dir)` up to `max_t` with each node AABB inflated outward
    /// by `radius_mm`.
    ///
    /// `visit_leaf(prim_index)` may return `Some(tighter_radius)` to dynamically shrink the
    /// active inflation radius for remaining subtrees.
    pub fn traverse_ray_inflated<F>(
        &self,
        origin: DVec3,
        dir: DVec3,
        radius_mm: f64,
        max_t: f64,
        mut visit_leaf: F,
    ) where
        F: FnMut(usize) -> Option<f64>,
    {
        if self.nodes.is_empty()
            || !origin.is_finite()
            || !dir.is_finite()
            || !radius_mm.is_finite()
            || radius_mm < 0.0
            || max_t.is_nan()
            || max_t < 0.0
            || dir.length_squared() < 1e-24
        {
            return;
        }

        let inv_dir = DVec3::new(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);
        let mut eff_radius = radius_mm;
        let root = &self.nodes[0];
        let Some(root_t) = intersect_aabb_slab_inflated(
            origin,
            inv_dir,
            root.aabb_min,
            root.aabb_max,
            eff_radius,
            max_t,
        ) else {
            return;
        };

        let mut stack: [(u32, f64); 64] = [(0, 0.0); 64];
        let mut sp = 1usize;
        stack[0] = (0, root_t);

        while sp > 0 {
            sp -= 1;
            let (node_idx, node_t) = stack[sp];
            if node_t > max_t {
                continue;
            }
            let node = &self.nodes[node_idx as usize];
            if intersect_aabb_slab_inflated(
                origin,
                inv_dir,
                node.aabb_min,
                node.aabb_max,
                eff_radius,
                max_t,
            )
            .is_none()
            {
                continue;
            }

            if node.is_leaf() {
                let first = node.left_or_first as usize;
                let end = first + node.tri_count as usize;
                for &pi in &self.prim_indices[first..end] {
                    if let Some(new_r) = visit_leaf(pi as usize)
                        && new_r.is_finite()
                        && new_r >= 0.0
                        && new_r < eff_radius
                    {
                        eff_radius = new_r;
                    }
                }
            } else {
                let left_idx = node.left_or_first;
                let right_idx = left_idx + 1;
                let left = &self.nodes[left_idx as usize];
                let right = &self.nodes[right_idx as usize];

                let t_left = intersect_aabb_slab_inflated(
                    origin,
                    inv_dir,
                    left.aabb_min,
                    left.aabb_max,
                    eff_radius,
                    max_t,
                );
                let t_right = intersect_aabb_slab_inflated(
                    origin,
                    inv_dir,
                    right.aabb_min,
                    right.aabb_max,
                    eff_radius,
                    max_t,
                );

                match (t_left, t_right) {
                    (Some(tl), Some(tr)) => {
                        if sp + 2 <= stack.len() {
                            if tl <= tr {
                                stack[sp] = (right_idx, tr);
                                stack[sp + 1] = (left_idx, tl);
                            } else {
                                stack[sp] = (left_idx, tl);
                                stack[sp + 1] = (right_idx, tr);
                            }
                            sp += 2;
                        }
                    }
                    (Some(tl), None) => {
                        if sp < stack.len() {
                            stack[sp] = (left_idx, tl);
                            sp += 1;
                        }
                    }
                    (None, Some(tr)) => {
                        if sp < stack.len() {
                            stack[sp] = (right_idx, tr);
                            sp += 1;
                        }
                    }
                    (None, None) => {}
                }
            }
        }
    }

    /// Traverses the BVH in nearest-child-first order to find the closest primitive to `point`.
    ///
    /// `eval_leaf(prim_index, best_dist_sq)` should return `Some((dist_sq, payload))` when
    /// `dist_sq < best_dist_sq` (or when no hit has been recorded yet).
    pub fn traverse_closest_point<T, F>(&self, point: DVec3, mut eval_leaf: F) -> Option<(f64, T)>
    where
        F: FnMut(usize, f64) -> Option<(f64, T)>,
    {
        if self.nodes.is_empty() || !point.is_finite() {
            return None;
        }

        let root_d2 = point_aabb_dist_sq(point, self.nodes[0].aabb_min, self.nodes[0].aabb_max);
        let mut best_d2 = f64::INFINITY;
        let mut best_val: Option<T> = None;

        let mut stack: [(u32, f64); 64] = [(0, 0.0); 64];
        let mut sp = 1usize;
        stack[0] = (0, root_d2);

        while sp > 0 {
            sp -= 1;
            let (node_idx, node_d2) = stack[sp];
            if node_d2 > best_d2 {
                continue;
            }
            let node = &self.nodes[node_idx as usize];
            if node.is_leaf() {
                let first = node.left_or_first as usize;
                let end = first + node.tri_count as usize;
                for &pi in &self.prim_indices[first..end] {
                    if let Some((d2, val)) = eval_leaf(pi as usize, best_d2)
                        && d2.is_finite()
                        && d2 >= 0.0
                        && (best_val.is_none() || d2 < best_d2)
                    {
                        best_d2 = d2;
                        best_val = Some(val);
                    }
                }
            } else {
                let left_idx = node.left_or_first;
                let right_idx = left_idx + 1;
                let left = &self.nodes[left_idx as usize];
                let right = &self.nodes[right_idx as usize];

                let dl = point_aabb_dist_sq(point, left.aabb_min, left.aabb_max);
                let dr = point_aabb_dist_sq(point, right.aabb_min, right.aabb_max);

                let visit_left = dl <= best_d2;
                let visit_right = dr <= best_d2;

                match (visit_left, visit_right) {
                    (true, true) => {
                        if sp + 2 <= stack.len() {
                            if dl <= dr {
                                stack[sp] = (right_idx, dr);
                                stack[sp + 1] = (left_idx, dl);
                            } else {
                                stack[sp] = (left_idx, dl);
                                stack[sp + 1] = (right_idx, dr);
                            }
                            sp += 2;
                        }
                    }
                    (true, false) => {
                        if sp < stack.len() {
                            stack[sp] = (left_idx, dl);
                            sp += 1;
                        }
                    }
                    (false, true) => {
                        if sp < stack.len() {
                            stack[sp] = (right_idx, dr);
                            sp += 1;
                        }
                    }
                    (false, false) => {}
                }
            }
        }

        best_val.map(|v| (best_d2, v))
    }
}

#[allow(clippy::too_many_arguments)]
fn build_recursive(
    node_idx: usize,
    start: usize,
    end: usize,
    depth: usize,
    prim_aabbs: &[Aabb3],
    centroids: &[DVec3],
    prim_indices: &mut [u32],
    nodes: &mut Vec<FlatBvhNode>,
) {
    let count = end - start;
    let mut node_aabb = Aabb3::EMPTY;
    let mut centroid_aabb = Aabb3::EMPTY;
    for &pi in &prim_indices[start..end] {
        let idx = pi as usize;
        node_aabb = node_aabb.union(&prim_aabbs[idx]);
        centroid_aabb.extend_point(centroids[idx]);
    }

    let (aabb_min, aabb_max) = aabb_to_f32_conservative(&node_aabb);

    if count <= MAX_LEAF_PRIMS || depth >= MAX_TREE_DEPTH {
        nodes[node_idx] = FlatBvhNode {
            aabb_min,
            left_or_first: start as u32,
            aabb_max,
            tri_count: count as u32,
        };
        return;
    }

    let centroid_extent = centroid_aabb.max - centroid_aabb.min;
    let max_centroid_extent = centroid_extent
        .x
        .max(centroid_extent.y)
        .max(centroid_extent.z);
    if !max_centroid_extent.is_finite() || max_centroid_extent <= 1e-12 {
        nodes[node_idx] = FlatBvhNode {
            aabb_min,
            left_or_first: start as u32,
            aabb_max,
            tri_count: count as u32,
        };
        return;
    }

    let parent_area = half_surface_area(&node_aabb);
    let mut best_cost = f64::INFINITY;
    let mut best_axis = 0usize;
    let mut best_split_bin = 0usize;

    if parent_area > 1e-18 {
        for axis in 0..3 {
            let extent = centroid_extent[axis];
            if !extent.is_finite() || extent <= 1e-12 {
                continue;
            }
            let c_min = centroid_aabb.min[axis];
            let scale = NUM_SAH_BINS as f64 / extent;

            let mut bin_aabbs = [Aabb3::EMPTY; NUM_SAH_BINS];
            let mut bin_counts = [0usize; NUM_SAH_BINS];

            for &pi in &prim_indices[start..end] {
                let idx = pi as usize;
                let b = (((centroids[idx][axis] - c_min) * scale) as usize).min(NUM_SAH_BINS - 1);
                bin_counts[b] += 1;
                bin_aabbs[b] = bin_aabbs[b].union(&prim_aabbs[idx]);
            }

            let mut left_areas = [0.0f64; NUM_SAH_BINS - 1];
            let mut left_counts = [0usize; NUM_SAH_BINS - 1];
            let mut left_acc = Aabb3::EMPTY;
            let mut l_cnt = 0usize;
            for i in 0..(NUM_SAH_BINS - 1) {
                l_cnt += bin_counts[i];
                left_acc = left_acc.union(&bin_aabbs[i]);
                left_counts[i] = l_cnt;
                left_areas[i] = half_surface_area(&left_acc);
            }

            let mut right_acc = Aabb3::EMPTY;
            let mut r_cnt = 0usize;
            for i in (1..NUM_SAH_BINS).rev() {
                r_cnt += bin_counts[i];
                right_acc = right_acc.union(&bin_aabbs[i]);
                let l_c = left_counts[i - 1];
                if l_c == 0 || r_cnt == 0 {
                    continue;
                }
                let r_area = half_surface_area(&right_acc);
                let cost =
                    0.5 + (left_areas[i - 1] * l_c as f64 + r_area * r_cnt as f64) / parent_area;
                if cost < best_cost {
                    best_cost = cost;
                    best_axis = axis;
                    best_split_bin = i;
                }
            }
        }
    }

    let leaf_cost = count as f64;
    if best_split_bin > 0 && best_cost >= leaf_cost && count <= MAX_SAH_LEAF_PRIMS {
        nodes[node_idx] = FlatBvhNode {
            aabb_min,
            left_or_first: start as u32,
            aabb_max,
            tri_count: count as u32,
        };
        return;
    }

    let mid = if best_split_bin > 0 {
        let c_min = centroid_aabb.min[best_axis];
        let extent = centroid_extent[best_axis];
        let scale = NUM_SAH_BINS as f64 / extent;

        let mut left = start;
        let mut right = end;
        while left < right {
            let idx = prim_indices[left] as usize;
            let b = (((centroids[idx][best_axis] - c_min) * scale) as usize).min(NUM_SAH_BINS - 1);
            if b < best_split_bin {
                left += 1;
            } else {
                right -= 1;
                prim_indices.swap(left, right);
            }
        }
        if left > start && left < end {
            left
        } else {
            start + count / 2
        }
    } else {
        // Fallback midpoint split along longest centroid axis (e.g. collinear 1D segments)
        let axis =
            if centroid_extent.x >= centroid_extent.y && centroid_extent.x >= centroid_extent.z {
                0
            } else if centroid_extent.y >= centroid_extent.z {
                1
            } else {
                2
            };
        let split_pos = 0.5 * (centroid_aabb.min[axis] + centroid_aabb.max[axis]);
        let mut left = start;
        let mut right = end;
        while left < right {
            let idx = prim_indices[left] as usize;
            if centroids[idx][axis] < split_pos {
                left += 1;
            } else {
                right -= 1;
                prim_indices.swap(left, right);
            }
        }
        if left > start && left < end {
            left
        } else {
            start + count / 2
        }
    };

    let left_idx = nodes.len();
    let placeholder = FlatBvhNode {
        aabb_min: [0.0; 3],
        left_or_first: 0,
        aabb_max: [0.0; 3],
        tri_count: 0,
    };
    nodes.push(placeholder);
    nodes.push(placeholder);

    nodes[node_idx] = FlatBvhNode {
        aabb_min,
        left_or_first: left_idx as u32,
        aabb_max,
        tri_count: 0,
    };

    build_recursive(
        left_idx,
        start,
        mid,
        depth + 1,
        prim_aabbs,
        centroids,
        prim_indices,
        nodes,
    );
    build_recursive(
        left_idx + 1,
        mid,
        end,
        depth + 1,
        prim_aabbs,
        centroids,
        prim_indices,
        nodes,
    );
}

#[inline]
pub(crate) fn is_finite_valid_aabb(aabb: &Aabb3) -> bool {
    aabb.is_valid() && aabb.min.is_finite() && aabb.max.is_finite()
}

#[inline]
fn half_surface_area(aabb: &Aabb3) -> f64 {
    if !aabb.is_valid() {
        return 0.0;
    }
    let s = aabb.max - aabb.min;
    s.x * s.y + s.y * s.z + s.z * s.x
}

/// Conservatively converts an exact `f64` lower bound into an `f32` value guaranteed `<= v - margin`.
#[inline]
pub(crate) fn f64_to_f32_down(v: f64) -> f32 {
    if !v.is_finite() {
        return f32::NEG_INFINITY;
    }
    let padded = v - 1e-6 * (1.0 + v.abs());
    let f = padded as f32;
    if f == f32::INFINITY {
        return f32::MAX;
    }
    if f == f32::NEG_INFINITY {
        return f32::NEG_INFINITY;
    }
    if f64::from(f) > padded {
        f.next_down()
    } else {
        f
    }
}

/// Conservatively converts an exact `f64` upper bound into an `f32` value guaranteed `>= v + margin`.
#[inline]
pub(crate) fn f64_to_f32_up(v: f64) -> f32 {
    if !v.is_finite() {
        return f32::INFINITY;
    }
    let padded = v + 1e-6 * (1.0 + v.abs());
    let f = padded as f32;
    if f == f32::NEG_INFINITY {
        return f32::MIN;
    }
    if f == f32::INFINITY {
        return f32::INFINITY;
    }
    if f64::from(f) < padded {
        f.next_up()
    } else {
        f
    }
}

#[inline]
pub(crate) fn aabb_to_f32_conservative(aabb: &Aabb3) -> ([f32; 3], [f32; 3]) {
    (
        [
            f64_to_f32_down(aabb.min.x),
            f64_to_f32_down(aabb.min.y),
            f64_to_f32_down(aabb.min.z),
        ],
        [
            f64_to_f32_up(aabb.max.x),
            f64_to_f32_up(aabb.max.y),
            f64_to_f32_up(aabb.max.z),
        ],
    )
}

#[inline]
pub(crate) fn aabb_overlaps_f32(node: &FlatBvhNode, aabb: &Aabb3) -> bool {
    f64::from(node.aabb_min[0]) <= aabb.max.x
        && f64::from(node.aabb_max[0]) >= aabb.min.x
        && f64::from(node.aabb_min[1]) <= aabb.max.y
        && f64::from(node.aabb_max[1]) >= aabb.min.y
        && f64::from(node.aabb_min[2]) <= aabb.max.z
        && f64::from(node.aabb_max[2]) >= aabb.min.z
}

#[inline]
pub(crate) fn aabb_overlaps_f64(a: &Aabb3, b: &Aabb3) -> bool {
    a.min.x <= b.max.x
        && a.max.x >= b.min.x
        && a.min.y <= b.max.y
        && a.max.y >= b.min.y
        && a.min.z <= b.max.z
        && a.max.z >= b.min.z
}

#[inline]
pub(crate) fn xy_circle_intersects_node(node: &FlatBvhNode, cx: f64, cy: f64, r_sq: f64) -> bool {
    let min_x = f64::from(node.aabb_min[0]);
    let max_x = f64::from(node.aabb_max[0]);
    let min_y = f64::from(node.aabb_min[1]);
    let max_y = f64::from(node.aabb_max[1]);

    let dx = if cx < min_x {
        min_x - cx
    } else if cx > max_x {
        cx - max_x
    } else {
        0.0
    };
    let dy = if cy < min_y {
        min_y - cy
    } else if cy > max_y {
        cy - max_y
    } else {
        0.0
    };
    dx * dx + dy * dy <= r_sq
}

#[inline]
pub(crate) fn xy_circle_intersects_aabb_f64(aabb: &Aabb3, cx: f64, cy: f64, r_sq: f64) -> bool {
    let dx = if cx < aabb.min.x {
        aabb.min.x - cx
    } else if cx > aabb.max.x {
        cx - aabb.max.x
    } else {
        0.0
    };
    let dy = if cy < aabb.min.y {
        aabb.min.y - cy
    } else if cy > aabb.max.y {
        cy - aabb.max.y
    } else {
        0.0
    };
    dx * dx + dy * dy <= r_sq
}

#[inline]
pub(crate) fn z_slab_overlaps_node(node: &FlatBvhNode, z_lo: f64, z_hi: f64) -> bool {
    f64::from(node.aabb_min[2]) <= z_hi && f64::from(node.aabb_max[2]) >= z_lo
}

/// Exact `f64` ray-slab intersection against conservatively padded `f32` node bounds.
#[inline]
pub(crate) fn intersect_aabb_slab(
    origin: DVec3,
    inv_dir: DVec3,
    aabb_min: [f32; 3],
    aabb_max: [f32; 3],
    max_t: f64,
) -> Option<f64> {
    let mut t_min = 0.0_f64;
    let mut t_max = max_t;

    let mins = [
        f64::from(aabb_min[0]),
        f64::from(aabb_min[1]),
        f64::from(aabb_min[2]),
    ];
    let maxs = [
        f64::from(aabb_max[0]),
        f64::from(aabb_max[1]),
        f64::from(aabb_max[2]),
    ];

    for axis in 0..3 {
        let inv_d = inv_dir[axis];
        if inv_d.is_finite() {
            let mut t0 = (mins[axis] - origin[axis]) * inv_d;
            let mut t1 = (maxs[axis] - origin[axis]) * inv_d;
            if t0 > t1 {
                std::mem::swap(&mut t0, &mut t1);
            }
            t_min = t_min.max(t0);
            t_max = t_max.min(t1);
            if !t_min.is_finite() || t_min > t_max {
                return None;
            }
        } else if origin[axis] < mins[axis] || origin[axis] > maxs[axis] {
            return None;
        }
    }
    Some(t_min)
}

/// Exact `f64` ray-slab intersection against node bounds inflated outward by `radius_mm`.
#[inline]
pub(crate) fn intersect_aabb_slab_inflated(
    origin: DVec3,
    inv_dir: DVec3,
    aabb_min: [f32; 3],
    aabb_max: [f32; 3],
    radius_mm: f64,
    max_t: f64,
) -> Option<f64> {
    let mut t_min = 0.0_f64;
    let mut t_max = max_t;
    let pad = radius_mm + 1e-6;

    let mins = [
        f64::from(aabb_min[0]) - pad,
        f64::from(aabb_min[1]) - pad,
        f64::from(aabb_min[2]) - pad,
    ];
    let maxs = [
        f64::from(aabb_max[0]) + pad,
        f64::from(aabb_max[1]) + pad,
        f64::from(aabb_max[2]) + pad,
    ];

    for axis in 0..3 {
        let inv_d = inv_dir[axis];
        if inv_d.is_finite() {
            let mut t0 = (mins[axis] - origin[axis]) * inv_d;
            let mut t1 = (maxs[axis] - origin[axis]) * inv_d;
            if t0 > t1 {
                std::mem::swap(&mut t0, &mut t1);
            }
            t_min = t_min.max(t0);
            t_max = t_max.min(t1);
            if !t_min.is_finite() || t_min > t_max {
                return None;
            }
        } else if origin[axis] < mins[axis] || origin[axis] > maxs[axis] {
            return None;
        }
    }
    Some(t_min)
}

/// Squared Euclidean distance from `p` to the conservative `f32` node AABB (`0.0` if inside).
#[inline]
pub(crate) fn point_aabb_dist_sq(p: DVec3, aabb_min: [f32; 3], aabb_max: [f32; 3]) -> f64 {
    let mut d2 = 0.0_f64;
    for axis in 0..3 {
        let v = p[axis];
        let lo = f64::from(aabb_min[axis]);
        let hi = f64::from(aabb_max[axis]);
        if v < lo {
            let d = lo - v;
            d2 += d * d;
        } else if v > hi {
            let d = v - hi;
            d2 += d * d;
        }
    }
    d2
}

/// Two-sided exact `f64` Möller–Trumbore ray-triangle intersection, returning `(t, bary_u, bary_v)`
/// for hits with `0.0 <= t <= max_t`.
#[inline]
pub fn ray_triangle_intersect(
    origin: DVec3,
    dir: DVec3,
    v0: DVec3,
    v1: DVec3,
    v2: DVec3,
    max_t: f64,
) -> Option<(f64, f64, f64)> {
    let e1 = v1 - v0;
    let e2 = v2 - v0;
    let pvec = dir.cross(e2);
    let det = e1.dot(pvec);
    let dir_len = dir.length();
    if det.abs() < 1e-12 * dir_len {
        return None;
    }
    let inv_det = 1.0 / det;
    let tvec = origin - v0;
    let u = tvec.dot(pvec) * inv_det;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let qvec = tvec.cross(e1);
    let v = dir.dot(qvec) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = e2.dot(qvec) * inv_det;
    if t >= 0.0 && t <= max_t {
        Some((t, u, v))
    } else {
        None
    }
}

/// Exact `f64` closest point on segment `a..b` to query point `p`, returning `(closest_pt, dist_sq, seg_u)`.
#[inline]
pub fn closest_point_on_segment(p: DVec3, a: DVec3, b: DVec3) -> (DVec3, f64, f64) {
    let ab = b - a;
    let len_sq = ab.length_squared();
    if len_sq <= 1e-24 {
        return (a, p.distance_squared(a), 0.0);
    }
    let u = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    let q = a + ab * u;
    (q, p.distance_squared(q), u)
}

/// Exact `f64` closest point on triangle `(a, b, c)` to query point `p` (Ericson §5.1.5 Voronoi
/// region dispatch).
///
/// Returns `(closest_pt, dist_sq, bary_u, bary_v)` such that
/// `closest_pt = (1 - bary_u - bary_v) * a + bary_u * b + bary_v * c`.
#[inline]
pub fn closest_point_on_triangle(p: DVec3, a: DVec3, b: DVec3, c: DVec3) -> (DVec3, f64, f64, f64) {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, p.distance_squared(a), 0.0, 0.0);
    }

    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, p.distance_squared(b), 1.0, 0.0);
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let denom = d1 - d3;
        let v = if denom > 0.0 { d1 / denom } else { 0.0 };
        let q = a + ab * v;
        return (q, p.distance_squared(q), v, 0.0);
    }

    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, p.distance_squared(c), 0.0, 1.0);
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let denom = d2 - d6;
        let w = if denom > 0.0 { d2 / denom } else { 0.0 };
        let q = a + ac * w;
        return (q, p.distance_squared(q), 0.0, w);
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let denom = (d4 - d3) + (d5 - d6);
        let w = if denom > 0.0 { (d4 - d3) / denom } else { 0.0 };
        let q = b + (c - b) * w;
        return (q, p.distance_squared(q), 1.0 - w, w);
    }

    let denom = va + vb + vc;
    if denom.abs() <= 1e-24 {
        let (q0, d0, t0) = closest_point_on_segment(p, a, b);
        let (q1, d1, t1) = closest_point_on_segment(p, b, c);
        let (q2, d2, t2) = closest_point_on_segment(p, c, a);
        if d0 <= d1 && d0 <= d2 {
            return (q0, d0, t0, 0.0);
        }
        if d1 <= d2 {
            return (q1, d1, 1.0 - t1, t1);
        }
        return (q2, d2, 0.0, 1.0 - t2);
    }

    let inv_denom = 1.0 / denom;
    let v = vb * inv_denom;
    let w = vc * inv_denom;
    let q = a + ab * v + ac * w;
    (q, p.distance_squared(q), v, w)
}

/// Clips 3D triangle `(v0, v1, v2)` to the horizontal Z-slab `[z_lo, z_hi]` via exact
/// Sutherland–Hodgman half-space clipping.
///
/// Writes the resulting convex polygon vertices into `out` and returns the output vertex count
/// (`0` if fewer than 2 vertices survive, otherwise `2..=5`). Infinite `z_lo` / `z_hi` bounds
/// are treated as unbounded.
pub fn clip_triangle_to_z_slab(
    v0: DVec3,
    v1: DVec3,
    v2: DVec3,
    z_lo: f64,
    z_hi: f64,
    out: &mut [DVec3; 7],
) -> usize {
    let mut buf_a = [
        v0,
        v1,
        v2,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
    ];
    let mut n = 3usize;

    // Clip against `z >= z_lo`
    if z_lo.is_finite() {
        let mut next_n = 0usize;
        let mut prev = buf_a[n - 1];
        let mut d_prev = prev.z - z_lo;
        let mut prev_in = d_prev >= -1e-12;
        for &cur in &buf_a[..n] {
            let d_cur = cur.z - z_lo;
            let cur_in = d_cur >= -1e-12;
            if cur_in != prev_in {
                let denom = d_prev - d_cur;
                if denom.abs() > 1e-15 && next_n < out.len() {
                    let t = (d_prev / denom).clamp(0.0, 1.0);
                    let mut hit = prev.lerp(cur, t);
                    hit.z = z_lo;
                    out[next_n] = hit;
                    next_n += 1;
                }
            }
            if cur_in && next_n < out.len() {
                out[next_n] = cur;
                next_n += 1;
            }
            prev = cur;
            d_prev = d_cur;
            prev_in = cur_in;
        }
        if next_n < 2 {
            return 0;
        }
        buf_a[..next_n].copy_from_slice(&out[..next_n]);
        n = next_n;
    }

    // Clip against `z <= z_hi`
    if z_hi.is_finite() {
        let mut next_n = 0usize;
        let mut prev = buf_a[n - 1];
        let mut d_prev = z_hi - prev.z;
        let mut prev_in = d_prev >= -1e-12;
        for &cur in &buf_a[..n] {
            let d_cur = z_hi - cur.z;
            let cur_in = d_cur >= -1e-12;
            if cur_in != prev_in {
                let denom = d_prev - d_cur;
                if denom.abs() > 1e-15 && next_n < out.len() {
                    let t = (d_prev / denom).clamp(0.0, 1.0);
                    let mut hit = prev.lerp(cur, t);
                    hit.z = z_hi;
                    out[next_n] = hit;
                    next_n += 1;
                }
            }
            if cur_in && next_n < out.len() {
                out[next_n] = cur;
                next_n += 1;
            }
            prev = cur;
            d_prev = d_cur;
            prev_in = cur_in;
        }
        if next_n < 2 {
            return 0;
        }
        next_n
    } else {
        out[..n].copy_from_slice(&buf_a[..n]);
        n
    }
}

/// Intersects triangle `(v0, v1, v2)` with horizontal plane `Z = z_plane` and returns the closest
/// 3D point on that cross-section (with `.z == z_plane`) to `(target_xy.x, target_xy.y)`.
pub fn closest_point_on_triangle_at_z(
    v0: DVec3,
    v1: DVec3,
    v2: DVec3,
    z_plane: f64,
    target_xy: chip_math::DVec2,
) -> Option<DVec3> {
    let z_min = v0.z.min(v1.z).min(v2.z);
    let z_max = v0.z.max(v1.z).max(v2.z);
    if z_plane < z_min - 1e-6 || z_plane > z_max + 1e-6 {
        return None;
    }

    // Nearly horizontal triangle: use exact 3D triangle closest point directly.
    if z_max - z_min < 1e-5 {
        let (p, _, _, _) =
            closest_point_on_triangle(DVec3::new(target_xy.x, target_xy.y, z_plane), v0, v1, v2);
        return Some(DVec3::new(p.x, p.y, z_plane));
    }

    let verts = [v0, v1, v2];
    let mut hits = [chip_math::DVec2::ZERO; 4];
    let mut n_hits = 0usize;

    for e in 0..3 {
        let a = verts[e];
        let b = verts[(e + 1) % 3];
        let da = a.z - z_plane;
        let db = b.z - z_plane;
        if da.abs() <= 1e-9 {
            if n_hits < hits.len() {
                hits[n_hits] = chip_math::DVec2::new(a.x, a.y);
                n_hits += 1;
            }
        } else if (da < 0.0 && db > 0.0) || (da > 0.0 && db < 0.0) {
            let t = da / (da - db);
            let p = a.lerp(b, t);
            if n_hits < hits.len() {
                hits[n_hits] = chip_math::DVec2::new(p.x, p.y);
                n_hits += 1;
            }
        }
    }

    if n_hits == 0 {
        let (p, _, _, _) =
            closest_point_on_triangle(DVec3::new(target_xy.x, target_xy.y, z_plane), v0, v1, v2);
        return Some(DVec3::new(p.x, p.y, z_plane));
    }
    if n_hits == 1 {
        return Some(DVec3::new(hits[0].x, hits[0].y, z_plane));
    }

    let mut best_pt = hits[0];
    let mut best_d2 = hits[0].distance_squared(target_xy);
    for i in 0..n_hits {
        let a = hits[i];
        let b = hits[(i + 1) % n_hits];
        let cp = chip_math::closest_point_on_segment_2d(target_xy, a, b);
        let d2 = cp.distance_squared(target_xy);
        if d2 < best_d2 {
            best_d2 = d2;
            best_pt = cp;
        }
    }
    Some(DVec3::new(best_pt.x, best_pt.y, z_plane))
}
