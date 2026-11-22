// Direct `chip-bvh` GPU BVH Target Rasterizer & Closest-Point Distance Evaluator
// Consumes `FlatBvhNode` (32B) and `GpuTriangle` (48B) directly with zero CPU conversion.
// Provides 4 compute entry points (each <= 5 storage buffers):
// - `rasterize_tridexel_plane`: axis-generic even-odd parity voxelization for Tri-Dexel target
// - `rasterize_cylindrical_target`: inward radial raycast for 4th-axis Cylindrical `r_target`
// - `rasterize_heightfield_target`: top-down `-Z` raycast for 2.5D Heightfield `z_target`
// - `closest_point_distance`: nearest-child-first BVH closest-point-on-triangle distance query

const INLINE_K: u32 = 4u;
const MAX_HITS: u32 = 32u;
const UNCUT_OP_PACKED: u32 = 0xffffffffu;

struct FlatBvhNode {
    aabb_min: vec3<f32>,
    left_or_first: u32,
    aabb_max: vec3<f32>,
    tri_count: u32,
}

struct GpuTriangle {
    v0: vec3<f32>,
    face_id: u32,
    v1: vec3<f32>,
    flags: u32,
    v2: vec3<f32>,
    prim_id: u32,
}

struct FlatDexelSpan {
    min_local: f32,
    max_local: f32,
    op_packed: u32,
    _pad: u32,
}

// 112-byte (7 x 16B) unified uniform buffer across all 4 BVH compute modes.
struct BvhDispatchUniforms {
    // [dim0, dim1, mode_param0 (axis_id / count), mode_param1 (plane_ray_offset / flags)]
    dims: vec4<u32>,
    // TriDexel: [origin_u, origin_v, origin_w, ray_start_w]
    // Heightfield: [origin_x, origin_y, origin_z, z_top_cast]
    // Cylindrical: [center.x, center.y, center.z, r_cast]
    v0: vec4<f32>,
    // TriDexel: [du, dv, max_t, 0]
    // Heightfield: [dx, dy, max_t, 0]
    // Cylindrical: [e1.x, e1.y, e1.z, u_min]
    v1: vec4<f32>,
    // Cylindrical: [e2.x, e2.y, e2.z, du]
    v2: vec4<f32>,
    // Cylindrical: [u_vec.x, u_vec.y, u_vec.z, dtheta]
    v3: vec4<f32>,
    // Cylindrical: [u_center, direction_sign, 0, 0]
    v4: vec4<f32>,
    // [node_count, tri_count, 0, 0]
    counts: vec4<u32>,
}

struct RayTriHit {
    hit: bool,
    t: f32,
    nz: f32,
}

struct ClosestTriResult {
    pt: vec3<f32>,
    dist_sq: f32,
}

@group(0) @binding(0) var<uniform> u: BvhDispatchUniforms;
@group(0) @binding(1) var<storage, read> bvh_nodes: array<FlatBvhNode>;
@group(0) @binding(2) var<storage, read> gpu_tris: array<GpuTriangle>;

// Mode-specific output bindings in @group(1) (distinct binding numbers so all 4 entry points
// coexist in one shader module while each pipeline binds only its own subset <= 5 storage bufs).
@group(1) @binding(0) var<storage, read_write> td_slots: array<FlatDexelSpan>;
@group(1) @binding(1) var<storage, read_write> td_counts: array<u32>;
@group(1) @binding(2) var<storage, read_write> td_overflow: array<atomic<u32>>;
@group(1) @binding(3) var<storage, read_write> cyl_r_target: array<f32>;
@group(1) @binding(4) var<storage, read_write> hf_z_target: array<f32>;
@group(1) @binding(5) var<storage, read> query_points: array<vec4<f32>>;
@group(1) @binding(6) var<storage, read_write> closest_out: array<vec4<f32>>;

fn neg_inf_f32() -> f32 {
    return bitcast<f32>(0xff800000u);
}

// NaN/Inf-safe ray-AABB slab intersection returning entry distance `>= 0.0`, or `< 0.0` on miss.
fn intersect_aabb_slab(
    origin: vec3<f32>,
    dir: vec3<f32>,
    bmin: vec3<f32>,
    bmax: vec3<f32>,
    max_t: f32,
) -> f32 {
    var t_min: f32 = 0.0;
    var t_max: f32 = max_t;

    if (abs(dir.x) < 1e-12) {
        if (origin.x < bmin.x || origin.x > bmax.x) {
            return -1.0;
        }
    } else {
        let inv = 1.0 / dir.x;
        let t0 = (bmin.x - origin.x) * inv;
        let t1 = (bmax.x - origin.x) * inv;
        t_min = max(t_min, min(t0, t1));
        t_max = min(t_max, max(t0, t1));
        if (t_min > t_max) {
            return -1.0;
        }
    }

    if (abs(dir.y) < 1e-12) {
        if (origin.y < bmin.y || origin.y > bmax.y) {
            return -1.0;
        }
    } else {
        let inv = 1.0 / dir.y;
        let t0 = (bmin.y - origin.y) * inv;
        let t1 = (bmax.y - origin.y) * inv;
        t_min = max(t_min, min(t0, t1));
        t_max = min(t_max, max(t0, t1));
        if (t_min > t_max) {
            return -1.0;
        }
    }

    if (abs(dir.z) < 1e-12) {
        if (origin.z < bmin.z || origin.z > bmax.z) {
            return -1.0;
        }
    } else {
        let inv = 1.0 / dir.z;
        let t0 = (bmin.z - origin.z) * inv;
        let t1 = (bmax.z - origin.z) * inv;
        t_min = max(t_min, min(t0, t1));
        t_max = min(t_max, max(t0, t1));
        if (t_min > t_max) {
            return -1.0;
        }
    }

    return t_min;
}

// Two-sided Moller-Trumbore ray-triangle intersection.
fn ray_tri_intersect(
    origin: vec3<f32>,
    dir: vec3<f32>,
    tri: GpuTriangle,
    max_t: f32,
) -> RayTriHit {
    let miss = RayTriHit(false, 0.0, 0.0);
    let e1 = tri.v1 - tri.v0;
    let e2 = tri.v2 - tri.v0;
    let pvec = cross(dir, e2);
    let det = dot(e1, pvec);
    if (abs(det) < 1e-10) {
        return miss;
    }
    let inv_det = 1.0 / det;
    let tvec = origin - tri.v0;
    let bu = dot(tvec, pvec) * inv_det;
    if (bu < 0.0 || bu > 1.0) {
        return miss;
    }
    let qvec = cross(tvec, e1);
    let bv = dot(dir, qvec) * inv_det;
    if (bv < 0.0 || bu + bv > 1.0) {
        return miss;
    }
    let t = dot(e2, qvec) * inv_det;
    if (t >= 0.0 && t <= max_t) {
        let n = cross(e1, e2);
        let n_len = length(n);
        let nz = select(0.0, n.z / n_len, n_len > 1e-10);
        return RayTriHit(true, t, nz);
    }
    return miss;
}

fn point_aabb_dist_sq(p: vec3<f32>, bmin: vec3<f32>, bmax: vec3<f32>) -> f32 {
    let clamped = clamp(p, bmin, bmax);
    let d = p - clamped;
    return dot(d, d);
}

fn closest_point_on_segment(p: vec3<f32>, a: vec3<f32>, b: vec3<f32>) -> ClosestTriResult {
    let ab = b - a;
    let len_sq = dot(ab, ab);
    if (len_sq <= 1e-12) {
        let d = p - a;
        return ClosestTriResult(a, dot(d, d));
    }
    let t = clamp(dot(p - a, ab) / len_sq, 0.0, 1.0);
    let q = a + ab * t;
    let d = p - q;
    return ClosestTriResult(q, dot(d, d));
}

// Ericson section 5.1.5 exact Voronoi closest point on triangle (a, b, c).
fn closest_point_on_triangle(p: vec3<f32>, a: vec3<f32>, b: vec3<f32>, c: vec3<f32>) -> ClosestTriResult {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        return ClosestTriResult(a, dot(ap, ap));
    }

    let bp = p - b;
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) {
        return ClosestTriResult(b, dot(bp, bp));
    }

    let vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        let denom = d1 - d3;
        let v = select(0.0, d1 / denom, denom > 0.0);
        let q = a + ab * v;
        let d = p - q;
        return ClosestTriResult(q, dot(d, d));
    }

    let cp = p - c;
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) {
        return ClosestTriResult(c, dot(cp, cp));
    }

    let vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        let denom = d2 - d6;
        let w = select(0.0, d2 / denom, denom > 0.0);
        let q = a + ac * w;
        let d = p - q;
        return ClosestTriResult(q, dot(d, d));
    }

    let va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        let denom = (d4 - d3) + (d5 - d6);
        let w = select(0.0, (d4 - d3) / denom, denom > 0.0);
        let q = b + (c - b) * w;
        let d = p - q;
        return ClosestTriResult(q, dot(d, d));
    }

    let denom = va + vb + vc;
    if (abs(denom) <= 1e-12) {
        let r0 = closest_point_on_segment(p, a, b);
        let r1 = closest_point_on_segment(p, b, c);
        let r2 = closest_point_on_segment(p, c, a);
        if (r0.dist_sq <= r1.dist_sq && r0.dist_sq <= r2.dist_sq) {
            return r0;
        }
        if (r1.dist_sq <= r2.dist_sq) {
            return r1;
        }
        return r2;
    }

    let inv_denom = 1.0 / denom;
    let v = vb * inv_denom;
    let w = vc * inv_denom;
    let q = a + ab * v + ac * w;
    let d = p - q;
    return ClosestTriResult(q, dot(d, d));
}

fn from_uvw(axis_id: u32, pu: f32, pv: f32, pw: f32) -> vec3<f32> {
    if (axis_id == 0u) {
        return vec3<f32>(pw, pu, pv);
    } else if (axis_id == 1u) {
        return vec3<f32>(pu, pw, pv);
    }
    return vec3<f32>(pu, pv, pw);
}

fn axis_dir(axis_id: u32) -> vec3<f32> {
    if (axis_id == 0u) {
        return vec3<f32>(1.0, 0.0, 0.0);
    } else if (axis_id == 1u) {
        return vec3<f32>(0.0, 1.0, 0.0);
    }
    return vec3<f32>(0.0, 0.0, 1.0);
}

// === Entry Point 1: GPU Tri-Dexel Target Voxelization (`raycast_all_hits` even-odd parity) ===
@compute @workgroup_size(16, 16, 1)
fn rasterize_tridexel_plane(@builtin(global_invocation_id) gid: vec3<u32>) {
    let iu = gid.x;
    let iv = gid.y;
    let nu = u.dims.x;
    let nv = u.dims.y;
    if (iu >= nu || iv >= nv) {
        return;
    }

    let axis_id = u.dims.z;
    let plane_ray_offset = u.dims.w;
    let local_ray_idx = iv * nu + iu;
    let global_ray_idx = plane_ray_offset + local_ray_idx;
    let slot_base = global_ray_idx * INLINE_K;

    if (u.counts.x == 0u) {
        td_counts[global_ray_idx] = 0u;
        for (var k: u32 = 0u; k < INLINE_K; k = k + 1u) {
            td_slots[slot_base + k] = FlatDexelSpan(0.0, 0.0, 0u, 0u);
        }
        return;
    }

    let origin_u = u.v0.x;
    let origin_v = u.v0.y;
    let origin_w = u.v0.z;
    let ray_start_w = u.v0.w;
    let du = u.v1.x;
    let dv = u.v1.y;
    let max_t = u.v1.z;

    // Deterministic sub-micron jitter scaled cleanly above f32 ULP to avoid shared-edge ties
    let jitter_u = 1.37191e-4 * du;
    let jitter_v = 2.71828e-4 * dv;
    let pu = origin_u + (f32(iu) + 0.5) * du + jitter_u;
    let pv = origin_v + (f32(iv) + 0.5) * dv + jitter_v;

    let ray_origin = from_uvw(axis_id, pu, pv, ray_start_w);
    let ray_dir = axis_dir(axis_id);

    var hits: array<f32, 32>;
    var n_hits: u32 = 0u;
    var hit_cap_exceeded = false;

    if (intersect_aabb_slab(ray_origin, ray_dir, bvh_nodes[0].aabb_min, bvh_nodes[0].aabb_max, max_t) >= 0.0) {
        var stack: array<u32, 64>;
        var sp: u32 = 1u;
        stack[0] = 0u;

        while (sp > 0u) {
            sp = sp - 1u;
            let node = bvh_nodes[stack[sp]];
            if (node.tri_count > 0u) {
                let first = node.left_or_first;
                let end = first + node.tri_count;
                for (var ti: u32 = first; ti < end; ti = ti + 1u) {
                    let h = ray_tri_intersect(ray_origin, ray_dir, gpu_tris[ti], max_t);
                    if (h.hit) {
                        let w_val = ray_start_w + h.t;
                        if (n_hits < MAX_HITS) {
                            hits[n_hits] = w_val;
                            n_hits = n_hits + 1u;
                        } else {
                            hit_cap_exceeded = true;
                        }
                    }
                }
            } else {
                let left_idx = node.left_or_first;
                let right_idx = left_idx + 1u;
                let right = bvh_nodes[right_idx];
                let left = bvh_nodes[left_idx];
                if (intersect_aabb_slab(ray_origin, ray_dir, right.aabb_min, right.aabb_max, max_t) >= 0.0 && sp < 64u) {
                    stack[sp] = right_idx;
                    sp = sp + 1u;
                }
                if (intersect_aabb_slab(ray_origin, ray_dir, left.aabb_min, left.aabb_max, max_t) >= 0.0 && sp < 64u) {
                    stack[sp] = left_idx;
                    sp = sp + 1u;
                }
            }
        }
    }

    if (n_hits < 2u) {
        td_counts[global_ray_idx] = 0u;
        for (var k: u32 = 0u; k < INLINE_K; k = k + 1u) {
            td_slots[slot_base + k] = FlatDexelSpan(0.0, 0.0, 0u, 0u);
        }
        return;
    }

    // Insertion sort hits ascending
    for (var i: u32 = 1u; i < n_hits; i = i + 1u) {
        let key = hits[i];
        var j: u32 = i;
        while (j > 0u && hits[j - 1u] > key) {
            hits[j] = hits[j - 1u];
            j = j - 1u;
        }
        hits[j] = key;
    }

    // Deduplicate hits within 1e-4 mm
    var clean_n: u32 = 0u;
    for (var i: u32 = 0u; i < n_hits; i = i + 1u) {
        let w = hits[i];
        if (clean_n == 0u || abs(w - hits[clean_n - 1u]) > 1e-4) {
            hits[clean_n] = w;
            clean_n = clean_n + 1u;
        }
    }

    let pairs = clean_n / 2u;
    var span_count: u32 = 0u;
    for (var p: u32 = 0u; p < pairs; p = p + 1u) {
        let a = hits[2u * p] - origin_w;
        let b = hits[2u * p + 1u] - origin_w;
        if (b - a > 1e-5) {
            if (span_count < INLINE_K) {
                td_slots[slot_base + span_count] = FlatDexelSpan(a, b, UNCUT_OP_PACKED, 0u);
            }
            span_count = span_count + 1u;
        }
    }

    for (var k: u32 = span_count; k < INLINE_K; k = k + 1u) {
        td_slots[slot_base + k] = FlatDexelSpan(0.0, 0.0, 0u, 0u);
    }

    if (span_count > INLINE_K || hit_cap_exceeded) {
        td_counts[global_ray_idx] = max(span_count, INLINE_K + 1u);
        atomicStore(&td_overflow[0], 1u);
        let word = 1u + (local_ray_idx >> 5u);
        let mask = 1u << (local_ray_idx & 31u);
        atomicOr(&td_overflow[word], mask);
    } else {
        td_counts[global_ray_idx] = span_count;
    }
}

// === Entry Point 2: GPU Cylindrical Target Inward Radial Raycast ===
@compute @workgroup_size(16, 16, 1)
fn rasterize_cylindrical_target(@builtin(global_invocation_id) gid: vec3<u32>) {
    let it = gid.x;
    let iu = gid.y;
    let nu = u.dims.x;
    let ntheta = u.dims.y;
    if (iu >= nu || it >= ntheta) {
        return;
    }

    let idx = iu * ntheta + it;
    if (u.counts.x == 0u) {
        cyl_r_target[idx] = -1.0;
        return;
    }

    let center = u.v0.xyz;
    let r_cast = u.v0.w;
    let e1 = u.v1.xyz;
    let u_min = u.v1.w;
    let e2 = u.v2.xyz;
    let du = u.v2.w;
    let u_vec = u.v3.xyz;
    let dtheta = u.v3.w;
    let u_center = u.v4.x;
    let pol = u.v4.y;

    let u_coord = u_min + (f32(iu) + 0.5) * du;
    let theta = f32(it) * dtheta;
    let d_rad = e1 * cos(theta) + e2 * (sin(theta) * pol);
    let p_axis = center + u_vec * (u_coord - u_center);
    let p_outer = p_axis + d_rad * r_cast;
    let ray_dir = -d_rad;

    var best_t = r_cast;
    var has_hit = false;

    let root_t = intersect_aabb_slab(p_outer, ray_dir, bvh_nodes[0].aabb_min, bvh_nodes[0].aabb_max, best_t);
    if (root_t >= 0.0) {
        var stack_idx: array<u32, 64>;
        var stack_t: array<f32, 64>;
        var sp: u32 = 1u;
        stack_idx[0] = 0u;
        stack_t[0] = root_t;

        while (sp > 0u) {
            sp = sp - 1u;
            if (stack_t[sp] > best_t) {
                continue;
            }
            let node = bvh_nodes[stack_idx[sp]];
            if (node.tri_count > 0u) {
                let first = node.left_or_first;
                let end = first + node.tri_count;
                for (var ti: u32 = first; ti < end; ti = ti + 1u) {
                    let h = ray_tri_intersect(p_outer, ray_dir, gpu_tris[ti], best_t);
                    if (h.hit && (!has_hit || h.t < best_t)) {
                        best_t = h.t;
                        has_hit = true;
                    }
                }
            } else {
                let left_idx = node.left_or_first;
                let right_idx = left_idx + 1u;
                let left = bvh_nodes[left_idx];
                let right = bvh_nodes[right_idx];
                let tl = intersect_aabb_slab(p_outer, ray_dir, left.aabb_min, left.aabb_max, best_t);
                let tr = intersect_aabb_slab(p_outer, ray_dir, right.aabb_min, right.aabb_max, best_t);
                if (tl >= 0.0 && tr >= 0.0) {
                    if (sp + 2u <= 64u) {
                        if (tl <= tr) {
                            stack_idx[sp] = right_idx; stack_t[sp] = tr;
                            stack_idx[sp + 1u] = left_idx; stack_t[sp + 1u] = tl;
                        } else {
                            stack_idx[sp] = left_idx; stack_t[sp] = tl;
                            stack_idx[sp + 1u] = right_idx; stack_t[sp + 1u] = tr;
                        }
                        sp = sp + 2u;
                    }
                } else if (tl >= 0.0 && sp < 64u) {
                    stack_idx[sp] = left_idx; stack_t[sp] = tl; sp = sp + 1u;
                } else if (tr >= 0.0 && sp < 64u) {
                    stack_idx[sp] = right_idx; stack_t[sp] = tr; sp = sp + 1u;
                }
            }
        }
    }

    if (has_hit) {
        cyl_r_target[idx] = max(r_cast - best_t, 0.0);
    } else {
        cyl_r_target[idx] = -1.0;
    }
}

// === Entry Point 3: GPU Heightfield Target Top-Down `-Z` Raycast ===
@compute @workgroup_size(16, 16, 1)
fn rasterize_heightfield_target(@builtin(global_invocation_id) gid: vec3<u32>) {
    let ix = gid.x;
    let iy = gid.y;
    let nx = u.dims.x;
    let ny = u.dims.y;
    if (ix >= nx || iy >= ny) {
        return;
    }

    let idx = iy * nx + ix;
    if (u.counts.x == 0u) {
        hf_z_target[idx] = neg_inf_f32();
        return;
    }

    let ox = u.v0.x;
    let oy = u.v0.y;
    let oz = u.v0.z;
    let z_top_cast = u.v0.w;
    let dx = u.v1.x;
    let dy = u.v1.y;
    let max_t = u.v1.z;
    let require_upward = u.dims.w != 0u;

    let px = ox + (f32(ix) + 0.5) * dx + 1.37191e-4 * dx;
    let py = oy + (f32(iy) + 0.5) * dy + 2.71828e-4 * dy;
    let ray_origin = vec3<f32>(px, py, z_top_cast);
    let ray_dir = vec3<f32>(0.0, 0.0, -1.0);

    var best_t = max_t;
    var has_hit = false;

    let root_t = intersect_aabb_slab(ray_origin, ray_dir, bvh_nodes[0].aabb_min, bvh_nodes[0].aabb_max, best_t);
    if (root_t >= 0.0) {
        var stack_idx: array<u32, 64>;
        var stack_t: array<f32, 64>;
        var sp: u32 = 1u;
        stack_idx[0] = 0u;
        stack_t[0] = root_t;

        while (sp > 0u) {
            sp = sp - 1u;
            if (stack_t[sp] > best_t) {
                continue;
            }
            let node = bvh_nodes[stack_idx[sp]];
            if (node.tri_count > 0u) {
                let first = node.left_or_first;
                let end = first + node.tri_count;
                for (var ti: u32 = first; ti < end; ti = ti + 1u) {
                    let h = ray_tri_intersect(ray_origin, ray_dir, gpu_tris[ti], best_t);
                    if (h.hit && (!require_upward || h.nz > 1e-4)) {
                        if (!has_hit || h.t < best_t) {
                            best_t = h.t;
                            has_hit = true;
                        }
                    }
                }
            } else {
                let left_idx = node.left_or_first;
                let right_idx = left_idx + 1u;
                let left = bvh_nodes[left_idx];
                let right = bvh_nodes[right_idx];
                let tl = intersect_aabb_slab(ray_origin, ray_dir, left.aabb_min, left.aabb_max, best_t);
                let tr = intersect_aabb_slab(ray_origin, ray_dir, right.aabb_min, right.aabb_max, best_t);
                if (tl >= 0.0 && tr >= 0.0) {
                    if (sp + 2u <= 64u) {
                        if (tl <= tr) {
                            stack_idx[sp] = right_idx; stack_t[sp] = tr;
                            stack_idx[sp + 1u] = left_idx; stack_t[sp + 1u] = tl;
                        } else {
                            stack_idx[sp] = left_idx; stack_t[sp] = tl;
                            stack_idx[sp + 1u] = right_idx; stack_t[sp + 1u] = tr;
                        }
                        sp = sp + 2u;
                    }
                } else if (tl >= 0.0 && sp < 64u) {
                    stack_idx[sp] = left_idx; stack_t[sp] = tl; sp = sp + 1u;
                } else if (tr >= 0.0 && sp < 64u) {
                    stack_idx[sp] = right_idx; stack_t[sp] = tr; sp = sp + 1u;
                }
            }
        }
    }

    if (has_hit) {
        let z_hit = z_top_cast - best_t;
        hf_z_target[idx] = z_hit - oz;
    } else {
        hf_z_target[idx] = neg_inf_f32();
    }
}

// === Entry Point 4: GPU Point-to-Mesh Closest-Point Distance Query ===
@compute @workgroup_size(64, 1, 1)
fn closest_point_distance(@builtin(global_invocation_id) gid: vec3<u32>) {
    let q_idx = gid.x;
    let num_queries = u.dims.z;
    if (q_idx >= num_queries) {
        return;
    }
    if (u.counts.x == 0u) {
        closest_out[q_idx] = vec4<f32>(0.0, 0.0, 0.0, -1.0);
        return;
    }

    let p = query_points[q_idx].xyz;
    var best_d2: f32 = 1e30;
    var best_pt = vec3<f32>(0.0, 0.0, 0.0);
    var found = false;

    let root_d2 = point_aabb_dist_sq(p, bvh_nodes[0].aabb_min, bvh_nodes[0].aabb_max);
    var stack_idx: array<u32, 64>;
    var stack_d2: array<f32, 64>;
    var sp: u32 = 1u;
    stack_idx[0] = 0u;
    stack_d2[0] = root_d2;

    while (sp > 0u) {
        sp = sp - 1u;
        if (stack_d2[sp] > best_d2) {
            continue;
        }
        let node = bvh_nodes[stack_idx[sp]];
        if (node.tri_count > 0u) {
            let first = node.left_or_first;
            let end = first + node.tri_count;
            for (var ti: u32 = first; ti < end; ti = ti + 1u) {
                let tri = gpu_tris[ti];
                let res = closest_point_on_triangle(p, tri.v0, tri.v1, tri.v2);
                if (!found || res.dist_sq < best_d2) {
                    best_d2 = res.dist_sq;
                    best_pt = res.pt;
                    found = true;
                }
            }
        } else {
            let left_idx = node.left_or_first;
            let right_idx = left_idx + 1u;
            let left = bvh_nodes[left_idx];
            let right = bvh_nodes[right_idx];
            let dl = point_aabb_dist_sq(p, left.aabb_min, left.aabb_max);
            let dr = point_aabb_dist_sq(p, right.aabb_min, right.aabb_max);
            let vl = dl <= best_d2;
            let vr = dr <= best_d2;
            if (vl && vr) {
                if (sp + 2u <= 64u) {
                    if (dl <= dr) {
                        stack_idx[sp] = right_idx; stack_d2[sp] = dr;
                        stack_idx[sp + 1u] = left_idx; stack_d2[sp + 1u] = dl;
                    } else {
                        stack_idx[sp] = left_idx; stack_d2[sp] = dl;
                        stack_idx[sp + 1u] = right_idx; stack_d2[sp + 1u] = dr;
                    }
                    sp = sp + 2u;
                }
            } else if (vl && sp < 64u) {
                stack_idx[sp] = left_idx; stack_d2[sp] = dl; sp = sp + 1u;
            } else if (vr && sp < 64u) {
                stack_idx[sp] = right_idx; stack_d2[sp] = dr; sp = sp + 1u;
            }
        }
    }

    if (found) {
        closest_out[q_idx] = vec4<f32>(best_pt, sqrt(max(best_d2, 0.0)));
    } else {
        closest_out[q_idx] = vec4<f32>(0.0, 0.0, 0.0, -1.0);
    }
}
