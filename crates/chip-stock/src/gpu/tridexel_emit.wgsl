// Direct GPU Tri-Dexel Surface Extraction Compute Pass (`tridexel_emit.wgsl`)
// Scans all 3 orthogonal dexel planes (`X`, `Y`, `Z`), emits packed 40-byte `SimColoredVertex`
// quads with smooth finite-difference normals on curved surfaces & crisp normals on walls,
// evaluates `Metallic` / `ByOperation` / signed `ErrorHeatmap` coloring directly on GPU,
// and populates `DrawIndirectArgs` (`[vertex_count, 1, 0, 0]`) for zero-readback `draw_indirect`.

const INLINE_K: u32 = 4u;
const UNCUT_OP_IDX: u32 = 0xFFFFFFFFu;
const UNCUT_OP_TAG_U16: u32 = 0xFFFFu;

const COLOR_MODE_METALLIC: u32 = 0u;
const COLOR_MODE_BY_OP: u32 = 1u;
const COLOR_MODE_HEATMAP: u32 = 2u;

struct TriDexelEmitUniforms {
    // [nx, ny, nz, total_rays]
    dims_and_total: vec4<u32>,
    // [ray_off_x, ray_off_y, ray_off_z, has_target]
    plane_offsets_and_target: vec4<u32>,
    // [color_mode_id, max_vertices, _pad0, _pad1]
    mode_and_cap: vec4<u32>,
    // [origin_x, origin_y, origin_z, tolerance_mm]
    origin_and_tol: vec4<f32>,
    // [dx, dy, dz, max_scale_mm]
    pitch_and_scale: vec4<f32>,
    raw_top_color: vec4<f32>,
    raw_side_color: vec4<f32>,
    machined_metal_color: vec4<f32>,
    op_palette: array<vec4<f32>, 16>,
}

struct FlatDexelSpan {
    min_local: f32,
    max_local: f32,
    op_packed: u32,
    _pad: u32,
}

// Exact 40-byte tightly-packed vertex matching Rust `SimColoredVertex` / `SimVertexGpu` (align 4).
struct PackedSimVertex {
    px: f32,
    py: f32,
    pz: f32,
    nx: f32,
    ny: f32,
    nz: f32,
    r: f32,
    g: f32,
    b: f32,
    a: f32,
}

// 1 uniform + 6 storage buffers (strictly within WebGPU default limit of 8 storage buffers/stage)
@group(0) @binding(0) var<uniform> u: TriDexelEmitUniforms;
@group(0) @binding(1) var<storage, read> m_slots: array<FlatDexelSpan>;
@group(0) @binding(2) var<storage, read> m_counts: array<u32>;
@group(0) @binding(3) var<storage, read> t_slots: array<FlatDexelSpan>;
@group(0) @binding(4) var<storage, read> t_counts: array<u32>;
@group(0) @binding(5) var<storage, read_write> out_vertices: array<PackedSimVertex>;
@group(0) @binding(6) var<storage, read_write> draw_args: array<atomic<u32>, 4>;

fn unpack_min_op(op_packed: u32) -> u32 {
    let lo = op_packed & 0xFFFFu;
    if (lo == UNCUT_OP_TAG_U16) {
        return UNCUT_OP_IDX;
    }
    return lo;
}

fn unpack_max_op(op_packed: u32) -> u32 {
    let hi = (op_packed >> 16u) & 0xFFFFu;
    if (hi == UNCUT_OP_TAG_U16) {
        return UNCUT_OP_IDX;
    }
    return hi;
}

fn eval_op_color(op_idx: u32, is_side: bool) -> vec4<f32> {
    if (op_idx == UNCUT_OP_IDX) {
        if (is_side) {
            return u.raw_side_color;
        }
        return u.raw_top_color;
    }
    if (u.mode_and_cap.x == COLOR_MODE_BY_OP) {
        return u.op_palette[op_idx & 15u];
    }
    return u.machined_metal_color;
}

fn deviation_to_heatmap_rgba(signed_dev: f32, tol_in: f32, max_scale: f32) -> vec4<f32> {
    let tol = max(tol_in, 1e-4);
    let span = max(max_scale - tol, tol);
    let abs_d = abs(signed_dev);
    if (abs_d <= tol) {
        return vec4<f32>(0.18, 0.78, 0.36, 1.0);
    }
    let t = clamp((abs_d - tol) / span, 0.0, 1.0);
    if (signed_dev > 0.0) {
        return vec4<f32>(
            0.22 + t * (0.12 - 0.22),
            0.80 + t * (0.28 - 0.80),
            0.94 + t * (0.88 - 0.94),
            1.0,
        );
    }
    return vec4<f32>(
        0.98 + t * (0.94 - 0.98),
        0.78 + t * (0.16 - 0.78),
        0.16 + t * (0.20 - 0.16),
        1.0,
    );
}

fn eval_signed_deviation(global_ray: u32, machined_coord: f32) -> f32 {
    let t_cnt = min(t_counts[global_ray], INLINE_K);
    if (t_cnt == 0u) {
        return 10.0;
    }
    let t_base = global_ray * INLINE_K;
    for (var k = 0u; k < t_cnt; k = k + 1u) {
        let ts = t_slots[t_base + k];
        if (machined_coord > ts.min_local && machined_coord < ts.max_local) {
            return -min(machined_coord - ts.min_local, ts.max_local - machined_coord);
        }
    }
    var min_dist = 1e30;
    for (var k = 0u; k < t_cnt; k = k + 1u) {
        let ts = t_slots[t_base + k];
        var d = 0.0;
        if (machined_coord <= ts.min_local) {
            d = ts.min_local - machined_coord;
        } else {
            d = machined_coord - ts.max_local;
        }
        min_dist = min(min_dist, d);
    }
    if (min_dist < 1e20) {
        return min_dist;
    }
    return 0.0;
}

fn eval_endpoint_color(global_ray: u32, w_local: f32, op_idx: u32, is_side: bool) -> vec4<f32> {
    if (u.mode_and_cap.x == COLOR_MODE_HEATMAP && u.plane_offsets_and_target.w != 0u) {
        let dev = eval_signed_deviation(global_ray, w_local);
        return deviation_to_heatmap_rgba(dev, u.origin_and_tol.w, u.pitch_and_scale.w);
    }
    return eval_op_color(op_idx, is_side);
}

fn from_uvw(axis_id: u32, cu: f32, cv: f32, cw: f32) -> vec3<f32> {
    if (axis_id == 0u) {
        return vec3<f32>(cw, cu, cv);
    }
    if (axis_id == 1u) {
        return vec3<f32>(cu, cw, cv);
    }
    return vec3<f32>(cu, cv, cw);
}

fn neighbor_endpoint(
    plane_offset: u32,
    nu: u32,
    nv: u32,
    iu: i32,
    iv: i32,
    want_min: bool,
    ref_w: f32,
    max_dw: f32,
) -> f32 {
    if (iu < 0 || iv < 0 || u32(iu) >= nu || u32(iv) >= nv) {
        return ref_w;
    }
    let n_ray = plane_offset + u32(iv) * nu + u32(iu);
    let n_cnt = m_counts[n_ray];
    if (n_cnt == 0u || n_cnt > INLINE_K) {
        return ref_w;
    }
    let base = n_ray * INLINE_K;
    var best_d = max_dw + 1.0;
    var best_w = ref_w;
    for (var k = 0u; k < n_cnt; k = k + 1u) {
        let s = m_slots[base + k];
        let c = select(s.max_local, s.min_local, want_min);
        let d = abs(c - ref_w);
        if (d <= max_dw && d < best_d) {
            best_d = d;
            best_w = c;
        }
    }
    return best_w;
}

fn make_vertex(p: vec3<f32>, n: vec3<f32>, col: vec4<f32>) -> PackedSimVertex {
    return PackedSimVertex(
        p.x, p.y, p.z,
        n.x, n.y, n.z,
        col.x, col.y, col.z, col.w,
    );
}

fn emit_quad(
    p0: vec3<f32>,
    p1: vec3<f32>,
    p2: vec3<f32>,
    p3: vec3<f32>,
    n: vec3<f32>,
    col: vec4<f32>,
) {
    let cap = u.mode_and_cap.y;
    if (atomicLoad(&draw_args[0]) + 6u > cap) {
        return;
    }
    let base = atomicAdd(&draw_args[0], 6u);
    if (base + 6u > cap) {
        atomicMin(&draw_args[0], cap);
        return;
    }
    let v0 = make_vertex(p0, n, col);
    let v1 = make_vertex(p1, n, col);
    let v2 = make_vertex(p2, n, col);
    let v3 = make_vertex(p3, n, col);
    out_vertices[base + 0u] = v0;
    out_vertices[base + 1u] = v1;
    out_vertices[base + 2u] = v2;
    out_vertices[base + 3u] = v0;
    out_vertices[base + 4u] = v2;
    out_vertices[base + 5u] = v3;
}

fn emit_endpoint_face(
    axis_id: u32,
    plane_offset: u32,
    global_ray: u32,
    nu: u32,
    nv: u32,
    iu: u32,
    iv: u32,
    du: f32,
    dv: f32,
    dw: f32,
    ou: f32,
    ov: f32,
    ow: f32,
    w_local: f32,
    op_idx: u32,
    want_min: bool,
) {
    let max_dw = 1.5 * dw;
    let iiu = i32(iu);
    let iiv = i32(iv);
    let wl = neighbor_endpoint(plane_offset, nu, nv, iiu - 1, iiv, want_min, w_local, max_dw);
    let wr = neighbor_endpoint(plane_offset, nu, nv, iiu + 1, iiv, want_min, w_local, max_dw);
    let wd = neighbor_endpoint(plane_offset, nu, nv, iiu, iiv - 1, want_min, w_local, max_dw);
    let wu = neighbor_endpoint(plane_offset, nu, nv, iiu, iiv + 1, want_min, w_local, max_dw);

    let dw_du = (wr - wl) / (2.0 * du);
    let dw_dv = (wu - wd) / (2.0 * dv);

    var n_uvw = vec3<f32>(-dw_du, -dw_dv, 1.0);
    if (want_min) {
        n_uvw = vec3<f32>(dw_du, dw_dv, -1.0);
    }
    let n_wcs = normalize(from_uvw(axis_id, n_uvw.x, n_uvw.y, n_uvw.z));

    let is_side = axis_id != 2u;
    let col = eval_endpoint_color(global_ray, w_local, op_idx, is_side);

    let u0 = ou + f32(iu) * du;
    let u1 = ou + f32(iu + 1u) * du;
    let v0 = ov + f32(iv) * dv;
    let v1 = ov + f32(iv + 1u) * dv;
    let w_base = ow + w_local;

    let half_du = 0.5 * dw_du * du;
    let half_dv = 0.5 * dw_dv * dv;
    let clamp_w = 0.5 * dw;
    let w00 = w_base + clamp(-half_du - half_dv, -clamp_w, clamp_w);
    let w10 = w_base + clamp(half_du - half_dv, -clamp_w, clamp_w);
    let w11 = w_base + clamp(half_du + half_dv, -clamp_w, clamp_w);
    let w01 = w_base + clamp(-half_du + half_dv, -clamp_w, clamp_w);

    var a: vec3<f32>;
    var b: vec3<f32>;
    var c: vec3<f32>;
    var d: vec3<f32>;
    if (axis_id == 1u) {
        // Y plane has flipped handedness: (0,0), (0,1), (1,1), (1,0)
        a = from_uvw(axis_id, u0, v0, w00);
        b = from_uvw(axis_id, u0, v1, w01);
        c = from_uvw(axis_id, u1, v1, w11);
        d = from_uvw(axis_id, u1, v0, w10);
    } else {
        a = from_uvw(axis_id, u0, v0, w00);
        b = from_uvw(axis_id, u1, v0, w10);
        c = from_uvw(axis_id, u1, v1, w11);
        d = from_uvw(axis_id, u0, v1, w01);
    }

    if (want_min) {
        emit_quad(a, d, c, b, n_wcs, col);
    } else {
        emit_quad(a, b, c, d, n_wcs, col);
    }
}

@compute @workgroup_size(64, 1, 1)
fn main(
    @builtin(workgroup_id) wg_id: vec3<u32>,
    @builtin(num_workgroups) num_wg: vec3<u32>,
    @builtin(local_invocation_id) lid: vec3<u32>,
) {
    let flat_wg = wg_id.y * num_wg.x + wg_id.x;
    let global_ray = flat_wg * 64u + lid.x;
    if (global_ray >= u.dims_and_total.w) {
        return;
    }

    let cnt = m_counts[global_ray];
    // Empty rays emit nothing; rare CPU overflow rays (cnt > INLINE_K) are drawn by overflow_mesh.
    if (cnt == 0u || cnt > INLINE_K) {
        return;
    }

    let nx = u.dims_and_total.x;
    let ny = u.dims_and_total.y;
    let nz = u.dims_and_total.z;
    let off_y = u.plane_offsets_and_target.y;
    let off_z = u.plane_offsets_and_target.z;

    var axis_id: u32;
    var plane_offset: u32;
    var nu: u32;
    var nv: u32;
    var du: f32;
    var dv: f32;
    var dw: f32;
    var ou: f32;
    var ov: f32;
    var ow: f32;

    if (global_ray < off_y) {
        // X plane: u = Y, v = Z, w = X
        axis_id = 0u;
        plane_offset = u.plane_offsets_and_target.x;
        nu = ny;
        nv = nz;
        du = u.pitch_and_scale.y;
        dv = u.pitch_and_scale.z;
        dw = u.pitch_and_scale.x;
        ou = u.origin_and_tol.y;
        ov = u.origin_and_tol.z;
        ow = u.origin_and_tol.x;
    } else if (global_ray < off_z) {
        // Y plane: u = X, v = Z, w = Y
        axis_id = 1u;
        plane_offset = off_y;
        nu = nx;
        nv = nz;
        du = u.pitch_and_scale.x;
        dv = u.pitch_and_scale.z;
        dw = u.pitch_and_scale.y;
        ou = u.origin_and_tol.x;
        ov = u.origin_and_tol.z;
        ow = u.origin_and_tol.y;
    } else {
        // Z plane: u = X, v = Y, w = Z
        axis_id = 2u;
        plane_offset = off_z;
        nu = nx;
        nv = ny;
        du = u.pitch_and_scale.x;
        dv = u.pitch_and_scale.y;
        dw = u.pitch_and_scale.z;
        ou = u.origin_and_tol.x;
        ov = u.origin_and_tol.y;
        ow = u.origin_and_tol.z;
    }

    let local_ray = global_ray - plane_offset;
    let iu = local_ray % nu;
    let iv = local_ray / nu;
    let slot_base = global_ray * INLINE_K;

    for (var k = 0u; k < cnt; k = k + 1u) {
        let s = m_slots[slot_base + k];
        if (s.max_local <= s.min_local) {
            continue;
        }
        let op_min = unpack_min_op(s.op_packed);
        let op_max = unpack_max_op(s.op_packed);
        emit_endpoint_face(
            axis_id, plane_offset, global_ray, nu, nv, iu, iv, du, dv, dw, ou, ov, ow,
            s.max_local, op_max, false,
        );
        emit_endpoint_face(
            axis_id, plane_offset, global_ray, nu, nv, iu, iv, du, dv, dw, ou, ov, ow,
            s.min_local, op_min, true,
        );
    }
}
