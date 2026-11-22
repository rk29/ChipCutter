// Direct GPU Tri-Dexel Surface Extraction Compute Pass (`tridexel_emit.wgsl`)
// Scans all 3 orthogonal dexel planes (`X`, `Y`, `Z`), emits compact 20-byte `PackedTriDexelQuad`
// records with finite-difference slopes and packed RGBA8 colors (`Metallic`, `ByOperation`, or
// signed `ErrorHeatmap`), and populates `DrawIndirectArgs` (`[vertex_count, 1, 0, 0]`) for
// zero-readback `draw_indirect` expansion in `vs_sim_tridexel_gpu`.

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
    // [color_mode_id, max_quads, _pad0, _pad1]
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

// Compact 20-byte quad record matching Rust `PackedTriDexelQuad` (align 4).
// Expanded directly into 6 vertices (2 triangles) by `vs_sim_tridexel_gpu` via `draw_indirect`.
struct PackedTriDexelQuad {
    // bits 0..30: global_ray, bit 31: want_min (1 = min endpoint, 0 = max endpoint)
    ray_and_flags: u32,
    // RGBA8 unorm packed color
    color_rgba8: u32,
    // Endpoint coordinate relative to plane origin_w (mm)
    w_local: f32,
    // Finite-difference slope dw/du
    dw_du: f32,
    // Finite-difference slope dw/dv
    dw_dv: f32,
}

// 1 uniform + 6 storage buffers (strictly within WebGPU default limit of 8 storage buffers/stage)
@group(0) @binding(0) var<uniform> u: TriDexelEmitUniforms;
@group(0) @binding(1) var<storage, read> m_slots: array<FlatDexelSpan>;
@group(0) @binding(2) var<storage, read> m_counts: array<u32>;
@group(0) @binding(3) var<storage, read> t_slots: array<FlatDexelSpan>;
@group(0) @binding(4) var<storage, read> t_counts: array<u32>;
@group(0) @binding(5) var<storage, read_write> out_quads: array<PackedTriDexelQuad>;
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

fn emit_packed_quad(
    global_ray: u32,
    want_min: bool,
    w_local: f32,
    dw_du: f32,
    dw_dv: f32,
    col: vec4<f32>,
) {
    let max_verts = u.mode_and_cap.y * 6u;
    if (atomicLoad(&draw_args[0]) + 6u > max_verts) {
        return;
    }
    let base_v = atomicAdd(&draw_args[0], 6u);
    if (base_v + 6u > max_verts) {
        atomicMin(&draw_args[0], max_verts);
        return;
    }
    let quad_idx = base_v / 6u;
    let ray_and_flags = (global_ray & 0x7FFFFFFFu) | select(0u, 0x80000000u, want_min);
    out_quads[quad_idx] = PackedTriDexelQuad(
        ray_and_flags,
        pack4x8unorm(col),
        w_local,
        dw_du,
        dw_dv,
    );
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

    let is_side = axis_id != 2u;
    let col = eval_endpoint_color(global_ray, w_local, op_idx, is_side);
    emit_packed_quad(global_ray, want_min, w_local, dw_du, dw_dv, col);
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

    if (global_ray < off_y) {
        // X plane: u = Y, v = Z, w = X
        axis_id = 0u;
        plane_offset = u.plane_offsets_and_target.x;
        nu = ny;
        nv = nz;
        du = u.pitch_and_scale.y;
        dv = u.pitch_and_scale.z;
        dw = u.pitch_and_scale.x;
    } else if (global_ray < off_z) {
        // Y plane: u = X, v = Z, w = Y
        axis_id = 1u;
        plane_offset = off_y;
        nu = nx;
        nv = nz;
        du = u.pitch_and_scale.x;
        dv = u.pitch_and_scale.z;
        dw = u.pitch_and_scale.y;
    } else {
        // Z plane: u = X, v = Y, w = Z
        axis_id = 2u;
        plane_offset = off_z;
        nu = nx;
        nv = ny;
        du = u.pitch_and_scale.x;
        dv = u.pitch_and_scale.y;
        dw = u.pitch_and_scale.z;
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
            axis_id, plane_offset, global_ray, nu, nv, iu, iv, du, dv, dw,
            s.max_local, op_max, false,
        );
        emit_endpoint_face(
            axis_id, plane_offset, global_ray, nu, nv, iu, iv, du, dv, dw,
            s.min_local, op_min, true,
        );
    }
}
