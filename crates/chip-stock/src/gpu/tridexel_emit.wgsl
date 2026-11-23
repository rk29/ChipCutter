// Direct GPU Tri-Dexel Dual-Vertex Surface Nets Extraction Compute Pass (`tridexel_emit.wgsl`)
// Scans all 3 orthogonal dexel planes (`X`, `Y`, `Z`), resolves exact 3-axis majority-vote
// voxel boundary faces with cross-plane snapped dual vertices (zero overlapping-plane ribbon
// artifacts, zero sawtooth cylinder rims, watertight manifold geometry matching CPU Surface Nets),
// evaluates dominant-axis crease-aware smooth normals and `Metallic` / `ByOperation` / signed
// `ErrorHeatmap` coloring directly on GPU, and packs 24-byte `PackedTriDexelQuad` records for
// zero-readback `draw_indirect` expansion in `vs_sim_tridexel_gpu`.

const INLINE_K: u32 = 4u;
const UNCUT_OP_IDX: u32 = 0xFFFFFFFFu;
const UNCUT_OP_TAG_U16: u32 = 0xFFFFu;

const COLOR_MODE_METALLIC: u32 = 0u;
const COLOR_MODE_BY_OP: u32 = 1u;
const COLOR_MODE_HEATMAP: u32 = 2u;

const DUAL_OFFSET_CLAMP: f32 = 0.55;
const DUAL_OFFSET_PACK_SCALE: f32 = 0.6;

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

// Compact 24-byte watertight Surface-Nets quad record matching Rust `PackedTriDexelQuad`.
// Expanded directly into 6 vertices (2 triangles) by `vs_sim_tridexel_gpu` via `draw_indirect`.
struct PackedTriDexelQuad {
    // bits 0..20: global_ray, bits 20..30: dw voxel face index, bit 31: want_min
    ray_dw_flags: u32,
    // RGBA8 unorm packed color
    color_rgba8: u32,
    // snorm8x4: xyz = corner 0 (du, dv, dw) / 0.6, w = normal.x
    corner0_nx: u32,
    // snorm8x4: xyz = corner 1 (du, dv, dw) / 0.6, w = normal.y
    corner1_ny: u32,
    // snorm8x4: xyz = corner 2 (du, dv, dw) / 0.6, w = normal.z
    corner2_nz: u32,
    // snorm8x4: xyz = corner 3 (du, dv, dw) / 0.6, w = 0
    corner3_pad: u32,
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
    var out = lo;
    if (lo == UNCUT_OP_TAG_U16) {
        out = UNCUT_OP_IDX;
    }
    return out;
}

fn unpack_max_op(op_packed: u32) -> u32 {
    let hi = (op_packed >> 16u) & 0xFFFFu;
    var out = hi;
    if (hi == UNCUT_OP_TAG_U16) {
        out = UNCUT_OP_IDX;
    }
    return out;
}

fn eval_op_color(op_idx: u32, is_side: bool) -> vec4<f32> {
    var out = u.machined_metal_color;
    if (op_idx == UNCUT_OP_IDX) {
        out = select(u.raw_top_color, u.raw_side_color, is_side);
    } else if (u.mode_and_cap.x == COLOR_MODE_BY_OP) {
        out = u.op_palette[op_idx & 15u];
    }
    return out;
}

fn deviation_to_heatmap_rgba(signed_dev: f32, tol_in: f32, max_scale: f32) -> vec4<f32> {
    let tol = max(tol_in, 1e-4);
    let span = max(max_scale - tol, tol);
    let abs_d = abs(signed_dev);
    var out = vec4<f32>(0.18, 0.78, 0.36, 1.0);
    if (abs_d > tol) {
        let t = clamp((abs_d - tol) / span, 0.0, 1.0);
        if (signed_dev > 0.0) {
            out = vec4<f32>(
                0.22 + t * (0.12 - 0.22),
                0.80 + t * (0.28 - 0.80),
                0.94 + t * (0.88 - 0.94),
                1.0,
            );
        } else {
            out = vec4<f32>(
                0.98 + t * (0.94 - 0.98),
                0.78 + t * (0.16 - 0.78),
                0.16 + t * (0.20 - 0.16),
                1.0,
            );
        }
    }
    return out;
}

fn eval_signed_deviation(global_ray: u32, machined_coord: f32) -> f32 {
    let t_cnt = min(t_counts[global_ray], INLINE_K);
    var out = 10.0;
    if (t_cnt > 0u) {
        let t_base = global_ray * INLINE_K;
        var inside = false;
        var inside_dev = 0.0;
        for (var k = 0u; k < t_cnt; k = k + 1u) {
            let ts = t_slots[t_base + k];
            if (machined_coord > ts.min_local && machined_coord < ts.max_local) {
                inside = true;
                inside_dev = -min(machined_coord - ts.min_local, ts.max_local - machined_coord);
                break;
            }
        }
        if (inside) {
            out = inside_dev;
        } else {
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
            out = select(0.0, min_dist, min_dist < 1e20);
        }
    }
    return out;
}

fn eval_endpoint_color(global_ray: u32, w_local: f32, op_idx: u32, is_side: bool) -> vec4<f32> {
    var out = eval_op_color(op_idx, is_side);
    if (u.mode_and_cap.x == COLOR_MODE_HEATMAP && u.plane_offsets_and_target.w != 0u) {
        let dev = eval_signed_deviation(global_ray, w_local);
        out = deviation_to_heatmap_rgba(dev, u.origin_and_tol.w, u.pitch_and_scale.w);
    }
    return out;
}

fn plane_u_axis(ax: u32) -> u32 {
    return select(0u, 1u, ax == 0u);
}

fn plane_v_axis(ax: u32) -> u32 {
    return select(2u, 1u, ax == 2u);
}

fn plane_offset_of(ax: u32) -> u32 {
    var out = u.plane_offsets_and_target.z;
    if (ax == 0u) {
        out = u.plane_offsets_and_target.x;
    } else if (ax == 1u) {
        out = u.plane_offsets_and_target.y;
    }
    return out;
}

fn axis_dim(ax: u32) -> u32 {
    var out = u.dims_and_total.z;
    if (ax == 0u) {
        out = u.dims_and_total.x;
    } else if (ax == 1u) {
        out = u.dims_and_total.y;
    }
    return out;
}

fn axis_pitch(ax: u32) -> f32 {
    var out = u.pitch_and_scale.z;
    if (ax == 0u) {
        out = u.pitch_and_scale.x;
    } else if (ax == 1u) {
        out = u.pitch_and_scale.y;
    }
    return out;
}

fn from_uvw_i32(ax: u32, cu: i32, cv: i32, cw: i32) -> vec3<i32> {
    var out = vec3<i32>(cu, cv, cw);
    if (ax == 0u) {
        out = vec3<i32>(cw, cu, cv);
    } else if (ax == 1u) {
        out = vec3<i32>(cu, cw, cv);
    }
    return out;
}

fn from_uvw_f32(ax: u32, cu: f32, cv: f32, cw: f32) -> vec3<f32> {
    var out = vec3<f32>(cu, cv, cw);
    if (ax == 0u) {
        out = vec3<f32>(cw, cu, cv);
    } else if (ax == 1u) {
        out = vec3<f32>(cu, cw, cv);
    }
    return out;
}

fn axis_get_i32(v: vec3<i32>, ax: u32) -> i32 {
    var out = v.z;
    if (ax == 0u) {
        out = v.x;
    } else if (ax == 1u) {
        out = v.y;
    }
    return out;
}

fn axis_get_f32(v: vec3<f32>, ax: u32) -> f32 {
    var out = v.z;
    if (ax == 0u) {
        out = v.x;
    } else if (ax == 1u) {
        out = v.y;
    }
    return out;
}

fn ray_contains_local(global_ray: u32, w_local: f32) -> bool {
    let cnt = min(m_counts[global_ray], INLINE_K);
    let base = global_ray * INLINE_K;
    var hit = false;
    for (var k = 0u; k < cnt; k = k + 1u) {
        let s = m_slots[base + k];
        if (w_local >= s.min_local - 1e-5 && w_local <= s.max_local + 1e-5) {
            hit = true;
            break;
        }
    }
    return hit;
}

fn is_voxel_solid_xyz(ix: i32, iy: i32, iz: i32) -> bool {
    let nx = i32(u.dims_and_total.x);
    let ny = i32(u.dims_and_total.y);
    let nz = i32(u.dims_and_total.z);
    var solid = false;
    if (ix >= 0 && iy >= 0 && iz >= 0 && ix < nx && iy < ny && iz < nz) {
        let cx = (f32(ix) + 0.5) * u.pitch_and_scale.x;
        let cy = (f32(iy) + 0.5) * u.pitch_and_scale.y;
        let cz = (f32(iz) + 0.5) * u.pitch_and_scale.z;
        var votes = 0u;
        let rx = u.plane_offsets_and_target.x + u32(iz) * u.dims_and_total.y + u32(iy);
        if (ray_contains_local(rx, cx)) {
            votes = votes + 1u;
        }
        let ry = u.plane_offsets_and_target.y + u32(iz) * u.dims_and_total.x + u32(ix);
        if (ray_contains_local(ry, cy)) {
            votes = votes + 1u;
        }
        let rz = u.plane_offsets_and_target.z + u32(iy) * u.dims_and_total.x + u32(ix);
        if (ray_contains_local(rz, cz)) {
            votes = votes + 1u;
        }
        solid = votes >= 2u;
    }
    return solid;
}

fn is_voxel_solid_uvw(axis_id: u32, iu: i32, iv: i32, iw: i32) -> bool {
    let p = from_uvw_i32(axis_id, iu, iv, iw);
    return is_voxel_solid_xyz(p.x, p.y, p.z);
}

struct EndpointQuery {
    found: bool,
    coord: f32,
}

fn nearest_endpoint_on_plane(
    ax: u32,
    iu: i32,
    iv: i32,
    want_min: bool,
    ref_w: f32,
    max_dw: f32,
) -> EndpointQuery {
    let nu = i32(axis_dim(plane_u_axis(ax)));
    let nv = i32(axis_dim(plane_v_axis(ax)));
    var out = EndpointQuery(false, ref_w);
    if (iu >= 0 && iv >= 0 && iu < nu && iv < nv) {
        let n_ray = plane_offset_of(ax) + u32(iv) * u32(nu) + u32(iu);
        let n_cnt = m_counts[n_ray];
        if (n_cnt > 0u && n_cnt <= INLINE_K) {
            let base = n_ray * INLINE_K;
            var best_d = max_dw + 1.0;
            for (var k = 0u; k < n_cnt; k = k + 1u) {
                let s = m_slots[base + k];
                if (s.max_local <= s.min_local) {
                    continue;
                }
                let c = select(s.max_local, s.min_local, want_min);
                let d = abs(c - ref_w);
                if (d <= max_dw && d < best_d) {
                    best_d = d;
                    out = EndpointQuery(true, c);
                }
            }
        }
    }
    return out;
}

fn occ18_bit(mask: u32, lu: u32, lv: u32, lw: u32) -> bool {
    let bit = (lw * 3u + lv) * 3u + lu;
    return ((mask >> bit) & 1u) != 0u;
}

// Computes the exact cross-plane Dual-Vertex Surface Nets sub-voxel offset `(off_u, off_v, off_w)`
// (in voxel units, clamped to `[-0.55, +0.55]`) for dual corner `(iu + ou_c, iv + ov_c, dw)`
// reading the pre-cached 3×3×2 voxel occupancy bitmask `occ_mask` (zero redundant occupancy queries).
fn compute_dual_corner_offset(
    axis_id: u32,
    iu: i32,
    iv: i32,
    dw: i32,
    ou_c: u32,
    ov_c: u32,
    occ_mask: u32,
) -> vec3<f32> {
    let u_ax = plane_u_axis(axis_id);
    let v_ax = plane_v_axis(axis_id);
    let pu = axis_pitch(u_ax);
    let pv = axis_pitch(v_ax);
    let pw = axis_pitch(axis_id);

    // 1. W-axis crossings along `axis_id`
    let base_w = f32(dw) * pw;
    let max_dw = 1.05 * pw;
    var w_sum = 0.0;
    var w_cnt = 0.0;
    for (var ov = 0u; ov <= 1u; ov = ov + 1u) {
        let lv = ov_c + ov;
        let qv = iv + i32(lv) - 1;
        for (var ou = 0u; ou <= 1u; ou = ou + 1u) {
            let lu = ou_c + ou;
            let lo = occ18_bit(occ_mask, lu, lv, 0u);
            let hi = occ18_bit(occ_mask, lu, lv, 1u);
            if (lo != hi) {
                let qu = iu + i32(lu) - 1;
                let q = nearest_endpoint_on_plane(axis_id, qu, qv, !lo && hi, base_w, max_dw);
                if (q.found) {
                    w_sum = w_sum + q.coord;
                    w_cnt = w_cnt + 1.0;
                }
            }
        }
    }
    let off_w = select(
        0.0,
        clamp((w_sum / w_cnt - base_w) / pw, -DUAL_OFFSET_CLAMP, DUAL_OFFSET_CLAMP),
        w_cnt > 0.0,
    );

    // 2. U-axis crossings along `u_ax`
    let base_u = f32(iu + i32(ou_c)) * pu;
    let max_du = 1.05 * pu;
    let u_u_ax = plane_u_axis(u_ax);
    let u_v_ax = plane_v_axis(u_ax);
    var u_sum = 0.0;
    var u_cnt = 0.0;
    for (var lw = 0u; lw <= 1u; lw = lw + 1u) {
        let qw = dw + i32(lw) - 1;
        for (var ov = 0u; ov <= 1u; ov = ov + 1u) {
            let lv = ov_c + ov;
            let lo = occ18_bit(occ_mask, ou_c, lv, lw);
            let hi = occ18_bit(occ_mask, ou_c + 1u, lv, lw);
            if (lo != hi) {
                let vox = from_uvw_i32(axis_id, iu + i32(ou_c), iv + i32(lv) - 1, qw);
                let ru = axis_get_i32(vox, u_u_ax);
                let rv = axis_get_i32(vox, u_v_ax);
                let q = nearest_endpoint_on_plane(u_ax, ru, rv, !lo && hi, base_u, max_du);
                if (q.found) {
                    u_sum = u_sum + q.coord;
                    u_cnt = u_cnt + 1.0;
                }
            }
        }
    }
    let off_u = select(
        0.0,
        clamp((u_sum / u_cnt - base_u) / pu, -DUAL_OFFSET_CLAMP, DUAL_OFFSET_CLAMP),
        u_cnt > 0.0,
    );

    // 3. V-axis crossings along `v_ax`
    let base_v = f32(iv + i32(ov_c)) * pv;
    let max_dv = 1.05 * pv;
    let v_u_ax = plane_u_axis(v_ax);
    let v_v_ax = plane_v_axis(v_ax);
    var v_sum = 0.0;
    var v_cnt = 0.0;
    for (var lw = 0u; lw <= 1u; lw = lw + 1u) {
        let qw = dw + i32(lw) - 1;
        for (var ou = 0u; ou <= 1u; ou = ou + 1u) {
            let lu = ou_c + ou;
            let lo = occ18_bit(occ_mask, lu, ov_c, lw);
            let hi = occ18_bit(occ_mask, lu, ov_c + 1u, lw);
            if (lo != hi) {
                let vox = from_uvw_i32(axis_id, iu + i32(lu) - 1, iv + i32(ov_c), qw);
                let ru = axis_get_i32(vox, v_u_ax);
                let rv = axis_get_i32(vox, v_v_ax);
                let q = nearest_endpoint_on_plane(v_ax, ru, rv, !lo && hi, base_v, max_dv);
                if (q.found) {
                    v_sum = v_sum + q.coord;
                    v_cnt = v_cnt + 1.0;
                }
            }
        }
    }
    let off_v = select(
        0.0,
        clamp((v_sum / v_cnt - base_v) / pv, -DUAL_OFFSET_CLAMP, DUAL_OFFSET_CLAMP),
        v_cnt > 0.0,
    );

    return vec3<f32>(off_u, off_v, off_w);
}

// Evaluates a crease-aware smooth surface normal on the dominant dexel plane of `n_face` so
// curved cylinders, spheres, domes, and helical flutes shade seamlessly across X/Y/Z face seams
// while sharp 90° edges fall back crisply to `n_face`.
fn compute_crease_aware_normal(center_local: vec3<f32>, n_face: vec3<f32>) -> vec3<f32> {
    let ax = abs(n_face.x);
    let ay = abs(n_face.y);
    let az = abs(n_face.z);
    var dom_ax = 2u;
    if (ax >= ay && ax >= az) {
        dom_ax = 0u;
    } else if (ay >= az) {
        dom_ax = 1u;
    }

    let dom_u_ax = plane_u_axis(dom_ax);
    let dom_v_ax = plane_v_axis(dom_ax);
    let du = axis_pitch(dom_u_ax);
    let dv = axis_pitch(dom_v_ax);
    let dw = axis_pitch(dom_ax);
    let nu = i32(axis_dim(dom_u_ax));
    let nv = i32(axis_dim(dom_v_ax));

    let diu = clamp(i32(floor(axis_get_f32(center_local, dom_u_ax) / du)), 0, nu - 1);
    let div = clamp(i32(floor(axis_get_f32(center_local, dom_v_ax) / dv)), 0, nv - 1);
    let cw = axis_get_f32(center_local, dom_ax);
    let dom_want_min = axis_get_f32(n_face, dom_ax) < 0.0;

    var out_n = n_face;
    let qc = nearest_endpoint_on_plane(dom_ax, diu, div, dom_want_min, cw, 1.35 * dw);
    if (qc.found) {
        let max_d = 1.35 * dw;
        let ql = nearest_endpoint_on_plane(dom_ax, diu - 1, div, dom_want_min, qc.coord, max_d);
        let qr = nearest_endpoint_on_plane(dom_ax, diu + 1, div, dom_want_min, qc.coord, max_d);
        let qd = nearest_endpoint_on_plane(dom_ax, diu, div - 1, dom_want_min, qc.coord, max_d);
        let qu = nearest_endpoint_on_plane(dom_ax, diu, div + 1, dom_want_min, qc.coord, max_d);

        var dw_du = 0.0;
        if (ql.found && qr.found) {
            dw_du = (qr.coord - ql.coord) / (2.0 * du);
        } else if (qr.found) {
            dw_du = (qr.coord - qc.coord) / du;
        } else if (ql.found) {
            dw_du = (qc.coord - ql.coord) / du;
        }

        var dw_dv = 0.0;
        if (qd.found && qu.found) {
            dw_dv = (qu.coord - qd.coord) / (2.0 * dv);
        } else if (qu.found) {
            dw_dv = (qu.coord - qc.coord) / dv;
        } else if (qd.found) {
            dw_dv = (qc.coord - qd.coord) / dv;
        }

        var n_uvw = vec3<f32>(-dw_du, -dw_dv, 1.0);
        if (dom_want_min) {
            n_uvw = vec3<f32>(dw_du, dw_dv, -1.0);
        }
        let n_smooth = normalize(from_uvw_f32(dom_ax, n_uvw.x, n_uvw.y, n_uvw.z));
        if (dot(n_smooth, n_face) >= 0.72) {
            out_n = n_smooth;
        }
    }
    return out_n;
}

fn emit_surface_net_quad(
    axis_id: u32,
    global_ray: u32,
    iu: u32,
    iv: u32,
    dw_idx: u32,
    w_local: f32,
    op_idx: u32,
    want_min: bool,
) {
    let max_verts = u.mode_and_cap.y * 6u;
    if (atomicLoad(&draw_args[0]) + 6u > max_verts) {
        return;
    }

    let iiu = i32(iu);
    let iiv = i32(iv);
    let idw = i32(dw_idx);

    // Cache 3×3×2 voxel occupancy around `(iu, iv, dw)` in a single 18-bit register bitmask
    var occ_mask = 0u;
    for (var lw = 0u; lw < 2u; lw = lw + 1u) {
        let vw = idw + i32(lw) - 1;
        for (var lv = 0u; lv < 3u; lv = lv + 1u) {
            let vv = iiv + i32(lv) - 1;
            for (var lu = 0u; lu < 3u; lu = lu + 1u) {
                let vu = iiu + i32(lu) - 1;
                if (is_voxel_solid_uvw(axis_id, vu, vv, vw)) {
                    let bit = (lw * 3u + lv) * 3u + lu;
                    occ_mask = occ_mask | (1u << bit);
                }
            }
        }
    }

    // Quad corner layout matching `Axis3::quad_corner_offsets()` in Rust `tridexel/mesher.rs`
    var ou0 = 0u; var ov0 = 0u;
    var ou1 = 1u; var ov1 = 0u;
    var ou2 = 1u; var ov2 = 1u;
    var ou3 = 0u; var ov3 = 1u;
    if (axis_id == 1u) {
        ou1 = 0u; ov1 = 1u;
        ou3 = 1u; ov3 = 0u;
    }

    let off0 = compute_dual_corner_offset(axis_id, iiu, iiv, idw, ou0, ov0, occ_mask);
    let off1 = compute_dual_corner_offset(axis_id, iiu, iiv, idw, ou1, ov1, occ_mask);
    let off2 = compute_dual_corner_offset(axis_id, iiu, iiv, idw, ou2, ov2, occ_mask);
    let off3 = compute_dual_corner_offset(axis_id, iiu, iiv, idw, ou3, ov3, occ_mask);

    let pu = axis_pitch(plane_u_axis(axis_id));
    let pv = axis_pitch(plane_v_axis(axis_id));
    let pw = axis_pitch(axis_id);

    let p0 = from_uvw_f32(
        axis_id,
        (f32(iu + ou0) + off0.x) * pu,
        (f32(iv + ov0) + off0.y) * pv,
        (f32(dw_idx) + off0.z) * pw,
    );
    let p1 = from_uvw_f32(
        axis_id,
        (f32(iu + ou1) + off1.x) * pu,
        (f32(iv + ov1) + off1.y) * pv,
        (f32(dw_idx) + off1.z) * pw,
    );
    let p2 = from_uvw_f32(
        axis_id,
        (f32(iu + ou2) + off2.x) * pu,
        (f32(iv + ov2) + off2.y) * pv,
        (f32(dw_idx) + off2.z) * pw,
    );
    let p3 = from_uvw_f32(
        axis_id,
        (f32(iu + ou3) + off3.x) * pu,
        (f32(iv + ov3) + off3.y) * pv,
        (f32(dw_idx) + off3.z) * pw,
    );

    var n_raw = cross(p2 - p0, p3 - p1);
    if (want_min) {
        n_raw = -n_raw;
    }
    let n_len = length(n_raw);
    let fallback_n = from_uvw_f32(axis_id, 0.0, 0.0, select(1.0, -1.0, want_min));
    let n_face = select(fallback_n, n_raw / n_len, n_len > 1e-6);

    let center_local = 0.25 * (p0 + p1 + p2 + p3);
    let n_final = compute_crease_aware_normal(center_local, n_face);

    let is_side = axis_id != 2u;
    let col = eval_endpoint_color(global_ray, w_local, op_idx, is_side);

    let base_v = atomicAdd(&draw_args[0], 6u);
    if (base_v + 6u > max_verts) {
        atomicMin(&draw_args[0], max_verts);
        return;
    }
    let quad_idx = base_v / 6u;
    let inv_scale = 1.0 / DUAL_OFFSET_PACK_SCALE;
    let ray_dw_flags = (global_ray & 0xFFFFFu)
        | ((dw_idx & 0x3FFu) << 20u)
        | select(0u, 0x80000000u, want_min);
    out_quads[quad_idx] = PackedTriDexelQuad(
        ray_dw_flags,
        pack4x8unorm(col),
        pack4x8snorm(vec4<f32>(off0 * inv_scale, n_final.x)),
        pack4x8snorm(vec4<f32>(off1 * inv_scale, n_final.y)),
        pack4x8snorm(vec4<f32>(off2 * inv_scale, n_final.z)),
        pack4x8snorm(vec4<f32>(off3 * inv_scale, 0.0)),
    );
}

fn find_boundary_dw(
    axis_id: u32,
    iu: i32,
    iv: i32,
    nw: i32,
    dw_est: i32,
    want_min: bool,
    last_dw: i32,
) -> i32 {
    var offsets = array<i32, 3>(0, -1, 1);
    var found = -1;
    for (var i = 0u; i < 3u; i = i + 1u) {
        let cand = dw_est + offsets[i];
        if (cand < 0 || cand > nw || cand == last_dw) {
            continue;
        }
        let lo = is_voxel_solid_uvw(axis_id, iu, iv, cand - 1);
        let hi = is_voxel_solid_uvw(axis_id, iu, iv, cand);
        let matches = select(lo && !hi, !lo && hi, want_min);
        if (matches) {
            found = cand;
            break;
        }
    }
    return found;
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
    var nw: u32;
    var dw_pitch: f32;

    if (global_ray < off_y) {
        // X plane: u = Y, v = Z, w = X
        axis_id = 0u;
        plane_offset = u.plane_offsets_and_target.x;
        nu = ny;
        nw = nx;
        dw_pitch = u.pitch_and_scale.x;
    } else if (global_ray < off_z) {
        // Y plane: u = X, v = Z, w = Y
        axis_id = 1u;
        plane_offset = off_y;
        nu = nx;
        nw = ny;
        dw_pitch = u.pitch_and_scale.y;
    } else {
        // Z plane: u = X, v = Y, w = Z
        axis_id = 2u;
        plane_offset = off_z;
        nu = nx;
        nw = nz;
        dw_pitch = u.pitch_and_scale.z;
    }

    let local_ray = global_ray - plane_offset;
    let iu = local_ray % nu;
    let iv = local_ray / nu;
    let slot_base = global_ray * INLINE_K;
    let iiu = i32(iu);
    let iiv = i32(iv);
    let inw = i32(nw);
    var last_dw = -1;

    for (var k = 0u; k < cnt; k = k + 1u) {
        let s = m_slots[slot_base + k];
        if (s.max_local <= s.min_local) {
            continue;
        }
        let op_min = unpack_min_op(s.op_packed);
        let op_max = unpack_max_op(s.op_packed);

        let dw_min_est = clamp(i32(round(s.min_local / dw_pitch)), 0, inw);
        let dw_min = find_boundary_dw(axis_id, iiu, iiv, inw, dw_min_est, true, last_dw);
        if (dw_min >= 0) {
            emit_surface_net_quad(
                axis_id, global_ray, iu, iv, u32(dw_min), s.min_local, op_min, true,
            );
            last_dw = dw_min;
        }

        let dw_max_est = clamp(i32(round(s.max_local / dw_pitch)), 0, inw);
        let dw_max = find_boundary_dw(axis_id, iiu, iiv, inw, dw_max_est, false, last_dw);
        if (dw_max >= 0) {
            emit_surface_net_quad(
                axis_id, global_ray, iu, iv, u32(dw_max), s.max_local, op_max, false,
            );
            last_dw = dw_max;
        }
    }
}
