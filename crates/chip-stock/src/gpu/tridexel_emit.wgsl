// Direct GPU Tri-Dexel Dual-Vertex Surface Nets Extraction Compute Pass (`tridexel_emit.wgsl`)
// Scans all 3 orthogonal dexel planes (`X`, `Y`, `Z`), resolves exact 3-axis majority-vote
// voxel boundary faces with cross-plane snapped dual vertices (zero overlapping-plane ribbon
// artifacts, zero sawtooth cylinder rims, watertight manifold geometry matching CPU Surface Nets),
// evaluates dominant-axis crease-aware smooth normals and `Metallic` / `ByOperation` / signed
// `ErrorHeatmap` coloring directly on GPU, and packs 24-byte `PackedTriDexelQuad` records for
// zero-readback `draw_indirect` expansion in `vs_sim_tridexel_gpu`.
//
// Structured to keep statically inlined loop count and nesting depth compact for Windows FXC
// (`D3DCompile` SM 5.0) compatibility: fixed-`K` (`K = 4`) ray lookups are straight-line checks
// (zero inner loops), dual-cell edge scans use a single flat 12-edge loop, and `main` processes
// both span endpoints through a single unified `emit_surface_net_quad` call site.

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
        var min_dist = 1e30;

        let ts0 = t_slots[t_base];
        if (machined_coord > ts0.min_local && machined_coord < ts0.max_local) {
            inside = true;
            inside_dev = -min(machined_coord - ts0.min_local, ts0.max_local - machined_coord);
        } else {
            let d0 = select(
                machined_coord - ts0.max_local,
                ts0.min_local - machined_coord,
                machined_coord <= ts0.min_local,
            );
            min_dist = min(min_dist, d0);
        }
        if (!inside && t_cnt > 1u) {
            let ts1 = t_slots[t_base + 1u];
            if (machined_coord > ts1.min_local && machined_coord < ts1.max_local) {
                inside = true;
                inside_dev = -min(machined_coord - ts1.min_local, ts1.max_local - machined_coord);
            } else {
                let d1 = select(
                    machined_coord - ts1.max_local,
                    ts1.min_local - machined_coord,
                    machined_coord <= ts1.min_local,
                );
                min_dist = min(min_dist, d1);
            }
        }
        if (!inside && t_cnt > 2u) {
            let ts2 = t_slots[t_base + 2u];
            if (machined_coord > ts2.min_local && machined_coord < ts2.max_local) {
                inside = true;
                inside_dev = -min(machined_coord - ts2.min_local, ts2.max_local - machined_coord);
            } else {
                let d2 = select(
                    machined_coord - ts2.max_local,
                    ts2.min_local - machined_coord,
                    machined_coord <= ts2.min_local,
                );
                min_dist = min(min_dist, d2);
            }
        }
        if (!inside && t_cnt > 3u) {
            let ts3 = t_slots[t_base + 3u];
            if (machined_coord > ts3.min_local && machined_coord < ts3.max_local) {
                inside = true;
                inside_dev = -min(machined_coord - ts3.min_local, ts3.max_local - machined_coord);
            } else {
                let d3 = select(
                    machined_coord - ts3.max_local,
                    ts3.min_local - machined_coord,
                    machined_coord <= ts3.min_local,
                );
                min_dist = min(min_dist, d3);
            }
        }

        if (inside) {
            out = inside_dev;
        } else {
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

// Straight-line fixed-K (`K = 4`) point-in-ray test (zero inner loops for fast FXC compile).
fn ray_contains_local(global_ray: u32, w_local: f32) -> bool {
    let cnt = min(m_counts[global_ray], INLINE_K);
    let base = global_ray * INLINE_K;
    var hit = false;
    if (cnt > 0u) {
        let s0 = m_slots[base];
        hit = (w_local >= s0.min_local - 1e-5 && w_local <= s0.max_local + 1e-5);
    }
    if (!hit && cnt > 1u) {
        let s1 = m_slots[base + 1u];
        hit = (w_local >= s1.min_local - 1e-5 && w_local <= s1.max_local + 1e-5);
    }
    if (!hit && cnt > 2u) {
        let s2 = m_slots[base + 2u];
        hit = (w_local >= s2.min_local - 1e-5 && w_local <= s2.max_local + 1e-5);
    }
    if (!hit && cnt > 3u) {
        let s3 = m_slots[base + 3u];
        hit = (w_local >= s3.min_local - 1e-5 && w_local <= s3.max_local + 1e-5);
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

// Straight-line fixed-K (`K = 4`) nearest endpoint lookup (zero inner loops for fast FXC compile).
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

            let s0 = m_slots[base];
            if (s0.max_local > s0.min_local) {
                let c0 = select(s0.max_local, s0.min_local, want_min);
                let d0 = abs(c0 - ref_w);
                if (d0 <= max_dw && d0 < best_d) {
                    best_d = d0;
                    out = EndpointQuery(true, c0);
                }
            }
            if (n_cnt > 1u) {
                let s1 = m_slots[base + 1u];
                if (s1.max_local > s1.min_local) {
                    let c1 = select(s1.max_local, s1.min_local, want_min);
                    let d1 = abs(c1 - ref_w);
                    if (d1 <= max_dw && d1 < best_d) {
                        best_d = d1;
                        out = EndpointQuery(true, c1);
                    }
                }
            }
            if (n_cnt > 2u) {
                let s2 = m_slots[base + 2u];
                if (s2.max_local > s2.min_local) {
                    let c2 = select(s2.max_local, s2.min_local, want_min);
                    let d2 = abs(c2 - ref_w);
                    if (d2 <= max_dw && d2 < best_d) {
                        best_d = d2;
                        out = EndpointQuery(true, c2);
                    }
                }
            }
            if (n_cnt > 3u) {
                let s3 = m_slots[base + 3u];
                if (s3.max_local > s3.min_local) {
                    let c3 = select(s3.max_local, s3.min_local, want_min);
                    let d3 = abs(c3 - ref_w);
                    if (d3 <= max_dw && d3 < best_d) {
                        out = EndpointQuery(true, c3);
                    }
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
// reading the pre-cached 3×3×2 voxel occupancy bitmask `occ_mask` via a single flat 12-edge loop.
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

    let base_w = f32(dw) * pw;
    let base_u = f32(iu + i32(ou_c)) * pu;
    let base_v = f32(iv + i32(ov_c)) * pv;

    let u_u_ax = plane_u_axis(u_ax);
    let u_v_ax = plane_v_axis(u_ax);
    let v_u_ax = plane_u_axis(v_ax);
    let v_v_ax = plane_v_axis(v_ax);

    var w_sum = 0.0;
    var w_cnt = 0.0;
    var u_sum = 0.0;
    var u_cnt = 0.0;
    var v_sum = 0.0;
    var v_cnt = 0.0;

    // Flat 12-edge scan: e in 0..4 => W-axis, 4..8 => U-axis, 8..12 => V-axis
    for (var e = 0u; e < 12u; e = e + 1u) {
        var lo = false;
        var hi = false;
        var q_ax = axis_id;
        var ru = 0;
        var rv = 0;
        var ref_c = base_w;
        var max_d = 1.05 * pw;

        if (e < 4u) {
            let ou = e & 1u;
            let ov = (e >> 1u) & 1u;
            let lu = ou_c + ou;
            let lv = ov_c + ov;
            lo = occ18_bit(occ_mask, lu, lv, 0u);
            hi = occ18_bit(occ_mask, lu, lv, 1u);
            ru = iu + i32(lu) - 1;
            rv = iv + i32(lv) - 1;
        } else if (e < 8u) {
            let sub = e - 4u;
            let ov = sub & 1u;
            let lw = (sub >> 1u) & 1u;
            let lv = ov_c + ov;
            lo = occ18_bit(occ_mask, ou_c, lv, lw);
            hi = occ18_bit(occ_mask, ou_c + 1u, lv, lw);
            let qw = dw + i32(lw) - 1;
            let vox = from_uvw_i32(axis_id, iu + i32(ou_c), iv + i32(lv) - 1, qw);
            q_ax = u_ax;
            ru = axis_get_i32(vox, u_u_ax);
            rv = axis_get_i32(vox, u_v_ax);
            ref_c = base_u;
            max_d = 1.05 * pu;
        } else {
            let sub = e - 8u;
            let ou = sub & 1u;
            let lw = (sub >> 1u) & 1u;
            let lu = ou_c + ou;
            lo = occ18_bit(occ_mask, lu, ov_c, lw);
            hi = occ18_bit(occ_mask, lu, ov_c + 1u, lw);
            let qw = dw + i32(lw) - 1;
            let vox = from_uvw_i32(axis_id, iu + i32(lu) - 1, iv + i32(ov_c), qw);
            q_ax = v_ax;
            ru = axis_get_i32(vox, v_u_ax);
            rv = axis_get_i32(vox, v_v_ax);
            ref_c = base_v;
            max_d = 1.05 * pv;
        }

        if (lo != hi) {
            let q = nearest_endpoint_on_plane(q_ax, ru, rv, !lo && hi, ref_c, max_d);
            if (q.found) {
                if (e < 4u) {
                    w_sum = w_sum + q.coord;
                    w_cnt = w_cnt + 1.0;
                } else if (e < 8u) {
                    u_sum = u_sum + q.coord;
                    u_cnt = u_cnt + 1.0;
                } else {
                    v_sum = v_sum + q.coord;
                    v_cnt = v_cnt + 1.0;
                }
            }
        }
    }

    let off_w = select(
        0.0,
        clamp((w_sum / max(w_cnt, 1.0) - base_w) / pw, -DUAL_OFFSET_CLAMP, DUAL_OFFSET_CLAMP),
        w_cnt > 0.0,
    );
    let off_u = select(
        0.0,
        clamp((u_sum / max(u_cnt, 1.0) - base_u) / pu, -DUAL_OFFSET_CLAMP, DUAL_OFFSET_CLAMP),
        u_cnt > 0.0,
    );
    let off_v = select(
        0.0,
        clamp((v_sum / max(v_cnt, 1.0) - base_v) / pv, -DUAL_OFFSET_CLAMP, DUAL_OFFSET_CLAMP),
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
    let max_d = 1.35 * dw;
    let qc = nearest_endpoint_on_plane(dom_ax, diu, div, dom_want_min, cw, max_d);
    if (qc.found) {
        var ql = EndpointQuery(false, qc.coord);
        var qr = EndpointQuery(false, qc.coord);
        var qd = EndpointQuery(false, qc.coord);
        var qu = EndpointQuery(false, qc.coord);

        for (var ni = 0u; ni < 4u; ni = ni + 1u) {
            var qiu = diu;
            var qiv = div;
            if (ni == 0u) {
                qiu = diu - 1;
            } else if (ni == 1u) {
                qiu = diu + 1;
            } else if (ni == 2u) {
                qiv = div - 1;
            } else {
                qiv = div + 1;
            }
            let qn = nearest_endpoint_on_plane(dom_ax, qiu, qiv, dom_want_min, qc.coord, max_d);
            if (ni == 0u) {
                ql = qn;
            } else if (ni == 1u) {
                qr = qn;
            } else if (ni == 2u) {
                qd = qn;
            } else {
                qu = qn;
            }
        }

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

    // Cache 3×3×2 voxel occupancy around `(iu, iv, dw)` in a single flat 18-iteration loop.
    var occ_mask = 0u;
    for (var idx = 0u; idx < 18u; idx = idx + 1u) {
        let lw = select(0u, 1u, idx >= 9u);
        let rem = idx - lw * 9u;
        let lv = select(select(0u, 1u, rem >= 3u), 2u, rem >= 6u);
        let lu = rem - lv * 3u;
        let vu = iiu + i32(lu) - 1;
        let vv = iiv + i32(lv) - 1;
        let vw = idw + i32(lw) - 1;
        if (is_voxel_solid_uvw(axis_id, vu, vv, vw)) {
            occ_mask = occ_mask | (1u << idx);
        }
    }

    let pu = axis_pitch(plane_u_axis(axis_id));
    let pv = axis_pitch(plane_v_axis(axis_id));
    let pw = axis_pitch(axis_id);

    // Quad corner layout matching `Axis3::quad_corner_offsets()` in Rust `tridexel/mesher.rs`.
    // Single 4-iteration loop avoids 4× inlining of `compute_dual_corner_offset` in FXC.
    var off0 = vec3<f32>(0.0);
    var off1 = vec3<f32>(0.0);
    var off2 = vec3<f32>(0.0);
    var off3 = vec3<f32>(0.0);
    var p0 = vec3<f32>(0.0);
    var p1 = vec3<f32>(0.0);
    var p2 = vec3<f32>(0.0);
    var p3 = vec3<f32>(0.0);

    for (var ci = 0u; ci < 4u; ci = ci + 1u) {
        var ouc = 0u;
        var ovc = 0u;
        if (ci == 1u) {
            ouc = select(1u, 0u, axis_id == 1u);
            ovc = select(0u, 1u, axis_id == 1u);
        } else if (ci == 2u) {
            ouc = 1u;
            ovc = 1u;
        } else if (ci == 3u) {
            ouc = select(0u, 1u, axis_id == 1u);
            ovc = select(1u, 0u, axis_id == 1u);
        }
        let off = compute_dual_corner_offset(axis_id, iiu, iiv, idw, ouc, ovc, occ_mask);
        let pos = from_uvw_f32(
            axis_id,
            (f32(iu + ouc) + off.x) * pu,
            (f32(iv + ovc) + off.y) * pv,
            (f32(dw_idx) + off.z) * pw,
        );
        if (ci == 0u) {
            off0 = off;
            p0 = pos;
        } else if (ci == 1u) {
            off1 = off;
            p1 = pos;
        } else if (ci == 2u) {
            off2 = off;
            p2 = pos;
        } else {
            off3 = off;
            p3 = pos;
        }
    }

    var n_raw = cross(p2 - p0, p3 - p1);
    if (want_min) {
        n_raw = -n_raw;
    }
    let n_len = length(n_raw);
    let fallback_n = from_uvw_f32(axis_id, 0.0, 0.0, select(1.0, -1.0, want_min));
    let n_face = select(fallback_n, n_raw / max(n_len, 1e-6), n_len > 1e-6);

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
    var found = -1;
    for (var i = 0u; i < 3u; i = i + 1u) {
        let offset = select(select(0, -1, i == 1u), 1, i == 2u);
        let cand = dw_est + offset;
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

    let safe_nu = max(nu, 1u);
    let local_ray = global_ray - plane_offset;
    let iu = local_ray % safe_nu;
    let iv = local_ray / safe_nu;
    let slot_base = global_ray * INLINE_K;
    let iiu = i32(iu);
    let iiv = i32(iv);
    let inw = i32(nw);
    var last_dw = -1;

    // Single flat loop over `(span_idx, side)` keeps `emit_surface_net_quad` at 1 call site.
    let total_sides = cnt * 2u;
    for (var ks = 0u; ks < total_sides; ks = ks + 1u) {
        let k = ks >> 1u;
        let want_min = (ks & 1u) == 0u;
        let s = m_slots[slot_base + k];
        if (s.max_local <= s.min_local) {
            continue;
        }
        let w_local = select(s.max_local, s.min_local, want_min);
        let op_idx = select(unpack_max_op(s.op_packed), unpack_min_op(s.op_packed), want_min);
        let dw_est = clamp(i32(round(w_local / dw_pitch)), 0, inw);
        let dw = find_boundary_dw(axis_id, iiu, iiv, inw, dw_est, want_min, last_dw);
        if (dw >= 0) {
            emit_surface_net_quad(
                axis_id, global_ray, iu, iv, u32(dw), w_local, op_idx, want_min,
            );
            last_dw = dw;
        }
    }
}
