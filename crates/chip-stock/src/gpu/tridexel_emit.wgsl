// Direct GPU Tri-Dexel Dual-Vertex Surface Nets Extraction Compute Pass (`tridexel_emit.wgsl`)
// Scans all 3 orthogonal dexel planes (`X`, `Y`, `Z`), resolves exact 3-axis majority-vote
// voxel boundary faces with cross-plane snapped dual vertices (zero overlapping-plane ribbon
// artifacts, zero sawtooth cylinder rims, watertight manifold geometry matching CPU Surface Nets),
// supports optional crease-preserving Mesh Skinning (deterministic corner relaxation + per-corner
// smooth normals) and `Metallic` / `ByOperation` / signed `ErrorHeatmap` coloring directly on GPU,
// and packs 32-byte `PackedTriDexelQuad` records (12-bit octahedral per-corner normals) for
// zero-readback `draw_indirect` expansion in `vs_sim_tridexel_gpu`.
//
// Structured to keep statically inlined loop count and nesting depth compact for Windows FXC
// (`D3DCompile` SM 5.0) compatibility: fixed-`K` (`K = 4`) ray lookups are straight-line checks
// (zero inner loops), dual-cell edge scans use a single flat 12-edge loop, corner relaxation and
// per-corner normal evaluation are straight-line (zero loops), and `main` processes both span
// endpoints through a single unified `emit_surface_net_quad` call site.

const INLINE_K: u32 = 4u;
const UNCUT_OP_IDX: u32 = 0xFFFFFFFFu;
const UNCUT_OP_TAG_U16: u32 = 0xFFFFu;

const COLOR_MODE_METALLIC: u32 = 0u;
const COLOR_MODE_BY_OP: u32 = 1u;
const COLOR_MODE_HEATMAP: u32 = 2u;

const DUAL_OFFSET_CLAMP: f32 = 0.55;
const DUAL_RELAX_OFFSET_CLAMP: f32 = 0.98;
const DUAL_OFFSET_PACK_SCALE: f32 = 1.0;
const CREASE_COS_THRESHOLD: f32 = 0.788;
const MAX_RELAX_DISP_RATIO: f32 = 0.45;

struct TriDexelEmitUniforms {
    // [nx, ny, nz, total_rays]
    dims_and_total: vec4<u32>,
    // [ray_off_x, ray_off_y, ray_off_z, has_target]
    plane_offsets_and_target: vec4<u32>,
    // [color_mode_id, max_quads, mesh_skinning (0/1), _pad1]
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

// Compact 32-byte aligned watertight Surface-Nets quad record matching Rust `PackedTriDexelQuad`.
// Expanded directly into 6 vertices (2 triangles) by `vs_sim_tridexel_gpu` via `draw_indirect`.
struct PackedTriDexelQuad {
    // bits 0..20: global_ray, bits 20..30: dw voxel face index, bit 31: want_min
    ray_dw_flags: u32,
    // RGBA8 unorm packed color
    color_rgba8: u32,
    // snorm8x3 corner 0 offset (du, dv, dw) in bytes 0..3 + 4+4-bit octahedral refinement in byte 3
    corner0_pad: u32,
    // snorm8x3 corner 1 offset (du, dv, dw) in bytes 0..3 + 4+4-bit octahedral refinement in byte 3
    corner1_pad: u32,
    // snorm8x3 corner 2 offset (du, dv, dw) in bytes 0..3 + 4+4-bit octahedral refinement in byte 3
    corner2_pad: u32,
    // snorm8x3 corner 3 offset (du, dv, dw) in bytes 0..3 + 4+4-bit octahedral refinement in byte 3
    corner3_pad: u32,
    // High-8-bit octahedral-encoded unit normals for corners 0 & 1 (x0, y0, x1, y1)
    n01_oct: u32,
    // High-8-bit octahedral-encoded unit normals for corners 2 & 3 (x2, y2, x3, y3)
    n23_oct: u32,
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

fn oct_encode(n_in: vec3<f32>) -> vec2<f32> {
    let l1 = abs(n_in.x) + abs(n_in.y) + abs(n_in.z);
    var p = vec2<f32>(0.0, 0.0);
    if (l1 > 1e-6) {
        p = n_in.xy / l1;
        if (n_in.z < 0.0) {
            let sx = select(-1.0, 1.0, p.x >= 0.0);
            let sy = select(-1.0, 1.0, p.y >= 0.0);
            p = vec2<f32>((1.0 - abs(p.y)) * sx, (1.0 - abs(p.x)) * sy);
        }
    }
    return clamp(p, vec2<f32>(-1.0), vec2<f32>(1.0));
}

fn oct_quant_12(n_in: vec3<f32>) -> vec2<u32> {
    let e = oct_encode(n_in);
    let qx = u32(clamp(round((e.x + 1.0) * 2047.0), 0.0, 4094.0));
    let qy = u32(clamp(round((e.y + 1.0) * 2047.0), 0.0, 4094.0));
    return vec2<u32>(qx, qy);
}

fn pack_corner_with_oct_lo(off_uvw: vec3<f32>, q: vec2<u32>) -> u32 {
    let inv_scale = 1.0 / DUAL_OFFSET_PACK_SCALE;
    let xyz_word = pack4x8snorm(vec4<f32>(off_uvw * inv_scale, 0.0)) & 0x00FFFFFFu;
    let lo_byte = (q.x & 0xFu) | ((q.y & 0xFu) << 4u);
    return xyz_word | (lo_byte << 24u);
}

fn pack_oct_pair_hi8(qa: vec2<u32>, qb: vec2<u32>) -> u32 {
    return ((qa.x >> 4u) & 0xFFu)
        | (((qa.y >> 4u) & 0xFFu) << 8u)
        | (((qb.x >> 4u) & 0xFFu) << 16u)
        | (((qb.y >> 4u) & 0xFFu) << 24u);
}

// Deterministic, axis-independent, crease-preserving dual-corner relaxation (zero loops).
// Because `c_xyz`, the 8-voxel occupancy gradient `n_occ`, and `dom_ax` depend ONLY on the shared
// dual corner `c_xyz`, all adjacent quads across X/Y/Z seams emit bit-identical relaxed positions.
fn relax_dual_corner_uvw(
    axis_id: u32,
    c_xyz: vec3<i32>,
    off_uvw: vec3<f32>,
    ouc: u32,
    ovc: u32,
    occ_mask: u32,
) -> vec3<f32> {
    let nx = i32(u.dims_and_total.x);
    let ny = i32(u.dims_and_total.y);
    let nz = i32(u.dims_and_total.z);
    var out_off = off_uvw;

    let is_bnd = c_xyz.x <= 0 || c_xyz.x >= nx
        || c_xyz.y <= 0 || c_xyz.y >= ny
        || c_xyz.z <= 0 || c_xyz.z >= nz;
    if (!is_bnd) {
        let b000 = select(0.0, 1.0, occ18_bit(occ_mask, ouc, ovc, 0u));
        let b100 = select(0.0, 1.0, occ18_bit(occ_mask, ouc + 1u, ovc, 0u));
        let b010 = select(0.0, 1.0, occ18_bit(occ_mask, ouc, ovc + 1u, 0u));
        let b110 = select(0.0, 1.0, occ18_bit(occ_mask, ouc + 1u, ovc + 1u, 0u));
        let b001 = select(0.0, 1.0, occ18_bit(occ_mask, ouc, ovc, 1u));
        let b101 = select(0.0, 1.0, occ18_bit(occ_mask, ouc + 1u, ovc, 1u));
        let b011 = select(0.0, 1.0, occ18_bit(occ_mask, ouc, ovc + 1u, 1u));
        let b111 = select(0.0, 1.0, occ18_bit(occ_mask, ouc + 1u, ovc + 1u, 1u));

        let gu = (b000 + b010 + b001 + b011) - (b100 + b110 + b101 + b111);
        let gv = (b000 + b100 + b001 + b101) - (b010 + b110 + b011 + b111);
        let gw = (b000 + b100 + b010 + b110) - (b001 + b101 + b011 + b111);
        let n_occ = from_uvw_f32(axis_id, gu, gv, gw);
        let ax = abs(n_occ.x);
        let ay = abs(n_occ.y);
        let az = abs(n_occ.z);

        if (ax + ay + az > 0.5) {
            var dom_ax = 2u;
            if (ax >= ay && ax >= az) {
                dom_ax = 0u;
            } else if (ay >= az) {
                dom_ax = 1u;
            }
            let dom_u_ax = plane_u_axis(dom_ax);
            let dom_v_ax = plane_v_axis(dom_ax);
            let dom_du = axis_pitch(dom_u_ax);
            let dom_dv = axis_pitch(dom_v_ax);
            let dom_dw = axis_pitch(dom_ax);
            let cu = axis_get_i32(c_xyz, dom_u_ax);
            let cv = axis_get_i32(c_xyz, dom_v_ax);

            let pitch_xyz = u.pitch_and_scale.xyz;
            let raw_off_xyz = from_uvw_f32(axis_id, off_uvw.x, off_uvw.y, off_uvw.z);
            let pos_0 = (vec3<f32>(c_xyz) + raw_off_xyz) * pitch_xyz;
            let ref_w = axis_get_f32(pos_0, dom_ax);
            let dom_want_min = axis_get_f32(n_occ, dom_ax) < 0.0;
            let max_d = 1.35 * dom_dw;

            let q00 = nearest_endpoint_on_plane(dom_ax, cu - 1, cv - 1, dom_want_min, ref_w, max_d);
            let q10 = nearest_endpoint_on_plane(dom_ax, cu, cv - 1, dom_want_min, ref_w, max_d);
            let q01 = nearest_endpoint_on_plane(dom_ax, cu - 1, cv, dom_want_min, ref_w, max_d);
            let q11 = nearest_endpoint_on_plane(dom_ax, cu, cv, dom_want_min, ref_w, max_d);

            if (q00.found && q10.found && q01.found && q11.found) {
                let s_u_mid = 0.5 * ((q10.coord - q00.coord) + (q11.coord - q01.coord)) / dom_du;
                let s_v_mid = 0.5 * ((q01.coord - q00.coord) + (q11.coord - q10.coord)) / dom_dv;

                let q_ll = nearest_endpoint_on_plane(dom_ax, cu - 2, cv - 1, dom_want_min, q00.coord, max_d);
                let q_rr = nearest_endpoint_on_plane(dom_ax, cu + 1, cv - 1, dom_want_min, q10.coord, max_d);
                let q_dd = nearest_endpoint_on_plane(dom_ax, cu - 1, cv - 2, dom_want_min, q00.coord, max_d);
                let q_uu = nearest_endpoint_on_plane(dom_ax, cu - 1, cv + 1, dom_want_min, q01.coord, max_d);

                let s_u_l = select(s_u_mid, (q00.coord - q_ll.coord) / dom_du, q_ll.found);
                let s_u_r = select(s_u_mid, (q_rr.coord - q10.coord) / dom_du, q_rr.found);
                let s_v_d = select(s_v_mid, (q00.coord - q_dd.coord) / dom_dv, q_dd.found);
                let s_v_u = select(s_v_mid, (q_uu.coord - q01.coord) / dom_dv, q_uu.found);

                let n_ul = normalize(vec3<f32>(-s_u_l, -s_v_mid, 1.0));
                let n_ur = normalize(vec3<f32>(-s_u_r, -s_v_mid, 1.0));
                let n_vd = normalize(vec3<f32>(-s_u_mid, -s_v_d, 1.0));
                let n_vu = normalize(vec3<f32>(-s_u_mid, -s_v_u, 1.0));

                if (dot(n_ul, n_ur) >= CREASE_COS_THRESHOLD && dot(n_vd, n_vu) >= CREASE_COS_THRESHOLD) {
                    let w_smooth = 0.25 * (q00.coord + q10.coord + q01.coord + q11.coord);
                    let target_xyz = from_uvw_f32(dom_ax, f32(cu) * dom_du, f32(cv) * dom_dv, w_smooth);
                    var delta = 0.65 * (target_xyz - pos_0);
                    let min_p = min(pitch_xyz.x, min(pitch_xyz.y, pitch_xyz.z));
                    let max_d_len = MAX_RELAX_DISP_RATIO * min_p;
                    let d_len = length(delta);
                    if (d_len > max_d_len && d_len > 1e-6) {
                        delta = delta * (max_d_len / d_len);
                    }
                    let relaxed_pos = pos_0 + delta;
                    let relaxed_off_xyz = clamp(
                        relaxed_pos / pitch_xyz - vec3<f32>(c_xyz),
                        vec3<f32>(-DUAL_RELAX_OFFSET_CLAMP),
                        vec3<f32>(DUAL_RELAX_OFFSET_CLAMP),
                    );
                    out_off = vec3<f32>(
                        axis_get_f32(relaxed_off_xyz, plane_u_axis(axis_id)),
                        axis_get_f32(relaxed_off_xyz, plane_v_axis(axis_id)),
                        axis_get_f32(relaxed_off_xyz, axis_id),
                    );
                }
            }
        }
    }
    return out_off;
}

// Evaluates a crease-aware smooth surface normal directly at dual corner `c_xyz` using the
// symmetric 2×2 ray stencil on the dominant dexel plane of `n_face` (zero loops, FXC-safe).
fn compute_corner_smooth_normal(
    c_xyz: vec3<i32>,
    corner_local: vec3<f32>,
    n_face: vec3<f32>,
) -> vec3<f32> {
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
    let cu = axis_get_i32(c_xyz, dom_u_ax);
    let cv = axis_get_i32(c_xyz, dom_v_ax);
    let cw = axis_get_f32(corner_local, dom_ax);
    let dom_want_min = axis_get_f32(n_face, dom_ax) < 0.0;
    let max_d = 1.35 * dw;

    let q00 = nearest_endpoint_on_plane(dom_ax, cu - 1, cv - 1, dom_want_min, cw, max_d);
    let q10 = nearest_endpoint_on_plane(dom_ax, cu, cv - 1, dom_want_min, cw, max_d);
    let q01 = nearest_endpoint_on_plane(dom_ax, cu - 1, cv, dom_want_min, cw, max_d);
    let q11 = nearest_endpoint_on_plane(dom_ax, cu, cv, dom_want_min, cw, max_d);

    var u_sum = 0.0;
    var u_cnt = 0.0;
    if (q00.found && q10.found) {
        u_sum = u_sum + (q10.coord - q00.coord) / du;
        u_cnt = u_cnt + 1.0;
    }
    if (q01.found && q11.found) {
        u_sum = u_sum + (q11.coord - q01.coord) / du;
        u_cnt = u_cnt + 1.0;
    }

    var v_sum = 0.0;
    var v_cnt = 0.0;
    if (q00.found && q01.found) {
        v_sum = v_sum + (q01.coord - q00.coord) / dv;
        v_cnt = v_cnt + 1.0;
    }
    if (q10.found && q11.found) {
        v_sum = v_sum + (q11.coord - q10.coord) / dv;
        v_cnt = v_cnt + 1.0;
    }

    var out_n = n_face;
    if (u_cnt > 0.0 || v_cnt > 0.0) {
        let dw_du = select(0.0, u_sum / max(u_cnt, 1.0), u_cnt > 0.0);
        let dw_dv = select(0.0, v_sum / max(v_cnt, 1.0), v_cnt > 0.0);
        var n_uvw = vec3<f32>(-dw_du, -dw_dv, 1.0);
        if (dom_want_min) {
            n_uvw = vec3<f32>(dw_du, dw_dv, -1.0);
        }
        let n_smooth = normalize(from_uvw_f32(dom_ax, n_uvw.x, n_uvw.y, n_uvw.z));
        if (dot(n_smooth, n_face) >= CREASE_COS_THRESHOLD) {
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
    let skinning = u.mode_and_cap.z != 0u;

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
    // Single 4-iteration loop avoids 4× inlining of corner helpers in FXC.
    var off0 = vec3<f32>(0.0);
    var off1 = vec3<f32>(0.0);
    var off2 = vec3<f32>(0.0);
    var off3 = vec3<f32>(0.0);
    var p0 = vec3<f32>(0.0);
    var p1 = vec3<f32>(0.0);
    var p2 = vec3<f32>(0.0);
    var p3 = vec3<f32>(0.0);
    var c0_xyz = vec3<i32>(0);
    var c1_xyz = vec3<i32>(0);
    var c2_xyz = vec3<i32>(0);
    var c3_xyz = vec3<i32>(0);

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
        let c_xyz = from_uvw_i32(axis_id, iiu + i32(ouc), iiv + i32(ovc), idw);
        var off = compute_dual_corner_offset(axis_id, iiu, iiv, idw, ouc, ovc, occ_mask);
        if (skinning) {
            off = relax_dual_corner_uvw(axis_id, c_xyz, off, ouc, ovc, occ_mask);
        }
        let pos = from_uvw_f32(
            axis_id,
            (f32(iu + ouc) + off.x) * pu,
            (f32(iv + ovc) + off.y) * pv,
            (f32(dw_idx) + off.z) * pw,
        );
        if (ci == 0u) {
            off0 = off;
            p0 = pos;
            c0_xyz = c_xyz;
        } else if (ci == 1u) {
            off1 = off;
            p1 = pos;
            c1_xyz = c_xyz;
        } else if (ci == 2u) {
            off2 = off;
            p2 = pos;
            c2_xyz = c_xyz;
        } else {
            off3 = off;
            p3 = pos;
            c3_xyz = c_xyz;
        }
    }

    var n_raw = cross(p2 - p0, p3 - p1);
    if (want_min) {
        n_raw = -n_raw;
    }
    let n_len = length(n_raw);
    let fallback_n = from_uvw_f32(axis_id, 0.0, 0.0, select(1.0, -1.0, want_min));
    let n_face = select(fallback_n, n_raw / max(n_len, 1e-6), n_len > 1e-6);

    var n0 = n_face;
    var n1 = n_face;
    var n2 = n_face;
    var n3 = n_face;
    if (skinning) {
        for (var ci = 0u; ci < 4u; ci = ci + 1u) {
            var c_xyz = c0_xyz;
            var pos = p0;
            if (ci == 1u) {
                c_xyz = c1_xyz;
                pos = p1;
            } else if (ci == 2u) {
                c_xyz = c2_xyz;
                pos = p2;
            } else if (ci == 3u) {
                c_xyz = c3_xyz;
                pos = p3;
            }
            let nc = compute_corner_smooth_normal(c_xyz, pos, n_face);
            if (ci == 0u) {
                n0 = nc;
            } else if (ci == 1u) {
                n1 = nc;
            } else if (ci == 2u) {
                n2 = nc;
            } else {
                n3 = nc;
            }
        }
    } else {
        let center_local = 0.25 * (p0 + p1 + p2 + p3);
        let n_final = compute_crease_aware_normal(center_local, n_face);
        n0 = n_final;
        n1 = n_final;
        n2 = n_final;
        n3 = n_final;
    }

    let is_side = axis_id != 2u;
    let col = eval_endpoint_color(global_ray, w_local, op_idx, is_side);

    let base_v = atomicAdd(&draw_args[0], 6u);
    if (base_v + 6u > max_verts) {
        atomicMin(&draw_args[0], max_verts);
        return;
    }
    let quad_idx = base_v / 6u;
    let q0 = oct_quant_12(n0);
    let q1 = oct_quant_12(n1);
    let q2 = oct_quant_12(n2);
    let q3 = oct_quant_12(n3);
    let ray_dw_flags = (global_ray & 0xFFFFFu)
        | ((dw_idx & 0x3FFu) << 20u)
        | select(0u, 0x80000000u, want_min);
    out_quads[quad_idx] = PackedTriDexelQuad(
        ray_dw_flags,
        pack4x8unorm(col),
        pack_corner_with_oct_lo(off0, q0),
        pack_corner_with_oct_lo(off1, q1),
        pack_corner_with_oct_lo(off2, q2),
        pack_corner_with_oct_lo(off3, q3),
        pack_oct_pair_hi8(q0, q1),
        pack_oct_pair_hi8(q2, q3),
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
