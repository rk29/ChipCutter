// Workgroup Shared-Memory Parallel Reduction Metrology Shader (`@workgroup_size(256, 1, 1)`)
// Computes remaining volume, target volume, excess/gouge volumes, RMS/MAE/Bias, excess/gouge RMS,
// max excess & max gouge with exact 3D WCS locations, exact in-tolerance count, and a 256-bin
// logarithmic error histogram (`0.0005 mm .. 100.0 mm` + exact zero bin) across Heightfield,
// Cylindrical, and Tri-Dexel GPU storage buffers (<= 6 storage buffers per entry point, 10 KB
// workgroup shared memory < 16 KB WebGPU default limit).

const WG_SIZE: u32 = 256u;
const INLINE_K: u32 = 4u;

const HIST_MIN_MM: f32 = 0.0005;
const HIST_MAX_MM: f32 = 100.0;
const HIST_LOG_MIN: f32 = -7.6009025; // ln(0.0005)
const HIST_LOG_INV_SPAN: f32 = 20.809314; // 254.0 / (ln(100.0) - ln(0.0005))

struct FlatDexelSpan {
    min_local: f32,
    max_local: f32,
    op_packed: u32,
    _pad: u32,
}

// 128-byte (8 x 16B) unified uniform buffer across Heightfield, Cylindrical, and Tri-Dexel reductions.
struct MetricsUniforms {
    // Heightfield: [nx, ny, 0, flags]
    // Cylindrical: [nu, ntheta, 0, flags]
    // TriDexel:    [nx, ny, nz, flags (bit 0 = has_target, bit 1 = volume_only)]
    dims: vec4<u32>,
    // TriDexel plane ray offsets: [off_x, off_y, off_z, total_rays]
    td_offsets: vec4<u32>,
    // [origin_wcs.x, origin_wcs.y, origin_wcs.z, tolerance_mm]
    origin_and_tol: vec4<f32>,
    // [dx_mm, dy_mm, dz_mm, mean_pitch_mm] (or Cylindrical: [du, dtheta, u_min, u_center])
    pitch_and_mean: vec4<f32>,
    // Cylindrical: [e1.x, e1.y, e1.z, direction_sign]
    cyl_e1_pol: vec4<f32>,
    // Cylindrical: [e2.x, e2.y, e2.z, 0.0]
    cyl_e2: vec4<f32>,
    // Cylindrical: [u_vec.x, u_vec.y, u_vec.z, 0.0]
    cyl_u_vec: vec4<f32>,
    // [num_workgroups, 0, 0, 0]
    dispatch: vec4<u32>,
}

// 96-byte (6 x 16B) per-workgroup partial reduction output read back to CPU for f64 finalization.
struct GpuMetricsPartial {
    remaining_vol: f32,
    target_vol: f32,
    excess_vol: f32,
    gouge_vol: f32,
    sum_sq: f32,
    sum_abs: f32,
    sum_signed: f32,
    excess_sum_sq: f32,
    gouge_sum_sq: f32,
    max_excess: f32,
    max_gouge: f32,
    _pad0: f32,
    max_excess_loc: vec3<f32>,
    sample_count: u32,
    max_gouge_loc: vec3<f32>,
    in_tol_count: u32,
    excess_count: u32,
    gouge_count: u32,
    _pad1: u32,
    _pad2: u32,
}

@group(0) @binding(0) var<uniform> u: MetricsUniforms;
@group(0) @binding(1) var<storage, read_write> partials: array<GpuMetricsPartial>;
@group(0) @binding(2) var<storage, read_write> global_hist: array<atomic<u32>>;

// Mode-specific read-only stock/target buffers in @group(1)
@group(1) @binding(0) var<storage, read> hf_z_stock: array<f32>;
@group(1) @binding(1) var<storage, read> hf_z_bottom: array<f32>;
@group(1) @binding(2) var<storage, read> hf_z_target: array<f32>;
@group(1) @binding(3) var<storage, read> cyl_r_stock: array<f32>;
@group(1) @binding(4) var<storage, read> cyl_r_target: array<f32>;
@group(1) @binding(5) var<storage, read> td_m_slots: array<FlatDexelSpan>;
@group(1) @binding(6) var<storage, read> td_m_counts: array<u32>;
@group(1) @binding(7) var<storage, read> td_t_slots: array<FlatDexelSpan>;
@group(1) @binding(8) var<storage, read> td_t_counts: array<u32>;

// 10 KB workgroup shared memory (strictly < 16 KB WebGPU default limit)
var<workgroup> sh_f0: array<f32, 256>;
var<workgroup> sh_f1: array<f32, 256>;
var<workgroup> sh_f2: array<f32, 256>;
var<workgroup> sh_f3: array<f32, 256>;
var<workgroup> sh_f4: array<f32, 256>;
var<workgroup> sh_u0: array<u32, 256>;
var<workgroup> sh_u1: array<u32, 256>;
var<workgroup> sh_u2: array<u32, 256>;
var<workgroup> sh_u3: array<u32, 256>;
var<workgroup> sh_hist: array<atomic<u32>, 256>;

struct ThreadAccum {
    remaining_vol: f32,
    target_vol: f32,
    excess_vol: f32,
    gouge_vol: f32,
    sum_sq: f32,
    sum_abs: f32,
    sum_signed: f32,
    excess_sum_sq: f32,
    gouge_sum_sq: f32,
    max_excess: f32,
    max_excess_loc: vec3<f32>,
    max_gouge: f32,
    max_gouge_loc: vec3<f32>,
    sample_count: u32,
    in_tol_count: u32,
    excess_count: u32,
    gouge_count: u32,
}

fn zero_accum() -> ThreadAccum {
    return ThreadAccum(
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, vec3<f32>(0.0, 0.0, 0.0),
        0.0, vec3<f32>(0.0, 0.0, 0.0),
        0u, 0u, 0u, 0u,
    );
}

fn is_finite_f32(v: f32) -> bool {
    let bits = bitcast<u32>(v);
    return (bits & 0x7f800000u) != 0x7f800000u;
}

fn error_hist_bin(abs_err: f32) -> u32 {
    if (abs_err < HIST_MIN_MM) {
        return 0u;
    }
    if (abs_err >= HIST_MAX_MM) {
        return 255u;
    }
    let t = (log(abs_err) - HIST_LOG_MIN) * HIST_LOG_INV_SPAN;
    return clamp(1u + u32(floor(t)), 1u, 254u);
}

fn push_deviation(acc: ptr<function, ThreadAccum>, signed_dev: f32, loc: vec3<f32>, tol: f32) {
    if (!is_finite_f32(signed_dev)) {
        return;
    }
    let abs_d = abs(signed_dev);
    (*acc).sample_count = (*acc).sample_count + 1u;
    (*acc).sum_sq = (*acc).sum_sq + signed_dev * signed_dev;
    (*acc).sum_abs = (*acc).sum_abs + abs_d;
    (*acc).sum_signed = (*acc).sum_signed + signed_dev;
    if (abs_d <= tol) {
        (*acc).in_tol_count = (*acc).in_tol_count + 1u;
    }
    let bin = error_hist_bin(abs_d);
    atomicAdd(&sh_hist[bin], 1u);

    if (signed_dev > 1e-6) {
        (*acc).excess_sum_sq = (*acc).excess_sum_sq + signed_dev * signed_dev;
        (*acc).excess_count = (*acc).excess_count + 1u;
        if (signed_dev > (*acc).max_excess) {
            (*acc).max_excess = signed_dev;
            (*acc).max_excess_loc = loc;
        }
    } else if (signed_dev < -1e-6) {
        let g = -signed_dev;
        (*acc).gouge_sum_sq = (*acc).gouge_sum_sq + g * g;
        (*acc).gouge_count = (*acc).gouge_count + 1u;
        if (g > (*acc).max_gouge) {
            (*acc).max_gouge = g;
            (*acc).max_gouge_loc = loc;
        }
    }
}

fn finalize_workgroup(lid: u32, wg_id: u32, acc: ThreadAccum, write_hist: bool) {
    // Flush shared histogram to global histogram (one atomic per non-zero bin per workgroup)
    workgroupBarrier();
    if (write_hist) {
        let h_val = atomicLoad(&sh_hist[lid]);
        if (h_val > 0u) {
            atomicAdd(&global_hist[lid], h_val);
        }
    }

    // Round 1: reduce volumes (4 f32) + counts (4 u32)
    sh_f0[lid] = acc.remaining_vol;
    sh_f1[lid] = acc.target_vol;
    sh_f2[lid] = acc.excess_vol;
    sh_f3[lid] = acc.gouge_vol;
    sh_u0[lid] = acc.sample_count;
    sh_u1[lid] = acc.in_tol_count;
    sh_u2[lid] = acc.excess_count;
    sh_u3[lid] = acc.gouge_count;
    workgroupBarrier();

    var stride: u32 = WG_SIZE >> 1u;
    while (stride > 0u) {
        if (lid < stride) {
            let o = lid + stride;
            sh_f0[lid] = sh_f0[lid] + sh_f0[o];
            sh_f1[lid] = sh_f1[lid] + sh_f1[o];
            sh_f2[lid] = sh_f2[lid] + sh_f2[o];
            sh_f3[lid] = sh_f3[lid] + sh_f3[o];
            sh_u0[lid] = sh_u0[lid] + sh_u0[o];
            sh_u1[lid] = sh_u1[lid] + sh_u1[o];
            sh_u2[lid] = sh_u2[lid] + sh_u2[o];
            sh_u3[lid] = sh_u3[lid] + sh_u3[o];
        }
        workgroupBarrier();
        stride = stride >> 1u;
    }

    var out: GpuMetricsPartial;
    if (lid == 0u) {
        out.remaining_vol = sh_f0[0];
        out.target_vol = sh_f1[0];
        out.excess_vol = sh_f2[0];
        out.gouge_vol = sh_f3[0];
        out.sample_count = sh_u0[0];
        out.in_tol_count = sh_u1[0];
        out.excess_count = sh_u2[0];
        out.gouge_count = sh_u3[0];
    }

    // Round 2: reduce error moments (5 f32)
    sh_f0[lid] = acc.sum_sq;
    sh_f1[lid] = acc.sum_abs;
    sh_f2[lid] = acc.sum_signed;
    sh_f3[lid] = acc.excess_sum_sq;
    sh_f4[lid] = acc.gouge_sum_sq;
    workgroupBarrier();

    stride = WG_SIZE >> 1u;
    while (stride > 0u) {
        if (lid < stride) {
            let o = lid + stride;
            sh_f0[lid] = sh_f0[lid] + sh_f0[o];
            sh_f1[lid] = sh_f1[lid] + sh_f1[o];
            sh_f2[lid] = sh_f2[lid] + sh_f2[o];
            sh_f3[lid] = sh_f3[lid] + sh_f3[o];
            sh_f4[lid] = sh_f4[lid] + sh_f4[o];
        }
        workgroupBarrier();
        stride = stride >> 1u;
    }

    if (lid == 0u) {
        out.sum_sq = sh_f0[0];
        out.sum_abs = sh_f1[0];
        out.sum_signed = sh_f2[0];
        out.excess_sum_sq = sh_f3[0];
        out.gouge_sum_sq = sh_f4[0];
    }

    // Round 3: argmax `max_excess` + 3D WCS location
    sh_f0[lid] = acc.max_excess;
    sh_f1[lid] = acc.max_excess_loc.x;
    sh_f2[lid] = acc.max_excess_loc.y;
    sh_f3[lid] = acc.max_excess_loc.z;
    workgroupBarrier();

    stride = WG_SIZE >> 1u;
    while (stride > 0u) {
        if (lid < stride) {
            let o = lid + stride;
            if (sh_f0[o] > sh_f0[lid]) {
                sh_f0[lid] = sh_f0[o];
                sh_f1[lid] = sh_f1[o];
                sh_f2[lid] = sh_f2[o];
                sh_f3[lid] = sh_f3[o];
            }
        }
        workgroupBarrier();
        stride = stride >> 1u;
    }

    if (lid == 0u) {
        out.max_excess = sh_f0[0];
        out.max_excess_loc = vec3<f32>(sh_f1[0], sh_f2[0], sh_f3[0]);
    }

    // Round 4: argmax `max_gouge` + 3D WCS location
    sh_f0[lid] = acc.max_gouge;
    sh_f1[lid] = acc.max_gouge_loc.x;
    sh_f2[lid] = acc.max_gouge_loc.y;
    sh_f3[lid] = acc.max_gouge_loc.z;
    workgroupBarrier();

    stride = WG_SIZE >> 1u;
    while (stride > 0u) {
        if (lid < stride) {
            let o = lid + stride;
            if (sh_f0[o] > sh_f0[lid]) {
                sh_f0[lid] = sh_f0[o];
                sh_f1[lid] = sh_f1[o];
                sh_f2[lid] = sh_f2[o];
                sh_f3[lid] = sh_f3[o];
            }
        }
        workgroupBarrier();
        stride = stride >> 1u;
    }

    if (lid == 0u) {
        out.max_gouge = sh_f0[0];
        out.max_gouge_loc = vec3<f32>(sh_f1[0], sh_f2[0], sh_f3[0]);
        out._pad0 = 0.0;
        out._pad1 = 0u;
        out._pad2 = 0u;
        partials[wg_id] = out;
    }
}

// === Entry Point 1: Heightfield Parallel Reduction ===
@compute @workgroup_size(256, 1, 1)
fn reduce_heightfield(
    @builtin(local_invocation_id) lid_v: vec3<u32>,
    @builtin(workgroup_id) wid_v: vec3<u32>,
) {
    let lid = lid_v.x;
    let wg_id = wid_v.x;
    atomicStore(&sh_hist[lid], 0u);
    workgroupBarrier();

    let nx = u.dims.x;
    let ny = u.dims.y;
    let total = nx * ny;
    let vol_only = (u.dims.w & 2u) != 0u;
    let num_wg = u.dispatch.x;
    let grid_stride = num_wg * WG_SIZE;

    let origin = u.origin_and_tol.xyz;
    let tol = max(u.origin_and_tol.w, 1e-6);
    let dx = u.pitch_and_mean.x;
    let dy = u.pitch_and_mean.y;
    let cell_area = dx * dy;

    var acc = zero_accum();

    var idx = wg_id * WG_SIZE + lid;
    while (idx < total) {
        let zb = hf_z_bottom[idx];
        if (is_finite_f32(zb)) {
            let zs_raw = hf_z_stock[idx];
            let has_mat = is_finite_f32(zs_raw) && zs_raw > zb;
            let zm = select(zb, zs_raw, has_mat);
            if (has_mat) {
                acc.remaining_vol = acc.remaining_vol + (zm - zb) * cell_area;
            }
            if (!vol_only) {
                let zt = hf_z_target[idx];
                if (is_finite_f32(zt) && zt > zb + 1e-6) {
                    acc.target_vol = acc.target_vol + (zt - zb) * cell_area;
                    let dev = zm - zt;
                    if (dev > 0.0) {
                        acc.excess_vol = acc.excess_vol + dev * cell_area;
                    } else {
                        acc.gouge_vol = acc.gouge_vol + (-dev) * cell_area;
                    }
                    let ix = idx % nx;
                    let iy = idx / nx;
                    let loc = vec3<f32>(
                        origin.x + (f32(ix) + 0.5) * dx,
                        origin.y + (f32(iy) + 0.5) * dy,
                        origin.z + zt,
                    );
                    push_deviation(&acc, dev, loc, tol);
                } else if (zm > zb + 1e-6) {
                    acc.excess_vol = acc.excess_vol + (zm - zb) * cell_area;
                }
            }
        }
        idx = idx + grid_stride;
    }

    finalize_workgroup(lid, wg_id, acc, !vol_only);
}

// === Entry Point 2: 4th-Axis Cylindrical Parallel Reduction ===
@compute @workgroup_size(256, 1, 1)
fn reduce_cylindrical(
    @builtin(local_invocation_id) lid_v: vec3<u32>,
    @builtin(workgroup_id) wid_v: vec3<u32>,
) {
    let lid = lid_v.x;
    let wg_id = wid_v.x;
    atomicStore(&sh_hist[lid], 0u);
    workgroupBarrier();

    let nu = u.dims.x;
    let ntheta = u.dims.y;
    let total = nu * ntheta;
    let vol_only = (u.dims.w & 2u) != 0u;
    let num_wg = u.dispatch.x;
    let grid_stride = num_wg * WG_SIZE;

    let center = u.origin_and_tol.xyz;
    let tol = max(u.origin_and_tol.w, 1e-6);
    let du = u.pitch_and_mean.x;
    let dtheta = u.pitch_and_mean.y;
    let u_min = u.pitch_and_mean.z;
    let u_center = u.pitch_and_mean.w;
    let e1 = u.cyl_e1_pol.xyz;
    let pol = u.cyl_e1_pol.w;
    let e2 = u.cyl_e2.xyz;
    let u_vec = u.cyl_u_vec.xyz;
    let factor = 0.5 * du * dtheta;

    var acc = zero_accum();

    var idx = wg_id * WG_SIZE + lid;
    while (idx < total) {
        let rm = max(cyl_r_stock[idx], 0.0);
        acc.remaining_vol = acc.remaining_vol + factor * rm * rm;
        if (!vol_only) {
            let rt = cyl_r_target[idx];
            if (rt >= 0.0 && is_finite_f32(rt)) {
                acc.target_vol = acc.target_vol + factor * rt * rt;
                let dev = rm - rt;
                if (dev > 0.0) {
                    acc.excess_vol = acc.excess_vol + factor * (rm * rm - rt * rt);
                } else {
                    acc.gouge_vol = acc.gouge_vol + factor * (rt * rt - rm * rm);
                }
                let iu = idx / ntheta;
                let it = idx % ntheta;
                let u_coord = u_min + (f32(iu) + 0.5) * du;
                let theta = f32(it) * dtheta;
                let loc = center
                    + u_vec * (u_coord - u_center)
                    + e1 * (rt * cos(theta))
                    + e2 * (rt * sin(theta) * pol);
                push_deviation(&acc, dev, loc, tol);
            } else if (rm > 1e-6) {
                acc.excess_vol = acc.excess_vol + factor * rm * rm;
            }
        }
        idx = idx + grid_stride;
    }

    finalize_workgroup(lid, wg_id, acc, !vol_only);
}

// === Tri-Dexel Helpers ===
fn load_td_span(idx: u32, is_target: bool) -> FlatDexelSpan {
    if (is_target) {
        return td_t_slots[idx];
    }
    return td_m_slots[idx];
}

fn td_spans_diff_len(
    a_base: u32,
    a_cnt: u32,
    a_is_target: bool,
    b_base: u32,
    b_cnt: u32,
    b_is_target: bool,
) -> f32 {
    if (a_cnt == 0u) {
        return 0.0;
    }
    var total: f32 = 0.0;
    if (b_cnt == 0u) {
        for (var i: u32 = 0u; i < a_cnt; i = i + 1u) {
            let sa = load_td_span(a_base + i, a_is_target);
            total = total + max(sa.max_local - sa.min_local, 0.0);
        }
        return total;
    }
    var j: u32 = 0u;
    for (var i: u32 = 0u; i < a_cnt; i = i + 1u) {
        let sa = load_td_span(a_base + i, a_is_target);
        var cur = sa.min_local;
        let end = sa.max_local;
        while (j < b_cnt) {
            let sb = load_td_span(b_base + j, b_is_target);
            if (sb.max_local <= cur) {
                j = j + 1u;
            } else {
                break;
            }
        }
        var k = j;
        while (k < b_cnt) {
            let sb = load_td_span(b_base + k, b_is_target);
            if (sb.min_local >= end) {
                break;
            }
            if (sb.min_local > cur) {
                total = total + (sb.min_local - cur);
            }
            cur = max(cur, sb.max_local);
            if (cur >= end) {
                break;
            }
            k = k + 1u;
        }
        if (cur < end) {
            total = total + (end - cur);
        }
    }
    return total;
}

fn td_signed_dev_at_target_endpoint(
    m_base: u32,
    m_cnt: u32,
    ts: FlatDexelSpan,
    upper_face: bool,
) -> f32 {
    if (upper_face) {
        let t_hi = ts.max_local;
        for (var i: u32 = 0u; i < m_cnt; i = i + 1u) {
            let ms = td_m_slots[m_base + i];
            if (t_hi >= ms.min_local - 1e-6 && t_hi <= ms.max_local + 1e-6) {
                return max(ms.max_local - t_hi, 0.0);
            }
        }
        var nearest_below = ts.min_local;
        for (var i: u32 = 0u; i < m_cnt; i = i + 1u) {
            let ms = td_m_slots[m_base + i];
            if (ms.max_local < t_hi) {
                nearest_below = max(nearest_below, ms.max_local);
            }
        }
        return -max(t_hi - nearest_below, 0.0);
    } else {
        let t_lo = ts.min_local;
        for (var i: u32 = 0u; i < m_cnt; i = i + 1u) {
            let ms = td_m_slots[m_base + i];
            if (t_lo >= ms.min_local - 1e-6 && t_lo <= ms.max_local + 1e-6) {
                return max(t_lo - ms.min_local, 0.0);
            }
        }
        var nearest_above = ts.max_local;
        for (var i: u32 = 0u; i < m_cnt; i = i + 1u) {
            let ms = td_m_slots[m_base + i];
            if (ms.min_local > t_lo) {
                nearest_above = min(nearest_above, ms.min_local);
            }
        }
        return -max(nearest_above - t_lo, 0.0);
    }
}

fn td_dist_to_air_1d(m_base: u32, m_cnt: u32, coord: f32) -> f32 {
    for (var i: u32 = 0u; i < m_cnt; i = i + 1u) {
        let s = td_m_slots[m_base + i];
        if (coord < s.min_local) {
            return 0.0;
        }
        if (coord <= s.max_local) {
            return min(coord - s.min_local, s.max_local - coord);
        }
    }
    return 0.0;
}

fn td_dist_to_solid_1d(m_base: u32, m_cnt: u32, coord: f32) -> f32 {
    if (m_cnt == 0u) {
        return 1e30;
    }
    var best: f32 = 1e30;
    for (var i: u32 = 0u; i < m_cnt; i = i + 1u) {
        let s = td_m_slots[m_base + i];
        if (coord >= s.min_local && coord <= s.max_local) {
            return 0.0;
        }
        let d = select(coord - s.max_local, s.min_local - coord, coord < s.min_local);
        best = min(best, d);
    }
    return best;
}

fn clamp_grid_idx(coord_local: f32, step: f32, len: u32) -> u32 {
    if (len == 0u) {
        return 0u;
    }
    let idx = i32(floor(coord_local / step));
    return u32(clamp(idx, 0, i32(len) - 1));
}

fn td_eval_ortho_plane(
    o_axis: u32,
    ix: u32,
    iy: u32,
    iz: u32,
    probe_local: vec3<f32>,
    is_excess: bool,
) -> f32 {
    var iu: u32;
    var iv: u32;
    var nu: u32;
    var ray_off: u32;
    var coord: f32;
    if (o_axis == 0u) {
        iu = iy; iv = iz; nu = u.dims.y; ray_off = u.td_offsets.x; coord = probe_local.x;
    } else if (o_axis == 1u) {
        iu = ix; iv = iz; nu = u.dims.x; ray_off = u.td_offsets.y; coord = probe_local.y;
    } else {
        iu = ix; iv = iy; nu = u.dims.x; ray_off = u.td_offsets.z; coord = probe_local.z;
    }
    let g_idx = ray_off + iv * nu + iu;
    let cnt = min(td_m_counts[g_idx], INLINE_K);
    let base = g_idx * INLINE_K;
    if (is_excess) {
        return td_dist_to_air_1d(base, cnt, coord);
    }
    return td_dist_to_solid_1d(base, cnt, coord);
}

fn td_refine_signed_dev(
    p_local: vec3<f32>,
    axis_id: u32,
    upper_face: bool,
    ts: FlatDexelSpan,
    d_primary: f32,
) -> f32 {
    let mean_pitch = u.pitch_and_mean.w;
    if (abs(d_primary) <= 0.5 * mean_pitch) {
        return d_primary;
    }
    let dir = select(-1.0, 1.0, upper_face);
    let is_excess = d_primary > 0.0;
    let span_len = max(ts.max_local - ts.min_local, 0.0);
    let shift = select(-min(0.25 * mean_pitch, 0.45 * span_len), 0.25 * mean_pitch, is_excess);
    var probe = p_local;
    if (axis_id == 0u) {
        probe.x = probe.x + dir * shift;
    } else if (axis_id == 1u) {
        probe.y = probe.y + dir * shift;
    } else {
        probe.z = probe.z + dir * shift;
    }
    let ix = clamp_grid_idx(probe.x, u.pitch_and_mean.x, u.dims.x);
    let iy = clamp_grid_idx(probe.y, u.pitch_and_mean.y, u.dims.y);
    let iz = clamp_grid_idx(probe.z, u.pitch_and_mean.z, u.dims.z);

    var o1_ax: u32;
    var o2_ax: u32;
    if (axis_id == 0u) {
        o1_ax = 1u; o2_ax = 2u;
    } else if (axis_id == 1u) {
        o1_ax = 0u; o2_ax = 2u;
    } else {
        o1_ax = 0u; o2_ax = 1u;
    }
    let o1 = td_eval_ortho_plane(o1_ax, ix, iy, iz, probe, is_excess);
    let o2 = td_eval_ortho_plane(o2_ax, ix, iy, iz, probe, is_excess);
    if (is_excess) {
        return min(d_primary, min(o1, o2));
    }
    return -min(-d_primary, min(o1, o2));
}

// === Entry Point 3: Tri-Dexel Parallel Reduction (All 3 Planes Simultaneously) ===
@compute @workgroup_size(256, 1, 1)
fn reduce_tridexel(
    @builtin(local_invocation_id) lid_v: vec3<u32>,
    @builtin(workgroup_id) wid_v: vec3<u32>,
) {
    let lid = lid_v.x;
    let wg_id = wid_v.x;
    atomicStore(&sh_hist[lid], 0u);
    workgroupBarrier();

    let nx = u.dims.x;
    let ny = u.dims.y;
    let has_target = (u.dims.w & 1u) != 0u;
    let vol_only = (u.dims.w & 2u) != 0u;
    let total_rays = u.td_offsets.w;
    let off_y = u.td_offsets.y;
    let off_z = u.td_offsets.z;

    let origin = u.origin_and_tol.xyz;
    let tol = max(u.origin_and_tol.w, 1e-6);
    let dx = u.pitch_and_mean.x;
    let dy = u.pitch_and_mean.y;
    let dz = u.pitch_and_mean.z;

    let num_wg = u.dispatch.x;
    let grid_stride = num_wg * WG_SIZE;

    var acc = zero_accum();

    var g_idx = wg_id * WG_SIZE + lid;
    while (g_idx < total_rays) {
        var axis_id: u32;
        var local_idx: u32;
        var nu: u32;
        var du: f32;
        var dv: f32;
        if (g_idx < off_y) {
            axis_id = 0u; local_idx = g_idx; nu = ny; du = dy; dv = dz;
        } else if (g_idx < off_z) {
            axis_id = 1u; local_idx = g_idx - off_y; nu = nx; du = dx; dv = dz;
        } else {
            axis_id = 2u; local_idx = g_idx - off_z; nu = nx; du = dx; dv = dy;
        }

        let cell_area_third = (du * dv) * (1.0 / 3.0);
        let m_raw_cnt = td_m_counts[g_idx];
        let base = g_idx * INLINE_K;

        // Skip rays residing in CPU overflow (> 4 spans); host adds their exact f64 contribution!
        if (m_raw_cnt <= INLINE_K) {
            var m_len: f32 = 0.0;
            for (var k: u32 = 0u; k < m_raw_cnt; k = k + 1u) {
                let s = td_m_slots[base + k];
                m_len = m_len + max(s.max_local - s.min_local, 0.0);
            }
            acc.remaining_vol = acc.remaining_vol + m_len * cell_area_third;
        }

        if (!vol_only && has_target) {
            let t_raw_cnt = td_t_counts[g_idx];
            if (m_raw_cnt <= INLINE_K && t_raw_cnt <= INLINE_K) {
                var t_len: f32 = 0.0;
                for (var k: u32 = 0u; k < t_raw_cnt; k = k + 1u) {
                    let s = td_t_slots[base + k];
                    t_len = t_len + max(s.max_local - s.min_local, 0.0);
                }
                acc.target_vol = acc.target_vol + t_len * cell_area_third;

                let ex_len = td_spans_diff_len(base, m_raw_cnt, false, base, t_raw_cnt, true);
                let g_len = td_spans_diff_len(base, t_raw_cnt, true, base, m_raw_cnt, false);
                acc.excess_vol = acc.excess_vol + ex_len * cell_area_third;
                acc.gouge_vol = acc.gouge_vol + g_len * cell_area_third;

                if (t_raw_cnt > 0u) {
                    let iu = local_idx % nu;
                    let iv = local_idx / nu;
                    let pu = (f32(iu) + 0.5) * du;
                    let pv = (f32(iv) + 0.5) * dv;
                    for (var k: u32 = 0u; k < t_raw_cnt; k = k + 1u) {
                        let ts = td_t_slots[base + k];
                        var p_hi_loc: vec3<f32>;
                        var p_lo_loc: vec3<f32>;
                        if (axis_id == 0u) {
                            p_hi_loc = vec3<f32>(ts.max_local, pu, pv);
                            p_lo_loc = vec3<f32>(ts.min_local, pu, pv);
                        } else if (axis_id == 1u) {
                            p_hi_loc = vec3<f32>(pu, ts.max_local, pv);
                            p_lo_loc = vec3<f32>(pu, ts.min_local, pv);
                        } else {
                            p_hi_loc = vec3<f32>(pu, pv, ts.max_local);
                            p_lo_loc = vec3<f32>(pu, pv, ts.min_local);
                        }

                        let d_hi_raw = td_signed_dev_at_target_endpoint(base, m_raw_cnt, ts, true);
                        let d_hi = td_refine_signed_dev(p_hi_loc, axis_id, true, ts, d_hi_raw);
                        push_deviation(&acc, d_hi, origin + p_hi_loc, tol);

                        let d_lo_raw = td_signed_dev_at_target_endpoint(base, m_raw_cnt, ts, false);
                        let d_lo = td_refine_signed_dev(p_lo_loc, axis_id, false, ts, d_lo_raw);
                        push_deviation(&acc, d_lo, origin + p_lo_loc, tol);
                    }
                }
            }
        }

        g_idx = g_idx + grid_stride;
    }

    finalize_workgroup(lid, wg_id, acc, !vol_only && has_target);
}
