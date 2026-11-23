// Axis-Generic Tiled 16x16 Flat Fixed-K (K = 4) Tri-Dexel WGSL Compute Sweeper
// Parameterized by `u.axis_id` (`0 = X`, `1 = Y`, `2 = Z`) using the exact `(u, v, w)`
// coordinate permutation of `crates/chip-stock/src/tridexel/sweep.rs` (zero X/Y/Z duplication).
// Covers all 7 cutter geometries (including single-point & multi-form ThreadMill undercuts),
// inline fixed-K span subtraction in registers, atomic Rules 1-4 + relieved-neck verification,
// and a lossless hybrid overflow bitset safety valve for rare `> 4`-span rays.

const INLINE_K: u32 = 4u;
const MIN_DEXEL_SPAN_MM: f32 = 1e-6;

const SHAPE_FLAT: u32 = 0u;
const SHAPE_BULLNOSE: u32 = 1u;
const SHAPE_BALL: u32 = 2u;
const SHAPE_CONICAL: u32 = 3u;
const SHAPE_FACE: u32 = 4u;
const SHAPE_THREADMILL: u32 = 5u;
const SHAPE_LOLLIPOP: u32 = 6u;
const SHAPE_SLOTMILL: u32 = 7u;
const SHAPE_DOVETAIL: u32 = 8u;

const PROF_FLAG_FLAT_ENDMILL: u32 = 1u;
const PROF_FLAG_THREAD_MILL: u32 = 2u;
const PROF_FLAG_RELIEVED_NECK: u32 = 8u;

const CHORD_FLAG_RAPID: u32 = 1u;
const CHORD_FLAG_VERIFY: u32 = 2u;

const SLOT_RAPID: u32 = 0u;
const SLOT_GOUGE: u32 = 8u;
const SLOT_FLUTE: u32 = 16u;
const SLOT_NECK: u32 = 24u;
const SLOT_HOLDER: u32 = 32u;
const WORD_MUTATED: u32 = 40u;
const WORD_OVERFLOW: u32 = 41u;
const OVERFLOW_BITS_BASE: u32 = 44u;

struct TriDexelPlaneUniforms {
    nu: u32,
    nv: u32,
    tiles_u: u32,
    num_tiles: u32,
    axis_id: u32,
    has_target: u32,
    plane_ray_offset: u32,
    _pad0: u32,
    du_mm: f32,
    dv_mm: f32,
    dz_step_mm: f32,
    _pad1: f32,
}

struct FlatDexelSpan {
    min_local: f32,
    max_local: f32,
    op_packed: u32,
    _pad: u32,
}

struct GpuToolProfile {
    shape_kind: u32,
    flags: u32,
    teeth_count: u32,
    _pad0: u32,
    cutting_radius_mm: f32,
    shank_radius_mm: f32,
    holder_radius_mm: f32,
    flute_length_mm: f32,
    effective_reach_mm: f32,
    stickout_length_mm: f32,
    neck_radius_mm: f32,
    neck_length_mm: f32,
    param0: f32,
    param1: f32,
    param2: f32,
    param3: f32,
}

struct GpuSweepChord {
    p0_rot: vec4<f32>,
    p1_rot: vec4<f32>,
    arc_params: vec4<f32>,
    sweep_and_tols: vec4<f32>,
    info: vec4<u32>,
}

struct Interval1D {
    valid: bool,
    lo: f32,
    hi: f32,
}

struct GougeHit {
    valid: bool,
    depth: f32,
    coord: f32,
}

// 1 uniform + 8 storage buffers (strictly within WebGPU default limit of 8 storage buffers/stage)
@group(0) @binding(0) var<uniform> u: TriDexelPlaneUniforms;
@group(0) @binding(1) var<storage, read_write> slots: array<FlatDexelSpan>;
@group(0) @binding(2) var<storage, read_write> counts: array<u32>;
@group(0) @binding(3) var<storage, read> target_slots: array<FlatDexelSpan>;
@group(0) @binding(4) var<storage, read> target_counts: array<u32>;
@group(0) @binding(5) var<storage, read> profiles: array<GpuToolProfile>;
@group(0) @binding(6) var<storage, read> chords: array<GpuSweepChord>;
@group(0) @binding(7) var<storage, read> tile_data: array<u32>;
@group(0) @binding(8) var<storage, read_write> accum_and_overflow: array<atomic<u32>>;

fn record_violation(slot_base: u32, depth: f32, chord_idx: u32, ray_idx: u32, loc_w: f32) {
    if (depth <= 0.0) {
        return;
    }
    let depth_bits = bitcast<u32>(depth);
    atomicStore(&accum_and_overflow[slot_base], 1u);
    let prev = atomicMax(&accum_and_overflow[slot_base + 1u], depth_bits);
    if (depth_bits > prev) {
        atomicStore(&accum_and_overflow[slot_base + 2u], chord_idx);
        atomicStore(&accum_and_overflow[slot_base + 3u], ray_idx);
        atomicStore(&accum_and_overflow[slot_base + 4u], bitcast<u32>(loc_w));
    }
}

fn flag_overflow_ray(ray_idx: u32) {
    atomicStore(&accum_and_overflow[WORD_OVERFLOW], 1u);
    let word = OVERFLOW_BITS_BASE + (ray_idx >> 5u);
    let mask = 1u << (ray_idx & 31u);
    atomicOr(&accum_and_overflow[word], mask);
}

fn pack_op16(op_idx: u32) -> u32 {
    return select(op_idx, 0xffffu, op_idx >= 0xffffu);
}

fn set_min_op(op_packed: u32, op_idx: u32) -> u32 {
    return (op_packed & 0xffff0000u) | pack_op16(op_idx);
}

fn set_max_op(op_packed: u32, op_idx: u32) -> u32 {
    return (op_packed & 0x0000ffffu) | (pack_op16(op_idx) << 16u);
}

fn query_overlap_local(
    local_spans: ptr<function, array<FlatDexelSpan, 4>>,
    cnt: u32,
    q_min: f32,
    q_max: f32,
) -> f32 {
    if (cnt == 0u || q_max <= q_min) {
        return 0.0;
    }
    var total: f32 = 0.0;
    for (var i: u32 = 0u; i < cnt; i = i + 1u) {
        let s = (*local_spans)[i];
        if (q_max <= s.min_local) {
            break;
        }
        if (q_min >= s.max_local) {
            continue;
        }
        let lo = max(q_min, s.min_local);
        let hi = min(q_max, s.max_local);
        if (hi > lo) {
            total = total + (hi - lo);
        }
    }
    return total;
}

fn query_target_gouge(
    global_ray_idx: u32,
    q_min: f32,
    q_max: f32,
    skin_mm: f32,
) -> GougeHit {
    var best = GougeHit(false, 0.0, 0.0);
    if (u.has_target == 0u || q_max <= q_min) {
        return best;
    }
    let t_cnt = min(target_counts[global_ray_idx], INLINE_K);
    if (t_cnt == 0u) {
        return best;
    }
    let base = global_ray_idx * INLINE_K;
    for (var i: u32 = 0u; i < t_cnt; i = i + 1u) {
        let ts = target_slots[base + i];
        if (q_max <= ts.min_local) {
            break;
        }
        if (q_min >= ts.max_local) {
            continue;
        }
        let eff_min = ts.min_local + skin_mm;
        let eff_max = ts.max_local - skin_mm;
        if (eff_max <= eff_min) {
            continue;
        }
        let ov_lo = max(q_min, eff_min);
        let ov_hi = min(q_max, eff_max);
        if (ov_hi > ov_lo) {
            let depth = ov_hi - ov_lo;
            var coord: f32;
            if (q_min > eff_min && q_max >= eff_max) {
                coord = ov_lo;
            } else if (q_max < eff_max && q_min <= eff_min) {
                coord = ov_hi;
            } else {
                coord = 0.5 * (ov_lo + ov_hi);
            }
            if (!best.valid || depth > best.depth) {
                best = GougeHit(true, depth, coord);
            }
        }
    }
    return best;
}

// Subtracts `[cut_min, cut_max]` from `local_spans[0..*cnt]`.
// Returns: `0` = unchanged, `1` = mutated inline, `-1` = overflow (> 4 spans).
fn subtract_interval_local(
    local_spans: ptr<function, array<FlatDexelSpan, 4>>,
    cnt: ptr<function, u32>,
    cut_min: f32,
    cut_max: f32,
    op_idx: u32,
) -> i32 {
    let n = *cnt;
    if (n == 0u || cut_max <= cut_min + 1e-7) {
        return 0;
    }
    var out_spans: array<FlatDexelSpan, 4>;
    var out_n: u32 = 0u;
    var mutated = false;

    for (var i: u32 = 0u; i < n; i = i + 1u) {
        let s = (*local_spans)[i];
        if (cut_max <= s.min_local || cut_min >= s.max_local) {
            if (out_n >= INLINE_K) {
                return -1;
            }
            out_spans[out_n] = s;
            out_n = out_n + 1u;
            continue;
        }
        let ov_min = max(cut_min, s.min_local);
        let ov_max = min(cut_max, s.max_local);
        if (ov_max - ov_min <= 1e-7) {
            if (out_n >= INLINE_K) {
                return -1;
            }
            out_spans[out_n] = s;
            out_n = out_n + 1u;
            continue;
        }
        mutated = true;
        let keep_lo = (ov_min - s.min_local) > MIN_DEXEL_SPAN_MM;
        let keep_hi = (s.max_local - ov_max) > MIN_DEXEL_SPAN_MM;
        if (keep_lo) {
            if (out_n >= INLINE_K) {
                return -1;
            }
            out_spans[out_n] = FlatDexelSpan(
                s.min_local,
                ov_min,
                set_max_op(s.op_packed, op_idx),
                0u,
            );
            out_n = out_n + 1u;
        }
        if (keep_hi) {
            if (out_n >= INLINE_K) {
                return -1;
            }
            out_spans[out_n] = FlatDexelSpan(
                ov_max,
                s.max_local,
                set_min_op(s.op_packed, op_idx),
                0u,
            );
            out_n = out_n + 1u;
        }
    }

    if (!mutated) {
        return 0;
    }
    for (var k: u32 = 0u; k < INLINE_K; k = k + 1u) {
        if (k < out_n) {
            (*local_spans)[k] = out_spans[k];
        } else {
            (*local_spans)[k] = FlatDexelSpan(0.0, 0.0, 0u, 0u);
        }
    }
    *cnt = out_n;
    return 1;
}

// Evaluates axial offset `dz >= 0` from cutter tip to lower cutting envelope at radius `r_mm`.
// Returns `< 0.0` if `r_mm` lies outside the cutting envelope.
fn eval_profile_dz(prof: GpuToolProfile, r_mm: f32) -> f32 {
    if (r_mm < 0.0 || r_mm > prof.cutting_radius_mm + 1e-6) {
        return -1.0;
    }
    let r = min(r_mm, prof.cutting_radius_mm);
    var dz: f32 = 0.0;
    switch (prof.shape_kind) {
        case SHAPE_FLAT: {
            dz = 0.0;
        }
        case SHAPE_BULLNOSE: {
            let flat_r = prof.param0;
            let corner_r = prof.param1;
            if (r > flat_r && corner_r > 1e-6) {
                let dr = min(r - flat_r, corner_r);
                dz = corner_r - sqrt(max((corner_r - dr) * (corner_r + dr), 0.0));
            }
        }
        case SHAPE_BALL: {
            let rad = prof.cutting_radius_mm;
            let rc = min(r, rad);
            dz = rad - sqrt(max((rad - rc) * (rad + rc), 0.0));
        }
        case SHAPE_CONICAL: {
            let tip_r = prof.param0;
            let tan_half = prof.param1;
            if (r > tip_r) {
                dz = (r - tip_r) / tan_half;
            }
        }
        case SHAPE_FACE: {
            let flat_r = prof.param0;
            let corner_r = prof.param1;
            let bottom_r = prof.param2;
            if (r <= flat_r) {
                dz = 0.0;
            } else if (r <= bottom_r) {
                if (corner_r > 1e-6) {
                    let dr = min(r - flat_r, corner_r);
                    dz = corner_r - sqrt(max((corner_r - dr) * (corner_r + dr), 0.0));
                }
            } else {
                dz = corner_r + (r - bottom_r);
            }
        }
        case SHAPE_THREADMILL: {
            if (r > prof.neck_radius_mm) {
                let z_start = prof.param1;
                let tooth_step = prof.param2;
                let tan_half = prof.param3;
                let z_first_crest = z_start + 0.5 * tooth_step;
                dz = max(z_first_crest - max(prof.cutting_radius_mm - r, 0.0) * tan_half, 0.0);
            }
        }
        case SHAPE_LOLLIPOP: {
            let flat_r = prof.param1;
            let sphere_r = select(prof.cutting_radius_mm, prof.param2, prof.param2 > 1e-6);
            if (r > flat_r) {
                let dr = min(r - flat_r, sphere_r);
                dz = sphere_r - sqrt(max((sphere_r - dr) * (sphere_r + dr), 0.0));
            }
        }
        case SHAPE_SLOTMILL: {
            let corner_r = prof.param1;
            let flat_r = prof.param2;
            if (r > flat_r && corner_r > 1e-6) {
                let dr = min(r - flat_r, corner_r);
                dz = corner_r - sqrt(max((corner_r - dr) * (corner_r + dr), 0.0));
            }
        }
        case SHAPE_DOVETAIL: {
            let corner_r = prof.param2;
            let r_c = max(prof.cutting_radius_mm - corner_r, 0.0);
            if (r > r_c && corner_r > 1e-6) {
                let dr = min(r - r_c, corner_r);
                dz = corner_r - sqrt(max((corner_r - dr) * (corner_r + dr), 0.0));
            }
        }
        default: {
            dz = 0.0;
        }
    }
    return dz;
}

// Evaluates exact bounded axial cutting interval `[dz_lo, dz_hi]` at radius `r_mm`.
fn eval_profile_z_interval(prof: GpuToolProfile, r_mm: f32) -> Interval1D {
    let miss = Interval1D(false, 0.0, 0.0);
    if (r_mm < 0.0 || r_mm > prof.cutting_radius_mm + 1e-6) {
        return miss;
    }
    let r = min(r_mm, prof.cutting_radius_mm);
    var out = miss;
    switch (prof.shape_kind) {
        case SHAPE_LOLLIPOP: {
            let z_join = prof.param0;
            let flat_r = prof.param1;
            let sphere_r = select(prof.cutting_radius_mm, prof.param2, prof.param2 > 1e-6);
            if (r <= flat_r) {
                out = Interval1D(true, 0.0, z_join);
            } else {
                let dr = min(r - flat_r, sphere_r);
                let half_h = sqrt(max((sphere_r - dr) * (sphere_r + dr), 0.0));
                let dz_lo = max(sphere_r - half_h, 0.0);
                let dz_hi = min(sphere_r + half_h, z_join);
                out = Interval1D(true, dz_lo, max(dz_hi, dz_lo));
            }
        }
        case SHAPE_SLOTMILL: {
            let h_cut = prof.param0;
            let corner_r = prof.param1;
            let flat_r = prof.param2;
            var dz_lo: f32 = 0.0;
            if (r > flat_r && corner_r > 1e-6) {
                let dr = min(r - flat_r, corner_r);
                dz_lo = corner_r - sqrt(max((corner_r - dr) * (corner_r + dr), 0.0));
            }
            let dz_hi = max(h_cut - dz_lo, dz_lo);
            out = Interval1D(true, dz_lo, dz_hi);
        }
        case SHAPE_DOVETAIL: {
            let h_cut = prof.param0;
            let tan_a = max(prof.param1, 1e-5);
            let corner_r = prof.param2;
            if (corner_r <= 1e-6) {
                let dz_hi = clamp((prof.cutting_radius_mm - r) * tan_a, 0.0, h_cut);
                out = Interval1D(true, 0.0, dz_hi);
            } else {
                let r_c = max(prof.cutting_radius_mm - corner_r, 0.0);
                var dz_lo: f32 = 0.0;
                if (r > r_c) {
                    let dr = min(r - r_c, corner_r);
                    dz_lo = corner_r - sqrt(max((corner_r - dr) * (corner_r + dr), 0.0));
                }
                let cos_a = 1.0 / sqrt(1.0 + tan_a * tan_a);
                let sin_a = tan_a * cos_a;
                let r_tan = r_c + corner_r * sin_a;
                var dz_hi: f32;
                if (r >= r_tan) {
                    let dr = min(r - r_c, corner_r);
                    dz_hi = min(corner_r + sqrt(max((corner_r - dr) * (corner_r + dr), 0.0)), h_cut);
                } else {
                    dz_hi = clamp(
                        corner_r + (corner_r - (r - r_c) * sin_a) / max(cos_a, 1e-5),
                        dz_lo,
                        h_cut,
                    );
                }
                out = Interval1D(true, dz_lo, max(dz_hi, dz_lo));
            }
        }
        default: {
            let dz_lo = eval_profile_dz(prof, r);
            if (dz_lo >= 0.0) {
                out = Interval1D(true, dz_lo, max(prof.flute_length_mm, dz_lo));
            }
        }
    }
    return out;
}

fn is_bounded_undercut_shape(shape_kind: u32) -> bool {
    return shape_kind == SHAPE_LOLLIPOP
        || shape_kind == SHAPE_SLOTMILL
        || shape_kind == SHAPE_DOVETAIL;
}

fn horizontal_cut_max_height(prof: GpuToolProfile) -> f32 {
    return select(prof.flute_length_mm, prof.effective_reach_mm, (prof.flags & PROF_FLAG_THREAD_MILL) != 0u);
}

fn thread_mill_tooth_radius_at_z(prof: GpuToolProfile, z_local: f32) -> f32 {
    let z_start = prof.param1;
    let tooth_step = prof.param2;
    let tan_half = prof.param3;
    let n = max(prof.teeth_count, 1u);
    let band_h = f32(n) * tooth_step;
    if (z_local < z_start - 1e-6 || z_local > z_start + band_h + 1e-6) {
        return prof.neck_radius_mm;
    }
    let local_z = clamp(z_local - z_start, 0.0, band_h);
    let idx = min(u32(floor(local_z / tooth_step)), n - 1u);
    let z_crest = z_start + (f32(idx) + 0.5) * tooth_step;
    let dz_from_crest = abs(z_local - z_crest);
    let r_flank = prof.cutting_radius_mm - dz_from_crest / tan_half;
    return clamp(r_flank, prof.neck_radius_mm, prof.cutting_radius_mm);
}

// Evaluates exact cutting envelope radius `r(dz) >= 0` at axial height `dz_mm >= 0`.
// Returns `< 0.0` if `dz_mm < -1e-6`.
fn eval_radius_at_dz(prof: GpuToolProfile, dz_mm: f32) -> f32 {
    if (dz_mm < -1e-6) {
        return -1.0;
    }
    let dz = max(dz_mm, 0.0);
    var out: f32 = prof.cutting_radius_mm;
    switch (prof.shape_kind) {
        case SHAPE_FLAT: {
            out = prof.cutting_radius_mm;
        }
        case SHAPE_BULLNOSE: {
            let flat_r = prof.param0;
            let corner_r = prof.param1;
            if (corner_r <= 1e-6 || dz >= corner_r) {
                out = flat_r + corner_r;
            } else {
                out = min(flat_r + sqrt(max(2.0 * corner_r * dz - dz * dz, 0.0)), prof.cutting_radius_mm);
            }
        }
        case SHAPE_BALL: {
            let rad = prof.cutting_radius_mm;
            if (dz >= rad) {
                out = rad;
            } else {
                out = min(sqrt(max(2.0 * rad * dz - dz * dz, 0.0)), rad);
            }
        }
        case SHAPE_CONICAL: {
            let tip_r = prof.param0;
            let tan_half = prof.param1;
            out = min(tip_r + dz * tan_half, prof.cutting_radius_mm);
        }
        case SHAPE_FACE: {
            let flat_r = prof.param0;
            let corner_r = prof.param1;
            let bottom_r = prof.param2;
            let outer_r = prof.param3;
            if (dz < corner_r && corner_r > 1e-6) {
                out = min(flat_r + sqrt(max(2.0 * corner_r * dz - dz * dz, 0.0)), bottom_r);
            } else {
                out = min(bottom_r + (dz - corner_r), outer_r);
            }
        }
        case SHAPE_THREADMILL: {
            if (dz <= prof.flute_length_mm + 1e-6) {
                out = thread_mill_tooth_radius_at_z(prof, dz);
            } else if (dz <= prof.neck_length_mm + 1e-6) {
                out = prof.neck_radius_mm;
            } else {
                out = prof.shank_radius_mm;
            }
        }
        case SHAPE_LOLLIPOP: {
            let z_join = prof.param0;
            if (dz > z_join + 1e-5) {
                out = -1.0;
            } else {
                let flat_r = prof.param1;
                let sphere_r = select(prof.cutting_radius_mm, prof.param2, prof.param2 > 1e-6);
                out = min(flat_r + sqrt(max(2.0 * sphere_r * dz - dz * dz, 0.0)), prof.cutting_radius_mm);
            }
        }
        case SHAPE_SLOTMILL: {
            let h_cut = prof.param0;
            if (dz > h_cut + 1e-5) {
                out = -1.0;
            } else {
                let corner_r = prof.param1;
                let flat_r = prof.param2;
                if (corner_r > 1e-6 && dz < corner_r) {
                    out = min(flat_r + sqrt(max(2.0 * corner_r * dz - dz * dz, 0.0)), prof.cutting_radius_mm);
                } else if (corner_r > 1e-6 && dz > h_cut - corner_r) {
                    let dtop = max(h_cut - dz, 0.0);
                    out = min(flat_r + sqrt(max(2.0 * corner_r * dtop - dtop * dtop, 0.0)), prof.cutting_radius_mm);
                } else {
                    out = prof.cutting_radius_mm;
                }
            }
        }
        case SHAPE_DOVETAIL: {
            let h_cut = prof.param0;
            if (dz > h_cut + 1e-5) {
                out = -1.0;
            } else {
                let tan_a = max(prof.param1, 1e-5);
                let corner_r = prof.param2;
                let top_r = prof.param3;
                if (corner_r <= 1e-6) {
                    out = clamp(prof.cutting_radius_mm - dz / tan_a, top_r, prof.cutting_radius_mm);
                } else {
                    let cos_a = 1.0 / sqrt(1.0 + tan_a * tan_a);
                    let sin_a = tan_a * cos_a;
                    let r_c = max(prof.cutting_radius_mm - corner_r, 0.0);
                    let z_tan = corner_r * (1.0 + cos_a);
                    if (dz <= z_tan) {
                        let d = dz - corner_r;
                        out = min(r_c + sqrt(max(corner_r * corner_r - d * d, 0.0)), prof.cutting_radius_mm);
                    } else {
                        let r_tan = r_c + corner_r * sin_a;
                        out = clamp(r_tan - (dz - z_tan) / tan_a, top_r, prof.cutting_radius_mm);
                    }
                }
            }
        }
        default: {
            out = prof.cutting_radius_mm;
        }
    }
    return out;
}

fn clamp_unit_interval(a: f32, b: f32) -> Interval1D {
    let lo = min(a, b);
    let hi = max(a, b);
    if (hi < 0.0 || lo > 1.0) {
        return Interval1D(false, 0.0, 0.0);
    }
    return Interval1D(true, max(lo, 0.0), min(hi, 1.0));
}

// Exact closed-form intersection of horizontal line `y = y_line` with 2D capsule of radius `r`
// swept along segment `p0 -> p1`.
fn intersect_horizontal_line_with_capsule_2d(
    p0: vec2<f32>,
    p1: vec2<f32>,
    r: f32,
    y_line: f32,
) -> Interval1D {
    let miss = Interval1D(false, 0.0, 0.0);
    if (r < 0.0) {
        return miss;
    }
    let y_min = min(p0.y, p1.y) - r;
    let y_max = max(p0.y, p1.y) + r;
    if (y_line < y_min || y_line > y_max) {
        return miss;
    }

    let r2 = r * r;
    var has_hit = false;
    var x_lo: f32 = 1e30;
    var x_hi: f32 = -1e30;

    let dy0 = y_line - p0.y;
    let d0_sq = r2 - dy0 * dy0;
    if (d0_sq >= 0.0) {
        let dx = sqrt(d0_sq);
        x_lo = min(x_lo, p0.x - dx);
        x_hi = max(x_hi, p0.x + dx);
        has_hit = true;
    }

    let dy1 = y_line - p1.y;
    let d1_sq = r2 - dy1 * dy1;
    if (d1_sq >= 0.0) {
        let dx = sqrt(d1_sq);
        x_lo = min(x_lo, p1.x - dx);
        x_hi = max(x_hi, p1.x + dx);
        has_hit = true;
    }

    let seg_v = p1 - p0;
    let len = length(seg_v);
    if (len > 1e-7) {
        let n = vec2<f32>(-seg_v.y / len, seg_v.x / len) * r;
        for (var si: i32 = -1; si <= 1; si = si + 2) {
            let sgn = f32(si);
            let a = p0 + n * sgn;
            let b = p1 + n * sgn;
            let dy = b.y - a.y;
            if (abs(dy) > 1e-7) {
                let s = (y_line - a.y) / dy;
                if (s >= 0.0 && s <= 1.0) {
                    let xc = a.x + s * (b.x - a.x);
                    x_lo = min(x_lo, xc);
                    x_hi = max(x_hi, xc);
                    has_hit = true;
                }
            }
        }
    }

    if (has_hit && x_hi >= x_lo) {
        return Interval1D(true, x_lo, x_hi);
    }
    return miss;
}

fn proj_horiz(p: vec3<f32>, swap_xy: bool) -> vec2<f32> {
    return select(vec2<f32>(p.x, p.y), vec2<f32>(p.y, p.x), swap_xy);
}

fn eval_horizontal_ray_cylinder_interval(
    p0: vec3<f32>,
    p1: vec3<f32>,
    cyl_radius: f32,
    cyl_height: f32,
    pz: f32,
    fixed_u: f32,
    swap_xy: bool,
) -> Interval1D {
    let miss = Interval1D(false, 0.0, 0.0);
    let min_z = min(p0.z, p1.z);
    let max_z = max(p0.z, p1.z);
    if (cyl_radius <= 1e-6 || pz < min_z - 1e-6 || pz > max_z + cyl_height + 1e-6) {
        return miss;
    }
    let dz_span = p1.z - p0.z;
    if (abs(dz_span) <= 1e-6) {
        let dz = pz - p0.z;
        if (dz <= 1e-6 || dz > cyl_height + 1e-6) {
            return miss;
        }
        return intersect_horizontal_line_with_capsule_2d(
            proj_horiz(p0, swap_xy),
            proj_horiz(p1, swap_xy),
            cyl_radius,
            fixed_u,
        );
    }
    let iv = clamp_unit_interval((pz - cyl_height - p0.z) / dz_span, (pz - p0.z) / dz_span);
    if (!iv.valid) {
        return miss;
    }
    let q0 = mix(p0, p1, iv.lo);
    let q1 = mix(p0, p1, iv.hi);
    return intersect_horizontal_line_with_capsule_2d(
        proj_horiz(q0, swap_xy),
        proj_horiz(q1, swap_xy),
        cyl_radius,
        fixed_u,
    );
}

fn eval_horizontal_ray_cut_interval(
    p0: vec3<f32>,
    p1: vec3<f32>,
    prof: GpuToolProfile,
    radial_shrink: f32,
    pz: f32,
    fixed_u: f32,
    swap_xy: bool,
) -> Interval1D {
    let miss = Interval1D(false, 0.0, 0.0);
    let max_h = horizontal_cut_max_height(prof);
    let min_z = min(p0.z, p1.z);
    let max_z = max(p0.z, p1.z);
    if (pz < min_z - 1e-6 || pz > max_z + max_h + 1e-6) {
        return miss;
    }

    let dz_span = p1.z - p0.z;
    if (abs(dz_span) <= 1e-6) {
        let dz = pz - p0.z;
        if (dz <= 1e-6 || dz > max_h + 1e-6) {
            return miss;
        }
        let raw_r = eval_radius_at_dz(prof, dz);
        if (raw_r < 0.0) {
            return miss;
        }
        let r = max(raw_r - radial_shrink, 0.0);
        if (r <= 1e-6) {
            return miss;
        }
        return intersect_horizontal_line_with_capsule_2d(
            proj_horiz(p0, swap_xy),
            proj_horiz(p1, swap_xy),
            r,
            fixed_u,
        );
    }

    let iv = clamp_unit_interval((pz - max_h - p0.z) / dz_span, (pz - p0.z) / dz_span);
    if (!iv.valid) {
        return miss;
    }

    if ((prof.flags & PROF_FLAG_THREAD_MILL) != 0u) {
        var has_hit = false;
        var lo: f32 = 1e30;
        var hi: f32 = -1e30;

        let r_neck = max(prof.neck_radius_mm - radial_shrink, 0.0);
        if (r_neck > 1e-6 && prof.neck_length_mm > prof.flute_length_mm + 1e-6) {
            let niv = clamp_unit_interval(
                (pz - prof.neck_length_mm - p0.z) / dz_span,
                (pz - prof.flute_length_mm - p0.z) / dz_span,
            );
            if (niv.valid) {
                let qn0 = mix(p0, p1, niv.lo);
                let qn1 = mix(p0, p1, niv.hi);
                let cand = intersect_horizontal_line_with_capsule_2d(
                    proj_horiz(qn0, swap_xy),
                    proj_horiz(qn1, swap_xy),
                    r_neck,
                    fixed_u,
                );
                if (cand.valid) {
                    lo = min(lo, cand.lo);
                    hi = max(hi, cand.hi);
                    has_hit = true;
                }
            }
        }

        let fiv = clamp_unit_interval(
            (pz - prof.flute_length_mm - p0.z) / dz_span,
            (pz - p0.z) / dz_span,
        );
        if (fiv.valid) {
            let qf0 = mix(p0, p1, fiv.lo);
            let qf1 = mix(p0, p1, fiv.hi);
            let sub_dz = abs(qf1.z - qf0.z);
            let z_start = prof.param1;
            let tooth_step = prof.param2;
            let n_teeth = max(prof.teeth_count, 1u);
            if (sub_dz <= 1e-6) {
                let r_mid = max(thread_mill_tooth_radius_at_z(prof, pz - qf0.z) - radial_shrink, 0.0);
                if (r_mid > 1e-6) {
                    let cand = intersect_horizontal_line_with_capsule_2d(
                        proj_horiz(qf0, swap_xy),
                        proj_horiz(qf1, swap_xy),
                        r_mid,
                        fixed_u,
                    );
                    if (cand.valid) {
                        lo = min(lo, cand.lo);
                        hi = max(hi, cand.hi);
                        has_hit = true;
                    }
                }
            } else {
                let step = max(min(u.dz_step_mm, 0.15 * tooth_step), 0.02);
                let n_sub = clamp(u32(ceil(sub_dz / step)), 2u, 32u);
                for (var s: u32 = 0u; s < n_sub; s = s + 1u) {
                    let u0 = f32(s) / f32(n_sub);
                    let u1 = f32(s + 1u) / f32(n_sub);
                    let a = mix(qf0, qf1, u0);
                    let b = mix(qf0, qf1, u1);
                    let r_mid = max(
                        thread_mill_tooth_radius_at_z(prof, pz - 0.5 * (a.z + b.z)) - radial_shrink,
                        0.0,
                    );
                    if (r_mid > 1e-6) {
                        let cand = intersect_horizontal_line_with_capsule_2d(
                            proj_horiz(a, swap_xy),
                            proj_horiz(b, swap_xy),
                            r_mid,
                            fixed_u,
                        );
                        if (cand.valid) {
                            lo = min(lo, cand.lo);
                            hi = max(hi, cand.hi);
                            has_hit = true;
                        }
                    }
                }
                let r_crest = max(prof.cutting_radius_mm - radial_shrink, 0.0);
                if (r_crest > 1e-6) {
                    for (var k: u32 = 0u; k < n_teeth; k = k + 1u) {
                        let z_crest = z_start + (f32(k) + 0.5) * tooth_step;
                        let tc = (pz - z_crest - p0.z) / dz_span;
                        if (tc >= fiv.lo - 1e-6 && tc <= fiv.hi + 1e-6) {
                            let pt = proj_horiz(mix(p0, p1, clamp(tc, fiv.lo, fiv.hi)), swap_xy);
                            let cand = intersect_horizontal_line_with_capsule_2d(pt, pt, r_crest, fixed_u);
                            if (cand.valid) {
                                lo = min(lo, cand.lo);
                                hi = max(hi, cand.hi);
                                has_hit = true;
                            }
                        }
                    }
                }
            }
        }

        if (has_hit && hi > lo) {
            return Interval1D(true, lo, hi);
        }
        return miss;
    }

    let q0 = mix(p0, p1, iv.lo);
    let q1 = mix(p0, p1, iv.hi);
    let has_relieved_neck = (prof.flags & PROF_FLAG_RELIEVED_NECK) != 0u;
    let r0 = max(max(eval_radius_at_dz(prof, pz - q0.z), 0.0) - radial_shrink, 0.0);
    let r1 = max(max(eval_radius_at_dz(prof, pz - q1.z), 0.0) - radial_shrink, 0.0);
    if (!has_relieved_neck && abs(r0 - r1) <= 1e-6 && r0 > 1e-6) {
        return intersect_horizontal_line_with_capsule_2d(
            proj_horiz(q0, swap_xy),
            proj_horiz(q1, swap_xy),
            r0,
            fixed_u,
        );
    }

    let sub_dz = abs(q1.z - q0.z);
    let n_sub = clamp(u32(ceil(sub_dz / max(u.dz_step_mm, 0.05))), 2u, 24u);
    var has_hit = false;
    var lo: f32 = 1e30;
    var hi: f32 = -1e30;
    for (var s: u32 = 0u; s < n_sub; s = s + 1u) {
        let u0 = f32(s) / f32(n_sub);
        let u1 = f32(s + 1u) / f32(n_sub);
        let a = mix(q0, q1, u0);
        let b = mix(q0, q1, u1);
        let r_mid = max(max(eval_radius_at_dz(prof, pz - 0.5 * (a.z + b.z)), 0.0) - radial_shrink, 0.0);
        if (r_mid > 1e-6) {
            let cand = intersect_horizontal_line_with_capsule_2d(
                proj_horiz(a, swap_xy),
                proj_horiz(b, swap_xy),
                r_mid,
                fixed_u,
            );
            if (cand.valid) {
                lo = min(lo, cand.lo);
                hi = max(hi, cand.hi);
                has_hit = true;
            }
        }
    }
    if (prof.shape_kind == SHAPE_LOLLIPOP) {
        let sphere_r = select(prof.cutting_radius_mm, prof.param2, prof.param2 > 1e-6);
        let tc = (pz - sphere_r - p0.z) / dz_span;
        if (tc >= iv.lo - 1e-6 && tc <= iv.hi + 1e-6) {
            let pt = proj_horiz(mix(p0, p1, clamp(tc, iv.lo, iv.hi)), swap_xy);
            let r_eq = max(prof.cutting_radius_mm - radial_shrink, 0.0);
            if (r_eq > 1e-6) {
                let cand = intersect_horizontal_line_with_capsule_2d(pt, pt, r_eq, fixed_u);
                if (cand.valid) {
                    lo = min(lo, cand.lo);
                    hi = max(hi, cand.hi);
                    has_hit = true;
                }
            }
        }
    }
    if (has_hit && hi > lo) {
        return Interval1D(true, lo, hi);
    }
    return miss;
}

fn eval_z_ray_cylinder_interval(
    p0: vec3<f32>,
    p1: vec3<f32>,
    px: f32,
    py: f32,
    cyl_radius: f32,
    cyl_height: f32,
) -> Interval1D {
    let miss = Interval1D(false, 0.0, 0.0);
    if (cyl_radius <= 1e-6) {
        return miss;
    }
    let p0_xy = p0.xy;
    let p1_xy = p1.xy;
    let q_xy = vec2<f32>(px, py);
    let d_xy = p1_xy - p0_xy;
    let len2 = dot(d_xy, d_xy);

    var t0: f32 = 0.0;
    var t1: f32 = 1.0;
    if (len2 <= 1e-10) {
        if (length(q_xy - p0_xy) > cyl_radius + 1e-6) {
            return miss;
        }
    } else {
        let f = p0_xy - q_xy;
        let b = 2.0 * dot(f, d_xy);
        let c = dot(f, f) - cyl_radius * cyl_radius;
        let disc = b * b - 4.0 * len2 * c;
        if (disc < 0.0) {
            return miss;
        }
        let sq = sqrt(disc);
        let inv2a = 0.5 / len2;
        let raw_t0 = (-b - sq) * inv2a;
        let raw_t1 = (-b + sq) * inv2a;
        if (raw_t1 < 0.0 || raw_t0 > 1.0) {
            return miss;
        }
        t0 = max(raw_t0, 0.0);
        t1 = min(raw_t1, 1.0);
    }

    let dz_span = p1.z - p0.z;
    let z0 = p0.z + t0 * dz_span;
    let z1 = p0.z + t1 * dz_span;
    return Interval1D(true, min(z0, z1), max(z0, z1) + cyl_height);
}

fn eval_z_ray_cut_interval(
    p0: vec3<f32>,
    p1: vec3<f32>,
    prof: GpuToolProfile,
    px: f32,
    py: f32,
    radius_limit: f32,
) -> Interval1D {
    let miss = Interval1D(false, 0.0, 0.0);
    let p0_xy = p0.xy;
    let p1_xy = p1.xy;
    let q_xy = vec2<f32>(px, py);
    let d_xy = p1_xy - p0_xy;
    let len2 = dot(d_xy, d_xy);

    var t_enter: f32 = 0.0;
    var t_exit: f32 = 1.0;
    var t_closest: f32 = 0.0;
    if (len2 <= 1e-10) {
        if (length(q_xy - p0_xy) > radius_limit + 1e-6) {
            return miss;
        }
    } else {
        let f = p0_xy - q_xy;
        let b = 2.0 * dot(f, d_xy);
        let c = dot(f, f) - radius_limit * radius_limit;
        let disc = b * b - 4.0 * len2 * c;
        if (disc < 0.0) {
            return miss;
        }
        let sq = sqrt(disc);
        let inv2a = 0.5 / len2;
        let raw_t0 = (-b - sq) * inv2a;
        let raw_t1 = (-b + sq) * inv2a;
        if (raw_t1 < 0.0 || raw_t0 > 1.0) {
            return miss;
        }
        t_enter = max(raw_t0, 0.0);
        t_exit = min(raw_t1, 1.0);
        t_closest = clamp(-dot(f, d_xy) / len2, t_enter, t_exit);
    }

    let dz_span = p1.z - p0.z;
    if (abs(dz_span) <= 1e-6) {
        let pt = mix(p0_xy, p1_xy, t_closest);
        let r = min(length(q_xy - pt), prof.cutting_radius_mm);
        let dz_iv = eval_profile_z_interval(prof, r);
        if (!dz_iv.valid) {
            return miss;
        }
        let z_tip = p0.z;
        return Interval1D(true, z_tip + dz_iv.lo, z_tip + dz_iv.hi);
    }

    if ((prof.flags & PROF_FLAG_FLAT_ENDMILL) != 0u) {
        let t_min_z = select(t_enter, t_exit, dz_span < 0.0);
        let t_max_z = select(t_exit, t_enter, dz_span < 0.0);
        let z_tip_lo = p0.z + t_min_z * dz_span;
        let z_tip_hi = p0.z + t_max_z * dz_span;
        return Interval1D(true, z_tip_lo, z_tip_hi + prof.flute_length_mm);
    }

    var best_z_cut: f32 = 1e30;
    var max_z_hi: f32 = -1e30;
    var found = false;
    for (var s: u32 = 0u; s <= 9u; s = s + 1u) {
        var t: f32;
        if (s <= 8u) {
            t = t_enter + (f32(s) / 8.0) * (t_exit - t_enter);
        } else {
            t = t_closest;
        }
        let pt = mix(p0_xy, p1_xy, t);
        let r = min(length(q_xy - pt), prof.cutting_radius_mm);
        let dz_iv = eval_profile_z_interval(prof, r);
        if (dz_iv.valid) {
            let z_tip = p0.z + t * dz_span;
            let z_cut = z_tip + dz_iv.lo;
            let z_hi = z_tip + dz_iv.hi;
            if (!found || z_cut < best_z_cut) {
                best_z_cut = z_cut;
            }
            if (!found || z_hi > max_z_hi) {
                max_z_hi = z_hi;
            }
            found = true;
        }
    }
    if (found) {
        return Interval1D(true, best_z_cut, max(max_z_hi, best_z_cut));
    }
    return miss;
}

// Conservative bounding check used to test whether a chord in this tile could touch an
// already-overflowed ray at `(pu, pv)` before flagging it for exact CPU replay.
fn chord_bbox_hits_ray(ch: GpuSweepChord, prof: GpuToolProfile, verify: bool, pu: f32, pv: f32) -> bool {
    let p0 = ch.p0_rot.xyz;
    let p1 = ch.p1_rot.xyz;
    let r_cut = prof.cutting_radius_mm;
    let max_h = horizontal_cut_max_height(prof);
    if (u.axis_id == 2u) {
        let r_chk = select(r_cut, max(r_cut, prof.holder_radius_mm), verify) + 1e-3;
        return pu >= min(p0.x, p1.x) - r_chk && pu <= max(p0.x, p1.x) + r_chk
            && pv >= min(p0.y, p1.y) - r_chk && pv <= max(p0.y, p1.y) + r_chk;
    }
    if (u.axis_id == 0u) {
        let r_chk = r_cut + 1e-3;
        return pu >= min(p0.y, p1.y) - r_chk && pu <= max(p0.y, p1.y) + r_chk
            && pv >= min(p0.z, p1.z) - 1e-3 && pv <= max(p0.z, p1.z) + max_h + 1e-3;
    }
    let r_chk = r_cut + 1e-3;
    return pu >= min(p0.x, p1.x) - r_chk && pu <= max(p0.x, p1.x) + r_chk
        && pv >= min(p0.z, p1.z) - 1e-3 && pv <= max(p0.z, p1.z) + max_h + 1e-3;
}

@compute @workgroup_size(16, 16, 1)
fn main(
    @builtin(global_invocation_id) gid: vec3<u32>,
    @builtin(workgroup_id) wid: vec3<u32>,
) {
    let iu = gid.x;
    let iv = gid.y;
    if (iu >= u.nu || iv >= u.nv) {
        return;
    }

    let tile_id = wid.y * u.tiles_u + wid.x;
    let start = tile_data[tile_id];
    let end = tile_data[tile_id + 1u];
    if (start >= end) {
        return;
    }

    let local_ray_idx = iv * u.nu + iu;
    let global_ray_idx = u.plane_ray_offset + local_ray_idx;
    var cnt = counts[global_ray_idx];
    if (cnt == 0u) {
        return;
    }

    let pu = (f32(iu) + 0.5) * u.du_mm;
    let pv = (f32(iv) + 0.5) * u.dv_mm;
    let list_base = u.num_tiles + 1u;

    // Already in CPU overflow side-table (`cnt > 4`): flag only if a chord in this tile overlaps
    // this ray's bounding range, then exit without mutating GPU inline slots!
    if (cnt > INLINE_K) {
        for (var k: u32 = start; k < end; k = k + 1u) {
            let ch = chords[tile_data[list_base + k]];
            let prof = profiles[ch.info.y];
            let verify = (ch.info.z & CHORD_FLAG_VERIFY) != 0u;
            if (chord_bbox_hits_ray(ch, prof, verify, pu, pv)) {
                flag_overflow_ray(local_ray_idx);
                break;
            }
        }
        return;
    }

    let slot_base = global_ray_idx * INLINE_K;
    var local_spans: array<FlatDexelSpan, 4>;
    for (var s: u32 = 0u; s < INLINE_K; s = s + 1u) {
        if (s < cnt) {
            local_spans[s] = slots[slot_base + s];
        } else {
            local_spans[s] = FlatDexelSpan(0.0, 0.0, 0u, 0u);
        }
    }

    var ray_mutated = false;
    var overflowed = false;

    for (var k: u32 = start; k < end; k = k + 1u) {
        if (cnt == 0u || overflowed) {
            break;
        }
        let ch = chords[tile_data[list_base + k]];
        let prof = profiles[ch.info.y];
        let op_idx = ch.info.x;
        let flags = ch.info.z;
        let chord_index = ch.info.w;
        let verify = (flags & CHORD_FLAG_VERIFY) != 0u;
        let is_rapid = (flags & CHORD_FLAG_RAPID) != 0u;
        let is_thread_mill = (prof.flags & PROF_FLAG_THREAD_MILL) != 0u;
        let has_relieved_neck = (prof.flags & PROF_FLAG_RELIEVED_NECK) != 0u;
        let rapid_tol = ch.sweep_and_tols.y;
        let gouge_tol = ch.sweep_and_tols.z;
        let wall_skin_tol = ch.sweep_and_tols.w;
        let p0 = ch.p0_rot.xyz;
        let p1 = ch.p1_rot.xyz;

        let r_cut = prof.cutting_radius_mm;
        let neck_skin = max(prof.neck_radius_mm - wall_skin_tol, 0.0);
        let shank_skin = max(prof.shank_radius_mm - wall_skin_tol, 0.0);
        let r_inner_skin = max(r_cut - wall_skin_tol, 0.0);

        if (u.axis_id == 2u) {
            // === Z-PLANE (`u = X`, `v = Y`, `w = Z`) ===
            let px = pu;
            let py = pv;
            let r_chk = select(r_cut, max(r_cut, prof.holder_radius_mm), verify) + 1e-3;
            if (px < min(p0.x, p1.x) - r_chk || px > max(p0.x, p1.x) + r_chk
                || py < min(p0.y, p1.y) - r_chk || py > max(p0.y, p1.y) + r_chk) {
                continue;
            }

            if (verify) {
                let p0_xy = p0.xy;
                let p1_xy = p1.xy;
                let d_xy = p1_xy - p0_xy;
                let l2 = dot(d_xy, d_xy);
                let q_xy = vec2<f32>(px, py);
                var t_c: f32 = 0.0;
                if (l2 > 1e-10) {
                    t_c = clamp(dot(q_xy - p0_xy, d_xy) / l2, 0.0, 1.0);
                }
                let c_xy = mix(p0_xy, p1_xy, t_c);
                let dist_xy = length(q_xy - c_xy);
                let z_tip_here = select(min(p0.z, p1.z), p0.z + t_c * (p1.z - p0.z), l2 > 1e-10);
                let top_z = local_spans[cnt - 1u].max_local;

                if (dist_xy <= prof.holder_radius_mm) {
                    let holder_z = z_tip_here + prof.stickout_length_mm;
                    let pen = top_z - holder_z;
                    if (pen > 0.05) {
                        record_violation(SLOT_HOLDER, pen, chord_index, local_ray_idx, holder_z);
                    }
                }
                if (!is_rapid && dist_xy <= shank_skin) {
                    let reach_top = z_tip_here + prof.effective_reach_mm;
                    let rub = top_z - reach_top;
                    if (rub > 0.15) {
                        record_violation(SLOT_FLUTE, rub, chord_index, local_ray_idx, reach_top);
                    }
                }
                if (has_relieved_neck && !is_rapid && dist_xy <= neck_skin) {
                    let neck_lo = z_tip_here + prof.flute_length_mm + 0.05;
                    let neck_hi = z_tip_here + prof.neck_length_mm;
                    if (neck_hi > neck_lo) {
                        let ov = query_overlap_local(&local_spans, cnt, neck_lo, neck_hi);
                        if (ov > 0.10) {
                            record_violation(SLOT_NECK, ov, chord_index, local_ray_idx, neck_lo);
                        }
                    }
                }
            }

            if (is_thread_mill) {
                let p0_xy = p0.xy;
                let p1_xy = p1.xy;
                let q_xy = vec2<f32>(px, py);
                let d_xy = p1_xy - p0_xy;
                let len2 = dot(d_xy, d_xy);
                var t_enter: f32 = 0.0;
                var t_exit: f32 = 1.0;
                var t_closest: f32 = 0.0;
                var in_cyl = true;
                if (len2 <= 1e-10) {
                    if (length(q_xy - p0_xy) > r_cut + 1e-6) {
                        in_cyl = false;
                    }
                } else {
                    let f = p0_xy - q_xy;
                    let b = 2.0 * dot(f, d_xy);
                    let c = dot(f, f) - r_cut * r_cut;
                    let disc = b * b - 4.0 * len2 * c;
                    if (disc < 0.0) {
                        in_cyl = false;
                    } else {
                        let sq = sqrt(disc);
                        let inv2a = 0.5 / len2;
                        let raw_t0 = (-b - sq) * inv2a;
                        let raw_t1 = (-b + sq) * inv2a;
                        if (raw_t1 < 0.0 || raw_t0 > 1.0) {
                            in_cyl = false;
                        } else {
                            t_enter = max(raw_t0, 0.0);
                            t_exit = min(raw_t1, 1.0);
                            t_closest = clamp(-dot(f, d_xy) / len2, t_enter, t_exit);
                        }
                    }
                }

                if (in_cyl) {
                    let dz_span = p1.z - p0.z;
                    let z_start = prof.param1;
                    let tooth_step = prof.param2;
                    let tan_half = prof.param3;
                    let n_teeth = max(prof.teeth_count, 1u);

                    if (verify && u.has_target != 0u) {
                        let z_in = eval_z_ray_cylinder_interval(
                            p0, p1, px, py, neck_skin, prof.neck_length_mm,
                        );
                        if (z_in.valid) {
                            let gh = query_target_gouge(global_ray_idx, z_in.lo, z_in.hi, gouge_tol);
                            if (gh.valid) {
                                record_violation(SLOT_GOUGE, gh.depth, chord_index, local_ray_idx, gh.coord);
                            }
                        }
                    }

                    if (len2 <= 1e-10 || abs(dz_span) <= 1e-6) {
                        let pt = mix(p0_xy, p1_xy, t_closest);
                        let r = length(q_xy - pt);
                        let z_lo_tip = min(p0.z, p1.z);
                        let z_hi_tip = max(p0.z, p1.z);
                        if (r <= prof.neck_radius_mm + 1e-6) {
                            let zlo = z_lo_tip;
                            let zhi = z_hi_tip + prof.neck_length_mm;
                            if (verify && is_rapid) {
                                let ov = query_overlap_local(&local_spans, cnt, zlo, zhi);
                                if (ov > rapid_tol) {
                                    record_violation(SLOT_RAPID, ov, chord_index, local_ray_idx, zlo);
                                }
                            }
                            let res = subtract_interval_local(&local_spans, &cnt, zlo, zhi, op_idx);
                            if (res < 0) { overflowed = true; } else if (res > 0) { ray_mutated = true; }
                        } else if (r <= r_cut + 1e-6) {
                            let half_w = max(r_cut - r, 0.0) * tan_half;
                            if (verify && is_rapid) {
                                for (var tk: u32 = 0u; tk < n_teeth; tk = tk + 1u) {
                                    let zk_lo = z_start + f32(tk) * tooth_step;
                                    let zk_hi = zk_lo + tooth_step;
                                    let z_crest = zk_lo + 0.5 * tooth_step;
                                    let lo = max(z_crest - half_w, zk_lo) + z_lo_tip;
                                    let hi = min(z_crest + half_w, zk_hi) + z_hi_tip;
                                    if (hi > lo + 1e-6) {
                                        let ov = query_overlap_local(&local_spans, cnt, lo, hi);
                                        if (ov > rapid_tol) {
                                            record_violation(SLOT_RAPID, ov, chord_index, local_ray_idx, lo);
                                        }
                                    }
                                }
                            }
                            for (var tk: u32 = 0u; tk < n_teeth; tk = tk + 1u) {
                                let zk_lo = z_start + f32(tk) * tooth_step;
                                let zk_hi = zk_lo + tooth_step;
                                let z_crest = zk_lo + 0.5 * tooth_step;
                                let lo = max(z_crest - half_w, zk_lo) + z_lo_tip;
                                let hi = min(z_crest + half_w, zk_hi) + z_hi_tip;
                                if (hi > lo + 1e-6) {
                                    let res = subtract_interval_local(&local_spans, &cnt, lo, hi, op_idx);
                                    if (res < 0) { overflowed = true; break; } else if (res > 0) { ray_mutated = true; }
                                }
                            }
                        }
                    } else {
                        var sample_r: array<f32, 6>;
                        var sample_z: array<f32, 6>;
                        var ts: array<f32, 6>;
                        ts[0] = t_enter;
                        ts[1] = t_enter + 0.25 * (t_exit - t_enter);
                        ts[2] = t_enter + 0.50 * (t_exit - t_enter);
                        ts[3] = t_enter + 0.75 * (t_exit - t_enter);
                        ts[4] = t_exit;
                        ts[5] = t_closest;
                        var neck_lo: f32 = 1e30;
                        var neck_hi: f32 = -1e30;
                        for (var si: u32 = 0u; si < 6u; si = si + 1u) {
                            let pt = mix(p0_xy, p1_xy, ts[si]);
                            let r = length(q_xy - pt);
                            let z_tip = p0.z + ts[si] * dz_span;
                            sample_r[si] = r;
                            sample_z[si] = z_tip;
                            if (r <= prof.neck_radius_mm + 1e-6) {
                                neck_lo = min(neck_lo, z_tip);
                                neck_hi = max(neck_hi, z_tip + prof.neck_length_mm);
                            }
                        }
                        if (verify && is_rapid) {
                            for (var tk: u32 = 0u; tk < n_teeth; tk = tk + 1u) {
                                let zk_lo = z_start + f32(tk) * tooth_step;
                                let zk_hi = zk_lo + tooth_step;
                                let z_crest = zk_lo + 0.5 * tooth_step;
                                var t_lo: f32 = 1e30;
                                var t_hi: f32 = -1e30;
                                for (var si: u32 = 0u; si < 6u; si = si + 1u) {
                                    let r = sample_r[si];
                                    if (r <= r_cut + 1e-6) {
                                        let half_w = max(r_cut - r, 0.0) * tan_half;
                                        let lo = sample_z[si] + max(z_crest - half_w, zk_lo);
                                        let hi = sample_z[si] + min(z_crest + half_w, zk_hi);
                                        if (hi > lo + 1e-6) {
                                            t_lo = min(t_lo, lo);
                                            t_hi = max(t_hi, hi);
                                        }
                                    }
                                }
                                if (t_hi > t_lo + 1e-6) {
                                    let ov = query_overlap_local(&local_spans, cnt, t_lo, t_hi);
                                    if (ov > rapid_tol) {
                                        record_violation(SLOT_RAPID, ov, chord_index, local_ray_idx, t_lo);
                                    }
                                }
                            }
                            if (neck_hi > neck_lo + 1e-6) {
                                let ov = query_overlap_local(&local_spans, cnt, neck_lo, neck_hi);
                                if (ov > rapid_tol) {
                                    record_violation(SLOT_RAPID, ov, chord_index, local_ray_idx, neck_lo);
                                }
                            }
                        }
                        for (var tk: u32 = 0u; tk < n_teeth; tk = tk + 1u) {
                            let zk_lo = z_start + f32(tk) * tooth_step;
                            let zk_hi = zk_lo + tooth_step;
                            let z_crest = zk_lo + 0.5 * tooth_step;
                            var t_lo: f32 = 1e30;
                            var t_hi: f32 = -1e30;
                            for (var si: u32 = 0u; si < 6u; si = si + 1u) {
                                let r = sample_r[si];
                                if (r <= r_cut + 1e-6) {
                                    let half_w = max(r_cut - r, 0.0) * tan_half;
                                    let lo = sample_z[si] + max(z_crest - half_w, zk_lo);
                                    let hi = sample_z[si] + min(z_crest + half_w, zk_hi);
                                    if (hi > lo + 1e-6) {
                                        t_lo = min(t_lo, lo);
                                        t_hi = max(t_hi, hi);
                                    }
                                }
                            }
                            if (t_hi > t_lo + 1e-6) {
                                let res = subtract_interval_local(&local_spans, &cnt, t_lo, t_hi, op_idx);
                                if (res < 0) { overflowed = true; break; } else if (res > 0) { ray_mutated = true; }
                            }
                        }
                        if (!overflowed && neck_hi > neck_lo + 1e-6) {
                            let res = subtract_interval_local(&local_spans, &cnt, neck_lo, neck_hi, op_idx);
                            if (res < 0) { overflowed = true; } else if (res > 0) { ray_mutated = true; }
                        }
                    }
                }
            } else {
                let z_iv = eval_z_ray_cut_interval(p0, p1, prof, px, py, r_cut);
                if (z_iv.valid) {
                    if (verify && is_rapid) {
                        let ov = query_overlap_local(&local_spans, cnt, z_iv.lo, z_iv.hi);
                        if (ov > rapid_tol) {
                            record_violation(SLOT_RAPID, ov, chord_index, local_ray_idx, z_iv.lo);
                        }
                    }
                    if (verify && u.has_target != 0u) {
                        let z_in = eval_z_ray_cut_interval(p0, p1, prof, px, py, r_inner_skin);
                        if (z_in.valid) {
                            let gh = query_target_gouge(global_ray_idx, z_in.lo, z_in.hi, gouge_tol);
                            if (gh.valid) {
                                record_violation(SLOT_GOUGE, gh.depth, chord_index, local_ray_idx, gh.coord);
                            }
                        }
                    }
                    let res = subtract_interval_local(&local_spans, &cnt, z_iv.lo, z_iv.hi, op_idx);
                    if (res < 0) { overflowed = true; } else if (res > 0) { ray_mutated = true; }
                }
            }
        } else {
            // === HORIZONTAL PLANES: `axis_id == 0u` (X: u=Y, v=Z, w=X) or `1u` (Y: u=X, v=Z, w=Y) ===
            let swap_xy = (u.axis_id == 1u);
            let pz = pv;
            let u0_c = select(p0.y, p0.x, swap_xy);
            let u1_c = select(p1.y, p1.x, swap_xy);
            let max_h = horizontal_cut_max_height(prof);
            if (pu < min(u0_c, u1_c) - r_cut - 1e-3 || pu > max(u0_c, u1_c) + r_cut + 1e-3
                || pz < min(p0.z, p1.z) - 1e-3 || pz > max(p0.z, p1.z) + max_h + 1e-3) {
                continue;
            }

            let w_iv = eval_horizontal_ray_cut_interval(p0, p1, prof, 0.0, pz, pu, swap_xy);
            if (w_iv.valid) {
                if (verify && is_rapid) {
                    let ov = query_overlap_local(&local_spans, cnt, w_iv.lo, w_iv.hi);
                    if (ov > rapid_tol) {
                        record_violation(
                            SLOT_RAPID,
                            ov,
                            chord_index,
                            local_ray_idx,
                            0.5 * (w_iv.lo + w_iv.hi),
                        );
                    }
                }
                if (verify && u.has_target != 0u && pz >= min(p0.z, p1.z) + gouge_tol) {
                    var w_in: Interval1D;
                    if (is_thread_mill) {
                        w_in = eval_horizontal_ray_cylinder_interval(
                            p0, p1, neck_skin, prof.neck_length_mm, pz, pu, swap_xy,
                        );
                    } else {
                        let flute_wall_skin = wall_skin_tol + max(gouge_tol - 0.02, 0.0);
                        w_in = eval_horizontal_ray_cut_interval(
                            p0, p1, prof, flute_wall_skin, pz, pu, swap_xy,
                        );
                    }
                    if (w_in.valid) {
                        let wall_skin = gouge_tol + wall_skin_tol;
                        let gh = query_target_gouge(global_ray_idx, w_in.lo, w_in.hi, wall_skin);
                        if (gh.valid) {
                            record_violation(SLOT_GOUGE, gh.depth, chord_index, local_ray_idx, gh.coord);
                        }
                    }
                }
                let res = subtract_interval_local(&local_spans, &cnt, w_iv.lo, w_iv.hi, op_idx);
                if (res < 0) { overflowed = true; } else if (res > 0) { ray_mutated = true; }
            }
        }
    }

    // On overflow, leave pre-batch `slots`/`counts` untouched in GPU memory so CPU replay reads
    // the exact pre-batch state of this ray and resolves this batch losslessly!
    if (overflowed) {
        flag_overflow_ray(local_ray_idx);
        return;
    }

    if (ray_mutated) {
        for (var s: u32 = 0u; s < INLINE_K; s = s + 1u) {
            slots[slot_base + s] = local_spans[s];
        }
        counts[global_ray_idx] = cnt;
        atomicStore(&accum_and_overflow[WORD_MUTATED], 1u);
    }
}
