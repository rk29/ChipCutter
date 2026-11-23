// Axis-Generic Tiled 16x16 Flat Fixed-K (K = 4) Tri-Dexel WGSL Compute Sweeper
// Parameterized by `u.axis_id` (`0 = X`, `1 = Y`, `2 = Z`) using the exact `(u, v, w)`
// coordinate permutation of `crates/chip-stock/src/tridexel/sweep.rs` (zero X/Y/Z duplication).
// Covers all 9 cutter geometries (including single-point & multi-form ThreadMill undercuts),
// inline fixed-K span subtraction in scalar/vec4 registers (`LocalSpans`, zero indexable temp
// arrays), atomic Rules 1-4 + relieved-neck verification, and a lossless hybrid overflow bitset
// safety valve for rare `> 4`-span rays.
//
// Split into two compact compute entry points for fast, reliable Windows FXC (`D3DCompile` SM 5.0)
// compilation without IR/inlining explosion:
// - `main`: cardinal 3-axis and indexed 4th-axis linear chords (used by all 3-axis/indexed ops)
// - `main_oriented`: general oriented 4th-axis rotary poses (lazily compiled on demand)

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
const CHORD_FLAG_CHECK_HOLDER: u32 = 8u;
const CHORD_FLAG_ORIENTED_POSE: u32 = 16u;
const CHORD_CARDINAL_SHIFT: u32 = 8u;
const CHORD_CARDINAL_MASK: u32 = 7u;

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

// Register-resident fixed-K (`K = 4`) inline span set (avoids FXC `dcl_indexableTemp` arrays).
struct LocalSpans {
    s0: FlatDexelSpan,
    s1: FlatDexelSpan,
    s2: FlatDexelSpan,
    s3: FlatDexelSpan,
    cnt: u32,
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

fn zero_span() -> FlatDexelSpan {
    return FlatDexelSpan(0.0, 0.0, 0u, 0u);
}

fn load_local_spans(slot_base: u32, cnt: u32) -> LocalSpans {
    let z = zero_span();
    var ls = LocalSpans(z, z, z, z, cnt);
    if (cnt > 0u) { ls.s0 = slots[slot_base]; }
    if (cnt > 1u) { ls.s1 = slots[slot_base + 1u]; }
    if (cnt > 2u) { ls.s2 = slots[slot_base + 2u]; }
    if (cnt > 3u) { ls.s3 = slots[slot_base + 3u]; }
    return ls;
}

fn store_local_spans(slot_base: u32, ls: LocalSpans) {
    slots[slot_base] = ls.s0;
    slots[slot_base + 1u] = ls.s1;
    slots[slot_base + 2u] = ls.s2;
    slots[slot_base + 3u] = ls.s3;
}

fn get_local_span(ls: LocalSpans, idx: u32) -> FlatDexelSpan {
    var out = ls.s3;
    if (idx == 0u) {
        out = ls.s0;
    } else if (idx == 1u) {
        out = ls.s1;
    } else if (idx == 2u) {
        out = ls.s2;
    }
    return out;
}

fn push_local_span(out: ptr<function, LocalSpans>, s: FlatDexelSpan) -> bool {
    let n = (*out).cnt;
    var ok = true;
    if (n == 0u) {
        (*out).s0 = s;
    } else if (n == 1u) {
        (*out).s1 = s;
    } else if (n == 2u) {
        (*out).s2 = s;
    } else if (n == 3u) {
        (*out).s3 = s;
    } else {
        ok = false;
    }
    if (ok) {
        (*out).cnt = n + 1u;
    }
    return ok;
}

fn record_violation(slot_base: u32, depth: f32, chord_idx: u32, ray_idx: u32, loc_w: f32) {
    if (depth <= 0.0) {
        return;
    }
    let depth_bits = bitcast<u32>(depth);
    atomicStore(&accum_and_overflow[slot_base], 1u);
    atomicMax(&accum_and_overflow[slot_base + 5u], 0xFFFFFFFFu - chord_idx);
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

fn span_overlap_len(s: FlatDexelSpan, q_min: f32, q_max: f32) -> f32 {
    return max(min(q_max, s.max_local) - max(q_min, s.min_local), 0.0);
}

// Straight-line fixed-K (`K = 4`) overlap length query (zero loops for fast FXC compile).
fn query_overlap_local(ls: LocalSpans, q_min: f32, q_max: f32) -> f32 {
    var total: f32 = 0.0;
    if (q_max > q_min) {
        if (ls.cnt > 0u) { total = total + span_overlap_len(ls.s0, q_min, q_max); }
        if (ls.cnt > 1u) { total = total + span_overlap_len(ls.s1, q_min, q_max); }
        if (ls.cnt > 2u) { total = total + span_overlap_len(ls.s2, q_min, q_max); }
        if (ls.cnt > 3u) { total = total + span_overlap_len(ls.s3, q_min, q_max); }
    }
    return total;
}

fn eval_target_span_gouge(
    best_in: GougeHit,
    ts: FlatDexelSpan,
    q_min: f32,
    q_max: f32,
    skin_mm: f32,
) -> GougeHit {
    var best = best_in;
    if (q_max > ts.min_local && q_min < ts.max_local) {
        let eff_min = ts.min_local + skin_mm;
        let eff_max = ts.max_local - skin_mm;
        if (eff_max > eff_min) {
            let ov_lo = max(q_min, eff_min);
            let ov_hi = min(q_max, eff_max);
            if (ov_hi > ov_lo) {
                let depth = ov_hi - ov_lo;
                var coord = 0.5 * (ov_lo + ov_hi);
                if (q_min > eff_min && q_max >= eff_max) {
                    coord = ov_lo;
                } else if (q_max < eff_max && q_min <= eff_min) {
                    coord = ov_hi;
                }
                if (!best.valid || depth > best.depth) {
                    best = GougeHit(true, depth, coord);
                }
            }
        }
    }
    return best;
}

// Straight-line fixed-K (`K = 4`) target gouge query (zero loops for fast FXC compile).
fn query_target_gouge(
    global_ray_idx: u32,
    q_min: f32,
    q_max: f32,
    skin_mm: f32,
) -> GougeHit {
    var best = GougeHit(false, 0.0, 0.0);
    if (u.has_target != 0u && q_max > q_min) {
        let t_cnt = min(target_counts[global_ray_idx], INLINE_K);
        if (t_cnt > 0u) {
            let base = global_ray_idx * INLINE_K;
            best = eval_target_span_gouge(best, target_slots[base], q_min, q_max, skin_mm);
            if (t_cnt > 1u) {
                best = eval_target_span_gouge(best, target_slots[base + 1u], q_min, q_max, skin_mm);
            }
            if (t_cnt > 2u) {
                best = eval_target_span_gouge(best, target_slots[base + 2u], q_min, q_max, skin_mm);
            }
            if (t_cnt > 3u) {
                best = eval_target_span_gouge(best, target_slots[base + 3u], q_min, q_max, skin_mm);
            }
        }
    }
    return best;
}

// Subtracts `[cut_min, cut_max]` from `*ls` in scalar/vec4 registers (zero local arrays).
// Returns: `0` = unchanged, `1` = mutated inline, `-1` = overflow (> 4 spans).
fn subtract_interval_local(
    ls: ptr<function, LocalSpans>,
    cut_min: f32,
    cut_max: f32,
    op_idx: u32,
) -> i32 {
    let n = (*ls).cnt;
    if (n == 0u || cut_max <= cut_min + 1e-7) {
        return 0;
    }
    let z = zero_span();
    var out = LocalSpans(z, z, z, z, 0u);
    var mutated = false;

    for (var i: u32 = 0u; i < n; i = i + 1u) {
        let s = get_local_span(*ls, i);
        if (cut_max <= s.min_local || cut_min >= s.max_local) {
            if (!push_local_span(&out, s)) {
                return -1;
            }
            continue;
        }
        let ov_min = max(cut_min, s.min_local);
        let ov_max = min(cut_max, s.max_local);
        if (ov_max - ov_min <= 1e-7) {
            if (!push_local_span(&out, s)) {
                return -1;
            }
            continue;
        }
        mutated = true;
        if ((ov_min - s.min_local) > MIN_DEXEL_SPAN_MM) {
            let lo_s = FlatDexelSpan(s.min_local, ov_min, set_max_op(s.op_packed, op_idx), 0u);
            if (!push_local_span(&out, lo_s)) {
                return -1;
            }
        }
        if ((s.max_local - ov_max) > MIN_DEXEL_SPAN_MM) {
            let hi_s = FlatDexelSpan(ov_max, s.max_local, set_min_op(s.op_packed, op_idx), 0u);
            if (!push_local_span(&out, hi_s)) {
                return -1;
            }
        }
    }

    if (!mutated) {
        return 0;
    }
    *ls = out;
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

fn eval_thread_mill_radius_at_dz(prof: GpuToolProfile, dz_mm: f32) -> f32 {
    var out = prof.shank_radius_mm;
    if (dz_mm < -1e-6) {
        out = -1.0;
    } else {
        let dz = max(dz_mm, 0.0);
        if (dz <= prof.flute_length_mm + 1e-6) {
            out = thread_mill_tooth_radius_at_z(prof, dz);
        } else if (dz <= prof.neck_length_mm + 1e-6) {
            out = prof.neck_radius_mm;
        }
    }
    return out;
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
            out = eval_thread_mill_radius_at_dz(prof, dz);
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
        let a_neg = p0 - n;
        let b_neg = p1 - n;
        let dy_neg = b_neg.y - a_neg.y;
        if (abs(dy_neg) > 1e-7) {
            let s = (y_line - a_neg.y) / dy_neg;
            if (s >= 0.0 && s <= 1.0) {
                let xc = a_neg.x + s * (b_neg.x - a_neg.x);
                x_lo = min(x_lo, xc);
                x_hi = max(x_hi, xc);
                has_hit = true;
            }
        }
        let a_pos = p0 + n;
        let b_pos = p1 + n;
        let dy_pos = b_pos.y - a_pos.y;
        if (abs(dy_pos) > 1e-7) {
            let s = (y_line - a_pos.y) / dy_pos;
            if (s >= 0.0 && s <= 1.0) {
                let xc = a_pos.x + s * (b_pos.x - a_pos.x);
                x_lo = min(x_lo, xc);
                x_hi = max(x_hi, xc);
                has_hit = true;
            }
        }
    }

    if (has_hit && x_hi >= x_lo) {
        return Interval1D(true, x_lo, x_hi);
    }
    return miss;
}

fn eval_horizontal_ray_cylinder_interval(
    p0: vec3<f32>,
    p1: vec3<f32>,
    cyl_radius: f32,
    cyl_height: f32,
    pz: f32,
    fixed_u: f32,
) -> Interval1D {
    let miss = Interval1D(false, 0.0, 0.0);
    let min_z = min(p0.z, p1.z);
    let max_z = max(p0.z, p1.z);
    if (cyl_radius <= 1e-6 || pz < min_z - 1e-6 || pz > max_z + cyl_height + 1e-6) {
        return miss;
    }
    let dz_span = p1.z - p0.z;
    var q0 = p0;
    var q1 = p1;
    if (abs(dz_span) <= 1e-6) {
        let dz = pz - p0.z;
        if (dz <= 1e-6 || dz > cyl_height + 1e-6) {
            return miss;
        }
    } else {
        let iv = clamp_unit_interval((pz - cyl_height - p0.z) / dz_span, (pz - p0.z) / dz_span);
        if (!iv.valid) {
            return miss;
        }
        q0 = mix(p0, p1, iv.lo);
        q1 = mix(p0, p1, iv.hi);
    }
    return intersect_horizontal_line_with_capsule_2d(q0.xy, q1.xy, cyl_radius, fixed_u);
}

// Unified horizontal-plane cut interval evaluator (minimal capsule/profile call sites for FXC).
fn eval_horizontal_ray_cut_interval(
    p0: vec3<f32>,
    p1: vec3<f32>,
    prof: GpuToolProfile,
    radial_shrink: f32,
    pz: f32,
    fixed_u: f32,
) -> Interval1D {
    let miss = Interval1D(false, 0.0, 0.0);
    let max_h = horizontal_cut_max_height(prof);
    let min_z = min(p0.z, p1.z);
    let max_z = max(p0.z, p1.z);
    if (pz < min_z - 1e-6 || pz > max_z + max_h + 1e-6) {
        return miss;
    }

    let dz_span = p1.z - p0.z;
    let is_flat_z = abs(dz_span) <= 1e-6;
    if (is_flat_z) {
        let dz = pz - p0.z;
        if (dz <= 1e-6 || dz > max_h + 1e-6) {
            return miss;
        }
    }

    if ((prof.flags & PROF_FLAG_THREAD_MILL) != 0u) {
        var has_hit = false;
        var lo: f32 = 1e30;
        var hi: f32 = -1e30;

        var n_neck: u32 = 0u;
        var qn0 = p0;
        var qn1 = p1;
        let r_neck = max(prof.neck_radius_mm - radial_shrink, 0.0);

        var n_flute: u32 = 0u;
        var n_crests: u32 = 0u;
        var qf0 = p0;
        var qf1 = p1;
        var fiv = Interval1D(false, 0.0, 0.0);
        let z_start = prof.param1;
        let tooth_step = prof.param2;
        let n_teeth = max(prof.teeth_count, 1u);
        let r_crest = max(prof.cutting_radius_mm - radial_shrink, 0.0);

        if (is_flat_z) {
            n_flute = 1u;
        } else {
            let iv = clamp_unit_interval((pz - max_h - p0.z) / dz_span, (pz - p0.z) / dz_span);
            if (!iv.valid) {
                return miss;
            }
            if (r_neck > 1e-6 && prof.neck_length_mm > prof.flute_length_mm + 1e-6) {
                let niv = clamp_unit_interval(
                    (pz - prof.neck_length_mm - p0.z) / dz_span,
                    (pz - prof.flute_length_mm - p0.z) / dz_span,
                );
                if (niv.valid) {
                    qn0 = mix(p0, p1, niv.lo);
                    qn1 = mix(p0, p1, niv.hi);
                    n_neck = 1u;
                }
            }
            fiv = clamp_unit_interval(
                (pz - prof.flute_length_mm - p0.z) / dz_span,
                (pz - p0.z) / dz_span,
            );
            if (fiv.valid) {
                qf0 = mix(p0, p1, fiv.lo);
                qf1 = mix(p0, p1, fiv.hi);
                let sub_dz = abs(qf1.z - qf0.z);
                if (sub_dz <= 1e-6) {
                    n_flute = 1u;
                } else {
                    let step = max(min(u.dz_step_mm, 0.15 * tooth_step), 0.02);
                    n_flute = clamp(u32(ceil(sub_dz / step)), 2u, 32u);
                    if (r_crest > 1e-6) {
                        n_crests = n_teeth;
                    }
                }
            }
        }

        let total_iters = n_neck + n_flute + n_crests;
        for (var s: u32 = 0u; s < total_iters; s = s + 1u) {
            var seg_a = p0.xy;
            var seg_b = p1.xy;
            var seg_r: f32 = 0.0;
            if (s < n_neck) {
                seg_a = qn0.xy;
                seg_b = qn1.xy;
                seg_r = r_neck;
            } else if (s < n_neck + n_flute) {
                let fi = s - n_neck;
                if (is_flat_z) {
                    seg_a = p0.xy;
                    seg_b = p1.xy;
                    let raw_r = eval_thread_mill_radius_at_dz(prof, pz - p0.z);
                    if (raw_r >= 0.0) {
                        seg_r = max(raw_r - radial_shrink, 0.0);
                    }
                } else if (n_flute == 1u) {
                    seg_a = qf0.xy;
                    seg_b = qf1.xy;
                    seg_r = max(thread_mill_tooth_radius_at_z(prof, pz - qf0.z) - radial_shrink, 0.0);
                } else {
                    let u0 = f32(fi) / f32(n_flute);
                    let u1 = f32(fi + 1u) / f32(n_flute);
                    let a = mix(qf0, qf1, u0);
                    let b = mix(qf0, qf1, u1);
                    seg_a = a.xy;
                    seg_b = b.xy;
                    seg_r = max(
                        thread_mill_tooth_radius_at_z(prof, pz - 0.5 * (a.z + b.z)) - radial_shrink,
                        0.0,
                    );
                }
            } else {
                let k = s - n_neck - n_flute;
                let z_crest = z_start + (f32(k) + 0.5) * tooth_step;
                let tc = (pz - z_crest - p0.z) / dz_span;
                if (tc >= fiv.lo - 1e-6 && tc <= fiv.hi + 1e-6) {
                    let pt = mix(p0, p1, clamp(tc, fiv.lo, fiv.hi)).xy;
                    seg_a = pt;
                    seg_b = pt;
                    seg_r = r_crest;
                }
            }
            if (seg_r > 1e-6) {
                let cand = intersect_horizontal_line_with_capsule_2d(seg_a, seg_b, seg_r, fixed_u);
                if (cand.valid) {
                    lo = min(lo, cand.lo);
                    hi = max(hi, cand.hi);
                    has_hit = true;
                }
            }
        }

        if (has_hit && hi > lo) {
            return Interval1D(true, lo, hi);
        }
        return miss;
    }

    var q0 = p0;
    var q1 = p1;
    var iv = Interval1D(true, 0.0, 1.0);
    var n_sub: u32 = 1u;
    var extra_lollipop: u32 = 0u;

    if (!is_flat_z) {
        iv = clamp_unit_interval((pz - max_h - p0.z) / dz_span, (pz - p0.z) / dz_span);
        if (!iv.valid) {
            return miss;
        }
        q0 = mix(p0, p1, iv.lo);
        q1 = mix(p0, p1, iv.hi);
        let has_relieved_neck = (prof.flags & PROF_FLAG_RELIEVED_NECK) != 0u;
        var r0: f32 = 0.0;
        var r1: f32 = 0.0;
        for (var ep: u32 = 0u; ep < 2u; ep = ep + 1u) {
            let zq = select(q1.z, q0.z, ep == 0u);
            let rv = max(max(eval_radius_at_dz(prof, pz - zq), 0.0) - radial_shrink, 0.0);
            if (ep == 0u) { r0 = rv; } else { r1 = rv; }
        }
        if (!has_relieved_neck && abs(r0 - r1) <= 1e-6 && r0 > 1e-6) {
            n_sub = 1u;
        } else {
            let sub_dz = abs(q1.z - q0.z);
            let n_sub_z = u32(ceil(sub_dz / max(u.dz_step_mm, 0.05)));
            let n_sub_r = u32(ceil(abs(r1 - r0) / 0.015));
            n_sub = clamp(max(n_sub_z, n_sub_r), 2u, 32u);
            if (prof.shape_kind == SHAPE_LOLLIPOP) {
                extra_lollipop = 1u;
            }
        }
    }

    var has_hit = false;
    var lo: f32 = 1e30;
    var hi: f32 = -1e30;
    let total_steps = n_sub + extra_lollipop;
    for (var s: u32 = 0u; s < total_steps; s = s + 1u) {
        var seg_a = q0.xy;
        var seg_b = q1.xy;
        var seg_r: f32 = 0.0;
        if (s < n_sub) {
            var a = q0;
            var b = q1;
            if (n_sub > 1u) {
                let u0 = f32(s) / f32(n_sub);
                let u1 = f32(s + 1u) / f32(n_sub);
                a = mix(q0, q1, u0);
                b = mix(q0, q1, u1);
            }
            seg_a = a.xy;
            seg_b = b.xy;
            let raw_r = eval_radius_at_dz(prof, pz - 0.5 * (a.z + b.z));
            if (raw_r >= 0.0) {
                seg_r = max(raw_r - radial_shrink, 0.0);
            }
        } else {
            let sphere_r = select(prof.cutting_radius_mm, prof.param2, prof.param2 > 1e-6);
            let tc = (pz - sphere_r - p0.z) / dz_span;
            if (tc >= iv.lo - 1e-6 && tc <= iv.hi + 1e-6) {
                let pt = mix(p0, p1, clamp(tc, iv.lo, iv.hi)).xy;
                seg_a = pt;
                seg_b = pt;
                seg_r = max(prof.cutting_radius_mm - radial_shrink, 0.0);
            }
        }
        if (seg_r > 1e-6) {
            let cand = intersect_horizontal_line_with_capsule_2d(seg_a, seg_b, seg_r, fixed_u);
            if (cand.valid) {
                lo = min(lo, cand.lo);
                hi = max(hi, cand.hi);
                has_hit = true;
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
    let is_flat_z = abs(dz_span) <= 1e-6;
    if (!is_flat_z && (prof.flags & PROF_FLAG_FLAT_ENDMILL) != 0u) {
        let t_min_z = select(t_enter, t_exit, dz_span < 0.0);
        let t_max_z = select(t_exit, t_enter, dz_span < 0.0);
        let z_tip_lo = p0.z + t_min_z * dz_span;
        let z_tip_hi = p0.z + t_max_z * dz_span;
        return Interval1D(true, z_tip_lo, z_tip_hi + prof.flute_length_mm);
    }

    let n_samples = select(10u, 1u, is_flat_z);
    var best_z_cut: f32 = 1e30;
    var max_z_hi: f32 = -1e30;
    var found = false;
    for (var s: u32 = 0u; s < n_samples; s = s + 1u) {
        var t: f32 = t_closest;
        if (!is_flat_z && s <= 8u) {
            t = t_enter + (f32(s) / 8.0) * (t_exit - t_enter);
        }
        let pt = mix(p0_xy, p1_xy, t);
        let r = min(length(q_xy - pt), prof.cutting_radius_mm);
        let dz_iv = eval_profile_z_interval(prof, r);
        if (dz_iv.valid) {
            let z_tip = select(p0.z + t * dz_span, p0.z, is_flat_z);
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

fn eval_tm_sloped_sample_t(si: u32, t_enter: f32, t_exit: f32, t_closest: f32) -> f32 {
    if (si >= 5u) {
        return t_closest;
    }
    return t_enter + (f32(si) * 0.25) * (t_exit - t_enter);
}

fn eval_tm_sloped_tooth_interval(
    mp0: vec3<f32>,
    mp1: vec3<f32>,
    q_xy: vec2<f32>,
    t_enter: f32,
    t_exit: f32,
    t_closest: f32,
    r_cut: f32,
    tan_half: f32,
    zk_lo: f32,
    zk_hi: f32,
) -> Interval1D {
    let dz_span = mp1.z - mp0.z;
    let z_crest = 0.5 * (zk_lo + zk_hi);
    var t_lo: f32 = 1e30;
    var t_hi: f32 = -1e30;
    for (var si: u32 = 0u; si < 6u; si = si + 1u) {
        let ts = eval_tm_sloped_sample_t(si, t_enter, t_exit, t_closest);
        let r = length(q_xy - mix(mp0.xy, mp1.xy, ts));
        if (r <= r_cut + 1e-6) {
            let z_tip = mp0.z + ts * dz_span;
            let half_w = max(r_cut - r, 0.0) * tan_half;
            let lo = z_tip + max(z_crest - half_w, zk_lo);
            let hi = z_tip + min(z_crest + half_w, zk_hi);
            if (hi > lo + 1e-6) {
                t_lo = min(t_lo, lo);
                t_hi = max(t_hi, hi);
            }
        }
    }
    if (t_hi > t_lo + 1e-6) {
        return Interval1D(true, t_lo, t_hi);
    }
    return Interval1D(false, 0.0, 0.0);
}

fn cardinal_axis(code: u32) -> u32 {
    if (code <= 1u) {
        return 2u;
    }
    if (code <= 3u) {
        return 1u;
    }
    return 0u;
}

fn cardinal_sign(code: u32) -> f32 {
    return select(-1.0, 1.0, (code & 1u) == 0u);
}

fn axis_get(v: vec3<f32>, ax: u32) -> f32 {
    if (ax == 0u) {
        return v.x;
    }
    if (ax == 1u) {
        return v.y;
    }
    return v.z;
}

fn plane_u_axis(ax: u32) -> u32 {
    return select(0u, 1u, ax == 0u);
}

fn plane_v_axis(ax: u32) -> u32 {
    return select(2u, 1u, ax == 2u);
}

fn plane_from_uvw(ax: u32, pu: f32, pv: f32, pw: f32) -> vec3<f32> {
    if (ax == 0u) {
        return vec3<f32>(pw, pu, pv);
    }
    if (ax == 1u) {
        return vec3<f32>(pu, pw, pv);
    }
    return vec3<f32>(pu, pv, pw);
}

fn axis_unit_vec(ax: u32) -> vec3<f32> {
    if (ax == 0u) {
        return vec3<f32>(1.0, 0.0, 0.0);
    }
    if (ax == 1u) {
        return vec3<f32>(0.0, 1.0, 0.0);
    }
    return vec3<f32>(0.0, 0.0, 1.0);
}

fn map_cz_interval_to_w(iv: Interval1D, s_sign: f32) -> Interval1D {
    if (!iv.valid) {
        return iv;
    }
    if (s_sign >= 0.0) {
        return iv;
    }
    return Interval1D(true, -iv.hi, -iv.lo);
}

fn solve_oriented_cylinder_interval(
    a: f32,
    b: f32,
    c: f32,
    h0: f32,
    d_dot_s: f32,
    radius: f32,
    h_lo: f32,
    h_hi: f32,
) -> Interval1D {
    var out = Interval1D(false, 0.0, 0.0);
    if (radius > 1e-6 && h_hi > h_lo + 1e-6) {
        var valid = true;
        var w_lo: f32 = -1e6;
        var w_hi: f32 = 1e6;
        let c_eff = c - radius * radius;
        if (a <= 1e-10) {
            if (c_eff > 0.0) {
                valid = false;
            }
        } else {
            let disc = b * b - a * c_eff;
            if (disc < 0.0) {
                valid = false;
            } else {
                let sd = sqrt(disc);
                w_lo = (-b - sd) / a;
                w_hi = (-b + sd) / a;
            }
        }
        if (valid) {
            if (abs(d_dot_s) <= 1e-10) {
                if (h0 < h_lo || h0 > h_hi) {
                    valid = false;
                }
            } else {
                let t0 = (h_lo - h0) / d_dot_s;
                let t1 = (h_hi - h0) / d_dot_s;
                w_lo = max(w_lo, min(t0, t1));
                w_hi = min(w_hi, max(t0, t1));
            }
            if (valid && w_hi > w_lo + 1e-5) {
                out = Interval1D(true, w_lo, w_hi);
            }
        }
    }
    return out;
}

fn eval_oriented_inside(
    w: f32,
    a: f32,
    b: f32,
    c: f32,
    h0: f32,
    d_dot_s: f32,
    dr_skin: f32,
    prof: GpuToolProfile,
) -> f32 {
    let rho = sqrt(max(a * w * w + 2.0 * b * w + c, 0.0)) + dr_skin;
    let h = h0 + w * d_dot_s;
    let ziv = eval_profile_z_interval(prof, rho);
    if (!ziv.valid) {
        return -1e30;
    }
    return min(h - ziv.lo, ziv.hi - h);
}

// Compact oriented-pose interval evaluator (2 `eval_oriented_inside` call sites total for FXC).
fn eval_oriented_pose_cut_interval(
    v0: vec3<f32>,
    d_axis: vec3<f32>,
    spindle: vec3<f32>,
    a: f32,
    b: f32,
    c: f32,
    h0: f32,
    d_dot_s: f32,
    prof: GpuToolProfile,
    r_lim: f32,
) -> Interval1D {
    var out = Interval1D(false, 0.0, 0.0);
    let r_eff = min(r_lim, prof.cutting_radius_mm);
    if (r_eff <= 1e-5) {
        return out;
    }
    let h_max = horizontal_cut_max_height(prof);
    let cyl = solve_oriented_cylinder_interval(a, b, c, h0, d_dot_s, r_eff, 0.0, h_max);
    if (!cyl.valid) {
        return out;
    }
    if (prof.shape_kind == SHAPE_FLAT) {
        return cyl;
    }
    if (prof.shape_kind == SHAPE_BALL) {
        let r_ball = prof.cutting_radius_mm;
        var out_lo: f32 = 1e30;
        var out_hi: f32 = -1e30;
        let q = v0 - spindle * r_ball;
        let bs = dot(q, d_axis);
        let cs = dot(q, q) - r_eff * r_eff;
        let disc = bs * bs - cs;
        if (disc >= 0.0) {
            let sd = sqrt(disc);
            var s_lo = -bs - sd;
            var s_hi = -bs + sd;
            if (abs(d_dot_s) > 1e-10) {
                let t0 = -h0 / d_dot_s;
                let t1 = (h_max - h0) / d_dot_s;
                s_lo = max(s_lo, min(t0, t1));
                s_hi = min(s_hi, max(t0, t1));
            } else if (h0 < 0.0 || h0 > h_max) {
                s_hi = s_lo - 1.0;
            }
            if (s_hi > s_lo + 1e-5) {
                out_lo = min(out_lo, s_lo);
                out_hi = max(out_hi, s_hi);
            }
        }
        let upper = solve_oriented_cylinder_interval(a, b, c, h0, d_dot_s, r_eff, r_ball, h_max);
        if (upper.valid) {
            out_lo = min(out_lo, upper.lo);
            out_hi = max(out_hi, upper.hi);
        }
        if (out_hi > out_lo + 1e-5) {
            out = Interval1D(true, out_lo, out_hi);
        }
        return out;
    }

    let dr_skin = max(prof.cutting_radius_mm - r_eff, 0.0);
    let w_lo = cyl.lo;
    let w_hi = cyl.hi;
    let w_min_rho = select(0.5 * (w_lo + w_hi), clamp(-b / a, w_lo, w_hi), a > 1e-10);

    var g_lo: f32 = -1e30;
    var g_hi: f32 = -1e30;
    var best_w = w_min_rho;
    var best_g: f32 = -1e30;
    for (var pi: u32 = 0u; pi < 6u; pi = pi + 1u) {
        var wp = w_lo;
        if (pi == 1u) {
            wp = w_hi;
        } else if (pi == 2u) {
            wp = w_min_rho;
        } else if (pi == 3u) {
            wp = w_lo + 0.25 * (w_hi - w_lo);
        } else if (pi == 4u) {
            wp = 0.5 * (w_lo + w_hi);
        } else if (pi == 5u) {
            wp = w_lo + 0.75 * (w_hi - w_lo);
        }
        let gv = eval_oriented_inside(wp, a, b, c, h0, d_dot_s, dr_skin, prof);
        if (pi == 0u) {
            g_lo = gv;
        } else if (pi == 1u) {
            g_hi = gv;
        } else if (gv > best_g) {
            best_g = gv;
            best_w = wp;
        }
    }

    if (g_lo >= 0.0 && g_hi >= 0.0) {
        out = cyl;
    } else {
        var need_left = false;
        var need_right = false;
        var lo_a = w_lo;
        var hi_a = w_hi;
        var lo_b = w_lo;
        var hi_b = w_hi;
        var res_lo = w_lo;
        var res_hi = w_hi;

        if (g_lo >= 0.0) {
            need_right = true;
            lo_b = w_lo;
            hi_b = w_hi;
        } else if (g_hi >= 0.0) {
            need_left = true;
            lo_a = w_lo;
            hi_a = w_hi;
        } else if (best_g > 0.0) {
            need_left = true;
            need_right = true;
            lo_a = w_lo;
            hi_a = best_w;
            lo_b = best_w;
            hi_b = w_hi;
        }

        if (need_left || need_right) {
            let it_start = select(11u, 0u, need_left);
            let it_end = select(11u, 22u, need_right);
            for (var it: u32 = it_start; it < it_end; it = it + 1u) {
                let is_left = it < 11u;
                let mid = select(0.5 * (lo_b + hi_b), 0.5 * (lo_a + hi_a), is_left);
                let inside = eval_oriented_inside(mid, a, b, c, h0, d_dot_s, dr_skin, prof) >= 0.0;
                if (is_left) {
                    if (inside) { hi_a = mid; } else { lo_a = mid; }
                } else {
                    if (inside) { lo_b = mid; } else { hi_b = mid; }
                }
            }
            if (need_left) { res_lo = hi_a; }
            if (need_right) { res_hi = lo_b; }
            if (res_hi > res_lo + 1e-5) {
                out = Interval1D(true, res_lo, res_hi);
            }
        }
    }
    return out;
}

fn cardinal_bbox_hits_ray(ch: GpuSweepChord, prof: GpuToolProfile, verify: bool, pu: f32, pv: f32) -> bool {
    let flags = ch.info.z;
    let p0 = ch.p0_rot.xyz;
    let p1 = ch.p1_rot.xyz;
    let r_cut = prof.cutting_radius_mm;
    let u_ax = plane_u_axis(u.axis_id);
    let v_ax = plane_v_axis(u.axis_id);
    let cardinal = (flags >> CHORD_CARDINAL_SHIFT) & CHORD_CARDINAL_MASK;
    let s_ax = cardinal_axis(cardinal);
    let s_sign = cardinal_sign(cardinal);
    let max_h = horizontal_cut_max_height(prof);
    if (u.axis_id == s_ax) {
        let r_chk = select(r_cut, max(r_cut, prof.holder_radius_mm), verify) + 1e-3;
        let u0 = min(axis_get(p0, u_ax), axis_get(p1, u_ax)) - r_chk;
        let u1 = max(axis_get(p0, u_ax), axis_get(p1, u_ax)) + r_chk;
        let v0 = min(axis_get(p0, v_ax), axis_get(p1, v_ax)) - r_chk;
        let v1 = max(axis_get(p0, v_ax), axis_get(p1, v_ax)) + r_chk;
        return pu >= u0 && pu <= u1 && pv >= v0 && pv <= v1;
    }
    let spindle_in_u = (u_ax == s_ax);
    let other_ax = select(u_ax, v_ax, spindle_in_u);
    let lat_min = min(axis_get(p0, other_ax), axis_get(p1, other_ax)) - r_cut - 1e-3;
    let lat_max = max(axis_get(p0, other_ax), axis_get(p1, other_ax)) + r_cut + 1e-3;
    let s0 = min(axis_get(p0, s_ax), axis_get(p1, s_ax));
    let s1 = max(axis_get(p0, s_ax), axis_get(p1, s_ax));
    let s_min = select(s0 - max_h - 1e-3, s0 - 1e-3, s_sign >= 0.0);
    let s_max = select(s1 + 1e-3, s1 + max_h + 1e-3, s_sign >= 0.0);
    if (spindle_in_u) {
        return pu >= s_min && pu <= s_max && pv >= lat_min && pv <= lat_max;
    }
    return pu >= lat_min && pu <= lat_max && pv >= s_min && pv <= s_max;
}

fn oriented_bbox_hits_ray(ch: GpuSweepChord, prof: GpuToolProfile, verify: bool, pu: f32, pv: f32) -> bool {
    let flags = ch.info.z;
    let p0 = ch.p0_rot.xyz;
    let spindle = ch.arc_params.xyz;
    let r_cut = prof.cutting_radius_mm;
    let u_ax = plane_u_axis(u.axis_id);
    let v_ax = plane_v_axis(u.axis_id);
    let check_holder = verify && ((flags & CHORD_FLAG_CHECK_HOLDER) != 0u);
    let r_chk = select(
        select(r_cut, max(r_cut, prof.shank_radius_mm), verify),
        max(r_cut, prof.holder_radius_mm),
        check_holder,
    ) + 1e-3;
    let reach = select(
        select(horizontal_cut_max_height(prof), max(prof.stickout_length_mm, prof.flute_length_mm), verify),
        max(prof.stickout_length_mm, prof.flute_length_mm) + 40.0,
        check_holder,
    );
    let top_pt = p0 + spindle * reach;
    let bmin = min(p0, top_pt) - vec3<f32>(r_chk);
    let bmax = max(p0, top_pt) + vec3<f32>(r_chk);
    return pu >= axis_get(bmin, u_ax) && pu <= axis_get(bmax, u_ax)
        && pv >= axis_get(bmin, v_ax) && pv <= axis_get(bmax, v_ax);
}

// === Entry Point 1: Cardinal 3-Axis & Indexed 4th-Axis Linear Chords ===
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
    let init_cnt = counts[global_ray_idx];
    if (init_cnt == 0u) {
        return;
    }

    let pu = (f32(iu) + 0.5) * u.du_mm;
    let pv = (f32(iv) + 0.5) * u.dv_mm;
    let list_base = u.num_tiles + 1u;

    if (init_cnt > INLINE_K) {
        for (var k: u32 = start; k < end; k = k + 1u) {
            let ch = chords[tile_data[list_base + k]];
            let prof = profiles[ch.info.y];
            let verify = (ch.info.z & CHORD_FLAG_VERIFY) != 0u;
            if (cardinal_bbox_hits_ray(ch, prof, verify, pu, pv)) {
                flag_overflow_ray(local_ray_idx);
                break;
            }
        }
        return;
    }

    let slot_base = global_ray_idx * INLINE_K;
    var ls = load_local_spans(slot_base, init_cnt);
    var ray_mutated = false;
    var overflowed = false;

    for (var k: u32 = start; k < end; k = k + 1u) {
        if (ls.cnt == 0u || overflowed) {
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

        let cardinal = (flags >> CHORD_CARDINAL_SHIFT) & CHORD_CARDINAL_MASK;
        let s_ax = cardinal_axis(cardinal);
        let s_sign = cardinal_sign(cardinal);
        let u_ax = plane_u_axis(u.axis_id);
        let v_ax = plane_v_axis(u.axis_id);

        // Single deferred cut & target-gouge intervals applied after axial/horizontal branch.
        var cut_iv = Interval1D(false, 0.0, 0.0);
        var gouge_iv = Interval1D(false, 0.0, 0.0);
        var gouge_skin: f32 = gouge_tol;
        var rapid_loc_mid = false;

        if (u.axis_id == s_ax) {
            // === AXIAL PLANE (parallel to cardinal spindle `s_sign * e_{s_ax}`) ===
            let mp0 = vec3<f32>(axis_get(p0, u_ax), axis_get(p0, v_ax), s_sign * axis_get(p0, s_ax));
            let mp1 = vec3<f32>(axis_get(p1, u_ax), axis_get(p1, v_ax), s_sign * axis_get(p1, s_ax));
            let px = pu;
            let py = pv;
            let r_chk = select(r_cut, max(r_cut, prof.holder_radius_mm), verify) + 1e-3;
            if (px < min(mp0.x, mp1.x) - r_chk || px > max(mp0.x, mp1.x) + r_chk
                || py < min(mp0.y, mp1.y) - r_chk || py > max(mp0.y, mp1.y) + r_chk) {
                continue;
            }

            if (verify) {
                let p0_xy = mp0.xy;
                let p1_xy = mp1.xy;
                let d_xy = p1_xy - p0_xy;
                let l2 = dot(d_xy, d_xy);
                let q_xy = vec2<f32>(px, py);
                var t_c: f32 = 0.0;
                if (l2 > 1e-10) {
                    t_c = clamp(dot(q_xy - p0_xy, d_xy) / l2, 0.0, 1.0);
                }
                let c_xy = mix(p0_xy, p1_xy, t_c);
                let dist_xy = length(q_xy - c_xy);
                let z_tip_here = select(min(mp0.z, mp1.z), mp0.z + t_c * (mp1.z - mp0.z), l2 > 1e-10);
                let top_span = get_local_span(ls, ls.cnt - 1u);
                let top_z = select(-ls.s0.min_local, top_span.max_local, s_sign >= 0.0);

                if (dist_xy <= prof.holder_radius_mm) {
                    let holder_z = z_tip_here + prof.stickout_length_mm;
                    let pen = top_z - holder_z;
                    if (pen > 0.05) {
                        record_violation(SLOT_HOLDER, pen, chord_index, local_ray_idx, s_sign * holder_z);
                    }
                }
                if (!is_rapid && dist_xy <= shank_skin) {
                    let reach_top = z_tip_here + prof.effective_reach_mm;
                    let rub = top_z - reach_top;
                    if (rub > 0.15) {
                        record_violation(SLOT_FLUTE, rub, chord_index, local_ray_idx, s_sign * reach_top);
                    }
                }
                if (has_relieved_neck && !is_rapid && dist_xy <= neck_skin) {
                    let neck_lo = z_tip_here + prof.flute_length_mm + 0.05;
                    let neck_hi = z_tip_here + prof.neck_length_mm;
                    if (neck_hi > neck_lo) {
                        let w_nk = map_cz_interval_to_w(Interval1D(true, neck_lo, neck_hi), s_sign);
                        let ov = query_overlap_local(ls, w_nk.lo, w_nk.hi);
                        if (ov > 0.10) {
                            record_violation(SLOT_NECK, ov, chord_index, local_ray_idx, s_sign * neck_lo);
                        }
                    }
                }
            }

            if (is_thread_mill) {
                let p0_xy = mp0.xy;
                let p1_xy = mp1.xy;
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
                    let dz_span = mp1.z - mp0.z;
                    let z_start = prof.param1;
                    let tooth_step = prof.param2;
                    let tan_half = prof.param3;
                    let n_teeth = max(prof.teeth_count, 1u);

                    if (verify && u.has_target != 0u) {
                        gouge_iv = map_cz_interval_to_w(
                            eval_z_ray_cylinder_interval(
                                mp0, mp1, px, py, neck_skin, prof.neck_length_mm,
                            ),
                            s_sign,
                        );
                    }

                    let is_flat_tm = len2 <= 1e-10 || abs(dz_span) <= 1e-6;
                    var tm_r_flat: f32 = 0.0;
                    var do_teeth = true;
                    if (is_flat_tm) {
                        tm_r_flat = length(q_xy - mix(p0_xy, p1_xy, t_closest));
                        if (tm_r_flat <= prof.neck_radius_mm + 1e-6) {
                            cut_iv = map_cz_interval_to_w(
                                Interval1D(true, min(mp0.z, mp1.z), max(mp0.z, mp1.z) + prof.neck_length_mm),
                                s_sign,
                            );
                            do_teeth = false;
                        } else if (tm_r_flat > r_cut + 1e-6) {
                            do_teeth = false;
                        }
                    } else {
                        var neck_lo: f32 = 1e30;
                        var neck_hi: f32 = -1e30;
                        for (var si: u32 = 0u; si < 6u; si = si + 1u) {
                            let ts = eval_tm_sloped_sample_t(si, t_enter, t_exit, t_closest);
                            let r = length(q_xy - mix(p0_xy, p1_xy, ts));
                            if (r <= prof.neck_radius_mm + 1e-6) {
                                let z_tip = mp0.z + ts * dz_span;
                                neck_lo = min(neck_lo, z_tip);
                                neck_hi = max(neck_hi, z_tip + prof.neck_length_mm);
                            }
                        }
                        if (neck_hi > neck_lo + 1e-6) {
                            cut_iv = map_cz_interval_to_w(Interval1D(true, neck_lo, neck_hi), s_sign);
                        }
                    }

                    if (do_teeth) {
                        let z_lo_tip = min(mp0.z, mp1.z);
                        let z_hi_tip = max(mp0.z, mp1.z);
                        let flat_half_w = max(r_cut - tm_r_flat, 0.0) * tan_half;
                        if (verify && is_rapid) {
                            for (var tk: u32 = 0u; tk < n_teeth; tk = tk + 1u) {
                                let zk_lo = z_start + f32(tk) * tooth_step;
                                let zk_hi = zk_lo + tooth_step;
                                var t_iv = Interval1D(false, 0.0, 0.0);
                                if (is_flat_tm) {
                                    let z_crest = zk_lo + 0.5 * tooth_step;
                                    let lo = max(z_crest - flat_half_w, zk_lo) + z_lo_tip;
                                    let hi = min(z_crest + flat_half_w, zk_hi) + z_hi_tip;
                                    if (hi > lo + 1e-6) {
                                        t_iv = Interval1D(true, lo, hi);
                                    }
                                } else {
                                    t_iv = eval_tm_sloped_tooth_interval(
                                        mp0, mp1, q_xy, t_enter, t_exit, t_closest,
                                        r_cut, tan_half, zk_lo, zk_hi,
                                    );
                                }
                                if (t_iv.valid) {
                                    let w_iv = map_cz_interval_to_w(t_iv, s_sign);
                                    let ov = query_overlap_local(ls, w_iv.lo, w_iv.hi);
                                    if (ov > rapid_tol) {
                                        record_violation(SLOT_RAPID, ov, chord_index, local_ray_idx, w_iv.lo);
                                    }
                                }
                            }
                            if (cut_iv.valid) {
                                let ov = query_overlap_local(ls, cut_iv.lo, cut_iv.hi);
                                if (ov > rapid_tol) {
                                    record_violation(SLOT_RAPID, ov, chord_index, local_ray_idx, cut_iv.lo);
                                }
                            }
                        }
                        for (var tk: u32 = 0u; tk < n_teeth; tk = tk + 1u) {
                            let zk_lo = z_start + f32(tk) * tooth_step;
                            let zk_hi = zk_lo + tooth_step;
                            var t_iv = Interval1D(false, 0.0, 0.0);
                            if (is_flat_tm) {
                                let z_crest = zk_lo + 0.5 * tooth_step;
                                let lo = max(z_crest - flat_half_w, zk_lo) + z_lo_tip;
                                let hi = min(z_crest + flat_half_w, zk_hi) + z_hi_tip;
                                if (hi > lo + 1e-6) {
                                    t_iv = Interval1D(true, lo, hi);
                                }
                            } else {
                                t_iv = eval_tm_sloped_tooth_interval(
                                    mp0, mp1, q_xy, t_enter, t_exit, t_closest,
                                    r_cut, tan_half, zk_lo, zk_hi,
                                );
                            }
                            if (t_iv.valid) {
                                let w_iv = map_cz_interval_to_w(t_iv, s_sign);
                                let res = subtract_interval_local(&ls, w_iv.lo, w_iv.hi, op_idx);
                                if (res < 0) { overflowed = true; break; } else if (res > 0) { ray_mutated = true; }
                            }
                        }
                        // Sloped neck rapid was already checked above; apply remaining neck cut below.
                        if (verify && is_rapid) {
                            if (!overflowed && cut_iv.valid) {
                                let res = subtract_interval_local(&ls, cut_iv.lo, cut_iv.hi, op_idx);
                                if (res < 0) { overflowed = true; } else if (res > 0) { ray_mutated = true; }
                            }
                            cut_iv = Interval1D(false, 0.0, 0.0);
                        }
                    }
                }
            } else {
                // Single call site to `eval_z_ray_cut_interval` for both cut (eval_pass 0) & gouge (eval_pass 1).
                for (var eval_pass: u32 = 0u; eval_pass < 2u; eval_pass = eval_pass + 1u) {
                    let r_lim = select(r_inner_skin, r_cut, eval_pass == 0u);
                    let z_iv = map_cz_interval_to_w(
                        eval_z_ray_cut_interval(mp0, mp1, prof, px, py, r_lim),
                        s_sign,
                    );
                    if (eval_pass == 0u) {
                        cut_iv = z_iv;
                        if (!cut_iv.valid || !verify || u.has_target == 0u) {
                            break;
                        }
                    } else {
                        gouge_iv = z_iv;
                    }
                }
            }
        } else {
            // === HORIZONTAL PLANES (orthogonal to cardinal spindle `s_sign * e_{s_ax}`) ===
            let spindle_in_u = (u_ax == s_ax);
            let other_ax = select(u_ax, v_ax, spindle_in_u);
            let mp0 = vec3<f32>(
                axis_get(p0, u.axis_id),
                axis_get(p0, other_ax),
                s_sign * axis_get(p0, s_ax),
            );
            let mp1 = vec3<f32>(
                axis_get(p1, u.axis_id),
                axis_get(p1, other_ax),
                s_sign * axis_get(p1, s_ax),
            );
            let pz = select(s_sign * pv, s_sign * pu, spindle_in_u);
            let pu_c = select(pu, pv, spindle_in_u);
            let max_h = horizontal_cut_max_height(prof);
            if (pu_c < min(mp0.y, mp1.y) - r_cut - 1e-3 || pu_c > max(mp0.y, mp1.y) + r_cut + 1e-3
                || pz < min(mp0.z, mp1.z) - 1e-3 || pz > max(mp0.z, mp1.z) + max_h + 1e-3) {
                continue;
            }

            rapid_loc_mid = true;
            gouge_skin = gouge_tol + wall_skin_tol;
            let flute_wall_skin = wall_skin_tol + max(gouge_tol - 0.02, 0.0);
            let do_h_gouge = verify && u.has_target != 0u && pz >= min(mp0.z, mp1.z) + gouge_tol;
            // Single call site to `eval_horizontal_ray_cut_interval` for cut (eval_pass 0) & gouge (eval_pass 1).
            for (var eval_pass: u32 = 0u; eval_pass < 2u; eval_pass = eval_pass + 1u) {
                let shrink = select(flute_wall_skin, 0.0, eval_pass == 0u);
                let w_iv = eval_horizontal_ray_cut_interval(mp0, mp1, prof, shrink, pz, pu_c);
                if (eval_pass == 0u) {
                    cut_iv = w_iv;
                    if (!cut_iv.valid || !do_h_gouge) {
                        break;
                    }
                    if (is_thread_mill) {
                        gouge_iv = eval_horizontal_ray_cylinder_interval(
                            mp0, mp1, neck_skin, prof.neck_length_mm, pz, pu_c,
                        );
                        break;
                    }
                } else {
                    gouge_iv = w_iv;
                }
            }
        }

        if (gouge_iv.valid) {
            let gh = query_target_gouge(global_ray_idx, gouge_iv.lo, gouge_iv.hi, gouge_skin);
            if (gh.valid) {
                record_violation(SLOT_GOUGE, gh.depth, chord_index, local_ray_idx, gh.coord);
            }
        }

        if (!overflowed && cut_iv.valid) {
            if (verify && is_rapid) {
                let ov = query_overlap_local(ls, cut_iv.lo, cut_iv.hi);
                if (ov > rapid_tol) {
                    let loc = select(cut_iv.lo, 0.5 * (cut_iv.lo + cut_iv.hi), rapid_loc_mid);
                    record_violation(SLOT_RAPID, ov, chord_index, local_ray_idx, loc);
                }
            }
            let res = subtract_interval_local(&ls, cut_iv.lo, cut_iv.hi, op_idx);
            if (res < 0) { overflowed = true; } else if (res > 0) { ray_mutated = true; }
        }
    }

    if (overflowed) {
        flag_overflow_ray(local_ray_idx);
        return;
    }

    if (ray_mutated) {
        store_local_spans(slot_base, ls);
        counts[global_ray_idx] = ls.cnt;
        atomicStore(&accum_and_overflow[WORD_MUTATED], 1u);
    }
}

// === Entry Point 2: General Oriented 4th-Axis Rotary Poses (Lazily Compiled) ===
@compute @workgroup_size(16, 16, 1)
fn main_oriented(
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
    let init_cnt = counts[global_ray_idx];
    if (init_cnt == 0u) {
        return;
    }

    let pu = (f32(iu) + 0.5) * u.du_mm;
    let pv = (f32(iv) + 0.5) * u.dv_mm;
    let list_base = u.num_tiles + 1u;

    if (init_cnt > INLINE_K) {
        for (var k: u32 = start; k < end; k = k + 1u) {
            let ch = chords[tile_data[list_base + k]];
            let prof = profiles[ch.info.y];
            let verify = (ch.info.z & CHORD_FLAG_VERIFY) != 0u;
            if (oriented_bbox_hits_ray(ch, prof, verify, pu, pv)) {
                flag_overflow_ray(local_ray_idx);
                break;
            }
        }
        return;
    }

    let slot_base = global_ray_idx * INLINE_K;
    var ls = load_local_spans(slot_base, init_cnt);
    var ray_mutated = false;
    var overflowed = false;
    let p_base = plane_from_uvw(u.axis_id, pu, pv, 0.0);
    let d_axis = axis_unit_vec(u.axis_id);

    for (var k: u32 = start; k < end; k = k + 1u) {
        if (ls.cnt == 0u || overflowed) {
            break;
        }
        let ch = chords[tile_data[list_base + k]];
        let prof = profiles[ch.info.y];
        let op_idx = ch.info.x;
        let flags = ch.info.z;
        let chord_index = ch.info.w;
        let verify = (flags & CHORD_FLAG_VERIFY) != 0u;
        let is_rapid = (flags & CHORD_FLAG_RAPID) != 0u;
        let has_relieved_neck = (prof.flags & PROF_FLAG_RELIEVED_NECK) != 0u;
        let rapid_tol = ch.sweep_and_tols.y;
        let gouge_tol = ch.sweep_and_tols.z;
        let wall_skin_tol = ch.sweep_and_tols.w;
        let p0 = ch.p0_rot.xyz;
        let spindle = ch.arc_params.xyz;

        let r_cut = prof.cutting_radius_mm;
        let neck_skin = max(prof.neck_radius_mm - wall_skin_tol, 0.0);
        let shank_skin = max(prof.shank_radius_mm - wall_skin_tol, 0.0);
        let r_inner_skin = max(r_cut - wall_skin_tol, 0.0);

        let v0 = p_base - p0;
        let h0 = dot(v0, spindle);
        let d_dot_s = dot(d_axis, spindle);
        let v0_perp = v0 - spindle * h0;
        let d_perp = d_axis - spindle * d_dot_s;
        let a = max(dot(d_perp, d_perp), 0.0);
        let b = dot(v0_perp, d_perp);
        let c = max(dot(v0_perp, v0_perp), 0.0);

        if (verify) {
            let stickout = max(prof.stickout_length_mm, prof.flute_length_mm);
            if ((flags & CHORD_FLAG_CHECK_HOLDER) != 0u) {
                let h_iv = solve_oriented_cylinder_interval(
                    a, b, c, h0, d_dot_s, prof.holder_radius_mm, stickout, stickout + 40.0,
                );
                if (h_iv.valid) {
                    let pen = query_overlap_local(ls, h_iv.lo, h_iv.hi);
                    if (pen > 0.05) {
                        record_violation(SLOT_HOLDER, pen, chord_index, local_ray_idx, h_iv.lo);
                    }
                }
            }
            if (!is_rapid) {
                let reach = prof.effective_reach_mm;
                if (stickout > reach + 1e-4) {
                    let f_iv = solve_oriented_cylinder_interval(
                        a, b, c, h0, d_dot_s, shank_skin, reach, stickout,
                    );
                    if (f_iv.valid) {
                        let rub = query_overlap_local(ls, f_iv.lo, f_iv.hi);
                        if (rub > 0.15) {
                            record_violation(SLOT_FLUTE, rub, chord_index, local_ray_idx, f_iv.lo);
                        }
                    }
                }
                if (has_relieved_neck && prof.neck_length_mm > prof.flute_length_mm + 0.05) {
                    let n_iv = solve_oriented_cylinder_interval(
                        a, b, c, h0, d_dot_s, neck_skin, prof.flute_length_mm + 0.05, prof.neck_length_mm,
                    );
                    if (n_iv.valid) {
                        let ov = query_overlap_local(ls, n_iv.lo, n_iv.hi);
                        if (ov > 0.10) {
                            record_violation(SLOT_NECK, ov, chord_index, local_ray_idx, n_iv.lo);
                        }
                    }
                }
            }
        }

        // Single call site to `eval_oriented_pose_cut_interval` for cut (eval_pass 0) & gouge (eval_pass 1).
        var cut_iv = Interval1D(false, 0.0, 0.0);
        for (var eval_pass: u32 = 0u; eval_pass < 2u; eval_pass = eval_pass + 1u) {
            let r_lim = select(r_inner_skin, r_cut, eval_pass == 0u);
            let w_iv = eval_oriented_pose_cut_interval(
                v0, d_axis, spindle, a, b, c, h0, d_dot_s, prof, r_lim,
            );
            if (eval_pass == 0u) {
                cut_iv = w_iv;
                if (!cut_iv.valid || !verify || u.has_target == 0u) {
                    break;
                }
            } else if (w_iv.valid) {
                let gh = query_target_gouge(
                    global_ray_idx, w_iv.lo, w_iv.hi, gouge_tol + wall_skin_tol,
                );
                if (gh.valid) {
                    record_violation(SLOT_GOUGE, gh.depth, chord_index, local_ray_idx, gh.coord);
                }
            }
        }

        if (cut_iv.valid) {
            if (verify && is_rapid) {
                let ov = query_overlap_local(ls, cut_iv.lo, cut_iv.hi);
                if (ov > rapid_tol) {
                    record_violation(
                        SLOT_RAPID,
                        ov,
                        chord_index,
                        local_ray_idx,
                        0.5 * (cut_iv.lo + cut_iv.hi),
                    );
                }
            }
            let res = subtract_interval_local(&ls, cut_iv.lo, cut_iv.hi, op_idx);
            if (res < 0) { overflowed = true; } else if (res > 0) { ray_mutated = true; }
        }
    }

    if (overflowed) {
        flag_overflow_ray(local_ray_idx);
        return;
    }

    if (ray_mutated) {
        store_local_spans(slot_base, ls);
        counts[global_ray_idx] = ls.cnt;
        atomicStore(&accum_and_overflow[WORD_MUTATED], 1u);
    }
}
