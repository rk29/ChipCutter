// Tiled 16x16 2.5D Z-Dexel Heightfield WGSL Compute Sweeper
// Mirrors `crates/chip-stock/src/heightfield/sweep.rs` across all 7 cutter geometries,
// sloped linear moves, closed-form planar XY arcs, through-drilling, and atomic Rules 1-4 + neck.

const SHAPE_FLAT: u32 = 0u;
const SHAPE_BULLNOSE: u32 = 1u;
const SHAPE_BALL: u32 = 2u;
const SHAPE_CONICAL: u32 = 3u;
const SHAPE_FACE: u32 = 4u;
const SHAPE_THREADMILL: u32 = 5u;

const PROF_FLAG_FLAT_ENDMILL: u32 = 1u;
const PROF_FLAG_THREAD_MILL: u32 = 2u;
const PROF_FLAG_CONICAL: u32 = 4u;

const CHORD_FLAG_RAPID: u32 = 1u;
const CHORD_FLAG_VERIFY: u32 = 2u;
const CHORD_FLAG_PLANAR_ARC_XY: u32 = 4u;
const CHORD_FLAG_CHECK_HOLDER: u32 = 8u;

const SLOT_RAPID: u32 = 0u;
const SLOT_GOUGE: u32 = 8u;
const SLOT_FLUTE: u32 = 16u;
const SLOT_NECK: u32 = 24u;
const SLOT_HOLDER: u32 = 32u;
const WORD_MUTATED: u32 = 40u;

const TAU: f32 = 6.283185307179586;

struct HeightfieldUniforms {
    nx: u32,
    ny: u32,
    tiles_x: u32,
    num_tiles: u32,
    dx_mm: f32,
    dy_mm: f32,
    _pad0: f32,
    _pad1: f32,
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

struct GpuViolationAccum {
    words: array<atomic<u32>, 44>,
}

struct CutterHit {
    valid: bool,
    z_cut: f32,
    z_tip: f32,
    r_mm: f32,
}

struct TipRadiusHit {
    valid: bool,
    r_axis: f32,
    z_tip_min: f32,
}

@group(0) @binding(0) var<uniform> u: HeightfieldUniforms;
@group(0) @binding(1) var<storage, read_write> z_stock: array<f32>;
@group(0) @binding(2) var<storage, read> z_bottom: array<f32>;
@group(0) @binding(3) var<storage, read> z_target: array<f32>;
@group(0) @binding(4) var<storage, read_write> last_op_idx: array<u32>;
@group(0) @binding(5) var<storage, read> profiles: array<GpuToolProfile>;
@group(0) @binding(6) var<storage, read> chords: array<GpuSweepChord>;
@group(0) @binding(7) var<storage, read> tile_data: array<u32>;
@group(0) @binding(8) var<storage, read_write> violations: GpuViolationAccum;

fn is_finite_f32(v: f32) -> bool {
    let bits = bitcast<u32>(v);
    return (bits & 0x7f800000u) != 0x7f800000u;
}

fn neg_inf_f32() -> f32 {
    return bitcast<f32>(0xff800000u);
}

fn rem_euclid_tau(a: f32) -> f32 {
    return a - TAU * floor(a / TAU);
}

fn record_violation(slot_base: u32, depth: f32, chord_idx: u32, cell_idx: u32, loc_z: f32) {
    if (depth <= 0.0) {
        return;
    }
    let depth_bits = bitcast<u32>(depth);
    atomicStore(&violations.words[slot_base], 1u);
    let prev = atomicMax(&violations.words[slot_base + 1u], depth_bits);
    if (depth_bits > prev) {
        atomicStore(&violations.words[slot_base + 2u], chord_idx);
        atomicStore(&violations.words[slot_base + 3u], cell_idx);
        atomicStore(&violations.words[slot_base + 4u], bitcast<u32>(loc_z));
    }
}

// Evaluates axial offset `dz >= 0` from cutter tip to lower cutting envelope at radius `r_mm`.
// Returns `< 0.0` when `r_mm` lies outside the cutting envelope.
fn eval_profile_dz(prof: GpuToolProfile, r_mm: f32) -> f32 {
    if (r_mm < 0.0 || r_mm > prof.cutting_radius_mm + 1e-6) {
        return -1.0;
    }
    let r = min(r_mm, prof.cutting_radius_mm);
    switch (prof.shape_kind) {
        case SHAPE_FLAT: {
            return 0.0;
        }
        case SHAPE_BULLNOSE: {
            let flat_r = prof.param0;
            let corner_r = prof.param1;
            if (r <= flat_r || corner_r <= 1e-6) {
                return 0.0;
            }
            let dr = min(r - flat_r, corner_r);
            return corner_r - sqrt(max((corner_r - dr) * (corner_r + dr), 0.0));
        }
        case SHAPE_BALL: {
            let rad = prof.cutting_radius_mm;
            let rc = min(r, rad);
            return rad - sqrt(max((rad - rc) * (rad + rc), 0.0));
        }
        case SHAPE_CONICAL: {
            let tip_r = prof.param0;
            let tan_half = prof.param1;
            if (r <= tip_r) {
                return 0.0;
            }
            return (r - tip_r) / tan_half;
        }
        case SHAPE_FACE: {
            let flat_r = prof.param0;
            let corner_r = prof.param1;
            let bottom_r = prof.param2;
            if (r <= flat_r) {
                return 0.0;
            } else if (r <= bottom_r) {
                if (corner_r <= 1e-6) {
                    return 0.0;
                }
                let dr = min(r - flat_r, corner_r);
                return corner_r - sqrt(max((corner_r - dr) * (corner_r + dr), 0.0));
            } else {
                return corner_r + (r - bottom_r);
            }
        }
        case SHAPE_THREADMILL: {
            if (r <= prof.neck_radius_mm) {
                return 0.0;
            }
            let z_start = prof.param1;
            let tooth_step = prof.param2;
            let tan_half = prof.param3;
            let z_first_crest = z_start + 0.5 * tooth_step;
            return max(z_first_crest - max(prof.cutting_radius_mm - r, 0.0) * tan_half, 0.0);
        }
        default: {
            return 0.0;
        }
    }
}

fn planar_arc_xy_distance(px: f32, py: f32, ch: GpuSweepChord) -> f32 {
    let cx = ch.arc_params.x;
    let cy = ch.arc_params.y;
    let radius = ch.arc_params.z;
    let a0 = ch.arc_params.w;
    let signed_sweep = ch.sweep_and_tols.x;
    let sweep_abs = abs(signed_sweep);

    let dx = px - cx;
    let dy = py - cy;
    let dist_c = length(vec2<f32>(dx, dy));

    if (sweep_abs >= TAU - 1e-5) {
        return abs(dist_c - radius);
    }
    if (dist_c > 1e-6) {
        let ang = atan2(dy, dx);
        var rel: f32;
        if (signed_sweep >= 0.0) {
            rel = rem_euclid_tau(ang - a0);
        } else {
            rel = rem_euclid_tau(a0 - ang);
        }
        if (rel <= sweep_abs) {
            return abs(dist_c - radius);
        }
    }
    let d0 = length(vec2<f32>(px - ch.p0_rot.x, py - ch.p0_rot.y));
    let d1 = length(vec2<f32>(px - ch.p1_rot.x, py - ch.p1_rot.y));
    return min(d0, d1);
}

fn min_tip_z_within_radius(ch: GpuSweepChord, px: f32, py: f32, r_max: f32) -> TipRadiusHit {
    var miss = TipRadiusHit(false, 0.0, 0.0);
    if (r_max <= 0.0) {
        return miss;
    }
    let is_arc = (ch.info.z & CHORD_FLAG_PLANAR_ARC_XY) != 0u;
    if (is_arc) {
        let r = planar_arc_xy_distance(px, py, ch);
        if (r <= r_max) {
            return TipRadiusHit(true, r, ch.p0_rot.z);
        }
        return miss;
    }

    let dx = ch.p1_rot.x - ch.p0_rot.x;
    let dy = ch.p1_rot.y - ch.p0_rot.y;
    let dz = ch.p1_rot.z - ch.p0_rot.z;
    let l2 = dx * dx + dy * dy;
    if (l2 <= 1e-10) {
        let r = length(vec2<f32>(px - ch.p0_rot.x, py - ch.p0_rot.y));
        if (r <= r_max) {
            return TipRadiusHit(true, r, min(ch.p0_rot.z, ch.p1_rot.z));
        }
        return miss;
    }

    let tc = ((px - ch.p0_rot.x) * dx + (py - ch.p0_rot.y) * dy) / l2;
    let qx = ch.p0_rot.x + tc * dx;
    let qy = ch.p0_rot.y + tc * dy;
    let rx = px - qx;
    let ry = py - qy;
    let d_perp_sq = rx * rx + ry * ry;
    let r2 = r_max * r_max;
    if (d_perp_sq > r2) {
        return miss;
    }
    let dt = sqrt(max((r2 - d_perp_sq) / l2, 0.0));
    let t_lo = max(tc - dt, 0.0);
    let t_hi = min(tc + dt, 1.0);
    if (t_lo > t_hi) {
        return miss;
    }
    let t_best = select(t_hi, t_lo, dz >= 0.0);
    let t_clamped = clamp(tc, t_lo, t_hi);
    let r_min = length(vec2<f32>(
        px - (ch.p0_rot.x + t_clamped * dx),
        py - (ch.p0_rot.y + t_clamped * dy),
    ));
    let z_min = ch.p0_rot.z + t_best * dz;
    return TipRadiusHit(true, r_min, z_min);
}

fn eval_cutter_at_xy(ch: GpuSweepChord, px: f32, py: f32, prof: GpuToolProfile) -> CutterHit {
    var miss = CutterHit(false, 0.0, 0.0, 0.0);
    let is_arc = (ch.info.z & CHORD_FLAG_PLANAR_ARC_XY) != 0u;
    if (is_arc) {
        let r = planar_arc_xy_distance(px, py, ch);
        let p_dz = eval_profile_dz(prof, r);
        if (p_dz < 0.0) {
            return miss;
        }
        return CutterHit(true, ch.p0_rot.z + p_dz, ch.p0_rot.z, r);
    }

    let dx = ch.p1_rot.x - ch.p0_rot.x;
    let dy = ch.p1_rot.y - ch.p0_rot.y;
    let dz = ch.p1_rot.z - ch.p0_rot.z;
    let l2 = dx * dx + dy * dy;

    if (l2 <= 1e-10) {
        let r = length(vec2<f32>(px - ch.p0_rot.x, py - ch.p0_rot.y));
        let p_dz = eval_profile_dz(prof, r);
        if (p_dz < 0.0) {
            return miss;
        }
        let z_tip = min(ch.p0_rot.z, ch.p1_rot.z);
        return CutterHit(true, z_tip + p_dz, z_tip, r);
    }

    if (abs(dz) <= 1e-6) {
        let t = clamp(((px - ch.p0_rot.x) * dx + (py - ch.p0_rot.y) * dy) / l2, 0.0, 1.0);
        let qx = ch.p0_rot.x + t * dx;
        let qy = ch.p0_rot.y + t * dy;
        let r = length(vec2<f32>(px - qx, py - qy));
        let p_dz = eval_profile_dz(prof, r);
        if (p_dz < 0.0) {
            return miss;
        }
        return CutterHit(true, ch.p0_rot.z + p_dz, ch.p0_rot.z, r);
    }

    // Sloped 3D linear move: exact `[t_lo, t_hi]` intersection with cylinder of radius `r_max`
    let r_max = prof.cutting_radius_mm;
    let tc = ((px - ch.p0_rot.x) * dx + (py - ch.p0_rot.y) * dy) / l2;
    let qx_inf = ch.p0_rot.x + tc * dx;
    let qy_inf = ch.p0_rot.y + tc * dy;
    let rx = px - qx_inf;
    let ry = py - qy_inf;
    let d_perp_sq = rx * rx + ry * ry;
    let r2 = r_max * r_max;
    if (d_perp_sq > r2) {
        return miss;
    }
    let dt = sqrt(max((r2 - d_perp_sq) / l2, 0.0));
    let t_lo = max(tc - dt, 0.0);
    let t_hi = min(tc + dt, 1.0);
    if (t_lo > t_hi) {
        return miss;
    }

    let t_min_r = clamp(tc, t_lo, t_hi);
    let min_r = length(vec2<f32>(
        px - (ch.p0_rot.x + t_min_r * dx),
        py - (ch.p0_rot.y + t_min_r * dy),
    ));

    if ((prof.flags & PROF_FLAG_FLAT_ENDMILL) != 0u) {
        let t_opt = select(t_hi, t_lo, dz >= 0.0);
        let z_tip = ch.p0_rot.z + t_opt * dz;
        return CutterHit(true, z_tip, z_tip, min_r);
    }

    // Shaped cutter on sloped 3D ramp: 7 uniform samples + `t_min_r` + exact conical stationary `t*`
    // Note: every `t` in `[t_lo, t_hi]` is guaranteed inside the cylinder `r <= r_max`; clamp `r`
    // against `r_max` so `f32` rounding at the exact boundary `t_lo`/`t_hi` never misses `s = 0`/`6`.
    var best = miss;
    for (var s: u32 = 0u; s <= 6u; s = s + 1u) {
        let t = t_lo + (t_hi - t_lo) * (f32(s) / 6.0);
        let r = min(
            length(vec2<f32>(
                px - (ch.p0_rot.x + t * dx),
                py - (ch.p0_rot.y + t * dy),
            )),
            r_max,
        );
        let p_dz = eval_profile_dz(prof, r);
        if (p_dz >= 0.0) {
            let z_tip = ch.p0_rot.z + t * dz;
            let z_cut = z_tip + p_dz;
            if (!best.valid || z_cut < best.z_cut) {
                best = CutterHit(true, z_cut, z_tip, min(min_r, r));
            }
        }
    }

    {
        let r = min(min_r, r_max);
        let p_dz = eval_profile_dz(prof, r);
        if (p_dz >= 0.0) {
            let z_tip = ch.p0_rot.z + t_min_r * dz;
            let z_cut = z_tip + p_dz;
            if (!best.valid || z_cut < best.z_cut) {
                best = CutterHit(true, z_cut, z_tip, min_r);
            }
        }
    }

    if ((prof.flags & PROF_FLAG_CONICAL) != 0u) {
        let k_cot = prof.param2;
        let denom_sq = k_cot * k_cot * l2 - dz * dz;
        if (denom_sq > 1e-8) {
            let d_perp = sqrt(max(d_perp_sq, 0.0));
            let l_xy = sqrt(l2);
            let sgn_dz = select(-1.0, 1.0, dz >= 0.0);
            let t_star = clamp(tc - sgn_dz * abs(dz) * d_perp / (l_xy * sqrt(denom_sq)), t_lo, t_hi);
            let r = min(
                length(vec2<f32>(
                    px - (ch.p0_rot.x + t_star * dx),
                    py - (ch.p0_rot.y + t_star * dy),
                )),
                r_max,
            );
            let p_dz = eval_profile_dz(prof, r);
            if (p_dz >= 0.0) {
                let z_tip = ch.p0_rot.z + t_star * dz;
                let z_cut = z_tip + p_dz;
                if (!best.valid || z_cut < best.z_cut) {
                    best = CutterHit(true, z_cut, z_tip, min(min_r, r));
                }
            }
        }
    }

    return best;
}

@compute @workgroup_size(16, 16, 1)
fn main(
    @builtin(global_invocation_id) gid: vec3<u32>,
    @builtin(workgroup_id) wid: vec3<u32>,
) {
    let i = gid.x;
    let j = gid.y;
    if (i >= u.nx || j >= u.ny) {
        return;
    }

    let tile_id = wid.y * u.tiles_x + wid.x;
    let start = tile_data[tile_id];
    let end = tile_data[tile_id + 1u];
    if (start >= end) {
        return;
    }

    let idx = j * u.nx + i;
    let zb = z_bottom[idx];
    let zt = z_target[idx];
    let zb_finite = is_finite_f32(zb);
    let has_target = is_finite_f32(zt);
    if (!zb_finite && !has_target) {
        return;
    }

    var zs = z_stock[idx];
    var op_tag = last_op_idx[idx];
    var cell_mutated = false;

    let px = (f32(i) + 0.5) * u.dx_mm;
    let py = (f32(j) + 0.5) * u.dy_mm;
    let list_base = u.num_tiles + 1u;

    for (var k: u32 = start; k < end; k = k + 1u) {
        let ch_idx = tile_data[list_base + k];
        let ch = chords[ch_idx];
        let prof = profiles[ch.info.y];
        let flags = ch.info.z;
        let verify = (flags & CHORD_FLAG_VERIFY) != 0u;
        let has_stock = zb_finite && is_finite_f32(zs) && zs > zb;

        if (!verify) {
            if (!has_stock) {
                continue;
            }
            let min_tip_z = min(ch.p0_rot.z, ch.p1_rot.z);
            if (zs <= min_tip_z) {
                continue;
            }
            let hit = eval_cutter_at_xy(ch, px, py, prof);
            if (hit.valid && hit.z_cut < zs) {
                zs = select(hit.z_cut, neg_inf_f32(), hit.z_cut <= zb);
                op_tag = ch.info.x;
                cell_mutated = true;
            }
            continue;
        }

        if (!has_stock && !has_target) {
            continue;
        }

        let is_rapid = (flags & CHORD_FLAG_RAPID) != 0u;
        let check_holder = (flags & CHORD_FLAG_CHECK_HOLDER) != 0u;
        let is_thread_mill = (prof.flags & PROF_FLAG_THREAD_MILL) != 0u;
        let wall_skin = ch.sweep_and_tols.w;
        let r_inner_skin = max(prof.cutting_radius_mm - wall_skin, 0.0);
        let shank_skin = max(prof.shank_radius_mm - wall_skin, 0.0);
        let neck_skin = max(prof.neck_radius_mm - wall_skin, 0.0);
        let holder_skin = max(prof.holder_radius_mm - wall_skin, 0.0);
        let reach_mm = prof.effective_reach_mm;
        let chord_index = ch.info.w;

        // Rule 4: Collet / holder collision
        if (check_holder && has_stock) {
            let h_hit = min_tip_z_within_radius(ch, px, py, holder_skin);
            if (h_hit.valid && h_hit.r_axis <= holder_skin) {
                let holder_z = h_hit.z_tip_min + prof.stickout_length_mm;
                let pen = zs - holder_z;
                if (pen > 0.03) {
                    record_violation(SLOT_HOLDER, pen, chord_index, idx, holder_z);
                }
            }
        }

        // Evaluate cutter cutting envelope
        let hit = eval_cutter_at_xy(ch, px, py, prof);
        if (hit.valid) {
            let gouge_r_limit = select(r_inner_skin, neck_skin, is_thread_mill);
            if (has_target && hit.r_mm <= gouge_r_limit) {
                let gouge = zt - hit.z_cut;
                if (gouge > ch.sweep_and_tols.z) {
                    record_violation(SLOT_GOUGE, gouge, chord_index, idx, hit.z_cut);
                }
            }

            if (has_stock) {
                let cut_depth = zs - hit.z_cut;

                // Rule 1: Rapid-into-Stock crash
                if (is_rapid && hit.r_mm <= r_inner_skin && cut_depth > ch.sweep_and_tols.y) {
                    record_violation(SLOT_RAPID, cut_depth, chord_index, idx, hit.z_cut);
                }

                // Relieved-neck clearance check for ThreadMill
                if (is_thread_mill && !is_rapid && hit.r_mm <= neck_skin && cut_depth > 0.02) {
                    let neck_bottom_z = hit.z_tip + prof.flute_length_mm;
                    let neck_pen = zs - neck_bottom_z;
                    if (neck_pen > 0.15) {
                        record_violation(SLOT_NECK, neck_pen, chord_index, idx, neck_bottom_z);
                    }
                }

                // Rule 3: Shank rub above usable reach
                if (hit.r_mm <= max(r_inner_skin, shank_skin) && cut_depth > 0.02) {
                    let engagement = zs - hit.z_tip;
                    let excess_flute = engagement - reach_mm;
                    if (excess_flute > 0.05) {
                        record_violation(SLOT_FLUTE, excess_flute, chord_index, idx, hit.z_tip);
                    }
                }

                // Mutate remaining heightmap stock in stock-local f32
                if (hit.z_cut < zs) {
                    zs = select(hit.z_cut, neg_inf_f32(), hit.z_cut <= zb);
                    op_tag = ch.info.x;
                    cell_mutated = true;
                }
            }
        } else if (has_stock && prof.shank_radius_mm > prof.cutting_radius_mm + 1e-4) {
            let s_hit = min_tip_z_within_radius(ch, px, py, shank_skin);
            if (s_hit.valid && s_hit.r_axis <= shank_skin) {
                let shank_bottom_z = s_hit.z_tip_min + reach_mm;
                let rub = zs - shank_bottom_z;
                if (rub > 0.05) {
                    record_violation(SLOT_FLUTE, rub, chord_index, idx, shank_bottom_z);
                }
            }
        }
    }

    if (cell_mutated) {
        z_stock[idx] = zs;
        last_op_idx[idx] = op_tag;
        atomicStore(&violations.words[WORD_MUTATED], 1u);
    }
}
