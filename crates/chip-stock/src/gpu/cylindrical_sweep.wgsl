// Tiled 16x16 4th-Axis Polar Cylindrical WGSL Compute Sweeper
// Mirrors `crates/chip-stock/src/cylindrical/sweep.rs` across all 7 cutter geometries,
// exact cylinder-ray interval + 10-step bisection, off-centerline simultaneous 4-axis moves,
// and atomic Rules 1-4 verification.

const SHAPE_FLAT: u32 = 0u;
const SHAPE_BULLNOSE: u32 = 1u;
const SHAPE_BALL: u32 = 2u;
const SHAPE_CONICAL: u32 = 3u;
const SHAPE_FACE: u32 = 4u;
const SHAPE_THREADMILL: u32 = 5u;

const POSE_FLAG_RAPID: u32 = 1u;
const POSE_FLAG_VERIFY: u32 = 2u;
const POSE_FLAG_CHECK_HOLDER: u32 = 4u;

const SLOT_RAPID: u32 = 0u;
const SLOT_GOUGE: u32 = 8u;
const SLOT_FLUTE: u32 = 16u;
const SLOT_HOLDER: u32 = 32u;
const WORD_MUTATED: u32 = 40u;

struct CylindricalUniforms {
    e1_and_u_min: vec4<f32>,
    e2_and_du: vec4<f32>,
    u_vec_and_dtheta: vec4<f32>,
    dims: vec4<u32>,
    params: vec4<f32>,
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

struct GpuRotaryPose {
    tip_and_rapid_tol: vec4<f32>,
    spindle_and_gouge_tol: vec4<f32>,
    info: vec4<u32>,
}

struct GpuViolationAccum {
    words: array<atomic<u32>, 44>,
}

struct RayInterval {
    valid: bool,
    lo: f32,
    hi: f32,
}

@group(0) @binding(0) var<uniform> u: CylindricalUniforms;
@group(0) @binding(1) var<storage, read_write> r_stock: array<f32>;
@group(0) @binding(2) var<storage, read> r_target: array<f32>;
@group(0) @binding(3) var<storage, read_write> last_op_idx: array<u32>;
@group(0) @binding(4) var<storage, read> profiles: array<GpuToolProfile>;
@group(0) @binding(5) var<storage, read> poses: array<GpuRotaryPose>;
@group(0) @binding(6) var<storage, read> tile_data: array<u32>;
@group(0) @binding(7) var<storage, read_write> violations: GpuViolationAccum;

fn record_violation(slot_base: u32, depth: f32, pose_idx: u32) {
    if (depth <= 0.0) {
        return;
    }
    let depth_bits = bitcast<u32>(depth);
    atomicStore(&violations.words[slot_base], 1u);
    let prev = atomicMax(&violations.words[slot_base + 1u], depth_bits);
    if (depth_bits > prev) {
        atomicStore(&violations.words[slot_base + 2u], pose_idx);
        atomicStore(&violations.words[slot_base + 3u], pose_idx);
        atomicStore(&violations.words[slot_base + 4u], 0u);
    }
}

fn eval_profile_dz(prof: GpuToolProfile, r_mm: f32) -> f32 {
    if (r_mm < 0.0 || r_mm > prof.cutting_radius_mm + 1e-5) {
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
            return corner_r - sqrt(max(corner_r * corner_r - dr * dr, 0.0));
        }
        case SHAPE_BALL: {
            let rad = prof.cutting_radius_mm;
            let rc = min(r, rad);
            return rad - sqrt(max(rad * rad - rc * rc, 0.0));
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
                return corner_r - sqrt(max(corner_r * corner_r - dr * dr, 0.0));
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

fn solve_cylinder_ray_interval(
    a: f32,
    b: f32,
    c: f32,
    radius: f32,
) -> RayInterval {
    let c_eff = c - radius * radius;
    if (a <= 1e-10) {
        if (c_eff <= 0.0) {
            return RayInterval(true, 0.0, 1e6);
        }
        return RayInterval(false, 0.0, 0.0);
    }
    let disc = b * b - a * c_eff;
    if (disc < 0.0) {
        return RayInterval(false, 0.0, 0.0);
    }
    let sd = sqrt(disc);
    return RayInterval(true, (-b - sd) / a, (-b + sd) / a);
}

fn eval_clearance(
    r: f32,
    v0: vec3<f32>,
    d_rad: vec3<f32>,
    spindle: vec3<f32>,
    prof: GpuToolProfile,
    r_cut: f32,
) -> f32 {
    let p_rel = v0 + d_rad * r;
    let h = dot(p_rel, spindle);
    let perp = p_rel - spindle * h;
    let rho = min(length(perp), r_cut);
    let env_dz = eval_profile_dz(prof, rho);
    if (env_dz < 0.0) {
        return -1e30;
    }
    return h - env_dz;
}

@compute @workgroup_size(16, 16, 1)
fn main(
    @builtin(global_invocation_id) gid: vec3<u32>,
    @builtin(workgroup_id) wid: vec3<u32>,
) {
    let it = gid.x;
    let iu = gid.y;
    let nu = u.dims.x;
    let ntheta = u.dims.y;
    if (iu >= nu || it >= ntheta) {
        return;
    }

    let tiles_theta = u.dims.z;
    let num_tiles = u.dims.w;
    let tile_id = wid.y * tiles_theta + wid.x;
    let start = tile_data[tile_id];
    let end = tile_data[tile_id + 1u];
    if (start >= end) {
        return;
    }

    let idx = iu * ntheta + it;
    var r_cur = r_stock[idx];
    if (r_cur <= 1e-4) {
        return;
    }
    var op_tag = last_op_idx[idx];
    let r_tgt = r_target[idx];
    var cell_mutated = false;

    let e1 = u.e1_and_u_min.xyz;
    let u_min = u.e1_and_u_min.w;
    let e2 = u.e2_and_du.xyz;
    let du = u.e2_and_du.w;
    let u_vec = u.u_vec_and_dtheta.xyz;
    let dtheta = u.u_vec_and_dtheta.w;
    let u_center = u.params.x;
    let pol = u.params.y;

    let u_coord = u_min + (f32(iu) + 0.5) * du;
    let axis_pt_rel = u_vec * (u_coord - u_center);
    let theta = f32(it) * dtheta;
    let d_rad = e1 * cos(theta) + e2 * (sin(theta) * pol);

    let list_base = num_tiles + 1u;

    for (var k: u32 = start; k < end; k = k + 1u) {
        if (r_cur <= 1e-4) {
            break;
        }
        let p_idx = tile_data[list_base + k];
        let pose = poses[p_idx];
        let prof = profiles[pose.info.y];
        let flags = pose.info.z;
        let verify = (flags & POSE_FLAG_VERIFY) != 0u;
        let is_rapid = (flags & POSE_FLAG_RAPID) != 0u;
        let pose_index = pose.info.w;

        let tip_part_rel = pose.tip_and_rapid_tol.xyz;
        let spindle = pose.spindle_and_gouge_tol.xyz;

        let v0 = axis_pt_rel - tip_part_rel;
        let h0 = dot(v0, spindle);
        let d_dot_s = dot(d_rad, spindle);

        // Cancellation-free orthogonal projections for quadratic cylinder intersection
        let v0_perp = v0 - spindle * h0;
        let d_perp = d_rad - spindle * d_dot_s;
        let a = max(dot(d_perp, d_perp), 0.0);
        let b = dot(v0_perp, d_perp);
        let c = max(dot(v0_perp, v0_perp), 0.0);

        let r_cut = max(prof.cutting_radius_mm, 0.05);
        let r_holder = max(prof.holder_radius_mm, r_cut);
        let stickout = max(prof.stickout_length_mm, prof.flute_length_mm);

        // Rule 4: Collet / holder collision
        if (verify && (flags & POSE_FLAG_CHECK_HOLDER) != 0u) {
            let h_int = solve_cylinder_ray_interval(a, b, c, r_holder);
            if (h_int.valid && h_int.hi > 0.0 && h_int.lo < r_cur) {
                let r_chk = min(r_cur, h_int.hi);
                let h_chk = h0 + r_chk * d_dot_s;
                if (h_chk > stickout + 0.05) {
                    let excess = min(r_cur - max(h_int.lo, 0.0), h_chk - stickout);
                    if (excess > 0.05) {
                        record_violation(SLOT_HOLDER, excess, pose_index);
                    }
                }
            }
        }

        // Cutter cylinder interval
        let c_int = solve_cylinder_ray_interval(a, b, c, r_cut);
        if (!c_int.valid || c_int.hi <= 0.0 || c_int.lo >= r_cur || c_int.hi < r_cur - 1e-4) {
            continue;
        }
        let r_enter = min(r_cur, c_int.hi);
        let r_exit = max(c_int.lo, 0.0);
        if (r_enter <= r_exit + 1e-6) {
            continue;
        }

        let c_enter = eval_clearance(r_enter, v0, d_rad, spindle, prof, r_cut);
        if (c_enter <= 1e-5) {
            continue;
        }

        let c_exit = eval_clearance(r_exit, v0, d_rad, spindle, prof, r_cut);
        var r_new: f32;
        if (c_exit >= 0.0) {
            r_new = r_exit;
        } else if (prof.shape_kind == SHAPE_FLAT && d_dot_s > 1e-6) {
            r_new = clamp(-h0 / d_dot_s, r_exit, r_enter);
        } else {
            var lo = r_exit;
            if (prof.shape_kind != SHAPE_BALL && d_dot_s > 1e-6) {
                lo = min(max(r_exit, -h0 / d_dot_s), r_enter);
            }
            var hi = r_enter;
            for (var iter: u32 = 0u; iter < 11u; iter = iter + 1u) {
                let mid = 0.5 * (lo + hi);
                if (eval_clearance(mid, v0, d_rad, spindle, prof, r_cut) >= 0.0) {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            r_new = hi;
        }

        if (r_new < r_cur - 1e-5) {
            if (verify) {
                let cut_depth = r_cur - r_new;
                if (cut_depth > 0.02) {
                    let h_enter = h0 + r_enter * d_dot_s;
                    if (h_enter > prof.effective_reach_mm + 0.05) {
                        record_violation(SLOT_FLUTE, h_enter - prof.effective_reach_mm, pose_index);
                    }
                }
                if (is_rapid && cut_depth > pose.tip_and_rapid_tol.w) {
                    record_violation(SLOT_RAPID, cut_depth, pose_index);
                }
                if (r_tgt >= 0.0 && r_new < r_tgt - pose.spindle_and_gouge_tol.w) {
                    record_violation(SLOT_GOUGE, r_tgt - r_new, pose_index);
                }
            }

            r_cur = max(r_new, 0.0);
            op_tag = pose.info.x;
            cell_mutated = true;
        }
    }

    if (cell_mutated) {
        r_stock[idx] = r_cur;
        last_op_idx[idx] = op_tag;
        atomicStore(&violations.words[WORD_MUTATED], 1u);
    }
}
