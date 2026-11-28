use chip_math::{Aabb3, DVec3};
use rayon::prelude::*;

use crate::types::{Axis3, RadialCutterProfile, SpanRuleViolations, StockVerifyTolerances};

use super::super::grid::{
    DexelSpan, INLINE_DEXEL_SLOTS, TriDexelStock, load_row_ray_into, query_overlap_length,
    query_target_gouge_depth, subtract_dexel_interval, write_row_ray_from,
};
use super::{OrientedPose3D, SweepContext};

#[inline]
fn aabb_overlaps(min_a: DVec3, max_a: DVec3, stock: Aabb3) -> bool {
    max_a.x >= stock.min.x
        && min_a.x <= stock.max.x
        && max_a.y >= stock.min.y
        && min_a.y <= stock.max.y
        && max_a.z >= stock.min.z
        && min_a.z <= stock.max.z
}

/// Computes the tight stock-intersecting world-space AABB `(aabb_min, aabb_max)` for an arbitrary
/// [`OrientedPose3D`] together with `check_holder` (`true` only when the holder/collet envelope
/// above `stickout_length` actually intersects `stock_aabb`). Returns `None` when no part of the
/// cutter/shank/holder intersects `stock_aabb` (e.g., clearance rapids above the workpiece).
#[inline]
pub(crate) fn oriented_pose_stock_aabb(
    pose: OrientedPose3D,
    prof: &RadialCutterProfile,
    stock_aabb: Aabb3,
    verify: bool,
) -> Option<(DVec3, DVec3, bool)> {
    let r_cut = prof.cutting_radius_mm.max(0.05);
    let max_h = prof.horizontal_cut_max_height_mm();
    let flute_top = pose.tip + pose.spindle * max_h;
    let mut bmin = pose.tip.min(flute_top) - DVec3::splat(r_cut);
    let mut bmax = pose.tip.max(flute_top) + DVec3::splat(r_cut);
    let mut active = aabb_overlaps(bmin, bmax, stock_aabb);
    let mut check_holder = false;

    if verify {
        let stickout = prof.stickout_length_mm.max(prof.flute_length_mm);
        let reach = prof.effective_reach_mm();
        if stickout > reach + 1e-4 {
            let s_lo = pose.tip + pose.spindle * reach;
            let s_hi = pose.tip + pose.spindle * stickout;
            let r_sh = prof.shank_radius_mm.max(r_cut);
            let smin = s_lo.min(s_hi) - DVec3::splat(r_sh);
            let smax = s_lo.max(s_hi) + DVec3::splat(r_sh);
            if aabb_overlaps(smin, smax, stock_aabb) {
                if active {
                    bmin = bmin.min(smin);
                    bmax = bmax.max(smax);
                } else {
                    bmin = smin;
                    bmax = smax;
                    active = true;
                }
            }
        }
        let h_lo = pose.tip + pose.spindle * stickout;
        let h_hi = pose.tip + pose.spindle * (stickout + 40.0);
        let r_h = prof.holder_radius_mm.max(r_cut);
        let hmin = h_lo.min(h_hi) - DVec3::splat(r_h);
        let hmax = h_lo.max(h_hi) + DVec3::splat(r_h);
        if aabb_overlaps(hmin, hmax, stock_aabb) {
            check_holder = true;
            if active {
                bmin = bmin.min(hmin);
                bmax = bmax.max(hmax);
            } else {
                bmin = hmin;
                bmax = hmax;
                active = true;
            }
        }
    }

    active.then_some((bmin, bmax, check_holder))
}

/// Solves ray `P(w) = p_base + w * d_axis` intersection with an oriented finite cylinder of
/// radius `radius` and axial span `h(w) ∈ [h_lo, h_hi]` along `spindle`.
#[inline]
#[allow(clippy::too_many_arguments)]
fn solve_oriented_cylinder_interval(
    a: f64,
    b: f64,
    c: f64,
    h0: f64,
    d_dot_s: f64,
    radius: f64,
    h_lo: f64,
    h_hi: f64,
) -> Option<(f64, f64)> {
    if radius <= 1e-9 || h_hi <= h_lo + 1e-9 {
        return None;
    }
    let c_eff = c - radius * radius;
    let (mut w_lo, mut w_hi) = if a <= 1e-12 {
        if c_eff <= 0.0 {
            (-1e6, 1e6)
        } else {
            return None;
        }
    } else {
        let disc = b * b - a * c_eff;
        if disc < 0.0 {
            return None;
        }
        let sd = disc.sqrt();
        ((-b - sd) / a, (-b + sd) / a)
    };

    if d_dot_s.abs() <= 1e-12 {
        if h0 < h_lo || h0 > h_hi {
            return None;
        }
    } else {
        let t0 = (h_lo - h0) / d_dot_s;
        let t1 = (h_hi - h0) / d_dot_s;
        w_lo = w_lo.max(t0.min(t1));
        w_hi = w_hi.min(t0.max(t1));
    }

    (w_hi > w_lo + 1e-6).then_some((w_lo, w_hi))
}

/// Exact ray-cutter cutting interval `[w_lo, w_hi]` along axis-aligned ray `P(w) = p_base + w * d`
/// for an arbitrarily oriented cutter pose `(tip, spindle)` and effective outer radius `r_lim`.
#[inline]
#[allow(clippy::too_many_arguments)]
fn eval_oriented_pose_cut_interval(
    v0: DVec3,
    d_axis: DVec3,
    spindle: DVec3,
    a: f64,
    b: f64,
    c: f64,
    h0: f64,
    d_dot_s: f64,
    prof: &RadialCutterProfile,
    r_lim: f64,
) -> Option<(f64, f64)> {
    let r_eff = r_lim.min(prof.cutting_radius_mm);
    if r_eff <= 1e-6 {
        return None;
    }
    let h_max = prof.horizontal_cut_max_height_mm();
    let (w_lo, w_hi) = solve_oriented_cylinder_interval(a, b, c, h0, d_dot_s, r_eff, 0.0, h_max)?;

    match prof.envelope().shape() {
        chip_tools::CutterEnvelopeShape::Flat { .. } => Some((w_lo, w_hi)),
        chip_tools::CutterEnvelopeShape::Ball { .. } => {
            let r_ball = prof.cutting_radius_mm;
            let mut out_lo = f64::INFINITY;
            let mut out_hi = f64::NEG_INFINITY;

            // Lower hemisphere: sphere centered at `tip + r_ball * spindle`, radius `r_eff`
            let q = v0 - spindle * r_ball;
            let bs = q.dot(d_axis);
            let cs = q.length_squared() - r_eff * r_eff;
            let disc = bs * bs - cs;
            if disc >= 0.0 {
                let sd = disc.sqrt();
                let mut s_lo = -bs - sd;
                let mut s_hi = -bs + sd;
                if d_dot_s.abs() > 1e-12 {
                    let t0 = -h0 / d_dot_s;
                    let t1 = (h_max - h0) / d_dot_s;
                    s_lo = s_lo.max(t0.min(t1));
                    s_hi = s_hi.min(t0.max(t1));
                } else if h0 < 0.0 || h0 > h_max {
                    s_hi = s_lo - 1.0;
                }
                if s_hi > s_lo + 1e-6 {
                    out_lo = out_lo.min(s_lo);
                    out_hi = out_hi.max(s_hi);
                }
            }
            // Upper cylindrical flutes `[r_ball, h_max]`
            if let Some((u_lo, u_hi)) =
                solve_oriented_cylinder_interval(a, b, c, h0, d_dot_s, r_eff, r_ball, h_max)
            {
                out_lo = out_lo.min(u_lo);
                out_hi = out_hi.max(u_hi);
            }
            (out_hi > out_lo + 1e-6).then_some((out_lo, out_hi))
        }
        _ => {
            let dr_skin = (prof.cutting_radius_mm - r_eff).max(0.0);
            let eval_inside = |w: f64| -> f64 {
                let rho = (a * w * w + 2.0 * b * w + c).max(0.0).sqrt() + dr_skin;
                let h = h0 + w * d_dot_s;
                let Some((dz_lo, dz_hi)) = prof.cutting_z_interval_at_radius(rho) else {
                    return -1e30;
                };
                (h - dz_lo).min(dz_hi - h)
            };

            let g_lo = eval_inside(w_lo);
            let g_hi = eval_inside(w_hi);
            if g_lo >= 0.0 && g_hi >= 0.0 {
                return Some((w_lo, w_hi));
            }
            if g_lo >= 0.0 {
                let mut lo = w_lo;
                let mut hi = w_hi;
                for _ in 0..11 {
                    let mid = 0.5 * (lo + hi);
                    if eval_inside(mid) >= 0.0 {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                return (lo > w_lo + 1e-6).then_some((w_lo, lo));
            }
            if g_hi >= 0.0 {
                let mut lo = w_lo;
                let mut hi = w_hi;
                for _ in 0..11 {
                    let mid = 0.5 * (lo + hi);
                    if eval_inside(mid) >= 0.0 {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                return (w_hi > hi + 1e-6).then_some((hi, w_hi));
            }

            // Both endpoints outside: probe interior for concave peak
            let w_min_rho = if a > 1e-12 {
                (-b / a).clamp(w_lo, w_hi)
            } else {
                0.5 * (w_lo + w_hi)
            };
            let probes = [
                w_min_rho,
                w_lo + 0.25 * (w_hi - w_lo),
                0.5 * (w_lo + w_hi),
                w_lo + 0.75 * (w_hi - w_lo),
            ];
            let mut best_w = w_min_rho;
            let mut best_g = f64::NEG_INFINITY;
            for wp in probes {
                let gv = eval_inside(wp);
                if gv > best_g {
                    best_g = gv;
                    best_w = wp;
                }
            }
            if best_g <= 0.0 {
                return None;
            }
            let mut lo_a = w_lo;
            let mut hi_a = best_w;
            for _ in 0..11 {
                let mid = 0.5 * (lo_a + hi_a);
                if eval_inside(mid) >= 0.0 {
                    hi_a = mid;
                } else {
                    lo_a = mid;
                }
            }
            let mut lo_b = best_w;
            let mut hi_b = w_hi;
            for _ in 0..11 {
                let mid = 0.5 * (lo_b + hi_b);
                if eval_inside(mid) >= 0.0 {
                    lo_b = mid;
                } else {
                    hi_b = mid;
                }
            }
            (lo_b > hi_a + 1e-6).then_some((hi_a, lo_b))
        }
    }
}

/// Evaluates an arbitrary [`OrientedPose3D`] against a single ray on plane `axis` at `(pu, pv)`.
#[allow(clippy::too_many_arguments)]
pub(super) fn apply_oriented_pose_to_ray(
    axis: Axis3,
    pu: f64,
    pv: f64,
    pose: OrientedPose3D,
    prof: &RadialCutterProfile,
    is_rapid: bool,
    tol: StockVerifyTolerances,
    op_idx: u32,
    verify: bool,
    check_holder: bool,
    target_spans: Option<&[DexelSpan]>,
    scratch: &mut Vec<DexelSpan>,
) -> (bool, SpanRuleViolations) {
    let mut rv = SpanRuleViolations::default();
    if scratch.is_empty() {
        return (false, rv);
    }
    let p_base = axis.from_uvw(pu, pv, 0.0);
    let d_axis = axis.unit_vec();
    let spindle = pose.spindle;
    let v0 = p_base - pose.tip;
    let h0 = v0.dot(spindle);
    let d_dot_s = d_axis.dot(spindle);
    let v0_perp = v0 - spindle * h0;
    let d_perp = d_axis - spindle * d_dot_s;
    let a = d_perp.length_squared().max(0.0);
    let b = v0_perp.dot(d_perp);
    let c = v0_perp.length_squared().max(0.0);

    let r_cut = prof.cutting_radius_mm;
    let shank_skin = (prof.shank_radius_mm - tol.radial_wall_skin_mm).max(0.0);
    let neck_skin = (prof.neck_radius_mm - tol.radial_wall_skin_mm).max(0.0);
    let r_inner_skin = (r_cut - tol.radial_wall_skin_mm).max(0.0);

    if verify {
        let stickout = prof.stickout_length_mm.max(prof.flute_length_mm);
        if check_holder
            && let Some((hw_lo, hw_hi)) = solve_oriented_cylinder_interval(
                a,
                b,
                c,
                h0,
                d_dot_s,
                prof.holder_radius_mm,
                stickout,
                stickout + 40.0,
            )
        {
            let pen = query_overlap_length(scratch, hw_lo, hw_hi);
            if pen > 0.05 {
                rv.record_holder(pen, pose.tip_wcs);
            }
        }
        if !is_rapid {
            let reach = prof.effective_reach_mm();
            if stickout > reach + 1e-4
                && let Some((fw_lo, fw_hi)) = solve_oriented_cylinder_interval(
                    a, b, c, h0, d_dot_s, shank_skin, reach, stickout,
                )
            {
                let rub = query_overlap_length(scratch, fw_lo, fw_hi);
                if rub > 0.15 {
                    rv.record_flute(rub, pose.tip_wcs);
                }
            }
            if prof.has_relieved_neck
                && prof.neck_length_mm > prof.flute_length_mm + 0.05
                && let Some((nw_lo, nw_hi)) = solve_oriented_cylinder_interval(
                    a,
                    b,
                    c,
                    h0,
                    d_dot_s,
                    neck_skin,
                    prof.flute_length_mm + 0.05,
                    prof.neck_length_mm,
                )
            {
                let ov = query_overlap_length(scratch, nw_lo, nw_hi);
                if ov > 0.10 {
                    rv.record_neck(ov, pose.tip_wcs);
                }
            }
        }
    }

    let mut ray_mut = false;
    if let Some((w_lo, w_hi)) =
        eval_oriented_pose_cut_interval(v0, d_axis, spindle, a, b, c, h0, d_dot_s, prof, r_cut)
    {
        if verify && is_rapid {
            let ov = query_overlap_length(scratch, w_lo, w_hi);
            if ov > tol.rapid_crash_tol_mm {
                rv.record_rapid(ov, pose.tip_wcs);
            }
        }
        if verify
            && let Some(t_spans) = target_spans
            && let Some((w_in_lo, w_in_hi)) = eval_oriented_pose_cut_interval(
                v0,
                d_axis,
                spindle,
                a,
                b,
                c,
                h0,
                d_dot_s,
                prof,
                r_inner_skin,
            )
            && let Some((pen, _)) = query_target_gouge_depth(
                t_spans,
                w_in_lo,
                w_in_hi,
                tol.gouge_tol_mm + tol.radial_wall_skin_mm,
            )
        {
            rv.record_gouge(pen, pose.tip_wcs);
        }
        if subtract_dexel_interval(scratch, w_lo, w_hi, op_idx) > 0.0 {
            ray_mut = true;
        }
    }

    (ray_mut, rv)
}

impl TriDexelStock {
    pub(super) fn apply_oriented_pose(
        &mut self,
        pose: OrientedPose3D,
        ctx: &SweepContext<'_>,
        verify: bool,
    ) -> (bool, SpanRuleViolations) {
        let Some((aabb_min, aabb_max, check_holder)) =
            oriented_pose_stock_aabb(pose, ctx.prof, self.aabb_wcs(), verify)
        else {
            return (false, SpanRuleViolations::default());
        };

        let mut violations = SpanRuleViolations::default();
        let mut mutated = false;
        for axis in Axis3::ALL {
            let (p_mut, p_viol) = self.sweep_oriented_pose_plane(
                axis,
                pose,
                aabb_min,
                aabb_max,
                ctx,
                verify,
                check_holder,
            );
            mutated |= p_mut;
            violations.merge(p_viol);
        }

        (mutated, violations)
    }

    #[allow(clippy::too_many_arguments)]
    fn sweep_oriented_pose_plane(
        &mut self,
        axis: Axis3,
        pose: OrientedPose3D,
        aabb_min: DVec3,
        aabb_max: DVec3,
        ctx: &SweepContext<'_>,
        verify: bool,
        check_holder: bool,
    ) -> (bool, SpanRuleViolations) {
        let u_ax = axis.u_axis();
        let v_ax = axis.v_axis();
        let plane = self.plane_mut(axis);
        let (Some((u0, u1)), Some((v0, v1))) = (
            plane.index_span_u(u_ax.get(aabb_min), u_ax.get(aabb_max)),
            plane.index_span_v(v_ax.get(aabb_min), v_ax.get(aabb_max)),
        ) else {
            return (false, SpanRuleViolations::default());
        };

        let nu = plane.nu;
        let ou = plane.origin_u_mm;
        let ov = plane.origin_v_mm;
        let ow = plane.origin_w_mm;
        let du = plane.du_mm;
        let dv = plane.dv_mm;
        let row_slot_stride = nu * INLINE_DEXEL_SLOTS;
        let target_plane = ctx.target.map(|t| t.plane(axis));
        let span_rays = (u1 - u0 + 1) * (v1 - v0 + 1);

        let slots_slice = &mut plane.slots[v0 * row_slot_stride..(v1 + 1) * row_slot_stride];
        let counts_slice = &mut plane.counts[v0 * nu..(v1 + 1) * nu];
        let overflow_ref = &plane.overflow;

        let (p_mut, p_viol, overflow_deltas) = if span_rays < 2048 {
            let mut p_mut = false;
            let mut p_viol = SpanRuleViolations::default();
            let mut ov_deltas: Vec<(u32, Option<Vec<DexelSpan>>)> = Vec::new();
            let mut scratch: Vec<DexelSpan> = Vec::with_capacity(8);

            for (v_off, (row_slots, row_counts)) in slots_slice
                .chunks_mut(row_slot_stride)
                .zip(counts_slice.chunks_mut(nu))
                .enumerate()
            {
                let iv = v0 + v_off;
                let pv = ov + (iv as f64 + 0.5) * dv;
                let row_base = iv * nu;

                for iu in u0..=u1 {
                    if row_counts[iu] == 0 {
                        continue;
                    }
                    let ray_idx = (row_base + iu) as u32;
                    let was_ov = load_row_ray_into(
                        iu,
                        ray_idx,
                        row_slots,
                        row_counts,
                        overflow_ref,
                        ow,
                        &mut scratch,
                    );
                    if scratch.is_empty() {
                        continue;
                    }
                    let pu = ou + (iu as f64 + 0.5) * du;
                    let target_spans_owned =
                        target_plane.map(|tp| tp.ray_spans_by_index(row_base + iu));
                    let (ray_mut, ray_viol) = apply_oriented_pose_to_ray(
                        axis,
                        pu,
                        pv,
                        pose,
                        ctx.prof,
                        ctx.is_rapid,
                        ctx.tol,
                        ctx.op_idx,
                        verify,
                        check_holder,
                        target_spans_owned.as_deref(),
                        &mut scratch,
                    );
                    p_viol.merge(ray_viol);
                    if ray_mut {
                        p_mut = true;
                        write_row_ray_from(
                            iu,
                            ray_idx,
                            row_slots,
                            row_counts,
                            ow,
                            &scratch,
                            was_ov,
                            &mut ov_deltas,
                        );
                    }
                }
            }
            (p_mut, p_viol, ov_deltas)
        } else {
            slots_slice
                .par_chunks_mut(row_slot_stride)
                .zip(counts_slice.par_chunks_mut(nu))
                .enumerate()
                .map(|(v_off, (row_slots, row_counts))| {
                    let iv = v0 + v_off;
                    let pv = ov + (iv as f64 + 0.5) * dv;
                    let row_base = iv * nu;
                    let mut row_mut = false;
                    let mut rv = SpanRuleViolations::default();
                    let mut row_ov_deltas: Vec<(u32, Option<Vec<DexelSpan>>)> = Vec::new();
                    let mut scratch: Vec<DexelSpan> = Vec::with_capacity(8);

                    for iu in u0..=u1 {
                        if row_counts[iu] == 0 {
                            continue;
                        }
                        let ray_idx = (row_base + iu) as u32;
                        let was_ov = load_row_ray_into(
                            iu,
                            ray_idx,
                            row_slots,
                            row_counts,
                            overflow_ref,
                            ow,
                            &mut scratch,
                        );
                        if scratch.is_empty() {
                            continue;
                        }
                        let pu = ou + (iu as f64 + 0.5) * du;
                        let target_spans_owned =
                            target_plane.map(|tp| tp.ray_spans_by_index(row_base + iu));
                        let (ray_mut, ray_viol) = apply_oriented_pose_to_ray(
                            axis,
                            pu,
                            pv,
                            pose,
                            ctx.prof,
                            ctx.is_rapid,
                            ctx.tol,
                            ctx.op_idx,
                            verify,
                            check_holder,
                            target_spans_owned.as_deref(),
                            &mut scratch,
                        );
                        rv.merge(ray_viol);
                        if ray_mut {
                            row_mut = true;
                            write_row_ray_from(
                                iu,
                                ray_idx,
                                row_slots,
                                row_counts,
                                ow,
                                &scratch,
                                was_ov,
                                &mut row_ov_deltas,
                            );
                        }
                    }
                    (row_mut, rv, row_ov_deltas)
                })
                .reduce(
                    || (false, SpanRuleViolations::default(), Vec::new()),
                    |(m0, mut v0, mut d0), (m1, v1, mut d1)| {
                        v0.merge(v1);
                        d0.append(&mut d1);
                        (m0 || m1, v0, d0)
                    },
                )
        };

        for (ray_idx, entry) in overflow_deltas {
            match entry {
                Some(vec) => {
                    plane.overflow.insert(ray_idx, vec);
                }
                None => {
                    plane.overflow.remove(&ray_idx);
                }
            }
        }

        (p_mut, p_viol)
    }
}
