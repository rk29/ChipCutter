use chip_math::{DVec2, DVec3};

use crate::types::{
    Axis3, RotaryStockSpec, SpanRuleViolations, StockVerifyTolerances, SweepToolSpec,
};

use super::super::grid::{
    DexelSpan, query_overlap_length, query_target_gouge_depth, subtract_dexel_interval,
};
use super::SweptPrimitive;
use super::cardinal::{
    cardinal_axial_chord, cardinal_horizontal_chord, eval_horizontal_ray_cut_interval,
    eval_horizontal_ray_cylinder_interval, eval_z_ray_cut_interval, eval_z_ray_cylinder_interval,
    eval_z_ray_thread_mill_intervals, map_cz_interval_to_w,
};
use super::oriented::apply_oriented_pose_to_ray;

/// Exact `f64` single-ray replay of one [`SweptPrimitive`] on plane `axis` at ray center `(pu, pv)`.
/// Shared by `WgpuComputeBackend`'s lossless hybrid overflow safety valve so rare `> K = 4`-span
/// rays execute identical subtraction and Rules 1–4 verification to the CPU Tri-Dexel oracle across
/// both 3-axis and 4th-axis rotary sweeps.
#[allow(clippy::too_many_arguments)]
pub(crate) fn replay_single_ray_primitive(
    axis: Axis3,
    pu: f64,
    pv: f64,
    pitch_mm: DVec3,
    prim: SweptPrimitive,
    rotary: Option<RotaryStockSpec>,
    tool: &SweepToolSpec,
    is_rapid: bool,
    tol: StockVerifyTolerances,
    op_idx: u32,
    verify: bool,
    target_spans: Option<&[DexelSpan]>,
    scratch: &mut Vec<DexelSpan>,
) -> (bool, SpanRuleViolations) {
    let mut rv = SpanRuleViolations::default();
    if scratch.is_empty() {
        return (false, rv);
    }
    let prof = &tool.profile;
    match prim {
        SweptPrimitive::OrientedPose(pose) => apply_oriented_pose_to_ray(
            axis,
            pu,
            pv,
            pose,
            prof,
            is_rapid,
            tol,
            op_idx,
            verify,
            verify,
            target_spans,
            scratch,
        ),
        SweptPrimitive::CardinalLinear {
            chord,
            spindle,
            rot_deg,
        } => {
            let to_diag_wcs = |pt_part: DVec3| -> DVec3 {
                if let Some(rot) = rotary
                    && rot_deg.abs() > 1e-6
                {
                    rot.rotate_point_wcs(pt_part, rot_deg)
                } else {
                    pt_part
                }
            };
            let r_cut = prof.cutting_radius_mm;
            let neck_skin = (prof.neck_radius_mm - tol.radial_wall_skin_mm).max(0.0);
            let shank_skin = (prof.shank_radius_mm - tol.radial_wall_skin_mm).max(0.0);
            let r_inner_skin = (r_cut - tol.radial_wall_skin_mm).max(0.0);
            let s_ax = spindle.axis();
            let s_sign = spindle.sign();

            if axis == s_ax {
                let mapped = cardinal_axial_chord(axis, s_sign, chord);
                let px = pu;
                let py = pv;
                let r_check = if verify {
                    r_cut.max(prof.holder_radius_mm)
                } else {
                    r_cut
                };
                if px < mapped.p0.x.min(mapped.p1.x) - r_check - 1e-6
                    || px > mapped.p0.x.max(mapped.p1.x) + r_check + 1e-6
                    || py < mapped.p0.y.min(mapped.p1.y) - r_check - 1e-6
                    || py > mapped.p0.y.max(mapped.p1.y) + r_check + 1e-6
                {
                    return (false, rv);
                }

                if verify {
                    let p0_xy = DVec2::new(mapped.p0.x, mapped.p0.y);
                    let p1_xy = DVec2::new(mapped.p1.x, mapped.p1.y);
                    let d_xy = p1_xy - p0_xy;
                    let l2 = d_xy.length_squared();
                    let t_c = if l2 > 1e-12 {
                        ((DVec2::new(px, py) - p0_xy).dot(d_xy) / l2).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };
                    let c_xy = p0_xy.lerp(p1_xy, t_c);
                    let dist_xy = (DVec2::new(px, py) - c_xy).length();
                    let z_tip_here = if l2 > 1e-12 {
                        mapped.p0.z + t_c * (mapped.p1.z - mapped.p0.z)
                    } else {
                        mapped.p0.z.min(mapped.p1.z)
                    };
                    let top_cz = if s_sign >= 0.0 {
                        scratch.last().map(|s| s.max_mm)
                    } else {
                        scratch.first().map(|s| -s.min_mm)
                    };
                    if let Some(top_z) = top_cz {
                        if dist_xy <= prof.holder_radius_mm {
                            let holder_z = z_tip_here + prof.stickout_length_mm;
                            let pen = top_z - holder_z;
                            if pen > 0.05 {
                                rv.record_holder(
                                    pen,
                                    to_diag_wcs(axis.from_uvw(px, py, s_sign * holder_z)),
                                );
                            }
                        }
                        if !is_rapid && dist_xy <= shank_skin {
                            let reach_top = z_tip_here + prof.effective_reach_mm();
                            let rub = top_z - reach_top;
                            if rub > 0.15 {
                                rv.record_flute(
                                    rub,
                                    to_diag_wcs(axis.from_uvw(px, py, s_sign * reach_top)),
                                );
                            }
                        }
                    }
                    if prof.has_relieved_neck && !is_rapid && dist_xy <= neck_skin {
                        let neck_lo = z_tip_here + prof.flute_length_mm + 0.05;
                        let neck_hi = z_tip_here + prof.neck_length_mm;
                        if neck_hi > neck_lo {
                            let (w_lo, w_hi) = map_cz_interval_to_w(neck_lo, neck_hi, s_sign);
                            let ov = query_overlap_length(scratch, w_lo, w_hi);
                            if ov > 0.10 {
                                rv.record_neck(
                                    ov,
                                    to_diag_wcs(axis.from_uvw(px, py, s_sign * neck_lo)),
                                );
                            }
                        }
                    }
                }

                let mut ray_mut = false;
                if prof.is_thread_mill {
                    let mut tm_intervals = Vec::new();
                    eval_z_ray_thread_mill_intervals(
                        mapped,
                        prof,
                        px,
                        py,
                        r_cut,
                        &mut tm_intervals,
                    );
                    if !tm_intervals.is_empty() {
                        if verify && is_rapid {
                            for &(cz_lo, cz_hi) in &tm_intervals {
                                let (w_lo, w_hi) = map_cz_interval_to_w(cz_lo, cz_hi, s_sign);
                                let ov = query_overlap_length(scratch, w_lo, w_hi);
                                if ov > tol.rapid_crash_tol_mm {
                                    rv.record_rapid(ov, to_diag_wcs(axis.from_uvw(px, py, w_lo)));
                                }
                            }
                        }
                        if verify
                            && let Some(t_spans) = target_spans
                            && let Some((cz_lo_in, cz_hi_in)) = eval_z_ray_cylinder_interval(
                                mapped,
                                px,
                                py,
                                neck_skin,
                                prof.neck_length_mm,
                            )
                        {
                            let (w_lo_in, w_hi_in) =
                                map_cz_interval_to_w(cz_lo_in, cz_hi_in, s_sign);
                            if let Some((pen, w_at)) = query_target_gouge_depth(
                                t_spans,
                                w_lo_in,
                                w_hi_in,
                                tol.gouge_tol_mm,
                            ) {
                                rv.record_gouge(pen, to_diag_wcs(axis.from_uvw(px, py, w_at)));
                            }
                        }
                        for &(cz_lo, cz_hi) in &tm_intervals {
                            let (w_lo, w_hi) = map_cz_interval_to_w(cz_lo, cz_hi, s_sign);
                            if subtract_dexel_interval(scratch, w_lo, w_hi, op_idx) > 0.0 {
                                ray_mut = true;
                            }
                        }
                    }
                } else if let Some((cz_lo, cz_hi, _)) =
                    eval_z_ray_cut_interval(mapped, prof, tool.is_flat_endmill, px, py, r_cut)
                {
                    let (w_lo, w_hi) = map_cz_interval_to_w(cz_lo, cz_hi, s_sign);
                    if verify && is_rapid {
                        let ov = query_overlap_length(scratch, w_lo, w_hi);
                        if ov > tol.rapid_crash_tol_mm {
                            rv.record_rapid(ov, to_diag_wcs(axis.from_uvw(px, py, w_lo)));
                        }
                    }
                    if verify
                        && let Some(t_spans) = target_spans
                        && let Some((cz_lo_in, cz_hi_in, _)) = eval_z_ray_cut_interval(
                            mapped,
                            prof,
                            tool.is_flat_endmill,
                            px,
                            py,
                            r_inner_skin,
                        )
                    {
                        let (w_lo_in, w_hi_in) = map_cz_interval_to_w(cz_lo_in, cz_hi_in, s_sign);
                        if let Some((pen, w_at)) =
                            query_target_gouge_depth(t_spans, w_lo_in, w_hi_in, tol.gouge_tol_mm)
                        {
                            rv.record_gouge(pen, to_diag_wcs(axis.from_uvw(px, py, w_at)));
                        }
                    }
                    if subtract_dexel_interval(scratch, w_lo, w_hi, op_idx) > 0.0 {
                        ray_mut = true;
                    }
                }
                (ray_mut, rv)
            } else {
                let (mapped, spindle_in_u) = cardinal_horizontal_chord(axis, spindle, chord);
                let (pz_c, pu_c) = if spindle_in_u {
                    (s_sign * pu, pv)
                } else {
                    (s_sign * pv, pu)
                };
                let lat0 = mapped.p0.y.min(mapped.p1.y) - r_cut;
                let lat1 = mapped.p0.y.max(mapped.p1.y) + r_cut;
                if pu_c < lat0 - 1e-6
                    || pu_c > lat1 + 1e-6
                    || pz_c < mapped.min_z() - 1e-6
                    || pz_c > mapped.max_z() + prof.horizontal_cut_max_height_mm() + 1e-6
                {
                    return (false, rv);
                }
                let dz_step_mm = 0.25 * s_ax.get(pitch_mm);
                let wall_skin = tol.gouge_tol_mm + tol.radial_wall_skin_mm;
                let flute_wall_skin = tol.radial_wall_skin_mm + (tol.gouge_tol_mm - 0.02).max(0.0);
                let mut ray_mut = false;
                if let Some((w_lo, w_hi)) = eval_horizontal_ray_cut_interval(
                    mapped, prof, 0.0, pz_c, pu_c, false, dz_step_mm,
                ) {
                    if verify && is_rapid {
                        let ov = query_overlap_length(scratch, w_lo, w_hi);
                        if ov > tol.rapid_crash_tol_mm {
                            rv.record_rapid(
                                ov,
                                to_diag_wcs(axis.from_uvw(pu, pv, 0.5 * (w_lo + w_hi))),
                            );
                        }
                    }
                    if verify
                        && pz_c >= mapped.min_z() + tol.gouge_tol_mm
                        && let Some(t_spans) = target_spans
                        && let Some((w_lo_in, w_hi_in)) = if prof.is_thread_mill {
                            eval_horizontal_ray_cylinder_interval(
                                mapped,
                                neck_skin,
                                prof.neck_length_mm,
                                pz_c,
                                pu_c,
                                false,
                            )
                        } else {
                            eval_horizontal_ray_cut_interval(
                                mapped,
                                prof,
                                flute_wall_skin,
                                pz_c,
                                pu_c,
                                false,
                                dz_step_mm,
                            )
                        }
                        && let Some((pen, w_at)) =
                            query_target_gouge_depth(t_spans, w_lo_in, w_hi_in, wall_skin)
                    {
                        rv.record_gouge(pen, to_diag_wcs(axis.from_uvw(pu, pv, w_at)));
                    }
                    if subtract_dexel_interval(scratch, w_lo, w_hi, op_idx) > 0.0 {
                        ray_mut = true;
                    }
                }
                (ray_mut, rv)
            }
        }
    }
}
