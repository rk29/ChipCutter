use chip_bvh::TriMesh3D;
use chip_math::{DVec2, DVec3};
use chip_tools::AnalyticCutterEnvelope;

use crate::{
    cl_data::MotionRole,
    strategies::StrategyExecContext,
    toolpath::{ToolpathBuilder, XyArcMove},
};

use super::trim::{is_air_position_clear, is_vertical_column_clear};
use super::{SolvedStation, TrimmedSubChain};

/// Emits a single collision-free 3D deburr sub-chain with smooth tangent arc lead-in/lead-out and
/// verified collision-free vertical approach/retract columns.
#[allow(clippy::too_many_arguments)]
pub(super) fn emit_deburr_subchain(
    ctx: &StrategyExecContext<'_>,
    sub: &TrimmedSubChain,
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    target_mesh: &TriMesh3D,
    wall_clearance: f64,
    retract_z: f64,
    cut_role: MotionRole,
    cand_buf: &mut Vec<usize>,
    builder: &mut ToolpathBuilder,
) -> bool {
    let Some(first) = sub.stations.first() else {
        return false;
    };
    let Some(last) = sub.stations.last() else {
        return false;
    };

    let p0 = first.sol.tip_wcs;
    let out0 = first.sol.outward_xy;
    let t0_xy = DVec2::new(first.sample.tangent.x, first.sample.tangent.y).normalize_or_zero();

    let p1 = last.sol.tip_wcs;
    let out1 = last.sol.outward_xy;
    let t1_xy = DVec2::new(last.sample.tangent.x, last.sample.tangent.y).normalize_or_zero();

    let pref_lead_r = if ctx.linking.lead_in.enabled {
        ctx.linking
            .lead_in
            .horizontal_radius
            .value_mm
            .clamp(0.4, 4.0)
    } else {
        0.0
    };

    // Resolve safe tangent lead-in arc + safe vertical plunge column!
    let (plunge_pt, lead_in_arc, linear_approach_pt) = plan_safe_lead(
        p0,
        t0_xy,
        out0,
        true,
        sub.is_underside,
        pref_lead_r,
        retract_z,
        envelope,
        stickout_mm,
        target_mesh,
        wall_clearance,
        cand_buf,
    );

    builder.pass_count += 1;

    // 1. Safe rapid to approach XY at `retract_z`
    if let Some(cur) = builder.current_pos()
        && cur.z < retract_z - 1e-6
    {
        builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
    }
    builder.rapid_to(DVec3::new(plunge_pt.x, plunge_pt.y, retract_z));
    let feed_z = ctx.heights.feed_z_wcs.max(plunge_pt.z);
    if feed_z < retract_z - 1e-4 {
        builder.rapid_to(DVec3::new(plunge_pt.x, plunge_pt.y, feed_z));
    }

    // 2. Plunge vertically in open air to approach Z
    builder.linear_to(
        plunge_pt,
        ctx.cutting.plunge_feed_mm_min,
        MotionRole::Plunge,
    );

    // 3. Horizontal linear approach (if pushed out past an overhang) + smooth tangent lead-in arc
    if let Some(mid_pt) = linear_approach_pt
        && mid_pt.distance(plunge_pt) > 1e-4
    {
        builder.linear_to(mid_pt, ctx.cutting.lead_feed_mm_min, MotionRole::LeadIn);
    }
    if let Some((center_xy, radius_mm, ccw)) = lead_in_arc {
        builder.arc_xy_to(XyArcMove {
            end_wcs: p0,
            center_xy,
            radius_mm,
            ccw,
            turns: 0,
            feed_mm_min: ctx.cutting.lead_feed_mm_min,
            role: MotionRole::LeadIn,
        });
    } else if builder.current_pos().is_some_and(|c| c.distance(p0) > 1e-4) {
        builder.linear_to(p0, ctx.cutting.lead_feed_mm_min, MotionRole::LeadIn);
    }

    // 4. Cut exact 3D non-planar tool-tip trajectory with smooth circular corner-rolls around sharp
    //    outer convex corners (preserving full flute & relieved-neck clearance around 90° miters!).
    let mut prev_st = first;
    for st in sub.stations.iter().skip(1) {
        let out_dot = prev_st
            .sol
            .outward_xy
            .dot(st.sol.outward_xy)
            .clamp(-1.0, 1.0);
        let corner_dist = prev_st.sample.pos_wcs.distance(st.sample.pos_wcs);
        if out_dot < 0.995 && corner_dist <= 0.25 {
            emit_corner_roll(
                prev_st,
                st,
                ctx.cutting.cutting_feed_mm_min,
                cut_role,
                builder,
            );
        } else if builder
            .current_pos()
            .is_none_or(|c| c.distance(st.sol.tip_wcs) > 1e-6)
        {
            builder.linear_to(st.sol.tip_wcs, ctx.cutting.cutting_feed_mm_min, cut_role);
        }
        prev_st = st;
    }

    // Close the corner smoothly on closed loops whose first and last edges meet at a sharp angle!
    let mut exit_tip = p1;
    let mut exit_tan_xy = t1_xy;
    let mut exit_out_xy = out1;
    if sub.closed
        && sub.stations.len() >= 3
        && last.sol.outward_xy.dot(first.sol.outward_xy) < 0.995
        && last.sample.pos_wcs.distance(first.sample.pos_wcs) <= 0.25
    {
        emit_corner_roll(
            last,
            first,
            ctx.cutting.cutting_feed_mm_min,
            cut_role,
            builder,
        );
        exit_tip = first.sol.tip_wcs;
        exit_tan_xy = t0_xy;
        exit_out_xy = out0;
    }

    // Optional closed-loop seam overlap to avoid dwell witness marks (tracking exact final exit state!)
    if sub.closed && sub.stations.len() >= 3 {
        let ov_target = ctx.linking.seam_overlap.value_mm.clamp(0.0, 2.0);
        if ov_target > 1e-3 {
            let mut acc = 0.0_f64;
            for w in sub.stations.windows(2) {
                let seg_len = w[0].sol.tip_wcs.distance(w[1].sol.tip_wcs);
                if seg_len < 1e-6 {
                    continue;
                }
                if w[0].sol.outward_xy.dot(w[1].sol.outward_xy) < 0.995 {
                    break;
                }
                if acc + seg_len >= ov_target {
                    let frac = ((ov_target - acc) / seg_len).clamp(0.0, 1.0);
                    let pt = w[0].sol.tip_wcs.lerp(w[1].sol.tip_wcs, frac);
                    builder.linear_to(pt, ctx.cutting.cutting_feed_mm_min, cut_role);
                    exit_tip = pt;
                    let t_interp = w[0].sample.tangent.lerp(w[1].sample.tangent, frac);
                    exit_tan_xy = DVec2::new(t_interp.x, t_interp.y).normalize_or_zero();
                    exit_out_xy = w[0]
                        .sol
                        .outward_xy
                        .lerp(w[1].sol.outward_xy, frac)
                        .normalize_or_zero();
                    break;
                }
                builder.linear_to(w[1].sol.tip_wcs, ctx.cutting.cutting_feed_mm_min, cut_role);
                exit_tip = w[1].sol.tip_wcs;
                exit_tan_xy =
                    DVec2::new(w[1].sample.tangent.x, w[1].sample.tangent.y).normalize_or_zero();
                exit_out_xy = w[1].sol.outward_xy;
                acc += seg_len;
            }
        }
    }

    // Resolve safe tangent lead-out arc + safe vertical retract column at exact `exit_tip`!
    let (retract_pt, lead_out_arc, linear_depart_pt) = plan_safe_lead(
        exit_tip,
        exit_tan_xy,
        exit_out_xy,
        false,
        sub.is_underside,
        pref_lead_r,
        retract_z,
        envelope,
        stickout_mm,
        target_mesh,
        wall_clearance,
        cand_buf,
    );

    // 5. Smooth tangent lead-out arc + optional horizontal clearance departure (for overhangs)
    if let Some((center_xy, radius_mm, ccw)) = lead_out_arc
        && let Some(end_arc) = linear_depart_pt.or(Some(retract_pt))
    {
        builder.arc_xy_to(XyArcMove {
            end_wcs: end_arc,
            center_xy,
            radius_mm,
            ccw,
            turns: 0,
            feed_mm_min: ctx.cutting.lead_feed_mm_min,
            role: MotionRole::LeadOut,
        });
    }
    if builder
        .current_pos()
        .is_some_and(|c| c.distance(retract_pt) > 1e-4)
    {
        builder.linear_to(
            retract_pt,
            ctx.cutting.lead_feed_mm_min,
            MotionRole::LeadOut,
        );
    }

    // 6. Safe vertical rapid retract up to `retract_z`
    builder.rapid_to(DVec3::new(retract_pt.x, retract_pt.y, retract_z));
    true
}

/// Rolls smoothly around a sharp outer convex corner vertex shared by `prev_st` and `next_st`
/// (interpolating the radial offset vector around the corner in `<= 5°` angular increments so
/// straight chords never clip the solid corner or rub a relieved lollipop neck!).
fn emit_corner_roll(
    prev_st: &SolvedStation,
    next_st: &SolvedStation,
    feed_mm_min: f64,
    role: MotionRole,
    builder: &mut ToolpathBuilder,
) {
    let corner_xy = 0.5
        * (DVec2::new(prev_st.sample.pos_wcs.x, prev_st.sample.pos_wcs.y)
            + DVec2::new(next_st.sample.pos_wcs.x, next_st.sample.pos_wcs.y));
    let v0 = DVec2::new(prev_st.sol.tip_wcs.x, prev_st.sol.tip_wcs.y) - corner_xy;
    let v1 = DVec2::new(next_st.sol.tip_wcs.x, next_st.sol.tip_wcs.y) - corner_xy;
    let r0 = v0.length();
    let r1 = v1.length();
    if r0 < 1e-4 || r1 < 1e-4 {
        builder.linear_to(next_st.sol.tip_wcs, feed_mm_min, role);
        return;
    }
    let a0 = v0.y.atan2(v0.x);
    let u0 = v0 / r0;
    let u1 = v1 / r1;
    let d_ang = (u0.x * u1.y - u0.y * u1.x).atan2(u0.dot(u1));
    let steps = (d_ang.abs() / 5.0_f64.to_radians()).ceil().clamp(2.0, 36.0) as usize;
    let z0 = prev_st.sol.tip_wcs.z;
    let z1 = next_st.sol.tip_wcs.z;

    for k in 1..=steps {
        let frac = (k as f64) / (steps as f64);
        let pt = if k == steps {
            next_st.sol.tip_wcs
        } else {
            let ang = a0 + d_ang * frac;
            let r = r0 + (r1 - r0) * frac;
            let (s, c) = ang.sin_cos();
            DVec3::new(
                corner_xy.x + r * c,
                corner_xy.y + r * s,
                z0 + (z1 - z0) * frac,
            )
        };
        if builder
            .current_pos()
            .is_none_or(|cur| cur.distance(pt) > 1e-6)
        {
            builder.linear_to(pt, feed_mm_min, role);
        }
    }
}

/// Plans a collision-free tangent arc lead-in (`is_lead_in = true`) or lead-out
/// (`is_lead_in = false`) plus a verified collision-free vertical plunge/retract column.
///
/// Returns `(vertical_column_pt, Option<(arc_center_xy, radius, ccw)>, Option<arc_outer_pt>)`.
#[allow(clippy::too_many_arguments)]
fn plan_safe_lead(
    edge_tip: DVec3,
    tan_xy: DVec2,
    out_xy: DVec2,
    is_lead_in: bool,
    is_underside: bool,
    pref_radius: f64,
    retract_z: f64,
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    target_mesh: &TriMesh3D,
    wall_clearance: f64,
    cand_buf: &mut Vec<usize>,
) -> (DVec3, Option<(DVec2, f64, bool)>, Option<DVec3>) {
    let tip_xy = DVec2::new(edge_tip.x, edge_tip.y);
    let radii = if pref_radius > 0.1 {
        [pref_radius, pref_radius * 0.5, pref_radius * 0.25]
    } else {
        [0.0, 0.0, 0.0]
    };

    // Helper: push a point horizontally along `out_xy` until the vertical plunge/retract column
    // clears any overhang roof / neighbor wall completely!
    let find_clear_vertical = |start_pt: DVec3, cand_buf: &mut Vec<usize>| -> Option<DVec3> {
        let extra_offsets: &[f64] = if is_underside {
            &[0.0, 1.0, 2.0, 3.5, 5.0, 8.0, 12.0]
        } else {
            &[0.0, 0.5, 1.5, 3.0, 6.0]
        };
        for &off in extra_offsets {
            let p = DVec3::new(
                start_pt.x + out_xy.x * off,
                start_pt.y + out_xy.y * off,
                start_pt.z,
            );
            if is_vertical_column_clear(
                p,
                retract_z,
                envelope,
                stickout_mm,
                target_mesh,
                wall_clearance,
                cand_buf,
            ) {
                return Some(p);
            }
        }
        None
    };

    if tan_xy.length_squared() > 0.5 {
        for &r in &radii {
            if r < 0.15 {
                continue;
            }
            let center_xy = tip_xy + out_xy * r;
            // Quarter-circle (90°) tangent arc on the `out_xy` side of the edge
            let arc_outer_xy = if is_lead_in {
                center_xy - tan_xy * r
            } else {
                center_xy + tan_xy * r
            };
            let arc_outer_pt = DVec3::new(arc_outer_xy.x, arc_outer_xy.y, edge_tip.z);

            // Arc midpoint (45° along the quarter-circle)
            let mid_dir = if is_lead_in {
                (-out_xy - tan_xy).normalize_or_zero()
            } else {
                (-out_xy + tan_xy).normalize_or_zero()
            };
            let arc_mid_xy = center_xy + mid_dir * r;
            let arc_mid_pt = DVec3::new(arc_mid_xy.x, arc_mid_xy.y, edge_tip.z);

            let arc_ok = is_air_position_clear(
                arc_outer_pt,
                envelope,
                stickout_mm,
                target_mesh,
                wall_clearance,
                cand_buf,
            ) && is_air_position_clear(
                arc_mid_pt,
                envelope,
                stickout_mm,
                target_mesh,
                wall_clearance,
                cand_buf,
            );
            if !arc_ok {
                continue;
            }

            if let Some(col_pt) = find_clear_vertical(arc_outer_pt, cand_buf) {
                // Determine exact XY arc orientation (`ccw`):
                // For lead-in: vector from center to `edge_tip` is `-out_xy`, arriving tangent is `tan_xy`
                // Cross `(-out_xy) × tan_xy > 0` => CCW!
                // For lead-out: leaving `edge_tip` with tangent `tan_xy`, radius `-out_xy` => same cross!
                let cross = (-out_xy.x) * tan_xy.y - (-out_xy.y) * tan_xy.x;
                let ccw = cross > 0.0;
                return (col_pt, Some((center_xy, r, ccw)), Some(arc_outer_pt));
            }
        }
    }

    // Fallback: straight outward clearance approach along `out_xy`
    let base_out = if is_underside {
        envelope.cutting_radius_mm() + wall_clearance + 1.0
    } else {
        wall_clearance + 0.5
    };
    let start_out = DVec3::new(
        edge_tip.x + out_xy.x * base_out,
        edge_tip.y + out_xy.y * base_out,
        edge_tip.z,
    );
    if let Some(col_pt) = find_clear_vertical(start_out, cand_buf) {
        return (col_pt, None, None);
    }
    if let Some(col_pt) = find_clear_vertical(edge_tip, cand_buf) {
        return (col_pt, None, None);
    }

    (edge_tip, None, None)
}
