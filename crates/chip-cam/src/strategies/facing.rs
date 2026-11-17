use chip_math::{Aabb2, DVec2, DVec3};

use crate::{
    cl_data::MotionRole,
    params::{FacingCutMode, FacingParams},
    selection::ExtractedFeature25D,
    strategies::{StrategyExecContext, compute_z_levels},
    toolpath::ToolpathBuilder,
    wcs::CamError,
};

/// Generates a 2.5D **Facing** toolpath (`05-operations.md` §1).
pub fn generate_facing(
    ctx: &StrategyExecContext<'_>,
    params: &FacingParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    let tool_d = ctx.tool.geometry.cutting_diameter_mm();
    let tool_r = 0.5 * tool_d;

    let stepover = params.stepover.value_mm;
    if !stepover.is_finite() || stepover <= 0.02 || stepover > tool_d {
        return Err(CamError::InvalidParameter {
            param: "facing.stepover",
            value: stepover,
            reason: "must be > 0.02 mm and <= tool diameter",
        });
    }

    let pass_ext = params.pass_extension.value_mm.max(0.05 * tool_d);

    // Determine XY footprint: union of selected planar faces if any, otherwise full Setup Stock XY AABB!
    let mut footprint = Aabb2::EMPTY;
    for feat in &ctx.selection.features {
        if let ExtractedFeature25D::PlanarFace {
            outer_boundary_wcs, ..
        } = feat
        {
            footprint = footprint.union(&outer_boundary_wcs.exact_aabb());
        } else if let ExtractedFeature25D::Contour { polyline_wcs, .. } = feat {
            footprint = footprint.union(&polyline_wcs.exact_aabb());
        }
    }
    if !footprint.is_valid() {
        let s3 = ctx.setup.stock.aabb_wcs();
        footprint = Aabb2 {
            min: DVec2::new(s3.min.x, s3.min.y),
            max: DVec2::new(s3.max.x, s3.max.y),
        };
    }

    let fp_size = footprint.size();
    let angle_rad = if params.auto_longest_axis {
        if fp_size.y > fp_size.x {
            std::f64::consts::FRAC_PI_2
        } else {
            0.0
        }
    } else {
        params.pass_direction_deg.to_radians()
    };

    let u_axis = DVec2::new(angle_rad.cos(), angle_rad.sin());
    let v_axis = DVec2::new(-angle_rad.sin(), angle_rad.cos());

    // Project the 4 corners of `footprint` onto `(u_axis, v_axis)`
    let corners = [
        footprint.min,
        DVec2::new(footprint.max.x, footprint.min.y),
        footprint.max,
        DVec2::new(footprint.min.x, footprint.max.y),
    ];
    let mut u_min = f64::INFINITY;
    let mut u_max = f64::NEG_INFINITY;
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    for c in corners {
        let pu = c.dot(u_axis);
        let pv = c.dot(v_axis);
        u_min = u_min.min(pu);
        u_max = u_max.max(pu);
        v_min = v_min.min(pv);
        v_max = v_max.max(pv);
    }

    // Extend along `u` by `tool_r + pass_ext` so the full cutter body starts & ends in air outside stock!
    let u_start_ext = u_min - (tool_r + pass_ext);
    let u_end_ext = u_max + (tool_r + pass_ext);

    // Inset lateral span slightly (`v_min + 0.35 * tool_r .. v_max - 0.35 * tool_r`) while guaranteeing
    // full coverage within `tool_r` of both `v_min` and `v_max`!
    let span_v = (v_max - v_min).max(0.0);
    let pass_vs: Vec<f64> = if span_v <= 2.0 * (tool_r - 0.05).max(0.0) {
        vec![0.5 * (v_min + v_max)]
    } else {
        let first_v = v_min + (tool_r - 0.25 * (tool_d - stepover)).clamp(0.0, 0.85 * tool_r);
        let last_v = v_max - (tool_r - 0.25 * (tool_d - stepover)).clamp(0.0, 0.85 * tool_r);
        if last_v <= first_v + 1e-6 {
            vec![0.5 * (v_min + v_max)]
        } else {
            let count = ((last_v - first_v) / stepover).ceil().max(1.0) as usize;
            let actual_step = (last_v - first_v) / (count as f64);
            (0..=count)
                .map(|i| first_v + actual_step * (i as f64))
                .collect()
        }
    };

    // Compute Z levels (with optional dedicated finishing stepdown)
    let target_bottom_z = ctx.heights.bottom_z_wcs + ctx.passes.stock_to_leave_axial.value_mm;
    let z_levels = if let Some(fin_spec) = &params.finishing_stepdown
        && fin_spec.value_mm > 1e-4
        && ctx.heights.top_z_wcs - target_bottom_z > fin_spec.value_mm + 1e-4
    {
        let rough_bottom = target_bottom_z + fin_spec.value_mm;
        let mut lvls = compute_z_levels(
            ctx.heights.top_z_wcs,
            rough_bottom,
            ctx.passes.multiple_depths,
            ctx.passes.max_stepdown.value_mm,
            ctx.passes.even_stepdowns,
        )?;
        lvls.push(target_bottom_z);
        lvls
    } else {
        compute_z_levels(
            ctx.heights.top_z_wcs,
            target_bottom_z,
            ctx.passes.multiple_depths,
            ctx.passes.max_stepdown.value_mm,
            ctx.passes.even_stepdowns,
        )?
    };

    let to_xy = |u: f64, v: f64| -> DVec2 { u_axis * u + v_axis * v };
    let retract_z = ctx.safe_retract_z();
    let feed_z = ctx.heights.feed_z_wcs;

    for (lvl_idx, &z_cut) in z_levels.iter().enumerate() {
        ctx.check_cancel()?;
        let is_finish_z = lvl_idx + 1 == z_levels.len() && params.finishing_stepdown.is_some();
        let cut_role = if is_finish_z {
            MotionRole::FinishCut
        } else {
            MotionRole::Cut
        };

        // Order `pass_vs` so OneWayClimb vs OneWayConventional keeps the uncut stock on the climb/conv side
        let ordered_vs: Vec<f64> = match params.cut_mode {
            FacingCutMode::OneWayClimb => {
                // Stepping from high `v` down to low `v` while cutting along `+u` puts uncut stock on the RIGHT (Climb for CW spindle!)
                let mut rev = pass_vs.clone();
                rev.reverse();
                rev
            }
            FacingCutMode::BothWaysZigZag | FacingCutMode::OneWayConventional => pass_vs.clone(),
        };

        for (i, &pv) in ordered_vs.iter().enumerate() {
            ctx.check_cancel()?;
            builder.pass_count += 1;

            let forward = match params.cut_mode {
                FacingCutMode::BothWaysZigZag => i % 2 == 0,
                FacingCutMode::OneWayClimb | FacingCutMode::OneWayConventional => true,
            };
            let (u0, u1) = if forward {
                (u_start_ext, u_end_ext)
            } else {
                (u_end_ext, u_start_ext)
            };
            let p_start = to_xy(u0, pv);
            let p_end = to_xy(u1, pv);

            if i == 0 || params.cut_mode != FacingCutMode::BothWaysZigZag {
                if builder.current_pos().is_none() {
                    builder.rapid_to(DVec3::new(
                        p_start.x,
                        p_start.y,
                        ctx.heights.clearance_z_wcs,
                    ));
                } else {
                    if let Some(cur) = builder.current_pos()
                        && cur.z < retract_z - 1e-6
                    {
                        builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
                    }
                    builder.rapid_to(DVec3::new(p_start.x, p_start.y, retract_z));
                }
                builder.rapid_to(DVec3::new(p_start.x, p_start.y, feed_z));
                builder.linear_to(
                    DVec3::new(p_start.x, p_start.y, z_cut),
                    ctx.cutting.plunge_feed_mm_min,
                    MotionRole::Plunge,
                );
            }

            builder.linear_to(
                DVec3::new(p_end.x, p_end.y, z_cut),
                ctx.cutting.cutting_feed_mm_min,
                cut_role,
            );

            // Smooth U-turn `G2`/`G3` linking arc outside the stock in BothWaysZigZag mode!
            if params.cut_mode == FacingCutMode::BothWaysZigZag
                && let Some(&next_pv) = ordered_vs.get(i + 1)
            {
                let p_next_start = to_xy(u1, next_pv);
                let mid_center = 0.5 * (p_end + p_next_start);
                let link_r = 0.5 * p_end.distance(p_next_start);
                // Turning from `+u` toward `+v` (`next_pv > pv`) is CCW; turning from `-u` toward `+v` is CW!
                let ccw = if forward { next_pv > pv } else { next_pv < pv };
                if link_r > 1e-4 {
                    builder.arc_xy_to(
                        DVec3::new(p_next_start.x, p_next_start.y, z_cut),
                        mid_center,
                        link_r,
                        ccw,
                        0,
                        ctx.cutting.lead_feed_mm_min,
                        MotionRole::LinkTransition,
                    );
                } else {
                    builder.linear_to(
                        DVec3::new(p_next_start.x, p_next_start.y, z_cut),
                        ctx.cutting.lead_feed_mm_min,
                        MotionRole::LinkTransition,
                    );
                }
            }
        }

        // Retract after completing Z level
        if let Some(cur) = builder.current_pos() {
            builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
        }
    }

    Ok(())
}
