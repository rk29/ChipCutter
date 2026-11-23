use chip_math::{Aabb2, ArcPolyline2D, ArcSegment2D, DVec2, DVec3};

use crate::{
    cl_data::MotionRole,
    params::{FacingCutMode, FacingParams},
    rest2d::{RegionSet2D, clip_polyline_outside_region_set},
    selection::ExtractedFeature25D,
    strategies::{StrategyExecContext, compute_z_levels},
    surface3d::{PushCutterField, SurfaceCutterProfile, SurfaceMesh3D, is_2d_arc_clear_of_avoid},
    toolpath::{ToolpathBuilder, XyArcMove},
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
    let eff_top_z = if let Some(td) = ctx.carryover_stock {
        let Some(max_z) = td.max_stock_z_in_xy_aabb(footprint) else {
            return Err(CamError::EmptyToolpath {
                op_name: ctx.op.name.clone(),
            });
        };
        if max_z <= target_bottom_z + 1e-3 {
            return Err(CamError::EmptyToolpath {
                op_name: ctx.op.name.clone(),
            });
        }
        ctx.heights.top_z_wcs.min(max_z)
    } else {
        ctx.heights.top_z_wcs
    };

    let z_levels = if let Some(fin_spec) = &params.finishing_stepdown
        && fin_spec.value_mm > 1e-4
        && eff_top_z - target_bottom_z > fin_spec.value_mm + 1e-4
    {
        let rough_bottom = target_bottom_z + fin_spec.value_mm;
        let mut lvls = compute_z_levels(
            eff_top_z,
            rough_bottom,
            ctx.passes.multiple_depths,
            ctx.passes.max_stepdown.value_mm,
            ctx.passes.even_stepdowns,
        )?;
        lvls.push(target_bottom_z);
        lvls
    } else {
        compute_z_levels(
            eff_top_z,
            target_bottom_z,
            ctx.passes.multiple_depths,
            ctx.passes.max_stepdown.value_mm,
            ctx.passes.even_stepdowns,
        )?
    };

    let to_xy = |u: f64, v: f64| -> DVec2 { u_axis * u + v_axis * v };
    let retract_z = ctx.safe_retract_z();
    let feed_z = ctx.heights.feed_z_wcs;

    let overhang_cutter = SurfaceCutterProfile::from_tool(
        ctx.tool,
        0.0,
        ctx.passes.stock_to_leave_axial.value_mm.max(0.0),
        0.5,
    );
    let overhang_surface = (ctx.passes.avoid_model_overhangs && ctx.doc.has_3d_geometry())
        .then(|| SurfaceMesh3D::from_cad_document(ctx.doc, &[], &ctx.setup.frame));
    let overhang_field = overhang_surface
        .as_ref()
        .filter(|s| !s.triangles.is_empty() && s.aabb_wcs.is_valid())
        .map(|s| PushCutterField::build_with_z_min(s, overhang_cutter, target_bottom_z));

    let build_pass_vs = |vmin: f64, vmax: f64| -> Vec<f64> {
        let sv = (vmax - vmin).max(0.0);
        if sv <= 2.0 * (tool_r - 0.05).max(0.0) {
            vec![0.5 * (vmin + vmax)]
        } else {
            let first_v = vmin + (tool_r - 0.25 * (tool_d - stepover)).clamp(0.0, 0.85 * tool_r);
            let last_v = vmax - (tool_r - 0.25 * (tool_d - stepover)).clamp(0.0, 0.85 * tool_r);
            if last_v <= first_v + 1e-6 {
                vec![0.5 * (vmin + vmax)]
            } else {
                let count = ((last_v - first_v) / stepover).ceil().max(1.0) as usize;
                let actual_step = (last_v - first_v) / (count as f64);
                (0..=count)
                    .map(|i| first_v + actual_step * (i as f64))
                    .collect()
            }
        }
    };

    for (lvl_idx, &z_cut) in z_levels.iter().enumerate() {
        ctx.check_cancel()?;
        let prev_z = if lvl_idx == 0 {
            eff_top_z
        } else {
            z_levels[lvl_idx - 1]
        };

        let (lvl_pass_vs, lvl_u_start_ext, lvl_u_end_ext, rem_polys_opt) = if let Some(td) =
            ctx.carryover_stock
        {
            let slab_lo = z_cut + 0.01;
            let slab_hi = prev_z.max(z_cut + 0.02) + 0.05;
            if !td.has_material_in_z_slab(slab_lo, slab_hi, 0.01) {
                continue;
            }
            let rem_polys = td.extract_remaining_material_regions_in_z_slab(slab_lo, slab_hi, 0.0);
            let mut ru_min = f64::INFINITY;
            let mut ru_max = f64::NEG_INFINITY;
            let mut rv_min = f64::INFINITY;
            let mut rv_max = f64::NEG_INFINITY;
            for poly in &rem_polys {
                for seg in &poly.segments {
                    let p = seg.start();
                    if p.x >= footprint.min.x - 1.0
                        && p.x <= footprint.max.x + 1.0
                        && p.y >= footprint.min.y - 1.0
                        && p.y <= footprint.max.y + 1.0
                    {
                        let pu = p.dot(u_axis);
                        let pv = p.dot(v_axis);
                        ru_min = ru_min.min(pu);
                        ru_max = ru_max.max(pu);
                        rv_min = rv_min.min(pv);
                        rv_max = rv_max.max(pv);
                    }
                }
            }
            if !(ru_min < ru_max && rv_min < rv_max) {
                continue;
            }
            let cu_min = ru_min.max(u_min);
            let cu_max = ru_max.min(u_max);
            let cv_min = rv_min.max(v_min);
            let cv_max = rv_max.min(v_max);
            if cu_max <= cu_min + 1e-4 || cv_max <= cv_min + 1e-4 {
                continue;
            }
            (
                build_pass_vs(cv_min, cv_max),
                cu_min - (tool_r + pass_ext),
                cu_max + (tool_r + pass_ext),
                Some(rem_polys),
            )
        } else {
            (pass_vs.clone(), u_start_ext, u_end_ext, None)
        };

        let is_finish_z = lvl_idx + 1 == z_levels.len() && params.finishing_stepdown.is_some();
        let cut_role = if is_finish_z {
            MotionRole::FinishCut
        } else {
            MotionRole::Cut
        };

        // Order `lvl_pass_vs` so OneWayClimb vs OneWayConventional keeps the uncut stock on the climb/conv side
        let mut ordered_vs: Vec<f64> = match params.cut_mode {
            FacingCutMode::OneWayClimb => {
                // Stepping from high `v` down to low `v` while cutting along `+u` puts uncut stock on the RIGHT (Climb for CW spindle!)
                let mut rev = lvl_pass_vs;
                rev.reverse();
                rev
            }
            FacingCutMode::BothWaysZigZag | FacingCutMode::OneWayConventional => lvl_pass_vs,
        };

        // Drop empty rows where carried-over stock has no material within `tool_r` of `pv`
        if let Some(ref rem_polys) = rem_polys_opt {
            let half_band = tool_r + 0.35;
            ordered_vs.retain(|&pv| {
                rem_polys.iter().any(|poly| {
                    let b = poly.exact_aabb();
                    let corners = [
                        b.min,
                        DVec2::new(b.max.x, b.min.y),
                        b.max,
                        DVec2::new(b.min.x, b.max.y),
                    ];
                    let p_vmin = corners
                        .iter()
                        .map(|c| c.dot(v_axis))
                        .fold(f64::INFINITY, f64::min);
                    let p_vmax = corners
                        .iter()
                        .map(|c| c.dot(v_axis))
                        .fold(f64::NEG_INFINITY, f64::max);
                    p_vmax >= pv - half_band && p_vmin <= pv + half_band
                })
            });
            if ordered_vs.is_empty() {
                continue;
            }
        }

        let overhang_avoid: Option<RegionSet2D> = overhang_field
            .as_ref()
            .map(|f| f.avoidance_region_z(z_cut))
            .filter(|r| !r.is_empty());

        for (i, &pv) in ordered_vs.iter().enumerate() {
            ctx.check_cancel()?;

            let forward = match params.cut_mode {
                FacingCutMode::BothWaysZigZag => i % 2 == 0,
                FacingCutMode::OneWayClimb | FacingCutMode::OneWayConventional => true,
            };
            let (u0, u1) = if forward {
                (lvl_u_start_ext, lvl_u_end_ext)
            } else {
                (lvl_u_end_ext, lvl_u_start_ext)
            };
            let p_start = to_xy(u0, pv);
            let p_end = to_xy(u1, pv);

            let row_spans: Vec<(DVec2, DVec2)> = if let Some(ref avoid) = overhang_avoid {
                let raw_line = ArcPolyline2D {
                    segments: vec![ArcSegment2D::Line {
                        start: p_start,
                        end: p_end,
                    }],
                    closed: false,
                };
                clip_polyline_outside_region_set(&raw_line, avoid)
                    .into_iter()
                    .filter_map(|poly| {
                        let s = poly.segments.first()?.start();
                        let e = poly.segments.last()?.end();
                        (s.distance(e) > 0.05).then_some((s, e))
                    })
                    .collect()
            } else {
                vec![(p_start, p_end)]
            };

            if row_spans.is_empty() {
                continue;
            }

            for (span_idx, &(span_start, span_end)) in row_spans.iter().enumerate() {
                builder.pass_count += 1;

                let can_zigzag_continue = i > 0
                    && span_idx == 0
                    && params.cut_mode == FacingCutMode::BothWaysZigZag
                    && builder.current_pos().is_some_and(|cur| {
                        (cur.z - z_cut).abs() <= 1e-4
                            && DVec2::new(cur.x, cur.y).distance(span_start) <= 1e-3
                    });

                if !can_zigzag_continue {
                    if builder.current_pos().is_none() {
                        builder.rapid_to(DVec3::new(
                            span_start.x,
                            span_start.y,
                            ctx.heights.clearance_z_wcs,
                        ));
                    } else {
                        if let Some(cur) = builder.current_pos()
                            && cur.z < retract_z - 1e-6
                        {
                            builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
                        }
                        builder.rapid_to(DVec3::new(span_start.x, span_start.y, retract_z));
                    }
                    builder.rapid_to(DVec3::new(span_start.x, span_start.y, feed_z));
                    builder.linear_to(
                        DVec3::new(span_start.x, span_start.y, z_cut),
                        ctx.cutting.plunge_feed_mm_min,
                        MotionRole::Plunge,
                    );
                }

                builder.linear_to(
                    DVec3::new(span_end.x, span_end.y, z_cut),
                    ctx.cutting.cutting_feed_mm_min,
                    cut_role,
                );

                // Smooth U-turn `G2`/`G3` linking arc outside the stock in BothWaysZigZag mode!
                let is_unclipped_end =
                    span_idx + 1 == row_spans.len() && span_end.distance(p_end) <= 1e-3;
                if is_unclipped_end
                    && params.cut_mode == FacingCutMode::BothWaysZigZag
                    && let Some(&next_pv) = ordered_vs.get(i + 1)
                    && (next_pv - pv).abs() <= stepover * 1.5 + 1e-4
                {
                    let p_next_start = to_xy(u1, next_pv);
                    let mid_center = 0.5 * (p_end + p_next_start);
                    let link_r = 0.5 * p_end.distance(p_next_start);
                    // Turning from `+u` toward `+v` (`next_pv > pv`) is CCW; turning from `-u` toward `+v` is CW!
                    let ccw = if forward { next_pv > pv } else { next_pv < pv };
                    let sweep = if ccw {
                        std::f64::consts::PI
                    } else {
                        -std::f64::consts::PI
                    };
                    let arc_clear = overhang_avoid.as_ref().is_none_or(|av| {
                        !av.contains_point(p_next_start)
                            && is_2d_arc_clear_of_avoid(p_end, mid_center, link_r, sweep, av)
                    });
                    if arc_clear {
                        if link_r > 1e-4 {
                            builder.arc_xy_to(XyArcMove {
                                end_wcs: DVec3::new(p_next_start.x, p_next_start.y, z_cut),
                                center_xy: mid_center,
                                radius_mm: link_r,
                                ccw,
                                turns: 0,
                                feed_mm_min: ctx.cutting.lead_feed_mm_min,
                                role: MotionRole::LinkTransition,
                            });
                        } else {
                            builder.linear_to(
                                DVec3::new(p_next_start.x, p_next_start.y, z_cut),
                                ctx.cutting.lead_feed_mm_min,
                                MotionRole::LinkTransition,
                            );
                        }
                    }
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
