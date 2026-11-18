use chip_math::{ArcPolyline2D, DVec2, DVec3};
use chip_tools::CutterGeometry;

use crate::{
    cl_data::MotionRole,
    offset2d::offset_polyline,
    params::{ChamferSourceMode, ChamferTraceMode, ChamferTraceParams, ContourCompensation},
    selection::ExtractedFeature25D,
    strategies::{StrategyExecContext, compute_z_levels},
    toolpath::{ToolpathBuilder, XyArcMove},
    wcs::CamError,
};

/// Generates an **Edge Chamfering** or **2D Curve Trace / Engraving** toolpath (`05-operations.md` §5).
pub fn generate_chamfer_trace(
    ctx: &StrategyExecContext<'_>,
    params: &ChamferTraceParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    match params.mode {
        ChamferTraceMode::EdgeChamfer => generate_edge_chamfer(ctx, params, builder),
        ChamferTraceMode::CurveTrace => generate_curve_trace(ctx, params, builder),
    }
}

fn generate_edge_chamfer(
    ctx: &StrategyExecContext<'_>,
    params: &ChamferTraceParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    // Extract chamfer cutter cone geometry (`included_angle = 2θ`, `tip_diameter`, `outer_diameter`)
    let (outer_dia_mm, tip_dia_mm, included_deg) = match ctx.tool.geometry {
        CutterGeometry::ChamferMill {
            diameter_mm,
            tip_diameter_mm,
            included_angle_deg,
            ..
        } => (diameter_mm, tip_diameter_mm, included_angle_deg),
        CutterGeometry::TwistDrill {
            diameter_mm,
            point_angle_deg,
            ..
        } => (diameter_mm, 0.0, point_angle_deg),
        _ => {
            return Err(CamError::InvalidChamferGeometry {
                reason: format!(
                    "Edge Chamfer requires a Chamfer Mill or angled Spot/Twist Drill, got {}",
                    ctx.tool.geometry.kind_label()
                ),
            });
        }
    };

    if !(10.0..178.0).contains(&included_deg) {
        return Err(CamError::InvalidChamferGeometry {
            reason: format!("invalid included cutter angle {included_deg:.1}°"),
        });
    }
    let theta_rad = 0.5 * included_deg.to_radians();
    let d_tip = params.tip_offset.value_mm;
    if !d_tip.is_finite() || d_tip < 0.0 {
        return Err(CamError::InvalidParameter {
            param: "chamfer.tip_offset",
            value: d_tip,
            reason: "must be >= 0.0 mm",
        });
    }
    let max_flute_radial = 0.5 * (outer_dia_mm - tip_dia_mm).max(0.0);
    let retract_z = ctx.safe_retract_z();

    // Collect chamfer targets from either `ModeledChamfer` faces or sharp `Contour` / `PlanarFace` edges!
    struct ChamferJob {
        upper_poly_wcs: ArcPolyline2D,
        offset_sign: f64,
        chamfer_width_mm: f64,
        top_z_wcs: f64,
        is_modeled_upper_edge: bool,
        adjacent_lower_floor_z_wcs: Option<f64>,
    }

    let mut jobs: Vec<ChamferJob> = Vec::new();

    for feat in &ctx.selection.features {
        match feat {
            ExtractedFeature25D::ModeledChamfer {
                upper_polyline_wcs,
                lower_polyline_wcs,
                radial_width_mm,
                bevel_half_angle_rad,
                top_z_wcs,
                outward_normal_xy_wcs,
                adjacent_lower_floor_z_wcs,
                ..
            } => {
                if (bevel_half_angle_rad - theta_rad).abs() > 2.5_f64.to_radians() {
                    builder.warnings.push(format!(
                        "Modeled chamfer angle ({:.1}°) differs from cutter half-angle ({:.1}°).",
                        bevel_half_angle_rad.to_degrees(),
                        theta_rad.to_degrees()
                    ));
                }

                // Determine outward bevel direction relative to `upper_polyline_wcs`!
                // If `lower_polyline_wcs` has larger signed area / lies to the RIGHT of `upper_polyline_wcs`,
                // the outward air side is `-1` (right); if `upper` is CCW around a boss, right is outward!
                let sign = determine_modeled_chamfer_offset_sign(
                    *outward_normal_xy_wcs,
                    upper_polyline_wcs,
                    lower_polyline_wcs,
                );
                if !upper_polyline_wcs.segments.is_empty() {
                    jobs.push(ChamferJob {
                        upper_poly_wcs: upper_polyline_wcs.clone(),
                        offset_sign: sign,
                        chamfer_width_mm: *radial_width_mm,
                        top_z_wcs: *top_z_wcs,
                        is_modeled_upper_edge: true,
                        adjacent_lower_floor_z_wcs: *adjacent_lower_floor_z_wcs,
                    });
                }
            }
            ExtractedFeature25D::Contour {
                polyline_wcs,
                max_z_wcs,
                side_flip,
                adjacent_lower_floor_z_wcs,
                ..
            } => {
                let w = match &params.chamfer_source {
                    ChamferSourceMode::SharpEdge { width } => width.value_mm,
                    ChamferSourceMode::ModeledFace => 0.5,
                };
                if w <= 1e-4 {
                    return Err(CamError::InvalidParameter {
                        param: "chamfer.width",
                        value: w,
                        reason: "must be > 0 mm",
                    });
                }
                let sign = if *side_flip { -1.0 } else { 1.0 };
                jobs.push(ChamferJob {
                    upper_poly_wcs: polyline_wcs.clone(),
                    offset_sign: sign,
                    chamfer_width_mm: w,
                    top_z_wcs: *max_z_wcs,
                    is_modeled_upper_edge: false,
                    adjacent_lower_floor_z_wcs: *adjacent_lower_floor_z_wcs,
                });
            }
            ExtractedFeature25D::PlanarFace {
                outer_boundary_wcs,
                face_z_wcs,
                adjacent_lower_floor_z_wcs,
                ..
            } => {
                let w = match &params.chamfer_source {
                    ChamferSourceMode::SharpEdge { width } => width.value_mm,
                    ChamferSourceMode::ModeledFace => 0.5,
                };
                // CCW outer boundary of a top face has solid inside (left) and air outside (right -> `-1.0`, or reverse to CW + `+1.0` for Climb!)
                let cw_outer = outer_boundary_wcs.ensure_orientation(false);
                jobs.push(ChamferJob {
                    upper_poly_wcs: cw_outer,
                    offset_sign: 1.0,
                    chamfer_width_mm: w.max(0.05),
                    top_z_wcs: *face_z_wcs,
                    is_modeled_upper_edge: false,
                    adjacent_lower_floor_z_wcs: *adjacent_lower_floor_z_wcs,
                });
            }
            ExtractedFeature25D::CylindricalHole { .. } => {}
        }
    }

    if jobs.is_empty() {
        return Err(CamError::EmptySelection {
            op_name: ctx.op.name.clone(),
            op_kind: ctx.op.kind.label(),
        });
    }

    for job in &jobs {
        ctx.check_cancel()?;
        let w = job.chamfer_width_mm;

        // Exact chamfer kinematics (`05-operations.md` §5A):
        //   axial_drop = w / tan(θ) + d_tip
        //   - For an unmodeled Sharp Edge (whose XY coincides with the vertical wall / lower chamfer edge):
        //       radial_offset = (tip_diameter / 2) + d_tip * tan(θ)
        //   - For a Modeled Chamfer Face (where `upper_poly_wcs` is set back inward by `w` from the lower edge):
        //       radial_offset = w + (tip_diameter / 2) + d_tip * tan(θ)
        let axial_drop = w / theta_rad.tan() + d_tip;
        let radial_offset = if job.is_modeled_upper_edge {
            w + 0.5 * tip_dia_mm + d_tip * theta_rad.tan()
        } else {
            0.5 * tip_dia_mm + d_tip * theta_rad.tan()
        };

        if d_tip * theta_rad.tan() + w > max_flute_radial + 1e-3 {
            return Err(CamError::InvalidChamferGeometry {
                reason: format!(
                    "chamfer width ({w:.2} mm) + tip_offset radial shift ({:.2} mm) exceeds cutter cone radial span ({max_flute_radial:.2} mm)",
                    d_tip * theta_rad.tan()
                ),
            });
        }

        let tip_z_wcs = job.top_z_wcs - axial_drop;

        // Adjacent lower step floor collision guard (`05-operations.md` §5A)!
        if params.check_adjacent_walls
            && let Some(floor_z_wcs) = job.adjacent_lower_floor_z_wcs
            && tip_z_wcs < floor_z_wcs - 1e-3
        {
            return Err(CamError::ChamferTipCollision {
                tip_z_wcs,
                floor_z_wcs,
            });
        }

        let offset_paths = offset_polyline(&job.upper_poly_wcs, job.offset_sign * radial_offset);
        for path in offset_paths {
            let Some((start_xy, start_tan)) = path.sample_at_fraction(0.0) else {
                continue;
            };
            builder.pass_count += 1;

            // Tangent horizontal lead-in if enabled
            let t = start_tan.normalize_or_zero();
            let n_safe = if job.offset_sign >= 0.0 {
                DVec2::new(-t.y, t.x)
            } else {
                DVec2::new(t.y, -t.x)
            };
            let lead_r = if ctx.linking.lead_in.enabled {
                ctx.linking
                    .lead_in
                    .horizontal_radius
                    .value_mm
                    .clamp(0.0, 5.0)
            } else {
                0.0
            };

            let approach_xy = if lead_r > 1e-4 {
                start_xy + n_safe * lead_r - t * lead_r
            } else {
                start_xy
            };

            if let Some(cur) = builder.current_pos()
                && cur.z < retract_z - 1e-6
            {
                builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
            }
            builder.rapid_to(DVec3::new(approach_xy.x, approach_xy.y, retract_z));
            builder.rapid_to(DVec3::new(
                approach_xy.x,
                approach_xy.y,
                ctx.heights.feed_z_wcs,
            ));
            builder.linear_to(
                DVec3::new(approach_xy.x, approach_xy.y, tip_z_wcs),
                ctx.cutting.plunge_feed_mm_min,
                MotionRole::Plunge,
            );

            if lead_r > 1e-4 {
                let c_in = start_xy + n_safe * lead_r;
                builder.arc_xy_to(XyArcMove {
                    end_wcs: DVec3::new(start_xy.x, start_xy.y, tip_z_wcs),
                    center_xy: c_in,
                    radius_mm: lead_r,
                    ccw: job.offset_sign >= 0.0,
                    turns: 0,
                    feed_mm_min: ctx.cutting.lead_feed_mm_min,
                    role: MotionRole::LeadIn,
                });
            }

            builder.emit_polyline_2d_z_interp(
                &path,
                tip_z_wcs,
                tip_z_wcs,
                ctx.cutting.cutting_feed_mm_min,
                MotionRole::Cut,
            );

            if path.closed {
                let ov = ctx
                    .linking
                    .seam_overlap
                    .value_mm
                    .clamp(0.0, 0.2 * path.perimeter());
                if ov > 1e-4 {
                    let sub = path.slice_interval(0.0, ov);
                    builder.emit_polyline_2d_z_interp(
                        &sub,
                        tip_z_wcs,
                        tip_z_wcs,
                        ctx.cutting.cutting_feed_mm_min,
                        MotionRole::Cut,
                    );
                }
            }

            if let Some(cur) = builder.current_pos() {
                builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
            }
        }
    }

    Ok(())
}

fn determine_modeled_chamfer_offset_sign(
    outward_normal_xy_wcs: Option<DVec2>,
    upper_poly_wcs: &ArcPolyline2D,
    lower_poly_wcs: &ArcPolyline2D,
) -> f64 {
    // Use the modeled chamfer face's outward horizontal normal component in WCS!
    if let Some(n_xy) = outward_normal_xy_wcs
        && let Some((_, tan2)) = upper_poly_wcs.sample_at_fraction(0.5)
    {
        let out_xy = n_xy.normalize_or_zero();
        let left_n = DVec2::new(-tan2.y, tan2.x);
        if out_xy.dot(left_n) < 0.0 {
            return -1.0;
        }
        return 1.0;
    }
    if !lower_poly_wcs.segments.is_empty()
        && let Some((up_pt, tan2)) = upper_poly_wcs.sample_at_fraction(0.5)
        && let Some(proj) = lower_poly_wcs.project_point(up_pt)
    {
        let dir = proj.point - up_pt;
        let left_n = DVec2::new(-tan2.y, tan2.x);
        if dir.dot(left_n) < 0.0 {
            return -1.0;
        }
    }
    1.0
}

fn generate_curve_trace(
    ctx: &StrategyExecContext<'_>,
    params: &ChamferTraceParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    let tool_r = 0.5 * ctx.tool.geometry.cutting_diameter_mm();
    let radial_leave = ctx.passes.stock_to_leave_radial.value_mm;
    let target_bottom_z = ctx.heights.bottom_z_wcs - params.trace_axial_offset.value_mm
        + ctx.passes.stock_to_leave_axial.value_mm;

    let z_levels = compute_z_levels(
        ctx.heights.top_z_wcs,
        target_bottom_z,
        ctx.passes.multiple_depths,
        ctx.passes.max_stepdown.value_mm,
        ctx.passes.even_stepdowns,
    )?;

    let mut traces: Vec<(ArcPolyline2D, bool)> = Vec::new();
    for feat in &ctx.selection.features {
        match feat {
            ExtractedFeature25D::Contour {
                polyline_wcs,
                side_flip,
                ..
            } => traces.push((polyline_wcs.clone(), *side_flip)),
            ExtractedFeature25D::PlanarFace {
                outer_boundary_wcs, ..
            } => traces.push((outer_boundary_wcs.clone(), false)),
            _ => {}
        }
    }

    if traces.is_empty() {
        return Err(CamError::EmptySelection {
            op_name: ctx.op.name.clone(),
            op_kind: ctx.op.kind.label(),
        });
    }

    let retract_z = ctx.safe_retract_z();

    for (raw_poly, side_flip) in &traces {
        ctx.check_cancel()?;
        let signed_off = match params.trace_compensation {
            ContourCompensation::CenterOff => 0.0,
            ContourCompensation::LeftClimb => {
                (if *side_flip { -1.0 } else { 1.0 }) * (tool_r + radial_leave)
            }
            ContourCompensation::RightConventional => {
                (if *side_flip { 1.0 } else { -1.0 }) * (tool_r + radial_leave)
            }
        };

        let paths = offset_polyline(raw_poly, signed_off);
        for path in paths {
            let Some((s_xy, _)) = path.sample_at_fraction(0.0) else {
                continue;
            };
            for &z_cut in &z_levels {
                ctx.check_cancel()?;
                builder.pass_count += 1;
                if let Some(cur) = builder.current_pos()
                    && cur.z < retract_z - 1e-6
                {
                    builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
                }
                builder.rapid_to(DVec3::new(s_xy.x, s_xy.y, retract_z));
                builder.rapid_to(DVec3::new(s_xy.x, s_xy.y, ctx.heights.feed_z_wcs));
                builder.linear_to(
                    DVec3::new(s_xy.x, s_xy.y, z_cut),
                    ctx.cutting.plunge_feed_mm_min,
                    MotionRole::Plunge,
                );
                builder.emit_polyline_2d_z_interp(
                    &path,
                    z_cut,
                    z_cut,
                    ctx.cutting.cutting_feed_mm_min,
                    MotionRole::Cut,
                );
                if let Some(cur) = builder.current_pos() {
                    builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
                }
            }
        }
    }

    Ok(())
}
