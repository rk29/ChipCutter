use chip_math::{DVec2, DVec3};
use chip_tools::CutterGeometry;

use crate::{
    cl_data::{DrillCycleKind, MotionRole},
    operation::{ToolOperationClass, cutter_supports_operation},
    params::{DrillBoreParams, HoleMachiningMode},
    selection::ExtractedFeature25D,
    strategies::StrategyExecContext,
    surface3d::{SurfaceMesh3D, check_hole_column_overhang},
    toolpath::{DrillCycleSpec, ToolpathBuilder, XyArcMove},
    wcs::CamError,
};

#[derive(Debug, Clone, Copy)]
struct ResolvedHoleTarget {
    center_wcs: DVec2,
    diameter_mm: f64,
    top_z_wcs: f64,
    bottom_z_wcs: f64,
    is_through_hole: bool,
}

#[derive(Debug, Clone, Copy)]
struct HelicalBoreSpec {
    hole: ResolvedHoleTarget,
    hole_top_z: f64,
    hole_bottom_z: f64,
    r_plane_z: f64,
    retract_z: f64,
}

/// Generates a **Drilling** or **Helical Bore Milling** toolpath (`05-operations.md` §4).
pub fn generate_drill_bore(
    ctx: &StrategyExecContext<'_>,
    params: &DrillBoreParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    let mut holes: Vec<ResolvedHoleTarget> = Vec::new();
    for feat in &ctx.selection.features {
        if let ExtractedFeature25D::CylindricalHole {
            center_wcs,
            diameter_mm,
            top_z_wcs,
            bottom_z_wcs,
            is_through_hole,
            ..
        } = *feat
        {
            holes.push(ResolvedHoleTarget {
                center_wcs,
                diameter_mm,
                top_z_wcs,
                bottom_z_wcs,
                is_through_hole,
            });
        }
    }

    if holes.is_empty() {
        return Err(CamError::EmptySelection {
            op_name: ctx.op.name.clone(),
            op_kind: ctx.op.kind.label(),
        });
    }

    if params.optimize_hole_order && holes.len() > 2 {
        sort_holes_nearest_2opt(&mut holes);
    }

    let tool_d = ctx.tool.geometry.cutting_diameter_mm();
    let retract_z = ctx.safe_retract_z();
    let overhang_surface = (ctx.passes.avoid_model_overhangs && ctx.doc.has_3d_geometry())
        .then(|| SurfaceMesh3D::from_cad_document(ctx.doc, &[], &ctx.setup.frame));

    let z_top_shift = ctx.heights.top_z_wcs - ctx.selection.inferred_top_z_wcs;
    let z_bot_shift = ctx.heights.bottom_z_wcs - ctx.selection.inferred_bottom_z_wcs;
    let min_hole_bottom_z = holes
        .iter()
        .map(|h| (h.bottom_z_wcs + z_bot_shift) + ctx.passes.stock_to_leave_axial.value_mm)
        .fold(f64::INFINITY, f64::min);
    let max_hole_top_z = holes
        .iter()
        .map(|h| h.top_z_wcs + z_top_shift)
        .fold(f64::NEG_INFINITY, f64::max);
    let avoid_2d = ctx.resolve_avoid_2d(min_hole_bottom_z, max_hole_top_z);
    builder.warnings.extend(avoid_2d.warnings);
    let avoid_keepout = avoid_2d.effective_region;
    let mut skipped_avoid_holes = 0usize;

    for (idx, hole) in holes.iter().enumerate() {
        ctx.check_cancel()?;

        // Allow per-hole top/bottom when FromSelection is active, while honoring explicit user offsets
        let hole_top_z = hole.top_z_wcs + z_top_shift;
        let hole_bottom_z =
            (hole.bottom_z_wcs + z_bot_shift) + ctx.passes.stock_to_leave_axial.value_mm;
        let r_plane_z =
            (hole_top_z + (ctx.heights.feed_z_wcs - ctx.heights.top_z_wcs).max(1.0)).min(retract_z);

        let use_bore = match params.mode {
            HoleMachiningMode::Drill => false,
            HoleMachiningMode::HelicalBore => true,
            HoleMachiningMode::AutoByTool => {
                let can_bore =
                    cutter_supports_operation(&ctx.tool.geometry, ToolOperationClass::HelicalBore);
                let is_drill_geom = matches!(
                    ctx.tool.geometry,
                    CutterGeometry::TwistDrill { .. } | CutterGeometry::ChamferMill { .. }
                );
                can_bore && !is_drill_geom && tool_d < hole.diameter_mm - 0.05
            }
        };

        if !avoid_keepout.is_empty() {
            let orbit_r = if use_bore {
                (0.5 * hole.diameter_mm - 0.5 * tool_d - ctx.passes.stock_to_leave_radial.value_mm)
                    .max(0.0)
            } else {
                0.0
            };
            let blocked_by_avoid = avoid_keepout.contains_point(hole.center_wcs)
                || (orbit_r > 0.05
                    && avoid_keepout.min_distance_to_boundary(hole.center_wcs) < orbit_r);
            if blocked_by_avoid {
                skipped_avoid_holes += 1;
                continue;
            }
        }

        if let Some(ref surf) = overhang_surface {
            let orbit_r = if use_bore {
                (0.5 * hole.diameter_mm - 0.5 * tool_d - ctx.passes.stock_to_leave_radial.value_mm)
                    .max(0.0)
            } else {
                0.0
            };
            if let Some(blocking_z) =
                check_hole_column_overhang(surf, ctx.tool, hole.center_wcs, orbit_r, hole_bottom_z)
            {
                builder.warnings.push(format!(
                    "Skipped hole #{} (Ø{:.2} mm at WCS ({:.2}, {:.2})): vertical plunge column occluded by overhanging 3D geometry up to Z={:.2} mm.",
                    idx + 1,
                    hole.diameter_mm,
                    hole.center_wcs.x,
                    hole.center_wcs.y,
                    blocking_z
                ));
                continue;
            }
        }

        if builder.current_pos().is_none() {
            builder.rapid_to(DVec3::new(
                hole.center_wcs.x,
                hole.center_wcs.y,
                ctx.heights.clearance_z_wcs,
            ));
        }

        if use_bore {
            emit_helical_bore(
                ctx,
                params,
                builder,
                HelicalBoreSpec {
                    hole: *hole,
                    hole_top_z,
                    hole_bottom_z,
                    r_plane_z,
                    retract_z,
                },
            )?;
        } else {
            // Validate tool diameter against hole diameter unless spot-drilling / chamfering mouth!
            let is_spot = params.cycle_kind == DrillCycleKind::Spot
                || matches!(ctx.tool.geometry, CutterGeometry::ChamferMill { .. });
            if !is_spot && tool_d > hole.diameter_mm + 0.05 {
                return Err(CamError::HoleSmallerThanTool {
                    hole_diameter_mm: hole.diameter_mm,
                    tool_diameter_mm: tool_d,
                });
            }

            let mut target_z = hole_bottom_z;
            if params.tip_breakthrough && hole.is_through_hole && !is_spot {
                let point_angle_deg = match ctx.tool.geometry {
                    CutterGeometry::TwistDrill {
                        point_angle_deg, ..
                    } => point_angle_deg,
                    CutterGeometry::ChamferMill {
                        included_angle_deg, ..
                    } => included_angle_deg,
                    _ => 180.0,
                };
                if (10.0..179.0).contains(&point_angle_deg) {
                    let half_rad = 0.5 * point_angle_deg.to_radians();
                    let tip_cone = (0.5 * tool_d) / half_rad.tan();
                    target_z -= tip_cone + params.breakthrough_extra.value_mm.max(0.0);
                }
            }

            builder.pass_count += 1;
            builder.emit_drill_cycle(DrillCycleSpec {
                kind: params.cycle_kind,
                bottom_wcs: DVec3::new(hole.center_wcs.x, hole.center_wcs.y, target_z),
                r_plane_z_wcs: r_plane_z,
                clear_z_wcs: retract_z,
                peck_q_mm: params.peck_depth.value_mm.max(0.2),
                decreasing_factor: params.decreasing_peck_factor,
                min_peck_mm: params.min_peck_depth.value_mm.max(0.1),
                chip_break_retract_mm: params.chip_break_retract.value_mm.max(0.1),
                dwell_sec: params.dwell_seconds.max(0.0),
                feed_mm_min: ctx.cutting.plunge_feed_mm_min,
            });
        }
    }

    if skipped_avoid_holes > 0 {
        builder.warnings.push(format!(
            "Skipped {skipped_avoid_holes} hole(s) inside Avoid / Keep-Out region."
        ));
    }

    Ok(())
}

#[inline]
fn bore_compensated_feed(base_feed: f64, orbit_r: f64, tool_r: f64, compensate: bool) -> f64 {
    if !compensate || !base_feed.is_finite() || base_feed <= 0.0 || orbit_r <= 1e-6 {
        return base_feed;
    }
    let cut_wall_r = orbit_r + tool_r;
    if cut_wall_r <= 1e-6 {
        return base_feed;
    }
    let scale = (orbit_r / cut_wall_r).clamp(0.20, 1.0);
    base_feed * scale
}

fn compute_chip_clear_z_levels(
    hole_top_z: f64,
    hole_bottom_z: f64,
    stage_depth_mm: f64,
) -> Vec<f64> {
    let total_depth = (hole_top_z - hole_bottom_z).max(0.0);
    if total_depth <= stage_depth_mm + 1e-6 || !stage_depth_mm.is_finite() || stage_depth_mm <= 0.5
    {
        return vec![hole_bottom_z];
    }
    let n_stages = (total_depth / stage_depth_mm).ceil().max(1.0) as usize;
    let dz = total_depth / (n_stages as f64);
    let mut levels = Vec::with_capacity(n_stages);
    for i in 1..=n_stages {
        let z = if i == n_stages {
            hole_bottom_z
        } else {
            hole_top_z - dz * (i as f64)
        };
        levels.push(z);
    }
    levels
}

fn emit_helical_bore(
    ctx: &StrategyExecContext<'_>,
    params: &DrillBoreParams,
    builder: &mut ToolpathBuilder,
    spec: HelicalBoreSpec,
) -> Result<(), CamError> {
    let HelicalBoreSpec {
        hole,
        hole_top_z,
        hole_bottom_z,
        r_plane_z,
        retract_z,
    } = spec;
    let tool_d = ctx.tool.geometry.cutting_diameter_mm();
    let tool_r = 0.5 * tool_d;
    let radial_leave = ctx.passes.stock_to_leave_radial.value_mm;
    let comp = params.bore_inner_arc_feed_comp;

    let r_final = 0.5 * hole.diameter_mm - tool_r - radial_leave;
    if r_final <= 0.02 {
        return Err(CamError::HoleSmallerThanTool {
            hole_diameter_mm: hole.diameter_mm,
            tool_diameter_mm: tool_d + 2.0 * radial_leave,
        });
    }

    let fin_allowance =
        if params.bore_finish_pass && r_final > params.bore_finish_stepover.value_mm + 0.05 {
            params.bore_finish_stepover.value_mm.max(0.02)
        } else {
            0.0
        };
    let r_rough = (r_final - fin_allowance).max(0.02);

    // Maximum central helical path radius that leaves ZERO uncut pin in the center (`r <= 0.42 * tool_d` -> `< 0.84 * r_tool`!)
    let r_core_max = 0.42 * tool_d;
    let r_core = r_rough.min(r_core_max);

    let angle_rad = params.bore_ramp_angle_deg.clamp(0.5, 20.0).to_radians();
    let circumference = std::f64::consts::TAU * r_core;
    let pitch_mm = match &params.bore_pitch_per_rev {
        Some(p) if p.value_mm > 0.05 => p.value_mm,
        _ => {
            (circumference * angle_rad.tan()).clamp(0.15, ctx.passes.max_stepdown.value_mm.max(0.5))
        }
    };

    let c = hole.center_wcs;
    if let Some(cur) = builder.current_pos()
        && cur.z < retract_z - 1e-6
    {
        builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
    }
    let start_xy = c + DVec2::new(r_core, 0.0);
    builder.rapid_to(DVec3::new(start_xy.x, start_xy.y, retract_z));
    builder.rapid_to(DVec3::new(start_xy.x, start_xy.y, r_plane_z));

    // If `r_rough > r_core + 1e-4` (wide bore where `hole_d > ~1.84 * tool_d`), step down in Z levels
    // clearing the central helical core AND radial expansion rings at each Z level so tool engagement stays safe!
    let radial_stepover = ctx.cutting.stepover_mm.clamp(0.08 * tool_d, 0.45 * tool_d);
    let needs_radial_expansion = r_rough > r_core + 1e-4;
    let chip_clear_depth = params
        .bore_chip_clear_depth
        .as_ref()
        .map(|d| d.value_mm)
        .filter(|&d| d.is_finite() && d > 0.5);

    let z_levels = if needs_radial_expansion {
        crate::strategies::compute_z_levels(
            hole_top_z,
            hole_bottom_z,
            ctx.passes.multiple_depths,
            ctx.passes.max_stepdown.value_mm,
            ctx.passes.even_stepdowns,
        )?
    } else if let Some(clear_depth) = chip_clear_depth {
        compute_chip_clear_z_levels(hole_top_z, hole_bottom_z, clear_depth)
    } else {
        vec![hole_bottom_z]
    };

    let core_ramp_feed = bore_compensated_feed(ctx.cutting.ramp_feed_mm_min, r_core, tool_r, comp);
    let core_cut_feed =
        bore_compensated_feed(ctx.cutting.cutting_feed_mm_min, r_core, tool_r, comp);

    let mut prev_z = r_plane_z;
    let mut last_chip_clear_z = hole_top_z;
    for &z_target in &z_levels {
        ctx.check_cancel()?;
        builder.pass_count += 1;

        // Ensure positioned at `(c.x + r_core, c.y, prev_z)`
        let cur_start = DVec3::new(start_xy.x, start_xy.y, prev_z);
        if builder
            .current_pos()
            .is_none_or(|p| p.distance(cur_start) > 1e-4)
        {
            builder.linear_to(cur_start, ctx.cutting.lead_feed_mm_min, MotionRole::LeadIn);
        }

        // Helical CCW spiral (`Climb` inside internal bore!) from `prev_z` down to `z_target`
        let drop = (prev_z - z_target).max(0.0);
        let n_revs = (drop / pitch_mm).ceil().max(1.0) as usize;
        let dz_per_rev = drop / (n_revs as f64);
        for rev in 1..=n_revs {
            let z_rev = if rev == n_revs {
                z_target
            } else {
                prev_z - dz_per_rev * (rev as f64)
            };
            // Split each revolution into two 180° helical half-arcs so every controller handles it cleanly
            let opp_xy = c - DVec2::new(r_core, 0.0);
            let z_mid = 0.5 * (builder.current_pos().map_or(prev_z, |p| p.z) + z_rev);
            builder.arc_xy_to(XyArcMove {
                end_wcs: DVec3::new(opp_xy.x, opp_xy.y, z_mid),
                center_xy: c,
                radius_mm: r_core,
                ccw: true,
                turns: 0,
                feed_mm_min: core_ramp_feed,
                role: MotionRole::Ramp,
            });
            builder.arc_xy_to(XyArcMove {
                end_wcs: DVec3::new(start_xy.x, start_xy.y, z_rev),
                center_xy: c,
                radius_mm: r_core,
                ccw: true,
                turns: 0,
                feed_mm_min: core_ramp_feed,
                role: MotionRole::Ramp,
            });
        }

        // Flat 360° cleanup circle at `r_core`, `z_target`
        emit_full_circle_ccw(builder, c, r_core, z_target, core_cut_feed, MotionRole::Cut);

        // Radial step-out rings from `r_core` out to `r_rough` for wide bores (zero center pin!)
        if needs_radial_expansion {
            let span_r = r_rough - r_core;
            let n_rings = (span_r / radial_stepover).ceil().max(1.0) as usize;
            let dr = span_r / (n_rings as f64);
            let mut cur_r = r_core;
            for ring in 1..=n_rings {
                let next_r = if ring == n_rings {
                    r_rough
                } else {
                    r_core + dr * (ring as f64)
                };
                // Smooth tangent 180° transition arc from `(c.x + cur_r, c.y)` to `(c.x - next_r, c.y)`
                // centered at `(c.x + 0.5*(cur_r - next_r), c.y)` with radius `0.5*(cur_r + next_r)`!
                let trans_center = c + DVec2::new(0.5 * (cur_r - next_r), 0.0);
                let trans_r = 0.5 * (cur_r + next_r);
                let ring_lead_feed =
                    bore_compensated_feed(ctx.cutting.lead_feed_mm_min, trans_r, tool_r, comp);
                let ring_cut_feed =
                    bore_compensated_feed(ctx.cutting.cutting_feed_mm_min, next_r, tool_r, comp);
                builder.arc_xy_to(XyArcMove {
                    end_wcs: DVec3::new(c.x - next_r, c.y, z_target),
                    center_xy: trans_center,
                    radius_mm: trans_r,
                    ccw: true,
                    turns: 0,
                    feed_mm_min: ring_lead_feed,
                    role: MotionRole::LinkTransition,
                });
                // Complete full 360° circle at `next_r` starting from `-X` around to `+X`!
                builder.arc_xy_to(XyArcMove {
                    end_wcs: DVec3::new(c.x + next_r, c.y, z_target),
                    center_xy: c,
                    radius_mm: next_r,
                    ccw: true,
                    turns: 0,
                    feed_mm_min: ring_cut_feed,
                    role: MotionRole::Cut,
                });
                builder.arc_xy_to(XyArcMove {
                    end_wcs: DVec3::new(c.x - next_r, c.y, z_target),
                    center_xy: c,
                    radius_mm: next_r,
                    ccw: true,
                    turns: 0,
                    feed_mm_min: ring_cut_feed,
                    role: MotionRole::Cut,
                });
                builder.arc_xy_to(XyArcMove {
                    end_wcs: DVec3::new(c.x + next_r, c.y, z_target),
                    center_xy: c,
                    radius_mm: next_r,
                    ccw: true,
                    turns: 0,
                    feed_mm_min: ring_cut_feed,
                    role: MotionRole::Cut,
                });
                cur_r = next_r;
            }
            // Return smoothly toward `r_core` at `z_target` before starting the next Z stepdown!
            if (z_target - hole_bottom_z).abs() > 1e-5 {
                builder.linear_to(
                    DVec3::new(start_xy.x, start_xy.y, z_target),
                    ctx.cutting.lead_feed_mm_min,
                    MotionRole::LeadOut,
                );
            }
        }

        // Optional Z-staged chip-clearing retract to `r_plane_z` between deep helical stages
        if let Some(clear_depth) = chip_clear_depth
            && z_target > hole_bottom_z + 1e-5
            && (!needs_radial_expansion || (last_chip_clear_z - z_target) >= clear_depth - 1e-5)
        {
            // Tangent arc-off from `(c.x + r_core, c.y)` to hole center `(c.x, c.y)` before retract
            let clear_lead_r = 0.5 * r_core;
            let clear_lead_center = c + DVec2::new(clear_lead_r, 0.0);
            builder.arc_xy_to(XyArcMove {
                end_wcs: DVec3::new(c.x, c.y, z_target),
                center_xy: clear_lead_center,
                radius_mm: clear_lead_r,
                ccw: true,
                turns: 0,
                feed_mm_min: ctx.cutting.lead_feed_mm_min,
                role: MotionRole::LeadOut,
            });
            // Pull straight up to `r_plane_z` at cleared hole center to evacuate chips
            builder.rapid_to(DVec3::new(c.x, c.y, r_plane_z));
            // Reposition above stock at helix orbit start and feed cleanly down to just above floor
            builder.rapid_to(DVec3::new(start_xy.x, start_xy.y, r_plane_z));
            let resume_z = (z_target + 0.5).min(r_plane_z);
            builder.linear_to(
                DVec3::new(start_xy.x, start_xy.y, resume_z),
                ctx.cutting.lead_feed_mm_min,
                MotionRole::LeadIn,
            );
            prev_z = resume_z;
            last_chip_clear_z = z_target;
        } else {
            prev_z = z_target;
        }
    }

    // Optional radial finish pass + spring pass at `r_final`, `hole_bottom_z`
    if fin_allowance > 1e-5 {
        let cur_r = r_rough;
        let trans_center = c + DVec2::new(0.5 * (cur_r - r_final), 0.0);
        let trans_r = 0.5 * (cur_r + r_final);
        let finish_lead_feed =
            bore_compensated_feed(ctx.cutting.lead_feed_mm_min, trans_r, tool_r, comp);
        let finish_cut_feed =
            bore_compensated_feed(ctx.cutting.cutting_feed_mm_min, r_final, tool_r, comp);
        builder.arc_xy_to(XyArcMove {
            end_wcs: DVec3::new(c.x - r_final, c.y, hole_bottom_z),
            center_xy: trans_center,
            radius_mm: trans_r,
            ccw: true,
            turns: 0,
            feed_mm_min: finish_lead_feed,
            role: MotionRole::LeadIn,
        });
        // Full 360° finish pass (plus optional spring pass)
        let laps = if params.bore_spring_pass { 2 } else { 1 };
        for _ in 0..laps {
            builder.arc_xy_to(XyArcMove {
                end_wcs: DVec3::new(c.x + r_final, c.y, hole_bottom_z),
                center_xy: c,
                radius_mm: r_final,
                ccw: true,
                turns: 0,
                feed_mm_min: finish_cut_feed,
                role: MotionRole::FinishCut,
            });
            builder.arc_xy_to(XyArcMove {
                end_wcs: DVec3::new(c.x - r_final, c.y, hole_bottom_z),
                center_xy: c,
                radius_mm: r_final,
                ccw: true,
                turns: 0,
                feed_mm_min: finish_cut_feed,
                role: MotionRole::FinishCut,
            });
        }
        // Smooth tangent arc-off toward hole center `(c.x, c.y)` so cutter never rubs bore wall on G0 retract!
        let lead_out_r = 0.5 * r_final;
        let lead_out_center = c - DVec2::new(lead_out_r, 0.0);
        builder.arc_xy_to(XyArcMove {
            end_wcs: DVec3::new(c.x, c.y, hole_bottom_z),
            center_xy: lead_out_center,
            radius_mm: lead_out_r,
            ccw: true,
            turns: 0,
            feed_mm_min: ctx.cutting.lead_feed_mm_min,
            role: MotionRole::LeadOut,
        });
    } else {
        // Smooth arc-off from `(c.x + r_rough, c.y)` to bore center `(c.x, c.y)`!
        let lead_out_r = 0.5 * r_rough;
        let lead_out_center = c + DVec2::new(lead_out_r, 0.0);
        builder.arc_xy_to(XyArcMove {
            end_wcs: DVec3::new(c.x, c.y, hole_bottom_z),
            center_xy: lead_out_center,
            radius_mm: lead_out_r,
            ccw: true,
            turns: 0,
            feed_mm_min: ctx.cutting.lead_feed_mm_min,
            role: MotionRole::LeadOut,
        });
    }

    builder.rapid_to(DVec3::new(c.x, c.y, retract_z));

    Ok(())
}

fn emit_full_circle_ccw(
    builder: &mut ToolpathBuilder,
    center: DVec2,
    radius: f64,
    z: f64,
    feed: f64,
    role: MotionRole,
) {
    let p_pos = center + DVec2::new(radius, 0.0);
    let p_neg = center - DVec2::new(radius, 0.0);
    builder.arc_xy_to(XyArcMove {
        end_wcs: DVec3::new(p_neg.x, p_neg.y, z),
        center_xy: center,
        radius_mm: radius,
        ccw: true,
        turns: 0,
        feed_mm_min: feed,
        role,
    });
    builder.arc_xy_to(XyArcMove {
        end_wcs: DVec3::new(p_pos.x, p_pos.y, z),
        center_xy: center,
        radius_mm: radius,
        ccw: true,
        turns: 0,
        feed_mm_min: feed,
        role,
    });
}

/// Multi-start Nearest-Neighbor + Open-Path 2-Opt TSP hole ordering to minimize rapid travel.
fn sort_holes_nearest_2opt(holes: &mut [ResolvedHoleTarget]) {
    let n = holes.len();
    if n <= 1 {
        return;
    }

    let path_len = |seq: &[ResolvedHoleTarget]| -> f64 {
        seq.windows(2)
            .map(|w| w[0].center_wcs.distance(w[1].center_wcs))
            .sum()
    };

    // 1. Multi-start Greedy Nearest-Neighbor (tries all starts for n <= 48, else extreme corner seed)
    let starts: Vec<usize> = if n <= 48 {
        (0..n).collect()
    } else {
        let mut min_idx = 0;
        for (idx, h) in holes.iter().enumerate() {
            let cur = holes[min_idx].center_wcs;
            if (h.center_wcs.x + h.center_wcs.y) < (cur.x + cur.y) {
                min_idx = idx;
            }
        }
        vec![0, min_idx]
    };

    let mut best_tour = holes.to_vec();
    let mut best_cost = path_len(&best_tour);

    for start_idx in starts {
        let mut cand = holes.to_vec();
        cand.swap(0, start_idx);
        for i in 0..n - 1 {
            let cur = cand[i].center_wcs;
            let mut best_j = i + 1;
            let mut best_d = cur.distance_squared(cand[i + 1].center_wcs);
            for (j, cand_j) in cand.iter().enumerate().take(n).skip(i + 2) {
                let d = cur.distance_squared(cand_j.center_wcs);
                if d < best_d {
                    best_d = d;
                    best_j = j;
                }
            }
            cand.swap(i + 1, best_j);
        }
        let cost = path_len(&cand);
        if cost + 1e-9 < best_cost {
            best_cost = cost;
            best_tour = cand;
        }
    }
    holes.copy_from_slice(&best_tour);

    // 2. Open-Path 2-Opt uncrossing refinement (including prefix `0..=k` and suffix `i..n` reversals!)
    for _pass in 0..8 {
        let mut improved = false;
        // Check prefix reversals `0..=k` (`k < n - 1`)
        for k in 1..(n - 1) {
            let old_edge = holes[k].center_wcs.distance(holes[k + 1].center_wcs);
            let new_edge = holes[0].center_wcs.distance(holes[k + 1].center_wcs);
            if old_edge - new_edge > 1e-6 {
                holes[0..=k].reverse();
                improved = true;
            }
        }
        // Check interior & suffix reversals `(i + 1)..=k`
        for i in 0..n.saturating_sub(2) {
            for k in (i + 2)..n {
                let a = holes[i].center_wcs;
                let b = holes[i + 1].center_wcs;
                let c = holes[k].center_wcs;
                let gain = if k + 1 < n {
                    let d = holes[k + 1].center_wcs;
                    (a.distance(b) + c.distance(d)) - (a.distance(c) + b.distance(d))
                } else {
                    a.distance(b) - a.distance(c)
                };
                if gain > 1e-6 {
                    holes[i + 1..=k].reverse();
                    improved = true;
                }
            }
        }
        if !improved {
            break;
        }
    }
}
