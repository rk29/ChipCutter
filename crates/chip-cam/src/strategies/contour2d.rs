use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3};

use crate::{
    cl_data::{ArcPlane, ClEvent, CutterCompMode, MotionRole},
    offset2d::offset_polyline,
    params::{
        Contour2DParams, ContourCompensation, EntryStrategy, HoldingTabsConfig, LeadConfig,
        TabPlacementMode, TabProfileKind,
    },
    rest2d::{ContourRestClipQuery, RegionSet2D, clip_contour_path_to_rest},
    selection::ExtractedFeature25D,
    strategies::{StrategyExecContext, compute_z_levels},
    surface3d::{
        PushCutterField, SurfaceCutterProfile, SurfaceMesh3D, clip_2d_contour_outside_overhangs,
        is_2d_arc_clear_of_avoid, is_2d_segment_clear_of_avoid,
    },
    toolpath::{TabMarker3D, ToolpathBuilder, VerticalArcMove, XyArcMove},
    wcs::CamError,
};

#[derive(Debug, Clone)]
struct SourceContour {
    poly_wcs: ArcPolyline2D,
    side_flip: bool,
}

#[derive(Debug, Clone, Copy)]
struct TabInterval {
    center_s_mm: f64,
    enter_s_mm: f64,
    apex_start_s_mm: f64,
    apex_end_s_mm: f64,
    exit_s_mm: f64,
    tab_top_z_wcs: f64,
    is_triangular: bool,
}

/// Generates a **2D Contour / Profile** toolpath with multi-pass side/depth offsets, horizontal & vertical
/// lead-in/lead-out arcs, seam overlap, spiral/linear ramps, and Rectangular/Triangular holding tabs (`05-operations.md` §2).
pub fn generate_contour2d(
    ctx: &StrategyExecContext<'_>,
    params: &Contour2DParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    let mut contours: Vec<SourceContour> = Vec::new();
    for feat in &ctx.selection.features {
        match feat {
            ExtractedFeature25D::Contour {
                polyline_wcs,
                side_flip,
                ..
            } => {
                if !polyline_wcs.segments.is_empty() {
                    contours.push(SourceContour {
                        poly_wcs: polyline_wcs.clone(),
                        side_flip: *side_flip,
                    });
                }
            }
            ExtractedFeature25D::PlanarFace {
                outer_boundary_wcs,
                raised_islands_wcs,
                ..
            } => {
                // For a planar face in 2D Contour, profile the outer boundary (outside if boss, inside if cavity;
                // default CCW outer loop with LeftClimb offsets inward unless flipped, so ensure CW for exterior climb
                // or honor compensation cleanly).
                if !outer_boundary_wcs.segments.is_empty() {
                    contours.push(SourceContour {
                        poly_wcs: outer_boundary_wcs.ensure_orientation(false),
                        side_flip: false,
                    });
                }
                for isl in raised_islands_wcs {
                    if !isl.segments.is_empty() {
                        contours.push(SourceContour {
                            poly_wcs: isl.ensure_orientation(false),
                            side_flip: false,
                        });
                    }
                }
            }
            _ => {}
        }
    }

    if contours.is_empty() {
        return Err(CamError::EmptySelection {
            op_name: ctx.op.name.clone(),
            op_kind: ctx.op.kind.label(),
        });
    }

    let tool_d = ctx.tool.geometry.cutting_diameter_mm();
    let tool_r = 0.5 * tool_d;
    let radial_leave = ctx.passes.stock_to_leave_radial.value_mm;
    let target_bottom_z = ctx.heights.bottom_z_wcs + ctx.passes.stock_to_leave_axial.value_mm;

    let z_levels = compute_z_levels(
        ctx.heights.top_z_wcs,
        target_bottom_z,
        ctx.passes.multiple_depths,
        ctx.passes.max_stepdown.value_mm,
        ctx.passes.even_stepdowns,
    )?;

    // Side pass offsets (extra radial allowance beyond `tool_r + radial_leave`), ordered outermost -> innermost!
    let finish_count = params.finish_passes as usize;
    let finish_step = params.finish_stepover.value_mm.max(0.01);
    let rough_count = params
        .roughing_passes
        .max(if finish_count == 0 { 1 } else { 0 }) as usize;
    let rough_step = params.roughing_stepover.value_mm.max(0.05);

    let total_finish_stock = (finish_count as f64) * finish_step;
    let mut side_passes: Vec<(f64, bool)> = Vec::new(); // `(extra_offset_mm, is_finish)`
    for r_idx in (0..rough_count).rev() {
        let extra = total_finish_stock + (r_idx as f64) * rough_step;
        side_passes.push((extra, false));
    }
    for f_idx in (0..finish_count).rev() {
        let extra = (f_idx as f64) * finish_step;
        side_passes.push((extra, true));
    }
    if params.repeat_finish_pass {
        side_passes.push((0.0, true));
    }
    if side_passes.is_empty() {
        side_passes.push((0.0, false));
    }

    let overhang_cutter = SurfaceCutterProfile::from_tool(
        ctx.tool,
        radial_leave.max(0.0),
        ctx.passes.stock_to_leave_axial.value_mm.max(0.0),
        0.5,
    );
    let overhang_surface = (ctx.passes.avoid_model_overhangs && ctx.doc.has_3d_geometry())
        .then(|| SurfaceMesh3D::from_cad_document(ctx.doc, &[], &ctx.setup.frame));
    let overhang_field = overhang_surface
        .as_ref()
        .filter(|s| !s.triangles.is_empty() && s.aabb_wcs.is_valid())
        .map(|s| PushCutterField::build_with_z_min(s, overhang_cutter, target_bottom_z));

    for (contour_idx, src) in contours.iter().enumerate() {
        ctx.check_cancel()?;

        // Determine signed left-hand offset direction (`+1` = left of travel, `-1` = right, `0` = center)
        let base_sign: f64 = match params.compensation {
            ContourCompensation::LeftClimb => {
                if src.side_flip {
                    -1.0
                } else {
                    1.0
                }
            }
            ContourCompensation::RightConventional => {
                if src.side_flip {
                    1.0
                } else {
                    -1.0
                }
            }
            ContourCompensation::CenterOff => 0.0,
        };

        // Precompute holding tab centers on the source contour (so markers display on the CAD edge
        // and project cleanly onto the final offset pass!)
        let src_tab_s_norms = if params.tabs.enabled && src.poly_wcs.closed {
            resolve_tab_s_norms(contour_idx, &src.poly_wcs, &params.tabs, tool_d)
        } else {
            Vec::new()
        };

        let is_tri = matches!(params.tabs.profile, TabProfileKind::Triangular { .. });
        for &s_norm in &src_tab_s_norms {
            if let Some((pt2, tan2)) = src.poly_wcs.sample_at_fraction(s_norm) {
                builder.tab_markers_wcs.push(TabMarker3D {
                    center_wcs: DVec3::new(pt2.x, pt2.y, target_bottom_z),
                    tangent_xy: tan2,
                    width_mm: params.tabs.width.value_mm.max(0.5),
                    height_mm: params.tabs.height.value_mm.max(0.1),
                    is_triangular: is_tri,
                    contour_index: contour_idx,
                    s_norm,
                });
            }
        }

        for &(extra_radial, is_finish) in &side_passes {
            ctx.check_cancel()?;
            let offset_mag = if base_sign == 0.0 {
                0.0
            } else {
                (tool_r + radial_leave + extra_radial).max(0.0)
            };
            let signed_offset = base_sign * offset_mag;

            let offset_loops = offset_polyline(&src.poly_wcs, signed_offset);
            if offset_loops.is_empty() {
                continue;
            }

            let cut_feed = if is_finish {
                params
                    .finish_feed_mm_min
                    .unwrap_or(ctx.cutting.cutting_feed_mm_min)
            } else {
                ctx.cutting.cutting_feed_mm_min
            };
            let cut_role = if is_finish {
                MotionRole::FinishCut
            } else {
                MotionRole::Cut
            };

            for raw_loop in offset_loops {
                ctx.check_cancel()?;
                // Shift closed-loop seam to the midpoint of its longest smooth segment so lead-in/out never lands on a sharp corner!
                let poly = if raw_loop.closed {
                    shift_seam_to_longest_segment_midpoint(&raw_loop)
                } else {
                    raw_loop
                };
                if poly.perimeter() <= 1e-4 {
                    continue;
                }

                // Project holding tabs onto this offset loop (only needed on the innermost radial pass where extra_radial == 0,
                // or any pass touching the part wall)
                let loop_tabs_s_mm: Vec<f64> =
                    if !src_tab_s_norms.is_empty() && extra_radial <= 1e-4 {
                        src_tab_s_norms
                            .iter()
                            .filter_map(|&sn| {
                                let (src_pt, _) = src.poly_wcs.sample_at_fraction(sn)?;
                                poly.project_point(src_pt).map(|pr| pr.s_mm)
                            })
                            .collect()
                    } else {
                        Vec::new()
                    };

                let rest_enabled = !ctx.passes.rest_machining.is_disabled();
                let avoid_overhangs = overhang_field.is_some();
                if params.continuous_spiral_ramp
                    && poly.closed
                    && loop_tabs_s_mm.is_empty()
                    && !rest_enabled
                    && !avoid_overhangs
                {
                    emit_continuous_spiral_contour(
                        ctx,
                        params,
                        builder,
                        &SpiralContourEmitSpec {
                            poly: &poly,
                            base_sign,
                            z_levels: &z_levels,
                            target_bottom_z,
                            cut_feed,
                            cut_role,
                        },
                    );
                } else if rest_enabled || avoid_overhangs {
                    for &z_cut in &z_levels {
                        ctx.check_cancel()?;
                        let mut spans = if rest_enabled {
                            clip_contour_path_to_rest(&ContourRestClipQuery {
                                raw_contour: &src.poly_wcs,
                                offset_sign: base_sign,
                                candidate_path: &poly,
                                current_tool_radius_mm: tool_r,
                                rest_cfg: &ctx.passes.rest_machining,
                                upstream: ctx.upstream_rest,
                                z_level_wcs: z_cut,
                            })
                        } else {
                            vec![poly.clone()]
                        };
                        let avoid_z = overhang_field
                            .as_ref()
                            .map(|f| f.avoidance_region_z(z_cut))
                            .filter(|r| !r.is_empty());
                        if let (Some(surf), Some(avoid)) =
                            (overhang_surface.as_ref(), avoid_z.as_ref())
                        {
                            spans = spans
                                .iter()
                                .flat_map(|sp| {
                                    clip_2d_contour_outside_overhangs(
                                        sp,
                                        avoid,
                                        surf,
                                        &overhang_cutter,
                                        z_cut,
                                    )
                                })
                                .collect();
                        }
                        for span in &spans {
                            let span_tabs: Vec<f64> = if !src_tab_s_norms.is_empty()
                                && extra_radial <= 1e-4
                                && span.closed
                            {
                                loop_tabs_s_mm.clone()
                            } else {
                                Vec::new()
                            };
                            emit_single_depth_contour_pass(
                                ctx,
                                params,
                                builder,
                                &SingleDepthContourPassSpec {
                                    poly: span,
                                    base_sign,
                                    z_cut,
                                    target_bottom_z,
                                    tab_centers_s_mm: &span_tabs,
                                    tool_r,
                                    cut_feed,
                                    cut_role,
                                    avoid_z: avoid_z.as_ref(),
                                },
                            );
                        }
                    }
                } else {
                    for &z_cut in &z_levels {
                        ctx.check_cancel()?;
                        emit_single_depth_contour_pass(
                            ctx,
                            params,
                            builder,
                            &SingleDepthContourPassSpec {
                                poly: &poly,
                                base_sign,
                                z_cut,
                                target_bottom_z,
                                tab_centers_s_mm: &loop_tabs_s_mm,
                                tool_r,
                                cut_feed,
                                cut_role,
                                avoid_z: None,
                            },
                        );
                    }
                }
            }
        }
    }

    Ok(())
}

fn shift_seam_to_longest_segment_midpoint(poly: &ArcPolyline2D) -> ArcPolyline2D {
    if !poly.closed || poly.segments.is_empty() {
        return poly.clone();
    }
    let mut best_mid_s = 0.0;
    let mut best_len = -1.0_f64;
    let mut accum = 0.0;
    for seg in &poly.segments {
        let len = seg.length();
        if len > best_len {
            best_len = len;
            best_mid_s = accum + 0.5 * len;
        }
        accum += len;
    }
    poly.rotate_closed_seam(best_mid_s)
}

fn resolve_tab_s_norms(
    contour_index: usize,
    poly: &ArcPolyline2D,
    tabs: &HoldingTabsConfig,
    tool_d: f64,
) -> Vec<f64> {
    let perim = poly.perimeter();
    if perim <= 1e-3 {
        return Vec::new();
    }
    match &tabs.placement {
        TabPlacementMode::Manual => tabs
            .manual_anchors
            .iter()
            .filter(|a| a.contour_index == contour_index)
            .map(|a| a.s_norm.clamp(0.0, 1.0))
            .collect(),
        TabPlacementMode::Automatic {
            count,
            by_spacing,
            target_spacing,
        } => {
            let n_tabs = if *by_spacing && target_spacing.value_mm > 1.0 {
                (perim / target_spacing.value_mm).round().clamp(1.0, 64.0) as usize
            } else {
                (*count).clamp(1, 64) as usize
            };

            // Build candidate safe segments (`Line` or `Arc` with `radius >= 2.0 * tool_d` and length `>= tab_width`)
            let min_arc_r = 2.0 * tool_d;
            let mut out = Vec::with_capacity(n_tabs);
            for i in 0..n_tabs {
                let ideal_frac = (i as f64 + 0.5) / (n_tabs as f64);
                let best_frac = nudge_tab_away_from_corners(poly, ideal_frac, min_arc_r);
                out.push(best_frac);
            }
            out
        }
    }
}

/// Nudges an automatic tab candidate within `±0.08 * perimeter` so it lands away from segment
/// vertices and avoids tight corner arcs (`radius < min_arc_r`).
fn nudge_tab_away_from_corners(poly: &ArcPolyline2D, ideal_frac: f64, min_arc_r: f64) -> f64 {
    let perim = poly.perimeter();
    if perim <= 1e-6 {
        return ideal_frac;
    }

    let score_s = |frac: f64| -> f64 {
        let f = frac.rem_euclid(1.0);
        let target_s = f * perim;
        let mut accum = 0.0;
        for seg in &poly.segments {
            let len = seg.length();
            if accum + len >= target_s {
                let dist_to_ends = (target_s - accum).min((accum + len) - target_s);
                let tight_arc_penalty = match *seg {
                    ArcSegment2D::Arc { radius, .. } if radius < min_arc_r => -1_000.0,
                    _ => 0.0,
                };
                let drift_penalty = -15.0 * (f - ideal_frac).abs();
                return dist_to_ends + tight_arc_penalty + drift_penalty;
            }
            accum += len;
        }
        0.0
    };

    let mut best_f = ideal_frac;
    let mut best_score = score_s(ideal_frac);
    for step in -8..=8 {
        let cand = (ideal_frac + (step as f64) * 0.01).rem_euclid(1.0);
        let sc = score_s(cand);
        if sc > best_score {
            best_score = sc;
            best_f = cand;
        }
    }
    best_f
}

struct SpiralContourEmitSpec<'a> {
    poly: &'a ArcPolyline2D,
    base_sign: f64,
    z_levels: &'a [f64],
    target_bottom_z: f64,
    cut_feed: f64,
    cut_role: MotionRole,
}

struct SingleDepthContourPassSpec<'a> {
    poly: &'a ArcPolyline2D,
    base_sign: f64,
    z_cut: f64,
    target_bottom_z: f64,
    tab_centers_s_mm: &'a [f64],
    tool_r: f64,
    cut_feed: f64,
    cut_role: MotionRole,
    avoid_z: Option<&'a RegionSet2D>,
}

struct LeadArcSpec<'a> {
    poly: &'a ArcPolyline2D,
    anchor_pt: DVec2,
    anchor_tan: DVec2,
    base_sign: f64,
    z_cut: f64,
    retract_z: f64,
    avoid_z: Option<&'a RegionSet2D>,
}

fn emit_continuous_spiral_contour(
    ctx: &StrategyExecContext<'_>,
    params: &Contour2DParams,
    builder: &mut ToolpathBuilder,
    spec: &SpiralContourEmitSpec<'_>,
) {
    let &SpiralContourEmitSpec {
        poly,
        base_sign,
        z_levels,
        target_bottom_z,
        cut_feed,
        cut_role,
    } = spec;
    let Some((entry_pt, entry_tan)) = poly.sample_at_fraction(0.0) else {
        return;
    };
    let retract_z = ctx.safe_retract_z();
    let start_z = ctx.heights.top_z_wcs;

    emit_approach_and_lead_in(
        ctx,
        &ctx.linking.lead_in,
        builder,
        &LeadArcSpec {
            poly,
            anchor_pt: entry_pt,
            anchor_tan: entry_tan,
            base_sign,
            z_cut: start_z,
            retract_z,
            avoid_z: None,
        },
    );

    if params.wear_comp_metadata && base_sign != 0.0 {
        builder.push_event(ClEvent::CutterComp {
            mode: if base_sign > 0.0 {
                CutterCompMode::WearLeft
            } else {
                CutterCompMode::WearRight
            },
        });
    }

    // Spiral continuously down through each Z level per revolution around `poly`
    let mut prev_z = start_z;
    for &z_lvl in z_levels {
        builder.pass_count += 1;
        builder.emit_polyline_2d_z_interp(poly, prev_z, z_lvl, cut_feed, MotionRole::Ramp);
        prev_z = z_lvl;
    }

    // Flat 360° cleanup lap at `target_bottom_z` + seam overlap!
    builder.pass_count += 1;
    builder.emit_polyline_2d_z_interp(poly, target_bottom_z, target_bottom_z, cut_feed, cut_role);
    let overlap = ctx
        .linking
        .seam_overlap
        .value_mm
        .clamp(0.0, 0.25 * poly.perimeter());
    if overlap > 1e-4 {
        let ov_slice = poly.slice_interval(0.0, overlap);
        builder.emit_polyline_2d_z_interp(
            &ov_slice,
            target_bottom_z,
            target_bottom_z,
            cut_feed,
            cut_role,
        );
    }

    if params.wear_comp_metadata && base_sign != 0.0 {
        builder.push_event(ClEvent::CutterComp {
            mode: CutterCompMode::Off,
        });
    }

    let exit_s = overlap;
    if let Some((exit_pt, exit_tan)) = poly.eval_at_distance(exit_s) {
        let lead_out_cfg = if ctx.linking.lead_out_same_as_in {
            &ctx.linking.lead_in
        } else {
            &ctx.linking.lead_out
        };
        emit_lead_out_and_retract(
            ctx,
            lead_out_cfg,
            builder,
            &LeadArcSpec {
                poly,
                anchor_pt: exit_pt,
                anchor_tan: exit_tan,
                base_sign,
                z_cut: target_bottom_z,
                retract_z,
                avoid_z: None,
            },
        );
    }
}

fn emit_single_depth_contour_pass(
    ctx: &StrategyExecContext<'_>,
    params: &Contour2DParams,
    builder: &mut ToolpathBuilder,
    spec: &SingleDepthContourPassSpec<'_>,
) {
    let &SingleDepthContourPassSpec {
        poly,
        base_sign,
        z_cut,
        target_bottom_z,
        tab_centers_s_mm,
        tool_r,
        cut_feed,
        cut_role,
        avoid_z,
    } = spec;
    let Some((entry_pt, entry_tan)) = poly.sample_at_fraction(0.0) else {
        return;
    };
    let retract_z = ctx.safe_retract_z();
    builder.pass_count += 1;

    // Linear ramp along closed contour when Lead-In is disabled and EntryStrategy::LinearRamp is requested!
    let use_profile_ramp = poly.closed
        && !ctx.linking.lead_in.enabled
        && matches!(
            ctx.linking.entry_mode,
            EntryStrategy::LinearRamp | EntryStrategy::HelicalRamp
        );

    if use_profile_ramp {
        let ramp_start_z = (z_cut + ctx.passes.max_stepdown.value_mm).min(ctx.heights.top_z_wcs);
        if let Some(cur) = builder.current_pos()
            && cur.z < retract_z - 1e-6
        {
            builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
        }
        builder.rapid_to(DVec3::new(entry_pt.x, entry_pt.y, retract_z));
        builder.rapid_to(DVec3::new(entry_pt.x, entry_pt.y, ctx.heights.feed_z_wcs));
        builder.linear_to(
            DVec3::new(entry_pt.x, entry_pt.y, ramp_start_z),
            ctx.cutting.plunge_feed_mm_min,
            MotionRole::Plunge,
        );
        let angle_rad = ctx.linking.ramp_angle_deg.clamp(0.5, 30.0).to_radians();
        let needed_len = ((ramp_start_z - z_cut).max(0.0)) / angle_rad.tan();
        let perim = poly.perimeter();
        let full_laps = (needed_len / perim).floor() as usize;
        let rem_len = needed_len - (full_laps as f64) * perim;
        let mut cur_z = ramp_start_z;
        let dz_per_mm = if needed_len > 1e-6 {
            (ramp_start_z - z_cut) / needed_len
        } else {
            0.0
        };
        for _ in 0..full_laps {
            let next_z = (cur_z - dz_per_mm * perim).max(z_cut);
            builder.emit_polyline_2d_z_interp(
                poly,
                cur_z,
                next_z,
                ctx.cutting.ramp_feed_mm_min,
                MotionRole::Ramp,
            );
            cur_z = next_z;
        }
        if rem_len > 1e-4 {
            let sub = poly.slice_interval(0.0, rem_len);
            builder.emit_polyline_2d_z_interp(
                &sub,
                cur_z,
                z_cut,
                ctx.cutting.ramp_feed_mm_min,
                MotionRole::Ramp,
            );
            // Complete the rest of the loop back to s=0 at z_cut so the full pass starts cleanly at s=0
            let tail = poly.slice_interval(rem_len, perim);
            builder.emit_polyline_2d_z_interp(&tail, z_cut, z_cut, cut_feed, cut_role);
        }
    } else {
        emit_approach_and_lead_in(
            ctx,
            &ctx.linking.lead_in,
            builder,
            &LeadArcSpec {
                poly,
                anchor_pt: entry_pt,
                anchor_tan: entry_tan,
                base_sign,
                z_cut,
                retract_z,
                avoid_z,
            },
        );
    }

    if params.wear_comp_metadata && base_sign != 0.0 {
        builder.push_event(ClEvent::CutterComp {
            mode: if base_sign > 0.0 {
                CutterCompMode::WearLeft
            } else {
                CutterCompMode::WearRight
            },
        });
    }

    // Check if Holding Tabs are active at `z_cut`!
    let tab_top_z = target_bottom_z + params.tabs.height.value_mm.max(0.0);
    let active_tabs = if poly.closed && !tab_centers_s_mm.is_empty() && z_cut < tab_top_z - 1e-5 {
        build_sorted_tab_intervals(
            poly.perimeter(),
            tab_centers_s_mm,
            &params.tabs,
            tool_r,
            z_cut,
            tab_top_z,
        )
    } else {
        Vec::new()
    };

    if active_tabs.is_empty() {
        builder.emit_polyline_2d_z_interp(poly, z_cut, z_cut, cut_feed, cut_role);
    } else {
        emit_contour_with_holding_tabs(ctx, builder, poly, z_cut, &active_tabs, cut_feed, cut_role);
    }

    // Seam overlap on closed contours (when `s=0` is not inside a tab)
    let mut exit_s = if poly.closed { 0.0 } else { poly.perimeter() };
    if poly.closed && active_tabs.is_empty() {
        let overlap = ctx
            .linking
            .seam_overlap
            .value_mm
            .clamp(0.0, 0.25 * poly.perimeter());
        if overlap > 1e-4 {
            let ov_slice = poly.slice_interval(0.0, overlap);
            builder.emit_polyline_2d_z_interp(&ov_slice, z_cut, z_cut, cut_feed, cut_role);
            exit_s = overlap;
        }
    }

    if params.wear_comp_metadata && base_sign != 0.0 {
        builder.push_event(ClEvent::CutterComp {
            mode: CutterCompMode::Off,
        });
    }

    if let Some((exit_pt, exit_tan)) = poly.eval_at_distance(exit_s) {
        let lead_out_cfg = if ctx.linking.lead_out_same_as_in {
            &ctx.linking.lead_in
        } else {
            &ctx.linking.lead_out
        };
        emit_lead_out_and_retract(
            ctx,
            lead_out_cfg,
            builder,
            &LeadArcSpec {
                poly,
                anchor_pt: exit_pt,
                anchor_tan: exit_tan,
                base_sign,
                z_cut,
                retract_z,
                avoid_z,
            },
        );
    } else if let Some(cur) = builder.current_pos() {
        builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
    }
}

fn build_sorted_tab_intervals(
    perim: f64,
    centers_s_mm: &[f64],
    tabs: &HoldingTabsConfig,
    tool_r: f64,
    z_cut: f64,
    tab_top_z: f64,
) -> Vec<TabInterval> {
    let dz = (tab_top_z - z_cut).max(0.0);
    let half_flat = 0.5 * tabs.width.value_mm.max(0.5) + tool_r;

    let mut list = Vec::with_capacity(centers_s_mm.len());
    for &raw_c in centers_s_mm {
        let c = raw_c.rem_euclid(perim);
        let (ramp_len, apex_half, is_tri) = match tabs.profile {
            TabProfileKind::Rectangular => (0.0, half_flat, false),
            TabProfileKind::Triangular { ramp_angle_deg } => {
                let rad = ramp_angle_deg.clamp(5.0, 80.0).to_radians();
                let r_len = dz / rad.tan();
                // Smooth apex bridge (`tool_r` + small flat crown) with symmetric sloped sides!
                let crown_half = tool_r + 0.15 * tabs.width.value_mm.max(0.4);
                (r_len, crown_half, true)
            }
        };
        let total_half = (apex_half + ramp_len).min(0.42 * perim);
        // Avoid wrapping across seam `s = 0` so single-pass interval slicing is monotonic `[0..perim]`
        let c_clamped = c.clamp(
            total_half + 1e-3,
            (perim - total_half - 1e-3).max(total_half + 1e-3),
        );
        if c_clamped + total_half >= perim {
            continue;
        }
        list.push(TabInterval {
            center_s_mm: c_clamped,
            enter_s_mm: (c_clamped - total_half).max(0.0),
            apex_start_s_mm: (c_clamped - apex_half).max(0.0),
            apex_end_s_mm: (c_clamped + apex_half).min(perim),
            exit_s_mm: (c_clamped + total_half).min(perim),
            tab_top_z_wcs: tab_top_z,
            is_triangular: is_tri,
        });
    }

    list.sort_by(|a, b| a.enter_s_mm.total_cmp(&b.enter_s_mm));

    // Drop any overlapping duplicate tabs
    let mut non_overlapping: Vec<TabInterval> = Vec::with_capacity(list.len());
    for item in list {
        if non_overlapping
            .last()
            .is_none_or(|prev| item.enter_s_mm > prev.exit_s_mm + 1e-3)
        {
            non_overlapping.push(item);
        }
    }
    non_overlapping
}

fn emit_contour_with_holding_tabs(
    ctx: &StrategyExecContext<'_>,
    builder: &mut ToolpathBuilder,
    poly: &ArcPolyline2D,
    z_cut: f64,
    tabs: &[TabInterval],
    cut_feed: f64,
    cut_role: MotionRole,
) {
    let perim = poly.perimeter();
    let mut cursor_s = 0.0_f64;

    for tab in tabs {
        // 1. Cut normally at `z_cut` from `cursor_s` up to `tab.enter_s_mm`
        if tab.enter_s_mm > cursor_s + 1e-4 {
            let cut_seg = poly.slice_interval(cursor_s, tab.enter_s_mm);
            builder.emit_polyline_2d_z_interp(&cut_seg, z_cut, z_cut, cut_feed, cut_role);
        }

        if tab.is_triangular {
            // 2a. Smooth 3D linear Z-ramp UP along the contour curve to `tab_top_z_wcs`!
            if tab.apex_start_s_mm > tab.enter_s_mm + 1e-4 {
                let up_seg = poly.slice_interval(tab.enter_s_mm, tab.apex_start_s_mm);
                builder.emit_polyline_2d_z_interp(
                    &up_seg,
                    z_cut,
                    tab.tab_top_z_wcs,
                    ctx.cutting.ramp_feed_mm_min,
                    MotionRole::TabRamp,
                );
            }
            // Crown across tab apex
            if tab.apex_end_s_mm > tab.apex_start_s_mm + 1e-4 {
                let crown_seg = poly.slice_interval(tab.apex_start_s_mm, tab.apex_end_s_mm);
                builder.emit_polyline_2d_z_interp(
                    &crown_seg,
                    tab.tab_top_z_wcs,
                    tab.tab_top_z_wcs,
                    cut_feed,
                    MotionRole::TabRamp,
                );
            }
            // Smooth 3D linear Z-ramp DOWN along the contour curve back to `z_cut`!
            if tab.exit_s_mm > tab.apex_end_s_mm + 1e-4 {
                let down_seg = poly.slice_interval(tab.apex_end_s_mm, tab.exit_s_mm);
                builder.emit_polyline_2d_z_interp(
                    &down_seg,
                    tab.tab_top_z_wcs,
                    z_cut,
                    ctx.cutting.ramp_feed_mm_min,
                    MotionRole::TabRamp,
                );
            }
        } else {
            // 2b. Rectangular tab: vertical feed lift -> bridge across tab -> vertical plunge!
            if let Some((p_in, _)) = poly.eval_at_distance(tab.enter_s_mm) {
                builder.linear_to(
                    DVec3::new(p_in.x, p_in.y, tab.tab_top_z_wcs),
                    ctx.cutting.lead_feed_mm_min,
                    MotionRole::TabRamp,
                );
            }
            let bridge_seg = poly.slice_interval(tab.enter_s_mm, tab.exit_s_mm);
            builder.emit_polyline_2d_z_interp(
                &bridge_seg,
                tab.tab_top_z_wcs,
                tab.tab_top_z_wcs,
                cut_feed,
                MotionRole::TabRamp,
            );
            if let Some((p_out, _)) = poly.eval_at_distance(tab.exit_s_mm) {
                builder.linear_to(
                    DVec3::new(p_out.x, p_out.y, z_cut),
                    ctx.cutting.plunge_feed_mm_min,
                    MotionRole::Plunge,
                );
            }
        }

        let _ = tab.center_s_mm;
        cursor_s = tab.exit_s_mm;
    }

    // 3. Cut remaining tail back to `perim`
    if perim > cursor_s + 1e-4 {
        let tail_seg = poly.slice_interval(cursor_s, perim);
        builder.emit_polyline_2d_z_interp(&tail_seg, z_cut, z_cut, cut_feed, cut_role);
    }
}

fn emit_approach_and_lead_in(
    ctx: &StrategyExecContext<'_>,
    lead: &LeadConfig,
    builder: &mut ToolpathBuilder,
    spec: &LeadArcSpec<'_>,
) {
    let &LeadArcSpec {
        poly,
        anchor_pt: entry_pt,
        anchor_tan: entry_tan,
        base_sign,
        z_cut,
        retract_z,
        avoid_z,
    } = spec;
    let feed_z = ctx.heights.feed_z_wcs;
    if let Some(cur) = builder.current_pos()
        && cur.z < retract_z - 1e-6
    {
        builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
    }

    if !lead.enabled || base_sign == 0.0 {
        builder.rapid_to(DVec3::new(entry_pt.x, entry_pt.y, retract_z));
        builder.rapid_to(DVec3::new(entry_pt.x, entry_pt.y, feed_z));
        builder.linear_to(
            DVec3::new(entry_pt.x, entry_pt.y, z_cut),
            ctx.cutting.plunge_feed_mm_min,
            MotionRole::Plunge,
        );
        return;
    }

    let aabb_sz = poly.exact_aabb().size();
    let max_safe_r = if poly.closed {
        (0.35 * aabb_sz.x.min(aabb_sz.y)).max(0.2)
    } else {
        50.0
    };
    let r_h = lead.horizontal_radius.value_mm.clamp(0.0, max_safe_r);
    let l_lin = lead.linear_length.value_mm.max(0.0);
    let sweep_rad = lead.horizontal_sweep_deg.clamp(0.0, 180.0).to_radians();
    let r_v = lead.vertical_radius.value_mm.max(0.0);

    let t = entry_tan.normalize_or_zero();
    let n_safe = if base_sign >= 0.0 {
        DVec2::new(-t.y, t.x)
    } else {
        DVec2::new(t.y, -t.x)
    };

    let p_arc_end = entry_pt - t * l_lin;
    let p_arc_start = if r_h > 1e-4 && sweep_rad > 1e-3 {
        p_arc_end + n_safe * (r_h * (1.0 - sweep_rad.cos())) - t * (r_h * sweep_rad.sin())
    } else {
        p_arc_end
    };

    // Optional Vertical Lead-In (`G18` XZ or `G19` YZ if tangent aligns with X/Y, otherwise smooth 3D linear/helical drop)
    let approach_xy = if r_v > 1e-4 && r_h <= 1e-4 {
        p_arc_start - t * r_v
    } else {
        p_arc_start
    };

    // Keep lead-in strictly inside verified overhang-free air when `avoid_model_overhangs` is active
    if let Some(av) = avoid_z {
        let c_in = p_arc_end + n_safe * r_h;
        let signed_sweep = if base_sign >= 0.0 {
            sweep_rad
        } else {
            -sweep_rad
        };
        let approach_ok = is_2d_segment_clear_of_avoid(approach_xy, p_arc_start, av);
        let arc_ok = r_h <= 1e-4
            || sweep_rad <= 1e-3
            || is_2d_arc_clear_of_avoid(p_arc_start, c_in, r_h, signed_sweep, av);
        let lin_ok = l_lin <= 1e-4 || is_2d_segment_clear_of_avoid(p_arc_end, entry_pt, av);
        if !(approach_ok && arc_ok && lin_ok) {
            builder.rapid_to(DVec3::new(entry_pt.x, entry_pt.y, retract_z));
            builder.rapid_to(DVec3::new(entry_pt.x, entry_pt.y, feed_z));
            builder.linear_to(
                DVec3::new(entry_pt.x, entry_pt.y, z_cut),
                ctx.cutting.plunge_feed_mm_min,
                MotionRole::Plunge,
            );
            return;
        }
    }
    let plunge_target_z = if r_v > 1e-4 {
        (z_cut + r_v).min(feed_z)
    } else {
        z_cut
    };

    builder.rapid_to(DVec3::new(approach_xy.x, approach_xy.y, retract_z));
    builder.rapid_to(DVec3::new(approach_xy.x, approach_xy.y, feed_z));
    builder.linear_to(
        DVec3::new(approach_xy.x, approach_xy.y, plunge_target_z),
        ctx.cutting.plunge_feed_mm_min,
        MotionRole::Plunge,
    );

    if r_v > 1e-4 && r_h <= 1e-4 {
        let dz = plunge_target_z - z_cut;
        if t.y.abs() <= 1e-4 && dz > 1e-4 {
            let sign_x = t.x.signum();
            let center_xz = DVec3::new(p_arc_start.x, p_arc_start.y, z_cut + r_v);
            let sweep = if sign_x > 0.0 {
                std::f64::consts::FRAC_PI_2
            } else {
                -std::f64::consts::FRAC_PI_2
            };
            builder.arc_vertical_plane_to(VerticalArcMove {
                plane: ArcPlane::XZ,
                end_wcs: DVec3::new(p_arc_start.x, p_arc_start.y, z_cut),
                center_wcs: center_xz,
                radius_mm: r_v,
                signed_sweep_rad: sweep,
                feed_mm_min: ctx.cutting.lead_feed_mm_min,
                role: MotionRole::LeadIn,
            });
        } else if t.x.abs() <= 1e-4 && dz > 1e-4 {
            let sign_y = t.y.signum();
            let center_yz = DVec3::new(p_arc_start.x, p_arc_start.y, z_cut + r_v);
            let sweep = if sign_y > 0.0 {
                std::f64::consts::FRAC_PI_2
            } else {
                -std::f64::consts::FRAC_PI_2
            };
            builder.arc_vertical_plane_to(VerticalArcMove {
                plane: ArcPlane::YZ,
                end_wcs: DVec3::new(p_arc_start.x, p_arc_start.y, z_cut),
                center_wcs: center_yz,
                radius_mm: r_v,
                signed_sweep_rad: sweep,
                feed_mm_min: ctx.cutting.lead_feed_mm_min,
                role: MotionRole::LeadIn,
            });
        } else {
            builder.linear_to(
                DVec3::new(p_arc_start.x, p_arc_start.y, z_cut),
                ctx.cutting.lead_feed_mm_min,
                MotionRole::LeadIn,
            );
        }
    }

    if r_h > 1e-4 && sweep_rad > 1e-3 {
        let c_in = p_arc_end + n_safe * r_h;
        builder.arc_xy_to(XyArcMove {
            end_wcs: DVec3::new(p_arc_end.x, p_arc_end.y, z_cut),
            center_xy: c_in,
            radius_mm: r_h,
            ccw: base_sign >= 0.0,
            turns: 0,
            feed_mm_min: ctx.cutting.lead_feed_mm_min,
            role: MotionRole::LeadIn,
        });
    }

    if l_lin > 1e-4 {
        builder.linear_to(
            DVec3::new(entry_pt.x, entry_pt.y, z_cut),
            ctx.cutting.lead_feed_mm_min,
            MotionRole::LeadIn,
        );
    }
}

fn emit_lead_out_and_retract(
    ctx: &StrategyExecContext<'_>,
    lead: &LeadConfig,
    builder: &mut ToolpathBuilder,
    spec: &LeadArcSpec<'_>,
) {
    let &LeadArcSpec {
        poly,
        anchor_pt: exit_pt,
        anchor_tan: exit_tan,
        base_sign,
        z_cut,
        retract_z,
        avoid_z,
    } = spec;
    if lead.enabled && base_sign != 0.0 {
        let aabb_sz = poly.exact_aabb().size();
        let max_safe_r = if poly.closed {
            (0.35 * aabb_sz.x.min(aabb_sz.y)).max(0.2)
        } else {
            50.0
        };
        let r_h = lead.horizontal_radius.value_mm.clamp(0.0, max_safe_r);
        let l_lin = lead.linear_length.value_mm.max(0.0);
        let sweep_rad = lead.horizontal_sweep_deg.clamp(0.0, 180.0).to_radians();

        let t = exit_tan.normalize_or_zero();
        let n_safe = if base_sign >= 0.0 {
            DVec2::new(-t.y, t.x)
        } else {
            DVec2::new(t.y, -t.x)
        };

        let p_arc_start = exit_pt + t * l_lin;
        let c_out = p_arc_start + n_safe * r_h;
        let signed_sweep = if base_sign >= 0.0 {
            sweep_rad
        } else {
            -sweep_rad
        };
        let lead_out_safe = avoid_z.is_none_or(|av| {
            let lin_ok = l_lin <= 1e-4 || is_2d_segment_clear_of_avoid(exit_pt, p_arc_start, av);
            let arc_ok = r_h <= 1e-4
                || sweep_rad <= 1e-3
                || is_2d_arc_clear_of_avoid(p_arc_start, c_out, r_h, signed_sweep, av);
            lin_ok && arc_ok
        });

        if lead_out_safe {
            if l_lin > 1e-4 {
                builder.linear_to(
                    DVec3::new(p_arc_start.x, p_arc_start.y, z_cut),
                    ctx.cutting.lead_feed_mm_min,
                    MotionRole::LeadOut,
                );
            }
            if r_h > 1e-4 && sweep_rad > 1e-3 {
                let p_arc_end = p_arc_start
                    + n_safe * (r_h * (1.0 - sweep_rad.cos()))
                    + t * (r_h * sweep_rad.sin());
                builder.arc_xy_to(XyArcMove {
                    end_wcs: DVec3::new(p_arc_end.x, p_arc_end.y, z_cut),
                    center_xy: c_out,
                    radius_mm: r_h,
                    ccw: base_sign >= 0.0,
                    turns: 0,
                    feed_mm_min: ctx.cutting.lead_feed_mm_min,
                    role: MotionRole::LeadOut,
                });
            }
        }
    }

    if let Some(cur) = builder.current_pos() {
        builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
    }
}
