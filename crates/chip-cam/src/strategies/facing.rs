use chip_math::{Aabb2, ArcSegment2D, DVec2, DVec3};

use crate::{
    cl_data::MotionRole,
    offset2d::{PocketRegion2D, clip_scanline_to_region},
    params::{FacingCutMode, FacingParams, MachiningBoundaryMode, ToolContainmentMode},
    rest2d::RegionSet2D,
    selection::ExtractedFeature25D,
    stock::{ResolvedStockGeometry, StockAxis},
    strategies::{StrategyExecContext, compute_z_levels},
    surface3d::{
        PushCutterField, SurfaceCutterProfile, SurfaceMesh3D, is_2d_arc_clear_of_avoid,
        region_aabb_2d, resolve_3d_containment_region_detailed, stock_xy_region,
    },
    toolpath::{ToolpathBuilder, XyArcMove},
    wcs::CamError,
};

struct FacingRowSpans {
    pv: f64,
    forward: bool,
    spans: Vec<(DVec2, DVec2)>,
}

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

    let boundary_offset = params.boundary.boundary_offset.value_mm;
    if !boundary_offset.is_finite() {
        return Err(CamError::InvalidParameter {
            param: "facing.boundary_offset",
            value: boundary_offset,
            reason: "must be finite",
        });
    }

    let pass_ext = params.pass_extension.value_mm.max(0.05 * tool_d);
    let radial_leave = ctx.passes.stock_to_leave_radial.value_mm.max(0.0);

    let is_classic_rect_stock = params.boundary.boundary_mode == MachiningBoundaryMode::StockBox
        && params.boundary.tool_containment == ToolContainmentMode::ToolOutsideBoundary
        && boundary_offset.abs() <= 1e-6
        && !matches!(
            ctx.setup.stock,
            ResolvedStockGeometry::Cylinder {
                axis: StockAxis::Z,
                ..
            }
        );

    // Resolve polygon regions when using general MachiningBoundaryConfig (SelectedRegions,
    // ModelSilhouette, custom containment/offset, or cylindrical Z-axis stock).
    let (
        footprint,
        clip_outer_opt,
        lane_domain_opt,
        island_avoid,
        u_ext_per_span,
        use_exact_center_lanes,
    ) = if is_classic_rect_stock {
        let s3 = ctx.setup.stock.aabb_wcs();
        let fp = Aabb2 {
            min: DVec2::new(s3.min.x, s3.min.y),
            max: DVec2::new(s3.max.x, s3.max.y),
        };
        (
            fp,
            None,
            None,
            RegionSet2D::empty(),
            tool_r + pass_ext,
            false,
        )
    } else {
        let (nominal_outer, raised_islands) =
            resolve_facing_nominal_and_islands(ctx, params, tool_r, builder);
        if nominal_outer.is_empty() {
            return Err(CamError::EmptyToolpath {
                op_name: ctx.op.name.clone(),
            });
        }

        let nominal_with_offset = offset_region_set(&nominal_outer, boundary_offset);
        let (lane_domain, clip_outer, span_ext, exact_center_lanes) =
            match params.boundary.tool_containment {
                ToolContainmentMode::ToolOutsideBoundary => (
                    nominal_with_offset.clone(),
                    nominal_with_offset.dilate_outward(tool_r),
                    pass_ext,
                    false,
                ),
                ToolContainmentMode::ToolCenterOnBoundary => {
                    (nominal_with_offset.clone(), nominal_with_offset, 0.0, true)
                }
                ToolContainmentMode::ToolInsideBoundary => {
                    let eroded = offset_region_set(&nominal_outer, boundary_offset - tool_r);
                    (eroded.clone(), eroded, 0.0, true)
                }
            };

        if clip_outer.is_empty() || lane_domain.is_empty() {
            return Err(CamError::EmptyToolpath {
                op_name: ctx.op.name.clone(),
            });
        }

        let island_av = if raised_islands.is_empty() {
            RegionSet2D::empty()
        } else {
            raised_islands.dilate_outward(tool_r + radial_leave)
        };

        let Some(fp) = region_aabb_2d(&lane_domain).filter(Aabb2::is_valid) else {
            return Err(CamError::EmptyToolpath {
                op_name: ctx.op.name.clone(),
            });
        };

        (
            fp,
            Some(clip_outer),
            Some(lane_domain),
            island_av,
            span_ext,
            exact_center_lanes,
        )
    };

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

    // Project corners (or exact region vertices when polygon-bounded) onto `(u_axis, v_axis)`.
    let (u_min, u_max, v_min, v_max) = if let Some(ref lane_dom) = lane_domain_opt {
        project_region_uv_bounds(lane_dom, u_axis, v_axis)
    } else {
        project_aabb_uv_bounds(footprint, u_axis, v_axis)
    };

    let (clip_u_min, clip_u_max) = if let Some(ref clip_reg) = clip_outer_opt {
        let (cu0, cu1, _, _) = project_region_uv_bounds(clip_reg, u_axis, v_axis);
        (cu0, cu1)
    } else {
        (u_min, u_max)
    };

    let build_pass_vs = |vmin: f64, vmax: f64| -> Vec<f64> {
        let sv = (vmax - vmin).max(0.0);
        if use_exact_center_lanes {
            if sv <= 0.1 {
                return vec![0.5 * (vmin + vmax)];
            }
            let inset = (0.05_f64).min(0.25 * sv);
            let first_v = vmin + inset;
            let last_v = vmax - inset;
            if last_v <= first_v + 1e-6 {
                vec![0.5 * (vmin + vmax)]
            } else {
                let count = ((last_v - first_v) / stepover).ceil().max(1.0) as usize;
                let actual_step = (last_v - first_v) / (count as f64);
                (0..=count)
                    .map(|i| first_v + actual_step * (i as f64))
                    .collect()
            }
        } else if sv <= 2.0 * (tool_r - 0.05).max(0.0) {
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

    // Extend along `u` by `tool_r + pass_ext` so the full cutter body starts & ends in air outside stock!
    let u_start_ext = u_min - (tool_r + pass_ext);
    let u_end_ext = u_max + (tool_r + pass_ext);
    let pass_vs = build_pass_vs(v_min, v_max);

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

        let total_avoid: Option<RegionSet2D> = match (island_avoid.is_empty(), &overhang_avoid) {
            (true, None) => None,
            (true, Some(ov)) => Some(ov.clone()),
            (false, None) => Some(island_avoid.clone()),
            (false, Some(ov)) => Some(island_avoid.union(ov)),
        };

        // Precompute exact clipped spans for every row at this Z level.
        let scan_u_lo = clip_u_min.min(lvl_u_start_ext) - tool_r - pass_ext - 2.0;
        let scan_u_hi = clip_u_max.max(lvl_u_end_ext) + tool_r + pass_ext + 2.0;
        let mut row_spans_all: Vec<FacingRowSpans> = Vec::with_capacity(ordered_vs.len());

        for (i, &pv) in ordered_vs.iter().enumerate() {
            ctx.check_cancel()?;

            let forward = match params.cut_mode {
                FacingCutMode::BothWaysZigZag => i % 2 == 0,
                FacingCutMode::OneWayClimb | FacingCutMode::OneWayConventional => true,
            };

            let mut u_intervals: Vec<(f64, f64)> = if let Some(ref clip_reg) = clip_outer_opt {
                let mut spans = clip_scanline_to_region_set_u(
                    to_xy(scan_u_lo, pv),
                    to_xy(scan_u_hi, pv),
                    u_axis,
                    clip_reg,
                );
                if u_ext_per_span > 1e-5 {
                    for s in &mut spans {
                        s.0 -= u_ext_per_span;
                        s.1 += u_ext_per_span;
                    }
                    spans = merge_u_intervals(spans);
                }
                spans
            } else {
                vec![(lvl_u_start_ext, lvl_u_end_ext)]
            };

            if let Some(ref avoid) = total_avoid
                && !u_intervals.is_empty()
            {
                let avoid_u = clip_scanline_to_region_set_u(
                    to_xy(scan_u_lo, pv),
                    to_xy(scan_u_hi, pv),
                    u_axis,
                    avoid,
                );
                if !avoid_u.is_empty() {
                    u_intervals = subtract_u_intervals(&u_intervals, &avoid_u);
                }
            }

            let mut spans: Vec<(DVec2, DVec2)> = u_intervals
                .into_iter()
                .filter(|&(a, b)| b - a > 0.05)
                .map(|(a, b)| {
                    if forward {
                        (to_xy(a, pv), to_xy(b, pv))
                    } else {
                        (to_xy(b, pv), to_xy(a, pv))
                    }
                })
                .collect();
            if !forward {
                spans.reverse();
            }

            row_spans_all.push(FacingRowSpans { pv, forward, spans });
        }

        for (i, row) in row_spans_all.iter().enumerate() {
            ctx.check_cancel()?;
            let row_spans = &row.spans;
            if row_spans.is_empty() {
                continue;
            }
            let pv = row.pv;
            let forward = row.forward;

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

                // Smooth U-turn `G2`/`G3` linking arc outside the stock in BothWaysZigZag mode
                // whenever the next row starts at the exact matching U station and the arc is clear!
                if span_idx + 1 == row_spans.len()
                    && params.cut_mode == FacingCutMode::BothWaysZigZag
                    && let Some(next_row) = row_spans_all.get(i + 1)
                    && let Some(&(p_next_start, _)) = next_row.spans.first()
                    && (next_row.pv - pv).abs() <= stepover * 1.5 + 1e-4
                    && (p_next_start.dot(u_axis) - span_end.dot(u_axis)).abs() <= 1e-3
                {
                    let next_pv = next_row.pv;
                    let mid_center = 0.5 * (span_end + p_next_start);
                    let link_r = 0.5 * span_end.distance(p_next_start);
                    let ccw = if forward { next_pv > pv } else { next_pv < pv };
                    let sweep = if ccw {
                        std::f64::consts::PI
                    } else {
                        -std::f64::consts::PI
                    };
                    let arc_clear = total_avoid.as_ref().is_none_or(|av| {
                        !av.contains_point(p_next_start)
                            && is_2d_arc_clear_of_avoid(span_end, mid_center, link_r, sweep, av)
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

fn resolve_facing_nominal_and_islands(
    ctx: &StrategyExecContext<'_>,
    params: &FacingParams,
    tool_r: f64,
    builder: &mut ToolpathBuilder,
) -> (RegionSet2D, RegionSet2D) {
    match params.boundary.boundary_mode {
        MachiningBoundaryMode::StockBox => (stock_xy_region(ctx.setup), RegionSet2D::empty()),
        MachiningBoundaryMode::ModelSilhouette => {
            if ctx.doc.has_3d_geometry() {
                let surface = SurfaceMesh3D::from_cad_document(ctx.doc, &[], &ctx.setup.frame);
                if !surface.triangles.is_empty() && surface.aabb_wcs.is_valid() {
                    let resolved = resolve_3d_containment_region_detailed(
                        &surface,
                        ctx.setup,
                        &ctx.selection,
                        MachiningBoundaryMode::ModelSilhouette,
                        ToolContainmentMode::ToolCenterOnBoundary,
                        0.0,
                        tool_r,
                    );
                    builder.warnings.extend(resolved.warnings);
                    return (resolved.nominal_region, RegionSet2D::empty());
                }
            }
            (stock_xy_region(ctx.setup), RegionSet2D::empty())
        }
        MachiningBoundaryMode::SelectedRegions => {
            let mut outer_sets = Vec::new();
            let mut island_sets = Vec::new();
            for feat in &ctx.selection.features {
                match feat {
                    ExtractedFeature25D::PlanarFace {
                        outer_boundary_wcs,
                        raised_islands_wcs,
                        ..
                    } => {
                        if outer_boundary_wcs.closed && !outer_boundary_wcs.segments.is_empty() {
                            let outer = outer_boundary_wcs.ensure_orientation(true);
                            outer_sets.push(RegionSet2D::from_single(PocketRegion2D::new(
                                outer,
                                Vec::new(),
                            )));
                        }
                        for isl in raised_islands_wcs {
                            if isl.closed && !isl.segments.is_empty() {
                                let ccw = isl.ensure_orientation(true);
                                island_sets.push(RegionSet2D::from_single(PocketRegion2D::new(
                                    ccw,
                                    Vec::new(),
                                )));
                            }
                        }
                    }
                    ExtractedFeature25D::Contour { polyline_wcs, .. }
                        if polyline_wcs.closed && !polyline_wcs.segments.is_empty() =>
                    {
                        let outer = polyline_wcs.ensure_orientation(true);
                        outer_sets.push(RegionSet2D::from_single(PocketRegion2D::new(
                            outer,
                            Vec::new(),
                        )));
                    }
                    _ => {}
                }
            }

            if outer_sets.is_empty() {
                builder.warnings.push(
                    "Machining boundary is set to 'Selected 2D Contours' but no closed 2D contours or planar faces are selected; falling back to Stock Bounds."
                        .to_string(),
                );
                return (stock_xy_region(ctx.setup), RegionSet2D::empty());
            }

            let nominal_outer = RegionSet2D::union_many(&mut outer_sets);
            let raised_islands = RegionSet2D::union_many(&mut island_sets);
            (nominal_outer, raised_islands)
        }
    }
}

fn offset_region_set(reg: &RegionSet2D, delta_mm: f64) -> RegionSet2D {
    if delta_mm > 1e-5 {
        reg.dilate_outward(delta_mm)
    } else if delta_mm < -1e-5 {
        reg.erode_inward(-delta_mm)
    } else {
        reg.clone()
    }
}

fn project_aabb_uv_bounds(aabb: Aabb2, u_axis: DVec2, v_axis: DVec2) -> (f64, f64, f64, f64) {
    let corners = [
        aabb.min,
        DVec2::new(aabb.max.x, aabb.min.y),
        aabb.max,
        DVec2::new(aabb.min.x, aabb.max.y),
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
    (u_min, u_max, v_min, v_max)
}

fn project_region_uv_bounds(
    region: &RegionSet2D,
    u_axis: DVec2,
    v_axis: DVec2,
) -> (f64, f64, f64, f64) {
    let mut u_min = f64::INFINITY;
    let mut u_max = f64::NEG_INFINITY;
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    let mut include_pt = |p: DVec2| {
        let pu = p.dot(u_axis);
        let pv = p.dot(v_axis);
        u_min = u_min.min(pu);
        u_max = u_max.max(pu);
        v_min = v_min.min(pv);
        v_max = v_max.max(pv);
    };

    for reg in &region.regions {
        for seg in &reg.outer_ccw.segments {
            match *seg {
                ArcSegment2D::Line { start, end } => {
                    include_pt(start);
                    include_pt(end);
                }
                ArcSegment2D::Arc { .. } => {
                    for k in 0..=16 {
                        if let Some((pt, _)) = seg.eval_at(f64::from(k) / 16.0) {
                            include_pt(pt);
                        }
                    }
                }
            }
        }
    }

    if !u_min.is_finite() {
        region_aabb_2d(region)
            .filter(Aabb2::is_valid)
            .map_or((0.0, 0.0, 0.0, 0.0), |b| {
                project_aabb_uv_bounds(b, u_axis, v_axis)
            })
    } else {
        (u_min, u_max, v_min, v_max)
    }
}

fn clip_scanline_to_region_set_u(
    p0: DVec2,
    p1: DVec2,
    u_axis: DVec2,
    set: &RegionSet2D,
) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    for reg in &set.regions {
        for (a, b) in clip_scanline_to_region(p0, p1, reg) {
            let ua = a.dot(u_axis);
            let ub = b.dot(u_axis);
            let (lo, hi) = if ua <= ub { (ua, ub) } else { (ub, ua) };
            if hi - lo > 1e-5 {
                out.push((lo, hi));
            }
        }
    }
    merge_u_intervals(out)
}

fn merge_u_intervals(mut intervals: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    if intervals.len() <= 1 {
        return intervals;
    }
    intervals.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut merged: Vec<(f64, f64)> = Vec::with_capacity(intervals.len());
    for (a, b) in intervals {
        if let Some(last) = merged.last_mut()
            && a <= last.1 + 1e-4
        {
            last.1 = last.1.max(b);
            continue;
        }
        merged.push((a, b));
    }
    merged
}

fn subtract_u_intervals(base: &[(f64, f64)], avoid: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut cur: Vec<(f64, f64)> = base.to_vec();
    for &(av_lo, av_hi) in avoid {
        let mut next = Vec::with_capacity(cur.len());
        for (a, b) in cur {
            if av_hi <= a + 1e-6 || av_lo >= b - 1e-6 {
                next.push((a, b));
            } else {
                if av_lo > a + 1e-5 {
                    next.push((a, av_lo));
                }
                if av_hi < b - 1e-5 {
                    next.push((av_hi, b));
                }
            }
        }
        cur = next;
    }
    cur
}
