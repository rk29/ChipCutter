use std::sync::atomic::{AtomicBool, Ordering};

use chip_math::{ArcPolyline2D, DVec2, DVec3};

use crate::{
    avoid_regions::clip_polyline_outside_avoid,
    cl_data::MotionRole,
    offset2d::{
        OffsetRegionNode, PocketRegion2D, build_offset_region_tree, clip_scanline_to_region,
        find_helical_entry_pole,
    },
    params::{
        EntryStrategy, Pocket2DParams, PocketBoundaryMode, PocketPassOrder, PocketStrategyKind,
    },
    rest2d::{
        ContourRestClipQuery, PocketRestQuery, RegionSet2D, clip_contour_path_to_rest,
        compute_rest_pocket_work_items,
    },
    selection::ExtractedFeature25D,
    strategies::{
        StrategyExecContext,
        adaptive2d::{AdaptiveSliceSpec, generate_adaptive_region_slice},
        compute_z_levels,
    },
    surface3d::{
        PushCutterField, SurfaceCutterProfile, SurfaceMesh3D, clip_2d_contour_outside_overhangs,
        is_2d_segment_clear_of_avoid, stock_xy_region,
    },
    toolpath::{ToolpathBuilder, XyArcMove},
    wcs::CamError,
};

/// 2D clearing pass produced by a [`PocketStrategy`] at a single Z level (`05-operations.md` §3).
#[derive(Debug, Clone)]
pub struct PocketSubpath2D {
    pub polyline: ArcPolyline2D,
    /// Optional collision-free sub-region from which a helical/linear entry should be computed
    /// when a stay-down transition from the previous pass is not possible.
    pub entry_region: Option<PocketRegion2D>,
    pub prefer_air_entry: bool,
    pub role: MotionRole,
}

/// Parameters supplied to [`PocketStrategy::generate_level_passes`] (`05-operations.md` §3).
/// Kept clean and strategy-neutral so constant-engagement Adaptive/Trochoidal clearing slots in seamlessly in v2.
#[derive(Debug, Clone, Copy)]
pub struct PocketEngagementParams {
    pub tool_radius_mm: f64,
    pub stepover_mm: f64,
    pub inside_out: bool,
    pub has_open_air_edges: bool,
}

/// Pluggable 2D pocket clearing strategy trait (`05-operations.md` §3).
pub trait PocketStrategy: Send + Sync {
    fn generate_level_passes(
        &self,
        eroded_region: &PocketRegion2D,
        raw_floor_outer_ccw: &ArcPolyline2D,
        engagement: &PocketEngagementParams,
        cancel: &AtomicBool,
    ) -> Result<Vec<PocketSubpath2D>, CamError>;
}

/// Strategy 1: **Contour-Parallel Offset** over an [`OffsetRegionNode`] tree (`05-operations.md` §3).
pub struct ContourParallelStrategy;

impl PocketStrategy for ContourParallelStrategy {
    fn generate_level_passes(
        &self,
        eroded_region: &PocketRegion2D,
        raw_floor_outer_ccw: &ArcPolyline2D,
        engagement: &PocketEngagementParams,
        cancel: &AtomicBool,
    ) -> Result<Vec<PocketSubpath2D>, CamError> {
        if cancel.load(Ordering::Relaxed) {
            return Err(CamError::Cancelled);
        }
        let tree = build_offset_region_tree(eroded_region, engagement.stepover_mm, 256);
        let mut passes = Vec::new();
        if engagement.inside_out {
            collect_tree_inside_out(&tree, &mut passes, cancel)?;
        } else {
            collect_tree_outside_in(
                &tree,
                raw_floor_outer_ccw,
                engagement.has_open_air_edges,
                &mut passes,
                cancel,
            )?;
        }
        Ok(passes)
    }
}

fn collect_tree_inside_out(
    node: &OffsetRegionNode,
    out: &mut Vec<PocketSubpath2D>,
    cancel: &AtomicBool,
) -> Result<(), CamError> {
    if cancel.load(Ordering::Relaxed) {
        return Err(CamError::Cancelled);
    }
    let is_leaf = node.children.is_empty();
    for child in &node.children {
        collect_tree_inside_out(child, out, cancel)?;
    }

    out.push(PocketSubpath2D {
        polyline: node.region.outer_ccw.clone(),
        entry_region: if is_leaf {
            Some(node.region.clone())
        } else {
            None
        },
        prefer_air_entry: false,
        role: MotionRole::Cut,
    });
    for isl in &node.region.islands_cw {
        out.push(PocketSubpath2D {
            polyline: isl.clone(),
            entry_region: None,
            prefer_air_entry: false,
            role: MotionRole::Cut,
        });
    }
    Ok(())
}

fn collect_tree_outside_in(
    node: &OffsetRegionNode,
    raw_floor_outer_ccw: &ArcPolyline2D,
    has_open_air_edges: bool,
    out: &mut Vec<PocketSubpath2D>,
    cancel: &AtomicBool,
) -> Result<(), CamError> {
    if cancel.load(Ordering::Relaxed) {
        return Err(CamError::Cancelled);
    }
    let is_root_open = node.depth_index == 0 && has_open_air_edges;
    let outer_poly = if is_root_open {
        rotate_seam_to_air_overhang(&node.region.outer_ccw, raw_floor_outer_ccw)
    } else {
        node.region.outer_ccw.clone()
    };

    out.push(PocketSubpath2D {
        polyline: outer_poly,
        entry_region: if node.depth_index == 0 && !has_open_air_edges {
            Some(node.region.clone())
        } else {
            None
        },
        prefer_air_entry: is_root_open,
        role: MotionRole::Cut,
    });
    for isl in &node.region.islands_cw {
        out.push(PocketSubpath2D {
            polyline: isl.clone(),
            entry_region: None,
            prefer_air_entry: false,
            role: MotionRole::Cut,
        });
    }

    for child in &node.children {
        collect_tree_outside_in(child, raw_floor_outer_ccw, has_open_air_edges, out, cancel)?;
    }
    Ok(())
}

/// Strategy 2: **Zig-Zag / Raster Clearing** + Boundary Cleanup (`05-operations.md` §3).
pub struct ZigZagPocketStrategy {
    pub pass_angle_deg: f64,
}

impl PocketStrategy for ZigZagPocketStrategy {
    fn generate_level_passes(
        &self,
        eroded_region: &PocketRegion2D,
        raw_floor_outer_ccw: &ArcPolyline2D,
        engagement: &PocketEngagementParams,
        cancel: &AtomicBool,
    ) -> Result<Vec<PocketSubpath2D>, CamError> {
        let aabb = eroded_region.exact_aabb();
        if !aabb.is_valid() {
            return Ok(Vec::new());
        }
        let angle = self.pass_angle_deg.to_radians();
        let u = DVec2::new(angle.cos(), angle.sin());
        let v = DVec2::new(-angle.sin(), angle.cos());

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
            u_min = u_min.min(c.dot(u));
            u_max = u_max.max(c.dot(u));
            v_min = v_min.min(c.dot(v));
            v_max = v_max.max(c.dot(v));
        }

        let step = engagement.stepover_mm.max(0.05);
        let span_v = (v_max - v_min).max(0.0);
        let n_lines = (span_v / step).ceil().max(1.0) as usize;
        let dv = if n_lines > 0 {
            span_v / (n_lines as f64)
        } else {
            0.0
        };

        // Leave a slight inset from `eroded_region` for the perimeter cleanup pass so scanline ends blend cleanly
        let mut raw_segments: Vec<Vec<(DVec2, DVec2)>> = Vec::with_capacity(n_lines + 1);
        for i in 0..=n_lines {
            if cancel.load(Ordering::Relaxed) {
                return Err(CamError::Cancelled);
            }
            let sv = (v_min + dv * (i as f64)).clamp(v_min + 1e-4, v_max - 1e-4);
            let p0 = u * (u_min - 2.0) + v * sv;
            let p1 = u * (u_max + 2.0) + v * sv;
            let mut spans = clip_scanline_to_region(p0, p1, eroded_region);
            if i % 2 == 1 {
                spans.reverse();
                for s in &mut spans {
                    std::mem::swap(&mut s.0, &mut s.1);
                }
            }
            if !spans.is_empty() {
                raw_segments.push(spans);
            }
        }

        // Chain consecutive scanline segments into continuous bi-directional polylines whenever the stay-down link stays inside `eroded_region`
        let mut passes: Vec<PocketSubpath2D> = Vec::new();
        let mut visited: Vec<Vec<bool>> = raw_segments
            .iter()
            .map(|row| vec![false; row.len()])
            .collect();

        for r in 0..raw_segments.len() {
            for c in 0..raw_segments[r].len() {
                if visited[r][c] {
                    continue;
                }
                visited[r][c] = true;
                let (mut cur_s, mut cur_e) = raw_segments[r][c];
                let mut chain_segs = vec![chip_math::ArcSegment2D::Line {
                    start: cur_s,
                    end: cur_e,
                }];

                let mut cur_row = r;
                while cur_row + 1 < raw_segments.len() {
                    let next_row = cur_row + 1;
                    // Find closest unvisited span in `next_row` within `2.2 * step` whose chord stays inside `eroded_region`
                    let mut best_col: Option<(usize, bool, f64)> = None;
                    for (nc, &(ns, ne)) in raw_segments[next_row].iter().enumerate() {
                        if visited[next_row][nc] {
                            continue;
                        }
                        for (flip, cand_start) in [(false, ns), (true, ne)] {
                            let d = cur_e.distance(cand_start);
                            if d <= 2.5 * step
                                && is_chord_inside_region(cur_e, cand_start, eroded_region)
                                && best_col.is_none_or(|(_, _, prev_d)| d < prev_d)
                            {
                                best_col = Some((nc, flip, d));
                            }
                        }
                    }
                    if let Some((nc, flip, d)) = best_col {
                        visited[next_row][nc] = true;
                        let (ns, ne) = raw_segments[next_row][nc];
                        let (next_s, next_e) = if flip { (ne, ns) } else { (ns, ne) };
                        if d > 1e-5 {
                            chain_segs.push(chip_math::ArcSegment2D::Line {
                                start: cur_e,
                                end: next_s,
                            });
                        }
                        chain_segs.push(chip_math::ArcSegment2D::Line {
                            start: next_s,
                            end: next_e,
                        });
                        cur_s = next_s;
                        cur_e = next_e;
                        cur_row = next_row;
                    } else {
                        break;
                    }
                }
                let _ = cur_s;
                let first_pt = chain_segs[0].start();
                let prefer_air =
                    engagement.has_open_air_edges && !raw_floor_outer_ccw.contains_point(first_pt);
                passes.push(PocketSubpath2D {
                    polyline: ArcPolyline2D {
                        segments: chain_segs,
                        closed: false,
                    },
                    entry_region: if passes.is_empty() && !prefer_air {
                        Some(eroded_region.clone())
                    } else {
                        None
                    },
                    prefer_air_entry: prefer_air,
                    role: MotionRole::Cut,
                });
            }
        }

        // Perimeter cleanup pass around outer boundary + every island!
        passes.push(PocketSubpath2D {
            polyline: if engagement.has_open_air_edges {
                rotate_seam_to_air_overhang(&eroded_region.outer_ccw, raw_floor_outer_ccw)
            } else {
                eroded_region.outer_ccw.clone()
            },
            entry_region: None,
            prefer_air_entry: engagement.has_open_air_edges,
            role: MotionRole::Cut,
        });
        for isl in &eroded_region.islands_cw {
            passes.push(PocketSubpath2D {
                polyline: isl.clone(),
                entry_region: None,
                prefer_air_entry: false,
                role: MotionRole::Cut,
            });
        }

        Ok(passes)
    }
}

/// Generates a **2D Pocket** toolpath (`05-operations.md` §3).
pub fn generate_pocket2d(
    ctx: &StrategyExecContext<'_>,
    params: &Pocket2DParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    let tool_d = ctx.tool.geometry.cutting_diameter_mm();
    let tool_r = 0.5 * tool_d;
    let stepover = params.stepover.value_mm;
    if !stepover.is_finite() || stepover <= 0.02 || stepover > tool_d {
        return Err(CamError::InvalidParameter {
            param: "pocket_2d.stepover",
            value: stepover,
            reason: "must be > 0.02 mm and <= tool diameter",
        });
    }

    let radial_leave = ctx.passes.stock_to_leave_radial.value_mm;
    let target_bottom_z = ctx.heights.bottom_z_wcs + ctx.passes.stock_to_leave_axial.value_mm;
    let z_levels = compute_z_levels(
        ctx.heights.top_z_wcs,
        target_bottom_z,
        ctx.passes.multiple_depths,
        ctx.passes.max_stepdown.value_mm,
        ctx.passes.even_stepdowns,
    )?;

    let finish_allowance = if params.finish_wall_pass {
        params.finish_wall_stepover.value_mm.max(0.0)
    } else {
        0.0
    };
    let is_adaptive = matches!(params.strategy, PocketStrategyKind::Adaptive(_));

    // `(clearing_region, raw_floor_outer_ccw, has_open_air_edges, skip_outer_finish_pass)`
    let mut raw_pockets: Vec<(PocketRegion2D, ArcPolyline2D, bool, bool)> = Vec::new();
    match params.boundary_mode {
        PocketBoundaryMode::SelectedPockets => {
            for feat in &ctx.selection.features {
                match feat {
                    ExtractedFeature25D::PlanarFace {
                        outer_boundary_wcs,
                        open_outer_segments,
                        raised_islands_wcs,
                        ..
                    } => {
                        let outer_ccw = outer_boundary_wcs.ensure_orientation(true);
                        if params.treat_outer_boundary_as_open {
                            let open_shift = 2.0 * tool_r
                                + radial_leave
                                + finish_allowance
                                + params.open_edge_overhang.value_mm.max(0.5);
                            let mut expanded = RegionSet2D::from_single(PocketRegion2D::new(
                                outer_ccw.clone(),
                                Vec::new(),
                            ))
                            .dilate_outward(open_shift);
                            if !raised_islands_wcs.is_empty() {
                                let mut isl_sets: Vec<RegionSet2D> = raised_islands_wcs
                                    .iter()
                                    .filter(|i| i.closed && !i.segments.is_empty())
                                    .map(|i| {
                                        RegionSet2D::from_single(PocketRegion2D::new(
                                            i.ensure_orientation(true),
                                            Vec::new(),
                                        ))
                                    })
                                    .collect();
                                let isl_union = RegionSet2D::union_many(&mut isl_sets);
                                if !isl_union.is_empty() {
                                    expanded = expanded.difference(&isl_union);
                                }
                            }
                            for reg in expanded.regions {
                                raw_pockets.push((reg, outer_ccw.clone(), true, true));
                            }
                        } else {
                            let has_open = open_outer_segments.iter().any(|&b| b);
                            let open_shift = if has_open {
                                let stock_aabb = ctx.setup.stock.aabb_wcs();
                                let feat_aabb = outer_boundary_wcs.exact_aabb();
                                let extra_stock_margin =
                                    if is_adaptive && stock_aabb.is_valid() && feat_aabb.is_valid()
                                    {
                                        (stock_aabb.max.x - feat_aabb.max.x)
                                            .max(feat_aabb.min.x - stock_aabb.min.x)
                                            .max(stock_aabb.max.y - feat_aabb.max.y)
                                            .max(feat_aabb.min.y - stock_aabb.min.y)
                                            .clamp(0.0, 25.0)
                                    } else {
                                        0.0
                                    };
                                let base_r = if is_adaptive {
                                    2.0 * tool_r + extra_stock_margin
                                } else {
                                    tool_r
                                };
                                base_r + radial_leave + params.open_edge_overhang.value_mm.max(0.2)
                            } else {
                                0.0
                            };
                            let region = PocketRegion2D::from_face_with_open_extensions(
                                outer_boundary_wcs,
                                open_outer_segments,
                                raised_islands_wcs,
                                open_shift,
                            );
                            raw_pockets.push((region, outer_ccw, has_open, false));
                        }
                    }
                    ExtractedFeature25D::Contour { polyline_wcs, .. } if polyline_wcs.closed => {
                        let ccw = polyline_wcs.ensure_orientation(true);
                        if params.treat_outer_boundary_as_open {
                            let open_shift = 2.0 * tool_r
                                + radial_leave
                                + finish_allowance
                                + params.open_edge_overhang.value_mm.max(0.5);
                            let expanded = RegionSet2D::from_single(PocketRegion2D::new(
                                ccw.clone(),
                                Vec::new(),
                            ))
                            .dilate_outward(open_shift);
                            for reg in expanded.regions {
                                raw_pockets.push((reg, ccw.clone(), true, true));
                            }
                        } else {
                            raw_pockets.push((
                                PocketRegion2D::new(ccw.clone(), Vec::new()),
                                ccw,
                                false,
                                false,
                            ));
                        }
                    }
                    _ => {}
                }
            }
        }
        PocketBoundaryMode::ClearOutsideSelectedToStock => {
            let mut boss_sets: Vec<RegionSet2D> = Vec::new();
            for feat in &ctx.selection.features {
                match feat {
                    ExtractedFeature25D::PlanarFace {
                        outer_boundary_wcs, ..
                    } if outer_boundary_wcs.closed && !outer_boundary_wcs.segments.is_empty() => {
                        boss_sets.push(RegionSet2D::from_single(PocketRegion2D::new(
                            outer_boundary_wcs.ensure_orientation(true),
                            Vec::new(),
                        )));
                    }
                    ExtractedFeature25D::Contour { polyline_wcs, .. }
                        if polyline_wcs.closed && !polyline_wcs.segments.is_empty() =>
                    {
                        boss_sets.push(RegionSet2D::from_single(PocketRegion2D::new(
                            polyline_wcs.ensure_orientation(true),
                            Vec::new(),
                        )));
                    }
                    _ => {}
                }
            }
            if boss_sets.is_empty() {
                return Err(CamError::EmptySelection {
                    op_name: ctx.op.name.clone(),
                    op_kind: ctx.op.kind.label(),
                });
            }
            let boss_union = RegionSet2D::union_many(&mut boss_sets);
            let raw_stock = stock_xy_region(ctx.setup);
            if let Some(raw_stock_outer) = raw_stock.regions.first().map(|r| r.outer_ccw.clone()) {
                let open_shift = 2.0 * tool_r
                    + radial_leave
                    + finish_allowance
                    + params.open_edge_overhang.value_mm.max(0.5);
                let expanded_stock = match ctx.setup.stock {
                    crate::stock::ResolvedStockGeometry::Cylinder {
                        axis: crate::stock::StockAxis::Z,
                        center_wcs,
                        radius_mm,
                        ..
                    } => RegionSet2D::from_circle(
                        DVec2::new(center_wcs.x, center_wcs.y),
                        radius_mm + open_shift,
                    ),
                    crate::stock::ResolvedStockGeometry::Box { aabb_wcs }
                    | crate::stock::ResolvedStockGeometry::Cylinder { aabb_wcs, .. } => {
                        crate::surface3d::rect_region_2d(
                            aabb_wcs.min.x - open_shift,
                            aabb_wcs.min.y - open_shift,
                            aabb_wcs.max.x + open_shift,
                            aabb_wcs.max.y + open_shift,
                        )
                    }
                };
                let clear_set = expanded_stock.difference(&boss_union);
                for reg in clear_set.regions {
                    raw_pockets.push((reg, raw_stock_outer.clone(), true, true));
                }
            }
        }
    }

    if raw_pockets.is_empty() {
        return Err(CamError::EmptySelection {
            op_name: ctx.op.name.clone(),
            op_kind: ctx.op.kind.label(),
        });
    }

    let legacy_strategy_impl: Option<Box<dyn PocketStrategy>> = match &params.strategy {
        PocketStrategyKind::ContourParallel => Some(Box::new(ContourParallelStrategy)),
        PocketStrategyKind::ZigZag { pass_angle_deg } => Some(Box::new(ZigZagPocketStrategy {
            pass_angle_deg: *pass_angle_deg,
        })),
        PocketStrategyKind::Adaptive(_) => None,
    };

    let d_finish = (tool_r + radial_leave).max(0.01);
    let d_rough = d_finish + finish_allowance;

    let mut any_valid_region = false;
    let retract_z = ctx.safe_retract_z();
    let stay_down_limit = ctx
        .linking
        .safe_stay_down_distance
        .value_mm
        .max(stepover * 1.6);

    let avoid_2d = ctx.resolve_avoid_2d(target_bottom_z, ctx.heights.top_z_wcs);
    builder.warnings.extend(avoid_2d.warnings);
    let avoid_keepout = avoid_2d.effective_region;
    let has_avoid_keepout = !avoid_keepout.is_empty();

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

    for (pkt_region, raw_floor_outer_ccw, has_open, skip_outer_finish) in &raw_pockets {
        ctx.check_cancel()?;

        let finish_regions = pkt_region.erode_inward(d_finish);
        if finish_regions.is_empty() {
            continue;
        }
        any_valid_region = true;

        let rough_regions = pkt_region.erode_inward(d_rough);
        let effective_rough = if rough_regions.is_empty() {
            &finish_regions
        } else {
            &rough_regions
        };

        let inside_out = match params.pass_order {
            PocketPassOrder::InsideOut => true,
            PocketPassOrder::OutsideIn => false,
            PocketPassOrder::Auto => !*has_open,
        };

        let engagement = PocketEngagementParams {
            tool_radius_mm: tool_r,
            stepover_mm: stepover,
            inside_out,
            has_open_air_edges: *has_open,
        };

        for (lvl_idx, &z_cut) in z_levels.iter().enumerate() {
            ctx.check_cancel()?;
            let prev_level_z = if lvl_idx == 0 {
                ctx.heights.top_z_wcs
            } else {
                z_levels[lvl_idx - 1]
            };

            // Multi-setup carried-over stock rest filtering: skip Z levels with no remaining
            // solid material and clip clearing regions to carried-over stock + safe overlap!
            let carryover_masks: Option<(RegionSet2D, RegionSet2D)> =
                if let Some(td) = ctx.carryover_stock {
                    let slab_lo = z_cut + 0.01;
                    let slab_hi = prev_level_z.max(z_cut + 0.02) + 0.05;
                    if !td.has_material_in_z_slab(slab_lo, slab_hi, 0.02) {
                        continue;
                    }
                    let mat_polys = td.extract_remaining_material_regions_in_z_slab(
                        slab_lo,
                        slab_hi,
                        tool_r + radial_leave + 0.5,
                    );
                    let mat_mask = crate::rest2d::contours_to_region_set(&mat_polys);
                    if mat_mask.is_empty() {
                        continue;
                    }
                    let tight_polys = td.extract_remaining_material_regions_in_z_slab(
                        slab_lo,
                        slab_hi,
                        tool_r + 0.05,
                    );
                    let tight_mask = crate::rest2d::contours_to_region_set(&tight_polys);
                    Some((mat_mask, tight_mask))
                } else {
                    None
                };

            let rest_enabled = !ctx.passes.rest_machining.is_disabled();
            let base_work_items: Vec<(PocketRegion2D, RegionSet2D)> = if rest_enabled {
                compute_rest_pocket_work_items(&PocketRestQuery {
                    raw_pocket: pkt_region,
                    safe_tool_regions: effective_rough,
                    current_tool_radius_mm: tool_r,
                    radial_stock_to_leave_mm: radial_leave,
                    rest_cfg: &ctx.passes.rest_machining,
                    upstream: ctx.upstream_rest,
                    z_level_wcs: z_cut,
                })
                .into_iter()
                .map(|w| (w.rest_tool_region, w.cleared_air_centers))
                .collect()
            } else {
                effective_rough
                    .iter()
                    .cloned()
                    .map(|r| (r, RegionSet2D::empty()))
                    .collect()
            };

            let mut work_items: Vec<(PocketRegion2D, RegionSet2D)> =
                if let Some((ref mat_mask, ref tight_mask)) = carryover_masks {
                    let mut clipped_items = Vec::new();
                    for (orig_reg, orig_air) in base_work_items {
                        let orig_set = RegionSet2D::from_single(orig_reg);
                        let clipped = orig_set.intersect(mat_mask);
                        if clipped.is_empty() {
                            continue;
                        }
                        let carryover_air = orig_set.difference(tight_mask);
                        let combined_air = if carryover_air.is_empty() {
                            orig_air
                        } else {
                            orig_air.union(&carryover_air)
                        };
                        for sub in clipped.regions {
                            if sub.net_area_mm2() > 0.05 {
                                clipped_items.push((sub, combined_air.clone()));
                            }
                        }
                    }
                    clipped_items
                } else {
                    base_work_items
                };

            let overhang_avoid: Option<RegionSet2D> = overhang_field
                .as_ref()
                .map(|f| f.avoidance_region_z(z_cut))
                .filter(|r| !r.is_empty());
            let combined_avoid: Option<RegionSet2D> =
                match (overhang_avoid.as_ref(), has_avoid_keepout) {
                    (Some(ov), true) => Some(ov.union(&avoid_keepout)),
                    (Some(ov), false) => Some(ov.clone()),
                    (None, true) => Some(avoid_keepout.clone()),
                    (None, false) => None,
                };
            if let Some(ref avoid) = combined_avoid {
                let mut safe_items = Vec::new();
                for (orig_reg, orig_air) in work_items {
                    let diff = RegionSet2D::from_single(orig_reg).difference(avoid);
                    for sub in diff.regions {
                        if sub.net_area_mm2() > 0.05 {
                            safe_items.push((sub, orig_air.clone()));
                        }
                    }
                }
                work_items = safe_items;
            }

            if let PocketStrategyKind::Adaptive(adaptive_params) = &params.strategy {
                for (sub_reg, cleared_air) in &work_items {
                    generate_adaptive_region_slice(
                        ctx,
                        &AdaptiveSliceSpec {
                            sub_reg,
                            finish_reg: None,
                            raw_floor_outer_ccw,
                            has_open_air: *has_open,
                            cleared_air_centers: cleared_air,
                            pocket_params: params,
                            adaptive_params,
                            prev_level_z,
                            z_cut,
                        },
                        builder,
                    )?;
                }
            } else if let Some(strategy_impl) = &legacy_strategy_impl {
                for (sub_reg, cleared_air) in &work_items {
                    let item_eng = PocketEngagementParams {
                        has_open_air_edges: engagement.has_open_air_edges
                            || !cleared_air.is_empty(),
                        inside_out: if !cleared_air.is_empty() {
                            false
                        } else {
                            engagement.inside_out
                        },
                        ..engagement
                    };
                    let subpaths = strategy_impl.generate_level_passes(
                        sub_reg,
                        raw_floor_outer_ccw,
                        &item_eng,
                        ctx.cancel,
                    )?;

                    for sp in &subpaths {
                        ctx.check_cancel()?;
                        if sp.polyline.segments.is_empty() {
                            continue;
                        }
                        builder.pass_count += 1;

                        // Rotate closed ring seam onto `cleared_air` (when Rest Machining) or closest cursor point!
                        let poly = if sp.polyline.closed && !cleared_air.is_empty() {
                            rotate_seam_to_cleared_air(&sp.polyline, cleared_air)
                        } else if sp.polyline.closed
                            && !sp.prefer_air_entry
                            && let Some(cur) = builder.current_pos()
                            && (cur.z - z_cut).abs() <= 1e-4
                            && let Some(proj) = sp.polyline.project_point(DVec2::new(cur.x, cur.y))
                        {
                            sp.polyline.rotate_closed_seam(proj.s_mm)
                        } else {
                            sp.polyline.clone()
                        };

                        let Some((start_xy, _)) = poly.sample_at_fraction(0.0) else {
                            continue;
                        };

                        let can_stay_down = if let Some(cur) = builder.current_pos()
                            && (cur.z - z_cut).abs() <= 1e-4
                        {
                            let cur_xy = DVec2::new(cur.x, cur.y);
                            let dist = cur_xy.distance(start_xy);
                            dist <= stay_down_limit
                                && is_chord_inside_region(cur_xy, start_xy, sub_reg)
                        } else {
                            false
                        };

                        if can_stay_down {
                            builder.linear_to(
                                DVec3::new(start_xy.x, start_xy.y, z_cut),
                                ctx.cutting.lead_feed_mm_min,
                                MotionRole::LinkTransition,
                            );
                        } else if sp.prefer_air_entry || cleared_air.contains_point(start_xy) {
                            retract_and_plunge(ctx, builder, start_xy, z_cut, retract_z);
                        } else if let Some(entry_reg) = &sp.entry_region {
                            emit_pocket_cavity_entry(
                                ctx,
                                builder,
                                &PocketCavityEntrySpec {
                                    entry_region: entry_reg,
                                    first_ring: &poly,
                                    ring_start_xy: start_xy,
                                    prev_level_z,
                                    z_cut,
                                    retract_z,
                                },
                            );
                        } else {
                            retract_and_plunge(ctx, builder, start_xy, z_cut, retract_z);
                        }

                        builder.emit_polyline_2d_z_interp(
                            &poly,
                            z_cut,
                            z_cut,
                            ctx.cutting.cutting_feed_mm_min,
                            sp.role,
                        );
                    }
                }
            }

            // Optional Finish Wall Pass around outer boundary & islands (`MotionRole::FinishCut`)
            if params.finish_wall_pass && !rough_regions.is_empty() && !work_items.is_empty() {
                for fin_reg in &finish_regions {
                    let outer_fin = if *has_open {
                        rotate_seam_to_air_overhang(&fin_reg.outer_ccw, raw_floor_outer_ccw)
                    } else if let Some(cur) = builder.current_pos()
                        && let Some(proj) =
                            fin_reg.outer_ccw.project_point(DVec2::new(cur.x, cur.y))
                    {
                        fin_reg.outer_ccw.rotate_closed_seam(proj.s_mm)
                    } else {
                        fin_reg.outer_ccw.clone()
                    };

                    for (loop_idx, loop_poly) in std::iter::once(&outer_fin)
                        .chain(fin_reg.islands_cw.iter())
                        .enumerate()
                    {
                        if loop_idx == 0 && *skip_outer_finish {
                            continue;
                        }
                        let raw_wall = if loop_idx == 0 {
                            raw_floor_outer_ccw
                        } else {
                            &pkt_region.islands_cw
                                [(loop_idx - 1).min(pkt_region.islands_cw.len().saturating_sub(1))]
                        };
                        let mut finish_spans = if rest_enabled {
                            clip_contour_path_to_rest(&ContourRestClipQuery {
                                raw_contour: raw_wall,
                                offset_sign: 1.0,
                                candidate_path: loop_poly,
                                current_tool_radius_mm: tool_r,
                                rest_cfg: &ctx.passes.rest_machining,
                                upstream: ctx.upstream_rest,
                                z_level_wcs: z_cut,
                            })
                        } else {
                            vec![loop_poly.clone()]
                        };
                        // Guard against closed island courtyard holes collapsing during 2D upstream
                        // swept-capsule union: verify directly against upstream cutting segments
                        // whether radial stock exceeding `min_rest_thickness` remains on an island wall.
                        if rest_enabled
                            && loop_idx >= 1
                            && finish_spans.is_empty()
                            && island_wall_has_uncut_rest_stock(
                                raw_wall,
                                &ctx.passes.rest_machining,
                                ctx.upstream_rest,
                                z_cut,
                            )
                        {
                            finish_spans.push(loop_poly.clone());
                        }

                        if let (Some(surf), Some(avoid)) =
                            (overhang_surface.as_ref(), overhang_avoid.as_ref())
                        {
                            finish_spans = finish_spans
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
                        if has_avoid_keepout {
                            finish_spans = finish_spans
                                .iter()
                                .flat_map(|sp| clip_polyline_outside_avoid(sp, &avoid_keepout))
                                .filter(|sp| sp.perimeter() > 0.05)
                                .collect();
                        }

                        for span in &finish_spans {
                            let Some((s_xy, _)) = span.sample_at_fraction(0.0) else {
                                continue;
                            };
                            let stay = builder.current_pos().is_some_and(|cur| {
                                let cur_xy = DVec2::new(cur.x, cur.y);
                                (cur.z - z_cut).abs() <= 1e-4
                                    && cur_xy.distance(s_xy) <= stay_down_limit
                                    && is_chord_inside_region(cur_xy, s_xy, fin_reg)
                                    && combined_avoid.as_ref().is_none_or(|av| {
                                        is_2d_segment_clear_of_avoid(cur_xy, s_xy, av)
                                    })
                            });
                            if stay {
                                builder.linear_to(
                                    DVec3::new(s_xy.x, s_xy.y, z_cut),
                                    ctx.cutting.lead_feed_mm_min,
                                    MotionRole::LeadIn,
                                );
                            } else {
                                retract_and_plunge(ctx, builder, s_xy, z_cut, retract_z);
                            }
                            builder.pass_count += 1;
                            builder.emit_polyline_2d_z_interp(
                                span,
                                z_cut,
                                z_cut,
                                ctx.cutting.cutting_feed_mm_min,
                                MotionRole::FinishCut,
                            );
                        }
                    }
                }
            }

            if let Some(cur) = builder.current_pos() {
                builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
            }
        }
    }

    if !any_valid_region {
        return Err(CamError::PocketTooNarrowForTool {
            tool_diameter_mm: tool_d + 2.0 * radial_leave,
        });
    }

    Ok(())
}

fn island_wall_has_uncut_rest_stock(
    raw_island: &ArcPolyline2D,
    rest_cfg: &crate::params::RestMachiningConfig,
    upstream: &[crate::rest2d::UpstreamSweptInput],
    z_level_wcs: f64,
) -> bool {
    let crate::params::RestMachiningMode::FromPrecedingOperations { source_op_ids } =
        &rest_cfg.mode
    else {
        return false;
    };
    let min_thick = rest_cfg.min_rest_thickness.value_mm.max(0.02);
    let active_upstream: Vec<&crate::rest2d::UpstreamSweptInput> = if source_op_ids.is_empty() {
        upstream.iter().collect()
    } else {
        upstream
            .iter()
            .filter(|u| source_op_ids.iter().any(|id| id == &u.op_id))
            .collect()
    };
    if active_upstream.is_empty() {
        return false;
    }

    let n_samples = 24usize;
    let mut sampled_any = false;
    for i in 0..n_samples {
        let frac = (i as f64 + 0.5) / (n_samples as f64);
        let Some((pt, _)) = raw_island.sample_at_fraction(frac) else {
            continue;
        };
        sampled_any = true;
        let mut min_remaining = f64::INFINITY;
        for up in &active_upstream {
            let r_tool = 0.5 * up.tool_diameter_mm;
            if r_tool <= 1e-4 {
                continue;
            }
            for seg in &up.toolpath.segments_wcs {
                if matches!(seg.motion, crate::toolpath::SegmentMotion3D::Rapid) {
                    continue;
                }
                if seg.start_wcs.z.max(seg.end_wcs.z) > z_level_wcs + 0.03 {
                    continue;
                }
                let d_xy = point_to_segment_xy_distance(pt, seg);
                min_remaining = min_remaining.min(d_xy - r_tool);
                if min_remaining <= min_thick {
                    break;
                }
            }
            if min_remaining <= min_thick {
                break;
            }
        }
        if min_remaining > min_thick {
            return true;
        }
    }
    let _ = sampled_any;
    false
}

fn point_to_segment_xy_distance(pt: DVec2, seg: &crate::toolpath::ToolpathSegment3D) -> f64 {
    let a = DVec2::new(seg.start_wcs.x, seg.start_wcs.y);
    let b = DVec2::new(seg.end_wcs.x, seg.end_wcs.y);
    match seg.motion {
        crate::toolpath::SegmentMotion3D::Arc {
            plane: crate::cl_data::ArcPlane::XY,
            center_wcs,
            radius_mm,
            signed_sweep_rad,
            ..
        } if radius_mm > 1e-5 => {
            let c = DVec2::new(center_wcs.x, center_wcs.y);
            let a0 = (a.y - c.y).atan2(a.x - c.x);
            let ap = (pt.y - c.y).atan2(pt.x - c.x);
            let in_sweep = if signed_sweep_rad.abs() >= std::f64::consts::TAU - 1e-3 {
                true
            } else if signed_sweep_rad >= 0.0 {
                (ap - a0).rem_euclid(std::f64::consts::TAU) <= signed_sweep_rad + 1e-5
            } else {
                (a0 - ap).rem_euclid(std::f64::consts::TAU) <= (-signed_sweep_rad) + 1e-5
            };
            if in_sweep {
                (pt.distance(c) - radius_mm).abs()
            } else {
                pt.distance(a).min(pt.distance(b))
            }
        }
        _ => {
            let ab = b - a;
            let len2 = ab.length_squared();
            if len2 <= 1e-12 {
                pt.distance(a)
            } else {
                let t = ((pt - a).dot(ab) / len2).clamp(0.0, 1.0);
                pt.distance(a + ab * t)
            }
        }
    }
}

fn rotate_seam_to_cleared_air(poly: &ArcPolyline2D, cleared_air: &RegionSet2D) -> ArcPolyline2D {
    let perim = poly.perimeter();
    if perim <= 1e-4 {
        return poly.clone();
    }
    let mut best_s = 0.0_f64;
    let mut best_dist = f64::INFINITY;
    for i in 0..48 {
        let frac = (i as f64 + 0.5) / 48.0;
        if let Some((pt, _)) = poly.sample_at_fraction(frac) {
            if cleared_air.contains_point(pt) {
                return poly.rotate_closed_seam(frac * perim);
            }
            let d = cleared_air.min_distance_to_boundary(pt);
            if d < best_dist {
                best_dist = d;
                best_s = frac * perim;
            }
        }
    }
    poly.rotate_closed_seam(best_s)
}

fn rotate_seam_to_air_overhang(
    expanded_outer_ccw: &ArcPolyline2D,
    raw_floor_outer_ccw: &ArcPolyline2D,
) -> ArcPolyline2D {
    let perim = expanded_outer_ccw.perimeter();
    if perim <= 1e-4 {
        return expanded_outer_ccw.clone();
    }
    let mut best_s = 0.0_f64;
    let mut best_outside_dist = -1.0_f64;
    for i in 0..32 {
        let frac = (i as f64 + 0.5) / 32.0;
        if let Some((pt, _)) = expanded_outer_ccw.sample_at_fraction(frac)
            && !raw_floor_outer_ccw.contains_point(pt)
        {
            let d = raw_floor_outer_ccw.distance_to_point(pt);
            if d > best_outside_dist {
                best_outside_dist = d;
                best_s = frac * perim;
            }
        }
    }
    if best_outside_dist > 0.0 {
        expanded_outer_ccw.rotate_closed_seam(best_s)
    } else {
        expanded_outer_ccw.clone()
    }
}

fn is_chord_inside_region(a: DVec2, b: DVec2, region: &PocketRegion2D) -> bool {
    let dist = a.distance(b);
    if dist <= 1e-5 {
        return true;
    }
    let steps = ((dist / 0.5).ceil() as usize).clamp(4, 32);
    for i in 1..steps {
        let t = (i as f64) / (steps as f64);
        if !region.contains_interior_point(a.lerp(b, t)) {
            return false;
        }
    }
    true
}

fn retract_and_plunge(
    ctx: &StrategyExecContext<'_>,
    builder: &mut ToolpathBuilder,
    xy: DVec2,
    z_cut: f64,
    retract_z: f64,
) {
    if let Some(cur) = builder.current_pos()
        && cur.z < retract_z - 1e-6
    {
        builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
    }
    builder.rapid_to(DVec3::new(xy.x, xy.y, retract_z));
    builder.rapid_to(DVec3::new(xy.x, xy.y, ctx.heights.feed_z_wcs));
    builder.linear_to(
        DVec3::new(xy.x, xy.y, z_cut),
        ctx.cutting.plunge_feed_mm_min,
        MotionRole::Plunge,
    );
}

struct PocketCavityEntrySpec<'a> {
    entry_region: &'a PocketRegion2D,
    first_ring: &'a ArcPolyline2D,
    ring_start_xy: DVec2,
    prev_level_z: f64,
    z_cut: f64,
    retract_z: f64,
}

struct RingLinearRampSpec<'a> {
    ring: &'a ArcPolyline2D,
    ring_start_xy: DVec2,
    ramp_top_z: f64,
    z_cut: f64,
    angle_rad: f64,
    retract_z: f64,
}

fn emit_pocket_cavity_entry(
    ctx: &StrategyExecContext<'_>,
    builder: &mut ToolpathBuilder,
    spec: &PocketCavityEntrySpec<'_>,
) {
    let &PocketCavityEntrySpec {
        entry_region,
        first_ring,
        ring_start_xy,
        prev_level_z,
        z_cut,
        retract_z,
    } = spec;
    let tool_d = ctx.tool.geometry.cutting_diameter_mm();
    let ramp_top_z = (prev_level_z + 0.5).min(ctx.heights.feed_z_wcs);
    let angle_rad = ctx.linking.ramp_angle_deg.clamp(0.5, 25.0).to_radians();

    match ctx.linking.entry_mode {
        EntryStrategy::Plunge => {
            retract_and_plunge(ctx, builder, ring_start_xy, z_cut, retract_z);
        }
        EntryStrategy::PreDrilled { point_wcs } => {
            retract_and_plunge(ctx, builder, point_wcs, z_cut, retract_z);
            builder.linear_to(
                DVec3::new(ring_start_xy.x, ring_start_xy.y, z_cut),
                ctx.cutting.lead_feed_mm_min,
                MotionRole::LeadIn,
            );
        }
        EntryStrategy::HelicalRamp => {
            let min_helix_r = 0.5 * ctx.linking.min_ramp_diameter.value_mm.max(0.2);
            if let Some(pole) = find_helical_entry_pole(entry_region)
                && pole.max_helix_radius_mm >= min_helix_r
            {
                let r_helix = pole.max_helix_radius_mm.min(0.42 * tool_d).max(min_helix_r);
                let c = pole.center_xy;
                let h_start = c + DVec2::new(r_helix, 0.0);
                let h_opp = c - DVec2::new(r_helix, 0.0);

                if let Some(cur) = builder.current_pos()
                    && cur.z < retract_z - 1e-6
                {
                    builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
                }
                builder.rapid_to(DVec3::new(h_start.x, h_start.y, retract_z));
                builder.rapid_to(DVec3::new(h_start.x, h_start.y, ctx.heights.feed_z_wcs));
                builder.linear_to(
                    DVec3::new(h_start.x, h_start.y, ramp_top_z),
                    ctx.cutting.plunge_feed_mm_min,
                    MotionRole::Plunge,
                );

                let drop = (ramp_top_z - z_cut).max(0.0);
                let pitch = (std::f64::consts::TAU * r_helix * angle_rad.tan())
                    .clamp(0.15, ctx.passes.max_stepdown.value_mm.max(0.5));
                let n_revs = (drop / pitch).ceil().max(1.0) as usize;
                let dz = drop / (n_revs as f64);

                for rev in 1..=n_revs {
                    let z_end = if rev == n_revs {
                        z_cut
                    } else {
                        ramp_top_z - dz * (rev as f64)
                    };
                    let z_mid = 0.5 * (builder.current_pos().map_or(ramp_top_z, |p| p.z) + z_end);
                    builder.arc_xy_to(XyArcMove {
                        end_wcs: DVec3::new(h_opp.x, h_opp.y, z_mid),
                        center_xy: c,
                        radius_mm: r_helix,
                        ccw: true,
                        turns: 0,
                        feed_mm_min: ctx.cutting.ramp_feed_mm_min,
                        role: MotionRole::Ramp,
                    });
                    builder.arc_xy_to(XyArcMove {
                        end_wcs: DVec3::new(h_start.x, h_start.y, z_end),
                        center_xy: c,
                        radius_mm: r_helix,
                        ccw: true,
                        turns: 0,
                        feed_mm_min: ctx.cutting.ramp_feed_mm_min,
                        role: MotionRole::Ramp,
                    });
                }
                // Flat cleanup circle at helix bottom, then feed to `ring_start_xy`!
                builder.arc_xy_to(XyArcMove {
                    end_wcs: DVec3::new(h_opp.x, h_opp.y, z_cut),
                    center_xy: c,
                    radius_mm: r_helix,
                    ccw: true,
                    turns: 0,
                    feed_mm_min: ctx.cutting.ramp_feed_mm_min,
                    role: MotionRole::Ramp,
                });
                builder.arc_xy_to(XyArcMove {
                    end_wcs: DVec3::new(h_start.x, h_start.y, z_cut),
                    center_xy: c,
                    radius_mm: r_helix,
                    ccw: true,
                    turns: 0,
                    feed_mm_min: ctx.cutting.ramp_feed_mm_min,
                    role: MotionRole::Ramp,
                });
                builder.linear_to(
                    DVec3::new(ring_start_xy.x, ring_start_xy.y, z_cut),
                    ctx.cutting.lead_feed_mm_min,
                    MotionRole::LeadIn,
                );
                return;
            }

            // Automatic fallback to Linear Profile Ramp when pocket is narrower than min_ramp_diameter (`05-operations.md` §3)!
            if builder.warnings.is_empty() {
                builder.warnings.push(
                    "Pocket sub-region narrower than min helix diameter; fell back to linear profile ramp."
                        .to_string(),
                );
            }
            emit_linear_ramp_on_ring(
                ctx,
                builder,
                &RingLinearRampSpec {
                    ring: first_ring,
                    ring_start_xy,
                    ramp_top_z,
                    z_cut,
                    angle_rad,
                    retract_z,
                },
            );
        }
        EntryStrategy::LinearRamp => {
            emit_linear_ramp_on_ring(
                ctx,
                builder,
                &RingLinearRampSpec {
                    ring: first_ring,
                    ring_start_xy,
                    ramp_top_z,
                    z_cut,
                    angle_rad,
                    retract_z,
                },
            );
        }
    }
}

fn emit_linear_ramp_on_ring(
    ctx: &StrategyExecContext<'_>,
    builder: &mut ToolpathBuilder,
    spec: &RingLinearRampSpec<'_>,
) {
    let &RingLinearRampSpec {
        ring,
        ring_start_xy,
        ramp_top_z,
        z_cut,
        angle_rad,
        retract_z,
    } = spec;
    if ring.perimeter() <= 1e-3 {
        retract_and_plunge(ctx, builder, ring_start_xy, z_cut, retract_z);
        return;
    }
    if let Some(cur) = builder.current_pos()
        && cur.z < retract_z - 1e-6
    {
        builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
    }
    builder.rapid_to(DVec3::new(ring_start_xy.x, ring_start_xy.y, retract_z));
    builder.rapid_to(DVec3::new(
        ring_start_xy.x,
        ring_start_xy.y,
        ctx.heights.feed_z_wcs,
    ));
    builder.linear_to(
        DVec3::new(ring_start_xy.x, ring_start_xy.y, ramp_top_z),
        ctx.cutting.plunge_feed_mm_min,
        MotionRole::Plunge,
    );

    let drop = (ramp_top_z - z_cut).max(0.0);
    let perim = ring.perimeter();
    let needed_len = drop / angle_rad.tan();
    let n_laps = (needed_len / perim).ceil().max(1.0) as usize;
    let dz = drop / (n_laps as f64);
    let mut cur_z = ramp_top_z;
    for lap in 1..=n_laps {
        let next_z = if lap == n_laps {
            z_cut
        } else {
            ramp_top_z - dz * (lap as f64)
        };
        builder.emit_polyline_2d_z_interp(
            ring,
            cur_z,
            next_z,
            ctx.cutting.ramp_feed_mm_min,
            MotionRole::Ramp,
        );
        cur_z = next_z;
    }
    if builder
        .current_pos()
        .is_some_and(|p| DVec2::new(p.x, p.y).distance(ring_start_xy) > 1e-4)
    {
        builder.linear_to(
            DVec3::new(ring_start_xy.x, ring_start_xy.y, z_cut),
            ctx.cutting.lead_feed_mm_min,
            MotionRole::LeadIn,
        );
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::sync::{Arc, atomic::AtomicBool};

    use chip_cad::{CadKernel, MonstertruckKernel, SurfaceKind, tutorial_adaptive_pocket_step};
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit, ParamLength, approx_eq};
    use chip_tools::{
        CoolantMode, CutterGeometry, CuttingPreset, SpindleDirection, SurfaceSpeedPresetSpec, Tool,
        ToolId,
    };

    use crate::{
        GeometrySelectionItem, Operation, OperationGenRequest, PocketStrategyKind,
        RestMachiningConfig, RestMachiningMode, Setup, StrategyParams, UpstreamSweptInput,
        WorkOffsetSlot, generate_operation_toolpath_with_rest, params::AdaptiveClearingParams,
        rest2d::swept_toolpath_coverage_2d,
    };

    #[test]
    fn adaptive_rest_cleans_circular_island_wall_stock_left_by_rougher() {
        let step = tutorial_adaptive_pocket_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("adap.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let mut setup = Setup::new("setup_1", "Top", WorkOffsetSlot::G54);
        setup.stock = crate::StockConfig::RelativeBox {
            side_pos_x: ParamLength::from_mm(0.0),
            side_neg_x: ParamLength::from_mm(0.0),
            side_pos_y: ParamLength::from_mm(0.0),
            side_neg_y: ParamLength::from_mm(0.0),
            symmetric_xy: true,
            top_offset_z: ParamLength::from_mm(0.0),
            bottom_offset_z: ParamLength::from_mm(0.0),
            sheet_thickness_z: ParamLength::from_mm(20.0),
            round_increment_mm: None,
        };
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let floor = doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { origin_mm, normal }
                    if normal.z > 0.99 && approx_eq(origin_mm.z, 10.0, 1e-2)
                )
            })
            .unwrap();
        let bind = doc.bind_face(floor.id).unwrap();

        let t10 = Tool {
            id: ToolId("t10".to_string()),
            tool_number: 10,
            length_offset_number: 10,
            diameter_offset_number: 10,
            name: "10mm".to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::AirBlast,
            stickout_length_mm: 30.0,
            geometry: CutterGeometry::FlatEndmill {
                diameter_mm: 10.0,
                flute_length_mm: 22.0,
                overall_length_mm: 65.0,
                shank_diameter_mm: 10.0,
                flutes: 3,
            },
            presets: vec![CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
                id: "al",
                name: "Al",
                material: "Al",
                diameter_mm: 10.0,
                flutes: 3,
                surface_speed_m_min: 280.0,
                chipload_mm_tooth: 0.055,
                default_stepdown_mm: 12.0,
                default_stepover_mm: 2.0,
                coolant_override: Some(CoolantMode::AirBlast),
            })],
        };
        let t3 = Tool {
            id: ToolId("t3".to_string()),
            tool_number: 3,
            length_offset_number: 3,
            diameter_offset_number: 3,
            name: "3mm".to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::AirBlast,
            stickout_length_mm: 18.0,
            geometry: CutterGeometry::FlatEndmill {
                diameter_mm: 3.0,
                flute_length_mm: 12.0,
                overall_length_mm: 45.0,
                shank_diameter_mm: 3.0,
                flutes: 3,
            },
            presets: vec![CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
                id: "al",
                name: "Al",
                material: "Al",
                diameter_mm: 3.0,
                flutes: 3,
                surface_speed_m_min: 200.0,
                chipload_mm_tooth: 0.022,
                default_stepdown_mm: 5.0,
                default_stepover_mm: 0.6,
                coolant_override: Some(CoolantMode::AirBlast),
            })],
        };

        let mut op1 = Operation::new_adaptive_2d("op1", "Rough");
        op1.tool_id = Some(t10.id.clone());
        op1.passes.stock_to_leave_radial = ParamLength::from_mm(0.2);
        op1.selections.push(GeometrySelectionItem::PlanarFace {
            binding: bind.clone(),
        });

        let cancel = AtomicBool::new(false);
        let tp1 = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: &[],
            setup: &resolved,
            op: &op1,
            tool: &t10,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap();

        let up = vec![UpstreamSweptInput {
            op_id: "op1".to_string(),
            tool_diameter_mm: 10.0,
            corner_radius_mm: 0.0,
            toolpath: Arc::new(tp1),
            indexed_angle_deg: None,
            is_rotary_native: false,
        }];
        let cov = swept_toolpath_coverage_2d(&up, -10.0);
        eprintln!(
            "cov regions={}, islands_on_0={}, contains(50,40)={}",
            cov.regions.len(),
            cov.regions.first().map_or(0, |r| r.islands_cw.len()),
            cov.contains_point(chip_math::DVec2::new(50.0, 40.0))
        );
        if let Some(isl) = cov.regions.first().and_then(|r| r.islands_cw.first()) {
            eprintln!("cov island aabb={:?}", isl.exact_aabb());
        }

        let mut op2 = Operation::new_adaptive_2d("op2", "Rest");
        op2.tool_id = Some(t3.id.clone());
        op2.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        op2.passes.rest_machining = RestMachiningConfig {
            mode: RestMachiningMode::FromPrecedingOperations {
                source_op_ids: vec!["op1".to_string()],
            },
            rest_overlap: ParamLength::from_mm(0.30),
            min_rest_thickness: ParamLength::from_mm(0.08),
        };
        if let StrategyParams::Pocket2D(p) = op2.strategy_mut() {
            p.strategy = PocketStrategyKind::Adaptive(AdaptiveClearingParams::default());
        }

        let tp2 = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: &[],
            setup: &resolved,
            op: &op2,
            tool: &t3,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: false,
            upstream_rest: &up,
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap();

        let island_fin_segs = tp2
            .segments_wcs
            .iter()
            .filter(|s| {
                matches!(
                    s.motion,
                    crate::SegmentMotion3D::Arc {
                        radius_mm,
                        role: crate::MotionRole::FinishCut,
                        ..
                    } if (radius_mm - 9.5).abs() < 0.05
                )
            })
            .count();
        assert!(
            island_fin_segs >= 4,
            "Rest finish pass must clean the 0.2 mm radial stock left around the Ø16 mm circular island, got {island_fin_segs} arc segments"
        );
    }
}
