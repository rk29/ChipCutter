use std::sync::atomic::{AtomicBool, Ordering};

use chip_math::{ArcPolyline2D, DVec2, DVec3};

use crate::{
    cl_data::MotionRole,
    offset2d::{
        OffsetRegionNode, PocketRegion2D, build_offset_region_tree, clip_scanline_to_region,
        find_helical_entry_pole,
    },
    params::{EntryStrategy, Pocket2DParams, PocketPassOrder, PocketStrategyKind},
    selection::ExtractedFeature25D,
    strategies::{StrategyExecContext, compute_z_levels},
    toolpath::ToolpathBuilder,
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

    let mut raw_pockets: Vec<(PocketRegion2D, ArcPolyline2D, bool)> = Vec::new();
    for feat in &ctx.selection.features {
        match feat {
            ExtractedFeature25D::PlanarFace {
                outer_boundary_wcs,
                open_outer_segments,
                raised_islands_wcs,
                ..
            } => {
                let has_open = open_outer_segments.iter().any(|&b| b);
                let open_shift = if has_open {
                    tool_r + radial_leave + params.open_edge_overhang.value_mm.max(0.2)
                } else {
                    0.0
                };
                let region = PocketRegion2D::from_face_with_open_extensions(
                    outer_boundary_wcs,
                    open_outer_segments,
                    raised_islands_wcs,
                    open_shift,
                );
                raw_pockets.push((
                    region,
                    outer_boundary_wcs.ensure_orientation(true),
                    has_open,
                ));
            }
            ExtractedFeature25D::Contour { polyline_wcs, .. } if polyline_wcs.closed => {
                let ccw = polyline_wcs.ensure_orientation(true);
                raw_pockets.push((PocketRegion2D::new(ccw.clone(), Vec::new()), ccw, false));
            }
            _ => {}
        }
    }

    if raw_pockets.is_empty() {
        return Err(CamError::EmptySelection {
            op_name: ctx.op.name.clone(),
            op_kind: ctx.op.kind.label(),
        });
    }

    let strategy_impl: Box<dyn PocketStrategy> = match params.strategy {
        PocketStrategyKind::ContourParallel => Box::new(ContourParallelStrategy),
        PocketStrategyKind::ZigZag { pass_angle_deg } => {
            Box::new(ZigZagPocketStrategy { pass_angle_deg })
        }
    };

    let finish_allowance = if params.finish_wall_pass {
        params.finish_wall_stepover.value_mm.max(0.0)
    } else {
        0.0
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

    for (pkt_region, raw_floor_outer_ccw, has_open) in &raw_pockets {
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

            for sub_reg in effective_rough {
                let subpaths = strategy_impl.generate_level_passes(
                    sub_reg,
                    raw_floor_outer_ccw,
                    &engagement,
                    ctx.cancel,
                )?;

                for sp in &subpaths {
                    ctx.check_cancel()?;
                    if sp.polyline.segments.is_empty() {
                        continue;
                    }
                    builder.pass_count += 1;

                    // Rotate closed ring seam to the closest point to `current_pos` when staying down!
                    let poly = if sp.polyline.closed
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

                    // Can we stay down from the previous pass at `z_cut` without retracting?
                    let can_stay_down = if let Some(cur) = builder.current_pos()
                        && (cur.z - z_cut).abs() <= 1e-4
                    {
                        let cur_xy = DVec2::new(cur.x, cur.y);
                        let dist = cur_xy.distance(start_xy);
                        dist <= stay_down_limit && is_chord_inside_region(cur_xy, start_xy, sub_reg)
                    } else {
                        false
                    };

                    if can_stay_down {
                        builder.linear_to(
                            DVec3::new(start_xy.x, start_xy.y, z_cut),
                            ctx.cutting.lead_feed_mm_min,
                            MotionRole::LinkTransition,
                        );
                    } else if sp.prefer_air_entry {
                        // Open-edge pocket: plunge safely in open air outside stock!
                        retract_and_plunge(ctx, builder, start_xy, z_cut, retract_z);
                    } else if let Some(entry_reg) = &sp.entry_region {
                        emit_pocket_cavity_entry(
                            ctx,
                            builder,
                            entry_reg,
                            &poly,
                            start_xy,
                            prev_level_z,
                            z_cut,
                            retract_z,
                        );
                    } else {
                        // Island or secondary cleanup loop inside already-cleared cavity!
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

            // Optional Finish Wall Pass around outer boundary & islands (`MotionRole::FinishCut`)
            if params.finish_wall_pass && !rough_regions.is_empty() {
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

                    for loop_poly in std::iter::once(&outer_fin).chain(fin_reg.islands_cw.iter()) {
                        let Some((s_xy, _)) = loop_poly.sample_at_fraction(0.0) else {
                            continue;
                        };
                        let stay = builder.current_pos().is_some_and(|cur| {
                            (cur.z - z_cut).abs() <= 1e-4
                                && DVec2::new(cur.x, cur.y).distance(s_xy) <= stay_down_limit
                                && is_chord_inside_region(DVec2::new(cur.x, cur.y), s_xy, fin_reg)
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
                            loop_poly,
                            z_cut,
                            z_cut,
                            ctx.cutting.cutting_feed_mm_min,
                            MotionRole::FinishCut,
                        );
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

#[allow(clippy::too_many_arguments)]
fn emit_pocket_cavity_entry(
    ctx: &StrategyExecContext<'_>,
    builder: &mut ToolpathBuilder,
    entry_region: &PocketRegion2D,
    first_ring: &ArcPolyline2D,
    ring_start_xy: DVec2,
    prev_level_z: f64,
    z_cut: f64,
    retract_z: f64,
) {
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
                    builder.arc_xy_to(
                        DVec3::new(h_opp.x, h_opp.y, z_mid),
                        c,
                        r_helix,
                        true,
                        0,
                        ctx.cutting.ramp_feed_mm_min,
                        MotionRole::Ramp,
                    );
                    builder.arc_xy_to(
                        DVec3::new(h_start.x, h_start.y, z_end),
                        c,
                        r_helix,
                        true,
                        0,
                        ctx.cutting.ramp_feed_mm_min,
                        MotionRole::Ramp,
                    );
                }
                // Flat cleanup circle at helix bottom, then feed to `ring_start_xy`!
                builder.arc_xy_to(
                    DVec3::new(h_opp.x, h_opp.y, z_cut),
                    c,
                    r_helix,
                    true,
                    0,
                    ctx.cutting.ramp_feed_mm_min,
                    MotionRole::Ramp,
                );
                builder.arc_xy_to(
                    DVec3::new(h_start.x, h_start.y, z_cut),
                    c,
                    r_helix,
                    true,
                    0,
                    ctx.cutting.ramp_feed_mm_min,
                    MotionRole::Ramp,
                );
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
                first_ring,
                ring_start_xy,
                ramp_top_z,
                z_cut,
                angle_rad,
                retract_z,
            );
        }
        EntryStrategy::LinearRamp => {
            emit_linear_ramp_on_ring(
                ctx,
                builder,
                first_ring,
                ring_start_xy,
                ramp_top_z,
                z_cut,
                angle_rad,
                retract_z,
            );
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn emit_linear_ramp_on_ring(
    ctx: &StrategyExecContext<'_>,
    builder: &mut ToolpathBuilder,
    ring: &ArcPolyline2D,
    ring_start_xy: DVec2,
    ramp_top_z: f64,
    z_cut: f64,
    angle_rad: f64,
    retract_z: f64,
) {
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
