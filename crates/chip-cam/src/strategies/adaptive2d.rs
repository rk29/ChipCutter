//! **2D Adaptive / Trochoidal Clearing** strategy (`05-operations.md` §3.3).
//!
//! Expands an already-machined 2D cavity wavefront $W_k \to W_{k+1} = \text{dilate}(W_k, a_e) \cap \Omega_{\text{safe}}$
//! so every cutting pass is a bounded-engagement crescent peel ($\text{radial bite} \le a_e$), linked via
//! dogleg-safe linear `G1` micro-lifts across cleared cavity air.

use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3};
use chip_tools::compute_adaptive_thinning_feed;

use crate::{
    cl_data::MotionRole,
    medial::find_medial_inscribed_pole,
    offset2d::{PocketRegion2D, clip_scanline_to_region},
    params::{AdaptiveClearingParams, AdaptiveCutDirection, EntryStrategy, Pocket2DParams},
    rest2d::{RegionSet2D, clip_polyline_outside_region_set, swept_segment_capsule_2d},
    strategies::StrategyExecContext,
    toolpath::{ToolpathBuilder, XyArcMove},
    wcs::CamError,
};

/// Context for clearing a single Z slice of one [`PocketRegion2D`] with **2D Adaptive Clearing**.
pub struct AdaptiveSliceSpec<'a> {
    pub sub_reg: &'a PocketRegion2D,
    pub finish_reg: Option<&'a PocketRegion2D>,
    pub raw_floor_outer_ccw: &'a ArcPolyline2D,
    pub has_open_air: bool,
    pub cleared_air_centers: &'a RegionSet2D,
    pub pocket_params: &'a Pocket2DParams,
    pub adaptive_params: &'a AdaptiveClearingParams,
    pub prev_level_z: f64,
    pub z_cut: f64,
}

struct PeelLinkContext<'a> {
    cleared_cavity: &'a RegionSet2D,
    z_cut: f64,
    micro_lift_z: f64,
    retract_z: f64,
    stay_down_limit: f64,
    high_feed_mm_min: f64,
    lead_radius_mm: f64,
    cut_feed_mm_min: f64,
}

/// Generates constant-engagement **2D Adaptive / Trochoidal** passes for one pocket region at `spec.z_cut`.
pub fn generate_adaptive_region_slice(
    ctx: &StrategyExecContext<'_>,
    spec: &AdaptiveSliceSpec<'_>,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    ctx.check_cancel()?;

    let tool_d = ctx.tool.geometry.cutting_diameter_mm().max(0.1);
    let tool_r = 0.5 * tool_d;
    let ae = spec
        .adaptive_params
        .optimal_load
        .value_mm
        .clamp(0.02 * tool_d, 0.50 * tool_d);
    let micro_lift = spec.adaptive_params.micro_lift_z.value_mm.clamp(0.05, 5.0);
    let retract_z = ctx.safe_retract_z();
    let stay_down_limit = ctx
        .linking
        .safe_stay_down_distance
        .value_mm
        .max(tool_d * 4.0);

    let cut_feed = if spec.adaptive_params.auto_chip_thinning {
        compute_adaptive_thinning_feed(
            tool_d,
            ae,
            ctx.cutting.cutting_feed_mm_min,
            spec.adaptive_params.max_thinning_feed_multiplier,
        )
    } else {
        ctx.cutting.cutting_feed_mm_min
    };
    let high_feed = (ctx.cutting.cutting_feed_mm_min
        * spec
            .adaptive_params
            .no_engagement_feed_multiplier
            .clamp(1.0, 5.0))
    .max(cut_feed);
    let lead_r = spec
        .adaptive_params
        .min_trochoid_radius
        .value_mm
        .clamp(0.05, 0.45 * ae)
        .min(0.25 * tool_d);

    let safe_set = RegionSet2D::from_single(spec.sub_reg.clone());
    if safe_set.is_empty() {
        return Ok(());
    }

    // 1. Initialize cleared tool-center cavity `w_cleared` (`W_0`) via Rest air, open stock air, or helical/linear ramp:
    let mut w_cleared = init_adaptive_cavity(ctx, spec, tool_d, tool_r, ae, retract_z, builder)?;

    // 2. Iteratively expand wavefront `W_k -> W_{k+1} = dilate(W_k, step) ∩ Ω_safe`
    let max_iters = 2048usize;
    let mut pass_parity = 0usize;

    for _iter in 0..max_iters {
        ctx.check_cancel()?;
        let remaining = safe_set.difference(&w_cleared);
        if remaining.is_empty() || remaining.total_area_mm2() <= 1e-4 {
            break;
        }

        let candidate_next = w_cleared.dilate_outward(ae).intersect(&safe_set);
        let prev_in_safe_area = w_cleared.intersect(&safe_set).total_area_mm2();
        let next_in_safe_area = candidate_next.total_area_mm2();

        // If `w_cleared` cannot reach a disconnected sub-pocket component across a narrow pinched neck,
        // seed a fresh helical/linear entry inside the largest remaining component!
        if next_in_safe_area - prev_in_safe_area <= 1e-4 {
            if let Some(comp) = remaining
                .regions
                .iter()
                .max_by(|a, b| a.net_area_mm2().total_cmp(&b.net_area_mm2()))
            {
                let seed = emit_closed_cavity_seed(
                    ctx,
                    comp,
                    tool_d,
                    spec.prev_level_z,
                    spec.z_cut,
                    retract_z,
                    builder,
                );
                if seed.is_empty() {
                    break;
                }
                w_cleared = w_cleared.union(&seed);
                continue;
            }
            break;
        }

        let crescent = candidate_next.difference(&w_cleared);
        let clip_eps = (0.025 * ae).clamp(2e-3, 0.02);
        let w_mask = w_cleared.dilate_outward(clip_eps);

        let mut peel_chains: Vec<ArcPolyline2D> = Vec::new();
        for cres_reg in &crescent.regions {
            let mut chains = clip_polyline_outside_region_set(&cres_reg.outer_ccw, &w_mask);
            for isl_cw in &cres_reg.islands_cw {
                chains.extend(clip_polyline_outside_region_set(isl_cw, &w_mask));
            }
            if chains.is_empty() && cres_reg.net_area_mm2() > 2e-3 {
                chains.push(cres_reg.outer_ccw.clone());
            }
            peel_chains.extend(chains);
        }

        order_chains_from_cursor(&mut peel_chains, builder.current_pos());

        // `w_cleared` is already safe air before cutting `crescent`; update incrementally as chains execute
        // or link inside `w_cleared` (which is 100% cleared prior to this wavefront step!).
        for raw_chain in peel_chains {
            ctx.check_cancel()?;
            if raw_chain.segments.is_empty() {
                continue;
            }
            let oriented_chain = match spec.adaptive_params.cut_direction {
                AdaptiveCutDirection::Climb => raw_chain,
                AdaptiveCutDirection::Conventional => raw_chain.reverse(),
                AdaptiveCutDirection::BothWays => {
                    if pass_parity % 2 == 1 {
                        raw_chain.reverse()
                    } else {
                        raw_chain
                    }
                }
            };
            pass_parity += 1;

            let link_ctx = PeelLinkContext {
                cleared_cavity: &w_cleared,
                z_cut: spec.z_cut,
                micro_lift_z: micro_lift,
                retract_z,
                stay_down_limit,
                high_feed_mm_min: high_feed,
                lead_radius_mm: lead_r,
                cut_feed_mm_min: cut_feed,
            };
            emit_peel_chain_with_micro_lift(ctx, &oriented_chain, &link_ctx, builder);
        }

        // Merge newly cleared `candidate_next` into `w_cleared`
        w_cleared = w_cleared.union(&candidate_next);
    }

    // 3. Optional Finish Wall Pass around outer boundary & islands (`MotionRole::FinishCut`)
    if let Some(fin_reg) = spec.finish_reg {
        let fin_direction_ccw = !matches!(
            spec.adaptive_params.cut_direction,
            AdaptiveCutDirection::Conventional
        );
        let mut loops: Vec<ArcPolyline2D> = Vec::with_capacity(1 + fin_reg.islands_cw.len());
        let outer_oriented = fin_reg.outer_ccw.ensure_orientation(fin_direction_ccw);
        loops.push(
            if let Some(cur) = builder.current_pos()
                && let Some(proj) = outer_oriented.project_point(DVec2::new(cur.x, cur.y))
            {
                outer_oriented.rotate_closed_seam(proj.s_mm)
            } else {
                outer_oriented
            },
        );
        for isl in &fin_reg.islands_cw {
            loops.push(isl.ensure_orientation(!fin_direction_ccw));
        }

        let fin_cavity = w_cleared.union(&RegionSet2D::from_single(fin_reg.clone()));
        for lp in &loops {
            let link_ctx = PeelLinkContext {
                cleared_cavity: &fin_cavity,
                z_cut: spec.z_cut,
                micro_lift_z: micro_lift,
                retract_z,
                stay_down_limit,
                high_feed_mm_min: high_feed,
                lead_radius_mm: lead_r,
                cut_feed_mm_min: ctx.cutting.cutting_feed_mm_min,
            };
            emit_finish_loop(ctx, lp, &link_ctx, builder);
        }
    }

    Ok(())
}

fn init_adaptive_cavity(
    ctx: &StrategyExecContext<'_>,
    spec: &AdaptiveSliceSpec<'_>,
    tool_d: f64,
    tool_r: f64,
    ae: f64,
    retract_z: f64,
    builder: &mut ToolpathBuilder,
) -> Result<RegionSet2D, CamError> {
    // 1. Rest Machining adjacent cavity air (`cleared_air_centers`):
    if !spec.cleared_air_centers.is_empty() {
        let probe = RegionSet2D::from_single(spec.sub_reg.clone()).dilate_outward(0.5 * ae + 0.05);
        if !spec.cleared_air_centers.intersect(&probe).is_empty() {
            return Ok(spec.cleared_air_centers.clone());
        }
    }

    // 2. Open-Edge Exterior Air Entry (outside physical stock XY footprint by `>= tool_r - 0.05`):
    if spec.has_open_air {
        let stock_aabb = ctx.setup.stock.aabb_wcs();
        let stock_engage_box = rect_region_2d(
            stock_aabb.min.x - (tool_r - 0.05),
            stock_aabb.min.y - (tool_r - 0.05),
            stock_aabb.max.x + (tool_r - 0.05),
            stock_aabb.max.y + (tool_r - 0.05),
        );
        let sub_set = RegionSet2D::from_single(spec.sub_reg.clone());
        let exterior_air = sub_set.difference(&stock_engage_box);
        if !exterior_air.is_empty() && exterior_air.total_area_mm2() > 1e-3 {
            return Ok(exterior_air);
        }

        // Also check air overhang outside `raw_floor_outer_ccw` when cutting a step shelf whose upper
        // open boundary already cleared stock at higher Z or opens into air:
        let raw_floor_set = RegionSet2D::from_single(PocketRegion2D::new(
            spec.raw_floor_outer_ccw.clone(),
            Vec::new(),
        ))
        .dilate_outward((tool_r - 0.05).max(0.01));
        let floor_air = sub_set.difference(&raw_floor_set);
        if !floor_air.is_empty() && floor_air.total_area_mm2() > 1e-3 {
            return Ok(floor_air);
        }
    }

    // 3. Closed Cavity: Helical or Linear Ramp Entry at Medial Inscribed Circle Pole
    let seed = emit_closed_cavity_seed(
        ctx,
        spec.sub_reg,
        tool_d,
        spec.prev_level_z,
        spec.z_cut,
        retract_z,
        builder,
    );
    if seed.is_empty() {
        return Err(CamError::PocketTooNarrowForTool {
            tool_diameter_mm: tool_d,
        });
    }
    Ok(seed)
}

fn emit_closed_cavity_seed(
    ctx: &StrategyExecContext<'_>,
    region: &PocketRegion2D,
    tool_d: f64,
    prev_level_z: f64,
    z_cut: f64,
    retract_z: f64,
    builder: &mut ToolpathBuilder,
) -> RegionSet2D {
    let ramp_top_z = (prev_level_z + 0.5).min(ctx.heights.feed_z_wcs);
    let angle_rad = ctx.linking.ramp_angle_deg.clamp(0.5, 20.0).to_radians();
    let min_helix_r = 0.5 * ctx.linking.min_ramp_diameter.value_mm.max(0.2);

    if !matches!(
        ctx.linking.entry_mode,
        EntryStrategy::LinearRamp | EntryStrategy::Plunge
    ) && let Some(pole) = find_medial_inscribed_pole(region)
        && pole.max_helix_radius_mm >= min_helix_r
    {
        let r_helix = (pole.max_helix_radius_mm - 0.01).clamp(min_helix_r, 0.42 * tool_d);
        let c = pole.center_xy;
        let h_start = c + DVec2::new(r_helix, 0.0);
        let h_opp = c - DVec2::new(r_helix, 0.0);

        retract_and_rapid_to_feed_z(ctx, builder, h_start, retract_z);

        let dz_total = (ramp_top_z - z_cut).max(0.0);
        if dz_total <= 1e-5 {
            builder.linear_to(
                DVec3::new(h_start.x, h_start.y, z_cut),
                ctx.cutting.plunge_feed_mm_min,
                MotionRole::Plunge,
            );
        } else {
            if ctx.heights.feed_z_wcs > ramp_top_z + 1e-6 {
                builder.linear_to(
                    DVec3::new(h_start.x, h_start.y, ramp_top_z),
                    ctx.cutting.plunge_feed_mm_min,
                    MotionRole::Plunge,
                );
            }
            let pitch =
                (std::f64::consts::TAU * r_helix * angle_rad.tan()).clamp(0.15, dz_total.max(0.15));
            let n_revs = (dz_total / pitch).ceil().max(1.0) as usize;
            let dz_rev = dz_total / (n_revs as f64);
            let mut z_cur = ramp_top_z;

            for i in 0..n_revs {
                let z_end = if i + 1 == n_revs {
                    z_cut
                } else {
                    z_cur - dz_rev
                };
                let z_mid = 0.5 * (z_cur + z_end);
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
                z_cur = z_end;
            }
        }

        // Full 360° flat floor cleanup at `z_cut` so the entire `r_helix` seed disk is flat at `z_cut`!
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
        builder.pass_count += 1;

        return RegionSet2D::from_circle(c, r_helix);
    }

    // Fallback for narrow slots (`max_helix_r < min_helix_r`): zig-zag linear ramp along a short safe chord
    // inside `region` (either between two interior points or along the longest segment of `region.outer_ccw`)!
    let Some((seg_a, seg_b)) = find_linear_ramp_chord(region, tool_d) else {
        return RegionSet2D::empty();
    };
    retract_and_rapid_to_feed_z(ctx, builder, seg_a, retract_z);
    let chord_len = seg_a.distance(seg_b).max(0.05);
    let dz_total = (ramp_top_z - z_cut).max(0.0);

    if matches!(ctx.linking.entry_mode, EntryStrategy::Plunge)
        || dz_total <= 1e-5
        || chord_len <= 0.1
    {
        builder.linear_to(
            DVec3::new(seg_a.x, seg_a.y, z_cut),
            ctx.cutting.plunge_feed_mm_min,
            MotionRole::Plunge,
        );
        builder.linear_to(
            DVec3::new(seg_b.x, seg_b.y, z_cut),
            ctx.cutting.ramp_feed_mm_min,
            MotionRole::Ramp,
        );
    } else {
        if ctx.heights.feed_z_wcs > ramp_top_z + 1e-6 {
            builder.linear_to(
                DVec3::new(seg_a.x, seg_a.y, ramp_top_z),
                ctx.cutting.plunge_feed_mm_min,
                MotionRole::Plunge,
            );
        }
        let dz_per_leg = (chord_len * angle_rad.tan()).clamp(0.10, dz_total.max(0.10));
        let n_legs = (dz_total / dz_per_leg).ceil().max(1.0) as usize;
        let step_z = dz_total / (n_legs as f64);
        let mut z_cur = ramp_top_z;
        for leg in 0..n_legs {
            z_cur = if leg + 1 == n_legs {
                z_cut
            } else {
                z_cur - step_z
            };
            let target_xy = if leg % 2 == 0 { seg_b } else { seg_a };
            builder.linear_to(
                DVec3::new(target_xy.x, target_xy.y, z_cur),
                ctx.cutting.ramp_feed_mm_min,
                MotionRole::Ramp,
            );
        }
        // Flat cleanup stroke back across the ramp chord so the full slot seed is flat at `z_cut`
        let other_xy = if n_legs % 2 == 1 { seg_a } else { seg_b };
        builder.linear_to(
            DVec3::new(other_xy.x, other_xy.y, z_cut),
            ctx.cutting.ramp_feed_mm_min,
            MotionRole::Ramp,
        );
    }
    builder.pass_count += 1;

    let seed_r = (0.08 * tool_d).clamp(0.05, 0.5);
    swept_segment_capsule_2d(
        &ArcSegment2D::Line {
            start: seg_a,
            end: seg_b,
        },
        seed_r,
    )
    .map(RegionSet2D::from_single)
    .unwrap_or_else(RegionSet2D::empty)
    .intersect(&RegionSet2D::from_single(region.clone()))
}

fn find_linear_ramp_chord(region: &PocketRegion2D, tool_d: f64) -> Option<(DVec2, DVec2)> {
    let max_len = (1.25 * tool_d).max(1.0);
    let best_seg = region
        .outer_ccw
        .segments
        .iter()
        .max_by(|a, b| a.length().total_cmp(&b.length()))?;
    let p0 = best_seg.start();
    let len = best_seg.length();
    if len <= 1e-4 {
        return Some((p0, p0));
    }
    let frac = (max_len / len).clamp(0.1, 1.0);
    let (p1, _) = best_seg.eval_at(frac)?;
    Some((p0, p1))
}

fn emit_peel_chain_with_micro_lift(
    ctx: &StrategyExecContext<'_>,
    chain: &ArcPolyline2D,
    link: &PeelLinkContext<'_>,
    builder: &mut ToolpathBuilder,
) {
    let poly = if chain.closed
        && let Some(cur) = builder.current_pos()
        && let Some(proj) = chain.project_point(DVec2::new(cur.x, cur.y))
    {
        chain.rotate_closed_seam(proj.s_mm)
    } else {
        chain.clone()
    };

    let Some(first_seg) = poly.segments.first() else {
        return;
    };
    let Some(last_seg) = poly.segments.last() else {
        return;
    };
    let Some((cut_start, tan_start)) = first_seg.eval_at(0.0) else {
        return;
    };
    let Some((cut_end, tan_end)) = last_seg.eval_at(1.0) else {
        return;
    };

    // 1. Prefer a smooth 60° tangential arc lead-in starting strictly inside `cleared_cavity`;
    //    otherwise locate the nearest zero-engagement point `air_xy` strictly inside `cleared_cavity`
    //    so vertical plunges/micro-drops NEVER touch uncut stock!
    let lead_in_arc = try_cleared_tangent_lead_in(
        cut_start,
        tan_start,
        link.lead_radius_mm,
        link.cleared_cavity,
    );

    let entry_air_xy = if let Some((pre_xy, _, _)) = lead_in_arc {
        pre_xy
    } else {
        find_nearest_air_point(cut_start, link.cleared_cavity).unwrap_or(cut_start)
    };

    // 2. Link from current cursor to `(entry_air_xy, z_cut)` in already-cleared air
    link_to_xy_at_z_cut(ctx, entry_air_xy, link, builder);

    // 3. Feed smoothly from `(entry_air_xy, z_cut)` onto `(cut_start, z_cut)`
    if let Some((_, center_xy, ccw)) = lead_in_arc {
        builder.arc_xy_to(XyArcMove {
            end_wcs: DVec3::new(cut_start.x, cut_start.y, link.z_cut),
            center_xy,
            radius_mm: link.lead_radius_mm,
            ccw,
            turns: 0,
            feed_mm_min: ctx.cutting.lead_feed_mm_min,
            role: MotionRole::LeadIn,
        });
    } else if entry_air_xy.distance(cut_start) > 1e-4 {
        builder.linear_to(
            DVec3::new(cut_start.x, cut_start.y, link.z_cut),
            ctx.cutting.lead_feed_mm_min,
            MotionRole::LeadIn,
        );
    }

    // 4. Sweep the bounded-engagement peel pass
    builder.pass_count += 1;
    builder.emit_polyline_2d_z_interp(
        &poly,
        link.z_cut,
        link.z_cut,
        link.cut_feed_mm_min,
        MotionRole::Cut,
    );

    // 5. For open crescent peels, arc/step slightly inward off the cut wall into `cleared_cavity`
    //    before lifting so the cutter never rubs the wall and begins the next link inside `cleared_cavity`!
    if !poly.closed {
        if let Some((out_xy, center_xy, ccw)) =
            try_cleared_tangent_lead_out(cut_end, tan_end, link.lead_radius_mm, link.cleared_cavity)
        {
            builder.arc_xy_to(XyArcMove {
                end_wcs: DVec3::new(out_xy.x, out_xy.y, link.z_cut),
                center_xy,
                radius_mm: link.lead_radius_mm,
                ccw,
                turns: 0,
                feed_mm_min: ctx.cutting.lead_feed_mm_min,
                role: MotionRole::LeadOut,
            });
        } else if let Some(exit_air) = find_nearest_air_point(cut_end, link.cleared_cavity)
            && exit_air.distance(cut_end) > 1e-4
            && exit_air.distance(cut_end) <= 3.0 * link.lead_radius_mm + 0.5
        {
            builder.linear_to(
                DVec3::new(exit_air.x, exit_air.y, link.z_cut),
                ctx.cutting.lead_feed_mm_min,
                MotionRole::LeadOut,
            );
        }
    }
}

fn find_nearest_air_point(target_xy: DVec2, cleared: &RegionSet2D) -> Option<DVec2> {
    if cleared.contains_point(target_xy) {
        return Some(target_xy);
    }
    let mut best_bdry: Option<(DVec2, f64)> = None;
    for reg in &cleared.regions {
        for lp in std::iter::once(&reg.outer_ccw).chain(reg.islands_cw.iter()) {
            if let Some(proj) = lp.project_point(target_xy) {
                let d = proj.distance_mm;
                if best_bdry.is_none_or(|(_, prev_d)| d < prev_d) {
                    best_bdry = Some((proj.point, d));
                }
            }
        }
    }
    let (bdry_pt, dist) = best_bdry?;
    let inward_dir = if dist > 1e-5 {
        (bdry_pt - target_xy) / dist
    } else {
        DVec2::X
    };
    for nudge in [0.03, 0.08, 0.18, 0.35] {
        let cand = bdry_pt + inward_dir * nudge;
        if cleared.contains_point(cand) {
            return Some(cand);
        }
    }
    // Try orthogonal nudges around `bdry_pt` if `target_xy` projected onto a sharp vertex
    for (dx, dy) in [(0.04, 0.0), (-0.04, 0.0), (0.0, 0.04), (0.0, -0.04)] {
        let cand = bdry_pt + DVec2::new(dx, dy);
        if cleared.contains_point(cand) {
            return Some(cand);
        }
    }
    None
}

fn emit_finish_loop(
    ctx: &StrategyExecContext<'_>,
    loop_poly: &ArcPolyline2D,
    link: &PeelLinkContext<'_>,
    builder: &mut ToolpathBuilder,
) {
    let Some((start_xy, _)) = loop_poly.sample_at_fraction(0.0) else {
        return;
    };
    link_to_xy_at_z_cut(ctx, start_xy, link, builder);
    builder.pass_count += 1;
    builder.emit_polyline_2d_z_interp(
        loop_poly,
        link.z_cut,
        link.z_cut,
        link.cut_feed_mm_min,
        MotionRole::FinishCut,
    );
}

fn link_to_xy_at_z_cut(
    ctx: &StrategyExecContext<'_>,
    target_xy: DVec2,
    link: &PeelLinkContext<'_>,
    builder: &mut ToolpathBuilder,
) {
    if let Some(cur) = builder.current_pos() {
        let cur_xy = DVec2::new(cur.x, cur.y);
        let dist = cur_xy.distance(target_xy);
        if dist <= 1e-4 && (cur.z - link.z_cut).abs() <= 1e-4 {
            return;
        }
        // Very short step-over (`<= 1.5 * lead_radius`) strictly inside cleared cavity: direct link at Z
        if (cur.z - link.z_cut).abs() <= link.micro_lift_z + 1e-3
            && dist <= link.stay_down_limit
            && is_chord_inside_region_set(cur_xy, target_xy, link.cleared_cavity)
        {
            if dist <= 2.0 * link.lead_radius_mm && (cur.z - link.z_cut).abs() <= 1e-4 {
                builder.linear_to(
                    DVec3::new(target_xy.x, target_xy.y, link.z_cut),
                    ctx.cutting.lead_feed_mm_min,
                    MotionRole::LinkTransition,
                );
                return;
            }
            // Dogleg-safe Micro-Lift Stay-Down (`G1` linear traverse at `high_feed_mm_min`)!
            let z_lift = (link.z_cut + link.micro_lift_z).min(link.retract_z);
            builder.linear_to(
                DVec3::new(cur_xy.x, cur_xy.y, z_lift),
                link.high_feed_mm_min,
                MotionRole::LeadOut,
            );
            builder.linear_to(
                DVec3::new(target_xy.x, target_xy.y, z_lift),
                link.high_feed_mm_min,
                MotionRole::LinkTransition,
            );
            builder.linear_to(
                DVec3::new(target_xy.x, target_xy.y, link.z_cut),
                ctx.cutting
                    .ramp_feed_mm_min
                    .max(ctx.cutting.plunge_feed_mm_min),
                MotionRole::LeadIn,
            );
            return;
        }
    }

    // Obstructed by an island / uncut stock or initial entry: full safe `G0` retract!
    retract_and_rapid_to_feed_z(ctx, builder, target_xy, link.retract_z);
    builder.linear_to(
        DVec3::new(target_xy.x, target_xy.y, link.z_cut),
        ctx.cutting.plunge_feed_mm_min,
        MotionRole::Plunge,
    );
}

fn try_cleared_tangent_lead_in(
    cut_start: DVec2,
    tan_start: DVec2,
    r: f64,
    cleared: &RegionSet2D,
) -> Option<(DVec2, DVec2, bool)> {
    let t = tan_start.normalize_or_zero();
    if t.length_squared() < 0.5 || r <= 1e-4 {
        return None;
    }
    // Test curving from LEFT of travel (`CCW` arc into cut) first (standard for Climb where cavity is Left!),
    // then RIGHT (`CW` arc into cut for Conventional).
    let left_n = DVec2::new(-t.y, t.x);
    for (normal, ccw) in [(left_n, true), (-left_n, false)] {
        let center = cut_start + normal * r;
        // `60°` gentle tangent lead-in arc
        let ang = std::f64::consts::FRAC_PI_3;
        let radial_end = -normal * r;
        let rot = if ccw { -ang } else { ang };
        let (s, c) = rot.sin_cos();
        let radial_start = DVec2::new(
            radial_end.x * c - radial_end.y * s,
            radial_end.x * s + radial_end.y * c,
        );
        let pre_xy = center + radial_start;
        let mid_rot = 0.5 * rot;
        let (ms, mc) = mid_rot.sin_cos();
        let mid_xy = center
            + DVec2::new(
                radial_end.x * mc - radial_end.y * ms,
                radial_end.x * ms + radial_end.y * mc,
            );
        if cleared.contains_point(pre_xy) && cleared.contains_point(mid_xy) {
            return Some((pre_xy, center, ccw));
        }
    }
    None
}

fn try_cleared_tangent_lead_out(
    cut_end: DVec2,
    tan_end: DVec2,
    r: f64,
    cleared: &RegionSet2D,
) -> Option<(DVec2, DVec2, bool)> {
    let t = tan_end.normalize_or_zero();
    if t.length_squared() < 0.5 || r <= 1e-4 {
        return None;
    }
    let left_n = DVec2::new(-t.y, t.x);
    for (normal, ccw) in [(left_n, true), (-left_n, false)] {
        let center = cut_end + normal * r;
        let ang = std::f64::consts::FRAC_PI_3;
        let radial_start = -normal * r;
        let rot = if ccw { ang } else { -ang };
        let (s, c) = rot.sin_cos();
        let out_xy = center
            + DVec2::new(
                radial_start.x * c - radial_start.y * s,
                radial_start.x * s + radial_start.y * c,
            );
        let mid_rot = 0.5 * rot;
        let (ms, mc) = mid_rot.sin_cos();
        let mid_xy = center
            + DVec2::new(
                radial_start.x * mc - radial_start.y * ms,
                radial_start.x * ms + radial_start.y * mc,
            );
        if cleared.contains_point(out_xy) && cleared.contains_point(mid_xy) {
            return Some((out_xy, center, ccw));
        }
    }
    None
}

fn is_chord_inside_region_set(a: DVec2, b: DVec2, set: &RegionSet2D) -> bool {
    let dist = a.distance(b);
    if dist <= 1e-5 {
        return set.contains_point(a);
    }
    // `clip_scanline_to_region` finds exact ray-line & ray-arc boundary crossings:
    // check whether a single RegionSet2D component contains the entire chord `a -> b` without leaving!
    for reg in &set.regions {
        if !reg.contains_interior_point(a) || !reg.contains_interior_point(b) {
            continue;
        }
        let spans = clip_scanline_to_region(a, b, reg);
        if spans.len() == 1 && spans[0].0.distance(a) <= 2e-3 && spans[0].1.distance(b) <= 2e-3 {
            return true;
        }
    }
    false
}

fn order_chains_from_cursor(chains: &mut Vec<ArcPolyline2D>, cur_pos: Option<DVec3>) {
    if chains.len() <= 1 {
        return;
    }
    let mut cursor = cur_pos.map_or(DVec2::ZERO, |p| DVec2::new(p.x, p.y));
    let mut ordered = Vec::with_capacity(chains.len());
    while !chains.is_empty() {
        let mut best_idx = 0usize;
        let mut best_d2 = f64::INFINITY;
        for (i, c) in chains.iter().enumerate() {
            let s = c.segments.first().map_or(cursor, ArcSegment2D::start);
            let d2 = cursor.distance_squared(s);
            if d2 < best_d2 {
                best_d2 = d2;
                best_idx = i;
            }
        }
        let chosen = chains.remove(best_idx);
        cursor = chosen.segments.last().map_or(cursor, ArcSegment2D::end);
        ordered.push(chosen);
    }
    *chains = ordered;
}

fn retract_and_rapid_to_feed_z(
    ctx: &StrategyExecContext<'_>,
    builder: &mut ToolpathBuilder,
    xy: DVec2,
    retract_z: f64,
) {
    if let Some(cur) = builder.current_pos()
        && cur.z < retract_z - 1e-6
    {
        builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
    }
    builder.rapid_to(DVec3::new(xy.x, xy.y, retract_z));
    builder.rapid_to(DVec3::new(xy.x, xy.y, ctx.heights.feed_z_wcs));
}

fn rect_region_2d(x0: f64, y0: f64, x1: f64, y1: f64) -> RegionSet2D {
    if x1 - x0 <= 1e-4 || y1 - y0 <= 1e-4 {
        return RegionSet2D::empty();
    }
    let p0 = DVec2::new(x0, y0);
    let p1 = DVec2::new(x1, y0);
    let p2 = DVec2::new(x1, y1);
    let p3 = DVec2::new(x0, y1);
    let rect = ArcPolyline2D {
        segments: vec![
            ArcSegment2D::Line { start: p0, end: p1 },
            ArcSegment2D::Line { start: p1, end: p2 },
            ArcSegment2D::Line { start: p2, end: p3 },
            ArcSegment2D::Line { start: p3, end: p0 },
        ],
        closed: true,
    };
    RegionSet2D::from_single(PocketRegion2D::new(rect, Vec::new()))
}
