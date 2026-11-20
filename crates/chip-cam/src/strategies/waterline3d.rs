//! **3D Waterline / Z-Level Machining** (`OperationKind::Waterline3D`).
//!
//! Slices imported triangle meshes (`STL`/`OBJ`/`3MF`) and tessellated STEP B-Rep solids using
//! exact analytical `f64` [`push_cutter_region_z`] avoidance boundaries, with steep-slope
//! filtering, adaptive cusp-limited intermediate Z-stepdowns, flat-plateau detection,
//! 2D containment boundary clipping, and collision-free contour ramping.

use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3};

use crate::{
    cl_data::MotionRole,
    params::{Waterline3DParams, WaterlineOrderMode},
    rest2d::RegionSet2D,
    strategies::StrategyExecContext,
    surface3d::{
        SurfaceCutterProfile, SurfaceMesh3D,
        levels::{
            clip_polyline_to_region, detect_flat_plateaus, points_to_polyline,
            resolve_3d_containment_region, reverse_arc_polyline, sample_seg,
            sample_surface_slope_deg,
        },
        push_cutter_region_z,
    },
    toolpath::{ToolpathBuilder, XyArcMove},
    wcs::CamError,
};

struct WaterlineContour {
    z_wcs: f64,
    polyline: ArcPolyline2D,
    centroid: DVec2,
    avoid_idx: usize,
}

/// Generates **3D Waterline / Z-Level** toolpaths across `ctx.doc` (STEP solid or triangle mesh).
pub fn generate_waterline3d(
    ctx: &StrategyExecContext<'_>,
    params: &Waterline3DParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    ctx.check_cancel()?;
    if !ctx.doc.has_3d_geometry() {
        return Err(CamError::EmptyCadDocument);
    }

    let surface = SurfaceMesh3D::from_cad_document(ctx.doc, &[], &ctx.setup.frame);
    if surface.triangles.is_empty() || !surface.aabb_wcs.is_valid() {
        return Err(CamError::EmptyCadDocument);
    }

    let radial_stock = ctx.passes.stock_to_leave_radial.value_mm.max(0.0);
    let axial_stock = ctx.passes.stock_to_leave_axial.value_mm.max(0.0);
    let cutter =
        SurfaceCutterProfile::from_tool_geometry(&ctx.tool.geometry, radial_stock, axial_stock);
    let tool_r = 0.5 * ctx.tool.geometry.cutting_diameter_mm().max(0.1);

    let z_top = ctx
        .heights
        .top_z_wcs
        .min(surface.aabb_wcs.max.z + axial_stock);
    let z_bottom = ctx
        .heights
        .bottom_z_wcs
        .max(surface.aabb_wcs.min.z + axial_stock);
    if z_top <= z_bottom + 1e-4 {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    let containment = resolve_3d_containment_region(
        &surface,
        ctx.setup,
        &ctx.selection,
        params.boundary_mode,
        params.tool_containment,
        params.boundary_offset.value_mm,
        tool_r,
    );
    if containment.is_empty() {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    // 1. Build primary + flat-plateau + adaptive cusp-limited Z levels
    let max_step = ctx.passes.max_stepdown.value_mm.clamp(0.05, 50.0);
    let min_step = params.min_stepdown.value_mm.clamp(0.05, max_step);
    let max_cusp = params.max_cusp_height.value_mm.clamp(0.005, 2.0);

    let mut z_levels = build_base_z_levels(z_top, z_bottom, max_step);
    if params.detect_flats {
        let flats =
            detect_flat_plateaus(&surface, z_bottom - axial_stock, z_top - axial_stock, 4.0);
        for z_flat in flats {
            let z_target = z_flat + axial_stock;
            if z_target >= z_bottom - 1e-4 && z_target <= z_top + 1e-4 {
                insert_unique_z(&mut z_levels, z_target, 0.02);
            }
        }
    }
    z_levels.sort_by(|a, b| b.total_cmp(a));

    // Adaptive intermediate Z-stepdown refinement on shallow slopes
    if params.adaptive_stepdown && z_levels.len() >= 2 {
        let eff_corner_r = match cutter.shape {
            crate::surface3d::SurfaceCutterShape::Ball { radius_mm } => radius_mm,
            crate::surface3d::SurfaceCutterShape::BullNose {
                corner_radius_mm, ..
            } => corner_radius_mm,
            _ => 0.0,
        };

        let mut refined = Vec::with_capacity(z_levels.len() * 2);
        let mut prev_z = z_levels[0];
        let mut prev_avoid = push_cutter_region_z(&surface, &cutter, prev_z);
        refined.push(prev_z);

        for &cur_z in &z_levels[1..] {
            ctx.check_cancel()?;
            let cur_avoid = push_cutter_region_z(&surface, &cutter, cur_z);
            let dz = (prev_z - cur_z).abs();
            if dz > min_step * 1.15 && !cur_avoid.is_empty() {
                let r_prev = (prev_avoid.total_area_mm2() / std::f64::consts::PI).sqrt();
                let r_cur = (cur_avoid.total_area_mm2() / std::f64::consts::PI).sqrt();
                let dr = (r_cur - r_prev).abs();
                let chord = dr.hypot(dz);
                let est_cusp = if eff_corner_r > 1e-3 {
                    (chord * chord) / (8.0 * eff_corner_r)
                } else {
                    // For flat endmills, limit horizontal stair-step width
                    dr * 0.25
                };
                if est_cusp > max_cusp && dr > 0.1 {
                    let ratio = (est_cusp / max_cusp).sqrt().ceil() as usize;
                    let max_sub = (dz / min_step).floor().max(1.0) as usize;
                    let n_sub = ratio.clamp(2, max_sub.min(8));
                    for s in 1..n_sub {
                        let frac = s as f64 / n_sub as f64;
                        refined.push(prev_z - frac * dz);
                    }
                }
            }
            refined.push(cur_z);
            prev_z = cur_z;
            prev_avoid = cur_avoid;
        }
        z_levels = refined;
    }

    // 2. Slice each Z level with exact `push_cutter_region_z`, clip to containment, & filter by slope
    let slope_min = params.min_slope_deg.clamp(0.0, 90.0);
    let slope_max = params.max_slope_deg.clamp(slope_min, 90.0);
    let filter_slopes = slope_min > 0.5 || slope_max < 89.5;
    let sample_step = (0.25 * tool_r).clamp(0.25, 1.5);

    let mut avoids_by_level: Vec<RegionSet2D> = Vec::with_capacity(z_levels.len());
    let mut all_contours: Vec<WaterlineContour> = Vec::new();
    let mut bvh_scratch = Vec::with_capacity(64);

    for &z_k in &z_levels {
        ctx.check_cancel()?;
        let avoid = push_cutter_region_z(&surface, &cutter, z_k);
        let avoid_idx = avoids_by_level.len();
        if avoid.is_empty() {
            avoids_by_level.push(avoid);
            continue;
        }

        for reg in &avoid.regions {
            // External boss boundary (`outer_ccw`): Climb = CW (reverse CCW), Conventional = CCW
            let outer_oriented = if params.climb_milling {
                reverse_arc_polyline(&reg.outer_ccw)
            } else {
                reg.outer_ccw.clone()
            };
            collect_waterline_chains(
                &outer_oriented,
                z_k,
                avoid_idx,
                &containment,
                &surface,
                &cutter,
                z_bottom,
                filter_slopes,
                slope_min,
                slope_max,
                sample_step,
                &mut bvh_scratch,
                &mut all_contours,
            );

            // Internal cavity walls (`islands_cw`): Climb = CCW (reverse CW), Conventional = CW
            for isl in &reg.islands_cw {
                let isl_oriented = if params.climb_milling {
                    reverse_arc_polyline(isl)
                } else {
                    isl.clone()
                };
                collect_waterline_chains(
                    &isl_oriented,
                    z_k,
                    avoid_idx,
                    &containment,
                    &surface,
                    &cutter,
                    z_bottom,
                    filter_slopes,
                    slope_min,
                    slope_max,
                    sample_step,
                    &mut bvh_scratch,
                    &mut all_contours,
                );
            }
        }
        avoids_by_level.push(avoid);
    }

    if all_contours.is_empty() {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    // 3. Order contours (`ByLevel` or `ByDepthRegion`)
    let ordered = order_waterline_contours(all_contours, params.order_mode, tool_r * 4.0);

    // 4. Emit collision-free ramped / arc-linked toolpath segments
    let retract_z = ctx.safe_retract_z();
    let stock_top = ctx.setup.stock.aabb_wcs().max.z;
    let cut_feed = ctx.cutting.cutting_feed_mm_min;
    let ramp_feed = ctx.cutting.ramp_feed_mm_min;
    let plunge_feed = ctx.cutting.plunge_feed_mm_min;
    let stay_down_limit = ctx
        .linking
        .safe_stay_down_distance
        .value_mm
        .max(tool_r * 4.0);
    let ramp_tan = params.ramp_angle_deg.clamp(0.5, 20.0).to_radians().tan();

    let mut cur_pos: Option<DVec3> = None;
    let mut cur_avoid_idx: Option<usize> = None;

    for wc in &ordered {
        ctx.check_cancel()?;
        let Some(first_seg) = wc.polyline.segments.first() else {
            continue;
        };
        let start_xy = first_seg.start();
        let avoid = &avoids_by_level[wc.avoid_idx];

        let mut stayed_down = false;
        if let (Some(prev_p), Some(prev_aidx)) = (cur_pos, cur_avoid_idx)
            && prev_p.z >= wc.z_wcs - 1e-4
        {
            let prev_xy = DVec2::new(prev_p.x, prev_p.y);
            let dist_xy = prev_xy.distance(start_xy);
            if dist_xy <= stay_down_limit {
                let prev_avoid = &avoids_by_level[prev_aidx];
                if segment_clear_of_avoid(prev_xy, start_xy, prev_avoid) {
                    // Move horizontally at `prev_p.z` (strictly clear of `prev_avoid`), then ramp to `wc.z_wcs`!
                    if dist_xy > 1e-4 {
                        builder.linear_to(
                            DVec3::new(start_xy.x, start_xy.y, prev_p.z),
                            ramp_feed,
                            MotionRole::LinkTransition,
                        );
                    }
                    let dz = (prev_p.z - wc.z_wcs).max(0.0);
                    if dz > 1e-4 {
                        emit_contour_ramp(
                            &wc.polyline,
                            prev_p.z,
                            wc.z_wcs,
                            ramp_tan,
                            ramp_feed,
                            builder,
                        );
                    }
                    stayed_down = true;
                }
            }
        }

        if !stayed_down {
            if let Some(prev_p) = cur_pos
                && prev_p.z < retract_z - 1e-4
            {
                builder.rapid_to(DVec3::new(prev_p.x, prev_p.y, retract_z));
            }
            // Approach safely above stock, feed down to ramp start height, then contour-ramp into `wc.z_wcs`
            builder.rapid_to(DVec3::new(start_xy.x, start_xy.y, retract_z));
            let ramp_top_z = (wc.z_wcs + max_step.min(2.5))
                .min(stock_top + 0.5)
                .max(wc.z_wcs + 0.25)
                .min(retract_z);
            if ramp_top_z < retract_z - 1e-4 {
                builder.linear_to(
                    DVec3::new(start_xy.x, start_xy.y, ramp_top_z),
                    plunge_feed,
                    MotionRole::Plunge,
                );
            }
            emit_contour_ramp(
                &wc.polyline,
                ramp_top_z,
                wc.z_wcs,
                ramp_tan,
                ramp_feed,
                builder,
            );
        }

        // Emit full contour pass at constant `wc.z_wcs`
        let end_xy = emit_polyline_at_z(
            &wc.polyline,
            wc.z_wcs,
            cut_feed,
            MotionRole::FinishCut,
            builder,
        );
        cur_pos = Some(DVec3::new(end_xy.x, end_xy.y, wc.z_wcs));
        cur_avoid_idx = Some(wc.avoid_idx);

        // Keep `avoid` reference used for collision-safe transition check above
        let _ = avoid;
    }

    if let Some(last_p) = cur_pos
        && last_p.z < retract_z - 1e-4
    {
        builder.rapid_to(DVec3::new(last_p.x, last_p.y, retract_z));
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn collect_waterline_chains(
    loop_poly: &ArcPolyline2D,
    z_k: f64,
    avoid_idx: usize,
    containment: &RegionSet2D,
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    z_floor: f64,
    filter_slopes: bool,
    slope_min: f64,
    slope_max: f64,
    sample_step: f64,
    bvh_scratch: &mut Vec<usize>,
    out: &mut Vec<WaterlineContour>,
) {
    let clipped = clip_polyline_to_region(loop_poly, containment, sample_step);
    for poly in clipped {
        if filter_slopes {
            let filtered = filter_polyline_by_slope(
                &poly,
                surface,
                cutter,
                z_floor,
                slope_min,
                slope_max,
                sample_step,
                bvh_scratch,
            );
            for sub in filtered {
                if let Some(c) = polyline_centroid(&sub) {
                    out.push(WaterlineContour {
                        z_wcs: z_k,
                        polyline: sub,
                        centroid: c,
                        avoid_idx,
                    });
                }
            }
        } else if let Some(c) = polyline_centroid(&poly) {
            out.push(WaterlineContour {
                z_wcs: z_k,
                polyline: poly,
                centroid: c,
                avoid_idx,
            });
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn filter_polyline_by_slope(
    poly: &ArcPolyline2D,
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    z_floor: f64,
    slope_min: f64,
    slope_max: f64,
    sample_step: f64,
    scratch: &mut Vec<usize>,
) -> Vec<ArcPolyline2D> {
    let mut chains: Vec<Vec<DVec2>> = Vec::new();
    let mut current: Vec<DVec2> = Vec::new();
    let mut all_pass = true;

    for seg in &poly.segments {
        let len = seg.length();
        let n = (len / sample_step).ceil().max(1.0) as usize;
        for k in 0..n {
            let t0 = k as f64 / n as f64;
            let t1 = (k + 1) as f64 / n as f64;
            let mid = sample_seg(seg, 0.5 * (t0 + t1));
            let slope = sample_surface_slope_deg(surface, cutter, mid, z_floor, scratch);
            if slope >= slope_min - 0.5 && slope <= slope_max + 0.5 {
                let p0 = sample_seg(seg, t0);
                let p1 = sample_seg(seg, t1);
                if current.is_empty() {
                    current.push(p0);
                }
                if current
                    .last()
                    .is_none_or(|l| l.distance_squared(p1) > 1e-10)
                {
                    current.push(p1);
                }
            } else {
                all_pass = false;
                if current.len() >= 2 {
                    chains.push(std::mem::take(&mut current));
                } else {
                    current.clear();
                }
            }
        }
    }
    if all_pass {
        return vec![poly.clone()];
    }
    if current.len() >= 2 {
        chains.push(current);
    }

    if poly.closed
        && chains.len() >= 2
        && let (Some(first_pt), Some(last_pt)) = (
            chains.first().and_then(|c| c.first()).copied(),
            chains.last().and_then(|c| c.last()).copied(),
        )
        && first_pt.distance(last_pt) <= 1e-3
        && let Some(mut tail) = chains.pop()
    {
        tail.pop();
        tail.extend(chains[0].iter().copied());
        chains[0] = tail;
    }

    chains
        .into_iter()
        .filter_map(|pts| points_to_polyline(&pts, false))
        .collect()
}

fn order_waterline_contours(
    mut contours: Vec<WaterlineContour>,
    order_mode: WaterlineOrderMode,
    cluster_radius: f64,
) -> Vec<WaterlineContour> {
    match order_mode {
        WaterlineOrderMode::ByLevel => {
            // Already grouped top-down by Z; apply greedy nearest-neighbor inside each Z band
            let mut out = Vec::with_capacity(contours.len());
            let mut last_xy = DVec2::ZERO;
            let mut i = 0;
            while i < contours.len() {
                let z = contours[i].z_wcs;
                let mut j = i + 1;
                while j < contours.len() && (contours[j].z_wcs - z).abs() <= 1e-4 {
                    j += 1;
                }
                let mut band: Vec<WaterlineContour> = contours.drain(i..j).collect();
                while !band.is_empty() {
                    let best_idx = band
                        .iter()
                        .enumerate()
                        .min_by(|(_, a), (_, b)| {
                            let da = a.polyline.segments[0].start().distance_squared(last_xy);
                            let db = b.polyline.segments[0].start().distance_squared(last_xy);
                            da.total_cmp(&db)
                        })
                        .map_or(0, |(idx, _)| idx);
                    let picked = band.remove(best_idx);
                    if let Some(last_seg) = picked.polyline.segments.last() {
                        last_xy = last_seg.end();
                    }
                    out.push(picked);
                }
                i = 0;
            }
            out
        }
        WaterlineOrderMode::ByDepthRegion => {
            // Cluster vertically stacked contours by centroid proximity and emit each stack top-down
            let mut groups: Vec<Vec<WaterlineContour>> = Vec::new();
            let rad_sq = cluster_radius.max(2.0).powi(2);
            for wc in contours {
                let target_group = groups.iter().position(|g| {
                    g.last()
                        .is_some_and(|prev| prev.centroid.distance_squared(wc.centroid) <= rad_sq)
                });
                if let Some(gi) = target_group {
                    groups[gi].push(wc);
                } else {
                    groups.push(vec![wc]);
                }
            }
            let mut out = Vec::new();
            for mut g in groups {
                g.sort_by(|a, b| b.z_wcs.total_cmp(&a.z_wcs));
                out.extend(g);
            }
            out
        }
    }
}

fn segment_clear_of_avoid(a: DVec2, b: DVec2, avoid: &RegionSet2D) -> bool {
    if avoid.is_empty() {
        return true;
    }
    let dist = a.distance(b);
    let n = (dist / 0.5).ceil().clamp(2.0, 64.0) as usize;
    for k in 1..n {
        let t = k as f64 / n as f64;
        let pt = a.lerp(b, t);
        if avoid.contains_point(pt) && avoid.min_distance_to_boundary(pt) > 0.02 {
            return false;
        }
    }
    true
}

/// Emits a smooth contour-following Z ramp descending from `z_start` down to `z_target`
/// along `poly` at slope `ramp_tan = tan(ramp_angle)`, returning to `poly`'s start point at `z_target`.
fn emit_contour_ramp(
    poly: &ArcPolyline2D,
    z_start: f64,
    z_target: f64,
    ramp_tan: f64,
    ramp_feed: f64,
    builder: &mut ToolpathBuilder,
) {
    let dz = z_start - z_target;
    if dz <= 1e-4 {
        return;
    }
    let total_len: f64 = poly.segments.iter().map(ArcSegment2D::length).sum();
    if total_len <= 1e-3 || !poly.closed {
        // Open or tiny contour: ramp along the first segment (or direct plunge if minuscule)
        if let Some(first) = poly.segments.first() {
            let p0 = first.start();
            let p1 = first.end();
            if p0.distance(p1) > 0.25 {
                let z_mid = 0.5 * (z_start + z_target);
                builder.linear_to(DVec3::new(p1.x, p1.y, z_mid), ramp_feed, MotionRole::Ramp);
                builder.linear_to(
                    DVec3::new(p0.x, p0.y, z_target),
                    ramp_feed,
                    MotionRole::Ramp,
                );
                return;
            }
        }
        if let Some(first) = poly.segments.first() {
            let p0 = first.start();
            builder.linear_to(
                DVec3::new(p0.x, p0.y, z_target),
                ramp_feed,
                MotionRole::Plunge,
            );
        }
        return;
    }

    // Required horizontal distance along closed contour to descend `dz` at `ramp_angle`:
    let desired_horiz = (dz / ramp_tan.max(0.01)).clamp(total_len.min(2.0), total_len * 2.0);
    let mut traveled = 0.0f64;

    // Descend along contour segments until reaching `z_target`, then finish back to `start_xy` at `z_target`
    while traveled < desired_horiz - 1e-5 {
        for seg in &poly.segments {
            let seg_len = seg.length();
            if seg_len <= 1e-7 {
                continue;
            }
            let n = (seg_len / 1.0).ceil().max(1.0) as usize;
            for k in 1..=n {
                let t = k as f64 / n as f64;
                let pt = sample_seg(seg, t);
                traveled += seg_len / n as f64;
                let frac = (traveled / desired_horiz).min(1.0);
                let z_cur = z_start - frac * dz;
                builder.linear_to(DVec3::new(pt.x, pt.y, z_cur), ramp_feed, MotionRole::Ramp);
                if frac >= 1.0 - 1e-9 {
                    break;
                }
            }
            if traveled >= desired_horiz - 1e-5 {
                break;
            }
        }
        if traveled <= 1e-6 {
            break;
        }
    }

    // Complete remainder of the closed loop at `z_target` back to start so the final cut pass starts cleanly
    let rem_pos = traveled % total_len;
    if rem_pos > 1e-3 && total_len - rem_pos > 1e-3 {
        let mut acc = 0.0;
        for seg in &poly.segments {
            let sl = seg.length();
            let next_acc = acc + sl;
            if next_acc > rem_pos + 1e-5 {
                let end = seg.end();
                builder.linear_to(
                    DVec3::new(end.x, end.y, z_target),
                    ramp_feed,
                    MotionRole::FinishCut,
                );
            }
            acc = next_acc;
        }
    }
}

fn emit_polyline_at_z(
    poly: &ArcPolyline2D,
    z_wcs: f64,
    feed: f64,
    role: MotionRole,
    builder: &mut ToolpathBuilder,
) -> DVec2 {
    let mut last = poly
        .segments
        .first()
        .map_or(DVec2::ZERO, ArcSegment2D::start);
    for seg in &poly.segments {
        match *seg {
            ArcSegment2D::Line { end, .. } => {
                builder.linear_to(DVec3::new(end.x, end.y, z_wcs), feed, role);
                last = end;
            }
            ArcSegment2D::Arc {
                end,
                center,
                radius,
                ccw,
                ..
            } => {
                if radius > 0.02 && last.distance(end) > 1e-5 {
                    builder.arc_xy_to(XyArcMove {
                        end_wcs: DVec3::new(end.x, end.y, z_wcs),
                        center_xy: center,
                        radius_mm: radius,
                        ccw,
                        turns: 0,
                        feed_mm_min: feed,
                        role,
                    });
                } else {
                    builder.linear_to(DVec3::new(end.x, end.y, z_wcs), feed, role);
                }
                last = end;
            }
        }
    }
    last
}

fn polyline_centroid(poly: &ArcPolyline2D) -> Option<DVec2> {
    if poly.segments.is_empty() {
        return None;
    }
    let mut sum = DVec2::ZERO;
    for seg in &poly.segments {
        sum += seg.start();
    }
    Some(sum / poly.segments.len() as f64)
}

fn build_base_z_levels(z_top: f64, z_bottom: f64, max_step: f64) -> Vec<f64> {
    let span = (z_top - z_bottom).max(0.0);
    let n = (span / max_step).ceil().max(1.0) as usize;
    let mut levels = Vec::with_capacity(n);
    for k in 1..=n {
        let z = z_top - (k as f64 / n as f64) * span;
        levels.push(z);
    }
    levels
}

fn insert_unique_z(levels: &mut Vec<f64>, z: f64, tol: f64) {
    if !levels.iter().any(|&existing| (existing - z).abs() <= tol) {
        levels.push(z);
    }
}
