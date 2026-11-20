//! **3D Constant-Scallop Finishing** (`OperationKind::Scallop3D`).
//!
//! Computes a slope-weighted 3D geodesic distance field `D_3d(x, y)` inward from the containment
//! boundary via Sethian's Fast Marching Method (Eikonal `|∇D_3d| = sec(θ)`), extracts marching-
//! squares iso-contours spaced by constant 3D arc-length `s_3d` (producing tighter XY spacing on
//! steep walls than on shallow crests for uniform cusp height), and projects every vertex onto
//! exact `f64` `drop_cutter_z`.

use std::{
    cmp::Ordering,
    collections::{BinaryHeap, HashMap},
};

use chip_math::{DVec2, DVec3};
use rayon::prelude::*;

use crate::{
    cl_data::MotionRole,
    params::Scallop3DParams,
    rest2d::RegionSet2D,
    strategies::StrategyExecContext,
    surface3d::{
        RestMaskGrid2D, SurfaceCutterProfile, SurfaceLinkConfig, SurfaceMesh3D, build_3d_rest_mask,
        drop_cutter_z_with_scratch, emit_final_surface_retract, emit_surface_safe_link,
        levels::{resolve_3d_containment_region, sample_surface_slope_deg},
        refine_convex_chords_3d, region_aabb_2d, simplify_3d_polyline_rdp,
        stepover_for_cusp_height,
    },
    toolpath::ToolpathBuilder,
    wcs::CamError,
};

/// Min-heap entry for deterministic Fast Marching Eikonal propagation.
#[derive(Debug, Clone, Copy)]
struct FmmNode {
    dist: f64,
    idx: usize,
}

impl PartialEq for FmmNode {
    fn eq(&self, other: &Self) -> bool {
        self.idx == other.idx && self.dist.to_bits() == other.dist.to_bits()
    }
}

impl Eq for FmmNode {}

impl PartialOrd for FmmNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for FmmNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reversed for min-heap behavior in `BinaryHeap`, with deterministic `idx` tie-break
        other
            .dist
            .total_cmp(&self.dist)
            .then_with(|| other.idx.cmp(&self.idx))
    }
}

/// Generates **3D Constant-Scallop Finishing** toolpaths across `ctx.doc`.
pub fn generate_scallop3d(
    ctx: &StrategyExecContext<'_>,
    params: &Scallop3DParams,
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
    let tool_d = ctx.tool.geometry.cutting_diameter_mm().max(0.1);
    let tool_r = 0.5 * tool_d;

    let z_bottom = ctx
        .heights
        .bottom_z_wcs
        .max(surface.aabb_wcs.min.z + axial_stock);
    let z_top = ctx
        .heights
        .top_z_wcs
        .min(surface.aabb_wcs.max.z + axial_stock);
    if z_top < z_bottom - 1e-4 {
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
    let Some(bounds) = region_aabb_2d(&containment) else {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    };

    let rest_mask = build_3d_rest_mask(
        &surface,
        &cutter,
        &containment,
        &ctx.passes.rest_machining,
        ctx.upstream_rest,
        z_bottom,
    );

    let min_step = params.min_stepover.value_mm.clamp(0.05, 0.75 * tool_d);
    let max_step = params
        .max_stepover
        .value_mm
        .clamp(min_step, (0.75 * tool_d).max(min_step));
    let raw_cusp_step = stepover_for_cusp_height(&cutter, params.cusp_height.value_mm, max_step);
    let target_s_3d = raw_cusp_step.clamp(min_step, max_step);
    let smooth_tol = params.smoothing_tolerance.value_mm.clamp(0.001, 0.010);

    let slope_min = params.min_slope_deg.clamp(0.0, 90.0);
    let slope_max = params.max_slope_deg.clamp(slope_min, 90.0);
    let filter_slopes = slope_min > 0.1 || slope_max < 89.9;

    // 1. Build padded 2D grid over `containment` and sample exact `drop_cutter_z` in parallel
    let w = (bounds.max.x - bounds.min.x).max(1.0);
    let h_span = (bounds.max.y - bounds.min.y).max(1.0);
    let base_cell = (0.35 * target_s_3d).clamp(0.25, 0.75);
    let cell = base_cell.max(((w * h_span) / 42_000.0).sqrt());
    let min_x = bounds.min.x - cell;
    let min_y = bounds.min.y - cell;
    let nx = ((w + 2.0 * cell) / cell).ceil() as usize + 1;
    let ny = ((h_span + 2.0 * cell) / cell).ceil() as usize + 1;
    let total = nx * ny;

    let (inside, z_grid): (Vec<bool>, Vec<f64>) = (0..ny)
        .into_par_iter()
        .map(|iy| {
            let y = min_y + iy as f64 * cell;
            let mut scratch = Vec::with_capacity(64);
            let mut row_in = vec![false; nx];
            let mut row_z = vec![z_bottom; nx];
            for ix in 0..nx {
                let x = min_x + ix as f64 * cell;
                let pt = DVec2::new(x, y);
                let is_in = containment.contains_point(pt);
                row_in[ix] = is_in;
                if is_in {
                    row_z[ix] =
                        drop_cutter_z_with_scratch(&surface, &cutter, x, y, z_bottom, &mut scratch)
                            .unwrap_or(z_bottom)
                            .clamp(z_bottom, z_top);
                }
            }
            (row_in, row_z)
        })
        .collect::<Vec<_>>()
        .into_iter()
        .fold(
            (Vec::with_capacity(total), Vec::with_capacity(total)),
            |(mut acc_in, mut acc_z), (ri, rz)| {
                acc_in.extend(ri);
                acc_z.extend(rz);
                (acc_in, acc_z)
            },
        );

    ctx.check_cancel()?;

    // 2. Compute local surface arc-length weight `w(x, y) = sec(θ) = sqrt(1 + (dz/dx)^2 + (dz/dy)^2)`
    //    clamped to `[1.0, 4.0]` so steep walls receive proportionally tighter XY stepovers without
    //    stacking degenerate infinite passes on 90° cliffs.
    let mut weight = vec![1.0_f64; total];
    for iy in 0..ny {
        for ix in 0..nx {
            let idx = iy * nx + ix;
            if !inside[idx] {
                continue;
            }
            let zc = z_grid[idx];
            let dz_dx = {
                let has_l = ix > 0 && inside[idx - 1];
                let has_r = ix + 1 < nx && inside[idx + 1];
                match (has_l, has_r) {
                    (true, true) => (z_grid[idx + 1] - z_grid[idx - 1]) / (2.0 * cell),
                    (true, false) => (zc - z_grid[idx - 1]) / cell,
                    (false, true) => (z_grid[idx + 1] - zc) / cell,
                    (false, false) => 0.0,
                }
            };
            let dz_dy = {
                let has_d = iy > 0 && inside[idx - nx];
                let has_u = iy + 1 < ny && inside[idx + nx];
                match (has_d, has_u) {
                    (true, true) => (z_grid[idx + nx] - z_grid[idx - nx]) / (2.0 * cell),
                    (true, false) => (zc - z_grid[idx - nx]) / cell,
                    (false, true) => (z_grid[idx + nx] - zc) / cell,
                    (false, false) => 0.0,
                }
            };
            weight[idx] = (1.0 + dz_dx * dz_dx + dz_dy * dz_dy).sqrt().clamp(1.0, 4.0);
        }
    }

    // 3. Fast Marching Eikonal solver from `∂containment` inward
    let dist = solve_eikonal_fmm(&inside, &weight, &containment, min_x, min_y, cell, nx, ny);

    let max_d = dist
        .iter()
        .copied()
        .filter(|d| d.is_finite() && *d > 0.0)
        .fold(0.0_f64, f64::max);
    if max_d <= 1e-3 {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    // Build iso-contour distance levels spaced by `target_s_3d`
    let mut levels = Vec::new();
    let mut d_cur = 0.5 * target_s_3d;
    while d_cur < max_d - 0.1 * target_s_3d {
        levels.push(d_cur);
        d_cur += target_s_3d;
    }
    if levels.is_empty() {
        levels.push(0.5 * max_d);
    }
    if params.inside_out {
        levels.reverse();
    }

    // 4. Extract iso-contours per level in parallel and project onto exact `f64` `drop_cutter_z`
    let level_chains: Vec<Vec<Vec<DVec3>>> = levels
        .par_iter()
        .map(|&iso| {
            extract_and_project_iso_level(
                iso,
                &dist,
                min_x,
                min_y,
                cell,
                nx,
                ny,
                &surface,
                &cutter,
                &containment,
                rest_mask.as_ref(),
                z_bottom,
                z_top,
                params.climb_milling,
                filter_slopes,
                slope_min,
                slope_max,
                smooth_tol,
            )
        })
        .collect();

    // 5. Emit passes sequentially with collision-checked stay-down links
    let retract_z = ctx.safe_retract_z();
    let stock_top_z = ctx.setup.stock.aabb_wcs().max.z;
    let cut_feed = ctx.cutting.cutting_feed_mm_min;
    let lead_feed = ctx.cutting.lead_feed_mm_min.max(0.4 * cut_feed);
    let plunge_feed = ctx.cutting.plunge_feed_mm_min.max(0.25 * cut_feed);
    let stay_down_limit = ctx
        .linking
        .safe_stay_down_distance
        .value_mm
        .clamp(target_s_3d * 1.5, (3.0 * tool_d).max(target_s_3d * 2.5));

    let link_cfg = SurfaceLinkConfig {
        retract_z,
        stock_top_z,
        z_floor: z_bottom,
        stay_down_limit_mm: stay_down_limit,
        sample_step_mm: (0.4 * cell).clamp(0.15, 0.4),
        lead_feed,
        plunge_feed,
    };

    let mut bvh_scratch = Vec::with_capacity(64);
    let mut emitted_passes = 0usize;

    for mut chains in level_chains {
        ctx.check_cancel()?;
        // Greedy nearest-chain ordering within each iso-level
        while !chains.is_empty() {
            let best_idx = if let Some(cur) = builder.current_pos() {
                let cur_xy = DVec2::new(cur.x, cur.y);
                chains
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| {
                        let da = DVec2::new(a[0].x, a[0].y).distance_squared(cur_xy);
                        let db = DVec2::new(b[0].x, b[0].y).distance_squared(cur_xy);
                        da.total_cmp(&db)
                    })
                    .map_or(0, |(i, _)| i)
            } else {
                0
            };

            let mut chain = chains.swap_remove(best_idx);
            if chain.len() < 2 {
                continue;
            }

            // If closed loop, rotate seam to point nearest current tool position to minimize linking
            if chain.len() >= 4
                && chain[0].distance(chain[chain.len() - 1]) <= 0.1
                && let Some(cur) = builder.current_pos()
            {
                let cur_xy = DVec2::new(cur.x, cur.y);
                let n_unique = chain.len() - 1;
                if let Some((best_k, _)) =
                    chain[..n_unique]
                        .iter()
                        .enumerate()
                        .min_by(|(_, a), (_, b)| {
                            let da = DVec2::new(a.x, a.y).distance_squared(cur_xy);
                            let db = DVec2::new(b.x, b.y).distance_squared(cur_xy);
                            da.total_cmp(&db)
                        })
                    && best_k > 0
                {
                    let mut rotated = Vec::with_capacity(chain.len());
                    rotated.extend_from_slice(&chain[best_k..n_unique]);
                    rotated.extend_from_slice(&chain[..best_k]);
                    rotated.push(rotated[0]);
                    chain = rotated;
                }
            }

            emit_surface_safe_link(
                &surface,
                &cutter,
                chain[0],
                &link_cfg,
                &mut bvh_scratch,
                builder,
            );
            for &pt in &chain[1..] {
                builder.linear_to(pt, cut_feed, MotionRole::FinishCut);
            }
            builder.pass_count += 1;
            emitted_passes += 1;
        }
    }

    if emitted_passes == 0 {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    emit_final_surface_retract(retract_z, lead_feed, builder);
    Ok(())
}

/// Solves `|∇D| = weight` inward from `∂containment` using Sethian's 2D Fast Marching Method.
#[allow(clippy::too_many_arguments)]
fn solve_eikonal_fmm(
    inside: &[bool],
    weight: &[f64],
    containment: &RegionSet2D,
    min_x: f64,
    min_y: f64,
    cell: f64,
    nx: usize,
    ny: usize,
) -> Vec<f64> {
    let total = nx * ny;
    let mut dist = vec![f64::INFINITY; total];
    let mut accepted = vec![false; total];
    let mut heap = BinaryHeap::new();

    // Outside cells get negative signed distance (`-0.5 * cell`) so marching-squares iso-contours
    // at `d > 0` never leak outside `containment`.
    for iy in 0..ny {
        for ix in 0..nx {
            let idx = iy * nx + ix;
            if !inside[idx] {
                dist[idx] = -0.5 * cell;
                accepted[idx] = true;
            }
        }
    }

    // Seed interior cells bordering at least one outside 4-neighbor with exact 3D boundary distance
    for iy in 0..ny {
        for ix in 0..nx {
            let idx = iy * nx + ix;
            if !inside[idx] {
                continue;
            }
            let touches_outside = (ix > 0 && !inside[idx - 1])
                || (ix + 1 < nx && !inside[idx + 1])
                || (iy > 0 && !inside[idx - nx])
                || (iy + 1 < ny && !inside[idx + nx]);
            if touches_outside {
                let pt = DVec2::new(min_x + ix as f64 * cell, min_y + iy as f64 * cell);
                let d_xy = containment.min_distance_to_boundary(pt).clamp(1e-4, cell);
                let d0 = d_xy * weight[idx];
                dist[idx] = d0;
                heap.push(FmmNode { dist: d0, idx });
            }
        }
    }

    while let Some(FmmNode { dist: d_cur, idx }) = heap.pop() {
        if accepted[idx] || d_cur > dist[idx] + 1e-12 {
            continue;
        }
        accepted[idx] = true;

        let ix = idx % nx;
        let iy = idx / nx;
        let neighbors = [
            (ix > 0).then(|| idx - 1),
            (ix + 1 < nx).then(|| idx + 1),
            (iy > 0).then(|| idx - nx),
            (iy + 1 < ny).then(|| idx + nx),
        ];

        for n_idx in neighbors.into_iter().flatten() {
            if accepted[n_idx] || !inside[n_idx] {
                continue;
            }
            let nix = n_idx % nx;
            let niy = n_idx / nx;
            let d_x = min_accepted_inside(
                nix.checked_sub(1).map(|x| niy * nx + x),
                (nix + 1 < nx).then(|| niy * nx + nix + 1),
                &accepted,
                inside,
                &dist,
            );
            let d_y = min_accepted_inside(
                niy.checked_sub(1).map(|y| y * nx + nix),
                (niy + 1 < ny).then(|| (niy + 1) * nx + nix),
                &accepted,
                inside,
                &dist,
            );

            let f = cell * weight[n_idx];
            let u = d_x.min(d_y);
            let v = d_x.max(d_y);
            if !u.is_finite() {
                continue;
            }
            let cand = if v.is_finite() && (v - u) < f {
                0.5 * (u + v + (2.0 * f * f - (v - u) * (v - u)).max(0.0).sqrt())
            } else {
                u + f
            };
            if cand < dist[n_idx] - 1e-12 {
                dist[n_idx] = cand;
                heap.push(FmmNode {
                    dist: cand,
                    idx: n_idx,
                });
            }
        }
    }

    dist
}

#[inline]
fn min_accepted_inside(
    a: Option<usize>,
    b: Option<usize>,
    accepted: &[bool],
    inside: &[bool],
    dist: &[f64],
) -> f64 {
    let mut best = f64::INFINITY;
    if let Some(i) = a
        && accepted[i]
        && inside[i]
    {
        best = best.min(dist[i]);
    }
    if let Some(i) = b
        && accepted[i]
        && inside[i]
    {
        best = best.min(dist[i]);
    }
    best
}

#[allow(clippy::too_many_arguments)]
fn extract_and_project_iso_level(
    iso: f64,
    dist: &[f64],
    min_x: f64,
    min_y: f64,
    cell: f64,
    nx: usize,
    ny: usize,
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    containment: &RegionSet2D,
    rest_mask: Option<&RestMaskGrid2D>,
    z_bottom: f64,
    z_top: f64,
    climb_milling: bool,
    filter_slopes: bool,
    slope_min: f64,
    slope_max: f64,
    smooth_tol: f64,
) -> Vec<Vec<DVec3>> {
    let segs = marching_squares_segments(dist, iso, min_x, min_y, cell, nx, ny);
    if segs.is_empty() {
        return Vec::new();
    }
    let polylines_2d = chain_segments_2d(&segs);
    let mut scratch = Vec::with_capacity(64);
    let mut out = Vec::new();

    for mut poly in polylines_2d {
        if poly.len() < 2 {
            continue;
        }
        let is_closed = poly.len() >= 4 && poly[0].distance(poly[poly.len() - 1]) <= 1e-2;
        if is_closed {
            // Orient closed contours consistently (`CCW` for climb outside-in / `CW` for conventional)
            let area = signed_area_2d(&poly);
            let want_ccw = climb_milling;
            if (want_ccw && area < 0.0) || (!want_ccw && area > 0.0) {
                poly.reverse();
            }
        }

        // Resample long spans and project onto exact `f64` `drop_cutter_z`, splitting by rest/slope
        let mut current: Vec<DVec3> = Vec::new();
        let mut sub_chains: Vec<Vec<DVec3>> = Vec::new();

        for &pt in &poly {
            let in_reg =
                containment.contains_point(pt) || containment.min_distance_to_boundary(pt) <= 0.05;
            let in_rest = rest_mask.is_none_or(|rm| rm.contains_xy(pt));
            let valid_z = if in_reg && in_rest {
                drop_cutter_z_with_scratch(surface, cutter, pt.x, pt.y, z_bottom, &mut scratch)
                    .filter(|&z| z <= z_top + 1e-3)
                    .and_then(|z| {
                        if filter_slopes {
                            let s = sample_surface_slope_deg(
                                surface,
                                cutter,
                                pt,
                                z_bottom,
                                &mut scratch,
                            );
                            if s >= slope_min - 1.0 && s <= slope_max + 1.0 {
                                Some(z)
                            } else {
                                None
                            }
                        } else {
                            Some(z)
                        }
                    })
            } else {
                None
            };

            if let Some(z) = valid_z {
                let p3 = DVec3::new(pt.x, pt.y, z);
                if current
                    .last()
                    .is_none_or(|last| last.distance_squared(p3) > 1e-8)
                {
                    current.push(p3);
                }
            } else if current.len() >= 2 {
                sub_chains.push(std::mem::take(&mut current));
            } else {
                current.clear();
            }
        }
        if current.len() >= 2 {
            sub_chains.push(current);
        }

        // Re-join wrap-around segment if the original loop was closed and wasn't broken at seam
        if is_closed
            && sub_chains.len() >= 2
            && let (Some(first_p), Some(last_p)) = (
                sub_chains.first().and_then(|c| c.first()).copied(),
                sub_chains.last().and_then(|c| c.last()).copied(),
            )
            && first_p.distance(last_p) <= 0.1
            && let Some(mut tail) = sub_chains.pop()
        {
            tail.pop();
            tail.extend(sub_chains[0].iter().copied());
            sub_chains[0] = tail;
        }

        for raw in sub_chains {
            let refined =
                refine_convex_chords_3d(surface, cutter, z_bottom, z_top, &raw, smooth_tol);
            let simplified = simplify_3d_polyline_rdp(&refined, smooth_tol);
            if simplified.len() >= 2 {
                out.push(simplified);
            }
        }
    }

    out
}

/// Extracts marching-squares iso-contour line segments at `iso` across scalar grid `dist`.
pub(crate) fn marching_squares_segments(
    dist: &[f64],
    iso: f64,
    min_x: f64,
    min_y: f64,
    cell: f64,
    nx: usize,
    ny: usize,
) -> Vec<(DVec2, DVec2)> {
    let mut segs = Vec::new();
    for iy in 0..ny.saturating_sub(1) {
        let y0 = min_y + iy as f64 * cell;
        let y1 = y0 + cell;
        for ix in 0..nx.saturating_sub(1) {
            let x0 = min_x + ix as f64 * cell;
            let x1 = x0 + cell;

            let d0 = dist[iy * nx + ix]; // bottom-left
            let d1 = dist[iy * nx + ix + 1]; // bottom-right
            let d2 = dist[(iy + 1) * nx + ix + 1]; // top-right
            let d3 = dist[(iy + 1) * nx + ix]; // top-left
            if !d0.is_finite() || !d1.is_finite() || !d2.is_finite() || !d3.is_finite() {
                continue;
            }

            let mask = (u8::from(d0 >= iso))
                | (u8::from(d1 >= iso) << 1)
                | (u8::from(d2 >= iso) << 2)
                | (u8::from(d3 >= iso) << 3);
            if mask == 0 || mask == 15 {
                continue;
            }

            let interp = |p_a: DVec2, p_b: DVec2, v_a: f64, v_b: f64| -> DVec2 {
                let denom = v_b - v_a;
                let t = if denom.abs() > 1e-12 {
                    ((iso - v_a) / denom).clamp(0.0, 1.0)
                } else {
                    0.5
                };
                p_a.lerp(p_b, t)
            };

            let p0 = DVec2::new(x0, y0);
            let p1 = DVec2::new(x1, y0);
            let p2 = DVec2::new(x1, y1);
            let p3 = DVec2::new(x0, y1);

            let e0 = || interp(p0, p1, d0, d1); // bottom
            let e1 = || interp(p1, p2, d1, d2); // right
            let e2 = || interp(p2, p3, d2, d3); // top
            let e3 = || interp(p3, p0, d3, d0); // left

            // Oriented with `>= iso` on the LEFT of each directed segment
            match mask {
                1 => segs.push((e0(), e3())),
                2 => segs.push((e1(), e0())),
                3 => segs.push((e1(), e3())),
                4 => segs.push((e2(), e1())),
                5 => {
                    let mid = 0.25 * (d0 + d1 + d2 + d3);
                    if mid >= iso {
                        segs.push((e0(), e1()));
                        segs.push((e2(), e3()));
                    } else {
                        segs.push((e0(), e3()));
                        segs.push((e2(), e1()));
                    }
                }
                6 => segs.push((e2(), e0())),
                7 => segs.push((e2(), e3())),
                8 => segs.push((e3(), e2())),
                9 => segs.push((e0(), e2())),
                10 => {
                    let mid = 0.25 * (d0 + d1 + d2 + d3);
                    if mid >= iso {
                        segs.push((e1(), e2()));
                        segs.push((e3(), e0()));
                    } else {
                        segs.push((e1(), e0()));
                        segs.push((e3(), e2()));
                    }
                }
                11 => segs.push((e1(), e2())),
                12 => segs.push((e3(), e1())),
                13 => segs.push((e0(), e1())),
                14 => segs.push((e3(), e0())),
                _ => {}
            }
        }
    }
    segs
}

/// Chains directed 2D segments into continuous closed loops and open polylines deterministically.
pub(crate) fn chain_segments_2d(segs: &[(DVec2, DVec2)]) -> Vec<Vec<DVec2>> {
    if segs.is_empty() {
        return Vec::new();
    }
    let quant =
        |p: DVec2| -> (i64, i64) { ((p.x * 1e4).round() as i64, (p.y * 1e4).round() as i64) };

    let mut outgoing: HashMap<(i64, i64), Vec<usize>> = HashMap::with_capacity(segs.len());
    for (i, &(a, b)) in segs.iter().enumerate() {
        if a.distance_squared(b) <= 1e-12 {
            continue;
        }
        outgoing.entry(quant(a)).or_default().push(i);
    }

    let mut used = vec![false; segs.len()];
    let mut chains = Vec::new();

    for start_i in 0..segs.len() {
        if used[start_i] || segs[start_i].0.distance_squared(segs[start_i].1) <= 1e-12 {
            continue;
        }
        used[start_i] = true;
        let mut chain = vec![segs[start_i].0, segs[start_i].1];
        let mut cur_end = segs[start_i].1;

        loop {
            let key = quant(cur_end);
            let next_opt = outgoing
                .get(&key)
                .and_then(|list| list.iter().copied().find(|&idx| !used[idx]));
            let Some(next_idx) = next_opt else {
                break;
            };
            used[next_idx] = true;
            cur_end = segs[next_idx].1;
            chain.push(cur_end);
        }

        chains.push(chain);
    }

    // Merge open chains whose endpoints meet at the same quantized key
    let mut merged: Vec<Vec<DVec2>> = Vec::with_capacity(chains.len());
    'outer: for mut c in chains {
        if c.len() < 2 {
            continue;
        }
        let c_start = quant(c[0]);
        let c_end = quant(c[c.len() - 1]);
        if c_start != c_end {
            for m in &mut merged {
                let m_start = quant(m[0]);
                let m_end = quant(m[m.len() - 1]);
                if m_start == m_end {
                    continue;
                }
                if m_end == c_start {
                    m.extend_from_slice(&c[1..]);
                    continue 'outer;
                }
                if c_end == m_start {
                    c.extend_from_slice(&m[1..]);
                    *m = c;
                    continue 'outer;
                }
            }
        }
        merged.push(c);
    }

    merged
}

fn signed_area_2d(pts: &[DVec2]) -> f64 {
    if pts.len() < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for w in pts.windows(2) {
        sum += w[0].x * w[1].y - w[1].x * w[0].y;
    }
    0.5 * sum
}
