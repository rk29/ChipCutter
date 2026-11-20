//! **3D Pencil / Concave Fillet Cleanup** (`OperationKind::Pencil3D`).
//!
//! Detects concave bi-tangent valleys, corner creases, and unmachined fillets by evaluating the
//! exact `f64` Drop-Cutter residual height `delta_z(x, y) = z_drop(ref_ball) - z_drop(cur_cutter)`
//! across a parallel grid, extracting exact ridge centerlines via priority-ordered topology-
//! preserving thinning (peeling lowest `delta_z` flanks first so the skeleton converges onto the
//! exact bi-tangent ridge peak), with optional multi-pass lateral cleanup offsets.

use std::{cmp::Ordering, collections::BinaryHeap};

use chip_math::{DVec2, DVec3};

use crate::{
    cl_data::MotionRole,
    params::Pencil3DParams,
    strategies::StrategyExecContext,
    surface3d::{
        SurfaceCutterProfile, SurfaceLinkConfig, SurfaceMesh3D,
        compute_morphological_rest_residual, drop_cutter_points_parallel,
        drop_cutter_z_with_scratch, emit_final_surface_retract, emit_surface_safe_link,
        levels::resolve_3d_containment_region, refine_convex_chords_3d, region_aabb_2d,
        simplify_3d_polyline_rdp,
    },
    toolpath::ToolpathBuilder,
    wcs::CamError,
};

#[derive(Debug, Clone, Copy)]
struct ThinNode {
    delta_z: f64,
    idx: usize,
}

impl PartialEq for ThinNode {
    fn eq(&self, other: &Self) -> bool {
        self.idx == other.idx && self.delta_z.to_bits() == other.delta_z.to_bits()
    }
}

impl Eq for ThinNode {}

impl PartialOrd for ThinNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for ThinNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap by `delta_z` so lowest flanks are eroded first, preserving ridge peaks
        other
            .delta_z
            .total_cmp(&self.delta_z)
            .then_with(|| other.idx.cmp(&self.idx))
    }
}

/// Generates **3D Pencil / Concave Fillet Cleanup** toolpaths across `ctx.doc`.
pub fn generate_pencil3d(
    ctx: &StrategyExecContext<'_>,
    params: &Pencil3DParams,
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

    // Virtual larger reference ball detecting unmachined concave valleys & creases
    let ref_d = params
        .reference_tool_diameter
        .value_mm
        .max(tool_d * 1.25 + 0.5);
    let ref_ball_geom = chip_tools::CutterGeometry::BallEndmill {
        diameter_mm: ref_d,
        flute_length_mm: cutter.flute_length_mm.max(ref_d * 2.0),
        overall_length_mm: cutter.flute_length_mm.max(ref_d * 2.0) + 20.0,
        shank_diameter_mm: ref_d,
        flutes: 2,
    };
    let ref_cutter =
        SurfaceCutterProfile::from_tool_geometry(&ref_ball_geom, radial_stock, axial_stock);

    // 1. Evaluate exact morphological 3D residual stock left by `ref_ball` vs `cur_cutter`
    let w = (bounds.max.x - bounds.min.x).max(1.0);
    let h_span = (bounds.max.y - bounds.min.y).max(1.0);
    let base_cell = (0.12 * tool_d).clamp(0.25, 0.55);
    let cell = base_cell.max(((w * h_span) / 48_000.0).sqrt());
    let pad = 0.5 * (ref_d - tool_d).max(0.0) + cell;
    let min_x = bounds.min.x - pad;
    let min_y = bounds.min.y - pad;
    let nx = ((w + 2.0 * pad) / cell).ceil() as usize + 1;
    let ny = ((h_span + 2.0 * pad) / cell).ceil() as usize + 1;
    let total = nx * ny;

    let mut coords = Vec::with_capacity(total);
    let mut inside = vec![false; total];
    for iy in 0..ny {
        let y = min_y + iy as f64 * cell;
        for ix in 0..nx {
            let x = min_x + ix as f64 * cell;
            let pt = DVec2::new(x, y);
            coords.push(pt);
            inside[iy * nx + ix] = containment.contains_point(pt);
        }
    }

    ctx.check_cancel()?;
    let z_ref = drop_cutter_points_parallel(&surface, &ref_cutter, &coords, z_bottom, None);
    let z_cur =
        drop_cutter_points_parallel(&surface, &cutter, &coords, z_bottom, Some(&containment));
    ctx.check_cancel()?;

    let residual = compute_morphological_rest_residual(
        &z_ref,
        &z_cur,
        ref_cutter.max_radius_mm(),
        cutter.max_radius_mm(),
        cell,
        nx,
        ny,
    );

    let threshold = params.valley_threshold.value_mm.clamp(0.01, 5.0);
    let mut delta_z = vec![0.0_f64; total];
    let mut fg = vec![false; total];
    let mut active_count = 0usize;

    for idx in 0..total {
        if !inside[idx] {
            continue;
        }
        if let Some(zc) = z_cur[idx] {
            if zc > z_top + 1e-3 {
                continue;
            }
            let dz = residual[idx];
            delta_z[idx] = dz;
            if dz >= threshold {
                fg[idx] = true;
                active_count += 1;
            }
        }
    }

    if active_count < 2 {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    // 2. Priority-ordered ridge thinning converging onto exact `delta_z` crest centerlines
    priority_ridge_thin(&mut fg, &delta_z, nx, ny);

    // Prune tiny noisy 1–3 pixel spur stubs while preserving true valley chains
    let min_chain_len_mm = (0.75 * tool_d).clamp(1.2, 4.0);
    let raw_2d_chains = extract_skeleton_chains(&fg, min_x, min_y, cell, nx, ny);

    let mut base_chains_2d: Vec<(Vec<DVec2>, bool)> = Vec::new();
    for raw in raw_2d_chains {
        let is_closed = raw.len() >= 4 && raw[0].distance(raw[raw.len() - 1]) <= 1.6 * cell;
        let len_mm: f64 = raw.windows(2).map(|w| w[0].distance(w[1])).sum();
        if len_mm < min_chain_len_mm {
            continue;
        }
        let smoothed = smooth_polyline_2d(&raw, is_closed, 3);
        base_chains_2d.push((smoothed, is_closed));
    }

    if base_chains_2d.is_empty() {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    // 3. Build centerline + optional lateral multi-pass offset chains (`num_offset_passes`)
    let num_offsets = params.num_offset_passes.min(4);
    let off_step = params.offset_stepover.value_mm.clamp(0.05, 0.75 * tool_d);
    let mut all_2d_passes: Vec<Vec<DVec2>> = Vec::new();

    for (center, is_closed) in &base_chains_2d {
        // Emit outer offset passes first, finishing with the exact bi-tangent centerline pass!
        for k in (1..=num_offsets).rev() {
            let d = f64::from(k) * off_step;
            for sign in [-1.0_f64, 1.0_f64] {
                let shifted = offset_chain_2d(center, *is_closed, sign * d);
                if shifted.len() >= 2 {
                    all_2d_passes.push(shifted);
                }
            }
        }
        all_2d_passes.push(center.clone());
    }

    // 4. Project every 2D pass onto exact `f64` `drop_cutter_z`, refine convex chords, and simplify
    let smooth_tol = params.smoothing_tolerance.value_mm.clamp(0.001, 0.010);
    let mut bvh_scratch = Vec::with_capacity(64);
    let mut chains_3d: Vec<Vec<DVec3>> = Vec::new();

    for pass_2d in all_2d_passes {
        let mut current: Vec<DVec3> = Vec::new();
        for pt in pass_2d {
            let in_reg =
                containment.contains_point(pt) || containment.min_distance_to_boundary(pt) <= 0.05;
            if in_reg
                && let Some(z) = drop_cutter_z_with_scratch(
                    &surface,
                    &cutter,
                    pt.x,
                    pt.y,
                    z_bottom,
                    &mut bvh_scratch,
                )
                && z <= z_top + 1e-3
            {
                let p3 = DVec3::new(pt.x, pt.y, z);
                if current.last().is_none_or(|l| l.distance_squared(p3) > 1e-8) {
                    current.push(p3);
                }
            } else if current.len() >= 2 {
                let raw = std::mem::take(&mut current);
                let refined =
                    refine_convex_chords_3d(&surface, &cutter, z_bottom, z_top, &raw, smooth_tol);
                let simplified = simplify_3d_polyline_rdp(&refined, smooth_tol);
                if simplified.len() >= 2 {
                    chains_3d.push(simplified);
                }
            } else {
                current.clear();
            }
        }
        if current.len() >= 2 {
            let refined =
                refine_convex_chords_3d(&surface, &cutter, z_bottom, z_top, &current, smooth_tol);
            let simplified = simplify_3d_polyline_rdp(&refined, smooth_tol);
            if simplified.len() >= 2 {
                chains_3d.push(simplified);
            }
        }
    }

    if chains_3d.is_empty() {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    // 5. Emit passes in nearest-neighbor order with surface-safe collision-checked linking
    let retract_z = ctx.safe_retract_z();
    let stock_top_z = ctx.setup.stock.aabb_wcs().max.z;
    let cut_feed = ctx.cutting.cutting_feed_mm_min;
    let lead_feed = ctx.cutting.lead_feed_mm_min.max(0.4 * cut_feed);
    let plunge_feed = ctx.cutting.plunge_feed_mm_min.max(0.25 * cut_feed);
    let stay_down_limit = ctx
        .linking
        .safe_stay_down_distance
        .value_mm
        .clamp(tool_d, 3.0 * tool_d);

    let link_cfg = SurfaceLinkConfig {
        retract_z,
        stock_top_z,
        z_floor: z_bottom,
        stay_down_limit_mm: stay_down_limit,
        sample_step_mm: (0.4 * cell).clamp(0.15, 0.4),
        lead_feed,
        plunge_feed,
    };

    while !chains_3d.is_empty() {
        ctx.check_cancel()?;
        let (best_idx, flip) = if let Some(cur) = builder.current_pos() {
            let cur_xy = DVec2::new(cur.x, cur.y);
            let mut best = (0usize, false, f64::INFINITY);
            for (i, c) in chains_3d.iter().enumerate() {
                let d0 = DVec2::new(c[0].x, c[0].y).distance_squared(cur_xy);
                if d0 < best.2 {
                    best = (i, false, d0);
                }
                let d1 = DVec2::new(c[c.len() - 1].x, c[c.len() - 1].y).distance_squared(cur_xy);
                if d1 < best.2 {
                    best = (i, true, d1);
                }
            }
            (best.0, best.1)
        } else {
            (0, false)
        };

        let mut chain = chains_3d.swap_remove(best_idx);
        if flip {
            chain.reverse();
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
    }

    emit_final_surface_retract(retract_z, lead_feed, builder);
    Ok(())
}

/// Priority-ordered topology-preserving skeletonization: removes simple boundary pixels in
/// ascending order of `delta_z` so the surviving 1-pixel skeleton lies right on the ridge peak.
fn priority_ridge_thin(fg: &mut [bool], delta_z: &[f64], nx: usize, ny: usize) {
    let mut heap = BinaryHeap::new();
    for iy in 1..ny.saturating_sub(1) {
        for ix in 1..nx.saturating_sub(1) {
            let idx = iy * nx + ix;
            if fg[idx] && is_border_4(fg, ix, iy, nx) {
                heap.push(ThinNode {
                    delta_z: delta_z[idx],
                    idx,
                });
            }
        }
    }

    while let Some(ThinNode { idx, .. }) = heap.pop() {
        if !fg[idx] {
            continue;
        }
        let ix = idx % nx;
        let iy = idx / nx;
        if ix == 0 || iy == 0 || ix + 1 >= nx || iy + 1 >= ny {
            continue;
        }
        if !is_border_4(fg, ix, iy, nx) || !is_simple_non_endpoint(fg, ix, iy, nx) {
            continue;
        }
        fg[idx] = false;

        // Re-evaluate neighboring foreground pixels that may now be exposed border candidates
        for dy in -1_isize..=1 {
            for dx in -1_isize..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let nix = (ix as isize + dx) as usize;
                let niy = (iy as isize + dy) as usize;
                let nidx = niy * nx + nix;
                if fg[nidx]
                    && nix > 0
                    && niy > 0
                    && nix + 1 < nx
                    && niy + 1 < ny
                    && is_border_4(fg, nix, niy, nx)
                {
                    heap.push(ThinNode {
                        delta_z: delta_z[nidx],
                        idx: nidx,
                    });
                }
            }
        }
    }
}

#[inline]
fn is_border_4(fg: &[bool], ix: usize, iy: usize, nx: usize) -> bool {
    let idx = iy * nx + ix;
    !fg[idx - 1] || !fg[idx + 1] || !fg[idx - nx] || !fg[idx + nx]
}

/// Checks Zhang–Suen / Hilditch simple-point condition:
/// - `2 <= N(p) <= 6` foreground 8-neighbors (prevents shrinking endpoints or isolated pixels),
/// - `S(p) == 1` (`0 -> 1` transitions around the 8-ring, preserving 8-connectivity).
#[inline]
fn is_simple_non_endpoint(fg: &[bool], ix: usize, iy: usize, nx: usize) -> bool {
    let p2 = fg[(iy - 1) * nx + ix];
    let p3 = fg[(iy - 1) * nx + ix + 1];
    let p4 = fg[iy * nx + ix + 1];
    let p5 = fg[(iy + 1) * nx + ix + 1];
    let p6 = fg[(iy + 1) * nx + ix];
    let p7 = fg[(iy + 1) * nx + ix - 1];
    let p8 = fg[iy * nx + ix - 1];
    let p9 = fg[(iy - 1) * nx + ix - 1];

    let ring = [p2, p3, p4, p5, p6, p7, p8, p9];
    let n_fg: usize = ring.iter().filter(|&&b| b).count();
    if !(2..=6).contains(&n_fg) {
        return false;
    }

    let mut transitions = 0usize;
    for k in 0..8 {
        if !ring[k] && ring[(k + 1) % 8] {
            transitions += 1;
        }
    }
    transitions == 1
}

/// Extracts ordered 2D chains (open paths and closed loops) from an 8-connected 1-pixel skeleton.
fn extract_skeleton_chains(
    fg: &[bool],
    min_x: f64,
    min_y: f64,
    cell: f64,
    nx: usize,
    ny: usize,
) -> Vec<Vec<DVec2>> {
    let offsets: [(isize, isize); 8] = [
        (1, 0),
        (0, 1),
        (-1, 0),
        (0, -1),
        (1, 1),
        (-1, 1),
        (-1, -1),
        (1, -1),
    ];

    let neighbors_of = |idx: usize| -> Vec<usize> {
        let ix = (idx % nx) as isize;
        let iy = (idx / nx) as isize;
        let mut out = Vec::with_capacity(8);
        for (dx, dy) in offsets {
            let nix = ix + dx;
            let niy = iy + dy;
            if nix >= 0 && niy >= 0 && (nix as usize) < nx && (niy as usize) < ny {
                let nidx = niy as usize * nx + nix as usize;
                if fg[nidx] {
                    out.push(nidx);
                }
            }
        }
        out
    };

    let to_xy = |idx: usize| -> DVec2 {
        DVec2::new(
            min_x + (idx % nx) as f64 * cell,
            min_y + (idx / nx) as f64 * cell,
        )
    };

    let mut visited = vec![false; nx * ny];
    let mut chains = Vec::new();

    // Pass 1: start from endpoints (degree == 1) and junctions (degree >= 3) to trace open branches cleanly
    for idx in 0..(nx * ny) {
        if !fg[idx] || visited[idx] {
            continue;
        }
        let deg = neighbors_of(idx).len();
        if deg == 1 {
            let chain = trace_from(idx, &neighbors_of, &mut visited, &to_xy);
            if chain.len() >= 2 {
                chains.push(chain);
            }
        }
    }

    // Pass 2: trace remaining unvisited closed loops (every pixel has degree == 2)
    for idx in 0..(nx * ny) {
        if !fg[idx] || visited[idx] {
            continue;
        }
        let mut chain = trace_from(idx, &neighbors_of, &mut visited, &to_xy);
        if chain.len() >= 4 {
            let first = chain[0];
            let last = chain[chain.len() - 1];
            if first.distance(last) <= 1.5 * cell {
                chain.push(first);
            }
        }
        if chain.len() >= 2 {
            chains.push(chain);
        }
    }

    chains
}

fn trace_from<F, G>(start: usize, neighbors_of: &F, visited: &mut [bool], to_xy: &G) -> Vec<DVec2>
where
    F: Fn(usize) -> Vec<usize>,
    G: Fn(usize) -> DVec2,
{
    let mut chain = Vec::new();
    let mut cur = start;
    visited[cur] = true;
    chain.push(to_xy(cur));

    loop {
        let nbrs = neighbors_of(cur);
        // Prefer unvisited 4-connected neighbors before diagonal neighbors for clean grid walking
        let next_opt = nbrs.into_iter().find(|&n| !visited[n]);
        let Some(next) = next_opt else {
            break;
        };
        visited[next] = true;
        chain.push(to_xy(next));
        cur = next;
    }
    chain
}

/// Applies gentle Laplacian smoothing to eliminate 8-connected pixel stair-stepping while
/// preserving loop closure and open chain endpoints.
fn smooth_polyline_2d(pts: &[DVec2], is_closed: bool, iters: usize) -> Vec<DVec2> {
    if pts.len() < 4 {
        return pts.to_vec();
    }
    let mut cur = if is_closed && pts[0].distance(pts[pts.len() - 1]) <= 1e-4 {
        pts[..pts.len() - 1].to_vec()
    } else {
        pts.to_vec()
    };
    let n = cur.len();
    if n < 3 {
        return pts.to_vec();
    }

    for _ in 0..iters {
        let prev = cur.clone();
        if is_closed {
            for i in 0..n {
                let a = prev[(i + n - 1) % n];
                let b = prev[i];
                let c = prev[(i + 1) % n];
                cur[i] = b * 0.5 + (a + c) * 0.25;
            }
        } else {
            for i in 1..(n - 1) {
                cur[i] = prev[i] * 0.5 + (prev[i - 1] + prev[i + 1]) * 0.25;
            }
        }
    }

    if is_closed {
        cur.push(cur[0]);
    }
    cur
}

/// Computes a lateral 2D offset of `pts` by signed distance `offset_mm` using averaged vertex normals.
fn offset_chain_2d(pts: &[DVec2], is_closed: bool, offset_mm: f64) -> Vec<DVec2> {
    if pts.len() < 2 {
        return Vec::new();
    }
    let work = if is_closed && pts.len() >= 4 && pts[0].distance(pts[pts.len() - 1]) <= 1e-4 {
        &pts[..pts.len() - 1]
    } else {
        pts
    };
    let n = work.len();
    if n < 2 {
        return Vec::new();
    }

    let mut out = Vec::with_capacity(n + usize::from(is_closed));
    for i in 0..n {
        let t = if is_closed {
            let p_prev = work[(i + n - 1) % n];
            let p_next = work[(i + 1) % n];
            (p_next - p_prev).normalize_or_zero()
        } else if i == 0 {
            (work[1] - work[0]).normalize_or_zero()
        } else if i + 1 == n {
            (work[n - 1] - work[n - 2]).normalize_or_zero()
        } else {
            (work[i + 1] - work[i - 1]).normalize_or_zero()
        };
        let normal = DVec2::new(-t.y, t.x);
        out.push(work[i] + normal * offset_mm);
    }

    if is_closed && let Some(&first) = out.first() {
        out.push(first);
    }
    out
}
