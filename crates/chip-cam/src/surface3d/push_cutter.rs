use std::collections::HashMap;

use chip_geom2d::{contours_to_region_set, simplify_rdp_2d};
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3};
use rayon::prelude::*;

use crate::{
    rest2d::RegionSet2D,
    surface3d::{SurfaceMesh3D, cutter::SurfaceCutterProfile, drop_cutter_z_with_scratch},
};

/// Slices `surface` with the horizontal plane `z = z_wcs` and chains the resulting
/// plane-triangle intersection segments into welded 2D contours (`ArcPolyline2D`).
///
/// Closed outer boundaries are oriented CCW and interior holes CW whenever triangle normals
/// point outward; open meshes emit open polylines when boundary edges terminate.
pub fn slice_surface_contours_z(surface: &SurfaceMesh3D, z_wcs: f64) -> Vec<ArcPolyline2D> {
    if !z_wcs.is_finite() || surface.triangles.is_empty() {
        return Vec::new();
    }

    // Tiny deterministic offset if `z_wcs` lands directly on a mesh vertex so shared vertices
    // never produce degenerate zero-length segments or double-counted crossings.
    let mut z_eval = z_wcs;
    let slab = surface.bvh.query_z_slab(z_wcs - 1e-4, z_wcs + 1e-4);
    if slab.is_empty() {
        return Vec::new();
    }
    let hits_vertex = slab.iter().any(|&ti| {
        let [i0, i1, i2] = surface.triangles[ti].indices;
        (surface.vertices_wcs[i0 as usize].z - z_eval).abs() <= 1e-8
            || (surface.vertices_wcs[i1 as usize].z - z_eval).abs() <= 1e-8
            || (surface.vertices_wcs[i2 as usize].z - z_eval).abs() <= 1e-8
    });
    if hits_vertex {
        z_eval += 1e-6;
    }

    let mut segments: Vec<(DVec2, DVec2)> = Vec::new();
    for &ti in &slab {
        let tri = &surface.triangles[ti];
        if tri.aabb.max.z < z_eval || tri.aabb.min.z > z_eval {
            continue;
        }
        let [i0, i1, i2] = tri.indices;
        let v0 = surface.vertices_wcs[i0 as usize];
        let v1 = surface.vertices_wcs[i1 as usize];
        let v2 = surface.vertices_wcs[i2 as usize];

        if let Some((p_a, p_b)) = intersect_triangle_with_z_plane(v0, v1, v2, tri.normal, z_eval)
            && p_a.distance_squared(p_b) > 1e-12
        {
            segments.push((p_a, p_b));
        }
    }

    chain_slice_segments(&segments)
}

/// Precomputed exact vertical-cutter avoidance heightfield (`Z_drop(x, y)`) over a [`SurfaceMesh3D`]
/// for a specific [`SurfaceCutterProfile`].
///
/// For any vertical 3-axis milling cutter (Flat, Ball, Bull-Nose, Tapered/V-Bit) with cylindrical
/// flutes/shank extending upward above the tip profile, a cutter placed at `(x, y, z_wcs)` is
/// collision-free against `surface` if and only if `drop_cutter_z(surface, cutter, x, y) <= z_wcs`.
///
/// Precomputing this field once per operation makes Z-level avoidance queries (`avoidance_region_z`)
/// strictly monotonic across descending Z slices (`z1 > z2 => Avoid(z1) ⊆ Avoid(z2)`), automatically
/// preserving overhanging features such as helical lofts, horizontal flanges, and undercuts while
/// refining every contour crossing against exact analytical `drop_cutter_z`.
pub struct PushCutterField<'a> {
    surface: &'a SurfaceMesh3D,
    safe_cutter: SurfaceCutterProfile,
    min_x: f64,
    min_y: f64,
    cell_mm: f64,
    nx: usize,
    ny: usize,
    z_floor: f64,
    z_grid: Vec<f64>,
}

impl<'a> PushCutterField<'a> {
    /// Radial sagitta safety allowance (`mm`) folded analytically into the avoidance profile so
    /// simplified polygonal contours never undercut curved walls or bosses.
    const RADIAL_SAG_MARGIN_MM: f64 = 0.015;

    /// Builds a [`PushCutterField`] in parallel across the XY footprint of `surface` dilated by
    /// `cutter.max_radius_mm()`.
    pub fn build(surface: &'a SurfaceMesh3D, cutter: SurfaceCutterProfile) -> Self {
        let safe_cutter = cutter.with_extra_radial_margin(Self::RADIAL_SAG_MARGIN_MM);
        if surface.triangles.is_empty() || !surface.aabb_wcs.is_valid() {
            return Self {
                surface,
                safe_cutter,
                min_x: 0.0,
                min_y: 0.0,
                cell_mm: 1.0,
                nx: 0,
                ny: 0,
                z_floor: -1000.0,
                z_grid: Vec::new(),
            };
        }

        let r_max = safe_cutter.max_radius_mm().max(0.05);
        let aabb = surface.aabb_wcs;
        let span_x = (aabb.max.x - aabb.min.x).max(0.1) + 2.0 * r_max;
        let span_y = (aabb.max.y - aabb.min.y).max(0.1) + 2.0 * r_max;

        let base_cell = (0.18 * r_max).clamp(0.15, 0.30);
        const MAX_CELLS: f64 = 120_000.0;
        let cell_mm = base_cell.max(((span_x * span_y) / MAX_CELLS).sqrt());

        // Pad by `r_max + 2 * cell_mm` on every side so the entire outer grid border is strictly
        // outside the cutter's reach (`z = -inf`), guaranteeing every extracted contour is closed.
        let pad = r_max + 2.0 * cell_mm;
        let min_x = aabb.min.x - pad;
        let min_y = aabb.min.y - pad;
        let max_x = aabb.max.x + pad;
        let max_y = aabb.max.y + pad;

        let nx = (((max_x - min_x) / cell_mm).ceil() as usize).max(4) + 1;
        let ny = (((max_y - min_y) / cell_mm).ceil() as usize).max(4) + 1;
        let z_floor = aabb.min.z - 1000.0;

        let z_grid: Vec<f64> = (0..ny)
            .into_par_iter()
            .flat_map_iter(|iy| {
                let mut row = vec![f64::NEG_INFINITY; nx];
                let mut scratch = Vec::with_capacity(64);
                let y = min_y + iy as f64 * cell_mm;
                for (ix, cell_z) in row.iter_mut().enumerate() {
                    let x = min_x + ix as f64 * cell_mm;
                    if let Some(z_hit) = drop_cutter_z_with_scratch(
                        surface,
                        &safe_cutter,
                        x,
                        y,
                        z_floor,
                        &mut scratch,
                    ) {
                        *cell_z = z_hit;
                    }
                }
                row
            })
            .collect();

        Self {
            surface,
            safe_cutter,
            min_x,
            min_y,
            cell_mm,
            nx,
            ny,
            z_floor,
            z_grid,
        }
    }

    #[inline]
    fn grid_xy(&self, ix: usize, iy: usize) -> DVec2 {
        DVec2::new(
            self.min_x + ix as f64 * self.cell_mm,
            self.min_y + iy as f64 * self.cell_mm,
        )
    }

    #[inline]
    fn is_inside_exact(&self, pt: DVec2, z_thresh: f64, scratch: &mut Vec<usize>) -> bool {
        drop_cutter_z_with_scratch(
            self.surface,
            &self.safe_cutter,
            pt.x,
            pt.y,
            self.z_floor,
            scratch,
        )
        .is_some_and(|z| z > z_thresh)
    }

    /// Extracts the exact collision-free avoidance [`RegionSet2D`] at cutter tip height `z_wcs`
    /// (including full vertical flute/shank overhang shadow for all geometry above `z_wcs`).
    pub fn avoidance_region_z(&self, z_wcs: f64) -> RegionSet2D {
        if !z_wcs.is_finite() || self.nx < 3 || self.ny < 3 || self.z_grid.is_empty() {
            return RegionSet2D::empty();
        }
        let z_eff = z_wcs - self.safe_cutter.stock_axial_mm;
        if z_eff > self.surface.aabb_wcs.max.z + 1e-5 {
            return RegionSet2D::empty();
        }

        let z_thresh = z_wcs + 1e-4;
        let nx = self.nx;
        let ny = self.ny;
        let inside: Vec<bool> = self.z_grid.iter().map(|&z| z > z_thresh).collect();

        let h_cols = nx - 1;
        let h_count = h_cols * ny;
        let v_count = nx * (ny - 1);
        let total_edges = h_count + v_count;

        let h_edge = |ix: usize, iy: usize| -> usize { iy * h_cols + ix };
        let v_edge = |ix: usize, iy: usize| -> usize { h_count + iy * nx + ix };

        // 1. Collect all grid edges crossing `z_thresh` (inside -> outside)
        let mut crossing_specs: Vec<(usize, DVec2, DVec2)> = Vec::new();
        for iy in 0..ny {
            let row = iy * nx;
            for ix in 0..h_cols {
                let a_in = inside[row + ix];
                let b_in = inside[row + ix + 1];
                if a_in != b_in {
                    let pa = self.grid_xy(ix, iy);
                    let pb = self.grid_xy(ix + 1, iy);
                    let (p_in, p_out) = if a_in { (pa, pb) } else { (pb, pa) };
                    crossing_specs.push((h_edge(ix, iy), p_in, p_out));
                }
            }
        }
        for iy in 0..(ny - 1) {
            let row0 = iy * nx;
            let row1 = (iy + 1) * nx;
            for ix in 0..nx {
                let a_in = inside[row0 + ix];
                let b_in = inside[row1 + ix];
                if a_in != b_in {
                    let pa = self.grid_xy(ix, iy);
                    let pb = self.grid_xy(ix, iy + 1);
                    let (p_in, p_out) = if a_in { (pa, pb) } else { (pb, pa) };
                    crossing_specs.push((v_edge(ix, iy), p_in, p_out));
                }
            }
        }
        if crossing_specs.is_empty() {
            return RegionSet2D::empty();
        }

        // 2. Refine every crossing point against exact analytical `drop_cutter_z` in parallel
        //    (7 bisection steps -> `< cell_mm / 128`, returning conservative outside bracket `t_hi`).
        let refined_points: Vec<(usize, DVec2)> = crossing_specs
            .par_iter()
            .map_init(
                || Vec::with_capacity(64),
                |scratch, &(eid, p_in, p_out)| {
                    let mut t_lo = 0.0_f64;
                    let mut t_hi = 1.0_f64;
                    for _ in 0..7 {
                        let t_mid = 0.5 * (t_lo + t_hi);
                        let p_mid = p_in.lerp(p_out, t_mid);
                        if self.is_inside_exact(p_mid, z_thresh, scratch) {
                            t_lo = t_mid;
                        } else {
                            t_hi = t_mid;
                        }
                    }
                    let t_safe = t_hi.clamp(1e-3, 1.0 - 1e-3);
                    (eid, p_in.lerp(p_out, t_safe))
                },
            )
            .collect();

        let mut edge_pos: Vec<DVec2> = vec![DVec2::ZERO; total_edges];
        for (eid, pt) in refined_points {
            edge_pos[eid] = pt;
        }

        // 3. Emit Marching Squares directed transitions (`inside` always to the LEFT of travel:
        //    outer obstacle loops CCW, interior cavity holes CW) with exact saddle disambiguation.
        let mut next_edge: Vec<u32> = vec![u32::MAX; total_edges];
        let mut active_edges: Vec<usize> = Vec::with_capacity(crossing_specs.len());
        let mut saddle_scratch = Vec::with_capacity(64);

        let mut link = |from: usize, to: usize| {
            next_edge[from] = to as u32;
            active_edges.push(from);
        };

        for iy in 0..(ny - 1) {
            let row0 = iy * nx;
            let row1 = (iy + 1) * nx;
            for ix in 0..h_cols {
                let mask = u8::from(inside[row0 + ix])
                    | (u8::from(inside[row0 + ix + 1]) << 1)
                    | (u8::from(inside[row1 + ix + 1]) << 2)
                    | (u8::from(inside[row1 + ix]) << 3);
                if mask == 0 || mask == 15 {
                    continue;
                }
                let eb = h_edge(ix, iy);
                let er = v_edge(ix + 1, iy);
                let et = h_edge(ix, iy + 1);
                let el = v_edge(ix, iy);

                match mask {
                    1 => link(eb, el),
                    2 => link(er, eb),
                    3 => link(er, el),
                    4 => link(et, er),
                    5 => {
                        let center = DVec2::new(
                            self.min_x + (ix as f64 + 0.5) * self.cell_mm,
                            self.min_y + (iy as f64 + 0.5) * self.cell_mm,
                        );
                        if self.is_inside_exact(center, z_thresh, &mut saddle_scratch) {
                            link(eb, er);
                            link(et, el);
                        } else {
                            link(eb, el);
                            link(et, er);
                        }
                    }
                    6 => link(et, eb),
                    7 => link(et, el),
                    8 => link(el, et),
                    9 => link(eb, et),
                    10 => {
                        let center = DVec2::new(
                            self.min_x + (ix as f64 + 0.5) * self.cell_mm,
                            self.min_y + (iy as f64 + 0.5) * self.cell_mm,
                        );
                        if self.is_inside_exact(center, z_thresh, &mut saddle_scratch) {
                            link(el, eb);
                            link(er, et);
                        } else {
                            link(er, eb);
                            link(el, et);
                        }
                    }
                    11 => link(er, et),
                    12 => link(el, er),
                    13 => link(eb, er),
                    14 => link(el, eb),
                    _ => {}
                }
            }
        }

        // 4. Trace guaranteed-closed loops, refine any convex chords against `drop_cutter_z`,
        //    and simplify cleanly for downstream `cavalier_contours` boolean ops.
        let mut visited = vec![false; total_edges];
        let mut polylines: Vec<ArcPolyline2D> = Vec::new();
        let mut refine_scratch = Vec::with_capacity(64);

        for start_e in active_edges {
            if visited[start_e] || next_edge[start_e] == u32::MAX {
                continue;
            }
            let mut loop_pts: Vec<DVec2> = Vec::new();
            let mut cur = start_e;
            let mut closed = false;
            while cur < total_edges && !visited[cur] {
                visited[cur] = true;
                let p = edge_pos[cur];
                if loop_pts
                    .last()
                    .is_none_or(|prev| prev.distance_squared(p) > 1e-6)
                {
                    loop_pts.push(p);
                }
                let nxt = next_edge[cur];
                if nxt == u32::MAX {
                    break;
                }
                cur = nxt as usize;
                if cur == start_e {
                    closed = true;
                    break;
                }
            }
            if !closed || loop_pts.len() < 3 {
                continue;
            }
            if loop_pts
                .first()
                .zip(loop_pts.last())
                .is_some_and(|(a, b)| a.distance_squared(*b) <= 1e-6)
            {
                loop_pts.pop();
            }
            if loop_pts.len() < 3 {
                continue;
            }

            // Refine any chord whose midpoint dips inside `Avoid` (keeps convex corners exact even on large parts)
            let mut refined: Vec<DVec2> = Vec::with_capacity(loop_pts.len());
            let n_pts = loop_pts.len();
            for i in 0..n_pts {
                let a = loop_pts[i];
                let b = loop_pts[(i + 1) % n_pts];
                refined.push(a);
                self.refine_avoid_chord(a, b, z_thresh, 0, &mut refine_scratch, &mut refined);
            }

            if let Some((min_idx, _)) = refined
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.x.total_cmp(&b.x).then_with(|| a.y.total_cmp(&b.y)))
            {
                refined.rotate_left(min_idx);
            }
            if refined.len() >= 4 {
                refined = simplify_rdp_2d(&refined, true, 0.005);
            }
            if refined.len() < 3 {
                continue;
            }

            let mut segs = Vec::with_capacity(refined.len());
            for w in refined.windows(2) {
                if w[0].distance_squared(w[1]) > 1e-6 {
                    segs.push(ArcSegment2D::Line {
                        start: w[0],
                        end: w[1],
                    });
                }
            }
            if let (Some(&first), Some(&last)) = (refined.first(), refined.last())
                && first.distance_squared(last) > 1e-6
            {
                segs.push(ArcSegment2D::Line {
                    start: last,
                    end: first,
                });
            }
            if segs.len() >= 3 {
                polylines.push(ArcPolyline2D {
                    segments: segs,
                    closed: true,
                });
            }
        }

        contours_to_region_set(&polylines)
    }

    fn refine_avoid_chord(
        &self,
        a: DVec2,
        b: DVec2,
        z_thresh: f64,
        depth: u32,
        scratch: &mut Vec<usize>,
        out: &mut Vec<DVec2>,
    ) {
        if depth >= 3 {
            return;
        }
        let seg = b - a;
        let len = seg.length();
        if len <= 0.30 {
            return;
        }
        let mid = 0.5 * (a + b);
        if !self.is_inside_exact(mid, z_thresh, scratch) {
            return;
        }
        // `Avoid` lies to the LEFT of `a -> b`, so the outward clear-air direction is the RIGHT normal.
        let out_n = DVec2::new(seg.y / len, -seg.x / len);
        let mut step_hi = 0.05_f64;
        let max_step = self.cell_mm.max(0.5);
        while step_hi < max_step && self.is_inside_exact(mid + out_n * step_hi, z_thresh, scratch) {
            step_hi *= 2.0;
        }
        if self.is_inside_exact(mid + out_n * step_hi, z_thresh, scratch) {
            return;
        }
        let mut lo = 0.0_f64;
        let mut hi = step_hi;
        for _ in 0..6 {
            let m = 0.5 * (lo + hi);
            if self.is_inside_exact(mid + out_n * m, z_thresh, scratch) {
                lo = m;
            } else {
                hi = m;
            }
        }
        let p_safe = mid + out_n * hi;
        if p_safe.distance_squared(a) <= 1e-6 || p_safe.distance_squared(b) <= 1e-6 {
            return;
        }
        self.refine_avoid_chord(a, p_safe, z_thresh, depth + 1, scratch, out);
        out.push(p_safe);
        self.refine_avoid_chord(p_safe, b, z_thresh, depth + 1, scratch, out);
    }
}

/// Evaluates the exact horizontal cutter-avoidance [`RegionSet2D`] at height `z_wcs` (`z_tip` in WCS)
/// for `cutter` (accounting for Flat, Ball, Bull-Nose, and Tapered/V-Bit profiles, radial & axial
/// stock-to-leave allowances, and the full vertical cutter shadow beneath overhanging geometry).
///
/// Any cutter center `(x, y)` placed **outside** the returned [`RegionSet2D`] is collision-free
/// against `surface` at height `z_wcs`, making its boundary directly usable for `Waterline3D` and
/// `Adaptive3D` toolpaths. When evaluating multiple Z levels on the same `(surface, cutter)`, prefer
/// building [`PushCutterField`] once and calling [`PushCutterField::avoidance_region_z`].
pub fn push_cutter_region_z(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    z_wcs: f64,
) -> RegionSet2D {
    if !z_wcs.is_finite() || surface.triangles.is_empty() || !surface.aabb_wcs.is_valid() {
        return RegionSet2D::empty();
    }
    let z_eff = z_wcs - cutter.stock_axial_mm;
    if z_eff > surface.aabb_wcs.max.z + 1e-6 {
        return RegionSet2D::empty();
    }
    PushCutterField::build(surface, *cutter).avoidance_region_z(z_wcs)
}

fn intersect_triangle_with_z_plane(
    v0: DVec3,
    v1: DVec3,
    v2: DVec3,
    normal: DVec3,
    z_plane: f64,
) -> Option<(DVec2, DVec2)> {
    let d0 = v0.z - z_plane;
    let d1 = v1.z - z_plane;
    let d2 = v2.z - z_plane;

    let a0 = d0 > 0.0;
    let a1 = d1 > 0.0;
    let a2 = d2 > 0.0;
    if (a0 && a1 && a2) || (!a0 && !a1 && !a2) {
        return None;
    }

    let lerp_xy = |p: DVec3, q: DVec3, dp: f64, dq: f64| -> DVec2 {
        let t = dp / (dp - dq);
        DVec2::new(p.x + t * (q.x - p.x), p.y + t * (q.y - p.y))
    };

    let mut pts = [DVec2::ZERO; 2];
    let mut n_pts = 0usize;
    for (p, q, dp, dq, ap, aq) in [
        (v0, v1, d0, d1, a0, a1),
        (v1, v2, d1, d2, a1, a2),
        (v2, v0, d2, d0, a2, a0),
    ] {
        if ap != aq && n_pts < 2 {
            pts[n_pts] = lerp_xy(p, q, dp, dq);
            n_pts += 1;
        }
    }
    if n_pts != 2 {
        return None;
    }

    let mut p_a = pts[0];
    let mut p_b = pts[1];
    // Orient segment so the triangle's outward horizontal normal points to the RIGHT of `p_a -> p_b`
    // (standard CCW solid boundary convention: interior is to the LEFT!).
    let seg = p_b - p_a;
    let right_dir = DVec2::new(seg.y, -seg.x);
    let n_xy = DVec2::new(normal.x, normal.y);
    if right_dir.dot(n_xy) < 0.0 {
        std::mem::swap(&mut p_a, &mut p_b);
    }
    Some((p_a, p_b))
}

/// Welds coincident 2D segment endpoints (`tol = 1e-4 mm`) and chains directed segments into polylines.
fn chain_slice_segments(segments: &[(DVec2, DVec2)]) -> Vec<ArcPolyline2D> {
    if segments.is_empty() {
        return Vec::new();
    }

    const WELD_TOL: f64 = 1e-4;
    let inv_tol = 1.0 / WELD_TOL;
    let weld_sq = WELD_TOL * WELD_TOL;

    let mut verts: Vec<DVec2> = Vec::with_capacity(segments.len());
    let mut grid: HashMap<(i64, i64), Vec<usize>> = HashMap::with_capacity(segments.len());

    let mut intern = |p: DVec2| -> usize {
        let gx = (p.x * inv_tol).floor() as i64;
        let gy = (p.y * inv_tol).floor() as i64;
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(bucket) = grid.get(&(gx + dx, gy + dy)) {
                    for &idx in bucket {
                        if verts[idx].distance_squared(p) <= weld_sq {
                            return idx;
                        }
                    }
                }
            }
        }
        let idx = verts.len();
        verts.push(p);
        grid.entry((gx, gy)).or_default().push(idx);
        idx
    };

    let mut edges: Vec<(usize, usize)> = Vec::with_capacity(segments.len());
    let mut outgoing: HashMap<usize, Vec<usize>> = HashMap::with_capacity(segments.len());
    for &(a, b) in segments {
        let ia = intern(a);
        let ib = intern(b);
        if ia == ib {
            continue;
        }
        let eid = edges.len();
        edges.push((ia, ib));
        outgoing.entry(ia).or_default().push(eid);
    }

    let mut used = vec![false; edges.len()];
    let mut polylines = Vec::new();

    for start_eid in 0..edges.len() {
        if used[start_eid] {
            continue;
        }
        used[start_eid] = true;
        let (start_v, mut cur_v) = edges[start_eid];
        let mut chain_indices = vec![start_v, cur_v];

        while cur_v != start_v {
            let next_edge = outgoing
                .get(&cur_v)
                .and_then(|list| list.iter().copied().find(|&eid| !used[eid]));
            let Some(eid) = next_edge else {
                break;
            };
            used[eid] = true;
            cur_v = edges[eid].1;
            chain_indices.push(cur_v);
        }

        let closed = cur_v == start_v && chain_indices.len() >= 4;
        // Filter micro-segments and simplify collinear/near-collinear zig-zag vertices (`tol = 5 µm`)
        // so downstream `cavalier_contours` offsets and booleans stay fast and numerically rock-solid.
        let mut pts: Vec<DVec2> = Vec::with_capacity(chain_indices.len());
        for &vi in &chain_indices {
            let p = verts[vi];
            if pts
                .last()
                .is_none_or(|prev| prev.distance_squared(p) > 1e-4)
            {
                pts.push(p);
            }
        }
        if closed
            && pts.len() >= 2
            && pts
                .first()
                .zip(pts.last())
                .is_some_and(|(a, b)| a.distance_squared(*b) <= 1e-4)
        {
            pts.pop();
        }
        if closed
            && let Some((min_idx, _)) = pts
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.x.total_cmp(&b.x).then_with(|| a.y.total_cmp(&b.y)))
        {
            pts.rotate_left(min_idx);
        }
        if pts.len() >= 4 {
            pts = simplify_rdp_2d(&pts, closed, 0.005);
        }
        let mut poly_segs = Vec::with_capacity(pts.len());
        for w in pts.windows(2) {
            if w[0].distance_squared(w[1]) > 1e-6 {
                poly_segs.push(ArcSegment2D::Line {
                    start: w[0],
                    end: w[1],
                });
            }
        }
        if closed
            && let (Some(&first), Some(&last)) = (pts.first(), pts.last())
            && first.distance_squared(last) > 1e-6
        {
            poly_segs.push(ArcSegment2D::Line {
                start: last,
                end: first,
            });
        }
        if !poly_segs.is_empty() {
            polylines.push(ArcPolyline2D {
                segments: poly_segs,
                closed,
            });
        }
    }

    polylines
}
