use chip_bvh::FlatBvh;
use chip_cad::CadDocument;
use chip_cam::{
    CamError, ResolvedSetup, Setup, SurfaceMesh3D, rect_region_2d, slice_surface_contours_z,
    stock_xy_region,
};
use chip_geom2d::{PocketRegion2D, RegionSet2D, contours_to_region_set, simplify_rdp_2d};
use chip_math::{Aabb2, Aabb3, ArcPolyline2D, ArcSegment2D, DVec2, DVec3};
use rayon::prelude::*;

use super::region_set_aabb;

/// Projects the target part's exact `f64` outer 2D silhouette [`RegionSet2D`] in Setup WCS,
/// optionally restricted to a vertical clamping Z band `[z_lo, z_hi]`.
pub fn extract_part_silhouette_2d(
    doc: &CadDocument,
    setup: &Setup,
    resolved: &ResolvedSetup,
    z_band: Option<(f64, f64)>,
) -> Result<RegionSet2D, CamError> {
    if !doc.has_3d_geometry() {
        let b = resolved.model_aabb_wcs;
        if b.is_valid() {
            return Ok(rect_region_2d(b.min.x, b.min.y, b.max.x, b.max.y));
        }
        return Ok(stock_xy_region(resolved));
    }

    let surface = SurfaceMesh3D::from_cad_document(doc, &setup.target_bodies, &resolved.frame);
    if surface.triangles.is_empty() || !surface.bounds.is_valid() {
        let b = resolved.model_aabb_wcs;
        if b.is_valid() {
            return Ok(rect_region_2d(b.min.x, b.min.y, b.max.x, b.max.y));
        }
        return Err(CamError::EmptyCadDocument);
    }

    let z_min_part = surface.bounds.min.z;
    let z_max_part = surface.bounds.max.z;
    let (z_lo, z_hi) = match z_band {
        Some((lo, hi)) => (
            lo.clamp(z_min_part, z_max_part),
            hi.clamp(z_min_part, z_max_part),
        ),
        None => (z_min_part, z_max_part),
    };
    let (z_lo, z_hi) = if z_hi > z_lo + 1e-4 {
        (z_lo, z_hi)
    } else {
        (z_min_part, z_max_part)
    };

    let swept = extract_swept_clamping_band_silhouette(&surface, z_lo, z_hi);
    if !swept.is_empty() {
        Ok(swept)
    } else {
        Ok(rect_region_2d(
            surface.bounds.min.x,
            surface.bounds.min.y,
            surface.bounds.max.x,
            surface.bounds.max.y,
        ))
    }
}

/// Exact continuous swept XY silhouette of `surface` across the vertical clamping Z band `[z_lo, z_hi]`.
///
/// Captures the full continuous horizontal projection of every triangle clipped to `[z_lo, z_hi]`
/// (eliminating discrete Z-slice scalloping on continuously twisting helical blades) combined with
/// exact mid-slab cross-section occupancy for purely vertical prism interiors, extracted via a fine
/// parallel occupancy grid + Marching Squares with analytical boundary bisection, convex-chord
/// refinement, and sharp-corner restoration.
pub(super) fn extract_swept_clamping_band_silhouette(
    surface: &SurfaceMesh3D,
    z_lo: f64,
    z_hi: f64,
) -> RegionSet2D {
    if surface.triangles.is_empty() || !surface.bounds.is_valid() {
        return RegionSet2D::empty();
    }

    let z_min_part = surface.bounds.min.z;
    let z_max_part = surface.bounds.max.z;
    // Include coplanar part bottom/top caps when the band reaches the part's extreme Z, while
    // excluding a wider lower ledge ending strictly at `z_lo` when gripping an elevated band.
    let z_lo_clip = if z_lo <= z_min_part + 1e-5 {
        z_lo - 1e-6
    } else {
        z_lo + 1e-6
    };
    let z_hi_clip = if z_hi >= z_max_part - 1e-5 {
        z_hi + 1e-6
    } else {
        z_hi - 1e-6
    };
    let (z_lo_clip, z_hi_clip) = if z_hi_clip > z_lo_clip {
        (z_lo_clip, z_hi_clip)
    } else {
        (z_lo - 1e-6, z_hi + 1e-6)
    };

    // 1. Mid-slab horizontal cross-section covers interior XY columns of purely vertical prisms
    //    whose vertical side walls have zero projected XY area inside `[z_lo, z_hi]`.
    let z_mid = 0.5 * (z_lo + z_hi);
    let mid_contours = slice_surface_contours_z(surface, z_mid);
    let mid_slice_region = contours_to_region_set(&mid_contours);

    // 2. Clip every triangle intersecting `[z_lo_clip, z_hi_clip]` to the Z slab and project the
    //    resulting convex polygon (3..5 vertices) into 2D XY triangles accelerated by a `FlatBvh`.
    let slab_tris = surface.bvh.query_z_slab(z_lo_clip, z_hi_clip);
    let mut proj_tris: Vec<[DVec2; 3]> = Vec::with_capacity(slab_tris.len() * 2);
    let mut proj_aabbs: Vec<Aabb3> = Vec::with_capacity(slab_tris.len() * 2);
    let mut swept_aabb = region_set_aabb(&mid_slice_region).unwrap_or(Aabb2::EMPTY);

    for &ti in &slab_tris {
        let tri = &surface.triangles[ti];
        if tri.aabb.max.z < z_lo_clip || tri.aabb.min.z > z_hi_clip {
            continue;
        }
        let [i0, i1, i2] = tri.indices;
        let v0 = surface.vertices[i0 as usize];
        let v1 = surface.vertices[i1 as usize];
        let v2 = surface.vertices[i2 as usize];

        let mut clipped_buf = [DVec3::ZERO; 7];
        let clipped_len =
            chip_bvh::clip_triangle_to_z_slab(v0, v1, v2, z_lo_clip, z_hi_clip, &mut clipped_buf);
        if clipped_len < 3 {
            continue;
        }
        let clipped = &clipped_buf[..clipped_len];
        let p0 = DVec2::new(clipped[0].x, clipped[0].y);
        for k in 1..(clipped.len() - 1) {
            let p1 = DVec2::new(clipped[k].x, clipped[k].y);
            let p2 = DVec2::new(clipped[k + 1].x, clipped[k + 1].y);
            let area2 = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
            if area2.abs() <= 1e-10 {
                continue;
            }
            let min_x = p0.x.min(p1.x).min(p2.x);
            let max_x = p0.x.max(p1.x).max(p2.x);
            let min_y = p0.y.min(p1.y).min(p2.y);
            let max_y = p0.y.max(p1.y).max(p2.y);
            swept_aabb.extend_point(DVec2::new(min_x, min_y));
            swept_aabb.extend_point(DVec2::new(max_x, max_y));
            proj_tris.push([p0, p1, p2]);
            proj_aabbs.push(Aabb3::new(
                DVec3::new(min_x, min_y, 0.0),
                DVec3::new(max_x, max_y, 0.0),
            ));
        }
    }

    if !swept_aabb.is_valid() {
        return RegionSet2D::empty();
    }

    // Fast path: if all geometry inside the slab is purely vertical (no non-vertical triangles
    // projected beyond `mid_slice_region`), return the exact filled outer hull of `mid_slice_region`.
    if proj_tris.is_empty() {
        let filled_hulls: Vec<PocketRegion2D> = mid_slice_region
            .regions
            .into_iter()
            .map(|r| PocketRegion2D::new(r.outer_ccw, Vec::new()))
            .collect();
        return RegionSet2D::from_regions(filled_hulls);
    }

    let bvh_2d = FlatBvh::build(&proj_aabbs);
    // Tiny 5 µm conservative radial margin guarantees simplified curved chords never undercut
    // continuously swept helical blade crests while keeping prismatic dimensions exact to < 0.01 mm.
    const CONSERVATIVE_MARGIN_MM: f64 = 0.005;
    const MARGIN_SQ: f64 = CONSERVATIVE_MARGIN_MM * CONSERVATIVE_MARGIN_MM;

    let is_occupied = |pt: DVec2, scratch: &mut Vec<usize>| -> bool {
        if mid_slice_region.contains_point(pt) {
            return true;
        }
        bvh_2d.query_xy_circle_into(pt.x, pt.y, CONSERVATIVE_MARGIN_MM + 1e-6, scratch);
        for &idx in scratch.iter() {
            let [a, b, c] = proj_tris[idx];
            if point_in_or_near_triangle_2d(pt, a, b, c, MARGIN_SQ) {
                return true;
            }
        }
        false
    };

    let span_x = (swept_aabb.max.x - swept_aabb.min.x).max(0.1);
    let span_y = (swept_aabb.max.y - swept_aabb.min.y).max(0.1);
    let base_cell = (0.005 * span_x.max(span_y)).clamp(0.12, 0.35);
    const MAX_CELLS: f64 = 120_000.0;
    let cell_mm = base_cell.max(((span_x * span_y) / MAX_CELLS).sqrt());

    // Pad by `2 * cell_mm` so every outer border node is guaranteed outside, forcing closed loops.
    let pad = 2.0 * cell_mm;
    let min_x = swept_aabb.min.x - pad;
    let min_y = swept_aabb.min.y - pad;
    let max_x = swept_aabb.max.x + pad;
    let max_y = swept_aabb.max.y + pad;

    let nx = (((max_x - min_x) / cell_mm).ceil() as usize).max(4) + 1;
    let ny = (((max_y - min_y) / cell_mm).ceil() as usize).max(4) + 1;
    let grid_xy = |ix: usize, iy: usize| -> DVec2 {
        DVec2::new(min_x + ix as f64 * cell_mm, min_y + iy as f64 * cell_mm)
    };

    // 3. Evaluate exact swept 2D occupancy across the grid in parallel
    let inside: Vec<bool> = (0..ny)
        .into_par_iter()
        .flat_map_iter(|iy| {
            let mut row = vec![false; nx];
            let mut scratch = Vec::with_capacity(64);
            let y = min_y + iy as f64 * cell_mm;
            for (ix, flag) in row.iter_mut().enumerate() {
                let x = min_x + ix as f64 * cell_mm;
                *flag = is_occupied(DVec2::new(x, y), &mut scratch);
            }
            row
        })
        .collect();

    let h_cols = nx - 1;
    let h_count = h_cols * ny;
    let v_count = nx * (ny - 1);
    let total_edges = h_count + v_count;
    let h_edge = |ix: usize, iy: usize| -> usize { iy * h_cols + ix };
    let v_edge = |ix: usize, iy: usize| -> usize { h_count + iy * nx + ix };

    let mut crossing_specs: Vec<(usize, DVec2, DVec2)> = Vec::new();
    for iy in 0..ny {
        let row = iy * nx;
        for ix in 0..h_cols {
            let a_in = inside[row + ix];
            let b_in = inside[row + ix + 1];
            if a_in != b_in {
                let pa = grid_xy(ix, iy);
                let pb = grid_xy(ix + 1, iy);
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
                let pa = grid_xy(ix, iy);
                let pb = grid_xy(ix, iy + 1);
                let (p_in, p_out) = if a_in { (pa, pb) } else { (pb, pa) };
                crossing_specs.push((v_edge(ix, iy), p_in, p_out));
            }
        }
    }
    if crossing_specs.is_empty() {
        return RegionSet2D::empty();
    }

    // 4. Analytical boundary bisection in parallel (8 steps -> `< cell_mm / 256`, conservative `t_hi`)
    let refined_points: Vec<(usize, DVec2)> = crossing_specs
        .par_iter()
        .map_init(
            || Vec::with_capacity(64),
            |scratch, &(eid, p_in, p_out)| {
                let mut t_lo = 0.0_f64;
                let mut t_hi = 1.0_f64;
                for _ in 0..8 {
                    let t_mid = 0.5 * (t_lo + t_hi);
                    let p_mid = p_in.lerp(p_out, t_mid);
                    if is_occupied(p_mid, scratch) {
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

    let mut edge_pos = vec![DVec2::ZERO; total_edges];
    for (eid, pt) in refined_points {
        edge_pos[eid] = pt;
    }

    // 5. Marching Squares directed edge transitions (`inside` on LEFT -> outer CCW, holes CW)
    let mut next_edge = vec![u32::MAX; total_edges];
    let mut active_edges = Vec::with_capacity(crossing_specs.len());
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
                        min_x + (ix as f64 + 0.5) * cell_mm,
                        min_y + (iy as f64 + 0.5) * cell_mm,
                    );
                    if is_occupied(center, &mut saddle_scratch) {
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
                        min_x + (ix as f64 + 0.5) * cell_mm,
                        min_y + (iy as f64 + 0.5) * cell_mm,
                    );
                    if is_occupied(center, &mut saddle_scratch) {
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

    // 6. Trace closed loops, refine convex chords, simplify, restore sharp corners, and keep
    //    filled outer CCW hulls so interior mounting holes never become floating soft-jaw pins.
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

        // Refine any chord whose midpoint dips inside the swept silhouette (eliminates sagitta undercut)
        let mut refined: Vec<DVec2> = Vec::with_capacity(loop_pts.len());
        let n_pts = loop_pts.len();
        for i in 0..n_pts {
            let a = loop_pts[i];
            let b = loop_pts[(i + 1) % n_pts];
            refined.push(a);
            refine_swept_chord(
                a,
                b,
                cell_mm,
                0,
                &is_occupied,
                &mut refine_scratch,
                &mut refined,
            );
        }

        if let Some((min_idx, _)) = refined
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.x.total_cmp(&b.x).then_with(|| a.y.total_cmp(&b.y)))
        {
            refined.rotate_left(min_idx);
        }
        if refined.len() >= 4 {
            refined = simplify_rdp_2d(&refined, true, 0.003);
        }
        if refined.len() >= 4 {
            refined =
                restore_sharp_corners_2d(&refined, cell_mm, &is_occupied, &mut refine_scratch);
        }
        if refined.len() < 3 {
            continue;
        }

        if let Some((min_idx, _)) = refined
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.x.total_cmp(&b.x).then_with(|| a.y.total_cmp(&b.y)))
        {
            refined.rotate_left(min_idx);
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

    let raw_reg = contours_to_region_set(&polylines);
    if raw_reg.is_empty() {
        return RegionSet2D::empty();
    }
    // Preserve filled outer boundaries so interior mounting holes never become floating pins
    // inside soft jaws or false-clearance cavities in flip-jigs.
    let filled_hulls: Vec<PocketRegion2D> = raw_reg
        .regions
        .into_iter()
        .map(|r| PocketRegion2D::new(r.outer_ccw, Vec::new()))
        .collect();
    RegionSet2D::from_regions(filled_hulls)
}

#[inline]
fn point_in_or_near_triangle_2d(p: DVec2, a: DVec2, b: DVec2, c: DVec2, margin_sq: f64) -> bool {
    if chip_math::point_in_triangle_2d(p, a, b, c) {
        return true;
    }
    if margin_sq <= 0.0 {
        return false;
    }
    chip_math::point_to_segment_dist_sq_2d(p, a, b) <= margin_sq
        || chip_math::point_to_segment_dist_sq_2d(p, b, c) <= margin_sq
        || chip_math::point_to_segment_dist_sq_2d(p, c, a) <= margin_sq
}

fn refine_swept_chord<F>(
    a: DVec2,
    b: DVec2,
    cell_mm: f64,
    depth: u32,
    is_occupied: &F,
    scratch: &mut Vec<usize>,
    out: &mut Vec<DVec2>,
) where
    F: Fn(DVec2, &mut Vec<usize>) -> bool,
{
    if depth >= 3 {
        return;
    }
    let seg = b - a;
    let len = seg.length();
    if len <= 0.15 {
        return;
    }
    let mid = 0.5 * (a + b);
    if !is_occupied(mid, scratch) {
        return;
    }
    // Occupied region lies to the LEFT of `a -> b`, so the outward direction is the RIGHT normal.
    let out_n = DVec2::new(seg.y / len, -seg.x / len);
    let mut step_hi = 0.02_f64;
    let max_step = cell_mm.max(0.5);
    while step_hi < max_step && is_occupied(mid + out_n * step_hi, scratch) {
        step_hi *= 2.0;
    }
    if is_occupied(mid + out_n * step_hi, scratch) {
        return;
    }
    let mut lo = 0.0_f64;
    let mut hi = step_hi;
    for _ in 0..7 {
        let m = 0.5 * (lo + hi);
        if is_occupied(mid + out_n * m, scratch) {
            lo = m;
        } else {
            hi = m;
        }
    }
    let p_safe = mid + out_n * hi;
    if p_safe.distance_squared(a) <= 1e-6 || p_safe.distance_squared(b) <= 1e-6 {
        return;
    }
    refine_swept_chord(a, p_safe, cell_mm, depth + 1, is_occupied, scratch, out);
    out.push(p_safe);
    refine_swept_chord(p_safe, b, cell_mm, depth + 1, is_occupied, scratch, out);
}

/// Collapses Marching-Squares single-cell corner bevels between adjacent long straight edges back
/// into their exact sharp intersection vertex when the true silhouette has a sharp corner (leaving
/// genuine rounded fillets untouched).
fn restore_sharp_corners_2d<F>(
    pts: &[DVec2],
    cell_mm: f64,
    is_occupied: &F,
    scratch: &mut Vec<usize>,
) -> Vec<DVec2>
where
    F: Fn(DVec2, &mut Vec<usize>) -> bool,
{
    let n = pts.len();
    if n < 5 {
        return pts.to_vec();
    }
    let min_long_len = (4.0 * cell_mm).max(1.5);
    let max_corner_dist = (2.5 * cell_mm).max(0.5);

    // Identify indices of "long" edges `i -> (i+1)%n`
    let mut long_edges: Vec<usize> = Vec::new();
    for i in 0..n {
        if pts[i].distance(pts[(i + 1) % n]) >= min_long_len {
            long_edges.push(i);
        }
    }
    if long_edges.len() < 2 {
        return pts.to_vec();
    }

    // Track corner replacements `(gap_start_vert_inclusive, gap_end_vert_inclusive, sharp_pt)`
    let mut corner_map: std::collections::HashMap<usize, (usize, DVec2)> =
        std::collections::HashMap::new();
    let mut skip_vert = vec![false; n];

    for k in 0..long_edges.len() {
        let e1 = long_edges[k];
        let e2 = long_edges[(k + 1) % long_edges.len()];
        let v_after_e1 = (e1 + 1) % n;
        let gap_verts = (e2 + n - v_after_e1) % n;
        if gap_verts == 0 || gap_verts > 6 {
            continue;
        }

        let a0 = pts[e1];
        let a1 = pts[v_after_e1];
        let b0 = pts[e2];
        let b1 = pts[(e2 + 1) % n];

        let d1 = (a1 - a0).normalize_or_zero();
        let d2 = (b1 - b0).normalize_or_zero();
        let denom = d1.x * d2.y - d1.y * d2.x;
        if denom.abs() < 0.34 {
            // Nearly collinear (< ~20° turn): not a sharp corner
            continue;
        }
        let diff = b0 - a1;
        let t1 = (diff.x * d2.y - diff.y * d2.x) / denom;
        let t2 = (diff.x * d1.y - diff.y * d1.x) / denom;
        // Intersection must lie forward of `a1` and backward of `b0` within `max_corner_dist`
        if t1 < -0.02 || t1 > max_corner_dist || t2 > 0.02 || -t2 > max_corner_dist {
            continue;
        }
        let corner = a1 + d1 * t1;

        // Verify every intermediate bevel vertex lies close to `corner`
        let mut all_near = true;
        for step in 0..=gap_verts {
            let vi = (v_after_e1 + step) % n;
            if pts[vi].distance(corner) > max_corner_dist {
                all_near = false;
                break;
            }
        }
        if !all_near {
            continue;
        }

        // Distinguish a genuine sharp corner from a true rounded fillet: an inward bisector probe
        // just 0.04 mm inside `corner` MUST lie inside the occupied swept silhouette!
        let inward = (DVec2::new(-d1.y, d1.x) + DVec2::new(-d2.y, d2.x)).normalize_or_zero();
        let probe = corner + inward * 0.04;
        if !is_occupied(corner, scratch) && !is_occupied(probe, scratch) {
            continue;
        }

        corner_map.insert(v_after_e1, (e2, corner));
        for step in 1..=gap_verts {
            let vi = (v_after_e1 + step) % n;
            skip_vert[vi] = true;
        }
    }

    if corner_map.is_empty() {
        return pts.to_vec();
    }

    let mut out = Vec::with_capacity(n);
    for (i, &p) in pts.iter().enumerate() {
        if let Some(&(_, sharp)) = corner_map.get(&i) {
            out.push(sharp);
        } else if !skip_vert[i] {
            out.push(p);
        }
    }
    if out.len() >= 3 { out } else { pts.to_vec() }
}
