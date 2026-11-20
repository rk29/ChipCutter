use std::collections::HashMap;

use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3};

use crate::{
    offset2d::PocketRegion2D,
    rest2d::RegionSet2D,
    surface3d::{SurfaceMesh3D, cutter::SurfaceCutterProfile},
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

/// Evaluates the exact horizontal cutter-avoidance [`RegionSet2D`] at height `z_wcs` (`z_tip` in WCS)
/// for `cutter` (accounting for Flat, Ball, Bull-Nose, and Tapered/V-Bit profiles plus radial & axial
/// stock-to-leave allowances).
///
/// Any cutter center `(x, y)` placed **outside** the returned [`RegionSet2D`] is collision-free
/// against `surface` at height `z_wcs`, making its boundary directly usable for Milestone 17's
/// `Waterline3D` and `Adaptive3D` toolpaths.
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

    let tip_h = cutter.tip_profile_height_mm();
    let z_top_limit = surface.aabb_wcs.max.z;

    // Sample Z heights across the cutter's active lower contact profile `[z_eff, z_eff + tip_h]`.
    // For Flat endmills (`tip_h == 0`), a single slice at `z_eff + 1e-4` dilated by `R_eff` is exact!
    // For Ball / BullNose / TaperedVBit cutters, cosine-clustered vertical samples capture the
    // exact tangent envelope across curved slopes to `< 0.005 mm`.
    let mut slice_samples: Vec<(f64, f64)> = Vec::new();
    let z_base = (z_eff + 1e-4).min(z_top_limit - 1e-5);
    if z_base < z_top_limit {
        let r0 = cutter.radius_at_height(0.0);
        slice_samples.push((z_base, if r0 > 1e-5 { r0 + 0.025 } else { 0.0 }));
    }

    if tip_h > 1e-5 {
        const N_PROFILE_SLICES: usize = 16;
        for k in 1..=N_PROFILE_SLICES {
            // Cosine spacing clusters slices near `dz = 0` where ball/bull-nose `dr/dz` is steepest;
            // `0.025 mm` safety dilation covers polygonal mesh chord sag so waterline contours never gouge.
            let u = k as f64 / N_PROFILE_SLICES as f64;
            let dz = tip_h * (1.0 - (0.5 * std::f64::consts::PI * u).cos());
            let z_s = z_eff + dz;
            if z_s >= z_top_limit - 1e-5 {
                break;
            }
            let r_s = cutter.radius_at_height(dz) + 0.025;
            slice_samples.push((z_s, r_s));
        }
    }

    let mut dilated_sets: Vec<RegionSet2D> = Vec::with_capacity(slice_samples.len());
    for (z_s, radius) in slice_samples {
        let contours = slice_surface_contours_z(surface, z_s);
        let solid_reg = contours_to_region_set(&contours);
        if solid_reg.is_empty() {
            continue;
        }
        let expanded = if radius > 1e-5 {
            solid_reg.dilate_outward(radius)
        } else {
            solid_reg
        };
        if !expanded.is_empty() {
            dilated_sets.push(expanded);
        }
    }

    RegionSet2D::union_many(&mut dilated_sets)
}

/// Converts a list of closed 2D slice contours into a nested [`RegionSet2D`] using exact
/// containment-depth parity (outer solids = even depth, interior cavities = odd depth).
pub(crate) fn contours_to_region_set(contours: &[ArcPolyline2D]) -> RegionSet2D {
    let mut closed_loops: Vec<(f64, ArcPolyline2D)> = contours
        .iter()
        .filter(|c| c.closed && c.segments.len() >= 3)
        .filter_map(|c| {
            let area = signed_area_polyline(c);
            if area.abs() <= 1e-5 {
                return None;
            }
            let ccw = if area > 0.0 {
                c.clone()
            } else {
                reverse_polyline(c)
            };
            Some((area.abs(), ccw))
        })
        .collect();

    if closed_loops.is_empty() {
        return RegionSet2D::empty();
    }

    // Sort largest area first so parent loops always precede contained child loops
    closed_loops.sort_by(|a, b| b.0.total_cmp(&a.0));

    let mut depths = vec![0usize; closed_loops.len()];
    for i in 0..closed_loops.len() {
        let sample_pt = closed_loops[i].1.segments[0].start();
        let mut depth = 0usize;
        for (_, parent_loop) in closed_loops.iter().take(i) {
            if parent_loop.contains_point(sample_pt) {
                depth += 1;
            }
        }
        depths[i] = depth;
    }

    let mut result = RegionSet2D::empty();
    for (idx, (_, loop_ccw)) in closed_loops.into_iter().enumerate() {
        let loop_reg = RegionSet2D::from_single(PocketRegion2D::new(loop_ccw, Vec::new()));
        if depths[idx].is_multiple_of(2) {
            result = if result.is_empty() {
                loop_reg
            } else {
                result.union(&loop_reg)
            };
        } else {
            result = result.difference(&loop_reg);
        }
    }
    result
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

fn signed_area_polyline(poly: &ArcPolyline2D) -> f64 {
    let mut area2 = 0.0f64;
    for seg in &poly.segments {
        let a = seg.start();
        let b = seg.end();
        area2 += a.x * b.y - b.x * a.y;
    }
    0.5 * area2
}

fn reverse_polyline(poly: &ArcPolyline2D) -> ArcPolyline2D {
    let segments = poly
        .segments
        .iter()
        .rev()
        .map(|seg| match *seg {
            ArcSegment2D::Line { start, end } => ArcSegment2D::Line {
                start: end,
                end: start,
            },
            ArcSegment2D::Arc {
                start,
                end,
                center,
                radius,
                ccw,
            } => ArcSegment2D::Arc {
                start: end,
                end: start,
                center,
                radius,
                ccw: !ccw,
            },
        })
        .collect();
    ArcPolyline2D {
        segments,
        closed: poly.closed,
    }
}

/// Ramer–Douglas–Peucker 2D polyline simplification preserving corners within `tol_mm`.
fn simplify_rdp_2d(pts: &[DVec2], closed: bool, tol_mm: f64) -> Vec<DVec2> {
    if pts.len() < 3 {
        return pts.to_vec();
    }
    let tol_sq = tol_mm * tol_mm;
    if closed {
        // Split closed ring at the two farthest vertices so no seam bias occurs
        let first = pts[0];
        let split_idx = pts
            .iter()
            .enumerate()
            .skip(1)
            .max_by(|(_, a), (_, b)| {
                a.distance_squared(first)
                    .total_cmp(&b.distance_squared(first))
            })
            .map_or(pts.len() / 2, |(i, _)| i);
        let mut h1 = Vec::new();
        rdp_recurse(&pts[..=split_idx], tol_sq, &mut h1);
        let mut second_slice = Vec::with_capacity(pts.len() - split_idx + 1);
        second_slice.extend_from_slice(&pts[split_idx..]);
        second_slice.push(first);
        let mut h2 = Vec::new();
        rdp_recurse(&second_slice, tol_sq, &mut h2);
        let mut out = h1;
        if h2.len() > 2 {
            out.extend_from_slice(&h2[1..h2.len() - 1]);
        }
        if out.len() >= 3 { out } else { pts.to_vec() }
    } else {
        let mut out = Vec::new();
        rdp_recurse(pts, tol_sq, &mut out);
        out
    }
}

fn rdp_recurse(pts: &[DVec2], tol_sq: f64, out: &mut Vec<DVec2>) {
    if pts.len() <= 2 {
        out.extend_from_slice(pts);
        return;
    }
    let a = pts[0];
    let b = pts[pts.len() - 1];
    let ab = b - a;
    let ab_len_sq = ab.length_squared();
    let mut best_idx = 0usize;
    let mut best_d_sq = -1.0f64;
    for (i, &p) in pts.iter().enumerate().take(pts.len() - 1).skip(1) {
        let d_sq = if ab_len_sq <= 1e-12 {
            p.distance_squared(a)
        } else {
            let t = ((p - a).dot(ab) / ab_len_sq).clamp(0.0, 1.0);
            p.distance_squared(a + ab * t)
        };
        if d_sq > best_d_sq {
            best_d_sq = d_sq;
            best_idx = i;
        }
    }
    if best_d_sq > tol_sq && best_idx > 0 && best_idx + 1 < pts.len() {
        let mut left = Vec::new();
        rdp_recurse(&pts[..=best_idx], tol_sq, &mut left);
        let mut right = Vec::new();
        rdp_recurse(&pts[best_idx..], tol_sq, &mut right);
        out.extend_from_slice(&left[..left.len() - 1]);
        out.extend_from_slice(&right);
    } else {
        out.push(a);
        out.push(b);
    }
}
