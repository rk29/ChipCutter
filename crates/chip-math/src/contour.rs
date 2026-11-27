//! Deterministic dual-grid binary occupancy contour tracing (`trace_binary_grid_contours`).

use glam::DVec2;

use crate::geom2d::{ArcPolyline2D, ArcSegment2D};

#[inline]
fn edge_dest(u: usize, v: usize, dir: u8) -> (usize, usize) {
    match dir {
        0 => (u + 1, v),
        1 => (u, v + 1),
        2 => (u - 1, v),
        _ => (u, v - 1),
    }
}

/// Traces all closed boundary loops around binary occupancy grid `occ` (`w × h`, row-major
/// `occ[j * w + i]`) on the dual corner grid `(w + 1) × (h + 1)`.
///
/// Solid lies strictly to the left of every directed edge, so outer boundaries are strictly `CCW`
/// and interior holes are strictly `CW`, with saddle corners resolved deterministically via
/// right-turn priority (guaranteeing non-self-intersecting simple loops) and lossless collinear
/// vertex merging.
///
/// When `simplify_tol > 1e-6`, contours with more than 12 vertices additionally undergo closed-loop
/// Ramer–Douglas–Peucker decimation within `simplify_tol`.
#[allow(clippy::too_many_arguments)]
pub fn trace_binary_grid_contours(
    occ: &[bool],
    w: usize,
    h: usize,
    origin_x: f64,
    origin_y: f64,
    dx: f64,
    dy: f64,
    simplify_tol: f64,
) -> Vec<ArcPolyline2D> {
    if w == 0 || h == 0 || occ.len() < w.saturating_mul(h) {
        return Vec::new();
    }
    let gw = w + 1;
    let gh = h + 1;
    let mut out_mask = vec![0u8; gw * gh];
    let mut edge_count = 0usize;

    let is_solid = |ix: isize, iy: isize| -> bool {
        if ix < 0 || iy < 0 || ix >= w as isize || iy >= h as isize {
            false
        } else {
            occ[(iy as usize) * w + (ix as usize)]
        }
    };

    for j in 0..h {
        let row = j * w;
        for i in 0..w {
            if !occ[row + i] {
                continue;
            }
            let ii = i as isize;
            let jj = j as isize;
            // Bottom edge `(i, j) -> (i+1, j)` (dir 0: +X) if south neighbor empty
            if !is_solid(ii, jj - 1) {
                out_mask[j * gw + i] |= 1 << 0;
                edge_count += 1;
            }
            // Right edge `(i+1, j) -> (i+1, j+1)` (dir 1: +Y) if east neighbor empty
            if !is_solid(ii + 1, jj) {
                out_mask[j * gw + (i + 1)] |= 1 << 1;
                edge_count += 1;
            }
            // Top edge `(i+1, j+1) -> (i, j+1)` (dir 2: -X) if north neighbor empty
            if !is_solid(ii, jj + 1) {
                out_mask[(j + 1) * gw + (i + 1)] |= 1 << 2;
                edge_count += 1;
            }
            // Left edge `(i, j+1) -> (i, j)` (dir 3: -Y) if west neighbor empty
            if !is_solid(ii - 1, jj) {
                out_mask[(j + 1) * gw + i] |= 1 << 3;
                edge_count += 1;
            }
        }
    }

    if edge_count == 0 {
        return Vec::new();
    }

    let mut contours = Vec::new();
    let mut pts_buf: Vec<DVec2> = Vec::new();

    for sv in 0..gh {
        for su in 0..gw {
            loop {
                let mask = out_mask[sv * gw + su];
                if mask == 0 {
                    break;
                }
                let start_dir = mask.trailing_zeros() as u8;
                pts_buf.clear();

                let mut cur_u = su;
                let mut cur_v = sv;
                let mut cur_dir = start_dir;
                let mut prev_dir: Option<u8> = None;

                for _ in 0..=edge_count {
                    let vidx = cur_v * gw + cur_u;
                    if (out_mask[vidx] & (1 << cur_dir)) == 0 {
                        break;
                    }
                    out_mask[vidx] &= !(1 << cur_dir);

                    if prev_dir != Some(cur_dir) {
                        pts_buf.push(DVec2::new(
                            origin_x + (cur_u as f64) * dx,
                            origin_y + (cur_v as f64) * dy,
                        ));
                    }
                    prev_dir = Some(cur_dir);

                    let (nu, nv) = edge_dest(cur_u, cur_v, cur_dir);
                    if nu == su && nv == sv {
                        break;
                    }
                    let next_mask = out_mask[nv * gw + nu];
                    if next_mask == 0 {
                        break;
                    }
                    // Turn priority: right turn `(cur_dir + 3) % 4`, straight `cur_dir`, left `(cur_dir + 1) % 4`.
                    let candidates = [
                        (cur_dir + 3) & 3,
                        cur_dir,
                        (cur_dir + 1) & 3,
                        (cur_dir + 2) & 3,
                    ];
                    let Some(&ndir) = candidates.iter().find(|&&d| (next_mask & (1 << d)) != 0)
                    else {
                        break;
                    };
                    cur_u = nu;
                    cur_v = nv;
                    cur_dir = ndir;
                }

                // Collapse collinear seam if first and last edges share the same direction
                if pts_buf.len() >= 3 {
                    let first = pts_buf[0];
                    let second = pts_buf[1];
                    let last = pts_buf[pts_buf.len() - 1];
                    let d1 = second - first;
                    let d0 = first - last;
                    if (d0.x * d1.y - d0.y * d1.x).abs() <= 1e-9 && d0.dot(d1) > 0.0 {
                        pts_buf.remove(0);
                    }
                }

                let final_pts = if simplify_tol > 1e-6 && pts_buf.len() > 12 {
                    simplify_closed_polygon_rdp(&pts_buf, simplify_tol)
                } else {
                    pts_buf.clone()
                };

                if final_pts.len() >= 3 {
                    let mut segs = Vec::with_capacity(final_pts.len());
                    for k in 0..final_pts.len() {
                        let a = final_pts[k];
                        let b = final_pts[(k + 1) % final_pts.len()];
                        if a.distance_squared(b) > 1e-12 {
                            segs.push(ArcSegment2D::Line { start: a, end: b });
                        }
                    }
                    if segs.len() >= 3 {
                        contours.push(ArcPolyline2D {
                            segments: segs,
                            closed: true,
                        });
                    }
                }
            }
        }
    }

    contours
}

/// Lightweight closed-loop Douglas-Peucker simplification preserving corner fidelity within `tol`.
fn simplify_closed_polygon_rdp(pts: &[DVec2], tol: f64) -> Vec<DVec2> {
    let n = pts.len();
    if n <= 4 {
        return pts.to_vec();
    }
    let p0 = pts[0];
    let mut split = 1usize;
    let mut best_d2 = -1.0_f64;
    for (i, &p) in pts.iter().enumerate().skip(1) {
        let d2 = p.distance_squared(p0);
        if d2 > best_d2 {
            best_d2 = d2;
            split = i;
        }
    }
    if split == 0 || split >= n - 1 {
        return pts.to_vec();
    }

    let half_a = &pts[..=split];
    let mut half_b: Vec<DVec2> = Vec::with_capacity(n - split + 1);
    half_b.extend_from_slice(&pts[split..]);
    half_b.push(pts[0]);

    let tol2 = tol * tol;
    let simp_a = rdp_open_2d(half_a, tol2);
    let simp_b = rdp_open_2d(&half_b, tol2);

    let mut out = Vec::with_capacity(simp_a.len() + simp_b.len());
    out.extend_from_slice(&simp_a);
    if simp_b.len() > 2 {
        out.extend_from_slice(&simp_b[1..simp_b.len() - 1]);
    }
    if out.len() >= 3 { out } else { pts.to_vec() }
}

fn rdp_open_2d(pts: &[DVec2], tol2: f64) -> Vec<DVec2> {
    let n = pts.len();
    if n <= 2 {
        return pts.to_vec();
    }
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[n - 1] = true;
    let mut stack = vec![(0usize, n - 1)];

    while let Some((lo, hi)) = stack.pop() {
        if hi <= lo + 1 {
            continue;
        }
        let a = pts[lo];
        let b = pts[hi];
        let ab = b - a;
        let ab_len2 = ab.length_squared();
        let mut max_d2 = -1.0_f64;
        let mut max_i = lo;
        for (i, &p) in pts.iter().enumerate().take(hi).skip(lo + 1) {
            let d2 = if ab_len2 <= 1e-18 {
                p.distance_squared(a)
            } else {
                let t = ((p - a).dot(ab) / ab_len2).clamp(0.0, 1.0);
                p.distance_squared(a + ab * t)
            };
            if d2 > max_d2 {
                max_d2 = d2;
                max_i = i;
            }
        }
        if max_d2 > tol2 {
            keep[max_i] = true;
            stack.push((lo, max_i));
            stack.push((max_i, hi));
        }
    }

    pts.iter()
        .enumerate()
        .filter_map(|(i, &p)| keep[i].then_some(p))
        .collect()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::units::approx_eq;

    #[test]
    fn traces_ccw_outer_and_cw_hole_contours_with_collinear_merging() {
        // 5x5 grid with a 3x3 ring (center cell empty)
        let mut occ = vec![false; 25];
        for j in 1..=3 {
            for i in 1..=3 {
                if !(i == 2 && j == 2) {
                    occ[j * 5 + i] = true;
                }
            }
        }
        let loops = trace_binary_grid_contours(&occ, 5, 5, 0.0, 0.0, 2.0, 2.0, 0.0);
        assert_eq!(loops.len(), 2);
        let mut areas: Vec<f64> = loops.iter().map(ArcPolyline2D::signed_area).collect();
        areas.sort_by(f64::total_cmp);
        // Inner 1x1 cell (2x2 mm) hole is CW (-4 mm²); outer 3x3 cells (6x6 mm) is CCW (+36 mm²)
        assert!(approx_eq(areas[0], -4.0, 1e-9));
        assert!(approx_eq(areas[1], 36.0, 1e-9));
        // Collinear merge collapses each square loop to exactly 4 edges
        assert!(loops.iter().all(|l| l.segments.len() == 4));
    }
}
