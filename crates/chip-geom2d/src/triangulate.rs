//! Exact `f64` 2D ear-clipping polygon triangulation (simple polygons and polygons with holes).

use chip_math::{DVec2, point_in_triangle_2d};

#[inline]
fn cross2d(u: DVec2, v: DVec2) -> f64 {
    u.x * v.y - u.y * v.x
}

fn signed_area2_ring(ring: &[DVec2]) -> f64 {
    let n = ring.len();
    let mut area2 = 0.0_f64;
    for i in 0..n {
        let a = ring[i];
        let b = ring[(i + 1) % n];
        area2 += a.x * b.y - b.x * a.y;
    }
    area2
}

fn ensure_ring_orientation(mut ring: Vec<DVec2>, ccw: bool) -> Vec<DVec2> {
    if ring.len() >= 3 && (signed_area2_ring(&ring) >= 0.0) != ccw {
        ring.reverse();
    }
    ring
}

/// Quality-weighted ear-clipping triangulator for a simple 2D polygon `pts`.
///
/// Automatically normalizes winding to CCW (returning index triples oriented CCW in `pts`).
/// Handles concave polygons and gracefully cleans up collinear spikes without panicking.
pub fn ear_clip_ccw_2d(pts: &[DVec2]) -> Vec<[usize; 3]> {
    let n = pts.len();
    if n < 3 {
        return Vec::new();
    }

    let mut indices: Vec<usize> = if signed_area2_ring(pts) >= 0.0 {
        (0..n).collect()
    } else {
        (0..n).rev().collect()
    };
    let mut tris = Vec::with_capacity(n.saturating_sub(2));

    let mut guard = n * n + 16;
    while indices.len() > 3 && guard > 0 {
        guard -= 1;
        let m = indices.len();
        let mut clipped = false;

        // Pick the best strictly-convex ear (preferring larger minimum angle / non-sliver)
        let mut best_ear: Option<(usize, f64)> = None;
        for i in 0..m {
            let i_prev = indices[(i + m - 1) % m];
            let i_curr = indices[i];
            let i_next = indices[(i + 1) % m];
            let a = pts[i_prev];
            let b = pts[i_curr];
            let c = pts[i_next];

            let area2 = cross2d(b - a, c - a);
            if area2 <= 1e-10 {
                continue;
            }
            let mut contains_other = false;
            for (k_pos, &idx) in indices.iter().enumerate() {
                if k_pos == i || k_pos == (i + m - 1) % m || k_pos == (i + 1) % m {
                    continue;
                }
                let p = pts[idx];
                if p.distance_squared(a) <= 1e-12
                    || p.distance_squared(b) <= 1e-12
                    || p.distance_squared(c) <= 1e-12
                {
                    continue;
                }
                if point_in_triangle_2d(p, a, b, c) {
                    contains_other = true;
                    break;
                }
            }
            if contains_other {
                continue;
            }
            let perim2 =
                a.distance_squared(b) + b.distance_squared(c) + c.distance_squared(a) + 1e-12;
            let quality = area2 / perim2;
            if best_ear.is_none_or(|(_, q)| quality > q) {
                best_ear = Some((i, quality));
            }
        }

        if let Some((ear_pos, _)) = best_ear {
            let i_prev = indices[(ear_pos + m - 1) % m];
            let i_curr = indices[ear_pos];
            let i_next = indices[(ear_pos + 1) % m];
            tris.push([i_prev, i_curr, i_next]);
            indices.remove(ear_pos);
            clipped = true;
        }

        if !clipped {
            // Remove collinear / zero-area spikes if any remain
            let mut removed_deg = false;
            for i in 0..m {
                let a = pts[indices[(i + m - 1) % m]];
                let b = pts[indices[i]];
                let c = pts[indices[(i + 1) % m]];
                if cross2d(b - a, c - a).abs() <= 1e-8 {
                    indices.remove(i);
                    removed_deg = true;
                    break;
                }
            }
            if !removed_deg {
                // Fallback: clip first positive-area ear
                for i in 0..m {
                    let i_prev = indices[(i + m - 1) % m];
                    let i_curr = indices[i];
                    let i_next = indices[(i + 1) % m];
                    if cross2d(pts[i_curr] - pts[i_prev], pts[i_next] - pts[i_prev]) > 0.0 {
                        tris.push([i_prev, i_curr, i_next]);
                        indices.remove(i);
                        removed_deg = true;
                        break;
                    }
                }
                if !removed_deg {
                    break;
                }
            }
        }
    }

    if indices.len() == 3 {
        let i0 = indices[0];
        let i1 = indices[1];
        let i2 = indices[2];
        if cross2d(pts[i1] - pts[i0], pts[i2] - pts[i0]) > 1e-10 {
            tris.push([i0, i1, i2]);
        }
    }

    tris
}

/// Exact `f64` 2D ear-clipping triangulator supporting a CCW outer ring with zero or more CW holes.
///
/// Holes are spliced into the outer boundary via non-intersecting mutual-visibility bridges
/// (David Eberly's rightmost-vertex bridge algorithm) before ear clipping.
pub fn triangulate_polygon_with_holes_2d(outer: &[DVec2], holes: &[Vec<DVec2>]) -> Vec<[DVec2; 3]> {
    let mut poly = ensure_ring_orientation(outer.to_vec(), true);
    if poly.len() < 3 {
        return Vec::new();
    }

    // Sort holes descending by rightmost X so bridges never cross unprocessed holes
    let mut ordered_holes: Vec<(f64, usize, Vec<DVec2>)> = holes
        .iter()
        .filter(|h| h.len() >= 3)
        .filter_map(|h| {
            let cw = ensure_ring_orientation(h.clone(), false);
            let (idx, max_x) = cw
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.x.total_cmp(&b.1.x))
                .map(|(i, p)| (i, p.x))?;
            Some((max_x, idx, cw))
        })
        .collect();
    ordered_holes.sort_by(|a, b| b.0.total_cmp(&a.0));

    for (_, hole_right_idx, hole) in ordered_holes {
        let m = hole[hole_right_idx];
        if let Some(outer_idx) = find_visible_bridge_vertex(&poly, m) {
            let mut merged = Vec::with_capacity(poly.len() + hole.len() + 2);
            merged.extend_from_slice(&poly[..=outer_idx]);
            for k in 0..=hole.len() {
                merged.push(hole[(hole_right_idx + k) % hole.len()]);
            }
            merged.push(poly[outer_idx]);
            merged.extend_from_slice(&poly[(outer_idx + 1)..]);
            poly = merged;
        }
    }

    ear_clip_ccw_2d(&poly)
        .into_iter()
        .map(|[a, b, c]| [poly[a], poly[b], poly[c]])
        .collect()
}

fn find_visible_bridge_vertex(poly: &[DVec2], m: DVec2) -> Option<usize> {
    let n = poly.len();
    // 1. Cast horizontal ray `+X` from `m` to find closest intersection with an edge of `poly`
    let mut best_x = f64::INFINITY;
    let mut best_edge_idx: Option<usize> = None;
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        let crosses = (a.y <= m.y && b.y > m.y) || (b.y <= m.y && a.y > m.y);
        if !crosses {
            continue;
        }
        let t = (m.y - a.y) / (b.y - a.y);
        let x_hit = a.x + t * (b.x - a.x);
        if x_hit >= m.x - 1e-9 && x_hit < best_x {
            best_x = x_hit;
            best_edge_idx = Some(i);
        }
    }

    let edge_i = best_edge_idx?;
    let a = poly[edge_i];
    let b = poly[(edge_i + 1) % n];
    let p_idx = if a.x >= b.x { edge_i } else { (edge_i + 1) % n };
    let p = poly[p_idx];
    let i_pt = DVec2::new(best_x, m.y);

    // 2. Check if any reflex vertex of `poly` lies strictly inside triangle `(m, i_pt, p)`
    let mut candidate_idx = p_idx;
    let mut best_angle = f64::INFINITY;
    let mut best_dist = f64::INFINITY;

    for (idx, &v) in poly.iter().enumerate() {
        if idx == p_idx {
            continue;
        }
        if v.distance_squared(m) < 1e-12 || v.distance_squared(p) < 1e-12 {
            continue;
        }
        if point_in_triangle_2d(v, m, i_pt, p) {
            let dir = v - m;
            let ang = dir.y.atan2(dir.x).abs();
            let dist = dir.length_squared();
            if ang < best_angle - 1e-9 || ((ang - best_angle).abs() <= 1e-9 && dist < best_dist) {
                best_angle = ang;
                best_dist = dist;
                candidate_idx = idx;
            }
        }
    }

    Some(candidate_idx)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::approx_eq;

    use super::*;

    #[test]
    fn triangulates_concave_polygon_with_hole_preserving_exact_area() {
        let outer = vec![
            DVec2::new(0.0, 0.0),
            DVec2::new(10.0, 0.0),
            DVec2::new(10.0, 10.0),
            DVec2::new(0.0, 10.0),
        ];
        let hole = vec![
            DVec2::new(3.0, 3.0),
            DVec2::new(3.0, 7.0),
            DVec2::new(7.0, 7.0),
            DVec2::new(7.0, 3.0),
        ];
        let tris = triangulate_polygon_with_holes_2d(&outer, &[hole]);
        let area: f64 = tris
            .iter()
            .map(|&[a, b, c]| 0.5 * cross2d(b - a, c - a))
            .sum();
        assert!(approx_eq(area, 84.0, 1e-9));
    }
}
