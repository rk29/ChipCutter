//! Exact 2D Ramer–Douglas–Peucker polyline decimation preserving corners within tolerance.

use chip_math::DVec2;

/// Ramer–Douglas–Peucker 2D polyline simplification preserving corners within `tol_mm`.
pub fn simplify_rdp_2d(pts: &[DVec2], closed: bool, tol_mm: f64) -> Vec<DVec2> {
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
        rdp_recurse_2d(&pts[..=split_idx], tol_sq, &mut h1);
        let mut second_slice = Vec::with_capacity(pts.len() - split_idx + 1);
        second_slice.extend_from_slice(&pts[split_idx..]);
        second_slice.push(first);
        let mut h2 = Vec::new();
        rdp_recurse_2d(&second_slice, tol_sq, &mut h2);
        let mut out = h1;
        if h2.len() > 2 {
            out.extend_from_slice(&h2[1..h2.len() - 1]);
        }
        if out.len() >= 3 { out } else { pts.to_vec() }
    } else {
        let mut out = Vec::new();
        rdp_recurse_2d(pts, tol_sq, &mut out);
        out
    }
}

fn rdp_recurse_2d(pts: &[DVec2], tol_sq: f64, out: &mut Vec<DVec2>) {
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
        rdp_recurse_2d(&pts[..=best_idx], tol_sq, &mut left);
        let mut right = Vec::new();
        rdp_recurse_2d(&pts[best_idx..], tol_sq, &mut right);
        out.extend_from_slice(&left[..left.len() - 1]);
        out.extend_from_slice(&right);
    } else {
        out.push(a);
        out.push(b);
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn simplify_rdp_2d_collapses_collinear_points_and_preserves_corners() {
        let open = vec![
            DVec2::new(0.0, 0.0),
            DVec2::new(5.0, 0.001),
            DVec2::new(10.0, 0.0),
            DVec2::new(10.0, 5.0),
            DVec2::new(10.0, 10.0),
        ];
        let simplified = simplify_rdp_2d(&open, false, 0.01);
        assert_eq!(
            simplified,
            vec![
                DVec2::new(0.0, 0.0),
                DVec2::new(10.0, 0.0),
                DVec2::new(10.0, 10.0),
            ]
        );

        let closed = vec![
            DVec2::new(0.0, 0.0),
            DVec2::new(5.0, 0.0),
            DVec2::new(10.0, 0.0),
            DVec2::new(10.0, 5.0),
            DVec2::new(10.0, 10.0),
            DVec2::new(5.0, 10.0),
            DVec2::new(0.0, 10.0),
            DVec2::new(0.0, 5.0),
        ];
        let closed_out = simplify_rdp_2d(&closed, true, 0.01);
        assert_eq!(closed_out.len(), 4);
    }
}
