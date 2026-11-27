//! Marching-squares 2D iso-contour segment extraction across scalar fields.

use chip_math::DVec2;

/// Extracts marching-squares iso-contour line segments at `iso` across row-major scalar grid
/// `dist` (`nx × ny`, origin `(min_x, min_y)`, uniform cell pitch `cell`).
///
/// Segments are oriented with `dist >= iso` strictly on the LEFT of each directed segment.
pub fn marching_squares_segments(
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
