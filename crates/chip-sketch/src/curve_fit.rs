//! Adaptive cubic/quadratic Bézier -> circular-arc (`ArcPolyline2D`) fitting (`11-sketches-vectors.md`).
//!
//! Approximates smooth `kurbo::BezPath` subpaths using true circular `ArcSegment2D::Arc` (`G2`/`G3`)
//! and `ArcSegment2D::Line` (`G1`) spans within `arc_tol_mm` so imported SVG paths and DXF `SPLINE`s
//! machine smoothly without dense micro-chord faceting.

use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2};
use kurbo::{BezPath, CubicBez, ParamCurve, ParamCurveDeriv, PathEl, Point, QuadBez};

#[inline]
fn to_dvec2(p: Point) -> DVec2 {
    DVec2::new(p.x, p.y)
}

/// Converts a `kurbo::BezPath` into zero or more [`ArcPolyline2D`] subpaths within `arc_tol_mm`.
pub fn fit_bezpath_to_arc_polylines(path: &BezPath, arc_tol_mm: f64) -> Vec<ArcPolyline2D> {
    let tol = arc_tol_mm.clamp(1e-4, 1.0);
    let mut out = Vec::new();
    let mut cur_segs: Vec<ArcSegment2D> = Vec::new();
    let mut subpath_start: Option<Point> = None;
    let mut last_pt: Option<Point> = None;

    let flush_subpath =
        |segs: &mut Vec<ArcSegment2D>, closed_hint: bool, dest: &mut Vec<ArcPolyline2D>| {
            if segs.is_empty() {
                return;
            }
            let closed = closed_hint
                || segs
                    .first()
                    .zip(segs.last())
                    .is_some_and(|(a, b)| a.start().distance(b.end()) <= 1e-4);
            dest.push(ArcPolyline2D {
                segments: std::mem::take(segs),
                closed,
            });
        };

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                flush_subpath(&mut cur_segs, false, &mut out);
                subpath_start = Some(p);
                last_pt = Some(p);
            }
            PathEl::LineTo(p) => {
                if let Some(p0) = last_pt {
                    let a = to_dvec2(p0);
                    let b = to_dvec2(p);
                    if a.distance(b) > 1e-5 {
                        cur_segs.push(ArcSegment2D::Line { start: a, end: b });
                        last_pt = Some(p);
                    }
                } else {
                    subpath_start = Some(p);
                    last_pt = Some(p);
                }
            }
            PathEl::QuadTo(p1, p2) => {
                if let Some(p0) = last_pt {
                    let cubic = QuadBez::new(p0, p1, p2).raise();
                    fit_cubic_recursive(cubic, tol, 0, &mut cur_segs);
                    last_pt = Some(p2);
                }
            }
            PathEl::CurveTo(p1, p2, p3) => {
                if let Some(p0) = last_pt {
                    let cubic = CubicBez::new(p0, p1, p2, p3);
                    fit_cubic_recursive(cubic, tol, 0, &mut cur_segs);
                    last_pt = Some(p3);
                }
            }
            PathEl::ClosePath => {
                if let (Some(p0), Some(ps)) = (last_pt, subpath_start) {
                    let a = to_dvec2(p0);
                    let b = to_dvec2(ps);
                    if a.distance(b) > 1e-5 {
                        cur_segs.push(ArcSegment2D::Line { start: a, end: b });
                    }
                }
                flush_subpath(&mut cur_segs, true, &mut out);
                subpath_start = None;
                last_pt = None;
            }
        }
    }

    flush_subpath(&mut cur_segs, false, &mut out);
    out
}

fn fit_cubic_recursive(cubic: CubicBez, tol_mm: f64, depth: u32, out: &mut Vec<ArcSegment2D>) {
    let p0 = to_dvec2(cubic.p0);
    let p3 = to_dvec2(cubic.p3);
    let chord_len = p0.distance(p3);

    // Guard degenerate micro-spans (note: closed loop `p0 == p3` with non-zero control polygon must subdivide first!)
    let ctrl_span = to_dvec2(cubic.p0)
        .distance(to_dvec2(cubic.p1))
        .max(to_dvec2(cubic.p1).distance(to_dvec2(cubic.p2)))
        .max(to_dvec2(cubic.p2).distance(to_dvec2(cubic.p3)));
    if ctrl_span <= 1e-5 {
        return;
    }

    // 1. Check if cubic is already straight within `0.25 * tol_mm`
    if chord_len > 1e-5 && is_cubic_collinear(&cubic, 0.25 * tol_mm) {
        out.push(ArcSegment2D::Line { start: p0, end: p3 });
        return;
    }

    // 2. Attempt single exact circular arc fit through `(p0, c(0.5), p3)` when sweep < ~140°
    if depth >= 1
        && chord_len > 1e-4
        && let Some(arc_seg) = try_fit_single_circular_arc(&cubic, tol_mm)
    {
        out.push(arc_seg);
        return;
    }
    // Also allow depth == 0 single-arc fit when control polygon doesn't flip curvature sign (no S-inflection)
    if depth == 0
        && chord_len > 1e-4
        && !has_inflection_sign_flip(&cubic)
        && let Some(arc_seg) = try_fit_single_circular_arc(&cubic, tol_mm)
    {
        out.push(arc_seg);
        return;
    }

    // 3. Subdivide or emit linear fallback at max recursion depth
    if depth >= 9 {
        if chord_len > 1e-5 {
            out.push(ArcSegment2D::Line { start: p0, end: p3 });
        }
        return;
    }

    let left = cubic.subsegment(0.0..0.5);
    let right = cubic.subsegment(0.5..1.0);
    fit_cubic_recursive(left, tol_mm, depth + 1, out);
    fit_cubic_recursive(right, tol_mm, depth + 1, out);
}

fn is_cubic_collinear(cubic: &CubicBez, line_tol: f64) -> bool {
    let a = to_dvec2(cubic.p0);
    let b = to_dvec2(cubic.p3);
    let ab = b - a;
    let len = ab.length();
    if len <= 1e-6 {
        return false;
    }
    let dir = ab / len;
    for t in [0.25, 0.5, 0.75] {
        let p = to_dvec2(cubic.eval(t));
        let v = p - a;
        let perp = (v.x * dir.y - v.y * dir.x).abs();
        let along = v.dot(dir);
        if perp > line_tol || along < -line_tol || along > len + line_tol {
            return false;
        }
    }
    true
}

fn has_inflection_sign_flip(cubic: &CubicBez) -> bool {
    let p0 = to_dvec2(cubic.p0);
    let p1 = to_dvec2(cubic.p1);
    let p2 = to_dvec2(cubic.p2);
    let p3 = to_dvec2(cubic.p3);
    let chord = p3 - p0;
    let c1 = chord.x * (p1.y - p0.y) - chord.y * (p1.x - p0.x);
    let c2 = chord.x * (p2.y - p0.y) - chord.y * (p2.x - p0.x);
    c1 * c2 < -1e-9
}

/// Fits the unique circular arc passing through `(c(0), c(0.5), c(1))` and verifies both interior
/// radial/point error `<= tol_mm` and monotonic angular progression (< 160°).
fn try_fit_single_circular_arc(cubic: &CubicBez, tol_mm: f64) -> Option<ArcSegment2D> {
    let p0 = to_dvec2(cubic.p0);
    let pm = to_dvec2(cubic.eval(0.5));
    let p1 = to_dvec2(cubic.p3);

    let (center, radius, ccw) = circumcircle_through_three_points(p0, pm, p1)?;
    if !radius.is_finite() || radius <= 1e-4 || radius > 50_000.0 {
        return None;
    }

    let candidate = ArcSegment2D::Arc {
        start: p0,
        end: p1,
        center,
        radius,
        ccw,
    };
    let sweep_abs = candidate.signed_sweep_rad().abs();
    if sweep_abs <= 1e-4 || sweep_abs > 2.80 {
        return None;
    }

    // Check endpoint tangent alignment (within ~8° so G1 continuity is clean)
    let deriv = cubic.deriv();
    let b_tan0 = to_dvec2(deriv.eval(0.0)).normalize_or_zero();
    let b_tan1 = to_dvec2(deriv.eval(1.0)).normalize_or_zero();
    if let (Some((_, a_tan0)), Some((_, a_tan1))) = (candidate.eval_at(0.0), candidate.eval_at(1.0))
    {
        if b_tan0.length_squared() > 0.5 && b_tan0.dot(a_tan0) < 0.985 {
            return None;
        }
        if b_tan1.length_squared() > 0.5 && b_tan1.dot(a_tan1) < 0.985 {
            return None;
        }
    }

    // Check 6 interior sample points against the arc within `tol_mm`
    let single_poly = ArcPolyline2D {
        segments: vec![candidate],
        closed: false,
    };
    for t in [0.125, 0.25, 0.375, 0.625, 0.75, 0.875] {
        let pt = to_dvec2(cubic.eval(t));
        let radial_err = (pt.distance(center) - radius).abs();
        if radial_err > tol_mm {
            return None;
        }
        if single_poly.distance_to_point(pt) > tol_mm {
            return None;
        }
    }

    Some(candidate)
}

pub use chip_math::circumcircle_through_three_points;
