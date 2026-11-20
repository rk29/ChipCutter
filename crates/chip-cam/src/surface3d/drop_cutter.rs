use chip_math::{DVec2, DVec3};
use rayon::prelude::*;

use crate::{
    rest2d::RegionSet2D,
    surface3d::{
        SurfaceMesh3D,
        cutter::{SurfaceCutterProfile, SurfaceCutterShape},
    },
};

/// Evaluates the exact analytical `f64` Drop-Cutter contact Z height (`z_tip` in Setup WCS mm)
/// at horizontal cutter axis position `(x, y)` above `surface`, floored by `z_floor`.
///
/// Returns `None` if no triangle in `surface` overlaps the cutter's horizontal footprint
/// (or if `x`/`y` are non-finite). When a contact occurs, `stock_axial_mm` is added to the
/// highest contact height and clamped to `>= z_floor`.
pub fn drop_cutter_z(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    x: f64,
    y: f64,
    z_floor: f64,
) -> Option<f64> {
    let mut scratch = Vec::with_capacity(64);
    drop_cutter_z_with_scratch(surface, cutter, x, y, z_floor, &mut scratch)
}

/// Zero-allocation inner [`drop_cutter_z`] reusing `candidate_buf` across dense scanlines.
pub fn drop_cutter_z_with_scratch(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    x: f64,
    y: f64,
    z_floor: f64,
    candidate_buf: &mut Vec<usize>,
) -> Option<f64> {
    if !x.is_finite() || !y.is_finite() || !z_floor.is_finite() {
        return None;
    }
    let r_max = cutter.max_radius_mm();
    surface
        .bvh
        .query_xy_circle_into(x, y, r_max + 1e-6, candidate_buf);
    if candidate_buf.is_empty() {
        return None;
    }

    let c_xy = DVec2::new(x, y);
    let mut best_z = f64::NEG_INFINITY;
    let mut hit_any = false;

    for &ti in candidate_buf.iter() {
        let tri = &surface.triangles[ti];
        // Quick Z pruning: if even the highest vertex of this triangle cannot beat `best_z`, skip!
        if hit_any && tri.aabb.max.z <= best_z {
            continue;
        }
        let [i0, i1, i2] = tri.indices;
        let v0 = surface.vertices_wcs[i0 as usize];
        let v1 = surface.vertices_wcs[i1 as usize];
        let v2 = surface.vertices_wcs[i2 as usize];

        if let Some(z_tri) = drop_cutter_triangle(cutter, c_xy, v0, v1, v2)
            && z_tri.is_finite()
            && (!hit_any || z_tri > best_z)
        {
            best_z = z_tri;
            hit_any = true;
        }
    }

    if hit_any {
        Some((best_z + cutter.stock_axial_mm).max(z_floor))
    } else {
        None
    }
}

/// Evaluates Drop-Cutter heights in parallel (`rayon`) across a list of `(x, y)` WCS sample points,
/// optionally filtering points outside a 2D containment boundary (`RegionSet2D`).
pub fn drop_cutter_points_parallel(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    points_xy: &[DVec2],
    z_floor: f64,
    containment: Option<&RegionSet2D>,
) -> Vec<Option<f64>> {
    points_xy
        .par_iter()
        .map_init(
            || Vec::with_capacity(64),
            |scratch, &pt| {
                if let Some(bounds) = containment
                    && !bounds.contains_point(pt)
                {
                    return None;
                }
                drop_cutter_z_with_scratch(surface, cutter, pt.x, pt.y, z_floor, scratch)
            },
        )
        .collect()
}

/// Evaluates exact analytical `f64` Drop-Cutter contact against a single 3D triangle `(v0, v1, v2)`:
/// 1. Facet interior / rim contact,
/// 2. All 3 triangle edges clipped to the cutter disk `d <= R_max`,
/// 3. All 3 triangle vertices inside `d <= R_max`.
pub(crate) fn drop_cutter_triangle(
    cutter: &SurfaceCutterProfile,
    c_xy: DVec2,
    v0: DVec3,
    v1: DVec3,
    v2: DVec3,
) -> Option<f64> {
    let r_max = cutter.max_radius_mm();
    let mut best: Option<f64> = None;
    let mut consider = |z: f64| {
        if z.is_finite() && best.is_none_or(|b| z > b) {
            best = Some(z);
        }
    };

    // 1. Vertex contacts
    for v in [v0, v1, v2] {
        let d = (DVec2::new(v.x, v.y) - c_xy).length();
        if let Some(h) = cutter.profile_height_at_radius(d) {
            consider(v.z - h);
        }
    }

    // 2. Edge contacts (exact closed-form for Flat/Ball/VBit; safeguarded unimodal refinement for BullNose)
    for (a, b) in [(v0, v1), (v1, v2), (v2, v0)] {
        if let Some(z_e) = drop_cutter_edge(cutter, c_xy, a, b, r_max) {
            consider(z_e);
        }
    }

    // 3. Facet interior & cutter-rim contact (non-vertical triangles only)
    let raw_n = (v1 - v0).cross(v2 - v0);
    let n_len = raw_n.length();
    if n_len > 1e-12 {
        let mut n = raw_n / n_len;
        if n.z < 0.0 {
            n = -n;
        }
        if n.z > 1e-9 {
            let plane_z =
                |p: DVec2| -> f64 { v0.z - (n.x * (p.x - v0.x) + n.y * (p.y - v0.y)) / n.z };
            let v0_2 = DVec2::new(v0.x, v0.y);
            let v1_2 = DVec2::new(v1.x, v1.y);
            let v2_2 = DVec2::new(v2.x, v2.y);

            // Always test axis center `(cx, cy)` (exact on horizontal triangles and flat center zones)
            if point_in_triangle_2d(c_xy, v0_2, v1_2, v2_2) {
                consider(plane_z(c_xy));
            }

            // Uphill XY direction `u = (-nx, -ny) / hypot(nx, ny)`
            let n_xy_len = n.x.hypot(n.y);
            if n_xy_len > 1e-11 {
                let u = DVec2::new(-n.x / n_xy_len, -n.y / n_xy_len);
                let slope = n_xy_len / n.z;

                match cutter.shape {
                    SurfaceCutterShape::Flat { radius_mm } => {
                        let p = c_xy + u * radius_mm;
                        if point_in_triangle_2d(p, v0_2, v1_2, v2_2) {
                            consider(plane_z(p));
                        }
                    }
                    SurfaceCutterShape::Ball { radius_mm } => {
                        let d_star = radius_mm * n_xy_len;
                        let p = c_xy + u * d_star;
                        if point_in_triangle_2d(p, v0_2, v1_2, v2_2)
                            && let Some(h) = cutter.profile_height_at_radius(d_star)
                        {
                            consider(plane_z(p) - h);
                        }
                    }
                    SurfaceCutterShape::BullNose {
                        flat_radius_mm,
                        corner_radius_mm,
                    } => {
                        let d_star = flat_radius_mm + corner_radius_mm * n_xy_len;
                        let p = c_xy + u * d_star;
                        if point_in_triangle_2d(p, v0_2, v1_2, v2_2)
                            && let Some(h) = cutter.profile_height_at_radius(d_star)
                        {
                            consider(plane_z(p) - h);
                        }
                    }
                    SurfaceCutterShape::TaperedVBit {
                        tip_radius_mm,
                        upper_radius_mm,
                        half_angle_rad,
                    } => {
                        let cone_slope = 1.0 / half_angle_rad.tan().max(1e-9);
                        let candidates: [f64; 2] = if slope < cone_slope {
                            [tip_radius_mm, tip_radius_mm]
                        } else if slope > cone_slope {
                            [upper_radius_mm, upper_radius_mm]
                        } else {
                            [tip_radius_mm, upper_radius_mm]
                        };
                        for d_c in candidates {
                            let p = c_xy + u * d_c;
                            if point_in_triangle_2d(p, v0_2, v1_2, v2_2)
                                && let Some(h) = cutter.profile_height_at_radius(d_c)
                            {
                                consider(plane_z(p) - h);
                            }
                        }
                    }
                }
            }
        }
    }

    best
}

/// Computes exact maximum of `p(t).z - h(||p_xy(t) - c_xy||)` over `t ∈ [0, 1]` where `d(t) <= r_max`.
fn drop_cutter_edge(
    cutter: &SurfaceCutterProfile,
    c_xy: DVec2,
    a: DVec3,
    b: DVec3,
    r_max: f64,
) -> Option<f64> {
    let a2 = DVec2::new(a.x, a.y) - c_xy;
    let d2 = DVec2::new(b.x - a.x, b.y - a.y);
    let dz = b.z - a.z;

    let len2_sq = d2.length_squared();
    if len2_sq <= 1e-18 {
        // Purely vertical edge: endpoints already evaluated, return highest point if inside disk
        let dist = a2.length();
        return cutter
            .profile_height_at_radius(dist)
            .map(|h| a.z.max(b.z) - h);
    }

    // Intersect `||a2 + t * d2||^2 <= r_max^2` with `t ∈ [0, 1]`
    let (t_lo, t_hi) = segment_disk_interval(a2, d2, len2_sq, r_max)?;

    let eval = |t: f64| -> Option<f64> {
        let dist = (a2 + d2 * t).length();
        let h = cutter.profile_height_at_radius(dist)?;
        Some(a.z + t * dz - h)
    };

    let mut best: Option<f64> = None;
    let mut consider_t = |t: f64| {
        let tc = t.clamp(t_lo, t_hi);
        if let Some(z) = eval(tc)
            && best.is_none_or(|b| z > b)
        {
            best = Some(z);
        }
    };

    consider_t(t_lo);
    consider_t(t_hi);

    let len2 = len2_sq.sqrt();
    let dir = d2 / len2;
    let s0 = a2.dot(dir);
    let u_perp = a2.x * dir.y - a2.y * dir.x;
    let m = dz / len2;

    match cutter.shape {
        SurfaceCutterShape::Flat { .. } => {
            // `f(t)` is linear on `[t_lo, t_hi]`; endpoints `t_lo`/`t_hi` are already exact!
        }
        SurfaceCutterShape::Ball { radius_mm } => {
            // Exact closed-form stationary point: `s* = (m / sqrt(1 + m^2)) * sqrt(R^2 - u_perp^2)`
            let r_eff_sq = (radius_mm * radius_mm - u_perp * u_perp).max(0.0);
            if r_eff_sq > 0.0 {
                let s_star = (m / (1.0 + m * m).sqrt()) * r_eff_sq.sqrt();
                let t_star = (s_star - s0) / len2;
                consider_t(t_star);
            }
        }
        SurfaceCutterShape::TaperedVBit {
            tip_radius_mm,
            half_angle_rad,
            ..
        } => {
            // Check tip-flat rim transitions `d(t) == tip_radius_mm`
            if let Some((f_lo, f_hi)) = segment_disk_interval(a2, d2, len2_sq, tip_radius_mm) {
                consider_t(f_lo);
                consider_t(f_hi);
            }
            // Exact closed-form cone stationary point: `k = m * tan(alpha)`, `|k| < 1`
            let k = m * half_angle_rad.tan();
            if k.abs() < 1.0 - 1e-12 {
                let s_star = u_perp.abs() * k / (1.0 - k * k).sqrt();
                let t_star = (s_star - s0) / len2;
                consider_t(t_star);
            }
        }
        SurfaceCutterShape::BullNose { flat_radius_mm, .. } => {
            // Check flat-zone transitions `d(t) == flat_radius_mm`
            if let Some((f_lo, f_hi)) = segment_disk_interval(a2, d2, len2_sq, flat_radius_mm) {
                consider_t(f_lo);
                consider_t(f_hi);
            }
            // `f(t)` is strictly concave (unimodal) on `[t_lo, t_hi]`: golden-section maximization
            // converges to `< 1e-10` in 48 iterations with guaranteed numerical stability.
            const INV_PHI: f64 = 0.618_033_988_749_894_9;
            let mut lo = t_lo;
            let mut hi = t_hi;
            let mut c = hi - INV_PHI * (hi - lo);
            let mut d = lo + INV_PHI * (hi - lo);
            let mut fc = eval(c).unwrap_or(f64::NEG_INFINITY);
            let mut fd = eval(d).unwrap_or(f64::NEG_INFINITY);
            for _ in 0..48 {
                if hi - lo <= 1e-12 {
                    break;
                }
                if fc < fd {
                    lo = c;
                    c = d;
                    fc = fd;
                    d = lo + INV_PHI * (hi - lo);
                    fd = eval(d).unwrap_or(f64::NEG_INFINITY);
                } else {
                    hi = d;
                    d = c;
                    fd = fc;
                    c = hi - INV_PHI * (hi - lo);
                    fc = eval(c).unwrap_or(f64::NEG_INFINITY);
                }
            }
            consider_t(0.5 * (lo + hi));
        }
    }

    best
}

/// Returns the sub-interval `[t_lo, t_hi] ⊆ [0, 1]` where `||a2 + t * d2|| <= radius`.
#[inline]
fn segment_disk_interval(a2: DVec2, d2: DVec2, len2_sq: f64, radius: f64) -> Option<(f64, f64)> {
    let b = a2.dot(d2);
    let c = a2.length_squared() - radius * radius;
    let disc = b * b - len2_sq * c;
    if disc < -1e-14 {
        return None;
    }
    let sqrt_d = disc.max(0.0).sqrt();
    let t0 = (-b - sqrt_d) / len2_sq;
    let t1 = (-b + sqrt_d) / len2_sq;
    let lo = t0.max(0.0);
    let hi = t1.min(1.0);
    if lo <= hi + 1e-12 {
        Some((lo.clamp(0.0, 1.0), hi.clamp(0.0, 1.0)))
    } else {
        None
    }
}

/// Exact `f64` 2D point-in-triangle test (orientation-independent, inclusive of edges within `1e-10`).
#[inline]
fn point_in_triangle_2d(p: DVec2, a: DVec2, b: DVec2, c: DVec2) -> bool {
    let area2 = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if area2.abs() <= 1e-14 {
        return false;
    }
    let inv = 1.0 / area2;
    let w0 = ((b.x - p.x) * (c.y - p.y) - (b.y - p.y) * (c.x - p.x)) * inv;
    let w1 = ((c.x - p.x) * (a.y - p.y) - (c.y - p.y) * (a.x - p.x)) * inv;
    let w2 = 1.0 - w0 - w1;
    w0 >= -1e-10 && w1 >= -1e-10 && w2 >= -1e-10
}
