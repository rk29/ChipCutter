use chip_math::{DVec2, DVec3, point_in_triangle_2d};
use rayon::prelude::*;

use crate::{
    fixture_avoidance::ToolAssemblyPart,
    rest2d::RegionSet2D,
    surface3d::{
        SurfaceMesh3D,
        cutter::{SurfaceCutterProfile, SurfaceCutterShape},
    },
};

/// Detailed analytical Drop-Cutter contact result separating flute-only contact from full-assembly
/// shank / collet-holder clearance lift.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DropCutterDetail {
    /// Final required tip Z (`mm` in Setup WCS, clamped to `>= z_floor`).
    pub z_tip_wcs: f64,
    /// Flute-only tip Z (`None` if flutes did not overlap any triangle).
    pub z_flute_wcs: Option<f64>,
    /// Assembly component that determined `z_tip_wcs` (`Flute`, `Shank`, or `Holder`).
    pub limiting_part: ToolAssemblyPart,
    /// Vertical lift (`mm >= 0`) added above flute-only contact (or `z_floor` if flutes missed)
    /// caused by wider shank or collet/holder clearance.
    pub assembly_lift_mm: f64,
}

/// Evaluates the exact analytical `f64` Drop-Cutter contact Z height (`z_tip` in Setup WCS mm)
/// at horizontal cutter axis position `(x, y)` above `surface`, floored by `z_floor`.
///
/// When `cutter.assembly` is populated ([`SurfaceCutterProfile::from_tool`]), also enforces exact
/// analytical clearance across wider non-cutting shanks (`dz >= flute_length_mm`), collet nuts
/// (`dz >= stickout_length_mm`), and tapered holder cones (`holder_cone_z0..z1`).
///
/// Returns `None` if no triangle in `surface` overlaps the active cutter/assembly footprint above
/// `z_floor` (or if `x`/`y`/`z_floor` are non-finite).
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
    drop_cutter_z_detailed_with_scratch(surface, cutter, x, y, z_floor, candidate_buf)
        .map(|d| d.z_tip_wcs)
}

/// Zero-allocation inner Drop-Cutter returning full [`DropCutterDetail`] (including whether wider
/// shank or holder clearance lifted `z_tip_wcs` above flute-only contact).
pub fn drop_cutter_z_detailed_with_scratch(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    x: f64,
    y: f64,
    z_floor: f64,
    candidate_buf: &mut Vec<usize>,
) -> Option<DropCutterDetail> {
    if !x.is_finite() || !y.is_finite() || !z_floor.is_finite() {
        return None;
    }
    let r_flute = cutter.max_radius_mm();
    let eval_assembly = cutter.assembly_can_overlap(surface.bounds.max.z, z_floor);

    if !eval_assembly {
        // Fast single-disk path: zero overhead on normal parts and bit-exact parity with flute-only.
        surface
            .bvh
            .query_xy_circle_into(x, y, r_flute + 1e-6, candidate_buf);
        if candidate_buf.is_empty() {
            return None;
        }

        let c_xy = DVec2::new(x, y);
        let mut best_z = f64::NEG_INFINITY;
        let mut hit_any = false;

        for &ti in candidate_buf.iter() {
            let tri = &surface.triangles[ti];
            if hit_any && tri.aabb.max.z <= best_z {
                continue;
            }
            let [i0, i1, i2] = tri.indices;
            let v0 = surface.vertices[i0 as usize];
            let v1 = surface.vertices[i1 as usize];
            let v2 = surface.vertices[i2 as usize];

            if let Some(z_tri) = drop_cutter_triangle_shape(&cutter.shape, c_xy, v0, v1, v2)
                && z_tri.is_finite()
                && (!hit_any || z_tri > best_z)
            {
                best_z = z_tri;
                hit_any = true;
            }
        }

        return if hit_any {
            let z_final = (best_z + cutter.stock_axial_mm).max(z_floor);
            Some(DropCutterDetail {
                z_tip_wcs: z_final,
                z_flute_wcs: Some(z_final),
                limiting_part: ToolAssemblyPart::Flute,
                assembly_lift_mm: 0.0,
            })
        } else {
            None
        };
    }

    // Full-assembly multi-tier path (`Flute` + wider `Shank` + `Collet Nut` + `Holder Cone`)
    let asm = cutter.assembly?;

    let r_shank_eff = asm.effective_shank_radius_mm();
    let check_shank = r_shank_eff > r_flute + 1e-6
        && surface.bounds.max.z + cutter.stock_axial_mm > z_floor + asm.flute_length_mm - 1e-4;

    let r_nut_eff = asm.effective_holder_nut_radius_mm();
    let check_nut = r_nut_eff > r_flute.max(r_shank_eff) + 1e-6
        && surface.bounds.max.z + asm.holder_clearance_mm > z_floor + asm.stickout_length_mm - 1e-4;

    let r_body_eff = asm.effective_holder_body_radius_mm();
    let cone_dz = (asm.holder_cone_z1 - asm.holder_cone_z0).max(1e-6);
    let check_cone = r_body_eff > r_nut_eff + 1e-6
        && surface.bounds.max.z + asm.holder_clearance_mm > z_floor + asm.holder_cone_z0 - 1e-4;

    let r_query = cutter.effective_query_radius_mm(surface.bounds.max.z, z_floor);
    surface
        .bvh
        .query_xy_circle_into(x, y, r_query + 1e-6, candidate_buf);
    if candidate_buf.is_empty() {
        return None;
    }

    let shank_shape = SurfaceCutterShape::Flat {
        radius_mm: r_shank_eff,
    };
    let nut_shape = SurfaceCutterShape::Flat {
        radius_mm: r_nut_eff,
    };
    let cone_shape = SurfaceCutterShape::TaperedVBit {
        tip_radius_mm: r_nut_eff,
        upper_radius_mm: r_body_eff,
        half_angle_rad: ((r_body_eff - r_nut_eff) / cone_dz).max(1e-6).atan(),
    };

    let c_xy = DVec2::new(x, y);
    let mut best_flute_raw = f64::NEG_INFINITY;
    let mut hit_flute = false;

    let mut best_asm_z = z_floor - 1e-6;
    let mut hit_asm = false;
    let mut best_asm_part = ToolAssemblyPart::Flute;

    let flute_xy_pad = r_flute + 1e-6;

    for &ti in candidate_buf.iter() {
        let tri = &surface.triangles[ti];
        let [i0, i1, i2] = tri.indices;
        let v0 = surface.vertices[i0 as usize];
        let v1 = surface.vertices[i1 as usize];
        let v2 = surface.vertices[i2 as usize];

        // 1. Flute contact (only when triangle XY AABB overlaps flute disk)
        let tri_overlaps_flute = tri.aabb.max.x >= x - flute_xy_pad
            && tri.aabb.min.x <= x + flute_xy_pad
            && tri.aabb.max.y >= y - flute_xy_pad
            && tri.aabb.min.y <= y + flute_xy_pad;
        if tri_overlaps_flute
            && (!hit_flute || tri.aabb.max.z > best_flute_raw)
            && let Some(z_tri) = drop_cutter_triangle_shape(&cutter.shape, c_xy, v0, v1, v2)
            && z_tri.is_finite()
            && (!hit_flute || z_tri > best_flute_raw)
        {
            best_flute_raw = z_tri;
            hit_flute = true;
        }

        let current_floor = if hit_flute {
            (best_flute_raw + cutter.stock_axial_mm).max(best_asm_z)
        } else {
            best_asm_z
        };

        // 2. Wider non-cutting shank tier (`dz = flute_length_mm`, `r = shank_r + stock_radial`)
        if check_shank
            && tri.aabb.max.z + cutter.stock_axial_mm - asm.flute_length_mm > current_floor
        {
            let tri_overlaps_shank = tri.aabb.max.x >= x - r_shank_eff - 1e-6
                && tri.aabb.min.x <= x + r_shank_eff + 1e-6
                && tri.aabb.max.y >= y - r_shank_eff - 1e-6
                && tri.aabb.min.y <= y + r_shank_eff + 1e-6;
            if tri_overlaps_shank
                && let Some(z_contact) = drop_cutter_triangle_shape(&shank_shape, c_xy, v0, v1, v2)
            {
                let z_tip = z_contact + cutter.stock_axial_mm - asm.flute_length_mm;
                if z_tip.is_finite() && z_tip > best_asm_z {
                    best_asm_z = z_tip;
                    hit_asm = true;
                    best_asm_part = ToolAssemblyPart::Shank;
                }
            }
        }

        let current_floor = if hit_flute {
            (best_flute_raw + cutter.stock_axial_mm).max(best_asm_z)
        } else {
            best_asm_z
        };

        // 3. Collet nut cylinder tier (`dz = stickout_length_mm`, `r = nut_r + holder_clearance`)
        if check_nut
            && tri.aabb.max.z + asm.holder_clearance_mm - asm.stickout_length_mm > current_floor
        {
            let tri_overlaps_nut = tri.aabb.max.x >= x - r_nut_eff - 1e-6
                && tri.aabb.min.x <= x + r_nut_eff + 1e-6
                && tri.aabb.max.y >= y - r_nut_eff - 1e-6
                && tri.aabb.min.y <= y + r_nut_eff + 1e-6;
            if tri_overlaps_nut
                && let Some(z_contact) = drop_cutter_triangle_shape(&nut_shape, c_xy, v0, v1, v2)
            {
                let z_tip = z_contact + asm.holder_clearance_mm - asm.stickout_length_mm;
                if z_tip.is_finite() && z_tip > best_asm_z {
                    best_asm_z = z_tip;
                    hit_asm = true;
                    best_asm_part = ToolAssemblyPart::Holder;
                }
            }
        }

        let current_floor = if hit_flute {
            (best_flute_raw + cutter.stock_axial_mm).max(best_asm_z)
        } else {
            best_asm_z
        };

        // 4. Tapered holder cone tier (`holder_cone_z0..holder_cone_z1`, `r_nut -> r_body`)
        if check_cone
            && tri.aabb.max.z + asm.holder_clearance_mm - asm.holder_cone_z0 > current_floor
            && let Some(z_contact) = drop_cutter_triangle_shape(&cone_shape, c_xy, v0, v1, v2)
        {
            let z_tip = z_contact + asm.holder_clearance_mm - asm.holder_cone_z0;
            if z_tip.is_finite() && z_tip > best_asm_z {
                best_asm_z = z_tip;
                hit_asm = true;
                best_asm_part = ToolAssemblyPart::Holder;
            }
        }
    }

    if !hit_flute && !hit_asm {
        return None;
    }

    let z_flute_clamped = hit_flute.then(|| (best_flute_raw + cutter.stock_axial_mm).max(z_floor));
    let base_z = z_flute_clamped.unwrap_or(z_floor);

    if hit_asm && best_asm_z > base_z + 1e-7 {
        let z_final = best_asm_z.max(z_floor);
        Some(DropCutterDetail {
            z_tip_wcs: z_final,
            z_flute_wcs: z_flute_clamped,
            limiting_part: best_asm_part,
            assembly_lift_mm: (z_final - base_z).max(0.0),
        })
    } else if let Some(zf) = z_flute_clamped {
        Some(DropCutterDetail {
            z_tip_wcs: zf,
            z_flute_wcs: Some(zf),
            limiting_part: ToolAssemblyPart::Flute,
            assembly_lift_mm: 0.0,
        })
    } else {
        let z_final = best_asm_z.max(z_floor);
        Some(DropCutterDetail {
            z_tip_wcs: z_final,
            z_flute_wcs: None,
            limiting_part: best_asm_part,
            assembly_lift_mm: (z_final - z_floor).max(0.0),
        })
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

/// Evaluates the highest analytical Drop-Cutter contact `(z_tip_raw, contact_pt_3d, tri_idx)`
/// at horizontal cutter axis `(c_xy.x, c_xy.y)`.
pub(crate) fn drop_cutter_highest_contact(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    c_xy: DVec2,
    cand_buf: &mut Vec<usize>,
) -> Option<(f64, DVec3, usize)> {
    let r_max = cutter.max_radius_mm();
    surface
        .bvh
        .query_xy_circle_into(c_xy.x, c_xy.y, r_max + 1e-6, cand_buf);
    if cand_buf.is_empty() {
        return None;
    }

    let mut best: Option<(f64, DVec3, usize)> = None;
    for &ti in cand_buf.iter() {
        let tri = &surface.triangles[ti];
        if let Some((best_z, _, _)) = best
            && tri.aabb.max.z <= best_z
        {
            continue;
        }
        let [i0, i1, i2] = tri.indices;
        let v0 = surface.vertices[i0 as usize];
        let v1 = surface.vertices[i1 as usize];
        let v2 = surface.vertices[i2 as usize];
        if let Some((z_tri, c_pt)) =
            drop_cutter_triangle_contact_shape(&cutter.shape, c_xy, v0, v1, v2)
            && z_tri.is_finite()
            && best.is_none_or(|(bz, _, _)| z_tri > bz)
        {
            best = Some((z_tri, c_pt, ti));
        }
    }
    best
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
    drop_cutter_triangle_shape(&cutter.shape, c_xy, v0, v1, v2)
}

#[inline]
pub(crate) fn drop_cutter_triangle_shape(
    shape: &SurfaceCutterShape,
    c_xy: DVec2,
    v0: DVec3,
    v1: DVec3,
    v2: DVec3,
) -> Option<f64> {
    drop_cutter_triangle_contact_shape(shape, c_xy, v0, v1, v2).map(|(z, _)| z)
}

pub(crate) fn drop_cutter_triangle_contact_shape(
    shape: &SurfaceCutterShape,
    c_xy: DVec2,
    v0: DVec3,
    v1: DVec3,
    v2: DVec3,
) -> Option<(f64, DVec3)> {
    let r_max = shape.max_radius_mm();
    let mut best: Option<(f64, DVec3)> = None;
    let mut consider = |z: f64, pt: DVec3| {
        if z.is_finite() && best.is_none_or(|(bz, _)| z > bz) {
            best = Some((z, pt));
        }
    };

    // 1. Vertex contacts
    for v in [v0, v1, v2] {
        let d = (DVec2::new(v.x, v.y) - c_xy).length();
        if let Some(h) = shape.profile_height_at_radius(d) {
            consider(v.z - h, v);
        }
    }

    // 2. Edge contacts (exact closed-form for Flat/Ball/VBit; safeguarded unimodal refinement for BullNose)
    for (a, b) in [(v0, v1), (v1, v2), (v2, v0)] {
        if let Some((z_e, pt_e)) = drop_cutter_edge_contact_shape(shape, c_xy, a, b, r_max) {
            consider(z_e, pt_e);
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
                let zp = plane_z(c_xy);
                consider(zp, DVec3::new(c_xy.x, c_xy.y, zp));
            }

            // Uphill XY direction `u = (-nx, -ny) / hypot(nx, ny)`
            let n_xy_len = n.x.hypot(n.y);
            if n_xy_len > 1e-11 {
                let u = DVec2::new(-n.x / n_xy_len, -n.y / n_xy_len);
                let slope = n_xy_len / n.z;

                match *shape {
                    SurfaceCutterShape::Flat { radius_mm } => {
                        let p = c_xy + u * radius_mm;
                        if point_in_triangle_2d(p, v0_2, v1_2, v2_2) {
                            let zp = plane_z(p);
                            consider(zp, DVec3::new(p.x, p.y, zp));
                        }
                    }
                    SurfaceCutterShape::Ball { radius_mm } => {
                        let d_star = radius_mm * n_xy_len;
                        let p = c_xy + u * d_star;
                        if point_in_triangle_2d(p, v0_2, v1_2, v2_2)
                            && let Some(h) = shape.profile_height_at_radius(d_star)
                        {
                            let zp = plane_z(p);
                            consider(zp - h, DVec3::new(p.x, p.y, zp));
                        }
                    }
                    SurfaceCutterShape::BullNose {
                        flat_radius_mm,
                        corner_radius_mm,
                    } => {
                        let d_star = flat_radius_mm + corner_radius_mm * n_xy_len;
                        let p = c_xy + u * d_star;
                        if point_in_triangle_2d(p, v0_2, v1_2, v2_2)
                            && let Some(h) = shape.profile_height_at_radius(d_star)
                        {
                            let zp = plane_z(p);
                            consider(zp - h, DVec3::new(p.x, p.y, zp));
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
                                && let Some(h) = shape.profile_height_at_radius(d_c)
                            {
                                let zp = plane_z(p);
                                consider(zp - h, DVec3::new(p.x, p.y, zp));
                            }
                        }
                    }
                }
            }
        }
    }

    best
}

/// Computes exact maximum `(z_tip, contact_pt)` of `p(t).z - h(||p_xy(t) - c_xy||)` over
/// `t ∈ [0, 1]` where `d(t) <= r_max`.
fn drop_cutter_edge_contact_shape(
    shape: &SurfaceCutterShape,
    c_xy: DVec2,
    a: DVec3,
    b: DVec3,
    r_max: f64,
) -> Option<(f64, DVec3)> {
    let a2 = DVec2::new(a.x, a.y) - c_xy;
    let d2 = DVec2::new(b.x - a.x, b.y - a.y);
    let dz = b.z - a.z;

    let len2_sq = d2.length_squared();
    if len2_sq <= 1e-18 {
        // Purely vertical edge: endpoints already evaluated, return highest point if inside disk
        let dist = a2.length();
        let top = if a.z >= b.z { a } else { b };
        return shape
            .profile_height_at_radius(dist)
            .map(|h| (top.z - h, top));
    }

    // Intersect `||a2 + t * d2||^2 <= r_max^2` with `t ∈ [0, 1]`
    let (t_lo, t_hi) = segment_disk_interval(a2, d2, len2_sq, r_max)?;

    let eval = |t: f64| -> Option<(f64, DVec3)> {
        let dist = (a2 + d2 * t).length();
        let h = shape.profile_height_at_radius(dist)?;
        let pt = a + (b - a) * t;
        Some((pt.z - h, pt))
    };

    let mut best: Option<(f64, DVec3)> = None;
    let mut consider_t = |t: f64| {
        let tc = t.clamp(t_lo, t_hi);
        if let Some((z, pt)) = eval(tc)
            && best.is_none_or(|(bz, _)| z > bz)
        {
            best = Some((z, pt));
        }
    };

    consider_t(t_lo);
    consider_t(t_hi);

    let len2 = len2_sq.sqrt();
    let dir = d2 / len2;
    let s0 = a2.dot(dir);
    let u_perp = a2.x * dir.y - a2.y * dir.x;
    let m = dz / len2;

    match *shape {
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
            let mut fc = eval(c).map_or(f64::NEG_INFINITY, |(z, _)| z);
            let mut fd = eval(d).map_or(f64::NEG_INFINITY, |(z, _)| z);
            for _ in 0..48 {
                if hi - lo <= 1e-12 {
                    break;
                }
                if fc < fd {
                    lo = c;
                    c = d;
                    fc = fd;
                    d = lo + INV_PHI * (hi - lo);
                    fd = eval(d).map_or(f64::NEG_INFINITY, |(z, _)| z);
                } else {
                    hi = d;
                    d = c;
                    fd = fc;
                    c = hi - INV_PHI * (hi - lo);
                    fc = eval(c).map_or(f64::NEG_INFINITY, |(z, _)| z);
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
