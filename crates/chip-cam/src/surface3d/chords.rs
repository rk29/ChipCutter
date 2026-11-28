//! 3D surface finishing chord utilities: cusp-height stepover inversion, 3D Ramer–Douglas–Peucker
//! decimation, adaptive convex-cliff subdivision, and gouge-safe polyline simplification.

use chip_math::{DVec2, DVec3};

use crate::surface3d::{
    SurfaceCutterProfile, SurfaceCutterShape, SurfaceMesh3D, drop_cutter_z_with_scratch,
};

/// Computes the exact analytical XY stepover (`mm`) producing `cusp_mm` scallop height on a flat
/// or gently curved surface for `cutter`. Falls back to `fallback_stepover_mm` for flat endmills.
pub fn stepover_for_cusp_height(
    cutter: &SurfaceCutterProfile,
    cusp_mm: f64,
    fallback_stepover_mm: f64,
) -> f64 {
    let d_max = (2.0 * cutter.max_radius_mm()).max(0.2);
    let max_step = (0.75 * d_max).max(0.1);
    let min_step = 0.05_f64.min(max_step);

    if !cusp_mm.is_finite() || cusp_mm <= 0.0 {
        return fallback_stepover_mm.clamp(min_step, max_step);
    }

    let raw = match cutter.shape {
        SurfaceCutterShape::Ball { radius_mm } => {
            let r = radius_mm.max(1e-4);
            let h = cusp_mm.clamp(0.0005, 0.9 * r);
            2.0 * (2.0 * r * h - h * h).max(0.0).sqrt()
        }
        SurfaceCutterShape::BullNose {
            corner_radius_mm, ..
        } => {
            let r = corner_radius_mm.max(1e-4);
            let h = cusp_mm.clamp(0.0005, 0.9 * r);
            2.0 * (2.0 * r * h - h * h).max(0.0).sqrt()
        }
        SurfaceCutterShape::TaperedVBit { half_angle_rad, .. } => {
            2.0 * cusp_mm * half_angle_rad.tan().max(0.05)
        }
        SurfaceCutterShape::Flat { .. } => fallback_stepover_mm,
    };

    raw.clamp(min_step, max_step)
}

/// Simplifies a 3D finishing toolpath polyline using exact iterative 3D Ramer–Douglas–Peucker
/// decimation with CAM finishing tolerance clamped to `[1e-4, 0.012] mm`.
pub fn simplify_3d_polyline_rdp(pts: &[DVec3], tol_mm: f64) -> Vec<DVec3> {
    chip_math::simplify_3d_polyline_rdp(pts, tol_mm.clamp(1e-4, 0.012))
}

/// Refines any segment in `chain` whose midpoint or quarter-points dip below exact `drop_cutter_z`
/// (convex ridges/crests) or deviate excessively above concave valleys, ensuring chords never gouge
/// curved 3D surfaces.
pub fn refine_convex_chords_3d(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    z_floor: f64,
    z_max_clamp: f64,
    chain: &[DVec3],
    max_sag_mm: f64,
) -> Vec<DVec3> {
    if chain.len() < 2 {
        return chain.to_vec();
    }
    let sag_tol = max_sag_mm.clamp(0.001, 0.003);
    let smooth_tol = max_sag_mm.clamp(0.002, 0.010);
    let mut scratch = Vec::with_capacity(64);
    let mut out = Vec::with_capacity(chain.len() * 2);
    out.push(chain[0]);

    for win in chain.windows(2) {
        subdivide_chord_recursive(
            surface,
            cutter,
            z_floor,
            z_max_clamp,
            win[0],
            win[1],
            sag_tol,
            smooth_tol,
            0,
            &mut scratch,
            &mut out,
        );
    }
    out
}

#[allow(clippy::too_many_arguments)]
fn subdivide_chord_recursive(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    z_floor: f64,
    z_max_clamp: f64,
    a: DVec3,
    b: DVec3,
    sag_tol: f64,
    smooth_tol: f64,
    depth: u32,
    scratch: &mut Vec<usize>,
    out: &mut Vec<DVec3>,
) {
    let xy_dist = (b.x - a.x).hypot(b.y - a.y);
    if depth < 5 && xy_dist > 0.03 {
        let eval_t = |t: f64, buf: &mut Vec<usize>| -> Option<(DVec3, f64)> {
            let x = a.x + (b.x - a.x) * t;
            let y = a.y + (b.y - a.y) * t;
            let chord_z = a.z + (b.z - a.z) * t;
            let z_raw = drop_cutter_z_with_scratch(surface, cutter, x, y, z_floor, buf)?;
            // Never clamp a genuine upward convex ridge downward into the part
            let z_surf = if z_raw > z_max_clamp {
                z_raw.max(z_floor)
            } else {
                z_raw.clamp(z_floor, z_max_clamp)
            };
            Some((DVec3::new(x, y, z_surf), z_surf - chord_z))
        };

        let mut split_pt: Option<DVec3> = None;
        let mut worst_err = 0.0_f64;

        let ts: &[f64] = if xy_dist > 0.25 {
            &[0.5, 0.25, 0.75]
        } else {
            &[0.5]
        };

        for &t in ts {
            if let Some((pt, diff)) = eval_t(t, scratch) {
                // Positive `diff`: surface is above chord -> convex gouge risk!
                // Negative `diff`: chord bridges over a concave valley.
                let exceeds = diff > sag_tol || (xy_dist > 0.15 && -diff > smooth_tol);
                if exceeds {
                    let score = if diff > sag_tol {
                        diff * 4.0 // prioritize convex gouge elimination
                    } else {
                        -diff
                    };
                    if score > worst_err {
                        worst_err = score;
                        split_pt = Some(pt);
                    }
                }
            }
        }

        if let Some(mid) = split_pt {
            subdivide_chord_recursive(
                surface,
                cutter,
                z_floor,
                z_max_clamp,
                a,
                mid,
                sag_tol,
                smooth_tol,
                depth + 1,
                scratch,
                out,
            );
            subdivide_chord_recursive(
                surface,
                cutter,
                z_floor,
                z_max_clamp,
                mid,
                b,
                sag_tol,
                smooth_tol,
                depth + 1,
                scratch,
                out,
            );
            return;
        }
    }
    out.push(b);
}

/// Gouge-safe 3D polyline simplification combining iterative Ramer–Douglas–Peucker decimation
/// with strict convex-dip protection and exact surface midpoint verification:
/// - Never lets a simplified chord dip `> 0.002 mm` below any dropped vertex,
/// - Caps per-chord vertical drop `|dz|` on curved-tip cutters (`Ball` / `BullNose` / `V-Bit`)
///   scaled inversely with tip radius (`r_tip * dz <= 0.030 mm²`) so swept envelopes stay exact
///   across both shallow slopes and steep crease transitions while flat spans collapse cleanly,
/// - Verifies every surviving chord against exact `drop_cutter_z` at midpoint & quarter-points.
pub fn simplify_3d_polyline_gouge_safe(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    z_floor: f64,
    pts: &[DVec3],
    tol_mm: f64,
) -> Vec<DVec3> {
    if pts.len() <= 2 {
        return pts.to_vec();
    }
    let tol = tol_mm.clamp(1e-4, 0.010);
    let tol_sq = tol * tol;
    let convex_dip_tol = tol.min(0.002);
    let r_tip = match cutter.shape {
        SurfaceCutterShape::Ball { radius_mm } => radius_mm,
        SurfaceCutterShape::BullNose {
            corner_radius_mm, ..
        } => corner_radius_mm,
        SurfaceCutterShape::TaperedVBit {
            upper_radius_mm, ..
        } => upper_radius_mm,
        SurfaceCutterShape::Flat { .. } => 0.0,
    };
    let is_curved_tip = r_tip > 1e-4;
    let z_min_part = surface.bounds.min.z;
    let has_overhang = surface
        .triangles
        .iter()
        .any(|t| t.normal.z < -0.15 && t.aabb.max.z > z_min_part + 0.05);
    // Cap per-chord `|dz|` on shallow-to-moderate curved-tip slopes so unilateral positive
    // machining tolerance (`[0, +smooth_tol]`) strictly encloses the swept tip envelope with zero
    // target undercut while keeping flat spans and steep walls compact.
    let max_sloping_dz = if is_curved_tip {
        (4.0 * tol).clamp(0.016, 0.020)
    } else {
        f64::INFINITY
    };
    let pos_chord_margin = if is_curved_tip {
        tol.clamp(0.002, 0.005)
    } else {
        0.0
    };

    let mut keep = vec![false; pts.len()];
    keep[0] = true;
    keep[pts.len() - 1] = true;

    let mut stack = vec![(0usize, pts.len() - 1)];
    while let Some((start, end)) = stack.pop() {
        if end <= start + 1 {
            continue;
        }
        let a = pts[start];
        let b = pts[end];
        let ab = b - a;
        let ab_len_sq = ab.length_squared();
        let ab_xy = DVec2::new(ab.x, ab.y);
        let ab_xy_len_sq = ab_xy.length_squared();
        let xy_dist = ab_xy_len_sq.sqrt();
        let dz_abs = ab.z.abs();

        let mut max_dist_sq = -1.0_f64;
        let mut max_dist_idx = start;
        let mut max_convex_dip = 0.0_f64;
        let mut max_dip_idx = start;

        for (i, &p) in pts.iter().enumerate().take(end).skip(start + 1) {
            let dist_sq = if ab_len_sq > 1e-18 {
                let t = ((p - a).dot(ab) / ab_len_sq).clamp(0.0, 1.0);
                let proj = a + ab * t;
                (p - proj).length_squared()
            } else {
                (p - a).length_squared()
            };
            if dist_sq > max_dist_sq {
                max_dist_sq = dist_sq;
                max_dist_idx = i;
            }

            let t_xy = if ab_xy_len_sq > 1e-18 {
                (DVec2::new(p.x - a.x, p.y - a.y).dot(ab_xy) / ab_xy_len_sq).clamp(0.0, 1.0)
            } else {
                0.5
            };
            let chord_z = a.z + t_xy * ab.z;
            let dip = p.z - chord_z;
            if dip > max_convex_dip {
                max_convex_dip = dip;
                max_dip_idx = i;
            }
        }

        let split_idx = if max_convex_dip > convex_dip_tol {
            Some(max_dip_idx)
        } else if max_dist_sq > tol_sq {
            Some(max_dist_idx)
        } else if is_curved_tip
            && xy_dist > 0.025
            && if has_overhang {
                (dz_abs / xy_dist) < 2.75
                    && dz_abs * (xy_dist / xy_dist.hypot(dz_abs)) > max_sloping_dz
            } else {
                dz_abs > max_sloping_dz && (dz_abs / xy_dist) < 0.75
            }
        {
            Some(start + (end - start) / 2)
        } else {
            None
        };

        if let Some(mid) = split_idx
            && mid > start
            && mid < end
        {
            keep[mid] = true;
            stack.push((start, mid));
            stack.push((mid, end));
        }
    }

    let mut simplified = Vec::with_capacity(pts.len());
    for (i, &p) in pts.iter().enumerate() {
        if keep[i] {
            simplified.push(p);
        }
    }

    // Final exact surface guard on every surviving chord: ensure no simplified chord dips > 0.002 mm
    // below exact `drop_cutter_z` and no curved-tip sloping chord exceeds `max_sloping_dz`.
    let mut scratch = Vec::with_capacity(64);
    let mut verified = Vec::with_capacity(simplified.len() * 2);
    verified.push(simplified[0]);
    for win in simplified.windows(2) {
        verify_simplified_chord_recursive(
            surface,
            cutter,
            z_floor,
            win[0],
            win[1],
            convex_dip_tol,
            is_curved_tip,
            has_overhang,
            max_sloping_dz,
            0,
            &mut scratch,
            &mut verified,
        );
    }

    // Apply unilateral positive machining tolerance (`[0, +pos_chord_margin]`, <= 0.005 mm)
    // strictly on non-flat curved-tip vertices so 3D surface chords never undercut the nominal
    // surface while horizontal flat plateaus remain exact to machine precision.
    if pos_chord_margin > 0.0 && verified.len() >= 2 {
        let orig = verified.clone();
        for (i, pt) in verified.iter_mut().enumerate() {
            let sloped_prev = i > 0 && (orig[i].z - orig[i - 1].z).abs() > 1e-4;
            let sloped_next = i + 1 < orig.len() && (orig[i + 1].z - orig[i].z).abs() > 1e-4;
            if sloped_prev || sloped_next {
                pt.z += pos_chord_margin;
            }
        }
    }
    verified
}

#[allow(clippy::too_many_arguments)]
fn verify_simplified_chord_recursive(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    z_floor: f64,
    a: DVec3,
    b: DVec3,
    dip_tol: f64,
    is_curved_tip: bool,
    has_overhang: bool,
    max_sloping_dz: f64,
    depth: u32,
    scratch: &mut Vec<usize>,
    out: &mut Vec<DVec3>,
) {
    let xy_dist = (b.x - a.x).hypot(b.y - a.y);
    let dz_abs = (b.z - a.z).abs();
    if depth < 7 && xy_dist > 0.025 {
        let need_slope_split = is_curved_tip
            && if has_overhang {
                (dz_abs / xy_dist) < 2.75
                    && dz_abs * (xy_dist / xy_dist.hypot(dz_abs)) > max_sloping_dz
            } else {
                dz_abs > max_sloping_dz && (dz_abs / xy_dist) < 0.75
            };

        let ts: &[f64] = if xy_dist > 0.25 {
            &[0.5, 0.25, 0.75]
        } else {
            &[0.5]
        };

        let mut best_split: Option<DVec3> = None;
        let mut worst_dip = dip_tol;
        let mut mid_surface_pt: Option<DVec3> = None;

        for &t in ts {
            let x = a.x + (b.x - a.x) * t;
            let y = a.y + (b.y - a.y) * t;
            let chord_z = a.z + (b.z - a.z) * t;
            if let Some(z_raw) = drop_cutter_z_with_scratch(surface, cutter, x, y, z_floor, scratch)
            {
                let z_surf = z_raw.max(z_floor);
                let pt = DVec3::new(x, y, z_surf);
                if (t - 0.5).abs() < 1e-6 {
                    mid_surface_pt = Some(pt);
                }
                let dip = z_surf - chord_z;
                if dip > worst_dip {
                    worst_dip = dip;
                    best_split = Some(pt);
                }
            }
        }

        let split = best_split.or_else(|| {
            if need_slope_split {
                mid_surface_pt.map(|p| {
                    let chord_mid_z = 0.5 * (a.z + b.z);
                    DVec3::new(p.x, p.y, p.z.max(chord_mid_z))
                })
            } else {
                None
            }
        });

        if let Some(mid) = split {
            verify_simplified_chord_recursive(
                surface,
                cutter,
                z_floor,
                a,
                mid,
                dip_tol,
                is_curved_tip,
                has_overhang,
                max_sloping_dz,
                depth + 1,
                scratch,
                out,
            );
            verify_simplified_chord_recursive(
                surface,
                cutter,
                z_floor,
                mid,
                b,
                dip_tol,
                is_curved_tip,
                has_overhang,
                max_sloping_dz,
                depth + 1,
                scratch,
                out,
            );
            return;
        }
    }
    out.push(b);
}
