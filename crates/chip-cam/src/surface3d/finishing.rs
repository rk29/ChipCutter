//! Shared 3D surface finishing utilities: cusp-height stepover inversion, 3D Ramer–Douglas–Peucker
//! chord decimation, adaptive convex-cliff subdivision, analytical 3D Rest Masking, and
//! surface-safe collision-free 3D linking.

use chip_math::{Aabb2, DVec2, DVec3, point_in_triangle_2d};
use rayon::prelude::*;

use crate::{
    cl_data::MotionRole,
    params::{RestMachiningConfig, RestMachiningMode},
    rest2d::{RegionSet2D, UpstreamSweptInput},
    surface3d::{
        SurfaceCutterProfile, SurfaceCutterShape, SurfaceMesh3D,
        drop_cutter::{drop_cutter_highest_contact, drop_cutter_triangle_contact_shape},
        drop_cutter_points_parallel, drop_cutter_z_with_scratch,
    },
    toolpath::ToolpathBuilder,
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

/// 2D boolean mask grid indicating XY columns where uncut 3D rest material remains above the
/// finished surface for `current_cutter`.
#[derive(Debug, Clone)]
pub struct RestMaskGrid2D {
    pub min_x: f64,
    pub min_y: f64,
    pub cell_mm: f64,
    pub nx: usize,
    pub ny: usize,
    pub mask: Vec<bool>,
}

impl RestMaskGrid2D {
    /// Returns `true` when `(pt.x, pt.y)` lies inside the active 3D rest region.
    #[inline]
    pub fn contains_xy(&self, pt: DVec2) -> bool {
        if self.nx == 0 || self.ny == 0 {
            return false;
        }
        let fx = (pt.x - self.min_x) / self.cell_mm;
        let fy = (pt.y - self.min_y) / self.cell_mm;
        if fx < -0.5 || fy < -0.5 {
            return false;
        }
        let ix = fx.round() as isize;
        let iy = fy.round() as isize;
        if ix < 0 || iy < 0 || ix >= self.nx as isize || iy >= self.ny as isize {
            return false;
        }
        self.mask[iy as usize * self.nx + ix as usize]
    }
}

/// Evaluates the exact morphological 3D residual stock height (`mm`) left behind by a larger
/// reference ball cutter (`ref_radius_mm`) that `current_cutter` can remove, using exact ball-in-
/// ball Minkowski erosion of `z_ref_center` by `dR = ref_radius_mm - cur_radius_mm`:
///
/// - On flat plateaus and **convex** steep walls: `residual == 0.0 mm` (since `ref_ball` sweeps
///   tangentially across every convex point).
/// - Inside **concave** valleys, creases, and fillets where `ref_ball` cannot fit: `residual > 0`
///   peaking directly on the exact bi-tangent crease!
#[allow(clippy::too_many_arguments)]
pub fn compute_morphological_rest_residual(
    z_ref: &[Option<f64>],
    z_cur: &[Option<f64>],
    ref_radius_mm: f64,
    cur_radius_mm: f64,
    cell_mm: f64,
    nx: usize,
    ny: usize,
) -> Vec<f64> {
    let total = nx * ny;
    let d_r = (ref_radius_mm - cur_radius_mm).max(0.0);
    if d_r <= 1e-4 {
        return (0..total)
            .map(|i| match (z_ref[i], z_cur[i]) {
                (Some(zr), Some(zc)) => (zr - zc).max(0.0),
                _ => 0.0,
            })
            .collect();
    }

    // Precompute circular erosion kernel `(dx, dy, sqrt(d_r^2 - r^2))`
    let rad_cells = (d_r / cell_mm).ceil() as isize;
    let mut kernel: Vec<(isize, isize, f64)> = Vec::new();
    for dy in -rad_cells..=rad_cells {
        for dx in -rad_cells..=rad_cells {
            let dist_sq = (dx as f64 * cell_mm).powi(2) + (dy as f64 * cell_mm).powi(2);
            if dist_sq <= d_r * d_r + 1e-9 {
                let dz_sphere = (d_r * d_r - dist_sq).max(0.0).sqrt();
                kernel.push((dx, dy, dz_sphere));
            }
        }
    }

    (0..ny)
        .into_par_iter()
        .flat_map_iter(|iy| {
            let mut row = vec![0.0_f64; nx];
            for (ix, cell_val) in row.iter_mut().enumerate() {
                let idx = iy * nx + ix;
                let Some(zc) = z_cur[idx] else {
                    continue;
                };
                let z_cur_center = zc + cur_radius_mm;
                let mut min_env_center = f64::INFINITY;
                for &(dx, dy, dz_sphere) in &kernel {
                    let nix = ix as isize + dx;
                    let niy = iy as isize + dy;
                    if nix < 0 || niy < 0 || nix >= nx as isize || niy >= ny as isize {
                        continue;
                    }
                    if let Some(zr) = z_ref[niy as usize * nx + nix as usize] {
                        let cand = (zr + ref_radius_mm) - dz_sphere;
                        if cand < min_env_center {
                            min_env_center = cand;
                        }
                    }
                }
                // Steep-wall rim anti-aliasing: on near-vertical convex surfaces (e.g. dome equators),
                // the exact morphological tangent point lies near `r ≈ 0.85..0.99 * d_r` along the
                // steepest downhill `z_ref` direction, where integer grid steps can miss the exact rim.
                // Evaluating 4 bilinearly-interpolated samples along the downhill direction eliminates
                // spurious convex-wall residual artifacts.
                if min_env_center.is_finite() && min_env_center > z_cur_center + 0.01 {
                    let gx = match (
                        ix.checked_sub(1).and_then(|x| z_ref[iy * nx + x]),
                        (ix + 1 < nx).then(|| z_ref[iy * nx + ix + 1]).flatten(),
                    ) {
                        (Some(zl), Some(zr)) => (zr - zl) / (2.0 * cell_mm),
                        _ => 0.0,
                    };
                    let gy = match (
                        iy.checked_sub(1).and_then(|y| z_ref[y * nx + ix]),
                        (iy + 1 < ny).then(|| z_ref[(iy + 1) * nx + ix]).flatten(),
                    ) {
                        (Some(zd), Some(zu)) => (zu - zd) / (2.0 * cell_mm),
                        _ => 0.0,
                    };
                    let g_len = gx.hypot(gy);
                    if g_len > 0.25 {
                        let ux = -gx / g_len;
                        let uy = -gy / g_len;
                        for &frac in &[0.82_f64, 0.90, 0.95, 0.99] {
                            let r_off = frac * d_r;
                            let fx = ix as f64 + (ux * r_off) / cell_mm;
                            let fy = iy as f64 + (uy * r_off) / cell_mm;
                            if fx >= 0.0
                                && fy >= 0.0
                                && fx <= (nx - 1) as f64
                                && fy <= (ny - 1) as f64
                            {
                                let x0 = (fx.floor() as usize).min(nx.saturating_sub(2));
                                let y0 = (fy.floor() as usize).min(ny.saturating_sub(2));
                                let tx = (fx - x0 as f64).clamp(0.0, 1.0);
                                let ty = (fy - y0 as f64).clamp(0.0, 1.0);
                                if let (Some(a), Some(b), Some(c), Some(d)) = (
                                    z_ref[y0 * nx + x0],
                                    z_ref[y0 * nx + x0 + 1],
                                    z_ref[(y0 + 1) * nx + x0],
                                    z_ref[(y0 + 1) * nx + x0 + 1],
                                ) {
                                    let ab = a + (b - a) * tx;
                                    let cd = c + (d - c) * tx;
                                    let zr_interp = ab + (cd - ab) * ty;
                                    let dz_sphere = (d_r * d_r - r_off * r_off).max(0.0).sqrt();
                                    let cand = (zr_interp + ref_radius_mm) - dz_sphere;
                                    if cand < min_env_center {
                                        min_env_center = cand;
                                    }
                                }
                            }
                        }
                    }
                }
                if min_env_center.is_finite() {
                    *cell_val = (min_env_center - z_cur_center).max(0.0);
                }
            }
            row
        })
        .collect()
}

/// Evaluates an analytical 3D Rest Mask grid by comparing exact morphological residual stock left
/// by a larger reference cutter against `current_cutter` over `containment`.
///
/// Returns `None` when Rest Machining is disabled or no larger upstream cutter applies.
#[allow(clippy::too_many_arguments)]
pub fn build_3d_rest_mask(
    surface: &SurfaceMesh3D,
    current_cutter: &SurfaceCutterProfile,
    containment: &RegionSet2D,
    rest_cfg: &RestMachiningConfig,
    upstream_rest: &[UpstreamSweptInput],
    z_floor: f64,
) -> Option<RestMaskGrid2D> {
    if rest_cfg.is_disabled() || containment.is_empty() {
        return None;
    }

    let cur_d = 2.0 * current_cutter.max_radius_mm();
    let (prev_d, prev_stock) = match &rest_cfg.mode {
        RestMachiningMode::Disabled => return None,
        RestMachiningMode::FromPreviousTool {
            prev_tool_diameter,
            prev_stock_to_leave,
        } => (
            prev_tool_diameter.value_mm.max(0.0),
            prev_stock_to_leave.value_mm.max(0.0),
        ),
        RestMachiningMode::FromPrecedingOperations { source_op_ids } => {
            let max_up_d = upstream_rest
                .iter()
                .filter(|u| {
                    source_op_ids.is_empty() || source_op_ids.iter().any(|id| id == &u.op_id)
                })
                .map(|u| u.tool_diameter_mm)
                .fold(0.0_f64, f64::max);
            if max_up_d <= cur_d + 0.05 {
                return None;
            }
            (max_up_d, 0.0)
        }
    };

    if prev_d <= cur_d + 0.05 && prev_stock <= 1e-4 {
        return None;
    }

    let bounds = region_aabb_2d(containment)?;
    let w = (bounds.max.x - bounds.min.x).max(1.0);
    let h = (bounds.max.y - bounds.min.y).max(1.0);

    // Choose grid resolution balanced between accuracy and speed (max ~35k samples)
    let base_res = (0.25 * cur_d).clamp(0.35, 1.0);
    let max_cells = 36_000.0_f64;
    let res = base_res.max(((w * h) / max_cells).sqrt());
    let ref_d_eff = prev_d.max(cur_d + 0.1);
    let pad = 0.5 * (ref_d_eff - cur_d).max(0.0) + res;
    let min_x = bounds.min.x - pad;
    let min_y = bounds.min.y - pad;
    let nx = (((w + 2.0 * pad) / res).ceil() as usize).max(2) + 1;
    let ny = (((h + 2.0 * pad) / res).ceil() as usize).max(2) + 1;

    let ref_ball_geom = chip_tools::CutterGeometry::BallEndmill {
        diameter_mm: ref_d_eff,
        flute_length_mm: current_cutter.flute_length_mm.max(ref_d_eff * 2.0),
        overall_length_mm: current_cutter.flute_length_mm.max(ref_d_eff * 2.0) + 20.0,
        shank_diameter_mm: ref_d_eff,
        flutes: 2,
    };
    let ref_cutter =
        SurfaceCutterProfile::from_tool_geometry(&ref_ball_geom, prev_stock, prev_stock);

    let mut coords = Vec::with_capacity(nx * ny);
    for iy in 0..ny {
        let y = min_y + iy as f64 * res;
        for ix in 0..nx {
            let x = min_x + ix as f64 * res;
            coords.push(DVec2::new(x, y));
        }
    }

    // Evaluate `z_ref` across padded grid (unclipped) so morphological erosion near boundary stays exact
    let z_ref = drop_cutter_points_parallel(surface, &ref_cutter, &coords, z_floor, None);
    let z_cur =
        drop_cutter_points_parallel(surface, current_cutter, &coords, z_floor, Some(containment));

    let mut residual = compute_morphological_rest_residual(
        &z_ref,
        &z_cur,
        0.5 * ref_d_eff + prev_stock,
        current_cutter.max_radius_mm(),
        res,
        nx,
        ny,
    );
    suppress_overhang_shadow_residuals(
        surface,
        current_cutter,
        0.5 * ref_d_eff + prev_stock,
        &z_cur,
        &mut residual,
        min_x,
        min_y,
        res,
        nx,
        ny,
    );

    let threshold = rest_cfg.min_rest_thickness.value_mm.clamp(0.01, 5.0);
    let mut raw_mask = vec![false; nx * ny];
    for idx in 0..(nx * ny) {
        if residual[idx] > threshold {
            raw_mask[idx] = true;
        }
    }

    // Dilate active rest mask by `rest_overlap + 0.25 * cur_r` for smooth lead-in/out overlap
    let overlap_mm =
        rest_cfg.rest_overlap.value_mm.max(0.0) + 0.25 * current_cutter.max_radius_mm();
    let dilate_cells = (overlap_mm / res).ceil() as isize;
    let mask = if dilate_cells > 0 {
        let r2 = dilate_cells * dilate_cells;
        (0..ny)
            .into_par_iter()
            .flat_map_iter(|iy| {
                let mut row = vec![false; nx];
                for (ix, cell_flag) in row.iter_mut().enumerate() {
                    'search: for dy in -dilate_cells..=dilate_cells {
                        let ny_i = iy as isize + dy;
                        if ny_i < 0 || ny_i >= ny as isize {
                            continue;
                        }
                        for dx in -dilate_cells..=dilate_cells {
                            if dx * dx + dy * dy > r2 {
                                continue;
                            }
                            let nx_i = ix as isize + dx;
                            if nx_i < 0 || nx_i >= nx as isize {
                                continue;
                            }
                            if raw_mask[ny_i as usize * nx + nx_i as usize] {
                                *cell_flag = true;
                                break 'search;
                            }
                        }
                    }
                }
                row
            })
            .collect()
    } else {
        raw_mask
    };

    Some(RestMaskGrid2D {
        min_x,
        min_y,
        cell_mm: res,
        nx,
        ny,
        mask,
    })
}

/// Computes the combined exact 2D bounding box of all outer contours in `region`.
pub fn region_aabb_2d(region: &RegionSet2D) -> Option<Aabb2> {
    region.exact_aabb()
}

/// Parameters controlling gouge-free 3D linking between consecutive finishing passes.
#[derive(Debug, Clone, Copy)]
pub struct SurfaceLinkConfig {
    pub retract_z: f64,
    pub stock_top_z: f64,
    pub z_floor: f64,
    pub stay_down_limit_mm: f64,
    pub sample_step_mm: f64,
    pub lead_feed: f64,
    pub plunge_feed: f64,
}

/// Emits a gouge-proof link from `builder.current_pos()` to `to_wcs`, either via a short
/// surface-hugging stay-down link or a safe retract + rapid traverse + vertical approach, forcing
/// a safe retract whenever the XY chord crosses `avoid_region`.
pub fn emit_surface_safe_link_with_avoid(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    to_wcs: DVec3,
    cfg: &SurfaceLinkConfig,
    avoid_region: Option<&RegionSet2D>,
    bvh_scratch: &mut Vec<usize>,
    builder: &mut ToolpathBuilder,
) {
    let safe_rapid_approach_z = (cfg.stock_top_z + 1.0)
        .max(to_wcs.z + 1.0)
        .min(cfg.retract_z);

    if let Some(from_wcs) = builder.current_pos() {
        let dist_xy = (to_wcs.x - from_wcs.x).hypot(to_wcs.y - from_wcs.y);
        if dist_xy <= 1e-4 && (to_wcs.z - from_wcs.z).abs() <= 1e-4 {
            return;
        }

        // Try gouge-proof surface-hugging stay-down link for short adjacent stepovers,
        // except when descending steeply off a split overhang cliff or crossing an Avoid / Keep-Out region.
        let steep_cliff_drop =
            (from_wcs.z - to_wcs.z) > 1.5 && dist_xy < (from_wcs.z - to_wcs.z) * 0.5;
        let crosses_avoid = avoid_region.is_some_and(|av| {
            !av.is_empty()
                && !crate::surface3d::is_2d_segment_clear_of_avoid(
                    DVec2::new(from_wcs.x, from_wcs.y),
                    DVec2::new(to_wcs.x, to_wcs.y),
                    av,
                )
        });
        if !steep_cliff_drop
            && !crosses_avoid
            && dist_xy <= cfg.stay_down_limit_mm
            && dist_xy > 1e-4
        {
            let step = cfg.sample_step_mm.clamp(0.10, 0.25);
            let n = (dist_xy / step).ceil().max(3.0) as usize;
            let mut raw_link = Vec::with_capacity(n + 3);
            let base_min_z = from_wcs.z.min(to_wcs.z);
            let max_allowed_rise = (2.0 * cutter.max_radius_mm()).clamp(2.5, 4.5);
            let link_clearance = 0.25_f64;

            // Smooth vertical micro-lift off `from_wcs` so turnaround links never graze the surface
            let from_lift_z = (from_wcs.z + link_clearance).min(cfg.retract_z);
            let to_lift_z = (to_wcs.z + link_clearance).min(cfg.retract_z);
            raw_link.push(DVec3::new(from_wcs.x, from_wcs.y, from_lift_z));

            for k in 1..n {
                let t = k as f64 / n as f64;
                let x = from_wcs.x + (to_wcs.x - from_wcs.x) * t;
                let y = from_wcs.y + (to_wcs.y - from_wcs.y) * t;
                let lerp_z = from_lift_z + (to_lift_z - from_lift_z) * t;
                let z_surf =
                    drop_cutter_z_with_scratch(surface, cutter, x, y, cfg.z_floor, bvh_scratch)
                        .unwrap_or(cfg.z_floor);
                let z = lerp_z.max(z_surf + link_clearance).min(cfg.retract_z);
                raw_link.push(DVec3::new(x, y, z));
            }
            raw_link.push(DVec3::new(to_wcs.x, to_wcs.y, to_lift_z));

            // Recursively verify every link chord clears exact `drop_cutter_z + 0.15 mm`
            let mut safe_link = Vec::with_capacity(raw_link.len() * 2 + 2);
            safe_link.push(raw_link[0]);
            for win in raw_link.windows(2) {
                refine_link_chord_recursive(
                    surface,
                    cutter,
                    cfg.z_floor,
                    win[0],
                    win[1],
                    0.15,
                    0,
                    bvh_scratch,
                    &mut safe_link,
                );
            }
            safe_link.push(to_wcs);

            let max_rise = safe_link
                .iter()
                .map(|p| p.z - base_min_z)
                .fold(0.0_f64, f64::max);

            if max_rise <= max_allowed_rise {
                for &p in &safe_link {
                    builder.linear_to(p, cfg.lead_feed, MotionRole::LinkTransition);
                }
                return;
            }
        }

        // Safe retract above stock/part before rapid traverse
        let pull_off_z = (from_wcs.z + 0.5).min(cfg.retract_z);
        if pull_off_z > from_wcs.z + 1e-4 {
            builder.linear_to(
                DVec3::new(from_wcs.x, from_wcs.y, pull_off_z),
                cfg.lead_feed,
                MotionRole::LeadOut,
            );
        }
        if pull_off_z < cfg.retract_z - 1e-4 {
            builder.rapid_to(DVec3::new(from_wcs.x, from_wcs.y, cfg.retract_z));
        }
    }

    // Rapid horizontally at safe retract height, rapid down only to guaranteed stock clearance,
    // then feed cleanly down onto `to_wcs`.
    builder.rapid_to(DVec3::new(to_wcs.x, to_wcs.y, cfg.retract_z));
    if safe_rapid_approach_z < cfg.retract_z - 1e-4 {
        builder.rapid_to(DVec3::new(to_wcs.x, to_wcs.y, safe_rapid_approach_z));
    }
    if to_wcs.z < safe_rapid_approach_z - 1e-4 {
        builder.linear_to(to_wcs, cfg.plunge_feed, MotionRole::Plunge);
    }
}

/// Emits a clean vertical pull-off and rapid retract to `retract_z` at the end of a 3D op.
pub fn emit_final_surface_retract(retract_z: f64, lead_feed: f64, builder: &mut ToolpathBuilder) {
    if let Some(last) = builder.current_pos()
        && last.z < retract_z - 1e-4
    {
        let pull_off_z = (last.z + 0.5).min(retract_z);
        if pull_off_z > last.z + 1e-4 {
            builder.linear_to(
                DVec3::new(last.x, last.y, pull_off_z),
                lead_feed,
                MotionRole::LeadOut,
            );
        }
        if pull_off_z < retract_z - 1e-4 {
            builder.rapid_to(DVec3::new(last.x, last.y, retract_z));
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn refine_link_chord_recursive(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    z_floor: f64,
    a: DVec3,
    b: DVec3,
    clearance_mm: f64,
    depth: u32,
    scratch: &mut Vec<usize>,
    out: &mut Vec<DVec3>,
) {
    let xy_dist = (b.x - a.x).hypot(b.y - a.y);
    if depth < 4 && xy_dist > 0.04 {
        let mid_x = 0.5 * (a.x + b.x);
        let mid_y = 0.5 * (a.y + b.y);
        let chord_mid_z = 0.5 * (a.z + b.z);
        if let Some(z_raw) =
            drop_cutter_z_with_scratch(surface, cutter, mid_x, mid_y, z_floor, scratch)
        {
            let req_z = z_raw.max(z_floor) + clearance_mm;
            if req_z > chord_mid_z + 0.002 {
                let mid = DVec3::new(mid_x, mid_y, req_z);
                refine_link_chord_recursive(
                    surface,
                    cutter,
                    z_floor,
                    a,
                    mid,
                    clearance_mm,
                    depth + 1,
                    scratch,
                    out,
                );
                refine_link_chord_recursive(
                    surface,
                    cutter,
                    z_floor,
                    mid,
                    b,
                    clearance_mm,
                    depth + 1,
                    scratch,
                    out,
                );
                return;
            }
        }
    }
    out.push(b);
}

/// Suppresses spurious 3D rest / pencil residual cells caused by disconnected overhanging roofs
/// above an open air void, while preserving 100% of genuine concave fillets, V-grooves,
/// vertical-wall-to-floor corners, and sculpted relief valleys.
///
/// For any cell `(x, y)` with `residual > 0`, locates the highest reference-ball surface contact
/// `c_ref` inside `r <= ref_radius_mm` and probes vertically beneath `c_ref`:
/// - If solid material connects `z_cur` continuously up to `c_ref`, the residual is genuine and
///   remains untouched.
/// - If `c_ref` sits on an overhanging structure above an open air gap underneath a downward-facing
///   surface (`normal.z < -0.15`), recomputes the maximum residual from non-overhanging connected
///   surfaces below the overhang ceiling (zeroing pure overhang-shadow bands while preserving any
///   genuine lower fillet nearby).
#[allow(clippy::too_many_arguments)]
pub fn suppress_overhang_shadow_residuals(
    surface: &SurfaceMesh3D,
    current_cutter: &SurfaceCutterProfile,
    ref_radius_mm: f64,
    z_cur: &[Option<f64>],
    residual: &mut [f64],
    min_x: f64,
    min_y: f64,
    cell_mm: f64,
    nx: usize,
    ny: usize,
) {
    if nx == 0 || ny == 0 || surface.triangles.is_empty() || residual.len() < nx * ny {
        return;
    }

    // Fast path: if the mesh has no downward-facing triangles strictly above its lowest floor,
    // no overhang shadow can exist anywhere on the part.
    let z_min_part = surface.bounds.min.z;
    let has_any_overhang = surface
        .triangles
        .iter()
        .any(|t| t.normal.z < -0.15 && t.aabb.max.z > z_min_part + 0.05);
    if !has_any_overhang {
        return;
    }

    let ref_r = ref_radius_mm.max(current_cutter.max_radius_mm());
    let ref_d = (2.0 * ref_r).max(0.1);
    let ref_ball_geom = chip_tools::CutterGeometry::BallEndmill {
        diameter_mm: ref_d,
        flute_length_mm: current_cutter.flute_length_mm.max(ref_d * 2.0),
        overall_length_mm: current_cutter.flute_length_mm.max(ref_d * 2.0) + 20.0,
        shank_diameter_mm: ref_d,
        flutes: 2,
    };
    let ref_cutter = SurfaceCutterProfile::from_tool_geometry(
        &ref_ball_geom,
        0.0,
        current_cutter.stock_axial_mm,
    );

    residual
        .par_chunks_mut(nx)
        .enumerate()
        .for_each(|(iy, row)| {
            if iy >= ny {
                return;
            }
            let y = min_y + iy as f64 * cell_mm;
            let mut cand_buf = Vec::with_capacity(64);
            let mut ray_buf = Vec::with_capacity(32);

            for (ix, res_val) in row.iter_mut().enumerate() {
                if *res_val <= 1e-4 {
                    continue;
                }
                let Some(zc) = z_cur[iy * nx + ix] else {
                    continue;
                };
                let x = min_x + ix as f64 * cell_mm;
                let p_xy = DVec2::new(x, y);
                let z_surf_cur = zc - current_cutter.stock_axial_mm;

                let Some((_best_z, best_pt, best_tri)) =
                    drop_cutter_highest_contact(surface, &ref_cutter, p_xy, &mut cand_buf)
                else {
                    continue;
                };

                let Some(z_overhang_bot) = overhang_ceiling_above_air_gap(
                    surface,
                    best_pt,
                    best_tri,
                    p_xy,
                    z_surf_cur,
                    &mut ray_buf,
                ) else {
                    continue;
                };

                // Highest `z_ref` contact rests on an overhanging roof above an open air gap.
                // Check whether any connected (non-overhanging) triangle below the overhang
                // ceiling still leaves a genuine concave residual at `(x, y)`.
                let mut best_connected_z = f64::NEG_INFINITY;
                for &ti in &cand_buf {
                    let tri = &surface.triangles[ti];
                    if tri.aabb.min.z >= z_overhang_bot - 0.05 {
                        continue;
                    }
                    let [i0, i1, i2] = tri.indices;
                    let v0 = surface.vertices[i0 as usize];
                    let v1 = surface.vertices[i1 as usize];
                    let v2 = surface.vertices[i2 as usize];
                    if let Some((z_tri, c_pt)) =
                        drop_cutter_triangle_contact_shape(&ref_cutter.shape, p_xy, v0, v1, v2)
                        && z_tri > best_connected_z
                        && overhang_ceiling_above_air_gap(
                            surface,
                            c_pt,
                            ti,
                            p_xy,
                            z_surf_cur,
                            &mut ray_buf,
                        )
                        .is_none()
                    {
                        best_connected_z = z_tri;
                    }
                }

                let max_connected_res = if best_connected_z.is_finite() {
                    (best_connected_z + ref_cutter.stock_axial_mm - zc).max(0.0)
                } else {
                    0.0
                };
                *res_val = (*res_val).min(max_connected_res);
            }
        });
}

/// Returns `true` when the transition between two nearby 3D cutter-tip points `a` and `b`
/// crosses a vertical overhang-shadow air gap (upper point resting on a disconnected overhanging
/// roof above a downward-facing underside `normal.z < -0.15`, lower point on the lower floor/hub),
/// rather than a continuous steep wall.
pub fn is_overhang_shadow_transition(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    a: DVec3,
    b: DVec3,
    cand_buf: &mut Vec<usize>,
    ray_buf: &mut Vec<usize>,
) -> bool {
    let (hi, lo) = if a.z >= b.z { (a, b) } else { (b, a) };
    if hi.z - lo.z <= 0.15 {
        return false;
    }
    let hi_xy = DVec2::new(hi.x, hi.y);
    let lo_xy = DVec2::new(lo.x, lo.y);
    let z_surf_lo = lo.z - cutter.stock_axial_mm;

    let Some((_z_hit, contact_pt, tri_idx)) =
        drop_cutter_highest_contact(surface, cutter, hi_xy, cand_buf)
    else {
        return false;
    };

    overhang_ceiling_above_air_gap(surface, contact_pt, tri_idx, lo_xy, z_surf_lo, ray_buf)
        .is_some()
}

/// Splits a 3D finishing cutting chain at steep downward overhang-shadow drop-offs so the cutter
/// retracts/approaches cleanly and plunges at `plunge_feed` via [`emit_surface_safe_link`] instead
/// of diving nearly vertically at full `cut_feed`, while keeping continuous steep walls intact.
pub fn split_chains_at_overhang_cliffs(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    chain: Vec<DVec3>,
) -> Vec<Vec<DVec3>> {
    if chain.len() < 2 {
        return Vec::new();
    }

    let z_min_part = surface.bounds.min.z;
    let has_any_overhang = surface
        .triangles
        .iter()
        .any(|t| t.normal.z < -0.15 && t.aabb.max.z > z_min_part + 0.05);
    if !has_any_overhang {
        return vec![chain];
    }

    let tool_d = 2.0 * cutter.max_radius_mm();
    let max_drop_mm = (0.5 * tool_d).max(1.5);
    // tan(70°) ≈ 2.747477419454622
    const TAN_70_DEG: f64 = 2.747_477_419_454_622;

    let mut cand_buf = Vec::with_capacity(64);
    let mut ray_buf = Vec::with_capacity(32);
    let mut out: Vec<Vec<DVec3>> = Vec::new();
    let mut cur: Vec<DVec3> = Vec::with_capacity(chain.len());
    cur.push(chain[0]);

    for &b in &chain[1..] {
        let a = *cur.last().unwrap_or(&b);
        let dz_drop = a.z - b.z;
        let dz_rise = b.z - a.z;
        let dxy = (b.x - a.x).hypot(b.y - a.y);
        let is_steep_drop = dz_drop > max_drop_mm && dz_drop > TAN_70_DEG * dxy;
        if is_steep_drop
            && is_overhang_shadow_transition(surface, cutter, a, b, &mut cand_buf, &mut ray_buf)
        {
            if cur.len() >= 2 {
                out.push(std::mem::take(&mut cur));
            } else {
                cur.clear();
            }
            cur.push(b);
        } else if dz_drop > 0.05 && dz_drop > TAN_70_DEG * dxy && dxy > 1e-4 {
            // Non-overhanging steep wall descent: step clear horizontally at high Z before descending
            cur.push(DVec3::new(b.x, b.y, a.z));
            cur.push(b);
        } else if dz_rise > 0.05 && dz_rise > TAN_70_DEG * dxy && dxy > 1e-4 {
            // Steep vertical wall / overhang climb: rise straight up in clear air before stepping onto crest
            cur.push(DVec3::new(a.x, a.y, b.z));
            cur.push(b);
        } else {
            cur.push(b);
        }
    }

    if cur.len() >= 2 {
        out.push(cur);
    }
    out
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ColumnGapCheck {
    OverhangGap(f64),
    SolidConnected,
    MissedUpper,
}

fn overhang_ceiling_above_air_gap(
    surface: &SurfaceMesh3D,
    contact_pt: DVec3,
    tri_idx: usize,
    ref_xy: DVec2,
    z_surf_lo: f64,
    ray_buf: &mut Vec<usize>,
) -> Option<f64> {
    if contact_pt.z <= z_surf_lo + 0.08 {
        return None;
    }
    let tri = &surface.triangles[tri_idx];
    let [i0, i1, i2] = tri.indices;
    let v0 = surface.vertices[i0 as usize];
    let v1 = surface.vertices[i1 as usize];
    let v2 = surface.vertices[i2 as usize];

    let c_xy = DVec2::new(contact_pt.x, contact_pt.y);
    let centroid_xy = DVec2::new((v0.x + v1.x + v2.x) / 3.0, (v0.y + v1.y + v2.y) / 3.0);
    let dir_in = (c_xy - ref_xy).normalize_or_zero();
    let has_dir = dir_in.length_squared() > 0.5;
    let perp = DVec2::new(-dir_in.y, dir_in.x);

    let mut probes = [DVec2::ZERO; 7];
    let mut n_probes = 0usize;
    if has_dir {
        probes[n_probes] = c_xy + dir_in * 0.08;
        n_probes += 1;
        probes[n_probes] = c_xy + dir_in * 0.02;
        n_probes += 1;
        // Grazing-corner fallback: step slightly to either lateral side of `dir_in` so rays
        // aligned directly with a roof corner edge still enter the overhanging solid cleanly.
        probes[n_probes] = c_xy + dir_in * 0.08 + perp * 0.06;
        n_probes += 1;
        probes[n_probes] = c_xy + dir_in * 0.08 - perp * 0.06;
        n_probes += 1;
    } else {
        probes[n_probes] = c_xy;
        n_probes += 1;
    }
    if tri.normal.z.abs() > 0.1 {
        probes[n_probes] = c_xy.lerp(centroid_xy, 0.25);
        n_probes += 1;
    }
    if has_dir {
        probes[n_probes] = c_xy + dir_in * 0.25;
        n_probes += 1;
    }

    for &q_xy in &probes[..n_probes] {
        match column_overhang_bottom_above_gap(
            surface,
            q_xy,
            z_surf_lo,
            contact_pt.z,
            tri.aabb.max.z,
            ray_buf,
        ) {
            ColumnGapCheck::OverhangGap(z_bot) => return Some(z_bot),
            ColumnGapCheck::SolidConnected => return None,
            ColumnGapCheck::MissedUpper => {}
        }
    }
    None
}

fn column_overhang_bottom_above_gap(
    surface: &SurfaceMesh3D,
    q_xy: DVec2,
    z_surf_lo: f64,
    z_contact: f64,
    z_tri_max: f64,
    ray_buf: &mut Vec<usize>,
) -> ColumnGapCheck {
    // Tiny deterministic XY perturbation prevents vertical rays from slipping through shared edges
    let q = DVec2::new(q_xy.x + 1.13e-7, q_xy.y + 2.37e-7);
    surface.bvh.query_xy_circle_into(q.x, q.y, 1e-5, ray_buf);
    if ray_buf.is_empty() {
        return ColumnGapCheck::MissedUpper;
    }

    let mut hits: Vec<(f64, f64)> = Vec::with_capacity(ray_buf.len().min(16));
    for &ti in ray_buf.iter() {
        let tri = &surface.triangles[ti];
        let nz = tri.normal.z;
        if nz.abs() <= 1e-5 {
            continue;
        }
        let [i0, i1, i2] = tri.indices;
        let v0 = surface.vertices[i0 as usize];
        let v1 = surface.vertices[i1 as usize];
        let v2 = surface.vertices[i2 as usize];
        let v0_2 = DVec2::new(v0.x, v0.y);
        let v1_2 = DVec2::new(v1.x, v1.y);
        let v2_2 = DVec2::new(v2.x, v2.y);
        if !point_in_triangle_2d(q, v0_2, v1_2, v2_2) {
            continue;
        }
        let n = tri.normal;
        let z_hit = v0.z - (n.x * (q.x - v0.x) + n.y * (q.y - v0.y)) / nz;
        if z_hit.is_finite() {
            hits.push((z_hit, nz));
        }
    }
    if hits.is_empty() {
        return ColumnGapCheck::MissedUpper;
    }

    // Sort ascending by Z; break near-coplanar ties with upward faces (`nz > 0`) first so flush
    // stacked-box internal seams (`+Z` coplanar with `-Z`) never register as an air gap.
    hits.sort_by(|a, b| {
        if (a.0 - b.0).abs() <= 1e-4 {
            b.1.total_cmp(&a.1)
        } else {
            a.0.total_cmp(&b.0)
        }
    });

    let z_upper_limit = z_contact.max(z_tri_max) + 0.15;
    let mut highest_below = z_surf_lo;
    let mut saw_upper_solid = false;

    for &(z_hit, nz) in &hits {
        if z_hit <= z_surf_lo + 0.05 {
            continue;
        }
        if z_hit > z_upper_limit {
            break;
        }
        if nz < -0.15 {
            // Confirm no coplanar upward-facing seam sits within 0.05 mm of this downward face
            let has_coplanar_up = hits
                .iter()
                .any(|&(zh, nzh)| nzh > 0.05 && (zh - z_hit).abs() <= 0.05);
            if !has_coplanar_up && z_hit - highest_below > 0.05 {
                return ColumnGapCheck::OverhangGap(z_hit);
            }
        } else if nz > 0.05 {
            if z_hit >= z_contact - 0.15 {
                saw_upper_solid = true;
                break;
            }
            highest_below = highest_below.max(z_hit);
        }
    }

    if saw_upper_solid {
        ColumnGapCheck::SolidConnected
    } else {
        ColumnGapCheck::MissedUpper
    }
}
