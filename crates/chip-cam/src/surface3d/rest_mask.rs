//! Analytical 2D grid rest-machining mask (`RestMaskGrid2D`) and morphological rest-residual
//! computation for 3D finishing strategies.

use chip_math::{Aabb2, DVec2};
use rayon::prelude::*;

use crate::{
    params::{RestMachiningConfig, RestMachiningMode},
    rest2d::{RegionSet2D, UpstreamSweptInput},
    surface3d::{
        SurfaceCutterProfile, SurfaceMesh3D, drop_cutter_points_parallel,
        linking::suppress_overhang_shadow_residuals,
    },
};

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
