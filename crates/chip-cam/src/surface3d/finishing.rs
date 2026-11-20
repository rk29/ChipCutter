//! Shared 3D surface finishing utilities: cusp-height stepover inversion, 3D Ramer–Douglas–Peucker
//! chord decimation, adaptive convex-cliff subdivision, analytical 3D Rest Masking, and
//! surface-safe collision-free 3D linking.

use chip_math::{Aabb2, DVec2, DVec3};
use rayon::prelude::*;

use crate::{
    cl_data::MotionRole,
    params::{RestMachiningConfig, RestMachiningMode},
    rest2d::{RegionSet2D, UpstreamSweptInput},
    surface3d::{
        SurfaceCutterProfile, SurfaceCutterShape, SurfaceMesh3D, drop_cutter_points_parallel,
        drop_cutter_z_with_scratch,
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

/// Simplifies a 3D toolpath polyline using exact iterative 3D Ramer–Douglas–Peucker decimation
/// within `tol_mm` chord deviation.
pub fn simplify_3d_polyline_rdp(pts: &[DVec3], tol_mm: f64) -> Vec<DVec3> {
    if pts.len() <= 2 {
        return pts.to_vec();
    }
    let tol = tol_mm.clamp(1e-4, 0.012);
    let tol_sq = tol * tol;

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

        let mut max_dist_sq = -1.0_f64;
        let mut max_idx = start;

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
                max_idx = i;
            }
        }

        if max_dist_sq > tol_sq {
            keep[max_idx] = true;
            stack.push((start, max_idx));
            stack.push((max_idx, end));
        }
    }

    let mut out = Vec::with_capacity(pts.len());
    for (i, &p) in pts.iter().enumerate() {
        if keep[i] {
            out.push(p);
        }
    }
    out
}

/// Refines any segment in `chain` whose midpoint dips below exact `drop_cutter_z` (e.g. sharp
/// convex ridges or plateau drop-offs) so chords never gouge convex edges.
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
    let sag_tol = max_sag_mm.clamp(0.002, 0.012);
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
    depth: u32,
    scratch: &mut Vec<usize>,
    out: &mut Vec<DVec3>,
) {
    let xy_dist = (b.x - a.x).hypot(b.y - a.y);
    let dz = (b.z - a.z).abs();
    if depth < 4 && xy_dist > 0.04 && (dz > 0.08 || xy_dist > 0.6) {
        let mid_xy = DVec2::new(0.5 * (a.x + b.x), 0.5 * (a.y + b.y));
        let chord_mid_z = 0.5 * (a.z + b.z);
        if let Some(z_raw) =
            drop_cutter_z_with_scratch(surface, cutter, mid_xy.x, mid_xy.y, z_floor, scratch)
        {
            let z_surf = z_raw.clamp(z_floor, z_max_clamp);
            if z_surf > chord_mid_z + sag_tol {
                let mid = DVec3::new(mid_xy.x, mid_xy.y, z_surf);
                subdivide_chord_recursive(
                    surface,
                    cutter,
                    z_floor,
                    z_max_clamp,
                    a,
                    mid,
                    sag_tol,
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

    /// Returns the number of active cells in the mask.
    pub fn active_count(&self) -> usize {
        self.mask.iter().filter(|&&b| b).count()
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

    let residual = compute_morphological_rest_residual(
        &z_ref,
        &z_cur,
        0.5 * ref_d_eff + prev_stock,
        current_cutter.max_radius_mm(),
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

/// Computes the combined 2D bounding box of all outer contours in `region`.
pub fn region_aabb_2d(region: &RegionSet2D) -> Option<Aabb2> {
    let mut aabb = Aabb2::EMPTY;
    for r in &region.regions {
        for seg in &r.outer_ccw.segments {
            match *seg {
                chip_math::ArcSegment2D::Line { start, end } => {
                    aabb = aabb.union(&Aabb2::from_points([start, end]));
                }
                chip_math::ArcSegment2D::Arc { center, radius, .. } => {
                    let r_vec = DVec2::splat(radius.abs());
                    aabb = aabb.union(&Aabb2::from_points([center - r_vec, center + r_vec]));
                }
            }
        }
    }
    if aabb.is_valid() { Some(aabb) } else { None }
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

/// Connects `builder.current_pos()` to the start of the next 3D pass `to_wcs` using either a
/// collision-checked surface-hugging stay-down link or a safe retract/rapid/plunge sequence.
pub fn emit_surface_safe_link(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    to_wcs: DVec3,
    cfg: &SurfaceLinkConfig,
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

        // Try gouge-proof surface-hugging stay-down link for short adjacent stepovers
        if dist_xy <= cfg.stay_down_limit_mm && dist_xy > 1e-4 {
            let step = cfg.sample_step_mm.clamp(0.15, 0.5);
            let n = (dist_xy / step).ceil().max(2.0) as usize;
            let mut link_pts = Vec::with_capacity(n + 1);
            let mut max_rise = 0.0_f64;
            let base_min_z = from_wcs.z.min(to_wcs.z);
            let max_allowed_rise = (3.0 * cutter.max_radius_mm()).max(6.0);

            for k in 1..=n {
                let t = k as f64 / n as f64;
                let x = from_wcs.x + (to_wcs.x - from_wcs.x) * t;
                let y = from_wcs.y + (to_wcs.y - from_wcs.y) * t;
                let lerp_z = from_wcs.z + (to_wcs.z - from_wcs.z) * t;
                let z_surf =
                    drop_cutter_z_with_scratch(surface, cutter, x, y, cfg.z_floor, bvh_scratch)
                        .unwrap_or(cfg.z_floor);
                // Strictly clear the surface by +0.02 mm along intermediate link vertices
                let z = if k == n {
                    to_wcs.z.max(z_surf)
                } else {
                    lerp_z.max(z_surf + 0.025)
                };
                max_rise = max_rise.max(z - base_min_z);
                link_pts.push(DVec3::new(x, y, z));
            }

            if max_rise <= max_allowed_rise {
                for p in link_pts {
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
