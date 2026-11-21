use chip_cad::{BodyId, CadDocument};
use chip_cam::{ResolvedSetup, ResolvedStockGeometry, StockAxis};
use chip_math::{Aabb3, DVec3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::heightmap::{SimError, SimResolution, UNCUT_OP_IDX};

/// Minimum retained dexel interval thickness (`mm`) below which a sliver is discarded.
pub(crate) const MIN_DEXEL_SPAN_MM: f64 = 1e-6;

/// Selectable volumetric stock simulation engine (`Phase 11`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimEngineKind {
    /// Fast 2.5D Z-Dexel heightmap (or polar cylindrical grid on 4th-axis setups).
    #[default]
    Heightmap,
    /// Full 3D 3-axis orthogonal Tri-Dexel volumetric solid (accurate vertical walls & undercuts).
    TriDexel,
}

impl SimEngineKind {
    pub const ALL: [Self; 2] = [Self::Heightmap, Self::TriDexel];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Heightmap => "Heightmap (Fast)",
            Self::TriDexel => "Tri-Dexel (Volumetric 3D)",
        }
    }

    pub const fn short_slug(self) -> &'static str {
        match self {
            Self::Heightmap => "heightmap",
            Self::TriDexel => "tri-dexel",
        }
    }
}

/// A single solid 1D material interval `[min_mm, max_mm]` along an orthogonal dexel ray,
/// recording which operation last machined each endpoint (`UNCUT_OP_IDX` = raw stock).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DexelSpan {
    pub min_mm: f64,
    pub max_mm: f64,
    pub min_op_idx: u16,
    pub max_op_idx: u16,
}

impl DexelSpan {
    #[inline]
    pub fn new(min_mm: f64, max_mm: f64, min_op_idx: u16, max_op_idx: u16) -> Self {
        Self {
            min_mm,
            max_mm,
            min_op_idx,
            max_op_idx,
        }
    }

    #[inline]
    pub fn len_mm(self) -> f64 {
        (self.max_mm - self.min_mm).max(0.0)
    }
}

/// Subtracts the open/closed cut interval `[cut_min, cut_max]` from a sorted, disjoint ray list
/// `spans`, maintaining sorted non-overlapping invariants and returning total removed length (`mm`).
///
/// Splitting an interior span into two pieces naturally represents true 3D undercuts, horizontal
/// cross-holes, and multi-interval cavities.
pub fn subtract_dexel_interval(
    spans: &mut Vec<DexelSpan>,
    cut_min: f64,
    cut_max: f64,
    op_idx: u16,
) -> f64 {
    if spans.is_empty() || cut_max <= cut_min + 1e-12 {
        return 0.0;
    }
    let mut removed = 0.0;
    let mut i = 0;
    while i < spans.len() {
        let s = spans[i];
        if cut_max <= s.min_mm {
            break;
        }
        if cut_min >= s.max_mm {
            i += 1;
            continue;
        }
        let ov_min = cut_min.max(s.min_mm);
        let ov_max = cut_max.min(s.max_mm);
        let ov = (ov_max - ov_min).max(0.0);
        if ov <= 1e-12 {
            i += 1;
            continue;
        }
        removed += ov;

        let keep_lo = ov_min - s.min_mm > MIN_DEXEL_SPAN_MM;
        let keep_hi = s.max_mm - ov_max > MIN_DEXEL_SPAN_MM;

        match (keep_lo, keep_hi) {
            (false, false) => {
                spans.remove(i);
            }
            (true, false) => {
                spans[i].max_mm = ov_min;
                spans[i].max_op_idx = op_idx;
                i += 1;
            }
            (false, true) => {
                spans[i].min_mm = ov_max;
                spans[i].min_op_idx = op_idx;
                i += 1;
            }
            (true, true) => {
                spans[i].max_mm = ov_min;
                spans[i].max_op_idx = op_idx;
                spans.insert(
                    i + 1,
                    DexelSpan::new(ov_max, s.max_mm, op_idx, s.max_op_idx),
                );
                i += 2;
            }
        }
    }
    removed
}

/// Computes the exact 1D overlap length (`mm`) between sorted disjoint `spans` and `[q_min, q_max]`.
#[inline]
pub fn query_overlap_length(spans: &[DexelSpan], q_min: f64, q_max: f64) -> f64 {
    if spans.is_empty() || q_max <= q_min {
        return 0.0;
    }
    let mut total = 0.0;
    for s in spans {
        if q_max <= s.min_mm {
            break;
        }
        if q_min >= s.max_mm {
            continue;
        }
        let lo = q_min.max(s.min_mm);
        let hi = q_max.min(s.max_mm);
        if hi > lo {
            total += hi - lo;
        }
    }
    total
}

/// Computes the exact 1D penetration depth (`mm`) of interval `[q_min, q_max]` into `target_spans`,
/// returning `(penetration_depth_mm, worst_coord_mm)` if the cut intrudes deeper than `skin_mm`
/// inside any target span.
#[inline]
pub fn query_target_gouge_depth(
    target_spans: &[DexelSpan],
    q_min: f64,
    q_max: f64,
    skin_mm: f64,
) -> Option<(f64, f64)> {
    if target_spans.is_empty() || q_max <= q_min {
        return None;
    }
    let mut best: Option<(f64, f64)> = None;
    for ts in target_spans {
        if q_max <= ts.min_mm {
            break;
        }
        if q_min >= ts.max_mm {
            continue;
        }
        let eff_min = ts.min_mm + skin_mm;
        let eff_max = ts.max_mm - skin_mm;
        if eff_max <= eff_min {
            continue;
        }
        let ov_lo = q_min.max(eff_min);
        let ov_hi = q_max.min(eff_max);
        if ov_hi > ov_lo {
            let depth = ov_hi - ov_lo;
            // Report the endpoint that penetrated deepest into the target span
            let coord = if q_min > eff_min && q_max >= eff_max {
                ov_lo
            } else if q_max < eff_max && q_min <= eff_min {
                ov_hi
            } else {
                0.5 * (ov_lo + ov_hi)
            };
            if best.is_none_or(|(b, _)| depth > b) {
                best = Some((depth, coord));
            }
        }
    }
    best
}

/// Exact 1D set difference length `len(a \ b)` between two sorted disjoint span lists (`O(|a|+|b|)`).
#[inline]
pub fn spans_difference_length(a: &[DexelSpan], b: &[DexelSpan]) -> f64 {
    if a.is_empty() {
        return 0.0;
    }
    if b.is_empty() {
        return a.iter().map(|s| s.len_mm()).sum();
    }
    let mut total = 0.0;
    let mut j = 0usize;
    for sa in a {
        let mut cur = sa.min_mm;
        let end = sa.max_mm;
        while j < b.len() && b[j].max_mm <= cur {
            j += 1;
        }
        let mut k = j;
        while k < b.len() && b[k].min_mm < end {
            if b[k].min_mm > cur {
                total += b[k].min_mm - cur;
            }
            cur = cur.max(b[k].max_mm);
            if cur >= end {
                break;
            }
            k += 1;
        }
        if cur < end {
            total += end - cur;
        }
    }
    total
}

/// Tests whether coordinate `coord` lies inside any solid span of `spans`.
#[inline]
pub fn spans_contain_coord(spans: &[DexelSpan], coord: f64) -> bool {
    for s in spans {
        if coord < s.min_mm {
            return false;
        }
        if coord <= s.max_mm {
            return true;
        }
    }
    false
}

/// 2D grid of parallel dexel rays along a single coordinate axis.
#[derive(Debug, Clone)]
pub struct DexelPlane {
    pub nu: usize,
    pub nv: usize,
    pub origin_u_mm: f64,
    pub origin_v_mm: f64,
    pub du_mm: f64,
    pub dv_mm: f64,
    pub rays: Vec<Vec<DexelSpan>>,
}

impl DexelPlane {
    pub fn new(
        nu: usize,
        nv: usize,
        origin_u_mm: f64,
        origin_v_mm: f64,
        du_mm: f64,
        dv_mm: f64,
    ) -> Self {
        Self {
            nu,
            nv,
            origin_u_mm,
            origin_v_mm,
            du_mm,
            dv_mm,
            rays: vec![Vec::new(); nu * nv],
        }
    }

    #[inline]
    pub fn ray(&self, u: usize, v: usize) -> &[DexelSpan] {
        &self.rays[v * self.nu + u]
    }

    #[inline]
    pub fn ray_mut(&mut self, u: usize, v: usize) -> &mut Vec<DexelSpan> {
        &mut self.rays[v * self.nu + u]
    }

    #[inline]
    pub fn center_u(&self, u: usize) -> f64 {
        self.origin_u_mm + (u as f64 + 0.5) * self.du_mm
    }

    #[inline]
    pub fn center_v(&self, v: usize) -> f64 {
        self.origin_v_mm + (v as f64 + 0.5) * self.dv_mm
    }

    #[inline]
    pub fn index_span_u(&self, min_u: f64, max_u: f64) -> Option<(usize, usize)> {
        if max_u < self.origin_u_mm || min_u > self.origin_u_mm + (self.nu as f64) * self.du_mm {
            return None;
        }
        let u0 = (((min_u - self.origin_u_mm) / self.du_mm).floor() as isize).max(0) as usize;
        let u1 = (((max_u - self.origin_u_mm) / self.du_mm).ceil() as isize)
            .clamp(0, (self.nu - 1) as isize) as usize;
        (u0 <= u1).then_some((u0, u1))
    }

    #[inline]
    pub fn index_span_v(&self, min_v: f64, max_v: f64) -> Option<(usize, usize)> {
        if max_v < self.origin_v_mm || min_v > self.origin_v_mm + (self.nv as f64) * self.dv_mm {
            return None;
        }
        let v0 = (((min_v - self.origin_v_mm) / self.dv_mm).floor() as isize).max(0) as usize;
        let v1 = (((max_v - self.origin_v_mm) / self.dv_mm).ceil() as isize)
            .clamp(0, (self.nv - 1) as isize) as usize;
        (v0 <= v1).then_some((v0, v1))
    }

    /// Total solid volume (`mm³`) integrated across this orthogonal ray plane.
    pub fn volume_mm3(&self) -> f64 {
        let cell_area = self.du_mm * self.dv_mm;
        let total_len: f64 = self
            .rays
            .par_iter()
            .map(|spans| spans.iter().map(|s| s.len_mm()).sum::<f64>())
            .sum();
        total_len * cell_area
    }
}

/// Snapshot of a [`TriDexelStock`] at `completed_segments` for interactive timeline scrubbing.
#[derive(Debug, Clone)]
pub struct TriDexelCheckpoint {
    pub completed_segments: usize,
    pub stock: TriDexelStock,
}

/// Full-3D 3-axis orthogonal **Tri-Dexel** volumetric stock solid in Setup WCS coordinates.
///
/// Stores three mutually orthogonal dexel planes sharing an isotropic `(nx, ny, nz)` lattice:
/// - `z_rays`: `(X, Y)` grid storing solid `Z` spans
/// - `x_rays`: `(Y, Z)` grid storing solid `X` spans
/// - `y_rays`: `(X, Z)` grid storing solid `Y` spans
#[derive(Debug, Clone)]
pub struct TriDexelStock {
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    pub origin_mm: DVec3,
    pub dx_mm: f64,
    pub dy_mm: f64,
    pub dz_mm: f64,
    pub z_rays: DexelPlane,
    pub x_rays: DexelPlane,
    pub y_rays: DexelPlane,
}

impl TriDexelStock {
    /// Maximum single-axis voxel dimension for Tri-Dexel at each [`SimResolution`] tier.
    pub const fn max_axis_dim_for_resolution(resolution: SimResolution) -> usize {
        match resolution {
            SimResolution::Standard => 160,
            SimResolution::Fine => 288,
            SimResolution::Ultra => 448,
        }
    }

    /// Allocates an empty Tri-Dexel lattice matching the exact `(origin, dx, dy, dz, nx, ny, nz)`
    /// of `aabb_wcs` and `resolution`.
    pub fn empty_for_bounds(aabb_wcs: Aabb3, resolution: SimResolution) -> Result<Self, SimError> {
        if !aabb_wcs.is_valid() {
            return Err(SimError::InvalidStockBounds);
        }
        let size = aabb_wcs.size();
        if size.x <= 1e-4 || size.y <= 1e-4 || size.z <= 1e-4 {
            return Err(SimError::InvalidStockBounds);
        }

        let max_dim = Self::max_axis_dim_for_resolution(resolution);
        let longest = size.x.max(size.y).max(size.z);
        let target_pitch = longest / (max_dim as f64);

        let nx = ((size.x / target_pitch).round() as usize).clamp(4, max_dim);
        let ny = ((size.y / target_pitch).round() as usize).clamp(4, max_dim);
        let nz = ((size.z / target_pitch).round() as usize).clamp(4, max_dim);

        let dx_mm = size.x / (nx as f64);
        let dy_mm = size.y / (ny as f64);
        let dz_mm = size.z / (nz as f64);
        let origin_mm = aabb_wcs.min;

        Ok(Self {
            nx,
            ny,
            nz,
            origin_mm,
            dx_mm,
            dy_mm,
            dz_mm,
            z_rays: DexelPlane::new(nx, ny, origin_mm.x, origin_mm.y, dx_mm, dy_mm),
            x_rays: DexelPlane::new(ny, nz, origin_mm.y, origin_mm.z, dy_mm, dz_mm),
            y_rays: DexelPlane::new(nx, nz, origin_mm.x, origin_mm.z, dx_mm, dz_mm),
        })
    }

    /// Allocates an empty Tri-Dexel lattice with identical grid bounds and resolution to `template`.
    pub fn empty_like(template: &Self) -> Self {
        Self {
            nx: template.nx,
            ny: template.ny,
            nz: template.nz,
            origin_mm: template.origin_mm,
            dx_mm: template.dx_mm,
            dy_mm: template.dy_mm,
            dz_mm: template.dz_mm,
            z_rays: DexelPlane::new(
                template.nx,
                template.ny,
                template.origin_mm.x,
                template.origin_mm.y,
                template.dx_mm,
                template.dy_mm,
            ),
            x_rays: DexelPlane::new(
                template.ny,
                template.nz,
                template.origin_mm.y,
                template.origin_mm.z,
                template.dy_mm,
                template.dz_mm,
            ),
            y_rays: DexelPlane::new(
                template.nx,
                template.nz,
                template.origin_mm.x,
                template.origin_mm.z,
                template.dx_mm,
                template.dz_mm,
            ),
        }
    }

    /// Initializes raw stock (`Box` or `Cylinder`) on all three orthogonal dexel planes.
    pub fn from_setup_stock(
        stock: &ResolvedStockGeometry,
        resolution: SimResolution,
    ) -> Result<Self, SimError> {
        let aabb = stock.aabb_wcs();
        let mut grid = Self::empty_for_bounds(aabb, resolution)?;

        match *stock {
            ResolvedStockGeometry::Box { aabb_wcs } => {
                let z_span =
                    DexelSpan::new(aabb_wcs.min.z, aabb_wcs.max.z, UNCUT_OP_IDX, UNCUT_OP_IDX);
                let x_span =
                    DexelSpan::new(aabb_wcs.min.x, aabb_wcs.max.x, UNCUT_OP_IDX, UNCUT_OP_IDX);
                let y_span =
                    DexelSpan::new(aabb_wcs.min.y, aabb_wcs.max.y, UNCUT_OP_IDX, UNCUT_OP_IDX);
                for r in &mut grid.z_rays.rays {
                    r.push(z_span);
                }
                for r in &mut grid.x_rays.rays {
                    r.push(x_span);
                }
                for r in &mut grid.y_rays.rays {
                    r.push(y_span);
                }
            }
            ResolvedStockGeometry::Cylinder {
                axis,
                center_wcs,
                radius_mm,
                min_axial_mm,
                max_axial_mm,
                ..
            } => {
                let r2 = radius_mm * radius_mm;
                // Z-rays: (x, y) -> [z0, z1]
                for j in 0..grid.ny {
                    let y = grid.z_rays.center_v(j);
                    for i in 0..grid.nx {
                        let x = grid.z_rays.center_u(i);
                        let span_opt = match axis {
                            StockAxis::Z => {
                                let d2 = (x - center_wcs.x).powi(2) + (y - center_wcs.y).powi(2);
                                (d2 <= r2).then_some((min_axial_mm, max_axial_mm))
                            }
                            StockAxis::X => {
                                let dy = y - center_wcs.y;
                                if x >= min_axial_mm && x <= max_axial_mm && dy * dy <= r2 {
                                    let dz = (r2 - dy * dy).max(0.0).sqrt();
                                    Some((center_wcs.z - dz, center_wcs.z + dz))
                                } else {
                                    None
                                }
                            }
                            StockAxis::Y => {
                                let dx = x - center_wcs.x;
                                if y >= min_axial_mm && y <= max_axial_mm && dx * dx <= r2 {
                                    let dz = (r2 - dx * dx).max(0.0).sqrt();
                                    Some((center_wcs.z - dz, center_wcs.z + dz))
                                } else {
                                    None
                                }
                            }
                        };
                        if let Some((a, b)) = span_opt
                            && b - a > MIN_DEXEL_SPAN_MM
                        {
                            grid.z_rays.ray_mut(i, j).push(DexelSpan::new(
                                a,
                                b,
                                UNCUT_OP_IDX,
                                UNCUT_OP_IDX,
                            ));
                        }
                    }
                }
                // X-rays: (y, z) -> [x0, x1]
                for k in 0..grid.nz {
                    let z = grid.x_rays.center_v(k);
                    for j in 0..grid.ny {
                        let y = grid.x_rays.center_u(j);
                        let span_opt = match axis {
                            StockAxis::X => {
                                let d2 = (y - center_wcs.y).powi(2) + (z - center_wcs.z).powi(2);
                                (d2 <= r2).then_some((min_axial_mm, max_axial_mm))
                            }
                            StockAxis::Y => {
                                let dz = z - center_wcs.z;
                                if y >= min_axial_mm && y <= max_axial_mm && dz * dz <= r2 {
                                    let dx = (r2 - dz * dz).max(0.0).sqrt();
                                    Some((center_wcs.x - dx, center_wcs.x + dx))
                                } else {
                                    None
                                }
                            }
                            StockAxis::Z => {
                                let dy = y - center_wcs.y;
                                if z >= min_axial_mm && z <= max_axial_mm && dy * dy <= r2 {
                                    let dx = (r2 - dy * dy).max(0.0).sqrt();
                                    Some((center_wcs.x - dx, center_wcs.x + dx))
                                } else {
                                    None
                                }
                            }
                        };
                        if let Some((a, b)) = span_opt
                            && b - a > MIN_DEXEL_SPAN_MM
                        {
                            grid.x_rays.ray_mut(j, k).push(DexelSpan::new(
                                a,
                                b,
                                UNCUT_OP_IDX,
                                UNCUT_OP_IDX,
                            ));
                        }
                    }
                }
                // Y-rays: (x, z) -> [y0, y1]
                for k in 0..grid.nz {
                    let z = grid.y_rays.center_v(k);
                    for i in 0..grid.nx {
                        let x = grid.y_rays.center_u(i);
                        let span_opt = match axis {
                            StockAxis::Y => {
                                let d2 = (x - center_wcs.x).powi(2) + (z - center_wcs.z).powi(2);
                                (d2 <= r2).then_some((min_axial_mm, max_axial_mm))
                            }
                            StockAxis::X => {
                                let dz = z - center_wcs.z;
                                if x >= min_axial_mm && x <= max_axial_mm && dz * dz <= r2 {
                                    let dy = (r2 - dz * dz).max(0.0).sqrt();
                                    Some((center_wcs.y - dy, center_wcs.y + dy))
                                } else {
                                    None
                                }
                            }
                            StockAxis::Z => {
                                let dx = x - center_wcs.x;
                                if z >= min_axial_mm && z <= max_axial_mm && dx * dx <= r2 {
                                    let dy = (r2 - dx * dx).max(0.0).sqrt();
                                    Some((center_wcs.y - dy, center_wcs.y + dy))
                                } else {
                                    None
                                }
                            }
                        };
                        if let Some((a, b)) = span_opt
                            && b - a > MIN_DEXEL_SPAN_MM
                        {
                            grid.y_rays.ray_mut(i, k).push(DexelSpan::new(
                                a,
                                b,
                                UNCUT_OP_IDX,
                                UNCUT_OP_IDX,
                            ));
                        }
                    }
                }
            }
        }

        Ok(grid)
    }

    /// Voxelizes the target 3D solid from `doc` (both STEP B-Rep solids and STL/OBJ/3MF meshes,
    /// filtered by `target_bodies` when non-empty) onto a Tri-Dexel grid matching `template`.
    pub fn from_target_cad(
        doc: &CadDocument,
        setup: &ResolvedSetup,
        target_bodies: &[BodyId],
        template: &Self,
    ) -> Self {
        let mut target = Self::empty_like(template);
        let include_body = |bid: BodyId| target_bodies.is_empty() || target_bodies.contains(&bid);

        let mut tris_wcs: Vec<[DVec3; 3]> = Vec::new();
        for tri in &doc.render_mesh.pick_triangles {
            if let Some(face) = doc.face(tri.face_id)
                && !include_body(face.body_id)
            {
                continue;
            }
            let v0 = setup.frame.world_to_local_point(tri.v0_mm);
            let v1 = setup.frame.world_to_local_point(tri.v1_mm);
            let v2 = setup.frame.world_to_local_point(tri.v2_mm);
            if (v1 - v0).cross(v2 - v0).length_squared() > 1e-14 {
                tris_wcs.push([v0, v1, v2]);
            }
        }
        if tris_wcs.is_empty() {
            return target;
        }

        rasterize_triangles_onto_plane(&mut target.z_rays, &tris_wcs, 0, 1, 2);
        rasterize_triangles_onto_plane(&mut target.x_rays, &tris_wcs, 1, 2, 0);
        rasterize_triangles_onto_plane(&mut target.y_rays, &tris_wcs, 0, 2, 1);
        target
    }

    /// Stock bounding box in Setup WCS coordinates.
    #[inline]
    pub fn aabb_wcs(&self) -> Aabb3 {
        Aabb3::new(
            self.origin_mm,
            self.origin_mm
                + DVec3::new(
                    self.nx as f64 * self.dx_mm,
                    self.ny as f64 * self.dy_mm,
                    self.nz as f64 * self.dz_mm,
                ),
        )
    }

    /// Mean cell pitch (`mm`) across the three axes.
    #[inline]
    pub fn mean_pitch_mm(&self) -> f64 {
        (self.dx_mm + self.dy_mm + self.dz_mm) / 3.0
    }

    /// Per-axis solid volumes `(vol_x, vol_y, vol_z)` in `mm³`.
    pub fn per_axis_volumes_mm3(&self) -> (f64, f64, f64) {
        (
            self.x_rays.volume_mm3(),
            self.y_rays.volume_mm3(),
            self.z_rays.volume_mm3(),
        )
    }

    /// Tri-Dexel averaged solid volume (`(vol_x + vol_y + vol_z) / 3.0`) in `mm³`.
    pub fn compute_volume_mm3(&self) -> f64 {
        let (vx, vy, vz) = self.per_axis_volumes_mm3();
        (vx + vy + vz) / 3.0
    }

    /// Samples the highest solid `Z` coordinate at WCS `(x, y)` (`None` if empty at `(x, y)`).
    pub fn sample_top_z(&self, x_wcs: f64, y_wcs: f64) -> Option<f64> {
        if x_wcs < self.origin_mm.x || y_wcs < self.origin_mm.y {
            return None;
        }
        let i = ((x_wcs - self.origin_mm.x) / self.dx_mm).floor() as isize;
        let j = ((y_wcs - self.origin_mm.y) / self.dy_mm).floor() as isize;
        if i < 0 || j < 0 || (i as usize) >= self.nx || (j as usize) >= self.ny {
            return None;
        }
        self.z_rays
            .ray(i as usize, j as usize)
            .last()
            .map(|s| s.max_mm)
    }

    /// Returns `true` if voxel center `(i, j, k)` is solid by majority vote across the 3 orthogonal
    /// dexel planes (eliminates single-axis grazing artifacts and guarantees watertight meshing).
    #[inline]
    pub fn is_voxel_solid(&self, i: usize, j: usize, k: usize) -> bool {
        let cx = self.origin_mm.x + (i as f64 + 0.5) * self.dx_mm;
        let cy = self.origin_mm.y + (j as f64 + 0.5) * self.dy_mm;
        let cz = self.origin_mm.z + (k as f64 + 0.5) * self.dz_mm;
        let votes = u8::from(spans_contain_coord(self.z_rays.ray(i, j), cz))
            + u8::from(spans_contain_coord(self.x_rays.ray(j, k), cx))
            + u8::from(spans_contain_coord(self.y_rays.ray(i, k), cy));
        votes >= 2
    }
}

/// Rasterizes a closed (or mostly closed) triangle soup onto a single orthogonal [`DexelPlane`]
/// using deterministic sub-micron ray-center jitter so rays never hit shared triangle edges/vertices.
fn rasterize_triangles_onto_plane(
    plane: &mut DexelPlane,
    tris_wcs: &[[DVec3; 3]],
    u_axis: usize,
    v_axis: usize,
    w_axis: usize,
) {
    let comp = |p: DVec3, a: usize| -> f64 {
        match a {
            0 => p.x,
            1 => p.y,
            _ => p.z,
        }
    };

    // Tiny deterministic irrational jitter prevents exact edge/vertex ties on axis-aligned boxes
    const JITTER_U: f64 = 1.371_91e-7;
    const JITTER_V: f64 = 2.718_28e-7;

    let nu = plane.nu;
    let nv = plane.nv;
    let mut hits: Vec<Vec<(f64, i8)>> = vec![Vec::new(); nu * nv];

    for &[p0, p1, p2] in tris_wcs {
        let u0 = comp(p0, u_axis);
        let v0 = comp(p0, v_axis);
        let w0 = comp(p0, w_axis);
        let u1 = comp(p1, u_axis);
        let v1 = comp(p1, v_axis);
        let w1 = comp(p1, w_axis);
        let u2 = comp(p2, u_axis);
        let v2 = comp(p2, v_axis);
        let w2 = comp(p2, w_axis);

        let area2 = (u1 - u0) * (v2 - v0) - (u2 - u0) * (v1 - v0);
        if area2.abs() <= 1e-13 {
            continue;
        }
        let inv_area2 = 1.0 / area2;

        // Determine outward normal sign along `w_axis` for signed-winding pairing
        let n = (p1 - p0).cross(p2 - p0);
        let nw = comp(n, w_axis);
        let dir_sign: i8 = if nw < 0.0 { 1 } else { -1 };

        let min_u = u0.min(u1).min(u2);
        let max_u = u0.max(u1).max(u2);
        let min_v = v0.min(v1).min(v2);
        let max_v = v0.max(v1).max(v2);

        let (Some((iu0, iu1)), Some((iv0, iv1))) = (
            plane.index_span_u(min_u, max_u),
            plane.index_span_v(min_v, max_v),
        ) else {
            continue;
        };

        for iv in iv0..=iv1 {
            let rv = plane.center_v(iv) + JITTER_V;
            let row = iv * nu;
            for iu in iu0..=iu1 {
                let ru = plane.center_u(iu) + JITTER_U;
                let a = ((u1 - ru) * (v2 - rv) - (u2 - ru) * (v1 - rv)) * inv_area2;
                let b = ((u2 - ru) * (v0 - rv) - (u0 - ru) * (v2 - rv)) * inv_area2;
                let c = 1.0 - a - b;
                if a >= 0.0 && b >= 0.0 && c >= 0.0 {
                    let w = a * w0 + b * w1 + c * w2;
                    if w.is_finite() {
                        hits[row + iu].push((w, dir_sign));
                    }
                }
            }
        }
    }

    plane
        .rays
        .par_iter_mut()
        .zip(hits.into_par_iter())
        .for_each(|(dst, mut ray_hits)| {
            dst.clear();
            if ray_hits.len() < 2 {
                return;
            }
            ray_hits.sort_by(|a, b| a.0.total_cmp(&b.0));

            // Deduplicate near-coincident hits with identical sign, or pair via even-odd parity
            // (even-odd is completely insensitive to triangle winding inversions on raw STLs!).
            let mut clean_w: Vec<f64> = Vec::with_capacity(ray_hits.len());
            for &(w, _) in &ray_hits {
                if clean_w.last().is_none_or(|&prev| (w - prev).abs() > 1e-8) {
                    clean_w.push(w);
                }
            }

            if clean_w.len() >= 2 {
                let pairs = clean_w.len() / 2;
                for p in 0..pairs {
                    let a = clean_w[2 * p];
                    let b = clean_w[2 * p + 1];
                    if b - a > MIN_DEXEL_SPAN_MM {
                        dst.push(DexelSpan::new(a, b, UNCUT_OP_IDX, UNCUT_OP_IDX));
                    }
                }
            }
        });
}
