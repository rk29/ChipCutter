use chip_math::{DVec2, DVec3};
use rayon::prelude::*;

use crate::types::{
    AnalyticHorizFace, Axis3, SimError, SimResolution, StockShapeSpec, TargetTriangle, UNCUT_OP_IDX,
};

/// High-resolution 2D Z-Dexel heightmap aligned with a Setup's WCS stock bounds (`06-simulation.md` §3).
///
/// All elevation buffers (`z_stock`, `z_bottom`, `z_target`) store stock-local `f32` offsets
/// relative to `origin_wcs.z`. Cells outside the initial stock silhouette have `z_bottom = -inf`;
/// through-drilled cells retain finite `z_bottom` with `z_stock = -inf` (no `in_stock: Vec<bool>`).
#[derive(Debug, Clone)]
pub struct HeightfieldStock {
    pub nx: usize,
    pub ny: usize,
    pub origin_wcs: DVec3,
    pub dx_mm: f64,
    pub dy_mm: f64,
    pub max_initial_top_z_mm: f64,
    pub min_bottom_z_mm: f64,
    /// Stock-local `f32` top elevation (`z_wcs - origin_wcs.z`), or `f32::NEG_INFINITY` if empty.
    pub z_stock: Vec<f32>,
    /// Stock-local `f32` bottom elevation (`z_wcs - origin_wcs.z`), or `f32::NEG_INFINITY` if outside raw stock.
    pub z_bottom: Vec<f32>,
    /// Stock-local `f32` target surface elevation (`z_wcs - origin_wcs.z`), or `f32::NEG_INFINITY` if none.
    pub z_target: Vec<f32>,
    /// `UNCUT_OP_IDX` (`u32::MAX`) indicates unmachined raw stock; `0..N` is the last op index.
    pub last_op_idx: Vec<u32>,
}

impl HeightfieldStock {
    /// Initializes a Z-dexel stock grid from a [`StockShapeSpec`] (`Box` or `Cylinder`).
    pub fn from_stock_spec(
        stock: &StockShapeSpec,
        resolution: SimResolution,
    ) -> Result<Self, SimError> {
        let aabb = stock.aabb_wcs();
        if !aabb.is_valid() {
            return Err(SimError::InvalidStockBounds);
        }
        let size = aabb.size();
        if size.x <= 1e-4 || size.y <= 1e-4 || size.z <= 1e-4 {
            return Err(SimError::InvalidStockBounds);
        }

        let max_dim = resolution.max_grid_dim() as f64;
        let longest_xy = size.x.max(size.y);
        let target_cell = longest_xy / max_dim;
        let nx = ((size.x / target_cell).round() as usize).clamp(8, resolution.max_grid_dim());
        let ny = ((size.y / target_cell).round() as usize).clamp(8, resolution.max_grid_dim());
        let dx_mm = size.x / (nx as f64);
        let dy_mm = size.y / (ny as f64);
        let origin_wcs = aabb.min;

        let total = nx * ny;
        let mut z_stock = vec![f32::NEG_INFINITY; total];
        let mut z_bottom = vec![f32::NEG_INFINITY; total];
        let z_target = vec![f32::NEG_INFINITY; total];
        let last_op_idx = vec![UNCUT_OP_IDX; total];

        match *stock {
            StockShapeSpec::Box { aabb_wcs } => {
                z_stock.fill((aabb_wcs.max.z - origin_wcs.z) as f32);
                z_bottom.fill((aabb_wcs.min.z - origin_wcs.z) as f32);
            }
            StockShapeSpec::Cylinder {
                axis,
                center_wcs,
                radius_mm,
                min_axial_mm,
                max_axial_mm,
                ..
            } => {
                let r2 = radius_mm * radius_mm;
                for j in 0..ny {
                    let y = origin_wcs.y + (j as f64 + 0.5) * dy_mm;
                    let row = j * nx;
                    for i in 0..nx {
                        let x = origin_wcs.x + (i as f64 + 0.5) * dx_mm;
                        let idx = row + i;
                        match axis {
                            Axis3::Z => {
                                let d2 = (x - center_wcs.x).powi(2) + (y - center_wcs.y).powi(2);
                                if d2 <= r2 {
                                    z_stock[idx] = (max_axial_mm - origin_wcs.z) as f32;
                                    z_bottom[idx] = (min_axial_mm - origin_wcs.z) as f32;
                                }
                            }
                            Axis3::X => {
                                let dy_c = y - center_wcs.y;
                                if x >= min_axial_mm && x <= max_axial_mm && dy_c * dy_c <= r2 {
                                    let dz = (r2 - dy_c * dy_c).max(0.0).sqrt();
                                    z_stock[idx] = (center_wcs.z + dz - origin_wcs.z) as f32;
                                    z_bottom[idx] = (center_wcs.z - dz - origin_wcs.z) as f32;
                                }
                            }
                            Axis3::Y => {
                                let dx_c = x - center_wcs.x;
                                if y >= min_axial_mm && y <= max_axial_mm && dx_c * dx_c <= r2 {
                                    let dz = (r2 - dx_c * dx_c).max(0.0).sqrt();
                                    z_stock[idx] = (center_wcs.z + dz - origin_wcs.z) as f32;
                                    z_bottom[idx] = (center_wcs.z - dz - origin_wcs.z) as f32;
                                }
                            }
                        }
                    }
                }
            }
        }

        Ok(Self {
            nx,
            ny,
            origin_wcs,
            dx_mm,
            dy_mm,
            max_initial_top_z_mm: aabb.max.z,
            min_bottom_z_mm: aabb.min.z,
            z_stock,
            z_bottom,
            z_target,
            last_op_idx,
        })
    }

    /// Initializes a 2.5D Z-dexel heightfield directly from the top Z envelope of a carried-over
    /// [`crate::tridexel::TriDexelStock`] (preserving Setup 1's machined top profile and XY footprint
    /// when Setup 2 runs in fast `Heightmap` mode).
    pub fn from_tridexel_projection(td: &crate::tridexel::TriDexelStock) -> Self {
        let z_plane = td.z_plane();
        let nx = z_plane.nu;
        let ny = z_plane.nv;
        let origin_wcs = td.origin_wcs;
        let dx_mm = z_plane.du_mm;
        let dy_mm = z_plane.dv_mm;
        let aabb = td.aabb_wcs();

        let total = nx * ny;
        let mut z_stock = vec![f32::NEG_INFINITY; total];
        let mut z_bottom = vec![f32::NEG_INFINITY; total];
        let z_target = vec![f32::NEG_INFINITY; total];
        let mut last_op_idx = vec![UNCUT_OP_IDX; total];

        for j in 0..ny {
            let row = j * nx;
            for i in 0..nx {
                let idx = row + i;
                if z_plane.counts[idx] == 0 {
                    continue;
                }
                let spans = z_plane.ray_spans_by_index(idx);
                if let (Some(first), Some(last)) = (spans.first(), spans.last()) {
                    z_bottom[idx] = (first.min_mm - origin_wcs.z) as f32;
                    z_stock[idx] = (last.max_mm - origin_wcs.z) as f32;
                    last_op_idx[idx] = last.max_op_idx;
                }
            }
        }

        Self {
            nx,
            ny,
            origin_wcs,
            dx_mm,
            dy_mm,
            max_initial_top_z_mm: aabb.max.z,
            min_bottom_z_mm: aabb.min.z,
            z_stock,
            z_bottom,
            z_target,
            last_op_idx,
        }
    }

    #[inline]
    pub fn z_wcs_to_local(&self, z_wcs: f64) -> f32 {
        (z_wcs - self.origin_wcs.z) as f32
    }

    #[inline]
    pub fn z_local_to_wcs(&self, z_local: f32) -> f64 {
        self.origin_wcs.z + f64::from(z_local)
    }

    #[inline]
    pub fn cell_in_stock(&self, idx: usize) -> bool {
        self.z_bottom[idx].is_finite()
    }

    #[inline]
    pub fn cell_has_material(&self, idx: usize) -> bool {
        self.z_bottom[idx].is_finite() && self.z_stock[idx] > self.z_bottom[idx]
    }

    #[inline]
    pub fn cell_center_xy(&self, i: usize, j: usize) -> DVec2 {
        DVec2::new(
            self.origin_wcs.x + (i as f64 + 0.5) * self.dx_mm,
            self.origin_wcs.y + (j as f64 + 0.5) * self.dy_mm,
        )
    }

    /// Samples remaining `z_stock` height (`mm` in WCS) at WCS `(x, y)` (`None` if outside stock or drilled through).
    pub fn sample_stock_z(&self, x_wcs: f64, y_wcs: f64) -> Option<f64> {
        if x_wcs < self.origin_wcs.x || y_wcs < self.origin_wcs.y {
            return None;
        }
        let i = ((x_wcs - self.origin_wcs.x) / self.dx_mm).floor() as isize;
        let j = ((y_wcs - self.origin_wcs.y) / self.dy_mm).floor() as isize;
        if i < 0 || j < 0 || (i as usize) >= self.nx || (j as usize) >= self.ny {
            return None;
        }
        let idx = (j as usize) * self.nx + (i as usize);
        let zb = self.z_bottom[idx];
        let zs = self.z_stock[idx];
        if !zb.is_finite() || f64::from(zs) <= f64::from(zb) + 1e-5 {
            None
        } else {
            Some(self.z_local_to_wcs(zs))
        }
    }

    /// Samples the maximum remaining `z_stock` height (`mm` in WCS) across all cells within `radius_mm`
    /// of `(cx, cy)` (always including the center cell; `None` if all cells are empty/through-drilled).
    pub fn sample_max_stock_z_in_disk(&self, cx: f64, cy: f64, radius_mm: f64) -> Option<f64> {
        let mut best = self.sample_stock_z(cx, cy);
        let r = radius_mm.max(0.0);
        if let (Some((i0, i1)), Some((j0, j1))) = (
            self.index_span_x(cx - r, cx + r),
            self.index_span_y(cy - r, cy + r),
        ) {
            let r2 = r * r;
            for j in j0..=j1 {
                let py = self.origin_wcs.y + (j as f64 + 0.5) * self.dy_mm;
                let dy = py - cy;
                let row = j * self.nx;
                for i in i0..=i1 {
                    let px = self.origin_wcs.x + (i as f64 + 0.5) * self.dx_mm;
                    let dx = px - cx;
                    if dx * dx + dy * dy <= r2 + 1e-9 {
                        let idx = row + i;
                        let zb = self.z_bottom[idx];
                        let zs = self.z_stock[idx];
                        if zb.is_finite() && f64::from(zs) > f64::from(zb) + 1e-5 {
                            let z = self.z_local_to_wcs(zs);
                            if best.is_none_or(|b| z > b) {
                                best = Some(z);
                            }
                        }
                    }
                }
            }
        }
        best
    }

    /// Computes total remaining stock volume (`mm³`) across all cells.
    pub fn compute_volume_mm3(&self) -> f64 {
        let cell_area = self.dx_mm * self.dy_mm;
        let sum_h: f64 = self
            .z_stock
            .par_iter()
            .zip(self.z_bottom.par_iter())
            .map(|(&zt, &zb)| {
                if zb.is_finite() && zt > zb {
                    f64::from(zt - zb)
                } else {
                    0.0
                }
            })
            .sum();
        sum_h * cell_area
    }

    /// Rasterizes pure geometric target surfaces (upward analytic horizontal faces + upward sloped/mesh
    /// triangles in Setup WCS) onto `self.z_target` in stock-local `f32` coordinates.
    pub fn rasterize_target(
        &mut self,
        horiz_up_faces: &[AnalyticHorizFace],
        triangles: &[TargetTriangle],
    ) {
        self.z_target.fill(f32::NEG_INFINITY);
        let origin_z = self.origin_wcs.z;

        // 1. Parallel row evaluation for all exact upward horizontal faces
        if !horiz_up_faces.is_empty() {
            let nx = self.nx;
            let ox = self.origin_wcs.x;
            let oy = self.origin_wcs.y;
            let dx = self.dx_mm;
            let dy = self.dy_mm;

            self.z_target
                .par_chunks_mut(nx)
                .enumerate()
                .for_each(|(j, row_z)| {
                    let y = oy + (j as f64 + 0.5) * dy;
                    for hf in horiz_up_faces {
                        if y < hf.aabb.min.y - 1e-6 || y > hf.aabb.max.y + 1e-6 {
                            continue;
                        }
                        let z_local = (hf.z_wcs - origin_z) as f32;
                        let i0 = (((hf.aabb.min.x - ox) / dx).floor() as isize).max(0) as usize;
                        let i1 = (((hf.aabb.max.x - ox) / dx).ceil() as isize)
                            .clamp(0, (nx - 1) as isize) as usize;
                        for (off, z_cell) in row_z[i0..=i1].iter_mut().enumerate() {
                            if *z_cell >= z_local {
                                continue;
                            }
                            let i = i0 + off;
                            let pt = DVec2::new(ox + (i as f64 + 0.5) * dx, y);
                            if hf.outer.contains_point(pt)
                                && !hf.inners.iter().any(|inn| inn.contains_point(pt))
                            {
                                *z_cell = (*z_cell).max(z_local);
                            }
                        }
                    }
                });
        }

        // 2. Rasterize sloped / freeform / mesh upward-facing triangles
        for tri in triangles {
            let [v0, v1, v2] = tri.v;
            let tri_n = (v1 - v0).cross(v2 - v0);
            let area2_xy = tri_n.z;
            if area2_xy.abs() <= 1e-8 {
                continue;
            }
            let n_unit = tri_n.normalize_or_zero();
            if n_unit.z <= 1e-4 {
                continue;
            }

            let min_x = v0.x.min(v1.x).min(v2.x);
            let max_x = v0.x.max(v1.x).max(v2.x);
            let min_y = v0.y.min(v1.y).min(v2.y);
            let max_y = v0.y.max(v1.y).max(v2.y);

            let (Some((i0, i1)), Some((j0, j1))) = (
                self.index_span_x(min_x, max_x),
                self.index_span_y(min_y, max_y),
            ) else {
                continue;
            };

            let inv_area2 = 1.0 / area2_xy;
            for j in j0..=j1 {
                let py = self.origin_wcs.y + (j as f64 + 0.5) * self.dy_mm;
                let row = j * self.nx;
                for i in i0..=i1 {
                    let px = self.origin_wcs.x + (i as f64 + 0.5) * self.dx_mm;
                    let w0 = ((v1.x - px) * (v2.y - py) - (v1.y - py) * (v2.x - px)) * inv_area2;
                    let w1 = ((v2.x - px) * (v0.y - py) - (v2.y - py) * (v0.x - px)) * inv_area2;
                    let w2 = 1.0 - w0 - w1;
                    if w0 >= -1e-6 && w1 >= -1e-6 && w2 >= -1e-6 {
                        let z_wcs = if let Some((o_wcs, n_wcs)) = tri.analytic_plane {
                            o_wcs.z
                                - (n_wcs.x * (px - o_wcs.x) + n_wcs.y * (py - o_wcs.y)) / n_wcs.z
                        } else {
                            w0 * v0.z + w1 * v1.z + w2 * v2.z
                        };
                        let zf = (z_wcs - origin_z) as f32;
                        if zf > self.z_target[row + i] {
                            self.z_target[row + i] = zf;
                        }
                    }
                }
            }
        }
    }

    pub(super) fn index_span_x(&self, min_x: f64, max_x: f64) -> Option<(usize, usize)> {
        if max_x < self.origin_wcs.x || min_x > self.origin_wcs.x + (self.nx as f64) * self.dx_mm {
            return None;
        }
        let i0 = (((min_x - self.origin_wcs.x) / self.dx_mm).floor() as isize).max(0) as usize;
        let i1 = (((max_x - self.origin_wcs.x) / self.dx_mm).ceil() as isize)
            .clamp(0, (self.nx - 1) as isize) as usize;
        (i0 <= i1).then_some((i0, i1))
    }

    pub(super) fn index_span_y(&self, min_y: f64, max_y: f64) -> Option<(usize, usize)> {
        if max_y < self.origin_wcs.y || min_y > self.origin_wcs.y + (self.ny as f64) * self.dy_mm {
            return None;
        }
        let j0 = (((min_y - self.origin_wcs.y) / self.dy_mm).floor() as isize).max(0) as usize;
        let j1 = (((max_y - self.origin_wcs.y) / self.dy_mm).ceil() as isize)
            .clamp(0, (self.ny - 1) as isize) as usize;
        (j0 <= j1).then_some((j0, j1))
    }
}
