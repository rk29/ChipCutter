use std::f64::consts::TAU;

use chip_bvh::TriMesh3D;
use chip_math::{Aabb3, DVec3};
use rayon::prelude::*;

use crate::types::{RotaryStockSpec, SimError, SimResolution, StockShapeSpec, UNCUT_OP_IDX};

/// Polar / cylindrical heightmap stock parameterized over `(u_axial, θ) -> r_stock(u, θ)`.
///
/// Stores stock-local radial `f32` fields relative to `origin_wcs` (`rotary.center_wcs`) and
/// WGSL-compatible `u32` operation tags.
#[derive(Debug, Clone)]
pub struct CylindricalStock {
    pub origin_wcs: DVec3,
    pub rotary: RotaryStockSpec,
    pub u_min: f64,
    pub u_max: f64,
    pub du: f64,
    pub dtheta: f64,
    pub nu: usize,
    pub ntheta: usize,
    /// Current machined outer radius (`mm`) from rotary centerline at cell `(iu, itheta)`.
    pub r_stock: Vec<f32>,
    /// Initial raw stock outer radius (`mm`) at cell `(iu, itheta)`.
    pub r_init: Vec<f32>,
    /// Polar-rasterized target CAD/mesh outermost radius (`mm`), or `-1.0` where no solid exists.
    pub r_target: Vec<f32>,
    /// Operation index that last machined cell `(iu, itheta)`, or [`UNCUT_OP_IDX`].
    pub last_op_idx: Vec<u32>,
}

impl CylindricalStock {
    /// Initializes a polar [`CylindricalStock`] from `stock` and `rotary`.
    pub fn from_stock_spec(
        stock: &StockShapeSpec,
        rotary: RotaryStockSpec,
        resolution: SimResolution,
    ) -> Result<Self, SimError> {
        let u_min = rotary.stock_u_min;
        let u_max = rotary.stock_u_max;
        let length = u_max - u_min;
        if !length.is_finite() || length <= 1e-3 || rotary.stock_outer_radius_mm <= 1e-3 {
            return Err(SimError::InvalidStockBounds);
        }

        let (nu, ntheta) = match resolution {
            SimResolution::Standard => (256usize, 360usize),
            SimResolution::Fine => (400usize, 540usize),
            SimResolution::Ultra => (512usize, 720usize),
        };
        let du = length / nu as f64;
        let dtheta = TAU / ntheta as f64;
        let total = nu * ntheta;

        let mut r_init = vec![0.0_f32; total];
        for iu in 0..nu {
            let u = u_min + (iu as f64 + 0.5) * du;
            for it in 0..ntheta {
                let theta = it as f64 * dtheta;
                let r = eval_initial_stock_radius(stock, &rotary, u, theta);
                r_init[iu * ntheta + it] = r.max(0.0) as f32;
            }
        }

        let r_stock = r_init.clone();
        let r_target = vec![-1.0_f32; total];
        let last_op_idx = vec![UNCUT_OP_IDX; total];

        Ok(Self {
            origin_wcs: rotary.center_wcs,
            rotary,
            u_min,
            u_max,
            du,
            dtheta,
            nu,
            ntheta,
            r_stock,
            r_init,
            r_target,
            last_op_idx,
        })
    }

    /// Computes total cylindrical volume (`mm³`) of the current `r_stock` polar field:
    /// `V = ∑ 0.5 * r(u, θ)² * dθ * du`.
    pub fn current_volume_mm3(&self) -> f64 {
        let factor = 0.5 * self.du * self.dtheta;
        self.r_stock
            .iter()
            .map(|&r| {
                let rf = f64::from(r.max(0.0));
                rf * rf * factor
            })
            .sum()
    }

    /// Computes initial raw stock volume (`mm³`) from `r_init`.
    pub fn initial_volume_mm3(&self) -> f64 {
        let factor = 0.5 * self.du * self.dtheta;
        self.r_init
            .iter()
            .map(|&r| {
                let rf = f64::from(r.max(0.0));
                rf * rf * factor
            })
            .sum()
    }

    /// Rasterizes a WCS-space [`TriMesh3D`] target solid into `self.r_target` via parallel inward raycasts.
    pub fn rasterize_target_mesh(&mut self, target_mesh: &TriMesh3D) {
        self.r_target.fill(-1.0_f32);
        if target_mesh.is_empty() {
            return;
        }
        let r_cast = self.rotary.stock_outer_radius_mm + 5.0;
        let nu = self.nu;
        let ntheta = self.ntheta;
        let u_min = self.u_min;
        let du = self.du;
        let dtheta = self.dtheta;
        let rotary = self.rotary;

        let target_rows: Vec<Vec<f32>> = (0..nu)
            .into_par_iter()
            .map(|iu| {
                // Deterministic sub-micron jitter avoids shared triangle-edge misses on aligned STL/STEP seams.
                let u = u_min + (iu as f64 + 0.5) * du + 1.371_91e-4 * du;
                let mut row = vec![-1.0_f32; ntheta];
                for (it, cell_r) in row.iter_mut().enumerate() {
                    let theta = it as f64 * dtheta + 2.718_28e-4 * dtheta;
                    let p_outer = rotary.cylindrical_to_wcs(u, theta, r_cast);
                    let p_axis = rotary.cylindrical_to_wcs(u, theta, 0.0);
                    let dir = (p_axis - p_outer).normalize_or_zero();
                    if let Some(hit) = target_mesh.raycast(p_outer, dir, r_cast) {
                        let r_hit = (r_cast - hit.t).max(0.0);
                        *cell_r = r_hit as f32;
                    }
                }
                row
            })
            .collect();

        for (iu, row) in target_rows.into_iter().enumerate() {
            let start = iu * ntheta;
            self.r_target[start..start + ntheta].copy_from_slice(&row);
        }
    }
}

/// Evaluates the exact raw stock outer radius (`mm`) from `rotary.center_wcs` at `(u, theta)`.
fn eval_initial_stock_radius(
    stock: &StockShapeSpec,
    rotary: &RotaryStockSpec,
    u: f64,
    theta: f64,
) -> f64 {
    let (e1, e2, u_vec) = rotary.basis();
    let pol = rotary.direction_sign;
    let d_rad = e1 * theta.cos() + e2 * (theta.sin() * pol);
    let u_center = rotary.center_wcs.dot(u_vec);
    let origin = rotary.center_wcs + u_vec * (u - u_center);

    match *stock {
        StockShapeSpec::Cylinder {
            axis,
            center_wcs,
            radius_mm,
            min_axial_mm,
            max_axial_mm,
            ..
        } if axis == rotary.axis => {
            if u < min_axial_mm - 1e-4 || u > max_axial_mm + 1e-4 {
                return 0.0;
            }
            let rel = origin - center_wcs;
            let perp_o = rel - u_vec * rel.dot(u_vec);
            let b = perp_o.dot(d_rad);
            let c = perp_o.length_squared() - radius_mm * radius_mm;
            let disc = b * b - c;
            if disc < 0.0 {
                0.0
            } else {
                (-b + disc.sqrt()).max(0.0)
            }
        }
        StockShapeSpec::Box { aabb_wcs } | StockShapeSpec::Cylinder { aabb_wcs, .. } => {
            ray_exit_aabb_distance(origin, d_rad, &aabb_wcs).unwrap_or(0.0)
        }
    }
}

/// Computes the exit distance `t_max >= 0` of ray `origin + t * dir` leaving `aabb`.
fn ray_exit_aabb_distance(origin: DVec3, dir: DVec3, aabb: &Aabb3) -> Option<f64> {
    let mut t_min = f64::NEG_INFINITY;
    let mut t_max = f64::INFINITY;
    for axis in 0..3 {
        let (o, d, bmin, bmax) = match axis {
            0 => (origin.x, dir.x, aabb.min.x, aabb.max.x),
            1 => (origin.y, dir.y, aabb.min.y, aabb.max.y),
            _ => (origin.z, dir.z, aabb.min.z, aabb.max.z),
        };
        if d.abs() < 1e-12 {
            if o < bmin - 1e-6 || o > bmax + 1e-6 {
                return None;
            }
        } else {
            let inv = 1.0 / d;
            let mut t0 = (bmin - o) * inv;
            let mut t1 = (bmax - o) * inv;
            if t0 > t1 {
                std::mem::swap(&mut t0, &mut t1);
            }
            t_min = t_min.max(t0);
            t_max = t_max.min(t1);
            if t_min > t_max + 1e-9 {
                return None;
            }
        }
    }
    if t_max >= 0.0 { Some(t_max) } else { None }
}
