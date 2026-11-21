use chip_math::DVec3;
use serde::{Deserialize, Serialize};

use crate::{
    cylindrical::CylindricalStock,
    heightmap::HeightfieldStock,
    tridexel::{DexelSpan, TriDexelStock, spans_difference_length},
};

/// Visual coloring mode for simulated 3D stock meshes (`Heightmap` and `TriDexel`).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimStockColorMode {
    /// Uniform machined metallic shading (raw uncut stock slightly darker).
    Metallic,
    /// Distinct color per machining operation / tool (`OPERATION_STOCK_PALETTE`).
    #[default]
    ByOperation,
    /// Signed surface-deviation heatmap comparing machined stock against target geometry:
    /// Green = within `±tolerance_mm`, Cyan -> Blue = uncut excess (`+`), Yellow -> Red = gouge (`-`).
    ErrorHeatmap {
        tolerance_mm: f64,
        max_scale_mm: f64,
    },
}

/// Maps a signed surface deviation (`+` = uncut excess, `-` = overcut/gouge) to a perceptual
/// diverging RGBA color for the 3D stock comparison heatmap.
#[inline]
pub fn deviation_to_heatmap_rgba(
    signed_dev_mm: f64,
    tolerance_mm: f64,
    max_scale_mm: f64,
) -> [f32; 4] {
    let tol = tolerance_mm.max(1e-4);
    let span = (max_scale_mm - tol).max(tol);
    let abs_d = signed_dev_mm.abs();
    if abs_d <= tol {
        return [0.18, 0.78, 0.36, 1.0]; // In-tolerance emerald green
    }
    let t = ((abs_d - tol) / span).clamp(0.0, 1.0) as f32;
    if signed_dev_mm > 0.0 {
        // Excess / uncut: cyan [0.22, 0.80, 0.94] -> deep blue [0.12, 0.28, 0.88]
        [
            0.22 + t * (0.12 - 0.22),
            0.80 + t * (0.28 - 0.80),
            0.94 + t * (0.88 - 0.94),
            1.0,
        ]
    } else {
        // Gouge / overcut: amber-yellow [0.98, 0.78, 0.16] -> crimson red [0.94, 0.16, 0.20]
        [
            0.98 + t * (0.94 - 0.98),
            0.78 + t * (0.16 - 0.78),
            0.16 + t * (0.20 - 0.16),
            1.0,
        ]
    }
}

/// Per-operation snapshot of remaining stock error volume and RMS deviation after completing one
/// operation—enabling apples-to-apples benchmarking of roughing, rest, and finishing strategies.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OperationErrorSnapshot {
    pub op_idx: usize,
    pub op_id: String,
    pub op_name: String,
    pub removed_volume_mm3: f64,
    pub excess_volume_mm3: f64,
    pub gouge_volume_mm3: f64,
    pub total_error_volume_mm3: f64,
    pub rms_error_mm: f64,
}

/// Quantitative stock-vs-target metrology report computed from simulated machined stock
/// (`TriDexelStock`, `HeightfieldStock`, or 4th-axis `CylindricalStock`) compared against the target 3D solid.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StockErrorReport {
    /// Reference tolerance band (`±tolerance_mm`) used for `in_tolerance_surface_pct`.
    pub tolerance_mm: f64,
    /// Reference target solid volume (`mm³`).
    pub target_volume_mm3: f64,
    /// Uncut excess material remaining outside the target solid (`V(Machined \ Target)`, `mm³`).
    pub excess_volume_mm3: f64,
    /// Overcut / gouged material removed from inside the target solid (`V(Target \ Machined)`, `mm³`).
    pub gouge_volume_mm3: f64,
    /// Total symmetric-difference error volume (`excess_volume_mm3 + gouge_volume_mm3`, `mm³`).
    pub total_error_volume_mm3: f64,
    /// Total error volume expressed as a percentage of `target_volume_mm3`.
    pub error_volume_pct_of_target: f64,
    /// Percentage of intended removable stock (`Initial - Target`) removed without excess (`[0, 100]`).
    pub material_removal_efficiency_pct: f64,
    /// Number of target surface boundary points sampled.
    pub sample_count: usize,
    /// Root-Mean-Square surface deviation (`sqrt(mean(d_i²))`, `mm`).
    pub rms_error_mm: f64,
    /// Root-Mean-Square deviation over uncut excess surface points (`d_i > 0`, `mm`).
    pub excess_rms_mm: f64,
    /// Root-Mean-Square deviation over gouged surface points (`d_i < 0`, `mm`).
    pub gouge_rms_mm: f64,
    /// Mean Absolute Error (`mean(|d_i|)`, `mm`).
    pub mean_abs_error_mm: f64,
    /// Mean Signed Error (`mean(d_i)`, `mm`; `+` = oversize/uncut bias, `-` = undersize/overcut bias).
    pub mean_signed_error_mm: f64,
    /// 95th percentile absolute surface error (`mm`).
    pub p95_abs_error_mm: f64,
    /// 99th percentile absolute surface error (`mm`).
    pub p99_abs_error_mm: f64,
    /// Peak uncut excess thickness (`mm`, e.g., worst scallop or unmachined corner).
    pub max_excess_mm: f64,
    /// WCS 3D coordinates of the worst uncut excess sample.
    pub max_excess_location_wcs: Option<DVec3>,
    /// Deepest overcut / gouge penetration (`mm`, positive magnitude).
    pub max_gouge_mm: f64,
    /// WCS 3D coordinates of the worst gouge sample.
    pub max_gouge_location_wcs: Option<DVec3>,
    /// Fraction (`0..=100%`) of sampled target surface points within `±tolerance_mm`.
    pub in_tolerance_surface_pct: f64,
    /// Chronological per-operation error progression for strategy A/B benchmarking.
    pub per_operation: Vec<OperationErrorSnapshot>,
}

struct DeviationStatsAcc {
    abs_samples: Vec<f64>,
    sum_sq: f64,
    sum_abs: f64,
    sum_signed: f64,
    excess_sum_sq: f64,
    excess_count: usize,
    gouge_sum_sq: f64,
    gouge_count: usize,
    in_tol_count: usize,
    max_excess: f64,
    max_excess_loc: Option<DVec3>,
    max_gouge: f64,
    max_gouge_loc: Option<DVec3>,
    tolerance_mm: f64,
}

impl DeviationStatsAcc {
    fn new(tolerance_mm: f64, cap: usize) -> Self {
        Self {
            abs_samples: Vec::with_capacity(cap),
            sum_sq: 0.0,
            sum_abs: 0.0,
            sum_signed: 0.0,
            excess_sum_sq: 0.0,
            excess_count: 0,
            gouge_sum_sq: 0.0,
            gouge_count: 0,
            in_tol_count: 0,
            max_excess: 0.0,
            max_excess_loc: None,
            max_gouge: 0.0,
            max_gouge_loc: None,
            tolerance_mm: tolerance_mm.max(1e-6),
        }
    }

    #[inline]
    fn push(&mut self, signed_dev: f64, loc: DVec3) {
        if !signed_dev.is_finite() {
            return;
        }
        let abs_d = signed_dev.abs();
        self.abs_samples.push(abs_d);
        self.sum_sq += signed_dev * signed_dev;
        self.sum_abs += abs_d;
        self.sum_signed += signed_dev;
        if abs_d <= self.tolerance_mm {
            self.in_tol_count += 1;
        }
        if signed_dev > 1e-9 {
            self.excess_sum_sq += signed_dev * signed_dev;
            self.excess_count += 1;
            if signed_dev > self.max_excess {
                self.max_excess = signed_dev;
                self.max_excess_loc = Some(loc);
            }
        } else if signed_dev < -1e-9 {
            let g = -signed_dev;
            self.gouge_sum_sq += g * g;
            self.gouge_count += 1;
            if g > self.max_gouge {
                self.max_gouge = g;
                self.max_gouge_loc = Some(loc);
            }
        }
    }

    fn finish(
        mut self,
        initial_stock_vol_mm3: f64,
        target_vol_mm3: f64,
        excess_vol_mm3: f64,
        gouge_vol_mm3: f64,
        per_operation: Vec<OperationErrorSnapshot>,
    ) -> StockErrorReport {
        let n = self.abs_samples.len();
        let (rms, mae, mean_signed, p95, p99, in_tol_pct) = if n > 0 {
            self.abs_samples.sort_by(f64::total_cmp);
            let pct_at = |q: f64| -> f64 {
                let idx = ((n as f64 - 1.0) * q).round() as usize;
                self.abs_samples[idx.min(n - 1)]
            };
            (
                (self.sum_sq / n as f64).sqrt(),
                self.sum_abs / n as f64,
                self.sum_signed / n as f64,
                pct_at(0.95),
                pct_at(0.99),
                100.0 * (self.in_tol_count as f64) / (n as f64),
            )
        } else {
            (0.0, 0.0, 0.0, 0.0, 0.0, 100.0)
        };

        let excess_rms = if self.excess_count > 0 {
            (self.excess_sum_sq / self.excess_count as f64).sqrt()
        } else {
            0.0
        };
        let gouge_rms = if self.gouge_count > 0 {
            (self.gouge_sum_sq / self.gouge_count as f64).sqrt()
        } else {
            0.0
        };

        let total_err_vol = excess_vol_mm3 + gouge_vol_mm3;
        let err_pct = if target_vol_mm3 > 1e-6 {
            100.0 * total_err_vol / target_vol_mm3
        } else {
            0.0
        };
        let intended_removable = (initial_stock_vol_mm3 - target_vol_mm3).max(0.0);
        let efficiency = if intended_removable > 1e-6 {
            (100.0 * (1.0 - (excess_vol_mm3 / intended_removable))).clamp(0.0, 100.0)
        } else {
            100.0
        };

        StockErrorReport {
            tolerance_mm: self.tolerance_mm,
            target_volume_mm3: target_vol_mm3,
            excess_volume_mm3: excess_vol_mm3,
            gouge_volume_mm3: gouge_vol_mm3,
            total_error_volume_mm3: total_err_vol,
            error_volume_pct_of_target: err_pct,
            material_removal_efficiency_pct: efficiency,
            sample_count: n,
            rms_error_mm: rms,
            excess_rms_mm: excess_rms,
            gouge_rms_mm: gouge_rms,
            mean_abs_error_mm: mae,
            mean_signed_error_mm: mean_signed,
            p95_abs_error_mm: p95,
            p99_abs_error_mm: p99,
            max_excess_mm: self.max_excess,
            max_excess_location_wcs: self.max_excess_loc,
            max_gouge_mm: self.max_gouge,
            max_gouge_location_wcs: self.max_gouge_loc,
            in_tolerance_surface_pct: in_tol_pct,
            per_operation,
        }
    }
}

/// Evaluates the signed deviation (`+` = uncut excess, `-` = gouge) at a target boundary endpoint
/// `target_coord` (`upper_face = true` for `max_mm`, `false` for `min_mm`) against `machined_spans`.
#[inline]
pub(crate) fn signed_deviation_at_target_endpoint(
    machined_spans: &[DexelSpan],
    target_span: DexelSpan,
    upper_face: bool,
) -> f64 {
    if upper_face {
        let t_hi = target_span.max_mm;
        for ms in machined_spans {
            if t_hi >= ms.min_mm - 1e-9 && t_hi <= ms.max_mm + 1e-9 {
                return (ms.max_mm - t_hi).max(0.0);
            }
        }
        // Machined stock does not cover t_hi -> gouged below t_hi
        let nearest_below = machined_spans
            .iter()
            .filter(|ms| ms.max_mm < t_hi)
            .map(|ms| ms.max_mm)
            .fold(target_span.min_mm, f64::max);
        -(t_hi - nearest_below).max(0.0)
    } else {
        let t_lo = target_span.min_mm;
        for ms in machined_spans {
            if t_lo >= ms.min_mm - 1e-9 && t_lo <= ms.max_mm + 1e-9 {
                return (t_lo - ms.min_mm).max(0.0);
            }
        }
        let nearest_above = machined_spans
            .iter()
            .filter(|ms| ms.min_mm > t_lo)
            .map(|ms| ms.min_mm)
            .fold(target_span.max_mm, f64::min);
        -(nearest_above - t_lo).max(0.0)
    }
}

/// Evaluates the signed deviation (`+` = excess outside target, `-` = gouge inside target) of a
/// machined surface point `(coord, want_min)` on a single ray against `target_spans` (used for
/// Tri-Dexel error heatmap vertex coloring!).
#[inline]
pub(crate) fn signed_deviation_at_machined_endpoint(
    target_spans: &[DexelSpan],
    machined_coord: f64,
    is_min_face: bool,
) -> f64 {
    if target_spans.is_empty() {
        // Pure excess stock outside target silhouette
        return 10.0;
    }
    // If machined_coord lies strictly inside a target span, it is a GOUGE (negative)
    for ts in target_spans {
        if machined_coord > ts.min_mm && machined_coord < ts.max_mm {
            let depth = if is_min_face {
                machined_coord - ts.min_mm
            } else {
                ts.max_mm - machined_coord
            };
            return -depth
                .min(machined_coord - ts.min_mm)
                .min(ts.max_mm - machined_coord);
        }
    }
    // Otherwise it lies outside target spans -> positive EXCESS distance to nearest target boundary
    let mut min_dist = f64::INFINITY;
    for ts in target_spans {
        let d = if machined_coord <= ts.min_mm {
            ts.min_mm - machined_coord
        } else {
            machined_coord - ts.max_mm
        };
        if d < min_dist {
            min_dist = d;
        }
    }
    if min_dist.is_finite() { min_dist } else { 0.0 }
}

#[inline]
fn dist_to_air_1d(spans: &[DexelSpan], coord: f64) -> f64 {
    for s in spans {
        if coord < s.min_mm {
            return 0.0;
        }
        if coord <= s.max_mm {
            return (coord - s.min_mm).min(s.max_mm - coord);
        }
    }
    0.0
}

#[inline]
fn dist_to_solid_1d(spans: &[DexelSpan], coord: f64) -> f64 {
    if spans.is_empty() {
        return f64::INFINITY;
    }
    let mut best = f64::INFINITY;
    for s in spans {
        if coord >= s.min_mm && coord <= s.max_mm {
            return 0.0;
        }
        let d = if coord < s.min_mm {
            s.min_mm - coord
        } else {
            coord - s.max_mm
        };
        if d < best {
            best = d;
        }
    }
    best
}

#[inline]
fn clamp_grid_idx(coord: f64, origin: f64, step: f64, len: usize) -> usize {
    if len == 0 {
        return 0;
    }
    (((coord - origin) / step).floor() as isize).clamp(0, (len - 1) as isize) as usize
}

/// Refines a 1D ray deviation `d_primary` at target boundary point `p` (`axis`: 0=X, 1=Y, 2=Z)
/// using the two orthogonal machined dexel planes so rays grazing parallel to a vertical wall or
/// flat floor measure true 3D surface distance instead of wall height / span length.
#[inline]
fn refine_tridexel_signed_deviation(
    machined: &TriDexelStock,
    p: DVec3,
    axis: u8,
    upper_face: bool,
    ts: DexelSpan,
    d_primary: f64,
) -> f64 {
    let pitch = machined.mean_pitch_mm();
    if d_primary.abs() <= 0.5 * pitch {
        return d_primary;
    }

    let dir = if upper_face { 1.0 } else { -1.0 };
    let mut probe = p;
    if d_primary > 0.0 {
        // Excess: step slightly outward into the supposed excess stock
        let shift = 0.25 * pitch;
        match axis {
            0 => probe.x += dir * shift,
            1 => probe.y += dir * shift,
            _ => probe.z += dir * shift,
        }
        let ix = clamp_grid_idx(probe.x, machined.origin_mm.x, machined.dx_mm, machined.nx);
        let iy = clamp_grid_idx(probe.y, machined.origin_mm.y, machined.dy_mm, machined.ny);
        let iz = clamp_grid_idx(probe.z, machined.origin_mm.z, machined.dz_mm, machined.nz);
        let (o1, o2) = match axis {
            0 => (
                dist_to_air_1d(machined.y_rays.ray(ix, iz), probe.y),
                dist_to_air_1d(machined.z_rays.ray(ix, iy), probe.z),
            ),
            1 => (
                dist_to_air_1d(machined.x_rays.ray(iy, iz), probe.x),
                dist_to_air_1d(machined.z_rays.ray(ix, iy), probe.z),
            ),
            _ => (
                dist_to_air_1d(machined.x_rays.ray(iy, iz), probe.x),
                dist_to_air_1d(machined.y_rays.ray(ix, iz), probe.y),
            ),
        };
        d_primary.min(o1).min(o2)
    } else {
        // Gouge: step slightly inward into the target solid span
        let shift = (0.25 * pitch).min(0.45 * ts.len_mm());
        match axis {
            0 => probe.x -= dir * shift,
            1 => probe.y -= dir * shift,
            _ => probe.z -= dir * shift,
        }
        let ix = clamp_grid_idx(probe.x, machined.origin_mm.x, machined.dx_mm, machined.nx);
        let iy = clamp_grid_idx(probe.y, machined.origin_mm.y, machined.dy_mm, machined.ny);
        let iz = clamp_grid_idx(probe.z, machined.origin_mm.z, machined.dz_mm, machined.nz);
        let (o1, o2) = match axis {
            0 => (
                dist_to_solid_1d(machined.y_rays.ray(ix, iz), probe.y),
                dist_to_solid_1d(machined.z_rays.ray(ix, iy), probe.z),
            ),
            1 => (
                dist_to_solid_1d(machined.x_rays.ray(iy, iz), probe.x),
                dist_to_solid_1d(machined.z_rays.ray(ix, iy), probe.z),
            ),
            _ => (
                dist_to_solid_1d(machined.x_rays.ray(iy, iz), probe.x),
                dist_to_solid_1d(machined.y_rays.ray(ix, iz), probe.y),
            ),
        };
        -((-d_primary).min(o1).min(o2))
    }
}

/// Computes exact 3D volumetric & surface-deviation [`StockErrorReport`] comparing a machined
/// [`TriDexelStock`] against its aligned target [`TriDexelStock`].
pub fn compute_tridexel_error_report(
    machined: &TriDexelStock,
    target: &TriDexelStock,
    initial_stock_vol_mm3: f64,
    tolerance_mm: f64,
    per_operation: Vec<OperationErrorSnapshot>,
) -> StockErrorReport {
    let target_vol = target.compute_volume_mm3();
    let (ex_x, ex_y, ex_z, g_x, g_y, g_z) = compute_tridexel_axis_error_volumes(machined, target);
    let excess_vol = (ex_x + ex_y + ex_z) / 3.0;
    let gouge_vol = (g_x + g_y + g_z) / 3.0;

    let est_samples = (target.nx * target.ny + target.ny * target.nz + target.nx * target.nz) / 2;
    let mut acc = DeviationStatsAcc::new(tolerance_mm, est_samples);

    // Sample Z-ray target endpoints (top & bottom faces)
    for j in 0..target.ny {
        let py = target.z_rays.center_v(j);
        for i in 0..target.nx {
            let t_spans = target.z_rays.ray(i, j);
            if t_spans.is_empty() {
                continue;
            }
            let px = target.z_rays.center_u(i);
            let m_spans = machined.z_rays.ray(i, j);
            for &ts in t_spans {
                let p_hi = DVec3::new(px, py, ts.max_mm);
                let d_hi = refine_tridexel_signed_deviation(
                    machined,
                    p_hi,
                    2,
                    true,
                    ts,
                    signed_deviation_at_target_endpoint(m_spans, ts, true),
                );
                acc.push(d_hi, p_hi);
                let p_lo = DVec3::new(px, py, ts.min_mm);
                let d_lo = refine_tridexel_signed_deviation(
                    machined,
                    p_lo,
                    2,
                    false,
                    ts,
                    signed_deviation_at_target_endpoint(m_spans, ts, false),
                );
                acc.push(d_lo, p_lo);
            }
        }
    }

    // Sample X-ray target endpoints (vertical X walls & undercuts)
    for k in 0..target.nz {
        let pz = target.x_rays.center_v(k);
        for j in 0..target.ny {
            let t_spans = target.x_rays.ray(j, k);
            if t_spans.is_empty() {
                continue;
            }
            let py = target.x_rays.center_u(j);
            let m_spans = machined.x_rays.ray(j, k);
            for &ts in t_spans {
                let p_hi = DVec3::new(ts.max_mm, py, pz);
                let d_hi = refine_tridexel_signed_deviation(
                    machined,
                    p_hi,
                    0,
                    true,
                    ts,
                    signed_deviation_at_target_endpoint(m_spans, ts, true),
                );
                acc.push(d_hi, p_hi);
                let p_lo = DVec3::new(ts.min_mm, py, pz);
                let d_lo = refine_tridexel_signed_deviation(
                    machined,
                    p_lo,
                    0,
                    false,
                    ts,
                    signed_deviation_at_target_endpoint(m_spans, ts, false),
                );
                acc.push(d_lo, p_lo);
            }
        }
    }

    // Sample Y-ray target endpoints (vertical Y walls & undercuts)
    for k in 0..target.nz {
        let pz = target.y_rays.center_v(k);
        for i in 0..target.nx {
            let t_spans = target.y_rays.ray(i, k);
            if t_spans.is_empty() {
                continue;
            }
            let px = target.y_rays.center_u(i);
            let m_spans = machined.y_rays.ray(i, k);
            for &ts in t_spans {
                let p_hi = DVec3::new(px, ts.max_mm, pz);
                let d_hi = refine_tridexel_signed_deviation(
                    machined,
                    p_hi,
                    1,
                    true,
                    ts,
                    signed_deviation_at_target_endpoint(m_spans, ts, true),
                );
                acc.push(d_hi, p_hi);
                let p_lo = DVec3::new(px, ts.min_mm, pz);
                let d_lo = refine_tridexel_signed_deviation(
                    machined,
                    p_lo,
                    1,
                    false,
                    ts,
                    signed_deviation_at_target_endpoint(m_spans, ts, false),
                );
                acc.push(d_lo, p_lo);
            }
        }
    }

    acc.finish(
        initial_stock_vol_mm3,
        target_vol,
        excess_vol,
        gouge_vol,
        per_operation,
    )
}

/// Fast per-operation snapshot helper returning `(excess_vol, gouge_vol, rms_mm)` for Tri-Dexel.
pub(crate) fn compute_tridexel_quick_error_summary(
    machined: &TriDexelStock,
    target: &TriDexelStock,
) -> (f64, f64, f64) {
    let report = compute_tridexel_error_report(machined, target, 0.0, 0.05, Vec::new());
    (
        report.excess_volume_mm3,
        report.gouge_volume_mm3,
        report.rms_error_mm,
    )
}

fn compute_tridexel_axis_error_volumes(
    machined: &TriDexelStock,
    target: &TriDexelStock,
) -> (f64, f64, f64, f64, f64, f64) {
    let plane_diffs =
        |m: &crate::tridexel::DexelPlane, t: &crate::tridexel::DexelPlane| -> (f64, f64) {
            let area = m.du_mm * m.dv_mm;
            let mut ex_len = 0.0;
            let mut g_len = 0.0;
            for (mr, tr) in m.rays.iter().zip(t.rays.iter()) {
                ex_len += spans_difference_length(mr, tr);
                g_len += spans_difference_length(tr, mr);
            }
            (ex_len * area, g_len * area)
        };
    let (ex_x, g_x) = plane_diffs(&machined.x_rays, &target.x_rays);
    let (ex_y, g_y) = plane_diffs(&machined.y_rays, &target.y_rays);
    let (ex_z, g_z) = plane_diffs(&machined.z_rays, &target.z_rays);
    (ex_x, ex_y, ex_z, g_x, g_y, g_z)
}

/// Computes top-projected 2.5D [`StockErrorReport`] from a [`HeightfieldStock`] whose `z_target`
/// has been rasterized against the target B-Rep/mesh solid.
///
/// Returns `None` if no target cells were rasterized (e.g., Pure-2D jobs with no 3D target model).
pub fn compute_heightmap_error_report(
    stock: &HeightfieldStock,
    initial_stock_vol_mm3: f64,
    tolerance_mm: f64,
    per_operation: Vec<OperationErrorSnapshot>,
) -> Option<StockErrorReport> {
    let cell_area = stock.dx_mm * stock.dy_mm;
    let mut target_vol = 0.0;
    let mut excess_vol = 0.0;
    let mut gouge_vol = 0.0;
    let mut acc = DeviationStatsAcc::new(tolerance_mm, stock.nx * stock.ny / 2);

    for j in 0..stock.ny {
        let py = stock.origin_y_mm + (j as f64 + 0.5) * stock.dy_mm;
        let row = j * stock.nx;
        for i in 0..stock.nx {
            let idx = row + i;
            if !stock.in_stock[idx] {
                continue;
            }
            let zb = f64::from(stock.z_bottom[idx]);
            let zm = if stock.z_stock[idx] > stock.z_bottom[idx] {
                f64::from(stock.z_stock[idx])
            } else {
                zb
            };
            let zt_f = stock.z_target[idx];
            if zt_f.is_finite() && f64::from(zt_f) > zb + 1e-6 {
                let zt = f64::from(zt_f);
                let px = stock.origin_x_mm + (i as f64 + 0.5) * stock.dx_mm;
                target_vol += (zt - zb) * cell_area;
                let dev = zm - zt;
                if dev > 0.0 {
                    excess_vol += dev * cell_area;
                } else {
                    gouge_vol += (-dev) * cell_area;
                }
                acc.push(dev, DVec3::new(px, py, zt));
            } else if zm > zb + 1e-6 {
                excess_vol += (zm - zb) * cell_area;
            }
        }
    }

    if acc.abs_samples.is_empty() {
        return None;
    }

    Some(acc.finish(
        initial_stock_vol_mm3,
        target_vol,
        excess_vol,
        gouge_vol,
        per_operation,
    ))
}

/// Fast per-operation heightmap summary `(excess_vol, gouge_vol, rms_mm)` for `per_operation` table.
pub(crate) fn compute_heightmap_quick_error_summary(
    stock: &HeightfieldStock,
) -> Option<(f64, f64, f64)> {
    let cell_area = stock.dx_mm * stock.dy_mm;
    let mut excess_vol = 0.0;
    let mut gouge_vol = 0.0;
    let mut sum_sq = 0.0;
    let mut cnt = 0usize;

    for idx in 0..stock.in_stock.len() {
        if !stock.in_stock[idx] {
            continue;
        }
        let zb = f64::from(stock.z_bottom[idx]);
        let zm = if stock.z_stock[idx] > stock.z_bottom[idx] {
            f64::from(stock.z_stock[idx])
        } else {
            zb
        };
        let zt_f = stock.z_target[idx];
        if zt_f.is_finite() && f64::from(zt_f) > zb + 1e-6 {
            let dev = zm - f64::from(zt_f);
            if dev > 0.0 {
                excess_vol += dev * cell_area;
            } else {
                gouge_vol += (-dev) * cell_area;
            }
            sum_sq += dev * dev;
            cnt += 1;
        } else if zm > zb + 1e-6 {
            excess_vol += (zm - zb) * cell_area;
        }
    }
    if cnt == 0 {
        None
    } else {
        Some((excess_vol, gouge_vol, (sum_sq / cnt as f64).sqrt()))
    }
}

/// Computes polar/cylindrical 4th-axis [`StockErrorReport`] from a [`CylindricalStock`] whose
/// `r_target` has been rasterized against the target B-Rep/mesh solid.
///
/// Returns `None` if no target cells were rasterized (e.g., empty CAD document with no 3D model).
pub fn compute_cylindrical_error_report(
    stock: &CylindricalStock,
    initial_stock_vol_mm3: f64,
    tolerance_mm: f64,
    per_operation: Vec<OperationErrorSnapshot>,
) -> Option<StockErrorReport> {
    let factor = 0.5 * stock.du * stock.dtheta;
    let mut target_vol = 0.0;
    let mut excess_vol = 0.0;
    let mut gouge_vol = 0.0;
    let mut acc = DeviationStatsAcc::new(tolerance_mm, stock.nu * stock.ntheta / 2);

    for iu in 0..stock.nu {
        let u = stock.u_min + (iu as f64 + 0.5) * stock.du;
        let row = iu * stock.ntheta;
        for it in 0..stock.ntheta {
            let idx = row + it;
            let rm = f64::from(stock.r_stock[idx]).max(0.0);
            let rt_f = stock.r_target[idx];
            if rt_f >= 0.0 && rt_f.is_finite() {
                let rt = f64::from(rt_f);
                target_vol += factor * rt * rt;
                let dev = rm - rt;
                if dev > 0.0 {
                    excess_vol += factor * (rm * rm - rt * rt);
                } else {
                    gouge_vol += factor * (rt * rt - rm * rm);
                }
                let theta = it as f64 * stock.dtheta;
                let loc = stock.rotary.cylindrical_to_wcs(u, theta, rt);
                acc.push(dev, loc);
            } else if rm > 1e-6 {
                excess_vol += factor * rm * rm;
            }
        }
    }

    if acc.abs_samples.is_empty() {
        return None;
    }

    Some(acc.finish(
        initial_stock_vol_mm3,
        target_vol,
        excess_vol,
        gouge_vol,
        per_operation,
    ))
}

/// Fast per-operation polar/cylindrical summary `(excess_vol, gouge_vol, rms_mm)` for `per_operation` table.
pub(crate) fn compute_cylindrical_quick_error_summary(
    stock: &CylindricalStock,
) -> Option<(f64, f64, f64)> {
    let factor = 0.5 * stock.du * stock.dtheta;
    let mut excess_vol = 0.0;
    let mut gouge_vol = 0.0;
    let mut sum_sq = 0.0;
    let mut cnt = 0usize;

    for idx in 0..stock.r_stock.len() {
        let rm = f64::from(stock.r_stock[idx]).max(0.0);
        let rt_f = stock.r_target[idx];
        if rt_f >= 0.0 && rt_f.is_finite() {
            let rt = f64::from(rt_f);
            let dev = rm - rt;
            if dev > 0.0 {
                excess_vol += factor * (rm * rm - rt * rt);
            } else {
                gouge_vol += factor * (rt * rt - rm * rm);
            }
            sum_sq += dev * dev;
            cnt += 1;
        } else if rm > 1e-6 {
            excess_vol += factor * rm * rm;
        }
    }
    if cnt == 0 {
        None
    } else {
        Some((excess_vol, gouge_vol, (sum_sq / cnt as f64).sqrt()))
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cam::ResolvedStockGeometry;
    use chip_math::Aabb3;

    use crate::{SimResolution, UNCUT_OP_IDX, tridexel::subtract_dexel_interval};

    use super::*;

    #[test]
    fn analytical_uniform_offset_and_deliberate_gouge_error_metrics_are_exact() {
        // Target block: 40 × 30 × 10 mm (`X ∈ [1, 41], Y ∈ [1, 31], Z ∈ [1, 11]`), vol = 12,000 mm³
        // Raw stock:    42 × 32 × 12 mm (`X ∈ [0, 42], Y ∈ [0, 32], Z ∈ [0, 12]`), vol = 16,128 mm³
        // Uniform 1.0 mm excess stock on ALL 6 faces -> exact RMS = 1.0 mm, MAE = 1.0 mm,
        // mean_signed = +1.0 mm, excess_vol = 4,128 mm³, gouge_vol = 0.0 mm³!
        let stock_aabb = Aabb3::new(DVec3::ZERO, DVec3::new(42.0, 32.0, 12.0));
        let target_aabb = Aabb3::new(DVec3::splat(1.0), DVec3::new(41.0, 31.0, 11.0));
        let stock_geom = ResolvedStockGeometry::Box {
            aabb_wcs: stock_aabb,
        };
        let machined =
            TriDexelStock::from_setup_stock(&stock_geom, SimResolution::Standard).unwrap();

        let mut target = TriDexelStock::empty_like(&machined);
        for j in 0..target.ny {
            let py = target.z_rays.center_v(j);
            for i in 0..target.nx {
                let px = target.z_rays.center_u(i);
                if px >= target_aabb.min.x
                    && px <= target_aabb.max.x
                    && py >= target_aabb.min.y
                    && py <= target_aabb.max.y
                {
                    target.z_rays.ray_mut(i, j).push(DexelSpan::new(
                        target_aabb.min.z,
                        target_aabb.max.z,
                        UNCUT_OP_IDX,
                        UNCUT_OP_IDX,
                    ));
                }
            }
        }
        for k in 0..target.nz {
            let pz = target.x_rays.center_v(k);
            for j in 0..target.ny {
                let py = target.x_rays.center_u(j);
                if py >= target_aabb.min.y
                    && py <= target_aabb.max.y
                    && pz >= target_aabb.min.z
                    && pz <= target_aabb.max.z
                {
                    target.x_rays.ray_mut(j, k).push(DexelSpan::new(
                        target_aabb.min.x,
                        target_aabb.max.x,
                        UNCUT_OP_IDX,
                        UNCUT_OP_IDX,
                    ));
                }
            }
        }
        for k in 0..target.nz {
            let pz = target.y_rays.center_v(k);
            for i in 0..target.nx {
                let px = target.y_rays.center_u(i);
                if px >= target_aabb.min.x
                    && px <= target_aabb.max.x
                    && pz >= target_aabb.min.z
                    && pz <= target_aabb.max.z
                {
                    target.y_rays.ray_mut(i, k).push(DexelSpan::new(
                        target_aabb.min.y,
                        target_aabb.max.y,
                        UNCUT_OP_IDX,
                        UNCUT_OP_IDX,
                    ));
                }
            }
        }

        let initial_vol = machined.compute_volume_mm3();
        let rep = compute_tridexel_error_report(&machined, &target, initial_vol, 0.05, Vec::new());

        assert!(
            (rep.rms_error_mm - 1.0).abs() < 1e-4,
            "rms={}",
            rep.rms_error_mm
        );
        assert!((rep.mean_abs_error_mm - 1.0).abs() < 1e-4);
        assert!((rep.mean_signed_error_mm - 1.0).abs() < 1e-4);
        assert!((rep.p95_abs_error_mm - 1.0).abs() < 1e-4);
        assert!((rep.max_excess_mm - 1.0).abs() < 1e-4);
        assert!(rep.gouge_volume_mm3 < 1e-6);
        assert!(rep.max_gouge_mm < 1e-6);
        assert!(rep.in_tolerance_surface_pct < 1e-6);
        assert!(
            (rep.excess_volume_mm3 - (42.0 * 32.0 * 12.0 - 40.0 * 30.0 * 10.0)).abs() < 150.0,
            "excess_vol={}",
            rep.excess_volume_mm3
        );

        // Now inject a deliberate 0.5 mm overcut across the top face (`z ∈ [10.5, 12.0]`) of `target`
        let mut gouged = target.clone();
        for ray in &mut gouged.z_rays.rays {
            subtract_dexel_interval(ray, 10.5, 12.0, 0);
        }
        for k in 0..gouged.nz {
            if gouged.x_rays.center_v(k) > 10.5 {
                for j in 0..gouged.ny {
                    gouged.x_rays.ray_mut(j, k).clear();
                }
                for i in 0..gouged.nx {
                    gouged.y_rays.ray_mut(i, k).clear();
                }
            }
        }
        let rep_g = compute_tridexel_error_report(&gouged, &target, initial_vol, 0.05, Vec::new());
        assert!(
            rep_g.gouge_volume_mm3 > 100.0,
            "gouge_vol={}",
            rep_g.gouge_volume_mm3
        );
        assert!(
            (rep_g.max_gouge_mm - 0.5).abs() < 1e-4,
            "max_gouge={}",
            rep_g.max_gouge_mm
        );
        assert!(rep_g.mean_signed_error_mm < 0.0);
        assert!(rep_g.max_gouge_location_wcs.is_some());
    }

    #[test]
    fn cylindrical_error_report_matches_analytical_excess_and_gouge_shells() {
        use std::f64::consts::PI;

        use chip_cad::CadDocument;
        use chip_cam::{
            AxisAnchor, BoxAnchor27, RotaryAxisKind, RotaryDriveMode, Setup, StockAxis,
            StockConfig, WcsOriginMode, WorkOffsetSlot,
        };
        use chip_math::{DisplayUnit, ParamLength};

        let mut setup = Setup::new("s_cyl", "Cyl Metrics", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27 {
                x: AxisAnchor::Center,
                y: AxisAnchor::Center,
                z: AxisAnchor::Center,
            },
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(42.0), // R_stock = 21 mm
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup.rotary.axis = RotaryAxisKind::XAxis;

        let doc = CadDocument::empty();
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let mut stock = CylindricalStock::from_setup(
            &resolved,
            resolved.rotary.unwrap(),
            SimResolution::Standard,
        )
        .unwrap();
        let initial_vol = stock.initial_volume_mm3();

        // With no target rasterized (`r_target == -1.0`), error report must be `None`.
        assert!(compute_cylindrical_error_report(&stock, initial_vol, 0.05, Vec::new()).is_none());

        // Set uniform cylindrical target R_target = 20.0 mm across entire length (uniform +1.0 mm excess shell).
        stock.r_target.fill(20.0_f32);
        let rep = compute_cylindrical_error_report(&stock, initial_vol, 0.05, Vec::new())
            .expect("error report must be populated when target exists");
        let exact_excess_vol = PI * (21.0_f64.powi(2) - 20.0_f64.powi(2)) * 80.0;
        assert!(
            (rep.rms_error_mm - 1.0).abs() < 1e-4,
            "rms={}",
            rep.rms_error_mm
        );
        assert!((rep.mean_abs_error_mm - 1.0).abs() < 1e-4);
        assert!((rep.mean_signed_error_mm - 1.0).abs() < 1e-4);
        assert!((rep.max_excess_mm - 1.0).abs() < 1e-4);
        assert!(rep.max_gouge_mm < 1e-6);
        assert!(rep.gouge_volume_mm3 < 1e-6);
        assert!(rep.in_tolerance_surface_pct < 1e-6);
        assert!(
            (rep.excess_volume_mm3 - exact_excess_vol).abs() < 1.0,
            "excess_vol={} vs exact={exact_excess_vol}",
            rep.excess_volume_mm3
        );

        // Machine cylinder cleanly to R = 20.0 mm except a single gouged band at R = 19.5 mm (-0.5 mm).
        stock.r_stock.fill(20.0_f32);
        let gouge_iu = stock.nu / 2;
        for it in 0..stock.ntheta {
            stock.r_stock[gouge_iu * stock.ntheta + it] = 19.5_f32;
        }
        let rep_g = compute_cylindrical_error_report(&stock, initial_vol, 0.05, Vec::new())
            .expect("gouged cylindrical error report");
        assert!((rep_g.max_gouge_mm - 0.5).abs() < 1e-4);
        assert!(rep_g.gouge_volume_mm3 > 10.0);
        assert!(rep_g.max_gouge_location_wcs.is_some());
        assert!(rep_g.in_tolerance_surface_pct > 99.0);
        assert!(rep_g.mean_signed_error_mm < 0.0);

        // Widening tolerance above 0.5 mm must bring 100% of surface in tolerance.
        let rep_wide = compute_cylindrical_error_report(&stock, initial_vol, 0.6, Vec::new())
            .expect("wide-tol report");
        assert!((rep_wide.in_tolerance_surface_pct - 100.0).abs() < 1e-6);
    }
}
