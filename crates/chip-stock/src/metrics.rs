use chip_math::DVec3;
use serde::{Deserialize, Serialize};

use crate::{
    cylindrical::CylindricalStock,
    heightfield::HeightfieldStock,
    tridexel::{DexelSpan, TriDexelStock, spans_difference_length},
    types::Axis3,
};

/// Visual coloring mode for simulated 3D stock meshes (`Heightfield`, `TriDexel`, and 4th-axis `CylindricalStock`).
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
        [
            0.22 + t * (0.12 - 0.22),
            0.80 + t * (0.28 - 0.80),
            0.94 + t * (0.88 - 0.94),
            1.0,
        ]
    } else {
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
    pub tolerance_mm: f64,
    pub target_volume_mm3: f64,
    pub excess_volume_mm3: f64,
    pub gouge_volume_mm3: f64,
    pub total_error_volume_mm3: f64,
    pub error_volume_pct_of_target: f64,
    pub material_removal_efficiency_pct: f64,
    pub sample_count: usize,
    pub rms_error_mm: f64,
    pub excess_rms_mm: f64,
    pub gouge_rms_mm: f64,
    pub mean_abs_error_mm: f64,
    pub mean_signed_error_mm: f64,
    pub p95_abs_error_mm: f64,
    pub p99_abs_error_mm: f64,
    pub max_excess_mm: f64,
    pub max_excess_location_wcs: Option<DVec3>,
    pub max_gouge_mm: f64,
    pub max_gouge_location_wcs: Option<DVec3>,
    pub in_tolerance_surface_pct: f64,
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
/// machined surface point `(coord, is_min_face)` on a single ray against `target_spans`.
#[inline]
pub(crate) fn signed_deviation_at_machined_endpoint(
    target_spans: &[DexelSpan],
    machined_coord: f64,
    is_min_face: bool,
) -> f64 {
    if target_spans.is_empty() {
        return 10.0;
    }
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

/// Axis-generic refinement of a 1D ray deviation `d_primary` at target boundary point `p` using
/// the two orthogonal machined dexel planes so rays grazing parallel to a vertical wall or flat
/// floor measure true 3D surface distance instead of wall height / span length.
#[inline]
fn refine_tridexel_signed_deviation(
    machined: &TriDexelStock,
    p: DVec3,
    axis: Axis3,
    upper_face: bool,
    ts: DexelSpan,
    d_primary: f64,
) -> f64 {
    let pitch = machined.mean_pitch_mm();
    if d_primary.abs() <= 0.5 * pitch {
        return d_primary;
    }

    let dir = if upper_face { 1.0 } else { -1.0 };
    let is_excess = d_primary > 0.0;
    let shift = if is_excess {
        0.25 * pitch
    } else {
        -(0.25 * pitch).min(0.45 * ts.len_mm())
    };
    let probe = axis.add_scaled(p, dir * shift);
    let idx_3d = [
        clamp_grid_idx(
            probe.x,
            machined.origin_wcs.x,
            machined.dx_mm(),
            machined.nx(),
        ),
        clamp_grid_idx(
            probe.y,
            machined.origin_wcs.y,
            machined.dy_mm(),
            machined.ny(),
        ),
        clamp_grid_idx(
            probe.z,
            machined.origin_wcs.z,
            machined.dz_mm(),
            machined.nz(),
        ),
    ];

    let [o_ax1, o_ax2] = axis.ortho_axes();
    let eval_ortho = |o_ax: Axis3| -> f64 {
        let iu = o_ax.u_axis().get_usize(idx_3d);
        let iv = o_ax.v_axis().get_usize(idx_3d);
        let spans = machined.plane(o_ax).ray_spans(iu, iv);
        let coord = o_ax.get(probe);
        if is_excess {
            dist_to_air_1d(&spans, coord)
        } else {
            dist_to_solid_1d(&spans, coord)
        }
    };

    let o1 = eval_ortho(o_ax1);
    let o2 = eval_ortho(o_ax2);
    if is_excess {
        d_primary.min(o1).min(o2)
    } else {
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
    let (excess_vol, gouge_vol) = compute_tridexel_error_volumes(machined, target);

    let [nx, ny, nz] = target.dims;
    let est_samples = (nx * ny + ny * nz + nx * nz) / 2;
    let mut acc = DeviationStatsAcc::new(tolerance_mm, est_samples);

    // Axis-generic sampling across all 3 orthogonal target dexel planes!
    for axis in [Axis3::Z, Axis3::X, Axis3::Y] {
        let t_plane = target.plane(axis);
        let m_plane = machined.plane(axis);
        for iv in 0..t_plane.nv {
            let pv = t_plane.center_v(iv);
            for iu in 0..t_plane.nu {
                let t_spans = t_plane.ray_spans(iu, iv);
                if t_spans.is_empty() {
                    continue;
                }
                let pu = t_plane.center_u(iu);
                let m_spans = m_plane.ray_spans(iu, iv);
                for &ts in t_spans.iter() {
                    let p_hi = axis.from_uvw(pu, pv, ts.max_mm);
                    let d_hi = refine_tridexel_signed_deviation(
                        machined,
                        p_hi,
                        axis,
                        true,
                        ts,
                        signed_deviation_at_target_endpoint(&m_spans, ts, true),
                    );
                    acc.push(d_hi, p_hi);

                    let p_lo = axis.from_uvw(pu, pv, ts.min_mm);
                    let d_lo = refine_tridexel_signed_deviation(
                        machined,
                        p_lo,
                        axis,
                        false,
                        ts,
                        signed_deviation_at_target_endpoint(&m_spans, ts, false),
                    );
                    acc.push(d_lo, p_lo);
                }
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
pub fn compute_tridexel_quick_error_summary(
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

fn compute_tridexel_error_volumes(machined: &TriDexelStock, target: &TriDexelStock) -> (f64, f64) {
    let mut ex_sum = 0.0;
    let mut g_sum = 0.0;
    for axis in Axis3::ALL {
        let m = machined.plane(axis);
        let t = target.plane(axis);
        let area = m.du_mm * m.dv_mm;
        let ray_count = m.ray_count().min(t.ray_count());
        let mut ex_len = 0.0;
        let mut g_len = 0.0;
        for r in 0..ray_count {
            let mr = m.ray_spans_by_index(r);
            let tr = t.ray_spans_by_index(r);
            ex_len += spans_difference_length(&mr, &tr);
            g_len += spans_difference_length(&tr, &mr);
        }
        ex_sum += ex_len * area;
        g_sum += g_len * area;
    }
    (ex_sum / 3.0, g_sum / 3.0)
}

/// Computes top-projected 2.5D [`StockErrorReport`] from a [`HeightfieldStock`] whose `z_target`
/// has been rasterized against the target B-Rep/mesh solid.
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
        let py = stock.origin_wcs.y + (j as f64 + 0.5) * stock.dy_mm;
        let row = j * stock.nx;
        for i in 0..stock.nx {
            let idx = row + i;
            let zb_f = stock.z_bottom[idx];
            if !zb_f.is_finite() {
                continue;
            }
            let zb = f64::from(zb_f);
            let zm = if stock.z_stock[idx] > zb_f {
                f64::from(stock.z_stock[idx])
            } else {
                zb
            };
            let zt_f = stock.z_target[idx];
            if zt_f.is_finite() && f64::from(zt_f) > zb + 1e-6 {
                let zt = f64::from(zt_f);
                let px = stock.origin_wcs.x + (i as f64 + 0.5) * stock.dx_mm;
                target_vol += (zt - zb) * cell_area;
                let dev = zm - zt;
                if dev > 0.0 {
                    excess_vol += dev * cell_area;
                } else {
                    gouge_vol += (-dev) * cell_area;
                }
                acc.push(dev, DVec3::new(px, py, stock.origin_wcs.z + zt));
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
pub fn compute_heightmap_quick_error_summary(stock: &HeightfieldStock) -> Option<(f64, f64, f64)> {
    let cell_area = stock.dx_mm * stock.dy_mm;
    let mut excess_vol = 0.0;
    let mut gouge_vol = 0.0;
    let mut sum_sq = 0.0;
    let mut cnt = 0usize;

    for idx in 0..stock.z_bottom.len() {
        let zb_f = stock.z_bottom[idx];
        if !zb_f.is_finite() {
            continue;
        }
        let zb = f64::from(zb_f);
        let zm = if stock.z_stock[idx] > zb_f {
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
pub fn compute_cylindrical_quick_error_summary(
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
    use std::f64::consts::PI;

    use chip_math::Aabb3;

    use crate::types::{RotaryStockSpec, SimResolution, StockShapeSpec, UNCUT_OP_IDX};

    use super::*;

    #[test]
    fn analytical_uniform_offset_and_deliberate_gouge_error_metrics_are_exact() {
        let stock_aabb = Aabb3::new(DVec3::ZERO, DVec3::new(42.0, 32.0, 12.0));
        let target_aabb = Aabb3::new(DVec3::splat(1.0), DVec3::new(41.0, 31.0, 11.0));
        let stock_geom = StockShapeSpec::Box {
            aabb_wcs: stock_aabb,
        };
        let machined =
            TriDexelStock::from_stock_spec(&stock_geom, SimResolution::Standard).unwrap();

        let mut target = TriDexelStock::empty_like(&machined);
        for axis in Axis3::ALL {
            let u_ax = axis.u_axis();
            let v_ax = axis.v_axis();
            let plane = target.plane_mut(axis);
            let nu = plane.nu;
            let nv = plane.nv;
            let span = DexelSpan::new(
                axis.get(target_aabb.min),
                axis.get(target_aabb.max),
                UNCUT_OP_IDX,
                UNCUT_OP_IDX,
            );
            for iv in 0..nv {
                let pv = plane.center_v(iv);
                if pv < v_ax.get(target_aabb.min) || pv > v_ax.get(target_aabb.max) {
                    continue;
                }
                for iu in 0..nu {
                    let pu = plane.center_u(iu);
                    if pu >= u_ax.get(target_aabb.min) && pu <= u_ax.get(target_aabb.max) {
                        plane.set_ray_spans(iu, iv, &[span]);
                    }
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

        // Inject a deliberate 0.5 mm overcut across the top face (`z ∈ [10.5, 12.0]`) of `target`
        let mut gouged = target.clone();
        let (nx, ny, nz) = (gouged.nx(), gouged.ny(), gouged.nz());
        for j in 0..ny {
            for i in 0..nx {
                gouged
                    .plane_mut(Axis3::Z)
                    .subtract_ray_interval(i, j, 10.5, 12.0, 0);
            }
        }
        for k in 0..nz {
            if gouged.x_plane().center_v(k) > 10.5 {
                for j in 0..ny {
                    gouged.plane_mut(Axis3::X).clear_ray(j, k);
                }
                for i in 0..nx {
                    gouged.plane_mut(Axis3::Y).clear_ray(i, k);
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
        let rotary = RotaryStockSpec {
            axis: Axis3::X,
            direction_sign: 1.0,
            axis_letter: 'A',
            center_wcs: DVec3::ZERO,
            stock_u_min: -40.0,
            stock_u_max: 40.0,
            stock_outer_radius_mm: 21.0,
            chuck_u_min: 0.0,
            chuck_u_max: 0.0,
            chuck_radius_mm: 0.0,
        };
        let stock_spec = StockShapeSpec::Cylinder {
            axis: Axis3::X,
            center_wcs: DVec3::ZERO,
            radius_mm: 21.0,
            min_axial_mm: -40.0,
            max_axial_mm: 40.0,
            aabb_wcs: Aabb3::new(
                DVec3::new(-40.0, -21.0, -21.0),
                DVec3::new(40.0, 21.0, 21.0),
            ),
        };
        let mut stock =
            CylindricalStock::from_stock_spec(&stock_spec, rotary, SimResolution::Standard)
                .unwrap();
        let initial_vol = stock.initial_volume_mm3();

        assert!(compute_cylindrical_error_report(&stock, initial_vol, 0.05, Vec::new()).is_none());

        stock.r_target.fill(20.0_f32);
        let rep = compute_cylindrical_error_report(&stock, initial_vol, 0.05, Vec::new())
            .expect("error report must be populated when target exists");
        let exact_excess_vol = PI * (21.0_f64.powi(2) - 20.0_f64.powi(2)) * 80.0;
        assert!((rep.rms_error_mm - 1.0).abs() < 1e-4);
        assert!((rep.mean_abs_error_mm - 1.0).abs() < 1e-4);
        assert!((rep.mean_signed_error_mm - 1.0).abs() < 1e-4);
        assert!((rep.max_excess_mm - 1.0).abs() < 1e-4);
        assert!(rep.max_gouge_mm < 1e-6);
        assert!(rep.gouge_volume_mm3 < 1e-6);
        assert!(rep.in_tolerance_surface_pct < 1e-6);
        assert!((rep.excess_volume_mm3 - exact_excess_vol).abs() < 1.0);

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

        let rep_wide = compute_cylindrical_error_report(&stock, initial_vol, 0.6, Vec::new())
            .expect("wide-tol report");
        assert!((rep_wide.in_tolerance_surface_pct - 100.0).abs() < 1e-6);
    }
}
