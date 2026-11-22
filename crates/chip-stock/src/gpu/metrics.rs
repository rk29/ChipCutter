//! GPU workgroup shared-memory parallel reduction metrology (`stock_metrics.wgsl`) computing
//! remaining volume, quick per-op summaries, full [`StockErrorReport`] metrics, and a 256-bin
//! logarithmic error histogram (`0.0005 mm .. 100.0 mm`) across `Heightfield`, `Cylindrical`,
//! and `TriDexel` GPU storage buffers without full-grid CPU readback.

use chip_math::DVec3;

use crate::{
    metrics::{OperationErrorSnapshot, StockErrorReport},
    types::RotaryStockSpec,
};

use super::{
    context::GpuSimContext,
    types::{
        GPU_METRICS_HIST_BINS, GPU_METRICS_MAX_WORKGROUPS, GpuMetricsPartial, MetricsUniforms,
    },
};

const HIST_MIN_MM: f64 = 0.0005;
const HIST_MAX_MM: f64 = 100.0;

/// 256-bin logarithmic surface error histogram (`bin 0`: `< 0.0005 mm`, `1..=254`: log-spaced
/// `[0.0005, 100.0] mm`, `bin 255`: `>= 100.0 mm`) supporting instant `P95`, `P99`, and
/// `in_tolerance_surface_pct(tol_mm)` evaluation across arbitrary tolerance thresholds.
#[derive(Debug, Clone)]
pub struct GpuErrorHistogram {
    pub bins: [u32; GPU_METRICS_HIST_BINS],
    pub total_samples: u64,
    pub max_abs_mm: f64,
}

impl GpuErrorHistogram {
    pub fn from_bins(bins: [u32; GPU_METRICS_HIST_BINS], max_abs_mm: f64) -> Self {
        let total_samples = bins.iter().map(|&c| u64::from(c)).sum();
        Self {
            bins,
            total_samples,
            max_abs_mm: max_abs_mm.max(0.0),
        }
    }

    #[inline]
    fn bin_bounds(bin: usize) -> (f64, f64) {
        if bin == 0 {
            return (0.0, HIST_MIN_MM);
        }
        if bin >= GPU_METRICS_HIST_BINS - 1 {
            return (HIST_MAX_MM, HIST_MAX_MM);
        }
        let log_min = HIST_MIN_MM.ln();
        let log_span = (HIST_MAX_MM / HIST_MIN_MM).ln();
        let t0 = (bin - 1) as f64 / (GPU_METRICS_HIST_BINS - 2) as f64;
        let t1 = bin as f64 / (GPU_METRICS_HIST_BINS - 2) as f64;
        (
            (log_min + t0 * log_span).exp(),
            (log_min + t1 * log_span).exp(),
        )
    }

    /// Estimates the `q`-th quantile (`q ∈ [0.0, 1.0]`, e.g. `0.95` or `0.99`) from the log histogram.
    pub fn quantile_mm(&self, q: f64) -> f64 {
        if self.total_samples == 0 {
            return 0.0;
        }
        let target_rank = ((self.total_samples as f64 - 1.0) * q.clamp(0.0, 1.0)).round() as u64;
        let mut cum: u64 = 0;
        for (bin, &cnt) in self.bins.iter().enumerate() {
            if cnt == 0 {
                continue;
            }
            let next = cum + u64::from(cnt);
            if target_rank < next {
                if bin == 0 {
                    return 0.0;
                }
                if bin >= GPU_METRICS_HIST_BINS - 1 {
                    return self.max_abs_mm.max(HIST_MAX_MM);
                }
                let (lo, hi) = Self::bin_bounds(bin);
                let frac = (target_rank - cum) as f64 / (cnt as f64).max(1.0);
                let est = (lo.ln() + frac * (hi.ln() - lo.ln())).exp();
                return est.min(self.max_abs_mm);
            }
            cum = next;
        }
        self.max_abs_mm
    }

    /// Estimates `in_tolerance_surface_pct` at an arbitrary tolerance threshold `tol_mm`.
    pub fn in_tolerance_surface_pct(&self, tol_mm: f64) -> f64 {
        if self.total_samples == 0 {
            return 100.0;
        }
        let tol = tol_mm.max(0.0);
        let mut in_tol: f64 = 0.0;
        for (bin, &cnt) in self.bins.iter().enumerate() {
            if cnt == 0 {
                continue;
            }
            let (lo, hi) = Self::bin_bounds(bin);
            if hi <= tol {
                in_tol += cnt as f64;
            } else if lo < tol && hi > lo {
                let frac = if bin == 0 {
                    (tol / hi).clamp(0.0, 1.0)
                } else {
                    ((tol.ln() - lo.ln()) / (hi.ln() - lo.ln())).clamp(0.0, 1.0)
                };
                in_tol += (cnt as f64) * frac;
            }
        }
        (100.0 * in_tol / (self.total_samples as f64)).clamp(0.0, 100.0)
    }
}

/// Aggregated `f64` reduction totals produced by `stock_metrics.wgsl`.
#[derive(Debug, Clone)]
pub(crate) struct ReducedMetricsTotals {
    pub(crate) remaining_vol_mm3: f64,
    pub(crate) target_vol_mm3: f64,
    pub(crate) excess_vol_mm3: f64,
    pub(crate) gouge_vol_mm3: f64,
    pub(crate) sum_sq: f64,
    pub(crate) sum_abs: f64,
    pub(crate) sum_signed: f64,
    pub(crate) excess_sum_sq: f64,
    pub(crate) gouge_sum_sq: f64,
    pub(crate) max_excess: f64,
    pub(crate) max_excess_loc: Option<DVec3>,
    pub(crate) max_gouge: f64,
    pub(crate) max_gouge_loc: Option<DVec3>,
    pub(crate) sample_count: usize,
    pub(crate) in_tol_count: usize,
    pub(crate) excess_count: usize,
    pub(crate) gouge_count: usize,
    pub(crate) histogram: GpuErrorHistogram,
}

impl ReducedMetricsTotals {
    pub(crate) fn quick_summary(&self) -> Option<(f64, f64, f64)> {
        if self.sample_count == 0 {
            return None;
        }
        let rms = (self.sum_sq / self.sample_count as f64).sqrt();
        Some((self.excess_vol_mm3, self.gouge_vol_mm3, rms))
    }

    pub(crate) fn into_error_report(
        self,
        initial_stock_vol_mm3: f64,
        tolerance_mm: f64,
        per_operation: Vec<OperationErrorSnapshot>,
        require_non_empty: bool,
    ) -> Option<StockErrorReport> {
        let n = self.sample_count;
        if require_non_empty && n == 0 {
            return None;
        }
        let tol = tolerance_mm.max(1e-6);
        let (rms, mae, mean_signed, p95, p99, in_tol_pct) = if n > 0 {
            (
                (self.sum_sq / n as f64).sqrt(),
                self.sum_abs / n as f64,
                self.sum_signed / n as f64,
                self.histogram.quantile_mm(0.95),
                self.histogram.quantile_mm(0.99),
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

        let total_err_vol = self.excess_vol_mm3 + self.gouge_vol_mm3;
        let err_pct = if self.target_vol_mm3 > 1e-6 {
            100.0 * total_err_vol / self.target_vol_mm3
        } else {
            0.0
        };
        let intended_removable = (initial_stock_vol_mm3 - self.target_vol_mm3).max(0.0);
        let efficiency = if intended_removable > 1e-6 {
            (100.0 * (1.0 - (self.excess_vol_mm3 / intended_removable))).clamp(0.0, 100.0)
        } else {
            100.0
        };

        Some(StockErrorReport {
            tolerance_mm: tol,
            target_volume_mm3: self.target_vol_mm3,
            excess_volume_mm3: self.excess_vol_mm3,
            gouge_volume_mm3: self.gouge_vol_mm3,
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
        })
    }
}

fn dispatch_count(total_elements: usize) -> u32 {
    if total_elements == 0 {
        return 0;
    }
    total_elements
        .div_ceil(256)
        .clamp(1, GPU_METRICS_MAX_WORKGROUPS) as u32
}

impl GpuSimContext {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn reduce_heightfield_metrics(
        &self,
        nx: usize,
        ny: usize,
        origin_wcs: DVec3,
        dx_mm: f64,
        dy_mm: f64,
        tolerance_mm: f64,
        volume_only: bool,
        z_stock_buf: &wgpu::Buffer,
        z_bottom_buf: &wgpu::Buffer,
        z_target_buf: &wgpu::Buffer,
    ) -> ReducedMetricsTotals {
        let num_wg = dispatch_count(nx * ny);
        if num_wg == 0 {
            return empty_totals();
        }
        let uniforms = MetricsUniforms {
            dims: [nx as u32, ny as u32, 0, if volume_only { 2 } else { 0 }],
            td_offsets: [0; 4],
            origin_and_tol: [
                origin_wcs.x as f32,
                origin_wcs.y as f32,
                origin_wcs.z as f32,
                tolerance_mm.max(1e-6) as f32,
            ],
            pitch_and_mean: [dx_mm as f32, dy_mm as f32, 0.0, 0.0],
            cyl_e1_pol: [0.0; 4],
            cyl_e2: [0.0; 4],
            cyl_u_vec: [0.0; 4],
            dispatch: [num_wg, 0, 0, 0],
        };
        let stock_bg = self.device().create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chip_metrics_hf_bg"),
            layout: &self.metrics_hf_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: z_stock_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: z_bottom_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: z_target_buf.as_entire_binding(),
                },
            ],
        });
        self.run_metrics_reduction(
            &self.metrics_hf_pipeline,
            &stock_bg,
            &uniforms,
            num_wg,
            !volume_only,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn reduce_cylindrical_metrics(
        &self,
        nu: usize,
        ntheta: usize,
        rotary: RotaryStockSpec,
        u_min: f64,
        du: f64,
        dtheta: f64,
        tolerance_mm: f64,
        volume_only: bool,
        r_stock_buf: &wgpu::Buffer,
        r_target_buf: &wgpu::Buffer,
    ) -> ReducedMetricsTotals {
        let num_wg = dispatch_count(nu * ntheta);
        if num_wg == 0 {
            return empty_totals();
        }
        let (e1, e2, u_vec) = rotary.basis();
        let u_center = rotary.center_wcs.dot(u_vec);
        let uniforms = MetricsUniforms {
            dims: [nu as u32, ntheta as u32, 0, if volume_only { 2 } else { 0 }],
            td_offsets: [0; 4],
            origin_and_tol: [
                rotary.center_wcs.x as f32,
                rotary.center_wcs.y as f32,
                rotary.center_wcs.z as f32,
                tolerance_mm.max(1e-6) as f32,
            ],
            pitch_and_mean: [du as f32, dtheta as f32, u_min as f32, u_center as f32],
            cyl_e1_pol: [
                e1.x as f32,
                e1.y as f32,
                e1.z as f32,
                rotary.direction_sign as f32,
            ],
            cyl_e2: [e2.x as f32, e2.y as f32, e2.z as f32, 0.0],
            cyl_u_vec: [u_vec.x as f32, u_vec.y as f32, u_vec.z as f32, 0.0],
            dispatch: [num_wg, 0, 0, 0],
        };
        let stock_bg = self.device().create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chip_metrics_cyl_bg"),
            layout: &self.metrics_cyl_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: r_stock_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: r_target_buf.as_entire_binding(),
                },
            ],
        });
        let mut totals = self.run_metrics_reduction(
            &self.metrics_cyl_pipeline,
            &stock_bg,
            &uniforms,
            num_wg,
            !volume_only,
        );
        // Mirror CPU `has_target` guard: if no target solid was rasterized (`r_target` all `-1.0`),
        // clear sample_count so `quick_summary()` and `into_error_report(..., true)` return `None`.
        if !volume_only && totals.target_vol_mm3 <= 1e-9 {
            totals.sample_count = 0;
        }
        totals
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn reduce_tridexel_metrics(
        &self,
        dims: [usize; 3],
        origin_wcs: DVec3,
        pitch_mm: DVec3,
        plane_ray_offsets: [usize; 3],
        total_rays: usize,
        has_target: bool,
        tolerance_mm: f64,
        volume_only: bool,
        m_slots_buf: &wgpu::Buffer,
        m_counts_buf: &wgpu::Buffer,
        t_slots_buf: &wgpu::Buffer,
        t_counts_buf: &wgpu::Buffer,
    ) -> ReducedMetricsTotals {
        let num_wg = dispatch_count(total_rays);
        if num_wg == 0 {
            return empty_totals();
        }
        let mut flags = 0u32;
        if has_target {
            flags |= 1;
        }
        if volume_only {
            flags |= 2;
        }
        let mean_pitch = (pitch_mm.x + pitch_mm.y + pitch_mm.z) / 3.0;
        let uniforms = MetricsUniforms {
            dims: [dims[0] as u32, dims[1] as u32, dims[2] as u32, flags],
            td_offsets: [
                plane_ray_offsets[0] as u32,
                plane_ray_offsets[1] as u32,
                plane_ray_offsets[2] as u32,
                total_rays as u32,
            ],
            origin_and_tol: [
                origin_wcs.x as f32,
                origin_wcs.y as f32,
                origin_wcs.z as f32,
                tolerance_mm.max(1e-6) as f32,
            ],
            pitch_and_mean: [
                pitch_mm.x as f32,
                pitch_mm.y as f32,
                pitch_mm.z as f32,
                mean_pitch as f32,
            ],
            cyl_e1_pol: [0.0; 4],
            cyl_e2: [0.0; 4],
            cyl_u_vec: [0.0; 4],
            dispatch: [num_wg, 0, 0, 0],
        };
        let stock_bg = self.device().create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chip_metrics_td_bg"),
            layout: &self.metrics_td_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: m_slots_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 6,
                    resource: m_counts_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 7,
                    resource: t_slots_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 8,
                    resource: t_counts_buf.as_entire_binding(),
                },
            ],
        });
        self.run_metrics_reduction(
            &self.metrics_td_pipeline,
            &stock_bg,
            &uniforms,
            num_wg,
            !volume_only && has_target,
        )
    }

    fn run_metrics_reduction(
        &self,
        pipeline: &wgpu::ComputePipeline,
        stock_bg: &wgpu::BindGroup,
        uniforms: &MetricsUniforms,
        num_wg: u32,
        read_hist: bool,
    ) -> ReducedMetricsTotals {
        let scratch = self
            .metrics_scratch
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let device = self.device();
        let queue = self.queue();

        queue.write_buffer(&scratch.uniform_buf, 0, bytemuck::bytes_of(uniforms));

        let partials_bytes = (num_wg as usize) * std::mem::size_of::<GpuMetricsPartial>();
        let hist_bytes = GPU_METRICS_HIST_BINS * std::mem::size_of::<u32>();

        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("chip_metrics_reduce_enc"),
        });
        if read_hist {
            enc.clear_buffer(&scratch.hist_buf, 0, None);
        }
        {
            let mut pass = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("chip_metrics_reduce_pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(pipeline);
            pass.set_bind_group(0, &scratch.common_bg, &[]);
            pass.set_bind_group(1, stock_bg, &[]);
            pass.dispatch_workgroups(num_wg, 1, 1);
        }
        enc.copy_buffer_to_buffer(
            &scratch.partials_buf,
            0,
            &scratch.partials_readback_buf,
            0,
            partials_bytes as u64,
        );
        if read_hist {
            enc.copy_buffer_to_buffer(
                &scratch.hist_buf,
                0,
                &scratch.hist_readback_buf,
                0,
                hist_bytes as u64,
            );
        }
        queue.submit(std::iter::once(enc.finish()));

        let mut partials = vec![GpuMetricsPartial::default(); num_wg as usize];
        let _ = self.readback_bytes(
            &scratch.partials_readback_buf,
            partials_bytes,
            bytemuck::cast_slice_mut(&mut partials),
        );
        let mut hist_bins = [0u32; GPU_METRICS_HIST_BINS];
        if read_hist {
            let _ = self.readback_bytes(
                &scratch.hist_readback_buf,
                hist_bytes,
                bytemuck::cast_slice_mut(&mut hist_bins),
            );
        }
        drop(scratch);

        let mut remaining_vol = 0.0_f64;
        let mut target_vol = 0.0_f64;
        let mut excess_vol = 0.0_f64;
        let mut gouge_vol = 0.0_f64;
        let mut sum_sq = 0.0_f64;
        let mut sum_abs = 0.0_f64;
        let mut sum_signed = 0.0_f64;
        let mut excess_sum_sq = 0.0_f64;
        let mut gouge_sum_sq = 0.0_f64;
        let mut max_excess = 0.0_f64;
        let mut max_excess_loc: Option<DVec3> = None;
        let mut max_gouge = 0.0_f64;
        let mut max_gouge_loc: Option<DVec3> = None;
        let mut sample_count = 0usize;
        let mut in_tol_count = 0usize;
        let mut excess_count = 0usize;
        let mut gouge_count = 0usize;

        for p in &partials {
            remaining_vol += f64::from(p.remaining_vol);
            target_vol += f64::from(p.target_vol);
            excess_vol += f64::from(p.excess_vol);
            gouge_vol += f64::from(p.gouge_vol);
            sum_sq += f64::from(p.sum_sq);
            sum_abs += f64::from(p.sum_abs);
            sum_signed += f64::from(p.sum_signed);
            excess_sum_sq += f64::from(p.excess_sum_sq);
            gouge_sum_sq += f64::from(p.gouge_sum_sq);
            sample_count += p.sample_count as usize;
            in_tol_count += p.in_tol_count as usize;
            excess_count += p.excess_count as usize;
            gouge_count += p.gouge_count as usize;

            let me = f64::from(p.max_excess);
            if me > max_excess {
                max_excess = me;
                max_excess_loc = Some(DVec3::new(
                    f64::from(p.max_excess_loc[0]),
                    f64::from(p.max_excess_loc[1]),
                    f64::from(p.max_excess_loc[2]),
                ));
            }
            let mg = f64::from(p.max_gouge);
            if mg > max_gouge {
                max_gouge = mg;
                max_gouge_loc = Some(DVec3::new(
                    f64::from(p.max_gouge_loc[0]),
                    f64::from(p.max_gouge_loc[1]),
                    f64::from(p.max_gouge_loc[2]),
                ));
            }
        }

        let histogram = GpuErrorHistogram::from_bins(hist_bins, max_excess.max(max_gouge));

        ReducedMetricsTotals {
            remaining_vol_mm3: remaining_vol,
            target_vol_mm3: target_vol,
            excess_vol_mm3: excess_vol,
            gouge_vol_mm3: gouge_vol,
            sum_sq,
            sum_abs,
            sum_signed,
            excess_sum_sq,
            gouge_sum_sq,
            max_excess,
            max_excess_loc,
            max_gouge,
            max_gouge_loc,
            sample_count,
            in_tol_count,
            excess_count,
            gouge_count,
            histogram,
        }
    }
}

fn empty_totals() -> ReducedMetricsTotals {
    ReducedMetricsTotals {
        remaining_vol_mm3: 0.0,
        target_vol_mm3: 0.0,
        excess_vol_mm3: 0.0,
        gouge_vol_mm3: 0.0,
        sum_sq: 0.0,
        sum_abs: 0.0,
        sum_signed: 0.0,
        excess_sum_sq: 0.0,
        gouge_sum_sq: 0.0,
        max_excess: 0.0,
        max_excess_loc: None,
        max_gouge: 0.0,
        max_gouge_loc: None,
        sample_count: 0,
        in_tol_count: 0,
        excess_count: 0,
        gouge_count: 0,
        histogram: GpuErrorHistogram::from_bins([0u32; GPU_METRICS_HIST_BINS], 0.0),
    }
}
