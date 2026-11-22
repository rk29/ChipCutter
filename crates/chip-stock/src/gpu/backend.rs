//! `WgpuComputeBackend` implementing [`SimStockBackend`] across all three volumetric stock
//! engines (`Heightfield`, `Cylindrical`, and `TriDexel`) with GPU parallel reduction metrology
//! and direct GPU BVH target rasterization.

use std::sync::Arc;

use chip_bvh::TriMesh3D;

use crate::{
    backend::{SimStockBackend, SimulatedStock, StockCheckpointSnapshot, StockModelKind},
    cylindrical::tessellate_cylindrical_stock_mesh_colored,
    heightfield::tessellate_stock_mesh_colored,
    metrics::{
        OperationErrorSnapshot, SimStockColorMode, StockErrorReport, compute_tridexel_error_report,
        compute_tridexel_quick_error_summary,
    },
    tridexel::{TriDexelStock, tessellate_tridexel_stock_mesh_colored},
    types::{ActiveSimBackendInfo, BatchSweepOutput, SimStockMesh, SweepChord, SweepToolSpec},
};

use super::{
    context::GpuSimContext, cylindrical::GpuCylindricalState, heightfield::GpuHeightfieldState,
    tridexel::GpuTriDexelState,
};

#[derive(Debug)]
enum GpuBackendInner {
    Heightfield(Box<GpuHeightfieldState>),
    Cylindrical(Box<GpuCylindricalState>),
    TriDexel(Box<GpuTriDexelState>),
}

/// Optional `wgpu` compute volumetric stock simulation backend.
///
/// Executes 2.5D `Heightfield`, 3D volumetric `TriDexel` (with lossless CPU overflow safety valve
/// for rare `> K = 4`-span rays), and 4th-axis `Cylindrical` stock removal, Rules 1–4
/// verification, and parallel reduction metrology end-to-end on the GPU.
#[derive(Debug)]
pub struct WgpuComputeBackend {
    ctx: Arc<GpuSimContext>,
    inner: GpuBackendInner,
}

impl WgpuComputeBackend {
    /// Uploads `stock` (and optional `target_tridexel`) into GPU storage buffers using `ctx`.
    pub fn new(
        ctx: Arc<GpuSimContext>,
        stock: SimulatedStock,
        target_tridexel: Option<TriDexelStock>,
    ) -> Self {
        let inner = match stock {
            SimulatedStock::Heightfield(hf) => GpuBackendInner::Heightfield(Box::new(
                GpuHeightfieldState::new(Arc::clone(&ctx), hf),
            )),
            SimulatedStock::Cylindrical(cyl) => GpuBackendInner::Cylindrical(Box::new(
                GpuCylindricalState::new(Arc::clone(&ctx), cyl),
            )),
            SimulatedStock::TriDexel(td) => GpuBackendInner::TriDexel(Box::new(
                GpuTriDexelState::new(Arc::clone(&ctx), *td, target_tridexel),
            )),
        };
        Self { ctx, inner }
    }

    #[inline]
    pub fn context(&self) -> &Arc<GpuSimContext> {
        &self.ctx
    }

    /// Rasterizes a WCS-space [`TriMesh3D`] target solid directly onto this backend's GPU target
    /// buffers using `bvh_distance.wgsl` (`FlatBvhNode` + `GpuTriangle` zero-conversion upload).
    pub fn rasterize_target_mesh_gpu(&mut self, target_mesh: &TriMesh3D) {
        match &mut self.inner {
            GpuBackendInner::Heightfield(hf) => hf.rasterize_target_mesh_gpu(target_mesh),
            GpuBackendInner::Cylindrical(c) => c.rasterize_target_mesh_gpu(target_mesh),
            GpuBackendInner::TriDexel(td) => td.rasterize_target_mesh_gpu(target_mesh),
        }
    }
}

impl SimStockBackend for WgpuComputeBackend {
    fn backend_info(&self) -> ActiveSimBackendInfo {
        self.ctx.adapter_info().clone()
    }

    fn kind(&self) -> StockModelKind {
        match &self.inner {
            GpuBackendInner::Heightfield(_) => StockModelKind::Heightfield25D,
            GpuBackendInner::Cylindrical(c) => StockModelKind::Cylindrical4Axis(c.rotary()),
            GpuBackendInner::TriDexel(_) => StockModelKind::TriDexel3D,
        }
    }

    fn capture_checkpoint(&self, completed_segments: usize) -> StockCheckpointSnapshot {
        match &self.inner {
            GpuBackendInner::Heightfield(hf) => {
                hf.with_synced_stock(|s| StockCheckpointSnapshot::Heightfield {
                    completed_segments,
                    z_stock: s.z_stock.clone(),
                    last_op_idx: s.last_op_idx.clone(),
                })
            }
            GpuBackendInner::Cylindrical(c) => {
                c.with_synced_stock(|s| StockCheckpointSnapshot::Cylindrical {
                    completed_segments,
                    r_stock: s.r_stock.clone(),
                    last_op_idx: s.last_op_idx.clone(),
                })
            }
            GpuBackendInner::TriDexel(td) => {
                td.with_synced_stock(|s| StockCheckpointSnapshot::TriDexel {
                    completed_segments,
                    stock: Box::new(s.clone()),
                })
            }
        }
    }

    fn reset_from_checkpoint(&mut self, cp: &StockCheckpointSnapshot) {
        match &mut self.inner {
            GpuBackendInner::Heightfield(hf) => hf.reset_from_checkpoint(cp),
            GpuBackendInner::Cylindrical(c) => c.reset_from_checkpoint(cp),
            GpuBackendInner::TriDexel(td) => td.reset_from_checkpoint(cp),
        }
    }

    fn sweep_batch(
        &mut self,
        chords: &[SweepChord],
        profiles: &[SweepToolSpec],
        verify: bool,
    ) -> BatchSweepOutput {
        match &mut self.inner {
            GpuBackendInner::Heightfield(hf) => hf.sweep_batch(chords, profiles, verify),
            GpuBackendInner::Cylindrical(c) => c.sweep_batch(chords, profiles, verify),
            GpuBackendInner::TriDexel(td) => td.sweep_batch(chords, profiles, verify),
        }
    }

    fn query_vertical_plunge_depth(
        &self,
        x_wcs: f64,
        y_wcs: f64,
        z_start_wcs: f64,
        z_end_wcs: f64,
        thread_check_radius_mm: Option<f64>,
    ) -> f64 {
        match &self.inner {
            GpuBackendInner::Heightfield(hf) => hf.with_synced_stock(|s| {
                let stock_z = if let Some(check_r) = thread_check_radius_mm {
                    s.sample_max_stock_z_in_disk(x_wcs, y_wcs, check_r)
                } else {
                    s.sample_stock_z(x_wcs, y_wcs)
                };
                stock_z.map_or(0.0, |sz| sz.min(z_start_wcs) - z_end_wcs)
            }),
            GpuBackendInner::Cylindrical(_) => 0.0,
            GpuBackendInner::TriDexel(td) => td.with_synced_stock(|s| {
                if let Some(check_r) = thread_check_radius_mm {
                    s.query_vertical_plunge_overlap(x_wcs, y_wcs, check_r, z_end_wcs, z_start_wcs)
                } else {
                    s.sample_top_z(x_wcs, y_wcs)
                        .map_or(0.0, |top_z| top_z.min(z_start_wcs) - z_end_wcs)
                }
            }),
        }
    }

    fn compute_volume_mm3(&self) -> f64 {
        match &self.inner {
            GpuBackendInner::Heightfield(hf) => {
                self.ctx
                    .reduce_heightfield_metrics(
                        hf.nx(),
                        hf.ny(),
                        hf.origin_wcs(),
                        hf.dx_mm(),
                        hf.dy_mm(),
                        0.05,
                        true,
                        hf.z_stock_buf(),
                        hf.z_bottom_buf(),
                        hf.z_target_buf(),
                    )
                    .remaining_vol_mm3
            }
            GpuBackendInner::Cylindrical(c) => {
                self.ctx
                    .reduce_cylindrical_metrics(
                        c.nu(),
                        c.ntheta(),
                        c.rotary(),
                        c.u_min(),
                        c.du(),
                        c.dtheta(),
                        0.05,
                        true,
                        c.r_stock_buf(),
                        c.r_target_buf(),
                    )
                    .remaining_vol_mm3
            }
            GpuBackendInner::TriDexel(td) => {
                let inline_vol = self
                    .ctx
                    .reduce_tridexel_metrics(
                        td.dims(),
                        td.origin_wcs(),
                        td.pitch_mm(),
                        td.plane_ray_offsets(),
                        td.total_rays(),
                        td.has_target(),
                        0.05,
                        true,
                        td.m_slots_buf(),
                        td.m_counts_buf(),
                        td.t_slots_buf(),
                        td.t_counts_buf(),
                    )
                    .remaining_vol_mm3;
                inline_vol + td.overflow_volume_mm3()
            }
        }
    }

    fn compute_quick_error_summary(&self) -> Option<(f64, f64, f64)> {
        match &self.inner {
            GpuBackendInner::Heightfield(hf) => self
                .ctx
                .reduce_heightfield_metrics(
                    hf.nx(),
                    hf.ny(),
                    hf.origin_wcs(),
                    hf.dx_mm(),
                    hf.dy_mm(),
                    0.05,
                    false,
                    hf.z_stock_buf(),
                    hf.z_bottom_buf(),
                    hf.z_target_buf(),
                )
                .quick_summary(),
            GpuBackendInner::Cylindrical(c) => self
                .ctx
                .reduce_cylindrical_metrics(
                    c.nu(),
                    c.ntheta(),
                    c.rotary(),
                    c.u_min(),
                    c.du(),
                    c.dtheta(),
                    0.05,
                    false,
                    c.r_stock_buf(),
                    c.r_target_buf(),
                )
                .quick_summary(),
            GpuBackendInner::TriDexel(td) => {
                if !td.has_target() {
                    return None;
                }
                if td.any_overflow() {
                    return td.with_synced_stock_and_target(|m, t| {
                        t.map(|tgt| compute_tridexel_quick_error_summary(m, tgt))
                    });
                }
                let totals = self.ctx.reduce_tridexel_metrics(
                    td.dims(),
                    td.origin_wcs(),
                    td.pitch_mm(),
                    td.plane_ray_offsets(),
                    td.total_rays(),
                    true,
                    0.05,
                    false,
                    td.m_slots_buf(),
                    td.m_counts_buf(),
                    td.t_slots_buf(),
                    td.t_counts_buf(),
                );
                let rms = if totals.sample_count > 0 {
                    (totals.sum_sq / totals.sample_count as f64).sqrt()
                } else {
                    0.0
                };
                Some((totals.excess_vol_mm3, totals.gouge_vol_mm3, rms))
            }
        }
    }

    fn compute_error_report(
        &self,
        initial_stock_vol_mm3: f64,
        tolerance_mm: f64,
        per_operation: Vec<OperationErrorSnapshot>,
    ) -> Option<StockErrorReport> {
        match &self.inner {
            GpuBackendInner::Heightfield(hf) => self
                .ctx
                .reduce_heightfield_metrics(
                    hf.nx(),
                    hf.ny(),
                    hf.origin_wcs(),
                    hf.dx_mm(),
                    hf.dy_mm(),
                    tolerance_mm,
                    false,
                    hf.z_stock_buf(),
                    hf.z_bottom_buf(),
                    hf.z_target_buf(),
                )
                .into_error_report(initial_stock_vol_mm3, tolerance_mm, per_operation, true),
            GpuBackendInner::Cylindrical(c) => self
                .ctx
                .reduce_cylindrical_metrics(
                    c.nu(),
                    c.ntheta(),
                    c.rotary(),
                    c.u_min(),
                    c.du(),
                    c.dtheta(),
                    tolerance_mm,
                    false,
                    c.r_stock_buf(),
                    c.r_target_buf(),
                )
                .into_error_report(initial_stock_vol_mm3, tolerance_mm, per_operation, true),
            GpuBackendInner::TriDexel(td) => {
                if !td.has_target() {
                    return None;
                }
                if td.any_overflow() {
                    return td.with_synced_stock_and_target(|m, t| {
                        t.map(|tgt| {
                            compute_tridexel_error_report(
                                m,
                                tgt,
                                initial_stock_vol_mm3,
                                tolerance_mm,
                                per_operation,
                            )
                        })
                    });
                }
                self.ctx
                    .reduce_tridexel_metrics(
                        td.dims(),
                        td.origin_wcs(),
                        td.pitch_mm(),
                        td.plane_ray_offsets(),
                        td.total_rays(),
                        true,
                        tolerance_mm,
                        false,
                        td.m_slots_buf(),
                        td.m_counts_buf(),
                        td.t_slots_buf(),
                        td.t_counts_buf(),
                    )
                    .into_error_report(initial_stock_vol_mm3, tolerance_mm, per_operation, false)
            }
        }
    }

    fn extract_stock_mesh(&self, color_mode: SimStockColorMode) -> SimStockMesh {
        match &self.inner {
            GpuBackendInner::Heightfield(hf) => {
                hf.with_synced_stock(|s| tessellate_stock_mesh_colored(s, color_mode))
            }
            GpuBackendInner::Cylindrical(c) => c.with_synced_stock(|s| {
                tessellate_cylindrical_stock_mesh_colored(s, color_mode, 0.0)
            }),
            GpuBackendInner::TriDexel(td) => td.with_synced_stock_and_target(|s, t| {
                tessellate_tridexel_stock_mesh_colored(s, t, color_mode)
            }),
        }
    }

    fn into_parts(self) -> (SimulatedStock, Option<TriDexelStock>) {
        match self.inner {
            GpuBackendInner::Heightfield(hf) => {
                (SimulatedStock::Heightfield(hf.into_stock()), None)
            }
            GpuBackendInner::Cylindrical(c) => (SimulatedStock::Cylindrical(c.into_stock()), None),
            GpuBackendInner::TriDexel(td) => {
                let (stock, target) = td.into_parts();
                (SimulatedStock::TriDexel(Box::new(stock)), target)
            }
        }
    }
}
