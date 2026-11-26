//! `WgpuComputeBackend` implementing [`SimStockBackend`] across all three volumetric stock
//! engines (`Heightfield`, `Cylindrical`, and `TriDexel`) with GPU parallel reduction metrology
//! and direct GPU BVH target rasterization.

use std::sync::Arc;

use chip_bvh::TriMesh3D;

use crate::{
    backend::{SimStockBackend, SimulatedStock, StockCheckpointSnapshot, StockModelKind},
    cylindrical::tessellate_cylindrical_stock_mesh_with_options,
    heightfield::tessellate_stock_mesh_with_options,
    metrics::{
        OperationErrorSnapshot, SimStockColorMode, StockErrorReport, compute_tridexel_error_report,
        compute_tridexel_quick_error_summary,
    },
    tridexel::{TriDexelStock, tessellate_tridexel_stock_mesh_with_options},
    types::{
        ActiveSimBackendInfo, BatchSweepOutput, SimMeshOptions, SimStockMesh, SweepChord,
        SweepToolSpec,
    },
};

use super::{
    checkpoints::GpuCheckpointBuffers, context::GpuSimContext, cylindrical::GpuCylindricalState,
    heightfield::GpuHeightfieldState, render_handle::GpuStockRenderHandle,
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

    /// Rasterizes a WCS-space [`TriMesh3D`] target solid directly onto this backend's GPU target
    /// buffers using `bvh_distance.wgsl` (`FlatBvhNode` + `GpuTriangle` zero-conversion upload).
    pub fn rasterize_target_mesh_gpu(&mut self, target_mesh: &TriMesh3D) {
        match &mut self.inner {
            GpuBackendInner::Heightfield(hf) => hf.rasterize_target_mesh_gpu(target_mesh),
            GpuBackendInner::Cylindrical(c) => c.rasterize_target_mesh_gpu(target_mesh),
            GpuBackendInner::TriDexel(td) => td.rasterize_target_mesh_gpu(target_mesh),
        }
    }

    /// Captures a read-only zero-readback [`GpuStockRenderHandle`] snapshot for direct viewport
    /// rendering on the shared `wgpu::Device` (`Heightfield`, `Cylindrical`, or `TriDexel`
    /// indirect-draw vertex buffer emitted via `tridexel_emit.wgsl`).
    pub fn capture_render_handle(
        &mut self,
        color_mode: SimStockColorMode,
    ) -> Arc<GpuStockRenderHandle> {
        match &mut self.inner {
            GpuBackendInner::Heightfield(hf) => hf.capture_render_handle(),
            GpuBackendInner::Cylindrical(c) => c.capture_render_handle(),
            GpuBackendInner::TriDexel(td) => td.capture_render_handle(color_mode),
        }
    }

    /// Captures a read-only zero-readback [`GpuStockRenderHandle`] snapshot with visual
    /// [`SimMeshOptions`] (`color_mode` + optional `mesh_skinning`).
    pub fn capture_render_handle_with_options(
        &mut self,
        options: SimMeshOptions,
    ) -> Arc<GpuStockRenderHandle> {
        match &mut self.inner {
            GpuBackendInner::Heightfield(hf) => hf.capture_render_handle_with_options(options),
            GpuBackendInner::Cylindrical(c) => c.capture_render_handle(),
            GpuBackendInner::TriDexel(td) => td.capture_render_handle_with_options(options),
        }
    }

    /// Uploads a CPU [`SimulatedStock`] state directly into this backend's GPU storage buffers
    /// (used when restoring a final stock state whose GPU checkpoint was evicted).
    pub fn copy_state_from_cpu_stock(&mut self, stock: &SimulatedStock) {
        match (&mut self.inner, stock) {
            (GpuBackendInner::Heightfield(hf), SimulatedStock::Heightfield(src)) => {
                hf.copy_state_from_cpu_stock(src);
            }
            (GpuBackendInner::Cylindrical(c), SimulatedStock::Cylindrical(src)) => {
                c.copy_state_from_cpu_stock(src);
            }
            (GpuBackendInner::TriDexel(td), SimulatedStock::TriDexel(src)) => {
                td.copy_state_from_cpu_stock(src);
            }
            _ => {}
        }
    }

    /// Synchronizes live GPU stock state into the CPU shadow copy (if dirty) and invokes `f`
    /// on the up-to-date [`SimulatedStock`] view. Useful for tests & point queries.
    pub fn with_synced_cpu_stock<R>(&self, f: impl FnOnce(&SimulatedStock) -> R) -> R {
        match &self.inner {
            GpuBackendInner::Heightfield(hf) => {
                hf.with_synced_stock(|s| f(&SimulatedStock::Heightfield(s.clone())))
            }
            GpuBackendInner::Cylindrical(c) => {
                c.with_synced_stock(|s| f(&SimulatedStock::Cylindrical(s.clone())))
            }
            GpuBackendInner::TriDexel(td) => {
                td.with_synced_stock(|s| f(&SimulatedStock::TriDexel(Box::new(s.clone()))))
            }
        }
    }

    pub(crate) fn checkpoint_byte_size(&self) -> usize {
        match &self.inner {
            GpuBackendInner::Heightfield(hf) => hf.checkpoint_byte_size(),
            GpuBackendInner::Cylindrical(c) => c.checkpoint_byte_size(),
            GpuBackendInner::TriDexel(td) => td.checkpoint_byte_size(),
        }
    }

    pub(crate) fn clone_live_to_checkpoint_buffers(&self) -> GpuCheckpointBuffers {
        match &self.inner {
            GpuBackendInner::Heightfield(hf) => hf.clone_live_to_checkpoint_buffers(),
            GpuBackendInner::Cylindrical(c) => c.clone_live_to_checkpoint_buffers(),
            GpuBackendInner::TriDexel(td) => td.clone_live_to_checkpoint_buffers(),
        }
    }

    pub(crate) fn restore_from_gpu_checkpoint_buffers(&mut self, cp: &GpuCheckpointBuffers) {
        match (&mut self.inner, cp) {
            (
                GpuBackendInner::Heightfield(hf),
                GpuCheckpointBuffers::Heightfield {
                    z_stock,
                    last_op_idx,
                    cell_bytes,
                },
            ) => hf.restore_from_gpu_checkpoint(z_stock, last_op_idx, *cell_bytes),
            (
                GpuBackendInner::Cylindrical(c),
                GpuCheckpointBuffers::Cylindrical {
                    r_stock,
                    last_op_idx,
                    cell_bytes,
                },
            ) => c.restore_from_gpu_checkpoint(r_stock, last_op_idx, *cell_bytes),
            (
                GpuBackendInner::TriDexel(td),
                GpuCheckpointBuffers::TriDexel {
                    m_slots,
                    m_counts,
                    slots_bytes,
                    counts_bytes,
                    overflow,
                },
            ) => td.restore_from_gpu_checkpoint(
                m_slots,
                m_counts,
                *slots_bytes,
                *counts_bytes,
                overflow,
            ),
            _ => {}
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
                if s.rotary.is_some() {
                    return 0.0;
                }
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

    fn extract_stock_mesh_with_options(&self, options: SimMeshOptions) -> SimStockMesh {
        match &self.inner {
            GpuBackendInner::Heightfield(hf) => {
                hf.with_synced_stock(|s| tessellate_stock_mesh_with_options(s, options))
            }
            GpuBackendInner::Cylindrical(c) => c.with_synced_stock(|s| {
                tessellate_cylindrical_stock_mesh_with_options(s, options, 0.0)
            }),
            GpuBackendInner::TriDexel(td) => td.with_synced_stock_and_target(|s, t| {
                tessellate_tridexel_stock_mesh_with_options(s, t, options)
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
