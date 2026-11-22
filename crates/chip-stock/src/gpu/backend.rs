//! `WgpuComputeBackend` implementing [`SimStockBackend`] over GPU-resident `Heightfield` and
//! `Cylindrical` compute sweepers (with clean CPU delegation for `TriDexel` until Milestone 53).

use std::sync::Arc;

use crate::{
    backend::{
        CpuExactBackend, SimStockBackend, SimulatedStock, StockCheckpointSnapshot, StockModelKind,
    },
    cylindrical::tessellate_cylindrical_stock_mesh_colored,
    heightfield::tessellate_stock_mesh_colored,
    metrics::{
        OperationErrorSnapshot, SimStockColorMode, StockErrorReport,
        compute_cylindrical_error_report, compute_cylindrical_quick_error_summary,
        compute_heightmap_error_report, compute_heightmap_quick_error_summary,
    },
    tridexel::TriDexelStock,
    types::{ActiveSimBackendInfo, BatchSweepOutput, SimStockMesh, SweepChord, SweepToolSpec},
};

use super::{
    context::GpuSimContext, cylindrical::GpuCylindricalState, heightfield::GpuHeightfieldState,
};

#[derive(Debug)]
enum GpuBackendInner {
    Heightfield(Box<GpuHeightfieldState>),
    Cylindrical(Box<GpuCylindricalState>),
    /// Temporary CPU delegation for `TriDexel` until Milestone 53's WGSL Tri-Dexel sweeper lands.
    TriDexelCpuFallback(Box<CpuExactBackend>),
}

/// Optional `wgpu` compute volumetric stock simulation backend.
///
/// Executes 2.5D `Heightfield` and 4th-axis `Cylindrical` stock removal and Rules 1–4 verification
/// on the GPU via tiled WGSL compute shaders, while transparently delegating `TriDexel` to
/// [`CpuExactBackend`] until Milestone 53.
#[derive(Debug)]
pub struct WgpuComputeBackend {
    ctx: Arc<GpuSimContext>,
    inner: GpuBackendInner,
}

impl WgpuComputeBackend {
    /// Uploads `stock` into GPU storage buffers (for `Heightfield` and `Cylindrical`) using `ctx`.
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
            SimulatedStock::TriDexel(td) => GpuBackendInner::TriDexelCpuFallback(Box::new(
                CpuExactBackend::new(SimulatedStock::TriDexel(td), target_tridexel),
            )),
        };
        Self { ctx, inner }
    }

    #[inline]
    pub fn context(&self) -> &Arc<GpuSimContext> {
        &self.ctx
    }
}

impl SimStockBackend for WgpuComputeBackend {
    fn backend_info(&self) -> ActiveSimBackendInfo {
        match &self.inner {
            GpuBackendInner::Heightfield(_) | GpuBackendInner::Cylindrical(_) => {
                self.ctx.adapter_info().clone()
            }
            GpuBackendInner::TriDexelCpuFallback(_) => ActiveSimBackendInfo::cpu(),
        }
    }

    fn kind(&self) -> StockModelKind {
        match &self.inner {
            GpuBackendInner::Heightfield(_) => StockModelKind::Heightfield25D,
            GpuBackendInner::Cylindrical(c) => {
                c.with_synced_stock(|s| StockModelKind::Cylindrical4Axis(s.rotary))
            }
            GpuBackendInner::TriDexelCpuFallback(cpu) => cpu.kind(),
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
            GpuBackendInner::TriDexelCpuFallback(cpu) => cpu.capture_checkpoint(completed_segments),
        }
    }

    fn reset_from_checkpoint(&mut self, cp: &StockCheckpointSnapshot) {
        match &mut self.inner {
            GpuBackendInner::Heightfield(hf) => hf.reset_from_checkpoint(cp),
            GpuBackendInner::Cylindrical(c) => c.reset_from_checkpoint(cp),
            GpuBackendInner::TriDexelCpuFallback(cpu) => cpu.reset_from_checkpoint(cp),
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
            GpuBackendInner::TriDexelCpuFallback(cpu) => cpu.sweep_batch(chords, profiles, verify),
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
            GpuBackendInner::TriDexelCpuFallback(cpu) => cpu.query_vertical_plunge_depth(
                x_wcs,
                y_wcs,
                z_start_wcs,
                z_end_wcs,
                thread_check_radius_mm,
            ),
        }
    }

    fn compute_volume_mm3(&self) -> f64 {
        match &self.inner {
            GpuBackendInner::Heightfield(hf) => hf.with_synced_stock(|s| s.compute_volume_mm3()),
            GpuBackendInner::Cylindrical(c) => c.with_synced_stock(|s| s.current_volume_mm3()),
            GpuBackendInner::TriDexelCpuFallback(cpu) => cpu.compute_volume_mm3(),
        }
    }

    fn compute_quick_error_summary(&self) -> Option<(f64, f64, f64)> {
        match &self.inner {
            GpuBackendInner::Heightfield(hf) => {
                hf.with_synced_stock(compute_heightmap_quick_error_summary)
            }
            GpuBackendInner::Cylindrical(c) => {
                c.with_synced_stock(compute_cylindrical_quick_error_summary)
            }
            GpuBackendInner::TriDexelCpuFallback(cpu) => cpu.compute_quick_error_summary(),
        }
    }

    fn compute_error_report(
        &self,
        initial_stock_vol_mm3: f64,
        tolerance_mm: f64,
        per_operation: Vec<OperationErrorSnapshot>,
    ) -> Option<StockErrorReport> {
        match &self.inner {
            GpuBackendInner::Heightfield(hf) => hf.with_synced_stock(|s| {
                compute_heightmap_error_report(
                    s,
                    initial_stock_vol_mm3,
                    tolerance_mm,
                    per_operation,
                )
            }),
            GpuBackendInner::Cylindrical(c) => c.with_synced_stock(|s| {
                compute_cylindrical_error_report(
                    s,
                    initial_stock_vol_mm3,
                    tolerance_mm,
                    per_operation,
                )
            }),
            GpuBackendInner::TriDexelCpuFallback(cpu) => {
                cpu.compute_error_report(initial_stock_vol_mm3, tolerance_mm, per_operation)
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
            GpuBackendInner::TriDexelCpuFallback(cpu) => cpu.extract_stock_mesh(color_mode),
        }
    }

    fn into_parts(self) -> (SimulatedStock, Option<TriDexelStock>) {
        match self.inner {
            GpuBackendInner::Heightfield(hf) => {
                (SimulatedStock::Heightfield(hf.into_stock()), None)
            }
            GpuBackendInner::Cylindrical(c) => (SimulatedStock::Cylindrical(c.into_stock()), None),
            GpuBackendInner::TriDexelCpuFallback(cpu) => cpu.into_parts(),
        }
    }
}
