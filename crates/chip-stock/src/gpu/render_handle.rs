//! Zero-readback GPU stock render handles (`GpuStockRenderHandle`) exposing read-only
//! `Arc<wgpu::Buffer>` snapshots and grid metadata for direct viewport rendering across
//! `Heightfield`, `Cylindrical`, and `TriDexel` (with GPU compute surface extraction and
//! `draw_indirect` args).

use std::sync::{
    Arc, Mutex,
    atomic::{AtomicU64, Ordering},
};

use crate::{
    mesher_common::{
        MACHINED_METAL_COLOR, OPERATION_STOCK_PALETTE, RAW_STOCK_SIDE_COLOR, RAW_STOCK_TOP_COLOR,
    },
    metrics::SimStockColorMode,
    types::{RotaryStockSpec, SimStockMesh},
};

use super::context::GpuSimContext;

/// Maximum emitted vertices per Tri-Dexel GPU indirect draw (`2_400_000 × 40 B = 96 MB < 128 MiB`
/// WebGPU default `max_storage_buffer_binding_size`).
pub const MAX_TRIDEXEL_EMIT_VERTICES: u32 = 2_400_000;

/// Uniform block uploaded to `tridexel_emit.wgsl` (`400 bytes`, 16-byte aligned).
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct TriDexelEmitUniforms {
    /// `[nx, ny, nz, total_rays]`
    pub dims_and_total: [u32; 4],
    /// `[ray_off_x, ray_off_y, ray_off_z, has_target]`
    pub plane_offsets_and_target: [u32; 4],
    /// `[color_mode_id, max_vertices, 0, 0]`
    pub mode_and_cap: [u32; 4],
    /// `[origin_x, origin_y, origin_z, tolerance_mm]`
    pub origin_and_tol: [f32; 4],
    /// `[dx, dy, dz, max_scale_mm]`
    pub pitch_and_scale: [f32; 4],
    pub raw_top_color: [f32; 4],
    pub raw_side_color: [f32; 4],
    pub machined_metal_color: [f32; 4],
    pub op_palette: [[f32; 4]; 16],
}

impl TriDexelEmitUniforms {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn build(
        dims: [u32; 3],
        origin_wcs: [f32; 3],
        pitch_mm: [f32; 3],
        plane_ray_offsets: [u32; 3],
        total_rays: u32,
        has_target: bool,
        max_vertices: u32,
        color_mode: SimStockColorMode,
    ) -> Self {
        let (mode_id, tol_mm, max_scale_mm) = match color_mode {
            SimStockColorMode::Metallic => (0u32, 0.05_f32, 1.0_f32),
            SimStockColorMode::ByOperation => (1u32, 0.05_f32, 1.0_f32),
            SimStockColorMode::ErrorHeatmap {
                tolerance_mm,
                max_scale_mm,
            } => (2u32, tolerance_mm as f32, max_scale_mm as f32),
        };
        let mut op_palette = [[0.0_f32; 4]; 16];
        for (i, slot) in op_palette.iter_mut().enumerate() {
            *slot = OPERATION_STOCK_PALETTE[i % OPERATION_STOCK_PALETTE.len()];
        }
        Self {
            dims_and_total: [dims[0], dims[1], dims[2], total_rays],
            plane_offsets_and_target: [
                plane_ray_offsets[0],
                plane_ray_offsets[1],
                plane_ray_offsets[2],
                u32::from(has_target),
            ],
            mode_and_cap: [mode_id, max_vertices, 0, 0],
            origin_and_tol: [origin_wcs[0], origin_wcs[1], origin_wcs[2], tol_mm],
            pitch_and_scale: [pitch_mm[0], pitch_mm[1], pitch_mm[2], max_scale_mm],
            raw_top_color: RAW_STOCK_TOP_COLOR,
            raw_side_color: RAW_STOCK_SIDE_COLOR,
            machined_metal_color: MACHINED_METAL_COLOR,
            op_palette,
        }
    }
}

/// Read-only GPU storage-buffer snapshot & metadata for direct procedural 2.5D Heightfield
/// viewport rendering (`vs_sim_heightfield_gpu` + `fs_sim_gpu_stock`).
#[derive(Clone)]
pub struct GpuHeightfieldRenderData {
    pub nx: u32,
    pub ny: u32,
    pub origin_wcs: [f32; 3],
    pub dx_mm: f32,
    pub dy_mm: f32,
    pub has_target: bool,
    pub z_stock_buf: Arc<wgpu::Buffer>,
    pub z_bottom_buf: Arc<wgpu::Buffer>,
    pub z_target_buf: Arc<wgpu::Buffer>,
    pub last_op_buf: Arc<wgpu::Buffer>,
}

impl std::fmt::Debug for GpuHeightfieldRenderData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuHeightfieldRenderData")
            .field("nx", &self.nx)
            .field("ny", &self.ny)
            .field("has_target", &self.has_target)
            .finish()
    }
}

/// Read-only GPU storage-buffer snapshot & metadata for direct procedural 4th-Axis Cylindrical
/// viewport rendering (`vs_sim_cylindrical_gpu` + `fs_sim_gpu_stock`).
#[derive(Clone)]
pub struct GpuCylindricalRenderData {
    pub nu: u32,
    pub ntheta: u32,
    pub rotary: RotaryStockSpec,
    pub u_min: f64,
    pub u_max: f64,
    pub du: f64,
    pub dtheta: f64,
    pub has_target: bool,
    pub r_stock_buf: Arc<wgpu::Buffer>,
    pub r_target_buf: Arc<wgpu::Buffer>,
    pub last_op_buf: Arc<wgpu::Buffer>,
}

impl std::fmt::Debug for GpuCylindricalRenderData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuCylindricalRenderData")
            .field("nu", &self.nu)
            .field("ntheta", &self.ntheta)
            .field("has_target", &self.has_target)
            .finish()
    }
}

/// Read-only GPU indirect-draw vertex buffer & `DrawIndirectArgs` handle produced by
/// `tridexel_emit.wgsl`, supporting instant `< 0.3 ms` GPU-only recoloring on `color_mode` or
/// `tolerance_mm` changes without touching the CPU.
pub struct GpuTriDexelRenderData {
    ctx: Arc<GpuSimContext>,
    pub dims: [u32; 3],
    pub origin_wcs: [f32; 3],
    pub pitch_mm: [f32; 3],
    pub plane_ray_offsets: [u32; 3],
    pub total_rays: u32,
    pub has_target: bool,
    pub vertex_buf: Arc<wgpu::Buffer>,
    pub indirect_args_buf: Arc<wgpu::Buffer>,
    pub max_vertices: u32,
    /// Optional tiny CPU supplement mesh ONLY for rare rays exceeding `K = 4` inline spans.
    pub overflow_mesh: Option<Arc<SimStockMesh>>,
    /// Immutable snapshot buffers kept alive for `emit_bind_group` instant recoloring.
    _m_slots_buf: Arc<wgpu::Buffer>,
    _m_counts_buf: Arc<wgpu::Buffer>,
    _t_slots_buf: Arc<wgpu::Buffer>,
    _t_counts_buf: Arc<wgpu::Buffer>,
    emit_uniform_buf: Arc<wgpu::Buffer>,
    emit_bind_group: Arc<wgpu::BindGroup>,
    emitted_color_mode: Mutex<Option<SimStockColorMode>>,
    emit_dispatches: AtomicU64,
}

impl std::fmt::Debug for GpuTriDexelRenderData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuTriDexelRenderData")
            .field("dims", &self.dims)
            .field("has_target", &self.has_target)
            .field("max_vertices", &self.max_vertices)
            .field("has_overflow_mesh", &self.overflow_mesh.is_some())
            .finish()
    }
}

impl GpuTriDexelRenderData {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new_and_emit(
        ctx: Arc<GpuSimContext>,
        dims: [u32; 3],
        origin_wcs: [f32; 3],
        pitch_mm: [f32; 3],
        plane_ray_offsets: [u32; 3],
        total_rays: u32,
        has_target: bool,
        m_slots_buf: Arc<wgpu::Buffer>,
        m_counts_buf: Arc<wgpu::Buffer>,
        t_slots_buf: Arc<wgpu::Buffer>,
        t_counts_buf: Arc<wgpu::Buffer>,
        vertex_buf: Arc<wgpu::Buffer>,
        indirect_args_buf: Arc<wgpu::Buffer>,
        emit_uniform_buf: Arc<wgpu::Buffer>,
        max_vertices: u32,
        overflow_mesh: Option<Arc<SimStockMesh>>,
        color_mode: SimStockColorMode,
    ) -> Self {
        let emit_bind_group =
            Arc::new(ctx.device().create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("chip_td_emit_bg"),
                layout: &ctx.tridexel_emit_bgl,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: emit_uniform_buf.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: m_slots_buf.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: m_counts_buf.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 3,
                        resource: t_slots_buf.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 4,
                        resource: t_counts_buf.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 5,
                        resource: vertex_buf.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 6,
                        resource: indirect_args_buf.as_entire_binding(),
                    },
                ],
            }));

        let data = Self {
            ctx,
            dims,
            origin_wcs,
            pitch_mm,
            plane_ray_offsets,
            total_rays,
            has_target,
            vertex_buf,
            indirect_args_buf,
            max_vertices,
            overflow_mesh,
            _m_slots_buf: m_slots_buf,
            _m_counts_buf: m_counts_buf,
            _t_slots_buf: t_slots_buf,
            _t_counts_buf: t_counts_buf,
            emit_uniform_buf,
            emit_bind_group,
            emitted_color_mode: Mutex::new(None),
            emit_dispatches: AtomicU64::new(0),
        };

        let mut encoder =
            data.ctx
                .device()
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("chip_td_emit_initial_encoder"),
                });
        data.ensure_emitted_for_color_mode(color_mode, &mut encoder);
        data.ctx.queue().submit(std::iter::once(encoder.finish()));

        data
    }

    /// Re-dispatches the lightweight `< 0.3 ms` `tridexel_emit.wgsl` compute pass into `encoder`
    /// ONLY when `color_mode` differs from the color mode already baked into `vertex_buf`.
    /// Returns `true` if a compute dispatch was recorded.
    pub fn ensure_emitted_for_color_mode(
        &self,
        color_mode: SimStockColorMode,
        encoder: &mut wgpu::CommandEncoder,
    ) -> bool {
        let mut guard = self
            .emitted_color_mode
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if *guard == Some(color_mode) {
            return false;
        }
        let uniforms = TriDexelEmitUniforms::build(
            self.dims,
            self.origin_wcs,
            self.pitch_mm,
            self.plane_ray_offsets,
            self.total_rays,
            self.has_target,
            self.max_vertices,
            color_mode,
        );
        self.ctx
            .queue()
            .write_buffer(&self.emit_uniform_buf, 0, bytemuck::bytes_of(&uniforms));

        encoder.copy_buffer_to_buffer(
            &self.ctx.tridexel_emit_init_args_buf,
            0,
            &self.indirect_args_buf,
            0,
            16,
        );

        let num_wg = self.total_rays.div_ceil(64).max(1);
        let wg_x = num_wg.min(65_535);
        let wg_y = num_wg.div_ceil(65_535);
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("chip_td_emit_pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.ctx.tridexel_emit_pipeline);
            pass.set_bind_group(0, &*self.emit_bind_group, &[]);
            pass.dispatch_workgroups(wg_x, wg_y, 1);
        }

        *guard = Some(color_mode);
        self.emit_dispatches.fetch_add(1, Ordering::Relaxed);
        true
    }

    /// Number of times `tridexel_emit.wgsl` has been dispatched on this handle.
    #[inline]
    pub fn emit_dispatch_count(&self) -> u64 {
        self.emit_dispatches.load(Ordering::Relaxed)
    }
}

/// Polymorphic read-only GPU stock render handle published by [`super::WgpuComputeBackend`] for
/// zero-readback direct rendering inside `chip-gui`'s `ViewportRenderer`.
#[derive(Debug)]
pub enum GpuStockRenderHandle {
    Heightfield(GpuHeightfieldRenderData),
    Cylindrical(GpuCylindricalRenderData),
    TriDexel(GpuTriDexelRenderData),
}

impl GpuStockRenderHandle {
    /// Returns `true` if target comparison data is present on the GPU for Error Heatmap display.
    #[inline]
    pub const fn has_target(&self) -> bool {
        match self {
            Self::Heightfield(h) => h.has_target,
            Self::Cylindrical(c) => c.has_target,
            Self::TriDexel(t) => t.has_target,
        }
    }
}
