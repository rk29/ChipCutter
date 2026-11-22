//! Zero-readback GPU stock render handles (`GpuStockRenderHandle`) exposing read-only
//! `Arc<wgpu::Buffer>` snapshots and grid metadata for direct viewport rendering across
//! `Heightfield`, `Cylindrical`, and `TriDexel` (with GPU compute surface extraction and
//! `draw_indirect` args).

use std::sync::{
    Arc, Mutex,
    atomic::{AtomicU64, Ordering},
};

use wgpu::util::DeviceExt as _;

use crate::{
    backend::SimulatedStock,
    mesher_common::{
        MACHINED_METAL_COLOR, OPERATION_STOCK_PALETTE, RAW_STOCK_SIDE_COLOR, RAW_STOCK_TOP_COLOR,
    },
    metrics::SimStockColorMode,
    tridexel::{FlatDexelSpan, INLINE_DEXEL_SLOTS, TriDexelStock},
    types::{Axis3, RotaryStockSpec, SimStockMesh},
};

use super::{context::GpuSimContext, tridexel::build_tridexel_overflow_supplement_mesh};

/// Compact 20-byte GPU Tri-Dexel quad record emitted by `tridexel_emit.wgsl` and expanded into
/// 6 vertices (2 triangles) on-the-fly by `vs_sim_tridexel_gpu`.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct PackedTriDexelQuad {
    /// Bits `0..30`: `global_ray`, bit `31`: `want_min` (`1` = min endpoint, `0` = max endpoint).
    pub ray_and_flags: u32,
    /// Packed `RGBA8` unorm color (`pack4x8unorm` / `unpack4x8unorm`).
    pub color_rgba8: u32,
    /// Endpoint coordinate relative to plane `origin_w` (`mm`).
    pub w_local: f32,
    /// Finite-difference slope `dw/du`.
    pub dw_du: f32,
    /// Finite-difference slope `dw/dv`.
    pub dw_dv: f32,
}

/// Byte size of one [`PackedTriDexelQuad`] storage element (`20 bytes`).
pub const TRIDEXEL_PACKED_QUAD_BYTES: usize = std::mem::size_of::<PackedTriDexelQuad>();

const _: () = assert!(TRIDEXEL_PACKED_QUAD_BYTES == 20);

/// Maximum emitted quads per Tri-Dexel GPU indirect draw (`6_000_000 × 20 B = 120 MB < 128 MiB`
/// WebGPU default `max_storage_buffer_binding_size`), exceeding even a worst-case `448³` Ultra
/// grid with all `K = 4` inline spans populated (`602_112 × 4 × 2 = 4_816_896` quads).
pub const MAX_TRIDEXEL_EMIT_QUADS: u32 = 6_000_000;

/// Maximum emitted vertices per Tri-Dexel GPU indirect draw (`6 × MAX_TRIDEXEL_EMIT_QUADS`).
pub const MAX_TRIDEXEL_EMIT_VERTICES: u32 = MAX_TRIDEXEL_EMIT_QUADS * 6;

/// Computes the exact inline quad capacity for `total_rays` clamped to both
/// [`MAX_TRIDEXEL_EMIT_QUADS`] and the active device's `max_storage_buffer_binding_size`.
pub(crate) fn compute_tridexel_max_quads(device: &wgpu::Device, total_rays: usize) -> u32 {
    let worst_case_quads = (total_rays as u64)
        .saturating_mul(INLINE_DEXEL_SLOTS as u64)
        .saturating_mul(2);
    let device_quad_cap =
        device.limits().max_storage_buffer_binding_size / (TRIDEXEL_PACKED_QUAD_BYTES as u64);
    let hard_cap = (MAX_TRIDEXEL_EMIT_QUADS as u64).min(device_quad_cap);
    worst_case_quads.clamp(1_000, hard_cap.max(1_000)) as u32
}

/// Uniform block uploaded to `tridexel_emit.wgsl` (`400 bytes`, 16-byte aligned).
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct TriDexelEmitUniforms {
    /// `[nx, ny, nz, total_rays]`
    pub dims_and_total: [u32; 4],
    /// `[ray_off_x, ray_off_y, ray_off_z, has_target]`
    pub plane_offsets_and_target: [u32; 4],
    /// `[color_mode_id, max_quads, 0, 0]`
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
        max_quads: u32,
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
            mode_and_cap: [mode_id, max_quads, 0, 0],
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

/// Read-only GPU indirect-draw compact quad buffer & `DrawIndirectArgs` handle produced by
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
    pub quad_buf: Arc<wgpu::Buffer>,
    pub indirect_args_buf: Arc<wgpu::Buffer>,
    pub max_quads: u32,
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
            .field("max_quads", &self.max_quads)
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
        quad_buf: Arc<wgpu::Buffer>,
        indirect_args_buf: Arc<wgpu::Buffer>,
        emit_uniform_buf: Arc<wgpu::Buffer>,
        max_quads: u32,
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
                        resource: quad_buf.as_entire_binding(),
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
            quad_buf,
            indirect_args_buf,
            max_quads,
            max_vertices: max_quads.saturating_mul(6),
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
    /// ONLY when `color_mode` differs from the color mode already baked into `quad_buf`.
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
            self.max_quads,
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

    /// Synchronously reads back the emitted vertex count from `indirect_args_buf` (`draw_args[0]`).
    /// Intended for verification and diagnostic tests (not the per-frame render loop).
    pub fn readback_emitted_vertex_count(&self) -> u32 {
        let staging = self.ctx.device().create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_td_indirect_args_readback"),
            size: 16,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut encoder =
            self.ctx
                .device()
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("chip_td_indirect_args_copy"),
                });
        encoder.copy_buffer_to_buffer(&self.indirect_args_buf, 0, &staging, 0, 16);
        self.ctx.queue().submit(std::iter::once(encoder.finish()));
        let mut raw = [0u8; 16];
        if self.ctx.readback_bytes(&staging, 16, &mut raw) {
            let args: [u32; 4] = *bytemuck::from_bytes(&raw);
            args[0]
        } else {
            0
        }
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
    /// Uploads flat stock buffers from a [`SimulatedStock`] snapshot directly onto `ctx`'s
    /// presentation `wgpu::Device` (zero CPU mesh tessellation, safe to call strictly on the
    /// main UI thread while background compute sweeps run on an independent headless device).
    pub fn from_simulated_stock(
        ctx: &Arc<GpuSimContext>,
        stock: &SimulatedStock,
        target_tridexel: Option<&TriDexelStock>,
        color_mode: SimStockColorMode,
    ) -> Arc<Self> {
        let device = ctx.device();
        match stock {
            SimulatedStock::Heightfield(hf) => {
                let z_stock_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_hf_render_z_stock"),
                        contents: bytemuck::cast_slice(&hf.z_stock),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let z_bottom_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_hf_render_z_bottom"),
                        contents: bytemuck::cast_slice(&hf.z_bottom),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let z_target_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_hf_render_z_target"),
                        contents: bytemuck::cast_slice(&hf.z_target),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let last_op_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_hf_render_last_op"),
                        contents: bytemuck::cast_slice(&hf.last_op_idx),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let has_target = hf.z_target.iter().any(|z| z.is_finite());
                Arc::new(Self::Heightfield(GpuHeightfieldRenderData {
                    nx: hf.nx as u32,
                    ny: hf.ny as u32,
                    origin_wcs: [
                        hf.origin_wcs.x as f32,
                        hf.origin_wcs.y as f32,
                        hf.origin_wcs.z as f32,
                    ],
                    dx_mm: hf.dx_mm as f32,
                    dy_mm: hf.dy_mm as f32,
                    has_target,
                    z_stock_buf,
                    z_bottom_buf,
                    z_target_buf,
                    last_op_buf,
                }))
            }
            SimulatedStock::Cylindrical(cyl) => {
                let r_stock_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_cyl_render_r_stock"),
                        contents: bytemuck::cast_slice(&cyl.r_stock),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let r_target_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_cyl_render_r_target"),
                        contents: bytemuck::cast_slice(&cyl.r_target),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let last_op_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_cyl_render_last_op"),
                        contents: bytemuck::cast_slice(&cyl.last_op_idx),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let has_target = cyl.r_target.iter().any(|&r| r >= 0.0 && r.is_finite());
                Arc::new(Self::Cylindrical(GpuCylindricalRenderData {
                    nu: cyl.nu as u32,
                    ntheta: cyl.ntheta as u32,
                    rotary: cyl.rotary,
                    u_min: cyl.u_min,
                    u_max: cyl.u_max,
                    du: cyl.du,
                    dtheta: cyl.dtheta,
                    has_target,
                    r_stock_buf,
                    r_target_buf,
                    last_op_buf,
                }))
            }
            SimulatedStock::TriDexel(td) => {
                let rx = td.planes[0].ray_count();
                let ry = td.planes[1].ray_count();
                let rz = td.planes[2].ray_count();
                let plane_ray_offsets = [0u32, rx as u32, (rx + ry) as u32];
                let total_rays = rx + ry + rz;
                let total_slots = total_rays * INLINE_DEXEL_SLOTS;

                let mut all_m_slots = Vec::with_capacity(total_slots.max(1));
                let mut all_m_counts = Vec::with_capacity(total_rays.max(1));
                for axis in Axis3::ALL {
                    let p = td.plane(axis);
                    all_m_slots.extend_from_slice(&p.slots);
                    all_m_counts.extend_from_slice(&p.counts);
                }
                if all_m_slots.is_empty() {
                    all_m_slots.push(FlatDexelSpan::ZERO);
                }
                if all_m_counts.is_empty() {
                    all_m_counts.push(0);
                }

                let m_slots_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_td_render_m_slots"),
                        contents: bytemuck::cast_slice(&all_m_slots),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let m_counts_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_td_render_m_counts"),
                        contents: bytemuck::cast_slice(&all_m_counts),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));

                let has_target = target_tridexel.is_some();
                let (t_slots_buf, t_counts_buf) = if let Some(tgt) = target_tridexel {
                    let mut all_t_slots = Vec::with_capacity(total_slots.max(1));
                    let mut all_t_counts = Vec::with_capacity(total_rays.max(1));
                    for axis in Axis3::ALL {
                        let p = tgt.plane(axis);
                        all_t_slots.extend_from_slice(&p.slots);
                        all_t_counts.extend_from_slice(&p.counts);
                    }
                    if all_t_slots.is_empty() {
                        all_t_slots.push(FlatDexelSpan::ZERO);
                    }
                    if all_t_counts.is_empty() {
                        all_t_counts.push(0);
                    }
                    (
                        Arc::new(
                            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                                label: Some("chip_td_render_t_slots"),
                                contents: bytemuck::cast_slice(&all_t_slots),
                                usage: wgpu::BufferUsages::STORAGE,
                            }),
                        ),
                        Arc::new(
                            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                                label: Some("chip_td_render_t_counts"),
                                contents: bytemuck::cast_slice(&all_t_counts),
                                usage: wgpu::BufferUsages::STORAGE,
                            }),
                        ),
                    )
                } else {
                    let dummy_slots = [FlatDexelSpan::ZERO];
                    let dummy_counts = [0u32; 4];
                    (
                        Arc::new(
                            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                                label: Some("chip_td_render_dummy_t_slots"),
                                contents: bytemuck::cast_slice(&dummy_slots),
                                usage: wgpu::BufferUsages::STORAGE,
                            }),
                        ),
                        Arc::new(
                            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                                label: Some("chip_td_render_dummy_t_counts"),
                                contents: bytemuck::cast_slice(&dummy_counts),
                                usage: wgpu::BufferUsages::STORAGE,
                            }),
                        ),
                    )
                };

                let max_quads = compute_tridexel_max_quads(device, total_rays);
                let quad_bytes = (max_quads as u64) * (TRIDEXEL_PACKED_QUAD_BYTES as u64);
                let quad_buf = Arc::new(device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("chip_td_render_emit_quads"),
                    size: quad_bytes.max(64),
                    usage: wgpu::BufferUsages::STORAGE,
                    mapped_at_creation: false,
                }));
                let indirect_args_buf = Arc::new(device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("chip_td_render_emit_indirect_args"),
                    size: 16,
                    usage: wgpu::BufferUsages::INDIRECT
                        | wgpu::BufferUsages::STORAGE
                        | wgpu::BufferUsages::COPY_DST
                        | wgpu::BufferUsages::COPY_SRC,
                    mapped_at_creation: false,
                }));
                let emit_uniform_buf = Arc::new(device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("chip_td_render_emit_uniforms"),
                    size: std::mem::size_of::<TriDexelEmitUniforms>() as u64,
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                }));

                let overflow_mesh =
                    build_tridexel_overflow_supplement_mesh(td, target_tridexel, color_mode);
                let data = GpuTriDexelRenderData::new_and_emit(
                    Arc::clone(ctx),
                    [td.dims[0] as u32, td.dims[1] as u32, td.dims[2] as u32],
                    [
                        td.origin_wcs.x as f32,
                        td.origin_wcs.y as f32,
                        td.origin_wcs.z as f32,
                    ],
                    [
                        td.pitch_mm.x as f32,
                        td.pitch_mm.y as f32,
                        td.pitch_mm.z as f32,
                    ],
                    plane_ray_offsets,
                    total_rays as u32,
                    has_target,
                    m_slots_buf,
                    m_counts_buf,
                    t_slots_buf,
                    t_counts_buf,
                    quad_buf,
                    indirect_args_buf,
                    emit_uniform_buf,
                    max_quads,
                    overflow_mesh,
                    color_mode,
                );
                Arc::new(Self::TriDexel(data))
            }
        }
    }

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
