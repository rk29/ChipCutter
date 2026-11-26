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
    types::{Axis3, RotaryStockSpec, SimMeshOptions, SimStockMesh},
};

use super::{context::GpuSimContext, tridexel::build_tridexel_overflow_supplement_mesh};

/// Scale factor used to pack Tri-Dexel dual corner offsets `(du, dv, dw)` into `snorm8×3`.
/// Covers `[-1.0, +1.0]` voxel pitch so both crisp (`±0.55`) and relaxed skinned (`±0.95`)
/// dual corners encode cleanly without clipping.
pub const TRIDEXEL_DUAL_OFFSET_PACK_SCALE: f32 = 1.0;

/// Encodes a 3D unit normal into signed 2D octahedral coordinates in `[-1.0, 1.0]²`.
#[inline]
pub fn oct_encode_unit_normal(n: [f32; 3]) -> [f32; 2] {
    let l1 = n[0].abs() + n[1].abs() + n[2].abs();
    if l1 <= 1e-6 {
        return [0.0, 0.0];
    }
    let mut px = n[0] / l1;
    let mut py = n[1] / l1;
    if n[2] < 0.0 {
        let sx = if px >= 0.0 { 1.0 } else { -1.0 };
        let sy = if py >= 0.0 { 1.0 } else { -1.0 };
        let nx = (1.0 - py.abs()) * sx;
        let ny = (1.0 - px.abs()) * sy;
        px = nx;
        py = ny;
    }
    [px.clamp(-1.0, 1.0), py.clamp(-1.0, 1.0)]
}

/// Decodes signed 2D octahedral coordinates in `[-1.0, 1.0]²` back to a 3D unit normal.
#[inline]
pub fn oct_decode_unit_normal(e: [f32; 2]) -> [f32; 3] {
    let mut x = e[0];
    let mut y = e[1];
    let z = 1.0 - x.abs() - y.abs();
    if z < 0.0 {
        let sx = if x >= 0.0 { 1.0 } else { -1.0 };
        let sy = if y >= 0.0 { 1.0 } else { -1.0 };
        let nx = (1.0 - y.abs()) * sx;
        let ny = (1.0 - x.abs()) * sy;
        x = nx;
        y = ny;
    }
    let len = (x * x + y * y + z * z).sqrt();
    if len > 1e-6 {
        [x / len, y / len, z / len]
    } else {
        [0.0, 0.0, 1.0]
    }
}

/// Packs two 3D unit normals `(n_a, n_b)` into a 12-bit-per-axis octahedral representation:
/// returns `(packed_hi8_ab, lo4_byte_a, lo4_byte_b)` where `packed_hi8_ab` stores the top 8 bits
/// of `(ax, ay, bx, by)` (`n01_oct` / `n23_oct`) and `lo4_byte_*` stores the 4+4-bit refinement
/// packed into the spare high byte of `corner*_pad`. Cardinal axes roundtrip with `0.0°` error
/// and arbitrary sphere directions roundtrip within `< 0.06°`.
#[inline]
pub fn pack_oct_normal_pair_12bit(n_a: [f32; 3], n_b: [f32; 3]) -> (u32, u8, u8) {
    let ea = oct_encode_unit_normal(n_a);
    let eb = oct_encode_unit_normal(n_b);
    let qax = ((ea[0] + 1.0) * 2047.0).round().clamp(0.0, 4094.0) as u32;
    let qay = ((ea[1] + 1.0) * 2047.0).round().clamp(0.0, 4094.0) as u32;
    let qbx = ((eb[0] + 1.0) * 2047.0).round().clamp(0.0, 4094.0) as u32;
    let qby = ((eb[1] + 1.0) * 2047.0).round().clamp(0.0, 4094.0) as u32;

    let hi8_word = ((qax >> 4) & 0xFF)
        | (((qay >> 4) & 0xFF) << 8)
        | (((qbx >> 4) & 0xFF) << 16)
        | (((qby >> 4) & 0xFF) << 24);
    let lo_a = ((qax & 0xF) | ((qay & 0xF) << 4)) as u8;
    let lo_b = ((qbx & 0xF) | ((qby & 0xF) << 4)) as u8;
    (hi8_word, lo_a, lo_b)
}

/// Unpacks one 12-bit-per-axis octahedral normal from `(hi8_word, is_second, corner_word)` matching
/// `vs_sim_tridexel_gpu` bit-for-bit.
#[inline]
pub fn unpack_oct_normal_12bit(hi8_word: u32, is_second: bool, corner_word: u32) -> [f32; 3] {
    let shift = if is_second { 16 } else { 0 };
    let hi8_x = (hi8_word >> shift) & 0xFF;
    let hi8_y = (hi8_word >> (shift + 8)) & 0xFF;
    let lo_byte = (corner_word >> 24) & 0xFF;
    let lo4_x = lo_byte & 0xF;
    let lo4_y = (lo_byte >> 4) & 0xF;
    let qx = (hi8_x << 4) | lo4_x;
    let qy = (hi8_y << 4) | lo4_y;
    let ex = (qx as f32) * (1.0 / 2047.0) - 1.0;
    let ey = (qy as f32) * (1.0 / 2047.0) - 1.0;
    oct_decode_unit_normal([ex, ey])
}

/// Compact 32-byte aligned GPU Tri-Dexel Surface-Nets quad record emitted by `tridexel_emit.wgsl`
/// and expanded into 6 vertices (2 watertight snapped triangles with per-corner crease-aware smooth
/// normals) on-the-fly by `vs_sim_tridexel_gpu`.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct PackedTriDexelQuad {
    /// Bits `0..20`: `global_ray` (up to `1,048,575`; Ultra `448³` is `602,112`),
    /// Bits `20..30`: voxel face plane index `dw` (`0..=1023`; Ultra `448³` is `0..=448`),
    /// Bit `31`: `want_min` (`1` = negative outward normal, `0` = positive outward normal).
    pub ray_dw_flags: u32,
    /// Packed `RGBA8` unorm color (`pack4x8unorm` / `unpack4x8unorm`).
    pub color_rgba8: u32,
    /// Corner 0 sub-voxel dual offset `(du, dv, dw)` in `snorm8×3` + 4+4-bit octahedral refinement in W.
    pub corner0_pad: u32,
    /// Corner 1 sub-voxel dual offset `(du, dv, dw)` in `snorm8×3` + 4+4-bit octahedral refinement in W.
    pub corner1_pad: u32,
    /// Corner 2 sub-voxel dual offset `(du, dv, dw)` in `snorm8×3` + 4+4-bit octahedral refinement in W.
    pub corner2_pad: u32,
    /// Corner 3 sub-voxel dual offset `(du, dv, dw)` in `snorm8×3` + 4+4-bit octahedral refinement in W.
    pub corner3_pad: u32,
    /// High-8-bit octahedral-encoded unit normals for corners 0 & 1 (`x0, y0, x1, y1`).
    pub n01_oct: u32,
    /// High-8-bit octahedral-encoded unit normals for corners 2 & 3 (`x2, y2, x3, y3`).
    pub n23_oct: u32,
}

/// Byte size of one [`PackedTriDexelQuad`] storage element (`32 bytes`).
pub const TRIDEXEL_PACKED_QUAD_BYTES: usize = std::mem::size_of::<PackedTriDexelQuad>();

const _: () = assert!(TRIDEXEL_PACKED_QUAD_BYTES == 32);

/// Maximum emitted quads per Tri-Dexel GPU indirect draw (`4_000_000 × 32 B = 128.0 MB < 128 MiB`
/// WebGPU default `max_storage_buffer_binding_size`).
pub const MAX_TRIDEXEL_EMIT_QUADS: u32 = 4_000_000;

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
    /// `[color_mode_id, max_quads, mesh_skinning (0/1), 0]`
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
        options: SimMeshOptions,
    ) -> Self {
        let (mode_id, tol_mm, max_scale_mm) = match options.color_mode {
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
            mode_and_cap: [mode_id, max_quads, u32::from(options.mesh_skinning), 0],
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
    emitted_options: Mutex<Option<SimMeshOptions>>,
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
        options: SimMeshOptions,
    ) -> Self {
        let emit_res = ctx.tridexel_emit_resources();
        let emit_bind_group =
            Arc::new(ctx.device().create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("chip_td_emit_bg"),
                layout: &emit_res.bgl,
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
            emitted_options: Mutex::new(None),
            emit_dispatches: AtomicU64::new(0),
        };

        let mut encoder =
            data.ctx
                .device()
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("chip_td_emit_initial_encoder"),
                });
        data.ensure_emitted_for_options(options, &mut encoder);
        data.ctx.queue().submit(std::iter::once(encoder.finish()));

        data
    }

    /// Re-dispatches the lightweight `tridexel_emit.wgsl` compute pass into `encoder` ONLY when
    /// `options` (`color_mode` or `mesh_skinning`) differs from what is already baked into
    /// `quad_buf`. Returns `true` if a compute dispatch was recorded.
    pub fn ensure_emitted_for_options(
        &self,
        options: SimMeshOptions,
        encoder: &mut wgpu::CommandEncoder,
    ) -> bool {
        let mut guard = self
            .emitted_options
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if *guard == Some(options) {
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
            options,
        );
        self.ctx
            .queue()
            .write_buffer(&self.emit_uniform_buf, 0, bytemuck::bytes_of(&uniforms));

        let emit_res = self.ctx.tridexel_emit_resources();
        encoder.copy_buffer_to_buffer(&emit_res.init_args_buf, 0, &self.indirect_args_buf, 0, 16);

        let num_wg = self.total_rays.div_ceil(64).max(1);
        let wg_x = num_wg.min(65_535);
        let wg_y = num_wg.div_ceil(65_535);
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("chip_td_emit_pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&emit_res.pipeline);
            pass.set_bind_group(0, &*self.emit_bind_group, &[]);
            pass.dispatch_workgroups(wg_x, wg_y, 1);
        }

        *guard = Some(options);
        self.emit_dispatches.fetch_add(1, Ordering::Relaxed);
        true
    }

    /// Synchronously re-emits `quad_buf` on the GPU when `options` changed (convenience wrapper
    /// for standalone/live toggle outside an active render-pass command encoder).
    pub fn refresh_for_options(&self, options: SimMeshOptions) -> bool {
        let mut encoder =
            self.ctx
                .device()
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("chip_td_emit_refresh_encoder"),
                });
        let dispatched = self.ensure_emitted_for_options(options, &mut encoder);
        if dispatched {
            self.ctx.queue().submit(std::iter::once(encoder.finish()));
        }
        dispatched
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

    /// Synchronously reads back all emitted [`PackedTriDexelQuad`] records from `quad_buf`.
    /// Intended for CPU-vs-GPU verification and parity tests (not the per-frame render loop).
    pub fn readback_emitted_quads(&self) -> Vec<PackedTriDexelQuad> {
        let quad_count = (self.readback_emitted_vertex_count() / 6) as usize;
        if quad_count == 0 {
            return Vec::new();
        }
        let byte_len = quad_count * TRIDEXEL_PACKED_QUAD_BYTES;
        let staging = self.ctx.device().create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_td_quads_readback"),
            size: byte_len as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut encoder =
            self.ctx
                .device()
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("chip_td_quads_copy"),
                });
        encoder.copy_buffer_to_buffer(&self.quad_buf, 0, &staging, 0, byte_len as u64);
        self.ctx.queue().submit(std::iter::once(encoder.finish()));
        let mut quads = vec![bytemuck::Zeroable::zeroed(); quad_count];
        let raw: &mut [u8] = bytemuck::cast_slice_mut(&mut quads);
        if self.ctx.readback_bytes(&staging, byte_len, raw) {
            quads
        } else {
            Vec::new()
        }
    }

    /// Expands one [`PackedTriDexelQuad`] into its 4 WCS `(pos_mm, unit_normal)` corners in
    /// outward CCW winding order, matching `vs_sim_tridexel_gpu` bit-for-bit.
    pub fn unpack_quad_corners(&self, q: &PackedTriDexelQuad) -> [([f32; 3], [f32; 3]); 4] {
        let global_ray = q.ray_dw_flags & 0xFFFFF;
        let dw_idx = (q.ray_dw_flags >> 20) & 0x3FF;
        let want_min = (q.ray_dw_flags & 0x8000_0000) != 0;

        let nx = self.dims[0];
        let ny = self.dims[1];
        let off_y = self.plane_ray_offsets[1];
        let off_z = self.plane_ray_offsets[2];

        let (axis_id, plane_offset, nu, du, dv, dw, ou, ov, ow) = if global_ray < off_y {
            (
                0u32,
                self.plane_ray_offsets[0],
                ny,
                self.pitch_mm[1],
                self.pitch_mm[2],
                self.pitch_mm[0],
                self.origin_wcs[1],
                self.origin_wcs[2],
                self.origin_wcs[0],
            )
        } else if global_ray < off_z {
            (
                1u32,
                off_y,
                nx,
                self.pitch_mm[0],
                self.pitch_mm[2],
                self.pitch_mm[1],
                self.origin_wcs[0],
                self.origin_wcs[2],
                self.origin_wcs[1],
            )
        } else {
            (
                2u32,
                off_z,
                nx,
                self.pitch_mm[0],
                self.pitch_mm[1],
                self.pitch_mm[2],
                self.origin_wcs[0],
                self.origin_wcs[1],
                self.origin_wcs[2],
            )
        };

        let local_ray = global_ray - plane_offset;
        let iu = local_ray % nu.max(1);
        let iv = local_ray / nu.max(1);

        let unpack_snorm8x3 = |w: u32| -> [f32; 3] {
            let b0 = (w & 0xFF) as u8 as i8;
            let b1 = ((w >> 8) & 0xFF) as u8 as i8;
            let b2 = ((w >> 16) & 0xFF) as u8 as i8;
            [
                (f32::from(b0) / 127.0).clamp(-1.0, 1.0) * TRIDEXEL_DUAL_OFFSET_PACK_SCALE,
                (f32::from(b1) / 127.0).clamp(-1.0, 1.0) * TRIDEXEL_DUAL_OFFSET_PACK_SCALE,
                (f32::from(b2) / 127.0).clamp(-1.0, 1.0) * TRIDEXEL_DUAL_OFFSET_PACK_SCALE,
            ]
        };

        let corner_words = [q.corner0_pad, q.corner1_pad, q.corner2_pad, q.corner3_pad];
        let steps: [(u32, u32); 4] = if axis_id == 1 {
            [(0, 0), (0, 1), (1, 1), (1, 0)]
        } else {
            [(0, 0), (1, 0), (1, 1), (0, 1)]
        };

        let mut raw_corners = [([0.0_f32; 3], [0.0_f32; 3]); 4];
        for ci in 0..4 {
            let off = unpack_snorm8x3(corner_words[ci]);
            let u = ou + ((iu + steps[ci].0) as f32 + off[0]) * du;
            let v = ov + ((iv + steps[ci].1) as f32 + off[1]) * dv;
            let w = ow + (dw_idx as f32 + off[2]) * dw;
            let pos = match axis_id {
                0 => [w, u, v],
                1 => [u, w, v],
                _ => [u, v, w],
            };
            let n = if ci < 2 {
                unpack_oct_normal_12bit(q.n01_oct, ci == 1, corner_words[ci])
            } else {
                unpack_oct_normal_12bit(q.n23_oct, ci == 3, corner_words[ci])
            };
            raw_corners[ci] = (pos, n);
        }

        if want_min {
            [
                raw_corners[0],
                raw_corners[3],
                raw_corners[2],
                raw_corners[1],
            ]
        } else {
            raw_corners
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
        Self::from_simulated_stock_with_options(
            ctx,
            stock,
            target_tridexel,
            SimMeshOptions::from(color_mode),
        )
    }

    /// Uploads flat stock buffers from a [`SimulatedStock`] snapshot directly onto `ctx`'s
    /// presentation `wgpu::Device` using visual [`SimMeshOptions`] (`color_mode` + optional
    /// `mesh_skinning`).
    pub fn from_simulated_stock_with_options(
        ctx: &Arc<GpuSimContext>,
        stock: &SimulatedStock,
        target_tridexel: Option<&TriDexelStock>,
        options: SimMeshOptions,
    ) -> Arc<Self> {
        let color_mode = options.color_mode;
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
                    usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
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
                    options,
                );
                Arc::new(Self::TriDexel(data))
            }
        }
    }

    /// Refreshes any GPU-baked render geometry (such as `TriDexel` indirect quads) to match
    /// `options` without re-running simulation sweeps. Returns `true` if a GPU compute pass ran.
    pub fn refresh_for_options(&self, options: SimMeshOptions) -> bool {
        match self {
            Self::Heightfield(_) | Self::Cylindrical(_) => false,
            Self::TriDexel(t) => t.refresh_for_options(options),
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
