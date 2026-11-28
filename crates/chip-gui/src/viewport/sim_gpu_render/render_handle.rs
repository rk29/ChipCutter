//! Zero-readback GPU stock render handles (`GpuStockRenderHandle`) wrapping immutable
//! `chip_stock::GpuStockBufferSnapshot` buffers and driving viewport-side Tri-Dexel Surface Nets
//! emission (`tridexel_emit.wgsl` + `draw_indirect`) directly inside `chip-gui`.

use std::sync::{
    Arc, Mutex, Weak,
    atomic::{AtomicU64, Ordering},
};

use chip_stock::{
    GpuCylindricalBuffers, GpuHeightfieldBuffers, GpuSimContext, GpuStockBufferSnapshot,
    GpuTriDexelBuffers, INLINE_DEXEL_SLOTS, MACHINED_METAL_COLOR, OPERATION_STOCK_PALETTE,
    RAW_STOCK_SIDE_COLOR, RAW_STOCK_TOP_COLOR, SimMeshOptions, SimStockColorMode, SimStockMesh,
    SimulatedStock, TriDexelOverflowData, TriDexelStock, build_tridexel_overflow_supplement_mesh,
};
use wgpu::util::DeviceExt as _;

/// Scale factor used to pack Tri-Dexel dual corner offsets `(du, dv, dw)` into `snorm8×3`.
/// Covers `[-1.0, +1.0]` voxel pitch so both crisp (`±0.55`) and relaxed skinned (`±0.95`)
/// dual corners encode cleanly without clipping.
#[cfg(test)]
pub const TRIDEXEL_DUAL_OFFSET_PACK_SCALE: f32 = 1.0;

/// Encodes a 3D unit normal into signed 2D octahedral coordinates in `[-1.0, 1.0]²`.
#[cfg(test)]
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
#[cfg(test)]
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
/// packed into the spare high byte of `corner*_pad`.
#[cfg(test)]
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
#[cfg(test)]
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

/// Cached viewport-side Tri-Dexel Surface Nets compute pipeline resources per `wgpu::Device`.
pub(crate) struct TriDexelEmitResources {
    pub(crate) bgl: wgpu::BindGroupLayout,
    pub(crate) pipeline: wgpu::ComputePipeline,
    pub(crate) init_args_buf: wgpu::Buffer,
}

fn storage_entry(binding: u32, read_only: bool) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

fn uniform_entry(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

fn build_tridexel_emit_pipeline(device: &wgpu::Device) -> TriDexelEmitResources {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("chip_tridexel_emit_wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("tridexel_emit.wgsl").into()),
    });

    // 1 uniform + 6 storage buffers (strictly within WebGPU default limit of 8 storage buffers/stage)
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("chip_td_emit_bgl"),
        entries: &[
            uniform_entry(0),        // TriDexelEmitUniforms
            storage_entry(1, true),  // m_slots (ro)
            storage_entry(2, true),  // m_counts (ro)
            storage_entry(3, true),  // t_slots (ro)
            storage_entry(4, true),  // t_counts (ro)
            storage_entry(5, false), // out_quads (rw)
            storage_entry(6, false), // draw_args (rw atomic)
        ],
    });

    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("chip_td_emit_layout"),
        bind_group_layouts: &[Some(&bgl)],
        immediate_size: 0,
    });

    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("chip_td_emit_pipeline"),
        layout: Some(&layout),
        module: &shader,
        entry_point: Some("main"),
        compilation_options: wgpu::PipelineCompilationOptions::default(),
        cache: None,
    });

    // DrawIndirectArgs { vertex_count: 0, instance_count: 1, first_vertex: 0, first_instance: 0 }
    let init_args: [u32; 4] = [0, 1, 0, 0];
    let init_args_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("chip_td_emit_init_args"),
        contents: bytemuck::cast_slice(&init_args),
        usage: wgpu::BufferUsages::COPY_SRC,
    });

    TriDexelEmitResources {
        bgl,
        pipeline,
        init_args_buf,
    }
}

/// Returns the cached `TriDexelEmitResources` compiled for `device` (keyed by weak `Arc<Device>`
/// identity so dropped devices are pruned automatically and FXC compilation is serialized).
pub(crate) fn tridexel_emit_resources_for_device(
    device: &Arc<wgpu::Device>,
) -> Arc<TriDexelEmitResources> {
    static CACHE: Mutex<Vec<(Weak<wgpu::Device>, Arc<TriDexelEmitResources>)>> =
        Mutex::new(Vec::new());
    let mut cache = CACHE.lock().unwrap_or_else(|e| e.into_inner());
    cache.retain(|(weak_dev, _)| weak_dev.strong_count() > 0);
    let dev_weak = Arc::downgrade(device);
    if let Some((_, res)) = cache.iter().find(|(w, _)| Weak::ptr_eq(w, &dev_weak)) {
        return Arc::clone(res);
    }
    let built = Arc::new(build_tridexel_emit_pipeline(device));
    cache.push((dev_weak, Arc::clone(&built)));
    built
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

/// Read-only GPU indirect-draw compact quad buffer & `DrawIndirectArgs` handle produced by
/// `tridexel_emit.wgsl`, supporting instant `< 0.3 ms` GPU-only recoloring on `color_mode` or
/// `tolerance_mm` changes without touching the CPU.
pub struct GpuTriDexelRenderData {
    ctx: Arc<GpuSimContext>,
    emit_res: Arc<TriDexelEmitResources>,
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
    overflow_data: Option<Arc<TriDexelOverflowData>>,
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
    pub(crate) fn from_buffers(
        ctx: Arc<GpuSimContext>,
        buffers: &GpuTriDexelBuffers,
        options: SimMeshOptions,
    ) -> Self {
        let device = ctx.device();
        let max_quads = compute_tridexel_max_quads(device, buffers.total_rays as usize);
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

        let overflow_mesh = buffers.overflow_data.as_ref().and_then(|ov| {
            build_tridexel_overflow_supplement_mesh(
                &ov.stock,
                ov.target.as_ref(),
                options.color_mode,
            )
        });

        let emit_res = tridexel_emit_resources_for_device(ctx.device_arc());
        let emit_bind_group = Arc::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chip_td_emit_bg"),
            layout: &emit_res.bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: emit_uniform_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: buffers.m_slots_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: buffers.m_counts_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: buffers.t_slots_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: buffers.t_counts_buf.as_entire_binding(),
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
            emit_res,
            dims: buffers.dims,
            origin_wcs: buffers.origin_wcs,
            pitch_mm: buffers.pitch_mm,
            plane_ray_offsets: buffers.plane_ray_offsets,
            total_rays: buffers.total_rays,
            has_target: buffers.has_target,
            quad_buf,
            indirect_args_buf,
            max_quads,
            max_vertices: max_quads.saturating_mul(6),
            overflow_data: buffers.overflow_data.clone(),
            overflow_mesh,
            _m_slots_buf: Arc::clone(&buffers.m_slots_buf),
            _m_counts_buf: Arc::clone(&buffers.m_counts_buf),
            _t_slots_buf: Arc::clone(&buffers.t_slots_buf),
            _t_counts_buf: Arc::clone(&buffers.t_counts_buf),
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

        encoder.copy_buffer_to_buffer(
            &self.emit_res.init_args_buf,
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
            pass.set_pipeline(&self.emit_res.pipeline);
            pass.set_bind_group(0, &*self.emit_bind_group, &[]);
            pass.dispatch_workgroups(wg_x, wg_y, 1);
        }

        *guard = Some(options);
        self.emit_dispatches.fetch_add(1, Ordering::Relaxed);
        true
    }

    /// Rebuilds the optional rare CPU overflow supplement mesh for `color_mode` if overflow rays exist.
    pub(crate) fn overflow_mesh_for_color_mode(
        &self,
        color_mode: SimStockColorMode,
    ) -> Option<Arc<SimStockMesh>> {
        if let Some(ov) = &self.overflow_data {
            build_tridexel_overflow_supplement_mesh(&ov.stock, ov.target.as_ref(), color_mode)
        } else {
            self.overflow_mesh.clone()
        }
    }

    /// Synchronously re-emits `quad_buf` on the GPU when `options` changed.
    #[cfg(test)]
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
    #[cfg(test)]
    #[inline]
    pub fn emit_dispatch_count(&self) -> u64 {
        self.emit_dispatches.load(Ordering::Relaxed)
    }

    /// Synchronously reads back the emitted vertex count from `indirect_args_buf` (`draw_args[0]`).
    #[cfg(test)]
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
    #[cfg(test)]
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
    #[cfg(test)]
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

/// Polymorphic read-only GPU stock render handle for zero-readback direct rendering inside
/// `chip-gui`'s `ViewportRenderer`.
#[derive(Debug)]
pub enum GpuStockRenderHandle {
    Heightfield(GpuHeightfieldBuffers),
    Cylindrical(GpuCylindricalBuffers),
    TriDexel(GpuTriDexelRenderData),
}

impl GpuStockRenderHandle {
    /// Wraps a raw read-only [`GpuStockBufferSnapshot`] from `chip-stock` into a viewport render
    /// handle (dispatching `tridexel_emit.wgsl` for `TriDexel` snapshots).
    pub fn from_buffer_snapshot(
        ctx: &Arc<GpuSimContext>,
        snapshot: &GpuStockBufferSnapshot,
        options: SimMeshOptions,
    ) -> Arc<Self> {
        match snapshot {
            GpuStockBufferSnapshot::Heightfield(hf) => Arc::new(Self::Heightfield(hf.clone())),
            GpuStockBufferSnapshot::Cylindrical(cyl) => Arc::new(Self::Cylindrical(cyl.clone())),
            GpuStockBufferSnapshot::TriDexel(td) => Arc::new(Self::TriDexel(
                GpuTriDexelRenderData::from_buffers(Arc::clone(ctx), td, options),
            )),
        }
    }

    /// Uploads flat stock buffers from a [`SimulatedStock`] snapshot directly onto `ctx`'s
    /// presentation `wgpu::Device` (zero CPU mesh tessellation).
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
    /// presentation `wgpu::Device` using visual [`SimMeshOptions`].
    pub fn from_simulated_stock_with_options(
        ctx: &Arc<GpuSimContext>,
        stock: &SimulatedStock,
        target_tridexel: Option<&TriDexelStock>,
        options: SimMeshOptions,
    ) -> Arc<Self> {
        let snapshot = GpuStockBufferSnapshot::from_simulated_stock(ctx, stock, target_tridexel);
        Self::from_buffer_snapshot(ctx, &snapshot, options)
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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::collections::HashMap;

    use chip_math::{Aabb3, DVec3, DisplayUnit};
    use chip_stock::{
        Axis3, RadialCutterProfile, RotaryStockSpec, SimColoredVertex, SimResolution,
        SimStockBackend, StockShapeSpec, StockVerifyTolerances, SweepChord, SweepMotion,
        SweepToolSpec, WgpuComputeBackend, tessellate_tridexel_stock_mesh_colored,
    };
    use chip_tools::{CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId};

    use super::*;

    fn make_tool(id: &str, stickout_mm: f64, geometry: CutterGeometry) -> SweepToolSpec {
        let is_flat_endmill = matches!(geometry, CutterGeometry::FlatEndmill { .. });
        let tool = Tool {
            id: ToolId(id.to_string()),
            tool_number: 1,
            length_offset_number: 1,
            diameter_offset_number: 1,
            name: id.to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::Mist,
            stickout_length_mm: stickout_mm,
            geometry,
            presets: Vec::new(),
        };
        SweepToolSpec {
            profile: RadialCutterProfile::from_tool(&tool),
            is_flat_endmill,
        }
    }

    fn dot3(a: [f32; 3], b: [f32; 3]) -> f32 {
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    }

    fn angle_deg(a: [f32; 3], b: [f32; 3]) -> f32 {
        let va = DVec3::new(f64::from(a[0]), f64::from(a[1]), f64::from(a[2]));
        let vb = DVec3::new(f64::from(b[0]), f64::from(b[1]), f64::from(b[2]));
        va.cross(vb).length().atan2(va.dot(vb)).to_degrees() as f32
    }

    fn measure_quad_mesh_smoothness(mesh: &SimStockMesh) -> (f64, usize, usize) {
        assert_eq!(mesh.vertices.len() % 4, 0, "expected quad vertex layout");
        let num_quads = mesh.vertices.len() / 4;
        let mut non_flat_quads = 0usize;
        let mut vertex_normals: HashMap<[i64; 3], Vec<[f32; 3]>> = HashMap::new();

        for qi in 0..num_quads {
            let vs = &mesh.vertices[qi * 4..qi * 4 + 4];
            let mut max_spread = 0.0_f32;
            for a in 0..4 {
                for b in (a + 1)..4 {
                    max_spread = max_spread.max(angle_deg(vs[a].normal, vs[b].normal));
                }
                let key = [
                    (f64::from(vs[a].pos_mm[0]) * 1000.0).round() as i64,
                    (f64::from(vs[a].pos_mm[1]) * 1000.0).round() as i64,
                    (f64::from(vs[a].pos_mm[2]) * 1000.0).round() as i64,
                ];
                vertex_normals.entry(key).or_default().push(vs[a].normal);
            }
            if max_spread > 0.1 {
                non_flat_quads += 1;
            }
        }

        let mut angle_sum = 0.0_f64;
        let mut pair_count = 0usize;
        for ns in vertex_normals.values() {
            if ns.len() < 2 {
                continue;
            }
            for a in 0..ns.len() {
                for b in (a + 1)..ns.len() {
                    let d = dot3(ns[a], ns[b]);
                    if d >= 0.788 {
                        angle_sum += f64::from(d.clamp(-1.0, 1.0).acos().to_degrees());
                        pair_count += 1;
                    }
                }
            }
        }
        let mean_discont = if pair_count > 0 {
            angle_sum / pair_count as f64
        } else {
            0.0
        };
        (mean_discont, non_flat_quads, num_quads)
    }

    fn build_curved_ball_cavity_tridexel() -> TriDexelStock {
        let aabb = Aabb3::new(DVec3::new(-20.0, -20.0, -16.0), DVec3::new(20.0, 20.0, 0.0));
        let mut stock = TriDexelStock::from_stock_spec(
            &StockShapeSpec::Box { aabb_wcs: aabb },
            SimResolution::Standard,
        )
        .unwrap();
        let ball = make_tool(
            "ball_12",
            30.0,
            CutterGeometry::BallEndmill {
                diameter_mm: 12.0,
                flute_length_mm: 18.0,
                overall_length_mm: 50.0,
                shank_diameter_mm: 12.0,
                flutes: 2,
            },
        );
        let tol = StockVerifyTolerances::default();
        let mut chords = Vec::new();
        for iy in -10..=10 {
            let y = iy as f64 * 1.2;
            let mut prev: Option<DVec3> = None;
            for ix in -12..=12 {
                let x = ix as f64 * 1.0;
                let r2 = x * x + y * y;
                let z = if r2 < 144.0 {
                    -2.0 - 8.0 * (1.0 - r2 / 144.0).sqrt()
                } else {
                    -2.0
                };
                let p = DVec3::new(x, y, z);
                if let Some(p0) = prev {
                    chords.push(SweepChord {
                        motion: SweepMotion::Linear {
                            p0,
                            p1: p,
                            rotary_deg_0: 0.0,
                            rotary_deg_1: 0.0,
                        },
                        op_idx: 0,
                        is_rapid: false,
                        profile_idx: 0,
                        verify_tol: tol,
                    });
                }
                prev = Some(p);
            }
        }
        let _ = stock.sweep_chords(&chords, &[ball], None, false);
        stock
    }

    fn build_rectangular_pocket_tridexel() -> TriDexelStock {
        let aabb = Aabb3::new(DVec3::new(-20.0, -20.0, -16.0), DVec3::new(20.0, 20.0, 0.0));
        let mut stock = TriDexelStock::from_stock_spec(
            &StockShapeSpec::Box { aabb_wcs: aabb },
            SimResolution::Standard,
        )
        .unwrap();
        let (nx, ny, nz) = (stock.nx(), stock.ny(), stock.nz());
        for j in 0..ny {
            let py = stock.z_plane().center_v(j);
            if (-8.0..=8.0).contains(&py) {
                for i in 0..nx {
                    let px = stock.z_plane().center_u(i);
                    if (-10.0..=10.0).contains(&px) {
                        stock
                            .plane_mut(Axis3::Z)
                            .subtract_ray_interval(i, j, -7.0, 0.0, 1);
                    }
                }
            }
        }
        for k in 0..nz {
            let pz = stock.x_plane().center_v(k);
            if pz >= -7.0 {
                for j in 0..ny {
                    let py = stock.x_plane().center_u(j);
                    if (-8.0..=8.0).contains(&py) {
                        stock
                            .plane_mut(Axis3::X)
                            .subtract_ray_interval(j, k, -10.0, 10.0, 1);
                    }
                }
                for i in 0..nx {
                    let px = stock.y_plane().center_u(i);
                    if (-10.0..=10.0).contains(&px) {
                        stock
                            .plane_mut(Axis3::Y)
                            .subtract_ray_interval(i, k, -8.0, 8.0, 1);
                    }
                }
            }
        }
        stock
    }

    #[test]
    fn octahedral_12bit_normal_roundtrip_accuracy_under_half_degree() {
        assert_eq!(std::mem::size_of::<PackedTriDexelQuad>(), 32);
        assert_eq!(TRIDEXEL_PACKED_QUAD_BYTES, 32);

        let cardinals = [
            [1.0_f32, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
        ];
        for &n in &cardinals {
            let (hi, lo0, lo1) = pack_oct_normal_pair_12bit(n, [-n[0], -n[1], -n[2]]);
            let u0 = unpack_oct_normal_12bit(hi, false, u32::from(lo0) << 24);
            let u1 = unpack_oct_normal_12bit(hi, true, u32::from(lo1) << 24);
            assert!(
                angle_deg(n, u0) < 1e-3,
                "cardinal {n:?} roundtrip failed: got {u0:?}"
            );
            assert!(
                angle_deg([-n[0], -n[1], -n[2]], u1) < 1e-3,
                "negated cardinal roundtrip failed: got {u1:?}"
            );
        }

        let golden = std::f64::consts::PI * (3.0 - 5.0_f64.sqrt());
        let mut dirs: Vec<[f32; 3]> = Vec::with_capacity(2008);
        for i in 0..2000 {
            let y = 1.0 - (i as f64 / 1999.0) * 2.0;
            let r = (1.0 - y * y).max(0.0).sqrt();
            let theta = golden * (i as f64);
            let x = theta.cos() * r;
            let z = theta.sin() * r;
            dirs.push([x as f32, y as f32, z as f32]);
        }
        let inv_s3 = 1.0 / 3.0_f32.sqrt();
        for sx in [-1.0_f32, 1.0] {
            for sy in [-1.0_f32, 1.0] {
                for sz in [-1.0_f32, 1.0] {
                    dirs.push([sx * inv_s3, sy * inv_s3, sz * inv_s3]);
                }
            }
        }

        let mut max_err_deg = 0.0_f32;
        for &[n_a, n_b] in dirs.as_chunks::<2>().0 {
            let fa = oct_decode_unit_normal(oct_encode_unit_normal(n_a));
            assert!(angle_deg(n_a, fa) < 1e-3);

            let (hi, lo_a, lo_b) = pack_oct_normal_pair_12bit(n_a, n_b);
            let dec_a = unpack_oct_normal_12bit(hi, false, u32::from(lo_a) << 24);
            let dec_b = unpack_oct_normal_12bit(hi, true, u32::from(lo_b) << 24);
            max_err_deg = max_err_deg.max(angle_deg(n_a, dec_a));
            max_err_deg = max_err_deg.max(angle_deg(n_b, dec_b));
        }

        assert!(
            max_err_deg < 0.5,
            "octahedral 12-bit normal roundtrip worst-case error {max_err_deg:.4}° >= 0.5°"
        );
    }

    #[test]
    fn gpu_tridexel_skinning_matches_cpu_smoothness_watertightness_and_sharp_creases() {
        let Some(ctx) = GpuSimContext::shared_headless() else {
            return;
        };

        let curved_stock = build_curved_ball_cavity_tridexel();
        let min_pitch = curved_stock
            .pitch_mm
            .x
            .min(curved_stock.pitch_mm.y)
            .min(curved_stock.pitch_mm.z);

        let mut gpu_be = WgpuComputeBackend::new(
            Arc::clone(&ctx),
            SimulatedStock::TriDexel(Box::new(curved_stock.clone())),
            None,
        );
        let snap = gpu_be.capture_buffer_snapshot();
        let handle = GpuStockRenderHandle::from_buffer_snapshot(
            &ctx,
            &snap,
            SimMeshOptions::new(SimStockColorMode::ByOperation, false),
        );
        let GpuStockRenderHandle::TriDexel(ref td_render) = *handle else {
            panic!("expected TriDexel GPU render handle");
        };
        assert_eq!(td_render.emit_dispatch_count(), 1);
        let crisp_quads = td_render.readback_emitted_quads();
        assert!(crisp_quads.len() > 1000);

        assert!(
            td_render
                .refresh_for_options(SimMeshOptions::new(SimStockColorMode::ByOperation, true))
        );
        assert_eq!(td_render.emit_dispatch_count(), 2);
        assert!(
            !td_render
                .refresh_for_options(SimMeshOptions::new(SimStockColorMode::ByOperation, true))
        );
        assert_eq!(td_render.emit_dispatch_count(), 2);

        let skinned_quads = td_render.readback_emitted_quads();
        assert_eq!(crisp_quads.len(), skinned_quads.len());

        let quads_to_mesh = |quads: &[PackedTriDexelQuad]| -> SimStockMesh {
            let mut vertices = Vec::with_capacity(quads.len() * 4);
            let mut indices = Vec::with_capacity(quads.len() * 6);
            for (qi, q) in quads.iter().enumerate() {
                let corners = td_render.unpack_quad_corners(q);
                let base = (qi as u32) * 4;
                for (pos, n) in corners {
                    vertices.push(SimColoredVertex {
                        pos_mm: pos,
                        normal: n,
                        color_rgba: [1.0; 4],
                    });
                }
                indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
            }
            SimStockMesh { vertices, indices }
        };

        let gpu_crisp_mesh = quads_to_mesh(&crisp_quads);
        let gpu_skinned_mesh = quads_to_mesh(&skinned_quads);

        let (gpu_crisp_disc, gpu_crisp_non_flat, _) = measure_quad_mesh_smoothness(&gpu_crisp_mesh);
        let (gpu_skin_disc, gpu_skin_non_flat, gpu_total_quads) =
            measure_quad_mesh_smoothness(&gpu_skinned_mesh);

        assert_eq!(
            gpu_crisp_non_flat, 0,
            "GPU crisp mode packs 1 normal per quad"
        );
        assert!(
            gpu_skin_non_flat > gpu_total_quads / 10,
            "GPU skinned mode must emit distinct per-corner normals on curved quads (got {gpu_skin_non_flat}/{gpu_total_quads})"
        );
        assert!(
            gpu_skin_disc < gpu_crisp_disc * 0.65,
            "GPU skinned mode must reduce normal discontinuity across adjacent curved quads (skinned={gpu_skin_disc:.3}° vs crisp={gpu_crisp_disc:.3}°)"
        );

        let max_allowed = 1.15 * min_pitch + 0.02;
        let mut max_gpu_disp = 0.0_f64;
        let mut corner_map: HashMap<(u32, u32, u32), [f32; 3]> = HashMap::new();
        let nx = td_render.dims[0];
        let ny = td_render.dims[1];
        let off_y = td_render.plane_ray_offsets[1];
        let off_z = td_render.plane_ray_offsets[2];

        let crisp_by_key: HashMap<u32, PackedTriDexelQuad> =
            crisp_quads.iter().map(|q| (q.ray_dw_flags, *q)).collect();
        for sq in &skinned_quads {
            let cq = crisp_by_key
                .get(&sq.ray_dw_flags)
                .expect("matching crisp quad must exist");
            let sc = td_render.unpack_quad_corners(sq);
            let cc = td_render.unpack_quad_corners(cq);
            for ci in 0..4 {
                let dx = f64::from(sc[ci].0[0]) - f64::from(cc[ci].0[0]);
                let dy = f64::from(sc[ci].0[1]) - f64::from(cc[ci].0[1]);
                let dz = f64::from(sc[ci].0[2]) - f64::from(cc[ci].0[2]);
                max_gpu_disp = max_gpu_disp.max((dx * dx + dy * dy + dz * dz).sqrt());
            }

            let global_ray = sq.ray_dw_flags & 0xFFFFF;
            let dw_idx = (sq.ray_dw_flags >> 20) & 0x3FF;
            let want_min = (sq.ray_dw_flags & 0x8000_0000) != 0;
            let (axis_id, plane_offset, nu) = if global_ray < off_y {
                (0u32, td_render.plane_ray_offsets[0], ny)
            } else if global_ray < off_z {
                (1u32, off_y, nx)
            } else {
                (2u32, off_z, nx)
            };
            let local_ray = global_ray - plane_offset;
            let iu = local_ray % nu.max(1);
            let iv = local_ray / nu.max(1);
            let steps: [(u32, u32); 4] = if axis_id == 1 {
                [(0, 0), (0, 1), (1, 1), (1, 0)]
            } else {
                [(0, 0), (1, 0), (1, 1), (0, 1)]
            };
            let order: [usize; 4] = if want_min { [0, 3, 2, 1] } else { [0, 1, 2, 3] };
            for (slot, &raw_ci) in order.iter().enumerate() {
                let cu = iu + steps[raw_ci].0;
                let cv = iv + steps[raw_ci].1;
                let c_xyz = match axis_id {
                    0 => (dw_idx, cu, cv),
                    1 => (cu, dw_idx, cv),
                    _ => (cu, cv, dw_idx),
                };
                let pos = sc[slot].0;
                if let Some(&prev) = corner_map.get(&c_xyz) {
                    let err = ((pos[0] - prev[0]).powi(2)
                        + (pos[1] - prev[1]).powi(2)
                        + (pos[2] - prev[2]).powi(2))
                    .sqrt();
                    assert!(
                        err < 1e-4,
                        "GPU Tri-Dexel crack at shared dual corner {c_xyz:?}: {pos:?} vs {prev:?} (err={err:.6})"
                    );
                } else {
                    corner_map.insert(c_xyz, pos);
                }
            }
        }
        assert!(
            max_gpu_disp > 1e-3,
            "GPU curved surface relaxation should move unpinned vertices (max_gpu_disp={max_gpu_disp:.6})"
        );
        assert!(
            max_gpu_disp <= max_allowed,
            "GPU relaxed displacement {max_gpu_disp:.6} mm exceeded 0.45 * pitch ({max_allowed:.6} mm)"
        );

        let cpu_skinned_mesh = curved_stock.extract_mesh_with_options(
            None,
            SimMeshOptions::new(SimStockColorMode::ByOperation, true),
        );
        let (cpu_skin_disc, _, _) = measure_quad_mesh_smoothness(&cpu_skinned_mesh);
        assert!(
            cpu_skin_disc < 5.0 && gpu_skin_disc < 5.0,
            "both CPU ({cpu_skin_disc:.2}°) and GPU ({gpu_skin_disc:.2}°) skinned curved surfaces must have low normal discontinuity"
        );

        let pocket_stock = build_rectangular_pocket_tridexel();
        let pocket_handle = GpuStockRenderHandle::from_simulated_stock_with_options(
            &ctx,
            &SimulatedStock::TriDexel(Box::new(pocket_stock)),
            None,
            SimMeshOptions::new(SimStockColorMode::ByOperation, true),
        );
        let GpuStockRenderHandle::TriDexel(ref p_td) = *pocket_handle else {
            panic!("expected TriDexel GPU handle");
        };
        let p_quads = p_td.readback_emitted_quads();
        let mut floor_verts = 0usize;
        let mut wall_verts = 0usize;
        for q in &p_quads {
            let cs = p_td.unpack_quad_corners(q);
            let cx = 0.25 * (cs[0].0[0] + cs[1].0[0] + cs[2].0[0] + cs[3].0[0]);
            let cy = 0.25 * (cs[0].0[1] + cs[1].0[1] + cs[2].0[1] + cs[3].0[1]);
            let cz = 0.25 * (cs[0].0[2] + cs[1].0[2] + cs[2].0[2] + cs[3].0[2]);
            if (-9.0..=9.0).contains(&cx)
                && (-7.0..=7.0).contains(&cy)
                && (cz - (-7.0)).abs() < 0.35
            {
                for (_, n) in cs {
                    assert!(
                        n[2] > 0.99,
                        "GPU 90° pocket floor must stay flat (n.z > 0.99), got {n:?}"
                    );
                    floor_verts += 1;
                }
            }
            let on_x_wall = (cx.abs() - 10.0).abs() < 0.35 && cy.abs() < 7.2;
            let on_y_wall = (cy.abs() - 8.0).abs() < 0.35 && cx.abs() < 9.2;
            if (-6.2..=-0.8).contains(&cz) && (on_x_wall || on_y_wall) {
                for (_, n) in cs {
                    assert!(
                        n[2].abs() < 0.05,
                        "GPU 90° pocket vertical wall must stay orthogonal (|n.z| < 0.05), got {n:?}"
                    );
                    wall_verts += 1;
                }
            }
        }
        assert!(floor_verts > 200);
        assert!(wall_verts > 200);
    }

    #[test]
    fn gpu_tridexel_ultra_emit_does_not_truncate_z_plane_above_old_2_4m_cap() {
        let ctx = GpuSimContext::shared_headless().expect("expected Vulkan/wgpu compute adapter");
        let stock_spec = StockShapeSpec::Box {
            aabb_wcs: Aabb3::new(DVec3::new(-50.0, -40.0, -29.0), DVec3::new(50.0, 40.0, 0.0)),
        };
        let mut td = TriDexelStock::from_stock_spec(&stock_spec, SimResolution::Ultra).unwrap();
        assert_eq!(td.dims[0], 448);

        let slot_tools = vec![make_tool(
            "flat_8",
            25.0,
            CutterGeometry::FlatEndmill {
                diameter_mm: 8.5,
                flute_length_mm: 16.0,
                overall_length_mm: 55.0,
                shank_diameter_mm: 8.5,
                flutes: 3,
            },
        )];
        let tol = StockVerifyTolerances::default();
        let chords = vec![
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(-55.0, -20.0, -15.0),
                    p1: DVec3::new(55.0, -20.0, -15.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 0,
                is_rapid: false,
                profile_idx: 0,
                verify_tol: tol,
            },
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(-55.0, 12.0, -15.0),
                    p1: DVec3::new(55.0, 12.0, -15.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 1,
                is_rapid: false,
                profile_idx: 0,
                verify_tol: tol,
            },
        ];

        let mut gpu = WgpuComputeBackend::new(
            Arc::clone(&ctx),
            SimulatedStock::TriDexel(Box::new(td.clone())),
            None,
        );
        let _ = gpu.sweep_batch(&chords, &slot_tools, false);
        let snap = gpu.capture_buffer_snapshot();
        let handle = GpuStockRenderHandle::from_buffer_snapshot(
            &ctx,
            &snap,
            SimMeshOptions::from(SimStockColorMode::ByOperation),
        );

        let (SimulatedStock::TriDexel(td_after), _) = gpu.into_parts() else {
            panic!("expected TriDexel stock");
        };
        td = *td_after;

        let mut expected_inline_spans: u64 = 0;
        for axis in Axis3::ALL {
            let p = td.plane(axis);
            for (r, &cnt) in p.counts.iter().enumerate() {
                let c = cnt as usize;
                if c > 0 && c <= INLINE_DEXEL_SLOTS {
                    let base = r * INLINE_DEXEL_SLOTS;
                    for k in 0..c {
                        let s = p.slots[base + k];
                        if s.max_local > s.min_local {
                            expected_inline_spans += 1;
                        }
                    }
                }
            }
        }
        let expected_verts = expected_inline_spans * 2 * 6;
        assert!(
            expected_verts > 2_400_000,
            "regression fixture must exceed old 2_400_000 vertex cap (got {expected_verts})"
        );

        let GpuStockRenderHandle::TriDexel(ref td_render) = *handle else {
            panic!("expected TriDexel GPU render handle");
        };
        let emitted_verts = u64::from(td_render.readback_emitted_vertex_count());
        assert_eq!(
            emitted_verts, expected_verts,
            "Ultra Tri-Dexel GPU emitter truncated surfaces: emitted {emitted_verts} vs expected {expected_verts} (max_vertices={})",
            td_render.max_vertices
        );

        let cube_rays = 3usize * 448 * 448;
        let max_quads = compute_tridexel_max_quads(ctx.device(), cube_rays);
        let device_cap = (ctx.device().limits().max_storage_buffer_binding_size
            / TRIDEXEL_PACKED_QUAD_BYTES as u64) as u32;
        assert_eq!(max_quads, MAX_TRIDEXEL_EMIT_QUADS.min(device_cap));
        assert!(
            (max_quads as usize) * TRIDEXEL_PACKED_QUAD_BYTES <= 128 * 1024 * 1024,
            "worst-case Ultra 448³ packed quad buffer must fit inside 128 MiB"
        );
    }

    #[test]
    fn gpu_tridexel_surface_nets_emit_matches_cpu_surface_nets_on_cylinder() {
        let Some(ctx) = GpuSimContext::shared_headless() else {
            return;
        };

        let r_stock = 21.0;
        let len = 80.0;
        let stock_spec = StockShapeSpec::Cylinder {
            axis: Axis3::X,
            center_wcs: DVec3::ZERO,
            radius_mm: r_stock,
            min_axial_mm: -0.5 * len,
            max_axial_mm: 0.5 * len,
            aabb_wcs: Aabb3::new(
                DVec3::new(-0.5 * len, -r_stock, -r_stock),
                DVec3::new(0.5 * len, r_stock, r_stock),
            ),
        };
        let rotary = RotaryStockSpec {
            axis: Axis3::X,
            direction_sign: 1.0,
            axis_letter: 'A',
            center_wcs: DVec3::ZERO,
            stock_u_min: -0.5 * len,
            stock_u_max: 0.5 * len,
            stock_outer_radius_mm: r_stock,
            chuck_u_min: -60.0,
            chuck_u_max: -40.0,
            chuck_radius_mm: 32.5,
        };
        let mut td = TriDexelStock::from_stock_spec(&stock_spec, SimResolution::Standard).unwrap();
        td.rotary = Some(rotary);

        let tools = vec![make_tool(
            "t_ball_3175",
            28.0,
            CutterGeometry::BallEndmill {
                diameter_mm: 3.175,
                flute_length_mm: 14.0,
                overall_length_mm: 50.0,
                shank_diameter_mm: 3.175,
                flutes: 2,
            },
        )];
        let tol = StockVerifyTolerances::default();
        let chords = vec![SweepChord {
            motion: SweepMotion::Linear {
                p0: DVec3::new(-25.0, 0.0, 14.0),
                p1: DVec3::new(25.0, 0.0, 14.0),
                rotary_deg_0: 0.0,
                rotary_deg_1: 180.0,
            },
            op_idx: 0,
            profile_idx: 0,
            is_rapid: false,
            verify_tol: tol,
        }];

        let mut gpu = WgpuComputeBackend::new(
            Arc::clone(&ctx),
            SimulatedStock::TriDexel(Box::new(td)),
            None,
        );
        let _ = gpu.sweep_batch(&chords, &tools, false);
        let snap = gpu.capture_buffer_snapshot();
        let handle = GpuStockRenderHandle::from_buffer_snapshot(
            &ctx,
            &snap,
            SimMeshOptions::from(SimStockColorMode::ByOperation),
        );
        let GpuStockRenderHandle::TriDexel(ref td_render) = *handle else {
            panic!("expected TriDexel GPU render handle");
        };
        let gpu_quads = u64::from(td_render.readback_emitted_vertex_count()) / 6;

        let (SimulatedStock::TriDexel(td_after), _) = gpu.into_parts() else {
            panic!("expected TriDexel stock");
        };
        let cpu_mesh =
            tessellate_tridexel_stock_mesh_colored(&td_after, None, SimStockColorMode::ByOperation);
        let cpu_quads = (cpu_mesh.indices.len() / 6) as u64;
        assert!(
            cpu_quads > 10_000,
            "expected >10k Surface Nets quads on machined cylinder, got {cpu_quads}"
        );
        let ratio = gpu_quads as f64 / cpu_quads as f64;
        assert!(
            (0.95..=1.05).contains(&ratio),
            "GPU Surface Nets quad count ({gpu_quads}) must closely match CPU Surface Nets ({cpu_quads}), ratio={ratio:.4}"
        );
    }
}
