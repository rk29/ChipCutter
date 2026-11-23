//! GPU-resident axis-generic flat fixed-`K` (`K = 4`) Tri-Dexel compute sweeper
//! (`GpuTriDexelState`) driving `tridexel_sweep.wgsl` across all 3 orthogonal planes (`X`, `Y`, `Z`)
//! in a single GPU command submission with a lossless hybrid CPU overflow safety valve.

use std::{
    collections::BTreeMap,
    sync::{Arc, Mutex},
};

use chip_bvh::TriMesh3D;
use chip_math::{Aabb3, DVec3};
use wgpu::util::DeviceExt as _;

use crate::{
    backend::StockCheckpointSnapshot,
    mesher_common::op_color,
    metrics::{
        SimStockColorMode, deviation_to_heatmap_rgba, signed_deviation_at_machined_endpoint,
    },
    tridexel::{
        DexelSpan, FlatDexelSpan, INLINE_DEXEL_SLOTS, TriDexelStock,
        sweep::{
            LinearChord3D, SweptPrimitive, decompose_chord_for_tridexel, oriented_pose_stock_aabb,
            replay_single_ray_primitive,
        },
    },
    types::{
        Axis3, BatchSweepOutput, RotaryStockSpec, SimColoredVertex, SimMeshOptions, SimStockMesh,
        SpanRuleViolations, StockVerifyTolerances, SweepChord, SweepToolSpec,
    },
};

use super::{
    checkpoints::GpuCheckpointBuffers,
    context::{GpuSimContext, GrowableStorageBuffer},
    render_handle::{
        GpuStockRenderHandle, GpuTriDexelRenderData, TRIDEXEL_PACKED_QUAD_BYTES,
        TriDexelEmitUniforms, compute_tridexel_max_quads,
    },
    types::{
        CHORD_CARDINAL_MASK, CHORD_CARDINAL_SHIFT, CHORD_FLAG_CHECK_HOLDER,
        CHORD_FLAG_ORIENTED_POSE, CHORD_FLAG_RAPID, CHORD_FLAG_VERIFY, GPU_TILE_SIZE,
        GpuSweepChord, GpuToolProfile, GpuViolationAccum, TriDexelPlaneUniforms,
    },
};

const ACCUM_WORDS: usize = std::mem::size_of::<GpuViolationAccum>() / 4; // 44 words

struct CpuShadow {
    stock: TriDexelStock,
    target: Option<TriDexelStock>,
    gpu_dirty: bool,
}

#[derive(Clone, Copy)]
struct FlattenedPrimitive {
    prim: SweptPrimitive,
    src_chord_idx: u32,
    op_idx: u32,
    profile_idx: usize,
    is_rapid: bool,
    verify_tol: StockVerifyTolerances,
    oriented_aabb: Option<(DVec3, DVec3)>,
}

struct GpuPlaneDispatch {
    axis: Axis3,
    nu: usize,
    nv: usize,
    tiles_u: usize,
    tiles_v: usize,
    ray_offset: usize,
    origin_u_mm: f64,
    origin_v_mm: f64,
    origin_w_mm: f64,
    du_mm: f64,
    dv_mm: f64,
    overflow_words: usize,
    uniform_buf: wgpu::Buffer,
    tile_data_buf: GrowableStorageBuffer,
    accum_and_overflow_buf: wgpu::Buffer,
    accum_readback_buf: wgpu::Buffer,
    overflow_readback_buf: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    tile_bins: Vec<Vec<u32>>,
    tile_data_scratch: Vec<u32>,
}

struct TriDexelRenderSnapSlot {
    m_slots: Arc<wgpu::Buffer>,
    m_counts: Arc<wgpu::Buffer>,
    quad_buf: Arc<wgpu::Buffer>,
    indirect_args_buf: Arc<wgpu::Buffer>,
    emit_uniform_buf: Arc<wgpu::Buffer>,
}

/// GPU-resident 3-axis orthogonal Tri-Dexel stock storing all 3 planes in contiguous flat `slots`
/// and `counts` storage buffers (`<= 8` storage buffers per shader stage) with lossless CPU
/// overflow safety-valve replay for rare `> K = 4`-span rays.
pub(crate) struct GpuTriDexelState {
    ctx: Arc<GpuSimContext>,
    dims: [usize; 3],
    origin_wcs: DVec3,
    pitch_mm: DVec3,
    rotary: Option<RotaryStockSpec>,
    plane_ray_offsets: [usize; 3],
    total_rays: usize,
    has_target: bool,
    m_slots_buf: wgpu::Buffer,
    m_counts_buf: wgpu::Buffer,
    t_slots_buf: Arc<wgpu::Buffer>,
    t_counts_buf: Arc<wgpu::Buffer>,
    m_slots_readback_buf: wgpu::Buffer,
    m_counts_readback_buf: wgpu::Buffer,
    render_snap_pool: Vec<TriDexelRenderSnapSlot>,
    profiles_buf: GrowableStorageBuffer,
    chords_buf: GrowableStorageBuffer,
    planes: [GpuPlaneDispatch; 3],
    gpu_chords_scratch: Vec<GpuSweepChord>,
    prim_chords_scratch: Vec<FlattenedPrimitive>,
    arc_linear_scratch: Vec<LinearChord3D>,
    prim_decompose_scratch: Vec<SweptPrimitive>,
    gpu_profiles_scratch: Vec<GpuToolProfile>,
    cpu: Mutex<CpuShadow>,
}

impl std::fmt::Debug for GpuTriDexelState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuTriDexelState")
            .field("dims", &self.dims)
            .field("has_target", &self.has_target)
            .finish()
    }
}

impl GpuTriDexelState {
    pub(crate) fn new(
        ctx: Arc<GpuSimContext>,
        stock: TriDexelStock,
        target: Option<TriDexelStock>,
    ) -> Self {
        let device = ctx.device();
        let dims = stock.dims;
        let origin_wcs = stock.origin_wcs;
        let pitch_mm = stock.pitch_mm;
        let rotary = stock.rotary;

        let rx = stock.planes[0].ray_count();
        let ry = stock.planes[1].ray_count();
        let rz = stock.planes[2].ray_count();
        let plane_ray_offsets = [0, rx, rx + ry];
        let total_rays = rx + ry + rz;
        let total_slots = total_rays * INLINE_DEXEL_SLOTS;

        let mut all_m_slots = Vec::with_capacity(total_slots);
        let mut all_m_counts = Vec::with_capacity(total_rays);
        for axis in Axis3::ALL {
            let p = stock.plane(axis);
            all_m_slots.extend_from_slice(&p.slots);
            all_m_counts.extend_from_slice(&p.counts);
        }

        let m_slots_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_td_m_slots"),
            contents: bytemuck::cast_slice(&all_m_slots),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
        });
        let m_counts_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_td_m_counts"),
            contents: bytemuck::cast_slice(&all_m_counts),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
        });

        let has_target = target.is_some();
        let (t_slots_buf, t_counts_buf) = if let Some(ref tgt) = target {
            let mut all_t_slots = Vec::with_capacity(total_slots);
            let mut all_t_counts = Vec::with_capacity(total_rays);
            for axis in Axis3::ALL {
                let p = tgt.plane(axis);
                all_t_slots.extend_from_slice(&p.slots);
                all_t_counts.extend_from_slice(&p.counts);
            }
            (
                Arc::new(
                    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("chip_td_t_slots"),
                        contents: bytemuck::cast_slice(&all_t_slots),
                        usage: wgpu::BufferUsages::STORAGE
                            | wgpu::BufferUsages::COPY_DST
                            | wgpu::BufferUsages::COPY_SRC,
                    }),
                ),
                Arc::new(
                    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("chip_td_t_counts"),
                        contents: bytemuck::cast_slice(&all_t_counts),
                        usage: wgpu::BufferUsages::STORAGE
                            | wgpu::BufferUsages::COPY_DST
                            | wgpu::BufferUsages::COPY_SRC,
                    }),
                ),
            )
        } else {
            let dummy_slot = [FlatDexelSpan::ZERO; INLINE_DEXEL_SLOTS];
            let dummy_cnt = [0u32; 4];
            (
                Arc::new(
                    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("chip_td_t_slots_dummy"),
                        contents: bytemuck::cast_slice(&dummy_slot),
                        usage: wgpu::BufferUsages::STORAGE,
                    }),
                ),
                Arc::new(
                    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("chip_td_t_counts_dummy"),
                        contents: bytemuck::cast_slice(&dummy_cnt),
                        usage: wgpu::BufferUsages::STORAGE,
                    }),
                ),
            )
        };

        let slots_bytes = (total_slots * std::mem::size_of::<FlatDexelSpan>()) as u64;
        let counts_bytes = (total_rays * std::mem::size_of::<u32>()) as u64;
        let m_slots_readback_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_td_m_slots_readback"),
            size: slots_bytes.max(16),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let m_counts_readback_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_td_m_counts_readback"),
            size: counts_bytes.max(16),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let profiles_buf = GrowableStorageBuffer::new(
            device,
            "chip_td_profiles",
            wgpu::BufferUsages::STORAGE,
            16 * std::mem::size_of::<GpuToolProfile>(),
        );
        let chords_buf = GrowableStorageBuffer::new(
            device,
            "chip_td_chords",
            wgpu::BufferUsages::STORAGE,
            128 * std::mem::size_of::<GpuSweepChord>(),
        );

        let dz_step = (0.25 * pitch_mm.z) as f32;
        let make_plane = |axis: Axis3| {
            let p = stock.plane(axis);
            let nu = p.nu;
            let nv = p.nv;
            let tiles_u = nu.div_ceil(GPU_TILE_SIZE);
            let tiles_v = nv.div_ceil(GPU_TILE_SIZE);
            let num_tiles = tiles_u * tiles_v;
            let ray_offset = plane_ray_offsets[axis.index()];
            let overflow_words = p.ray_count().div_ceil(32).max(1);

            let uniforms = TriDexelPlaneUniforms {
                nu: nu as u32,
                nv: nv as u32,
                tiles_u: tiles_u as u32,
                num_tiles: num_tiles as u32,
                axis_id: axis.index() as u32,
                has_target: u32::from(has_target),
                plane_ray_offset: ray_offset as u32,
                _pad0: 0,
                du_mm: p.du_mm as f32,
                dv_mm: p.dv_mm as f32,
                dz_step_mm: dz_step,
                _pad1: 0.0,
            };
            let uniform_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("chip_td_plane_uniforms"),
                contents: bytemuck::bytes_of(&uniforms),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            });
            let tile_data_buf = GrowableStorageBuffer::new(
                device,
                "chip_td_tile_data",
                wgpu::BufferUsages::STORAGE,
                (num_tiles + 1 + 256) * std::mem::size_of::<u32>(),
            );

            let accum_ov_bytes =
                ((ACCUM_WORDS + overflow_words) * std::mem::size_of::<u32>()) as u64;
            let accum_and_overflow_buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_td_accum_and_overflow"),
                size: accum_ov_bytes,
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_DST
                    | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            });
            let accum_readback_buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_td_accum_readback"),
                size: std::mem::size_of::<GpuViolationAccum>() as u64,
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });
            let overflow_readback_buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_td_overflow_readback"),
                size: (overflow_words * std::mem::size_of::<u32>()) as u64,
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });

            let bind_group = build_td_bind_group(
                device,
                &ctx.tridexel_bgl,
                &uniform_buf,
                &m_slots_buf,
                &m_counts_buf,
                &t_slots_buf,
                &t_counts_buf,
                profiles_buf.buffer(),
                chords_buf.buffer(),
                tile_data_buf.buffer(),
                &accum_and_overflow_buf,
            );

            GpuPlaneDispatch {
                axis,
                nu,
                nv,
                tiles_u,
                tiles_v,
                ray_offset,
                origin_u_mm: p.origin_u_mm,
                origin_v_mm: p.origin_v_mm,
                origin_w_mm: p.origin_w_mm,
                du_mm: p.du_mm,
                dv_mm: p.dv_mm,
                overflow_words,
                uniform_buf,
                tile_data_buf,
                accum_and_overflow_buf,
                accum_readback_buf,
                overflow_readback_buf,
                bind_group,
                tile_bins: vec![Vec::new(); num_tiles],
                tile_data_scratch: Vec::with_capacity(num_tiles + 1 + 256),
            }
        };

        let planes = [
            make_plane(Axis3::X),
            make_plane(Axis3::Y),
            make_plane(Axis3::Z),
        ];

        Self {
            ctx,
            dims,
            origin_wcs,
            pitch_mm,
            rotary,
            plane_ray_offsets,
            total_rays,
            has_target,
            m_slots_buf,
            m_counts_buf,
            t_slots_buf,
            t_counts_buf,
            m_slots_readback_buf,
            m_counts_readback_buf,
            render_snap_pool: Vec::new(),
            profiles_buf,
            chords_buf,
            planes,
            gpu_chords_scratch: Vec::with_capacity(128),
            prim_chords_scratch: Vec::with_capacity(128),
            arc_linear_scratch: Vec::with_capacity(32),
            prim_decompose_scratch: Vec::with_capacity(32),
            gpu_profiles_scratch: Vec::with_capacity(16),
            cpu: Mutex::new(CpuShadow {
                stock,
                target,
                gpu_dirty: false,
            }),
        }
    }

    #[inline]
    pub(crate) fn dims(&self) -> [usize; 3] {
        self.dims
    }

    #[inline]
    pub(crate) fn origin_wcs(&self) -> DVec3 {
        self.origin_wcs
    }

    #[inline]
    pub(crate) fn pitch_mm(&self) -> DVec3 {
        self.pitch_mm
    }

    #[inline]
    pub(crate) fn plane_ray_offsets(&self) -> [usize; 3] {
        self.plane_ray_offsets
    }

    #[inline]
    pub(crate) fn total_rays(&self) -> usize {
        self.total_rays
    }

    #[inline]
    pub(crate) fn has_target(&self) -> bool {
        self.has_target
    }

    #[inline]
    pub(crate) fn m_slots_buf(&self) -> &wgpu::Buffer {
        &self.m_slots_buf
    }

    #[inline]
    pub(crate) fn m_counts_buf(&self) -> &wgpu::Buffer {
        &self.m_counts_buf
    }

    #[inline]
    pub(crate) fn t_slots_buf(&self) -> &wgpu::Buffer {
        &self.t_slots_buf
    }

    #[inline]
    pub(crate) fn t_counts_buf(&self) -> &wgpu::Buffer {
        &self.t_counts_buf
    }

    /// Returns total volume (`mm³`) of all rays currently residing in the CPU overflow side-table
    /// across all 3 planes (averaged by `/ 3.0`), without touching GPU buffers.
    pub(crate) fn overflow_volume_mm3(&self) -> f64 {
        let guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
        let mut sum = 0.0;
        for axis in Axis3::ALL {
            let plane = guard.stock.plane(axis);
            if !plane.overflow.is_empty() {
                let area = plane.du_mm * plane.dv_mm;
                let len_sum: f64 = plane
                    .overflow
                    .values()
                    .map(|spans| spans.iter().map(|s| s.len_mm()).sum::<f64>())
                    .sum();
                sum += len_sum * area;
            }
        }
        sum / 3.0
    }

    /// Returns `true` if any ray in either machined or target Tri-Dexel stock resides in CPU overflow.
    pub(crate) fn any_overflow(&self) -> bool {
        let guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
        let m_ov = guard.stock.planes.iter().any(|p| !p.overflow.is_empty());
        let t_ov = guard
            .target
            .as_ref()
            .is_some_and(|t| t.planes.iter().any(|p| !p.overflow.is_empty()));
        m_ov || t_ov
    }

    /// Rasterizes `target_mesh` directly onto GPU target buffers (`t_slots_buf`, `t_counts_buf`)
    /// via `bvh_distance.wgsl`, updating the CPU shadow target copy and rebinding sweep planes.
    pub(crate) fn rasterize_target_mesh_gpu(&mut self, target_mesh: &TriMesh3D) {
        let device = self.ctx.device();
        let queue = self.ctx.queue();
        let total_slots = self.total_rays * INLINE_DEXEL_SLOTS;

        if !self.has_target {
            let slots_bytes = (total_slots * std::mem::size_of::<FlatDexelSpan>()) as u64;
            let counts_bytes = (self.total_rays * std::mem::size_of::<u32>()) as u64;
            self.t_slots_buf = Arc::new(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_td_t_slots"),
                size: slots_bytes.max(16),
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_DST
                    | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            }));
            self.t_counts_buf = Arc::new(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_td_t_counts"),
                size: counts_bytes.max(16),
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_DST
                    | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            }));
            self.has_target = true;

            let dz_step = (0.25 * self.pitch_mm.z) as f32;
            for plane in &mut self.planes {
                let uniforms = TriDexelPlaneUniforms {
                    nu: plane.nu as u32,
                    nv: plane.nv as u32,
                    tiles_u: plane.tiles_u as u32,
                    num_tiles: (plane.tiles_u * plane.tiles_v) as u32,
                    axis_id: plane.axis.index() as u32,
                    has_target: 1,
                    plane_ray_offset: plane.ray_offset as u32,
                    _pad0: 0,
                    du_mm: plane.du_mm as f32,
                    dv_mm: plane.dv_mm as f32,
                    dz_step_mm: dz_step,
                    _pad1: 0.0,
                };
                queue.write_buffer(&plane.uniform_buf, 0, bytemuck::bytes_of(&uniforms));
                plane.bind_group = build_td_bind_group(
                    device,
                    &self.ctx.tridexel_bgl,
                    &plane.uniform_buf,
                    &self.m_slots_buf,
                    &self.m_counts_buf,
                    &self.t_slots_buf,
                    &self.t_counts_buf,
                    self.profiles_buf.buffer(),
                    self.chords_buf.buffer(),
                    plane.tile_data_buf.buffer(),
                    &plane.accum_and_overflow_buf,
                );
            }
        }

        let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
        let mut target_stock = TriDexelStock::empty_like(&guard.stock);
        self.ctx.rasterize_tridexel_into_buffers(
            &mut target_stock,
            self.plane_ray_offsets,
            target_mesh,
            &self.t_slots_buf,
            &self.t_counts_buf,
            &self.m_slots_readback_buf,
            &self.m_counts_readback_buf,
        );
        guard.target = Some(target_stock);
    }

    /// Snapshots `m_slots` & `m_counts` into a pooled immutable GPU buffer set, dispatches
    /// `tridexel_emit.wgsl` into a reusable compact GPU quad buffer + `DrawIndirectArgs` buffer,
    /// and returns a read-only [`GpuStockRenderHandle::TriDexel`] (zero CPU-to-GPU upload for
    /// inline rays; optional tiny CPU supplement mesh ONLY when rare overflow rays exist).
    pub(crate) fn capture_render_handle(
        &mut self,
        color_mode: SimStockColorMode,
    ) -> Arc<GpuStockRenderHandle> {
        self.capture_render_handle_with_options(SimMeshOptions::from(color_mode))
    }

    /// Snapshots `m_slots` & `m_counts` into a pooled immutable GPU buffer set and dispatches
    /// `tridexel_emit.wgsl` with visual [`SimMeshOptions`] (`color_mode` + `mesh_skinning`).
    pub(crate) fn capture_render_handle_with_options(
        &mut self,
        options: SimMeshOptions,
    ) -> Arc<GpuStockRenderHandle> {
        let color_mode = options.color_mode;
        let total_slots = self.total_rays * INLINE_DEXEL_SLOTS;
        let slots_bytes = (total_slots * std::mem::size_of::<FlatDexelSpan>()) as u64;
        let counts_bytes = (self.total_rays * std::mem::size_of::<u32>()) as u64;
        let max_quads = compute_tridexel_max_quads(self.ctx.device(), self.total_rays);
        let quad_bytes = (max_quads as u64) * (TRIDEXEL_PACKED_QUAD_BYTES as u64);

        let slot = if let Some(existing) = self.render_snap_pool.iter().find(|s| {
            Arc::strong_count(&s.m_slots) == 1
                && Arc::strong_count(&s.m_counts) == 1
                && Arc::strong_count(&s.quad_buf) == 1
                && Arc::strong_count(&s.indirect_args_buf) == 1
                && Arc::strong_count(&s.emit_uniform_buf) == 1
        }) {
            (
                Arc::clone(&existing.m_slots),
                Arc::clone(&existing.m_counts),
                Arc::clone(&existing.quad_buf),
                Arc::clone(&existing.indirect_args_buf),
                Arc::clone(&existing.emit_uniform_buf),
            )
        } else {
            let device = self.ctx.device();
            let m_slots = Arc::new(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_td_render_snap_slots"),
                size: slots_bytes.max(16),
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }));
            let m_counts = Arc::new(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_td_render_snap_counts"),
                size: counts_bytes.max(16),
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }));
            let quad_buf = Arc::new(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_td_emit_quads"),
                size: quad_bytes.max(64),
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            }));
            let indirect_args_buf = Arc::new(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_td_emit_indirect_args"),
                size: 16,
                usage: wgpu::BufferUsages::INDIRECT
                    | wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_DST
                    | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            }));
            let emit_uniform_buf = Arc::new(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_td_emit_uniforms"),
                size: std::mem::size_of::<TriDexelEmitUniforms>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }));
            self.render_snap_pool.push(TriDexelRenderSnapSlot {
                m_slots: Arc::clone(&m_slots),
                m_counts: Arc::clone(&m_counts),
                quad_buf: Arc::clone(&quad_buf),
                indirect_args_buf: Arc::clone(&indirect_args_buf),
                emit_uniform_buf: Arc::clone(&emit_uniform_buf),
            });
            (
                m_slots,
                m_counts,
                quad_buf,
                indirect_args_buf,
                emit_uniform_buf,
            )
        };

        if slots_bytes > 0 && counts_bytes > 0 {
            let mut encoder =
                self.ctx
                    .device()
                    .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                        label: Some("chip_td_render_snap_copy"),
                    });
            encoder.copy_buffer_to_buffer(&self.m_slots_buf, 0, &slot.0, 0, slots_bytes);
            encoder.copy_buffer_to_buffer(&self.m_counts_buf, 0, &slot.1, 0, counts_bytes);
            self.ctx.queue().submit(std::iter::once(encoder.finish()));
        }

        let overflow_mesh = self.build_overflow_supplement_mesh(color_mode);

        let data = GpuTriDexelRenderData::new_and_emit(
            Arc::clone(&self.ctx),
            [
                self.dims[0] as u32,
                self.dims[1] as u32,
                self.dims[2] as u32,
            ],
            [
                self.origin_wcs.x as f32,
                self.origin_wcs.y as f32,
                self.origin_wcs.z as f32,
            ],
            [
                self.pitch_mm.x as f32,
                self.pitch_mm.y as f32,
                self.pitch_mm.z as f32,
            ],
            [
                self.plane_ray_offsets[0] as u32,
                self.plane_ray_offsets[1] as u32,
                self.plane_ray_offsets[2] as u32,
            ],
            self.total_rays as u32,
            self.has_target,
            slot.0,
            slot.1,
            Arc::clone(&self.t_slots_buf),
            Arc::clone(&self.t_counts_buf),
            slot.2,
            slot.3,
            slot.4,
            max_quads,
            overflow_mesh,
            options,
        );

        Arc::new(GpuStockRenderHandle::TriDexel(data))
    }

    fn build_overflow_supplement_mesh(
        &self,
        color_mode: SimStockColorMode,
    ) -> Option<Arc<SimStockMesh>> {
        let guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
        build_tridexel_overflow_supplement_mesh(&guard.stock, guard.target.as_ref(), color_mode)
    }

    pub(crate) fn checkpoint_byte_size(&self) -> usize {
        self.total_rays * INLINE_DEXEL_SLOTS * std::mem::size_of::<FlatDexelSpan>()
            + self.total_rays * std::mem::size_of::<u32>()
    }

    pub(crate) fn clone_live_to_checkpoint_buffers(&self) -> GpuCheckpointBuffers {
        let total_slots = self.total_rays * INLINE_DEXEL_SLOTS;
        let slots_bytes = (total_slots * std::mem::size_of::<FlatDexelSpan>()) as u64;
        let counts_bytes = (self.total_rays * std::mem::size_of::<u32>()) as u64;
        let device = self.ctx.device();
        let m_slots = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_gpu_cp_td_slots"),
            size: slots_bytes.max(16),
            usage: wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let m_counts = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_gpu_cp_td_counts"),
            size: counts_bytes.max(16),
            usage: wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        if slots_bytes > 0 && counts_bytes > 0 {
            let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("chip_td_cp_capture"),
            });
            encoder.copy_buffer_to_buffer(&self.m_slots_buf, 0, &m_slots, 0, slots_bytes);
            encoder.copy_buffer_to_buffer(&self.m_counts_buf, 0, &m_counts, 0, counts_bytes);
            self.ctx.queue().submit(std::iter::once(encoder.finish()));
        }
        let guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
        let overflow = [
            guard.stock.planes[0].overflow.clone(),
            guard.stock.planes[1].overflow.clone(),
            guard.stock.planes[2].overflow.clone(),
        ];
        GpuCheckpointBuffers::TriDexel {
            m_slots,
            m_counts,
            slots_bytes,
            counts_bytes,
            overflow,
        }
    }

    pub(crate) fn restore_from_gpu_checkpoint(
        &mut self,
        m_slots: &wgpu::Buffer,
        m_counts: &wgpu::Buffer,
        slots_bytes: u64,
        counts_bytes: u64,
        overflow: &[BTreeMap<u32, Vec<DexelSpan>>; 3],
    ) {
        if slots_bytes > 0 && counts_bytes > 0 {
            let mut encoder =
                self.ctx
                    .device()
                    .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                        label: Some("chip_td_cp_restore"),
                    });
            encoder.copy_buffer_to_buffer(m_slots, 0, &self.m_slots_buf, 0, slots_bytes);
            encoder.copy_buffer_to_buffer(m_counts, 0, &self.m_counts_buf, 0, counts_bytes);
            self.ctx.queue().submit(std::iter::once(encoder.finish()));
        }
        let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
        for (ax, ov) in overflow.iter().enumerate() {
            guard.stock.planes[ax].overflow.clone_from(ov);
        }
        guard.gpu_dirty = true;
    }

    pub(crate) fn copy_state_from_cpu_stock(&mut self, stock: &TriDexelStock) {
        let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
        guard.stock.clone_from(stock);
        guard.gpu_dirty = false;
        let queue = self.ctx.queue();
        for axis in Axis3::ALL {
            let p = guard.stock.plane(axis);
            let ray_off = self.plane_ray_offsets[axis.index()];
            let slot_byte_off =
                (ray_off * INLINE_DEXEL_SLOTS * std::mem::size_of::<FlatDexelSpan>()) as u64;
            let cnt_byte_off = (ray_off * std::mem::size_of::<u32>()) as u64;
            queue.write_buffer(
                &self.m_slots_buf,
                slot_byte_off,
                bytemuck::cast_slice(&p.slots),
            );
            queue.write_buffer(
                &self.m_counts_buf,
                cnt_byte_off,
                bytemuck::cast_slice(&p.counts),
            );
        }
    }

    /// Dispatches all 3 orthogonal dexel planes in one `CommandEncoder` submission and resolves
    /// any rare `> K = 4`-span overflow rays losslessly on CPU.
    pub(crate) fn sweep_batch(
        &mut self,
        chords: &[SweepChord],
        profiles: &[SweepToolSpec],
        verify: bool,
    ) -> BatchSweepOutput {
        if chords.is_empty() || profiles.is_empty() {
            return BatchSweepOutput::default();
        }

        self.gpu_chords_scratch.clear();
        self.prim_chords_scratch.clear();
        let min_pitch = self.pitch_mm.x.min(self.pitch_mm.y).min(self.pitch_mm.z);
        let stock_aabb = Aabb3::new(
            self.origin_wcs,
            self.origin_wcs
                + DVec3::new(
                    self.dims[0] as f64 * self.pitch_mm.x,
                    self.dims[1] as f64 * self.pitch_mm.y,
                    self.dims[2] as f64 * self.pitch_mm.z,
                ),
        );
        let mut chuck_violations = SpanRuleViolations::default();
        let mut prev_oriented_end: Option<(DVec3, f64, u32, usize, bool)> = None;

        for (raw_idx, ch) in chords.iter().enumerate() {
            let Some(tool) = profiles.get(ch.profile_idx) else {
                continue;
            };
            let start_pt = ch.motion.sample_point(0.0);
            let start_rot = ch.motion.sample_rotary_deg(0.0);
            let skip_start = prev_oriented_end.is_some_and(|(pp, pr, pop, pprof, prap)| {
                pop == ch.op_idx
                    && pprof == ch.profile_idx
                    && prap == ch.is_rapid
                    && pp.distance_squared(start_pt) <= 1e-10
                    && (pr - start_rot).abs() <= 1e-6
            });
            self.prim_decompose_scratch.clear();
            decompose_chord_for_tridexel(
                ch,
                raw_idx as u32,
                self.rotary,
                min_pitch,
                &tool.profile,
                verify,
                skip_start,
                &mut self.arc_linear_scratch,
                &mut self.prim_decompose_scratch,
                &mut chuck_violations,
            );
            prev_oriented_end = if self
                .prim_decompose_scratch
                .last()
                .is_some_and(|p| matches!(p, SweptPrimitive::OrientedPose(_)))
            {
                Some((
                    ch.motion.sample_point(1.0),
                    ch.motion.sample_rotary_deg(1.0),
                    ch.op_idx,
                    ch.profile_idx,
                    ch.is_rapid,
                ))
            } else {
                None
            };
            for &prim in &self.prim_decompose_scratch {
                let mut flags = 0u32;
                if ch.is_rapid {
                    flags |= CHORD_FLAG_RAPID;
                }
                if verify {
                    flags |= CHORD_FLAG_VERIFY;
                }
                let mut oriented_aabb = None;
                let gpu_idx = self.gpu_chords_scratch.len() as u32;
                let gpu_chord = match prim {
                    SweptPrimitive::CardinalLinear {
                        chord: lin,
                        spindle,
                        rot_deg,
                    } => {
                        flags |= (spindle.code() & CHORD_CARDINAL_MASK) << CHORD_CARDINAL_SHIFT;
                        GpuSweepChord {
                            p0_rot: [
                                (lin.p0.x - self.origin_wcs.x) as f32,
                                (lin.p0.y - self.origin_wcs.y) as f32,
                                (lin.p0.z - self.origin_wcs.z) as f32,
                                rot_deg as f32,
                            ],
                            p1_rot: [
                                (lin.p1.x - self.origin_wcs.x) as f32,
                                (lin.p1.y - self.origin_wcs.y) as f32,
                                (lin.p1.z - self.origin_wcs.z) as f32,
                                rot_deg as f32,
                            ],
                            arc_params: [0.0; 4],
                            sweep_and_tols: [
                                0.0,
                                ch.verify_tol.rapid_crash_tol_mm as f32,
                                ch.verify_tol.gouge_tol_mm as f32,
                                ch.verify_tol.radial_wall_skin_mm as f32,
                            ],
                            meta: [ch.op_idx, ch.profile_idx as u32, flags, gpu_idx],
                        }
                    }
                    SweptPrimitive::OrientedPose(pose) => {
                        let Some((aabb_min, aabb_max, check_holder)) =
                            oriented_pose_stock_aabb(pose, &tool.profile, stock_aabb, verify)
                        else {
                            continue;
                        };
                        oriented_aabb = Some((aabb_min, aabb_max));
                        flags |= CHORD_FLAG_ORIENTED_POSE;
                        if check_holder {
                            flags |= CHORD_FLAG_CHECK_HOLDER;
                        }
                        GpuSweepChord {
                            p0_rot: [
                                (pose.tip.x - self.origin_wcs.x) as f32,
                                (pose.tip.y - self.origin_wcs.y) as f32,
                                (pose.tip.z - self.origin_wcs.z) as f32,
                                0.0,
                            ],
                            p1_rot: [0.0; 4],
                            arc_params: [
                                pose.spindle.x as f32,
                                pose.spindle.y as f32,
                                pose.spindle.z as f32,
                                0.0,
                            ],
                            sweep_and_tols: [
                                0.0,
                                ch.verify_tol.rapid_crash_tol_mm as f32,
                                ch.verify_tol.gouge_tol_mm as f32,
                                ch.verify_tol.radial_wall_skin_mm as f32,
                            ],
                            meta: [ch.op_idx, ch.profile_idx as u32, flags, gpu_idx],
                        }
                    }
                };
                self.gpu_chords_scratch.push(gpu_chord);
                self.prim_chords_scratch.push(FlattenedPrimitive {
                    prim,
                    src_chord_idx: raw_idx as u32,
                    op_idx: ch.op_idx,
                    profile_idx: ch.profile_idx,
                    is_rapid: ch.is_rapid,
                    verify_tol: ch.verify_tol,
                    oriented_aabb,
                });
            }
        }

        if self.gpu_chords_scratch.is_empty() {
            return BatchSweepOutput {
                mutated: false,
                violations: chuck_violations,
            };
        }

        self.gpu_profiles_scratch.clear();
        self.gpu_profiles_scratch
            .extend(profiles.iter().map(GpuToolProfile::from_tool_spec));

        let shared_rebind = {
            let device = self.ctx.device();
            let queue = self.ctx.queue();
            let mut rebind = false;
            rebind |= self
                .profiles_buf
                .upload_slice(device, queue, &self.gpu_profiles_scratch);
            rebind |= self
                .chords_buf
                .upload_slice(device, queue, &self.gpu_chords_scratch);
            rebind
        };

        // Partition flattened primitives into maximal contiguous runs of `CardinalLinear` vs
        // `OrientedPose` so each run dispatches its dedicated compact shader (`main` vs lazy
        // `main_oriented`) in exact chronological order with zero extra passes for pure batches.
        let total_prims = self.prim_chords_scratch.len();
        let mut any_mutated = false;
        let mut merged_violations = chuck_violations;
        let mut run_start = 0usize;
        let mut need_shared_rebind = shared_rebind;
        while run_start < total_prims {
            let is_oriented = matches!(
                self.prim_chords_scratch[run_start].prim,
                SweptPrimitive::OrientedPose(_)
            );
            let mut run_end = run_start + 1;
            while run_end < total_prims
                && matches!(
                    self.prim_chords_scratch[run_end].prim,
                    SweptPrimitive::OrientedPose(_)
                ) == is_oriented
            {
                run_end += 1;
            }
            let (run_mut, run_viol) = self.dispatch_primitive_run(
                run_start..run_end,
                is_oriented,
                need_shared_rebind,
                profiles,
                verify,
            );
            need_shared_rebind = false;
            any_mutated |= run_mut;
            merged_violations.merge(run_viol);
            run_start = run_end;
        }

        if any_mutated {
            let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
            guard.gpu_dirty = true;
        }

        BatchSweepOutput {
            mutated: any_mutated,
            violations: merged_violations,
        }
    }

    fn dispatch_primitive_run(
        &mut self,
        run_range: std::ops::Range<usize>,
        is_oriented: bool,
        shared_rebind: bool,
        profiles: &[SweepToolSpec],
        verify: bool,
    ) -> (bool, SpanRuleViolations) {
        // Coarse 16x16 tile-binning for all 3 planes in strict chronological chord order
        let mut active_planes = [false; 3];
        for (ax_idx, plane) in self.planes.iter_mut().enumerate() {
            for bin in &mut plane.tile_bins {
                bin.clear();
            }
            let axis = plane.axis;
            let u_ax = axis.u_axis();
            let v_ax = axis.v_axis();
            let mut total_binned = 0usize;

            for gpu_idx in run_range.clone() {
                let fc = &self.prim_chords_scratch[gpu_idx];
                let Some(tool) = profiles.get(fc.profile_idx) else {
                    continue;
                };
                let prof = &tool.profile;
                let r_cut = prof.cutting_radius_mm;
                let (min_u, max_u, min_v, max_v) = match fc.prim {
                    SweptPrimitive::CardinalLinear { chord, spindle, .. } => {
                        let s_ax = spindle.axis();
                        if axis == s_ax {
                            let r_chk = if verify {
                                r_cut.max(prof.holder_radius_mm)
                            } else {
                                r_cut
                            };
                            (
                                u_ax.get(chord.p0).min(u_ax.get(chord.p1)) - r_chk,
                                u_ax.get(chord.p0).max(u_ax.get(chord.p1)) + r_chk,
                                v_ax.get(chord.p0).min(v_ax.get(chord.p1)) - r_chk,
                                v_ax.get(chord.p0).max(v_ax.get(chord.p1)) + r_chk,
                            )
                        } else {
                            let max_h = prof.horizontal_cut_max_height_mm();
                            let spindle_in_u = u_ax == s_ax;
                            let other_ax = if spindle_in_u { v_ax } else { u_ax };
                            let min_lat =
                                other_ax.get(chord.p0).min(other_ax.get(chord.p1)) - r_cut;
                            let max_lat =
                                other_ax.get(chord.p0).max(other_ax.get(chord.p1)) + r_cut;
                            let (min_s, max_s) = if spindle.sign() >= 0.0 {
                                (
                                    s_ax.get(chord.p0).min(s_ax.get(chord.p1)),
                                    s_ax.get(chord.p0).max(s_ax.get(chord.p1)) + max_h,
                                )
                            } else {
                                (
                                    s_ax.get(chord.p0).min(s_ax.get(chord.p1)) - max_h,
                                    s_ax.get(chord.p0).max(s_ax.get(chord.p1)),
                                )
                            };
                            if spindle_in_u {
                                (min_s, max_s, min_lat, max_lat)
                            } else {
                                (min_lat, max_lat, min_s, max_s)
                            }
                        }
                    }
                    SweptPrimitive::OrientedPose(pose) => {
                        let (aabb_min, aabb_max) = fc.oriented_aabb.unwrap_or_else(|| {
                            let reach = prof.horizontal_cut_max_height_mm();
                            let top_pt = pose.tip + pose.spindle * reach;
                            (
                                pose.tip.min(top_pt) - DVec3::splat(r_cut),
                                pose.tip.max(top_pt) + DVec3::splat(r_cut),
                            )
                        });
                        (
                            u_ax.get(aabb_min),
                            u_ax.get(aabb_max),
                            v_ax.get(aabb_min),
                            v_ax.get(aabb_max),
                        )
                    }
                };

                let (Some((u0, u1)), Some((v0, v1))) = (
                    plane_index_span(plane.origin_u_mm, plane.du_mm, plane.nu, min_u, max_u),
                    plane_index_span(plane.origin_v_mm, plane.dv_mm, plane.nv, min_v, max_v),
                ) else {
                    continue;
                };

                let tu0 = u0 / GPU_TILE_SIZE;
                let tu1 = u1 / GPU_TILE_SIZE;
                let tv0 = v0 / GPU_TILE_SIZE;
                let tv1 = v1 / GPU_TILE_SIZE;
                for tv in tv0..=tv1 {
                    let row_tile = tv * plane.tiles_u;
                    for tu in tu0..=tu1 {
                        plane.tile_bins[row_tile + tu].push(gpu_idx as u32);
                        total_binned += 1;
                    }
                }
            }

            if total_binned > 0 {
                active_planes[ax_idx] = true;
                let num_tiles = plane.tiles_u * plane.tiles_v;
                plane.tile_data_scratch.clear();
                plane
                    .tile_data_scratch
                    .reserve(num_tiles + 1 + total_binned);
                let mut cursor = 0u32;
                for bin in &plane.tile_bins {
                    plane.tile_data_scratch.push(cursor);
                    cursor += bin.len() as u32;
                }
                plane.tile_data_scratch.push(cursor);
                for bin in &plane.tile_bins {
                    plane.tile_data_scratch.extend_from_slice(bin);
                }
            }
        }

        if !active_planes.iter().any(|&a| a) {
            return (false, SpanRuleViolations::default());
        }

        let device = self.ctx.device();
        let queue = self.ctx.queue();
        for (ax_idx, plane) in self.planes.iter_mut().enumerate() {
            let mut plane_rebind = shared_rebind;
            if active_planes[ax_idx] {
                plane_rebind |=
                    plane
                        .tile_data_buf
                        .upload_slice(device, queue, &plane.tile_data_scratch);
            }
            if plane_rebind {
                plane.bind_group = build_td_bind_group(
                    device,
                    &self.ctx.tridexel_bgl,
                    &plane.uniform_buf,
                    &self.m_slots_buf,
                    &self.m_counts_buf,
                    &self.t_slots_buf,
                    &self.t_counts_buf,
                    self.profiles_buf.buffer(),
                    self.chords_buf.buffer(),
                    plane.tile_data_buf.buffer(),
                    &plane.accum_and_overflow_buf,
                );
            }
        }

        let pipeline = if is_oriented {
            self.ctx.tridexel_oriented_pipeline()
        } else {
            &self.ctx.tridexel_pipeline
        };
        let accum_bytes = std::mem::size_of::<GpuViolationAccum>();
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("chip_td_sweep_encoder"),
        });

        for (ax_idx, plane) in self.planes.iter().enumerate() {
            if !active_planes[ax_idx] {
                continue;
            }
            encoder.clear_buffer(&plane.accum_and_overflow_buf, 0, None);
            {
                let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                    label: Some("chip_td_sweep_pass"),
                    timestamp_writes: None,
                });
                pass.set_pipeline(pipeline);
                pass.set_bind_group(0, &plane.bind_group, &[]);
                pass.dispatch_workgroups(plane.tiles_u as u32, plane.tiles_v as u32, 1);
            }
            encoder.copy_buffer_to_buffer(
                &plane.accum_and_overflow_buf,
                0,
                &plane.accum_readback_buf,
                0,
                accum_bytes as u64,
            );
        }

        queue.submit(std::iter::once(encoder.finish()));

        let mut any_mutated = false;
        let mut merged_violations = SpanRuleViolations::default();
        let mut overflow_planes = [false; 3];

        // Single-poll batched readback across all active planes (1 CPU-GPU sync instead of 3!)
        let mut raw0 = [0u8; std::mem::size_of::<GpuViolationAccum>()];
        let mut raw1 = [0u8; std::mem::size_of::<GpuViolationAccum>()];
        let mut raw2 = [0u8; std::mem::size_of::<GpuViolationAccum>()];
        let read_ok = {
            let mut entries: [(&wgpu::Buffer, usize, &mut [u8]); 3] = [
                (
                    &self.planes[0].accum_readback_buf,
                    if active_planes[0] { accum_bytes } else { 0 },
                    &mut raw0,
                ),
                (
                    &self.planes[1].accum_readback_buf,
                    if active_planes[1] { accum_bytes } else { 0 },
                    &mut raw1,
                ),
                (
                    &self.planes[2].accum_readback_buf,
                    if active_planes[2] { accum_bytes } else { 0 },
                    &mut raw2,
                ),
            ];
            self.ctx.readback_bytes_batch(&mut entries)
        };
        let raw_slices: [&[u8]; 3] = [&raw0, &raw1, &raw2];

        for (ax_idx, plane) in self.planes.iter().enumerate() {
            if !active_planes[ax_idx] {
                continue;
            }
            let accum = if read_ok[ax_idx] {
                *bytemuck::from_bytes::<GpuViolationAccum>(raw_slices[ax_idx])
            } else {
                GpuViolationAccum::default()
            };
            if accum.overflow_flag != 0 {
                overflow_planes[ax_idx] = true;
            }
            let axis = plane.axis;
            let nu = plane.nu;
            let ou = plane.origin_u_mm;
            let ov = plane.origin_v_mm;
            let ow = plane.origin_w_mm;
            let du = plane.du_mm;
            let dv = plane.dv_mm;
            let rotary = self.rotary;
            let prims = &self.prim_chords_scratch;
            let (plane_mut, plane_viol) =
                accum.into_span_violations_mapped(|peak_ch, first_ch, ray_idx, loc_w_local| {
                    let onset_src = prims
                        .get(first_ch as usize)
                        .map_or(0, |fc| fc.src_chord_idx);
                    if let Some(fc) = prims.get(peak_ch as usize) {
                        match fc.prim {
                            SweptPrimitive::OrientedPose(pose) => return (pose.tip_wcs, onset_src),
                            SweptPrimitive::CardinalLinear { rot_deg, .. } => {
                                let idx = ray_idx as usize;
                                let iu = idx % nu;
                                let iv = idx / nu;
                                let pu = ou + (iu as f64 + 0.5) * du;
                                let pv = ov + (iv as f64 + 0.5) * dv;
                                let pw = ow + f64::from(loc_w_local);
                                let pt_part = axis.from_uvw(pu, pv, pw);
                                if let Some(rot) = rotary
                                    && rot_deg.abs() > 1e-6
                                {
                                    return (rot.rotate_point_wcs(pt_part, rot_deg), onset_src);
                                }
                                return (pt_part, onset_src);
                            }
                        }
                    }
                    let idx = ray_idx as usize;
                    let iu = idx % nu;
                    let iv = idx / nu;
                    let pu = ou + (iu as f64 + 0.5) * du;
                    let pv = ov + (iv as f64 + 0.5) * dv;
                    let pw = ow + f64::from(loc_w_local);
                    (axis.from_uvw(pu, pv, pw), onset_src)
                });
            any_mutated |= plane_mut;
            merged_violations.merge(plane_viol);
        }

        // Lossless hybrid CPU overflow safety valve: only runs on the rare batch where a ray
        // exceeds K = 4 spans (or an already-overflowed ray is touched by a chord in this batch).
        for (ax_idx, &had_overflow) in overflow_planes.iter().enumerate() {
            if !had_overflow {
                continue;
            }
            let (ov_mut, ov_viol) =
                self.resolve_plane_overflow(ax_idx, run_range.clone(), profiles, verify);
            any_mutated |= ov_mut;
            merged_violations.merge(ov_viol);
        }

        (any_mutated, merged_violations)
    }

    /// Reads back ONLY the flagged overflow ray bitset for plane `ax_idx`, fetches pre-batch
    /// inline spans ONLY for newly-overflowed rays, replays this run's primitives in exact
    /// `f64`, updates `plane.overflow`, and migrates rays back into GPU inline slots if they
    /// shrink to `<= K = 4` spans!
    fn resolve_plane_overflow(
        &self,
        ax_idx: usize,
        run_range: std::ops::Range<usize>,
        profiles: &[SweepToolSpec],
        verify: bool,
    ) -> (bool, SpanRuleViolations) {
        let plane = &self.planes[ax_idx];
        let device = self.ctx.device();
        let queue = self.ctx.queue();
        let bitset_bytes = plane.overflow_words * std::mem::size_of::<u32>();
        let bitset_offset = (ACCUM_WORDS * std::mem::size_of::<u32>()) as u64;

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("chip_td_overflow_bitset_readback"),
        });
        encoder.copy_buffer_to_buffer(
            &plane.accum_and_overflow_buf,
            bitset_offset,
            &plane.overflow_readback_buf,
            0,
            bitset_bytes as u64,
        );
        queue.submit(std::iter::once(encoder.finish()));

        let mut bitset = vec![0u32; plane.overflow_words];
        if !self.ctx.readback_bytes(
            &plane.overflow_readback_buf,
            bitset_bytes,
            bytemuck::cast_slice_mut(&mut bitset),
        ) {
            return (false, SpanRuleViolations::default());
        }

        let ray_count = plane.nu * plane.nv;
        let mut flagged_rays: Vec<u32> = Vec::new();
        for (word_idx, &bits) in bitset.iter().enumerate() {
            let mut w = bits;
            while w != 0 {
                let tz = w.trailing_zeros();
                let ray_idx = (word_idx as u32) * 32 + tz;
                if (ray_idx as usize) < ray_count {
                    flagged_rays.push(ray_idx);
                }
                w &= w - 1;
            }
        }
        if flagged_rays.is_empty() {
            return (false, SpanRuleViolations::default());
        }

        let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
        let mut need_gpu_fetch: Vec<u32> = Vec::new();
        for &r in &flagged_rays {
            if !guard.stock.planes[ax_idx].overflow.contains_key(&r) {
                need_gpu_fetch.push(r);
            }
        }

        // Batch-copy pre-batch inline `(slots[r*4..r*4+4], counts[r])` for newly-overflowed rays
        let mut fetched_inline: std::collections::HashMap<
            u32,
            ([FlatDexelSpan; INLINE_DEXEL_SLOTS], u32),
        > = std::collections::HashMap::with_capacity(need_gpu_fetch.len());
        if !need_gpu_fetch.is_empty() {
            let slot_chunk = INLINE_DEXEL_SLOTS * std::mem::size_of::<FlatDexelSpan>(); // 64 bytes
            let total_slot_bytes = (need_gpu_fetch.len() * slot_chunk) as u64;
            let total_cnt_bytes = (need_gpu_fetch.len() * std::mem::size_of::<u32>()) as u64;
            let slot_staging = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_td_ov_ray_slots_rb"),
                size: total_slot_bytes.max(16),
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });
            let cnt_staging = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_td_ov_ray_cnts_rb"),
                size: total_cnt_bytes.max(16),
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });
            let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("chip_td_ov_ray_fetch"),
            });
            for (k, &local_r) in need_gpu_fetch.iter().enumerate() {
                let global_r = plane.ray_offset + local_r as usize;
                enc.copy_buffer_to_buffer(
                    &self.m_slots_buf,
                    (global_r * slot_chunk) as u64,
                    &slot_staging,
                    (k * slot_chunk) as u64,
                    slot_chunk as u64,
                );
                enc.copy_buffer_to_buffer(
                    &self.m_counts_buf,
                    (global_r * std::mem::size_of::<u32>()) as u64,
                    &cnt_staging,
                    (k * std::mem::size_of::<u32>()) as u64,
                    std::mem::size_of::<u32>() as u64,
                );
            }
            queue.submit(std::iter::once(enc.finish()));

            let mut raw_slots =
                vec![FlatDexelSpan::ZERO; need_gpu_fetch.len() * INLINE_DEXEL_SLOTS];
            let mut raw_cnts = vec![0u32; need_gpu_fetch.len()];
            let ok_s = self.ctx.readback_bytes(
                &slot_staging,
                total_slot_bytes as usize,
                bytemuck::cast_slice_mut(&mut raw_slots),
            );
            let ok_c = self.ctx.readback_bytes(
                &cnt_staging,
                total_cnt_bytes as usize,
                bytemuck::cast_slice_mut(&mut raw_cnts),
            );
            if ok_s && ok_c {
                for (k, &local_r) in need_gpu_fetch.iter().enumerate() {
                    let base = k * INLINE_DEXEL_SLOTS;
                    let arr = [
                        raw_slots[base],
                        raw_slots[base + 1],
                        raw_slots[base + 2],
                        raw_slots[base + 3],
                    ];
                    fetched_inline.insert(local_r, (arr, raw_cnts[k]));
                }
            }
        }

        let pitch_mm = self.pitch_mm;
        let rotary = self.rotary;
        let axis = plane.axis;
        let nu = plane.nu;
        let ou = plane.origin_u_mm;
        let ov = plane.origin_v_mm;
        let ow = plane.origin_w_mm;
        let du = plane.du_mm;
        let dv = plane.dv_mm;
        let mut any_mut = false;
        let mut merged_viol = SpanRuleViolations::default();
        let mut scratch: Vec<DexelSpan> = Vec::with_capacity(16);

        for &local_r in &flagged_rays {
            scratch.clear();
            if let Some(existing) = guard.stock.planes[ax_idx].overflow.get(&local_r) {
                scratch.extend_from_slice(existing);
            } else if let Some(&(arr, cnt)) = fetched_inline.get(&local_r) {
                let n = (cnt as usize).min(INLINE_DEXEL_SLOTS);
                for slot in arr.iter().take(n) {
                    scratch.push(slot.to_wcs(ow));
                }
            } else {
                continue;
            }

            let iu = (local_r as usize) % nu;
            let iv = (local_r as usize) / nu;
            let pu = ou + (iu as f64 + 0.5) * du;
            let pv = ov + (iv as f64 + 0.5) * dv;
            let target_spans_owned: Option<Vec<DexelSpan>> = guard
                .target
                .as_ref()
                .map(|t| t.plane(axis).ray_spans_by_index(local_r as usize).to_vec());
            let target_slice = target_spans_owned.as_deref();

            let mut ray_mut = false;
            for fc in &self.prim_chords_scratch[run_range.clone()] {
                let Some(tool) = profiles.get(fc.profile_idx) else {
                    continue;
                };
                let (m, v) = replay_single_ray_primitive(
                    axis,
                    pu,
                    pv,
                    pitch_mm,
                    fc.prim,
                    rotary,
                    tool,
                    fc.is_rapid,
                    fc.verify_tol,
                    fc.op_idx,
                    verify,
                    target_slice,
                    &mut scratch,
                );
                ray_mut |= m;
                merged_viol.merge(v);
            }

            let global_r = plane.ray_offset + local_r as usize;
            let slot_byte_off =
                (global_r * INLINE_DEXEL_SLOTS * std::mem::size_of::<FlatDexelSpan>()) as u64;
            let cnt_byte_off = (global_r * std::mem::size_of::<u32>()) as u64;

            if scratch.len() <= INLINE_DEXEL_SLOTS {
                // Migrate cleanly back into GPU inline flat pool!
                guard.stock.planes[ax_idx].overflow.remove(&local_r);
                let mut packed = [FlatDexelSpan::ZERO; INLINE_DEXEL_SLOTS];
                let mut written = 0usize;
                for &s in &scratch {
                    let flat = FlatDexelSpan::from_wcs(s, ow);
                    if flat.max_local > flat.min_local {
                        packed[written] = flat;
                        written += 1;
                    }
                }
                let cnt_u32 = written as u32;
                queue.write_buffer(
                    &self.m_slots_buf,
                    slot_byte_off,
                    bytemuck::cast_slice(&packed),
                );
                queue.write_buffer(
                    &self.m_counts_buf,
                    cnt_byte_off,
                    bytemuck::bytes_of(&cnt_u32),
                );
            } else {
                // Keep exact full span vector in CPU overflow side-table; mark GPU `counts > K`.
                let mut packed = [FlatDexelSpan::ZERO; INLINE_DEXEL_SLOTS];
                for (k, slot) in packed.iter_mut().enumerate() {
                    *slot = FlatDexelSpan::from_wcs(scratch[k], ow);
                }
                let cnt_u32 = scratch.len() as u32;
                guard.stock.planes[ax_idx]
                    .overflow
                    .insert(local_r, scratch.clone());
                queue.write_buffer(
                    &self.m_slots_buf,
                    slot_byte_off,
                    bytemuck::cast_slice(&packed),
                );
                queue.write_buffer(
                    &self.m_counts_buf,
                    cnt_byte_off,
                    bytemuck::bytes_of(&cnt_u32),
                );
            }
            any_mut |= ray_mut;
        }

        (any_mut, merged_viol)
    }

    pub(crate) fn with_synced_stock<R>(&self, f: impl FnOnce(&TriDexelStock) -> R) -> R {
        let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
        if guard.gpu_dirty {
            self.sync_gpu_to_cpu_locked(&mut guard);
        }
        f(&guard.stock)
    }

    pub(crate) fn with_synced_stock_and_target<R>(
        &self,
        f: impl FnOnce(&TriDexelStock, Option<&TriDexelStock>) -> R,
    ) -> R {
        let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
        if guard.gpu_dirty {
            self.sync_gpu_to_cpu_locked(&mut guard);
        }
        f(&guard.stock, guard.target.as_ref())
    }

    pub(crate) fn reset_from_checkpoint(&mut self, cp: &StockCheckpointSnapshot) {
        if let StockCheckpointSnapshot::TriDexel { stock, .. } = cp {
            let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
            guard.stock.clone_from(stock);
            guard.gpu_dirty = false;
            let queue = self.ctx.queue();
            for axis in Axis3::ALL {
                let p = guard.stock.plane(axis);
                let ray_off = self.plane_ray_offsets[axis.index()];
                let slot_byte_off =
                    (ray_off * INLINE_DEXEL_SLOTS * std::mem::size_of::<FlatDexelSpan>()) as u64;
                let cnt_byte_off = (ray_off * std::mem::size_of::<u32>()) as u64;
                queue.write_buffer(
                    &self.m_slots_buf,
                    slot_byte_off,
                    bytemuck::cast_slice(&p.slots),
                );
                queue.write_buffer(
                    &self.m_counts_buf,
                    cnt_byte_off,
                    bytemuck::cast_slice(&p.counts),
                );
            }
        }
    }

    pub(crate) fn into_parts(self) -> (TriDexelStock, Option<TriDexelStock>) {
        {
            let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
            if guard.gpu_dirty {
                self.sync_gpu_to_cpu_locked(&mut guard);
            }
        }
        let shadow = self.cpu.into_inner().unwrap_or_else(|e| e.into_inner());
        (shadow.stock, shadow.target)
    }

    fn sync_gpu_to_cpu_locked(&self, guard: &mut CpuShadow) {
        let total_slots = self.total_rays * INLINE_DEXEL_SLOTS;
        let slots_bytes = total_slots * std::mem::size_of::<FlatDexelSpan>();
        let counts_bytes = self.total_rays * std::mem::size_of::<u32>();
        if slots_bytes == 0 || counts_bytes == 0 {
            guard.gpu_dirty = false;
            return;
        }
        let mut encoder =
            self.ctx
                .device()
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("chip_td_sync_readback_encoder"),
                });
        encoder.copy_buffer_to_buffer(
            &self.m_slots_buf,
            0,
            &self.m_slots_readback_buf,
            0,
            slots_bytes as u64,
        );
        encoder.copy_buffer_to_buffer(
            &self.m_counts_buf,
            0,
            &self.m_counts_readback_buf,
            0,
            counts_bytes as u64,
        );
        self.ctx.queue().submit(std::iter::once(encoder.finish()));

        let mut all_slots = vec![FlatDexelSpan::ZERO; total_slots];
        let mut all_counts = vec![0u32; self.total_rays];
        let ok_s = self.ctx.readback_bytes(
            &self.m_slots_readback_buf,
            slots_bytes,
            bytemuck::cast_slice_mut(&mut all_slots),
        );
        let ok_c = self.ctx.readback_bytes(
            &self.m_counts_readback_buf,
            counts_bytes,
            bytemuck::cast_slice_mut(&mut all_counts),
        );
        if ok_s && ok_c {
            for axis in Axis3::ALL {
                let p = guard.stock.plane_mut(axis);
                let ray_off = self.plane_ray_offsets[axis.index()];
                let rc = p.ray_count();
                let slot_off = ray_off * INLINE_DEXEL_SLOTS;
                p.slots
                    .copy_from_slice(&all_slots[slot_off..slot_off + rc * INLINE_DEXEL_SLOTS]);
                p.counts.copy_from_slice(&all_counts[ray_off..ray_off + rc]);
            }
        }
        guard.gpu_dirty = false;
    }
}

pub(super) fn build_tridexel_overflow_supplement_mesh(
    stock: &TriDexelStock,
    target: Option<&TriDexelStock>,
    color_mode: SimStockColorMode,
) -> Option<Arc<SimStockMesh>> {
    if !stock.planes.iter().any(|p| !p.overflow.is_empty()) {
        return None;
    }
    let color_by_op = matches!(color_mode, SimStockColorMode::ByOperation);
    let mut mesh = SimStockMesh::default();
    for axis in Axis3::ALL {
        let plane = stock.plane(axis);
        if plane.overflow.is_empty() {
            continue;
        }
        let target_plane = target.map(|t| t.plane(axis));
        let is_side = axis.is_side_axis();
        let pos_n = axis.unit_vec_f32();
        let neg_n = [-pos_n[0], -pos_n[1], -pos_n[2]];
        for (&local_r, spans) in &plane.overflow {
            let iu = (local_r as usize) % plane.nu;
            let iv = (local_r as usize) / plane.nu;
            let u0 = plane.origin_u_mm + (iu as f64) * plane.du_mm;
            let u1 = u0 + plane.du_mm;
            let v0 = plane.origin_v_mm + (iv as f64) * plane.dv_mm;
            let v1 = v0 + plane.dv_mm;
            let tgt_spans = target_plane.map(|tp| tp.ray_spans(iu, iv));
            for &s in spans {
                for (want_min, coord_w, op_idx, n) in [
                    (false, s.max_mm, s.max_op_idx, pos_n),
                    (true, s.min_mm, s.min_op_idx, neg_n),
                ] {
                    let col = match (color_mode, tgt_spans.as_ref()) {
                        (
                            SimStockColorMode::ErrorHeatmap {
                                tolerance_mm,
                                max_scale_mm,
                            },
                            Some(ts),
                        ) => {
                            let dev = signed_deviation_at_machined_endpoint(ts, coord_w, want_min);
                            deviation_to_heatmap_rgba(dev, tolerance_mm, max_scale_mm)
                        }
                        _ => op_color(op_idx, color_by_op, is_side),
                    };
                    let to_f32 = |u: f64, v: f64, w: f64| -> [f32; 3] {
                        let p = axis.from_uvw(u, v, w);
                        [p.x as f32, p.y as f32, p.z as f32]
                    };
                    let (a, b, c, d) = if axis == Axis3::Y {
                        (
                            to_f32(u0, v0, coord_w),
                            to_f32(u0, v1, coord_w),
                            to_f32(u1, v1, coord_w),
                            to_f32(u1, v0, coord_w),
                        )
                    } else {
                        (
                            to_f32(u0, v0, coord_w),
                            to_f32(u1, v0, coord_w),
                            to_f32(u1, v1, coord_w),
                            to_f32(u0, v1, coord_w),
                        )
                    };
                    let (p0, p1, p2, p3) = if want_min { (a, d, c, b) } else { (a, b, c, d) };
                    let base = mesh.vertices.len() as u32;
                    mesh.vertices.extend_from_slice(&[
                        SimColoredVertex {
                            pos_mm: p0,
                            normal: n,
                            color_rgba: col,
                        },
                        SimColoredVertex {
                            pos_mm: p1,
                            normal: n,
                            color_rgba: col,
                        },
                        SimColoredVertex {
                            pos_mm: p2,
                            normal: n,
                            color_rgba: col,
                        },
                        SimColoredVertex {
                            pos_mm: p3,
                            normal: n,
                            color_rgba: col,
                        },
                    ]);
                    mesh.indices.extend_from_slice(&[
                        base,
                        base + 1,
                        base + 2,
                        base,
                        base + 2,
                        base + 3,
                    ]);
                }
            }
        }
    }
    (!mesh.vertices.is_empty()).then(|| Arc::new(mesh))
}

#[inline]
fn plane_index_span(
    origin: f64,
    step: f64,
    len: usize,
    min_c: f64,
    max_c: f64,
) -> Option<(usize, usize)> {
    if len == 0 || max_c < origin || min_c > origin + (len as f64) * step {
        return None;
    }
    let i0 = (((min_c - origin) / step).floor() as isize).max(0) as usize;
    let i1 = (((max_c - origin) / step).ceil() as isize).clamp(0, (len - 1) as isize) as usize;
    (i0 <= i1).then_some((i0, i1))
}

#[allow(clippy::too_many_arguments)]
fn build_td_bind_group(
    device: &wgpu::Device,
    bgl: &wgpu::BindGroupLayout,
    uniform_buf: &wgpu::Buffer,
    slots_buf: &wgpu::Buffer,
    counts_buf: &wgpu::Buffer,
    target_slots_buf: &wgpu::Buffer,
    target_counts_buf: &wgpu::Buffer,
    profiles_buf: &wgpu::Buffer,
    chords_buf: &wgpu::Buffer,
    tile_data_buf: &wgpu::Buffer,
    accum_and_overflow_buf: &wgpu::Buffer,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("chip_td_sweep_bg"),
        layout: bgl,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: slots_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: counts_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: target_slots_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: target_counts_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 5,
                resource: profiles_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 6,
                resource: chords_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 7,
                resource: tile_data_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 8,
                resource: accum_and_overflow_buf.as_entire_binding(),
            },
        ],
    })
}
