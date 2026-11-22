//! GPU-resident 2.5D Heightfield compute sweeper (`GpuHeightfieldState`) driving
//! `heightfield_sweep.wgsl` with coarse `16×16` CPU tile-binning and lazy CPU readback.

use std::sync::{Arc, Mutex};

use chip_math::{Aabb2, DVec2, DVec3};
use wgpu::util::DeviceExt as _;

use crate::{
    backend::StockCheckpointSnapshot,
    heightfield::HeightfieldStock,
    types::{BatchSweepOutput, SweepChord, SweepMotion, SweepToolSpec},
};

use super::{
    context::{GpuSimContext, GrowableStorageBuffer},
    types::{GPU_TILE_SIZE, GpuSweepChord, GpuToolProfile, GpuViolationAccum, HeightfieldUniforms},
};

struct CpuShadow {
    stock: HeightfieldStock,
    gpu_dirty: bool,
}

/// GPU-resident 2.5D heightfield stock with cached compute buffers and lazy CPU readback.
pub(crate) struct GpuHeightfieldState {
    ctx: Arc<GpuSimContext>,
    nx: usize,
    ny: usize,
    tiles_x: usize,
    tiles_y: usize,
    origin_wcs: DVec3,
    dx_mm: f64,
    dy_mm: f64,
    max_initial_top_z_mm: f64,
    uniform_buf: wgpu::Buffer,
    z_stock_buf: wgpu::Buffer,
    z_bottom_buf: wgpu::Buffer,
    z_target_buf: wgpu::Buffer,
    last_op_buf: wgpu::Buffer,
    profiles_buf: GrowableStorageBuffer,
    chords_buf: GrowableStorageBuffer,
    tile_data_buf: GrowableStorageBuffer,
    violations_buf: wgpu::Buffer,
    violations_readback_buf: wgpu::Buffer,
    z_stock_readback_buf: wgpu::Buffer,
    last_op_readback_buf: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    /// Reusable CPU tile scratch buffers (zero per-batch Vec reallocations).
    tile_bins: Vec<Vec<u32>>,
    tile_data_scratch: Vec<u32>,
    gpu_chords_scratch: Vec<GpuSweepChord>,
    gpu_profiles_scratch: Vec<GpuToolProfile>,
    cpu: Mutex<CpuShadow>,
}

impl std::fmt::Debug for GpuHeightfieldState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuHeightfieldState")
            .field("nx", &self.nx)
            .field("ny", &self.ny)
            .finish()
    }
}

impl GpuHeightfieldState {
    pub(crate) fn new(ctx: Arc<GpuSimContext>, stock: HeightfieldStock) -> Self {
        let device = ctx.device();
        let nx = stock.nx;
        let ny = stock.ny;
        let tiles_x = nx.div_ceil(GPU_TILE_SIZE);
        let tiles_y = ny.div_ceil(GPU_TILE_SIZE);
        let num_tiles = tiles_x * tiles_y;
        let cell_bytes = (nx * ny * std::mem::size_of::<f32>()) as u64;

        let uniforms = HeightfieldUniforms {
            nx: nx as u32,
            ny: ny as u32,
            tiles_x: tiles_x as u32,
            num_tiles: num_tiles as u32,
            dx_mm: stock.dx_mm as f32,
            dy_mm: stock.dy_mm as f32,
            _pad0: 0.0,
            _pad1: 0.0,
        };
        let uniform_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_hf_uniforms"),
            contents: bytemuck::bytes_of(&uniforms),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let z_stock_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_hf_z_stock"),
            contents: bytemuck::cast_slice(&stock.z_stock),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
        });
        let z_bottom_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_hf_z_bottom"),
            contents: bytemuck::cast_slice(&stock.z_bottom),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let z_target_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_hf_z_target"),
            contents: bytemuck::cast_slice(&stock.z_target),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let last_op_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_hf_last_op_idx"),
            contents: bytemuck::cast_slice(&stock.last_op_idx),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
        });

        let profiles_buf = GrowableStorageBuffer::new(
            device,
            "chip_hf_profiles",
            wgpu::BufferUsages::STORAGE,
            16 * std::mem::size_of::<GpuToolProfile>(),
        );
        let chords_buf = GrowableStorageBuffer::new(
            device,
            "chip_hf_chords",
            wgpu::BufferUsages::STORAGE,
            64 * std::mem::size_of::<GpuSweepChord>(),
        );
        let tile_data_buf = GrowableStorageBuffer::new(
            device,
            "chip_hf_tile_data",
            wgpu::BufferUsages::STORAGE,
            (num_tiles + 1 + 256) * std::mem::size_of::<u32>(),
        );

        let accum_size = std::mem::size_of::<GpuViolationAccum>() as u64;
        let violations_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_hf_violations"),
            size: accum_size,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let violations_readback_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_hf_violations_readback"),
            size: accum_size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let z_stock_readback_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_hf_z_stock_readback"),
            size: cell_bytes.max(16),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let last_op_readback_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_hf_last_op_readback"),
            size: cell_bytes.max(16),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let bind_group = build_hf_bind_group(
            device,
            &ctx.heightfield_bgl,
            &uniform_buf,
            &z_stock_buf,
            &z_bottom_buf,
            &z_target_buf,
            &last_op_buf,
            profiles_buf.buffer(),
            chords_buf.buffer(),
            tile_data_buf.buffer(),
            &violations_buf,
        );

        let origin_wcs = stock.origin_wcs;
        let dx_mm = stock.dx_mm;
        let dy_mm = stock.dy_mm;
        let max_initial_top_z_mm = stock.max_initial_top_z_mm;

        Self {
            ctx,
            nx,
            ny,
            tiles_x,
            tiles_y,
            origin_wcs,
            dx_mm,
            dy_mm,
            max_initial_top_z_mm,
            uniform_buf,
            z_stock_buf,
            z_bottom_buf,
            z_target_buf,
            last_op_buf,
            profiles_buf,
            chords_buf,
            tile_data_buf,
            violations_buf,
            violations_readback_buf,
            z_stock_readback_buf,
            last_op_readback_buf,
            bind_group,
            tile_bins: vec![Vec::new(); num_tiles],
            tile_data_scratch: Vec::with_capacity(num_tiles + 1 + 256),
            gpu_chords_scratch: Vec::with_capacity(64),
            gpu_profiles_scratch: Vec::with_capacity(16),
            cpu: Mutex::new(CpuShadow {
                stock,
                gpu_dirty: false,
            }),
        }
    }

    /// Dispatches a tiled `16×16` GPU heightfield compute sweep over `chords` in strict
    /// chronological order per tile, reading back only the 176-byte `GpuViolationAccum`.
    pub(crate) fn sweep_batch(
        &mut self,
        chords: &[SweepChord],
        profiles: &[SweepToolSpec],
        verify: bool,
    ) -> BatchSweepOutput {
        if chords.is_empty() || profiles.is_empty() {
            return BatchSweepOutput::default();
        }

        for bin in &mut self.tile_bins {
            bin.clear();
        }
        self.gpu_chords_scratch.clear();

        let mut total_binned = 0usize;
        for (raw_idx, ch) in chords.iter().enumerate() {
            let Some(tool) = profiles.get(ch.profile_idx) else {
                continue;
            };
            let prof = &tool.profile;
            let check_holder = verify
                && (motion_min_tip_z(&ch.motion) + prof.stickout_length_mm
                    < self.max_initial_top_z_mm - 0.02);
            let sweep_r = if verify {
                if check_holder {
                    prof.max_assembly_radius_mm()
                } else {
                    prof.cutting_radius_mm.max(prof.shank_radius_mm)
                }
            } else {
                prof.cutting_radius_mm
            };
            let aabb = motion_xy_bounds(&ch.motion, sweep_r);
            let (Some((i0, i1)), Some((j0, j1))) = (
                self.index_span_x(aabb.min.x, aabb.max.x),
                self.index_span_y(aabb.min.y, aabb.max.y),
            ) else {
                continue;
            };

            let gpu_idx = self.gpu_chords_scratch.len() as u32;
            self.gpu_chords_scratch
                .push(GpuSweepChord::from_heightfield_chord(
                    ch,
                    self.origin_wcs,
                    verify,
                    check_holder,
                    raw_idx as u32,
                ));

            let tx0 = i0 / GPU_TILE_SIZE;
            let tx1 = i1 / GPU_TILE_SIZE;
            let ty0 = j0 / GPU_TILE_SIZE;
            let ty1 = j1 / GPU_TILE_SIZE;
            for ty in ty0..=ty1 {
                let row_tile = ty * self.tiles_x;
                for tx in tx0..=tx1 {
                    self.tile_bins[row_tile + tx].push(gpu_idx);
                    total_binned += 1;
                }
            }
        }

        if total_binned == 0 || self.gpu_chords_scratch.is_empty() {
            return BatchSweepOutput::default();
        }

        let num_tiles = self.tiles_x * self.tiles_y;
        self.tile_data_scratch.clear();
        self.tile_data_scratch.reserve(num_tiles + 1 + total_binned);
        let mut cursor = 0u32;
        for bin in &self.tile_bins {
            self.tile_data_scratch.push(cursor);
            cursor += bin.len() as u32;
        }
        self.tile_data_scratch.push(cursor);
        for bin in &self.tile_bins {
            self.tile_data_scratch.extend_from_slice(bin);
        }

        self.gpu_profiles_scratch.clear();
        self.gpu_profiles_scratch
            .extend(profiles.iter().map(GpuToolProfile::from_tool_spec));

        let device = self.ctx.device();
        let queue = self.ctx.queue();
        let mut rebind = false;
        rebind |= self
            .profiles_buf
            .upload_slice(device, queue, &self.gpu_profiles_scratch);
        rebind |= self
            .chords_buf
            .upload_slice(device, queue, &self.gpu_chords_scratch);
        rebind |= self
            .tile_data_buf
            .upload_slice(device, queue, &self.tile_data_scratch);

        if rebind {
            self.bind_group = build_hf_bind_group(
                device,
                &self.ctx.heightfield_bgl,
                &self.uniform_buf,
                &self.z_stock_buf,
                &self.z_bottom_buf,
                &self.z_target_buf,
                &self.last_op_buf,
                self.profiles_buf.buffer(),
                self.chords_buf.buffer(),
                self.tile_data_buf.buffer(),
                &self.violations_buf,
            );
        }

        let zero_accum = GpuViolationAccum::default();
        queue.write_buffer(&self.violations_buf, 0, bytemuck::bytes_of(&zero_accum));

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("chip_hf_sweep_encoder"),
        });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("chip_hf_sweep_pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.ctx.heightfield_pipeline);
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.dispatch_workgroups(self.tiles_x as u32, self.tiles_y as u32, 1);
        }

        let accum_bytes = std::mem::size_of::<GpuViolationAccum>();
        encoder.copy_buffer_to_buffer(
            &self.violations_buf,
            0,
            &self.violations_readback_buf,
            0,
            accum_bytes as u64,
        );
        queue.submit(std::iter::once(encoder.finish()));

        let mut raw = [0u8; std::mem::size_of::<GpuViolationAccum>()];
        let mut accum = GpuViolationAccum::default();
        if self
            .ctx
            .readback_bytes(&self.violations_readback_buf, accum_bytes, &mut raw)
        {
            accum = *bytemuck::from_bytes(&raw);
        }

        let nx = self.nx;
        let ox = self.origin_wcs.x;
        let oy = self.origin_wcs.y;
        let oz = self.origin_wcs.z;
        let dx = self.dx_mm;
        let dy = self.dy_mm;

        let (mutated, violations) = accum.into_span_violations(|cell_idx, loc_z_local| {
            let idx = cell_idx as usize;
            let i = idx % nx;
            let j = idx / nx;
            DVec3::new(
                ox + (i as f64 + 0.5) * dx,
                oy + (j as f64 + 0.5) * dy,
                oz + f64::from(loc_z_local),
            )
        });

        if mutated {
            let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
            guard.gpu_dirty = true;
        }

        BatchSweepOutput {
            mutated,
            violations,
        }
    }

    /// Synchronizes GPU `z_stock` and `last_op_idx` back into the CPU shadow copy when dirty,
    /// then invokes `f` on the up-to-date [`HeightfieldStock`].
    pub(crate) fn with_synced_stock<R>(&self, f: impl FnOnce(&HeightfieldStock) -> R) -> R {
        let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
        if guard.gpu_dirty {
            self.sync_gpu_to_cpu_locked(&mut guard);
        }
        f(&guard.stock)
    }

    /// Restores both CPU shadow and GPU storage buffers from a checkpoint snapshot.
    pub(crate) fn reset_from_checkpoint(&mut self, cp: &StockCheckpointSnapshot) {
        if let StockCheckpointSnapshot::Heightfield {
            z_stock,
            last_op_idx,
            ..
        } = cp
        {
            let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
            guard.stock.z_stock.clone_from(z_stock);
            guard.stock.last_op_idx.clone_from(last_op_idx);
            guard.gpu_dirty = false;
            self.ctx
                .queue()
                .write_buffer(&self.z_stock_buf, 0, bytemuck::cast_slice(z_stock));
            self.ctx
                .queue()
                .write_buffer(&self.last_op_buf, 0, bytemuck::cast_slice(last_op_idx));
        }
    }

    /// Consumes this GPU state and returns the fully synchronized CPU [`HeightfieldStock`].
    pub(crate) fn into_stock(self) -> HeightfieldStock {
        {
            let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
            if guard.gpu_dirty {
                self.sync_gpu_to_cpu_locked(&mut guard);
            }
        }
        self.cpu
            .into_inner()
            .unwrap_or_else(|e| e.into_inner())
            .stock
    }

    fn sync_gpu_to_cpu_locked(&self, guard: &mut CpuShadow) {
        let cell_count = self.nx * self.ny;
        let byte_len = cell_count * std::mem::size_of::<f32>();
        if byte_len == 0 {
            guard.gpu_dirty = false;
            return;
        }
        let mut encoder =
            self.ctx
                .device()
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("chip_hf_sync_readback_encoder"),
                });
        encoder.copy_buffer_to_buffer(
            &self.z_stock_buf,
            0,
            &self.z_stock_readback_buf,
            0,
            byte_len as u64,
        );
        encoder.copy_buffer_to_buffer(
            &self.last_op_buf,
            0,
            &self.last_op_readback_buf,
            0,
            byte_len as u64,
        );
        self.ctx.queue().submit(std::iter::once(encoder.finish()));

        let z_dst: &mut [u8] = bytemuck::cast_slice_mut(&mut guard.stock.z_stock);
        let _ = self
            .ctx
            .readback_bytes(&self.z_stock_readback_buf, byte_len, z_dst);
        let op_dst: &mut [u8] = bytemuck::cast_slice_mut(&mut guard.stock.last_op_idx);
        let _ = self
            .ctx
            .readback_bytes(&self.last_op_readback_buf, byte_len, op_dst);
        guard.gpu_dirty = false;
    }

    fn index_span_x(&self, min_x: f64, max_x: f64) -> Option<(usize, usize)> {
        if max_x < self.origin_wcs.x || min_x > self.origin_wcs.x + (self.nx as f64) * self.dx_mm {
            return None;
        }
        let i0 = (((min_x - self.origin_wcs.x) / self.dx_mm).floor() as isize).max(0) as usize;
        let i1 = (((max_x - self.origin_wcs.x) / self.dx_mm).ceil() as isize)
            .clamp(0, (self.nx - 1) as isize) as usize;
        (i0 <= i1).then_some((i0, i1))
    }

    fn index_span_y(&self, min_y: f64, max_y: f64) -> Option<(usize, usize)> {
        if max_y < self.origin_wcs.y || min_y > self.origin_wcs.y + (self.ny as f64) * self.dy_mm {
            return None;
        }
        let j0 = (((min_y - self.origin_wcs.y) / self.dy_mm).floor() as isize).max(0) as usize;
        let j1 = (((max_y - self.origin_wcs.y) / self.dy_mm).ceil() as isize)
            .clamp(0, (self.ny - 1) as isize) as usize;
        (j0 <= j1).then_some((j0, j1))
    }
}

#[allow(clippy::too_many_arguments)]
fn build_hf_bind_group(
    device: &wgpu::Device,
    bgl: &wgpu::BindGroupLayout,
    uniform_buf: &wgpu::Buffer,
    z_stock_buf: &wgpu::Buffer,
    z_bottom_buf: &wgpu::Buffer,
    z_target_buf: &wgpu::Buffer,
    last_op_buf: &wgpu::Buffer,
    profiles_buf: &wgpu::Buffer,
    chords_buf: &wgpu::Buffer,
    tile_data_buf: &wgpu::Buffer,
    violations_buf: &wgpu::Buffer,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("chip_hf_sweep_bg"),
        layout: bgl,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: z_stock_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: z_bottom_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: z_target_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: last_op_buf.as_entire_binding(),
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
                resource: violations_buf.as_entire_binding(),
            },
        ],
    })
}

#[inline]
fn motion_min_tip_z(prim: &SweepMotion) -> f64 {
    match *prim {
        SweepMotion::Linear { p0, p1, .. } => p0.z.min(p1.z),
        SweepMotion::PlanarArcXY { start, end, .. } => start.z.min(end.z),
    }
}

#[inline]
fn motion_xy_bounds(prim: &SweepMotion, expand_r: f64) -> Aabb2 {
    match *prim {
        SweepMotion::Linear { p0, p1, .. } => Aabb2 {
            min: DVec2::new(p0.x.min(p1.x) - expand_r, p0.y.min(p1.y) - expand_r),
            max: DVec2::new(p0.x.max(p1.x) + expand_r, p0.y.max(p1.y) + expand_r),
        },
        SweepMotion::PlanarArcXY {
            center_xy,
            radius_mm,
            ..
        } => {
            let r_tot = radius_mm + expand_r;
            Aabb2 {
                min: DVec2::new(center_xy.x - r_tot, center_xy.y - r_tot),
                max: DVec2::new(center_xy.x + r_tot, center_xy.y + r_tot),
            }
        }
    }
}
