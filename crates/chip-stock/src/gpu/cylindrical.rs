//! GPU-resident 4th-Axis Cylindrical compute sweeper (`GpuCylindricalState`) driving
//! `cylindrical_sweep.wgsl` with coarse `16×16` polar `(iu, itheta)` tile-binning, exact `2π`
//! wraparound & `full_circle` handling, chuck keep-out verification, and lazy CPU readback.

use std::{
    f64::consts::TAU,
    sync::{Arc, Mutex},
};

use chip_math::DVec3;
use wgpu::util::DeviceExt as _;

use crate::{
    backend::StockCheckpointSnapshot,
    cylindrical::CylindricalStock,
    types::{
        Axis3, BatchSweepOutput, RotaryStockSpec, SpanRuleViolations, SweepChord, SweepToolSpec,
    },
};

use super::{
    context::{GpuSimContext, GrowableStorageBuffer},
    types::{
        CylindricalUniforms, GPU_TILE_SIZE, GpuRotaryPose, GpuToolProfile, GpuViolationAccum,
        POSE_FLAG_CHECK_HOLDER, POSE_FLAG_RAPID, POSE_FLAG_VERIFY,
    },
};

struct CpuShadow {
    stock: CylindricalStock,
    gpu_dirty: bool,
}

/// GPU-resident 4th-axis polar cylindrical stock with cached compute buffers and lazy CPU readback.
pub(crate) struct GpuCylindricalState {
    ctx: Arc<GpuSimContext>,
    rotary: RotaryStockSpec,
    u_min: f64,
    u_max: f64,
    du: f64,
    dtheta: f64,
    nu: usize,
    ntheta: usize,
    tiles_u: usize,
    tiles_theta: usize,
    uniform_buf: wgpu::Buffer,
    r_stock_buf: wgpu::Buffer,
    r_target_buf: wgpu::Buffer,
    last_op_buf: wgpu::Buffer,
    profiles_buf: GrowableStorageBuffer,
    poses_buf: GrowableStorageBuffer,
    tile_data_buf: GrowableStorageBuffer,
    violations_buf: wgpu::Buffer,
    violations_readback_buf: wgpu::Buffer,
    r_stock_readback_buf: wgpu::Buffer,
    last_op_readback_buf: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    tile_bins: Vec<Vec<u32>>,
    tile_data_scratch: Vec<u32>,
    gpu_poses_scratch: Vec<GpuRotaryPose>,
    pose_tips_wcs_scratch: Vec<DVec3>,
    gpu_profiles_scratch: Vec<GpuToolProfile>,
    cpu: Mutex<CpuShadow>,
}

impl std::fmt::Debug for GpuCylindricalState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuCylindricalState")
            .field("nu", &self.nu)
            .field("ntheta", &self.ntheta)
            .finish()
    }
}

impl GpuCylindricalState {
    pub(crate) fn new(ctx: Arc<GpuSimContext>, stock: CylindricalStock) -> Self {
        let device = ctx.device();
        let nu = stock.nu;
        let ntheta = stock.ntheta;
        let tiles_u = nu.div_ceil(GPU_TILE_SIZE);
        let tiles_theta = ntheta.div_ceil(GPU_TILE_SIZE);
        let num_tiles = tiles_u * tiles_theta;
        let cell_bytes = (nu * ntheta * std::mem::size_of::<f32>()) as u64;

        let (e1, e2, u_vec) = stock.rotary.basis();
        let u_center = stock.rotary.center_wcs.dot(u_vec);
        let uniforms = CylindricalUniforms {
            e1_and_u_min: [e1.x as f32, e1.y as f32, e1.z as f32, stock.u_min as f32],
            e2_and_du: [e2.x as f32, e2.y as f32, e2.z as f32, stock.du as f32],
            u_vec_and_dtheta: [
                u_vec.x as f32,
                u_vec.y as f32,
                u_vec.z as f32,
                stock.dtheta as f32,
            ],
            dims: [
                nu as u32,
                ntheta as u32,
                tiles_theta as u32,
                num_tiles as u32,
            ],
            params: [
                u_center as f32,
                stock.rotary.direction_sign as f32,
                0.0,
                0.0,
            ],
        };
        let uniform_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_cyl_uniforms"),
            contents: bytemuck::bytes_of(&uniforms),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let r_stock_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_cyl_r_stock"),
            contents: bytemuck::cast_slice(&stock.r_stock),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
        });
        let r_target_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_cyl_r_target"),
            contents: bytemuck::cast_slice(&stock.r_target),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let last_op_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_cyl_last_op_idx"),
            contents: bytemuck::cast_slice(&stock.last_op_idx),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
        });

        let profiles_buf = GrowableStorageBuffer::new(
            device,
            "chip_cyl_profiles",
            wgpu::BufferUsages::STORAGE,
            16 * std::mem::size_of::<GpuToolProfile>(),
        );
        let poses_buf = GrowableStorageBuffer::new(
            device,
            "chip_cyl_poses",
            wgpu::BufferUsages::STORAGE,
            256 * std::mem::size_of::<GpuRotaryPose>(),
        );
        let tile_data_buf = GrowableStorageBuffer::new(
            device,
            "chip_cyl_tile_data",
            wgpu::BufferUsages::STORAGE,
            (num_tiles + 1 + 1024) * std::mem::size_of::<u32>(),
        );

        let accum_size = std::mem::size_of::<GpuViolationAccum>() as u64;
        let violations_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_cyl_violations"),
            size: accum_size,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let violations_readback_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_cyl_violations_readback"),
            size: accum_size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let r_stock_readback_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_cyl_r_stock_readback"),
            size: cell_bytes.max(16),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let last_op_readback_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_cyl_last_op_readback"),
            size: cell_bytes.max(16),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let bind_group = build_cyl_bind_group(
            device,
            &ctx.cylindrical_bgl,
            &uniform_buf,
            &r_stock_buf,
            &r_target_buf,
            &last_op_buf,
            profiles_buf.buffer(),
            poses_buf.buffer(),
            tile_data_buf.buffer(),
            &violations_buf,
        );

        let rotary = stock.rotary;
        let u_min = stock.u_min;
        let u_max = stock.u_max;
        let du = stock.du;
        let dtheta = stock.dtheta;

        Self {
            ctx,
            rotary,
            u_min,
            u_max,
            du,
            dtheta,
            nu,
            ntheta,
            tiles_u,
            tiles_theta,
            uniform_buf,
            r_stock_buf,
            r_target_buf,
            last_op_buf,
            profiles_buf,
            poses_buf,
            tile_data_buf,
            violations_buf,
            violations_readback_buf,
            r_stock_readback_buf,
            last_op_readback_buf,
            bind_group,
            tile_bins: vec![Vec::new(); num_tiles],
            tile_data_scratch: Vec::with_capacity(num_tiles + 1 + 1024),
            gpu_poses_scratch: Vec::with_capacity(256),
            pose_tips_wcs_scratch: Vec::with_capacity(256),
            gpu_profiles_scratch: Vec::with_capacity(16),
            cpu: Mutex::new(CpuShadow {
                stock,
                gpu_dirty: false,
            }),
        }
    }

    /// Subdivides rotary chords identically to `cylindrical/sweep.rs`, checks rotary chuck
    /// keep-out on CPU in exact `f64`, bins poses into `16×16` polar tiles in strict
    /// chronological order, and executes `cylindrical_sweep.wgsl`.
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
        self.gpu_poses_scratch.clear();
        self.pose_tips_wcs_scratch.clear();

        let step_u = (0.5 * self.du).max(0.15);
        let step_deg = (0.5 * self.dtheta.to_degrees()).max(0.75);
        let u_vec = self.rotary.axis_unit();
        let has_chuck = self.rotary.chuck_u_max > self.rotary.chuck_u_min + 1e-4;

        let mut chuck_violations = SpanRuleViolations::default();
        let mut total_binned = 0usize;

        for ch in chords {
            let Some(tool) = profiles.get(ch.profile_idx) else {
                continue;
            };
            let prof = &tool.profile;
            let r_cut = prof.cutting_radius_mm.max(0.05);
            let r_holder = prof.holder_radius_mm.max(r_cut);
            let stickout = prof.stickout_length_mm.max(prof.flute_length_mm);
            let max_possible_r =
                self.rotary.stock_outer_radius_mm + if verify { r_holder } else { r_cut } + 2.0;

            let lin_len = ch.motion.length_mm();
            let d_deg = ch.motion.rotary_delta_deg();
            let n_lin = (lin_len / step_u).ceil() as usize;
            let n_rot = (d_deg / step_deg).ceil() as usize;
            let n_steps = n_lin.max(n_rot).clamp(1, 2_000);

            for k in 0..=n_steps {
                let t = k as f64 / n_steps as f64;
                let tip_wcs = ch.motion.sample_point(t);
                let rot_deg = ch.motion.sample_rotary_deg(t);

                let tip_part = self.rotary.rotate_point_wcs(tip_wcs, -rot_deg);
                let spindle_part = self.rotary.rotate_vector_wcs(DVec3::Z, -rot_deg);
                let (u_tip, theta_tip, r_tip) = self.rotary.wcs_to_cylindrical(tip_part);

                // 1. Exact rotary chuck keep-out check (matches cylindrical/sweep.rs lines 88-98)
                if verify && has_chuck {
                    let chuck_clearance_r = self.rotary.chuck_radius_mm + r_cut;
                    let u_overlaps_chuck = (u_tip + r_cut) >= self.rotary.chuck_u_min
                        && (u_tip - r_cut) <= self.rotary.chuck_u_max;
                    if u_overlaps_chuck && r_tip < chuck_clearance_r - 0.05 {
                        let excess = chuck_clearance_r - r_tip;
                        chuck_violations.record_holder(excess, tip_wcs);
                    }
                }

                let tip_part_rel = tip_part - self.rotary.center_wcs;
                let tip_h_above_axis = tip_part_rel.dot(spindle_part);
                if tip_h_above_axis > max_possible_r {
                    continue;
                }

                let holder_can_reach_stock = verify
                    && (tip_h_above_axis + stickout) <= self.rotary.stock_outer_radius_mm + 1.0;
                let r_footprint = if holder_can_reach_stock {
                    r_holder
                } else {
                    r_cut
                };

                let u_reach = r_footprint
                    + if holder_can_reach_stock {
                        (stickout + 10.0) * spindle_part.dot(u_vec).abs()
                    } else {
                        prof.flute_length_mm * spindle_part.dot(u_vec).abs()
                    }
                    + self.du;
                let u_lo = (u_tip - u_reach).max(self.u_min);
                let u_hi = (u_tip + u_reach).min(self.u_max);
                if u_lo > u_hi {
                    continue;
                }
                let iu_min = (((u_lo - self.u_min) / self.du).floor() as isize)
                    .clamp(0, self.nu as isize - 1) as usize;
                let iu_max = (((u_hi - self.u_min) / self.du).ceil() as isize)
                    .clamp(0, self.nu as isize - 1) as usize;

                let radial_tip = tip_part_rel - u_vec * tip_part_rel.dot(u_vec);
                let c_off_center = radial_tip.cross(spindle_part).length();
                let full_circle = r_tip <= r_footprint + 1.0 || self.rotary.axis == Axis3::Z;

                let mut flags = 0u32;
                if ch.is_rapid {
                    flags |= POSE_FLAG_RAPID;
                }
                if verify {
                    flags |= POSE_FLAG_VERIFY;
                }
                if holder_can_reach_stock {
                    flags |= POSE_FLAG_CHECK_HOLDER;
                }

                let pose_idx = self.gpu_poses_scratch.len() as u32;
                self.gpu_poses_scratch.push(GpuRotaryPose {
                    tip_and_rapid_tol: [
                        tip_part_rel.x as f32,
                        tip_part_rel.y as f32,
                        tip_part_rel.z as f32,
                        ch.verify_tol.rapid_crash_tol_mm as f32,
                    ],
                    spindle_and_gouge_tol: [
                        spindle_part.x as f32,
                        spindle_part.y as f32,
                        spindle_part.z as f32,
                        ch.verify_tol.gouge_tol_mm as f32,
                    ],
                    meta: [ch.op_idx, ch.profile_idx as u32, flags, pose_idx],
                });
                self.pose_tips_wcs_scratch.push(tip_wcs);

                let tu0 = iu_min / GPU_TILE_SIZE;
                let tu1 = iu_max / GPU_TILE_SIZE;

                if full_circle {
                    for tu in tu0..=tu1 {
                        let row_tile = tu * self.tiles_theta;
                        for tt in 0..self.tiles_theta {
                            self.tile_bins[row_tile + tt].push(pose_idx);
                            total_binned += 1;
                        }
                    }
                } else {
                    let off_center_extra = (c_off_center / r_tip.max(1.0)).clamp(0.0, 1.0).asin();
                    let half_angle = ((r_footprint + 1.0) / r_tip).clamp(0.0, 1.0).asin()
                        + off_center_extra
                        + 0.12;
                    let center_norm = theta_tip.rem_euclid(TAU);
                    let ic = (center_norm / self.dtheta).round() as isize;
                    let hs = ((half_angle / self.dtheta).ceil() as isize + 1)
                        .min((self.ntheta / 2) as isize);
                    let it_start = ic - hs;
                    let it_end = ic + hs;

                    if it_end - it_start + 1 >= self.ntheta as isize {
                        for tu in tu0..=tu1 {
                            let row_tile = tu * self.tiles_theta;
                            for tt in 0..self.tiles_theta {
                                self.tile_bins[row_tile + tt].push(pose_idx);
                                total_binned += 1;
                            }
                        }
                    } else {
                        let s0 = it_start.rem_euclid(self.ntheta as isize) as usize;
                        let e0 = it_end.rem_euclid(self.ntheta as isize) as usize;
                        if s0 <= e0 {
                            let tt0 = s0 / GPU_TILE_SIZE;
                            let tt1 = e0 / GPU_TILE_SIZE;
                            for tu in tu0..=tu1 {
                                let row_tile = tu * self.tiles_theta;
                                for tt in tt0..=tt1 {
                                    self.tile_bins[row_tile + tt].push(pose_idx);
                                    total_binned += 1;
                                }
                            }
                        } else {
                            let ta0 = s0 / GPU_TILE_SIZE;
                            let ta1 = (self.ntheta - 1) / GPU_TILE_SIZE;
                            let tb1 = e0 / GPU_TILE_SIZE;
                            if tb1 >= ta0 {
                                for tu in tu0..=tu1 {
                                    let row_tile = tu * self.tiles_theta;
                                    for tt in 0..self.tiles_theta {
                                        self.tile_bins[row_tile + tt].push(pose_idx);
                                        total_binned += 1;
                                    }
                                }
                            } else {
                                for tu in tu0..=tu1 {
                                    let row_tile = tu * self.tiles_theta;
                                    for tt in 0..=tb1 {
                                        self.tile_bins[row_tile + tt].push(pose_idx);
                                        total_binned += 1;
                                    }
                                    for tt in ta0..=ta1 {
                                        self.tile_bins[row_tile + tt].push(pose_idx);
                                        total_binned += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if total_binned == 0 || self.gpu_poses_scratch.is_empty() {
            return BatchSweepOutput {
                mutated: false,
                violations: chuck_violations,
            };
        }

        let num_tiles = self.tiles_u * self.tiles_theta;
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
            .poses_buf
            .upload_slice(device, queue, &self.gpu_poses_scratch);
        rebind |= self
            .tile_data_buf
            .upload_slice(device, queue, &self.tile_data_scratch);

        if rebind {
            self.bind_group = build_cyl_bind_group(
                device,
                &self.ctx.cylindrical_bgl,
                &self.uniform_buf,
                &self.r_stock_buf,
                &self.r_target_buf,
                &self.last_op_buf,
                self.profiles_buf.buffer(),
                self.poses_buf.buffer(),
                self.tile_data_buf.buffer(),
                &self.violations_buf,
            );
        }

        let zero_accum = GpuViolationAccum::default();
        queue.write_buffer(&self.violations_buf, 0, bytemuck::bytes_of(&zero_accum));

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("chip_cyl_sweep_encoder"),
        });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("chip_cyl_sweep_pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.ctx.cylindrical_pipeline);
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.dispatch_workgroups(self.tiles_theta as u32, self.tiles_u as u32, 1);
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

        let pose_tips = &self.pose_tips_wcs_scratch;
        let (mutated, mut violations) = accum.into_span_violations(|pose_idx, _z_unused| {
            pose_tips
                .get(pose_idx as usize)
                .copied()
                .unwrap_or(self.rotary.center_wcs)
        });
        violations.merge(chuck_violations);

        if mutated {
            let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
            guard.gpu_dirty = true;
        }

        BatchSweepOutput {
            mutated,
            violations,
        }
    }

    /// Synchronizes GPU `r_stock` and `last_op_idx` back into the CPU shadow copy when dirty,
    /// then invokes `f` on the up-to-date [`CylindricalStock`].
    pub(crate) fn with_synced_stock<R>(&self, f: impl FnOnce(&CylindricalStock) -> R) -> R {
        let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
        if guard.gpu_dirty {
            self.sync_gpu_to_cpu_locked(&mut guard);
        }
        f(&guard.stock)
    }

    /// Restores both CPU shadow and GPU storage buffers from a checkpoint snapshot.
    pub(crate) fn reset_from_checkpoint(&mut self, cp: &StockCheckpointSnapshot) {
        if let StockCheckpointSnapshot::Cylindrical {
            r_stock,
            last_op_idx,
            ..
        } = cp
        {
            let mut guard = self.cpu.lock().unwrap_or_else(|e| e.into_inner());
            guard.stock.r_stock.clone_from(r_stock);
            guard.stock.last_op_idx.clone_from(last_op_idx);
            guard.gpu_dirty = false;
            self.ctx
                .queue()
                .write_buffer(&self.r_stock_buf, 0, bytemuck::cast_slice(r_stock));
            self.ctx
                .queue()
                .write_buffer(&self.last_op_buf, 0, bytemuck::cast_slice(last_op_idx));
        }
    }

    /// Consumes this GPU state and returns the fully synchronized CPU [`CylindricalStock`].
    pub(crate) fn into_stock(self) -> CylindricalStock {
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
        let cell_count = self.nu * self.ntheta;
        let byte_len = cell_count * std::mem::size_of::<f32>();
        if byte_len == 0 {
            guard.gpu_dirty = false;
            return;
        }
        let mut encoder =
            self.ctx
                .device()
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("chip_cyl_sync_readback_encoder"),
                });
        encoder.copy_buffer_to_buffer(
            &self.r_stock_buf,
            0,
            &self.r_stock_readback_buf,
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

        let r_dst: &mut [u8] = bytemuck::cast_slice_mut(&mut guard.stock.r_stock);
        let _ = self
            .ctx
            .readback_bytes(&self.r_stock_readback_buf, byte_len, r_dst);
        let op_dst: &mut [u8] = bytemuck::cast_slice_mut(&mut guard.stock.last_op_idx);
        let _ = self
            .ctx
            .readback_bytes(&self.last_op_readback_buf, byte_len, op_dst);
        guard.gpu_dirty = false;
    }
}

#[allow(clippy::too_many_arguments)]
fn build_cyl_bind_group(
    device: &wgpu::Device,
    bgl: &wgpu::BindGroupLayout,
    uniform_buf: &wgpu::Buffer,
    r_stock_buf: &wgpu::Buffer,
    r_target_buf: &wgpu::Buffer,
    last_op_buf: &wgpu::Buffer,
    profiles_buf: &wgpu::Buffer,
    poses_buf: &wgpu::Buffer,
    tile_data_buf: &wgpu::Buffer,
    violations_buf: &wgpu::Buffer,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("chip_cyl_sweep_bg"),
        layout: bgl,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: r_stock_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: r_target_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: last_op_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: profiles_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 5,
                resource: poses_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 6,
                resource: tile_data_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 7,
                resource: violations_buf.as_entire_binding(),
            },
        ],
    })
}
