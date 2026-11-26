//! Direct `chip-bvh` GPU upload (`FlatBvhNode` 32B + `GpuTriangle` 48B) and compute dispatchers
//! for GPU Tri-Dexel target voxelization, Cylindrical inward radial raycast, Heightfield top-down
//! raycast, and point-to-mesh closest-distance queries (`bvh_distance.wgsl`).

use chip_bvh::{FlatBvhNode, GpuTriangle, TriMesh3D};
use chip_math::DVec3;
use wgpu::util::DeviceExt as _;

use crate::{
    cylindrical::CylindricalStock,
    heightfield::HeightfieldStock,
    tridexel::{DexelSpan, FlatDexelSpan, INLINE_DEXEL_SLOTS, TriDexelStock},
    types::{Axis3, RotaryStockSpec, UNCUT_OP_IDX},
};

use super::{
    context::GpuSimContext,
    types::{BvhDispatchUniforms, GPU_TILE_SIZE},
};

/// Uploaded GPU-resident flat binary BVH (`FlatBvhNode` + `GpuTriangle`) with zero CPU conversion.
pub struct GpuMeshBvh {
    nodes_buf: wgpu::Buffer,
    tris_buf: wgpu::Buffer,
    node_count: u32,
    tri_count: u32,
    bounds_min: DVec3,
    bounds_max: DVec3,
}

impl std::fmt::Debug for GpuMeshBvh {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuMeshBvh")
            .field("node_count", &self.node_count)
            .field("tri_count", &self.tri_count)
            .finish()
    }
}

impl GpuMeshBvh {
    /// Uploads `mesh.nodes()` (32B `Pod`) and `mesh.gpu_triangles()` (48B `Pod`) directly to GPU.
    pub fn upload(ctx: &GpuSimContext, mesh: &TriMesh3D) -> Self {
        let device = ctx.device();
        let nodes = mesh.nodes();
        let tris = mesh.gpu_triangles();
        let node_count = nodes.len() as u32;
        let tri_count = tris.len() as u32;

        let dummy_node = [FlatBvhNode {
            aabb_min: [0.0; 3],
            left_or_first: 0,
            aabb_max: [0.0; 3],
            tri_count: 0,
        }];
        let dummy_tri = [GpuTriangle {
            v0: [0.0; 3],
            face_id: 0,
            v1: [0.0; 3],
            flags: 0,
            v2: [0.0; 3],
            prim_id: 0,
        }];

        let nodes_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_bvh_nodes"),
            contents: if nodes.is_empty() {
                bytemuck::cast_slice(&dummy_node)
            } else {
                bytemuck::cast_slice(nodes)
            },
            usage: wgpu::BufferUsages::STORAGE,
        });
        let tris_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_bvh_tris"),
            contents: if tris.is_empty() {
                bytemuck::cast_slice(&dummy_tri)
            } else {
                bytemuck::cast_slice(&tris)
            },
            usage: wgpu::BufferUsages::STORAGE,
        });

        let bounds = mesh.bounds();
        Self {
            nodes_buf,
            tris_buf,
            node_count,
            tri_count,
            bounds_min: if bounds.is_valid() {
                bounds.min
            } else {
                DVec3::ZERO
            },
            bounds_max: if bounds.is_valid() {
                bounds.max
            } else {
                DVec3::ZERO
            },
        }
    }
}

impl GpuSimContext {
    /// Voxelizes a closed WCS-space [`TriMesh3D`] target solid onto a [`TriDexelStock`] lattice
    /// matching `template` directly on GPU via `bvh_distance.wgsl` (`raycast_all_hits` even-odd
    /// parity across all 3 orthogonal axes), with exact CPU overflow fallback for any ray with
    /// `> K = 4` spans.
    pub fn rasterize_tridexel_target_mesh(
        &self,
        template: &TriDexelStock,
        target_mesh: &TriMesh3D,
    ) -> TriDexelStock {
        let mut target = TriDexelStock::empty_like(template);
        if target_mesh.is_empty() {
            return target;
        }

        let rx = target.planes[0].ray_count();
        let ry = target.planes[1].ray_count();
        let rz = target.planes[2].ray_count();
        let plane_ray_offsets = [0, rx, rx + ry];
        let total_rays = rx + ry + rz;
        let total_slots = total_rays * INLINE_DEXEL_SLOTS;

        let device = self.device();
        let slots_bytes = (total_slots * std::mem::size_of::<FlatDexelSpan>()) as u64;
        let counts_bytes = (total_rays * std::mem::size_of::<u32>()) as u64;

        let t_slots_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_bvh_td_slots"),
            size: slots_bytes.max(16),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let t_counts_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_bvh_td_counts"),
            size: counts_bytes.max(16),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let slots_rb = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_bvh_td_slots_rb"),
            size: slots_bytes.max(16),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let counts_rb = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_bvh_td_counts_rb"),
            size: counts_bytes.max(16),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        self.rasterize_tridexel_into_buffers(
            &mut target,
            plane_ray_offsets,
            target_mesh,
            &t_slots_buf,
            &t_counts_buf,
            &slots_rb,
            &counts_rb,
        );
        target
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn rasterize_tridexel_into_buffers(
        &self,
        target_out: &mut TriDexelStock,
        plane_ray_offsets: [usize; 3],
        target_mesh: &TriMesh3D,
        t_slots_buf: &wgpu::Buffer,
        t_counts_buf: &wgpu::Buffer,
        slots_readback_buf: &wgpu::Buffer,
        counts_readback_buf: &wgpu::Buffer,
    ) {
        let bvh = GpuMeshBvh::upload(self, target_mesh);
        let device = self.device();
        let queue = self.queue();

        let max_plane_rays = target_out
            .planes
            .iter()
            .map(|p| p.ray_count())
            .max()
            .unwrap_or(1);
        let ov_words = 1 + max_plane_rays.div_ceil(32).max(1);
        let ov_bytes = (ov_words * std::mem::size_of::<u32>()) as u64;

        let mut uniform_bufs = Vec::with_capacity(3);
        let mut common_bgs = Vec::with_capacity(3);
        let mut ov_bufs = Vec::with_capacity(3);
        let mut ov_rbs = Vec::with_capacity(3);
        let mut td_bgs = Vec::with_capacity(3);

        for axis in Axis3::ALL {
            let p = target_out.plane(axis);
            let min_w = axis.get(bvh.bounds_min).min(p.origin_w_mm) - 1.0;
            let max_w = axis
                .get(bvh.bounds_max)
                .max(p.origin_w_mm + (target_out.dim(axis) as f64) * target_out.pitch(axis))
                + 1.0;
            let max_t = (max_w - min_w).max(1.0);
            let uniforms = BvhDispatchUniforms {
                dims: [
                    p.nu as u32,
                    p.nv as u32,
                    axis.index() as u32,
                    plane_ray_offsets[axis.index()] as u32,
                ],
                v0: [
                    p.origin_u_mm as f32,
                    p.origin_v_mm as f32,
                    p.origin_w_mm as f32,
                    min_w as f32,
                ],
                v1: [p.du_mm as f32, p.dv_mm as f32, max_t as f32, 0.0],
                v2: [0.0; 4],
                v3: [0.0; 4],
                v4: [0.0; 4],
                counts: [bvh.node_count, bvh.tri_count, 0, 0],
            };
            let u_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("chip_bvh_td_u"),
                contents: bytemuck::bytes_of(&uniforms),
                usage: wgpu::BufferUsages::UNIFORM,
            });
            let c_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("chip_bvh_common_bg"),
                layout: &self.bvh_common_bgl,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: u_buf.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: bvh.nodes_buf.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: bvh.tris_buf.as_entire_binding(),
                    },
                ],
            });
            let ov_buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_bvh_td_ov"),
                size: ov_bytes,
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_DST
                    | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            });
            let ov_rb = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_bvh_td_ov_rb"),
                size: ov_bytes,
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });
            let td_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("chip_bvh_td_bg"),
                layout: &self.bvh_td_bgl,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: t_slots_buf.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: t_counts_buf.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: ov_buf.as_entire_binding(),
                    },
                ],
            });
            uniform_bufs.push(u_buf);
            common_bgs.push(c_bg);
            ov_bufs.push(ov_buf);
            ov_rbs.push(ov_rb);
            td_bgs.push(td_bg);
        }

        let total_rays: usize = target_out.planes.iter().map(|p| p.ray_count()).sum();
        let total_slots = total_rays * INLINE_DEXEL_SLOTS;
        let slots_bytes = total_slots * std::mem::size_of::<FlatDexelSpan>();
        let counts_bytes = total_rays * std::mem::size_of::<u32>();

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("chip_bvh_td_encoder"),
        });
        for axis in Axis3::ALL {
            let ax = axis.index();
            let p = target_out.plane(axis);
            encoder.clear_buffer(&ov_bufs[ax], 0, None);
            {
                let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                    label: Some("chip_bvh_td_pass"),
                    timestamp_writes: None,
                });
                pass.set_pipeline(&self.bvh_tridexel_pipeline);
                pass.set_bind_group(0, &common_bgs[ax], &[]);
                pass.set_bind_group(1, &td_bgs[ax], &[]);
                pass.dispatch_workgroups(
                    p.nu.div_ceil(GPU_TILE_SIZE) as u32,
                    p.nv.div_ceil(GPU_TILE_SIZE) as u32,
                    1,
                );
            }
            encoder.copy_buffer_to_buffer(&ov_bufs[ax], 0, &ov_rbs[ax], 0, ov_bytes);
        }
        encoder.copy_buffer_to_buffer(t_slots_buf, 0, slots_readback_buf, 0, slots_bytes as u64);
        encoder.copy_buffer_to_buffer(t_counts_buf, 0, counts_readback_buf, 0, counts_bytes as u64);
        queue.submit(std::iter::once(encoder.finish()));

        let mut all_slots = vec![FlatDexelSpan::ZERO; total_slots];
        let mut all_counts = vec![0u32; total_rays];
        let ok_s = self.readback_bytes(
            slots_readback_buf,
            slots_bytes,
            bytemuck::cast_slice_mut(&mut all_slots),
        );
        let ok_c = self.readback_bytes(
            counts_readback_buf,
            counts_bytes,
            bytemuck::cast_slice_mut(&mut all_counts),
        );
        if ok_s && ok_c {
            for axis in Axis3::ALL {
                let ax = axis.index();
                let p = target_out.plane_mut(axis);
                p.overflow.clear();
                let ray_off = plane_ray_offsets[ax];
                let rc = p.ray_count();
                let slot_off = ray_off * INLINE_DEXEL_SLOTS;
                p.slots
                    .copy_from_slice(&all_slots[slot_off..slot_off + rc * INLINE_DEXEL_SLOTS]);
                p.counts.copy_from_slice(&all_counts[ray_off..ray_off + rc]);
            }
        }

        // Exact CPU fallback for any rare target ray with > 4 spans (`ov_bits[0] != 0`)
        let mut ov_bits = vec![0u32; ov_words];
        let mut hits_buf = Vec::new();
        for axis in Axis3::ALL {
            let ax = axis.index();
            if !self.readback_bytes(
                &ov_rbs[ax],
                ov_bytes as usize,
                bytemuck::cast_slice_mut(&mut ov_bits),
            ) || ov_bits[0] == 0
            {
                continue;
            }
            let min_w = axis
                .get(bvh.bounds_min)
                .min(target_out.plane(axis).origin_w_mm)
                - 1.0;
            let max_w = axis.get(bvh.bounds_max).max(
                target_out.plane(axis).origin_w_mm
                    + (target_out.dim(axis) as f64) * target_out.pitch(axis),
            ) + 1.0;
            let max_t = (max_w - min_w).max(1.0);
            let dir = axis.unit_vec();
            let p = target_out.plane_mut(axis);
            let rc = p.ray_count();
            for (word_idx, &bits) in ov_bits[1..].iter().enumerate() {
                let mut w = bits;
                while w != 0 {
                    let tz = w.trailing_zeros();
                    let local_r = (word_idx as u32) * 32 + tz;
                    if (local_r as usize) < rc {
                        let iu = (local_r as usize) % p.nu;
                        let iv = (local_r as usize) / p.nu;
                        let pu = p.center_u(iu) + 1.371_91e-7;
                        let pv = p.center_v(iv) + 2.718_28e-7;
                        let origin = axis.from_uvw(pu, pv, min_w);
                        target_mesh.raycast_all_hits(origin, dir, max_t, &mut hits_buf);
                        let mut clean_w: Vec<f64> = Vec::new();
                        for h in &hits_buf {
                            let wc = min_w + h.t;
                            if clean_w.last().is_none_or(|&prev| (wc - prev).abs() > 1e-6) {
                                clean_w.push(wc);
                            }
                        }
                        let mut spans: Vec<DexelSpan> = Vec::new();
                        for pair in 0..(clean_w.len() / 2) {
                            let a = clean_w[2 * pair];
                            let b = clean_w[2 * pair + 1];
                            if b - a > 1e-6 {
                                spans.push(DexelSpan::new(a, b, UNCUT_OP_IDX, UNCUT_OP_IDX));
                            }
                        }
                        p.set_ray_spans(iu, iv, &spans);
                    }
                    w &= w - 1;
                }
            }
        }
        let _ = uniform_bufs;
    }

    /// Rasterizes a WCS-space [`TriMesh3D`] target solid into `stock.r_target` via GPU BVH inward
    /// radial raycasts (`bvh_distance.wgsl`).
    pub fn rasterize_cylindrical_target_mesh(
        &self,
        stock: &mut CylindricalStock,
        target_mesh: &TriMesh3D,
    ) {
        stock.r_target.fill(-1.0_f32);
        if target_mesh.is_empty() || stock.r_target.is_empty() {
            return;
        }
        let device = self.device();
        let byte_len = stock.r_target.len() * std::mem::size_of::<f32>();
        let out_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_bvh_cyl_r_target"),
            size: (byte_len as u64).max(16),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let rb_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_bvh_cyl_r_target_rb"),
            size: (byte_len as u64).max(16),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        self.rasterize_cylindrical_target_into_buffer(
            stock.nu,
            stock.ntheta,
            stock.rotary,
            stock.u_min,
            stock.du,
            stock.dtheta,
            target_mesh,
            &out_buf,
        );

        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("chip_bvh_cyl_rb_enc"),
        });
        enc.copy_buffer_to_buffer(&out_buf, 0, &rb_buf, 0, byte_len as u64);
        self.queue().submit(std::iter::once(enc.finish()));
        let _ = self.readback_bytes(
            &rb_buf,
            byte_len,
            bytemuck::cast_slice_mut(&mut stock.r_target),
        );
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn rasterize_cylindrical_target_into_buffer(
        &self,
        nu: usize,
        ntheta: usize,
        rotary: RotaryStockSpec,
        u_min: f64,
        du: f64,
        dtheta: f64,
        target_mesh: &TriMesh3D,
        r_target_buf: &wgpu::Buffer,
    ) {
        let bvh = GpuMeshBvh::upload(self, target_mesh);
        let (e1, e2, u_vec) = rotary.basis();
        let u_center = rotary.center_wcs.dot(u_vec);
        let r_cast = rotary.stock_outer_radius_mm + 5.0;

        let uniforms = BvhDispatchUniforms {
            dims: [nu as u32, ntheta as u32, 0, 0],
            v0: [
                rotary.center_wcs.x as f32,
                rotary.center_wcs.y as f32,
                rotary.center_wcs.z as f32,
                r_cast as f32,
            ],
            v1: [e1.x as f32, e1.y as f32, e1.z as f32, u_min as f32],
            v2: [e2.x as f32, e2.y as f32, e2.z as f32, du as f32],
            v3: [
                u_vec.x as f32,
                u_vec.y as f32,
                u_vec.z as f32,
                dtheta as f32,
            ],
            v4: [u_center as f32, rotary.direction_sign as f32, 0.0, 0.0],
            counts: [bvh.node_count, bvh.tri_count, 0, 0],
        };

        let device = self.device();
        let u_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_bvh_cyl_u"),
            contents: bytemuck::bytes_of(&uniforms),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let common_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chip_bvh_cyl_common_bg"),
            layout: &self.bvh_common_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: u_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: bvh.nodes_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: bvh.tris_buf.as_entire_binding(),
                },
            ],
        });
        let cyl_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chip_bvh_cyl_out_bg"),
            layout: &self.bvh_cyl_bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 3,
                resource: r_target_buf.as_entire_binding(),
            }],
        });

        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("chip_bvh_cyl_enc"),
        });
        {
            let mut pass = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("chip_bvh_cyl_pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.bvh_cyl_pipeline);
            pass.set_bind_group(0, &common_bg, &[]);
            pass.set_bind_group(1, &cyl_bg, &[]);
            pass.dispatch_workgroups(
                ntheta.div_ceil(GPU_TILE_SIZE) as u32,
                nu.div_ceil(GPU_TILE_SIZE) as u32,
                1,
            );
        }
        self.queue().submit(std::iter::once(enc.finish()));
    }

    /// Rasterizes a WCS-space [`TriMesh3D`] target solid into `stock.z_target` via GPU BVH top-down
    /// `-Z` raycasts (`bvh_distance.wgsl`).
    pub fn rasterize_heightfield_target_mesh(
        &self,
        stock: &mut HeightfieldStock,
        target_mesh: &TriMesh3D,
    ) {
        stock.z_target.fill(f32::NEG_INFINITY);
        if target_mesh.is_empty() || stock.z_target.is_empty() {
            return;
        }
        let device = self.device();
        let byte_len = stock.z_target.len() * std::mem::size_of::<f32>();
        let out_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_bvh_hf_z_target"),
            size: (byte_len as u64).max(16),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let rb_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_bvh_hf_z_target_rb"),
            size: (byte_len as u64).max(16),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        self.rasterize_heightfield_target_into_buffer(
            stock.nx,
            stock.ny,
            stock.origin_wcs,
            stock.dx_mm,
            stock.dy_mm,
            target_mesh,
            &out_buf,
        );

        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("chip_bvh_hf_rb_enc"),
        });
        enc.copy_buffer_to_buffer(&out_buf, 0, &rb_buf, 0, byte_len as u64);
        self.queue().submit(std::iter::once(enc.finish()));
        let _ = self.readback_bytes(
            &rb_buf,
            byte_len,
            bytemuck::cast_slice_mut(&mut stock.z_target),
        );
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn rasterize_heightfield_target_into_buffer(
        &self,
        nx: usize,
        ny: usize,
        origin_wcs: DVec3,
        dx_mm: f64,
        dy_mm: f64,
        target_mesh: &TriMesh3D,
        z_target_buf: &wgpu::Buffer,
    ) {
        let bvh = GpuMeshBvh::upload(self, target_mesh);
        let z_top_cast = bvh.bounds_max.z.max(origin_wcs.z) + 5.0;
        let z_bot = bvh.bounds_min.z.min(origin_wcs.z) - 5.0;
        let max_t = (z_top_cast - z_bot).max(1.0);

        let uniforms = BvhDispatchUniforms {
            dims: [nx as u32, ny as u32, 0, 1], // require_upward = 1
            v0: [
                origin_wcs.x as f32,
                origin_wcs.y as f32,
                origin_wcs.z as f32,
                z_top_cast as f32,
            ],
            v1: [dx_mm as f32, dy_mm as f32, max_t as f32, 0.0],
            v2: [0.0; 4],
            v3: [0.0; 4],
            v4: [0.0; 4],
            counts: [bvh.node_count, bvh.tri_count, 0, 0],
        };

        let device = self.device();
        let u_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_bvh_hf_u"),
            contents: bytemuck::bytes_of(&uniforms),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let common_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chip_bvh_hf_common_bg"),
            layout: &self.bvh_common_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: u_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: bvh.nodes_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: bvh.tris_buf.as_entire_binding(),
                },
            ],
        });
        let hf_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chip_bvh_hf_out_bg"),
            layout: &self.bvh_hf_bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 4,
                resource: z_target_buf.as_entire_binding(),
            }],
        });

        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("chip_bvh_hf_enc"),
        });
        {
            let mut pass = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("chip_bvh_hf_pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.bvh_hf_pipeline);
            pass.set_bind_group(0, &common_bg, &[]);
            pass.set_bind_group(1, &hf_bg, &[]);
            pass.dispatch_workgroups(
                nx.div_ceil(GPU_TILE_SIZE) as u32,
                ny.div_ceil(GPU_TILE_SIZE) as u32,
                1,
            );
        }
        self.queue().submit(std::iter::once(enc.finish()));
    }

    /// Evaluates exact point-to-mesh closest point & Euclidean distance (`mm`) on GPU for a batch
    /// of 3D WCS query points via nearest-child-first BVH traversal (`bvh_distance.wgsl`).
    pub fn query_mesh_closest_points(
        &self,
        target_mesh: &TriMesh3D,
        points: &[DVec3],
    ) -> Vec<Option<(DVec3, f64)>> {
        if points.is_empty() {
            return Vec::new();
        }
        if target_mesh.is_empty() {
            return vec![None; points.len()];
        }

        let bvh = GpuMeshBvh::upload(self, target_mesh);
        let device = self.device();
        let q_packed: Vec<[f32; 4]> = points
            .iter()
            .map(|p| [p.x as f32, p.y as f32, p.z as f32, 0.0])
            .collect();

        let uniforms = BvhDispatchUniforms {
            dims: [0, 0, points.len() as u32, 0],
            v0: [0.0; 4],
            v1: [0.0; 4],
            v2: [0.0; 4],
            v3: [0.0; 4],
            v4: [0.0; 4],
            counts: [bvh.node_count, bvh.tri_count, 0, 0],
        };
        let u_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_bvh_closest_u"),
            contents: bytemuck::bytes_of(&uniforms),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let q_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("chip_bvh_closest_q"),
            contents: bytemuck::cast_slice(&q_packed),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let out_bytes = points.len() * std::mem::size_of::<[f32; 4]>();
        let out_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_bvh_closest_out"),
            size: (out_bytes as u64).max(16),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let rb_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_bvh_closest_rb"),
            size: (out_bytes as u64).max(16),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let common_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chip_bvh_closest_common_bg"),
            layout: &self.bvh_common_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: u_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: bvh.nodes_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: bvh.tris_buf.as_entire_binding(),
                },
            ],
        });
        let io_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chip_bvh_closest_io_bg"),
            layout: &self.bvh_closest_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: q_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 6,
                    resource: out_buf.as_entire_binding(),
                },
            ],
        });

        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("chip_bvh_closest_enc"),
        });
        {
            let mut pass = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("chip_bvh_closest_pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.bvh_closest_pipeline);
            pass.set_bind_group(0, &common_bg, &[]);
            pass.set_bind_group(1, &io_bg, &[]);
            pass.dispatch_workgroups(points.len().div_ceil(64) as u32, 1, 1);
        }
        enc.copy_buffer_to_buffer(&out_buf, 0, &rb_buf, 0, out_bytes as u64);
        self.queue().submit(std::iter::once(enc.finish()));

        let mut raw_out = vec![[0.0_f32; 4]; points.len()];
        if !self.readback_bytes(&rb_buf, out_bytes, bytemuck::cast_slice_mut(&mut raw_out)) {
            return vec![None; points.len()];
        }
        raw_out
            .into_iter()
            .map(|r| {
                if r[3] >= 0.0 && r[3].is_finite() {
                    Some((
                        DVec3::new(f64::from(r[0]), f64::from(r[1]), f64::from(r[2])),
                        f64::from(r[3]),
                    ))
                } else {
                    None
                }
            })
            .collect()
    }
}
