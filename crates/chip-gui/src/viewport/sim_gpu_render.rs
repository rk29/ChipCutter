//! Zero-readback direct GPU stock render pipelines (`Heightfield`, `Cylindrical`, and `TriDexel`
//! indirect draw) and instant `< 0.1 ms` `SimStockVisualUniforms` heatmap/palette updates.

use std::sync::Arc;

use bytemuck::{Pod, Zeroable};
use chip_sim::{
    GpuCylindricalRenderData, GpuHeightfieldRenderData, GpuStockRenderHandle,
    GpuTriDexelRenderData, MACHINED_METAL_COLOR, OPERATION_STOCK_PALETTE, RAW_STOCK_SIDE_COLOR,
    RAW_STOCK_TOP_COLOR, SimMeshOptions, SimStockColorMode,
};

use super::gpu_types::{COLOR_FORMAT, DEPTH_FORMAT, GrowableSimIndexedMeshGpu, SAMPLE_COUNT};

/// Shared 336-byte (`21 × 16B`) visual styling uniform buffer updated via a single
/// `queue.write_buffer` whenever `color_mode` (`ByOperation` / `Metallic` / `ErrorHeatmap`),
/// `mesh_skinning`, or `tolerance_mm` changes—zero stock re-sweep and zero CPU re-meshing.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct SimStockVisualUniformsGpu {
    /// `[color_mode_id (0=Metallic, 1=ByOperation, 2=ErrorHeatmap), has_target, mesh_skinning (0/1), 0]`
    pub mode_and_flags: [u32; 4],
    /// `[tolerance_mm, max_scale_mm, 0.0, 0.0]`
    pub heatmap_params: [f32; 4],
    pub raw_top_color: [f32; 4],
    pub raw_side_color: [f32; 4],
    pub machined_metal_color: [f32; 4],
    pub op_palette: [[f32; 4]; 16],
}

impl SimStockVisualUniformsGpu {
    #[allow(dead_code)]
    pub(crate) fn from_color_mode(color_mode: SimStockColorMode, has_target: bool) -> Self {
        Self::from_options(SimMeshOptions::from(color_mode), has_target)
    }

    pub(crate) fn from_options(options: SimMeshOptions, has_target: bool) -> Self {
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
            mode_and_flags: [
                mode_id,
                u32::from(has_target),
                u32::from(options.mesh_skinning),
                0,
            ],
            heatmap_params: [tol_mm, max_scale_mm, 0.0, 0.0],
            raw_top_color: RAW_STOCK_TOP_COLOR,
            raw_side_color: RAW_STOCK_SIDE_COLOR,
            machined_metal_color: MACHINED_METAL_COLOR,
            op_palette,
        }
    }
}

/// 64-byte (`4 × 16B`) heightfield grid geometry uniform bound at `@group(1) @binding(1)`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct GpuHeightfieldRenderUniformsGpu {
    /// `[nx, ny, vx, vy]`
    pub dims: [u32; 4],
    /// `[stride, 0, 0, 0]`
    pub meta: [u32; 4],
    /// `[origin_x, origin_y, origin_z, 0.0]`
    pub origin_wcs: [f32; 4],
    /// `[vdx, vdy, 0.0, 0.0]`
    pub pitch_mm: [f32; 4],
}

/// 112-byte (`7 × 16B`) cylindrical grid geometry uniform bound at `@group(1) @binding(6)`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct GpuCylindricalRenderUniformsGpu {
    /// `[e1.x, e1.y, e1.z, u_min]`
    pub e1_and_u_min: [f32; 4],
    /// `[e2_signed.x, e2_signed.y, e2_signed.z, du]`
    pub e2_signed_and_du: [f32; 4],
    /// `[u_vec.x, u_vec.y, u_vec.z, dtheta]`
    pub u_vec_and_dtheta: [f32; 4],
    /// `[center_wcs.x, center_wcs.y, center_wcs.z, u_max]`
    pub center_and_u_max: [f32; 4],
    /// `[nu, ntheta, su, st]`
    pub dims: [u32; 4],
    /// `[step_u, step_t, shell_verts, 0]`
    pub steps_and_counts: [u32; 4],
    /// `[u_center, 0.0, 0.0, 0.0]`
    pub extra: [f32; 4],
}

/// 64-byte (`4 × 16B`) Tri-Dexel grid geometry uniform bound at `@group(1) @binding(10)`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct GpuTriDexelRenderUniformsGpu {
    /// `[nx, ny, nz, total_rays]`
    pub dims_and_total: [u32; 4],
    /// `[ray_off_x, ray_off_y, ray_off_z, 0]`
    pub plane_offsets: [u32; 4],
    /// `[origin_x, origin_y, origin_z, 0.0]`
    pub origin_wcs: [f32; 4],
    /// `[dx, dy, dz, 0.0]`
    pub pitch_mm: [f32; 4],
}

enum ActiveGpuDraw {
    Heightfield {
        bind_group: wgpu::BindGroup,
        vertex_count: u32,
        /// Cached buffer identity to avoid recreating BindGroups on idle/recolor frames.
        z_stock_buf: Arc<wgpu::Buffer>,
        last_op_buf: Arc<wgpu::Buffer>,
        z_target_buf: Arc<wgpu::Buffer>,
    },
    Cylindrical {
        bind_group: wgpu::BindGroup,
        vertex_count: u32,
        r_stock_buf: Arc<wgpu::Buffer>,
        last_op_buf: Arc<wgpu::Buffer>,
        r_target_buf: Arc<wgpu::Buffer>,
    },
    TriDexel {
        bind_group: wgpu::BindGroup,
        quad_buf: Arc<wgpu::Buffer>,
        indirect_args_buf: Arc<wgpu::Buffer>,
        has_overflow_mesh: bool,
    },
}

/// Owns the direct GPU stock render pipelines (`vs_sim_heightfield_gpu`, `vs_sim_cylindrical_gpu`,
/// `vs_sim_tridexel_gpu`) and reusable `SimStockVisualUniforms` / bind-group cache for
/// `ViewportRenderer`.
pub(crate) struct SimGpuStockRenderer {
    visual_uniform_buf: wgpu::Buffer,
    hf_uniform_buf: wgpu::Buffer,
    cyl_uniform_buf: wgpu::Buffer,
    td_uniform_buf: wgpu::Buffer,
    hf_bgl: wgpu::BindGroupLayout,
    cyl_bgl: wgpu::BindGroupLayout,
    td_bgl: wgpu::BindGroupLayout,
    hf_pipeline: wgpu::RenderPipeline,
    cyl_pipeline: wgpu::RenderPipeline,
    td_pipeline: wgpu::RenderPipeline,
    active_draw: Option<ActiveGpuDraw>,
    overflow_mesh_gpu: GrowableSimIndexedMeshGpu,
    overflow_rev: u64,
}

impl SimGpuStockRenderer {
    pub(crate) fn new(
        device: &wgpu::Device,
        shader: &wgpu::ShaderModule,
        vp_bgl: &wgpu::BindGroupLayout,
    ) -> Self {
        let visual_uniform_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_sim_gpu_vis_uniforms"),
            size: std::mem::size_of::<SimStockVisualUniformsGpu>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let hf_uniform_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_sim_gpu_hf_uniforms"),
            size: std::mem::size_of::<GpuHeightfieldRenderUniformsGpu>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let cyl_uniform_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_sim_gpu_cyl_uniforms"),
            size: std::mem::size_of::<GpuCylindricalRenderUniformsGpu>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let td_uniform_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_sim_gpu_td_uniforms"),
            size: std::mem::size_of::<GpuTriDexelRenderUniformsGpu>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let ro_storage =
            |binding: u32, visibility: wgpu::ShaderStages| wgpu::BindGroupLayoutEntry {
                binding,
                visibility,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            };
        let uniform = |binding: u32, visibility: wgpu::ShaderStages| wgpu::BindGroupLayoutEntry {
            binding,
            visibility,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        };

        // Heightfield direct GPU BGL: 2 uniforms + 4 read-only storage buffers (<= 8 total)
        let hf_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("chip_sim_gpu_hf_bgl"),
            entries: &[
                uniform(0, wgpu::ShaderStages::VERTEX_FRAGMENT),
                uniform(1, wgpu::ShaderStages::VERTEX_FRAGMENT),
                ro_storage(2, wgpu::ShaderStages::VERTEX_FRAGMENT),
                ro_storage(3, wgpu::ShaderStages::VERTEX_FRAGMENT),
                ro_storage(4, wgpu::ShaderStages::VERTEX_FRAGMENT),
                ro_storage(5, wgpu::ShaderStages::VERTEX_FRAGMENT),
            ],
        });

        // Cylindrical direct GPU BGL: 2 uniforms + 3 read-only storage buffers (<= 8 total)
        let cyl_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("chip_sim_gpu_cyl_bgl"),
            entries: &[
                uniform(0, wgpu::ShaderStages::VERTEX_FRAGMENT),
                uniform(6, wgpu::ShaderStages::VERTEX),
                ro_storage(7, wgpu::ShaderStages::VERTEX),
                ro_storage(8, wgpu::ShaderStages::VERTEX),
                ro_storage(9, wgpu::ShaderStages::VERTEX),
            ],
        });

        // Tri-Dexel compact-quad direct GPU BGL: 1 uniform + 1 read-only storage buffer
        let td_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("chip_sim_gpu_td_bgl"),
            entries: &[
                uniform(10, wgpu::ShaderStages::VERTEX),
                ro_storage(11, wgpu::ShaderStages::VERTEX),
            ],
        });

        let multisample = wgpu::MultisampleState {
            count: SAMPLE_COUNT,
            mask: !0,
            alpha_to_coverage_enabled: false,
        };
        let depth_stencil = Some(wgpu::DepthStencilState {
            format: DEPTH_FORMAT,
            depth_write_enabled: Some(true),
            depth_compare: Some(wgpu::CompareFunction::LessEqual),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        });

        let hf_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("chip_sim_gpu_hf_layout"),
            bind_group_layouts: &[Some(vp_bgl), Some(&hf_bgl)],
            immediate_size: 0,
        });
        let hf_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("chip_sim_gpu_hf_pipeline"),
            layout: Some(&hf_layout),
            vertex: wgpu::VertexState {
                module: shader,
                entry_point: Some("vs_sim_heightfield_gpu"),
                buffers: &[],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: depth_stencil.clone(),
            multisample,
            fragment: Some(wgpu::FragmentState {
                module: shader,
                entry_point: Some("fs_sim_gpu_stock"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: COLOR_FORMAT,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview_mask: None,
            cache: None,
        });

        let cyl_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("chip_sim_gpu_cyl_layout"),
            bind_group_layouts: &[Some(vp_bgl), Some(&cyl_bgl)],
            immediate_size: 0,
        });
        let cyl_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("chip_sim_gpu_cyl_pipeline"),
            layout: Some(&cyl_layout),
            vertex: wgpu::VertexState {
                module: shader,
                entry_point: Some("vs_sim_cylindrical_gpu"),
                buffers: &[],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: depth_stencil.clone(),
            multisample,
            fragment: Some(wgpu::FragmentState {
                module: shader,
                entry_point: Some("fs_sim_mesh"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: COLOR_FORMAT,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview_mask: None,
            cache: None,
        });

        let td_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("chip_sim_gpu_td_layout"),
            bind_group_layouts: &[Some(vp_bgl), Some(&td_bgl)],
            immediate_size: 0,
        });
        let td_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("chip_sim_gpu_td_pipeline"),
            layout: Some(&td_layout),
            vertex: wgpu::VertexState {
                module: shader,
                entry_point: Some("vs_sim_tridexel_gpu"),
                buffers: &[],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil,
            multisample,
            fragment: Some(wgpu::FragmentState {
                module: shader,
                entry_point: Some("fs_sim_mesh"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: COLOR_FORMAT,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview_mask: None,
            cache: None,
        });

        Self {
            visual_uniform_buf,
            hf_uniform_buf,
            cyl_uniform_buf,
            td_uniform_buf,
            hf_bgl,
            cyl_bgl,
            td_bgl,
            hf_pipeline,
            cyl_pipeline,
            td_pipeline,
            active_draw: None,
            overflow_mesh_gpu: GrowableSimIndexedMeshGpu::new(
                "chip_td_overflow_vbuf",
                "chip_td_overflow_ibuf",
            ),
            overflow_rev: 0,
        }
    }

    pub(crate) fn clear(&mut self) {
        self.active_draw = None;
        self.overflow_mesh_gpu.clear();
    }

    pub(crate) fn prepare_with_options(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        handle: &GpuStockRenderHandle,
        options: SimMeshOptions,
    ) {
        let vis = SimStockVisualUniformsGpu::from_options(options, handle.has_target());
        queue.write_buffer(&self.visual_uniform_buf, 0, bytemuck::bytes_of(&vis));

        match handle {
            GpuStockRenderHandle::Heightfield(hf) => self.prepare_heightfield(device, queue, hf),
            GpuStockRenderHandle::Cylindrical(cyl) => self.prepare_cylindrical(device, queue, cyl),
            GpuStockRenderHandle::TriDexel(td) => {
                self.prepare_tridexel(device, queue, encoder, td, options);
            }
        }
    }

    fn prepare_heightfield(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        hf: &GpuHeightfieldRenderData,
    ) {
        const MAX_VISUAL_DIM: u32 = 512;
        let stride = hf.nx.max(hf.ny).div_ceil(MAX_VISUAL_DIM).max(1);
        let vx = hf.nx / stride;
        let vy = hf.ny / stride;
        if vx == 0 || vy == 0 {
            self.active_draw = None;
            return;
        }
        let vdx = (hf.nx as f32 * hf.dx_mm) / (vx as f32);
        let vdy = (hf.ny as f32 * hf.dy_mm) / (vy as f32);
        let uniforms = GpuHeightfieldRenderUniformsGpu {
            dims: [hf.nx, hf.ny, vx, vy],
            meta: [stride, 0, 0, 0],
            origin_wcs: [hf.origin_wcs[0], hf.origin_wcs[1], hf.origin_wcs[2], 0.0],
            pitch_mm: [vdx, vdy, 0.0, 0.0],
        };
        queue.write_buffer(&self.hf_uniform_buf, 0, bytemuck::bytes_of(&uniforms));

        let vertex_count = vx.saturating_mul(vy).saturating_mul(36);
        let reuse_bg = matches!(
            &self.active_draw,
            Some(ActiveGpuDraw::Heightfield {
                z_stock_buf,
                last_op_buf,
                z_target_buf,
                ..
            }) if Arc::ptr_eq(z_stock_buf, &hf.z_stock_buf)
                && Arc::ptr_eq(last_op_buf, &hf.last_op_buf)
                && Arc::ptr_eq(z_target_buf, &hf.z_target_buf)
        );
        if reuse_bg {
            if let Some(ActiveGpuDraw::Heightfield {
                vertex_count: vc, ..
            }) = &mut self.active_draw
            {
                *vc = vertex_count;
            }
            return;
        }

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chip_sim_gpu_hf_bg"),
            layout: &self.hf_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: self.visual_uniform_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: self.hf_uniform_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: hf.z_stock_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: hf.z_bottom_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: hf.z_target_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: hf.last_op_buf.as_entire_binding(),
                },
            ],
        });
        self.active_draw = Some(ActiveGpuDraw::Heightfield {
            bind_group,
            vertex_count,
            z_stock_buf: Arc::clone(&hf.z_stock_buf),
            last_op_buf: Arc::clone(&hf.last_op_buf),
            z_target_buf: Arc::clone(&hf.z_target_buf),
        });
    }

    fn prepare_cylindrical(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        cyl: &GpuCylindricalRenderData,
    ) {
        let nu = cyl.nu;
        let nt = cyl.ntheta;
        if nu < 2 || nt < 3 {
            self.active_draw = None;
            return;
        }
        let step_u = (nu / 180).max(1);
        let step_t = (nt / 180).max(1);
        let su = nu.div_ceil(step_u);
        let st = nt.div_ceil(step_t);
        let shell_verts = (su - 1).saturating_mul(st).saturating_mul(6);
        let cap_verts = st.saturating_mul(6);
        let vertex_count = shell_verts.saturating_add(cap_verts);

        let (e1, e2, u_vec) = cyl.rotary.basis();
        let e2_signed = e2 * cyl.rotary.direction_sign;
        let u_center = cyl.rotary.center_wcs.dot(u_vec);

        let uniforms = GpuCylindricalRenderUniformsGpu {
            e1_and_u_min: [e1.x as f32, e1.y as f32, e1.z as f32, cyl.u_min as f32],
            e2_signed_and_du: [
                e2_signed.x as f32,
                e2_signed.y as f32,
                e2_signed.z as f32,
                cyl.du as f32,
            ],
            u_vec_and_dtheta: [
                u_vec.x as f32,
                u_vec.y as f32,
                u_vec.z as f32,
                cyl.dtheta as f32,
            ],
            center_and_u_max: [
                cyl.rotary.center_wcs.x as f32,
                cyl.rotary.center_wcs.y as f32,
                cyl.rotary.center_wcs.z as f32,
                cyl.u_max as f32,
            ],
            dims: [nu, nt, su, st],
            steps_and_counts: [step_u, step_t, shell_verts, 0],
            extra: [u_center as f32, 0.0, 0.0, 0.0],
        };
        queue.write_buffer(&self.cyl_uniform_buf, 0, bytemuck::bytes_of(&uniforms));

        let reuse_bg = matches!(
            &self.active_draw,
            Some(ActiveGpuDraw::Cylindrical {
                r_stock_buf,
                last_op_buf,
                r_target_buf,
                ..
            }) if Arc::ptr_eq(r_stock_buf, &cyl.r_stock_buf)
                && Arc::ptr_eq(last_op_buf, &cyl.last_op_buf)
                && Arc::ptr_eq(r_target_buf, &cyl.r_target_buf)
        );
        if reuse_bg {
            if let Some(ActiveGpuDraw::Cylindrical {
                vertex_count: vc, ..
            }) = &mut self.active_draw
            {
                *vc = vertex_count;
            }
            return;
        }

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chip_sim_gpu_cyl_bg"),
            layout: &self.cyl_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: self.visual_uniform_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 6,
                    resource: self.cyl_uniform_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 7,
                    resource: cyl.r_stock_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 8,
                    resource: cyl.r_target_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 9,
                    resource: cyl.last_op_buf.as_entire_binding(),
                },
            ],
        });
        self.active_draw = Some(ActiveGpuDraw::Cylindrical {
            bind_group,
            vertex_count,
            r_stock_buf: Arc::clone(&cyl.r_stock_buf),
            last_op_buf: Arc::clone(&cyl.last_op_buf),
            r_target_buf: Arc::clone(&cyl.r_target_buf),
        });
    }

    fn prepare_tridexel(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        td: &GpuTriDexelRenderData,
        options: SimMeshOptions,
    ) {
        // Zero-op if options already baked; < 0.5 ms GPU compute re-emit if color_mode or mesh_skinning changed!
        let _ = td.ensure_emitted_for_options(options, encoder);

        let uniforms = GpuTriDexelRenderUniformsGpu {
            dims_and_total: [td.dims[0], td.dims[1], td.dims[2], td.total_rays],
            plane_offsets: [
                td.plane_ray_offsets[0],
                td.plane_ray_offsets[1],
                td.plane_ray_offsets[2],
                0,
            ],
            origin_wcs: [td.origin_wcs[0], td.origin_wcs[1], td.origin_wcs[2], 0.0],
            pitch_mm: [td.pitch_mm[0], td.pitch_mm[1], td.pitch_mm[2], 0.0],
        };
        queue.write_buffer(&self.td_uniform_buf, 0, bytemuck::bytes_of(&uniforms));

        let has_overflow_mesh = if let Some(ov) = &td.overflow_mesh
            && !ov.vertices.is_empty()
            && !ov.indices.is_empty()
        {
            self.overflow_rev = self.overflow_rev.wrapping_add(1);
            self.overflow_mesh_gpu.sync(
                device,
                queue,
                Some((self.overflow_rev, &ov.vertices, &ov.indices)),
            );
            true
        } else {
            self.overflow_mesh_gpu.clear();
            false
        };

        let reuse_bg = matches!(
            &self.active_draw,
            Some(ActiveGpuDraw::TriDexel {
                quad_buf,
                indirect_args_buf,
                ..
            }) if Arc::ptr_eq(quad_buf, &td.quad_buf)
                && Arc::ptr_eq(indirect_args_buf, &td.indirect_args_buf)
        );
        if reuse_bg {
            if let Some(ActiveGpuDraw::TriDexel {
                has_overflow_mesh: ov_flag,
                ..
            }) = &mut self.active_draw
            {
                *ov_flag = has_overflow_mesh;
            }
            return;
        }

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chip_sim_gpu_td_bg"),
            layout: &self.td_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 10,
                    resource: self.td_uniform_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 11,
                    resource: td.quad_buf.as_entire_binding(),
                },
            ],
        });

        self.active_draw = Some(ActiveGpuDraw::TriDexel {
            bind_group,
            quad_buf: Arc::clone(&td.quad_buf),
            indirect_args_buf: Arc::clone(&td.indirect_args_buf),
            has_overflow_mesh,
        });
    }

    /// Records the direct GPU stock draw commands into `pass`. Returns `true` if GPU stock was drawn.
    pub(crate) fn draw<'a>(
        &'a self,
        pass: &mut wgpu::RenderPass<'a>,
        sim_stock_pipeline: &'a wgpu::RenderPipeline,
    ) -> bool {
        let Some(draw) = &self.active_draw else {
            return false;
        };
        match draw {
            ActiveGpuDraw::Heightfield {
                bind_group,
                vertex_count,
                ..
            } => {
                if *vertex_count == 0 {
                    return false;
                }
                pass.set_pipeline(&self.hf_pipeline);
                pass.set_bind_group(1, bind_group, &[]);
                pass.draw(0..*vertex_count, 0..1);
                true
            }
            ActiveGpuDraw::Cylindrical {
                bind_group,
                vertex_count,
                ..
            } => {
                if *vertex_count == 0 {
                    return false;
                }
                pass.set_pipeline(&self.cyl_pipeline);
                pass.set_bind_group(1, bind_group, &[]);
                pass.draw(0..*vertex_count, 0..1);
                true
            }
            ActiveGpuDraw::TriDexel {
                bind_group,
                indirect_args_buf,
                has_overflow_mesh,
                ..
            } => {
                pass.set_pipeline(&self.td_pipeline);
                pass.set_bind_group(1, bind_group, &[]);
                pass.draw_indirect(indirect_args_buf, 0);
                if *has_overflow_mesh
                    && let Some((vbuf, ibuf, icount)) = self.overflow_mesh_gpu.active()
                {
                    pass.set_pipeline(sim_stock_pipeline);
                    pass.set_vertex_buffer(0, vbuf.slice(..));
                    pass.set_index_buffer(ibuf.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..icount, 0, 0..1);
                }
                true
            }
        }
    }
}
