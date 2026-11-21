use chip_cad::CadDocument;
use chip_math::DisplayUnit;
use wgpu::util::DeviceExt as _;

use super::{
    gpu_types::{
        COLOR_FORMAT, DEPTH_FORMAT, EdgeInstanceGpu, FrameTargets, GrowableGpuBuffer,
        OverlayVertexGpu, SAMPLE_COUNT, SimVertexGpu, SolidVertexGpu, UploadedCadGpu,
        UploadedSimIndexedMeshGpu, UploadedToolpathsGpu, ViewportUniformsGpu,
        upload_sim_indexed_mesh,
    },
    scene_builders::{
        HighlightState, adaptive_grid_steps, build_edge_and_scene_instances,
        build_toolpath_gpu_instances, compute_toolpath_gpu_signature, dmat4_to_f32_2d,
    },
};
use crate::camera::CadCamera;

/// 4x MSAA `wgpu` 3D CAD/CAM viewport renderer shared by `egui` and headless tests.
pub struct ViewportRenderer {
    bgl: wgpu::BindGroupLayout,
    uniform_buf: wgpu::Buffer,
    face_state_buf: wgpu::Buffer,
    face_state_capacity: usize,
    bind_group: wgpu::BindGroup,
    bg_pipeline: wgpu::RenderPipeline,
    solid_pipeline: wgpu::RenderPipeline,
    solid_ghost_pipeline: wgpu::RenderPipeline,
    overlay_pipeline: wgpu::RenderPipeline,
    edge_pipeline: wgpu::RenderPipeline,
    sim_stock_pipeline: wgpu::RenderPipeline,
    sim_tool_pipeline: wgpu::RenderPipeline,
    targets: Option<FrameTargets>,
    egui_tex_id: Option<egui::TextureId>,
    uploaded_cad: Option<UploadedCadGpu>,
    uploaded_toolpaths: Option<UploadedToolpathsGpu>,
    uploaded_sim_stock: Option<UploadedSimIndexedMeshGpu>,
    uploaded_sim_tool: Option<UploadedSimIndexedMeshGpu>,
    overlay_buf: GrowableGpuBuffer,
    edge_buf: GrowableGpuBuffer,
}

impl ViewportRenderer {
    pub fn new(device: &wgpu::Device) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("chip_viewport_shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders.wgsl").into()),
        });

        let uniform_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_vp_uniforms"),
            size: std::mem::size_of::<ViewportUniformsGpu>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let face_state_capacity = 256usize;
        let face_state_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("chip_vp_face_states"),
            size: (face_state_capacity * std::mem::size_of::<u32>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("chip_vp_bgl"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });

        let bind_group = create_vp_bind_group(device, &bgl, &uniform_buf, &face_state_buf);

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("chip_vp_layout"),
            bind_group_layouts: &[Some(&bgl)],
            immediate_size: 0,
        });

        let multisample = wgpu::MultisampleState {
            count: SAMPLE_COUNT,
            mask: !0,
            alpha_to_coverage_enabled: false,
        };

        let bg_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("chip_bg_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_background"),
                buffers: &[],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::Always),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample,
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_background"),
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

        let solid_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("chip_solid_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_solid"),
                buffers: &[Some(SolidVertexGpu::layout())],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample,
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_solid"),
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

        let solid_ghost_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("chip_solid_ghost_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_solid"),
                buffers: &[Some(SolidVertexGpu::layout())],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(false),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample,
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_solid"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: COLOR_FORMAT,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview_mask: None,
            cache: None,
        });

        let overlay_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("chip_overlay_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_overlay"),
                buffers: &[Some(OverlayVertexGpu::layout())],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(false),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample,
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_overlay"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: COLOR_FORMAT,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview_mask: None,
            cache: None,
        });

        let edge_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("chip_edge_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_edge"),
                buffers: &[Some(EdgeInstanceGpu::layout())],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(false),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample,
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_edge"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: COLOR_FORMAT,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview_mask: None,
            cache: None,
        });

        let sim_stock_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("chip_sim_stock_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_sim_stock"),
                buffers: &[Some(SimVertexGpu::layout())],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample,
            fragment: Some(wgpu::FragmentState {
                module: &shader,
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

        let sim_tool_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("chip_sim_tool_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_sim_tool"),
                buffers: &[Some(SimVertexGpu::layout())],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample,
            fragment: Some(wgpu::FragmentState {
                module: &shader,
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
            bgl,
            uniform_buf,
            face_state_buf,
            face_state_capacity,
            bind_group,
            bg_pipeline,
            solid_pipeline,
            solid_ghost_pipeline,
            overlay_pipeline,
            edge_pipeline,
            sim_stock_pipeline,
            sim_tool_pipeline,
            targets: None,
            egui_tex_id: None,
            uploaded_cad: None,
            uploaded_toolpaths: None,
            uploaded_sim_stock: None,
            uploaded_sim_tool: None,
            overlay_buf: GrowableGpuBuffer::new("chip_overlay_tris", wgpu::BufferUsages::VERTEX),
            edge_buf: GrowableGpuBuffer::new("chip_edge_instances", wgpu::BufferUsages::VERTEX),
        }
    }

    pub fn render_for_egui(
        &mut self,
        render_state: &egui_wgpu::RenderState,
        size_px: [u32; 2],
        camera: &CadCamera,
        doc: Option<(&CadDocument, u64)>,
        highlight: &HighlightState,
        display_unit: DisplayUnit,
    ) -> egui::TextureId {
        let w = size_px[0].clamp(2, 4096);
        let h = size_px[1].clamp(2, 4096);
        let needs_recreate = self
            .targets
            .as_ref()
            .is_none_or(|t| t.width != w || t.height != h);

        if needs_recreate {
            let new_targets = FrameTargets::create(&render_state.device, w, h);
            let mut renderer_guard = render_state.renderer.write();
            let tex_id = match self.egui_tex_id {
                Some(existing) => {
                    renderer_guard.update_egui_texture_from_wgpu_texture(
                        &render_state.device,
                        &new_targets.resolved_color_view,
                        wgpu::FilterMode::Linear,
                        existing,
                    );
                    existing
                }
                None => renderer_guard.register_native_texture(
                    &render_state.device,
                    &new_targets.resolved_color_view,
                    wgpu::FilterMode::Linear,
                ),
            };
            self.egui_tex_id = Some(tex_id);
            self.targets = Some(new_targets);
        }

        self.render_internal(
            &render_state.device,
            &render_state.queue,
            camera,
            doc,
            highlight,
            display_unit,
        );

        self.egui_tex_id.unwrap_or(egui::TextureId::Managed(0))
    }

    fn render_internal(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        camera: &CadCamera,
        doc: Option<(&CadDocument, u64)>,
        highlight: &HighlightState,
        display_unit: DisplayUnit,
    ) {
        let Some(targets) = &self.targets else {
            return;
        };
        let w = targets.width;
        let h = targets.height;

        match doc {
            Some((cad_doc, rev)) => {
                if self
                    .uploaded_cad
                    .as_ref()
                    .is_none_or(|u| u.doc_revision != rev)
                {
                    let verts: Vec<SolidVertexGpu> = cad_doc
                        .render_mesh
                        .vertices
                        .iter()
                        .map(|v| SolidVertexGpu {
                            pos_mm: v.position_mm,
                            normal: v.normal,
                            face_id: v.face_id,
                        })
                        .collect();
                    let vertex_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("chip_cad_solid_vbuf"),
                        contents: bytemuck::cast_slice(&verts),
                        usage: wgpu::BufferUsages::VERTEX,
                    });
                    let index_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("chip_cad_solid_ibuf"),
                        contents: bytemuck::cast_slice(&cad_doc.render_mesh.indices),
                        usage: wgpu::BufferUsages::INDEX,
                    });
                    self.uploaded_cad = Some(UploadedCadGpu {
                        doc_revision: rev,
                        vertex_buf,
                        index_buf,
                        index_count: cad_doc.render_mesh.indices.len() as u32,
                    });
                }
            }
            None => {
                self.uploaded_cad = None;
            }
        }

        // Update face highlight storage buffer
        let (hov_face, face_states) = highlight.build_face_states(doc.map(|(d, _)| d));
        if face_states.len() > self.face_state_capacity {
            self.face_state_capacity = face_states.len().next_power_of_two();
            self.face_state_buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("chip_vp_face_states"),
                size: (self.face_state_capacity * std::mem::size_of::<u32>()) as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.bind_group =
                create_vp_bind_group(device, &self.bgl, &self.uniform_buf, &self.face_state_buf);
        }
        queue.write_buffer(&self.face_state_buf, 0, bytemuck::cast_slice(&face_states));

        // Build translucent overlay triangles + wireframe/edge/arrow instances into reusable GPU buffers
        let mut overlay_tris = Vec::new();
        let edge_instances = build_edge_and_scene_instances(
            doc.map(|(d, _)| d),
            highlight,
            camera,
            &mut overlay_tris,
        );
        self.overlay_buf.upload_slice(device, queue, &overlay_tris);
        self.edge_buf.upload_slice(device, queue, &edge_instances);

        // Update cached Pass #5 GPU-instanced toolpaths only when toolpaths/frame/toggles change!
        let tp_sig = compute_toolpath_gpu_signature(highlight);
        if self
            .uploaded_toolpaths
            .as_ref()
            .is_none_or(|u| u.signature_hash != tp_sig)
        {
            let tp_instances = build_toolpath_gpu_instances(highlight);
            let buf = if tp_instances.is_empty() {
                None
            } else {
                let b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("chip_toolpath_instances"),
                    contents: bytemuck::cast_slice(&tp_instances),
                    usage: wgpu::BufferUsages::VERTEX,
                });
                Some((b, tp_instances.len() as u32))
            };
            self.uploaded_toolpaths = Some(UploadedToolpathsGpu {
                signature_hash: tp_sig,
                buf,
            });
        }

        // Upload Pass #6 3D machined stock mesh & 3D revolved cutter assembly when revision changes
        match &highlight.sim_stock_mesh {
            Some((rev, stock_mesh)) => {
                if self
                    .uploaded_sim_stock
                    .as_ref()
                    .is_none_or(|u| u.revision != *rev)
                {
                    self.uploaded_sim_stock = upload_sim_indexed_mesh(
                        device,
                        *rev,
                        &stock_mesh.vertices,
                        &stock_mesh.indices,
                        "chip_sim_stock",
                    );
                }
            }
            None => {
                self.uploaded_sim_stock = None;
            }
        }

        let mut tool_tip_wcs = [0.0_f32; 4];
        match &highlight.sim_tool_mesh {
            Some((rev, tool_mesh, tip_wcs)) => {
                tool_tip_wcs = [tip_wcs.x as f32, tip_wcs.y as f32, tip_wcs.z as f32, 1.0];
                if self
                    .uploaded_sim_tool
                    .as_ref()
                    .is_none_or(|u| u.revision != *rev)
                {
                    self.uploaded_sim_tool = upload_sim_indexed_mesh(
                        device,
                        *rev,
                        &tool_mesh.vertices,
                        &tool_mesh.indices,
                        "chip_sim_tool",
                    );
                }
            }
            None => {
                self.uploaded_sim_tool = None;
            }
        }

        let (vp, inv_vp, eye, view) = camera.gpu_matrices(w, h);
        let (minor_mm, major_mm) = adaptive_grid_steps(camera, h as f32, display_unit);
        let wcs_l2w = dmat4_to_f32_2d(highlight.wcs_frame.to_dmat4_local_to_world());
        let wcs_w2l = dmat4_to_f32_2d(highlight.wcs_frame.to_dmat4_world_to_local());
        let cad_model = dmat4_to_f32_2d(highlight.cad_world_transform());

        let tok = crate::theme::ThemeTokens::for_theme(highlight.theme);
        let uniforms = ViewportUniformsGpu {
            view_proj: vp.to_cols_array_2d(),
            inv_view_proj: inv_vp.to_cols_array_2d(),
            view: view.to_cols_array_2d(),
            wcs_world_to_local: wcs_w2l,
            wcs_local_to_world: wcs_l2w,
            cad_model,
            camera_pos: [eye.x, eye.y, eye.z, 1.0],
            viewport_and_grid: [w as f32, h as f32, minor_mm, major_mm],
            pick_flags: [
                hov_face,
                u32::from(highlight.ghost_part),
                u32::from(highlight.show_grid),
                u32::from(camera.projection.is_orthographic()),
            ],
            sim_tool_tip_wcs: tool_tip_wcs,
            bg_top_rgb: [
                tok.vp_bg_top_rgb[0],
                tok.vp_bg_top_rgb[1],
                tok.vp_bg_top_rgb[2],
                1.0,
            ],
            bg_bottom_rgb: [
                tok.vp_bg_bottom_rgb[0],
                tok.vp_bg_bottom_rgb[1],
                tok.vp_bg_bottom_rgb[2],
                1.0,
            ],
            grid_rgb_and_scale: [
                tok.vp_grid_rgb[0],
                tok.vp_grid_rgb[1],
                tok.vp_grid_rgb[2],
                tok.vp_grid_alpha_scale,
            ],
        };
        queue.write_buffer(&self.uniform_buf, 0, bytemuck::bytes_of(&uniforms));

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("chip_viewport_encoder"),
        });

        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("chip_viewport_msaa_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &targets.msaa_color_view,
                    resolve_target: Some(&targets.resolved_color_view),
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.09,
                            g: 0.10,
                            b: 0.12,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Discard,
                    },
                    depth_slice: None,
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &targets.msaa_depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                occlusion_query_set: None,
                timestamp_writes: None,
                multiview_mask: None,
            });

            pass.set_bind_group(0, &self.bind_group, &[]);

            // 1. Studio gradient + active Setup WCS XY grid
            pass.set_pipeline(&self.bg_pipeline);
            pass.draw(0..3, 0..1);

            // 2A / Pass #6A: Shaded 3D Machined Stock Mesh (when simulation stock view is active)
            let showing_sim_stock = if let Some(sim_stock_gpu) = &self.uploaded_sim_stock
                && sim_stock_gpu.index_count > 0
            {
                pass.set_pipeline(&self.sim_stock_pipeline);
                pass.set_vertex_buffer(0, sim_stock_gpu.vertex_buf.slice(..));
                pass.set_index_buffer(sim_stock_gpu.index_buf.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..sim_stock_gpu.index_count, 0, 0..1);
                true
            } else {
                false
            };

            // 2B. Shaded B-Rep solid (opaque normally, or translucent ghost when comparing against machined stock)
            if (!showing_sim_stock || highlight.ghost_part)
                && let Some(cad_gpu) = &self.uploaded_cad
                && cad_gpu.index_count > 0
            {
                if highlight.ghost_part {
                    pass.set_pipeline(&self.solid_ghost_pipeline);
                } else {
                    pass.set_pipeline(&self.solid_pipeline);
                }
                pass.set_vertex_buffer(0, cad_gpu.vertex_buf.slice(..));
                pass.set_index_buffer(cad_gpu.index_buf.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..cad_gpu.index_count, 0, 0..1);
            }

            // Pass #6B: Animated 3D Revolved Cutter + Shank + Collet/Holder Assembly
            if let Some(sim_tool_gpu) = &self.uploaded_sim_tool
                && sim_tool_gpu.index_count > 0
            {
                pass.set_pipeline(&self.sim_tool_pipeline);
                pass.set_vertex_buffer(0, sim_tool_gpu.vertex_buf.slice(..));
                pass.set_index_buffer(sim_tool_gpu.index_buf.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..sim_tool_gpu.index_count, 0, 0..1);
            }

            // 3. Translucent Stock & Z-Height Planes
            if let Some((ov_buf, vert_count)) = self.overlay_buf.active() {
                pass.set_pipeline(&self.overlay_pipeline);
                pass.set_vertex_buffer(0, ov_buf.slice(..));
                pass.draw(0..vert_count, 0..1);
            }

            // 4. Topological edges, Stock wireframe, WCS triad, 3D chain arrows, & Tab bridges
            if let Some((edge_buf, count)) = self.edge_buf.active() {
                pass.set_pipeline(&self.edge_pipeline);
                pass.set_vertex_buffer(0, edge_buf.slice(..));
                pass.draw(0..6, 0..count);
            }

            // 5. GPU-Instanced Toolpath Pass (500,000+ segments @ 60 FPS)
            if let Some(tp_gpu) = &self.uploaded_toolpaths
                && let Some((tp_buf, tp_count)) = &tp_gpu.buf
                && *tp_count > 0
            {
                pass.set_pipeline(&self.edge_pipeline);
                pass.set_vertex_buffer(0, tp_buf.slice(..));
                pass.draw(0..6, 0..*tp_count);
            }
        }

        queue.submit(std::iter::once(encoder.finish()));
    }
}

fn create_vp_bind_group(
    device: &wgpu::Device,
    bgl: &wgpu::BindGroupLayout,
    uniform_buf: &wgpu::Buffer,
    face_state_buf: &wgpu::Buffer,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("chip_vp_bg"),
        layout: bgl,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: face_state_buf.as_entire_binding(),
            },
        ],
    })
}

/// Renders a [`CadDocument`] completely headless via `wgpu` (e.g. Mesa Lavapipe in CI)
/// and encodes an RGBA8 PNG byte buffer.
pub fn render_offscreen_png(
    doc: &CadDocument,
    camera: &CadCamera,
    highlight: &HighlightState,
    width: u32,
    height: u32,
) -> Result<Vec<u8>, String> {
    let instance =
        wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::default(),
        compatible_surface: None,
        force_fallback_adapter: false,
        apply_limit_buckets: false,
    }))
    .map_err(|e| format!("failed to request wgpu adapter: {e}"))?;

    let (device, queue) =
        pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default()))
            .map_err(|e| format!("failed to create wgpu device: {e}"))?;

    let mut vp = ViewportRenderer::new(&device);
    vp.targets = Some(FrameTargets::create(&device, width, height));
    vp.render_internal(
        &device,
        &queue,
        camera,
        Some((doc, 1)),
        highlight,
        DisplayUnit::Millimeter,
    );

    let Some(targets) = &vp.targets else {
        return Err("missing offscreen frame targets".to_string());
    };
    let w = targets.width;
    let h = targets.height;
    let unpadded_row = w * 4;
    let align = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let padded_row = unpadded_row.div_ceil(align) * align;

    let readback_buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("chip_offscreen_readback"),
        size: (padded_row * h) as u64,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("chip_readback_encoder"),
    });
    encoder.copy_texture_to_buffer(
        targets.resolved_color.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &readback_buf,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded_row),
                rows_per_image: Some(h),
            },
        },
        wgpu::Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        },
    );
    queue.submit(std::iter::once(encoder.finish()));

    let slice = readback_buf.slice(..);
    let (tx, rx) = crossbeam_channel::bounded(1);
    slice.map_async(wgpu::MapMode::Read, move |res| {
        let _ = tx.send(res);
    });
    let _ = device.poll(wgpu::PollType::wait_indefinitely());
    rx.recv()
        .map_err(|e| format!("readback channel failed: {e}"))?
        .map_err(|e| format!("buffer map failed: {e}"))?;

    let mapped = slice
        .get_mapped_range()
        .map_err(|e| format!("get_mapped_range failed: {e}"))?;
    let mut pixels = Vec::with_capacity((w * h * 4) as usize);
    for row in 0..h {
        let start = (row * padded_row) as usize;
        let end = start + (unpadded_row as usize);
        pixels.extend_from_slice(&mapped[start..end]);
    }
    drop(mapped);
    readback_buf.unmap();

    let mut png_bytes = Vec::new();
    let encoder = image::codecs::png::PngEncoder::new(&mut png_bytes);
    use image::ImageEncoder as _;
    encoder
        .write_image(&pixels, w, h, image::ExtendedColorType::Rgba8)
        .map_err(|e| format!("PNG encode failed: {e}"))?;

    Ok(png_bytes)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::{collections::HashSet, sync::Arc};

    use chip_cad::{
        CadKernel, FaceId, MonstertruckKernel, PickTarget, sample_machined_plate_step,
        sample_multi_feature_bracket_step,
    };
    use chip_cam::{Setup, WorkOffsetSlot};
    use chip_math::DEFAULT_TESS_CHORD_TOL_MM;

    use super::*;
    use crate::viewport::scene_builders::{ToolpathRenderEntry, ZPlaneVisual};

    #[test]
    fn growable_gpu_buffer_reuses_allocation_and_grows_by_powers_of_two() {
        let instance =
            wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::default(),
            compatible_surface: None,
            force_fallback_adapter: false,
            apply_limit_buckets: false,
        }))
        .unwrap();
        let (device, queue) =
            pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default())).unwrap();

        let mut gbuf = GrowableGpuBuffer::new("test_growable", wgpu::BufferUsages::VERTEX);
        assert!(gbuf.active().is_none());
        assert_eq!(gbuf.capacity_bytes(), 0);

        gbuf.upload_slice::<OverlayVertexGpu>(&device, &queue, &[]);
        assert!(gbuf.active().is_none());
        assert_eq!(gbuf.capacity_bytes(), 0);

        // 3 * 28 bytes = 84 bytes -> clamped to minimum 256 bytes
        let small = vec![
            OverlayVertexGpu {
                pos_mm: [0.0, 0.0, 0.0],
                color: [1.0, 0.0, 0.0, 1.0],
            };
            3
        ];
        gbuf.upload_slice(&device, &queue, &small);
        assert_eq!(gbuf.active().map(|(_, c)| c), Some(3));
        assert_eq!(gbuf.capacity_bytes(), 256);

        // Empty frame keeps buffer allocated for reuse
        gbuf.upload_slice::<OverlayVertexGpu>(&device, &queue, &[]);
        assert!(gbuf.active().is_none());
        assert_eq!(gbuf.capacity_bytes(), 256);

        // Smaller/equal frame reuses existing 256-byte buffer without reallocating
        gbuf.upload_slice(&device, &queue, &small[..2]);
        assert_eq!(gbuf.active().map(|(_, c)| c), Some(2));
        assert_eq!(gbuf.capacity_bytes(), 256);

        // 20 * 28 bytes = 560 bytes -> grows to next power of two (1024 bytes)
        let larger = vec![
            OverlayVertexGpu {
                pos_mm: [1.0, 2.0, 3.0],
                color: [0.0, 1.0, 0.0, 0.5],
            };
            20
        ];
        gbuf.upload_slice(&device, &queue, &larger);
        assert_eq!(gbuf.active().map(|(_, c)| c), Some(20));
        assert_eq!(gbuf.capacity_bytes(), 1024);
    }

    #[test]
    fn renders_offscreen_cad_viewport_frame_with_shaded_faces_and_edges() {
        let step = sample_machined_plate_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("sample.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();

        let mut cam = CadCamera::default();
        cam.fit_aabb(&doc.aabb_mm);

        let highlight = HighlightState {
            selected: Some(PickTarget::Face(FaceId(0))),
            ..HighlightState::default()
        };

        let png = render_offscreen_png(&doc, &cam, &highlight, 640, 480).unwrap();
        assert!(png.starts_with(b"\x89PNG\r\n\x1a\n"));
        assert!(
            png.len() > 8_000,
            "expected rich shaded CAD PNG (>8KB), got {} bytes",
            png.len()
        );
    }

    #[test]
    fn renders_stock_z_planes_wcs_triad_ghosting_and_multi_face_highlights() {
        let step = sample_multi_feature_bracket_step(false).unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("multi.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();

        let setup = Setup::new("s1", "Op 1", WorkOffsetSlot::G54);
        let resolved = setup
            .resolve_immutable(&doc, DisplayUnit::Millimeter)
            .unwrap();

        let mut cam = CadCamera::default();
        cam.align_to_setup_top(&resolved.frame, &resolved.stock.aabb_wcs());

        let mut sel_faces = HashSet::new();
        sel_faces.insert(FaceId(1));
        sel_faces.insert(FaceId(2));

        let highlight = HighlightState {
            extra_selected_faces: sel_faces,
            wcs_frame: resolved.frame,
            stock: Some(resolved.stock),
            show_stock: true,
            z_planes_wcs: vec![
                ZPlaneVisual {
                    z_wcs: resolved.clearance_z_wcs,
                    fill_rgba: [0.25, 0.72, 1.0, 0.12],
                    border_rgba: [0.35, 0.80, 1.0, 0.85],
                },
                ZPlaneVisual {
                    z_wcs: resolved.retract_z_wcs,
                    fill_rgba: [1.0, 0.72, 0.22, 0.10],
                    border_rgba: [1.0, 0.78, 0.28, 0.85],
                },
            ],
            ghost_part: true,
            show_grid: true,
            ..HighlightState::default()
        };

        let png = render_offscreen_png(&doc, &cam, &highlight, 640, 480).unwrap();
        assert!(png.starts_with(b"\x89PNG\r\n\x1a\n"));
        assert!(png.len() > 8_000);
    }

    #[test]
    fn renders_instanced_toolpaths_and_holding_tab_markers() {
        use std::sync::atomic::AtomicBool;

        use chip_cad::sample_cam_torture_block_step;
        use chip_cam::{
            HeightReference, HeightSpec, Operation, OperationKind, StrategyParams, TabProfileKind,
            generate_operation_toolpath,
        };
        use chip_tools::default_starter_tool_library;

        let step = sample_cam_torture_block_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("torture.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let setup = Setup::new("s1", "Op 1", WorkOffsetSlot::G54);
        let resolved = setup
            .resolve_immutable(&doc, DisplayUnit::Millimeter)
            .unwrap();

        let starter = default_starter_tool_library();
        let flat_6mm = &starter.tools[0];
        let mut op = Operation::new("op_cnt", "Profile", OperationKind::Contour2D);
        op.tool_id = Some(flat_6mm.id.clone());
        let first_loop = doc.loops[0].id;
        op.selections
            .push(chip_cam::GeometrySelectionItem::ClosedLoop {
                binding: doc.bind_loop(first_loop).unwrap(),
                reverse: false,
                side_flip: false,
            });
        op.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
        if let StrategyParams::Contour2D(p) = op.strategy_mut() {
            p.tabs.enabled = true;
            p.tabs.profile = TabProfileKind::Triangular {
                ramp_angle_deg: 25.0,
            };
        }

        let tp = Arc::new(
            generate_operation_toolpath(
                &doc,
                &resolved,
                &op,
                flat_6mm,
                DisplayUnit::Millimeter,
                true,
                &AtomicBool::new(false),
            )
            .unwrap(),
        );

        let mut cam = CadCamera::default();
        cam.fit_aabb(&doc.aabb_mm);

        let highlight = HighlightState {
            wcs_frame: resolved.frame,
            stock: Some(resolved.stock),
            show_stock: true,
            toolpaths: vec![ToolpathRenderEntry {
                op_id: op.id.clone(),
                cache_key: [7_u8; 32],
                toolpath: tp,
                is_focused: true,
            }],
            show_toolpaths: true,
            show_rapids: true,
            ..HighlightState::default()
        };

        let png = render_offscreen_png(&doc, &cam, &highlight, 640, 480).unwrap();
        assert!(png.starts_with(b"\x89PNG\r\n\x1a\n"));
        assert!(png.len() > 10_000);
    }

    #[test]
    fn renders_pass_6_machined_stock_mesh_and_3d_cutter_assembly() {
        use std::sync::atomic::AtomicBool;

        use chip_cad::{SurfaceKind, sample_cam_torture_block_step};
        use chip_cam::{
            GeometrySelectionItem, Operation, OperationKind, generate_operation_toolpath,
        };
        use chip_sim::{
            MachineKinematicsConfig, SetupSimulationInput, SimOperationInput, SimResolution,
            SimVerificationConfig, build_cutter_assembly_mesh, run_setup_simulation,
            tessellate_stock_mesh,
        };
        use chip_tools::default_starter_tool_library;

        let step = sample_cam_torture_block_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("torture.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let setup = Setup::new("s1", "Op 1", WorkOffsetSlot::G54);
        let resolved = setup
            .resolve_immutable(&doc, DisplayUnit::Millimeter)
            .unwrap();

        let starter = default_starter_tool_library();
        let flat_6mm = &starter.tools[0];
        let pkt_floor = doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { origin_mm, normal }
                    if normal.z > 0.99 && (origin_mm.z - 12.0).abs() < 0.05
                )
            })
            .unwrap()
            .id;

        let mut op = Operation::new("op_pkt", "Island Pocket", OperationKind::Pocket2D);
        op.tool_id = Some(flat_6mm.id.clone());
        op.selections.push(GeometrySelectionItem::PlanarFace {
            binding: doc.bind_face(pkt_floor).unwrap(),
        });

        let tp = generate_operation_toolpath(
            &doc,
            &resolved,
            &op,
            flat_6mm,
            DisplayUnit::Millimeter,
            true,
            &AtomicBool::new(false),
        )
        .unwrap();

        let sim_ops = [SimOperationInput::from_operation(&op, flat_6mm, &tp)];
        let sim_res = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &setup.target_bodies,
            operations: &sim_ops,
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            cancel: None,
        })
        .unwrap();

        let stock_mesh = Arc::new(tessellate_stock_mesh(&sim_res.final_stock, true));
        let tool_mesh = Arc::new(build_cutter_assembly_mesh(flat_6mm, 24, false));
        let mid_snap = sim_res
            .timeline
            .sample_at_time(sim_res.timeline.total_time_sec * 0.5)
            .unwrap();

        let mut cam = CadCamera::default();
        cam.fit_aabb(&doc.aabb_mm);

        let highlight = HighlightState {
            wcs_frame: resolved.frame,
            sim_stock_mesh: Some((1, stock_mesh)),
            sim_tool_mesh: Some((1, tool_mesh, mid_snap.tip_pos_wcs)),
            sim_trail_limit: Some(mid_snap.global_seg_idx),
            ..HighlightState::default()
        };

        let png = render_offscreen_png(&doc, &cam, &highlight, 640, 480).unwrap();
        assert!(png.starts_with(b"\x89PNG\r\n\x1a\n"));
        assert!(png.len() > 10_000);
    }

    #[test]
    fn renders_offscreen_viewport_in_both_dark_and_light_themes() {
        use crate::theme::AppTheme;

        let step_str = sample_machined_plate_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("plate.step", step_str.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let mut cam = CadCamera::default();
        cam.fit_aabb(&doc.aabb_mm);

        let dark_hl = HighlightState {
            theme: AppTheme::Dark,
            ..HighlightState::default()
        };
        let light_hl = HighlightState {
            theme: AppTheme::Light,
            ..HighlightState::default()
        };

        let dark_png = render_offscreen_png(&doc, &cam, &dark_hl, 320, 240).unwrap();
        let light_png = render_offscreen_png(&doc, &cam, &light_hl, 320, 240).unwrap();

        let dark_img = image::load_from_memory(&dark_png).unwrap().to_rgba8();
        let light_img = image::load_from_memory(&light_png).unwrap().to_rgba8();

        // Corner pixel `(4, 4)` samples the studio background gradient directly
        let dp = dark_img.get_pixel(4, 4);
        let lp = light_img.get_pixel(4, 4);
        assert!(
            dp[0] < 80 && lp[0] > 180,
            "expected dark corner {:?} < 80 and light corner {:?} > 180",
            dp,
            lp
        );
    }

    #[test]
    fn cad_world_transform_matches_rotary_wcs_rotation_across_axes_directions_and_tilted_wcs() {
        use chip_cam::{
            ResolvedRotaryConfig, RotaryAxisKind, RotaryDirection, RotaryDriveMode,
            RotaryUnwindMode,
        };
        use chip_math::{DMat3, DVec3, Ray3, RigidFrame3};

        let basis = DMat3::from_axis_angle(DVec3::new(0.3, 0.5, 0.8).normalize(), 0.65);
        let wcs_frame = RigidFrame3::from_z_and_x_hint(
            DVec3::new(12.0, -7.5, 5.0),
            basis.z_axis,
            Some(basis.x_axis),
            false,
            false,
            0,
        )
        .unwrap();

        let sample_pts = [
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(25.0, -14.0, 18.5),
            DVec3::new(-9.25, 31.0, -6.75),
        ];

        for axis in RotaryAxisKind::ALL {
            for direction in RotaryDirection::ALL {
                let rot = ResolvedRotaryConfig {
                    mode: RotaryDriveMode::Simultaneous4Axis,
                    axis,
                    axis_letter: axis.default_letter(),
                    direction,
                    center_wcs: DVec3::new(3.5, -2.0, 4.25),
                    stock_u_min: -40.0,
                    stock_u_max: 40.0,
                    stock_outer_radius_mm: 22.0,
                    clearance_radius_mm: 30.0,
                    chuck_end_positive: false,
                    chuck_u_min: -60.0,
                    chuck_u_max: -40.0,
                    chuck_radius_mm: 32.0,
                    unwind_mode: RotaryUnwindMode::ContinuousMultiTurn,
                };

                let hl = HighlightState {
                    wcs_frame,
                    rotary: Some(rot),
                    sim_rotary_deg: 127.5,
                    ..HighlightState::default()
                };
                assert!(hl.has_active_cad_rotary_transform());

                let mat = hl.cad_world_transform();
                for p_world in sample_pts {
                    let p_wcs = wcs_frame.world_to_local_point(p_world);
                    let expected_world =
                        wcs_frame.local_to_world_point(rot.rotate_point_wcs(p_wcs, 127.5));
                    let actual_world = mat.transform_point3(p_world);
                    assert!(
                        actual_world.distance(expected_world) < 1e-9,
                        "axis={axis:?} dir={direction:?}: actual={actual_world:?} expected={expected_world:?}"
                    );
                }

                // Verify picking ray inverse round-trip
                let ray_model = Ray3::new(
                    DVec3::new(40.0, -20.0, 60.0),
                    DVec3::new(-0.4, 0.25, -0.88).normalize(),
                )
                .unwrap();
                let ray_world = Ray3::new(
                    mat.transform_point3(ray_model.origin),
                    mat.transform_vector3(ray_model.dir),
                )
                .unwrap();
                let ray_back = hl.world_ray_to_cad_model_space(&ray_world);
                assert!(ray_back.origin.distance(ray_model.origin) < 1e-9);
                assert!(ray_back.dir.distance(ray_model.dir) < 1e-9);
            }
        }
    }

    #[test]
    fn rotary_simulation_rotates_ghost_model_solid_and_brep_edges_in_sync() {
        use chip_cam::{
            ResolvedRotaryConfig, RotaryAxisKind, RotaryDirection, RotaryDriveMode,
            RotaryUnwindMode,
        };
        use chip_math::{DVec3, RigidFrame3};

        let step = sample_machined_plate_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("plate.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();

        let rot = ResolvedRotaryConfig {
            mode: RotaryDriveMode::Simultaneous4Axis,
            axis: RotaryAxisKind::XAxis,
            axis_letter: 'A',
            direction: RotaryDirection::RightHandCcw,
            center_wcs: DVec3::new(0.0, 20.0, 5.0),
            stock_u_min: -60.0,
            stock_u_max: 60.0,
            stock_outer_radius_mm: 40.0,
            clearance_radius_mm: 50.0,
            chuck_end_positive: false,
            chuck_u_min: -80.0,
            chuck_u_max: -60.0,
            chuck_radius_mm: 45.0,
            unwind_mode: RotaryUnwindMode::ContinuousMultiTurn,
        };

        // Pre-rotate a reference copy of the CAD render mesh + B-Rep edge polylines by +90°
        let angle_deg = 90.0;
        let mut doc_rot90 = doc.clone();
        for v in &mut doc_rot90.render_mesh.vertices {
            let p = rot.rotate_point_wcs(
                DVec3::new(
                    f64::from(v.position_mm[0]),
                    f64::from(v.position_mm[1]),
                    f64::from(v.position_mm[2]),
                ),
                angle_deg,
            );
            let n_tip = rot.rotate_point_wcs(
                rot.center_wcs
                    + DVec3::new(
                        f64::from(v.normal[0]),
                        f64::from(v.normal[1]),
                        f64::from(v.normal[2]),
                    ),
                angle_deg,
            );
            let n = (n_tip - rot.center_wcs).normalize_or_zero();
            v.position_mm = [p.x as f32, p.y as f32, p.z as f32];
            v.normal = [n.x as f32, n.y as f32, n.z as f32];
        }
        for poly in &mut doc_rot90.render_mesh.edge_polylines {
            for pt in &mut poly.points_mm {
                *pt = rot.rotate_point_wcs(*pt, angle_deg);
            }
        }

        let mut cam = CadCamera::default();
        cam.fit_aabb(&chip_math::Aabb3::new(
            DVec3::new(-70.0, -70.0, -70.0),
            DVec3::new(70.0, 70.0, 70.0),
        ));

        let base_hl = HighlightState {
            wcs_frame: RigidFrame3::IDENTITY,
            rotary: Some(rot),
            ghost_part: true,
            show_stock: false,
            show_grid: false,
            ..HighlightState::default()
        };

        let mut hl_sim90 = base_hl.clone();
        hl_sim90.sim_rotary_deg = angle_deg;

        let png_sim90 = render_offscreen_png(&doc, &cam, &hl_sim90, 320, 240).unwrap();
        let png_ref90 = render_offscreen_png(&doc_rot90, &cam, &base_hl, 320, 240).unwrap();
        let png_unrot = render_offscreen_png(&doc, &cam, &base_hl, 320, 240).unwrap();

        let img_sim90 = image::load_from_memory(&png_sim90).unwrap().to_rgba8();
        let img_ref90 = image::load_from_memory(&png_ref90).unwrap().to_rgba8();
        let img_unrot = image::load_from_memory(&png_unrot).unwrap().to_rgba8();

        let mut diff_vs_ref = 0usize;
        let mut diff_vs_unrot = 0usize;
        for (p_sim, (p_ref, p_unrot)) in img_sim90
            .pixels()
            .zip(img_ref90.pixels().zip(img_unrot.pixels()))
        {
            let max_d_ref = (0..4)
                .map(|c| p_sim[c].abs_diff(p_ref[c]))
                .max()
                .unwrap_or(0);
            if max_d_ref > 2 {
                diff_vs_ref += 1;
            }
            let max_d_unrot = (0..4)
                .map(|c| p_sim[c].abs_diff(p_unrot[c]))
                .max()
                .unwrap_or(0);
            if max_d_unrot > 8 {
                diff_vs_unrot += 1;
            }
        }

        assert!(
            diff_vs_unrot > 1_000,
            "ghost model at 90° must move visibly relative to 0° (only {diff_vs_unrot} pixels changed)"
        );
        assert!(
            diff_vs_ref <= 16,
            "ghost model shaded solid + B-Rep edges at sim_rotary_deg=90° must match pre-rotated reference model (got {diff_vs_ref} differing pixels)"
        );
    }
}
