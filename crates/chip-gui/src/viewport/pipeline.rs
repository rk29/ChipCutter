use std::collections::HashSet;

use bytemuck::{Pod, Zeroable};
use chip_cad::{CadDocument, EdgeId, PickTarget};
use chip_math::{DisplayUnit, MM_PER_INCH};
use wgpu::util::DeviceExt as _;

use crate::camera::{CadCamera, StandardView};

const SAMPLE_COUNT: u32 = 4;
const COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct ViewportUniformsGpu {
    view_proj: [[f32; 4]; 4],
    inv_view_proj: [[f32; 4]; 4],
    view: [[f32; 4]; 4],
    camera_pos: [f32; 4],
    viewport_and_grid: [f32; 4],
    pick_flags: [u32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct SolidVertexGpu {
    pos_mm: [f32; 3],
    normal: [f32; 3],
    face_id: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct EdgeInstanceGpu {
    p0_mm: [f32; 3],
    p1_mm: [f32; 3],
    color: [f32; 4],
    width_px: f32,
}

/// Active hover and selection highlight state fed into the 3D CAD viewport.
#[derive(Debug, Clone, Default)]
pub struct HighlightState {
    pub hovered: Option<PickTarget>,
    pub selected: Option<PickTarget>,
    pub show_grid: bool,
}

impl HighlightState {
    fn face_ids(&self) -> (u32, u32) {
        let hov = match &self.hovered {
            Some(PickTarget::Face(fid)) => fid.0,
            _ => u32::MAX,
        };
        let sel = match &self.selected {
            Some(PickTarget::Face(fid)) => fid.0,
            _ => u32::MAX,
        };
        (hov, sel)
    }

    fn collect_edge_sets(&self) -> (HashSet<EdgeId>, HashSet<EdgeId>) {
        let mut hov = HashSet::new();
        let mut sel = HashSet::new();
        match &self.hovered {
            Some(PickTarget::Edge(eid)) => {
                hov.insert(*eid);
            }
            Some(PickTarget::Loop { edges, .. }) => {
                hov.extend(edges.iter().copied());
            }
            _ => {}
        }
        match &self.selected {
            Some(PickTarget::Edge(eid)) => {
                sel.insert(*eid);
            }
            Some(PickTarget::Loop { edges, .. }) => {
                sel.extend(edges.iter().copied());
            }
            _ => {}
        }
        (hov, sel)
    }
}

struct FrameTargets {
    width: u32,
    height: u32,
    _msaa_color: wgpu::Texture,
    msaa_color_view: wgpu::TextureView,
    _msaa_depth: wgpu::Texture,
    msaa_depth_view: wgpu::TextureView,
    resolved_color: wgpu::Texture,
    resolved_color_view: wgpu::TextureView,
}

impl FrameTargets {
    fn create(device: &wgpu::Device, width: u32, height: u32) -> Self {
        let w = width.clamp(2, 4096);
        let h = height.clamp(2, 4096);
        let size = wgpu::Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        };

        let msaa_color = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("chip_vp_msaa_color"),
            size,
            mip_level_count: 1,
            sample_count: SAMPLE_COUNT,
            dimension: wgpu::TextureDimension::D2,
            format: COLOR_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let msaa_color_view = msaa_color.create_view(&wgpu::TextureViewDescriptor::default());

        let msaa_depth = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("chip_vp_msaa_depth"),
            size,
            mip_level_count: 1,
            sample_count: SAMPLE_COUNT,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let msaa_depth_view = msaa_depth.create_view(&wgpu::TextureViewDescriptor::default());

        let resolved_color = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("chip_vp_resolved_color"),
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: COLOR_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let resolved_color_view =
            resolved_color.create_view(&wgpu::TextureViewDescriptor::default());

        Self {
            width: w,
            height: h,
            _msaa_color: msaa_color,
            msaa_color_view,
            _msaa_depth: msaa_depth,
            msaa_depth_view,
            resolved_color,
            resolved_color_view,
        }
    }
}

struct UploadedCadGpu {
    doc_revision: u64,
    vertex_buf: wgpu::Buffer,
    index_buf: wgpu::Buffer,
    index_count: u32,
}

/// 4x MSAA `wgpu` 3D CAD/CAM viewport renderer shared by `egui` and headless tests.
pub struct ViewportRenderer {
    uniform_buf: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    bg_pipeline: wgpu::RenderPipeline,
    solid_pipeline: wgpu::RenderPipeline,
    edge_pipeline: wgpu::RenderPipeline,
    targets: Option<FrameTargets>,
    egui_tex_id: Option<egui::TextureId>,
    uploaded_cad: Option<UploadedCadGpu>,
    edge_buf: Option<(wgpu::Buffer, u32)>,
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

        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("chip_vp_bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("chip_vp_bg"),
            layout: &bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buf.as_entire_binding(),
            }],
        });

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

        // Pass 1: Background gradient + depth-writing XY grid
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

        // Pass 2: Shaded B-Rep solid faces
        let solid_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("chip_solid_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_solid"),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<SolidVertexGpu>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![
                        0 => Float32x3,
                        1 => Float32x3,
                        2 => Uint32,
                    ],
                })],
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

        // Pass 3: Screen-space topological edges & WCS triad
        let edge_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("chip_edge_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_edge"),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<EdgeInstanceGpu>() as u64,
                    step_mode: wgpu::VertexStepMode::Instance,
                    attributes: &wgpu::vertex_attr_array![
                        0 => Float32x3,
                        1 => Float32x3,
                        2 => Float32x4,
                        3 => Float32,
                    ],
                })],
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

        Self {
            uniform_buf,
            bind_group,
            bg_pipeline,
            solid_pipeline,
            edge_pipeline,
            targets: None,
            egui_tex_id: None,
            uploaded_cad: None,
            edge_buf: None,
        }
    }

    /// Ensures the offscreen MSAA + resolved targets match `(width_px, height_px)` and updates
    /// the registered `egui::TextureId` inside `egui_wgpu::RenderState`.
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

        // Upload solid triangle buffers only when CAD document revision changes
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

        // Build edge + WCS origin triad instance buffer
        let edge_instances = build_edge_and_triad_instances(doc.map(|(d, _)| d), highlight, camera);
        if edge_instances.is_empty() {
            self.edge_buf = None;
        } else {
            let buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("chip_edge_instances"),
                contents: bytemuck::cast_slice(&edge_instances),
                usage: wgpu::BufferUsages::VERTEX,
            });
            self.edge_buf = Some((buf, edge_instances.len() as u32));
        }

        // Update viewport uniform buffer
        let (vp, inv_vp, eye, view) = camera.gpu_matrices(w, h);
        let (minor_mm, major_mm) = adaptive_grid_steps(camera, h as f32, display_unit);
        let (hov_face, sel_face) = highlight.face_ids();

        let uniforms = ViewportUniformsGpu {
            view_proj: vp.to_cols_array_2d(),
            inv_view_proj: inv_vp.to_cols_array_2d(),
            view: view.to_cols_array_2d(),
            camera_pos: [eye.x, eye.y, eye.z, 1.0],
            viewport_and_grid: [w as f32, h as f32, minor_mm, major_mm],
            pick_flags: [
                hov_face,
                sel_face,
                u32::from(highlight.show_grid),
                u32::from(camera.projection.is_orthographic()),
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

            // 1. Studio gradient + WCS XY grid
            pass.set_pipeline(&self.bg_pipeline);
            pass.draw(0..3, 0..1);

            // 2. Shaded B-Rep solid
            if let Some(cad_gpu) = &self.uploaded_cad
                && cad_gpu.index_count > 0
            {
                pass.set_pipeline(&self.solid_pipeline);
                pass.set_vertex_buffer(0, cad_gpu.vertex_buf.slice(..));
                pass.set_index_buffer(cad_gpu.index_buf.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..cad_gpu.index_count, 0, 0..1);
            }

            // 3. Topological B-Rep edges + RGB WCS origin triad
            if let Some((edge_buf, count)) = &self.edge_buf
                && *count > 0
            {
                pass.set_pipeline(&self.edge_pipeline);
                pass.set_vertex_buffer(0, edge_buf.slice(..));
                pass.draw(0..6, 0..*count);
            }
        }

        queue.submit(std::iter::once(encoder.finish()));
    }
}

fn adaptive_grid_steps(camera: &CadCamera, height_px: f32, unit: DisplayUnit) -> (f32, f32) {
    let mm_per_px = camera.mm_per_pixel(height_px);
    let target_minor_mm = mm_per_px * 18.0;
    match unit {
        DisplayUnit::Millimeter => {
            let candidates = [0.1, 0.5, 1.0, 5.0, 10.0, 25.0, 50.0, 100.0];
            let minor = candidates
                .into_iter()
                .find(|&c| c >= target_minor_mm)
                .unwrap_or(100.0);
            (minor as f32, (minor * 10.0) as f32)
        }
        DisplayUnit::Inch => {
            let candidates_in = [0.01, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0];
            let target_in = target_minor_mm / MM_PER_INCH;
            let minor_in = candidates_in
                .into_iter()
                .find(|&c| c >= target_in)
                .unwrap_or(5.0);
            let minor_mm = minor_in * MM_PER_INCH;
            let major_mm = minor_mm * 10.0;
            (minor_mm as f32, major_mm as f32)
        }
    }
}

fn build_edge_and_triad_instances(
    doc: Option<&CadDocument>,
    highlight: &HighlightState,
    camera: &CadCamera,
) -> Vec<EdgeInstanceGpu> {
    let mut out = Vec::new();
    let (hov_edges, sel_edges) = highlight.collect_edge_sets();

    if let Some(cad_doc) = doc {
        for poly in &cad_doc.render_mesh.edge_polylines {
            let (color, width_px) = if sel_edges.contains(&poly.edge_id) {
                ([1.0, 0.78, 0.18, 1.0], 3.4)
            } else if hov_edges.contains(&poly.edge_id) {
                ([0.28, 0.88, 1.0, 1.0], 3.1)
            } else {
                ([0.11, 0.13, 0.16, 0.92], 1.55)
            };

            for w in poly.points_mm.windows(2) {
                out.push(EdgeInstanceGpu {
                    p0_mm: [w[0].x as f32, w[0].y as f32, w[0].z as f32],
                    p1_mm: [w[1].x as f32, w[1].y as f32, w[1].z as f32],
                    color,
                    width_px,
                });
            }
        }
    }

    // WCS XYZ Origin Triad scaled gently to scene size
    let triad_len = (camera.scene_radius_mm * 0.28).clamp(8.0, 45.0) as f32;
    out.push(EdgeInstanceGpu {
        p0_mm: [0.0, 0.0, 0.0],
        p1_mm: [triad_len, 0.0, 0.0],
        color: [0.95, 0.28, 0.28, 1.0],
        width_px: 2.6,
    });
    out.push(EdgeInstanceGpu {
        p0_mm: [0.0, 0.0, 0.0],
        p1_mm: [0.0, triad_len, 0.0],
        color: [0.28, 0.88, 0.42, 1.0],
        width_px: 2.6,
    });
    out.push(EdgeInstanceGpu {
        p0_mm: [0.0, 0.0, 0.0],
        p1_mm: [0.0, 0.0, triad_len],
        color: [0.32, 0.58, 0.98, 1.0],
        width_px: 2.6,
    });

    out
}

/// Draws an interactive 3D orientation gizmo + quick-snap ViewCube buttons in the top-right corner
/// of `viewport_rect`.
pub fn draw_view_cube_overlay(
    ui: &mut egui::Ui,
    viewport_rect: egui::Rect,
    camera: &mut CadCamera,
) {
    let box_w = 152.0;
    let box_h = 152.0;
    if viewport_rect.width() < box_w + 24.0 || viewport_rect.height() < box_h + 24.0 {
        return;
    }

    let overlay_rect = egui::Rect::from_min_size(
        egui::pos2(
            viewport_rect.right() - box_w - 10.0,
            viewport_rect.top() + 10.0,
        ),
        egui::vec2(box_w, box_h),
    );

    let painter = ui.painter_at(overlay_rect);
    painter.rect(
        overlay_rect,
        6.0,
        egui::Color32::from_rgba_unmultiplied(20, 24, 30, 210),
        egui::Stroke::new(1.0, egui::Color32::from_rgb(58, 66, 78)),
        egui::StrokeKind::Inside,
    );

    // Draw live rotating 3D XYZ orientation triad in upper half of the ViewCube card
    let center = egui::pos2(overlay_rect.center().x, overlay_rect.top() + 44.0);
    let (_, _, _, view_mat) = camera.gpu_matrices(100, 100);
    let axes = [
        (
            glam::Vec3::X,
            "X",
            egui::Color32::from_rgb(242, 78, 78),
            StandardView::Right,
        ),
        (
            glam::Vec3::Y,
            "Y",
            egui::Color32::from_rgb(76, 218, 108),
            StandardView::Back,
        ),
        (
            glam::Vec3::Z,
            "Z",
            egui::Color32::from_rgb(86, 152, 250),
            StandardView::Top,
        ),
    ];

    let arm_px = 28.0_f32;
    let mut sorted_axes: Vec<_> = axes
        .into_iter()
        .map(|(dir, label, col, snap_view)| {
            let v = view_mat.transform_vector3(dir);
            (v, label, col, snap_view)
        })
        .collect();
    sorted_axes.sort_by(|a, b| a.0.z.total_cmp(&b.0.z));

    let mut clicked_view: Option<StandardView> = None;

    for (v, label, col, snap_view) in sorted_axes {
        let tip = egui::pos2(center.x + v.x * arm_px, center.y - v.y * arm_px);
        painter.line_segment([center, tip], egui::Stroke::new(2.2, col));
        let badge_rect = egui::Rect::from_center_size(tip, egui::vec2(15.0, 15.0));
        let resp = ui.interact(
            badge_rect,
            ui.scope_id().with(("viewcube_axis", label)),
            egui::Sense::click(),
        );
        let fill = if resp.hovered() {
            egui::Color32::WHITE
        } else {
            col
        };
        painter.circle_filled(tip, 7.2, fill);
        painter.text(
            tip,
            egui::Align2::CENTER_CENTER,
            label,
            egui::FontId::proportional(10.0),
            egui::Color32::from_rgb(16, 18, 22),
        );
        if resp.clicked() {
            clicked_view = Some(snap_view);
        }
    }

    // Quick-snap CAD view buttons in lower half of card
    let btn_area = egui::Rect::from_min_max(
        egui::pos2(overlay_rect.left() + 6.0, overlay_rect.top() + 84.0),
        egui::pos2(overlay_rect.right() - 6.0, overlay_rect.bottom() - 6.0),
    );
    let mut child_ui = ui.new_child(
        egui::UiBuilder::new()
            .max_rect(btn_area)
            .layout(egui::Layout::top_down(egui::Align::Center)),
    );
    child_ui.spacing_mut().item_spacing = egui::vec2(4.0, 4.0);
    child_ui.horizontal(|ui| {
        for (lbl, v) in [
            ("Top", StandardView::Top),
            ("Front", StandardView::Front),
            ("Right", StandardView::Right),
        ] {
            if ui.small_button(lbl).clicked() {
                clicked_view = Some(v);
            }
        }
    });
    child_ui.horizontal(|ui| {
        for (lbl, v) in [
            ("Iso", StandardView::Isometric),
            ("Left", StandardView::Left),
            ("Bot", StandardView::Bottom),
        ] {
            if ui.small_button(lbl).clicked() {
                clicked_view = Some(v);
            }
        }
    });

    if let Some(v) = clicked_view {
        camera.set_standard_view(v);
    }
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
    use chip_cad::{CadKernel, FaceId, MonstertruckKernel, PickTarget, sample_machined_plate_step};
    use chip_math::DEFAULT_TESS_CHORD_TOL_MM;

    use super::*;

    #[test]
    fn renders_offscreen_cad_viewport_frame_with_shaded_faces_and_edges() {
        let step = sample_machined_plate_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("sample.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();

        let mut cam = CadCamera::default();
        cam.fit_aabb(&doc.aabb_mm);

        let highlight = HighlightState {
            hovered: None,
            selected: Some(PickTarget::Face(FaceId(0))),
            show_grid: true,
        };

        let png = render_offscreen_png(&doc, &cam, &highlight, 640, 480).unwrap();
        assert!(png.starts_with(b"\x89PNG\r\n\x1a\n"));
        assert!(
            png.len() > 8_000,
            "expected rich shaded CAD PNG (>8KB), got {} bytes",
            png.len()
        );
    }
}
