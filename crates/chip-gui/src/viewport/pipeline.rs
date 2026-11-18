use std::{
    collections::HashSet,
    hash::{Hash as _, Hasher as _},
    sync::Arc,
};

use bytemuck::{Pod, Zeroable};
use chip_cad::{CadDocument, EdgeId, FaceId, PickTarget, expand_coaxial_hole_faces};
use chip_cam::{
    BoxAnchor27, ChainDirectionArrow3D, GeneratedToolpath, MotionRole, ResolvedStockGeometry,
    SegmentMotion3D, StockAxis, TabMarker3D,
};
use chip_math::{Aabb3, DMat4, DVec3, DisplayUnit, MM_PER_INCH, RigidFrame3};
use chip_sim::{SimColoredVertex, SimStockMesh, SimToolMesh};
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
    wcs_world_to_local: [[f32; 4]; 4],
    wcs_local_to_world: [[f32; 4]; 4],
    camera_pos: [f32; 4],
    viewport_and_grid: [f32; 4],
    pick_flags: [u32; 4],
    sim_tool_tip_wcs: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct SimColoredVertexGpu {
    pos_wcs_mm: [f32; 3],
    normal_wcs: [f32; 3],
    color_rgba: [f32; 4],
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
struct OverlayVertexGpu {
    pos_mm: [f32; 3],
    color: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct EdgeInstanceGpu {
    p0_mm: [f32; 3],
    p1_mm: [f32; 3],
    color: [f32; 4],
    width_px: f32,
}

/// Visual Z-height reference plane rendered across the active Setup's stock XY footprint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZPlaneVisual {
    pub z_wcs: f64,
    pub fill_rgba: [f32; 4],
    pub border_rgba: [f32; 4],
}

/// Cached generated toolpath item queued for Pass #5 GPU-instanced rendering (`08-ui-viewport.md`).
#[derive(Debug, Clone)]
pub struct ToolpathRenderEntry {
    pub op_id: String,
    pub cache_key: [u8; 32],
    pub toolpath: Arc<GeneratedToolpath>,
    pub is_focused: bool,
}

/// Active hover, multi-selection, WCS, Stock, Z-plane, and Toolpath scene state fed into the 3D CAD/CAM viewport.
#[derive(Debug, Clone)]
pub struct HighlightState {
    pub hovered: Option<PickTarget>,
    pub selected: Option<PickTarget>,
    pub extra_selected_faces: HashSet<FaceId>,
    pub extra_selected_edges: HashSet<EdgeId>,
    pub chain_arrows: Vec<ChainDirectionArrow3D>,
    pub wcs_frame: RigidFrame3,
    pub stock: Option<ResolvedStockGeometry>,
    pub show_stock: bool,
    pub z_planes_wcs: Vec<ZPlaneVisual>,
    pub ghost_part: bool,
    pub show_grid: bool,
    pub toolpaths: Vec<ToolpathRenderEntry>,
    pub show_toolpaths: bool,
    pub show_rapids: bool,
    pub preview_tab_marker_wcs: Option<TabMarker3D>,
    /// Pass #6: Optional 3D machined `HeightfieldStock` mesh (`(mesh_rev, Arc<SimStockMesh>)`).
    pub sim_stock_mesh: Option<(u64, Arc<SimStockMesh>)>,
    /// Pass #6: Optional 3D revolved Flute+Shank+Holder mesh (`(tool_rev, Arc<SimToolMesh>)`) + WCS tip position.
    pub sim_tool_mesh: Option<(u64, Arc<SimToolMesh>, DVec3)>,
    /// Optional global motion segment limit for progressive toolpath trail during playback.
    pub sim_trail_limit: Option<usize>,
}

impl Default for HighlightState {
    fn default() -> Self {
        Self {
            hovered: None,
            selected: None,
            extra_selected_faces: HashSet::new(),
            extra_selected_edges: HashSet::new(),
            chain_arrows: Vec::new(),
            wcs_frame: RigidFrame3::IDENTITY,
            stock: None,
            show_stock: true,
            z_planes_wcs: Vec::new(),
            ghost_part: false,
            show_grid: true,
            toolpaths: Vec::new(),
            show_toolpaths: true,
            show_rapids: true,
            preview_tab_marker_wcs: None,
            sim_stock_mesh: None,
            sim_tool_mesh: None,
            sim_trail_limit: None,
        }
    }
}

impl HighlightState {
    fn build_face_states(&self, doc: Option<&CadDocument>) -> (u32, Vec<u32>) {
        let face_count = doc.map_or(64, |d| d.faces.len().max(64));
        let mut states = vec![0_u32; face_count];

        let mut hovered_fid = u32::MAX;
        if let Some(PickTarget::Face(fid)) = &self.hovered {
            hovered_fid = fid.0;
            if let Some(cad) = doc {
                for gfid in expand_coaxial_hole_faces(cad, *fid) {
                    if let Some(slot) = states.get_mut(gfid.0 as usize) {
                        *slot = 1;
                    }
                }
            } else if let Some(slot) = states.get_mut(fid.0 as usize) {
                *slot = 1;
            }
        }

        if let Some(PickTarget::Face(fid)) = &self.selected {
            if let Some(cad) = doc {
                for gfid in expand_coaxial_hole_faces(cad, *fid) {
                    if let Some(slot) = states.get_mut(gfid.0 as usize) {
                        *slot = 2;
                    }
                }
            } else if let Some(slot) = states.get_mut(fid.0 as usize) {
                *slot = 2;
            }
        }

        for &fid in &self.extra_selected_faces {
            if let Some(slot) = states.get_mut(fid.0 as usize) {
                *slot = 2;
            }
        }

        (hovered_fid, states)
    }

    fn collect_edge_sets(&self) -> (HashSet<EdgeId>, HashSet<EdgeId>) {
        let mut hov = HashSet::new();
        let mut sel = self.extra_selected_edges.clone();
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

struct UploadedToolpathsGpu {
    signature_hash: u64,
    buf: Option<(wgpu::Buffer, u32)>,
}

struct UploadedSimIndexedMeshGpu {
    revision: u64,
    vertex_buf: wgpu::Buffer,
    index_buf: wgpu::Buffer,
    index_count: u32,
}

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
    overlay_buf: Option<(wgpu::Buffer, u32)>,
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

        let solid_vbuf_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<SolidVertexGpu>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &wgpu::vertex_attr_array![
                0 => Float32x3,
                1 => Float32x3,
                2 => Uint32,
            ],
        };

        let solid_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("chip_solid_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_solid"),
                buffers: &[Some(solid_vbuf_layout.clone())],
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
                buffers: &[Some(solid_vbuf_layout)],
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
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<OverlayVertexGpu>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![
                        0 => Float32x3,
                        1 => Float32x4,
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

        let sim_vbuf_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<SimColoredVertexGpu>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &wgpu::vertex_attr_array![
                0 => Float32x3,
                1 => Float32x3,
                2 => Float32x4,
            ],
        };

        let sim_stock_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("chip_sim_stock_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_sim_stock"),
                buffers: &[Some(sim_vbuf_layout.clone())],
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
                buffers: &[Some(sim_vbuf_layout)],
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
            overlay_buf: None,
            edge_buf: None,
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

        // Build translucent overlay triangles + wireframe/edge/arrow instances
        let mut overlay_tris = Vec::new();
        let mut edge_instances = build_edge_and_scene_instances(
            doc.map(|(d, _)| d),
            highlight,
            camera,
            &mut overlay_tris,
        );

        if overlay_tris.is_empty() {
            self.overlay_buf = None;
        } else {
            let buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("chip_overlay_tris"),
                contents: bytemuck::cast_slice(&overlay_tris),
                usage: wgpu::BufferUsages::VERTEX,
            });
            self.overlay_buf = Some((buf, overlay_tris.len() as u32));
        }

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
        edge_instances.clear();

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

        let uniforms = ViewportUniformsGpu {
            view_proj: vp.to_cols_array_2d(),
            inv_view_proj: inv_vp.to_cols_array_2d(),
            view: view.to_cols_array_2d(),
            wcs_world_to_local: wcs_w2l,
            wcs_local_to_world: wcs_l2w,
            camera_pos: [eye.x, eye.y, eye.z, 1.0],
            viewport_and_grid: [w as f32, h as f32, minor_mm, major_mm],
            pick_flags: [
                hov_face,
                u32::from(highlight.ghost_part),
                u32::from(highlight.show_grid),
                u32::from(camera.projection.is_orthographic()),
            ],
            sim_tool_tip_wcs: tool_tip_wcs,
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
            if let Some((ov_buf, vert_count)) = &self.overlay_buf
                && *vert_count > 0
            {
                pass.set_pipeline(&self.overlay_pipeline);
                pass.set_vertex_buffer(0, ov_buf.slice(..));
                pass.draw(0..*vert_count, 0..1);
            }

            // 4. Topological edges, Stock wireframe, WCS triad, 3D chain arrows, & Tab bridges
            if let Some((edge_buf, count)) = &self.edge_buf
                && *count > 0
            {
                pass.set_pipeline(&self.edge_pipeline);
                pass.set_vertex_buffer(0, edge_buf.slice(..));
                pass.draw(0..6, 0..*count);
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

fn upload_sim_indexed_mesh(
    device: &wgpu::Device,
    revision: u64,
    vertices: &[SimColoredVertex],
    indices: &[u32],
    label: &'static str,
) -> Option<UploadedSimIndexedMeshGpu> {
    if vertices.is_empty() || indices.is_empty() {
        return None;
    }
    let gpu_verts: Vec<SimColoredVertexGpu> = vertices
        .iter()
        .map(|v| SimColoredVertexGpu {
            pos_wcs_mm: v.pos_mm,
            normal_wcs: v.normal,
            color_rgba: v.color_rgba,
        })
        .collect();
    let vertex_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some(label),
        contents: bytemuck::cast_slice(&gpu_verts),
        usage: wgpu::BufferUsages::VERTEX,
    });
    let index_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some(label),
        contents: bytemuck::cast_slice(indices),
        usage: wgpu::BufferUsages::INDEX,
    });
    Some(UploadedSimIndexedMeshGpu {
        revision,
        vertex_buf,
        index_buf,
        index_count: indices.len() as u32,
    })
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

fn dmat4_to_f32_2d(m: DMat4) -> [[f32; 4]; 4] {
    let cols = m.to_cols_array_2d();
    let mut out = [[0.0_f32; 4]; 4];
    for c in 0..4 {
        for r in 0..4 {
            out[c][r] = cols[c][r] as f32;
        }
    }
    out
}

fn dvec3_f32(v: DVec3) -> [f32; 3] {
    [v.x as f32, v.y as f32, v.z as f32]
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

fn build_edge_and_scene_instances(
    doc: Option<&CadDocument>,
    highlight: &HighlightState,
    camera: &CadCamera,
    overlay_tris: &mut Vec<OverlayVertexGpu>,
) -> Vec<EdgeInstanceGpu> {
    let mut out = Vec::new();
    let (hov_edges, sel_edges) = highlight.collect_edge_sets();

    // 1. B-Rep topological edges (hidden when solid machined stock replaces B-Rep unless ghost target is on)
    if (highlight.sim_stock_mesh.is_none() || highlight.ghost_part)
        && let Some(cad_doc) = doc
    {
        for poly in &cad_doc.render_mesh.edge_polylines {
            let (color, width_px) = if sel_edges.contains(&poly.edge_id) {
                ([1.0, 0.78, 0.18, 1.0], 3.4)
            } else if hov_edges.contains(&poly.edge_id) {
                ([0.28, 0.88, 1.0, 1.0], 3.1)
            } else {
                ([0.11, 0.13, 0.16, 0.90], 1.55)
            };

            for w in poly.points_mm.windows(2) {
                out.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(w[0]),
                    p1_mm: dvec3_f32(w[1]),
                    color,
                    width_px,
                });
            }
        }
    }

    // 2. Stock translucent shell + wireframe in active Setup WCS (when not replaced by 3D machined stock)
    if highlight.show_stock
        && highlight.sim_stock_mesh.is_none()
        && let Some(stock) = &highlight.stock
    {
        emit_stock_visuals(stock, &highlight.wcs_frame, overlay_tris, &mut out);
    }

    // 3. Translucent Z-Height reference planes across Stock XY footprint
    if !highlight.z_planes_wcs.is_empty() {
        let footprint = highlight.stock.map(|s| s.aabb_wcs()).unwrap_or_else(|| {
            Aabb3::new(DVec3::new(-40.0, -30.0, 0.0), DVec3::new(40.0, 30.0, 10.0))
        });
        let pad = 3.0;
        let x0 = footprint.min.x - pad;
        let x1 = footprint.max.x + pad;
        let y0 = footprint.min.y - pad;
        let y1 = footprint.max.y + pad;

        for plane in &highlight.z_planes_wcs {
            let z = plane.z_wcs;
            let p00 = highlight
                .wcs_frame
                .local_to_world_point(DVec3::new(x0, y0, z));
            let p10 = highlight
                .wcs_frame
                .local_to_world_point(DVec3::new(x1, y0, z));
            let p11 = highlight
                .wcs_frame
                .local_to_world_point(DVec3::new(x1, y1, z));
            let p01 = highlight
                .wcs_frame
                .local_to_world_point(DVec3::new(x0, y1, z));
            push_quad_tris(overlay_tris, p00, p10, p11, p01, plane.fill_rgba);
            for (a, b) in [(p00, p10), (p10, p11), (p11, p01), (p01, p00)] {
                out.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(a),
                    p1_mm: dvec3_f32(b),
                    color: plane.border_rgba,
                    width_px: 1.8,
                });
            }
        }
    }

    // 4. Active Setup WCS XYZ Origin Triad with 3D Arrowheads
    let triad_len = (camera.scene_radius_mm * 0.30).clamp(8.0, 48.0);
    let head = triad_len * 0.18;
    let origin_w = highlight.wcs_frame.origin_mm;
    let triad_axes = [
        (
            highlight.wcs_frame.x_axis,
            highlight.wcs_frame.y_axis,
            [0.96, 0.30, 0.30, 1.0],
        ),
        (
            highlight.wcs_frame.y_axis,
            highlight.wcs_frame.x_axis,
            [0.28, 0.90, 0.42, 1.0],
        ),
        (
            highlight.wcs_frame.z_axis,
            highlight.wcs_frame.x_axis,
            [0.34, 0.62, 1.0, 1.0],
        ),
    ];
    for (dir, side, color) in triad_axes {
        let tip = origin_w + dir * triad_len;
        out.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(origin_w),
            p1_mm: dvec3_f32(tip),
            color,
            width_px: 2.9,
        });
        let wing_a = tip - dir * head + side * (0.45 * head);
        let wing_b = tip - dir * head - side * (0.45 * head);
        out.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(tip),
            p1_mm: dvec3_f32(wing_a),
            color,
            width_px: 2.4,
        });
        out.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(tip),
            p1_mm: dvec3_f32(wing_b),
            color,
            width_px: 2.4,
        });
    }

    // 5. 3D Chain Travel Direction & Offset-Side Indicators
    let arrow_scale = (camera.scene_radius_mm * 0.14).clamp(4.0, 18.0);
    for arrow in &highlight.chain_arrows {
        let p0 = arrow.anchor_world_mm - arrow.tangent_world * (0.4 * arrow_scale);
        let p_tip = arrow.anchor_world_mm + arrow.tangent_world * (0.6 * arrow_scale);
        let dir_col = [0.18, 0.96, 0.62, 1.0];
        out.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(p0),
            p1_mm: dvec3_f32(p_tip),
            color: dir_col,
            width_px: 3.2,
        });
        let w_len = 0.35 * arrow_scale;
        for sign in [-1.0, 1.0] {
            let wing = p_tip - arrow.tangent_world * w_len
                + arrow.side_normal_world * (sign * 0.55 * w_len);
            out.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(p_tip),
                p1_mm: dvec3_f32(wing),
                color: dir_col,
                width_px: 2.6,
            });
        }
        // Perpendicular offset-side tick (amber/magenta) showing Left vs Right cutter compensation side
        let side_tip = arrow.anchor_world_mm + arrow.side_normal_world * (0.55 * arrow_scale);
        out.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(arrow.anchor_world_mm),
            p1_mm: dvec3_f32(side_tip),
            color: [1.0, 0.45, 0.78, 1.0],
            width_px: 2.6,
        });
    }

    // 6. 3D Holding Tab Bridge Markers (from generated toolpaths + live interactive cursor preview)
    if highlight.show_toolpaths {
        for entry in &highlight.toolpaths {
            let alpha_scale = if entry.is_focused { 1.0 } else { 0.45 };
            for tab in &entry.toolpath.tab_markers_wcs {
                emit_tab_marker_visual(
                    tab,
                    &highlight.wcs_frame,
                    false,
                    alpha_scale,
                    overlay_tris,
                    &mut out,
                );
            }
        }
    }
    if let Some(preview_tab) = &highlight.preview_tab_marker_wcs {
        emit_tab_marker_visual(
            preview_tab,
            &highlight.wcs_frame,
            true,
            1.0,
            overlay_tris,
            &mut out,
        );
    }

    out
}

fn emit_tab_marker_visual(
    tab: &TabMarker3D,
    wcs_frame: &RigidFrame3,
    is_hover_preview: bool,
    alpha_scale: f32,
    overlay_tris: &mut Vec<OverlayVertexGpu>,
    edges: &mut Vec<EdgeInstanceGpu>,
) {
    let tan = chip_math::DVec2::new(tab.tangent_xy.x, tab.tangent_xy.y).normalize_or_zero();
    let t2 = if tan.length_squared() < 0.2 {
        chip_math::DVec2::X
    } else {
        tan
    };
    let n2 = chip_math::DVec2::new(-t2.y, t2.x);
    let half_w = 0.5 * tab.width_mm.max(0.8);
    let half_t = 1.1; // visual bridge thickness across contour edge
    let z0 = tab.center_wcs.z;
    let z1 = z0 + tab.height_mm.max(0.3);

    let fill_col = if is_hover_preview {
        [0.24, 0.98, 0.72, 0.36 * alpha_scale]
    } else {
        [0.98, 0.76, 0.22, 0.28 * alpha_scale]
    };
    let wire_col = if is_hover_preview {
        [0.24, 0.98, 0.72, 0.98 * alpha_scale]
    } else {
        [1.0, 0.82, 0.26, 0.95 * alpha_scale]
    };

    let c2 = chip_math::DVec2::new(tab.center_wcs.x, tab.center_wcs.y);
    let to_w =
        |xy: chip_math::DVec2, z: f64| wcs_frame.local_to_world_point(DVec3::new(xy.x, xy.y, z));

    if tab.is_triangular {
        // Triangular prism bridge peaking at `(c2, z1)`
        let bl0 = to_w(c2 - t2 * half_w - n2 * half_t, z0);
        let bl1 = to_w(c2 - t2 * half_w + n2 * half_t, z0);
        let br0 = to_w(c2 + t2 * half_w - n2 * half_t, z0);
        let br1 = to_w(c2 + t2 * half_w + n2 * half_t, z0);
        let ap0 = to_w(c2 - n2 * half_t, z1);
        let ap1 = to_w(c2 + n2 * half_t, z1);

        push_quad_tris(overlay_tris, bl0, ap0, ap1, bl1, fill_col);
        push_quad_tris(overlay_tris, ap0, br0, br1, ap1, fill_col);
        for (a, b) in [
            (bl0, ap0),
            (ap0, br0),
            (bl1, ap1),
            (ap1, br1),
            (ap0, ap1),
            (bl0, bl1),
            (br0, br1),
        ] {
            edges.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(a),
                p1_mm: dvec3_f32(b),
                color: wire_col,
                width_px: 2.3,
            });
        }
    } else {
        // Rectangular bridge box
        let b00 = to_w(c2 - t2 * half_w - n2 * half_t, z0);
        let b10 = to_w(c2 + t2 * half_w - n2 * half_t, z0);
        let b11 = to_w(c2 + t2 * half_w + n2 * half_t, z0);
        let b01 = to_w(c2 - t2 * half_w + n2 * half_t, z0);
        let t00 = to_w(c2 - t2 * half_w - n2 * half_t, z1);
        let t10 = to_w(c2 + t2 * half_w - n2 * half_t, z1);
        let t11 = to_w(c2 + t2 * half_w + n2 * half_t, z1);
        let t01 = to_w(c2 - t2 * half_w + n2 * half_t, z1);

        push_quad_tris(overlay_tris, t00, t10, t11, t01, fill_col);
        for (a, b) in [
            (t00, t10),
            (t10, t11),
            (t11, t01),
            (t01, t00),
            (b00, t00),
            (b10, t10),
            (b11, t11),
            (b01, t01),
        ] {
            edges.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(a),
                p1_mm: dvec3_f32(b),
                color: wire_col,
                width_px: 2.3,
            });
        }
    }
}

fn compute_toolpath_gpu_signature(highlight: &HighlightState) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    highlight.show_toolpaths.hash(&mut h);
    highlight.show_rapids.hash(&mut h);
    highlight.sim_trail_limit.hash(&mut h);
    for v in [
        highlight.wcs_frame.origin_mm,
        highlight.wcs_frame.x_axis,
        highlight.wcs_frame.y_axis,
        highlight.wcs_frame.z_axis,
    ] {
        v.x.to_bits().hash(&mut h);
        v.y.to_bits().hash(&mut h);
        v.z.to_bits().hash(&mut h);
    }
    if highlight.show_toolpaths {
        highlight.toolpaths.len().hash(&mut h);
        for item in &highlight.toolpaths {
            item.op_id.hash(&mut h);
            item.cache_key.hash(&mut h);
            item.is_focused.hash(&mut h);
        }
    }
    h.finish()
}

fn build_toolpath_gpu_instances(highlight: &HighlightState) -> Vec<EdgeInstanceGpu> {
    if !highlight.show_toolpaths || highlight.toolpaths.is_empty() {
        return Vec::new();
    }

    let mut out = Vec::new();
    let frame = &highlight.wcs_frame;
    let mut global_seg_counter = 0usize;

    for entry in &highlight.toolpaths {
        let alpha = if entry.is_focused { 0.96_f32 } else { 0.36_f32 };
        let width_scale = if entry.is_focused { 1.0_f32 } else { 0.80_f32 };

        for seg in &entry.toolpath.segments_wcs {
            let cur_global = global_seg_counter;
            global_seg_counter += 1;
            if let Some(limit) = highlight.sim_trail_limit
                && cur_global > limit
            {
                continue;
            }
            let (color, width_px, arc_steps) = match seg.motion {
                SegmentMotion3D::Rapid => {
                    if !highlight.show_rapids {
                        continue;
                    }
                    ([0.98, 0.75, 0.16, 0.82 * alpha], 1.5 * width_scale, 1usize)
                }
                SegmentMotion3D::Linear { role, .. } => {
                    let (rgb, w) = role_color_and_width(role);
                    ([rgb[0], rgb[1], rgb[2], alpha], w * width_scale, 1usize)
                }
                SegmentMotion3D::Arc {
                    radius_mm,
                    signed_sweep_rad,
                    role,
                    ..
                } => {
                    let (rgb, w) = role_color_and_width(role);
                    // Discretize circular/helical arcs strictly on GPU upload to <= ~0.025 mm chord error
                    let step_angle = if radius_mm > 0.05 {
                        (2.0 * (1.0 - (0.025 / radius_mm).min(0.5)).acos()).clamp(0.06, 0.32)
                    } else {
                        0.25
                    };
                    let n = ((signed_sweep_rad.abs() / step_angle).ceil() as usize).clamp(4, 96);
                    ([rgb[0], rgb[1], rgb[2], alpha], w * width_scale, n)
                }
            };

            if arc_steps <= 1 {
                let p0 = frame.local_to_world_point(seg.start_wcs);
                let p1 = frame.local_to_world_point(seg.end_wcs);
                out.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(p0),
                    p1_mm: dvec3_f32(p1),
                    color,
                    width_px,
                });
            } else {
                let mut prev = frame.local_to_world_point(seg.start_wcs);
                for k in 1..=arc_steps {
                    let t = (k as f64) / (arc_steps as f64);
                    let cur = frame.local_to_world_point(seg.sample_point(t));
                    out.push(EdgeInstanceGpu {
                        p0_mm: dvec3_f32(prev),
                        p1_mm: dvec3_f32(cur),
                        color,
                        width_px,
                    });
                    prev = cur;
                }
            }
        }
    }

    out
}

fn role_color_and_width(role: MotionRole) -> ([f32; 3], f32) {
    match role {
        MotionRole::Cut => ([0.22, 0.74, 0.98], 2.25),
        MotionRole::FinishCut => ([0.14, 0.90, 0.94], 2.45),
        MotionRole::LeadIn | MotionRole::LeadOut => ([0.20, 0.84, 0.60], 1.95),
        MotionRole::Plunge | MotionRole::Ramp => ([0.98, 0.44, 0.52], 2.15),
        MotionRole::TabRamp => ([0.98, 0.80, 0.24], 2.35),
        MotionRole::LinkTransition => ([0.56, 0.78, 0.98], 1.75),
    }
}

fn push_quad_tris(
    tris: &mut Vec<OverlayVertexGpu>,
    p0: DVec3,
    p1: DVec3,
    p2: DVec3,
    p3: DVec3,
    color: [f32; 4],
) {
    for p in [p0, p1, p2, p0, p2, p3] {
        tris.push(OverlayVertexGpu {
            pos_mm: dvec3_f32(p),
            color,
        });
    }
}

fn emit_stock_visuals(
    stock: &ResolvedStockGeometry,
    wcs_frame: &RigidFrame3,
    overlay_tris: &mut Vec<OverlayVertexGpu>,
    edges: &mut Vec<EdgeInstanceGpu>,
) {
    let fill_col = [0.92, 0.74, 0.28, 0.085];
    let wire_col = [0.95, 0.78, 0.32, 0.62];

    match *stock {
        ResolvedStockGeometry::Box { aabb_wcs } => {
            if !aabb_wcs.is_valid() {
                return;
            }
            let min = aabb_wcs.min;
            let max = aabb_wcs.max;
            let corners_wcs = [
                DVec3::new(min.x, min.y, min.z),
                DVec3::new(max.x, min.y, min.z),
                DVec3::new(max.x, max.y, min.z),
                DVec3::new(min.x, max.y, min.z),
                DVec3::new(min.x, min.y, max.z),
                DVec3::new(max.x, min.y, max.z),
                DVec3::new(max.x, max.y, max.z),
                DVec3::new(min.x, max.y, max.z),
            ];
            let c: [DVec3; 8] = corners_wcs.map(|p| wcs_frame.local_to_world_point(p));

            let quads = [
                (0, 1, 2, 3),
                (4, 5, 6, 7),
                (0, 1, 5, 4),
                (1, 2, 6, 5),
                (2, 3, 7, 6),
                (3, 0, 4, 7),
            ];
            for (i0, i1, i2, i3) in quads {
                push_quad_tris(overlay_tris, c[i0], c[i1], c[i2], c[i3], fill_col);
            }

            let box_edges = [
                (0, 1),
                (1, 2),
                (2, 3),
                (3, 0),
                (4, 5),
                (5, 6),
                (6, 7),
                (7, 4),
                (0, 4),
                (1, 5),
                (2, 6),
                (3, 7),
            ];
            for (a, b) in box_edges {
                edges.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(c[a]),
                    p1_mm: dvec3_f32(c[b]),
                    color: wire_col,
                    width_px: 1.5,
                });
            }
        }
        ResolvedStockGeometry::Cylinder {
            axis,
            center_wcs,
            radius_mm,
            min_axial_mm,
            max_axial_mm,
            ..
        } => {
            const N: usize = 36;
            let map_cyl = |theta: f64, ax: f64| -> DVec3 {
                let (s, c) = theta.sin_cos();
                let u = radius_mm * c;
                let v = radius_mm * s;
                let p_wcs = match axis {
                    StockAxis::Z => DVec3::new(center_wcs.x + u, center_wcs.y + v, ax),
                    StockAxis::X => DVec3::new(ax, center_wcs.y + u, center_wcs.z + v),
                    StockAxis::Y => DVec3::new(center_wcs.x + u, ax, center_wcs.z + v),
                };
                wcs_frame.local_to_world_point(p_wcs)
            };

            for i in 0..N {
                let a0 = (i as f64) * std::f64::consts::TAU / (N as f64);
                let a1 = ((i + 1) as f64) * std::f64::consts::TAU / (N as f64);
                let b0 = map_cyl(a0, min_axial_mm);
                let b1 = map_cyl(a1, min_axial_mm);
                let t0 = map_cyl(a0, max_axial_mm);
                let t1 = map_cyl(a1, max_axial_mm);

                push_quad_tris(overlay_tris, b0, b1, t1, t0, fill_col);
                edges.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(b0),
                    p1_mm: dvec3_f32(b1),
                    color: wire_col,
                    width_px: 1.5,
                });
                edges.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(t0),
                    p1_mm: dvec3_f32(t1),
                    color: wire_col,
                    width_px: 1.5,
                });
                if i % 9 == 0 {
                    edges.push(EdgeInstanceGpu {
                        p0_mm: dvec3_f32(b0),
                        p1_mm: dvec3_f32(t0),
                        color: wire_col,
                        width_px: 1.3,
                    });
                }
            }
        }
    }
}

/// Interactive 27-point 3D anchor dot overlay for picking a Stock or Model WCS Origin directly
/// inside the 3D viewport (`03-setups-wcs.md`).
pub fn draw_wcs_origin_27pt_overlay(
    ui: &mut egui::Ui,
    viewport_rect: egui::Rect,
    camera: &CadCamera,
    wcs_frame: &RigidFrame3,
    box_wcs: &Aabb3,
    active_anchor: BoxAnchor27,
) -> Option<BoxAnchor27> {
    if !box_wcs.is_valid() {
        return None;
    }
    let painter = ui.painter_at(viewport_rect);
    let mut picked = None;

    for (idx, anchor) in BoxAnchor27::all_27().into_iter().enumerate() {
        let p_wcs = anchor.eval_on_aabb(box_wcs);
        let p_world = wcs_frame.local_to_world_point(p_wcs);
        let Some(screen_pt) = camera.project_world_to_screen(p_world, viewport_rect) else {
            continue;
        };
        if !viewport_rect.contains(screen_pt) {
            continue;
        }

        let hit_rect = egui::Rect::from_center_size(screen_pt, egui::vec2(14.0, 14.0));
        let resp = ui.interact(
            hit_rect,
            ui.scope_id().with(("wcs_anchor_27", idx)),
            egui::Sense::click(),
        );

        let is_active = anchor == active_anchor;
        let hovered = resp.hovered();
        let (radius, fill, stroke_col) = if is_active {
            (
                6.2,
                egui::Color32::from_rgb(255, 204, 56),
                egui::Color32::WHITE,
            )
        } else if hovered {
            (
                5.8,
                egui::Color32::from_rgb(92, 224, 255),
                egui::Color32::WHITE,
            )
        } else {
            (
                4.0,
                egui::Color32::from_rgba_unmultiplied(215, 225, 240, 175),
                egui::Color32::from_rgb(28, 34, 42),
            )
        };

        painter.circle(screen_pt, radius, fill, egui::Stroke::new(1.3, stroke_col));

        if resp
            .on_hover_text(format!("Set WCS Origin (X0 Y0 Z0): {}", anchor.label()))
            .clicked()
        {
            picked = Some(anchor);
        }
    }

    picked
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

    let center = egui::pos2(overlay_rect.center().x, overlay_rect.top() + 44.0);
    let (_, _, _, view_mat) = camera.gpu_matrices(100, 100);
    let tf = &camera.turntable_frame;
    let x_dir = glam::Vec3::new(tf.x_axis.x as f32, tf.x_axis.y as f32, tf.x_axis.z as f32);
    let y_dir = glam::Vec3::new(tf.y_axis.x as f32, tf.y_axis.y as f32, tf.y_axis.z as f32);
    let z_dir = glam::Vec3::new(tf.z_axis.x as f32, tf.z_axis.y as f32, tf.z_axis.z as f32);

    let axes = [
        (
            x_dir,
            "X",
            egui::Color32::from_rgb(242, 78, 78),
            StandardView::Right,
        ),
        (
            y_dir,
            "Y",
            egui::Color32::from_rgb(76, 218, 108),
            StandardView::Back,
        ),
        (
            z_dir,
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
    use chip_cad::{
        CadKernel, FaceId, MonstertruckKernel, PickTarget, sample_machined_plate_step,
        sample_multi_feature_bracket_step,
    };
    use chip_cam::{Setup, WorkOffsetSlot};
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
            MachineKinematicsConfig, SimOperationInput, SimResolution, SimVerificationConfig,
            build_cutter_assembly_mesh, run_setup_simulation, tessellate_stock_mesh,
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
        let sim_res = run_setup_simulation(
            &doc,
            &resolved,
            &setup.target_bodies,
            &sim_ops,
            SimResolution::Standard,
            MachineKinematicsConfig::default(),
            SimVerificationConfig::default(),
            None,
        )
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
}
