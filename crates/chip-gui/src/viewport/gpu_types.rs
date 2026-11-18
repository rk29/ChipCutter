use bytemuck::{Pod, Zeroable};
use chip_sim::SimColoredVertex;
use wgpu::util::DeviceExt as _;

pub(crate) const SAMPLE_COUNT: u32 = 4;
pub(crate) const COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
pub(crate) const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub(crate) struct ViewportUniformsGpu {
    pub(crate) view_proj: [[f32; 4]; 4],
    pub(crate) inv_view_proj: [[f32; 4]; 4],
    pub(crate) view: [[f32; 4]; 4],
    pub(crate) wcs_world_to_local: [[f32; 4]; 4],
    pub(crate) wcs_local_to_world: [[f32; 4]; 4],
    pub(crate) camera_pos: [f32; 4],
    pub(crate) viewport_and_grid: [f32; 4],
    pub(crate) pick_flags: [u32; 4],
    pub(crate) sim_tool_tip_wcs: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub(crate) struct SimVertexGpu {
    pub(crate) pos_wcs_mm: [f32; 3],
    pub(crate) normal_wcs: [f32; 3],
    pub(crate) color_rgba: [f32; 4],
}

impl SimVertexGpu {
    const ATTRIBS: [wgpu::VertexAttribute; 3] = wgpu::vertex_attr_array![
        0 => Float32x3,
        1 => Float32x3,
        2 => Float32x4,
    ];

    pub(crate) fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBS,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub(crate) struct SolidVertexGpu {
    pub(crate) pos_mm: [f32; 3],
    pub(crate) normal: [f32; 3],
    pub(crate) face_id: u32,
}

impl SolidVertexGpu {
    const ATTRIBS: [wgpu::VertexAttribute; 3] = wgpu::vertex_attr_array![
        0 => Float32x3,
        1 => Float32x3,
        2 => Uint32,
    ];

    pub(crate) fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBS,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub(crate) struct OverlayVertexGpu {
    pub(crate) pos_mm: [f32; 3],
    pub(crate) color: [f32; 4],
}

impl OverlayVertexGpu {
    const ATTRIBS: [wgpu::VertexAttribute; 2] = wgpu::vertex_attr_array![
        0 => Float32x3,
        1 => Float32x4,
    ];

    pub(crate) fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBS,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub(crate) struct EdgeInstanceGpu {
    pub(crate) p0_mm: [f32; 3],
    pub(crate) p1_mm: [f32; 3],
    pub(crate) color: [f32; 4],
    pub(crate) width_px: f32,
}

impl EdgeInstanceGpu {
    const ATTRIBS: [wgpu::VertexAttribute; 4] = wgpu::vertex_attr_array![
        0 => Float32x3,
        1 => Float32x3,
        2 => Float32x4,
        3 => Float32,
    ];

    pub(crate) fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as u64,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &Self::ATTRIBS,
        }
    }
}

/// Reusable dynamically-grown `wgpu::Buffer` that eliminates per-frame buffer allocation churn.
pub(crate) struct GrowableGpuBuffer {
    buf: Option<wgpu::Buffer>,
    capacity_bytes: u64,
    count: u32,
    usage: wgpu::BufferUsages,
    label: &'static str,
}

impl GrowableGpuBuffer {
    pub(crate) fn new(label: &'static str, usage: wgpu::BufferUsages) -> Self {
        Self {
            buf: None,
            capacity_bytes: 0,
            count: 0,
            usage: usage | wgpu::BufferUsages::COPY_DST,
            label,
        }
    }

    pub(crate) fn upload_slice<T: Pod>(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        items: &[T],
    ) {
        if items.is_empty() {
            self.count = 0;
            return;
        }
        let bytes: &[u8] = bytemuck::cast_slice(items);
        let needed = bytes.len() as u64;
        if self.buf.is_none() || needed > self.capacity_bytes {
            let cap = needed.next_power_of_two().max(256);
            let new_buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(self.label),
                size: cap,
                usage: self.usage | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.buf = Some(new_buf);
            self.capacity_bytes = cap;
        }
        if let Some(buf) = &self.buf {
            queue.write_buffer(buf, 0, bytes);
            self.count = items.len() as u32;
        }
    }

    pub(crate) fn active(&self) -> Option<(&wgpu::Buffer, u32)> {
        self.buf
            .as_ref()
            .filter(|_| self.count > 0)
            .map(|b| (b, self.count))
    }

    #[cfg(test)]
    pub(crate) fn capacity_bytes(&self) -> u64 {
        self.capacity_bytes
    }
}

pub(crate) struct FrameTargets {
    pub(crate) width: u32,
    pub(crate) height: u32,
    _msaa_color: wgpu::Texture,
    pub(crate) msaa_color_view: wgpu::TextureView,
    _msaa_depth: wgpu::Texture,
    pub(crate) msaa_depth_view: wgpu::TextureView,
    pub(crate) resolved_color: wgpu::Texture,
    pub(crate) resolved_color_view: wgpu::TextureView,
}

impl FrameTargets {
    pub(crate) fn create(device: &wgpu::Device, width: u32, height: u32) -> Self {
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

pub(crate) struct UploadedCadGpu {
    pub(crate) doc_revision: u64,
    pub(crate) vertex_buf: wgpu::Buffer,
    pub(crate) index_buf: wgpu::Buffer,
    pub(crate) index_count: u32,
}

pub(crate) struct UploadedToolpathsGpu {
    pub(crate) signature_hash: u64,
    pub(crate) buf: Option<(wgpu::Buffer, u32)>,
}

pub(crate) struct UploadedSimIndexedMeshGpu {
    pub(crate) revision: u64,
    pub(crate) vertex_buf: wgpu::Buffer,
    pub(crate) index_buf: wgpu::Buffer,
    pub(crate) index_count: u32,
}

pub(crate) fn upload_sim_indexed_mesh(
    device: &wgpu::Device,
    revision: u64,
    vertices: &[SimColoredVertex],
    indices: &[u32],
    label: &'static str,
) -> Option<UploadedSimIndexedMeshGpu> {
    if vertices.is_empty() || indices.is_empty() {
        return None;
    }
    let gpu_verts: Vec<SimVertexGpu> = vertices
        .iter()
        .map(|v| SimVertexGpu {
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
