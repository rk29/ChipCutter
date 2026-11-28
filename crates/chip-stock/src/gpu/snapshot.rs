//! Pure read-only GPU storage-buffer snapshots (`GpuStockBufferSnapshot`) exposing immutable
//! `Arc<wgpu::Buffer>` handles and grid metadata for `Heightfield`, `Cylindrical`, and `TriDexel`
//! stock states without any viewport shaders, surface-extraction quads, or presentation color state.

use std::sync::Arc;

use wgpu::util::DeviceExt as _;

use crate::{
    backend::SimulatedStock,
    tridexel::{FlatDexelSpan, INLINE_DEXEL_SLOTS, TriDexelStock},
    types::{Axis3, RotaryStockSpec},
};

use super::context::GpuSimContext;

/// Read-only GPU storage-buffer snapshot & grid metadata for a 2.5D Heightfield stock.
#[derive(Clone)]
pub struct GpuHeightfieldBuffers {
    pub nx: u32,
    pub ny: u32,
    pub origin_wcs: [f32; 3],
    pub dx_mm: f32,
    pub dy_mm: f32,
    pub has_target: bool,
    pub z_stock_buf: Arc<wgpu::Buffer>,
    pub z_bottom_buf: Arc<wgpu::Buffer>,
    pub z_target_buf: Arc<wgpu::Buffer>,
    pub last_op_buf: Arc<wgpu::Buffer>,
}

impl std::fmt::Debug for GpuHeightfieldBuffers {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuHeightfieldBuffers")
            .field("nx", &self.nx)
            .field("ny", &self.ny)
            .field("has_target", &self.has_target)
            .finish()
    }
}

/// Read-only GPU storage-buffer snapshot & grid metadata for a 4th-Axis Cylindrical stock.
#[derive(Clone)]
pub struct GpuCylindricalBuffers {
    pub nu: u32,
    pub ntheta: u32,
    pub rotary: RotaryStockSpec,
    pub u_min: f64,
    pub u_max: f64,
    pub du: f64,
    pub dtheta: f64,
    pub has_target: bool,
    pub r_stock_buf: Arc<wgpu::Buffer>,
    pub r_target_buf: Arc<wgpu::Buffer>,
    pub last_op_buf: Arc<wgpu::Buffer>,
}

impl std::fmt::Debug for GpuCylindricalBuffers {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuCylindricalBuffers")
            .field("nu", &self.nu)
            .field("ntheta", &self.ntheta)
            .field("has_target", &self.has_target)
            .finish()
    }
}

/// CPU-side Tri-Dexel state retained ONLY when rare rays overflow the `K = 4` inline GPU slots,
/// allowing presentation consumers to emit a tiny supplement mesh for those overflow rays.
#[derive(Debug, Clone)]
pub struct TriDexelOverflowData {
    pub stock: TriDexelStock,
    pub target: Option<TriDexelStock>,
}

/// Read-only GPU storage-buffer snapshot & grid metadata for a 3D Tri-Dexel stock.
#[derive(Clone)]
pub struct GpuTriDexelBuffers {
    pub dims: [u32; 3],
    pub origin_wcs: [f32; 3],
    pub pitch_mm: [f32; 3],
    pub plane_ray_offsets: [u32; 3],
    pub total_rays: u32,
    pub has_target: bool,
    pub m_slots_buf: Arc<wgpu::Buffer>,
    pub m_counts_buf: Arc<wgpu::Buffer>,
    pub t_slots_buf: Arc<wgpu::Buffer>,
    pub t_counts_buf: Arc<wgpu::Buffer>,
    /// Present ONLY when at least one ray overflows `K = 4` inline slots on the CPU shadow copy.
    pub overflow_data: Option<Arc<TriDexelOverflowData>>,
}

impl std::fmt::Debug for GpuTriDexelBuffers {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuTriDexelBuffers")
            .field("dims", &self.dims)
            .field("total_rays", &self.total_rays)
            .field("has_target", &self.has_target)
            .field("has_overflow_data", &self.overflow_data.is_some())
            .finish()
    }
}

/// Polymorphic read-only GPU storage-buffer snapshot published by [`super::WgpuComputeBackend`]
/// (or uploaded from a CPU [`SimulatedStock`]) for downstream presentation/inspection layers.
#[derive(Debug, Clone)]
pub enum GpuStockBufferSnapshot {
    Heightfield(GpuHeightfieldBuffers),
    Cylindrical(GpuCylindricalBuffers),
    TriDexel(GpuTriDexelBuffers),
}

impl GpuStockBufferSnapshot {
    /// Uploads flat stock buffers from a CPU [`SimulatedStock`] snapshot directly onto `ctx`'s
    /// `wgpu::Device` without performing any CPU mesh tessellation or surface-emission compute.
    pub fn from_simulated_stock(
        ctx: &GpuSimContext,
        stock: &SimulatedStock,
        target_tridexel: Option<&TriDexelStock>,
    ) -> Arc<Self> {
        let device = ctx.device();
        match stock {
            SimulatedStock::Heightfield(hf) => {
                let z_stock_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_hf_snap_z_stock"),
                        contents: bytemuck::cast_slice(&hf.z_stock),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let z_bottom_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_hf_snap_z_bottom"),
                        contents: bytemuck::cast_slice(&hf.z_bottom),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let z_target_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_hf_snap_z_target"),
                        contents: bytemuck::cast_slice(&hf.z_target),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let last_op_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_hf_snap_last_op"),
                        contents: bytemuck::cast_slice(&hf.last_op_idx),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let has_target = hf.z_target.iter().any(|z| z.is_finite());
                Arc::new(Self::Heightfield(GpuHeightfieldBuffers {
                    nx: hf.nx as u32,
                    ny: hf.ny as u32,
                    origin_wcs: [
                        hf.origin_wcs.x as f32,
                        hf.origin_wcs.y as f32,
                        hf.origin_wcs.z as f32,
                    ],
                    dx_mm: hf.dx_mm as f32,
                    dy_mm: hf.dy_mm as f32,
                    has_target,
                    z_stock_buf,
                    z_bottom_buf,
                    z_target_buf,
                    last_op_buf,
                }))
            }
            SimulatedStock::Cylindrical(cyl) => {
                let r_stock_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_cyl_snap_r_stock"),
                        contents: bytemuck::cast_slice(&cyl.r_stock),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let r_target_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_cyl_snap_r_target"),
                        contents: bytemuck::cast_slice(&cyl.r_target),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let last_op_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_cyl_snap_last_op"),
                        contents: bytemuck::cast_slice(&cyl.last_op_idx),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let has_target = cyl.r_target.iter().any(|&r| r >= 0.0 && r.is_finite());
                Arc::new(Self::Cylindrical(GpuCylindricalBuffers {
                    nu: cyl.nu as u32,
                    ntheta: cyl.ntheta as u32,
                    rotary: cyl.rotary,
                    u_min: cyl.u_min,
                    u_max: cyl.u_max,
                    du: cyl.du,
                    dtheta: cyl.dtheta,
                    has_target,
                    r_stock_buf,
                    r_target_buf,
                    last_op_buf,
                }))
            }
            SimulatedStock::TriDexel(td) => {
                let rx = td.planes[0].ray_count();
                let ry = td.planes[1].ray_count();
                let rz = td.planes[2].ray_count();
                let plane_ray_offsets = [0u32, rx as u32, (rx + ry) as u32];
                let total_rays = rx + ry + rz;
                let total_slots = total_rays * INLINE_DEXEL_SLOTS;

                let mut all_m_slots = Vec::with_capacity(total_slots.max(1));
                let mut all_m_counts = Vec::with_capacity(total_rays.max(1));
                for axis in Axis3::ALL {
                    let p = td.plane(axis);
                    all_m_slots.extend_from_slice(&p.slots);
                    all_m_counts.extend_from_slice(&p.counts);
                }
                if all_m_slots.is_empty() {
                    all_m_slots.push(FlatDexelSpan::ZERO);
                }
                if all_m_counts.is_empty() {
                    all_m_counts.push(0);
                }

                let m_slots_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_td_snap_m_slots"),
                        contents: bytemuck::cast_slice(&all_m_slots),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));
                let m_counts_buf = Arc::new(device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some("chip_td_snap_m_counts"),
                        contents: bytemuck::cast_slice(&all_m_counts),
                        usage: wgpu::BufferUsages::STORAGE,
                    },
                ));

                let has_target = target_tridexel.is_some();
                let (t_slots_buf, t_counts_buf) = if let Some(tgt) = target_tridexel {
                    let mut all_t_slots = Vec::with_capacity(total_slots.max(1));
                    let mut all_t_counts = Vec::with_capacity(total_rays.max(1));
                    for axis in Axis3::ALL {
                        let p = tgt.plane(axis);
                        all_t_slots.extend_from_slice(&p.slots);
                        all_t_counts.extend_from_slice(&p.counts);
                    }
                    if all_t_slots.is_empty() {
                        all_t_slots.push(FlatDexelSpan::ZERO);
                    }
                    if all_t_counts.is_empty() {
                        all_t_counts.push(0);
                    }
                    (
                        Arc::new(
                            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                                label: Some("chip_td_snap_t_slots"),
                                contents: bytemuck::cast_slice(&all_t_slots),
                                usage: wgpu::BufferUsages::STORAGE,
                            }),
                        ),
                        Arc::new(
                            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                                label: Some("chip_td_snap_t_counts"),
                                contents: bytemuck::cast_slice(&all_t_counts),
                                usage: wgpu::BufferUsages::STORAGE,
                            }),
                        ),
                    )
                } else {
                    let dummy_slots = [FlatDexelSpan::ZERO];
                    let dummy_counts = [0u32; 4];
                    (
                        Arc::new(
                            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                                label: Some("chip_td_snap_dummy_t_slots"),
                                contents: bytemuck::cast_slice(&dummy_slots),
                                usage: wgpu::BufferUsages::STORAGE,
                            }),
                        ),
                        Arc::new(
                            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                                label: Some("chip_td_snap_dummy_t_counts"),
                                contents: bytemuck::cast_slice(&dummy_counts),
                                usage: wgpu::BufferUsages::STORAGE,
                            }),
                        ),
                    )
                };

                let has_overflow = td.planes.iter().any(|p| !p.overflow.is_empty());
                let overflow_data = has_overflow.then(|| {
                    Arc::new(TriDexelOverflowData {
                        stock: (**td).clone(),
                        target: target_tridexel.cloned(),
                    })
                });

                Arc::new(Self::TriDexel(GpuTriDexelBuffers {
                    dims: [td.dims[0] as u32, td.dims[1] as u32, td.dims[2] as u32],
                    origin_wcs: [
                        td.origin_wcs.x as f32,
                        td.origin_wcs.y as f32,
                        td.origin_wcs.z as f32,
                    ],
                    pitch_mm: [
                        td.pitch_mm.x as f32,
                        td.pitch_mm.y as f32,
                        td.pitch_mm.z as f32,
                    ],
                    plane_ray_offsets,
                    total_rays: total_rays as u32,
                    has_target,
                    m_slots_buf,
                    m_counts_buf,
                    t_slots_buf,
                    t_counts_buf,
                    overflow_data,
                }))
            }
        }
    }

    /// Returns `true` if target comparison data is present on the GPU.
    #[inline]
    pub const fn has_target(&self) -> bool {
        match self {
            Self::Heightfield(h) => h.has_target,
            Self::Cylindrical(c) => c.has_target,
            Self::TriDexel(t) => t.has_target,
        }
    }
}
