//! Shared or headless `wgpu` compute context (`GpuSimContext`) caching compiled WGSL compute
//! pipelines and bind group layouts for `Heightfield` and `Cylindrical` sweepers.

use std::sync::{Arc, OnceLock, mpsc};

use crate::types::ActiveSimBackendInfo;

/// Reusable `wgpu` compute device, queue, and pre-compiled stock simulation pipelines.
///
/// Can be constructed either from an existing shared `eframe`/`ViewportRenderer` device
/// ([`Self::from_shared`]) or initialized headless ([`Self::try_new_headless`]).
pub struct GpuSimContext {
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
    adapter_info: ActiveSimBackendInfo,
    pub(crate) heightfield_bgl: wgpu::BindGroupLayout,
    pub(crate) heightfield_pipeline: wgpu::ComputePipeline,
    pub(crate) cylindrical_bgl: wgpu::BindGroupLayout,
    pub(crate) cylindrical_pipeline: wgpu::ComputePipeline,
}

impl std::fmt::Debug for GpuSimContext {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuSimContext")
            .field("adapter_info", &self.adapter_info)
            .finish()
    }
}

impl GpuSimContext {
    /// Creates a [`GpuSimContext`] sharing an existing `wgpu::Device` & `wgpu::Queue`
    /// (e.g., from `eframe`/`ViewportRenderer`) and pre-compiles all compute pipelines.
    pub fn from_shared(
        device: Arc<wgpu::Device>,
        queue: Arc<wgpu::Queue>,
        adapter_info: ActiveSimBackendInfo,
    ) -> Self {
        let (heightfield_bgl, heightfield_pipeline) = build_heightfield_pipeline(&device);
        let (cylindrical_bgl, cylindrical_pipeline) = build_cylindrical_pipeline(&device);

        Self {
            device,
            queue,
            adapter_info,
            heightfield_bgl,
            heightfield_pipeline,
            cylindrical_bgl,
            cylindrical_pipeline,
        }
    }

    /// Attempts to initialize a headless `wgpu` compute context, respecting `WGPU_BACKEND` and
    /// `WGPU_POWER_PREF`. Tries hardware adapters first and falls back to software adapters
    /// (e.g. Mesa `lavapipe`) if no hardware adapter is available. Returns `None` cleanly if no
    /// usable compute adapter exists.
    pub fn try_new_headless() -> Option<Arc<Self>> {
        let instance =
            wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
        let power_preference =
            wgpu::PowerPreference::from_env().unwrap_or(wgpu::PowerPreference::HighPerformance);

        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference,
            compatible_surface: None,
            force_fallback_adapter: false,
            apply_limit_buckets: false,
        }))
        .or_else(|_| {
            pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
                power_preference,
                compatible_surface: None,
                force_fallback_adapter: true,
                apply_limit_buckets: false,
            }))
        })
        .ok()?;

        let raw_info = adapter.get_info();
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("chip_stock_gpu_compute"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::default(),
            experimental_features: wgpu::ExperimentalFeatures::disabled(),
            memory_hints: wgpu::MemoryHints::Performance,
            trace: wgpu::Trace::Off,
        }))
        .ok()?;

        let adapter_info = ActiveSimBackendInfo {
            is_gpu: true,
            adapter_name: raw_info.name,
            backend_api: format!("{:?}", raw_info.backend),
        };

        Some(Arc::new(Self::from_shared(
            Arc::new(device),
            Arc::new(queue),
            adapter_info,
        )))
    }

    /// Returns a process-wide lazily-initialized headless [`GpuSimContext`] so repeated
    /// CLI/test simulations reuse the compiled Vulkan/Metal/DX12 device & pipelines.
    pub fn shared_headless() -> Option<Arc<Self>> {
        static SHARED: OnceLock<Option<Arc<GpuSimContext>>> = OnceLock::new();
        SHARED.get_or_init(Self::try_new_headless).clone()
    }

    #[inline]
    pub fn device(&self) -> &wgpu::Device {
        &self.device
    }

    #[inline]
    pub fn queue(&self) -> &wgpu::Queue {
        &self.queue
    }

    #[inline]
    pub fn device_arc(&self) -> Arc<wgpu::Device> {
        Arc::clone(&self.device)
    }

    #[inline]
    pub fn queue_arc(&self) -> Arc<wgpu::Queue> {
        Arc::clone(&self.queue)
    }

    #[inline]
    pub fn adapter_info(&self) -> &ActiveSimBackendInfo {
        &self.adapter_info
    }

    /// Synchronously maps `staging_buf` (after a copy command has been submitted to `queue`)
    /// and copies `byte_len` bytes into `dst`.
    pub(crate) fn readback_bytes(
        &self,
        staging_buf: &wgpu::Buffer,
        byte_len: usize,
        dst: &mut [u8],
    ) -> bool {
        if byte_len == 0 || dst.len() < byte_len {
            return false;
        }
        let slice = staging_buf.slice(0..byte_len as u64);
        let (tx, rx) = mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |res| {
            let _ = tx.send(res);
        });
        let _ = self.device.poll(wgpu::PollType::wait_indefinitely());
        let ok = match rx.recv() {
            Ok(Ok(())) => {
                if let Ok(mapped) = slice.get_mapped_range() {
                    dst[..byte_len].copy_from_slice(&mapped[..byte_len]);
                    true
                } else {
                    false
                }
            }
            _ => false,
        };
        staging_buf.unmap();
        ok
    }
}

fn storage_entry(binding: u32, read_only: bool) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

fn uniform_entry(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

fn build_heightfield_pipeline(
    device: &wgpu::Device,
) -> (wgpu::BindGroupLayout, wgpu::ComputePipeline) {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("chip_heightfield_sweep_wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("heightfield_sweep.wgsl").into()),
    });

    // 1 uniform + 8 storage buffers (within WebGPU default limit of 8 storage buffers/stage)
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("chip_hf_sweep_bgl"),
        entries: &[
            uniform_entry(0),        // HeightfieldUniforms
            storage_entry(1, false), // z_stock (rw)
            storage_entry(2, true),  // z_bottom (ro)
            storage_entry(3, true),  // z_target (ro)
            storage_entry(4, false), // last_op_idx (rw)
            storage_entry(5, true),  // profiles (ro)
            storage_entry(6, true),  // chords (ro)
            storage_entry(7, true),  // tile_data (ro: offsets + chord indices)
            storage_entry(8, false), // violations (rw atomic)
        ],
    });

    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("chip_hf_sweep_layout"),
        bind_group_layouts: &[Some(&bgl)],
        immediate_size: 0,
    });

    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("chip_hf_sweep_pipeline"),
        layout: Some(&layout),
        module: &shader,
        entry_point: Some("main"),
        compilation_options: wgpu::PipelineCompilationOptions::default(),
        cache: None,
    });

    (bgl, pipeline)
}

fn build_cylindrical_pipeline(
    device: &wgpu::Device,
) -> (wgpu::BindGroupLayout, wgpu::ComputePipeline) {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("chip_cylindrical_sweep_wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("cylindrical_sweep.wgsl").into()),
    });

    // 1 uniform + 7 storage buffers (within WebGPU default limit of 8 storage buffers/stage)
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("chip_cyl_sweep_bgl"),
        entries: &[
            uniform_entry(0),        // CylindricalUniforms
            storage_entry(1, false), // r_stock (rw)
            storage_entry(2, true),  // r_target (ro)
            storage_entry(3, false), // last_op_idx (rw)
            storage_entry(4, true),  // profiles (ro)
            storage_entry(5, true),  // poses (ro)
            storage_entry(6, true),  // tile_data (ro: offsets + pose indices)
            storage_entry(7, false), // violations (rw atomic)
        ],
    });

    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("chip_cyl_sweep_layout"),
        bind_group_layouts: &[Some(&bgl)],
        immediate_size: 0,
    });

    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("chip_cyl_sweep_pipeline"),
        layout: Some(&layout),
        module: &shader,
        entry_point: Some("main"),
        compilation_options: wgpu::PipelineCompilationOptions::default(),
        cache: None,
    });

    (bgl, pipeline)
}

/// Reusable GPU storage buffer that grows geometrically to avoid per-batch allocations.
pub(crate) struct GrowableStorageBuffer {
    label: &'static str,
    usage: wgpu::BufferUsages,
    buffer: wgpu::Buffer,
    capacity_bytes: usize,
}

impl GrowableStorageBuffer {
    pub(crate) fn new(
        device: &wgpu::Device,
        label: &'static str,
        usage: wgpu::BufferUsages,
        initial_bytes: usize,
    ) -> Self {
        let cap = initial_bytes.max(64);
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: cap as u64,
            usage: usage | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        Self {
            label,
            usage,
            buffer,
            capacity_bytes: cap,
        }
    }

    pub(crate) fn upload_slice<T: bytemuck::Pod>(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        data: &[T],
    ) -> bool {
        let bytes = bytemuck::cast_slice(data);
        let needed = bytes.len().max(16);
        let mut reallocated = false;
        if needed > self.capacity_bytes {
            let new_cap = needed.next_power_of_two();
            self.buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(self.label),
                size: new_cap as u64,
                usage: self.usage | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.capacity_bytes = new_cap;
            reallocated = true;
        }
        if !bytes.is_empty() {
            queue.write_buffer(&self.buffer, 0, bytes);
        }
        reallocated
    }

    #[inline]
    pub(crate) fn buffer(&self) -> &wgpu::Buffer {
        &self.buffer
    }
}
