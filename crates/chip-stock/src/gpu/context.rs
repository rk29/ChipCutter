//! Shared or headless `wgpu` compute context (`GpuSimContext`) caching compiled WGSL compute
//! pipelines and bind group layouts for `Heightfield`, `Cylindrical`, `TriDexel`, BVH target
//! rasterization/distance, and parallel reduction metrology.

use std::sync::{Arc, Mutex, OnceLock, mpsc};

use crate::types::ActiveSimBackendInfo;

use super::types::{
    GPU_METRICS_HIST_BINS, GPU_METRICS_MAX_WORKGROUPS, GpuMetricsPartial, MetricsUniforms,
};

pub(crate) struct MetricsScratchBuffers {
    pub(crate) uniform_buf: wgpu::Buffer,
    pub(crate) partials_buf: wgpu::Buffer,
    pub(crate) partials_readback_buf: wgpu::Buffer,
    pub(crate) hist_buf: wgpu::Buffer,
    pub(crate) hist_readback_buf: wgpu::Buffer,
    pub(crate) common_bg: wgpu::BindGroup,
}

/// Process-wide serialization lock for `wgpu::Device::create_compute_pipeline` / shader module
/// compilation. On Windows DX12 with FXC (`d3dcompiler_47.dll`), `wgpu-hal`'s shader cache shares
/// `ID3DBlob` objects whose COM `AddRef`/`Release` is non-atomic; serializing pipeline builds
/// prevents concurrent thread races inside FXC during lazy or multi-context initialization.
fn with_pipeline_compile_lock<R>(f: impl FnOnce() -> R) -> R {
    static PIPELINE_COMPILE_LOCK: Mutex<()> = Mutex::new(());
    let _guard = PIPELINE_COMPILE_LOCK
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    f()
}

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
    pub(crate) tridexel_bgl: wgpu::BindGroupLayout,
    pub(crate) tridexel_pipeline: wgpu::ComputePipeline,
    tridexel_oriented_pipeline: OnceLock<wgpu::ComputePipeline>,
    pub(crate) bvh_common_bgl: wgpu::BindGroupLayout,
    pub(crate) bvh_td_bgl: wgpu::BindGroupLayout,
    pub(crate) bvh_cyl_bgl: wgpu::BindGroupLayout,
    pub(crate) bvh_hf_bgl: wgpu::BindGroupLayout,
    pub(crate) bvh_closest_bgl: wgpu::BindGroupLayout,
    pub(crate) bvh_tridexel_pipeline: wgpu::ComputePipeline,
    pub(crate) bvh_cyl_pipeline: wgpu::ComputePipeline,
    pub(crate) bvh_hf_pipeline: wgpu::ComputePipeline,
    pub(crate) bvh_closest_pipeline: wgpu::ComputePipeline,
    pub(crate) metrics_hf_bgl: wgpu::BindGroupLayout,
    pub(crate) metrics_cyl_bgl: wgpu::BindGroupLayout,
    pub(crate) metrics_td_bgl: wgpu::BindGroupLayout,
    pub(crate) metrics_hf_pipeline: wgpu::ComputePipeline,
    pub(crate) metrics_cyl_pipeline: wgpu::ComputePipeline,
    pub(crate) metrics_td_pipeline: wgpu::ComputePipeline,
    pub(crate) metrics_scratch: Mutex<MetricsScratchBuffers>,
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
        let (
            (heightfield_bgl, heightfield_pipeline),
            (cylindrical_bgl, cylindrical_pipeline),
            (tridexel_bgl, tridexel_pipeline),
            bvh,
            met,
        ) = with_pipeline_compile_lock(|| {
            (
                build_heightfield_pipeline(&device),
                build_cylindrical_pipeline(&device),
                build_tridexel_pipeline(&device),
                build_bvh_pipelines(&device),
                build_metrics_pipelines(&device),
            )
        });

        Self {
            device,
            queue,
            adapter_info,
            heightfield_bgl,
            heightfield_pipeline,
            cylindrical_bgl,
            cylindrical_pipeline,
            tridexel_bgl,
            tridexel_pipeline,
            tridexel_oriented_pipeline: OnceLock::new(),
            bvh_common_bgl: bvh.common_bgl,
            bvh_td_bgl: bvh.td_bgl,
            bvh_cyl_bgl: bvh.cyl_bgl,
            bvh_hf_bgl: bvh.hf_bgl,
            bvh_closest_bgl: bvh.closest_bgl,
            bvh_tridexel_pipeline: bvh.td_pipeline,
            bvh_cyl_pipeline: bvh.cyl_pipeline,
            bvh_hf_pipeline: bvh.hf_pipeline,
            bvh_closest_pipeline: bvh.closest_pipeline,
            metrics_hf_bgl: met.hf_bgl,
            metrics_cyl_bgl: met.cyl_bgl,
            metrics_td_bgl: met.td_bgl,
            metrics_hf_pipeline: met.hf_pipeline,
            metrics_cyl_pipeline: met.cyl_pipeline,
            metrics_td_pipeline: met.td_pipeline,
            metrics_scratch: Mutex::new(met.scratch),
        }
    }

    /// Attempts to initialize a headless `wgpu` compute context, respecting `WGPU_BACKEND` and
    /// `WGPU_POWER_PREF`. Tries hardware adapters first and falls back to software adapters
    /// (e.g. Mesa `lavapipe`) if no hardware adapter is available. Returns `None` cleanly if no
    /// usable compute adapter exists.
    pub fn try_new_headless() -> Option<Arc<Self>> {
        Self::try_new_headless_with_backend().map(|(ctx, _)| ctx)
    }

    fn try_new_headless_with_backend() -> Option<(Arc<Self>, wgpu::Backend)> {
        // Serialize headless instance/adapter/device creation so concurrent first-use threads
        // (especially under EGL/Mesa `Backend::Gl`) never race during driver/display init.
        static INIT_LOCK: Mutex<()> = Mutex::new(());
        let _init_guard = INIT_LOCK.lock().unwrap_or_else(|e| e.into_inner());

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
        let backend = raw_info.backend;
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
            backend_api: format!("{backend:?}"),
        };

        Some((
            Arc::new(Self::from_shared(
                Arc::new(device),
                Arc::new(queue),
                adapter_info,
            )),
            backend,
        ))
    }

    /// Returns a lazily-initialized headless [`GpuSimContext`] so repeated CLI/test simulations
    /// reuse the compiled Vulkan/Metal/DX12/GL device & pipelines.
    ///
    /// On multi-thread-safe backends (`Vulkan`, `Metal`, `Dx12`), a single process-wide context is
    /// shared across threads. On OpenGL/GLES (`Backend::Gl`), `wgpu-hal` guards every device and
    /// queue operation behind a single `AdapterContext` mutex with a fixed timeout (`6 s` on EGL,
    /// `1 s` on WGL) that panics when another thread holds the GL context across a long compute
    /// dispatch (e.g. software `llvmpipe` Tri-Dexel sweeps on Linux CI). For `Backend::Gl`,
    /// contexts are therefore leased from a process-wide idle pool (identified via
    /// `Arc::strong_count == 1`) so concurrent workers and parallel tests never contend on a
    /// shared GL context while sequential calls still reuse compiled pipelines without leaking or
    /// touching TLS destructors at thread exit.
    pub fn shared_headless() -> Option<Arc<Self>> {
        enum SharedHeadless {
            None,
            ProcessWide(Arc<GpuSimContext>),
            PooledGl(Mutex<Vec<Arc<GpuSimContext>>>),
        }

        static SHARED: OnceLock<SharedHeadless> = OnceLock::new();

        let mode = SHARED.get_or_init(|| match Self::try_new_headless_with_backend() {
            None => SharedHeadless::None,
            Some((ctx, wgpu::Backend::Gl)) => SharedHeadless::PooledGl(Mutex::new(vec![ctx])),
            Some((ctx, _)) => SharedHeadless::ProcessWide(ctx),
        });

        match mode {
            SharedHeadless::None => None,
            SharedHeadless::ProcessWide(ctx) => Some(Arc::clone(ctx)),
            SharedHeadless::PooledGl(pool) => {
                {
                    let guard = pool.lock().unwrap_or_else(|e| e.into_inner());
                    if let Some(idle) = guard.iter().find(|ctx| {
                        Arc::strong_count(ctx) == 1
                            && Arc::strong_count(&ctx.device) == 1
                            && Arc::strong_count(&ctx.queue) == 1
                    }) {
                        return Some(Arc::clone(idle));
                    }
                }
                let created = Self::try_new_headless()?;
                let mut guard = pool.lock().unwrap_or_else(|e| e.into_inner());
                guard.push(Arc::clone(&created));
                Some(created)
            }
        }
    }

    #[inline]
    pub fn device(&self) -> &wgpu::Device {
        &self.device
    }

    #[inline]
    pub fn device_arc(&self) -> &Arc<wgpu::Device> {
        &self.device
    }

    #[inline]
    pub fn queue(&self) -> &wgpu::Queue {
        &self.queue
    }

    #[inline]
    pub fn adapter_info(&self) -> &ActiveSimBackendInfo {
        &self.adapter_info
    }

    /// Lazily compiles and caches the 4th-axis oriented-pose Tri-Dexel sweep pipeline
    /// (`main_oriented` in `tridexel_sweep.wgsl`) so 3-axis/indexed simulations only compile the
    /// compact cardinal sweeper (`main`).
    pub(crate) fn tridexel_oriented_pipeline(&self) -> &wgpu::ComputePipeline {
        self.tridexel_oriented_pipeline.get_or_init(|| {
            with_pipeline_compile_lock(|| {
                build_tridexel_oriented_pipeline(&self.device, &self.tridexel_bgl)
            })
        })
    }

    /// Synchronously maps `staging_buf` (after a copy command has been submitted to `queue`)
    /// and copies `byte_len` bytes into `dst`.
    pub fn readback_bytes(
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

    /// Synchronously maps up to 3 staging buffers with a single `device.poll(wait_indefinitely)`
    /// round-trip so multi-plane Tri-Dexel sweeps avoid 3× CPU-GPU synchronization stalls.
    pub(crate) fn readback_bytes_batch(
        &self,
        entries: &mut [(&wgpu::Buffer, usize, &mut [u8])],
    ) -> [bool; 3] {
        let mut results = [false; 3];
        let mut rxs: [Option<mpsc::Receiver<Result<(), wgpu::BufferAsyncError>>>; 3] =
            [None, None, None];
        let mut mapped_flags = [false; 3];

        for (idx, (buf, byte_len, dst)) in entries.iter().enumerate().take(3) {
            if *byte_len == 0 || dst.len() < *byte_len {
                continue;
            }
            let slice = buf.slice(0..*byte_len as u64);
            let (tx, rx) = mpsc::channel();
            slice.map_async(wgpu::MapMode::Read, move |res| {
                let _ = tx.send(res);
            });
            rxs[idx] = Some(rx);
            mapped_flags[idx] = true;
        }

        if mapped_flags.iter().any(|&m| m) {
            let _ = self.device.poll(wgpu::PollType::wait_indefinitely());
        }

        for (idx, (buf, byte_len, dst)) in entries.iter_mut().enumerate().take(3) {
            if !mapped_flags[idx] {
                continue;
            }
            if let Some(rx) = rxs[idx].take()
                && matches!(rx.recv(), Ok(Ok(())))
            {
                let slice = buf.slice(0..*byte_len as u64);
                if let Ok(mapped) = slice.get_mapped_range() {
                    dst[..*byte_len].copy_from_slice(&mapped[..*byte_len]);
                    results[idx] = true;
                }
            }
            buf.unmap();
        }

        results
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

fn build_tridexel_pipeline(
    device: &wgpu::Device,
) -> (wgpu::BindGroupLayout, wgpu::ComputePipeline) {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("chip_tridexel_sweep_wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("tridexel_sweep.wgsl").into()),
    });

    // 1 uniform + 8 storage buffers (strictly within WebGPU default limit of 8 storage buffers/stage)
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("chip_td_sweep_bgl"),
        entries: &[
            uniform_entry(0),        // TriDexelPlaneUniforms
            storage_entry(1, false), // slots (rw)
            storage_entry(2, false), // counts (rw)
            storage_entry(3, true),  // target_slots (ro)
            storage_entry(4, true),  // target_counts (ro)
            storage_entry(5, true),  // profiles (ro)
            storage_entry(6, true),  // chords (ro)
            storage_entry(7, true),  // tile_data (ro)
            storage_entry(8, false), // accum_and_overflow (rw atomic)
        ],
    });

    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("chip_td_sweep_layout"),
        bind_group_layouts: &[Some(&bgl)],
        immediate_size: 0,
    });

    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("chip_td_sweep_pipeline"),
        layout: Some(&layout),
        module: &shader,
        entry_point: Some("main"),
        compilation_options: wgpu::PipelineCompilationOptions::default(),
        cache: None,
    });

    (bgl, pipeline)
}

fn build_tridexel_oriented_pipeline(
    device: &wgpu::Device,
    bgl: &wgpu::BindGroupLayout,
) -> wgpu::ComputePipeline {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("chip_tridexel_sweep_oriented_wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("tridexel_sweep.wgsl").into()),
    });

    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("chip_td_sweep_oriented_layout"),
        bind_group_layouts: &[Some(bgl)],
        immediate_size: 0,
    });

    device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("chip_td_sweep_oriented_pipeline"),
        layout: Some(&layout),
        module: &shader,
        entry_point: Some("main_oriented"),
        compilation_options: wgpu::PipelineCompilationOptions::default(),
        cache: None,
    })
}

struct BuiltBvhPipelines {
    common_bgl: wgpu::BindGroupLayout,
    td_bgl: wgpu::BindGroupLayout,
    cyl_bgl: wgpu::BindGroupLayout,
    hf_bgl: wgpu::BindGroupLayout,
    closest_bgl: wgpu::BindGroupLayout,
    td_pipeline: wgpu::ComputePipeline,
    cyl_pipeline: wgpu::ComputePipeline,
    hf_pipeline: wgpu::ComputePipeline,
    closest_pipeline: wgpu::ComputePipeline,
}

fn build_bvh_pipelines(device: &wgpu::Device) -> BuiltBvhPipelines {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("chip_bvh_distance_wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("bvh_distance.wgsl").into()),
    });

    let common_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("chip_bvh_common_bgl"),
        entries: &[
            uniform_entry(0),       // BvhDispatchUniforms
            storage_entry(1, true), // bvh_nodes (ro)
            storage_entry(2, true), // gpu_tris (ro)
        ],
    });
    let td_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("chip_bvh_td_bgl"),
        entries: &[
            storage_entry(0, false), // td_slots (rw)
            storage_entry(1, false), // td_counts (rw)
            storage_entry(2, false), // td_overflow (rw atomic)
        ],
    });
    let cyl_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("chip_bvh_cyl_bgl"),
        entries: &[storage_entry(3, false)], // cyl_r_target (rw)
    });
    let hf_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("chip_bvh_hf_bgl"),
        entries: &[storage_entry(4, false)], // hf_z_target (rw)
    });
    let closest_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("chip_bvh_closest_bgl"),
        entries: &[
            storage_entry(5, true),  // query_points (ro)
            storage_entry(6, false), // closest_out (rw)
        ],
    });

    let make_pipe = |label: &str, entry: &str, group1: &wgpu::BindGroupLayout| {
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some(label),
            bind_group_layouts: &[Some(&common_bgl), Some(group1)],
            immediate_size: 0,
        });
        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(label),
            layout: Some(&layout),
            module: &shader,
            entry_point: Some(entry),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            cache: None,
        })
    };

    let td_pipeline = make_pipe("chip_bvh_td_pipe", "rasterize_tridexel_plane", &td_bgl);
    let cyl_pipeline = make_pipe(
        "chip_bvh_cyl_pipe",
        "rasterize_cylindrical_target",
        &cyl_bgl,
    );
    let hf_pipeline = make_pipe("chip_bvh_hf_pipe", "rasterize_heightfield_target", &hf_bgl);
    let closest_pipeline = make_pipe(
        "chip_bvh_closest_pipe",
        "closest_point_distance",
        &closest_bgl,
    );

    BuiltBvhPipelines {
        common_bgl,
        td_bgl,
        cyl_bgl,
        hf_bgl,
        closest_bgl,
        td_pipeline,
        cyl_pipeline,
        hf_pipeline,
        closest_pipeline,
    }
}

struct BuiltMetricsPipelines {
    hf_bgl: wgpu::BindGroupLayout,
    cyl_bgl: wgpu::BindGroupLayout,
    td_bgl: wgpu::BindGroupLayout,
    hf_pipeline: wgpu::ComputePipeline,
    cyl_pipeline: wgpu::ComputePipeline,
    td_pipeline: wgpu::ComputePipeline,
    scratch: MetricsScratchBuffers,
}

fn build_metrics_pipelines(device: &wgpu::Device) -> BuiltMetricsPipelines {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("chip_stock_metrics_wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("stock_metrics.wgsl").into()),
    });

    let common_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("chip_metrics_common_bgl"),
        entries: &[
            uniform_entry(0),        // MetricsUniforms
            storage_entry(1, false), // partials (rw)
            storage_entry(2, false), // global_hist (rw atomic)
        ],
    });
    let hf_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("chip_metrics_hf_bgl"),
        entries: &[
            storage_entry(0, true), // hf_z_stock (ro)
            storage_entry(1, true), // hf_z_bottom (ro)
            storage_entry(2, true), // hf_z_target (ro)
        ],
    });
    let cyl_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("chip_metrics_cyl_bgl"),
        entries: &[
            storage_entry(3, true), // cyl_r_stock (ro)
            storage_entry(4, true), // cyl_r_target (ro)
        ],
    });
    let td_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("chip_metrics_td_bgl"),
        entries: &[
            storage_entry(5, true), // td_m_slots (ro)
            storage_entry(6, true), // td_m_counts (ro)
            storage_entry(7, true), // td_t_slots (ro)
            storage_entry(8, true), // td_t_counts (ro)
        ],
    });

    let make_pipe = |label: &str, entry: &str, group1: &wgpu::BindGroupLayout| {
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some(label),
            bind_group_layouts: &[Some(&common_bgl), Some(group1)],
            immediate_size: 0,
        });
        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(label),
            layout: Some(&layout),
            module: &shader,
            entry_point: Some(entry),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            cache: None,
        })
    };

    let hf_pipeline = make_pipe("chip_metrics_hf_pipe", "reduce_heightfield", &hf_bgl);
    let cyl_pipeline = make_pipe("chip_metrics_cyl_pipe", "reduce_cylindrical", &cyl_bgl);
    let td_pipeline = make_pipe("chip_metrics_td_pipe", "reduce_tridexel", &td_bgl);

    let uniform_buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("chip_metrics_uniforms"),
        size: std::mem::size_of::<MetricsUniforms>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let partials_bytes =
        (GPU_METRICS_MAX_WORKGROUPS * std::mem::size_of::<GpuMetricsPartial>()) as u64;
    let partials_buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("chip_metrics_partials"),
        size: partials_bytes,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let partials_readback_buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("chip_metrics_partials_readback"),
        size: partials_bytes,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let hist_bytes = (GPU_METRICS_HIST_BINS * std::mem::size_of::<u32>()) as u64;
    let hist_buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("chip_metrics_hist"),
        size: hist_bytes,
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let hist_readback_buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("chip_metrics_hist_readback"),
        size: hist_bytes,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let common_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("chip_metrics_common_bg"),
        layout: &common_bgl,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: partials_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: hist_buf.as_entire_binding(),
            },
        ],
    });

    BuiltMetricsPipelines {
        hf_bgl,
        cyl_bgl,
        td_bgl,
        hf_pipeline,
        cyl_pipeline,
        td_pipeline,
        scratch: MetricsScratchBuffers {
            uniform_buf,
            partials_buf,
            partials_readback_buf,
            hist_buf,
            hist_readback_buf,
            common_bg,
        },
    }
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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use wgpu::naga::{Block, Statement, front::wgsl, valid};

    fn block_contains_return(block: &Block) -> bool {
        block.iter().any(|stmt| match stmt {
            Statement::Return { .. } => true,
            Statement::Block(inner) => block_contains_return(inner),
            Statement::If { accept, reject, .. } => {
                block_contains_return(accept) || block_contains_return(reject)
            }
            Statement::Switch { cases, .. } => cases.iter().any(|c| block_contains_return(&c.body)),
            Statement::Loop {
                body, continuing, ..
            } => block_contains_return(body) || block_contains_return(continuing),
            _ => false,
        })
    }

    fn switch_contains_return(block: &Block) -> bool {
        block.iter().any(|stmt| match stmt {
            Statement::Switch { cases, .. } => cases.iter().any(|c| block_contains_return(&c.body)),
            Statement::Block(inner) => switch_contains_return(inner),
            Statement::If { accept, reject, .. } => {
                switch_contains_return(accept) || switch_contains_return(reject)
            }
            Statement::Loop {
                body, continuing, ..
            } => switch_contains_return(body) || switch_contains_return(continuing),
            _ => false,
        })
    }

    #[derive(Default, Clone, Copy)]
    struct InlinedComplexity {
        loops: usize,
        max_depth: usize,
        switches: usize,
        stmts: usize,
        local_arrays: usize,
    }

    fn count_local_arrays(module: &wgpu::naga::Module, func: &wgpu::naga::Function) -> usize {
        func.local_variables
            .iter()
            .filter(|(_, lv)| {
                matches!(
                    module.types[lv.ty].inner,
                    wgpu::naga::TypeInner::Array { .. }
                )
            })
            .count()
    }

    fn inlined_complexity(
        module: &wgpu::naga::Module,
        block: &Block,
        depth: usize,
    ) -> InlinedComplexity {
        let mut out = InlinedComplexity {
            max_depth: depth,
            ..Default::default()
        };
        for stmt in block.iter() {
            if !matches!(stmt, Statement::Emit(_)) {
                out.stmts += 1;
            }
            match stmt {
                Statement::Block(inner) => {
                    let sub = inlined_complexity(module, inner, depth);
                    out.loops += sub.loops;
                    out.max_depth = out.max_depth.max(sub.max_depth);
                    out.switches += sub.switches;
                    out.stmts += sub.stmts;
                    out.local_arrays += sub.local_arrays;
                }
                Statement::If { accept, reject, .. } => {
                    let a = inlined_complexity(module, accept, depth);
                    let r = inlined_complexity(module, reject, depth);
                    out.loops += a.loops + r.loops;
                    out.max_depth = out.max_depth.max(a.max_depth).max(r.max_depth);
                    out.switches += a.switches + r.switches;
                    out.stmts += a.stmts + r.stmts;
                    out.local_arrays += a.local_arrays + r.local_arrays;
                }
                Statement::Switch { cases, .. } => {
                    out.switches += 1;
                    for case in cases {
                        let sub = inlined_complexity(module, &case.body, depth);
                        out.loops += sub.loops;
                        out.max_depth = out.max_depth.max(sub.max_depth);
                        out.switches += sub.switches;
                        out.stmts += sub.stmts;
                        out.local_arrays += sub.local_arrays;
                    }
                }
                Statement::Loop {
                    body, continuing, ..
                } => {
                    let next_depth = depth + 1;
                    let b = inlined_complexity(module, body, next_depth);
                    let c = inlined_complexity(module, continuing, next_depth);
                    out.loops += 1 + b.loops + c.loops;
                    out.max_depth = out
                        .max_depth
                        .max(next_depth)
                        .max(b.max_depth)
                        .max(c.max_depth);
                    out.switches += b.switches + c.switches;
                    out.stmts += b.stmts + c.stmts;
                    out.local_arrays += b.local_arrays + c.local_arrays;
                }
                Statement::Call { function, .. } => {
                    let callee = &module.functions[*function];
                    let sub = inlined_complexity(module, &callee.body, depth);
                    out.loops += sub.loops;
                    out.max_depth = out.max_depth.max(sub.max_depth);
                    out.switches += sub.switches;
                    out.stmts += sub.stmts;
                    out.local_arrays += count_local_arrays(module, callee) + sub.local_arrays;
                }
                _ => {}
            }
        }
        out
    }

    fn assert_fxc_safe_wgsl(label: &str, src: &str) {
        let module = wgsl::parse_str(src)
            .unwrap_or_else(|err| panic!("failed to parse WGSL shader `{label}`: {err}"));
        let mut validator =
            valid::Validator::new(valid::ValidationFlags::all(), valid::Capabilities::empty());
        validator
            .validate(&module)
            .unwrap_or_else(|err| panic!("failed to validate WGSL shader `{label}`: {err}"));

        let check_fn = |fn_name: &str, func: &wgpu::naga::Function| {
            if func.result.is_none() {
                return;
            }
            // FXC (D3DCompile SM 5.x) emits `error X3507: Not all control paths return a value`
            // when a non-void function ends in a `switch` or `if/else` chain without an
            // unconditional top-level `return`, or returns early inside `switch` cases.
            assert!(
                matches!(func.body.last(), Some(Statement::Return { value: Some(_) })),
                "WGSL shader `{label}` function `{fn_name}` must end with an unconditional \
                 top-level `return` statement for Windows FXC compatibility"
            );
            assert!(
                !switch_contains_return(&func.body),
                "WGSL shader `{label}` function `{fn_name}` contains a `return` inside a \
                 `switch` case, which can trigger FXC error X3507 on Windows"
            );
        };

        for (_, func) in module.functions.iter() {
            let fn_name = func.name.as_deref().unwrap_or("<unnamed>");
            check_fn(fn_name, func);
        }
        for ep in &module.entry_points {
            check_fn(&ep.name, &ep.function);
            // FXC (D3DCompile SM 5.0) inlines every user function and wraps each WGSL loop in
            // Naga's 64-bit `loop_bound` + `while(true)` scaffold. Deeply nested loops, heavily
            // duplicated switches/statements, or dynamically-indexed local arrays cause FXC to
            // spin for minutes or crash (`STATUS_INTEGER_DIVIDE_BY_ZERO` / `STATUS_ACCESS_VIOLATION`)
            // on Windows.
            let mut stats = inlined_complexity(&module, &ep.function.body, 0);
            stats.local_arrays += count_local_arrays(&module, &ep.function);
            let loop_cap = match label {
                "tridexel_emit.wgsl" => 12,
                "tridexel_sweep.wgsl" => 16,
                _ => 36,
            };
            let stmt_cap = match label {
                "tridexel_emit.wgsl" => 3_200,
                "tridexel_sweep.wgsl" => 1_800,
                _ => 1_800,
            };
            assert!(
                stats.max_depth <= 3,
                "WGSL shader `{label}` entry `{}` has inlined loop nesting depth {} (> 3), \
                 which can hang or crash Windows FXC (`d3dcompiler_47.dll`)",
                ep.name,
                stats.max_depth
            );
            assert!(
                stats.loops <= loop_cap,
                "WGSL shader `{label}` entry `{}` expands to {} inlined loops (> {loop_cap}), \
                 which can hang or crash Windows FXC (`d3dcompiler_47.dll`)",
                ep.name,
                stats.loops
            );
            assert!(
                stats.switches <= 12,
                "WGSL shader `{label}` entry `{}` expands to {} inlined switches (> 12), \
                 which can hang or crash Windows FXC (`d3dcompiler_47.dll`)",
                ep.name,
                stats.switches
            );
            assert!(
                stats.stmts <= stmt_cap,
                "WGSL shader `{label}` entry `{}` expands to {} inlined statements (> {stmt_cap}), \
                 which can hang or crash Windows FXC (`d3dcompiler_47.dll`)",
                ep.name,
                stats.stmts
            );
            if matches!(label, "tridexel_sweep.wgsl" | "tridexel_emit.wgsl") {
                assert_eq!(
                    stats.local_arrays, 0,
                    "WGSL shader `{label}` entry `{}` declares {} inlined function-local arrays \
                     (avoid FXC `dcl_indexableTemp` pressure)",
                    ep.name, stats.local_arrays
                );
            }
        }
    }

    #[test]
    fn all_compute_wgsl_shaders_have_fxc_safe_return_paths() {
        let shaders: [(&str, &str); 5] = [
            (
                "heightfield_sweep.wgsl",
                include_str!("heightfield_sweep.wgsl"),
            ),
            (
                "cylindrical_sweep.wgsl",
                include_str!("cylindrical_sweep.wgsl"),
            ),
            ("tridexel_sweep.wgsl", include_str!("tridexel_sweep.wgsl")),
            ("bvh_distance.wgsl", include_str!("bvh_distance.wgsl")),
            ("stock_metrics.wgsl", include_str!("stock_metrics.wgsl")),
        ];
        for (label, src) in shaders {
            assert_fxc_safe_wgsl(label, src);
        }
    }
}
