//! VRAM-bounded LRU GPU checkpoint cache (`GpuCheckpointCache`) for `< 1 ms` Ultra-resolution
//! timeline scrubbing with automatic spillover/fallback to CPU [`StockCheckpointSnapshot`]s.

use std::collections::{BTreeMap, HashMap};

use crate::{
    backend::StockCheckpointSnapshot,
    tridexel::{DexelSpan, FlatDexelSpan, INLINE_DEXEL_SLOTS},
    types::Axis3,
};

use super::{backend::WgpuComputeBackend, context::GpuSimContext};

/// Default GPU checkpoint cache VRAM budget (`256 MiB`).
pub const DEFAULT_GPU_CHECKPOINT_BUDGET_BYTES: usize = 256 * 1024 * 1024;

pub(crate) enum GpuCheckpointBuffers {
    Heightfield {
        z_stock: wgpu::Buffer,
        last_op_idx: wgpu::Buffer,
        cell_bytes: u64,
    },
    Cylindrical {
        r_stock: wgpu::Buffer,
        last_op_idx: wgpu::Buffer,
        cell_bytes: u64,
    },
    TriDexel {
        m_slots: wgpu::Buffer,
        m_counts: wgpu::Buffer,
        slots_bytes: u64,
        counts_bytes: u64,
        overflow: [BTreeMap<u32, Vec<DexelSpan>>; 3],
    },
}

struct GpuCheckpointEntry {
    byte_size: usize,
    last_used_tick: u64,
    buffers: GpuCheckpointBuffers,
}

/// VRAM-bounded LRU cache of GPU-resident stock checkpoints (`copy_buffer_to_buffer` restore).
///
/// Automatically evicts the least-recently-used GPU checkpoint whenever inserting a new snapshot
/// would exceed `budget_bytes`, transparently falling back to CPU [`StockCheckpointSnapshot`]
/// upload when an evicted checkpoint is requested again.
pub struct GpuCheckpointCache {
    budget_bytes: usize,
    used_bytes: usize,
    tick: u64,
    entries: HashMap<usize, GpuCheckpointEntry>,
    evictions: usize,
    hits: usize,
    misses: usize,
}

impl std::fmt::Debug for GpuCheckpointCache {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuCheckpointCache")
            .field("budget_bytes", &self.budget_bytes)
            .field("used_bytes", &self.used_bytes)
            .field("entries", &self.entries.len())
            .field("evictions", &self.evictions)
            .field("hits", &self.hits)
            .field("misses", &self.misses)
            .finish()
    }
}

impl Default for GpuCheckpointCache {
    fn default() -> Self {
        Self::new(DEFAULT_GPU_CHECKPOINT_BUDGET_BYTES)
    }
}

impl GpuCheckpointCache {
    /// Creates an empty GPU checkpoint cache bounded to at most `budget_bytes` of VRAM.
    pub fn new(budget_bytes: usize) -> Self {
        Self {
            budget_bytes,
            used_bytes: 0,
            tick: 0,
            entries: HashMap::new(),
            evictions: 0,
            hits: 0,
            misses: 0,
        }
    }

    #[inline]
    pub const fn budget_bytes(&self) -> usize {
        self.budget_bytes
    }

    #[inline]
    pub const fn used_bytes(&self) -> usize {
        self.used_bytes
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    #[inline]
    pub fn contains(&self, completed_segments: usize) -> bool {
        self.entries.contains_key(&completed_segments)
    }

    #[inline]
    pub const fn eviction_count(&self) -> usize {
        self.evictions
    }

    #[inline]
    pub const fn hit_count(&self) -> usize {
        self.hits
    }

    #[inline]
    pub const fn miss_count(&self) -> usize {
        self.misses
    }

    /// Releases all GPU checkpoint buffers immediately.
    pub fn clear(&mut self) {
        self.entries.clear();
        self.used_bytes = 0;
        self.tick = 0;
        self.evictions = 0;
        self.hits = 0;
        self.misses = 0;
    }

    /// Returns the VRAM footprint (`bytes`) required to store one checkpoint of `cp`.
    pub fn snapshot_byte_size(cp: &StockCheckpointSnapshot) -> usize {
        match cp {
            StockCheckpointSnapshot::Heightfield {
                z_stock,
                last_op_idx,
                ..
            } => z_stock.len() * 4 + last_op_idx.len() * 4,
            StockCheckpointSnapshot::Cylindrical {
                r_stock,
                last_op_idx,
                ..
            } => r_stock.len() * 4 + last_op_idx.len() * 4,
            StockCheckpointSnapshot::TriDexel { stock, .. } => {
                let rays: usize = stock.planes.iter().map(|p| p.ray_count()).sum();
                rays * INLINE_DEXEL_SLOTS * std::mem::size_of::<FlatDexelSpan>() + rays * 4
            }
        }
    }

    fn next_tick(&mut self) -> u64 {
        self.tick = self.tick.wrapping_add(1);
        self.tick
    }

    fn evict_for(&mut self, incoming_bytes: usize, replace_key: Option<usize>) -> bool {
        if incoming_bytes == 0 || incoming_bytes > self.budget_bytes {
            return false;
        }
        if let Some(key) = replace_key
            && let Some(old) = self.entries.remove(&key)
        {
            self.used_bytes = self.used_bytes.saturating_sub(old.byte_size);
        }
        while self.used_bytes + incoming_bytes > self.budget_bytes {
            let Some((&lru_key, _)) = self
                .entries
                .iter()
                .min_by_key(|(_, entry)| entry.last_used_tick)
            else {
                return false;
            };
            if let Some(removed) = self.entries.remove(&lru_key) {
                self.used_bytes = self.used_bytes.saturating_sub(removed.byte_size);
                self.evictions += 1;
            }
        }
        true
    }

    /// Captures the live GPU buffers of `backend` into VRAM under key `completed_segments` using
    /// GPU-to-GPU `copy_buffer_to_buffer` (evicting LRU entries if needed to respect `budget_bytes`).
    /// Returns `false` cleanly if a single checkpoint exceeds `budget_bytes`.
    pub fn capture_from_backend(
        &mut self,
        backend: &WgpuComputeBackend,
        completed_segments: usize,
    ) -> bool {
        let byte_size = backend.checkpoint_byte_size();
        if !self.evict_for(byte_size, Some(completed_segments)) {
            return false;
        }
        let buffers = backend.clone_live_to_checkpoint_buffers();
        let tick = self.next_tick();
        self.used_bytes += byte_size;
        self.entries.insert(
            completed_segments,
            GpuCheckpointEntry {
                byte_size,
                last_used_tick: tick,
                buffers,
            },
        );
        true
    }

    /// Uploads a CPU [`StockCheckpointSnapshot`] into the VRAM LRU cache.
    /// Returns `false` if the snapshot exceeds `budget_bytes`.
    pub fn upload_from_cpu_snapshot(
        &mut self,
        ctx: &GpuSimContext,
        cp: &StockCheckpointSnapshot,
    ) -> bool {
        let key = cp.completed_segments();
        let byte_size = Self::snapshot_byte_size(cp);
        if !self.evict_for(byte_size, Some(key)) {
            return false;
        }
        let device = ctx.device();
        let queue = ctx.queue();
        let buffers = match cp {
            StockCheckpointSnapshot::Heightfield {
                z_stock,
                last_op_idx,
                ..
            } => {
                let cell_bytes = (z_stock.len() * std::mem::size_of::<f32>()) as u64;
                let z_buf = device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("chip_gpu_cp_hf_z"),
                    size: cell_bytes.max(16),
                    usage: wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                let op_buf = device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("chip_gpu_cp_hf_op"),
                    size: cell_bytes.max(16),
                    usage: wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                if cell_bytes > 0 {
                    queue.write_buffer(&z_buf, 0, bytemuck::cast_slice(z_stock));
                    queue.write_buffer(&op_buf, 0, bytemuck::cast_slice(last_op_idx));
                }
                GpuCheckpointBuffers::Heightfield {
                    z_stock: z_buf,
                    last_op_idx: op_buf,
                    cell_bytes,
                }
            }
            StockCheckpointSnapshot::Cylindrical {
                r_stock,
                last_op_idx,
                ..
            } => {
                let cell_bytes = (r_stock.len() * std::mem::size_of::<f32>()) as u64;
                let r_buf = device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("chip_gpu_cp_cyl_r"),
                    size: cell_bytes.max(16),
                    usage: wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                let op_buf = device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("chip_gpu_cp_cyl_op"),
                    size: cell_bytes.max(16),
                    usage: wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                if cell_bytes > 0 {
                    queue.write_buffer(&r_buf, 0, bytemuck::cast_slice(r_stock));
                    queue.write_buffer(&op_buf, 0, bytemuck::cast_slice(last_op_idx));
                }
                GpuCheckpointBuffers::Cylindrical {
                    r_stock: r_buf,
                    last_op_idx: op_buf,
                    cell_bytes,
                }
            }
            StockCheckpointSnapshot::TriDexel { stock, .. } => {
                let total_rays: usize = stock.planes.iter().map(|p| p.ray_count()).sum();
                let total_slots = total_rays * INLINE_DEXEL_SLOTS;
                let slots_bytes = (total_slots * std::mem::size_of::<FlatDexelSpan>()) as u64;
                let counts_bytes = (total_rays * std::mem::size_of::<u32>()) as u64;
                let m_slots = device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("chip_gpu_cp_td_slots"),
                    size: slots_bytes.max(16),
                    usage: wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                let m_counts = device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("chip_gpu_cp_td_counts"),
                    size: counts_bytes.max(16),
                    usage: wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                let mut ray_off = 0usize;
                for axis in Axis3::ALL {
                    let p = stock.plane(axis);
                    let slot_off =
                        (ray_off * INLINE_DEXEL_SLOTS * std::mem::size_of::<FlatDexelSpan>())
                            as u64;
                    let cnt_off = (ray_off * std::mem::size_of::<u32>()) as u64;
                    if !p.slots.is_empty() {
                        queue.write_buffer(&m_slots, slot_off, bytemuck::cast_slice(&p.slots));
                        queue.write_buffer(&m_counts, cnt_off, bytemuck::cast_slice(&p.counts));
                    }
                    ray_off += p.ray_count();
                }
                let overflow = [
                    stock.planes[0].overflow.clone(),
                    stock.planes[1].overflow.clone(),
                    stock.planes[2].overflow.clone(),
                ];
                GpuCheckpointBuffers::TriDexel {
                    m_slots,
                    m_counts,
                    slots_bytes,
                    counts_bytes,
                    overflow,
                }
            }
        };

        let tick = self.next_tick();
        self.used_bytes += byte_size;
        self.entries.insert(
            key,
            GpuCheckpointEntry {
                byte_size,
                last_used_tick: tick,
                buffers,
            },
        );
        true
    }

    /// Restores checkpoint `completed_segments` into `backend`.
    ///
    /// - On a VRAM cache hit, executes a fast GPU-to-GPU `copy_buffer_to_buffer` (zero PCIe copy).
    /// - On a VRAM cache miss (e.g., evicted due to tight budget), restores transparently from
    ///   `cpu_fallback` and promotes it back into the LRU cache when it fits within `budget_bytes`.
    pub fn restore_into_backend(
        &mut self,
        backend: &mut WgpuComputeBackend,
        completed_segments: usize,
        cpu_fallback: Option<&StockCheckpointSnapshot>,
    ) -> bool {
        if self.entries.contains_key(&completed_segments) {
            let tick = self.next_tick();
            if let Some(entry) = self.entries.get_mut(&completed_segments) {
                entry.last_used_tick = tick;
                backend.restore_from_gpu_checkpoint_buffers(&entry.buffers);
                self.hits += 1;
                return true;
            }
        }

        self.misses += 1;
        if let Some(cp) = cpu_fallback {
            use crate::backend::SimStockBackend as _;
            backend.reset_from_checkpoint(cp);
            // Promote restored checkpoint into VRAM LRU cache if it fits within budget
            let _ = self.capture_from_backend(backend, completed_segments);
            return true;
        }
        false
    }
}
