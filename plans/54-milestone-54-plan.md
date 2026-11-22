# Milestone 54 Implementation Plan — Zero-Readback Direct GPU Stock Rendering (`SimStockRenderSource::GpuGrid`) & Instant Error Heatmap

## Goal

Eliminate CPU stock re-tessellation and PCIe readback during interactive simulation playback by adding **`SimStockRenderSource::GpuGrid`** to `chip-gui`'s `ViewportRenderer`, rendering Heightfield, Cylindrical, and Tri-Dexel GPU compute storage buffers directly on the shared `eframe`/`wgpu` device, evaluating signed **Error Heatmap** coloring instantaneously in WGSL (making `±tol` slider drags `< 0.1 ms`), and caching VRAM-bounded GPU checkpoints for smooth `< 1 ms` Ultra-resolution timeline scrubbing.

---

## Proposed Changes

### 1. Shared Device Plumbing & `SimStockRenderSource::GpuGrid` (`chip-gui`, `chip-stock`)

#### [MODIFY] `crates/chip-gui/src/viewport/highlight.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/pipeline.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/shaders.wgsl`
#### [NEW] `crates/chip-gui/src/viewport/sim_gpu_render.rs`
- Pass `eframe::CreationContext`'s `wgpu_render_state` `(Arc<wgpu::Device>, Arc<wgpu::Queue>)` into `GpuSimContext::from_shared(...)` so `WgpuComputeBackend` and `ViewportRenderer` share the exact same `wgpu::Device` with zero cross-device copies!
- Extend `SimStockRenderSource` in `viewport/highlight.rs`:
  - `CpuMesh { revision: u64, mesh: Arc<SimStockMesh>, stock_model: DMat4 }` (preserved for CPU backend, tests, and non-GPU fallback).
  - `GpuGrid(Arc<GpuStockRenderHandle>)` (behind `#[cfg(feature = "gpu-compute")]`) referencing the live GPU storage buffers, grid dimensions/origin uniform, `stock_model: DMat4`, `SimStockColorMode` (`ByOperation` | `Uniform` | `ErrorHeatmap { tolerance_mm }`), and monotonic `revision`.

---

### 2. Direct Vertex-Displaced Heightfield/Cylindrical Shaders & GPU Tri-Dexel Indirect Mesh Pass

- **Direct GPU Heightfield Renderer (`vs_sim_heightfield_gpu` + `fs_sim_gpu_stock`):**
  - Renders a reusable static index/quad grid (or vertex-index procedural grid `draw(0..cell_count * 6, 0..1)` with zero vertex buffer!) sampling `z_stock`, `z_bottom`, `z_target`, and `last_op_idx` directly from the compute storage buffers.
  - Emits crisp top surfaces and vertical riser skirts (discarding empty/outside cells where `z_stock <= z_bottom` cleanly) with finite-difference normals computed in WGSL matching `heightfield/mesher.rs`.
- **Direct GPU 4th-Axis Cylindrical Renderer (`vs_sim_cylindrical_gpu` + `fs_sim_gpu_stock`):**
  - Renders the `(nu × ntheta)` polar cylinder shell + endcaps directly from `r_stock`, `r_target`, and `last_op_idx` storage buffers, applying `u_vp.stock_model` on the GPU so rotary scrubbing and animation require **zero CPU work and zero PCIe uploads**!
- **Direct GPU Tri-Dexel Surface Extraction (`tridexel_emit.wgsl` + `draw_indirect`):**
  - Lightweight WGSL compute pass scanning `Axis3::{X, Y, Z}` `FlatDexelSpan` endpoints and writing `SimColoredVertex` (40B `Pod`) quads + an atomic `DrawIndirectArgs` counter directly into a reusable GPU vertex buffer (with CPU merge only for rare overflow rays when `any_overflow != 0`), then drawing with `render_pass.draw_indirect(...)` on the same GPU—eliminating CPU-to-GPU upload of hundreds of thousands of Ultra Tri-Dexel triangles!

---

### 3. Instant GPU Error Heatmap & VRAM-Bounded GPU Scrub Checkpoints

#### [MODIFY] `crates/chip-gui/src/panels/simulation/worker.rs`
#### [MODIFY] `crates/chip-gui/src/panels/simulation/playback.rs`
#### [MODIFY] `crates/chip-stock/src/gpu/backend.rs`
- **Zero-Cost Heatmap & Tolerance Slider:**
  - Store `color_mode`, `tolerance_mm`, and the 16-entry operation color palette in a tiny GPU uniform buffer (`SimStockVisualUniforms`).
  - `fs_sim_gpu_stock` / `tridexel_emit` evaluates the exact same signed deviation ramp (`blue = excess`, `green = within ±tol`, `red = gouge`) used by `chip-stock/src/mesher_common.rs` directly on GPU—so toggling `Color by Op` ↔ `Error Heatmap` or dragging the `±tol` slider updates in `< 0.1 ms` without re-meshing or re-sweeping!
- **VRAM-Bounded GPU Checkpoints & Async GPU Playback Worker:**
  - Keep periodic GPU-side checkpoint copies (`copy_buffer_to_buffer`) inside a configurable VRAM budget (e.g., `<= 256 MB`, with graceful LRU spill to CPU `StockCheckpointSnapshot` on low-VRAM adapters).
  - Update `StockRemeshWorker` so when backed by `GpuSimContext`, scrubbing restores the nearest GPU checkpoint via `copy_buffer_to_buffer`, dispatches the short tail-delta compute sweep on GPU, and publishes an updated `SimStockRenderSource::GpuGrid` handle directly—retaining 100% deterministic synchronous completion (`queue.submit` + `device.poll(Wait)`) for headless `--capture-*` screenshots!

---

## Verification Plan

1. **Direct GPU Render & Heatmap Unit Tests (`chip-gui`):**
   - Headless `wgpu` offscreen render tests comparing rendered pixel output of `SimStockRenderSource::GpuGrid` against `SimStockRenderSource::CpuMesh` across Heightfield, Cylindrical, and Tri-Dexel stocks (both `Color by Op` and `Error Heatmap` modes).
   - Verify changing `tolerance_mm` or toggling `Error Heatmap` updates GPU uniform state with zero stock re-sweep and zero CPU mesh allocation.
   - Verify VRAM budget enforcement cleanly falls back to CPU checkpoint spill when budget is tight.
2. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
