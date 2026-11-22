# Milestone 52 Implementation Plan — `WgpuComputeBackend` Foundation & Heightfield / Cylindrical WGSL Compute Sweepers

## Goal

Stand up ChipCutter's optional **`wgpu` Compute** simulation backend (`WgpuComputeBackend` implementing `SimStockBackend` behind `#[cfg(feature = "gpu-compute")]`) with **zero new external dependencies**, automatic transparent fallback to `CpuExactBackend` when no GPU compute adapter is available, and tiled WGSL compute sweepers for **2.5D Heightfield** and **4th-Axis Cylindrical** stocks—including atomic Rules 1–4 crash/gouge/flute/holder verification feeding `chip-sim`'s existing `ViolationBatchCoalescer`.

---

## Proposed Changes

### 1. Feature Plumbing & `GpuSimContext` (`chip-stock`, `chip-sim`, `chip-project`, `chip-cli`, `chip-gui`)

#### [MODIFY] `crates/chip-stock/Cargo.toml`
#### [MODIFY] `crates/chip-sim/Cargo.toml`
#### [MODIFY] `crates/chip-project/Cargo.toml`
#### [MODIFY] `crates/chip-cli/Cargo.toml`
#### [MODIFY] `crates/chip-gui/Cargo.toml`
#### [NEW] `crates/chip-stock/src/gpu/mod.rs`
#### [NEW] `crates/chip-stock/src/gpu/context.rs`
#### [NEW] `crates/chip-stock/src/gpu/types.rs`
- Add optional `gpu-compute = ["dep:wgpu", "dep:pollster"]` feature to `chip-stock` (re-exported via `chip-sim/gpu-compute` and `chip-project/gpu-compute`).
  - Default features of `chip-stock`, `chip-sim`, and `chip-project` remain **100% `wgpu`-free** (`AGENTS.md` Invariant #1).
  - `chip-cli` enables `default = ["gpu-compute"]`; `chip-gui` enables `gpu-compute` in both `default` and `flatpak-portal`.
- Implement `GpuSimContext`:
  - `GpuSimContext::from_shared(device: Arc<wgpu::Device>, queue: Arc<wgpu::Queue>, adapter_info: GpuAdapterSummary)` for zero-readback device sharing with `chip-gui`'s `eframe`/`ViewportRenderer`.
  - `GpuSimContext::try_new_headless() -> Option<Arc<Self>>` initializing a headless `wgpu` compute device (respecting `WGPU_BACKEND` and `WGPU_POWER_PREF`, preferring discrete/integrated hardware GPUs over software fallbacks unless software is the only adapter), returning `None` cleanly if no usable compute adapter exists.
  - Cache compiled `wgpu::ComputePipeline`s and `wgpu::BindGroupLayout`s inside `GpuSimContext` so repeated simulation runs and scrub deltas incur zero shader recompilation cost.
- Add `SimComputeBackendChoice` (`Auto` | `Gpu` | `Cpu`) and `ActiveSimBackendInfo { is_gpu: bool, adapter_name: String, backend_api: String }` in `chip-stock` / `chip-sim`.

---

### 2. GPU Sweep Data Contracts & Atomic Verification (`crates/chip-stock/src/gpu/types.rs`)

- Define tightly-packed, 16-byte-aligned `#[repr(C)] bytemuck::Pod + Zeroable` GPU buffer structs:
  - `GpuToolProfile`: encodes all 7 `AnalyticCutterEnvelope` geometries (`Flat`, `Ball`, `BullNose`, `Chamfer`/`VBit`, `Drill`, `FaceMill`, `ThreadMill` single-point & multi-form) plus `cutting_radius_mm`, `effective_reach_mm`, `flute_length_mm`, `stickout_length_mm`, `holder_radius_mm`.
  - `GpuSweepChord`: encodes stock-local `p0: [f32; 3]`, `p1: [f32; 3]`, `rot_deg_start/end: f32`, `op_idx: u32`, `profile_idx: u32`, `flags: u32` (`is_rapid`, `verify`, `motion_kind`), `chord_index: u32`, plus `rapid_crash_tol`, `gouge_tol` (linearized arcs already arrive as exact `SweepChord` segments from `chip-sim`'s unified chord emitter).
  - `GpuViolationAccum`: `atomic<u32>` counters per batch for Rules 1–4 (`rapid_flag`, `rapid_max_depth_um`, `rapid_chord_idx`, `gouge_flag`, `gouge_max_depth_um`, `gouge_chord_idx`, `flute_flag`, `flute_max_depth_um`, `flute_chord_idx`, `holder_flag`, `holder_max_depth_um`, `holder_chord_idx`, `mutated_flag`) using monotonic `atomicMax` in integer micrometers (`1e-3 mm` scaled × 1000 or IEEE-754 sortable `u32` bit-cast for positive `f32`), mapped back into `BatchSweepOutput::violations` with exact chord tip coordinates for click-to-jump diagnostics.

---

### 3. Tiled WGSL Compute Sweepers — Heightfield & Cylindrical

#### [NEW] `crates/chip-stock/src/gpu/heightfield_sweep.wgsl`
#### [NEW] `crates/chip-stock/src/gpu/heightfield.rs`
#### [NEW] `crates/chip-stock/src/gpu/cylindrical_sweep.wgsl`
#### [NEW] `crates/chip-stock/src/gpu/cylindrical.rs`
#### [NEW] `crates/chip-stock/src/gpu/backend.rs`
- **2.5D Heightfield Compute Sweeper (`heightfield_sweep.wgsl`):**
  - Upload `z_stock`, `z_bottom`, `z_target`, `last_op_idx` directly as flat storage buffers (already stock-local `f32` and `u32` from Milestone 50!).
  - Perform fast CPU coarse tile-binning into `16×16` cell tiles (`tile_offsets`, `tile_chord_indices`) so each `@workgroup_size(16, 16, 1)` workgroup only iterates over chords whose expanded XY AABB overlaps its tile.
  - Mirror the exact analytical swept formulas from `heightfield/sweep.rs`:
    - Flat / FaceMill (including the `v0.14.1` sloped linear interval intersection fix!), Ball, Bull-Nose, closed-form sloped conical V-Bit / Chamfer, Twist Drill, and ThreadMill envelope.
    - Respect `z_bottom` (`-inf` = outside raw stock) and empty-cell through-drilling (`z_stock < z_bottom -> -inf`).
    - Record Rules 1–4 violations atomically into `GpuViolationAccum` when `verify == true`.
- **4th-Axis Cylindrical Compute Sweeper (`cylindrical_sweep.wgsl`):**
  - Upload `r_stock`, `r_init`, `r_target`, `last_op_idx` directly as flat storage buffers.
  - Perform CPU coarse `(iu, itheta)` tile-binning into `16×16` polar tiles (handling `2π` angular wrap cleanly) so each `@workgroup_size(16, 16, 1)` workgroup sweeps only intersecting rotary chords.
  - Subdivide each rotary chord along `(tip_wcs(t), rot_deg(t))` identically to `cylindrical/sweep.rs`, solving the exact cylinder-ray interval + 10-step bisection for off-centerline simultaneous 4-axis (`X+Y+Z+A/B`), `Z+X+A` spiral/wrapped, and indexed `3+1` moves, plus chuck keep-out and Rules 1–4 atomic violation checks.
- **Wire `WgpuComputeBackend` into `chip-sim/src/driver.rs`:**
  - `run_setup_simulation_with_backend` accepts `WgpuComputeBackend` transparently for `Heightfield25D` and `Cylindrical4Axis` (falling back to `CpuExactBackend` for `TriDexel3D` until Milestone 53 lands, or when `Auto` finds no GPU adapter).
  - Implement clean readback (`into_parts`, `capture_checkpoint`, `reset_from_checkpoint`) so existing `SetupSimulationResult` callers work identically.

---

## Verification Plan

1. **Struct Layout & Shader Compilation Tests (`chip-stock` with `--features gpu-compute`):**
   - Assert byte sizes, alignments, and `bytemuck::Pod` roundtrips for `GpuToolProfile`, `GpuSweepChord`, and `GpuViolationAccum`.
   - Validate WGSL shader compilation on Vulkan (hardware Radeon 780M and software `lavapipe`).
2. **CPU-vs-GPU Differential Oracle Tests (`chip-stock`, `chip-sim`):**
   - **Heightfield parity:** Sweep all 7 cutter geometries (including sloped flat endmills, conical V-carve corner climbs, and rapid/gouge/flute/holder fault injection) on both `CpuExactBackend` and `WgpuComputeBackend`; assert volume parity `< 0.05%`, per-cell `|z_gpu - z_cpu| <= 2e-3 mm`, and **identical** Rule 1–5 diagnostic counts & severities.
   - **Cylindrical parity:** Run `rotary_shaft`, `rotary_wrapped`, and `rotary_4axis_sim` setups on both backends; assert volume parity `< 0.1%`, per-cell `|r_gpu - r_cpu| <= 5e-3 mm`, and identical crash/gouge detection.
   - **Pure-CPU Hygiene:** Verify `cargo check --locked -p chip-stock` (default features) compiles with zero `wgpu` dependency.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
