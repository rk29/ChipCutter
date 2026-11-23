# 06 — Toolpath Simulation & Crash Verification

## Overview (`chip-sim` & `chip-stock`)
Before exporting G-code to a CNC controller, `chip-sim` (orchestrator, timeline, 6-rule verifier, CAD target adapters, and stage timing) and `chip-stock` (UI-agnostic volumetric stock kernels, meshers, metrology, and compute backends) provide **interactive visual playback**, **automated 6-rule crash/gouge/fixture verification**, **multi-setup volumetric stock carryover**, and **quantitative stock-vs-target error-volume / RMS metrology** across 3D STEP/STL/OBJ/3MF solids, 4th-axis rotary setups, and Pure-2D sheet setups (`models = []`).

---

## 1. Animated Toolpath & Cutter Playback
- **Color-Coded Toolpath Overlay:**
  - `G0` Rapid moves: Dashed yellow/amber lines
  - `G1` Plunge / Ramp moves: Red/coral lines
  - `G1` Cutting feed moves: Royal blue / cyan lines
  - `G2` / `G3` Circular, Helical, & Trochoidal arcs: Emerald green curves
  - Lead-in / Lead-out & Adaptive Micro-Lift transitions: Teal/magenta accents
- **Interactive Playback Scrubber:**
  - Play / Pause (`Space`), Step Forward / Step Backward by single motion segment (`←` / `→`), Jump to Next Operation / Tool Change (including two-stage V-Carve Flat Clearance -> V-Bit transitions).
  - Continuous timeline slider parameterized by **cumulative machining time** or **path distance**, with playback speed multiplier (`0.1x` to `50x`, plus "Instant Jump to End").
- **3D Cutter & Shank Visualization:**
  - Renders an accurate 3D revolution mesh of all 10 active cutter geometries (`Flat`, `BullNose`, `Ball`, `Chamfer`/V-Bit, `TwistDrill`, `FaceMill`, `ThreadMill`, `LollipopEndmill`, `SlotMill`, and `DovetailCutter`): cutting flutes/spheres/discs/inverted dovetail cones, relieved narrow necks, non-cutting shanks, and simplified collet/holder cones above `stickout_length`.
  - Live HUD readout at current scrub position: `Operation`, `Tool`, `XYZ` (in WCS), motion type (`G0`/`G1`/`G2`/`G3`), commanded `Feedrate`, `RPM`, and `Elapsed / Total Time` (or `Distance`) with percent complete (`0%`–`100%`).

---

## 2. Machining Time Estimator
- Computes per-operation and per-setup cycle time using a trapezoidal velocity profile approximation:
  - Uses programmed feedrate (`Vf`, including Adaptive radial chip-thinning and micro-lift return feeds) on `G1`/`G2`/`G3` moves, machine rapid rate (configurable per post/machine profile, default `5000 mm/min`) on `G0` moves, dwell durations (`G4`), and an optional corner acceleration limit (`default 500 mm/s²`) + tool-change penalty (`default 15 s` ATC or `0 s` manual).

---

## 3. Volumetric Stock Engines (`chip-stock`)
All three volumetric stock representations share a unified polymorphic container (`SimulatedStock`), unified periodic checkpoints (`StockCheckpointSnapshot`), a single playback cursor state machine (`StockPlaybackCursor`), shared cutter profile math (`AnalyticCutterEnvelope` across all 10 cutter geometries including non-monotonic relieved-neck `ThreadMill`, `LollipopEndmill`, `SlotMill`, and `DovetailCutter`), and stock-local `f32`/`u32` GPU-compatible layouts:

### 3.1 2.5D Heightmap (Z-Dexel) Stock (`HeightfieldStock`)
- Uniform XY grid aligned with the Setup's stock bounds (`Standard`: `~512`, `Fine`: `~1024`, `Ultra`: `~2048` cells along the longest XY axis).
- Stores stock-local `z_stock: Vec<f32>`, `z_target: Vec<f32>` (upper envelope of the target B-Rep/mesh solid, or `-∞` in Pure-2D jobs), and `last_op_idx: Vec<u32>` (`UNCUT_OP_IDX = u32::MAX` for unmachined raw stock, enabling **Color by Operation**).
- Evaluates swept cutter envelopes analytically across linear, circular/helical, and sloped 3D V-carve/chamfer segments (`z_cut = z_tip(t) + P(r)`).

### 3.2 Full 3D Volumetric Tri-Dexel Stock (`TriDexelStock`)
- Axis-generic 3-plane orthogonal dexel lattice (`planes: [DexelPlane; 3]` parameterized by `Axis3::{X, Y, Z}`) at isotropic resolution (`Standard`: `160`, `Fine`: `288`, `Ultra`: `448` cells along the longest 3D axis).
- Each ray stores sorted solid intervals in a **flat fixed-`K` inline span pool** (`K = 4` × 16-byte `FlatDexelSpan { lo: f32, hi: f32, op_lo: u32, op_hi: u32 }` + `counts: Vec<u32>` + a rare CPU `BTreeMap` overflow side-table for `> K` cavities such as multi-form internal thread grooves).
- Accurately models vertical/steep walls without heightmap stair-stepping, horizontal features, true **3D undercuts** (helical internal/external thread milling, T-slots, `60°`/angled dovetails, concave Lollipop overhangs, and Lollipop underside back-chamfers) via exact bounded axial cut intervals (`axial_z_interval_at_radius`) that preserve solid overhanging roofs intact, and full **4th-axis volumetric rotary stock removal** (analytical cardinal `0°/90°/180°/270°` indexed `3+1` fast path plus arbitrary-orientation rotary pose sweeping for oblique `3+1`, `2-Linear + Rotary`, and `Simultaneous 4-Axis` setups) on both CPU and GPU with live `stock_model` workpiece rotation and rotary chuck keep-out verification.

### 3.3 4th-Axis Polar Cylindrical Stock (`CylindricalStock`)
- Automatically selected on rotary Setups (`X/A`, `Y/B`, `Z/C` axes) when `SimEngineKind::Heightmap` is active (whereas `SimEngineKind::TriDexel` runs full 3D volumetric `TriDexelStock` on rotary Setups to capture non-star-shaped overhangs, deep below-centerline indexed roughing passes, and undercut helical flutes). Stores a polar `(u_axial, θ) -> r_stock: Vec<f32>` radius grid alongside `r_target: Vec<f32>`, `last_op_idx: Vec<u32>`, and chuck keep-out geometry.
- Supports `2-Linear + Rotary` (wrapped/spiral/axial), `Indexed 3+1`, and full off-centerline `Simultaneous 4-Axis` swept contact, rotating the rendered workpiece cleanly on the GPU via the `stock_model: mat4x4<f32>` uniform.

---

## 4. `SimStockBackend` & Optional `wgpu` Compute (`gpu-compute`)
`chip-sim`'s single polymorphic driver (`run_setup_simulation_with_backend`) drives any implementation of `SimStockBackend`:
- **`CpuExactBackend` (Always Available, Pure-CPU Default):** Multi-threaded `rayon` exact volumetric kernels with zero `wgpu` dependency in default `chip-stock`/`chip-sim`/`chip-project` builds.
- **`WgpuComputeBackend` (`#[cfg(feature = "gpu-compute")]`):** Enabled by default in `chip-gui` (including `flatpak-portal`) and `chip-cli` with selectable `SimComputeBackendChoice { Auto, Gpu, Cpu }`:
  - **Shared or Headless `GpuSimContext`:** Shares `eframe`/`ViewportRenderer`'s `(Arc<wgpu::Device>, Arc<wgpu::Queue>)` in the GUI (zero extra device context) and lazily initializes a headless Vulkan/Metal/DX12 adapter in CLI/tests, falling back transparently to `CpuExactBackend` whenever no compute adapter is available.
  - **Tiled WGSL Compute Sweepers:** `16×16` tiled compute shaders with coarse AABB tile-binning and atomic fixed-point Rule 1–4 violation accumulation across all 10 cutter profiles (`GpuToolProfile` 64-byte `Pod` layout including `SHAPE_LOLLIPOP`, `SHAPE_SLOTMILL`, and `SHAPE_DOVETAIL`):
    - `heightfield_sweep.wgsl` (2.5D heightfield)
    - `cylindrical_sweep.wgsl` (4th-axis polar cylindrical)
    - `tridexel_sweep.wgsl` (axis-generic `X`/`Y`/`Z` flat fixed-`K` pool with exact bounded undercut-head subtraction, relieved-neck/shank verification, and a hybrid per-ray atomic overflow safety valve that seamlessly resolves rare `> K` rays on CPU)
  - **GPU BVH Target Rasterization & Distance (`bvh_distance.wgsl`):** Uploads `chip-bvh`'s 32-byte `FlatBvhNode` and 48-byte `GpuTriangle` buffers directly to GPU for fast mesh target rasterization and signed-distance evaluation.
  - **GPU Parallel Reduction Metrology (`stock_metrics.wgsl`):** Computes remaining/excess/gouge volume, RMS, MAE, signed bias, P95/P99 histogram percentiles, max excess/gouge WCS locations, and `in_tolerance_surface_pct` via parallel GPU workgroup reductions in `< 1 ms`.
  - **Zero-Readback Direct GPU Rendering (`SimStockRenderSource::GpuGrid`) & Instant Error Heatmap:**
    - `Heightfield` and `Cylindrical` storage buffers render directly in `ViewportRenderer` via vertex-displaced WGSL pipelines (`vs_sim_heightfield_gpu`, `vs_sim_cylindrical_gpu`) with on-the-fly finite-difference normals—eliminating CPU tessellation and PCIe readback during Ultra playback.
    - `TriDexel` buffers render directly via a GPU Dual-Vertex Surface Nets compact-emit compute pass (`tridexel_emit.wgsl`, 24-byte `PackedTriDexelQuad` records with snapped dual corners and dominant-axis crease-aware smooth normals covering the full worst-case `Ultra` grid within WebGPU's 128 MiB storage buffer limit) expanded on-the-fly by `vs_sim_tridexel_gpu` via `draw_indirect`.
    - Signed stock-vs-target **Error Heatmap** shading evaluates directly in WGSL against GPU target buffers and a `tolerance_mm` uniform (`< 0.1 ms` toggle and live `±tol` slider updates, even during live playback).
    - **VRAM-Bounded `GpuCheckpointCache`:** Stores periodic GPU buffer snapshots (`copy_buffer_to_buffer` restore + GPU tail-delta sweep) with automatic CPU spill on constrained adapters for `< 1 ms` interactive Ultra timeline scrubbing.

---

## 5. Automated Crash, Gouge, Fixture, & Engagement Verification
During simulation (and automatically during `chipcutter simulate` / `chipcutter build`), `chip-sim` evaluates six safety rules (with identical diagnostics across `CpuExactBackend` and `WgpuComputeBackend`) and logs structured **Simulation Diagnostics** clickable in the UI to jump the camera + timeline straight to the offending move:

| Check | Condition | Severity |
|---|---|---|
| **1. Rapid-into-Stock Crash** | A `G0` rapid move sweeps any cell/ray where the cutter enters uncut stock beyond `rapid_crash_tol_mm` | **Critical Error** (blocks G-code export unless explicitly acknowledged) |
| **2. Target Model Gouge** | When `models` has part solids, any cutting or rapid move penetrates the target solid beyond `gouge_tol_mm` (default `0.02 mm`); `ModelRole::Fixture` solids are excluded | **Critical Error** |
| **3. Flute Length / Neck Clearance Exceeded** | Engagement depth exceeds `flute_length` (Warning), OR non-cutting relieved neck / shank collides with uncut stock on `ThreadMill`, `LollipopEndmill`, `SlotMill`, or `DovetailCutter` (e.g., missing pre-slot throat or overhanging ceiling rub — **Critical Error**) | **Warning** / **Critical Error** (relieved-neck collision) |
| **4. Holder / Collet / Chuck Collision** | Stock or rotary chuck penetrates holder/collet envelope above `stickout_length` | **Critical Error** |
| **5. Excessive Plunge in Material** | Vertical plunge move (`ΔXY == 0`) with non-center-cutting cutter or axial penetration exceeding safe plunge depth (**Critical Error** when plunging a `ThreadMill`, `SlotMill`, or `DovetailCutter` vertically into solid un-cleared stock) | **Warning** / **Critical Error** (undercut/thread mill plunge) |
| **6. 3D Fixture Collision (`FixtureCollision`)** | Swept flute, shank, or holder/collet envelope penetrates any enabled `Setup.fixtures` solid (+ `clearance_margin_mm`, honoring `allowed_overcut_mm` on sacrificial spoilboards) across Heightfield, Tri-Dexel, and Cylindrical engines | **Critical Error** (blocks G-code export; colliding hardware highlights crimson in 3D viewport) |

### Multi-Setup & Multi-Angle `Indexed3Plus1` Volumetric Stock Carryover & Cumulative Metrology
- When `Setup.carryover_from_setup_id` is configured, `chip-project` simulates upstream Setups in `TriDexel` mode, rigidly transforms the final remaining `TriDexelStock` into the downstream Setup's WCS frame (`t = 0.0`), and evaluates cumulative end-of-job `StockErrorReport` metrics across all Setups.
- Inside a single `Indexed3Plus1` Setup (`0°` / `120°` / `240°`, etc.), whenever a downstream 3D roughing operation (`Adaptive3D`, `Waterline3D`) enables `RestMachiningMode::FromPrecedingOperations` following operations at different rotary index angles, `chip-project` sequentially sweeps preceding operations into a shared volumetric `TriDexelStock` and transforms the remaining in-process stock into the active virtual indexed WCS frame via `TriDexelStock::transform_to_frame`—verifying zero cross-angle rapid-into-stock crashes and zero target gouges across both CPU and GPU backends.
- `chipcutter simulate --all-setups` executes the full multi-setup carryover chain sequentially and emits both per-Setup diagnostics/metrics and the cumulative end-of-job `cumulative_error_report` in JSON (`--json`) and human-readable text modes.

---

## 6. Quantitative Error Metrology, GUI Controls, & CLI Benchmarking
- **`StockErrorReport`:** Computed across Heightfield, Tri-Dexel, and Cylindrical engines—reporting `total_error_volume_mm3`, `excess_volume_mm3`, `gouge_volume_mm3`, `material_removal_efficiency_pct`, `rms_error_mm`, `mean_abs_error_mm`, `mean_signed_error_mm`, `p95_abs_error_mm`, `p99_abs_error_mm`, `in_tolerance_surface_pct`, worst excess/gouge WCS locations, and per-operation error progression.
- **Simulation Timing Metadata (`SimTimingBreakdown`):** Every `SetupSimulationResult` records `target_setup_ms`, `sweep_ms`, `metrics_ms`, `total_ms`, `total_segments`, and `segments_per_sec` alongside `ActiveSimBackendInfo` (`is_gpu`, `adapter_name`, `backend_api`).
- **Desktop GUI Controls (`chip-gui`):** The Simulation Timeline toolbar provides Engine (`Heightmap` / `Tri-Dexel`), Resolution (`Standard` / `Fine` / `Ultra`), Backend (`Auto (GPU)` / `GPU` / `CPU`, persisted in `prefs.toml`), an active backend/adapter status pill (`GPU · <Adapter>`, `CPU (Exact)`, or `CPU Fallback`) with stage timing tooltips, and Error Heatmap controls, plus headless capture overrides (`--capture-sim-backend`, `--capture-sim-engine`, `--capture-sim-resolution`, `--capture-sim-heatmap`). Simulation calculation is started/cancelled from the top toolbar `Simulate` button, and live `0%`–`100%` progress is displayed in a single compact progress bar next to the top toolbar's `Generate` / `Simulate` / `Post` buttons.
- **Headless CLI (`chipcutter simulate` & `chipcutter build`):** `simulate` supports `--engine <heightmap|tri-dexel>`, `--backend <auto|gpu|cpu>`, `--resolution <standard|fine|ultra>`, `--tolerance <mm>`, `--benchmark` (structured timing & throughput table), `--json` (machine-readable stdout JSON including `backend`, `timing`, `diagnostics`, and `error_report`), and `--report <path>`. `chipcutter build` automatically promotes its mandatory pre-post safety gate to volumetric `TriDexel` (`Auto`) whenever a Setup uses undercut strategies (`SlotUndercut`, `Dovetail`, `Undercut3D`), relieved-neck undercut cutters (`LollipopEndmill`, `SlotMill`, `DovetailCutter`), or multi-setup stock carryover.
