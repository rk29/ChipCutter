# Milestone 48 Implementation Plan — Shared `AnalyticCutterEnvelope`, Zero-Copy GPU Stock Buffers, & Simulation/Rotary Deduplication

## Goal

Deliver the highest-ROI Tier 1 refactoring quick wins from the post-Milestone-46 architectural audit with zero behavior regressions and byte-for-byte golden G-code parity: unify analytic cutter profile math across `chip-cam` and `chip-sim`, eliminate per-frame GPU vertex copies and 4th-axis CPU re-tessellation in `chip-gui`, unify shared `chip-sim` verification/color/arc helpers, deduplicate 4th-axis rotary strip-mesh builders, and move bulky inline integration tests out of library entrypoints.

---

## Proposed Changes

### 1. Shared `AnalyticCutterEnvelope` (`chip-tools`, `chip-cam`, `chip-sim`)

#### [NEW] `crates/chip-tools/src/envelope.rs`
#### [MODIFY] [crates/chip-tools/src/lib.rs](file:///workspace/ChipCutter/crates/chip-tools/src/lib.rs)
#### [MODIFY] [crates/chip-cam/src/surface3d/cutter.rs](file:///workspace/ChipCutter/crates/chip-cam/src/surface3d/cutter.rs)
#### [MODIFY] [crates/chip-sim/src/cutter_profile.rs](file:///workspace/ChipCutter/crates/chip-sim/src/cutter_profile.rs)
- Introduce `AnalyticCutterEnvelope` in `chip-tools` directly alongside `CutterGeometry`, providing canonical `f64` millimeter queries for all 7 cutter geometries (`FlatEndmill`, `BullNoseEndmill`, `BallEndmill`, `ChamferVBit`, `Drill`, `FaceMill`, `ThreadMill`):
  - `from_geometry(geom: &CutterGeometry)` and `with_stock_allowance(radial_mm: f64, axial_mm: f64)` (exact Minkowski disk dilation for 3D surface roughing/finishing).
  - `profile_dz_at_radius(r_mm: f64) -> Option<f64>`
  - `radius_at_dz(dz_mm: f64) -> Option<f64>`
  - `conical_slope_cot() -> Option<f64>`
  - Non-monotonic `ThreadMill` queries: `effective_reach_mm()`, `thread_mill_tooth_radius_at_z(z_local)`, `cutter_radius_at_z(z_local)`, and `thread_mill_axial_intervals_at_radius(r_mm)`.
- Delegate `chip_cam::surface3d::SurfaceCutterProfile` and `chip_sim::RadialCutterProfile` to `chip_tools::AnalyticCutterEnvelope` so both crates share one exact geometric source of truth while keeping `chip-cam`'s drop-cutter contact solver and `chip-sim`'s shank/holder/3D mesh assembly cleanly separated.

---

### 2. Zero-Copy GPU Stock Buffers & GPU Rotary Matrix (`chip-sim`, `chip-gui`)

#### [MODIFY] [crates/chip-sim/Cargo.toml](file:///workspace/ChipCutter/crates/chip-sim/Cargo.toml)
#### [MODIFY] [crates/chip-sim/src/cutter_profile.rs](file:///workspace/ChipCutter/crates/chip-sim/src/cutter_profile.rs)
#### [MODIFY] [crates/chip-gui/src/viewport/gpu_types.rs](file:///workspace/ChipCutter/crates/chip-gui/src/viewport/gpu_types.rs)
#### [MODIFY] [crates/chip-gui/src/viewport/shaders.wgsl](file:///workspace/ChipCutter/crates/chip-gui/src/viewport/shaders.wgsl)
#### [MODIFY] [crates/chip-gui/src/viewport/pipeline.rs](file:///workspace/ChipCutter/crates/chip-gui/src/viewport/pipeline.rs)
#### [MODIFY] [crates/chip-gui/src/panels/simulation.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/simulation.rs)
- Add `bytemuck` (already in workspace `Cargo.toml`) to `chip-sim` and annotate `SimColoredVertex` with `#[repr(C)]` + `bytemuck::Pod` + `bytemuck::Zeroable` matching `SimVertexGpu`'s exact 40-byte vertex layout (`position: [f32; 3]`, `normal: [f32; 3]`, `color: [f32; 4]`).
- Replace `device.create_buffer_init` + per-frame `Vec<SimVertexGpu>` copies in `upload_sim_indexed_mesh` with zero-copy `bytemuck::cast_slice(&mesh.vertices)` uploads into reusable `GrowableGpuBuffer` allocations for both `uploaded_sim_stock` and `uploaded_sim_tool`.
- Add a `stock_model: mat4x4<f32>` uniform buffer to `vs_sim_stock` in `shaders.wgsl` (identity for 3-axis Heightmap/Tri-Dexel; rotary axis transform for 4th-axis Cylindrical stock).
- Stop triggering CPU `tessellate_cylindrical_stock_mesh_colored` on rotary angle-only ticks in `panels/simulation.rs` when `r_stock` has not changed—update only the `stock_model` GPU uniform for smooth 60 FPS rotary playback!

---

### 3. Shared `chip-sim` Verification/Mesher Helpers & `chip-cam` Rotary Deduplication

#### [NEW] `crates/chip-sim/src/mesher_common.rs`
#### [NEW] `crates/chip-sim/src/verify_common.rs`
#### [MODIFY] [crates/chip-sim/src/heightmap/mesher.rs](file:///workspace/ChipCutter/crates/chip-sim/src/heightmap/mesher.rs)
#### [MODIFY] [crates/chip-sim/src/heightmap/sweeper.rs](file:///workspace/ChipCutter/crates/chip-sim/src/heightmap/sweeper.rs)
#### [MODIFY] [crates/chip-sim/src/tridexel/mesher.rs](file:///workspace/ChipCutter/crates/chip-sim/src/tridexel/mesher.rs)
#### [MODIFY] [crates/chip-sim/src/tridexel/sweep.rs](file:///workspace/ChipCutter/crates/chip-sim/src/tridexel/sweep.rs)
#### [MODIFY] [crates/chip-sim/src/cylindrical.rs](file:///workspace/ChipCutter/crates/chip-sim/src/cylindrical.rs)
#### [MODIFY] [crates/chip-cam/src/rotary.rs](file:///workspace/ChipCutter/crates/chip-cam/src/rotary.rs)
#### [MODIFY] [crates/chip-cam/src/strategies/rotary2linear.rs](file:///workspace/ChipCutter/crates/chip-cam/src/strategies/rotary2linear.rs)
#### [MODIFY] [crates/chip-cam/src/strategies/rotary4axis_sim.rs](file:///workspace/ChipCutter/crates/chip-cam/src/strategies/rotary4axis_sim.rs)
- Unify `RAW_STOCK_TOP_COLOR`, `RAW_STOCK_SIDE_COLOR`, `MACHINED_METAL_COLOR`, and `op_color` in `crates/chip-sim/src/mesher_common.rs` (resolving the drifted `UNCUT_STOCK_RGBA` in `cylindrical.rs`).
- Extract shared arc-to-chord decomposition (`decompose_segment_to_chords`), Rule 1–5 violation accumulator & bounded diagnostic coalescing (`ViolationBatchCoalescer`), and Rule 5 vertical plunge validation (`evaluate_vertical_plunge_violation`) into `crates/chip-sim/src/verify_common.rs`.
  - Remove dead `recorded_op_kinds` in `tridexel/sweep.rs`.
  - Give `cylindrical.rs` consistent bounded diagnostic coalescing matching Heightmap and Tri-Dexel.
- Deduplicate `build_rotated_strip_mesh` and rotary chuck-clearance axial window resolution from `rotary2linear.rs` and `rotary4axis_sim.rs` into shared helpers in `crates/chip-cam/src/rotary.rs`.

---

### 4. Inline Integration Test Hygiene (`chip-project`, `chip-sim`)

#### [NEW] `crates/chip-project/tests/project_integration.rs`
#### [MODIFY] [crates/chip-project/src/lib.rs](file:///workspace/ChipCutter/crates/chip-project/src/lib.rs)
#### [NEW] `crates/chip-sim/tests/sim_integration.rs`
#### [MODIFY] [crates/chip-sim/src/lib.rs](file:///workspace/ChipCutter/crates/chip-sim/src/lib.rs)
- Move the `2,861` LOC of inline integration tests out of `crates/chip-project/src/lib.rs` and `1,360` LOC out of `crates/chip-sim/src/lib.rs` into dedicated `tests/` integration targets so library compilation skips parsing thousands of test lines during normal builds.

---

## Verification Plan

1. **Exact Math & GPU Upload Tests:**
   - `chip-tools`: Unit tests verifying `AnalyticCutterEnvelope` parity across all 7 cutter geometries (including Minkowski allowance and single-point/multi-form `ThreadMill` tooth bands).
   - `chip-gui`: Offscreen Vulkan test confirming zero-copy `GrowableGpuBuffer` stock uploads and `stock_model` matrix rotation for 4th-axis cylindrical stock.
2. **Full Workspace Parity Gate:**
   - `cargo fmt --all -- --check`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace` (preserving byte-for-byte golden G-code snapshots and all zero-crash/zero-gouge simulation oracles)
