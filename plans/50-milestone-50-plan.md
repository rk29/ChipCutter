# Milestone 50 Implementation Plan — Unified `SimStockBackend`, Axis-Generic Flat-Pool Tri-Dexel, & `chip-stock` Crate Extraction

## Goal

Unify all three simulation stock engines (`HeightfieldStock`, `TriDexelStock`, `CylindricalStock`) behind a clean polymorphic `SimStockBackend` trait and `SimulatedStock` enum, eliminate ~450 LOC of Tri-Dexel X/Y/Z axis copy-paste, migrate `TriDexelStock` from ~600k heap `Vec<Vec<DexelSpan>>` allocations to a GPU-ready **flat fixed-`K` span pool**, normalize Heightfield/Cylindrical buffers to stock-local `f32` + `u32` op tags, and extract the pure volumetric stock kernels into a dedicated **`chip-stock`** crate with zero `chip-cad`/`chip-cam` dependency—breaking the `chip-cam` ↔ `chip-sim` dependency cycle to enable future 3D rest machining and `G54 -> G55` multi-setup stock carryover.

---

## Proposed Changes

### 1. Axis-Generic Tri-Dexel & Flat Fixed-`K` Span Pool

- **Axis-Symmetry Parameterization (`Axis3`):**
  - Replace duplicated X/Y/Z loops across `tridexel/grid.rs`, `tridexel/sweep.rs`, `tridexel/mesher.rs`, and `metrics.rs` with `planes: [DexelPlane; 3]` indexed by `Axis3 { X, Y, Z }` descriptors (`u_axis`, `v_axis`, `w_axis`, coordinate permutation helpers).
  - Fix `target_bodies` filtering in `TriDexelStock::from_target_cad` so imported STL/OBJ/3MF mesh bodies (`doc.meshes`) are filtered accurately just like B-Rep faces.
- **Flat Fixed-`K` Span Pool (`DexelPlane`):**
  - Replace `rays: Vec<Vec<DexelSpan>>` (~600,000 separate heap allocations at Ultra `448³`) with a flat contiguous layout:
    - `slots: Vec<FlatDexelSpan>` (`#[repr(C)] bytemuck::Pod + Zeroable`, 16 bytes: `min_local: f32`, `max_local: f32`, `op_packed: u32`, `_pad: u32`) sized `nu * nv * K` (`K = 4` default, covering >99.9% of machined rays inline)
    - `counts: Vec<u32>` (`nu * nv`, storing active span count per ray; values `> K` transparently spill into a rare `overflow: BTreeMap<u32, Vec<DexelSpan>>` side-table so deep multi-form thread mills or stacked cross-holes remain 100% exact!)
  - Cuts `TriDexelStock` heap allocations from **~600,000 -> 6 flat `Vec`s**, makes `TriDexelCheckpoint` clones 10–50× faster (`memcpy`s), and gives a 1:1 WGSL storage-buffer layout for future GPU compute tiles!
- **Stock-Local `f32` & `u32` Op Tags (`HeightfieldStock`, `CylindricalStock`):**
  - Store `origin_wcs: DVec3` alongside stock-local `f32` `z_stock` / `z_bottom` / `z_target` (encoding empty cells via `< z_bottom` / `f32::NEG_INFINITY`, removing `in_stock: Vec<bool>`) and `last_op_idx: Vec<u32>` (replacing `Vec<u16>`), matching WGSL storage/atomic types directly.

---

### 2. Unified `SimStockBackend` Trait, Single Driver, & Single Playback Cursor

- Define clean backend abstraction:
  - `StockModelKind`: `Heightfield25D`, `Cylindrical4Axis(RotaryStockSpec)`, `TriDexel3D`.
  - `SimulatedStock` enum (`Heightfield(HeightfieldStock)` | `TriDexel(TriDexelStock)` | `Cylindrical(CylindricalStock)`) and `StockCheckpointSnapshot` enum—removing the 3 parallel stock/checkpoint vectors from `SetupSimulationResult` and eliminating dummy/projected `HeightfieldStock` placeholders! (Keep ergonomic accessor methods on `SetupSimulationResult` for smooth caller migration.)
  - `SimStockBackend` trait (`CpuExactBackend` default; future `WgpuComputeBackend` plugs in behind `#[cfg(feature = "gpu-compute")]`):
    - `kind()`, `capture_checkpoint(completed_segments)`, `reset_from_checkpoint(&cp)`
    - `sweep_batch(chords, profiles, verify_ctx) -> BatchSweepOutput` (coarse-grained batching for Rayon today and GPU compute dispatch later)
    - `compute_volume_mm3()`, `compute_error_report(...)`, `extract_stock_mesh(color_mode)`
- Replace the 3 duplicated setup simulation loops (`run_setup_simulation_with_engine`, `run_tridexel_setup_simulation`, `run_cylindrical_setup_simulation`) with a single `run_setup_simulation_with_backend` driver using the shared `ViolationBatchCoalescer` and per-op `OperationErrorSnapshot` capture.
- Replace the 3 incompatible playback sync functions (`sync_stock_to_time_with_profiles`, `sync_cylindrical_stock_to_time`, `sync_to_scrub_snapshot`) with a single `SimulatedStock::sync_to_scrub_snapshot(...)` cursor state machine.

---

### 3. New Internal Crate: `chip-stock` (Pure Volumetric Stock Kernel)

#### [NEW] `crates/chip-stock/Cargo.toml`
#### [NEW] `crates/chip-stock/src/lib.rs`
#### [NEW] `crates/chip-stock/src/heightfield/{mod.rs, grid.rs, sweep.rs, mesher.rs}`
#### [NEW] `crates/chip-stock/src/tridexel/{mod.rs, grid.rs, sweep.rs, mesher.rs}`
#### [NEW] `crates/chip-stock/src/cylindrical/{mod.rs, grid.rs, sweep.rs, mesher.rs}`
#### [NEW] `crates/chip-stock/src/mesher_common.rs`
#### [NEW] `crates/chip-stock/src/metrics.rs`
#### [NEW] `crates/chip-stock/src/backend.rs`
#### [MODIFY] [Cargo.toml](file:///workspace/ChipCutter/Cargo.toml)
#### [MODIFY] [crates/chip-sim/Cargo.toml](file:///workspace/ChipCutter/crates/chip-sim/Cargo.toml)
#### [MODIFY] [crates/chip-sim/src/lib.rs](file:///workspace/ChipCutter/crates/chip-sim/src/lib.rs)
#### [MODIFY] [crates/chip-gui/src/panels/simulation.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/simulation.rs)
- Dependencies of `chip-stock`: `chip-math`, `chip-bvh`, `chip-tools`, `bytemuck`, `rayon`, `serde`, `thiserror` (**ZERO dependency on `chip-cad`, `chip-cam`, `egui`, or `wgpu`**!).
- Accepts generic sweep chords (`SweepChord`), `AnalyticCutterEnvelope` / `RadialCutterProfile` specs, and `chip_bvh::TriMesh3D` target solids.
- `chip-sim` becomes a lean, clean orchestrator depending on `chip-stock`, `chip-cad`, `chip-cam`, and `chip-tools`:
  - `SimulationTimeline` (`timeline.rs`), 5-rule `SimVerifier` (`verifier.rs`, `verify_common.rs`), `CadDocument -> TriMesh3D / AnalyticHorizFace` target adapter, 3D cutter assembly mesh builder, and `run_setup_simulation` facade re-exporting all public `chip-stock` types so `chip-project`, `chip-cli`, and `chip-gui` stay clean and stable!
- Simplify `chip-gui/src/panels/simulation.rs` to hold a single `working_stock: Option<SimulatedStock>` instead of 3 parallel stock fields and 3-way `if/else` playback glue.

---

## Verification Plan

1. **Flat-Pool Tri-Dexel & Backend Oracle Tests (`chip-stock`, `chip-sim`):**
   - Analytical box/cylinder volume accuracy across all 3 axes with flat fixed-`K` span pools.
   - Overflow-path stress test (`> K` stacked undercut grooves, e.g., 6-tooth multi-form internal thread mill) confirming zero span loss and exact volume/mesh parity.
   - Multi-body STL/OBJ mesh `target_bodies` filtering test in `TriDexelStock::from_target_cad`.
   - Playback checkpoint + mid-segment incremental scrub parity across Heightfield, Tri-Dexel, and Cylindrical engines via the unified `SimulatedStock` cursor.
   - All existing Heightmap, Tri-Dexel, Cylindrical, 4th-axis, and ThreadMill fault-injection tests passing with identical diagnostic detection.
2. **Full Workspace Parity Gate:**
   - `cargo fmt --all -- --check`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
