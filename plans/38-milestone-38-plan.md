# Milestone 38 Implementation Plan — Tri-Dexel Volumetric Stock Kernel & Swept-Cutter Subtraction

## Goal

Add a pure-Rust, UI-agnostic **Tri-Dexel volumetric CAM simulation engine** (`TriDexelStock`) to `chip-sim` alongside the existing **Z-Dexel Heightmap** (`HeightfieldStock`) and **Cylindrical Rotary** (`CylindricalStock`) engines—keeping the existing heightmap simulation fully selectable and unchanged in behavior while unlocking accurate 3D vertical walls, multi-interval cavities, horizontal features, and undercuts.

---

## Proposed Changes

### 1. Selectable Simulation Engine Enum (`chip-sim`, `chip-project`)

#### [MODIFY] [crates/chip-sim/src/lib.rs](file:///workspace/ChipCutter/crates/chip-sim/src/lib.rs)
#### [MODIFY] [crates/chip-project/src/pipeline.rs](file:///workspace/ChipCutter/crates/chip-project/src/pipeline.rs)
- Introduce `#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)] pub enum SimEngineKind { #[default] Heightmap, TriDexel }`.
- Extend `SetupSimulationInput` and `SetupSimRequest` with `pub engine: SimEngineKind` (defaulting to `SimEngineKind::Heightmap` so all existing callers, tests, and default fast previews remain 100% compatible).
- Keep cylindrical rotary setups routed cleanly or optionally voxelized in Tri-Dexel when `SimEngineKind::TriDexel` is explicitly requested for indexed `3+1` / simultaneous 4-axis verification.

---

### 2. Orthogonal Tri-Dexel Volumetric Data Structure (`chip-sim`)

#### [NEW] `crates/chip-sim/src/tridexel/mod.rs`
#### [NEW] `crates/chip-sim/src/tridexel/grid.rs`
- Define `DexelSpan { min_mm: f64, max_mm: f64, min_op_idx: u16, max_op_idx: u16 }` representing a solid 1D interval along a ray, with strict invariants (`min_mm < max_mm`, sorted, non-overlapping, automatically merged within `1e-9 mm`).
- Define `DexelPlane` (a 2D `(u, v)` grid of `Vec<DexelSpan>` ray lists) and `TriDexelStock` holding three mutually orthogonal planes covering the Setup stock AABB in WCS space:
  - `z_rays`: grid over `(X, Y)` storing solid `Z` intervals
  - `x_rays`: grid over `(Y, Z)` storing solid `X` intervals
  - `y_rays`: grid over `(X, Z)` storing solid `Y` intervals
- Support resolution tiers matching `SimResolution` (`Draft`, `Standard`, `Fine`) with tuned cell pitch `h` so memory stays bounded and deterministic even on large stock blanks.
- Implement `TriDexelStock::from_box_stock(aabb_wcs, pitch_mm)` and `TriDexelStock::from_cylinder_stock(...)` for raw stock initialization, plus exact volume computation averaging the three orthogonal axes (`(vol_x + vol_y + vol_z) / 3.0`) with cross-axis consistency diagnostics.
- Implement `TriDexelStock::from_target_cad(doc, wcs_frame, pitch_mm)` via exact ray-triangle parity / winding-number intersection over `CadDocument`'s transformed B-Rep/mesh triangles (accelerated by `rstar` / `MeshBvh`) to produce a reference Tri-Dexel target solid on the exact same `(X, Y, Z)` ray lattice!

---

### 3. Swept-Cutter Boolean Subtraction & 5-Rule Verifier (`chip-sim`)

#### [NEW] `crates/chip-sim/src/tridexel/sweep.rs`
#### [NEW] `crates/chip-sim/src/tridexel/mesh.rs`
- Implement exact analytical ray-vs-swept-cutter interval calculation along linear (`G1`/`G0`) and arc (`G2`/`G3`) motion spans for all 6 cutter profiles (`FlatEndmill`, `BullNoseEndmill`, `BallEndmill`, `ChamferMill`, `VBit`, `TwistDrill`, `FaceMill`), subtracting `[t_enter, t_exit]` from affected `X`, `Y`, and `Z` ray lists in parallel via `rayon`.
- Support arbitrary tool-axis orientation (`+Z` for 3-axis and rotated tool axis vectors for indexed `3+1` / simultaneous 4-axis moves) by transforming rays into the cutter's local frame!
- Wire all 5 mandatory physical verification rules against the Tri-Dexel stock & target solid:
  1. `G0` rapid-into-stock crash
  2. Target part gouge (`> 0.02 mm`, now detecting vertical wall, chamfer, and undercut gouges!)
  3. Non-cutting shank / flute-length rub
  4. Tool-holder collision
  5. Non-center-cutting plunge check
- Implement crisp Tri-Dexel surface extraction (`tessellate_tridexel_stock_mesh`) using Dual Contouring / Surface Nets over the 3 orthogonal ray intersection points + analytical/gradient surface normals, producing a watertight `SimStockMesh` with sharp machined edges and per-operation face colors, plus timeline checkpoints (`StockCheckpoint`) for smooth playback scrubbing.

---

## Verification Plan

1. **Analytical Volume & Boolean Oracles (`chip-sim`):**
   - Box & cylinder initialization volumes match analytical formulas within `< 0.5%` across all 3 orthogonal axes (`X`, `Y`, `Z`).
   - Single-pass slot, pocket, ball-nose hemisphere, and vertical-wall undercut cuts match exact analytical swept volumes and produce zero heightmap stair-stepping on vertical walls.
   - Fault-injection tests confirm all 5 safety rules fire accurately (including a deliberate vertical-wall undercut gouge invisible to 2.5D heightmaps).
2. **Quality Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo test --locked -p chip-sim -p chip-project`
