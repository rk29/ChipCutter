# Milestone 62 Implementation Plan — Multi-Setup In-Process Volumetric Stock Carryover (`G54` -> `G55`) & Rest-Aware Op-2 Machining

## Goal

Connect multi-setup machining (`Setup 1 — Top Op [G54]` → `Setup 2 — Flip Bottom Op [G55]` → `Setup N`) into a continuous volumetric pipeline:
1. **Exact Rigid-Frame Tri-Dexel Stock Transfer (`chip-stock`):** Transfer Setup `i`'s exact final remaining `TriDexelStock` into Setup `j`'s oriented WCS frame (`T_{j -> i}`)—with an exact zero-resampling axis-permutation fast path for `90°`/`180°` flip setups and exact BVH surface re-voxelization for arbitrary oblique orientations.
2. **Rest-Aware Op-2 Toolpaths (`chip-cam`, `chip-project`):** Feed Setup 1's carried-over remaining stock into Op-2 roughing/facing/pocketing strategies so Op-2 automatically skips air regions already cleared in Op-1.
3. **Continuous Multi-Setup Simulation & Cumulative Error-Volume Metrology (`chip-sim`, `chip-project`, `chip-gui`, `chip-cli`):** Start Setup 2's simulation and timeline playback (`t = 0.0`) from Setup 1's actual machined solid, verify Op-2 collisions against real remaining stock (e.g., carrier stock/tabs/hat stock), and compute cumulative end-of-job `StockErrorReport` metrics across the full multi-setup chain.

---

## Architectural Readiness (Enabled by Phase 14 Milestone 50)

Phase 14 Milestone 50 explicitly extracted `chip-stock` as a zero-`chip-cad`/zero-`chip-cam` pure-CPU volumetric stock crate specifically to break the `chip-cam` ↔ `chip-sim` dependency cycle for this milestone:
- `chip-cam` can cleanly depend on pure-CPU `chip-stock` (default features — zero `wgpu`!) to query carried-over `TriDexelStock` material occupancy directly during Op-2 rest roughing!
- `chip-project` orchestrates Setup dependency order (`Setup 1` → `Setup 2`) when generating Op-2 rest toolpaths and when running multi-setup simulation chains.

---

## Proposed Changes

### 1. Exact Rigid-Frame `TriDexelStock` Carryover Transfer (`chip-stock`)

#### [NEW] `crates/chip-stock/src/tridexel/transfer.rs`
#### [MODIFY] `crates/chip-stock/src/tridexel/mod.rs`
#### [MODIFY] `crates/chip-stock/src/lib.rs`
#### [MODIFY] `crates/chip-stock/src/heightfield/grid.rs`
- **Rigid-Frame Transfer (`TriDexelStock::transform_to_frame`):**
  - Input: `src: &TriDexelStock` (in `src_frame: RigidFrame3`), `dst_frame: RigidFrame3`, target resolution / pitch (`SimResolution`).
  - Computes exact relative rigid transform `M_src_to_dst = dst_frame^{-1} * src_frame` (rotation `R`, translation `t`).
  - **Fast Exact Path (90°/180° Signed Axis Permutations — covers 95%+ of flip/6-sided setups):**
    - Detects when every column of `R` is within `1e-6` of a signed cardinal axis (`±X`, `±Y`, `±Z`).
    - Directly permutes `(Plane X, Plane Y, Plane Z)` indices, reverses ray coordinate order and negates/shifts interval endpoints `[min_mm, max_mm]` where the corresponding axis sign is `-1`, and preserves `op_idx` provenance tags with **zero resampling blur, zero volume drift, and `< 2 ms` runtime even at Ultra!**
  - **General Oblique Orientation Path (Arbitrary Tilted / Angled Vise Setups):**
    - Extracts watertight `TriMesh3D` surface of `src` (via existing `extract_tridexel_mesh`), transforms vertices by `M_src_to_dst` into `dst_frame` WCS, builds a `FlatBvh`, and voxelizes into a fresh `TriDexelStock` at `dst` resolution (preserving exact volume within `< 0.5%` and exact bounds).
- **Heightfield Projection & Fast 2D/Z-Slice Occupancy Queries (`chip-stock`):**
  - Add `HeightfieldStock::from_tridexel_projection(td: &TriDexelStock)` so users choosing fast `Heightmap` mode on Setup 2 still start from the accurate top Z envelope of Setup 1's carried-over stock (while noting in the UI that `Tri-Dexel` preserves underside cavities!).
  - Add fast pure-CPU query helpers on `TriDexelStock` for `chip-cam` rest-aware toolpaths:
    - `has_material_in_z_slab(z_lo, z_hi, min_thickness_mm) -> bool`
    - `extract_remaining_material_regions_in_z_slab(z_lo, z_hi, cell_pad_mm) -> Vec<ArcPolyline2D>` (2D contour mask of where uncut stock still remains in a Z slice — directly consumable by `Adaptive3D`, `Waterline3D`, `Pocket2D`, `Adaptive2D`, and `Facing`!).
    - `max_stock_z_in_xy_aabb(aabb2) -> Option<f64>` (lets Op-2 `Facing` and `Adaptive3D` skip empty air Z-levels above already-faced surfaces!).

---

### 2. Schema-Safe `StockConfig::FromPrecedingSetup` & Rest-Aware Op-2 Toolpaths (`chip-cam`, `chip-project`)

#### [MODIFY] `crates/chip-cam/Cargo.toml` (add pure-CPU `chip-stock` dependency — default features, zero `wgpu`)
#### [MODIFY] `crates/chip-cam/src/stock.rs`
#### [MODIFY] `crates/chip-cam/src/setup.rs`
#### [MODIFY] `crates/chip-cam/src/operation.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/{facing.rs, pocket2d.rs, adaptive2d.rs, adaptive3d.rs, waterline3d.rs}`
#### [MODIFY] `crates/chip-project/src/validation.rs`
#### [MODIFY] `crates/chip-project/src/cache.rs`
#### [MODIFY] `crates/chip-project/src/dispatcher.rs`
- **Carryover Stock Mode (`StockConfig::FromPrecedingSetup`):**
  - Add `StockConfig::FromPrecedingSetup { source_setup_id: String, fallback_box: Box<StockConfig> }` (or `carryover_from_setup_id: Option<String>` on `Setup` with `#[serde(default, skip_serializing_if = "Option::is_none")]` so `Setup.stock` still cleanly defines WCS stock-box anchors before Setup 1 is simulated, and seamlessly upgrades to Setup 1's exact carried-over `TriDexelStock` once simulated!).
  - *Design Note (Why `carryover_from_setup_id: Option<String>` on `Setup` + `StockConfig` UI mode is cleaner):*
    - Keeping the raw bounding box resolvable synchronously (from the source Setup's resolved stock transformed into `dst_frame`) means WCS `StockBoxPoint` 27-point anchors and `stock_x`/`stock_y`/`stock_z` expressions resolve deterministically in `< 0.1 ms` even *before* Setup 1's simulation runs!
    - Validate strictly against cycles (`Setup 1 -> Setup 2 -> Setup 1`) and forward/self references in `chip-project/src/validation.rs` (source setup must precede target setup in `project.setups`, or form a valid acyclic DAG).
- **Rest-Aware Op-2 Toolpath Generation:**
  - Add optional `carryover_stock: Option<Arc<TriDexelStock>>` on `OperationGenRequest` (default `None` — zero impact on single-setup projects!).
  - When `carryover_stock` is present AND the operation opts into rest/stock awareness (enabled automatically for `Facing`, `Adaptive3D`, `Waterline3D`, and `Pocket2D`/`Adaptive2D` when `rest.mode != Off` or `clip_to_carryover_stock = true`):
    - **Z-Level Air-Cut Skipping:** Query `max_stock_z_in_xy_aabb` / `has_material_in_z_slab` to skip upper Z stepdowns that are 100% air (e.g., pockets or exterior shoulders already cleared from Op-1!).
    - **XY Region Clipping (`chip-geom2d`):** Intersect candidate roughing pockets/faces with the carried-over material mask (inflated by `tool_r + stock_to_leave + 0.5 mm` safety entry overlap) so Op-2 roughs only the remaining hat stock / back-side material!
  - In `JobDispatcher` & `ToolpathCache`:
    - Include the source Setup's operation-hash digest in Setup 2's cache keys whenever `carryover_from_setup_id` is set, so editing an Op-1 operation automatically marks dependent Op-2 rest operations `Dirty`.
    - When generating Setup 2 operations that require carried-over stock, automatically ensure Setup 1's toolpaths are generated and a fast `TriDexel` carryover snapshot is computed (reusing GPU compute when available in `chip-project`!) before launching Setup 2's wave.

---

### 3. Multi-Setup Simulation Chain, Cumulative Metrology, GUI, & CLI (`chip-sim`, `chip-project`, `chip-gui`, `chip-cli`)

#### [MODIFY] `crates/chip-sim/src/driver.rs`
#### [MODIFY] `crates/chip-project/src/pipeline.rs`
#### [MODIFY] `crates/chip-gui/src/panels/setup_inspector/stock_tab.rs`
#### [MODIFY] `crates/chip-gui/src/panels/simulation/{mod.rs, worker.rs, ui.rs}`
#### [MODIFY] `crates/chip-cli/src/main.rs`
#### [MODIFY] `crates/chip-cli/src/commands/validate_sim_build.rs`
- **Initial Stock Injection in `chip-sim`:**
  - Add optional `initial_stock: Option<&'a TriDexelStock>` on `SetupSimulationInput<'a>` (default `None`).
  - When `Some(carried)`, initialize Setup 2's `SimulatedStock` directly from `carried` (either exact `TriDexelStock` or projected `HeightfieldStock` if Heightmap engine is selected), preserve `initial_stock_volume_mm3` from the chain's original raw blank for cumulative `removed_volume_mm3` / `material_removal_efficiency_pct`, and record `carried_over_from_setup: Option<String>` on `SetupSimulationResult`.
  - Timeline checkpoint 0 (`t = 0.0`) captures Setup 2's **carried-over starting solid** (showing Op-1's pockets/contours already cut before Op-2 makes its first move!).
- **Automatic Chain Simulation (`simulate_project_setup_with_choice` & `chipcutter simulate --all-setups`):**
  - When simulating Setup `k` with `carryover_from_setup_id = Some(src_id)`, `chip-project/src/pipeline.rs` automatically resolves the upstream chain (`src_id` simulated in `TriDexel` -> `transform_to_frame` into Setup `k` WCS) and passes `initial_stock` into Setup `k`'s simulation (caching intermediate carryover stock so switching tabs or scrubbing is instant!).
  - Add `--all-setups` flag to `chipcutter simulate` emitting per-setup and cumulative final `StockErrorReport` (`ErrVol`, `RMS`, `Max Gouge`, `In-Tol %`) across the entire multi-setup job.
- **Setup `[Stock]` Tab & Simulation UI (`chip-gui`):**
  - Add a **Stock Carryover** card at the top of Setup Inspector's `[Stock]` tab (visible whenever `setup_idx > 0`) with a dropdown: `Independent Raw Stock` vs. `Carry Over From: <Preceding Setup>` + live badge showing whether the upstream carryover stock is up-to-date.
  - Show a subtle `"Stock carried over from Setup 1"` pill in the Simulation Timeline bar when viewing Op-2's simulation.

---

## Verification Plan

1. **Exact `TriDexelStock` Transfer Tests (`chip-stock`):**
   - `180°` X-flip and `180°` Y-flip fast-path test: carve an asymmetric L-pocket and counterbore in Setup 1, transfer to Setup 2, assert **exact volume conservation** (`|V_dst - V_src| < 1e-3 mm³`) and exact mirrored ray endpoints!
   - `35°` oblique rigid-transform test: assert volume conservation within `< 0.5%` and exact surface point containment after BVH re-voxelization.
2. **Multi-Setup Carryover Simulation & Air-Cut Reduction Tests (`chip-sim`, `chip-project`, `chip-cam`):**
   - Two-sided stepped block (`Setup 1 — Top Op [G54]` + `Setup 2 — Flip Bottom Op [G55]` with `carryover_from_setup_id = Some("setup_1")`):
     - Verify Setup 2 starts at `t = 0.0` with Setup 1's remaining volume (NOT full raw blank volume).
     - Verify Op-2 rest roughing reduces total Op-2 cutting/air-cut path length significantly (`>= 30%` reduction vs. blind raw-stock Op-2) while keeping `0` critical errors and `max_gouge <= 0.02 mm`.
     - Verify cumulative Op-1 + Op-2 `StockErrorReport` achieves `In-Tol >= 95%` on the finished 2-sided part (whereas Setup 1 alone leaves the bottom hat stock uncut!).
     - Verify CPU vs. GPU parity on Setup 2 carried-over Tri-Dexel simulation.
3. **Headless Vulkan Visual Verification:**
   - Capture Setup 2 at `t = 0.0` (clearly showing Op-1's flipped machined features + bottom hat stock) and at `t = 1.0` (fully finished two-sided part with Error Heatmap).
4. **Full Workspace Gate:**
   - `cargo fmt --all -- --check`, pure-CPU `cargo check --locked -p chip-stock`, `cargo deny check` (default + `flatpak-portal`), `cargo clippy` (all targets & feature sets), and `WGPU_BACKEND=vulkan cargo test --locked --workspace`.
