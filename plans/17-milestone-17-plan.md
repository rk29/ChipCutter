# ChipCutter — Milestone 17 Implementation Plan

## Goal Description
Implement **Milestone 17 (3D Adaptive Roughing & 3D Waterline / Z-Level Machining)** from [10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md#L113-L116)—building directly on Milestone 16's analytical `f64` Drop-Cutter/Push-Cutter kernel (`surface3d`) and unified STEP/STL/OBJ/3MF `CadDocument`:

1. **Operation & Tool Compatibility Model (`chip-tools`, `chip-cam`):**
   - Extend `ToolOperationClass` with `Waterline3D` and `Adaptive3D` (supporting Flat, Ball, and Bull-Nose endmills; rejecting Drills/Chamfer/Face mills cleanly).
   - Add `OperationKind::Waterline3D` (`"waterline_3d"`) and `OperationKind::Adaptive3D` (`"adaptive_3d"`), along with typed `Waterline3DParams`, `Adaptive3DParams`, and `BoundaryContainmentMode` (`Silhouette`, `StockBounds`, `SelectedContours` + `ToolInside` / `ToolCenter` / `ToolOutside` boundary offset).
2. **3D Waterline / Z-Level Slicing Strategy (`crates/chip-cam/src/strategies/waterline3d.rs`):**
   - Horizontal Z-level contour slicing across triangle meshes (`STL`/`OBJ`/`3MF`) and STEP B-Rep solids using `push_cutter_region_z(&surface, &cutter, z_k)`.
   - Steep-slope filtering (`min_slope_deg..max_slope_deg`, evaluated via exact triangle normal / local Drop-Cutter gradient), adaptive intermediate Z-stepdowns on shallow ledges (`detect_flat_plateaus` + scallop-limited intermediate Z levels), closed & open containment boundary clipping, helical/ramp entry & tangential arc lead-in/lead-out, and level-by-level or island/depth ordering.
3. **3D Adaptive Clearing / Roughing Strategy (`crates/chip-cam/src/strategies/adaptive3d.rs`):**
   - Multi-level constant-engagement 3D trochoidal roughing combining primary roughing Z-steps (`max_roughing_stepdown`), automatic **fine step-up passes** (`fine_stepdown`) climbing terraced staircases between coarse levels (peeling *only* the newly exposed `Clearable(z_stepup) \ Clearable(z_prev)` band—zero air-cutting!), and exact flat-plateau detection (e.g., landing cleanly on the `Z = 8 mm` pedestal of the Sculpted Dome).
   - Reuses Milestone 6's constant-engagement `generate_adaptive_region_slice` wavefront engine for bounded radial engagement (`optimal_load`), open-stock air entry vs. enclosed helical ramping, micro-lift stay-down linking, radial/axial stock-to-leave, and radial chip-thinning feed compensation.
4. **3D Target-Mesh Gouge Verification, Sample Job, & Desktop GUI (`chip-sim`, `chip-project`, `chip-gui`):**
   - Verify & harden `chip-sim`'s 5-rule heightmap verifier for 3D Ball/Bull-Nose/Flat sweeps against rasterized triangle meshes (including fault-injection gouge tests and 0-crash/0-gouge verification on Sculpted Dome & Relief Wave).
   - Seed `build_sculpted_mesh_dome_project` (`--sample mesh-dome`) with a ready-to-generate, ready-to-simulate two-operation 3D workflow (**Op 1: 3D Adaptive Roughing** with Ø6mm Flat Endmill leaving `0.5 mm` radial / `0.3 mm` axial stock → **Op 2: 3D Waterline Semi-Finish** with Ø6mm Ball Endmill), plus `3D Adaptive` / `3D Waterline` Inspector UI, CAM Browser buttons, and `Ctrl+K` Command Palette commands.

---

## Key Architectural Decisions & Scope Guardrails

> [!IMPORTANT]
> **Reusing Proven Exact-`f64` Building Blocks (Zero New Dependencies):**
> - Both `Waterline3D` and `Adaptive3D` build directly on `SurfaceMesh3D`, `SurfaceCutterProfile`, `push_cutter_region_z`, `drop_cutter_z`, `RegionSet2D` (`cavalier_contours`), and `generate_adaptive_region_slice` (`adaptive2d.rs`).
> - **Backward Compatibility:** `schema_version` remains `3` (all new `OperationKind` and `StrategyParams` variants are additive), `schema_version = 1/2` migrations remain lossless, and all Phase 1 + Phase 2 golden G-code snapshots remain byte-for-byte identical.
> - **Unified STEP + Mesh Target Surface:** Both strategies operate on `SurfaceMesh3D::from_cad_document(doc, &setup.target_bodies, &setup.frame)`—so they work identically on imported `.stl`/`.obj`/`.3mf` meshes AND on curved/sculpted `.step` B-Rep solids!

> [!NOTE]
> **Zero-Selection Default for 3D Operations:**
> - Unlike 2.5D Pocket/Contour which require explicit face/loop selections, `Waterline3D` and `Adaptive3D` machine the entire target model inside Stock bounds (or Model Silhouette) out of the box with **zero mandatory selections**, while optionally accepting any 2D Sketch region or B-Rep loop in `op.selections` as a 2D machining containment boundary (`BoundaryContainmentMode`)!

> [!TIP]
> **Flat-Plateau & Fine Step-Up Accuracy:**
> - Detecting horizontal flat plateaus in `SurfaceMesh3D` (clustering upward triangles with `normal.z > 0.995` and total area `>= 4.0 mm²`) guarantees both `Adaptive3D` and `Waterline3D` place an exact Z-level at `z_plateau + stock_axial_mm`, leaving clean flat steps without leaving thick unmachined ledges.

---

## Proposed Changes

### 1. Tool Compatibility, Operation Kinds, & 3D Strategy Parameters (`chip-tools`, `chip-cam`)

#### [MODIFY] [crates/chip-tools/src/model.rs](file:///workspace/ChipCutter/crates/chip-tools/src/model.rs)
- Add `ToolOperationClass::Waterline3D` and `ToolOperationClass::Adaptive3D`.
- Update `CutterGeometry::supports_operation`:
  - `Adaptive3D`: `FlatEndmill`, `BullNoseEndmill`, `BallEndmill`.
  - `Waterline3D`: `FlatEndmill`, `BullNoseEndmill`, `BallEndmill`.

#### [MODIFY] [crates/chip-cam/src/operation.rs](file:///workspace/ChipCutter/crates/chip-cam/src/operation.rs), [crates/chip-cam/src/params.rs](file:///workspace/ChipCutter/crates/chip-cam/src/params.rs), [crates/chip-cam/src/lib.rs](file:///workspace/ChipCutter/crates/chip-cam/src/lib.rs)
- Add `OperationKind::Waterline3D` (`"waterline_3d"`, label `"3D Waterline / Z-Level"`) and `OperationKind::Adaptive3D` (`"adaptive_3d"`, label `"3D Adaptive Roughing"`).
- Add shared 3D containment types in [params.rs](file:///workspace/ChipCutter/crates/chip-cam/src/params.rs):
  - `MachiningBoundaryMode` (`StockBox`, `ModelSilhouette`, `SelectedRegions`)
  - `ToolContainmentMode` (`ToolCenterOnBoundary`, `ToolInsideBoundary`, `ToolOutsideBoundary`)
  - `WaterlineOrderMode` (`ByLevel`, `ByDepthRegion`)
  - `Waterline3DParams`:
    - `boundary_mode: MachiningBoundaryMode`, `tool_containment: ToolContainmentMode`, `boundary_offset: ParamLength`
    - `min_slope_deg: f64` (`0.0` default; e.g. `30.0` for steep-only walls), `max_slope_deg: f64` (`90.0` default)
    - `adaptive_stepdown: bool`, `min_stepdown: ParamLength`, `max_cusp_height: ParamLength`
    - `detect_flats: bool`, `order_mode: WaterlineOrderMode`, `climb_milling: bool`, `ramp_angle_deg: f64`
  - `Adaptive3DParams`:
    - `boundary_mode: MachiningBoundaryMode`, `tool_containment: ToolContainmentMode`, `boundary_offset: ParamLength`
    - `adaptive: AdaptiveClearingParams` (reusing `optimal_load`, `micro_lift_z`, `min_trochoid_radius`, `auto_chip_thinning`, `max_thinning_feed_multiplier`, `no_engagement_feed_multiplier`, `cut_direction`)
    - `max_roughing_stepdown: ParamLength`, `fine_stepdown: ParamLength` (step-up resolution between coarse Z-steps), `detect_flats: bool`
    - `flat_area_min_mm2: f64`
- Add `StrategyParams::Waterline3D(Waterline3DParams)` and `StrategyParams::Adaptive3D(Adaptive3DParams)` with `reevaluate(&EvalContext)` support and sensible defaults on `Operation::new` (e.g., `Adaptive3D` defaults to `stock_to_leave_radial = 0.5 mm`, `stock_to_leave_axial = 0.3 mm`, heights `StockTop` → `ModelBottom`; `Waterline3D` defaults to `stock_to_leave = 0.0`, `max_stepdown = 1.0 mm`, heights `ModelTop` → `ModelBottom`).

---

### 2. Shared 3D Boundary / Plateau Helpers & 3D Strategies (`crates/chip-cam`)

#### [NEW] `crates/chip-cam/src/surface3d/levels.rs` & [MODIFY] [crates/chip-cam/src/surface3d/mod.rs](file:///workspace/ChipCutter/crates/chip-cam/src/surface3d/mod.rs)
- **`detect_flat_plateaus(surface, z_min, z_max, min_area_mm2) -> Vec<f64>`:** Clusters upward-facing horizontal triangles (`normal.z > 0.995`) into distinct plateau Z heights within `[z_min, z_max]`.
- **`resolve_3d_containment_region(surface, setup, selection, mode, containment, extra_offset, tool_radius) -> RegionSet2D`:**
  - `StockBox`: exact WCS stock XY rectangle (or inscribed cylinder circle for cylindrical stock).
  - `ModelSilhouette`: projected XY union of model footprint / lowest valid slices.
  - `SelectedRegions`: union of selected 2D Sketch regions or B-Rep planar/loop contours (falling back cleanly to `ModelSilhouette` if selections are empty).
  - Applies `ToolInsideBoundary` (`erode_inward(tool_r)`), `ToolCenterOnBoundary` (no shift), or `ToolOutsideBoundary` (`dilate_outward(tool_r)`) plus `extra_offset`.
- **`sample_surface_slope_deg(surface, cutter, pt_xy, z_level) -> f64`:** Evaluates local surface steepness near `(pt_xy, z_level)` from neighboring triangles / `drop_cutter_z` central-difference gradient so `Waterline3D` can filter shallow vs. steep segments accurately (`min_slope_deg..max_slope_deg`).

#### [NEW] `crates/chip-cam/src/strategies/waterline3d.rs`
- Computes primary Z-levels from `top_z` down to `bottom_z` using `passes.max_stepdown`.
- When `adaptive_stepdown` is enabled, inserts intermediate Z-levels wherever the horizontal XY radial jump between consecutive `push_cutter_region_z` boundaries implies a scallop/stepover exceeding `max_cusp_height` (clamped by `min_stepdown`).
- When `detect_flats` is enabled, injects exact `z_plateau + stock_axial_mm` levels.
- At each level `z_k`:
  - Computes `avoid = push_cutter_region_z(&surface, &cutter, z_k)`.
  - Extracts outer CCW & inner CW contour loops of `avoid` (oriented for climb vs. conventional cutting) and clips them to the resolved 2D containment region.
  - Applies slope filtering (`min_slope_deg..max_slope_deg`) when narrower than `0°..90°`, splitting closed loops into clean open contour chains where shallow zones are skipped.
  - Emits smooth **helical / tangential arc lead-in** (verified collision-free against `avoid`!), cutting contours (`MotionRole::Finish` / `Rough`), tangential lead-out, and safe stay-down or retract links (`ByLevel` or `ByDepthRegion` spatial clustering).

#### [NEW] `crates/chip-cam/src/strategies/adaptive3d.rs`
- Builds primary roughing Z-levels from `top_z` down to `bottom_z` using `max_roughing_stepdown`, plus detected flat-plateau Z-levels (`z_plateau + stock_axial_mm`).
- For each primary roughing band `(z_prev -> z_curr)`:
  1. Computes `avoid_curr = push_cutter_region_z(&surface, &cutter, z_curr)` and `clearable_curr = containment.difference(&avoid_curr)`.
  2. Clears each `PocketRegion2D` component of `clearable_curr` at `z_curr` using `generate_adaptive_region_slice` (determining `has_open_air` automatically wherever `clearable_curr` reaches the outside of the stock boundary so the cutter enters smoothly from air instead of plunging!).
  3. **Fine Step-Up Passes (`fine_stepdown < max_roughing_stepdown`):** Steps *upward* through intermediate Z-levels `z_up` in `(z_curr, z_prev)` spaced by `fine_stepdown` (plus any intermediate flat plateaus inside `(z_curr, z_prev)`). At each `z_up`:
     - Computes `clearable_up = containment.difference(&push_cutter_region_z(&surface, &cutter, z_up))`.
     - Computes the **already-cleared open air** `prev_air = clearable_prev` (from the coarse level above!).
     - If `clearable_up.difference(&prev_air)` has non-trivial remaining area (`> 0.25 mm²`), runs `generate_adaptive_region_slice` with `cleared_air_centers = &prev_air` so the tool starts immediately from the already-machined cavity above and peels *only* the terraced slope staircase!
- Supports `RestMachiningMode::FromPrecedingOperations` seamlessly via `upstream_rest` swept coverage unioned into `prev_air`.

#### [MODIFY] [crates/chip-cam/src/strategies/mod.rs](file:///workspace/ChipCutter/crates/chip-cam/src/strategies/mod.rs)
- Dispatch `OperationKind::Waterline3D` → `waterline3d::generate_waterline3d` and `OperationKind::Adaptive3D` → `adaptive3d::generate_adaptive3d`.
- Provide `doc: &'a CadDocument` on `StrategyExecContext` (so 3D strategies build `SurfaceMesh3D` once per operation cleanly).

---

### 3. 3D Simulation Verification & Sculpted Sample (`chip-sim`, `chip-project`, `chip-cli`)

#### [MODIFY] [crates/chip-sim/src/heightmap/sweeper.rs](file:///workspace/ChipCutter/crates/chip-sim/src/heightmap/sweeper.rs) & [crates/chip-sim/src/lib.rs](file:///workspace/ChipCutter/crates/chip-sim/src/lib.rs)
- Audit and verify `sweeper.rs` 3D inclined-segment sweeping for `BallEndmill` and `BullNoseEndmill` (ensuring `z_min` along sloped helical/ramp links evaluates the exact 3D segment parameter `t` rather than flat-bottom assumptions) and add dedicated unit tests in `chip-sim`:
  - **Zero-Crash / Zero-Gouge 3D Oracle:** Run `Adaptive3D` (Ø6mm Flat) + `Waterline3D` (Ø6mm Ball) on the watertight `Sculpted Dome (STL Mesh)` and verify `0` rapid crashes, `0` target gouges, `0` flute rubs, `0` holder crashes, and substantial stock volume removal (`> 50%` of excess material).
  - **3D Target-Mesh Gouge Fault Injection:** Inject an intentional `-0.5 mm` gouge into the dome crest and verify `chip-sim` flags `SimIssueKind::TargetGouge` accurately on the rasterized triangle mesh!

#### [MODIFY] [crates/chip-project/src/demos.rs](file:///workspace/ChipCutter/crates/chip-project/src/demos.rs) & [crates/chip-project/src/validation.rs](file:///workspace/ChipCutter/crates/chip-project/src/validation.rs)
- Update `build_sculpted_mesh_dome_project` so `--sample mesh-dome` ships with two ready-to-run operations:
  1. `Op 1: 3D Adaptive Roughing` (T1 Ø6mm Flat Endmill, `max_roughing_stepdown = 5.0 mm`, `fine_stepdown = 1.5 mm`, `optimal_load = 1.2 mm`, `0.5 mm` radial / `0.3 mm` axial stock-to-leave),
  2. `Op 2: 3D Waterline Semi-Finish` (T3 Ø6mm Ball Endmill, `max_stepdown = 1.25 mm`, `adaptive_stepdown = true`, `detect_flats = true`, `0.0 mm` stock-to-leave).
- Ensure `validate_project` allows empty `op.selections` for `OperationKind::Adaptive3D` and `OperationKind::Waterline3D` (matching `Facing`) while validating `doc.has_3d_geometry()`!

---

### 4. Desktop GUI Controls & Sample Showcase (`crates/chip-gui`)

#### [NEW] `crates/chip-gui/src/panels/operation_inspector/passes_tab/surface_3d.rs` & [MODIFY] [passes_tab/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/passes_tab/mod.rs), [geometry_tab.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/geometry_tab.rs), [browser.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/browser.rs), [menus_toolbar.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/menus_toolbar.rs), [command_palette.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/command_palette.rs), [app/loader.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/loader.rs)
- **Passes Tab (`surface_3d.rs`):**
  - Clean two-column Inspector cards for **`3D Adaptive Roughing`** (`Optimal Load`, `Max Roughing Stepdown`, `Fine Step-Up`, `Detect Flat Plateaus`, `Micro-Lift`, `Chip Thinning`, Radial/Axial Stock-to-Leave) and **`3D Waterline / Z-Level`** (`Max Stepdown`, `Adaptive Stepdown` + `Min Stepdown` / `Max Cusp`, `Slope Range (°)`, `Detect Flat Plateaus`, `Order Mode`, `Climb / Conventional`, Radial/Axial Stock-to-Leave).
- **Geometry Tab (`geometry_tab.rs`):**
  - Add 3D Machining Boundary card (`Stock Bounds` / `Model Silhouette` / `Selected 2D Contours`), `Tool Containment` (`Inside` / `Center` / `Outside`), and `Boundary Offset` expression input, plus optional Sketch/Loop containment picking.
- **Browser, Menus, & `Ctrl+K` Command Palette:**
  - Add `3D Adaptive Roughing` and `3D Waterline / Z-Level` to the **Operations** menu, Setup `+ Op…` menu, and `Ctrl+K` Command Palette (`CommandId::AddOpAdaptive3D`, `CommandId::AddOpWaterline3D`).
  - Auto-generate & simulate `Sculpted Dome (STL Mesh)` (`LoadPurpose::SeedSculptedMeshDome`) so opening the sample immediately displays the 3D trochoidal roughing + waterline ball-nose toolpaths and verified 3D machined stock!

---

## Verification Plan

1. **Automated Strategy & Physics Oracle Tests (`chip-cam`, `chip-sim`, `chip-project`):**
   - `chip-cam::strategies::tests` & `tests/strategies_invariants.rs`:
     - Verify `Adaptive3D` + `Waterline3D` on **Sculpted Dome (STL)**, **Relief Wave (OBJ)**, and a **3D STEP solid** (`sample_cam_torture_block_step`):
       - Non-empty CL-data & 3D toolpath segments, exact flat-plateau landing at `Z = 8.0 + stock_axial`, fine step-up passes present between coarse Z-steps, steep-slope filtering (`30°..90°`) skips flat pedestal while preserving dome walls, and 2D containment boundary clips all XY positions strictly inside the chosen boundary.
       - Pass `chip-sim` 5-rule verification with **0 rapid crashes, 0 target gouges, 0 flute rubs, 0 holder crashes** using Flat, Ball, and Bull-Nose cutters!
   - `chip-sim::tests`:
     - Add 3D target-mesh gouge fault-injection test confirming `SimIssueKind::TargetGouge` triggers when a ball-nose pass dips below the dome surface.
   - `chip-project::tests`:
     - Deterministic Schema v3 `.chip.toml` round-trip of `Adaptive3D` + `Waterline3D` operations, `build_sculpted_mesh_dome_project` generation + simulation + GRBL & LinuxCNC G-code post-processing.
2. **Full Workspace Regression Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings`
   - `cargo deny check` (`0` new third-party dependencies)
   - `WGPU_BACKEND=vulkan cargo test --workspace` (all Phase 1 + Phase 2 golden G-code snapshots byte-for-byte identical).
3. **Headless Vulkan Visual Verification (`xvfb-run` + Lavapipe):**
   - Capture and inspect `--sample mesh-dome --capture-generate --self-test-screenshot` (both 3D toolpath view and `Sim Verified (0 Crashes)` machined 3D dome stock view in Studio Dark & Daylight Light themes) via `read_file`.
