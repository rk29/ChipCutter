# ChipCutter — Milestone 18 Implementation Plan

## Goal Description
Implement **Milestone 18 (3D Surface Finishing: Parallel Raster, Constant-Scallop, & Pencil Cleanup)** from [10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md#L118-L121)—completing ChipCutter's full 3-axis sculpted surface CAM suite on top of Milestone 16's exact `f64` Drop-Cutter kernel (`surface3d`) and Milestone 17's 3D Adaptive & Waterline infrastructure:

1. **Operation & Tool Compatibility Extensions (`chip-tools`, `chip-cam`):**
   - Extend `ToolOperationClass` with `Parallel3D`, `Scallop3D`, and `Pencil3D` (supporting Ball, Bull-Nose, Flat, and Tapered/V-Bit cutters for finishing, with Ball/Bull-Nose preferred).
   - Add `OperationKind::Parallel3D` (`"parallel_3d"`), `OperationKind::Scallop3D` (`"scallop_3d"`), and `OperationKind::Pencil3D` (`"pencil_3d"`), along with typed `Parallel3DParams`, `Scallop3DParams`, and `Pencil3DParams` (reusing `MachiningBoundaryMode`, `ToolContainmentMode`, and `RestMachiningConfig`).
2. **Exact Drop-Cutter 3D Parallel / Raster Finishing (`crates/chip-cam/src/strategies/parallel3d.rs`):**
   - Parallel raster scanlines rotated to configurable `pass_angle_deg`, sampled along exact `f64` `drop_cutter_points_parallel` (`rayon`), with automatic cusp-height stepover (`cusp_height` → exact ball/bull-nose stepover formula), slope-range filtering (`min_slope_deg..max_slope_deg`, e.g., `0°..45°` shallow finishing paired with steep Waterline), optional perpendicular `cross_pass` (`pass_angle_deg + 90°`), 3D Ramer–Douglas–Peucker chord decimation (`tolerance_mm`), and collision-free surface-hugging stepover links or smooth retracts.
3. **Constant-Scallop & 3D Pencil Finishing (`crates/chip-cam/src/strategies/scallop3d.rs`, `pencil3d.rs`):**
   - **Constant-Scallop (`Scallop3D`):** Propagates 3D surface-projected offset contours inward from the containment boundary with slope-adaptive XY stepover `Δs_xy(p) = s_3d * cos(θ(p))` so steep walls receive proportionally tighter XY passes than shallow domes, yielding uniform cusp height everywhere.
   - **3D Pencil (`Pencil3D`):** Detects concave bi-tangent valleys and unmachined corner fillets (e.g., the 90° dome-to-pedestal crease on `Sculpted Dome` and concave troughs on `Relief Wave`) by evaluating exact Drop-Cutter residual height `z_drop(larger_ball) - z_drop(current_tool)` + local concavity curvature, tracing smoothed ridge/valley chains with optional multi-pass lateral offsets (`num_offset_passes`, `stepover`).
4. **3D Rest Boundaries, Sample Showcase, GUI Controls, & 3-Axis CLI Goldens (`chip-cam`, `chip-project`, `chip-gui`, `chip-cli`):**
   - Add analytical 3D Rest Masking (`RestMachiningMode::FromPreviousTool` / `FromPrecedingOperations`) so `Parallel3D`, `Scallop3D`, and `Pencil3D` restrict finishing strictly to uncut 3D regions!
   - Add `Parallel3D`, `Scallop3D`, and `Pencil3D` Inspector cards, CAM Browser / menu / `Ctrl+K` actions, a new **`Relief Wave (OBJ Mesh)`** (`--sample mesh-wave`) sample showcasing 3D Adaptive → 3D Parallel / Scallop finishing, plus `Op 3: 3D Pencil Fillet Cleanup` on `Sculpted Dome` (`--sample mesh-dome`).
   - Add headless `chipcutter` Phase 5 3-axis golden G-code regression tests (`crates/chip-cli/tests/goldens/linuxcnc/v3_mesh_dome_3axis.ngc` & `grbl/`) while keeping all Phase 1 + Phase 2 goldens byte-for-byte identical.

---

## Key Architectural Decisions & Scope Guardrails

> [!IMPORTANT]
> **Exact `f64` Drop-Cutter Surface Fidelity & Zero New Dependencies:**
> - All three finishing strategies (`Parallel3D`, `Scallop3D`, `Pencil3D`) evaluate exact `f64` heights via Milestone 16's parallel `drop_cutter_points_parallel` (`rayon` + `MeshBvh`) and `SurfaceCutterProfile`—adding **zero** third-party crates (`cargo deny check` remains unchanged).
> - **3D Chord Decimation for Clean G-Code:** Dense `f64` Drop-Cutter scanlines (e.g., `0.15 mm` sampling) are simplified in 3D `(x, y, z)` using exact 3D Ramer–Douglas–Peucker (`tolerance_mm = 0.005 mm` default) so flat plateaus collapse to single `G1` moves while curved domes stay smooth within `5 µm`—keeping G-code files compact and smooth on real GRBL/LinuxCNC controllers!
> - **Backward Compatibility:** `schema_version` remains `3`, existing `schema_version = 1/2/3` manifests load losslessly, and Phase 1 + Phase 2 golden G-code snapshots remain byte-for-byte identical.

> [!NOTE]
> **Gouge-Proof 3D Linking:**
> - Every surface-hugging stepover link between adjacent passes in `Parallel3D`, `Scallop3D`, and `Pencil3D` is evaluated against `drop_cutter_z` along the link path (lifting smoothly over any intervening crest or retracting safely to `retract_z_wcs` when jumping across gaps or steep cliffs), guaranteeing **0 crashes and 0 target gouges** in `chip-sim`'s 5-rule verifier.

> [!TIP]
> **Analytical 3D Rest Detection (`compute_3d_rest_mask`):**
> - Comparing `drop_cutter_z(surface, reference_ball, x, y) - drop_cutter_z(surface, current_tool, x, y) > rest_threshold_mm` across a fast parallel `rayon` grid accurately identifies every concave fillet, valley, and corner where the larger previous tool could not reach—serving BOTH `Pencil3D` valley detection AND `Parallel3D`/`Scallop3D` 3D Rest Machining!

---

## Proposed Changes

### 1. Tool Compatibility, Operation Kinds, & 3D Finishing Parameters (`chip-tools`, `chip-cam`)

#### [MODIFY] [crates/chip-tools/src/model.rs](file:///workspace/ChipCutter/crates/chip-tools/src/model.rs)
- Add `ToolOperationClass::{Parallel3D, Scallop3D, Pencil3D}`.
- Update `CutterGeometry::supports_operation`:
  - `BallEndmill`, `BullNoseEndmill`, `FlatEndmill` → support `Parallel3D`, `Scallop3D`, `Pencil3D`.
  - `ChamferMill` (tapered / V-bit) → supports `Parallel3D` and `Pencil3D` (enabling fine relief carving with tapered ball/V-bits!).

#### [MODIFY] [crates/chip-cam/src/operation.rs](file:///workspace/ChipCutter/crates/chip-cam/src/operation.rs), [crates/chip-cam/src/params.rs](file:///workspace/ChipCutter/crates/chip-cam/src/params.rs), [crates/chip-cam/src/lib.rs](file:///workspace/ChipCutter/crates/chip-cam/src/lib.rs)
- Add `OperationKind::{Parallel3D, Scallop3D, Pencil3D}` (`"parallel_3d"`, `"scallop_3d"`, `"pencil_3d"`), updating `OperationKind::ALL`, `label()`, `default_name()`, `is_3d_surface()`, `is_compatible_with_tool()`, and `Operation::new` defaults (`ModelTop → ModelBottom`, `stock_to_leave = 0.0`).
- Add typed parameter structs in [params.rs](file:///workspace/ChipCutter/crates/chip-cam/src/params.rs):
  - `ParallelCutDirection` (`ZigZag`, `OneWayClimb`, `OneWayConventional`)
  - `Parallel3DParams`:
    - `boundary_mode: MachiningBoundaryMode`, `tool_containment: ToolContainmentMode`, `boundary_offset: ParamLength`
    - `stepover: ParamLength`, `use_cusp_height: bool`, `cusp_height: ParamLength`
    - `pass_angle_deg: f64` (`0.0` default), `cross_pass: bool` (`false` default — adds perpendicular `angle + 90°` pass)
    - `cut_direction: ParallelCutDirection`
    - `min_slope_deg: f64` (`0.0`), `max_slope_deg: f64` (`90.0`), `sample_step: ParamLength` (`0.25 mm`), `smoothing_tolerance: ParamLength` (`0.005 mm`)
  - `Scallop3DParams`:
    - `boundary_mode: MachiningBoundaryMode`, `tool_containment: ToolContainmentMode`, `boundary_offset: ParamLength`
    - `cusp_height: ParamLength` (`0.02 mm` default), `max_stepover: ParamLength` (`1.2 mm` default), `min_stepover: ParamLength` (`0.15 mm` default)
    - `climb_milling: bool`, `min_slope_deg: f64` (`0.0`), `max_slope_deg: f64` (`90.0`)
    - `inside_out: bool` (`false` = outside-in), `smoothing_tolerance: ParamLength` (`0.005 mm`)
  - `Pencil3DParams`:
    - `boundary_mode: MachiningBoundaryMode`, `tool_containment: ToolContainmentMode`, `boundary_offset: ParamLength`
    - `reference_tool_diameter: ParamLength` (virtual larger ball diameter detecting concave corners/valleys; defaults to `tool_d * 2.0` / `12.0 mm`)
    - `valley_threshold: ParamLength` (`0.08 mm` minimum residual stock thickness to trigger a pencil pass)
    - `num_offset_passes: u32` (`0` = single centerline valley trace; `1..4` adds lateral cleanup offsets on each side)
    - `offset_stepover: ParamLength` (`0.4 mm`), `smoothing_tolerance: ParamLength` (`0.005 mm`)
- Add `StrategyParams::{Parallel3D, Scallop3D, Pencil3D}` with `reevaluate(&EvalContext)` support.

---

### 2. Shared 3D Finishing & Rest-Mask Utilities (`crates/chip-cam/src/surface3d/finishing.rs`)

#### [NEW] `crates/chip-cam/src/surface3d/finishing.rs` & [MODIFY] [crates/chip-cam/src/surface3d/mod.rs](file:///workspace/ChipCutter/crates/chip-cam/src/surface3d/mod.rs)
- **`stepover_for_cusp_height(cutter: &SurfaceCutterProfile, cusp_mm: f64, fallback_stepover_mm: f64) -> f64`:**
  - Exact analytical chord-scallop inverse: `2.0 * (2.0 * r_corner * cusp - cusp²).sqrt()` for Ball and Bull-Nose cutters (clamped to `[0.05 mm, 0.75 * D]`), falling back gracefully to `fallback_stepover_mm` for Flat endmills.
- **`simplify_3d_polyline_rdp(pts: &[DVec3], tol_mm: f64) -> Vec<DVec3>`:**
  - Exact 3D point-to-segment distance Ramer–Douglas–Peucker decimation preserving sharp crease points while collapsing collinear flat/linear runs to minimal `G1` segments.
- **`build_3d_rest_mask(surface, current_cutter, containment, rest_cfg, upstream_rest, grid_res_mm) -> Option<RestMaskGrid2D>`:**
  - Supports `RestMachiningMode::FromPreviousTool { previous_tool_diameter, previous_corner_radius, .. }` by evaluating exact `drop_cutter_points_parallel` for the larger reference cutter vs. `current_cutter` over the containment grid and marking cells where `z_prev - z_cur > threshold` (dilated by `0.5 * tool_r` for smooth lead-in overlap).
  - Supports `RestMachiningMode::FromPrecedingOperations` using the largest preceding tool in `upstream_rest` (or swept coverage bounds) so finishing ops cleanly target remaining stock!
- **`emit_surface_safe_link(surface, cutter, from_wcs, to_wcs, retract_z, stay_down_limit, feed, builder)`:**
  - Checks if `from_wcs -> to_wcs` is short (`<= stay_down_limit`) and clear above `drop_cutter_z`; if clear (or gently rising over a shallow step), emits a smooth surface-hugging link (`z = max(lerp_z, drop_z + 0.05)`); otherwise retracts cleanly to `retract_z`, rapids to `(to.x, to.y, retract_z)`, and feeds down to `to_wcs` at `plunge_feed` / `ramp_feed`!

---

### 3. Three 3D Finishing Strategies (`crates/chip-cam/src/strategies/`)

#### [NEW] `crates/chip-cam/src/strategies/parallel3d.rs`
- Rotates containment bounding box by `-pass_angle_deg`, generates parallel scanlines spaced by effective stepover (`cusp_height` or `stepover`), and samples each line at `sample_step` in parallel (`drop_cutter_points_parallel`).
- Clips points to `containment`, optional `RestMaskGrid2D`, `[z_bottom, z_top]`, and `[min_slope_deg, max_slope_deg]` (splitting lines into clean contiguous 3D chains).
- Simplifies each chain via `simplify_3d_polyline_rdp`, orders passes in boustrophedon (`ZigZag`) or unidirectional (`OneWayClimb` / `OneWayConventional`) order, and connects adjacent lines via `emit_surface_safe_link`.
- When `cross_pass == true`, generates a second perpendicular set of passes at `pass_angle_deg + 90°` seamlessly in the same operation!

#### [NEW] `crates/chip-cam/src/strategies/scallop3d.rs`
- Computes target 3D cusp stepover `s_3d` from `cusp_height` (clamped to `[min_stepover, max_stepover]`).
- Evaluates a fast `rayon` 2D slope-weighted distance field (fast-marching / Eikonal propagation on a fine grid inside `containment`, where local metric speed is `1 / cos(θ(x, y))` so iso-contours advance by `s_3d * cos(θ)` in XY = **exact constant 3D arc-length stepover `s_3d` along the 3D surface**):
  - Starting from `d = 0` on `∂containment`, marching inward with local step weight `sec(θ(x, y)) = 1.0 / cos(θ).max(0.25)` produces a smooth scalar distance field `D_3d(x, y)` measuring geodesic distance along the 3D surface.
  - Extracting marching-squares iso-contours at `D_3d = 0.5*s_3d, 1.5*s_3d, 2.5*s_3d, ...` naturally handles concave merges, island splits around bosses (such as the dome), and self-intersections without `cavalier_contours` offset instabilities.
  - Every extracted contour point is projected directly onto exact `f64` `drop_cutter_z(x, y)` (filtered by slope & optional 3D rest mask) and simplified with `simplify_3d_polyline_rdp`.
- Supports `outside_in` / `inside_out` ordering, `climb_milling` orientation, and smooth surface-safe helical/arc transitions between consecutive rings.

#### [NEW] `crates/chip-cam/src/strategies/pencil3d.rs`
- Evaluates exact `f64` residual fillet height `delta_z(x, y) = drop_cutter_z(ref_ball) - drop_cutter_z(cur_cutter)` across a fine `rayon` grid over `containment` (where `ref_ball` has diameter `reference_tool_diameter`).
- Identifies all valley cells (`delta_z >= valley_threshold`), extracts crisp 1-cell-wide valley centerlines (Guo–Hall / Zhang–Suen morphological thinning or non-maximum ridge tracing along the local `delta_z` crest!), chains connected valley pixels into smooth 2D polylines, and optionally offsets each chain laterally by `±k * offset_stepover` for `num_offset_passes`.
- Projects every point onto exact `f64` `drop_cutter_z(surface, current_cutter, x, y)`, simplifies via `simplify_3d_polyline_rdp`, and emits collision-free ramped/retracted passes (`MotionRole::FinishCut`).

#### [MODIFY] [crates/chip-cam/src/strategies/mod.rs](file:///workspace/ChipCutter/crates/chip-cam/src/strategies/mod.rs)
- Dispatch `StrategyParams::Parallel3D`, `StrategyParams::Scallop3D`, and `StrategyParams::Pencil3D`.

---

### 4. Sample Projects, GUI Controls, & Phase 5 3-Axis CLI Goldens (`chip-project`, `chip-gui`, `chip-cli`)

#### [MODIFY] [crates/chip-project/src/demos.rs](file:///workspace/ChipCutter/crates/chip-project/src/demos.rs) & [crates/chip-project/src/lib.rs](file:///workspace/ChipCutter/crates/chip-project/src/lib.rs)
- Enhance `build_sculpted_mesh_dome_project` (`--sample mesh-dome`) with **Op 3: `3D Pencil Fillet Cleanup`** (Ø3mm Ball Endmill tracing the concave dome-to-pedestal crease left by the Ø6mm ball semi-finish!), demonstrating the complete 3-axis rough → semi-finish → corner cleanup pipeline.
- Add `build_relief_wave_mesh_project` (`--sample mesh-wave`) using `sample_relief_wave_obj_text()` (`80 × 60 × 15 mm` OBJ wave relief) seeded with **Op 1: `3D Adaptive Roughing`** (Ø6mm Flat) → **Op 2: `3D Parallel Raster Finish`** (Ø6mm Ball, `0.02 mm` cusp height, `45°` pass angle) → **Op 3: `3D Constant-Scallop Finish`** (optional/ready-to-simulate showcase!).

#### [MODIFY] [crates/chip-gui/src/panels/operation_inspector/passes_tab/surface_3d.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/passes_tab/surface_3d.rs), [passes_tab/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/passes_tab/mod.rs), [browser.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/browser.rs), [command_palette.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/command_palette.rs), [menus_toolbar.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/menus_toolbar.rs), [welcome_gallery.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/welcome_gallery.rs), [icons.rs](file:///workspace/ChipCutter/crates/chip-gui/src/widgets/icons.rs), [app/loader.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/loader.rs), [app/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/mod.rs)
- Add two-column Inspector cards in `surface_3d.rs` for **`3D Parallel Finishing`**, **`3D Constant-Scallop`**, and **`3D Pencil Cleanup`**, plus Rest Machining card support on `Parallel3D` and `Scallop3D`.
- Wire `3D Parallel Finishing`, `3D Constant-Scallop`, and `3D Pencil Cleanup` into the Setup `+ Op…` menu, Operations menu, and `Ctrl+K` Command Palette.
- Add **`Relief Wave (OBJ Mesh)`** (`SampleProjectId::ReliefWaveMesh`, `--sample mesh-wave`) to the Sample Projects Gallery (`SampleCategory::Mesh3D`).

#### [NEW] `crates/chip-cli/tests/goldens/{linuxcnc,grbl}/v3_mesh_dome_3axis.*` & [MODIFY] [crates/chip-cli/tests/cli_end_to_end.rs](file:///workspace/ChipCutter/crates/chip-cli/tests/cli_end_to_end.rs)
- Add `v3_golden_3axis_mesh_surfacing_snapshots_match_byte_for_byte` verifying deterministic headless `chipcutter build` on a 3-axis Sculpted Dome project (`Adaptive3D` + `Parallel3D` + `Pencil3D`) for LinuxCNC and GRBL (with `UPDATE_V3_GOLDENS=1` support matching `UPDATE_V2_GOLDENS`), preserving all Phase 1 and Phase 2 golden snapshots untouched!

---

## Verification Plan

1. **Automated 3D Finishing & Physics Oracle Tests (`chip-cam`, `chip-project`, `chip-cli`):**
   - `chip-cam::strategies::tests` & `tests/strategies_invariants.rs`:
     - **`Parallel3D` Oracle:** Verify exact `drop_cutter_z` surface fidelity, cusp-height stepover calculation, `pass_angle_deg = 45°`, `cross_pass = true` doubling pass orientation coverage, shallow slope limit (`0°..40°`) skipping vertical walls, and **0 crashes / 0 gouges** in `chip-sim` on `Sculpted Dome (STL)` and `Relief Wave (OBJ)`.
     - **`Scallop3D` Oracle:** Verify slope-adaptive XY spacing is demonstrably tighter on the steep equator (`r ≈ 22 mm`) than on the shallow dome crest (`r ≈ 5 mm`), uniform finish, and **0 crashes / 0 gouges** in `chip-sim`.
     - **`Pencil3D` Oracle:** Verify concave valley detection traces the exact `r = 25 + R_ball` circular fillet crease around the `Sculpted Dome` base pedestal (all valley points within `< 0.5 mm` of exact crease radius `25 + 1.5 = 26.5 mm` for Ø3mm ball!) with **0 crashes / 0 gouges** in `chip-sim`.
     - **3D Rest Boundary Oracle:** Verify `Parallel3D` with `RestMachiningMode::FromPreviousTool` (`Ø12 mm` previous ball → `Ø3 mm` finishing ball) significantly reduces cutting distance by skipping already-finished flat/convex regions!
   - `chip-project::tests`:
     - Deterministic Schema v3 `.chip.toml` round-trip, validation, simulation, and GRBL/LinuxCNC post-processing for both `Sculpted Dome (STL)` (3 ops) and `Relief Wave (OBJ)` (3 ops).
   - `chip-cli/tests/cli_end_to_end.rs`:
     - Phase 5 3-axis golden G-code snapshots + full Phase 1 & Phase 2 golden parity.
2. **Full Workspace Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings`
   - `cargo deny check` (`0` new third-party dependencies)
   - `WGPU_BACKEND=vulkan cargo test --workspace`
3. **Headless Vulkan Visual Verification (`xvfb-run` + Lavapipe):**
   - Capture and visually inspect `--sample mesh-wave` (`Relief Wave OBJ` with 3D Parallel / Scallop toolpaths & smooth machined wave stock) and `--sample mesh-dome` (3-op Adaptive + Waterline + Pencil fillet cleanup) in Studio Dark & Daylight Light themes via `read_file`.
