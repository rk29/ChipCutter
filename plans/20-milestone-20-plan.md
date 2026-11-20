# ChipCutter — Milestone 20 Implementation Plan

## Goal Description
Implement **Milestone 20 (2-Linear + Rotary `Z+X` / `Z+Y` & Indexed `3+1` Multi-Face Milling)** from [10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md#L128-L133) on top of Milestone 19's 4th-axis kinematics, rotary `ClEvent` IR, `G93`/`G94`/swapped-axis post-processors, and polar `CylindricalStock` simulator:

1. **Per-Operation `Indexed 3+1` & `WrapCylinder` Configuration (`chip-cam`, `chip-project`):**
   - Add `OperationRotaryMode` (`None`, `Indexed3Plus1 { angle_deg }`, `WrapCylinder { wrap_radius, axial_offset, angle_offset_deg }`) to `Operation` (omitted from `.chip.toml` when `None` for zero-diff backward compatibility).
   - **Universal `Indexed 3+1` Multi-Face Execution:** Automatically rotate the virtual Setup frame by `-angle_deg` around the active rotary axis, execute **any** existing 2.5D or 3-axis strategy (`Facing`, `Contour2D`, `Pocket2D`, `Adaptive2D`, `DrillBore`, `ChamferTrace`, `VCarve`, `Adaptive3D`, `Waterline3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`), and wrap the output with safe clearance-radius retracts and `ClEvent::RotaryIndex` positioning moves—plus a B-Rep face-normal helper computing the exact index angle aligning any picked face straight up to `+Z`.
   - **2-Linear + Rotary Cylindrical Wrapping (`WrapCylinder`):** Wrap any 2D Sketch/DXF/SVG `Contour2D`, `Pocket2D`, `ChamferTrace`, `VCarve`, or `DrillBore` toolpath onto a target cylinder radius `R_wrap` while strictly locking the cross-axis at the rotary centerline (`Z + X + A` or `Z + Y + B`), subdividing curved arcs along the circumference and emitting exact `RotaryLinearMove` / `RotaryRapidMove` spans (compatible with both `linuxcnc_4axis`/`grblhal_4axis` `G93` and 3-axis swapped-axis `grbl_rotary_wrap`!).
2. **Centerline-Locked 2-Linear + Rotary 3D Surfacing (`OperationKind::Rotary2Linear`) (`chip-tools`, `chip-cam`):**
   - Add `ToolOperationClass::Rotary2Linear`, `OperationKind::Rotary2Linear` (`"rotary_2linear"`, label `"4th-Axis Rotary Surfacing"`), and `Rotary2LinearParams`.
   - Implement exact `f64` rotary Drop-Cutter surfacing (`crates/chip-cam/src/strategies/rotary2linear.rs`) supporting **Continuous Helical Spiral** and **Axial Parallel** passes over arbitrary STEP solids and STL/OBJ/3MF meshes (with multi-depth radial roughing stepdowns, cusp-height stepover, axial & angular span limits, and strict cross-axis centerline locking for `Z+X+A` or `Z+Y+B`).
3. **Live 3D Wrapped/Indexed Toolpath Viewport Rendering & Rotary Operation Inspector (`chip-gui`, `chip-sim`):**
   - Transform rotary toolpath segments into workpiece-wrapped 3D curves (`rotate_point_wcs(pos_wcs, -rotary_deg)`) in `build_toolpath_gpu_instances` so helical spirals, wrapped 2D pockets/engravings, and 4-sided `3+1` indexed passes wrap crisply around the 3D shaft/cylinder (rotating dynamically in sync with machined stock during playback!).
   - Add Operation Inspector cards for **Rotary / 3+1 Positioning** (with `0° / 90° / 180° / 270°` quick index buttons & **"Align to Selected Face"**) and **`4th-Axis Rotary Surfacing`** parameters, plus Setup `+ Op…` / menu / `Ctrl+K` actions.
4. **Verified 4th-Axis Samples & Cylindrical Simulation Oracles (`chip-project`, `chip-cam`, `chip-sim`):**
   - Upgrade **`Rotary Indexed Shaft (4th-Axis A)`** (`--sample rotary-shaft`) with live crash-free `4th-Axis Rotary Spiral` (carving the turned collar groove & 3-lobe cam) + `Indexed 3+1` multi-face operations (`0°`, `90°`, `180°`, `270°` wrenching flats!).
   - Add **`Wrapped Rotary Engraving (Z+X+A)`** (`--sample rotary-wrap`) showcasing 2-linear + rotary cylindrical wrapping of 2D Sketch/Text engraving & pocket slots onto a round bar (verified in `CylindricalStock` with `0` crashes/gouges and post-processed cleanly to both `linuxcnc_4axis` and `grbl_rotary_wrap`).

---

## Key Architectural Decisions & Scope Guardrails

> [!IMPORTANT]
> **Strict 2-Linear + Rotary Guarantee & Zero Regression:**
> - Both `OperationKind::Rotary2Linear` and `OperationRotaryMode::WrapCylinder` strictly lock the cross-axis to `rotary.center_wcs` (`Y == center_y` when rotary axis is `X/A`; `X == center_x` when rotary axis is `Y/B`), guaranteeing pure `Z + X + Rotary` or `Z + Y + Rotary` motion on every cutting and linking segment!
> - `Operation::rotary` uses `#[serde(default, skip_serializing_if = "OperationRotaryMode::is_none")]`, keeping all existing 3-axis `.chip.toml` files and Phase 1 / Phase 2 / M18 v3 golden G-code snapshots **100% byte-for-byte identical**.
> - **Zero New External Dependencies:** Reuses `SurfaceMesh3D`, `drop_cutter_z`, `CylindricalStock`, `rayon`, and `glam`.

> [!NOTE]
> **Exact Rotary Drop-Cutter Over Arbitrary Meshes & STEP Solids:**
> - In `rotary2linear.rs`, to evaluate the exact minimum safe cutter radius `r_contact(u, θ)` for a cutter positioned directly above centerline at `(u, 0, z)` with the part rotated by `θ`:
>   - For `AxialParallel` (discrete angular stations `θ_k`): rotate `SurfaceMesh3D` vertices by `+θ_k` once per angular station (or in parallel across stations with `rayon`), then query exact `f64` `drop_cutter_z_with_scratch` along the centerline line `u ∈ [u_start, u_end]`!
>   - For `ContinuousSpiral`: evaluate exact rotary drop-cutter heights on a fine polar `(u_i, θ_j)` grid (one `rayon` parallel job per angular slice `θ_j` rotating vertices by `+θ_j` and running exact `drop_cutter_z_with_scratch` across all `u_i`!) and bilinearly sample + exact-verify along the continuous helical spiral, guaranteeing **sub-10 µm accuracy and 0 target gouges** across Flat, Ball, Bull-Nose, and V-Bit cutters in `< 80 ms`!

> [!TIP]
> **Safe Clearance Retracts for `Indexed 3+1`:**
> - Before any `Indexed 3+1` operation begins cutting at `angle_deg`, it lifts/starts at `z_safe = max(retract_z_wcs, center_z + rotary.clearance_radius_mm)`, emits `ClEvent::RotaryIndex { angle_deg, rapid: true, feed_deg_min: None }`, executes the rotated operation, and finishes back at `z_safe`—preventing the tool from ever clipping corners of square or eccentric stock during `90°`/`180°` index rotations!

---

## Proposed Changes

### 1. Per-Operation Rotary Mode, Face-Normal Indexing, & Cylindrical Wrapping (`chip-cam`, `chip-tools`)

#### [MODIFY] [crates/chip-tools/src/model.rs](file:///workspace/ChipCutter/crates/chip-tools/src/model.rs)
- Add `ToolOperationClass::Rotary2Linear` (supported by `BallEndmill`, `BullNoseEndmill`, `FlatEndmill`, and `ChamferMill`/V-Bit).

#### [MODIFY] [crates/chip-cam/src/rotary.rs](file:///workspace/ChipCutter/crates/chip-cam/src/rotary.rs)
- Add `OperationRotaryMode` (`Serialize`/`Deserialize`, `#[serde(tag = "mode", rename_all = "snake_case")]`):
  - `None` (default)
  - `Indexed3Plus1 { angle_deg: f64 }`
  - `WrapCylinder { wrap_radius: ParamLength, axial_offset: ParamLength, angle_offset_deg: f64 }`
- Add helper `compute_index_angle_for_normal(normal_wcs: DVec3, rotary: &ResolvedRotaryConfig) -> Option<f64>` returning the exact rotary angle `θ_deg` that rotates a picked B-Rep planar face normal directly to `+Z_wcs` (spindle up) around `X/A` or `Y/B`!
- Add helper `build_indexed_virtual_setup(setup: &ResolvedSetup, rotary: &ResolvedRotaryConfig, angle_deg: f64) -> ResolvedSetup` producing the exact rotated `RigidFrame3` + updated bounds for executing any 2.5D/3-axis operation at `angle_deg`.

#### [MODIFY] [crates/chip-cam/src/operation.rs](file:///workspace/ChipCutter/crates/chip-cam/src/operation.rs), [crates/chip-cam/src/params.rs](file:///workspace/ChipCutter/crates/chip-cam/src/params.rs), [crates/chip-cam/src/lib.rs](file:///workspace/ChipCutter/crates/chip-cam/src/lib.rs)
- Add `#[serde(default, skip_serializing_if = "OperationRotaryMode::is_none")] pub rotary: OperationRotaryMode` to `Operation` (and reevaluate its `ParamLength` fields in `Operation::reevaluate_all`).
- Add `OperationKind::Rotary2Linear` (`"rotary_2linear"`, label `"4th-Axis Rotary Surfacing"`, default name `"4th-Axis Rotary"`, `is_3d_surface() == true`).
- Add `RotaryPatternMode` (`ContinuousSpiral`, `AxialParallel`), `RotaryCutDirection` (`Climb`, `Conventional`, `ZigZag`), and `Rotary2LinearParams`:
  - `pattern: RotaryPatternMode` (`ContinuousSpiral` default)
  - `cut_direction: RotaryCutDirection`
  - `stepover: ParamLength` (`1.2 mm` default; axial pitch per rev in Spiral, circumferential surface stepover in AxialParallel)
  - `use_cusp_height: bool`, `cusp_height: ParamLength` (`0.03 mm` default)
  - `start_angle_deg: f64` (`0.0`), `end_angle_deg: f64` (`360.0`)
  - `axial_margin_start: ParamLength` (`0.0`), `axial_margin_end: ParamLength` (`0.0`)
  - `angular_sample_deg: f64` (`2.0°` default), `axial_sample_step: ParamLength` (`0.4 mm` default)
  - `smoothing_tolerance: ParamLength` (`0.008 mm` default)
- Wire `StrategyParams::Rotary2Linear(Rotary2LinearParams)` into `StrategyParams`.

---

### 2. Cylindrical Wrapping Engine & `Rotary2Linear` Strategy (`crates/chip-cam/src/strategies/`)

#### [NEW] `crates/chip-cam/src/strategies/rotary_wrap.rs`
- **`wrap_planar_toolpath_to_cylinder(flat_tp: &GeneratedToolpath, rotary: &ResolvedRotaryConfig, wrap_radius_mm: f64, axial_offset_mm: f64, angle_offset_deg: f64, safe_z_wcs: f64) -> GeneratedToolpath`:**
  - Maps flat `(x_2d, y_2d, z_2d)` toolpaths (`Contour2D`, `Pocket2D`, `ChamferTrace`, `VCarve`, `DrillBore`) onto cylinder radius `R = wrap_radius_mm`:
    - When `rotary.axis == XAxis` (`Z+X+A`): `u_axial = x_2d + axial_offset`, circumferential arc `s = y_2d`, cross-axis locked `y_wcs = rotary.center_wcs.y`.
    - When `rotary.axis == YAxis` (`Z+Y+B`): `u_axial = y_2d + axial_offset`, circumferential arc `s = -x_2d`, cross-axis locked `x_wcs = rotary.center_wcs.x`.
    - `rotary_deg = angle_offset_deg + (s / R).to_degrees()`, radial height `z_wcs = rotary.center_wcs.z + (R + z_2d)` (where `z_2d = 0` sits right on the cylinder OD and `z_2d < 0` cuts radially inward!).
  - Linearizes XY arcs (`ArcMove`) and long lines into dense chords (`<= 1.5°` / `<= 0.35 mm`) so wrapped contours hug the cylinder surface cleanly without chord flats, emitting `ClEvent::RotaryRapidMove` / `ClEvent::RotaryLinearMove` via `ToolpathBuilder`!
- **`wrap_indexed_3plus1_toolpath(local_tp: &GeneratedToolpath, rotary: &ResolvedRotaryConfig, angle_deg: f64) -> GeneratedToolpath`:**
  - Prepends a safe vertical retract to `z_safe = (rotary.center_wcs.z + rotary.clearance_radius_mm).max(...)`, emits `ClEvent::RotaryIndex { angle_deg, rapid: true, feed_deg_min: None }`, copies all operation events & segments tagged with `start_rotary_deg = end_rotary_deg = angle_deg`, and appends a final safe retract to `z_safe`!

#### [NEW] `crates/chip-cam/src/strategies/rotary2linear.rs`
- Implements **exact `f64` 2-Linear + Rotary 3D Surfacing** (`generate_rotary2linear`) for `XAxis` (`Z+X+A`) and `YAxis` (`Z+Y+B`):
  - Builds WCS `SurfaceMesh3D` from `ctx.doc` (STEP solid or STL/OBJ/3MF mesh).
  - Evaluates exact `f64` rotary drop-cutter contact radius `r_surf(u, θ)` across a parallel `rayon` polar grid (`n_theta × n_u`, rotating triangles around `rotary.axis` by `+θ` per slice and running `drop_cutter_z_with_scratch` right along the centerline!).
  - Supports multi-depth radial roughing (`ctx.passes.multiple_depths == true`) stepping inward from `stock_outer_radius_mm` to `r_surf(u, θ) + radial_stock` in increments of `max_stepdown`, OR single-pass finishing directly on `r_surf(u, θ) + radial_stock`.
  - Generates either:
    - **`ContinuousSpiral`:** Smooth continuous helical spiral from `u_start` to `u_end` (`total_turns = (u_end - u_start) / pitch`, including optional 360° cleanup rings at both ends!) with 3D chord decimation along `(u, θ, r)`.
    - **`AxialParallel`:** Clean axial raster passes along `u_start ↔ u_end` stepped around `[start_angle_deg..end_angle_deg]` with safe surface-hugging or clearance-radius links.
  - Emits synchronized `ClEvent::RotaryRapidMove` / `ClEvent::RotaryLinearMove` with exact `effective_radius_mm` for `G93` inverse-time feed!

#### [MODIFY] [crates/chip-cam/src/strategies/mod.rs](file:///workspace/ChipCutter/crates/chip-cam/src/strategies/mod.rs)
- Dispatch `StrategyParams::Rotary2Linear` to `rotary2linear::generate_rotary2linear`.
- Inside `generate_operation_toolpath_with_rest`:
  - When `setup.rotary` is `Some(rot)` and `op.rotary` is `OperationRotaryMode::Indexed3Plus1 { angle_deg }`: build the rotated virtual setup, execute the inner strategy, and wrap with `wrap_indexed_3plus1_toolpath`.
  - When `setup.rotary` is `Some(rot)` and `op.rotary` is `OperationRotaryMode::WrapCylinder { .. }`: execute the flat 2D strategy on an unwrapped virtual stock top (`Z = 0`) and wrap with `wrap_planar_toolpath_to_cylinder`.

---

### 3. Live 3D Wrapped/Indexed Toolpath Visualization & Rotary Operation UI (`chip-gui`, `chip-project`)

#### [MODIFY] [crates/chip-gui/src/viewport/scene_builders.rs](file:///workspace/ChipCutter/crates/chip-gui/src/viewport/scene_builders.rs), [crates/chip-gui/src/app/viewport_controller.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/viewport_controller.rs)
- Add `pub sim_rotary_deg: f64` to `HighlightState` (set to live `ScrubSnapshot::rotary_deg` when simulation playback is active, `0.0` otherwise).
- In `build_toolpath_gpu_instances`: when `highlight.rotary` is `Some(rot)`:
  - Subdivide rotary segments (`has_rotary_motion()` or `start_rotary_deg != 0`) and transform each sample point into workpiece-wrapped coordinates `rot.rotate_point_wcs(pt_wcs, -seg_rotary_deg + highlight.sim_rotary_deg)` so helical spirals, wrapped 2D curves, and `3+1` indexed passes appear wrapped accurately around the 3D shaft/cylinder AND rotate live with the stock during playback!

#### [MODIFY] [crates/chip-gui/src/panels/operation_inspector/passes_tab/surface_3d.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/passes_tab/surface_3d.rs), [passes_tab/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/passes_tab/mod.rs), [operation_inspector/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/mod.rs), [browser.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/browser.rs), [command_palette.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/command_palette.rs), [menus_toolbar.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/menus_toolbar.rs), [icons.rs](file:///workspace/ChipCutter/crates/chip-gui/src/widgets/icons.rs)
- Add the **`4th-Axis Rotary Surfacing`** Inspector card (`Rotary2LinearParams`) and a **`4th-Axis Indexing / Wrapping`** Inspector card (shown automatically on any operation belonging to a rotary Setup) with:
  - Mode selector (`None (0°)`, `Indexed 3+1 Angle`, `Wrap 2D on Cylinder`),
  - Quick angle buttons (`0°`, `90°`, `180°`, `270°`) + **"Align to Selected 3D Face"** action when a planar B-Rep face is selected,
  - Target Wrap Radius, Axial Offset, and Start Angle Offset fields for `WrapCylinder`.
- Wire `4th-Axis Rotary Surfacing` into Setup `+ Op…`, Operations menu, and `Ctrl+K` Command Palette (`CommandId::AddOpRotary2Linear`).

#### [MODIFY] [crates/chip-project/src/demos.rs](file:///workspace/ChipCutter/crates/chip-project/src/demos.rs), [crates/chip-project/src/lib.rs](file:///workspace/ChipCutter/crates/chip-project/src/lib.rs), [crates/chip-project/src/validation.rs](file:///workspace/ChipCutter/crates/chip-project/src/validation.rs), [crates/chip-gui/src/panels/welcome_gallery.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/welcome_gallery.rs), [crates/chip-gui/src/app/loader.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/loader.rs), [crates/chip-gui/src/app/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/mod.rs), [crates/chip-gui/src/main.rs](file:///workspace/ChipCutter/crates/chip-gui/src/main.rs)
- Upgrade `build_rotary_shaft_preview_project` (`--sample rotary-shaft`) into a complete, crash-free multi-op 4th-axis showcase:
  - **Op 1:** `4th-Axis Rotary Spiral Surfacing` (`OperationKind::Rotary2Linear`, Ø6mm Ball Endmill carving the turned central collar & 3-lobe cam in continuous `Z+X+A` helical spiral!),
  - **Op 2–5:** `Indexed 3+1` flat facing passes at `A = 0°, 90°, 180°, 270°` milling the 4 wrenching flats in one chucking!
- Add **`Wrapped Rotary Engraving (Z+X+A)`** (`build_rotary_wrapped_engraving_project`, `--sample rotary-wrap`) wrapping a 2D Sketch decorative band + `"CHIP"` text engraving & slot onto a `Ø40 × 80 mm` aluminum cylinder (verified in `CylindricalStock` with `0` crashes/gouges and clean `linuxcnc_4axis` + `grbl_rotary_wrap` G-code!).
- Validate in [validation.rs](file:///workspace/ChipCutter/crates/chip-project/src/validation.rs) that `OperationKind::Rotary2Linear` and non-`None` `OperationRotaryMode` require an enabled `Setup::rotary` configuration.

---

## Verification Plan

1. **Automated 4th-Axis & Cylindrical Physics Oracle Tests (`chip-cam`, `chip-sim`, `chip-project`, `chip-post`):**
   - `chip-cam::tests/strategies_invariants.rs`:
     - **`Rotary2Linear` Centerline & 0-Gouge Oracle (`Z+X+A` & `Z+Y+B`):** Verify BOTH `ContinuousSpiral` and `AxialParallel` on `sample_rotary_shaft_stl_bytes()` strictly lock the cross-axis (`|y| < 1e-9` for `X/A`, `|x| < 1e-9` for `Y/B`), remove substantial stock volume, and pass `chip-sim`'s 5-rule `CylindricalStock` verifier with **0 crashes and 0 target gouges**!
     - **`WrapCylinder` 2-Linear + Rotary Oracle:** Verify wrapping a 2D `Pocket2D`, `Contour2D`, and `VCarve` text sketch onto a `Ø40 mm` cylinder strictly locks the cross-axis, maps circumferential arc lengths to exact angles (`s = R * θ`), passes `CylindricalStock` verification with **0 crashes**, and posts cleanly to BOTH `linuxcnc_4axis` (`G93`) and `grbl_rotary_wrap` (Y-swapped, zero `A` words)!
     - **`Indexed 3+1` Multi-Face Oracle:** Verify `0° / 90° / 180° / 270°` indexed operations on `sample_rotary_shaft_stl_bytes()` emit safe clearance-radius retracts before every `RotaryIndex`, accurately mill all 4 flats at `r = 15 mm`, and pass `CylindricalStock` verification with **0 crashes and 0 target gouges**!
     - **Face-Normal Alignment Oracle:** Verify `compute_index_angle_for_normal` computes exact `0°`, `90°`, `180°`, `270°`, and `45°` index angles for `X/A` and `Y/B` setups.
   - `chip-project::tests`:
     - Deterministic Schema v3 `.chip.toml` round-trip, validation, polar simulation (`0` critical errors), and 4-axis G-code post-processing for BOTH `Rotary Indexed Shaft` and `Wrapped Rotary Engraving`.
2. **Full Workspace Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings`
   - `cargo deny check` (`0` new dependencies)
   - `WGPU_BACKEND=vulkan cargo test --workspace` (preserving Phase 1, Phase 2, and M18 v3 CLI goldens byte-for-byte!)
3. **Headless Vulkan Visual Verification (`xvfb-run` + Lavapipe):**
   - Capture and visually inspect `--sample rotary-shaft` (showing 3D wrapped helical spiral + 4-sided `3+1` indexed toolpaths & simulated cylindrical machined shaft rotating live in Dark & Light themes) and `--sample rotary-wrap` (wrapped 2D engraving/slots hugging the 3D cylinder) via `read_file`.
