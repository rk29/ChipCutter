# Milestone 21 Implementation Plan — Simultaneous 4-Axis Milling (`X + Y + Z + Rotary`), Phase 5 Tutorials, & Release Hardening

## Goal

Complete **Phase 5 (3-Axis Surface Milling, Mesh Support, & 4th-Axis Rotary Milling)** by delivering:
1. **Simultaneous 4-Axis Surface & Off-Centerline Milling (`OperationKind::Rotary4AxisSim`)** driving all three linear axes (`X`, `Y`, `Z`) together with the rotary axis (`A`/`B`) for off-centerline tilted surface finishing, steep flanks, and multi-start helical flutes/cams with gouge-free tool-axis tilt smoothing.
2. **Full Simultaneous 4-Axis Verification, Exact `G93` Workpiece-Surface Feed Control, & 4-Axis Cycle-Time Estimation** across `chip-cam`, `chip-sim`, and `chip-post`.
3. **Phase 5 Golden G-Code Regression Suites (`chip-cli`), Illustrated Phase 5 Tutorials (`docs/tutorials/08..10`), & Release Hardening** — preserving all Phase 1, Phase 2, and M18 v3 CLI golden snapshots byte-for-byte, zero new dependencies (`cargo deny check` clean), `#![deny(unsafe_code)]`, and UI-agnostic core crates.

---

## Proposed Changes

### 1. Simultaneous 4-Axis Data Model, Helical Flute STL Fixture, & Tool Compatibility (`chip-cad`, `chip-tools`, `chip-cam`)

#### [MODIFY] [crates/chip-cad/src/mesh/fixtures.rs](file:///workspace/ChipCutter/crates/chip-cad/src/mesh/fixtures.rs), [crates/chip-cad/src/mesh/mod.rs](file:///workspace/ChipCutter/crates/chip-cad/src/mesh/mod.rs), [crates/chip-cad/src/lib.rs](file:///workspace/ChipCutter/crates/chip-cad/src/lib.rs)
- Add a watertight programmatic binary STL fixture **`sample_helical_flute_shaft_stl_bytes()`**:
  - `Ø40 mm × 80 mm` shaft aligned along WCS `X` (`X ∈ [-40, +40]`, centerline `Y=0, Z=0`), with cylindrical end journals (`R = 20 mm`) and a sculpted **3-start helical flute / twisted cam section** over `X ∈ [-24, +24]` (`120°` helical twist with smooth asymmetric flank walls and concave root valleys down to `R = 13.5 mm`) designed specifically to showcase simultaneous `X+Y+Z+A` off-centerline milling.
  - Add a unit test verifying watertightness, positive volume, and exact bounds.

#### [MODIFY] [crates/chip-tools/src/model.rs](file:///workspace/ChipCutter/crates/chip-tools/src/model.rs)
- Add `ToolOperationClass::Rotary4AxisSim` and enable it for `FlatEndmill`, `BullNoseEndmill`, `BallEndmill`, and `ChamferMill`.

#### [MODIFY] [crates/chip-cam/src/operation.rs](file:///workspace/ChipCutter/crates/chip-cam/src/operation.rs), [crates/chip-cam/src/params.rs](file:///workspace/ChipCutter/crates/chip-cam/src/params.rs), [crates/chip-cam/src/lib.rs](file:///workspace/ChipCutter/crates/chip-cam/src/lib.rs)
- Add `OperationKind::Rotary4AxisSim` (`"Simultaneous 4-Axis (X+Y+Z+A)"`, `is_3d_surface() == true`, `is_rotary_native() == true`).
- Add `Rotary4AxisPattern` (`SimultaneousSpiral`, `HelicalFlowline`), `RotaryTiltMode` (`FixedLateralOffset`, `SurfaceNormalAutoTilt`), and `Rotary4AxisSimParams`:
  - `pattern: Rotary4AxisPattern` (`SimultaneousSpiral` default)
  - `tilt_mode: RotaryTiltMode` (`SurfaceNormalAutoTilt` default)
  - `lateral_offset: ParamLength` (explicit cross-axis offset for `FixedLateralOffset`, default `2.5 mm`)
  - `max_tilt_deg: f64` (maximum effective off-centerline tilt angle in `SurfaceNormalAutoTilt`, default `18.0°`, clamped to `[0.0, 40.0]`)
  - `tilt_smoothing_window: u32` (Gaussian/box smoothing radius in grid cells so `Y` and `A` transition smoothly without chatter, default `3`)
  - `stepover: ParamLength`, `use_cusp_height: bool`, `cusp_height: ParamLength`
  - `start_angle_deg: f64`, `end_angle_deg: f64` (`0.0..360.0`)
  - `axial_margin_start: ParamLength`, `axial_margin_end: ParamLength`
  - `angular_sample_deg: f64`, `axial_sample_step: ParamLength`, `smoothing_tolerance: ParamLength`
- Add `StrategyParams::Rotary4AxisSim(Rotary4AxisSimParams)` and re-export all new types from `chip-cam`.

---

### 2. Simultaneous 4-Axis CAM Strategy & Exact `G93` / Cycle-Time Kinematics (`chip-cam`, `chip-sim`, `chip-post`)

#### [NEW] `crates/chip-cam/src/strategies/rotary4axis_sim.rs`
- Implements **Simultaneous 4-Axis Milling (`X + Y + Z + Rotary`)** (`generate_rotary4axis_sim`) for `XAxis` (`X+Y+Z+A`) and `YAxis` (`X+Y+Z+B`):
  - Requires `setup.rotary` with `mode == RotaryDriveMode::Simultaneous4Axis` (and horizontal `XAxis` or `YAxis`; returns a clear `CamError::InvalidRotarySetup` if rotary is disabled or restricted to `TwoLinearPlusRotary` / `Indexed3Plus1`).
  - **Stage A — Off-Centerline Tilt Field `c_cross(u, θ)`:**
    - For `FixedLateralOffset`: uses constant `c_cross = lateral_offset` clamped safely within `0.75 * r_outer`.
    - For `SurfaceNormalAutoTilt`: computes a coarse centerline surface profile `r_center(u, θ)`, estimates the local circumferential slope `dr / (r · dθ)` (signed flank tilt of helical flutes & cam lobes!), converts into a bounded optimal lateral cross-axis shift `c_cross(u, θ) = clamp(r · sin(α_target), -c_max, +c_max)` (which shifts the cutter off-centerline toward the flank normal so the ball/bull-nose side cuts cleanly without dead-center tip rubbing!), and applies periodic multi-pass smoothing (`tilt_smoothing_window`) so `Y(t)` and `A(t)` remain `C¹`-smooth with zero sudden cross-axis spikes!
  - **Stage B — Exact `f64` Off-Centerline Drop-Cutter Grid `z_contact(u, θ)`:**
    - Evaluates exact `drop_cutter_z_with_scratch` in parallel (`rayon`) at each rotated slice `+θ` at the **exact off-centerline position** `(qx = u, qy = center.y + c_cross(u, θ))` (for `X/A`) or `(qx = center.x + c_cross(u, θ), qy = u)` (for `Y/B`), pre-filtering triangles to the exact off-centerline cutter strip (`|cross - center| <= max_|c_cross| + r_max`).
    - Guarantees **mathematically gouge-free** simultaneous `(X, Y, Z, A)` contact across arbitrary STEP/STL/OBJ/3MF meshes!
  - **Stage C — Multi-Depth Radial Roughing & Pattern Emission:**
    - Supports `ctx.passes.multiple_depths` with continuous non-unwinding inter-layer rotary progression (zero mid-air rapid unwinding!) and shoulder-safe chord decimation (never dipping > 3 µm below convex ridges).
    - Emits `SimultaneousSpiral` (continuous 4-axis helix with 360° end cleanup rings) or `HelicalFlowline` (smooth axial/helical flowline strips stepping circumferentially around `[start_angle_deg..end_angle_deg]`).
    - Tags every `ClEvent::RotaryLinearMove` with the exact off-centerline effective radius `r_eff = hypot(c_cross, z_wcs - center.z)` so `G93` inverse-time feedrate accounts for both cross-axis `Y` motion and rotary arc velocity!

#### [MODIFY] [crates/chip-cam/src/strategies/mod.rs](file:///workspace/ChipCutter/crates/chip-cam/src/strategies/mod.rs)
- Register `pub mod rotary4axis_sim;` and dispatch `StrategyParams::Rotary4AxisSim(p)`.

#### [MODIFY] [crates/chip-cam/src/rotary.rs](file:///workspace/ChipCutter/crates/chip-cam/src/rotary.rs), [crates/chip-sim/src/timeline.rs](file:///workspace/ChipCutter/crates/chip-sim/src/timeline.rs), [crates/chip-sim/src/cylindrical.rs](file:///workspace/ChipCutter/crates/chip-sim/src/cylindrical.rs)
- **Exact Simultaneous 4-Axis `G93` Workpiece-Relative Surface Velocity (`rotary.rs`):**
  - Enhance `compute_inverse_time_feed_g93` (and add a dedicated helper `compute_workpiece_arc_distance_mm`) to account accurately for combined `(X, Y, Z, A/B)` moves where cross-axis `Y` and angular `A` move simultaneously, ensuring true constant workpiece surface feedrate and strict `max_rotary_feed_deg_min` clamping.
- **Accurate 4-Axis Cycle-Time Estimation (`timeline.rs`):**
  - When a `ToolpathSegment3D` carries rotary motion (`d_deg > 1e-6`), compute effective workpiece surface travel `s_workpiece = hypot(lin_len, r_eff * d_rad)` (where `r_eff` is derived from midpoint radius `start_wcs`/`end_wcs` relative to `rotary` or radial Z) so `dt_surface = s_workpiece / feed_mm_s` is included alongside `dt_linear` and `dt_rotary`! (Fixes under-estimated cycle times on pure-rotary helical/wrapped moves where linear `X/Z` travel is small compared to circumferential arc travel.)
- **Off-Centerline Collision Audit & Fault-Injection Coverage (`cylindrical.rs`):**
  - Audit `CylindricalStock::sweep_pose_at` for off-centerline simultaneous 4-axis poses (`|c_cross| > 0`): ensure `r_tip` and the polar angular search wedge `[it_center ± it_half_span]` expand accurately for off-centerline lateral offsets (where the cutter flank grazes tangentially near the stock OD) so off-centerline shank/holder collisions and target-model gouges are caught with zero blind spots.
  - Add an explicit off-centerline fault-injection unit test in `chip-sim::cylindrical::tests`.

---

### 3. Simultaneous 4-Axis Inspector UI, New Sample Project, & Phase 5 CLI Goldens (`chip-gui`, `chip-project`, `chip-cli`)

#### [MODIFY] [crates/chip-project/src/demos.rs](file:///workspace/ChipCutter/crates/chip-project/src/demos.rs), [crates/chip-project/src/validation.rs](file:///workspace/ChipCutter/crates/chip-project/src/validation.rs), [crates/chip-project/src/lib.rs](file:///workspace/ChipCutter/crates/chip-project/src/lib.rs)
- Add **`build_rotary_helical_flute_project`** (`--sample rotary-4axis`): `Ø40 × 80 mm` helical 3-flute shaft chucked on `X/A` (`Simultaneous4Axis`, `linuxcnc_4axis`) with **Op 1: `Simultaneous 4-Axis Helical Roughing`** (Ø6mm Ball, multi-depth `3.0 mm` stepdowns, `SurfaceNormalAutoTilt` `16°`) + **Op 2: `Simultaneous 4-Axis Helical Finishing`** (Ø6mm Ball, fine `1.2 mm` spiral stepover, `SurfaceNormalAutoTilt` `18°`), passing `CylindricalStock` verification with **0 critical errors**.
- Validate in [validation.rs](file:///workspace/ChipCutter/crates/chip-project/src/validation.rs) that `OperationKind::Rotary4AxisSim` requires `setup.rotary.mode == RotaryDriveMode::Simultaneous4Axis`.

#### [MODIFY] [crates/chip-gui/src/panels/operation_inspector/passes_tab/surface_3d.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/passes_tab/surface_3d.rs), [passes_tab/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/passes_tab/mod.rs), [browser.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/browser.rs), [command_palette.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/command_palette.rs), [welcome_gallery.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/welcome_gallery.rs), [menus_toolbar.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/menus_toolbar.rs), [app/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/mod.rs), [app/loader.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/loader.rs), [icons.rs](file:///workspace/ChipCutter/crates/chip-gui/src/widgets/icons.rs), [main.rs](file:///workspace/ChipCutter/crates/chip-gui/src/main.rs)
- Add the **`Simultaneous 4-Axis (X+Y+Z+A/B)`** Inspector card (`show_rotary4axis_sim_params`) with pattern selector, tilt mode (`Auto Surface-Normal Tilt` / `Fixed Lateral Offset`), `Max Tilt (°)` / `Lateral Offset`, tilt smoothing window, cusp/stepover, angular window, axial margins, and sampling/smoothing controls.
- Wire `OperationKind::Rotary4AxisSim` into Setup `+ Op…`, Operations menu, `Ctrl+K` Command Palette (`CommandId::AddOpRotary4AxisSim`), and `SampleProjectId::RotaryHelicalFlute` (`--sample rotary-4axis`, auto-generate + auto-simulate).

#### [MODIFY] [crates/chip-cli/tests/cli_end_to_end.rs](file:///workspace/ChipCutter/crates/chip-cli/tests/cli_end_to_end.rs) + [NEW] Phase 5 Rotary Golden Files in `crates/chip-cli/tests/golden/`
- Add deterministic Phase 5 CLI end-to-end golden G-code regression tests (`chipcutter validate` → `simulate` → `build`):
  - `rotary_indexed_shaft_linuxcnc_4axis.ngc` (`G93` inverse-time continuous spiral + `A90/A180/A270` indexed 3+1 flats),
  - `rotary_wrapped_grbl_wrap.nc` (`grbl_rotary_wrap` swapped `Y`-axis wrapping, strictly zero `A` motion words),
  - `rotary_4axis_sim_grblhal_4axis.nc` (`grblhal_4axis` simultaneous `G93 G1 X... Y... Z... A... F...` off-centerline helical flute milling).
- Preserve all existing Phase 1, Phase 2, and M18 golden snapshots byte-for-byte without modification.

---

### 4. Phase 5 Illustrated Tutorials & Documentation Hardening (`docs/tutorials/`, `docs/images/tutorials/`, `README.md`, `specs/10-roadmap.md`)

#### [NEW] Minimal Phase 5 Starter Mesh Assets in `docs/tutorials/assets/`
- Export deterministic compact mesh fixtures for hands-on tutorial practice:
  - `docs/tutorials/assets/08-sculpted-dome.stl`
  - `docs/tutorials/assets/09-rotary-shaft.stl`
  - `docs/tutorials/assets/10-helical-flute-shaft.stl`

#### [NEW] Illustrated Phase 5 Tutorials (`docs/tutorials/`) + Headless Vulkan Screenshots (`docs/images/tutorials/`)
- **`docs/tutorials/08-3d-mesh-surfacing.md`:** Importing `STL`/`OBJ`/`3MF` meshes, 3D Adaptive Roughing, 3D Parallel & Constant-Scallop Finishing, and 3D Pencil Cleanup (with Dark/Light Vulkan screenshots).
- **`docs/tutorials/09-4th-axis-rotary-wrapping-indexing.md`:** Setup 4th-axis kinematics (`X/A`, `Y/B`, chuck & clearance cylinder), Wrapped 2D Engraving (`Z+X+A`, `linuxcnc_4axis` vs `grbl_rotary_wrap`), Continuous Rotary Spiral Surfacing, and `Indexed 3+1` Multi-Face Flats (with Vulkan screenshots).
- **`docs/tutorials/10-simultaneous-4axis-milling.md`:** Simultaneous `X+Y+Z+A` off-centerline milling with auto surface-normal tilt smoothing, `G93` inverse-time feedrates, cylindrical collision/gouge verification, and `linuxcnc_4axis` / `grblhal_4axis` output (with Vulkan screenshots).

#### [MODIFY] [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md), [README.md](file:///workspace/ChipCutter/README.md), [specs/10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md)
- Add tutorials `08`–`10` and their starter mesh assets to the Tutorial Curriculum table in `docs/tutorials/README.md`.
- Update `README.md` feature overview / tutorial links to reflect complete Phase 5 3-axis mesh & 4th-axis rotary capabilities.
- Mark **Phase 5 (Milestones 16–21)** completed in `specs/10-roadmap.md` once all verification gates pass.

---

## Verification Plan

1. **Automated Simultaneous 4-Axis & Cylindrical Physics Oracles:**
   - `chip-cad`: watertightness & bounds test for `sample_helical_flute_shaft_stl_bytes()`.
   - `chip-cam::tests/strategies_invariants.rs`:
     - **`Rotary4AxisSim` Simultaneous 4-Axis Oracle (`X+Y+Z+A` & `X+Y+Z+B`):** Verify `SimultaneousSpiral` & `HelicalFlowline` genuinely engage all 4 axes simultaneously (`max |y - center.y| > 1.0 mm` for `X/A`, `max |x - center.x| > 1.0 mm` for `Y/B`), keep cross-axis tilt smooth (`max |Δy| <= smooth_bound`), remove substantial flute material (`> 2,500 mm³`), and pass `CylindricalStock` 5-rule verification with **0 crashes and 0 target gouges**.
     - Verify `Rotary4AxisSim` cleanly rejects `TwoLinearPlusRotary` / disabled rotary setups with `CamError::InvalidRotarySetup`.
   - `chip-sim`: off-centerline holder/shank collision & target-gouge fault-injection tests + 4-axis circumferential cycle-time accuracy test.
   - `chip-project`: deterministic Schema v3 TOML round-trip, validation, simulation (`0` critical errors), and `linuxcnc_4axis` / `grblhal_4axis` post-processing for `Simultaneous 4-Axis Helical Flute`.
   - `chip-cli`: Phase 5 rotary golden G-code regression tests (`linuxcnc_4axis`, `grbl_rotary_wrap`, `grblhal_4axis`) with **100% byte-for-byte preservation** of all Phase 1, Phase 2, and M18 goldens.
2. **Full Workspace Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings`
   - `cargo deny check` (`0` new dependencies)
   - `WGPU_BACKEND=vulkan cargo test --workspace`
3. **Headless Vulkan Visual Verification & Docs Link Audit:**
   - Capture and visually inspect `--sample rotary-4axis` (Dark & Light) plus all new tutorial screenshots (`docs/images/tutorials/08..10/`) via `read_file`.
   - Verify every Markdown link and image path across `README.md` and `docs/tutorials/*.md` resolves on disk with zero broken links.
