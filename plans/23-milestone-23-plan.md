# Milestone 23 Implementation Plan — Deep-Dive 4th-Axis Rotary Tutorials: Wrapping, Indexing (`3+1`), & Simultaneous 4-Axis

## Goal

Expand the Phase 5 rotary overviews (`09-4th-axis-rotary-wrapping-indexing.md` and `10-simultaneous-4axis-milling.md`) into **three dedicated, step-by-step illustrated 4th-axis tutorials** covering every rotary drive mode, pattern, axis orientation (`X/A` & `Y/B`), and B-Rep face-normal alignment workflow:

1. **`docs/tutorials/10-rotary-wrapping-surfacing.md`** — `Z+X+A` & `Z+Y+B` Setup Kinematics, **`WrapCylinder`** (2D V-Carve, Pocket, & Contour wrapping on `linuxcnc_4axis` & 3-axis swapped-axis `grbl_rotary_wrap`), and **`Rotary2Linear`** (`ContinuousSpiral`, **`AxialParallel`**, and partial angular sectors `Start°..End°`).
2. **`docs/tutorials/11-indexed-3plus1-milling.md`** — Multi-face **`Indexed 3+1`** machining covering both cardinal `0°/90°/180°/270°` wrench flats on a mesh shaft AND a brand-new **prismatic angled STEP solid** walkthrough demonstrating **Align to Selected 3D Face** for drilling & pocketing angled faces in a single chucking.
3. **`docs/tutorials/12-simultaneous-4axis-milling.md`** — Full simultaneous **`X+Y+Z+A/B`** (`Rotary4AxisSim`) comparing **`Auto Surface-Normal Tilt`** (`Max Auto Tilt`, `Tilt Smoothing`) vs. **`Fixed Lateral Offset`** and **`SimultaneousSpiral`** vs. **`HelicalFlowline`**, with off-centerline `CylindricalStock` verification and `G93` inverse-time G-code inspection.

---

## Proposed Changes

### 1. New Prismatic Multi-Face STEP Fixture for `Indexed 3+1` Face-Normal Alignment (`chip-cad`, `chip-project`, `chip-cli`)

#### [MODIFY] [crates/chip-cad/src/fixtures.rs](file:///workspace/ChipCutter/crates/chip-cad/src/fixtures.rs), [crates/chip-cad/src/lib.rs](file:///workspace/ChipCutter/crates/chip-cad/src/lib.rs)
- Add a clean, minimal programmatic STEP solid fixture **`tutorial_indexed_4side_block_step()`**:
  - A `70 × 32 × 32 mm` rectangular/octagonal bar stock block aligned along WCS `X` (`X ∈ [-35, +35]`, `Y ∈ [-16, +16]`, `Z ∈ [-16, +16]`) with **4 cardinal faces** (`+Z` at `0°`, `+Y` at `90°`, `-Z` at `180°`, `-Y` at `270°`) AND a **45° chamfered/angled mounting facet** (or stepped pocket + transverse cross-drilled holes) so machinists can click any planar STEP face in 3D and click **`Align to Selected 3D Face`** to compute exact `0°`, `45°`, `90°`, `180°`, and `270°` index angles!
  - Add a unit test verifying STEP round-trip and exact planar face normals + `compute_index_angle_for_normal` alignment.

#### [MODIFY] [crates/chip-project/src/demos.rs](file:///workspace/ChipCutter/crates/chip-project/src/demos.rs), [crates/chip-project/src/lib.rs](file:///workspace/ChipCutter/crates/chip-project/src/lib.rs)
- Add dedicated tutorial project builders for Tutorials `10`, `11`, and `12`:
  - `build_tutorial_rotary_axial_parallel_project(model_path, sha256, tools)`: showcases `Rotary2Linear` with `RotaryPatternMode::AxialParallel` + partial angular window (`-60°..+60°`) alongside `ContinuousSpiral` (and documents `Y-Axis (B)` setup configuration).
  - `build_tutorial_indexed_step_multiface_project(doc, model_path, sha256, tools)`: loads `11-indexed-multiface-block.step` in `FixedBox` chucked stock (`36 × 36 × 70 mm` on `X/A`), runs `Indexed3Plus1` `Pocket2D` / `Facing` / `DrillBore` operations aligned to multiple planar faces (including an angled face via `compute_index_angle_for_normal`), and verifies **0 crashes / 0 gouges** in `CylindricalStock`.
  - `build_tutorial_rotary_4axis_flowline_project(model_path, sha256, tools)`: showcases `Rotary4AxisSim` comparing `Auto Surface-Normal Tilt` (`SimultaneousSpiral`) and `Fixed Lateral Offset` (`HelicalFlowline`) on `10-helical-flute-shaft.stl`.
- Add `chip-project` unit tests confirming all three projects serialize deterministically, validate with 0 errors, verify crash-free in `CylindricalStock`, and post-process cleanly (`linuxcnc_4axis`, `grblhal_4axis`, `grbl_rotary_wrap`).

#### [MODIFY] [crates/chip-cli/src/commands/inspect_sample.rs](file:///workspace/ChipCutter/crates/chip-cli/src/commands/inspect_sample.rs)
- Extend `chipcutter generate-sample-step --tutorial` with `10-rotary-wrap`, `10-rotary-axial`, `11-indexed-step`, `11-indexed-shaft`, and `12-simultaneous-4axis` (supporting `--empty-project` and `--project-out`).
- Generate and check in `docs/tutorials/assets/11-indexed-multiface-block.step` (and rename/alias starter STL assets cleanly for `10`–`12` in `docs/tutorials/assets/`).

---

### 2. Three Deep-Dive Illustrated Rotary Tutorials & Screenshots (`docs/tutorials/`, `docs/images/tutorials/`)

#### [NEW] `docs/tutorials/10-rotary-wrapping-surfacing.md`
- Covers Setup → **Rotary** tab (`X/A` & `Y/B`, direction polarity, chuck keep-out cylinder, clearance radius), **`WrapCylinder`** (V-Carve + Pocket + Contour wrapped onto `Ø40 mm` cylinder), and **`Rotary2Linear`** (`ContinuousSpiral`, **`AxialParallel`**, partial angular sectors `Start°..End°`).
- **Step-by-step screenshots (`docs/images/tutorials/10-rotary-wrapping-surfacing/`, 5 PNGs):**
  1. `01-setup-rotary-tab.png` — `Setup 1 → Rotary` tab showing `2-Linear + Rotary`, `X-Axis (A)`, chuck & clearance cylinder 3D gizmo.
  2. `02-wrap2d-vcarve-inspector.png` — `Wrapped V-Carve 'CHIP'` with `4th-Axis Indexing / Wrapping` card (`Wrap 2D`, `Wrap Radius = 20 mm`) + wrapped 3D toolpath.
  3. `03-wrap2d-sim-and-m6.png` — `CylindricalStock` simulation of wrapped `"CHIP"` + flute pocket (showing `✓ Verified Crash-Free` and `A=<deg>°` HUD).
  4. `04-rotary2linear-axial-parallel.png` — `4th-Axis Rotary Surfacing` Inspector switched to **`Axial Parallel`** + partial angular window (`-60°` to `+60°`) on `rotary_shaft.stl`.
  5. `05-gcode-grbl-wrap.png` — `G-Code Preview` set to **`grbl_rotary_wrap`** proving `A` is swapped onto linear `Y` with zero `A` words, contrasted with `linuxcnc_4axis` `G93`.

#### [NEW] `docs/tutorials/11-indexed-3plus1-milling.md`
- Covers cardinal `0°/90°/180°/270°` wrench flats on `rotary_shaft.stl` AND prismatic/angled STEP multi-face machining on `11-indexed-multiface-block.step` using **Align to Selected 3D Face**.
- **Step-by-step screenshots (`docs/images/tutorials/11-indexed-3plus1-milling/`, 5 PNGs):**
  1. `01-indexed-step-setup.png` — `11-indexed-multiface-block.step` chucked in square bar stock with rotary centerline & clearance cage (`--capture-focus setup:rotary`).
  2. `02-align-3d-face-inspector.png` — `4th-Axis Indexing / Wrapping` card in `Indexed 3+1` mode highlighting `Quick Index (0°/90°/180°/270°)` and **`Align to Selected 3D Face`**.
  3. `03-multi-face-toolpaths.png` — Generated multi-face `3+1` pockets & holes positioned around all indexed faces with safe clearance-radius retracts.
  4. `04-rotary-shaft-flats-sim.png` — `CylindricalStock` simulation of the 4 wrenching flats + continuous spiral collar/cam (`✓ Verified Crash-Free`).
  5. `05-gcode-indexed-linuxcnc.png` — `G-Code Preview` (`linuxcnc_4axis`) showing safe `G0 Z` clearance retract followed by `G0 A90.000` / `G0 A180.000` / `G0 A270.000`.

#### [NEW] `docs/tutorials/12-simultaneous-4axis-milling.md` (upgrading `10-simultaneous-4axis-milling.md`)
- Deep dive into **`OperationKind::Rotary4AxisSim`** (`X+Y+Z+A/B`) comparing **`Auto Surface-Normal Tilt`** (`Max Auto Tilt`, `Tilt Smoothing`) vs. **`Fixed Lateral Offset`**, and **`SimultaneousSpiral`** vs. **`HelicalFlowline`**.
- **Step-by-step screenshots (`docs/images/tutorials/12-simultaneous-4axis-milling/`, 5 PNGs):**
  1. `01-helical-setup-sim4ax.png` — `10-helical-flute-shaft.stl` in `Simultaneous 4-Axis (X+Y+Z+Rotary)` Setup (`--capture-focus setup:rotary`).
  2. `02-auto-tilt-spiral-passes.png` — `Simultaneous 4-Axis` Inspector (`Auto Surface-Normal Tilt`, `18°`, `3 cells` smoothing) + simultaneous helical spiral toolpaths.
  3. `03-fixed-offset-flowline.png` — `Simultaneous 4-Axis` Inspector in **`Axial / Helical Flowline`** + **`Fixed Lateral Offset (2.5 mm)`** mode showing off-centerline flowline strips.
  4. `04-simulation-xyza-hud.png` — Live cylindrical simulation highlighting active off-centerline `X`, `Y ≠ 0`, `Z`, `A` HUD readout and `✓ Verified Crash-Free` (plus Daylight Light theme companion view).
  5. `05-gcode-g93-grblhal.png` — `G-Code Preview` (`grblhal_4axis`) inspecting simultaneous `G93` `X... Y... Z... A... F...` blocks and `G94` rapid restoration.

#### [CLEANUP] Remove superseded M21 overview files (`09-4th-axis-rotary-wrapping-indexing.md`, `10-simultaneous-4axis-milling.md`, and their superseded image folders) and update [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md).

---

## Verification Plan

1. **Automated CAD, CAM, Sim, & Post Verification:**
   - `cargo test -p chip-cad tutorial_indexed` — confirms `tutorial_indexed_4side_block_step()` round-trips with exact planar faces & index angles.
   - `cargo test -p chip-project tutorial_rotary` — confirms all M23 tutorial projects serialize deterministically, validate with 0 errors, simulate in `CylindricalStock` with **0 critical errors**, and post-process cleanly across `linuxcnc_4axis`, `grblhal_4axis`, and `grbl_rotary_wrap`.
   - Run all CLI commands from Section 7 of Tutorials `10`–`12` directly with `chipcutter`.
2. **Visual Inspection & Link Audit:**
   - Capture all 15 headless Vulkan PNGs and inspect every PNG via `read_file`.
   - Verify 0 broken links across `README.md` and `docs/tutorials/*.md`.
3. **Workspace Gates:**
   - `cargo fmt --all -- --check`, `cargo clippy --workspace --all-targets -- -D warnings`, `cargo deny check`, and `WGPU_BACKEND=vulkan cargo test --workspace`.
