# Milestone 22 Implementation Plan — Dedicated 3-Axis Surface Milling Tutorials

## Goal

Replace the single Phase 5 overview page (`docs/tutorials/08-3d-mesh-surfacing.md`) with **two deep-dive, step-by-step illustrated 3-axis tutorials** matching Phase 4's `01`–`07` standard—giving **`3D Adaptive Roughing`**, **`3D Waterline / Z-Level`**, **`3D Parallel Finishing`**, **`3D Constant-Scallop`**, **3D Rest Machining**, and **`3D Pencil Cleanup`** complete parameter walkthroughs, minimal starter mesh assets, multi-step headless Vulkan screenshots, and verified CLI equivalents:

1. **`docs/tutorials/08-3d-adaptive-waterline.md`** — Mesh Import (`STL`/`OBJ`/`3MF`), **3D Adaptive Roughing**, 2D Sketch Containment Boundaries, & **3D Waterline / Z-Level**.
2. **`docs/tutorials/09-3d-surface-finishing.md`** — **3D Parallel Raster Finishing**, **3D Constant-Scallop**, **3D Rest Machining**, & **3D Pencil Fillet Cleanup**.

---

## Proposed Changes

### 1. Minimal Starter Mesh Assets & Seeded Tutorial Builders (`chip-project`, `chip-cli`, `docs/tutorials/assets/`)

#### [MODIFY] [crates/chip-project/src/demos.rs](file:///workspace/ChipCutter/crates/chip-project/src/demos.rs), [crates/chip-project/src/lib.rs](file:///workspace/ChipCutter/crates/chip-project/src/lib.rs)
- Add dedicated tutorial project builders so every tutorial step and screenshot is 100% reproducible and regression-tested in `chip-project`:
  - `build_tutorial_3d_adaptive_waterline_project(model_path, sha256, tools)`: loads `08-sculpted-dome.stl`, adds an optional 2D `Setup XY` containment sketch (`Dome Window`), **Op 1: `3D Adaptive Roughing`** (`T1` Ø6mm Flat, `Detect Flat Areas`, `0.4 mm / 0.25 mm` stock-to-leave), and **Op 2: `3D Waterline / Z-Level`** (`T4` Ø6mm Ball, `Adaptive Shallow Z-Stepdowns = true`, `WaterlineOrderMode::TopDownByLevel`, steep-wall finish with `0.0 mm` stock-to-leave).
  - `build_tutorial_3d_surface_finishing_project(model_path, sha256, tools)`: loads `09-relief-wave.obj` (and pairs with `08-sculpted-dome.stl` for the `Pencil3D` crease walkthrough), configuring **Op 1: `3D Adaptive Roughing`**, **Op 2: `3D Parallel Finishing`** (`45°` pass angle, `Use Target Cusp Height = 0.04 mm`, `Cross-Pass +90°`, `Slope Range 0°–60°`), **Op 3: `3D Constant-Scallop`** (`Inside-Out`, `Min/Max Stepover`, `3D Rest Machining` `FromPreviousTool`), and **Op 4: `3D Pencil Cleanup`** (`T11` Ø3mm Ball, `Num Offset Passes = 1`) on `08-sculpted-dome.stl`.
- Add `chip-project` unit test assertions verifying every new tutorial project round-trips `.chip.toml` deterministically, validates with `0` errors, simulates in `chip-sim` with **`0` critical errors**, and posts cleanly to `linuxcnc` and `grbl`.

#### [MODIFY] [crates/chip-cli/src/commands/inspect_sample.rs](file:///workspace/ChipCutter/crates/chip-cli/src/commands/inspect_sample.rs)
- Extend `chipcutter generate-sample-step --tutorial` with:
  - `08-3d-adaptive-waterline` (writes `08-sculpted-dome.stl` + optional `--empty-project` / seeded `--project-out`),
  - `09-3d-surface-finishing` (writes `09-relief-wave.obj` + optional `--empty-project` / seeded `--project-out`).
- Generate and check in `docs/tutorials/assets/09-relief-wave.obj` alongside existing `docs/tutorials/assets/08-sculpted-dome.stl`.

---

### 2. Illustrated Tutorials & Headless Vulkan Screenshots (`docs/tutorials/`, `docs/images/tutorials/`)

#### [NEW] `docs/tutorials/08-3d-adaptive-waterline.md` (replacing `08-3d-mesh-surfacing.md`)
- Follows the uniform 7-part tutorial structure (`Goal`, `Prerequisites`, `Step-by-Step GUI Walkthrough`, `Key Parameters Reference`, `Generate & 3D Simulation`, `Post-Processing`, `Headless CLI Equivalent`).
- **Step-by-step screenshots (`docs/images/tutorials/08-3d-adaptive-waterline/`, 5 PNGs):**
  1. `01-mesh-import-stock.png` — Bare `08-sculpted-dome.stl` imported with watertightness diagnostics & Setup Stock/WCS (`--capture-focus setup:stock`).
  2. `02-adaptive3d-passes.png` — Op 1 `3D Adaptive Roughing` Passes tab (`Optimal Load`, `Detect Flat Areas`, `3D Machining Boundary` with 2D sketch containment) + generated trochoidal roughing toolpath.
  3. `03-waterline3d-passes.png` — Op 2 `3D Waterline / Z-Level` Passes tab (`WaterlineOrderMode`, `Adaptive Shallow Z-Stepdowns`, `Slope Range`) + dense contour Z-level toolpath.
  4. `04-simulation-verified.png` — 3D Z-dexel machined stock colored by operation (`T1` rough terraces + `T4` smooth waterline dome) with `✓ Verified Crash-Free`.
  5. `05-gcode-preview.png` — Bottom `G-Code Preview` tab showing LinuxCNC ATC (`M6` `T1 → T4`) and explaining GRBL split-file output.

#### [NEW] `docs/tutorials/09-3d-surface-finishing.md`
- Deep-dive finishing guide covering **`Parallel3D`**, **`Scallop3D`**, **3D Rest Machining**, and **`Pencil3D`**.
- **Step-by-step screenshots (`docs/images/tutorials/09-3d-surface-finishing/`, 5 PNGs):**
  1. `01-relief-wave-import.png` — `09-relief-wave.obj` imported in Setup 1 with stock & WCS.
  2. `02-parallel3d-passes.png` — `3D Parallel Finishing` Inspector (`Use Cusp Height`, `Pass Angle = 45°`, `Cross-Pass (+90°)`, `Slope Range`) + raster finishing paths.
  3. `03-scallop3d-rest.png` — `3D Constant-Scallop` Inspector (`Target Cusp Height`, `Inside -> Out`, `Min/Max Stepover`, `3D Rest Machining` card) + geodesic iso-contours.
  4. `04-pencil3d-fillet.png` — `3D Pencil Cleanup` on `08-sculpted-dome.stl` (`Reference Tool Ø = 6 mm`, `Valley Threshold`, `Offset Passes`) highlighting the bi-tangent pedestal-dome crease path (`T11` Ø3mm Ball).
  5. `05-finishing-simulation.png` — Final machined Relief Wave Z-dexel stock showing crisp scalloped finish and `✓ Verified Crash-Free`.

#### [DELETE] Old M21 overview file `docs/tutorials/08-3d-mesh-surfacing.md` & `docs/images/tutorials/08-3d-mesh-surfacing/`
- Update [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md) (temporarily renumbering the Phase 5 rotary overview files or keeping links cleanly aligned until M23 expands them into `10`–`12`!).

---

## Verification Plan

1. **Automated Simulation & CLI Verification:**
   - `cargo test -p chip-project tutorial_3d` — confirms both `08` and `09` tutorial projects serialize deterministically, validate cleanly, simulate in `chip-sim` with **0 crashes / 0 gouges**, and post-process to GRBL + LinuxCNC.
   - Execute every CLI command listed in Section 7 of both `08-3d-adaptive-waterline.md` and `09-3d-surface-finishing.md` against `target/debug/chipcutter` to confirm zero errors.
2. **Visual Inspection & Link Audit:**
   - Capture all 10 headless Vulkan PNGs (`WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -a ...`) and visually inspect every PNG via `read_file` for crisp framing, readable Inspector cards, and `✓ Verified Crash-Free` badges.
   - Audit all Markdown links and image paths across `README.md` and `docs/tutorials/*.md` (`0` broken links).
3. **Workspace Gates:**
   - `cargo fmt --all -- --check`, `cargo clippy --workspace --all-targets -- -D warnings`, `cargo deny check`, and `WGPU_BACKEND=vulkan cargo test --workspace`.
