# Milestone 37 Implementation Plan — Illustrated Tutorial 16: Rotating, Laying Flat, Scaling, & Moving STEP & STL Models

## Goal

Author a complete, illustrated 7-part tutorial (**Tutorial 16**) teaching machinists both workflows for positioning, scaling, and orienting imported 3D parts—**non-destructive Model Transform** (especially for Y-up, rotated, or oversized/undersized STL/OBJ/3MF meshes and off-origin STEP files) and **Setup WCS Orientation & Multi-Setup Op-1/Op-2 Flipping** (for tilted STEP solids and two-sided machining)—backed by minimal reproducible starter assets and visually verified headless Vulkan screenshots.

---

## Proposed Changes

### 1. Reproducible Minimal Tutorial Starter Assets (`chip-cad`, `chip-cli`, `docs/tutorials/assets/`)

#### [MODIFY] [crates/chip-cad/src/fixtures.rs](file:///workspace/ChipCutter/crates/chip-cad/src/fixtures.rs)
#### [MODIFY] [crates/chip-cli/src/commands/inspect_sample.rs](file:///workspace/ChipCutter/crates/chip-cli/src/commands/inspect_sample.rs)
- Add two minimal tutorial fixture generators exposed via `chipcutter generate-sample-step`:
  - `--tutorial 16-orient-stl`: Generates `docs/tutorials/assets/16-yup-bracket.stl` — a crisp, watertight sculpted mounting block exported **Y-up** (lying sideways along `+Y`) and offset off-origin, ideal for practicing **`Y-Up → Z-Up (+90° X)`**, **`Lay Flat on Face`**, **`Center XY`**, and **`Drop Bottom to Z=0`**.
  - `--tutorial 16-orient-step`: Generates `docs/tutorials/assets/16-tilted-bracket.step` — a clean prismatic STEP bracket tilted `30° X / 25° Z` with a top pocket and mounting holes, ideal for practicing **Setup WCS `Z & X` / `XY Plane` alignment**, `Flip +Z`, `Clock X 90°`, 27-point origin, `Additional Offset`, and adding a flipped **Setup 2 (`G55`)** for bottom-side operations.
- Commit both minimal starter files under `docs/tutorials/assets/`.

---

### 2. Illustrated Tutorial 16 & Headless Vulkan Screenshots (`docs/tutorials/`, `docs/images/tutorials/`)

#### [NEW] `docs/tutorials/16-model-orientation-positioning.md`
#### [NEW] `docs/images/tutorials/16-model-orientation-positioning/*.png`
- Follow the uniform 7-part tutorial structure from [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md):
  1. **Goal & Model Transform vs. Setup WCS Decision Guide:** Concise comparison table explaining when to move/rotate the *Model* (fixing Y-up STLs, laying a mesh flat, centering raw CAD exports) vs. when to orient the *Setup WCS* (tilted STEP assemblies, angled vises, Op-1 `G54` + Op-2 `G55` part flips).
  2. **Part A — Fixing & Scaling a Y-Up STL Mesh on Import & via Model Transform:**
     - Importing `16-yup-bracket.stl` with live orientation & scale preview in the Mesh Import modal.
     - Using the **Model Transform Inspector** (`Y-Up → Z-Up (+90° X)`, `Lay Flat on Clicked Face…`, **`Scale` / `Target Size`** to resize a mesh to fit a specific billet, `Center XY`, `Drop Bottom to Z=0`, and exact `Rotate/Move` fields).
  3. **Part B — Rotating, Scaling, & Moving a STEP Solid Non-Destructively (Preserving `Ctrl+R` Reload):**
     - Applying a `ModelTransform` (uniform scale, rotation, and translation) to a STEP part and verifying exact B-Rep hole/face picking and associative STEP reload keep the saved transform intact.
  4. **Part C — Aligning a Tilted STEP Part via Setup WCS & Multi-Setup Op-2 Flip:**
     - Selecting **`Z & X`** or **`XY Plane`** on `16-tilted-bracket.step`, using **`Flip +Z`**, **`Clock X 90°`**, **`Fine Z Rot (°)`**, **`Align Top`**, 27-point Stock/Model WCS Origin, and **`Additional Offset (X/Y/Z)`**.
     - Adding **Setup 2 (`G55`)** with **`Flip +Z`** to machine the underside without duplicating or modifying the STEP file.
  5. **Key Parameters Reference Table** covering both Model Transform and Setup WCS controls.
  6. **Generate (`Ctrl+G`), 3D Simulation Verification, & Post-Processing.**
  7. **Headless CLI Equivalents (`chipcutter transform-model`, `import-mesh`, `validate`, `build`).**
- Capture 4–5 crisp headless Vulkan screenshots into `docs/images/tutorials/16-model-orientation-positioning/` using `WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -a` and visually inspect every PNG via `read_file`:
  1. `01-yup-stl-before-transform.png` — Sideways Y-up STL imported before orientation.
  2. `02-model-transform-inspector.png` — Upright, centered, `Z=0`-aligned STL with the **Model Transform & Orientation** Inspector visible.
  3. `03-tilted-step-wcs-align.png` — `16-tilted-bracket.step` aligned via Setup WCS `Z & X` / `XY Plane` with 27-point origin & WCS triad.
  4. `04-setup2-flipped-op2.png` — Setup 2 (`G55`) with `Flip +Z` for bottom-side machining + verified simulation toolpaths.

---

### 3. Curriculum & Roadmap Index Updates

#### [MODIFY] [docs/tutorials/15-machine-definitions-tool-sets.md](file:///workspace/ChipCutter/docs/tutorials/15-machine-definitions-tool-sets.md)
- Add **Next:** link pointing to Tutorial 16.
#### [MODIFY] [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md)
- Add Tutorial 16 to the curriculum table and `16-yup-bracket.stl` / `16-tilted-bracket.step` to the starter assets table.
#### [MODIFY] [README.md](file:///workspace/ChipCutter/README.md)
- Update tutorial curriculum range (`00`–`16`) and mention non-destructive STEP/STL model orientation & positioning in Highlights.
#### [MODIFY] [specs/10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md)
- Mark **Phase 10 (`M35`–`M37`)** Completed once all verification gates pass.

---

## Verification Plan

1. **Automated Asset & Simulation Test (`chip-project`):**
   - Verify both `16-yup-bracket.stl` (after `y_up_to_z_up + center_xy + drop_bottom_to_z0`) and `16-tilted-bracket.step` (with WCS alignment + Setup 2 flip) validate, generate toolpaths, and simulate with `0` critical crashes/gouges.
2. **Markdown & Screenshot Audit:**
   - Run the broken-link and orphaned-PNG audit across `README.md` and `docs/tutorials/*.md` (`0` broken links, `0` orphaned PNGs), and visually verify every new PNG via `read_file`.
3. **Full Workspace Gate:**
   - `cargo fmt --all -- --check`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
