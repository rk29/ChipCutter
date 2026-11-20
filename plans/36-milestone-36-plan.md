# Milestone 36 Implementation Plan — Interactive Model Positioning & Scaling UI, Mesh Import Orientation, & Setup WCS Alignment Polish

## Goal

Expose intuitive, interactive **Model Transform, Orientation, & Scaling** controls in `chip-gui` for both STEP solids and STL/OBJ/3MF triangle meshes—including 1-click **`Y-Up → Z-Up`**, **`±90° X/Y/Z`**, 3D viewport **`Lay Flat on Clicked Face…`**, **`Scale` (Uniform Lock or X/Y/Z)** + **`Target Size X/Y/Z`**, **`Center XY`**, **`Drop Bottom to Z=0`**, and exact numeric/expression fields—plus quick orientation/scale preview in the Mesh Import modal and arbitrary X-axis clocking angle in Setup WCS.

---

## Proposed Changes

### 1. Model Transform & Orientation Inspector Panel (`chip-gui`)

#### [NEW] `crates/chip-gui/src/panels/model_inspector.rs`
#### [MODIFY] [crates/chip-gui/src/panels/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/mod.rs)
#### [MODIFY] [crates/chip-gui/src/panels/browser.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/browser.rs)
#### [MODIFY] [crates/chip-gui/src/app/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/mod.rs)
#### [MODIFY] [crates/chip-gui/src/app/pane_behavior.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/pane_behavior.rs)
- Add `InspectorFocus::Model { model_idx: usize }` so clicking a model row in the CAM Browser (`Models` section) focuses the right-hand Inspector on the **Model Transform & Orientation** panel (and adds a quick `Orient / Move…` button next to the model entry).
- Store both the untransformed base `raw_doc: Arc<CadDocument>` and the derived transformed `doc: Arc<CadDocument>` on `LoadedModelState` so editing `ModelEntry.transform` (or Undo/Redo `Ctrl+Z`/`Ctrl+Y`) recomputes the transformed `CadDocument`, `CadPicker`, `TopologyGraph`, and `RenderMesh` cleanly in milliseconds without re-parsing STEP/STL bytes from disk!
- Render clean, aligned Inspector cards in `model_inspector.rs`:
  1. **Model Info & Live Bounding Box:** Filename, format (`STEP B-Rep` vs. `STL/OBJ/3MF Mesh`), face/triangle count, and live transformed `Size X × Y × Z` and `Z Range [min .. max]` in active display units (`mm` / `in`).
  2. **Quick Orientation:**
     - **`Y-Up → Z-Up (+90° X)`** (most common fix for Blender / Fusion / 3D-print STLs)
     - **`-90° X` / `+90° X`**, **`-90° Y` / `+90° Y`**, **`-90° Z` / `+90° Z`**
     - **`Lay Flat on Clicked Face…`** (enters `ViewportPickMode::PickModelLayFlatFace { model_idx }`, showing the top-center viewport HUD banner `"Click any planar STEP face or STL triangle to lay it flat (-Z) on the table (Esc to cancel)"`; raycast picks either a B-Rep planar face normal or a mesh triangle normal and applies `lay_flat_normal_to_neg_z` + drops bottom to `Z = 0`!).
  3. **Scale & Target Dimensions:**
     - **`Lock Uniform Scale`** toggle (checked by default) with **`Scale`** multiplier (or independent **`Scale X / Y / Z`** when unlocked, showing a brief advisory if non-uniformly scaling a STEP solid with cylindrical/arc features).
     - Quick scale presets: **`0.5×`**, **`2×`**, **`in → mm (25.4×)`**, **`mm → in (÷25.4)`**, **`1.0×`**.
     - **`Target Size X / Y / Z`** fields (unit-aware `mm`/`in`): editing any target dimension automatically computes the exact uniform scale factor to fit a specific stock blank or plaque!
  4. **Quick Placement:**
     - **`Center XY on Origin`**
     - **`Drop Bottom to Z = 0`**
     - **`Align Top to Z = 0`**
     - **`Reset Transform`**
  5. **Exact Rotation & Translation:**
     - **`Rotate X / Y / Z (°)`** numeric fields (applied cleanly around model center or world origin).
     - **`Move X / Y / Z`** unit-aware expression fields (`mm` / `in`).
  6. **Contextual Guidance Note:** Brief tip clarifying that *Model Transform* fixes raw CAD/STL export orientation/size for the whole project, whereas *Setup → WCS & Post* defines per-operation fixture datums (`G54`/`G55`, Op-1/Op-2 flip).
- Whenever `ModelEntry.transform` changes (or on Undo/Redo when the transform differs from the currently built `LoadedModelState`), re-apply `raw_doc.transformed(&entry.transform)`, rebuild `CadPicker` + `TopologyGraph`, increment `cad_revision`, mark dependent operations `Dirty`, and re-fit camera if requested.
- Support `--capture-focus model` (or `model:0`) in [capture.rs](file:///workspace/ChipCutter/crates/chip-gui/src/capture.rs) for deterministic Tutorial 16 screenshots!

---

### 2. Quick Orientation Controls in Mesh Import Modal (`chip-gui`)

#### [MODIFY] [crates/chip-gui/src/panels/mesh_import_modal.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/mesh_import_modal.rs)
#### [MODIFY] [crates/chip-gui/src/app/loader.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/loader.rs)
- Extend `MeshImportCommit` with `pub transform: ModelTransform`.
- Add a compact **Initial Orientation, Scale, & Placement** section inside the **Import 3D Mesh (STL / OBJ / 3MF)** modal:
  - Quick buttons: **`As-Is`**, **`Y-Up → Z-Up (+90° X)`**, **`+90° X`**, **`+90° Y`**, **`+90° Z`**, uniform **`Scale`** multiplier (default `1.0`), plus checkboxes **`Center XY`** and **`Drop Bottom to Z = 0`**.
  - Live bounding-box (`Size X × Y × Z`) updates immediately so the machinist sees the exact upright, scaled dimensions *before* clicking **Import Mesh**.
- Persist the chosen `ModelTransform` directly into `ModelEntry.transform` when committing the mesh import.

---

### 3. Setup WCS Arbitrary X-Axis Clocking Angle (`chip-cam`, `chip-gui`)

#### [MODIFY] [crates/chip-cam/src/wcs.rs](file:///workspace/ChipCutter/crates/chip-cam/src/wcs.rs)
#### [MODIFY] [crates/chip-gui/src/panels/setup_inspector.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/setup_inspector.rs)
- Add optional `#[serde(default, skip_serializing_if = "is_zero_angle")] pub extra_z_rot_deg: f64` to `WcsOrientationMode` variants (omitted when `0.0` for 100% TOML backward compatibility), allowing fine angular rotation around the Setup `+Z` axis (e.g., `15°` or `45°` vise clocking) in addition to `Clock X 90°`.
- Expose a compact **`Fine Z Rot (°)`** field alongside `Clock X 90°` in `setup_inspector.rs`, and wire `ViewportPickMode::PickModelLayFlatFace` in `viewport_controller.rs`.

---

## Verification Plan

1. **Unit Tests (`chip-cam`, `chip-gui`):**
   - `WcsOrientationMode` with `extra_z_rot_deg == 0.0` omits the field in TOML and produces bit-identical `WcsFrame`; non-zero `extra_z_rot_deg` rotates Setup `+X`/`+Y` accurately around `+Z`.
   - `CaptureFocus::parse("model")` and `CaptureFocus::parse("model:0")` parse cleanly.
   - `ModelTransform` undo/redo in GUI restores both `ModelEntry.transform` and the rendered `LoadedModelState` geometry.
2. **Compilation & Quality Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo test --locked -p chip-cam -p chip-gui --lib -- --skip renders_`
3. **Headless Vulkan Smoke Test:**
   - Launch `chip-gui` via `WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -a` with `--capture-focus model` and visually inspect the Model Transform Inspector PNG via `read_file`.
