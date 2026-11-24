# Milestone 89 Implementation Plan — Simulation, Project / Model / Sketch Polish, Multi-Setup Batch Actions, & Phase 24 Spec Sync

## Goal

Close the remaining simulation, project/model/sketch, vector import, and multi-setup export UI gaps identified in the backend-vs-GUI audit, and synchronize specifications so every user-facing backend capability in ChipCutter has clean, discoverable GUI parity.

---

## Problem Diagnosis

1. **Simulation Metrology & Batch Gaps (`panels/simulation/`):**
   - `SimulationPanelState::heatmap_max_scale_mm` is hardcoded to `1.0 mm` with no control in `metrology_ui.rs`.
   - `SimVerificationConfig` tolerances (`gouge_tol_mm`, `rapid_crash_tol_mm`, `radial_wall_skin_mm`) are hardcoded to defaults in `worker.rs` with no user override controls.
   - `chipcutter simulate --all-setups --json --report <path>` has no GUI equivalent for batch-verifying multi-setup jobs or exporting a JSON metrology report.
2. **Project, Model, Sketch, & Vector Import Polish:**
   - `ProjectDocument::project_name` is display-only everywhere in the GUI (no rename control).
   - `VectorImportModalState::arc_fit_tolerance_mm` is stored in modal state and wired to both DXF/SVG importers, but has no widget in `vector_import_modal.rs`.
   - `ModelEntry::unit_override` and `ModelEntry::role` (`Part` vs. `Fixture`) cannot be inspected or edited post-import in `model_inspector.rs`.
   - `SketchAnchor::setup_id` cannot be reassigned to another Setup in `sketch_inspector/mod.rs`; `SketchTextSpec::arc_fit_tolerance_mm` and `SketchFontSource::CustomFile::face_index` are absent from `entity_editors.rs`.
3. **Multi-Setup G-Code Export & Post Profile Visibility (`panels/gcode_preview.rs`):**
   - GUI exports one active Setup at a time (no **Export All Setups…** batch workflow matching `chipcutter build`), and `.post.toml` profile details cannot be inspected or cloned/customized inside the GUI.

---

## Proposed Changes

### 1. Simulation Heatmap Scale, Verification Tolerances, & Multi-Setup Report Export

#### [MODIFY] `crates/chip-gui/src/panels/simulation/mod.rs`
#### [MODIFY] `crates/chip-gui/src/panels/simulation/metrology_ui.rs`
#### [MODIFY] `crates/chip-gui/src/panels/simulation/worker.rs`
- Pass `&mut self.heatmap_max_scale_mm` into `metrology_ui.rs` and render a unit-aware **`Heatmap Max Scale`** `DragValue` (e.g., `0.05..20.0 mm`, default `1.0 mm`) alongside `Tolerance`.
- Add a collapsible **`Verification Tolerances`** section in the Simulation panel wired to `SimVerificationConfig::{gouge_tol_mm, rapid_crash_tol_mm, radial_wall_skin_mm}` (with a `"Reset Defaults"` button) and pass it into `worker.rs`.
- Add **`Export Metrology JSON…`** (saving structured error-volume & crash report JSON) and **`Simulate All Setups`** (when `project.setups.len() > 1`, running the end-of-job multi-setup verification chain and displaying per-setup + cumulative summary).

---

### 2. Project Rename, Vector Import Tolerance, Model Inspector, & Sketch Polish

#### [MODIFY] `crates/chip-gui/src/panels/browser.rs`
#### [MODIFY] `crates/chip-gui/src/panels/vector_import_modal.rs`
#### [MODIFY] `crates/chip-gui/src/panels/model_inspector.rs`
#### [MODIFY] `crates/chip-gui/src/panels/sketch_inspector/mod.rs`
#### [MODIFY] `crates/chip-gui/src/panels/sketch_inspector/entity_editors.rs`
- **Project Rename (`browser.rs`):** Allow inline renaming of `project.project_name` via double-click / edit pencil icon on the root project node in the CAM Browser (with atomic Undo).
- **Vector Import `arc_fit_tolerance_mm` (`vector_import_modal.rs`):** Render the missing unit-aware `Arc-Fit Tolerance` `DragValue` beside `Join Tolerance`.
- **Model Inspector (`model_inspector.rs`):** Display and allow editing `ModelEntry::role` (`Target Part` / `Fixture / Workholding`) and, for triangle mesh entries, `ModelEntry::unit_override` (triggering clean mesh reload + Undo snapshot).
- **Sketch Inspector (`sketch_inspector/mod.rs`, `entity_editors.rs`):**
  - Add a **`Setup`** reassignment `ComboBox` for `SketchAnchor::SetupXy` / `SketchAnchor::CadFace` when `project.setups.len() > 1`.
  - Expose `SketchTextSpec::arc_fit_tolerance_mm` and (for multi-face `.ttc` custom font files) `SketchFontSource::CustomFile::face_index` in `entity_editors.rs`.

---

### 3. Multi-Setup Batch G-Code Export, Post Profile Inspector, & Phase 24 Spec Sync

#### [MODIFY] `crates/chip-gui/src/panels/gcode_preview.rs`
#### [MODIFY] `crates/chip-gui/src/app/menus.rs`
#### [MODIFY] `specs/05-operations.md`
#### [MODIFY] `specs/06-simulation.md`
#### [MODIFY] `specs/08-ui-viewport.md`
#### [MODIFY] `specs/10-roadmap.md`
- Add **`Export All Setups…`** in `gcode_preview.rs` and `File → Export All Setups…` (prompting for an output directory and writing each Setup's sanitized G-code file using the same safety rules as `chipcutter build`, reporting per-setup status & warnings).
- Add a **`Post Profile Inspector / Customizer`** drawer/modal accessible from `gcode_preview.rs` & `setup_inspector` displaying the active `PostProfile` settings (units, decimals, arc mode IJK/R, drill-cycle expansion, tool-change mode, G43/G41/G42, rotary post config) as read-only for built-in profiles with a 1-click **`Clone to Custom .post.toml`** workflow (mirroring `Machine Manager`!).
- Synchronize `specs/05-operations.md`, `specs/06-simulation.md`, `specs/08-ui-viewport.md`, and mark **Phase 24 (Completed — M87–M89)** in `specs/10-roadmap.md`.

---

## Verification Plan

1. **Headless `egui` & Export Regression Tests (`chip-gui`):**
   - Test `heatmap_max_scale_mm` and `SimVerificationConfig` controls + JSON metrology report serialization.
   - Test project inline rename + Undo, `vector_import_modal` `arc_fit_tolerance_mm` widget, `model_inspector` role/unit edits, and `sketch_inspector` Setup reassignment & text arc-fit/face-index controls.
   - Test **Export All Setups** multi-setup batch write (verifying sanitized filenames + non-empty G-code per setup) and `PostProfile` read-only built-in inspection + clone-to-custom `.post.toml` roundtrip.
2. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check && cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
