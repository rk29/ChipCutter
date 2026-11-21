# Milestone 42 Implementation Plan — Universal Control Tooltips, Heatmap Toggle & Playback Performance, & Unsaved-Changes Close Guard

## Goal

Ensure every interactive control in `chip-gui` explains itself on hover, make enabling/disabling the signed **Error Heatmap** fast and intuitive without slowing down simulation playback, and guard against accidental data loss by prompting to save unsaved changes when closing the application.

---

## Proposed Changes

### 1. Universal Hover Tooltips Across All GUI Controls (`chip-gui`)

#### [MODIFY] [crates/chip-gui/src/app/menus_toolbar.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/menus_toolbar.rs)
#### [MODIFY] [crates/chip-gui/src/app/viewport_controller.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/viewport_controller.rs)
#### [MODIFY] [crates/chip-gui/src/panels/browser.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/browser.rs)
#### [MODIFY] [crates/chip-gui/src/panels/setup_inspector.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/setup_inspector.rs)
#### [MODIFY] [crates/chip-gui/src/panels/operation_inspector.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector.rs)
#### [MODIFY] [crates/chip-gui/src/panels/model_inspector.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/model_inspector.rs)
#### [MODIFY] [crates/chip-gui/src/panels/sketch_inspector.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/sketch_inspector.rs)
#### [MODIFY] [crates/chip-gui/src/panels/simulation.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/simulation.rs)
#### [MODIFY] [crates/chip-gui/src/panels/gcode_preview.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/gcode_preview.rs)
#### [MODIFY] [crates/chip-gui/src/panels/diagnostics.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/diagnostics.rs)
#### [MODIFY] [crates/chip-gui/src/panels/tool_library.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/tool_library.rs)
#### [MODIFY] [crates/chip-gui/src/panels/machine_manager.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/machine_manager.rs)
#### [MODIFY] [crates/chip-gui/src/panels/mesh_import_modal.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/mesh_import_modal.rs)
#### [MODIFY] [crates/chip-gui/src/panels/vector_import_modal.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/vector_import_modal.rs)
- Audit every interactive widget (`button`, `small_button`, `checkbox`, `selectable_value`, `radio_value`, `ComboBox`, `DragValue`, `Slider`, `TextEdit`, `icon_button`, `edit_param_length`) across all panels, toolbars, viewport overlays, and modals.
- Attach concise, machinist-friendly `.on_hover_text(...)` tooltips explaining what each control does (and any keyboard shortcut or unit convention where relevant), using pure ASCII arrows (`->`) to avoid missing font glyphs.

---

### 2. Error Heatmap Display Toggle & Simulation Playback Performance (`chip-gui`, `chip-sim`)

#### [MODIFY] [crates/chip-gui/src/panels/simulation.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/simulation.rs)
#### [MODIFY] [crates/chip-sim/src/tridexel/mesher.rs](file:///workspace/ChipCutter/crates/chip-sim/src/tridexel/mesher.rs)
#### [MODIFY] [crates/chip-sim/src/heightmap/mesher.rs](file:///workspace/ChipCutter/crates/chip-sim/src/heightmap/mesher.rs)
- **Clear Heatmap Enable/Disable Control:**
  - Make **`Error Heatmap`** a prominent, dedicated checkbox (mutually clear vs. `Color by Op`: disabling `Color by Op` visually when `Error Heatmap` is active, or presenting clear tooltips so toggling `Error Heatmap` off immediately restores fast non-heatmap rendering).
  - Add a smart playback guard / option: during continuous timeline playback (`is_playing == true`), avoid recomputing expensive full-mesh per-vertex target deviation queries on every single animation frame unless explicitly paused/scrubbed, or accelerate per-vertex heatmap lookup using `rayon` parallel row evaluation and direct cached target-endpoint queries—making both static heatmap toggling and live playback fast!
- **Mesher Optimization (`chip-sim`):**
  - Profile and optimize `tessellate_tridexel_stock_mesh_colored` (e.g., hoist color-mode dispatch out of inner loops, parallelize vertex color computation across slices, and fast-path non-heatmap modes with zero target lookups).

---

### 3. Unsaved-Changes Confirmation Prompt on Application Close (`chip-gui`)

#### [MODIFY] [crates/chip-gui/src/app/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/mod.rs)
#### [MODIFY] [crates/chip-gui/src/app/loader.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/loader.rs)
- Track pending close confirmation (`show_unsaved_close_modal: bool`, `allow_close_without_prompt: bool`) on `ChipCutterApp`.
- In `eframe::App::ui` (`app/mod.rs`):
  - Check `ctx.input(|i| i.viewport().close_requested())`.
  - When `self.is_project_dirty()` is `true` and `!self.allow_close_without_prompt` and `self.self_test.is_none()`:
    - Issue `ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose)` and open the centered **Unsaved Changes** modal dialog.
  - Provide three clear actions:
    - **`Save & Close`** — saves to `self.project_file_path` (or opens native Save As dialog if untitled), and if saved cleanly sets `allow_close_without_prompt = true` and emits `ViewportCommand::Close`.
    - **`Discard & Close`** — sets `allow_close_without_prompt = true` and emits `ViewportCommand::Close`.
    - **`Cancel`** — dismisses the dialog and keeps the workspace open.
- Also wire `File -> Quit` (`Ctrl+Q`) through the exact same unsaved-changes guard!

---

## Verification Plan

1. **Unit Tests:**
   - Add tests verifying `is_project_dirty()` triggers the unsaved-changes close guard and resets cleanly on save / discard.
   - Add simulation panel tests verifying disabling `Error Heatmap` skips deviation coloring and produces fast mesh revisions.
2. **Workspace Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-sim -p chip-gui`
