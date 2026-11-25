# Milestone 91 Implementation Plan — Tree Context Menus, Drag-and-Drop Reordering, & Selection-Driven Right-Panel Editors

## Goal

Make every node in the Project Tree interactive and self-sufficient: rich right-click context menus combining common actions (`Rename`, `Duplicate`, `Suppress`, `Show/Hide`, `Move Up/Down`, `Delete`) with strictly type-safe item-specific actions (e.g., adding only valid sub-item types), drag-and-drop reordering within every semantically reorderable section, and instant selection-to-right-panel editor activation for every node type (including dedicated right-side inspectors for the Project, Keep-Out Regions, Project Tools, and Fixtures).

---

## Problem Diagnosis

1. **Right-Side Inspector Gaps for Tree Nodes (`crates/chip-gui/src/panels/mod.rs`, `app/pane_behavior.rs`):**
   - `InspectorFocus` currently covers only `CadEntity`, `Model`, `Sketch`, `Setup`, and `Operation`. Selecting the **Project root**, a **Keep-Out Region**, a **Project Tool**, or a specific **Fixture** has no dedicated right-panel target (Keep-Out Regions could only be edited inline inside an Operation's `[Geometry]` Avoid card).
2. **Inconsistent Actions & Reordering Limited to Operations:**
   - Currently only Operations support drag-and-drop reordering and rich context menus; Setups, Sketches, Sketch Entities, Keep-Out Regions, and Fixtures lack consistent right-click menus (`Rename`, `Duplicate`, `Delete`, `+ Add <allowed child>`) and drag-and-drop reordering.

---

## Proposed Changes

### 1. Right-Click Context Menus & Inline Node Rename (`browser/context_menus.rs`)

#### [NEW] `crates/chip-gui/src/panels/browser/context_menus.rs`
#### [MODIFY] `crates/chip-gui/src/panels/browser/mod.rs`
#### [MODIFY] `crates/chip-gui/src/app/pane_behavior.rs`
- Attach consistent right-click context menus (`response.context_menu`) to every tree node (with atomic Undo & downstream cache/Rest/Carryover dirty propagation):
  - **Project Root:** `Project Properties...`, `Manage Tools...`, `+ New Setup`, `+ New 2D Sketch (Setup XY)`, `+ New Keep-Out Region`, `Import STEP Model...`, `Import 3D Mesh...`, `Import 2D Vector...`, `Expand All`, `Collapse All`.
  - **`Models` folder:** `Import STEP Model...`, `Import 3D Mesh (.stl / .obj / .3mf)...`, `Sample Projects Gallery...`.
  - **Model item:** `Inspect / Transform`, `Reload from Disk (Ctrl+R)`, `Replace Model...`, `Remove Model`.
  - **`Sketches` folder:** `+ New 2D Sketch (Setup XY)`, `+ New 2D Sketch on 3D Face...`, `Import 2D Vector (.dxf / .svg)...`.
  - **Sketch item:** `Add Entity ->` submenu (`Rectangle`, `Circle`, `Slot`, `Hex / Polygon`, `Bolt Circle`, `CAM Text`), `Look At Sketch`, `Show / Hide`, `Rename (F2)`, `Duplicate`, `Move Up / Down`, `Delete`.
  - **Sketch Entity item:** `Toggle Construction`, `Rename (F2)`, `Duplicate`, `Move Up / Down`, `Delete`.
  - **`Keep-Out Regions` folder:** `+ New Keep-Out Region`.
  - **Keep-Out Region item:** `Edit Keep-Out Region`, `Suppress / Unsuppress`, `Apply to Active Setup Ops`, `Rename (F2)`, `Duplicate`, `Move Up / Down`, `Delete`.
  - **`Project Tools` folder:** `Manage Tools...`, `Remove Unused Tools`.
  - **Project Tool item:** `Edit in Tool Library...`, `Sync from Machine Library`, `Remove from Project` (guarded if referenced by operations).
  - **`Setups` folder:** `+ New Setup`.
  - **Setup item:** `Set Active`, `Generate Setup`, `Align View to Setup Top`, `Add Operation ->` categorized submenu (2.5D / 3D / Undercut & Deburr / 4th-Axis — only valid operation kinds), `Add Fixture ->` submenu (`Vise Jaws`, `Toe Clamp`, `Spoilboard`, `Box Keep-Out`, `Cylinder Keep-Out`, `Flip-Jig Wizard...`, `Auto Soft-Jaw Wizard...`), `Rename (F2)`, `Duplicate Setup`, `Move Up / Down`, `Delete Setup` (disabled when only 1 Setup remains).
  - **`Fixtures` folder:** `Add Fixture ->` (`Vise Jaws`, `Toe Clamp`, `Spoilboard`, `Box Keep-Out`, `Cylinder Keep-Out`), `Flip-Jig Wizard...`, `Auto Soft-Jaw Wizard...`.
  - **Fixture item:** `Suppress / Unsuppress`, `Rename (F2)`, `Duplicate`, `Move Up / Down`, `Delete`.
  - **Operation item:** `Generate Operation`, `Show / Hide Path`, `Suppress / Unsuppress`, `Rename (F2)`, `Duplicate (Ctrl+D)`, `Move Up (Ctrl+Up)` / `Move Down (Ctrl+Down)`, `Delete (Del)`.
- Support clean inline **Rename** (triggered strictly via right-click `Rename` or `F2`—no cluttered permanent `Rename` buttons in the tree!) on named user items (Setups, Operations, Sketches, Sketch Entities, Keep-Out Regions, Fixtures), committing via `Enter` / focus-lost with atomic Undo and cancelling via `Esc`.

---

### 2. Drag-and-Drop Reordering Across All Reorderable Sections (`browser/drag_reorder.rs`)

#### [NEW] `crates/chip-gui/src/panels/browser/drag_reorder.rs`
- Generalize the existing operation drag-and-drop engine (grab cursor, drag payload, crisp accent-colored horizontal insertion line, auto-drop on pointer release) across every section where order is meaningful:
  1. **Operations within their parent Setup** (`move_operation_to_slot` — preserves existing Rest Machining dirty propagation & `InspectorFocus::Operation` tracking).
  2. **Setups within `Setups`** (`move_setup_to_slot` — updates `active_setup_idx` and `InspectorFocus`, and marks any Setup using `carryover_from_setup_id` `Dirty` if upstream ordering changed).
  3. **Sketches within `Sketches`** (`move_sketch_to_slot` — updates `InspectorFocus::Sketch` cleanly).
  4. **Sketch Entities within their parent Sketch** (`move_sketch_entity_to_slot` — updates `selected_sketch_entity_idx` cleanly).
  5. **Keep-Out Regions within `Keep-Out Regions`** (`move_avoid_region_to_slot` — updates `InspectorFocus::KeepOutRegion` cleanly).
  6. **Fixtures within their parent Setup's `Fixtures`** (`move_fixture_to_slot` — updates selected fixture index cleanly).
- Enforce strict section boundaries: items can only be dropped within their own parent container (`Models` and `Project Tools` do not accept drag reordering).

---

### 3. Selection-Driven Right-Side Panel Editors

#### [MODIFY] `crates/chip-gui/src/panels/mod.rs`
#### [NEW] `crates/chip-gui/src/panels/keepout_inspector.rs`
#### [NEW] `crates/chip-gui/src/panels/project_tool_inspector.rs`
#### [MODIFY] `crates/chip-gui/src/panels/project_properties.rs`
#### [MODIFY] `crates/chip-gui/src/panels/setup_inspector/fixtures_tab.rs`
#### [MODIFY] `crates/chip-gui/src/app/pane_behavior.rs`
#### [MODIFY] `crates/chip-gui/src/app/viewport_controller.rs`
- Extend `InspectorFocus` so selecting **any** item in the Project Tree immediately activates its editor in the right-hand Inspector pane:
  - `InspectorFocus::Project` -> renders the aligned two-column **Project Inspector** directly in the right panel (sharing the exact same `General` & `Project Summary` cards as `Edit -> Project Properties...` so there is zero duplicated code!).
  - `InspectorFocus::Model { model_idx }` -> **Model Transform & Orientation** inspector.
  - `InspectorFocus::Sketch { sketch_idx }` -> **Sketch Inspector** (selecting a child Sketch Entity sets `selected_sketch_entity_idx = Some(entity_idx)` and focuses its editor card!).
  - `InspectorFocus::KeepOutRegion { region_idx }` -> new **`keepout_inspector.rs`** right-panel editor with aligned `section_card` + `inspector_grid`: Name, Suppressed toggle, optional `Radial Clearance` / `Axial Clearance` overrides, Setup Fixture chips, Keep-Out Geometry selections + `"Pick 3D Avoid Geometry..."` toggle + 2D Sketch `+ Avoid` quick-add, and `"Linked Operations"` checkboxes across Setups — rendering live coral/red 3D viewport keep-out overlays while selected!
  - `InspectorFocus::ProjectTool { tool_idx }` -> new **`project_tool_inspector.rs`** right-panel inspector showing the tool's 2D cutter schematic, key dimensions & offsets summary, active referencing operations list, `"Edit in Tool Library..."`, `"Sync from Machine Library"`, and `"Remove from Project"` (guarded when in use).
  - `InspectorFocus::Setup { setup_idx }` -> **Setup Inspector** (selecting a child Fixture sets `setup_tab = SetupTab::Fixtures` and highlights/scrolls to that `fixture_idx`!).
  - `InspectorFocus::Operation { setup_idx, op_idx }` -> **Operation Inspector**.

---

## Verification Plan

1. **Headless `egui` Interaction & Reorder Tests (`chip-gui`):**
   - Test selecting every tree node type (`Project`, `Model`, `Sketch`, `SketchEntity`, `KeepOutRegion`, `ProjectTool`, `Setup`, `Fixture`, `Operation`) activates the exact corresponding `InspectorFocus` and renders its right-side editor cleanly with zero panics.
   - Test right-click context menu actions (add valid sub-items, inline `Rename`, `Duplicate`, `Suppress`, `Delete`) push atomic Undo snapshots and propagate `Dirty` invalidation properly.
   - Test drag-and-drop helper functions (`move_operation_to_slot`, `move_setup_to_slot`, `move_sketch_to_slot`, `move_sketch_entity_to_slot`, `move_avoid_region_to_slot`, `move_fixture_to_slot`) including focus preservation, Rest/Carryover dirty propagation, and cross-section drop rejection.
2. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check && cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
