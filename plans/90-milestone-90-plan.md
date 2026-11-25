# Milestone 90 Implementation Plan — Procedural Tree Icon Suite & Hierarchical Project Tree-View Browser Architecture

## Goal

Replace the left-hand CAM Browser's flat card stack with a true collapsible **Project Tree View** rooted at the active project (`<project_name>` -> `Models`, `Sketches`, `Keep-Out Regions`, `Project Tools`, and `Setups` -> `Fixtures` & `Operations`), paired with crisp, theme-aware procedural vector icons for every node type (project, STEP/mesh/fixture models, sketches & sketch entities, keep-out regions, cutters/tools, setups, workholding fixtures, and all 18 operation strategies).

---

## Problem Diagnosis

1. **Flat Card Stack Instead of a True Project Hierarchy (`crates/chip-gui/src/panels/browser.rs`):**
   - The current left pane renders five separate stacked `section_card` panels (`Models`, `2D Sketches`, `Keep-Out Regions`, `Project Tools`, and per-Setup cards) with inline action buttons crammed into each card—consuming vertical space, separating related objects visually, and lacking a unified Project root node or nested sub-items (such as `Fixtures` under a `Setup` or entities under a `Sketch`).
2. **Incomplete Icon Coverage (`crates/chip-gui/src/widgets/icons.rs:88-113`):**
   - `IconKind` lacks dedicated icons for tree containers (`Project`, `ModelsFolder`, `MeshModel`, `FixtureModel`, `KeepOutZone`, `Setup`, `Fixture`), sketch entity types (`Rect`, `Circle`, `Slot`, `Polygon`, `BoltCircle`, `Polyline`, `Text`), and 3D / 4th-axis strategies (`Adaptive3D`, `Waterline3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`, `Rotary2Linear`, `Rotary4AxisSim` currently reuse 2D or `OrthoCube` icons).

---

## Proposed Changes

### 1. Complete Procedural Vector Icon Suite (`chip-gui`)

#### [MODIFY] `crates/chip-gui/src/widgets/icons.rs`
#### [MODIFY] `crates/chip-gui/src/widgets/mod.rs`
- Add crisp, anti-aliased `egui::Painter` icons to `IconKind`:
  - **Hierarchy & Document Nodes:** `Project`, `ModelsFolder`, `MeshModel` (triangulated wireframe cube), `FixtureModel` (clamp/jaw outline), `KeepOutZone` (slashed shield / keep-out octagon), `Setup` (WCS triad + stock frame), `Fixture` (vise/clamp icon), `ChevronRight`, `ChevronDown`.
  - **Dedicated 3D & 4th-Axis Operation Icons:** `Adaptive3D`, `Waterline3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`, `Rotary2Linear`, `Rotary4AxisSim`, updating `IconKind::for_operation()` so every one of the 18 `OperationKind` variants renders its own unmistakable vector glyph.
  - **Sketch Entity Icons:** `EntityRect`, `EntityCircle`, `EntitySlot`, `EntityPolygon`, `EntityBoltCircle`, `EntityPolyline`, `EntityText`, plus `IconKind::for_sketch_entity(&SketchEntity)` helper.
  - **Cutter/Tool Helper:** `IconKind::for_model_entry(&ModelEntry)` distinguishing STEP solids, triangle meshes (`.stl`/`.obj`/`.3mf`), and imported fixture solids (`ModelRole::Fixture`).

---

### 2. Modular Tree-View Widget & Project Tree Hierarchy

#### [NEW] `crates/chip-gui/src/panels/browser/mod.rs`
#### [NEW] `crates/chip-gui/src/panels/browser/tree_widget.rs`
#### [NEW] `crates/chip-gui/src/panels/browser/actions.rs`
#### [DELETE] `crates/chip-gui/src/panels/browser.rs` (migrated into `panels/browser/` directory)
- Build a reusable, lightweight, zero-allocation-per-idle-frame tree row widget (`tree_widget.rs`) supporting:
  - Indentation levels with subtle theme-matched vertical tree guide lines (`tok.border` at low opacity).
  - Expand/collapse chevrons (`ChevronRight` / `ChevronDown`) persisted per stable node ID in `egui` memory, with sensible defaults (Project root, `Models`, `Sketches`, `Setups`, and active Setup expanded by default; `Keep-Out Regions` and `Project Tools` auto-expanded when non-empty; `Sketch` entity children collapsed by default until expanded).
  - Crisp `14 px` procedural vector icon + single-line truncated **item name** + compact right-aligned badges (status icon, `G54` / `Active` pill, tool + cycle-time `T1 · 42s`, usage count `3 ops`, optional visibility eye / suppress checkbox where applicable).
  - Distinct active-selection highlight (`tok.accent_soft` fill + `tok.accent` left indicator bar) and hover feedback across both `Studio Dark` and `Daylight Light`.
  - Keyboard tree navigation (`Up`/`Down` to step visible rows, `Right`/`Left` to expand/collapse or jump to parent, `F2` to rename, `Del` to delete).
- Render the exact unified Project Tree hierarchy:
  ```text
  [Project] <project.project_name>
   ├── [Models] Models (N)
   │    └── [STEP / Mesh / Fixture] <model display name>
   ├── [Sketches] Sketches (N)
   │    └── [Sketch] <sketch.name>  [eye]
   │         └── [Entity] <entity.name>
   ├── [KeepOut] Keep-Out Regions (N)
   │    └── [KeepOut] <region.name>  [3 ops] [suppress]
   ├── [Tools] Project Tools (N)
   │    └── [Tool] T1 · <tool.name>  [2 ops / unused]
   └── [Setups] Setups (N)
        └── [Setup] <setup.name>  [G54] [Active]
             ├── [Fixture] Fixtures (N)
             │    └── [Fixture] <fixture.name>  [suppress]
             └── [Op] <op.name>  [Status] [T1 · 42s] [eye] [suppress]
  ```
- Keep public helper APIs (`BrowserAction`, `CamBrowserCtx`, `move_operation_to_slot`, `duplicate_operation_in_setup`, `delete_operation_in_setup`, `create_default_operation_in_setup`, `format_compact_cycle_time`) cleanly exported from `panels/browser/mod.rs` so existing callers and tests stay clean.

---

## Verification Plan

1. **Icon & Tree Rendering Tests (`chip-gui`):**
   - Unit test verifying `paint_icon` executes cleanly for every `IconKind` variant without panicking and `IconKind::for_operation` maps all 18 `OperationKind` variants distinctly.
   - Headless `egui` tests verifying the full tree renders with Project as root node, all 5 next-level sections (`Models`, `Sketches`, `Keep-Out Regions`, `Project Tools`, `Setups`), nested `Sketch` entities, nested `Setup -> Fixtures` & `Operations`, and expand/collapse toggles persist cleanly across frames.
2. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check && cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
