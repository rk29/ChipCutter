# ChipCutter — Milestone 11 Implementation Plan

## Goal Description
Implement **Milestone 11 (CAM Browser Ergonomics, Two-Column Inspector Grids, & Expression UX)** from [10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md#L69-L73) and [08-ui-viewport.md](file:///workspace/ChipCutter/specs/08-ui-viewport.md#L30-L39)—the second milestone of **ChipCutter v3 (UI Polish & Usability)**:

1. **Refined CAM Browser Tree & Operation Ergonomics (`crates/chip-gui/src/panels/browser.rs`, `menus_toolbar.rs`):**
   - Redesign CAM Browser operation rows so full operation names remain readable even in narrow left panes: display `[StatusIcon] [StrategyIcon] <idx>. <Name>` on the left paired with a right-aligned `T# · <est_time>` metadata pill (`T1 · 1m12s` or `T1+T8 · 2m04s` for Two-Stage V-Carve) when idle, and reveal compact quick-action buttons (`Duplicate`, `Suppress/Enable`, `Delete`) on row hover or selection.
   - Add pointer **drag-and-drop operation reordering** within a Setup (with a crisp `ThemeTokens::accent` drop-target insertion line and automatic Rest Machining dirty propagation) alongside keyboard shortcuts **`Ctrl+Up` / `Ctrl+Down`** (reorder), **`Ctrl+D`** (duplicate), and **`Delete`** (remove operation or selected sketch entity) when no text field holds focus.
   - Streamline the per-Setup operation creation bar into primary quick-create pills (`+ Adaptive`, `+ Contour`, `+ V-Carve`) plus a compact `+ More ▾` menu so Setup headers never wrap into multi-row button walls.
2. **Standardized Two-Column Inspector Grids & Modular Section Cards (`crates/chip-gui/src/widgets/inspector.rs` & `panels/*`):**
   - Introduce reusable design-system layout primitives (`section_card`, `inspector_grid`, `grid_row`, and `inline_banner`) backed by `ThemeTokens` so parameter labels left-align cleanly in Column 1 (with machinist hover tooltips) and input controls align along a uniform vertical axis in Column 2.
   - Modularize large inspector files (`passes_tab.rs` and `sketch_inspector.rs`) into focused submodules (`< 650` lines each) while upgrading **Operation Inspector** (`Tool`, `Geometry`, `Heights`, `Passes`, `Linking`), **Setup Inspector** (`WCS`, `Stock`, `Post`, `Heights`), and **Sketch Inspector** to the two-column card layout.
3. **Upgraded Parametric Expression Input Widget (`crates/chip-gui/src/widgets/expr_input.rs`):**
   - Enhance `edit_param_length` & `ExpressionInput` with:
     - Clean **unit suffix pill** (`mm` / `in`) on plain numeric literals (avoiding redundant `3  = 3.000 mm` duplication) vs. a live **`= <resolved>` badge** whenever a formula or mixed unit (`tool_d * 0.4`, `1/4 in`) is active.
     - **Scoped Variable Reference & Autocomplete Popover (`fx` button + identifier prefix suggestions):** Lists active `EvalContext` variables (`tool_d`, `tool_r`, `flute_len`, `flutes`, `tip_angle`, `stock_x`, `stock_y`, `stock_z`, `model_top_z`, `model_bottom_z`, `feature_top_z`, `feature_bottom_z`, `hole_d`) with descriptions and live values in `ctx.default_unit`, supporting 1-click or `Tab` insertion.
     - **Keyboard `ArrowUp` / `ArrowDown` Nudging** (`0.1` / `Shift: 1.0` / `Alt: 0.01` display-unit steps) and **horizontal drag-scrubbing** on the unit pill for plain numeric values, with coalesced Undo snapshots.

---

## Key Architectural Decisions & Scope Guardrails

> [!IMPORTANT]
> **Core Crates Remain 100% Untouched:**
> - All Milestone 11 changes live strictly inside `crates/chip-gui`. `EvalContext::get_var` and `EvalContext::get_scalar_var` in `chip-math` already expose every scoped CAM variable (`tool_d`, `stock_x`, `feature_top_z`, `hole_d`, etc.), so variable discovery and live value formatting require zero changes to `chip-math`, `chip-cam`, `chip-sim`, `chip-post`, or `chip-project`, keeping all 8 v1 + v2 golden G-code files byte-for-byte identical.

> [!TIP]
> **Deterministic Dirty Propagation on Drag-and-Drop / `Ctrl+Up` / `Ctrl+Down` Reordering:**
> - Reordering operations in a Setup can change upstream stock coverage for operations using `RestMachiningMode::FromPrecedingOperations`. Whenever an operation's index changes via drag-and-drop or keyboard shortcut, we call `chip_project::invalidate_dirty_operations` (and mark affected `FromPrecedingOperations` operations dirty) after recording a single Undo snapshot, keeping Rest Machining DAG waves 100% sound.

> [!NOTE]
> **Submodule Decomposition for Maintainability:**
> - Before adding two-column grids and section cards, we split `operation_inspector/passes_tab.rs` (`1,196` lines) and `sketch_inspector.rs` (`1,297` lines) into focused submodules so every file remains concise, readable, and free of `#[allow(clippy::too_many_arguments)]`.

---

## Proposed Changes

### 1. Inspector Layout Primitives & Upgraded Expression Widget (`crates/chip-gui/src/widgets`)

#### [NEW] `crates/chip-gui/src/widgets/inspector.rs` & [MODIFY] [widgets/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/widgets/mod.rs)
- Implement `ThemeTokens`-driven inspector building blocks:
  - `section_card(ui, id_salt, title, default_open, add_body)`: Rounded `tok.card_bg` container (`1px` `tok.border`, `6px` corner radius) with a clean collapsible header for grouping logical parameter sections.
  - `inspector_grid(ui, id_salt, add_rows)` & `grid_row(ui, label, tooltip, add_control)`: Standardized two-column `egui::Grid` (`min_col_width: 118.0`, `spacing: [10.0, 6.0]`) aligning labels on the left and inputs on the right.
  - `BannerKind` (`Clean`, `Info`, `Warn`, `Error`) + `inline_banner(ui, kind, text)`: Consistent bordered semantic status callout box with procedural vector icon.
  - `copy_icon` (`IconKind::Duplicate`) added to [icons.rs](file:///workspace/ChipCutter/crates/chip-gui/src/widgets/icons.rs) for operation duplication buttons.

#### [MODIFY] [expr_input.rs](file:///workspace/ChipCutter/crates/chip-gui/src/widgets/expr_input.rs)
- Define `ScopedVarInfo { name, description, formatted_value }` and `collect_scoped_vars(ctx: &EvalContext) -> Vec<ScopedVarInfo>` inspecting active length and scalar variables in `ctx`.
- Upgrade `edit_param_length` (and `ExpressionInput`) with:
  - Uniform `TextEdit` width + inline `fx` variable-helper toggle button.
  - Smart right-hand suffix pill:
    - Plain numeric literal (`param.expr.is_none()`): draggable `[mm]` / `[in]` unit pill (horizontal drag scrubs the value smoothly).
    - Formula or explicit unit (`param.expr.is_some()`): `= <formatted_length>` live preview pill (or semantic `Error` icon with hover diagnostic on invalid syntax while typing).
  - `ArrowUp` / `ArrowDown` step nudging when focused on a plain numeric value (`base = 0.1 mm` / `0.005 in`, `Shift = 10x`, `Alt = 0.1x`).
  - Floating variable reference / prefix-filtered autocomplete popup (`egui::Area` / `Popup`) when `fx` is toggled or an identifier prefix (`>= 2` alphabetic chars matching a scoped variable) is typed, inserting the chosen variable token on click or `Tab`.
- Add unit tests in `expr_input.rs` covering `collect_scoped_vars`, identifier prefix extraction/insertion, and `ArrowUp`/`ArrowDown` display-unit step increments in both `Millimeter` and `Inch` modes.

---

### 2. CAM Browser Ergonomics, Drag Reordering, & Keyboard Shortcuts (`crates/chip-gui`)

#### [MODIFY] [browser.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/browser.rs) & [menus_toolbar.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/menus_toolbar.rs)
- **Single-Line Operation Row Widget (`browser.rs`):**
  - Allocate a full-width interactive row (`height = 24.0`, `Sense::click_and_drag()`) painting selection/hover card states from `ThemeTokens`.
  - Left side: `[StatusBadge / Spinner]` + `[Operation Vector Icon]` + truncated `<idx>. <Name>` (with subtle `Adp` / `Rest` tag when applicable).
  - Right side (at rest): compact `T#` + estimated cycle-time pill (`T1 · 42s`, or `T1+T8 · 1m18s` when `clearing_tool_id` is set on `VCarve`) in `tok.text_muted`.
  - Right side (on hover): compact vector icon buttons for `Duplicate (Ctrl+D)`, `Suppress/Enable`, and `Delete (Del)`.
- **Drag-and-Drop Reordering (`browser.rs`):**
  - Track dragged `(setup_idx, op_idx)` in `egui` temporary state; while dragging over sibling operations in the same Setup, compute insertion slot `0..=op_len`, paint a `2px` `tok.accent` horizontal drop-indicator line with endpoint dots, and on pointer release move the operation to the target index, record `"Reorder Operation"` in `UndoHistory`, and refresh Rest Machining dirty flags.
- **Keyboard Shortcuts & Menu Entries (`menus_toolbar.rs` & `app/mod.rs`):**
  - Extract shared helper methods on `ChipCutterApp`: `duplicate_focused_operation()`, `move_focused_operation(delta: isize)`, and `delete_focused_item()` (handling focused `Operation` or focused `SketchEntity` when `!ctx.egui_wants_keyboard_input()`).
  - Bind `Ctrl+D`, `Ctrl+ArrowUp`, `Ctrl+ArrowDown`, and `Delete` in `handle_global_shortcuts`, and surface matching items with shortcut hints in the `Edit` and `Operations` menus.
- **Compact `+ Operation` Bar (`browser.rs`):**
  - Replace the 7-button wrapped row inside each Setup with primary quick buttons (`+ Adaptive`, `+ Contour`, `+ V-Carve`) + a `+ More ▾` `menu_button` (`Facing`, `2D Pocket`, `Drill / Bore`, `Chamfer / Trace`).

---

### 3. Two-Column Inspector Grid & Section Card Rollout (`crates/chip-gui/src/panels`)

#### [MODIFY / SPLIT] `crates/chip-gui/src/panels/operation_inspector/*`
- Split [passes_tab.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/passes_tab.rs) into:
  - `passes_tab/mod.rs` (shared `Depth Passes & Stock to Leave` card, `2.5D Rest Machining` card, and strategy dispatch)
  - `passes_tab/milling_2d.rs` (`Facing`, `Contour2D` + Holding Tabs card, `Pocket2D` + `2D Adaptive` card, `DrillBore` card, `ChamferTrace` card)
  - `passes_tab/vcarve.rs` (`V-Carve Strategy`, `Two-Stage Flat Area Clearance`, and `Mated Male V-Inlay Plug` cards)
- Upgrade [tool_tab.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/tool_tab.rs), [geometry_tab.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/geometry_tab.rs), [heights_tab.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/heights_tab.rs), [linking_tab.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/linking_tab.rs), and [operation_inspector/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/mod.rs) to use `section_card`, `inspector_grid`, `grid_row`, and `inline_banner`.

#### [MODIFY / SPLIT] [setup_inspector.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/setup_inspector.rs) & `sketch_inspector/*`
- Split [sketch_inspector.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/sketch_inspector.rs) into `sketch_inspector/mod.rs` (layer header, anchor card, authoring toolbar, entity list) and `sketch_inspector/entity_editors.rs` (two-column parametric entity & TrueType/Hershey text cards).
- Upgrade [setup_inspector.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/setup_inspector.rs) (`WCS`, `Stock`, `Post`, `Heights` tabs) to aligned two-column `inspector_grid` rows inside `section_card`s.

---

## Verification Plan

1. **Automated Unit Tests (`crates/chip-gui`):**
   - `expr_input::tests`:
     - Verify `collect_scoped_vars` discovers tool, stock, model, and feature variables in deterministic order and formats lengths/angles accurately in both `mm` and `in`.
     - Verify identifier prefix extraction & token replacement (`"tool_d * 0.5 + st"` $\to$ `"tool_d * 0.5 + stock_z"`).
     - Verify numeric step-nudging math (`Normal`, `Shift`, `Alt` modifiers in `mm` and `in`) and Undo-safe `ParamLength` updates.
   - `browser::tests`:
     - Verify operation reorder helper (`move_operation_to_slot`) preserves operation IDs, updates focused operation index accurately across upward and downward moves, and marks `FromPrecedingOperations` Rest Machining operations `Dirty` when preceding coverage order changes.
2. **Workspace Lint, License, & Regression Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings` (`0` warnings, `0` `too_many_arguments`)
   - `cargo deny check` (`0` new dependencies)
   - `WGPU_BACKEND=vulkan cargo test --workspace` (all existing 59+ tests & all 8 v1 + v2 golden G-code snapshots passing 100% unchanged).
3. **Headless Vulkan Visual Verification (`xvfb-run` + Lavapipe in Dark & Light Themes):**
   - Capture and visually inspect headless screenshots for both `--demo-vcarve` and the default 3D STEP `CAM Torture Block` in **Studio Dark** and **Daylight Light** via `read_file`, verifying:
     - Un-truncated operation names + crisp `T# · <time>` badges in the CAM Browser;
     - Clean vertical column alignment and card hierarchy in the right-hand Inspector;
     - Clean `[mm]` unit pills vs. `= 3.000 mm` formula badges in expression inputs.
