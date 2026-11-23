# Milestone 77 Implementation Plan — Active-Setup Indication, Top Action Bar Operation Creation & Separators, and Setup Card Simplification

## Goal

Eliminate duplicate operation-creation buttons across Setup cards and the top action bar by making the **Active Setup** unmistakably visible and selectable on each Setup card, removing per-card operation-add bars from the CAM Browser, and consolidating operation creation into cleanly grouped, separator-divided sections in the top action bar targeting the active setup.

---

## Problem Diagnosis

1. **Redundant Operation-Creation Controls (`crates/chip-gui/src/panels/browser.rs`, `crates/chip-gui/src/app/toolbar.rs`):**
   - Every Setup card in the CAM Browser renders its own 4-button operation creation row (`+ Adap`, `+ Cont`, `+ VCarve`, `+ Op…` dropdown at `browser.rs:648-702`), duplicating the `+ Adaptive` / `+ V-Carve` buttons in the top action bar (`toolbar.rs:106-135`) and the `Operations` menu (`menus.rs`).
   - In multi-setup projects (e.g., Top Op + Flip Op + 4th-Axis Op), this repeats 4+ buttons on every single card and inflates vertical CAM Browser height.
2. **Weak Active-Setup Visual Indicator (`browser.rs:589-605`):**
   - Currently the active setup is denoted only by a tiny `"• "` text prefix on the setup header label. Users looking at the top action bar cannot immediately tell at a glance which setup will receive newly added operations.
3. **Incomplete Top Action Bar Operation Coverage & Grouping (`toolbar.rs`):**
   - The top action bar only exposes `+ Sketch`, `+ Adaptive`, and `+ V-Carve` (missing `+ Contour` and an organized `+ Op ▾` picker for all other 2D/3D/4-axis/undercut strategies), and button groups need clear, consistent visual separators between distinct functional sets.

---

## Proposed Changes

### 1. Clear Active-Setup Indicator & Setup Card Simplification (`panels/browser.rs`)

#### [MODIFY] `crates/chip-gui/src/panels/browser.rs`
- **Remove Per-Setup Operation Creation Row:**
  - Delete the secondary per-setup operation-add strip (`+ Adap`, `+ Cont`, `+ VCarve`, and the 16-strategy `+ Op…` dropdown menu inside each Setup card).
  - Remove now-unused per-setup browser add-operation plumbing if all operation additions route through the active setup (keep `BrowserAction` clean and minimal; double-check context menus if right-click `"Add Operation…"` is desired or if top action bar + menu + Command Palette suffice).
- **Prominent Active-Setup Styling & Selection:**
  - Give the active Setup card (`is_active_setup == (*active_setup_idx == s_idx)`) unmistakable visual emphasis consistent with both Dark and Light workspace themes (`theme.rs`):
    - High-contrast accent border stroke (`1.5 px` accent color) and subtle accent-tinted card header/fill.
    - Explicit **`Active`** pill badge next to the WCS slot (`G54`, `G55`, …) instead of the subtle `"• "` prefix.
  - Make switching the active setup effortless:
    - Clicking anywhere on an inactive Setup card's header bar (or a compact `"Activate"` / `"Select"` affordance on hover/header) immediately sets `*active_setup_idx = s_idx`, selects the setup in the Inspector, and updates the viewport/simulation focus.
    - Clicking any operation inside a setup continues to activate that setup automatically.
- **Compact Setup Card Controls:**
  - Retain only setup-scoped actions on the Setup card header/row: `Stock & WCS`, `Gen` (generate dirty ops for this setup), `Top` (align view to setup WCS), and `Trash` (delete setup when `project.setups.len() > 1`).

---

### 2. Top Action Bar Operation Creation & Logical Button Separators (`app/toolbar.rs`)

#### [MODIFY] `crates/chip-gui/src/app/toolbar.rs`
#### [MODIFY] `crates/chip-gui/src/app/commands.rs`
#### [MODIFY] `crates/chip-gui/src/app/mod.rs`
- **Unified Active-Setup Operation Creation Helper:**
  - Ensure all top-action-bar operation buttons route through a single helper `add_operation_to_active_setup(strategy)` that validates `self.active_setup_idx`, creates the operation with sensible machine/tool defaults, selects it in the Inspector, records an Undo step, and logs which setup received the operation.
  - Display the active setup's name in every operation button's hover tooltip (e.g., `"Add 2D Contour to active setup: Setup 1 — Top Op"`).
- **Cleanly Grouped Top Action Bar with Explicit `ui.separator()` Dividers:**
  Structure the single-row top action bar (`chip_toolbar`) into visually distinct groups separated by `ui.separator()` dividers:
  1. **File & History:** `Open`, `Save`, `.chipz`, `DXF/SVG`, `Reload` (when model loaded), `Undo`, `Redo`
  2. `ui.separator()`
  3. **Geometry & Add Operations (targets Active Setup):**
     - `+ Sketch`
     - `+ Adaptive` (2D Adaptive Pocket)
     - `+ Contour` (2D Contour)
     - `+ V-Carve` (2D V-Carve / Engrave)
     - `+ Op ▾` categorized menu button grouping all remaining strategies clearly into:
       - **2D / 2.5D:** `Facing`, `2D Pocket`, `Drill / Bore`, `Thread Mill`, `2D Chamfer / Trace`, `2D Text Engrave`, `2D Rest Machining`
       - **3D Surface:** `3D Adaptive Rough`, `3D Waterline`, `3D Parallel`, `3D Scallop`, `3D Pencil`, `3D Spiral / Radial`, `3D Undercut / Dovetail`, `3D Edge Deburr`
       - **4th-Axis Rotary:** `Rotary 2-Axis Indexed`, `Rotary 4-Axis Simultaneous`
     - Subtle inline indicator or tooltip feedback when the project has multiple setups so the active setup target is unambiguous.
  4. `ui.separator()`
  5. **CAM Pipeline Execution:** `Generate` / `Cancel`, `Simulate` / `Cancel`, `Post`, and the shared background task `toolbar_progress_bar`
  6. `ui.separator()`
  7. **Tools:** single `Tools` button (further consolidated across the workspace in Milestone 78)
  8. **Right-Aligned Utility Group:** `Ctrl+K` Command Palette `|` `Settings` button (from Milestone 76)
- Ensure horizontal spacing and compact button sizing keep the toolbar single-row at `1280×720` and wider without clipping or awkward wrapping.

---

## Verification Plan

1. **Active-Setup & Operation-Creation Behavior Tests (`chip-gui`):**
   - Unit test verifying `add_operation_to_active_setup` adds operations to `project.setups[active_setup_idx]` (e.g., Setup 2 when Setup 2 is active), updates selection to the newly added operation, and preserves atomic Undo/Redo.
   - Verify deleting the active setup safely clamps `active_setup_idx` to a valid remaining setup.
2. **UI Visual Verification (Headless/Manual Check):**
   - Confirm Setup cards no longer render `+ Adap`, `+ Cont`, `+ VCarve`, or `+ Op…` rows.
   - Confirm the active Setup card displays the accent border + `"Active"` badge cleanly in both Dark and Light themes.
   - Confirm the top action bar displays clear `ui.separator()` dividers between File/History, Geometry/Operations, Pipeline, Tools, and right-side Utility groups, and fits cleanly on a single row at standard desktop width.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
