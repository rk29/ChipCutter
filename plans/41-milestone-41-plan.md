# Milestone 41 Implementation Plan — Unified Collapsible Cards, Strict Two-Column Inspectors, & Consistent Visibility/Active Controls

## Goal

Bring the left **CAM Browser** up to the same elevated collapsible-card polish as the right **Inspector**, tighten wide right-panel cards (such as **WCS Origin**) into a clean two-column `Name | Value` layout, and make **Eyeball = Visibility** and **Checkbox = Active / Enabled** 100% consistent across both Operations and 2D Sketches.

---

## Proposed Changes

### 1. Unified Collapsible Card Styling on Left & Right Panels (`chip-gui`)

#### [MODIFY] [crates/chip-gui/src/panels/browser.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/browser.rs)
#### [MODIFY] [crates/chip-gui/src/widgets/section_card.rs](file:///workspace/ChipCutter/crates/chip-gui/src/widgets/section_card.rs)
- Refactor the left **CAM Browser** top-level sections (**`Models`**, **`2D Sketches`**, **`Project Tools`**, and each **`Setup`** container) to use the shared collapsible `section_card` widget (matching the right Inspector's card surface, border stroke, corner radius, header typography, and collapse chevron).
- Ensure card spacing, header action buttons (e.g., `+ Setup`, `Tools`, `Gen`, `Top`), and nested operation lists remain compact and crisp in both **Studio Dark** and **Daylight Light** themes.

---

### 2. Strict Two-Column `Name | Value` Inspector Layout (`chip-gui`)

#### [MODIFY] [crates/chip-gui/src/panels/setup_inspector.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/setup_inspector.rs)
#### [MODIFY] [crates/chip-gui/src/panels/operation_inspector.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector.rs)
#### [MODIFY] [crates/chip-gui/src/panels/model_inspector.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/model_inspector.rs)
#### [MODIFY] [crates/chip-gui/src/panels/sketch_inspector.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/sketch_inspector.rs)
- Audit every card in the right-hand Inspector panes for horizontal overflow or side-by-side pairs of unrelated controls.
- Refactor **WCS Origin** (and related **WCS Orientation**, **Stock**, **Rotary**, **Operation Passes / Linking / Heights**, and **Model Transform** cards) to follow a strict, consistent two-column `Name | Value / Selection` `inspector_grid` layout:
  - Replace multi-pair horizontal rows (two `label + input` pairs on one line) with clean individual two-column rows (`X Anchor`, `Y Anchor`, `Z Anchor`, `Origin Mode`, etc.).
  - Keep action buttons cleanly wrapped or grouped in dedicated rows so the right panel never widens unexpectedly or clips controls at standard pane widths (`~320–360 px`).

---

### 3. Consistent Eyeball (`Visibility`) vs. Checkbox (`Active / Enabled`) Semantics (`chip-gui`)

#### [MODIFY] [crates/chip-gui/src/panels/browser.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/browser.rs)
#### [MODIFY] [crates/chip-gui/src/app/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/mod.rs)
#### [MODIFY] [crates/chip-gui/src/app/viewport_controller.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/viewport_controller.rs)
- **Operations list:**
  - Replace the eyeball "suppress" toggle with a standard **Checkbox** (`checked = Active`, `unchecked = Suppressed/Inactive`) wired to `!op.suppressed` (with full Undo/Redo support).
  - Use the **Eyeball icon** (`IconKind::Eye` / `IconKind::EyeOff`) strictly for **viewport toolpath visibility** (`hidden_operation_ids: HashSet<String>` in GUI viewport state so hiding an operation's toolpath in the 3D viewport does **not** suppress it from simulation or G-code export!).
- **2D Sketches list:**
  - Replace the sketch visibility checkbox with the **Eyeball icon** (`IconKind::Eye` / `IconKind::EyeOff`) bound to `sketch.visible`.
  - Keep active/inactive semantics clearly aligned with checkboxes across the entire browser tree.

---

## Verification Plan

1. **Unit Tests:**
   - Update/add browser tests in `panels/browser.rs` verifying checkbox active/suppress toggling (with Rest Machining dirty propagation & Undo/Redo) and independent eyeball toolpath/sketch visibility toggling.
2. **Headless Visual Inspection:**
   - Capture headless Vulkan screenshots of the full workspace (Setup WCS tab, Operation Passes tab, and Pure-2D Sketches browser) in both Dark and Light themes and inspect via `read_file` to verify identical collapsible card styling on left & right panels, zero wide-card clipping, and clean checkbox/eyeball alignment.
3. **Workspace Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-gui`
