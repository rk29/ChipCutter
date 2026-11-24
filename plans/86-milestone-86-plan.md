# Milestone 86 Implementation Plan — Avoid / Keep-Out `[Geometry]` UI, Shared Region Manager, Red-Tinted 3D Overlays, Visual Verification, & Phase 23 Spec Sync

## Goal

Expose Milestone 85's **Per-Operation & Shared Avoid / Keep-Out Regions** intuitively inside the Operation Inspector **`[Geometry]`** tab and CAM Browser, render high-contrast **coral/red 3D viewport keep-out overlays** alongside the cyan/amber cutting-area previews, visually verify both M84 (auto-rotated `Indexed 3+1` + `Model Bounds`) and M85–M86 (shared/local fixture & sketch keep-out trimming) via headless Vulkan screenshots, and synchronize specifications and tutorials for Phase 23.

---

## Problem Diagnosis

1. **No UI to Author or Inspect Keep-Out Regions:**
   - After M85 lands the `Operation::avoid: AvoidRegionConfig` kernel, users need an unmistakable, low-friction way in `[Geometry]` to toggle fixture exclusion (`Avoid All Setup Fixtures` or individual clamps), switch viewport picking between **Cutting Geometry** and **Avoid / Keep-Out**, quick-add 2D sketch keep-out loops, adjust radial/axial clearances, and see/remove active keep-out entries.
2. **No Visual Distinction Between Inclusion & Exclusion Zones in the 3D Viewport:**
   - Without a distinct red-tinted keep-out overlay in `BoundaryPreviewOverlay` and `gizmos.rs`, users cannot confirm visually whether a clamp or keep-out rectangle has enough radial clearance before generating toolpaths.

---

## Proposed Changes

### 1. `Avoid / Keep-Out Regions` Card in `[Geometry]` & Shared Keep-Out Management

#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/geometry_tab.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/mod.rs`
#### [MODIFY] `crates/chip-gui/src/panels/cam_browser.rs`
#### [MODIFY] `crates/chip-gui/src/panels/mod.rs`
#### [MODIFY] `crates/chip-gui/src/app/viewport_controller.rs`
- Add a collapsible **`Avoid / Keep-Out Regions`** `section_card` in `[Geometry]` (auto-expanded whenever `!op.avoid.is_empty()`):
  - **Shared Keep-Out Regions (`ProjectDocument::avoid_regions`):**
    - Display toggleable chips for every named `SharedAvoidRegion` in the project (`☑ Clamp Zone A (2)`, `☐ Bearing Boss (1)`, dimmed if suppressed) so any operation can reuse an existing keep-out region in **1 click** without re-picking!
    - Provide inline actions on shared regions:
      - **`Save Local as Shared…`** (when local `op.avoid.selections` or `fixture_ids` are present): promotes the operation's local keep-out items into a named `SharedAvoidRegion` and links the operation to it in one atomic Undo step.
      - **`+ New Shared`** to create a named shared keep-out region directly.
      - **`Edit`** on a linked shared region chip: routes viewport picking / sketch quick-add directly into that shared region with a clear banner (`"Editing Shared Region: Clamp Zone A — linked to 3 operations"`) and optional clearance override inputs.
      - **`Detach to Local`** on a linked shared region chip: copies its selections/fixtures into `op.avoid` and unlinks the shared ID so one operation can diverge without modifying the shared region.
      - **`Apply to Setup Ops…`** quick popover to link/unlink a shared region across multiple operations in the current Setup at once.
  - **Fixture Keep-Outs:**
    - `Avoid All Setup Fixtures` checkbox (`op.avoid.avoid_all_fixtures`) with count badge (e.g., `(3 fixtures in Setup)`).
    - When `avoid_all_fixtures == false` and the Setup has fixtures, display toggleable per-fixture chips (`Vise Jaw`, `Toe Clamp 1`, …) toggling membership in `op.avoid.fixture_ids`.
  - **Geometry & Sketch Keep-Outs:**
    - Segmented pick-target toggle (**Pick Target:** `Cutting Area` | `Avoid / Keep-Out`) right above/inside the pick controls so clicking 3D faces, closed loops, or holes routes directly into `op.avoid.selections` (or the actively edited `SharedAvoidRegion`) when `Avoid / Keep-Out` is active (with a subtle amber/red viewport pick-mode indicator!).
    - Compact `+ Avoid` quick-add buttons for 2D Sketch layers/entities directly inside the Avoid card.
    - Removable list of active local `op.avoid.selections` with a `Clear Local Avoid` button.
  - **Clearances:**
    - `Radial Clearance` (`op.avoid.radial_clearance`) and `Axial Clearance` (`op.avoid.axial_clearance`, shown when fixtures or rotary ops are active) `ParamLength` expression inputs.
- Add a collapsible **`Keep-Out Regions (N)`** section in the **CAM Browser** (`cam_browser.rs`, shown when `!project.avoid_regions.is_empty()` or via `+ Keep-Out`) displaying each shared region's name, usage count (`used by 3 ops`), rename, suppress toggle, duplicate, and safe delete (pruning stale `shared_region_ids` across all operations in one atomic Undo step).
- Ensure every mutation sets `mutated_label`, pushing an atomic Undo snapshot and marking all affected operations (and downstream Rest ops) `Dirty`.

---

### 2. Red-Tinted 3D Viewport Keep-Out Overlays

#### [MODIFY] `crates/chip-gui/src/viewport/boundary_preview.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/gizmos.rs`
#### [MODIFY] `crates/chip-gui/src/app/viewport_controller.rs`
- Extend `BoundaryPreviewInput` with resolved Setup fixtures (`&[ResolvedFixtureSolid]`) and `&[SharedAvoidRegion]`, and hash `op.avoid` + all linked `SharedAvoidRegion` entries + active fixture geometry signatures inside `compute_boundary_preview_signature` (zero per-frame recomputation on idle frames!).
- Extend `BoundaryPreviewOverlay`:
  - `Planar2D` & `StockOuterEnvelope`: carry `avoid_nominal_region: RegionSet2D` and `avoid_effective_region: RegionSet2D`.
  - `RotarySector`: carry `avoid_axial_bands: Vec<(f64, f64)>`.
- Render keep-out regions in `emit_boundary_preview_visuals()` (`viewport/gizmos.rs`) whenever `[Geometry]` is active:
  - **2.5D & 3-Axis:** Crisp coral/red nominal keep-out outline + translucent red top-plane fill and subtle dashed outer clearance ring (`avoid_effective_region`) showing the exact `tool_r + radial_clearance` exclusion zone.
  - **4th-Axis Rotary:** Translucent coral/red axial keep-out sleeve bands spanning each excluded `[u_lo, u_hi]` interval along the workpiece cylinder.
- Ensure high contrast and legibility in both `Studio Dark` and `Daylight Light` themes.

---

### 3. Headless Vulkan Visual Verification & Phase 23 Spec / Docs Sync

#### [MODIFY] `specs/05-operations.md`
#### [MODIFY] `specs/08-ui-viewport.md`
#### [MODIFY] `specs/10-roadmap.md`
#### [MODIFY] `docs/tutorials/08-3d-adaptive-waterline.md`
#### [MODIFY] `docs/tutorials/11-indexed-3plus1-milling.md`
#### [MODIFY] `docs/tutorials/19-workholding-flip-jigs-soft-jaws.md`
- Capture and visually inspect headless Vulkan screenshots (`WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan timeout --foreground 45s xvfb-run -a ...` into scratch dir):
  1. `Indexed 3+1` operation at `90°` auto-rotated upright under `+Z` with **`Model Bounds`** (`ModelBoundingBox`) cyan/amber 3D boundary overlay.
  2. Workholding fixture + sketch **Avoid / Keep-Out Regions** rendered in red on `[Geometry]` with trimmed toolpaths cleanly avoiding the clamp.
- Update `specs/05-operations.md`, `specs/08-ui-viewport.md`, relevant tutorials, and mark **Phase 23 (Completed — M84–M86)** in `specs/10-roadmap.md`.

---

## Verification Plan

1. **GUI & Cache Regression Tests (`chip-gui`):**
   - Headless `egui` tests verifying `[Geometry]` renders `Avoid / Keep-Out Regions`, promotes local keep-out selections via `Save Local as Shared…`, links/unlinks shared keep-out chips across multiple operations, detaches to local, toggles `avoid_all_fixtures` and individual fixture chips, edits `radial_clearance`/`axial_clearance`, records Undo steps, and switches viewport pick target between `Cutting Area` and `Avoid / Keep-Out`.
   - `BoundaryPreviewCache` tests verifying `avoid_nominal_region` / `avoid_effective_region` update immediately when `op.avoid`, a linked `SharedAvoidRegion`, or fixture positions change and perform zero recomputations on idle frames.
2. **Docs & Samples Parity:**
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-gui --test docs_samples_parity`
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
