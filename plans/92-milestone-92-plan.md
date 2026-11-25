# Milestone 92 Implementation Plan — Tree-View Visual Verification, Full Screenshot Refresh, & Phase 25 Spec / Tutorial Sync

## Goal

Visually verify the new hierarchical Project Tree View, procedural icon suite, selection-driven right-panel editors (including Keep-Out Region, Project, and Project Tool inspectors), and context menus in both **Studio Dark** and **Daylight Light** themes via headless Vulkan screenshots, polish any spacing/truncation/contrast blemishes, **re-capture all repository screenshots** (main hero, all tutorials, and all 26 Sample Gallery projects) so every image reflects the new Project Tree View, and synchronize `specs/08-ui-viewport.md`, `specs/10-roadmap.md`, and `docs/tutorials/` for Phase 25.

---

## Proposed Changes

### 1. Headless Vulkan Visual Verification & UI Polish

#### [MODIFY] `crates/chip-gui/src/panels/browser/` (as needed for visual polish)
#### [MODIFY] `crates/chip-gui/src/widgets/icons.rs` (as needed for crisp stroke weights)
- Capture headless Vulkan verification screenshots into the session scratch directory (`WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan timeout --foreground 45s xvfb-run -a ...`) covering:
  1. Rich multi-section project (`--sample keepout-clamps`) in **Studio Dark** showing the expanded Project Tree (`Project` -> `Models`, `Sketches` with child entity, `Keep-Out Regions`, `Project Tools`, `Setups` -> `Fixtures` & `Operations`) and right-side `Keep-Out Region` inspector with live coral/red 3D overlays.
  2. Same tree view in **Daylight Light** verifying icon legibility, subtle tree guide lines, selection highlight contrast, and zero clipped labels or missing glyphs.
  3. 4th-axis rotary project (`--sample indexed-model-bounds`) verifying dedicated 3D/rotary operation icons, Setup `Active` indicator, and clean narrow-pane truncation.
- Fix any visual spacing, icon alignment, or truncation issues discovered during screenshot inspection before regenerating repository screenshots.

---

### 2. Full Repository Screenshot Refresh (Hero, All Tutorials, & All 26 Sample Gallery Projects)

#### [MODIFY] `docs/images/chipcutter-gui.png`
#### [MODIFY] `docs/images/tutorials/**`
#### [MODIFY] `docs/images/samples/**`
#### [MODIFY] `scripts/capture_tutorial_screenshots.sh` (if any capture targets need adjustment)
#### [MODIFY] `scripts/capture_sample_gallery.sh` (if any capture targets need adjustment)
- Because the left CAM Browser appears in every full-workspace screenshot across the documentation, deterministically re-capture **all** screenshots using the release binary (`CARGO_TARGET_DIR=/workspace/ChipCutter/target`):
  1. Re-capture all tutorial screenshots + the main README hero (`docs/images/chipcutter-gui.png`) via `./scripts/capture_tutorial_screenshots.sh`.
  2. Re-capture all 26 Sample Gallery projects (hero, 480 px thumbnails, and per-Setup/per-Operation simulation frames) via `./scripts/capture_sample_gallery.sh`.
  3. Visually spot-check refreshed hero, tutorial, and sample PNGs via `read_file` to confirm crisp Project Tree rendering, zero clipped labels, and zero visual regressions.

---

### 3. Phase 25 Specification & Tutorial Text Sync

#### [MODIFY] `specs/08-ui-viewport.md`
#### [MODIFY] `specs/10-roadmap.md`
#### [MODIFY] `docs/tutorials/*.md` (where instructions reference old left-panel cards)
- Update `specs/08-ui-viewport.md` to document:
  - The hierarchical **Project Tree View** (`Project` root -> `Models`, `Sketches` + entities, `Keep-Out Regions`, `Project Tools`, `Setups` -> `Fixtures` & `Operations`).
  - Complete procedural icon coverage (`IconKind` for containers, models, sketch entities, cutters, fixtures, and all 18 operation strategies).
  - Unified right-click context menus (common + type-safe sub-item creation actions), inline `F2` / menu renaming, section-scoped drag-and-drop reordering, and all `InspectorFocus` right-panel editors (`Project`, `Model`, `Sketch`, `KeepOutRegion`, `ProjectTool`, `Setup`/`Fixture`, `Operation`).
- Audit and update step-by-step instructions in `docs/tutorials/` wherever they reference removed card-stack left-panel buttons (e.g., `"Stock & WCS"`, `"+ New Sketch"` card buttons, `"Orient / Scale"` under `Models` card) so they match the new Project Tree View and right-click context menus accurately.
- Mark **Phase 25 (Completed — M90–M92)** in `specs/10-roadmap.md`.

---

## Verification Plan

1. **Headless Visual, Screenshot, & Parity Verification:**
   - Visually inspect captured Studio Dark & Daylight Light verification PNGs plus spot-checked refreshed hero/tutorial/sample PNGs via `read_file`.
   - Confirm `git status` shows updated PNGs across `docs/images/chipcutter-gui.png`, `docs/images/tutorials/`, and `docs/images/samples/` with zero missing/corrupt files.
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-gui --test docs_samples_parity`
2. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check && cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
