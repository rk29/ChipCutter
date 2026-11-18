# ChipCutter — Milestone 10 Implementation Plan

## Goal Description
Implement **Milestone 10 (Design System, Dark/Light Themes, Vector Iconography, & Single-Row Workspace Chrome)** from [10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md#L60-L68) and [08-ui-viewport.md](file:///workspace/ChipCutter/specs/08-ui-viewport.md#L3-L39)—the first milestone of **ChipCutter v3 (UI Polish & Usability)**:

1. **Centralized Semantic Design System & Dual Themes (`crates/chip-gui/src/theme.rs`):**
   - Introduce `AppTheme` (`Dark` — **Studio Dark**, `Light` — **Daylight High-Contrast**) and `ThemeTokens` defining semantic colors (`panel_bg`, `card_bg`, `elevated_bg`, `border_subtle`, `accent_primary`, `status_clean`, `status_dirty`, `status_error`, `status_suppressed`, `text_primary`, `text_muted`), spacing/padding scales, corner radii, and typography hierarchy.
   - Replace scattered hardcoded `Color32::from_rgb(...)` literals across `chip-gui` panels and modals with `ThemeTokens` lookups, and synchronize the 3D `wgpu` viewport background studio gradient, WCS reference grid, ViewCube card, and overlay labels with the active `AppTheme`!
2. **Procedural HiDPI Vector Iconography (`crates/chip-gui/src/widgets/icons.rs`):**
   - Build a zero-dependency `egui::Painter` vector icon module rendering crisp anti-aliased strokes and shapes at any HiDPI scale factor (`1.0x`–`3.0x`) for:
     - **Status Badges:** `Clean` (check), `Dirty` (refresh arrows), `Generating` (arc spinner), `Error / Needs Re-Pick` (warning triangle), `Suppressed` (slash circle).
     - **Operation & Sketch Icons:** `Facing`, `Contour2D`, `Pocket2D`, `Adaptive2D`, `DrillBore`, `ChamferTrace`, `VCarve`, `Sketch2D`, `StepModel`, `Tool`.
     - **Action & Transport Icons:** `Open`, `Save`, `Bundle`, `Undo`, `Redo`, `Play`, `Pause`, `StepPrev`, `StepNext`, `Generate`, `Simulate`, `PostGCode`, `Sun`/`Moon` theme toggle, `Eye` visibility, `Trash`, and `Search`.
3. **Decluttered Single-Row Workflow Toolbar & Floating Viewport View HUD (`menus_toolbar.rs`, `viewport/overlays.rs`, `pane_behavior.rs`):**
   - Refactor the top toolbar so it never wraps awkwardly onto a second row: group primary workflow actions cleanly left-to-right (`[Open] [Save] [.chipz] [DXF/SVG] | [+ Sketch] [+ Adaptive] [+ V-Carve] | [Generate (Ctrl+G)] [Simulate] [Post G-Code]`) with a right-aligned `[mm | in]` unit pill and `[Dark | Light]` theme toggle.
   - Relocate camera & layer visibility toggles (`Ortho/Persp (P)`, `Fit (F)`, `Setup Top`, `Mouse Preset`, `Orbit Mode`, `Toolpaths`, `Rapids`, `Stock`, `Ghost`) into a sleek translucent **Floating Viewport View Bar** anchored along the bottom edge of the 3D viewport!

---

## Key Architectural Decisions & Scope Alignment

> [!IMPORTANT]
> **Zero External Icon Font or SVG Rasterization Dependencies:**
> - Drawing icons procedurally with `egui::Painter` (`line_segment`, `circle_stroke`, `convex_polygon`, cubic/arc polylines) in normalized `[0, 1]²` coordinates guarantees pixel-sharp strokes at any DPI, automatic tinting via `ThemeTokens` (enabled, hovered, active, disabled), zero binary bloat, and zero new third-party crates.

> [!TIP]
> **Theme-Coupled 3D `wgpu` Studio Background & Grid Contrast:**
> - Switching to `AppTheme::Light` updates `egui::Style` / `egui::Visuals` **and** passes theme-aware sky/horizon gradient colors + grid line alpha into `ViewportUniformsGpu` (`viewport/gpu_types.rs` & `viewport_shaders.wgsl`) so CAD edges, cyan/amber toolpaths, and sketch curves remain high-contrast in bright shop lighting!

> [!NOTE]
> **Core Engine Crates Remain 100% Untouched:**
> - All Milestone 10 changes live strictly inside `crates/chip-gui` (preserving 100% UI-agnostic core crates and keeping all v1 + v2 golden G-code snapshots byte-for-byte identical).

---

## Proposed Changes

### 1. Semantic Design System & Dual Theme Engine (`crates/chip-gui`)

#### [NEW] `crates/chip-gui/src/theme.rs`
- Define `AppTheme` (`Dark`, `Light` — `Serialize`/`Deserialize` ready for Milestone 12 `prefs.toml` persistence) and `ThemeTokens`:
  - Surface hierarchy: `app_bg`, `panel_bg`, `card_bg`, `hud_bg_translucent`, `input_bg`, `border`, `border_focused`.
  - Brand & semantic accents: `accent`, `accent_hover`, `clean_fg` / `clean_bg`, `dirty_fg` / `dirty_bg`, `error_fg` / `error_bg`, `warn_fg` / `warn_bg`, `suppressed_fg`.
  - 3D Viewport studio palette: `vp_bg_top_rgb`, `vp_bg_bottom_rgb`, `vp_grid_minor_rgba`, `vp_grid_major_rgba`.
- Implement `apply_theme(ctx: &egui::Context, theme: AppTheme)` configuring `egui::Style` spacing (`item_spacing`, `button_padding`, `window_margin`), refined `CornerRadius` (`6px` cards/HUDs, `4px` inputs/buttons), and `Visuals` widget states.

#### [MODIFY] [gpu_types.rs](file:///workspace/ChipCutter/crates/chip-gui/src/viewport/gpu_types.rs), [pipeline.rs](file:///workspace/ChipCutter/crates/chip-gui/src/viewport/pipeline.rs), & `viewport_shaders.wgsl`
- Pass `bg_top_color` and `bg_bottom_color` (plus light/dark grid contrast flag) in `ViewportUniformsGpu` so the `wgpu` studio background and WCS grid adapt seamlessly between **Studio Dark** and **Daylight Light** themes (including offscreen `wgpu` unit test coverage!).

---

### 2. Procedural Vector Iconography & Styled UI Primitives (`crates/chip-gui/src/widgets`)

#### [NEW] `crates/chip-gui/src/widgets/icons.rs` & [MODIFY] [widgets/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/widgets/mod.rs)
- Define `IconKind` enum covering status badges, all 6 `OperationKind`s + `Adaptive2D`, `SketchLayer` entity types, file/project actions, simulation transport buttons, and viewport toggles.
- Implement:
  - `paint_icon(painter: &egui::Painter, rect: egui::Rect, icon: IconKind, color: egui::Color32)`
  - `icon_button(ui: &mut egui::Ui, icon: IconKind, label: Option<&str>, tooltip: &str) -> egui::Response`
  - `icon_toggle_pill(ui: &mut egui::Ui, active: &mut bool, icon: IconKind, label: &str, tooltip: &str) -> bool`
  - `status_badge_pill(ui: &mut egui::Ui, tokens: &ThemeTokens, status: BadgeKind, tooltip: Option<&str>)`
- Replace legacy ASCII badges (`[OK]`, `[*]`, `[!]`, `[?]`, `[-]`, `|<`, `< Seg`, `Seg >`, `>| End`) across [browser.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/browser.rs), [diagnostics.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/diagnostics.rs), and [simulation.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/simulation.rs).

---

### 3. Single-Row Workflow Toolbar & Floating Viewport View HUD (`crates/chip-gui`)

#### [MODIFY] [menus_toolbar.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/menus_toolbar.rs), [overlays.rs](file:///workspace/ChipCutter/crates/chip-gui/src/viewport/overlays.rs), [pane_behavior.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/pane_behavior.rs), [app/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/mod.rs), & [main.rs](file:///workspace/ChipCutter/crates/chip-gui/src/main.rs)
- **Single-Row Top Action Bar (`menus_toolbar.rs`):**
  - Clean single-row `egui::Sides` / horizontal layout with vector-icon buttons:
    - Left/Center workflow clusters: `File/Import` (`Open`, `Save`, `.chipz`, `DXF/SVG`) $\to$ `Author` (`+ Sketch`, `+ Adaptive`, `+ V-Carve`) $\to$ `Execute` (`Generate` primary accent button, `Simulate`, `Post G-Code`, `Tool Library`).
    - Right utility cluster: `[mm | in]` segmented unit switch and `[Dark / Light]` theme toggle (plus `View -> Theme` menu item and `--theme light|dark` CLI flag for headless screenshot testing!).
- **Floating Bottom Viewport HUD (`overlays.rs` & `pane_behavior.rs`):**
  - Translucent rounded HUD bar centered near the bottom of `Viewport3D` containing:
    - Camera controls: `Ortho / Persp (P)`, `Fit (F)`, `Setup Top`, compact `Mouse Preset` & `Turntable / Trackball` selectors.
    - Layer visibility pills: `Toolpaths`, `Rapids`, `Stock`, `Ghost`.
  - Relocate the concise Setup/Coordinate status readout cleanly to the bottom-left corner without colliding with the floating HUD bar or ViewCube.
- Sweep remaining panels (`operation_inspector/*`, `setup_inspector.rs`, `sketch_inspector.rs`, `gcode_preview.rs`, `tool_library.rs`, `vector_import_modal.rs`) to consume `ThemeTokens` instead of hardcoded dark-only `Color32` colors so every panel is legible in both Dark and Light themes.

---

## Verification Plan

1. **Automated Unit & Offscreen GPU Tests (`crates/chip-gui`):**
   - Unit test `ThemeTokens::for_theme(AppTheme::Dark)` and `ThemeTokens::for_theme(AppTheme::Light)` contrast ratios (WCAG AA `>= 4.5:1` for primary text against panel/card/input backgrounds!).
   - Offscreen `wgpu` render test verifying both `AppTheme::Dark` and `AppTheme::Light` viewport studio background & WCS grid rendering without panics or zero-contrast artifacts.
2. **Workspace Regression & Lint Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings` (`0` warnings, `0` `too_many_arguments`)
   - `cargo deny check` (`0` new dependencies)
   - `WGPU_BACKEND=vulkan cargo test --workspace` (all existing 57+ tests & all v1 + v2 golden G-code snapshots passing unchanged).
3. **Headless Dual-Theme Visual Verification (`xvfb-run` + Vulkan Lavapipe):**
   - Capture and visually inspect **both** Dark and Light theme headless screenshots (`--demo-vcarve --self-test-screenshot` and `--demo-vcarve --theme light --self-test-screenshot`) via `read_file` to verify single-row toolbar alignment, crisp vector icons, floating viewport HUD legibility, and Light/Dark contrast across all panes.
