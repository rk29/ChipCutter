# ChipCutter — Milestone 12 Implementation Plan

## Goal Description
Implement **Milestone 12 (Interactive Viewport Guidance HUD, Welcome & Sample Gallery, Command Palette, Shortcut Cheat-Sheet, & Persistent Preferences)** from [10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md#L74-L78) and [08-ui-viewport.md](file:///workspace/ChipCutter/specs/08-ui-viewport.md#L87-L93)—completing **ChipCutter v3 (UI Polish & Usability)**:

1. **Active Pick & Sketch Guidance HUD (`crates/chip-gui/src/viewport/overlays.rs`, `app/pane_behavior.rs`):**
   - Replace the plain text viewport pick line with a structured, translucent **Top-Center Floating Guidance HUD** whenever a 3D B-Rep pick mode, manual holding-tab placement, or 2D sketch authoring tool is active.
   - Display mode icon + title, live selection/entity/tab counter pill, concise next-click instructions, and inline **`[Done]`** / **`[Cancel (Esc)]`** buttons right inside the viewport banner.
2. **Welcome Quick-Start Overlay & Sample Projects Gallery (`crates/chip-gui/src/panels/welcome_gallery.rs`):**
   - Present a clean **Welcome / Quick-Start Overlay** on bare `chip-gui` launch (`Open Project / .chipz`, `Import 3D STEP`, `Import 2D DXF / SVG`, `New Pure-2D Sketch`, `Recent Projects`, and `"Show Welcome on startup"` toggle).
   - Move all built-in demo jobs out of the `Models` browser tree and `File` menu into a dedicated, filterable **Sample Projects Gallery** modal (`All`, `3D STEP`, `Pure-2D & Sketches`, `V-Carve & Typography`) with rich descriptions, strategy/tool tags, and 1-click loading.
3. **`Ctrl+K` Command Palette, `?` / `F1` Shortcut Cheat-Sheet, & `prefs.toml` Persistence (`crates/chip-gui`):**
   - Add a fuzzy-searchable **Command Palette (`Ctrl+K`)** indexing all file, import, sketch, operation, generation/simulation/post, camera/view, theme/unit, and sample-project actions with shortcut hints and keyboard navigation.
   - Add a categorized **Keyboard Shortcut Cheat-Sheet (`?` / `F1`)** modal and a `Help` menu (`Welcome & Quick-Start`, `Sample Projects Gallery…`, `Command Palette (Ctrl+K)`, `Keyboard Shortcuts (? / F1)`).
   - Persist **`prefs.toml`** in the OS config directory (`~/.config/chipcutter/prefs.toml` alongside `default.tools.toml`) storing `AppTheme`, default display units, mouse navigation preset, orbit mode, camera projection, viewport visibility toggles, `egui_tiles` workspace pane shares, `show_welcome_on_startup`, and recent project paths (`File -> Open Recent`).
   - Refresh `README.md`, `ARCHITECTURE.md`, and `DEVELOPMENT.md` for **ChipCutter v3** completion.

---

## Key Architectural Decisions & Scope Guardrails

> [!IMPORTANT]
> **Zero New External Crate Dependencies & Core Crates 100% Untouched:**
> - `dirs`, `toml`, and `serde` are already workspace dependencies in root [Cargo.toml](file:///workspace/ChipCutter/Cargo.toml#L40-L45); linking `dirs.workspace = true` and `toml.workspace = true` in `crates/chip-gui/Cargo.toml` adds zero new crates (`cargo deny check` unchanged), and all core geometry, CAM, simulation, post-processing, and project crates stay 100% untouched.

> [!TIP]
> **Deterministic Headless Self-Test Isolation:**
> - When `--self-test-screenshot` is active, `ChipCutterApp` skips reading/writing the host user's `~/.config/chipcutter/prefs.toml` and keeps loading the seeded `CAM Torture Block` by default unless `--welcome` (or `--demo-*`) is explicitly passed on the CLI—preserving deterministic CI screenshots while allowing headless visual verification of the new Welcome Overlay and Guidance HUD!

> [!NOTE]
> **Safe `egui_tiles` Share Persistence:**
> - Rather than serializing opaque internal `egui_tiles::TileId` arenas (which could break if panes are added in v4), `WorkspaceLayoutShares` stores normalized clamped ratios (`browser_share`, `viewport_share`, `inspector_share`, `top_row_share` in `[0.12, 0.78]`) applied to `create_workspace_tree_with_shares`, with 1-click recovery via `View -> Reset Default Layout`.

---

## Proposed Changes

### 1. Persistent User Preferences & Workspace Layout Shares (`crates/chip-gui`)

#### [NEW] `crates/chip-gui/src/prefs.rs`, [MODIFY] [Cargo.toml](file:///workspace/ChipCutter/crates/chip-gui/Cargo.toml), [layout.rs](file:///workspace/ChipCutter/crates/chip-gui/src/layout.rs), & `camera.rs`
- Link `dirs.workspace = true` and `toml.workspace = true` in `crates/chip-gui/Cargo.toml`; add `Serialize`/`Deserialize` derives to `MousePreset`, `OrbitMode`, and `ProjectionMode` in `camera.rs`.
- Implement `UserPreferences` (`schema_version = 1`, `#[serde(default)]`):
  - `theme: AppTheme`, `default_units: DisplayUnit`, `mouse_preset: MousePreset`, `orbit_mode: OrbitMode`, `projection: ProjectionMode`
  - `show_toolpaths: bool`, `show_rapids: bool`, `show_stock: bool`, `ghost_part: bool`
  - `layout_shares: WorkspaceLayoutShares` (`extract_workspace_shares(&Tree<WorkspacePane>)` & `create_workspace_tree_with_shares(&WorkspaceLayoutShares)` in `layout.rs`)
  - `show_welcome_on_startup: bool`
  - `recent_files: Vec<PathBuf>` (`push_recent_file` deduplicating canonical/normalized paths and capping at `10` MRU entries)
- Implement `default_prefs_path()`, `load_or_default_prefs()`, and atomic/clean `save_prefs(&UserPreferences)` (auto-saved on dirty state change when `self_test.is_none()`).

---

### 2. Active Pick & Sketch Viewport Guidance HUD (`crates/chip-gui/src/viewport`)

#### [MODIFY] [overlays.rs](file:///workspace/ChipCutter/crates/chip-gui/src/viewport/overlays.rs), [panels/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/mod.rs), [pane_behavior.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/pane_behavior.rs), & [icons.rs](file:///workspace/ChipCutter/crates/chip-gui/src/widgets/icons.rs)
- Add `IconKind::{Search, Sparkles, Keyboard, Crosshair}` to `widgets/icons.rs`.
- Add structured metadata helpers on `ViewportPickMode` (`guidance_title()`, `guidance_instruction()`, `supports_done_button()`).
- Implement `GuidanceHudAction` (`Done`, `ResetChainStart`, `Cancel`) and `draw_viewport_guidance_hud(ui, viewport_rect, pick_mode, context_badge)` in `overlays.rs`, rendering a top-center rounded `tok.hud_bg` card (`1px` `tok.accent` border) with live count badge (`3 selected`, `2 tabs`, `1st corner set`) and clickable `[Done]` / `[Cancel (Esc)]` pills.

---

### 3. Welcome Quick-Start Overlay & Sample Projects Gallery (`crates/chip-gui/src/panels`)

#### [NEW] `crates/chip-gui/src/panels/welcome_gallery.rs` & [MODIFY] [browser.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/browser.rs)
- Define `SampleProjectId` (`CamTortureBlock`, `AdaptiveRest`, `Pure2dPanel`, `EngravedNameplate`, `VCarveSignInlay`, `MultiFeatureBracket`, `Tilted30Bracket`, `CadRevisionV1V2`), `SampleCategory` (`All`, `Step3D`, `Pure2dSketch`, `VCarveTypography`), and static `SAMPLE_CATALOG` metadata (title, subtitle, category, tools/strategies tags).
- Implement:
  - `show_welcome_modal`: Quick-Start cards (`Open Project / .chipz`, `Import 3D STEP`, `Import 2D DXF / SVG`, `New Pure-2D Sketch`, `Sample Projects Gallery…`), `Recent Projects` MRU list with missing-file badges & `Clear`, featured sample cards, and `"Show Welcome on startup"` checkbox.
  - `show_sample_gallery_modal`: Category filter pills + search box + 2-column sample cards with `[Open Sample]` buttons.
- Clean up `browser.rs` `Models` section: replace the inline `Samples…` dropdown with a clean single `Sample Gallery…` action (or `Import STEP…` / `Reload` / `Replace…` only when a model is loaded), keeping the `Models` tree strictly focused on CAD geometry.
- Update startup flow in `main.rs` & `app/mod.rs`:
  - Add `--demo-torture` and `--welcome` CLI flags (`--theme` made `Option<String>` so omitted `--theme` honors `prefs.toml`, while explicit `--theme dark|light` overrides it!).
  - On interactive bare launch (`initial_path.is_none() && demo_mode == None && self_test.is_none()`), initialize a clean empty workspace and open the Welcome Overlay when `prefs.show_welcome_on_startup` is `true` (while `--self-test-screenshot` without `--welcome` continues rendering `CAM Torture Block`).

---

### 4. `Ctrl+K` Command Palette, `?` / `F1` Shortcut Cheat-Sheet, & v3 Docs (`crates/chip-gui` & Docs)

#### [NEW] `crates/chip-gui/src/panels/command_palette.rs`, [MODIFY] [menus_toolbar.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/menus_toolbar.rs), [app/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/mod.rs), [README.md](file:///workspace/ChipCutter/README.md), [ARCHITECTURE.md](file:///workspace/ChipCutter/ARCHITECTURE.md), & [DEVELOPMENT.md](file:///workspace/ChipCutter/DEVELOPMENT.md)
- Implement `CommandId`, `PaletteEntry { id, category, label, keywords, shortcut }`, `fuzzy_match_score(query, text) -> Option<i32>`, `CommandPaletteState` (`Ctrl+K` toggle, auto-focus search input, `ArrowUp`/`ArrowDown` navigation, `Enter` dispatch, `Esc` dismiss), and `show_shortcut_cheatsheet_modal` (`?` or `F1` when `!ctx.egui_wants_keyboard_input()`).
- Add a compact `[Search / Ctrl+K]` button in the right utility cluster of the single-row top toolbar (`menus_toolbar.rs`), `File -> Open Recent` submenu, and a `Help` menu (`Command Palette (Ctrl+K)`, `Keyboard Shortcuts (? / F1)`, `Sample Projects Gallery…`, `Welcome Screen`).
- Update `README.md`, `ARCHITECTURE.md`, and `DEVELOPMENT.md` to reflect the completed **ChipCutter v3** UI & usability capabilities.

---

## Verification Plan

1. **Automated Unit Tests (`crates/chip-gui`):**
   - `prefs::tests`:
     - Round-trip `UserPreferences` to/from TOML, verify backward-compatible defaults when optional keys are missing, verify `push_recent_file` MRU deduplication & `10`-entry cap, and verify `WorkspaceLayoutShares` clamping & round-trip on `create_workspace_tree_with_shares` / `extract_workspace_shares`.
   - `command_palette::tests`:
     - Verify `fuzzy_match_score` ranks prefix/word-boundary matches (`"vcarve"`, `"ctrl+g"`, `"light"`, `"grbl"`) above scattered subsequence matches and rejects non-matching queries.
2. **Workspace Lint, License, & Regression Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings` (`0` warnings, `0` `too_many_arguments`)
   - `cargo deny check` (`0` new third-party dependencies)
   - `WGPU_BACKEND=vulkan cargo test --workspace` (all 64+ existing tests + new M12 unit tests + all 8 v1 & v2 golden G-code snapshots passing byte-for-byte).
3. **Headless Vulkan Visual Verification (`xvfb-run` + Lavapipe):**
   - Capture and visually inspect headless screenshots via `read_file` for:
     - Welcome / Quick-Start Overlay (`--welcome --self-test-screenshot`) in Studio Dark & Daylight Light;
     - Standard 3D CAM workspace (`--self-test-screenshot` -> updated `docs/images/chipcutter-gui.png`) verifying the `[Ctrl+K]` toolbar pill, `Help` menu, and clean `Models` tree.
