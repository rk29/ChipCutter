# Milestone 76 Implementation Plan — Unified Settings Page (`prefs.toml`) & Workspace Chrome Decluttering

## Goal

Introduce a dedicated, persistent **Settings** page/modal (`prefs.toml`-backed) for infrequently changed workspace, navigation, simulation, and default-machine preferences, and move those low-frequency controls off the main viewport HUD, simulation timeline bar, and top action bar so everyday machining chrome stays clean and focused.

---

## Problem Diagnosis

1. **Scattered Low-Frequency Controls Across Workspace Chrome:**
   - **Simulation Timeline Bar (`crates/chip-gui/src/panels/simulation/ui.rs`):** Displays three dropdowns (`sim_engine_combo`: Heightmap vs. Tri-Dexel, `sim_res_combo`: Fast/Standard/Fine/Ultra, `sim_backend_combo`: Auto/GPU/CPU) permanently on screen, crowding playback controls even though most users pick a preferred simulation engine/resolution once and rarely change it.
   - **Floating Viewport Bottom HUD (`crates/chip-gui/src/viewport/overlays.rs`):** Renders the `MousePreset` dropdown (`Fusion 360` / `Blender` / `SolidWorks` / `FreeCAD`) and `OrbitMode` (`+Z Up` / `Free`) toggle permanently over the 3D canvas even though navigation style is a set-and-forget preference.
   - **Top Action Bar (`crates/chip-gui/src/app/toolbar.rs`):** Reserves permanent right-hand space for `Theme` (`Dark`/`Light`) and display units (`in`/`mm`) toggles alongside high-frequency CAM actions.
2. **Incomplete Preference Persistence (`crates/chip-gui/src/prefs.rs`):**
   - `UserPreferences` (`~/.config/chipcutter/prefs.toml`) persists `theme`, `default_units`, `default_machine_id`, `mouse_preset`, `orbit_mode`, `sim_engine`, `sim_backend`, and `show_welcome_on_startup`, but does **not** yet persist `sim_resolution` or the new `sim_mesh_skinning` surface display option (wired to CPU & GPU meshers in Milestone 79).
3. **No Central Settings Entry Point:**
   - Users must hunt across `View`, `Simulate`, `File`, the toolbar, and the viewport HUD to discover where global preferences live.

---

## Proposed Changes

### 1. Extend `UserPreferences` Schema (`prefs.toml`)

#### [MODIFY] `crates/chip-gui/src/prefs.rs`
- Add backward-compatible fields to `UserPreferences` (using `#[serde(default)]` so existing `prefs.toml` files deserialize cleanly with zero migration errors):
  - `#[serde(default)] pub sim_resolution: SimResolution` (defaults to `SimResolution::Standard`).
  - `#[serde(default = "default_true")] pub sim_mesh_skinning: bool` (defaults to `true` for smooth skinned simulation surfaces once Milestone 79 is wired, or cleanly togglable to `false` for crisp voxel/facet display).
- Add unit tests verifying round-trip serialization of the new fields and backward-compatible deserialization of legacy `prefs.toml` payloads lacking `sim_resolution` / `sim_mesh_skinning`.

---

### 2. Dedicated Settings Page / Modal (`panels/settings.rs`)

#### [NEW] `crates/chip-gui/src/panels/settings.rs`
#### [MODIFY] `crates/chip-gui/src/panels/mod.rs`
#### [MODIFY] `crates/chip-gui/src/app/mod.rs`
#### [MODIFY] `crates/chip-gui/src/app/modals.rs`
#### [MODIFY] `crates/chip-gui/src/app/menus.rs`
#### [MODIFY] `crates/chip-gui/src/app/shortcuts.rs`
#### [MODIFY] `crates/chip-gui/src/app/commands.rs`
#### [MODIFY] `crates/chip-gui/src/panels/command_palette.rs`
- Create `SettingsModalState` with `pub open: bool` and `pub active_section: SettingsSection` (`General`, `Viewport`, `Simulation`, `Machine`), supporting direct deep-linking (e.g. `open_section(SettingsSection::Simulation)`).
- Render a clean two-pane `egui::Window` (`"Settings"` with left category navigation rail + right scrollable two-column form matching ChipCutter's inspector design system):
  1. **General & Appearance:**
     - **Workspace Theme:** `Studio Dark` / `Daylight Light` (applies live + saves to `prefs.toml`).
     - **Default Display Units (New Projects):** `Millimeters (mm)` / `Inches (in)`, plus an explicit **"Current Project Units"** switch (`mm` / `in`) clearly labeled as per-project.
     - **Startup Behavior:** `Show Welcome Gallery on startup` checkbox.
     - **Workspace Layout:** `Reset Default Workspace Layout` button.
  2. **Viewport & Navigation:**
     - **Mouse Navigation Preset:** `Fusion 360`, `Blender`, `SolidWorks`, `FreeCAD` (with concise mouse-binding cheat sheet underneath).
     - **Orbit Mode:** `Turntable (+Z Up)` / `Free Trackball`.
     - **Default Camera Projection:** `Orthographic` / `Perspective`.
     - **Viewport Defaults:** `Show Z-Height Planes`, `Show All Operations in Active Setup`.
  3. **Simulation:**
     - **Simulation Engine:** `Tri-Dexel (Full 3D & Undercuts)` / `Heightmap (Fast 2.5D)`.
     - **Detail Level (Resolution):** `Fast`, `Standard`, `Fine`, `Ultra` (with pitch/memory hint per level).
     - **Compute Backend:** `Auto (GPU with CPU fallback)`, `GPU Only`, `CPU Only` (displaying the active GPU adapter name when available).
     - **Surface Display (Mesh Skinning):** `Smooth Skinned Surfaces` toggle (`prefs.sim_mesh_skinning`) with concise explanation that it smooths curved machined surfaces visually without changing volumetric gouge/error metrology (full CPU & GPU mesher implementation lands in Milestone 79).
  4. **Machine Defaults:**
     - **Default Machine for New Projects:** dropdown of built-in & custom machines (`prefs.default_machine_id`) plus a quick **"Open Machine Manager…"** button.
- Wire entry points consistently:
  - `Edit → Settings…` menu item (`Ctrl+,` shortcut in `shortcuts.rs`).
  - `CommandId::OpenSettings` in `command_palette.rs` and `app/commands.rs`.
  - Single compact **`Settings`** button on the far right of the top action bar (`toolbar.rs`).
- Persist changes immediately via `self.save_prefs()` whenever any preference mutates, and initialize `SimulationPanelState` on startup from `prefs.sim_engine`, `prefs.sim_resolution`, `prefs.sim_backend`, and `prefs.sim_mesh_skinning`.

---

### 3. Declutter Infrequent On-Screen Controls

#### [MODIFY] `crates/chip-gui/src/panels/simulation/ui.rs`
#### [MODIFY] `crates/chip-gui/src/panels/simulation/mod.rs`
- Remove the three permanent dropdowns (`sim_engine_combo`, `sim_res_combo`, `sim_backend_combo`) from the Simulation Timeline top bar.
- Replace them with a single compact, clickable status summary chip (e.g., `Tri-Dexel · Standard · GPU`) with a hover tooltip (`"Click to configure Simulation Engine, Detail Level, Backend, and Mesh Skinning in Settings"`) that emits an action opening `Settings → Simulation`.
- Keep high-value contextual runtime indicators on the Simulation bar: Play/Pause/Scrub controls, speed, Stock Color Mode (`Metallic` / `By Op` / `Error Heatmap`), carryover status pill, and the contextual 1-click **"Switch to Tri-Dexel"** warning pill when an undercut/4-axis op is selected while `Heightmap` is active.

#### [MODIFY] `crates/chip-gui/src/viewport/overlays.rs`
- Remove the `MousePreset` combo box and `OrbitMode` (`+Z Up` / `Free`) button from `draw_viewport_bottom_hud`, leaving only high-frequency viewport controls (`Ortho`/`Persp`, `Fit`, `Top`, and the `Paths`/`Rapids`/`Stock`/`Fixtures`/`Ghost` visibility pills).

#### [MODIFY] `crates/chip-gui/src/app/toolbar.rs`
#### [MODIFY] `crates/chip-gui/src/app/menus.rs`
- Remove the global `Theme` (`Dark`/`Light`) toggle and `in`/`mm` display-units toggle from the right side of the top action bar (these now live in `Settings → General & Appearance`, and project units remain editable there and via Command Palette).
- Trim duplicated engine/resolution/theme radio submenus in `menus.rs` where they solely duplicate Settings, replacing them with a direct `Settings → Simulation…` / `Settings…` link where appropriate while keeping `View` visibility toggles intact.

---

## Verification Plan

1. **Preferences Serialization & Migration Tests (`chip-gui`):**
   - Verify `UserPreferences` round-trips `sim_resolution` and `sim_mesh_skinning` via TOML cleanly.
   - Verify legacy `prefs.toml` strings without `sim_resolution` or `sim_mesh_skinning` load with sensible defaults and zero errors.
2. **Settings Modal & Chrome Integration Checks (`chip-gui`):**
   - Verify `Ctrl+,`, `Edit → Settings…`, Command Palette `OpenSettings`, the toolbar `Settings` button, and the Simulation panel summary chip all open `SettingsModalState` (with the Simulation chip jumping directly to `SettingsSection::Simulation`).
   - Verify mutating `sim_engine`, `sim_resolution`, `sim_backend`, `theme`, `mouse_preset`, and `orbit_mode` in Settings updates live app state and persists to `UserPreferences`.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
