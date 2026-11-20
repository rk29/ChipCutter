# Milestone 31 Implementation Plan — Machine Manager GUI, Per-Machine Tool Library Switching, & Verification

## Goal

Expose **Machine Definitions** and **per-machine tool libraries** natively across the ChipCutter desktop GUI (`chip-gui`): a persistent default machine preference, a **Machine Manager** modal to inspect/clone/edit/import/export `*.machine.toml` profiles and manage each machine's tool list, seamless machine switching in the Setup Inspector and Tool Library modal, automatic kinematics/post/rotary syncing, machine envelope/RPM warnings in Diagnostics, and user documentation.

---

## Proposed Changes

### 1. Persistent Machine Registry & User Preferences (`chip-gui`)

#### [MODIFY] [crates/chip-gui/src/prefs.rs](file:///workspace/ChipCutter/crates/chip-gui/src/prefs.rs)
- Add `#[serde(default = "default_machine_id")] pub default_machine_id: String` (`"generic"`) to `UserPreferences` (`prefs.toml`), preserving 100% backward compatibility with existing `prefs.toml` files.

#### [MODIFY] `crates/chip-gui/src/app/mod.rs`, `crates/chip-gui/src/app/loader.rs`, `crates/chip-gui/src/app/menus_toolbar.rs`
- Load `machines: Vec<MachineDefinition>` on startup via `chip_tools::load_available_machines(None)` and keep the active machine's `ToolLibrary` synchronized with `self.tool_library` whenever `project.machine_id` changes!
- When creating a new project (`File → New` / importing a bare STEP/mesh/DXF/SVG), initialize `project.machine_id = Some(prefs.default_machine_id.clone())` and apply that machine's default post-processor (and 4th-axis defaults when rotary is enabled).
- Add **`Machines...`** menu entry under `Tools` (and Command Palette action `Open Machine Manager` + `Switch Active Machine...` in `crates/chip-gui/src/panels/command_palette.rs`).

---

### 2. Machine Manager Modal & Per-Machine Tool Library Integration (`crates/chip-gui/src/panels/`)

#### [NEW] `crates/chip-gui/src/panels/machine_manager.rs`
- Clean split-pane modal (`Machine Manager`, matching `ToolLibraryModal`'s dark/light theme tokens):
  - **Left Pane — Machine List:** Lists all available machines (`Generic 3/4-Axis CNC Mill`, `Millennium Machines Milo v1.5 (LDO Kit)`, `Makera Carvera Air`, plus all user custom machines from `~/.config/chipcutter/machines/`), badged with `[Built-In]` (read-only) / `[Custom]` (upgrade-safe user machine), travel envelope, RPM range, `4th-Axis` badge, and tool count (`N tools`). Actions: **`Clone to Customize...`**, **`Import (.machine.toml)...`**, **`Export / Share (.machine.toml)...`**, **`Set as Default`**, **`Delete`** (enabled on custom machines only).
  - **Right Pane — Machine Details / Editor Tabs:**
    - When a **built-in** machine is selected: displays a clear info banner (`Built-in preset (read-only) — updated cleanly on app upgrades. Click "Clone to Customize" to create your own editable copy in user config space.`) with a 1-click **`Clone to Customize`** button; all fields are displayed read-only via `ui.add_enabled(false, ...)`.
    - When a **custom** machine is selected: all tabs are fully editable and auto-saved to `~/.config/chipcutter/machines/<id>.machine.toml` via `save_user_machine`:
      - **`Specs & Kinematics`:** Name, manufacturer, XYZ travel envelope (`mm`), spindle RPM min/max, power (`W`), collet type, tool-change workflow (`Manual` / `QuickChange` / `AutomaticAtc`), toolsetter toggle, rapid/cut feed ceilings, acceleration, junction deviation, tool-change time, and default post-processor dropdown.
      - **`4th Axis`:** Toggle 4th-axis rotary capability, axis (`A`/`B`), max stock diameter & length (`mm`), chuck diameter & length (`mm`), max rotary slew rate (`deg/min`), and 4-axis default post profile.
      - **`Machine Tools (N)`:** Summary list of the machine's dedicated tool set with quick **`Edit Tools in Tool Library →`** jump button and **`Copy Tool From Another Machine...`** action.
  - **First-Class Sharing (`Export / Share` & `Import`):**
    - **`Export / Share (.machine.toml)`** opens a native save dialog (`rfd::FileDialog`) exporting a single self-contained `.machine.toml` (machine specs + full tool set) for any built-in or custom machine.
    - **`Import (.machine.toml)...`** opens a native file dialog, parses & validates the shared file via `import_machine_from_path`, automatically handles any ID collision against built-in or existing custom machines, saves the imported machine safely in `~/.config/chipcutter/machines/`, and selects it immediately with a status toast.

#### [MODIFY] [crates/chip-gui/src/panels/tool_library.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/tool_library.rs)
- Add a clean **Machine Scope selector bar** at the top of the **Tool Library** modal (`Showing tools for: [Active Machine Dropdown] | Manage Machines...`):
  - On a **built-in** machine: tools are browsable and selectable for operations (read-only), with a prominent **`Clone Machine to Edit Tools`** button that clones the built-in into user config space, switches the project to the new custom machine, and unlocks full tool editing immediately!
  - On a **custom** machine: adding, editing, deleting, copying from another machine, or importing Fusion 360 `.json`/`.tools` saves directly into that machine's `~/.config/chipcutter/machines/<id>.machine.toml` (upgrade-safe!).
  - Operation Inspector's tool picker automatically displays the active project machine's tools (plus any tools already embedded in the `.chip.toml` project so opening a project made on another machine never hides its existing tools!).

#### [MODIFY] [crates/chip-gui/src/panels/setup_inspector.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/setup_inspector.rs), [crates/chip-gui/src/panels/simulation.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/simulation.rs), [crates/chip-gui/src/panels/diagnostics.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/diagnostics.rs)
- **Setup Inspector (`WCS & Post` tab):** Add a compact **`Machine`** dropdown selector (e.g., `Generic`, `Milo v1.5 (LDO)`, `Carvera Air`, plus custom machines) with a 1-click **`Apply Machine Defaults`** button (updates default post profile, rotary chuck/envelope defaults, and active tool library scope without silently wiping existing operations).
- **Simulation Timeline:** Automatically feed `MachineKinematicsConfig::from_machine_spec(&active_machine.kinematics)` into `run_setup_simulation` so cycle-time estimates reflect the Milo v1.5's `2,000 mm/min` rapids vs. Carvera Air's `4,000 mm/min` rapids and `12 s` quick-change time!
- **Diagnostics Panel:** Surface machine validation warnings (spindle RPM out of range, feedrate above machine ceiling, stock/toolpath exceeding XYZ travel envelope, rotary enabled on 3-axis-only machine) cleanly alongside CAM & simulation diagnostics.
- **Headless Capture (`crates/chip-gui/src/capture.rs`):** Extend `--capture-modal machines` so the Machine Manager modal can be captured deterministically in headless Vulkan tests/docs.

---

### 3. Documentation & Roadmap Completion (`docs/`, `README.md`, `specs/10-roadmap.md`)

#### [NEW] `docs/tutorials/15-machine-definitions-tool-sets.md` (and `docs/images/tutorials/15-machine-definitions-tool-sets/`)
- Concise, illustrated guide covering:
  - Selecting a read-only built-in machine (`Generic`, `Millennium Milo v1.5 (LDO Kit)`, `Makera Carvera Air`) and using its starter tool library.
  - Cloning a built-in machine into upgrade-safe user config space (`~/.config/chipcutter/machines/`), customizing its specs and tools, and exporting/sharing or importing `.machine.toml` files with other machinists.
  - Reading machine envelope/RPM diagnostics and managing machines via `chipcutter machines list|inspect|clone|export|import`.
- Capture 2–3 headless Vulkan screenshots (Machine Manager modal, Scoped Tool Library for `Milo v1.5` / `Carvera Air`, Setup Inspector machine selector) and visually verify via `read_file`.

#### [MODIFY] [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md), [README.md](file:///workspace/ChipCutter/README.md), [specs/10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md)
- Link the new Machine Definitions guide in `docs/tutorials/README.md`, mention built-in machine definitions in `README.md`, and mark **Phase 8 (`M29`–`M31`)** completed in `specs/10-roadmap.md`.

---

## Verification Plan

1. **Automated Unit & Integration Tests:**
   - `cargo test --locked -p chip-gui --lib -- --skip renders_` — unit tests for `UserPreferences` `default_machine_id` roundtrip, machine switching tool-library sync, and `--capture-modal machines` parsing.
   - `cargo test --locked -p chip-project -p chip-tools -p chip-cli` — confirms `.chip.toml` / `.chipz` roundtrip, machine diagnostics, and zero regressions on all 15 golden G-code snapshots.
2. **Visual Headless Vulkan Verification:**
   - Capture the Machine Manager modal and scoped Tool Library modal under headless Vulkan (`WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -a ...`) and inspect every PNG via `read_file` for crisp Dark/Light theme contrast and zero layout clipping.
3. **Full Workspace Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
   - Markdown link & PNG audit (`0` broken links, `0` orphaned PNGs).
