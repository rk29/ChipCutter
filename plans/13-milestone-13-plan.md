# ChipCutter — Milestone 13 Implementation Plan

## Goal Description
Implement **Milestone 13 (Tutorial Framework, Screenshot Capture Pipeline, & Getting Started Guide)** from [10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md#L85-L88)—establishing the documentation structure, deterministic headless screenshot capture pipeline, and introductory illustrated guide for **Phase 4 (Illustrated Toolpath Strategy Tutorials)**:

1. **Deterministic Headless Screenshot Capture Support (`crates/chip-gui`, `scripts/capture_tutorial_screenshots.sh`):**
   - Extend `chip-gui`'s existing `--self-test-screenshot` mechanism with structured headless capture options (`CaptureSpec`) so tutorials across Milestones 13–15 can reproducibly capture exact workspace states (any built-in sample project or `.chip.toml` file, specific camera preset, Setup/Operation/Sketch Inspector tab, bottom pane, modal overlay, single-operation toolpath isolation, and 3D machined-stock simulation progress) without touching interactive UX or core crates.
   - Add a repeatable capture script (`scripts/capture_tutorial_screenshots.sh`) running under `WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -a` to generate all Milestone 13 screenshots into `docs/images/tutorials/00-getting-started/`, structured so Milestones 14 and 15 can append their tutorial shot lists cleanly.
2. **Tutorials Hub, Authoring Conventions, & Starter Asset Map (`docs/tutorials/README.md`):**
   - Create `docs/tutorials/README.md` containing prerequisites, the full Phase 4 curriculum table (`00` through `07`), a starter model/sample mapping table covering every strategy, and the standardized Markdown tutorial layout & screenshot naming conventions for Milestones 13–15.
3. **Illustrated Getting Started & Workspace Orientation Guide (`docs/tutorials/00-getting-started.md`):**
   - Author an end-to-end illustrated beginner guide covering launching `chip-gui`, Welcome Overlay & Sample Projects Gallery, 3D/2D viewport navigation & workspace layout, Setup WCS / stock / Z-height configuration, Tool Library & parametric expressions, generating toolpaths (`Ctrl+G`), 3D heightmap simulation & 5-rule crash verification, GRBL/LinuxCNC post-processing, and equivalent headless `chipcutter` CLI commands—backed by 7 verified headless screenshots in `docs/images/tutorials/00-getting-started/`.

---

## Key Architectural Decisions & Scope Guardrails

> [!IMPORTANT]
> **Zero New External Crate Dependencies & Core Crates 100% Untouched:**
> - All headless capture options live strictly inside `crates/chip-gui` using existing `clap`, `eframe`, `egui`, and `image` dependencies (`cargo deny check` remains unchanged).
> - All UI-agnostic core crates (`chip-math`, `chip-cad`, `chip-sketch`, `chip-tools`, `chip-cam`, `chip-sim`, `chip-post`, `chip-project`, `chip-cli`) remain 100% untouched in Milestone 13.
> - No in-app tutorial engine, UI modals, or `prefs.toml` schema changes are introduced—Phase 4 tutorials are purely Markdown guides with screenshots.

> [!TIP]
> **Backward Compatibility for `--self-test-screenshot` & Interactive Runs:**
> - Running `chip-gui --self-test-screenshot <PATH>` with no `--capture-*` flags preserves exact existing behavior (loads `CAM Torture Block`, scrubs simulation to `82%`, captures PNG, and exits).
> - All `--capture-*` flags are ignored unless `--self-test-screenshot` is active, and `prefs.toml` continues to be skipped whenever `self_test.is_some()`, ensuring zero interference with host user preferences.
> - Existing `--demo-torture`, `--demo-pure-2d`, `--demo-nameplate`, `--demo-vcarve`, and `--welcome` flags remain intact alongside the new `--sample <id>` selector.

> [!NOTE]
> **Zero Broken Links While M14–M15 Are Pending:**
> - In `docs/tutorials/README.md`, `00-getting-started.md` links directly to the live guide, while tutorials `01`–`07` appear in the curriculum roadmap table as plain titles (`Planned — Milestone 14` / `Planned — Milestone 15`) until their respective milestones land, keeping all Markdown links 100% valid at every commit.

---

## Proposed Changes

### 1. Deterministic Headless Screenshot Capture Options (`crates/chip-gui`)

#### [NEW] `crates/chip-gui/src/capture.rs` & [MODIFY] [lib.rs](file:///workspace/ChipCutter/crates/chip-gui/src/lib.rs)
Define pure, unit-testable capture configuration types (zero `wgpu`/`eframe` dependencies so parsing and formatting are trivially unit-tested):
- `CaptureView`: `Isometric`, `Top`, `Bottom`, `Front`, `Back`, `Right`, `Left`, `SetupTop`, `ActiveSketch` (parsed from `"iso"`, `"top"`, `"front"`, `"right"`, `"left"`, `"back"`, `"bottom"`, `"setup-top"`, `"sketch"`).
- `CaptureFocus`:
  - `Setup(SetupTab)` (`"setup:wcs"`, `"setup:stock"`, `"setup:heights"`),
  - `Operation { setup_idx: usize, op_idx: usize, tab: OperationTab }` (`"op:<idx>:<tool|geometry|heights|passes|linking>"`, optional `"op:<setup>:<idx>:<tab>"`),
  - `Sketch { sketch_idx: usize }` (`"sketch:<idx>"`).
- `CapturePane`: `Simulation`, `GCode`, `Diagnostics` (mapped to `WorkspacePane::{SimulationTimeline, GCodePreview, DiagnosticsLog}`).
- `CaptureModal`: `Welcome`, `SampleGallery`, `ToolLibrary`, `ShortcutCheatsheet`.
- `CaptureSpec`:
  ```rust
  #[derive(Debug, Clone, Default, PartialEq)]
  pub struct CaptureSpec {
      pub view: Option<CaptureView>,
      pub focus: Option<CaptureFocus>,
      pub pane: Option<CapturePane>,
      pub modal: Option<CaptureModal>,
      pub isolate_op: bool,
      pub show_sim_stock: Option<bool>,
      pub sim_progress: Option<f64>, // clamped to 0.0..=1.0
      pub auto_generate: bool,
      pub show_z_planes: Option<bool>,
      pub show_27pt_dots: Option<bool>,
      pub ghost_part: Option<bool>,
  }
  ```
- Add `SampleProjectId::from_cli_slug(&str) -> Option<Self>` in [welcome_gallery.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/welcome_gallery.rs) supporting slugs: `cam-torture`, `adaptive-rest`, `pure-2d`, `nameplate`, `vcarve`, `multi-feature`, `tilted-bracket`, `revision-v1`, `revision-v2`, `sample-plate`.

#### [MODIFY] [main.rs](file:///workspace/ChipCutter/crates/chip-gui/src/main.rs), [app/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/mod.rs), [app/loader.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/loader.rs), & [app/pane_behavior.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/pane_behavior.rs)
- Extend `GuiArgs` in `main.rs` with:
  - `--sample <SLUG>` (maps to `StartupDemoMode::Sample(SampleProjectId)`),
  - `--capture-view <VIEW>`, `--capture-focus <FOCUS>`, `--capture-pane <PANE>`, `--capture-modal <MODAL>`, `--capture-isolate-op`, `--capture-sim-stock <true|false>`, `--capture-sim-progress <0.0..=1.0>`, `--capture-generate`, `--capture-z-planes <true|false>`, `--capture-27pt-dots <true|false>`, `--capture-ghost-part <true|false>`.
- Store `pub capture: CaptureSpec` and `pub(crate) capture_applied: bool` / `pub(crate) generate_triggered: bool` on `SelfTestConfig`.
- Update `ChipCutterApp::handle_self_test` in [pane_behavior.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/pane_behavior.rs#L742-L802):
  1. If `capture.auto_generate` is set and a `.chip.toml`/`.chipz` project finished loading (`!self.loading_in_flight && self.has_geometry()`) and `!st.generate_triggered`, call `self.trigger_generate_dirty(None, None)` once and wait for `self.dispatcher.is_busy() || self.sim_panel.is_busy()` to complete.
  2. Once loading, toolpath generation, and simulation are idle, apply `CaptureSpec` deterministically once before frame settle (`frames_rendered == 1`):
     - Apply `focus` (`active_setup_idx`, `inspector_focus`, `setup_tab`, `op_tab`),
     - Apply `isolate_op` (`self.show_all_setup_toolpaths = !capture.isolate_op`),
     - Apply `show_z_planes`, `show_27pt_dots`, `ghost_part`,
     - Apply `show_sim_stock` (`self.sim_panel.show_machined_stock`) and `sim_progress` (defaulting to `0.82` when `None` for exact backward compatibility with existing `--self-test-screenshot` calls),
     - Apply `pane` via `focus_workspace_pane(&mut self.tree, ...)` (and if `CapturePane::GCode` is requested, ensure `gcode_panel.ensure_profiles_loaded(...)` so G-code preview populates before screenshot capture),
     - Apply `modal` (`welcome_modal.open`, `sample_gallery_modal.open`, `tool_library_modal.open`, `shortcut_cheatsheet.open`),
     - Apply `view` (`camera.set_standard_view(...)`, `align_camera_to_active_setup()`, or `align_camera_to_sketch(...)`).
  3. Wait `6` settled frames (`frames_rendered >= 6`) so `egui_tiles` tab focus, `GCodePreviewPanelState` lazy rebuild, and `wgpu` MSAA viewport buffers reflect the applied state cleanly before requesting the PNG screenshot and closing.

---

### 2. Repeatable Tutorial Screenshot Capture Script (`scripts/`)

#### [NEW] `scripts/capture_tutorial_screenshots.sh`
- Executable POSIX/bash helper script supporting:
  - `--milestone 13` (or `--all`, `--tutorial 00-getting-started`),
  - `--theme dark|light` (default `dark`),
  - `--debug` vs. `--release` (default `--release` build once up front so each screenshot takes ~1–2 seconds rather than recompiling).
- Always invokes `chip-gui` with `WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -a` per `AGENTS.md` §4 so no visible windows open on host Wayland desktops.
- Captures the 7 Milestone 13 screenshots into `docs/images/tutorials/00-getting-started/`:
  1. `01-welcome-overlay.png` — `--welcome` (Quick-Start overlay on bare launch).
  2. `02-sample-gallery.png` — `--sample cam-torture --capture-modal samples` (Filterable Sample Projects Gallery).
  3. `03-workspace-overview.png` — `--sample multi-feature --capture-view iso --capture-sim-stock false --capture-focus op:0:passes` (Clean 3D workspace showing Browser, Viewport, ViewCube, View HUD, Inspector, and Bottom Tabs).
  4. `04-setup-wcs-stock.png` — `--sample multi-feature --capture-view iso --capture-sim-stock false --capture-focus setup:wcs --capture-27pt-dots true --capture-z-planes true` (Setup WCS Inspector + 27-point origin dots + Z planes).
  5. `05-tool-library.png` — `--sample multi-feature --capture-modal tools` (Tool Library modal with cutter catalog, material presets, and SFM/chipload calculator).
  6. `06-simulation-stock.png` — `--sample multi-feature --capture-view iso --capture-pane sim --capture-sim-stock true --capture-sim-progress 0.85` (3D Z-dexel machined stock, animated cutter assembly, and Simulation Timeline).
  7. `07-gcode-preview.png` — `--sample multi-feature --capture-view iso --capture-pane gcode --capture-sim-stock true --capture-sim-progress 1.0` (Verified G-Code Preview tab with LinuxCNC/GRBL post-processor output).

---

### 3. Tutorials Index, Standard Template, & Starter Asset Map (`docs/tutorials/README.md`)

#### [NEW] `docs/tutorials/README.md`
Structured landing page for all ChipCutter tutorials containing:
1. **Introduction & Prerequisites:** Building/launching `chip-gui` and `chipcutter`, units (`mm` / `in` expression support), and a concise CNC safety reminder (always verify zero crashes/gouges in Simulation and dry-run/air-cut on real machines).
2. **Tutorial Curriculum Table:**
   | # | Tutorial | Strategies & Concepts | Starter Project / Model | Status |
   |---|---|---|---|---|
   | 00 | [Getting Started & Workspace Orientation](00-getting-started.md) | Workspace, Viewport, Setup WCS/Stock, Tools, `Ctrl+G`, Sim, Post | `Multi-Feature Bracket` (Sample Gallery) | Available |
   | 01 | Facing (Stock Surfacing) | Stock top allowance, pass extension, zig-zag vs. one-way | `CAM Torture Block` / `Multi-Feature Bracket` | Planned (M14) |
   | 02 | 2D Contour / Profile & Holding Tabs | Closed/open loops, ramps, lead-in/out arcs, rough/finish, tabs | `Pure-2D Router Panel` & `CAM Torture Block` | Planned (M14) |
   | 03 | 2D Pocket (Multi-Island & Open-Shoulder) | Closed island pockets, open-shoulder overhang, stock-to-leave | `CAM Torture Block` | Planned (M14) |
   | 04 | Drilling & Helical Bore Milling | 3D holes & 2D bolt circles, `G81`/`G83`/`G73`, helical bores | `CAM Torture Block` & `Engraved Nameplate` | Planned (M14) |
   | 05 | 2D Adaptive Clearing & 2.5D Rest Machining | Trochoidal `optimal_load`, chip-thinning, `FromPrecedingOperations` DAG | `Adaptive + Rest Demo` | Planned (M15) |
   | 06 | Edge Chamfering & 2D Curve Trace / Engraving | Sharp vs. `45°` modeled bevels (`tip_offset`), Hershey text trace | `CAM Torture Block` & `Engraved Nameplate` | Planned (M15) |
   | 07 | 3D V-Carving, Flat Clearance, & Mated V-Inlays | Medial-axis 3D climbs, Stage 1+2 clearance, Male V-Inlay plug | `V-Carve Sign & V-Inlay` | Planned (M15) |
3. **Standard Tutorial Format & Conventions:** Documents the consistent 7-section layout (`Goal & Finished Result`, `Prerequisites & Starter Project`, `Step-by-Step Walkthrough`, `Key Parameters Reference`, `Generate & Verify in 3D Simulation`, `Post-Processing (GRBL & LinuxCNC)`, `Headless CLI Equivalent`) and screenshot regeneration command (`scripts/capture_tutorial_screenshots.sh`).

---

### 4. Illustrated Getting Started Guide (`docs/tutorials/00-getting-started.md`)

#### [NEW] `docs/tutorials/00-getting-started.md`
A clear, practical, machinist-friendly walkthrough embedding all 7 screenshots from `../images/tutorials/00-getting-started/`:
1. **Goal & What You'll Learn** — End-to-end orientation from opening a 3D STEP bracket to producing verified G-code.
2. **Launching ChipCutter & Opening a Starter Job** — Welcome Overlay (`01-welcome-overlay.png`), Sample Projects Gallery (`02-sample-gallery.png`), opening `Prismatic Multi-Feature Bracket`, or importing your own `.step` / `.dxf` / `.svg`.
3. **Workspace Anatomy & Viewport Navigation** — Annotated walkthrough of `03-workspace-overview.png` (Toolbar, CAM Browser, 3D/2D Viewport + ViewCube + Floating View HUD, Inspector, Bottom Tabs), mouse presets (`Fusion 360`, `SolidWorks`, `FreeCAD/Blender`), and essential hotkeys (`P`, `F`, `Ctrl+G`, `Space`, `Ctrl+K`, `F1`/`?`, `Ctrl+Z`/`Ctrl+Y`).
4. **Configuring Setup Stock, WCS Origin, & Safe Heights** — Walkthrough of `04-setup-wcs-stock.png` explaining box/cylinder stock sizing, WCS axis alignment, 27-point origin placement, and the `Clearance >= Retract >= Feed >= Top >= Bottom` invariant.
5. **Tools, Material Presets, & Parametric Expressions** — Walkthrough of `05-tool-library.png` covering starter cutters (`T1`–`T8`), SFM/chipload coupling, Fusion 360 `.json`/`.tools` import, and writing mixed-unit expressions (`1/4 in`, `tool_d * 0.4`, `model_top_z + 10 mm`).
6. **Generating Toolpaths (`Ctrl+G`) & Running 3D Simulation** — Status badge meanings (`Clean`, `Dirty`, `Generating`, `Error`), triggering `Ctrl+G`, scrubbing the Simulation Timeline (`06-simulation-stock.png`), inspecting machined stock & cutter assembly, and verifying the 5-rule crash/gouge gate.
7. **Previewing & Exporting G-Code (GRBL vs. LinuxCNC)** — Walkthrough of `07-gcode-preview.png` covering post-processor profiles (`linuxcnc`, `grbl`, `grblhal_atc`), ATC vs. automatic split-file-per-tool emission, and exporting `.nc`/`.ngc` files.
8. **Headless CLI Equivalent (`chipcutter`)** — Copy-pasteable commands generating, validating, simulating, and building the same `Multi-Feature Bracket` project headlessly, plus a **"Next Steps"** link back to `README.md` (and onward to `01-facing.md` once M14 lands).

---

## Verification Plan

1. **Automated Unit Tests (`crates/chip-gui`):**
   - `capture::tests`: Verify parsing of all `CaptureView`, `CaptureFocus` (`setup:*`, `op:<idx>:<tab>`, `op:<setup>:<idx>:<tab>`, `sketch:<idx>`), `CapturePane`, `CaptureModal`, and `SampleProjectId::from_cli_slug` values, clamping of `sim_progress` to `[0.0, 1.0]`, and clean error messages for malformed flag inputs.
2. **Workspace Lint, License, & Regression Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings`
   - `cargo deny check` (`0` new third-party dependencies)
   - `WGPU_BACKEND=vulkan cargo test --workspace`
3. **Headless Screenshot Generation & Visual Inspection:**
   - Run `./scripts/capture_tutorial_screenshots.sh --milestone 13` (using `WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -a`).
   - Visually inspect all 7 generated PNGs in `docs/images/tutorials/00-getting-started/` via `read_file` to confirm every panel, modal, toolpath, machined stock mesh, and G-code preview renders cleanly without overlap, clipping, or blank frames.
   - Verify legacy `WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -a cargo run -p chip-gui -- --self-test-screenshot /tmp/m13_compat.png` still works identically.
4. **CLI Walkthrough & Link Audit:**
   - Execute every `chipcutter` CLI command documented in `docs/tutorials/00-getting-started.md` in `/tmp` and confirm exit code `0` with zero critical crashes/gouges.
   - Verify every relative Markdown link and image reference in `docs/tutorials/README.md` and `docs/tutorials/00-getting-started.md` resolves to an existing file.
