# ChipCutter

**ChipCutter** is an offline desktop 2.5D, 3-Axis Surface, V-Carve, & 4th-Axis Rotary Computer-Aided Manufacturing (CAM) application (`chip-gui`) and headless CLI (`chipcutter`).

It turns 3D **STEP** B-Rep CAD solids (`AP203` / `AP214` / `AP242`), 3D triangle meshes (**STL**, **OBJ**, **3MF**), 2D **DXF / SVG** vector drawings, and parametric 2D sketches & typography into physically verified, arc-accurate (`G2`/`G3`) and inverse-time 4-axis (`G93`) CNC milling, routing, engraving, surfacing, and rotary G-code—without external CAD build dependencies, cloud accounts, or copyleft libraries.

![ChipCutter Desktop CAM Interface — Simultaneous 4-Axis Helical Flute](docs/images/chipcutter-gui.png)

> **Project Website & Illustrated Docs:** [**https://rk29.github.io/ChipCutter/**](https://rk29.github.io/ChipCutter/) · [**Sample Projects Gallery**](docs/samples/README.md) · [**Tutorials (00–17)**](docs/tutorials/README.md)

---

## Download & Install (Prebuilt Releases)

Ready-to-run desktop & CLI packages for Linux, Windows, and macOS are published on the [**GitHub Releases**](https://github.com/rk29/ChipCutter/releases) page alongside a unified `SHA256SUMS.txt` checksum manifest.

### Linux (Flatpak & Standalone CLI)

Install the sandboxed desktop + CLI Flatpak bundle (`x86_64`):

```bash
# Install for the current user (requires Flathub / Freedesktop runtime):
flatpak install --user ChipCutter-<version>-linux-x86_64.flatpak

# Launch the desktop CAM application:
flatpak run io.github.rk29.ChipCutter

# Run the bundled headless CLI:
flatpak run --command=chipcutter io.github.rk29.ChipCutter --version
flatpak run --command=chipcutter io.github.rk29.ChipCutter list-posts
```

For headless servers, CI pipelines, or non-Flatpak workflows, download the standalone CLI archive `chipcutter-<version>-linux-x86_64.tar.gz`, extract it, and place `chipcutter` on your `PATH`.

### Windows (`x86_64`)

- **Installer (recommended):** Download and run `ChipCutter-<version>-windows-x86_64.msi`. This installs `chip-gui.exe` and `chipcutter.exe`, adds Start Menu shortcuts, registers `.chip.toml` / `.chipz` file associations, and optionally adds `chipcutter.exe` to your `PATH`.
- **Portable archive:** Download and extract `ChipCutter-<version>-windows-x86_64.zip` anywhere and launch `chip-gui.exe` (or run `chipcutter.exe` from PowerShell / Command Prompt).

> **Windows SmartScreen note:** Unsigned community releases may trigger a *"Windows protected your PC"* prompt on first launch. Verify the download against `SHA256SUMS.txt` below, then click **More info** → **Run anyway**.

### macOS (Universal2 — Apple Silicon & Intel)

- **Desktop App (`.dmg`):** Download `ChipCutter-<version>-macos-universal.dmg` (macOS `12.0+`, native `arm64` + `x86_64`), open the disk image, and drag **`ChipCutter.app`** into **`Applications`**.
- **Optional CLI symlink:** Expose the bundled `chipcutter` CLI in your terminal:
  ```bash
  ln -s /Applications/ChipCutter.app/Contents/MacOS/chipcutter /usr/local/bin/chipcutter
  chipcutter --version
  ```
  A standalone Universal2 CLI archive (`chipcutter-<version>-macos-universal.tar.gz`) is also attached to each release for terminal-only setups.

> **macOS Gatekeeper note:** Community releases that are ad-hoc signed (not Apple-notarized) may show a Gatekeeper warning on first launch. After verifying `SHA256SUMS.txt`, either **right-click `ChipCutter.app` → Open** in Finder or clear the download quarantine attribute:
> ```bash
> xattr -dr com.apple.quarantine /Applications/ChipCutter.app
> ```

### Verifying Checksums

Download `SHA256SUMS.txt` from the same GitHub Release alongside your installer/archive and verify its SHA-256 digest before running:

```bash
# Linux (ignores artifacts you did not download):
sha256sum -c --ignore-missing SHA256SUMS.txt

# macOS (check a specific downloaded file against SHA256SUMS.txt):
grep "ChipCutter-<version>-macos-universal.dmg" SHA256SUMS.txt | shasum -a 256 -c -
```

```powershell
# Windows (PowerShell):
Get-FileHash .\ChipCutter-<version>-windows-x86_64.msi -Algorithm SHA256
# Compare the Hash output with the matching line in SHA256SUMS.txt
```

---

## Highlights

- **STEP B-Rep Kernel & Associative Reload (`Ctrl+R`):**
  - Imports ISO 10303-21 `.step`/`.stp` solids via `monstertruck`, automatically normalizing units (`mm` / `in`) into a canonical `f64` millimeter kernel and classifying analytic planar faces, cylindrical holes/bosses, cones, lines, and circular arcs.
  - Interactive 3D BVH ray-picking of planar faces, closed edge loops, shortest-path open edge chains, coplanar faces, and same-diameter holes with automatic Z-top / Z-bottom inference.
  - **Associative CAD Revision Re-Matching:** Reloading an edited STEP model (`Ctrl+R` or `chipcutter reload-step`) re-binds Setup WCS references first, fuzzy-matches modified/shifted features via `GeometricSignature`, heals open edge chains, marks matched operations `Dirty`, and flags deleted or ambiguous features for 1-click recovery while preserving all tool, feed/speed, height, pass, and holding-tab settings.

- **Multi-Setup WCS, Non-Destructive Model Orientation/Scaling, & Parametric Machinist Expressions:**
  - Non-destructive per-model **`ModelTransform`** for both **STEP** solids and **STL / OBJ / 3MF** meshes (`Y-Up -> Z-Up`, `±90° X/Y/Z`, 3D viewport **`Lay Flat on Clicked Face…`**, uniform/non-uniform **Scale** & **`Target Size X/Y/Z`**, `Center XY`, `Drop Bottom to Z=0`, preserved across `Ctrl+R` reload and `.chipz` bundles, plus `chipcutter transform-model`).
  - Multiple Setups (`G54`–`G59.3`) per project with box or cylindrical stock, B-Rep face/edge WCS orientation alignment (including tilted parts, Op-2 `Flip +Z`, `Clock X 90°`, and arbitrary `Fine Z Rot (°)`), and an interactive 3D **27-point** stock/model WCS origin picker.
  - Every dimensional input accepts mixed-unit formulas and variables (`1/4 in`, `tool_d * 0.4`, `model_top_z + 10 mm`, `hole_d / 2`).

- **Pure-2D DXF / SVG Vector Import, Parametric Sketches, & Dual-Mode CAM Typography (`chip-sketch`):**
  - **Zero-STEP Pure-2D CNC Jobs & Associative 3D Face Sketches:** Import AutoCAD **DXF** (`LINE`, `ARC`, `CIRCLE`, `LWPOLYLINE` with true bulge arcs, `SPLINE`, `INSERT` blocks) and W3C **SVG** (`path` Béziers/arcs, shapes, `viewBox`, CSS/physical units) with endpoint gap healing (`0.05 mm`) and even-odd island nesting, or author parametric **Rectangles** (with fillets), **Circles**, **Slots**, **Regular Polygons**, **Bolt Circles / Linear Grids**, and **Polylines** directly on Setup XY or any planar 3D STEP face.
  - **Dual-Mode CAM Typography:** TrueType / OpenType (`.ttf`/`.otf`) closed-outline text (`skrifa` + bundled `DejaVuSans-Bold` + custom fonts packed inside `.chipz` `fonts/*`) for pocketing and V-carving, plus built-in **Hershey** single-line stroke fonts (`SimplexSans`, `EngineeringRoman`) for fast 1-pass part-number engraving.

- **2.5D, 3-Axis Surface, V-Carve, & 4th-Axis Rotary Machining Strategies:**
  1. **Facing** — Zig-zag or one-way surfacing across stock XY bounds with pass extension and corner roll arcs.
  2. **2D Contour / Profile** — Exact circular-arc offsetting (`cavalier_contours`), multi-depth stepdowns, helical/linear ramps, horizontal + vertical tangent lead-in/out arcs, rough/finish wall passes, 2.5D Rest Machining, and automatic or click-on-contour rectangular/triangular holding tabs.
  3. **2D Pocket & High-Speed 2D Adaptive Clearing** — Closed pockets with arbitrary islands and open-shoulder air-entry pockets using either contour-parallel/zig-zag clearing OR constant-engagement **2D Adaptive / Trochoidal Clearing** (`optimal_load` `ae`, `both_ways` dynamic Climb/Conventional feed, micro-lift smooth arcs, radial chip-thinning feed boost) plus exact **2.5D Rest Machining** (`FromPreviousTool` & multi-operation `FromPrecedingOperations` DAG waves).
  4. **Drilling & Helical Bore Milling** — Spot, simple drill (`G81`), peck drill (`G83`/`G73`), and multi-pitch helical interpolation + flat-bottom cleanup circle for oversized bores and counterbores.
  5. **Edge Chamfering & 2D Curve Trace** — Exact chamfer-mill tip-offset kinematics (`tip_dia`, included angle, `tip_offset`) for both sharp unmodeled CAD edges and modeled `45°` bevel faces, plus 1-pass centerline curve/Hershey engraving.
  6. **3D Medial-Axis V-Carving, Two-Tool Flat Clearance, & Mated V-Inlays** — Simultaneous 3-axis `(X, Y, Z)` Voronoi medial-axis V-carving (`boostvoronoi`) with razor-sharp 3D corner climbs, optional `flat_depth` clamping with **Two-Stage Flat Endmill Area Clearance**, and mirrored **Male V-Inlay Plug** generation (`start_depth` engagement + `glue_gap_bottom`).
  7. **3-Axis Surface Milling over `STL` / `OBJ` / `3MF` Meshes & STEP Solids** — Pure-Rust `MeshBvh` exact `f64` Drop-Cutter & Push-Cutter kernel powering **3D Adaptive Roughing**, **3D Waterline / Z-Level**, **3D Parallel Raster Finishing**, **3D Constant-Scallop Finishing** (Sethian Fast-Marching Eikonal), and **3D Pencil Cleanup** (morphological ball-in-ball Minkowski rest residual).
  8. **4th-Axis Rotary Milling (`A` / `B` / `C`)** — Configurable rotary setups with chuck/clearance visualization and polar `CylindricalStock` 5-rule verification supporting **2-Linear + Rotary Cylindrical Wrapping (`Z+X+A` / `Z+Y+B`)**, **Continuous Rotary Spiral & Axial Surfacing**, **Indexed `3+1` Multi-Face Milling** (with B-Rep face-normal alignment), and **Simultaneous 4-Axis (`X+Y+Z+A/B`)** off-centerline auto-tilted helical milling with `G93` inverse-time feedrates (`linuxcnc_4axis`, `grblhal_4axis`, and swapped-axis `grbl_rotary_wrap`).

- **Machine Definitions & Per-Machine Tool Libraries (`machines/`):**
  - Ships with read-only built-in profiles for **`Generic 3/4-Axis CNC Mill`**, **`Millennium Machines Milo v1.5 (LDO Kit)`** (with the 5-tool LDO DLC starter endmill set), and **`Makera Carvera Air`** (with its 8-tool `1/8"` starter bit set and optional harmonic-drive 4th-axis rotary module).
  - Each machine carries its own travel envelope, spindle RPM limits, simulation kinematics, default post-processor, rotary capacity, and dedicated tool library—with non-blocking envelope/RPM/feedrate diagnostics in GUI & CLI.
  - Built-in profiles remain read-only so application upgrades stay clean; clone any machine into upgrade-safe user configuration space (`~/.config/chipcutter/machines/*.machine.toml`), or export/share and import single-file `.machine.toml` bundles via the **Machine Manager** (`Tools → Machine Manager…`) or `chipcutter machines`.

- **Tool Library & Coupled SFM / Chipload Calculator:**
  - Supports Flat, Ball, Bull-Nose, Chamfer, Face Mill, and Twist Drill geometries with per-material cutting presets (`Aluminum 6061`, `Mild Steel`, `Hardwood`, `Acrylic`, etc.), radial chip-thinning compensation, machine-scoped tool switching, `.tools.toml` libraries, and a **Fusion 360 `.json` / `.tools` (ZIP)** importer.

- **Selectable Heightmap & Tri-Dexel 3D Volumetric Simulation, Error-Volume Metrology, & Mandatory Crash Gate:**
  - Multi-threaded `rayon` toolpath generation (`Ctrl+G`) and `< 1 ms` `100`-step Undo/Redo (`Ctrl+Z` / `Ctrl+Y`).
  - Switch between fast **2.5D Z-Dexel Heightmap** (plus polar cylindrical 4th-axis stock) and full **3D Tri-Dexel Volumetric** simulation (`X`/`Y`/`Z` orthogonal dexels + sharp Dual-Vertex Surface Nets meshing) with interactive 3D machined-stock & revolved cutter/shank/holder timeline playback and trapezoidal-acceleration cycle-time estimation.
  - **Quantitative Stock-vs-Target Error Metrics & Deviation Heatmap:** Computes Total / Excess / Gouge Error Volume (`mm³` & `%`), Removal Efficiency, **RMS**, MAE, Mean Signed Bias, P95/P99, In-Tolerance Surface %, Max Excess/Gouge with 1-click camera **`Locate`**, Per-Operation Error Progression, diverging Red/Green/Blue 3D Error Heatmap, and structured JSON reports via `chipcutter simulate --engine tri-dexel --report`.
  - **5-Rule Physical Verifier:** Automatically checks every setup for `G0` rapid-into-stock crashes, target B-Rep/mesh part gouges (`> 0.02 mm`, including vertical-wall & undercut gouges in Tri-Dexel mode), non-cutting shank/flute-length collisions, holder collisions, and non-center-cutting plunges—blocking unsafe G-code export in both CLI and GUI.

- **Configurable TOML Post-Processors & Portable `.chipz` Bundles:**
  - Ships with built-in **Grbl 1.1 / FluidNC** (automatic file-per-tool splitting—including Two-Stage Flat Endmill + V-Bit V-Carve operations—`<= 80`-char line guard, quadrant arc splitting, expanded peck drills), **grblHAL / FluidNC ATC**, and **LinuxCNC (RS274NGC)** (`M6` + `G43 H`, `G64 P`, native canned cycles) profiles in [`posts/`](posts/).
  - Git-diff-friendly `.chip.toml` project manifests (`schema_version = 2`, backward-compatible with `v1`) plus byte-deterministic, self-contained **`.chipz`** portable bundles (`project.chip.toml` + `models/*.step` + `vectors/*.{dxf,svg}` + `fonts/*.{ttf,otf}` + custom `posts/*.post.toml` + custom `machines/*.machine.toml`).

- **Studio Dark / Daylight Light Desktop UX & Persistent Preferences:**
  - Centralized `ThemeTokens` design system with WCAG-verified **Studio Dark** and **Daylight High-Contrast Light** themes (including theme-coupled 3D `wgpu` studio background & WCS grid), zero-dependency procedural HiDPI vector iconography, aligned two-column Inspector property grids, and `fx` scoped-variable expression autocomplete & nudging.
  - **Quick-Start Welcome Overlay & Sample Projects Gallery**, fuzzy **`Ctrl+K` Command Palette**, **`F1` / `?` Keyboard Shortcut Cheat-Sheet**, active 3D/2D viewport pick guidance HUD, drag-and-drop operation reordering, and cross-session **`~/.config/chipcutter/prefs.toml`** persistence.

---

## Quickstart

### 1. Prerequisites
- **Rust `1.85+` stable** (Rust Edition 2024).
- On Linux, install native windowing/Vulkan headers for `chip-gui` (macOS and Windows include Metal / DirectX 12 windowing out of the box—see [**`DEVELOPMENT.md`**](DEVELOPMENT.md) for Ubuntu/Debian, Fedora, Arch, macOS, and Windows setup commands).

### 2. Launch the Desktop CAM Application (`chip-gui`)
```bash
# Launch the desktop CAM workspace (opens the Quick-Start Welcome Overlay on bare launch):
cargo run --release -p chip-gui

# Launch directly into built-in 3D STEP, Mesh, 4th-Axis Rotary, Pure-2D, or V-Carve samples:
cargo run --release -p chip-gui -- --sample cam-torture
cargo run --release -p chip-gui -- --sample mesh-dome
cargo run --release -p chip-gui -- --sample rotary-wrap
cargo run --release -p chip-gui -- --sample rotary-shaft
cargo run --release -p chip-gui -- --sample rotary-4axis
cargo run --release -p chip-gui -- --sample pure-2d
cargo run --release -p chip-gui -- --sample vcarve

# Launch in Daylight Light shop-floor theme (theme choice persists in prefs.toml):
cargo run --release -p chip-gui -- --theme light

# Or open an existing .chip.toml project, .chipz bundle, .dxf/.svg vector, or .step model directly:
cargo run --release -p chip-gui -- path/to/part.step
```

#### Essential Desktop Shortcuts
| Shortcut | Action |
|---|---|
| `Ctrl+K` | Open fuzzy **Command Palette** across all actions, views, & sample jobs |
| `F1` / `?` | Open **Keyboard Shortcut & Mouse Reference** cheat-sheet |
| `Ctrl+G` | Generate dirty 2.5D / 3D V-Carve toolpaths on background `rayon` pool |
| `Ctrl+D` / `Del` | Duplicate focused operation / Delete focused operation or sketch entity |
| `Ctrl+Up` / `Ctrl+Down` | Reorder focused operation within its Setup (or drag-and-drop row) |
| `Ctrl+R` | Reload active STEP model & associatively re-match CAM features |
| `Ctrl+O` / `Ctrl+S` | Open (`.chip.toml` / `.chipz`) / Save (`.chip.toml`) project |
| `Ctrl+Z` / `Ctrl+Y` | Instant Undo / Redo (`100` steps) |
| `Space` | Play / Pause 3D simulation timeline scrubbing |
| `P` / `F` | Toggle Orthographic vs. Perspective camera / Fit model to viewport |
| `Esc` | Exit active 3D picking / sketch mode or cancel in-flight generation |

### 3. Headless CLI Workflow (`chipcutter`)
Every CAD, toolpath, simulation, post-processing, associative reload, and bundling feature works headlessly without a GPU or display server:

```bash
# 1. Generate a sample machined STEP solid + seeded 8-operation .chip.toml project
cargo run -p chip-cli -- generate-sample-step /tmp/torture.step --cam-torture --project-out /tmp/torture.chip.toml

# 2. Inspect normalized STEP B-Rep geometry (planes, cylindrical holes, cones, arcs)
cargo run -p chip-cli -- inspect /tmp/torture.step

# 3. Validate project invariants and generate all 2.5D toolpaths
cargo run -p chip-cli -- validate --generate /tmp/torture.chip.toml

# 4. Run headless stock removal (heightmap or tri-dexel) & 5-rule crash/gouge + error-volume/RMS report
cargo run -p chip-cli -- simulate /tmp/torture.chip.toml --engine tri-dexel --resolution fine --tolerance 0.05 --report /tmp/sim_report.json

# 5. Build safety-verified G-code (LinuxCNC single ATC file or Grbl split-by-tool files)
cargo run -p chip-cli -- build /tmp/torture.chip.toml -o /tmp/out_linuxcnc --post linuxcnc
cargo run -p chip-cli -- build /tmp/torture.chip.toml -o /tmp/out_grbl --post grbl

# 6. Pack a self-contained portable .chipz archive (and build directly from it)
cargo run -p chip-cli -- bundle pack /tmp/torture.chip.toml -o /tmp/torture.chipz
cargo run -p chip-cli -- build /tmp/torture.chipz -o /tmp/out_from_bundle --post linuxcnc

# 7. Reload a revised STEP file and associatively re-bind operations
cargo run -p chip-cli -- reload-step /tmp/torture.chip.toml --step /tmp/torture_v2.step -o /tmp/torture_v2.chip.toml

# 8. Inspect & import a 2D DXF or SVG vector drawing into a Pure-2D .chip.toml job
cargo run -p chip-cli -- inspect panel.dxf
cargo run -p chip-cli -- import-2d panel.dxf -o /tmp/panel.chip.toml --thickness 6.0

# 9. Convert a Fusion 360 tool library (.json or .tools) into native .tools.toml
cargo run -p chip-cli -- tools import-fusion shop_tools.json -o shop.tools.toml

# 10. List, inspect, clone, export, and import machine definitions (Generic, Milo v1.5, Carvera Air)
cargo run -p chip-cli -- machines list
cargo run -p chip-cli -- machines inspect milo-1.5
cargo run -p chip-cli -- machines clone carvera-air my-carvera --name "My Carvera Air"
cargo run -p chip-cli -- machines export my-carvera -o my-carvera.machine.toml
```

---

## Documentation

- [**`docs/samples/`**](docs/samples/README.md) — **Illustrated Sample Projects Gallery** with thumbnails of all 15 built-in STEP, Mesh, 4th-Axis Rotary, Pure-2D, and V-Carve samples plus per-Setup and per-Operation completion simulation screenshots
- [**`docs/tutorials/`**](docs/tutorials/README.md) — Illustrated step-by-step Getting Started, 2.5D, 3-Axis Surface, 4th-Axis Rotary, Machine Definitions, Model Orientation/Scaling, Tri-Dexel Simulation & Error Metrics, & Production Workflow tutorials (`00`–`17`)
- [**`ARCHITECTURE.md`**](ARCHITECTURE.md) — End-to-end data pipeline, crate dependency graph, threading/caching model, and 3D render passes
- [**`DEVELOPMENT.md`**](DEVELOPMENT.md) — Workspace layout, platform build prerequisites, native Linux/macOS/Windows setup, and headless CI instructions
- [**`specs/00-overview.md`**](specs/00-overview.md) — Product scope, non-goals, and full specification index (`00` through `11`)
- [**`specs/10-roadmap.md`**](specs/10-roadmap.md) — Completed Phase 1–12 (`M1`–`M43`) milestones and future phases
- [**`AGENTS.md`**](AGENTS.md) — Architectural invariants and contributor guidelines

---

## Verification & Testing

```bash
# Formatting, strict Clippy lints, and permissive license audit (deny.toml)
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo deny check

# Full unit, physical heightmap oracle, offscreen wgpu, and golden CLI snapshot suite
cargo test --workspace

# Headless Linux GUI launch + screenshot self-test (using Lavapipe + Xvfb)
WGPU_BACKEND=vulkan xvfb-run -a cargo run -p chip-gui -- --self-test-screenshot /tmp/preview.png
```

---

## License

Licensed under either of:

- **Apache License, Version 2.0** ([`LICENSE-APACHE`](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- **MIT License** ([`LICENSE-MIT`](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option. All third-party dependencies are verified permissive-only via [`deny.toml`](deny.toml) (`cargo deny check licenses`).

## Disclaimers

This is a personal project. The views, code, and opinions expressed here are my own and do not represent those of my current or past employers.

This software is currently, at best, alpha quality. Do not trust the output it generates without manually verifying it yourself. Using the generated gcode output could result in damage to your CNC machine.
