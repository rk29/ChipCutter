# ChipCutter Toolpath Strategy Tutorials

Welcome to the **ChipCutter Tutorials** library. Each tutorial is a self-contained, illustrated Markdown walkthrough that takes you from a starter 3D **STEP** solid, **DXF/SVG** drawing, or built-in **2D Sketch** all the way through setup, unified **`Geometry`** cutting-area controls with live 3D boundary previews, toolpath generation (`Ctrl+G`), 3D **Heightmap** and **Tri-Dexel** volumetric simulation, and safety-verified **GRBL / FluidNC** and **LinuxCNC** G-code.

> [!TIP]
> Want to browse finished visual previews of all 24 documented built-in projects first? Visit the [**Illustrated Sample Projects Gallery**](../samples/README.md) with thumbnails and operation-by-operation simulation screenshots.

> [!WARNING]
> **CNC Machine Safety First:** ChipCutter verifies toolpaths against modeled stock, target B-Rep/mesh geometry, and any enabled 3D workholding fixtures (`Setup → Fixtures`, Rule 6 `FixtureCollision`), but it cannot detect unmodeled hardware on your physical table. Always verify `0 Crashes` in ChipCutter's 3D Simulation, confirm your physical fixtures and `G54`/`G55` zeros match your project, and run an air-cut or single-block dry run on any new program.

---

## Prerequisites

- **ChipCutter installed:** Download and install a prebuilt release for Linux (Flatpak), Windows (`.msi` / `.zip`), or macOS (`.dmg`) from [**GitHub Releases**](https://github.com/rk29/ChipCutter/releases) (see [**Download & Install**](https://github.com/rk29/ChipCutter#download--install-prebuilt-releases), or [**`DEVELOPMENT.md`**](../../DEVELOPMENT.md) if building from source).
- **Launch the desktop CAM app:** Open **ChipCutter** from your application launcher (Start Menu / Applications / desktop menu), or launch it from a terminal:
  ```bash
  chip-gui
  ```
  *(On Linux Flatpak, run `flatpak run io.github.rk29.ChipCutter` for the GUI and `flatpak run --command=chipcutter io.github.rk29.ChipCutter` for the CLI if you do not have the standalone `chipcutter` binary on your `PATH`.)*
- **Units, Preferences, & Settings:** ChipCutter computes internally in millimeters (`mm`), but every numeric field accepts mixed units and parametric variables (for example `1/4 in`, `tool_d * 0.4`, or `model_top_z + 10 mm`). Open **`Edit → Settings… (Ctrl+,)`** (or click the **gear icon** at the right of the **top toolbar**) to configure display units (`mm` / `in`), workspace theme, mouse navigation, **Simulation** engine/detail/backend/**Smooth Skinned Surfaces (Mesh Skinning)**, and your default CNC machine.
- **Active Setup & Top Action Bar:** New operations (`+ Sketch`, `+ Adaptive`, `+ Contour`, `+ V-Carve`, `+ Op…`) are added from the **top toolbar** (or **`Operations`** menu) directly into the currently **Active Setup** (marked with the **`Active`** badge in the left **CAM Browser**).

---

## Tutorial Curriculum

Work through the guides in order if you are new to ChipCutter, or jump directly to the machining strategy you need:

| # | Tutorial | Strategies & Concepts Covered | Starter Model / Asset | Status |
|---|---|---|---|---|
| 00 | [**Getting Started & Workspace Orientation**](00-getting-started.md) | Welcome Overlay, Viewport Navigation, Setup WCS & Stock, Tool Library, `Ctrl+G`, 3D Simulation, & Post-Processing | `Prismatic Multi-Feature Bracket` (Sample Gallery) | **Available** |
| 01 | [**Facing (Stock Surfacing)**](01-facing.md) | Stock top allowance, WCS origin, face/flat mills, pass extension, stepover, zig-zag vs. one-way cutting | [`assets/01-facing-billet.step`](assets/01-facing-billet.step) (`100×80×20 mm` plain billet) | **Available** |
| 02 | [**2D Contour / Profile & Holding Tabs**](02-2d-contour-tabs.md) | Closed outer profiles & open edge chains, stepdowns, helical/linear ramps, lead-in/out arcs, rough/finish passes, holding tabs | [`assets/02-contour-plate.step`](assets/02-contour-plate.step) (`100×70×12 mm` rounded/chamfered plate) | **Available** |
| 03 | [**2D Pocket (Multi-Island & Open-Shoulder)**](03-2d-pocket.md) | Closed multi-island cavities, open-shoulder overhang air entry, contour-parallel vs. zig-zag, radial/axial stock-to-leave | [`assets/03-pocket-block.step`](assets/03-pocket-block.step) (1 island pocket + 1 open step) | **Available** |
| 04 | [**Drilling & Helical Bore Milling**](04-drilling-helical-bore.md) | 3D cylindrical holes (`+ Same Ø`), spot/peck cycles (`G81`/`G83`/`G73`), breakthrough, helical bore interpolation + floor cleanup | [`assets/04-drill-bore-plate.step`](assets/04-drill-bore-plate.step) (4× Ø6mm holes + 1× Ø18mm bore) | **Available** |
| 05 | [**2D Adaptive Clearing & 2.5D Rest Machining**](05-adaptive-rest-machining.md) | Constant-engagement trochoidal roughing (`optimal_load`), radial chip-thinning, `FromPrecedingOperations` DAG waves | [`assets/05-adaptive-pocket.step`](assets/05-adaptive-pocket.step) (`100×80×20 mm` deep sharp-corner pocket) | **Available** |
| 06 | [**Edge Chamfering & 2D Curve Trace / Engraving**](06-chamfer-curve-trace.md) | Modeled `45°` bevel faces vs. sharp unmodeled edges (`tip_offset`, `Floor Guard`), 1-pass single-line Hershey engraving | [`assets/06-chamfer-engrave-block.step`](assets/06-chamfer-engrave-block.step) (`45°` front bevel + sharp back edge) | **Available** |
| 07 | [**3D V-Carving, Two-Tool Flat Clearance, & Mated V-Inlays**](07-vcarve-inlays.md) | TrueType outline text, medial-axis 3D corner climbs, Stage 1 Flat + Stage 2 V-Bit clearance, mirrored Male V-Inlay plugs | Blank Pure-2D Sketch (`"CHIP"` TrueType text) | **Available** |
| 08 | [**3D Adaptive Roughing & 3D Waterline / Z-Level**](08-3d-adaptive-waterline.md) | Triangle mesh import (`STL`/`OBJ`/`3MF`), watertightness diagnostics, `Adaptive3D` + 2D sketch containment, `Waterline3D` adaptive shallow Z-stepdowns | [`assets/08-sculpted-dome.stl`](assets/08-sculpted-dome.stl) (`Ø50 mm` dome on `80×80 mm` pedestal) | **Available** |
| 09 | [**3D Surface Finishing: Parallel Raster, Constant-Scallop, & Pencil Cleanup**](09-3d-surface-finishing.md) | `Parallel3D` (`Pass Angle`, `Cross-Pass (+90°)`, `Slope Range (°)`), `Scallop3D` (`Inside -> Out`, `Min Stepover` / `Max Stepover`), 3D Rest Machining, `Pencil3D` fillet cleanup | [`assets/09-relief-wave.obj`](assets/09-relief-wave.obj) (`80×60×15 mm` relief wave) & [`assets/08-sculpted-dome.stl`](assets/08-sculpted-dome.stl) | **Available** |
| 10 | [**4th-Axis Rotary Wrapping & Continuous Surfacing (`Z+X+A` / `Z+Y+B`)**](10-rotary-wrapping-surfacing.md) | Setup `4th Axis` (`X Axis (A-Axis)` & `Y Axis (B-Axis)`, chuck & clearance), `Wrap 2D` (V-Carve/Pocket/Contour on `linuxcnc_4axis` & `grbl_rotary_wrap`), `4th-Axis Rotary Surfacing` (`Continuous Helical Spiral` & `Axial Parallel (Index & Sweep)`) | Pure-2D Cylinder Sketch & [`assets/09-rotary-shaft.stl`](assets/09-rotary-shaft.stl) | **Available** |
| 11 | [**Indexed `3+1` Multi-Face Milling (STEP Face Alignment & Rotary Flats)**](11-indexed-3plus1-milling.md) | Cardinal `0°/90°/180°/270°` wrench flats, prismatic & angled STEP multi-face chucking, **Align to Selected 3D Face**, safe clearance-radius retracts | [`assets/11-indexed-multiface-block.step`](assets/11-indexed-multiface-block.step) & [`assets/09-rotary-shaft.stl`](assets/09-rotary-shaft.stl) | **Available** |
| 12 | [**Simultaneous 4-Axis Milling (`X + Y + Z + A/B`)**](12-simultaneous-4axis-milling.md) | Full simultaneous `X+Y+Z+A/B` (`Simultaneous 4-Axis (X+Y+Z+A)`), unified **`Geometry → 4th-Axis Rotary Cutting Area (-U / +U)`** (`Axial Source`, `Axial Containment`, signed margins + chuck keep-out, angular presets, live 3D sector preview) & `Heights` `R_max`/`R_min` readouts, `Auto Surface-Normal Tilt` vs. `Fixed Lateral Offset`, `Simultaneous Helical Spiral` vs. `Axial / Helical Flowline`, `G93` inverse-time feeds | [`assets/10-helical-flute-shaft.stl`](assets/10-helical-flute-shaft.stl) (`Ø40×80 mm` 3-start helical flute shaft) | **Available** |
| 13 | [**Pure-2D `DXF` / `SVG` Vector Import & Portable `.chipz` Bundles**](13-pure2d-dxf-svg-workflow.md) | Zero-STEP Pure-2D jobs, `.dxf`/`.svg` units & SVG Pixel DPI, sub-layers, `0.05 mm` gap healing, island nesting, virtual sheet stock, `.chipz` bundle packing & headless build | [`assets/13-router-panel.dxf`](assets/13-router-panel.dxf) (`160×100 mm` multi-layer router front panel) | **Available** |
| 14 | [**Associative STEP Revision Reload (`Ctrl+R`) & Re-Pick Recovery**](14-associative-step-reload.md) | Associative STEP reload (`Ctrl+R` / `reload-step`), WCS-first `GeometricSignature` fuzzy re-matching, selective `Dirty` regeneration, 1-click `NeedsRepick` recovery | [`assets/14-bracket-v1.step`](assets/14-bracket-v1.step) → [`assets/14-bracket-v2.step`](assets/14-bracket-v2.step) | **Available** |
| 15 | [**Machine Definitions, Per-Machine Tool Sets, & Sharing**](15-machine-definitions-tool-sets.md) | 7 built-in read-only profiles (**Generic 3/4-Axis CNC Mill**, **Milo v1.5**, **Milo v2.0**, **Voron Cascade**, **Makera Carvera**, **Carvera Air**, **Carvera Z1**), upgrade-safe cloning, single **`Tools`** bar + actionable **`Project Tools`** card (`+ Add Tool…`, usage badges, `Remove Unused`), grouped/searchable **`Tool`** picker, interactive **Switch Machine** tool-swapping dialog, `.machine.toml` export/import, & envelope/RPM diagnostics | [`machines/*.machine.toml`](../../machines/) | **Available** |
| 16 | [**Rotating, Laying Flat, Scaling, & Moving STEP & STL Models**](16-model-orientation-positioning.md) | Non-destructive `ModelTransform` (`Y-Up -> Z-Up`, `Lay Flat on Clicked Face…`, uniform/non-uniform `Scale` & `Target Size`, `Center XY`, `Drop Bottom to Z=0`, preserved across `Ctrl+R`), Mesh Import orientation, Setup WCS (`Z & X`, `Flip +Z`, `Fine Z Rot`, Op-2 `G55` flip), & `chipcutter transform-model` | [`assets/16-yup-bracket.stl`](assets/16-yup-bracket.stl) & [`assets/16-tilted-bracket.step`](assets/16-tilted-bracket.step) | **Available** |
| 17 | [**Tri-Dexel Volumetric Simulation & Quantitative Error-Volume Analysis**](17-tridexel-simulation-error-metrics.md) | **`Settings → Simulation`** (`Ctrl+,` or Simulation status chip): **Heightmap (Fast)** vs. **Tri-Dexel (Volumetric 3D)**, `Detail Level`, **Smooth Skinned Surfaces (Mesh Skinning)**, signed **Error Heatmap**, **Total / Excess / Gouge Volume**, **RMS / MAE / Bias / P95 / P99 / In-Tol %**, 1-click **`Locate`**, **Per-Operation Error Progression**, wall-clock runtime log, & `chipcutter simulate --engine tri-dexel --report` | `CAM Torture Block` & `Sculpted Dome (STL Mesh)` (Sample Gallery) | **Available** |
| 18 | [**Helical Thread Milling (Internal Holes & External Bosses)**](18-thread-milling.md) | Single-point & multi-form `ThreadMill` cutters, ISO Metric (`M8 × 1.25`, `M12 × 1.75`) & Unified (`1/4-20 UNC`) presets + tap-drill guidance, bottom-up `G3` vs. top-down `G2` climb kinematics, equal-area radial passes, spring passes, pitch-diameter offset, centerline feed compensation, Tri-Dexel undercut grooves, & GRBL/LinuxCNC posting | [`assets/18-threaded-manifold.step`](assets/18-threaded-manifold.step) (`80×60×20 mm` manifold with M8, 1/4-20 UNC, & M12 boss) | **Available** |
| 19 | [**Two-Sided Flip Workholding, 3D Fixtures, & Auto Soft Jaws**](19-workholding-flip-jigs-soft-jaws.md) | 3D workholding (`ViseJaws` + parallels, `ToeClamp`, `Spoilboard`, keep-outs, imported fixture models), **Auto-Avoid Rapids** & Rule 6 `FixtureCollision`, multi-setup **Tri-Dexel Stock Carryover** (`G54 → G55`), **Flip-Jig Dowel-Pin Wizard** (with Poka-Yoke offset), **Auto Soft-Jaw Wizard** (CNC setup + 3D-printable `.stl`), & `chipcutter workholding` CLI | `Two-Sided Vise & Soft-Jaw Bracket` (`--sample two-sided-vise`) & [`assets/02-contour-plate.step`](assets/02-contour-plate.step) | **Available** |
| 20 | [**GPU-Accelerated Simulation, Benchmarking, & Feedrate Optimization**](20-gpu-simulation-feed-optimization.md) | **`Settings → Simulation`** `wgpu` Compute backend (`Auto` / `GPU` / `CPU`) & **Smooth Skinned Surfaces**, clickable Simulation status chip + timing tooltip & wall-clock Activity Log, zero-readback direct GPU stock rendering & `< 0.1 ms` **Error Heatmap** tuning, **Feed Optimization** (corner slowdown, internal-arc scaling, chip-thinning boost), modulated feed telemetry, cusp-height tuning, & `chipcutter simulate --backend gpu --benchmark --json` | `Feed-Optimized Adaptive Pocket` (`--sample feed-opt-pocket`), [`assets/20-feed-opt-pocket.step`](assets/20-feed-opt-pocket.step), `Sculpted Dome`, & `Simultaneous 4-Axis Helical Flute` | **Available** |
| 21 | [**3D Edge Deburring & Non-Planar Chamfering (`Deburr3D`)**](21-3d-edge-deburring-nonplanar-chamfering.md) | Non-planar 3D B-Rep edge deburring (`ChamferMill`, `BallEndmill`, & `LollipopEndmill`), **`Geometry → Cutting Area / Machining Boundary`** (+ live 3D boundary preview) constraining **`Auto-Detect Edges`** (`Min Edge Angle`, `Exclude Vertical`), automatic `FlatBvh` inside-corner wall collision trimming (`Wall Clearance`), circular corner rolls, **Lollipop Underside Back-Chamfering**, & GPU Tri-Dexel verification (`intentional_edge_break_mm`) | `3D Curved & Underside Deburring` (`--sample curved-deburr-3d`), [`assets/21-sloped-boss-deburr.step`](assets/21-sloped-boss-deburr.step), & [`assets/21-overhang-flange.step`](assets/21-overhang-flange.step) | **Available** |
| 22 | [**Undercut Milling: Keyways, T-Slots, Dovetails, & Lollipop 3D Undercuts**](22-undercut-tslot-dovetail-lollipop.md) | Relieved-neck `SlotMill`, `DovetailCutter` (live derived `Top Ø`), & `LollipopEndmill` cutters, mandatory throat pre-clearing, **DIN 650 / ASME T-Slots** & **Blind Keyways** (`SlotUndercut` + transparent `Stock to Leave` / `Finish Allowance`), **Internal `60°` Dovetail Slots & External Dovetail Rails** (`Dovetail`), **Lollipop 3D Overhang Undercuts** (`Undercut3D` + **`Geometry → Cutting Area / Machining Boundary`** & live 3D preview), & GPU Tri-Dexel Error Heatmap verification | `Undercut & 3D Deburr Block` (`--sample undercut-deburr`), `External 60° Dovetail Rail & Blind Keyway` (`--sample dovetail-rail-keyway`), & [`assets/22-undercut-deburr-block.step`](assets/22-undercut-deburr-block.step) | **Available** |

---

## Minimal Tutorial Starter Assets (`docs/tutorials/assets/`)

Each tutorial starts from an **empty project** (or a clean seeded revision) and loads a minimal STEP solid, 3D triangle mesh, or 2D DXF drawing containing only the geometric features required to practice that strategy—keeping the CAM Browser uncluttered:

| Asset File | Dimensions & Features | Reproducible CLI Generator |
|---|---|---|
| [`01-facing-billet.step`](assets/01-facing-billet.step) | `100 × 80 × 20 mm` plain billet (6 planar faces, zero holes/pockets) | `chipcutter generate-sample-step <out.step> --tutorial 01-facing` |
| [`02-contour-plate.step`](assets/02-contour-plate.step) | `100 × 70 × 12 mm` plate with 1× `R10` fillet & 1× `45°` chamfer | `chipcutter generate-sample-step <out.step> --tutorial 02-contour` |
| [`03-pocket-block.step`](assets/03-pocket-block.step) | `100 × 80 × 20 mm` block with 1 closed `Ø14 mm` island pocket + 1 open step | `chipcutter generate-sample-step <out.step> --tutorial 03-pocket` |
| [`04-drill-bore-plate.step`](assets/04-drill-bore-plate.step) | `100 × 70 × 16 mm` plate with 4× `Ø6 mm` through-holes + 1× `Ø18 mm` blind bore | `chipcutter generate-sample-step <out.step> --tutorial 04-drill-bore` |
| [`05-adaptive-pocket.step`](assets/05-adaptive-pocket.step) | `100 × 80 × 20 mm` block with 1 deep `10 mm` sharp-corner pocket + `Ø16 mm` boss | `chipcutter generate-sample-step <out.step> --tutorial 05-adaptive-rest` |
| [`06-chamfer-engrave-block.step`](assets/06-chamfer-engrave-block.step) | `100 × 70 × 18 mm` block with 1 modeled `45° × 1.5 mm` front bevel & sharp back edge | `chipcutter generate-sample-step <out.step> --tutorial 06-chamfer-trace` |
| [`08-sculpted-dome.stl`](assets/08-sculpted-dome.stl) | Watertight `80 × 80 × 25 mm` sculpted dome on pedestal (`STL` mesh) | `chipcutter generate-sample-step <out.stl> --tutorial 08-3d-adaptive-waterline` |
| [`08-bracket.3mf`](assets/08-bracket.3mf) | Watertight sculpted mounting bracket mesh (`3MF` package) | `chipcutter generate-sample-step <out.3mf> --tutorial 08-bracket-3mf` |
| [`09-relief-wave.obj`](assets/09-relief-wave.obj) | Watertight `80 × 60 × 15 mm` sinusoidal sculpted relief wave (`OBJ` mesh) | `chipcutter generate-sample-step <out.obj> --tutorial 09-3d-surface-finishing` |
| [`09-rotary-shaft.stl`](assets/09-rotary-shaft.stl) | Watertight `Ø40 × 80 mm` shaft with 4 wrench flats, collar groove, & 3-lobe cam | `chipcutter generate-sample-step <out.stl> --tutorial 10-rotary-axial` |
| [`11-indexed-multiface-block.step`](assets/11-indexed-multiface-block.step) | `70 × 32 × 32 mm` prismatic STEP block with cardinal & `45°` angled faces | `chipcutter generate-sample-step <out.step> --tutorial 11-indexed-step` |
| [`10-helical-flute-shaft.stl`](assets/10-helical-flute-shaft.stl) | Watertight `Ø40 × 80 mm` 3-start helical impeller/flute shaft | `chipcutter generate-sample-step <out.stl> --tutorial 12-simultaneous-4axis` |
| [`13-router-panel.dxf`](assets/13-router-panel.dxf) | `160 × 100 mm` multi-layer router front panel (`OUTLINE`, `HOLES`, `POCKETS`, `CUTOUTS`) | `chipcutter generate-sample-step <out.dxf> --tutorial 13-pure2d-dxf` |
| [`13-badge-logo.svg`](assets/13-badge-logo.svg) | Pure-2D `96 DPI` cubic Bézier emblem badge artwork (`SVG` vector with counter-islands) | `chipcutter generate-sample-step <out.svg> --tutorial 13-pure2d-svg` |
| [`14-bracket-v1.step`](assets/14-bracket-v1.step) | `90 × 70 × 20 mm` Revision v1 bracket (blind pocket, `Ø14 mm` bore, 2× `Ø6 mm` holes) | `chipcutter generate-sample-step <out.step> --tutorial 14-reload-v1` |
| [`14-bracket-v2.step`](assets/14-bracket-v2.step) | `92 × 70 × 20 mm` Revision v2 bracket (resized pocket, shifted bore/hole, 1 deleted hole) | `chipcutter generate-sample-step <out.step> --tutorial 14-reload-v2` |
| [`16-yup-bracket.stl`](assets/16-yup-bracket.stl) | Watertight `60 × 40 × 18 mm` sculpted mounting boss exported **Y-up** & off-origin (`STL`) | `chipcutter generate-sample-step <out.stl> --tutorial 16-orient-stl` |
| [`16-tilted-bracket.step`](assets/16-tilted-bracket.step) | `90 × 70 × 20 mm` prismatic bracket tilted `30° X / 25° Z` for Setup WCS & Op-2 flip | `chipcutter generate-sample-step <out.step> --tutorial 16-orient-step` |
| [`18-threaded-manifold.step`](assets/18-threaded-manifold.step) | `80 × 60 × 20 mm` aluminum manifold (2× `Ø6.80 mm` M8 pilots, 2× `Ø5.11 mm` 1/4-20 UNC pilots, 1× proud `Ø12.0 mm` boss) | `chipcutter generate-sample-step <out.step> --tutorial 18-thread-milling` |
| [`20-feed-opt-pocket.step`](assets/20-feed-opt-pocket.step) | `100 × 80 × 20 mm` sharp-corner pocket block with raised `Ø16 mm` central island boss | `chipcutter generate-sample-step <out.step> --tutorial 20-feed-opt` |
| [`21-sloped-boss-deburr.step`](assets/21-sloped-boss-deburr.step) | `80 × 50 × 23 mm` stepped shelf (`Z = 14 mm`) & curved 3D circular-arc arch (`Z = 15..23 mm`) for `Deburr3D` wall-trimming | `chipcutter generate-sample-step <out.step> --tutorial 21-deburr-boss` |
| [`21-overhang-flange.step`](assets/21-overhang-flange.step) | `80 × 50 × 18 mm` T-shaped stepped overhang flange (`12 mm` underside lips at `Z = 12 mm`) for Lollipop back-chamfering | `chipcutter generate-sample-step <out.step> --tutorial 21-deburr-flange` |
| [`22-undercut-deburr-block.step`](assets/22-undercut-deburr-block.step) | `100 × 80 × 28 mm` manifold block with DIN 650 M8 T-slot, `60°` dovetail guideway, & `+Y` concave overhang groove | `chipcutter generate-sample-step <out.step> --tutorial 22-undercut-block` |

You can launch any starter STEP, mesh, or DXF model directly into an empty ChipCutter workspace:

```bash
chip-gui docs/tutorials/assets/01-facing-billet.step
```

---

## Tutorial Authoring & Screenshot Conventions

All ChipCutter guides (`00`–`22`) follow a uniform 7-part structure so machinists and contributors can scan any strategy quickly:

1. **Goal & What You'll Learn** — Target machining outcome and key takeaways.
2. **Prerequisites & Starter Project** — How to open the exact starter model or sample job.
3. **Step-by-Step GUI Walkthrough** — Numbered steps with exact menu/tab names, keyboard shortcuts, and annotated screenshots.
4. **Key Parameters Reference** — Concise table explaining the strategy's most important fields and safe starting values.
5. **Generate (`Ctrl+G`) & 3D Simulation Verification** — Inspecting toolpaths, Z-dexel / cylindrical machined stock, and the 5-rule crash/gouge gate.
6. **Post-Processing (GRBL & LinuxCNC)** — Reviewing emitted G-code (`ATC M6` vs. split-file-per-tool, `G93` inverse-time, or `.chipz` bundle export).
7. **Headless CLI Equivalent (`chipcutter`)** — Copy-pasteable commands that reproduce validation, simulation, and G-code build from the terminal.

### Regenerating Tutorial Screenshots

Screenshots are stored under `docs/images/tutorials/<nn>-<slug>/<step>-<name>.png` and captured deterministically via `chip-gui --self-test-screenshot` under headless Xvfb + Vulkan Lavapipe:

```bash
# Capture screenshots for all tutorials across all milestones:
./scripts/capture_tutorial_screenshots.sh --milestone all

# Capture only a single milestone (13, 14, 15, 22, 23, 24, 47, 63, 69, or 70):
./scripts/capture_tutorial_screenshots.sh --milestone 70

# Or capture a single tutorial in Daylight Light theme:
./scripts/capture_tutorial_screenshots.sh --tutorial 13-pure2d-dxf-svg-workflow --theme light
```
