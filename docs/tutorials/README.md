# ChipCutter Toolpath Strategy Tutorials

Welcome to the **ChipCutter Tutorials** library. Each tutorial is a self-contained, illustrated Markdown walkthrough that takes you from a starter 3D **STEP** solid, **DXF/SVG** drawing, or built-in **2D Sketch** all the way through setup, toolpath generation (`Ctrl+G`), 3D heightmap simulation, and safety-verified **GRBL / FluidNC** and **LinuxCNC** G-code.

> [!WARNING]
> **CNC Machine Safety First:** Simulation checks toolpaths against modeled stock and target B-Rep geometry, but it does **not** know where your clamps, vise jaws, or spoilboard fasteners sit. Always verify `0 Crashes` in ChipCutter's 3D Simulation, double-check your physical `G54` zero on the machine, and run an air-cut or single-block dry run on any new program.

---

## Prerequisites

- **ChipCutter built locally:** See [**`DEVELOPMENT.md`**](../../DEVELOPMENT.md) if you have not built `chip-gui` and `chipcutter` yet.
- **Launch the desktop CAM app:**
  ```bash
  cargo run --release -p chip-gui
  ```
- **Units & Expressions:** ChipCutter computes internally in millimeters (`mm`), but every numeric field accepts mixed units and parametric variables (for example `1/4 in`, `tool_d * 0.4`, or `model_top_z + 10 mm`). You can toggle display units (`mm` / `in`) at any time on the right side of the top toolbar.

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
| 12 | [**Simultaneous 4-Axis Milling (`X + Y + Z + A/B`)**](12-simultaneous-4axis-milling.md) | Full simultaneous `X+Y+Z+A/B` (`Simultaneous 4-Axis (X+Y+Z+A)`), `Auto Surface-Normal Tilt` vs. `Fixed Lateral Offset`, `Simultaneous Helical Spiral` vs. `Axial / Helical Flowline`, `G93` inverse-time feeds | [`assets/10-helical-flute-shaft.stl`](assets/10-helical-flute-shaft.stl) (`Ø40×80 mm` 3-start helical flute shaft) | **Available** |
| 13 | [**Pure-2D `DXF` / `SVG` Vector Import & Portable `.chipz` Bundles**](13-pure2d-dxf-svg-workflow.md) | Zero-STEP Pure-2D jobs, `.dxf`/`.svg` units & SVG Pixel DPI, sub-layers, `0.05 mm` gap healing, island nesting, virtual sheet stock, `.chipz` bundle packing & headless build | [`assets/13-router-panel.dxf`](assets/13-router-panel.dxf) (`160×100 mm` multi-layer router front panel) | **Available** |
| 14 | [**Associative STEP Revision Reload (`Ctrl+R`) & Re-Pick Recovery**](14-associative-step-reload.md) | Associative STEP reload (`Ctrl+R` / `reload-step`), WCS-first `GeometricSignature` fuzzy re-matching, selective `Dirty` regeneration, 1-click `NeedsRepick` recovery | [`assets/14-bracket-v1.step`](assets/14-bracket-v1.step) → [`assets/14-bracket-v2.step`](assets/14-bracket-v2.step) | **Available** |
| 15 | [**Machine Definitions, Per-Machine Tool Sets, & Sharing**](15-machine-definitions-tool-sets.md) | Built-in read-only profiles (**Generic 3/4-Axis CNC Mill**, **Millennium Machines Milo v1.5 (LDO Kit)**, **Makera Carvera Air**), upgrade-safe cloning, per-machine tool libraries, `.machine.toml` export/import, envelope/RPM diagnostics | [`machines/*.machine.toml`](../../machines/) | **Available** |
| 16 | [**Rotating, Laying Flat, Scaling, & Moving STEP & STL Models**](16-model-orientation-positioning.md) | Non-destructive `ModelTransform` (`Y-Up -> Z-Up`, `Lay Flat on Clicked Face…`, uniform/non-uniform `Scale` & `Target Size`, `Center XY`, `Drop Bottom to Z=0`, preserved across `Ctrl+R`), Mesh Import orientation, Setup WCS (`Z & X`, `Flip +Z`, `Fine Z Rot`, Op-2 `G55` flip), & `chipcutter transform-model` | [`assets/16-yup-bracket.stl`](assets/16-yup-bracket.stl) & [`assets/16-tilted-bracket.step`](assets/16-tilted-bracket.step) | **Available** |
| 17 | [**Tri-Dexel Volumetric Simulation & Quantitative Error-Volume Analysis**](17-tridexel-simulation-error-metrics.md) | Selectable **Heightmap (Fast)** vs. **Tri-Dexel (Volumetric 3D)** simulation, signed **Error Heatmap** (`Red` gouge / `Green` ±tol / `Blue` excess), **Total / Excess / Gouge Volume**, **RMS / MAE / Bias / P95 / P99 / In-Tol %**, 1-click **`Locate`**, **Per-Operation Error Progression**, & `chipcutter simulate --engine tri-dexel --report` | `CAM Torture Block` & `Sculpted Dome (STL Mesh)` (Sample Gallery) | **Available** |

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
| [`09-relief-wave.obj`](assets/09-relief-wave.obj) | Watertight `80 × 60 × 15 mm` sinusoidal sculpted relief wave (`OBJ` mesh) | `chipcutter generate-sample-step <out.obj> --tutorial 09-3d-surface-finishing` |
| [`09-rotary-shaft.stl`](assets/09-rotary-shaft.stl) | Watertight `Ø40 × 80 mm` shaft with 4 wrench flats, collar groove, & 3-lobe cam | `chipcutter generate-sample-step <out.stl> --tutorial 10-rotary-axial` |
| [`11-indexed-multiface-block.step`](assets/11-indexed-multiface-block.step) | `70 × 32 × 32 mm` prismatic STEP block with cardinal & `45°` angled faces | `chipcutter generate-sample-step <out.step> --tutorial 11-indexed-step` |
| [`10-helical-flute-shaft.stl`](assets/10-helical-flute-shaft.stl) | Watertight `Ø40 × 80 mm` 3-start helical impeller/flute shaft | `chipcutter generate-sample-step <out.stl> --tutorial 12-simultaneous-4axis` |
| [`13-router-panel.dxf`](assets/13-router-panel.dxf) | `160 × 100 mm` multi-layer router front panel (`OUTLINE`, `HOLES`, `POCKETS`, `CUTOUTS`) | `chipcutter generate-sample-step <out.dxf> --tutorial 13-pure2d-dxf` |
| [`14-bracket-v1.step`](assets/14-bracket-v1.step) | `90 × 70 × 20 mm` Revision v1 bracket (blind pocket, `Ø14 mm` bore, 2× `Ø6 mm` holes) | `chipcutter generate-sample-step <out.step> --tutorial 14-reload-v1` |
| [`14-bracket-v2.step`](assets/14-bracket-v2.step) | `92 × 70 × 20 mm` Revision v2 bracket (resized pocket, shifted bore/hole, 1 deleted hole) | `chipcutter generate-sample-step <out.step> --tutorial 14-reload-v2` |
| [`16-yup-bracket.stl`](assets/16-yup-bracket.stl) | Watertight `60 × 40 × 18 mm` sculpted mounting boss exported **Y-up** & off-origin (`STL`) | `chipcutter generate-sample-step <out.stl> --tutorial 16-orient-stl` |
| [`16-tilted-bracket.step`](assets/16-tilted-bracket.step) | `90 × 70 × 20 mm` prismatic bracket tilted `30° X / 25° Z` for Setup WCS & Op-2 flip | `chipcutter generate-sample-step <out.step> --tutorial 16-orient-step` |

You can launch any starter STEP, mesh, or DXF model directly into an empty ChipCutter workspace:

```bash
cargo run --release -p chip-gui -- docs/tutorials/assets/01-facing-billet.step
```

---

## Tutorial Authoring & Screenshot Conventions

All Phase 4 and Phase 6 guides (`00`–`14`) follow a uniform 7-part structure so machinists and contributors can scan any strategy quickly:

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
# Capture screenshots for all 15 tutorials across Phases 4 & 6 (Milestones 13–15 & 22–24):
./scripts/capture_tutorial_screenshots.sh --milestone all

# Capture only a single milestone (13, 14, 15, 22, 23, or 24):
./scripts/capture_tutorial_screenshots.sh --milestone 24

# Or capture a single tutorial in Daylight Light theme:
./scripts/capture_tutorial_screenshots.sh --tutorial 13-pure2d-dxf-svg-workflow --theme light
```
