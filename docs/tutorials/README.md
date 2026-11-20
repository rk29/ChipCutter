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
- **Units & Expressions:** ChipCutter computes internally in millimeters (`mm`), but every numeric field accepts mixed units and parametric variables (for example `1/4 in`, `tool_d * 0.4`, or `model_top_z + 10 mm`). You can toggle display units (`mm` / `in`) at any time from the top-right toolbar.

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
| 08 | [**3D Mesh Surfacing (`STL` / `OBJ` / `3MF`)**](08-3d-mesh-surfacing.md) | Triangle mesh import, 3D Adaptive Roughing, 3D Waterline, 3D Parallel Raster, 3D Constant-Scallop, & 3D Pencil Cleanup | [`assets/08-sculpted-dome.stl`](assets/08-sculpted-dome.stl) (`Ø50 mm` dome on `80×80 mm` pedestal) | **Available** |
| 09 | [**4th-Axis Rotary Wrapping, Spiral Surfacing, & Indexed `3+1`**](09-4th-axis-rotary-wrapping-indexing.md) | Rotary Setup (`A`/`B`/`C`, chuck & clearance), `Z+X+A` 2D cylindrical wrapping (`grbl_rotary_wrap`), continuous rotary spiral, `Indexed 3+1` flats | [`assets/09-rotary-shaft.stl`](assets/09-rotary-shaft.stl) (`Ø40×80 mm` shaft w/ flats, groove, & 3-lobe cam) | **Available** |
| 10 | [**Simultaneous 4-Axis Milling (`X + Y + Z + A/B`)**](10-simultaneous-4axis-milling.md) | Full simultaneous `X+Y+Z+A` off-centerline auto surface-normal tilt smoothing, `G93` inverse-time feeds, cylindrical gouge/holder verification | [`assets/10-helical-flute-shaft.stl`](assets/10-helical-flute-shaft.stl) (`Ø40×80 mm` 3-start helical flute shaft) | **Available** |

---

## Minimal Tutorial STEP Assets (`docs/tutorials/assets/`)

Each 3D operation tutorial starts from an **empty project** and loads a minimal STEP model containing only the geometric features required to practice that strategy—keeping the CAM Browser uncluttered:

| Asset File | Dimensions & Features | Reproducible CLI Generator |
|---|---|---|
| [`01-facing-billet.step`](assets/01-facing-billet.step) | `100 × 80 × 20 mm` plain billet (6 planar faces, zero holes/pockets) | `chipcutter generate-sample-step <out.step> --tutorial 01-facing` |
| [`02-contour-plate.step`](assets/02-contour-plate.step) | `100 × 70 × 12 mm` plate with 1× `R10` fillet & 1× `45°` chamfer | `chipcutter generate-sample-step <out.step> --tutorial 02-contour` |
| [`03-pocket-block.step`](assets/03-pocket-block.step) | `100 × 80 × 20 mm` block with 1 closed `Ø14 mm` island pocket + 1 open step | `chipcutter generate-sample-step <out.step> --tutorial 03-pocket` |
| [`04-drill-bore-plate.step`](assets/04-drill-bore-plate.step) | `100 × 70 × 16 mm` plate with 4× `Ø6 mm` through-holes + 1× `Ø18 mm` blind bore | `chipcutter generate-sample-step <out.step> --tutorial 04-drill-bore` |
| [`05-adaptive-pocket.step`](assets/05-adaptive-pocket.step) | `100 × 80 × 20 mm` block with 1 deep `10 mm` sharp-corner pocket + `Ø16 mm` boss | `chipcutter generate-sample-step <out.step> --tutorial 05-adaptive-rest` |
| [`06-chamfer-engrave-block.step`](assets/06-chamfer-engrave-block.step) | `100 × 70 × 18 mm` block with 1 modeled `45° × 1.5 mm` front bevel & sharp back edge | `chipcutter generate-sample-step <out.step> --tutorial 06-chamfer-trace` |
| [`08-sculpted-dome.stl`](assets/08-sculpted-dome.stl) | Watertight `80 × 80 × 25 mm` sculpted dome on pedestal (`STL` mesh) | `chipcutter generate-sample-step <out.stl> --tutorial 08-mesh-surfacing` |
| [`09-rotary-shaft.stl`](assets/09-rotary-shaft.stl) | Watertight `Ø40 × 80 mm` shaft with 4 wrench flats, collar groove, & 3-lobe cam | `chipcutter generate-sample-step <out.stl> --tutorial 09-rotary-shaft` |
| [`10-helical-flute-shaft.stl`](assets/10-helical-flute-shaft.stl) | Watertight `Ø40 × 80 mm` 3-start helical impeller/flute shaft | `chipcutter generate-sample-step <out.stl> --tutorial 10-simultaneous-4axis` |

You can launch any starter STEP model directly into an empty ChipCutter workspace:

```bash
cargo run --release -p chip-gui -- docs/tutorials/assets/01-facing-billet.step
```

---

## Tutorial Authoring & Screenshot Conventions

All Phase 4 guides follow a uniform 7-part structure so machinists and contributors can scan any strategy quickly:

1. **Goal & What You'll Learn** — Target machining outcome and key takeaways.
2. **Prerequisites & Starter Project** — How to open the exact starter model or sample job.
3. **Step-by-Step GUI Walkthrough** — Numbered steps with exact menu/tab names, keyboard shortcuts, and annotated screenshots.
4. **Key Parameters Reference** — Concise table explaining the strategy's most important fields and safe starting values.
5. **Generate (`Ctrl+G`) & 3D Simulation Verification** — Inspecting toolpaths, Z-dexel machined stock, and the 5-rule crash/gouge gate.
6. **Post-Processing (GRBL & LinuxCNC)** — Reviewing emitted G-code (`ATC M6` vs. split-file-per-tool).
7. **Headless CLI Equivalent (`chipcutter`)** — Copy-pasteable commands that reproduce validation, simulation, and G-code build from the terminal.

### Regenerating Tutorial Screenshots

Screenshots are stored under `docs/images/tutorials/<nn>-<slug>/<step>-<name>.png` and captured deterministically via `chip-gui --self-test-screenshot` under headless Xvfb + Vulkan Lavapipe:

```bash
# Capture screenshots for all Phase 4 tutorials (41 PNGs across Milestones 13–15):
./scripts/capture_tutorial_screenshots.sh --milestone all

# Or capture a single tutorial in Daylight Light theme:
./scripts/capture_tutorial_screenshots.sh --tutorial 05-adaptive-rest-machining --theme light
```
