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

| # | Tutorial | Strategies & Concepts Covered | Starter Sample / Asset | Status |
|---|---|---|---|---|
| 00 | [**Getting Started & Workspace Orientation**](00-getting-started.md) | Welcome Overlay, Viewport Navigation, Setup WCS & Stock, Tool Library, `Ctrl+G`, 3D Simulation, & Post-Processing | `Prismatic Multi-Feature Bracket` (Sample Gallery) | **Available** |
| 01 | Facing (Stock Surfacing) | Stock top allowance, WCS origin, face/flat mills, pass extension, stepover, zig-zag vs. one-way cutting | `CAM Torture Block` / `Multi-Feature Bracket` | Planned (Milestone 14) |
| 02 | 2D Contour / Profile & Holding Tabs | Closed outer profiles & open edge chains, stepdowns, helical/linear ramps, lead-in/out arcs, rough/finish passes, holding tabs | `Pure-2D Router Panel` & `CAM Torture Block` | Planned (Milestone 14) |
| 03 | 2D Pocket (Multi-Island & Open-Shoulder) | Closed multi-island cavities, open-shoulder overhang air entry, contour-parallel vs. zig-zag, radial/axial stock-to-leave | `CAM Torture Block` | Planned (Milestone 14) |
| 04 | Drilling & Helical Bore Milling | 3D cylindrical holes & 2D bolt circles, spot/peck cycles (`G81`/`G83`/`G73`), helical bore interpolation + floor cleanup | `CAM Torture Block` & `Engraved Nameplate` | Planned (Milestone 14) |
| 05 | 2D Adaptive Clearing & 2.5D Rest Machining | Constant-engagement trochoidal roughing (`optimal_load`), radial chip-thinning, `FromPrecedingOperations` DAG waves | `2D Adaptive Clearing + Rest Machining` | Planned (Milestone 15) |
| 06 | Edge Chamfering & 2D Curve Trace / Engraving | Sharp unmodeled CAD edges vs. modeled `45°` bevel faces (`tip_offset`), 1-pass single-line Hershey text engraving | `CAM Torture Block` & `Engraved Nameplate` | Planned (Milestone 15) |
| 07 | 3D V-Carving, Two-Tool Flat Clearance, & Mated V-Inlays | TrueType/SVG contours, medial-axis 3D corner climbs, Stage 1 Flat + Stage 2 V-Bit clearance, mirrored Male V-Inlay plugs | `V-Carve Sign & Mated Male V-Inlay` | Planned (Milestone 15) |

---

## Starter Models & Reproducible Assets

Every tutorial uses either a built-in **Sample Projects Gallery** entry (accessible from the Welcome Overlay, `Models → Sample Gallery…`, or `Help → Sample Projects Gallery…`) or a deterministic model written by `chipcutter generate-sample-step`:

| Built-In Sample (`Sample Projects Gallery`) | CLI `--sample` Slug | Primary Tutorials |
|---|---|---|
| **Prismatic Multi-Feature Bracket** | `multi-feature` | `00` (Getting Started), `01` (Facing) |
| **CAM Torture Block (8 Operations)** | `cam-torture` | `01` (Facing), `02` (Contour), `03` (Pocket), `04` (Drill/Bore), `06` (Chamfer) |
| **Pure-2D CNC Router Front Panel (.dxf)** | `pure-2d` | `02` (2D Contour & Holding Tabs) |
| **Engraved Nameplate (TrueType + Hershey)** | `nameplate` | `04` (2D Bolt Circles), `06` (Curve Trace / Hershey Engraving) |
| **2D Adaptive Clearing + Rest Machining** | `adaptive-rest` | `05` (Adaptive Clearing & 2.5D Rest Machining) |
| **V-Carve Sign & Mated Male V-Inlay** | `vcarve` | `07` (3D V-Carving, Flat Clearance, & V-Inlays) |

You can also launch any sample directly from the command line:

```bash
cargo run --release -p chip-gui -- --sample multi-feature
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
# Capture screenshots for Milestone 13 (00-getting-started):
./scripts/capture_tutorial_screenshots.sh --milestone 13

# Or capture a single tutorial in Daylight Light theme:
./scripts/capture_tutorial_screenshots.sh --tutorial 00-getting-started --theme light
```
