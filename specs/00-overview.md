# 00 — ChipCutter Overview

## Vision
**ChipCutter** is an offline desktop Computer-Aided Manufacturing (CAM) application (`eframe` + `wgpu`) and headless CLI. It bridges 3D parametric **STEP** models, **2D DXF/SVG vector drawings**, and built-in **2D Sketch/Text layouts** into verified, arc-accurate CNC milling and routing G-code.

---

## Scope Summary (v1 & v2 Completed + v3 UI Polish Active)
- **Machining Paradigm:** 2.5D CNC milling, high-efficiency adaptive roughing, routing, engraving, and V-carving (architected cleanly to extend to 3-axis surface milling in v4+).
- **CAD & 2D Vector Input:**
  - **3D B-Rep Solids (`chip-cad`):** ISO 10303-21 **STEP** (AP203 / AP214 / AP242) via `monstertruck` behind the `CadKernel` trait.
  - **2D Sketches, Vectors, & Typography (`chip-sketch` — *v2*):** Standalone pure-2D router/panel projects (no STEP required) AND associative 2D Sketch Layers on Setup XY or planar STEP faces, supporting **DXF** & **SVG** imports (with gap-healing & biarc fitting), parametric 2D shapes (rectangles, circles, slots, polygons, polylines, hole patterns), **TrueType/OpenType** outline text, and **Hershey** single-line engraving fonts.
- **Geometry Selection:** Unified interactive 3D/2D picking of planar B-Rep faces, closed/open edge loops, cylindrical holes, 2D Sketch regions/loops/chains, and 2D hole patterns, with automatic Z-top / Z-bottom inference and manual overrides.
- **Setups & WCS:** Multi-setup workflow (`Op1`, `Op2`, …) per project with box/cylinder stock (from 3D model bounds, 2D sketch bounds, or fixed dimensions), B-Rep-driven axis alignment, WCS origin placement, and setup-level safe retract planes.
- **Machining Operations:**
  1. **Facing** (stock top surfacing)
  2. **2D Contour / Profile** (stepdowns, lead-in/out, ramps, rough/finish passes, rectangular & ramped holding tabs, corner rest machining)
  3. **2D Pocket & 2D Adaptive Clearing** (closed pockets with islands + open-edge pockets; contour-parallel offset, zig-zag raster, and *v2* constant-engagement **Adaptive / Trochoidal roughing** with micro-lift linking & chip-thinning compensation)
  4. **Drilling & Helical Bore Milling** (spot, simple drill, peck drill, helical interpolation from 3D holes or 2D point patterns)
  5. **Edge Chamfering & 2D Curve Trace / Engraving** (modeled/sharp chamfers, curve tracing, and single-line Hershey text engraving)
  6. **V-Carving & Bevel Inlay (*v2*)** (segment-Voronoi medial-axis variable-depth V-bit carving, sharp 3D corner climbs, flat-bottom max depth with optional Flat Endmill area clearance, and V-inlay plug mode)
  - **2.5D Rest Machining (*v2*):** Exact 2D boolean rest machining (`FromPreviousTool` and `FromPrecedingOperations`) across Adaptive, Pocket, Contour, and V-Carve.
- **Tool Library:** Global + per-project TOML/JSON libraries with per-material feeds/speeds presets, inline SFM/chipload calculator, radial chip-thinning formulas, and Fusion 360 `.json` tool import.
- **Regeneration Model:** Explicit calculation (`Ctrl+G` / "Generate Toolpaths"); editing parameters marks affected (and downstream Rest Machining) operations **Dirty** until recalculated on background worker threads, paired with instant document-state Undo/Redo.
- **Simulation & Verification:** Animated 3D toolpath + cutter playback (`G0`/`G1`/`G2`/`G3`), machining time estimation, and 2.5D heightmap stock material removal with rapid-into-stock crash and model-gouge detection.
- **Post-Processing:** User-editable TOML dialect configs + template blocks over a typed Cutter Location (CL) IR, supporting ATC (`M6`) and split-file-per-tool workflows, shipping with **GRBL / grblHAL / FluidNC** and **LinuxCNC (RS274NGC)** profiles.
- **Platforms & Persistence:** Native Linux, macOS, and Windows GUI + headless CLI; human-readable `.chip.toml` project files with portable `.chipz` bundle export and geometric signature re-matching on STEP reload.

---

## Explicit Non-Goals (Deferred to v4+)
- **No 3D triangle mesh inputs or 3-axis surfacing in v3:** STL, OBJ, and 3MF meshes plus 3D Waterline/Scallop/Raster surfacing are deferred to v4.
- **No nonlinear 2D CAD constraint solver:** Built-in 2D Sketch tools use direct parametric expressions and viewport snapping rather than a parametric CAD sketch solver.
- **No machine control / G-code streaming:** ChipCutter is pure offline CAM; jogging, probing, and streaming stay in dedicated senders (LinuxCNC, gSender, FluidNC UI, etc.).
- **No standalone printable HTML/PDF setup sheets in v3:** Operator summary metadata is emitted directly as structured G-code header comments.
- **No 3D fixture/clamp collision bodies, thread milling, or tri-dexel flip stock in v3:** Workholding clearance uses Setup-level Safe/Clearance Z planes and contour holding tabs; thread milling and volumetric multi-setup stock carry-over are deferred to v4.
- **No WebAssembly/browser target:** Native desktop + headless CLI only.
- **No copyleft dependencies:** Third-party crates must use permissive or weak-copyleft licenses (`MIT`, `Apache-2.0`, `BSD`, `MPL-2.0`, `BSL-1.0`, `Zlib`, `ISC`).

---

## Specification Index
| Document | Scope |
|---|---|
| [00-overview.md](file:///workspace/ChipCutter/specs/00-overview.md) | Vision, v1–v3 scope boundaries, non-goals, and spec map |
| [01-architecture.md](file:///workspace/ChipCutter/specs/01-architecture.md) | Cargo workspace crates, data pipeline, units/expressions, DAG threading, licensing |
| [02-cad-selection.md](file:///workspace/ChipCutter/specs/02-cad-selection.md) | `CadKernel` trait, STEP pipeline, unified B-Rep + Sketch picking, reload re-matching |
| [03-setups-wcs.md](file:///workspace/ChipCutter/specs/03-setups-wcs.md) | Multi-setup model, pure-2D & 3D stock geometry, WCS orientation/origin, Z planes |
| [04-tool-library.md](file:///workspace/ChipCutter/specs/04-tool-library.md) | Cutter types, material presets, SFM/chipload & chip-thinning math, Fusion 360 import |
| [05-operations.md](file:///workspace/ChipCutter/specs/05-operations.md) | Operation lifecycle, Adaptive roughing, 2.5D Rest Machining, V-Carving, and v1 strategies |
| [06-simulation.md](file:///workspace/ChipCutter/specs/06-simulation.md) | Toolpath playback, cycle time model, heightmap stock removal, crash/gouge checks |
| [07-post-processor.md](file:///workspace/ChipCutter/specs/07-post-processor.md) | Typed CL-data IR, TOML post configs, ATC vs split-per-tool, GRBL & LinuxCNC posts |
| [08-ui-viewport.md](file:///workspace/ChipCutter/specs/08-ui-viewport.md) | `eframe` + `wgpu` + `egui_tiles` shell, Dark/Light design system, Viewport HUDs, Command Palette, Undo/Redo |
| [09-persistence-cli.md](file:///workspace/ChipCutter/specs/09-persistence-cli.md) | `.chip.toml` (`v2`) & `.chipz` formats, headless CLI subcommands, testing strategy |
| [10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md) | Completed v1 (M1–M5) & v2 (M6–M9) milestones, active v3 UI Polish milestones (M10–M12), and v4+ horizon |
| [11-sketches-vectors.md](file:///workspace/ChipCutter/specs/11-sketches-vectors.md) | `chip-sketch` 2D Sketch Layers, parametric shapes, DXF/SVG importers, TrueType & Hershey text |
