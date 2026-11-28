# 00 — ChipCutter Overview

## Vision
**ChipCutter** (`v0.23.0`) is an offline desktop Computer-Aided Manufacturing (CAM) application (`eframe` + `wgpu`) and headless CLI for **2.5D, 3-axis surface milling, undercut/deburr/thread milling, and 4th-axis rotary CAM + G-code verification**. It bridges 3D parametric **STEP** solids, imported **triangle meshes** (`.stl`, `.obj`, `.3mf`), **2D DXF/SVG vector drawings**, and built-in **2D Sketch/Text layouts** into verified, arc-accurate CNC G-code.

---

## Scope Summary (Phases 1–24 Shipped)
- **Machining Paradigm:** Full 2.5D milling & routing, high-efficiency adaptive roughing, 3-axis surface roughing & finishing, helical thread milling, relieved-neck undercut & 3D edge deburring, and 4th-axis rotary CAM (2-Linear+Rotary surfacing/wrapping, Indexed `3+1` multi-face machining, and Simultaneous 4-axis milling).
- **CAD, Mesh, & 2D Vector Input:**
  - **3D B-Rep Solids (`chip-cad`):** ISO 10303-21 **STEP** (AP203 / AP214 / AP242) via `monstertruck` behind the `CadKernel` trait, with analytic surface/curve recognition and geometric-signature reload re-matching.
  - **3D Triangle Meshes (`chip-cad`):** Unified Binary/ASCII **`.stl`**, **`.obj`**, and **`.3mf`** import (`CadDocument.meshes`), per-model `unit_override`, non-destructive `ModelTransform` (scale/rotate/translate), and `ModelRole::{Part, Fixture}` separation for imported workholding solids.
  - **2D Sketches, Vectors, & Typography (`chip-sketch`):** Standalone pure-2D router/panel projects (`models = []`) and associative 2D Sketch Layers on Setup XY or planar STEP faces, supporting **DXF** & **SVG** imports (with gap-healing & biarc fitting), parametric 2D shapes (rectangles, circles, slots, polygons, polylines, hole patterns), **TrueType/OpenType** outline text, and **Hershey** single-line engraving fonts.
- **Geometry Selection:** Unified interactive 3D/2D picking of planar B-Rep faces, closed/open edge loops, cylindrical holes, modeled chamfers, 2D Sketch regions/loops/chains, and 2D hole patterns, with automatic Z-top / Z-bottom inference and 9 explicit `HeightReference` modes.
- **Setups, WCS, Workholding, & 4th-Axis Rotary:** Multi-setup workflow (`Op1`, `Op2`, …) per project with box/cylinder stock, B-Rep-driven axis alignment & arbitrary Z rotation, WCS origin placement, setup-level safe retract planes, **multi-setup volumetric `TriDexelStock` carryover** (`carryover_from_setup_id`), **3D workholding fixtures & wizards** (`Setup.fixtures`, flip-jig dowel pins, auto soft jaws), and **4th-axis rotary setup kinematics** (`Setup.rotary`).
- **Machining Operations (18 `OperationKind::ALL` Strategies):**
  1. `Facing` — Stock top surfacing
  2. `Contour2D` — 2D profile/contour with stepdowns, lead-in/out, ramps, rough/finish passes, holding tabs, and corner rest machining
  3. `Pocket2D` — Closed & open-edge 2D pocketing (contour-parallel offset, zig-zag raster, and constant-engagement **2D Adaptive / Trochoidal roughing** with micro-lift linking & chip-thinning compensation)
  4. `DrillBore` — Spot, simple, peck, chip-break drilling & helical bore milling from 3D holes or 2D point patterns
  5. `ThreadMill` — Internal & external helical thread milling (single-point and multi-form thread mills)
  6. `SlotUndercut` — Keyway, T-slot, and relieved-neck slot undercut milling
  7. `Dovetail` — Dovetail slot & rail undercut milling
  8. `ChamferTrace` — Modeled/sharp 2D edge chamfering, curve tracing, and single-line Hershey engraving
  9. `VCarve` — Segment-Voronoi medial-axis variable-depth V-carving, sharp 3D corner climbs, flat-depth endmill clearance, and V-inlay plug mode
  10. `Adaptive3D` — 3-axis constant-engagement volumetric adaptive roughing with rest machining & step-up intermediate Z levels
  11. `Waterline3D` — 3-axis Z-level constant-Z contouring & steep-wall roughing/finishing
  12. `Parallel3D` — 3-axis drop-cutter parallel raster surface finishing
  13. `Scallop3D` — 3-axis constant-scallop contour-following surface finishing
  14. `Pencil3D` — 3-axis concave fillet & internal corner pencil cleanup
  15. `Deburr3D` — 3D edge deburring / chamfering (ball, chamfer, and lollipop cutters including underside back-chamfer)
  16. `Undercut3D` — 3-axis spherical lollipop undercut surface milling
  17. `Rotary2Linear` — 4th-axis centerline-locked (`Z+X+A` / `Z+Y+B`) rotary surfacing & cylindrical wrapping
  18. `Rotary4AxisSim` — Full simultaneous 4-axis (`X+Y+Z+A/B`) surface milling with `G93` inverse-time / `G94` angular feedrate control
  - **Cross-Cutting Operation Features:** Exact 2.5D & 3D Rest Machining, engagement-based **Feed Optimization**, **Machining Boundaries** (`MachiningBoundaryConfig` for 2.5D/3D and `RotaryAxialBoundaryConfig` for rotary), and **Per-Operation & Project-Shared Avoid/Keep-Out Regions** (`op.avoid` + `ProjectDocument::avoid_regions`).
- **Tool Libraries & Machine Definitions (`chip-tools`, `chip-machines`):**
  - **Tool Libraries (`chip-tools`):** Global + per-project native **`.tools.toml`** libraries across 10 cutter geometries, canonical `AnalyticCutterEnvelope` & `ToolAssemblyEnvelope`, ISO/ASME thread tables, workpiece material catalog, per-material feeds/speeds presets, inline SFM/chipload calculator, radial chip-thinning formulas, and optional one-way Fusion 360 `.json`/`.tools` import (`fusion-import` feature).
  - **Machine Definitions (`chip-machines`):** Self-contained **`.machine.toml`** profiles capturing XYZ travel envelopes, spindle RPM/power/collet specs, kinematics, ATC/toolsetter capabilities, optional rotary defaults, and dedicated per-machine tool sets—shipping with **7 read-only built-in machines** (`generic`, `milo-1.5`, `milo-2.0`, `voron-cascade`, `carvera`, `carvera-air`, `carvera-z1`) plus user-editable clones backed by shared `PresetStore` storage.
- **Regeneration Model:** Explicit calculation (`Ctrl+G` / "Generate Toolpaths"); editing parameters marks affected (and downstream Rest Machining / carryover) operations **Dirty** until recalculated on background worker threads, paired with instant document-state Undo/Redo.
- **Simulation & Verification (`chip-stock`, `chip-sim`):** Animated 3D toolpath + cutter playback (`G0`/`G1`/`G2`/`G3` + rotary moves), kinematic cycle-time estimation, **3 volumetric stock removal engines** (`Heightfield`, volumetric `TriDexel`, and 4th-axis `Cylindrical`), CPU + optional **WGPU GPU compute** (`gpu-compute` feature), smooth mesh skinning with gouge/excess metrology, and an automated **6-rule verifier** (`RapidIntoStock`, `TargetModelGouge`, `FluteLengthExceeded`, `HolderCollision`, `UnsafeVerticalPlunge`, and `FixtureCollision`).
- **Post-Processing (`chip-post`):** User-editable TOML dialect configs + template blocks over a typed Cutter Location (CL) IR, supporting ATC (`M6`), split-file-per-tool, and `G93`/`G94` rotary emission, shipping with **6 built-in profiles** (`grbl`, `grblhal_atc`, `linuxcnc`, `linuxcnc_4axis`, `grblhal_4axis`, `grbl_rotary_wrap`) plus user-config clones.
- **Platforms & Persistence (`chip-project`, `chip-cli`):** Native Linux, macOS, and Windows GUI + headless CLI; human-readable `.chip.toml` (`schema_version = 3`, auto-migrating v1/v2) project files with portable `.chipz` bundle export and geometric signature re-matching on STEP reload.

---

## Explicit Non-Goals (Deferred to Future Releases)
- **No nonlinear 2D CAD constraint solver:** Built-in 2D Sketch tools use direct parametric expressions and viewport snapping rather than a parametric CAD sketch solver.
- **No machine control / G-code streaming or jogging:** ChipCutter is pure offline CAM; jogging and live G-code streaming stay in dedicated senders (LinuxCNC, gSender, FluidNC UI, etc.).
- **No in-machine `G38.2` probing routines:** ChipCutter is pure offline CAM and does not directly interface with a live machine; probing cycles belong to dedicated controller or sender macros.
- **No standalone printable HTML/PDF setup sheets yet:** Operator summary metadata is emitted directly as structured G-code header comments.
- **No WebAssembly/browser target:** Native desktop + headless CLI only.
- **No copyleft dependencies:** Third-party crates must use permissive or weak-copyleft licenses (`MIT`, `Apache-2.0`, `BSD`, `MPL-2.0`, `BSL-1.0`, `Zlib`, `ISC`).

---

## Specification Index
| Document | Scope |
|---|---|
| [00-overview.md](00-overview.md) | Vision, shipped `v0.23.0` (Phases 1–24) scope, non-goals, and spec map |
| [01-architecture.md](01-architecture.md) | Cargo workspace crates, data pipeline, units/expressions, DAG threading, licensing |
| [02-cad-selection.md](02-cad-selection.md) | `CadKernel` trait, STEP & triangle mesh pipelines, `ModelTransform`, unified B-Rep + Sketch picking, reload re-matching |
| [03-setups-wcs.md](03-setups-wcs.md) | Multi-setup model, stock & carryover, 4th-axis rotary setup kinematics, 3D workholding fixtures & wizards, WCS, Z planes |
| [04-tool-library.md](04-tool-library.md) | 10 cutter geometries, `.tools.toml` presets, SFM/chipload & chip-thinning math, Fusion 360 import, `.machine.toml` Machine Definitions |
| [05-operations.md](05-operations.md) | Operation lifecycle, all 18 2.5D/3-axis/undercut/4th-axis strategies, rest machining, boundaries, avoid regions, feed optimization |
| [06-simulation.md](06-simulation.md) | Toolpath playback, cycle time model, Heightfield / TriDexel / Cylindrical stock engines, GPU compute, 6-rule verifier |
| [07-post-processor.md](07-post-processor.md) | Typed CL-data IR, TOML post configs, ATC vs split-per-tool, all 6 built-in GRBL/grblHAL/LinuxCNC 3- & 4-axis posts |
| [08-ui-viewport.md](08-ui-viewport.md) | `eframe` + `wgpu` + `egui_tiles` shell, Dark/Light design system, Viewport HUDs, Command Palette, Undo/Redo |
| [09-persistence-cli.md](09-persistence-cli.md) | `.chip.toml` (`schema_version = 3`, auto-migrating v1/v2) & `.chipz` formats, headless CLI subcommands, testing strategy |
| [10-roadmap.md](10-roadmap.md) | Shipped milestones and future roadmap |
| [11-sketches-vectors.md](11-sketches-vectors.md) | `chip-sketch` 2D Sketch Layers, parametric shapes, DXF/SVG importers, TrueType & Hershey text |
