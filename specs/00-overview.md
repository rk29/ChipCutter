# 00 — ChipCutter Overview

## Vision
**ChipCutter** is a desktop Computer-Aided Manufacturing (CAM) application and headless CLI written in **Rust** with an **egui** (`eframe` + `wgpu`) user interface. It bridges 3D parametric CAD models and CNC mills/routers by turning imported **STEP** solids into verified, arc-accurate G-code through fast interactive 3D feature selection.

---

## v1 Scope Summary
- **Machining Paradigm:** 2.5D CNC milling & routing (architected cleanly to extend to 3-axis surface milling in v2+).
- **CAD Input:** 3D B-Rep solids in **STEP** format (AP203 / AP214 / AP242) via pure-Rust `monstertruck` (`monstertruck-io` + `monstertruck-meshing`, the maintained continuation of `truck`) behind a kernel abstraction trait.
- **Geometry Selection:** Interactive 3D picking of planar faces, closed/open edge loops, and cylindrical holes directly on the B-Rep model, with automatic Z-top / Z-bottom inference and manual overrides.
- **Setups & WCS:** Multi-setup workflow (`Op1`, `Op2`, …) per project with box/cylinder stock, B-Rep-driven axis alignment, WCS origin placement, and setup-level safe retract planes.
- **v1 Operations:**
  1. **Facing** (stock top surfacing)
  2. **2D Contour / Profile** (stepdowns, lead-in/out, ramps, rough/finish passes, rectangular & ramped holding tabs)
  3. **2D Pocket** (closed pockets with island avoidance + open-edge pockets; arc-preserving contour-parallel offset & zig-zag clearing)
  4. **Drilling & Helical Bore Milling** (spot, simple drill, peck drill, helical interpolation)
  5. **Edge Chamfering & 2D Curve Trace/Engraving**
- **Tool Library:** Global + per-project TOML/JSON libraries with per-material feeds/speeds presets, inline SFM/chipload calculator, and Fusion 360 `.json` tool import.
- **Regeneration Model:** Explicit calculation (`Ctrl+G` / "Generate Toolpaths"); editing parameters marks affected operations **Dirty** until recalculated on background worker threads, paired with instant document-state Undo/Redo.
- **Simulation & Verification:** Animated 3D toolpath + cutter playback (`G0`/`G1`/`G2`/`G3`), machining time estimation, and 2.5D heightmap stock material removal with rapid-into-stock crash and model-gouge detection.
- **Post-Processing:** User-editable TOML dialect configs + template blocks over a typed Cutter Location (CL) IR, supporting ATC (`M6`) and split-file-per-tool workflows, shipping with **GRBL / grblHAL / FluidNC** and **LinuxCNC (RS274NGC)** profiles.
- **Platforms & Persistence:** Native Linux, macOS, and Windows GUI + headless CLI; human-readable `.chip.toml` project files with portable `.chipz` bundle export and geometric signature re-matching on STEP reload.

---

## Explicit v1 Non-Goals (Deferred to v2+)
- **No mesh inputs in v1:** STL, OBJ, and 3MF triangle meshes are out of scope.
- **No 2D CAD or 2D vector imports in v1:** Built-in 2D sketch drawing, text layout, and DXF/SVG file imports are deferred to v2.
- **No Adaptive/Trochoidal clearing in v1:** Strategy traits are designed for it, but v1 ships with contour-parallel offset and zig-zag clearing.
- **No machine control / G-code streaming:** ChipCutter is pure offline CAM; jogging, probing, and streaming stay in dedicated senders (LinuxCNC, gSender, FluidNC UI, etc.).
- **No standalone printable setup sheets in v1:** Operator summary metadata (stock, WCS zero, required tools, estimated times) is emitted directly as structured G-code header comments.
- **No 3D fixture/clamp collision bodies in v1:** Workholding clearance uses Setup-level Safe/Clearance Z planes and contour holding tabs.
- **No WebAssembly/browser target:** Native desktop + headless CLI only.
- **No copyleft dependencies:** Third-party crates must use permissive or weak-copyleft licenses (`MIT`, `Apache-2.0`, `BSD`, `MPL-2.0`, `Zlib`, `ISC`).

---

## Specification Index
| Document | Scope |
|---|---|
| [00-overview.md](file:///workspace/ChipCutter/specs/00-overview.md) | Vision, v1 boundaries, non-goals, and spec map |
| [01-architecture.md](file:///workspace/ChipCutter/specs/01-architecture.md) | Cargo workspace crates, data pipeline, units/expressions, threading, licensing |
| [02-cad-selection.md](file:///workspace/ChipCutter/specs/02-cad-selection.md) | `CadKernel` trait, `truck` STEP pipeline, B-Rep picking, geometric reload re-matching |
| [03-setups-wcs.md](file:///workspace/ChipCutter/specs/03-setups-wcs.md) | Multi-setup model, stock geometry, WCS orientation/origin, Z clearance planes |
| [04-tool-library.md](file:///workspace/ChipCutter/specs/04-tool-library.md) | Cutter types, material presets, SFM/chipload math, Fusion 360 JSON import |
| [05-operations.md](file:///workspace/ChipCutter/specs/05-operations.md) | Operation lifecycle, linking/ramps/tabs, and the 5 v1 2.5D toolpath algorithms |
| [06-simulation.md](file:///workspace/ChipCutter/specs/06-simulation.md) | Toolpath playback, cycle time model, heightmap stock removal, crash/gouge checks |
| [07-post-processor.md](file:///workspace/ChipCutter/specs/07-post-processor.md) | Typed CL-data IR, TOML post configs, ATC vs split-per-tool, GRBL & LinuxCNC posts |
| [08-ui-viewport.md](file:///workspace/ChipCutter/specs/08-ui-viewport.md) | `eframe` + `wgpu` + `egui_tiles` workspace, CAD render passes, camera presets, Undo/Redo |
| [09-persistence-cli.md](file:///workspace/ChipCutter/specs/09-persistence-cli.md) | `.chip.toml` & `.chipz` formats, headless CLI subcommands, testing & golden verification |
| [10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md) | Phased v1 engineering milestones and v2+ expansion roadmap |
