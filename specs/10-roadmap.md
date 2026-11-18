# 10 — Implementation Roadmap & Horizon

## Phased v1 Milestones (Completed — M1–M5)

ChipCutter v1 delivered a complete offline 3D STEP-to-G-code 2.5D CAM pipeline across five vertical-slice milestones, followed by a pre-v2 workspace modularization and CAD-decoupling refactor:

### Milestone 1 — STEP B-Rep Kernel, Units/Expressions, & 3D `wgpu` Viewport (`plans/01-milestone-1-plan.md`)
- **Math & Unit Foundation (`chip-math`):** Canonical `f64` millimeter geometry, orthonormal `RigidFrame3` frames, exact 2D line/arc polylines (`ArcPolyline2D`), and machinist unit/fraction expression evaluation.
- **STEP B-Rep Kernel (`chip-cad`):** Pure-Rust `monstertruck` ISO 10303-21 STEP importer behind the `CadKernel` trait, analytic surface/curve classification (planes, cylinders, cones, lines, circular arcs), and R-tree BVH 3D ray picking.
- **Desktop Shell & 3D Viewport (`chip-gui`):** `eframe` + `egui_tiles` docking workspace, `4x MSAA` offscreen `wgpu` B-Rep face/edge renderer, turntable/trackball camera presets, ViewCube, and `chipcutter inspect` CLI.

### Milestone 2 — Multi-Setup WCS, Feature Picking, Tool Library, & Undo/Redo (`plans/02-milestone-2-plan.md`)
- **Setups, Stock, & WCS (`chip-cam`):** Multi-setup workflow (`G54`–`G59.3`), box and cylindrical stock sizing, B-Rep face/edge WCS axis alignment (including tilted parts), 3D 27-point WCS origin picking, and Z-height hierarchy resolution.
- **Interactive Feature Extraction:** 3D viewport selection of planar faces, closed edge loops, shortest-path open edge chains, coplanar faces, and same-diameter holes with automatic Z-top / Z-bottom inference.
- **Tool Library & Persistence (`chip-tools`, `chip-project`):** Six cutter geometries, material presets, coupled SFM/chipload calculator, Fusion 360 `.json`/`.tools` importer, deterministic `.chip.toml` (`v1`) persistence, and `< 1 ms` 100-step Undo/Redo.

### Milestone 3 — Parallel Dispatcher & Five Core 2.5D Machining Strategies (`plans/03-milestone-3-plan.md`)
- **Offsetting & Background Execution:** Exact arc-preserving 2D parallel offsetting (`cavalier_contours`), typed `ClEvent` IR, content-addressed `ToolpathCache`, and non-blocking `rayon` `JobDispatcher` with `Esc` cancellation.
- **Core 2.5D Strategies (`chip-cam`):** **Facing** (stock surfacing), **Drilling & Helical Bore Milling** (`G81`/`G83`/`G73` + multi-pitch helical interpolation), and **2D Contour** (helical/linear ramps, tangent lead-in/out arcs, rough/finish passes, and rectangular/triangular holding tabs).
- **Pocketing & Chamfering (`chip-cam`):** **2D Pocket** (closed multi-island region trees, open-shoulder air overhang expansion, contour-parallel & zig-zag clearing) and **Edge Chamfer / 2D Curve Trace** (modeled bevels, sharp CAD edges, and centerline engraving).

### Milestone 4 — Z-Dexel Heightmap Simulation, Crash Verifier, & Post-Processors (`plans/04-milestone-4-plan.md`)
- **Timeline & Cycle-Time Engine (`chip-sim`):** Trapezoidal acceleration motion planner (`G0`/`G1`/`G2`/`G3`), dwell/ATC accounting, timeline scrubbing, and revolved 3D cutter/shank/holder meshes.
- **2.5D Stock Removal & 5-Rule Crash Gate (`chip-sim`):** Multi-threaded Z-dexel stock heightmap sweeper and mandatory physical verification catching rapid-into-stock crashes, target B-Rep gouges, flute-length/shank rubs, holder collisions, and non-center-cutting plunges.
- **Configurable Post-Processors (`chip-post`):** Modal G-code state machine, quadrant arc & peck-drill expansion, ATC (`M6`) vs. split-file-per-tool emission, and built-in **Grbl / FluidNC**, **grblHAL ATC**, and **LinuxCNC (RS274NGC)** `.post.toml` profiles.

### Milestone 5 — Associative STEP Reload, `.chipz` Bundles, & Headless CLI (`plans/05-milestone-5-plan.md`)
- **Associative CAD Revision Re-Matching (`chip-cad`, `chip-project`):** WCS-first `GeometricSignature` fuzzy re-matching across revised STEP files (`Ctrl+R` / `reload-step`), healing open edge chains, marking matched operations `Dirty`, and flagging deleted/ambiguous features for 1-click re-pick.
- **Portable `.chipz` Archives (`chip-project`):** Byte-deterministic, zip-slip-hardened self-contained project bundles packing `project.chip.toml`, STEP models, and custom `.post.toml` profiles with SHA-256 integrity checks.
- **Headless CLI & v1 Hardening (`chip-cli`):** Complete headless `chipcutter` CLI (`inspect`, `validate`, `simulate`, `build`, `reload-step`, `bundle`, `tools`) and byte-for-byte golden G-code regression snapshots.

---

## Phased v2 Milestones (Completed — M6–M9)

ChipCutter v2 extended the engine across four vertical-slice milestones with high-efficiency 2D adaptive roughing, exact 2.5D rest machining, standalone pure-2D vector/sketch/typography workflows, and two-tool V-carving & V-inlays:

### Milestone 6 — 2D Adaptive / Trochoidal Roughing & 2.5D Rest Machining (`plans/06-milestone-6-plan.md`)
- **High-Speed 2D Adaptive Clearing (`chip-cam`, `chip-tools`):** Constant-engagement wavefront / trochoidal pocketing (`optimal_load` `ae`) with medial-axis maximum-inscribed-circle helical entry, open-edge air entry, micro-lift low-clearance linking, and radial chip-thinning feedrate compensation.
- **Exact 2.5D Rest Machining (`chip-cam`, `chip-project`):** Exact 2D boolean rest machining (`RegionSet2D`) supporting both stateless `FromPreviousTool` corner cleanup and multi-operation `FromPrecedingOperations` swept-toolpath subtraction.
- **Topological DAG Wave Execution (`chip-project`, `chip-gui`):** Multi-wave `JobDispatcher` scheduling with automatic downstream `Dirty` invalidation on upstream edits/reorders, Adaptive + Rest Inspector controls, and heightmap engagement/cleanup verification.

### Milestone 7 — `chip-sketch` Foundation, Pure-2D Projects, & DXF / SVG Import (`plans/07-milestone-7-plan.md`)
- **Pure-2D Setups & Schema v2 (`chip-sketch`, `chip-project`):** UI-agnostic `chip-sketch` crate, lossless `.chip.toml` `v1 -> v2` migration, `.chipz` `vectors/*` bundling, and zero-STEP sheet setups with fixed or sketch-relative stock sizing.
- **DXF & SVG Vector Importers (`chip-sketch`):** Pure-Rust AutoCAD DXF and W3C SVG importers with unit/DPI normalization, cubic-Bézier-to-biarc fitting, duplicate removal, R-tree endpoint gap healing, and even-odd outer-loop + island planarization.
- **CLI & Viewport Integration (`chip-cli`, `chip-gui`):** `chipcutter inspect` (`.dxf`/`.svg`) and `import-2d` CLI subcommands, live DXF/SVG Import Modal, 2D sketch `wgpu` rendering, and unified 2D region/loop/chain viewport picking.

### Milestone 8 — Interactive 2D Sketch Primitives & CAM Typography (`plans/08-milestone-8-plan.md`)
- **Parametric 2D Sketch Entities (`chip-sketch`, `chip-cam`):** Expression-driven `Rectangle` (with fillets), `Circle`, `Slot`, `RegularPolygon`, `Polyline`, and `PointPattern` (linear grids & bolt circles), plus associative `SketchAnchor::PlanarFace` sketching on 3D STEP faces that re-binds across STEP reloads.
- **Dual-Mode CAM Typography (`chip-sketch`, `chip-project`):** Pure-Rust TrueType/OpenType outline text (`skrifa` + bundled `DejaVuSans-Bold` + `.chipz` `fonts/*` bundling) with kerning, biarc fitting, and counter-island classification, alongside single-line Hershey engraving stroke fonts.
- **Interactive Viewport Authoring (`chip-gui`):** `2D Sketches` browser tree, orthographic `Look At Sketch` camera alignment, interactive rubber-band drawing tools with a screen-space 2D Snap HUD, and a parametric Sketch Inspector.

### Milestone 9 — V-Carving, Flat-Bottom Clearance, V-Inlays, & v2 Hardening (`plans/09-milestone-9-plan.md`)
- **3D Medial-Axis V-Carving (`chip-tools`, `chip-cam`):** Starter `60°` V-Bit (`T8`), pruned `boostvoronoi` interior medial-axis chain extraction with exact parabolic sampling, simultaneous 3-axis `(X, Y, Z)` V-bit toolpaths, sharp 3D corner climbs, and multi-pass stepdowns.
- **Two-Stage Flat Clearance & Mated V-Inlays (`chip-cam`, `chip-project`):** `flat_depth` clamping with optional Stage 1 Flat Endmill Pocket/Adaptive clearing + Stage 2 rest-machined V-Bit bevels/corners (across both ATC `M6` and split-file GRBL posts), plus mirrored **Male V-Inlay Plug** generation.
- **Analytical Conical Sweeper & v2 Release Hardening (`chip-sim`, `chip-cli`, `chip-gui`):** Closed-form sloped 3D conical V-bit heightmap sweeper, V-Carve & Inlay Inspector UI (`--demo-vcarve`), new v2 golden G-code snapshots alongside byte-identical v1 goldens, and updated release documentation.

---

## Phased v3 Milestones — UI Polish & Usability (Active — M10–M12)

ChipCutter v3 focuses exclusively on **Desktop UI Polish, Visual Consistency, & Machinist Workflow Usability** across `chip-gui` while keeping all core geometry, CAM, simulation, and post-processing crates untouched:

### Milestone 10 — Design System, Dark/Light Themes, Vector Iconography, & Single-Row Chrome (`plans/10-milestone-10-plan.md`)
- **Centralized Design System & Dual Themes (`ThemeTokens`):** Semantic color, spacing, corner-radius, and typography tokens powering both **Studio Dark** and **Daylight High-Contrast Light** themes (including theme-matched 3D `wgpu` viewport background gradients and grid contrast).
- **Procedural Vector Iconography:** Resolution-independent `egui::Painter` vector icons for operation types, status badges (`Clean`, `Dirty`, `Generating`, `Error / Needs Re-Pick`, `Suppressed`), transport controls, and view toggles—eliminating ASCII bracket placeholders (`[OK]`, `[*]`, `^`, `v`).
- **Single-Row Workflow Toolbar & Floating Viewport View Bar:** Decluttered single-row top action toolbar (`File/Import | Sketch & Operations | Generate / Simulate / Post`) paired with a compact floating viewport HUD pill for camera projection, standard views, mouse preset, and layer visibility toggles.

### Milestone 11 — CAM Browser Ergonomics, Two-Column Inspector Grids, & Expression UX (`plans/11-milestone-11-plan.md`)
- **Refined CAM Browser Tree:** Clean single-line operation rows with vector status/strategy icons, `T#` tool + estimated cycle-time badges, hover-revealed quick actions, drag-and-drop + keyboard (`Ctrl+Up`/`Ctrl+Down`) reordering, `Ctrl+D` duplication, `Delete` removal, and deduplicated Setup WCS headers.
- **Two-Column Aligned Inspector Grids:** Standardized `inspector_row` two-column property grids, collapsible section cards, uniform control widths, and contextual inline validation banners across Setup, Sketch, Operation, Tool Library, Simulation, and G-Code panels.
- **Upgraded Parametric Expression Inputs:** Live variable reference/autocomplete popover (`tool_d`, `tool_r`, `stock_x`, `feature_top_z`, etc.), `Up`/`Down` arrow-key nudging, drag-scrubbing on numeric literals, and explicit unit suffix pills.

### Milestone 12 — Interactive Viewport Guidance HUD, Welcome Gallery, Command Palette, & Preferences (`plans/12-milestone-12-plan.md`)
- **Active Pick & Sketch Guidance HUD:** Top-center floating viewport banner whenever a 3D B-Rep pick mode, manual tab placement, or 2D sketch authoring tool is active—displaying step-by-step click instructions, selection counts, and `[Done]` / `[Cancel (Esc)]` actions.
- **Welcome Overlay & Sample Projects Gallery:** Clean Quick-Start welcome overlay on bare `chip-gui` launch (`Import 3D STEP`, `Import 2D DXF / SVG`, `New Pure-2D Sketch`, `Recent Projects`, and a dedicated **Sample Projects Gallery** modal replacing the inline demo buttons previously inside the `Models` tree).
- **Command Palette, Shortcut Cheat-Sheet, & Persistent Preferences:** Fuzzy-searchable **Command Palette (`Ctrl+K`)**, **Keyboard Shortcut Cheat-Sheet (`?` / `F1`)**, and cross-session **`prefs.toml`** persistence for theme, display units, mouse navigation preset, orbit mode, viewport visibility toggles, workspace pane layout, and recent files.

---

## Deferred Horizon (v4 and Beyond)
1. **Functional & Behavioral User-Level Testing Phase (v4):**
   - Durable end-to-end user-journey and GUI behavioral test harness (`egui_kittest` / headless interactive workflows + CLI workflows) verifying every feature from a machinist's point of view without brittle coupling to internal implementation details.
2. **3-Axis Surface Milling & Mesh Support (v4):**
   - Importing **STL / OBJ / 3MF** meshes alongside STEP solids.
   - 3D Waterline (Z-level), Parallel/Raster 3D finishing, Scallop, Pencil, and 3D Adaptive roughing strategies.
3. **Advanced Workholding & Multi-Setup Volumetric Stock (v4):**
   - 3D fixture/vise/clamp collision bodies with automatic safe-rapid routing.
   - Carrying simulated stock across flipped setups via tri-dexel volumetric stock.
4. **Ecosystem & Extensibility (v4+):**
   - Optional OpenCASCADE (`opencascade-rs`) `CadKernel` feature backend.
   - Thread milling (`ThreadMill` single/multi-form cutter geometry, ISO Metric & Unified thread tables, radial arc-in/out helical passes), additional controller dialects (Fanuc/Haas, Mach4, Masso, Centroid), and printable HTML/PDF setup sheets.
