# 10 — Implementation Roadmap & Future Phases

## Phase 1 — Core 2.5D STEP-to-GCode Engine (Completed — M1–M5)

Phase 1 delivered a complete offline 3D STEP-to-G-code 2.5D CAM pipeline across five vertical-slice milestones, followed by a workspace modularization and CAD-decoupling refactor:

### Milestone 1 — STEP B-Rep Kernel, Units/Expressions, & 3D `wgpu` Viewport (`plans/01-milestone-1-plan.md`)
- **Math & Unit Foundation (`chip-math`):** Canonical `f64` millimeter geometry, orthonormal `RigidFrame3` frames, exact 2D line/arc polylines (`ArcPolyline2D`), and machinist unit/fraction expression evaluation.
- **STEP B-Rep Kernel (`chip-cad`):** `monstertruck` ISO 10303-21 STEP importer behind the `CadKernel` trait, analytic surface/curve classification (planes, cylinders, cones, lines, circular arcs), and R-tree BVH 3D ray picking.
- **Desktop Shell & 3D Viewport (`chip-gui`):** `eframe` + `egui_tiles` docking workspace, `4x MSAA` offscreen `wgpu` B-Rep face/edge renderer, turntable/trackball camera presets, ViewCube, and `chipcutter inspect` CLI.

### Milestone 2 — Multi-Setup WCS, Feature Picking, Tool Library, & Undo/Redo (`plans/02-milestone-2-plan.md`)
- **Setups, Stock, & WCS (`chip-cam`):** Multi-setup workflow (`G54`–`G59.3`), box and cylindrical stock sizing, B-Rep face/edge WCS axis alignment (including tilted parts), 3D 27-point WCS origin picking, and Z-height hierarchy resolution.
- **Interactive Feature Extraction:** 3D viewport selection of planar faces, closed edge loops, shortest-path open edge chains, coplanar faces, and same-diameter holes with automatic Z-top / Z-bottom inference.
- **Tool Library & Persistence (`chip-tools`, `chip-project`):** Six cutter geometries, material presets, coupled SFM/chipload calculator, Fusion 360 `.json`/`.tools` importer, deterministic `.chip.toml` (`schema_version = 1`) persistence, and `< 1 ms` 100-step Undo/Redo.

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
- **Headless CLI & Phase 1 Hardening (`chip-cli`):** Complete headless `chipcutter` CLI (`inspect`, `validate`, `simulate`, `build`, `reload-step`, `bundle`, `tools`) and byte-for-byte golden G-code regression snapshots.

---

## Phase 2 — Adaptive Roughing, Rest Machining, 2D Sketches/Vectors, Typography, & V-Carving (Completed — M6–M9)

Phase 2 extended the engine across four vertical-slice milestones with high-efficiency 2D adaptive roughing, exact 2.5D rest machining, standalone pure-2D vector/sketch/typography workflows, and two-tool V-carving & V-inlays:

### Milestone 6 — 2D Adaptive / Trochoidal Roughing & 2.5D Rest Machining (`plans/06-milestone-6-plan.md`)
- **High-Speed 2D Adaptive Clearing (`chip-cam`, `chip-tools`):** Constant-engagement wavefront / trochoidal pocketing (`optimal_load` `ae`) with medial-axis maximum-inscribed-circle helical entry, open-edge air entry, micro-lift low-clearance linking, and radial chip-thinning feedrate compensation.
- **Exact 2.5D Rest Machining (`chip-cam`, `chip-project`):** Exact 2D boolean rest machining (`RegionSet2D`) supporting both stateless `FromPreviousTool` corner cleanup and multi-operation `FromPrecedingOperations` swept-toolpath subtraction.
- **Topological DAG Wave Execution (`chip-project`, `chip-gui`):** Multi-wave `JobDispatcher` scheduling with automatic downstream `Dirty` invalidation on upstream edits/reorders, Adaptive + Rest Inspector controls, and heightmap engagement/cleanup verification.

### Milestone 7 — `chip-sketch` Foundation, Pure-2D Projects, & DXF / SVG Import (`plans/07-milestone-7-plan.md`)
- **Pure-2D Setups & Schema v2 (`chip-sketch`, `chip-project`):** UI-agnostic `chip-sketch` crate, lossless `.chip.toml` `schema_version = 1 -> 2` migration, `.chipz` `vectors/*` bundling, and zero-STEP sheet setups with fixed or sketch-relative stock sizing.
- **DXF & SVG Vector Importers (`chip-sketch`):** AutoCAD DXF and W3C SVG importers with unit/DPI normalization, cubic-Bézier-to-biarc fitting, duplicate removal, R-tree endpoint gap healing, and even-odd outer-loop + island planarization.
- **CLI & Viewport Integration (`chip-cli`, `chip-gui`):** `chipcutter inspect` (`.dxf`/`.svg`) and `import-2d` CLI subcommands, live DXF/SVG Import Modal, 2D sketch `wgpu` rendering, and unified 2D region/loop/chain viewport picking.

### Milestone 8 — Interactive 2D Sketch Primitives & CAM Typography (`plans/08-milestone-8-plan.md`)
- **Parametric 2D Sketch Entities (`chip-sketch`, `chip-cam`):** Expression-driven `Rectangle` (with fillets), `Circle`, `Slot`, `RegularPolygon`, `Polyline`, and `PointPattern` (linear grids & bolt circles), plus associative `SketchAnchor::PlanarFace` sketching on 3D STEP faces that re-binds across STEP reloads.
- **Dual-Mode CAM Typography (`chip-sketch`, `chip-project`):** TrueType/OpenType outline text (`skrifa` + bundled `DejaVuSans-Bold` + `.chipz` `fonts/*` bundling) with kerning, biarc fitting, and counter-island classification, alongside single-line Hershey engraving stroke fonts.
- **Interactive Viewport Authoring (`chip-gui`):** `2D Sketches` browser tree, orthographic `Look At Sketch` camera alignment, interactive rubber-band drawing tools with a screen-space 2D Snap HUD, and a parametric Sketch Inspector.

### Milestone 9 — V-Carving, Flat-Bottom Clearance, V-Inlays, & Phase 2 Hardening (`plans/09-milestone-9-plan.md`)
- **3D Medial-Axis V-Carving (`chip-tools`, `chip-cam`):** Starter `60°` V-Bit (`T8`), pruned `boostvoronoi` interior medial-axis chain extraction with exact parabolic sampling, simultaneous 3-axis `(X, Y, Z)` V-bit toolpaths, sharp 3D corner climbs, and multi-pass stepdowns.
- **Two-Stage Flat Clearance & Mated V-Inlays (`chip-cam`, `chip-project`):** `flat_depth` clamping with optional Stage 1 Flat Endmill Pocket/Adaptive clearing + Stage 2 rest-machined V-Bit bevels/corners (across both ATC `M6` and split-file GRBL posts), plus mirrored **Male V-Inlay Plug** generation.
- **Analytical Conical Sweeper & Release Hardening (`chip-sim`, `chip-cli`, `chip-gui`):** Closed-form sloped 3D conical V-bit heightmap sweeper, V-Carve & Inlay Inspector UI (`--demo-vcarve`), Phase 2 golden G-code snapshots alongside byte-identical Phase 1 goldens, and updated documentation.

---

## Phase 3 — UI Polish & Usability (Completed — M10–M12)

Phase 3 focused exclusively on **Desktop UI Polish, Visual Consistency, & Machinist Workflow Usability** across `chip-gui` while keeping all core geometry, CAM, simulation, and post-processing crates untouched:

### Milestone 10 — Design System, Dark/Light Themes, Vector Iconography, & Single-Row Chrome (`plans/10-milestone-10-plan.md`)
- **Centralized Design System & Dual Themes (`ThemeTokens`):** Semantic color, spacing, corner-radius, and typography tokens powering both **Studio Dark** and **Daylight High-Contrast Light** themes (including theme-matched 3D `wgpu` viewport background gradients and grid contrast).
- **Procedural Vector Iconography:** Resolution-independent `egui::Painter` vector icons for operation types, status badges (`Clean`, `Dirty`, `Generating`, `Error / Needs Re-Pick`, `Suppressed`), transport controls, and view toggles—eliminating ASCII bracket placeholders (`[OK]`, `[*]`, `^`, `v`).
- **Single-Row Workflow Toolbar & Floating Viewport View Bar:** Decluttered single-row top action toolbar (`File/Import | Sketch & Operations | Generate / Simulate / Post`) paired with a compact floating viewport HUD pill for camera projection, standard views, mouse preset, and layer visibility toggles.

### Milestone 11 — CAM Browser Ergonomics, Two-Column Inspector Grids, & Expression UX (`plans/11-milestone-11-plan.md`)
- **Refined CAM Browser Tree:** Clean single-line operation rows with vector status/strategy icons, `T#` tool + estimated cycle-time badges, hover-revealed quick actions, drag-and-drop + keyboard (`Ctrl+Up`/`Ctrl+Down`) reordering, `Ctrl+D` duplication, `Delete` removal, and deduplicated Setup WCS headers.
- **Two-Column Aligned Inspector Grids:** Standardized two-column property grids, collapsible section cards, uniform control widths, and contextual inline validation banners across Setup, Sketch, Operation, Tool Library, Simulation, and G-Code panels.
- **Upgraded Parametric Expression Inputs:** Live variable reference/autocomplete popover (`tool_d`, `tool_r`, `stock_x`, `feature_top_z`, etc.), `Up`/`Down` arrow-key nudging, drag-scrubbing on numeric literals, and explicit unit suffix pills.

### Milestone 12 — Interactive Viewport Guidance HUD, Welcome Gallery, Command Palette, & Preferences (`plans/12-milestone-12-plan.md`)
- **Active Pick & Sketch Guidance HUD:** Top-center floating viewport banner whenever a 3D B-Rep pick mode, manual tab placement, or 2D sketch authoring tool is active—displaying step-by-step click instructions, selection counts, and `[Done]` / `[Cancel (Esc)]` actions.
- **Welcome Overlay & Sample Projects Gallery:** Clean Quick-Start welcome overlay on bare `chip-gui` launch (`Import 3D STEP`, `Import 2D DXF / SVG`, `New Pure-2D Sketch`, `Recent Projects`, and a dedicated **Sample Projects Gallery** modal replacing the inline demo buttons previously inside the `Models` tree).
- **Command Palette, Shortcut Cheat-Sheet, & Persistent Preferences:** Fuzzy-searchable **Command Palette (`Ctrl+K`)**, **Keyboard Shortcut Cheat-Sheet (`?` / `F1`)**, and cross-session **`prefs.toml`** persistence for theme, display units, mouse navigation preset, orbit mode, viewport visibility toggles, workspace pane layout, and recent files.

---

## Phase 4 — Illustrated Toolpath Strategy Tutorials (Completed — M13–M15)

Phase 4 delivered a complete set of illustrated Markdown tutorials in `docs/tutorials/` (with step-by-step instructions, minimal single-purpose `.step` starter assets in `docs/tutorials/assets/`, and 41 repeatable headless screenshots in `docs/images/tutorials/`) walking users end-to-end through setup, geometry selection, parameters, toolpath generation (`Ctrl+G`), 3D heightmap simulation, and verified G-code post-processing for every ChipCutter machining strategy—without modifying core CAM, simulation, post-processing, or UI architecture:

### Milestone 13 — Tutorial Framework, Screenshot Capture Pipeline, & Getting Started Guide (`plans/13-milestone-13-plan.md`)
- **Tutorials Index & Standardized Structure (`docs/tutorials/README.md`):** Establish the tutorials index, learning progression, and consistent per-guide layout (Goal & Result Preview, Starter Model/Project, Step-by-Step GUI Walkthrough with parameter tables, `Ctrl+G` & 3D Simulation checks, GRBL/LinuxCNC Post-Processing, and equivalent `chipcutter` CLI commands).
- **Reproducible Starter Assets & Headless Screenshot Pipeline (`docs/images/tutorials/`, `chip-gui`):** Map each tutorial to a reproducible built-in sample or generated starter model and provide repeatable headless Vulkan/Xvfb screenshot capture helpers (building on `chip-gui --self-test-screenshot`) so viewport, inspector, toolpath, and simulated-stock screenshots stay sharp and consistent.
- **Getting Started & Workspace Orientation (`docs/tutorials/00-getting-started.md`):** Illustrated introductory guide covering launching `chip-gui`, 3D/2D viewport navigation & camera presets, importing STEP/DXF/SVG files, Setup WCS & stock sizing, Tool Library basics, and the Generate (`Ctrl+G`) → Simulate → Post workflow.

### Milestone 14 — Core 2.5D Milling Tutorials: Facing, 2D Contour, 2D Pocket, & Drilling/Boring (`plans/14-milestone-14-plan.md`)
- **Facing (`docs/tutorials/01-facing.md`):** Illustrated step-by-step guide to stock top surfacing—stock allowance & WCS origin, face/flat mill selection, pass extension, stepover, zig-zag vs. one-way cutting, 3D simulation check, and G-code inspection.
- **2D Contour / Profile & Holding Tabs (`docs/tutorials/02-2d-contour-tabs.md`):** Illustrated guide covering closed outer profiles and open edge chains, multi-depth stepdowns, helical/linear ramps, tangent lead-in/out arcs, rough/finish wall passes, and automatic + manual rectangular/triangular holding tabs.
- **2D Pocket (`docs/tutorials/03-2d-pocket.md`):** Illustrated guide covering closed multi-island cavities and open-shoulder pockets, contour-parallel vs. zig-zag clearing, overhang air-entry expansion, and radial/axial stock-to-leave (`wall`/`floor`) followed by a finishing pass.
- **Drilling & Helical Bore Milling (`docs/tutorials/04-drilling-helical-bore.md`):** Illustrated guide covering 3D cylindrical holes and 2D bolt-circle point patterns, spot drilling, peck cycles (`G83`/`G73`), multi-pitch helical bore interpolation + flat-bottom cleanup, and comparing LinuxCNC canned cycles against expanded GRBL G-code.

### Milestone 15 — Advanced & Specialty Tutorials: Adaptive/Rest, Chamfer/Trace, V-Carving/Inlays, & Verification (`plans/15-milestone-15-plan.md`)
- **2D Adaptive Clearing & 2.5D Rest Machining (`docs/tutorials/05-adaptive-rest-machining.md`):** Illustrated guide to constant-engagement trochoidal roughing (`optimal_load` `ae`, `both_ways`, micro-lift linking, radial chip-thinning compensation) chained into smaller-tool `FromPrecedingOperations` Rest Machining, showing DAG wave regeneration and heightmap corner cleanup.
- **Edge Chamfering & 2D Curve Trace / Engraving (`docs/tutorials/06-chamfer-curve-trace.md`):** Illustrated guide to chamfer-mill tip-offset kinematics (`tip_dia`, included angle, `tip_offset`) on sharp unmodeled CAD edges and modeled `45°` bevel faces, plus authoring and 1-pass tracing single-line Hershey engraving text.
- **3D Medial-Axis V-Carving, Two-Tool Flat Clearance, & Mated V-Inlays (`docs/tutorials/07-vcarve-inlays.md`):** Illustrated guide to TrueType/SVG artwork setup, variable-depth `60°` V-Bit medial-axis carving with sharp 3D corner climbs, `flat_depth` Stage 1 Flat Endmill clearing + Stage 2 V-Bit bevels (ATC `M6` vs. split-file GRBL), and mirrored Male V-Inlay plugs (`start_depth` + `glue_gap_bottom`).
- **End-to-End Verification & Docs Polish (`README.md`, `docs/tutorials/`):** Verify every tutorial's referenced commands/projects pass `chip-sim` with zero crashes/gouges and emit valid GRBL + LinuxCNC G-code, audit all Markdown links and images, and link `docs/tutorials/` from `README.md`.

---

## Phase 5 — 3-Axis Surface Milling, Mesh Support, & 4th-Axis Rotary Milling (Active — M16–M21)

Phase 5 extends ChipCutter beyond 2.5D prismatic and V-carve machining into **triangle mesh imports (`STL`/`OBJ`/`3MF`)**, full **3-axis sculpted surface roughing & finishing**, and **4th-axis rotary milling** (supporting configurable `A`/`B`/`C` rotary axes, 2-linear + rotary `Z+X`/`Z+Y` workflows, indexed `3+1` positioning, and full simultaneous `X+Y+Z+Rotary` machining)—while preserving all UI-agnostic crate boundaries, `f64` millimeter accuracy, lossless `schema_version = 1/2` compatibility, and existing Phase 1–2 golden G-code parity:

### Milestone 16 — Triangle Mesh Import (`STL` / `OBJ` / `3MF`), BVH Surface Kernel, & Schema v3 (`plans/16-milestone-16-plan.md`)
- **Pure-Rust Mesh Importers & `MeshBvh` (`chip-cad`, `chip-math`):** Permissively licensed importers for binary/ASCII **STL**, Wavefront **OBJ**, and **3MF** with unit normalization into canonical `f64` millimeters, vertex welding, winding/manifold diagnostics, and an `f64` triangle BVH (`MeshBvh`) coexisting cleanly with STEP B-Rep solids.
- **Analytical Drop-Cutter & Push-Cutter Kernel (`chip-cam`):** Exact `f64` drop-cutter (`z_contact(x, y)`) and push-cutter contact solvers over `MeshBvh` and high-resolution STEP B-Rep surfaces supporting Flat, Ball, Bull-Nose, and Tapered/V-Bit cutters with radial/axial stock-to-leave and 2D containment boundary clipping.
- **Schema v3, `.chipz` Mesh Bundling, CLI, & Viewport (`chip-project`, `chip-cli`, `chip-gui`):** Lossless `.chip.toml` `schema_version = 2 -> 3` migration supporting `.stl`/`.obj`/`.3mf` model entries (SHA-256 + `.chipz` `models/*` bundling), `chipcutter inspect` mesh support, `wgpu` shaded mesh rendering, and Setup stock/WCS alignment on imported meshes.

### Milestone 17 — 3D Adaptive Roughing & 3D Waterline (Z-Level) Machining (`plans/17-milestone-17-plan.md`)
- **3D Waterline / Z-Level Slicing (`chip-cam`):** Horizontal Z-level contour slicing across triangle meshes and STEP solids (`OperationKind::Waterline3D`) with steep-slope filtering, adaptive intermediate Z-stepdowns on shallow plateaus, closed/open boundary support, and smooth helical/ramp entry.
- **3D Adaptive Clearing / Roughing (`chip-cam`):** Multi-level constant-engagement 3D trochoidal roughing (`OperationKind::Adaptive3D`) combining Z-level slices, intermediate flat-step detection, stock-to-leave allowances, micro-lift stay-down linking, and radial chip-thinning feed compensation for sculpted 3D cavities and bosses.
- **3D Target Gouge Verification & UI (`chip-sim`, `chip-gui`):** 2D sketch/silhouette containment boundaries, 3D target-mesh gouge checking in `chip-sim`'s 5-rule verifier (including analytical Ball & Bull-Nose sweepers), and `3D Waterline` / `3D Adaptive` Inspector controls.

### Milestone 18 — 3D Surface Finishing: Parallel Raster, Constant-Scallop, & Pencil Cleanup (`plans/18-milestone-18-plan.md`)
- **3D Parallel / Raster Finishing (`chip-cam`):** Exact drop-cutter 3D parallel surfacing (`OperationKind::Parallel3D`) with configurable pass angle, shallow-area slope limits, optional perpendicular cross-finishing pass, smooth arc linking, and cusp-height stepover control.
- **Constant-Scallop & 3D Pencil Finishing (`chip-cam`):** Constant-cusp-height **3D Scallop** (`OperationKind::Scallop3D`) generating surface-projected offset contours across variable slopes for uniform finish, paired with **3D Pencil** (`OperationKind::Pencil3D`) bi-tangent valley/fillet tracing to clean concave corners after larger ball-nose passes.
- **3D Rest Boundaries, Viewport Showcase, & CLI Goldens (`chip-sim`, `chip-gui`, `chip-cli`):** Rest-machining containment derived from preceding 3D roughing/finishing operations, 3D Finishing Inspector UI + built-in sculpted 3D sample project, and headless `chipcutter` 3-axis golden G-code verification.

### Milestone 19 — 4th-Axis Machine Kinematics, Rotary Stock/WCS, & Rotary CL-Data / Posts (`plans/19-milestone-19-plan.md`)
- **Configurable Rotary Axis & Setup Model (`chip-cam`, `chip-project`):** Setup-level 4th-axis configuration aligning the rotary axis along **`X` (`A`-axis)**, **`Y` (`B`-axis)**, or **`Z` (`C`-axis)** with configurable rotation direction (`CW`/`CCW`), centerline origin/offset, cylindrical rotary stock sizing, and configurable post-processor axis letter mapping.
- **Rotary `ClEvent` IR & 4-Axis Post-Processors (`chip-cam`, `chip-post`):** Extend typed `ClEvent` with rotary angular coordinates, inverse-time feed (`G93`) vs. surface/angular feed (`G94`), continuous multi-turn unwinding / shortest-path indexing, and built-in 4-axis `.post.toml` profiles (`linuxcnc_4axis`, `grblhal_4axis`, plus swapped-axis rotary wrapping output).
- **Polar Stock Simulation & Rotary Viewport Gizmo (`chip-sim`, `chip-gui`):** Polar/cylindrical heightmap `(u, θ) -> r` material-removal sweeper and 5-rule collision/gouge verifier, 3D viewport rotary axis centerline/chuck gizmo with animated rotating workpiece/tool playback, and Setup Rotary Inspector controls.

### Milestone 20 — 2-Linear + Rotary (`Z + X` / `Z + Y`) & Indexed `3+1` Multi-Face Milling (`plans/20-milestone-20-plan.md`)
- **2-Linear + Rotary Mode (`Z + X + Rotary` or `Z + Y + Rotary`) (`chip-cam`):** Dedicated centerline-locked rotary mode driving only two linear axes plus the rotary axis (`Z + X` or `Z + Y`, locking the cross-axis at centerline):
  - **Cylindrical Wrapping:** Wrapping 2D Sketch/DXF/SVG contours, pockets, curve traces, and V-carving onto a target cylinder radius.
  - **Continuous Rotary Spiral & Axial Surfacing:** Continuous helical spiral and axial parallel 3D clearing/finishing over arbitrary STEP/STL/OBJ/3MF solids (ideal for 3-axis controllers with a swapped axis or 4-axis machines).
- **Indexed `3+1` Positional Multi-Face Milling (`chip-cam`, `chip-project`):** Orientation-indexed operations (arbitrary rotary angle or B-Rep face normal alignment) executing all existing 2.5D & 3-axis strategies across multiple sides of a part in one chucking with safe rotary-clearance retracts between index rotations.
- **Rotary Preview & Cylindrical Oracle Verification (`chip-sim`, `chip-gui`):** Cylindrical heightmap crash/gouge verification for wrapped, spiral, and indexed toolpaths, live wrapped-path 3D viewport rendering, and Rotary Mode / Index Angle Inspector UI.

### Milestone 21 — Simultaneous 4-Axis Milling (`X + Y + Z + Rotary`), Phase 5 Tutorials, & Hardening (`plans/21-milestone-21-plan.md`)
- **Simultaneous 4-Axis Surface & Undercut Milling (`chip-cam`):** Full simultaneous 4-axis motion operating all three linear axes (`X`, `Y`, `Z`) together with the rotary axis (`A`/`B`/`C`) for off-centerline walls, undercuts, helical flutes/cams, and continuous rotary flowline/swarf finishing with gouge-free tool-axis tilt smoothing.
- **Full 4-Axis Verification & `G93` Feed Control (`chip-sim`, `chip-post`):** Off-centerline shank/holder and target-model collision checks, exact `G93` inverse-time feedrate calculation over simultaneous `(X, Y, Z, A/B/C)` moves, and 4-axis cycle-time estimation.
- **Phase 5 Goldens, Illustrated Tutorials, & Release Hardening (`chip-cli`, `docs/tutorials/`):** End-to-end golden G-code regression suites covering 3-axis mesh surfacing, `Z+X`/`Z+Y` rotary wrapping & spiral surfacing, `3+1` indexed machining, and simultaneous 4-axis milling (preserving Phase 1–2 goldens byte-for-byte), new illustrated Markdown tutorials (`docs/tutorials/`) with minimal starter models & screenshots, and synchronized documentation.

---

## Future Phases

1. **Functional & Behavioral User-Level Testing:**
   - Durable end-to-end user-journey and GUI behavioral test harness (`egui_kittest` / headless interactive workflows + CLI workflows) verifying every feature from a machinist's point of view without brittle coupling to internal implementation details.
2. **Advanced Workholding & Multi-Setup Volumetric Stock:**
   - 3D fixture/vise/clamp collision bodies with automatic safe-rapid routing.
   - Carrying simulated stock across flipped setups via tri-dexel volumetric stock.
3. **Ecosystem & Extensibility:**
   - Optional OpenCASCADE (`opencascade-rs`) `CadKernel` feature backend.
   - Thread milling (`ThreadMill` single/multi-form cutter geometry, ISO Metric & Unified thread tables, radial arc-in/out helical passes), additional controller dialects (Fanuc/Haas, Mach4, Masso, Centroid), and printable HTML/PDF setup sheets.
