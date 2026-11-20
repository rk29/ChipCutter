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

## Phase 5 — 3-Axis Surface Milling, Mesh Support, & 4th-Axis Rotary Milling (Completed — M16–M21)

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

## Phase 6 — Deep-Dive 3-Axis, 4th-Axis, & Production Workflow Tutorials (Completed — M22–M24)

Phase 6 brings every Phase 5 3-axis surfacing strategy, 4th-axis rotary mode, and core production workflow (`DXF`/`SVG` Pure-2D and associative STEP revision reload) up to the same dedicated, step-by-step illustrated tutorial standard as Phase 4's 2.5D guides (`01`–`07`), complete with focused starter models, multi-step headless Vulkan screenshots, and verified CLI equivalents:

### Milestone 22 — Dedicated 3-Axis Surface Milling Tutorials (`plans/22-milestone-22-plan.md`)
- **3D Adaptive Roughing & 3D Waterline / Z-Level (`docs/tutorials/08-3d-adaptive-waterline.md`):** Dedicated step-by-step guide covering `STL`/`OBJ`/`3MF` mesh import & watertightness diagnostics, `Adaptive3D` (`optimal_load`, flat-area detection, 2D sketch containment boundaries, stock-to-leave), and a full **`Waterline3D`** walkthrough (`WaterlineOrderMode`, `Adaptive Shallow Z-Stepdowns`, steep-slope filtering, and smooth helical ramp entry) with multi-step Vulkan screenshots.
- **3D Surface Finishing: Parallel Raster, Constant-Scallop, & Pencil Cleanup (`docs/tutorials/09-3d-surface-finishing.md`):** Dedicated finishing walkthrough comparing `Parallel3D` (`Pass Angle`, `Cross-Pass +90°`, `Slope Range`), `Scallop3D` (target cusp height, `Inside-Out`, `Min/Max Stepover`), 3D Rest Machining (`FromPreviousTool` / `FromPrecedingOperations`), and `Pencil3D` multi-offset concave fillet cleanup on sculpted meshes and STEP solids.

### Milestone 23 — Deep-Dive 4th-Axis Rotary Tutorials: Wrapping, Indexing, & Simultaneous 4-Axis (`plans/23-milestone-23-plan.md`)
- **2-Linear + Rotary Wrapping & Surfacing (`docs/tutorials/10-rotary-wrapping-surfacing.md`):** Step-by-step walkthrough of `Z+X+A` and `Z+Y+B` rotary Setups, `WrapCylinder` (wrapping 2D V-Carve, Pocket, and Contour sketches onto `linuxcnc_4axis` and 3-axis swapped-axis `grbl_rotary_wrap`), and `Rotary2Linear` covering both `ContinuousSpiral` and **`AxialParallel`** patterns plus partial angular sectors (`Start°..End°`).
- **Indexed `3+1` Multi-Face Machining on STEP & Mesh Parts (`docs/tutorials/11-indexed-3plus1-milling.md`):** Multi-face chucking guide covering cardinal `0°/90°/180°/270°` wrench flats plus a dedicated prismatic/angled **STEP solid** starter model demonstrating **Align to Selected 3D Face** for drilling and pocketing angled faces in a single chucking with safe clearance-radius retracts.
- **Simultaneous 4-Axis Milling (`X + Y + Z + A/B`) (`docs/tutorials/12-simultaneous-4axis-milling.md`):** Deep-dive guide comparing **`Auto Surface-Normal Tilt`** (`Max Auto Tilt`, `Tilt Smoothing`) vs. **`Fixed Lateral Offset`**, and **`SimultaneousSpiral`** vs. **`HelicalFlowline`**, with off-centerline `CylindricalStock` verification and `G93` inverse-time inspection on `linuxcnc_4axis` and `grblhal_4axis`.

### Milestone 24 — Production Workflow Tutorials (`DXF`/`SVG`, Associative Reload, `.chipz`) & Screenshot Harness (`plans/24-milestone-24-plan.md`)
- **Pure-2D DXF / SVG Vector Import & Portable `.chipz` Bundles (`docs/tutorials/13-pure2d-dxf-svg-workflow.md`):** End-to-end Pure-2D job walkthrough covering `.dxf` and `.svg` import (units, SVG DPI, sub-layers, endpoint gap healing, island nesting), assigning operations without a 3D STEP model, and packing/building self-contained `.chipz` archives.
- **Associative STEP Revision Reload (`Ctrl+R`) & Re-Pick Recovery (`docs/tutorials/14-associative-step-reload.md`):** Hands-on workflow (`revision-v1` → `revision-v2`) demonstrating automatic `GeometricSignature` re-matching, selective `Dirty` regeneration, and 1-click `NeedsRepick` recovery for deleted or modified CAD features while preserving tool/pass/tab settings.
- **Automated Screenshot Harness & Verification (`scripts/capture_tutorial_screenshots.sh`, `docs/tutorials/README.md`, `README.md`):** Extend the deterministic headless Vulkan screenshot harness to cover all Phase 4 + Phase 6 tutorials, verify every tutorial's CLI commands/projects pass `chip-sim` with zero crashes/gouges, and audit all Markdown links and images.

---

## Phase 7 — GitHub Releases & Cross-Platform Distribution (Completed — M25–M28)

Phase 7 automated cross-platform CI and tag-driven **GitHub Releases** so machinists can download ready-to-run **Linux (Flatpak)**, **Windows**, and **macOS** installers and binaries directly from GitHub without compiling from source:

### Milestone 25 — Release Engineering, App Metadata, & GitHub Actions CI/Release Foundation (`plans/25-milestone-25-plan.md`)
- **Shared Packaging & Branding Assets (`packaging/`):** Vector SVG + multi-resolution PNG app icons, Windows `.ico`, desktop window icon embedding in `chip-gui`, consistent `--version` output for both `chip-gui` and `chipcutter`, and tuned `[profile.release]` settings (LTO, stripped symbols).
- **Installed-Bundle Runtime Safety:** Built-in post-processors, starter libraries, and user config/state directories (`dirs`) resolve reliably when launched from read-only installed locations (Flatpak `/app`, Windows `Program Files`, macOS `.app/Contents/Resources`) rather than a repository working directory.
- **GitHub Actions CI & Release Skeleton (`.github/workflows/ci.yml`, `.github/workflows/release.yml`):** Multi-OS PR/push CI (`ubuntu-latest`, `windows-latest`, `macos-latest`) running `cargo fmt`, `cargo clippy`, `cargo deny`, and core tests; tag-triggered (`v*.*.*`) release workflow with draft/published GitHub Release creation, auto-generated changelog notes, and unified `SHA256SUMS.txt` checksum publishing.

### Milestone 26 — Linux Flatpak Packaging & Release Bundle (`plans/26-milestone-26-plan.md`)
- **Flatpak Sandbox & Portal Readiness:** Configured `rfd` / desktop file-chooser support for `xdg-desktop-portal` so sandboxed users can open/save `.step`, `.stl`, `.obj`, `.3mf`, `.dxf`, `.svg`, `.chip.toml`, `.chipz`, and G-code files cleanly without broad host filesystem access; verified Wayland (`--socket=wayland`), X11 fallback (`--socket=fallback-x11`), and GPU/Vulkan (`--device=dri`) permissions.
- **Manifest, Desktop Entry, & AppStream Metadata (`packaging/flatpak/`):** Reverse-DNS Flatpak manifest (Freedesktop/GNOME runtime + `rust-stable` SDK extension, bundling both `chip-gui` and `chipcutter` CLI), `.desktop` launcher with `.chip.toml` / `.chipz` MIME associations, and AppStream `.metainfo.xml` validated via `appstreamcli validate`.
- **Automated Flatpak CI Build & Release Upload:** GitHub Actions job building the Flatpak with `flatpak-builder`, smoke-testing installation/CLI execution, exporting a single-file `ChipCutter-<version>-linux-x86_64.flatpak` bundle (plus companion Linux CLI tarball), and attaching it to the GitHub Release.

### Milestone 27 — Windows Release Binaries & Installer (`plans/27-milestone-27-plan.md`)
- **Native Windows Build Polish (`x86_64-pc-windows-msvc`):** Embedded Windows PE icon and version metadata into `chip-gui.exe` and `chipcutter.exe`, enabled `#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]` on release GUI builds (no stray console window), and verified `wgpu` DX12/Vulkan startup and native Windows file dialogs (`common-controls-v6`).
- **Portable ZIP & Installer (`packaging/windows/`):** Packaged both a portable `ChipCutter-<version>-windows-x86_64.zip` (`chip-gui.exe`, `chipcutter.exe`, license/README) and a Windows installer (`ChipCutter-<version>-windows-x86_64.msi`) adding Start Menu shortcuts, optional `PATH` registration for `chipcutter.exe`, and `.chip.toml` / `.chipz` file associations.
- **Automated Windows CI Job & Signing Hooks:** Wired the Windows build, packaging, headless CLI smoke test, optional Authenticode signing (guarded by GitHub Actions secrets), and GitHub Release upload into `.github/workflows/release.yml`.

### Milestone 28 — macOS App Bundle, Universal DMG, & Release Docs (`plans/28-milestone-28-plan.md`)
- **macOS `.app` Bundle & Universal2 Targets (`packaging/macos/Info.plist.in`, `scripts/package_macos.sh`):** Builds native Apple Silicon (`aarch64-apple-darwin`) and Intel (`x86_64-apple-darwin`) binaries combined via `lipo` into Universal2 executables inside `ChipCutter.app` with templated `Info.plist`, Retina `.icns` iconset, `CFBundleDocumentTypes` + `UTExportedTypeDeclarations` for `.chipz`, `.chip.toml`, and CAD/mesh/vector imports, bundled `chipcutter` CLI, and `LSMinimumSystemVersion` `12.0`.
- **DMG Disk Image, CLI Tarball, & Optional Signing/Notarization:** Generates a compressed UDZO drag-to-Applications `ChipCutter-<version>-macos-universal.dmg` (with `/Applications` symlink and licenses) plus companion `chipcutter-<version>-macos-universal.tar.gz` in `.github/workflows/release.yml`, with optional Apple Developer ID `codesign` + `notarytool` notarization & `stapler` stapling driven by GitHub Actions secrets and an ad-hoc Apple Silicon fallback verified via `lipo -info`, `plutil -lint`, `codesign --verify`, and a headless CLI smoke test.
- **Release Installation Documentation (`README.md`, `specs/10-roadmap.md`):** Documented Linux (Flatpak & CLI), Windows (`.msi` / `.zip` + SmartScreen), macOS (Universal2 `.dmg`, CLI symlink, & Gatekeeper), and `SHA256SUMS.txt` checksum verification in `README.md`.

---

## Phase 8 — Machine Definitions & Per-Machine Tool Libraries (Completed — M29–M31)

Phase 8 introduces first-class **Machine Definitions** (`*.machine.toml`) so machinists can configure each CNC machine's physical envelope, spindle limits, kinematics, default post-processor, optional 4th-axis rotary module, and **its own dedicated set of tools**—shipping with ready-to-use profiles for a **`generic`** baseline machine (used across existing tests and tutorials), the **Millennium Machines Milo v1.5 (LDO Kit)**, and the **Makera Carvera Air**:

### Milestone 29 — Machine Definition Schema, Persistence, & Per-Machine Tool Libraries (`plans/29-milestone-29-plan.md`)
- **Core `MachineDefinition` Model & Upgrade-Safe TOML Storage (`~/.config/chipcutter/machines/*.machine.toml`):** Define machine ID/name, XYZ travel envelope (`mm`), max rapid/cutting feedrates & acceleration (`MachineKinematicsConfig`), spindle RPM min/max & power, collet/tool-change workflow (manual, quick-change, ATC), optional 4th-axis rotary module defaults (axis, max stock diameter/length, chuck dimensions), default post-processor profile ID, and a **self-contained machine-scoped tool library** (`Vec<Tool>`). Built-in definitions remain strictly **read-only** so application upgrades update them cleanly; cloned and imported custom machines live safely in the user's configuration directory.
- **Project, Setup, & `.chipz` Integration:** Associate projects/setups with a `machine_id` (defaulting cleanly to `generic` for full backward compatibility), bundle custom machine definitions & their tool sets inside portable `.chipz` archives, and add validation checks flagging toolpaths that exceed the active machine's XYZ travel envelope or spindle RPM limits.
- **CLI Support (`chipcutter`):** Add `chipcutter machines list`, `chipcutter machines inspect <id>`, `chipcutter machines clone <src> <new-id>`, `chipcutter machines export <id> -o <file>`, `chipcutter machines import <file>`, and `--machine <id>` validation options.

### Milestone 30 — Built-In Machine Profiles: Generic, Millennium Milo v1.5 (LDO Kit), & Makera Carvera Air (`plans/30-milestone-30-plan.md`)
- **`generic` (`Generic 3/4-Axis CNC Mill`):** Canonical read-only baseline machine preserving the existing starter tool library and 100% compatibility across unit tests, golden G-code snapshots, and tutorials.
- **`milo-1.5` (`Millennium Machines Milo v1.5 — LDO Kit`):** Read-only built-in with `340 × 160 × 120 mm` working envelope, 1.5 kW `10,000–24,000 RPM` ER16 spindle (`3 mm` & `6 mm` collets), `2,000 / 2,000 / 1,000 mm/min` rapids, toolsetter workflow, default post profile, and the **LDO DLC-coated starter tool set** (`Ø6×12 mm` 1FL Flat, `Ø6×22 mm` 1FL Flat, `Ø3×12 mm` 1FL Flat, `Ø3 mm` 2FL Ball, `Ø6 mm 90°` 2FL Chamfer).
- **`carvera-air` (`Makera Carvera Air`):** Read-only built-in with `300 × 200 × 130 mm` working envelope, 200 W `0–13,000 RPM` lever quick-change spindle (`1/8" / 3.175 mm` collet), `4,000 mm/min` max rapid, optional harmonic-drive **4th-Axis Rotary Module** (`X/A`, up to `Ø92 mm × 200 mm` stock), `grblhal_4axis` / Carvera post defaults, and the **Makera Carvera Air starter bit set** (`Ø3.175 mm` 1FL 'O'-flute spiral endmills, `Ø2 mm` & `Ø1 mm` micro endmills, `Ø3.175 mm` ball nose, `30°`/`60°` V-engraving bits, and `90°` chamfer bit).

### Milestone 31 — Machine Manager GUI, Per-Machine Tool Library Switching, & Verification (`plans/31-milestone-31-plan.md`)
- **Machine Selector, Read-Only Built-Ins, & Clone/Share UI (`chip-gui`):** Add a machine selector in Setup / New Project / Preferences (remembering the user's preferred default machine in `prefs.toml`), plus a **Machine Manager** modal where built-in machines are displayed **read-only** with a 1-click **`Clone to Customize`** action (saving the editable clone in user config space), alongside **`Export / Share (.machine.toml)`** and **`Import (.machine.toml)`** for sharing machine + tool definitions with other machinists.
- **Scoped Tool Library & Kinematics Auto-Wiring:** Selecting a machine scopes the Tool Library modal and operation tool picker to that machine's tools (read-only with a quick **`Clone Machine to Edit Tools`** prompt on built-ins, fully editable on user machines, plus copy/import from other machines), automatically syncs simulation kinematics, default post-processor, and 4th-axis rotary defaults, and surfaces machine-envelope/RPM warnings in Diagnostics.
- **Tests & Documentation:** End-to-end roundtrip, read-only built-in protection, clone/export/import, envelope/RPM validation, and `.chipz` portability tests covering `generic`, `milo-1.5`, and `carvera-air`, plus documentation on cloning, customizing, and sharing machine definitions.

---

## Phase 9 — Security Hardening & Safe Untrusted-File Handling (Completed — M32–M34)

Phase 9 addresses every finding from the codebase security evaluation so machinists can safely inspect, simulate, and post-process untrusted `.chipz` bundles, `.chip.toml` projects, shared `.machine.toml` / `.post.toml` files, Fusion 360 `.tools` archives, `.3mf` meshes, and `.dxf` drawings—and ensures CI/CD release pipelines enforce least privilege and immutable action pinning:

### Milestone 32 — Post-Processor Integrity, Safe Output Paths, & G-Code Injection Defense (`plans/32-milestone-32-plan.md`)
- **Safe Output Filenames (`chip-post`, `chip-cli`, `chip-gui`):** Sanitize `PostProfile::extension` to strict ASCII alphanumerics (`[A-Za-z0-9]{1,16}`, fallback `nc`) and enforce single-component filename validation before writing emitted G-code in both `chipcutter build` and multi-file GUI export—eliminating arbitrary file-write path traversal.
- **Read-Only Built-In Post-Processors & Explicit Path Resolution (`chip-post`, `chip-tools`, `chip-cli`, `chip-gui`):** Prevent untrusted `.chipz` bundles, project directories, or CWD `./posts/` from shadowing built-in post profiles (`grbl`, `grblhal_atc`, `linuxcnc`, etc.); separate explicit CLI file-path loading from untrusted project IDs (`setup.post_profile`, `project.machine_id`); and display an explicit warning banner whenever a project uses a custom bundled `.post.toml`.
- **G-Code Injection & Non-Finite Guard (`chip-post`):** Sanitize `{op_name}`, `{strategy_name}`, and `{tool_desc}` via `sanitize_comment_ascii` prior to template interpolation, reject embedded `\r`/`\n` inside `LineWriter::push_code_line`, and fail closed on non-finite (`NaN`/`±Inf`) values in `format_decimal`.

### Milestone 33 — Untrusted Archive, Parser DoS, Temp Directory, & `.chipz` Packing Hardening (`plans/33-milestone-33-plan.md`)
- **Cumulative Zip-Bomb Defenses (`chip-project`, `chip-tools`):** Enforce strict actual-bytes-read cumulative decompression budgets, entry-count caps, and symlink rejection in both `.chipz` unpacking (`unpack_chipz_bytes`) and Fusion 360 `.tools` ZIP import (`import_fusion_library_bytes`)—never trusting ZIP central directory `entry.size()` headers alone.
- **Bounded Recursion & Expansion Caps (`chip-math`, `chip-cad`, `chip-sketch`):** Add explicit nesting-depth (`MAX_EXPR_DEPTH`) and token-count (`MAX_EXPR_TOKENS`) limits to the parametric expression evaluator (`chip-math::expr`), cumulative triangle cap (`MAX_3MF_TRIANGLES`) to 3MF component instantiation (`chip-cad::mesh::threemf`), and cumulative entity cap (`MAX_DXF_ITEMS`) to DXF `INSERT` block expansion (`chip-sketch::dxf_import`).
- **Exclusive `0o700` Temp Extraction & Out-of-Project `.chipz` Packing Guard (`chip-project`, `chip-cli`, `chip-gui`):** Replace predictable `/tmp` `create_dir_all` extraction directories with exclusive, private (`0o700` on Unix) temporary directory creation, and guard `.chipz` packing against silently exfiltrating out-of-project local files referenced by untrusted `.chip.toml` manifests.

### Milestone 34 — CI/CD Hardening, Packaging Semver Validation, & Security Policy (`plans/34-milestone-34-plan.md`)
- **Expression-Injection Elimination & Least-Privilege Tokens (`.github/workflows/`):** Route all workflow inputs and job outputs (`inputs.tag`, `needs.prepare.outputs.version`) exclusively through `env:` blocks, enforce strict semver regex validation in `prepare`, `scripts/package_macos.sh`, and `scripts/build_flatpak.sh`, and restrict `permissions: contents: write` strictly to `publish-release` (`contents: read` everywhere else).
- **Immutable Supply-Chain Pinning & Security Documentation:** Pin third-party GitHub Actions in `.github/workflows/release.yml` and `.github/workflows/ci.yml` to immutable 40-character commit SHAs (with version comments) and publish `SECURITY.md` documenting vulnerability reporting and ChipCutter's threat model for untrusted CAM files.

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
4. **Tutorials for Upcoming Future-Phase Capabilities:**
   - Dedicated illustrated guides accompanying new future-phase strategies (thread milling, 3D fixture/vise collision avoidance, and multi-setup volumetric stock workflows).
