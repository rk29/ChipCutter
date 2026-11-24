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
- **Flatpak Sandbox & Portal Readiness:** Configured `rfd` / desktop file-chooser support for `xdg-desktop-portal` (`flatpak-portal`) paired with `--filesystem=host` so multi-file `.chip.toml` projects can resolve sibling and referenced `.step`, `.stl`, `.obj`, `.3mf`, `.dxf`, `.svg`, custom font, and local `.machine.toml`/`.post.toml` assets cleanly alongside `.chipz` bundles and G-code exports; verified Wayland (`--socket=wayland`), X11 fallback (`--socket=fallback-x11`), and GPU/Vulkan (`--device=dri`) permissions.
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

## Phase 10 — Model Orientation, Positioning, Scaling, & Setup Alignment (Completed — M35–M37)

Phase 10 gives machinists full, non-destructive control over rotating, laying flat, scaling, and translating imported **STEP** solids and **STL / OBJ / 3MF** triangle meshes after import—preserving exact `f64` B-Rep/mesh geometry and associative `Ctrl+R` reload—paired with an illustrated tutorial covering both per-model transforms and multi-setup WCS alignment:

### Milestone 35 — Non-Destructive `ModelTransform` Kernel (Rotate, Scale, Move), Persistence, & CLI (`plans/35-milestone-35-plan.md`)
- **Exact `f64` `ModelTransform` Model (`chip-cad`, `chip-project`):** Persist optional non-destructive `transform: ModelTransform` (`translate_mm: [f64; 3]`, `rotate_deg: [f64; 3]`, `scale: [f64; 3]`, omitted in `.chip.toml` when identity for 100% backward compatibility) on `ModelEntry`, applying exact `f64` Scale → Rotate → Translate transforms across STEP B-Rep surfaces/curves/normals/loops/`TopologyGraph` (exact analytic radius scaling for uniform scale; clean polyline/freeform degradation for stretched arcs/cylinders under non-uniform scale) and `TriangleMeshBody` vertices/normals/`MeshBvh`/`RenderMesh`.
- **Orientation, Scaling, & Placement Presets + Associative Reload Preservation:** Provide deterministic helpers (`y_up_to_z_up`, `rotate_90_{x,y,z}`, `lay_flat_on_normal` for both STEP faces and mesh triangles, `uniform_scale`, `fit_to_target_dimension_{x,y,z}`, `center_xy_on_origin`, `drop_bottom_to_z0`, `align_top_to_z0`, `reset`) and retain `ModelEntry.transform` automatically across associative `Ctrl+R` / `reload-step` CAD revisions and `.chipz` roundtrips.
- **Headless CLI Support (`chipcutter`):** Add `chipcutter transform-model <project.chip.toml>` (`--rot-x`, `--rot-y`, `--rot-z`, `--scale`, `--scale-x/--scale-y/--scale-z`, `--fit-size-x/--fit-size-y/--fit-size-z`, `--move-x`, `--move-y`, `--move-z`, `--y-up-to-z-up`, `--center-xy`, `--drop-z0`, `--reset`) and optional `--rot-x/--rot-y/--rot-z` / `--scale` flags on `import-mesh`.

### Milestone 36 — Interactive Model Positioning & Scaling UI, Mesh Import Orientation, & Setup WCS Alignment Polish (`plans/36-milestone-36-plan.md`)
- **Model Transform Inspector (`chip-gui`):** Clicking a model in the CAM Browser opens a dedicated **Model Transform & Orientation** inspector panel with 1-click **`Y-Up → Z-Up (+90° X)`**, **`±90° X / Y / Z`**, interactive 3D **`Lay Flat on Clicked Face…`** (supporting both STEP planar faces and STL/OBJ/3MF mesh triangles), **`Center XY`**, **`Drop Bottom to Z=0`**, **`Align Top to Z=0`**, **`Reset`**, exact expression-capable `Rotate X/Y/Z (°)`, `Move X/Y/Z`, **`Scale` (Uniform Lock or Independent X/Y/Z)** + **`Target Size X/Y/Z`** (auto-computing exact uniform scale to fit a stock plaque), live transformed bounding-box readout, full `< 1 ms` Undo/Redo, and automatic `Dirty` invalidation on dependent setups/operations.
- **Mesh Import Orientation & Scale Preview (`mesh_import_modal.rs`):** Add quick orientation buttons (`Keep As-Is`, `Y-Up → Z-Up`, `±90° X/Y/Z`) and uniform **`Scale` / `Target Size`** controls directly inside the **Import 3D Mesh (STL / OBJ / 3MF)** modal with live physical bounding-box updates before import.
- **Setup WCS Orientation Polish (`setup_inspector.rs`):** Add optional arbitrary X-axis clocking angle (`°`) alongside `Clock X 90°` on `WcsOrientationMode` (for angled vise setups without modifying model transforms) plus clear UI tooltips explaining when to use *Model Transform* vs. *Setup WCS*.

### Milestone 37 — Illustrated Tutorial 16: Rotating, Laying Flat, Scaling, & Moving STEP & STL Models (`plans/37-milestone-37-plan.md`)
- **Minimal Starter Fixtures (`docs/tutorials/assets/`):** Add reproducible CLI tutorial generators (`--tutorial 16-orient-stl` Y-up/rotated STL mesh and `--tutorial 16-orient-step` tilted/off-origin STEP bracket).
- **Tutorial 16 (`docs/tutorials/16-model-orientation-positioning.md`):** Publish a complete 7-part illustrated guide with visually verified headless Vulkan screenshots covering (1) fixing a Y-up STL mesh on import and via Model Transform (`Y-Up → Z-Up`, `Lay Flat on Face`, uniform **`Scale` / `Target Size`**, `Center XY`, `Drop to Z=0`), (2) rotating, scaling, and moving a STEP solid while preserving associative `Ctrl+R` reload, (3) aligning a tilted STEP part via Setup WCS (`Z & X`, `XY Plane`, `Flip +Z`, `Clock X 90°`, 27-point origin, `Additional Offset`) and creating an Op-2 flipped Setup (`G55`), and (4) headless `chipcutter` CLI equivalents.

---

## Phase 11 — Tri-Dexel Volumetric CAM Simulation & Quantitative Error-Volume Metrics (Completed — M38–M40)

Phase 11 introduces a true 3D **Tri-Dexel volumetric simulation engine** alongside the existing fast **Z-Dexel Heightmap** (and cylindrical rotary) simulation—keeping the existing heightmap engine selectable at all times—and adds comprehensive **stock-vs-target quantitative error metrics** (total error volume, excess/gouge volume, RMS deviation, MAE, signed bias, P95/P99 percentiles, max excess/gouge, and in-tolerance surface %) so future toolpath generation improvements can be measured, visualized, and benchmarked objectively in both GUI and CLI:

### Milestone 38 — Tri-Dexel Volumetric Stock Kernel & Swept-Cutter Subtraction (`plans/38-milestone-38-plan.md`)
- **Selectable Simulation Engine (`chip-sim`, `chip-project`):** Introduce `SimEngineKind` (`Heightmap` | `TriDexel`) so machinists and CLI scripts can choose between the existing ultra-fast 2.5D Z-Dexel heightmap and the new full-3D Tri-Dexel volumetric engine without losing backward compatibility.
- **3-Axis Orthogonal Dexel Grid (`TriDexelStock`):** Represent raw stock and target solids using three orthogonal 1D sorted interval lists (`X`-rays, `Y`-rays, `Z`-rays) in canonical `f64` millimeters with per-endpoint operation provenance (`op_idx`), supporting multi-interval cavities, vertical/steep walls with zero heightmap stair-stepping, horizontal features, and true 3D undercuts across all 6 cutter geometries (Flat, Ball, Bull-Nose, Chamfer, V-Bit, Twist Drill, Face Mill).
- **Swept Subtraction, 5-Rule Verification, & Crisp Surface Extraction:** Implement `rayon`-parallel analytical swept-cutter subtraction along `G1`/`G2`/`G3` and indexed/rotary moves, full 3D 5-rule collision/gouge verification (catching vertical-wall and undercut gouges), progressive timeline checkpoints, and sharp-feature-preserving tri-dexel mesh extraction for the 3D `wgpu` viewport.

### Milestone 39 — Quantitative Error-Volume Metrics, Deviation Heatmaps, & Algorithm Benchmarking (`plans/39-milestone-39-plan.md`)
- **Stock-vs-Target Metrology (`StockErrorReport` in `chip-sim`):** Compare simulated machined stock against the exact target B-Rep/mesh solid (supported by **both** Tri-Dexel and Heightmap engines) and report:
  - **Volumetric Error (`mm³` & `%`):** **Total Error Volume** (`excess + gouge`), **Excess (Uncut / Rest) Volume**, **Overcut / Gouge Volume**, target volume, and material removal efficiency (`%`).
  - **Surface Deviation Statistics (`mm`):** **Root-Mean-Square (RMS) Error** (overall, excess-only, and gouge-only), **Mean Absolute Error (MAE)**, **Mean Signed Bias** (oversize vs. undersize drift), **P95 & P99 Absolute Error**, **Max Excess** (peak scallop / uncut corner) and **Max Gouge** with exact 3D WCS coordinates, and **In-Tolerance Surface Fraction (`% within ±tol_mm`)**.
  - **Per-Operation Error Progression:** Record remaining error volume and RMS after each operation to enable apples-to-apples benchmarking of roughing, rest-machining, and finishing algorithms.
- **Signed Deviation Heatmap & Headless CLI Benchmarking (`chip-sim`, `chip-cli`):** Color rendered stock vertices by signed target deviation (blue = uncut excess, green = within tolerance, red = gouge/overcut), extend `chipcutter simulate` with `--engine <heightmap|tri-dexel>` and `--tolerance <mm>`, and emit full `StockErrorReport` JSON metrics so future toolpath algorithm PRs can track quantitative error-volume and RMS improvements automatically.

### Milestone 40 — GUI Engine Selector, Error-Metrics Dashboard, & Illustrated Tutorial 17 (`plans/40-milestone-40-plan.md`)
- **Simulation Engine & Deviation Heatmap UI (`chip-gui`):** Add an engine selector (`Heightmap (Fast)` vs. `Tri-Dexel (Volumetric 3D)`) and `Color by Op` / `Error Heatmap` toggle (with adjustable `±tol` legend) to the Simulation Timeline toolbar, persisting engine preference in `prefs.toml`.
- **Error Metrics Inspector & Click-to-Jump (`chip-gui`):** Display Total/Excess/Gouge Error Volume, RMS, MAE, P95/P99, In-Tolerance %, and Max Excess / Max Gouge cards with 1-click camera jump to worst excess or gouge spots, plus per-operation error breakdown for comparing toolpath settings interactively.
- **Illustrated Tutorial 17 (`docs/tutorials/17-tridexel-simulation-error-metrics.md`):** Publish a 7-part illustrated guide with headless Vulkan screenshots teaching machinists when to use Heightmap vs. Tri-Dexel simulation, how to interpret error volume / RMS / deviation heatmaps to tune stepover and rest machining, and how to run headless CLI error benchmarks.

---

## Phase 12 — GUI Polish, Consistent Panel UX, Unsaved-Changes Guard, & Illustrated Sample Gallery (Completed — M41–M43)

Phase 12 focuses on desktop usability, visual consistency, responsiveness, and discoverability across the entire `chip-gui` workspace and sample-project documentation:

### Milestone 41 — Unified Collapsible Cards, Strict Two-Column Inspectors, & Consistent Visibility/Active Controls (`plans/41-milestone-41-plan.md`)
- **Unified Collapsible Card Styling Across Left & Right Panels (`panels/browser.rs`, `widgets/section_card.rs`):** Restyle the left **CAM Browser** (`Models`, `2D Sketches`, `Project Tools`, and per-`Setup` sections) using the same elevated collapsible `section_card` framing and typography already used on the right Inspector panels so both sidebars look cohesive in Studio Dark and Daylight Light themes.
- **Strict Two-Column Inspector Cards (`panels/setup_inspector.rs`, `panels/operation_inspector.rs`, `panels/model_inspector.rs`, `panels/sketch_inspector.rs`):** Audit and refactor overly wide right-panel cards—especially **WCS Origin** and any sections placing two label/value control pairs side-by-side—into a clean, compact two-column `Name | Value / Selection` layout that fits narrow Inspector widths without horizontal scrolling or clipping.
- **Consistent Visibility (`Eyeball`) vs. Active/Inactive (`Checkbox`) Semantics (`panels/browser.rs`, `chip-gui`):** Standardize icons and checkboxes across the CAM Browser so **Eyeball icons** always toggle **viewport visibility** (2D Sketches & individual Operation toolpaths) and **Checkboxes** always toggle **Active / Enabled vs. Suppressed / Inactive** state (Operations & Sketches), eliminating the inverted eyeball-as-suppress and checkbox-as-visibility mismatch.

### Milestone 42 — Universal Control Tooltips, Heatmap Toggle & Playback Performance, & Unsaved-Changes Close Guard (`plans/42-milestone-42-plan.md`)
- **Universal Hover Tooltips (`chip-gui`):** Audit every interactive control across the top menu/toolbar, CAM Browser, right-side Inspectors, bottom panes (`Simulation Timeline`, `G-Code Preview`, `Diagnostics & Log`), viewport HUD, and modals so every button, checkbox, dropdown, slider, and numeric input provides clear hover text explaining what it does.
- **Error Heatmap Display Toggle & Fast Simulation Playback (`panels/simulation.rs`, `chip-sim`):** Make enabling/disabling the signed **Error Heatmap** display unmistakable and responsive (resolving overlap with `Color by Op`, guaranteeing zero heatmap cost when disabled, deferring/optimizing expensive per-vertex deviation coloring during live scrubbing, and caching deviation lookups so playback stays smooth).
- **Unsaved-Changes Prompt on Window Close (`app/mod.rs`, `app/loader.rs`):** Intercept application close requests (`ViewportEvent::CloseRequested`) whenever the active project has unsaved edits since the last save, prompting the user with **`Save`**, **`Discard & Close`**, or **`Cancel`** (while leaving headless `--self-test-screenshot` automation unaffected).

### Milestone 43 — Illustrated Sample Projects Gallery Documentation (`plans/43-milestone-43-plan.md`)
- **Visual Sample Gallery Index (`docs/samples/README.md`):** Publish a thumbnail gallery showcasing every built-in sample project (`SAMPLE_CATALOG`, all 15 STEP, STL/OBJ, Pure-2D, V-Carve, and 4th-Axis Rotary samples), linking each card to its dedicated detail page.
- **Dedicated Per-Sample Pages & Per-Operation Completion Captures (`docs/samples/<slug>.md`, `docs/images/samples/<slug>/`):** Extend headless capture support (`--capture-sim-after-op` / `--capture-setup`) with a reproducible script (`scripts/capture_sample_gallery.sh`) to generate full screenshots and descriptions for every sample—including an individual screenshot at the exact simulation completion point of **each Setup and Operation** for multi-op and multi-setup samples.

---

## Phase 13 — Expanded Machine Definitions & Helical Thread Milling (Completed — M44–M47)

Phase 13 expands ChipCutter's built-in desktop CNC machine catalog with four widely used maker/benchtop machines and introduces full end-to-end **Helical Thread Milling** (single-point and multi-form cutters, built-in ISO Metric & Unified thread tables, internal/external helical arc-in/out toolpaths, Tri-Dexel undercut simulation, Tool Library & Operation Inspector UI, a built-in sample project, and illustrated Tutorial 18):

### Milestone 44 — Built-In Machine Definitions: Carvera, Carvera Z1, Milo 2.0, & Voron Cascade (`plans/44-milestone-44-plan.md`)
- **Four New Read-Only Built-In Machine Profiles (`chip-tools`, `machines/`):**
  - **`carvera` (`Makera Carvera`):** `360 × 240 × 140 mm` travel, `200 W` `1,000–15,000 RPM` `1/8"` spindle, 6-tool **`AutomaticAtc`** carousel + fixed toolsetter, `grblhal_atc`, optional 4th-axis rotary (`Ø92 × 240 mm`, `grblhal_4axis`), and Makera Carvera ATC starter tool set.
  - **`carvera-z1` (`Makera Carvera Z1`):** `200 × 200 × 100 mm` travel, `150 W` `1,000–13,000 RPM` lever **`QuickChange`** `1/8"` spindle + auto tool-height probing, `grblhal_atc`, optional 4th-axis rotary (`Ø80 × 150 mm`, `grblhal_4axis`), and Z1 starter tool set tuned for the `150 W` spindle.
  - **`milo-2.0` (`Millennium Machines Milo v2.0`):** `348 × 210 × 120 mm` travel, `1.5 kW` `10,000–24,000 RPM` `ER16` spindle, `Manual` + toolsetter, `linuxcnc`, and upgraded Milo 2.0 starter tool set (including starter single-point thread mill).
  - **`voron-cascade` (`Voron Cascade`):** `275 × 220 × 90 mm` enclosed travel (`340 × 225 mm` tooling plate), `1.5 kW` `8,000–24,000 RPM` `ER16` VFD spindle, `Manual` + toolsetter, `grblhal_atc`, and aluminum/acetal starter tool set (including starter single-point thread mill).
- **CLI, GUI, & Documentation Parity:** Checked-in reference TOML files under `machines/`, unit tests enforcing code-vs-TOML parity and envelope/spindle validation, and updated Machine Manager docs (`docs/tutorials/15-machine-definitions-tool-sets.md`, `README.md`).

### Milestone 45 — `ThreadMill` Cutter Geometry, ISO Metric & Unified Thread Tables, & Helical Thread Milling Kernel (`plans/45-milestone-45-plan.md`)
- **`ThreadMill` Cutter Geometry (`chip-tools`):** Add `CutterGeometry::ThreadMill` supporting both single-point (`teeth_count = 1`) and multi-form (`teeth_count > 1`) cutters with relieved neck (`neck_diameter_mm`, `neck_length_mm`), `pitch_mm`, and `thread_angle_deg` (`60°` default), plus validation and Fusion 360 `.tools` import mapping.
- **Built-In Standard Thread Tables (`chip-cam`):** Deterministic lookup tables for **ISO Metric Coarse & Fine** (`M2`–`M24`) and **Unified UNC / UNF** (`#4-40`–`3/4-10`) providing major/minor/pitch diameters, standard pitch, and recommended pilot/tap-drill diameters.
- **Helical Thread Milling Strategy (`OperationKind::ThreadMill` in `chip-cam`, `chip-post`):**
  - Supports **Internal** (cylindrical B-Rep holes / sketch circles) and **External** (cylindrical bosses / closed circular loops), **Right-Hand** and **Left-Hand**, and **Climb** (bottom-up internal RH / top-down external RH) vs. **Conventional** direction.
  - Emits exact helical `G2`/`G3` interpolation (`I`, `J`, `Z` pitch advance per `360°`) with smooth **radial tangent arc-in / arc-out** from centerline or outside clearance, multi-step **radial roughing passes**, **spring passes**, **pitch-diameter fit offset**, multi-form single-revolution optimization, and automatic **centerline-compensated feedrate scaling** (`F_center = F_edge * (D_major ± D_tool) / D_major`).
  - Strict geometric safety guards (cutter diameter vs. minor diameter, neck reach vs. thread depth, multi-form pitch match) and verified `GRBL` quadrant-split helical arcs + `LinuxCNC` full helical `G2`/`G3` posting.

### Milestone 46 — Thread Mill Simulation (Tri-Dexel Undercuts, 3D Cutter Mesh) & Desktop GUI (`plans/46-milestone-46-plan.md`)
- **Simulation & 5-Rule Verification (`chip-sim`):**
  - Add revolved 3D `ThreadMill` cutter geometry (relieved neck + 60° V-tooth profile) to `cutter_profile.rs`.
  - Support true **3D helical undercut thread-groove subtraction** in **Tri-Dexel** mode (`X`/`Y`/`Z` dexels) alongside safe minor/major verification in **Heightmap** mode, updating the 5-rule verifier to recognize relieved-neck clearance inside pre-drilled pilot bores while still catching rapid crashes, un-drilled plunges, and shank/holder collisions.
- **Tool Library & Operation Inspector UI (`chip-gui`):**
  - Add **Thread Mill** cutter type and 2D relieved-neck schematic to **Tool Library Manager** (`tool_library.rs`).
  - Add `+ Thread Mill` creation actions, `IconKind::ThreadMill`, and a strict two-column **Thread Mill Passes** card (Thread Standard preset picker `ISO Metric` / `UNC` / `UNF` / `Custom`, Internal/External, Right/Left Hand, Climb/Conventional, Pitch, Major/Minor Ø, Radial Passes, Spring Passes, Pitch-Diameter Fit Offset, Centerline Feed Compensation, and live clearance/tap-drill guidance banner) with universal hover tooltips.

### Milestone 47 — Built-In Thread Milling Sample, Illustrated Tutorial 18, Gallery Update, & Golden Verification (`plans/47-milestone-47-plan.md`)
- **Starter Fixture & Built-In Sample (`chip-cad`, `chip-project`, `chip-gui`, `chip-cli`):** Add `docs/tutorials/assets/18-threaded-manifold.step` (`chipcutter generate-sample-step --tutorial 18-thread-milling`) and built-in sample **`thread-mill`** (`SampleProjectId::ThreadMillManifold`) featuring Facing, pilot drilling/boring, internal `M8×1.25` & `1/4-20 UNC` thread milling, and external `M12×1.75` boss thread milling.
- **Illustrated Tutorial 18 & Sample Gallery (`docs/tutorials/18-thread-milling.md`, `docs/samples/thread-mill.md`):** Publish a 7-part illustrated guide with headless Vulkan screenshots (including Tri-Dexel undercut thread-groove close-ups), add the new sample & per-operation completion screenshots to the Sample Projects Gallery and GitHub Pages site, and lock down `GRBL` + `LinuxCNC` golden G-code regression tests.

---

## Phase 14 — Core Architecture Refactoring, Internal Algorithm Crates, & GPU-Compute Readiness (Completed — M48–M51)

Phase 14 consolidated duplicated simulation/geometry kernels, eliminated the `O(N)` mesh raycast bottleneck, broke the `chip-cam` ↔ `chip-sim` dependency bottleneck for future 3D rest/carryover workflows, extracted three focused UI-agnostic algorithm crates (`chip-bvh`, `chip-geom2d`, `chip-stock`), split oversized modules for faster incremental builds, moved interactive stock remeshing off the `egui` UI thread, and transitioned simulation stock buffers and viewport rendering onto GPU-compute-ready flat layouts—strictly preserving byte-for-byte golden G-code parity, lossless `.chip.toml` schema compatibility, and zero new external dependencies:

### Milestone 48 — Shared `AnalyticCutterEnvelope`, Zero-Copy GPU Stock Buffers, & Simulation/Rotary Deduplication (`plans/48-milestone-48-plan.md`)
- **Shared Cutter Math (`chip-tools`):** Extracted `AnalyticCutterEnvelope` into `chip-tools` and delegated both `chip-cam::surface3d::SurfaceCutterProfile` and `chip-sim::RadialCutterProfile` to it, eliminating all cutter-profile math duplication across 7 cutter types (including non-monotonic `ThreadMill` teeth).
- **Zero-Copy Viewport Stock Uploads & GPU Rotary Matrix (`chip-gui`):** Made `SimColoredVertex` `#[repr(C)] bytemuck::Pod + Zeroable`, uploaded simulation stock/cutter meshes via `GrowableGpuBuffer` (zero per-frame GPU buffer allocations or vertex copies), and added a `stock_model: mat4x4<f32>` uniform to `vs_sim_stock` in `shaders.wgsl` so 4th-axis playback rotates on the GPU instead of re-tessellating the cylinder on CPU every `0.01°`.
- **Simulation & Rotary Deduplication + Test Hygiene (`chip-sim`, `chip-cam`, `chip-project`):** Unified stock color palette (`mesher_common.rs`), arc-to-chord decomposition, Rule 1–5 violation accumulator/coalescer (removing dead `recorded_op_kinds` and adding bounded coalescing to `cylindrical.rs`), and Rule 5 vertical plunge checks; deduplicated `build_rotated_strip_mesh` & rotary window setup into `chip-cam/src/rotary.rs`; moved inline integration tests from `chip-project/src/lib.rs` (2,861 LOC) and `chip-sim/src/lib.rs` (1,360 LOC) into `tests/`.

### Milestone 49 — Flat-Array Spatial Index (`chip-bvh`) & 2D Exact Geometry Kernel (`chip-geom2d`) (`plans/49-milestone-49-plan.md`)
- **New Crate `chip-bvh`:** Implemented a flat-array binary SAH BVH (`FlatBvhNode` 32-byte `Pod`, `GpuTriangle` 48-byte `Pod`) and canonical `TriMesh3D` supporting `O(log N)` front-to-back `raycast`, `raycast_all_hits` (even-odd dexel voxelization), `closest_point_on_mesh`, `query_xy_circle_into` (Drop-Cutter), and `query_z_slab` (Push-Cutter). Migrated `chip-cad` (`MeshBvh`, `CadPicker::pick` — fixing `O(N)` linear scans), `chip-cam/src/surface3d`, and `chip-sim/src/cylindrical.rs` (removing `chip-sim`'s dependency on `chip_cam::SurfaceMesh3D`).
- **New Crate `chip-geom2d`:** Extracted exact 2D arc-polyline offsetting (`offset2d`), pure-2D `RegionSet2D` / swept capsules / clipping / even-odd nesting, Voronoi medial-axis extraction (`medial`), and `simplify_rdp_2d` (zero new external deps). Migrated `chip-sketch` (`ttf_outline.rs`, `heal_chain.rs` — unifying all 3 `cavalier_contours` bridges and propagating the `WORK_SCALE = 16.0` evolute-cusp panic safeguard) and `chip-cam` (preserving facade re-exports).

### Milestone 50 — Unified `SimStockBackend`, Axis-Generic Flat-Pool Tri-Dexel, & `chip-stock` Crate Extraction (`plans/50-milestone-50-plan.md`)
- **Axis-Generic Tri-Dexel & Multi-Body Fix:** Parameterized `TriDexelStock` across `planes: [DexelPlane; 3]` + `Axis3` descriptor (`u_axis`, `v_axis`, `w_axis`) across `grid.rs`, `sweep.rs`, `mesher.rs`, and `metrics.rs` (~450 LOC removed) and fixed `target_bodies` filtering on imported STL/OBJ/3MF meshes in `from_target_cad`.
- **GPU-Ready Flat Buffer Layouts:** Migrated `DexelPlane` from `Vec<Vec<DexelSpan>>` (~600k heap allocs at Ultra) to a **flat fixed-`K` span pool** (`K = 4..8` × 16-byte spans + `counts: Vec<u32>` + rare CPU overflow map for `>K` cavities), cutting allocations from ~600,000 to 6 flat `Vec`s; normalized `HeightfieldStock` and `CylindricalStock` to stock-local `f32` (+ `origin_wcs: DVec3`) and `u32` op tags.
- **Unified `SimStockBackend` & New Crate `chip-stock`:** Replaced the 3 parallel stock/checkpoint vectors in `SetupSimulationResult` (removing dummy `HeightfieldStock` placeholders) with a unified `SimulatedStock` enum, batched `SimStockBackend` trait (`CpuExactBackend` default, plug-in point for future `WgpuComputeBackend`), single simulation driver, and single playback cursor. Extracted pure volumetric stock kernels, sweepers, meshers, and metrology into UI-agnostic **`chip-stock`** (zero `chip-cad`/`chip-cam` dependency — breaking the `chip-cam` ↔ `chip-sim` cycle for future 3D rest & `G54 -> G55` stock carryover).

### Milestone 51 — Oversized Module Decomposition, Off-Thread Stock Remeshing, & Architecture Spec Sync (`plans/51-milestone-51-plan.md`)
- **Non-Blocking Simulation Remeshing & `SimStockRenderSource` (`chip-gui`):** Introduced `SimStockRenderSource` (`CpuMesh { revision, mesh: Arc<SimStockMesh>, stock_model: DMat4 }`, cleanly extensible to future `GpuGrid`) in `viewport/highlight.rs`, moved `SimulatedStock` scrub synchronization and mesh extraction off the `egui` UI thread onto a coalescing latest-wins background worker (`panels/simulation/worker.rs`) with deterministic synchronous flush for headless `--capture-*` screenshots, and split `panels/simulation.rs` into `panels/simulation/{mod, worker, playback, ui, metrology_ui}.rs`.
- **Maintainable Module Splits (Zero Behavior Change):** Decomposed `chip-cam/src/{params.rs, selection.rs}`, `chip-tools/src/builtin_machines.rs`, `chip-project/src/demos.rs`, `chip-cli/src/commands/inspect_sample.rs`, and `chip-gui/src/{panels/setup_inspector.rs, panels/tool_library.rs, app/menus_toolbar.rs, app/loader.rs, app/mod.rs, viewport/scene_builders.rs}` into focused, compact submodules while preserving all public APIs.
- **Architecture Docs & Full Parity Gate:** Synchronized `specs/01-architecture.md` (13-crate workspace layout, dependency graph, GPU-ready layouts, and off-thread remeshing) and `specs/10-roadmap.md`, and verified 100% byte-for-byte golden G-code parity, all simulation/error-volume oracles, headless Vulkan screenshot determinism, `cargo deny check` (default + `flatpak-portal`), `clippy`, and full Vulkan workspace tests.

---

## Phase 15 — GPU-Accelerated Simulation (`wgpu` Compute) & Direct GPU Stock Rendering (Completed — M52–M55)

Phase 15 built directly on Phase 14's GPU-ready flat layouts (`chip-bvh` `FlatBvhNode`/`GpuTriangle`, `chip-stock` flat fixed-`K` Tri-Dexel span pools & stock-local `f32`/`u32` grids, `SimStockBackend`, and `SimStockRenderSource`) to deliver an optional **`wgpu` Compute** simulation and rendering backend (`gpu-compute` feature, zero new external dependencies, automatic transparent fallback to exact `CpuExactBackend` whenever no compute adapter is available) across all three volumetric engines:

### Milestone 52 — `WgpuComputeBackend` Foundation & Heightfield / Cylindrical WGSL Compute Sweepers (`plans/52-milestone-52-plan.md`)
- **Feature-Gated `GpuSimContext` & Compute Plumbing (`chip-stock`, `chip-sim`, `chip-project`, `chip-cli`, `chip-gui`):** Added optional `gpu-compute` feature (`wgpu`, `pollster`) on `chip-stock` (forwarded through `chip-sim`/`chip-project`, enabled by default in `chip-gui` including `flatpak-portal` and in `chip-cli`, while keeping default `chip-stock`/`chip-sim` 100% pure-CPU). Supported both shared `eframe`/`ViewportRenderer` `(Arc<wgpu::Device>, Arc<wgpu::Queue>)` and headless adapter creation with `SimComputeBackendChoice { Auto, Gpu, Cpu }` and transparent CPU fallback.
- **GPU Sweep Data Contracts & Atomic Verification:** Defined tightly-packed `#[repr(C)] bytemuck::Pod` `GpuSweepChord`, `GpuToolProfile` (all 7 cutter geometries via `AnalyticCutterEnvelope`, including multi-form `ThreadMill`), and `GpuViolationCounters` (`atomic<u32>` flags + fixed-point max penetration + offending chord index feeding `ViolationBatchCoalescer`).
- **Tiled WGSL Compute Sweepers (`heightfield_sweep.wgsl`, `cylindrical_sweep.wgsl`):** Implemented `16×16` tiled compute sweepers with coarse AABB tile-binning for 2.5D `HeightfieldStock` (including closed-form conical V-bit and sloped flat-endmill intervals) and 4th-axis `CylindricalStock` (off-centerline simultaneous 4-axis, spiral/wrapped, and indexed `3+1` + chuck/holder/flute/gouge checks), verified by CPU-vs-GPU differential parity tests.

### Milestone 53 — GPU Flat Fixed-`K` Tri-Dexel Compute Sweeper, GPU BVH Distance, & GPU Error Metrics (`plans/53-milestone-53-plan.md`)
- **Axis-Generic Flat Fixed-`K` Tri-Dexel WGSL Sweeper (`tridexel_sweep.wgsl`):** Dispatched a single axis-parameterized compute shader across `Axis3::{X, Y, Z}` over the 16-byte `FlatDexelSpan` (`K = 4` inline slots + `counts: array<u32>`) buffers with Rules 1–4 atomic verification and a hybrid overflow safety valve (per-ray atomic overflow flags falling back seamlessly to the exact CPU `BTreeMap` overflow side-table only for rare `> K` rays such as multi-form internal thread mills).
- **GPU Target BVH & Parallel Reduction Metrology (`bvh_distance.wgsl`, `stock_metrics.wgsl`):** Uploaded `chip-bvh`'s `FlatBvhNode`/`GpuTriangle` buffers directly to GPU for fast target rasterization and signed-distance evaluation, and computed `StockErrorReport` metrics (remaining/excess/gouge volume, RMS, MAE, signed bias, P95/P99 percentiles, max excess/gouge coordinates, and in-tolerance %) via parallel GPU reductions.
- **Differential Oracle Parity:** Verified CPU-vs-GPU Tri-Dexel and `StockErrorReport` parity across 2.5D, 3-axis sculpted meshes, multi-body STL `target_bodies` filtering, and multi-form `ThreadMill` overflow stress tests.

### Milestone 54 — Zero-Readback Direct GPU Stock Rendering (`SimStockRenderSource::GpuGrid`) & Instant Error Heatmap (`plans/54-milestone-54-plan.md`)
- **Direct GPU Stock Rendering (`chip-gui/src/viewport/`):** Added `SimStockRenderSource::GpuGrid` alongside `CpuMesh` so `Heightfield` and `Cylindrical` compute storage buffers render directly via vertex-displaced GPU grids (`vs_sim_heightfield_gpu`, `vs_sim_cylindrical_gpu`) with on-the-fly finite-difference normals—eliminating CPU re-tessellation and PCIe readback during Ultra playback scrubbing—paired with a GPU compact-emit mesh pass (`tridexel_emit.wgsl` + `draw_indirect`) for direct GPU `TriDexel` rendering.
- **Instant GPU Error Heatmap & Tolerance Slider:** Evaluated signed stock-vs-target deviation coloring directly in WGSL using GPU target buffers and a `tolerance_mm` uniform so toggling `Color by Op` vs. `Error Heatmap` or dragging the `±tol` slider updates in `< 0.1 ms` with zero re-meshing.
- **VRAM-Bounded GPU Checkpoints:** Added `GpuCheckpointCache` keeping compact GPU-side checkpoint buffer snapshots (`copy_buffer_to_buffer` restore + automatic CPU spill on constrained adapters) for `< 1 ms` interactive Ultra timeline scrubbing.

### Milestone 55 — GUI/CLI Backend Controls, Benchmarks, Visual Parity, & Spec Sync (`plans/55-milestone-55-plan.md`)
- **GUI & CLI Controls (`chip-gui`, `chip-cli`, `chip-sim`):** Added `SimTimingBreakdown` stage timing (`target_setup_ms`, `sweep_ms`, `metrics_ms`, `total_ms`, `segments_per_sec`) on `SetupSimulationResult`, Simulation Backend selector (`Auto (GPU)` / `GPU` / `CPU`) and active adapter status pill in the Simulation Timeline toolbar persisted in `prefs.toml`, headless `--capture-sim-backend` / `--capture-sim-resolution` capture flags, and `chipcutter simulate --backend <auto|gpu|cpu> --benchmark --json` reporting.
- **Headless Visual Parity & Speedup Benchmarks:** Verified CPU-vs-GPU visual parity screenshots across Heightfield (`cam-torture`), Tri-Dexel (`thread-mill`), and Cylindrical Ultra + Error Heatmap (`rotary-4axis`) via Vulkan/Xvfb, and benchmarked CPU vs. GPU throughput across Standard/Fine/Ultra on all three stock engines.
- **Spec Synchronization:** Updated `specs/01-architecture.md`, `specs/06-simulation.md`, and `specs/10-roadmap.md` to document the `gpu-compute` backend, `SimStockRenderSource::GpuGrid`, and mark Phase 15 complete.

---

## Phase 16 — Toolpath Algorithm Quality & Engagement-Aware Feedrate Optimization (Completed — M56–M59)

Phase 16 leveraged Phase 15's GPU-accelerated Ultra simulation and `chipcutter simulate --benchmark --json` metrology harness to systematically eliminate surface ridges, staircase lifting artifacts, chord gouges, and uncut rest islands across 4th-axis rotary, 3-axis surfacing, and 2.5D strategies, and introduced engagement-aware corner-slowdown and constant-chipload feedrate optimization:

- **Verified v0.15.0 → Phase 16 GPU Benchmark Improvements (`±0.05 mm` tolerance, `0` critical errors across all benchmarks):**
  - `12-simultaneous-4axis` (GPU Ultra): `In-Tol` `17.8%` → **`70.8%`** (`~97.6%` of machined area excluding 11 mm chuck/tailstock ends), `ErrVol` `3,735.9` → **`2,531.3 mm³`**, `RMS` `0.5291` → **`0.4827 mm`**, `max_gouge = 0.015 mm`.
  - `09-rotary-shaft` (GPU Ultra): `In-Tol` `34.0%` → **`56.5%`**, `ErrVol` `5,090.5` → **`3,964.1 mm³`**, `RMS` `0.8843` → **`0.8011 mm`**, `max_gouge` `0.014` → **`0.004 mm`**.
  - `10-rotary-axial` (GPU Ultra): `In-Tol` `16.1%` → **`36.4%`**, `ErrVol` `13,454.6` → **`12,518.7 mm³`**, `max_gouge` `0.019` → **`0.005 mm`**.
  - `11-indexed-step` (GPU Ultra): Fixed STEP polar target rasterization & metrics (`RMS 0.0000*` bug → accurate `RMS = 9.1130 mm`, `In-Tol = 77.7%`, `max_gouge = 0.000 mm`).
  - `09-3d-surface-finishing` (GPU Tri-Dexel Fine): **`gouge_volume`** **`65.1 mm³` → `0.00 mm³`**, **`max_gouge`** **`-0.146 mm` → `0.000 mm`** (100% gouge elimination), `In-Tol` `75.2%` → **`77.5%`**.
  - `08-mesh-surfacing` & `08-3d-adaptive-waterline` (GPU Tri-Dexel Fine): `0.00 mm³` gouge (`max_gouge <= 0.001 mm`), `In-Tol` `53.2% / 54.5%` → **`55.2% / 55.4%`**.
  - `05-adaptive-rest` (GPU Tri-Dexel Fine): `ErrVol` **`131.0` → `27.4 mm³`** (`-79.1%`), `RMS = 0.0346 mm`, `In-Tol = 99.6%`, `gouge_volume = 0.05 mm³`.
  - `18-thread-milling` (GPU Tri-Dexel Fine): `0` critical errors, `In-Tol = 95.5%`, `RMS = 0.0823 mm`.

### Milestone 56 — 4th-Axis Rotary Toolpath Quality & High-Fidelity Cylindrical Verification (`plans/56-milestone-56-plan.md`)
- **Eliminate Staircase Lifting & Coarse Grid Artifacts (`rotary4axis_sim.rs`, `rotary2linear.rs`):** Remove the quantized `sample_safe()` staircase lifts (`z_max + 0.06 * c_spread` and `r_max` jumps that caused `+0.52 mm` systematic bias and `17.8%` in-tolerance surface on `12-simultaneous-4axis` and `16.1–34.0%` on `09/10-rotary-*`). Evaluate exact `f64` `drop_cutter_z_with_scratch` directly along the spiral/flowline curve (backed by `chip-bvh`) paired with adaptive midpoint/sagitta chord subdivision (`<= 0.005 mm`) so convex ridges and helical flanks are both ridge-free and gouge-free.
- **Slope-Compensated Cusp-Height Stepover & Gouge-Free Off-Centerline Tilt:** Scale rotary spiral/flowline stepover accurately for local axial flank slope and target cusp height, re-evaluate exact off-centerline drop-cutter heights after `c_cross` smoothing, and tighten rotary decimation tolerances so helical flutes finish smoothly without visible spiral ridges.
- **High-Fidelity Cylindrical Verification (`chip-stock` CPU & GPU):** Tighten rotary pose subdivision (`<= 0.25°` / `<= 0.5 * du`, fast on Phase 15's GPU sweeper) to eliminate simulation-induced circumferential beading at Ultra resolution, and fix STEP solid polar target rasterization/metrics (`11-indexed-step` `RMS = 0.000` reporting bug).

### Milestone 57 — 3-Axis Surface Finishing & Roughing Quality Polish (`plans/57-milestone-57-plan.md`)
- **Zero-Gouge Exact 3D Chord Refinement (`surface3d/finishing.rs`, `parallel3d.rs`, `scallop3d.rs`, `pencil3d.rs`):** Fix the `dz > 0.08 || xy_dist > 0.6` blind spot in `refine_convex_chords_3d` and tighten `Scallop3D` Eikonal grid resolution & iso-contour projection to eliminate the `-0.146 mm` / `65.1 mm³` gouge measured on `09-3d-surface-finishing` while driving `In-Tol %` dramatically upward.
- **Local-Slope `Waterline3D`, `Adaptive3D` Rest Margins, & Sub-Cell `Pencil3D`:** Replace `Waterline3D`'s global area-radius intermediate stepdown heuristic with true local contour-offset (`dr`) slope refinement, tighten `Adaptive3D` + `Parallel3D` containment/boundary margins (`08-mesh-surfacing` & `08-3d-adaptive-waterline`), and add sub-cell parabolic ridge centering in `Pencil3D` for crisp concave fillet cleanup.

### Milestone 58 — Engagement-Aware Feedrate Optimization & 2.5D Corner Cleanup (`plans/58-milestone-58-plan.md`)
- **Shared Feedrate Optimizer (`crates/chip-cam/src/feed_opt.rs`):** Implement curvature- and engagement-aware feedrate modulation supporting (1) exact internal-arc perimeter-to-centerline scaling `F_center = F_nom * (R_path - R_tool) / R_path` and lookahead deceleration ramps into sharp concave corners, (2) radial chip-thinning modulation across light vs. heavy engagements, and (3) smooth Z-ramp / rotary radius feed transitions.
- **Schema-Safe Config, Inspector UI, & Timeline Telemetry (`chip-cam`, `chip-sim`, `chip-gui`):** Add optional `FeedOptimizationConfig` on `CuttingParams` (default-compatible so Phase 1–2 golden G-code snapshots stay byte-identical), add a two-column **Feed Optimization** card in the Operation Inspector with hover tooltips, surface min/avg/max modulated feedrate telemetry in `SimulationTimeline`, and polish 2.5D Adaptive/Rest & V-Carve corner residuals (`05-adaptive-rest`).

### Milestone 59 — Quantitative Quality Regression Suite, Sample Polish, Visual Verification, & Spec Sync (`plans/59-milestone-59-plan.md`)
- **Automated Quality Regression Gate (`chip-project`, `chip-cli`):** Add deterministic error-metric regression tests asserting hard thresholds on `RMS`, `max_gouge_mm`, `gouge_volume_mm3`, and `in_tolerance_surface_pct` across 4th-axis rotary, 3-axis surface, 2.5D adaptive/rest, and thread-milling benchmarks.
- **Sample Polish, Visual Verification, & Docs Sync (`chip-project`, `docs/`, `specs/`):** Upgrade built-in 4th-axis & 3-axis sample projects to showcase crisp cusp-controlled finishing (updating affected Phase 5 golden snapshots transparently while keeping Phase 1–2 goldens byte-identical), visually verify before/after headless Vulkan screenshots (Ultra + Error Heatmap), and synchronize `specs/05-operations.md` and `specs/10-roadmap.md`.

---

## Phase 17 — Shop-Floor Workholding, Fixture Collision Verification, & Multi-Setup Stock Carryover (Completed — M60–M63)

Phase 17 delivered real shop-floor workholding awareness, full-assembly 3D fixture collision verification, true multi-setup volumetric Tri-Dexel continuity, and shop-floor Flip-Jig & Auto Soft-Jaw automation wizards—building directly on Phase 14's `chip-bvh`/`chip-geom2d`/`chip-stock` architecture and Phase 15's GPU-accelerated simulation while preserving 100% lossless `.chip.toml` backward compatibility and byte-for-byte Phase 1–2 golden G-code parity (with `G38.2` probing operations explicitly retained in Future Phases):

### Milestone 60 — 3D Fixture, Vise, & Clamp Workholding Model, Persistence, & Viewport Rendering (`plans/60-milestone-60-plan.md`)
- **Schema-Safe Workholding Model (`chip-cam/src/workholding.rs`):** Add optional `fixtures: Vec<FixtureBody>` on `Setup` (omitted when empty for zero diff on existing projects) supporting parametric **`ViseJaws`** (fixed/movable jaws, step shelves, parallels auto-clamped to stock), **`ToeClamp`** (placed strap/toe clamps with overhang & bolt stud), **`Spoilboard`** (sacrificial bed with allowed overcut margin), **`BoxKeepOut` / `CylinderKeepOut`**, and **`ImportedModel`** (`.step`/`.stl`/`.obj`/`.3mf` fixture solids bundled inside `.chipz` and automatically excluded from target-part gouge metrics).
- **Exact `f64` Resolution (`chip-bvh`, `chip-geom2d`):** Resolve every enabled fixture into exact Setup-WCS `TriMesh3D` + `FlatBvh` collision volumes and 2D XY keep-out footprints (`RegionSet2D`) with configurable `clearance_margin_mm`.
- **Setup `[Fixtures]` Tab & Viewport Rendering (`chip-gui`, `chip-cli`):** Add a strict two-column **`[Fixtures]`** tab in Setup Inspector (1-click preset creation, interactive 3D viewport click-to-place for clamps/keep-outs, `< 1 ms` Undo/Redo), translucent `wgpu` fixture shading + `Fixtures` viewport visibility toggle, and `chipcutter workholding list/inspect` CLI helpers.

### Milestone 61 — Automatic Safe-Rapid Routing & 3D Fixture Collision Verification (`plans/61-milestone-61-plan.md`)
- **Fixture-Aware Safe-Rapid & Linking Routing (`chip-cam/src/fixture_avoidance.rs`):** Automatically inspect non-cutting rapids and stay-down links against enabled fixture `FlatBvh` volumes (accounting for full flute, shank, and holder/collet radial envelopes plus `clearance_margin_mm`), lifting colliding rapids cleanly over clamps/jaws without silently mutating cutting passes—leaving projects with no fixtures byte-identical.
- **Rule 6: Full-Assembly `FixtureCollision` Verification (`chip-sim`, `chip-stock`):** Add `SimIssueKind::FixtureCollision` (`SimSeverity::CriticalError`, export-blocking and clickable to jump camera + timeline) checking swept flute, shank, and holder/collet envelopes against every enabled fixture across `Heightfield`, `TriDexel`, and `Cylindrical` engines with 100% CPU-vs-GPU parity, plus crimson collision highlighting in the 3D viewport.

### Milestone 62 — Multi-Setup In-Process Volumetric Stock Carryover (`G54` -> `G55`) & Rest-Aware Op-2 Machining (`plans/62-milestone-62-plan.md`)
- **Cross-Setup Tri-Dexel Carryover (`chip-stock/src/tridexel/transfer.rs`, `chip-cam`):** Add `StockConfig::FromPrecedingSetup` transferring Setup `N`'s exact final `TriDexelStock` into Setup `N+1`'s oriented WCS frame (`T_{N+1 -> N}`)—using an exact zero-blur plane permutation/flip fast path for `90°`/`180°` flip setups and exact BVH surface re-voxelization for arbitrary oblique orientations.
- **Rest-Aware Op-2 Toolpaths & Cumulative Metrology (`chip-cam`, `chip-project`, `chip-sim`, `chip-gui`, `chip-cli`):** Pass carried-over remaining stock into Op-2 roughing/facing/pocketing (`Adaptive3D`, `Waterline3D`, `Facing`, `Pocket2D`, `Adaptive2D`) to skip already-machined air regions, initialize Op-2 simulation playback from Op-1's real machined solid (at `t = 0.0`) for accurate Op-2 crash detection and cumulative `ErrVol` / `RMS` / `In-Tol %` metrics, and support `chipcutter simulate --all-setups` + Setup `[Stock]` carryover UI preview.

### Milestone 63 — Two-Sided Flip-Jig & Auto Soft-Jaw Wizards, Built-In Workholding Sample, & Spec Sync (`plans/63-milestone-63-plan.md`)
- **Two-Sided Flip-Jig Dowel-Pin Wizard (`chip-cam`, `chip-gui`, `chip-cli`):** Automatically place collision-free dowel-pin registration holes strictly symmetric about the chosen flip axis (`X` or `Y`), creating the `Flip-Jig Pins` sketch, stock/spoilboard pin-boring operations, paired `180°` Op-2 Setup (`G55`) with `FromPrecedingSetup` stock carryover, and matching pin keep-out fixture bodies.
- **Auto Soft-Jaw Generator Wizard (`chip-cam`, `chip-gui`, `chip-cli`):** Project the exact `f64` clamping-band part silhouette (with radial clearance and internal corner relief fillets) onto fixed/movable vise jaws—generating both ready-to-machine aluminum soft-jaw setups/toolpaths AND watertight 3D-printable `.stl` soft-jaw meshes (`chipcutter workholding soft-jaws --export-stl`).
- **Built-In Sample, Verification Gate, & Spec Sync:** Add built-in `two-sided-vise` sample project demonstrating Op-1 vise/clamp avoidance + Op-2 flipped Tri-Dexel stock carryover + soft jaws, lock down end-to-end workholding/carryover/wizard regression tests & headless Vulkan visual verification, and synchronize `specs/03-setups-wcs.md`, `specs/06-simulation.md`, `specs/08-ui-viewport.md`, `specs/09-persistence-cli.md`, `specs/10-roadmap.md`, and `AGENTS.md`.

---

## Phase 18 — Expanded Machining Capabilities: 3D Edge Deburring & Undercut / Slot / T-Slot / Dovetail Milling (Completed — M64–M67)

Phase 18 expands ChipCutter's cutter catalog, volumetric Tri-Dexel undercut simulation (CPU & GPU), and toolpath strategy suite to machine real-world **3D non-planar chamfered/deburred edges**, **keyways**, **T-slots**, **dovetail rails/slots**, and **3D lollipop undercuts**—building directly on `AnalyticCutterEnvelope`, `chip-bvh`, and `TriDexelStock` while preserving 100% schema compatibility, zero new external dependencies, and byte-for-byte Phase 1–2 golden G-code parity (leaving **2D Sheet Nesting & Multi-Part Arrays** and **PCB Isolation Milling & Drilling** in Future Phases):

### Milestone 64 — Undercut Cutter Geometries (`Lollipop`, `Slot / T-Slot`, `Dovetail`), Tri-Dexel Undercut Simulation (CPU + GPU), & Tool Library UI (`plans/64-milestone-64-plan.md`)
- **Three New Cutter Geometries (`chip-tools`):** Add `CutterGeometry::LollipopEndmill`, `CutterGeometry::SlotMill`, and `CutterGeometry::DovetailCutter` (with relieved `neck_diameter_mm` & `neck_length_mm`), exact non-monotonic `AnalyticCutterEnvelope` radial & axial interval math (`axial_z_interval_at_radius`), Minkowski stock allowance dilation, Fusion 360 `.json`/`.tools` import mapping, and non-colliding starter library presets.
- **CPU + GPU Volumetric Undercut Sweeping & Revolved 3D Cutter Meshes (`chip-stock`, `chip-sim`):** Extend `GpuToolProfile` (`SHAPE_LOLLIPOP`, `SHAPE_SLOTMILL`, `SHAPE_DOVETAIL` within the existing 64-byte layout), CPU & WGSL `TriDexelStock` / `HeightfieldStock` / `CylindricalStock` sweepers, generalized relieved-neck collision verification (catching neck rubs against overhanging ceilings and un-cleared slot plunges), and 3D revolved cutter meshes in `build_cutter_assembly_mesh`.
- **Tool Library Manager UI (`chip-gui`):** Add strict two-column editors, hover tooltips, and procedural 2D schematics for Lollipop, Slot/T-Slot, and Dovetail cutters.

### Milestone 65 — 3D Edge Deburring & Non-Planar Chamfering (`OperationKind::Deburr3D`) (`plans/65-milestone-65-plan.md`)
- **Exact 3D B-Rep Edge Following & Bisector Contact (`chip-cam`):** Add `OperationKind::Deburr3D` following arbitrary curved, sloped, stepped, and multi-edge 3D B-Rep chains (explicit selection or auto-detected sharp edges above `min_dihedral_angle_deg`) using Chamfer Mill, Ball Endmill, or Lollipop Endmill (including underside back-chamfering with Lollipop!).
- **Automatic Neighbor-Wall Collision Trimming (`chip-cam`, `chip-sim`, `chip-gui`):** Evaluate full cutter/neck/shank clearance against target `FlatBvh` to trim cleanly away from adjacent inside corners/walls by `wall_clearance_mm` with tangent lead-in/out, fixture-aware safe rapids, strict two-column **3D Deburr Passes** Inspector UI, and CPU/GPU Tri-Dexel verification.

### Milestone 66 — Undercut Milling Strategies: Keyways, T-Slots, Dovetails, & Lollipop 3D Undercuts (`plans/66-milestone-66-plan.md`)
- **Keyway & T-Slot Milling (`OperationKind::SlotUndercut`):** Supports open/blind keyways and T-slot undercuts (with standard metric/imperial presets, optional throat-clearance validation, multi-pass radial stepovers & Z-levels, mandatory safe open-end / entry-bore lead-in, and guaranteed horizontal exit before any `+Z` retract).
- **Dovetail Milling (`OperationKind::Dovetail`):** Supports straight & contoured dovetail slots and external dovetail rails with throat-clearance validation, progressive lateral stepovers from throat to final angled wall + spring pass, and safe throat-centered retracts.
- **Lollipop 3D Undercut Finishing (`OperationKind::Undercut3D`):** Horizontal Z-level contour finishing across overhanging downward-facing 3D B-Rep/mesh surfaces (`n_z < 0`) using exact upper-hemisphere contact with automatic relieved-neck ceiling-clearance clipping, plus strict two-column Inspector cards and safety banners in `chip-gui`.

### Milestone 67 — Built-In Undercut & 3D Deburr Sample, E2E Regression Suite, Visual Verification, & Spec Sync (`plans/67-milestone-67-plan.md`)
- **Programmatic STEP Fixture & Built-In Sample (`chip-cad`, `chip-project`, `chip-gui`, `chip-cli`):** Add `sample_undercut_deburr_block_step_bytes()` (`--undercut-block`) and built-in sample **`undercut-deburr`** combining pre-slot clearing, T-Slot, `60°` Dovetail, Lollipop 3D Undercut, and 3D Edge Deburr in a single crash-free Setup.
- **E2E Regression, CPU/GPU Parity, Visual Verification, & Spec Sync:** Add `crates/chip-project/tests/expanded_machining_e2e.rs` (including fault-injection tests for overhang neck crashes & vertical undercut retracts, Tri-Dexel accuracy gates, and GRBL/LinuxCNC post parity), verify headless Vulkan Tri-Dexel screenshots, and synchronize `specs/04`, `05`, `06`, `08`, and `10`.

---

## Phase 19 — Complete Feature Tutorials, Expanded Sample Projects, & Gallery Parity (Completed — M68–M71)

Phase 19 closes every remaining tutorial, built-in sample, starter-asset, and Sample Gallery documentation gap across Phases 15–18 (and earlier `.svg`/`.3mf` import formats) so **100% of shipped ChipCutter strategies, cutter geometries, workholding wizards, GPU simulation/feed-optimization features, and file formats** have both a ready-to-run built-in sample project and an illustrated, headless-verified Markdown tutorial and Sample Gallery page—without changing core CAM/simulation algorithms or breaking existing Phase 1–5 golden G-code parity:

### Milestone 68 — Gap-Closing Built-In Sample Projects & Minimal Tutorial Starter Assets (`plans/68-milestone-68-plan.md`)
- **Six New Built-In Sample Projects (`chip-project`, `chip-gui`, `chip-cli`):** Add crash-free, CPU/GPU-verified built-in samples covering every feature that previously lacked a dedicated 1-click sample:
  - **`feed-opt-pocket`** (Phase 16 engagement-aware `FeedOptimizationConfig`: internal-arc centerline scaling, sharp-corner lookahead deceleration ramps, radial chip-thinning boost, and timeline feedrate telemetry).
  - **`flip-jig-softjaws`** (Phase 17 `Spoilboard` + **Flip-Jig Dowel-Pin Wizard** registration holes + `CylinderKeepOut` pins + `G54 -> G55` Tri-Dexel carryover + generated **Auto Soft-Jaw** machining setup).
  - **`curved-deburr-3d`** (Phase 18 non-planar sloped-boss & stepped-pocket `Deburr3D` with `ChamferMill` inside-corner wall collision trimming, `BallEndmill`, and `LollipopEndmill` underside back-chamfering).
  - **`dovetail-rail-keyway`** (Phase 18 `DovetailMode::ExternalRail` external dovetail rail + blind `SlotUndercutMode::Keyway` entered via `UndercutEntryMode::EntryBoreCenter`).
  - **`svg-badge`** (Pure-2D `.svg` import with `96 DPI` cubic-Bézier biarc `G2`/`G3` fitting and nested counter-islands) and **`mesh-3mf-bracket`** (watertight unit-aware `.3mf` 3D mesh surfacing).
- **Minimal Tutorial Starter Assets & Dynamic Catalog Counts (`docs/tutorials/assets/`, `chip-cli`, `chip-gui`):** Add reproducible `--tutorial` CLI generators and checked-in starter files (`20-feed-opt-pocket.step`, `21-sloped-boss-deburr.step`, `21-overhang-flange.step`, `22-undercut-deburr-block.step`, `13-badge-logo.svg`, `08-bracket.3mf`), and replace hardcoded sample count labels in `welcome_gallery.rs` with dynamic `SAMPLE_CATALOG.len()` formatting.

### Milestone 69 — Illustrated Tutorial 20: GPU-Accelerated Simulation, Benchmarking, & Feedrate Optimization (`plans/69-milestone-69-plan.md`)
- **Tutorial 20 (`docs/tutorials/20-gpu-simulation-feed-optimization.md`):** Standardized 7-part illustrated walkthrough covering Phase 15's `wgpu` Compute backend (`Auto (GPU)` / `GPU` / `CPU`, adapter badge & stage-timing tooltips, instant `< 0.1 ms` GPU Error Heatmap & live `±tol` slider, `chipcutter simulate --backend gpu --resolution ultra --benchmark --json`) and Phase 16's engagement-aware **Feed Optimization** card (`Internal-Arc Scaling`, `Sharp-Corner Lookahead`, `Radial Chip-Thinning`, `SimulationTimeline` min/avg/max feedrate telemetry, and cusp-height surface quality tuning) using `20-feed-opt-pocket.step` / `--sample feed-opt-pocket`.
- **Automated Headless Vulkan Captures (`scripts/capture_tutorial_screenshots.sh`, `docs/images/tutorials/20-gpu-simulation-feed-optimization/`):** Add repeatable Vulkan screenshot captures, visually inspect every PNG, and cross-link from `17-tridexel-simulation-error-metrics.md` & `docs/tutorials/README.md`.

### Milestone 70 — Illustrated Tutorials 21 & 22: 3D Edge Deburring & Undercut / T-Slot / Dovetail / Lollipop Milling (`plans/70-milestone-70-plan.md`)
- **Tutorial 21 (`docs/tutorials/21-3d-edge-deburring-nonplanar-chamfering.md`):** Illustrated 7-part walkthrough covering `LollipopEndmill` setup in Tool Library, explicit 3D curved/sloped B-Rep edge selection vs. `Auto-Detect Sharp Edges` (`min_dihedral_angle_deg`, `exclude_vertical_edges`), automatic `FlatBvh` inside-corner wall collision trimming (`wall_clearance_mm`) on `21-sloped-boss-deburr.step`, underside overhang back-chamfering on `21-overhang-flange.step`, and GPU Tri-Dexel verification.
- **Tutorial 22 (`docs/tutorials/22-undercut-tslot-dovetail-lollipop.md`):** Illustrated 7-part walkthrough covering `SlotMill`, `DovetailCutter` (with derived `Top Ø`), and `LollipopEndmill` Tool Library configuration, pre-slot throat clearing & fault-injection safety checks, DIN 650 M8 T-Slot (`BothEndsThrough`) & blind Keyway (`EntryBoreCenter`), `60°` Internal Dovetail Slot & External Dovetail Rail, Lollipop 3D overhang undercut finishing (`Undercut3D` with relieved-neck ceiling clipping), auto-Tri-Dexel `chipcutter build`, and GRBL/LinuxCNC posting.
- **Automated Headless Vulkan Captures (`scripts/capture_tutorial_screenshots.sh`, `docs/images/tutorials/21-*/`, `22-*/`):** Add repeatable capture functions and visually verify every tutorial screenshot.

### Milestone 71 — Full Sample Projects Gallery Parity (`docs/samples/`), GitHub Pages Sync, & Documentation Verification Gate (`plans/71-milestone-71-plan.md`)
- **100% Sample Gallery Parity (`docs/samples/`, `docs/images/samples/`, `scripts/capture_sample_gallery.sh`):** Harden `capture_sample_gallery.sh` & `capture_tutorial_screenshots.sh` against stale X11 sockets, generate full hero, `480 px` thumbnail, and per-Setup/per-Operation completion screenshots for all 8 undocumented/new samples (`two-sided-vise`, `undercut-deburr`, `feed-opt-pocket`, `flip-jig-softjaws`, `curved-deburr-3d`, `dovetail-rail-keyway`, `svg-badge`, `mesh-3mf-bracket`), and publish dedicated `docs/samples/<slug>.md` pages plus updated `docs/samples/README.md`, `docs/tutorials/README.md`, `docs/index.html`, and `README.md`.
- **Automated Parity & Link Audit (`chip-project`, `specs/10-roadmap.md`):** Add automated tests verifying every sample in `SAMPLE_CATALOG` loads, generates, simulates with `0` critical errors, and has a matching `docs/samples/<slug>.md` page and valid PNG links, audit all Markdown links/images across `docs/` and `README.md`, and mark Phase 19 complete in `specs/10-roadmap.md`.

---

## Phase 20 — Overhang-Safe & Helical-Loft Toolpathing Across 3-Axis, 2.5D, Indexed `3+1`, & 4th-Axis Milling (Completed — M72–M75)

Building on `v0.19.1`'s monotonic `PushCutterField` fix for overhanging helical lofts in `Adaptive3D` and `Waterline3D` (#23), Phase 20 systematically hardens every remaining toolpathing, rest-machining, workholding-silhouette, and 4th-axis rotary kernel across ChipCutter against overhanging, cantilevered, and continuously twisting helical geometry—while preserving 100% lossless `.chip.toml` backward compatibility and byte-for-byte parity on existing Phase 1–5 golden G-code snapshots:

### Milestone 72 — Full-Assembly Shank/Holder Overhang Avoidance & 2.5D Overhang Safety Guards (`plans/72-milestone-72-plan.md`)
- **Full-Assembly Target-Part Clearance (`chip-cam`):** Extend `SurfaceCutterProfile`, `drop_cutter_z`, and `PushCutterField` with optional multi-tier shank (`shank_diameter_mm > cutting_diameter_mm`) and collet/holder (`dz >= stickout_length_mm`) clearance envelopes derived from `ToolAssemblyProfile`, automatically protecting overhanging helical blades from upper-shank and holder crashes with reach-limit diagnostics.
- **2.5D Overhang Collision Guard & Opt-In Model Avoidance (`chip-cam`, `chip-gui`):** Automatically warn when 2.5D operations (`Pocket2D`, `Contour2D`, `DrillBore`, `Facing`, `ChamferTrace`) plunge through overhanging 3D solids above selected features, and add a schema-safe `PassesConfig::avoid_model_overhangs` toggle (default `false`) that trims/clips 2.5D regions and hole columns cleanly around overhanging geometry.

### Milestone 73 — `Indexed3Plus1` Cross-Angle Rest Machining, Volumetric Carryover, & Swept Slab Excess Masks (`plans/73-milestone-73-plan.md`)
- **Angle-Scoped 2D Rest & Cross-Angle Volumetric `TriDexelStock` Carryover (`chip-cam`, `chip-stock`, `chip-project`):** Tag `UpstreamSweptInput` with its rotary index angle so 2D `swept_toolpath_coverage_2d` never mixes coordinates across different `Indexed3Plus1` orientations (`0°` vs. `120°`/`240°`), and wire in-setup sequential `TriDexelStock` carryover across indexed angles via `TriDexelStock::transform_to_frame` to skip already-cleared outer stock safely in 3D.
- **Swept Z-Slab Target-Part Masking (`chip-cam`):** Replace sparse 1–2 Z-plane `slice_surface_contours_z` subtraction in `Adaptive3D` and `Waterline3D` carryover excess-stock masks with an exact swept Z-slab XY footprint so rapidly twisting helical blades never leave false excess-stock slivers between sample planes.

### Milestone 74 — Overhang-Aware 3D Finishing (`Parallel3D`, `Scallop3D`, `Pencil3D`, Rest Masks) & Swept Soft-Jaw Silhouettes (`plans/74-milestone-74-plan.md`)
- **Overhang-Shadow Cliff Suppression (`chip-cam`):** Upgrade `compute_morphological_rest_residual`, `build_3d_rest_mask`, and `Pencil3D` to distinguish genuine concave valleys/fillets from vertical overhang drop-off discontinuities, eliminating spurious overhang-shadow pencil traces and false rest bands.
- **Cliff-Safe `Scallop3D` & `Parallel3D` + Exact Swept Soft-Jaw Silhouettes (`chip-cam`):** Guard `Scallop3D`'s Fast Marching `sec(θ)` slope weights against overhang-shadow discontinuities, split `Parallel3D`/`Scallop3D` chains cleanly at vertical overhang drop-offs instead of diving at full `cut_feed`, and replace the 9-slice `workholding_wizards.rs` union with a continuous swept clamping-band silhouette so generated soft jaws clear twisted helical crests without scalloping.

### Milestone 75 — Helical-Loft 4th-Axis Rotary Improvements, Benchmark Suite, & Phase 20 Spec Sync (`plans/75-milestone-75-plan.md`)
- **Verified Partial-Sector `ZigZag` Linking & Multi-Offset `SurfaceNormalAutoTilt` (`chip-cam`):** Precompute exact `drop_cutter_rotary_z_rel`-verified axial lane links for partial-sector `ZigZag` passes in `Rotary2Linear` and `Rotary4AxisSim`, upgrade `SurfaceNormalAutoTilt` with circumferential multi-offset flank probing beneath overhanging helical crests, and emit reachability diagnostics for axial (`u`-direction) helical undercuts.
- **Helical-Loft Benchmark, E2E Regression Suite, & Spec Sync (`chip-cad`, `chip-project`, `specs/`):** Add a watertight programmatic multi-blade helical-loft benchmark fixture, lock down CPU & GPU Tri-Dexel/Cylindrical E2E regression tests covering M72–M75, verify Phase 1–5 golden G-code parity, and synchronize `specs/05-operations.md`, `specs/06-simulation.md`, and `specs/10-roadmap.md`.

---

## Phase 21 — Workspace Decluttering, Settings Page, Streamlined Tool Management, & Simulation Mesh Skinning (Completed — M76–M79)

Focuses on day-to-day usability and visual polish across the workspace: moving infrequently changed preferences off-screen into a dedicated Settings page, simplifying Setup cards and top-action-bar operation creation around a clearly indicated Active Setup, streamlining project tool selection/pruning and machine-switch tool replacement with a single `"Tools"` button, and adding smooth crease-preserving simulation mesh skinning across CPU & GPU engines.

### Milestone 76 — Unified Settings Page (`prefs.toml`) & Workspace Chrome Decluttering (`plans/76-milestone-76-plan.md`)
- **Dedicated Settings Page (`chip-gui`):** Add a categorized `Settings` modal (`General & Appearance`, `Viewport & Navigation`, `Simulation`, `Machine Defaults`) reachable via `Edit → Settings…` (`Ctrl+,`), Command Palette, and a compact toolbar `Settings` button, extending `prefs.toml` with backward-compatible `sim_resolution` and `sim_mesh_skinning` fields.
- **On-Screen Chrome Decluttering (`chip-gui`):** Move low-frequency `Engine` / `Resolution` / `Backend` dropdowns off the Simulation Timeline bar (leaving a compact clickable summary chip into `Settings → Simulation`), move `MousePreset` and `OrbitMode` off the floating viewport HUD, and move global `Theme` and default `Units` toggles off the top action bar.

### Milestone 77 — Active-Setup Indication, Top Action Bar Operation Creation & Separators, and Setup Card Simplification (`plans/77-milestone-77-plan.md`)
- **Active-Setup Highlight & Setup Card Cleanup (`chip-gui`):** Remove per-setup `+ Adap` / `+ Cont` / `+ VCarve` / `+ Op…` rows from CAM Browser Setup cards and make the Active Setup unmistakably clear with an accent border, tinted header, `"Active"` badge pill, and 1-click activation.
- **Grouped Top Action Bar Operation Creation (`chip-gui`):** Route top-action-bar operation creation (`+ Sketch`, `+ Adaptive`, `+ Contour`, `+ V-Carve`, and categorized `+ Op ▾`) cleanly to the active setup with explicit `ui.separator()` dividers between File/History, Geometry/Operations, Pipeline, Tools, and Utility groups.

### Milestone 78 — Streamlined Tool Management, Single `"Tools"` Entry Point, `Project Tools` Controls, & Interactive Machine-Switch Tool Swapping (`plans/78-milestone-78-plan.md`)
- **Single `"Tools"` Entry Point & Actionable `Project Tools` Card (`chip-gui`, `chip-project`):** Consolidate the three duplicate `"Tools"` buttons down to one (top action bar), add inline `+ Add`, per-tool `Remove` (with usage badges), and 1-click `Remove Unused (N)` controls directly on the `Project Tools` card, and improve the operation `Tool` picker to group compatible Project & Machine tools with search/filtering.
- **Interactive Machine-Switch Tool Swapping (`chip-project`, `chip-gui`):** Whenever switching the project's target machine from Setup Inspector, Machine Manager, or Tool Library, automatically detect operations referencing tools missing from the new machine and open a guided **Swap Machine Tools** dialog pre-selecting closest compatible replacements and applying the swap + optional unused-tool pruning as one atomic Undo step.

### Milestone 79 — Simulation Mesh Skinning (Smooth Surface Display on CPU & GPU), Visual Verification, & Phase 21 Spec Sync (`plans/79-milestone-79-plan.md`)
- **Crease-Preserving Simulation Mesh Skinning (`chip-stock`, `chip-sim`, `chip-gui`):** Wire `prefs.sim_mesh_skinning` from `Settings → Simulation` into live CPU & GPU `TriDexelStock` (crease-preserving tangential Taubin relaxation + per-vertex crease-aware smooth normals + 32-byte octahedral `PackedTriDexelQuad` GPU parity), `HeightfieldStock`, and `CylindricalStock` meshers—keeping 90° edges razor-sharp and volumetric error/crash metrology 100% invariant.
- **Visual Verification & Spec Sync (`chip-stock`, `chip-sim`, `specs/`):** Add smoothness, watertightness, sharp-crease, CPU-vs-GPU parity, and metrology-invariance tests, visually verify headless Vulkan before/after renders, and synchronize `specs/06-simulation.md`, `specs/08-ui-viewport.md`, and `specs/10-roadmap.md`.

---

## Phase 22 — Unified Cutting Area & Machining Boundary Controls Across Operations (Completed — M80–M83)

Standardizes how machinists specify *what area to cut* across 3-axis, 4th-axis rotary, undercut/deburr, and 2.5D operations: replacing fake AABB silhouettes with true projected model silhouettes, giving both 4th-axis rotary operations full `-U`/`+U` axial boundary source, containment, signed offset, and outer radial ceiling controls, upgrading 2.5D Facing and Pocket/Adaptive cutting areas, consolidating all boundary controls into the Operation Inspector `[Geometry]` tab, and adding live 3D viewport boundary previews.

### Milestone 80 — Unified 3D Machining Boundary Schema, True Model Silhouette, & `Undercut3D`/`Deburr3D` Containment (`plans/80-milestone-80-plan.md`)
- **Shared `MachiningBoundaryConfig` & True `ModelSilhouette` (`chip-cam`, `chip-project`):** Consolidate duplicated `boundary_mode` / `tool_containment` / `boundary_offset` fields into a single backward-compatible `MachiningBoundaryConfig` (`#[serde(flatten)]`), replace `resolve_3d_containment_region`'s rectangular AABB fallback for `ModelSilhouette` with true projected 2D mesh silhouette polygons (reusing `slab_projection.rs`), preserve true circular XY footprints for cylindrical Z-axis stock, and fix `Adaptive3D`'s `StockBox` bug (honoring `tool_containment` and signed `boundary_offset`).
- **3D Surface Parity Across `Undercut3D` & `Deburr3D` (`chip-cam`):** Wire `MachiningBoundaryConfig` into `Undercut3D` (replacing its hardcoded `ToolCenterOnBoundary` / `0.0 mm` containment) and `Deburr3D` (constraining `auto_detect_edges` within the resolved XY boundary), and emit structured diagnostics instead of silent fallbacks when `SelectedRegions` has no selections.

### Milestone 81 — 4th-Axis Rotary Axial Boundary Unification (`-U`/`+U` Bounds, Containment, Radial Ceiling, & Wrapper Chuck Safety) (`plans/81-milestone-81-plan.md`)
- **Shared `RotaryAxialBoundaryConfig` for `Rotary2Linear` & `Rotary4AxisSim` (`chip-cam`, `chip-project`):** Replace inward-only `axial_margin_start/end` with a backward-compatible `RotaryAxialBoundaryConfig` supporting `StockBounds`, `ModelBounds`, `SelectedGeometry` (projecting selected 3D faces/loops onto `-U..+U`), and `ExplicitRange { u_start, u_end }`, paired with axial `ToolContainmentMode` (`Center` / `Inside` / `Outside`) and signed start/end offsets (allowing controlled shoulder overrun while strictly enforcing the rotary chuck keep-out guard).
- **Radial Envelope & Rotary Wrapper Safety (`chip-cam`):** Honor `heights.top_z_wcs` as an outer radial roughing ceiling (eliminating air-cutting on stepped shafts) alongside `heights.bottom_z_wcs` as core radius floor, and enforce optional axial/angular limits plus mandatory rotary chuck keep-out protection on `WrapCylinder` and `Indexed3Plus1`.

### Milestone 82 — 2.5D Cutting Area Consistency & Inspector Control Hygiene (`plans/82-milestone-82-plan.md`)
- **Exact Polygon `Facing` & `Pocket2D`/`Adaptive2D` Outer-Bound Clearing (`chip-cam`):** Upgrade `Facing` to adopt `MachiningBoundaryConfig` (`StockBox`, `ModelSilhouette`, `SelectedRegions`, `tool_containment`, `boundary_offset`) and clip facing passes to the exact selected polygon + raised islands rather than collapsing selections to a rectangular `Aabb2`; add `StockContour` (clear around selected bosses up to stock bounds) and sketch/loop open-edge support to `Pocket2D`/`Adaptive2D`, plus optional XY containment clipping on `Contour2D`.
- **Strategy Parameter & Height Hygiene (`chip-cam`, `chip-gui`):** Hide unused `Multiple Depths` / `Radial Stock` controls on strategies that ignore them (`Parallel3D`, `Scallop3D`, `Pencil3D`, `ThreadMill`), clarify `VCarve` / `EdgeChamfer` height behavior, and expose `SlotUndercut` / `Dovetail` stock-to-leave consistently.

### Milestone 83 — Unified `[Geometry]` Tab Boundary Controls, Live 3D Viewport Previews, & Phase 22 Spec Sync (`plans/83-milestone-83-plan.md`)
- **Consolidated `[Geometry]` Cutting-Area UI (`chip-gui`):** Move all boundary controls (`MachiningBoundaryConfig`, `RotaryAxialBoundaryConfig`, angular sector bounds, and `Pocket2D` stock-contour options) out of `[Passes]` into `[Geometry]` right beside selections, replacing the misleading rotary 3D banner with accurate `-U`/`+U` axial boundary controls and live resolved-dimension readouts.
- **Live 3D Viewport Cutting-Area Overlays & Spec Sync (`chip-gui`, `specs/`):** Render live 3D viewport overlays whenever `[Geometry]` is active—crisp boundary polygon + dashed effective tool-center containment loop for 2.5D/3-axis ops, and translucent cylindrical `[u_start, u_end] × [start_deg, end_deg]` sector bands with `-U`/`+U` endcap rings and chuck keep-out plane for 4th-axis rotary ops—and synchronize `specs/05-operations.md`, `specs/08-ui-viewport.md`, and `specs/10-roadmap.md`.

---

## Phase 23 — Per-Operation Keep-Out Regions, Model Bounds, Sketch Quick-Select Polish, & Indexed 4th-Axis Auto-Rotate (Completed — M84–M86)

Completes the cutting-area workflow started in Phase 22 by adding an explicit rectangular **Model Bounds** XY boundary source, auto-expanding **2D Sketch Quick-Select** controls, automatic **Indexed 3+1 viewport rotation** so the machined face sits upright under `+Z` while editing, and first-class **Per-Operation Avoid / Keep-Out Regions** (faces, 2D sketch loops, and Setup workholding fixtures) subtracted directly from cutting areas with red-tinted 3D viewport overlays.

### Milestone 84 — `Model Bounds` (`ModelBoundingBox`), Sketch Quick-Select Auto-Expand, & Indexed 4th-Axis Auto-Rotate Viewport (`plans/84-milestone-84-plan.md`)
- **Explicit `Model Bounds` (`MachiningBoundaryMode::ModelBoundingBox`) with Per-Side Signed Offsets (`chip-cam`, `chip-gui`):** Add a rectangular XY model bounding-box boundary mode with independent signed `-X`, `+X`, `-Y`, and `+Y` side offsets (positive expands outward, negative insets inward) alongside `Stock Bounds`, `Model Silhouette`, and `Selected 2D Contours` across all 8 boundary-aware strategies (`Adaptive3D`, `Waterline3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`, `Undercut3D`, `Deburr3D`, `Facing`) and `BoundaryPreviewOverlay`.
- **Sketch Quick-Select Polish & Indexed 4th-Axis Auto-Rotate (`chip-gui`, `chip-cam`):** Auto-expand **`2D Sketch Layers & Quick Select`** (with per-entity quick-add and empty-state `+ New Sketch` shortcut) when switching to `Selected 2D Contours`, and automatically rotate the displayed 3D model, raw setup stock wireframe, chuck visuals, picking raycasts, Z-height planes, and `BoundaryPreviewOverlay` to match the selected `Indexed 3+1` operation's `angle_deg` (with a toggle in `4th-Axis Indexing / Wrapping` & `Settings → Viewport`).

### Milestone 85 — Per-Operation & Shared Avoid / Keep-Out Regions & Fixture-Trimmed Cutting Areas Kernel (`plans/85-milestone-85-plan.md`)
- **Backward-Compatible `AvoidRegionConfig`, Reusable `SharedAvoidRegion`, & Resolution Kernel (`chip-cam`, `chip-project`):** Add project-level named `SharedAvoidRegion` definitions (`ProjectDocument::avoid_regions`, linkable across multiple operations within a Setup or across Setups) alongside `Operation::avoid: AvoidRegionConfig` (`shared_region_ids`, local `selections`, `fixture_ids`, `avoid_all_fixtures`, `radial_clearance`, `axial_clearance`)—all hashed in `OperationInputHash` and re-matched on associative STEP reload—and implement `resolve_avoid_region_2d` projecting selected 3D faces, 2D sketch loops, and Z-overlapping `ResolvedFixtureSolid` bodies into exact keep-out regions.
- **Cross-Strategy Cutting-Area Subtraction (`chip-cam`, `chip-sim`):** Subtract merged local + shared keep-out regions cleanly across 3-axis/3D ops (`Adaptive3D`, `Waterline3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`, `Undercut3D`, `Deburr3D`), 2.5D ops (`Facing`, `Pocket2D`/`Adaptive2D`, `Contour2D`, `VCarve`, `DrillBore` hole skip with diagnostics), and 4th-axis rotary axial keep-out bands—verified crash-free and fixture-collision-free in `chip-sim`.

### Milestone 86 — Avoid / Keep-Out `[Geometry]` UI, Shared Region Manager, Red-Tinted 3D Overlays, Visual Verification, & Phase 23 Spec Sync (`plans/86-milestone-86-plan.md`)
- **Interactive Keep-Out Controls, Shared Region Reuse, Fixed-Box Alignment Controls, & Red 3D Overlays (`chip-gui`):** Add an **`Avoid / Keep-Out Regions`** card in `[Geometry]` (1-click shared region toggle chips, `Save Local as Shared…`, `Edit` / `Detach to Local` / `Apply to Setup Ops…`, fixture chips, `Avoid All Setup Fixtures` toggle, Cutting vs. Avoid pick-target toggle, sketch quick-add, clearance inputs, CAM Browser `Keep-Out Regions` management with atomic Undo), add `XY Alignment` & `Z Alignment` mode selectors to `Fixed-Size Box Stock` (`Stock Offsets`, defaulting to `Centered in Z` on rotary Setups), and render nominal/effective keep-out zones in high-contrast coral/red wireframe & translucent fill in the 3D viewport.
- **Visual Verification & Spec Sync (`chip-gui`, `specs/`):** Visually verify headless Vulkan screenshots of auto-rotated `Indexed 3+1` (`Model Bounds`) and fixture/sketch keep-out trimming, and synchronize `specs/05-operations.md`, `specs/08-ui-viewport.md`, and `specs/10-roadmap.md`.

---

## Phase 24 — UI Completeness & Inspector Parity (Planned — M87–M89)

Closes every backend-vs-GUI gap uncovered across `chip-cam`, `chip-tools`, `chip-project`, `chip-sketch`, `chip-post`, and `chip-sim` so every active backend parameter, override, wizard option, and multi-setup workflow is directly accessible in `chip-gui` without editing TOML by hand.

### Milestone 87 — Feeds, Speeds, Tool Library CRUD, & Operation Strategy / Linking UI Completeness (`plans/87-milestone-87-plan.md`)
- **Feeds, Presets, & Tool CRUD (`chip-gui`):** Expose `OperationFeedOverrides::{plunge_feed_mm_min, ramp_feed_mm_min, lead_feed_mm_min, stepdown_mm}` in `[Tool]`, `CuttingPreset::{plunge/ramp/lead feeds, material, coolant_override}` + Delete Preset in `presets_editor.rs`, and `Tool::{spindle_direction, default_coolant, vendor, product_id}` + Duplicate/Delete Tool in `tool_library/mod.rs`.
- **Linking & Strategy Parameters (`chip-gui`):** Wire full independent `op.linking.lead_out` controls (decoupled from `lead_in.enabled`) and `EntryStrategy::PreDrilled` in `[Linking]`, plus all 6 missing `DrillBore` fields (G82 dwell, G73 chip-break retract, peck decay, min peck, bore pitch & finish stepover), `Contour2D::finish_feed_mm_min`, `Adaptive2D`/`Adaptive3D` trochoid/feed/flat fields, and `VCarve::clearing_preset_id`.

### Milestone 88 — Setup Inspector, WCS, Stock, Workholding Wizards, & Machine Manager UI Completeness (`plans/88-milestone-88-plan.md`)
- **Setup, WCS, Stock, & Fixtures (`chip-gui`):** Switch Setup Inspector post dropdown to `load_available_post_profiles(...)` (exposing custom `.post.toml`), add multi-body `Setup::target_bodies` selector with Undo, wire `SelectXYPlane::x_edge_ref`, `SelectZAndX::x_ref` clear-to-Auto, and `EdgeStart`/`EdgeMidpoint`/`EdgeEnd` origin switcher, make `RelativeBox::round_increment_mm` editable, allow `SetupClearance` retract ref, and add `FixtureBody::color_rgba` picker/reset.
- **Workholding Wizards & Machine Manager (`chip-gui`):** Wire `SoftJawBoltPattern::Custom::cbore_depth_mm`, `SoftJawConfig::z_band_bottom_offset_mm`, `FlipJigConfig::pin_protrusion_z_mm`, and optional wizard tool/WCS overrides; expose `supported_shank_diameters_mm`, `junction_deviation_mm`, `power_watts`, and `description` in Machine Manager.

### Milestone 89 — Simulation, Project / Model / Sketch Polish, Multi-Setup Batch Actions, & Phase 24 Spec Sync (`plans/89-milestone-89-plan.md`)
- **Simulation, Project, Model, & Sketch Polish (`chip-gui`):** Make `heatmap_max_scale_mm` and `SimVerificationConfig` tolerances editable with **Simulate All Setups** + **Export Metrology JSON**, add inline `project_name` rename, expose `VectorImportModalState::arc_fit_tolerance_mm`, post-import `ModelEntry::{role, unit_override}`, and Sketch Setup reassignment + text arc-fit/face-index controls.
- **Multi-Setup Batch Export, Post Profile Inspector, & Spec Sync (`chip-gui`, `specs/`):** Add **Export All Setups…** batch G-code export, add a read-only / clone-to-custom `.post.toml` inspector modal, and synchronize `specs/{05,06,08,10}-*.md`.

---

## Future Phases

ChipCutter's long-term roadmap focuses on bringing pro-grade CAM capabilities to hobby and benchtop machinists—making everyday job setup faster, safer, and easier to use without commercial CAM complexity or licensing cost.

1. **Physics-Backed Safety & Machinability Guardrails:**
   - **Spindle Power, MRR, & Tool Deflection Advisor:** Compare each operation's material removal rate and estimated radial cutting force against the active machine's real spindle power curve (e.g. `150 W` Carvera Z1 vs. `1.5 kW` Milo/Cascade) and tool stickout—flagging stall risk, chatter, or excessive deflection before cutting.
   - **In-Viewport Measurement & Machinability Inspector:** Click-to-measure distances, hole diameters, depths, and angles directly in the 3D viewport, plus a **Minimum-Radius & Reach Heatmap** highlighting tight internal corners smaller than the chosen cutter and pockets deeper than flute/neck reach.
2. **Guided Operation Setup & Reusable Machining Recipes:**
   - **Guided Operation Setup Mode (Automatic Feature Recognition):** Interactive workflow inspecting imported STEP/mesh/sketch geometry against the active Setup WCS, stock, and target machine's tool library to automatically recognize machinable features and propose a sequenced operation plan—**Facing**, **2D Adaptive / Pocket**, **Drill vs. Helical Bore / Thread Mill**, **2D Contour + Tabs**, **Edge Chamfer**, and **3D Roughing/Finishing**—letting the machinist review, tweak, accept, or skip each step.
   - **Reusable Operation Templates & Recipes:** Save proven multi-op patterns (e.g., *"6061-T6 Pocket + Finish + Chamfer"* or *"M6 Counterbored Hole = Spot + Drill + Helical Bore"*) and apply them in one click to new parts.
3. **In-Machine Probing Operations (`G38.2`):**
   - **Probing Operations (`G38.2`):** First-class WCS corner, bore/boss center, Z-surface, and 2-point rotation/skew probing cycles integrated directly into Setups for grblHAL/Carvera and LinuxCNC controllers.
4. **2D Sheet Nesting, Multi-Part Arrays, & PCB Isolation Milling:**
   - **2D Sheet Nesting & Multi-Part Arrays:** Pack multiple parts onto plate, plywood, or composite sheets with tool-diameter spacing, grain-direction rules, and automatic tabs.
   - **PCB Isolation Milling & Drilling (Gerber / Excellon):** Isolation routing, copper rub-out, Excellon drill import, and height-probe leveling hooks.
5. **Frictionless CAD Prep, Verification, & Ecosystem:**
   - **Live CAD Auto-Reload & Diff Preview:** Watch exported `.step`/`.dxf`/`.svg` files from FreeCAD/Fusion/Onshape, reload automatically on save, and visually highlight changed faces and re-matched selections.
   - **STL / Mesh 2.5D Feature Extraction:** Auto-detect flat Z-planes, cylindrical holes, and outer silhouettes from triangle meshes so STL models can use fast 2.5D strategies in addition to 3D surface passes.
   - **Standalone G-Code Backplot & Crash Verification Mode:** Open any external `.nc`/`.ngc` file and run ChipCutter's Tri-Dexel simulator and 5-rule crash verifier on it.
   - **Broader Tool Library Interop & Printable Setup Sheets:** Import tool libraries from FreeCAD (`.fctb`), Carbide Create, and Vectric alongside Fusion 360; richer material presets (brass, acetal/Delrin, G10/carbon fiber, mild steel, hardwoods); and printable HTML/PDF setup sheets with isometric views, WCS origin diagrams, stock dimensions, tool stickout checklists, and cycle times.
   - **Additional Controller Dialects & CAD Kernel Backends:** Fanuc/Haas, Mach4, Masso, and Centroid post dialects, plus an optional OpenCASCADE (`opencascade-rs`) `CadKernel` feature backend.
6. **Functional & Behavioral User-Level Testing:**
   - Durable end-to-end user-journey and GUI behavioral test harness (`egui_kittest` / headless interactive workflows + CLI workflows) verifying every feature from a machinist's point of view without brittle coupling to internal implementation details.
7. **Tutorials for Upcoming Future-Phase Capabilities:**
   - Dedicated illustrated guides and sample projects accompanying upcoming future-phase capabilities (physics machinability advisor, guided operation setup, probing, 2D nesting, and PCB isolation milling).
