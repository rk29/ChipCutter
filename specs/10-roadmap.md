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

## Future Phases

ChipCutter's long-term roadmap focuses on bringing pro-grade CAM capabilities to hobby and benchtop machinists—making everyday job setup faster, safer, and easier to use without commercial CAM complexity or licensing cost.

1. **Toolpath Algorithm Quality & Engagement-Aware Feedrate Optimization:**
   - **Error-Metric-Driven Quality Polish:** Use Tri-Dexel error-volume & RMS benchmarks across 2.5D, 3D surface (`Adaptive3D`, `Waterline3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`), V-Carve, Thread Milling, and 4th-axis rotary strategies to systematically eliminate residual scallops, corner overcuts, and uncut islands.
   - **Constant-Chipload & Corner Slowdown Feedrate Optimization:** Automatically throttle feedrates inside tight internal corners where radial engagement spikes and accelerate through light-engagement cuts to extend endmill life on hobby-rigidity machines.
2. **Physics-Backed Safety & Machinability Guardrails:**
   - **Spindle Power, MRR, & Tool Deflection Advisor:** Compare each operation's material removal rate and estimated radial cutting force against the active machine's real spindle power curve (e.g. `150 W` Carvera Z1 vs. `1.5 kW` Milo/Cascade) and tool stickout—flagging stall risk, chatter, or excessive deflection before cutting.
   - **In-Viewport Measurement & Machinability Inspector:** Click-to-measure distances, hole diameters, depths, and angles directly in the 3D viewport, plus a **Minimum-Radius & Reach Heatmap** highlighting tight internal corners smaller than the chosen cutter and pockets deeper than flute/neck reach.
3. **Guided Operation Setup & Reusable Machining Recipes:**
   - **Guided Operation Setup Mode (Automatic Feature Recognition):** Interactive workflow inspecting imported STEP/mesh/sketch geometry against the active Setup WCS, stock, and target machine's tool library to automatically recognize machinable features and propose a sequenced operation plan—**Facing**, **2D Adaptive / Pocket**, **Drill vs. Helical Bore / Thread Mill**, **2D Contour + Tabs**, **Edge Chamfer**, and **3D Roughing/Finishing**—letting the machinist review, tweak, accept, or skip each step.
   - **Reusable Operation Templates & Recipes:** Save proven multi-op patterns (e.g., *"6061-T6 Pocket + Finish + Chamfer"* or *"M6 Counterbored Hole = Spot + Drill + Helical Bore"*) and apply them in one click to new parts.
4. **Shop-Floor Workholding, Probing, & Multi-Setup Stock Carryover:**
   - **Probing Operations (`G38.2`):** First-class WCS corner, bore/boss center, Z-surface, and 2-point rotation/skew probing cycles integrated directly into Setups for grblHAL/Carvera and LinuxCNC controllers.
   - **3D Fixture, Vise, & Clamp Collision Bodies:** Automatic safe-rapid routing and crash verification against workholding solids.
   - **In-Process Stock Carryover Across Setups (`G54` -> `G55`):** Pass the exact remaining Tri-Dexel solid from Setup 1 into Setup 2 (e.g. after flipping or re-clamping the part) so both simulation and rest-aware toolpaths know exactly which features were already machined—enabling zero-air-cut multi-op roughing, accurate Op-2 collision checks, and cumulative error-volume metrics.
   - **Two-Sided Flip-Jig & Auto Soft-Jaw Wizards:** Automatically place symmetric registration dowel-pin holes for flip machining without a vise, or auto-generate ready-to-machine aluminum / 3D-printed vise soft-jaw pockets directly from the part's silhouette.
5. **Expanded Machining Capabilities:**
   - **4th-Axis Cylindrical Wrapping:** Wrap 2D sketches, SVG artwork, text, pockets, and V-carve paths cleanly onto cylindrical rotary stock (`A`-axis).
   - **3D Edge Deburring & Non-Planar Chamfering:** Follow 3D B-Rep edges with a chamfer or lollipop cutter so curved and stepped parts come off the machine already deburred.
   - **Undercuts, Keyways, T-Slots, & Dovetails:** Lollipop, slot-mill, T-slot, and dovetail cutter geometries + strategies backed by Tri-Dexel undercut simulation.
   - **2D Sheet Nesting & Multi-Part Arrays:** Pack multiple parts onto plate, plywood, or composite sheets with tool-diameter spacing, grain-direction rules, and automatic tabs.
   - **PCB Isolation Milling & Drilling (Gerber / Excellon):** Isolation routing, copper rub-out, Excellon drill import, and height-probe leveling hooks.
6. **Frictionless CAD Prep, Verification, & Ecosystem:**
   - **Live CAD Auto-Reload & Diff Preview:** Watch exported `.step`/`.dxf`/`.svg` files from FreeCAD/Fusion/Onshape, reload automatically on save, and visually highlight changed faces and re-matched selections.
   - **STL / Mesh 2.5D Feature Extraction:** Auto-detect flat Z-planes, cylindrical holes, and outer silhouettes from triangle meshes so STL models can use fast 2.5D strategies in addition to 3D surface passes.
   - **Standalone G-Code Backplot & Crash Verification Mode:** Open any external `.nc`/`.ngc` file and run ChipCutter's Tri-Dexel simulator and 5-rule crash verifier on it.
   - **Broader Tool Library Interop & Printable Setup Sheets:** Import tool libraries from FreeCAD (`.fctb`), Carbide Create, and Vectric alongside Fusion 360; richer material presets (brass, acetal/Delrin, G10/carbon fiber, mild steel, hardwoods); and printable HTML/PDF setup sheets with isometric views, WCS origin diagrams, stock dimensions, tool stickout checklists, and cycle times.
   - **Additional Controller Dialects & CAD Kernel Backends:** Fanuc/Haas, Mach4, Masso, and Centroid post dialects, plus an optional OpenCASCADE (`opencascade-rs`) `CadKernel` feature backend.
7. **GPU-Accelerated Simulation (`wgpu` Compute):**
   - **Optional Headless GPU Compute Backend (`gpu-compute` feature):** Accelerate 2.5D Heightmap, 4th-Axis Cylindrical, and Volumetric 3D (fixed-`K` Tri-Dexel span pools or sparse brickmap SDF) stock subtraction using tiled WGSL compute shaders on `wgpu`—enabling real-time Ultra-resolution playback scrubbing while keeping the deterministic `f64` CPU engine as the exact fallback.
   - **Direct GPU Stock Rendering & Instant Error Heatmap:** Displace heightfield/cylindrical stock directly from GPU buffers without CPU re-tessellation, evaluate target-mesh signed distances via a GPU BVH for instantaneous tolerance-slider Heatmap recoloring, and run parallel GPU crash bitmasks feeding CPU diagnostic reporting.
8. **Functional & Behavioral User-Level Testing:**
   - Durable end-to-end user-journey and GUI behavioral test harness (`egui_kittest` / headless interactive workflows + CLI workflows) verifying every feature from a machinist's point of view without brittle coupling to internal implementation details.
9. **Tutorials for Upcoming Future-Phase Capabilities:**
   - Dedicated illustrated guides accompanying new future-phase capabilities (guided operation setup, probing, 3D workholding & multi-setup Tri-Dexel stock carryover, soft jaws, 4th-axis wrapping, nesting, and PCB isolation milling).
