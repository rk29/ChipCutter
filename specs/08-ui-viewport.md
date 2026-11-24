# 08 — User Interface (`egui`) & 3D/2D Viewport (`wgpu`)

## Workspace Shell & Design System (`chip-gui`)
ChipCutter uses `eframe` with the `wgpu` backend and `egui_tiles` for a responsive, high-DPI, **Studio Dark / Daylight Light** CAD/CAM workspace backed by centralized semantic `ThemeTokens`, procedural vector iconography, persistent user preferences (`prefs.toml`), and a **1-click "Reset Default Workspace Layout"** action in the View menu.

### Default Multi-Pane Layout & Floating Viewport HUDs
```text
┌────────────────────────────────────────────────────────────────────────────────────────┐
│ Menu Bar: File | Edit | Setup | Operations | Simulate | Post | View | Help             │
├────────────────────────────────────────────────────────────────────────────────────────┤
│ Workflow Toolbar: [Open][Save][.chipz][DXF/SVG][Reload?][Undo][Redo] | [+ Sketch]      │
│   [+ Adaptive][+ Contour][+ V-Carve][+ Op…] | [Generate][Simulate][Post][Prog] |       │
│   [Tools]                                                      [Cmd (Ctrl+K)][Settings]│
├──────────────────────┬──────────────────────────────────────────┬──────────────────────┤
│ CAM Browser          │ [Setup/WCS/Units] [Pick/Sketch HUD]      │ Inspector            │
│ (Left Tile)          │                                          │ (Right Tile)         │
│                      │           3D CAD / 2D Sketch             │ [Tool] [Geometry]    │
│ ▼ Models             │           Viewport (4x MSAA `wgpu`)      │ [Heights] [Passes]   │
│   • bracket.step     │                                [ViewCube]│ [Linking]            │
│ ▼ 2D Sketches        │                                          │ Two-column aligned   │
│   • Top Logo.svg     │                                          │ `inspector_grid` &   │
│ ▼ Setup 1 [G54]      │                                          │ formula autocomplete │
│   • Stock & WCS      │                                          │                      │
│   ✓ 1. Adaptive T1   │                                          │                      │
│   ✓ 2. V-Carve  T8   │         [Floating View / Layer HUD]      │                      │
├──────────────────────┴──────────────────────────────────────────┴──────────────────────┤
│ Bottom Tile Tabs: [Simulation Timeline] | [G-Code Preview] | [Diagnostics & Log]       │
└────────────────────────────────────────────────────────────────────────────────────────┘
```

### Design System, Vector Icons, & Status Badges *(v3)*
- **Semantic Theme Tokens (`ThemeTokens`):** All panel backgrounds, card borders, accent highlights, semantic status colors, and 3D viewport background/grid colors derive from `ThemeTokens` (`AppTheme::Dark` and `AppTheme::Light`)—never scattered ad-hoc RGB literals.
- **Procedural Vector Icons:** Crisp HiDPI `egui::Painter` vector glyphs for all operation families (including `ThreadMill`, `Deburr3D`, `SlotUndercut`, `Dovetail`, and `Undercut3D`), toolbar actions, visibility toggles, and browser status badges:
  - **Clean** (emerald check): Up-to-date cached toolpath, paired with `T#` and estimated cycle-time pill.
  - **Dirty** (amber refresh): Parameters, tool, setup, upstream Rest Machining op, CAD, or Sketch changed; awaits `Ctrl+G`.
  - **Generating** (animated spinner): Background `rayon` task in flight.
  - **Error / Needs Re-Pick** (coral warning): Validation error, empty cut, crash detected, or orphaned B-Rep selection after STEP reload.
  - **Suppressed** (muted slash-circle + strikethrough): Skipped in generation, Rest Machining coverage, and post output.
- **Drag & Keyboard Browser Ergonomics:** Operations support drag-and-drop and `Ctrl+Up` / `Ctrl+Down` reordering, `Ctrl+D` duplication, `Delete` removal, and hover-revealed row actions so operation names remain readable even in narrow left panes.

---

## Custom `wgpu` 3D / 2D Sketch Viewport Pipeline
Integrated into the central `egui` panel via an offscreen **`4x MSAA` `ViewportRenderer`** whose resolved color target is registered and updated through `egui_wgpu::Renderer::register_native_texture` / `update_egui_texture_from_wgpu_texture` with persistent growable GPU buffers (`GrowableGpuBuffer` — zero per-frame buffer re-allocation churn):

### Render Passes (`viewport/pipeline.rs` Draw Order)
1. **Background Gradient / Grid Pass (`bg_pipeline`):** Subtle studio vertical gradient + WCS / Active Sketch XY reference grid with major/minor millimeter/inch ticks.
2. **Pass 2A / 6A — Simulation Machined Stock Pass (rendered *before* ghost B-Rep so translucent target solids overlay cleanly over carved stock):**
   - Renders either a CPU indexed mesh (`SimStockRenderSource::CpuMesh` via `sim_stock_pipeline`) or a **direct zero-readback GPU stock** (`SimStockRenderSource::GpuGrid` via `SimGpuStockRenderer`) across **`Heightfield`**, volumetric **`TriDexel`**, and 4th-axis **`Cylindrical`** stocks (with GPU `stock_model` matrix rotation during 4th-axis playback).
   - Supports **`Color Cuts by Operation`**, instant **Signed Error Deviation Heatmap** (`±tol` red gouge / green in-tolerance / blue uncut excess), and **crease-preserving simulation mesh skinning** across CPU & GPU backends.
3. **Pass 2B — Shaded B-Rep Solid & Triangle Mesh Pass (`solid_pipeline` / `solid_ghost_pipeline`):**
   - Phong/PBR-lite studio lighting locked relative to camera (headlight + rim + ambient), rendered opaque normally or as a translucent ghost over machined stock.
   - Per-face state tinting: neutral metallic gray (default), soft blue highlight (hovered `FaceId`), vivid cyan/gold highlight (selected `FaceId`s), translucent ghosting mode.
4. **Pass 6B — Animated 3D Cutter & Holder Assembly Pass (`sim_tool_pipeline`):**
   - Revolved 3D tool, relieved neck, shank, and collet/holder assembly mesh animated along the simulation timeline.
5. **Pass 3 / 3B — Translucent Stock, Z-Height Planes, Cutting-Area & Keep-Out Previews, & 3D Workholding Fixtures Pass (`overlay_pipeline`, `fixture_pipeline`):**
   - Translucent tinted box or cylinder for Setup stock; translucent shaded 3D workholding hardware (`ViseJaws`, `ToeClamp`, `Spoilboard`, `BoxKeepOut`, `CylinderKeepOut`, and `ImportedModel` fixtures, with crimson highlighting on any `FixtureCollision` body and a `Fixtures` viewport HUD toggle); translucent Z-height reference planes (`Clearance`, `Retract`, `Top`, `Bottom`) shown live while editing the **Heights** tab; and cached **Live 3D Cutting-Area & Avoid / Keep-Out Previews** (`BoundaryPreviewOverlay`, zero per-frame mesh projection on idle frames) shown whenever the **`[Geometry]`** tab is active—rendering crisp cyan/accent nominal boundary loops + dashed amber effective tool-center containment loops (and vertical top-to-bottom reference ticks) for 2.5D `Facing`, 3-axis surface, `Undercut3D`, and `Deburr3D` operations, outer stock clearing envelopes for `Pocket2D` `ClearOutsideSelectedToStock`, translucent cylindrical `[u_start, u_end] × [start_angle_deg, end_angle_deg]` sector sleeves with `-U`/`+U` endcap rings, inner `r_floor` rings, and amber/crimson rotary chuck keep-out planes (`chuck_guard_u`) for 4th-axis rotary operations, plus high-contrast **coral/red Avoid / Keep-Out overlays** (solid coral/red nominal outline + translucent red top-plane fill + dashed coral outer clearance ring for 2.5D/3-axis ops, and translucent coral/red axial keep-out sleeve bands for 4th-axis rotary ops) legible in both `Studio Dark` and `Daylight Light`.
   - **Indexed `3+1` Auto-Rotate Viewport (*Phase 23*):** When editing an `OperationRotaryMode::Indexed3Plus1` operation outside active simulation playback (and `auto_rotate_indexed_op` is enabled in `[Passes] → 4th-Axis Indexing / Wrapping` or `Application Settings → Viewport`), the viewport automatically rotates the 3D CAD solid, raw setup stock wireframe, picking raycasts, Z-height planes, and `BoundaryPreviewOverlay` to `angle_deg` so the machined face sits upright under `+Z` with a live `Indexed A = <angle>°` HUD badge.
6. **Pass 4 — Topological B-Rep Edge, 2D Sketch Vector, Stock Wireframe, WCS Gizmo, & Tab Pass (`edge_pipeline`):**
   - Screen-space width-stabilized line/curve pass rendering exact `BRepEdge` curves, `SketchLayer` `ArcPolyline2D` entities, construction geometry (dashed), open-gap diagnostic markers, closed planar 2D region translucent hover/selection fills, Stock wireframe, RGB XYZ WCS triad arrows (`X`=Red, `Y`=Green, `Z`=Blue), 3D directional chain cones, and interactive 3D holding tab markers.
7. **Pass 5 — GPU-Instanced Toolpath Pass (`edge_pipeline`):**
   - GPU-instanced line/arc segments supporting `500,000+` segments at `60 FPS` (essential for dense high-speed 2D Adaptive trochoidal paths, 3-axis finishing rasters, and V-Carve scrollwork).

---

## 2D Sketch Authoring & Import Workflows *(v2, extended Phase 24)*
- **Sketch Inspector & Interactive Authoring Tools:**
  - Selecting or creating a `SketchLayer` in the **2D Sketches** browser section reveals Sketch Plane anchor settings (`Setup XY` vs. `Planar STEP Face`), multi-setup **`Setup`** reassignment (`ComboBox` whenever `project.setups.len() > 1`), 2D Snap Grid controls, and the **6 interactive sketch creation tools** (`Rectangle`, `Circle`, `Slot`, `Polygon`, `BoltCircle`, `PlaceText`); imported `Polyline` entities from DXF/SVG are also listed and editable (translation, uniform scale, rotation) in the Sketch Inspector.
  - Clicking any `SketchEntity` opens its exact expression-backed parameters (`width`, `height`, `corner_radius`, `cap_height`, `font`, Hershey vs. TrueType mode, `arc_fit_tolerance_mm`, and custom `.ttc` `face_index`, etc.) in the right-hand Inspector.
- **DXF / SVG Import Modal (`vector_import_modal.rs`):**
  - Live bounding-box dimension preview (`X × Y` in display units) before confirming import so unitless DXFs (`mm` vs. `in`) and pixel SVGs (`96` / `72` / `90` DPI) are never accidentally imported at `25.4x` wrong scale.
  - Exposes unit-aware **`Join Tolerance`** (`join_tolerance_mm`, gap-healing preview reporting closed loops vs. open chains), **`Arc-Fit Tolerance`** (`arc_fit_tolerance_mm`, Bézier/spline-to-biarc deviation), and DXF layer filtering.

## 3D Mesh Import & Model Transform Inspector *(Phase 5 & Phase 10, extended Phase 24)*
- **Import 3D Mesh (STL / OBJ / 3MF) Modal (`mesh_import_modal.rs`):**
  - Accessible from `File → Import 3D Mesh (.stl / .obj / .3mf)…` and the Welcome Overlay, with built-in sample previews (`Sample Dome (STL)`, `Sample Wave (OBJ)`).
  - Auto-detects 3MF header units or prompts for unitless STL/OBJ physical units (`mm`, `in`, `cm`, `m`, `ft`, `um`), displays live transformed bounding-box dimensions and watertight/manifold diagnostics, and provides pre-import orientation (`Y-Up → Z-Up`, `±90° X/Y/Z`), uniform **`Scale` / `Target Size`**, and **`Center XY`** / **`Drop Bottom to Z=0`** toggles.
- **Model Transform & Orientation Inspector (`model_inspector.rs`):**
  - Clicking any imported STEP solid or STL/OBJ/3MF mesh in the CAM Browser's `Models` card opens a two-column Inspector showing live transformed bounding-box size and Z range, post-import **`ModelEntry::role`** (`Target Part` / `Fixture / Workholding`) and triangle-mesh **`ModelEntry::unit_override`** selectors (triggering clean background reload + atomic Undo), quick orientation buttons (`Y-Up → Z-Up (+90° X)`, `±90° X/Y/Z`), interactive 3D **`Lay Flat on Clicked Face…`** (supporting STEP planar faces and mesh triangles), **`Center XY`**, **`Drop Bottom to Z=0`**, **`Align Top to Z=0`**, **`Reset`**, exact expression-capable `Rotate X/Y/Z`, `Move X/Y/Z`, and `Scale` (Uniform Lock or independent `X/Y/Z`) + `Target Size X/Y/Z` controls—all with `< 1 ms` Undo/Redo and automatic `Dirty` invalidation on dependent operations.

---

## Camera Projection & Mouse Navigation Presets
- **Keyboard Camera Hotkeys:** **Orthographic** by default, with `P` toggling **Orthographic / Perspective** projection and `F` invoking **Fit All** (`Fit Model to Viewport`).
- **View Controls, ViewCube, & Alignment Actions:**
  - Interactive top-right corner **ViewCube / Orientation Widget** + Floating Viewport View HUD buttons, menu entries (`View` / `Setup`), Sketch Inspector buttons, and `Ctrl+K` Command Palette actions for standard views (`Top`, `Bottom`, `Front`, `Back`, `Left`, `Right`, `Isometric`), **`Align View to Active Setup Top`**, and **`Look At Sketch`**.
  - Top-left read-only **Setup / WCS / Units pill** displays the active Setup name, work offset (`G54`–`G59.3`), and display units (`mm`/`in`), while the bottom-center **Floating View / Layer HUD** provides projection, view-alignment, and layer visibility toggles.
- **Configurable Mouse Navigation Presets (in `Application Settings → Viewport & Navigation`):**
  - **Fusion 360:** `MMB` Pan, `Shift + MMB` Orbit, `Scroll` Zoom-to-cursor.
  - **SolidWorks:** `MMB` Orbit, `Ctrl + MMB` Pan, `Scroll` Zoom-to-cursor.
  - **FreeCAD / Blender:** `MMB` Orbit, `Shift + MMB` Pan, `Scroll` Zoom-to-cursor.
  - **Orbit Mode Toggle:** `Turntable` (locks horizon to Setup `+Z`, default) vs. `Free Trackball`.

---

## Parametric Input Widgets, Discoverability, & Persistent Preferences
- **Two-Column Aligned Inspector Grids & Expression-Aware Inputs *(v3)*:**
  - Inspector forms use standardized two-column property grids (`inspector_grid` and `grid_row` in `widgets/inspector.rs`) inside collapsible `section_card` containers with uniform input widths and unit suffix badges.
  - Expression fields display resolved values in preferred display units (`mm` or `in`), offer a live scoped variable reference/autocomplete popover (`tool_d`, `tool_r`, `stock_x`, `feature_top_z`, etc.), support `Up`/`Down` arrow-key nudging and drag-scrubbing on plain numbers, validate syntax live, and commit on `Enter` or focus loss.
- **Active Pick / Sketch Guidance HUD, Welcome Overlay, & Command Palette *(v3)*:**
  - Whenever a 3D B-Rep pick mode, manual tab placement, or 2D sketch tool is active, a prominent floating guidance banner at the top-center of the viewport explains the expected next click and provides `[Done]` / `[Cancel (Esc)]` controls.
  - Launching `chip-gui` with no file argument opens a clean **Welcome / Quick-Start Overlay** (`"Open Project or Portable Bundle…"`, `"Import 3D STEP Solid…"`, `"Import 3D Mesh (STL / OBJ / 3MF)…"`, `"Import 2D DXF / SVG Artwork…"`, `"New Pure-2D Parametric Sketch"`, `Recent Projects`, and `"Browse Sample Projects Gallery…"` modal).
  - **`Ctrl+K` Command Palette** provides instant fuzzy search across all menu, view, sample, and operation-creation actions, paired with a **`?` / `F1` Keyboard Shortcut Cheat-Sheet** and **`Help → About ChipCutter…`** version/build info modal.
- **Setup `[Fixtures]` Tab, Interactive Placement, & Workholding Wizard Modals:**
  - The Setup Inspector's strict two-column **`[Fixtures]`** tab provides 1-click `+ Vise`, `+ Toe Clamp`, `+ Spoilboard`, `+ Keep-Out`, `+ Cylinder`, and `+ Import Fixture…` actions, interactive 3D viewport click-to-place (`Place Clamp in Viewport…`, `Place Pin in Viewport…`), live `Auto-Avoid Rapids` lift & collision banners, and direct buttons to launch the **Flip-Jig Wizard…** and **Auto Soft-Jaw Wizard…** modals (also accessible from the **`Setup`** top menu and **`Ctrl+K`** Command Palette).
  - Both wizard modals provide strict two-column parameter grids, universal hover tooltips, live symmetry/clearance/pocket/STL previews, single-step `< 1 ms` Undo on **Apply to Project**, and native / XDG-portal **`Export Soft Jaws (.stl)…`** folder export.
- **Tool Library Schematics, Undercut / 3D Deburr Inspector Cards, & Tri-Dexel Advisory:**
  - The **Tool Library Manager** (`Edit → Manage Tools...` or the top action bar `Tools` button) provides aligned two-column property editors inside collapsible section cards (`Tool Identity`, `Physical Geometry`, and `Material Cutting Presets` expanded by default; `Controller Offsets (H / D)` and `Vendor & Catalog Info` collapsed by default), universal hover tooltips, and procedural 2D cross-section schematics for all 10 cutter geometries—including relieved-neck **Lollipop Endmill** (spherical head + neck join), **Slot / T-Slot Mill** (disc head + corner radius + neck), and **Dovetail Cutter** (inverted angled cone + top minor diameter readout).
  - The **Operation Inspector** renders dedicated strict two-column **`[Passes]`** cards for **`3D Deburr`** (`chamfer_width`, `tip_offset`, `wall_clearance`, auto-detect sharp edges, `allow_underside_back_chamfer` Lollipop badge), **`Slot / T-Slot Undercut`** (DIN 650 / ASME / DIN 6885 preset picker, throat/neck clearance safety banner, roof-safe `UndercutEntryMode`), **`Dovetail`** (`InternalSlot` / `ExternalRail`, angle-match & throat-fit diagnostics), and **`3D Lollipop Undercut`** (Z-stepdown / cusp height, overhang angle, neck clearance, open-air lead-out), all accessible from the `Operations` menu, top toolbar `"+ Op…"`, and `Ctrl+K` Command Palette.
  - When a Setup contains undercut cutters while the Simulation Timeline is set to 2.5D `Heightmap`, an inline amber advisory banner offers a 1-click **`Use Tri-Dexel`** switch; the **Sample Projects Gallery** includes the built-in **`Undercut & 3D Deburr Block`** (`--sample undercut-deburr`) showcase running in volumetric `Tri-Dexel` mode.
- **Unified Application Settings Dialog (`prefs.toml` — *Phase 21*):**
  - Accessible via `Edit → Application Settings...` (`Ctrl+,`), the `Ctrl+K` Command Palette (`Open Application Settings`), the compact top-right `Application Settings` toolbar button, and the Simulation Timeline summary chip.
  - Organizes low-frequency workspace choices into four sections (**General & Appearance**: theme, project & default display units, startup Welcome screen, layout reset; **Viewport & Navigation**: mouse navigation preset, orbit mode, default projection, overlays; **Simulation**: engine, detail level, compute backend, and **Smooth Skinned Surfaces / Mesh Skinning**; **Machine Defaults**: default machine for new projects) and removes those infrequent controls from the main toolbar, viewport bottom HUD, and simulation timeline bar.
  - Persists `AppTheme`, default display units, default machine ID, mouse navigation preset, orbit mode, viewport visibility toggles, `sim_engine`, `sim_resolution`, `sim_backend`, `sim_mesh_skinning`, `egui_tiles` pane shares, and recent project paths across sessions in `prefs.toml`.
- **Active-Setup Indication & Top Action Bar Operation Creation (*Phase 21*):**
  - Each Setup card in the CAM Browser clearly highlights the **Active Setup** with an accent border and an `"Active"` badge pill (with 1-click activation on inactive setups), eliminating redundant per-card operation-add bars.
  - Operation creation (`+ Sketch`, `+ Adaptive`, `+ Contour`, `+ V-Carve`, and categorized `"+ Op…"`) lives cleanly in the top action bar targeting the active Setup, with explicit visual separators between File/History, Geometry/Operations, CAM Pipeline, Tools, and right-aligned Utility groups.
- **Streamlined Project Tool Management & Machine-Switch Tool Swapping (*Phase 21*):**
  - Workspace chrome exposes a single top-toolbar `"Tools"` button alongside `Edit → Manage Tools...`, with `Machine Manager…` under the `Setup` menu.
  - The CAM Browser's **`Project Tools`** card provides inline searchable **`+ Add Tool…`** (from the active machine library), per-tool usage badges (`N ops` / `Unused`), guarded per-tool removal, and 1-click **`Remove Unused (N)`** pruning.
  - The Operation Inspector's **Active Cutter** picker groups compatible `Project Tools` and `Machine Library` cutters first with inline search and an opt-in incompatible toggle.
  - Switching a project's target machine from Setup Inspector, Project Properties, Machine Manager, or Tool Library automatically inspects referenced operations (including V-Carve clearing tools) and opens a guided **`"Switch Machine — {target_machine.name}"`** dialog pre-selecting the closest compatible replacements and applying the swap (`"Switch Machine & Swap Tools"`) + optional unused-tool pruning in a single atomic Undo step.
- **Consolidated `[Geometry]` Tab Cutting-Area Controls, Avoid / Keep-Out Manager, & Fixed-Box Alignment (*Phase 22–23*):**
  - All cutting-area and machining-boundary controls live in a consistent top-of-tab **`Cutting Area / Machining Boundary`** card on the Operation Inspector **`[Geometry]`** tab (removed from `[Passes]` so there is a single authoritative home for *what area to cut*):
    - **2.5D `Facing`, 3-Axis (`Adaptive3D`, `Waterline3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`), `Undercut3D`, & `Deburr3D`:** `Boundary Source` (`Stock Bounds` | `Model Bounds` with independent signed `-X`/`+X`/`-Y`/`+Y` offsets | `Model Silhouette` | `Selected 2D Contours`), `Tool Containment` (`Tool Inside` | `Tool Center` | `Tool Outside`), signed `Boundary Offset`, live XY span & tool-center shift readout, contextual `Auto-Detect Edges` / empty-selection warnings, auto-collapsed 3D Pick card when `Stock Bounds`, `Model Bounds`, or `Model Silhouette` is active, and auto-expanded **`2D Sketch Layers & Quick Select`** (with per-entity toggle chips and empty-state `+ New Sketch`) whenever `Selected 2D Contours` is active.
    - **4th-Axis Rotary (`Rotary2Linear`, `Rotary4AxisSim`):** Dedicated **`4th-Axis Rotary Cutting Area (-U / +U)`** card with `Axial Source` (`Stock Bounds` | `Model Bounds` | `Selected Geometry (-U..+U)` | `Explicit Range`), `Axial Containment` (`Tool Inside` | `Tool Center` | `Tool Outside`), signed `-U` / `+U` margins, `Start U` / `End U` (when `Explicit Range`), `Start Angle (°)` / `End Angle (°)` with quick preset chips (`Full 360°`, `0–180°`, `±90°`), live `Effective U` + `Chuck Guard` readout with chuck-clamp warning banner, and auto-collapsed 3D Pick card unless `Selected Geometry` is active.
    - **`Pocket2D` / `Adaptive2D` & `Contour2D`:** `Pocket Area Mode` (`Inside Selected Pockets` | `Clear Outside Selected Bosses to Stock`) + `Open Outer Boundary` checkbox for pockets, and `Clip to Stock Bounds` + `Open Chain Extension` for contours.
  - **Per-Operation & Shared `Avoid / Keep-Out Regions` UI (*Phase 23*):**
    - A collapsible **`Avoid / Keep-Out Regions`** card in `[Geometry]` (auto-expanded whenever `!op.avoid.is_empty()` or actively editing a shared region) exposes 1-click toggle chips for project-level `SharedAvoidRegion` entries, `Save Local as Shared…`, `+ New Shared`, inline `Edit` (with active coral/red banner, optional radial/axial clearance overrides, fixture chips, and sketch `+ Avoid`), `Detach to Local`, `Apply to Setup Ops…`, `Avoid All Setup Fixtures` + per-fixture chips, local sketch `+ Avoid` quick-add, removable local keep-out list, and `Radial Clearance` / `Axial Clearance` expression inputs.
    - A segmented **`Pick Target:`** (`Cutting Area` | `Avoid / Keep-Out`) toggle inside **`Interactive 3D Pick Mode`** routes 3D viewport and 2D sketch clicks directly into `op.avoid.selections` or the actively edited `SharedAvoidRegion`, accented by a coral/red `[Avoid / Keep-Out]` viewport Guidance HUD badge.
    - The **CAM Browser** includes a collapsible **`Keep-Out Regions (N)`** card showing every `SharedAvoidRegion`, its usage count (`used by N ops`), `+ Keep-Out`, inline rename, suppress toggle, duplicate, and safe delete (`ProjectDocument::remove_avoid_region`) in atomic Undo steps.
  - **Fixed-Size Box Stock Alignment (`Setup → [Stock] → Stock Offsets` — *Phase 23*):**
    - Exposes interactive **`XY Alignment`** (`Centered in XY` | `Offset from Front-Left` with `Front-Left ΔX` / `ΔY`) and **`Z Alignment`** (`Offset from Top (+Z)` | `Offset from Bottom (-Z)` | `Centered in Z`) selectors, auto-defaulting rotary-enabled Setups to `Centered in Z` when switching to `Fixed-Size Box Stock`.
- **Complete Inspector Parity, Project Properties, Batch Post, & Post Profile Inspector (*Phase 24 — M87–M89*):**
  - **Project Properties & Model Inspector:** Dedicated `Edit → Project Properties...` modal (`Ctrl+K` searchable) for editing `project.project_name` (atomic Undo), `display_units`, target CNC machine (`request_machine_switch` + `Manage Machines...`), and inspecting a live project summary (file path, schema version, models, setups/operations, project tools, 2D sketches, and shared keep-out regions), keeping the CAM Browser header clean next to `+ Setup`; post-import `ModelEntry::role` (`Target Part` / `Fixture / Workholding`) and triangle-mesh `ModelEntry::unit_override` selectors triggering clean background mesh reload + Undo.
  - **Setup, Workholding, & Machine Manager Completeness:** Multi-body `Setup::target_bodies`, custom `.post.toml` dropdowns, `SelectXYPlane` `+X` edge picking, `EdgeStart`/`EdgeMidpoint`/`EdgeEnd` origin switching, `RelativeBox::round_increment_mm`, `SetupClearance` retract height ref, per-fixture color picker, wizard tool/WCS/counterbore/protrusion controls, and Machine Manager shank list, junction deviation, spindle watts, and description fields.
  - **Multi-Setup Batch G-Code Export (`Export All Setups...`):** Accessible from `[G-Code Preview]`, `File → Export All Setups...`, and `Post → Export All Setups...`; writes sanitized per-Setup `.nc`/`.ngc` files into a chosen output folder using `chip-post`'s single-component filename & extension safety rules (`validate_safe_output_filename`), reporting per-setup status and warnings in the Activity Log and status banner.
  - **Post Profile Inspector / Customizer Modal:** Accessible from `[G-Code Preview]`, Setup Inspector `[WCS & Post]`, and `Post → Post Profile Inspector / Customizer...`; inspects all `PostProfile` settings (units, decimals, line numbers, comment style, arc mode IJK/R, `max_arc_sweep_deg`, drill-cycle expansion, tool-change mode, G43/G41/G42, headers/footers, and `RotaryPostConfig`) as strictly **read-only** for built-in profiles with a 1-click **`Clone to Custom .post.toml`** workflow saving upgrade-safe custom profiles into `dirs::config_dir()/chipcutter/posts/`.
- **Document Undo / Redo (`Ctrl+Z` / `Ctrl+Y`):**
  - Every committed mutation to Setups, Sketches, Selections, Operations, Models, Keep-Out Regions, or Project Tools pushes a compact document state snapshot onto an Undo stack (default depth `100` steps, captured lazily only on actual mutation so idle 60 FPS frames perform zero document clones).
  - Heavy cached data (B-Rep render meshes, `MeshBvh` spatial indices, generated toolpaths, swept 2D Rest polygons, heightmaps, and volumetric `TriDexelStock` / `CylindricalStock` grids) is stored out-of-band by revision/key so Undo/Redo is instantaneous (`< 1 ms`).
