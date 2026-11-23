# 08 — User Interface (`egui`) & 3D/2D Viewport (`wgpu`)

## Workspace Shell & Design System (`chip-gui`)
ChipCutter uses `eframe` with the `wgpu` backend and `egui_tiles` for a responsive, high-DPI, **Studio Dark / Daylight Light** CAD/CAM workspace backed by centralized semantic `ThemeTokens`, procedural vector iconography, persistent user preferences (`prefs.toml`), and a **1-click "Reset Default Layout"** action in the View menu.

### Default Multi-Pane Layout & Floating Viewport HUDs
```text
┌──────────────────────────────────────────────────────────────────────────────┐
│ Menu Bar: File | Edit | Setup | Operations | Tools | Simulate | Post | View | Help │
├──────────────────────────────────────────────────────────────────────────────┤
│ Single-Row Workflow Toolbar: [Open][Save][.chipz][DXF/SVG] | [+ Sketch]      │
│   [+ Adaptive][+ V-Carve] | [Generate][Simulate][Post][Progress] | [Tools]   │
├────────────────────┬────────────────────────────────────┬────────────────────┤
│ CAM Browser        │  [Active Pick / Sketch Banner HUD] │ Inspector          │
│ (Left Tile)        │                                    │ (Right Tile)       │
│                    │       3D CAD / 2D Sketch           │ [Tool] [Geometry]  │
│ ▼ Models           │       Viewport (`egui_wgpu`)       │ [Heights] [Passes] │
│   • bracket.step   │                                    │ [Linking]          │
│ ▼ 2D Sketches      │                            [ViewCube]                   │
│   • Top Logo.svg   │                                    │ Two-column aligned │
│ ▼ Setup 1 [G54]    │                                    │ property grids &   │
│   • Stock & WCS    │                                    │ formula autocomplete│
│   ✓ 1. Adaptive T1 │                                    │                    │
│   ✓ 2. V-Carve  T8 │  [Floating View / Layer / Unit HUD]│                    │
├────────────────────┴────────────────────────────────────┴────────────────────┤
│ Bottom Tile Tabs: [Simulation Timeline] | [G-Code Preview] | [Diagnostics]   │
└──────────────────────────────────────────────────────────────────────────────┘
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
Integrated into the central `egui` panel via `egui_wgpu::CallbackTrait` rendering into a multisampled (`4x MSAA`) color + depth target with persistent growable GPU buffers (`GrowableGpuBuffer` — zero per-frame buffer re-allocation churn):

### Render Passes
1. **Background Gradient / Grid Pass:** Subtle studio vertical gradient + WCS / Active Sketch XY reference grid with major/minor millimeter/inch ticks.
2. **Shaded B-Rep Solid Pass:**
   - Phong/PBR-lite studio lighting locked relative to camera (headlight + rim + ambient).
   - Per-face state tinting: neutral metallic gray (default), soft blue highlight (hovered `FaceId`), vivid cyan/gold highlight (selected `FaceId`s), translucent ghosting mode.
3. **Topological B-Rep Edge & 2D Sketch Vector Pass *(v2 extended)*:**
   - Screen-space width-stabilized line/curve pass rendering exact `BRepEdge` curves AND `SketchLayer` `ArcPolyline2D` entities, construction geometry (dashed), open-gap diagnostic markers, closed planar 2D region translucent hover/selection fills, and 3D directional chain cones.
4. **Stock, Workholding Fixtures, & WCS Gizmo Pass:**
   - Translucent wireframe/tinted box or cylinder for Setup stock; translucent shaded 3D workholding hardware (`ViseJaws`, `ToeClamp`, `Spoilboard`, `BoxKeepOut`, `CylinderKeepOut`, and `ImportedModel` fixtures, with crimson highlighting on any `FixtureCollision` body and a `Fixtures` viewport HUD toggle); RGB XYZ triad arrows (`X`=Red, `Y`=Green, `Z`=Blue) anchored at the WCS origin; translucent Z-height reference planes (`Clearance`, `Retract`, `Top`, `Bottom`) shown live while editing the **Heights** tab.
5. **Instanced Toolpath & Tab Pass:**
   - GPU-instanced line/arc segments supporting `500,000+` segments at `60 FPS` (essential for dense high-speed 2D Adaptive trochoidal paths and V-Carve scrollwork), plus interactive 3D holding tab markers.
6. **Simulation Stock & Cutter Pass:**
   - Dynamic heightmap mesh (colored optionally by operation/tool, showcasing roughing vs. Rest Machining vs. V-Carve bevels) + animated 3D tool/shank/holder mesh.

---

## 2D Sketch Authoring & Import Workflows *(v2)*
- **Sketch Inspector & Floating Creation Bar:**
  - Selecting or creating a `SketchLayer` in the **Sketches** browser section reveals Sketch Plane anchor settings (`Setup XY` vs. `Planar STEP Face`) and creation tools (`Rectangle`, `Circle`, `Slot`, `Polygon`, `Polyline`, `Hole Pattern`, `Text`).
  - Clicking any `SketchEntity` opens its exact expression-backed parameters (`width`, `height`, `corner_radius`, `cap_height`, `font`, Hershey vs. TrueType mode, etc.) in the right-hand Inspector.
- **DXF / SVG Import Modal:**
  - Live bounding-box dimension preview (`X × Y` in display units) before confirming import so unitless DXFs (`mm` vs. `in`) and pixel SVGs (`96` / `72` / `90` DPI) are never accidentally imported at `25.4x` wrong scale!
  - Exposes `join_tolerance_mm` (gap-healing preview reporting closed loops vs. open chains) and layer filtering.

---

## Camera Projection & Mouse Navigation Presets
- **Projection:** **Orthographic** by default, with a 1-key (`P`) toggle to **Perspective**.
- **View Controls & ViewCube:**
  - Interactive corner **ViewCube / Orientation Widget** + hotkeys for `Top`, `Bottom`, `Front`, `Back`, `Left`, `Right`, `Isometric`, `Fit All (F)`, **`Align View to Active Setup Top`**, and **`Look At Active Sketch`**.
- **Configurable Mouse Navigation Presets:**
  - **Fusion 360:** `MMB` Pan, `Shift + MMB` Orbit, `Scroll` Zoom-to-cursor.
  - **SolidWorks:** `MMB` Orbit, `Ctrl + MMB` Pan, `Scroll` Zoom-to-cursor.
  - **FreeCAD / Blender:** `MMB` Orbit, `Shift + MMB` Pan, `Scroll` Zoom-to-cursor.
  - **Orbit Mode Toggle:** `Turntable` (locks horizon to Setup `+Z`, default) vs. `Free Trackball`.

---

## Parametric Input Widgets, Discoverability, & Persistent Preferences
- **Two-Column Aligned Inspector Grids & Expression-Aware Inputs *(v3)*:**
  - Inspector forms use aligned two-column property grids (`inspector_row`) and collapsible section cards with uniform input widths and unit suffix badges.
  - Expression fields display resolved values in preferred display units (`mm` or `in`), offer a live scoped variable reference/autocomplete popover (`tool_d`, `tool_r`, `stock_x`, `feature_top_z`, etc.), support `Up`/`Down` arrow-key nudging and drag-scrubbing on plain numbers, validate syntax live, and commit on `Enter` or focus loss.
- **Active Pick / Sketch Guidance HUD, Welcome Overlay, & Command Palette *(v3)*:**
  - Whenever a 3D B-Rep pick mode, manual tab placement, or 2D sketch tool is active, a prominent floating guidance banner at the top-center of the viewport explains the expected next click and provides `[Done]` / `[Cancel (Esc)]` controls.
  - Launching `chip-gui` with no file argument opens a clean **Welcome / Quick-Start Overlay** (`Import 3D STEP`, `Import 2D DXF / SVG`, `Start Pure-2D Sketch`, `Recent Projects`, and a **Sample Projects Gallery** modal).
  - **`Ctrl+K` Command Palette** provides instant fuzzy search across all menu, view, sample, and operation-creation actions, paired with a **`?` / `F1` Keyboard Shortcut Cheat-Sheet** and **`Help → About ChipCutter…`** version/build info modal.
- **Setup `[Fixtures]` Tab, Interactive Placement, & Workholding Wizard Modals:**
  - The Setup Inspector's strict two-column **`[Fixtures]`** tab provides 1-click `+ Vise`, `+ Toe Clamp`, `+ Spoilboard`, `+ Keep-Out`, `+ Cylinder`, and `+ Import Fixture…` actions, interactive 3D viewport click-to-place (`Place Clamp in Viewport…`, `Place Pin in Viewport…`), live `Auto-Avoid Rapids` lift & collision banners, and direct buttons to launch the **Flip-Jig Wizard…** and **Auto Soft-Jaw Wizard…** modals (also accessible from the **`Setup`** top menu and **`Ctrl+K`** Command Palette).
  - Both wizard modals provide strict two-column parameter grids, universal hover tooltips, live symmetry/clearance/pocket/STL previews, single-step `< 1 ms` Undo on **Apply to Project**, and native / XDG-portal **`Export Soft Jaws (.stl)…`** folder export.
- **Tool Library Schematics, Undercut / 3D Deburr Inspector Cards, & Tri-Dexel Advisory:**
  - The **Tool Library Manager** provides strict two-column geometry editors, universal hover tooltips, and procedural 2D cross-section schematics for all 10 cutter geometries—including relieved-neck **Lollipop Endmill** (spherical head + neck join), **Slot / T-Slot Mill** (disc head + corner radius + neck), and **Dovetail Cutter** (inverted angled cone + top minor diameter readout).
  - The **Operation Inspector** renders dedicated strict two-column **`[Passes]`** cards for **`3D Deburr`** (`chamfer_width`, `tip_offset`, `wall_clearance`, auto-detect sharp edges, `allow_underside_back_chamfer` Lollipop badge), **`Slot / T-Slot Undercut`** (DIN 650 / ASME / DIN 6885 preset picker, throat/neck clearance safety banner, roof-safe `UndercutEntryMode`), **`Dovetail`** (`InternalSlot` / `ExternalRail`, angle-match & throat-fit diagnostics), and **`3D Lollipop Undercut`** (Z-stepdown / cusp height, overhang angle, neck clearance, open-air lead-out), all accessible from the `Operations` menu, CAM Browser, and `Ctrl+K` Command Palette.
  - When a Setup contains undercut cutters while the Simulation Timeline is set to 2.5D `Heightmap`, an inline amber advisory banner offers a 1-click **`Use Tri-Dexel`** switch; the **Sample Projects Gallery** includes the built-in **`Undercut & 3D Deburr Block`** (`--sample undercut-deburr`) showcase running in volumetric `Tri-Dexel` mode.
- **Persistent User Preferences (`prefs.toml` — *v3*):**
  - Stored in the user's platform config directory alongside the global tool library, persisting `AppTheme` (`Dark`/`Light`), default display units, mouse navigation preset, orbit mode, viewport visibility toggles, `egui_tiles` pane shares, and recent project paths across sessions.
- **Document Undo / Redo (`Ctrl+Z` / `Ctrl+Y`):**
  - Every committed mutation to Setups, Sketches, Selections, Operations, or Project Tools pushes a compact document state snapshot onto an Undo stack (default depth `100` steps, captured lazily only on actual mutation so idle 60 FPS frames perform zero document clones).
  - Heavy cached data (B-Rep meshes, generated toolpaths, swept 2D Rest polygons, heightmaps) is stored out-of-band by revision/key so Undo/Redo is instantaneous (`< 1 ms`).
