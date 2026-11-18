# 08 — User Interface (`egui`) & 3D/2D Viewport (`wgpu`)

## Workspace Shell (`chip-gui`)
ChipCutter uses `eframe` with the `wgpu` backend and `egui_tiles` for a responsive, high-DPI, dark/light-theme CAD/CAM workspace with a **1-click "Reset Default Layout"** action in the View menu.

### Default Multi-Pane Layout
```text
┌──────────────────────────────────────────────────────────────────────────────┐
│ Menu Bar: File | Edit | Sketch | Setup | Operations | Tools | Simulate | Post│
├──────────────────────────────────────────────────────────────────────────────┤
│ Contextual Toolbar: [Import STEP] [Import DXF/SVG] [New Sketch] | [Face]     │
│   [Adaptive][Pocket][Contour][Drill][Chamfer][V-Carve] | [Gen (Ctrl+G)] [Sim]│
├────────────────────┬────────────────────────────────────┬────────────────────┤
│ CAM Browser        │                                    │ Inspector          │
│ (Left Tile)        │                                    │ (Right Tile)       │
│                    │                                    │                    │
│ ▼ Models           │       3D CAD / 2D Sketch           │ [Tool] [Geometry]  │
│   • bracket.step   │       Viewport (`egui_wgpu`)       │ [Heights] [Passes] │
│ ▼ Sketches (v2)    │                                    │ [Linking]          │
│   • Top Logo.svg   │   (Or Interactive Sketch Toolbar:  │                    │
│   • Serial Text    │    Rect | Circle | Slot | Poly |   │ Parametric fields  │
│ ▼ Setup 1 (G54)    │    Line/Arc | Holes | Text)        │ w/ inline math &   │
│   • Stock & WCS    │                                    │ unit expressions   │
│   ✓ 1. Adaptive    │                                    │           [ViewCube]│
│   ✓ 2. Rest Cont.  │                                    │                    │
│   ✓ 3. V-Carve     │                                    │                    │
├────────────────────┴────────────────────────────────────┴────────────────────┤
│ Bottom Tile Tabs: [Simulation Timeline] | [G-Code Preview] | [Diagnostics]   │
└──────────────────────────────────────────────────────────────────────────────┘
```

### Status Badges in CAM Browser
- `✓` **Clean** (green): Up-to-date cached toolpath.
- `⟳` **Dirty** (amber): Parameters, tool, setup, upstream Rest Machining op, CAD, or Sketch changed; awaits `Ctrl+G`.
- `◐` **Generating** (animated spinner): Background `rayon` task in flight.
- `⚠` **Error / Needs Re-pick** (red): Validation error, empty cut, crash detected, or orphaned B-Rep selection after STEP reload.
- `⊘` **Suppressed** (dimmed strikethrough): Skipped in generation, Rest Machining coverage, and post output.

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
4. **Stock & WCS Gizmo Pass:**
   - Translucent wireframe/tinted box or cylinder for Setup stock; RGB XYZ triad arrows (`X`=Red, `Y`=Green, `Z`=Blue) anchored at the WCS origin; translucent Z-height reference planes (`Clearance`, `Retract`, `Top`, `Bottom`) shown live while editing the **Heights** tab.
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

## Parametric Input Widgets & Zero-Idle-Clone Undo/Redo
- **Expression-Aware Numeric Widget:**
  - Displays resolved value in preferred display units (`mm` or `in`), expands to show raw formula when focused (e.g., `tool_d * 0.20`), validates syntax live, and commits on `Enter` or focus loss.
- **Document Undo / Redo (`Ctrl+Z` / `Ctrl+Y`):**
  - Every committed mutation to Setups, Sketches, Selections, Operations, or Project Tools pushes a compact document state snapshot onto an Undo stack (default depth `100` steps, captured lazily only on actual mutation so idle 60 FPS frames perform zero document clones).
  - Heavy cached data (B-Rep meshes, generated toolpaths, swept 2D Rest polygons, heightmaps) is stored out-of-band by revision/key so Undo/Redo is instantaneous (`< 1 ms`).
