# 08 — User Interface (`egui`) & 3D Viewport (`wgpu`)

## Workspace Shell (`chip-gui`)
ChipCutter uses `eframe` with the `wgpu` backend and `egui_tiles` for a responsive, high-DPI, dark/light-theme CAD/CAM workspace with a **1-click "Reset Default Layout"** action in the View menu.

### Default Multi-Pane Layout
```text
┌──────────────────────────────────────────────────────────────────────────────┐
│ Menu Bar: File | Edit | Setup | Operations | Tools | Simulate | Post | View  │
├──────────────────────────────────────────────────────────────────────────────┤
│ Contextual Toolbar: [Import STEP] [New Setup] | [Face][Contour][Pocket][Drill]│
│                     [Chamfer] | [Generate Dirty (Ctrl+G)] [Simulate] [Post]  │
├────────────────────┬────────────────────────────────────┬────────────────────┤
│ CAM Browser        │                                    │ Inspector          │
│ (Left Tile)        │                                    │ (Right Tile)       │
│                    │                                    │                    │
│ ▼ Models           │         3D CAD/CAM Viewport        │ [Tool] [Geometry]  │
│   • bracket.step   │         (`egui_wgpu` Custom        │ [Heights] [Passes] │
│ ▼ Setup 1 (G54)    │          Render Pipeline)          │ [Linking]          │
│   • Stock & WCS    │                                    │                    │
│   ✓ 1. Face Top    │                                    │ Parametric fields  │
│   ⟳ 2. Outside Cnt │                                    │ w/ inline math &   │
│   ✓ 3. Center Pkt  │                                    │ unit expressions   │
│   ⚠ 4. M5 Holes    │                                    │           [ViewCube]│
├────────────────────┴────────────────────────────────────┴────────────────────┤
│ Bottom Tile Tabs: [Simulation Timeline] | [G-Code Preview] | [Diagnostics]   │
└──────────────────────────────────────────────────────────────────────────────┘
```

### Status Badges in CAM Browser
- `✓` **Clean** (green): Up-to-date cached toolpath.
- `⟳` **Dirty** (amber): Parameters, tool, setup, or CAD changed; awaits `Ctrl+G`.
- `◐` **Generating** (animated spinner): Background `rayon` task in flight.
- `⚠` **Error / Needs Re-pick** (red): Validation error, empty cut, crash detected, or orphaned B-Rep selection after STEP reload.
- `⊘` **Suppressed** (dimmed strikethrough): Skipped in generation and post output.

---

## Custom `wgpu` 3D Viewport Pipeline
Integrated into the central `egui` panel via `egui_wgpu::CallbackTrait` rendering into a multisampled (`4x MSAA`) color + depth target:

### Render Passes
1. **Background Gradient / Grid Pass:** Subtle studio vertical gradient + optional WCS XY reference grid with major/minor millimeter/inch ticks.
2. **Shaded B-Rep Solid Pass:**
   - Phong/PBR-lite studio lighting locked relative to camera (headlight + rim + ambient) so undersides and deep pockets remain legible from any orbit angle.
   - Per-face state tinting: neutral metallic gray (default), soft blue highlight (hovered `FaceId`), vivid cyan/gold highlight (selected `FaceId`s), translucent ghosting mode when inspecting internal holes/toolpaths.
3. **Topological B-Rep Edge Pass:**
   - Screen-space width-stabilized line shader (with slight depth bias toward camera to eliminate Z-fighting) rendering exact `BRepEdge` curves, highlighting hovered/chained loops and open-edge directions with 3D directional cones.
4. **Stock & WCS Gizmo Pass:**
   - Translucent wireframe/tinted box or cylinder for Setup stock; RGB XYZ triad arrows (`X`=Red, `Y`=Green, `Z`=Blue) anchored at the WCS origin; translucent Z-height reference planes (`Clearance`, `Retract`, `Top`, `Bottom`) shown live while editing the **Heights** tab in the Inspector!
5. **Instanced Toolpath & Tab Pass:**
   - GPU-instanced line/arc segments supporting `500,000+` segments at `60 FPS`, plus 3D tab markers (draggable along contour edges).
6. **Simulation Stock & Cutter Pass:**
   - Dynamic heightmap mesh (when Stock Removal view is active) + animated 3D tool/shank/holder mesh.

---

## Camera Projection & Mouse Navigation Presets
- **Projection:** **Orthographic** by default (essential for judging vertical walls, tool clearances, and Z planes), with a 1-key (`P`) toggle to **Perspective**.
- **View Controls & ViewCube:**
  - Interactive corner **ViewCube / Orientation Widget** + hotkeys for `Top`, `Bottom`, `Front`, `Back`, `Left`, `Right`, `Isometric`, `Fit All (F)`, and **`Align View to Active Setup Top`** (orients camera straight down Setup `-Z` with `+X` right and `+Y` up, regardless of raw CAD model orientation!).
- **Configurable Mouse Navigation Presets** (selectable in Preferences):
  - **Fusion 360:** `MMB` Pan, `Shift + MMB` Orbit, `Scroll` Zoom-to-cursor.
  - **SolidWorks:** `MMB` Orbit, `Ctrl + MMB` Pan, `Scroll` Zoom-to-cursor.
  - **FreeCAD / Blender:** `MMB` Orbit, `Shift + MMB` Pan, `Scroll` Zoom-to-cursor.
  - **Orbit Mode Toggle:** `Turntable` (locks horizon to Setup `+Z`, default) vs. `Free Trackball`.

---

## Parametric Input Widgets & Undo/Redo
- **Expression-Aware Numeric Widget:**
  - Custom `egui` wrapper around `TextEdit` / `DragValue`: displays resolved value in preferred display units (`mm` or `in`), expands to show raw formula when focused (e.g., `tool_d * 0.35 + 0.1mm`), validates syntax live with inline tooltip preview, and commits on `Enter` or focus loss.
- **Document Undo / Redo (`Ctrl+Z` / `Ctrl+Y`):**
  - Every committed mutation to Setups, Selections, Operations, or Project Tools pushes a compact immutable document state diff/snapshot onto an Undo stack (default depth `100` steps).
  - Heavy cached data (B-Rep meshes, generated toolpaths, heightmaps) is stored out-of-band by content/revision hash so Undo/Redo is instantaneous (`< 1 ms`) and consumes negligible RAM; undoing back to a previously generated state immediately restores its cached `Clean` toolpath if still resident!
