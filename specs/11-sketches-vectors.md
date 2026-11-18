# 11 — 2D Sketch Layers, DXF / SVG Vector Import, & Text Layout (`chip-sketch`)

## Overview
While ChipCutter v1 targeted 3D STEP solids exclusively, CNC routers, panel makers, and machine shops routinely cut pure-2D sheet goods (DXF profiles, SVG logos, front-panel cutouts) or engrave part numbers, alignment scribes, and V-carved text onto 3D machined parts.

`chip-sketch` is a UI-agnostic core crate (`chip-math` -> `chip-sketch` -> `chip-cam` / `chip-project`) that provides:
1. **Planar 2D Sketch Layers** supporting both **standalone pure-2D jobs** (zero `.step` files required) and **3D-anchored sketches** attached associatively to Setup XY or planar STEP B-Rep faces.
2. **DXF & SVG Vector Importers** with unit/DPI normalization, cubic Bézier biarc fitting into true circular arcs (`ArcPolyline2D`), and deterministic gap-healing / loop topology reconstruction.
3. **Parametric 2D CAM Primitives** (rectangles, circles, slots, polygons, polylines, linear/bolt-circle hole patterns) with inline unit/math expressions—intentionally scoped without a nonlinear CAD geometric constraint solver.
4. **CAM Typography** supporting both **TrueType / OpenType closed-outline fonts** (for V-Carving, Pocketing, and Profiling) and **built-in single-line Hershey engraving fonts** (for fast 1-pass scribed part numbers and labels).

---

## Sketch Layer Coordinate Frames & Anchoring

A project stores zero or more ordered `SketchLayer`s (`SketchId`). Each `SketchLayer` defines an orthonormal 2D plane `(O_sketch, X_sketch, Y_sketch, Z_normal)` mapped into the active Setup WCS:

### Anchor Modes (`SketchAnchor`)
1. **`SetupXY` *(Default for Pure-2D Jobs & Top-Side Layouts)*:**
   - Aligned directly with a Setup's WCS `XY` plane (`X_sketch = +X_wcs`, `Y_sketch = +Y_wcs`, `+Z_normal = +Z_wcs`).
   - `plane_z`: Reference Z height (`Stock Top` *(default)*, `Model Top`, `Stock Bottom`, or `WCS Origin`) plus signed expression offset `z_offset`.
   - Works identically whether the project contains 3D STEP models or `models = []` (pure-2D mode).
2. **`PlanarFace` *(Associative 3D B-Rep Sketch)*:**
   - Anchored to a planar `FaceId` on an imported `ModelId`, storing a `GeometricSignature` for automatic re-binding on **Reload STEP Model**.
   - `origin_mode`: `FaceCentroid`, `SetupWcsProjected`, or `SelectedVertexOrHoleCenter`.
   - `x_axis_clock_deg` (or reference linear `EdgeId`): Clocks `+X_sketch` within the face plane (`+Z_normal` points outward along the B-Rep face normal).
   - If a Setup's `+Z_wcs` matches `+Z_normal` (`cos θ > 0.9999`), all entities on the sketch layer are directly selectable for 2.5D machining in that Setup.

---

## Parametric 2D Sketch Entities

Every entity inside a `SketchLayer` carries a stable `SketchEntityId`, human-readable `name`, optional sub-layer/group name (preserving imported DXF/SVG layers), `is_construction: bool` toggle (construction geometry aids snapping/alignment but is ignored by region selection), and an `evaluated` cache of canonical `f64` millimeter `ArcPolyline2D`s or 2D points:

| Entity Variant | Parametric Expression Fields | Evaluated CAM Geometry |
|---|---|---|
| **`Rectangle`** | `anchor` (`Center` \| `FrontLeft`), `x`, `y`, `width`, `height`, `corner_radius` (`>= 0`), `rotation_deg` | Closed `ArcPolyline2D` (with exact `90°` corner arcs when `corner_radius > 0`) |
| **`Circle`** | `center_x`, `center_y`, `diameter` | Closed 2-arc `ArcPolyline2D` + center point (usable as either a 2D Contour/Pocket loop OR a Drill/Bore hole!) |
| **`Slot`** | `mode` (`CenterToCenter` \| `OverallLength`), `x`, `y`, `length`, `width`, `angle_deg` | Closed stadium `ArcPolyline2D` (`2` tangent lines + `2` `180°` end arcs) |
| **`RegularPolygon`** | `center_x`, `center_y`, `sides` (`3..=64`), `sizing` (`InscribedDiameter` \| `CircumscribedDiameter` \| `AcrossFlats`), `diameter`, `corner_radius`, `rotation_deg` | Closed `ArcPolyline2D` (ideal for hex nut pockets and hex bosses) |
| **`Polyline`** | Closed or open `ArcPolyline2D` (`vertices` with bulge factors `tan(θ/4)`), `translate_xy`, `scale`, `rotation_deg` | Closed loop or open chain preserving analytic lines and circular arcs |
| **`PointPattern`** | `Single { x, y }`, `LinearGrid { origin_xy, count_x, count_y, spacing_x, spacing_y, angle_deg }`, or `BoltCircle { center_xy, bolt_circle_diameter, hole_diameter, count, start_angle_deg }` | Ordered `Vec<HolePoint2D>` with nominal `hole_diameter` feeding directly into **Drilling & Helical Bore** |
| **`Text`** | `content`, `font_source`, `cap_height`, `origin_xy`, `h_align` (`Left`\|`Center`\|`Right`), `v_align` (`Baseline`\|`Middle`\|`Top`), `letter_spacing`, `line_spacing_factor`, `rotation_deg`, `mirror_x` | Closed outer+island glyph loops (**Outline**) OR single-pass stroke chains (**SingleLine**) |

### Expression Scope Inside Sketches
Parametric fields inside `SketchLayer`s evaluate with `chip-math`'s unit/expression parser and can reference Setup variables (`stock_x`, `stock_y`, `stock_z`) so panel layouts can center or offset relative to stock dimensions (e.g., `x = "stock_x / 2"`, `width = "stock_x - 1.0 in"`).

---

## CAM Typography: Outline Fonts & Single-Line Engraving

1. **Outline Fonts (TrueType / OpenType `.ttf` / `.otf` via `ttf-parser`):**
   - Ships with a bundled permissively licensed sans-serif default font (`OFL-1.1` / `Apache-2.0`) so headless CLI builds and portable `.chipz` archives reproduce identical glyph geometry across Linux, macOS, and Windows, plus support for project-embedded custom `.ttf`/`.otf` font files.
   - Extracts exact glyph outlines, applies `kern` table pair adjustments, scales by `cap_height` (or `em_size`), converts quadratic/cubic Bézier curves to `ArcPolyline2D` via biarc approximation within `arc_fit_tolerance_mm` (default `0.005 mm`), and classifies outer contours vs. counter islands (e.g., inside `O`, `A`, `B`, `8`, `e`) via `cavalier_contours` boolean union (automatically welding overlapping script glyphs!).
   - Directly drives **V-Carving**, **2D Pocket / Adaptive**, **2D Contour**, and **2D Trace**.
2. **Single-Line Engraving Fonts (Built-In Public-Domain Hershey Vector Fonts):**
   - Standard outline fonts produce *double-line* hollow borders if traced directly with a cutter. For crisp 1-pass part numbers, serial tags, and dial graduations, `chip-sketch` embeds public-domain NBS Hershey stroke tables:
     - `HersheySimplexSans` *(default clean industrial sans-serif)*
     - `HersheyDuplexRoman`
     - `HersheyMono`
     - `HersheyScript`
   - Evaluates to open/closed centerline `ArcPolyline2D` strokes intended for **2D Curve Trace / Engraving** at fixed axial depth.

---

## DXF & SVG Import Pipeline

### 1. DXF Importer (`dxf` crate, MIT)
- **Supported Entities:** `LINE`, `ARC`, `CIRCLE`, `LWPOLYLINE` (preserving exact bulge arcs directly!), `POLYLINE`, `ELLIPSE` (arc-approximated), `SPLINE` (degree-2/3 NURBS evaluated via `kurbo` and biarc-fitted), `POINT`, and recursive `INSERT` block expansion with scale/rotation transforms.
- **Units Resolution:** Reads DXF header `$INSUNITS` (`Millimeters`, `Inches`, `Centimeters`, `Meters`, `Feet`). Because many CAD tools export `"Unitless"` DXF files, the importer exposes an explicit `ImportUnits` parameter (`AutoDetectWithFallback(mm | in)` in CLI, live dimension preview in GUI).
- **Layer Preservation:** Preserves DXF layer names on imported `SketchEntity` items (skipping frozen/off layers and `DEFPOINTS` by default, configurable on import).

### 2. SVG Importer (`usvg` + `kurbo`, Apache-2.0 / MIT)
- Parses W3C SVG 1.1 / 2.0 documents, flattening `<g transform="...">`, `<use>`, `<clipPath>` (optional), `<rect>`, `<circle>`, `<ellipse>`, `<line>`, `<polyline>`, `<polygon>`, and `<path>` commands into world-space `kurbo::BezPath`s.
- **Physical Scale & DPI:** Honors physical SVG width/height units (`mm`, `cm`, `in`, `pt`, `pc`). When width/height are unitless pixels (`px`), converts using configurable `svg_dpi` (`96.0` CSS/Inkscape 0.92+ standard default, with `72.0` Adobe Illustrator and `90.0` legacy Inkscape presets) plus optional target width/height scaling on import.
- **Y-Axis Flip:** Automatically flips SVG's top-left `+Y down` screen convention into right-handed CAM `+Y up` coordinates anchored at the viewBox/artwork origin.

### 3. Shared Curve Fitting, Gap-Healing, & Region Classification
Raw CAD/illustration exports frequently contain disconnected segments, tiny endpoint gaps, duplicate overlapping lines, or faceted curves:
1. **Exact Biarc / Circular-Arc Fitting (`kurbo` -> `ArcPolyline2D`):**
   - Cubic and quadratic Bézier segments (from SVG, DXF `SPLINE`, and TrueType glyphs) are approximated into G1-continuous circular arcs (`ArcSegment2D`) within `arc_fit_tolerance_mm` (default `0.01 mm`), collapsing collinear spans into `Line` segments so controllers receive compact, smooth `G2`/`G3` toolpaths.
2. **Deduplication & Gap-Healing Chainer:**
   - Removes exact/near-exact duplicate overlapping segments within `coincidence_tol` (`1e-4 mm`).
   - Chains open segment endpoints within configurable `join_tolerance_mm` (default `0.05 mm`, `0.0` to disable) using an R-tree (`rstar`) nearest-endpoint matcher, inserting micro-bridge closures or snapping endpoints cleanly when `gap <= join_tolerance_mm`.
   - Emits non-fatal `SketchImportDiagnostic` warnings (highlighting open tail vertices or self-intersections in the viewport!).
3. **Planar Region Hierarchy (`PlanarVectorRegion`):**
   - Closed non-self-intersecting loops on a `SketchLayer` are organized into a containment forest (outer CCW loops paired with direct inner CW island/hole loops) via `cavalier_contours` winding/containment queries. Clicking inside a closed 2D region in the viewport (e.g., between two concentric circles or inside a letter `'B'`) automatically selects the outer loop together with its enclosed islands—matching 3D planar B-Rep face selection ergonomics!

---

## Interactive Viewport Authoring & Snapping (`chip-gui`)
When creating or editing a `SketchLayer` in `chip-gui`:
- Camera offers a 1-click **Look At Sketch Plane** orthographic view (while remaining fully viewable and pickable in 3D orbit alongside stock and STEP bodies).
- **2D Snapping Engine:** Cursor snaps to Grid increments, Sketch Origin (`X0 Y0`), Entity Endpoints/Midpoints/Arc Centers, Stock Corners/Center, and projected STEP B-Rep vertices/hole centers, with live numeric dimension HUD badges.
