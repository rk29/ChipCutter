# 11 — 2D Sketch Layers, DXF / SVG Vector Import, & Text Layout (`chip-sketch`)

## Overview
While ChipCutter v1 targeted 3D STEP solids exclusively, CNC routers, panel makers, and machine shops routinely cut pure-2D sheet goods (DXF profiles, SVG logos, front-panel cutouts) or engrave part numbers, alignment scribes, and V-carved text onto 3D machined parts.

`chip-sketch` is a UI-agnostic core crate (`chip-math` -> `chip-sketch` -> `chip-cam` / `chip-project`) that provides:
1. **Planar 2D Sketch Layers** supporting both **standalone pure-2D jobs** (zero `.step` files required) and **3D-anchored sketches** attached associatively to Setup XY or planar STEP B-Rep faces.
2. **DXF & SVG Vector Importers** with unit/DPI normalization, recursive 3-point circumcircle arc fitting into true circular arcs (`ArcPolyline2D`), and deterministic gap-healing / loop topology reconstruction.
3. **Parametric 2D CAM Primitives** (rectangles, circles, slots, polygons, polylines, linear/bolt-circle hole patterns) with inline unit/math expressions—intentionally scoped without a nonlinear CAD geometric constraint solver.
4. **CAM Typography** supporting both **TrueType / OpenType closed-outline fonts** (`skrifa`, for V-Carving, Pocketing, and Profiling) and **built-in single-line Hershey engraving fonts** (for fast 1-pass scribed part numbers and labels).

---

## Sketch Layer Coordinate Frames & Anchoring

A project stores zero or more ordered `SketchLayer`s identified by plain `String` IDs (`SketchLayer::id`, `SketchEntityEntry::id`, `PlanarVectorRegion::id`). Each `SketchLayer` defines an orthonormal 2D plane `(O_sketch, X_sketch, Y_sketch, Z_normal)` mapped into the active Setup WCS:

### Anchor Modes (`SketchAnchor`)
1. **`SketchAnchor::SetupXy` *(Default for Pure-2D Jobs & Top-Side Layouts)*:**
   - Fields: `setup_id: String`, `plane_ref: SketchPlaneRef`, `z_offset: ParamLength`.
   - Aligned directly with the referenced Setup's WCS `XY` plane (`X_sketch = +X_wcs`, `Y_sketch = +Y_wcs`, `+Z_normal = +Z_wcs`).
   - `plane_ref` selects `SketchPlaneRef::{StockTop, ModelTop, StockBottom, WcsOrigin}` (`StockTop` default) plus signed expression offset `z_offset`.
   - Works identically whether the project contains 3D STEP models or `models = []` (pure-2D mode).
2. **`SketchAnchor::PlanarFace` *(Associative 3D B-Rep Sketch)*:**
   - Fields: `setup_id: String`, `face_binding: SketchFaceBinding`, `origin_mode: FaceSketchOriginMode`, `x_axis_clock_deg: f64`, `z_offset: ParamLength`.
   - Anchored to a planar B-Rep face via `SketchFaceBinding` (`body_id`, `last_face_index`, `normal_world`, `centroid_world_mm`, `area_mm2`, `aabb_mm`), mapping 1-to-1 with `chip_cad::SelectionBinding` for automatic re-binding on **Reload STEP Model**.
   - `origin_mode`: `FaceSketchOriginMode::{FaceCentroid, SetupWcsProjected}` (`FaceCentroid` default).
   - `x_axis_clock_deg` clocks `+X_sketch` within the face plane (`+Z_normal` points outward along the B-Rep face normal).
   - If a Setup's `+Z_wcs` matches `+Z_normal` (`cos θ > 0.9999`), all entities on the sketch layer are directly selectable for 2.5D machining in that Setup.

---

## Parametric 2D Sketch Entities

Every entity inside a `SketchLayer` is stored in a `SketchEntityEntry` carrying a stable `id: String`, human-readable `name: String`, optional `sub_layer: Option<String>` (preserving imported DXF/SVG layers), `is_construction: bool` toggle (construction geometry aids snapping/alignment but is ignored by region classification and selection), and `entity: SketchEntity`. Entities evaluate on demand into canonical `f64` millimeter `ArcPolyline2D`s (`to_arc_polylines`) or hole lists (`evaluate_holes`), while classified closed loops are stored in `SketchLayer::regions: Vec<PlanarVectorRegion>`:

| `SketchEntity` Variant | Parametric Expression / Schema Fields | Evaluated CAM Geometry |
|---|---|---|
| **`Rectangle`** | `anchor: RectAnchorMode` (`Center` \| `FrontLeft`), `x`, `y`, `width`, `height`, `corner_radius` (`>= 0`), `rotation_deg` | Closed `ArcPolyline2D` (with exact `90°` corner arcs when `corner_radius > 0`) |
| **`Circle`** | `center_x`, `center_y`, `diameter` | Closed 2-arc `ArcPolyline2D` + center point (usable as either a 2D Contour/Pocket loop OR a Drill/Bore hole) |
| **`Slot`** | `mode: SlotLengthMode` (`CenterToCenter` \| `OverallLength`), `x`, `y`, `length`, `width`, `angle_deg` | Closed stadium `ArcPolyline2D` (`2` tangent lines + `2` `180°` end arcs) |
| **`RegularPolygon`** | `center_x`, `center_y`, `sides` (`3..=64`), `sizing: PolygonSizingMode` (`InscribedDiameter` \| `CircumscribedDiameter` \| `AcrossFlats`), `diameter`, `corner_radius`, `rotation_deg` | Closed `ArcPolyline2D` (ideal for hex nut pockets and hex bosses) |
| **`Polyline`** | `polyline: ArcPolyline2D`, `translate_x`, `translate_y`, `scale`, `rotation_deg` | Closed loop or open chain preserving analytic lines and circular arcs |
| **`PointPattern(PointPatternSpec)`** | `Single { x, y, hole_diameter }`, `LinearGrid { origin_x, origin_y, count_x, count_y, spacing_x, spacing_y, angle_deg, hole_diameter }`, or `BoltCircle { center_x, center_y, bolt_circle_diameter, hole_diameter, count, start_angle_deg }` | Ordered `Vec<HolePoint2D>` feeding directly into **Drilling & Helical Bore** (and circle polylines for visual/contour evaluation) |
| **`Text(SketchTextSpec)`** | `content`, `font: SketchFontSource`, `cap_height`, `x`, `y`, `h_align` (`Left`\|`Center`\|`Right`), `v_align` (`Baseline`\|`Middle`\|`Top`), `letter_spacing`, `line_spacing_factor`, `rotation_deg`, `mirror_x`, `arc_fit_tolerance_mm` | Closed outer+island glyph loops (**Outline**) OR single-pass stroke chains (**SingleLine**) |

### Expression Scope Inside Sketches
Parametric fields inside `SketchLayer`s evaluate with `chip-math`'s unit/expression parser and can reference Setup variables (`stock_x`, `stock_y`, `stock_z`) so panel layouts can center or offset relative to stock dimensions (e.g., `x = "stock_x / 2"`, `width = "stock_x - 1.0 in"`).

---

## CAM Typography: Outline Fonts & Single-Line Engraving

1. **Outline Fonts (TrueType / OpenType `.ttf` / `.otf` via `skrifa`):**
   - Ships with a bundled permissively licensed bold sans-serif default font (`DejaVuSans-Bold.ttf`, `SketchFontSource::BuiltinSansBold`) so headless CLI builds and portable `.chipz` archives reproduce identical glyph geometry across Linux, macOS, and Windows, plus support for project-referenced custom `.ttf`/`.otf` font files (`SketchFontSource::CustomFile { path, sha256, face_index }`).
   - Extracts exact glyph outlines via `skrifa`, applies `kern` table pair adjustments, scales by `cap_height`, converts quadratic/cubic Bézier curves to `ArcPolyline2D` via recursive 3-point circumcircle arc fitting within `arc_fit_tolerance_mm` (default `0.008 mm`, computed as `0.008_f64.max(0.5 * DEFAULT_ARC_FIT_TOL_MM)`), welds overlapping glyph contours via `cavalier_contours` boolean union (`try_union_overlapping_ccw_loops`), and classifies outer contours vs. counter islands (e.g., inside `O`, `A`, `B`, `8`, `e`).
   - Directly drives **V-Carving**, **2D Pocket / Adaptive**, **2D Contour**, and **2D Trace**.
2. **Single-Line Engraving Fonts (`SketchFontSource::Hershey { font: HersheyFontKind }`):**
   - Standard outline fonts produce *double-line* hollow borders if traced directly with a cutter. For crisp 1-pass part numbers, serial tags, and dial graduations, `chip-sketch` embeds a single-stroke vector glyph table (`21`-unit cap height) supporting four `HersheyFontKind` styles sharing the base stroke table with style modifiers:
     - `HersheyFontKind::SimplexSans` *(default clean industrial sans-serif)*
     - `HersheyFontKind::DuplexRoman` *(adds serif modifiers on key uppercase stems)*
     - `HersheyFontKind::Mono` *(fixed `16`-unit character advance + slashed zero `'0'`)*
     - `HersheyFontKind::Script` *(italic horizontal shear `0.22`)*
   - Evaluates to open/closed centerline `ArcPolyline2D` strokes intended for **2D Curve Trace / Engraving** at fixed axial depth.

---

## DXF & SVG Import Pipeline

### 1. DXF Importer (`dxf` crate, MIT)
- **Supported Entities:** `LINE`, `ARC`, `CIRCLE`, `LWPOLYLINE` (preserving exact bulge arcs directly), `POLYLINE`, `ELLIPSE` (arc-fitted), `SPLINE` (custom Cox-de Boor NURBS evaluation into a `kurbo::BezPath`, then circular-arc fitted via `curve_fit.rs`), and recursive `INSERT` block expansion with scale/rotation transforms (bounded by `MAX_DXF_IMPORTED_ITEMS` and `MAX_DXF_INSERT_EXPANSIONS` safety limits).
- **Units Resolution (`VectorImportUnits`):** Reads DXF header `$INSUNITS` (`Millimeters`, `Inches`, `Centimeters`, `Meters`, `Feet`). Because many CAD tools export `"Unitless"` DXF files, the importer exposes `VectorImportUnits::{AutoDetect { fallback }, Millimeters, Inches, Centimeters}` (CLI option and live dimension preview in GUI).
- **Layer Preservation:** Preserves DXF layer names on imported `SketchEntityEntry::sub_layer`, hard-skipping `DEFPOINTS` and off layers (`!layer.is_layer_on`), with an optional `include_layers: Option<HashSet<String>>` allow-list filter on `DxfImportOptions`.

### 2. SVG Importer (`usvg` + `kurbo`, Apache-2.0 / MIT)
- Parses W3C SVG 1.1 / 2.0 documents via `usvg`, flattening `<g transform="...">`, `<use>`, `<rect>`, `<circle>`, `<ellipse>`, `<line>`, `<polyline>`, `<polygon>`, and `<path>` commands into world-space `kurbo::BezPath`s.
- **Physical Scale & DPI:** Honors physical SVG width/height units (`mm`, `cm`, `in`, `pt`, `pc`). When width/height are unitless pixels (`px`), converts using configurable `svg_dpi` (`96.0` CSS/Inkscape 0.92+ standard default, with `72.0` Adobe Illustrator and `90.0` legacy Inkscape presets), with an optional `units_override: Option<VectorImportUnits>` (`Millimeters`, `Inches`, `Centimeters`) to force user-space pixel scale.
- **Y-Axis Flip:** Automatically flips SVG's top-left `+Y down` screen convention into right-handed CAM `+Y up` coordinates anchored at the artwork origin.

### 3. Shared Curve Fitting, Gap-Healing, & Region Classification
Raw CAD/illustration exports frequently contain disconnected segments, tiny endpoint gaps, duplicate overlapping lines, or faceted curves:
1. **Recursive 3-Point Circumcircle Arc Fitting (`curve_fit.rs`):**
   - `fit_bezpath_to_arc_polylines` raises quadratic Béziers to cubics and recursively subdivides cubic Bézier segments (from SVG paths, DXF `SPLINE`/`ELLIPSE` curves, and `skrifa` TrueType outlines) into `ArcSegment2D::Arc` (via 3-point circumcircle fitting) or `ArcSegment2D::Line` spans within `arc_fit_tolerance_mm` (default `DEFAULT_ARC_FIT_TOL_MM = 0.01 mm` for DXF/SVG, `0.008 mm` for text) so controllers receive compact, smooth `G2`/`G3` toolpaths.
2. **Deduplication & Gap-Healing Chainer (`heal_chain.rs`):**
   - Removes exact/near-exact duplicate overlapping segments and circles within `1e-4 mm`.
   - Chains open segment endpoints within configurable `join_tolerance_mm` (default `0.05 mm`, `0.0` to disable) using an R-tree (`rstar`) nearest-endpoint matcher, inserting micro-bridge closures or snapping endpoints cleanly when `gap <= join_tolerance_mm`.
   - Emits non-fatal `SketchImportDiagnostic` notes (`SketchDiagnosticKind::{HealedEndpointGap, DuplicateSegmentRemoved, OpenChainRemaining, DegenerateSkipped}`) highlighting open chain endpoints and healed gaps in the viewport.
3. **Planar Region Hierarchy (`PlanarVectorRegion`):**
   - Closed loops on a `SketchLayer` are partitioned per `sub_layer`, filtered for non-self-intersection via `chip_geom2d::is_simple_closed_polyline` (self-intersecting loops are silently dropped from region classification without emitting a diagnostic), and organized into an even-odd containment forest (outer `CCW` loops paired with direct inner `CW` island/hole loops) using `ArcPolyline2D::contains_point`. Clicking inside a closed 2D region in the viewport (e.g., between two concentric circles or inside a letter `'B'`) automatically selects the outer loop together with its enclosed islands—matching 3D planar B-Rep face selection ergonomics.

---

## Interactive Viewport Authoring & Snapping (`chip-gui`)
When creating or editing a `SketchLayer` in `chip-gui`:
- Camera offers a 1-click **Look At Sketch Plane** orthographic view (while remaining fully viewable and pickable in 3D orbit alongside stock and STEP bodies).
- **2D Snapping Engine:** Cursor snaps to Grid increments, Sketch Origin (`X0 Y0`), Entity Endpoints/Midpoints/Arc Centers, Stock Corners/Center, and projected STEP B-Rep vertices/hole centers, with live numeric dimension HUD badges.
