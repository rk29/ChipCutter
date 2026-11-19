# 02 — CAD Import, B-Rep Kernel, & Unified 3D/2D Selection

## CAD Kernel Abstraction (`chip-cad`)
All interaction with STEP files and B-Rep topology goes through an internal trait boundary (`CadKernel`) so `chip-cam` and `chip-gui` never depend directly on `monstertruck` data structures. This keeps `cargo build` self-contained while permitting an optional OpenCASCADE (`opencascade-rs`) backend behind a Cargo feature flag in future versions if needed.

### Core Domain Types
```text
CadDocument
 └── SolidBody (BodyId)
      ├── BRepFace (FaceId)
      │    ├── surface_kind: Plane { origin, normal }
      │    │               | Cylinder { axis_origin, axis_dir, radius, is_internal }
      │    │               | Cone { apex, axis_dir, half_angle }
      │    │               | Torus | NurbsSurface
      │    ├── outer_loop: LoopId
      │    ├── inner_loops: Vec<LoopId>
      │    └── signature: GeometricSignature
      ├── BRepLoop (LoopId)
      │    └── oriented_edges: Vec<(EdgeId, Orientation)>
      └── BRepEdge (EdgeId)
           ├── curve_kind: Line { start, end }
           │             | CircleArc { center, normal, radius, start, end }
           │             | EllipseArc | NurbsCurve
           └── adjacent_faces: SmallVec<[FaceId; 2]>
```

---

## STEP Import Pipeline
1. **Supported Schemas:** ISO 10303-21 STEP files (`.step`, `.stp`) covering AP203, AP214, and AP242 solid and shell representations via `monstertruck-io` (`step` feature).
2. **Unit Normalization:** Read the STEP file's `LENGTH_UNIT` (`MILLIMETRE`, `INCH`, `METRE`, etc.) and scale all imported geometry into canonical `f64` millimeters on load.
3. **Analytic Surface & Curve Classification:**
   - Even when a CAD exporter encodes planar faces or circular edges as degree-1/degree-2 rational B-splines, `chip-cad` runs an analytic recognizer on import (within `1e-5 mm` tolerance) to classify true `Plane`, `Cylinder`, `Cone`, `Line`, and `CircleArc` primitives. This is essential for hole recognition, WCS alignment, and `G2`/`G3` arc toolpaths.
4. **Viewport Tessellation & Pick Buffers:**
   - Each `BRepFace` is tessellated via `monstertruck-meshing` into an indexed triangle mesh tagged with its `FaceId` and vertex normals.
   - Each `BRepEdge` is discretized into polyline vertices tagged with its `EdgeId`.
   - CPU-side BVH / R-tree acceleration structures are built over face triangles, edge segments, and 2D `SketchLayer` curves/regions for sub-millisecond hover and click raycasting in the 3D viewport.

---

## Unified 3D B-Rep & 2D Sketch Selection Modes
All 2.5D selections operate relative to the active **Setup WCS** (`+Z` = spindle axis pointing up from part toward tool). Both 3D STEP topology and 2D `SketchLayer` entities (see [11-sketches-vectors.md](file:///workspace/ChipCutter/specs/11-sketches-vectors.md)) normalize into identical `ExtractedFeature25D` inputs for `chip-cam`:

| Selection Mode | User Action in Viewport | Extracted `ExtractedFeature25D` |
|---|---|---|
| **Planar Face** *(3D)* | Click a horizontal B-Rep face (`normal ∥ ±Z`) | `PlanarFace`: Outer `ArcPolyline2D` + inner/raised island loops + adjacent lower floor `Z` |
| **Sketch Region** *(2D — v2)* | Click inside a closed 2D Sketch/DXF/SVG/Text region | `PlanarFace`: Outer CCW `ArcPolyline2D` + enclosed CW counter/island loops at Sketch `Z` |
| **Closed Loop** *(3D or 2D)* | Click a B-Rep edge loop or closed 2D Sketch curve | `Contour` (closed): `ArcPolyline2D` projected onto Setup XY with `reverse` / `side_flip` |
| **Open Chain** *(3D or 2D)* | Click start/end B-Rep edges or open 2D Sketch/Hershey stroke | `Contour` (open): `ArcPolyline2D` for open contours, chamfers, or trace engraving |
| **Sketch Layer / Sub-Layer** *(2D — v2)* | Select an entire `SketchLayer` or named DXF/SVG layer / `Text` item | Expands all closed regions (for V-Carve / Pocket) or strokes (for Trace) dynamically when edited! |
| **Cylindrical Hole / 2D Hole Point** | Click internal B-Rep cylinder (`axis ∥ ±Z`) OR 2D Sketch `Circle` / `PointPattern` | `CylindricalHole`: Center `(X, Y)`, diameter `2R`, top `Z`, bottom `Z`, through-hole flag |
| **Same-Diameter / Coplanar Assist** | Modifier button ("Select Same Diameter" / "Select Coplanar Faces") | Expands current selection to matching holes or pocket floors on the same body/sketch |

### Automatic Z-Height Inference
Every operation maintains explicit **Top Height** and **Bottom Height** references, defaulting to values inferred from the selection:
- **Pocket on 3D Planar Floor Face:** Bottom = selected face `Z`; Top = highest adjacent wall top `Z` (or Setup Stock Top if open to top).
- **Operation on 2D Sketch Entity / Pure-2D Job *(v2)*:** Top = Sketch plane `Z` (default `Stock Top`); Bottom = `Top - cut_depth_offset` (e.g., `Top - 5 mm` for pockets, `Stock Bottom` for profile cutouts, or automatic variable medial depth / `flat_depth` for V-Carve).
- **Contour on Bottom Edge Loop:** Bottom = loop `Z`; Top = Setup Model Top / Stock Top.
- **Cylindrical Hole:** Top = cylinder upper rim `Z`; Bottom = cylinder lower rim `Z` (plus optional drill tip breakthrough compensation for through-holes).
- Every inferred height can be overridden by switching its reference mode (`From Selection`, `Stock Top`, `Stock Bottom`, `Model Top`, `Model Bottom`, `WCS Origin`) plus a signed offset expression.

---

## CAD Reload & Geometric Signature Re-Matching
STEP exports from parametric CAD tools do not guarantee stable entity numbering across edits. To prevent losing CAM work when a part is revised:

### Stored Selection Binding
Each B-Rep geometry reference (including `SketchAnchor::PlanarFace` anchors in v2!) in `.chip.toml` stores both the last-known topology index and a **GeometricSignature**:
- `entity_kind`: `Face` | `Loop` | `Edge` | `CylinderHole`
- `surface_or_curve_type`: e.g., `Plane`, `InternalCylinder`, `CircleArc`
- `normal_or_axis`: unit `DVec3`
- `centroid`: `DVec3` (mm)
- `characteristic_measure`: area (`mm²`), perimeter/length (`mm`), or radius (`mm`)
- `aabb`: `[min_xyz, max_xyz]`

*(Note: 2D `SketchLayer` entities created natively in ChipCutter carry stable `SketchEntityId`s; re-importing an updated external `.dxf`/`.svg` file re-matches layer names and closed-loop geometric signatures similarly.)*

### Re-Matching Algorithm on "Reload STEP Model"
1. Load and normalize the updated STEP file.
2. First re-bind each Setup's WCS alignment entities and any `SketchAnchor::PlanarFace` sketch planes referenced to B-Rep geometry.
3. Score candidate entities of matching `entity_kind` and `surface_or_curve_type` using a weighted distance over orientation (`cos θ > 0.999`), radius/area delta, and centroid shift:
   - **Exact / High-Confidence Match:** Automatically re-bind to the new `FaceId`/`LoopId`/`EdgeId` and mark dependent operations **Dirty**.
   - **Ambiguous or Missing Match:** Mark the operation (or face-anchored sketch) **Needs Re-pick** with a warning badge in the CAM tree while preserving all feeds, speeds, tool, and linking parameters for 1-click re-binding.
