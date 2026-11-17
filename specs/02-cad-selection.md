# 02 — CAD Import, B-Rep Kernel, & 3D Selection

## CAD Kernel Abstraction (`chip-cad`)
All interaction with STEP files and B-Rep topology goes through an internal trait boundary (`CadKernel`) so `chip-cam` and `chip-gui` never depend directly on `truck` data structures. This keeps `cargo build` 100% pure-Rust in v1 while permitting an optional OpenCASCADE (`opencascade-rs`) backend behind a Cargo feature flag in future versions if needed.

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
   - CPU-side BVH / R-tree acceleration structures are built over face triangles and edge segments for sub-millisecond hover and click raycasting in the 3D viewport.

---

## Interactive 3D Selection Modes
All 2.5D selections operate relative to the active **Setup WCS** (`+Z` = spindle axis pointing up from part toward tool):

| Selection Mode | User Action in 3D Viewport | Extracted 2.5D CAM Geometry |
|---|---|---|
| **Planar Face** | Click a horizontal face (`normal ∥ ±Z`) | Outer boundary `ArcPolyline2D` + inner island/hole loops; `Z` height of the face |
| **Closed Edge Loop** | Click an edge with "Loop Chain" active (or double-click edge) | Tangent/connected closed `ArcPolyline2D` projected onto Setup XY; `Z` height from loop vertices |
| **Open Edge Chain** | Click start edge, then end edge (shortest/tangent path) | Open `ArcPolyline2D` for open contours, open-edge pockets, chamfers, or trace engraving |
| **Cylindrical Hole** | Click an internal cylindrical face (`axis ∥ ±Z`) | Hole center `(X, Y)`, exact diameter `2R`, top `Z`, bottom `Z`, and through-hole vs blind-hole flag |
| **Same-Diameter / Coplanar Assist** | Modifier button ("Select Same Diameter" / "Select Coplanar Faces") | Expands current selection to matching holes or pocket floors on the same body |

### Automatic Z-Height Inference
Every operation maintains explicit **Top Height** and **Bottom Height** references, defaulting to values inferred from the B-Rep selection:
- **Pocket on Planar Floor Face:** Bottom = selected face `Z`; Top = highest adjacent wall top `Z` (or Setup Stock Top if open to top).
- **Contour on Bottom Edge Loop:** Bottom = loop `Z`; Top = Setup Model Top / Stock Top.
- **Cylindrical Hole:** Top = cylinder upper rim `Z`; Bottom = cylinder lower rim `Z` (plus optional drill tip breakthrough compensation for through-holes).
- Every inferred height can be overridden by switching its reference mode (`From Selection`, `Stock Top`, `Stock Bottom`, `Model Top`, `Model Bottom`, `WCS Origin`) plus a signed offset expression.

---

## CAD Reload & Geometric Signature Re-Matching
STEP exports from parametric CAD tools do not guarantee stable entity numbering across edits. To prevent losing CAM work when a part is revised:

### Stored Selection Binding
Each geometry reference in `.chip.toml` stores both the last-known topology index and a **GeometricSignature** computed in Setup-local coordinates (or body-local coordinates for Setup WCS references):
- `entity_kind`: `Face` | `Loop` | `Edge` | `CylinderHole`
- `surface_or_curve_type`: e.g., `Plane`, `InternalCylinder`, `CircleArc`
- `normal_or_axis`: unit `DVec3`
- `centroid`: `DVec3` (mm)
- `characteristic_measure`: area (`mm²`), perimeter/length (`mm`), or radius (`mm`)
- `aabb`: `[min_xyz, max_xyz]`

### Re-Matching Algorithm on "Reload STEP Model"
1. Load and normalize the updated STEP file.
2. First re-bind the Setup's WCS alignment entities (if referenced to B-Rep geometry).
3. Score candidate entities of matching `entity_kind` and `surface_or_curve_type` using a weighted distance over orientation (`cos θ > 0.999`), radius/area delta, and centroid shift:
   - **Exact / High-Confidence Match** (e.g., identical normal, same or slightly adjusted dimension/position with unambiguous winner): Automatically re-bind to the new `FaceId`/`LoopId`/`EdgeId` and mark dependent operations **Dirty**.
   - **Ambiguous or Missing Match** (e.g., pocket deleted, split into two faces, or tilted off-axis): Mark the operation **Invalid Selection (Needs Re-pick)** with a warning badge in the CAM tree while preserving all feeds, speeds, tool, and linking parameters for 1-click re-binding.
