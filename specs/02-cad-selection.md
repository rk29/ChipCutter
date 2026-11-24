# 02 — CAD Import, B-Rep & Mesh Pipelines, & Unified 3D/2D Selection

## CAD Kernel Abstraction (`chip-cad`)
All interaction with STEP files and B-Rep topology goes through an internal trait boundary (`CadKernel`) so `chip-cam` and `chip-gui` never depend directly on `monstertruck` data structures. This keeps `cargo build` self-contained while permitting an optional OpenCASCADE (`opencascade-rs`) backend behind a Cargo feature flag in future versions if needed.

### Core Domain Types (`crates/chip-cad/src/model.rs`)
`CadDocument` stores B-Rep topology and imported triangle meshes in flat indexed arenas alongside the combined viewport `render_mesh`:

```text
CadDocument
 ├── bodies: Vec<SolidBody>          (BodyId)
 │    └── { id, name, faces: Vec<FaceId>, edges: Vec<EdgeId>, aabb_mm }
 ├── meshes: Vec<TriangleMeshBody>   (BodyId; imported STL / OBJ / 3MF)
 │    └── { id, name, format, detected_unit, applied_unit, vertices_mm, triangles, aabb_mm, diagnostics }
 ├── faces: Vec<BRepFace>            (FaceId)
 │    ├── surface: SurfaceKind
 │    │    ├── Plane { origin_mm, normal }
 │    │    ├── Cylinder { axis_origin_mm, axis_dir, radius_mm, is_internal }
 │    │    ├── Cone { apex_mm, axis_dir, half_angle_rad }
 │    │    ├── Torus { center_mm, axis_dir, major_radius_mm, minor_radius_mm }
 │    │    └── OtherNurbs
 │    ├── outer_loop: Option<LoopId>
 │    ├── inner_loops: Vec<LoopId>
 │    ├── area_mm2, aabb_mm
 │    └── signature: GeometricSignature
 ├── loops: Vec<BRepLoop>            (LoopId)
 │    ├── face_id, is_outer
 │    ├── edges: Vec<OrientedEdgeUse>  ({ edge_id: EdgeId, forward: bool })
 │    └── signature: GeometricSignature
 ├── edges: Vec<BRepEdge>            (EdgeId)
 │    ├── curve: CurveKind
 │    │    ├── Line { start_mm, end_mm }
 │    │    ├── CircleArc { center_mm, normal, radius_mm, start_mm, end_mm, is_full_circle }
 │    │    ├── EllipseArc
 │    │    └── OtherNurbs
 │    ├── length_mm, aabb_mm
 │    ├── adjacent_faces: Vec<FaceId>
 │    ├── loops: Vec<LoopId>
 │    └── signature: GeometricSignature
 └── render_mesh: RenderMesh         (vertices, indices, pick_triangles, edge_polylines)
```

---

## STEP Import Pipeline (`crates/chip-cad/src/monstertruck_kernel.rs`)
1. **Supported Schemas:** ISO 10303-21 STEP files (`.step`, `.stp`) covering AP203, AP214, and AP242 solid and shell representations via `monstertruck-io` (`step` feature). STEP shell entity IDs are sorted before conversion so `BodyId`/`FaceId`/`LoopId`/`EdgeId` indices are deterministic.
2. **Unit Normalization:** Read the STEP file's `LENGTH_UNIT` (`MILLIMETRE`, `CENTIMETRE`, `METRE`, `INCH`, or custom scale) into `StepUnitKind` and scale all imported geometry into canonical `f64` millimeters on load.
3. **Analytic Surface & Curve Classification:**
   - Even when a CAD exporter encodes planar faces, cylindrical faces, or circular edges as rational B-splines, `chip-cad` runs analytic recognizers anchored to `ANALYTIC_CLASSIFY_TOL_MM = 1e-5 mm`:
     - **`CurveKind::Line`:** `<= 1e-5 mm` (`1× ANALYTIC_CLASSIFY_TOL_MM`) perpendicular deviation across curve samples.
     - **`CurveKind::CircleArc`:** `<= 5e-5 mm` (`5× ANALYTIC_CLASSIFY_TOL_MM`) plane and radial deviation against a 3-point 3D circumcircle fit.
     - **`SurfaceKind::Plane` & `SurfaceKind::Cylinder`:** `<= 1e-4 mm` (`10× ANALYTIC_CLASSIFY_TOL_MM`) planar distance / least-squares axis-scatter tolerance (handling `CylindricalSurface`, extruded arcs, and NURBS cylinders, and automatically classifying `is_internal` holes vs external bosses).
     - **`SurfaceKind::Cone` & `SurfaceKind::Torus`:** Matched directly from explicit STEP elementary surface tags (`ConicalSurface` / `ToroidalSurface`), not fitted from arbitrary B-splines.
4. **Viewport Tessellation & Spatial Pick Indexes (`crates/chip-cad/src/picking.rs`):**
   - Each `BRepFace` is tessellated via `monstertruck-meshing` into indexed `RenderVertex` GPU triangles tagged with `FaceId` plus exact `f64` `FaceTriangle` pick triangles.
   - Each `BRepEdge` is discretized into `EdgePolyline` segments tagged with `EdgeId`.
   - `CadPicker` builds flat SAH BVHs over exact `f64` face triangles (`TriMesh3D`) and edge segments (`SegmentBvh`) for `O(log N)` sub-millisecond hover and click raycasting (no R-tree in `CadPicker`).
   - Interactive 2D sketch picking (`pick_sketch_layer_2d` in `chip-sketch`) performs a direct scan over visible layer entities and closed regions (while `rstar` is used during DXF/SVG endpoint gap-healing in `chip-sketch`).

---

## Unified Triangle Mesh Pipeline & Non-Destructive `ModelTransform`
- **Triangle Mesh Import (`crates/chip-cad/src/mesh/`):**
  - Parses **Binary & ASCII `.stl`**, **Wavefront `.obj`**, and **`.3mf`** (OPC ZIP archives) into `CadDocument.meshes: Vec<TriangleMeshBody>` in canonical `f64` millimeters.
  - Respects file-declared units (e.g., 3MF `<model unit="...">`) or per-model `unit_override: Option<MeshUnitKind>` (`mm`, `cm`, `m`, `in`, `ft`, `µm`), spatially welds coincident vertices (`1e-4 mm` default), filters degenerate triangles, and computes `MeshDiagnostics` (`raw_triangle_count`, `valid_triangle_count`, `degenerate_triangles_removed`, `welded_vertex_count`, `open_boundary_edges`, `non_manifold_edges`, `is_watertight`, `signed_volume_mm3`).
  - Builds viewport meshes with **`35°` crease-angle normal smoothing** tagged with **`MESH_FACE_SENTINEL`** (`FaceId(u32::MAX - 1)`), exposes **`MeshBvh`** for mesh raycasting, and unifies both STEP B-Rep `pick_triangles` and `CadDocument.meshes` into Setup-WCS `TriMesh3D` solids via **`CadDocument::build_wcs_tri_mesh`** for all 3-axis and 4th-axis surface strategies and gouge checks.
- **Non-Destructive `ModelTransform` (`crates/chip-cad/src/transform.rs`):**
  - Every `ModelEntry` in `.chip.toml` supports an optional `transform: ModelTransform` (`translate_mm`, `rotate_deg`, `scale`, omitted when identity), applied in exact `f64` millimeters as `p' = Rz(rz) * Ry(ry) * Rx(rx) * (S * p) + t` without modifying the external STEP/mesh file on disk.
  - Provides shop helpers (`y_up_to_z_up`, world-axis rotation, `laid_flat_normal_to_neg_z`, uniform fit-to-axis scaling, XY centering, drop-bottom-to-Z0, align-top-to-Z0). Uniform scaling preserves all analytic B-Rep surfaces and curves exactly; non-uniform scaling preserves planes, lines, polylines, and triangle meshes while safely downgrading stretched circles/cylinders/cones/tori to `EllipseArc` / `OtherNurbs`.

---

## Unified 3D B-Rep & 2D Sketch Selection Modes
All 2.5D selections operate relative to the active **Setup WCS** (`+Z` = spindle axis pointing up from part toward tool). Both 3D STEP topology and 2D `SketchLayer` entities (see [11-sketches-vectors.md](11-sketches-vectors.md)) normalize into identical `ExtractedFeature25D` inputs for `chip-cam`:

| Selection Mode | User Action in Viewport | Extracted `ExtractedFeature25D` |
|---|---|---|
| **Planar Face** *(3D)* | Click a horizontal B-Rep face (`normal ∥ ±Z`) | `PlanarFace`: Outer `ArcPolyline2D` + open-edge segment flags + inner/raised island loops + adjacent lower floor `Z` |
| **Modeled Chamfer** *(3D)* | Click a beveled/conical chamfer face or modeled chamfer edge | `ModeledChamfer`: Upper & lower `ArcPolyline2D` contours, `radial_width_mm`, `axial_drop_mm`, `bevel_half_angle_rad`, top/bottom `Z`, outward XY normal, and adjacent lower floor `Z` |
| **Sketch Region** *(2D)* | Click inside a closed 2D Sketch/DXF/SVG/Text region | `PlanarFace`: Outer CCW `ArcPolyline2D` + enclosed CW counter/island loops at Sketch `Z` |
| **Closed Loop** *(3D or 2D)* | Click a B-Rep edge loop or closed 2D Sketch curve | `Contour` (closed): `ArcPolyline2D` projected onto Setup XY with `reverse` / `side_flip` |
| **Open Chain** *(3D or 2D)* | Click start/end B-Rep edges or open 2D Sketch/Hershey stroke | `Contour` (open): `ArcPolyline2D` for open contours, chamfers, or trace engraving |
| **Sketch Layer / Sub-Layer** *(2D)* | Select an entire `SketchLayer` or named DXF/SVG layer / `Text` item | Expands all closed regions (for V-Carve / Pocket) or strokes (for Trace) dynamically when edited |
| **Cylindrical Hole / 2D Hole Point** | Click internal B-Rep cylinder (`axis ∥ ±Z`) or 2D Sketch `Circle` / `PointPattern` | `CylindricalHole`: Center `(X, Y)`, diameter `2R`, top `Z`, bottom `Z`, through-hole flag |
| **Same-Diameter / Coplanar Assist** | Modifier button ("Select Same Diameter" / "Select Coplanar Faces") | Expands current selection to matching holes or pocket floors on the same body/sketch |

- **Native 4th-Axis Rotary Selections (`extract_rotary_selection_features`):** `Rotary2Linear` and `Rotary4AxisSim` use `extract_rotary_selection_features`, which tolerates arbitrarily oriented 3D B-Rep faces and rotary-axis cylindrical faces (projecting them directly onto the rotary axial window `-U..+U`) while still validating referenced entities and extracting any 2D sketch selections.

### Automatic Z-Height Inference & `HeightReference`
Every operation maintains explicit 5-plane Z heights (`Clearance >= Retract >= Feed >= Top >= Bottom`), defaulting to values inferred from the selection:
- **Pocket on 3D Planar Floor Face:** Bottom = selected face `Z`; Top = highest adjacent wall top `Z` (or Setup Stock Top if open to top).
- **Operation on 2D Sketch Entity / Pure-2D Job:** Top = Sketch plane `Z` (default `Stock Top`); Bottom = `Top - cut_depth_offset` (or `Stock Bottom` for profile cutouts, or automatic medial depth / `flat_depth` for V-Carve).
- **Contour on Bottom Edge Loop:** Bottom = loop `Z`; Top = Setup Model Top / Stock Top.
- **Cylindrical Hole:** Top = cylinder upper rim `Z`; Bottom = cylinder lower rim `Z` (plus optional drill tip breakthrough compensation for through-holes).
- Every height plane can be overridden by choosing any of the **9 `HeightReference` modes** (`FromSelection`, `StockTop`, `StockBottom`, `ModelTop`, `ModelBottom`, `WcsOrigin`, `SetupClearance`, `SetupRetract`, `OperationTop`) plus a signed `ParamLength` offset expression.

---

## STEP CAD Reload & Geometric Signature Re-Matching
STEP exports from parametric CAD tools do not guarantee stable entity numbering across edits. To prevent losing CAM work when a STEP part is revised:

### Stored Selection Binding
Each 3D B-Rep geometry reference (including `SketchAnchor::PlanarFace` anchors, Setup WCS refs, operation selections, and avoid-region selections) in `.chip.toml` stores a `SelectionBinding { body_id, last_index, signature: GeometricSignature }`:
- `kind: EntitySignatureKind` (`PlanarFace`, `InternalCylinderFace`, `ExternalCylinderFace`, `ConicalFace`, `OtherFace`, `LineEdge`, `CircleEdge`, `OtherEdge`, `EdgeLoop`)
- `normal_or_axis: DVec3` (unit surface normal, cylinder/cone/torus axis, or edge direction)
- `centroid_mm: DVec3` (mm; for cylinders, exact 3D axis midpoint invariant to STEP seam rotation)
- `characteristic_measure: f64` — area (`mm²`) for planar/conical/general faces, radius (`mm`) for cylinders/arcs/tori, or length/perimeter (`mm`) for linear/loop entities
- `aabb_mm: Aabb3`

*(2D `SketchLayer` entities, regions, and sub-layers are addressed by plain stable `String` IDs; geometric signature reload re-matching applies to STEP B-Rep reloads.)*

### Re-Matching Algorithm on "Reload STEP Model" (`crates/chip-project/src/reload.rs`)
1. Load and normalize the updated STEP file and update the model entry's `sha256` (and path if changed).
2. Re-bind associative `SketchAnchor::PlanarFace` sketch planes against the new B-Rep faces.
3. Re-bind all 3D B-Rep selections stored inside reusable **`ProjectDocument::avoid_regions`**.
4. For each Setup:
   - Clamp `setup.target_bodies` if the body count shrank.
   - Re-bind Setup WCS orientation (`SelectZAndX`, `SelectXYPlane`) and feature origin (`SelectedCadFeature`) references.
   - Re-bind every Operation's primary `op.selections` (including `OpenChain` edge healing via `TopologyGraph`) AND per-operation **`op.avoid.selections`**.
5. Candidate matching checks an exact-index fast path first, then filters matching `EntitySignatureKind` candidates by orientation alignment (**`cos_align >= 0.998`** for faces; **`cos_align >= 0.995`** for loops and edges) and scores weighted orientation, radius/area/perimeter delta, and centroid shift:
   - **Exact / High-Confidence Match:** Automatically re-bind to the new `FaceId`/`LoopId`/`EdgeId` and mark dependent operations **Dirty**.
   - **Ambiguous or Missing Match:** Mark the operation (or warn on face-anchored sketches / shared avoid regions / WCS refs) **Needs Re-pick** with a warning badge in the CAM tree while preserving all feeds, speeds, tool, heights, and linking parameters for 1-click re-binding.
