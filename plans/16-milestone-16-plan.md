# ChipCutter — Milestone 16 Implementation Plan

## Goal Description
Implement **Milestone 16 (Triangle Mesh Import (`STL` / `OBJ` / `3MF`), BVH Surface Kernel, & Schema v3)** from [10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md#L108-L111)—establishing the Phase 5 foundation for 3-axis surface milling and mesh-based CAM workflows:

1. **Pure-Rust Triangle Mesh Importers & `MeshBvh` (`crates/chip-cad`):**
   - Implement permissively licensed, zero-new-dependency importers for **binary & ASCII STL**, **Wavefront OBJ**, and **3MF** (ZIP + XML using existing workspace `zip` crate) normalizing all vertex coordinates into canonical `f64` millimeters.
   - Add spatial vertex welding, degenerate-triangle filtering, manifold / open-boundary / winding diagnostics (`MeshDiagnostics`), and a fast `f64` R-tree/BVH spatial index (`MeshBvh`) integrated cleanly alongside STEP B-Rep solids on `CadDocument`.
   - Provide deterministic sculpted 3D mesh fixtures (hemispherical dome + fillet pedestal and sinusoidal relief wave) for analytical kernel and visual verification.
2. **Exact Analytical `f64` Drop-Cutter & Push-Cutter Kernel (`crates/chip-cam`):**
   - Implement exact closed-form `f64` **Drop-Cutter** (`drop_cutter_z(x, y)`) and **Push-Cutter** (`push_cutter_slice_z(z)`) surface contact solvers in `crates/chip-cam/src/surface3d/` operating over `SurfaceMesh3D` (built from either imported `TriangleMeshBody` meshes or fine-evaluated `f64` STEP B-Rep surfaces).
   - Support **Flat Endmill**, **Ball Endmill**, **Bull-Nose / Toroidal Endmill**, and **Tapered / Chamfer / V-Bit** cutters with radial/axial stock-to-leave allowances and 2D containment boundary clipping (`RegionSet2D`).
3. **Schema v3, `.chipz` Mesh Bundling, CLI, & 3D Viewport Integration (`chip-project`, `chip-cli`, `chip-gui`):**
   - Advance `.chip.toml` cleanly to **`schema_version = 3`** (lossless auto-migration of `schema_version = 1` and `2`), extending `ModelEntry` with optional mesh format/unit metadata (`unit_override`) and preserving full `.chipz` `models/*` SHA-256 bundling.
   - Extend `chipcutter inspect` for `.stl`/`.obj`/`.3mf` files and add `chipcutter import-mesh <FILE> -o <PROJECT.chip.toml> [--units auto|mm|in|cm]`.
   - Add **Import 3D Mesh (STL / OBJ / 3MF)…** (with a unit-preview modal for unitless STL/OBJ files preventing `25.4×` scale mistakes), `wgpu` shaded mesh rendering, Setup stock/WCS 27-point alignment on meshes, and `--demo-mesh-dome` / Sample Gallery preview support in `chip-gui`.

---

## Key Architectural Decisions & Scope Guardrails

> [!IMPORTANT]
> **Zero New External Crate Dependencies & Strict `f64` Separation:**
> - `zip = "9"`, `rstar = "0.12"`, `glam = "0.29"`, `rayon = "1.10"`, `serde`, and `thiserror` are already workspace dependencies in root [Cargo.toml](file:///workspace/ChipCutter/Cargo.toml#L38-L55). Linking `zip.workspace = true` in `crates/chip-cad/Cargo.toml` adds **zero new third-party crates** (`cargo deny check` remains unchanged, `#![deny(unsafe_code)]`, no `unwrap()`/`expect()`).
> - **Viewport vs. CAM Geometry Invariant (`AGENTS.md` §2.3):** `TriangleMeshBody` stores canonical `f64` vertices/triangles for `MeshBvh`, Setup stock bounds, and Drop-Cutter/Push-Cutter queries; `f32` `RenderMesh` buffers are derived strictly for `wgpu` rendering and CPU ray-picking.
> - **Backward Compatibility:** Every existing `schema_version = 1` and `schema_version = 2` `.chip.toml`/`.chipz` project loads losslessly, and all Phase 1 + Phase 2 golden G-code snapshots remain byte-for-byte identical.

> [!NOTE]
> **Unified `CadDocument` Representation for STEP & Meshes:**
> - Extending `CadDocument` with `pub meshes: Vec<TriangleMeshBody>` (defaulting to empty on STEP/Pure-2D loads) allows `aabb_mm`, `render_mesh`, `CadPicker`, `compute_tight_oriented_model_aabb`, Setup Stock (`RelativeBox` / `RelativeCylinder`), 27-point WCS origin dots, `.chipz` bundling, and `chip-gui` background loading to support STEP solids AND triangle meshes seamlessly without duplicating viewport pipelines.

> [!TIP]
> **3MF Archive Safety Hardening:**
> - 3MF files are OPC ZIP containers; the 3MF loader enforces the exact same zip-slip path validation and maximum uncompressed byte ceiling (`MAX_MESH_UNPACK_BYTES = 128 MiB`) used in `chip-project::bundle`, rejecting malformed or hostile archives with structured `CadError` diagnostics rather than panicking.

---

## Proposed Changes

### 1. Pure-Rust Mesh Importers (`STL` / `OBJ` / `3MF`), Diagnostics, & `MeshBvh` (`crates/chip-cad`)

#### [NEW] `crates/chip-cad/src/mesh/` (`mod.rs`, `stl.rs`, `obj.rs`, `threemf.rs`, `bvh.rs`, `fixtures.rs`) & [MODIFY] [Cargo.toml](file:///workspace/ChipCutter/crates/chip-cad/Cargo.toml), [model.rs](file:///workspace/ChipCutter/crates/chip-cad/src/model.rs), [lib.rs](file:///workspace/ChipCutter/crates/chip-cad/src/lib.rs)
- **Data Model (`mesh/mod.rs`):**
  - `MeshFormat` (`StlBinary`, `StlAscii`, `Obj`, `ThreeMf`), `MeshUnitKind` (`Millimeter`, `Centimeter`, `Meter`, `Inch`, `Foot`, `Micron`).
  - `MeshImportOptions { unit_override: Option<MeshUnitKind>, weld_tolerance_mm: f64 }` (default `weld_tolerance_mm = 1e-4`).
  - `MeshTriangle { indices: [u32; 3], normal: DVec3, aabb: Aabb3 }`.
  - `MeshDiagnostics { raw_triangle_count, valid_triangle_count, degenerate_triangles_removed, welded_vertex_count, open_boundary_edges, non_manifold_edges, is_watertight, signed_volume_mm3 }`.
  - `TriangleMeshBody { id: BodyId, name: String, format: MeshFormat, detected_unit: Option<MeshUnitKind>, applied_unit: MeshUnitKind, vertices_mm: Vec<DVec3>, triangles: Vec<MeshTriangle>, aabb_mm: Aabb3, diagnostics: MeshDiagnostics }`.
- **Importers:**
  - **`stl.rs`:** Disambiguates binary vs. ASCII STL reliably (checking `bytes.len() == 84 + 50 * tri_count` before trusting a leading `"solid"` header!), parses binary little-endian float records and ASCII `facet normal` / `vertex` blocks, rejects `NaN`/`Inf`.
  - **`obj.rs`:** Parses `v`, `f` (`v`, `v/vt`, `v//vn`, `v/vt/vn`, negative relative indices), fan-triangulates convex quads/n-gons, tracks `o`/`g` names, ignores `vt`/`vn`/`mtllib` cleanly, detects optional `# units: mm|in` metadata comments.
  - **`threemf.rs`:** Reads OPC ZIP archive with size/path guards, resolves `_rels/.rels` → `3D/3dmodel.model`, parses `<model unit="...">`, `<resources>` `<object>` `<mesh>` (`<vertices>`, `<triangles>`), `<components>` with 3×4 affine transforms, and `<build>` `<item>` instances into flattened `f64` millimeter triangles.
- **Vertex Welding, Winding Diagnostics, & `MeshBvh` (`mesh/bvh.rs`):**
  - Quantized spatial-grid vertex welding (`weld_tolerance_mm`), degenerate-area filtering (`area < 1e-10 mm²`), half-edge counting for open-boundary vs. non-manifold diagnostics, signed volume computation, and `rstar::RTree` spatial acceleration (`MeshBvh`):
    - `query_xy_circle(cx, cy, radius)` → candidate triangle indices for Drop-Cutter,
    - `query_z_slab(z_min, z_max)` → candidate triangle indices for Push-Cutter / Waterline slicing,
    - `raycast(ray)` → exact Möller–Trumbore `f64` hit for viewport picking.
- **Unified `CadDocument` Integration (`model.rs`, `picking.rs`):**
  - Add `pub meshes: Vec<TriangleMeshBody>` to `CadDocument` + helper `CadDocument::from_mesh(mesh: TriangleMeshBody)` and `CadDocument::has_3d_geometry()`.
  - Populate `RenderMesh` for imported meshes with crisp crease-angle normals (flat shading above `35°` crease angle, smooth vertex normals below `35°`) so sculpted domes look smooth while faceted blocks show sharp edges!
- **Programmatic 3D Mesh Fixtures (`mesh/fixtures.rs`):**
  - `sample_sculpted_dome_stl_bytes()` — Exact analytical `R = 25 mm` hemispherical dome seated on a `70 × 70 × 8 mm` base pedestal (binary STL bytes).
  - `sample_relief_wave_obj_text()` — Smooth sinusoidal wave relief (`80 × 60 × 15 mm`, Wavefront OBJ text).
  - `sample_box_3mf_bytes(unit)` — Valid minimal 3MF ZIP archive (`unit="inch"` and `unit="millimeter"`) verifying unit normalization and 3×4 `<item transform="...">` handling.

---

### 2. Analytical `f64` Drop-Cutter & Push-Cutter Kernel (`crates/chip-cam`)

#### [NEW] `crates/chip-cam/src/surface3d/` (`mod.rs`, `cutter.rs`, `drop_cutter.rs`, `push_cutter.rs`) & [MODIFY] [stock.rs](file:///workspace/ChipCutter/crates/chip-cam/src/stock.rs), [lib.rs](file:///workspace/ChipCutter/crates/chip-cam/src/lib.rs)
- **`SurfaceMesh3D` (`surface3d/mod.rs`):**
  - Transforms `TriangleMeshBody` (or fine `f64` tessellation of a STEP `CadDocument`) into Setup WCS `f64` coordinates via `RigidFrame3::world_to_local_point` and builds a WCS-space `MeshBvh`.
  - Update `compute_tight_oriented_model_aabb` in [stock.rs](file:///workspace/ChipCutter/crates/chip-cam/src/stock.rs) to include `doc.meshes` vertices alongside B-Rep vertices so Setup stock sizing and 27-point WCS origin picking work identically on meshes.
- **`SurfaceCutterProfile` (`surface3d/cutter.rs`):**
  - Maps `chip_tools::CutterGeometry` + `(stock_radial_mm, stock_axial_mm)` into an exact analytical contact envelope:
    - `Flat { radius_mm }`
    - `Ball { radius_mm }`
    - `BullNose { flat_radius_mm, corner_radius_mm }`
    - `TaperedVBit { tip_radius_mm, upper_radius_mm, half_angle_rad, flute_length_mm }`
- **Exact Analytical Drop-Cutter (`surface3d/drop_cutter.rs`):**
  - `drop_cutter_z(surface: &SurfaceMesh3D, cutter: &SurfaceCutterProfile, x: f64, y: f64, z_floor: f64) -> Option<f64>`:
    - Queries `MeshBvh::query_xy_circle(x, y, cutter.max_radius_mm())`.
    - For each candidate triangle, evaluates exact closed-form `f64` contact heights against:
      1. **Triangle facet interior** (projecting the cutter contact point onto the triangle plane and checking barycentric containment),
      2. **3 triangle edges** (analytic cylinder/sphere/torus/cone edge-contact solver — closed-form quadratic for Flat/Ball/V-Bit, bracketed Brent/Newton refinement to `1e-9 mm` for Bull-Nose torus),
      3. **3 triangle vertices** (exact radial profile height `vz - profile_drop(d)`).
    - Returns `max(z_contact) + stock_axial_mm`, guarded against `NaN`/`Inf` and degenerate vertical facets.
  - Add batch parallel evaluation helper `drop_cutter_grid_parallel` (`rayon`) and 2D containment boundary filter (`RegionSet2D`) for M17/M18 strategies.
- **Analytical Push-Cutter / Z-Slice Solver (`surface3d/push_cutter.rs`):**
  - `slice_surface_contours_z(surface: &SurfaceMesh3D, z_wcs: f64) -> Vec<ArcPolyline2D>`: exact plane-triangle segment extraction + vertex-welded loop chaining at height `z_wcs`.
  - `push_cutter_region_z(surface: &SurfaceMesh3D, cutter: &SurfaceCutterProfile, z_wcs: f64) -> RegionSet2D`: evaluates the cutter-offset horizontal avoidance region at height `z_wcs` across all triangles intersecting `[z_wcs, z_wcs + cutter.active_contact_height()]` (accounting for Flat/Ball/Bull-Nose/V-Bit radial profile at each triangle's Z height) and unions via `cavalier_contours` (`RegionSet2D`)—directly powering Milestone 17's `Waterline3D` and `Adaptive3D`!

---

### 3. Schema v3, `.chipz` Mesh Bundling, & Headless CLI (`chip-project`, `chip-cli`)

#### [MODIFY] [document.rs](file:///workspace/ChipCutter/crates/chip-project/src/document.rs), [toml_io.rs](file:///workspace/ChipCutter/crates/chip-project/src/toml_io.rs), [validation.rs](file:///workspace/ChipCutter/crates/chip-project/src/validation.rs), [bundle.rs](file:///workspace/ChipCutter/crates/chip-project/src/bundle.rs), & [lib.rs](file:///workspace/ChipCutter/crates/chip-project/src/lib.rs)
- Advance `ProjectDocument::CURRENT_SCHEMA_VERSION = 3` while keeping `from_toml_str` migrating `1 -> 3` and `2 -> 3` losslessly.
- Extend `ModelEntry` with backward-compatible optional fields (`#[serde(default, skip_serializing_if = "Option::is_none")]`):
  - `pub unit_override: Option<MeshUnitKind>` (omitted for STEP and unit-aware 3MF files so existing `.chip.toml` diffs stay clean!).
- Add `is_mesh_path(path: &Path) -> bool` (`.stl`, `.obj`, `.3mf`) and unified `load_model_entry_cad_doc(path, unit_override)` helper shared by `chip-cli` and `chip-gui` (dispatching STEP to `MonstertruckKernel` and STL/OBJ/3MF to `chip_cad::load_mesh_bytes`).
- Verify `bundle.rs` packs `.stl`/`.obj`/`.3mf` entries cleanly under `models/*` with SHA-256 verification and zip-slip protection.

#### [MODIFY] [main.rs](file:///workspace/ChipCutter/crates/chip-cli/src/main.rs) & [inspect_sample.rs](file:///workspace/ChipCutter/crates/chip-cli/src/commands/inspect_sample.rs)
- Extend `chipcutter inspect <PATH>` (`--units mm|in`, `--mesh-units auto|mm|in|cm`) to detect `.stl`/`.obj`/`.3mf` and print format, unit, triangle & welded vertex counts, bounding box (`X × Y × Z`), open-boundary/non-manifold edge counts, watertight status, and signed volume.
- Add `chipcutter import-mesh <INPUT> -o <OUTPUT.chip.toml> [--mesh-units auto|mm|in|cm]` creating a ready-to-use `schema_version = 3` project with Setup stock & `G54` WCS initialized around the mesh bounds.

---

### 4. Desktop GUI Mesh Import Modal, Viewport Rendering, & Sample Preview (`crates/chip-gui`)

#### [NEW] `crates/chip-gui/src/panels/mesh_import_modal.rs` & [MODIFY] [panels/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/mod.rs), [app/loader.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/loader.rs), [app/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/mod.rs), [app/menus_toolbar.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/menus_toolbar.rs), [panels/browser.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/browser.rs), [panels/welcome_gallery.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/welcome_gallery.rs), & [panels/command_palette.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/command_palette.rs)
- **Mesh Import Unit Modal (`mesh_import_modal.rs`):**
  - Opens when importing a unitless `.stl` or `.obj` (or via `File → Import 3D Mesh (STL / OBJ / 3MF)…`), showing a live physical bounding-box preview (`X × Y × Z` in display units), triangle/vertex count, watertight/manifold badge, and `mm` / `in` / `cm` / `m` unit selector so unitless meshes are never imported at `25.4×` wrong scale! (`.3mf` files show their declared unit pre-selected.)
- **Background Loading & Viewport Integration (`app/loader.rs`, `browser.rs`):**
  - Background `rayon` loading of `.stl`/`.obj`/`.3mf` into `LoadedModelState` (reusing `solid_pipeline` and `CadPicker` seamlessly).
  - Update `Models` panel in `browser.rs` to show `Import Mesh…` alongside `Import STEP…` and display mesh summary (`N triangles · M vertices · Watertight` / `Open edges`).
  - Wire **Import 3D Mesh (STL / OBJ / 3MF)…** into the **Welcome Overlay**, **`File` menu**, **`Ctrl+K` Command Palette**, and positional CLI path auto-detection (`chip-gui part.stl`).
  - Add **`Sculpted Dome (STL Mesh)`** (`SampleProjectId::SculptedMeshDome`, CLI `--sample mesh-dome`) to the Sample Projects Gallery for instant visual verification & user exploration!

---

## Verification Plan

1. **Automated Unit Tests (`chip-cad`, `chip-cam`, `chip-project`):**
   - `chip-cad::mesh::tests`:
     - Binary STL (including binary files starting with `"solid"` header trap) & ASCII STL parsing, OBJ (quads/n-gons/negative indices/groups), and 3MF (ZIP + XML + `unit="inch"` + 3×4 `<item transform="...">` + zip-slip rejection).
     - Vertex welding, degenerate triangle removal, watertight vs. open-edge/non-manifold diagnostics, and `MeshBvh` XY-circle, Z-slab, & raycast accuracy.
   - `chip-cam::surface3d::tests`:
     - **Analytical Ground-Truth Drop-Cutter Oracle:** Verify `drop_cutter_z` on the exact analytical `R = 25 mm` hemispherical dome + flat pedestal against closed-form formulas for **Flat**, **Ball** (`z_center = sqrt((R_dome + R_ball)² - r²) - R_ball`), **Bull-Nose**, and **60° V-Bit** cutters across interior, edge, vertex, and radial/axial stock-to-leave cases (`|error| < 0.05 mm` due solely to mesh chord tessellation, and `< 1e-6 mm` on exact planar/vertical edge/vertex test triangles!).
     - **Push-Cutter / Waterline Slice Oracle:** Verify `slice_surface_contours_z` and `push_cutter_region_z` on the `R = 25 mm` dome at multiple Z levels match exact expected circle radii (`r(z) = sqrt(R² - z²) + R_cutter_effective`) to `< 0.05 mm`.
   - `chip-project::tests`:
     - Lossless `schema_version = 1 -> 3` and `2 -> 3` TOML migration, deterministic serialization of `ModelEntry` (`unit_override` omitted for STEP), `.chipz` round-trip packing/unpacking of `.stl`/`.obj`/`.3mf` meshes with SHA-256 verification.
2. **CLI Integration & Regression Gates (`chip-cli`, Workspace):**
   - Add CLI end-to-end test in `crates/chip-cli/tests/cli_end_to_end.rs` covering `chipcutter inspect` on `.stl`/`.obj`/`.3mf`, `chipcutter import-mesh`, and `chipcutter bundle pack` / `unpack` on a mesh project.
   - Run full gates:
     - `cargo fmt --all -- --check`
     - `cargo clippy --workspace --all-targets -- -D warnings`
     - `cargo deny check` (`0` new third-party dependencies)
     - `WGPU_BACKEND=vulkan cargo test --workspace` (all existing unit, heightmap oracle, offscreen `wgpu`, and Phase 1 + Phase 2 golden G-code snapshots passing byte-for-byte).
3. **Headless Vulkan Visual Verification (`xvfb-run` + Lavapipe):**
   - Capture and visually inspect headless screenshots via `read_file` for:
     - `Sculpted Dome (STL Mesh)` (`--sample mesh-dome --self-test-screenshot`) verifying smooth studio shading, stock wireframe, 27-point WCS origin dots, and `Models` mesh summary in Studio Dark & Daylight Light themes.
