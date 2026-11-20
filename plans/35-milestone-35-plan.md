# Milestone 35 Implementation Plan — Non-Destructive `ModelTransform` Kernel (Rotate, Scale, Move), Persistence, & CLI

## Goal

Add a UI-agnostic, exact `f64` affine **`ModelTransform`** (3D scaling + rotation + translation) to `chip-cad` and `ModelEntry` in `chip-project` so both **STEP B-Rep solids** and **STL / OBJ / 3MF triangle meshes** can be scaled (uniformly or per-axis), rotated, laid flat on any face/triangle normal, centered, and translated non-destructively after import—while maintaining 100% backward compatibility with existing `.chip.toml` files, associative `Ctrl+R` STEP reload, `.chipz` bundles, and golden G-code snapshots.

---

## Proposed Changes

### 1. Exact `f64` Affine Transform Primitives & Geometry Application (`chip-cad`)

#### [NEW] `crates/chip-cad/src/transform.rs`
#### [MODIFY] [crates/chip-cad/src/lib.rs](file:///workspace/ChipCutter/crates/chip-cad/src/lib.rs)
- Define `ModelTransform` (`Serialize, Deserialize, Clone, Copy, Debug, PartialEq`):
  - `#[serde(default)] pub translate_mm: [f64; 3]`
  - `#[serde(default)] pub rotate_deg: [f64; 3]` (documented XYZ Euler convention: `R = Rz(rz) * Ry(ry) * Rx(rx)`)
  - `#[serde(default = "default_unit_scale", skip_serializing_if = "is_unit_scale")] pub scale: [f64; 3]` (default `[1.0, 1.0, 1.0]`, strictly positive finite factors validated in `[1e-4, 1e4]`).
  - Canonical TRS composition: **Scale → Rotate → Translate**, i.e., `p' = R * (S * p) + t`, and normal transformation via normalized inverse-transpose `n' = normalize(R * (S^-1 * n))`.
  - Helpers:
    - `ModelTransform::IDENTITY`, `is_identity(&self) -> bool`, `is_uniform_scale(&self) -> Option<f64>`, `validate(&self) -> Result<(), CadError>`.
    - `to_rotation_matrix(&self) -> DMat3`, `to_linear_matrix(&self) -> DMat3` (`R * S`), `to_affine_matrix(&self) -> DMat4`, `transform_point(&self, p: DVec3) -> DVec3`, `transform_normal(&self, n: DVec3) -> DVec3`.
    - `compose_post(&self, delta_rot: DMat3, delta_translate: DVec3) -> Self` (extracting canonical Euler angles cleanly so repeated `±90° X/Y/Z` clicks and `Lay Flat on Face` compose intuitively in world space!).
    - Preset constructors/mutators:
      - `y_up_to_z_up()` (`rotate_deg: [90.0, 0.0, 0.0]`)
      - `rotate_world_axis_deg(&self, axis: DVec3, angle_deg: f64, pivot_mm: DVec3) -> Self` (rotates around the model's bounding-box center or origin without flying off-screen!)
      - `lay_flat_normal_to_neg_z(&self, world_face_normal: DVec3) -> Self` (shortest-arc Rodrigues rotation mapping the clicked face/triangle normal to `-Z` so that face sits flat on the CNC table/spoilboard!)
      - `with_uniform_scale(&self, factor: f64) -> Self` and `scale_to_fit_axis(&self, current_aabb: Aabb3, axis: usize, target_mm: f64) -> Self` (computes exact uniform scale factor from current transformed span so a machinist can resize any part to exact stock dimensions, e.g., `X = 150 mm`!).
      - `center_xy_for_aabb(&self, transformed_aabb: Aabb3) -> Self`
      - `drop_bottom_to_z0_for_aabb(&self, transformed_aabb: Aabb3) -> Self`
      - `align_top_to_z0_for_aabb(&self, transformed_aabb: Aabb3) -> Self`
- Implement `CadDocument::transformed(&self, xf: &ModelTransform) -> CadDocument`:
  - Returns `self.clone()` unchanged when `xf.is_identity()`.
  - **STEP `SolidBody` handling:**
    - **Uniform scale** (`xf.is_uniform_scale() == Some(s)`): Preserves **all** exact `f64` B-Rep analytic types (`Plane`, `Cylinder` with `radius *= s`, `Cone` with `radius *= s`, `Line`, `Circle`/`Arc` with `radius *= s`, `BSpline`, loop polygons, bounding boxes, vertices, and `RenderMesh`) with zero approximation!
    - **Non-uniform scale** (`sx != sy` or `sy != sz`): Preserves exact `Plane` normals (`normalize(R * S^-1 * n)`), exact `Line` endpoints, exact transformed `BSpline`/loop polylines, and `RenderMesh`; gracefully converts any circular `Circle`/`Arc` or `Cylinder`/`Cone` whose cross-section axes are stretched anisotropically into exact scaled `BSpline` / `Freeform` geometry (keeping 3D surface milling and 2D contour/pocket extraction valid without misreporting an elliptical bore as circular).
  - **Mesh `TriangleMeshBody` handling (`STL`/`OBJ`/`3MF`):**
    - Transforms every vertex `v' = xf.transform_point(v)` (exact for both uniform and non-uniform scaling!), recomputes exact geometric triangle normals from transformed vertices, updates `aabb_mm` and `signed_volume_mm3`, and rebuilds `RenderMesh` so `MeshBvh`, 3D Drop-Cutter/Push-Cutter, and ray picking operate in exact transformed coordinates.
  - Recomputes `doc.aabb_mm` deterministically.

---

### 2. Project Schema, Associative Reload, & `.chipz` Integration (`chip-project`)

#### [MODIFY] [crates/chip-project/src/document.rs](file:///workspace/ChipCutter/crates/chip-project/src/document.rs)
- Add `#[serde(default, skip_serializing_if = "ModelTransform::is_identity")] pub transform: ModelTransform` on `ModelEntry`.
- Update `load_model_entry_cad_doc_transformed(path, unit_override, &entry.transform)` (and `load_model_bytes_cad_doc_transformed`) so both CLI and GUI apply `entry.transform` identically immediately after loading raw STEP/STL/OBJ/3MF bytes.
- Keep `ModelEntry::sha256` computed over the **raw unmodified file bytes** on disk so SHA-256 verification and associative reload work seamlessly regardless of transform edits.

#### [MODIFY] [crates/chip-project/src/reload.rs](file:///workspace/ChipCutter/crates/chip-project/src/reload.rs)
- Verify `rebind_project_to_reloaded_cad` preserves `project.models[model_idx].transform` unchanged while re-matching against the newly loaded + transformed `CadDocument` and `TopologyGraph`.

#### [MODIFY] [crates/chip-project/src/bundle.rs](file:///workspace/ChipCutter/crates/chip-project/src/bundle.rs)
- Verify `.chipz` pack/unpack preserves `ModelEntry.transform` inside `project.chip.toml` alongside raw untouched `models/*` bytes.

---

### 3. Headless CLI Commands (`chip-cli`)

#### [MODIFY] [crates/chip-cli/src/main.rs](file:///workspace/ChipCutter/crates/chip-cli/src/main.rs)
#### [MODIFY] [crates/chip-cli/src/commands/mod.rs](file:///workspace/ChipCutter/crates/chip-cli/src/commands/mod.rs)
#### [MODIFY] [crates/chip-cli/src/commands/inspect_sample.rs](file:///workspace/ChipCutter/crates/chip-cli/src/commands/inspect_sample.rs)
- Update `load_primary_cad_for_project` to apply `first.transform` automatically so `validate`, `simulate`, and `build` honor saved model transforms identically to GUI.
- Add `chipcutter transform-model <project.chip.toml>`:
  - Flags: `--rot-x <deg>`, `--rot-y <deg>`, `--rot-z <deg>`, `--scale <factor>`, `--scale-x <f>`, `--scale-y <f>`, `--scale-z <f>`, `--fit-size-x <mm>`, `--fit-size-y <mm>`, `--fit-size-z <mm>`, `--move-x <mm>`, `--move-y <mm>`, `--move-z <mm>`, `--y-up-to-z-up`, `--center-xy`, `--drop-z0`, `--align-top-z0`, `--reset`, `--output <out.chip.toml>`.
  - Prints before/after bounding box (`X × Y × Z`, `Min/Max Z`) and marks dependent operations `Dirty`.
- Add optional `--rot-x`, `--rot-y`, `--rot-z`, `--scale`, `--y-up-to-z-up`, `--center-xy`, `--drop-z0` flags to `chipcutter import-mesh`.

---

## Verification Plan

1. **Unit & Geometry Accuracy Tests (`chip-cad`):**
   - `ModelTransform` identity omission in TOML (including default `[1.0, 1.0, 1.0]` scale), validation rejecting `<= 0` / `NaN` scale factors, Euler compose roundtrip, `scale_to_fit_axis`, and `lay_flat_normal_to_neg_z` accuracy (`< 1e-9` residual dot product with `-Z` across cardinal, diagonal, and antiparallel `+Z` normals).
   - Uniformly scaling + rotating a STEP B-Rep solid (with planar faces, cylindrical holes, and circular arcs) scales exact `f64` cylinder diameters, arc radii, hole centers, and AABB dimensions to `< 1e-9 mm`; non-uniform scaling transforms planar faces/polylines cleanly and degrades stretched circular features safely without bogus circular radii.
   - Transforming (scaling + rotating + translating) an STL/OBJ/3MF `TriangleMeshBody` preserves watertightness, exact scaled signed volume (`V' = V * sx * sy * sz`), and exact `MeshBvh` raycast Z-heights.
2. **Project, Reload, & CLI End-to-End Tests (`chip-project`, `chip-cli`):**
   - Existing `.chip.toml` files without `transform` deserialize cleanly as `ModelTransform::IDENTITY` and re-serialize without emitting `[models.transform]`.
   - Transforming a Y-up STL (`--y-up-to-z-up --center-xy --drop-z0`) -> `validate --generate` -> `simulate` -> `build` produces crash-free G-code with `0` errors.
   - Associative `reload-step` preserves `ModelEntry.transform` and re-matches features cleanly.
   - All 15 existing byte-for-byte golden G-code snapshots remain unchanged.
