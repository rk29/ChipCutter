# Milestone 97 Implementation Plan — Dead Code Cleanup, Pure Geometry Consolidation, & Canonical `ToolAssemblyEnvelope`

## Goal

Eliminate dead wrappers, layer-leaking shims, duplicated pure geometry/STL/formatting routines, and the 4-way cutter/holder envelope duplication across the lower workspace crates (`chip-math`, `chip-bvh`, `chip-geom2d`, `chip-cad`, `chip-sketch`, `chip-tools`, `chip-stock`, `chip-cam`, `chip-sim`, `chip-post`, `chip-gui`)—preserving 100% lossless `.chip.toml` compatibility, zero new external dependencies, and byte-for-byte golden G-code parity.

---

## Proposed Changes

### 1. Dead Code Removal & `cavalier_contours` Encapsulation
- **Delete dead code:**
  - Delete `crates/chip-cad/src/mesh/bvh.rs` (`MeshBvh`, `MeshRayHit`) and `TriangleMeshBody::build_bvh()` (zero non-test callers; superseded by `chip_bvh::TriMesh3D`).
  - Delete dead `Ray3::intersect_triangle` in `crates/chip-math/src/geom3d.rs` (all callers use `chip_bvh::ray_triangle_intersect`).
  - Delete leftover re-export shims `crates/chip-cam/src/medial.rs` and `crates/chip-cam/src/offset2d.rs`, plus `pub use chip_sketch;` in `crates/chip-cam/src/lib.rs` (update 10 internal `chip-cam` files to import `chip_geom2d` directly, add `chip-geom2d` to `chip-gui/Cargo.toml`, and add `chip-sketch` to `chip-sim`'s `[dev-dependencies]`).
  - Delete duplicated `referencing_operations` in `crates/chip-gui/src/panels/tool_library/mod.rs` (delegate to `ProjectDocument::tool_usage_map`).
- **Encapsulate `cavalier_contours`:**
  - Add closed-form `ArcSegment2D::from_bulge(p0, p1, bulge)` in `crates/chip-math/src/geom2d.rs`.
  - Replace the 3-line `cavalier_contours` bulge call in `crates/chip-sketch/src/dxf_import.rs` with `ArcSegment2D::from_bulge`, and remove `cavalier_contours` from `crates/chip-sketch/Cargo.toml`.
  - Change `to_cavalier_polyline` and `from_cavalier_polyline` in `crates/chip-geom2d/src/offset.rs` to `pub(crate)` and drop their public re-exports from `chip-geom2d/src/lib.rs` and `chip-cam/src/lib.rs`.
  - Replace hand-rolled `Affine2D` in `crates/chip-sketch/src/dxf_import.rs` with `glam::DAffine2` (re-exported via `chip-math`).

### 2. Pure Math, Spatial, 2D Geometry, & STL Consolidation
- **`chip-math`:**
  - Add `ArcPolyline2D::circle(center, radius)` and `ArcPolyline2D::transformed(translation, scale, rotation_deg)` (delegating `chip-sketch::primitives::{build_circle_polyline, transform_arc_polyline_2d}` and `chip-geom2d` inline circle builders).
  - Add `RigidFrame3::transformed_by(&DMat4)` (replacing `chip-gui/src/app/viewport_controller.rs::transform_rigid_frame_by_dmat4`).
  - Add `circumcircle_through_three_points` (`geom2d.rs`), `circumcircle_3d` and `rotation_matrix_to_xyz_euler_deg` (`geom3d.rs`), delegating from `chip-sketch`/`chip-cad`.
  - Add canonical 2D primitives in `geom2d.rs`: `point_in_triangle_2d`, `point_to_segment_dist_2d`, `closest_point_on_segment_2d`, `segment_to_segment_dist_2d` (eliminating 2x–4x duplicates in `chip-cam`).
  - Add `trace_binary_grid_contours` in `chip-math` (eliminating the verbatim duplicate between `chip-cam/src/surface3d/slab_projection.rs` and `chip-stock/src/tridexel/transfer.rs`).
  - Add `format_cycle_time_hms` in `chip-math` (eliminating duplicate in `chip-post/src/emitter.rs` and re-exporting from `chip-sim`).
  - Move `simplify_3d_polyline_rdp` out of `chip-geom2d/src/simplify.rs` into `chip-math/src/geom3d.rs` (unclamped; keep CAM finishing tolerance clamping in `chip-cam/src/surface3d/finishing.rs`).
- **`chip-bvh`:**
  - Add `clip_triangle_to_z_slab` (eliminating 3x duplicate in `fixture_avoidance.rs`, `surface3d/slab_projection.rs`, `workholding_wizards.rs`) and `closest_point_on_triangle_at_z` (2x duplicate in `deburr3d.rs` & `undercut3d.rs`).
  - Delete `deburr3d.rs::closest_point_on_triangle_3d` and call `chip_bvh::closest_point_on_triangle`.
- **`chip-geom2d`:**
  - Add canonical `ear_clip_ccw_2d` / `triangulate_polygon_with_holes_2d` (eliminating 3x duplicates in `chip-gui/src/viewport/gizmos.rs`, `chip-cam/src/workholding.rs`, `chip-cam/src/workholding_wizards.rs`), `region_from_convex_hull_2d`, `RegionSet2D::exact_aabb`, and `marching_squares_segments` (`scallop3d.rs`).
  - Deduplicate `PlanarVectorRegion::contains_point` and Even-Odd nesting in `chip-sketch` onto `chip-geom2d`.
- **`chip-cad` & `chip-cam`:**
  - Add `pub fn encode_binary_stl` in `crates/chip-cad/src/mesh/stl.rs` (eliminating duplicate in `workholding_wizards.rs` & `chip-cad/src/mesh/fixtures.rs`).
  - Deduplicate `chip-cam/src/surface3d/finishing.rs:1399–1697` (~300 LOC) into `surface3d/drop_cutter.rs`.
  - Replace `SurfaceMesh3D`'s duplicated triangle/BVH storage with a zero-copy wrapper over `chip_bvh::TriMesh3D` (avoiding full mesh clones in `avoid_regions.rs`).

### 3. Canonical `ToolAssemblyEnvelope` & Tool Helpers (`chip-tools`)
- Define `ToolAssemblyPart` (`Flute`, `Shank`, `Holder`) and `ToolAssemblyEnvelope` (constructed via `ToolAssemblyEnvelope::from_tool(tool: &Tool)`) in `crates/chip-tools/src/envelope.rs` owning the canonical flute/neck/shank envelope, stickout, holder nut/body/cone dimensions, `effective_non_rubbing_reach_mm`, `is_center_cutting`, and `max_safe_plunge_depth_mm`.
- Add `CutterGeometry::corner_radius_mm(&self) -> f64` and `is_exact_tool_match` / `rank_replacement_tools` in `chip-tools` (removing private helpers in `chip-project`).
- Migrate `chip-cam/src/fixture_avoidance.rs` (`ToolAssemblyProfile`), `chip-cam/src/reach_analysis.rs`, `chip-stock/src/types.rs` (`RadialCutterProfile`), `chip-sim/src/cutter_profile.rs`, and `chip-cam/src/surface3d/cutter.rs` (`AssemblyClearanceSpec` / `SurfaceCutterProfile`) onto `chip_tools::ToolAssemblyEnvelope`—fixing the stale-envelope bug in `SurfaceCutterProfile::with_radial_shrink` (`surface3d/cutter.rs:431`).

---

## Verification Plan
- `cargo fmt --all -- --check`
- `cargo clippy --locked --workspace --all-targets -- -D warnings`
- `WGPU_BACKEND=vulkan cargo test --locked --workspace`
- Verify zero golden G-code diffs (`crates/chip-cli/tests/golden/`).
