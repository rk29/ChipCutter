# Milestone 49 Implementation Plan — Flat-Array Spatial Index (`chip-bvh`) & 2D Exact Geometry Kernel (`chip-geom2d`)

## Goal

Extract two clean, UI-agnostic internal algorithm crates—**`chip-bvh`** and **`chip-geom2d`**—with zero new external dependencies, eliminating the `O(N)` linear raycast performance cliff in `chip-cad`, unifying all three `cavalier_contours` bridges (propagating the M43 `WORK_SCALE = 16.0` evolute-cusp panic guard to `chip-sketch`), removing `chip-sim`'s dependency on `chip_cam::SurfaceMesh3D`, and producing a GPU-uploadable flat SAH BVH buffer layout ready for future GPU Error Heatmap queries.

---

## Proposed Changes

### 1. New Internal Crate: `chip-bvh` (Flat-Array SAH BVH & `TriMesh3D`)

#### [NEW] `crates/chip-bvh/Cargo.toml`
#### [NEW] `crates/chip-bvh/src/lib.rs`
#### [NEW] `crates/chip-bvh/src/bvh.rs`
#### [NEW] `crates/chip-bvh/src/mesh.rs`
#### [MODIFY] [Cargo.toml](file:///workspace/ChipCutter/Cargo.toml)
- Dependencies: `chip-math`, `glam`, `bytemuck`, `rayon` (zero new external crates; `#![deny(unsafe_code)]`, UI-agnostic).
- Define GPU-compatible flat structures:
  - `FlatBvhNode` (32 bytes, `#[repr(C)] bytemuck::Pod + Zeroable`): `aabb_min: [f32; 3]`, `left_or_first: u32`, `aabb_max: [f32; 3]`, `tri_count: u32` (`0` = internal node with children `left` / `left + 1`; `> 0` = leaf referencing `tri_indices[first..first + count]`).
  - `GpuTriangle` (48 bytes, `#[repr(C)] bytemuck::Pod + Zeroable`): `v0`, `v1`, `v2` (`[f32; 3]` + `face_id`/`flags` padding).
  - Exact `f64` vertex/triangle storage in `TriMesh3D` paired with a binned-SAH flat BVH (`FlatBvh`) so CPU queries remain exact `f64` while `nodes()` / `gpu_triangles()` can be uploaded directly to WGSL storage buffers later.
- Implement core `O(log N)` spatial queries:
  - `raycast(origin, dir, max_t) -> Option<RayHit>` (front-to-back slab traversal with ordered child stack)
  - `raycast_all_hits(origin, dir, max_t, out: &mut Vec<RayHit>)` (for fast even-odd Tri-Dexel target voxelization)
  - `closest_point(point: DVec3) -> Option<ClosestPointHit>` (exact point-to-triangle distance with priority AABB pruning for 3D Error Heatmap & future gouge checks)
  - `query_xy_circle_into(cx, cy, radius, out: &mut Vec<usize>)` (Drop-Cutter)
  - `query_z_slab_into(z_min, z_max, out: &mut Vec<usize>)` (Push-Cutter / Waterline)
  - Segment BVH helper (`SegmentBvh`) for fast `O(log N)` viewport edge picking in `CadPicker`.

#### [MODIFY] [crates/chip-cad/Cargo.toml](file:///workspace/ChipCutter/crates/chip-cad/Cargo.toml)
#### [MODIFY] [crates/chip-cad/src/mesh/bvh.rs](file:///workspace/ChipCutter/crates/chip-cad/src/mesh/bvh.rs)
#### [MODIFY] [crates/chip-cad/src/picking.rs](file:///workspace/ChipCutter/crates/chip-cad/src/picking.rs)
#### [MODIFY] [crates/chip-cam/Cargo.toml](file:///workspace/ChipCutter/crates/chip-cam/Cargo.toml)
#### [MODIFY] [crates/chip-cam/src/surface3d/mod.rs](file:///workspace/ChipCutter/crates/chip-cam/src/surface3d/mod.rs)
#### [MODIFY] [crates/chip-sim/Cargo.toml](file:///workspace/ChipCutter/crates/chip-sim/Cargo.toml)
#### [MODIFY] [crates/chip-sim/src/cylindrical.rs](file:///workspace/ChipCutter/crates/chip-sim/src/cylindrical.rs)
- Replace the `rstar` linear-scan `MeshBvh::raycast` in `chip-cad/src/mesh/bvh.rs` and `CadPicker::pick` / `pick_surface_normal` in `chip-cad/src/picking.rs` with `chip-bvh` `O(log N)` traversal.
- Back `chip_cam::surface3d::SurfaceMesh3D` with `chip_bvh::TriMesh3D` (keeping facade compatibility for Drop-Cutter, Push-Cutter, and rotary strip meshes).
- Switch `chip-sim/src/cylindrical.rs` from importing `chip_cam::SurfaceMesh3D` to building/raycasting `chip_bvh::TriMesh3D` directly.

---

### 2. New Internal Crate: `chip-geom2d` (Exact 2D Offsetting, Booleans, & Medial Axis)

#### [NEW] `crates/chip-geom2d/Cargo.toml`
#### [NEW] `crates/chip-geom2d/src/lib.rs`
#### [NEW] `crates/chip-geom2d/src/offset.rs`
#### [NEW] `crates/chip-geom2d/src/region.rs`
#### [NEW] `crates/chip-geom2d/src/medial.rs`
#### [NEW] `crates/chip-geom2d/src/simplify.rs`
#### [MODIFY] [Cargo.toml](file:///workspace/ChipCutter/Cargo.toml)
- Dependencies: `chip-math`, `glam`, `cavalier_contours`, `boostvoronoi` (zero new external crates; `#![deny(unsafe_code)]`, UI-agnostic).
- Move pure 2D geometric algorithms out of `chip-cam`:
  - `offset.rs`: canonical `to_cavalier_polyline`, `from_cavalier_polyline` (including the M43 `WORK_SCALE = 16.0` evolute-cusp panic safeguard), `offset_polyline`, `PocketRegion2D`, `OffsetRegionNode`, `build_offset_region_tree`, `clip_scanline_to_region`, `HelicalEntryPole`, `find_helical_entry_pole`.
  - `region.rs`: `RegionSet2D`, `swept_segment_capsule_2d`, `clip_polyline_outside_region_set`, `contours_to_region_set` (even-odd nesting).
  - `medial.rs`: `MedialNode2D`, `MedialChain2D`, `extract_interior_medial_nodes`, `extract_medial_axis_chains`, `find_medial_inscribed_pole`.
  - `simplify.rs`: `simplify_rdp_2d` (and move `simplify_3d_polyline_rdp` into `chip-math` or re-export cleanly).

#### [MODIFY] [crates/chip-sketch/Cargo.toml](file:///workspace/ChipCutter/crates/chip-sketch/Cargo.toml)
#### [MODIFY] [crates/chip-sketch/src/typography/ttf_outline.rs](file:///workspace/ChipCutter/crates/chip-sketch/src/typography/ttf_outline.rs)
#### [MODIFY] [crates/chip-sketch/src/heal_chain.rs](file:///workspace/ChipCutter/crates/chip-sketch/src/heal_chain.rs)
#### [MODIFY] [crates/chip-cam/Cargo.toml](file:///workspace/ChipCutter/crates/chip-cam/Cargo.toml)
#### [MODIFY] [crates/chip-cam/src/offset2d.rs](file:///workspace/ChipCutter/crates/chip-cam/src/offset2d.rs)
#### [MODIFY] [crates/chip-cam/src/rest2d.rs](file:///workspace/ChipCutter/crates/chip-cam/src/rest2d.rs)
#### [MODIFY] [crates/chip-cam/src/medial.rs](file:///workspace/ChipCutter/crates/chip-cam/src/medial.rs)
#### [MODIFY] [crates/chip-cam/src/surface3d/push_cutter.rs](file:///workspace/ChipCutter/crates/chip-cam/src/surface3d/push_cutter.rs)
- Replace the duplicate `cavalier_contours` polyline converters in `chip-sketch/src/typography/ttf_outline.rs` and `chip-sketch/src/heal_chain.rs` with `chip_geom2d` helpers.
- Re-export moved types in `chip-cam` (`offset2d`, `medial`, `rest2d`) so all downstream consumers remain unaffected, while keeping CAM-specific `UpstreamSweptInput` / `swept_toolpath_coverage_2d` in `chip-cam/src/rest2d.rs`.
- Remove direct `boostvoronoi` dependency from `chip-cam` (and `cavalier_contours` where no longer needed directly).

---

## Verification Plan

1. **New Crate Unit & Oracle Tests:**
   - `chip-bvh`: Verify `FlatBvh` `raycast`, `raycast_all_hits`, `closest_point`, `query_xy_circle_into`, and `query_z_slab_into` against brute-force reference oracles on random & analytical meshes (box, cylinder, dome, multi-island solid), plus `bytemuck::Pod` size/alignment assertions (`size_of::<FlatBvhNode>() == 32`).
   - `chip-geom2d`: Verify offsetting, `RegionSet2D` booleans/capsules/clipping, Voronoi medial axis, and evolute-cusp high-curvature spline stress cases (e.g., Relief Wave).
2. **Full Workspace Parity Gate:**
   - `cargo fmt --all -- --check`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace` (100% golden G-code parity + picking + 3D surface + rotary tests)
