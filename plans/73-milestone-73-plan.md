# Milestone 73 Implementation Plan — `Indexed3Plus1` Cross-Angle Rest Machining, Volumetric Carryover, & Swept Slab Excess Masks

## Goal

Make multi-angle `Indexed3Plus1` roughing and multi-setup stock carryover completely safe and accurate on twisted helical lofts:
1. **Fix Cross-Angle `upstream_rest` Frame Corruption (`chip-cam`):** Prevent 2D swept toolpath footprints from one rotary index angle (e.g., `0°`) from being subtracted untransformed in a different index angle's XY plane (e.g., `120°` / `240°`).
2. **In-Setup Cross-Angle Volumetric `TriDexelStock` Carryover for `Indexed3Plus1` (`chip-cam`, `chip-stock`, `chip-project`):** Carry real 3D machined stock across sequential `Indexed3Plus1` operations (`0° -> 120° -> 240°`) by rotating the running `TriDexelStock` into each indexed virtual Setup frame—eliminating redundant outer-stock air-cutting without ever misclassifying overhanging helical blades as cleared air.
3. **Swept Z-Slab Part Masks in `Adaptive3D` & `Waterline3D` (`chip-cam`):** Replace sparse 1–2 Z-plane `slice_surface_contours_z` subtraction in carryover excess-stock masking with an exact swept Z-slab projection so fast-twisting helical lofts never leave false excess-stock slivers between sample planes.

---

## Problem Diagnosis

1. **Untransformed `upstream_rest` in `Indexed3Plus1` (`crates/chip-cam/src/strategies/mod.rs:223-238`, `crates/chip-cam/src/rest2d.rs:49-119`):**
   - `generate_operation_toolpath_with_rest` builds a rotated `virtual_setup` for `OperationRotaryMode::Indexed3Plus1 { angle_deg }`, but passes `upstream_rest` untouched into the recursive call and hard-codes `carryover_stock: None`.
   - `swept_toolpath_coverage_2d` projects upstream `segments_wcs` onto XY without checking `start_rotary_deg` / `end_rotary_deg`.
   - On a multi-index helical part (`0°`, `120°`, `240°`) with `RestMachiningMode::FromPrecedingOperations` enabled, the `0°` toolpath's XY footprint is subtracted directly in the `120°` virtual XY plane—wrong coordinates that can falsely label uncut helical blade stock as `upstream_air` (causing unsafe micro-lift links or skipped cuts).
2. **No Cross-Angle Volumetric Carryover Inside a Rotary Setup (`chip-project/src/dispatcher.rs`, `chip-cam/src/strategies/mod.rs`):**
   - Even when `0°` removes 60%+ of the outer cylindrical stock, `120°` and `240°` currently start from raw unmachined stock because `carryover_stock` is always `None` for `Indexed3Plus1`.
3. **Sparse 1–2 Plane Carryover Subtraction (`adaptive3d.rs:136-157`, `waterline3d.rs:180-192`):**
   - `carryover_excess_masks` in `Adaptive3D` and `slice_containment` in `Waterline3D` subtract `slice_surface_contours_z` at only 1–2 discrete Z values inside `[slab_lo, slab_hi]` to separate remaining uncut stock from the finished target solid.
   - On a steep helical loft whose XY cross-section rotates across a 2–4 mm coarse roughing stepdown, the blade sweeps between those sample planes, creating false "excess stock" slivers around already-finished blade flanks.

---

## Proposed Changes

### 1. Rotary-Angle-Scoped 2D Rest Filtering (`chip-cam`, `chip-project`)

#### [MODIFY] `crates/chip-cam/src/rest2d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/mod.rs`
#### [MODIFY] `crates/chip-project/src/dispatcher.rs`
#### [MODIFY] `crates/chip-project/src/pipeline.rs`
- **Index-Angle Provenance on `UpstreamSweptInput` (`rest2d.rs`):**
  - Add `pub indexed_angle_deg: Option<f64>` (normalized mod `360°`, `None` for standard 3-axis / `0°` non-rotary ops or non-planar simultaneous 4-axis ops) and `pub is_rotary_native: bool` on `UpstreamSweptInput`.
  - Add helper `filter_upstream_for_indexed_angle(inputs: &[UpstreamSweptInput], target_angle_deg: Option<f64>) -> Vec<UpstreamSweptInput>`:
    - Matches only upstream inputs whose `indexed_angle_deg` equals `target_angle_deg` within `1e-3°` (modulo `360°`) and excludes rotary-native simultaneous/wrapped toolpaths from 2D planar `swept_toolpath_coverage_2d`.
  - In `strategies/mod.rs`: filter `upstream_rest` before recursing into `Indexed3Plus1` (and into `WrapCylinder`) so 2D swept coverage NEVER mixes coordinates from different rotary angles!
  - Inside `swept_toolpath_coverage_2d`: also defensively skip any segment whose `start_rotary_deg` and `end_rotary_deg` differ by `> 1e-3°` (simultaneous rotary motion is never a flat 2D XY capsule).

---

### 2. In-Setup Cross-Angle Volumetric `TriDexelStock` Carryover for `Indexed3Plus1`

#### [MODIFY] `crates/chip-cam/src/strategies/mod.rs`
#### [MODIFY] `crates/chip-stock/src/tridexel/transfer.rs`
#### [MODIFY] `crates/chip-project/src/dispatcher.rs`
#### [MODIFY] `crates/chip-project/src/pipeline.rs`
#### [MODIFY] `crates/chip-project/src/cache.rs`
- **In-Setup Sequential Indexed Carryover (`chip-project`, `chip-cam`):**
  - Remove the hard-coded `carryover_stock: None` override inside `Indexed3Plus1` in `strategies/mod.rs`; instead allow a pre-rotated `carryover_stock` (already expressed in `virtual_setup.frame`, or rotated on demand from `setup.frame` via `TriDexelStock::transform_to_frame`) to flow into the inner `Adaptive3D` / `Waterline3D` / `Pocket2D` / `Facing` strategy!
  - In `chip-project` (`dispatcher.rs` & `pipeline.rs`):
    - When a Setup has `RotaryDriveMode::Indexed3Plus1` (or contains multiple `Indexed3Plus1` operations at distinct angles) and an indexed operation enables `RestMachiningMode::FromPrecedingOperations`, build/update a fast in-setup `TriDexelStock` (using `SimResolution::Standard`, or GPU compute when available) by sweeping the preceding operations (at their respective rotary angles) into Setup WCS, transform the resulting `TriDexelStock` into the current operation's `virtual_setup.frame`, and pass it as `carryover_stock`!
    - Include upstream operation hashes in the `ToolpathCache` key so editing the `0°` op automatically invalidates dependent `120°` / `240°` rest ops cleanly through the existing DAG wave scheduler.
  - **Fast Rotary-Axis Transfer Check (`chip-stock`):**
    - Verify `TriDexelStock::transform_to_frame` handles `90°`/`180°`/`270°` cardinal index angles via the zero-blur exact axis-permutation fast path and arbitrary angles (`60°`, `120°`, `240°`, etc.) via the watertight BVH re-voxelization path with `< 0.5%` volume accuracy and zero missed overhang solids.

---

### 3. Swept Z-Slab Target-Part Masking in `Adaptive3D` & `Waterline3D`

#### [NEW] `crates/chip-cam/src/surface3d/slab_projection.rs` (or helper in `crates/chip-cam/src/surface3d/push_cutter.rs`)
#### [MODIFY] `crates/chip-cam/src/strategies/adaptive3d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/waterline3d.rs`
- **Exact Swept Z-Slab Target Footprint (`project_surface_z_slab_xy`):**
  - Given `surface: &SurfaceMesh3D` and `[z_lo, z_hi]`:
    - Query `surface.bvh.query_z_slab(z_lo, z_hi)`.
    - Instead of sampling only 1–2 horizontal Z planes, build the exact swept XY footprint of the target solid across the entire slab `[z_lo, z_hi]` by combining:
      1. Clipped horizontal cross-sections at adaptive spacing `dz_sub <= 0.5 * xy_pitch` (capped cleanly), OR
      2. Exact 2D polygon projection of every triangle clipped to `[z_lo, z_hi]` (or rasterized onto the carryover `TriDexelStock`'s XY grid + Marching Squares contour extraction, which runs in `< 2 ms` in parallel and is guaranteed to cover every intermediate twisted helical blade position inside `[z_lo, z_hi]`!).
  - Use this swept slab footprint (dilated by `halo`) inside both `Adaptive3D::carryover_excess_masks` and `Waterline3D::slice_containment`.
  - Guarantee that `clearable_curr` is still intersected with `!push_field.avoidance_region_z(z_curr)`, preserving strict monotonicity and zero overhang gouges!

---

## Verification Plan

1. **Cross-Angle `Indexed3Plus1` Safety & Carryover Tests (`chip-cam`, `chip-project`, `chip-sim`):**
   - **Regression Test (Bug Reproduction):** 3-angle `Indexed3Plus1` (`0°`, `120°`, `240°`) `Adaptive3D` on a twisted helical loft with `RestMachiningMode::FromPrecedingOperations` enabled on `120°` and `240°`:
     - Verify `0°` 2D swept capsules are NEVER subtracted in `120°`/`240°` virtual XY space.
     - Verify volumetric Tri-Dexel carryover reduces total `120°` + `240°` toolpath length significantly (`>= 25%` air-cut reduction vs. raw-stock roughing) while maintaining **`0.0 mm³` target gouge** and **`0` rapid-into-stock crashes** in full multi-op `TriDexel` simulation!
   - **Same-Angle 2D Rest Preservation Test:** Two sequential `Indexed3Plus1` ops at the *same* angle (`90°` roughing Ø8 -> `90°` rest cleanup Ø3) still share exact 2D `swept_toolpath_coverage_2d`.
2. **Swept Z-Slab Mask Accuracy Test (`chip-cam`):**
   - Fast-twisting helical blade inside a 4 mm Z slab where the blade rotates `45°` across `[z_lo, z_hi]`:
     - Verify `project_surface_z_slab_xy` covers 100% of the twisted blade's XY sweep (zero false excess-stock slivers on already-finished blades).
3. **Phase 1–5 Golden Parity & Full Workspace Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
