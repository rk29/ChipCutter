# Milestone 72 Implementation Plan — Full-Assembly Shank/Holder Overhang Avoidance & 2.5D Overhang Safety Guards

## Goal

Close the remaining vertical-assembly and 2.5D collision blind spots around overhanging helical lofts and tall steep overhangs:
1. **Full-Assembly Target-Part Clearance (`SurfaceCutterProfile` / `drop_cutter_z` / `PushCutterField`):** Protect overhanging helical blades and tall walls against **wider non-cutting shanks** (`shank_diameter_mm > cutting_diameter_mm`) and **collet nut / holder bodies** (`dz >= stickout_length_mm`), reusing the canonical `ToolAssemblyProfile` geometry from `fixture_avoidance.rs`.
2. **2.5D Overhang Safety Guards & Opt-In Model Avoidance (`Pocket2D`, `Contour2D`, `DrillBore`, `Facing`, `ChamferTrace`):** Automatically detect and warn when selected 2.5D features sit beneath overhanging 3D solids, and provide a schema-safe `avoid_model_overhangs` option that subtracts/clips overhanging 3D avoidance shadows out of 2.5D toolpaths—while preserving 100% byte-for-byte parity on all Phase 1–5 golden G-code snapshots.

---

## Problem Diagnosis

1. **Cutter-Only `SurfaceCutterProfile` (`crates/chip-cam/src/surface3d/{cutter.rs, drop_cutter.rs}`):**
   - `drop_cutter_z` and `PushCutterField` evaluate only `cutter.max_radius_mm()` (`cutting_radius_mm`) extending upward to $+\infty$.
   - When `shank_diameter_mm > cutting_diameter_mm` (e.g., a Ø3 mm endmill on a Ø6 mm shank reaching down alongside a tall helical loft) or when an overhanging helical blade rises above `stickout_length_mm` into the collet nut / holder cone (`ToolAssemblyProfile`), the cutting tip clears the part while the upper shank or holder crashes into the overhanging blade.
   - `ToolAssemblyProfile` in `crates/chip-cam/src/fixture_avoidance.rs` already models flutes, relieved neck, shank, collet nut, and tapered holder cone—but is currently evaluated only against workholding fixtures, not against the target part mesh.
2. **Unchecked 2.5D Plunges Beneath Overhangs (`pocket2d.rs`, `contour2d.rs`, `drill_bore.rs`, `facing.rs`, `chamfer_trace.rs`):**
   - By design, 2.5D strategies operate solely on selected `ExtractedFeature25D` faces/contours/holes without querying `SurfaceMesh3D`.
   - If a user picks a lower planar shelf, contour, or hole tucked beneath an overhanging helical loft (in 3-axis or indexed `3+1` setups), the generated 2.5D toolpath plunges straight through the overhanging blade above it without warning.

---

## Proposed Changes

### 1. Full-Assembly (`Flute + Shank + Holder`) Clearance in `drop_cutter_z` & `PushCutterField`

#### [MODIFY] `crates/chip-cam/src/surface3d/cutter.rs`
#### [MODIFY] `crates/chip-cam/src/surface3d/drop_cutter.rs`
#### [MODIFY] `crates/chip-cam/src/surface3d/push_cutter.rs`
#### [MODIFY] `crates/chip-cam/src/fixture_avoidance.rs`
- **Unified Assembly Envelope (`SurfaceCutterProfile::from_tool`):**
  - Add an optional `AssemblyClearanceSpec` on `SurfaceCutterProfile` (default `None` for `from_tool_geometry(...)` so low-level analytical unit tests remain unchanged; populated via `SurfaceCutterProfile::from_tool(tool, stock_radial, stock_axial, holder_clearance_mm)` in 3D strategies when `shank_diameter_mm > cutting_diameter_mm + 1e-4` or holder avoidance is enabled).
  - Reuse `ToolAssemblyProfile::from_tool(tool)` radial zones (`flute_length_mm`, `stickout_length_mm`, `shank_radius_mm`, `holder_nut_radius_mm`, `holder_body_radius_mm`, `holder_cone_z0..z1`) with a configurable `holder_clearance_mm` (default `0.5 mm`).
- **Exact Analytical Multi-Tier Drop-Cutter Evaluation (`drop_cutter.rs`):**
  - Keep the fast single-disk path when `shank_radius <= cutting_radius` and `surface.aabb_wcs.max.z - z_floor < stickout_length_mm` (zero overhead for normal short-wall parts!).
  - When wider shank or holder tiers can overlap `surface.aabb_wcs`:
    - Query `surface.bvh` with `r_query = max(cutting_radius, shank_radius, holder_body_radius)` (pruned strictly by `tri.aabb.max.z > best_z + dz_tier_min`).
    - For each tier `(dz_lo, r_tier)` (stepped shank above `flute_length_mm`, collet nut above `stickout_length_mm`, and tapered holder cone `holder_cone_z0..z1`), compute exact analytical triangle/edge/vertex contact height `z_tier = z_contact_tier - dz_tier` and take `max(z_flute, z_shank, z_holder)`.
  - Because `PushCutterField` derives directly from `drop_cutter_z_with_scratch`, `Adaptive3D`, `Waterline3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`, and `emit_surface_safe_link` all inherit full-assembly shank and holder overhang avoidance automatically and remain strictly monotonic across descending Z levels!
- **Reach-Limit Diagnostics:**
  - Record a clear non-blocking warning in `GeneratedToolpath::warnings` whenever shank or holder clearance lifts a 3D pass above the flute contact height (reporting max lift and recommending longer flute/stickout or a relieved-neck cutter).

---

### 2. 2.5D Overhang Collision Guard & Opt-In Model Avoidance

#### [NEW] `crates/chip-cam/src/surface3d/overhang_guard.rs`
#### [MODIFY] `crates/chip-cam/src/surface3d/mod.rs`
#### [MODIFY] `crates/chip-cam/src/params/common.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/{mod.rs, pocket2d.rs, contour2d.rs, drill_bore.rs, facing.rs, chamfer_trace.rs}`
#### [MODIFY] `crates/chip-gui/src/panels/op_inspector/passes_tab.rs`
- **Schema-Safe `PassesConfig` Field (`params/common.rs`):**
  - Add `#[serde(default, skip_serializing_if = "is_false")] pub avoid_model_overhangs: bool` on `PassesConfig` (defaults to `false` so every existing `.chip.toml` / `.chipz` and all Phase 1–5 golden G-code snapshots remain 100% byte-identical).
- **Automatic Overhang Detection Warning for All 2.5D Ops (`overhang_guard.rs`, `strategies/mod.rs`):**
  - After any 2.5D strategy finishes when `ctx.doc.has_3d_geometry()` and `avoid_model_overhangs == false`, run a fast BVH check over emitted cutting/plunge segments against `SurfaceMesh3D` (checking only triangles strictly above `seg_z + 0.05 mm`, skipping self-floor/wall grazing).
  - If an overhanging solid intersects the cutter's vertical shadow or plunge column, append a clear diagnostic warning to `GeneratedToolpath::warnings` naming the affected Z height and suggesting enabling **Avoid Model Overhangs** or switching to `Adaptive3D` / `Undercut3D`.
- **Opt-In 2.5D Overhang Avoidance (`avoid_model_overhangs == true`):**
  - **`Pocket2D` & `Facing`:** Build a single `PushCutterField` for the active tool (including shank/holder tiers) and subtract `push_field.avoidance_region_z(z_cut)` from each Z level's clearable region (`effective_rough` / facing strips), ensuring neither cutting passes nor helical/ramp entries touch overhanging blades above `z_cut`.
  - **`Contour2D` & `ChamferTrace`:** Clip offset contour polylines strictly outside `push_field.avoidance_region_z(z_cut)` via `clip_polyline_outside_region_set`, keeping tangent lead-in/out moves inside verified overhang-free air.
  - **`DrillBore`:** Verify each hole center's full vertical column `[z_bottom, retract_z]` against `drop_cutter_z` (including shank/holder); skip occluded holes with a clear per-hole warning rather than plunging through an overhanging loft.
- **Inspector UI (`chip-gui`):**
  - Add a two-column **"Avoid Model Overhangs"** checkbox with hover tooltip in the `Passes` tab for 2.5D operations when a 3D model is loaded, and surface overhang warnings in the Diagnostics panel.

---

## Verification Plan

1. **Shank & Holder Overhang Avoidance Tests (`chip-cam`, `chip-sim`):**
   - Tall overhanging helical/T-flange model machined with a reduced-flute cutter (`cutting_d = 4 mm`, `flute_length = 8 mm`, `shank_d = 8 mm`, `stickout = 18 mm`):
     - Verify `drop_cutter_z` and `PushCutterField` stay clear of the upper overhang by `>= shank_r` above `flute_length` and `>= holder_nut_r` above `stickout`.
     - Verify `Adaptive3D`, `Waterline3D`, and `Parallel3D` pass `chip-sim` Tri-Dexel verification with **0 `ShankRub`**, **0 `HolderCollision`**, and **0 `TargetModelGouge`** errors, plus emit the reach-limit warning.
   - Verify a standard `shank_d == cutting_d` tool with adequate stickout produces **exact same results** (zero regression on existing 3D goldens).
2. **2.5D Overhang Guard & Avoidance Tests (`chip-cam`, `chip-sim`):**
   - Stepped pocket / hole placed partially underneath an overhanging helical bridge:
     - With `avoid_model_overhangs = false`: verify `Pocket2D`, `Contour2D`, and `DrillBore` emit a structured overhang collision warning.
     - With `avoid_model_overhangs = true`: verify `Pocket2D`, `Contour2D`, `Facing`, and `DrillBore` trim cleanly around the overhang shadow and pass `chip-sim` Tri-Dexel verification with **0 target gouges** and **0 rapid-into-stock crashes**.
   - Confirm 100% byte-for-byte parity on all Phase 1–5 golden G-code snapshots.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
