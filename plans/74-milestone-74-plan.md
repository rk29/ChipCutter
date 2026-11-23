# Milestone 74 Implementation Plan — Overhang-Aware 3D Finishing (`Parallel3D`, `Scallop3D`, `Pencil3D`, Rest Masks) & Swept Soft-Jaw Silhouettes

## Goal

Eliminate overhang-shadow artifacts across 3D surface finishing strategies and workholding silhouette generation on helical lofts and cantilevered parts:
1. **Suppress False Overhang-Shadow Creases & Rest Bands (`finishing.rs`, `pencil3d.rs`):** Distinguish genuine concave valleys/fillets from vertical overhang drop-off discontinuities in `compute_morphological_rest_residual`, `build_3d_rest_mask`, and `Pencil3D`.
2. **Overhang-Cliff-Safe `Parallel3D` & `Scallop3D` (`parallel3d.rs`, `scallop3d.rs`):** Prevent `Scallop3D` Fast Marching iso-contours from piling up along artificial overhang shadow cliffs, and prevent `Parallel3D`/`Scallop3D` from plunging vertically down overhang shadow drop-offs at full `cut_feed`.
3. **Exact Swept Clamping-Band Silhouettes for Auto Soft-Jaws & Flip-Jigs (`workholding_wizards.rs`):** Replace the coarse 9-slice `slice_surface_contours_z` union with a continuous swept XY envelope over `[z_lo, z_hi]` so twisted helical blades never interfere with generated soft-jaw pockets.

---

## Problem Diagnosis

1. **Overhang Drop-Off False Positives in `compute_morphological_rest_residual` (`crates/chip-cam/src/surface3d/finishing.rs:489-609`) & `Pencil3D` (`pencil3d.rs:151-186`):**
   - `compute_morphological_rest_residual` computes the morphological difference between `z_ref` (larger reference ball) and `z_cur` (smaller ball) assuming `z(x, y)` is a continuous single-valued surface.
   - Directly outside the silhouette of an overhanging helical blade, `z_ref` still rests on the overhanging blade `ΔR = R_ref - R_cur` further outward than `z_cur`, which has already dropped onto the lower floor/hub beneath. That creates a tall spurious residual band all along the overhang shadow edge:
     - `Pencil3D` traces false "valley" passes along every overhang drop-off shadow cliff.
     - `build_3d_rest_mask` marks overhang shadow fringes as active 3D rest regions for `Parallel3D` and `Scallop3D`.
2. **Vertical Overhang Shadow Cliffs in `Scallop3D` (`scallop3d.rs:183-216`) & `Parallel3D` (`parallel3d.rs:285-334`):**
   - `Scallop3D` estimates surface slope `sec(θ)` via finite differences of `z_grid`. Across an overhang drop-off discontinuity (`Δz` jump between an upper blade and a disconnected lower hub), finite differences see a fake `90°` vertical wall (`weight = 4.0`), packing dense unnecessary iso-contours right along the shadow edge.
   - Both `Parallel3D` and `Scallop3D` emit a continuous polyline across an overhang drop-off, diving almost vertically from the upper blade edge down onto the lower hub at full `cut_feed` instead of splitting/linking cleanly or using `plunge_feed`.
3. **Scalloped 9-Slice Silhouettes in `workholding_wizards.rs` (`workholding_wizards.rs:386-445`):**
   - `extract_part_silhouette_in_z_band` unions `slice_surface_contours_z` at only `N_SAMPLES = 9` discrete Z planes (plus horizontal B-Rep planes).
   - A continuously twisting helical loft has no intermediate horizontal planes, so 9 slices leave scalloped gaps between slice heights—producing soft-jaw pockets that can slightly under-clear the helical blade crests.

---

## Proposed Changes

### 1. Overhang-Shadow Discontinuity Discrimination in `compute_morphological_rest_residual`, `build_3d_rest_mask`, & `Pencil3D`

#### [MODIFY] `crates/chip-cam/src/surface3d/finishing.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/pencil3d.rs`
- **Detect True Concave Crease vs. Overhang Shadow Cliff:**
  - Add a fast, exact geometric discriminator `is_overhang_shadow_drop(surface, x, y, z_cur, z_ref_env, cur_r, ref_r, scratch)` (or grid-based continuity + vertical gap check):
    - A genuine **concave fillet/valley** has continuous solid material connecting the wall and floor at heights between `z_cur` and `z_ref_env`, so probing horizontally between the `z_cur` contact and the high `z_ref` rim encounters continuous rising surface points AND upward-facing or vertical wall triangles bridging `[z_cur, z_ref_env]`.
    - An **overhang shadow drop-off** exhibits a sharp vertical jump (`z_ref - z_cur > 0.35 * d_r`) where `z_cur` drops discontinuously across an open void beneath an overhanging downward-facing surface (`n.z < -0.15`)—with **no solid material** bridging the vertical gap `(z_cur + cur_r, z_overhang_bottom)` at `(x, y)`!
  - In `compute_morphological_rest_residual` (or a new companion filter `suppress_overhang_shadow_residuals` called by `build_3d_rest_mask` and `Pencil3D`):
    - Zero out residual cells whose high `z_ref` is propped solely by a disconnected overhanging roof above an open air gap rather than a connected concave wall/fillet.
  - Preserve **100% sensitivity** on true concave corners, V-grooves, vertical-wall-to-floor fillets, and sculpted relief valleys (all existing `Pencil3D` and `build_3d_rest_mask` tests stay exact!).

---

### 2. Overhang-Cliff-Safe `Scallop3D` Weighting & `Parallel3D`/`Scallop3D` Cliff Splitting

#### [MODIFY] `crates/chip-cam/src/strategies/scallop3d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/parallel3d.rs`
#### [MODIFY] `crates/chip-cam/src/surface3d/finishing.rs`
- **Discontinuity-Aware Slope Weighting (`scallop3d.rs`):**
  - When computing `dz_dx` / `dz_dy` in `Scallop3D`, detect adjacent-cell vertical step discontinuities where `|z_neighbor - z_c| > max(2.0 * cell, 0.4 * tool_r)` AND the midpoint between the two cells exhibits an open overhang shadow gap (not a continuous steep wall).
  - Switch automatically to one-sided differences on the continuous side of the cliff (falling back to `0.0` if isolated) so the Eikonal `sec(θ)` metric reflects true local surface slope without creating artificial high-density contour bands along overhang shadows.
- **Safe Downward Cliff Splitting & Linking (`parallel3d.rs`, `scallop3d.rs`):**
  - Add a shared helper `split_chains_at_overhang_cliffs(surface, cutter, chain, max_drop_mm)` in `finishing.rs`:
    - Whenever a segment descends steeply (`a.z - b.z > max(1.5, 0.5 * tool_d)` with `slope_drop > 70°`) across an overhang shadow drop-off, split the cutting chain at the cliff crest instead of diving straight down at full `cut_feed`.
    - Adjacent sub-chains then connect automatically via `emit_surface_safe_link` (which retracts/approaches cleanly and plunges at `plunge_feed` instead of `cut_feed`).
  - Keep continuous steep walls (like the sculpted dome flank) intact as smooth uncut chains!

---

### 3. Exact Continuous Swept Silhouette Extraction for Workholding Wizards

#### [MODIFY] `crates/chip-cam/src/workholding_wizards.rs`
#### [MODIFY] `crates/chip-cam/src/surface3d/push_cutter.rs`
- **Continuous Swept XY Envelope (`extract_part_silhouette_in_z_band`):**
  - Replace the fixed `N_SAMPLES = 9` discrete Z-slice union in `workholding_wizards.rs` with an exact continuous swept XY footprint across `[z_lo, z_hi]`:
    - Clip every triangle of `surface` to the Z slab `[z_lo, z_hi]` (preserving full horizontal projection of every clipped triangle, including continuously twisting helical flanks between Z planes!).
    - Extract the exact outer silhouette `RegionSet2D` using a fine parallel occupancy/distance grid + Marching Squares with exact analytical boundary bisection (reusing the `PushCutterField` pattern with zero-radius vertical-ray / triangle-XY projection on clipped triangles, plus exact inclusion of any planar horizontal face loops).
  - Keep the existing `filled_hulls` guarantee (no interior mounting holes become floating soft-jaw pins) and deterministic vertex ordering.

---

## Verification Plan

1. **Overhang-Shadow Residual & `Pencil3D` Tests (`chip-cam`, `chip-sim`):**
   - T-flange / helical-blade overhang test with a genuine concave fillet on the hub base AND an overhanging blade edge:
     - Verify `Pencil3D` detects and traces the true concave hub fillet while emitting **zero spurious passes** along the overhanging blade's shadow drop-off edge!
     - Verify `build_3d_rest_mask` masks the concave hub fillet without false overhang-shadow bands.
2. **`Parallel3D` & `Scallop3D` Overhang-Cliff Tests (`chip-cam`, `chip-sim`):**
   - Overhanging helical-loft benchmark machined with `Parallel3D` and `Scallop3D`:
     - Verify zero rapid-into-stock crashes, zero target gouges (`max_gouge <= 0.015 mm`), no full-`cut_feed` near-vertical dives off overhang cliffs, and uniform iso-contour spacing in `Scallop3D`.
   - Confirm sculpted dome (`08-mesh-surfacing`) and sinusoidal relief (`09-3d-finishing`) quality thresholds from Milestone 59 remain strictly met or improved!
3. **Helical Soft-Jaw Silhouette Zero-Interference Test (`chip-cam`):**
   - Generate Auto Soft-Jaws clamping a continuously twisted helical shaft/loft (`90°` twist across the clamping band):
     - Verify every vertex and sampled point along the helical loft inside `[z_lo, z_hi]` lies strictly inside the generated soft-jaw pocket (zero jaw-to-blade interference between slice planes!), with smooth non-scalloped jaw contours.
4. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
