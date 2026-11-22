# Milestone 57 Implementation Plan — 3-Axis Surface Finishing & Roughing Quality Polish

## Goal

Eliminate the target surface gouges measured on 3-axis sculpted finishing (`09-3d-surface-finishing`: `gouge_volume = 65.1 mm³`, `max_gouge = -0.146 mm`) and substantially reduce excess volume and `RMS` error across `Parallel3D`, `Scallop3D`, `Waterline3D`, `Adaptive3D`, and `Pencil3D`—while preserving 100% byte-for-byte parity on Phase 1 & Phase 2 golden G-code snapshots.

---

## Baseline Diagnosis (Measured on v0.15.0 GPU Tri-Dexel Fine)

| Sample | Baseline Err Vol (`mm³`) | Excess / Gouge (`mm³`) | Baseline RMS (`mm`) | Max Ex / Gg (`mm`) | Baseline In-Tol (`±0.05 mm`) | Key Root Causes |
|---|---:|---|---:|---|---:|---|
| `09-3d-finishing` | 3,553.9 | 3,488.8 / **65.1** | 0.5420 | +2.000 / **-0.146** | 75.2% | `refine_convex_chords_3d` (`surface3d/finishing.rs:108`) skips chords where `dz <= 0.08 && xy_dist <= 0.6`—allowing chords spanning sharp sinusoidal ridges/crests to dip below `drop_cutter_z`; `Scallop3D` caps Eikonal grid at 42k cells (`0.25–0.75 mm`) and projects coarse marching-squares vertices without exact post-RDP convex verification. |
| `08-mesh-surfacing` | 5,597.7 | 5,597.7 / 0.0 | 0.6468 | +2.000 / -0.003 | 53.2% | Steep-wall stepover expansion in `Parallel3D` (fixed XY stepover stretches 3D scallop on >45° dome slopes), boundary containment clipping leaving perimeter ridges (`+2.0 mm`), and coarse `Pencil3D` cell snapping. |
| `08-3d-adaptive-wl` | 5,727.1 | 5,727.1 / 0.0 | 0.6546 | +2.155 / -0.001 | 54.5% | `Waterline3D` adaptive stepdown (`waterline3d.rs:124`) uses global average-radius `sqrt(area/pi)` instead of true local radial contour offset `dr`, missing localized shallow shelves/crests; `Adaptive3D` fine step-ups skip narrow annular bands `< 0.25 mm²`. |

---

## Proposed Changes

### 1. Exact Gouge-Free 3D Chord Refinement & Curvature-Aware `Scallop3D`

#### [MODIFY] `crates/chip-cam/src/surface3d/finishing.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/scallop3d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/parallel3d.rs`
- **Fix `refine_convex_chords_3d` Blind Spot (`finishing.rs`):**
  - Remove the `dz > 0.08 || xy_dist > 0.6` skip gate; instead evaluate exact `drop_cutter_z_with_scratch` at chord midpoint (and quarter-points on chords `> 0.25 mm`) whenever `xy_dist > 0.03 mm`, recursively bisecting up to depth 5 whenever `z_surf > chord_mid_z + sag_tol` (`sag_tol` clamped to `<= 0.003 mm`) OR when the chord's concave valley deviation exceeds `smooth_tol`!
  - Enforce **post-RDP gouge safety** (`simplify_3d_polyline_rdp` followed by a convex-dip guard ensuring simplified chords NEVER dip `> 0.002 mm` below any dropped vertex or exact surface midpoint!).
  - Audit `emit_surface_safe_link` so low-clearance stay-down links between passes also verify exact `drop_cutter_z` clearance along the link arc/polyline (`+0.02 mm` safety lift when skimming over ridges).
- **Higher-Fidelity `Scallop3D` (`scallop3d.rs`):**
  - Raise Eikonal grid budget cleanly (`42_000 -> 120_000` cells, `base_cell` down to `0.15 mm` — fast in parallel with `chip-bvh`) and smooth the surface slope metric `sec(θ)` before Fast Marching so iso-contours are clean and kink-free.
  - Project every extracted iso-contour vertex onto exact `drop_cutter_z`, run exact `refine_convex_chords_3d`, and properly close/overlap closed scallop loops to prevent visible start/end seam cusps.
- **Slope-Aware `Parallel3D` (`parallel3d.rs`):**
  - Ensure `sample_step` adapts cleanly to tool diameter and smoothing tolerance (`<= 0.25 mm` on curved reliefs) and run post-RDP exact convex chord verification so parallel passes never clip crests or leave faceted zig-zag turnaround scars.

---

### 2. Local-Slope `Waterline3D`, Crisp `Adaptive3D`, & Sub-Cell `Pencil3D`

#### [MODIFY] `crates/chip-cam/src/strategies/waterline3d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/adaptive3d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/pencil3d.rs`
- **True Local Radial Offset Adaptive Z-Stepdowns (`waterline3d.rs`):**
  - Replace the global `sqrt(area / pi)` approximation in `Waterline3D` adaptive stepdown with the actual maximum 2D Hausdorff / point-to-boundary distance `dr_max` between `prev_avoid` and `cur_avoid` contours, inserting cusp-bounded intermediate Z-levels precisely where localized shallow shelves or fillets widen!
  - Ensure helical/smooth ramp transitions between Z-levels stay strictly outside `avoid_curr` with zero vertical witness marks.
- **Tighter `Adaptive3D` Fine Step-Ups & Rest Coverage (`adaptive3d.rs`):**
  - Lower the minimum clearable band threshold cleanly for narrow fine step-up terraces (avoiding missed upper dome rings) and ensure `detect_flats` accurately aligns Z-levels to horizontal shelves within `0.005 mm`.
- **Sub-Cell Ridge Centering in `Pencil3D` (`pencil3d.rs`):**
  - After priority-ordered thinning extracts concave valley centerlines, apply local 1D quadratic sub-cell refinement perpendicular to the valley tangent using exact `drop_cutter_z` + morphological residual, snapping each pencil vertex onto the true continuous bi-tangent crease rather than quantized grid-cell centers (eliminating jagged zig-zag pencil passes along diagonal fillets!).

---

## Verification Plan

1. **Zero-Gouge & Quality Improvement Tests (`chip-cam`, `chip-sim`):**
   - Sinusoidal relief wave (`09-3d-finishing`) Tri-Dexel oracle test asserting **`gouge_volume_mm3 <= 1.0 mm³`**, **`max_gouge_mm <= 0.015 mm`** (down from `65.1 mm³` / `-0.146 mm`!), and improved `RMS` & `In-Tol %`.
   - Sculpted dome (`08-mesh-surfacing` & `08-3d-adaptive-wl`) Tri-Dexel oracle tests confirming zero critical errors, zero target gouges, and improved surface `RMS` / `In-Tol %`.
   - Diagonal concave fillet `Pencil3D` unit test verifying sub-cell ridge centering tracks the exact analytical fillet crease within `< 0.05 mm` lateral error.
   - Verify **100% byte-for-byte parity** on all Phase 1 & Phase 2 golden G-code snapshots!
2. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-stock --features gpu-compute`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-sim --features gpu-compute`
