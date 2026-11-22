# Milestone 56 Implementation Plan — 4th-Axis Rotary Toolpath Quality & High-Fidelity Cylindrical Verification

## Goal

Eliminate the visible spiral ridges, staircase excess bands, off-centerline chord artifacts, and simulation pose beading observed on ChipCutter's 4th-axis rotary samples even at Ultra resolution—driving quantitative Ultra `CylindricalStock` accuracy from the v0.15.0 baseline (`12-simultaneous-4axis`: `RMS 0.529 mm`, `In-Tol 17.8%`; `09-rotary-shaft`: `RMS 0.884 mm`, `In-Tol 34.0%`; `10-rotary-axial`: `RMS 2.291 mm`, `In-Tol 16.1%`) to crisp, smooth, gouge-free rotary surfaces (`RMS <= 0.08 mm`, `In-Tol >= 80–90%`, `max_gouge <= 0.015 mm` on machined zones), while fixing STEP indexed target-surface metrics (`11-indexed-step`).

---

## Baseline Diagnosis (Measured on v0.15.0 GPU Ultra)

| Sample | Baseline Err Vol (`mm³`) | Baseline RMS (`mm`) | Baseline Max Ex / Gg (`mm`) | Baseline In-Tol (`±0.05 mm`) | Root Causes Identified in Source |
|---|---:|---:|---|---:|---|
| `12-simultaneous-4axis` | 3,735.9 | 0.5291 | +1.111 / -0.005 | 17.8% | `Rotary4AxisGrid::sample_safe` (`rotary4axis_sim.rs:476`) lifts `z` to `z_max + 0.06 * c_spread` whenever `z_max - z_bi > 0.008` or `c_spread > 0.10`; coarse `(0.40 mm × 2.0°)` grid interpolation instead of exact direct drop-cutter; fixed `1.5 mm` stepover without flank-slope cusp compensation; `CylindricalStock` `0.75°` pose step beading. |
| `09-rotary-shaft` | 5,090.5 | 0.8843 | +6.000 / -0.014 | 34.0% | `RotaryRadiusGrid::sample_safe` (`rotary2linear.rs:280`) lifts to `r_max` whenever `r_max - r_bi > 0.015`; coarse `1.6 mm` spiral stepover + `2.5°` angular sampling; axial end margins leave uncut collar shoulders. |
| `10-rotary-axial` | 13,454.6 | 2.2914 | +6.000 / -0.019 | 16.1% | Same `RotaryRadiusGrid::sample_safe` staircase lift + `1.6 mm` spiral stepover and `2.5°` sampling. |
| `11-indexed-step` | 23,480.7 | 0.0000* | 0.000* | 100.0%* | Polar target-mesh rasterization / surface-metric exclusion edge case on STEP solids in `CylindricalStock` (`r_target` / `metrics.rs`). |

---

## Proposed Changes

### 1. Exact Direct Drop-Cutter & Adaptive Chord-Sagitta Subdivision (`rotary4axis_sim.rs`, `rotary2linear.rs`, `rotary.rs`)

#### [MODIFY] `crates/chip-cam/src/rotary.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/rotary4axis_sim.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/rotary2linear.rs`
- **Remove Staircase `sample_safe` Hacks:**
  - Replace coarse grid-corner `z_max + 0.06 * c_spread` (`rotary4axis_sim.rs`) and `r_max` (`rotary2linear.rs`) staircase lifting with **exact `f64` direct drop-cutter evaluation** along the emitted path points `(u(k), θ(k), c_cross(u, θ))` in parallel via `rayon` (leveraging `chip-bvh`'s `O(log N)` flat SAH BVH and a reusable per-thread rotated-mesh / local-frame query helper in `rotary.rs`—avoiding full-mesh rebuilds when rotating query rays/cutter frames into part space!).
  - **Key Speedup & Exactness Insight:** Instead of rotating the entire triangle mesh `720+` times per operation (`build_rotated_strip_mesh` per angular station), provide an exact `drop_cutter_rotary_at(base_mesh, rot, cutter, u, c_cross, theta_deg, z_floor, scratch)` helper that queries `base_mesh`'s existing `FlatBvh` directly (rotating only the local cutter search cylinder / candidate triangles in the narrow strip around `(u, θ, c_cross)`). This allows evaluating **exact `f64` drop-cutter at arbitrary `(u, θ, c_cross)`** during both curve sampling AND recursive chord midpoint refinement!
- **Adaptive Gouge-Free Chord-Midpoint Subdivision (`refine_rotary_chords`):**
  - After evaluating exact `(u, θ, c_cross, z_rel)` along each spiral/flowline/axial-parallel strip, recursively check the midpoint `(u_mid, θ_mid, c_mid)` of every chord whose angular span `> 0.5°`, radial/Z change `> 0.02 mm`, or cross-axis change `> 0.05 mm` (up to depth 4).
  - Whenever exact `z_exact(mid)` differs from linear chord interpolation `z_interp` by `> max_sag_tol` (default `0.004 mm` for convex peaks to prevent ANY chord gouging, and `<= smooth_tol` for concave valleys to eliminate faceting!), insert the exact midpoint!
  - Update `decimate_4axis_strip` and `decimate_rotary_strip` to strictly enforce `max_convex_dip <= 0.002 mm` and smooth angular limits (`<= 1.5°` on curved profiles).
- **Slope-Compensated Cusp-Height Stepover & Clean Off-Centerline Tilt:**
  - In `compute_cross_offset_field` (`rotary4axis_sim.rs`): compute smooth, C¹-continuous cross-axis tilt `c_cross(u, θ)` from exact circumferential surface gradients, then evaluate **exact off-centerline drop-cutter** at the final smoothed `c_cross(u, θ)` (eliminating tilt-vs-contact mismatch that caused both excess ridges and local flank gouges).
  - Add slope-compensated axial stepover scaling when `use_cusp_height` is enabled (accounting for helical flank axial slope `cos(φ)` so steep helical flutes achieve the same cusp height as shallow lands!), and tighten default `angular_sample_deg` (`1.5°` max) and `axial_sample_step` (`0.25 mm` max) bounds.

---

### 2. High-Fidelity Cylindrical Simulation Sweeper & Indexed STEP Metrics Fix (`chip-stock`, `chip-sim`)

#### [MODIFY] `crates/chip-stock/src/cylindrical/sweep.rs`
#### [MODIFY] `crates/chip-stock/src/gpu/cylindrical.rs`
#### [MODIFY] `crates/chip-stock/src/metrics.rs`
#### [MODIFY] `crates/chip-sim/src/cad_adapter.rs`
- **Eliminate Cylindrical Pose Beading at Fine/Ultra:**
  - Tighten rotary pose subdivision in both CPU (`cylindrical/sweep.rs`) and GPU (`gpu/cylindrical.rs`) from `step_deg = (0.5 * dtheta).max(0.75)` to resolution-scaled `step_deg = (0.5 * dtheta).clamp(0.15, 0.35)` and `step_u = (0.5 * du).clamp(0.05, 0.12)` (with adaptive sub-step cap scaling safely with chord length/rotation). At Ultra (`ntheta = 720`, `dtheta = 0.5°`), `step_deg = 0.25°` cuts circumferential pose stamp spacing by **3×** (`0.275 mm -> 0.09 mm` at `r = 21 mm`), eliminating visible simulation beading while remaining super fast on Phase 15's GPU sweeper!
- **Fix `11-indexed-step` Polar Target Rasterization & Error Metrics:**
  - Audit why `11-indexed-step` reported `errVol = 23,480.7 mm³` alongside `RMS = 0.0000`, `inTol = 100.0%` (`extract_cylindrical_target_mesh_from_cad` / `compute_cylindrical_error_report` / `stock_metrics.wgsl` surface-sample inclusion condition when stock is a `FixedBox` bar or STEP faces sit inside/outside `r_target`), fix both CPU and GPU reductions so `RMS`, `MAE`, `P95/P99`, `max_excess`, and `in_tolerance_surface_pct` match accurately on STEP rotary setups, and add a unit test!

---

## Verification Plan

1. **4th-Axis Accuracy & Gouge-Freedom Unit/Integration Tests (`chip-cam`, `chip-sim`, `chip-stock`):**
   - Analytical cam/flute rotary benchmark asserting zero staircase bias (`|mean_bias| < 0.03 mm`), zero target gouges (`max_gouge <= 0.015 mm`), and `> 85%` in-tolerance surface on machined zones across both `Rotary4AxisSim` and `Rotary2Linear`.
   - Exact CPU-vs-GPU differential parity test confirming updated `CylindricalStock` pose subdivision & fixed STEP indexed metrics match across `CpuExactBackend` and `WgpuComputeBackend`.
   - `11-indexed-step` target rasterization & non-zero `RMS` / `in_tolerance_surface_pct` regression test.
2. **Headless Vulkan Visual Verification:**
   - Capture Ultra GPU + Error Heatmap screenshots of `rotary-4axis` and `rotary-shaft` to `/tmp/m56_*.png` and inspect via `read_file` to confirm smooth, ridge-free, gouge-free machined surfaces.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-stock --features gpu-compute`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-sim --features gpu-compute`
