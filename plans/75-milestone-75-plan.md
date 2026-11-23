# Milestone 75 Implementation Plan — Helical-Loft 4th-Axis Rotary Improvements, Benchmark Suite, & Phase 20 Spec Sync

## Goal

Harden 4th-axis rotary toolpathing (`Rotary2Linear` and `Rotary4AxisSim`) for deeply twisted helical lofts, lock in comprehensive CPU & GPU regression gates across all Phase 20 work, and synchronize specifications:
1. **Gouge-Free Partial-Sector `ZigZag` Linking (`rotary4axis_sim.rs`, `rotary2linear.rs`):** Replace unchecked direct axial `LinkTransition` moves between partial-sector spiral lanes with `drop_cutter_rotary_z_rel`-verified link transitions so diagonal helical crests are never clipped at sector endpoints.
2. **Overhang-Aware Flank Probing for `Rotary4AxisSim` `SurfaceNormalAutoTilt` (`rotary4axis_sim.rs`, `rotary.rs`):** Upgrade cross-axis tilt computation so overhanging helical blades no longer blind auto-tilt to recessed helical groove flanks beneath the blade crest, and emit actionable warnings when axial (`u`-direction) undercuts require a `LollipopEndmill` (`Undercut3D`).
3. **Programmatic Helical-Loft Benchmark, Phase 20 E2E Regression Suite, & Spec Sync (`chip-cad`, `chip-cam`, `chip-project`, `specs/`):** Add a deterministic helical-loft multi-blade benchmark fixture and full E2E verification across 3-axis, 2.5D, indexed `3+1`, workholding, and 4th-axis strategies, plus synchronized specs (`05`, `06`, `10`).

---

## Problem Diagnosis

1. **Unchecked Partial-Sector `ZigZag` Lane Stepovers (`rotary4axis_sim.rs:861-869`, `rotary2linear.rs:500-506`):**
   - While `emit_helical_flowline` and `emit_axial_parallel` precompute exact gouge-free `lane_links` between adjacent angular lanes, the partial-sector (`end_deg - start_deg < 360°`) branch of `emit_simultaneous_spiral` and `emit_continuous_spiral` emits a single unchecked `rotary_linear_to(p0, ...)` from the end of lane `k` (`u_k`) straight to the start of lane `k+1` (`u_{k+1}`).
   - On a helical loft whose blade crest crosses diagonally between `u_k` and `u_{k+1}` at `start_deg` or `end_deg`, that single straight chord can dip right through the helical blade!
2. **Centerline-Only `SurfaceNormalAutoTilt` (`rotary4axis_sim.rs:422-502`):**
   - `compute_cross_offset_field` estimates circumferential slope strictly from centerline `r_center(u, θ)` finite differences (`dr / (r · dθ)`).
   - When a helical blade overhangs the rotary centerline, `r_center` sits on top of the overhanging crest rather than the recessed flank beneath it, so auto-tilt cannot "see" into the circumferential overhang pocket to tilt the cutter smoothly onto the true flank.
   - Additionally, axial (`u`-direction) helical undercuts cannot be reached by any vertical-axis 4th-axis endmill; notifying the user clearly prevents confusion when residual material remains under axial overhangs.

---

## Proposed Changes

### 1. Verified Partial-Sector `ZigZag` Lane Links in `Rotary2Linear` & `Rotary4AxisSim`

#### [MODIFY] `crates/chip-cam/src/strategies/rotary2linear.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/rotary4axis_sim.rs`
- **Precomputed Axial `u` Lane Links (`lane_links`):**
  - In the partial-sector (`!is_full_turn`) branch of both `emit_continuous_spiral` (`rotary2linear.rs`) and `emit_simultaneous_spiral` (`rotary4axis_sim.rs`):
    - Whenever `cut_direction == RotaryCutDirection::ZigZag` and `lane > 0`, sample the axial stepover `(u_prev -> u_cur)` at constant endpoint angle `th0` using `evaluate_and_refine_rotary_strip` / `evaluate_and_refine_4axis_strip` (`step <= 0.25 mm`, recursive convex midpoint guard `<= 4 µm`).
    - Clamp every link vertex against the active radial roughing shell (`r.max(r_ceil)` / `clamp_to_shell(lc, lz, r_ceil)`) and emit the verified multi-point `MotionRole::LinkTransition` strip instead of a single unchecked chord!
  - Keeps full `360°` continuous spirals untouched (preserving existing Phase 5 rotary golden snapshots byte-for-byte!).

---

### 2. Overhang-Aware Flank Probing for `Rotary4AxisSim` `SurfaceNormalAutoTilt`

#### [MODIFY] `crates/chip-cam/src/rotary.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/rotary4axis_sim.rs`
- **Multi-Offset Circumferential Flank Probing (`compute_cross_offset_field`):**
  - Keep centerline `r_center(u, θ)` as the baseline smooth field, and detect steep circumferential overhang zones where `|dr / (r · dθ)|` exceeds a cliff threshold (`tan(35°)`) or where the local surface normal at the centerline contact points strongly circumferentially.
  - At those cells, evaluate a small symmetric fan of candidate lateral offsets `c ∈ {-c_max, ..., +c_max}` (e.g., 5–7 samples bounded by `min(r_center, max_safe_cross)`) via `drop_cutter_rotary_z_rel` to determine which side allows the cutter to engage deeper into the circumferential flank without gouging the opposite helical blade (`r_eff = z_rel.hypot(c)` minimized while keeping `z_rel >= r_floor`).
  - Blend the probed optimal lateral offset with the slope-derived estimate and pass through `smooth_cross_offset_grid` so simultaneous `Y/X + Z + A/B` motion remains `C¹`-smooth with zero sudden cross-axis jerks!
- **Axial-Overhang Reach Diagnostic:**
  - After generating a `Rotary2Linear` or `Rotary4AxisSim` toolpath, run a fast check across `base_mesh.triangles` for significant overhanging downward-facing/axial-undercut surface area that remains occluded from all sampled `(u, c, θ)` poses; when present, append a helpful non-blocking warning in `GeneratedToolpath::warnings` noting that axial helical undercuts may require an indexed `Undercut3D` (`LollipopEndmill`) pass.

---

### 3. Programmatic Helical-Loft Benchmark, E2E Regression Suite, & Spec Sync

#### [MODIFY] `crates/chip-cad/src/lib.rs` (or `crates/chip-cad/src/mesh/samples.rs`)
#### [NEW] `crates/chip-project/tests/helical_loft_overhang_e2e.rs`
#### [MODIFY] `specs/05-operations.md`
#### [MODIFY] `specs/06-simulation.md`
#### [MODIFY] `specs/10-roadmap.md`
- **Deterministic Multi-Blade Helical Loft Fixture (`chip-cad`):**
  - Add a compact, watertight programmatic triple-helix loft mesh generator (`sample_triple_helix_loft_stl_bytes()` — ~3–5k triangles, genuine circumferential + axial overhanging twisted blades around a central hub) usable across unit and integration tests without relying on external user files.
- **Comprehensive Phase 20 E2E Regression Suite (`helical_loft_overhang_e2e.rs`):**
  - Lock down deterministic verification across all 4 milestones:
    1. **Shank & Holder Overhang Avoidance (M72):** Reduced-flute + wider-shank endmill on helical loft -> `0` `ShankRub`, `0` `HolderCollision`, `0` `TargetModelGouge`.
    2. **2.5D Overhang Guard & Avoidance (M72):** Warning emitted when disabled; `0` gouges when `avoid_model_overhangs = true`.
    3. **`Indexed3Plus1` (`0°`/`120°`/`240°`) Multi-Angle Rest & Volumetric Carryover (M73):** `0.0 mm³` gouge volume, `0` rapid-into-stock crashes, `>= 25%` air-cut reduction vs. independent raw stock, and zero false excess-stock slivers on swept Z slabs.
    4. **Overhang-Safe 3D Finishing & Soft-Jaws (M74):** `Pencil3D` suppresses false overhang-shadow creases while keeping genuine hub fillets; `Parallel3D`/`Scallop3D` finish cleanly with zero cliff-dive gouges; Auto Soft-Jaws clear 100% of the twisted helical blade envelope.
    5. **4th-Axis Partial-Sector `ZigZag` & Auto-Tilt Helical Loft (M75):** Partial-sector (`0°..180°`) `ZigZag` in `Rotary2Linear` and `Rotary4AxisSim` achieves `0` crest-clipping gouges; `SurfaceNormalAutoTilt` reduces residual excess volume on circumferential helical grooves vs. centerline-only `Rotary2Linear`.
  - Verify **CPU vs. GPU parity** on both `TriDexel` and `Cylindrical` simulation backends.
- **Specification Sync (`specs/`):**
  - Update `specs/05-operations.md` (full-assembly 3D drop/push-cutter clearance, 2.5D `avoid_model_overhangs`, indexed `3+1` volumetric carryover, overhang-cliff finishing rules, and multi-offset rotary auto-tilt).
  - Update `specs/06-simulation.md` (multi-angle indexed carryover verification notes).
  - Mark Phase 20 (`M72–M75`) complete in `specs/10-roadmap.md`.

---

## Verification Plan

1. **Partial-Sector `ZigZag` & Helical Auto-Tilt Tests (`chip-cam`, `chip-sim`):**
   - `0°..180°` partial-sector `ZigZag` `Rotary2Linear` and `Rotary4AxisSim` on `sample_triple_helix_loft_stl_bytes()`: assert **`max_gouge_mm <= 0.015 mm`** and **`0` critical collisions** in `CylindricalStock` + `TriDexelStock` verification.
   - `Rotary4AxisSim` `SurfaceNormalAutoTilt` vs. `Rotary2Linear` on twisted helical grooves: assert measurably lower excess volume and lower `RMS` error with `0` gouges and smooth bounded `c_cross` transitions.
2. **Phase 1–5 Golden Parity & Full Workspace Gate:**
   - Verify 100% byte-for-byte parity on all Phase 1–5 golden G-code snapshots.
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-stock --features gpu-compute`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-sim --features gpu-compute`
