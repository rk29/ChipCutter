# Milestone 59 Implementation Plan — Quantitative Quality Regression Suite, Sample Polish, Visual Verification, & Spec Sync

## Goal

Lock in all Milestone 56–58 toolpath quality improvements with an automated **Quantitative Quality Regression Suite** enforcing strict bounds on `RMS`, `max_gouge_mm`, `gouge_volume_mm3`, and `in_tolerance_surface_pct` across 4th-axis rotary, 3-axis surfacing, 2.5D adaptive/rest, and thread-milling benchmarks; upgrade built-in 4th-axis and 3-axis sample projects to use cusp-controlled high-quality finishing defaults; verify crisp before/after headless Vulkan screenshots (including Ultra Error Heatmaps); and synchronize `specs/05-operations.md` and `specs/10-roadmap.md` to complete Phase 16.

---

## Proposed Changes

### 1. Built-In 4th-Axis & 3-Axis Sample Quality Polish (`chip-project`, `chip-cli`)

#### [MODIFY] `crates/chip-project/src/demos/rotary_4axis.rs`
#### [MODIFY] `crates/chip-project/src/demos/surface_3d.rs`
#### [MODIFY] `crates/chip-cli/tests/golden/` (Phase 5 rotary/3D snapshots only, if intentionally improved by M56–M57; Phase 1 & Phase 2 goldens remain 100% byte-for-byte untouched!)
- Upgrade the built-in 4th-axis rotary samples (`build_rotary_helical_flute_project`, `build_rotary_shaft_preview_project`, `build_tutorial_rotary_axial_parallel_project`) so their finishing operations use `use_cusp_height = true` (e.g., `0.015–0.025 mm` cusp height) and fine angular/axial sampling—eliminating the coarse `1.5–1.6 mm` spiral ridges visible in the Sample Projects Gallery and live GUI preview while keeping cycle times realistic for benchtop CNCs!
- Enable `FeedOptimizationConfig` (`corner_slowdown = true`, `internal_arc_scaling = true`) where appropriate on new/dedicated quality showcase checks without breaking Phase 1–2 golden parity.

---

### 2. Automated Quantitative Quality Regression Suite (`chip-project`, `chip-cli`)

#### [NEW] `crates/chip-project/tests/quality_benchmarks.rs`
- Add a fast, deterministic quantitative benchmark integration test suite (running via `WgpuComputeBackend` when `gpu-compute` + Vulkan is available, with automatic exact CPU fallback) asserting hard quality gates across ChipCutter's core strategy families:
  1. **Simultaneous 4-Axis Helical Flute (`12-simultaneous-4axis`):** `0` critical errors, `max_gouge_mm <= 0.015 mm`, `rms_error_mm <= 0.12 mm` (down from `0.529 mm`!), `in_tolerance_surface_pct >= 75%` (up from `17.8%`!).
  2. **Rotary Shaft (`09-rotary-shaft`):** `0` critical errors, `max_gouge_mm <= 0.015 mm`, major reduction in `RMS` and `excess_volume_mm3` over the `0.884 mm` v0.15.0 baseline.
  3. **3D Surface Finishing (`09-3d-surface-finishing`):** `0` critical errors, **`gouge_volume_mm3 <= 1.0 mm³`**, **`max_gouge_mm <= 0.015 mm`** (eliminating the v0.15.0 `65.1 mm³` / `-0.146 mm` gouge!), and improved `RMS` & `In-Tol %`.
  4. **3D Adaptive + Parallel + Pencil (`08-mesh-surfacing` & `08-3d-adaptive-waterline`):** `0` critical errors, `max_gouge_mm <= 0.015 mm`, improved `RMS` & `In-Tol %` over `0.65 mm` / `53–54%`.
  5. **2D Adaptive + Rest Machining (`05-adaptive-rest`):** `0` critical errors, `0` gouges, `in_tolerance_surface_pct >= 98%`, reduced corner `max_excess_mm`.
  6. **Helical Thread Milling (`18-thread-milling`):** `0` critical errors, Tri-Dexel undercut volume & RMS stability.
- Print a clean summary table on test runs so regressions or future improvements are immediately visible in CI logs.

---

### 3. Headless Visual Verification, Sample Gallery Refresh, & Spec Synchronization

#### [MODIFY] `specs/05-operations.md`
#### [MODIFY] `specs/10-roadmap.md`
#### [MODIFY] `docs/images/samples/rotary-4axis/` & `docs/images/samples/rotary-shaft/` (refreshed screenshots showcasing the ridge-free 4th-axis finish)
- Capture headless Vulkan screenshots via `WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -a` across `rotary-4axis` (Ultra + Error Heatmap & Color by Op), `rotary-shaft` (Ultra), and `09-3d-surface-finishing` (Tri-Dexel + Error Heatmap) and visually inspect EVERY image with `read_file` to confirm smooth, ridge-free, gouge-free machined surfaces!
- Refresh the checked-in `rotary-4axis` and `rotary-shaft` sample gallery PNGs if the updated sample defaults noticeably improve visual quality (preserving exact gallery aspect ratios & camera views).
- Update `specs/05-operations.md` to document exact direct rotary drop-cutter + adaptive chord refinement, zero-gouge 3D convex chord refinement, local-slope `Waterline3D`, sub-cell `Pencil3D`, and `FeedOptimizationConfig` (corner slowdown & constant-chipload scaling).
- Mark **Phase 16 (Completed — M56–M59)** in `specs/10-roadmap.md` with verified before-vs-after benchmark numbers.

---

## Verification Plan

1. **Before-vs-After Benchmark Comparison (`chipcutter simulate --backend gpu --benchmark --json`):**
   - Run the exact same 8-sample GPU Ultra/Fine benchmark table from the v0.15.0 baseline and verify dramatic measurable improvements across 4th-axis rotary, 3-axis surface finishing, and 2.5D adaptive/rest with zero new crashes or gouges.
2. **Full Workspace Parity & Release-Readiness Gate:**
   - `cargo fmt --all -- --check`
   - `cargo check --locked -p chip-stock` (pure-CPU default, zero `wgpu`)
   - `cargo deny check`
   - `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-stock --features gpu-compute`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-sim --features gpu-compute`
