# Milestone 39 Implementation Plan — Quantitative Error-Volume Metrics, Deviation Heatmaps, & Algorithm Benchmarking

## Goal

Compute industry-standard **stock-vs-target quantitative error metrics** (total error volume, excess/gouge volume, RMS deviation, MAE, signed bias, P95/P99 percentiles, max excess/gouge with 3D coordinates, in-tolerance surface %, and per-operation error progression) across **both** Tri-Dexel and Heightmap simulations—plus deviation-heatmap vertex coloring and headless `chipcutter` CLI JSON reporting to guide future toolpath algorithm improvements.

---

## Proposed Changes

### 1. Stock-vs-Target Error-Volume & Surface Deviation Metrics (`chip-sim`)

#### [NEW] `crates/chip-sim/src/metrics.rs`
#### [MODIFY] [crates/chip-sim/src/heightmap.rs](file:///workspace/ChipCutter/crates/chip-sim/src/heightmap.rs)
#### [MODIFY] [crates/chip-sim/src/lib.rs](file:///workspace/ChipCutter/crates/chip-sim/src/lib.rs)
- Define serializable `StockErrorReport` attached to `SetupSimulationResult` (always computed when a 3D STEP/mesh target model is present):
  - **Volumetric Error Metrics (`mm³` & `%`):**
    - `target_volume_mm3: f64` — reference target solid volume
    - `excess_volume_mm3: f64` — total uncut material remaining outside the target solid (`V(Machined \ Target)`)
    - `gouge_volume_mm3: f64` — total overcut/gouged material missing from inside the target solid (`V(Target \ Machined)`)
    - `total_error_volume_mm3: f64` — symmetric difference volume (`excess_volume_mm3 + gouge_volume_mm3`)
    - `error_volume_pct_of_target: f64` — `100.0 * total_error_volume_mm3 / target_volume_mm3`
    - `material_removal_efficiency_pct: f64` — fraction of intended removable stock (`Initial \ Target`) actually removed
  - **Surface Deviation Statistics (`mm`):**
    - `rms_error_mm: f64` — Root-Mean-Square deviation (`sqrt(mean(d_i²))`) over all sampled target surface points (plus `excess_rms_mm` and `gouge_rms_mm`)
    - `mean_abs_error_mm: f64` — Mean Absolute Error (`MAE = mean(|d_i|)`)
    - `mean_signed_error_mm: f64` — signed mean bias (`+` = oversize/uncut, `-` = undersize/overcut)
    - `p95_abs_error_mm: f64` and `p99_abs_error_mm: f64` — 95th and 99th percentile absolute surface errors (robust against isolated unmachinable inner-corner fillets)
    - `max_excess_mm: f64` + `max_excess_location_wcs: Option<DVec3>` — worst uncut scallop/corner height & exact 3D position
    - `max_gouge_mm: f64` + `max_gouge_location_wcs: Option<DVec3>` — deepest overcut/gouge & exact 3D position
    - `tolerance_mm: f64` (configurable, default `0.05 mm`) & `in_tolerance_surface_pct: f64` — percentage of target surface area within `±tolerance_mm`
  - **Per-Operation Error Progression (`Vec<OperationErrorSnapshot>`):**
    - After each operation completes, record `op_id`, `op_name`, `removed_volume_mm3`, `remaining_excess_volume_mm3`, `gouge_volume_mm3`, `total_error_volume_mm3`, and `rms_error_mm`.
- Implement exact symmetric-difference interval computation on the shared Tri-Dexel ray lattice (`excess` = intervals in `Machined \ Target`, `gouge` = intervals in `Target \ Machined`, averaged across X/Y/Z axes) AND compatible top-projected Z-column error metrics on `HeightfieldStock` so both engines report the exact same `StockErrorReport` schema!

---

### 2. Signed Surface-Deviation Heatmap Coloring (`chip-sim`)

#### [MODIFY] `crates/chip-sim/src/tridexel/mesh.rs`
#### [MODIFY] [crates/chip-sim/src/heightmap.rs](file:///workspace/ChipCutter/crates/chip-sim/src/heightmap.rs)
- Add `SimStockColorMode { ByOperation, ErrorHeatmap { tolerance_mm: f64, max_scale_mm: f64 } }` to both mesh tessellators.
- In `ErrorHeatmap` mode, color each machined stock vertex using a crisp perceptual diverging ramp:
  - **Green:** within `±tolerance_mm` of target surface
  - **Cyan → Deep Blue:** positive signed deviation (uncut excess stock / scallops / rest corners)
  - **Yellow → Red:** negative signed deviation (overcut / gouge into target part)

---

### 3. Headless CLI Engine Selection & Benchmark Reporting (`chip-cli`)

#### [MODIFY] [crates/chip-cli/src/main.rs](file:///workspace/ChipCutter/crates/chip-cli/src/main.rs)
#### [MODIFY] [crates/chip-cli/src/commands/simulate_build.rs](file:///workspace/ChipCutter/crates/chip-cli/src/commands/simulate_build.rs)
- Add `--engine <heightmap|tri-dexel>` (default `heightmap`) and `--tolerance <mm>` (default `0.05`) flags to `chipcutter simulate`.
- Print a clean summary table of **Total Error Volume**, **Excess / Gouge Volume**, **RMS Error**, **MAE**, **P95 / P99**, **Max Excess / Gouge**, and **In-Tolerance %** to stdout, and include the full `StockErrorReport` (including per-operation progression) in `--report <sim_report.json>`.
- Add a headless CLI end-to-end test verifying deterministic JSON `StockErrorReport` output across both `--engine heightmap` and `--engine tri-dexel`, plus a regression benchmark test confirming finishing operations strictly reduce `total_error_volume_mm3` and `rms_error_mm` compared to roughing alone!

---

## Verification Plan

1. **Analytical Error-Volume & RMS Unit Tests (`chip-sim`):**
   - Known `1.0 mm` uniform stock-to-leave offset over a `40 × 30 × 10 mm` block produces `rms_error_mm == 1.0 ± 0.02 mm`, `mean_signed_error_mm == +1.0 ± 0.02 mm`, `gouge_volume_mm3 == 0.0`, and exact analytical excess volume.
   - Known deliberate `0.5 mm` overcut slot produces exact analytical `gouge_volume_mm3`, `max_gouge_mm ≈ 0.5 mm`, and negative `mean_signed_error_mm`.
   - Adding a finishing pass after a roughing pass strictly lowers `total_error_volume_mm3`, `excess_volume_mm3`, and `rms_error_mm`.
2. **Quality Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo test --locked -p chip-sim -p chip-project -p chip-cli`
