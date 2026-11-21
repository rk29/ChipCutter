# Milestone 40 Implementation Plan — GUI Engine Selector, Error-Metrics Dashboard, & Illustrated Tutorial 17

## Goal

Expose the **Heightmap vs. Tri-Dexel** engine selector, live **Deviation Error Heatmap** visualization, and interactive **Error Volume & RMS Metrics Dashboard** inside `chip-gui`, and publish **Tutorial 17** showing machinists how to inspect tri-dexel volumetric cuts, interpret error volume/RMS reports, and use quantitative metrics to evaluate and tune toolpath strategies.

---

## Proposed Changes

### 1. Simulation Toolbar Engine Selector & Deviation Heatmap (`chip-gui`)

#### [MODIFY] [crates/chip-gui/src/panels/simulation.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/simulation.rs)
#### [MODIFY] [crates/chip-gui/src/prefs.rs](file:///workspace/ChipCutter/crates/chip-gui/src/prefs.rs)
#### [MODIFY] [crates/chip-gui/src/capture.rs](file:///workspace/ChipCutter/crates/chip-gui/src/capture.rs)
- Add a compact **Engine** dropdown (`Heightmap (Fast)` | `Tri-Dexel (Volumetric 3D)`) next to the resolution selector (`Draft` / `Standard` / `Fine`) in the Simulation Timeline toolbar, persisted in `prefs.toml` (defaulting to `Heightmap` for instant startup responsiveness, with 1-click switch to `Tri-Dexel`).
- Add a **`Color by Op` / `Error Heatmap`** toggle alongside an inline tolerance selector (`±0.05 mm` default, editable) and a crisp color-bar legend (`-Red Gouge | Green ±Tol | +Blue Excess`).
- Extend headless screenshot capture flags (`--capture-sim-engine <heightmap|tri-dexel>`, `--capture-sim-heatmap`) for deterministic Tutorial 17 captures.

---

### 2. Quantitative Error-Volume & RMS Dashboard (`chip-gui`)

#### [MODIFY] [crates/chip-gui/src/panels/simulation.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/simulation.rs)
#### [MODIFY] [crates/chip-gui/src/panels/diagnostics.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/diagnostics.rs)
- Display a compact, readable **Stock-vs-Target Error Metrics** summary card whenever a simulation completes on a 3D target model:
  - **Total Error Volume** (`mm³` & `% of target`), **Excess (Uncut) Volume**, **Gouge Volume**
  - **RMS Error**, **MAE**, **Mean Signed Bias**, **P95 / P99 Error**, **In-Tolerance Surface %**
  - **Max Excess** and **Max Gouge** with 1-click **`Locate`** buttons that frame the 3D viewport camera directly on the worst uncut corner or overcut location!
  - Expandable **Per-Operation Error Progression** mini-table showing how much error volume and RMS each operation removed—making it effortless to evaluate stepover, rest-machining, or strategy changes directly inside the app.

---

### 3. Illustrated Tutorial 17 & Roadmap Completion (`docs/tutorials/`, `README.md`, `specs/10-roadmap.md`)

#### [NEW] `docs/tutorials/17-tridexel-simulation-error-metrics.md`
#### [NEW] `docs/images/tutorials/17-tridexel-simulation-error-metrics/*.png`
#### [MODIFY] [docs/tutorials/16-model-orientation-positioning.md](file:///workspace/ChipCutter/docs/tutorials/16-model-orientation-positioning.md)
#### [MODIFY] [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md)
#### [MODIFY] [README.md](file:///workspace/ChipCutter/README.md)
#### [MODIFY] [specs/10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md)
- Publish **Tutorial 17: Tri-Dexel Volumetric Simulation & Quantitative Error-Volume Analysis**, following the 7-part tutorial standard with visually verified headless Vulkan screenshots:
  1. Comparing **Heightmap (Fast)** vs. **Tri-Dexel (Volumetric 3D)** on vertical walls, deep pockets, and 3D sculpted domes.
  2. Reading the **Error Heatmap** (`Blue = Excess`, `Green = In-Tolerance`, `Red = Gouge`) and jumping to worst excess/gouge spots.
  3. Interpreting **Total Error Volume**, **RMS**, **MAE**, **P95/P99**, and **Per-Operation Error Progression** to compare roughing vs. finishing stepovers and strategies (`Parallel3D` vs. `Scallop3D` + `Pencil3D`).
  4. Exporting machine-readable `StockErrorReport` JSON via `chipcutter simulate --engine tri-dexel --report` for repeatable toolpath algorithm benchmarking.
- Mark **Phase 11 (`M38`–`M40`)** completed in `specs/10-roadmap.md`.

---

## Verification Plan

1. **GUI & Headless Vulkan Verification:**
   - Capture & visually inspect all Tutorial 17 screenshots via `WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -a` + `read_file`.
   - Verify Markdown/PNG link audit (`0` broken links, `0` orphaned PNGs).
2. **Full Workspace Gate:**
   - `cargo fmt --all -- --check`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
