# Milestone 69 Implementation Plan — Illustrated Tutorial 20: GPU-Accelerated Simulation, Benchmarking, & Feedrate Optimization

## Goal

Publish **Tutorial 20 (`docs/tutorials/20-gpu-simulation-feed-optimization.md`)**—a complete, 7-part illustrated walkthrough covering **Phase 15's `wgpu` Compute Simulation & Direct GPU Stock Rendering** and **Phase 16's Engagement-Aware Feedrate Optimization & Surface Finish Quality Tuning**—backed by deterministic headless Vulkan screenshots under `docs/images/tutorials/20-gpu-simulation-feed-optimization/` and repeatable capture automation in `scripts/capture_tutorial_screenshots.sh`.

---

## Proposed Changes

### 1. Illustrated Tutorial 20 (`docs/tutorials/20-gpu-simulation-feed-optimization.md`)

#### [NEW] `docs/tutorials/20-gpu-simulation-feed-optimization.md`
#### [MODIFY] `docs/tutorials/17-tridexel-simulation-error-metrics.md` (add concise cross-link tip pointing to Tutorial 20 for GPU acceleration & benchmarking)
#### [MODIFY] `docs/tutorials/README.md` (add Tutorial 20 row & `20-feed-opt-pocket.step` starter asset entry)
- Follow ChipCutter's standardized 7-part tutorial layout (using installed `chip-gui` and `chipcutter` commands):
  1. **Goal & What You'll Learn:** Accelerating Ultra Heightmap, Tri-Dexel, and 4th-Axis Cylindrical simulations via `wgpu` Compute (`Auto (GPU)` / `GPU` / `CPU`), interpreting stage-timing breakdowns, instant GPU Error Heatmap tuning, and optimizing cutting feedrates with curvature/engagement-aware modulation (`FeedOptimizationConfig`).
  2. **Prerequisites & Starter Project:** Opening `--sample feed-opt-pocket` (or starting from `docs/tutorials/assets/20-feed-opt-pocket.step` and `--sample rotary-4axis` for Ultra GPU benchmarking).
  3. **Step-by-Step GUI Walkthrough (6+ annotated Vulkan screenshots):**
     - **Step 1 — Selecting the GPU Compute Backend:** Simulation Timeline toolbar `Backend` dropdown (`Auto (GPU)` vs. `GPU` vs. `CPU`), reading the live adapter pill (`GPU · <Adapter>` vs. `CPU (Exact)` / `CPU Fallback`) and hovering for `target_setup_ms`, `sweep_ms`, `metrics_ms`, and `segments/sec` stage timings.
     - **Step 2 — Direct GPU Stock Rendering & Live Error Heatmap Slider:** Switching to `Tri-Dexel (Fine)` / `Cylindrical (Ultra)` and toggling `Error Heatmap` + adjusting `±tol` smoothly in `< 0.1 ms` with zero CPU re-meshing (and verifying isolated background compute stays responsive during window resize!).
     - **Step 3 — Enabling Engagement-Aware Feedrate Optimization:** Opening the Operation Inspector **`Passes`** tab → **`Feed Optimization`** card, enabling **Internal-Arc Perimeter-to-Centerline Scaling** (`F_center = F_nom * (R_path - R_tool) / R_path` with min-feed clamp), **Sharp-Corner Lookahead Deceleration Ramps** (angle threshold, ramp length, corner feed fraction), and **Radial Chip-Thinning Boost** (max boost factor).
     - **Step 4 — Inspecting Modulated Feedrate Telemetry:** Regenerating (`Ctrl+G`) and reading `SimulationTimeline`'s live `Min / Avg / Max` modulated cutting feedrates and per-segment feed readout while scrubbing through sharp concave pocket corners and small-radius internal arcs.
     - **Step 5 — Tuning Surface Finish Quality with Ultra Error Heatmap:** Comparing fixed stepover vs. cusp-controlled finishing on curved 3D/4th-axis geometry using `In-Tol %`, `RMS`, and `Max Gouge` metrics.
  4. **Key Parameters Reference:** Two concise reference tables covering (a) Simulation Backend & Resolution controls (`Auto`/`GPU`/`CPU`, `Standard`/`Fine`/`Ultra`, VRAM checkpoint behavior) and (b) `FeedOptimizationConfig` parameters with safe starting ranges for aluminum, hardwood, and plastics.
  5. **Generate (`Ctrl+G`) & Verification Gate:** Confirming `0` critical errors, `max_gouge <= 0.02 mm`, and byte-identical geometry with modulated feedrates.
  6. **Post-Processing (`GRBL` & `LinuxCNC`):** Inspecting emitted `F` words decelerating smoothly into internal arcs and concave corners (`G1`/`G2`/`G3`) and restoring nominal feed on straightaways.
  7. **Headless CLI Equivalent (`chipcutter`):** Running `chipcutter simulate --engine tri-dexel --backend gpu --resolution fine --benchmark --json` and comparing `--backend cpu` vs. `--backend gpu` throughput & parity tables.

---

### 2. Repeatable Headless Vulkan Screenshot Pipeline (`scripts/capture_tutorial_screenshots.sh`)

#### [MODIFY] `scripts/capture_tutorial_screenshots.sh`
#### [NEW] `docs/images/tutorials/20-gpu-simulation-feed-optimization/*.png`
- Harden `capture_shot` in `scripts/capture_tutorial_screenshots.sh` with a clean free-display picker (checking BOTH `/tmp/.X<N>-lock` AND `/tmp/.X11-unix/X<N>` sockets so stale sockets never stall `xvfb-run`!).
- Add `capture_20_gpu_feed_opt()` (`--milestone 69` / `--tutorial 20-gpu-simulation-feed-optimization`) capturing crisp `1440×900` screenshots:
  - `01-gpu-backend-toolbar.png` (Simulation Timeline showing `GPU · <Adapter>` badge + Tri-Dexel Fine stock)
  - `02-gpu-ultra-heatmap.png` (`rotary-4axis` GPU Ultra + Error Heatmap + high `In-Tol %` metrology)
  - `03-feed-opt-inspector.png` (`feed-opt-pocket` Operation Inspector with expanded **Feed Optimization** card)
  - `04-modulated-feed-timeline.png` (`SimulationTimeline` showing min/avg/max modulated feedrates and corner deceleration)
  - `05-gcode-modulated-feeds.png` (`G-Code Preview` showing corner/arc `F` feed modulation)
- Visually inspect **every single captured PNG** via `read_file` to verify crisp legibility, zero clipped labels, zero blank frames, and accurate UI state!

---

## Verification Plan

1. **Tutorial CLI Command Verification:**
   - Execute every CLI command shown in `docs/tutorials/20-gpu-simulation-feed-optimization.md` (`generate-sample-step --tutorial 20-feed-opt`, `validate`, `simulate --backend gpu --benchmark --json`, `simulate --backend cpu --benchmark`, `build --post grbl`, `build --post linuxcnc`) and assert exit code `0` and `0` critical errors.
2. **Visual & Markdown Audit:**
   - Visually inspect all captured PNGs under `docs/images/tutorials/20-gpu-simulation-feed-optimization/` via `read_file`.
   - Verify all relative image and navigation links in `docs/tutorials/20-gpu-simulation-feed-optimization.md`, `17-tridexel-simulation-error-metrics.md`, and `docs/tutorials/README.md` resolve cleanly on disk.
3. **Workspace Sanity Gate:**
   - `bash -n scripts/capture_tutorial_screenshots.sh`
   - `cargo fmt --all -- --check` (if any capture flag helper was touched)
   - `git status` clean of any stray temporary files.
