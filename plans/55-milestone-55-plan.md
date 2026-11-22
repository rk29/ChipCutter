# Milestone 55 Implementation Plan — GUI/CLI Backend Controls, Benchmarks, Visual Parity, & Spec Sync

## Goal

Expose clean user-facing controls and observability for the `wgpu` Compute backend in both `chip-gui` and `chipcutter` CLI (`--backend <auto|gpu|cpu>` and `--benchmark`), run head-to-head CPU-vs-GPU performance and visual parity verification across all three stock engines (Heightfield, Tri-Dexel, Cylindrical) at Standard/Fine/Ultra resolutions, and synchronize `specs/01-architecture.md`, `specs/06-simulation.md`, and `specs/10-roadmap.md` to close out Phase 15 and hand off cleanly to Phase 16 (Toolpath Algorithm Quality).

---

## Proposed Changes

### 1. GUI Backend Selector, Adapter Badge, & Preferences (`chip-gui`)

#### [MODIFY] `crates/chip-gui/src/prefs.rs`
#### [MODIFY] `crates/chip-gui/src/panels/simulation/mod.rs`
#### [MODIFY] `crates/chip-gui/src/panels/simulation/ui.rs`
#### [MODIFY] `crates/chip-gui/src/app/mod.rs`
- Persist `sim_backend: SimComputeBackendChoice` (`Auto` default, `Gpu`, `Cpu`) in `prefs.toml`.
- Add a compact backend selector and status pill in the **Simulation Timeline** toolbar (with clear hover tooltips consistent with Phase 12's universal tooltip standard):
  - Displays active backend & adapter (e.g., `GPU · Radeon 780M` or `CPU (Exact)` / `CPU Fallback`).
  - Switching between `Auto (GPU)` and `CPU (Exact)` triggers a clean simulation refresh so machinists (and developers debugging Phase 16 toolpaths) can cross-check GPU vs. CPU results in one click.
- Add headless CLI capture flag `--capture-sim-backend <auto|gpu|cpu>` on `chip-gui` for deterministic automated visual parity testing.

---

### 2. CLI `--backend` & `--benchmark` Reporting (`chip-cli`)

#### [MODIFY] `crates/chip-cli/src/main.rs`
#### [MODIFY] `crates/chip-cli/src/commands/simulate.rs` (or `commands/mod.rs` simulation command)
- Extend `chipcutter simulate` with:
  - `--backend <auto|gpu|cpu>` (default `auto`: uses GPU compute when available, transparently falling back to `CpuExactBackend`).
  - `--benchmark` flag printing structured timing & throughput metrics (and embedding them into `--json` output):
    - `backend` (`gpu` / `cpu`), `adapter_name`, `backend_api` (`Vulkan` / `Metal` / `DX12` / `CPU`)
    - `resolution`, `engine`, `total_segments`, `target_setup_ms`, `sweep_ms`, `metrics_ms`, `total_ms`, `segments_per_sec`
    - Full `StockErrorReport` (`total_error_volume_mm3`, `excess_volume_mm3`, `gouge_volume_mm3`, `rms_error_mm`, `p95_abs_error_mm`, `p99_abs_error_mm`, `in_tolerance_surface_pct`, per-operation error progression).
- Designed directly as the repeatable benchmark harness for **Phase 16** toolpath algorithm quality work (including 4th-axis rotary Ultra benchmarks).

---

### 3. Visual Parity, Speedup Benchmarks, & Architecture Spec Synchronization

#### [MODIFY] `specs/01-architecture.md`
#### [MODIFY] `specs/06-simulation.md`
#### [MODIFY] `specs/10-roadmap.md`
- **Headless Visual Parity Smoke Suite:**
  - Capture side-by-side CPU vs. GPU Vulkan screenshots (to `/tmp/`, never inside the repo) using `WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -a` across:
    1. `cam-torture` (`heightmap`, `Color by Op` + `Error Heatmap`)
    2. `thread-mill` (`tri-dexel`, Ultra/Fine undercut inspection)
    3. `rotary-4axis` (`cylindrical`, Ultra + `Error Heatmap`)
  - Visually inspect all screenshots via `read_file` to verify crisp rendering, exact normal/lighting/color parity, and zero blank/lagged frames.
- **CPU vs. GPU Benchmark Run:**
  - Run `chipcutter simulate --benchmark` at `Standard`, `Fine`, and `Ultra` across Heightfield, Tri-Dexel, and 4th-axis Cylindrical samples comparing `--backend cpu` vs. `--backend gpu`, recording verified speedup numbers and confirming identical diagnostics + matching `StockErrorReport` metrics.
- **Documentation Sync:**
  - Update `specs/06-simulation.md` to document Tri-Dexel, 4th-axis Cylindrical, `SimStockBackend`, and `WgpuComputeBackend` GPU compute + direct GPU rendering.
  - Update `specs/01-architecture.md` to document the `gpu-compute` feature and `SimStockRenderSource::GpuGrid`.
  - Mark **Phase 15 (Completed — M52–M55)** in `specs/10-roadmap.md`.

---

## Verification Plan

1. **CLI & GUI Integration Tests (`chip-cli`, `chip-gui`):**
   - `chipcutter simulate --backend cpu --json` vs. `--backend gpu --json` parity test verifying matching error metrics and identical Rule 1–5 crash/gouge results.
   - `prefs.toml` roundtrip test for `sim_backend`.
   - Zero `.chip.toml` schema drift and byte-for-byte golden G-code parity across all `crates/chip-cli/tests/golden/` snapshots.
2. **Full Workspace Parity & Release-Readiness Gate:**
   - `cargo fmt --all -- --check`
   - `cargo check --locked -p chip-stock` (default pure-CPU feature — zero `wgpu`)
   - `cargo deny check`
   - `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
