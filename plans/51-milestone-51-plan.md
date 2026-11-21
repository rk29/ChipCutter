# Milestone 51 Implementation Plan — Oversized Module Decomposition, Off-Thread Stock Remeshing, & Architecture Spec Sync

## Goal

Eliminate the remaining UI-thread frame stall during high-resolution stock remeshing (`AGENTS.md` Invariant #6), break up oversized >1,100-LOC source files across `chip-cam`, `chip-tools`, `chip-project`, `chip-cli`, and `chip-gui` into focused, maintainable submodules (zero behavior or schema changes), and synchronize `specs/01-architecture.md` and `specs/10-roadmap.md` with the refactored 13-crate workspace architecture.

---

## Proposed Changes

### 1. Non-Blocking Off-Thread Stock Remeshing & `SimStockRenderSource` (`chip-gui`)

#### [MODIFY] [crates/chip-gui/src/viewport/scene_builders.rs](file:///workspace/ChipCutter/crates/chip-gui/src/viewport/scene_builders.rs) -> `viewport/{mod.rs, highlight.rs, gizmos.rs, toolpath_instances.rs}`
#### [MODIFY] [crates/chip-gui/src/panels/simulation.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/simulation.rs) -> `panels/simulation/{mod.rs, worker.rs, playback.rs, ui.rs, metrology_ui.rs}`
- Introduce `SimStockRenderSource` in `chip-gui/src/viewport` (`CpuMesh { revision, mesh: Arc<SimStockMesh>, stock_model: DMat4 }` now, with clean room for `GpuGrid` when the optional `gpu-compute` backend lands in a future phase).
- Move `SimulatedStock::sync_to_scrub_snapshot` + `tessellate_stock_mesh_colored` off the `egui` UI thread onto a lightweight background remesh worker channel (coalescing rapid scrubber drags so only the latest requested timestamp/color-mode is meshed, while retaining synchronous deterministic completion in headless `--capture-*` mode!).
- Split `panels/simulation.rs` (2,012 LOC) cleanly into `panels/simulation/{mod.rs, worker.rs, playback.rs, ui.rs, metrology_ui.rs}`.

---

### 2. Clean Submodule Decomposition (Zero API / Schema / Behavior Drift)

Split the largest production files identified in the post-M46 audit while keeping all public module re-exports intact:

- **`chip-cam`:**
  - `crates/chip-cam/src/params.rs` (1,992 LOC) -> `params/{mod.rs, common.rs, milling_2d.rs, surface_3d.rs, rotary.rs, thread_mill.rs}`
  - `crates/chip-cam/src/selection.rs` (1,383 LOC) -> `selection/{mod.rs, brep.rs, sketch.rs}`
- **`chip-tools`:**
  - `crates/chip-tools/src/builtin_machines.rs` (2,537 LOC) -> `builtin_machines/{mod.rs, generic.rs, milo.rs, voron_cascade.rs, carvera.rs}`
- **`chip-project`:**
  - `crates/chip-project/src/demos.rs` (3,952 LOC) -> `demos/{mod.rs, milling_2d.rs, surface_3d.rs, rotary_4axis.rs, tutorials_2d.rs, tutorials_3d.rs}`
- **`chip-cli`:**
  - `crates/chip-cli/src/commands/inspect_sample.rs` (1,142 LOC) -> `commands/{generate_sample.rs, import_mesh.rs, import_2d.rs, transform_model.rs, inspect.rs}`
- **`chip-gui`:**
  - `crates/chip-gui/src/panels/setup_inspector.rs` (2,053 LOC) -> `panels/setup_inspector/{mod.rs, wcs_tab.rs, stock_tab.rs, heights_tab.rs, rotary_tab.rs}`
  - `crates/chip-gui/src/panels/tool_library.rs` (1,634 LOC) -> `panels/tool_library/{mod.rs, geometry_editor.rs, presets_editor.rs}`
  - `crates/chip-gui/src/app/menus_toolbar.rs` (1,590 LOC) -> `app/{shortcuts.rs, commands.rs, menus.rs, toolbar.rs}`
  - `crates/chip-gui/src/app/loader.rs` (1,745 LOC) -> `app/loader/{mod.rs, sample_loaders.rs, cad_apply.rs}`
  - `crates/chip-gui/src/app/mod.rs` (1,348 LOC) -> group modal states into `ModalStates` struct in `app/modals.rs`

---

### 3. Architecture Spec Synchronization

#### [MODIFY] [specs/01-architecture.md](file:///workspace/ChipCutter/specs/01-architecture.md)
#### [MODIFY] [specs/10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md)
- Update `specs/01-architecture.md` with the new 13-crate workspace layout (`chip-math`, `chip-bvh`, `chip-geom2d`, `chip-tools`, `chip-cad`, `chip-sketch`, `chip-cam`, `chip-stock`, `chip-sim`, `chip-post`, `chip-project`, `chip-cli`, `chip-gui`) and updated dependency graph diagram.
- Mark **Phase 14 (Completed — M48–M51)** in `specs/10-roadmap.md`.

---

## Verification Plan

1. **Off-Thread Remesh & Capture Tests (`chip-gui`):**
   - Verify interactive background remesh coalescing ignores stale scrub frames and updates `SimStockRenderSource` cleanly.
   - Verify headless `--capture-*` screenshots still render deterministically with zero blank or lagged stock frames (smoke-capture 1 sample and visually inspect via `read_file`).
2. **Full Workspace Parity & Release-Readiness Gate:**
   - Verify zero `.chip.toml` schema drift (`schema_version = 1 / 2 / 3`) and byte-for-byte golden G-code parity across all `crates/chip-cli/tests/golden/` snapshots.
   - `cargo fmt --all -- --check`
   - `cargo deny check`
   - `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
