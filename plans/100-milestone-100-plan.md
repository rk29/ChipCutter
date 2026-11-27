# Milestone 100 Implementation Plan — Internal Monolith Decomposition, Inline Test Extraction, & Phase 27 Spec Sync

## Goal

Decompose the remaining oversized production monoliths across `chip-gui`, `chip-cam`, `chip-stock`, and `chip-sim` into clean, cohesive submodules, move ~13,000+ LOC of inline integration tests from `src/` into `tests/`, synchronize all living architecture documentation, and run the full Phase 27 verification gate (including headless Vulkan visual verification).

---

## Proposed Changes

### 1. Inline Integration Test Extraction (`src/` → `tests/`)
- Move massive inline `#[cfg(test)]` integration suites out of production `src/` files into crate `tests/` directories (strictly testing public APIs):
  - `crates/chip-sim/src/driver.rs` (~3,170 LOC) → `crates/chip-sim/tests/driver_tests.rs`
  - `crates/chip-stock/src/gpu/mod.rs` (~2,637 LOC) → `crates/chip-stock/tests/gpu_parity.rs` (`#![cfg(feature = "gpu-compute")]`)
  - `crates/chip-stock/src/mesh_skinning_tests.rs` (895 LOC) → `crates/chip-stock/tests/mesh_skinning.rs`
  - `crates/chip-tools/src/lib.rs` (~1,004 LOC) → `crates/chip-tools/tests/tools_integration.rs`
  - `crates/chip-post/src/lib.rs` (~948 LOC) → `crates/chip-post/tests/post_integration.rs` (and add `#![deny(unsafe_code)]` to `chip-post/src/lib.rs`)
  - `crates/chip-bvh/src/lib.rs` (~574 LOC) → `crates/chip-bvh/tests/bvh_oracles.rs`
  - Large GUI inline suites in `panels/operation_inspector/mod.rs` (~2,377 LOC) and `viewport/pipeline.rs` (~2,113 LOC) → sibling `tests.rs` / `crates/chip-gui/tests/` modules.

### 2. Core Engine Monolith Splits (`chip-cam`, `chip-stock`, `chip-sim`)
- **`chip-cam`:**
  - Split `strategies/deburr3d.rs` (~2.8k prod LOC) into `strategies/deburr3d/{mod, extract, contact, trim, emit}.rs`.
  - Split `rotary.rs` into `rotary/{mod, indexed, window, surface_drop}.rs`.
  - Split `fixture_avoidance.rs` into `fixture_collision.rs` (pure swept-assembly vs. BVH kernel) and `fixture_routing.rs` (CL rapid re-router).
  - Split `workholding.rs` into `workholding/{mod, model, mesh_gen}.rs`.
  - Split `surface3d/finishing.rs` into `surface3d/{chords, rest_mask, linking}.rs`.
- **`chip-stock`:**
  - Split `tridexel/sweep.rs` (2,421 LOC) into `tridexel/sweep/{mod, cardinal, oriented, replay}.rs`.
  - Deduplicate `gpu/backend.rs` vertical plunge query against `backend.rs`, and share bounding helpers between `gpu/heightfield.rs` and `heightfield/sweep.rs`.
- **`chip-sim`:**
  - Split `SetupSimulationResult` / playback sync out of `driver.rs` into `result.rs`; unify `prepare_initial_stock` & `prepare_initial_stock_with_gpu`; unify CPU/GPU playback cursor sync; deduplicate the 27-line GPU flush block.
  - Deduplicate `cad_adapter.rs` analytic horizontal-face collection and rename `extract_cylindrical_target_mesh_from_cad` → `extract_target_trimesh_from_cad`.

### 3. `chip-gui` Monolith Decomposition
- Split `panels/browser/mod.rs` (`show_cam_browser`, 2,635-LOC single function) into per-section submodules (`models_section.rs`, `sketches_section.rs`, `setups_section.rs`, `tools_section.rs`, `avoid_section.rs`).
- Split `app/viewport_controller.rs` (2,186 prod LOC) into `viewport_sync.rs`, `picking.rs`, `sketch_authoring.rs`, `measure_controller.rs`, and `placement_modes.rs`.
- Split `viewport/gizmos.rs` (2,111 LOC) into `viewport/gizmos/{mod, stock_wcs, sketch, rotary, boundary, measure_reach}.rs`.
- Extract the 515-LOC headless capture / self-test runner out of `app/pane_behavior.rs` into `app/self_test.rs`.
- Extract cutting-area cards out of `panels/operation_inspector/geometry_tab.rs` into `cutting_area_cards.rs`.
- Split `panels/workholding_wizards.rs` into `workholding_wizards/{mod, flip_jig, soft_jaws}.rs`.
- Group `ChipCutterApp`'s 66 flat fields in `app/mod.rs` into cohesive domain sub-structs (`WorkspaceSession`, `SelectionAndPickState`, `BackgroundWorkers`, `ViewportOverlayPrefs`).

### 4. Architecture Spec Sync & Full Verification Gate
- Synchronize `specs/00-overview.md`, `specs/01-architecture.md` (updated 17-crate workspace layout, layered DAG diagram, and module boundaries), `specs/10-roadmap.md` (mark Phase 27 Completed M97–M100), `AGENTS.md`, `ARCHITECTURE.md`, and `DEVELOPMENT.md`.
- Run the complete verification checklist:
  - `cargo fmt --all -- --check`
  - `cargo clippy --locked --workspace --all-targets -- -D warnings`
  - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
  - `cargo deny check && cargo deny --no-default-features --features chip-gui/flatpak-portal check`
  - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
  - Headless Vulkan GUI screenshot smoke test (`--self-test-screenshot`) visually verified via `read_file`.
