# Milestone 98 Implementation Plan — Direct Cross-Crate Moves & CLI/GUI Domain Unification

## Goal

Relocate ~6,500+ LOC of misplaced code directly between *existing* workspace crates so each crate owns its true domain concepts, and unify all duplicated (and drifted) CLI/GUI project workflows into `chip-project`, `chip-cad`, `chip-cam`, `chip-sim`, `chip-sketch`, and `chip-post`—fixing real divergence bugs uncovered by the audit.

---

## Proposed Changes

### 1. Cross-Crate Moves Around `chip-cam`, `chip-tools`, `chip-cad`, `chip-stock`, `chip-gui`
- **`chip-cam` → `chip-tools`:** Move `crates/chip-cam/src/thread_tables.rs` (525 LOC) to `crates/chip-tools/src/thread_tables.rs` (zero `chip-cam` deps or callers).
- **`chip-tools` → `chip-cam`:**
  - Move `ToolOperationClass` and `CutterGeometry::supports_operation` from `chip-tools/src/model.rs` into `chip-cam/src/operation.rs` (eliminating the Layer 1 → Layer 3 inversion).
  - Move `crates/chip-tools/src/machinability.rs` (1,100 LOC, sole non-test caller is `chip-cam::machinability_advisor`) to `crates/chip-cam/src/machinability_physics.rs` (breaking `chip-tools`'s internal tie to `MachineSpindleSpec`).
- **`chip-cam` → `chip-cad`:** Move B-Rep/mesh `measurement.rs` (~940 prod LOC, zero `chip-cam` imports) to `crates/chip-cad/src/measurement.rs`; keep the ~80-LOC `snap_sketch_to_measure_target` adapter in `chip-cam`.
- **`chip-cad` → `chip-cam`:** Move the 4 `spindle_up_world` CAM feature helpers (`highest_adjacent_wall_top`, `classify_planar_face_loops`, `extract_modeled_chamfer_strip`, `highest_adjacent_lower_floor_proj`) from `chip-cad/src/topology.rs` to `chip-cam/src/selection/brep.rs`.
- **`chip-project` → `chip-cad`:** Move `is_mesh_path`, `load_model_bytes_cad_doc`, `load_model_entry_cad_doc`, and `load_model_entry_cad_doc_transformed` from `chip-project/src/document.rs` to `crates/chip-cad/src/loader.rs` (and update `chip-cli/src/commands/bundle_reload_tools.rs` so `reload-step` uses the unified loader).
- **`chip-project` → `chip-cam`:** Move `StrategyParams` / rotary / undercut / deburr validation (~650 LOC in `chip-project/src/validation.rs:675–1285, 1320–1350`) to `crates/chip-cam/src/validation.rs`; move `sketch_face_to_cad_binding` / `cad_binding_to_sketch_face` bridge from `chip-project/src/reload.rs` into `chip-cam`.
- **`chip-cam` → `chip-project`:** Move `crates/chip-cam/src/workholding_wizards.rs` (~3.1k prod LOC, zero production callers in `chip-cam`) into `crates/chip-project/src/wizards/{mod, silhouette, flip_jig, soft_jaw, soft_jaw_mesh}.rs`.
- **`chip-cam` → `chip-stock`:** Move `infer_tridexel_resolution` from `chip-cam/src/strategies/mod.rs` to `TriDexelStock::infer_resolution` in `chip-stock`.
- **`chip-gui` → `chip-cam`:** Move `crates/chip-gui/src/viewport/boundary_preview.rs` (926 prod LOC, zero `egui`/`wgpu` deps) to `crates/chip-cam/src/boundary_preview.rs`.

### 2. CLI/GUI Domain Unification & Divergence Fixes
- **Pure `ProjectDocument` Mutations (`crates/chip-project/src/mutations.rs`):** Move document CRUD, Rest Machining & carryover dirty invalidation, slot reordering, shared avoid-region promotion/linking, and `FlipJigPlan`/`SoftJawPlan` document application out of `chip-gui/src/panels/{browser/actions.rs, browser/drag_reorder.rs, operation_inspector/avoid_card.rs, sketch_inspector/mod.rs, workholding_wizards.rs}` and `chip-cli/src/commands/workholding.rs`. Deduplicate `compute_sha256_hex` / `compute_sketch_sha256_hex`.
- **Headless Project Export, Scaffolding, & Validation (`chip-project`, `chip-cad`, `chip-sketch`):**
  - Move `export_all_setups_to_dir` (`gcode_preview.rs` + `cli/validate_sim_build.rs`), `TempExtractGuard`/`load_project_or_chipz` (`cli/commands/mod.rs`), `ProjectDocument::new_pure_2d_sheet` (fixing CLI `2.0 mm` vs. GUI `3.0 mm` drift), `ProjectDocument::new_from_mesh`, `ProjectDocument::apply_model_transform`, `setup_requires_volumetric_sim` (fixing CLI vs. GUI Tri-Dexel drift), `validate_setup_machine_limits` (`diagnostics.rs`), and `evaluate_{operation,setup}_machinability` (`machinability_card.rs`, with `primary_fixable_finding` on `chip-cam::OperationMachinabilityReport`) into `chip-project`.
  - Move `ModelTransform::compose_for_mesh_import` into `chip-cad` and `create_default_sketch_entity` into `chip-sketch`.
- **Simulation & Kinematics Consolidation (`chip-post`, `chip-sim`, `chip-project`):**
  - Add `PostProfile::to_kinematics_spec(&self) -> MachineKinematicsSpec` in `chip-post` and canonical `resolve_setup_kinematics(project, setup_idx, project_dir)` in `chip-project/src/pipeline.rs` (fixing CLI, `dispatcher.rs`, and carryover silently ignoring `project.machine_id`).
  - Unify `chip-sim::MachineKinematicsConfig` with `chip_tools::MachineKinematicsSpec` and alias `SimVerificationConfig = StockVerifyTolerances`.
  - Move `compute_stock_model_wcs` into `chip-sim` (`SetupSimulationResult::stock_model_wcs_transform`) and add `from_str_loose` / `FromStr` for `SimEngineKind`, `SimComputeBackendChoice`, and `SimResolution` (deduplicating `chip-cli/src/commands/mod.rs` and `chip-gui/src/capture.rs`).
  - Collapse the 9 telescoping `simulate_project_setup*` / `simulate_all_setups*` functions in `chip-project/src/pipeline.rs` to 2 request-struct entry points (`SetupSimRequest`, `AllSetupsSimRequest`), and collapse the 7 `run_setup_simulation*` wrappers in `chip-sim/src/driver.rs` to 2.

---

## Verification Plan
- `cargo fmt --all -- --check`
- `cargo clippy --locked --workspace --all-targets -- -D warnings`
- `WGPU_BACKEND=vulkan cargo test --locked --workspace`
- Verify zero golden G-code diffs and add targeted unit tests for unified `resolve_setup_kinematics`, `setup_requires_volumetric_sim`, and `new_pure_2d_sheet`.
