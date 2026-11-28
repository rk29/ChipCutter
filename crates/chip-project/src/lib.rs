//! Project document state, deterministic `.chip.toml` serialization, CAD SHA-256 verification,
//! project invariant validation, and instant Undo/Redo history (`chip-project`).

pub mod bundle;
pub mod cache;
pub mod dispatcher;
pub mod document;
pub mod history;
pub mod mutations;
pub mod pipeline;
pub mod reload;
pub mod toml_io;
pub mod validation;
pub mod wizards;

pub use bundle::{
    BundleError, ChipzExportSummary, ChipzPackOptions, LoadedProjectInput, MAX_CHIPZ_ENTRIES,
    MAX_CHIPZ_UNPACK_BYTES, TempExtractGuard, UnpackedChipzBundle, create_private_temp_extract_dir,
    export_chipz_bundle, export_chipz_bundle_with_options,
    external_or_suspicious_assets_referenced, is_chipz_path, load_primary_cad_for_project,
    load_project_or_chipz, pack_chipz_bundle_bytes, pack_chipz_bundle_with_options,
    unpack_chipz_bundle, unpack_chipz_bytes,
};
pub use cache::{
    OperationInputHash, ToolpathCache, compute_operation_input_hash,
    compute_setup_carryover_digest, resolve_upstream_rest_op_indices,
};
pub use dispatcher::{
    BatchGenerationReport, DispatcherEvent, GenerationProgressSnapshot, JobDispatcher,
    format_exec_duration, generate_project_blocking, generate_project_blocking_in_dir,
};
pub use document::{
    DEFAULT_PURE_2D_SHEET_MARGIN_MM, MachineToolSwapPlan, MachineToolSwapSummary, ModelEntry,
    ModelRole, ProjectDocument, ToolOperationRef, ToolSwapChoice, ToolSwapEntry,
    compute_sha256_hex, is_flatpak_document_portal_path, portable_model_path, resolve_model_path,
    setup_requires_volumetric_sim,
};
pub use history::UndoHistory;
pub use mutations::{
    QuickFixturePreset, add_fixture_preset_in_setup, add_setup_in_project,
    add_sketch_entity_preset_in_project, apply_flip_jig_plan, apply_soft_jaw_plan,
    create_adaptive_2d_operation_in_setup, create_and_link_shared_avoid,
    create_default_operation_in_setup, delete_operation_in_setup, detach_shared_avoid_to_local,
    duplicate_fixture_in_setup, duplicate_operation_in_setup, duplicate_setup_in_project,
    duplicate_sketch_entity_in_sketch, duplicate_sketch_in_project,
    invalidate_changed_rest_dependents, mark_ops_dirty_with_rest,
    mark_shared_avoid_dependents_dirty, mark_sketch_dependent_ops_dirty,
    move_avoid_region_to_slot_in_project, move_fixture_to_slot_in_project, move_operation_to_slot,
    move_setup_to_slot_in_project, move_sketch_entity_to_slot_in_project,
    move_sketch_to_slot_in_project, next_shared_avoid_id_and_name, next_unique_setup_id,
    next_unique_setup_id_with_base, next_work_offset_slot, pick_flat_endmill_id, pick_pin_tool,
    promote_local_avoid_to_shared, remap_index_after_slot_move, set_shared_avoid_for_all_setup_ops,
    snapshot_rest_upstream_ids, toggle_shared_avoid_link,
};
pub use pipeline::{
    AllSetupsSimRequest, BatchExportSummary, BatchSetupExportItem, CarryoverStockResult,
    ReadySetupOperation, SetupPostRequest, SetupSimRequest, collect_ready_setup_operations,
    compute_setup_carryover_stock, export_all_setups_to_dir, kinematics_for_post_profile,
    post_process_project_setup, resolve_kinematics_for_key, resolve_setup_kinematics,
    sanitize_program_stem, simulate_all_setups, simulate_project_setup,
};
pub use reload::{CadReloadReport, OperationReloadOutcome, rebind_project_to_reloaded_cad};
pub use toml_io::ProjectIoError;
pub use validation::{
    DiagnosticSeverity, ProjectMachinabilityReport, ProjectValidationReport, ValidationDiagnostic,
    analyze_project_machinability, evaluate_operation_machinability, evaluate_setup_machinability,
    validate_project, validate_project_with_machine, validate_setup_machine_limits,
};
pub use wizards::{
    FlipAxis, FlipJigConfig, FlipJigPinCount, FlipJigPlan, STANDARD_PIN_DIAMETERS_MM,
    SoftJawBoltPattern, SoftJawConfig, SoftJawOutputMode, SoftJawPlan, build_soft_jaw_plan,
    build_two_sided_flip_jig, extract_part_silhouette_2d,
};
