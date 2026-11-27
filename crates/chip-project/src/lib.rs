//! Project document state, deterministic `.chip.toml` serialization, CAD SHA-256 verification,
//! project invariant validation, and instant Undo/Redo history (`chip-project`).

pub mod bundle;
pub mod cache;
pub mod demos;
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
pub use demos::{
    build_adaptive_rest_demo_project, build_cam_torture_project, build_curved_deburr_3d_project,
    build_dovetail_rail_keyway_project, build_engraved_nameplate_demo_project,
    build_feed_opt_pocket_project, build_flip_jig_softjaws_project,
    build_indexed_model_bounds_project, build_keepout_clamps_project,
    build_mesh_3mf_bracket_project, build_multi_feature_bracket_project,
    build_pure2d_router_panel_demo_project, build_relief_wave_mesh_project,
    build_revision_v1_demo_project, build_rotary_helical_flute_project,
    build_rotary_shaft_preview_project, build_rotary_wrapped_engraving_project,
    build_sculpted_mesh_dome_project, build_svg_badge_sign_project,
    build_thread_mill_manifold_project, build_tutorial_3d_adaptive_waterline_project,
    build_tutorial_3d_surface_finishing_project, build_tutorial_16_tilted_step_project,
    build_tutorial_16_yup_stl_project, build_tutorial_adaptive_rest_project,
    build_tutorial_chamfer_trace_project, build_tutorial_contour_project,
    build_tutorial_drill_bore_project, build_tutorial_facing_project,
    build_tutorial_indexed_step_multiface_project, build_tutorial_pocket_project,
    build_tutorial_rotary_4axis_flowline_project, build_tutorial_rotary_axial_parallel_project,
    build_tutorial_vcarve_inlay_project, build_two_sided_vise_workholding_project,
    build_undercut_deburr_block_project, build_vcarve_sign_and_inlay_demo_project,
    sample_two_sided_vise_part_stl_bytes,
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
