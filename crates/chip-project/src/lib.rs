//! Project document state, deterministic `.chip.toml` serialization, CAD SHA-256 verification,
//! project invariant validation, and instant Undo/Redo history (`chip-project`).

pub mod bundle;
pub mod cache;
pub mod demos;
pub mod dispatcher;
pub mod document;
pub mod history;
pub mod pipeline;
pub mod reload;
pub mod toml_io;
pub mod validation;

pub use bundle::{
    BundleError, ChipzExportSummary, ChipzPackOptions, MAX_CHIPZ_ENTRIES, MAX_CHIPZ_UNPACK_BYTES,
    UnpackedChipzBundle, create_private_temp_extract_dir, export_chipz_bundle,
    export_chipz_bundle_with_options, external_or_suspicious_assets_referenced, is_chipz_path,
    pack_chipz_bundle_bytes, pack_chipz_bundle_with_options, unpack_chipz_bundle,
    unpack_chipz_bytes,
};
pub use cache::{
    OperationInputHash, ToolpathCache, compute_operation_input_hash,
    resolve_upstream_rest_op_indices,
};
pub use demos::{
    build_adaptive_rest_demo_project, build_cam_torture_project,
    build_engraved_nameplate_demo_project, build_multi_feature_bracket_project,
    build_pure2d_router_panel_demo_project, build_relief_wave_mesh_project,
    build_revision_v1_demo_project, build_rotary_helical_flute_project,
    build_rotary_shaft_preview_project, build_rotary_wrapped_engraving_project,
    build_sculpted_mesh_dome_project, build_thread_mill_manifold_project,
    build_tutorial_3d_adaptive_waterline_project, build_tutorial_3d_surface_finishing_project,
    build_tutorial_16_tilted_step_project, build_tutorial_16_yup_stl_project,
    build_tutorial_adaptive_rest_project, build_tutorial_chamfer_trace_project,
    build_tutorial_contour_project, build_tutorial_drill_bore_project,
    build_tutorial_facing_project, build_tutorial_indexed_step_multiface_project,
    build_tutorial_pocket_project, build_tutorial_rotary_4axis_flowline_project,
    build_tutorial_rotary_axial_parallel_project, build_tutorial_vcarve_inlay_project,
    build_vcarve_sign_and_inlay_demo_project,
};
pub use dispatcher::{
    BatchGenerationReport, DispatcherEvent, JobDispatcher, generate_project_blocking,
};
pub use document::{
    ModelEntry, ModelRole, ProjectDocument, compute_sha256_hex, is_mesh_path,
    load_model_bytes_cad_doc, load_model_bytes_cad_doc_transformed, load_model_entry_cad_doc,
    load_model_entry_cad_doc_transformed, portable_model_path, resolve_model_path,
};
pub use history::UndoHistory;
pub use pipeline::{
    ReadySetupOperation, SetupPostRequest, SetupSimRequest, collect_ready_setup_operations,
    kinematics_for_post_profile, post_process_project_setup, resolve_kinematics_for_key,
    simulate_project_setup, simulate_project_setup_with_choice, simulate_project_setup_with_engine,
};
pub use reload::{
    CadReloadReport, OperationReloadOutcome, cad_binding_to_sketch_face,
    rebind_project_to_reloaded_cad, sketch_face_to_cad_binding,
};
pub use toml_io::ProjectIoError;
pub use validation::{
    DiagnosticSeverity, ProjectValidationReport, ValidationDiagnostic, validate_project,
    validate_project_with_machine,
};
