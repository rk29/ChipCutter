//! Tool geometry, per-material cutting presets, coupled SFM/chipload calculator,
//! `.tools.toml` persistence, and Fusion 360 JSON / `.tools` importer (`chip-tools`).

pub mod builtin;
pub mod envelope;
pub mod feeds_speeds;
#[cfg(feature = "fusion-import")]
pub mod fusion_import;
pub mod materials;
pub mod model;
#[cfg(feature = "user-config")]
pub mod preset_store;
pub mod storage;
pub mod thread_tables;

pub use builtin::{
    default_starter_tool_library, starter_dovetail_60deg_12mm, starter_lollipop_6mm,
    starter_tslot_12mm, starter_undercut_tools,
};
pub use envelope::{
    AnalyticCutterEnvelope, CutterEnvelopeShape, ToolAssemblyEnvelope, ToolAssemblyPart,
};
pub use feeds_speeds::{
    ChipThinningHint, CuttingPreset, SurfaceSpeedPresetSpec, chipload_from_feed,
    compute_adaptive_thinning_feed, compute_radial_chip_thinning, feed_from_chipload,
    rpm_from_surface_speed, surface_speed_from_rpm,
};
#[cfg(feature = "fusion-import")]
pub use fusion_import::{
    FusionImportError, FusionImportReport, import_fusion_library_bytes, import_fusion_library_path,
};
pub use materials::{
    ACETAL_DELRIN_ID, ACRYLIC_PMMA_ID, ALUMINUM_6061_T6_ID, ALUMINUM_6063_T5_ID,
    ALUMINUM_7075_T6_ID, BRASS_C360_ID, CUSTOM_WORKPIECE_MATERIAL_ID,
    DEFAULT_WORKPIECE_MATERIAL_ID, G10_CARBON_FIBER_ID, HARDWOOD_PLYWOOD_ID, HDPE_PLASTIC_ID,
    MDF_ID, MILD_STEEL_1018_ID, MaterialCategory, ProjectMaterialSelection,
    WorkpieceMaterialProfile, builtin_workpiece_materials, match_best_preset_for_material,
    resolve_workpiece_material,
};
pub use model::{
    CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId, ToolValidationError,
    is_exact_tool_match, rank_replacement_tools,
};
pub use storage::{ToolLibrary, ToolStorageError};
#[cfg(feature = "user-config")]
pub use storage::{default_global_tool_library_path, load_or_default_global_library};
pub use thread_tables::{
    ISO_60_H_OVER_P, ThreadSpecEntry, ThreadStandardKind, all_thread_specs, find_thread_spec,
    iso60_minor_diameter_internal_mm, iso60_pitch_diameter_mm, suggest_thread_for_hole_diameter,
    thread_specs_for_standard,
};
