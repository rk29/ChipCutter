//! User-editable TOML post-processor profiles, ATC vs. split-by-tool partitioning,
//! modal state tracking, arc splitting, and G-code emission (`chip-post`).

#![deny(unsafe_code)]

mod cycles_arcs;
pub mod emitter;
mod modal;
pub mod partition;
pub mod profile;
pub mod rotary;

pub use emitter::post_process_setup;
pub use partition::{
    OperationPostInput, PostOutputFile, PostRotarySetup, PostStockSummary, SetupPostInput,
    sanitize_file_extension, validate_safe_output_filename,
};
pub use profile::{
    ArcCenterMode, BUILTIN_GRBL_ROTARY_WRAP_TOML, BUILTIN_GRBL_TOML, BUILTIN_GRBLHAL_4AXIS_TOML,
    BUILTIN_GRBLHAL_ATC_TOML, BUILTIN_LINUXCNC_4AXIS_TOML, BUILTIN_LINUXCNC_TOML, CommentStyle,
    LineNumberConfig, PostError, PostProfile, PostProfileSource, PostUnits, RotaryPostConfig,
    SwapAxisTarget, ToolChangeMode, allocate_unique_custom_post_id, builtin_post_profiles,
    clone_post_profile, delete_user_post_profile, delete_user_post_profile_in,
    export_post_profile_to_path, import_post_profile_from_path, import_post_profile_from_path_in,
    is_builtin_post_id, load_available_post_profiles, load_available_post_profiles_in,
    resolve_post_profile, resolve_post_profile_cli_override, sanitize_post_id,
    save_user_post_profile, save_user_post_profile_in, user_post_profiles_dir, validate_post_id,
};
pub use rotary::{
    compute_inverse_time_feed_g93, compute_rotary_angular_feed_deg_min,
    compute_workpiece_arc_distance_mm, resolve_rotary_target_deg,
};
