use std::path::{Path, PathBuf};

use chip_math::MM_PER_INCH;
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub const BUILTIN_GRBL_TOML: &str = include_str!("../../../posts/grbl.post.toml");
pub const BUILTIN_GRBLHAL_ATC_TOML: &str = include_str!("../../../posts/grblhal_atc.post.toml");
pub const BUILTIN_LINUXCNC_TOML: &str = include_str!("../../../posts/linuxcnc.post.toml");
pub const BUILTIN_LINUXCNC_4AXIS_TOML: &str =
    include_str!("../../../posts/linuxcnc_4axis.post.toml");
pub const BUILTIN_GRBLHAL_4AXIS_TOML: &str = include_str!("../../../posts/grblhal_4axis.post.toml");
pub const BUILTIN_GRBL_ROTARY_WRAP_TOML: &str =
    include_str!("../../../posts/grbl_rotary_wrap.post.toml");

/// Origin of a resolved [`PostProfile`] so UI and CLI can flag untrusted project/bundle posts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum PostProfileSource {
    #[default]
    BuiltIn,
    UserConfig,
    ProjectOrBundle,
    ExplicitFile,
}

/// Errors raised during post-profile loading or G-code emission (`chip-post`).
#[derive(Debug, Clone, PartialEq, Error)]
pub enum PostError {
    #[error("failed to read post profile '{path}': {message}")]
    Io { path: String, message: String },
    #[error("invalid TOML post profile '{source_name}': {message}")]
    Parse {
        source_name: String,
        message: String,
    },
    #[error("unknown post-processor profile '{id_or_path}'")]
    UnknownProfile { id_or_path: String },
    #[error("invalid post-processor profile id '{id}'")]
    InvalidProfileId { id: String },
    #[error("built-in post-processor profile '{id}' is read-only; clone it to a custom ID first")]
    BuiltInReadOnly { id: String },
    #[error("could not determine OS user configuration directory")]
    NoUserConfigDir,
    #[error("unsafe output filename '{filename}' rejected")]
    UnsafeOutputFilename { filename: String },
    #[error("unsafe G-code line rejected: {reason}")]
    UnsafeGCodeLine { reason: String },
    #[error("non-finite numeric value in '{field}' cannot be emitted to G-code")]
    NonFiniteValue { field: &'static str },
    #[error("setup '{setup_name}' has no enabled operations to post-process")]
    NoOperations { setup_name: String },
    #[error("emitted G-code line ({len} chars) exceeds profile max_line_length ({max}): {line}")]
    LineTooLong {
        len: usize,
        max: usize,
        line: String,
    },
}

/// Output length/feedrate unit system (`G21` mm vs `G20` inch).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PostUnits {
    #[default]
    #[serde(alias = "millimeter", alias = "millimeters")]
    Mm,
    #[serde(alias = "inches", alias = "in")]
    Inch,
}

impl PostUnits {
    #[inline]
    pub const fn g_code(self) -> &'static str {
        match self {
            Self::Mm => "G21",
            Self::Inch => "G20",
        }
    }

    #[inline]
    pub const fn unit_suffix(self) -> &'static str {
        match self {
            Self::Mm => "mm",
            Self::Inch => "in",
        }
    }

    /// Converts canonical `f64` millimeters (or `mm/min` feedrate) into output units (`mm` or `in`, `mm/min` or `ipm`).
    #[inline]
    pub fn scale_from_mm(self, val_mm: f64) -> f64 {
        match self {
            Self::Mm => val_mm,
            Self::Inch => val_mm / MM_PER_INCH,
        }
    }
}

/// Comment syntax style (`( ... )` vs `; ...`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommentStyle {
    #[default]
    Parentheses,
    Semicolon,
}

impl CommentStyle {
    pub const ALL: [Self; 2] = [Self::Parentheses, Self::Semicolon];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Parentheses => "Parentheses ( ... )",
            Self::Semicolon => "Semicolon ; ...",
        }
    }
}

/// Circular arc center parameterization (`07-post-processor.md` §3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ArcCenterMode {
    #[default]
    IncrementalIjk,
    AbsoluteIjk,
    Radius,
}

impl ArcCenterMode {
    pub const ALL: [Self; 3] = [Self::IncrementalIjk, Self::AbsoluteIjk, Self::Radius];

    pub const fn label(self) -> &'static str {
        match self {
            Self::IncrementalIjk => "Incremental IJK (G91.1)",
            Self::AbsoluteIjk => "Absolute IJK (G90.1)",
            Self::Radius => "Radius (R)",
        }
    }
}

/// Multi-tool partitioning / ATC policy (`07-post-processor.md` §5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolChangeMode {
    /// Emits a single G-code file with `tool_change_block` (`T# M6`, `G43 H#`) whenever tool changes.
    #[default]
    M6,
    /// Partitions consecutive operations sharing a tool into numbered `.nc` files for non-ATC machines.
    SplitFilesByTool,
    /// Suppresses `M6` when the entire job uses a single tool; emits `M6` if multiple tools exist.
    IgnoreIfSingleTool,
}

impl ToolChangeMode {
    pub const ALL: [Self; 3] = [Self::M6, Self::SplitFilesByTool, Self::IgnoreIfSingleTool];

    pub const fn label(self) -> &'static str {
        match self {
            Self::M6 => "Single File (T# M6)",
            Self::SplitFilesByTool => "Split Files by Tool",
            Self::IgnoreIfSingleTool => "Ignore M6 if Single Tool",
        }
    }
}

/// Optional `N10`, `N20` block numbering config.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct LineNumberConfig {
    pub start: u32,
    pub increment: u32,
}

fn default_schema_version() -> u32 {
    1
}
fn default_extension() -> String {
    "nc".to_string()
}
fn default_coord_decimals() -> usize {
    3
}
fn default_feed_decimals() -> usize {
    0
}
fn default_true() -> bool {
    true
}
fn default_max_arc_sweep() -> f64 {
    90.0
}
fn default_helical_tol() -> f64 {
    0.015
}
fn default_rapid_rate() -> f64 {
    5_000.0
}
fn default_accel() -> f64 {
    500.0
}
fn default_tool_change_sec() -> f64 {
    15.0
}
fn default_rotary_axis_letter() -> char {
    'A'
}
fn default_angle_decimals() -> usize {
    3
}
fn default_max_rotary_feed() -> f64 {
    3_600.0
}

/// Optional linear axis replacement target for 3-axis controllers running a swapped-axis rotary chuck.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SwapAxisTarget {
    ReplaceY,
    ReplaceX,
}

/// Optional 4th-axis rotary post-processor dialect block (`[rotary]` in `*.post.toml`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RotaryPostConfig {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default = "default_rotary_axis_letter")]
    pub axis_letter: char,
    #[serde(default = "default_angle_decimals")]
    pub angle_decimals: usize,
    #[serde(default)]
    pub supports_g93_inverse_time: bool,
    #[serde(default)]
    pub use_g93_for_simultaneous: bool,
    #[serde(default = "default_max_rotary_feed")]
    pub max_rotary_feed_deg_min: f64,
    #[serde(default)]
    pub unwind_mode: chip_cam::RotaryUnwindMode,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub swap_linear_axis: Option<SwapAxisTarget>,
}

impl Default for RotaryPostConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            axis_letter: default_rotary_axis_letter(),
            angle_decimals: default_angle_decimals(),
            supports_g93_inverse_time: false,
            use_g93_for_simultaneous: false,
            max_rotary_feed_deg_min: default_max_rotary_feed(),
            unwind_mode: chip_cam::RotaryUnwindMode::ContinuousMultiTurn,
            swap_linear_axis: None,
        }
    }
}

impl RotaryPostConfig {
    #[inline]
    pub const fn is_disabled(&self) -> bool {
        !self.enabled
    }
}

/// User-editable TOML Post-Processor Profile (`posts/*.post.toml`, `07-post-processor.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PostProfile {
    #[serde(default = "default_schema_version")]
    pub schema_version: u32,
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub description: String,
    #[serde(default = "default_extension")]
    pub extension: String,

    #[serde(default)]
    pub units: PostUnits,
    #[serde(default = "default_coord_decimals")]
    pub coord_decimals: usize,
    #[serde(default = "default_feed_decimals")]
    pub feed_decimals: usize,
    #[serde(default)]
    pub spindle_decimals: usize,
    #[serde(default)]
    pub suppress_trailing_zeros: bool,
    #[serde(default)]
    pub line_numbers: Option<LineNumberConfig>,
    #[serde(default)]
    pub max_line_length: Option<usize>,
    #[serde(default)]
    pub comment_style: CommentStyle,
    #[serde(default)]
    pub path_blend_tol_mm: Option<f64>,

    #[serde(default = "default_true")]
    pub emit_wcs_code: bool,
    #[serde(default = "default_true")]
    pub allow_extended_wcs: bool,
    #[serde(default)]
    pub use_g43_length_offset: bool,
    #[serde(default)]
    pub use_cutter_comp_g41_g42: bool,
    #[serde(default)]
    pub safe_home_z_on_end: bool,

    #[serde(default)]
    pub arc_center_mode: ArcCenterMode,
    #[serde(default = "default_max_arc_sweep")]
    pub max_arc_sweep_deg: f64,
    #[serde(default = "default_true")]
    pub allow_helical_arcs: bool,
    #[serde(default = "default_helical_tol")]
    pub helical_linearize_tol_mm: f64,

    #[serde(default)]
    pub expand_drill_cycles: bool,
    #[serde(default)]
    pub tool_change_mode: ToolChangeMode,

    #[serde(default = "default_rapid_rate")]
    pub rapid_rate_mm_min: f64,
    #[serde(default = "default_accel")]
    pub accel_mm_s2: f64,
    #[serde(default = "default_tool_change_sec")]
    pub tool_change_time_sec: f64,

    #[serde(default)]
    pub program_header: Vec<String>,
    #[serde(default)]
    pub operation_header: Vec<String>,
    #[serde(default)]
    pub tool_change_block: Vec<String>,
    #[serde(default)]
    pub spindle_start_block: Vec<String>,
    #[serde(default)]
    pub operation_footer: Vec<String>,
    #[serde(default)]
    pub program_footer: Vec<String>,

    #[serde(default, skip_serializing_if = "RotaryPostConfig::is_disabled")]
    pub rotary: RotaryPostConfig,

    #[serde(skip)]
    pub is_builtin: bool,
    #[serde(skip)]
    pub source: PostProfileSource,
}

impl PostProfile {
    pub fn from_toml_str(source_name: &str, toml_str: &str) -> Result<Self, PostError> {
        let mut prof: Self = toml::from_str(toml_str).map_err(|e| PostError::Parse {
            source_name: source_name.to_string(),
            message: e.to_string(),
        })?;
        prof.is_builtin = false;
        prof.source = PostProfileSource::ExplicitFile;
        Ok(prof)
    }

    pub fn load_from_path(path: &Path) -> Result<Self, PostError> {
        let raw = std::fs::read_to_string(path).map_err(|e| PostError::Io {
            path: path.display().to_string(),
            message: e.to_string(),
        })?;
        Self::from_toml_str(&path.display().to_string(), &raw)
    }

    pub fn to_toml_string(&self) -> Result<String, PostError> {
        toml::to_string_pretty(self).map_err(|e| PostError::Parse {
            source_name: self.id.clone(),
            message: e.to_string(),
        })
    }
}

/// Returns all built-in post profiles (`grbl`, `grblhal_atc`, `linuxcnc`, `linuxcnc_4axis`,
/// `grblhal_4axis`, and `grbl_rotary_wrap`).
pub fn builtin_post_profiles() -> Vec<PostProfile> {
    let mut out = Vec::with_capacity(6);
    for (name, src) in [
        ("grbl.post.toml", BUILTIN_GRBL_TOML),
        ("grblhal_atc.post.toml", BUILTIN_GRBLHAL_ATC_TOML),
        ("linuxcnc.post.toml", BUILTIN_LINUXCNC_TOML),
        ("linuxcnc_4axis.post.toml", BUILTIN_LINUXCNC_4AXIS_TOML),
        ("grblhal_4axis.post.toml", BUILTIN_GRBLHAL_4AXIS_TOML),
        ("grbl_rotary_wrap.post.toml", BUILTIN_GRBL_ROTARY_WRAP_TOML),
    ] {
        if let Ok(mut p) = PostProfile::from_toml_str(name, src) {
            p.is_builtin = true;
            p.source = PostProfileSource::BuiltIn;
            out.push(p);
        }
    }
    out
}

/// Returns `true` if `id` matches a reserved read-only built-in post-processor ID.
pub fn is_builtin_post_id(id: &str) -> bool {
    let needle = id.trim();
    builtin_post_profiles()
        .iter()
        .any(|p| p.id.eq_ignore_ascii_case(needle))
}

/// Validates a custom post profile slug ID (`[A-Za-z0-9._-]`, non-empty, no leading `.`, no `..`).
pub fn validate_post_id(id: &str) -> Result<(), PostError> {
    if id.is_empty()
        || id.starts_with('.')
        || id.contains("..")
        || !id
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'-' | b'_' | b'.'))
    {
        return Err(PostError::InvalidProfileId { id: id.to_string() });
    }
    Ok(())
}

/// Returns the user override post directory (`~/.config/chipcutter/posts` on Linux).
pub fn user_post_profiles_dir() -> Option<PathBuf> {
    dirs::config_dir().map(|d| d.join("chipcutter").join("posts"))
}

/// Discovers all available post profiles, merging compiled-in read-only built-in profiles
/// with `.post.toml` files in `extra_search_dir` (project / `.chipz`) and `user_dir`.
///
/// Built-in post IDs are strictly read-only and can never be shadowed by untrusted project,
/// `.chipz`, or user files; user config profiles take precedence over untrusted project/bundle posts.
pub fn load_available_post_profiles_in(
    user_dir: Option<&Path>,
    extra_search_dir: Option<&Path>,
) -> Vec<PostProfile> {
    let mut profiles = builtin_post_profiles();

    let mut search_dirs: Vec<(PathBuf, PostProfileSource)> = Vec::new();
    if let Some(extra) = extra_search_dir {
        search_dirs.push((extra.to_path_buf(), PostProfileSource::ProjectOrBundle));
        search_dirs.push((extra.join("posts"), PostProfileSource::ProjectOrBundle));
    }
    if let Some(udir) = user_dir {
        search_dirs.push((udir.to_path_buf(), PostProfileSource::UserConfig));
    }

    for (dir, source) in search_dirs {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        let mut paths: Vec<PathBuf> = entries
            .filter_map(|e| e.ok().map(|de| de.path()))
            .filter(|p| {
                p.file_name()
                    .and_then(|s| s.to_str())
                    .is_some_and(|n| n.ends_with(".post.toml"))
            })
            .collect();
        paths.sort();

        for p in paths {
            if let Ok(mut prof) = PostProfile::load_from_path(&p) {
                if validate_post_id(&prof.id).is_err() || is_builtin_post_id(&prof.id) {
                    continue;
                }
                prof.is_builtin = false;
                prof.source = source;
                if let Some(existing) = profiles
                    .iter_mut()
                    .find(|x| !x.is_builtin && x.id.eq_ignore_ascii_case(&prof.id))
                {
                    *existing = prof;
                } else {
                    profiles.push(prof);
                }
            }
        }
    }

    profiles
}

/// Discovers all available post profiles, merging compiled-in read-only built-in profiles
/// with `.post.toml` files in `extra_search_dir` (project / `.chipz`) and `~/.config/chipcutter/posts/`.
///
/// Built-in post IDs are strictly read-only and can never be shadowed by untrusted project,
/// `.chipz`, or user files; user config profiles take precedence over untrusted project/bundle posts.
pub fn load_available_post_profiles(extra_search_dir: Option<&Path>) -> Vec<PostProfile> {
    let udir = user_post_profiles_dir();
    load_available_post_profiles_in(udir.as_deref(), extra_search_dir)
}

/// Sanitizes an arbitrary string into a valid post-profile slug ID (`[a-z0-9_-]`).
pub fn sanitize_post_id(raw: &str) -> String {
    let mut out = String::with_capacity(raw.len());
    for ch in raw.trim().chars() {
        if ch.is_ascii_alphanumeric() {
            out.push(ch.to_ascii_lowercase());
        } else if matches!(ch, '-' | '_' | ' ' | '.')
            && !out.ends_with('_')
            && !out.ends_with('-')
            && !out.is_empty()
        {
            out.push('_');
        }
    }
    let trimmed = out.trim_matches(|c| c == '_' || c == '-').to_string();
    if trimmed.is_empty() {
        "custom_post".to_string()
    } else {
        trimmed
    }
}

/// Picks a unique non-builtin custom post profile ID given `base_id` and `existing_ids`.
pub fn allocate_unique_custom_post_id(base_id: &str, existing_ids: &[String]) -> String {
    let clean = sanitize_post_id(base_id);
    let is_taken = |cand: &str| -> bool {
        is_builtin_post_id(cand) || existing_ids.iter().any(|e| e.eq_ignore_ascii_case(cand))
    };
    if !is_taken(&clean) {
        return clean;
    }
    let custom = format!("{clean}_custom");
    if !is_taken(&custom) {
        return custom;
    }
    for n in 2..10_000 {
        let cand = format!("{clean}_{n}");
        if !is_taken(&cand) {
            return cand;
        }
    }
    format!("{clean}_user")
}

/// Clones any built-in or custom [`PostProfile`] into a mutable user custom profile
/// (`is_builtin = false`, `source = PostProfileSource::UserConfig`).
/// Rejects reserved built-in IDs with [`PostError::BuiltInReadOnly`].
pub fn clone_post_profile(
    source: &PostProfile,
    new_id: impl Into<String>,
    new_name: impl Into<String>,
) -> Result<PostProfile, PostError> {
    let new_id = new_id.into();
    validate_post_id(&new_id)?;
    if is_builtin_post_id(&new_id) {
        return Err(PostError::BuiltInReadOnly { id: new_id });
    }
    let mut cloned = source.clone();
    cloned.id = new_id;
    let name_str = new_name.into();
    cloned.name = if name_str.trim().is_empty() {
        format!("{} (Custom)", source.name)
    } else {
        name_str.trim().to_string()
    };
    cloned.is_builtin = false;
    cloned.source = PostProfileSource::UserConfig;
    Ok(cloned)
}

/// Saves a custom user [`PostProfile`] into `user_dir/<id>.post.toml`.
/// Rejects built-in / reserved post profile IDs with [`PostError::BuiltInReadOnly`].
pub fn save_user_post_profile_in(
    profile: &PostProfile,
    user_dir: &Path,
) -> Result<PathBuf, PostError> {
    validate_post_id(&profile.id)?;
    if profile.is_builtin || is_builtin_post_id(&profile.id) {
        return Err(PostError::BuiltInReadOnly {
            id: profile.id.clone(),
        });
    }
    std::fs::create_dir_all(user_dir).map_err(|e| PostError::Io {
        path: user_dir.display().to_string(),
        message: e.to_string(),
    })?;
    let dest = user_dir.join(format!("{}.post.toml", profile.id));
    let raw = profile.to_toml_string()?;
    std::fs::write(&dest, raw).map_err(|e| PostError::Io {
        path: dest.display().to_string(),
        message: e.to_string(),
    })?;
    Ok(dest)
}

/// Saves a custom user [`PostProfile`] into OS user config space (`dirs::config_dir()/chipcutter/posts/`).
pub fn save_user_post_profile(profile: &PostProfile) -> Result<PathBuf, PostError> {
    let dir = user_post_profiles_dir().ok_or(PostError::NoUserConfigDir)?;
    save_user_post_profile_in(profile, &dir)
}

/// Deletes a custom user `.post.toml` from `user_dir`. Rejects reserved built-in IDs.
pub fn delete_user_post_profile_in(id: &str, user_dir: &Path) -> Result<(), PostError> {
    validate_post_id(id)?;
    if is_builtin_post_id(id) {
        return Err(PostError::BuiltInReadOnly { id: id.to_string() });
    }
    let path = user_dir.join(format!("{id}.post.toml"));
    if !path.exists() {
        return Err(PostError::UnknownProfile {
            id_or_path: id.to_string(),
        });
    }
    std::fs::remove_file(&path).map_err(|e| PostError::Io {
        path: path.display().to_string(),
        message: e.to_string(),
    })
}

/// Deletes a custom user `.post.toml` from OS user config space.
pub fn delete_user_post_profile(id: &str) -> Result<(), PostError> {
    let dir = user_post_profiles_dir().ok_or(PostError::NoUserConfigDir)?;
    delete_user_post_profile_in(id, &dir)
}

/// Exports any [`PostProfile`] (built-in or custom) as a shareable `.post.toml` file.
pub fn export_post_profile_to_path(profile: &PostProfile, dest: &Path) -> Result<(), PostError> {
    if let Some(parent) = dest.parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent).map_err(|e| PostError::Io {
            path: parent.display().to_string(),
            message: e.to_string(),
        })?;
    }
    let raw = profile.to_toml_string()?;
    std::fs::write(dest, raw).map_err(|e| PostError::Io {
        path: dest.display().to_string(),
        message: e.to_string(),
    })
}

/// Imports a `.post.toml` file into `user_dir`, ensuring `is_builtin = false` and a unique
/// non-builtin ID so built-in post profiles can never be shadowed.
pub fn import_post_profile_from_path_in(
    src: &Path,
    user_dir: &Path,
    existing_ids: &[String],
) -> Result<PostProfile, PostError> {
    let mut prof = PostProfile::load_from_path(src)?;
    prof.is_builtin = false;
    prof.source = PostProfileSource::UserConfig;
    let unique = allocate_unique_custom_post_id(&prof.id, existing_ids);
    prof.id = unique;
    save_user_post_profile_in(&prof, user_dir)?;
    Ok(prof)
}

/// Imports a `.post.toml` file into OS user config space (`dirs::config_dir()/chipcutter/posts/`).
pub fn import_post_profile_from_path(src: &Path) -> Result<PostProfile, PostError> {
    let dir = user_post_profiles_dir().ok_or(PostError::NoUserConfigDir)?;
    let existing: Vec<String> = load_available_post_profiles_in(Some(&dir), None)
        .into_iter()
        .map(|p| p.id)
        .collect();
    import_post_profile_from_path_in(src, &dir, &existing)
}

/// Resolves a post profile strictly by profile ID (`grbl`, `grblhal_atc`, `linuxcnc`, or a
/// discovered custom ID) without probing arbitrary filesystem paths from untrusted project data.
pub fn resolve_post_profile(
    id: &str,
    extra_search_dir: Option<&Path>,
) -> Result<PostProfile, PostError> {
    let needle = id.trim().to_ascii_lowercase();
    let available = load_available_post_profiles(extra_search_dir);
    available
        .into_iter()
        .find(|p| p.id.to_ascii_lowercase() == needle)
        .ok_or_else(|| PostError::UnknownProfile {
            id_or_path: id.to_string(),
        })
}

/// Resolves an explicit CLI `--post` override, accepting either a profile ID or an explicit
/// `.post.toml` file path on disk chosen directly by the operator.
pub fn resolve_post_profile_cli_override(
    id_or_path: &str,
    extra_search_dir: Option<&Path>,
) -> Result<PostProfile, PostError> {
    let candidate_path = Path::new(id_or_path);
    if candidate_path.is_file() {
        let mut prof = PostProfile::load_from_path(candidate_path)?;
        prof.is_builtin = false;
        prof.source = PostProfileSource::ExplicitFile;
        return Ok(prof);
    }
    resolve_post_profile(id_or_path, extra_search_dir)
}
