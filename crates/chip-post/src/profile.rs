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

/// Circular arc center parameterization (`07-post-processor.md` §3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ArcCenterMode {
    #[default]
    IncrementalIjk,
    AbsoluteIjk,
    Radius,
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
}

impl PostProfile {
    pub fn from_toml_str(source_name: &str, toml_str: &str) -> Result<Self, PostError> {
        toml::from_str(toml_str).map_err(|e| PostError::Parse {
            source_name: source_name.to_string(),
            message: e.to_string(),
        })
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
        if let Ok(p) = PostProfile::from_toml_str(name, src) {
            out.push(p);
        }
    }
    out
}

/// Returns the user override post directory (`~/.config/chipcutter/posts` on Linux).
pub fn user_post_profiles_dir() -> Option<PathBuf> {
    dirs::config_dir().map(|d| d.join("chipcutter").join("posts"))
}

/// Discovers all available post profiles, merging built-in profiles with `.post.toml` files
/// in `./posts/`, `extra_search_dir`, and `~/.config/chipcutter/posts/`.
pub fn load_available_post_profiles(extra_search_dir: Option<&Path>) -> Vec<PostProfile> {
    let mut profiles = builtin_post_profiles();

    let mut search_dirs: Vec<PathBuf> = vec![PathBuf::from("posts")];
    if let Some(extra) = extra_search_dir {
        search_dirs.push(extra.to_path_buf());
        search_dirs.push(extra.join("posts"));
    }
    if let Some(user_dir) = user_post_profiles_dir() {
        search_dirs.push(user_dir);
    }

    for dir in search_dirs {
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
            if let Ok(prof) = PostProfile::load_from_path(&p) {
                if let Some(existing) = profiles.iter_mut().find(|x| x.id == prof.id) {
                    *existing = prof;
                } else {
                    profiles.push(prof);
                }
            }
        }
    }

    profiles
}

/// Resolves a post profile either from a file path (`*.toml`) or by profile ID (`grbl`, `grblhal_atc`, `linuxcnc`).
pub fn resolve_post_profile(
    id_or_path: &str,
    extra_search_dir: Option<&Path>,
) -> Result<PostProfile, PostError> {
    let candidate_path = Path::new(id_or_path);
    if candidate_path.is_file() {
        return PostProfile::load_from_path(candidate_path);
    }
    let needle = id_or_path.trim().to_ascii_lowercase();
    let available = load_available_post_profiles(extra_search_dir);
    available
        .into_iter()
        .find(|p| p.id.to_ascii_lowercase() == needle)
        .ok_or_else(|| PostError::UnknownProfile {
            id_or_path: id_or_path.to_string(),
        })
}
