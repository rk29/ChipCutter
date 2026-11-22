use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    builtin_machines::{
        builtin_carvera_air_machine, builtin_carvera_machine, builtin_carvera_z1_machine,
        builtin_milo_1_5_machine, builtin_milo_2_0_machine, builtin_voron_cascade_machine,
    },
    model::{Tool, ToolId, ToolValidationError},
    storage::ToolLibrary,
};

pub use crate::builtin_machines::builtin_generic_machine;

/// Canonical default machine profile ID used when a project omits `machine_id`.
pub const DEFAULT_MACHINE_ID: &str = "generic";

/// Errors raised when reading, validating, cloning, importing, exporting, or saving a `.machine.toml`.
#[derive(Debug, Error)]
pub enum MachineStorageError {
    #[error("I/O error accessing machine definition '{path}': {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to parse `.machine.toml`: {0}")]
    TomlParse(#[from] toml::de::Error),
    #[error("failed to serialize `.machine.toml`: {0}")]
    TomlSerialize(#[from] toml::ser::Error),
    #[error("unsupported machine definition schema version {found} (max supported: {supported})")]
    UnsupportedSchema { found: u32, supported: u32 },
    #[error("invalid machine definition '{id}': {message}")]
    Validation { id: String, message: String },
    #[error("invalid tool in machine '{machine_id}': {source}")]
    ToolValidation {
        machine_id: String,
        #[source]
        source: ToolValidationError,
    },
    #[error(
        "built-in machine definition '{id}' is read-only; clone it to create an editable custom machine"
    )]
    BuiltInReadOnly { id: String },
    #[error(
        "invalid machine id '{id}': must be non-empty and contain only lowercase ASCII letters, digits, '-', '_', or '.'"
    )]
    InvalidMachineId { id: String },
    #[error("custom machine '{id}' already exists")]
    MachineAlreadyExists { id: String },
    #[error("unknown machine definition '{id_or_path}'")]
    UnknownMachine { id_or_path: String },
    #[error("could not determine OS user configuration directory for custom machines")]
    NoUserConfigDir,
}

/// Physical XYZ axis travel envelope (`mm`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct MachineTravelEnvelope {
    pub x_mm: f64,
    pub y_mm: f64,
    pub z_mm: f64,
}

impl Default for MachineTravelEnvelope {
    fn default() -> Self {
        Self {
            x_mm: 400.0,
            y_mm: 300.0,
            z_mm: 150.0,
        }
    }
}

/// Spindle speed range, power rating, and collet/shank capabilities.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MachineSpindleSpec {
    pub min_rpm: f64,
    pub max_rpm: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub power_watts: Option<f64>,
    #[serde(default = "default_collet_type")]
    pub collet_type: String,
    #[serde(default)]
    pub supported_shank_diameters_mm: Vec<f64>,
}

fn default_collet_type() -> String {
    "ER16".to_string()
}

impl Default for MachineSpindleSpec {
    fn default() -> Self {
        Self {
            min_rpm: 1_000.0,
            max_rpm: 24_000.0,
            power_watts: Some(1_500.0),
            collet_type: "ER16 / ER20".to_string(),
            supported_shank_diameters_mm: vec![3.0, 3.175, 4.0, 6.0, 6.35, 8.0, 10.0, 12.0],
        }
    }
}

/// Motion limits and cycle-time kinematics for a machine definition.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct MachineKinematicsSpec {
    pub max_rapid_xy_mm_min: f64,
    pub max_rapid_z_mm_min: f64,
    pub max_cut_feed_xy_mm_min: f64,
    pub max_cut_feed_z_mm_min: f64,
    pub accel_mm_s2: f64,
    pub junction_deviation_mm: f64,
    pub tool_change_time_sec: f64,
    #[serde(default = "default_max_rotary_rate_deg_min")]
    pub max_rotary_rate_deg_min: f64,
}

fn default_max_rotary_rate_deg_min() -> f64 {
    3_600.0
}

impl Default for MachineKinematicsSpec {
    fn default() -> Self {
        Self {
            max_rapid_xy_mm_min: 5_000.0,
            max_rapid_z_mm_min: 2_500.0,
            max_cut_feed_xy_mm_min: 5_000.0,
            max_cut_feed_z_mm_min: 2_500.0,
            accel_mm_s2: 500.0,
            junction_deviation_mm: 0.05,
            tool_change_time_sec: 15.0,
            max_rotary_rate_deg_min: default_max_rotary_rate_deg_min(),
        }
    }
}

/// Physical tool-changing mechanism on a CNC machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MachineToolChangeKind {
    #[default]
    Manual,
    QuickChange,
    AutomaticAtc,
}

impl MachineToolChangeKind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Manual => "Manual Collet",
            Self::QuickChange => "Quick-Change Lever",
            Self::AutomaticAtc => "Automatic Tool Changer (ATC)",
        }
    }
}

/// Optional 4th-axis rotary module capabilities and default chuck geometry.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MachineRotaryDefaults {
    /// Rotary axis letter (`"A"` around X, `"B"` around Y).
    #[serde(default = "default_rotary_axis")]
    pub axis: String,
    pub max_stock_diameter_mm: f64,
    pub max_stock_length_mm: f64,
    pub chuck_diameter_mm: f64,
    pub chuck_length_mm: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub default_post_profile_4axis: Option<String>,
}

fn default_rotary_axis() -> String {
    "A".to_string()
}

fn default_post_profile() -> String {
    "grbl".to_string()
}

fn default_schema_version() -> u32 {
    MachineDefinition::CURRENT_SCHEMA_VERSION
}

/// Complete, self-contained `.machine.toml` CNC machine definition including its dedicated tool set.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MachineDefinition {
    #[serde(default = "default_schema_version")]
    pub schema_version: u32,
    pub id: String,
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub manufacturer: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// True for compiled-in read-only presets (`generic`, `milo-1.5`, `milo-2.0`, `voron-cascade`,
    /// `carvera`, `carvera-air`, `carvera-z1`); never serialized.
    #[serde(skip)]
    pub is_builtin: bool,
    #[serde(default)]
    pub travel_mm: MachineTravelEnvelope,
    #[serde(default)]
    pub spindle: MachineSpindleSpec,
    #[serde(default)]
    pub kinematics: MachineKinematicsSpec,
    #[serde(default)]
    pub tool_change: MachineToolChangeKind,
    #[serde(default)]
    pub has_toolsetter: bool,
    #[serde(default = "default_post_profile")]
    pub default_post_profile: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rotary: Option<MachineRotaryDefaults>,
    /// Dedicated tool library belonging to this machine definition.
    #[serde(default)]
    pub tools: Vec<Tool>,
}

impl MachineDefinition {
    pub const CURRENT_SCHEMA_VERSION: u32 = 1;

    /// Returns a [`ToolLibrary`] view of this machine's dedicated tool set.
    pub fn to_tool_library(&self) -> ToolLibrary {
        let mut lib = ToolLibrary {
            schema_version: ToolLibrary::CURRENT_SCHEMA_VERSION,
            name: format!("{} — Tools", self.name),
            tools: self.tools.clone(),
        };
        lib.sort_deterministically();
        lib
    }

    pub fn tool(&self, id: &ToolId) -> Option<&Tool> {
        self.tools.iter().find(|t| &t.id == id)
    }

    /// Sorts tools deterministically by `(tool_number, id)` for clean diffs and sharing.
    pub fn sort_tools_deterministically(&mut self) {
        self.tools.sort_by(|a, b| {
            a.tool_number
                .cmp(&b.tool_number)
                .then_with(|| a.id.cmp(&b.id))
        });
    }

    /// Validates all physical invariants and every tool in this machine's library.
    pub fn validate(&self) -> Result<(), MachineStorageError> {
        if self.schema_version > Self::CURRENT_SCHEMA_VERSION {
            return Err(MachineStorageError::UnsupportedSchema {
                found: self.schema_version,
                supported: Self::CURRENT_SCHEMA_VERSION,
            });
        }
        validate_machine_id(&self.id)?;
        if self.name.trim().is_empty() {
            return Err(MachineStorageError::Validation {
                id: self.id.clone(),
                message: "machine name cannot be empty".to_string(),
            });
        }
        let t = &self.travel_mm;
        if !(t.x_mm.is_finite()
            && t.x_mm > 0.0
            && t.y_mm.is_finite()
            && t.y_mm > 0.0
            && t.z_mm.is_finite()
            && t.z_mm > 0.0)
        {
            return Err(MachineStorageError::Validation {
                id: self.id.clone(),
                message: "travel_mm (x_mm, y_mm, z_mm) must all be finite and > 0".to_string(),
            });
        }
        let s = &self.spindle;
        if !(s.min_rpm.is_finite()
            && s.max_rpm.is_finite()
            && s.min_rpm >= 0.0
            && s.max_rpm > 0.0
            && s.min_rpm <= s.max_rpm)
        {
            return Err(MachineStorageError::Validation {
                id: self.id.clone(),
                message: format!(
                    "spindle RPM range [{}, {}] is invalid (require 0 <= min_rpm <= max_rpm and max_rpm > 0)",
                    s.min_rpm, s.max_rpm
                ),
            });
        }
        if let Some(w) = s.power_watts
            && (!w.is_finite() || w <= 0.0)
        {
            return Err(MachineStorageError::Validation {
                id: self.id.clone(),
                message: "spindle power_watts must be finite and > 0".to_string(),
            });
        }
        let k = &self.kinematics;
        if !(k.max_rapid_xy_mm_min.is_finite()
            && k.max_rapid_xy_mm_min > 0.0
            && k.max_rapid_z_mm_min.is_finite()
            && k.max_rapid_z_mm_min > 0.0
            && k.max_cut_feed_xy_mm_min.is_finite()
            && k.max_cut_feed_xy_mm_min > 0.0
            && k.max_cut_feed_z_mm_min.is_finite()
            && k.max_cut_feed_z_mm_min > 0.0
            && k.accel_mm_s2.is_finite()
            && k.accel_mm_s2 > 0.0
            && k.junction_deviation_mm.is_finite()
            && k.junction_deviation_mm >= 0.0
            && k.tool_change_time_sec.is_finite()
            && k.tool_change_time_sec >= 0.0
            && k.max_rotary_rate_deg_min.is_finite()
            && k.max_rotary_rate_deg_min > 0.0)
        {
            return Err(MachineStorageError::Validation {
                id: self.id.clone(),
                message: "kinematics speeds/accel/rotary rates must be finite and positive"
                    .to_string(),
            });
        }
        if self.default_post_profile.trim().is_empty() {
            return Err(MachineStorageError::Validation {
                id: self.id.clone(),
                message: "default_post_profile cannot be empty".to_string(),
            });
        }
        if let Some(rot) = &self.rotary {
            let axis = rot.axis.trim();
            if !axis.eq_ignore_ascii_case("A") && !axis.eq_ignore_ascii_case("B") {
                return Err(MachineStorageError::Validation {
                    id: self.id.clone(),
                    message: format!("rotary.axis '{}' must be 'A' or 'B'", rot.axis),
                });
            }
            if !(rot.max_stock_diameter_mm.is_finite()
                && rot.max_stock_diameter_mm > 0.0
                && rot.max_stock_length_mm.is_finite()
                && rot.max_stock_length_mm > 0.0
                && rot.chuck_diameter_mm.is_finite()
                && rot.chuck_diameter_mm > 0.0
                && rot.chuck_length_mm.is_finite()
                && rot.chuck_length_mm >= 0.0)
            {
                return Err(MachineStorageError::Validation {
                    id: self.id.clone(),
                    message: "rotary stock and chuck dimensions must be finite and > 0".to_string(),
                });
            }
        }
        for tool in &self.tools {
            tool.validate()
                .map_err(|source| MachineStorageError::ToolValidation {
                    machine_id: self.id.clone(),
                    source,
                })?;
        }
        Ok(())
    }

    pub fn to_toml_string(&self) -> Result<String, MachineStorageError> {
        let mut sorted = self.clone();
        sorted.sort_tools_deterministically();
        sorted.validate()?;
        Ok(toml::to_string_pretty(&sorted)?)
    }

    pub fn from_toml_str(text: &str) -> Result<Self, MachineStorageError> {
        let mut def: Self = toml::from_str(text)?;
        def.is_builtin = false;
        def.validate()?;
        def.sort_tools_deterministically();
        Ok(def)
    }

    pub fn load_from_path(path: &Path) -> Result<Self, MachineStorageError> {
        let content = std::fs::read_to_string(path).map_err(|source| MachineStorageError::Io {
            path: path.display().to_string(),
            source,
        })?;
        Self::from_toml_str(&content)
    }

    pub fn save_to_path(&self, path: &Path) -> Result<(), MachineStorageError> {
        let text = self.to_toml_string()?;
        if let Some(parent) = path.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent).map_err(|source| MachineStorageError::Io {
                path: parent.display().to_string(),
                source,
            })?;
        }
        std::fs::write(path, text).map_err(|source| MachineStorageError::Io {
            path: path.display().to_string(),
            source,
        })
    }
}

/// Validates a machine slug ID (`[a-z0-9._-]`, non-empty, no leading `.`, no `..`).
pub fn validate_machine_id(id: &str) -> Result<(), MachineStorageError> {
    if id.is_empty()
        || id.starts_with('.')
        || id.contains("..")
        || !id.bytes().all(|b| {
            b.is_ascii_lowercase() || b.is_ascii_digit() || matches!(b, b'-' | b'_' | b'.')
        })
    {
        return Err(MachineStorageError::InvalidMachineId { id: id.to_string() });
    }
    Ok(())
}

/// Converts an arbitrary user string into a safe machine ID slug.
pub fn sanitize_machine_id(raw: &str) -> String {
    let mut out = String::new();
    let mut prev_dash = false;
    for ch in raw.trim().chars() {
        let lower = ch.to_ascii_lowercase();
        if lower.is_ascii_lowercase() || lower.is_ascii_digit() || matches!(lower, '_' | '.') {
            if out.is_empty() && lower == '.' {
                continue;
            }
            if lower == '.' && out.ends_with('.') {
                continue;
            }
            out.push(lower);
            prev_dash = false;
        } else if !out.is_empty() && !prev_dash {
            out.push('-');
            prev_dash = true;
        }
    }
    while out.ends_with('-') || out.ends_with('.') {
        out.pop();
    }
    if out.is_empty() {
        "custom-machine".to_string()
    } else {
        out
    }
}

/// Returns all compiled-in read-only [`MachineDefinition`] presets
/// (`generic`, `milo-1.5`, `milo-2.0`, `voron-cascade`, `carvera`, `carvera-air`, and `carvera-z1`).
pub fn builtin_machine_definitions() -> Vec<MachineDefinition> {
    vec![
        builtin_generic_machine(),
        builtin_milo_1_5_machine(),
        builtin_milo_2_0_machine(),
        builtin_voron_cascade_machine(),
        builtin_carvera_machine(),
        builtin_carvera_air_machine(),
        builtin_carvera_z1_machine(),
    ]
}

/// Returns `true` if `id` is a reserved read-only built-in machine ID.
pub fn is_builtin_machine_id(id: &str) -> bool {
    let needle = id.trim();
    builtin_machine_definitions()
        .iter()
        .any(|m| m.id.eq_ignore_ascii_case(needle))
}

/// Returns the upgrade-safe OS user configuration directory for custom machines
/// (e.g., `~/.config/chipcutter/machines` on Linux).
pub fn user_machines_dir() -> Option<PathBuf> {
    dirs::config_dir().map(|base| base.join("chipcutter").join("machines"))
}

/// Discovers all built-in (read-only) and custom user machine definitions.
///
/// Built-in definitions always appear first and cannot be shadowed by files in
/// `user_dir` or `extra_search_dir`, ensuring application upgrades update built-ins cleanly.
pub fn load_available_machines_in(
    user_dir: Option<&Path>,
    extra_search_dir: Option<&Path>,
) -> Vec<MachineDefinition> {
    let mut machines = builtin_machine_definitions();

    let mut search_dirs: Vec<PathBuf> = Vec::new();
    if let Some(extra) = extra_search_dir {
        search_dirs.push(extra.to_path_buf());
        search_dirs.push(extra.join("machines"));
    }
    if let Some(udir) = user_dir {
        search_dirs.push(udir.to_path_buf());
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
                    .is_some_and(|n| n.ends_with(".machine.toml"))
            })
            .collect();
        paths.sort();

        for p in paths {
            let Ok(mut def) = MachineDefinition::load_from_path(&p) else {
                continue;
            };
            // Never allow a user file to shadow a reserved built-in machine ID.
            if is_builtin_machine_id(&def.id) {
                continue;
            }
            def.is_builtin = false;
            if let Some(existing) = machines
                .iter_mut()
                .find(|m| !m.is_builtin && m.id.eq_ignore_ascii_case(&def.id))
            {
                *existing = def;
            } else {
                machines.push(def);
            }
        }
    }

    machines
}

/// Discovers all built-in and custom machines using the default OS user config directory.
pub fn load_available_machines(extra_search_dir: Option<&Path>) -> Vec<MachineDefinition> {
    let udir = user_machines_dir();
    load_available_machines_in(udir.as_deref(), extra_search_dir)
}

/// Resolves a machine definition strictly by ID (`generic`, `milo-1.5`, `milo-2.0`,
/// `voron-cascade`, `carvera`, `carvera-air`, `carvera-z1`, or a discovered custom ID) without
/// probing arbitrary filesystem paths from untrusted project files.
pub fn resolve_machine_definition_in(
    id: &str,
    user_dir: Option<&Path>,
    extra_search_dir: Option<&Path>,
) -> Result<MachineDefinition, MachineStorageError> {
    let needle = id.trim().to_ascii_lowercase();
    load_available_machines_in(user_dir, extra_search_dir)
        .into_iter()
        .find(|m| m.id.to_ascii_lowercase() == needle)
        .ok_or_else(|| MachineStorageError::UnknownMachine {
            id_or_path: id.to_string(),
        })
}

/// Resolves a machine definition strictly by ID using the default OS user config directory.
pub fn resolve_machine_definition(
    id: &str,
    extra_search_dir: Option<&Path>,
) -> Result<MachineDefinition, MachineStorageError> {
    let udir = user_machines_dir();
    resolve_machine_definition_in(id, udir.as_deref(), extra_search_dir)
}

/// Resolves an explicit CLI `--machine` / `machines inspect` argument, accepting either a
/// machine ID or an explicit `.machine.toml` file path provided directly by the operator.
pub fn resolve_machine_definition_cli_override_in(
    id_or_path: &str,
    user_dir: Option<&Path>,
    extra_search_dir: Option<&Path>,
) -> Result<MachineDefinition, MachineStorageError> {
    let candidate = Path::new(id_or_path);
    if candidate.is_file() {
        return MachineDefinition::load_from_path(candidate);
    }
    resolve_machine_definition_in(id_or_path, user_dir, extra_search_dir)
}

/// Resolves an explicit CLI machine override using the default OS user config directory.
pub fn resolve_machine_definition_cli_override(
    id_or_path: &str,
    extra_search_dir: Option<&Path>,
) -> Result<MachineDefinition, MachineStorageError> {
    let udir = user_machines_dir();
    resolve_machine_definition_cli_override_in(id_or_path, udir.as_deref(), extra_search_dir)
}

/// Clones any built-in or custom machine (including its complete tool set) into a new
/// mutable custom [`MachineDefinition`] with `is_builtin = false`.
pub fn clone_machine_definition(
    source: &MachineDefinition,
    new_id: impl Into<String>,
    new_name: impl Into<String>,
) -> Result<MachineDefinition, MachineStorageError> {
    let new_id = new_id.into();
    validate_machine_id(&new_id)?;
    if is_builtin_machine_id(&new_id) {
        return Err(MachineStorageError::BuiltInReadOnly { id: new_id });
    }
    let mut cloned = source.clone();
    cloned.id = new_id;
    cloned.name = new_name.into();
    cloned.is_builtin = false;
    cloned.validate()?;
    cloned.sort_tools_deterministically();
    Ok(cloned)
}

/// Saves a custom user [`MachineDefinition`] into `user_dir/<id>.machine.toml`.
/// Rejects built-in / reserved machine IDs with [`MachineStorageError::BuiltInReadOnly`].
pub fn save_user_machine_in(
    machine: &MachineDefinition,
    user_dir: &Path,
) -> Result<PathBuf, MachineStorageError> {
    if machine.is_builtin || is_builtin_machine_id(&machine.id) {
        return Err(MachineStorageError::BuiltInReadOnly {
            id: machine.id.clone(),
        });
    }
    machine.validate()?;
    let dest = user_dir.join(format!("{}.machine.toml", machine.id));
    machine.save_to_path(&dest)?;
    Ok(dest)
}

/// Saves a custom user [`MachineDefinition`] into the OS user config directory.
pub fn save_user_machine(machine: &MachineDefinition) -> Result<PathBuf, MachineStorageError> {
    let dir = user_machines_dir().ok_or(MachineStorageError::NoUserConfigDir)?;
    save_user_machine_in(machine, &dir)
}

/// Deletes a custom user machine from `user_dir`. Rejects reserved built-in IDs.
pub fn delete_user_machine_in(id: &str, user_dir: &Path) -> Result<(), MachineStorageError> {
    validate_machine_id(id)?;
    if is_builtin_machine_id(id) {
        return Err(MachineStorageError::BuiltInReadOnly { id: id.to_string() });
    }
    let path = user_dir.join(format!("{id}.machine.toml"));
    if !path.exists() {
        return Err(MachineStorageError::UnknownMachine {
            id_or_path: id.to_string(),
        });
    }
    std::fs::remove_file(&path).map_err(|source| MachineStorageError::Io {
        path: path.display().to_string(),
        source,
    })
}

/// Deletes a custom user machine from the OS user config directory.
pub fn delete_user_machine(id: &str) -> Result<(), MachineStorageError> {
    let dir = user_machines_dir().ok_or(MachineStorageError::NoUserConfigDir)?;
    delete_user_machine_in(id, &dir)
}

/// Exports any machine definition (built-in or custom) as a single self-contained,
/// shareable `.machine.toml` file (including its complete tool set).
pub fn export_machine_to_path(
    machine: &MachineDefinition,
    dest: &Path,
) -> Result<(), MachineStorageError> {
    machine.save_to_path(dest)
}

/// Picks a unique non-builtin custom machine ID given `base_id` and `existing_ids`.
pub fn allocate_unique_custom_machine_id(base_id: &str, existing_ids: &[String]) -> String {
    let clean = sanitize_machine_id(base_id);
    let is_taken = |cand: &str| -> bool {
        is_builtin_machine_id(cand) || existing_ids.iter().any(|e| e.eq_ignore_ascii_case(cand))
    };
    if !is_taken(&clean) {
        return clean;
    }
    let imported = format!("{clean}-custom");
    if !is_taken(&imported) {
        return imported;
    }
    for n in 2..10_000 {
        let cand = format!("{clean}-{n}");
        if !is_taken(&cand) {
            return cand;
        }
    }
    format!("{clean}-imported")
}

/// Validates and imports a shared `.machine.toml` file into `user_dir`, ensuring `is_builtin = false`.
///
/// If `id_override` is `Some`, it must be valid, non-builtin, and not already taken.
/// Otherwise, if the imported file's ID collides with a built-in or existing custom ID,
/// a unique non-colliding ID is allocated automatically.
pub fn import_machine_from_path_in(
    src: &Path,
    user_dir: &Path,
    existing_ids: &[String],
    id_override: Option<&str>,
) -> Result<MachineDefinition, MachineStorageError> {
    let mut def = MachineDefinition::load_from_path(src)?;
    def.is_builtin = false;

    if let Some(custom_id) = id_override {
        validate_machine_id(custom_id)?;
        if is_builtin_machine_id(custom_id) {
            return Err(MachineStorageError::BuiltInReadOnly {
                id: custom_id.to_string(),
            });
        }
        if existing_ids
            .iter()
            .any(|e| e.eq_ignore_ascii_case(custom_id))
        {
            return Err(MachineStorageError::MachineAlreadyExists {
                id: custom_id.to_string(),
            });
        }
        def.id = custom_id.to_string();
    } else {
        let unique = allocate_unique_custom_machine_id(&def.id, existing_ids);
        if unique != def.id {
            def.id = unique;
        }
    }

    save_user_machine_in(&def, user_dir)?;
    Ok(def)
}

/// Imports a shared `.machine.toml` into the OS user config directory.
pub fn import_machine_from_path(
    src: &Path,
    id_override: Option<&str>,
) -> Result<MachineDefinition, MachineStorageError> {
    let dir = user_machines_dir().ok_or(MachineStorageError::NoUserConfigDir)?;
    let existing: Vec<String> = load_available_machines_in(Some(&dir), None)
        .into_iter()
        .map(|m| m.id)
        .collect();
    import_machine_from_path_in(src, &dir, &existing, id_override)
}
