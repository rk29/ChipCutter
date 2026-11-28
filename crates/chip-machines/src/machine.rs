use std::path::{Path, PathBuf};

use chip_tools::{
    Tool, ToolId, ToolLibrary, ToolValidationError,
    preset_store::{
        PresetStoreError, PresetStoreSpec, allocate_unique_custom_id, delete_user_preset_in,
        discover_presets_in, export_preset_to_path, import_preset_from_path_in, resolve_preset_in,
        sanitize_slug_id, save_user_preset_in, validate_slug_id,
    },
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::builtin_machines::{
    CARVERA_AIR_MACHINE_ID, CARVERA_MACHINE_ID, CARVERA_Z1_MACHINE_ID, MILO_1_5_MACHINE_ID,
    MILO_2_0_MACHINE_ID, VORON_CASCADE_MACHINE_ID, builtin_carvera_air_machine,
    builtin_carvera_machine, builtin_carvera_z1_machine, builtin_milo_1_5_machine,
    builtin_milo_2_0_machine, builtin_voron_cascade_machine,
};

pub use crate::builtin_machines::builtin_generic_machine;

/// Canonical default machine profile ID used when a project omits `machine_id`.
pub const DEFAULT_MACHINE_ID: &str = "generic";

/// Constant table of all reserved read-only built-in machine IDs.
pub const BUILTIN_MACHINE_IDS: &[&str] = &[
    DEFAULT_MACHINE_ID,
    MILO_1_5_MACHINE_ID,
    MILO_2_0_MACHINE_ID,
    VORON_CASCADE_MACHINE_ID,
    CARVERA_MACHINE_ID,
    CARVERA_AIR_MACHINE_ID,
    CARVERA_Z1_MACHINE_ID,
];

/// Shared [`PresetStoreSpec`] governing `.machine.toml` user-config storage.
pub const MACHINE_PRESET_STORE_SPEC: PresetStoreSpec = PresetStoreSpec {
    subdir_name: "machines",
    file_suffix: ".machine.toml",
    builtin_ids: BUILTIN_MACHINE_IDS,
    slug_separator: '-',
    default_slug: "custom-machine",
    custom_suffix: "-custom",
    fallback_suffix: "-imported",
    allow_uppercase_ids: false,
    preserve_dots_in_sanitize: true,
};

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

fn map_preset_err(err: PresetStoreError<MachineStorageError>) -> MachineStorageError {
    match err {
        PresetStoreError::InvalidId { id } => MachineStorageError::InvalidMachineId { id },
        PresetStoreError::BuiltInReadOnly { id } => MachineStorageError::BuiltInReadOnly { id },
        PresetStoreError::AlreadyExists { id } => MachineStorageError::MachineAlreadyExists { id },
        PresetStoreError::NotFound { id } => MachineStorageError::UnknownMachine { id_or_path: id },
        PresetStoreError::Io { path, source } => MachineStorageError::Io { path, source },
        PresetStoreError::Codec(e) => e,
    }
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rated_power_rpm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spindle_efficiency: Option<f64>,
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
            rated_power_rpm: None,
            spindle_efficiency: None,
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
    #[serde(default)]
    pub tools: Vec<Tool>,
}

impl MachineDefinition {
    pub const CURRENT_SCHEMA_VERSION: u32 = 1;

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

    pub fn sort_tools_deterministically(&mut self) {
        self.tools.sort_by(|a, b| {
            a.tool_number
                .cmp(&b.tool_number)
                .then_with(|| a.id.cmp(&b.id))
        });
    }

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
        if let Some(rpm) = s.rated_power_rpm
            && (!rpm.is_finite() || rpm <= 0.0)
        {
            return Err(MachineStorageError::Validation {
                id: self.id.clone(),
                message: "spindle rated_power_rpm must be finite and > 0".to_string(),
            });
        }
        if let Some(eff) = s.spindle_efficiency
            && (!eff.is_finite() || !(0.05..=1.0).contains(&eff))
        {
            return Err(MachineStorageError::Validation {
                id: self.id.clone(),
                message: "spindle spindle_efficiency must be finite and in 0.05..=1.0".to_string(),
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
        export_preset_to_path(self, path, Self::to_toml_string).map_err(map_preset_err)
    }
}

/// Validates a machine slug ID (`[a-z0-9._-]`, non-empty, no leading `.`, no `..`).
pub fn validate_machine_id(id: &str) -> Result<(), MachineStorageError> {
    if !validate_slug_id(id, MACHINE_PRESET_STORE_SPEC.allow_uppercase_ids) {
        return Err(MachineStorageError::InvalidMachineId { id: id.to_string() });
    }
    Ok(())
}

/// Converts an arbitrary user string into a safe machine ID slug.
pub fn sanitize_machine_id(raw: &str) -> String {
    sanitize_slug_id(raw, &MACHINE_PRESET_STORE_SPEC)
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

/// Returns `true` if `id` is a reserved read-only built-in machine ID (O(1) table lookup).
pub fn is_builtin_machine_id(id: &str) -> bool {
    MACHINE_PRESET_STORE_SPEC.is_builtin_id(id)
}

/// Returns the upgrade-safe OS user configuration directory for custom machines
/// (e.g., `~/.config/chipcutter/machines` on Linux).
pub fn user_machines_dir() -> Option<PathBuf> {
    MACHINE_PRESET_STORE_SPEC.user_dir()
}

/// Discovers all built-in (read-only) and custom user machine definitions.
pub fn load_available_machines_in(
    user_dir: Option<&Path>,
    extra_search_dir: Option<&Path>,
) -> Vec<MachineDefinition> {
    discover_presets_in(
        builtin_machine_definitions(),
        user_dir,
        extra_search_dir,
        &MACHINE_PRESET_STORE_SPEC,
        |p| MachineDefinition::load_from_path(p).ok(),
        |m| &m.id,
        |m| m.is_builtin,
        |m, _origin| m.is_builtin = false,
    )
}

/// Discovers all built-in and custom machines using the default OS user config directory.
pub fn load_available_machines(extra_search_dir: Option<&Path>) -> Vec<MachineDefinition> {
    let udir = user_machines_dir();
    load_available_machines_in(udir.as_deref(), extra_search_dir)
}

/// Resolves a machine definition strictly by ID without probing arbitrary filesystem paths.
pub fn resolve_machine_definition_in(
    id: &str,
    user_dir: Option<&Path>,
    extra_search_dir: Option<&Path>,
) -> Result<MachineDefinition, MachineStorageError> {
    let available = load_available_machines_in(user_dir, extra_search_dir);
    resolve_preset_in(id, available, |m| &m.id).ok_or_else(|| MachineStorageError::UnknownMachine {
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
pub fn save_user_machine_in(
    machine: &MachineDefinition,
    user_dir: &Path,
) -> Result<PathBuf, MachineStorageError> {
    save_user_preset_in(
        machine,
        user_dir,
        &MACHINE_PRESET_STORE_SPEC,
        |m| &m.id,
        |m| m.is_builtin,
        MachineDefinition::to_toml_string,
    )
    .map_err(map_preset_err)
}

/// Saves a custom user [`MachineDefinition`] into the OS user config directory.
pub fn save_user_machine(machine: &MachineDefinition) -> Result<PathBuf, MachineStorageError> {
    let dir = user_machines_dir().ok_or(MachineStorageError::NoUserConfigDir)?;
    save_user_machine_in(machine, &dir)
}

/// Deletes a custom user machine from `user_dir`. Rejects reserved built-in IDs.
pub fn delete_user_machine_in(id: &str, user_dir: &Path) -> Result<(), MachineStorageError> {
    delete_user_preset_in(id, user_dir, &MACHINE_PRESET_STORE_SPEC).map_err(|err| match err {
        PresetStoreError::InvalidId { id } => MachineStorageError::InvalidMachineId { id },
        PresetStoreError::BuiltInReadOnly { id } => MachineStorageError::BuiltInReadOnly { id },
        PresetStoreError::NotFound { id } => MachineStorageError::UnknownMachine { id_or_path: id },
        PresetStoreError::Io { path, source } => MachineStorageError::Io { path, source },
        PresetStoreError::AlreadyExists { id } => MachineStorageError::MachineAlreadyExists { id },
        PresetStoreError::Codec(never) => match never {},
    })
}

/// Deletes a custom user machine from the OS user config directory.
pub fn delete_user_machine(id: &str) -> Result<(), MachineStorageError> {
    let dir = user_machines_dir().ok_or(MachineStorageError::NoUserConfigDir)?;
    delete_user_machine_in(id, &dir)
}

/// Exports any machine definition (built-in or custom) as a single self-contained `.machine.toml`.
pub fn export_machine_to_path(
    machine: &MachineDefinition,
    dest: &Path,
) -> Result<(), MachineStorageError> {
    machine.save_to_path(dest)
}

/// Picks a unique non-builtin custom machine ID given `base_id` and `existing_ids`.
pub fn allocate_unique_custom_machine_id(base_id: &str, existing_ids: &[String]) -> String {
    allocate_unique_custom_id(base_id, existing_ids, &MACHINE_PRESET_STORE_SPEC)
}

/// Validates and imports a shared `.machine.toml` file into `user_dir`, ensuring `is_builtin = false`.
pub fn import_machine_from_path_in(
    src: &Path,
    user_dir: &Path,
    existing_ids: &[String],
    id_override: Option<&str>,
) -> Result<MachineDefinition, MachineStorageError> {
    import_preset_from_path_in(
        src,
        user_dir,
        existing_ids,
        id_override,
        &MACHINE_PRESET_STORE_SPEC,
        MachineDefinition::load_from_path,
        |m| &m.id,
        |m, id| m.id = id,
        |m| m.is_builtin = false,
        |m| m.is_builtin,
        MachineDefinition::to_toml_string,
    )
    .map_err(map_preset_err)
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
