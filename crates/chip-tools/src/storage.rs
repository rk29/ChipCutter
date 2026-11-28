use std::path::Path;
#[cfg(feature = "user-config")]
use std::path::PathBuf;

use serde::{Deserialize, Serialize};
use thiserror::Error;

#[cfg(feature = "user-config")]
use crate::preset_store::chipcutter_config_dir;
use crate::{
    builtin::default_starter_tool_library,
    model::{Tool, ToolId, ToolValidationError},
};

/// Errors reading, validating, or saving a `.tools.toml` library.
#[derive(Debug, Error)]
pub enum ToolStorageError {
    #[error("I/O error accessing tool library '{path}': {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to parse `.tools.toml`: {0}")]
    TomlParse(#[from] toml::de::Error),
    #[error("failed to serialize `.tools.toml`: {0}")]
    TomlSerialize(#[from] toml::ser::Error),
    #[error("unsupported tool library schema version {found} (max supported: {supported})")]
    UnsupportedSchema { found: u32, supported: u32 },
    #[error(transparent)]
    Validation(#[from] ToolValidationError),
}

/// Serializable `.tools.toml` collection of [`Tool`]s.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ToolLibrary {
    pub schema_version: u32,
    pub name: String,
    #[serde(default)]
    pub tools: Vec<Tool>,
}

impl Default for ToolLibrary {
    fn default() -> Self {
        default_starter_tool_library()
    }
}

impl ToolLibrary {
    pub const CURRENT_SCHEMA_VERSION: u32 = 1;

    pub fn tool(&self, id: &ToolId) -> Option<&Tool> {
        self.tools.iter().find(|t| &t.id == id)
    }

    pub fn tool_mut(&mut self, id: &ToolId) -> Option<&mut Tool> {
        self.tools.iter_mut().find(|t| &t.id == id)
    }

    /// Sorts tools deterministically by `(tool_number, id)` for clean `git diff` output.
    pub fn sort_deterministically(&mut self) {
        self.tools.sort_by(|a, b| {
            a.tool_number
                .cmp(&b.tool_number)
                .then_with(|| a.id.cmp(&b.id))
        });
    }

    pub fn validate_all(&self) -> Result<(), ToolStorageError> {
        if self.schema_version > Self::CURRENT_SCHEMA_VERSION {
            return Err(ToolStorageError::UnsupportedSchema {
                found: self.schema_version,
                supported: Self::CURRENT_SCHEMA_VERSION,
            });
        }
        for tool in &self.tools {
            tool.validate()?;
        }
        Ok(())
    }

    pub fn to_toml_string(&self) -> Result<String, ToolStorageError> {
        let mut sorted = self.clone();
        sorted.sort_deterministically();
        Ok(toml::to_string_pretty(&sorted)?)
    }

    pub fn from_toml_str(text: &str) -> Result<Self, ToolStorageError> {
        let lib: Self = toml::from_str(text)?;
        lib.validate_all()?;
        Ok(lib)
    }

    pub fn load_from_path(path: &Path) -> Result<Self, ToolStorageError> {
        let content = std::fs::read_to_string(path).map_err(|source| ToolStorageError::Io {
            path: path.display().to_string(),
            source,
        })?;
        Self::from_toml_str(&content)
    }

    pub fn save_to_path(&self, path: &Path) -> Result<(), ToolStorageError> {
        self.validate_all()?;
        let text = self.to_toml_string()?;
        if let Some(parent) = path.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent).map_err(|source| ToolStorageError::Io {
                path: parent.display().to_string(),
                source,
            })?;
        }
        std::fs::write(path, text).map_err(|source| ToolStorageError::Io {
            path: path.display().to_string(),
            source,
        })
    }
}

/// Returns the default OS user configuration path for the global ChipCutter tool library
/// (e.g., `~/.config/chipcutter/default.tools.toml` on Linux).
#[cfg(feature = "user-config")]
pub fn default_global_tool_library_path() -> Option<PathBuf> {
    chipcutter_config_dir().map(|base| base.join("default.tools.toml"))
}

/// Loads the user's global `.tools.toml` if present, otherwise initializes and returns the
/// built-in starter library.
#[cfg(feature = "user-config")]
pub fn load_or_default_global_library() -> ToolLibrary {
    if let Some(path) = default_global_tool_library_path()
        && path.exists()
        && let Ok(lib) = ToolLibrary::load_from_path(&path)
    {
        return lib;
    }
    default_starter_tool_library()
}
