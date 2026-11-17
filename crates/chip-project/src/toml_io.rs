use std::path::Path;

use thiserror::Error;

use crate::document::ProjectDocument;

/// Errors reading or writing a `.chip.toml` project file.
#[derive(Debug, Error)]
pub enum ProjectIoError {
    #[error("I/O error accessing project '{path}': {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to parse `.chip.toml`: {0}")]
    TomlParse(#[from] toml::de::Error),
    #[error("failed to serialize `.chip.toml`: {0}")]
    TomlSerialize(#[from] toml::ser::Error),
    #[error("unsupported `.chip.toml` schema version {found} (max supported: {supported})")]
    UnsupportedSchema { found: u32, supported: u32 },
}

impl ProjectDocument {
    /// Serializes this project to deterministic, Git-diff-friendly TOML text.
    pub fn to_toml_string(&self) -> Result<String, ProjectIoError> {
        let mut normalized = self.clone();
        normalized.sort_tools_deterministically();
        Ok(toml::to_string_pretty(&normalized)?)
    }

    /// Parses a `.chip.toml` manifest from TOML text and checks `schema_version`.
    pub fn from_toml_str(toml_text: &str) -> Result<Self, ProjectIoError> {
        let doc: Self = toml::from_str(toml_text)?;
        if doc.schema_version > Self::CURRENT_SCHEMA_VERSION {
            return Err(ProjectIoError::UnsupportedSchema {
                found: doc.schema_version,
                supported: Self::CURRENT_SCHEMA_VERSION,
            });
        }
        Ok(doc)
    }

    pub fn load_from_path(path: &Path) -> Result<Self, ProjectIoError> {
        let content = std::fs::read_to_string(path).map_err(|source| ProjectIoError::Io {
            path: path.display().to_string(),
            source,
        })?;
        Self::from_toml_str(&content)
    }

    pub fn save_to_path(&self, path: &Path) -> Result<(), ProjectIoError> {
        let text = self.to_toml_string()?;
        if let Some(parent) = path.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent).map_err(|source| ProjectIoError::Io {
                path: parent.display().to_string(),
                source,
            })?;
        }
        std::fs::write(path, text).map_err(|source| ProjectIoError::Io {
            path: path.display().to_string(),
            source,
        })
    }
}
