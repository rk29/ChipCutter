//! Cross-session persistent desktop user preferences (`~/.config/chipcutter/prefs.toml`).

use std::path::{Path, PathBuf};

use chip_math::DisplayUnit;
use chip_sim::{SimComputeBackendChoice, SimEngineKind, SimResolution};
use serde::{Deserialize, Serialize};

use crate::{
    camera::{MousePreset, OrbitMode},
    layout::WorkspaceLayoutShares,
    theme::AppTheme,
};

/// Maximum number of recent project / CAD / vector file paths retained in `prefs.toml`.
pub const MAX_RECENT_FILES: usize = 10;

fn default_schema_version() -> u32 {
    1
}

fn default_true() -> bool {
    true
}

fn default_machine_id() -> String {
    chip_tools::DEFAULT_MACHINE_ID.to_string()
}

/// Persistent user preferences stored in `prefs.toml` (`08-ui-viewport.md` §Persistent User Preferences).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct UserPreferences {
    #[serde(default = "default_schema_version")]
    pub schema_version: u32,
    pub theme: AppTheme,
    pub default_units: DisplayUnit,
    #[serde(default = "default_machine_id")]
    pub default_machine_id: String,
    pub mouse_preset: MousePreset,
    pub orbit_mode: OrbitMode,
    #[serde(default = "default_true")]
    pub orthographic: bool,
    #[serde(default = "default_true")]
    pub show_toolpaths: bool,
    #[serde(default = "default_true")]
    pub show_rapids: bool,
    #[serde(default = "default_true")]
    pub show_stock: bool,
    #[serde(default = "default_true")]
    pub show_fixtures: bool,
    pub ghost_part: bool,
    /// Automatically rotate displayed 3D CAD model, raw setup stock, chuck jaws, Z planes, and
    /// cutting-area preview to match the selected `Indexed 3+1` operation's index angle.
    #[serde(default = "default_true")]
    pub auto_rotate_indexed_op: bool,
    #[serde(default)]
    pub sim_engine: SimEngineKind,
    #[serde(default)]
    pub sim_resolution: SimResolution,
    #[serde(default)]
    pub sim_backend: SimComputeBackendChoice,
    #[serde(default = "default_true")]
    pub sim_mesh_skinning: bool,
    pub layout_shares: WorkspaceLayoutShares,
    #[serde(default = "default_true")]
    pub show_welcome_on_startup: bool,
    pub recent_files: Vec<PathBuf>,
}

impl Default for UserPreferences {
    fn default() -> Self {
        Self {
            schema_version: 1,
            theme: AppTheme::Dark,
            default_units: DisplayUnit::Millimeter,
            default_machine_id: default_machine_id(),
            mouse_preset: MousePreset::Fusion360,
            orbit_mode: OrbitMode::TurntableZUp,
            orthographic: true,
            show_toolpaths: true,
            show_rapids: true,
            show_stock: true,
            show_fixtures: true,
            ghost_part: false,
            auto_rotate_indexed_op: true,
            sim_engine: SimEngineKind::Heightmap,
            sim_resolution: SimResolution::Standard,
            sim_backend: SimComputeBackendChoice::Auto,
            sim_mesh_skinning: true,
            layout_shares: WorkspaceLayoutShares::default(),
            show_welcome_on_startup: true,
            recent_files: Vec::new(),
        }
    }
}

impl UserPreferences {
    pub fn from_toml_str(text: &str) -> Result<Self, String> {
        let mut parsed: Self = toml::from_str(text).map_err(|e| e.to_string())?;
        parsed.layout_shares = parsed.layout_shares.clamped();
        if parsed.recent_files.len() > MAX_RECENT_FILES {
            parsed.recent_files.truncate(MAX_RECENT_FILES);
        }
        Ok(parsed)
    }

    pub fn to_toml_string(&self) -> Result<String, String> {
        let mut norm = self.clone();
        norm.layout_shares = norm.layout_shares.clamped();
        toml::to_string_pretty(&norm).map_err(|e| e.to_string())
    }

    /// Inserts `path` at index `0` of `recent_files`, deduplicating matching paths and capping at
    /// [`MAX_RECENT_FILES`].
    pub fn push_recent_file(&mut self, path: &Path) {
        let normalized = std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
        self.recent_files
            .retain(|existing| existing != &normalized && existing != path);
        self.recent_files.insert(0, normalized);
        if self.recent_files.len() > MAX_RECENT_FILES {
            self.recent_files.truncate(MAX_RECENT_FILES);
        }
    }

    pub fn clear_recent_files(&mut self) {
        self.recent_files.clear();
    }
}

/// Returns the default OS user configuration path for `prefs.toml`
/// (e.g., `~/.config/chipcutter/prefs.toml` on Linux).
pub fn default_prefs_path() -> Option<PathBuf> {
    dirs::config_dir().map(|base| base.join("chipcutter").join("prefs.toml"))
}

/// Loads `prefs.toml` from [`default_prefs_path`], falling back cleanly to [`UserPreferences::default`]
/// if the file does not exist or cannot be parsed.
pub fn load_or_default_prefs() -> UserPreferences {
    if let Some(path) = default_prefs_path()
        && let Ok(text) = std::fs::read_to_string(&path)
        && let Ok(prefs) = UserPreferences::from_toml_str(&text)
    {
        return prefs;
    }
    UserPreferences::default()
}

/// Persists `prefs` to [`default_prefs_path`].
pub fn save_prefs(prefs: &UserPreferences) -> Result<(), String> {
    let Some(path) = default_prefs_path() else {
        return Ok(());
    };
    save_prefs_to_path(prefs, &path)
}

/// Persists `prefs` to an explicit `path` (used by unit tests and [`save_prefs`]).
pub fn save_prefs_to_path(prefs: &UserPreferences, path: &Path) -> Result<(), String> {
    let text = prefs.to_toml_string()?;
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    std::fs::write(path, text).map_err(|e| e.to_string())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::layout::{create_workspace_tree_with_shares, extract_workspace_shares};

    #[test]
    fn preferences_roundtrip_toml_backward_compat_and_clamp_shares() {
        let mut prefs = UserPreferences {
            theme: AppTheme::Light,
            default_units: DisplayUnit::Inch,
            default_machine_id: "milo-1.5".to_string(),
            mouse_preset: MousePreset::SolidWorks,
            orbit_mode: OrbitMode::FreeTrackball,
            orthographic: false,
            show_rapids: false,
            ghost_part: true,
            auto_rotate_indexed_op: false,
            sim_engine: SimEngineKind::TriDexel,
            sim_backend: SimComputeBackendChoice::Cpu,
            layout_shares: WorkspaceLayoutShares {
                browser_share: 0.25,
                viewport_share: 0.50,
                inspector_share: 0.25,
                top_row_share: 0.70,
            },
            show_welcome_on_startup: false,
            ..UserPreferences::default()
        };
        prefs.push_recent_file(Path::new("/tmp/bracket_job.chip.toml"));

        let text = prefs.to_toml_string().unwrap();
        let decoded = UserPreferences::from_toml_str(&text).unwrap();
        assert_eq!(decoded, prefs);

        // Minimal partial TOML falls back cleanly via `#[serde(default)]` and clamps extreme shares
        let minimal = r#"
            theme = "light"
            [layout_shares]
            browser_share = 99.0
            viewport_share = -5.0
            inspector_share = 0.2
            top_row_share = 0.99
        "#;
        let partial = UserPreferences::from_toml_str(minimal).unwrap();
        assert_eq!(partial.theme, AppTheme::Light);
        assert_eq!(partial.default_machine_id, chip_tools::DEFAULT_MACHINE_ID);
        assert_eq!(partial.sim_engine, SimEngineKind::Heightmap);
        assert_eq!(partial.sim_backend, SimComputeBackendChoice::Auto);
        assert!(partial.show_toolpaths);
        assert!(partial.show_fixtures);
        assert!(partial.auto_rotate_indexed_op);
        assert!(partial.show_welcome_on_startup);
        assert!(partial.layout_shares.top_row_share <= 0.88 + 1e-6);
        let sum = partial.layout_shares.browser_share
            + partial.layout_shares.viewport_share
            + partial.layout_shares.inspector_share;
        assert!((sum - 1.0).abs() < 1e-5);

        // Workspace tree share round-trip
        let tree = create_workspace_tree_with_shares(prefs.layout_shares);
        let extracted = extract_workspace_shares(&tree);
        assert!(extracted.approx_eq(prefs.layout_shares));
    }

    #[test]
    fn recent_files_deduplicate_mru_and_cap_at_ten_entries() {
        let mut prefs = UserPreferences::default();
        for idx in 0..14 {
            prefs.push_recent_file(Path::new(&format!("/tmp/nonexistent_job_{idx}.chip.toml")));
        }
        assert_eq!(prefs.recent_files.len(), MAX_RECENT_FILES);
        assert_eq!(
            prefs.recent_files[0],
            PathBuf::from("/tmp/nonexistent_job_13.chip.toml")
        );

        // Re-opening an older item moves it to index 0 without duplicating
        prefs.push_recent_file(Path::new("/tmp/nonexistent_job_7.chip.toml"));
        assert_eq!(prefs.recent_files.len(), MAX_RECENT_FILES);
        assert_eq!(
            prefs.recent_files[0],
            PathBuf::from("/tmp/nonexistent_job_7.chip.toml")
        );
        assert_eq!(
            prefs
                .recent_files
                .iter()
                .filter(|p| p.as_path() == Path::new("/tmp/nonexistent_job_7.chip.toml"))
                .count(),
            1
        );
    }

    #[test]
    fn sim_backend_preferences_roundtrip_all_variants() {
        for choice in SimComputeBackendChoice::ALL {
            let prefs = UserPreferences {
                sim_backend: choice,
                ..UserPreferences::default()
            };
            let toml_text = prefs.to_toml_string().unwrap();
            let decoded = UserPreferences::from_toml_str(&toml_text).unwrap();
            assert_eq!(decoded.sim_backend, choice);
        }
        // Legacy prefs.toml without `sim_backend`, `sim_resolution`, or `sim_mesh_skinning`
        // defaults cleanly.
        let legacy = r#"
            schema_version = 1
            theme = "dark"
            sim_engine = "heightmap"
        "#;
        let decoded = UserPreferences::from_toml_str(legacy).unwrap();
        assert_eq!(decoded.sim_backend, SimComputeBackendChoice::Auto);
        assert_eq!(decoded.sim_resolution, SimResolution::Standard);
        assert!(decoded.sim_mesh_skinning);

        for res in SimResolution::ALL {
            for skin in [false, true] {
                let prefs = UserPreferences {
                    sim_resolution: res,
                    sim_mesh_skinning: skin,
                    ..UserPreferences::default()
                };
                let toml_text = prefs.to_toml_string().unwrap();
                let rt = UserPreferences::from_toml_str(&toml_text).unwrap();
                assert_eq!(rt.sim_resolution, res);
                assert_eq!(rt.sim_mesh_skinning, skin);
            }
        }
    }
}
