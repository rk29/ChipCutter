use std::path::{Component, Path, PathBuf};

use chip_cam::{Setup, WorkOffsetSlot};
use chip_math::DisplayUnit;
use chip_tools::{Tool, ToolId};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// External STEP CAD model reference stored in `.chip.toml` (`09-persistence-cli.md`).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ModelEntry {
    pub id: String,
    pub path: String,
    pub sha256: String,
}

/// Top-level ChipCutter `.chip.toml` project document (`schema_version = 1`).
///
/// Kept lightweight and free of binary/tessellation blobs so `git diff` stays clean and
/// in-memory Undo/Redo snapshots take `< 1 ms`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProjectDocument {
    pub schema_version: u32,
    pub project_name: String,
    #[serde(default)]
    pub display_units: DisplayUnit,
    #[serde(default)]
    pub models: Vec<ModelEntry>,
    /// Project-embedded snapshots of tools referenced by operations (`04-tool-library.md`).
    #[serde(default)]
    pub tools: Vec<Tool>,
    #[serde(default)]
    pub setups: Vec<Setup>,
}

impl Default for ProjectDocument {
    fn default() -> Self {
        Self::new("Untitled Job")
    }
}

impl ProjectDocument {
    pub const CURRENT_SCHEMA_VERSION: u32 = 1;

    /// Creates a fresh project initialized with `Setup 1 — Op 1 (G54)`.
    pub fn new(project_name: impl Into<String>) -> Self {
        Self {
            schema_version: Self::CURRENT_SCHEMA_VERSION,
            project_name: project_name.into(),
            display_units: DisplayUnit::Millimeter,
            models: Vec::new(),
            tools: Vec::new(),
            setups: vec![Setup::new(
                "setup_1",
                "Setup 1 — Top Op",
                WorkOffsetSlot::G54,
            )],
        }
    }

    pub fn tool(&self, id: &ToolId) -> Option<&Tool> {
        self.tools.iter().find(|t| &t.id == id)
    }

    pub fn tool_mut(&mut self, id: &ToolId) -> Option<&mut Tool> {
        self.tools.iter_mut().find(|t| &t.id == id)
    }

    /// Embeds a snapshot of `global_tool` into `self.tools` if not already present.
    pub fn ensure_tool_embedded(&mut self, global_tool: &Tool) {
        if self.tool(&global_tool.id).is_none() {
            self.tools.push(global_tool.clone());
            self.sort_tools_deterministically();
        }
    }

    /// Explicitly overwrites an embedded project tool with an updated definition from the global library,
    /// marking any operations referencing that tool as `Dirty`.
    pub fn sync_tool_from_global(&mut self, updated_tool: &Tool) {
        if let Some(existing) = self.tool_mut(&updated_tool.id) {
            *existing = updated_tool.clone();
        } else {
            self.tools.push(updated_tool.clone());
        }
        self.sort_tools_deterministically();
        for setup in &mut self.setups {
            for op in &mut setup.operations {
                if op.tool_id.as_ref() == Some(&updated_tool.id) {
                    op.mark_dirty();
                }
            }
        }
    }

    pub fn sort_tools_deterministically(&mut self) {
        self.tools.sort_by(|a, b| {
            a.tool_number
                .cmp(&b.tool_number)
                .then_with(|| a.id.cmp(&b.id))
        });
    }
}

/// Computes lowercase `64`-character SHA-256 hex digest of `bytes`.
pub fn compute_sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut out = String::with_capacity(64);
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for &b in digest.as_slice() {
        out.push(HEX[(b >> 4) as usize] as char);
        out.push(HEX[(b & 0x0f) as usize] as char);
    }
    out
}

/// Computes a portable `/` relative path from `project_file`'s directory to `step_path` whenever
/// they share a common directory prefix; falls back to `step_path` otherwise.
pub fn portable_model_path(project_file: Option<&Path>, step_path: &Path) -> String {
    let Some(proj) = project_file else {
        return normalize_slashes(&step_path.to_string_lossy());
    };
    let Some(proj_dir) = proj.parent() else {
        return normalize_slashes(&step_path.to_string_lossy());
    };
    let proj_abs = std::fs::canonicalize(proj_dir).unwrap_or_else(|_| proj_dir.to_path_buf());
    let step_abs = std::fs::canonicalize(step_path).unwrap_or_else(|_| step_path.to_path_buf());

    if let Ok(rel) = step_abs.strip_prefix(&proj_abs) {
        return normalize_slashes(&rel.to_string_lossy());
    }

    // Also handle sibling directories (e.g., `../cad/part.step`) up to 2 parent hops
    let p_comps: Vec<Component<'_>> = proj_abs.components().collect();
    let s_comps: Vec<Component<'_>> = step_abs.components().collect();
    let common = p_comps
        .iter()
        .zip(s_comps.iter())
        .take_while(|(a, b)| a == b)
        .count();

    if common > 1 && p_comps.len().saturating_sub(common) <= 2 {
        let mut rel = PathBuf::new();
        for _ in common..p_comps.len() {
            rel.push("..");
        }
        for c in &s_comps[common..] {
            rel.push(c.as_os_str());
        }
        return normalize_slashes(&rel.to_string_lossy());
    }

    normalize_slashes(&step_path.to_string_lossy())
}

/// Resolves a stored `ModelEntry::path` relative to `project_dir` (if relative).
pub fn resolve_model_path(project_dir: Option<&Path>, stored_path: &str) -> PathBuf {
    let p = Path::new(stored_path);
    if p.is_absolute() {
        p.to_path_buf()
    } else if let Some(dir) = project_dir {
        dir.join(p)
    } else {
        p.to_path_buf()
    }
}

fn normalize_slashes(raw: &str) -> String {
    raw.replace('\\', "/")
}
