use std::{
    collections::BTreeMap,
    path::{Component, Path, PathBuf},
    sync::Arc,
};

use chip_cad::{
    CadDocument, CadError, CadKernel, MeshImportOptions, MeshUnitKind, ModelTransform,
    MonstertruckKernel, is_mesh_extension, load_mesh_cad_document,
};
use chip_cam::{Setup, WorkOffsetSlot};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit, ParamLength};
use chip_sketch::{CustomFontMap, SketchEntity, SketchFontSource, SketchLayer};
use chip_tools::{DEFAULT_MACHINE_ID, MachineDefinition, Tool, ToolId};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Semantic role of a 3D model file referenced in `ProjectDocument.models`.
///
/// [`Self::Part`] (default, omitted from TOML) represents a machined target solid that drives
/// `StockConfig::RelativeBox`, WCS anchors, and target-part gouge/excess metrology.
/// [`Self::Fixture`] represents imported workholding hardware (vises, clamps, tombstones) used
/// exclusively by `FixtureKind::ImportedModel` so it never pollutes target part bounds or stock.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ModelRole {
    #[default]
    Part,
    Fixture,
}

impl ModelRole {
    #[inline]
    pub const fn is_part(&self) -> bool {
        matches!(self, Self::Part)
    }

    #[inline]
    pub const fn is_fixture(&self) -> bool {
        matches!(self, Self::Fixture)
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::Part => "Target Part",
            Self::Fixture => "Fixture / Workholding",
        }
    }
}

/// External 3D CAD (`.step`/`.stp`) or Triangle Mesh (`.stl`/`.obj`/`.3mf`) model reference
/// stored in `.chip.toml` (`09-persistence-cli.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ModelEntry {
    pub id: String,
    pub path: String,
    pub sha256: String,
    /// Semantic role (`part` default, omitted in TOML | `fixture` for imported workholding solids).
    #[serde(default, skip_serializing_if = "ModelRole::is_part")]
    pub role: ModelRole,
    /// Optional explicit unit override for imported triangle meshes (omitted for STEP and
    /// unit-aware 3MF files so existing `.chip.toml` diffs remain clean).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub unit_override: Option<MeshUnitKind>,
    /// Optional non-destructive model scale/rotation/translation applied in exact `f64` mm
    /// after loading (omitted when identity for full backward compatibility).
    #[serde(default, skip_serializing_if = "ModelTransform::is_identity")]
    pub transform: ModelTransform,
}

impl ModelEntry {
    pub fn new(id: impl Into<String>, path: impl Into<String>, sha256: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            path: path.into(),
            sha256: sha256.into(),
            role: ModelRole::Part,
            unit_override: None,
            transform: ModelTransform::IDENTITY,
        }
    }

    pub fn new_fixture(
        id: impl Into<String>,
        path: impl Into<String>,
        sha256: impl Into<String>,
    ) -> Self {
        Self {
            id: id.into(),
            path: path.into(),
            sha256: sha256.into(),
            role: ModelRole::Fixture,
            unit_override: None,
            transform: ModelTransform::IDENTITY,
        }
    }
}

/// Top-level ChipCutter `.chip.toml` project document (`schema_version = 3`, backward-compatible with `1` and `2`).
///
/// Kept lightweight and free of binary/tessellation blobs so `git diff` stays clean and
/// in-memory Undo/Redo snapshots take `< 1 ms`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProjectDocument {
    pub schema_version: u32,
    pub project_name: String,
    #[serde(default)]
    pub display_units: DisplayUnit,
    /// Optional target CNC machine definition ID (`machines/<id>.machine.toml`);
    /// omitted when `None` (defaults cleanly to `"generic"` for full backward compatibility).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub machine_id: Option<String>,
    #[serde(default)]
    pub models: Vec<ModelEntry>,
    /// Planar 2D Sketch Layers (DXF/SVG imports, 2D CAM shapes, & Pure-2D job geometry; `11-sketches-vectors.md`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub sketches: Vec<SketchLayer>,
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
    pub const CURRENT_SCHEMA_VERSION: u32 = 3;

    /// Creates a fresh project initialized with `Setup 1 — Op 1 (G54)`.
    pub fn new(project_name: impl Into<String>) -> Self {
        Self {
            schema_version: Self::CURRENT_SCHEMA_VERSION,
            project_name: project_name.into(),
            display_units: DisplayUnit::Millimeter,
            machine_id: None,
            models: Vec::new(),
            sketches: Vec::new(),
            tools: Vec::new(),
            setups: vec![Setup::new(
                "setup_1",
                "Setup 1 — Top Op",
                WorkOffsetSlot::G54,
            )],
        }
    }

    /// Returns the effective target machine ID (`"generic"` when omitted).
    pub fn effective_machine_id(&self) -> &str {
        self.machine_id
            .as_deref()
            .filter(|s| !s.trim().is_empty())
            .unwrap_or(DEFAULT_MACHINE_ID)
    }

    /// Associates this project with `machine` and optionally syncs each setup's
    /// default post-processor (and 4th-axis chuck defaults when rotary is enabled)
    /// without removing already-embedded project tools.
    pub fn apply_machine_defaults(&mut self, machine: &MachineDefinition, sync_setups: bool) {
        self.machine_id = Some(machine.id.clone());
        if sync_setups {
            for setup in &mut self.setups {
                if !setup.rotary.is_disabled() {
                    if let Some(rot) = &machine.rotary {
                        if let Some(post4) = &rot.default_post_profile_4axis {
                            setup.post_profile = post4.clone();
                        }
                        setup.rotary.chuck_diameter = ParamLength::from_mm(rot.chuck_diameter_mm);
                        setup.rotary.chuck_length = ParamLength::from_mm(rot.chuck_length_mm);
                    }
                } else {
                    setup.post_profile = machine.default_post_profile.clone();
                }
            }
        }
    }

    pub fn sketch(&self, id: &str) -> Option<&SketchLayer> {
        self.sketches.iter().find(|s| s.id == id)
    }

    pub fn sketch_mut(&mut self, id: &str) -> Option<&mut SketchLayer> {
        self.sketches.iter_mut().find(|s| s.id == id)
    }

    /// Returns the primary machined target part model (`role == ModelRole::Part`), ignoring
    /// any imported workholding fixture models (`ModelRole::Fixture`).
    pub fn primary_part_model(&self) -> Option<&ModelEntry> {
        self.models.iter().find(|m| m.role.is_part())
    }

    /// Returns a mutable reference to the primary machined target part model.
    pub fn primary_part_model_mut(&mut self) -> Option<&mut ModelEntry> {
        self.models.iter_mut().find(|m| m.role.is_part())
    }

    /// Returns the index in `self.models` of the primary machined target part model.
    pub fn primary_part_model_index(&self) -> Option<usize> {
        self.models.iter().position(|m| m.role.is_part())
    }

    /// Iterates over all machined target part models (`role == ModelRole::Part`).
    pub fn part_models(&self) -> impl Iterator<Item = &ModelEntry> {
        self.models.iter().filter(|m| m.role.is_part())
    }

    /// Iterates over all imported workholding fixture models (`role == ModelRole::Fixture`).
    pub fn fixture_models(&self) -> impl Iterator<Item = &ModelEntry> {
        self.models.iter().filter(|m| m.role.is_fixture())
    }

    /// Looks up any model entry (`Part` or `Fixture`) by `id`.
    pub fn model_by_id(&self, id: &str) -> Option<&ModelEntry> {
        self.models.iter().find(|m| m.id == id)
    }

    /// Loads all `ModelRole::Fixture` entries reachable from `project_dir` into a deterministic
    /// `model_id -> CadDocument` map ready for [`chip_cam::resolve_setup_fixtures`].
    pub fn load_fixture_cad_documents(
        &self,
        project_dir: Option<&Path>,
    ) -> BTreeMap<String, CadDocument> {
        let mut docs = BTreeMap::new();
        for entry in self.fixture_models() {
            let disk_path = resolve_model_path(project_dir, &entry.path);
            if let Ok(doc) = load_model_entry_cad_doc_transformed(
                &disk_path,
                entry.unit_override,
                &entry.transform,
            ) {
                docs.insert(entry.id.clone(), doc);
            }
        }
        docs
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

    /// Loads any custom `.ttf`/`.otf` font byte buffers referenced by [`SketchFontSource::CustomFile`]
    /// relative to `project_dir`.
    pub fn load_custom_fonts(&self, project_dir: Option<&Path>) -> CustomFontMap {
        let mut map = CustomFontMap::new();
        for sk in &self.sketches {
            for ent in &sk.entities {
                if let SketchEntity::Text(txt) = &ent.entity
                    && let SketchFontSource::CustomFile { path, sha256, .. } = &txt.font
                    && !map.contains_key(path)
                {
                    let disk_path = resolve_model_path(project_dir, path);
                    if let Ok(bytes) = std::fs::read(&disk_path) {
                        let arc_buf = Arc::new(bytes);
                        map.insert(path.clone(), Arc::clone(&arc_buf));
                        if !sha256.is_empty() {
                            map.insert(sha256.clone(), arc_buf);
                        }
                    }
                }
            }
        }
        map
    }

    /// Re-evaluates all parametric [`SketchLayer`] expressions (`stock_x`, `stock_y`, `stock_z`, etc.)
    /// against their associated [`Setup`]s and rebuilds closed [`chip_sketch::PlanarVectorRegion`]s.
    pub fn reevaluate_sketches(
        &mut self,
        cad_doc: &CadDocument,
        custom_fonts: Option<&CustomFontMap>,
    ) {
        if self.sketches.is_empty() {
            return;
        }
        let snapshot_sketches = self.sketches.clone();
        let resolved_setups: Vec<(String, chip_math::EvalContext)> = self
            .setups
            .iter()
            .filter_map(|s| {
                s.resolve_with_sketches(cad_doc, &snapshot_sketches, self.display_units)
                    .ok()
                    .map(|rs| (s.id.clone(), rs.eval_context(self.display_units)))
            })
            .collect();

        for sk in &mut self.sketches {
            let target_id = sk.anchor.setup_id();
            let ctx = resolved_setups
                .iter()
                .find(|(id, _)| id == target_id)
                .or_else(|| resolved_setups.first())
                .map(|(_, c)| c.clone())
                .unwrap_or_else(|| chip_math::EvalContext::new(self.display_units));
            sk.reevaluate(&ctx);
            sk.rebuild_regions(custom_fonts);
        }
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

/// Returns `true` if `path` has a supported 3D triangle mesh extension (`.stl`, `.obj`, `.3mf`).
pub fn is_mesh_path(path: &Path) -> bool {
    path.extension()
        .and_then(|s| s.to_str())
        .is_some_and(is_mesh_extension)
}

/// Unified CAD/Mesh loader from in-memory bytes, dispatching `.stl`/`.obj`/`.3mf` to
/// [`load_mesh_cad_document`] and STEP files to [`MonstertruckKernel::load_step_bytes`].
pub fn load_model_bytes_cad_doc(
    source_name: &str,
    bytes: &[u8],
    unit_override: Option<MeshUnitKind>,
) -> Result<CadDocument, CadError> {
    if is_mesh_path(Path::new(source_name)) {
        load_mesh_cad_document(
            source_name,
            bytes,
            &MeshImportOptions {
                unit_override,
                ..MeshImportOptions::default()
            },
        )
    } else {
        MonstertruckKernel.load_step_bytes(source_name, bytes, DEFAULT_TESS_CHORD_TOL_MM)
    }
}

/// Unified CAD/Mesh loader from in-memory bytes that validates and applies `transform`.
pub fn load_model_bytes_cad_doc_transformed(
    source_name: &str,
    bytes: &[u8],
    unit_override: Option<MeshUnitKind>,
    transform: &ModelTransform,
) -> Result<CadDocument, CadError> {
    transform.validate()?;
    let raw = load_model_bytes_cad_doc(source_name, bytes, unit_override)?;
    if transform.is_identity() {
        Ok(raw)
    } else {
        Ok(raw.transformed(transform))
    }
}

/// Unified CAD/Mesh loader reading `path` from disk and dispatching STEP vs. STL/OBJ/3MF cleanly.
pub fn load_model_entry_cad_doc(
    path: &Path,
    unit_override: Option<MeshUnitKind>,
) -> Result<CadDocument, CadError> {
    let bytes = std::fs::read(path)?;
    let source_name = path
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("model.step");
    load_model_bytes_cad_doc(source_name, &bytes, unit_override)
}

/// Unified CAD/Mesh loader reading `path` from disk and applying `transform` in exact `f64` mm.
pub fn load_model_entry_cad_doc_transformed(
    path: &Path,
    unit_override: Option<MeshUnitKind>,
    transform: &ModelTransform,
) -> Result<CadDocument, CadError> {
    transform.validate()?;
    let raw = load_model_entry_cad_doc(path, unit_override)?;
    if transform.is_identity() {
        Ok(raw)
    } else {
        Ok(raw.transformed(transform))
    }
}
