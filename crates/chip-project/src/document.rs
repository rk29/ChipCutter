use std::{
    collections::BTreeMap,
    path::{Component, Path, PathBuf},
    sync::Arc,
};

use chip_cad::{
    CadDocument, CadError, CadKernel, MeshImportOptions, MeshUnitKind, ModelTransform,
    MonstertruckKernel, is_mesh_extension, load_mesh_cad_document,
};
use chip_cam::{Setup, SharedAvoidRegion, StrategyParams, WorkOffsetSlot};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit, ParamLength};
use chip_sketch::{CustomFontMap, SketchEntity, SketchFontSource, SketchLayer};
use chip_tools::{
    DEFAULT_MACHINE_ID, MachineDefinition, ProjectMaterialSelection, Tool, ToolId,
    WorkpieceMaterialProfile, match_best_preset_for_material, resolve_workpiece_material,
};
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
    /// Optional project-level workpiece stock material selection (`chip_tools::materials`);
    /// omitted when `None` (defaults cleanly to Aluminum 6061-T6 via
    /// [`Self::effective_workpiece_material`] for 100% backward compatibility).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub workpiece_material: Option<ProjectMaterialSelection>,
    #[serde(default)]
    pub models: Vec<ModelEntry>,
    /// Planar 2D Sketch Layers (DXF/SVG imports, 2D CAM shapes, & Pure-2D job geometry; `11-sketches-vectors.md`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub sketches: Vec<SketchLayer>,
    /// Reusable named Avoid / Keep-Out regions linkable by ID across any number of operations & setups.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub avoid_regions: Vec<SharedAvoidRegion>,
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
            workpiece_material: None,
            models: Vec::new(),
            sketches: Vec::new(),
            avoid_regions: Vec::new(),
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

    /// Resolves the effective project-level [`WorkpieceMaterialProfile`] (defaults to
    /// Aluminum 6061-T6 when [`Self::workpiece_material`] is `None`).
    pub fn effective_workpiece_material(&self) -> WorkpieceMaterialProfile {
        resolve_workpiece_material(self.workpiece_material.as_ref())
    }

    /// Counts `(ops_needing_preset_update, ops_with_assigned_tool)` across all setups for the
    /// current [`Self::effective_workpiece_material`].
    pub fn count_operations_needing_preset_match(&self) -> (usize, usize) {
        let material = self.effective_workpiece_material();
        let mut needing_update = 0usize;
        let mut with_tool = 0usize;
        for setup in &self.setups {
            for op in &setup.operations {
                let mut op_needs = false;
                if let Some(tid) = op.tool_id.as_ref()
                    && let Some(tool) = self.tool(tid)
                {
                    with_tool += 1;
                    if let Some(best) = match_best_preset_for_material(tool, &material)
                        && op.preset_id.as_deref() != Some(best.id.as_str())
                    {
                        op_needs = true;
                    }
                }
                if let Some(StrategyParams::VCarve(vp)) = &op.strategy
                    && vp.clearing_preset_id.is_some()
                    && let Some(ctid) = vp.clearing_tool_id.as_ref()
                    && let Some(ctool) = self.tool(ctid)
                    && let Some(best) = match_best_preset_for_material(ctool, &material)
                    && vp.clearing_preset_id.as_deref() != Some(best.id.as_str())
                {
                    op_needs = true;
                }
                if op_needs {
                    needing_update += 1;
                }
            }
        }
        (needing_update, with_tool)
    }

    /// Updates each operation's `preset_id` (and explicit V-Carve `clearing_preset_id`, if set)
    /// to the best matching cutter preset for [`Self::effective_workpiece_material`], while
    /// preserving any explicit user `feed_overrides` untouched and marking modified ops `Dirty`.
    ///
    /// Returns the number of operations whose preset changed.
    pub fn apply_matching_presets_for_material(&mut self) -> usize {
        let material = self.effective_workpiece_material();
        let best_by_tool: BTreeMap<ToolId, String> = self
            .tools
            .iter()
            .filter_map(|t| {
                match_best_preset_for_material(t, &material).map(|p| (t.id.clone(), p.id.clone()))
            })
            .collect();

        let mut updated = 0usize;
        for setup in &mut self.setups {
            for op in &mut setup.operations {
                let mut changed = false;
                if let Some(tid) = op.tool_id.as_ref()
                    && let Some(best_pid) = best_by_tool.get(tid)
                    && op.preset_id.as_deref() != Some(best_pid.as_str())
                {
                    op.preset_id = Some(best_pid.clone());
                    changed = true;
                }
                if let Some(StrategyParams::VCarve(vp)) = &mut op.strategy
                    && vp.clearing_preset_id.is_some()
                    && let Some(ctid) = vp.clearing_tool_id.as_ref()
                    && let Some(best_pid) = best_by_tool.get(ctid)
                    && vp.clearing_preset_id.as_deref() != Some(best_pid.as_str())
                {
                    vp.clearing_preset_id = Some(best_pid.clone());
                    changed = true;
                }
                if changed {
                    op.mark_dirty();
                    updated += 1;
                }
            }
        }
        updated
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

    /// Looks up a project-level reusable [`SharedAvoidRegion`] by `id`.
    pub fn avoid_region(&self, id: &str) -> Option<&SharedAvoidRegion> {
        self.avoid_regions.iter().find(|r| r.id == id)
    }

    /// Looks up a mutable project-level reusable [`SharedAvoidRegion`] by `id`.
    pub fn avoid_region_mut(&mut self, id: &str) -> Option<&mut SharedAvoidRegion> {
        self.avoid_regions.iter_mut().find(|r| r.id == id)
    }

    /// Removes the [`SharedAvoidRegion`] with `id`, prunes all stale `shared_region_ids`
    /// references across every Setup/Operation, and marks affected operations `Dirty`.
    pub fn remove_avoid_region(&mut self, id: &str) -> bool {
        let before = self.avoid_regions.len();
        self.avoid_regions.retain(|r| r.id != id);
        let removed = self.avoid_regions.len() < before;
        for setup in &mut self.setups {
            for op in &mut setup.operations {
                let before_refs = op.avoid.shared_region_ids.len();
                op.avoid.shared_region_ids.retain(|rid| rid != id);
                if op.avoid.shared_region_ids.len() < before_refs {
                    op.mark_dirty();
                }
            }
        }
        removed
    }

    /// Counts how many operations across all Setups reference the [`SharedAvoidRegion`] `id`.
    pub fn avoid_region_usage_count(&self, id: &str) -> usize {
        self.setups
            .iter()
            .flat_map(|s| s.operations.iter())
            .filter(|op| op.avoid.shared_region_ids.iter().any(|rid| rid == id))
            .count()
    }

    /// Returns the primary machined target part model (`role == ModelRole::Part`), ignoring
    /// any imported workholding fixture models (`ModelRole::Fixture`).
    pub fn primary_part_model(&self) -> Option<&ModelEntry> {
        self.models.iter().find(|m| m.role.is_part())
    }

    /// Returns the index in `self.models` of the primary machined target part model.
    pub fn primary_part_model_index(&self) -> Option<usize> {
        self.models.iter().position(|m| m.role.is_part())
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

    /// Maps every referenced [`ToolId`] (both primary `op.tool_id` and V-Carve
    /// `clearing_tool_id`) to the operations that use it.
    pub fn tool_usage_map(&self) -> BTreeMap<ToolId, Vec<ToolOperationRef>> {
        let mut map: BTreeMap<ToolId, Vec<ToolOperationRef>> = BTreeMap::new();
        for (setup_idx, setup) in self.setups.iter().enumerate() {
            for (op_idx, op) in setup.operations.iter().enumerate() {
                if let Some(tid) = &op.tool_id {
                    map.entry(tid.clone()).or_default().push(ToolOperationRef {
                        setup_idx,
                        op_idx,
                        setup_name: setup.name.clone(),
                        op_name: op.name.clone(),
                        is_clearing_tool: false,
                    });
                }
                if let Some(ctid) = op.clearing_tool_id() {
                    map.entry(ctid.clone()).or_default().push(ToolOperationRef {
                        setup_idx,
                        op_idx,
                        setup_name: setup.name.clone(),
                        op_name: op.name.clone(),
                        is_clearing_tool: true,
                    });
                }
            }
        }
        map
    }

    /// Returns embedded project tools that are not referenced by any operation.
    pub fn unused_tool_ids(&self) -> Vec<ToolId> {
        let used = self.tool_usage_map();
        self.tools
            .iter()
            .filter(|t| !used.contains_key(&t.id))
            .map(|t| t.id.clone())
            .collect()
    }

    /// Removes an embedded project tool if and only if no operation references it.
    pub fn remove_tool_if_unused(&mut self, id: &ToolId) -> bool {
        let used = self.tool_usage_map();
        if used.contains_key(id) {
            return false;
        }
        let before = self.tools.len();
        self.tools.retain(|t| &t.id != id);
        self.tools.len() < before
    }

    /// Removes all unreferenced embedded project tools and returns how many were removed.
    pub fn prune_unused_tools(&mut self) -> usize {
        let used = self.tool_usage_map();
        let before = self.tools.len();
        self.tools.retain(|t| used.contains_key(&t.id));
        before - self.tools.len()
    }

    /// Inspects all project tools referenced by operations against `target_machine.tools` and
    /// produces a deterministic replacement plan for interactive machine switching.
    pub fn plan_machine_tool_swap(
        &self,
        target_machine: &MachineDefinition,
    ) -> MachineToolSwapPlan {
        let usage_map = self.tool_usage_map();
        let mut exact_matches = Vec::new();
        let mut entries = Vec::new();

        for (tid, usages) in &usage_map {
            let Some(cur_tool) = self.tool(tid) else {
                continue;
            };
            let is_exact = target_machine
                .tools
                .iter()
                .any(|cand| chip_tools::is_exact_tool_match(cand, cur_tool));
            if is_exact {
                exact_matches.push(tid.clone());
                continue;
            }

            let cur_disc = std::mem::discriminant(&cur_tool.geometry);
            let ranked = chip_tools::rank_replacement_tools(cur_tool, &target_machine.tools);

            let suggested = ranked
                .first()
                .filter(|cand| std::mem::discriminant(&cand.geometry) == cur_disc)
                .cloned()
                .map_or(ToolSwapChoice::KeepExisting, |t| {
                    ToolSwapChoice::ReplaceWith(Box::new(t))
                });

            entries.push(ToolSwapEntry {
                current_tool: cur_tool.clone(),
                usages: usages.clone(),
                candidates: ranked,
                suggested,
            });
        }

        let unused_project_tools = self.unused_tool_ids();
        MachineToolSwapPlan {
            target_machine_id: target_machine.id.clone(),
            target_machine_name: target_machine.name.clone(),
            exact_matches,
            entries,
            unused_project_tools,
        }
    }

    /// Switches this project to `target_machine`, applies user-chosen tool replacements across all
    /// setups/operations (including V-Carve clearing tools), marks modified operations `Dirty`,
    /// and optionally prunes unreferenced tools.
    pub fn apply_machine_tool_swap(
        &mut self,
        target_machine: &MachineDefinition,
        sync_setups: bool,
        choices: &BTreeMap<ToolId, ToolSwapChoice>,
        prune_unused: bool,
    ) -> MachineToolSwapSummary {
        self.apply_machine_defaults(target_machine, sync_setups);

        let mut replacement_map: BTreeMap<ToolId, Tool> = BTreeMap::new();
        for (old_id, choice) in choices {
            if let ToolSwapChoice::ReplaceWith(new_tool) = choice {
                self.ensure_tool_embedded(new_tool);
                replacement_map.insert(old_id.clone(), (**new_tool).clone());
            }
        }

        let material = self.effective_workpiece_material();
        let mut updated_operation_count = 0usize;
        if !replacement_map.is_empty() {
            for setup in &mut self.setups {
                for op in &mut setup.operations {
                    let mut op_changed = false;
                    if let Some(old_tid) = op.tool_id.clone()
                        && let Some(new_tool) = replacement_map.get(&old_tid)
                        && new_tool.id != old_tid
                    {
                        op.tool_id = Some(new_tool.id.clone());
                        op.preset_id = match_best_preset_for_material(new_tool, &material)
                            .map(|p| p.id.clone());
                        op.feed_overrides.clear_all();
                        op_changed = true;
                    }
                    if let Some(StrategyParams::VCarve(vp)) = &mut op.strategy
                        && let Some(old_ctid) = vp.clearing_tool_id.clone()
                        && let Some(new_tool) = replacement_map.get(&old_ctid)
                        && new_tool.id != old_ctid
                    {
                        vp.clearing_tool_id = Some(new_tool.id.clone());
                        op_changed = true;
                    }
                    if op_changed {
                        op.mark_dirty();
                        updated_operation_count += 1;
                    }
                }
            }
        }

        let pruned_unused_count = if prune_unused {
            self.prune_unused_tools()
        } else {
            0
        };
        self.sort_tools_deterministically();

        MachineToolSwapSummary {
            replaced_tool_count: replacement_map.len(),
            updated_operation_count,
            pruned_unused_count,
        }
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

/// Reference from a CAM operation to an embedded project tool (`ProjectDocument::tool_usage_map`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ToolOperationRef {
    pub setup_idx: usize,
    pub op_idx: usize,
    pub setup_name: String,
    pub op_name: String,
    pub is_clearing_tool: bool,
}

/// User selection for a project tool when switching CNC machines.
#[derive(Debug, Clone, PartialEq)]
pub enum ToolSwapChoice {
    ReplaceWith(Box<Tool>),
    KeepExisting,
}

/// Single project tool that requires user confirmation/replacement when switching machines.
#[derive(Debug, Clone, PartialEq)]
pub struct ToolSwapEntry {
    pub current_tool: Tool,
    pub usages: Vec<ToolOperationRef>,
    /// Target machine tools sorted by geometry compatibility and closest cutting diameter.
    pub candidates: Vec<Tool>,
    pub suggested: ToolSwapChoice,
}

/// Planned tool migration summary when switching a project to a different [`MachineDefinition`].
#[derive(Debug, Clone, PartialEq)]
pub struct MachineToolSwapPlan {
    pub target_machine_id: String,
    pub target_machine_name: String,
    pub exact_matches: Vec<ToolId>,
    pub entries: Vec<ToolSwapEntry>,
    pub unused_project_tools: Vec<ToolId>,
}

impl MachineToolSwapPlan {
    /// Returns `true` when at least one operation uses a tool missing from the target machine.
    pub fn needs_user_prompt(&self) -> bool {
        !self.entries.is_empty()
    }
}

/// Summary returned after applying a [`MachineToolSwapPlan`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MachineToolSwapSummary {
    pub replaced_tool_count: usize,
    pub updated_operation_count: usize,
    pub pruned_unused_count: usize,
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

/// Returns `true` when `path` points inside a Linux Flatpak `xdg-document-portal` FUSE mount
/// (`/run/user/<uid>/doc/<doc-id>/...` or `/var/run/user/<uid>/doc/<doc-id>/...`).
pub fn is_flatpak_document_portal_path(path: &Path) -> bool {
    let mut comps = path.components();
    if !matches!(comps.next(), Some(Component::RootDir)) {
        return false;
    }
    let Some(Component::Normal(first)) = comps.next() else {
        return false;
    };
    let run_matched = if first == "run" {
        true
    } else if first == "var" {
        matches!(comps.next(), Some(Component::Normal(s)) if s == "run")
    } else {
        false
    };
    if !run_matched {
        return false;
    }
    if !matches!(comps.next(), Some(Component::Normal(s)) if s == "user") {
        return false;
    }
    let Some(Component::Normal(uid)) = comps.next() else {
        return false;
    };
    if uid.is_empty()
        || !uid
            .to_str()
            .is_some_and(|s| s.bytes().all(|b| b.is_ascii_digit()))
    {
        return false;
    }
    if !matches!(comps.next(), Some(Component::Normal(s)) if s == "doc") {
        return false;
    }
    matches!(comps.next(), Some(Component::Normal(doc_id)) if !doc_id.is_empty())
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

    // Never emit `../<other-doc-id>/...` across distinct Flatpak `xdg-document-portal` mounts.
    if is_flatpak_document_portal_path(&proj_abs) || is_flatpak_document_portal_path(&step_abs) {
        return normalize_slashes(&step_path.to_string_lossy());
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

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::{is_flatpak_document_portal_path, portable_model_path};

    #[test]
    fn flatpak_document_portal_path_detection() {
        assert!(is_flatpak_document_portal_path(Path::new(
            "/run/user/1000/doc/a1b2c3d4/part.step"
        )));
        assert!(is_flatpak_document_portal_path(Path::new(
            "/var/run/user/1000/doc/a1b2c3d4/part.step"
        )));
        assert!(is_flatpak_document_portal_path(Path::new(
            "/run/user/0/doc/deadbeef"
        )));

        assert!(!is_flatpak_document_portal_path(Path::new(
            "/run/user/1000/doc"
        )));
        assert!(!is_flatpak_document_portal_path(Path::new(
            "/run/user/alice/doc/a1b2c3d4/part.step"
        )));
        assert!(!is_flatpak_document_portal_path(Path::new(
            "/home/user/projects/part.step"
        )));
        assert!(!is_flatpak_document_portal_path(Path::new(
            "relative/part.step"
        )));
    }

    #[test]
    fn portable_model_path_does_not_walk_across_distinct_flatpak_portal_mounts() {
        let proj = Path::new("/run/user/1000/doc/bbbb2222/job.chip.toml");
        let other_portal_step = Path::new("/run/user/1000/doc/aaaa1111/part.step");
        assert_eq!(
            portable_model_path(Some(proj), other_portal_step),
            "/run/user/1000/doc/aaaa1111/part.step"
        );

        let same_portal_step = Path::new("/run/user/1000/doc/bbbb2222/models/part.step");
        assert_eq!(
            portable_model_path(Some(proj), same_portal_step),
            "models/part.step"
        );

        let normal_proj = Path::new("/home/user/cnc/cam/job.chip.toml");
        let normal_sibling = Path::new("/home/user/cnc/cad/part.step");
        assert_eq!(
            portable_model_path(Some(normal_proj), normal_sibling),
            "../cad/part.step"
        );
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn tool_usage_pruning_and_machine_swap_plan_and_apply() {
        use std::collections::BTreeMap;

        use chip_cam::{Operation, OperationKind, OperationStatus, StrategyParams};
        use chip_tools::{CutterGeometry, Tool, ToolId, builtin_generic_machine};

        use super::{ProjectDocument, ToolSwapChoice};

        let generic = builtin_generic_machine();
        let flat6 = generic
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { .. }))
            .unwrap()
            .clone();
        let vbit60 = generic
            .tools
            .iter()
            .find(|t| OperationKind::VCarve.is_compatible_with_tool(t))
            .unwrap()
            .clone();
        let unused_ball = generic
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::BallEndmill { .. }))
            .unwrap()
            .clone();

        let mut doc = ProjectDocument::new("Swap Test");
        doc.ensure_tool_embedded(&flat6);
        doc.ensure_tool_embedded(&vbit60);
        doc.ensure_tool_embedded(&unused_ball);

        let mut op1 = Operation::new("op1", "Pocket 1", OperationKind::Pocket2D);
        op1.tool_id = Some(flat6.id.clone());
        op1.runtime_status = OperationStatus::Clean;

        let mut op2 = Operation::new("op2", "VCarve 1", OperationKind::VCarve);
        op2.tool_id = Some(vbit60.id.clone());
        op2.runtime_status = OperationStatus::Clean;
        if let Some(StrategyParams::VCarve(vp)) = &mut op2.strategy {
            vp.clearing_tool_id = Some(flat6.id.clone());
        }
        doc.setups[0].operations.push(op1);
        doc.setups[0].operations.push(op2);

        // Usage map captures both primary tool_id and V-Carve clearing_tool_id
        let usage = doc.tool_usage_map();
        assert_eq!(usage.get(&flat6.id).unwrap().len(), 2);
        assert!(
            usage
                .get(&flat6.id)
                .unwrap()
                .iter()
                .any(|u| u.is_clearing_tool)
        );
        assert_eq!(usage.get(&vbit60.id).unwrap().len(), 1);
        assert_eq!(doc.unused_tool_ids(), vec![unused_ball.id.clone()]);

        // Cannot remove in-use tool, can remove unused tool
        assert!(!doc.remove_tool_if_unused(&flat6.id));
        assert!(doc.remove_tool_if_unused(&unused_ball.id));
        assert!(doc.unused_tool_ids().is_empty());

        // Target machine has a closer-diameter flat (different ID), a farther flat, and exact vbit60
        let base_dia = flat6.geometry.cutting_diameter_mm();
        let mut flat_close = flat6.clone();
        flat_close.id = ToolId("target_flat_close".to_string());
        flat_close.name = "Close Flat Endmill".to_string();
        if let CutterGeometry::FlatEndmill { diameter_mm, .. } = &mut flat_close.geometry {
            *diameter_mm = base_dia + 0.35;
        }
        let mut flat_far = flat6.clone();
        flat_far.id = ToolId("target_flat_far".to_string());
        flat_far.name = "Far Flat Endmill".to_string();
        if let CutterGeometry::FlatEndmill { diameter_mm, .. } = &mut flat_far.geometry {
            *diameter_mm = (base_dia - 3.0).max(1.0);
        }

        let mut target_machine = generic.clone();
        target_machine.id = "custom_router".to_string();
        target_machine.name = "Custom Router".to_string();
        target_machine.tools = vec![flat_far.clone(), flat_close.clone(), vbit60.clone()];

        let plan = doc.plan_machine_tool_swap(&target_machine);
        assert!(plan.needs_user_prompt());
        assert_eq!(plan.exact_matches, vec![vbit60.id.clone()]);
        assert_eq!(plan.entries.len(), 1);
        assert_eq!(plan.entries[0].current_tool.id, flat6.id);
        // Closest diameter flat endmill is suggested over the farther diameter
        assert_eq!(
            plan.entries[0].suggested,
            ToolSwapChoice::ReplaceWith(Box::new(flat_close.clone()))
        );

        let mut choices = BTreeMap::new();
        choices.insert(
            flat6.id.clone(),
            ToolSwapChoice::ReplaceWith(Box::new(flat_close.clone())),
        );
        let summary = doc.apply_machine_tool_swap(&target_machine, true, &choices, true);
        assert_eq!(summary.replaced_tool_count, 1);
        assert_eq!(summary.updated_operation_count, 2);
        assert_eq!(summary.pruned_unused_count, 1);

        assert_eq!(doc.machine_id.as_deref(), Some("custom_router"));
        assert_eq!(
            doc.setups[0].operations[0].tool_id,
            Some(flat_close.id.clone())
        );
        assert_eq!(
            doc.setups[0].operations[0].runtime_status,
            OperationStatus::Dirty
        );
        assert_eq!(
            doc.setups[0].operations[1].clearing_tool_id(),
            Some(&flat_close.id)
        );
        assert_eq!(
            doc.setups[0].operations[1].runtime_status,
            OperationStatus::Dirty
        );
        assert!(
            doc.tool(&flat6.id).is_none(),
            "old replaced tool should be pruned"
        );
        assert!(doc.tool(&flat_close.id).is_some());

        // Switching to a machine with zero compatible flat endmills suggests KeepExisting
        let mut vbit_only_machine = target_machine.clone();
        vbit_only_machine.id = "engraver".to_string();
        vbit_only_machine.tools = vec![Tool { ..vbit60 }];
        let plan2 = doc.plan_machine_tool_swap(&vbit_only_machine);
        assert_eq!(plan2.entries.len(), 1);
        assert_eq!(plan2.entries[0].suggested, ToolSwapChoice::KeepExisting);
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn workpiece_material_toml_roundtrip_and_apply_matching_presets_preserves_overrides() {
        use chip_cam::{Operation, OperationKind, OperationStatus};
        use chip_tools::{
            ALUMINUM_6061_T6_ID, ALUMINUM_6063_T5_ID, CUSTOM_WORKPIECE_MATERIAL_ID,
            ProjectMaterialSelection, builtin_carvera_machine,
        };

        use super::ProjectDocument;

        // 1. Default project omits `workpiece_material` in TOML and resolves to 6061-T6
        let doc_default = ProjectDocument::new("Legacy Clean Job");
        assert!(doc_default.workpiece_material.is_none());
        assert_eq!(
            doc_default.effective_workpiece_material().id,
            ALUMINUM_6061_T6_ID
        );
        let toml_default = doc_default.to_toml_string().unwrap();
        assert!(
            !toml_default.contains("workpiece_material"),
            "unset workpiece_material must be omitted from TOML for zero legacy diff"
        );
        let reloaded_default = ProjectDocument::from_toml_str(&toml_default).unwrap();
        assert_eq!(reloaded_default.workpiece_material, None);

        // 2. Built-in 6063-T5 selection roundtrips losslessly
        let mut doc_6063 = ProjectDocument::new("6063 Extrusion Bracket");
        doc_6063.workpiece_material = Some(ProjectMaterialSelection::builtin(ALUMINUM_6063_T5_ID));
        let toml_6063 = doc_6063.to_toml_string().unwrap();
        assert!(toml_6063.contains("material_id = \"aluminum-6063-t5\""));
        assert!(!toml_6063.contains("custom_name"));
        let reloaded_6063 = ProjectDocument::from_toml_str(&toml_6063).unwrap();
        assert_eq!(
            reloaded_6063.workpiece_material,
            doc_6063.workpiece_material
        );
        assert_eq!(
            reloaded_6063.effective_workpiece_material().id,
            ALUMINUM_6063_T5_ID
        );

        // 3. Custom material with physics overrides roundtrips losslessly
        let mut doc_custom = ProjectDocument::new("Custom Billet");
        doc_custom.workpiece_material = Some(ProjectMaterialSelection::custom(
            "Shop 6063-T52",
            0.88,
            880.0,
            0.011,
        ));
        let toml_custom = doc_custom.to_toml_string().unwrap();
        let reloaded_custom = ProjectDocument::from_toml_str(&toml_custom).unwrap();
        assert_eq!(
            reloaded_custom.workpiece_material,
            doc_custom.workpiece_material
        );
        let eff_custom = reloaded_custom.effective_workpiece_material();
        assert_eq!(eff_custom.id, CUSTOM_WORKPIECE_MATERIAL_ID);
        assert_eq!(eff_custom.name, "Shop 6063-T52");
        assert!((eff_custom.specific_cutting_energy_w_s_mm3 - 0.88).abs() < 1e-9);
        assert!((eff_custom.specific_cutting_force_n_mm2 - 880.0).abs() < 1e-9);
        assert!((eff_custom.min_chip_thickness_mm - 0.011).abs() < 1e-9);

        // 4. apply_matching_presets_for_material updates presets, preserves overrides, marks Dirty
        let carvera = builtin_carvera_machine();
        let t1 = carvera.tools.iter().find(|t| t.tool_number == 1).unwrap();
        let t2 = carvera.tools.iter().find(|t| t.tool_number == 2).unwrap();
        doc_6063.ensure_tool_embedded(t1);
        doc_6063.ensure_tool_embedded(t2);

        let mut op1 = Operation::new("op1", "Pocket 1", OperationKind::Pocket2D);
        op1.tool_id = Some(t1.id.clone());
        op1.preset_id = Some("carvera_atc_al6061".to_string());
        op1.feed_overrides.spindle_rpm = Some(11_500.0);
        op1.feed_overrides.cutting_feed_mm_min = Some(320.0);
        op1.runtime_status = OperationStatus::Clean;

        let mut op2 = Operation::new("op2", "Contour 1", OperationKind::Contour2D);
        op2.tool_id = Some(t2.id.clone());
        op2.preset_id = Some("carvera_atc_al6063_deep".to_string()); // already matched
        op2.runtime_status = OperationStatus::Clean;

        doc_6063.setups[0].operations = vec![op1, op2];
        assert_eq!(doc_6063.count_operations_needing_preset_match(), (1, 2));

        let updated = doc_6063.apply_matching_presets_for_material();
        assert_eq!(updated, 1);
        assert_eq!(
            doc_6063.setups[0].operations[0].preset_id.as_deref(),
            Some("carvera_atc_al6063")
        );
        // Explicit user overrides must remain untouched!
        assert_eq!(
            doc_6063.setups[0].operations[0].feed_overrides.spindle_rpm,
            Some(11_500.0)
        );
        assert_eq!(
            doc_6063.setups[0].operations[0]
                .feed_overrides
                .cutting_feed_mm_min,
            Some(320.0)
        );
        assert_eq!(
            doc_6063.setups[0].operations[0].runtime_status,
            OperationStatus::Dirty
        );
        // Already-matched op2 stays Clean
        assert_eq!(
            doc_6063.setups[0].operations[1].runtime_status,
            OperationStatus::Clean
        );
        assert_eq!(doc_6063.count_operations_needing_preset_match(), (0, 2));
    }
}
