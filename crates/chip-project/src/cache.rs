use std::{
    collections::{HashMap, HashSet, VecDeque},
    sync::Arc,
};

use chip_cam::{GeneratedToolpath, OperationStatus, RestMachiningMode, Setup};
use sha2::{Digest, Sha256};

use crate::document::ProjectDocument;

/// Deterministic 256-bit content digest of all inputs affecting a single operation's toolpath
/// (`CAD revision + Setup WCS/Stock/Heights + Embedded Tool + Operation params + Upstream Rest Hashes`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct OperationInputHash(pub [u8; 32]);

/// Resolves the ordered indices `u_idx in 0..op_idx` of non-suppressed preceding operations within
/// `setup` referenced by `setup.operations[op_idx]` when `RestMachiningMode::FromPrecedingOperations`
/// is active (empty slice when Rest Machining is `Disabled` or `FromPreviousTool`).
pub fn resolve_upstream_rest_op_indices(setup: &Setup, op_idx: usize) -> Vec<usize> {
    let Some(op) = setup.operations.get(op_idx) else {
        return Vec::new();
    };
    let RestMachiningMode::FromPrecedingOperations { source_op_ids } =
        &op.passes.rest_machining.mode
    else {
        return Vec::new();
    };

    let mut indices = Vec::new();
    for (u_idx, u_op) in setup.operations.iter().enumerate().take(op_idx) {
        if u_op.suppressed {
            continue;
        }
        if source_op_ids.is_empty() || source_op_ids.iter().any(|id| id == &u_op.id) {
            indices.push(u_idx);
        }
    }
    indices
}

/// Computes the [`OperationInputHash`] for `project.setups[setup_idx].operations[op_idx]`.
pub fn compute_operation_input_hash(
    project: &ProjectDocument,
    setup_idx: usize,
    op_idx: usize,
    cad_revision: u64,
) -> Option<OperationInputHash> {
    let setup = project.setups.get(setup_idx)?;
    let op = setup.operations.get(op_idx)?;

    let mut hasher = Sha256::new();
    hasher.update(b"chipcutter_op_v1:");
    hasher.update(cad_revision.to_le_bytes());
    hasher.update([match project.display_units {
        chip_math::DisplayUnit::Millimeter => 0_u8,
        chip_math::DisplayUnit::Inch => 1_u8,
    }]);

    for m in &project.models {
        hasher.update(m.id.as_bytes());
        hasher.update(b":");
        hasher.update(m.sha256.as_bytes());
        hasher.update(b";");
    }

    for b in &setup.target_bodies {
        hasher.update(b.0.to_le_bytes());
    }
    if let Ok(s) = toml::to_string(&setup.wcs) {
        hasher.update(s.as_bytes());
    }
    if let Ok(s) = toml::to_string(&setup.stock) {
        hasher.update(s.as_bytes());
    }
    if let Ok(s) = toml::to_string(&setup.heights) {
        hasher.update(s.as_bytes());
    }
    if let Some(tid) = &op.tool_id
        && let Some(tool) = project.tool(tid)
        && let Ok(s) = toml::to_string(tool)
    {
        hasher.update(s.as_bytes());
    }
    if let Some(ctid) = op.clearing_tool_id()
        && let Some(ctool) = project.tool(ctid)
        && let Ok(s) = toml::to_string(ctool)
    {
        hasher.update(b"|clearing_tool:");
        hasher.update(s.as_bytes());
    }
    if let Ok(s) = toml::to_string(op) {
        hasher.update(s.as_bytes());
    }

    // Fold 2D SketchLayers when present (skipped when `sketches.is_empty()` so v1 hashes stay bit-identical!)
    if !project.sketches.is_empty() {
        hasher.update(b"|sketches_v2:");
        for sk in &project.sketches {
            if let Ok(s) = toml::to_string(sk) {
                hasher.update(s.as_bytes());
                hasher.update(b";");
            }
        }
    }

    // Fold enabled 3D Workholding fixtures + rapid-avoidance toggle (skipped when no fixtures exist
    // and `auto_avoid_fixtures_on_rapids` is default `true` so v1/v2/v3 hashes stay bit-identical!)
    let any_enabled_fixture = setup.fixtures.iter().any(|f| f.enabled);
    if any_enabled_fixture || !setup.auto_avoid_fixtures_on_rapids {
        hasher.update(b"|workholding_v1:");
        hasher.update([u8::from(setup.auto_avoid_fixtures_on_rapids)]);
        for fix in &setup.fixtures {
            if fix.enabled
                && let Ok(s) = toml::to_string(fix)
            {
                hasher.update(s.as_bytes());
                hasher.update(b";");
            }
        }
    }

    // Fold upstream Rest Machining operation hashes recursively (`u_idx < op_idx` guarantees DAG acyclicity;
    // skipped completely when `rest_machining` is disabled or `FromPreviousTool` so v1 hashes stay bit-identical!)
    let upstream_indices = resolve_upstream_rest_op_indices(setup, op_idx);
    if !upstream_indices.is_empty() {
        hasher.update(b"|rest_upstream:");
        for u_idx in upstream_indices {
            hasher.update(setup.operations[u_idx].id.as_bytes());
            hasher.update(b"=");
            if let Some(u_hash) =
                compute_operation_input_hash(project, setup_idx, u_idx, cad_revision)
            {
                hasher.update(u_hash.0);
            }
            hasher.update(b";");
        }
    }

    let digest = hasher.finalize();
    let mut bytes = [0_u8; 32];
    bytes.copy_from_slice(digest.as_slice());
    Some(OperationInputHash(bytes))
}

/// Out-of-band hash-addressed cache of generated toolpaths (`08-ui-viewport.md` §Undo/Redo).
///
/// Keeps `ProjectDocument` and `.chip.toml` free of heavy toolpath buffers while allowing
/// `Ctrl+Z` / `Ctrl+Y` to restore previously `Clean` toolpaths in `< 1 ms`.
#[derive(Debug, Clone)]
pub struct ToolpathCache {
    max_entries: usize,
    entries: VecDeque<((String, OperationInputHash), Arc<GeneratedToolpath>)>,
    errors: HashMap<(String, OperationInputHash), String>,
}

impl Default for ToolpathCache {
    fn default() -> Self {
        Self::new(128)
    }
}

impl ToolpathCache {
    pub fn new(max_entries: usize) -> Self {
        Self {
            max_entries: max_entries.max(8),
            entries: VecDeque::new(),
            errors: HashMap::new(),
        }
    }

    pub fn clear(&mut self) {
        self.entries.clear();
        self.errors.clear();
    }

    pub fn get(&self, op_id: &str, hash: OperationInputHash) -> Option<Arc<GeneratedToolpath>> {
        self.entries
            .iter()
            .rev()
            .find(|((id, h), _)| id == op_id && *h == hash)
            .map(|(_, tp)| Arc::clone(tp))
    }

    /// Returns the resident [`GeneratedToolpath`] for a specific operation in `project` if its
    /// current [`OperationInputHash`] matches a cached `Clean` run.
    pub fn get_for_operation(
        &self,
        project: &ProjectDocument,
        setup_idx: usize,
        op_idx: usize,
        cad_revision: u64,
    ) -> Option<(OperationInputHash, Arc<GeneratedToolpath>)> {
        let setup = project.setups.get(setup_idx)?;
        let op = setup.operations.get(op_idx)?;
        let hash = compute_operation_input_hash(project, setup_idx, op_idx, cad_revision)?;
        let tp = self.get(&op.id, hash)?;
        Some((hash, tp))
    }

    pub fn insert(
        &mut self,
        op_id: impl Into<String>,
        hash: OperationInputHash,
        toolpath: Arc<GeneratedToolpath>,
    ) {
        let id = op_id.into();
        self.errors.remove(&(id.clone(), hash));
        self.entries
            .retain(|((eid, eh), _)| !(eid == &id && *eh == hash));
        if self.entries.len() >= self.max_entries {
            self.entries.pop_front();
        }
        self.entries.push_back(((id, hash), toolpath));
    }

    pub fn record_error(
        &mut self,
        op_id: impl Into<String>,
        hash: OperationInputHash,
        message: impl Into<String>,
    ) {
        self.errors.insert((op_id.into(), hash), message.into());
    }

    pub fn get_error(&self, op_id: &str, hash: OperationInputHash) -> Option<&str> {
        self.errors
            .get(&(op_id.to_string(), hash))
            .map(String::as_str)
    }

    /// Synchronizes every operation's runtime `Clean` / `Dirty` / `Generating` / `Error` status
    /// against `self` and `in_flight_ops`.
    pub fn reconcile_project_statuses(
        &self,
        project: &mut ProjectDocument,
        cad_revision: u64,
        in_flight_ops: &HashSet<String>,
    ) {
        for s_idx in 0..project.setups.len() {
            let op_len = project.setups[s_idx].operations.len();
            for o_idx in 0..op_len {
                let Some(hash) = compute_operation_input_hash(project, s_idx, o_idx, cad_revision)
                else {
                    continue;
                };
                let op = &mut project.setups[s_idx].operations[o_idx];
                if in_flight_ops.contains(&op.id) {
                    op.runtime_status = OperationStatus::Generating;
                } else if self.get(&op.id, hash).is_some() {
                    op.runtime_status = OperationStatus::Clean;
                } else if let Some(err_msg) = self.errors.get(&(op.id.clone(), hash)) {
                    op.runtime_status = OperationStatus::Error {
                        message: err_msg.clone(),
                    };
                } else if !matches!(op.runtime_status, OperationStatus::NeedsRepick { .. }) {
                    op.runtime_status = OperationStatus::Dirty;
                }
            }
        }
    }
}
