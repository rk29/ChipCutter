use std::path::Path;

use chip_cad::CadDocument;
use chip_cam::{Operation, OperationKind, RestMachiningMode, Setup, WorkOffsetSlot};
use chip_project::{ModelEntry, ProjectDocument, UndoHistory, resolve_upstream_rest_op_indices};
use chip_tools::ToolLibrary;

use crate::panels::{InspectorFocus, OperationTab, SetupTab, ViewportPickMode};

/// High-level UI actions emitted by the CAM Browser pane.
#[derive(Debug, Clone)]
pub enum BrowserAction {
    OpenStepDialog,
    OpenMeshImportModal { browse_file: bool },
    OpenVectorImportModal { browse_file: bool },
    ReloadStepModel,
    ReplaceStepDialog,
    OpenSampleGallery,
    NewSetupXySketch,
    StartPickSketchHostFace,
    AlignCameraToActiveSetup,
    GenerateSetup(usize),
    GenerateSingleOp { setup_idx: usize, op_id: String },
    Log(String),
}

pub(super) fn snapshot_rest_upstream_ids(setup: &Setup) -> Vec<(String, Vec<String>)> {
    setup
        .operations
        .iter()
        .enumerate()
        .filter(|(_, op)| {
            matches!(
                op.passes.rest_machining.mode,
                RestMachiningMode::FromPrecedingOperations { .. }
            )
        })
        .map(|(idx, op)| {
            let upstream_ids = resolve_upstream_rest_op_indices(setup, idx)
                .into_iter()
                .map(|u| setup.operations[u].id.clone())
                .collect();
            (op.id.clone(), upstream_ids)
        })
        .collect()
}

pub(super) fn invalidate_changed_rest_dependents(
    setup: &mut Setup,
    before: &[(String, Vec<String>)],
) {
    for idx in 0..setup.operations.len() {
        if !matches!(
            setup.operations[idx].passes.rest_machining.mode,
            RestMachiningMode::FromPrecedingOperations { .. }
        ) {
            continue;
        }
        let after_ids: Vec<String> = resolve_upstream_rest_op_indices(setup, idx)
            .into_iter()
            .map(|u| setup.operations[u].id.clone())
            .collect();
        let op_id = &setup.operations[idx].id;
        let unchanged = before
            .iter()
            .find(|(id, _)| id == op_id)
            .is_some_and(|(_, prev)| prev == &after_ids);
        if !unchanged {
            setup.operations[idx].mark_dirty();
        }
    }
}

/// Moves `setup.operations[from_idx]` to insertion slot `to_slot` (`0..=operations.len()`),
/// returning the new operation index if its position actually changed, and marking any affected
/// `FromPrecedingOperations` Rest Machining dependents `Dirty`.
pub fn move_operation_to_slot(setup: &mut Setup, from_idx: usize, to_slot: usize) -> Option<usize> {
    let len = setup.operations.len();
    if from_idx >= len || to_slot > len || to_slot == from_idx || to_slot == from_idx + 1 {
        return None;
    }
    let before_rest = snapshot_rest_upstream_ids(setup);
    let moved = setup.operations.remove(from_idx);
    let dst_idx = if to_slot > from_idx {
        to_slot - 1
    } else {
        to_slot
    };
    setup.operations.insert(dst_idx, moved);
    invalidate_changed_rest_dependents(setup, &before_rest);
    Some(dst_idx)
}

/// Duplicates `setup.operations[op_idx]` immediately below `op_idx`, returning the new index and
/// propagating Rest Machining dirty invalidation to downstream dependents.
pub fn duplicate_operation_in_setup(setup: &mut Setup, op_idx: usize) -> Option<usize> {
    if op_idx >= setup.operations.len() {
        return None;
    }
    let before_rest = snapshot_rest_upstream_ids(setup);
    let mut dup = setup.operations[op_idx].clone();
    dup.id = format!("{}_copy_{}", dup.id, setup.operations.len() + 1);
    dup.name = format!("{} (Copy)", dup.name);
    dup.mark_dirty();
    let new_idx = op_idx + 1;
    setup.operations.insert(new_idx, dup);
    invalidate_changed_rest_dependents(setup, &before_rest);
    Some(new_idx)
}

/// Removes `setup.operations[op_idx]`, propagating Rest Machining dirty invalidation to downstream
/// dependents.
pub fn delete_operation_in_setup(setup: &mut Setup, op_idx: usize) -> bool {
    if op_idx >= setup.operations.len() {
        return false;
    }
    let before_rest = snapshot_rest_upstream_ids(setup);
    setup.operations.remove(op_idx);
    invalidate_changed_rest_dependents(setup, &before_rest);
    true
}

/// Formats an estimated operation cycle time into a compact browser badge (`42s`, `1m12s`, `1h04m`).
pub fn format_compact_cycle_time(seconds: f64) -> String {
    if !seconds.is_finite() || seconds <= 0.0 {
        return "0s".to_string();
    }
    let total = seconds.round().max(1.0) as u64;
    if total < 60 {
        format!("{total}s")
    } else if total < 3_600 {
        format!("{}m{:02}s", total / 60, total % 60)
    } else {
        format!("{}h{:02}m", total / 3_600, (total % 3_600) / 60)
    }
}

pub(super) fn format_operation_badge(
    project: &ProjectDocument,
    op: &Operation,
    est_time_sec: Option<f64>,
) -> Option<String> {
    let primary_t = op
        .tool_id
        .as_ref()
        .and_then(|id| project.tool(id))
        .map(|t| t.tool_number);
    let clearing_t = op
        .clearing_tool_id()
        .and_then(|id| project.tool(id))
        .map(|t| t.tool_number);

    let tool_part = match (clearing_t, primary_t) {
        (Some(ct), Some(pt)) if ct != pt => Some(format!("T{ct}+T{pt}")),
        (_, Some(pt)) => Some(format!("T{pt}")),
        _ => None,
    };

    match (tool_part, est_time_sec) {
        (Some(tp), Some(sec)) => Some(format!("{tp} · {}", format_compact_cycle_time(sec))),
        (Some(tp), None) => Some(tp),
        (None, Some(sec)) => Some(format_compact_cycle_time(sec)),
        (None, None) => None,
    }
}

/// Creates a default [`Operation`] of `kind` inside `project.setups[setup_idx]`, automatically
/// selecting and embedding the first compatible cutter from `project.tools` or `global_tools`.
pub fn create_default_operation_in_setup(
    project: &mut ProjectDocument,
    global_tools: &ToolLibrary,
    setup_idx: usize,
    kind: OperationKind,
) -> Option<usize> {
    if setup_idx >= project.setups.len() {
        return None;
    }
    let next_op_num = project.setups[setup_idx].operations.len() + 1;
    let mut op = Operation::new(
        format!(
            "op_{setup_idx}_{next_op_num}_{}",
            project.setups[setup_idx].operations.len()
        ),
        format!("{} {next_op_num}", kind.default_name()),
        kind,
    );
    if let Some(compat) = project
        .tools
        .iter()
        .chain(global_tools.tools.iter())
        .find(|t| kind.is_compatible_with_tool(t))
        .cloned()
    {
        project.ensure_tool_embedded(&compat);
        op.tool_id = Some(compat.id);
        op.preset_id = compat.presets.first().map(|p| p.id.clone());
    }
    project.setups[setup_idx].operations.push(op);
    Some(project.setups[setup_idx].operations.len() - 1)
}

/// Returns the default interactive 3D picking mode for an operation on `tab`.
pub fn default_pick_mode_for_op(
    setup_idx: usize,
    op_idx: usize,
    kind: OperationKind,
    tab: OperationTab,
) -> ViewportPickMode {
    if tab != OperationTab::Geometry {
        return ViewportPickMode::InspectEntity;
    }
    match kind {
        OperationKind::Facing
        | OperationKind::Pocket2D
        | OperationKind::VCarve
        | OperationKind::SlotUndercut
        | OperationKind::Dovetail => ViewportPickMode::OpPlanarFaces { setup_idx, op_idx },
        OperationKind::Contour2D | OperationKind::ChamferTrace | OperationKind::Deburr3D => {
            ViewportPickMode::OpClosedLoops { setup_idx, op_idx }
        }
        OperationKind::DrillBore | OperationKind::ThreadMill => {
            ViewportPickMode::OpCylindricalHoles { setup_idx, op_idx }
        }
        OperationKind::Adaptive3D
        | OperationKind::Waterline3D
        | OperationKind::Parallel3D
        | OperationKind::Scallop3D
        | OperationKind::Pencil3D
        | OperationKind::Undercut3D
        | OperationKind::Rotary2Linear
        | OperationKind::Rotary4AxisSim => ViewportPickMode::InspectEntity,
    }
}

/// Resolves a concise display name for a [`ModelEntry`] row in the Project Tree.
pub(super) fn model_display_name(
    entry: &ModelEntry,
    cad_doc: Option<&CadDocument>,
    is_primary: bool,
) -> String {
    if is_primary
        && let Some(doc) = cad_doc
        && !doc.source_name.trim().is_empty()
    {
        return doc.source_name.clone();
    }
    Path::new(&entry.path)
        .file_name()
        .and_then(|s| s.to_str())
        .filter(|s| !s.is_empty())
        .unwrap_or(entry.id.as_str())
        .to_string()
}

/// Creates and activates a new clamping Setup in `project`, recording an atomic Undo snapshot.
pub(super) fn add_new_setup(
    project: &mut ProjectDocument,
    history: &mut UndoHistory,
    active_setup_idx: &mut usize,
    focus: &mut InspectorFocus,
    setup_tab: &mut SetupTab,
    pick_mode: &mut ViewportPickMode,
) -> BrowserAction {
    history.record_before_edit("Add Setup", None, project);
    let next_n = project.setups.len() + 1;
    let slot = match next_n {
        1 => WorkOffsetSlot::G54,
        2 => WorkOffsetSlot::G55,
        3 => WorkOffsetSlot::G56,
        4 => WorkOffsetSlot::G57,
        _ => WorkOffsetSlot::G58,
    };
    project.setups.push(Setup::new(
        format!("setup_{next_n}"),
        format!("Setup {next_n} — Op {next_n}"),
        slot,
    ));
    let idx = project.setups.len() - 1;
    *active_setup_idx = idx;
    *focus = InspectorFocus::Setup { setup_idx: idx };
    *setup_tab = SetupTab::Wcs;
    *pick_mode = ViewportPickMode::InspectEntity;
    BrowserAction::Log(format!("Created Setup {next_n}."))
}
