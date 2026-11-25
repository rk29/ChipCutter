use std::path::Path;

use chip_cad::CadDocument;
use chip_cam::{
    FixtureBody, FixtureKind, Operation, OperationKind, RestMachiningMode, Setup, WorkOffsetSlot,
};
use chip_math::{DVec2, DVec3, ParamLength};
use chip_project::{ModelEntry, ProjectDocument, UndoHistory, resolve_upstream_rest_op_indices};
use chip_tools::{ToolId, ToolLibrary};

use crate::panels::{
    InspectorFocus, OperationTab, SetupTab, ViewportPickMode,
    sketch_inspector::{create_default_sketch_entity, mark_sketch_dependent_ops_dirty},
};

/// High-level UI actions emitted by the CAM Browser pane.
#[derive(Debug, Clone)]
pub enum BrowserAction {
    OpenProjectProperties,
    OpenToolLibrary { select_tool_id: Option<ToolId> },
    OpenStepDialog,
    OpenMeshImportModal { browse_file: bool },
    OpenVectorImportModal { browse_file: bool },
    ReloadStepModel,
    ReplaceStepDialog,
    RemoveModel(usize),
    OpenSampleGallery,
    NewSetupXySketch,
    StartPickSketchHostFace,
    LookAtSketch(usize),
    AlignCameraToActiveSetup,
    OpenFlipJigWizard(usize),
    OpenSoftJawWizard(usize),
    GenerateSetup(usize),
    GenerateSingleOp { setup_idx: usize, op_id: String },
    InvalidateFrameCache,
    Log(String),
}

/// Valid 2D Sketch entity types insertable from a Sketch node's `Add Entity ->` context submenu.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SketchEntityPreset {
    Rectangle,
    Circle,
    Slot,
    HexPolygon,
    BoltCircle,
    CamText,
}

impl SketchEntityPreset {
    pub const ALL: [Self; 6] = [
        Self::Rectangle,
        Self::Circle,
        Self::Slot,
        Self::HexPolygon,
        Self::BoltCircle,
        Self::CamText,
    ];

    pub const fn menu_label(self) -> &'static str {
        match self {
            Self::Rectangle => "Rectangle",
            Self::Circle => "Circle",
            Self::Slot => "Slot",
            Self::HexPolygon => "Hex / Polygon",
            Self::BoltCircle => "Bolt Circle",
            Self::CamText => "CAM Text",
        }
    }

    pub const fn spec_index(self) -> (u8, Option<&'static str>) {
        match self {
            Self::Rectangle => (0, None),
            Self::Circle => (1, None),
            Self::Slot => (2, None),
            Self::HexPolygon => (3, None),
            Self::BoltCircle => (4, Some("HOLES")),
            Self::CamText => (5, None),
        }
    }
}

/// Valid workholding fixture presets insertable from a Setup or Fixtures folder context menu.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuickFixturePreset {
    ViseJaws,
    ToeClamp,
    Spoilboard,
    BoxKeepOut,
    CylinderKeepOut,
}

impl QuickFixturePreset {
    pub const ALL: [Self; 5] = [
        Self::ViseJaws,
        Self::ToeClamp,
        Self::Spoilboard,
        Self::BoxKeepOut,
        Self::CylinderKeepOut,
    ];

    pub const fn menu_label(self) -> &'static str {
        match self {
            Self::ViseJaws => "Vise Jaws",
            Self::ToeClamp => "Toe Clamp",
            Self::Spoilboard => "Spoilboard",
            Self::BoxKeepOut => "Box Keep-Out",
            Self::CylinderKeepOut => "Cylinder Keep-Out",
        }
    }
}

/// Valid operation kinds insertable from a Setup node's categorized `Add Operation ->` submenu.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddOperationPreset {
    Adaptive2D,
    Standard(OperationKind),
}

impl AddOperationPreset {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Adaptive2D => "2D Adaptive Clearing",
            Self::Standard(k) => k.label(),
        }
    }
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

/// Duplicates `setup.operations[op_idx]` immediately below `op_idx`, returning the new index and
/// propagating Rest Machining dirty invalidation to downstream dependents.
pub fn duplicate_operation_in_setup(setup: &mut Setup, op_idx: usize) -> Option<usize> {
    if op_idx >= setup.operations.len() {
        return None;
    }
    let before_rest = snapshot_rest_upstream_ids(setup);
    let mut dup = setup.operations[op_idx].clone();
    let mut n = setup.operations.len() + 1;
    let base_id = dup.id.clone();
    dup.id = loop {
        let cand = format!("{base_id}_copy_{n}");
        if !setup.operations.iter().any(|o| o.id == cand) {
            break cand;
        }
        n += 1;
    };
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
        (Some(tp), Some(sec)) => Some(format!("{tp} | {}", format_compact_cycle_time(sec))),
        (Some(tp), None) => Some(tp),
        (None, Some(sec)) => Some(format_compact_cycle_time(sec)),
        (None, None) => None,
    }
}

fn next_unique_op_id(setup: &Setup, setup_idx: usize, suffix: &str) -> String {
    let mut n = setup.operations.len() + 1;
    loop {
        let cand = format!("op_{setup_idx}_{n}_{suffix}");
        if !setup.operations.iter().any(|o| o.id == cand) {
            return cand;
        }
        n += 1;
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
    let op_id = next_unique_op_id(
        &project.setups[setup_idx],
        setup_idx,
        &next_op_num.to_string(),
    );
    let mut op = Operation::new(
        op_id,
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
        let material = project.effective_workpiece_material();
        project.ensure_tool_embedded(&compat);
        op.preset_id =
            chip_tools::match_best_preset_for_material(&compat, &material).map(|p| p.id.clone());
        op.tool_id = Some(compat.id);
    }
    project.setups[setup_idx].operations.push(op);
    Some(project.setups[setup_idx].operations.len() - 1)
}

/// Creates a **2D Adaptive Clearing** operation inside `project.setups[setup_idx]`.
pub(super) fn create_adaptive_2d_operation_in_setup(
    project: &mut ProjectDocument,
    global_tools: &ToolLibrary,
    setup_idx: usize,
) -> Option<usize> {
    if setup_idx >= project.setups.len() {
        return None;
    }
    let next_op_num = project.setups[setup_idx].operations.len() + 1;
    let op_id = next_unique_op_id(&project.setups[setup_idx], setup_idx, "adaptive");
    let mut op = Operation::new_adaptive_2d(op_id, format!("2D Adaptive {next_op_num}"));
    if let Some(compat) = project
        .tools
        .iter()
        .chain(global_tools.tools.iter())
        .find(|t| OperationKind::Pocket2D.is_compatible_with_tool(t))
        .cloned()
    {
        let material = project.effective_workpiece_material();
        project.ensure_tool_embedded(&compat);
        op.preset_id =
            chip_tools::match_best_preset_for_material(&compat, &material).map(|p| p.id.clone());
        op.tool_id = Some(compat.id);
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

fn next_unique_setup_id(project: &ProjectDocument) -> String {
    let mut n = project.setups.len() + 1;
    loop {
        let cand = format!("setup_{n}");
        if !project.setups.iter().any(|s| s.id == cand) {
            return cand;
        }
        n += 1;
    }
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
    let id = next_unique_setup_id(project);
    project.setups.push(Setup::new(
        id,
        format!("Setup {next_n} - Op {next_n}"),
        slot,
    ));
    let idx = project.setups.len() - 1;
    *active_setup_idx = idx;
    *focus = InspectorFocus::Setup { setup_idx: idx };
    *setup_tab = SetupTab::Wcs;
    *pick_mode = ViewportPickMode::InspectEntity;
    BrowserAction::Log(format!("Created Setup {next_n}."))
}

/// Duplicates `project.setups[setup_idx]` immediately below it with unique ID and dirty cloned ops.
pub(super) fn duplicate_setup_in_project(
    project: &mut ProjectDocument,
    setup_idx: usize,
) -> Option<usize> {
    let mut copy = project.setups.get(setup_idx)?.clone();
    copy.id = next_unique_setup_id(project);
    copy.name = format!("{} (Copy)", copy.name);
    for op in &mut copy.operations {
        op.mark_dirty();
    }
    let dst = setup_idx + 1;
    project.setups.insert(dst, copy);
    Some(dst)
}

/// Duplicates `project.sketches[sketch_idx]` immediately below it with a unique ID.
pub(super) fn duplicate_sketch_in_project(
    project: &mut ProjectDocument,
    sketch_idx: usize,
) -> Option<usize> {
    let mut copy = project.sketches.get(sketch_idx)?.clone();
    let mut n = project.sketches.len() + 1;
    copy.id = loop {
        let cand = format!("sketch_{n}");
        if !project.sketches.iter().any(|s| s.id == cand) {
            break cand;
        }
        n += 1;
    };
    copy.name = format!("{} (Copy)", copy.name);
    let dst = sketch_idx + 1;
    project.sketches.insert(dst, copy);
    Some(dst)
}

/// Inserts a preset entity into `project.sketches[sketch_idx]`, rebuilds regions, and dirties ops.
pub(super) fn add_sketch_entity_preset(
    project: &mut ProjectDocument,
    sketch_idx: usize,
    preset: SketchEntityPreset,
) -> Option<usize> {
    let sk = project.sketches.get_mut(sketch_idx)?;
    let (spec, sub_layer) = preset.spec_index();
    let mut n = sk.entities.len() + 1;
    let entry = loop {
        let cand = create_default_sketch_entity(n, spec, sub_layer);
        if !sk.entities.iter().any(|e| e.id == cand.id) {
            break cand;
        }
        n += 1;
    };
    sk.entities.push(entry);
    sk.rebuild_regions(None);
    let sk_id = sk.id.clone();
    let new_idx = sk.entities.len() - 1;
    mark_sketch_dependent_ops_dirty(project, &sk_id);
    Some(new_idx)
}

/// Duplicates `project.sketches[sketch_idx].entities[entity_idx]` directly beneath it.
pub(super) fn duplicate_sketch_entity_in_sketch(
    project: &mut ProjectDocument,
    sketch_idx: usize,
    entity_idx: usize,
) -> Option<usize> {
    let sk = project.sketches.get_mut(sketch_idx)?;
    let mut copy = sk.entities.get(entity_idx)?.clone();
    let base_id = copy.id.clone();
    let mut n = sk.entities.len() + 1;
    copy.id = loop {
        let cand = format!("{base_id}_copy_{n}");
        if !sk.entities.iter().any(|e| e.id == cand) {
            break cand;
        }
        n += 1;
    };
    copy.name = format!("{} (Copy)", copy.name);
    let dst = entity_idx + 1;
    sk.entities.insert(dst, copy);
    sk.rebuild_regions(None);
    let sk_id = sk.id.clone();
    mark_sketch_dependent_ops_dirty(project, &sk_id);
    Some(dst)
}

fn next_unique_fixture_id(setup: &Setup, prefix: &str) -> String {
    let mut n = setup.fixtures.len() + 1;
    loop {
        let cand = format!("{prefix}_{n}");
        if !setup.fixtures.iter().any(|f| f.id == cand) {
            return cand;
        }
        n += 1;
    }
}

/// Adds a default workholding fixture of `preset` into `project.setups[setup_idx]` and marks setup ops Dirty.
pub(super) fn add_fixture_preset_in_setup(
    project: &mut ProjectDocument,
    setup_idx: usize,
    preset: QuickFixturePreset,
) -> Option<usize> {
    let setup = project.setups.get_mut(setup_idx)?;
    let body = match preset {
        QuickFixturePreset::ViseJaws => {
            let id = next_unique_fixture_id(setup, "vise");
            let n = setup
                .fixtures
                .iter()
                .filter(|f| matches!(f.kind, FixtureKind::ViseJaws { .. }))
                .count()
                + 1;
            FixtureBody::default_vise(id, format!("Machinist Vise {n}"))
        }
        QuickFixturePreset::ToeClamp => {
            let id = next_unique_fixture_id(setup, "clamp");
            let n = setup
                .fixtures
                .iter()
                .filter(|f| matches!(f.kind, FixtureKind::ToeClamp { .. }))
                .count()
                + 1;
            let yaw = ((n - 1) % 4) as f64 * 90.0;
            FixtureBody::default_toe_clamp(id, format!("Toe Clamp {n}"), DVec2::new(40.0, 0.0), yaw)
        }
        QuickFixturePreset::Spoilboard => {
            let id = next_unique_fixture_id(setup, "spoilboard");
            FixtureBody::default_spoilboard(id, "MDF Spoilboard")
        }
        QuickFixturePreset::BoxKeepOut => {
            let id = next_unique_fixture_id(setup, "keepout");
            let n = setup
                .fixtures
                .iter()
                .filter(|f| matches!(f.kind, FixtureKind::BoxKeepOut { .. }))
                .count()
                + 1;
            FixtureBody::default_box_keepout(
                id,
                format!("Keep-Out Box {n}"),
                DVec3::new(50.0, 0.0, 0.0),
            )
        }
        QuickFixturePreset::CylinderKeepOut => {
            let id = next_unique_fixture_id(setup, "pin");
            let n = setup
                .fixtures
                .iter()
                .filter(|f| matches!(f.kind, FixtureKind::CylinderKeepOut { .. }))
                .count()
                + 1;
            FixtureBody::default_cylinder_keepout(
                id,
                format!("Keep-Out Pin {n}"),
                DVec2::new(-30.0, 30.0),
                -15.0,
                10.0,
            )
        }
    };
    setup.fixtures.push(body);
    for op in &mut setup.operations {
        op.mark_dirty();
    }
    Some(setup.fixtures.len() - 1)
}

/// Duplicates `project.setups[setup_idx].fixtures[fixture_idx]` directly beneath it and marks ops Dirty.
pub(super) fn duplicate_fixture_in_setup(
    project: &mut ProjectDocument,
    setup_idx: usize,
    fixture_idx: usize,
) -> Option<usize> {
    let setup = project.setups.get_mut(setup_idx)?;
    let mut copy = setup.fixtures.get(fixture_idx)?.clone();
    copy.id = next_unique_fixture_id(setup, "fixture");
    copy.name = format!("{} (Copy)", copy.name);
    if let FixtureKind::ToeClamp { anchor_xy, .. } = &mut copy.kind {
        anchor_xy[1] = ParamLength::from_mm(anchor_xy[1].value_mm + 20.0);
    } else if let FixtureKind::BoxKeepOut { center_wcs, .. } = &mut copy.kind {
        center_wcs[1] = ParamLength::from_mm(center_wcs[1].value_mm + 20.0);
    } else if let FixtureKind::CylinderKeepOut { center_xy_wcs, .. } = &mut copy.kind {
        center_xy_wcs[1] = ParamLength::from_mm(center_xy_wcs[1].value_mm + 20.0);
    }
    let dst = fixture_idx + 1;
    setup.fixtures.insert(dst, copy);
    for op in &mut setup.operations {
        op.mark_dirty();
    }
    Some(dst)
}
