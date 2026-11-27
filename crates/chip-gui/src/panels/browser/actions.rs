use std::path::Path;

use chip_cad::CadDocument;
use chip_cam::{Operation, OperationKind};
use chip_project::{
    ModelEntry, ProjectDocument, UndoHistory, add_setup_in_project,
    add_sketch_entity_preset_in_project,
};
pub use chip_project::{
    QuickFixturePreset, add_fixture_preset_in_setup, create_adaptive_2d_operation_in_setup,
    create_default_operation_in_setup, delete_operation_in_setup, duplicate_fixture_in_setup,
    duplicate_operation_in_setup, duplicate_setup_in_project, duplicate_sketch_entity_in_sketch,
    duplicate_sketch_in_project, invalidate_changed_rest_dependents, snapshot_rest_upstream_ids,
};
use chip_tools::ToolId;

use crate::panels::{InspectorFocus, OperationTab, SetupTab, ViewportPickMode};

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
    let (idx, next_n) = add_setup_in_project(project);
    *active_setup_idx = idx;
    *focus = InspectorFocus::Setup { setup_idx: idx };
    *setup_tab = SetupTab::Wcs;
    *pick_mode = ViewportPickMode::InspectEntity;
    BrowserAction::Log(format!("Created Setup {next_n}."))
}

/// Inserts a preset entity into `project.sketches[sketch_idx]`, rebuilds regions, and dirties ops.
pub(super) fn add_sketch_entity_preset(
    project: &mut ProjectDocument,
    sketch_idx: usize,
    preset: SketchEntityPreset,
) -> Option<usize> {
    let (spec, sub_layer) = preset.spec_index();
    add_sketch_entity_preset_in_project(project, sketch_idx, spec, sub_layer)
}
