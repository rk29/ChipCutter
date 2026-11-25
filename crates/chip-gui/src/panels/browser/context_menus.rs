//! Consistent right-click context menus (`response.context_menu`) and inline node rename for every
//! node in the Project Tree (`browser/mod.rs`).

use chip_cam::OperationKind;
use chip_project::ProjectDocument;
use chip_tools::ToolId;
use egui::{Color32, Pos2, Rect, Stroke, Ui, Vec2};

use crate::{
    panels::{
        InspectorFocus, ViewportPickMode,
        browser::{
            actions::{AddOperationPreset, BrowserAction, QuickFixturePreset, SketchEntityPreset},
            tree_widget::{
                TREE_CHEVRON_W, TREE_ICON_SIZE, TREE_INDENT_STEP, TREE_LEFT_PAD, TREE_ROW_HEIGHT,
                TreeNodeKey, set_node_open,
            },
        },
    },
    theme::ThemeTokens,
    widgets::{IconKind, paint_icon},
};

/// Active inline rename editor state stored in `egui` temp memory.
///
/// Triggered strictly via right-click `Rename (F2)` or pressing `F2` (never permanent row buttons).
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct InlineRenameState {
    pub key: TreeNodeKey,
    pub buffer: String,
    pub original: String,
    pub request_focus: bool,
}

/// Deferred browser mutations accumulated during tree rendering and executed atomically after the
/// `ScrollArea` closes so idle frames never clone `ProjectDocument`.
#[derive(Default)]
pub(super) struct DeferredBrowserMutations {
    pub expand_collapse_all: Option<bool>,
    pub add_setup: bool,
    pub setup_duplicate: Option<usize>,
    pub setup_move_slot: Option<(usize, usize)>,
    pub setup_delete: Option<usize>,
    pub setup_add_op: Option<(usize, AddOperationPreset)>,
    pub setup_add_fixture: Option<(usize, QuickFixturePreset)>,

    pub sketch_vis_toggle: Option<usize>,
    pub sketch_add_entity: Option<(usize, SketchEntityPreset)>,
    pub sketch_duplicate: Option<usize>,
    pub sketch_move_slot: Option<(usize, usize)>,
    pub sketch_delete: Option<usize>,

    pub sketch_entity_toggle_construction: Option<(usize, usize)>,
    pub sketch_entity_duplicate: Option<(usize, usize)>,
    pub sketch_entity_move_slot: Option<(usize, usize, usize)>,
    pub sketch_entity_delete: Option<(usize, usize)>,

    pub avoid_add: bool,
    pub avoid_toggle_suppress: Option<usize>,
    pub avoid_apply_to_active_setup: Option<String>,
    pub avoid_duplicate: Option<usize>,
    pub avoid_move_slot: Option<(usize, usize)>,
    pub avoid_delete: Option<String>,

    pub add_tool: Option<chip_tools::Tool>,
    pub sync_tool_from_machine: Option<ToolId>,
    pub remove_tool: Option<ToolId>,
    pub prune_unused_tools: bool,

    pub fixture_toggle_enabled: Option<(usize, usize)>,
    pub fixture_duplicate: Option<(usize, usize)>,
    pub fixture_move_slot: Option<(usize, usize, usize)>,
    pub fixture_delete: Option<(usize, usize)>,

    pub op_move_slot: Option<(usize, usize, usize)>,
    pub op_delete: Option<(usize, usize)>,
    pub op_duplicate: Option<(usize, usize)>,
    pub op_toggle_suppress: Option<(usize, usize)>,
    pub op_toggle_visibility: Option<String>,

    pub inline_rename_commit: Option<(TreeNodeKey, String)>,
}

/// Constructs an [`InlineRenameState`] for any user-renamable tree node (`Setup`, `Operation`,
/// `Sketch`, `SketchEntity`, `AvoidRegion`, `Fixture`).
pub(super) fn start_inline_rename_for_key(
    key: &TreeNodeKey,
    project: &ProjectDocument,
) -> Option<InlineRenameState> {
    let current = match *key {
        TreeNodeKey::Setup(s_idx) => project.setups.get(s_idx)?.name.clone(),
        TreeNodeKey::Operation { setup_idx, op_idx } => project
            .setups
            .get(setup_idx)?
            .operations
            .get(op_idx)?
            .name
            .clone(),
        TreeNodeKey::Sketch(sk_idx) => project.sketches.get(sk_idx)?.name.clone(),
        TreeNodeKey::SketchEntity {
            sketch_idx,
            entity_idx,
        } => project
            .sketches
            .get(sketch_idx)?
            .entities
            .get(entity_idx)?
            .name
            .clone(),
        TreeNodeKey::AvoidRegion(r_idx) => project.avoid_regions.get(r_idx)?.name.clone(),
        TreeNodeKey::Fixture {
            setup_idx,
            fixture_idx,
        } => project
            .setups
            .get(setup_idx)?
            .fixtures
            .get(fixture_idx)?
            .name
            .clone(),
        _ => return None,
    };
    Some(InlineRenameState {
        key: key.clone(),
        buffer: current.clone(),
        original: current,
        request_focus: true,
    })
}

/// Renders an inline rename text field in place of a tree row.
///
/// Commits on `Enter` or focus loss (`Some((key, trimmed))`) and cancels on `Escape` (`None`).
pub(super) fn render_inline_rename_row(
    ui: &mut Ui,
    depth: usize,
    icon: IconKind,
    icon_color: Color32,
    state: &mut Option<InlineRenameState>,
) -> Option<(TreeNodeKey, String)> {
    let tok = ThemeTokens::from_ui(ui);
    let mut commit: Option<(TreeNodeKey, String)> = None;
    let mut cancel = false;

    let st = state.as_mut()?;

    let full_w = ui.available_width().max(120.0);
    let (_, row_rect) = ui.allocate_space(Vec2::new(full_w, TREE_ROW_HEIGHT));
    if ui.is_rect_visible(row_rect) {
        let guide_stroke = Stroke::new(1.0, tok.border.gamma_multiply(0.55));
        for lvl in 0..depth {
            let gx = row_rect.left()
                + TREE_LEFT_PAD
                + (lvl as f32) * TREE_INDENT_STEP
                + TREE_CHEVRON_W * 0.5;
            ui.painter().line_segment(
                [
                    Pos2::new(gx, row_rect.top()),
                    Pos2::new(gx, row_rect.bottom()),
                ],
                guide_stroke,
            );
        }
    }

    let left_x =
        row_rect.left() + TREE_LEFT_PAD + (depth as f32) * TREE_INDENT_STEP + TREE_CHEVRON_W;
    let icon_rect = Rect::from_min_size(
        Pos2::new(left_x + 2.0, row_rect.center().y - TREE_ICON_SIZE * 0.5),
        Vec2::splat(TREE_ICON_SIZE),
    );
    if ui.is_rect_visible(icon_rect) {
        paint_icon(ui.painter(), icon_rect, icon, icon_color);
    }

    let edit_rect = Rect::from_min_max(
        Pos2::new(icon_rect.right() + 5.0, row_rect.top() + 1.0),
        Pos2::new(
            (row_rect.right() - 4.0).max(icon_rect.right() + 40.0),
            row_rect.bottom() - 1.0,
        ),
    );
    let mut child = ui.new_child(egui::UiBuilder::new().max_rect(edit_rect));
    let resp = child.add(
        egui::TextEdit::singleline(&mut st.buffer)
            .id_salt(("tree_inline_rename", &st.key))
            .desired_width(edit_rect.width()),
    );
    let just_requested = st.request_focus;
    if st.request_focus {
        resp.request_focus();
        st.request_focus = false;
    }

    let esc_pressed = child.input(|i| i.key_pressed(egui::Key::Escape));
    let enter_pressed = child.input(|i| i.key_pressed(egui::Key::Enter));

    if esc_pressed {
        cancel = true;
    } else if enter_pressed || (resp.lost_focus() && !just_requested) {
        let trimmed = st.buffer.trim();
        if !trimmed.is_empty() && trimmed != st.original {
            commit = Some((st.key.clone(), trimmed.to_string()));
        }
        cancel = true;
    }

    if cancel {
        *state = None;
    }
    commit
}

/// Applies `Expand All` or `Collapse All` across all expandable nodes in the Project Tree.
pub(super) fn apply_expand_collapse_all(
    ui: &Ui,
    tree_scope: egui::Id,
    project: &ProjectDocument,
    expand: bool,
) {
    let nid = |k: TreeNodeKey| tree_scope.with("tree_node").with(k);
    // Keep the Project root open so all top-level folders remain accessible
    set_node_open(ui, nid(TreeNodeKey::Project), true);
    for folder in [
        TreeNodeKey::ModelsFolder,
        TreeNodeKey::SketchesFolder,
        TreeNodeKey::AvoidRegionsFolder,
        TreeNodeKey::ToolsFolder,
        TreeNodeKey::SetupsFolder,
    ] {
        set_node_open(ui, nid(folder), expand);
    }
    for sk_idx in 0..project.sketches.len() {
        set_node_open(ui, nid(TreeNodeKey::Sketch(sk_idx)), expand);
    }
    for s_idx in 0..project.setups.len() {
        set_node_open(ui, nid(TreeNodeKey::Setup(s_idx)), expand);
        set_node_open(
            ui,
            nid(TreeNodeKey::FixturesFolder { setup_idx: s_idx }),
            expand,
        );
    }
}

// -----------------------------------------------------------------------------
// Context Menus (one clean helper per tree node type)
// -----------------------------------------------------------------------------

pub(super) fn show_project_root_context_menu(
    ui: &mut Ui,
    pending: &mut DeferredBrowserMutations,
    emitted_action: &mut Option<BrowserAction>,
) {
    if ui.button("Project Properties...").clicked() {
        *emitted_action = Some(BrowserAction::OpenProjectProperties);
        ui.close();
    }
    if ui.button("Manage Tools...").clicked() {
        *emitted_action = Some(BrowserAction::OpenToolLibrary {
            select_tool_id: None,
        });
        ui.close();
    }
    ui.separator();
    if ui.button("+ New Setup").clicked() {
        pending.add_setup = true;
        ui.close();
    }
    if ui.button("+ New 2D Sketch (Setup XY)").clicked() {
        *emitted_action = Some(BrowserAction::NewSetupXySketch);
        ui.close();
    }
    if ui.button("+ New Keep-Out Region").clicked() {
        pending.avoid_add = true;
        ui.close();
    }
    ui.separator();
    if ui.button("Import STEP Model...").clicked() {
        *emitted_action = Some(BrowserAction::OpenStepDialog);
        ui.close();
    }
    if ui.button("Import 3D Mesh...").clicked() {
        *emitted_action = Some(BrowserAction::OpenMeshImportModal { browse_file: true });
        ui.close();
    }
    if ui.button("Import 2D Vector...").clicked() {
        *emitted_action = Some(BrowserAction::OpenVectorImportModal { browse_file: true });
        ui.close();
    }
    ui.separator();
    if ui.button("Expand All").clicked() {
        pending.expand_collapse_all = Some(true);
        ui.close();
    }
    if ui.button("Collapse All").clicked() {
        pending.expand_collapse_all = Some(false);
        ui.close();
    }
}

pub(super) fn show_models_folder_context_menu(
    ui: &mut Ui,
    emitted_action: &mut Option<BrowserAction>,
) {
    if ui.button("Import STEP Model...").clicked() {
        *emitted_action = Some(BrowserAction::OpenStepDialog);
        ui.close();
    }
    if ui
        .button("Import 3D Mesh (.stl / .obj / .3mf)...")
        .clicked()
    {
        *emitted_action = Some(BrowserAction::OpenMeshImportModal { browse_file: true });
        ui.close();
    }
    ui.separator();
    if ui.button("Sample Projects Gallery...").clicked() {
        *emitted_action = Some(BrowserAction::OpenSampleGallery);
        ui.close();
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn show_model_item_context_menu(
    ui: &mut Ui,
    model_idx: usize,
    is_primary: bool,
    has_loaded_cad: bool,
    can_remove: bool,
    focus: &mut InspectorFocus,
    pick_mode: &mut ViewportPickMode,
    emitted_action: &mut Option<BrowserAction>,
) {
    if ui.button("Inspect / Transform").clicked() {
        *focus = InspectorFocus::Model { model_idx };
        *pick_mode = ViewportPickMode::InspectEntity;
        ui.close();
    }
    if ui
        .add_enabled(
            is_primary && has_loaded_cad,
            egui::Button::new("Reload from Disk (Ctrl+R)"),
        )
        .on_disabled_hover_text("Only available for the currently loaded primary 3D model")
        .clicked()
    {
        *emitted_action = Some(BrowserAction::ReloadStepModel);
        ui.close();
    }
    if ui
        .add_enabled(is_primary, egui::Button::new("Replace Model..."))
        .on_disabled_hover_text("Only available for the primary target part model")
        .clicked()
    {
        *emitted_action = Some(BrowserAction::ReplaceStepDialog);
        ui.close();
    }
    ui.separator();
    if ui
        .add_enabled(can_remove, egui::Button::new("Remove Model"))
        .clicked()
    {
        *emitted_action = Some(BrowserAction::RemoveModel(model_idx));
        ui.close();
    }
}

pub(super) fn show_sketches_folder_context_menu(
    ui: &mut Ui,
    has_cad_doc: bool,
    emitted_action: &mut Option<BrowserAction>,
) {
    if ui.button("+ New 2D Sketch (Setup XY)").clicked() {
        *emitted_action = Some(BrowserAction::NewSetupXySketch);
        ui.close();
    }
    if ui
        .add_enabled(
            has_cad_doc,
            egui::Button::new("+ New 2D Sketch on 3D Face..."),
        )
        .on_disabled_hover_text("Import a 3D STEP model first to attach a sketch to a planar face")
        .clicked()
    {
        *emitted_action = Some(BrowserAction::StartPickSketchHostFace);
        ui.close();
    }
    ui.separator();
    if ui.button("Import 2D Vector (.dxf / .svg)...").clicked() {
        *emitted_action = Some(BrowserAction::OpenVectorImportModal { browse_file: true });
        ui.close();
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn show_sketch_item_context_menu(
    ui: &mut Ui,
    sketch_idx: usize,
    sketch_count: usize,
    is_visible: bool,
    sketch_name: &str,
    pending: &mut DeferredBrowserMutations,
    rename_state: &mut Option<InlineRenameState>,
    emitted_action: &mut Option<BrowserAction>,
) {
    ui.menu_button("Add Entity", |ui| {
        for preset in SketchEntityPreset::ALL {
            if ui.button(preset.menu_label()).clicked() {
                pending.sketch_add_entity = Some((sketch_idx, preset));
                ui.close();
            }
        }
    });
    if ui.button("Look At Sketch").clicked() {
        *emitted_action = Some(BrowserAction::LookAtSketch(sketch_idx));
        ui.close();
    }
    if ui
        .button(if is_visible { "Hide" } else { "Show" })
        .clicked()
    {
        pending.sketch_vis_toggle = Some(sketch_idx);
        ui.close();
    }
    ui.separator();
    if ui.button("Rename (F2)").clicked() {
        *rename_state = Some(InlineRenameState {
            key: TreeNodeKey::Sketch(sketch_idx),
            buffer: sketch_name.to_string(),
            original: sketch_name.to_string(),
            request_focus: true,
        });
        ui.close();
    }
    if ui.button("Duplicate").clicked() {
        pending.sketch_duplicate = Some(sketch_idx);
        ui.close();
    }
    if ui
        .add_enabled(sketch_idx > 0, egui::Button::new("Move Up"))
        .clicked()
    {
        pending.sketch_move_slot = Some((sketch_idx, sketch_idx - 1));
        ui.close();
    }
    if ui
        .add_enabled(
            sketch_idx + 1 < sketch_count,
            egui::Button::new("Move Down"),
        )
        .clicked()
    {
        pending.sketch_move_slot = Some((sketch_idx, sketch_idx + 2));
        ui.close();
    }
    ui.separator();
    if ui.button("Delete").clicked() {
        pending.sketch_delete = Some(sketch_idx);
        ui.close();
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn show_sketch_entity_context_menu(
    ui: &mut Ui,
    sketch_idx: usize,
    entity_idx: usize,
    entity_count: usize,
    entity_name: &str,
    pending: &mut DeferredBrowserMutations,
    rename_state: &mut Option<InlineRenameState>,
) {
    if ui.button("Toggle Construction").clicked() {
        pending.sketch_entity_toggle_construction = Some((sketch_idx, entity_idx));
        ui.close();
    }
    ui.separator();
    if ui.button("Rename (F2)").clicked() {
        *rename_state = Some(InlineRenameState {
            key: TreeNodeKey::SketchEntity {
                sketch_idx,
                entity_idx,
            },
            buffer: entity_name.to_string(),
            original: entity_name.to_string(),
            request_focus: true,
        });
        ui.close();
    }
    if ui.button("Duplicate").clicked() {
        pending.sketch_entity_duplicate = Some((sketch_idx, entity_idx));
        ui.close();
    }
    if ui
        .add_enabled(entity_idx > 0, egui::Button::new("Move Up"))
        .clicked()
    {
        pending.sketch_entity_move_slot = Some((sketch_idx, entity_idx, entity_idx - 1));
        ui.close();
    }
    if ui
        .add_enabled(
            entity_idx + 1 < entity_count,
            egui::Button::new("Move Down"),
        )
        .clicked()
    {
        pending.sketch_entity_move_slot = Some((sketch_idx, entity_idx, entity_idx + 2));
        ui.close();
    }
    ui.separator();
    if ui.button("Delete").clicked() {
        pending.sketch_entity_delete = Some((sketch_idx, entity_idx));
        ui.close();
    }
}

pub(super) fn show_avoid_folder_context_menu(ui: &mut Ui, pending: &mut DeferredBrowserMutations) {
    if ui.button("+ New Keep-Out Region").clicked() {
        pending.avoid_add = true;
        ui.close();
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn show_avoid_item_context_menu(
    ui: &mut Ui,
    region_idx: usize,
    region_count: usize,
    region_id: &str,
    region_name: &str,
    is_suppressed: bool,
    focus: &mut InspectorFocus,
    editing_shared_avoid_id: &mut Option<String>,
    pick_mode: &mut ViewportPickMode,
    pending: &mut DeferredBrowserMutations,
    rename_state: &mut Option<InlineRenameState>,
) {
    if ui.button("Edit Keep-Out Region").clicked() {
        *focus = InspectorFocus::KeepOutRegion { region_idx };
        *editing_shared_avoid_id = Some(region_id.to_string());
        *pick_mode = ViewportPickMode::InspectEntity;
        ui.close();
    }
    if ui
        .button(if is_suppressed {
            "Unsuppress"
        } else {
            "Suppress"
        })
        .clicked()
    {
        pending.avoid_toggle_suppress = Some(region_idx);
        ui.close();
    }
    if ui.button("Apply to Active Setup Ops").clicked() {
        pending.avoid_apply_to_active_setup = Some(region_id.to_string());
        ui.close();
    }
    ui.separator();
    if ui.button("Rename (F2)").clicked() {
        *rename_state = Some(InlineRenameState {
            key: TreeNodeKey::AvoidRegion(region_idx),
            buffer: region_name.to_string(),
            original: region_name.to_string(),
            request_focus: true,
        });
        ui.close();
    }
    if ui.button("Duplicate").clicked() {
        pending.avoid_duplicate = Some(region_idx);
        ui.close();
    }
    if ui
        .add_enabled(region_idx > 0, egui::Button::new("Move Up"))
        .clicked()
    {
        pending.avoid_move_slot = Some((region_idx, region_idx - 1));
        ui.close();
    }
    if ui
        .add_enabled(
            region_idx + 1 < region_count,
            egui::Button::new("Move Down"),
        )
        .clicked()
    {
        pending.avoid_move_slot = Some((region_idx, region_idx + 2));
        ui.close();
    }
    ui.separator();
    if ui.button("Delete").clicked() {
        pending.avoid_delete = Some(region_id.to_string());
        ui.close();
    }
}

pub(super) fn show_tools_folder_context_menu(
    ui: &mut Ui,
    unused_count: usize,
    pending: &mut DeferredBrowserMutations,
    emitted_action: &mut Option<BrowserAction>,
) {
    if ui.button("Manage Tools...").clicked() {
        *emitted_action = Some(BrowserAction::OpenToolLibrary {
            select_tool_id: None,
        });
        ui.close();
    }
    if ui
        .add_enabled(
            unused_count > 0,
            egui::Button::new(format!("Remove Unused Tools ({unused_count})")),
        )
        .on_disabled_hover_text("All project tools are currently referenced by operations")
        .clicked()
    {
        pending.prune_unused_tools = true;
        ui.close();
    }
}

pub(super) fn show_tool_item_context_menu(
    ui: &mut Ui,
    tool_id: &ToolId,
    in_machine_library: bool,
    use_count: usize,
    pending: &mut DeferredBrowserMutations,
    emitted_action: &mut Option<BrowserAction>,
) {
    if ui.button("Edit in Tool Library...").clicked() {
        *emitted_action = Some(BrowserAction::OpenToolLibrary {
            select_tool_id: Some(tool_id.clone()),
        });
        ui.close();
    }
    if ui
        .add_enabled(
            in_machine_library,
            egui::Button::new("Sync from Machine Library"),
        )
        .on_disabled_hover_text("No tool with matching ID exists in the active machine library")
        .clicked()
    {
        pending.sync_tool_from_machine = Some(tool_id.clone());
        ui.close();
    }
    ui.separator();
    if ui
        .add_enabled(use_count == 0, egui::Button::new("Remove from Project"))
        .on_disabled_hover_text(
            "Tool is referenced by active operations; reassign those operations first",
        )
        .clicked()
    {
        pending.remove_tool = Some(tool_id.clone());
        ui.close();
    }
}

pub(super) fn show_setups_folder_context_menu(ui: &mut Ui, pending: &mut DeferredBrowserMutations) {
    if ui.button("+ New Setup").clicked() {
        pending.add_setup = true;
        ui.close();
    }
}

fn show_add_operation_submenu(
    ui: &mut Ui,
    setup_idx: usize,
    pending: &mut DeferredBrowserMutations,
) {
    const OPS_25D: [AddOperationPreset; 8] = [
        AddOperationPreset::Adaptive2D,
        AddOperationPreset::Standard(OperationKind::Facing),
        AddOperationPreset::Standard(OperationKind::Contour2D),
        AddOperationPreset::Standard(OperationKind::Pocket2D),
        AddOperationPreset::Standard(OperationKind::DrillBore),
        AddOperationPreset::Standard(OperationKind::ThreadMill),
        AddOperationPreset::Standard(OperationKind::ChamferTrace),
        AddOperationPreset::Standard(OperationKind::VCarve),
    ];
    const OPS_3D: [AddOperationPreset; 5] = [
        AddOperationPreset::Standard(OperationKind::Adaptive3D),
        AddOperationPreset::Standard(OperationKind::Waterline3D),
        AddOperationPreset::Standard(OperationKind::Parallel3D),
        AddOperationPreset::Standard(OperationKind::Scallop3D),
        AddOperationPreset::Standard(OperationKind::Pencil3D),
    ];
    const OPS_UNDERCUT_DEBURR: [AddOperationPreset; 4] = [
        AddOperationPreset::Standard(OperationKind::SlotUndercut),
        AddOperationPreset::Standard(OperationKind::Dovetail),
        AddOperationPreset::Standard(OperationKind::Deburr3D),
        AddOperationPreset::Standard(OperationKind::Undercut3D),
    ];
    const OPS_4AXIS: [AddOperationPreset; 2] = [
        AddOperationPreset::Standard(OperationKind::Rotary2Linear),
        AddOperationPreset::Standard(OperationKind::Rotary4AxisSim),
    ];

    ui.menu_button("Add Operation", |ui| {
        ui.menu_button("2.5D Milling", |ui| {
            for preset in OPS_25D {
                if ui.button(preset.label()).clicked() {
                    pending.setup_add_op = Some((setup_idx, preset));
                    ui.close();
                }
            }
        });
        ui.menu_button("3D Surface", |ui| {
            for preset in OPS_3D {
                if ui.button(preset.label()).clicked() {
                    pending.setup_add_op = Some((setup_idx, preset));
                    ui.close();
                }
            }
        });
        ui.menu_button("Undercut & Deburr", |ui| {
            for preset in OPS_UNDERCUT_DEBURR {
                if ui.button(preset.label()).clicked() {
                    pending.setup_add_op = Some((setup_idx, preset));
                    ui.close();
                }
            }
        });
        ui.menu_button("4th-Axis Rotary", |ui| {
            for preset in OPS_4AXIS {
                if ui.button(preset.label()).clicked() {
                    pending.setup_add_op = Some((setup_idx, preset));
                    ui.close();
                }
            }
        });
    });
}

fn show_add_fixture_submenu(
    ui: &mut Ui,
    setup_idx: usize,
    include_wizards: bool,
    pending: &mut DeferredBrowserMutations,
    emitted_action: &mut Option<BrowserAction>,
) {
    ui.menu_button("Add Fixture", |ui| {
        for preset in QuickFixturePreset::ALL {
            if ui.button(preset.menu_label()).clicked() {
                pending.setup_add_fixture = Some((setup_idx, preset));
                ui.close();
            }
        }
        if include_wizards {
            ui.separator();
            if ui.button("Flip-Jig Wizard...").clicked() {
                *emitted_action = Some(BrowserAction::OpenFlipJigWizard(setup_idx));
                ui.close();
            }
            if ui.button("Auto Soft-Jaw Wizard...").clicked() {
                *emitted_action = Some(BrowserAction::OpenSoftJawWizard(setup_idx));
                ui.close();
            }
        }
    });
}

#[allow(clippy::too_many_arguments)]
pub(super) fn show_setup_item_context_menu(
    ui: &mut Ui,
    setup_idx: usize,
    setup_count: usize,
    setup_name: &str,
    active_setup_idx: &mut usize,
    focus: &mut InspectorFocus,
    pick_mode: &mut ViewportPickMode,
    pending: &mut DeferredBrowserMutations,
    rename_state: &mut Option<InlineRenameState>,
    emitted_action: &mut Option<BrowserAction>,
) {
    if ui.button("Set Active").clicked() {
        *active_setup_idx = setup_idx;
        *focus = InspectorFocus::Setup { setup_idx };
        *pick_mode = ViewportPickMode::InspectEntity;
        ui.close();
    }
    if ui.button("Generate Setup").clicked() {
        *active_setup_idx = setup_idx;
        *emitted_action = Some(BrowserAction::GenerateSetup(setup_idx));
        ui.close();
    }
    if ui.button("Align View to Setup Top").clicked() {
        *active_setup_idx = setup_idx;
        *emitted_action = Some(BrowserAction::AlignCameraToActiveSetup);
        ui.close();
    }
    ui.separator();
    show_add_operation_submenu(ui, setup_idx, pending);
    show_add_fixture_submenu(ui, setup_idx, true, pending, emitted_action);
    ui.separator();
    if ui.button("Rename (F2)").clicked() {
        *rename_state = Some(InlineRenameState {
            key: TreeNodeKey::Setup(setup_idx),
            buffer: setup_name.to_string(),
            original: setup_name.to_string(),
            request_focus: true,
        });
        ui.close();
    }
    if ui.button("Duplicate Setup").clicked() {
        pending.setup_duplicate = Some(setup_idx);
        ui.close();
    }
    if ui
        .add_enabled(setup_idx > 0, egui::Button::new("Move Up"))
        .clicked()
    {
        pending.setup_move_slot = Some((setup_idx, setup_idx - 1));
        ui.close();
    }
    if ui
        .add_enabled(setup_idx + 1 < setup_count, egui::Button::new("Move Down"))
        .clicked()
    {
        pending.setup_move_slot = Some((setup_idx, setup_idx + 2));
        ui.close();
    }
    ui.separator();
    if ui
        .add_enabled(setup_count > 1, egui::Button::new("Delete Setup"))
        .on_disabled_hover_text("A project must keep at least one Setup")
        .clicked()
    {
        pending.setup_delete = Some(setup_idx);
        ui.close();
    }
}

pub(super) fn show_fixtures_folder_context_menu(
    ui: &mut Ui,
    setup_idx: usize,
    pending: &mut DeferredBrowserMutations,
    emitted_action: &mut Option<BrowserAction>,
) {
    show_add_fixture_submenu(ui, setup_idx, false, pending, emitted_action);
    ui.separator();
    if ui.button("Flip-Jig Wizard...").clicked() {
        *emitted_action = Some(BrowserAction::OpenFlipJigWizard(setup_idx));
        ui.close();
    }
    if ui.button("Auto Soft-Jaw Wizard...").clicked() {
        *emitted_action = Some(BrowserAction::OpenSoftJawWizard(setup_idx));
        ui.close();
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn show_fixture_item_context_menu(
    ui: &mut Ui,
    setup_idx: usize,
    fixture_idx: usize,
    fixture_count: usize,
    is_enabled: bool,
    fixture_name: &str,
    pending: &mut DeferredBrowserMutations,
    rename_state: &mut Option<InlineRenameState>,
) {
    if ui
        .button(if is_enabled { "Suppress" } else { "Unsuppress" })
        .clicked()
    {
        pending.fixture_toggle_enabled = Some((setup_idx, fixture_idx));
        ui.close();
    }
    ui.separator();
    if ui.button("Rename (F2)").clicked() {
        *rename_state = Some(InlineRenameState {
            key: TreeNodeKey::Fixture {
                setup_idx,
                fixture_idx,
            },
            buffer: fixture_name.to_string(),
            original: fixture_name.to_string(),
            request_focus: true,
        });
        ui.close();
    }
    if ui.button("Duplicate").clicked() {
        pending.fixture_duplicate = Some((setup_idx, fixture_idx));
        ui.close();
    }
    if ui
        .add_enabled(fixture_idx > 0, egui::Button::new("Move Up"))
        .clicked()
    {
        pending.fixture_move_slot = Some((setup_idx, fixture_idx, fixture_idx - 1));
        ui.close();
    }
    if ui
        .add_enabled(
            fixture_idx + 1 < fixture_count,
            egui::Button::new("Move Down"),
        )
        .clicked()
    {
        pending.fixture_move_slot = Some((setup_idx, fixture_idx, fixture_idx + 2));
        ui.close();
    }
    ui.separator();
    if ui.button("Delete").clicked() {
        pending.fixture_delete = Some((setup_idx, fixture_idx));
        ui.close();
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn show_operation_item_context_menu(
    ui: &mut Ui,
    setup_idx: usize,
    op_idx: usize,
    op_len: usize,
    op_id: &str,
    op_name: &str,
    is_suppressed: bool,
    is_hidden_in_vp: bool,
    pending: &mut DeferredBrowserMutations,
    rename_state: &mut Option<InlineRenameState>,
    emitted_action: &mut Option<BrowserAction>,
) {
    if ui
        .button("Generate Operation")
        .on_hover_text("Compute or refresh the toolpath for this operation")
        .clicked()
    {
        *emitted_action = Some(BrowserAction::GenerateSingleOp {
            setup_idx,
            op_id: op_id.to_string(),
        });
        ui.close();
    }
    if ui
        .button(if is_hidden_in_vp {
            "Show Path"
        } else {
            "Hide Path"
        })
        .on_hover_text("Toggle 3D viewport visibility of this operation's toolpath")
        .clicked()
    {
        pending.op_toggle_visibility = Some(op_id.to_string());
        ui.close();
    }
    if ui
        .button(if is_suppressed {
            "Unsuppress"
        } else {
            "Suppress"
        })
        .on_hover_text("Enable or suppress this operation in generation, simulation, and G-code")
        .clicked()
    {
        pending.op_toggle_suppress = Some((setup_idx, op_idx));
        ui.close();
    }
    ui.separator();
    if ui.button("Rename (F2)").clicked() {
        *rename_state = Some(InlineRenameState {
            key: TreeNodeKey::Operation { setup_idx, op_idx },
            buffer: op_name.to_string(),
            original: op_name.to_string(),
            request_focus: true,
        });
        ui.close();
    }
    if ui
        .button("Duplicate (Ctrl+D)")
        .on_hover_text("Insert a copy of this operation directly beneath it")
        .clicked()
    {
        pending.op_duplicate = Some((setup_idx, op_idx));
        ui.close();
    }
    if ui
        .add_enabled(op_idx > 0, egui::Button::new("Move Up (Ctrl+Up)"))
        .on_disabled_hover_text("Already at the top of the Setup")
        .clicked()
    {
        pending.op_move_slot = Some((setup_idx, op_idx, op_idx - 1));
        ui.close();
    }
    if ui
        .add_enabled(
            op_idx + 1 < op_len,
            egui::Button::new("Move Down (Ctrl+Down)"),
        )
        .on_disabled_hover_text("Already at the bottom of the Setup")
        .clicked()
    {
        pending.op_move_slot = Some((setup_idx, op_idx, op_idx + 2));
        ui.close();
    }
    ui.separator();
    if ui
        .button("Delete (Del)")
        .on_hover_text("Remove this operation from the Setup (undoable with Ctrl+Z)")
        .clicked()
    {
        pending.op_delete = Some((setup_idx, op_idx));
        ui.close();
    }
}
