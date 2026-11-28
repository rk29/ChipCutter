mod actions;
mod avoid_section;
mod context_menus;
mod drag_reorder;
mod models_section;
mod setups_section;
mod sketches_section;
mod tools_section;
mod tree_widget;

use chip_cad::CadDocument;
use chip_cam::{OperationStatus, ResolvedSetup, SharedAvoidRegion};
use chip_machines::MachineDefinition;
use chip_project::{ProjectDocument, ToolpathCache, UndoHistory};
use chip_tools::ToolLibrary;
use egui::Id;

use crate::{
    panels::{
        InspectorFocus, OperationTab, SetupTab, ViewportPickMode,
        operation_inspector::avoid_card::{
            mark_shared_avoid_dependents_dirty, next_shared_avoid_id_and_name,
            set_shared_avoid_for_all_setup_ops,
        },
        sketch_inspector::mark_sketch_dependent_ops_dirty,
    },
    theme::ThemeTokens,
    widgets::IconKind,
};

pub use actions::{
    AddOperationPreset, BrowserAction, QuickFixturePreset, SketchEntityPreset,
    create_default_operation_in_setup, default_pick_mode_for_op, delete_operation_in_setup,
    duplicate_operation_in_setup, format_compact_cycle_time,
};
pub use drag_reorder::{
    DragSection, DraggedTreeItem, move_avoid_region_to_slot, move_fixture_to_slot,
    move_operation_to_slot, move_setup_to_slot, move_sketch_entity_to_slot, move_sketch_to_slot,
    remap_index_after_slot_move, resolve_drop_slot_for_section,
};
pub use tree_widget::{
    TreeNavAction, TreeNavKey, TreeNodeKey, VisibleTreeNode, is_node_open,
    resolve_tree_keyboard_nav, set_node_open,
};

use actions::{
    add_fixture_preset_in_setup, add_new_setup, add_sketch_entity_preset,
    create_adaptive_2d_operation_in_setup, duplicate_fixture_in_setup, duplicate_setup_in_project,
    duplicate_sketch_entity_in_sketch, duplicate_sketch_in_project,
    invalidate_changed_rest_dependents, snapshot_rest_upstream_ids,
};
use avoid_section::show_avoid_section;
use context_menus::{
    DeferredBrowserMutations, InlineRenameState, apply_expand_collapse_all,
    show_project_root_context_menu, start_inline_rename_for_key,
};
use models_section::show_models_section;
use setups_section::show_setups_section;
use sketches_section::show_sketches_section;
use tools_section::show_tools_section;
use tree_widget::{TreeRowSpec, poll_tree_nav_key, tree_row};

/// Context references passed to [`show_cam_browser`].
pub struct CamBrowserCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub global_tools: &'a ToolLibrary,
    pub active_machine: Option<&'a MachineDefinition>,
    pub resolved_active_setup: Option<&'a ResolvedSetup>,
    pub history: &'a mut UndoHistory,
    pub toolpath_cache: &'a ToolpathCache,
    pub cad_doc: Option<&'a CadDocument>,
    pub cad_revision: u64,
    pub active_setup_idx: &'a mut usize,
    pub focus: &'a mut InspectorFocus,
    pub setup_tab: &'a mut SetupTab,
    pub op_tab: &'a mut OperationTab,
    pub pick_mode: &'a mut ViewportPickMode,
    pub hidden_operation_ids: &'a mut std::collections::HashSet<String>,
    pub editing_shared_avoid_id: &'a mut Option<String>,
    pub selected_sketch_entity_idx: &'a mut Option<usize>,
    pub selected_fixture_idx: &'a mut Option<usize>,
}

/// Shared per-frame rendering state threaded through browser section modules.
struct TreeRenderState {
    tree_scope: Id,
    drag_state_id: Id,
    tok: ThemeTokens,
    root_key: TreeNodeKey,
    primary_model_idx: usize,
    active_drag: Option<DraggedTreeItem>,
    kbd_focused_key: Option<TreeNodeKey>,
    tree_kbd_active: bool,
    rename_state: Option<InlineRenameState>,
    pending: DeferredBrowserMutations,
    visible_nodes: Vec<VisibleTreeNode>,
    any_row_clicked: bool,
    emitted_action: Option<BrowserAction>,
}

fn node_id_for(scope: Id, key: &TreeNodeKey) -> Id {
    scope.with("tree_node").with(key)
}

fn focus_to_tree_key(
    focus: InspectorFocus,
    setup_tab: SetupTab,
    selected_sketch_entity_idx: Option<usize>,
    selected_fixture_idx: Option<usize>,
    primary_model_idx: usize,
) -> TreeNodeKey {
    match focus {
        InspectorFocus::Project => TreeNodeKey::Project,
        InspectorFocus::CadEntity => TreeNodeKey::Model(primary_model_idx),
        InspectorFocus::Model { model_idx } => TreeNodeKey::Model(model_idx),
        InspectorFocus::Sketch { sketch_idx } => match selected_sketch_entity_idx {
            Some(entity_idx) => TreeNodeKey::SketchEntity {
                sketch_idx,
                entity_idx,
            },
            None => TreeNodeKey::Sketch(sketch_idx),
        },
        InspectorFocus::KeepOutRegion { region_idx } => TreeNodeKey::AvoidRegion(region_idx),
        InspectorFocus::ProjectTool { tool_idx } => TreeNodeKey::Tool(tool_idx),
        InspectorFocus::Setup { setup_idx } => {
            if setup_tab == SetupTab::Fixtures
                && let Some(fixture_idx) = selected_fixture_idx
            {
                TreeNodeKey::Fixture {
                    setup_idx,
                    fixture_idx,
                }
            } else {
                TreeNodeKey::Setup(setup_idx)
            }
        }
        InspectorFocus::Operation { setup_idx, op_idx } => {
            TreeNodeKey::Operation { setup_idx, op_idx }
        }
    }
}

/// Renders the left-hand hierarchical **Project Tree View CAM Browser** (`08-ui-viewport.md`).
pub fn show_cam_browser(ui: &mut egui::Ui, mut ctx: CamBrowserCtx<'_>) -> Option<BrowserAction> {
    if !ctx.project.setups.is_empty() && *ctx.active_setup_idx >= ctx.project.setups.len() {
        *ctx.active_setup_idx = ctx.project.setups.len() - 1;
    }

    let tree_scope = ui.make_persistent_id("cam_project_tree");
    let drag_state_id = tree_scope.with("tree_drag_item");
    let nav_focus_id = tree_scope.with("kbd_focus_key");
    let nav_active_id = tree_scope.with("kbd_active");
    let inline_rename_id = tree_scope.with("inline_rename_state");

    let active_drag: Option<DraggedTreeItem> = ui
        .ctx()
        .data_mut(|d| d.get_temp::<DraggedTreeItem>(drag_state_id));
    let primary_model_idx = ctx.project.primary_part_model_index().unwrap_or(0);
    let kbd_focused_key: Option<TreeNodeKey> = ui
        .ctx()
        .data_mut(|d| d.get_temp::<TreeNodeKey>(nav_focus_id))
        .or_else(|| {
            Some(focus_to_tree_key(
                *ctx.focus,
                *ctx.setup_tab,
                *ctx.selected_sketch_entity_idx,
                *ctx.selected_fixture_idx,
                primary_model_idx,
            ))
        });
    let tree_kbd_active: bool = ui
        .ctx()
        .data_mut(|d| d.get_temp::<bool>(nav_active_id))
        .unwrap_or(false);
    let rename_state: Option<InlineRenameState> = ui
        .ctx()
        .data_mut(|d| d.get_temp::<Option<InlineRenameState>>(inline_rename_id))
        .flatten();

    let mut state = TreeRenderState {
        tree_scope,
        drag_state_id,
        tok: ThemeTokens::from_ui(ui),
        root_key: TreeNodeKey::Project,
        primary_model_idx,
        active_drag,
        kbd_focused_key,
        tree_kbd_active,
        rename_state,
        pending: DeferredBrowserMutations::default(),
        visible_nodes: Vec::with_capacity(32),
        any_row_clicked: false,
        emitted_action: None,
    };

    let scroll_out = egui::ScrollArea::vertical()
        .id_salt("cam_project_tree_scroll")
        .auto_shrink([false, false])
        .show(ui, |ui| {
            ui.spacing_mut().item_spacing.y = 1.0;
            state.tok = ThemeTokens::from_ui(ui);

            // -----------------------------------------------------------------
            // Root: [Project] <project.project_name>
            // -----------------------------------------------------------------
            let root_key = state.root_key.clone();
            let root_id = node_id_for(state.tree_scope, &root_key);
            let root_selected = *ctx.focus == InspectorFocus::Project;
            let root_out = tree_row(
                ui,
                TreeRowSpec {
                    node_id: root_id,
                    depth: 0,
                    icon: IconKind::Project,
                    icon_color: state.tok.accent,
                    label: &ctx.project.project_name,
                    label_color: state.tok.text_primary,
                    bold: true,
                    strikethrough: false,
                    italic: false,
                    has_children: true,
                    default_open: true,
                    selected: root_selected,
                    keyboard_focused: state.tree_kbd_active
                        && state.kbd_focused_key.as_ref() == Some(&root_key),
                    draggable: false,
                    tooltip: Some(
                        "Click to inspect Project Properties (Right-click for project actions)",
                    ),
                },
                |_ui, _hovered| {},
            );
            state.visible_nodes.push(VisibleTreeNode {
                key: root_key.clone(),
                node_id: root_id,
                parent: None,
                has_children: true,
                is_open: root_out.is_open,
            });
            if root_out.response.clicked() {
                *ctx.focus = InspectorFocus::Project;
                *ctx.pick_mode = ViewportPickMode::InspectEntity;
                state.kbd_focused_key = Some(root_key.clone());
                state.any_row_clicked = true;
            } else if root_out.chevron_toggled {
                state.kbd_focused_key = Some(root_key.clone());
                state.any_row_clicked = true;
            }
            root_out.response.context_menu(|ui| {
                show_project_root_context_menu(ui, &mut state.pending, &mut state.emitted_action);
            });

            if !root_out.is_open {
                return;
            }

            show_models_section(ui, &mut ctx, &mut state);
            show_sketches_section(ui, &mut ctx, &mut state);
            show_avoid_section(ui, &mut ctx, &mut state);
            show_tools_section(ui, &mut ctx, &mut state);
            show_setups_section(ui, &mut ctx, &mut state);
        });

    let CamBrowserCtx {
        project,
        global_tools,
        history,
        active_setup_idx,
        focus,
        setup_tab,
        op_tab,
        pick_mode,
        hidden_operation_ids,
        editing_shared_avoid_id,
        selected_sketch_entity_idx,
        selected_fixture_idx,
        ..
    } = ctx;
    let TreeRenderState {
        tree_scope,
        drag_state_id,
        active_drag,
        mut kbd_focused_key,
        mut tree_kbd_active,
        mut rename_state,
        mut pending,
        visible_nodes,
        any_row_clicked,
        mut emitted_action,
        ..
    } = state;

    let tree_rect = scroll_out.inner_rect;
    if any_row_clicked {
        tree_kbd_active = true;
    } else if ui.input(|i| i.pointer.any_click())
        && let Some(pos) = ui.input(|i| i.pointer.interact_pos())
        && !tree_rect.contains(pos)
    {
        tree_kbd_active = false;
    }

    // Keyboard tree navigation (Up/Down/Left/Right/F2/Del)
    let can_nav = tree_kbd_active || ui.rect_contains_pointer(tree_rect);
    if can_nav && let Some(nav_key) = poll_tree_nav_key(ui) {
        tree_kbd_active = true;
        let actions = resolve_tree_keyboard_nav(&visible_nodes, kbd_focused_key.as_ref(), nav_key);
        for act in actions {
            match act {
                TreeNavAction::SetOpen { node_id, open } => {
                    set_node_open(ui, node_id, open);
                }
                TreeNavAction::Select(key) => {
                    apply_tree_node_selection(
                        &key,
                        project,
                        active_setup_idx,
                        focus,
                        setup_tab,
                        op_tab,
                        pick_mode,
                        editing_shared_avoid_id,
                        selected_sketch_entity_idx,
                        selected_fixture_idx,
                    );
                    kbd_focused_key = Some(key);
                }
                TreeNavAction::Rename(key) => {
                    if let Some(st) = start_inline_rename_for_key(&key, project) {
                        rename_state = Some(st);
                    }
                }
                TreeNavAction::Delete(key) => match key {
                    TreeNodeKey::Operation { setup_idx, op_idx } => {
                        pending.op_delete = Some((setup_idx, op_idx));
                    }
                    TreeNodeKey::Sketch(sk_idx) => {
                        pending.sketch_delete = Some(sk_idx);
                    }
                    TreeNodeKey::SketchEntity {
                        sketch_idx,
                        entity_idx,
                    } => {
                        pending.sketch_entity_delete = Some((sketch_idx, entity_idx));
                    }
                    TreeNodeKey::AvoidRegion(r_idx) if r_idx < project.avoid_regions.len() => {
                        pending.avoid_delete = Some(project.avoid_regions[r_idx].id.clone());
                    }
                    TreeNodeKey::Fixture {
                        setup_idx,
                        fixture_idx,
                    } => {
                        pending.fixture_delete = Some((setup_idx, fixture_idx));
                    }
                    TreeNodeKey::Tool(t_idx) if t_idx < project.tools.len() => {
                        let tid = project.tools[t_idx].id.clone();
                        let in_use = project.tool_usage_map().contains_key(&tid);
                        if !in_use {
                            pending.remove_tool = Some(tid);
                        }
                    }
                    TreeNodeKey::Setup(s_idx) if project.setups.len() > 1 => {
                        pending.setup_delete = Some(s_idx);
                    }
                    _ => {}
                },
            }
        }
    }

    // Apply Expand All / Collapse All
    if let Some(expand) = pending.expand_collapse_all {
        apply_expand_collapse_all(ui, tree_scope, project, expand);
    }

    // Persist temp UI states
    ui.ctx().data_mut(|d| {
        d.insert_temp(inline_rename_id, rename_state);
        if let Some(k) = kbd_focused_key {
            d.insert_temp(nav_focus_id, k);
        }
        d.insert_temp(nav_active_id, tree_kbd_active);
    });

    if active_drag.is_some() && ui.input(|i| i.pointer.any_released()) {
        ui.ctx()
            .data_mut(|d| d.remove::<DraggedTreeItem>(drag_state_id));
    }

    // -------------------------------------------------------------------------
    // Apply deferred mutations cleanly with atomic Undo & Dirty propagation
    // -------------------------------------------------------------------------
    if let Some((key, new_name)) = pending.inline_rename_commit {
        apply_inline_rename_commit(project, history, &key, new_name);
    }

    if pending.add_setup {
        emitted_action = Some(add_new_setup(
            project,
            history,
            active_setup_idx,
            focus,
            setup_tab,
            pick_mode,
        ));
    }
    if let Some(s_idx) = pending.setup_duplicate
        && s_idx < project.setups.len()
    {
        history.record_before_edit("Duplicate Setup", None, project);
        if let Some(new_idx) = duplicate_setup_in_project(project, s_idx) {
            *active_setup_idx = new_idx;
            *focus = InspectorFocus::Setup { setup_idx: new_idx };
            *pick_mode = ViewportPickMode::InspectEntity;
            emitted_action = Some(BrowserAction::InvalidateFrameCache);
        }
    }
    if let Some((from_s, to_slot)) = pending.setup_move_slot
        && from_s < project.setups.len()
        && to_slot != from_s
        && to_slot != from_s + 1
    {
        history.record_before_edit("Reorder Setup", None, project);
        if move_setup_to_slot(project, from_s, to_slot, active_setup_idx, focus).is_some() {
            emitted_action = Some(BrowserAction::InvalidateFrameCache);
        }
    }
    if let Some((s_idx, op_preset)) = pending.setup_add_op
        && s_idx < project.setups.len()
    {
        history.record_before_edit(format!("Add {}", op_preset.label()), None, project);
        let new_op = match op_preset {
            AddOperationPreset::Adaptive2D => {
                create_adaptive_2d_operation_in_setup(project, global_tools, s_idx)
            }
            AddOperationPreset::Standard(kind) => {
                create_default_operation_in_setup(project, global_tools, s_idx, kind)
            }
        };
        if let Some(o_idx) = new_op {
            let kind = project.setups[s_idx].operations[o_idx].kind;
            *active_setup_idx = s_idx;
            *focus = InspectorFocus::Operation {
                setup_idx: s_idx,
                op_idx: o_idx,
            };
            *op_tab = if kind == chip_cam::OperationKind::Facing || kind.is_3d_surface() {
                OperationTab::Passes
            } else {
                OperationTab::Geometry
            };
            *pick_mode = default_pick_mode_for_op(s_idx, o_idx, kind, *op_tab);
            set_node_open(
                ui,
                node_id_for(tree_scope, &TreeNodeKey::Setup(s_idx)),
                true,
            );
            set_node_open(
                ui,
                node_id_for(
                    tree_scope,
                    &TreeNodeKey::OperationsFolder { setup_idx: s_idx },
                ),
                true,
            );
            emitted_action = Some(BrowserAction::InvalidateFrameCache);
        }
    }
    if let Some((s_idx, preset)) = pending.setup_add_fixture
        && s_idx < project.setups.len()
    {
        history.record_before_edit(format!("Add {}", preset.menu_label()), None, project);
        if let Some(f_idx) = add_fixture_preset_in_setup(project, s_idx, preset) {
            *active_setup_idx = s_idx;
            *focus = InspectorFocus::Setup { setup_idx: s_idx };
            *setup_tab = SetupTab::Fixtures;
            *selected_fixture_idx = Some(f_idx);
            *pick_mode = ViewportPickMode::InspectEntity;
            set_node_open(
                ui,
                node_id_for(tree_scope, &TreeNodeKey::Setup(s_idx)),
                true,
            );
            set_node_open(
                ui,
                node_id_for(
                    tree_scope,
                    &TreeNodeKey::FixturesFolder { setup_idx: s_idx },
                ),
                true,
            );
            emitted_action = Some(BrowserAction::InvalidateFrameCache);
        }
    }

    if let Some(ti) = pending.sketch_vis_toggle
        && ti < project.sketches.len()
    {
        history.record_before_edit("Toggle Sketch Visibility", None, project);
        project.sketches[ti].visible = !project.sketches[ti].visible;
    }
    if let Some((sk_idx, preset)) = pending.sketch_add_entity
        && sk_idx < project.sketches.len()
    {
        history.record_before_edit(format!("Add Sketch {}", preset.menu_label()), None, project);
        if let Some(ent_idx) = add_sketch_entity_preset(project, sk_idx, preset) {
            *focus = InspectorFocus::Sketch { sketch_idx: sk_idx };
            *selected_sketch_entity_idx = Some(ent_idx);
            *pick_mode = ViewportPickMode::InspectEntity;
            set_node_open(
                ui,
                node_id_for(tree_scope, &TreeNodeKey::Sketch(sk_idx)),
                true,
            );
            emitted_action = Some(BrowserAction::InvalidateFrameCache);
        }
    }
    if let Some(sk_idx) = pending.sketch_duplicate
        && sk_idx < project.sketches.len()
    {
        history.record_before_edit("Duplicate Sketch Layer", None, project);
        if let Some(new_idx) = duplicate_sketch_in_project(project, sk_idx) {
            *focus = InspectorFocus::Sketch {
                sketch_idx: new_idx,
            };
            *selected_sketch_entity_idx = None;
            emitted_action = Some(BrowserAction::InvalidateFrameCache);
        }
    }
    if let Some((from_sk, to_slot)) = pending.sketch_move_slot
        && from_sk < project.sketches.len()
        && to_slot != from_sk
        && to_slot != from_sk + 1
    {
        history.record_before_edit("Reorder Sketch Layer", None, project);
        let _ = move_sketch_to_slot(project, from_sk, to_slot, focus);
    }
    if let Some(di) = pending.sketch_delete
        && di < project.sketches.len()
    {
        history.record_before_edit("Delete Sketch Layer", None, project);
        let del_sk_id = project.sketches[di].id.clone();
        project.sketches.remove(di);
        mark_sketch_dependent_ops_dirty(project, &del_sk_id);
        *selected_sketch_entity_idx = None;
        if matches!(*focus, InspectorFocus::Sketch { .. }) {
            *focus = InspectorFocus::Setup {
                setup_idx: *active_setup_idx,
            };
        }
        emitted_action = Some(BrowserAction::InvalidateFrameCache);
    }

    if let Some((sk_idx, ent_idx)) = pending.sketch_entity_toggle_construction
        && sk_idx < project.sketches.len()
        && ent_idx < project.sketches[sk_idx].entities.len()
    {
        history.record_before_edit("Toggle Sketch Entity Construction", None, project);
        project.sketches[sk_idx].entities[ent_idx].is_construction =
            !project.sketches[sk_idx].entities[ent_idx].is_construction;
        project.sketches[sk_idx].rebuild_regions(None);
        let sk_id = project.sketches[sk_idx].id.clone();
        mark_sketch_dependent_ops_dirty(project, &sk_id);
        emitted_action = Some(BrowserAction::InvalidateFrameCache);
    }
    if let Some((sk_idx, ent_idx)) = pending.sketch_entity_duplicate
        && sk_idx < project.sketches.len()
        && ent_idx < project.sketches[sk_idx].entities.len()
    {
        history.record_before_edit("Duplicate Sketch Entity", None, project);
        if let Some(new_idx) = duplicate_sketch_entity_in_sketch(project, sk_idx, ent_idx) {
            *focus = InspectorFocus::Sketch { sketch_idx: sk_idx };
            *selected_sketch_entity_idx = Some(new_idx);
            emitted_action = Some(BrowserAction::InvalidateFrameCache);
        }
    }
    if let Some((sk_idx, from_e, to_slot)) = pending.sketch_entity_move_slot
        && sk_idx < project.sketches.len()
        && to_slot != from_e
        && to_slot != from_e + 1
    {
        history.record_before_edit("Reorder Sketch Entity", None, project);
        if move_sketch_entity_to_slot(project, sk_idx, from_e, to_slot, selected_sketch_entity_idx)
            .is_some()
        {
            emitted_action = Some(BrowserAction::InvalidateFrameCache);
        }
    }
    if let Some((sk_idx, ent_idx)) = pending.sketch_entity_delete
        && sk_idx < project.sketches.len()
        && ent_idx < project.sketches[sk_idx].entities.len()
    {
        history.record_before_edit("Delete Sketch Entity", None, project);
        project.sketches[sk_idx].entities.remove(ent_idx);
        project.sketches[sk_idx].rebuild_regions(None);
        let sk_id = project.sketches[sk_idx].id.clone();
        mark_sketch_dependent_ops_dirty(project, &sk_id);
        let rem = project.sketches[sk_idx].entities.len();
        *selected_sketch_entity_idx = if rem > 0 {
            Some(ent_idx.min(rem - 1))
        } else {
            None
        };
        emitted_action = Some(BrowserAction::InvalidateFrameCache);
    }

    if pending.avoid_add {
        history.record_before_edit("Add Shared Keep-Out Region", None, project);
        let (id, name) = next_shared_avoid_id_and_name(project, None);
        project
            .avoid_regions
            .push(SharedAvoidRegion::new(id.clone(), name.clone()));
        let new_idx = project.avoid_regions.len() - 1;
        *focus = InspectorFocus::KeepOutRegion {
            region_idx: new_idx,
        };
        *editing_shared_avoid_id = Some(id);
        *pick_mode = ViewportPickMode::InspectEntity;
        set_node_open(
            ui,
            node_id_for(tree_scope, &TreeNodeKey::AvoidRegionsFolder),
            true,
        );
        emitted_action = Some(BrowserAction::Log(format!(
            "Created shared keep-out region '{name}'."
        )));
    }
    if let Some(r_idx) = pending.avoid_toggle_suppress
        && r_idx < project.avoid_regions.len()
    {
        history.record_before_edit("Toggle Shared Keep-Out Suppress", None, project);
        let rid = project.avoid_regions[r_idx].id.clone();
        project.avoid_regions[r_idx].suppressed = !project.avoid_regions[r_idx].suppressed;
        mark_shared_avoid_dependents_dirty(project, &rid);
        emitted_action = Some(BrowserAction::InvalidateFrameCache);
    }
    if let Some(rid) = pending.avoid_apply_to_active_setup
        && set_shared_avoid_for_all_setup_ops(project, history, *active_setup_idx, &rid, true)
    {
        emitted_action = Some(BrowserAction::Log(format!(
            "Applied keep-out region '{rid}' to all operations in active Setup."
        )));
    }
    if let Some(r_idx) = pending.avoid_duplicate
        && r_idx < project.avoid_regions.len()
    {
        history.record_before_edit("Duplicate Shared Keep-Out Region", None, project);
        let mut dup = project.avoid_regions[r_idx].clone();
        let copy_name = format!("{} (Copy)", dup.name);
        let (new_id, new_name) = next_shared_avoid_id_and_name(project, Some(&copy_name));
        dup.id = new_id.clone();
        dup.name = new_name.clone();
        let dst = r_idx + 1;
        project.avoid_regions.insert(dst, dup);
        *focus = InspectorFocus::KeepOutRegion { region_idx: dst };
        *editing_shared_avoid_id = Some(new_id);
        emitted_action = Some(BrowserAction::Log(format!(
            "Duplicated shared keep-out region as '{new_name}'."
        )));
    }
    if let Some((from_r, to_slot)) = pending.avoid_move_slot
        && from_r < project.avoid_regions.len()
        && to_slot != from_r
        && to_slot != from_r + 1
    {
        history.record_before_edit("Reorder Shared Keep-Out Region", None, project);
        let _ = move_avoid_region_to_slot(project, from_r, to_slot, focus);
    }
    if let Some(del_id) = pending.avoid_delete {
        history.record_before_edit("Delete Shared Keep-Out Region", None, project);
        mark_shared_avoid_dependents_dirty(project, &del_id);
        if project.remove_avoid_region(&del_id) {
            if editing_shared_avoid_id.as_deref() == Some(del_id.as_str()) {
                *editing_shared_avoid_id = None;
            }
            if let InspectorFocus::KeepOutRegion { region_idx } = *focus {
                *focus = if !project.avoid_regions.is_empty() {
                    InspectorFocus::KeepOutRegion {
                        region_idx: region_idx.min(project.avoid_regions.len() - 1),
                    }
                } else {
                    InspectorFocus::Setup {
                        setup_idx: *active_setup_idx,
                    }
                };
            }
            emitted_action = Some(BrowserAction::Log(format!(
                "Deleted shared keep-out region '{del_id}' and unlinked referencing operations."
            )));
        }
    }

    if let Some(new_tool) = pending.add_tool {
        history.record_before_edit("Add Project Tool", None, project);
        let label = new_tool.summary_label();
        project.ensure_tool_embedded(&new_tool);
        emitted_action = Some(BrowserAction::Log(format!(
            "Added tool to project: {label}."
        )));
    }
    if let Some(sync_id) = pending.sync_tool_from_machine
        && let Some(gt) = global_tools.tool(&sync_id).cloned()
    {
        history.record_before_edit("Sync Tool from Machine Library", None, project);
        let lbl = gt.summary_label();
        project.sync_tool_from_global(&gt);
        emitted_action = Some(BrowserAction::Log(format!(
            "Synced project tool from machine library: {lbl}."
        )));
    }
    if let Some(rm_id) = pending.remove_tool {
        history.record_before_edit("Remove Project Tool", None, project);
        if project.remove_tool_if_unused(&rm_id) {
            if let InspectorFocus::ProjectTool { tool_idx } = *focus {
                *focus = if !project.tools.is_empty() {
                    InspectorFocus::ProjectTool {
                        tool_idx: tool_idx.min(project.tools.len() - 1),
                    }
                } else {
                    InspectorFocus::Project
                };
            }
            emitted_action = Some(BrowserAction::Log(format!(
                "Removed unused tool '{}' from project.",
                rm_id.0
            )));
        }
    }
    if pending.prune_unused_tools {
        history.record_before_edit("Remove Unused Project Tools", None, project);
        let removed = project.prune_unused_tools();
        if removed > 0 {
            if let InspectorFocus::ProjectTool { tool_idx } = *focus {
                *focus = if !project.tools.is_empty() {
                    InspectorFocus::ProjectTool {
                        tool_idx: tool_idx.min(project.tools.len() - 1),
                    }
                } else {
                    InspectorFocus::Project
                };
            }
            emitted_action = Some(BrowserAction::Log(format!(
                "Removed {removed} unused tool(s) from project."
            )));
        }
    }

    if let Some(vis_op_id) = pending.op_toggle_visibility
        && !hidden_operation_ids.remove(&vis_op_id)
    {
        hidden_operation_ids.insert(vis_op_id);
    }

    if let Some((s_idx, f_idx)) = pending.fixture_toggle_enabled
        && s_idx < project.setups.len()
        && f_idx < project.setups[s_idx].fixtures.len()
    {
        history.record_before_edit("Toggle Fixture Enabled", None, project);
        project.setups[s_idx].fixtures[f_idx].enabled =
            !project.setups[s_idx].fixtures[f_idx].enabled;
        for op in &mut project.setups[s_idx].operations {
            op.mark_dirty();
        }
        emitted_action = Some(BrowserAction::InvalidateFrameCache);
    }
    if let Some((s_idx, f_idx)) = pending.fixture_duplicate
        && s_idx < project.setups.len()
        && f_idx < project.setups[s_idx].fixtures.len()
    {
        history.record_before_edit("Duplicate Fixture", None, project);
        if let Some(new_idx) = duplicate_fixture_in_setup(project, s_idx, f_idx) {
            *active_setup_idx = s_idx;
            *focus = InspectorFocus::Setup { setup_idx: s_idx };
            *setup_tab = SetupTab::Fixtures;
            *selected_fixture_idx = Some(new_idx);
            emitted_action = Some(BrowserAction::InvalidateFrameCache);
        }
    }
    if let Some((s_idx, from_f, to_slot)) = pending.fixture_move_slot
        && s_idx < project.setups.len()
        && to_slot != from_f
        && to_slot != from_f + 1
    {
        history.record_before_edit("Reorder Fixture", None, project);
        if move_fixture_to_slot(project, s_idx, from_f, to_slot, selected_fixture_idx).is_some() {
            emitted_action = Some(BrowserAction::InvalidateFrameCache);
        }
    }
    if let Some((s_idx, f_idx)) = pending.fixture_delete
        && s_idx < project.setups.len()
        && f_idx < project.setups[s_idx].fixtures.len()
    {
        history.record_before_edit("Delete Fixture", None, project);
        project.setups[s_idx].fixtures.remove(f_idx);
        for op in &mut project.setups[s_idx].operations {
            op.mark_dirty();
        }
        let rem = project.setups[s_idx].fixtures.len();
        *selected_fixture_idx = if rem > 0 {
            Some(f_idx.min(rem - 1))
        } else {
            None
        };
        emitted_action = Some(BrowserAction::InvalidateFrameCache);
    }

    if let Some(del_idx) = pending.setup_delete
        && project.setups.len() > 1
        && del_idx < project.setups.len()
    {
        history.record_before_edit("Delete Setup", None, project);
        let removed_id = project.setups[del_idx].id.clone();
        project.setups.remove(del_idx);
        for s in &mut project.setups {
            if s.carryover_from_setup_id.as_deref() == Some(removed_id.as_str()) {
                s.carryover_from_setup_id = None;
                for op in &mut s.operations {
                    op.mark_dirty();
                }
            }
        }
        *active_setup_idx = (*active_setup_idx).min(project.setups.len().saturating_sub(1));
        *focus = InspectorFocus::Setup {
            setup_idx: *active_setup_idx,
        };
        emitted_action = Some(BrowserAction::InvalidateFrameCache);
    }

    if let Some((s_idx, o_idx)) = pending.op_duplicate
        && s_idx < project.setups.len()
    {
        history.record_before_edit("Duplicate Operation", None, project);
        if let Some(new_idx) = duplicate_operation_in_setup(&mut project.setups[s_idx], o_idx) {
            *focus = InspectorFocus::Operation {
                setup_idx: s_idx,
                op_idx: new_idx,
            };
        }
    }

    if let Some((s_idx, o_idx, slot)) = pending.op_move_slot
        && s_idx < project.setups.len()
        && slot != o_idx
        && slot != o_idx + 1
    {
        history.record_before_edit("Reorder Operation", None, project);
        if let Some(dst_idx) = move_operation_to_slot(&mut project.setups[s_idx], o_idx, slot) {
            *focus = InspectorFocus::Operation {
                setup_idx: s_idx,
                op_idx: dst_idx,
            };
        }
    }

    if let Some((s_idx, o_idx)) = pending.op_toggle_suppress
        && s_idx < project.setups.len()
        && o_idx < project.setups[s_idx].operations.len()
    {
        history.record_before_edit("Toggle Suppress Operation", None, project);
        let before_rest = snapshot_rest_upstream_ids(&project.setups[s_idx]);
        let op = &mut project.setups[s_idx].operations[o_idx];
        op.suppressed = !op.suppressed;
        invalidate_changed_rest_dependents(&mut project.setups[s_idx], &before_rest);
    }

    if let Some((s_idx, o_idx)) = pending.op_delete
        && s_idx < project.setups.len()
    {
        history.record_before_edit("Delete Operation", None, project);
        if delete_operation_in_setup(&mut project.setups[s_idx], o_idx) {
            let rem = project.setups[s_idx].operations.len();
            *focus = if rem > 0 {
                InspectorFocus::Operation {
                    setup_idx: s_idx,
                    op_idx: o_idx.min(rem - 1),
                }
            } else {
                InspectorFocus::Setup { setup_idx: s_idx }
            };
            *pick_mode = ViewportPickMode::InspectEntity;
        }
    }

    emitted_action
}

fn apply_inline_rename_commit(
    project: &mut ProjectDocument,
    history: &mut UndoHistory,
    key: &TreeNodeKey,
    new_name: String,
) {
    match *key {
        TreeNodeKey::Setup(s_idx) if s_idx < project.setups.len() => {
            history.record_before_edit("Rename Setup", None, project);
            project.setups[s_idx].name = new_name;
        }
        TreeNodeKey::Operation { setup_idx, op_idx }
            if setup_idx < project.setups.len()
                && op_idx < project.setups[setup_idx].operations.len() =>
        {
            history.record_before_edit("Rename Operation", None, project);
            project.setups[setup_idx].operations[op_idx].name = new_name;
        }
        TreeNodeKey::Sketch(sk_idx) if sk_idx < project.sketches.len() => {
            history.record_before_edit("Rename Sketch", None, project);
            project.sketches[sk_idx].name = new_name;
        }
        TreeNodeKey::SketchEntity {
            sketch_idx,
            entity_idx,
        } if sketch_idx < project.sketches.len()
            && entity_idx < project.sketches[sketch_idx].entities.len() =>
        {
            history.record_before_edit("Rename Sketch Entity", None, project);
            project.sketches[sketch_idx].entities[entity_idx].name = new_name;
        }
        TreeNodeKey::AvoidRegion(r_idx) if r_idx < project.avoid_regions.len() => {
            history.record_before_edit("Rename Shared Keep-Out Region", None, project);
            project.avoid_regions[r_idx].name = new_name;
        }
        TreeNodeKey::Fixture {
            setup_idx,
            fixture_idx,
        } if setup_idx < project.setups.len()
            && fixture_idx < project.setups[setup_idx].fixtures.len() =>
        {
            history.record_before_edit("Rename Fixture", None, project);
            project.setups[setup_idx].fixtures[fixture_idx].name = new_name;
        }
        _ => {}
    }
}

#[allow(clippy::too_many_arguments)]
fn apply_tree_node_selection(
    key: &TreeNodeKey,
    project: &ProjectDocument,
    active_setup_idx: &mut usize,
    focus: &mut InspectorFocus,
    setup_tab: &mut SetupTab,
    op_tab: &mut OperationTab,
    pick_mode: &mut ViewportPickMode,
    editing_shared_avoid_id: &mut Option<String>,
    selected_sketch_entity_idx: &mut Option<usize>,
    selected_fixture_idx: &mut Option<usize>,
) {
    match *key {
        TreeNodeKey::Project => {
            *focus = InspectorFocus::Project;
            *pick_mode = ViewportPickMode::InspectEntity;
        }
        TreeNodeKey::Model(model_idx) => {
            *focus = InspectorFocus::Model { model_idx };
            *pick_mode = ViewportPickMode::InspectEntity;
        }
        TreeNodeKey::Sketch(sketch_idx) if sketch_idx < project.sketches.len() => {
            *focus = InspectorFocus::Sketch { sketch_idx };
            *selected_sketch_entity_idx = None;
            *pick_mode = ViewportPickMode::InspectEntity;
        }
        TreeNodeKey::SketchEntity {
            sketch_idx,
            entity_idx,
        } if sketch_idx < project.sketches.len()
            && entity_idx < project.sketches[sketch_idx].entities.len() =>
        {
            *focus = InspectorFocus::Sketch { sketch_idx };
            *selected_sketch_entity_idx = Some(entity_idx);
            *pick_mode = ViewportPickMode::InspectEntity;
        }
        TreeNodeKey::AvoidRegion(r_idx) if r_idx < project.avoid_regions.len() => {
            *focus = InspectorFocus::KeepOutRegion { region_idx: r_idx };
            *editing_shared_avoid_id = Some(project.avoid_regions[r_idx].id.clone());
            *pick_mode = ViewportPickMode::InspectEntity;
        }
        TreeNodeKey::Tool(tool_idx) if tool_idx < project.tools.len() => {
            *focus = InspectorFocus::ProjectTool { tool_idx };
            *pick_mode = ViewportPickMode::InspectEntity;
        }
        TreeNodeKey::Setup(setup_idx) if setup_idx < project.setups.len() => {
            *active_setup_idx = setup_idx;
            *focus = InspectorFocus::Setup { setup_idx };
            if *setup_tab == SetupTab::Fixtures {
                *selected_fixture_idx = None;
            }
            *pick_mode = ViewportPickMode::InspectEntity;
        }
        TreeNodeKey::FixturesFolder { setup_idx } if setup_idx < project.setups.len() => {
            *active_setup_idx = setup_idx;
            *focus = InspectorFocus::Setup { setup_idx };
            *setup_tab = SetupTab::Fixtures;
            *selected_fixture_idx = None;
            *pick_mode = ViewportPickMode::InspectEntity;
        }
        TreeNodeKey::Fixture {
            setup_idx,
            fixture_idx,
        } if setup_idx < project.setups.len()
            && fixture_idx < project.setups[setup_idx].fixtures.len() =>
        {
            *active_setup_idx = setup_idx;
            *focus = InspectorFocus::Setup { setup_idx };
            *setup_tab = SetupTab::Fixtures;
            *selected_fixture_idx = Some(fixture_idx);
            *pick_mode = ViewportPickMode::InspectEntity;
        }
        TreeNodeKey::OperationsFolder { setup_idx } if setup_idx < project.setups.len() => {
            *active_setup_idx = setup_idx;
        }
        TreeNodeKey::Operation { setup_idx, op_idx }
            if setup_idx < project.setups.len()
                && op_idx < project.setups[setup_idx].operations.len() =>
        {
            *active_setup_idx = setup_idx;
            *focus = InspectorFocus::Operation { setup_idx, op_idx };
            let op = &project.setups[setup_idx].operations[op_idx];
            if !op.suppressed && matches!(op.runtime_status, OperationStatus::NeedsRepick { .. }) {
                *op_tab = OperationTab::Geometry;
            }
            *pick_mode = default_pick_mode_for_op(setup_idx, op_idx, op.kind, *op_tab);
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests;
