use chip_cam::OperationStatus;
use egui::Rect;

use crate::{
    panels::{
        InspectorFocus, OperationTab, SetupTab, ViewportPickMode,
        diagnostics::format_cycle_time,
        operation_inspector::machinability_card::{
            compact_browser_warning_summary, evaluate_operation_machinability,
        },
    },
    widgets::{IconKind, StatusBadgeKind, small_icon_button, status_badge_icon},
};

use super::{
    BrowserAction, CamBrowserCtx, DragSection, DraggedTreeItem, TreeNodeKey, TreeRenderState,
    VisibleTreeNode,
    actions::{default_pick_mode_for_op, format_operation_badge},
    context_menus::{
        render_inline_rename_row, show_add_operation_categories, show_fixture_item_context_menu,
        show_fixtures_folder_context_menu, show_operation_item_context_menu,
        show_operations_folder_context_menu, show_setup_item_context_menu,
        show_setups_folder_context_menu,
    },
    drag_reorder::handle_section_drag_overlay,
    node_id_for,
    tree_widget::{
        self, TreeRowSpec, is_node_open, tree_badge_pill, tree_folder_menu_button, tree_row,
    },
};

pub(super) fn show_setups_section(
    ui: &mut egui::Ui,
    ctx: &mut CamBrowserCtx<'_>,
    state: &mut TreeRenderState,
) {
    let CamBrowserCtx {
        project,
        global_tools,
        active_machine,
        resolved_active_setup,
        toolpath_cache,
        cad_doc,
        cad_revision,
        active_setup_idx,
        focus,
        setup_tab,
        op_tab,
        pick_mode,
        hidden_operation_ids,
        selected_fixture_idx,
        ..
    } = ctx;
    let global_tools = *global_tools;
    let active_machine = *active_machine;
    let resolved_active_setup = *resolved_active_setup;
    let toolpath_cache = *toolpath_cache;
    let cad_doc = *cad_doc;
    let cad_revision = *cad_revision;
    let active_setup_idx: &mut usize = active_setup_idx;
    let focus: &mut InspectorFocus = focus;
    let setup_tab: &mut SetupTab = setup_tab;
    let op_tab: &mut OperationTab = op_tab;
    let pick_mode: &mut ViewportPickMode = pick_mode;
    let hidden_operation_ids: &mut std::collections::HashSet<String> = hidden_operation_ids;
    let selected_fixture_idx: &mut Option<usize> = selected_fixture_idx;
    let TreeRenderState {
        tree_scope,
        drag_state_id,
        tok,
        root_key,
        active_drag,
        kbd_focused_key,
        tree_kbd_active,
        rename_state,
        pending,
        visible_nodes,
        any_row_clicked,
        emitted_action,
        ..
    } = state;
    let tree_scope = *tree_scope;
    let drag_state_id = *drag_state_id;
    let tok = *tok;
    let tree_kbd_active = *tree_kbd_active;

    // -----------------------------------------------------------------
    // 5. Setups (N) -> <setup.name> -> Fixtures (N) & Operations
    // -----------------------------------------------------------------
    let setups_key = TreeNodeKey::SetupsFolder;
    let setups_id = node_id_for(tree_scope, &setups_key);
    let setups_label = format!("Setups ({})", project.setups.len());
    let setups_out = tree_row(
        ui,
        TreeRowSpec {
            node_id: setups_id,
            depth: 1,
            icon: IconKind::Setup,
            icon_color: tok.text_secondary,
            label: &setups_label,
            label_color: tok.text_primary,
            bold: true,
            strikethrough: false,
            italic: false,
            has_children: true,
            default_open: true,
            selected: false,
            keyboard_focused: tree_kbd_active && kbd_focused_key.as_ref() == Some(&setups_key),
            draggable: false,
            tooltip: Some("Clamping Setups, workholding Fixtures, and CAM Operations"),
        },
        |ui, _hovered| {
            if small_icon_button(ui, IconKind::Plus, "Add a new clamping Setup").clicked() {
                pending.add_setup = true;
            }
        },
    );
    visible_nodes.push(VisibleTreeNode {
        key: setups_key.clone(),
        node_id: setups_id,
        parent: Some(root_key.clone()),
        has_children: true,
        is_open: setups_out.is_open,
    });
    if setups_out.response.clicked() || setups_out.chevron_toggled {
        *kbd_focused_key = Some(setups_key.clone());
        *any_row_clicked = true;
    }
    setups_out.response.context_menu(|ui| {
        show_setups_folder_context_menu(ui, pending);
    });

    if setups_out.is_open {
        let setup_count = project.setups.len();
        let mut setup_row_rects: Vec<Rect> = Vec::with_capacity(setup_count);
        for s_idx in 0..setup_count {
            let is_active_setup = *active_setup_idx == s_idx;
            let s_key = TreeNodeKey::Setup(s_idx);
            let s_id = node_id_for(tree_scope, &s_key);
            let setup_name = project.setups[s_idx].name.clone();
            let wcs_code = project.setups[s_idx].wcs_slot.short_code();
            let fixture_selected_here = *focus == InspectorFocus::Setup { setup_idx: s_idx }
                && *setup_tab == SetupTab::Fixtures
                && selected_fixture_idx.is_some();
            let is_setup_sel =
                *focus == InspectorFocus::Setup { setup_idx: s_idx } && !fixture_selected_here;
            let is_setup_dragged = active_drag
                .is_some_and(|d| d.section == DragSection::Setups && d.item_idx == s_idx);

            let s_is_open;
            if rename_state.as_ref().is_some_and(|r| r.key == s_key) {
                let r_rect = ui.available_rect_before_wrap();
                if let Some(commit) =
                    render_inline_rename_row(ui, 2, IconKind::Setup, tok.accent, rename_state)
                {
                    pending.inline_rename_commit = Some(commit);
                }
                s_is_open = is_node_open(ui, s_id, is_active_setup);
                setup_row_rects.push(Rect::from_min_size(
                    r_rect.min,
                    egui::vec2(r_rect.width(), tree_widget::TREE_ROW_HEIGHT),
                ));
                visible_nodes.push(VisibleTreeNode {
                    key: s_key.clone(),
                    node_id: s_id,
                    parent: Some(setups_key.clone()),
                    has_children: true,
                    is_open: s_is_open,
                });
            } else {
                let setup_tip = if is_active_setup {
                    format!(
                        "{setup_name} [{wcs_code}] (Active Setup - new operations are added here)"
                    )
                } else {
                    format!("Click to activate and inspect {setup_name} [{wcs_code}]")
                };

                let s_out = tree_row(
                    ui,
                    TreeRowSpec {
                        node_id: s_id,
                        depth: 2,
                        icon: IconKind::Setup,
                        icon_color: if is_active_setup || is_setup_sel || is_setup_dragged {
                            tok.accent
                        } else {
                            tok.text_secondary
                        },
                        label: &setup_name,
                        label_color: tok.text_primary,
                        bold: is_active_setup,
                        strikethrough: false,
                        italic: false,
                        has_children: true,
                        default_open: is_active_setup,
                        selected: is_setup_sel || is_setup_dragged,
                        keyboard_focused: tree_kbd_active
                            && kbd_focused_key.as_ref() == Some(&s_key),
                        draggable: setup_count > 1,
                        tooltip: Some(&setup_tip),
                    },
                    |ui, hovered| {
                        if hovered && active_drag.is_none() {
                            if setup_count > 1
                                && small_icon_button(ui, IconKind::Trash, "Delete Setup").clicked()
                            {
                                pending.setup_delete = Some(s_idx);
                            }
                            if small_icon_button(
                                ui,
                                IconKind::TopView,
                                "Align camera down this Setup's +Z spindle axis",
                            )
                            .clicked()
                            {
                                *active_setup_idx = s_idx;
                                *emitted_action = Some(BrowserAction::AlignCameraToActiveSetup);
                            }
                            if small_icon_button(
                                ui,
                                IconKind::Generate,
                                "Generate dirty toolpaths in this Setup (Ctrl+G)",
                            )
                            .clicked()
                            {
                                *active_setup_idx = s_idx;
                                *emitted_action = Some(BrowserAction::GenerateSetup(s_idx));
                            }
                        } else {
                            if is_active_setup {
                                tree_badge_pill(
                                    ui,
                                    "Active",
                                    tok.accent,
                                    None,
                                    tok.accent_on_fg,
                                    Some("Active Setup"),
                                );
                            }
                            tree_badge_pill(
                                ui,
                                wcs_code,
                                tok.elevated_bg,
                                Some(tok.border),
                                tok.text_muted,
                                Some("Work Coordinate Offset"),
                            );
                        }
                    },
                );
                s_is_open = s_out.is_open;
                setup_row_rects.push(s_out.row_rect);
                visible_nodes.push(VisibleTreeNode {
                    key: s_key.clone(),
                    node_id: s_id,
                    parent: Some(setups_key.clone()),
                    has_children: true,
                    is_open: s_is_open,
                });
                if s_out.response.clicked() {
                    *active_setup_idx = s_idx;
                    *focus = InspectorFocus::Setup { setup_idx: s_idx };
                    if *setup_tab == SetupTab::Fixtures {
                        *selected_fixture_idx = None;
                    }
                    *pick_mode = ViewportPickMode::InspectEntity;
                    *kbd_focused_key = Some(s_key.clone());
                    *any_row_clicked = true;
                } else if s_out.chevron_toggled {
                    *kbd_focused_key = Some(s_key.clone());
                    *any_row_clicked = true;
                }
                if s_out.response.drag_started() && setup_count > 1 {
                    let st = DraggedTreeItem {
                        section: DragSection::Setups,
                        item_idx: s_idx,
                    };
                    *active_drag = Some(st);
                    ui.ctx().data_mut(|d| d.insert_temp(drag_state_id, st));
                }
                s_out.response.context_menu(|ui| {
                    show_setup_item_context_menu(
                        ui,
                        s_idx,
                        setup_count,
                        &setup_name,
                        active_setup_idx,
                        focus,
                        pick_mode,
                        pending,
                        rename_state,
                        emitted_action,
                    );
                });
            }

            if !s_is_open {
                continue;
            }

            // 5a. Setup -> Fixtures (N)
            let fix_count = project.setups[s_idx].fixtures.len();
            let fix_folder_key = TreeNodeKey::FixturesFolder { setup_idx: s_idx };
            let fix_folder_id = node_id_for(tree_scope, &fix_folder_key);
            let fix_folder_label = format!("Fixtures ({fix_count})");
            let fix_folder_out = tree_row(
                ui,
                TreeRowSpec {
                    node_id: fix_folder_id,
                    depth: 3,
                    icon: IconKind::Fixture,
                    icon_color: tok.text_secondary,
                    label: &fix_folder_label,
                    label_color: tok.text_secondary,
                    bold: false,
                    strikethrough: false,
                    italic: false,
                    has_children: fix_count > 0,
                    default_open: fix_count > 0,
                    selected: *focus == InspectorFocus::Setup { setup_idx: s_idx }
                        && *setup_tab == SetupTab::Fixtures
                        && selected_fixture_idx.is_none(),
                    keyboard_focused: tree_kbd_active
                        && kbd_focused_key.as_ref() == Some(&fix_folder_key),
                    draggable: false,
                    tooltip: Some("Click to open this Setup's 3D Workholding & Fixtures tab"),
                },
                |_ui, _hovered| {},
            );
            visible_nodes.push(VisibleTreeNode {
                key: fix_folder_key.clone(),
                node_id: fix_folder_id,
                parent: Some(s_key.clone()),
                has_children: fix_count > 0,
                is_open: fix_folder_out.is_open,
            });
            if fix_folder_out.response.clicked() {
                *active_setup_idx = s_idx;
                *focus = InspectorFocus::Setup { setup_idx: s_idx };
                *setup_tab = SetupTab::Fixtures;
                *selected_fixture_idx = None;
                *pick_mode = ViewportPickMode::InspectEntity;
                *kbd_focused_key = Some(fix_folder_key.clone());
                *any_row_clicked = true;
            } else if fix_folder_out.chevron_toggled {
                *kbd_focused_key = Some(fix_folder_key.clone());
                *any_row_clicked = true;
            }
            fix_folder_out.response.context_menu(|ui| {
                show_fixtures_folder_context_menu(ui, s_idx, pending, emitted_action);
            });

            if fix_folder_out.is_open {
                let mut fix_row_rects: Vec<Rect> = Vec::with_capacity(fix_count);
                for (f_idx, fix) in project.setups[s_idx].fixtures.iter().enumerate() {
                    let f_key = TreeNodeKey::Fixture {
                        setup_idx: s_idx,
                        fixture_idx: f_idx,
                    };
                    let f_id = node_id_for(tree_scope, &f_key);
                    let is_f_sel = *focus == InspectorFocus::Setup { setup_idx: s_idx }
                        && *setup_tab == SetupTab::Fixtures
                        && *selected_fixture_idx == Some(f_idx);
                    let is_f_dragged = active_drag.is_some_and(|d| {
                        d.section == DragSection::Fixtures { setup_idx: s_idx }
                            && d.item_idx == f_idx
                    });
                    let f_enabled = fix.enabled;
                    let f_name = fix.name.clone();

                    if rename_state.as_ref().is_some_and(|r| r.key == f_key) {
                        let r_rect = ui.available_rect_before_wrap();
                        if let Some(commit) = render_inline_rename_row(
                            ui,
                            4,
                            IconKind::Fixture,
                            tok.accent,
                            rename_state,
                        ) {
                            pending.inline_rename_commit = Some(commit);
                        }
                        fix_row_rects.push(Rect::from_min_size(
                            r_rect.min,
                            egui::vec2(r_rect.width(), tree_widget::TREE_ROW_HEIGHT),
                        ));
                        visible_nodes.push(VisibleTreeNode {
                            key: f_key,
                            node_id: f_id,
                            parent: Some(fix_folder_key.clone()),
                            has_children: false,
                            is_open: false,
                        });
                        continue;
                    }

                    let f_out = tree_row(
                        ui,
                        TreeRowSpec {
                            node_id: f_id,
                            depth: 4,
                            icon: IconKind::Fixture,
                            icon_color: if !f_enabled {
                                tok.suppressed_fg
                            } else if is_f_sel || is_f_dragged {
                                tok.accent
                            } else {
                                tok.text_secondary
                            },
                            label: &f_name,
                            label_color: if f_enabled {
                                tok.text_primary
                            } else {
                                tok.suppressed_fg
                            },
                            bold: false,
                            strikethrough: !f_enabled,
                            italic: false,
                            has_children: false,
                            default_open: false,
                            selected: is_f_sel || is_f_dragged,
                            keyboard_focused: tree_kbd_active
                                && kbd_focused_key.as_ref() == Some(&f_key),
                            draggable: fix_count > 1,
                            tooltip: Some("Click to edit this workholding fixture"),
                        },
                        |ui, _hovered| {
                            let mut enabled_box = f_enabled;
                            if ui
                                .checkbox(&mut enabled_box, "")
                                .on_hover_text(if f_enabled {
                                    "Active fixture - uncheck to suppress from collision checks & rapid avoidance"
                                } else {
                                    "Suppressed fixture - check to re-enable"
                                })
                                .changed()
                            {
                                pending.fixture_toggle_enabled = Some((s_idx, f_idx));
                            }
                        },
                    );
                    fix_row_rects.push(f_out.row_rect);
                    visible_nodes.push(VisibleTreeNode {
                        key: f_key.clone(),
                        node_id: f_id,
                        parent: Some(fix_folder_key.clone()),
                        has_children: false,
                        is_open: false,
                    });
                    if f_out.response.clicked() {
                        *active_setup_idx = s_idx;
                        *focus = InspectorFocus::Setup { setup_idx: s_idx };
                        *setup_tab = SetupTab::Fixtures;
                        *selected_fixture_idx = Some(f_idx);
                        *pick_mode = ViewportPickMode::InspectEntity;
                        *kbd_focused_key = Some(f_key);
                        *any_row_clicked = true;
                    }
                    if f_out.response.drag_started() && fix_count > 1 {
                        let st = DraggedTreeItem {
                            section: DragSection::Fixtures { setup_idx: s_idx },
                            item_idx: f_idx,
                        };
                        *active_drag = Some(st);
                        ui.ctx().data_mut(|d| d.insert_temp(drag_state_id, st));
                    }
                    f_out.response.context_menu(|ui| {
                        show_fixture_item_context_menu(
                            ui,
                            s_idx,
                            f_idx,
                            fix_count,
                            f_enabled,
                            &f_name,
                            pending,
                            rename_state,
                        );
                    });
                }

                if let Some((from_i, to_slot)) = handle_section_drag_overlay(
                    ui,
                    active_drag,
                    DragSection::Fixtures { setup_idx: s_idx },
                    4,
                    &fix_row_rects,
                    drag_state_id,
                ) {
                    pending.fixture_move_slot = Some((s_idx, from_i, to_slot));
                }
            }

            // 5b. Setup -> Operations (N)
            let op_len = project.setups[s_idx].operations.len();
            let ops_folder_key = TreeNodeKey::OperationsFolder { setup_idx: s_idx };
            let ops_folder_id = node_id_for(tree_scope, &ops_folder_key);
            let ops_folder_label = format!("Operations ({op_len})");
            let ops_folder_out = tree_row(
                ui,
                TreeRowSpec {
                    node_id: ops_folder_id,
                    depth: 3,
                    icon: IconKind::OperationsFolder,
                    icon_color: tok.text_secondary,
                    label: &ops_folder_label,
                    label_color: tok.text_secondary,
                    bold: false,
                    strikethrough: false,
                    italic: false,
                    has_children: op_len > 0,
                    default_open: true,
                    selected: false,
                    keyboard_focused: tree_kbd_active
                        && kbd_focused_key.as_ref() == Some(&ops_folder_key),
                    draggable: false,
                    tooltip: Some("Ordered machining operations for this Setup"),
                },
                |ui, _hovered| {
                    tree_folder_menu_button(
                        ui,
                        ("tree_ops_add_menu", s_idx),
                        "Add a CAM Operation to this Setup",
                        |ui| {
                            show_add_operation_categories(ui, s_idx, pending);
                        },
                    );
                },
            );
            visible_nodes.push(VisibleTreeNode {
                key: ops_folder_key.clone(),
                node_id: ops_folder_id,
                parent: Some(s_key.clone()),
                has_children: op_len > 0,
                is_open: ops_folder_out.is_open,
            });
            if ops_folder_out.response.clicked() {
                *active_setup_idx = s_idx;
                *kbd_focused_key = Some(ops_folder_key.clone());
                *any_row_clicked = true;
            } else if ops_folder_out.chevron_toggled {
                *kbd_focused_key = Some(ops_folder_key.clone());
                *any_row_clicked = true;
            }
            ops_folder_out.response.context_menu(|ui| {
                show_operations_folder_context_menu(
                    ui,
                    s_idx,
                    active_setup_idx,
                    pending,
                    emitted_action,
                );
            });

            if ops_folder_out.is_open {
                let mut row_rects: Vec<Rect> = Vec::with_capacity(op_len);
                for o_idx in 0..op_len {
                    let is_sel = *focus
                        == InspectorFocus::Operation {
                            setup_idx: s_idx,
                            op_idx: o_idx,
                        };
                    let cached_tp =
                        toolpath_cache.get_for_operation(project, s_idx, o_idx, cad_revision);
                    let op = &project.setups[s_idx].operations[o_idx];
                    let op_key = TreeNodeKey::Operation {
                        setup_idx: s_idx,
                        op_idx: o_idx,
                    };
                    let op_node_id = node_id_for(tree_scope, &op_key);
                    let op_id_clone = op.id.clone();
                    let op_name_clone = op.name.clone();
                    let op_kind_val = op.kind;
                    let op_is_adap = op.is_adaptive_2d();

                    if rename_state.as_ref().is_some_and(|r| r.key == op_key) {
                        let r_rect = ui.available_rect_before_wrap();
                        if let Some(commit) = render_inline_rename_row(
                            ui,
                            4,
                            IconKind::for_operation(op_kind_val, op_is_adap),
                            tok.accent,
                            rename_state,
                        ) {
                            pending.inline_rename_commit = Some(commit);
                        }
                        row_rects.push(Rect::from_min_size(
                            r_rect.min,
                            egui::vec2(r_rect.width(), tree_widget::TREE_ROW_HEIGHT),
                        ));
                        visible_nodes.push(VisibleTreeNode {
                            key: op_key,
                            node_id: op_node_id,
                            parent: Some(ops_folder_key.clone()),
                            has_children: false,
                            is_open: false,
                        });
                        continue;
                    }

                    let est_sec = cached_tp.as_ref().map(|(_, tp)| {
                        tp.stats.total_est_time_sec
                            + tp.clearing_stage
                                .as_ref()
                                .map_or(0.0, |c| c.toolpath.stats.total_est_time_sec)
                    });

                    let (status_kind, mut hover_note) = if op.suppressed {
                        (
                            StatusBadgeKind::Suppressed,
                            Some("Suppressed (unchecked)".to_string()),
                        )
                    } else {
                        match &op.runtime_status {
                            OperationStatus::Clean => (
                                StatusBadgeKind::Clean,
                                est_sec.map(|sec| {
                                    let passes = cached_tp
                                        .as_ref()
                                        .map_or(0, |(_, tp)| tp.stats.pass_count);
                                    format!(
                                        "Clean: {passes} pass(es), est. {} (drag or Ctrl+Up/Down to reorder)",
                                        format_cycle_time(sec)
                                    )
                                }),
                            ),
                            OperationStatus::Dirty => (
                                StatusBadgeKind::Dirty,
                                Some("Dirty - press Ctrl+G to regenerate".to_string()),
                            ),
                            OperationStatus::Generating => (
                                StatusBadgeKind::Dirty,
                                Some("Generating on background thread...".to_string()),
                            ),
                            OperationStatus::Error { message } => {
                                (StatusBadgeKind::Error, Some(message.clone()))
                            }
                            OperationStatus::NeedsRepick { message } => (
                                StatusBadgeKind::NeedsRepick,
                                Some(format!("Needs Re-Pick: {message}")),
                            ),
                        }
                    };

                    if !op.suppressed {
                        let rs_for_op = (s_idx == *active_setup_idx)
                            .then_some(resolved_active_setup)
                            .flatten();
                        if let Some(rep) = evaluate_operation_machinability(
                            project,
                            s_idx,
                            o_idx,
                            global_tools,
                            active_machine,
                            rs_for_op,
                            cad_doc,
                        ) && let Some(warn_line) = compact_browser_warning_summary(&rep)
                        {
                            hover_note = match hover_note {
                                Some(existing) => Some(format!("{existing}\n{warn_line}")),
                                None => Some(warn_line),
                            };
                        }
                    }

                    let badge_txt = if op.suppressed {
                        None
                    } else {
                        format_operation_badge(project, op, est_sec)
                    };

                    let mode_tag = match (op.is_adaptive_2d(), op.has_rest_machining()) {
                        (true, true) => " | Adp+Rest",
                        (true, false) => "",
                        (false, true) => " | Rest",
                        (false, false) => "",
                    };
                    let label_txt = format!("{}. {}{mode_tag}", o_idx + 1, op.name);
                    let is_sup = op.suppressed;
                    let is_hidden_in_vp = hidden_operation_ids.contains(&op_id_clone);
                    let is_needs_repick =
                        !is_sup && matches!(op.runtime_status, OperationStatus::NeedsRepick { .. });
                    let is_gen =
                        !is_sup && matches!(op.runtime_status, OperationStatus::Generating);
                    let is_being_dragged = active_drag.is_some_and(|d| {
                        d.section == DragSection::Operations { setup_idx: s_idx }
                            && d.item_idx == o_idx
                    });

                    let op_out = tree_row(
                        ui,
                        TreeRowSpec {
                            node_id: op_node_id,
                            depth: 4,
                            icon: IconKind::for_operation(op_kind_val, op_is_adap),
                            icon_color: if is_sup {
                                tok.suppressed_fg
                            } else if is_sel || is_being_dragged {
                                tok.accent
                            } else {
                                tok.text_secondary
                            },
                            label: &label_txt,
                            label_color: if is_sup {
                                tok.suppressed_fg
                            } else {
                                tok.text_primary
                            },
                            bold: false,
                            strikethrough: is_sup,
                            italic: false,
                            has_children: false,
                            default_open: false,
                            selected: is_sel || is_being_dragged,
                            keyboard_focused: tree_kbd_active
                                && kbd_focused_key.as_ref() == Some(&op_key),
                            draggable: op_len > 1,
                            tooltip: hover_note.as_deref(),
                        },
                        |ui, hovered| {
                            let show_hover_tools = hovered && active_drag.is_none();
                            let mut active = !is_sup;
                            if ui
                                .checkbox(&mut active, "")
                                .on_hover_text(if is_sup {
                                    "Suppressed - check to re-enable this operation for simulation & G-code"
                                } else {
                                    "Active - uncheck to suppress this operation from simulation & G-code"
                                })
                                .changed()
                            {
                                pending.op_toggle_suppress = Some((s_idx, o_idx));
                            }

                            if show_hover_tools {
                                if small_icon_button(ui, IconKind::Trash, "Delete Operation (Del)")
                                    .clicked()
                                {
                                    pending.op_delete = Some((s_idx, o_idx));
                                }
                                if small_icon_button(
                                    ui,
                                    IconKind::Duplicate,
                                    "Duplicate Operation (Ctrl+D)",
                                )
                                .clicked()
                                {
                                    pending.op_duplicate = Some((s_idx, o_idx));
                                }
                            }

                            if (show_hover_tools || is_hidden_in_vp)
                                && small_icon_button(
                                    ui,
                                    if is_hidden_in_vp {
                                        IconKind::EyeOff
                                    } else {
                                        IconKind::Eye
                                    },
                                    if is_hidden_in_vp {
                                        "Show operation toolpath in 3D viewport"
                                    } else {
                                        "Hide operation toolpath in 3D viewport (keeps operation active)"
                                    },
                                )
                                .clicked()
                            {
                                pending.op_toggle_visibility = Some(op_id_clone.clone());
                            }

                            if !show_hover_tools && let Some(badge) = &badge_txt {
                                tree_badge_pill(
                                    ui,
                                    badge,
                                    tok.elevated_bg,
                                    Some(tok.border),
                                    tok.text_muted,
                                    None,
                                );
                            }

                            if is_gen {
                                ui.add(egui::Spinner::new().size(13.0));
                            } else {
                                status_badge_icon(ui, status_kind, hover_note.as_deref());
                            }
                        },
                    );

                    visible_nodes.push(VisibleTreeNode {
                        key: op_key.clone(),
                        node_id: op_node_id,
                        parent: Some(ops_folder_key.clone()),
                        has_children: false,
                        is_open: false,
                    });

                    if op_out.response.clicked() {
                        *active_setup_idx = s_idx;
                        *focus = InspectorFocus::Operation {
                            setup_idx: s_idx,
                            op_idx: o_idx,
                        };
                        if is_needs_repick {
                            *op_tab = OperationTab::Geometry;
                        }
                        *pick_mode = default_pick_mode_for_op(s_idx, o_idx, op_kind_val, *op_tab);
                        *kbd_focused_key = Some(op_key);
                        *any_row_clicked = true;
                    }
                    if op_out.response.drag_started() && op_len > 1 {
                        let st = DraggedTreeItem {
                            section: DragSection::Operations { setup_idx: s_idx },
                            item_idx: o_idx,
                        };
                        *active_drag = Some(st);
                        ui.ctx().data_mut(|d| d.insert_temp(drag_state_id, st));
                    }

                    op_out.response.context_menu(|ui| {
                        show_operation_item_context_menu(
                            ui,
                            s_idx,
                            o_idx,
                            op_len,
                            &op_id_clone,
                            &op_name_clone,
                            is_sup,
                            is_hidden_in_vp,
                            pending,
                            rename_state,
                            emitted_action,
                        );
                    });

                    row_rects.push(op_out.row_rect);
                }

                if let Some((from_i, to_slot)) = handle_section_drag_overlay(
                    ui,
                    active_drag,
                    DragSection::Operations { setup_idx: s_idx },
                    4,
                    &row_rects,
                    drag_state_id,
                ) {
                    pending.op_move_slot = Some((s_idx, from_i, to_slot));
                }
            }
        }

        if let Some((from_i, to_slot)) = handle_section_drag_overlay(
            ui,
            active_drag,
            DragSection::Setups,
            2,
            &setup_row_rects,
            drag_state_id,
        ) {
            pending.setup_move_slot = Some((from_i, to_slot));
        }
    }
}
