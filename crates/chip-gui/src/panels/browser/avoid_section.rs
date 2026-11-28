use egui::Rect;

use crate::{
    panels::{InspectorFocus, ViewportPickMode},
    widgets::{IconKind, small_icon_button},
};

use super::{
    CamBrowserCtx, DragSection, DraggedTreeItem, TreeNodeKey, TreeRenderState, VisibleTreeNode,
    context_menus::{
        render_inline_rename_row, show_avoid_folder_context_menu, show_avoid_item_context_menu,
    },
    drag_reorder::handle_section_drag_overlay,
    node_id_for,
    tree_widget::{self, TreeRowSpec, tree_badge_pill, tree_hint_row, tree_row},
};

pub(super) fn show_avoid_section(
    ui: &mut egui::Ui,
    ctx: &mut CamBrowserCtx<'_>,
    state: &mut TreeRenderState,
) {
    let CamBrowserCtx {
        project,
        focus,
        pick_mode,
        editing_shared_avoid_id,
        ..
    } = ctx;
    let focus: &mut InspectorFocus = focus;
    let pick_mode: &mut ViewportPickMode = pick_mode;
    let editing_shared_avoid_id: &mut Option<String> = editing_shared_avoid_id;
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
        ..
    } = state;
    let tree_scope = *tree_scope;
    let drag_state_id = *drag_state_id;
    let tok = *tok;
    let tree_kbd_active = *tree_kbd_active;

    // -----------------------------------------------------------------
    // 3. Keep-Out Regions (N)
    // -----------------------------------------------------------------
    let avoid_key = TreeNodeKey::AvoidRegionsFolder;
    let avoid_id = node_id_for(tree_scope, &avoid_key);
    let avoid_label = format!("Keep-Out Regions ({})", project.avoid_regions.len());
    let avoid_out = tree_row(
        ui,
        TreeRowSpec {
            node_id: avoid_id,
            depth: 1,
            icon: IconKind::KeepOutZone,
            icon_color: tok.text_secondary,
            label: &avoid_label,
            label_color: tok.text_primary,
            bold: true,
            strikethrough: false,
            italic: false,
            has_children: true,
            default_open: !project.avoid_regions.is_empty(),
            selected: false,
            keyboard_focused: tree_kbd_active && kbd_focused_key.as_ref() == Some(&avoid_key),
            draggable: false,
            tooltip: Some("Project-level reusable Shared Avoid / Keep-Out regions"),
        },
        |ui, _hovered| {
            if small_icon_button(
                ui,
                IconKind::Plus,
                "Create a new project-level Shared Keep-Out region",
            )
            .clicked()
            {
                pending.avoid_add = true;
            }
        },
    );
    visible_nodes.push(VisibleTreeNode {
        key: avoid_key.clone(),
        node_id: avoid_id,
        parent: Some(root_key.clone()),
        has_children: true,
        is_open: avoid_out.is_open,
    });
    if avoid_out.response.clicked() || avoid_out.chevron_toggled {
        *kbd_focused_key = Some(avoid_key.clone());
        *any_row_clicked = true;
    }
    avoid_out.response.context_menu(|ui| {
        show_avoid_folder_context_menu(ui, pending);
    });

    if avoid_out.is_open {
        let avoid_len = project.avoid_regions.len();
        if avoid_len == 0 {
            tree_hint_row(ui, 2, "No shared keep-out regions yet");
        } else {
            let mut avoid_row_rects: Vec<Rect> = Vec::with_capacity(avoid_len);
            for r_idx in 0..avoid_len {
                let r = &project.avoid_regions[r_idx];
                let rid = r.id.clone();
                let rname = r.name.clone();
                let is_sup = r.suppressed;
                let is_selected = *focus == InspectorFocus::KeepOutRegion { region_idx: r_idx }
                    || editing_shared_avoid_id.as_deref() == Some(rid.as_str());
                let is_dragged = active_drag
                    .is_some_and(|d| d.section == DragSection::AvoidRegions && d.item_idx == r_idx);
                let usage = project.avoid_region_usage_count(&rid);
                let usage_txt = if usage == 1 {
                    "used by 1 op".to_string()
                } else {
                    format!("used by {usage} ops")
                };
                let r_key = TreeNodeKey::AvoidRegion(r_idx);
                let r_id = node_id_for(tree_scope, &r_key);

                if rename_state.as_ref().is_some_and(|st| st.key == r_key) {
                    let r_rect = ui.available_rect_before_wrap();
                    if let Some(commit) = render_inline_rename_row(
                        ui,
                        2,
                        IconKind::KeepOutZone,
                        tok.warn_fg,
                        rename_state,
                    ) {
                        pending.inline_rename_commit = Some(commit);
                    }
                    avoid_row_rects.push(Rect::from_min_size(
                        r_rect.min,
                        egui::vec2(r_rect.width(), tree_widget::TREE_ROW_HEIGHT),
                    ));
                    visible_nodes.push(VisibleTreeNode {
                        key: r_key,
                        node_id: r_id,
                        parent: Some(avoid_key.clone()),
                        has_children: false,
                        is_open: false,
                    });
                    continue;
                }

                let item_count = r.selections.len() + r.fixture_ids.len();
                let tip = format!(
                    "{rname} ({item_count} item(s)) | {usage_txt} (Click to edit, F2 to rename)"
                );
                let r_out = tree_row(
                    ui,
                    TreeRowSpec {
                        node_id: r_id,
                        depth: 2,
                        icon: IconKind::KeepOutZone,
                        icon_color: if is_sup {
                            tok.suppressed_fg
                        } else if is_selected || is_dragged {
                            tok.error_fg
                        } else {
                            tok.warn_fg
                        },
                        label: &rname,
                        label_color: if is_sup {
                            tok.suppressed_fg
                        } else if is_selected {
                            tok.error_fg
                        } else {
                            tok.text_primary
                        },
                        bold: is_selected,
                        strikethrough: is_sup,
                        italic: false,
                        has_children: false,
                        default_open: false,
                        selected: is_selected || is_dragged,
                        keyboard_focused: tree_kbd_active
                            && kbd_focused_key.as_ref() == Some(&r_key),
                        draggable: avoid_len > 1,
                        tooltip: Some(&tip),
                    },
                    |ui, hovered| {
                        let mut active = !is_sup;
                        if ui
                            .checkbox(&mut active, "")
                            .on_hover_text(if is_sup {
                                "Suppressed - check to re-enable this keep-out region"
                            } else {
                                "Active - uncheck to suppress this keep-out region"
                            })
                            .changed()
                        {
                            pending.avoid_toggle_suppress = Some(r_idx);
                        }
                        if hovered && active_drag.is_none() {
                            if small_icon_button(
                                ui,
                                IconKind::Trash,
                                "Delete shared keep-out region",
                            )
                            .clicked()
                            {
                                pending.avoid_delete = Some(rid.clone());
                            }
                            if small_icon_button(
                                ui,
                                IconKind::Duplicate,
                                "Duplicate shared keep-out region",
                            )
                            .clicked()
                            {
                                pending.avoid_duplicate = Some(r_idx);
                            }
                        } else {
                            tree_badge_pill(
                                ui,
                                &usage_txt,
                                tok.elevated_bg,
                                Some(tok.border),
                                tok.text_muted,
                                None,
                            );
                        }
                    },
                );
                avoid_row_rects.push(r_out.row_rect);
                visible_nodes.push(VisibleTreeNode {
                    key: r_key.clone(),
                    node_id: r_id,
                    parent: Some(avoid_key.clone()),
                    has_children: false,
                    is_open: false,
                });
                if r_out.response.clicked() {
                    *focus = InspectorFocus::KeepOutRegion { region_idx: r_idx };
                    *editing_shared_avoid_id = Some(rid.clone());
                    *pick_mode = ViewportPickMode::InspectEntity;
                    *kbd_focused_key = Some(r_key);
                    *any_row_clicked = true;
                }
                if r_out.response.drag_started() && avoid_len > 1 {
                    let st = DraggedTreeItem {
                        section: DragSection::AvoidRegions,
                        item_idx: r_idx,
                    };
                    *active_drag = Some(st);
                    ui.ctx().data_mut(|d| d.insert_temp(drag_state_id, st));
                }
                r_out.response.context_menu(|ui| {
                    show_avoid_item_context_menu(
                        ui,
                        r_idx,
                        avoid_len,
                        &rid,
                        &rname,
                        is_sup,
                        focus,
                        editing_shared_avoid_id,
                        pick_mode,
                        pending,
                        rename_state,
                    );
                });
            }

            if let Some((from_i, to_slot)) = handle_section_drag_overlay(
                ui,
                active_drag,
                DragSection::AvoidRegions,
                2,
                &avoid_row_rects,
                drag_state_id,
            ) {
                pending.avoid_move_slot = Some((from_i, to_slot));
            }
        }
    }
}
