use crate::{
    panels::{InspectorFocus, ViewportPickMode},
    widgets::{IconKind, small_icon_button},
};

use super::{
    CamBrowserCtx, TreeNodeKey, TreeRenderState, VisibleTreeNode,
    context_menus::{show_tool_item_context_menu, show_tools_folder_context_menu},
    node_id_for,
    tree_widget::{TreeRowSpec, tree_badge_pill, tree_folder_menu_button, tree_hint_row, tree_row},
};

pub(super) fn show_tools_section(
    ui: &mut egui::Ui,
    ctx: &mut CamBrowserCtx<'_>,
    state: &mut TreeRenderState,
) {
    let CamBrowserCtx {
        project,
        global_tools,
        focus,
        pick_mode,
        ..
    } = ctx;
    let global_tools = *global_tools;
    let focus: &mut InspectorFocus = focus;
    let pick_mode: &mut ViewportPickMode = pick_mode;
    let TreeRenderState {
        tree_scope,
        tok,
        root_key,
        kbd_focused_key,
        tree_kbd_active,
        pending,
        visible_nodes,
        any_row_clicked,
        emitted_action,
        ..
    } = state;
    let tree_scope = *tree_scope;
    let tok = *tok;
    let tree_kbd_active = *tree_kbd_active;

    // -----------------------------------------------------------------
    // 4. Project Tools (N)  [Never drag-reorderable]
    // -----------------------------------------------------------------
    let usage_map = project.tool_usage_map();
    let unused_count = project.unused_tool_ids().len();
    let tools_key = TreeNodeKey::ToolsFolder;
    let tools_id = node_id_for(tree_scope, &tools_key);
    let tools_label = format!("Project Tools ({})", project.tools.len());
    let tools_out = tree_row(
        ui,
        TreeRowSpec {
            node_id: tools_id,
            depth: 1,
            icon: IconKind::Tool,
            icon_color: tok.text_secondary,
            label: &tools_label,
            label_color: tok.text_primary,
            bold: true,
            strikethrough: false,
            italic: false,
            has_children: true,
            default_open: false,
            selected: false,
            keyboard_focused: tree_kbd_active && kbd_focused_key.as_ref() == Some(&tools_key),
            draggable: false,
            tooltip: Some("Cutting tools embedded in this project"),
        },
        |ui, _hovered| {
            tree_folder_menu_button(
                ui,
                "tree_tools_add_menu",
                "Add a tool from the active machine or prune unused tools",
                |ui| {
                    let filter_id = ui.make_persistent_id("browser_add_tool_filter");
                    let mut filter_text = ui
                        .data_mut(|d| d.get_temp::<String>(filter_id))
                        .unwrap_or_default();
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("Filter:").small());
                        ui.add(
                            egui::TextEdit::singleline(&mut filter_text)
                                .desired_width(170.0)
                                .hint_text("diameter, flat, ball, v-bit..."),
                        );
                    });
                    ui.data_mut(|d| d.insert_temp(filter_id, filter_text.clone()));
                    ui.separator();

                    let needle = filter_text.trim().to_ascii_lowercase();
                    egui::ScrollArea::vertical()
                        .id_salt("browser_add_tool_list")
                        .max_height(220.0)
                        .show(ui, |ui| {
                            let mut shown_any = false;
                            for gt in &global_tools.tools {
                                let lbl = gt.summary_label();
                                if !needle.is_empty()
                                    && !lbl.to_ascii_lowercase().contains(&needle)
                                {
                                    continue;
                                }
                                shown_any = true;
                                let already_in = project.tool(&gt.id).is_some();
                                let row_lbl = if already_in {
                                    format!("{lbl} (in project)")
                                } else {
                                    lbl
                                };
                                if ui
                                    .add_enabled(!already_in, egui::Button::new(row_lbl))
                                    .on_hover_text(
                                        "Embed this cutter from the active machine into Project Tools",
                                    )
                                    .on_disabled_hover_text(
                                        "Already embedded in Project Tools",
                                    )
                                    .clicked()
                                {
                                    pending.add_tool = Some(gt.clone());
                                    ui.close();
                                }
                            }
                            if !shown_any {
                                ui.weak("No matching tools in active machine library.");
                            }
                        });

                    if unused_count > 0 {
                        ui.separator();
                        if ui
                            .button(format!("Remove Unused Tools ({unused_count})"))
                            .clicked()
                        {
                            pending.prune_unused_tools = true;
                            ui.close();
                        }
                    }
                },
            );
            if unused_count > 0 {
                tree_badge_pill(
                    ui,
                    &format!("{unused_count} unused"),
                    tok.warn_bg,
                    Some(tok.warn_border),
                    tok.warn_fg,
                    Some("Unused tools can be removed from the + menu or right-click menu"),
                );
            }
        },
    );
    visible_nodes.push(VisibleTreeNode {
        key: tools_key.clone(),
        node_id: tools_id,
        parent: Some(root_key.clone()),
        has_children: true,
        is_open: tools_out.is_open,
    });
    if tools_out.response.clicked() || tools_out.chevron_toggled {
        *kbd_focused_key = Some(tools_key.clone());
        *any_row_clicked = true;
    }
    tools_out.response.context_menu(|ui| {
        show_tools_folder_context_menu(ui, unused_count, pending, emitted_action);
    });

    if tools_out.is_open {
        if project.tools.is_empty() {
            tree_hint_row(ui, 2, "No project tools embedded yet");
        } else {
            for (t_idx, tool) in project.tools.iter().enumerate() {
                let usages = usage_map.get(&tool.id);
                let use_count = usages.map_or(0, Vec::len);
                let t_key = TreeNodeKey::Tool(t_idx);
                let t_id = node_id_for(tree_scope, &t_key);
                let is_t_sel = *focus == InspectorFocus::ProjectTool { tool_idx: t_idx };
                let main_txt = format!("T{} - {}", tool.tool_number, tool.name);
                let tip = if use_count == 0 {
                    format!("{} (unused)", tool.summary_label())
                } else {
                    let names: Vec<&str> = usages
                        .into_iter()
                        .flatten()
                        .map(|u| u.op_name.as_str())
                        .collect();
                    format!("{} | used by {}", tool.summary_label(), names.join(", "))
                };
                let in_machine_lib = global_tools.tool(&tool.id).is_some();
                let t_out = tree_row(
                    ui,
                    TreeRowSpec {
                        node_id: t_id,
                        depth: 2,
                        icon: IconKind::Tool,
                        icon_color: if is_t_sel {
                            tok.accent
                        } else {
                            tok.text_secondary
                        },
                        label: &main_txt,
                        label_color: tok.text_primary,
                        bold: false,
                        strikethrough: false,
                        italic: false,
                        has_children: false,
                        default_open: false,
                        selected: is_t_sel,
                        keyboard_focused: tree_kbd_active
                            && kbd_focused_key.as_ref() == Some(&t_key),
                        draggable: false,
                        tooltip: Some(&tip),
                    },
                    |ui, hovered| {
                        if hovered && use_count == 0 {
                            if small_icon_button(
                                ui,
                                IconKind::Trash,
                                &format!("Remove unused tool T{} from project", tool.tool_number),
                            )
                            .clicked()
                            {
                                pending.remove_tool = Some(tool.id.clone());
                            }
                        } else {
                            let (badge_txt, bg, border, fg) = if use_count == 0 {
                                (
                                    "unused".to_string(),
                                    tok.warn_bg,
                                    Some(tok.warn_border),
                                    tok.warn_fg,
                                )
                            } else if use_count == 1 {
                                (
                                    "1 op".to_string(),
                                    tok.elevated_bg,
                                    Some(tok.border),
                                    tok.text_muted,
                                )
                            } else {
                                (
                                    format!("{use_count} ops"),
                                    tok.elevated_bg,
                                    Some(tok.border),
                                    tok.text_muted,
                                )
                            };
                            tree_badge_pill(ui, &badge_txt, bg, border, fg, None);
                        }
                    },
                );
                visible_nodes.push(VisibleTreeNode {
                    key: t_key.clone(),
                    node_id: t_id,
                    parent: Some(tools_key.clone()),
                    has_children: false,
                    is_open: false,
                });
                if t_out.response.clicked() {
                    *focus = InspectorFocus::ProjectTool { tool_idx: t_idx };
                    *pick_mode = ViewportPickMode::InspectEntity;
                    *kbd_focused_key = Some(t_key);
                    *any_row_clicked = true;
                }
                t_out.response.context_menu(|ui| {
                    show_tool_item_context_menu(
                        ui,
                        &tool.id,
                        in_machine_lib,
                        use_count,
                        pending,
                        emitted_action,
                    );
                });
            }
        }
    }
}
