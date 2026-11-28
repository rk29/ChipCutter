use egui::Rect;

use crate::{
    panels::{InspectorFocus, ViewportPickMode},
    widgets::{IconKind, small_icon_button},
};

use super::{
    BrowserAction, CamBrowserCtx, DragSection, DraggedTreeItem, TreeNodeKey, TreeRenderState,
    VisibleTreeNode,
    context_menus::{
        render_inline_rename_row, show_sketch_entity_context_menu, show_sketch_item_context_menu,
        show_sketches_folder_context_menu,
    },
    drag_reorder::handle_section_drag_overlay,
    node_id_for,
    tree_widget::{
        self, TreeRowSpec, is_node_open, tree_badge_pill, tree_folder_menu_button, tree_hint_row,
        tree_row,
    },
};

pub(super) fn show_sketches_section(
    ui: &mut egui::Ui,
    ctx: &mut CamBrowserCtx<'_>,
    state: &mut TreeRenderState,
) {
    let CamBrowserCtx {
        project,
        cad_doc,
        focus,
        pick_mode,
        selected_sketch_entity_idx,
        ..
    } = ctx;
    let cad_doc = *cad_doc;
    let focus: &mut InspectorFocus = focus;
    let pick_mode: &mut ViewportPickMode = pick_mode;
    let selected_sketch_entity_idx: &mut Option<usize> = selected_sketch_entity_idx;
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
    // 2. Sketches (N) -> <sketch.name> [eye] -> <entity.name>
    // -----------------------------------------------------------------
    let sketches_key = TreeNodeKey::SketchesFolder;
    let sketches_id = node_id_for(tree_scope, &sketches_key);
    let sketches_label = format!("Sketches ({})", project.sketches.len());
    let sketches_out = tree_row(
        ui,
        TreeRowSpec {
            node_id: sketches_id,
            depth: 1,
            icon: IconKind::Sketch2D,
            icon_color: tok.text_secondary,
            label: &sketches_label,
            label_color: tok.text_primary,
            bold: true,
            strikethrough: false,
            italic: false,
            has_children: true,
            default_open: !project.sketches.is_empty(),
            selected: false,
            keyboard_focused: tree_kbd_active && kbd_focused_key.as_ref() == Some(&sketches_key),
            draggable: false,
            tooltip: Some("2D parametric sketches and imported DXF/SVG vector layers"),
        },
        |ui, _hovered| {
            tree_folder_menu_button(
                ui,
                "tree_sketches_add_menu",
                "Create a new 2D Sketch or import DXF/SVG",
                |ui| {
                    if ui.button("New 2D Sketch (Setup XY)").clicked() {
                        *emitted_action = Some(BrowserAction::NewSetupXySketch);
                        ui.close();
                    }
                    if cad_doc.is_some() && ui.button("New 2D Sketch on 3D Face...").clicked() {
                        *emitted_action = Some(BrowserAction::StartPickSketchHostFace);
                        ui.close();
                    }
                    ui.separator();
                    if ui.button("Import DXF / SVG...").clicked() {
                        *emitted_action =
                            Some(BrowserAction::OpenVectorImportModal { browse_file: true });
                        ui.close();
                    }
                },
            );
        },
    );
    visible_nodes.push(VisibleTreeNode {
        key: sketches_key.clone(),
        node_id: sketches_id,
        parent: Some(root_key.clone()),
        has_children: true,
        is_open: sketches_out.is_open,
    });
    if sketches_out.response.clicked() || sketches_out.chevron_toggled {
        *kbd_focused_key = Some(sketches_key.clone());
        *any_row_clicked = true;
    }
    sketches_out.response.context_menu(|ui| {
        show_sketches_folder_context_menu(ui, cad_doc.is_some(), emitted_action);
    });

    if sketches_out.is_open {
        let sk_len = project.sketches.len();
        if sk_len == 0 {
            tree_hint_row(ui, 2, "No 2D sketches yet");
        } else {
            let mut sketch_row_rects: Vec<Rect> = Vec::with_capacity(sk_len);
            for (sk_idx, sk) in project.sketches.iter().enumerate() {
                let sk_key = TreeNodeKey::Sketch(sk_idx);
                let sk_id = node_id_for(tree_scope, &sk_key);
                let has_entities = !sk.entities.is_empty();
                let sk_visible = sk.visible;
                let sk_name = sk.name.clone();
                let is_sketch_focused = *focus == InspectorFocus::Sketch { sketch_idx: sk_idx };
                let sk_open_preview = has_entities && is_node_open(ui, sk_id, false);
                let is_sk_row_selected =
                    is_sketch_focused && (!sk_open_preview || selected_sketch_entity_idx.is_none());
                let is_sk_dragged = active_drag
                    .is_some_and(|d| d.section == DragSection::Sketches && d.item_idx == sk_idx);

                let sk_is_open;
                if rename_state.as_ref().is_some_and(|r| r.key == sk_key) {
                    let r_rect = ui.available_rect_before_wrap();
                    if let Some(commit) = render_inline_rename_row(
                        ui,
                        2,
                        IconKind::Sketch2D,
                        tok.accent,
                        rename_state,
                    ) {
                        pending.inline_rename_commit = Some(commit);
                    }
                    sk_is_open = sk_open_preview;
                    sketch_row_rects.push(Rect::from_min_size(
                        r_rect.min,
                        egui::vec2(r_rect.width(), tree_widget::TREE_ROW_HEIGHT),
                    ));
                    visible_nodes.push(VisibleTreeNode {
                        key: sk_key.clone(),
                        node_id: sk_id,
                        parent: Some(sketches_key.clone()),
                        has_children: has_entities,
                        is_open: sk_is_open,
                    });
                } else {
                    let tip = format!(
                        "{sk_name} ({} region(s), {} entity/entities)",
                        sk.regions.len(),
                        sk.entities.len()
                    );
                    let sk_out = tree_row(
                        ui,
                        TreeRowSpec {
                            node_id: sk_id,
                            depth: 2,
                            icon: IconKind::Sketch2D,
                            icon_color: if !sk_visible {
                                tok.suppressed_fg
                            } else if is_sk_row_selected || is_sk_dragged {
                                tok.accent
                            } else {
                                tok.text_secondary
                            },
                            label: &sk_name,
                            label_color: if sk_visible {
                                tok.text_primary
                            } else {
                                tok.text_muted
                            },
                            bold: false,
                            strikethrough: false,
                            italic: false,
                            has_children: has_entities,
                            default_open: false,
                            selected: is_sk_row_selected || is_sk_dragged,
                            keyboard_focused: tree_kbd_active
                                && kbd_focused_key.as_ref() == Some(&sk_key),
                            draggable: sk_len > 1,
                            tooltip: Some(&tip),
                        },
                        |ui, hovered| {
                            if hovered
                                && active_drag.is_none()
                                && small_icon_button(ui, IconKind::Trash, "Delete Sketch Layer")
                                    .clicked()
                            {
                                pending.sketch_delete = Some(sk_idx);
                            }
                            if small_icon_button(
                                ui,
                                if sk_visible {
                                    IconKind::Eye
                                } else {
                                    IconKind::EyeOff
                                },
                                if sk_visible {
                                    "Hide 2D Sketch in 3D viewport"
                                } else {
                                    "Show 2D Sketch in 3D viewport"
                                },
                            )
                            .clicked()
                            {
                                pending.sketch_vis_toggle = Some(sk_idx);
                            }
                        },
                    );
                    sk_is_open = sk_out.is_open;
                    sketch_row_rects.push(sk_out.row_rect);
                    visible_nodes.push(VisibleTreeNode {
                        key: sk_key.clone(),
                        node_id: sk_id,
                        parent: Some(sketches_key.clone()),
                        has_children: has_entities,
                        is_open: sk_is_open,
                    });
                    if sk_out.response.clicked() {
                        *focus = InspectorFocus::Sketch { sketch_idx: sk_idx };
                        *selected_sketch_entity_idx = None;
                        *pick_mode = ViewportPickMode::InspectEntity;
                        *kbd_focused_key = Some(sk_key.clone());
                        *any_row_clicked = true;
                    } else if sk_out.chevron_toggled {
                        *kbd_focused_key = Some(sk_key.clone());
                        *any_row_clicked = true;
                    }
                    if sk_out.response.drag_started() && sk_len > 1 {
                        let st = DraggedTreeItem {
                            section: DragSection::Sketches,
                            item_idx: sk_idx,
                        };
                        *active_drag = Some(st);
                        ui.ctx().data_mut(|d| d.insert_temp(drag_state_id, st));
                    }
                    sk_out.response.context_menu(|ui| {
                        show_sketch_item_context_menu(
                            ui,
                            sk_idx,
                            sk_len,
                            sk_visible,
                            &sk_name,
                            pending,
                            rename_state,
                            emitted_action,
                        );
                    });
                }

                if sk_is_open {
                    let ent_len = sk.entities.len();
                    let mut entity_row_rects: Vec<Rect> = Vec::with_capacity(ent_len);
                    for (ent_idx, ent) in sk.entities.iter().enumerate() {
                        let ent_key = TreeNodeKey::SketchEntity {
                            sketch_idx: sk_idx,
                            entity_idx: ent_idx,
                        };
                        let ent_id = node_id_for(tree_scope, &ent_key);
                        let is_ent_sel =
                            is_sketch_focused && *selected_sketch_entity_idx == Some(ent_idx);
                        let ent_icon = IconKind::for_sketch_entity(&ent.entity);
                        let ent_name = ent.name.clone();
                        let ent_is_const = ent.is_construction;
                        let is_ent_dragged = active_drag.is_some_and(|d| {
                            d.section == DragSection::SketchEntities { sketch_idx: sk_idx }
                                && d.item_idx == ent_idx
                        });

                        if rename_state.as_ref().is_some_and(|r| r.key == ent_key) {
                            let r_rect = ui.available_rect_before_wrap();
                            if let Some(commit) =
                                render_inline_rename_row(ui, 3, ent_icon, tok.accent, rename_state)
                            {
                                pending.inline_rename_commit = Some(commit);
                            }
                            entity_row_rects.push(Rect::from_min_size(
                                r_rect.min,
                                egui::vec2(r_rect.width(), tree_widget::TREE_ROW_HEIGHT),
                            ));
                            visible_nodes.push(VisibleTreeNode {
                                key: ent_key,
                                node_id: ent_id,
                                parent: Some(sk_key.clone()),
                                has_children: false,
                                is_open: false,
                            });
                            continue;
                        }

                        let ent_tip = format!("{ent_name} ({})", ent.entity.kind_label());
                        let ent_out = tree_row(
                            ui,
                            TreeRowSpec {
                                node_id: ent_id,
                                depth: 3,
                                icon: ent_icon,
                                icon_color: if is_ent_sel || is_ent_dragged {
                                    tok.accent
                                } else if ent_is_const {
                                    tok.text_muted
                                } else {
                                    tok.text_secondary
                                },
                                label: &ent_name,
                                label_color: if ent_is_const {
                                    tok.text_muted
                                } else {
                                    tok.text_primary
                                },
                                bold: false,
                                strikethrough: false,
                                italic: ent_is_const,
                                has_children: false,
                                default_open: false,
                                selected: is_ent_sel || is_ent_dragged,
                                keyboard_focused: tree_kbd_active
                                    && kbd_focused_key.as_ref() == Some(&ent_key),
                                draggable: ent_len > 1,
                                tooltip: Some(&ent_tip),
                            },
                            |ui, _hovered| {
                                if ent_is_const {
                                    tree_badge_pill(
                                        ui,
                                        "Const",
                                        tok.elevated_bg,
                                        Some(tok.border),
                                        tok.text_muted,
                                        Some("Construction geometry (excluded from regions)"),
                                    );
                                }
                            },
                        );
                        entity_row_rects.push(ent_out.row_rect);
                        visible_nodes.push(VisibleTreeNode {
                            key: ent_key.clone(),
                            node_id: ent_id,
                            parent: Some(sk_key.clone()),
                            has_children: false,
                            is_open: false,
                        });
                        if ent_out.response.clicked() {
                            *focus = InspectorFocus::Sketch { sketch_idx: sk_idx };
                            *selected_sketch_entity_idx = Some(ent_idx);
                            *pick_mode = ViewportPickMode::InspectEntity;
                            *kbd_focused_key = Some(ent_key);
                            *any_row_clicked = true;
                        }
                        if ent_out.response.drag_started() && ent_len > 1 {
                            let st = DraggedTreeItem {
                                section: DragSection::SketchEntities { sketch_idx: sk_idx },
                                item_idx: ent_idx,
                            };
                            *active_drag = Some(st);
                            ui.ctx().data_mut(|d| d.insert_temp(drag_state_id, st));
                        }
                        ent_out.response.context_menu(|ui| {
                            show_sketch_entity_context_menu(
                                ui,
                                sk_idx,
                                ent_idx,
                                ent_len,
                                &ent_name,
                                pending,
                                rename_state,
                            );
                        });
                    }

                    if let Some((from_i, to_slot)) = handle_section_drag_overlay(
                        ui,
                        active_drag,
                        DragSection::SketchEntities { sketch_idx: sk_idx },
                        3,
                        &entity_row_rects,
                        drag_state_id,
                    ) {
                        pending.sketch_entity_move_slot = Some((sk_idx, from_i, to_slot));
                    }
                }
            }

            if let Some((from_i, to_slot)) = handle_section_drag_overlay(
                ui,
                active_drag,
                DragSection::Sketches,
                2,
                &sketch_row_rects,
                drag_state_id,
            ) {
                pending.sketch_move_slot = Some((from_i, to_slot));
            }
        }
    }
}
