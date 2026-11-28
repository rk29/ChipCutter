use crate::{
    panels::{InspectorFocus, ViewportPickMode},
    widgets::IconKind,
};

use super::{
    BrowserAction, CamBrowserCtx, TreeNodeKey, TreeRenderState, VisibleTreeNode,
    actions::model_display_name,
    context_menus::{show_model_item_context_menu, show_models_folder_context_menu},
    node_id_for,
    tree_widget::{TreeRowSpec, tree_badge_pill, tree_folder_menu_button, tree_hint_row, tree_row},
};

pub(super) fn show_models_section(
    ui: &mut egui::Ui,
    ctx: &mut CamBrowserCtx<'_>,
    state: &mut TreeRenderState,
) {
    let CamBrowserCtx {
        project,
        cad_doc,
        focus,
        pick_mode,
        ..
    } = ctx;
    let cad_doc = *cad_doc;
    let focus: &mut InspectorFocus = focus;
    let pick_mode: &mut ViewportPickMode = pick_mode;
    let TreeRenderState {
        tree_scope,
        tok,
        root_key,
        primary_model_idx,
        kbd_focused_key,
        tree_kbd_active,
        visible_nodes,
        any_row_clicked,
        emitted_action,
        ..
    } = state;
    let tree_scope = *tree_scope;
    let tok = *tok;
    let primary_model_idx = *primary_model_idx;
    let tree_kbd_active = *tree_kbd_active;

    // -----------------------------------------------------------------
    // 1. Models (N)
    // -----------------------------------------------------------------
    let models_count = if !project.models.is_empty() {
        project.models.len()
    } else if cad_doc.is_some() {
        1
    } else {
        0
    };
    let models_key = TreeNodeKey::ModelsFolder;
    let models_id = node_id_for(tree_scope, &models_key);
    let models_label = format!("Models ({models_count})");
    let has_step_bodies = cad_doc.is_some_and(|d| !d.bodies.is_empty());
    let models_out = tree_row(
        ui,
        TreeRowSpec {
            node_id: models_id,
            depth: 1,
            icon: IconKind::ModelsFolder,
            icon_color: tok.text_secondary,
            label: &models_label,
            label_color: tok.text_primary,
            bold: true,
            strikethrough: false,
            italic: false,
            has_children: true,
            default_open: true,
            selected: false,
            keyboard_focused: tree_kbd_active && kbd_focused_key.as_ref() == Some(&models_key),
            draggable: false,
            tooltip: Some("3D STEP solids, triangle meshes, and imported fixture models"),
        },
        |ui, _hovered| {
            tree_folder_menu_button(
                ui,
                "tree_models_add_menu",
                "Import STEP, 3D Mesh, or open Sample Gallery",
                |ui| {
                    if ui.button("Import STEP...").clicked() {
                        *emitted_action = Some(BrowserAction::OpenStepDialog);
                        ui.close();
                    }
                    if ui.button("Import Mesh (.stl / .obj / .3mf)...").clicked() {
                        *emitted_action =
                            Some(BrowserAction::OpenMeshImportModal { browse_file: true });
                        ui.close();
                    }
                    if has_step_bodies {
                        ui.separator();
                        if ui.button("Reload STEP (Ctrl+R)").clicked() {
                            *emitted_action = Some(BrowserAction::ReloadStepModel);
                            ui.close();
                        }
                        if ui.button("Replace STEP...").clicked() {
                            *emitted_action = Some(BrowserAction::ReplaceStepDialog);
                            ui.close();
                        }
                    }
                    ui.separator();
                    if ui.button("Sample Gallery...").clicked() {
                        *emitted_action = Some(BrowserAction::OpenSampleGallery);
                        ui.close();
                    }
                },
            );
        },
    );
    visible_nodes.push(VisibleTreeNode {
        key: models_key.clone(),
        node_id: models_id,
        parent: Some(root_key.clone()),
        has_children: true,
        is_open: models_out.is_open,
    });
    if models_out.response.clicked() || models_out.chevron_toggled {
        *kbd_focused_key = Some(models_key.clone());
        *any_row_clicked = true;
    }
    models_out.response.context_menu(|ui| {
        show_models_folder_context_menu(ui, emitted_action);
    });

    if models_out.is_open {
        if !project.models.is_empty() {
            for (m_idx, entry) in project.models.iter().enumerate() {
                let is_primary = m_idx == primary_model_idx;
                let key = TreeNodeKey::Model(m_idx);
                let nid = node_id_for(tree_scope, &key);
                let is_sel = matches!(
                    *focus,
                    InspectorFocus::Model { model_idx } if model_idx == m_idx
                ) || (is_primary && matches!(*focus, InspectorFocus::CadEntity));
                let name = model_display_name(entry, cad_doc, is_primary);
                let icon = IconKind::for_model_entry(entry);
                let has_xf = !entry.transform.is_identity();
                let tip = if is_primary && let Some(doc) = cad_doc {
                    if let Some(mesh) = doc.meshes.first() {
                        let status = if mesh.diagnostics.is_watertight {
                            "Watertight"
                        } else {
                            "Open edges"
                        };
                        format!(
                            "{} | {} tris | {} verts | {status}",
                            mesh.format.label(),
                            mesh.diagnostics.valid_triangle_count,
                            mesh.diagnostics.welded_vertex_count
                        )
                    } else {
                        format!(
                            "{} | {} faces | {} loops | {} edges",
                            entry.path,
                            doc.faces.len(),
                            doc.loops.len(),
                            doc.edges.len()
                        )
                    }
                } else {
                    format!("{} ({})", entry.path, entry.role.label())
                };
                let out = tree_row(
                    ui,
                    TreeRowSpec {
                        node_id: nid,
                        depth: 2,
                        icon,
                        icon_color: if is_sel {
                            tok.accent
                        } else {
                            tok.text_secondary
                        },
                        label: &name,
                        label_color: tok.text_primary,
                        bold: false,
                        strikethrough: false,
                        italic: false,
                        has_children: false,
                        default_open: false,
                        selected: is_sel,
                        keyboard_focused: tree_kbd_active && kbd_focused_key.as_ref() == Some(&key),
                        draggable: false,
                        tooltip: Some(&tip),
                    },
                    |ui, _hovered| {
                        if entry.role.is_fixture() {
                            tree_badge_pill(
                                ui,
                                "Fixture",
                                tok.elevated_bg,
                                Some(tok.border),
                                tok.text_muted,
                                Some("Imported workholding fixture solid"),
                            );
                        } else if has_xf {
                            tree_badge_pill(
                                ui,
                                "Xf",
                                tok.info_bg,
                                Some(tok.info_border),
                                tok.info_fg,
                                Some("Non-identity model transform applied"),
                            );
                        }
                    },
                );
                visible_nodes.push(VisibleTreeNode {
                    key: key.clone(),
                    node_id: nid,
                    parent: Some(models_key.clone()),
                    has_children: false,
                    is_open: false,
                });
                if out.response.clicked() {
                    *focus = InspectorFocus::Model { model_idx: m_idx };
                    *pick_mode = ViewportPickMode::InspectEntity;
                    *kbd_focused_key = Some(key);
                    *any_row_clicked = true;
                }
                out.response.context_menu(|ui| {
                    show_model_item_context_menu(
                        ui,
                        m_idx,
                        is_primary,
                        cad_doc.is_some(),
                        true,
                        focus,
                        pick_mode,
                        emitted_action,
                    );
                });
            }
        } else if let Some(doc) = cad_doc {
            let key = TreeNodeKey::Model(0);
            let nid = node_id_for(tree_scope, &key);
            let is_sel = matches!(
                *focus,
                InspectorFocus::Model { model_idx: 0 } | InspectorFocus::CadEntity
            );
            let icon = if !doc.meshes.is_empty() && doc.bodies.is_empty() {
                IconKind::MeshModel
            } else {
                IconKind::StepModel
            };
            let out = tree_row(
                ui,
                TreeRowSpec {
                    node_id: nid,
                    depth: 2,
                    icon,
                    icon_color: if is_sel {
                        tok.accent
                    } else {
                        tok.text_secondary
                    },
                    label: &doc.source_name,
                    label_color: tok.text_primary,
                    bold: false,
                    strikethrough: false,
                    italic: false,
                    has_children: false,
                    default_open: false,
                    selected: is_sel,
                    keyboard_focused: tree_kbd_active && kbd_focused_key.as_ref() == Some(&key),
                    draggable: false,
                    tooltip: Some("Click to inspect loaded 3D model"),
                },
                |_ui, _hovered| {},
            );
            visible_nodes.push(VisibleTreeNode {
                key: key.clone(),
                node_id: nid,
                parent: Some(models_key.clone()),
                has_children: false,
                is_open: false,
            });
            if out.response.clicked() {
                *focus = InspectorFocus::Model { model_idx: 0 };
                *pick_mode = ViewportPickMode::InspectEntity;
                *kbd_focused_key = Some(key);
                *any_row_clicked = true;
            }
            out.response.context_menu(|ui| {
                show_model_item_context_menu(
                    ui,
                    0,
                    true,
                    true,
                    false,
                    focus,
                    pick_mode,
                    emitted_action,
                );
            });
        } else {
            tree_hint_row(ui, 2, "No 3D STEP or mesh model loaded");
        }
    }
}
