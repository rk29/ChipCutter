mod actions;
mod context_menus;
mod drag_reorder;
mod tree_widget;

use chip_cad::CadDocument;
use chip_cam::{OperationStatus, ResolvedSetup, SharedAvoidRegion};
use chip_machines::MachineDefinition;
use chip_project::{ProjectDocument, ToolpathCache, UndoHistory};
use chip_tools::ToolLibrary;
use egui::{Id, Rect};

use crate::{
    panels::{
        InspectorFocus, OperationTab, SetupTab, ViewportPickMode,
        diagnostics::format_cycle_time,
        operation_inspector::{
            avoid_card::{
                mark_shared_avoid_dependents_dirty, next_shared_avoid_id_and_name,
                set_shared_avoid_for_all_setup_ops,
            },
            machinability_card::{
                compact_browser_warning_summary, evaluate_operation_machinability,
            },
        },
        sketch_inspector::mark_sketch_dependent_ops_dirty,
    },
    theme::ThemeTokens,
    widgets::{IconKind, StatusBadgeKind, small_icon_button, status_badge_icon},
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
    duplicate_sketch_entity_in_sketch, duplicate_sketch_in_project, format_operation_badge,
    invalidate_changed_rest_dependents, model_display_name, snapshot_rest_upstream_ids,
};
use context_menus::{
    DeferredBrowserMutations, InlineRenameState, apply_expand_collapse_all,
    render_inline_rename_row, show_add_operation_categories, show_avoid_folder_context_menu,
    show_avoid_item_context_menu, show_fixture_item_context_menu,
    show_fixtures_folder_context_menu, show_model_item_context_menu,
    show_models_folder_context_menu, show_operation_item_context_menu,
    show_operations_folder_context_menu, show_project_root_context_menu,
    show_setup_item_context_menu, show_setups_folder_context_menu, show_sketch_entity_context_menu,
    show_sketch_item_context_menu, show_sketches_folder_context_menu, show_tool_item_context_menu,
    show_tools_folder_context_menu, start_inline_rename_for_key,
};
use drag_reorder::handle_section_drag_overlay;
use tree_widget::{
    TreeRowSpec, poll_tree_nav_key, tree_badge_pill, tree_folder_menu_button, tree_hint_row,
    tree_row,
};

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
pub fn show_cam_browser(ui: &mut egui::Ui, ctx: CamBrowserCtx<'_>) -> Option<BrowserAction> {
    let CamBrowserCtx {
        project,
        global_tools,
        active_machine,
        resolved_active_setup,
        history,
        toolpath_cache,
        cad_doc,
        cad_revision,
        active_setup_idx,
        focus,
        setup_tab,
        op_tab,
        pick_mode,
        hidden_operation_ids,
        editing_shared_avoid_id,
        selected_sketch_entity_idx,
        selected_fixture_idx,
    } = ctx;
    let mut emitted_action: Option<BrowserAction> = None;

    if !project.setups.is_empty() && *active_setup_idx >= project.setups.len() {
        *active_setup_idx = project.setups.len() - 1;
    }

    let tree_scope = ui.make_persistent_id("cam_project_tree");
    let drag_state_id = tree_scope.with("tree_drag_item");
    let nav_focus_id = tree_scope.with("kbd_focus_key");
    let nav_active_id = tree_scope.with("kbd_active");
    let inline_rename_id = tree_scope.with("inline_rename_state");

    let mut active_drag: Option<DraggedTreeItem> = ui
        .ctx()
        .data_mut(|d| d.get_temp::<DraggedTreeItem>(drag_state_id));
    let primary_model_idx = project.primary_part_model_index().unwrap_or(0);
    let mut kbd_focused_key: Option<TreeNodeKey> = ui
        .ctx()
        .data_mut(|d| d.get_temp::<TreeNodeKey>(nav_focus_id))
        .or_else(|| {
            Some(focus_to_tree_key(
                *focus,
                *setup_tab,
                *selected_sketch_entity_idx,
                *selected_fixture_idx,
                primary_model_idx,
            ))
        });
    let mut tree_kbd_active: bool = ui
        .ctx()
        .data_mut(|d| d.get_temp::<bool>(nav_active_id))
        .unwrap_or(false);
    let mut rename_state: Option<InlineRenameState> = ui
        .ctx()
        .data_mut(|d| d.get_temp::<Option<InlineRenameState>>(inline_rename_id))
        .flatten();

    let mut pending = DeferredBrowserMutations::default();
    let mut visible_nodes: Vec<VisibleTreeNode> = Vec::with_capacity(32);
    let mut any_row_clicked = false;

    let scroll_out = egui::ScrollArea::vertical()
        .id_salt("cam_project_tree_scroll")
        .auto_shrink([false, false])
        .show(ui, |ui| {
            ui.spacing_mut().item_spacing.y = 1.0;
            let tok = ThemeTokens::from_ui(ui);

            // -----------------------------------------------------------------
            // Root: [Project] <project.project_name>
            // -----------------------------------------------------------------
            let root_key = TreeNodeKey::Project;
            let root_id = node_id_for(tree_scope, &root_key);
            let root_selected = *focus == InspectorFocus::Project;
            let root_out = tree_row(
                ui,
                TreeRowSpec {
                    node_id: root_id,
                    depth: 0,
                    icon: IconKind::Project,
                    icon_color: tok.accent,
                    label: &project.project_name,
                    label_color: tok.text_primary,
                    bold: true,
                    strikethrough: false,
                    italic: false,
                    has_children: true,
                    default_open: true,
                    selected: root_selected,
                    keyboard_focused: tree_kbd_active
                        && kbd_focused_key.as_ref() == Some(&root_key),
                    draggable: false,
                    tooltip: Some("Click to inspect Project Properties (Right-click for project actions)"),
                },
                |_ui, _hovered| {},
            );
            visible_nodes.push(VisibleTreeNode {
                key: root_key.clone(),
                node_id: root_id,
                parent: None,
                has_children: true,
                is_open: root_out.is_open,
            });
            if root_out.response.clicked() {
                *focus = InspectorFocus::Project;
                *pick_mode = ViewportPickMode::InspectEntity;
                kbd_focused_key = Some(root_key.clone());
                any_row_clicked = true;
            } else if root_out.chevron_toggled {
                kbd_focused_key = Some(root_key.clone());
                any_row_clicked = true;
            }
            root_out.response.context_menu(|ui| {
                show_project_root_context_menu(ui, &mut pending, &mut emitted_action);
            });

            if !root_out.is_open {
                return;
            }

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
                    keyboard_focused: tree_kbd_active
                        && kbd_focused_key.as_ref() == Some(&models_key),
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
                                emitted_action = Some(BrowserAction::OpenStepDialog);
                                ui.close();
                            }
                            if ui.button("Import Mesh (.stl / .obj / .3mf)...").clicked() {
                                emitted_action =
                                    Some(BrowserAction::OpenMeshImportModal { browse_file: true });
                                ui.close();
                            }
                            if has_step_bodies {
                                ui.separator();
                                if ui.button("Reload STEP (Ctrl+R)").clicked() {
                                    emitted_action = Some(BrowserAction::ReloadStepModel);
                                    ui.close();
                                }
                                if ui.button("Replace STEP...").clicked() {
                                    emitted_action = Some(BrowserAction::ReplaceStepDialog);
                                    ui.close();
                                }
                            }
                            ui.separator();
                            if ui.button("Sample Gallery...").clicked() {
                                emitted_action = Some(BrowserAction::OpenSampleGallery);
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
                kbd_focused_key = Some(models_key.clone());
                any_row_clicked = true;
            }
            models_out.response.context_menu(|ui| {
                show_models_folder_context_menu(ui, &mut emitted_action);
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
                                keyboard_focused: tree_kbd_active
                                    && kbd_focused_key.as_ref() == Some(&key),
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
                            kbd_focused_key = Some(key);
                            any_row_clicked = true;
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
                                &mut emitted_action,
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
                            keyboard_focused: tree_kbd_active
                                && kbd_focused_key.as_ref() == Some(&key),
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
                        kbd_focused_key = Some(key);
                        any_row_clicked = true;
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
                            &mut emitted_action,
                        );
                    });
                } else {
                    tree_hint_row(ui, 2, "No 3D STEP or mesh model loaded");
                }
            }

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
                    keyboard_focused: tree_kbd_active
                        && kbd_focused_key.as_ref() == Some(&sketches_key),
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
                                emitted_action = Some(BrowserAction::NewSetupXySketch);
                                ui.close();
                            }
                            if cad_doc.is_some()
                                && ui.button("New 2D Sketch on 3D Face...").clicked()
                            {
                                emitted_action = Some(BrowserAction::StartPickSketchHostFace);
                                ui.close();
                            }
                            ui.separator();
                            if ui.button("Import DXF / SVG...").clicked() {
                                emitted_action = Some(BrowserAction::OpenVectorImportModal {
                                    browse_file: true,
                                });
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
                kbd_focused_key = Some(sketches_key.clone());
                any_row_clicked = true;
            }
            sketches_out.response.context_menu(|ui| {
                show_sketches_folder_context_menu(ui, cad_doc.is_some(), &mut emitted_action);
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
                        let is_sketch_focused =
                            *focus == InspectorFocus::Sketch { sketch_idx: sk_idx };
                        let sk_open_preview = has_entities && is_node_open(ui, sk_id, false);
                        let is_sk_row_selected = is_sketch_focused
                            && (!sk_open_preview || selected_sketch_entity_idx.is_none());
                        let is_sk_dragged = active_drag.is_some_and(|d| {
                            d.section == DragSection::Sketches && d.item_idx == sk_idx
                        });

                        let sk_is_open;
                        if rename_state.as_ref().is_some_and(|r| r.key == sk_key) {
                            let r_rect = ui.available_rect_before_wrap();
                            if let Some(commit) = render_inline_rename_row(
                                ui,
                                2,
                                IconKind::Sketch2D,
                                tok.accent,
                                &mut rename_state,
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
                                        && small_icon_button(
                                            ui,
                                            IconKind::Trash,
                                            "Delete Sketch Layer",
                                        )
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
                                kbd_focused_key = Some(sk_key.clone());
                                any_row_clicked = true;
                            } else if sk_out.chevron_toggled {
                                kbd_focused_key = Some(sk_key.clone());
                                any_row_clicked = true;
                            }
                            if sk_out.response.drag_started() && sk_len > 1 {
                                let st = DraggedTreeItem {
                                    section: DragSection::Sketches,
                                    item_idx: sk_idx,
                                };
                                active_drag = Some(st);
                                ui.ctx().data_mut(|d| d.insert_temp(drag_state_id, st));
                            }
                            sk_out.response.context_menu(|ui| {
                                show_sketch_item_context_menu(
                                    ui,
                                    sk_idx,
                                    sk_len,
                                    sk_visible,
                                    &sk_name,
                                    &mut pending,
                                    &mut rename_state,
                                    &mut emitted_action,
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
                                let is_ent_sel = is_sketch_focused
                                    && *selected_sketch_entity_idx == Some(ent_idx);
                                let ent_icon = IconKind::for_sketch_entity(&ent.entity);
                                let ent_name = ent.name.clone();
                                let ent_is_const = ent.is_construction;
                                let is_ent_dragged = active_drag.is_some_and(|d| {
                                    d.section == DragSection::SketchEntities { sketch_idx: sk_idx }
                                        && d.item_idx == ent_idx
                                });

                                if rename_state.as_ref().is_some_and(|r| r.key == ent_key) {
                                    let r_rect = ui.available_rect_before_wrap();
                                    if let Some(commit) = render_inline_rename_row(
                                        ui,
                                        3,
                                        ent_icon,
                                        tok.accent,
                                        &mut rename_state,
                                    ) {
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

                                let ent_tip =
                                    format!("{ent_name} ({})", ent.entity.kind_label());
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
                                                Some(
                                                    "Construction geometry (excluded from regions)",
                                                ),
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
                                    kbd_focused_key = Some(ent_key);
                                    any_row_clicked = true;
                                }
                                if ent_out.response.drag_started() && ent_len > 1 {
                                    let st = DraggedTreeItem {
                                        section: DragSection::SketchEntities {
                                            sketch_idx: sk_idx,
                                        },
                                        item_idx: ent_idx,
                                    };
                                    active_drag = Some(st);
                                    ui.ctx().data_mut(|d| d.insert_temp(drag_state_id, st));
                                }
                                ent_out.response.context_menu(|ui| {
                                    show_sketch_entity_context_menu(
                                        ui,
                                        sk_idx,
                                        ent_idx,
                                        ent_len,
                                        &ent_name,
                                        &mut pending,
                                        &mut rename_state,
                                    );
                                });
                            }

                            if let Some((from_i, to_slot)) = handle_section_drag_overlay(
                                ui,
                                &mut active_drag,
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
                        &mut active_drag,
                        DragSection::Sketches,
                        2,
                        &sketch_row_rects,
                        drag_state_id,
                    ) {
                        pending.sketch_move_slot = Some((from_i, to_slot));
                    }
                }
            }

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
                    keyboard_focused: tree_kbd_active
                        && kbd_focused_key.as_ref() == Some(&avoid_key),
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
                kbd_focused_key = Some(avoid_key.clone());
                any_row_clicked = true;
            }
            avoid_out.response.context_menu(|ui| {
                show_avoid_folder_context_menu(ui, &mut pending);
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
                        let is_dragged = active_drag.is_some_and(|d| {
                            d.section == DragSection::AvoidRegions && d.item_idx == r_idx
                        });
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
                                &mut rename_state,
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
                            kbd_focused_key = Some(r_key);
                            any_row_clicked = true;
                        }
                        if r_out.response.drag_started() && avoid_len > 1 {
                            let st = DraggedTreeItem {
                                section: DragSection::AvoidRegions,
                                item_idx: r_idx,
                            };
                            active_drag = Some(st);
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
                                &mut pending,
                                &mut rename_state,
                            );
                        });
                    }

                    if let Some((from_i, to_slot)) = handle_section_drag_overlay(
                        ui,
                        &mut active_drag,
                        DragSection::AvoidRegions,
                        2,
                        &avoid_row_rects,
                        drag_state_id,
                    ) {
                        pending.avoid_move_slot = Some((from_i, to_slot));
                    }
                }
            }

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
                    keyboard_focused: tree_kbd_active
                        && kbd_focused_key.as_ref() == Some(&tools_key),
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
                kbd_focused_key = Some(tools_key.clone());
                any_row_clicked = true;
            }
            tools_out.response.context_menu(|ui| {
                show_tools_folder_context_menu(
                    ui,
                    unused_count,
                    &mut pending,
                    &mut emitted_action,
                );
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
                                        &format!(
                                            "Remove unused tool T{} from project",
                                            tool.tool_number
                                        ),
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
                            kbd_focused_key = Some(t_key);
                            any_row_clicked = true;
                        }
                        t_out.response.context_menu(|ui| {
                            show_tool_item_context_menu(
                                ui,
                                &tool.id,
                                in_machine_lib,
                                use_count,
                                &mut pending,
                                &mut emitted_action,
                            );
                        });
                    }
                }
            }

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
                    keyboard_focused: tree_kbd_active
                        && kbd_focused_key.as_ref() == Some(&setups_key),
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
                kbd_focused_key = Some(setups_key.clone());
                any_row_clicked = true;
            }
            setups_out.response.context_menu(|ui| {
                show_setups_folder_context_menu(ui, &mut pending);
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
                    let is_setup_sel = *focus == InspectorFocus::Setup { setup_idx: s_idx }
                        && !fixture_selected_here;
                    let is_setup_dragged = active_drag.is_some_and(|d| {
                        d.section == DragSection::Setups && d.item_idx == s_idx
                    });

                    let s_is_open;
                    if rename_state.as_ref().is_some_and(|r| r.key == s_key) {
                        let r_rect = ui.available_rect_before_wrap();
                        if let Some(commit) = render_inline_rename_row(
                            ui,
                            2,
                            IconKind::Setup,
                            tok.accent,
                            &mut rename_state,
                        ) {
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
                                        && small_icon_button(ui, IconKind::Trash, "Delete Setup")
                                            .clicked()
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
                                        emitted_action =
                                            Some(BrowserAction::AlignCameraToActiveSetup);
                                    }
                                    if small_icon_button(
                                        ui,
                                        IconKind::Generate,
                                        "Generate dirty toolpaths in this Setup (Ctrl+G)",
                                    )
                                    .clicked()
                                    {
                                        *active_setup_idx = s_idx;
                                        emitted_action = Some(BrowserAction::GenerateSetup(s_idx));
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
                            kbd_focused_key = Some(s_key.clone());
                            any_row_clicked = true;
                        } else if s_out.chevron_toggled {
                            kbd_focused_key = Some(s_key.clone());
                            any_row_clicked = true;
                        }
                        if s_out.response.drag_started() && setup_count > 1 {
                            let st = DraggedTreeItem {
                                section: DragSection::Setups,
                                item_idx: s_idx,
                            };
                            active_drag = Some(st);
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
                                &mut pending,
                                &mut rename_state,
                                &mut emitted_action,
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
                            tooltip: Some(
                                "Click to open this Setup's 3D Workholding & Fixtures tab",
                            ),
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
                        kbd_focused_key = Some(fix_folder_key.clone());
                        any_row_clicked = true;
                    } else if fix_folder_out.chevron_toggled {
                        kbd_focused_key = Some(fix_folder_key.clone());
                        any_row_clicked = true;
                    }
                    fix_folder_out.response.context_menu(|ui| {
                        show_fixtures_folder_context_menu(
                            ui,
                            s_idx,
                            &mut pending,
                            &mut emitted_action,
                        );
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
                                    &mut rename_state,
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
                                kbd_focused_key = Some(f_key);
                                any_row_clicked = true;
                            }
                            if f_out.response.drag_started() && fix_count > 1 {
                                let st = DraggedTreeItem {
                                    section: DragSection::Fixtures { setup_idx: s_idx },
                                    item_idx: f_idx,
                                };
                                active_drag = Some(st);
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
                                    &mut pending,
                                    &mut rename_state,
                                );
                            });
                        }

                        if let Some((from_i, to_slot)) = handle_section_drag_overlay(
                            ui,
                            &mut active_drag,
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
                                    show_add_operation_categories(ui, s_idx, &mut pending);
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
                        kbd_focused_key = Some(ops_folder_key.clone());
                        any_row_clicked = true;
                    } else if ops_folder_out.chevron_toggled {
                        kbd_focused_key = Some(ops_folder_key.clone());
                        any_row_clicked = true;
                    }
                    ops_folder_out.response.context_menu(|ui| {
                        show_operations_folder_context_menu(
                            ui,
                            s_idx,
                            active_setup_idx,
                            &mut pending,
                            &mut emitted_action,
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
                            let cached_tp = toolpath_cache.get_for_operation(
                                project,
                                s_idx,
                                o_idx,
                                cad_revision,
                            );
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
                                    &mut rename_state,
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
                                let rs_for_op =
                                    (s_idx == *active_setup_idx).then_some(resolved_active_setup).flatten();
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
                            let is_needs_repick = !is_sup
                                && matches!(op.runtime_status, OperationStatus::NeedsRepick { .. });
                            let is_gen = !is_sup
                                && matches!(op.runtime_status, OperationStatus::Generating);
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
                                        if small_icon_button(
                                            ui,
                                            IconKind::Trash,
                                            "Delete Operation (Del)",
                                        )
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

                                    if !show_hover_tools
                                        && let Some(badge) = &badge_txt
                                    {
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
                                *pick_mode =
                                    default_pick_mode_for_op(s_idx, o_idx, op_kind_val, *op_tab);
                                kbd_focused_key = Some(op_key);
                                any_row_clicked = true;
                            }
                            if op_out.response.drag_started() && op_len > 1 {
                                let st = DraggedTreeItem {
                                    section: DragSection::Operations { setup_idx: s_idx },
                                    item_idx: o_idx,
                                };
                                active_drag = Some(st);
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
                                    &mut pending,
                                    &mut rename_state,
                                    &mut emitted_action,
                                );
                            });

                            row_rects.push(op_out.row_rect);
                        }

                        if let Some((from_i, to_slot)) = handle_section_drag_overlay(
                            ui,
                            &mut active_drag,
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
                    &mut active_drag,
                    DragSection::Setups,
                    2,
                    &setup_row_rects,
                    drag_state_id,
                ) {
                    pending.setup_move_slot = Some((from_i, to_slot));
                }
            }
        });

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
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use chip_cam::{
        FixtureBody, Operation, OperationKind, RestMachiningMode, Setup, WorkOffsetSlot,
    };
    use chip_math::{EvalContext, ParamLength};
    use chip_project::ModelEntry;
    use chip_sketch::{SketchEntity, SketchEntityEntry, SketchLayer};
    use egui::Pos2;

    use crate::{
        panels::{
            keepout_inspector::{KeepOutInspectorCtx, show_keepout_inspector},
            model_inspector::{ModelInspectorCtx, show_model_inspector},
            operation_inspector::{OperationInspectorCtx, show_operation_inspector},
            project_properties::{ProjectPropertiesCtx, show_project_inspector},
            project_tool_inspector::{ProjectToolInspectorCtx, show_project_tool_inspector},
            setup_inspector::{FixtureSafetySummary, SetupInspectorCtx, show_setup_inspector},
            sketch_inspector::{SketchInspectorCtx, show_sketch_inspector},
        },
        theme::{AppTheme, apply_theme},
    };

    fn collect_texts(shape: &egui::Shape, out: &mut Vec<String>) {
        match shape {
            egui::Shape::Text(t) => out.push(t.galley.text().to_string()),
            egui::Shape::Vec(v) => {
                for s in v {
                    collect_texts(s, out);
                }
            }
            _ => {}
        }
    }

    fn find_text_rect(shapes: &[egui::epaint::ClippedShape], needle: &str) -> Option<Rect> {
        fn visit(shape: &egui::Shape, needle: &str) -> Option<Rect> {
            match shape {
                egui::Shape::Text(t) if t.galley.text() == needle => Some(t.visual_bounding_rect()),
                egui::Shape::Vec(v) => v.iter().find_map(|s| visit(s, needle)),
                _ => None,
            }
        }
        shapes.iter().find_map(|cs| visit(&cs.shape, needle))
    }

    fn find_text_rect_containing(
        shapes: &[egui::epaint::ClippedShape],
        needle: &str,
    ) -> Option<Rect> {
        fn visit(shape: &egui::Shape, needle: &str) -> Option<Rect> {
            match shape {
                egui::Shape::Text(t) if t.galley.text().contains(needle) => {
                    Some(t.visual_bounding_rect())
                }
                egui::Shape::Vec(v) => v.iter().find_map(|s| visit(s, needle)),
                _ => None,
            }
        }
        shapes.iter().find_map(|cs| visit(&cs.shape, needle))
    }

    fn click_btn(pos: Pos2, button: egui::PointerButton) -> Vec<egui::Event> {
        vec![
            egui::Event::PointerMoved(pos),
            egui::Event::PointerButton {
                pos,
                button,
                pressed: true,
                modifiers: egui::Modifiers::NONE,
            },
            egui::Event::PointerButton {
                pos,
                button,
                pressed: false,
                modifiers: egui::Modifiers::NONE,
            },
        ]
    }

    fn build_rich_test_project() -> ProjectDocument {
        let mut project = ProjectDocument::new("Aluminum Manifold Rev B");
        project
            .models
            .push(ModelEntry::new("m_step", "parts/manifold.step", "sha_step"));
        project
            .models
            .push(ModelEntry::new("m_mesh", "parts/relief.stl", "sha_stl"));
        project.models.push(ModelEntry::new_fixture(
            "m_fix",
            "fixtures/kurt_vise.step",
            "sha_fix",
        ));

        let mut sk = SketchLayer::new("sk_1", "Mounting Holes", "setup_1");
        sk.entities.push(SketchEntityEntry {
            id: "ent_1".to_string(),
            name: "Outer Pocket".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::new_circle(
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(20.0),
            ),
        });
        project.sketches.push(sk);

        project
            .avoid_regions
            .push(SharedAvoidRegion::new("avoid_1", "Strap Clamps"));

        let lib = chip_tools::default_starter_tool_library();
        if let Some(t) = lib.tools.first() {
            project.ensure_tool_embedded(t);
        }

        project.setups[0]
            .fixtures
            .push(FixtureBody::default_vise("fix_1", "Kurt 4in Vise"));
        let mut op = Operation::new("op_1", "Adaptive Rough", OperationKind::Adaptive3D);
        op.tool_id = project.tools.first().map(|t| t.id.clone());
        project.setups[0].operations.push(op);
        project
    }

    struct BrowserTestHarness {
        ctx: egui::Context,
        inspector_ctx: egui::Context,
        project: ProjectDocument,
        global_tools: ToolLibrary,
        machines: Vec<chip_machines::MachineDefinition>,
        history: UndoHistory,
        cache: ToolpathCache,
        active_setup_idx: usize,
        focus: InspectorFocus,
        setup_tab: SetupTab,
        op_tab: OperationTab,
        pick_mode: ViewportPickMode,
        pick_target: crate::panels::GeometryPickTarget,
        hidden: std::collections::HashSet<String>,
        editing_avoid: Option<String>,
        sel_ent: Option<usize>,
        sel_fix: Option<usize>,
        time: f64,
    }

    impl BrowserTestHarness {
        fn new(project: ProjectDocument) -> Self {
            Self {
                ctx: egui::Context::default(),
                inspector_ctx: egui::Context::default(),
                project,
                global_tools: chip_tools::default_starter_tool_library(),
                machines: chip_machines::builtin_machine_definitions(),
                history: UndoHistory::default(),
                cache: ToolpathCache::new(4),
                active_setup_idx: 0,
                focus: InspectorFocus::Setup { setup_idx: 0 },
                setup_tab: SetupTab::Wcs,
                op_tab: OperationTab::Geometry,
                pick_mode: ViewportPickMode::InspectEntity,
                pick_target: crate::panels::GeometryPickTarget::CuttingArea,
                hidden: std::collections::HashSet::new(),
                editing_avoid: None,
                sel_ent: None,
                sel_fix: None,
                time: 1.0,
            }
        }

        fn step(
            &mut self,
            events: Vec<egui::Event>,
        ) -> (Vec<egui::epaint::ClippedShape>, Option<BrowserAction>) {
            self.time += 0.1;
            let mut action = None;
            let mut out = self.ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(360.0, 960.0))),
                    time: Some(self.time),
                    events,
                    ..Default::default()
                },
                |ui| {
                    egui::CentralPanel::default().show(ui, |ui| {
                        action = show_cam_browser(
                            ui,
                            CamBrowserCtx {
                                project: &mut self.project,
                                global_tools: &self.global_tools,
                                active_machine: None,
                                resolved_active_setup: None,
                                history: &mut self.history,
                                toolpath_cache: &self.cache,
                                cad_doc: None,
                                cad_revision: 0,
                                active_setup_idx: &mut self.active_setup_idx,
                                focus: &mut self.focus,
                                setup_tab: &mut self.setup_tab,
                                op_tab: &mut self.op_tab,
                                pick_mode: &mut self.pick_mode,
                                hidden_operation_ids: &mut self.hidden,
                                editing_shared_avoid_id: &mut self.editing_avoid,
                                selected_sketch_entity_idx: &mut self.sel_ent,
                                selected_fixture_idx: &mut self.sel_fix,
                            },
                        );
                    });
                },
            );
            out.textures_delta.clear();
            (out.shapes, action)
        }

        fn render_active_inspector(&mut self) -> Vec<String> {
            self.time += 0.1;
            let eval_ctx = EvalContext::new(self.project.display_units);
            let mut out = self.inspector_ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(420.0, 900.0))),
                    time: Some(self.time),
                    ..Default::default()
                },
                |ui| {
                    egui::CentralPanel::default().show(ui, |ui| match self.focus {
                        InspectorFocus::Project => {
                            let _ = show_project_inspector(
                                ui,
                                ProjectPropertiesCtx {
                                    project: &self.project,
                                    project_file_path: None,
                                    machines: &self.machines,
                                },
                            );
                        }
                        InspectorFocus::Model { model_idx } => {
                            let mut lock = true;
                            let _ = show_model_inspector(
                                ui,
                                ModelInspectorCtx {
                                    project: &mut self.project,
                                    model_idx,
                                    history: &mut self.history,
                                    cad_doc: None,
                                    lock_uniform_scale: &mut lock,
                                    pick_mode: &mut self.pick_mode,
                                    focus: &mut self.focus,
                                },
                            );
                        }
                        InspectorFocus::Sketch { sketch_idx } => {
                            let mut snap = None;
                            let mut start_xy = None;
                            let _ = show_sketch_inspector(
                                ui,
                                SketchInspectorCtx {
                                    project: &mut self.project,
                                    sketch_idx,
                                    active_setup_idx: self.active_setup_idx,
                                    selected_entity_idx: &mut self.sel_ent,
                                    snap_grid_mm: &mut snap,
                                    author_start_xy: &mut start_xy,
                                    history: &mut self.history,
                                    pick_mode: &mut self.pick_mode,
                                    eval_ctx: &eval_ctx,
                                    has_cad_model: false,
                                    project_file: None,
                                },
                            );
                        }
                        InspectorFocus::KeepOutRegion { region_idx } => {
                            let _ = show_keepout_inspector(
                                ui,
                                KeepOutInspectorCtx {
                                    project: &mut self.project,
                                    region_idx,
                                    active_setup_idx: self.active_setup_idx,
                                    history: &mut self.history,
                                    eval_ctx: &eval_ctx,
                                    pick_mode: &mut self.pick_mode,
                                    editing_shared_avoid_id: &mut self.editing_avoid,
                                    focus: &mut self.focus,
                                },
                            );
                        }
                        InspectorFocus::ProjectTool { tool_idx } => {
                            let _ = show_project_tool_inspector(
                                ui,
                                ProjectToolInspectorCtx {
                                    project: &self.project,
                                    global_tools: &self.global_tools,
                                    tool_idx,
                                },
                            );
                        }
                        InspectorFocus::Setup { setup_idx } => {
                            let mut show_fix = true;
                            let mut show_dots = true;
                            let mut show_z = true;
                            let mut open_mm = false;
                            let mut open_post = None;
                            let mut m_ch = false;
                            let mut sw_m = None;
                            let mut imp_f = false;
                            let mut fj = false;
                            let mut sj = false;
                            let _ = show_setup_inspector(
                                ui,
                                SetupInspectorCtx {
                                    project: &mut self.project,
                                    setup_idx,
                                    history: &mut self.history,
                                    project_dir: None,
                                    cad_doc: None,
                                    resolved: None,
                                    resolved_fixtures: &[],
                                    fixture_safety: FixtureSafetySummary::default(),
                                    tab: &mut self.setup_tab,
                                    pick_mode: &mut self.pick_mode,
                                    selected_fixture_idx: &mut self.sel_fix,
                                    show_fixtures: &mut show_fix,
                                    show_27pt_dots: &mut show_dots,
                                    show_z_planes: &mut show_z,
                                    machines: &self.machines,
                                    open_machine_manager: &mut open_mm,
                                    open_post_inspector: &mut open_post,
                                    machine_changed: &mut m_ch,
                                    switch_machine_id: &mut sw_m,
                                    import_fixture_requested: &mut imp_f,
                                    open_flip_jig_wizard: &mut fj,
                                    open_soft_jaw_wizard: &mut sj,
                                },
                            );
                        }
                        InspectorFocus::Operation { setup_idx, op_idx } => {
                            let mut lock_cl = true;
                            let mut show_z = true;
                            let mut show_reach = false;
                            let mut auto_rot = true;
                            let mut open_tm = false;
                            let _ = show_operation_inspector(
                                ui,
                                OperationInspectorCtx {
                                    project: &mut self.project,
                                    setup_idx,
                                    op_idx,
                                    global_tools: &self.global_tools,
                                    active_machine: None,
                                    history: &mut self.history,
                                    cad_doc: None,
                                    resolved_setup: None,
                                    surface_mesh: None,
                                    boundary_preview: None,
                                    selected_face_normal_wcs: None,
                                    extracted: None,
                                    tab: &mut self.op_tab,
                                    pick_mode: &mut self.pick_mode,
                                    pick_target: &mut self.pick_target,
                                    editing_shared_avoid_id: &mut self.editing_avoid,
                                    lock_chipload: &mut lock_cl,
                                    show_z_planes: &mut show_z,
                                    show_reach_heatmap: &mut show_reach,
                                    force_expand_machinability: false,
                                    auto_rotate_indexed_op: &mut auto_rot,
                                    open_tool_manager: &mut open_tm,
                                    sim_engine: chip_sim::SimEngineKind::Heightmap,
                                },
                            );
                        }
                        InspectorFocus::CadEntity => {}
                    });
                },
            );
            out.textures_delta.clear();
            let mut texts = Vec::new();
            for cs in &out.shapes {
                collect_texts(&cs.shape, &mut texts);
            }
            texts
        }
    }

    #[test]
    fn move_duplicate_and_delete_propagate_rest_machining_dirty_state() {
        let mut setup = Setup::new("s1", "Setup 1", WorkOffsetSlot::G54);
        let mut op1 = Operation::new("op_1", "Rough 1", OperationKind::Pocket2D);
        op1.runtime_status = OperationStatus::Clean;
        let mut op2 = Operation::new("op_2", "Rough 2", OperationKind::Pocket2D);
        op2.runtime_status = OperationStatus::Clean;
        let mut op3 = Operation::new("op_3", "Rest Finish", OperationKind::Contour2D);
        op3.passes.rest_machining.mode = RestMachiningMode::FromPrecedingOperations {
            source_op_ids: Vec::new(),
        };
        op3.runtime_status = OperationStatus::Clean;

        setup.operations = vec![op1, op2, op3];

        assert_eq!(move_operation_to_slot(&mut setup, 1, 1), None);
        assert_eq!(move_operation_to_slot(&mut setup, 1, 2), None);
        assert_eq!(setup.operations[2].runtime_status, OperationStatus::Clean);

        let dst = move_operation_to_slot(&mut setup, 1, 3);
        assert_eq!(dst, Some(2));
        assert_eq!(setup.operations[0].id, "op_1");
        assert_eq!(setup.operations[1].id, "op_3");
        assert_eq!(setup.operations[2].id, "op_2");
        assert_eq!(setup.operations[1].runtime_status, OperationStatus::Dirty);

        setup.operations[1].runtime_status = OperationStatus::Clean;
        let dup_idx = duplicate_operation_in_setup(&mut setup, 0);
        assert_eq!(dup_idx, Some(1));
        assert_eq!(setup.operations.len(), 4);
        assert_eq!(setup.operations[2].id, "op_3");
        assert_eq!(setup.operations[2].runtime_status, OperationStatus::Dirty);

        setup.operations[2].runtime_status = OperationStatus::Clean;
        assert!(delete_operation_in_setup(&mut setup, 1));
        assert_eq!(setup.operations[1].id, "op_3");
        assert_eq!(setup.operations[1].runtime_status, OperationStatus::Dirty);
    }

    #[test]
    fn all_six_drag_reorder_helpers_preserve_focus_propagate_dirty_and_reject_cross_section() {
        let mut project = build_rich_test_project();

        // 1. Setups + carryover dirty propagation
        let mut s2 = Setup::new("setup_2", "Setup 2 - Op 2", WorkOffsetSlot::G55);
        s2.carryover_from_setup_id = Some("setup_1".to_string());
        let mut s2_op = Operation::new("s2_op1", "Op 2 Finish", OperationKind::Contour2D);
        s2_op.runtime_status = OperationStatus::Clean;
        s2.operations.push(s2_op);
        let s3 = Setup::new("setup_3", "Setup 3 - Op 3", WorkOffsetSlot::G56);
        project.setups.push(s2);
        project.setups.push(s3);

        let mut active_setup = 1usize;
        let mut focus = InspectorFocus::Setup { setup_idx: 1 };
        // Move setup_3 (idx 2) before setup_2 (slot 1) -> setup_2's upstream ordering changed!
        let dst = move_setup_to_slot(&mut project, 2, 1, &mut active_setup, &mut focus);
        assert_eq!(dst, Some(1));
        assert_eq!(active_setup, 2, "active_setup_idx must track setup_2");
        assert_eq!(focus, InspectorFocus::Setup { setup_idx: 2 });
        assert_eq!(
            project.setups[2].operations[0].runtime_status,
            OperationStatus::Dirty,
            "carryover-dependent setup must be marked Dirty when upstream ordering changes"
        );

        // 2. Sketches
        project
            .sketches
            .push(SketchLayer::new("sk_2", "Text Engraving", "setup_1"));
        focus = InspectorFocus::Sketch { sketch_idx: 0 };
        assert_eq!(move_sketch_to_slot(&mut project, 0, 2, &mut focus), Some(1));
        assert_eq!(project.sketches[0].id, "sk_2");
        assert_eq!(project.sketches[1].id, "sk_1");
        assert_eq!(focus, InspectorFocus::Sketch { sketch_idx: 1 });

        // 3. Sketch Entities
        project.sketches[1].entities.push(SketchEntityEntry {
            id: "ent_2".to_string(),
            name: "Inner Hole".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::new_circle(
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(6.0),
            ),
        });
        let mut sel_ent = Some(1);
        assert_eq!(
            move_sketch_entity_to_slot(&mut project, 1, 1, 0, &mut sel_ent),
            Some(0)
        );
        assert_eq!(project.sketches[1].entities[0].id, "ent_2");
        assert_eq!(sel_ent, Some(0));

        // 4. Keep-Out Regions
        project
            .avoid_regions
            .push(SharedAvoidRegion::new("avoid_2", "Toe Clamp North"));
        focus = InspectorFocus::KeepOutRegion { region_idx: 1 };
        assert_eq!(
            move_avoid_region_to_slot(&mut project, 1, 0, &mut focus),
            Some(0)
        );
        assert_eq!(project.avoid_regions[0].id, "avoid_2");
        assert_eq!(focus, InspectorFocus::KeepOutRegion { region_idx: 0 });

        // 5. Fixtures
        project.setups[0]
            .fixtures
            .push(FixtureBody::default_spoilboard("fix_2", "MDF Spoilboard"));
        let mut sel_fix = Some(0);
        assert_eq!(
            move_fixture_to_slot(&mut project, 0, 0, 2, &mut sel_fix),
            Some(1)
        );
        assert_eq!(project.setups[0].fixtures[1].id, "fix_1");
        assert_eq!(sel_fix, Some(1));

        // 6. Strict cross-section boundary rejection
        let rects = vec![
            Rect::from_min_size(Pos2::new(0.0, 0.0), egui::vec2(200.0, 24.0)),
            Rect::from_min_size(Pos2::new(0.0, 25.0), egui::vec2(200.0, 24.0)),
        ];
        let dragged_op_s0 = DraggedTreeItem {
            section: DragSection::Operations { setup_idx: 0 },
            item_idx: 0,
        };
        assert_eq!(
            resolve_drop_slot_for_section(
                dragged_op_s0,
                DragSection::Operations { setup_idx: 1 },
                &rects,
                Pos2::new(50.0, 10.0)
            ),
            None,
            "Operation from Setup 0 must never drop into Setup 1"
        );
        assert_eq!(
            resolve_drop_slot_for_section(
                dragged_op_s0,
                DragSection::Fixtures { setup_idx: 0 },
                &rects,
                Pos2::new(50.0, 10.0)
            ),
            None,
            "Operation must never drop into Fixtures section"
        );
        assert_eq!(
            resolve_drop_slot_for_section(
                dragged_op_s0,
                DragSection::Operations { setup_idx: 0 },
                &rects,
                Pos2::new(50.0, 45.0)
            ),
            Some(2),
            "Same-section drop resolves cleanly"
        );
    }

    #[test]
    fn selecting_all_nine_tree_node_types_activates_exact_focus_and_renders_right_panel() {
        let mut h = BrowserTestHarness::new(build_rich_test_project());
        let (shapes, _) = h.step(Vec::new());

        // Expand Sketch and Project Tools so child entities and tools are visible too
        let sk_rect = find_text_rect(&shapes, "Mounting Holes").expect("sketch label");
        let _ = h.step(click_btn(
            Pos2::new(sk_rect.left() - 26.5, sk_rect.center().y),
            egui::PointerButton::Primary,
        ));
        let (shapes, _) = h.step(Vec::new());
        let tools_rect =
            find_text_rect_containing(&shapes, "Project Tools (").expect("tools folder label");
        let _ = h.step(click_btn(
            Pos2::new(tools_rect.left() - 26.5, tools_rect.center().y),
            egui::PointerButton::Primary,
        ));
        let (shapes, _) = h.step(Vec::new());

        // 1. Project root -> InspectorFocus::Project
        let pos = find_text_rect(&shapes, "Aluminum Manifold Rev B")
            .expect("project root")
            .center();
        let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
        assert_eq!(h.focus, InspectorFocus::Project);
        let texts = h.render_active_inspector();
        assert!(texts.iter().any(|t| t == "Project Properties"));
        assert!(texts.iter().any(|t| t == "Project Summary"));

        // 2. Model -> InspectorFocus::Model
        let pos = find_text_rect(&shapes, "manifold.step")
            .expect("model row")
            .center();
        let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
        assert_eq!(h.focus, InspectorFocus::Model { model_idx: 0 });
        let _ = h.render_active_inspector();

        // 3. Sketch -> InspectorFocus::Sketch (entity None)
        let pos = find_text_rect(&shapes, "Mounting Holes")
            .expect("sketch row")
            .center();
        let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
        assert_eq!(h.focus, InspectorFocus::Sketch { sketch_idx: 0 });
        assert_eq!(h.sel_ent, None);
        let texts = h.render_active_inspector();
        assert!(texts.iter().any(|t| t == "2D Sketch Layer"));

        // 4. SketchEntity -> InspectorFocus::Sketch + selected_sketch_entity_idx = Some(0)
        let pos = find_text_rect(&shapes, "Outer Pocket")
            .expect("sketch entity row")
            .center();
        let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
        assert_eq!(h.focus, InspectorFocus::Sketch { sketch_idx: 0 });
        assert_eq!(h.sel_ent, Some(0));
        let texts = h.render_active_inspector();
        assert!(texts.iter().any(|t| t.contains("Outer Pocket")));

        // 5. KeepOutRegion -> InspectorFocus::KeepOutRegion
        let pos = find_text_rect(&shapes, "Strap Clamps")
            .expect("keep-out row")
            .center();
        let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
        assert_eq!(h.focus, InspectorFocus::KeepOutRegion { region_idx: 0 });
        let texts = h.render_active_inspector();
        assert!(texts.iter().any(|t| t == "Keep-Out Region"));
        assert!(texts.iter().any(|t| t.contains("Pick 3D Avoid Geometry")));
        assert!(texts.iter().any(|t| t.contains("Linked Operations")));

        // 6. ProjectTool -> InspectorFocus::ProjectTool
        let pos = find_text_rect_containing(&shapes, "T1 -")
            .expect("project tool row")
            .center();
        let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
        assert_eq!(h.focus, InspectorFocus::ProjectTool { tool_idx: 0 });
        let texts = h.render_active_inspector();
        assert!(texts.iter().any(|t| t == "Cutter Profile"));
        assert!(texts.iter().any(|t| t == "Dimensions & Offsets"));
        assert!(texts.iter().any(|t| t == "Edit in Tool Library..."));

        // 7. Setup -> InspectorFocus::Setup
        let pos = find_text_rect(&shapes, "Setup 1 — Top Op")
            .expect("setup row")
            .center();
        let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
        assert_eq!(h.focus, InspectorFocus::Setup { setup_idx: 0 });
        let texts = h.render_active_inspector();
        assert!(texts.iter().any(|t| t == "Setup Name"));

        // 8. Fixture -> InspectorFocus::Setup + SetupTab::Fixtures + selected_fixture_idx = Some(0)
        let pos = find_text_rect(&shapes, "Kurt 4in Vise")
            .expect("fixture row")
            .center();
        let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
        assert_eq!(h.focus, InspectorFocus::Setup { setup_idx: 0 });
        assert_eq!(h.setup_tab, SetupTab::Fixtures);
        assert_eq!(h.sel_fix, Some(0));
        let texts = h.render_active_inspector();
        assert!(texts.iter().any(|t| t == "Selected Fixture Settings"));

        // 9. Operation -> InspectorFocus::Operation
        let pos = find_text_rect(&shapes, "1. Adaptive Rough")
            .expect("operation row")
            .center();
        let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
        assert_eq!(
            h.focus,
            InspectorFocus::Operation {
                setup_idx: 0,
                op_idx: 0
            }
        );
        let _ = h.render_active_inspector();
    }

    #[test]
    fn context_menu_actions_inline_f2_rename_undo_and_dirty_propagation() {
        let mut project = build_rich_test_project();
        project.setups[0].operations[0].runtime_status = OperationStatus::Clean;
        let mut h = BrowserTestHarness::new(project);
        let (shapes, _) = h.step(Vec::new());

        // 1. Right-click Keep-Out Region -> Apply to Active Setup Ops -> links op + dirties + Undo
        let avoid_pos = find_text_rect(&shapes, "Strap Clamps")
            .expect("keep-out row")
            .center();
        let _ = h.step(click_btn(avoid_pos, egui::PointerButton::Secondary));
        let (menu_shapes, _) = h.step(Vec::new());
        let apply_pos = find_text_rect(&menu_shapes, "Apply to Active Setup Ops")
            .expect("Apply to Active Setup Ops context item")
            .center();
        let _ = h.step(click_btn(apply_pos, egui::PointerButton::Primary));
        assert!(
            h.project.setups[0].operations[0]
                .avoid
                .shared_region_ids
                .contains(&"avoid_1".to_string())
        );
        assert_eq!(
            h.project.setups[0].operations[0].runtime_status,
            OperationStatus::Dirty
        );

        // Mark op Clean again, then right-click Keep-Out Region -> Suppress -> dirties + atomic Undo
        h.project.setups[0].operations[0].runtime_status = OperationStatus::Clean;
        let (shapes, _) = h.step(Vec::new());
        let avoid_pos = find_text_rect(&shapes, "Strap Clamps")
            .expect("keep-out row")
            .center();
        let _ = h.step(click_btn(avoid_pos, egui::PointerButton::Secondary));
        let (menu_shapes, _) = h.step(Vec::new());
        let sup_pos = find_text_rect(&menu_shapes, "Suppress")
            .expect("Suppress context item")
            .center();
        let _ = h.step(click_btn(sup_pos, egui::PointerButton::Primary));
        assert!(h.project.avoid_regions[0].suppressed);
        assert_eq!(
            h.project.setups[0].operations[0].runtime_status,
            OperationStatus::Dirty,
            "suppressing linked Keep-Out Region must dirty referencing operations"
        );
        assert_eq!(
            h.history.undo(&mut h.project).as_deref(),
            Some("Toggle Shared Keep-Out Suppress")
        );
        assert!(!h.project.avoid_regions[0].suppressed);

        // 2. Inline F2 rename on Operation -> Escape cancels without mutating or pushing Undo
        let (shapes, _) = h.step(Vec::new());
        let op_pos = find_text_rect(&shapes, "1. Adaptive Rough")
            .expect("operation row")
            .center();
        let _ = h.step(click_btn(op_pos, egui::PointerButton::Primary));
        let _ = h.step(vec![egui::Event::Key {
            key: egui::Key::F2,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::NONE,
        }]);
        let _ = h.step(vec![
            egui::Event::Text("Cancelled Name".to_string()),
            egui::Event::Key {
                key: egui::Key::Escape,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::NONE,
            },
        ]);
        assert_eq!(h.project.setups[0].operations[0].name, "Adaptive Rough");

        // 3. Inline F2 rename on Operation -> type new name + Enter -> atomic Undo
        let _ = h.step(Vec::new());
        let _ = h.step(vec![egui::Event::Key {
            key: egui::Key::F2,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::NONE,
        }]);
        let _ = h.step(Vec::new());
        let _ = h.step(vec![
            egui::Event::Key {
                key: egui::Key::A,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::COMMAND,
            },
            egui::Event::Text("Roughing Pocket Pass".to_string()),
            egui::Event::Key {
                key: egui::Key::Enter,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::NONE,
            },
        ]);
        assert_eq!(
            h.project.setups[0].operations[0].name,
            "Roughing Pocket Pass"
        );
        assert_eq!(
            h.history.undo(&mut h.project).as_deref(),
            Some("Rename Operation")
        );
        assert_eq!(h.project.setups[0].operations[0].name, "Adaptive Rough");

        // 4. Right-click Setup -> Duplicate Setup -> atomic Undo
        let (shapes, _) = h.step(Vec::new());
        let setup_pos = find_text_rect(&shapes, "Setup 1 — Top Op")
            .expect("setup row")
            .center();
        let _ = h.step(click_btn(setup_pos, egui::PointerButton::Secondary));
        let (menu_shapes, _) = h.step(Vec::new());
        let dup_pos = find_text_rect(&menu_shapes, "Duplicate Setup")
            .expect("Duplicate Setup item")
            .center();
        let _ = h.step(click_btn(dup_pos, egui::PointerButton::Primary));
        assert_eq!(h.project.setups.len(), 2);
        assert_eq!(
            h.history.undo(&mut h.project).as_deref(),
            Some("Duplicate Setup")
        );
        assert_eq!(h.project.setups.len(), 1);

        // 5. Expand Project Tools, then right-click Project Tool -> Sync from Machine Library -> marks referencing op Dirty + Undo
        h.project.setups[0].operations[0].runtime_status = OperationStatus::Clean;
        let (shapes, _) = h.step(Vec::new());
        let tools_rect =
            find_text_rect_containing(&shapes, "Project Tools (").expect("tools folder label");
        let _ = h.step(click_btn(
            Pos2::new(tools_rect.left() - 26.5, tools_rect.center().y),
            egui::PointerButton::Primary,
        ));
        let (shapes, _) = h.step(Vec::new());
        let tool_pos = find_text_rect_containing(&shapes, "T1 -")
            .expect("tool row")
            .center();
        let _ = h.step(click_btn(tool_pos, egui::PointerButton::Secondary));
        let (menu_shapes, _) = h.step(Vec::new());
        let sync_pos = find_text_rect(&menu_shapes, "Sync from Machine Library")
            .expect("Sync from Machine Library item")
            .center();
        let _ = h.step(click_btn(sync_pos, egui::PointerButton::Primary));
        assert_eq!(
            h.project.setups[0].operations[0].runtime_status,
            OperationStatus::Dirty
        );
        assert_eq!(
            h.history.undo(&mut h.project).as_deref(),
            Some("Sync Tool from Machine Library")
        );

        // 6. Right-click Fixture -> Duplicate -> atomic Undo + dirties setup ops
        h.project.setups[0].operations[0].runtime_status = OperationStatus::Clean;
        let (shapes, _) = h.step(Vec::new());
        let fix_pos = find_text_rect(&shapes, "Kurt 4in Vise")
            .expect("fixture row")
            .center();
        let _ = h.step(click_btn(fix_pos, egui::PointerButton::Secondary));
        let (menu_shapes, _) = h.step(Vec::new());
        let dup_fix_pos = find_text_rect(&menu_shapes, "Duplicate")
            .expect("Fixture Duplicate item")
            .center();
        let _ = h.step(click_btn(dup_fix_pos, egui::PointerButton::Primary));
        assert_eq!(h.project.setups[0].fixtures.len(), 2);
        assert_eq!(
            h.project.setups[0].operations[0].runtime_status,
            OperationStatus::Dirty
        );
        assert_eq!(
            h.history.undo(&mut h.project).as_deref(),
            Some("Duplicate Fixture")
        );
        assert_eq!(h.project.setups[0].fixtures.len(), 1);

        // 7. Right-click Project Root -> Collapse All hides child nodes; Expand All restores them
        let (shapes, _) = h.step(Vec::new());
        let root_pos = find_text_rect(&shapes, "Aluminum Manifold Rev B")
            .expect("root row")
            .center();
        let _ = h.step(click_btn(root_pos, egui::PointerButton::Secondary));
        let (menu_shapes, _) = h.step(Vec::new());
        let collapse_pos = find_text_rect(&menu_shapes, "Collapse All")
            .expect("Collapse All item")
            .center();
        let _ = h.step(click_btn(collapse_pos, egui::PointerButton::Primary));
        let (collapsed_shapes, _) = h.step(Vec::new());
        assert!(
            find_text_rect(&collapsed_shapes, "1. Adaptive Rough").is_none(),
            "Collapse All must collapse folder children"
        );
        let _ = h.step(click_btn(root_pos, egui::PointerButton::Secondary));
        let (menu_shapes, _) = h.step(Vec::new());
        let expand_pos = find_text_rect(&menu_shapes, "Expand All")
            .expect("Expand All item")
            .center();
        let _ = h.step(click_btn(expand_pos, egui::PointerButton::Primary));
        let (expanded_shapes, _) = h.step(Vec::new());
        assert!(
            find_text_rect(&expanded_shapes, "Outer Pocket").is_some(),
            "Expand All must expand nested Sketch children"
        );
    }

    #[test]
    fn formats_compact_cycle_time_cleanly() {
        assert_eq!(format_compact_cycle_time(0.2), "1s");
        assert_eq!(format_compact_cycle_time(42.4), "42s");
        assert_eq!(format_compact_cycle_time(72.0), "1m12s");
        assert_eq!(format_compact_cycle_time(3_840.0), "1h04m");
    }

    #[test]
    fn creating_default_thread_mill_op_auto_selects_compatible_thread_mill_cutter() {
        use chip_math::DisplayUnit;
        use chip_tools::{CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId};

        let mut project = ProjectDocument::new("thread_mill_test");
        let mut global_lib = chip_tools::default_starter_tool_library();
        let expected_tm_id = if let Some(existing) = global_lib
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::ThreadMill { .. }))
        {
            existing.id.clone()
        } else {
            let tid = ToolId("test_thread_mill_475".to_string());
            global_lib.tools.push(Tool {
                id: tid.clone(),
                tool_number: 15,
                length_offset_number: 15,
                diameter_offset_number: 15,
                name: "4.75mm Single-Point Thread Mill".to_string(),
                vendor: None,
                product_id: None,
                native_unit: DisplayUnit::Millimeter,
                spindle_direction: SpindleDirection::Cw,
                default_coolant: CoolantMode::AirBlast,
                stickout_length_mm: 22.0,
                geometry: CutterGeometry::ThreadMill {
                    diameter_mm: 4.75,
                    neck_diameter_mm: 2.85,
                    neck_length_mm: 16.0,
                    pitch_mm: 1.0,
                    teeth_count: 1,
                    thread_angle_deg: 60.0,
                    flute_length_mm: 1.2,
                    overall_length_mm: 50.0,
                    shank_diameter_mm: 6.0,
                    flutes: 3,
                },
                presets: Vec::new(),
            });
            tid
        };

        let o_idx = create_default_operation_in_setup(
            &mut project,
            &global_lib,
            0,
            OperationKind::ThreadMill,
        );
        assert_eq!(o_idx, Some(0));

        let op = &project.setups[0].operations[0];
        assert_eq!(op.kind, OperationKind::ThreadMill);
        assert_eq!(op.tool_id.as_ref(), Some(&expected_tm_id));
        let embedded = project.tool(&expected_tm_id);
        assert!(
            embedded.is_some_and(|t| matches!(t.geometry, CutterGeometry::ThreadMill { .. })),
            "expected compatible ThreadMill cutter to be embedded in project"
        );
        assert_eq!(
            default_pick_mode_for_op(0, 0, OperationKind::ThreadMill, OperationTab::Geometry),
            ViewportPickMode::OpCylindricalHoles {
                setup_idx: 0,
                op_idx: 0
            }
        );
    }

    #[test]
    fn browser_header_shows_project_title_without_redundant_root_buttons() {
        let mut h = BrowserTestHarness::new(ProjectDocument::new("Aluminum Manifold Rev B"));
        let (shapes, _) = h.step(Vec::new());

        let mut texts = Vec::new();
        for cs in &shapes {
            collect_texts(&cs.shape, &mut texts);
        }
        assert!(
            texts.iter().any(|t| t == "Aluminum Manifold Rev B"),
            "CAM Browser root must display the project title; got {texts:?}"
        );
        assert!(
            !texts.iter().any(|t| t == "+ Setup"),
            "CAM Browser root must not show redundant + Setup button (Setups folder '+' handles adding setups)"
        );
        assert!(
            !texts.iter().any(|t| t == "Rename"),
            "Inline Rename button must not appear in CAM Browser tree"
        );
    }

    #[test]
    fn project_tree_renders_full_hierarchy_and_persists_expand_collapse_across_frames() {
        for theme in AppTheme::ALL {
            let mut h = BrowserTestHarness::new(build_rich_test_project());
            apply_theme(&h.ctx, theme);

            let (shapes1, _) = h.step(Vec::new());
            let mut texts1 = Vec::new();
            for cs in &shapes1 {
                collect_texts(&cs.shape, &mut texts1);
            }
            let joined1 = texts1.join("\n");
            for expected in [
                "Aluminum Manifold Rev B",
                "Models (3)",
                "manifold.step",
                "relief.stl",
                "kurt_vise.step",
                "Sketches (1)",
                "Mounting Holes",
                "Keep-Out Regions (1)",
                "Strap Clamps",
                "Project Tools (1)",
                "Setups (1)",
                "Setup 1 — Top Op",
                "G54",
                "Active",
                "Fixtures (1)",
                "Kurt 4in Vise",
                "Operations (1)",
                "1. Adaptive Rough",
            ] {
                assert!(
                    joined1.contains(expected),
                    "{theme:?}: expected '{expected}' in tree output:\n{joined1}"
                );
            }
            assert!(
                !joined1.contains("Outer Pocket"),
                "{theme:?}: expected sketch entities collapsed by default"
            );

            let sk_rect = find_text_rect(&shapes1, "Mounting Holes").expect("sketch label rect");
            let chev_click_pos = Pos2::new(sk_rect.left() - 28.0, sk_rect.center().y);
            let _ = h.step(click_btn(chev_click_pos, egui::PointerButton::Primary));

            let (shapes2, _) = h.step(Vec::new());
            let mut texts2 = Vec::new();
            for cs in &shapes2 {
                collect_texts(&cs.shape, &mut texts2);
            }
            assert!(
                texts2.iter().any(|t| t == "Outer Pocket"),
                "{theme:?}: expanded Sketch entity must persist across frames; got {texts2:?}"
            );

            let ent_rect = find_text_rect(&shapes2, "Outer Pocket").expect("entity label rect");
            let _ = h.step(click_btn(ent_rect.center(), egui::PointerButton::Primary));
            assert_eq!(h.focus, InspectorFocus::Sketch { sketch_idx: 0 });
            assert_eq!(h.sel_ent, Some(0));

            let fix_rect = find_text_rect(&shapes2, "Kurt 4in Vise").expect("fixture label rect");
            let _ = h.step(click_btn(fix_rect.center(), egui::PointerButton::Primary));
            assert_eq!(h.focus, InspectorFocus::Setup { setup_idx: 0 });
            assert_eq!(h.setup_tab, SetupTab::Fixtures);
            assert_eq!(h.sel_fix, Some(0));

            // Collapsing Operations (1) hides child operations and persists across frames
            let ops_rect =
                find_text_rect(&shapes2, "Operations (1)").expect("operations folder label rect");
            let ops_chev_pos = Pos2::new(ops_rect.left() - 28.0, ops_rect.center().y);
            let _ = h.step(click_btn(ops_chev_pos, egui::PointerButton::Primary));
            let (shapes3, _) = h.step(Vec::new());
            assert!(
                find_text_rect(&shapes3, "1. Adaptive Rough").is_none(),
                "{theme:?}: collapsing Operations (1) must hide child operations"
            );
            let _ = h.step(click_btn(ops_chev_pos, egui::PointerButton::Primary));
            let (shapes4, _) = h.step(Vec::new());
            assert!(
                find_text_rect(&shapes4, "1. Adaptive Rough").is_some(),
                "{theme:?}: re-expanding Operations (1) must restore child operations"
            );
        }
    }

    #[test]
    fn keyboard_navigation_steps_expands_collapses_and_jumps_to_parent() {
        let root_id = Id::unique("n_root");
        let sk_folder_id = Id::unique("n_sk_folder");
        let sk_id = Id::unique("n_sk0");
        let ent_id = Id::unique("n_ent0");

        let collapsed_tree = vec![
            VisibleTreeNode {
                key: TreeNodeKey::Project,
                node_id: root_id,
                parent: None,
                has_children: true,
                is_open: true,
            },
            VisibleTreeNode {
                key: TreeNodeKey::SketchesFolder,
                node_id: sk_folder_id,
                parent: Some(TreeNodeKey::Project),
                has_children: true,
                is_open: true,
            },
            VisibleTreeNode {
                key: TreeNodeKey::Sketch(0),
                node_id: sk_id,
                parent: Some(TreeNodeKey::SketchesFolder),
                has_children: true,
                is_open: false,
            },
        ];

        assert_eq!(
            resolve_tree_keyboard_nav(
                &collapsed_tree,
                Some(&TreeNodeKey::Project),
                TreeNavKey::Down
            ),
            vec![TreeNavAction::Select(TreeNodeKey::SketchesFolder)]
        );
        assert_eq!(
            resolve_tree_keyboard_nav(
                &collapsed_tree,
                Some(&TreeNodeKey::Sketch(0)),
                TreeNavKey::Up
            ),
            vec![TreeNavAction::Select(TreeNodeKey::SketchesFolder)]
        );
        assert_eq!(
            resolve_tree_keyboard_nav(
                &collapsed_tree,
                Some(&TreeNodeKey::Sketch(0)),
                TreeNavKey::Right
            ),
            vec![TreeNavAction::SetOpen {
                node_id: sk_id,
                open: true
            }]
        );

        let expanded_tree = vec![
            collapsed_tree[0].clone(),
            collapsed_tree[1].clone(),
            VisibleTreeNode {
                key: TreeNodeKey::Sketch(0),
                node_id: sk_id,
                parent: Some(TreeNodeKey::SketchesFolder),
                has_children: true,
                is_open: true,
            },
            VisibleTreeNode {
                key: TreeNodeKey::SketchEntity {
                    sketch_idx: 0,
                    entity_idx: 0,
                },
                node_id: ent_id,
                parent: Some(TreeNodeKey::Sketch(0)),
                has_children: false,
                is_open: false,
            },
        ];

        assert_eq!(
            resolve_tree_keyboard_nav(
                &expanded_tree,
                Some(&TreeNodeKey::Sketch(0)),
                TreeNavKey::Right
            ),
            vec![TreeNavAction::Select(TreeNodeKey::SketchEntity {
                sketch_idx: 0,
                entity_idx: 0
            })]
        );
        assert_eq!(
            resolve_tree_keyboard_nav(
                &expanded_tree,
                Some(&TreeNodeKey::SketchEntity {
                    sketch_idx: 0,
                    entity_idx: 0
                }),
                TreeNavKey::Left
            ),
            vec![TreeNavAction::Select(TreeNodeKey::Sketch(0))]
        );
        assert_eq!(
            resolve_tree_keyboard_nav(
                &expanded_tree,
                Some(&TreeNodeKey::Sketch(0)),
                TreeNavKey::Left
            ),
            vec![TreeNavAction::SetOpen {
                node_id: sk_id,
                open: false
            }]
        );
        assert_eq!(
            resolve_tree_keyboard_nav(
                &expanded_tree,
                Some(&TreeNodeKey::Sketch(0)),
                TreeNavKey::Rename
            ),
            vec![TreeNavAction::Rename(TreeNodeKey::Sketch(0))]
        );
        assert_eq!(
            resolve_tree_keyboard_nav(
                &expanded_tree,
                Some(&TreeNodeKey::Sketch(0)),
                TreeNavKey::Delete
            ),
            vec![TreeNavAction::Delete(TreeNodeKey::Sketch(0))]
        );
    }

    #[test]
    fn headless_egui_keyboard_navigation_expands_and_selects_in_tree() {
        let mut h = BrowserTestHarness::new(build_rich_test_project());
        let (shapes, _) = h.step(Vec::new());
        let sk_pos = find_text_rect(&shapes, "Mounting Holes")
            .expect("sketch row")
            .center();
        let _ = h.step(click_btn(sk_pos, egui::PointerButton::Primary));
        assert_eq!(h.focus, InspectorFocus::Sketch { sketch_idx: 0 });
        assert_eq!(h.sel_ent, None);

        let key_evt = |key: egui::Key| {
            vec![egui::Event::Key {
                key,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::NONE,
            }]
        };

        let _ = h.step(key_evt(egui::Key::ArrowRight));
        let _ = h.step(key_evt(egui::Key::ArrowRight));
        assert_eq!(h.focus, InspectorFocus::Sketch { sketch_idx: 0 });
        assert_eq!(h.sel_ent, Some(0));

        let _ = h.step(key_evt(egui::Key::ArrowLeft));
        assert_eq!(h.focus, InspectorFocus::Sketch { sketch_idx: 0 });
        assert_eq!(h.sel_ent, None);

        let _ = h.step(key_evt(egui::Key::ArrowLeft));
        let (shapes_after, _) = h.step(Vec::new());
        let mut texts = Vec::new();
        for cs in &shapes_after {
            collect_texts(&cs.shape, &mut texts);
        }
        assert!(
            !texts.iter().any(|t| t == "Outer Pocket"),
            "Left arrow on open Sketch must collapse its children"
        );
    }
}
