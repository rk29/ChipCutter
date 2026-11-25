mod actions;
mod tree_widget;

use chip_cad::CadDocument;
use chip_cam::{OperationStatus, SharedAvoidRegion};
use chip_project::{ProjectDocument, ToolpathCache, UndoHistory};
use chip_tools::ToolLibrary;
use egui::{CursorIcon, Id, Pos2, Rect, Stroke};

use crate::{
    panels::{
        InspectorFocus, OperationTab, SetupTab, ViewportPickMode,
        diagnostics::format_cycle_time,
        operation_inspector::avoid_card::{
            mark_shared_avoid_dependents_dirty, next_shared_avoid_id_and_name,
        },
    },
    theme::ThemeTokens,
    widgets::{IconKind, StatusBadgeKind, small_icon_button, status_badge_icon},
};

pub use actions::{
    BrowserAction, create_default_operation_in_setup, default_pick_mode_for_op,
    delete_operation_in_setup, duplicate_operation_in_setup, format_compact_cycle_time,
    move_operation_to_slot,
};
pub use tree_widget::{
    TreeNavAction, TreeNavKey, TreeNodeKey, VisibleTreeNode, is_node_open,
    resolve_tree_keyboard_nav, set_node_open,
};

use actions::{
    add_new_setup, format_operation_badge, invalidate_changed_rest_dependents, model_display_name,
    snapshot_rest_upstream_ids,
};
use tree_widget::{
    TREE_INDENT_STEP, TreeRowSpec, poll_tree_nav_key, tree_badge_pill, tree_folder_menu_button,
    tree_hint_row, tree_row,
};

/// Context references passed to [`show_cam_browser`].
pub struct CamBrowserCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub global_tools: &'a ToolLibrary,
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

#[derive(Debug, Clone, Copy)]
struct DraggedOpState {
    setup_idx: usize,
    op_idx: usize,
}

#[derive(Default)]
struct DeferredBrowserMutations {
    add_setup: bool,
    sketch_vis_toggle: Option<usize>,
    sketch_delete: Option<usize>,
    sketch_entity_delete: Option<(usize, usize)>,
    avoid_add: bool,
    avoid_toggle_suppress: Option<usize>,
    avoid_duplicate: Option<usize>,
    avoid_delete: Option<String>,
    avoid_rename: Option<(usize, String)>,
    add_tool: Option<chip_tools::Tool>,
    remove_tool: Option<chip_tools::ToolId>,
    prune_unused_tools: bool,
    setup_delete: Option<usize>,
    fixture_toggle_enabled: Option<(usize, usize)>,
    fixture_delete: Option<(usize, usize)>,
    op_move_slot: Option<(usize, usize, usize)>,
    op_delete: Option<(usize, usize)>,
    op_duplicate: Option<(usize, usize)>,
    op_toggle_suppress: Option<(usize, usize)>,
    op_toggle_visibility: Option<String>,
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
        InspectorFocus::CadEntity => TreeNodeKey::Model(primary_model_idx),
        InspectorFocus::Model { model_idx } => TreeNodeKey::Model(model_idx),
        InspectorFocus::Sketch { sketch_idx } => match selected_sketch_entity_idx {
            Some(entity_idx) => TreeNodeKey::SketchEntity {
                sketch_idx,
                entity_idx,
            },
            None => TreeNodeKey::Sketch(sketch_idx),
        },
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
    let drag_state_id = tree_scope.with("op_drag");
    let nav_focus_id = tree_scope.with("kbd_focus_key");
    let nav_active_id = tree_scope.with("kbd_active");
    let rename_hook_id = tree_scope.with("pending_rename_key");
    let rename_id_key = ui.make_persistent_id("browser_avoid_renaming_id");
    let rename_buf_key = ui.make_persistent_id("browser_avoid_renaming_buf");

    let mut active_drag: Option<DraggedOpState> = ui
        .ctx()
        .data_mut(|d| d.get_temp::<DraggedOpState>(drag_state_id));
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
    let mut renaming_id: Option<String> = ui
        .data_mut(|d| d.get_temp::<Option<String>>(rename_id_key))
        .flatten();
    let mut renaming_buf: String = ui
        .data_mut(|d| d.get_temp::<String>(rename_buf_key))
        .unwrap_or_default();

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
                    selected: false,
                    keyboard_focused: tree_kbd_active
                        && kbd_focused_key.as_ref() == Some(&root_key),
                    draggable: false,
                    tooltip: Some("Project root (edit metadata via Edit -> Project Properties...)"),
                },
                |ui, _hovered| {
                    if ui
                        .small_button("+ Setup")
                        .on_hover_text("Add a new clamping Setup (e.g., Op 2 Flip)")
                        .clicked()
                    {
                        pending.add_setup = true;
                    }
                },
            );
            visible_nodes.push(VisibleTreeNode {
                key: root_key.clone(),
                node_id: root_id,
                parent: None,
                has_children: true,
                is_open: root_out.is_open,
            });
            if root_out.response.clicked() || root_out.chevron_toggled {
                kbd_focused_key = Some(root_key.clone());
                any_row_clicked = true;
            }

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
                            if ui.button("Import STEP…").clicked() {
                                emitted_action = Some(BrowserAction::OpenStepDialog);
                                ui.close();
                            }
                            if ui.button("Import Mesh (.stl / .obj / .3mf)…").clicked() {
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
                                if ui.button("Replace STEP…").clicked() {
                                    emitted_action = Some(BrowserAction::ReplaceStepDialog);
                                    ui.close();
                                }
                            }
                            ui.separator();
                            if ui.button("Sample Gallery…").clicked() {
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
                                    "{} · {} tris · {} verts · {status}",
                                    mesh.format.label(),
                                    mesh.diagnostics.valid_triangle_count,
                                    mesh.diagnostics.welded_vertex_count
                                )
                            } else {
                                format!(
                                    "{} · {} faces · {} loops · {} edges",
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
                    default_open: true,
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
                                && ui.button("New 2D Sketch on 3D Face…").clicked()
                            {
                                emitted_action = Some(BrowserAction::StartPickSketchHostFace);
                                ui.close();
                            }
                            ui.separator();
                            if ui.button("Import DXF / SVG…").clicked() {
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

            if sketches_out.is_open {
                if project.sketches.is_empty() {
                    tree_hint_row(ui, 2, "No 2D sketches yet");
                } else {
                    for (sk_idx, sk) in project.sketches.iter().enumerate() {
                        let sk_key = TreeNodeKey::Sketch(sk_idx);
                        let sk_id = node_id_for(tree_scope, &sk_key);
                        let has_entities = !sk.entities.is_empty();
                        let is_sketch_focused =
                            *focus == InspectorFocus::Sketch { sketch_idx: sk_idx };
                        let sk_open_preview =
                            has_entities && is_node_open(ui, sk_id, false);
                        let is_sk_row_selected = is_sketch_focused
                            && (!sk_open_preview || selected_sketch_entity_idx.is_none());
                        let tip = format!(
                            "{} ({} region(s), {} entity/entities)",
                            sk.name,
                            sk.regions.len(),
                            sk.entities.len()
                        );
                        let sk_visible = sk.visible;
                        let sk_out = tree_row(
                            ui,
                            TreeRowSpec {
                                node_id: sk_id,
                                depth: 2,
                                icon: IconKind::Sketch2D,
                                icon_color: if !sk_visible {
                                    tok.suppressed_fg
                                } else if is_sk_row_selected {
                                    tok.accent
                                } else {
                                    tok.text_secondary
                                },
                                label: &sk.name,
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
                                selected: is_sk_row_selected,
                                keyboard_focused: tree_kbd_active
                                    && kbd_focused_key.as_ref() == Some(&sk_key),
                                draggable: false,
                                tooltip: Some(&tip),
                            },
                            |ui, hovered| {
                                if hovered
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
                        visible_nodes.push(VisibleTreeNode {
                            key: sk_key.clone(),
                            node_id: sk_id,
                            parent: Some(sketches_key.clone()),
                            has_children: has_entities,
                            is_open: sk_out.is_open,
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

                        if sk_out.is_open {
                            for (ent_idx, ent) in sk.entities.iter().enumerate() {
                                let ent_key = TreeNodeKey::SketchEntity {
                                    sketch_idx: sk_idx,
                                    entity_idx: ent_idx,
                                };
                                let ent_id = node_id_for(tree_scope, &ent_key);
                                let is_ent_sel = is_sketch_focused
                                    && *selected_sketch_entity_idx == Some(ent_idx);
                                let ent_icon = IconKind::for_sketch_entity(&ent.entity);
                                let ent_tip = format!(
                                    "{} ({})",
                                    ent.name,
                                    ent.entity.kind_label()
                                );
                                let ent_out = tree_row(
                                    ui,
                                    TreeRowSpec {
                                        node_id: ent_id,
                                        depth: 3,
                                        icon: ent_icon,
                                        icon_color: if is_ent_sel {
                                            tok.accent
                                        } else if ent.is_construction {
                                            tok.text_muted
                                        } else {
                                            tok.text_secondary
                                        },
                                        label: &ent.name,
                                        label_color: if ent.is_construction {
                                            tok.text_muted
                                        } else {
                                            tok.text_primary
                                        },
                                        bold: false,
                                        strikethrough: false,
                                        italic: ent.is_construction,
                                        has_children: false,
                                        default_open: false,
                                        selected: is_ent_sel,
                                        keyboard_focused: tree_kbd_active
                                            && kbd_focused_key.as_ref() == Some(&ent_key),
                                        draggable: false,
                                        tooltip: Some(&ent_tip),
                                    },
                                    |ui, _hovered| {
                                        if ent.is_construction {
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
                            }
                        }
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

            if avoid_out.is_open {
                if project.avoid_regions.is_empty() {
                    tree_hint_row(ui, 2, "No shared keep-out regions yet");
                } else {
                    for r_idx in 0..project.avoid_regions.len() {
                        let r = &project.avoid_regions[r_idx];
                        let rid = r.id.clone();
                        let rname = r.name.clone();
                        let is_sup = r.suppressed;
                        let is_editing = editing_shared_avoid_id.as_deref() == Some(rid.as_str());
                        let usage = project.avoid_region_usage_count(&rid);
                        let usage_txt = if usage == 1 {
                            "used by 1 op".to_string()
                        } else {
                            format!("used by {usage} ops")
                        };
                        let r_key = TreeNodeKey::AvoidRegion(r_idx);
                        let r_id = node_id_for(tree_scope, &r_key);

                        if renaming_id.as_deref() == Some(rid.as_str()) {
                            ui.horizontal(|ui| {
                                ui.add_space(TREE_INDENT_STEP * 2.0 + 20.0);
                                let resp = ui.add(
                                    egui::TextEdit::singleline(&mut renaming_buf)
                                        .desired_width((ui.available_width() - 62.0).max(60.0)),
                                );
                                let enter = resp.lost_focus()
                                    && ui.input(|i| i.key_pressed(egui::Key::Enter));
                                if ui.small_button("OK").clicked() || enter {
                                    let trimmed = renaming_buf.trim();
                                    if !trimmed.is_empty() && trimmed != rname {
                                        pending.avoid_rename = Some((r_idx, trimmed.to_string()));
                                    }
                                    renaming_id = None;
                                }
                                if ui.small_button("Esc").clicked()
                                    || (resp.lost_focus()
                                        && ui.input(|i| i.key_pressed(egui::Key::Escape)))
                                {
                                    renaming_id = None;
                                }
                            });
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
                            "{rname} ({item_count} item(s)) · {usage_txt} (double-click or F2 to rename)"
                        );
                        let r_out = tree_row(
                            ui,
                            TreeRowSpec {
                                node_id: r_id,
                                depth: 2,
                                icon: IconKind::KeepOutZone,
                                icon_color: if is_sup {
                                    tok.suppressed_fg
                                } else if is_editing {
                                    tok.error_fg
                                } else {
                                    tok.warn_fg
                                },
                                label: &rname,
                                label_color: if is_sup {
                                    tok.suppressed_fg
                                } else if is_editing {
                                    tok.error_fg
                                } else {
                                    tok.text_primary
                                },
                                bold: is_editing,
                                strikethrough: is_sup,
                                italic: false,
                                has_children: false,
                                default_open: false,
                                selected: is_editing,
                                keyboard_focused: tree_kbd_active
                                    && kbd_focused_key.as_ref() == Some(&r_key),
                                draggable: false,
                                tooltip: Some(&tip),
                            },
                            |ui, hovered| {
                                let mut active = !is_sup;
                                if ui
                                    .checkbox(&mut active, "")
                                    .on_hover_text(if is_sup {
                                        "Suppressed — check to re-enable this keep-out region"
                                    } else {
                                        "Active — uncheck to suppress this keep-out region"
                                    })
                                    .changed()
                                {
                                    pending.avoid_toggle_suppress = Some(r_idx);
                                }
                                if hovered {
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
                        visible_nodes.push(VisibleTreeNode {
                            key: r_key.clone(),
                            node_id: r_id,
                            parent: Some(avoid_key.clone()),
                            has_children: false,
                            is_open: false,
                        });
                        if r_out.response.clicked() {
                            *editing_shared_avoid_id = Some(rid.clone());
                            kbd_focused_key = Some(r_key);
                            any_row_clicked = true;
                        }
                        if r_out.response.double_clicked() {
                            renaming_id = Some(rid.clone());
                            renaming_buf = rname.clone();
                        }
                        r_out.response.context_menu(|ui| {
                            if ui.button("Rename").clicked() {
                                renaming_id = Some(rid.clone());
                                renaming_buf = rname.clone();
                                ui.close();
                            }
                            if ui
                                .button(if is_sup { "Enable" } else { "Suppress" })
                                .clicked()
                            {
                                pending.avoid_toggle_suppress = Some(r_idx);
                                ui.close();
                            }
                            if ui.button("Duplicate").clicked() {
                                pending.avoid_duplicate = Some(r_idx);
                                ui.close();
                            }
                            ui.separator();
                            if ui.button("Delete").clicked() {
                                pending.avoid_delete = Some(rid.clone());
                                ui.close();
                            }
                        });
                    }
                }
            }

            // -----------------------------------------------------------------
            // 4. Project Tools (N)
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
                    default_open: !project.tools.is_empty(),
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
                                        .hint_text("diameter, flat, ball, v-bit…"),
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
                            Some("Unused tools can be removed from the + menu"),
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

            if tools_out.is_open {
                if project.tools.is_empty() {
                    tree_hint_row(ui, 2, "No project tools embedded yet");
                } else {
                    for (t_idx, tool) in project.tools.iter().enumerate() {
                        let usages = usage_map.get(&tool.id);
                        let use_count = usages.map_or(0, Vec::len);
                        let t_key = TreeNodeKey::Tool(t_idx);
                        let t_id = node_id_for(tree_scope, &t_key);
                        let main_txt = format!("T{} · {}", tool.tool_number, tool.name);
                        let tip = if use_count == 0 {
                            format!("{} (unused)", tool.summary_label())
                        } else {
                            let names: Vec<&str> = usages
                                .into_iter()
                                .flatten()
                                .map(|u| u.op_name.as_str())
                                .collect();
                            format!("{} · used by {}", tool.summary_label(), names.join(", "))
                        };
                        let t_out = tree_row(
                            ui,
                            TreeRowSpec {
                                node_id: t_id,
                                depth: 2,
                                icon: IconKind::Tool,
                                icon_color: tok.text_secondary,
                                label: &main_txt,
                                label_color: tok.text_primary,
                                bold: false,
                                strikethrough: false,
                                italic: false,
                                has_children: false,
                                default_open: false,
                                selected: false,
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
                            kbd_focused_key = Some(t_key);
                            any_row_clicked = true;
                        }
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

            if setups_out.is_open {
                let setup_count = project.setups.len();
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
                    let setup_tip = if is_active_setup {
                        format!(
                            "{setup_name} [{wcs_code}] (Active Setup — new operations are added here)"
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
                            icon_color: if is_active_setup || is_setup_sel {
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
                            selected: is_setup_sel,
                            keyboard_focused: tree_kbd_active
                                && kbd_focused_key.as_ref() == Some(&s_key),
                            draggable: false,
                            tooltip: Some(&setup_tip),
                        },
                        |ui, hovered| {
                            if hovered {
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
                                    emitted_action = Some(BrowserAction::AlignCameraToActiveSetup);
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
                    visible_nodes.push(VisibleTreeNode {
                        key: s_key.clone(),
                        node_id: s_id,
                        parent: Some(setups_key.clone()),
                        has_children: true,
                        is_open: s_out.is_open,
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
                    s_out.response.context_menu(|ui| {
                        if ui.button("Set Active").clicked() {
                            *active_setup_idx = s_idx;
                            *focus = InspectorFocus::Setup { setup_idx: s_idx };
                            *pick_mode = ViewportPickMode::InspectEntity;
                            ui.close();
                        }
                        if ui.button("Generate Setup").clicked() {
                            *active_setup_idx = s_idx;
                            emitted_action = Some(BrowserAction::GenerateSetup(s_idx));
                            ui.close();
                        }
                        if ui.button("Align Camera to Top").clicked() {
                            *active_setup_idx = s_idx;
                            emitted_action = Some(BrowserAction::AlignCameraToActiveSetup);
                            ui.close();
                        }
                        ui.separator();
                        if ui
                            .add_enabled(setup_count > 1, egui::Button::new("Delete Setup"))
                            .clicked()
                        {
                            pending.setup_delete = Some(s_idx);
                            ui.close();
                        }
                    });

                    if !s_out.is_open {
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

                    if fix_folder_out.is_open {
                        for (f_idx, fix) in project.setups[s_idx].fixtures.iter().enumerate() {
                            let f_key = TreeNodeKey::Fixture {
                                setup_idx: s_idx,
                                fixture_idx: f_idx,
                            };
                            let f_id = node_id_for(tree_scope, &f_key);
                            let is_f_sel = *focus == InspectorFocus::Setup { setup_idx: s_idx }
                                && *setup_tab == SetupTab::Fixtures
                                && *selected_fixture_idx == Some(f_idx);
                            let f_enabled = fix.enabled;
                            let f_name = fix.name.clone();
                            let f_out = tree_row(
                                ui,
                                TreeRowSpec {
                                    node_id: f_id,
                                    depth: 4,
                                    icon: IconKind::Fixture,
                                    icon_color: if !f_enabled {
                                        tok.suppressed_fg
                                    } else if is_f_sel {
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
                                    selected: is_f_sel,
                                    keyboard_focused: tree_kbd_active
                                        && kbd_focused_key.as_ref() == Some(&f_key),
                                    draggable: false,
                                    tooltip: Some("Click to edit this workholding fixture"),
                                },
                                |ui, _hovered| {
                                    let mut enabled_box = f_enabled;
                                    if ui
                                        .checkbox(&mut enabled_box, "")
                                        .on_hover_text(if f_enabled {
                                            "Active fixture — uncheck to suppress from collision checks & rapid avoidance"
                                        } else {
                                            "Suppressed fixture — check to re-enable"
                                        })
                                        .changed()
                                    {
                                        pending.fixture_toggle_enabled = Some((s_idx, f_idx));
                                    }
                                },
                            );
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
                        }
                    }

                    // 5b. Setup -> Operations
                    let op_len = project.setups[s_idx].operations.len();
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

                        let est_sec = cached_tp.as_ref().map(|(_, tp)| {
                            tp.stats.total_est_time_sec
                                + tp.clearing_stage
                                    .as_ref()
                                    .map_or(0.0, |c| c.toolpath.stats.total_est_time_sec)
                        });

                        let (status_kind, hover_note) = if op.suppressed {
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
                                    Some("Dirty — press Ctrl+G to regenerate".to_string()),
                                ),
                                OperationStatus::Generating => (
                                    StatusBadgeKind::Dirty,
                                    Some("Generating on background thread…".to_string()),
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

                        let badge_txt = if op.suppressed {
                            None
                        } else {
                            format_operation_badge(project, op, est_sec)
                        };

                        let mode_tag = match (op.is_adaptive_2d(), op.has_rest_machining()) {
                            (true, true) => " · Adp+Rest",
                            (true, false) => "",
                            (false, true) => " · Rest",
                            (false, false) => "",
                        };
                        let label_txt = format!("{}. {}{mode_tag}", o_idx + 1, op.name);
                        let op_id_clone = op.id.clone();
                        let op_kind_val = op.kind;
                        let op_is_adap = op.is_adaptive_2d();
                        let is_sup = op.suppressed;
                        let is_hidden_in_vp = hidden_operation_ids.contains(&op_id_clone);
                        let is_needs_repick = !is_sup
                            && matches!(op.runtime_status, OperationStatus::NeedsRepick { .. });
                        let is_gen =
                            !is_sup && matches!(op.runtime_status, OperationStatus::Generating);
                        let is_being_dragged = active_drag
                            .is_some_and(|d| d.setup_idx == s_idx && d.op_idx == o_idx);

                        let op_key = TreeNodeKey::Operation {
                            setup_idx: s_idx,
                            op_idx: o_idx,
                        };
                        let op_node_id = node_id_for(tree_scope, &op_key);

                        let op_out = tree_row(
                            ui,
                            TreeRowSpec {
                                node_id: op_node_id,
                                depth: 3,
                                icon: IconKind::for_operation(op_kind_val, op_is_adap),
                                icon_color: if is_sup {
                                    tok.suppressed_fg
                                } else if is_sel {
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
                                // Right-to-left order renders visually left-to-right as:
                                // [Status] [T1 · 42s] [eye] [suppress]
                                let mut active = !is_sup;
                                if ui
                                    .checkbox(&mut active, "")
                                    .on_hover_text(if is_sup {
                                        "Suppressed — check to re-enable this operation for simulation & G-code"
                                    } else {
                                        "Active — uncheck to suppress this operation from simulation & G-code"
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
                            parent: Some(s_key.clone()),
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
                            let st = DraggedOpState {
                                setup_idx: s_idx,
                                op_idx: o_idx,
                            };
                            active_drag = Some(st);
                            ui.ctx().data_mut(|d| d.insert_temp(drag_state_id, st));
                        }

                        op_out.response.context_menu(|ui| {
                            if ui
                                .button("Generate Toolpath")
                                .on_hover_text("Compute or refresh the toolpath for this operation")
                                .clicked()
                            {
                                emitted_action = Some(BrowserAction::GenerateSingleOp {
                                    setup_idx: s_idx,
                                    op_id: op_id_clone.clone(),
                                });
                                ui.close();
                            }
                            if ui
                                .button("Duplicate Operation (Ctrl+D)")
                                .on_hover_text("Insert a copy of this operation directly beneath it")
                                .clicked()
                            {
                                pending.op_duplicate = Some((s_idx, o_idx));
                                ui.close();
                            }
                            ui.separator();
                            if ui
                                .add_enabled(o_idx > 0, egui::Button::new("Move Up (Ctrl+Up)"))
                                .on_hover_text(
                                    "Move this operation earlier in the Setup execution order",
                                )
                                .on_disabled_hover_text("Already at the top of the Setup")
                                .clicked()
                            {
                                pending.op_move_slot = Some((s_idx, o_idx, o_idx - 1));
                                ui.close();
                            }
                            if ui
                                .add_enabled(
                                    o_idx + 1 < op_len,
                                    egui::Button::new("Move Down (Ctrl+Down)"),
                                )
                                .on_hover_text(
                                    "Move this operation later in the Setup execution order",
                                )
                                .on_disabled_hover_text("Already at the bottom of the Setup")
                                .clicked()
                            {
                                pending.op_move_slot = Some((s_idx, o_idx, o_idx + 2));
                                ui.close();
                            }
                            ui.separator();
                            if ui
                                .button(if is_hidden_in_vp {
                                    "Show Toolpath in Viewport"
                                } else {
                                    "Hide Toolpath in Viewport"
                                })
                                .on_hover_text(
                                    "Toggle 3D viewport visibility of this operation's toolpath without suppressing simulation or G-code",
                                )
                                .clicked()
                            {
                                pending.op_toggle_visibility = Some(op_id_clone.clone());
                                ui.close();
                            }
                            if ui
                                .button(if is_sup {
                                    "Enable Operation"
                                } else {
                                    "Suppress Operation"
                                })
                                .on_hover_text(
                                    "Enable or suppress this operation in generation, simulation, and G-code posting",
                                )
                                .clicked()
                            {
                                pending.op_toggle_suppress = Some((s_idx, o_idx));
                                ui.close();
                            }
                            if ui
                                .button("Delete Operation (Del)")
                                .on_hover_text(
                                    "Permanently remove this operation from the Setup (undoable with Ctrl+Z)",
                                )
                                .clicked()
                            {
                                pending.op_delete = Some((s_idx, o_idx));
                                ui.close();
                            }
                        });

                        row_rects.push(op_out.row_rect);
                    }

                    // Live drag-and-drop insertion indicator & drop within this Setup
                    if let Some(drag) = active_drag
                        && drag.setup_idx == s_idx
                        && !row_rects.is_empty()
                    {
                        ui.ctx().set_cursor_icon(CursorIcon::Grabbing);
                        if let Some(ptr) = ui.input(|i| i.pointer.interact_pos()) {
                            let mut slot = op_len;
                            for (idx, r) in row_rects.iter().enumerate() {
                                if ptr.y < r.center().y {
                                    slot = idx;
                                    break;
                                }
                            }
                            if slot != drag.op_idx && slot != drag.op_idx + 1 {
                                let y_line = if slot == 0 {
                                    row_rects[0].top() - 1.0
                                } else if slot >= row_rects.len() {
                                    row_rects[row_rects.len() - 1].bottom() + 1.0
                                } else {
                                    0.5 * (row_rects[slot - 1].bottom() + row_rects[slot].top())
                                };
                                let x_left = row_rects[0].left() + TREE_INDENT_STEP * 3.0 + 4.0;
                                let x_right = row_rects[0].right() - 4.0;
                                ui.painter().line_segment(
                                    [Pos2::new(x_left, y_line), Pos2::new(x_right, y_line)],
                                    Stroke::new(2.0, tok.accent),
                                );
                                ui.painter().circle_filled(
                                    Pos2::new(x_left, y_line),
                                    3.2,
                                    tok.accent,
                                );
                            }
                            if ui.input(|i| i.pointer.any_released()) {
                                pending.op_move_slot = Some((s_idx, drag.op_idx, slot));
                                active_drag = None;
                                ui.ctx()
                                    .data_mut(|d| d.remove::<DraggedOpState>(drag_state_id));
                            }
                        }
                    }

                    if op_len == 0 {
                        tree_hint_row(ui, 3, "No operations yet");
                    }
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
                    ui.ctx()
                        .data_mut(|d| d.insert_temp(rename_hook_id, key.clone()));
                    if let TreeNodeKey::AvoidRegion(r_idx) = key
                        && r_idx < project.avoid_regions.len()
                    {
                        renaming_id = Some(project.avoid_regions[r_idx].id.clone());
                        renaming_buf = project.avoid_regions[r_idx].name.clone();
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

    // Persist temp UI states
    ui.data_mut(|d| {
        d.insert_temp(rename_id_key, renaming_id);
        d.insert_temp(rename_buf_key, renaming_buf);
    });
    ui.ctx().data_mut(|d| {
        if let Some(k) = kbd_focused_key {
            d.insert_temp(nav_focus_id, k);
        }
        d.insert_temp(nav_active_id, tree_kbd_active);
    });

    if active_drag.is_some() && ui.input(|i| i.pointer.any_released()) {
        ui.ctx()
            .data_mut(|d| d.remove::<DraggedOpState>(drag_state_id));
    }

    // -------------------------------------------------------------------------
    // Apply deferred mutations cleanly with atomic Undo snapshots
    // -------------------------------------------------------------------------
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

    if let Some(ti) = pending.sketch_vis_toggle
        && ti < project.sketches.len()
    {
        history.record_before_edit("Toggle Sketch Visibility", None, project);
        project.sketches[ti].visible = !project.sketches[ti].visible;
    }
    if let Some(di) = pending.sketch_delete
        && di < project.sketches.len()
    {
        history.record_before_edit("Delete Sketch Layer", None, project);
        project.sketches.remove(di);
        *selected_sketch_entity_idx = None;
        if matches!(*focus, InspectorFocus::Sketch { .. }) {
            *focus = InspectorFocus::Setup {
                setup_idx: *active_setup_idx,
            };
        }
    }
    if let Some((sk_idx, ent_idx)) = pending.sketch_entity_delete
        && sk_idx < project.sketches.len()
        && ent_idx < project.sketches[sk_idx].entities.len()
    {
        history.record_before_edit("Delete Sketch Entity", None, project);
        project.sketches[sk_idx].entities.remove(ent_idx);
        project.sketches[sk_idx].rebuild_regions(None);
        let rem = project.sketches[sk_idx].entities.len();
        *selected_sketch_entity_idx = if rem > 0 {
            Some(ent_idx.min(rem - 1))
        } else {
            None
        };
    }

    if pending.avoid_add {
        history.record_before_edit("Add Shared Keep-Out Region", None, project);
        let (id, name) = next_shared_avoid_id_and_name(project, None);
        project
            .avoid_regions
            .push(SharedAvoidRegion::new(id, name.clone()));
        emitted_action = Some(BrowserAction::Log(format!(
            "Created shared keep-out region '{name}'."
        )));
    }
    if let Some((r_idx, new_name)) = pending.avoid_rename
        && r_idx < project.avoid_regions.len()
    {
        history.record_before_edit("Rename Shared Keep-Out Region", None, project);
        project.avoid_regions[r_idx].name = new_name;
    }
    if let Some(r_idx) = pending.avoid_toggle_suppress
        && r_idx < project.avoid_regions.len()
    {
        history.record_before_edit("Toggle Shared Keep-Out Suppress", None, project);
        let rid = project.avoid_regions[r_idx].id.clone();
        project.avoid_regions[r_idx].suppressed = !project.avoid_regions[r_idx].suppressed;
        mark_shared_avoid_dependents_dirty(project, &rid);
    }
    if let Some(r_idx) = pending.avoid_duplicate
        && r_idx < project.avoid_regions.len()
    {
        history.record_before_edit("Duplicate Shared Keep-Out Region", None, project);
        let mut dup = project.avoid_regions[r_idx].clone();
        let copy_name = format!("{} (Copy)", dup.name);
        let (new_id, new_name) = next_shared_avoid_id_and_name(project, Some(&copy_name));
        dup.id = new_id;
        dup.name = new_name.clone();
        project.avoid_regions.insert(r_idx + 1, dup);
        emitted_action = Some(BrowserAction::Log(format!(
            "Duplicated shared keep-out region as '{new_name}'."
        )));
    }
    if let Some(del_id) = pending.avoid_delete {
        history.record_before_edit("Delete Shared Keep-Out Region", None, project);
        mark_shared_avoid_dependents_dirty(project, &del_id);
        if project.remove_avoid_region(&del_id) {
            if editing_shared_avoid_id.as_deref() == Some(del_id.as_str()) {
                *editing_shared_avoid_id = None;
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
    if let Some(rm_id) = pending.remove_tool {
        history.record_before_edit("Remove Project Tool", None, project);
        if project.remove_tool_if_unused(&rm_id) {
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
    }
    if let Some((s_idx, f_idx)) = pending.fixture_delete
        && s_idx < project.setups.len()
        && f_idx < project.setups[s_idx].fixtures.len()
    {
        history.record_before_edit("Delete Fixture", None, project);
        project.setups[s_idx].fixtures.remove(f_idx);
        let rem = project.setups[s_idx].fixtures.len();
        *selected_fixture_idx = if rem > 0 {
            Some(f_idx.min(rem - 1))
        } else {
            None
        };
    }

    if let Some(del_idx) = pending.setup_delete
        && project.setups.len() > 1
        && del_idx < project.setups.len()
    {
        history.record_before_edit("Delete Setup", None, project);
        project.setups.remove(del_idx);
        *active_setup_idx = (*active_setup_idx).min(project.setups.len().saturating_sub(1));
        *focus = InspectorFocus::Setup {
            setup_idx: *active_setup_idx,
        };
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
            *editing_shared_avoid_id = Some(project.avoid_regions[r_idx].id.clone());
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
    use chip_math::ParamLength;
    use chip_project::ModelEntry;
    use chip_sketch::{SketchEntity, SketchEntityEntry, SketchLayer};

    use crate::theme::{AppTheme, apply_theme};

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
    fn browser_header_shows_project_title_and_add_setup_without_inline_rename_button() {
        let ctx = egui::Context::default();
        let mut project = ProjectDocument::new("Aluminum Manifold Rev B");
        let global_tools = chip_tools::default_starter_tool_library();
        let mut history = UndoHistory::default();
        let cache = ToolpathCache::new(4);
        let mut active_setup_idx = 0;
        let mut focus = InspectorFocus::Setup { setup_idx: 0 };
        let mut setup_tab = SetupTab::Wcs;
        let mut op_tab = OperationTab::Geometry;
        let mut pick_mode = ViewportPickMode::InspectEntity;
        let mut hidden = std::collections::HashSet::new();
        let mut editing_avoid = None;
        let mut sel_ent = None;
        let mut sel_fix = None;

        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(320.0, 600.0))),
                ..Default::default()
            },
            |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    let _ = show_cam_browser(
                        ui,
                        CamBrowserCtx {
                            project: &mut project,
                            global_tools: &global_tools,
                            history: &mut history,
                            toolpath_cache: &cache,
                            cad_doc: None,
                            cad_revision: 0,
                            active_setup_idx: &mut active_setup_idx,
                            focus: &mut focus,
                            setup_tab: &mut setup_tab,
                            op_tab: &mut op_tab,
                            pick_mode: &mut pick_mode,
                            hidden_operation_ids: &mut hidden,
                            editing_shared_avoid_id: &mut editing_avoid,
                            selected_sketch_entity_idx: &mut sel_ent,
                            selected_fixture_idx: &mut sel_fix,
                        },
                    );
                });
            },
        );
        out.textures_delta.clear();

        let mut texts = Vec::new();
        for cs in &out.shapes {
            collect_texts(&cs.shape, &mut texts);
        }
        assert!(
            texts.iter().any(|t| t == "Aluminum Manifold Rev B"),
            "CAM Browser root must display the project title; got {texts:?}"
        );
        assert!(
            texts.iter().any(|t| t == "+ Setup"),
            "CAM Browser root must retain the + Setup button"
        );
        assert!(
            !texts.iter().any(|t| t == "Rename"),
            "Inline Rename button must not appear in CAM Browser tree"
        );
    }

    #[test]
    fn project_tree_renders_full_hierarchy_and_persists_expand_collapse_across_frames() {
        for theme in AppTheme::ALL {
            let ctx = egui::Context::default();
            apply_theme(&ctx, theme);
            let mut project = build_rich_test_project();
            let global_tools = chip_tools::default_starter_tool_library();
            let mut history = UndoHistory::default();
            let cache = ToolpathCache::new(4);
            let mut active_setup_idx = 0;
            let mut focus = InspectorFocus::Setup { setup_idx: 0 };
            let mut setup_tab = SetupTab::Wcs;
            let mut op_tab = OperationTab::Geometry;
            let mut pick_mode = ViewportPickMode::InspectEntity;
            let mut hidden = std::collections::HashSet::new();
            let mut editing_avoid = None;
            let mut sel_ent = None;
            let mut sel_fix = None;

            let mut run_frame = |raw: egui::RawInput,
                                 project: &mut ProjectDocument,
                                 active_setup_idx: &mut usize,
                                 focus: &mut InspectorFocus,
                                 setup_tab: &mut SetupTab,
                                 op_tab: &mut OperationTab,
                                 pick_mode: &mut ViewportPickMode,
                                 hidden: &mut std::collections::HashSet<String>,
                                 editing_avoid: &mut Option<String>,
                                 sel_ent: &mut Option<usize>,
                                 sel_fix: &mut Option<usize>|
             -> Vec<egui::epaint::ClippedShape> {
                let mut out = ctx.run_ui(raw, |ui| {
                    egui::CentralPanel::default().show(ui, |ui| {
                        let _ = show_cam_browser(
                            ui,
                            CamBrowserCtx {
                                project,
                                global_tools: &global_tools,
                                history: &mut history,
                                toolpath_cache: &cache,
                                cad_doc: None,
                                cad_revision: 0,
                                active_setup_idx,
                                focus,
                                setup_tab,
                                op_tab,
                                pick_mode,
                                hidden_operation_ids: hidden,
                                editing_shared_avoid_id: editing_avoid,
                                selected_sketch_entity_idx: sel_ent,
                                selected_fixture_idx: sel_fix,
                            },
                        );
                    });
                });
                out.textures_delta.clear();
                out.shapes
            };

            let raw = egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(340.0, 900.0))),
                ..Default::default()
            };
            let shapes1 = run_frame(
                raw.clone(),
                &mut project,
                &mut active_setup_idx,
                &mut focus,
                &mut setup_tab,
                &mut op_tab,
                &mut pick_mode,
                &mut hidden,
                &mut editing_avoid,
                &mut sel_ent,
                &mut sel_fix,
            );
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
                "1. Adaptive Rough",
            ] {
                assert!(
                    joined1.contains(expected),
                    "{theme:?}: expected '{expected}' in tree output:\n{joined1}"
                );
            }
            // Sketch entity children must start collapsed by default
            assert!(
                !joined1.contains("Outer Pocket"),
                "{theme:?}: expected sketch entities collapsed by default"
            );

            // Click the Sketch disclosure chevron to expand it
            let sk_rect = find_text_rect(&shapes1, "Mounting Holes").expect("sketch label rect");
            let chev_click_pos = Pos2::new(sk_rect.left() - 28.0, sk_rect.center().y);
            let click_raw = |pos: Pos2| egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(340.0, 900.0))),
                events: vec![
                    egui::Event::PointerMoved(pos),
                    egui::Event::PointerButton {
                        pos,
                        button: egui::PointerButton::Primary,
                        pressed: true,
                        modifiers: egui::Modifiers::NONE,
                    },
                    egui::Event::PointerButton {
                        pos,
                        button: egui::PointerButton::Primary,
                        pressed: false,
                        modifiers: egui::Modifiers::NONE,
                    },
                ],
                ..Default::default()
            };
            let _ = run_frame(
                click_raw(chev_click_pos),
                &mut project,
                &mut active_setup_idx,
                &mut focus,
                &mut setup_tab,
                &mut op_tab,
                &mut pick_mode,
                &mut hidden,
                &mut editing_avoid,
                &mut sel_ent,
                &mut sel_fix,
            );

            // Next frame with no input: expanded state must persist and show "Outer Pocket"
            let shapes2 = run_frame(
                raw.clone(),
                &mut project,
                &mut active_setup_idx,
                &mut focus,
                &mut setup_tab,
                &mut op_tab,
                &mut pick_mode,
                &mut hidden,
                &mut editing_avoid,
                &mut sel_ent,
                &mut sel_fix,
            );
            let mut texts2 = Vec::new();
            for cs in &shapes2 {
                collect_texts(&cs.shape, &mut texts2);
            }
            assert!(
                texts2.iter().any(|t| t == "Outer Pocket"),
                "{theme:?}: expanded Sketch entity must persist across frames; got {texts2:?}"
            );

            // Click the expanded Sketch Entity and verify InspectorFocus + selected_sketch_entity_idx
            let ent_rect = find_text_rect(&shapes2, "Outer Pocket").expect("entity label rect");
            let _ = run_frame(
                click_raw(ent_rect.center()),
                &mut project,
                &mut active_setup_idx,
                &mut focus,
                &mut setup_tab,
                &mut op_tab,
                &mut pick_mode,
                &mut hidden,
                &mut editing_avoid,
                &mut sel_ent,
                &mut sel_fix,
            );
            assert_eq!(focus, InspectorFocus::Sketch { sketch_idx: 0 });
            assert_eq!(sel_ent, Some(0));

            // Click the Setup Fixture and verify SetupTab::Fixtures + selected_fixture_idx
            let fix_rect = find_text_rect(&shapes2, "Kurt 4in Vise").expect("fixture label rect");
            let _ = run_frame(
                click_raw(fix_rect.center()),
                &mut project,
                &mut active_setup_idx,
                &mut focus,
                &mut setup_tab,
                &mut op_tab,
                &mut pick_mode,
                &mut hidden,
                &mut editing_avoid,
                &mut sel_ent,
                &mut sel_fix,
            );
            assert_eq!(focus, InspectorFocus::Setup { setup_idx: 0 });
            assert_eq!(setup_tab, SetupTab::Fixtures);
            assert_eq!(sel_fix, Some(0));
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

        // Down steps through visible rows
        assert_eq!(
            resolve_tree_keyboard_nav(
                &collapsed_tree,
                Some(&TreeNodeKey::Project),
                TreeNavKey::Down
            ),
            vec![TreeNavAction::Select(TreeNodeKey::SketchesFolder)]
        );
        // Up steps backward
        assert_eq!(
            resolve_tree_keyboard_nav(
                &collapsed_tree,
                Some(&TreeNodeKey::Sketch(0)),
                TreeNavKey::Up
            ),
            vec![TreeNavAction::Select(TreeNodeKey::SketchesFolder)]
        );
        // Right on closed parent expands it
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

        // Right on open parent jumps to first child
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
        // Left on leaf jumps to parent
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
        // Left on open parent collapses it
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
        // F2 & Delete emit Rename / Delete actions
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
        struct NavHarness {
            ctx: egui::Context,
            project: ProjectDocument,
            global_tools: ToolLibrary,
            history: UndoHistory,
            cache: ToolpathCache,
            active_setup_idx: usize,
            focus: InspectorFocus,
            setup_tab: SetupTab,
            op_tab: OperationTab,
            pick_mode: ViewportPickMode,
            hidden: std::collections::HashSet<String>,
            editing_avoid: Option<String>,
            sel_ent: Option<usize>,
            sel_fix: Option<usize>,
        }

        impl NavHarness {
            fn step(&mut self, events: Vec<egui::Event>) -> Vec<egui::epaint::ClippedShape> {
                let mut out = self.ctx.run_ui(
                    egui::RawInput {
                        screen_rect: Some(Rect::from_min_size(
                            Pos2::ZERO,
                            egui::vec2(340.0, 900.0),
                        )),
                        events,
                        ..Default::default()
                    },
                    |ui| {
                        egui::CentralPanel::default().show(ui, |ui| {
                            let _ = show_cam_browser(
                                ui,
                                CamBrowserCtx {
                                    project: &mut self.project,
                                    global_tools: &self.global_tools,
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
                out.shapes
            }
        }

        let mut h = NavHarness {
            ctx: egui::Context::default(),
            project: build_rich_test_project(),
            global_tools: chip_tools::default_starter_tool_library(),
            history: UndoHistory::default(),
            cache: ToolpathCache::new(4),
            active_setup_idx: 0,
            focus: InspectorFocus::Setup { setup_idx: 0 },
            setup_tab: SetupTab::Wcs,
            op_tab: OperationTab::Geometry,
            pick_mode: ViewportPickMode::InspectEntity,
            hidden: std::collections::HashSet::new(),
            editing_avoid: None,
            sel_ent: None,
            sel_fix: None,
        };

        let shapes = h.step(Vec::new());
        let sk_pos = find_text_rect(&shapes, "Mounting Holes")
            .expect("sketch row")
            .center();
        // Click sketch row to give tree keyboard focus
        let _ = h.step(vec![
            egui::Event::PointerMoved(sk_pos),
            egui::Event::PointerButton {
                pos: sk_pos,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: egui::Modifiers::NONE,
            },
            egui::Event::PointerButton {
                pos: sk_pos,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::NONE,
            },
        ]);
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

        // Right expands collapsed Sketch
        let _ = h.step(key_evt(egui::Key::ArrowRight));
        // Next frame renders child entity; Right again selects first child entity
        let _ = h.step(key_evt(egui::Key::ArrowRight));
        assert_eq!(h.focus, InspectorFocus::Sketch { sketch_idx: 0 });
        assert_eq!(h.sel_ent, Some(0));

        // Left jumps back to parent Sketch
        let _ = h.step(key_evt(egui::Key::ArrowLeft));
        assert_eq!(h.focus, InspectorFocus::Sketch { sketch_idx: 0 });
        assert_eq!(h.sel_ent, None);

        // Left again collapses the Sketch
        let _ = h.step(key_evt(egui::Key::ArrowLeft));
        let shapes_after = h.step(Vec::new());
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
