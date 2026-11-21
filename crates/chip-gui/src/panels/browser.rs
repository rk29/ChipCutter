use chip_cad::CadDocument;
use chip_cam::{
    Operation, OperationKind, OperationStatus, RestMachiningMode, Setup, WorkOffsetSlot,
};
use chip_project::{ProjectDocument, ToolpathCache, UndoHistory, resolve_upstream_rest_op_indices};
use egui::{CornerRadius, CursorIcon, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};

use crate::{
    panels::{
        InspectorFocus, OperationTab, SetupTab, ViewportPickMode, diagnostics::format_cycle_time,
    },
    theme::ThemeTokens,
    widgets::{
        IconKind, StatusBadgeKind, inline_icon, section_card, small_icon_button, status_badge_icon,
    },
};

/// High-level UI actions emitted by the CAM Browser pane.
#[derive(Debug, Clone)]
pub enum BrowserAction {
    OpenStepDialog,
    OpenMeshImportModal { browse_file: bool },
    OpenVectorImportModal { browse_file: bool },
    ReloadStepModel,
    ReplaceStepDialog,
    OpenSampleGallery,
    NewSetupXySketch,
    StartPickSketchHostFace,
    OpenToolLibrary,
    AlignCameraToActiveSetup,
    GenerateSetup(usize),
    GenerateSingleOp { setup_idx: usize, op_id: String },
    Log(String),
}

/// Context references passed to [`show_cam_browser`].
pub struct CamBrowserCtx<'a> {
    pub project: &'a mut ProjectDocument,
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
}

#[derive(Debug, Clone, Copy)]
struct DraggedOpState {
    setup_idx: usize,
    op_idx: usize,
}

fn snapshot_rest_upstream_ids(setup: &Setup) -> Vec<(String, Vec<String>)> {
    setup
        .operations
        .iter()
        .enumerate()
        .filter(|(_, op)| {
            matches!(
                op.passes.rest_machining.mode,
                RestMachiningMode::FromPrecedingOperations { .. }
            )
        })
        .map(|(idx, op)| {
            let upstream_ids = resolve_upstream_rest_op_indices(setup, idx)
                .into_iter()
                .map(|u| setup.operations[u].id.clone())
                .collect();
            (op.id.clone(), upstream_ids)
        })
        .collect()
}

fn invalidate_changed_rest_dependents(setup: &mut Setup, before: &[(String, Vec<String>)]) {
    for idx in 0..setup.operations.len() {
        if !matches!(
            setup.operations[idx].passes.rest_machining.mode,
            RestMachiningMode::FromPrecedingOperations { .. }
        ) {
            continue;
        }
        let after_ids: Vec<String> = resolve_upstream_rest_op_indices(setup, idx)
            .into_iter()
            .map(|u| setup.operations[u].id.clone())
            .collect();
        let op_id = &setup.operations[idx].id;
        let unchanged = before
            .iter()
            .find(|(id, _)| id == op_id)
            .is_some_and(|(_, prev)| prev == &after_ids);
        if !unchanged {
            setup.operations[idx].mark_dirty();
        }
    }
}

/// Moves `setup.operations[from_idx]` to insertion slot `to_slot` (`0..=operations.len()`),
/// returning the new operation index if its position actually changed, and marking any affected
/// `FromPrecedingOperations` Rest Machining dependents `Dirty`.
pub fn move_operation_to_slot(setup: &mut Setup, from_idx: usize, to_slot: usize) -> Option<usize> {
    let len = setup.operations.len();
    if from_idx >= len || to_slot > len || to_slot == from_idx || to_slot == from_idx + 1 {
        return None;
    }
    let before_rest = snapshot_rest_upstream_ids(setup);
    let moved = setup.operations.remove(from_idx);
    let dst_idx = if to_slot > from_idx {
        to_slot - 1
    } else {
        to_slot
    };
    setup.operations.insert(dst_idx, moved);
    invalidate_changed_rest_dependents(setup, &before_rest);
    Some(dst_idx)
}

/// Duplicates `setup.operations[op_idx]` immediately below `op_idx`, returning the new index and
/// propagating Rest Machining dirty invalidation to downstream dependents.
pub fn duplicate_operation_in_setup(setup: &mut Setup, op_idx: usize) -> Option<usize> {
    if op_idx >= setup.operations.len() {
        return None;
    }
    let before_rest = snapshot_rest_upstream_ids(setup);
    let mut dup = setup.operations[op_idx].clone();
    dup.id = format!("{}_copy_{}", dup.id, setup.operations.len() + 1);
    dup.name = format!("{} (Copy)", dup.name);
    dup.mark_dirty();
    let new_idx = op_idx + 1;
    setup.operations.insert(new_idx, dup);
    invalidate_changed_rest_dependents(setup, &before_rest);
    Some(new_idx)
}

/// Removes `setup.operations[op_idx]`, propagating Rest Machining dirty invalidation to downstream
/// dependents.
pub fn delete_operation_in_setup(setup: &mut Setup, op_idx: usize) -> bool {
    if op_idx >= setup.operations.len() {
        return false;
    }
    let before_rest = snapshot_rest_upstream_ids(setup);
    setup.operations.remove(op_idx);
    invalidate_changed_rest_dependents(setup, &before_rest);
    true
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

fn format_operation_badge(
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
        (Some(tp), Some(sec)) => Some(format!("{tp} · {}", format_compact_cycle_time(sec))),
        (Some(tp), None) => Some(tp),
        (None, Some(sec)) => Some(format_compact_cycle_time(sec)),
        (None, None) => None,
    }
}

/// Renders the left-hand hierarchical **CAM Browser** (`08-ui-viewport.md`).
pub fn show_cam_browser(ui: &mut egui::Ui, ctx: CamBrowserCtx<'_>) -> Option<BrowserAction> {
    let CamBrowserCtx {
        project,
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
    } = ctx;
    let mut emitted_action = None;

    if !project.setups.is_empty() && *active_setup_idx >= project.setups.len() {
        *active_setup_idx = project.setups.len() - 1;
    }

    ui.horizontal(|ui| {
        let title_w = (ui.available_width() - 108.0).max(60.0);
        ui.add_sized(
            [title_w, 18.0],
            egui::Label::new(egui::RichText::new(&project.project_name).strong()).truncate(),
        )
        .on_hover_text(&project.project_name);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui
                .small_button("Tools")
                .on_hover_text("Open Tool Library Manager")
                .clicked()
            {
                emitted_action = Some(BrowserAction::OpenToolLibrary);
            }
            if ui
                .small_button("+ Setup")
                .on_hover_text("Add a new clamping Setup (e.g., Op 2 Flip)")
                .clicked()
            {
                history.record_before_edit("Add Setup", None, project);
                let next_n = project.setups.len() + 1;
                let slot = match next_n {
                    1 => WorkOffsetSlot::G54,
                    2 => WorkOffsetSlot::G55,
                    3 => WorkOffsetSlot::G56,
                    4 => WorkOffsetSlot::G57,
                    _ => WorkOffsetSlot::G58,
                };
                project.setups.push(Setup::new(
                    format!("setup_{next_n}"),
                    format!("Setup {next_n} — Op {next_n}"),
                    slot,
                ));
                let idx = project.setups.len() - 1;
                *active_setup_idx = idx;
                *focus = InspectorFocus::Setup { setup_idx: idx };
                *setup_tab = SetupTab::Wcs;
                *pick_mode = ViewportPickMode::InspectEntity;
                emitted_action = Some(BrowserAction::Log(format!("Created Setup {next_n}.")));
            }
        });
    });
    ui.separator();

    let drag_state_id = ui.scope_id().with("cam_browser_op_drag");
    let mut active_drag: Option<DraggedOpState> = ui
        .ctx()
        .data_mut(|d| d.get_temp::<DraggedOpState>(drag_state_id));

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            // 1. Models section (unified collapsible card matching right-side Inspector)
            section_card(ui, "browser_models_card", "Models", true, |ui| {
                if let Some(doc) = cad_doc {
                    let sel = matches!(
                        *focus,
                        InspectorFocus::Model { model_idx: 0 } | InspectorFocus::CadEntity
                    );
                    let has_xf = project
                        .models
                        .first()
                        .is_some_and(|m| !m.transform.is_identity());
                    let body_summary = if !doc.meshes.is_empty() && doc.bodies.is_empty() {
                        format!(
                            "• {} ({} mesh){}",
                            doc.source_name,
                            doc.meshes.len(),
                            if has_xf { " · Xf" } else { "" }
                        )
                    } else {
                        format!(
                            "• {} ({} body){}",
                            doc.source_name,
                            doc.bodies.len(),
                            if has_xf { " · Xf" } else { "" }
                        )
                    };
                    if ui
                        .selectable_label(sel, body_summary)
                        .on_hover_text(
                            "Click to open Model Transform, Orientation, & Scaling Inspector",
                        )
                        .clicked()
                    {
                        *focus = InspectorFocus::Model { model_idx: 0 };
                        *pick_mode = ViewportPickMode::InspectEntity;
                    }
                    ui.indent("model_stats", |ui| {
                        if let Some(mesh) = doc.meshes.first() {
                            let d = &mesh.diagnostics;
                            let status = if d.is_watertight {
                                "Watertight"
                            } else {
                                "Open edges"
                            };
                            ui.weak(format!(
                                "{} · {} tris · {} verts · {status}",
                                mesh.format.label(),
                                d.valid_triangle_count,
                                d.welded_vertex_count
                            ));
                        } else {
                            ui.weak(format!(
                                "{} faces · {} loops · {} edges",
                                doc.faces.len(),
                                doc.loops.len(),
                                doc.edges.len()
                            ));
                        }
                    });
                } else {
                    ui.weak("No 3D STEP or mesh model loaded.");
                }

                ui.add_space(2.0);
                ui.horizontal_wrapped(|ui| {
                    if cad_doc.is_some()
                        && ui
                            .small_button("Orient / Scale")
                            .on_hover_text(
                                "Rotate, lay flat on a face, scale, or move the loaded 3D model",
                            )
                            .clicked()
                    {
                        *focus = InspectorFocus::Model { model_idx: 0 };
                        *pick_mode = ViewportPickMode::InspectEntity;
                    }
                    if ui
                        .small_button("Import STEP…")
                        .on_hover_text("Import a 3D STEP (.step / .stp) B-Rep solid model")
                        .clicked()
                    {
                        emitted_action = Some(BrowserAction::OpenStepDialog);
                    }
                    if ui
                        .small_button("Import Mesh…")
                        .on_hover_text(
                            "Import a 3D triangle mesh (.stl, .obj, .3mf) with physical unit preview",
                        )
                        .clicked()
                    {
                        emitted_action =
                            Some(BrowserAction::OpenMeshImportModal { browse_file: true });
                    }
                    if cad_doc.is_some_and(|d| !d.bodies.is_empty()) {
                        if ui
                            .small_button("Reload (Ctrl+R)")
                            .on_hover_text(
                                "Re-read the active STEP model from disk and associatively re-match CAM operations",
                            )
                            .clicked()
                        {
                            emitted_action = Some(BrowserAction::ReloadStepModel);
                        }
                        if ui
                            .small_button("Replace…")
                            .on_hover_text(
                                "Pick a revised STEP file on disk and associatively re-bind Setups & Operations",
                            )
                            .clicked()
                        {
                            emitted_action = Some(BrowserAction::ReplaceStepDialog);
                        }
                    }
                    if ui
                        .small_button("Sample Gallery…")
                        .on_hover_text(
                            "Browse built-in 3D STEP, Mesh, 4th-Axis Rotary, Pure-2D, and V-Carve sample projects",
                        )
                        .clicked()
                    {
                        emitted_action = Some(BrowserAction::OpenSampleGallery);
                    }
                });
            });

            // 1b. 2D Sketches section (`11-sketches-vectors.md`) — Eyeball = viewport visibility!
            let sketches_title = format!("2D Sketches ({})", project.sketches.len());
            section_card(
                ui,
                "browser_sketches_card",
                &sketches_title,
                !project.sketches.is_empty(),
                |ui| {
                    if project.sketches.is_empty() {
                        ui.weak("No 2D vector sketches imported yet.");
                    } else {
                        let mut del_sketch: Option<usize> = None;
                        let mut vis_toggle: Option<usize> = None;
                        for (sk_idx, sk) in project.sketches.iter().enumerate() {
                            let is_sel = *focus == InspectorFocus::Sketch { sketch_idx: sk_idx };
                            ui.horizontal(|ui| {
                                ui.spacing_mut().item_spacing.x = 4.0;
                                if small_icon_button(
                                    ui,
                                    if sk.visible {
                                        IconKind::Eye
                                    } else {
                                        IconKind::EyeOff
                                    },
                                    if sk.visible {
                                        "Hide 2D Sketch in 3D viewport"
                                    } else {
                                        "Show 2D Sketch in 3D viewport"
                                    },
                                )
                                .clicked()
                                {
                                    vis_toggle = Some(sk_idx);
                                }
                                let lbl = format!(
                                    "{} ({} rgn, {} ent)",
                                    sk.name,
                                    sk.regions.len(),
                                    sk.entities.len()
                                );
                                let rich = if sk.visible {
                                    egui::RichText::new(lbl)
                                } else {
                                    egui::RichText::new(lbl).weak()
                                };
                                let sk_w = (ui.available_width() - 26.0).max(48.0);
                                if ui
                                    .add_sized(
                                        [sk_w, 20.0],
                                        egui::Button::selectable(is_sel, rich).truncate(),
                                    )
                                    .on_hover_text("Click to inspect or edit this 2D Sketch layer")
                                    .clicked()
                                {
                                    *focus = InspectorFocus::Sketch { sketch_idx: sk_idx };
                                    *pick_mode = ViewportPickMode::InspectEntity;
                                }
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        if small_icon_button(
                                            ui,
                                            IconKind::Trash,
                                            "Delete Sketch Layer",
                                        )
                                        .clicked()
                                        {
                                            del_sketch = Some(sk_idx);
                                        }
                                    },
                                );
                            });
                        }
                        if let Some(ti) = vis_toggle {
                            history.record_before_edit("Toggle Sketch Visibility", None, project);
                            project.sketches[ti].visible = !project.sketches[ti].visible;
                        }
                        if let Some(di) = del_sketch {
                            history.record_before_edit("Delete Sketch Layer", None, project);
                            project.sketches.remove(di);
                            if matches!(*focus, InspectorFocus::Sketch { .. }) {
                                *focus = InspectorFocus::Setup {
                                    setup_idx: *active_setup_idx,
                                };
                            }
                        }
                    }

                    ui.add_space(2.0);
                    ui.horizontal_wrapped(|ui| {
                        if ui
                            .small_button("+ New Sketch")
                            .on_hover_text("Create a new parametric 2D Sketch on Setup XY")
                            .clicked()
                        {
                            emitted_action = Some(BrowserAction::NewSetupXySketch);
                        }
                        if cad_doc.is_some()
                            && ui
                                .small_button("+ Sketch on Face")
                                .on_hover_text(
                                    "Click any planar 3D STEP face to anchor an associative 2D Sketch",
                                )
                                .clicked()
                        {
                            emitted_action = Some(BrowserAction::StartPickSketchHostFace);
                        }
                        if ui
                            .small_button("+ Import DXF/SVG…")
                            .on_hover_text("Import AutoCAD DXF or W3C SVG vector drawing")
                            .clicked()
                        {
                            emitted_action =
                                Some(BrowserAction::OpenVectorImportModal { browse_file: true });
                        }
                    });
                },
            );

            // 2. Project-Embedded Tools section
            let tools_title = format!("Project Tools ({})", project.tools.len());
            section_card(ui, "browser_tools_card", &tools_title, false, |ui| {
                if project.tools.is_empty() {
                    ui.weak("Tools are embedded automatically when assigned to operations.");
                } else {
                    for tool in &project.tools {
                        ui.label(format!(
                            "• T{} — {} ({})",
                            tool.tool_number,
                            tool.name,
                            tool.native_unit
                                .format_length(tool.geometry.cutting_diameter_mm())
                        ));
                    }
                }
                ui.add_space(2.0);
                if ui
                    .small_button("Manage Tool Library…")
                    .on_hover_text("Open the Tool Library manager to edit, add, or import tools")
                    .clicked()
                {
                    emitted_action = Some(BrowserAction::OpenToolLibrary);
                }
            });

            // 3. Setups & Operations tree (each Setup in a unified collapsible card)
            let mut deferred_setup_delete: Option<usize> = None;
            let mut deferred_op_add: Option<(usize, OperationKind)> = None;
            let mut deferred_adaptive_add: Option<usize> = None;
            let mut deferred_op_move_slot: Option<(usize, usize, usize)> = None;
            let mut deferred_op_delete: Option<(usize, usize)> = None;
            let mut deferred_op_duplicate: Option<(usize, usize)> = None;
            let mut deferred_op_toggle_suppress: Option<(usize, usize)> = None;
            let mut deferred_op_toggle_visibility: Option<String> = None;

            for s_idx in 0..project.setups.len() {
                let is_active_setup = *active_setup_idx == s_idx;
                let setup_header = {
                    let s = &project.setups[s_idx];
                    let slot = s.wcs_slot.short_code();
                    let base = if s.name.contains(slot) {
                        s.name.clone()
                    } else {
                        format!("{} [{slot}]", s.name)
                    };
                    if is_active_setup {
                        format!("• {base}")
                    } else {
                        base
                    }
                };

                section_card(ui, ("setup_hdr", s_idx), &setup_header, true, |ui| {
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 4.0;
                        let stock_selected = *focus == InspectorFocus::Setup { setup_idx: s_idx };
                        if ui
                            .selectable_label(stock_selected, "Stock & WCS")
                            .on_hover_text("Inspect and edit this Setup's WCS origin, orientation, stock, and post-processor")
                            .clicked()
                        {
                            *active_setup_idx = s_idx;
                            *focus = InspectorFocus::Setup { setup_idx: s_idx };
                            *pick_mode = ViewportPickMode::InspectEntity;
                        }
                        if ui
                            .small_button("Gen")
                            .on_hover_text("Generate dirty toolpaths in this Setup (Ctrl+G)")
                            .clicked()
                        {
                            *active_setup_idx = s_idx;
                            emitted_action = Some(BrowserAction::GenerateSetup(s_idx));
                        }
                        if ui
                            .small_button("Top")
                            .on_hover_text("Align camera straight down this Setup's +Z spindle axis")
                            .clicked()
                        {
                            *active_setup_idx = s_idx;
                            emitted_action = Some(BrowserAction::AlignCameraToActiveSetup);
                        }
                        if project.setups.len() > 1 {
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    if small_icon_button(ui, IconKind::Trash, "Delete Setup")
                                        .clicked()
                                    {
                                        deferred_setup_delete = Some(s_idx);
                                    }
                                },
                            );
                        }
                    });

                    // Streamlined single-row Operation creation bar
                    ui.horizontal_wrapped(|ui| {
                        ui.spacing_mut().item_spacing.x = 3.0;
                        if ui
                            .small_button("+ Adap")
                            .on_hover_text("Add 2D Adaptive / Trochoidal Clearing operation")
                            .clicked()
                        {
                            deferred_adaptive_add = Some(s_idx);
                        }
                        if ui
                            .small_button("+ Cont")
                            .on_hover_text("Add 2D Contour / Profile operation")
                            .clicked()
                        {
                            deferred_op_add = Some((s_idx, OperationKind::Contour2D));
                        }
                        if ui
                            .small_button("+ VCarve")
                            .on_hover_text("Add 3D V-Carve / Bevel Inlay operation")
                            .clicked()
                        {
                            deferred_op_add = Some((s_idx, OperationKind::VCarve));
                        }
                        ui.menu_button("+ Op…", |ui| {
                            for (lbl, kind) in [
                                ("Facing", OperationKind::Facing),
                                ("2D Pocket", OperationKind::Pocket2D),
                                ("Drill / Helical Bore", OperationKind::DrillBore),
                                ("Edge Chamfer / Trace", OperationKind::ChamferTrace),
                                ("3D Adaptive Roughing", OperationKind::Adaptive3D),
                                ("3D Waterline / Z-Level", OperationKind::Waterline3D),
                                ("3D Parallel Finishing", OperationKind::Parallel3D),
                                ("3D Constant-Scallop", OperationKind::Scallop3D),
                                ("3D Pencil Cleanup", OperationKind::Pencil3D),
                                ("4th-Axis Rotary Surfacing", OperationKind::Rotary2Linear),
                                (
                                    "Simultaneous 4-Axis (X+Y+Z+A)",
                                    OperationKind::Rotary4AxisSim,
                                ),
                            ] {
                                if ui.button(lbl).clicked() {
                                    deferred_op_add = Some((s_idx, kind));
                                    ui.close();
                                }
                            }
                        })
                        .response
                        .on_hover_text("Add another 2.5D, 3-axis surfacing, or 4th-axis rotary operation");
                    });

                    ui.add_space(2.0);

                    // Operations list: Checkbox = Active/Suppressed, Eyeball = Viewport Toolpath Visibility
                    let op_len = project.setups[s_idx].operations.len();
                    let mut row_rects: Vec<Rect> = Vec::with_capacity(op_len);
                    let tok = ThemeTokens::from_ui(ui);

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
                            (StatusBadgeKind::Suppressed, Some("Suppressed (unchecked)".to_string()))
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
                        let rich = if op.suppressed {
                            egui::RichText::new(label_txt).strikethrough().weak()
                        } else {
                            egui::RichText::new(label_txt)
                        };

                        let op_id_clone = op.id.clone();
                        let op_kind_val = op.kind;
                        let op_is_adap = op.is_adaptive_2d();
                        let is_sup = op.suppressed;
                        let is_hidden_in_vp = hidden_operation_ids.contains(&op_id_clone);
                        let is_needs_repick = !is_sup
                            && matches!(op.runtime_status, OperationStatus::NeedsRepick { .. });
                        let is_gen = !is_sup
                            && matches!(op.runtime_status, OperationStatus::Generating);

                        let row_h = 24.0;
                        let full_w = ui.available_width().max(110.0);
                        let preview_rect =
                            Rect::from_min_size(ui.cursor().min, Vec2::new(full_w, row_h));
                        let row_hovered =
                            active_drag.is_none() && ui.rect_contains_pointer(preview_rect);

                        let is_being_dragged =
                            active_drag.is_some_and(|d| d.setup_idx == s_idx && d.op_idx == o_idx);

                        let row_resp = ui.horizontal(|ui| {
                            ui.spacing_mut().item_spacing.x = 3.0;
                            // Checkbox = Active / Enabled (unchecked = suppressed)
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
                                deferred_op_toggle_suppress = Some((s_idx, o_idx));
                            }
                            if is_gen {
                                ui.add(egui::Spinner::new().size(13.0));
                            } else {
                                status_badge_icon(ui, status_kind, hover_note.as_deref());
                            }
                            inline_icon(
                                ui,
                                IconKind::for_operation(op_kind_val, op_is_adap),
                                if is_sup {
                                    tok.suppressed_fg
                                } else if is_sel {
                                    tok.accent
                                } else {
                                    tok.text_secondary
                                },
                                13.5,
                            );

                            // Measure exact `T# · <time>` pill width against card right edge
                            let measured_pill_w = badge_txt.as_ref().map(|b| {
                                ui.painter()
                                    .layout_no_wrap(
                                        b.clone(),
                                        egui::FontId::proportional(10.5),
                                        tok.text_muted,
                                    )
                                    .size()
                                    .x
                                    + 10.0
                            });
                            let visible_right = ui.clip_rect().right() - 12.0;
                            let right_w = if row_hovered {
                                66.0
                            } else if is_hidden_in_vp {
                                22.0 + measured_pill_w.unwrap_or(0.0)
                            } else {
                                measured_pill_w.unwrap_or(0.0)
                            };
                            let lbl_w =
                                (visible_right - ui.cursor().min.x - right_w - 4.0).max(36.0);
                            let mut resp = ui.add_sized(
                                [lbl_w, 22.0],
                                egui::Button::selectable(is_sel || is_being_dragged, rich)
                                    .sense(Sense::click_and_drag())
                                    .truncate(),
                            );
                            if let Some(tip) = hover_note {
                                resp = resp.on_hover_text(tip);
                            }
                            if resp.clicked() {
                                *active_setup_idx = s_idx;
                                *focus = InspectorFocus::Operation {
                                    setup_idx: s_idx,
                                    op_idx: o_idx,
                                };
                                if is_needs_repick {
                                    *op_tab = OperationTab::Geometry;
                                }
                                *pick_mode = default_pick_mode_for_op(
                                    s_idx,
                                    o_idx,
                                    op_kind_val,
                                    *op_tab,
                                );
                            }
                            if resp.drag_started() && op_len > 1 {
                                let st = DraggedOpState {
                                    setup_idx: s_idx,
                                    op_idx: o_idx,
                                };
                                active_drag = Some(st);
                                ui.ctx().data_mut(|d| d.insert_temp(drag_state_id, st));
                            }

                            resp.context_menu(|ui| {
                                if ui.button("Generate Toolpath").clicked() {
                                    emitted_action = Some(BrowserAction::GenerateSingleOp {
                                        setup_idx: s_idx,
                                        op_id: op_id_clone.clone(),
                                    });
                                    ui.close();
                                }
                                if ui.button("Duplicate Operation (Ctrl+D)").clicked() {
                                    deferred_op_duplicate = Some((s_idx, o_idx));
                                    ui.close();
                                }
                                ui.separator();
                                if ui
                                    .add_enabled(
                                        o_idx > 0,
                                        egui::Button::new("Move Up (Ctrl+Up)"),
                                    )
                                    .clicked()
                                {
                                    deferred_op_move_slot = Some((s_idx, o_idx, o_idx - 1));
                                    ui.close();
                                }
                                if ui
                                    .add_enabled(
                                        o_idx + 1 < op_len,
                                        egui::Button::new("Move Down (Ctrl+Down)"),
                                    )
                                    .clicked()
                                {
                                    deferred_op_move_slot = Some((s_idx, o_idx, o_idx + 2));
                                    ui.close();
                                }
                                ui.separator();
                                if ui
                                    .button(if is_hidden_in_vp {
                                        "Show Toolpath in Viewport"
                                    } else {
                                        "Hide Toolpath in Viewport"
                                    })
                                    .clicked()
                                {
                                    deferred_op_toggle_visibility = Some(op_id_clone.clone());
                                    ui.close();
                                }
                                if ui
                                    .button(if is_sup {
                                        "Enable Operation"
                                    } else {
                                        "Suppress Operation"
                                    })
                                    .clicked()
                                {
                                    deferred_op_toggle_suppress = Some((s_idx, o_idx));
                                    ui.close();
                                }
                                if ui.button("Delete Operation (Del)").clicked() {
                                    deferred_op_delete = Some((s_idx, o_idx));
                                    ui.close();
                                }
                            });

                            if row_hovered {
                                ui.spacing_mut().item_spacing.x = 2.0;
                                // Eyeball = Viewport Toolpath Visibility!
                                if small_icon_button(
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
                                    deferred_op_toggle_visibility = Some(op_id_clone.clone());
                                }
                                if small_icon_button(
                                    ui,
                                    IconKind::Duplicate,
                                    "Duplicate Operation (Ctrl+D)",
                                )
                                .clicked()
                                {
                                    deferred_op_duplicate = Some((s_idx, o_idx));
                                }
                                if small_icon_button(
                                    ui,
                                    IconKind::Trash,
                                    "Delete Operation (Del)",
                                )
                                .clicked()
                                {
                                    deferred_op_delete = Some((s_idx, o_idx));
                                }
                            } else {
                                if is_hidden_in_vp
                                    && small_icon_button(
                                        ui,
                                        IconKind::EyeOff,
                                        "Toolpath hidden in 3D viewport — click to show",
                                    )
                                    .clicked()
                                {
                                    deferred_op_toggle_visibility = Some(op_id_clone.clone());
                                }
                                if let (Some(badge), Some(pill_w)) = (&badge_txt, measured_pill_w) {
                                    let (p_rect, _) = ui.allocate_exact_size(
                                        Vec2::new(pill_w, 17.0),
                                        Sense::hover(),
                                    );
                                    if ui.is_rect_visible(p_rect) {
                                        ui.painter().rect(
                                            p_rect,
                                            CornerRadius::same(4),
                                            tok.elevated_bg,
                                            Stroke::new(1.0, tok.border),
                                            StrokeKind::Inside,
                                        );
                                        ui.painter().text(
                                            p_rect.center(),
                                            egui::Align2::CENTER_CENTER,
                                            badge,
                                            egui::FontId::proportional(10.5),
                                            tok.text_muted,
                                        );
                                    }
                                }
                            }
                        });

                        row_rects.push(row_resp.response.rect);
                    }

                    // Handle live drag-and-drop insertion indicator & drop within this Setup
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
                                let x_left = row_rects[0].left() + 4.0;
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
                                deferred_op_move_slot = Some((s_idx, drag.op_idx, slot));
                                active_drag = None;
                                ui.ctx().data_mut(|d| d.remove::<DraggedOpState>(drag_state_id));
                            }
                        }
                    }
                });
            }

            if let Some(vis_op_id) = deferred_op_toggle_visibility
                && !hidden_operation_ids.remove(&vis_op_id)
            {
                hidden_operation_ids.insert(vis_op_id);
            }

            if active_drag.is_some() && ui.input(|i| i.pointer.any_released()) {
                ui.ctx().data_mut(|d| d.remove::<DraggedOpState>(drag_state_id));
            }

            if let Some(del_idx) = deferred_setup_delete {
                history.record_before_edit("Delete Setup", None, project);
                project.setups.remove(del_idx);
                *active_setup_idx = (*active_setup_idx).min(project.setups.len().saturating_sub(1));
                *focus = InspectorFocus::Setup {
                    setup_idx: *active_setup_idx,
                };
            }

            if let Some(s_idx) = deferred_adaptive_add {
                history.record_before_edit("Add 2D Adaptive Clearing", None, project);
                let next_op_num = project.setups[s_idx].operations.len() + 1;
                let mut op = Operation::new_adaptive_2d(
                    format!("op_{s_idx}_{next_op_num}_adaptive"),
                    format!("2D Adaptive {next_op_num}"),
                );
                if let Some(first_compat) = project
                    .tools
                    .iter()
                    .find(|t| OperationKind::Pocket2D.is_compatible_with_tool(t))
                {
                    op.tool_id = Some(first_compat.id.clone());
                    op.preset_id = first_compat.presets.first().map(|p| p.id.clone());
                }
                project.setups[s_idx].operations.push(op);
                let o_idx = project.setups[s_idx].operations.len() - 1;
                *active_setup_idx = s_idx;
                *focus = InspectorFocus::Operation {
                    setup_idx: s_idx,
                    op_idx: o_idx,
                };
                *op_tab = OperationTab::Geometry;
                *pick_mode =
                    default_pick_mode_for_op(s_idx, o_idx, OperationKind::Pocket2D, *op_tab);
            }

            if let Some((s_idx, kind)) = deferred_op_add {
                history.record_before_edit(format!("Add {}", kind.label()), None, project);
                let next_op_num = project.setups[s_idx].operations.len() + 1;
                let mut op = Operation::new(
                    format!(
                        "op_{s_idx}_{next_op_num}_{}",
                        project.setups[s_idx].operations.len()
                    ),
                    format!("{} {}", kind.default_name(), next_op_num),
                    kind,
                );
                if let Some(first_compat) = project
                    .tools
                    .iter()
                    .find(|t| kind.is_compatible_with_tool(t))
                {
                    op.tool_id = Some(first_compat.id.clone());
                    op.preset_id = first_compat.presets.first().map(|p| p.id.clone());
                }
                project.setups[s_idx].operations.push(op);
                let o_idx = project.setups[s_idx].operations.len() - 1;
                *active_setup_idx = s_idx;
                *focus = InspectorFocus::Operation {
                    setup_idx: s_idx,
                    op_idx: o_idx,
                };
                *op_tab = if kind == OperationKind::Facing {
                    OperationTab::Passes
                } else {
                    OperationTab::Geometry
                };
                *pick_mode = default_pick_mode_for_op(s_idx, o_idx, kind, *op_tab);
            }

            if let Some((s_idx, o_idx)) = deferred_op_duplicate {
                history.record_before_edit("Duplicate Operation", None, project);
                if let Some(new_idx) =
                    duplicate_operation_in_setup(&mut project.setups[s_idx], o_idx)
                {
                    *focus = InspectorFocus::Operation {
                        setup_idx: s_idx,
                        op_idx: new_idx,
                    };
                }
            }

            if let Some((s_idx, o_idx, slot)) = deferred_op_move_slot
                && slot != o_idx
                && slot != o_idx + 1
            {
                history.record_before_edit("Reorder Operation", None, project);
                if let Some(dst_idx) =
                    move_operation_to_slot(&mut project.setups[s_idx], o_idx, slot)
                {
                    *focus = InspectorFocus::Operation {
                        setup_idx: s_idx,
                        op_idx: dst_idx,
                    };
                }
            }

            if let Some((s_idx, o_idx)) = deferred_op_toggle_suppress {
                history.record_before_edit("Toggle Suppress Operation", None, project);
                let before_rest = snapshot_rest_upstream_ids(&project.setups[s_idx]);
                let op = &mut project.setups[s_idx].operations[o_idx];
                op.suppressed = !op.suppressed;
                invalidate_changed_rest_dependents(&mut project.setups[s_idx], &before_rest);
            }

            if let Some((s_idx, o_idx)) = deferred_op_delete {
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
                let _ = setup_tab;
            }
        });

    emitted_action
}

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
        OperationKind::Facing | OperationKind::Pocket2D | OperationKind::VCarve => {
            ViewportPickMode::OpPlanarFaces { setup_idx, op_idx }
        }
        OperationKind::Contour2D | OperationKind::ChamferTrace => {
            ViewportPickMode::OpClosedLoops { setup_idx, op_idx }
        }
        OperationKind::DrillBore => ViewportPickMode::OpCylindricalHoles { setup_idx, op_idx },
        OperationKind::Adaptive3D
        | OperationKind::Waterline3D
        | OperationKind::Parallel3D
        | OperationKind::Scallop3D
        | OperationKind::Pencil3D
        | OperationKind::Rotary2Linear
        | OperationKind::Rotary4AxisSim => ViewportPickMode::InspectEntity,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

        // No-op drag onto own slot returns None and preserves Clean
        assert_eq!(move_operation_to_slot(&mut setup, 1, 1), None);
        assert_eq!(move_operation_to_slot(&mut setup, 1, 2), None);
        assert_eq!(setup.operations[2].runtime_status, OperationStatus::Clean);

        // Moving `op_2` AFTER `op_3` (slot 3) changes `op_3`'s upstream Rest coverage -> `op_3` becomes Dirty!
        let dst = move_operation_to_slot(&mut setup, 1, 3);
        assert_eq!(dst, Some(2));
        assert_eq!(setup.operations[0].id, "op_1");
        assert_eq!(setup.operations[1].id, "op_3");
        assert_eq!(setup.operations[2].id, "op_2");
        assert_eq!(setup.operations[1].runtime_status, OperationStatus::Dirty);

        // Reset Clean and test duplication before `op_3`
        setup.operations[1].runtime_status = OperationStatus::Clean;
        let dup_idx = duplicate_operation_in_setup(&mut setup, 0);
        assert_eq!(dup_idx, Some(1));
        assert_eq!(setup.operations.len(), 4);
        // `op_3` is now at index 2 and gained a new preceding op -> marked Dirty!
        assert_eq!(setup.operations[2].id, "op_3");
        assert_eq!(setup.operations[2].runtime_status, OperationStatus::Dirty);

        // Reset Clean and delete preceding duplicate -> `op_3` marked Dirty again
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
}
