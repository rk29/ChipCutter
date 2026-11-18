use chip_cad::CadDocument;
use chip_cam::{Operation, OperationKind, OperationStatus, Setup, WorkOffsetSlot};
use chip_project::{ProjectDocument, ToolpathCache, UndoHistory};

use crate::panels::{
    InspectorFocus, OperationTab, SetupTab, ViewportPickMode, diagnostics::format_cycle_time,
};

/// High-level UI actions emitted by the CAM Browser pane.
#[derive(Debug, Clone)]
pub enum BrowserAction {
    OpenStepDialog,
    ReloadStepModel,
    ReplaceStepDialog,
    LoadMultiFeatureSample { tilted: bool },
    LoadCamTortureBlock,
    LoadRevisionDemoV1,
    ReloadRevisionDemoV2,
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
    } = ctx;
    let mut emitted_action = None;

    if !project.setups.is_empty() && *active_setup_idx >= project.setups.len() {
        *active_setup_idx = project.setups.len() - 1;
    }

    ui.horizontal(|ui| {
        ui.strong(&project.project_name);
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

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            // 1. Models section
            egui::CollapsingHeader::new("Models")
                .default_open(true)
                .show(ui, |ui| {
                    if let Some(doc) = cad_doc {
                        let sel = *focus == InspectorFocus::CadEntity;
                        if ui
                            .selectable_label(
                                sel,
                                format!("• {} ({} body)", doc.source_name, doc.bodies.len()),
                            )
                            .clicked()
                        {
                            *focus = InspectorFocus::CadEntity;
                            *pick_mode = ViewportPickMode::InspectEntity;
                        }
                        ui.indent("model_stats", |ui| {
                            ui.weak(format!(
                                "{} faces · {} loops · {} edges",
                                doc.faces.len(),
                                doc.loops.len(),
                                doc.edges.len()
                            ));
                        });
                    } else {
                        ui.weak("No STEP model loaded.");
                    }

                    ui.horizontal_wrapped(|ui| {
                        if ui.small_button("Import STEP…").clicked() {
                            emitted_action = Some(BrowserAction::OpenStepDialog);
                        }
                        if cad_doc.is_some() {
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
                            .small_button("CAM Torture")
                            .on_hover_text("Load M3 CAM Torture Block (island pocket, open step, Ø18 bore, 45° bevel)")
                            .clicked()
                        {
                            emitted_action = Some(BrowserAction::LoadCamTortureBlock);
                        }
                        if ui
                            .small_button("Bracket")
                            .on_hover_text("Load M2 prismatic bracket")
                            .clicked()
                        {
                            emitted_action =
                                Some(BrowserAction::LoadMultiFeatureSample { tilted: false });
                        }
                        if ui
                            .small_button("Tilted 30°")
                            .on_hover_text("Load 30°-tilted STEP bracket")
                            .clicked()
                        {
                            emitted_action =
                                Some(BrowserAction::LoadMultiFeatureSample { tilted: true });
                        }
                        if ui
                            .small_button("Rev v1")
                            .on_hover_text("Load Milestone 5 CAD Revision v1 demo job")
                            .clicked()
                        {
                            emitted_action = Some(BrowserAction::LoadRevisionDemoV1);
                        }
                        if ui
                            .small_button("-> Rev v2")
                            .on_hover_text("Associatively reload CAD Revision v2 (resized pocket, moved/deleted holes)")
                            .clicked()
                        {
                            emitted_action = Some(BrowserAction::ReloadRevisionDemoV2);
                        }
                    });
                });

            ui.add_space(4.0);

            // 2. Project-Embedded Tools section
            egui::CollapsingHeader::new(format!("Project Tools ({})", project.tools.len()))
                .default_open(false)
                .show(ui, |ui| {
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
                    if ui.small_button("Manage Tool Library…").clicked() {
                        emitted_action = Some(BrowserAction::OpenToolLibrary);
                    }
                });

            ui.add_space(6.0);

            // 3. Setups & Operations tree
            let mut deferred_setup_delete: Option<usize> = None;
            let mut deferred_op_add: Option<(usize, OperationKind)> = None;
            let mut deferred_op_move: Option<(usize, usize, isize)> = None;
            let mut deferred_op_delete: Option<(usize, usize)> = None;
            let mut deferred_op_duplicate: Option<(usize, usize)> = None;
            let mut deferred_op_toggle_suppress: Option<(usize, usize)> = None;

            for s_idx in 0..project.setups.len() {
                let is_active_setup = *active_setup_idx == s_idx;
                let setup_header = {
                    let s = &project.setups[s_idx];
                    let marker = if is_active_setup { "> " } else { "  " };
                    format!("{marker}{} [{}]", s.name, s.wcs_slot.short_code())
                };

                egui::CollapsingHeader::new(egui::RichText::new(setup_header).strong())
                    .id_salt(("setup_hdr", s_idx))
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.horizontal_wrapped(|ui| {
                            let stock_selected = *focus == InspectorFocus::Setup { setup_idx: s_idx };
                            if ui
                                .selectable_label(stock_selected, "Stock & WCS")
                                .clicked()
                            {
                                *active_setup_idx = s_idx;
                                *focus = InspectorFocus::Setup { setup_idx: s_idx };
                                *pick_mode = ViewportPickMode::InspectEntity;
                            }
                            if ui
                                .small_button("Gen (Ctrl+G)")
                                .on_hover_text("Generate dirty toolpaths in this Setup")
                                .clicked()
                            {
                                *active_setup_idx = s_idx;
                                emitted_action = Some(BrowserAction::GenerateSetup(s_idx));
                            }
                            if ui
                                .small_button("Top View")
                                .on_hover_text("Align camera straight down this Setup's +Z spindle axis")
                                .clicked()
                            {
                                *active_setup_idx = s_idx;
                                emitted_action = Some(BrowserAction::AlignCameraToActiveSetup);
                            }
                            if project.setups.len() > 1
                                && ui
                                    .small_button("Del")
                                    .on_hover_text("Delete Setup")
                                    .clicked()
                            {
                                deferred_setup_delete = Some(s_idx);
                            }
                        });

                        // Quick Add Operation bar
                        ui.horizontal_wrapped(|ui| {
                            ui.weak("+ Op:");
                            for (short, kind) in [
                                ("Face", OperationKind::Facing),
                                ("Contour", OperationKind::Contour2D),
                                ("Pocket", OperationKind::Pocket2D),
                                ("Drill/Bore", OperationKind::DrillBore),
                                ("Chamfer", OperationKind::ChamferTrace),
                            ] {
                                if ui.small_button(short).clicked() {
                                    deferred_op_add = Some((s_idx, kind));
                                }
                            }
                        });

                        // Operations list
                        let op_len = project.setups[s_idx].operations.len();
                        for o_idx in 0..op_len {
                            let is_sel = *focus
                                == InspectorFocus::Operation {
                                    setup_idx: s_idx,
                                    op_idx: o_idx,
                                };
                            let cached_tp = toolpath_cache
                                .get_for_operation(project, s_idx, o_idx, cad_revision);
                            let op = &project.setups[s_idx].operations[o_idx];

                            let (badge_str, badge_col, hover_note) = if op.suppressed {
                                ("[-]", egui::Color32::from_gray(120), Some("Suppressed".to_string()))
                            } else {
                                match &op.runtime_status {
                                    OperationStatus::Clean => (
                                        "[OK]",
                                        egui::Color32::from_rgb(86, 224, 134),
                                        cached_tp.as_ref().map(|(_, tp)| {
                                            format!(
                                                "Clean: {} passes, est. {}",
                                                tp.stats.pass_count,
                                                format_cycle_time(tp.stats.total_est_time_sec)
                                            )
                                        }),
                                    ),
                                    OperationStatus::Dirty => (
                                        "[*]",
                                        egui::Color32::from_rgb(245, 190, 66),
                                        Some("Dirty — press Ctrl+G to regenerate".to_string()),
                                    ),
                                    OperationStatus::Generating => (
                                        "..",
                                        egui::Color32::from_rgb(90, 190, 250),
                                        Some("Generating on background thread…".to_string()),
                                    ),
                                    OperationStatus::Error { message } => (
                                        "[!]",
                                        egui::Color32::from_rgb(248, 92, 92),
                                        Some(message.clone()),
                                    ),
                                    OperationStatus::NeedsRepick { message } => (
                                        "[?]",
                                        egui::Color32::from_rgb(255, 146, 72),
                                        Some(format!("Needs Re-Pick: {message}")),
                                    ),
                                }
                            };

                            let label_txt = format!("{}. {}", o_idx + 1, op.name);
                            let rich = if op.suppressed {
                                egui::RichText::new(label_txt).strikethrough().weak()
                            } else {
                                egui::RichText::new(label_txt)
                            };

                            let op_id_clone = op.id.clone();
                            let op_kind_val = op.kind;
                            let is_sup = op.suppressed;
                            let is_needs_repick = !is_sup
                                && matches!(op.runtime_status, OperationStatus::NeedsRepick { .. });
                            let is_gen = !is_sup
                                && matches!(op.runtime_status, OperationStatus::Generating);

                            ui.horizontal(|ui| {
                                if is_gen {
                                    ui.add(egui::Spinner::new().size(12.0));
                                } else {
                                    ui.colored_label(badge_col, badge_str);
                                }

                                let max_lbl_w = (ui.available_width() - 86.0).max(48.0);
                                let mut resp = ui.add_sized(
                                    [max_lbl_w, 18.0],
                                    egui::Button::selectable(is_sel, rich).truncate(),
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

                                resp.context_menu(|ui| {
                                    if ui.button("Generate Toolpath").clicked() {
                                        emitted_action = Some(BrowserAction::GenerateSingleOp {
                                            setup_idx: s_idx,
                                            op_id: op_id_clone.clone(),
                                        });
                                        ui.close();
                                    }
                                    if ui.button("Duplicate Operation").clicked() {
                                        deferred_op_duplicate = Some((s_idx, o_idx));
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
                                    if ui.button("Delete Operation").clicked() {
                                        deferred_op_delete = Some((s_idx, o_idx));
                                        ui.close();
                                    }
                                });

                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        if ui
                                            .small_button("×")
                                            .on_hover_text("Delete Operation")
                                            .clicked()
                                        {
                                            deferred_op_delete = Some((s_idx, o_idx));
                                        }
                                        if ui
                                            .small_button(if is_sup { "On" } else { "Off" })
                                            .on_hover_text("Toggle Suppressed")
                                            .clicked()
                                        {
                                            deferred_op_toggle_suppress = Some((s_idx, o_idx));
                                        }
                                        if o_idx + 1 < op_len && ui.small_button("v").clicked() {
                                            deferred_op_move = Some((s_idx, o_idx, 1));
                                        }
                                        if o_idx > 0 && ui.small_button("^").clicked() {
                                            deferred_op_move = Some((s_idx, o_idx, -1));
                                        }
                                    },
                                );
                            });
                        }
                    });
            }

            if let Some(del_idx) = deferred_setup_delete {
                history.record_before_edit("Delete Setup", None, project);
                project.setups.remove(del_idx);
                *active_setup_idx = (*active_setup_idx).min(project.setups.len().saturating_sub(1));
                *focus = InspectorFocus::Setup {
                    setup_idx: *active_setup_idx,
                };
            }

            if let Some((s_idx, kind)) = deferred_op_add {
                history.record_before_edit(format!("Add {}", kind.label()), None, project);
                let next_op_num = project.setups[s_idx].operations.len() + 1;
                let mut op = Operation::new(
                    format!("op_{s_idx}_{next_op_num}_{}", project.setups[s_idx].operations.len()),
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
                let mut dup = project.setups[s_idx].operations[o_idx].clone();
                dup.id = format!("{}_copy_{}", dup.id, project.setups[s_idx].operations.len() + 1);
                dup.name = format!("{} (Copy)", dup.name);
                dup.mark_dirty();
                project.setups[s_idx].operations.insert(o_idx + 1, dup);
                *focus = InspectorFocus::Operation {
                    setup_idx: s_idx,
                    op_idx: o_idx + 1,
                };
            }

            if let Some((s_idx, o_idx, dir)) = deferred_op_move {
                let target = (o_idx as isize + dir) as usize;
                history.record_before_edit("Reorder Operation", None, project);
                project.setups[s_idx].operations.swap(o_idx, target);
                *focus = InspectorFocus::Operation {
                    setup_idx: s_idx,
                    op_idx: target,
                };
            }

            if let Some((s_idx, o_idx)) = deferred_op_toggle_suppress {
                history.record_before_edit("Toggle Suppress Operation", None, project);
                let op = &mut project.setups[s_idx].operations[o_idx];
                op.suppressed = !op.suppressed;
            }

            if let Some((s_idx, o_idx)) = deferred_op_delete {
                history.record_before_edit("Delete Operation", None, project);
                project.setups[s_idx].operations.remove(o_idx);
                *focus = InspectorFocus::Setup { setup_idx: s_idx };
                *pick_mode = ViewportPickMode::InspectEntity;
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
        OperationKind::Facing | OperationKind::Pocket2D => {
            ViewportPickMode::OpPlanarFaces { setup_idx, op_idx }
        }
        OperationKind::Contour2D | OperationKind::ChamferTrace => {
            ViewportPickMode::OpClosedLoops { setup_idx, op_idx }
        }
        OperationKind::DrillBore => ViewportPickMode::OpCylindricalHoles { setup_idx, op_idx },
    }
}
