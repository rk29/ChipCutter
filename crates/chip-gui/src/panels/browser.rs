use chip_cad::CadDocument;
use chip_cam::{Operation, OperationKind, Setup, WorkOffsetSlot};
use chip_project::{ProjectDocument, UndoHistory};

use crate::panels::{InspectorFocus, OperationTab, SetupTab, ViewportPickMode};

/// High-level UI actions emitted by the CAM Browser pane.
#[derive(Debug, Clone)]
pub enum BrowserAction {
    OpenStepDialog,
    LoadMultiFeatureSample { tilted: bool },
    OpenToolLibrary,
    AlignCameraToActiveSetup,
    Log(String),
}

/// Renders the left-hand hierarchical **CAM Browser** (`08-ui-viewport.md`).
#[allow(clippy::too_many_arguments)]
pub fn show_cam_browser(
    ui: &mut egui::Ui,
    project: &mut ProjectDocument,
    history: &mut UndoHistory,
    cad_doc: Option<&CadDocument>,
    active_setup_idx: &mut usize,
    focus: &mut InspectorFocus,
    setup_tab: &mut SetupTab,
    op_tab: &mut OperationTab,
    pick_mode: &mut ViewportPickMode,
) -> Option<BrowserAction> {
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
                        if ui
                            .small_button("Sample Bracket")
                            .on_hover_text("Load prismatic bracket with 4x Ø6 holes, 2 coplanar pockets, and blind bore")
                            .clicked()
                        {
                            emitted_action =
                                Some(BrowserAction::LoadMultiFeatureSample { tilted: false });
                        }
                        if ui
                            .small_button("Tilted 30°")
                            .on_hover_text("Load 30°-tilted STEP bracket to test interactive WCS face/edge alignment")
                            .clicked()
                        {
                            emitted_action =
                                Some(BrowserAction::LoadMultiFeatureSample { tilted: true });
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
                        for (o_idx, op) in project.setups[s_idx].operations.iter().enumerate() {
                            let is_sel = *focus
                                == InspectorFocus::Operation {
                                    setup_idx: s_idx,
                                    op_idx: o_idx,
                                };
                            let badge = if op.suppressed {
                                "[-]"
                            } else {
                                op.runtime_status.badge_icon()
                            };
                            let label_txt = format!(
                                "{badge} {}. {} ({})",
                                o_idx + 1,
                                op.name,
                                op.selections.len()
                            );
                            let rich = if op.suppressed {
                                egui::RichText::new(label_txt).strikethrough().weak()
                            } else {
                                egui::RichText::new(label_txt)
                            };

                            ui.horizontal(|ui| {
                                if ui.selectable_label(is_sel, rich).clicked() {
                                    *active_setup_idx = s_idx;
                                    *focus = InspectorFocus::Operation {
                                        setup_idx: s_idx,
                                        op_idx: o_idx,
                                    };
                                    *pick_mode = default_pick_mode_for_op(
                                        s_idx,
                                        o_idx,
                                        op.kind,
                                        *op_tab,
                                    );
                                }
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
                                            .small_button(if op.suppressed { "On" } else { "Off" })
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
                    format!("op_{s_idx}_{next_op_num}"),
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
                *op_tab = OperationTab::Geometry;
                *pick_mode = default_pick_mode_for_op(s_idx, o_idx, kind, *op_tab);
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
