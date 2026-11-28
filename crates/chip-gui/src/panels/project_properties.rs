//! **Project Properties** modal (`Edit -> Project Properties...`) for editing project-level
//! metadata (name, display units, target CNC machine) and inspecting a live project summary.

use std::path::Path;

use chip_machines::MachineDefinition;
use chip_math::DisplayUnit;
use chip_project::ProjectDocument;
use chip_tools::{MaterialCategory, ProjectMaterialSelection, builtin_workpiece_materials};

use crate::{
    theme::ThemeTokens,
    widgets::{INPUT_COL_WIDTH, grid_row, inspector_grid, section_card, static_card},
};

const PROP_CONTROL_W: f32 = INPUT_COL_WIDTH + 138.0;
const INSPECTOR_CONTROL_W: f32 = INPUT_COL_WIDTH + 48.0;
const PROJECT_PROPERTIES_WINDOW_ID: &str = "chipcutter_project_properties_modal";

/// Persistent UI state for the **Project Properties** dialog.
#[derive(Debug, Default)]
pub struct ProjectPropertiesModalState {
    pub open: bool,
    name_draft: String,
    synced_project_name: String,
}

impl ProjectPropertiesModalState {
    /// Opens the dialog seeded with `project_name`.
    pub fn open_for_project(&mut self, project_name: &str) {
        self.open = true;
        self.name_draft = project_name.to_string();
        self.synced_project_name = project_name.to_string();
    }
}

/// Borrowed project & machine context passed to [`show_project_properties_modal`].
pub struct ProjectPropertiesCtx<'a> {
    pub project: &'a ProjectDocument,
    pub project_file_path: Option<&'a Path>,
    pub machines: &'a [MachineDefinition],
}

/// Side-effect actions emitted by [`show_project_properties_modal`] back to `ChipCutterApp`.
#[derive(Debug, Default, PartialEq)]
pub struct ProjectPropertiesActions {
    pub rename_project: Option<String>,
    pub display_units_changed: Option<DisplayUnit>,
    pub switch_machine_id: Option<String>,
    pub open_machine_manager: bool,
    pub workpiece_material_changed: Option<ProjectMaterialSelection>,
    pub workpiece_material_coalesce: bool,
    pub apply_matching_presets_to_ops: bool,
}

/// Renders the shared **General** & **Project Summary** aligned two-column `section_card` content
/// used by both the `Edit -> Project Properties...` modal and the right-hand **Project Inspector**.
pub fn show_project_properties_cards(
    ui: &mut egui::Ui,
    id_scope: &str,
    control_width: f32,
    name_draft: &mut String,
    pctx: &ProjectPropertiesCtx<'_>,
    commit_name_requested: &mut bool,
) -> ProjectPropertiesActions {
    let mut actions = ProjectPropertiesActions::default();
    let tok = ThemeTokens::from_ui(ui);

    section_card(
        ui,
        (id_scope, "project_props_general_card"),
        "General",
        true,
        |ui| {
            inspector_grid(ui, (id_scope, "project_props_general_grid"), |ui| {
                let name_tip = "Project display name shown in the window title, CAM Browser, and posted G-code headers (undoable)";
                grid_row(ui, "Project Name", Some(name_tip), |ui| {
                    let edit_resp = ui
                        .add(
                            egui::TextEdit::singleline(name_draft)
                                .id_salt((id_scope, "project_props_name_edit"))
                                .desired_width(control_width),
                        )
                        .on_hover_text(name_tip);
                    let trimmed = name_draft.trim();
                    let is_dirty = !trimmed.is_empty() && trimmed != pctx.project.project_name;
                    let enter_pressed = (edit_resp.has_focus() || edit_resp.lost_focus())
                        && ui.input(|i| i.key_pressed(egui::Key::Enter));
                    if ui
                        .add_enabled(is_dirty, egui::Button::new("Apply").small())
                        .on_hover_text("Commit new project name (Enter)")
                        .on_disabled_hover_text("Edit the project name above to apply")
                        .clicked()
                        || (is_dirty && (enter_pressed || edit_resp.lost_focus()))
                    {
                        *commit_name_requested = true;
                    }
                });

                let units_tip = "Display lengths and feedrates for this project (stored internally in millimeters; undoable)";
                grid_row(ui, "Display Units", Some(units_tip), |ui| {
                    let mut u_sel = pctx.project.display_units;
                    ui.selectable_value(&mut u_sel, DisplayUnit::Millimeter, "Millimeters (mm)")
                        .on_hover_text(units_tip);
                    ui.selectable_value(&mut u_sel, DisplayUnit::Inch, "Inches (in)")
                        .on_hover_text(units_tip);
                    if u_sel != pctx.project.display_units {
                        actions.display_units_changed = Some(u_sel);
                    }
                });

                let active_mid = pctx.project.effective_machine_id();
                let active_machine = pctx
                    .machines
                    .iter()
                    .find(|m| m.id.eq_ignore_ascii_case(active_mid))
                    .or_else(|| pctx.machines.first());
                let active_label = active_machine.map_or(active_mid, |m| m.name.as_str());
                let machine_tip = "Target CNC machine profile for travel envelope, spindle RPM limits, and tool library";
                grid_row(ui, "Target CNC Machine", Some(machine_tip), |ui| {
                    egui::ComboBox::from_id_salt((id_scope, "project_props_machine_cb"))
                        .width(control_width)
                        .truncate()
                        .selected_text(active_label)
                        .show_ui(ui, |ui| {
                            for m in pctx.machines {
                                let tag = if m.is_builtin {
                                    "[Built-In]"
                                } else {
                                    "[Custom]"
                                };
                                let is_cur = m.id.eq_ignore_ascii_case(active_mid);
                                if ui
                                    .selectable_label(
                                        is_cur,
                                        format!("{} {} ({} tools)", m.name, tag, m.tools.len()),
                                    )
                                    .on_hover_text(
                                        "Switch this project's target CNC machine profile",
                                    )
                                    .clicked()
                                    && !is_cur
                                {
                                    actions.switch_machine_id = Some(m.id.clone());
                                }
                            }
                        })
                        .response
                        .on_hover_text(machine_tip);

                    if ui
                        .button("Manage Machines...")
                        .on_hover_text(
                            "Open Machine Manager to inspect, clone, or configure CNC machine profiles",
                        )
                        .clicked()
                    {
                        actions.open_machine_manager = true;
                    }
                });
            });
        },
    );

    let eff_mat = pctx.project.effective_workpiece_material();
    let is_custom_mat = pctx
        .project
        .workpiece_material
        .as_ref()
        .is_some_and(ProjectMaterialSelection::is_custom);
    let builtin_mats = builtin_workpiece_materials();

    section_card(
        ui,
        (id_scope, "project_props_material_card"),
        "Workpiece Material",
        true,
        |ui| {
            inspector_grid(ui, (id_scope, "project_props_material_grid"), |ui| {
                let mat_tip = "Project-level stock material inherited by all operations for cutting physics and automatic tool preset matching (undoable)";
                grid_row(ui, "Stock Material", Some(mat_tip), |ui| {
                    egui::ComboBox::from_id_salt((id_scope, "project_props_material_cb"))
                        .width(control_width)
                        .truncate()
                        .selected_text(&eff_mat.name)
                        .show_ui(ui, |ui| {
                            for (cat_idx, &cat) in
                                MaterialCategory::BUILTIN_ORDER.iter().enumerate()
                            {
                                if cat_idx > 0 {
                                    ui.separator();
                                }
                                ui.label(
                                    egui::RichText::new(cat.label())
                                        .small()
                                        .strong()
                                        .color(tok.text_muted),
                                );
                                for profile in builtin_mats.iter().filter(|m| m.category == cat) {
                                    let is_cur = !is_custom_mat && eff_mat.id == profile.id;
                                    if ui
                                        .selectable_label(is_cur, &profile.name)
                                        .on_hover_text(&profile.notes)
                                        .clicked()
                                        && !is_cur
                                    {
                                        actions.workpiece_material_changed =
                                            Some(ProjectMaterialSelection::builtin(&profile.id));
                                        actions.workpiece_material_coalesce = false;
                                    }
                                }
                            }
                            ui.separator();
                            ui.label(
                                egui::RichText::new(MaterialCategory::Custom.label())
                                    .small()
                                    .strong()
                                    .color(tok.text_muted),
                            );
                            if ui
                                .selectable_label(is_custom_mat, "Custom Material...")
                                .on_hover_text(
                                    "Define custom workpiece cutting physics (u_s, K_c, h_min)",
                                )
                                .clicked()
                                && !is_custom_mat
                            {
                                actions.workpiece_material_changed =
                                    Some(ProjectMaterialSelection::custom(
                                        eff_mat.name.clone(),
                                        eff_mat.specific_cutting_energy_w_s_mm3,
                                        eff_mat.specific_cutting_force_n_mm2,
                                        eff_mat.min_chip_thickness_mm,
                                    ));
                                actions.workpiece_material_coalesce = false;
                            }
                        })
                        .response
                        .on_hover_text(mat_tip);
                });

                let physics_text = format!(
                    "Kc {:.0} N/mm2 | us {:.2} J/mm3",
                    eff_mat.specific_cutting_force_n_mm2, eff_mat.specific_cutting_energy_w_s_mm3
                );
                let physics_tip = "Specific cutting force (Kc) and specific cutting energy (us) for force, deflection, and spindle load calculations";
                grid_row(ui, "Cutting Physics", Some(physics_tip), |ui| {
                    ui.add(
                        egui::Label::new(
                            egui::RichText::new(&physics_text)
                                .small()
                                .color(tok.text_primary),
                        )
                        .truncate(),
                    )
                    .on_hover_text(physics_tip);
                });

                let chip_coolant_text = format!(
                    "h_min {:.3} mm | {}",
                    eff_mat.min_chip_thickness_mm,
                    eff_mat.preferred_coolant.label()
                );
                let chip_tip = "Minimum uncut chip thickness floor (rubbing/welding bound) and recommended coolant mode";
                grid_row(ui, "Chip & Coolant", Some(chip_tip), |ui| {
                    ui.add(
                        egui::Label::new(
                            egui::RichText::new(&chip_coolant_text)
                                .small()
                                .color(tok.text_primary),
                        )
                        .truncate(),
                    )
                    .on_hover_text(chip_tip);
                    if eff_mat.gummy_chip_weld_risk {
                        ui.colored_label(tok.warn_fg, "[Weld Risk]")
                            .on_hover_text(
                                "Gummy/ductile material prone to chip welding if cut dry or below h_min",
                            );
                    }
                });

                if is_custom_mat {
                    let mut custom_sel =
                        pctx.project.workpiece_material.clone().unwrap_or_else(|| {
                            ProjectMaterialSelection::custom(
                                eff_mat.name.clone(),
                                eff_mat.specific_cutting_energy_w_s_mm3,
                                eff_mat.specific_cutting_force_n_mm2,
                                eff_mat.min_chip_thickness_mm,
                            )
                        });
                    let mut custom_changed = false;

                    let mut cname = custom_sel
                        .custom_name
                        .clone()
                        .unwrap_or_else(|| eff_mat.name.clone());
                    grid_row(
                        ui,
                        "Custom Name",
                        Some("Display name for this custom workpiece material"),
                        |ui| {
                            let r = ui.add(
                                egui::TextEdit::singleline(&mut cname)
                                    .id_salt((id_scope, "project_props_custom_mat_name"))
                                    .desired_width(control_width),
                            );
                            if r.changed() {
                                custom_sel.custom_name = Some(cname.clone());
                                custom_changed = true;
                            }
                        },
                    );

                    let mut us_val = custom_sel
                        .custom_specific_cutting_energy
                        .unwrap_or(eff_mat.specific_cutting_energy_w_s_mm3);
                    grid_row(
                        ui,
                        "Specific Energy",
                        Some("Specific cutting energy us in J/mm3 (= W*s/mm3)"),
                        |ui| {
                            if ui
                                .add(
                                    egui::DragValue::new(&mut us_val)
                                        .speed(0.02)
                                        .max_decimals(2)
                                        .range(0.05..=20.0)
                                        .suffix(" J/mm3"),
                                )
                                .changed()
                            {
                                custom_sel.custom_specific_cutting_energy = Some(us_val);
                                custom_changed = true;
                            }
                        },
                    );

                    let mut kc_val = custom_sel
                        .custom_specific_cutting_force
                        .unwrap_or(eff_mat.specific_cutting_force_n_mm2);
                    grid_row(
                        ui,
                        "Specific Force",
                        Some("Specific cutting force Kc in N/mm2 (= MPa)"),
                        |ui| {
                            if ui
                                .add(
                                    egui::DragValue::new(&mut kc_val)
                                        .speed(10.0)
                                        .max_decimals(0)
                                        .range(50.0..=10_000.0)
                                        .suffix(" N/mm2"),
                                )
                                .changed()
                            {
                                custom_sel.custom_specific_cutting_force = Some(kc_val);
                                custom_changed = true;
                            }
                        },
                    );

                    let mut hmin_val = custom_sel
                        .custom_min_chip_thickness_mm
                        .unwrap_or(eff_mat.min_chip_thickness_mm);
                    grid_row(
                        ui,
                        "Min Chip (h_min)",
                        Some("Minimum uncut chip thickness floor in millimeters"),
                        |ui| {
                            if ui
                                .add(
                                    egui::DragValue::new(&mut hmin_val)
                                        .speed(0.0005)
                                        .max_decimals(4)
                                        .range(0.001..=0.5)
                                        .suffix(" mm"),
                                )
                                .changed()
                            {
                                custom_sel.custom_min_chip_thickness_mm = Some(hmin_val);
                                custom_changed = true;
                            }
                        },
                    );

                    if custom_changed {
                        actions.workpiece_material_changed = Some(custom_sel);
                        actions.workpiece_material_coalesce = true;
                    }
                }
            });

            if !eff_mat.notes.is_empty() {
                ui.add_space(4.0);
                ui.add(
                    egui::Label::new(
                        egui::RichText::new(&eff_mat.notes)
                            .small()
                            .color(tok.text_muted),
                    )
                    .wrap(),
                );
            }

            let (needing_update, ops_with_tools) =
                pctx.project.count_operations_needing_preset_match();
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                let apply_tip = "Update each operation's material preset to the best match for this workpiece material (preserves explicit user feed/speed overrides)";
                let btn = ui
                    .add_enabled(
                        needing_update > 0,
                        egui::Button::new("Apply Matching Presets to Ops").small(),
                    )
                    .on_hover_text(apply_tip)
                    .on_disabled_hover_text(if ops_with_tools == 0 {
                        "Add operations with assigned tools first"
                    } else {
                        "All operations already use the best matching preset for this material"
                    });
                if btn.clicked() {
                    actions.apply_matching_presets_to_ops = true;
                }
                let status_str = if ops_with_tools == 0 {
                    "No ops with tools".to_string()
                } else if needing_update == 0 {
                    format!("All {ops_with_tools} op(s) matched")
                } else {
                    format!("{needing_update} of {ops_with_tools} op(s) to update")
                };
                ui.label(
                    egui::RichText::new(status_str)
                        .small()
                        .color(tok.text_muted),
                );
            });
        },
    );

    section_card(
        ui,
        (id_scope, "project_props_summary_card"),
        "Project Summary",
        true,
        |ui| {
            let file_path_text = pctx.project_file_path.map_or_else(
                || "Unsaved Project".to_string(),
                |p| p.display().to_string(),
            );
            let schema_text = format!("v{}", pctx.project.schema_version);
            let models_text = if pctx.project.models.is_empty() {
                "0 models".to_string()
            } else {
                format!("{} model(s)", pctx.project.models.len())
            };
            let total_ops: usize = pctx.project.setups.iter().map(|s| s.operations.len()).sum();
            let setups_ops_text = format!(
                "{} setup(s) / {} operation(s)",
                pctx.project.setups.len(),
                total_ops
            );
            let tools_text = format!("{} tool(s)", pctx.project.tools.len());
            let sketches_text = format!("{} sketch(es)", pctx.project.sketches.len());
            let avoid_text = format!("{} region(s)", pctx.project.avoid_regions.len());

            inspector_grid(ui, (id_scope, "project_props_summary_grid"), |ui| {
                summary_row(
                    ui,
                    &tok,
                    "File Path",
                    &file_path_text,
                    Some("On-disk location of the active .chip.toml project file"),
                );
                summary_row(
                    ui,
                    &tok,
                    "Schema Version",
                    &schema_text,
                    Some("Project document serialization schema version"),
                );
                summary_row(
                    ui,
                    &tok,
                    "Workpiece Material",
                    &eff_mat.name,
                    Some(
                        "Effective project-level workpiece stock material inherited by all operations",
                    ),
                );
                summary_row(
                    ui,
                    &tok,
                    "Models",
                    &models_text,
                    Some("Linked 3D STEP solids and triangle meshes in this project"),
                );
                summary_row(
                    ui,
                    &tok,
                    "Setups / Operations",
                    &setups_ops_text,
                    Some("Total clamping Setups and CAM operations in this project"),
                );
                summary_row(
                    ui,
                    &tok,
                    "Project Tools",
                    &tools_text,
                    Some("Embedded cutter snapshots stored inside this project"),
                );
                summary_row(
                    ui,
                    &tok,
                    "2D Sketches",
                    &sketches_text,
                    Some("Parametric and imported DXF/SVG 2D sketch layers"),
                );
                summary_row(
                    ui,
                    &tok,
                    "Shared Keep-Out Regions",
                    &avoid_text,
                    Some("Reusable project-level shared Avoid / Keep-Out regions"),
                );
            });
        },
    );

    actions
}

/// Renders the right-hand **Project Inspector** pane (`InspectorFocus::Project`), sharing the exact
/// same `General` & `Project Summary` cards as `Edit -> Project Properties...`.
pub fn show_project_inspector(
    ui: &mut egui::Ui,
    pctx: ProjectPropertiesCtx<'_>,
) -> ProjectPropertiesActions {
    let draft_id = ui.make_persistent_id("project_inspector_name_draft");
    let synced_id = ui.make_persistent_id("project_inspector_synced_name");
    let mut name_draft = ui
        .ctx()
        .data_mut(|d| d.get_temp::<String>(draft_id))
        .unwrap_or_else(|| pctx.project.project_name.clone());
    let mut synced_name = ui
        .ctx()
        .data_mut(|d| d.get_temp::<String>(synced_id))
        .unwrap_or_else(|| pctx.project.project_name.clone());

    if synced_name != pctx.project.project_name {
        name_draft = pctx.project.project_name.clone();
        synced_name = pctx.project.project_name.clone();
    }

    let mut commit_name_requested = false;
    let mut actions = ProjectPropertiesActions::default();

    static_card(ui, None, |ui| {
        ui.horizontal(|ui| {
            ui.strong("Project Properties");
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.weak(format!("v{}", pctx.project.schema_version));
            });
        });
    });

    egui::ScrollArea::vertical()
        .id_salt("project_inspector_scroll")
        .auto_shrink([false, false])
        .show(ui, |ui| {
            actions = show_project_properties_cards(
                ui,
                "inspector",
                INSPECTOR_CONTROL_W,
                &mut name_draft,
                &pctx,
                &mut commit_name_requested,
            );
        });

    if commit_name_requested {
        let trimmed = name_draft.trim();
        if !trimmed.is_empty() && trimmed != pctx.project.project_name {
            actions.rename_project = Some(trimmed.to_string());
            synced_name = trimmed.to_string();
        }
    }

    ui.ctx().data_mut(|d| {
        d.insert_temp(draft_id, name_draft);
        d.insert_temp(synced_id, synced_name);
    });
    actions
}

/// Renders the **Project Properties** modal when `state.open == true`.
pub fn show_project_properties_modal(
    ctx: &egui::Context,
    state: &mut ProjectPropertiesModalState,
    pctx: ProjectPropertiesCtx<'_>,
) -> ProjectPropertiesActions {
    let mut actions = ProjectPropertiesActions::default();
    if !state.open {
        return actions;
    }

    if state.synced_project_name != pctx.project.project_name {
        state.name_draft = pctx.project.project_name.clone();
        state.synced_project_name = pctx.project.project_name.clone();
    }

    if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
        state.name_draft = pctx.project.project_name.clone();
        state.open = false;
        return actions;
    }

    let mut keep_open = state.open;
    let mut close_clicked = false;
    let mut commit_name_requested = false;

    egui::Window::new("Project Properties")
        .id(egui::Id::unique(PROJECT_PROPERTIES_WINDOW_ID))
        .open(&mut keep_open)
        .default_size([540.0, 620.0])
        .min_size([460.0, 340.0])
        .resizable(true)
        .collapsible(false)
        .show(ctx, |ui| {
            let total_w = ui.available_width().max(440.0);
            let total_h = ui.available_height().max(260.0);
            let footer_h = 34.0_f32;
            let body_h = (total_h - footer_h).max(180.0);
            ui.set_min_width(total_w);

            ui.allocate_ui_with_layout(
                egui::vec2(total_w, body_h),
                egui::Layout::top_down(egui::Align::LEFT),
                |ui| {
                    egui::ScrollArea::vertical()
                        .id_salt("project_properties_scroll")
                        .auto_shrink([false, false])
                        .show(ui, |ui| {
                            ui.set_width((total_w - 14.0).max(380.0));
                            actions = show_project_properties_cards(
                                ui,
                                "modal",
                                PROP_CONTROL_W,
                                &mut state.name_draft,
                                &pctx,
                                &mut commit_name_requested,
                            );
                        });
                },
            );

            ui.separator();
            ui.horizontal(|ui| {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui
                        .button("Close")
                        .on_hover_text("Apply pending changes and close Project Properties")
                        .clicked()
                    {
                        commit_name_requested = true;
                        close_clicked = true;
                    }
                });
            });
        });

    if commit_name_requested {
        let trimmed = state.name_draft.trim();
        if !trimmed.is_empty() && trimmed != pctx.project.project_name {
            actions.rename_project = Some(trimmed.to_string());
            state.synced_project_name = trimmed.to_string();
        }
    }

    state.open = keep_open && !close_clicked;
    actions
}

fn summary_row(
    ui: &mut egui::Ui,
    tok: &ThemeTokens,
    label: &str,
    value: &str,
    tooltip: Option<&str>,
) {
    grid_row(ui, label, tooltip, |ui| {
        let resp =
            ui.add(egui::Label::new(egui::RichText::new(value).color(tok.text_primary)).truncate());
        if let Some(tip) = tooltip {
            resp.on_hover_text(tip);
        }
    });
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::path::Path;

    use chip_cam::{Operation, OperationKind};
    use chip_math::DisplayUnit;
    use chip_project::{ProjectDocument, UndoHistory};

    use super::{
        PROJECT_PROPERTIES_WINDOW_ID, ProjectPropertiesActions, ProjectPropertiesCtx,
        ProjectPropertiesModalState, show_project_properties_modal,
    };

    fn collect_text_shapes(shapes: &[egui::epaint::ClippedShape]) -> Vec<(String, egui::Rect)> {
        fn visit(shape: &egui::Shape, out: &mut Vec<(String, egui::Rect)>) {
            match shape {
                egui::Shape::Text(ts) => {
                    out.push((ts.galley.text().to_string(), ts.visual_bounding_rect()));
                }
                egui::Shape::Vec(v) => {
                    for s in v {
                        visit(s, out);
                    }
                }
                _ => {}
            }
        }
        let mut out = Vec::new();
        for cs in shapes {
            visit(&cs.shape, &mut out);
        }
        out
    }

    fn click_at(pos: egui::Pos2) -> Vec<egui::Event> {
        vec![
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
        ]
    }

    #[test]
    fn project_properties_rename_units_machine_summary_and_undo() {
        let ctx = egui::Context::default();
        let machines = chip_machines::builtin_machine_definitions();
        let mut project = ProjectDocument::new("Original Job");
        project.setups[0].operations.push(Operation::new(
            "op1",
            "Pocket 1",
            OperationKind::Pocket2D,
        ));
        let mut history = UndoHistory::default();
        let mut state = ProjectPropertiesModalState::default();
        state.open_for_project(&project.project_name);

        let mut time = 1.0;
        let run_frame = |events: Vec<egui::Event>,
                         state: &mut ProjectPropertiesModalState,
                         project: &ProjectDocument,
                         file_path: Option<&Path>,
                         t: &mut f64| {
            *t += 0.25;
            let mut acts = ProjectPropertiesActions::default();
            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(1280.0, 800.0),
                    )),
                    time: Some(*t),
                    events,
                    ..Default::default()
                },
                |ui| {
                    acts = show_project_properties_modal(
                        ui.ctx(),
                        state,
                        ProjectPropertiesCtx {
                            project,
                            project_file_path: file_path,
                            machines: &machines,
                        },
                    );
                },
            );
            out.textures_delta.clear();
            (collect_text_shapes(&out.shapes), acts)
        };

        let _ = run_frame(Vec::new(), &mut state, &project, None, &mut time);
        let (texts, _) = run_frame(Vec::new(), &mut state, &project, None, &mut time);
        for expected in [
            "Project Properties",
            "General",
            "Project Name",
            "Display Units",
            "Target CNC Machine",
            "Manage Machines...",
            "Project Summary",
            "Unsaved Project",
            "v3",
            "1 setup(s) / 1 operation(s)",
        ] {
            assert!(
                texts.iter().any(|(t, _)| t == expected),
                "expected '{expected}' in Project Properties modal"
            );
        }

        // Click Project Name input, select all, type new name, and press Enter -> atomic Undo.
        let name_pos = texts
            .iter()
            .find(|(t, _)| t == "Original Job")
            .expect("project name draft must render")
            .1
            .center();
        let _ = run_frame(click_at(name_pos), &mut state, &project, None, &mut time);
        let (_, rename_acts) = run_frame(
            vec![
                egui::Event::Key {
                    key: egui::Key::A,
                    physical_key: None,
                    pressed: true,
                    repeat: false,
                    modifiers: egui::Modifiers::COMMAND,
                },
                egui::Event::Text("Aluminum Manifold Rev B".to_string()),
                egui::Event::Key {
                    key: egui::Key::Enter,
                    physical_key: None,
                    pressed: true,
                    repeat: false,
                    modifiers: egui::Modifiers::NONE,
                },
            ],
            &mut state,
            &project,
            None,
            &mut time,
        );
        let new_name = rename_acts
            .rename_project
            .expect("Enter must emit rename_project action");
        assert_eq!(new_name, "Aluminum Manifold Rev B");
        history.record_before_edit("Rename Project", None, &project);
        project.project_name = new_name;
        assert_eq!(
            history.undo(&mut project).as_deref(),
            Some("Rename Project")
        );
        assert_eq!(project.project_name, "Original Job");

        // Switch display units to Inches
        let (texts2, _) = run_frame(Vec::new(), &mut state, &project, None, &mut time);
        let inch_pos = texts2
            .iter()
            .find(|(t, _)| t == "Inches (in)")
            .expect("Inches (in) button must render")
            .1
            .center();
        let (_, unit_acts) = run_frame(click_at(inch_pos), &mut state, &project, None, &mut time);
        assert_eq!(unit_acts.display_units_changed, Some(DisplayUnit::Inch));

        // Switch Target CNC Machine via ComboBox -> emits switch_machine_id
        if machines.len() >= 2 {
            let active_mid = project.effective_machine_id();
            let active_name = machines
                .iter()
                .find(|m| m.id.eq_ignore_ascii_case(active_mid))
                .map_or(active_mid, |m| m.name.as_str());
            let other_machine = machines
                .iter()
                .find(|m| !m.id.eq_ignore_ascii_case(active_mid))
                .expect("at least two built-in machines exist");
            let combo_pos = texts2
                .iter()
                .find(|(t, _)| t == active_name)
                .expect("active machine combo must render")
                .1
                .center();
            let _ = run_frame(click_at(combo_pos), &mut state, &project, None, &mut time);
            let (popup_texts, _) = run_frame(Vec::new(), &mut state, &project, None, &mut time);
            let other_pos = popup_texts
                .iter()
                .find(|(t, _)| t.starts_with(&other_machine.name))
                .expect("alternate machine option must render in popup")
                .1
                .center();
            let (_, switch_acts) =
                run_frame(click_at(other_pos), &mut state, &project, None, &mut time);
            assert_eq!(
                switch_acts.switch_machine_id.as_deref(),
                Some(other_machine.id.as_str())
            );
        }

        // Click Manage Machines...
        let mm_pos = texts2
            .iter()
            .find(|(t, _)| t == "Manage Machines...")
            .expect("Manage Machines... button must render")
            .1
            .center();
        let (_, mm_acts) = run_frame(click_at(mm_pos), &mut state, &project, None, &mut time);
        assert!(mm_acts.open_machine_manager);

        // Press Escape -> closes cleanly
        let _ = run_frame(
            vec![egui::Event::Key {
                key: egui::Key::Escape,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::NONE,
            }],
            &mut state,
            &project,
            None,
            &mut time,
        );
        assert!(!state.open);
    }

    #[test]
    fn project_properties_window_opens_full_size_with_zero_creep_for_30_frames() {
        let ctx = egui::Context::default();
        let machines = chip_machines::builtin_machine_definitions();
        let project = ProjectDocument::new("Size Stability Job");
        let mut state = ProjectPropertiesModalState::default();
        state.open_for_project(&project.project_name);
        let win_id = egui::Id::unique(PROJECT_PROPERTIES_WINDOW_ID);

        let mut rects = Vec::new();
        for frame in 0..30 {
            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(1280.0, 800.0),
                    )),
                    time: Some(frame as f64 * 0.016),
                    ..Default::default()
                },
                |ui| {
                    let _ = show_project_properties_modal(
                        ui.ctx(),
                        &mut state,
                        ProjectPropertiesCtx {
                            project: &project,
                            project_file_path: Some(Path::new("/tmp/demo.chip.toml")),
                            machines: &machines,
                        },
                    );
                },
            );
            out.textures_delta.clear();
            let rect = ctx
                .memory(|m| m.area_rect(win_id))
                .expect("Project Properties window rect must exist");
            rects.push(rect);
        }

        assert!(
            rects[0].width() >= 520.0,
            "Frame 1 Project Properties window must open at full width (>= 520 px), got {:.1} px",
            rects[0].width()
        );
        let base_w = rects[1].width();
        let base_h = rects[1].height();
        for (idx, r) in rects.iter().enumerate().skip(2) {
            assert!(
                (r.width() - base_w).abs() < 1.0 && (r.height() - base_h).abs() < 1.0,
                "Project Properties window crept on frame {}: ({:.2}, {:.2}) vs frame 2 ({:.2}, {:.2})",
                idx + 1,
                r.width(),
                r.height(),
                base_w,
                base_h
            );
        }
    }

    #[test]
    fn project_properties_and_inspector_workpiece_material_selection_apply_and_undo() {
        use chip_machines::builtin_carvera_machine;
        use chip_tools::{ALUMINUM_6061_T6_ID, ALUMINUM_6063_T5_ID, ProjectMaterialSelection};

        use super::show_project_inspector;

        let ctx = egui::Context::default();
        let machines = chip_machines::builtin_machine_definitions();
        let carvera = builtin_carvera_machine();
        let t1 = carvera.tools.iter().find(|t| t.tool_number == 1).unwrap();

        let mut project = ProjectDocument::new("6063 Extrusion Job");
        project.ensure_tool_embedded(t1);
        let mut op = Operation::new("op1", "Pocket 1", OperationKind::Pocket2D);
        op.tool_id = Some(t1.id.clone());
        op.preset_id = Some("carvera_atc_al6061".to_string());
        op.feed_overrides.spindle_rpm = Some(11_200.0);
        project.setups[0].operations.push(op);

        let mut history = UndoHistory::default();
        let mut state = ProjectPropertiesModalState::default();
        state.open_for_project(&project.project_name);

        let mut time = 1.0;
        let run_modal = |events: Vec<egui::Event>,
                         state: &mut ProjectPropertiesModalState,
                         project: &ProjectDocument,
                         t: &mut f64| {
            *t += 0.25;
            let mut acts = ProjectPropertiesActions::default();
            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(1280.0, 900.0),
                    )),
                    time: Some(*t),
                    events,
                    ..Default::default()
                },
                |ui| {
                    acts = show_project_properties_modal(
                        ui.ctx(),
                        state,
                        ProjectPropertiesCtx {
                            project,
                            project_file_path: None,
                            machines: &machines,
                        },
                    );
                },
            );
            out.textures_delta.clear();
            (collect_text_shapes(&out.shapes), acts)
        };

        let _ = run_modal(Vec::new(), &mut state, &project, &mut time);
        let (texts, _) = run_modal(Vec::new(), &mut state, &project, &mut time);
        for expected in [
            "Workpiece Material",
            "Stock Material",
            "Aluminum 6061-T6",
            "Kc 750 N/mm2 | us 0.75 J/mm3",
            "Apply Matching Presets to Ops",
        ] {
            assert!(
                texts.iter().any(|(t, _)| t == expected),
                "expected '{expected}' in Project Properties modal, got: {:?}",
                texts.iter().map(|(t, _)| t.as_str()).collect::<Vec<_>>()
            );
        }

        // Open Stock Material ComboBox and pick Aluminum 6063-T5 Extrusion
        let mat_combo_pos = texts
            .iter()
            .find(|(t, _)| t == "Aluminum 6061-T6")
            .expect("Stock Material ComboBox must render")
            .1
            .center();
        let _ = run_modal(click_at(mat_combo_pos), &mut state, &project, &mut time);
        let (popup_texts, _) = run_modal(Vec::new(), &mut state, &project, &mut time);
        let opt_6063 = popup_texts
            .iter()
            .find(|(t, _)| t == "Aluminum 6063-T5 Extrusion")
            .expect("Aluminum 6063-T5 Extrusion must appear in grouped material ComboBox")
            .1
            .center();
        let (_, mat_acts) = run_modal(click_at(opt_6063), &mut state, &project, &mut time);
        assert_eq!(
            mat_acts.workpiece_material_changed,
            Some(ProjectMaterialSelection::builtin(ALUMINUM_6063_T5_ID))
        );
        assert!(!mat_acts.workpiece_material_coalesce);

        // Apply with atomic Undo
        history.record_before_edit("Change Workpiece Material", None, &project);
        project.workpiece_material = mat_acts.workpiece_material_changed;
        assert_eq!(
            project.effective_workpiece_material().id,
            ALUMINUM_6063_T5_ID
        );

        // Modal now shows 6063-T5 physics readout & enabled Apply Matching Presets button
        let (texts_6063, _) = run_modal(Vec::new(), &mut state, &project, &mut time);
        assert!(
            texts_6063
                .iter()
                .any(|(t, _)| t == "Kc 850 N/mm2 | us 0.85 J/mm3")
        );
        assert!(texts_6063.iter().any(|(t, _)| t == "[Weld Risk]"));

        let apply_btn_pos = texts_6063
            .iter()
            .find(|(t, _)| t == "Apply Matching Presets to Ops")
            .expect("Apply Matching Presets to Ops button must render")
            .1
            .center();
        let (_, apply_acts) = run_modal(click_at(apply_btn_pos), &mut state, &project, &mut time);
        assert!(apply_acts.apply_matching_presets_to_ops);

        history.record_before_edit("Apply Matching Material Presets", None, &project);
        assert_eq!(project.apply_matching_presets_for_material(), 1);
        assert_eq!(
            project.setups[0].operations[0].preset_id.as_deref(),
            Some("carvera_atc_al6063")
        );
        assert_eq!(
            project.setups[0].operations[0].feed_overrides.spindle_rpm,
            Some(11_200.0),
            "Apply Matching Presets must preserve user feed_overrides"
        );

        // Atomic Undo reverts Apply Matching Presets, then reverts Workpiece Material
        assert_eq!(
            history.undo(&mut project).as_deref(),
            Some("Apply Matching Material Presets")
        );
        assert_eq!(
            project.setups[0].operations[0].preset_id.as_deref(),
            Some("carvera_atc_al6061")
        );
        assert_eq!(
            history.undo(&mut project).as_deref(),
            Some("Change Workpiece Material")
        );
        assert_eq!(
            project.effective_workpiece_material().id,
            ALUMINUM_6061_T6_ID
        );

        // Right-hand Project Inspector also renders Workpiece Material & Custom fields cleanly
        let inspector_ctx = egui::Context::default();
        project.workpiece_material = Some(ProjectMaterialSelection::custom(
            "Custom Billet",
            0.92,
            920.0,
            0.012,
        ));
        let mut insp_texts = Vec::new();
        for frame in 0..10 {
            let mut out = inspector_ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(380.0, 900.0),
                    )),
                    time: Some(frame as f64 * 0.05),
                    ..Default::default()
                },
                |ui| {
                    egui::CentralPanel::default().show(ui, |ui| {
                        let _ = show_project_inspector(
                            ui,
                            ProjectPropertiesCtx {
                                project: &project,
                                project_file_path: None,
                                machines: &machines,
                            },
                        );
                    });
                },
            );
            out.textures_delta.clear();
            insp_texts = collect_text_shapes(&out.shapes);
        }
        for expected in [
            "Workpiece Material",
            "Custom Name",
            "Specific Energy",
            "Specific Force",
            "Min Chip (h_min)",
            "Kc 920 N/mm2 | us 0.92 J/mm3",
        ] {
            assert!(
                insp_texts.iter().any(|(t, _)| t == expected),
                "expected '{expected}' in right-hand Project Inspector"
            );
        }
    }
}
