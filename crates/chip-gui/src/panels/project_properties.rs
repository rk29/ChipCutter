//! **Project Properties** modal (`Edit -> Project Properties...`) for editing project-level
//! metadata (name, display units, target CNC machine) and inspecting a live project summary.

use std::path::Path;

use chip_math::DisplayUnit;
use chip_project::ProjectDocument;
use chip_tools::MachineDefinition;

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
#[derive(Debug, Default, PartialEq, Eq)]
pub struct ProjectPropertiesActions {
    pub rename_project: Option<String>,
    pub display_units_changed: Option<DisplayUnit>,
    pub switch_machine_id: Option<String>,
    pub open_machine_manager: bool,
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
        .default_size([540.0, 440.0])
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
        let machines = chip_tools::builtin_machine_definitions();
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
        let machines = chip_tools::builtin_machine_definitions();
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
}
