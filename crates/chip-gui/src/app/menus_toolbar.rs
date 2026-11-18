use std::path::Path;

use chip_cam::{Operation, OperationKind, Setup, WorkOffsetSlot};
use chip_math::DisplayUnit;
use chip_project::{ProjectDocument, validate_project};
use chip_sim::SimResolution;

use crate::{
    app::ChipCutterApp,
    camera::{MousePreset, OrbitMode},
    layout::{WorkspacePane, create_default_workspace_tree, focus_workspace_pane},
    panels::{InspectorFocus, OperationTab, ViewportPickMode, browser::default_pick_mode_for_op},
};

impl ChipCutterApp {
    pub(crate) fn draw_top_bars(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        self.sync_window_title(&ctx);
        self.handle_global_shortcuts(&ctx);
        self.draw_menu_bar(ui, &ctx);
        self.draw_quick_toolbar(ui, &ctx);
        self.draw_status_banners(ui, &ctx);
    }

    fn sync_window_title(&mut self, ctx: &egui::Context) {
        let is_dirty_doc = self.clean_undo_len != Some(self.history.undo_len());
        let desired_title = format!(
            "ChipCutter — {}{}",
            self.project.project_name,
            if is_dirty_doc { "*" } else { "" }
        );
        if desired_title != self.last_window_title {
            self.last_window_title = desired_title.clone();
            ctx.send_viewport_cmd(egui::ViewportCommand::Title(desired_title));
        }
    }

    fn handle_global_shortcuts(&mut self, ctx: &egui::Context) {
        let (
            gen_shortcut,
            reload_shortcut,
            undo_shortcut,
            redo_shortcut,
            save_shortcut,
            open_shortcut,
            esc_key,
        ) = ctx.input(|i| {
            let cmd = i.modifiers.command || i.modifiers.ctrl;
            (
                cmd && i.key_pressed(egui::Key::G),
                cmd && i.key_pressed(egui::Key::R),
                cmd && !i.modifiers.shift && i.key_pressed(egui::Key::Z),
                (cmd && i.key_pressed(egui::Key::Y))
                    || (cmd && i.modifiers.shift && i.key_pressed(egui::Key::Z)),
                cmd && i.key_pressed(egui::Key::S),
                cmd && i.key_pressed(egui::Key::O),
                i.key_pressed(egui::Key::Escape),
            )
        });

        if gen_shortcut {
            self.trigger_generate_dirty(Some(self.active_setup_idx), None);
        }
        if reload_shortcut {
            self.reload_active_step_model(ctx);
        }
        if undo_shortcut && !ctx.egui_wants_keyboard_input() {
            self.perform_undo();
        }
        if redo_shortcut && !ctx.egui_wants_keyboard_input() {
            self.perform_redo();
        }
        if save_shortcut {
            self.save_project_dialog();
        }
        if open_shortcut {
            self.open_project_dialog(ctx);
        }
        if esc_key {
            if self.pick_mode != ViewportPickMode::InspectEntity {
                self.pick_mode = ViewportPickMode::InspectEntity;
                self.highlight.preview_tab_marker_wcs = None;
            } else if self.dispatcher.is_busy() {
                let n = self.dispatcher.cancel_all();
                self.logs
                    .push(format!("Cancelled {n} in-flight generation job(s)."));
            }
        }

        if !ctx.egui_wants_keyboard_input() {
            if ctx.input(|i| i.key_pressed(egui::Key::P)) {
                self.camera.toggle_projection();
            }
            if ctx.input(|i| i.key_pressed(egui::Key::F)) {
                self.fit_scene_to_viewport();
            }
        }
    }

    fn draw_menu_bar(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        egui::Panel::top("chip_menu_bar").show(ui, |ui| {
            egui::MenuBar::new().ui(ui, |ui| {
                self.draw_file_menu(ui, ctx);
                self.draw_edit_menu(ui);
                self.draw_setup_menu(ui);
                self.draw_operations_menu(ui);
                ui.menu_button("Tools", |ui| {
                    if ui.button("Tool Library Manager…").clicked() {
                        self.tool_library_modal.open = true;
                        ui.close();
                    }
                });
                self.draw_simulate_menu(ui, ctx);
                self.draw_post_menu(ui);
                self.draw_view_menu(ui);

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if self.loading_in_flight {
                        ui.label("Loading STEP…");
                        ui.spinner();
                    } else if let Some(m) = &self.model {
                        ui.weak(format!(
                            "{} | {} faces, {} edges",
                            m.doc.source_name,
                            m.doc.faces.len(),
                            m.doc.edges.len()
                        ));
                    } else if !self.project.sketches.is_empty() {
                        ui.weak(format!(
                            "Pure-2D Job | {} sketch layer(s)",
                            self.project.sketches.len()
                        ));
                    }
                });
            });
        });
    }

    fn draw_file_menu(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        ui.menu_button("File", |ui| {
            if ui.button("New Project").clicked() {
                self.clear_bundle_extract_dir();
                self.history.clear();
                self.clean_undo_len = Some(0);
                self.last_reload_banner = None;
                self.external_step_modified = false;
                self.project = ProjectDocument::new("Untitled Job");
                self.project_file_path = None;
                self.active_setup_idx = 0;
                self.inspector_focus = InspectorFocus::Setup { setup_idx: 0 };
                ui.close();
            }
            if ui
                .button("Open Project / Bundle (.chip.toml, .chipz)… (Ctrl+O)")
                .clicked()
            {
                ui.close();
                self.open_project_dialog(ctx);
            }
            if ui.button("Save Project (Ctrl+S)").clicked() {
                ui.close();
                self.save_project_dialog();
            }
            if ui.button("Save Project As…").clicked() {
                ui.close();
                if let Some(path) = rfd::FileDialog::new()
                    .set_file_name("job.chip.toml")
                    .save_file()
                {
                    self.save_project_to_disk(&path);
                }
            }
            if ui
                .button("Export Portable Bundle (.chipz)…")
                .on_hover_text(
                    "Pack project.chip.toml + STEP models + custom .post.toml into a deterministic self-contained .chipz archive",
                )
                .clicked()
            {
                ui.close();
                self.export_chipz_dialog();
            }
            if ui.button("Export G-Code (.nc / .ngc)…").clicked() {
                ui.close();
                self.trigger_gcode_export_from_menu();
            }
            ui.separator();
            if ui.button("Import STEP Model…").clicked() {
                ui.close();
                self.open_step_dialog(ctx);
            }
            if ui
                .button("Import 2D Vector (.dxf / .svg)…")
                .on_hover_text(
                    "Import AutoCAD DXF or W3C SVG 2D vector artwork into a Sketch Layer or Pure-2D CNC job",
                )
                .clicked()
            {
                ui.close();
                self.vector_import_modal.open_with_file_picker();
            }
            if ui
                .add_enabled(
                    self.model.is_some(),
                    egui::Button::new("Reload STEP Model (Ctrl+R)"),
                )
                .clicked()
            {
                ui.close();
                self.reload_active_step_model(ctx);
            }
            if ui
                .add_enabled(
                    self.model.is_some(),
                    egui::Button::new("Replace STEP Model (Associative Re-Bind)…"),
                )
                .clicked()
            {
                ui.close();
                self.replace_step_dialog(ctx);
            }
            ui.separator();
            if ui.button("Load CAM Torture Block").clicked() {
                ui.close();
                self.spawn_cam_torture_sample_load(true, ctx.clone());
            }
            if ui
                .button("Demo: Load Pure-2D Router Panel (.dxf)")
                .clicked()
            {
                ui.close();
                self.load_pure2d_router_panel_demo(ctx);
            }
            if ui.button("Load Multi-Feature Sample Bracket").clicked() {
                ui.close();
                self.spawn_multi_feature_sample_load(false, true, ctx.clone());
            }
            if ui.button("Load 30°-Tilted Sample Bracket").clicked() {
                ui.close();
                self.spawn_multi_feature_sample_load(true, false, ctx.clone());
            }
            if ui.button("Load M1 Sample Plate").clicked() {
                ui.close();
                self.spawn_sample_plate_load(ctx.clone());
            }
            ui.separator();
            if ui.button("Demo: Load CAD Revision v1").clicked() {
                ui.close();
                self.spawn_revision_demo_load(false, ctx.clone());
            }
            if ui.button("Demo: Reload Revised Part (v1 -> v2)").clicked() {
                ui.close();
                self.spawn_revision_demo_load(true, ctx.clone());
            }
            ui.separator();
            if ui.button("Quit").clicked() {
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
        });
    }

    fn draw_edit_menu(&mut self, ui: &mut egui::Ui) {
        ui.menu_button("Edit", |ui| {
            let undo_lbl = match self.history.peek_undo_label() {
                Some(l) => format!("Undo {l} (Ctrl+Z)"),
                None => "Undo (Ctrl+Z)".to_string(),
            };
            if ui
                .add_enabled(self.history.can_undo(), egui::Button::new(undo_lbl))
                .clicked()
            {
                self.perform_undo();
                ui.close();
            }
            let redo_lbl = match self.history.peek_redo_label() {
                Some(l) => format!("Redo {l} (Ctrl+Y)"),
                None => "Redo (Ctrl+Y)".to_string(),
            };
            if ui
                .add_enabled(self.history.can_redo(), egui::Button::new(redo_lbl))
                .clicked()
            {
                self.perform_redo();
                ui.close();
            }
        });
    }

    fn draw_setup_menu(&mut self, ui: &mut egui::Ui) {
        ui.menu_button("Setup", |ui| {
            if ui.button("New Setup").clicked() {
                self.history
                    .record_before_edit("Add Setup", None, &self.project);
                let n = self.project.setups.len() + 1;
                self.project.setups.push(Setup::new(
                    format!("setup_{n}"),
                    format!("Setup {n} — Op {n}"),
                    WorkOffsetSlot::G55,
                ));
                self.active_setup_idx = self.project.setups.len() - 1;
                self.inspector_focus = InspectorFocus::Setup {
                    setup_idx: self.active_setup_idx,
                };
                ui.close();
            }
            if ui.button("Align View to Active Setup Top").clicked() {
                self.align_camera_to_active_setup();
                ui.close();
            }
            if ui.button("Validate Project Invariants").clicked() {
                let rep = validate_project(
                    &self.project,
                    self.active_project_dir(),
                    self.model.as_ref().map(|m| m.doc.as_ref()),
                );
                self.logs.push(format!(
                    "Validation check: {} error(s), {} warning(s).",
                    rep.error_count(),
                    rep.warning_count()
                ));
                for d in rep.diagnostics {
                    self.logs
                        .push(format!("  [{:?}] {}: {}", d.severity, d.scope, d.message));
                }
                ui.close();
            }
        });
    }

    fn draw_operations_menu(&mut self, ui: &mut egui::Ui) {
        ui.menu_button("Operations", |ui| {
            if ui
                .button("Generate Dirty in Active Setup (Ctrl+G)")
                .clicked()
            {
                self.trigger_generate_dirty(Some(self.active_setup_idx), None);
                ui.close();
            }
            if ui.button("Generate All Setups").clicked() {
                self.trigger_generate_dirty(None, None);
                ui.close();
            }
            if self.dispatcher.is_busy() && ui.button("Cancel Generation (Esc)").clicked() {
                self.dispatcher.cancel_all();
                ui.close();
            }
            ui.separator();
            if ui.button("+ 2D Adaptive / Trochoidal Clearing").clicked() {
                self.create_new_adaptive_operation();
                ui.close();
            }
            for kind in OperationKind::ALL {
                if ui.button(format!("+ {}", kind.label())).clicked()
                    && self.active_setup_idx < self.project.setups.len()
                {
                    self.history.record_before_edit(
                        format!("Add {}", kind.label()),
                        None,
                        &self.project,
                    );
                    let s_idx = self.active_setup_idx;
                    let n = self.project.setups[s_idx].operations.len() + 1;
                    let mut op = Operation::new(
                        format!("op_{s_idx}_{n}"),
                        format!("{} {n}", kind.default_name()),
                        kind,
                    );
                    if let Some(t) = self
                        .project
                        .tools
                        .iter()
                        .find(|t| kind.is_compatible_with_tool(t))
                    {
                        op.tool_id = Some(t.id.clone());
                        op.preset_id = t.presets.first().map(|p| p.id.clone());
                    }
                    self.project.setups[s_idx].operations.push(op);
                    let o_idx = self.project.setups[s_idx].operations.len() - 1;
                    self.inspector_focus = InspectorFocus::Operation {
                        setup_idx: s_idx,
                        op_idx: o_idx,
                    };
                    self.op_tab = if kind == OperationKind::Facing {
                        OperationTab::Passes
                    } else {
                        OperationTab::Geometry
                    };
                    self.pick_mode = default_pick_mode_for_op(s_idx, o_idx, kind, self.op_tab);
                    ui.close();
                }
            }
        });
    }

    fn create_new_adaptive_operation(&mut self) {
        if self.active_setup_idx >= self.project.setups.len() {
            return;
        }
        self.history
            .record_before_edit("Add 2D Adaptive Clearing", None, &self.project);
        let s_idx = self.active_setup_idx;
        let n = self.project.setups[s_idx].operations.len() + 1;
        let mut op = Operation::new_adaptive_2d(
            format!("op_{s_idx}_{n}_adaptive"),
            format!("2D Adaptive {n}"),
        );
        if let Some(t) = self
            .project
            .tools
            .iter()
            .find(|t| OperationKind::Pocket2D.is_compatible_with_tool(t))
        {
            op.tool_id = Some(t.id.clone());
            op.preset_id = t.presets.first().map(|p| p.id.clone());
        }
        self.project.setups[s_idx].operations.push(op);
        let o_idx = self.project.setups[s_idx].operations.len() - 1;
        self.inspector_focus = InspectorFocus::Operation {
            setup_idx: s_idx,
            op_idx: o_idx,
        };
        self.op_tab = OperationTab::Geometry;
        self.pick_mode =
            default_pick_mode_for_op(s_idx, o_idx, OperationKind::Pocket2D, self.op_tab);
    }

    fn draw_simulate_menu(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        ui.menu_button("Simulate", |ui| {
            if ui.button("Run / Refresh Heightmap Simulation").clicked() {
                focus_workspace_pane(&mut self.tree, WorkspacePane::SimulationTimeline);
                self.trigger_active_setup_simulation(ctx);
                ui.close();
            }
            let play_lbl = if self.sim_panel.is_playing {
                "Pause Scrubbing (Space)"
            } else {
                "Play Scrubbing (Space)"
            };
            if ui
                .add_enabled(
                    self.sim_panel.active_result.is_some(),
                    egui::Button::new(play_lbl),
                )
                .clicked()
            {
                self.sim_panel.is_playing = !self.sim_panel.is_playing;
                ui.close();
            }
            if ui
                .add_enabled(
                    self.sim_panel.active_result.is_some(),
                    egui::Button::new("Jump to Final Machined Stock"),
                )
                .clicked()
            {
                if let Some(res) = &self.sim_panel.active_result {
                    self.sim_panel.current_time_sec = res.timeline.total_time_sec;
                    self.sim_panel.is_playing = false;
                }
                ui.close();
            }
            ui.separator();
            ui.label("Resolution:");
            for res in SimResolution::ALL {
                ui.radio_value(&mut self.sim_panel.resolution, res, res.label());
            }
            ui.separator();
            ui.checkbox(
                &mut self.sim_panel.show_machined_stock,
                "Show 3D Machined Stock",
            );
            ui.checkbox(
                &mut self.sim_panel.color_by_operation,
                "Color Cuts by Operation",
            );
            ui.checkbox(
                &mut self.sim_panel.show_cutter_assembly,
                "Show 3D Cutter & Holder",
            );
        });
    }

    fn draw_post_menu(&mut self, ui: &mut egui::Ui) {
        ui.menu_button("Post", |ui| {
            if ui.button("Open G-Code Preview Tab").clicked() {
                focus_workspace_pane(&mut self.tree, WorkspacePane::GCodePreview);
                ui.close();
            }
            if ui.button("Export G-Code (.nc / .ngc)…").clicked() {
                ui.close();
                self.trigger_gcode_export_from_menu();
            }
            ui.separator();
            let proj_dir = self
                .project_file_path
                .as_deref()
                .and_then(Path::parent)
                .or(self.bundle_extract_dir.as_deref());
            self.gcode_panel.ensure_profiles_loaded(proj_dir);
            if self.active_setup_idx < self.project.setups.len() {
                ui.label("Active Setup Post Profile:");
                let cur_id = self.project.setups[self.active_setup_idx]
                    .post_profile
                    .clone();
                let profiles: Vec<(String, String)> = self
                    .gcode_panel
                    .available_profiles()
                    .iter()
                    .map(|p| (p.id.clone(), p.name.clone()))
                    .collect();
                for (pid, pname) in profiles {
                    let is_sel = cur_id.eq_ignore_ascii_case(&pid);
                    if ui.selectable_label(is_sel, pname).clicked() && !is_sel {
                        self.history
                            .record_before_edit("Change Post Profile", None, &self.project);
                        self.project.setups[self.active_setup_idx].post_profile = pid;
                        self.gcode_panel.invalidate_preview();
                        ui.close();
                    }
                }
            }
        });
    }

    fn draw_view_menu(&mut self, ui: &mut egui::Ui) {
        ui.menu_button("View", |ui| {
            if ui.button("Reset Default Workspace Layout").clicked() {
                self.tree = create_default_workspace_tree();
                ui.close();
            }
            ui.separator();
            ui.checkbox(
                &mut self.highlight.show_toolpaths,
                "Show 3D Generated Toolpaths",
            );
            ui.checkbox(&mut self.highlight.show_rapids, "Show Rapid Moves (G0)");
            ui.checkbox(
                &mut self.show_all_setup_toolpaths,
                "Show All Operations in Active Setup",
            );
            ui.separator();
            ui.checkbox(
                &mut self.highlight.show_grid,
                "Show Active Setup WCS XY Grid",
            );
            ui.checkbox(&mut self.highlight.show_stock, "Show Translucent Stock");
            ui.checkbox(&mut self.highlight.ghost_part, "Ghost Part Translucency");
            ui.checkbox(
                &mut self.show_z_planes,
                "Show Z-Height Planes in Heights Tab",
            );
            ui.separator();
            if ui.button("Toggle Orthographic / Perspective (P)").clicked() {
                self.camera.toggle_projection();
                ui.close();
            }
            if ui.button("Fit Model to Viewport (F)").clicked() {
                self.fit_scene_to_viewport();
                ui.close();
            }
            if ui.button("Align View to Active Setup Top (+Z)").clicked() {
                self.align_camera_to_active_setup();
                ui.close();
            }
        });
    }

    fn draw_quick_toolbar(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        egui::Panel::top("chip_toolbar").show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                if ui
                    .button("Open")
                    .on_hover_text("Open .chip.toml Project or .chipz Bundle (Ctrl+O)")
                    .clicked()
                {
                    self.open_project_dialog(ctx);
                }
                if ui
                    .button("Save")
                    .on_hover_text("Save .chip.toml Project (Ctrl+S)")
                    .clicked()
                {
                    self.save_project_dialog();
                }
                if ui
                    .button(".chipz")
                    .on_hover_text("Export Self-Contained Portable Bundle (.chipz)")
                    .clicked()
                {
                    self.export_chipz_dialog();
                }
                if ui
                    .button("DXF/SVG")
                    .on_hover_text("Import 2D Vector Drawing (.dxf / .svg) or Start Pure-2D Job")
                    .clicked()
                {
                    self.vector_import_modal.open = true;
                }
                if ui
                    .add_enabled(self.model.is_some(), egui::Button::new("Reload CAD"))
                    .on_hover_text("Associatively reload active STEP model (Ctrl+R)")
                    .clicked()
                {
                    self.reload_active_step_model(ctx);
                }
                if ui
                    .add_enabled(self.history.can_undo(), egui::Button::new("Undo"))
                    .clicked()
                {
                    self.perform_undo();
                }
                if ui
                    .add_enabled(self.history.can_redo(), egui::Button::new("Redo"))
                    .clicked()
                {
                    self.perform_redo();
                }

                ui.separator();
                if ui
                    .button("+ Adaptive")
                    .on_hover_text("Add a new 2D Adaptive / Trochoidal Clearing operation")
                    .clicked()
                {
                    self.create_new_adaptive_operation();
                }
                if self.dispatcher.is_busy() {
                    ui.spinner();
                    if ui
                        .button(format!("Cancel ({})", self.dispatcher.in_flight_count()))
                        .on_hover_text("Cancel in-flight toolpath generation (Esc)")
                        .clicked()
                    {
                        self.dispatcher.cancel_all();
                    }
                } else if ui
                    .button("Generate (Ctrl+G)")
                    .on_hover_text("Generate dirty 2.5D toolpaths in active Setup on rayon pool")
                    .clicked()
                {
                    self.trigger_generate_dirty(Some(self.active_setup_idx), None);
                }

                if ui
                    .button("Simulate")
                    .on_hover_text("Run 2.5D Z-Dexel Heightmap Stock Removal & Crash Checks")
                    .clicked()
                {
                    focus_workspace_pane(&mut self.tree, WorkspacePane::SimulationTimeline);
                    self.trigger_active_setup_simulation(ctx);
                }
                if ui
                    .button("Post G-Code")
                    .on_hover_text("Open G-Code Preview & Safety-Gated Export")
                    .clicked()
                {
                    focus_workspace_pane(&mut self.tree, WorkspacePane::GCodePreview);
                }

                if ui.button("Tool Library").clicked() {
                    self.tool_library_modal.open = true;
                }
                if ui
                    .button("Setup Top")
                    .on_hover_text("Align camera straight down Active Setup +Z")
                    .clicked()
                {
                    self.align_camera_to_active_setup();
                }
                ui.toggle_value(&mut self.highlight.show_toolpaths, "Toolpaths");
                ui.toggle_value(&mut self.highlight.show_rapids, "Rapids");
                ui.toggle_value(&mut self.highlight.show_stock, "Stock");
                ui.toggle_value(&mut self.highlight.ghost_part, "Ghost");

                ui.separator();
                let proj_label = if self.camera.projection.is_orthographic() {
                    "Ortho (P)"
                } else {
                    "Persp (P)"
                };
                if ui.button(proj_label).clicked() {
                    self.camera.toggle_projection();
                }
                if ui.button("Fit (F)").clicked() {
                    self.fit_scene_to_viewport();
                }

                ui.separator();
                ui.label("Mouse:");
                egui::ComboBox::from_id_salt("mouse_preset_combo")
                    .selected_text(self.camera.mouse_preset.short_label())
                    .show_ui(ui, |ui| {
                        for preset in MousePreset::ALL {
                            ui.selectable_value(
                                &mut self.camera.mouse_preset,
                                preset,
                                preset.label(),
                            );
                        }
                    });

                let next_orbit = match self.camera.orbit_mode {
                    OrbitMode::TurntableZUp => OrbitMode::FreeTrackball,
                    OrbitMode::FreeTrackball => OrbitMode::TurntableZUp,
                };
                if ui.button(self.camera.orbit_mode.label()).clicked() {
                    self.camera.set_orbit_mode(next_orbit);
                }

                ui.separator();
                ui.label("Units:");
                let mut u_sel = self.project.display_units;
                ui.selectable_value(&mut u_sel, DisplayUnit::Millimeter, "mm");
                ui.selectable_value(&mut u_sel, DisplayUnit::Inch, "in");
                if u_sel != self.project.display_units {
                    self.history
                        .record_before_edit("Change Display Units", None, &self.project);
                    self.project.display_units = u_sel;
                    self.refresh_dependent_expressions();
                }
            });
        });
    }

    fn draw_status_banners(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        if self.external_step_modified {
            egui::Panel::top("chip_external_step_banner")
                .frame(
                    egui::Frame::NONE
                        .fill(egui::Color32::from_rgb(54, 42, 18))
                        .inner_margin(egui::Margin::symmetric(10, 4)),
                )
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.colored_label(
                            egui::Color32::from_rgb(255, 206, 86),
                            "[*] External STEP file updated on disk — CAD revision change detected.",
                        );
                        if ui.small_button("Reload & Re-Match (Ctrl+R)").clicked() {
                            self.reload_active_step_model(ctx);
                        }
                        if ui.small_button("Dismiss").clicked() {
                            self.external_step_modified = false;
                        }
                    });
                });
        }

        if let Some((msg, has_warn)) = self.last_reload_banner.clone() {
            let bg = if has_warn {
                egui::Color32::from_rgb(56, 32, 24)
            } else {
                egui::Color32::from_rgb(22, 48, 34)
            };
            let fg = if has_warn {
                egui::Color32::from_rgb(255, 172, 98)
            } else {
                egui::Color32::from_rgb(112, 232, 154)
            };
            egui::Panel::top("chip_reload_summary_banner")
                .frame(
                    egui::Frame::NONE
                        .fill(bg)
                        .inner_margin(egui::Margin::symmetric(10, 4)),
                )
                .show(ui, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        ui.colored_label(fg, &msg);
                        if ui.small_button("Regenerate Dirty (Ctrl+G)").clicked() {
                            self.auto_sim_after_gen = true;
                            self.trigger_generate_dirty(None, None);
                        }
                        if ui.small_button("Dismiss").clicked() {
                            self.last_reload_banner = None;
                        }
                    });
                });
        }
    }

    pub(crate) fn trigger_gcode_export_from_menu(&mut self) {
        focus_workspace_pane(&mut self.tree, WorkspacePane::GCodePreview);
        let eff = self.effective_cad_arc();
        let cad_doc = eff.as_ref().map(|(d, _)| d);
        let cad_rev = eff.as_ref().map_or(0, |(_, r)| *r);
        let virtual_proj_path = self.virtual_project_file_path();
        let proj_dir = virtual_proj_path.as_deref().and_then(Path::parent);
        self.gcode_panel.ensure_profiles_loaded(proj_dir);
        self.gcode_panel.rebuild_preview_if_needed(
            &self.project,
            self.active_setup_idx,
            cad_doc,
            cad_rev,
            &self.toolpath_cache,
            virtual_proj_path.as_deref(),
        );
        self.gcode_panel.trigger_export_action(
            &self.project,
            self.active_setup_idx,
            &self.toolpath_cache,
            cad_rev,
            &self.sim_panel,
            &mut self.logs,
        );
    }
}
