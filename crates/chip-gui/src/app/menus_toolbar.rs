use std::path::Path;

use chip_cam::{Operation, OperationKind, Setup, WorkOffsetSlot};
use chip_math::DisplayUnit;
use chip_project::{ProjectDocument, validate_project};
use chip_sim::SimResolution;

use crate::{
    app::ChipCutterApp,
    camera::{MousePreset, OrbitMode},
    layout::{WorkspacePane, create_default_workspace_tree, focus_workspace_pane},
    panels::{
        InspectorFocus, OperationTab, ViewportPickMode,
        browser::{
            default_pick_mode_for_op, delete_operation_in_setup, duplicate_operation_in_setup,
            move_operation_to_slot,
        },
        command_palette::CommandId,
    },
    theme::{AppTheme, ThemeTokens},
    widgets::{IconKind, StatusBadgeKind, icon_button, primary_icon_button, status_badge_icon},
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
            palette_shortcut,
            f1_shortcut,
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
                cmd && i.key_pressed(egui::Key::K),
                i.key_pressed(egui::Key::F1),
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

        if palette_shortcut {
            if self.command_palette.open {
                self.command_palette.open = false;
            } else {
                self.command_palette.open_fresh();
            }
            return;
        }
        if f1_shortcut {
            self.shortcut_cheatsheet.open = !self.shortcut_cheatsheet.open;
            return;
        }
        if self.command_palette.open {
            return;
        }

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
            if self.sketch_author_start_xy.take().is_some() {
                // Cancelled first rubber-band click while staying in SketchAuthor tool
            } else if self.pick_mode != ViewportPickMode::InspectEntity {
                self.pick_mode = ViewportPickMode::InspectEntity;
                self.highlight.preview_tab_marker_wcs = None;
            } else if self.dispatcher.is_busy() {
                let n = self.dispatcher.cancel_all();
                self.logs
                    .push(format!("Cancelled {n} in-flight generation job(s)."));
            }
        }

        if !ctx.egui_wants_keyboard_input() {
            let (dup_op, move_up, move_down, delete_key) = ctx.input(|i| {
                let cmd = i.modifiers.command || i.modifiers.ctrl;
                (
                    cmd && i.key_pressed(egui::Key::D),
                    cmd && i.key_pressed(egui::Key::ArrowUp),
                    cmd && i.key_pressed(egui::Key::ArrowDown),
                    !cmd && (i.key_pressed(egui::Key::Delete)
                        || (i.key_pressed(egui::Key::Backspace)
                            && self.pick_mode == ViewportPickMode::InspectEntity
                            && self.sketch_author_start_xy.is_none())),
                )
            });
            if dup_op {
                self.duplicate_focused_operation();
            }
            if move_up {
                self.move_focused_operation(-1);
            }
            if move_down {
                self.move_focused_operation(1);
            }
            if delete_key {
                self.delete_focused_item();
            }
            if ctx.input(|i| !i.modifiers.any() && i.key_pressed(egui::Key::P)) {
                self.camera.toggle_projection();
            }
            if ctx.input(|i| !i.modifiers.any() && i.key_pressed(egui::Key::F)) {
                self.fit_scene_to_viewport();
            }
            let typed_question = ctx.input(|i| {
                i.events
                    .iter()
                    .any(|ev| matches!(ev, egui::Event::Text(t) if t == "?"))
            });
            if typed_question {
                self.shortcut_cheatsheet.open = !self.shortcut_cheatsheet.open;
            }
        }
    }

    fn draw_menu_bar(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let tok = ThemeTokens::for_theme(self.theme);
        egui::Panel::top("chip_menu_bar")
            .frame(
                egui::Frame::NONE
                    .fill(tok.menubar_bg)
                    .stroke(egui::Stroke::new(1.0, tok.border))
                    .inner_margin(egui::Margin::symmetric(8, 3)),
            )
            .show(ui, |ui| {
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
                    self.draw_view_menu(ui, ctx);
                    self.draw_help_menu(ui);

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if self.loading_in_flight {
                            ui.label("Loading 3D Model…");
                            ui.spinner();
                        } else if let Some(m) = &self.model {
                            if let Some(mesh) = m.doc.meshes.first() {
                                ui.weak(format!(
                                    "{} · {} tris · {}",
                                    m.doc.source_name,
                                    mesh.diagnostics.valid_triangle_count,
                                    if mesh.diagnostics.is_watertight {
                                        "Watertight"
                                    } else {
                                        "Open edges"
                                    }
                                ));
                            } else {
                                ui.weak(format!(
                                    "{} · {} faces · {} edges",
                                    m.doc.source_name,
                                    m.doc.faces.len(),
                                    m.doc.edges.len()
                                ));
                            }
                        } else if !self.project.sketches.is_empty() {
                            ui.weak(format!(
                                "Pure-2D Job · {} sketch layer(s)",
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
                self.reset_blank_project("Untitled Job");
                ui.close();
            }
            if ui
                .button("Open Project / Bundle (.chip.toml, .chipz)… (Ctrl+O)")
                .clicked()
            {
                ui.close();
                self.open_project_dialog(ctx);
            }
            ui.menu_button("Open Recent", |ui| {
                if self.prefs.recent_files.is_empty() {
                    ui.weak("No recent projects.");
                } else {
                    let recents = self.prefs.recent_files.clone();
                    for p in recents {
                        let exists = p.exists();
                        let lbl = p.display().to_string();
                        if ui
                            .add_enabled(exists, egui::Button::new(lbl))
                            .clicked()
                        {
                            ui.close();
                            self.open_any_path(&p, ctx.clone());
                        }
                    }
                    ui.separator();
                    if ui.button("Clear Recent Files").clicked() {
                        self.prefs.clear_recent_files();
                        ui.close();
                    }
                }
            });
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
                .button("Import 3D Mesh (.stl / .obj / .3mf)…")
                .on_hover_text(
                    "Import a sculpted 3D triangle mesh with live physical unit & manifold preview",
                )
                .clicked()
            {
                ui.close();
                self.mesh_import_modal.open_with_file_picker();
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
            if ui.button("Sample Projects Gallery…").clicked() {
                self.sample_gallery_modal.open = true;
                ui.close();
            }
            if ui.button("Welcome & Quick-Start…").clicked() {
                self.welcome_modal.open = true;
                ui.close();
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

            let focused_op = match self.inspector_focus {
                InspectorFocus::Operation { setup_idx, op_idx }
                    if setup_idx < self.project.setups.len()
                        && op_idx < self.project.setups[setup_idx].operations.len() =>
                {
                    Some((setup_idx, op_idx))
                }
                _ => None,
            };
            let can_delete = focused_op.is_some()
                || matches!(
                    (self.inspector_focus, self.selected_sketch_entity_idx),
                    (InspectorFocus::Sketch { sketch_idx }, Some(e_idx))
                        if sketch_idx < self.project.sketches.len()
                            && e_idx < self.project.sketches[sketch_idx].entities.len()
                );

            ui.separator();
            if ui
                .add_enabled(
                    focused_op.is_some(),
                    egui::Button::new("Duplicate Operation (Ctrl+D)"),
                )
                .clicked()
            {
                self.duplicate_focused_operation();
                ui.close();
            }
            if ui
                .add_enabled(
                    focused_op.is_some_and(|(_, o)| o > 0),
                    egui::Button::new("Move Operation Up (Ctrl+Up)"),
                )
                .clicked()
            {
                self.move_focused_operation(-1);
                ui.close();
            }
            if ui
                .add_enabled(
                    focused_op
                        .is_some_and(|(s, o)| o + 1 < self.project.setups[s].operations.len()),
                    egui::Button::new("Move Operation Down (Ctrl+Down)"),
                )
                .clicked()
            {
                self.move_focused_operation(1);
                ui.close();
            }
            if ui
                .add_enabled(can_delete, egui::Button::new("Delete Selected (Del)"))
                .clicked()
            {
                self.delete_focused_item();
                ui.close();
            }
        });
    }

    pub(crate) fn duplicate_focused_operation(&mut self) {
        if let InspectorFocus::Operation { setup_idx, op_idx } = self.inspector_focus
            && setup_idx < self.project.setups.len()
            && op_idx < self.project.setups[setup_idx].operations.len()
        {
            self.history
                .record_before_edit("Duplicate Operation", None, &self.project);
            if let Some(new_idx) =
                duplicate_operation_in_setup(&mut self.project.setups[setup_idx], op_idx)
            {
                self.inspector_focus = InspectorFocus::Operation {
                    setup_idx,
                    op_idx: new_idx,
                };
            }
        }
    }

    pub(crate) fn move_focused_operation(&mut self, delta: isize) {
        if let InspectorFocus::Operation { setup_idx, op_idx } = self.inspector_focus
            && setup_idx < self.project.setups.len()
        {
            let len = self.project.setups[setup_idx].operations.len();
            let slot_opt = if delta < 0 && op_idx > 0 {
                Some(op_idx - 1)
            } else if delta > 0 && op_idx + 1 < len {
                Some(op_idx + 2)
            } else {
                None
            };
            if let Some(slot) = slot_opt {
                self.history
                    .record_before_edit("Reorder Operation", None, &self.project);
                if let Some(dst_idx) =
                    move_operation_to_slot(&mut self.project.setups[setup_idx], op_idx, slot)
                {
                    self.inspector_focus = InspectorFocus::Operation {
                        setup_idx,
                        op_idx: dst_idx,
                    };
                }
            }
        }
    }

    pub(crate) fn delete_focused_item(&mut self) {
        match self.inspector_focus {
            InspectorFocus::Operation { setup_idx, op_idx }
                if setup_idx < self.project.setups.len()
                    && op_idx < self.project.setups[setup_idx].operations.len() =>
            {
                self.history
                    .record_before_edit("Delete Operation", None, &self.project);
                if delete_operation_in_setup(&mut self.project.setups[setup_idx], op_idx) {
                    let rem = self.project.setups[setup_idx].operations.len();
                    self.inspector_focus = if rem > 0 {
                        InspectorFocus::Operation {
                            setup_idx,
                            op_idx: op_idx.min(rem - 1),
                        }
                    } else {
                        InspectorFocus::Setup { setup_idx }
                    };
                    self.pick_mode = ViewportPickMode::InspectEntity;
                }
            }
            InspectorFocus::Sketch { sketch_idx } if sketch_idx < self.project.sketches.len() => {
                if let Some(e_idx) = self.selected_sketch_entity_idx
                    && e_idx < self.project.sketches[sketch_idx].entities.len()
                {
                    self.history
                        .record_before_edit("Delete Sketch Entity", None, &self.project);
                    let eval_ctx = self.eval_context();
                    let project_dir = self.active_project_dir();
                    let font_map = self.project.load_custom_fonts(project_dir);
                    let fonts_opt = (!font_map.is_empty()).then_some(&font_map);
                    let sk = &mut self.project.sketches[sketch_idx];
                    sk.entities.remove(e_idx);
                    sk.reevaluate(&eval_ctx);
                    sk.rebuild_regions(fonts_opt);
                    let rem = sk.entities.len();
                    self.selected_sketch_entity_idx = if rem > 0 {
                        Some(e_idx.min(rem - 1))
                    } else {
                        None
                    };
                    self.cached_frame_geo = None;
                }
            }
            _ => {}
        }
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
            if ui.button("+ New 2D Sketch (Setup XY)").clicked() {
                self.create_new_setup_xy_sketch();
                ui.close();
            }
            if ui
                .add_enabled(
                    self.model.is_some(),
                    egui::Button::new("+ New 2D Sketch on 3D Face…"),
                )
                .clicked()
            {
                self.pick_mode = ViewportPickMode::PickSketchHostFace {
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
                    self.op_tab = if kind == OperationKind::Facing || kind.is_3d_surface() {
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

    fn draw_view_menu(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        ui.menu_button("View", |ui| {
            ui.label("Workspace Theme:");
            for t in AppTheme::ALL {
                let is_sel = self.theme == t;
                if ui.selectable_label(is_sel, t.label()).clicked() && !is_sel {
                    self.set_theme(ctx, t);
                    ui.close();
                }
            }
            ui.separator();
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
            ui.separator();
            ui.menu_button("Mouse Navigation Preset", |ui| {
                for preset in MousePreset::ALL {
                    ui.selectable_value(&mut self.camera.mouse_preset, preset, preset.label());
                }
                ui.separator();
                for om in [OrbitMode::TurntableZUp, OrbitMode::FreeTrackball] {
                    let is_sel = self.camera.orbit_mode == om;
                    if ui.selectable_label(is_sel, om.label()).clicked() && !is_sel {
                        self.camera.set_orbit_mode(om);
                        ui.close();
                    }
                }
            });
        });
    }

    fn draw_quick_toolbar(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let tok = ThemeTokens::for_theme(self.theme);
        egui::Panel::top("chip_toolbar")
            .frame(
                egui::Frame::NONE
                    .fill(tok.toolbar_bg)
                    .stroke(egui::Stroke::new(1.0, tok.border))
                    .inner_margin(egui::Margin::symmetric(8, 5)),
            )
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 5.0;

                    if icon_button(
                        ui,
                        IconKind::FolderOpen,
                        Some("Open"),
                        "Open .chip.toml Project or .chipz Bundle (Ctrl+O)",
                    )
                    .clicked()
                    {
                        self.open_project_dialog(ctx);
                    }
                    if icon_button(
                        ui,
                        IconKind::Save,
                        Some("Save"),
                        "Save .chip.toml Project (Ctrl+S)",
                    )
                    .clicked()
                    {
                        self.save_project_dialog();
                    }
                    if icon_button(
                        ui,
                        IconKind::BundleZip,
                        Some(".chipz"),
                        "Export Self-Contained Portable Bundle (.chipz)",
                    )
                    .clicked()
                    {
                        self.export_chipz_dialog();
                    }
                    if icon_button(
                        ui,
                        IconKind::VectorImport,
                        Some("DXF/SVG"),
                        "Import 2D Vector Drawing (.dxf / .svg) or Start Pure-2D Job",
                    )
                    .clicked()
                    {
                        self.vector_import_modal.open = true;
                    }
                    if self.model.is_some()
                        && icon_button(
                            ui,
                            IconKind::Reload,
                            None,
                            "Associatively Reload Active STEP Model (Ctrl+R)",
                        )
                        .clicked()
                    {
                        self.reload_active_step_model(ctx);
                    }

                    ui.add_enabled_ui(self.history.can_undo(), |ui| {
                        if icon_button(ui, IconKind::Undo, None, "Undo (Ctrl+Z)").clicked() {
                            self.perform_undo();
                        }
                    });
                    ui.add_enabled_ui(self.history.can_redo(), |ui| {
                        if icon_button(ui, IconKind::Redo, None, "Redo (Ctrl+Y)").clicked() {
                            self.perform_redo();
                        }
                    });

                    ui.separator();

                    if icon_button(
                        ui,
                        IconKind::Sketch2D,
                        Some("+ Sketch"),
                        "Create a New 2D Sketch Layer on Active Setup XY",
                    )
                    .clicked()
                    {
                        self.create_new_setup_xy_sketch();
                    }
                    if icon_button(
                        ui,
                        IconKind::Adaptive2D,
                        Some("+ Adaptive"),
                        "Add a 2D Adaptive / Trochoidal Clearing Operation",
                    )
                    .clicked()
                    {
                        self.create_new_adaptive_operation();
                    }
                    if icon_button(
                        ui,
                        IconKind::VCarve,
                        Some("+ V-Carve"),
                        "Add a 3D Medial-Axis V-Carve / Flat Clearance / V-Inlay Operation",
                    )
                    .clicked()
                    {
                        self.create_new_vcarve_operation();
                    }

                    ui.separator();

                    if self.dispatcher.is_busy() {
                        ui.spinner();
                        let cancel_lbl = format!("Cancel ({})", self.dispatcher.in_flight_count());
                        if icon_button(
                            ui,
                            IconKind::CancelStop,
                            Some(&cancel_lbl),
                            "Cancel In-Flight Toolpath Generation (Esc)",
                        )
                        .clicked()
                        {
                            self.dispatcher.cancel_all();
                        }
                    } else if primary_icon_button(
                        ui,
                        IconKind::Generate,
                        "Generate",
                        "Generate Dirty Toolpaths in Active Setup (Ctrl+G)",
                    )
                    .clicked()
                    {
                        self.trigger_generate_dirty(Some(self.active_setup_idx), None);
                    }

                    if icon_button(
                        ui,
                        IconKind::Simulate,
                        Some("Simulate"),
                        "Run 2.5D Z-Dexel Heightmap Stock Removal & 5-Rule Crash Verification",
                    )
                    .clicked()
                    {
                        focus_workspace_pane(&mut self.tree, WorkspacePane::SimulationTimeline);
                        self.trigger_active_setup_simulation(ctx);
                    }
                    if icon_button(
                        ui,
                        IconKind::PostGCode,
                        Some("Post"),
                        "Open G-Code Preview & Safety-Gated Export",
                    )
                    .clicked()
                    {
                        focus_workspace_pane(&mut self.tree, WorkspacePane::GCodePreview);
                    }
                    if icon_button(
                        ui,
                        IconKind::Tool,
                        Some("Tools"),
                        "Open Tool Library Manager",
                    )
                    .clicked()
                    {
                        self.tool_library_modal.open = true;
                    }

                    // Right-aligned Theme & Display Units controls
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.spacing_mut().item_spacing.x = 5.0;
                        let (theme_icon, theme_tip) = match self.theme {
                            AppTheme::Dark => (
                                IconKind::Sun,
                                "Switch to Daylight High-Contrast Light Theme",
                            ),
                            AppTheme::Light => (IconKind::Moon, "Switch to Studio Dark Theme"),
                        };
                        if icon_button(ui, theme_icon, Some(self.theme.short_label()), theme_tip)
                            .clicked()
                        {
                            self.set_theme(ctx, self.theme.toggled());
                        }

                        ui.separator();

                        let mut u_sel = self.project.display_units;
                        ui.selectable_value(&mut u_sel, DisplayUnit::Inch, "in");
                        ui.selectable_value(&mut u_sel, DisplayUnit::Millimeter, "mm");
                        if u_sel != self.project.display_units {
                            self.history.record_before_edit(
                                "Change Display Units",
                                None,
                                &self.project,
                            );
                            self.project.display_units = u_sel;
                            self.prefs.default_units = u_sel;
                            self.refresh_dependent_expressions();
                        }

                        ui.separator();

                        if icon_button(
                            ui,
                            IconKind::Search,
                            Some("Ctrl+K"),
                            "Open Fuzzy Command Palette (Ctrl+K)",
                        )
                        .clicked()
                        {
                            self.command_palette.open_fresh();
                        }
                    });
                });
            });
    }

    fn draw_help_menu(&mut self, ui: &mut egui::Ui) {
        ui.menu_button("Help", |ui| {
            if ui.button("Command Palette… (Ctrl+K)").clicked() {
                self.command_palette.open_fresh();
                ui.close();
            }
            if ui.button("Keyboard Shortcuts (F1 / ?)").clicked() {
                self.shortcut_cheatsheet.open = true;
                ui.close();
            }
            ui.separator();
            if ui.button("Sample Projects Gallery…").clicked() {
                self.sample_gallery_modal.open = true;
                ui.close();
            }
            if ui.button("Welcome & Quick-Start…").clicked() {
                self.welcome_modal.open = true;
                ui.close();
            }
        });
    }

    pub(crate) fn reset_blank_project(&mut self, name: &str) {
        self.dispatcher.cancel_all();
        self.sim_panel.cancel_in_flight();
        self.sim_panel.active_result = None;
        self.gcode_panel.invalidate_preview();
        self.clear_bundle_extract_dir();
        self.history.clear();
        self.clean_undo_len = Some(0);
        self.last_reload_banner = None;
        self.external_step_modified = false;
        let mut doc = ProjectDocument::new(name);
        doc.display_units = self.prefs.default_units;
        self.project = doc;
        self.project_file_path = None;
        self.model = None;
        self.active_setup_idx = 0;
        self.inspector_focus = InspectorFocus::Setup { setup_idx: 0 };
        self.cached_frame_geo = None;
    }

    pub(crate) fn add_standard_operation(&mut self, kind: OperationKind) {
        if self.active_setup_idx >= self.project.setups.len() {
            return;
        }
        self.history
            .record_before_edit(format!("Add {}", kind.label()), None, &self.project);
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
            .chain(self.global_tools.tools.iter())
            .find(|t| kind.is_compatible_with_tool(t))
            .cloned()
        {
            self.project.ensure_tool_embedded(&t);
            op.tool_id = Some(t.id);
            op.preset_id = t.presets.first().map(|p| p.id.clone());
        }
        self.project.setups[s_idx].operations.push(op);
        let o_idx = self.project.setups[s_idx].operations.len() - 1;
        self.inspector_focus = InspectorFocus::Operation {
            setup_idx: s_idx,
            op_idx: o_idx,
        };
        self.op_tab = if kind == OperationKind::Facing || kind.is_3d_surface() {
            OperationTab::Passes
        } else {
            OperationTab::Geometry
        };
        self.pick_mode = default_pick_mode_for_op(s_idx, o_idx, kind, self.op_tab);
    }

    pub(crate) fn execute_palette_command(&mut self, cmd: CommandId, ctx: &egui::Context) {
        if let Some(sample_id) = cmd.as_sample() {
            self.load_sample_project(sample_id, ctx);
            return;
        }
        match cmd {
            CommandId::NewProject => self.reset_blank_project("Untitled Job"),
            CommandId::OpenProject => self.open_project_dialog(ctx),
            CommandId::SaveProject => self.save_project_dialog(),
            CommandId::ExportChipzBundle => self.export_chipz_dialog(),
            CommandId::ImportStepModel => self.open_step_dialog(ctx),
            CommandId::ImportMesh3D => self.mesh_import_modal.open_with_file_picker(),
            CommandId::ReloadStepModel => self.reload_active_step_model(ctx),
            CommandId::ImportVector2D => self.vector_import_modal.open_with_file_picker(),
            CommandId::OpenWelcomeScreen => self.welcome_modal.open = true,
            CommandId::OpenSampleGallery => self.sample_gallery_modal.open = true,
            CommandId::NewSetup => {
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
            }
            CommandId::NewSketchSetupXy => self.create_new_setup_xy_sketch(),
            CommandId::NewSketchOn3dFace => {
                if self.model.is_some() {
                    self.pick_mode = ViewportPickMode::PickSketchHostFace {
                        setup_idx: self.active_setup_idx,
                    };
                }
            }
            CommandId::AddOpAdaptive2D => self.create_new_adaptive_operation(),
            CommandId::AddOpFacing => self.add_standard_operation(OperationKind::Facing),
            CommandId::AddOpContour2D => self.add_standard_operation(OperationKind::Contour2D),
            CommandId::AddOpPocket2D => self.add_standard_operation(OperationKind::Pocket2D),
            CommandId::AddOpDrillBore => self.add_standard_operation(OperationKind::DrillBore),
            CommandId::AddOpChamferTrace => {
                self.add_standard_operation(OperationKind::ChamferTrace)
            }
            CommandId::AddOpVCarve => self.create_new_vcarve_operation(),
            CommandId::AddOpAdaptive3D => self.add_standard_operation(OperationKind::Adaptive3D),
            CommandId::AddOpWaterline3D => self.add_standard_operation(OperationKind::Waterline3D),
            CommandId::AddOpParallel3D => self.add_standard_operation(OperationKind::Parallel3D),
            CommandId::AddOpScallop3D => self.add_standard_operation(OperationKind::Scallop3D),
            CommandId::AddOpPencil3D => self.add_standard_operation(OperationKind::Pencil3D),
            CommandId::AddOpRotary2Linear => {
                self.add_standard_operation(OperationKind::Rotary2Linear);
            }
            CommandId::AddOpRotary4AxisSim => {
                self.add_standard_operation(OperationKind::Rotary4AxisSim);
            }
            CommandId::DuplicateOperation => self.duplicate_focused_operation(),
            CommandId::MoveOperationUp => self.move_focused_operation(-1),
            CommandId::MoveOperationDown => self.move_focused_operation(1),
            CommandId::DeleteSelected => self.delete_focused_item(),
            CommandId::Undo => self.perform_undo(),
            CommandId::Redo => self.perform_redo(),
            CommandId::GenerateActiveSetup => {
                self.trigger_generate_dirty(Some(self.active_setup_idx), None);
            }
            CommandId::GenerateAllSetups => self.trigger_generate_dirty(None, None),
            CommandId::RunSimulation => {
                focus_workspace_pane(&mut self.tree, WorkspacePane::SimulationTimeline);
                self.trigger_active_setup_simulation(ctx);
            }
            CommandId::OpenGCodePreview => {
                focus_workspace_pane(&mut self.tree, WorkspacePane::GCodePreview);
            }
            CommandId::ExportGCode => self.trigger_gcode_export_from_menu(),
            CommandId::OpenToolLibrary => self.tool_library_modal.open = true,
            CommandId::OpenDiagnosticsLog => {
                focus_workspace_pane(&mut self.tree, WorkspacePane::DiagnosticsLog);
            }
            CommandId::FitToViewport => self.fit_scene_to_viewport(),
            CommandId::ToggleOrthoPerspective => self.camera.toggle_projection(),
            CommandId::AlignViewSetupTop => self.align_camera_to_active_setup(),
            CommandId::ResetWorkspaceLayout => {
                self.tree = create_default_workspace_tree();
                self.prefs.layout_shares = Default::default();
            }
            CommandId::ToggleThemeDarkLight => self.set_theme(ctx, self.theme.toggled()),
            CommandId::SwitchUnitsMillimeter => {
                if self.project.display_units != DisplayUnit::Millimeter {
                    self.history
                        .record_before_edit("Set Units to mm", None, &self.project);
                    self.project.display_units = DisplayUnit::Millimeter;
                    self.prefs.default_units = DisplayUnit::Millimeter;
                    self.refresh_dependent_expressions();
                }
            }
            CommandId::SwitchUnitsInch => {
                if self.project.display_units != DisplayUnit::Inch {
                    self.history
                        .record_before_edit("Set Units to in", None, &self.project);
                    self.project.display_units = DisplayUnit::Inch;
                    self.prefs.default_units = DisplayUnit::Inch;
                    self.refresh_dependent_expressions();
                }
            }
            CommandId::OpenShortcutCheatsheet => self.shortcut_cheatsheet.open = true,
            CommandId::SampleCamTorture
            | CommandId::SampleAdaptiveRest
            | CommandId::SampleVCarveInlay
            | CommandId::SampleEngravedNameplate
            | CommandId::SamplePure2dPanel
            | CommandId::SampleSculptedMeshDome
            | CommandId::SampleReliefWaveMesh
            | CommandId::SampleRotaryIndexedShaft
            | CommandId::SampleRotaryWrappedEngraving
            | CommandId::SampleRotaryHelicalFlute
            | CommandId::SampleRevisionReloadV2 => {}
        }
    }

    fn create_new_vcarve_operation(&mut self) {
        if self.active_setup_idx >= self.project.setups.len() {
            return;
        }
        self.history
            .record_before_edit("Add V-Carve", None, &self.project);
        let s_idx = self.active_setup_idx;
        let n = self.project.setups[s_idx].operations.len() + 1;
        let mut op = Operation::new(
            format!("op_{s_idx}_{n}_vcarve"),
            format!("V-Carve {n}"),
            OperationKind::VCarve,
        );
        if let Some(t) = self
            .project
            .tools
            .iter()
            .chain(self.global_tools.tools.iter())
            .find(|t| OperationKind::VCarve.is_compatible_with_tool(t))
            .cloned()
        {
            self.project.ensure_tool_embedded(&t);
            op.tool_id = Some(t.id);
            op.preset_id = t.presets.first().map(|p| p.id.clone());
        }
        self.project.setups[s_idx].operations.push(op);
        let o_idx = self.project.setups[s_idx].operations.len() - 1;
        self.inspector_focus = InspectorFocus::Operation {
            setup_idx: s_idx,
            op_idx: o_idx,
        };
        self.op_tab = OperationTab::Geometry;
        self.pick_mode = default_pick_mode_for_op(s_idx, o_idx, OperationKind::VCarve, self.op_tab);
    }

    fn draw_status_banners(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let tok = ThemeTokens::for_theme(self.theme);
        if self.external_step_modified {
            egui::Panel::top("chip_external_step_banner")
                .frame(
                    egui::Frame::NONE
                        .fill(tok.dirty_bg)
                        .stroke(egui::Stroke::new(1.0, tok.dirty_border))
                        .inner_margin(egui::Margin::symmetric(10, 4)),
                )
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        status_badge_icon(ui, StatusBadgeKind::Dirty, None);
                        ui.colored_label(
                            tok.dirty_fg,
                            "External STEP file updated on disk — CAD revision change detected.",
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
            let (bg, border, fg, badge) = if has_warn {
                (
                    tok.warn_bg,
                    tok.warn_border,
                    tok.warn_fg,
                    StatusBadgeKind::NeedsRepick,
                )
            } else {
                (
                    tok.clean_bg,
                    tok.clean_border,
                    tok.clean_fg,
                    StatusBadgeKind::Clean,
                )
            };
            egui::Panel::top("chip_reload_summary_banner")
                .frame(
                    egui::Frame::NONE
                        .fill(bg)
                        .stroke(egui::Stroke::new(1.0, border))
                        .inner_margin(egui::Margin::symmetric(10, 4)),
                )
                .show(ui, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        status_badge_icon(ui, badge, None);
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
