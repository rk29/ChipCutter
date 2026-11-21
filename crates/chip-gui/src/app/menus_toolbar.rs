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
        let is_dirty_doc = self.is_project_dirty();
        let desired_title =
            crate::panels::about::format_window_title(&self.project.project_name, is_dirty_doc);
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
            quit_shortcut,
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
                cmd && i.key_pressed(egui::Key::Q),
                i.key_pressed(egui::Key::Escape),
            )
        });

        if quit_shortcut {
            self.request_app_close(ctx);
            return;
        }

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
            let _ = self.save_project_dialog();
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
                        if ui
                            .button("Tool Library Manager…")
                            .on_hover_text(
                                "Browse, edit, import, and export cutting tools and feeds/speeds presets",
                            )
                            .clicked()
                        {
                            self.tool_library_modal.open = true;
                            ui.close();
                        }
                        if ui
                            .button("Machine Manager…")
                            .on_hover_text(
                                "Configure CNC machine travels, spindle limits, 4th-axis rotary kinematics, and default tool libraries",
                            )
                            .clicked()
                        {
                            let active_mid = self.project.effective_machine_id().to_string();
                            self.machine_manager_modal
                                .open_selecting(&self.machines, &active_mid);
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
            if ui
                .button("New Project")
                .on_hover_text("Start a fresh blank CAM project with your default machine settings")
                .clicked()
            {
                self.reset_blank_project("Untitled Job");
                ui.close();
            }
            if ui
                .button("Open Project / Bundle (.chip.toml, .chipz)… (Ctrl+O)")
                .on_hover_text("Open an existing ChipCutter project file or self-contained .chipz archive")
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
                            .on_hover_text("Reopen this recently used project or model file")
                            .on_disabled_hover_text("File could not be found at its last saved location")
                            .clicked()
                        {
                            ui.close();
                            self.open_any_path(&p, ctx.clone());
                        }
                    }
                    ui.separator();
                    if ui
                        .button("Clear Recent Files")
                        .on_hover_text("Remove all entries from the Open Recent menu")
                        .clicked()
                    {
                        self.prefs.clear_recent_files();
                        ui.close();
                    }
                }
            });
            if ui
                .button("Save Project (Ctrl+S)")
                .on_hover_text("Save the current project to disk (opens Save As if untitled)")
                .clicked()
            {
                ui.close();
                let _ = self.save_project_dialog();
            }
            if ui
                .button("Save Project As…")
                .on_hover_text("Save the current project to a new .chip.toml file path")
                .clicked()
            {
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
            if ui
                .button("Export G-Code (.nc / .ngc)…")
                .on_hover_text("Post-process the active Setup and save CNC G-code to disk")
                .clicked()
            {
                ui.close();
                self.trigger_gcode_export_from_menu();
            }
            ui.separator();
            if ui
                .button("Import STEP Model…")
                .on_hover_text("Import an ISO 10303 STEP AP214/AP203 B-Rep 3D CAD solid")
                .clicked()
            {
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
                .on_hover_text("Re-read the active STEP model from disk and associatively re-bind CAM selections")
                .on_disabled_hover_text("Import a 3D STEP model first to enable associative reload")
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
                .on_hover_text("Swap the active STEP file with a new revision while preserving CAM operations and geometry bindings")
                .on_disabled_hover_text("Import a 3D STEP model first to replace it with a new revision")
                .clicked()
            {
                ui.close();
                self.replace_step_dialog(ctx);
            }
            ui.separator();
            if ui
                .button("Sample Projects Gallery…")
                .on_hover_text("Browse and load built-in 2D, 3D, V-Carve, mesh, and 4th-axis sample projects")
                .clicked()
            {
                self.sample_gallery_modal.open = true;
                ui.close();
            }
            if ui
                .button("Welcome & Quick-Start…")
                .on_hover_text("Open the Welcome overlay with recent projects and quick-start actions")
                .clicked()
            {
                self.welcome_modal.open = true;
                ui.close();
            }
            ui.separator();
            if ui
                .button("Quit (Ctrl+Q)")
                .on_hover_text("Exit ChipCutter (prompts to save if there are unsaved changes)")
                .clicked()
            {
                ui.close();
                self.request_app_close(ctx);
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
                .on_hover_text("Revert the most recent project edit")
                .on_disabled_hover_text("Nothing to undo")
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
                .on_hover_text("Re-apply the most recently undone edit")
                .on_disabled_hover_text("Nothing to redo")
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
                .on_hover_text("Clone the selected CAM operation directly beneath it in the Setup")
                .on_disabled_hover_text("Select a CAM operation first to duplicate it")
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
                .on_hover_text("Run the selected operation earlier in the Setup sequence")
                .on_disabled_hover_text("Select an operation below the first slot to move it up")
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
                .on_hover_text("Run the selected operation later in the Setup sequence")
                .on_disabled_hover_text("Select an operation above the last slot to move it down")
                .clicked()
            {
                self.move_focused_operation(1);
                ui.close();
            }
            if ui
                .add_enabled(can_delete, egui::Button::new("Delete Selected (Del)"))
                .on_hover_text("Delete the currently selected operation or 2D sketch entity")
                .on_disabled_hover_text("Select an operation or 2D sketch entity to delete")
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
            if ui
                .button("New Setup")
                .on_hover_text("Create a new machining Setup with its own WCS origin, stock, and work offset (G54-G59)")
                .clicked()
            {
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
            if ui
                .button("+ New 2D Sketch (Setup XY)")
                .on_hover_text("Add a new 2D vector sketch layer on the active Setup's XY plane")
                .clicked()
            {
                self.create_new_setup_xy_sketch();
                ui.close();
            }
            if ui
                .add_enabled(
                    self.model.is_some(),
                    egui::Button::new("+ New 2D Sketch on 3D Face…"),
                )
                .on_hover_text("Pick a planar 3D CAD face in the viewport to host a new 2D sketch plane")
                .on_disabled_hover_text("Import a 3D STEP model first to attach a sketch to a 3D face")
                .clicked()
            {
                self.pick_mode = ViewportPickMode::PickSketchHostFace {
                    setup_idx: self.active_setup_idx,
                };
                ui.close();
            }
            if ui
                .button("Align View to Active Setup Top")
                .on_hover_text("Rotate the 3D viewport camera to look straight down the active Setup's +Z spindle axis")
                .clicked()
            {
                self.align_camera_to_active_setup();
                ui.close();
            }
            if ui
                .button("Validate Project Invariants")
                .on_hover_text("Run safety and configuration checks on tools, WCS frames, clearances, and selections")
                .clicked()
            {
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
                .on_hover_text("Compute or refresh all out-of-date toolpaths in the active Setup")
                .clicked()
            {
                self.trigger_generate_dirty(Some(self.active_setup_idx), None);
                ui.close();
            }
            if ui
                .button("Generate All Setups")
                .on_hover_text("Compute or refresh all out-of-date toolpaths across every Setup in the project")
                .clicked()
            {
                self.trigger_generate_dirty(None, None);
                ui.close();
            }
            if self.dispatcher.is_busy()
                && ui
                    .button("Cancel Generation (Esc)")
                    .on_hover_text("Abort all currently running background toolpath calculations")
                    .clicked()
            {
                self.dispatcher.cancel_all();
                ui.close();
            }
            ui.separator();
            if ui
                .button("+ 2D Adaptive / Trochoidal Clearing")
                .on_hover_text("Add a high-efficiency constant-engagement 2D adaptive roughing pocket operation")
                .clicked()
            {
                self.create_new_adaptive_operation();
                ui.close();
            }
            for kind in OperationKind::ALL {
                if ui
                    .button(format!("+ {}", kind.label()))
                    .on_hover_text(format!("Add a new {} operation to the active Setup", kind.label()))
                    .clicked()
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
            if ui
                .button("Run / Refresh Simulation")
                .on_hover_text("Run material removal simulation and 5-rule crash verification for the active Setup")
                .clicked()
            {
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
                .on_hover_text("Play or pause live cutter and stock-removal playback along the timeline")
                .on_disabled_hover_text("Run a simulation first to scrub or play back stock removal")
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
                .on_hover_text("Scrub directly to the end of the simulated Setup to inspect the finished part")
                .on_disabled_hover_text("Run a simulation first to inspect the final machined stock")
                .clicked()
            {
                if let Some(res) = &self.sim_panel.active_result {
                    self.sim_panel.current_time_sec = res.timeline.total_time_sec;
                    self.sim_panel.is_playing = false;
                }
                ui.close();
            }
            ui.separator();
            ui.label("Engine:");
            for eng in chip_sim::SimEngineKind::ALL {
                if ui
                    .radio_value(&mut self.sim_panel.engine, eng, eng.label())
                    .on_hover_text(eng.description())
                    .changed()
                    && self.sim_panel.active_result.is_some()
                {
                    self.trigger_active_setup_simulation(ctx);
                }
            }
            ui.separator();
            ui.label("Resolution:");
            let active_eng = self.sim_panel.engine;
            for res in SimResolution::ALL {
                ui.radio_value(
                    &mut self.sim_panel.resolution,
                    res,
                    res.label_for_engine(active_eng),
                )
                .on_hover_text(res.description_for_engine(active_eng));
            }
            ui.separator();
            ui.checkbox(
                &mut self.sim_panel.show_machined_stock,
                "Show 3D Machined Stock",
            )
            .on_hover_text("Display the simulated carved stock solid inside the 3D viewport");
            ui.checkbox(
                &mut self.sim_panel.color_by_operation,
                "Color Cuts by Operation",
            )
            .on_hover_text("Tint carved stock faces by the operation that last cut them (overridden when Error Heatmap is active)");
            ui.checkbox(
                &mut self.sim_panel.show_error_heatmap,
                "Error Deviation Heatmap",
            )
            .on_hover_text("Color machined stock by signed error against the CAD model (red = gouge, green = in-tolerance, blue = rest stock). Uncheck for faster playback.");
            ui.checkbox(
                &mut self.sim_panel.show_cutter_assembly,
                "Show 3D Cutter & Holder",
            )
            .on_hover_text("Render the spinning cutter, shank, and tool holder at the active timeline position");
        });
    }

    fn draw_post_menu(&mut self, ui: &mut egui::Ui) {
        ui.menu_button("Post", |ui| {
            if ui
                .button("Open G-Code Preview Tab")
                .on_hover_text("Focus the G-Code Preview & Export pane in the lower dock")
                .clicked()
            {
                focus_workspace_pane(&mut self.tree, WorkspacePane::GCodePreview);
                ui.close();
            }
            if ui
                .button("Export G-Code (.nc / .ngc)…")
                .on_hover_text(
                    "Run safety-gated G-code post-processing and save the program to disk",
                )
                .clicked()
            {
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
                    if ui
                        .selectable_label(is_sel, &pname)
                        .on_hover_text(format!(
                            "Assign '{pname}' as the G-code post-processor for the active Setup"
                        ))
                        .clicked()
                        && !is_sel
                    {
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
                if ui
                    .selectable_label(is_sel, t.label())
                    .on_hover_text(format!("Switch workspace palette to {}", t.label()))
                    .clicked()
                    && !is_sel
                {
                    self.set_theme(ctx, t);
                    ui.close();
                }
            }
            ui.separator();
            if ui
                .button("Reset Default Workspace Layout")
                .on_hover_text("Restore default panel docking sizes and tab positions")
                .clicked()
            {
                self.tree = create_default_workspace_tree();
                ui.close();
            }
            ui.separator();
            ui.checkbox(
                &mut self.highlight.show_toolpaths,
                "Show 3D Generated Toolpaths",
            )
            .on_hover_text("Toggle visibility of all generated cutting toolpaths in the 3D viewport");
            ui.checkbox(&mut self.highlight.show_rapids, "Show Rapid Moves (G0)")
                .on_hover_text("Toggle dashed yellow rapid transit lines (G0) between cuts");
            ui.checkbox(
                &mut self.show_all_setup_toolpaths,
                "Show All Operations in Active Setup",
            )
            .on_hover_text("When checked, display toolpaths for every visible operation in the Setup; when unchecked, show only the selected operation");
            ui.separator();
            ui.checkbox(
                &mut self.highlight.show_grid,
                "Show Active Setup WCS XY Grid",
            )
            .on_hover_text("Display the reference coordinate grid on the active Setup's Z = 0 plane");
            ui.checkbox(&mut self.highlight.show_stock, "Show Translucent Stock")
                .on_hover_text("Overlay the translucent raw stock bounding block around the part");
            ui.checkbox(&mut self.highlight.ghost_part, "Ghost Part Translucency")
                .on_hover_text("Render the 3D CAD solid semi-transparent so internal pockets, holes, and toolpaths remain visible");
            ui.checkbox(
                &mut self.show_z_planes,
                "Show Z-Height Planes in Heights Tab",
            )
            .on_hover_text("Render color-coded Clearance, Retract, Top, and Bottom Z planes when editing Heights");
            ui.separator();
            if ui
                .button("Toggle Orthographic / Perspective (P)")
                .on_hover_text("Switch the 3D camera between parallel orthographic and perspective projection")
                .clicked()
            {
                self.camera.toggle_projection();
                ui.close();
            }
            if ui
                .button("Fit Model to Viewport (F)")
                .on_hover_text("Frame the entire CAD model, stock, and sketches cleanly inside the viewport")
                .clicked()
            {
                self.fit_scene_to_viewport();
                ui.close();
            }
            if ui
                .button("Align View to Active Setup Top (+Z)")
                .on_hover_text("Orient the camera straight down the active Setup's +Z spindle axis")
                .clicked()
            {
                self.align_camera_to_active_setup();
                ui.close();
            }
            ui.separator();
            ui.menu_button("Mouse Navigation Preset", |ui| {
                for preset in MousePreset::ALL {
                    ui.selectable_value(&mut self.camera.mouse_preset, preset, preset.label())
                        .on_hover_text(format!("Use {} 3D viewport orbit/pan/zoom mouse bindings", preset.label()));
                }
                ui.separator();
                for om in [OrbitMode::TurntableZUp, OrbitMode::FreeTrackball] {
                    let is_sel = self.camera.orbit_mode == om;
                    if ui
                        .selectable_label(is_sel, om.label())
                        .on_hover_text(match om {
                            OrbitMode::TurntableZUp => "Keep world +Z upright while orbiting around the part",
                            OrbitMode::FreeTrackball => "Allow unrestricted 3-axis trackball rotation in any direction",
                        })
                        .clicked()
                        && !is_sel
                    {
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
                        ui.selectable_value(&mut u_sel, DisplayUnit::Inch, "in")
                            .on_hover_text("Display lengths and feed rates in Imperial inches (in, IPM, SFM)");
                        ui.selectable_value(&mut u_sel, DisplayUnit::Millimeter, "mm")
                            .on_hover_text("Display lengths and feed rates in Metric millimeters (mm, mm/min, m/min)");
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
            if ui
                .button("Command Palette… (Ctrl+K)")
                .on_hover_text("Search and invoke any ChipCutter action, sample project, or setting by keyboard")
                .clicked()
            {
                self.command_palette.open_fresh();
                ui.close();
            }
            if ui
                .button("Keyboard Shortcuts (F1 / ?)")
                .on_hover_text("View the complete reference of viewport, editing, and simulation hotkeys")
                .clicked()
            {
                self.shortcut_cheatsheet.open = true;
                ui.close();
            }
            ui.separator();
            if ui
                .button("Sample Projects Gallery…")
                .on_hover_text("Browse and load built-in 2D, 3D, V-Carve, mesh, and 4th-axis sample projects")
                .clicked()
            {
                self.sample_gallery_modal.open = true;
                ui.close();
            }
            if ui
                .button("Welcome & Quick-Start…")
                .on_hover_text("Open the Welcome screen with recent projects and quick-start workflows")
                .clicked()
            {
                self.welcome_modal.open = true;
                ui.close();
            }
            ui.separator();
            if ui
                .button("About ChipCutter…")
                .on_hover_text("Show ChipCutter version, graphics backend, and open-source licensing details")
                .clicked()
            {
                self.about_modal.open = true;
                ui.close();
            }
        });
    }

    pub(crate) fn reset_blank_project(&mut self, name: &str) {
        self.prepare_for_project_replacement(None, None);
        let mut doc = ProjectDocument::new(name);
        doc.display_units = self.prefs.default_units;
        if self.prefs.default_machine_id != chip_tools::DEFAULT_MACHINE_ID
            && let Some(m) = self
                .machines
                .iter()
                .find(|m| m.id.eq_ignore_ascii_case(&self.prefs.default_machine_id))
                .cloned()
        {
            doc.apply_machine_defaults(&m, true);
        }
        self.project = doc;
        self.project_file_path = None;
        self.sync_active_machine_tools();
        self.refresh_dependent_expressions();
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
            CommandId::SaveProject => {
                let _ = self.save_project_dialog();
            }
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
            CommandId::OpenMachineManager => {
                let active_mid = self.project.effective_machine_id().to_string();
                self.machine_manager_modal
                    .open_selecting(&self.machines, &active_mid);
            }
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
            CommandId::OpenAboutModal => self.about_modal.open = true,
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
                        if ui
                            .small_button("Reload & Re-Match (Ctrl+R)")
                            .on_hover_text("Reload the modified STEP model from disk and associatively re-bind CAM faces and edges")
                            .clicked()
                        {
                            self.reload_active_step_model(ctx);
                        }
                        if ui
                            .small_button("Dismiss")
                            .on_hover_text("Hide this file-change notification without reloading")
                            .clicked()
                        {
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
                        if ui
                            .small_button("Regenerate Dirty (Ctrl+G)")
                            .on_hover_text("Recompute all toolpaths invalidated by the CAD reload and refresh simulation")
                            .clicked()
                        {
                            self.auto_sim_after_gen = true;
                            self.trigger_generate_dirty(None, None);
                        }
                        if ui
                            .small_button("Dismiss")
                            .on_hover_text("Close this reload summary banner")
                            .clicked()
                        {
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
