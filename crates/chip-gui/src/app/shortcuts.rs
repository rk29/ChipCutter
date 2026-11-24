use crate::{app::ChipCutterApp, panels::ViewportPickMode};

impl ChipCutterApp {
    pub(crate) fn sync_window_title(&mut self, ctx: &egui::Context) {
        let is_dirty_doc = self.is_project_dirty();
        let desired_title =
            crate::panels::about::format_window_title(&self.project.project_name, is_dirty_doc);
        if desired_title != self.last_window_title {
            self.last_window_title = desired_title.clone();
            ctx.send_viewport_cmd(egui::ViewportCommand::Title(desired_title));
        }
    }

    pub(crate) fn handle_global_shortcuts(&mut self, ctx: &egui::Context) {
        let (
            palette_shortcut,
            settings_shortcut,
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
                cmd && i.key_pressed(egui::Key::Comma),
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

        if settings_shortcut {
            self.modals.settings.open = !self.modals.settings.open;
            return;
        }

        if palette_shortcut {
            if self.modals.command_palette.open {
                self.modals.command_palette.open = false;
            } else {
                self.modals.command_palette.open_fresh();
            }
            return;
        }
        if f1_shortcut {
            self.modals.shortcut_cheatsheet.open = !self.modals.shortcut_cheatsheet.open;
            return;
        }
        if self.modals.command_palette.open {
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
                self.geometry_pick_target = crate::panels::GeometryPickTarget::CuttingArea;
                self.editing_shared_avoid_id = None;
                self.highlight.preview_tab_marker_wcs = None;
            } else if self.dispatcher.is_busy() {
                let elapsed = self.dispatcher.batch_elapsed();
                let n = self.dispatcher.cancel_all();
                if let Some(d) = elapsed {
                    self.logs.push(format!(
                        "Cancelled {n} in-flight generation job(s) after {}.",
                        chip_project::format_exec_duration(d)
                    ));
                } else {
                    self.logs
                        .push(format!("Cancelled {n} in-flight generation job(s)."));
                }
            } else if self.sim_panel.is_busy() {
                if let Some(d) = self.sim_panel.cancel_in_flight() {
                    self.logs.push(format!(
                        "Cancelled in-flight simulation after {}.",
                        chip_project::format_exec_duration(d)
                    ));
                } else {
                    self.logs
                        .push("Cancelled in-flight simulation.".to_string());
                }
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
                self.modals.shortcut_cheatsheet.open = !self.modals.shortcut_cheatsheet.open;
            }
        }
    }
}
