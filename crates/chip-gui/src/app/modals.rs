use crate::{
    app::ChipCutterApp,
    panels::{
        InspectorFocus,
        about::{AboutModalState, show_about_modal},
        command_palette::{
            CommandPaletteState, ShortcutCheatsheetState, show_command_palette,
            show_shortcut_cheatsheet_modal,
        },
        machine_manager::MachineManagerModalState,
        mesh_import_modal::MeshImportModalState,
        tool_library::{ToolLibraryMachineContext, ToolLibraryModalState},
        vector_import_modal::{VectorImportModalState, VectorModalAction, VectorModalCtx},
        welcome_gallery::{
            SampleGalleryModalState, WelcomeGalleryAction, WelcomeModalState,
            show_sample_gallery_modal, show_welcome_modal,
        },
    },
    prefs::save_prefs,
    theme::ThemeTokens,
};

/// Grouped modal and overlay dialog states owned by [`ChipCutterApp`].
pub struct ModalStates {
    pub(crate) tool_library: ToolLibraryModalState,
    pub(crate) machine_manager: MachineManagerModalState,
    pub(crate) mesh_import: MeshImportModalState,
    pub(crate) vector_import: VectorImportModalState,
    pub(crate) welcome: WelcomeModalState,
    pub(crate) sample_gallery: SampleGalleryModalState,
    pub(crate) command_palette: CommandPaletteState,
    pub(crate) shortcut_cheatsheet: ShortcutCheatsheetState,
    pub(crate) about: AboutModalState,
    pub(crate) show_unsaved_close: bool,
}

impl Default for ModalStates {
    fn default() -> Self {
        Self::new()
    }
}

impl ModalStates {
    pub fn new() -> Self {
        Self {
            tool_library: ToolLibraryModalState::new(),
            machine_manager: MachineManagerModalState::new(),
            mesh_import: MeshImportModalState::default(),
            vector_import: VectorImportModalState::default(),
            welcome: WelcomeModalState::default(),
            sample_gallery: SampleGalleryModalState::default(),
            command_palette: CommandPaletteState::default(),
            shortcut_cheatsheet: ShortcutCheatsheetState::default(),
            about: AboutModalState::default(),
            show_unsaved_close: false,
        }
    }
}

impl ChipCutterApp {
    pub(crate) fn show_primary_modals(&mut self, ctx: &egui::Context) {
        let active_mid = self.project.effective_machine_id().to_string();
        let tl_actions = self.modals.tool_library.show_with_machine(
            ctx,
            &mut self.global_tools,
            &mut self.project,
            &mut self.history,
            &mut self.logs,
            Some(ToolLibraryMachineContext {
                machines: &self.machines,
                active_machine_id: &active_mid,
            }),
        );
        if let Some(new_mid) = tl_actions.switch_machine_id {
            self.history
                .record_before_edit("Switch Target Machine", None, &self.project);
            self.project.machine_id = Some(new_mid);
            self.sync_active_machine_tools();
        }
        if tl_actions.clone_active_machine {
            let src_id = self.project.effective_machine_id().to_string();
            if let Some(src) = self
                .machines
                .iter()
                .find(|m| m.id.eq_ignore_ascii_case(&src_id))
                .cloned()
            {
                let existing: Vec<String> = self.machines.iter().map(|m| m.id.clone()).collect();
                let new_id = chip_tools::allocate_unique_custom_machine_id(&src.id, &existing);
                let new_name = format!("{} (Custom)", src.name);
                match chip_tools::clone_machine_definition(&src, &new_id, &new_name).and_then(
                    |cloned| {
                        chip_tools::save_user_machine(&cloned)?;
                        Ok(cloned)
                    },
                ) {
                    Ok(cloned) => {
                        let msg = format!(
                            "Cloned '{}' -> editable custom machine '{}' ({}).",
                            src.name, cloned.name, cloned.id
                        );
                        self.logs.push(format!("[MACHINES] {msg}"));
                        self.modals.tool_library.status_banner = Some(msg);
                        self.project.machine_id = Some(cloned.id.clone());
                        self.machines.push(cloned);
                        self.sync_active_machine_tools();
                    }
                    Err(err) => {
                        self.modals.tool_library.status_banner =
                            Some(format!("Clone error: {err}"));
                    }
                }
            }
        }
        if tl_actions.save_active_machine_tools {
            let cur_id = self.project.effective_machine_id().to_string();
            if let Some(m) = self
                .machines
                .iter_mut()
                .find(|m| !m.is_builtin && m.id.eq_ignore_ascii_case(&cur_id))
            {
                m.tools = self.global_tools.tools.clone();
                m.sort_tools_deterministically();
                if let Err(err) = chip_tools::save_user_machine(m) {
                    self.modals.tool_library.status_banner =
                        Some(format!("Save machine tools failed: {err}"));
                }
            }
        }
        if tl_actions.open_machine_manager {
            let cur_id = self.project.effective_machine_id().to_string();
            self.modals
                .machine_manager
                .open_selecting(&self.machines, &cur_id);
        }

        let active_mid_now = self.project.effective_machine_id().to_string();
        let default_pref_mid = self.prefs.default_machine_id.clone();
        let mm_actions = self.modals.machine_manager.show(
            ctx,
            &mut self.machines,
            &active_mid_now,
            &default_pref_mid,
            &mut self.logs,
        );
        if let Some((mid, sync_setups)) = mm_actions.select_project_machine {
            self.history
                .record_before_edit("Change Target Machine", None, &self.project);
            if let Some(m) = self
                .machines
                .iter()
                .find(|m| m.id.eq_ignore_ascii_case(&mid))
                .cloned()
            {
                self.project.apply_machine_defaults(&m, sync_setups);
            } else {
                self.project.machine_id = Some(mid);
            }
            self.sync_active_machine_tools();
        }
        if let Some(def_mid) = mm_actions.set_default_machine_id {
            self.prefs.default_machine_id = def_mid;
        }
        if let Some(tl_mid) = mm_actions.open_tool_library_for_machine {
            self.project.machine_id = Some(tl_mid);
            self.sync_active_machine_tools();
            self.modals.tool_library.open = true;
        }
        if mm_actions.machines_modified {
            self.sync_active_machine_tools();
        }

        if let Some(mesh_commit) = self
            .modals
            .mesh_import
            .show(ctx, self.project.display_units)
        {
            self.spawn_mesh_bytes_load(
                mesh_commit.source_name,
                mesh_commit.raw_bytes,
                mesh_commit.source_path,
                mesh_commit.unit_override,
                mesh_commit.transform,
                ctx.clone(),
            );
        }

        let proj_path_clone = self.project_file_path.clone();
        if let Some(vec_act) = self.modals.vector_import.show(
            ctx,
            VectorModalCtx {
                project: &mut self.project,
                project_file_path: proj_path_clone.as_deref(),
                active_setup_idx: self.active_setup_idx,
                global_tools: &self.global_tools,
                history: &mut self.history,
                logs: &mut self.logs,
            },
        ) {
            match vec_act {
                VectorModalAction::AppendedSketch { .. } => {
                    self.cached_frame_geo = None;
                    if self.model.is_none() {
                        self.fit_scene_to_viewport();
                    }
                }
                VectorModalAction::CreatedPure2dJob { .. } => {
                    self.prepare_for_project_replacement(None, None);
                    self.project_file_path = None;
                    self.inspector_focus = InspectorFocus::Sketch { sketch_idx: 0 };
                    self.sync_active_machine_tools();
                    self.refresh_dependent_expressions();
                    self.fit_scene_to_viewport();
                }
            }
        }
    }

    pub(crate) fn show_overlay_modals(&mut self, ctx: &egui::Context) {
        // Welcome Overlay, Sample Projects Gallery, Command Palette (Ctrl+K), & Shortcut Cheat-Sheet (F1 / ?)
        if let Some(w_act) = show_welcome_modal(ctx, &mut self.modals.welcome, &mut self.prefs) {
            match w_act {
                WelcomeGalleryAction::OpenProjectDialog => self.open_project_dialog(ctx),
                WelcomeGalleryAction::ImportStepDialog => self.open_step_dialog(ctx),
                WelcomeGalleryAction::ImportMeshDialog => {
                    self.modals.mesh_import.open_with_file_picker();
                }
                WelcomeGalleryAction::ImportVectorDialog => {
                    self.modals.vector_import.open_with_file_picker();
                }
                WelcomeGalleryAction::StartPure2dSketchJob => {
                    self.reset_blank_project("Pure-2D Sketch Job");
                    self.create_new_setup_xy_sketch();
                    self.fit_scene_to_viewport();
                }
                WelcomeGalleryAction::OpenSampleGallery => {
                    self.modals.sample_gallery.open = true;
                }
                WelcomeGalleryAction::LoadSample(id) => self.load_sample_project(id, ctx),
                WelcomeGalleryAction::OpenRecentPath(path) => {
                    self.open_any_path(&path, ctx.clone());
                }
                WelcomeGalleryAction::ClearRecentFiles => {
                    self.prefs.clear_recent_files();
                    if self.self_test.is_none() {
                        let _ = save_prefs(&self.prefs);
                        self.last_saved_prefs = self.prefs.clone();
                    }
                }
                WelcomeGalleryAction::PrefsChanged => {
                    if self.self_test.is_none() {
                        let _ = save_prefs(&self.prefs);
                        self.last_saved_prefs = self.prefs.clone();
                    }
                }
            }
        }

        if let Some(sample_id) = show_sample_gallery_modal(ctx, &mut self.modals.sample_gallery) {
            self.load_sample_project(sample_id, ctx);
        }

        if let Some(cmd) = show_command_palette(ctx, self.theme, &mut self.modals.command_palette) {
            self.execute_palette_command(cmd, ctx);
        }

        show_shortcut_cheatsheet_modal(ctx, &mut self.modals.shortcut_cheatsheet);

        let graphics_backend = self.wgpu_render_state.as_ref().map(|rs| {
            let info = rs.adapter.get_info();
            format!("{:?} ({})", info.backend, info.name)
        });
        show_about_modal(ctx, &mut self.modals.about, graphics_backend.as_deref());
    }

    pub(crate) fn show_unsaved_close_prompt(&mut self, ctx: &egui::Context) {
        if !self.modals.show_unsaved_close {
            return;
        }
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.modals.show_unsaved_close = false;
            return;
        }

        let tok = ThemeTokens::for_theme(self.theme);
        let mut open = true;
        egui::Window::new("Unsaved Changes")
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
            .fixed_size(egui::vec2(420.0, 0.0))
            .show(ctx, |ui| {
                ui.label(
                    egui::RichText::new(format!(
                        "Save changes to '{}' before closing?",
                        self.project.project_name
                    ))
                    .strong()
                    .size(14.0),
                );
                ui.add_space(4.0);
                if let Some(path) = &self.project_file_path {
                    ui.label(
                        egui::RichText::new(format!("Target file: {}", path.display()))
                            .small()
                            .color(tok.text_muted),
                    );
                } else {
                    ui.label(
                        egui::RichText::new(
                            "This project has not been saved to disk yet. Choosing Save & Close opens the Save Project dialog.",
                        )
                        .small()
                        .color(tok.text_muted),
                    );
                }
                ui.add_space(10.0);
                ui.separator();
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    if ui
                        .add(
                            egui::Button::new(
                                egui::RichText::new("Save & Close")
                                    .strong()
                                    .color(tok.accent_on_fg),
                            )
                            .fill(tok.accent),
                        )
                        .on_hover_text(
                            "Save the current project to disk and then exit ChipCutter.",
                        )
                        .clicked()
                        && self.save_project_dialog()
                    {
                        self.modals.show_unsaved_close = false;
                        self.allow_close_without_prompt = true;
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }

                    if ui
                        .add(
                            egui::Button::new(
                                egui::RichText::new("Discard & Close").color(tok.error_fg),
                            )
                            .fill(tok.panel_bg)
                            .stroke(egui::Stroke::new(1.0, tok.error_border)),
                        )
                        .on_hover_text(
                            "Exit ChipCutter immediately without saving unsaved project edits.",
                        )
                        .clicked()
                    {
                        self.modals.show_unsaved_close = false;
                        self.allow_close_without_prompt = true;
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .button("Cancel")
                            .on_hover_text("Keep ChipCutter open and return to the workspace.")
                            .clicked()
                        {
                            self.modals.show_unsaved_close = false;
                        }
                    });
                });
            });
        if !open {
            self.modals.show_unsaved_close = false;
        }
    }
}
