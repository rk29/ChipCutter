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
        machine_tool_swap::MachineToolSwapModalState,
        mesh_import_modal::MeshImportModalState,
        post_profile_inspector::PostProfileInspectorModalState,
        project_properties::{
            ProjectPropertiesCtx, ProjectPropertiesModalState, show_project_properties_modal,
        },
        settings::{SettingsModalCtx, SettingsModalState, show_settings_modal},
        tool_library::{ToolLibraryMachineContext, ToolLibraryModalState},
        vector_import_modal::{VectorImportModalState, VectorModalAction, VectorModalCtx},
        welcome_gallery::{
            SampleGalleryModalState, WelcomeGalleryAction, WelcomeModalState,
            show_sample_gallery_modal, show_welcome_modal,
        },
        workholding_wizards::{FlipJigWizardModalState, SoftJawWizardModalState},
    },
    prefs::save_prefs,
    theme::ThemeTokens,
};

/// Grouped modal and overlay dialog states owned by [`ChipCutterApp`].
pub struct ModalStates {
    pub(crate) tool_library: ToolLibraryModalState,
    pub(crate) machine_manager: MachineManagerModalState,
    pub(crate) post_profile_inspector: PostProfileInspectorModalState,
    pub(crate) machine_tool_swap: MachineToolSwapModalState,
    pub(crate) project_properties: ProjectPropertiesModalState,
    pub(crate) settings: SettingsModalState,
    pub(crate) mesh_import: MeshImportModalState,
    pub(crate) vector_import: VectorImportModalState,
    pub(crate) welcome: WelcomeModalState,
    pub(crate) sample_gallery: SampleGalleryModalState,
    pub(crate) command_palette: CommandPaletteState,
    pub(crate) shortcut_cheatsheet: ShortcutCheatsheetState,
    pub(crate) about: AboutModalState,
    pub(crate) flip_jig_wizard: FlipJigWizardModalState,
    pub(crate) soft_jaw_wizard: SoftJawWizardModalState,
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
            post_profile_inspector: PostProfileInspectorModalState::new(),
            machine_tool_swap: MachineToolSwapModalState::default(),
            project_properties: ProjectPropertiesModalState::default(),
            settings: SettingsModalState::default(),
            mesh_import: MeshImportModalState::default(),
            vector_import: VectorImportModalState::default(),
            welcome: WelcomeModalState::default(),
            sample_gallery: SampleGalleryModalState::default(),
            command_palette: CommandPaletteState::default(),
            shortcut_cheatsheet: ShortcutCheatsheetState::default(),
            about: AboutModalState::default(),
            flip_jig_wizard: FlipJigWizardModalState::default(),
            soft_jaw_wizard: SoftJawWizardModalState::default(),
            show_unsaved_close: false,
        }
    }
}

impl ChipCutterApp {
    /// Applies side-effect actions emitted by either the `Edit -> Project Properties...` modal
    /// or the right-hand **Project Inspector** pane (`InspectorFocus::Project`).
    pub(crate) fn apply_project_properties_actions(
        &mut self,
        pp_actions: crate::panels::project_properties::ProjectPropertiesActions,
    ) {
        if let Some(new_name) = pp_actions.rename_project {
            let trimmed = new_name.trim();
            if !trimmed.is_empty() && trimmed != self.project.project_name {
                self.history
                    .record_before_edit("Rename Project", None, &self.project);
                self.project.project_name = trimmed.to_string();
                self.logs.push(format!(
                    "Renamed project to '{}'.",
                    self.project.project_name
                ));
            }
        }
        if let Some(new_units) = pp_actions.display_units_changed
            && new_units != self.project.display_units
        {
            self.history
                .record_before_edit("Change Display Units", None, &self.project);
            self.project.display_units = new_units;
            self.refresh_dependent_expressions();
        }
        if let Some(new_mid) = pp_actions.switch_machine_id {
            self.request_machine_switch(new_mid, false);
        }
        if pp_actions.open_machine_manager {
            let cur_id = self.project.effective_machine_id().to_string();
            self.modals
                .machine_manager
                .open_selecting(&self.machines, &cur_id);
        }
    }

    /// Opens the Tool Library Manager, optionally focusing and scrolling to `select_tool_id`.
    pub(crate) fn open_tool_library_for_tool(
        &mut self,
        select_tool_id: Option<&chip_tools::ToolId>,
    ) {
        self.modals.tool_library.open = true;
        if let Some(tid) = select_tool_id {
            if !self.global_tools.tools.iter().any(|t| &t.id == tid)
                && let Some(pt) = self.project.tool(tid).cloned()
            {
                self.global_tools.tools.push(pt);
            }
            if let Some(idx) = self.global_tools.tools.iter().position(|t| &t.id == tid) {
                self.modals.tool_library.selected_tool_idx = idx;
                self.modals.tool_library.selected_preset_idx = 0;
                self.modals.tool_library.scroll_to_selected = true;
            }
        }
    }

    /// Unified machine-switching entry point called by Setup Inspector, Machine Manager, and
    /// Tool Library. If existing operations use tools absent from the target machine, opens the
    /// interactive **Swap Machine Tools** dialog; otherwise switches immediately.
    pub(crate) fn request_machine_switch(&mut self, target_machine_id: String, sync_setups: bool) {
        let current_mid = self.project.effective_machine_id().to_string();
        if current_mid.eq_ignore_ascii_case(&target_machine_id) && !sync_setups {
            return;
        }
        let Some(target_machine) = self
            .machines
            .iter()
            .find(|m| m.id.eq_ignore_ascii_case(&target_machine_id))
            .cloned()
        else {
            self.history
                .record_before_edit("Change Target Machine", None, &self.project);
            self.project.machine_id = Some(target_machine_id);
            self.sync_active_machine_tools();
            return;
        };

        let plan = self.project.plan_machine_tool_swap(&target_machine);
        if plan.needs_user_prompt() {
            let source_name = self
                .machines
                .iter()
                .find(|m| m.id.eq_ignore_ascii_case(&current_mid))
                .map_or_else(|| current_mid.clone(), |m| m.name.clone());
            self.modals.machine_tool_swap.open_for_plan(
                source_name,
                target_machine,
                plan,
                sync_setups,
            );
        } else {
            self.history
                .record_before_edit("Change Target Machine", None, &self.project);
            self.project
                .apply_machine_defaults(&target_machine, sync_setups);
            self.sync_active_machine_tools();
            self.cached_frame_geo = None;
            self.logs.push(format!(
                "[MACHINES] Switched project machine to '{}'.",
                target_machine.name
            ));
        }
    }

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
            self.request_machine_switch(new_mid, false);
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
            self.request_machine_switch(mid, sync_setups);
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

        if self.modals.post_profile_inspector.open {
            let proj_dir = self.active_project_dir().map(std::path::Path::to_path_buf);
            self.gcode_panel.ensure_profiles_loaded(proj_dir.as_deref());
            let active_post_id = self
                .project
                .setups
                .get(self.active_setup_idx)
                .map_or_else(|| "grbl".to_string(), |s| s.post_profile.clone());
            let post_actions = self.modals.post_profile_inspector.show(
                ctx,
                self.gcode_panel.available_profiles_mut(),
                &active_post_id,
                &mut self.logs,
            );
            if let Some(new_pid) = post_actions.select_setup_post_id
                && self.active_setup_idx < self.project.setups.len()
            {
                self.history
                    .record_before_edit("Change Post Profile", None, &self.project);
                self.project.setups[self.active_setup_idx].post_profile = new_pid;
                self.gcode_panel.invalidate_preview();
            }
            if post_actions.profiles_modified {
                self.gcode_panel.refresh_profiles(proj_dir.as_deref());
                self.gcode_panel.invalidate_preview();
            }
        }

        if let Some(commit) = self.modals.machine_tool_swap.show(ctx, self.theme) {
            self.history
                .record_before_edit("Switch Machine & Swap Tools", None, &self.project);
            let summary = self.project.apply_machine_tool_swap(
                &commit.target_machine,
                commit.sync_setups,
                &commit.choices,
                commit.prune_replaced_unused,
            );
            self.sync_active_machine_tools();
            self.cached_frame_geo = None;
            self.logs.push(format!(
                "[MACHINES] Switched to '{}': swapped {} tool(s), updated {} op(s), removed {} unused tool(s).",
                commit.target_machine.name,
                summary.replaced_tool_count,
                summary.updated_operation_count,
                summary.pruned_unused_count
            ));
        }

        let pp_actions = show_project_properties_modal(
            ctx,
            &mut self.modals.project_properties,
            ProjectPropertiesCtx {
                project: &self.project,
                project_file_path: self.project_file_path.as_deref(),
                machines: &self.machines,
            },
        );
        self.apply_project_properties_actions(pp_actions);

        let gpu_info_owned = self.sim_panel.gpu_adapter_info().cloned();
        let settings_actions = show_settings_modal(
            ctx,
            &mut self.modals.settings,
            SettingsModalCtx {
                theme: self.theme,
                project_units: self.project.display_units,
                prefs: &mut self.prefs,
                mouse_preset: &mut self.camera.mouse_preset,
                orbit_mode: &mut self.camera.orbit_mode,
                orthographic: self.camera.projection.is_orthographic(),
                show_z_planes: &mut self.show_z_planes,
                show_all_setup_toolpaths: &mut self.show_all_setup_toolpaths,
                show_grid: &mut self.highlight.show_grid,
                sim_engine: &mut self.sim_panel.engine,
                sim_resolution: &mut self.sim_panel.resolution,
                sim_backend: &mut self.sim_panel.backend_choice,
                sim_mesh_skinning: &mut self.sim_panel.mesh_skinning,
                gpu_adapter_info: gpu_info_owned.as_ref(),
                machines: &self.machines,
            },
        );
        if let Some(new_theme) = settings_actions.theme_changed {
            self.set_theme(ctx, new_theme);
        }
        if let Some(new_units) = settings_actions.project_units_changed {
            self.history
                .record_before_edit("Change Display Units", None, &self.project);
            self.project.display_units = new_units;
            self.refresh_dependent_expressions();
        }
        if let Some(new_orbit) = settings_actions.orbit_mode_changed {
            self.camera.set_orbit_mode(new_orbit);
        }
        if let Some(want_ortho) = settings_actions.projection_changed
            && want_ortho != self.camera.projection.is_orthographic()
        {
            self.camera.toggle_projection();
        }
        if settings_actions.reset_workspace_layout {
            self.tree = crate::layout::create_default_workspace_tree();
            self.prefs.layout_shares = Default::default();
        }
        if settings_actions.sim_rerun_requested
            && !self.sim_panel.is_busy()
            && self.sim_panel.active_result.is_some()
        {
            self.trigger_active_setup_simulation(ctx);
        }
        if settings_actions.sim_skinning_changed {
            ctx.request_repaint();
        }
        if settings_actions.open_machine_manager {
            let cur_id = self.project.effective_machine_id().to_string();
            self.modals
                .machine_manager
                .open_selecting(&self.machines, &cur_id);
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

        // Workholding Automation Wizards (Flip-Jig Dowel Pins & Auto Soft-Jaws)
        let wizard_cad = self
            .model
            .as_ref()
            .map(|m| std::sync::Arc::clone(&m.doc))
            .unwrap_or_else(|| std::sync::Arc::clone(&self.empty_cad));
        if let Some(res) = self.modals.flip_jig_wizard.show(
            ctx,
            &mut self.project,
            wizard_cad.as_ref(),
            &mut self.history,
        ) {
            self.active_setup_idx = res.new_setup_idx;
            self.inspector_focus = InspectorFocus::Setup {
                setup_idx: res.new_setup_idx,
            };
            self.highlight.show_fixtures = true;
            self.cached_frame_geo = None;
            self.logs.push(res.log_message);
        }
        if let Some(res) = self.modals.soft_jaw_wizard.show(
            ctx,
            &mut self.project,
            wizard_cad.as_ref(),
            &mut self.history,
        ) {
            if let Some(s_idx) = res.new_setup_idx {
                self.active_setup_idx = s_idx;
                self.inspector_focus = InspectorFocus::Setup { setup_idx: s_idx };
                self.cached_frame_geo = None;
            }
            self.logs.push(res.log_message);
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
