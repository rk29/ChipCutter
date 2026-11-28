use std::path::Path;

use chip_cam::{OperationKind, Setup, WorkOffsetSlot};
use chip_project::validate_project;

use crate::{
    app::ChipCutterApp,
    layout::{WorkspacePane, create_default_workspace_tree, focus_workspace_pane},
    panels::{InspectorFocus, ViewportPickMode, settings::SettingsSection},
    theme::ThemeTokens,
};

impl ChipCutterApp {
    pub(crate) fn draw_menu_bar(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
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
                    self.draw_simulate_menu(ui, ctx);
                    self.draw_post_menu(ui);
                    self.draw_view_menu(ui, ctx);
                    self.draw_help_menu(ui);

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if self.workers.loading_in_flight {
                            ui.label("Loading 3D Model…");
                            ui.spinner();
                        } else if let Some(m) = &self.assets.model {
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
                if self.session.prefs.recent_files.is_empty() {
                    ui.weak("No recent projects.");
                } else {
                    let recents = self.session.prefs.recent_files.clone();
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
                        self.session.prefs.clear_recent_files();
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
            if ui
                .button("Export All Setups...")
                .on_hover_text("Choose an output directory and batch-export sanitized G-code files for every Setup in this project")
                .clicked()
            {
                ui.close();
                self.trigger_export_all_setups_from_menu();
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
                self.modals.mesh_import.open_with_file_picker();
            }
            if ui
                .button("Import 2D Vector (.dxf / .svg)…")
                .on_hover_text(
                    "Import AutoCAD DXF or W3C SVG 2D vector artwork into a Sketch Layer or Pure-2D CNC job",
                )
                .clicked()
            {
                ui.close();
                self.modals.vector_import.open_with_file_picker();
            }
            if ui
                .add_enabled(
                    self.assets.model.is_some(),
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
                    self.assets.model.is_some() || self.project.primary_part_model().is_some(),
                    egui::Button::new("Replace STEP Model (Associative Re-Bind)…"),
                )
                .on_hover_text("Swap or locate the active project's STEP file while preserving CAM operations and geometry bindings")
                .on_disabled_hover_text("Import a 3D STEP model or open a 3D project first to replace or locate its model")
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
                self.modals.sample_gallery.open = true;
                ui.close();
            }
            if ui
                .button("Welcome & Quick-Start…")
                .on_hover_text("Open the Welcome overlay with recent projects and quick-start actions")
                .clicked()
            {
                self.modals.welcome.open = true;
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

            let focused_op = match self.selection.inspector_focus {
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
                    (self.selection.inspector_focus, self.selection.selected_sketch_entity_idx),
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
            ui.separator();
            if ui
                .button("Project Properties...")
                .on_hover_text(
                    "Edit project name, display units, target CNC machine, and inspect project summary",
                )
                .clicked()
            {
                self.modals
                    .project_properties
                    .open_for_project(&self.project.project_name);
                ui.close();
            }
            if ui
                .button("Manage Tools...")
                .on_hover_text("Open the Tool Library & Cutting Presets manager for the active machine")
                .clicked()
            {
                self.modals.tool_library.open = true;
                ui.close();
            }
            ui.separator();
            if ui
                .button("Application Settings... (Ctrl+,)")
                .on_hover_text(
                    "Configure application-level theme, units, mouse navigation, simulation engine/detail/skinning, and default machine",
                )
                .clicked()
            {
                self.modals.settings.open = true;
                ui.close();
            }
        });
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
                self.selection.active_setup_idx = self.project.setups.len() - 1;
                self.selection.inspector_focus = InspectorFocus::Setup {
                    setup_idx: self.selection.active_setup_idx,
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
                    self.assets.model.is_some(),
                    egui::Button::new("+ New 2D Sketch on 3D Face…"),
                )
                .on_hover_text("Pick a planar 3D CAD face in the viewport to host a new 2D sketch plane")
                .on_disabled_hover_text("Import a 3D STEP model first to attach a sketch to a 3D face")
                .clicked()
            {
                self.selection.pick_mode = ViewportPickMode::PickSketchHostFace {
                    setup_idx: self.selection.active_setup_idx,
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
            ui.separator();
            if ui
                .button("Flip-Jig Wizard…")
                .on_hover_text("Generate symmetric two-sided dowel-pin registration holes, Op-1 pin drilling, and a paired 180° Op-2 flip Setup")
                .clicked()
            {
                self.modals
                    .flip_jig_wizard
                    .open_for_setup(self.selection.active_setup_idx);
                ui.close();
            }
            if ui
                .button("Auto Soft-Jaw Wizard…")
                .on_hover_text("Generate contoured CNC aluminum soft-jaw machining ops and/or watertight 3D-printable .stl soft jaws")
                .clicked()
            {
                self.modals
                    .soft_jaw_wizard
                    .open_for_setup(self.selection.active_setup_idx);
                ui.close();
            }
            ui.separator();
            if ui
                .button("Machine Manager…")
                .on_hover_text(
                    "Configure CNC machine travels, spindle limits, 4th-axis rotary kinematics, and default tool libraries",
                )
                .clicked()
            {
                let active_mid = self.project.effective_machine_id().to_string();
                self.modals
                    .machine_manager
                    .open_selecting(&self.machines, &active_mid);
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
                    self.assets.model.as_ref().map(|m| m.doc.as_ref()),
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
                self.trigger_generate_dirty(Some(self.selection.active_setup_idx), None);
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
            if self.workers.dispatcher.is_busy()
                && ui
                    .button("Cancel Generation (Esc)")
                    .on_hover_text("Abort all currently running background toolpath calculations")
                    .clicked()
            {
                let elapsed = self.workers.dispatcher.batch_elapsed();
                let n = self.workers.dispatcher.cancel_all();
                if let Some(d) = elapsed {
                    self.logs.push(format!(
                        "Cancelled {n} in-flight generation job(s) after {}.",
                        chip_project::format_exec_duration(d)
                    ));
                } else {
                    self.logs
                        .push(format!("Cancelled {n} in-flight generation job(s)."));
                }
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
                {
                    self.add_standard_operation(kind);
                    ui.close();
                }
            }
        });
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
            if ui
                .button("Simulation Settings…")
                .on_hover_text("Configure Simulation Engine, Detail Level, Compute Backend, and Mesh Skinning")
                .clicked()
            {
                self.modals
                    .settings
                    .open_section(SettingsSection::Simulation);
                ui.close();
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
            if ui
                .button("Export All Setups...")
                .on_hover_text(
                    "Choose an output directory and batch-export sanitized G-code files for every Setup in this project",
                )
                .clicked()
            {
                ui.close();
                self.trigger_export_all_setups_from_menu();
            }
            if ui
                .button("Post Profile Inspector / Customizer...")
                .on_hover_text(
                    "Inspect read-only built-in .post.toml profiles or clone & customize profiles in user config space",
                )
                .clicked()
            {
                ui.close();
                self.open_post_profile_inspector(None);
            }
            ui.separator();
            let proj_dir = self
                .session.project_file_path
                .as_deref()
                .and_then(Path::parent)
                .or(self.session.bundle_extract_dir.as_deref());
            self.gcode_panel.ensure_profiles_loaded(proj_dir);
            if self.selection.active_setup_idx < self.project.setups.len() {
                ui.label("Active Setup Post Profile:");
                let cur_id = self.project.setups[self.selection.active_setup_idx]
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
                        self.project.setups[self.selection.active_setup_idx].post_profile = pid;
                        self.gcode_panel.invalidate_preview();
                        ui.close();
                    }
                }
            }
        });
    }

    fn draw_view_menu(&mut self, ui: &mut egui::Ui, _ctx: &egui::Context) {
        ui.menu_button("View", |ui| {
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
                &mut self.overlay_prefs.show_all_setup_toolpaths,
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
                &mut self.overlay_prefs.show_z_planes,
                "Show Z-Height Planes in Heights Tab",
            )
            .on_hover_text("Render color-coded Clearance, Retract, Top, and Bottom Z planes when editing Heights");
            ui.checkbox(
                &mut self.highlight.show_reach_heatmap,
                "Show Reach / Min-R Heatmap",
            )
            .on_hover_text("Highlight tight internal fillets, blocked narrow slots, and deep pockets exceeding flute or stickout reach");
            let mut measure_on = self.selection.pick_mode == ViewportPickMode::Measure;
            if ui
                .checkbox(&mut measure_on, "Measure Tool (M)")
                .on_hover_text("Click-to-Measure distances, hole diameters, Z depths, and dihedral angles in active Setup WCS")
                .changed()
            {
                self.toggle_measure_mode();
                ui.close();
            }
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
            if ui
                .button("Appearance & Navigation Settings…")
                .on_hover_text("Configure workspace theme, mouse navigation preset, and orbit mode")
                .clicked()
            {
                self.modals
                    .settings
                    .open_section(SettingsSection::Viewport);
                ui.close();
            }
        });
    }

    fn draw_help_menu(&mut self, ui: &mut egui::Ui) {
        ui.menu_button("Help", |ui| {
            if ui
                .button("Command Palette… (Ctrl+K)")
                .on_hover_text("Search and invoke any ChipCutter action, sample project, or setting by keyboard")
                .clicked()
            {
                self.modals.command_palette.open_fresh();
                ui.close();
            }
            if ui
                .button("Keyboard Shortcuts (F1 / ?)")
                .on_hover_text("View the complete reference of viewport, editing, and simulation hotkeys")
                .clicked()
            {
                self.modals.shortcut_cheatsheet.open = true;
                ui.close();
            }
            ui.separator();
            if ui
                .button("Sample Projects Gallery…")
                .on_hover_text("Browse and load built-in 2D, 3D, V-Carve, mesh, and 4th-axis sample projects")
                .clicked()
            {
                self.modals.sample_gallery.open = true;
                ui.close();
            }
            if ui
                .button("Welcome & Quick-Start…")
                .on_hover_text("Open the Welcome screen with recent projects and quick-start workflows")
                .clicked()
            {
                self.modals.welcome.open = true;
                ui.close();
            }
            ui.separator();
            if ui
                .button("About ChipCutter…")
                .on_hover_text("Show ChipCutter version, graphics backend, and open-source licensing details")
                .clicked()
            {
                self.modals.about.open = true;
                ui.close();
            }
        });
    }
}
