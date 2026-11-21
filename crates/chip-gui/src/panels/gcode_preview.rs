use std::{
    hash::{Hash as _, Hasher as _},
    path::Path,
    sync::Arc,
};

use chip_cad::CadDocument;
use chip_post::{
    PostOutputFile, PostProfile, PostProfileSource, load_available_post_profiles,
    validate_safe_output_filename,
};
use chip_project::{
    ProjectDocument, SetupPostRequest, ToolpathCache, UndoHistory, post_process_project_setup,
};
use chip_sim::{SimDiagnostic, SimSeverity};

use crate::panels::simulation::SimulationPanelState;

/// Actions emitted by `[G-Code Preview]` back to `ChipCutterApp`.
#[derive(Debug, Clone)]
pub enum GCodeUiAction {
    RunSimulationFirst,
    JumpToCrash(SimDiagnostic),
}

/// Context references passed to [`GCodePreviewPanelState::show_ui`] and `rebuild_preview_if_needed`.
pub struct GCodePanelCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub history: &'a mut UndoHistory,
    pub setup_idx: usize,
    pub cad_doc: Option<&'a Arc<CadDocument>>,
    pub cad_revision: u64,
    pub cache: &'a ToolpathCache,
    pub project_file_path: Option<&'a Path>,
    pub sim_state: &'a SimulationPanelState,
    pub logs: &'a mut Vec<String>,
}

#[derive(Default)]
pub struct GCodePreviewPanelState {
    available_profiles: Vec<PostProfile>,
    custom_loaded_profiles: Vec<PostProfile>,
    cached_sig: Option<u64>,
    pub cached_files: Vec<PostOutputFile>,
    pub cached_lines_per_file: Vec<Vec<String>>,
    pub last_error: Option<String>,
    pub selected_file_tab: usize,
    show_safety_gate_modal: bool,
    pending_unverified_warning: bool,
}

impl GCodePreviewPanelState {
    pub fn ensure_profiles_loaded(&mut self, project_dir: Option<&Path>) {
        if self.available_profiles.is_empty() {
            self.refresh_profiles(project_dir);
        }
    }

    pub fn refresh_profiles(&mut self, project_dir: Option<&Path>) {
        let mut list = load_available_post_profiles(project_dir);
        for custom in &self.custom_loaded_profiles {
            if let Some(existing) = list.iter_mut().find(|p| p.id == custom.id) {
                *existing = custom.clone();
            } else {
                list.push(custom.clone());
            }
        }
        self.available_profiles = list;
        self.cached_sig = None;
    }

    pub fn custom_profiles(&self) -> &[PostProfile] {
        &self.custom_loaded_profiles
    }

    pub fn available_profiles(&self) -> &[PostProfile] {
        &self.available_profiles
    }

    pub fn invalidate_preview(&mut self) {
        self.cached_sig = None;
        self.cached_files.clear();
        self.cached_lines_per_file.clear();
        self.last_error = None;
        self.selected_file_tab = 0;
        self.show_safety_gate_modal = false;
        self.pending_unverified_warning = false;
    }

    /// Clears cached G-code preview state, safety-gate modal flags, and project-loaded custom
    /// post profiles when switching projects, then refreshes available profiles for `project_dir`.
    pub fn reset_for_project(&mut self, project_dir: Option<&Path>) {
        self.custom_loaded_profiles.clear();
        self.invalidate_preview();
        self.refresh_profiles(project_dir);
    }

    fn compute_preview_signature(
        &self,
        project: &ProjectDocument,
        setup_idx: usize,
        cache: &ToolpathCache,
        cad_revision: u64,
    ) -> u64 {
        let mut h = std::collections::hash_map::DefaultHasher::new();
        cad_revision.hash(&mut h);
        setup_idx.hash(&mut h);
        project.project_name.hash(&mut h);
        if let Some(setup) = project.setups.get(setup_idx) {
            setup.name.hash(&mut h);
            setup.post_profile.hash(&mut h);
            format!("{:?}", setup.wcs_slot).hash(&mut h);
            for (o_idx, op) in setup.operations.iter().enumerate() {
                op.suppressed.hash(&mut h);
                op.name.hash(&mut h);
                if !op.suppressed
                    && let Some((hk, _)) =
                        cache.get_for_operation(project, setup_idx, o_idx, cad_revision)
                {
                    hk.0.hash(&mut h);
                }
            }
        }
        h.finish()
    }

    pub(crate) fn rebuild_preview_if_needed(
        &mut self,
        project: &ProjectDocument,
        setup_idx: usize,
        cad_doc: Option<&Arc<CadDocument>>,
        cad_revision: u64,
        cache: &ToolpathCache,
        project_file_path: Option<&Path>,
    ) {
        let sig = self.compute_preview_signature(project, setup_idx, cache, cad_revision);
        if self.cached_sig == Some(sig) {
            return;
        }
        self.cached_sig = Some(sig);
        self.cached_files.clear();
        self.cached_lines_per_file.clear();
        self.last_error = None;

        let Some(doc) = cad_doc else {
            self.last_error = Some("Load a STEP model or 2D Sketch first.".to_string());
            return;
        };
        let Some(setup) = project.setups.get(setup_idx) else {
            self.last_error = Some("No active Setup.".to_string());
            return;
        };

        let profile = self
            .available_profiles
            .iter()
            .find(|p| p.id.eq_ignore_ascii_case(&setup.post_profile))
            .or_else(|| self.available_profiles.first())
            .cloned();
        let Some(post_profile) = profile else {
            self.last_error = Some("No post-processor profile available.".to_string());
            return;
        };

        let proj_label = project_file_path
            .and_then(|p| p.file_name())
            .and_then(|s| s.to_str())
            .unwrap_or(&project.project_name);
        let stem = project_file_path
            .and_then(|p| p.file_stem())
            .and_then(|s| s.to_str())
            .map(|s| s.trim_end_matches(".chip"))
            .unwrap_or(&project.project_name);

        match post_process_project_setup(&SetupPostRequest {
            project,
            setup_idx,
            cad_doc: doc.as_ref(),
            cad_revision,
            cache,
            post_profile: &post_profile,
            project_label: proj_label,
            program_stem: stem,
        }) {
            Ok(files) => {
                self.cached_lines_per_file = files
                    .iter()
                    .map(|f| f.content.lines().map(ToOwned::to_owned).collect())
                    .collect();
                self.cached_files = files;
                if self.selected_file_tab >= self.cached_files.len() {
                    self.selected_file_tab = 0;
                }
            }
            Err(e) => {
                self.last_error = Some(e);
            }
        }
    }

    pub fn show_ui(&mut self, ui: &mut egui::Ui, ctx: GCodePanelCtx<'_>) -> Option<GCodeUiAction> {
        let GCodePanelCtx {
            project,
            history,
            setup_idx,
            cad_doc,
            cad_revision,
            cache,
            project_file_path,
            sim_state,
            logs,
        } = ctx;
        let mut action: Option<GCodeUiAction> = None;
        let proj_dir = project_file_path.and_then(|p| p.parent());
        self.ensure_profiles_loaded(proj_dir);

        // Top Bar: Post-Processor Profile Selector, Load Custom .post.toml, Safety Status, & Export Button
        ui.horizontal_wrapped(|ui| {
            ui.label("Post Profile:");
            if setup_idx < project.setups.len() {
                let current_id = project.setups[setup_idx].post_profile.clone();
                let current_label = self
                    .available_profiles
                    .iter()
                    .find(|p| p.id.eq_ignore_ascii_case(&current_id))
                    .map(|p| p.name.as_str())
                    .unwrap_or(&current_id);

                let mut chosen_id = current_id.clone();
                egui::ComboBox::from_id_salt("gcode_post_profile_combo")
                    .selected_text(current_label)
                    .width(240.0)
                    .show_ui(ui, |ui| {
                        for prof in &self.available_profiles {
                            ui.selectable_value(&mut chosen_id, prof.id.clone(), &prof.name)
                                .on_hover_text(format!("Post-process active Setup with '{}'", prof.name));
                        }
                    })
                    .response
                    .on_hover_text("Select CNC controller post-processor profile for G-code preview and export");
                if chosen_id != current_id {
                    history.record_before_edit("Change Post Profile", None, project);
                    project.setups[setup_idx].post_profile = chosen_id;
                    self.cached_sig = None;
                }
            }

            if ui
                .small_button("Load .post.toml…")
                .on_hover_text("Load a custom TOML post-processor profile from disk")
                .clicked()
                && let Some(path) = rfd::FileDialog::new()
                    .add_filter("ChipCutter Post Profile", &["toml"])
                    .pick_file()
            {
                match PostProfile::load_from_path(&path) {
                    Ok(prof) => {
                        let pid = prof.id.clone();
                        let pname = prof.name.clone();
                        self.custom_loaded_profiles.push(prof);
                        self.refresh_profiles(proj_dir);
                        if setup_idx < project.setups.len() {
                            history.record_before_edit("Load Custom Post Profile", None, project);
                            project.setups[setup_idx].post_profile = pid;
                        }
                        logs.push(format!(
                            "Loaded custom post profile '{pname}' from '{}'.",
                            path.display()
                        ));
                    }
                    Err(e) => {
                        logs.push(format!(
                            "ERROR loading post profile '{}': {e}",
                            path.display()
                        ));
                    }
                }
            }

            ui.separator();

            let verified =
                sim_state.is_verified_up_to_date(project, setup_idx, cache, cad_revision);
            let crit_count = sim_state
                .active_result
                .as_ref()
                .map_or(0, |r| r.critical_error_count());

            let tok = crate::theme::ThemeTokens::from_ui(ui);
            if verified && crit_count == 0 {
                crate::widgets::status_badge_icon(ui, crate::widgets::StatusBadgeKind::Clean, None);
                ui.colored_label(tok.clean_fg, "Sim Verified (0 Crashes)");
            } else if crit_count > 0 {
                crate::widgets::status_badge_icon(ui, crate::widgets::StatusBadgeKind::Error, None);
                ui.colored_label(
                    tok.error_fg,
                    format!("{crit_count} Critical Crash/Gouge(s)!"),
                );
            } else {
                crate::widgets::status_badge_icon(ui, crate::widgets::StatusBadgeKind::Dirty, None);
                ui.colored_label(tok.dirty_fg, "Unverified — Run Simulation before machining");
            }

            let can_export = !self.cached_files.is_empty();
            if ui
                .add_enabled(can_export, egui::Button::new("Export G-Code…"))
                .on_hover_text("Save posted CNC G-code program(s) to disk (opens safety gate if unverified or collisions exist)")
                .on_disabled_hover_text("Generate clean toolpaths in the active Setup first to preview and export G-code")
                .clicked()
            {
                if crit_count > 0 {
                    self.show_safety_gate_modal = true;
                    self.pending_unverified_warning = false;
                } else if !verified {
                    self.show_safety_gate_modal = true;
                    self.pending_unverified_warning = true;
                } else {
                    self.export_files_with_dialog(logs);
                }
            }
        });

        self.rebuild_preview_if_needed(
            project,
            setup_idx,
            cad_doc,
            cad_revision,
            cache,
            project_file_path,
        );

        // Safety Gate Confirmation Modal (`06-simulation.md` §4)
        if self.show_safety_gate_modal {
            let mut open = true;
            egui::Window::new(if self.pending_unverified_warning {
                "Simulation Not Yet Verified"
            } else {
                "CRITICAL CRASH / GOUGE DETECTED — EXPORT BLOCKED"
            })
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .open(&mut open)
            .show(ui.ctx(), |ui| {
                let tok = crate::theme::ThemeTokens::from_ui(ui);
                ui.set_max_width(480.0);
                if self.pending_unverified_warning {
                    ui.colored_label(
                        tok.warn_fg,
                        "Heightmap crash & gouge verification has not been run on the current toolpaths.",
                    );
                    ui.label("Run Simulation first to verify zero G0 rapid crashes, gouges, or holder collisions.");
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        if ui
                            .button("Run Simulation Now (Recommended)")
                            .on_hover_text("Close this prompt and run material removal and crash verification first")
                            .clicked()
                        {
                            self.show_safety_gate_modal = false;
                            action = Some(GCodeUiAction::RunSimulationFirst);
                        }
                        if ui
                            .button("Export Unverified Anyway")
                            .on_hover_text("Bypass simulation verification and save the posted G-code directly to disk")
                            .clicked()
                        {
                            self.show_safety_gate_modal = false;
                            self.export_files_with_dialog(logs);
                        }
                    });
                } else {
                    ui.colored_label(
                        tok.error_fg,
                        "ChipCutter detected critical physical collisions or part gouges in this Setup:",
                    );
                    let mut first_crit: Option<SimDiagnostic> = None;
                    if let Some(res) = &sim_state.active_result {
                        for d in res
                            .diagnostics
                            .iter()
                            .filter(|d| d.severity == SimSeverity::CriticalError)
                        {
                            if first_crit.is_none() {
                                first_crit = Some(d.clone());
                            }
                            ui.monospace(format!("• [{}] {}: {}", d.kind.label(), d.op_name, d.message));
                        }
                    }
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        if let Some(diag) = first_crit
                            && ui
                                .button("Cancel & Jump to Crash in 3D")
                                .on_hover_text("Abort G-code export and focus the 3D viewport on the first detected collision")
                                .clicked()
                        {
                            self.show_safety_gate_modal = false;
                            action = Some(GCodeUiAction::JumpToCrash(diag));
                        }
                        if ui
                            .button(
                                egui::RichText::new("Acknowledge Risk & Override Export")
                                    .color(tok.error_fg),
                            )
                            .on_hover_text("Override the crash safety gate and export the G-code files anyway (use with caution)")
                            .clicked()
                        {
                            self.show_safety_gate_modal = false;
                            self.export_files_with_dialog(logs);
                        }
                    });
                }
            });
            if !open {
                self.show_safety_gate_modal = false;
            }
        }

        ui.separator();

        if let Some(setup) = project.setups.get(setup_idx)
            && let Some(active_prof) = self
                .available_profiles
                .iter()
                .find(|p| p.id.eq_ignore_ascii_case(&setup.post_profile))
            && active_prof.source == PostProfileSource::ProjectOrBundle
        {
            let tok = crate::theme::ThemeTokens::from_ui(ui);
            ui.horizontal_wrapped(|ui| {
                crate::widgets::status_badge_icon(
                    ui,
                    crate::widgets::StatusBadgeKind::NeedsRepick,
                    None,
                );
                ui.colored_label(
                    tok.warn_fg,
                    format!(
                        "Custom post profile '{}' was loaded from this project/bundle — review header, tool-change, and footer G-code before machining.",
                        active_prof.id
                    ),
                );
            });
            ui.separator();
        }

        if let Some(err) = &self.last_error {
            ui.weak(err);
            return action;
        }
        if self.cached_files.is_empty() {
            ui.weak("No G-code output generated.");
            return action;
        }

        // File Tabs (when `split_files_by_tool` emits multiple `.nc` files) + Line Stats
        ui.horizontal_wrapped(|ui| {
            for (idx, f) in self.cached_files.iter().enumerate() {
                let lbl = format!("{} ({}L)", f.filename, f.line_count);
                ui.selectable_value(&mut self.selected_file_tab, idx, lbl)
                    .on_hover_text(format!("Preview generated G-code file '{}'", f.filename));
            }
            if let Some(active) = self.cached_files.get(self.selected_file_tab) {
                ui.separator();
                ui.weak(format!(
                    "Max line: {} chars | Tools: {:?}",
                    active.max_line_chars, active.tool_numbers
                ));
            }
        });

        // Virtualized 60-FPS Monospace G-Code Viewer (`show_rows`)
        if let Some(lines) = self.cached_lines_per_file.get(self.selected_file_tab) {
            let tok = crate::theme::ThemeTokens::from_ui(ui);
            egui::Frame::NONE
                .fill(tok.code_bg)
                .stroke(egui::Stroke::new(1.0, tok.border))
                .inner_margin(4.0)
                .corner_radius(4.0)
                .show(ui, |ui| {
                    let row_h = ui.text_style_height(&egui::TextStyle::Monospace);
                    egui::ScrollArea::vertical()
                        .id_salt(("gcode_preview_scroll", self.selected_file_tab))
                        .auto_shrink([false, false])
                        .show_rows(ui, row_h, lines.len(), |ui, row_range| {
                            for line_idx in row_range {
                                let line = &lines[line_idx];
                                let col = gcode_line_color(line, &tok);
                                ui.horizontal(|ui| {
                                    ui.weak(format!("{:>5} ", line_idx + 1));
                                    ui.colored_label(col, egui::RichText::new(line).monospace());
                                });
                            }
                        });
                });
        }

        action
    }

    pub fn trigger_export_action(
        &mut self,
        project: &ProjectDocument,
        setup_idx: usize,
        cache: &ToolpathCache,
        cad_revision: u64,
        sim_state: &SimulationPanelState,
        logs: &mut Vec<String>,
    ) {
        let verified = sim_state.is_verified_up_to_date(project, setup_idx, cache, cad_revision);
        let crit_count = sim_state
            .active_result
            .as_ref()
            .map_or(0, |r| r.critical_error_count());
        if crit_count > 0 {
            self.show_safety_gate_modal = true;
            self.pending_unverified_warning = false;
        } else if !verified {
            self.show_safety_gate_modal = true;
            self.pending_unverified_warning = true;
        } else {
            self.export_files_with_dialog(logs);
        }
    }

    fn export_files_with_dialog(&self, logs: &mut Vec<String>) {
        if self.cached_files.is_empty() {
            return;
        }
        if self.cached_files.len() == 1 {
            let f = &self.cached_files[0];
            if let Some(dest) = rfd::FileDialog::new()
                .set_file_name(&f.filename)
                .save_file()
            {
                match std::fs::write(&dest, &f.content) {
                    Ok(()) => logs.push(format!(
                        "Exported G-code ({} lines) to '{}'.",
                        f.line_count,
                        dest.display()
                    )),
                    Err(e) => logs.push(format!("ERROR writing '{}': {e}", dest.display())),
                }
            }
        } else if let Some(dir) = rfd::FileDialog::new().pick_folder() {
            for f in &self.cached_files {
                let safe_name = match validate_safe_output_filename(&f.filename) {
                    Ok(name) => name,
                    Err(e) => {
                        logs.push(format!("ERROR skipping unsafe filename: {e}"));
                        continue;
                    }
                };
                let dest = dir.join(safe_name);
                match std::fs::write(&dest, &f.content) {
                    Ok(()) => logs.push(format!(
                        "Exported '{}' ({} lines).",
                        dest.display(),
                        f.line_count
                    )),
                    Err(e) => logs.push(format!("ERROR writing '{}': {e}", dest.display())),
                }
            }
        }
    }
}

fn gcode_line_color(line: &str, tok: &crate::theme::ThemeTokens) -> egui::Color32 {
    let t = line.trim_start();
    if t.starts_with('(') || t.starts_with(';') || t == "%" {
        tok.gcode_comment
    } else if t.contains("G0 ") || t.ends_with("G0") {
        tok.gcode_rapid
    } else if t.contains("G1 ") || t.ends_with("G1") {
        tok.gcode_feed
    } else if t.contains("G2 ")
        || t.contains("G3 ")
        || t.contains("G81")
        || t.contains("G82")
        || t.contains("G83")
        || t.contains("G73")
    {
        tok.gcode_arc
    } else if t.contains("M6")
        || t.contains("M3")
        || t.contains("M5")
        || t.contains("M30")
        || t.contains("M2")
    {
        tok.gcode_spindle
    } else {
        tok.gcode_coord
    }
}
