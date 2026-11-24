use std::path::{Path, PathBuf};

use chip_cam::RotaryUnwindMode;
use chip_post::{
    ArcCenterMode, CommentStyle, LineNumberConfig, PostError, PostProfile, PostProfileSource,
    PostUnits, SwapAxisTarget, ToolChangeMode, allocate_unique_custom_post_id, clone_post_profile,
    delete_user_post_profile, export_post_profile_to_path, import_post_profile_from_path,
    save_user_post_profile,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PostProfileTab {
    #[default]
    GeneralFormat,
    ArcsCyclesComp,
    TemplatesBlocks,
    Rotary,
}

/// Actions emitted by [`PostProfileInspectorModalState::show`] back to [`crate::app::ChipCutterApp`].
#[derive(Debug, Default)]
pub struct PostProfileInspectorActions {
    /// Assign this post-processor profile ID to the active Setup (with Undo).
    pub select_setup_post_id: Option<String>,
    /// True whenever a custom profile was cloned, edited, imported, or deleted.
    pub profiles_modified: bool,
}

/// Persistent UI state for the **Post Profile Inspector / Customizer** modal (`M89`).
///
/// Built-in profiles (`grbl`, `grblhal_atc`, `linuxcnc`, etc.) are strictly read-only;
/// cloning saves an upgrade-safe custom `.post.toml` into `dirs::config_dir()/chipcutter/posts/`.
#[derive(Debug, Default)]
pub struct PostProfileInspectorModalState {
    pub open: bool,
    pub selected_idx: usize,
    pub tab: PostProfileTab,
    pub clone_draft_open: bool,
    pub clone_new_id: String,
    pub clone_new_name: String,
    pub status_banner: Option<(bool, String)>,
    #[cfg(test)]
    pub(crate) custom_save_dir: Option<PathBuf>,
}

impl PostProfileInspectorModalState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn open_selecting(&mut self, profiles: &[PostProfile], post_id: &str) {
        self.open = true;
        self.clone_draft_open = false;
        if let Some(idx) = profiles
            .iter()
            .position(|p| p.id.eq_ignore_ascii_case(post_id))
        {
            self.selected_idx = idx;
        } else if self.selected_idx >= profiles.len() {
            self.selected_idx = 0;
        }
    }

    fn persist_profile(&self, profile: &PostProfile) -> Result<PathBuf, PostError> {
        #[cfg(test)]
        if let Some(dir) = &self.custom_save_dir {
            return chip_post::save_user_post_profile_in(profile, dir);
        }
        save_user_post_profile(profile)
    }

    fn delete_profile(&self, id: &str) -> Result<(), PostError> {
        #[cfg(test)]
        if let Some(dir) = &self.custom_save_dir {
            return chip_post::delete_user_post_profile_in(id, dir);
        }
        delete_user_post_profile(id)
    }

    fn import_profile(
        &self,
        src: &Path,
        existing_ids: &[String],
    ) -> Result<PostProfile, PostError> {
        #[cfg(test)]
        if let Some(dir) = &self.custom_save_dir {
            return chip_post::import_post_profile_from_path_in(src, dir, existing_ids);
        }
        let _ = existing_ids;
        import_post_profile_from_path(src)
    }

    pub fn show(
        &mut self,
        ctx: &egui::Context,
        profiles: &mut Vec<PostProfile>,
        active_setup_post_id: &str,
        logs: &mut Vec<String>,
    ) -> PostProfileInspectorActions {
        let mut actions = PostProfileInspectorActions::default();
        if !self.open || profiles.is_empty() {
            return actions;
        }
        if self.selected_idx >= profiles.len() {
            self.selected_idx = 0;
        }

        let mut keep_open = self.open;
        let tok = crate::theme::ThemeTokens::from_ctx(ctx);

        egui::Window::new("Post Profile Inspector / Customizer")
            .open(&mut keep_open)
            .default_size([920.0, 660.0])
            .resizable(true)
            .show(ctx, |ui| {
                let selected_is_builtin = profiles[self.selected_idx].is_builtin;
                let selected_source = profiles[self.selected_idx].source;
                let selected_is_user_custom =
                    !selected_is_builtin && selected_source == PostProfileSource::UserConfig;
                let selected_id = profiles[self.selected_idx].id.clone();
                let selected_name = profiles[self.selected_idx].name.clone();

                // Top action toolbar
                ui.horizontal_wrapped(|ui| {
                    if ui
                        .button("Clone to Custom .post.toml")
                        .on_hover_text(
                            "Create an editable, upgrade-safe copy in your user config folder (~/.config/chipcutter/posts/)",
                        )
                        .clicked()
                    {
                        let existing: Vec<String> =
                            profiles.iter().map(|p| p.id.clone()).collect();
                        self.clone_new_id =
                            allocate_unique_custom_post_id(&selected_id, &existing);
                        self.clone_new_name = format!("{selected_name} (Custom)");
                        self.clone_draft_open = true;
                    }

                    if ui
                        .button("Import .post.toml...")
                        .on_hover_text("Import a custom .post.toml file into user configuration")
                        .clicked()
                        && let Some(path) = rfd::FileDialog::new()
                            .add_filter("ChipCutter Post Profile", &["toml"])
                            .pick_file()
                    {
                        let existing: Vec<String> =
                            profiles.iter().map(|p| p.id.clone()).collect();
                        match self.import_profile(&path, &existing) {
                            Ok(imported) => {
                                let msg = format!(
                                    "Imported custom post profile '{}' ({}) into user configuration.",
                                    imported.name, imported.id
                                );
                                logs.push(format!("[POST] {msg}"));
                                self.status_banner = Some((false, msg));
                                profiles.push(imported);
                                self.selected_idx = profiles.len() - 1;
                                actions.profiles_modified = true;
                            }
                            Err(err) => {
                                let msg = format!("Import failed: {err}");
                                logs.push(format!("[POST] {msg}"));
                                self.status_banner = Some((true, msg));
                            }
                        }
                    }

                    if ui
                        .button("Export .post.toml...")
                        .on_hover_text("Export this post profile as a shareable .post.toml file")
                        .clicked()
                        && let Some(dest) = rfd::FileDialog::new()
                            .set_file_name(format!("{selected_id}.post.toml"))
                            .add_filter("ChipCutter Post Profile", &["toml"])
                            .save_file()
                    {
                        match export_post_profile_to_path(&profiles[self.selected_idx], &dest) {
                            Ok(()) => {
                                let msg = format!(
                                    "Exported post profile '{}' to '{}'.",
                                    selected_name,
                                    dest.display()
                                );
                                logs.push(format!("[POST] {msg}"));
                                self.status_banner = Some((false, msg));
                            }
                            Err(err) => {
                                let msg = format!("Export failed: {err}");
                                logs.push(format!("[POST] {msg}"));
                                self.status_banner = Some((true, msg));
                            }
                        }
                    }

                    ui.separator();

                    let is_active = selected_id.eq_ignore_ascii_case(active_setup_post_id);
                    if ui
                        .add_enabled(!is_active, egui::Button::new("Use in Active Setup"))
                        .on_hover_text("Assign this post profile to the active Setup")
                        .clicked()
                    {
                        actions.select_setup_post_id = Some(selected_id.clone());
                        self.status_banner = Some((
                            false,
                            format!("Active Setup post profile set to '{selected_name}'."),
                        ));
                    }

                    if ui
                        .add_enabled(selected_is_user_custom, egui::Button::new("Delete Custom"))
                        .on_hover_text(if selected_is_builtin {
                            "Built-in post profiles are strictly read-only and cannot be deleted."
                        } else if !selected_is_user_custom {
                            "Only custom profiles saved in your user config folder can be deleted here."
                        } else {
                            "Delete this custom .post.toml from your user configuration folder"
                        })
                        .clicked()
                    {
                        match self.delete_profile(&selected_id) {
                            Ok(()) => {
                                let msg = format!(
                                    "Deleted custom post profile '{}' ({}).",
                                    selected_name, selected_id
                                );
                                logs.push(format!("[POST] {msg}"));
                                self.status_banner = Some((false, msg));
                                profiles.remove(self.selected_idx);
                                if self.selected_idx >= profiles.len() {
                                    self.selected_idx = profiles.len().saturating_sub(1);
                                }
                                actions.profiles_modified = true;
                            }
                            Err(err) => {
                                self.status_banner =
                                    Some((true, format!("Delete failed: {err}")));
                            }
                        }
                    }
                });

                // Inline Clone prompt
                if self.clone_draft_open {
                    ui.add_space(4.0);
                    egui::Frame::group(ui.style()).show(ui, |ui| {
                        ui.horizontal_wrapped(|ui| {
                            ui.strong("Clone Post Profile:");
                            ui.label("ID:");
                            ui.add(
                                egui::TextEdit::singleline(&mut self.clone_new_id)
                                    .desired_width(160.0),
                            )
                            .on_hover_text(
                                "Unique filename-safe slug ID saved as <id>.post.toml in user config space",
                            );
                            ui.label("Name:");
                            ui.add(
                                egui::TextEdit::singleline(&mut self.clone_new_name)
                                    .desired_width(240.0),
                            )
                            .on_hover_text("Display name shown in Setup Inspector and G-Code Preview");
                            if ui
                                .button("Create Custom Profile")
                                .on_hover_text("Save cloned editable .post.toml into user config space")
                                .clicked()
                            {
                                match clone_post_profile(
                                    &profiles[self.selected_idx],
                                    self.clone_new_id.trim(),
                                    self.clone_new_name.trim(),
                                )
                                .and_then(|cloned| {
                                    let dest = self.persist_profile(&cloned)?;
                                    Ok((cloned, dest))
                                }) {
                                    Ok((cloned, dest)) => {
                                        let msg = format!(
                                            "Cloned '{}' -> '{}' ({}) at '{}'.",
                                            selected_name,
                                            cloned.name,
                                            cloned.id,
                                            dest.display()
                                        );
                                        logs.push(format!("[POST] {msg}"));
                                        self.status_banner = Some((false, msg));
                                        profiles.push(cloned);
                                        self.selected_idx = profiles.len() - 1;
                                        self.clone_draft_open = false;
                                        actions.profiles_modified = true;
                                    }
                                    Err(err) => {
                                        self.status_banner =
                                            Some((true, format!("Clone failed: {err}")));
                                    }
                                }
                            }
                            if ui.button("Cancel").clicked() {
                                self.clone_draft_open = false;
                            }
                        });
                    });
                }

                if let Some((is_err, msg)) = &self.status_banner {
                    ui.add_space(4.0);
                    let color = if *is_err { tok.error_fg } else { tok.info_fg };
                    ui.colored_label(color, msg);
                }

                ui.separator();

                // Split layout: left profile list, right tabbed inspector/editor
                ui.columns(2, |cols| {
                    cols[0].heading(format!("Post Profiles ({})", profiles.len()));
                    egui::ScrollArea::vertical()
                        .id_salt("post_inspector_list")
                        .max_height(520.0)
                        .show(&mut cols[0], |ui| {
                            for (idx, p) in profiles.iter().enumerate() {
                                let selected = idx == self.selected_idx;
                                let mut badges = match p.source {
                                    PostProfileSource::BuiltIn => "[Built-In]".to_string(),
                                    PostProfileSource::UserConfig => "[Custom]".to_string(),
                                    PostProfileSource::ProjectOrBundle => "[Project]".to_string(),
                                    PostProfileSource::ExplicitFile => "[File]".to_string(),
                                };
                                if p.id.eq_ignore_ascii_case(active_setup_post_id) {
                                    badges.push_str("  [Active]");
                                }
                                let rotary_tag = if p.rotary.enabled {
                                    format!("4-Axis ({})", p.rotary.axis_letter)
                                } else {
                                    "3-Axis".to_string()
                                };
                                let label = format!(
                                    "{}\n{}  |  {} ({})  |  .{}  |  {}  |  {}",
                                    p.name,
                                    badges,
                                    p.units.g_code(),
                                    p.units.unit_suffix(),
                                    p.extension,
                                    p.tool_change_mode.label(),
                                    rotary_tag
                                );
                                if ui
                                    .selectable_label(selected, label)
                                    .on_hover_text("Inspect or customize this post-processor dialect")
                                    .clicked()
                                {
                                    self.selected_idx = idx;
                                    self.clone_draft_open = false;
                                }
                            }
                        });

                    // Right column: read-only banner + tabs + controls
                    let right = &mut cols[1];
                    let is_builtin = profiles[self.selected_idx].is_builtin;
                    let src = profiles[self.selected_idx].source;
                    let is_read_only = is_builtin || src != PostProfileSource::UserConfig;
                    let p_id_copy = profiles[self.selected_idx].id.clone();

                    if is_read_only {
                        egui::Frame::group(right.style()).show(right, |ui| {
                            ui.horizontal_wrapped(|ui| {
                                let msg = if is_builtin {
                                    "Built-in post profile (read-only) -- updated cleanly on upgrades. Clone to customize."
                                } else {
                                    "Project/external post profile (read-only here) -- clone into user config to customize."
                                };
                                ui.colored_label(tok.warn_fg, msg);
                                if ui
                                    .small_button("Clone to Custom .post.toml")
                                    .on_hover_text(
                                        "Create an editable custom copy in user configuration space",
                                    )
                                    .clicked()
                                {
                                    let existing: Vec<String> =
                                        profiles.iter().map(|p| p.id.clone()).collect();
                                    let name_now = profiles[self.selected_idx].name.clone();
                                    self.clone_new_id =
                                        allocate_unique_custom_post_id(&p_id_copy, &existing);
                                    self.clone_new_name = format!("{name_now} (Custom)");
                                    self.clone_draft_open = true;
                                }
                            });
                        });
                        right.add_space(4.0);
                    }

                    right.horizontal_wrapped(|ui| {
                        ui.selectable_value(
                            &mut self.tab,
                            PostProfileTab::GeneralFormat,
                            "General & Format",
                        );
                        ui.selectable_value(
                            &mut self.tab,
                            PostProfileTab::ArcsCyclesComp,
                            "Arcs, Cycles & Comp",
                        );
                        ui.selectable_value(
                            &mut self.tab,
                            PostProfileTab::TemplatesBlocks,
                            "Headers & Footers",
                        );
                        ui.selectable_value(
                            &mut self.tab,
                            PostProfileTab::Rotary,
                            "4th-Axis Rotary",
                        );
                    });
                    right.separator();

                    let mut dirty = false;
                    egui::ScrollArea::vertical()
                        .id_salt("post_inspector_editor")
                        .max_height(480.0)
                        .show(right, |ui| {
                            let prof = &mut profiles[self.selected_idx];
                            ui.add_enabled_ui(!is_read_only, |ui| match self.tab {
                                PostProfileTab::GeneralFormat => {
                                    dirty |= show_general_format_tab(ui, prof);
                                }
                                PostProfileTab::ArcsCyclesComp => {
                                    dirty |= show_arcs_cycles_comp_tab(ui, prof);
                                }
                                PostProfileTab::TemplatesBlocks => {
                                    dirty |= show_templates_blocks_tab(ui, prof);
                                }
                                PostProfileTab::Rotary => {
                                    dirty |= show_rotary_post_tab(ui, prof);
                                }
                            });
                        });

                    if dirty && !is_read_only {
                        let prof = &profiles[self.selected_idx];
                        match self.persist_profile(prof) {
                            Ok(dest) => {
                                self.status_banner = Some((
                                    false,
                                    format!("Saved '{}' to '{}'.", prof.name, dest.display()),
                                ));
                                actions.profiles_modified = true;
                            }
                            Err(err) => {
                                self.status_banner = Some((true, format!("Save failed: {err}")));
                            }
                        }
                    }
                });
            });

        self.open = keep_open;
        actions
    }
}

fn show_general_format_tab(ui: &mut egui::Ui, p: &mut PostProfile) -> bool {
    let mut dirty = false;
    egui::Grid::new("post_general_grid")
        .num_columns(2)
        .spacing([12.0, 6.0])
        .show(ui, |ui| {
            ui.label("Profile ID:");
            ui.monospace(&p.id);
            ui.end_row();

            ui.label("Display Name:");
            dirty |= ui
                .text_edit_singleline(&mut p.name)
                .on_hover_text("Human-readable post-processor profile name")
                .changed();
            ui.end_row();

            ui.label("Description:");
            dirty |= ui
                .add(
                    egui::TextEdit::multiline(&mut p.description)
                        .desired_rows(2)
                        .desired_width(280.0),
                )
                .on_hover_text("Notes describing this controller dialect")
                .changed();
            ui.end_row();

            ui.label("File Extension:");
            dirty |= ui
                .add(egui::TextEdit::singleline(&mut p.extension).desired_width(90.0))
                .on_hover_text("Output G-code extension without leading dot (e.g., nc, ngc, tap)")
                .changed();
            ui.end_row();

            ui.label("Output Units:");
            egui::ComboBox::from_id_salt("post_units_cb")
                .selected_text(format!("{} ({})", p.units.g_code(), p.units.unit_suffix()))
                .show_ui(ui, |ui| {
                    for u in [PostUnits::Mm, PostUnits::Inch] {
                        dirty |= ui
                            .selectable_value(
                                &mut p.units,
                                u,
                                format!("{} ({})", u.g_code(), u.unit_suffix()),
                            )
                            .changed();
                    }
                });
            ui.end_row();

            ui.label("Decimals (Coord / Feed / RPM):");
            ui.horizontal(|ui| {
                dirty |= ui
                    .add(
                        egui::DragValue::new(&mut p.coord_decimals)
                            .range(1..=6)
                            .prefix("XYZ "),
                    )
                    .on_hover_text("Coordinate decimal places (typically 3 for mm, 4 for inch)")
                    .changed();
                dirty |= ui
                    .add(
                        egui::DragValue::new(&mut p.feed_decimals)
                            .range(0..=4)
                            .prefix("F "),
                    )
                    .on_hover_text("Feedrate decimal places")
                    .changed();
                dirty |= ui
                    .add(
                        egui::DragValue::new(&mut p.spindle_decimals)
                            .range(0..=2)
                            .prefix("S "),
                    )
                    .on_hover_text("Spindle RPM decimal places")
                    .changed();
            });
            ui.end_row();

            ui.label("Trailing Zeros:");
            dirty |= ui
                .checkbox(&mut p.suppress_trailing_zeros, "Suppress trailing zeros")
                .on_hover_text("Format 10.500 as 10.5 to keep lines compact")
                .changed();
            ui.end_row();

            ui.label("Line Numbers (N#):");
            ui.horizontal(|ui| {
                let mut enabled = p.line_numbers.is_some();
                if ui.checkbox(&mut enabled, "Emit N-blocks").changed() {
                    p.line_numbers = enabled.then_some(LineNumberConfig {
                        start: 10,
                        increment: 10,
                    });
                    dirty = true;
                }
                if let Some(ln) = &mut p.line_numbers {
                    dirty |= ui
                        .add(
                            egui::DragValue::new(&mut ln.start)
                                .range(1..=10_000)
                                .prefix("Start "),
                        )
                        .changed();
                    dirty |= ui
                        .add(
                            egui::DragValue::new(&mut ln.increment)
                                .range(1..=1_000)
                                .prefix("Step "),
                        )
                        .changed();
                }
            });
            ui.end_row();

            ui.label("Max Line Length:");
            ui.horizontal(|ui| {
                let mut has_limit = p.max_line_length.is_some();
                if ui.checkbox(&mut has_limit, "Enforce").changed() {
                    p.max_line_length = has_limit.then_some(80);
                    dirty = true;
                }
                if let Some(lim) = &mut p.max_line_length {
                    dirty |= ui
                        .add(egui::DragValue::new(lim).range(32..=512).suffix(" chars"))
                        .on_hover_text(
                            "Maximum characters per emitted G-code line (GRBL 1.1 requires <= 80)",
                        )
                        .changed();
                }
            });
            ui.end_row();

            ui.label("Comment Style:");
            egui::ComboBox::from_id_salt("post_comment_style_cb")
                .selected_text(p.comment_style.label())
                .show_ui(ui, |ui| {
                    for st in CommentStyle::ALL {
                        dirty |= ui
                            .selectable_value(&mut p.comment_style, st, st.label())
                            .changed();
                    }
                });
            ui.end_row();

            ui.label("Path Blend (G64 P):");
            ui.horizontal(|ui| {
                let mut has_blend = p.path_blend_tol_mm.is_some();
                if ui.checkbox(&mut has_blend, "Emit G64 P").changed() {
                    p.path_blend_tol_mm = has_blend.then_some(0.05);
                    dirty = true;
                }
                if let Some(tol) = &mut p.path_blend_tol_mm {
                    dirty |= ui
                        .add(
                            egui::DragValue::new(tol)
                                .speed(0.005)
                                .range(0.001..=5.0)
                                .fixed_decimals(3)
                                .suffix(" mm"),
                        )
                        .changed();
                }
            });
            ui.end_row();

            ui.label("Kinematics (Sim Fallback):");
            ui.horizontal(|ui| {
                dirty |= ui
                    .add(
                        egui::DragValue::new(&mut p.rapid_rate_mm_min)
                            .speed(50.0)
                            .range(100.0..=50_000.0)
                            .prefix("G0 ")
                            .suffix(" mm/m"),
                    )
                    .changed();
                dirty |= ui
                    .add(
                        egui::DragValue::new(&mut p.accel_mm_s2)
                            .speed(10.0)
                            .range(10.0..=20_000.0)
                            .prefix("Acc ")
                            .suffix(" mm/s2"),
                    )
                    .changed();
                dirty |= ui
                    .add(
                        egui::DragValue::new(&mut p.tool_change_time_sec)
                            .speed(0.5)
                            .range(0.0..=300.0)
                            .prefix("ATC ")
                            .suffix(" s"),
                    )
                    .changed();
            });
            ui.end_row();
        });
    dirty
}

fn show_arcs_cycles_comp_tab(ui: &mut egui::Ui, p: &mut PostProfile) -> bool {
    let mut dirty = false;
    egui::Grid::new("post_arcs_cycles_grid")
        .num_columns(2)
        .spacing([12.0, 6.0])
        .show(ui, |ui| {
            ui.label("Arc Center Mode:");
            egui::ComboBox::from_id_salt("post_arc_mode_cb")
                .selected_text(p.arc_center_mode.label())
                .show_ui(ui, |ui| {
                    for mode in ArcCenterMode::ALL {
                        dirty |= ui
                            .selectable_value(&mut p.arc_center_mode, mode, mode.label())
                            .changed();
                    }
                });
            ui.end_row();

            ui.label("Max Arc Sweep:");
            dirty |= ui
                .add(
                    egui::DragValue::new(&mut p.max_arc_sweep_deg)
                        .speed(1.0)
                        .range(15.0..=360.0)
                        .suffix(" deg"),
                )
                .on_hover_text("Split arcs larger than this angular sweep (90 deg for quadrant splitting, 180/360 for full circles)")
                .changed();
            ui.end_row();

            ui.label("Helical Arcs:");
            ui.horizontal(|ui| {
                dirty |= ui
                    .checkbox(&mut p.allow_helical_arcs, "Allow helical G2/G3 with Z")
                    .changed();
                if !p.allow_helical_arcs {
                    dirty |= ui
                        .add(
                            egui::DragValue::new(&mut p.helical_linearize_tol_mm)
                                .speed(0.002)
                                .range(0.001..=0.5)
                                .fixed_decimals(3)
                                .prefix("Tol ")
                                .suffix(" mm"),
                        )
                        .changed();
                }
            });
            ui.end_row();

            ui.label("Drill Cycles (G81/G83/G73):");
            dirty |= ui
                .checkbox(
                    &mut p.expand_drill_cycles,
                    "Expand canned drill cycles into G0/G1 moves",
                )
                .on_hover_text("Enable for controllers without native G81/G82/G83/G73 support (such as GRBL 1.1)")
                .changed();
            ui.end_row();

            ui.label("Tool Change Mode:");
            egui::ComboBox::from_id_salt("post_tc_mode_cb")
                .selected_text(p.tool_change_mode.label())
                .show_ui(ui, |ui| {
                    for mode in ToolChangeMode::ALL {
                        dirty |= ui
                            .selectable_value(&mut p.tool_change_mode, mode, mode.label())
                            .changed();
                    }
                });
            ui.end_row();

            ui.label("WCS & Offsets:");
            ui.vertical(|ui| {
                dirty |= ui
                    .checkbox(&mut p.emit_wcs_code, "Emit WCS code (G54..G59)")
                    .changed();
                dirty |= ui
                    .checkbox(&mut p.allow_extended_wcs, "Allow extended WCS (G59.1..G59.3)")
                    .changed();
                dirty |= ui
                    .checkbox(&mut p.use_g43_length_offset, "Emit G43 H# tool length offset")
                    .changed();
                dirty |= ui
                    .checkbox(
                        &mut p.use_cutter_comp_g41_g42,
                        "Emit G41/G42 D# wear cutter compensation",
                    )
                    .changed();
                dirty |= ui
                    .checkbox(&mut p.safe_home_z_on_end, "Retract to safe home Z on program end")
                    .changed();
            });
            ui.end_row();
        });
    dirty
}

fn show_templates_blocks_tab(ui: &mut egui::Ui, p: &mut PostProfile) -> bool {
    let mut dirty = false;
    for (id_salt, label, lines) in [
        (
            "post_tpl_prog_hdr",
            "Program Header (one block per line):",
            &mut p.program_header,
        ),
        (
            "post_tpl_op_hdr",
            "Operation Header:",
            &mut p.operation_header,
        ),
        (
            "post_tpl_tc_blk",
            "Tool Change Block:",
            &mut p.tool_change_block,
        ),
        (
            "post_tpl_spindle",
            "Spindle Start Block:",
            &mut p.spindle_start_block,
        ),
        (
            "post_tpl_op_ftr",
            "Operation Footer:",
            &mut p.operation_footer,
        ),
        (
            "post_tpl_prog_ftr",
            "Program Footer:",
            &mut p.program_footer,
        ),
    ] {
        ui.label(label);
        let mut buf = lines.join("\n");
        if ui
            .add(
                egui::TextEdit::multiline(&mut buf)
                    .id_salt(id_salt)
                    .font(egui::TextStyle::Monospace)
                    .desired_rows(3)
                    .desired_width(f32::INFINITY),
            )
            .changed()
        {
            *lines = buf
                .lines()
                .map(|s| s.trim_end().to_string())
                .filter(|s| !s.is_empty())
                .collect();
            dirty = true;
        }
        ui.add_space(4.0);
    }
    dirty
}

fn show_rotary_post_tab(ui: &mut egui::Ui, p: &mut PostProfile) -> bool {
    let mut dirty = false;
    let r = &mut p.rotary;
    egui::Grid::new("post_rotary_grid")
        .num_columns(2)
        .spacing([12.0, 6.0])
        .show(ui, |ui| {
            ui.label("Rotary Output:");
            dirty |= ui
                .checkbox(&mut r.enabled, "Enable 4th-axis rotary output ([rotary])")
                .changed();
            ui.end_row();

            ui.label("Axis Letter:");
            egui::ComboBox::from_id_salt("post_rotary_axis_cb")
                .selected_text(format!("{}", r.axis_letter))
                .show_ui(ui, |ui| {
                    for ch in ['A', 'B', 'C'] {
                        dirty |= ui
                            .selectable_value(&mut r.axis_letter, ch, format!("{ch}-Axis"))
                            .changed();
                    }
                });
            ui.end_row();

            ui.label("Angle Decimals:");
            dirty |= ui
                .add(egui::DragValue::new(&mut r.angle_decimals).range(1..=5))
                .changed();
            ui.end_row();

            ui.label("Inverse Time Feed (G93):");
            ui.vertical(|ui| {
                dirty |= ui
                    .checkbox(
                        &mut r.supports_g93_inverse_time,
                        "Controller supports G93 inverse-time feed",
                    )
                    .changed();
                dirty |= ui
                    .checkbox(
                        &mut r.use_g93_for_simultaneous,
                        "Use G93 for simultaneous 4-axis moves",
                    )
                    .changed();
            });
            ui.end_row();

            ui.label("Max Rotary Feed:");
            dirty |= ui
                .add(
                    egui::DragValue::new(&mut r.max_rotary_feed_deg_min)
                        .speed(60.0)
                        .range(60.0..=100_000.0)
                        .suffix(" deg/min"),
                )
                .changed();
            ui.end_row();

            ui.label("Unwind Mode:");
            egui::ComboBox::from_id_salt("post_rotary_unwind_cb")
                .selected_text(r.unwind_mode.label())
                .show_ui(ui, |ui| {
                    for mode in RotaryUnwindMode::ALL {
                        dirty |= ui
                            .selectable_value(&mut r.unwind_mode, mode, mode.label())
                            .changed();
                    }
                });
            ui.end_row();

            ui.label("Swapped Linear Axis:");
            let swap_lbl = match r.swap_linear_axis {
                None => "None (Native 4th Axis)",
                Some(SwapAxisTarget::ReplaceY) => "Replace Y Axis (Rotary Wrap)",
                Some(SwapAxisTarget::ReplaceX) => "Replace X Axis (Rotary Wrap)",
            };
            egui::ComboBox::from_id_salt("post_rotary_swap_cb")
                .selected_text(swap_lbl)
                .show_ui(ui, |ui| {
                    dirty |= ui
                        .selectable_value(&mut r.swap_linear_axis, None, "None (Native 4th Axis)")
                        .changed();
                    dirty |= ui
                        .selectable_value(
                            &mut r.swap_linear_axis,
                            Some(SwapAxisTarget::ReplaceY),
                            "Replace Y Axis (Rotary Wrap)",
                        )
                        .changed();
                    dirty |= ui
                        .selectable_value(
                            &mut r.swap_linear_axis,
                            Some(SwapAxisTarget::ReplaceX),
                            "Replace X Axis (Rotary Wrap)",
                        )
                        .changed();
                });
            ui.end_row();
        });
    dirty
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_post::{
        PostError, PostProfileSource, builtin_post_profiles, clone_post_profile,
        load_available_post_profiles_in, save_user_post_profile_in,
    };
    use chip_project::create_private_temp_extract_dir;

    use super::{PostProfileInspectorModalState, PostProfileTab};

    fn find_text(shape: &egui::Shape, needle: &str) -> Option<egui::Pos2> {
        match shape {
            egui::Shape::Text(t) if t.galley.text() == needle => {
                Some(t.pos + t.galley.size() * 0.5)
            }
            egui::Shape::Vec(v) => v.iter().find_map(|s| find_text(s, needle)),
            _ => None,
        }
    }

    fn click(pt: egui::Pos2) -> Vec<egui::Event> {
        vec![
            egui::Event::PointerMoved(pt),
            egui::Event::PointerButton {
                pos: pt,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: egui::Modifiers::NONE,
            },
            egui::Event::PointerButton {
                pos: pt,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::NONE,
            },
        ]
    }

    #[test]
    fn builtin_post_profiles_are_read_only_and_clone_to_custom_roundtrips_cleanly() {
        let tmp = create_private_temp_extract_dir("chipcutter_m89_post_inspector").unwrap();
        let mut profiles = builtin_post_profiles();
        let grbl = profiles.iter().find(|p| p.id == "grbl").unwrap().clone();
        assert!(grbl.is_builtin);
        assert!(matches!(
            save_user_post_profile_in(&grbl, &tmp),
            Err(PostError::BuiltInReadOnly { .. })
        ));
        assert!(matches!(
            clone_post_profile(&grbl, "linuxcnc", "Shadow"),
            Err(PostError::BuiltInReadOnly { .. })
        ));

        let mut modal = PostProfileInspectorModalState::new();
        modal.custom_save_dir = Some(tmp.clone());
        modal.open_selecting(&profiles, "grbl");
        assert_eq!(profiles[modal.selected_idx].id, "grbl");

        let ctx = egui::Context::default();
        let mut logs = Vec::new();
        let run_frame = |events: Vec<egui::Event>,
                         modal: &mut PostProfileInspectorModalState,
                         profiles: &mut Vec<chip_post::PostProfile>,
                         logs: &mut Vec<String>| {
            let mut actions = Default::default();
            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(1024.0, 760.0),
                    )),
                    events,
                    ..Default::default()
                },
                |ui| {
                    actions = modal.show(ui.ctx(), profiles, "grbl", logs);
                },
            );
            out.textures_delta.clear();
            (out, actions)
        };

        // Frame 1 (warmup) + Frame 2: click "Clone to Custom .post.toml"
        let _ = run_frame(vec![], &mut modal, &mut profiles, &mut logs);
        let (out1, _) = run_frame(vec![], &mut modal, &mut profiles, &mut logs);
        let clone_btn = out1
            .shapes
            .iter()
            .find_map(|cs| find_text(&cs.shape, "Clone to Custom .post.toml"))
            .expect("Clone to Custom .post.toml button must render");
        let _ = run_frame(click(clone_btn), &mut modal, &mut profiles, &mut logs);
        let (out2, _) = run_frame(vec![], &mut modal, &mut profiles, &mut logs);
        assert!(modal.clone_draft_open);

        // Frame 4: click "Create Custom Profile"
        let create_btn = out2
            .shapes
            .iter()
            .find_map(|cs| find_text(&cs.shape, "Create Custom Profile"))
            .expect("Create Custom Profile button must render in clone bar");
        let (_, act_clone) = run_frame(click(create_btn), &mut modal, &mut profiles, &mut logs);
        assert!(act_clone.profiles_modified);
        assert!(!modal.clone_draft_open);

        let cloned_prof = &profiles[modal.selected_idx];
        assert!(!cloned_prof.is_builtin);
        assert_eq!(cloned_prof.source, PostProfileSource::UserConfig);
        let cloned_id = cloned_prof.id.clone();
        let saved_path = tmp.join(format!("{cloned_id}.post.toml"));
        assert!(
            saved_path.is_file(),
            "cloned profile must be saved to {}",
            saved_path.display()
        );

        // Switch to Arcs/Cycles tab and toggle expand_drill_cycles on the editable custom profile
        modal.tab = PostProfileTab::ArcsCyclesComp;
        let orig_expand = profiles[modal.selected_idx].expand_drill_cycles;
        let (out3, _) = run_frame(vec![], &mut modal, &mut profiles, &mut logs);
        let expand_pt = out3
            .shapes
            .iter()
            .find_map(|cs| find_text(&cs.shape, "Expand canned drill cycles into G0/G1 moves"))
            .expect("expand_drill_cycles checkbox must render");
        let (_, act_edit) = run_frame(click(expand_pt), &mut modal, &mut profiles, &mut logs);
        assert!(act_edit.profiles_modified);
        assert_eq!(
            profiles[modal.selected_idx].expand_drill_cycles,
            !orig_expand
        );

        // Reload from tmp user dir and verify persisted roundtrip
        let reloaded = load_available_post_profiles_in(Some(&tmp), None);
        let found = reloaded
            .iter()
            .find(|p| p.id == cloned_id)
            .expect("cloned custom post profile must roundtrip from user config dir");
        assert!(!found.is_builtin);
        assert_eq!(found.source, PostProfileSource::UserConfig);
        assert_eq!(found.expand_drill_cycles, !orig_expand);

        let _ = std::fs::remove_dir_all(&tmp);
    }
}
