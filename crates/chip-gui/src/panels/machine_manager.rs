use chip_machines::{
    MachineDefinition, MachineRotaryDefaults, MachineToolChangeKind,
    allocate_unique_custom_machine_id, clone_machine_definition, delete_user_machine,
    export_machine_to_path, import_machine_from_path, save_user_machine,
};
use chip_post::load_available_post_profiles;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MachineEditorTab {
    #[default]
    SpecsKinematics,
    Rotary,
    Tools,
}

/// Actions emitted by [`MachineManagerModalState::show`] back to [`crate::app::ChipCutterApp`].
#[derive(Debug, Default)]
pub struct MachineManagerActions {
    /// Switch the active project to `(machine_id, apply_setup_defaults)`.
    pub select_project_machine: Option<(String, bool)>,
    /// Persist a new default machine ID into `prefs.toml`.
    pub set_default_machine_id: Option<String>,
    /// Open the Tool Library modal scoped to the currently selected machine.
    pub open_tool_library_for_machine: Option<String>,
    /// True whenever the `machines` list or a machine's tools were mutated.
    pub machines_modified: bool,
}

/// Persistent UI state for the floating **Machine Manager** modal (`M31`).
#[derive(Debug)]
pub struct MachineManagerModalState {
    pub open: bool,
    pub selected_idx: usize,
    pub tab: MachineEditorTab,
    pub clone_draft_open: bool,
    pub clone_new_id: String,
    pub clone_new_name: String,
    pub copy_source_machine_idx: usize,
    pub new_shank_draft_mm: f64,
    pub status_banner: Option<(bool, String)>,
    #[cfg(test)]
    pub(crate) custom_save_dir: Option<std::path::PathBuf>,
}

impl Default for MachineManagerModalState {
    fn default() -> Self {
        Self {
            open: false,
            selected_idx: 0,
            tab: MachineEditorTab::SpecsKinematics,
            clone_draft_open: false,
            clone_new_id: String::new(),
            clone_new_name: String::new(),
            copy_source_machine_idx: 0,
            new_shank_draft_mm: 6.0,
            status_banner: None,
            #[cfg(test)]
            custom_save_dir: None,
        }
    }
}

impl MachineManagerModalState {
    pub fn new() -> Self {
        Self::default()
    }

    fn persist_machine(
        &self,
        machine: &MachineDefinition,
    ) -> Result<std::path::PathBuf, chip_machines::MachineStorageError> {
        #[cfg(test)]
        if let Some(dir) = &self.custom_save_dir {
            return chip_machines::save_user_machine_in(machine, dir);
        }
        save_user_machine(machine)
    }

    pub fn open_selecting(&mut self, machines: &[MachineDefinition], machine_id: &str) {
        self.open = true;
        if let Some(idx) = machines
            .iter()
            .position(|m| m.id.eq_ignore_ascii_case(machine_id))
        {
            self.selected_idx = idx;
        }
    }

    pub fn show(
        &mut self,
        ctx: &egui::Context,
        machines: &mut Vec<MachineDefinition>,
        active_project_machine_id: &str,
        default_pref_machine_id: &str,
        logs: &mut Vec<String>,
    ) -> MachineManagerActions {
        let mut actions = MachineManagerActions::default();
        if !self.open {
            return actions;
        }
        if machines.is_empty() {
            return actions;
        }
        if self.selected_idx >= machines.len() {
            self.selected_idx = 0;
        }

        let mut keep_open = self.open;
        let available_posts = load_available_post_profiles(None);

        egui::Window::new("ChipCutter Machine Manager")
            .open(&mut keep_open)
            .default_size([940.0, 700.0])
            .resizable(true)
            .show(ctx, |ui| {
                // Top action toolbar
                let selected_is_builtin = machines[self.selected_idx].is_builtin;
                let selected_id = machines[self.selected_idx].id.clone();
                let selected_name = machines[self.selected_idx].name.clone();

                ui.horizontal_wrapped(|ui| {
                    if ui
                        .button("Clone to Customize...")
                        .on_hover_text(
                            "Create an editable, upgrade-safe copy in your user configuration folder.",
                        )
                        .clicked()
                    {
                        let existing: Vec<String> =
                            machines.iter().map(|m| m.id.clone()).collect();
                        self.clone_new_id =
                            allocate_unique_custom_machine_id(&selected_id, &existing);
                        self.clone_new_name = format!("{selected_name} (Custom)");
                        self.clone_draft_open = true;
                    }

                    if ui
                        .button("Import (.machine.toml)...")
                        .on_hover_text(
                            "Import a shared machine definition (and its tool set) into user config space.",
                        )
                        .clicked()
                        && let Some(path) = rfd::FileDialog::new()
                            .add_filter("ChipCutter Machine", &["toml"])
                            .pick_file()
                    {
                        match import_machine_from_path(&path, None) {
                            Ok(imported) => {
                                let msg = format!(
                                    "Imported '{}' ({}, {} tools) into user configuration.",
                                    imported.name,
                                    imported.id,
                                    imported.tools.len()
                                );
                                logs.push(format!("[MACHINES] {msg}"));
                                self.status_banner = Some((false, msg));
                                machines.push(imported);
                                self.selected_idx = machines.len() - 1;
                                actions.machines_modified = true;
                            }
                            Err(err) => {
                                let msg = format!("Import failed: {err}");
                                logs.push(format!("[MACHINES] {msg}"));
                                self.status_banner = Some((true, msg));
                            }
                        }
                    }

                    if ui
                        .button("Export / Share (.machine.toml)...")
                        .on_hover_text(
                            "Export this machine definition and its full tool set as a single shareable file.",
                        )
                        .clicked()
                        && let Some(dest) = rfd::FileDialog::new()
                            .set_file_name(format!("{selected_id}.machine.toml"))
                            .add_filter("ChipCutter Machine", &["toml"])
                            .save_file()
                    {
                        match export_machine_to_path(&machines[self.selected_idx], &dest) {
                            Ok(()) => {
                                let msg = format!(
                                    "Exported '{}' ({} tools) to {}",
                                    selected_name,
                                    machines[self.selected_idx].tools.len(),
                                    dest.display()
                                );
                                logs.push(format!("[MACHINES] {msg}"));
                                self.status_banner = Some((false, msg));
                            }
                            Err(err) => {
                                let msg = format!("Export failed: {err}");
                                logs.push(format!("[MACHINES] {msg}"));
                                self.status_banner = Some((true, msg));
                            }
                        }
                    }

                    ui.separator();

                    let is_active_proj = selected_id.eq_ignore_ascii_case(active_project_machine_id);
                    if ui
                        .add_enabled(!is_active_proj, egui::Button::new("Use in Current Project"))
                        .clicked()
                    {
                        actions.select_project_machine = Some((selected_id.clone(), false));
                        self.status_banner = Some((
                            false,
                            format!("Active project machine switched to '{selected_name}'."),
                        ));
                    }

                    let is_default_pref = selected_id.eq_ignore_ascii_case(default_pref_machine_id);
                    if ui
                        .add_enabled(!is_default_pref, egui::Button::new("Set as Default"))
                        .on_hover_text("Use this machine by default for newly created projects.")
                        .clicked()
                    {
                        actions.set_default_machine_id = Some(selected_id.clone());
                        self.status_banner = Some((
                            false,
                            format!("Default machine for new projects set to '{selected_name}'."),
                        ));
                    }

                    if ui
                        .add_enabled(!selected_is_builtin, egui::Button::new("Delete Custom"))
                        .on_hover_text(if selected_is_builtin {
                            "Built-in presets are read-only and cannot be deleted."
                        } else {
                            "Remove this custom machine definition from your user configuration."
                        })
                        .clicked()
                    {
                        match delete_user_machine(&selected_id) {
                            Ok(()) => {
                                let msg = format!("Deleted custom machine '{selected_name}' ({selected_id}).");
                                logs.push(format!("[MACHINES] {msg}"));
                                self.status_banner = Some((false, msg));
                                machines.remove(self.selected_idx);
                                if self.selected_idx >= machines.len() {
                                    self.selected_idx = machines.len().saturating_sub(1);
                                }
                                actions.machines_modified = true;
                            }
                            Err(err) => {
                                self.status_banner = Some((true, format!("Delete failed: {err}")));
                            }
                        }
                    }
                });

                // Optional inline Clone prompt
                if self.clone_draft_open {
                    ui.add_space(4.0);
                    egui::Frame::group(ui.style()).show(ui, |ui| {
                        ui.horizontal_wrapped(|ui| {
                            ui.strong("Clone Machine:");
                            ui.label("ID:");
                            ui.add(
                                egui::TextEdit::singleline(&mut self.clone_new_id)
                                    .desired_width(160.0),
                            )
                            .on_hover_text("Unique filesystem-safe identifier for the cloned custom machine (.machine.toml)");
                            ui.label("Name:");
                            ui.add(
                                egui::TextEdit::singleline(&mut self.clone_new_name)
                                    .desired_width(240.0),
                            )
                            .on_hover_text("Display name shown in Machine Manager and Setup Inspector");
                            if ui
                                .button("Create Clone")
                                .on_hover_text("Save the cloned editable custom machine profile to your user directory")
                                .clicked()
                            {
                                match clone_machine_definition(
                                    &machines[self.selected_idx],
                                    self.clone_new_id.trim(),
                                    self.clone_new_name.trim(),
                                )
                                .and_then(|cloned| {
                                    save_user_machine(&cloned)?;
                                    Ok(cloned)
                                }) {
                                    Ok(cloned) => {
                                        let msg = format!(
                                            "Created editable custom machine '{}' ({}) with {} tools.",
                                            cloned.name,
                                            cloned.id,
                                            cloned.tools.len()
                                        );
                                        logs.push(format!("[MACHINES] {msg}"));
                                        self.status_banner = Some((false, msg));
                                        machines.push(cloned);
                                        self.selected_idx = machines.len() - 1;
                                        self.clone_draft_open = false;
                                        actions.machines_modified = true;
                                    }
                                    Err(err) => {
                                        self.status_banner =
                                            Some((true, format!("Clone failed: {err}")));
                                    }
                                }
                            }
                            if ui
                                .button("Cancel")
                                .on_hover_text("Dismiss the clone machine prompt without creating a new profile")
                                .clicked()
                            {
                                self.clone_draft_open = false;
                            }
                        });
                    });
                }

                if let Some((is_err, msg)) = &self.status_banner {
                    ui.add_space(4.0);
                    let color = if *is_err {
                        egui::Color32::from_rgb(239, 68, 68)
                    } else {
                        egui::Color32::from_rgb(56, 189, 248)
                    };
                    ui.colored_label(color, msg);
                }

                ui.separator();

                // Split pane: left machine list, right specs/rotary/tools
                ui.columns(2, |cols| {
                    // Left column: available machines
                    cols[0].heading(format!("Machines ({})", machines.len()));
                    egui::ScrollArea::vertical()
                        .id_salt("machine_manager_list")
                        .max_height(560.0)
                        .show(&mut cols[0], |ui| {
                            for (idx, m) in machines.iter().enumerate() {
                                let selected = idx == self.selected_idx;
                                let mut badges = String::new();
                                if m.is_builtin {
                                    badges.push_str("[Built-In]");
                                } else {
                                    badges.push_str("[Custom]");
                                }
                                if m.id.eq_ignore_ascii_case(active_project_machine_id) {
                                    badges.push_str("  [Active]");
                                }
                                if m.id.eq_ignore_ascii_case(default_pref_machine_id) {
                                    badges.push_str("  [Default]");
                                }
                                let rotary_tag = m
                                    .rotary
                                    .as_ref()
                                    .map_or("3-Axis", |_| "4th-Axis");
                                let label = format!(
                                    "{}\n{}  |  {:.0}×{:.0}×{:.0} mm  |  {:.0}–{:.0} RPM  |  {}  |  {} tools",
                                    m.name,
                                    badges,
                                    m.travel_mm.x_mm,
                                    m.travel_mm.y_mm,
                                    m.travel_mm.z_mm,
                                    m.spindle.min_rpm,
                                    m.spindle.max_rpm,
                                    rotary_tag,
                                    m.tools.len()
                                );
                                if ui
                                    .selectable_label(selected, label)
                                    .on_hover_text("Inspect this CNC machine profile's travels, spindle, rotary specs, and tool library")
                                    .clicked()
                                {
                                    self.selected_idx = idx;
                                    self.clone_draft_open = false;
                                }
                            }
                        });

                    // Right column: details / editor
                    let right = &mut cols[1];
                    let is_builtin = machines[self.selected_idx].is_builtin;
                    let m_id_copy = machines[self.selected_idx].id.clone();

                    if is_builtin {
                        egui::Frame::group(right.style()).show(right, |ui| {
                            ui.horizontal_wrapped(|ui| {
                                ui.colored_label(
                                    egui::Color32::from_rgb(250, 204, 21),
                                    "Built-in preset (read-only)",
                                );
                                ui.label(
                                    "— updated cleanly on app upgrades. Clone to customize specs or tools.",
                                );
                                if ui
                                    .button("Clone to Customize")
                                    .on_hover_text("Create an editable custom copy of this built-in machine profile")
                                    .clicked()
                                {
                                    let existing: Vec<String> =
                                        machines.iter().map(|m| m.id.clone()).collect();
                                    let name_now = machines[self.selected_idx].name.clone();
                                    self.clone_new_id =
                                        allocate_unique_custom_machine_id(&m_id_copy, &existing);
                                    self.clone_new_name = format!("{name_now} (Custom)");
                                    self.clone_draft_open = true;
                                }
                            });
                        });
                        right.add_space(4.0);
                    }

                    right.horizontal(|ui| {
                        ui.selectable_value(
                            &mut self.tab,
                            MachineEditorTab::SpecsKinematics,
                            "Specs & Kinematics",
                        )
                        .on_hover_text("View or edit machine axis travels, spindle RPM limits, feed/rapid limits, and default post profile");
                        ui.selectable_value(&mut self.tab, MachineEditorTab::Rotary, "4th Axis")
                            .on_hover_text("View or edit optional 4th-axis rotary chuck dimensions, slew rate, and 4-axis post profile");
                        ui.selectable_value(
                            &mut self.tab,
                            MachineEditorTab::Tools,
                            format!("Tools ({})", machines[self.selected_idx].tools.len()),
                        )
                        .on_hover_text("Inspect or copy cutting tools belonging to this CNC machine profile");
                    });
                    right.separator();

                    let mut dirty = false;
                    let mut copy_from_idx_and_tool: Option<(usize, usize)> = None;

                    egui::ScrollArea::vertical()
                        .id_salt("machine_manager_editor")
                        .max_height(520.0)
                        .show(right, |ui| {
                            match self.tab {
                                MachineEditorTab::SpecsKinematics => {
                                    let m = &mut machines[self.selected_idx];
                                    ui.add_enabled_ui(!is_builtin, |ui| {
                                        egui::Grid::new("machine_specs_grid")
                                            .num_columns(2)
                                            .spacing([12.0, 6.0])
                                            .show(ui, |ui| {
                                                ui.label("Machine ID:");
                                                ui.monospace(&m.id);
                                                ui.end_row();

                                                ui.label("Display Name:");
                                                if ui
                                                    .text_edit_singleline(&mut m.name)
                                                    .on_hover_text("Human-readable CNC machine name")
                                                    .changed()
                                                {
                                                    dirty = true;
                                                }
                                                ui.end_row();

                                                ui.label("Manufacturer:");
                                                let mut mfr =
                                                    m.manufacturer.clone().unwrap_or_default();
                                                if ui
                                                    .text_edit_singleline(&mut mfr)
                                                    .on_hover_text("Optional CNC machine builder or vendor name")
                                                    .changed()
                                                {
                                                    m.manufacturer = if mfr.trim().is_empty() {
                                                        None
                                                    } else {
                                                        Some(mfr)
                                                    };
                                                    dirty = true;
                                                }
                                                ui.end_row();

                                                ui.label("Description:");
                                                let mut desc =
                                                    m.description.clone().unwrap_or_default();
                                                if ui
                                                    .add(
                                                        egui::TextEdit::multiline(&mut desc)
                                                            .desired_rows(2)
                                                            .desired_width(280.0)
                                                            .hint_text("Optional notes about this CNC machine"),
                                                    )
                                                    .on_hover_text(
                                                        "Optional multi-line notes or setup remarks saved inside .machine.toml",
                                                    )
                                                    .changed()
                                                {
                                                    m.description = if desc.trim().is_empty() {
                                                        None
                                                    } else {
                                                        Some(desc)
                                                    };
                                                    dirty = true;
                                                }
                                                ui.end_row();

                                                ui.label("Travel X / Y / Z (mm):");
                                                ui.horizontal(|ui| {
                                                    dirty |= ui
                                                        .add(
                                                            egui::DragValue::new(&mut m.travel_mm.x_mm)
                                                                .speed(1.0)
                                                                .range(10.0..=5000.0)
                                                                .prefix("X "),
                                                        )
                                                        .on_hover_text("Maximum X-axis working envelope travel (mm)")
                                                        .changed();
                                                    dirty |= ui
                                                        .add(
                                                            egui::DragValue::new(&mut m.travel_mm.y_mm)
                                                                .speed(1.0)
                                                                .range(10.0..=5000.0)
                                                                .prefix("Y "),
                                                        )
                                                        .on_hover_text("Maximum Y-axis working envelope travel (mm)")
                                                        .changed();
                                                    dirty |= ui
                                                        .add(
                                                            egui::DragValue::new(&mut m.travel_mm.z_mm)
                                                                .speed(1.0)
                                                                .range(10.0..=2000.0)
                                                                .prefix("Z "),
                                                        )
                                                        .on_hover_text("Maximum Z-axis vertical travel (mm)")
                                                        .changed();
                                                });
                                                ui.end_row();

                                                ui.label("Spindle RPM (Min / Max):");
                                                ui.horizontal(|ui| {
                                                    dirty |= ui
                                                        .add(
                                                            egui::DragValue::new(&mut m.spindle.min_rpm)
                                                                .speed(100.0)
                                                                .range(0.0..=60_000.0),
                                                        )
                                                        .on_hover_text("Minimum supported spindle speed (RPM)")
                                                        .changed();
                                                    ui.label("to");
                                                    dirty |= ui
                                                        .add(
                                                            egui::DragValue::new(&mut m.spindle.max_rpm)
                                                                .speed(100.0)
                                                                .range(100.0..=100_000.0),
                                                        )
                                                        .on_hover_text("Maximum supported spindle speed (RPM)")
                                                        .changed();
                                                });
                                                ui.end_row();

                                                ui.label("Spindle Power (W):");
                                                ui.horizontal(|ui| {
                                                    let mut has_power =
                                                        m.spindle.power_watts.is_some();
                                                    if ui
                                                        .checkbox(&mut has_power, "Rated")
                                                        .on_hover_text(
                                                            "Specify continuous spindle motor power rating in watts",
                                                        )
                                                        .changed()
                                                    {
                                                        m.spindle.power_watts =
                                                            has_power.then_some(1_500.0);
                                                        dirty = true;
                                                    }
                                                    if let Some(watts) = &mut m.spindle.power_watts
                                                    {
                                                        if ui
                                                            .add(
                                                                egui::DragValue::new(watts)
                                                                    .speed(50.0)
                                                                    .range(50.0..=50_000.0)
                                                                    .max_decimals(0)
                                                                    .suffix(" W"),
                                                            )
                                                            .on_hover_text(
                                                                "Continuous spindle motor power rating (watts)",
                                                            )
                                                            .changed()
                                                        {
                                                            dirty = true;
                                                        }
                                                    } else {
                                                        ui.weak("Unspecified");
                                                    }
                                                });
                                                ui.end_row();

                                                ui.label("Collet / Holder:");
                                                if ui
                                                    .text_edit_singleline(&mut m.spindle.collet_type)
                                                    .on_hover_text("Spindle taper or collet system (e.g., ER11, ER20, ISO20, BT30)")
                                                    .changed()
                                                {
                                                    dirty = true;
                                                }
                                                ui.end_row();

                                                ui.label("Supported Shanks:");
                                                ui.vertical(|ui| {
                                                    let mut remove_shank_idx: Option<usize> = None;
                                                    ui.horizontal_wrapped(|ui| {
                                                        if m.spindle
                                                            .supported_shank_diameters_mm
                                                            .is_empty()
                                                        {
                                                            ui.weak("Any shank diameter (unrestricted)");
                                                        } else {
                                                            for (s_idx, &dia) in m
                                                                .spindle
                                                                .supported_shank_diameters_mm
                                                                .iter()
                                                                .enumerate()
                                                            {
                                                                if ui
                                                                    .small_button(format!(
                                                                        "{} mm [x]",
                                                                        format_shank_mm(dia)
                                                                    ))
                                                                    .on_hover_text(
                                                                        "Click to remove this supported collet shank diameter",
                                                                    )
                                                                    .clicked()
                                                                {
                                                                    remove_shank_idx = Some(s_idx);
                                                                }
                                                            }
                                                        }
                                                    });
                                                    if let Some(r_idx) = remove_shank_idx {
                                                        m.spindle
                                                            .supported_shank_diameters_mm
                                                            .remove(r_idx);
                                                        dirty = true;
                                                    }
                                                    ui.horizontal_wrapped(|ui| {
                                                        ui.add(
                                                            egui::DragValue::new(
                                                                &mut self.new_shank_draft_mm,
                                                            )
                                                            .speed(0.1)
                                                            .range(0.5..=50.0)
                                                            .max_decimals(3)
                                                            .suffix(" mm"),
                                                        )
                                                        .on_hover_text(
                                                            "Custom collet shank diameter to add (mm)",
                                                        );
                                                        if ui
                                                            .small_button("+ Add Shank")
                                                            .on_hover_text(
                                                                "Add the specified shank diameter to this spindle's supported collet list",
                                                            )
                                                            .clicked()
                                                            && self.new_shank_draft_mm.is_finite()
                                                            && self.new_shank_draft_mm > 0.0
                                                        {
                                                            let exists = m
                                                                .spindle
                                                                .supported_shank_diameters_mm
                                                                .iter()
                                                                .any(|d| {
                                                                    (*d - self.new_shank_draft_mm)
                                                                        .abs()
                                                                        < 1e-3
                                                                });
                                                            if !exists {
                                                                m.spindle
                                                                    .supported_shank_diameters_mm
                                                                    .push(self.new_shank_draft_mm);
                                                                m.spindle
                                                                    .supported_shank_diameters_mm
                                                                    .sort_by(f64::total_cmp);
                                                                dirty = true;
                                                            }
                                                        }
                                                    });
                                                    ui.horizontal_wrapped(|ui| {
                                                        ui.weak("Presets:");
                                                        for preset in
                                                            [3.175_f64, 4.0, 6.0, 6.35, 8.0, 12.0]
                                                        {
                                                            let present = m
                                                                .spindle
                                                                .supported_shank_diameters_mm
                                                                .iter()
                                                                .any(|d| (*d - preset).abs() < 1e-3);
                                                            if ui
                                                                .selectable_label(
                                                                    present,
                                                                    format!(
                                                                        "{} mm",
                                                                        format_shank_mm(preset)
                                                                    ),
                                                                )
                                                                .on_hover_text(
                                                                    "Toggle this standard collet shank diameter",
                                                                )
                                                                .clicked()
                                                            {
                                                                if present {
                                                                    m.spindle
                                                                        .supported_shank_diameters_mm
                                                                        .retain(|d| {
                                                                            (*d - preset).abs()
                                                                                >= 1e-3
                                                                        });
                                                                } else {
                                                                    m.spindle
                                                                        .supported_shank_diameters_mm
                                                                        .push(preset);
                                                                    m.spindle
                                                                        .supported_shank_diameters_mm
                                                                        .sort_by(f64::total_cmp);
                                                                }
                                                                dirty = true;
                                                            }
                                                        }
                                                    });
                                                });
                                                ui.end_row();

                                                ui.label("Tool Change:");
                                                egui::ComboBox::from_id_salt("machine_tc_combo")
                                                    .selected_text(m.tool_change.label())
                                                    .show_ui(ui, |ui| {
                                                        for kind in [
                                                            MachineToolChangeKind::Manual,
                                                            MachineToolChangeKind::QuickChange,
                                                            MachineToolChangeKind::AutomaticAtc,
                                                        ] {
                                                            if ui
                                                                .selectable_value(
                                                                    &mut m.tool_change,
                                                                    kind,
                                                                    kind.label(),
                                                                )
                                                                .on_hover_text("Select tool-changing capability of this CNC machine")
                                                                .changed()
                                                            {
                                                                dirty = true;
                                                            }
                                                        }
                                                    })
                                                    .response
                                                    .on_hover_text("Select manual, quick-change, or automatic tool changer (ATC)");
                                                ui.end_row();

                                                ui.label("Toolsetter / Probe:");
                                                if ui
                                                    .checkbox(
                                                        &mut m.has_toolsetter,
                                                        "Machine has fixed toolsetter",
                                                    )
                                                    .on_hover_text("Enable if the machine automatically probes tool length on tool changes")
                                                    .changed()
                                                {
                                                    dirty = true;
                                                }
                                                ui.end_row();

                                                ui.label("Max Rapid XY / Z (mm/min):");
                                                ui.horizontal(|ui| {
                                                    dirty |= ui
                                                        .add(
                                                            egui::DragValue::new(
                                                                &mut m.kinematics.max_rapid_xy_mm_min,
                                                            )
                                                            .speed(50.0)
                                                            .range(100.0..=50_000.0)
                                                            .prefix("XY "),
                                                        )
                                                        .on_hover_text("Maximum XY G0 rapid traverse speed used for cycle-time estimation (mm/min)")
                                                        .changed();
                                                    dirty |= ui
                                                        .add(
                                                            egui::DragValue::new(
                                                                &mut m.kinematics.max_rapid_z_mm_min,
                                                            )
                                                            .speed(50.0)
                                                            .range(50.0..=30_000.0)
                                                            .prefix("Z "),
                                                        )
                                                        .on_hover_text("Maximum Z G0 rapid traverse speed used for cycle-time estimation (mm/min)")
                                                        .changed();
                                                });
                                                ui.end_row();

                                                ui.label("Max Cut Feed XY / Z (mm/min):");
                                                ui.horizontal(|ui| {
                                                    dirty |= ui
                                                        .add(
                                                            egui::DragValue::new(
                                                                &mut m.kinematics.max_cut_feed_xy_mm_min,
                                                            )
                                                            .speed(50.0)
                                                            .range(50.0..=30_000.0)
                                                            .prefix("XY "),
                                                        )
                                                        .on_hover_text("Maximum XY cutting feed rate (mm/min)")
                                                        .changed();
                                                    dirty |= ui
                                                        .add(
                                                            egui::DragValue::new(
                                                                &mut m.kinematics.max_cut_feed_z_mm_min,
                                                            )
                                                            .speed(25.0)
                                                            .range(20.0..=15_000.0)
                                                            .prefix("Z "),
                                                        )
                                                        .on_hover_text("Maximum Z plunge cutting feed rate (mm/min)")
                                                        .changed();
                                                });
                                                ui.end_row();

                                                ui.label("Accel / Tool Change Time:");
                                                ui.horizontal(|ui| {
                                                    dirty |= ui
                                                        .add(
                                                            egui::DragValue::new(
                                                                &mut m.kinematics.accel_mm_s2,
                                                            )
                                                            .speed(10.0)
                                                            .range(10.0..=20_000.0)
                                                            .suffix(" mm/s²"),
                                                        )
                                                        .on_hover_text("Nominal axis acceleration for trapezoidal cycle-time estimation (mm/s^2)")
                                                        .changed();
                                                    dirty |= ui
                                                        .add(
                                                            egui::DragValue::new(
                                                                &mut m.kinematics.tool_change_time_sec,
                                                            )
                                                            .speed(1.0)
                                                            .range(0.0..=300.0)
                                                            .suffix(" s"),
                                                        )
                                                        .on_hover_text("Estimated seconds per tool change added to simulated cycle time")
                                                        .changed();
                                                });
                                                ui.end_row();

                                                ui.label("Junction Deviation:");
                                                dirty |= ui
                                                    .add(
                                                        egui::DragValue::new(
                                                            &mut m.kinematics.junction_deviation_mm,
                                                        )
                                                        .speed(0.005)
                                                        .range(0.0..=5.0)
                                                        .fixed_decimals(3)
                                                        .suffix(" mm"),
                                                    )
                                                    .on_hover_text(
                                                        "Corner-blending junction deviation used for cycle-time lookahead estimation (mm)",
                                                    )
                                                    .changed();
                                                ui.end_row();

                                                ui.label("Default Post Profile:");
                                                egui::ComboBox::from_id_salt("machine_post_combo")
                                                    .selected_text(&m.default_post_profile)
                                                    .show_ui(ui, |ui| {
                                                        for p in &available_posts {
                                                            if ui
                                                                .selectable_value(
                                                                    &mut m.default_post_profile,
                                                                    p.id.clone(),
                                                                    format!("{} ({})", p.name, p.id),
                                                                )
                                                                .on_hover_text("Assign this post-processor as the machine default")
                                                                .changed()
                                                            {
                                                                dirty = true;
                                                            }
                                                        }
                                                    })
                                                    .response
                                                    .on_hover_text("Default G-code post-processor profile applied to new Setups on this machine");
                                                ui.end_row();
                                            });
                                    });
                                }
                                MachineEditorTab::Rotary => {
                                    let m = &mut machines[self.selected_idx];
                                    ui.add_enabled_ui(!is_builtin, |ui| {
                                        let mut has_rotary = m.rotary.is_some();
                                        if ui
                                            .checkbox(
                                                &mut has_rotary,
                                                "Equipped with 4th-Axis Rotary Module",
                                            )
                                            .changed()
                                        {
                                            if has_rotary {
                                                m.rotary = Some(MachineRotaryDefaults {
                                                    axis: "A".to_string(),
                                                    max_stock_diameter_mm: 92.0,
                                                    max_stock_length_mm: 200.0,
                                                    chuck_diameter_mm: 50.0,
                                                    chuck_length_mm: 20.0,
                                                    default_post_profile_4axis: Some(
                                                        "grblhal_4axis".to_string(),
                                                    ),
                                                });
                                            } else {
                                                m.rotary = None;
                                            }
                                            dirty = true;
                                        }

                                        if let Some(rot) = &mut m.rotary {
                                            ui.add_space(6.0);
                                            egui::Grid::new("machine_rotary_grid")
                                                .num_columns(2)
                                                .spacing([12.0, 6.0])
                                                .show(ui, |ui| {
                                                    ui.label("Rotary Axis:");
                                                    ui.horizontal(|ui| {
                                                        dirty |= ui
                                                            .selectable_value(
                                                                &mut rot.axis,
                                                                "A".to_string(),
                                                                "A-Axis (around X)",
                                                            )
                                                            .changed();
                                                        dirty |= ui
                                                            .selectable_value(
                                                                &mut rot.axis,
                                                                "B".to_string(),
                                                                "B-Axis (around Y)",
                                                            )
                                                            .changed();
                                                    });
                                                    ui.end_row();

                                                    ui.label("Max Stock Diameter / Length:");
                                                    ui.horizontal(|ui| {
                                                        dirty |= ui
                                                            .add(
                                                                egui::DragValue::new(
                                                                    &mut rot.max_stock_diameter_mm,
                                                                )
                                                                .speed(1.0)
                                                                .range(5.0..=500.0)
                                                                .prefix("Ø")
                                                                .suffix(" mm"),
                                                            )
                                                            .on_hover_text("Maximum stock swing diameter over the rotary bed (mm)")
                                                            .changed();
                                                        dirty |= ui
                                                            .add(
                                                                egui::DragValue::new(
                                                                    &mut rot.max_stock_length_mm,
                                                                )
                                                                .speed(1.0)
                                                                .range(10.0..=2000.0)
                                                                .prefix("L ")
                                                                .suffix(" mm"),
                                                            )
                                                            .on_hover_text("Maximum cylindrical stock length between chuck and tailstock (mm)")
                                                            .changed();
                                                    });
                                                    ui.end_row();

                                                    ui.label("Chuck Diameter / Length:");
                                                    ui.horizontal(|ui| {
                                                        dirty |= ui
                                                            .add(
                                                                egui::DragValue::new(
                                                                    &mut rot.chuck_diameter_mm,
                                                                )
                                                                .speed(1.0)
                                                                .range(10.0..=300.0)
                                                                .prefix("Ø")
                                                                .suffix(" mm"),
                                                            )
                                                            .on_hover_text("Outer diameter of the rotary chuck jaws used for collision checks (mm)")
                                                            .changed();
                                                        dirty |= ui
                                                            .add(
                                                                egui::DragValue::new(
                                                                    &mut rot.chuck_length_mm,
                                                                )
                                                                .speed(1.0)
                                                                .range(0.0..=200.0)
                                                                .prefix("L ")
                                                                .suffix(" mm"),
                                                            )
                                                            .on_hover_text("Axial length of the rotary chuck body protruding along the rotary axis (mm)")
                                                            .changed();
                                                    });
                                                    ui.end_row();

                                                    ui.label("Max Rotary Slew:");
                                                    dirty |= ui
                                                        .add(
                                                            egui::DragValue::new(
                                                                &mut m.kinematics
                                                                    .max_rotary_rate_deg_min,
                                                            )
                                                            .speed(100.0)
                                                            .range(60.0..=36_000.0)
                                                            .suffix(" deg/min"),
                                                        )
                                                        .on_hover_text("Maximum angular slew speed of the 4th-axis rotary table (deg/min)")
                                                        .changed();
                                                    ui.end_row();

                                                    ui.label("Default 4-Axis Post:");
                                                    let current_p4 = rot
                                                        .default_post_profile_4axis
                                                        .clone()
                                                        .unwrap_or_else(|| {
                                                            "linuxcnc_4axis".to_string()
                                                        });
                                                    egui::ComboBox::from_id_salt(
                                                        "machine_4ax_post_combo",
                                                    )
                                                    .selected_text(&current_p4)
                                                    .show_ui(ui, |ui| {
                                                        for p in available_posts
                                                            .iter()
                                                            .filter(|p| p.rotary.enabled)
                                                        {
                                                            if ui
                                                                .selectable_label(
                                                                    current_p4 == p.id,
                                                                    format!(
                                                                        "{} ({})",
                                                                        p.name, p.id
                                                                    ),
                                                                )
                                                                .on_hover_text("Select default 4-axis post-processor for this machine")
                                                                .clicked()
                                                            {
                                                                rot.default_post_profile_4axis =
                                                                    Some(p.id.clone());
                                                                dirty = true;
                                                            }
                                                        }
                                                    })
                                                    .response
                                                    .on_hover_text("Default 4-axis rotary post-processor profile for Setups using this machine");
                                                    ui.end_row();
                                                });
                                        }
                                    });
                                }
                                MachineEditorTab::Tools => {
                                    ui.horizontal_wrapped(|ui| {
                                        if ui
                                            .button("Open in Tool Library ->")
                                            .on_hover_text("Open the Tool Library Manager focused on this machine's tool set")
                                            .clicked()
                                        {
                                            actions.open_tool_library_for_machine =
                                                Some(m_id_copy.clone());
                                        }
                                        if !is_builtin && machines.len() > 1 {
                                            if self.copy_source_machine_idx >= machines.len()
                                                || self.copy_source_machine_idx == self.selected_idx
                                            {
                                                self.copy_source_machine_idx =
                                                    usize::from(self.selected_idx == 0);
                                            }
                                            ui.separator();
                                            ui.label("Copy tool from:");
                                            egui::ComboBox::from_id_salt("copy_tool_src_machine")
                                                .selected_text(
                                                    &machines[self.copy_source_machine_idx].name,
                                                )
                                                .show_ui(ui, |ui| {
                                                    for (i, other) in machines.iter().enumerate() {
                                                        if i == self.selected_idx {
                                                            continue;
                                                        }
                                                        ui.selectable_value(
                                                            &mut self.copy_source_machine_idx,
                                                            i,
                                                            &other.name,
                                                        )
                                                        .on_hover_text("Choose source machine to copy a cutting tool from");
                                                    }
                                                })
                                                .response
                                                .on_hover_text("Select another CNC machine profile to copy tools from");
                                            ui.menu_button("+ Add Tool", |ui| {
                                                for (t_idx, tool) in machines
                                                    [self.copy_source_machine_idx]
                                                    .tools
                                                    .iter()
                                                    .enumerate()
                                                {
                                                    if ui
                                                        .button(format!(
                                                            "T{} {}",
                                                            tool.tool_number, tool.name
                                                        ))
                                                        .on_hover_text("Copy this tool and its presets into the selected custom machine")
                                                        .clicked()
                                                    {
                                                        copy_from_idx_and_tool = Some((
                                                            self.copy_source_machine_idx,
                                                            t_idx,
                                                        ));
                                                        ui.close();
                                                    }
                                                }
                                            });
                                        }
                                    });
                                    ui.add_space(4.0);
                                    for tool in &machines[self.selected_idx].tools {
                                        ui.label(format!(
                                            "T{:<2}  {}  —  {} | Ø{:.3} mm | LOC {:.1} mm | Shank Ø{:.3} mm ({} preset{})",
                                            tool.tool_number,
                                            tool.name,
                                            tool.geometry.kind_label(),
                                            tool.geometry.cutting_diameter_mm(),
                                            tool.geometry.flute_length_mm(),
                                            tool.geometry.shank_diameter_mm(),
                                            tool.presets.len(),
                                            if tool.presets.len() == 1 { "" } else { "s" }
                                        ));
                                    }
                                }
                            }
                        });

                    if let Some((src_m_idx, src_t_idx)) = copy_from_idx_and_tool {
                        let mut copied = machines[src_m_idx].tools[src_t_idx].clone();
                        let dst = &mut machines[self.selected_idx];
                        let next_t = dst.tools.iter().map(|t| t.tool_number).max().unwrap_or(0) + 1;
                        copied.tool_number = next_t;
                        copied.length_offset_number = next_t;
                        copied.diameter_offset_number = next_t;
                        if dst.tools.iter().any(|t| t.id == copied.id) {
                            copied.id =
                                chip_tools::ToolId(format!("{}_t{next_t}", copied.id.0));
                        }
                        dst.tools.push(copied);
                        dst.sort_tools_deterministically();
                        dirty = true;
                    }

                    if dirty && !is_builtin {
                        match self.persist_machine(&machines[self.selected_idx]) {
                            Ok(_) => {
                                actions.machines_modified = true;
                                self.status_banner = Some((
                                    false,
                                    format!(
                                        "Saved custom machine '{}'.",
                                        machines[self.selected_idx].name
                                    ),
                                ));
                            }
                            Err(err) => {
                                self.status_banner = Some((true, format!("Save error: {err}")));
                            }
                        }
                    }
                });
            });

        self.open = keep_open;
        actions
    }
}

fn format_shank_mm(dia_mm: f64) -> String {
    let raw = format!("{dia_mm:.3}");
    raw.trim_end_matches('0').trim_end_matches('.').to_string()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_machines::{MachineDefinition, builtin_generic_machine, clone_machine_definition};

    use super::{MachineManagerActions, MachineManagerModalState};

    fn find_text_center(shape: &egui::Shape, needle: &str) -> Option<egui::Pos2> {
        match shape {
            egui::Shape::Text(t) if t.galley.text() == needle => {
                Some(t.pos + t.galley.size() * 0.5)
            }
            egui::Shape::Vec(v) => v.iter().find_map(|s| find_text_center(s, needle)),
            _ => None,
        }
    }

    fn click_at(pt: egui::Pos2) -> Vec<egui::Event> {
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

    fn type_replace_events(text: &str, commit_enter: bool) -> Vec<egui::Event> {
        let mut ev = vec![
            egui::Event::Key {
                key: egui::Key::A,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::COMMAND,
            },
            egui::Event::Text(text.to_string()),
        ];
        if commit_enter {
            ev.push(egui::Event::Key {
                key: egui::Key::Enter,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::NONE,
            });
        }
        ev
    }

    fn run_mm_frame(
        ctx: &egui::Context,
        state: &mut MachineManagerModalState,
        machines: &mut Vec<MachineDefinition>,
        logs: &mut Vec<String>,
        events: Vec<egui::Event>,
    ) -> (Vec<egui::epaint::ClippedShape>, MachineManagerActions) {
        let mut actions = MachineManagerActions::default();
        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(1100.0, 900.0),
                )),
                events,
                ..Default::default()
            },
            |ui| {
                actions = state.show(ui.ctx(), machines, "generic", "generic", logs);
            },
        );
        out.textures_delta.clear();
        (out.shapes, actions)
    }

    #[test]
    fn m88_machine_manager_fields_readonly_builtin_and_custom_toml_roundtrip() {
        let tmp =
            std::env::temp_dir().join(format!("chipcutter_m88_machine_mgr_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();

        let builtin = builtin_generic_machine();
        let mut custom =
            clone_machine_definition(&builtin, "shop-bench-mill", "Shop Bench Mill").unwrap();
        custom.description = None;
        custom.spindle.power_watts = Some(1_500.0);
        custom.spindle.supported_shank_diameters_mm = vec![3.175, 6.0];
        custom.kinematics.junction_deviation_mm = 0.05;

        let mut machines = vec![builtin.clone(), custom];
        let mut logs = Vec::new();
        let ctx = egui::Context::default();
        let mut state = MachineManagerModalState::new();
        state.open = true;
        state.selected_idx = 0; // Built-in first
        state.custom_save_dir = Some(tmp.clone());

        // 1. Built-in machine must remain strictly read-only when clicking "+ Add Shank" or presets
        let _ = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, Vec::new());
        let (s_bi, _) = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, Vec::new());
        let add_bi_pt = s_bi
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "+ Add Shank"))
            .expect("Built-in Specs tab must still render '+ Add Shank' (disabled)");
        state.new_shank_draft_mm = 9.5;
        let (_, bi_act) = run_mm_frame(
            &ctx,
            &mut state,
            &mut machines,
            &mut logs,
            click_at(add_bi_pt),
        );
        assert!(!bi_act.machines_modified);
        assert_eq!(machines[0], builtin);

        // 2. Switch to custom machine and edit Description, Spindle Power, Shanks, and Junction Deviation
        state.selected_idx = 1;
        state.new_shank_draft_mm = 5.0;
        let _ = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, Vec::new());
        let (s_c1, _) = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, Vec::new());

        // Edit Description
        let desc_pt = s_c1
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Optional notes about this CNC machine"))
            .expect("Custom machine must show Description hint text");
        let _ = run_mm_frame(
            &ctx,
            &mut state,
            &mut machines,
            &mut logs,
            click_at(desc_pt),
        );
        let (_, act_desc) = run_mm_frame(
            &ctx,
            &mut state,
            &mut machines,
            &mut logs,
            type_replace_events("Rigid cast-iron benchtop mill", false),
        );
        assert!(act_desc.machines_modified);
        assert_eq!(
            machines[1].description.as_deref(),
            Some("Rigid cast-iron benchtop mill")
        );

        // Edit Spindle Power (W): 1500 W -> 2200 W
        let (s_c2, _) = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, Vec::new());
        let pwr_pt = s_c2
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "1500"))
            .expect("Spindle Power DragValue must show '1500'");
        let _ = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, click_at(pwr_pt));
        let _ = run_mm_frame(
            &ctx,
            &mut state,
            &mut machines,
            &mut logs,
            type_replace_events("2200", true),
        );
        let _ = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, Vec::new());
        assert_eq!(machines[1].spindle.power_watts, Some(2200.0));

        // Add custom shank 5.0 mm via "+ Add Shank"
        let (s_c3, _) = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, Vec::new());
        let add_pt = s_c3
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "+ Add Shank"))
            .expect("'+ Add Shank' button must be rendered");
        let _ = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, click_at(add_pt));
        assert_eq!(
            machines[1].spindle.supported_shank_diameters_mm,
            vec![3.175, 5.0, 6.0]
        );

        // Toggle quick preset "6.35 mm" ON
        let (s_c4, _) = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, Vec::new());
        let preset_pt = s_c4
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "6.35 mm"))
            .expect("Quick preset chip '6.35 mm' must be rendered");
        let _ = run_mm_frame(
            &ctx,
            &mut state,
            &mut machines,
            &mut logs,
            click_at(preset_pt),
        );
        assert_eq!(
            machines[1].spindle.supported_shank_diameters_mm,
            vec![3.175, 5.0, 6.0, 6.35]
        );

        // Remove chip "6 mm [x]"
        let (s_c5, _) = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, Vec::new());
        let rem_pt = s_c5
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "6 mm [x]"))
            .expect("Supported shank chip '6 mm [x]' must be rendered");
        let _ = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, click_at(rem_pt));
        assert_eq!(
            machines[1].spindle.supported_shank_diameters_mm,
            vec![3.175, 5.0, 6.35]
        );

        // Edit Junction Deviation: 0.050 mm -> 0.025 mm
        let (s_c6, _) = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, Vec::new());
        let jd_pt = s_c6
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "0.050"))
            .expect("Junction Deviation DragValue must display '0.050'");
        let _ = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, click_at(jd_pt));
        let _ = run_mm_frame(
            &ctx,
            &mut state,
            &mut machines,
            &mut logs,
            type_replace_events("0.025", true),
        );
        let _ = run_mm_frame(&ctx, &mut state, &mut machines, &mut logs, Vec::new());
        assert!(
            (machines[1].kinematics.junction_deviation_mm - 0.025).abs() < 1e-6,
            "junction_deviation_mm must update to 0.025; got {}",
            machines[1].kinematics.junction_deviation_mm
        );

        // 3. Verify .machine.toml disk roundtrip preserves every edited field!
        let saved_path = tmp.join("shop-bench-mill.machine.toml");
        assert!(
            saved_path.is_file(),
            "Custom machine TOML must be saved on edit"
        );
        let reloaded = MachineDefinition::load_from_path(&saved_path).unwrap();
        assert_eq!(
            reloaded.description.as_deref(),
            Some("Rigid cast-iron benchtop mill")
        );
        assert_eq!(reloaded.spindle.power_watts, Some(2200.0));
        assert_eq!(
            reloaded.spindle.supported_shank_diameters_mm,
            vec![3.175, 5.0, 6.35]
        );
        assert!((reloaded.kinematics.junction_deviation_mm - 0.025).abs() < 1e-9);

        let _ = std::fs::remove_dir_all(&tmp);
    }
}
