use chip_math::DisplayUnit;
use chip_project::{ProjectDocument, UndoHistory};
use chip_tools::{
    CoolantMode, CutterGeometry, CuttingPreset, MachineDefinition, SpindleDirection,
    SurfaceSpeedPresetSpec, Tool, ToolId, ToolLibrary, default_global_tool_library_path,
    import_fusion_library_path,
};

/// Borrowed machine context for machine-scoped Tool Library display and read-only built-in protection.
pub struct ToolLibraryMachineContext<'a> {
    pub machines: &'a [MachineDefinition],
    pub active_machine_id: &'a str,
}

/// Actions emitted by [`ToolLibraryModalState::show_with_machine`] back to the app.
#[derive(Debug, Default)]
pub struct ToolLibraryMachineActions {
    pub switch_machine_id: Option<String>,
    pub clone_active_machine: bool,
    pub open_machine_manager: bool,
    pub save_active_machine_tools: bool,
}

const TOOL_NUMBER_LABEL: &str = "Tool Number (T#):";
const TOOL_NUMBER_TOOLTIP: &str =
    "Tool slot / pocket number emitted in tool-change G-code (e.g., T1 M6).";

const LENGTH_OFFSET_LABEL: &str = "Length Offset (H#):";
const LENGTH_OFFSET_TOOLTIP: &str = "Tool length offset register emitted with G43 H# (e.g., G43 H1) for Z height compensation. Typically matches Tool Number (T#).";

const DIAMETER_OFFSET_LABEL: &str = "Diameter Offset (D#):";
const DIAMETER_OFFSET_TOOLTIP: &str = "Cutter diameter/radius wear compensation register emitted with G41/G42 D# (e.g., G41 D1). Typically matches Tool Number (T#).";

const NATIVE_UNIT_TOOLTIP: &str = "Preferred display unit for this cutter's dimensions and presets (stored internally in millimeters).";

/// Persistent UI state for the modal/floating **Tool Library Manager** window (`04-tool-library.md`).
#[derive(Debug, Default)]
pub struct ToolLibraryModalState {
    pub open: bool,
    pub selected_tool_idx: usize,
    pub selected_preset_idx: usize,
    pub lock_chipload: bool,
    pub status_banner: Option<String>,
}

impl ToolLibraryModalState {
    pub fn new() -> Self {
        Self {
            open: false,
            selected_tool_idx: 0,
            selected_preset_idx: 0,
            lock_chipload: true,
            status_banner: None,
        }
    }

    /// Shows the floating **Tool Library Manager** window if `self.open == true`.
    pub fn show(
        &mut self,
        ctx: &egui::Context,
        global_lib: &mut ToolLibrary,
        project: &mut ProjectDocument,
        history: &mut UndoHistory,
        logs: &mut Vec<String>,
    ) {
        let _ = self.show_with_machine(ctx, global_lib, project, history, logs, None);
    }

    /// Shows the floating **Tool Library Manager** scoped to the active [`MachineDefinition`].
    pub fn show_with_machine(
        &mut self,
        ctx: &egui::Context,
        global_lib: &mut ToolLibrary,
        project: &mut ProjectDocument,
        history: &mut UndoHistory,
        logs: &mut Vec<String>,
        machine_ctx: Option<ToolLibraryMachineContext<'_>>,
    ) -> ToolLibraryMachineActions {
        let mut actions = ToolLibraryMachineActions::default();
        if !self.open {
            return actions;
        }
        let mut keep_open = self.open;
        let active_machine_opt = machine_ctx.as_ref().and_then(|mc| {
            mc.machines
                .iter()
                .find(|m| m.id.eq_ignore_ascii_case(mc.active_machine_id))
                .or_else(|| mc.machines.first())
        });
        let is_readonly_builtin = active_machine_opt.is_some_and(|m| m.is_builtin);

        egui::Window::new("ChipCutter Tool Library & Cutting Presets")
            .open(&mut keep_open)
            .default_size([860.0, 590.0])
            .resizable(true)
            .show(ctx, |ui| {
                // Machine Scope Bar
                if let Some(mc) = &machine_ctx {
                    ui.horizontal_wrapped(|ui| {
                        ui.strong("Machine:");
                        let active_label = active_machine_opt
                            .map_or(mc.active_machine_id, |m| m.name.as_str());
                        egui::ComboBox::from_id_salt("tool_lib_machine_scope")
                            .selected_text(active_label)
                            .show_ui(ui, |ui| {
                                for m in mc.machines {
                                    let tag = if m.is_builtin {
                                        "[Built-In]"
                                    } else {
                                        "[Custom]"
                                    };
                                    if ui
                                        .selectable_label(
                                            m.id.eq_ignore_ascii_case(mc.active_machine_id),
                                            format!("{}  {} ({} tools)", m.name, tag, m.tools.len()),
                                        )
                                        .on_hover_text("Switch active project machine and view its tool library")
                                        .clicked()
                                    {
                                        actions.switch_machine_id = Some(m.id.clone());
                                        self.selected_tool_idx = 0;
                                        self.selected_preset_idx = 0;
                                    }
                                }
                            })
                            .response
                            .on_hover_text("Select which CNC machine's tool library is shown and edited");
                        if is_readonly_builtin {
                            ui.colored_label(
                                egui::Color32::from_rgb(250, 204, 21),
                                "Read-Only Built-In",
                            );
                            if ui
                                .button("Clone Machine to Edit Tools")
                                .on_hover_text(
                                    "Clone this built-in machine into your user configuration folder to customize its tools.",
                                )
                                .clicked()
                            {
                                actions.clone_active_machine = true;
                            }
                        } else {
                            ui.colored_label(
                                egui::Color32::from_rgb(56, 189, 248),
                                "Custom (Editable)",
                            );
                        }
                        if ui
                            .button("Manage Machines...")
                            .on_hover_text("Open Machine Manager to configure machine travels, spindle limits, and custom profiles")
                            .clicked()
                        {
                            actions.open_machine_manager = true;
                        }
                    });
                    ui.separator();
                }

                // Top toolbar: Import Fusion 360, Load/Save `.tools.toml`, Add Tool
                ui.horizontal_wrapped(|ui| {
                    if ui
                        .add_enabled(!is_readonly_builtin, egui::Button::new("+ New Cutter"))
                        .on_hover_text("Create a new cutting tool in the active machine's library")
                        .on_disabled_hover_text("Clone the built-in machine first to add custom cutters")
                        .clicked()
                    {
                        let next_n = global_lib
                            .tools
                            .iter()
                            .map(|t| t.tool_number)
                            .max()
                            .unwrap_or(0)
                            + 1;
                        global_lib.tools.push(Tool {
                            id: ToolId(format!("user_t{next_n}")),
                            tool_number: next_n,
                            length_offset_number: next_n,
                            diameter_offset_number: next_n,
                            name: format!("T{next_n} 6mm Flat Endmill"),
                            vendor: None,
                            product_id: None,
                            native_unit: DisplayUnit::Millimeter,
                            spindle_direction: SpindleDirection::Cw,
                            default_coolant: CoolantMode::AirBlast,
                            stickout_length_mm: 24.0,
                            geometry: CutterGeometry::FlatEndmill {
                                diameter_mm: 6.0,
                                flute_length_mm: 18.0,
                                overall_length_mm: 50.0,
                                shank_diameter_mm: 6.0,
                                flutes: 3,
                            },
                            presets: vec![CuttingPreset::from_surface_speed(
                                SurfaceSpeedPresetSpec {
                                    id: "default_al",
                                    name: "Aluminum 6061",
                                    material: "Aluminum 6061",
                                    diameter_mm: 6.0,
                                    flutes: 3,
                                    surface_speed_m_min: 240.0,
                                    chipload_mm_tooth: 0.04,
                                    default_stepdown_mm: 3.0,
                                    default_stepover_mm: 2.4,
                                    coolant_override: None,
                                },
                            )],
                        });
                        self.selected_tool_idx = global_lib.tools.len() - 1;
                        self.selected_preset_idx = 0;
                        actions.save_active_machine_tools = true;
                    }

                    if ui
                        .add_enabled(
                            !is_readonly_builtin,
                            egui::Button::new("Import Fusion 360 (.json / .tools)…"),
                        )
                        .on_hover_text("Import cutters and cutting presets from an Autodesk Fusion 360 JSON or .tools archive")
                        .on_disabled_hover_text("Clone the built-in machine first to import tools")
                        .clicked()
                        && let Some(path) = rfd::FileDialog::new()
                            .add_filter("Fusion 360 Tool Library", &["json", "tools"])
                            .pick_file()
                    {
                        match import_fusion_library_path(&path, None) {
                            Ok(rep) => {
                                for t in rep.library.tools {
                                    if let Some(existing) = global_lib.tool_mut(&t.id) {
                                        *existing = t;
                                    } else {
                                        global_lib.tools.push(t);
                                    }
                                }
                                global_lib.sort_deterministically();
                                actions.save_active_machine_tools = true;
                                let msg = format!(
                                    "Imported {} tool(s) & {} preset(s) from '{}' ({} warning(s)).",
                                    rep.imported_tool_count,
                                    rep.imported_preset_count,
                                    path.display(),
                                    rep.warnings.len()
                                );
                                logs.push(msg.clone());
                                self.status_banner = Some(msg);
                            }
                            Err(err) => {
                                self.status_banner = Some(format!("Import error: {err}"));
                            }
                        }
                    }

                    if ui
                        .add_enabled(
                            !is_readonly_builtin,
                            egui::Button::new("Load .tools.toml…"),
                        )
                        .on_hover_text("Replace the active library with tools loaded from a .tools.toml file")
                        .on_disabled_hover_text("Clone the built-in machine first to load a custom tool library")
                        .clicked()
                        && let Some(path) = rfd::FileDialog::new()
                            .add_filter("ChipCutter Tool Library", &["toml"])
                            .pick_file()
                    {
                        match ToolLibrary::load_from_path(&path) {
                            Ok(loaded) => {
                                *global_lib = loaded;
                                self.selected_tool_idx = 0;
                                actions.save_active_machine_tools = true;
                                self.status_banner =
                                    Some(format!("Loaded library '{}'.", path.display()));
                            }
                            Err(e) => {
                                self.status_banner = Some(format!("Load failed: {e}"));
                            }
                        }
                    }

                    if machine_ctx.is_some() {
                        if ui
                            .add_enabled(
                                !is_readonly_builtin,
                                egui::Button::new("Save Machine Tools"),
                            )
                            .on_hover_text("Persist changes to the active custom machine's tool library on disk")
                            .on_disabled_hover_text("Built-in machines are read-only; clone the machine first to save edits")
                            .clicked()
                        {
                            actions.save_active_machine_tools = true;
                            self.status_banner = Some(
                                "Saved updated tool library to custom machine definition."
                                    .to_string(),
                            );
                        }
                    } else if ui
                        .button("Save Global Library")
                        .on_hover_text("Save edits to the user's default global tool library file")
                        .clicked()
                        && let Some(def_path) = default_global_tool_library_path()
                    {
                        match global_lib.save_to_path(&def_path) {
                            Ok(()) => {
                                self.status_banner = Some(format!(
                                    "Saved global tool library to '{}'.",
                                    def_path.display()
                                ));
                            }
                            Err(e) => {
                                self.status_banner = Some(format!("Save failed: {e}"));
                            }
                        }
                    }

                    if ui
                        .button("Export .tools.toml As…")
                        .on_hover_text("Export the current tool library to a portable .tools.toml file")
                        .clicked()
                        && let Some(path) = rfd::FileDialog::new()
                            .set_file_name("shop_tools.tools.toml")
                            .save_file()
                    {
                        match global_lib.save_to_path(&path) {
                            Ok(()) => {
                                self.status_banner =
                                    Some(format!("Exported to '{}'.", path.display()));
                            }
                            Err(e) => {
                                self.status_banner = Some(format!("Export failed: {e}"));
                            }
                        }
                    }
                });

                if let Some(banner) = &self.status_banner {
                    let tok = crate::theme::ThemeTokens::from_ui(ui);
                    ui.colored_label(tok.info_fg, banner);
                }
                ui.separator();

                if global_lib.tools.is_empty() {
                    ui.weak("Library is empty. Click '+ New Cutter' or import a Fusion 360 library.");
                    return;
                }
                if self.selected_tool_idx >= global_lib.tools.len() {
                    self.selected_tool_idx = global_lib.tools.len() - 1;
                }

                ui.columns(2, |cols| {
                    // Left column: cutter list + 2D profile schematic
                    cols[0].strong(format!(
                        "{} ({} tools)",
                        global_lib.name,
                        global_lib.tools.len()
                    ));
                    egui::ScrollArea::vertical()
                        .id_salt("tool_lib_left_list")
                        .max_height(280.0)
                        .show(&mut cols[0], |ui| {
                            for (idx, t) in global_lib.tools.iter().enumerate() {
                                let sel = idx == self.selected_tool_idx;
                                if ui
                                    .selectable_label(sel, t.summary_label())
                                    .on_hover_text("Inspect or edit this cutting tool's geometry and material presets")
                                    .clicked()
                                {
                                    self.selected_tool_idx = idx;
                                    self.selected_preset_idx = 0;
                                }
                            }
                        });

                    cols[0].separator();
                    let preview_tool = &global_lib.tools[self.selected_tool_idx];
                    cols[0].strong("2D Cutter Profile Preview");
                    draw_cutter_schematic(&mut cols[0], preview_tool);

                    cols[0].horizontal(|ui| {
                        if ui
                            .button("Sync / Embed into Active Project")
                            .on_hover_text("Copy/update this tool inside `.chip.toml` and mark dependent operations Dirty")
                            .clicked()
                        {
                            history.record_before_edit("Sync Tool into Project", None, project);
                            project.sync_tool_from_global(preview_tool);
                            self.status_banner = Some(format!(
                                "Synced T{} ('{}') into project.",
                                preview_tool.tool_number, preview_tool.name
                            ));
                        }
                    });

                    // Right column: Geometry + Coupled Cutting Presets editor
                    let tool = &mut global_lib.tools[self.selected_tool_idx];
                    egui::ScrollArea::vertical()
                        .id_salt("tool_lib_right_editor")
                        .show(&mut cols[1], |ui| {
                            ui.add_enabled_ui(!is_readonly_builtin, |ui| {
                            ui.strong("Cutter Identity & Offsets");
                            egui::Grid::new("tool_identity_offsets_grid")
                                .num_columns(2)
                                .spacing([10.0, 4.0])
                                .show(ui, |ui| {
                                    ui.label("Name:");
                                    ui.text_edit_singleline(&mut tool.name)
                                        .on_hover_text("Descriptive name for this cutting tool (shown in Toolpath Browser and G-code comments)");
                                    ui.end_row();

                                    edit_offset_number_row(
                                        ui,
                                        TOOL_NUMBER_LABEL,
                                        TOOL_NUMBER_TOOLTIP,
                                        &mut tool.tool_number,
                                    );
                                    edit_offset_number_row(
                                        ui,
                                        LENGTH_OFFSET_LABEL,
                                        LENGTH_OFFSET_TOOLTIP,
                                        &mut tool.length_offset_number,
                                    );
                                    edit_offset_number_row(
                                        ui,
                                        DIAMETER_OFFSET_LABEL,
                                        DIAMETER_OFFSET_TOOLTIP,
                                        &mut tool.diameter_offset_number,
                                    );

                                    ui.add(egui::Label::new("Unit:").sense(egui::Sense::hover()))
                                        .on_hover_text(NATIVE_UNIT_TOOLTIP);
                                    ui.horizontal(|ui| {
                                        ui.selectable_value(
                                            &mut tool.native_unit,
                                            DisplayUnit::Millimeter,
                                            "mm",
                                        )
                                        .on_hover_text(NATIVE_UNIT_TOOLTIP);
                                        ui.selectable_value(
                                            &mut tool.native_unit,
                                            DisplayUnit::Inch,
                                            "in",
                                        )
                                        .on_hover_text(NATIVE_UNIT_TOOLTIP);
                                    });
                                    ui.end_row();
                                });

                            let unit = tool.native_unit;
                            ui.separator();
                            ui.strong("Physical Geometry");

                            // Cutter Type switcher
                            let cur_label = tool.geometry.kind_label();
                            egui::ComboBox::from_id_salt("cutter_type_switch")
                                .selected_text(cur_label)
                                .show_ui(ui, |ui| {
                                    let d = tool.geometry.cutting_diameter_mm();
                                    let fl = tool.geometry.flute_length_mm();
                                    let oal = tool.geometry.overall_length_mm();
                                    let shk = tool.geometry.shank_diameter_mm();
                                    let n = tool.geometry.flutes();

                                    if ui
                                        .selectable_label(
                                            matches!(tool.geometry, CutterGeometry::FlatEndmill { .. }),
                                            "Flat Endmill",
                                        )
                                        .on_hover_text("Square-corner flat-bottom milling cutter")
                                        .clicked()
                                    {
                                        tool.geometry = CutterGeometry::FlatEndmill {
                                            diameter_mm: d,
                                            flute_length_mm: fl,
                                            overall_length_mm: oal,
                                            shank_diameter_mm: shk,
                                            flutes: n,
                                        };
                                    }
                                    if ui
                                        .selectable_label(
                                            matches!(
                                                tool.geometry,
                                                CutterGeometry::BullNoseEndmill { .. }
                                            ),
                                            "Bull-Nose Endmill",
                                        )
                                        .on_hover_text("Flat-bottom endmill with a defined corner fillet radius")
                                        .clicked()
                                    {
                                        tool.geometry = CutterGeometry::BullNoseEndmill {
                                            diameter_mm: d,
                                            corner_radius_mm: (0.1 * d).max(0.2),
                                            flute_length_mm: fl,
                                            overall_length_mm: oal,
                                            shank_diameter_mm: shk,
                                            flutes: n,
                                        };
                                    }
                                    if ui
                                        .selectable_label(
                                            matches!(tool.geometry, CutterGeometry::BallEndmill { .. }),
                                            "Ball Endmill",
                                        )
                                        .on_hover_text("Hemispherical ball-nose cutter for 3D surface finishing")
                                        .clicked()
                                    {
                                        tool.geometry = CutterGeometry::BallEndmill {
                                            diameter_mm: d,
                                            flute_length_mm: fl,
                                            overall_length_mm: oal,
                                            shank_diameter_mm: shk,
                                            flutes: n,
                                        };
                                    }
                                    if ui
                                        .selectable_label(
                                            matches!(tool.geometry, CutterGeometry::ChamferMill { .. }),
                                            "Chamfer / Spot Mill",
                                        )
                                        .on_hover_text("Conical V-bit or chamfer cutter for bevels, spot drilling, and V-carving")
                                        .clicked()
                                    {
                                        tool.geometry = CutterGeometry::ChamferMill {
                                            diameter_mm: d,
                                            tip_diameter_mm: 0.2,
                                            included_angle_deg: 90.0,
                                            flute_length_mm: fl,
                                            overall_length_mm: oal,
                                            shank_diameter_mm: shk,
                                            flutes: n,
                                        };
                                    }
                                    if ui
                                        .selectable_label(
                                            matches!(tool.geometry, CutterGeometry::TwistDrill { .. }),
                                            "Twist Drill",
                                        )
                                        .on_hover_text("Pointed twist drill for axial drilling and peck cycles")
                                        .clicked()
                                    {
                                        tool.geometry = CutterGeometry::TwistDrill {
                                            diameter_mm: d,
                                            point_angle_deg: 118.0,
                                            flute_length_mm: fl,
                                            overall_length_mm: oal,
                                            shank_diameter_mm: shk,
                                            flutes: 2,
                                        };
                                    }
                                    if ui
                                        .selectable_label(
                                            matches!(tool.geometry, CutterGeometry::FaceMill { .. }),
                                            "Face / Fly Cutter",
                                        )
                                        .on_hover_text("Large-diameter indexable face mill or fly cutter for stock facing")
                                        .clicked()
                                    {
                                        tool.geometry = CutterGeometry::FaceMill {
                                            cutting_diameter_mm: d,
                                            outside_diameter_mm: d * 1.05,
                                            corner_radius_mm: 0.8,
                                            flute_length_mm: fl,
                                            overall_length_mm: oal,
                                            shank_diameter_mm: shk,
                                            flutes: n,
                                        };
                                    }
                                })
                                .response
                                .on_hover_text("Select physical cutter profile geometry");

                            edit_cutter_geometry_fields(ui, &mut tool.geometry, unit);
                            edit_mm_drag(ui, "Holder Stickout:", &mut tool.stickout_length_mm, unit);

                            let tok = crate::theme::ThemeTokens::from_ui(ui);
                            match tool.validate() {
                                Ok(()) => {
                                    ui.horizontal(|ui| {
                                        crate::widgets::status_badge_icon(
                                            ui,
                                            crate::widgets::StatusBadgeKind::Clean,
                                            None,
                                        );
                                        ui.colored_label(tok.clean_fg, "Geometry Valid");
                                    });
                                }
                                Err(err) => {
                                    ui.horizontal(|ui| {
                                        crate::widgets::status_badge_icon(
                                            ui,
                                            crate::widgets::StatusBadgeKind::Error,
                                            None,
                                        );
                                        ui.colored_label(tok.error_fg, err.to_string());
                                    });
                                }
                            }

                            ui.separator();
                            ui.horizontal(|ui| {
                                ui.strong("Material Cutting Presets");
                                if ui
                                    .small_button("+ Preset")
                                    .on_hover_text("Add a new material feeds-and-speeds preset for this cutter")
                                    .clicked()
                                {
                                    let d = tool.geometry.cutting_diameter_mm();
                                    let fl = tool.geometry.flutes();
                                    let pid = format!("preset_{}", tool.presets.len() + 1);
                                    tool.presets.push(CuttingPreset::from_surface_speed(
                                        SurfaceSpeedPresetSpec {
                                            id: &pid,
                                            name: "New Material Preset",
                                            material: "Custom",
                                            diameter_mm: d,
                                            flutes: fl,
                                            surface_speed_m_min: 200.0,
                                            chipload_mm_tooth: 0.035,
                                            default_stepdown_mm: 0.5 * d,
                                            default_stepover_mm: 0.4 * d,
                                            coolant_override: None,
                                        },
                                    ));
                                    self.selected_preset_idx = tool.presets.len() - 1;
                                }
                            });

                            if !tool.presets.is_empty() {
                                if self.selected_preset_idx >= tool.presets.len() {
                                    self.selected_preset_idx = 0;
                                }
                                ui.horizontal_wrapped(|ui| {
                                    for (pi, p) in tool.presets.iter().enumerate() {
                                        if ui
                                            .selectable_label(pi == self.selected_preset_idx, &p.name)
                                            .on_hover_text("Select this material preset to edit its feeds, speeds, and cut depths")
                                            .clicked()
                                        {
                                            self.selected_preset_idx = pi;
                                        }
                                    }
                                });

                                let dia_mm = tool.geometry.cutting_diameter_mm();
                                let flutes = tool.geometry.flutes();
                                let preset = &mut tool.presets[self.selected_preset_idx];

                                ui.checkbox(
                                    &mut self.lock_chipload,
                                    "Lock Chipload when editing RPM / Surface Speed",
                                )
                                .on_hover_text(
                                    "When checked, changing RPM or surface speed automatically scales cutting feed to keep chipload per tooth constant",
                                );
                                ui.horizontal(|ui| {
                                    ui.label("Preset Name:");
                                    ui.text_edit_singleline(&mut preset.name)
                                        .on_hover_text("Descriptive name for this material/cutting preset");
                                });

                                let mut rpm = preset.spindle_rpm;
                                ui.horizontal(|ui| {
                                    ui.label("Spindle RPM:");
                                    if ui
                                        .add(
                                            egui::DragValue::new(&mut rpm)
                                                .speed(25.0)
                                                .range(100.0..=60_000.0)
                                                .suffix(" RPM"),
                                        )
                                        .on_hover_text("Spindle rotation speed (RPM); linked to Surface Speed and Chipload")
                                        .changed()
                                    {
                                        preset.set_spindle_rpm(
                                            rpm,
                                            dia_mm,
                                            flutes,
                                            self.lock_chipload,
                                        );
                                    }
                                });

                                let mut vc_disp =
                                    unit.surface_speed_from_m_min(preset.surface_speed_m_min);
                                ui.horizontal(|ui| {
                                    ui.label("Surface Speed:");
                                    if ui
                                        .add(
                                            egui::DragValue::new(&mut vc_disp)
                                                .speed(1.0)
                                                .range(1.0..=5_000.0)
                                                .suffix(format!(" {}", unit.surface_speed_suffix())),
                                        )
                                        .on_hover_text("Cutter perimeter cutting speed (m/min or SFM); automatically updates RPM")
                                        .changed()
                                    {
                                        preset.set_surface_speed_m_min(
                                            unit.surface_speed_to_m_min(vc_disp),
                                            dia_mm,
                                            flutes,
                                            self.lock_chipload,
                                        );
                                    }
                                });

                                let mut vf_disp =
                                    unit.feed_from_mm_min(preset.cutting_feed_mm_min);
                                ui.horizontal(|ui| {
                                    ui.label("Cutting Feed:");
                                    if ui
                                        .add(
                                            egui::DragValue::new(&mut vf_disp)
                                                .speed(5.0)
                                                .range(1.0..=25_000.0)
                                                .suffix(format!(" {}", unit.feed_suffix())),
                                        )
                                        .on_hover_text("Linear XY cutting feed rate; automatically updates Chipload")
                                        .changed()
                                    {
                                        preset.set_cutting_feed_mm_min(
                                            unit.feed_to_mm_min(vf_disp),
                                            flutes,
                                        );
                                    }
                                });

                                let mut fz_disp =
                                    unit.chipload_from_mm(preset.chipload_mm_tooth);
                                ui.horizontal(|ui| {
                                    ui.label("Chipload:");
                                    if ui
                                        .add(
                                            egui::DragValue::new(&mut fz_disp)
                                                .speed(0.001)
                                                .max_decimals(5)
                                                .range(0.0001..=5.0)
                                                .suffix(format!(" {}", unit.chipload_suffix())),
                                        )
                                        .on_hover_text("Chip thickness removed per flute per spindle revolution")
                                        .changed()
                                    {
                                        preset.set_chipload_mm_tooth(
                                            unit.chipload_to_mm(fz_disp),
                                            flutes,
                                        );
                                    }
                                });

                                edit_mm_drag(
                                    ui,
                                    "Default Stepdown:",
                                    &mut preset.default_stepdown_mm,
                                    unit,
                                );
                                edit_mm_drag(
                                    ui,
                                    "Default Stepover:",
                                    &mut preset.default_stepover_mm,
                                    unit,
                                );
                                let mut has_opt = preset.optimal_load_mm.is_some();
                                ui.horizontal(|ui| {
                                    if ui
                                        .checkbox(&mut has_opt, "Adaptive Optimal Load:")
                                        .on_hover_text("Set a preset radial engagement target for 2D/3D Adaptive roughing")
                                        .changed()
                                    {
                                        preset.optimal_load_mm = if has_opt {
                                            Some((0.20 * dia_mm).max(0.1))
                                        } else {
                                            None
                                        };
                                    }
                                    if let Some(opt_mm) = &mut preset.optimal_load_mm {
                                        let mut disp = unit.from_mm(*opt_mm);
                                        if ui
                                            .add(
                                                egui::DragValue::new(&mut disp)
                                                    .speed(0.05)
                                                    .range(0.01..=200.0)
                                                    .suffix(format!(" {}", unit.suffix())),
                                            )
                                            .on_hover_text("Target radial stepover width for constant-engagement Adaptive clearing")
                                            .changed()
                                        {
                                            *opt_mm = unit.to_mm(disp);
                                        }
                                    }
                                });
                            }
                            });
                        });
                });
            });

        self.open = keep_open;
        actions
    }
}

fn edit_offset_number_row(ui: &mut egui::Ui, label: &str, tooltip: &str, value: &mut u32) {
    ui.add(egui::Label::new(label).sense(egui::Sense::hover()))
        .on_hover_text(tooltip);
    ui.add(egui::DragValue::new(value).range(1..=999))
        .on_hover_text(tooltip);
    ui.end_row();
}

fn edit_mm_drag(ui: &mut egui::Ui, label: &str, val_mm: &mut f64, unit: DisplayUnit) {
    let mut disp = unit.from_mm(*val_mm);
    let tip = format!(
        "Edit {} in current tool units ({})",
        label.trim_end_matches(':'),
        unit.suffix()
    );
    ui.horizontal(|ui| {
        ui.label(label);
        if ui
            .add(
                egui::DragValue::new(&mut disp)
                    .speed(0.05)
                    .max_decimals(4)
                    .range(0.0..=1_000.0)
                    .suffix(format!(" {}", unit.suffix())),
            )
            .on_hover_text(tip)
            .changed()
        {
            *val_mm = unit.to_mm(disp);
        }
    });
}

fn edit_cutter_geometry_fields(ui: &mut egui::Ui, geom: &mut CutterGeometry, unit: DisplayUnit) {
    match geom {
        CutterGeometry::FlatEndmill {
            diameter_mm,
            flute_length_mm,
            overall_length_mm,
            shank_diameter_mm,
            flutes,
        }
        | CutterGeometry::BallEndmill {
            diameter_mm,
            flute_length_mm,
            overall_length_mm,
            shank_diameter_mm,
            flutes,
        } => {
            edit_mm_drag(ui, "Cutting Diameter:", diameter_mm, unit);
            edit_mm_drag(ui, "Flute Length:", flute_length_mm, unit);
            edit_mm_drag(ui, "Overall Length:", overall_length_mm, unit);
            edit_mm_drag(ui, "Shank Diameter:", shank_diameter_mm, unit);
            ui.horizontal(|ui| {
                ui.label("Flutes:");
                ui.add(egui::DragValue::new(flutes).range(1..=24))
                    .on_hover_text("Number of cutting flutes on the tool");
            });
        }
        CutterGeometry::BullNoseEndmill {
            diameter_mm,
            corner_radius_mm,
            flute_length_mm,
            overall_length_mm,
            shank_diameter_mm,
            flutes,
        } => {
            edit_mm_drag(ui, "Cutting Diameter:", diameter_mm, unit);
            edit_mm_drag(ui, "Corner Radius:", corner_radius_mm, unit);
            edit_mm_drag(ui, "Flute Length:", flute_length_mm, unit);
            edit_mm_drag(ui, "Overall Length:", overall_length_mm, unit);
            edit_mm_drag(ui, "Shank Diameter:", shank_diameter_mm, unit);
            ui.horizontal(|ui| {
                ui.label("Flutes:");
                ui.add(egui::DragValue::new(flutes).range(1..=24))
                    .on_hover_text("Number of cutting flutes on the bull-nose cutter");
            });
        }
        CutterGeometry::ChamferMill {
            diameter_mm,
            tip_diameter_mm,
            included_angle_deg,
            flute_length_mm,
            overall_length_mm,
            shank_diameter_mm,
            flutes,
        } => {
            edit_mm_drag(ui, "Cutting Diameter:", diameter_mm, unit);
            edit_mm_drag(ui, "Tip Diameter:", tip_diameter_mm, unit);
            ui.horizontal(|ui| {
                ui.label("Included Angle:");
                ui.add(
                    egui::DragValue::new(included_angle_deg)
                        .range(10.0..=170.0)
                        .suffix("°"),
                )
                .on_hover_text("Full included cone angle of the chamfer mill or V-bit (e.g., 60 or 90 degrees)");
            });
            edit_mm_drag(ui, "Flute Length:", flute_length_mm, unit);
            edit_mm_drag(ui, "Overall Length:", overall_length_mm, unit);
            edit_mm_drag(ui, "Shank Diameter:", shank_diameter_mm, unit);
            ui.horizontal(|ui| {
                ui.label("Flutes:");
                ui.add(egui::DragValue::new(flutes).range(1..=24))
                    .on_hover_text("Number of cutting flutes on the chamfer/V-bit cutter");
            });
        }
        CutterGeometry::TwistDrill {
            diameter_mm,
            point_angle_deg,
            flute_length_mm,
            overall_length_mm,
            shank_diameter_mm,
            flutes,
        } => {
            edit_mm_drag(ui, "Drill Diameter:", diameter_mm, unit);
            ui.horizontal(|ui| {
                ui.label("Point Angle:");
                ui.add(
                    egui::DragValue::new(point_angle_deg)
                        .range(60.0..=160.0)
                        .suffix("°"),
                )
                .on_hover_text("Drill tip point angle (commonly 118 or 135 degrees)");
            });
            edit_mm_drag(ui, "Flute Length:", flute_length_mm, unit);
            edit_mm_drag(ui, "Overall Length:", overall_length_mm, unit);
            edit_mm_drag(ui, "Shank Diameter:", shank_diameter_mm, unit);
            ui.horizontal(|ui| {
                ui.label("Flutes:");
                ui.add(egui::DragValue::new(flutes).range(1..=8))
                    .on_hover_text("Number of flutes on the drill bit");
            });
        }
        CutterGeometry::FaceMill {
            cutting_diameter_mm,
            outside_diameter_mm,
            corner_radius_mm,
            flute_length_mm,
            overall_length_mm,
            shank_diameter_mm,
            flutes,
        } => {
            edit_mm_drag(ui, "Cutting Diameter:", cutting_diameter_mm, unit);
            edit_mm_drag(ui, "Outside Diameter:", outside_diameter_mm, unit);
            edit_mm_drag(ui, "Insert Corner R:", corner_radius_mm, unit);
            edit_mm_drag(ui, "Axial Depth:", flute_length_mm, unit);
            edit_mm_drag(ui, "Overall Length:", overall_length_mm, unit);
            edit_mm_drag(ui, "Arbor/Shank Dia:", shank_diameter_mm, unit);
            ui.horizontal(|ui| {
                ui.label("Inserts / Flutes:");
                ui.add(egui::DragValue::new(flutes).range(1..=16))
                    .on_hover_text("Number of carbide inserts or cutting teeth on the face mill");
            });
        }
    }
}

fn draw_cutter_schematic(ui: &mut egui::Ui, tool: &Tool) {
    let tok = crate::theme::ThemeTokens::from_ui(ui);
    let (rect, _) = ui.allocate_exact_size(egui::vec2(260.0, 128.0), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect(
        rect,
        4.0,
        tok.code_bg,
        egui::Stroke::new(1.0, tok.border),
        egui::StrokeKind::Inside,
    );

    let d = tool.geometry.cutting_diameter_mm().max(1.0) as f32;
    let shk = tool.geometry.shank_diameter_mm().max(1.0) as f32;
    let fl = tool.geometry.flute_length_mm().max(1.0) as f32;
    let oal = (tool.geometry.overall_length_mm() as f32).max(fl + 2.0);

    let max_w = d.max(shk).max(4.0);
    let scale = (96.0 / oal).min(90.0 / max_w).clamp(0.4, 8.0);
    let cx = rect.center().x;
    let top_y = rect.top() + 14.0;
    let shank_bot_y = top_y + (oal - fl) * scale;
    let tip_y = top_y + oal * scale;

    let r_shk = 0.5 * shk * scale;
    let r_cut = 0.5 * d * scale;

    // Shank rectangle (steel gray)
    let shank_rect = egui::Rect::from_min_max(
        egui::pos2(cx - r_shk, top_y),
        egui::pos2(cx + r_shk, shank_bot_y),
    );
    painter.rect_filled(shank_rect, 1.0, egui::Color32::from_rgb(122, 130, 142));

    // Fluted cutting body (gold/amber carbide tint)
    let flute_col = egui::Color32::from_rgb(232, 182, 68);
    match tool.geometry {
        CutterGeometry::ChamferMill {
            tip_diameter_mm,
            included_angle_deg,
            ..
        } => {
            let r_tip = 0.5 * (tip_diameter_mm as f32) * scale;
            let half_rad = 0.5 * (included_angle_deg as f32).to_radians();
            let cone_h = ((r_cut - r_tip) / half_rad.tan().max(0.1)).min(fl * scale);
            let shoulder_y = tip_y - cone_h;
            painter.add(egui::Shape::convex_polygon(
                vec![
                    egui::pos2(cx - r_cut, shank_bot_y),
                    egui::pos2(cx + r_cut, shank_bot_y),
                    egui::pos2(cx + r_cut, shoulder_y),
                    egui::pos2(cx + r_tip, tip_y),
                    egui::pos2(cx - r_tip, tip_y),
                    egui::pos2(cx - r_cut, shoulder_y),
                ],
                flute_col,
                egui::Stroke::NONE,
            ));
        }
        CutterGeometry::TwistDrill {
            point_angle_deg, ..
        } => {
            let half_rad = 0.5 * (point_angle_deg as f32).to_radians();
            let cone_h = (r_cut / half_rad.tan().max(0.2)).min(fl * scale);
            let shoulder_y = tip_y - cone_h;
            painter.add(egui::Shape::convex_polygon(
                vec![
                    egui::pos2(cx - r_cut, shank_bot_y),
                    egui::pos2(cx + r_cut, shank_bot_y),
                    egui::pos2(cx + r_cut, shoulder_y),
                    egui::pos2(cx, tip_y),
                    egui::pos2(cx - r_cut, shoulder_y),
                ],
                flute_col,
                egui::Stroke::NONE,
            ));
        }
        CutterGeometry::BallEndmill { .. } => {
            let body_bot = (tip_y - r_cut).max(shank_bot_y);
            painter.rect_filled(
                egui::Rect::from_min_max(
                    egui::pos2(cx - r_cut, shank_bot_y),
                    egui::pos2(cx + r_cut, body_bot),
                ),
                0.0,
                flute_col,
            );
            painter.circle_filled(egui::pos2(cx, body_bot), r_cut, flute_col);
        }
        _ => {
            painter.rect_filled(
                egui::Rect::from_min_max(
                    egui::pos2(cx - r_cut, shank_bot_y),
                    egui::pos2(cx + r_cut, tip_y),
                ),
                1.5,
                flute_col,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chip_tools::default_starter_tool_library;

    fn collect_text_shapes(shapes: &[egui::epaint::ClippedShape]) -> Vec<(String, egui::Rect)> {
        fn visit(shape: &egui::Shape, out: &mut Vec<(String, egui::Rect)>) {
            match shape {
                egui::Shape::Text(text_shape) => {
                    let rect = text_shape.visual_bounding_rect();
                    out.push((text_shape.galley.text().to_owned(), rect));
                }
                egui::Shape::Vec(children) => {
                    for child in children {
                        visit(child, out);
                    }
                }
                _ => {}
            }
        }

        let mut out = Vec::new();
        for clipped in shapes {
            visit(&clipped.shape, &mut out);
        }
        out
    }

    fn run_modal_frame(
        ctx: &egui::Context,
        input: egui::RawInput,
        modal: &mut ToolLibraryModalState,
        global_lib: &mut ToolLibrary,
        project: &mut ProjectDocument,
        history: &mut UndoHistory,
        logs: &mut Vec<String>,
    ) -> egui::FullOutput {
        let mut out = ctx.run_ui(input, |ui| {
            modal.show(ui.ctx(), global_lib, project, history, logs);
        });
        out.textures_delta.clear();
        out
    }

    #[test]
    fn offset_fields_show_descriptive_labels_and_hover_tooltips() {
        let ctx = egui::Context::default();
        for theme in [egui::Theme::Dark, egui::Theme::Light] {
            ctx.style_mut_of(theme, |s| s.interaction.tooltip_delay = 0.0);
        }

        let mut modal = ToolLibraryModalState::new();
        modal.open = true;
        let mut global_lib = default_starter_tool_library();
        if let Some(first_tool) = global_lib.tools.first_mut() {
            first_tool.tool_number = 11;
            first_tool.length_offset_number = 22;
            first_tool.diameter_offset_number = 33;
        }
        let mut project = ProjectDocument::new("test");
        let mut history = UndoHistory::new(8);
        let mut logs = Vec::new();

        let base_input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(1280.0, 800.0),
            )),
            ..Default::default()
        };

        // Run two layout frames so window & grid geometry settles.
        let _ = run_modal_frame(
            &ctx,
            base_input.clone(),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        let settled = run_modal_frame(
            &ctx,
            base_input.clone(),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );

        let texts = collect_text_shapes(&settled.shapes);
        let mut t = 1.0;
        for &(expected_label, value_text, expected_tooltip) in &[
            (TOOL_NUMBER_LABEL, "11", TOOL_NUMBER_TOOLTIP),
            (LENGTH_OFFSET_LABEL, "22", LENGTH_OFFSET_TOOLTIP),
            (DIAMETER_OFFSET_LABEL, "33", DIAMETER_OFFSET_TOOLTIP),
        ] {
            let label_rect = texts
                .iter()
                .find(|(text, _)| text == expected_label)
                .map(|(_, rect)| *rect);
            assert!(
                label_rect.is_some(),
                "expected descriptive label '{expected_label}' to be rendered in Tool Library Manager"
            );
            let value_rect = texts
                .iter()
                .find(|(text, _)| text == value_text)
                .map(|(_, rect)| *rect);
            assert!(
                value_rect.is_some(),
                "expected DragValue '{value_text}' for '{expected_label}' to be rendered"
            );

            for (target_kind, rect_opt) in [("label", label_rect), ("input", value_rect)] {
                let Some(rect) = rect_opt else {
                    return;
                };

                // Clear prior hover state by moving pointer off the window before hovering target.
                let mut clear_input = base_input.clone();
                clear_input.time = Some(t);
                clear_input
                    .events
                    .push(egui::Event::PointerMoved(egui::pos2(1270.0, 790.0)));
                t += 1.0;
                let _ = run_modal_frame(
                    &ctx,
                    clear_input,
                    &mut modal,
                    &mut global_lib,
                    &mut project,
                    &mut history,
                    &mut logs,
                );

                let mut hover_input = base_input.clone();
                hover_input.time = Some(t);
                hover_input
                    .events
                    .push(egui::Event::PointerMoved(rect.center()));
                t += 1.0;

                let _ = run_modal_frame(
                    &ctx,
                    hover_input,
                    &mut modal,
                    &mut global_lib,
                    &mut project,
                    &mut history,
                    &mut logs,
                );
                let mut still_input = base_input.clone();
                still_input.time = Some(t);
                t += 1.0;
                let hovered_out = run_modal_frame(
                    &ctx,
                    still_input,
                    &mut modal,
                    &mut global_lib,
                    &mut project,
                    &mut history,
                    &mut logs,
                );
                let hovered_texts = collect_text_shapes(&hovered_out.shapes);
                assert!(
                    hovered_texts
                        .iter()
                        .any(|(txt, _)| txt.contains(expected_tooltip)),
                    "hovering {target_kind} at {:?} for '{expected_label}' should render tooltip containing '{expected_tooltip}'",
                    rect.center(),
                );
            }
        }
    }
}
