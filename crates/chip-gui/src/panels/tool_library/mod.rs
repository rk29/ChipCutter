mod geometry_editor;
mod presets_editor;

use chip_math::DisplayUnit;
use chip_project::{ProjectDocument, UndoHistory};
use chip_tools::{
    CoolantMode, CutterGeometry, CuttingPreset, MachineDefinition, SpindleDirection,
    SurfaceSpeedPresetSpec, Tool, ToolId, ToolLibrary, default_global_tool_library_path,
    import_fusion_library_path, starter_dovetail_60deg_12mm, starter_lollipop_6mm,
    starter_tslot_12mm,
};

use self::{
    geometry_editor::{draw_cutter_schematic, edit_offset_number_row, show_cutter_geometry_editor},
    presets_editor::show_presets_editor,
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

                    ui.add_enabled_ui(!is_readonly_builtin, |ui| {
                        ui.menu_button("+ New Undercut...", |ui| {
                            let mut picked: Option<(Tool, &'static str)> = None;
                            if ui
                                .button("Lollipop Endmill (Ø6 mm)")
                                .on_hover_text(
                                    "Create a relieved-neck spherical lollipop cutter for 3D underside deburring and concave undercuts",
                                )
                                .clicked()
                            {
                                picked = Some((starter_lollipop_6mm(), "lollipop"));
                            }
                            if ui
                                .button("Slot / T-Slot Mill (Ø12 × 4 mm)")
                                .on_hover_text(
                                    "Create a relieved-neck horizontal T-slot / keyway disc cutter",
                                )
                                .clicked()
                            {
                                picked = Some((starter_tslot_12mm(), "slot"));
                            }
                            if ui
                                .button("Dovetail Cutter (Ø12 mm 60°)")
                                .on_hover_text(
                                    "Create a relieved-neck 60° inverted-cone dovetail cutter",
                                )
                                .clicked()
                            {
                                picked = Some((starter_dovetail_60deg_12mm(), "dovetail"));
                            }
                            if let Some((mut tool, slug)) = picked {
                                let next_n = global_lib
                                    .tools
                                    .iter()
                                    .map(|t| t.tool_number)
                                    .max()
                                    .unwrap_or(0)
                                    + 1;
                                tool.id = ToolId(format!("user_{slug}_t{next_n}"));
                                tool.tool_number = next_n;
                                tool.length_offset_number = next_n;
                                tool.diameter_offset_number = next_n;
                                global_lib.tools.push(tool);
                                self.selected_tool_idx = global_lib.tools.len() - 1;
                                self.selected_preset_idx = 0;
                                actions.save_active_machine_tools = true;
                                ui.close();
                            }
                        })
                        .response
                        .on_hover_text(
                            "Create a starter Lollipop, Slot/T-Slot, or Dovetail undercut cutter",
                        )
                        .on_disabled_hover_text(
                            "Clone the built-in machine first to add custom cutters",
                        );
                    });

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


                            show_cutter_geometry_editor(ui, tool, unit);

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


                            show_presets_editor(
                                ui,
                                tool,
                                unit,
                                &mut self.selected_preset_idx,
                                &mut self.lock_chipload,
                            );
                            });
                        });
                });
            });

        self.open = keep_open;
        actions
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

    #[test]
    fn thread_mill_editor_and_schematic_render_single_point_and_multi_form() {
        let ctx = egui::Context::default();
        let mut global_lib = default_starter_tool_library();
        let single_idx = global_lib.tools.len();
        global_lib.tools.push(Tool {
            id: ToolId("test_tm_single".to_string()),
            tool_number: 20,
            length_offset_number: 20,
            diameter_offset_number: 20,
            name: "T20 4.75mm Single-Point Thread Mill".to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::AirBlast,
            stickout_length_mm: 24.0,
            geometry: CutterGeometry::ThreadMill {
                diameter_mm: 4.75,
                neck_diameter_mm: 2.85,
                neck_length_mm: 16.0,
                pitch_mm: 1.0,
                teeth_count: 1,
                thread_angle_deg: 60.0,
                flute_length_mm: 1.2,
                overall_length_mm: 50.0,
                shank_diameter_mm: 6.0,
                flutes: 3,
            },
            presets: Vec::new(),
        });
        let multi_idx = global_lib.tools.len();
        global_lib.tools.push(Tool {
            id: ToolId("test_tm_multi".to_string()),
            tool_number: 21,
            length_offset_number: 21,
            diameter_offset_number: 21,
            name: "T21 M8x1.25 Multi-Form Thread Mill".to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::Flood,
            stickout_length_mm: 26.0,
            geometry: CutterGeometry::ThreadMill {
                diameter_mm: 5.8,
                neck_diameter_mm: 3.6,
                neck_length_mm: 20.0,
                pitch_mm: 1.25,
                teeth_count: 6,
                thread_angle_deg: 60.0,
                flute_length_mm: 7.5,
                overall_length_mm: 55.0,
                shank_diameter_mm: 6.0,
                flutes: 3,
            },
            presets: Vec::new(),
        });

        let mut modal = ToolLibraryModalState::new();
        modal.open = true;
        let mut project = ProjectDocument::new("test_tm");
        let mut history = UndoHistory::new(8);
        let mut logs = Vec::new();
        let base_input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(1280.0, 900.0),
            )),
            ..Default::default()
        };

        for (idx, expected_mode) in [(single_idx, "(Single-Point)"), (multi_idx, "(Multi-Form)")] {
            modal.selected_tool_idx = idx;
            let _ = run_modal_frame(
                &ctx,
                base_input.clone(),
                &mut modal,
                &mut global_lib,
                &mut project,
                &mut history,
                &mut logs,
            );
            let out = run_modal_frame(
                &ctx,
                base_input.clone(),
                &mut modal,
                &mut global_lib,
                &mut project,
                &mut history,
                &mut logs,
            );
            let texts = collect_text_shapes(&out.shapes);
            assert!(
                texts.iter().any(|(t, _)| t == "Thread Mill"),
                "expected cutter type selector to display 'Thread Mill'"
            );
            assert!(
                texts.iter().any(|(t, _)| t == "Neck Length (Reach):"),
                "expected Thread Mill geometry fields to render"
            );
            assert!(
                texts.iter().any(|(t, _)| t == expected_mode),
                "expected mode tag '{expected_mode}' to render"
            );
            assert!(
                texts.iter().any(|(t, _)| t == "Geometry Valid"),
                "expected Thread Mill tool to validate cleanly"
            );
        }
    }

    #[test]
    fn undercut_cutters_render_editor_fields_derived_top_dia_and_schematics() {
        let ctx = egui::Context::default();
        let mut global_lib = default_starter_tool_library();
        let lolli_idx = global_lib.tools.len();
        global_lib.tools.push(starter_lollipop_6mm());
        let slot_idx = global_lib.tools.len();
        global_lib.tools.push(starter_tslot_12mm());
        let dove_idx = global_lib.tools.len();
        global_lib.tools.push(starter_dovetail_60deg_12mm());

        let mut modal = ToolLibraryModalState::new();
        modal.open = true;
        let mut project = ProjectDocument::new("test_undercuts");
        let mut history = UndoHistory::new(8);
        let mut logs = Vec::new();
        let base_input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(1280.0, 900.0),
            )),
            ..Default::default()
        };

        for (idx, expected_kind, expected_field) in [
            (lolli_idx, "Lollipop Endmill", "Sphere Join Height:"),
            (slot_idx, "Slot / T-Slot Mill", "Cutting Thickness:"),
            (dove_idx, "Dovetail Cutter", "Top Ø (Derived):"),
        ] {
            modal.selected_tool_idx = idx;
            let _ = run_modal_frame(
                &ctx,
                base_input.clone(),
                &mut modal,
                &mut global_lib,
                &mut project,
                &mut history,
                &mut logs,
            );
            let out = run_modal_frame(
                &ctx,
                base_input.clone(),
                &mut modal,
                &mut global_lib,
                &mut project,
                &mut history,
                &mut logs,
            );
            let texts = collect_text_shapes(&out.shapes);
            assert!(
                texts.iter().any(|(t, _)| t == expected_kind),
                "expected cutter selector to show '{expected_kind}'"
            );
            assert!(
                texts.iter().any(|(t, _)| t == expected_field),
                "expected parameter row '{expected_field}' for '{expected_kind}'"
            );
            assert!(
                texts.iter().any(|(t, _)| t == "Geometry Valid"),
                "expected '{expected_kind}' starter tool to validate cleanly"
            );
        }
    }
}
