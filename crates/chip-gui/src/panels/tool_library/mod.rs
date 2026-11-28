mod geometry_editor;
mod presets_editor;

use chip_machines::MachineDefinition;
use chip_math::DisplayUnit;
use chip_project::{ProjectDocument, UndoHistory};
use chip_tools::{
    CoolantMode, CutterGeometry, CuttingPreset, SpindleDirection, SurfaceSpeedPresetSpec, Tool,
    ToolId, ToolLibrary, default_global_tool_library_path, import_fusion_library_path,
    starter_dovetail_60deg_12mm, starter_lollipop_6mm, starter_tslot_12mm,
};

use crate::widgets::{grid_row, inspector_grid, section_card};

pub(crate) use self::geometry_editor::draw_cutter_schematic;
use self::{
    geometry_editor::{TOOL_COMBO_W, edit_offset_number_row, show_cutter_geometry_editor},
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

const TOOL_NUMBER_LABEL: &str = "Tool Number (T)";
const TOOL_NUMBER_TOOLTIP: &str =
    "Tool slot / pocket number emitted in tool-change G-code (e.g., T1 M6).";

const LENGTH_OFFSET_LABEL: &str = "Length Offset (H)";
const LENGTH_OFFSET_TOOLTIP: &str = "Tool length offset register emitted with G43 H# (e.g., G43 H1) for Z height compensation. Typically matches Tool Number (T).";

const DIAMETER_OFFSET_LABEL: &str = "Diameter Offset (D)";
const DIAMETER_OFFSET_TOOLTIP: &str = "Cutter diameter/radius wear compensation register emitted with G41/G42 D# (e.g., G41 D1). Typically matches Tool Number (T).";

const NATIVE_UNIT_TOOLTIP: &str = "Preferred display unit for this cutter's dimensions and presets (stored internally in millimeters).";

/// Persistent UI state for the modal/floating **Tool Library Manager** window (`04-tool-library.md`).
#[derive(Debug, Default)]
pub struct ToolLibraryModalState {
    pub open: bool,
    pub selected_tool_idx: usize,
    pub selected_preset_idx: usize,
    pub lock_chipload: bool,
    pub status_banner: Option<String>,
    pub scroll_to_selected: bool,
    pub pending_delete_tool_id: Option<ToolId>,
}

impl ToolLibraryModalState {
    pub fn new() -> Self {
        Self {
            open: false,
            selected_tool_idx: 0,
            selected_preset_idx: 0,
            lock_chipload: true,
            status_banner: None,
            scroll_to_selected: false,
            pending_delete_tool_id: None,
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
            .id(egui::Id::unique("chipcutter_tool_library_modal"))
            .open(&mut keep_open)
            .default_size([920.0, 680.0])
            .min_size([740.0, 480.0])
            .resizable(true)
            .collapsible(false)
            .show(ctx, |ui| {
                ui.set_min_width(ui.available_width().max(720.0));
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
                                        self.pending_delete_tool_id = None;
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
                        self.pending_delete_tool_id = None;
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
                                self.pending_delete_tool_id = None;
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

                    let has_selected_tool = !global_lib.tools.is_empty();
                    if ui
                        .add_enabled(
                            !is_readonly_builtin && has_selected_tool,
                            egui::Button::new("Duplicate Tool"),
                        )
                        .on_hover_text(
                            "Clone the selected cutter with a new unique tool number and ID",
                        )
                        .on_disabled_hover_text(if is_readonly_builtin {
                            "Clone the built-in machine first to duplicate cutters"
                        } else {
                            "Select a cutter first to duplicate it"
                        })
                        .clicked()
                    {
                        let sel_idx = self.selected_tool_idx.min(global_lib.tools.len() - 1);
                        let mut dup = global_lib.tools[sel_idx].clone();
                        let next_n = global_lib
                            .tools
                            .iter()
                            .map(|t| t.tool_number)
                            .max()
                            .unwrap_or(0)
                            + 1;
                        let mut copy_idx = 1usize;
                        let mut new_id = ToolId(format!("{}_copy_t{next_n}", dup.id.0));
                        while global_lib.tools.iter().any(|t| t.id == new_id) {
                            copy_idx += 1;
                            new_id = ToolId(format!("{}_copy{copy_idx}_t{next_n}", dup.id.0));
                        }
                        dup.id = new_id;
                        dup.tool_number = next_n;
                        dup.length_offset_number = next_n;
                        dup.diameter_offset_number = next_n;
                        dup.name = format!("{} (Copy)", dup.name);
                        let dup_name = dup.name.clone();
                        global_lib.tools.push(dup);
                        self.selected_tool_idx = global_lib.tools.len() - 1;
                        self.selected_preset_idx = 0;
                        self.scroll_to_selected = true;
                        self.pending_delete_tool_id = None;
                        actions.save_active_machine_tools = true;
                        self.status_banner = Some(format!(
                            "Duplicated tool as T{next_n} ('{dup_name}')."
                        ));
                    }

                    if ui
                        .add_enabled(
                            !is_readonly_builtin && has_selected_tool,
                            egui::Button::new("Delete Tool"),
                        )
                        .on_hover_text("Remove the selected cutter from this tool library")
                        .on_disabled_hover_text(if is_readonly_builtin {
                            "Clone the built-in machine first to delete cutters"
                        } else {
                            "No cutter selected to delete"
                        })
                        .clicked()
                    {
                        let sel_idx = self.selected_tool_idx.min(global_lib.tools.len() - 1);
                        let target_id = global_lib.tools[sel_idx].id.clone();
                        let refs = project
                            .tool_usage_map()
                            .remove(&target_id)
                            .unwrap_or_default();
                        if refs.is_empty() {
                            let removed = global_lib.tools.remove(sel_idx);
                            if !global_lib.tools.is_empty() {
                                self.selected_tool_idx = sel_idx.min(global_lib.tools.len() - 1);
                            } else {
                                self.selected_tool_idx = 0;
                            }
                            self.selected_preset_idx = 0;
                            self.pending_delete_tool_id = None;
                            actions.save_active_machine_tools = true;
                            self.status_banner = Some(format!(
                                "Deleted T{} ('{}') from library.",
                                removed.tool_number, removed.name
                            ));
                        } else {
                            self.pending_delete_tool_id = Some(target_id);
                        }
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
                    self.pending_delete_tool_id = None;
                    ui.weak("Library is empty. Click '+ New Cutter' or import a Fusion 360 library.");
                    return;
                }
                if self.selected_tool_idx >= global_lib.tools.len() {
                    self.selected_tool_idx = global_lib.tools.len() - 1;
                }

                // Active-operation usage warning & confirmation guard for in-use tool deletion.
                let selected_id = global_lib.tools[self.selected_tool_idx].id.clone();
                let active_refs: Vec<String> = {
                    let mut seen = std::collections::HashSet::new();
                    project
                        .tool_usage_map()
                        .remove(&selected_id)
                        .unwrap_or_default()
                        .into_iter()
                        .filter(|r| seen.insert((r.setup_idx, r.op_idx)))
                        .map(|r| format!("{} / {}", r.setup_name, r.op_name))
                        .collect()
                };
                if !active_refs.is_empty() {
                    let preview_refs = if active_refs.len() <= 3 {
                        active_refs.join(", ")
                    } else {
                        format!(
                            "{}, +{} more",
                            active_refs[..3].join(", "),
                            active_refs.len() - 3
                        )
                    };
                    let pending_this = self.pending_delete_tool_id.as_ref() == Some(&selected_id);
                    if pending_this && !is_readonly_builtin {
                        crate::widgets::inline_banner(
                            ui,
                            crate::widgets::BannerKind::Warn,
                            format!(
                                "In-use warning: referenced by {} active operation(s) ({preview_refs}). Removing deletes it from this machine library only (project-embedded copy remains intact).",
                                active_refs.len()
                            ),
                        );
                        ui.horizontal(|ui| {
                            if ui
                                .button("Confirm Delete Tool")
                                .on_hover_text("Remove this cutter from the library while keeping any project-embedded copy")
                                .clicked()
                            {
                                let removed = global_lib.tools.remove(self.selected_tool_idx);
                                if !global_lib.tools.is_empty() {
                                    self.selected_tool_idx =
                                        self.selected_tool_idx.min(global_lib.tools.len() - 1);
                                } else {
                                    self.selected_tool_idx = 0;
                                }
                                self.selected_preset_idx = 0;
                                self.pending_delete_tool_id = None;
                                actions.save_active_machine_tools = true;
                                self.status_banner = Some(format!(
                                    "Deleted in-use T{} ('{}') from library.",
                                    removed.tool_number, removed.name
                                ));
                            }
                            if ui
                                .button("Cancel")
                                .on_hover_text("Keep this tool in the library")
                                .clicked()
                            {
                                self.pending_delete_tool_id = None;
                            }
                        });
                        if global_lib.tools.is_empty() {
                            return;
                        }
                    } else {
                        crate::widgets::inline_banner(
                            ui,
                            crate::widgets::BannerKind::Warn,
                            format!(
                                "Referenced by {} active operation(s): {preview_refs}",
                                active_refs.len()
                            ),
                        );
                    }
                } else if self.pending_delete_tool_id.is_some() {
                    self.pending_delete_tool_id = None;
                }

                let body_w = ui.available_width().max(700.0);
                let body_h = ui.available_height().max(320.0);
                let left_w = (body_w * 0.36).clamp(276.0, 340.0);
                let gap_w = 10.0_f32;
                let right_w = (body_w - left_w - gap_w - 2.0).max(380.0);
                let list_h = (body_h - 210.0).clamp(150.0, 320.0);

                ui.horizontal_top(|ui| {
                    ui.spacing_mut().item_spacing.x = gap_w;

                    // Left column: cutter list + 2D profile schematic
                    ui.allocate_ui_with_layout(
                        egui::vec2(left_w, body_h),
                        egui::Layout::top_down(egui::Align::LEFT),
                        |ui| {
                            ui.set_max_width(left_w);
                            ui.strong(format!(
                                "{} ({} tools)",
                                global_lib.name,
                                global_lib.tools.len()
                            ));
                            egui::ScrollArea::vertical()
                                .id_salt("tool_lib_left_list")
                                .max_height(list_h)
                                .auto_shrink([false, false])
                                .show(ui, |ui| {
                                    ui.set_width((left_w - 14.0).max(200.0));
                                    let scroll_now = std::mem::take(&mut self.scroll_to_selected);
                                    for (idx, t) in global_lib.tools.iter().enumerate() {
                                        let sel = idx == self.selected_tool_idx;
                                        let resp = ui
                                            .selectable_label(sel, t.summary_label())
                                            .on_hover_text("Inspect or edit this cutting tool's geometry and material presets");
                                        if sel && scroll_now {
                                            resp.scroll_to_me(Some(egui::Align::Center));
                                        }
                                        if resp.clicked() {
                                            self.selected_tool_idx = idx;
                                            self.selected_preset_idx = 0;
                                            self.pending_delete_tool_id = None;
                                        }
                                    }
                                });

                            ui.separator();
                            let preview_tool = &global_lib.tools[self.selected_tool_idx];
                            ui.strong("2D Cutter Profile Preview");
                            draw_cutter_schematic(ui, preview_tool);

                            ui.horizontal(|ui| {
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
                        },
                    );

                    // Right column: Aligned two-column collapsible property cards
                    let tool = &mut global_lib.tools[self.selected_tool_idx];
                    ui.allocate_ui_with_layout(
                        egui::vec2(right_w, body_h),
                        egui::Layout::top_down(egui::Align::LEFT),
                        |ui| {
                            egui::ScrollArea::vertical()
                                .id_salt("tool_lib_right_editor")
                                .auto_shrink([false, false])
                                .show(ui, |ui| {
                                    ui.set_width((right_w - 14.0).max(340.0));

                                    // 1. Tool Identity (expanded by default)
                                    section_card(
                                        ui,
                                        "tool_lib_identity_card",
                                        "Tool Identity",
                                        true,
                                        |ui| {
                                            ui.add_enabled_ui(!is_readonly_builtin, |ui| {
                                                inspector_grid(ui, "tool_identity_grid", |ui| {
                                                    let name_tip = "Descriptive name for this cutting tool (shown in Toolpath Browser and G-code comments)";
                                                    grid_row(ui, "Name", Some(name_tip), |ui| {
                                                        ui.add(
                                                            egui::TextEdit::singleline(&mut tool.name)
                                                                .desired_width(TOOL_COMBO_W),
                                                        )
                                                        .on_hover_text(name_tip);
                                                    });

                                                    edit_offset_number_row(
                                                        ui,
                                                        TOOL_NUMBER_LABEL,
                                                        TOOL_NUMBER_TOOLTIP,
                                                        &mut tool.tool_number,
                                                    );

                                                    grid_row(
                                                        ui,
                                                        "Native Unit",
                                                        Some(NATIVE_UNIT_TOOLTIP),
                                                        |ui| {
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
                                                        },
                                                    );

                                                    let spindle_tip = "Spindle rotation direction emitted in G-code (M3 clockwise or M4 counter-clockwise)";
                                                    grid_row(
                                                        ui,
                                                        "Spindle Direction",
                                                        Some(spindle_tip),
                                                        |ui| {
                                                            ui.selectable_value(
                                                                &mut tool.spindle_direction,
                                                                SpindleDirection::Cw,
                                                                SpindleDirection::Cw.label(),
                                                            )
                                                            .on_hover_text(spindle_tip);
                                                            ui.selectable_value(
                                                                &mut tool.spindle_direction,
                                                                SpindleDirection::Ccw,
                                                                SpindleDirection::Ccw.label(),
                                                            )
                                                            .on_hover_text(spindle_tip);
                                                        },
                                                    );

                                                    let coolant_tip = "Default coolant command used when a preset or operation does not override coolant";
                                                    grid_row(
                                                        ui,
                                                        "Default Coolant",
                                                        Some(coolant_tip),
                                                        |ui| {
                                                            egui::ComboBox::from_id_salt(
                                                                "tool_default_coolant_cb",
                                                            )
                                                            .width(TOOL_COMBO_W)
                                                            .selected_text(
                                                                tool.default_coolant.label(),
                                                            )
                                                            .show_ui(ui, |ui| {
                                                                for c in CoolantMode::ALL {
                                                                    ui.selectable_value(
                                                                        &mut tool.default_coolant,
                                                                        c,
                                                                        c.label(),
                                                                    )
                                                                    .on_hover_text(coolant_tip);
                                                                }
                                                            })
                                                            .response
                                                            .on_hover_text(coolant_tip);
                                                        },
                                                    );
                                                });
                                            });
                                        },
                                    );

                                    let unit = tool.native_unit;

                                    // 2. Physical Geometry (expanded by default)
                                    section_card(
                                        ui,
                                        "tool_lib_geometry_card",
                                        "Physical Geometry",
                                        true,
                                        |ui| {
                                            let tok = crate::theme::ThemeTokens::from_ui(ui);
                                            ui.horizontal(|ui| match tool.validate() {
                                                Ok(()) => {
                                                    crate::widgets::status_badge_icon(
                                                        ui,
                                                        crate::widgets::StatusBadgeKind::Clean,
                                                        None,
                                                    );
                                                    ui.colored_label(
                                                        tok.clean_fg,
                                                        "Geometry Valid",
                                                    );
                                                }
                                                Err(err) => {
                                                    crate::widgets::status_badge_icon(
                                                        ui,
                                                        crate::widgets::StatusBadgeKind::Error,
                                                        None,
                                                    );
                                                    ui.colored_label(tok.error_fg, err.to_string());
                                                }
                                            });
                                            ui.add_space(4.0);
                                            ui.add_enabled_ui(!is_readonly_builtin, |ui| {
                                                show_cutter_geometry_editor(ui, tool, unit);
                                            });
                                        },
                                    );

                                    // 3. Material Cutting Presets (expanded by default)
                                    section_card(
                                        ui,
                                        "tool_lib_presets_card",
                                        "Material Cutting Presets",
                                        true,
                                        |ui| {
                                            ui.add_enabled_ui(!is_readonly_builtin, |ui| {
                                                if show_presets_editor(
                                                    ui,
                                                    tool,
                                                    unit,
                                                    &mut self.selected_preset_idx,
                                                    &mut self.lock_chipload,
                                                ) {
                                                    actions.save_active_machine_tools = true;
                                                }
                                            });
                                        },
                                    );

                                    // 4. Controller Offsets (H / D) (collapsed by default)
                                    section_card(
                                        ui,
                                        "tool_lib_offsets_card",
                                        "Controller Offsets (H / D)",
                                        false,
                                        |ui| {
                                            ui.add_enabled_ui(!is_readonly_builtin, |ui| {
                                                inspector_grid(ui, "tool_offsets_grid", |ui| {
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
                                                });
                                            });
                                        },
                                    );

                                    // 5. Vendor & Catalog Info (collapsed by default)
                                    section_card(
                                        ui,
                                        "tool_lib_vendor_card",
                                        "Vendor & Catalog Info",
                                        false,
                                        |ui| {
                                            ui.add_enabled_ui(!is_readonly_builtin, |ui| {
                                                inspector_grid(ui, "tool_vendor_grid", |ui| {
                                                    let vendor_tip = "Optional cutter manufacturer or supplier name (e.g., Harvey Tool, Datron, Lakeshore Carbide)";
                                                    grid_row(ui, "Vendor", Some(vendor_tip), |ui| {
                                                        let mut vendor_buf =
                                                            tool.vendor.clone().unwrap_or_default();
                                                        if ui
                                                            .add(
                                                                egui::TextEdit::singleline(
                                                                    &mut vendor_buf,
                                                                )
                                                                .desired_width(TOOL_COMBO_W)
                                                                .hint_text(
                                                                    "Manufacturer / supplier",
                                                                ),
                                                            )
                                                            .on_hover_text(vendor_tip)
                                                            .changed()
                                                        {
                                                            tool.vendor =
                                                                if vendor_buf.trim().is_empty() {
                                                                    None
                                                                } else {
                                                                    Some(vendor_buf)
                                                                };
                                                        }
                                                    });

                                                    let sku_tip = "Optional manufacturer catalog SKU or part number for reordering";
                                                    grid_row(ui, "Product ID", Some(sku_tip), |ui| {
                                                        let mut sku_buf = tool
                                                            .product_id
                                                            .clone()
                                                            .unwrap_or_default();
                                                        if ui
                                                            .add(
                                                                egui::TextEdit::singleline(
                                                                    &mut sku_buf,
                                                                )
                                                                .desired_width(TOOL_COMBO_W)
                                                                .hint_text("Catalog SKU / part #"),
                                                            )
                                                            .on_hover_text(sku_tip)
                                                            .changed()
                                                        {
                                                            tool.product_id =
                                                                if sku_buf.trim().is_empty() {
                                                                    None
                                                                } else {
                                                                    Some(sku_buf)
                                                                };
                                                        }
                                                    });
                                                });
                                            });
                                        },
                                    );
                                });
                        },
                    );
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
                egui::vec2(1280.0, 900.0),
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

        let initial_texts = collect_text_shapes(&settled.shapes);
        assert!(
            initial_texts.iter().any(|(t, _)| t == TOOL_NUMBER_LABEL),
            "Tool Number (T) must be visible by default in Tool Identity card"
        );
        assert!(
            !initial_texts.iter().any(|(t, _)| t == LENGTH_OFFSET_LABEL),
            "Length Offset (H) must be hidden by default while Controller Offsets (H / D) card is collapsed"
        );
        assert!(
            !initial_texts
                .iter()
                .any(|(t, _)| t == DIAMETER_OFFSET_LABEL),
            "Diameter Offset (D) must be hidden by default while Controller Offsets (H / D) card is collapsed"
        );

        // Collapse Physical Geometry & Material Cutting Presets so Controller Offsets (H / D)
        // is visible without scrolling, then expand Controller Offsets (H / D).
        let mut t = 1.0;
        for header_title in [
            "Physical Geometry",
            "Material Cutting Presets",
            "Controller Offsets (H / D)",
        ] {
            let cur_out = run_modal_frame(
                &ctx,
                egui::RawInput {
                    time: Some(t),
                    ..base_input.clone()
                },
                &mut modal,
                &mut global_lib,
                &mut project,
                &mut history,
                &mut logs,
            );
            t += 0.5;
            let cur_texts = collect_text_shapes(&cur_out.shapes);
            let hdr_pos = cur_texts
                .iter()
                .find(|(txt, _)| txt == header_title)
                .unwrap_or_else(|| {
                    panic!("expected section card header '{header_title}' to render")
                })
                .1
                .center();
            let _ = run_modal_frame(
                &ctx,
                egui::RawInput {
                    time: Some(t),
                    events: click_at(hdr_pos),
                    ..base_input.clone()
                },
                &mut modal,
                &mut global_lib,
                &mut project,
                &mut history,
                &mut logs,
            );
            t += 0.5;
        }

        let expanded = run_modal_frame(
            &ctx,
            egui::RawInput {
                time: Some(t),
                ..base_input.clone()
            },
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        t += 0.5;

        let texts = collect_text_shapes(&expanded.shapes);
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
                texts.iter().any(|(t, _)| t == "Neck Length (Reach)"),
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
            (lolli_idx, "Lollipop Endmill", "Sphere Join Height"),
            (slot_idx, "Slot / T-Slot Mill", "Cutting Thickness"),
            (dove_idx, "Dovetail Cutter", "Top Dia (Derived)"),
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

    fn click_at(pos: egui::Pos2) -> Vec<egui::Event> {
        vec![
            egui::Event::PointerMoved(pos),
            egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: egui::Modifiers::NONE,
            },
            egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::NONE,
            },
        ]
    }

    #[test]
    #[allow(clippy::unwrap_used, clippy::expect_used)]
    fn m87_tool_metadata_crud_and_presets_editor_regression() {
        use chip_cam::{Operation, OperationKind};
        use chip_machines::builtin_machine_definitions;

        let ctx = egui::Context::default();
        let mut global_lib = default_starter_tool_library();
        let initial_len = global_lib.tools.len();
        assert!(initial_len >= 2);
        // Ensure tool 0 starts with 1 preset and CW / non-Flood coolant for deterministic testing.
        global_lib.tools[0].spindle_direction = SpindleDirection::Cw;
        global_lib.tools[0].default_coolant = CoolantMode::AirBlast;
        global_lib.tools[0].vendor = None;
        global_lib.tools[0].product_id = None;
        global_lib.tools[0].native_unit = DisplayUnit::Millimeter;
        global_lib.tools[0].presets.truncate(1);
        global_lib.tools[0].presets[0].coolant_override = None;

        let mut modal = ToolLibraryModalState::new();
        modal.open = true;
        let mut project = ProjectDocument::new("m87_tl");
        let mut history = UndoHistory::new(8);
        let mut logs = Vec::new();
        let mut time = 1.0;

        let base_input = |events: Vec<egui::Event>, t: &mut f64| {
            *t += 0.25;
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(1280.0, 1400.0),
                )),
                time: Some(*t),
                events,
                ..Default::default()
            }
        };

        // 1. Settle layout and verify new metadata labels render.
        let _ = run_modal_frame(
            &ctx,
            base_input(Vec::new(), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        let settled = run_modal_frame(
            &ctx,
            base_input(Vec::new(), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        let texts = collect_text_shapes(&settled.shapes);
        for expected in [
            "Tool Identity",
            "Physical Geometry",
            "Spindle Direction",
            "CW (M3)",
            "CCW (M4)",
            "Default Coolant",
            "Duplicate Tool",
            "Delete Tool",
        ] {
            assert!(
                texts.iter().any(|(t, _)| t == expected),
                "expected Tool Library label/button '{expected}' to render"
            );
        }
        // Vendor & Product ID live inside the default-collapsed Vendor & Catalog Info card.
        assert!(
            !texts.iter().any(|(t, _)| t == "Vendor"),
            "Vendor field must be collapsed by default"
        );
        assert!(
            !texts.iter().any(|(t, _)| t == "Product ID"),
            "Product ID field must be collapsed by default"
        );

        // 2. Click CCW (M4) -> mutates spindle_direction.
        let ccw_pos = texts
            .iter()
            .find(|(t, _)| t == "CCW (M4)")
            .expect("CCW (M4) toggle must render")
            .1
            .center();
        let _ = run_modal_frame(
            &ctx,
            base_input(click_at(ccw_pos), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        assert_eq!(global_lib.tools[0].spindle_direction, SpindleDirection::Ccw);

        // 3. Edit Default Coolant via ComboBox -> Flood (M8).
        let cool_rect = texts
            .iter()
            .find(|(t, _)| t == "Air Blast")
            .expect("Default Coolant combo must show Air Blast")
            .1;
        let _ = run_modal_frame(
            &ctx,
            base_input(click_at(cool_rect.center()), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        let opened_cool = run_modal_frame(
            &ctx,
            base_input(Vec::new(), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        let flood_pos = collect_text_shapes(&opened_cool.shapes)
            .into_iter()
            .find(|(t, _)| t == "Flood (M8)")
            .expect("Flood (M8) option must render in coolant popup")
            .1
            .center();
        let _ = run_modal_frame(
            &ctx,
            base_input(click_at(flood_pos), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        assert_eq!(global_lib.tools[0].default_coolant, CoolantMode::Flood);

        // 4. Collapse Physical Geometry & Material Cutting Presets, then expand Vendor & Catalog Info
        //    and edit Vendor and Product ID text boxes.
        for header_title in [
            "Physical Geometry",
            "Material Cutting Presets",
            "Vendor & Catalog Info",
        ] {
            let cur_out = run_modal_frame(
                &ctx,
                base_input(Vec::new(), &mut time),
                &mut modal,
                &mut global_lib,
                &mut project,
                &mut history,
                &mut logs,
            );
            let hdr_pos = collect_text_shapes(&cur_out.shapes)
                .into_iter()
                .find(|(t, _)| t == header_title)
                .unwrap_or_else(|| {
                    panic!("expected section card header '{header_title}' to render")
                })
                .1
                .center();
            let _ = run_modal_frame(
                &ctx,
                base_input(click_at(hdr_pos), &mut time),
                &mut modal,
                &mut global_lib,
                &mut project,
                &mut history,
                &mut logs,
            );
        }

        let after_cool = run_modal_frame(
            &ctx,
            base_input(Vec::new(), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        let texts_ac = collect_text_shapes(&after_cool.shapes);
        assert!(texts_ac.iter().any(|(t, _)| t == "Vendor"));
        assert!(texts_ac.iter().any(|(t, _)| t == "Product ID"));
        let vendor_hint = texts_ac
            .iter()
            .find(|(t, _)| t == "Manufacturer / supplier")
            .expect("Vendor hint must render")
            .1
            .center();
        let _ = run_modal_frame(
            &ctx,
            base_input(click_at(vendor_hint), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        let _ = run_modal_frame(
            &ctx,
            base_input(
                vec![egui::Event::Text("Harvey Tool".to_string())],
                &mut time,
            ),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        assert_eq!(global_lib.tools[0].vendor.as_deref(), Some("Harvey Tool"));

        let sku_hint = texts_ac
            .iter()
            .find(|(t, _)| t == "Catalog SKU / part #")
            .expect("Product ID hint must render")
            .1
            .center();
        let _ = run_modal_frame(
            &ctx,
            base_input(click_at(sku_hint), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        let _ = run_modal_frame(
            &ctx,
            base_input(vec![egui::Event::Text("HT-98765".to_string())], &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        assert_eq!(global_lib.tools[0].product_id.as_deref(), Some("HT-98765"));

        // 5. Duplicate Tool adds a cloned copy with unique ID and incremented tool number.
        let dup_pos = texts_ac
            .iter()
            .find(|(t, _)| t == "Duplicate Tool")
            .expect("Duplicate Tool button must render")
            .1
            .center();
        let _ = run_modal_frame(
            &ctx,
            base_input(click_at(dup_pos), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        assert_eq!(global_lib.tools.len(), initial_len + 1);
        let dup_idx = global_lib.tools.len() - 1;
        assert_eq!(modal.selected_tool_idx, dup_idx);
        assert!(global_lib.tools[dup_idx].name.ends_with("(Copy)"));
        assert_ne!(global_lib.tools[dup_idx].id, global_lib.tools[0].id);

        // 6. Delete unused duplicated tool removes it immediately.
        let del_frame = run_modal_frame(
            &ctx,
            base_input(Vec::new(), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        let del_pos = collect_text_shapes(&del_frame.shapes)
            .into_iter()
            .find(|(t, _)| t == "Delete Tool")
            .expect("Delete Tool button must render")
            .1
            .center();
        let _ = run_modal_frame(
            &ctx,
            base_input(click_at(del_pos), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        assert_eq!(global_lib.tools.len(), initial_len);

        // 7. In-use tool shows usage warning and requires confirmation click.
        modal.selected_tool_idx = 0;
        let t0 = global_lib.tools[0].clone();
        project.ensure_tool_embedded(&t0);
        let mut op = Operation::new("op_in_use", "Rough Pocket", OperationKind::Pocket2D);
        op.tool_id = Some(t0.id.clone());
        project.setups[0].operations.push(op);

        let warn_frame = run_modal_frame(
            &ctx,
            base_input(Vec::new(), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        let warn_texts = collect_text_shapes(&warn_frame.shapes);
        assert!(
            warn_texts
                .iter()
                .any(|(t, _)| t.contains("Referenced by 1 active operation(s)")),
            "expected in-use warning banner for tool referenced by operation"
        );
        let del_in_use_pos = warn_texts
            .iter()
            .find(|(t, _)| t == "Delete Tool")
            .expect("Delete Tool button must render")
            .1
            .center();
        let confirm_frame = run_modal_frame(
            &ctx,
            base_input(click_at(del_in_use_pos), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        assert_eq!(
            global_lib.tools.len(),
            initial_len,
            "first click on Delete Tool for in-use cutter must guard deletion"
        );
        assert_eq!(modal.pending_delete_tool_id.as_ref(), Some(&t0.id));
        let confirm_pos = collect_text_shapes(&confirm_frame.shapes)
            .into_iter()
            .find(|(t, _)| t == "Confirm Delete Tool")
            .expect("Confirm Delete Tool button must render")
            .1
            .center();
        let _ = run_modal_frame(
            &ctx,
            base_input(click_at(confirm_pos), &mut time),
            &mut modal,
            &mut global_lib,
            &mut project,
            &mut history,
            &mut logs,
        );
        assert_eq!(global_lib.tools.len(), initial_len - 1);
        assert!(
            project.tool(&t0.id).is_some(),
            "project-embedded copy must remain intact after removing from machine library"
        );

        // 8. Built-in read-only machine disables Duplicate Tool & Delete Tool.
        let builtin_machines = builtin_machine_definitions();
        let builtin_id = builtin_machines[0].id.clone();
        let count_before_ro = global_lib.tools.len();
        let mut ro_out = ctx.run_ui(base_input(Vec::new(), &mut time), |ui| {
            let _ = modal.show_with_machine(
                ui.ctx(),
                &mut global_lib,
                &mut project,
                &mut history,
                &mut logs,
                Some(ToolLibraryMachineContext {
                    machines: &builtin_machines,
                    active_machine_id: &builtin_id,
                }),
            );
        });
        ro_out.textures_delta.clear();
        let ro_dup_pos = collect_text_shapes(&ro_out.shapes)
            .into_iter()
            .find(|(t, _)| t == "Duplicate Tool")
            .expect("Duplicate Tool button must render")
            .1
            .center();
        let mut ro_click = ctx.run_ui(base_input(click_at(ro_dup_pos), &mut time), |ui| {
            let _ = modal.show_with_machine(
                ui.ctx(),
                &mut global_lib,
                &mut project,
                &mut history,
                &mut logs,
                Some(ToolLibraryMachineContext {
                    machines: &builtin_machines,
                    active_machine_id: &builtin_id,
                }),
            );
        });
        ro_click.textures_delta.clear();
        assert_eq!(
            global_lib.tools.len(),
            count_before_ro,
            "Duplicate Tool must be disabled on read-only built-in machines"
        );

        // 9. Presets Editor: Material, Coolant Override, Lead/Ramp/Plunge feeds (in Inch), + Preset & - Delete Preset.
        let preset_ctx = egui::Context::default();
        let mut preset_tool = t0;
        preset_tool.presets.truncate(1);
        preset_tool.presets[0].material = "Aluminum 6061".to_string();
        preset_tool.presets[0].coolant_override = None;
        preset_tool.presets[0].lead_feed_mm_min = 600.0;
        preset_tool.presets[0].ramp_feed_mm_min = 400.0;
        preset_tool.presets[0].plunge_feed_mm_min = 250.0;
        let mut sel_preset = 0usize;
        let mut lock_cl = true;

        let run_presets = |events: Vec<egui::Event>,
                           tool: &mut Tool,
                           sel: &mut usize,
                           lock: &mut bool,
                           t: &mut f64| {
            let mut out = preset_ctx.run_ui(base_input(events, t), |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    show_presets_editor(ui, tool, DisplayUnit::Inch, sel, lock);
                });
            });
            out.textures_delta.clear();
            collect_text_shapes(&out.shapes)
        };

        let p_texts = run_presets(
            Vec::new(),
            &mut preset_tool,
            &mut sel_preset,
            &mut lock_cl,
            &mut time,
        );
        for expected in [
            "Material",
            "Coolant Override",
            "Lead Feed",
            "Ramp Feed",
            "Plunge Feed",
            "- Delete Preset",
        ] {
            assert!(
                p_texts.iter().any(|(t, _)| t == expected),
                "expected preset editor control '{expected}' to render"
            );
        }
        // Delete Preset disabled when only 1 preset exists.
        let del_p_pos = p_texts
            .iter()
            .find(|(t, _)| t == "- Delete Preset")
            .expect("- Delete Preset button must render")
            .1
            .center();
        let _ = run_presets(
            click_at(del_p_pos),
            &mut preset_tool,
            &mut sel_preset,
            &mut lock_cl,
            &mut time,
        );
        assert_eq!(
            preset_tool.presets.len(),
            1,
            "Delete Preset must be guarded when only 1 preset exists"
        );

        let drag_preset_widget = |pos: egui::Pos2,
                                  dx: f32,
                                  tool: &mut Tool,
                                  sel: &mut usize,
                                  lock: &mut bool,
                                  t: &mut f64| {
            let end = egui::pos2(pos.x + dx, pos.y);
            let _ = run_presets(
                vec![
                    egui::Event::PointerMoved(pos),
                    egui::Event::PointerButton {
                        pos,
                        button: egui::PointerButton::Primary,
                        pressed: true,
                        modifiers: egui::Modifiers::NONE,
                    },
                ],
                tool,
                sel,
                lock,
                t,
            );
            let _ = run_presets(vec![egui::Event::PointerMoved(end)], tool, sel, lock, t);
            let _ = run_presets(
                vec![egui::Event::PointerButton {
                    pos: end,
                    button: egui::PointerButton::Primary,
                    pressed: false,
                    modifiers: egui::Modifiers::NONE,
                }],
                tool,
                sel,
                lock,
                t,
            );
        };

        // Edit Lead/Ramp/Plunge feed DragValues in Inch mode (ipm) -> verify canonical mm/min mutation.
        let lead_orig = preset_tool.presets[0].lead_feed_mm_min;
        let ramp_orig = preset_tool.presets[0].ramp_feed_mm_min;
        let plunge_orig = preset_tool.presets[0].plunge_feed_mm_min;
        for lbl in ["Lead Feed", "Ramp Feed", "Plunge Feed"] {
            let p_now = run_presets(
                Vec::new(),
                &mut preset_tool,
                &mut sel_preset,
                &mut lock_cl,
                &mut time,
            );
            let lbl_rect = p_now
                .iter()
                .find(|(t, _)| t == lbl)
                .expect("preset feed label must render")
                .1;
            let drag_rect = p_now
                .iter()
                .find(|(t, r)| {
                    t.ends_with(" ipm") && (r.center().y - lbl_rect.center().y).abs() < 12.0
                })
                .expect("unit-aware ipm DragValue must render next to feed label")
                .1;
            drag_preset_widget(
                drag_rect.center(),
                40.0,
                &mut preset_tool,
                &mut sel_preset,
                &mut lock_cl,
                &mut time,
            );
        }
        assert_ne!(preset_tool.presets[0].lead_feed_mm_min, lead_orig);
        assert_ne!(preset_tool.presets[0].ramp_feed_mm_min, ramp_orig);
        assert_ne!(preset_tool.presets[0].plunge_feed_mm_min, plunge_orig);

        // Edit preset Material text and Coolant Override ComboBox.
        let p_mat = run_presets(
            Vec::new(),
            &mut preset_tool,
            &mut sel_preset,
            &mut lock_cl,
            &mut time,
        );
        let mat_val_pos = p_mat
            .iter()
            .find(|(t, _)| t == "Aluminum 6061")
            .expect("Material text edit must render")
            .1
            .center();
        let _ = run_presets(
            click_at(mat_val_pos),
            &mut preset_tool,
            &mut sel_preset,
            &mut lock_cl,
            &mut time,
        );
        let _ = run_presets(
            vec![egui::Event::Text(" T6".to_string())],
            &mut preset_tool,
            &mut sel_preset,
            &mut lock_cl,
            &mut time,
        );
        assert!(preset_tool.presets[0].material.contains("T6"));

        let cool_ovr_pos = p_mat
            .iter()
            .find(|(t, _)| t.starts_with("Tool Default"))
            .expect("Coolant Override combo must render")
            .1
            .center();
        let _ = run_presets(
            click_at(cool_ovr_pos),
            &mut preset_tool,
            &mut sel_preset,
            &mut lock_cl,
            &mut time,
        );
        let opened_ovr = run_presets(
            Vec::new(),
            &mut preset_tool,
            &mut sel_preset,
            &mut lock_cl,
            &mut time,
        );
        let mist_pos = opened_ovr
            .iter()
            .find(|(t, _)| t == "Mist (M7)")
            .expect("Mist (M7) option must render")
            .1
            .center();
        let _ = run_presets(
            click_at(mist_pos),
            &mut preset_tool,
            &mut sel_preset,
            &mut lock_cl,
            &mut time,
        );
        assert_eq!(
            preset_tool.presets[0].coolant_override,
            Some(CoolantMode::Mist)
        );

        // Add a second preset, then delete it cleanly.
        let add_p_pos = p_mat
            .iter()
            .find(|(t, _)| t == "+ Preset")
            .expect("+ Preset button must render")
            .1
            .center();
        let after_add = run_presets(
            click_at(add_p_pos),
            &mut preset_tool,
            &mut sel_preset,
            &mut lock_cl,
            &mut time,
        );
        assert_eq!(preset_tool.presets.len(), 2);
        assert_eq!(sel_preset, 1);

        let del_p2_pos = after_add
            .iter()
            .find(|(t, _)| t == "- Delete Preset")
            .expect("- Delete Preset button must render")
            .1
            .center();
        let _ = run_presets(
            click_at(del_p2_pos),
            &mut preset_tool,
            &mut sel_preset,
            &mut lock_cl,
            &mut time,
        );
        assert_eq!(preset_tool.presets.len(), 1);
        assert_eq!(sel_preset, 0);
    }

    #[test]
    #[allow(clippy::unwrap_used, clippy::expect_used)]
    fn tool_library_window_opens_full_width_and_stays_stable_for_30_frames() {
        let ctx = egui::Context::default();
        let mut modal = ToolLibraryModalState::new();
        modal.open = true;
        let mut global_lib = default_starter_tool_library();
        let mut project = ProjectDocument::new("tl_size_test");
        let mut history = UndoHistory::new(8);
        let mut logs = Vec::new();
        let win_id = egui::Id::unique("chipcutter_tool_library_modal");

        let mut rects = Vec::new();
        for frame in 0..30 {
            let input = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(1400.0, 900.0),
                )),
                time: Some(frame as f64 * 0.016),
                ..Default::default()
            };
            let _ = run_modal_frame(
                &ctx,
                input,
                &mut modal,
                &mut global_lib,
                &mut project,
                &mut history,
                &mut logs,
            );
            let rect = ctx
                .memory(|m| m.area_rect(win_id))
                .expect("Tool Library window rect must exist");
            rects.push(rect);
        }

        assert!(
            rects[0].width() >= 860.0,
            "Frame 1 Tool Library window should open at full width (>= 860 px), got {:.1} px",
            rects[0].width()
        );
        let base_w = rects[1].width();
        let base_h = rects[1].height();
        for (idx, r) in rects.iter().enumerate().skip(2) {
            assert!(
                (r.width() - base_w).abs() < 1.0 && (r.height() - base_h).abs() < 1.0,
                "Tool Library window crept on frame {}: ({:.2}, {:.2}) vs frame 2 ({:.2}, {:.2})",
                idx + 1,
                r.width(),
                r.height(),
                base_w,
                base_h
            );
        }
    }
}
