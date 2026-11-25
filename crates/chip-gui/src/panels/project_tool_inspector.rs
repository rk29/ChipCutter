//! Dedicated right-panel **Project Tool Inspector** (`InspectorFocus::ProjectTool`) displaying a
//! cutter's 2D schematic, key dimensions & offsets, active referencing operations, and library sync
//! / removal actions.

use chip_project::ProjectDocument;
use chip_tools::{ToolId, ToolLibrary};

use crate::{
    panels::tool_library::draw_cutter_schematic,
    theme::ThemeTokens,
    widgets::{BannerKind, grid_row, inline_banner, inspector_grid, section_card, static_card},
};

/// Borrowed context passed to [`show_project_tool_inspector`].
pub struct ProjectToolInspectorCtx<'a> {
    pub project: &'a ProjectDocument,
    pub global_tools: &'a ToolLibrary,
    pub tool_idx: usize,
}

/// High-level actions emitted by [`show_project_tool_inspector`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProjectToolInspectorAction {
    EditInToolLibrary(ToolId),
    SyncFromMachineLibrary(ToolId),
    RemoveFromProject(ToolId),
    FocusOperation { setup_idx: usize, op_idx: usize },
}

/// Renders the dedicated right-panel **Project Tool Inspector**.
pub fn show_project_tool_inspector(
    ui: &mut egui::Ui,
    ctx: ProjectToolInspectorCtx<'_>,
) -> Option<ProjectToolInspectorAction> {
    let ProjectToolInspectorCtx {
        project,
        global_tools,
        tool_idx,
    } = ctx;

    let Some(tool) = project.tools.get(tool_idx) else {
        ui.weak("No Project Tool selected.");
        return None;
    };

    let tok = ThemeTokens::from_ui(ui);
    let u = project.display_units;
    let usage_map = project.tool_usage_map();
    let usages = usage_map.get(&tool.id).map_or(&[][..], Vec::as_slice);
    let in_machine_lib = global_tools.tool(&tool.id).is_some();
    let can_remove = usages.is_empty();
    let mut action: Option<ProjectToolInspectorAction> = None;

    static_card(ui, None, |ui| {
        ui.strong(format!("T{} - {}", tool.tool_number, tool.name));
        ui.colored_label(
            tok.text_muted,
            format!(
                "{} | {} preset(s) | {}",
                tool.geometry.kind_label(),
                tool.presets.len(),
                if usages.is_empty() {
                    "Unused".to_string()
                } else if usages.len() == 1 {
                    "Used by 1 operation".to_string()
                } else {
                    format!("Used by {} operations", usages.len())
                }
            ),
        );
        ui.add_space(5.0);
        ui.horizontal_wrapped(|ui| {
            if ui
                .small_button("Edit in Tool Library...")
                .on_hover_text(
                    "Open the Tool Library Manager focused on this cutter to edit geometry and cutting presets",
                )
                .clicked()
            {
                action = Some(ProjectToolInspectorAction::EditInToolLibrary(
                    tool.id.clone(),
                ));
            }
            if ui
                .add_enabled(
                    in_machine_lib,
                    egui::Button::new("Sync from Machine Library").small(),
                )
                .on_hover_text(
                    "Overwrite this embedded project tool snapshot with the active machine's latest definition and mark referencing ops Dirty",
                )
                .on_disabled_hover_text(
                    "No tool with a matching ID exists in the active machine's tool library",
                )
                .clicked()
            {
                action = Some(ProjectToolInspectorAction::SyncFromMachineLibrary(
                    tool.id.clone(),
                ));
            }
            if ui
                .add_enabled(
                    can_remove,
                    egui::Button::new("Remove from Project").small(),
                )
                .on_hover_text("Remove this unused embedded cutter snapshot from the project")
                .on_disabled_hover_text(
                    "Tool is referenced by active operations; reassign those operations before removing",
                )
                .clicked()
            {
                action = Some(ProjectToolInspectorAction::RemoveFromProject(
                    tool.id.clone(),
                ));
            }
        });
    });

    egui::ScrollArea::vertical()
        .id_salt(("project_tool_inspector_scroll", tool_idx))
        .auto_shrink([false, false])
        .show(ui, |ui| {
            // 1. 2D Cutter Schematic Preview
            section_card(
                ui,
                ("project_tool_schematic_card", tool_idx),
                "Cutter Profile",
                true,
                |ui| {
                    ui.vertical_centered(|ui| {
                        draw_cutter_schematic(ui, tool);
                    });
                },
            );

            // 2. Dimensions & Offsets Summary
            section_card(
                ui,
                ("project_tool_dims_card", tool_idx),
                "Dimensions & Offsets",
                true,
                |ui| {
                    let g = &tool.geometry;
                    inspector_grid(ui, ("project_tool_dims_grid", tool_idx), |ui| {
                        info_row(
                            ui,
                            &tok,
                            "Cutter Type",
                            g.kind_label(),
                            "Cutting tool geometry category",
                        );
                        info_row(
                            ui,
                            &tok,
                            "Cutting Diameter",
                            &u.format_length(g.cutting_diameter_mm()),
                            "Nominal cutting diameter",
                        );
                        info_row(
                            ui,
                            &tok,
                            "Flute Length",
                            &u.format_length(g.flute_length_mm()),
                            "Effective axial cutting flute length (LOC)",
                        );
                        info_row(
                            ui,
                            &tok,
                            "Overall Length",
                            &u.format_length(g.overall_length_mm()),
                            "Total physical tool length (OAL)",
                        );
                        info_row(
                            ui,
                            &tok,
                            "Stickout Length",
                            &u.format_length(tool.stickout_length_mm),
                            "Protrusion length below collet / holder face",
                        );
                        info_row(
                            ui,
                            &tok,
                            "Shank Diameter",
                            &u.format_length(g.shank_diameter_mm()),
                            "Collet / holder clamping shank diameter",
                        );
                        info_row(
                            ui,
                            &tok,
                            "Flutes",
                            &g.flutes().to_string(),
                            "Number of cutting flutes",
                        );
                        info_row(
                            ui,
                            &tok,
                            "Offsets (T / H / D)",
                            &format!(
                                "T{} / H{} / D{}",
                                tool.tool_number,
                                tool.length_offset_number,
                                tool.diameter_offset_number
                            ),
                            "Tool number (T), length offset register (H), and diameter wear register (D)",
                        );
                        info_row(
                            ui,
                            &tok,
                            "Spindle / Coolant",
                            &format!(
                                "{} / {}",
                                tool.spindle_direction.label(),
                                tool.default_coolant.label()
                            ),
                            "Default spindle rotation direction and coolant mode",
                        );
                        if let Some(vendor) = tool.vendor.as_deref().filter(|s| !s.trim().is_empty())
                        {
                            info_row(ui, &tok, "Vendor", vendor, "Tool manufacturer / supplier");
                        }
                        if let Some(pid) =
                            tool.product_id.as_deref().filter(|s| !s.trim().is_empty())
                        {
                            info_row(
                                ui,
                                &tok,
                                "Product ID",
                                pid,
                                "Manufacturer part / catalog number",
                            );
                        }
                    });
                },
            );

            // 3. Referencing Operations List
            section_card(
                ui,
                ("project_tool_usage_card", tool_idx),
                &format!("Referencing Operations ({})", usages.len()),
                true,
                |ui| {
                    if usages.is_empty() {
                        inline_banner(
                            ui,
                            BannerKind::Info,
                            "Not referenced by any active operation. You can safely remove this tool from the project.",
                        );
                    } else {
                        for uref in usages {
                            let role = if uref.is_clearing_tool {
                                " [Clearing]"
                            } else {
                                ""
                            };
                            let lbl = format!("{} / {}{}", uref.setup_name, uref.op_name, role);
                            if ui
                                .selectable_label(false, lbl)
                                .on_hover_text("Click to inspect this operation")
                                .clicked()
                            {
                                action = Some(ProjectToolInspectorAction::FocusOperation {
                                    setup_idx: uref.setup_idx,
                                    op_idx: uref.op_idx,
                                });
                            }
                        }
                    }
                },
            );
        });

    action
}

fn info_row(ui: &mut egui::Ui, tok: &ThemeTokens, label: &str, value: &str, tip: &str) {
    grid_row(ui, label, Some(tip), |ui| {
        ui.add(egui::Label::new(egui::RichText::new(value).color(tok.text_primary)).truncate())
            .on_hover_text(tip);
    });
}
