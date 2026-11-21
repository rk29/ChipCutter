use chip_math::DisplayUnit;
use chip_project::ProjectDocument;
use chip_tools::{
    CoolantMode, Tool, ToolLibrary, chipload_from_feed, compute_radial_chip_thinning,
    feed_from_chipload, rpm_from_surface_speed, surface_speed_from_rpm,
};

use crate::{
    theme::ThemeTokens,
    widgets::{BannerKind, INPUT_COL_WIDTH, grid_row, inline_banner, inspector_grid, section_card},
};

const COMBO_COL_WIDTH: f32 = INPUT_COL_WIDTH + 54.0;

pub struct ToolTabCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub setup_idx: usize,
    pub op_idx: usize,
    pub global_tools: &'a ToolLibrary,
    pub display_unit: DisplayUnit,
    pub lock_chipload: &'a mut bool,
    pub open_tool_manager: &'a mut bool,
    pub mutated_label: &'a mut Option<&'static str>,
}

pub fn show_tool_tab(ui: &mut egui::Ui, ctx: ToolTabCtx<'_>) {
    let ToolTabCtx {
        project,
        setup_idx,
        op_idx,
        global_tools,
        display_unit,
        lock_chipload,
        open_tool_manager,
        mutated_label,
    } = ctx;

    let op_kind = project.setups[setup_idx].operations[op_idx].kind;
    let tok = ThemeTokens::from_ui(ui);

    section_card(
        ui,
        ("tool_select_card", setup_idx, op_idx),
        "Cutter & Material Preset",
        true,
        |ui| {
            let cur_tid = project.setups[setup_idx].operations[op_idx].tool_id.clone();
            let mut selected_candidate: Option<Tool> = None;

            inspector_grid(ui, ("tool_select_grid", setup_idx, op_idx), |ui| {
                grid_row(
                    ui,
                    "Active Cutter",
                    Some("Select cutter from project-embedded or global tool library"),
                    |ui| {
                        let mut available_refs: Vec<&Tool> = project.tools.iter().collect();
                        for gt in &global_tools.tools {
                            if !project.tools.iter().any(|pt| pt.id == gt.id) {
                                available_refs.push(gt);
                            }
                        }

                        let selected_label = cur_tid
                            .as_ref()
                            .and_then(|id| available_refs.iter().find(|t| &t.id == id))
                            .map(|t| t.summary_label())
                            .unwrap_or_else(|| "Select a Tool…".to_string());

                        egui::ComboBox::from_id_salt(("op_tool_cb", setup_idx, op_idx))
                            .width(COMBO_COL_WIDTH)
                            .truncate()
                            .selected_text(selected_label)
                            .show_ui(ui, |ui| {
                                for &candidate in &available_refs {
                                    let compat = op_kind.is_compatible_with_tool(candidate);
                                    let prefix = if compat { "" } else { "[Incompat] " };
                                    let lbl = format!("{prefix}{}", candidate.summary_label());
                                    let is_sel = cur_tid.as_ref() == Some(&candidate.id);
                                    if ui
                                        .selectable_label(is_sel, lbl)
                                        .on_hover_text(if compat {
                                            "Assign this cutting tool to the operation"
                                        } else {
                                            "Warning: this cutter geometry is incompatible with the selected strategy"
                                        })
                                        .clicked()
                                        && !is_sel
                                    {
                                        selected_candidate = Some(candidate.clone());
                                    }
                                }
                            })
                            .response
                            .on_hover_text("Select cutting tool assigned to this CAM operation");
                        if ui
                            .small_button("Library…")
                            .on_hover_text("Open Tool Library Manager")
                            .clicked()
                        {
                            *open_tool_manager = true;
                        }
                    },
                );

                if let Some(chosen) = selected_candidate.take() {
                    project.ensure_tool_embedded(&chosen);
                    let op = &mut project.setups[setup_idx].operations[op_idx];
                    op.tool_id = Some(chosen.id.clone());
                    op.preset_id = chosen.presets.first().map(|p| p.id.clone());
                    op.feed_overrides.clear_all();
                    *mutated_label = Some("Assign Tool to Operation");
                }

                let active_tool_opt = project.setups[setup_idx].operations[op_idx]
                    .tool_id
                    .as_ref()
                    .and_then(|id| project.tool(id).cloned());

                if let Some(ref active_tool) = active_tool_opt {
                    if !active_tool.presets.is_empty() {
                        let cur_pid = project.setups[setup_idx].operations[op_idx]
                            .preset_id
                            .clone()
                            .unwrap_or_else(|| active_tool.presets[0].id.clone());
                        let cur_pname = active_tool
                            .presets
                            .iter()
                            .find(|p| p.id == cur_pid)
                            .map_or("Default", |p| p.name.as_str());

                        grid_row(
                            ui,
                            "Material Preset",
                            Some("Load manufacturer/material SFM, chipload, and stepover baseline"),
                            |ui| {
                                egui::ComboBox::from_id_salt(("op_preset_cb", setup_idx, op_idx))
                                    .width(COMBO_COL_WIDTH)
                                    .truncate()
                                    .selected_text(cur_pname)
                                    .show_ui(ui, |ui| {
                                        for p in &active_tool.presets {
                                            let is_p = cur_pid == p.id;
                                            if ui
                                                .selectable_label(is_p, &p.name)
                                                .on_hover_text("Load feeds, speeds, and stepover defaults from this material preset")
                                                .clicked()
                                                && !is_p
                                            {
                                                let op = &mut project.setups[setup_idx].operations
                                                    [op_idx];
                                                op.preset_id = Some(p.id.clone());
                                                op.feed_overrides.clear_all();
                                                *mutated_label = Some("Change Material Preset");
                                            }
                                        }
                                    })
                                    .response
                                    .on_hover_text("Choose material cutting preset for RPM, feedrate, and coolant defaults");
                            },
                        );
                    }

                    grid_row(
                        ui,
                        "RPM Coupling",
                        Some(
                            "Keep chipload per tooth constant when editing Spindle RPM or Surface Speed",
                        ),
                        |ui| {
                            ui.checkbox(lock_chipload, "Lock Chipload")
                                .on_hover_text("Keep chipload per tooth constant by scaling feedrate automatically when RPM or Surface Speed changes");
                            let op = &mut project.setups[setup_idx].operations[op_idx];
                            if !op.feed_overrides.is_all_inherited()
                                && ui
                                    .small_button("Reset")
                                    .on_hover_text(
                                        "Revert all locally overridden feeds/speeds to the material preset",
                                    )
                                    .clicked()
                            {
                                op.feed_overrides.clear_all();
                                *mutated_label = Some("Reset Feed/Speed Overrides");
                            }
                        },
                    );
                }
            });

            let active_check = project.setups[setup_idx].operations[op_idx]
                .tool_id
                .as_ref()
                .and_then(|id| project.tool(id));
            match active_check {
                None => {
                    ui.add_space(4.0);
                    inline_banner(
                        ui,
                        BannerKind::Info,
                        "Assign a tool above to configure cutting presets and coupled SFM/chipload.",
                    );
                }
                Some(t) if !op_kind.is_compatible_with_tool(t) => {
                    ui.add_space(4.0);
                    inline_banner(
                        ui,
                        BannerKind::Error,
                        format!(
                            "'{}' is not compatible with {}!",
                            t.geometry.kind_label(),
                            op_kind.label()
                        ),
                    );
                }
                Some(_) => {}
            }
        },
    );

    let Some(active_tool) = project.setups[setup_idx].operations[op_idx]
        .tool_id
        .as_ref()
        .and_then(|id| project.tool(id).cloned())
    else {
        return;
    };

    let op = &mut project.setups[setup_idx].operations[op_idx];
    let eff = op.effective_cutting_params(&active_tool);
    let dia_mm = active_tool.geometry.cutting_diameter_mm();
    let flutes = active_tool.geometry.flutes();

    section_card(
        ui,
        ("tool_spindle_card", setup_idx, op_idx),
        "Coupled Spindle & Surface Speed",
        true,
        |ui| {
            inspector_grid(ui, ("tool_spindle_grid", setup_idx, op_idx), |ui| {
                let mut rpm_val = eff.spindle_rpm;
                grid_row(
                    ui,
                    "Spindle Speed",
                    Some("Rotational spindle speed (coupled to Surface Speed via cutter diameter)"),
                    |ui| {
                        let r = ui
                            .add(
                                egui::DragValue::new(&mut rpm_val)
                                    .speed(25.0)
                                    .range(100.0..=60_000.0)
                                    .suffix(" RPM"),
                            )
                            .on_hover_text("Spindle rotation speed (RPM); coupled to Surface Speed and Chipload");
                        if op.feed_overrides.spindle_rpm.is_some() {
                            ui.colored_label(tok.dirty_fg, "*")
                                .on_hover_text("Overridden from material preset");
                        }
                        if r.changed() {
                            op.feed_overrides.spindle_rpm = Some(rpm_val);
                            if let (true, Some(fz)) = (
                                *lock_chipload,
                                chipload_from_feed(
                                    eff.spindle_rpm,
                                    flutes,
                                    eff.cutting_feed_mm_min,
                                ),
                            ) {
                                op.feed_overrides.cutting_feed_mm_min =
                                    Some(feed_from_chipload(rpm_val, flutes, fz));
                            }
                            *mutated_label = Some("Adjust Spindle RPM");
                        }
                    },
                );

                let cur_vc_m_min = surface_speed_from_rpm(rpm_val, dia_mm).unwrap_or(0.0);
                let mut vc_disp = display_unit.surface_speed_from_m_min(cur_vc_m_min);
                grid_row(
                    ui,
                    "Surface Speed",
                    Some("Peripheral cutting speed Vc = π · D · RPM"),
                    |ui| {
                        let r = ui
                            .add(
                                egui::DragValue::new(&mut vc_disp)
                                    .speed(1.0)
                                    .range(1.0..=5_000.0)
                                    .suffix(format!(" {}", display_unit.surface_speed_suffix())),
                            )
                            .on_hover_text("Peripheral cutter surface speed (m/min or SFM); automatically updates Spindle RPM");
                        if r.changed() {
                            let new_vc_m = display_unit.surface_speed_to_m_min(vc_disp);
                            if let Some(new_rpm) = rpm_from_surface_speed(new_vc_m, dia_mm) {
                                op.feed_overrides.spindle_rpm = Some(new_rpm);
                                if let (true, Some(fz)) = (
                                    *lock_chipload,
                                    chipload_from_feed(
                                        eff.spindle_rpm,
                                        flutes,
                                        eff.cutting_feed_mm_min,
                                    ),
                                ) {
                                    op.feed_overrides.cutting_feed_mm_min =
                                        Some(feed_from_chipload(new_rpm, flutes, fz));
                                }
                                *mutated_label = Some("Adjust Surface Speed");
                            }
                        }
                    },
                );
            });
        },
    );

    let eff2 = op.effective_cutting_params(&active_tool);
    let cur_fz_mm =
        chipload_from_feed(eff2.spindle_rpm, flutes, eff2.cutting_feed_mm_min).unwrap_or(0.0);

    section_card(
        ui,
        ("tool_feed_card", setup_idx, op_idx),
        "Feedrate, Stepover, & Coolant",
        true,
        |ui| {
            inspector_grid(ui, ("tool_feed_grid", setup_idx, op_idx), |ui| {
                let mut vf_disp = display_unit.feed_from_mm_min(eff2.cutting_feed_mm_min);
                grid_row(
                    ui,
                    "Cutting Feed",
                    Some("Linear XY cutting feedrate Vf = RPM · flutes · fz"),
                    |ui| {
                        let r = ui
                            .add(
                                egui::DragValue::new(&mut vf_disp)
                                    .speed(5.0)
                                    .range(1.0..=25_000.0)
                                    .suffix(format!(" {}", display_unit.feed_suffix())),
                            )
                            .on_hover_text(
                                "Linear XY cutting feedrate; automatically updates Chipload",
                            );
                        if op.feed_overrides.cutting_feed_mm_min.is_some() {
                            ui.colored_label(tok.dirty_fg, "*")
                                .on_hover_text("Overridden from material preset");
                        }
                        if r.changed() {
                            op.feed_overrides.cutting_feed_mm_min =
                                Some(display_unit.feed_to_mm_min(vf_disp));
                            *mutated_label = Some("Adjust Cutting Feedrate");
                        }
                    },
                );

                let mut fz_disp = display_unit.chipload_from_mm(cur_fz_mm);
                let chipload_lbl = format!("Chipload ({flutes} fl)");
                grid_row(
                    ui,
                    &chipload_lbl,
                    Some("Uncut chip thickness per flute tooth (fz)"),
                    |ui| {
                        let r = ui
                            .add(
                                egui::DragValue::new(&mut fz_disp)
                                    .speed(0.001)
                                    .max_decimals(5)
                                    .range(0.0001..=5.0)
                                    .suffix(format!(" {}", display_unit.chipload_suffix())),
                            )
                            .on_hover_text("Uncut chip thickness per flute tooth; automatically updates Cutting Feed");
                        if r.changed() {
                            let new_fz_mm = display_unit.chipload_to_mm(fz_disp);
                            op.feed_overrides.cutting_feed_mm_min =
                                Some(feed_from_chipload(eff2.spindle_rpm, flutes, new_fz_mm));
                            *mutated_label = Some("Adjust Chipload");
                        }
                    },
                );

                let mut ae_disp = display_unit.from_mm(eff2.stepover_mm);
                grid_row(
                    ui,
                    "Radial Stepover",
                    Some(
                        "Preset radial width of cut (ae) used for radial chip-thinning diagnostics",
                    ),
                    |ui| {
                        if ui
                            .add(
                                egui::DragValue::new(&mut ae_disp)
                                    .speed(0.05)
                                    .range(0.01..=500.0)
                                    .suffix(format!(" {}", display_unit.suffix())),
                            )
                            .changed()
                        {
                            op.feed_overrides.stepover_mm = Some(display_unit.to_mm(ae_disp));
                            *mutated_label = Some("Adjust Stepover");
                        }
                        if op.feed_overrides.stepover_mm.is_some() {
                            ui.colored_label(tok.dirty_fg, "*")
                                .on_hover_text("Overridden from material preset");
                        }
                    },
                );

                let mut cool = eff2.coolant;
                grid_row(
                    ui,
                    "Coolant",
                    Some("Post-processor coolant command (M8 flood, M7 mist, air blast, or off)"),
                    |ui| {
                        egui::ComboBox::from_id_salt(("op_cool_cb", setup_idx, op_idx))
                            .width(COMBO_COL_WIDTH)
                            .selected_text(cool.label())
                            .show_ui(ui, |ui| {
                                for c in CoolantMode::ALL {
                                    if ui.selectable_value(&mut cool, c, c.label()).changed() {
                                        op.feed_overrides.coolant = Some(cool);
                                        *mutated_label = Some("Change Operation Coolant");
                                    }
                                }
                            });
                        if op.feed_overrides.coolant.is_some() {
                            ui.colored_label(tok.dirty_fg, "*")
                                .on_hover_text("Overridden from material preset");
                        }
                    },
                );
            });

            if let Some(rct) = compute_radial_chip_thinning(
                dia_mm,
                eff2.stepover_mm,
                cur_fz_mm,
                eff2.cutting_feed_mm_min,
            ) {
                ui.add_space(4.0);
                inline_banner(
                    ui,
                    BannerKind::Info,
                    format!(
                        "RCT ({:.0}% engagement): actual hex = {:.4} {} — RCT feed hint: {:.0} {}",
                        (eff2.stepover_mm / dia_mm) * 100.0,
                        display_unit.chipload_from_mm(rct.actual_hex_mm),
                        display_unit.chipload_suffix(),
                        display_unit.feed_from_mm_min(rct.compensated_feed_mm_min),
                        display_unit.feed_suffix(),
                    ),
                );
            }
        },
    );
}
