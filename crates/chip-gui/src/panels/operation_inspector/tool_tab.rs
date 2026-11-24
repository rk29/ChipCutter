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
                    Some("Select a compatible cutter from Project Tools or Active Machine Library"),
                    |ui| {
                        let mut project_compat: Vec<&Tool> = Vec::new();
                        let mut machine_compat: Vec<&Tool> = Vec::new();
                        let mut incompat_tools: Vec<&Tool> = Vec::new();
                        for pt in &project.tools {
                            if op_kind.is_compatible_with_tool(pt) {
                                project_compat.push(pt);
                            } else {
                                incompat_tools.push(pt);
                            }
                        }
                        for gt in &global_tools.tools {
                            if project.tools.iter().any(|pt| pt.id == gt.id) {
                                continue;
                            }
                            if op_kind.is_compatible_with_tool(gt) {
                                machine_compat.push(gt);
                            } else {
                                incompat_tools.push(gt);
                            }
                        }

                        let selected_label = cur_tid
                            .as_ref()
                            .and_then(|id| {
                                project
                                    .tools
                                    .iter()
                                    .chain(global_tools.tools.iter())
                                    .find(|t| &t.id == id)
                            })
                            .map(|t| t.summary_label())
                            .unwrap_or_else(|| "Select a Tool…".to_string());

                        let total_compat = project_compat.len() + machine_compat.len();
                        let filter_id =
                            ui.make_persistent_id(("op_tool_filter", setup_idx, op_idx));
                        let show_incompat_id =
                            ui.make_persistent_id(("op_tool_show_incompat", setup_idx, op_idx));

                        egui::ComboBox::from_id_salt(("op_tool_cb", setup_idx, op_idx))
                            .width(COMBO_COL_WIDTH)
                            .truncate()
                            .selected_text(selected_label)
                            .show_ui(ui, |ui| {
                                let mut filter_txt = if total_compat > 6 {
                                    let mut txt = ui
                                        .data_mut(|d| d.get_temp::<String>(filter_id))
                                        .unwrap_or_default();
                                    ui.add(
                                        egui::TextEdit::singleline(&mut txt)
                                            .desired_width(COMBO_COL_WIDTH - 12.0)
                                            .hint_text("Search tools…"),
                                    );
                                    ui.data_mut(|d| d.insert_temp(filter_id, txt.clone()));
                                    ui.separator();
                                    txt
                                } else {
                                    String::new()
                                };
                                filter_txt = filter_txt.trim().to_ascii_lowercase();
                                let matches_filter = |t: &Tool| -> bool {
                                    filter_txt.is_empty()
                                        || t.summary_label()
                                            .to_ascii_lowercase()
                                            .contains(&filter_txt)
                                };

                                if !project_compat.is_empty() {
                                    ui.label(
                                        egui::RichText::new("Project Tools")
                                            .small()
                                            .strong()
                                            .color(tok.text_muted),
                                    );
                                    for &candidate in &project_compat {
                                        if !matches_filter(candidate) {
                                            continue;
                                        }
                                        let is_sel = cur_tid.as_ref() == Some(&candidate.id);
                                        if ui
                                            .selectable_label(is_sel, candidate.summary_label())
                                            .on_hover_text(
                                                "Assign this project tool to the operation",
                                            )
                                            .clicked()
                                            && !is_sel
                                        {
                                            selected_candidate = Some(candidate.clone());
                                        }
                                    }
                                }

                                if !machine_compat.is_empty() {
                                    if !project_compat.is_empty() {
                                        ui.separator();
                                    }
                                    ui.label(
                                        egui::RichText::new("Machine Library (auto-embeds)")
                                            .small()
                                            .strong()
                                            .color(tok.text_muted),
                                    );
                                    for &candidate in &machine_compat {
                                        if !matches_filter(candidate) {
                                            continue;
                                        }
                                        let is_sel = cur_tid.as_ref() == Some(&candidate.id);
                                        if ui
                                            .selectable_label(is_sel, candidate.summary_label())
                                            .on_hover_text(
                                                "Embed from active machine library and assign to this operation",
                                            )
                                            .clicked()
                                            && !is_sel
                                        {
                                            selected_candidate = Some(candidate.clone());
                                        }
                                    }
                                }

                                if total_compat == 0 {
                                    ui.weak("No compatible cutters found in current machine library.");
                                }

                                if !incompat_tools.is_empty() {
                                    ui.separator();
                                    let mut show_incompat = ui
                                        .data_mut(|d| d.get_temp::<bool>(show_incompat_id))
                                        .unwrap_or(false);
                                    ui.checkbox(
                                        &mut show_incompat,
                                        format!("Show incompatible ({})", incompat_tools.len()),
                                    );
                                    ui.data_mut(|d| {
                                        d.insert_temp(show_incompat_id, show_incompat);
                                    });
                                    if show_incompat {
                                        for &candidate in &incompat_tools {
                                            if !matches_filter(candidate) {
                                                continue;
                                            }
                                            let is_sel = cur_tid.as_ref() == Some(&candidate.id);
                                            if ui
                                                .selectable_label(
                                                    is_sel,
                                                    format!("[Incompat] {}", candidate.summary_label()),
                                                )
                                                .on_hover_text(
                                                    "Warning: this cutter geometry is incompatible with the selected strategy",
                                                )
                                                .clicked()
                                                && !is_sel
                                            {
                                                selected_candidate = Some(candidate.clone());
                                            }
                                        }
                                    }
                                }
                            })
                            .response
                            .on_hover_text("Select cutting tool assigned to this CAM operation");
                        if ui
                            .small_button("Edit…")
                            .on_hover_text(
                                "Open Tool Library Manager to edit cutter geometry or presets",
                            )
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
    let preset_baseline = {
        let mut base_op = op.clone();
        base_op.feed_overrides.clear_all();
        base_op.effective_cutting_params(&active_tool)
    };
    let cur_fz_mm =
        chipload_from_feed(eff2.spindle_rpm, flutes, eff2.cutting_feed_mm_min).unwrap_or(0.0);
    let feed_min_disp = display_unit.feed_from_mm_min(1.0);
    let feed_max_disp = display_unit.feed_from_mm_min(25_000.0);
    let feed_speed_disp = display_unit.feed_from_mm_min(5.0);
    let feed_decimals = if display_unit == DisplayUnit::Inch {
        2
    } else {
        1
    };

    section_card(
        ui,
        ("tool_feed_card", setup_idx, op_idx),
        "Feedrate, Stepover, & Coolant",
        true,
        |ui| {
            inspector_grid(ui, ("tool_feed_grid", setup_idx, op_idx), |ui| {
                let mut vf_disp = display_unit.feed_from_mm_min(eff2.cutting_feed_mm_min);
                let preset_vf_disp =
                    display_unit.feed_from_mm_min(preset_baseline.cutting_feed_mm_min);
                grid_row(
                    ui,
                    "Cutting Feed",
                    Some("Linear XY cutting feedrate Vf = RPM * flutes * fz"),
                    |ui| {
                        let r = ui
                            .add(
                                egui::DragValue::new(&mut vf_disp)
                                    .speed(feed_speed_disp)
                                    .max_decimals(feed_decimals)
                                    .range(feed_min_disp..=feed_max_disp)
                                    .suffix(format!(" {}", display_unit.feed_suffix())),
                            )
                            .on_hover_text(format!(
                                "Linear XY cutting feedrate; automatically updates Chipload (preset: {:.1} {})",
                                preset_vf_disp,
                                display_unit.feed_suffix()
                            ));
                        if op.feed_overrides.cutting_feed_mm_min.is_some() {
                            ui.colored_label(tok.dirty_fg, "*").on_hover_text(format!(
                                "Overridden from material preset ({:.1} {})",
                                preset_vf_disp,
                                display_unit.feed_suffix()
                            ));
                            if ui
                                .small_button("Reset")
                                .on_hover_text(format!(
                                    "Revert Cutting Feed to preset ({:.1} {})",
                                    preset_vf_disp,
                                    display_unit.feed_suffix()
                                ))
                                .clicked()
                            {
                                op.feed_overrides.cutting_feed_mm_min = None;
                                *mutated_label = Some("Reset Cutting Feedrate");
                            }
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

                // Lead-In / Lead-Out Feedrate override
                show_feed_override_row(
                    ui,
                    FeedOverrideRowSpec {
                        label: "Lead Feed",
                        tooltip: "Feedrate for tangent lead-in and lead-out arc/linear moves",
                        effective_mm_min: eff2.lead_feed_mm_min,
                        preset_mm_min: preset_baseline.lead_feed_mm_min,
                        display_unit,
                        tok: &tok,
                        edit_undo_label: "Adjust Lead Feedrate",
                        reset_undo_label: "Reset Lead Feedrate",
                    },
                    &mut op.feed_overrides.lead_feed_mm_min,
                    mutated_label,
                );

                // Ramp Feedrate override
                show_feed_override_row(
                    ui,
                    FeedOverrideRowSpec {
                        label: "Ramp Feed",
                        tooltip: "Feedrate during helical and linear cavity entry ramps",
                        effective_mm_min: eff2.ramp_feed_mm_min,
                        preset_mm_min: preset_baseline.ramp_feed_mm_min,
                        display_unit,
                        tok: &tok,
                        edit_undo_label: "Adjust Ramp Feedrate",
                        reset_undo_label: "Reset Ramp Feedrate",
                    },
                    &mut op.feed_overrides.ramp_feed_mm_min,
                    mutated_label,
                );

                // Plunge Feedrate override
                show_feed_override_row(
                    ui,
                    FeedOverrideRowSpec {
                        label: "Plunge Feed",
                        tooltip: "Vertical Z plunge and drilling feedrate",
                        effective_mm_min: eff2.plunge_feed_mm_min,
                        preset_mm_min: preset_baseline.plunge_feed_mm_min,
                        display_unit,
                        tok: &tok,
                        edit_undo_label: "Adjust Plunge Feedrate",
                        reset_undo_label: "Reset Plunge Feedrate",
                    },
                    &mut op.feed_overrides.plunge_feed_mm_min,
                    mutated_label,
                );

                // Axial Stepdown override
                let mut ap_disp = display_unit.from_mm(eff2.stepdown_mm);
                let preset_ap_str = display_unit.format_length(preset_baseline.stepdown_mm);
                let len_min = display_unit.from_mm(0.01);
                let len_max = display_unit.from_mm(500.0);
                let len_speed = display_unit.from_mm(0.05);
                let len_decimals = if display_unit == DisplayUnit::Inch {
                    4
                } else {
                    3
                };
                grid_row(
                    ui,
                    "Axial Stepdown",
                    Some(
                        "Per-operation axial depth of cut (ap) override (defaults to material preset)",
                    ),
                    |ui| {
                        let r = ui
                            .add(
                                egui::DragValue::new(&mut ap_disp)
                                    .speed(len_speed)
                                    .max_decimals(len_decimals)
                                    .range(len_min..=len_max)
                                    .suffix(format!(" {}", display_unit.suffix())),
                            )
                            .on_hover_text(format!(
                                "Per-operation axial stepdown override (preset: {preset_ap_str})"
                            ));
                        if r.changed() {
                            op.feed_overrides.stepdown_mm = Some(display_unit.to_mm(ap_disp));
                            *mutated_label = Some("Adjust Stepdown Override");
                        }
                        if op.feed_overrides.stepdown_mm.is_some() {
                            ui.colored_label(tok.dirty_fg, "*").on_hover_text(format!(
                                "Overridden from material preset ({preset_ap_str})"
                            ));
                            if ui
                                .small_button("Reset")
                                .on_hover_text(format!(
                                    "Revert Axial Stepdown to preset ({preset_ap_str})"
                                ))
                                .clicked()
                            {
                                op.feed_overrides.stepdown_mm = None;
                                *mutated_label = Some("Reset Stepdown Override");
                            }
                        } else {
                            ui.weak("(preset)").on_hover_text(format!(
                                "Inherited from material preset ({preset_ap_str})"
                            ));
                        }
                    },
                );

                let mut ae_disp = display_unit.from_mm(eff2.stepover_mm);
                let preset_ae_str = display_unit.format_length(preset_baseline.stepover_mm);
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
                                    .speed(len_speed)
                                    .max_decimals(len_decimals)
                                    .range(len_min..=len_max)
                                    .suffix(format!(" {}", display_unit.suffix())),
                            )
                            .on_hover_text(format!(
                                "Radial stepover override (preset: {preset_ae_str})"
                            ))
                            .changed()
                        {
                            op.feed_overrides.stepover_mm = Some(display_unit.to_mm(ae_disp));
                            *mutated_label = Some("Adjust Stepover");
                        }
                        if op.feed_overrides.stepover_mm.is_some() {
                            ui.colored_label(tok.dirty_fg, "*").on_hover_text(format!(
                                "Overridden from material preset ({preset_ae_str})"
                            ));
                            if ui
                                .small_button("Reset")
                                .on_hover_text(format!(
                                    "Revert Radial Stepover to preset ({preset_ae_str})"
                                ))
                                .clicked()
                            {
                                op.feed_overrides.stepover_mm = None;
                                *mutated_label = Some("Reset Stepover");
                            }
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
                            ui.colored_label(tok.dirty_fg, "*").on_hover_text(format!(
                                "Overridden from material preset ({})",
                                preset_baseline.coolant.label()
                            ));
                            if ui
                                .small_button("Reset")
                                .on_hover_text(format!(
                                    "Revert Coolant to preset/tool default ({})",
                                    preset_baseline.coolant.label()
                                ))
                                .clicked()
                            {
                                op.feed_overrides.coolant = None;
                                *mutated_label = Some("Reset Operation Coolant");
                            }
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

struct FeedOverrideRowSpec<'a> {
    label: &'static str,
    tooltip: &'static str,
    effective_mm_min: f64,
    preset_mm_min: f64,
    display_unit: DisplayUnit,
    tok: &'a ThemeTokens,
    edit_undo_label: &'static str,
    reset_undo_label: &'static str,
}

fn show_feed_override_row(
    ui: &mut egui::Ui,
    spec: FeedOverrideRowSpec<'_>,
    override_slot: &mut Option<f64>,
    mutated_label: &mut Option<&'static str>,
) {
    let u = spec.display_unit;
    let mut disp = u.feed_from_mm_min(spec.effective_mm_min);
    let preset_disp = u.feed_from_mm_min(spec.preset_mm_min);
    let min_disp = u.feed_from_mm_min(1.0);
    let max_disp = u.feed_from_mm_min(25_000.0);
    let speed_disp = u.feed_from_mm_min(5.0);
    let decimals = if u == DisplayUnit::Inch { 2 } else { 1 };
    let suffix = u.feed_suffix();

    grid_row(ui, spec.label, Some(spec.tooltip), |ui| {
        let r = ui
            .add(
                egui::DragValue::new(&mut disp)
                    .speed(speed_disp)
                    .max_decimals(decimals)
                    .range(min_disp..=max_disp)
                    .suffix(format!(" {suffix}")),
            )
            .on_hover_text(format!(
                "{} (preset: {:.1} {suffix})",
                spec.tooltip, preset_disp
            ));
        if r.changed() {
            *override_slot = Some(u.feed_to_mm_min(disp));
            *mutated_label = Some(spec.edit_undo_label);
        }
        if override_slot.is_some() {
            ui.colored_label(spec.tok.dirty_fg, "*")
                .on_hover_text(format!(
                    "Overridden from material preset ({preset_disp:.1} {suffix})"
                ));
            if ui
                .small_button("Reset")
                .on_hover_text(format!(
                    "Revert {} to material preset ({preset_disp:.1} {suffix})",
                    spec.label
                ))
                .clicked()
            {
                *override_slot = None;
                *mutated_label = Some(spec.reset_undo_label);
            }
        } else {
            ui.weak("(preset)").on_hover_text(format!(
                "Inherited from material preset ({preset_disp:.1} {suffix})"
            ));
        }
    });
}
