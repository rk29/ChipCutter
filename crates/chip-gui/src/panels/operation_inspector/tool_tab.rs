use chip_math::DisplayUnit;
use chip_project::ProjectDocument;
use chip_tools::{
    CoolantMode, Tool, ToolLibrary, chipload_from_feed, compute_radial_chip_thinning,
    feed_from_chipload, rpm_from_surface_speed, surface_speed_from_rpm,
};

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

    // Collect union of Project-Embedded tools and Global Library tools by reference (zero per-frame Tool clones!)
    let cur_tid = project.setups[setup_idx].operations[op_idx].tool_id.clone();
    let mut selected_candidate: Option<Tool> = None;

    {
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

        ui.horizontal(|ui| {
            egui::ComboBox::from_id_salt(("op_tool_cb", setup_idx, op_idx))
                .width(210.0)
                .selected_text(selected_label)
                .show_ui(ui, |ui| {
                    for &candidate in &available_refs {
                        let compat = op_kind.is_compatible_with_tool(candidate);
                        let prefix = if compat { "" } else { "[Incompat] " };
                        let lbl = format!("{prefix}{}", candidate.summary_label());
                        let is_sel = cur_tid.as_ref() == Some(&candidate.id);
                        if ui.selectable_label(is_sel, lbl).clicked() && !is_sel {
                            selected_candidate = Some(candidate.clone());
                        }
                    }
                });
            if ui
                .small_button("Library…")
                .on_hover_text("Open Tool Library Manager")
                .clicked()
            {
                *open_tool_manager = true;
            }
        });
    }

    if let Some(chosen) = selected_candidate {
        project.ensure_tool_embedded(&chosen);
        let op = &mut project.setups[setup_idx].operations[op_idx];
        op.tool_id = Some(chosen.id.clone());
        op.preset_id = chosen.presets.first().map(|p| p.id.clone());
        op.feed_overrides.clear_all();
        *mutated_label = Some("Assign Tool to Operation");
    }

    let Some(active_tool) = project.setups[setup_idx].operations[op_idx]
        .tool_id
        .as_ref()
        .and_then(|id| project.tool(id).cloned())
    else {
        ui.weak("Assign a tool above to configure cutting presets and coupled SFM/chipload.");
        return;
    };

    if !op_kind.is_compatible_with_tool(&active_tool) {
        ui.colored_label(
            egui::Color32::from_rgb(245, 96, 96),
            format!(
                "[!] '{}' is not compatible with {}!",
                active_tool.geometry.kind_label(),
                op_kind.label()
            ),
        );
    }

    // Material Preset selector
    if !active_tool.presets.is_empty() {
        let cur_pid = project.setups[setup_idx].operations[op_idx]
            .preset_id
            .clone()
            .unwrap_or_else(|| active_tool.presets[0].id.clone());
        let cur_pname = active_tool
            .presets
            .iter()
            .find(|p| p.id == cur_pid)
            .map(|p| p.name.as_str())
            .unwrap_or("Default");

        ui.horizontal(|ui| {
            ui.label("Material Preset:");
            egui::ComboBox::from_id_salt(("op_preset_cb", setup_idx, op_idx))
                .selected_text(cur_pname)
                .show_ui(ui, |ui| {
                    for p in &active_tool.presets {
                        let is_p = cur_pid == p.id;
                        if ui.selectable_label(is_p, &p.name).clicked() && !is_p {
                            let op = &mut project.setups[setup_idx].operations[op_idx];
                            op.preset_id = Some(p.id.clone());
                            op.feed_overrides.clear_all();
                            *mutated_label = Some("Change Material Preset");
                        }
                    }
                });
        });
    }

    let op = &mut project.setups[setup_idx].operations[op_idx];
    let eff = op.effective_cutting_params(&active_tool);
    let dia_mm = active_tool.geometry.cutting_diameter_mm();
    let flutes = active_tool.geometry.flutes();

    ui.horizontal(|ui| {
        ui.checkbox(lock_chipload, "Lock Chipload on RPM/SFM edit");
        if !op.feed_overrides.is_all_inherited()
            && ui
                .small_button("Reset Overrides")
                .on_hover_text("Revert all locally overridden feeds/speeds to the material preset")
                .clicked()
        {
            op.feed_overrides.clear_all();
            *mutated_label = Some("Reset Feed/Speed Overrides");
        }
    });
    ui.separator();

    // Coupled Spindle RPM <-> Surface Speed
    ui.strong("Coupled Spindle & Surface Speed");
    let mut rpm_val = eff.spindle_rpm;
    ui.horizontal(|ui| {
        ui.label("Spindle Speed:");
        let r = ui.add(
            egui::DragValue::new(&mut rpm_val)
                .speed(25.0)
                .range(100.0..=60_000.0)
                .suffix(" RPM"),
        );
        if op.feed_overrides.spindle_rpm.is_some() {
            ui.colored_label(egui::Color32::from_rgb(245, 190, 66), "*")
                .on_hover_text("Overridden from preset");
        }
        if r.changed() {
            op.feed_overrides.spindle_rpm = Some(rpm_val);
            if *lock_chipload
                && let Some(fz) =
                    chipload_from_feed(eff.spindle_rpm, flutes, eff.cutting_feed_mm_min)
            {
                op.feed_overrides.cutting_feed_mm_min =
                    Some(feed_from_chipload(rpm_val, flutes, fz));
            }
            *mutated_label = Some("Adjust Spindle RPM");
        }
    });

    let cur_vc_m_min = surface_speed_from_rpm(rpm_val, dia_mm).unwrap_or(0.0);
    let mut vc_disp = display_unit.surface_speed_from_m_min(cur_vc_m_min);
    ui.horizontal(|ui| {
        ui.label("Surface Speed:");
        let r = ui.add(
            egui::DragValue::new(&mut vc_disp)
                .speed(1.0)
                .range(1.0..=5_000.0)
                .suffix(format!(" {}", display_unit.surface_speed_suffix())),
        );
        if r.changed() {
            let new_vc_m = display_unit.surface_speed_to_m_min(vc_disp);
            if let Some(new_rpm) = rpm_from_surface_speed(new_vc_m, dia_mm) {
                op.feed_overrides.spindle_rpm = Some(new_rpm);
                if *lock_chipload
                    && let Some(fz) =
                        chipload_from_feed(eff.spindle_rpm, flutes, eff.cutting_feed_mm_min)
                {
                    op.feed_overrides.cutting_feed_mm_min =
                        Some(feed_from_chipload(new_rpm, flutes, fz));
                }
                *mutated_label = Some("Adjust Surface Speed");
            }
        }
    });

    ui.add_space(4.0);
    ui.strong("Coupled Feedrate & Chipload");

    let eff2 = op.effective_cutting_params(&active_tool);
    let mut vf_disp = display_unit.feed_from_mm_min(eff2.cutting_feed_mm_min);
    ui.horizontal(|ui| {
        ui.label("Cutting Feed:");
        let r = ui.add(
            egui::DragValue::new(&mut vf_disp)
                .speed(5.0)
                .range(1.0..=25_000.0)
                .suffix(format!(" {}", display_unit.feed_suffix())),
        );
        if op.feed_overrides.cutting_feed_mm_min.is_some() {
            ui.colored_label(egui::Color32::from_rgb(245, 190, 66), "*");
        }
        if r.changed() {
            op.feed_overrides.cutting_feed_mm_min = Some(display_unit.feed_to_mm_min(vf_disp));
            *mutated_label = Some("Adjust Cutting Feedrate");
        }
    });

    let cur_fz_mm =
        chipload_from_feed(eff2.spindle_rpm, flutes, eff2.cutting_feed_mm_min).unwrap_or(0.0);
    let mut fz_disp = display_unit.chipload_from_mm(cur_fz_mm);
    ui.horizontal(|ui| {
        ui.label(format!("Chipload ({flutes} fl):"));
        let r = ui.add(
            egui::DragValue::new(&mut fz_disp)
                .speed(0.001)
                .max_decimals(5)
                .range(0.0001..=5.0)
                .suffix(format!(" {}", display_unit.chipload_suffix())),
        );
        if r.changed() {
            let new_fz_mm = display_unit.chipload_to_mm(fz_disp);
            op.feed_overrides.cutting_feed_mm_min =
                Some(feed_from_chipload(eff2.spindle_rpm, flutes, new_fz_mm));
            *mutated_label = Some("Adjust Chipload");
        }
    });

    // Stepdown, Stepover, & Coolant
    let mut ae_disp = display_unit.from_mm(eff2.stepover_mm);
    ui.horizontal(|ui| {
        ui.label("Radial Stepover:");
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
    });

    if let Some(rct) = compute_radial_chip_thinning(
        dia_mm,
        eff2.stepover_mm,
        cur_fz_mm,
        eff2.cutting_feed_mm_min,
    ) {
        ui.colored_label(
            egui::Color32::from_rgb(110, 212, 248),
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

    let mut cool = eff2.coolant;
    ui.horizontal(|ui| {
        ui.label("Coolant:");
        egui::ComboBox::from_id_salt(("op_cool_cb", setup_idx, op_idx))
            .selected_text(cool.label())
            .show_ui(ui, |ui| {
                for c in CoolantMode::ALL {
                    if ui.selectable_value(&mut cool, c, c.label()).changed() {
                        op.feed_overrides.coolant = Some(cool);
                        *mutated_label = Some("Change Operation Coolant");
                    }
                }
            });
    });
}
