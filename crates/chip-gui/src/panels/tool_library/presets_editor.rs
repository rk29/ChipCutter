use chip_math::DisplayUnit;
use chip_tools::{CoolantMode, CuttingPreset, SurfaceSpeedPresetSpec, Tool};

use crate::widgets::{grid_row, inspector_grid};

use super::geometry_editor::{TOOL_COMBO_W, TOOL_NUM_W, edit_mm_grid_row};

pub(super) fn show_presets_editor(
    ui: &mut egui::Ui,
    tool: &mut Tool,
    unit: DisplayUnit,
    selected_preset_idx: &mut usize,
    lock_chipload: &mut bool,
) -> bool {
    let mut presets_list_changed = false;
    ui.horizontal(|ui| {
        if ui
            .small_button("+ Preset")
            .on_hover_text("Add a new material feeds-and-speeds preset for this cutter")
            .clicked()
        {
            let d = tool.geometry.cutting_diameter_mm();
            let fl = tool.geometry.flutes();
            let mut next_idx = tool.presets.len() + 1;
            let mut pid = format!("preset_{next_idx}");
            while tool.presets.iter().any(|p| p.id == pid) {
                next_idx += 1;
                pid = format!("preset_{next_idx}");
            }
            tool.presets
                .push(CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
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
                }));
            *selected_preset_idx = tool.presets.len() - 1;
            presets_list_changed = true;
        }

        let can_delete = tool.presets.len() > 1;
        if ui
            .add_enabled(can_delete, egui::Button::new("- Delete Preset").small())
            .on_hover_text("Remove the currently selected material cutting preset")
            .on_disabled_hover_text("At least one cutting preset must remain on the tool")
            .clicked()
        {
            let rm_idx = (*selected_preset_idx).min(tool.presets.len().saturating_sub(1));
            tool.presets.remove(rm_idx);
            *selected_preset_idx = rm_idx.min(tool.presets.len().saturating_sub(1));
            presets_list_changed = true;
        }
    });

    if !tool.presets.is_empty() {
        if *selected_preset_idx >= tool.presets.len() {
            *selected_preset_idx = tool.presets.len() - 1;
        }
        ui.add_space(4.0);
        ui.horizontal_wrapped(|ui| {
            for (pi, p) in tool.presets.iter().enumerate() {
                if ui
                    .selectable_label(pi == *selected_preset_idx, &p.name)
                    .on_hover_text(
                        "Select this material preset to edit its feeds, speeds, and cut depths",
                    )
                    .clicked()
                {
                    *selected_preset_idx = pi;
                }
            }
        });

        let dia_mm = tool.geometry.cutting_diameter_mm();
        let flutes = tool.geometry.flutes();
        let default_coolant_label = tool.default_coolant.label();
        let preset = &mut tool.presets[*selected_preset_idx];

        ui.add_space(4.0);
        ui.checkbox(
            &mut *lock_chipload,
            "Lock Chipload when editing RPM / Surface Speed",
        )
        .on_hover_text(
            "When checked, changing RPM or surface speed automatically scales cutting feed to keep chipload per tooth constant",
        );
        ui.add_space(4.0);

        inspector_grid(ui, ("tool_presets_grid", *selected_preset_idx), |ui| {
            let name_tip = "Descriptive name for this material/cutting preset";
            grid_row(ui, "Preset Name", Some(name_tip), |ui| {
                ui.add(egui::TextEdit::singleline(&mut preset.name).desired_width(TOOL_COMBO_W))
                    .on_hover_text(name_tip);
            });

            let mat_tip = "Workpiece material targeted by this preset (e.g., Aluminum 6061, Hardwood, Mild Steel)";
            grid_row(ui, "Material", Some(mat_tip), |ui| {
                ui.add(
                    egui::TextEdit::singleline(&mut preset.material).desired_width(TOOL_COMBO_W),
                )
                .on_hover_text(mat_tip);
            });

            let cool_tip =
                "Optional per-material coolant override (defaults to the cutter's default coolant)";
            grid_row(ui, "Coolant Override", Some(cool_tip), |ui| {
                let cur_cool_text = match preset.coolant_override {
                    None => format!("Tool Default ({default_coolant_label})"),
                    Some(c) => c.label().to_string(),
                };
                egui::ComboBox::from_id_salt(("preset_coolant_override_cb", *selected_preset_idx))
                    .width(TOOL_COMBO_W)
                    .selected_text(cur_cool_text)
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut preset.coolant_override,
                            None,
                            format!("Tool Default ({default_coolant_label})"),
                        )
                        .on_hover_text("Inherit the tool's default coolant mode");
                        for c in CoolantMode::ALL {
                            ui.selectable_value(&mut preset.coolant_override, Some(c), c.label())
                                .on_hover_text(format!(
                                    "Override coolant to {} for this material preset",
                                    c.label()
                                ));
                        }
                    })
                    .response
                    .on_hover_text(cool_tip);
            });

            let mut rpm = preset.spindle_rpm;
            let rpm_tip = "Spindle rotation speed (RPM); linked to Surface Speed and Chipload";
            grid_row(ui, "Spindle RPM", Some(rpm_tip), |ui| {
                if ui
                    .add_sized(
                        [TOOL_NUM_W, 20.0],
                        egui::DragValue::new(&mut rpm)
                            .speed(25.0)
                            .range(100.0..=60_000.0)
                            .suffix(" RPM"),
                    )
                    .on_hover_text(rpm_tip)
                    .changed()
                {
                    preset.set_spindle_rpm(rpm, dia_mm, flutes, *lock_chipload);
                }
            });

            let mut vc_disp = unit.surface_speed_from_m_min(preset.surface_speed_m_min);
            let vc_tip = "Cutter perimeter cutting speed (m/min or SFM); automatically updates RPM";
            grid_row(ui, "Surface Speed", Some(vc_tip), |ui| {
                if ui
                    .add_sized(
                        [TOOL_NUM_W, 20.0],
                        egui::DragValue::new(&mut vc_disp)
                            .speed(1.0)
                            .range(1.0..=5_000.0)
                            .suffix(format!(" {}", unit.surface_speed_suffix())),
                    )
                    .on_hover_text(vc_tip)
                    .changed()
                {
                    preset.set_surface_speed_m_min(
                        unit.surface_speed_to_m_min(vc_disp),
                        dia_mm,
                        flutes,
                        *lock_chipload,
                    );
                }
            });

            let feed_min = unit.feed_from_mm_min(1.0);
            let feed_max = unit.feed_from_mm_min(25_000.0);
            let feed_speed = unit.feed_from_mm_min(5.0);
            let feed_decimals = if unit == DisplayUnit::Inch { 2 } else { 1 };

            let mut vf_disp = unit.feed_from_mm_min(preset.cutting_feed_mm_min);
            let vf_tip = "Linear XY cutting feed rate; automatically updates Chipload";
            grid_row(ui, "Cutting Feed", Some(vf_tip), |ui| {
                if ui
                    .add_sized(
                        [TOOL_NUM_W, 20.0],
                        egui::DragValue::new(&mut vf_disp)
                            .speed(feed_speed)
                            .max_decimals(feed_decimals)
                            .range(feed_min..=feed_max)
                            .suffix(format!(" {}", unit.feed_suffix())),
                    )
                    .on_hover_text(vf_tip)
                    .changed()
                {
                    preset.set_cutting_feed_mm_min(unit.feed_to_mm_min(vf_disp), flutes);
                }
            });

            let mut fz_disp = unit.chipload_from_mm(preset.chipload_mm_tooth);
            let fz_tip = "Chip thickness removed per flute per spindle revolution";
            grid_row(ui, "Chipload", Some(fz_tip), |ui| {
                if ui
                    .add_sized(
                        [TOOL_NUM_W, 20.0],
                        egui::DragValue::new(&mut fz_disp)
                            .speed(0.001)
                            .max_decimals(5)
                            .range(0.0001..=5.0)
                            .suffix(format!(" {}", unit.chipload_suffix())),
                    )
                    .on_hover_text(fz_tip)
                    .changed()
                {
                    preset.set_chipload_mm_tooth(unit.chipload_to_mm(fz_disp), flutes);
                }
            });

            edit_preset_feed_row(
                ui,
                "Lead Feed",
                "Feedrate used for tangent lead-in and lead-out moves",
                &mut preset.lead_feed_mm_min,
                unit,
            );
            edit_preset_feed_row(
                ui,
                "Ramp Feed",
                "Feedrate used for helical and linear cavity entry ramps",
                &mut preset.ramp_feed_mm_min,
                unit,
            );
            edit_preset_feed_row(
                ui,
                "Plunge Feed",
                "Feedrate used for vertical Z plunges and axial drilling cycles",
                &mut preset.plunge_feed_mm_min,
                unit,
            );

            edit_mm_grid_row(
                ui,
                "Default Stepdown",
                "Default axial depth of cut per Z pass when applying this preset",
                &mut preset.default_stepdown_mm,
                unit,
            );
            edit_mm_grid_row(
                ui,
                "Default Stepover",
                "Default radial width of cut between passes when applying this preset",
                &mut preset.default_stepover_mm,
                unit,
            );

            let opt_tip = "Set a preset radial engagement target for 2D/3D Adaptive roughing";
            let mut has_opt = preset.optimal_load_mm.is_some();
            grid_row(ui, "Adaptive Optimal Load", Some(opt_tip), |ui| {
                if ui
                    .checkbox(&mut has_opt, "Enable")
                    .on_hover_text(opt_tip)
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
                        .add_sized(
                            [TOOL_NUM_W, 20.0],
                            egui::DragValue::new(&mut disp)
                                .speed(0.05)
                                .range(0.01..=200.0)
                                .suffix(format!(" {}", unit.suffix())),
                        )
                        .on_hover_text(
                            "Target radial stepover width for constant-engagement Adaptive clearing",
                        )
                        .changed()
                    {
                        *opt_mm = unit.to_mm(disp);
                    }
                }
            });
        });
    }
    presets_list_changed
}

fn edit_preset_feed_row(
    ui: &mut egui::Ui,
    label: &str,
    tooltip: &str,
    val_mm_min: &mut f64,
    unit: DisplayUnit,
) {
    let feed_min = unit.feed_from_mm_min(1.0);
    let feed_max = unit.feed_from_mm_min(25_000.0);
    let feed_speed = unit.feed_from_mm_min(5.0);
    let feed_decimals = if unit == DisplayUnit::Inch { 2 } else { 1 };
    let mut disp = unit.feed_from_mm_min(*val_mm_min);
    grid_row(ui, label, Some(tooltip), |ui| {
        if ui
            .add_sized(
                [TOOL_NUM_W, 20.0],
                egui::DragValue::new(&mut disp)
                    .speed(feed_speed)
                    .max_decimals(feed_decimals)
                    .range(feed_min..=feed_max)
                    .suffix(format!(" {}", unit.feed_suffix())),
            )
            .on_hover_text(tooltip)
            .changed()
        {
            *val_mm_min = unit.feed_to_mm_min(disp);
        }
    });
}
