use chip_math::DisplayUnit;
use chip_tools::{CuttingPreset, SurfaceSpeedPresetSpec, Tool};

use super::geometry_editor::edit_mm_drag;

pub(super) fn show_presets_editor(
    ui: &mut egui::Ui,
    tool: &mut Tool,
    unit: DisplayUnit,
    selected_preset_idx: &mut usize,
    lock_chipload: &mut bool,
) {
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
        }
    });

    if !tool.presets.is_empty() {
        if *selected_preset_idx >= tool.presets.len() {
            *selected_preset_idx = 0;
        }
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
        let preset = &mut tool.presets[*selected_preset_idx];

        ui.checkbox(
                                    &mut *lock_chipload,
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
                preset.set_spindle_rpm(rpm, dia_mm, flutes, *lock_chipload);
            }
        });

        let mut vc_disp = unit.surface_speed_from_m_min(preset.surface_speed_m_min);
        ui.horizontal(|ui| {
            ui.label("Surface Speed:");
            if ui
                .add(
                    egui::DragValue::new(&mut vc_disp)
                        .speed(1.0)
                        .range(1.0..=5_000.0)
                        .suffix(format!(" {}", unit.surface_speed_suffix())),
                )
                .on_hover_text(
                    "Cutter perimeter cutting speed (m/min or SFM); automatically updates RPM",
                )
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

        let mut vf_disp = unit.feed_from_mm_min(preset.cutting_feed_mm_min);
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
                preset.set_cutting_feed_mm_min(unit.feed_to_mm_min(vf_disp), flutes);
            }
        });

        let mut fz_disp = unit.chipload_from_mm(preset.chipload_mm_tooth);
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
                preset.set_chipload_mm_tooth(unit.chipload_to_mm(fz_disp), flutes);
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
                    .on_hover_text(
                        "Target radial stepover width for constant-engagement Adaptive clearing",
                    )
                    .changed()
                {
                    *opt_mm = unit.to_mm(disp);
                }
            }
        });
    }
}
