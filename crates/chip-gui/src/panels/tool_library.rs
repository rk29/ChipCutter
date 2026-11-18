use chip_math::DisplayUnit;
use chip_project::{ProjectDocument, UndoHistory};
use chip_tools::{
    CoolantMode, CutterGeometry, CuttingPreset, SpindleDirection, SurfaceSpeedPresetSpec, Tool,
    ToolId, ToolLibrary, default_global_tool_library_path, import_fusion_library_path,
};

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
        if !self.open {
            return;
        }
        let mut keep_open = self.open;

        egui::Window::new("ChipCutter Tool Library & Cutting Presets")
            .open(&mut keep_open)
            .default_size([840.0, 560.0])
            .resizable(true)
            .show(ctx, |ui| {
                // Top toolbar: Import Fusion 360, Load/Save `.tools.toml`, Add Tool
                ui.horizontal_wrapped(|ui| {
                    if ui.button("+ New Cutter").clicked() {
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
                    }

                    if ui
                        .button("Import Fusion 360 (.json / .tools)…")
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

                    if ui.button("Load .tools.toml…").clicked()
                        && let Some(path) = rfd::FileDialog::new()
                            .add_filter("ChipCutter Tool Library", &["toml"])
                            .pick_file()
                    {
                        match ToolLibrary::load_from_path(&path) {
                            Ok(loaded) => {
                                *global_lib = loaded;
                                self.selected_tool_idx = 0;
                                self.status_banner =
                                    Some(format!("Loaded library '{}'.", path.display()));
                            }
                            Err(e) => {
                                self.status_banner = Some(format!("Load failed: {e}"));
                            }
                        }
                    }

                    if ui.button("Save Global Library").clicked()
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

                    if ui.button("Export .tools.toml As…").clicked()
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
                    ui.colored_label(egui::Color32::from_rgb(130, 215, 250), banner);
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
                                if ui.selectable_label(sel, t.summary_label()).clicked() {
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
                            ui.strong("Cutter Identity & Offsets");
                            ui.horizontal(|ui| {
                                ui.label("Name:");
                                ui.text_edit_singleline(&mut tool.name);
                            });
                            ui.horizontal(|ui| {
                                ui.label("T#:");
                                ui.add(egui::DragValue::new(&mut tool.tool_number).range(1..=999));
                                ui.label("H#:");
                                ui.add(
                                    egui::DragValue::new(&mut tool.length_offset_number)
                                        .range(1..=999),
                                );
                                ui.label("D#:");
                                ui.add(
                                    egui::DragValue::new(&mut tool.diameter_offset_number)
                                        .range(1..=999),
                                );
                                ui.label("Unit:");
                                ui.selectable_value(
                                    &mut tool.native_unit,
                                    DisplayUnit::Millimeter,
                                    "mm",
                                );
                                ui.selectable_value(
                                    &mut tool.native_unit,
                                    DisplayUnit::Inch,
                                    "in",
                                );
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
                                });

                            edit_cutter_geometry_fields(ui, &mut tool.geometry, unit);
                            edit_mm_drag(ui, "Holder Stickout:", &mut tool.stickout_length_mm, unit);

                            match tool.validate() {
                                Ok(()) => {
                                    ui.colored_label(
                                        egui::Color32::from_rgb(90, 215, 128),
                                        "[OK] Geometry Valid",
                                    );
                                }
                                Err(err) => {
                                    ui.colored_label(
                                        egui::Color32::from_rgb(248, 92, 92),
                                        format!("[!] {err}"),
                                    );
                                }
                            }

                            ui.separator();
                            ui.horizontal(|ui| {
                                ui.strong("Material Cutting Presets");
                                if ui.small_button("+ Preset").clicked() {
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
                                            .clicked()
                                        {
                                            self.selected_preset_idx = pi;
                                        }
                                    }
                                });

                                let dia_mm = tool.geometry.cutting_diameter_mm();
                                let flutes = tool.geometry.flutes();
                                let preset = &mut tool.presets[self.selected_preset_idx];

                                ui.checkbox(&mut self.lock_chipload, "Lock Chipload when editing RPM / Surface Speed");
                                ui.horizontal(|ui| {
                                    ui.label("Preset Name:");
                                    ui.text_edit_singleline(&mut preset.name);
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

        self.open = keep_open;
    }
}

fn edit_mm_drag(ui: &mut egui::Ui, label: &str, val_mm: &mut f64, unit: DisplayUnit) {
    let mut disp = unit.from_mm(*val_mm);
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
                ui.add(egui::DragValue::new(flutes).range(1..=24));
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
                ui.add(egui::DragValue::new(flutes).range(1..=24));
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
                );
            });
            edit_mm_drag(ui, "Flute Length:", flute_length_mm, unit);
            edit_mm_drag(ui, "Overall Length:", overall_length_mm, unit);
            edit_mm_drag(ui, "Shank Diameter:", shank_diameter_mm, unit);
            ui.horizontal(|ui| {
                ui.label("Flutes:");
                ui.add(egui::DragValue::new(flutes).range(1..=24));
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
                );
            });
            edit_mm_drag(ui, "Flute Length:", flute_length_mm, unit);
            edit_mm_drag(ui, "Overall Length:", overall_length_mm, unit);
            edit_mm_drag(ui, "Shank Diameter:", shank_diameter_mm, unit);
            ui.horizontal(|ui| {
                ui.label("Flutes:");
                ui.add(egui::DragValue::new(flutes).range(1..=8));
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
                ui.add(egui::DragValue::new(flutes).range(1..=16));
            });
        }
    }
}

fn draw_cutter_schematic(ui: &mut egui::Ui, tool: &Tool) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(260.0, 128.0), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(18, 21, 26));

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
