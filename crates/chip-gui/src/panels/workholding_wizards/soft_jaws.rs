use chip_cad::CadDocument;
use chip_cam::{ViseClampAxis, WorkOffsetSlot};
use chip_project::{
    ProjectDocument, SoftJawBoltPattern, SoftJawConfig, SoftJawOutputMode, SoftJawPlan,
    UndoHistory, apply_soft_jaw_plan, build_soft_jaw_plan, next_unique_setup_id_with_base,
    next_work_offset_slot, pick_flat_endmill_id,
};

use super::{
    SoftJawApplyResult, edit_length_mm, render_optional_tool_combo, render_optional_wcs_combo,
};
use crate::{
    theme::ThemeTokens,
    widgets::{
        BannerKind, INPUT_COL_WIDTH, IconKind, grid_row, inline_banner, inline_icon,
        inspector_grid, primary_icon_button, static_card,
    },
};

/// Modal state for the **Auto Soft-Jaw Generator Wizard**.
pub struct SoftJawWizardModalState {
    pub open: bool,
    pub setup_idx: usize,
    pub config: SoftJawConfig,
    pub wcs_slot_override: Option<WorkOffsetSlot>,
    bolt_preset_idx: usize,
    custom_spacing_mm: f64,
    custom_bolt_dia_mm: f64,
    custom_cbore_dia_mm: f64,
    custom_cbore_depth_mm: f64,
    cached_preview: Option<Result<SoftJawPlan, String>>,
    status_banner: Option<(String, bool)>,
    dirty_preview: bool,
}

impl Default for SoftJawWizardModalState {
    fn default() -> Self {
        Self {
            open: false,
            setup_idx: 0,
            config: SoftJawConfig::default(),
            wcs_slot_override: None,
            bolt_preset_idx: 2, // Milo 4"
            custom_spacing_mm: 63.5,
            custom_bolt_dia_mm: 8.5,
            custom_cbore_dia_mm: 14.0,
            custom_cbore_depth_mm: 8.5,
            cached_preview: None,
            status_banner: None,
            dirty_preview: true,
        }
    }
}

impl SoftJawWizardModalState {
    pub fn open_for_setup(&mut self, setup_idx: usize) {
        self.setup_idx = setup_idx;
        self.open = true;
        self.status_banner = None;
        self.dirty_preview = true;
    }

    pub fn show(
        &mut self,
        ctx: &egui::Context,
        project: &mut ProjectDocument,
        cad_doc: &CadDocument,
        history: &mut UndoHistory,
    ) -> Option<SoftJawApplyResult> {
        if !self.open {
            return None;
        }
        if project.setups.is_empty() {
            self.open = false;
            return None;
        }
        if self.setup_idx >= project.setups.len() {
            self.setup_idx = 0;
        }

        if self.dirty_preview {
            self.recompute_preview(project, cad_doc);
        }

        let mut keep_open = self.open;
        let mut apply_setup_clicked = false;
        let mut export_stl_clicked = false;
        let mut close_clicked = false;
        let display_unit = project.display_units;

        egui::Window::new("Auto Soft-Jaw Generator Wizard")
            .id(egui::Id::unique("chipcutter_soft_jaw_wizard_modal"))
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .default_width(560.0)
            .open(&mut keep_open)
            .show(ctx, |ui| {
                let tok = ThemeTokens::from_ui(ui);
                ui.set_min_width(525.0);

                ui.horizontal(|ui| {
                    inline_icon(ui, IconKind::Pocket2D, tok.accent, 15.0);
                    ui.strong(format!(
                        "Target Part Setup: {}",
                        project.setups[self.setup_idx].name
                    ));
                });
                ui.add_space(4.0);

                static_card(ui, Some("Soft-Jaw Dimensions, Fit, & Bolt Pattern"), |ui| {
                    let mut changed = false;
                    inspector_grid(ui, "soft_jaw_wiz_grid", |ui| {
                        grid_row(
                            ui,
                            "Clamp Axis",
                            Some("WCS axis along which the fixed and movable soft jaws clamp the part"),
                            |ui| {
                                let prev = self.config.clamp_axis;
                                egui::ComboBox::from_id_salt("sj_axis_cb")
                                    .width(INPUT_COL_WIDTH + 60.0)
                                    .selected_text(self.config.clamp_axis.label())
                                    .show_ui(ui, |ui| {
                                        for a in ViseClampAxis::ALL {
                                            ui.selectable_value(&mut self.config.clamp_axis, a, a.label())
                                                .on_hover_text(a.label());
                                        }
                                    })
                                    .response
                                    .on_hover_text("Select front/rear (Y) or left/right (X) vise jaw axis");
                                if self.config.clamp_axis != prev {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Output Mode",
                            Some("Generate a CNC aluminum soft-jaw machining setup, 3D-printable STL files, or both"),
                            |ui| {
                                let prev = self.config.output_mode;
                                egui::ComboBox::from_id_salt("sj_out_cb")
                                    .width(INPUT_COL_WIDTH + 60.0)
                                    .selected_text(self.config.output_mode.label())
                                    .show_ui(ui, |ui| {
                                        for m in SoftJawOutputMode::ALL {
                                            ui.selectable_value(&mut self.config.output_mode, m, m.label())
                                                .on_hover_text(m.label());
                                        }
                                    })
                                    .response
                                    .on_hover_text("Select CNC setup output, 3D-printable STL export, or both");
                                if self.config.output_mode != prev {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Jaw Width (mm)",
                            Some("Length of each soft jaw plate along the non-clamping axis"),
                            |ui| {
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut self.config.jaw_width_mm)
                                            .speed(0.5)
                                            .range(20.0..=400.0)
                                            .suffix(" mm"),
                                    )
                                    .on_hover_text("Width of soft jaw plate (e.g. 100 mm for 4\" vise, 152.4 mm for 6\" Kurt)")
                                    .changed()
                                {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Jaw Depth (mm)",
                            Some("Front-to-back thickness of each soft jaw plate along clamping axis"),
                            |ui| {
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut self.config.jaw_depth_mm)
                                            .speed(0.5)
                                            .range(10.0..=150.0)
                                            .suffix(" mm"),
                                    )
                                    .on_hover_text("Thickness of each soft jaw blank")
                                    .changed()
                                {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Jaw Height (mm)",
                            Some("Total vertical height of each soft jaw blank"),
                            |ui| {
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut self.config.jaw_height_mm)
                                            .speed(0.5)
                                            .range(12.0..=150.0)
                                            .suffix(" mm"),
                                    )
                                    .on_hover_text("Total vertical height of soft jaw blank")
                                    .changed()
                                {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Jaw Spacer Gap",
                            Some("Closed spacer gap placed between fixed and movable jaws during machining/clamping"),
                            |ui| {
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut self.config.jaw_gap_mm)
                                            .speed(0.25)
                                            .range(0.0..=40.0)
                                            .suffix(" mm"),
                                    )
                                    .on_hover_text("Spacer gap between fixed and movable jaws (default 6.0 mm)")
                                    .changed()
                                {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Grip Depth (mm)",
                            Some("Vertical pocket step depth cut into the top of the soft jaws"),
                            |ui| {
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut self.config.grip_depth_mm)
                                            .speed(0.2)
                                            .range(0.5..=60.0)
                                            .suffix(" mm"),
                                    )
                                    .on_hover_text("Pocket step depth into top of soft jaws (default 4.0 mm)")
                                    .changed()
                                {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Grip Band Bottom Offset",
                            Some("Height above part bottom where the soft-jaw clamping silhouette Z-band starts (0 = from part bottom)"),
                            |ui| {
                                if edit_length_mm(
                                    ui,
                                    &mut self.config.z_band_bottom_offset_mm,
                                    0.0..=150.0,
                                    0.25,
                                    2,
                                    display_unit,
                                    "",
                                    "Offset upward from part bottom Z where the soft-jaw gripping band starts",
                                ) {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Part Clearance",
                            Some("Radial slip-fit clearance around part silhouette (0.05 mm aluminum, 0.15 mm 3D-printed)"),
                            |ui| {
                                ui.horizontal(|ui| {
                                    if ui
                                        .add(
                                            egui::DragValue::new(&mut self.config.part_clearance_mm)
                                                .speed(0.01)
                                                .range(0.0..=2.0)
                                                .fixed_decimals(3)
                                                .suffix(" mm"),
                                        )
                                        .on_hover_text("Outward radial fit clearance around target part silhouette")
                                        .changed()
                                    {
                                        changed = true;
                                    }
                                    if ui
                                        .small_button("Al 0.05")
                                        .on_hover_text("Set 0.05 mm precision slip fit for CNC-machined aluminum jaws")
                                        .clicked()
                                    {
                                        self.config.part_clearance_mm = 0.05;
                                        changed = true;
                                    }
                                    if ui
                                        .small_button("3DP 0.15")
                                        .on_hover_text("Set 0.15 mm slip fit for FDM/SLA 3D-printed jaws")
                                        .clicked()
                                    {
                                        self.config.part_clearance_mm = 0.15;
                                        changed = true;
                                    }
                                });
                            },
                        );

                        grid_row(
                            ui,
                            "Corner Relief R",
                            Some("Dogbone corner relief radius at sharp convex corners so square parts seat flat without binding"),
                            |ui| {
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut self.config.corner_relief_radius_mm)
                                            .speed(0.1)
                                            .range(0.0..=12.0)
                                            .suffix(" mm"),
                                    )
                                    .on_hover_text("Internal sharp-corner dogbone relief radius (0 disables)")
                                    .changed()
                                {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Bolt Pattern",
                            Some("Horizontal vise mounting bolt + counterbore pattern cut into 3D-printable STL jaws"),
                            |ui| {
                                let presets = [
                                    (0usize, SoftJawBoltPattern::None.label()),
                                    (1usize, SoftJawBoltPattern::Kurt6Inch.label()),
                                    (2usize, SoftJawBoltPattern::Milo4Inch.label()),
                                    (3usize, SoftJawBoltPattern::CarveraModVise.label()),
                                    (4usize, "Custom Bolt Pattern"),
                                ];
                                let prev_idx = self.bolt_preset_idx;
                                let sel_lbl = presets
                                    .iter()
                                    .find(|(i, _)| *i == self.bolt_preset_idx)
                                    .map_or("Custom", |(_, l)| *l);
                                egui::ComboBox::from_id_salt("sj_bolt_cb")
                                    .width(INPUT_COL_WIDTH + 60.0)
                                    .selected_text(sel_lbl)
                                    .show_ui(ui, |ui| {
                                        for (i, lbl) in presets {
                                            ui.selectable_value(&mut self.bolt_preset_idx, i, lbl)
                                                .on_hover_text(lbl);
                                        }
                                    })
                                    .response
                                    .on_hover_text("Select standard vise mounting bolt pattern or Custom");
                                if self.bolt_preset_idx != prev_idx {
                                    changed = true;
                                }
                            },
                        );

                        if self.bolt_preset_idx == 4 {
                            grid_row(
                                ui,
                                "Custom Spacing / Dia",
                                Some("Custom 2-bolt center spacing and bolt/counterbore diameters"),
                                |ui| {
                                    ui.horizontal(|ui| {
                                        changed |= edit_length_mm(
                                            ui,
                                            &mut self.custom_spacing_mm,
                                            10.0..=300.0,
                                            0.5,
                                            2,
                                            display_unit,
                                            "C-C ",
                                            "Center-to-center bolt spacing",
                                        );
                                        changed |= edit_length_mm(
                                            ui,
                                            &mut self.custom_bolt_dia_mm,
                                            2.0..=25.0,
                                            0.1,
                                            2,
                                            display_unit,
                                            "Dia ",
                                            "Through-bolt clearance hole diameter",
                                        );
                                        changed |= edit_length_mm(
                                            ui,
                                            &mut self.custom_cbore_dia_mm,
                                            3.0..=35.0,
                                            0.1,
                                            2,
                                            display_unit,
                                            "CB Dia ",
                                            "Socket-head counterbore diameter",
                                        );
                                    });
                                },
                            );
                            grid_row(
                                ui,
                                "Custom C'Bore Depth",
                                Some("Socket-head counterbore pocket depth cut into the rear face of each soft jaw"),
                                |ui| {
                                    changed |= edit_length_mm(
                                        ui,
                                        &mut self.custom_cbore_depth_mm,
                                        0.0..=80.0,
                                        0.1,
                                        2,
                                        display_unit,
                                        "",
                                        "Counterbore pocket depth for custom vise mounting bolts",
                                    );
                                },
                            );
                        }
                    });

                    ui.add_space(4.0);
                    let auto_sj_slot = next_work_offset_slot(project);
                    let auto_em_hint = pick_flat_endmill_id(&project.tools)
                        .as_ref()
                        .and_then(|id| project.tool(id))
                        .map(|t| format!("T{}", t.tool_number))
                        .unwrap_or_else(|| "None".to_string());
                    egui::CollapsingHeader::new("Advanced Tool & WCS Overrides")
                        .id_salt("sj_adv_overrides")
                        .default_open(false)
                        .show(ui, |ui| {
                            inspector_grid(ui, "sj_adv_grid", |ui| {
                                grid_row(
                                    ui,
                                    "Roughing Tool",
                                    Some("Optional explicit tool for Facing & Pocket2D roughing on the generated soft-jaw Setup"),
                                    |ui| {
                                        if render_optional_tool_combo(
                                            ui,
                                            "sj_rough_tool_cb",
                                            &mut self.config.roughing_tool_id,
                                            &project.tools,
                                            &auto_em_hint,
                                            "Override the automatically selected flat endmill for soft-jaw roughing",
                                        ) {
                                            changed = true;
                                        }
                                    },
                                );
                                grid_row(
                                    ui,
                                    "Finishing Tool",
                                    Some("Optional explicit tool for Contour2D wall finishing on the generated soft-jaw Setup"),
                                    |ui| {
                                        if render_optional_tool_combo(
                                            ui,
                                            "sj_finish_tool_cb",
                                            &mut self.config.finishing_tool_id,
                                            &project.tools,
                                            &auto_em_hint,
                                            "Override the automatically selected flat endmill for soft-jaw finishing",
                                        ) {
                                            changed = true;
                                        }
                                    },
                                );
                                grid_row(
                                    ui,
                                    "Soft-Jaw WCS",
                                    Some("Work coordinate offset slot assigned to the generated soft-jaw CNC Setup"),
                                    |ui| {
                                        if render_optional_wcs_combo(
                                            ui,
                                            "sj_wcs_cb",
                                            &mut self.wcs_slot_override,
                                            auto_sj_slot,
                                            "Override the work offset slot for the soft-jaw CNC Setup (defaults to next unused G55..G59)",
                                        ) {
                                            if let Some(slot) = self.wcs_slot_override {
                                                self.config.wcs_slot = slot;
                                            }
                                            changed = true;
                                        }
                                    },
                                );
                            });
                        });

                    if changed {
                        self.sync_bolt_pattern();
                        self.dirty_preview = true;
                    }
                });

                ui.add_space(5.0);

                static_card(ui, Some("Live Pocket & STL Preview"), |ui| match &self.cached_preview {
                    Some(Ok(plan)) => {
                        inline_banner(
                            ui,
                            BannerKind::Info,
                            format!(
                                "Grip depth = {} · Clearance = {} · {} corner relief(s) · Fixed pocket = {:.1} mm², Movable pocket = {:.1} mm²",
                                display_unit.format_length(plan.grip_depth_mm),
                                display_unit.format_length(plan.part_clearance_mm),
                                plan.corner_relief_count,
                                plan.fixed_pocket_2d.total_area_mm2(),
                                plan.movable_pocket_2d.total_area_mm2(),
                            ),
                        );
                        if plan.fixed_jaw_stl.is_some() && plan.movable_jaw_stl.is_some() {
                            ui.small(format!(
                                "Watertight STLs ready: fixed_jaw.stl ({} tris) & movable_jaw.stl ({} tris).",
                                plan.fixed_jaw_triangle_count, plan.movable_jaw_triangle_count
                            ));
                        }
                        if let Some(sj) = &plan.machining_setup {
                            ui.small(format!(
                                "CNC Setup ready: '{}' ({} operations: Facing, Pocket2D, Contour2D Finish).",
                                sj.name,
                                sj.operations.len()
                            ));
                        }
                    }
                    Some(Err(err)) => {
                        inline_banner(ui, BannerKind::Error, err);
                    }
                    None => {
                        ui.weak("Computing soft-jaw silhouette & pockets…");
                    }
                });

                if let Some((msg, is_err)) = &self.status_banner {
                    ui.add_space(4.0);
                    inline_banner(
                        ui,
                        if *is_err {
                            BannerKind::Error
                        } else {
                            BannerKind::Info
                        },
                        msg,
                    );
                }

                ui.add_space(6.0);
                ui.separator();
                ui.horizontal(|ui| {
                    let has_setup = matches!(
                        &self.cached_preview,
                        Some(Ok(p)) if p.machining_setup.is_some()
                    );
                    let has_stl = matches!(
                        &self.cached_preview,
                        Some(Ok(p)) if p.fixed_jaw_stl.is_some() && p.movable_jaw_stl.is_some()
                    );

                    ui.add_enabled_ui(has_setup, |ui| {
                        if primary_icon_button(
                            ui,
                            IconKind::Sparkles,
                            "Apply to Project",
                            "Add the 'Soft Jaws — Fixed & Movable' CNC Setup + Pocket Sketch in a single Undo step",
                        )
                        .clicked()
                        {
                            apply_setup_clicked = true;
                        }
                    });

                    ui.add_enabled_ui(has_stl, |ui| {
                        if ui
                            .button("Export Soft Jaws (.stl)…")
                            .on_hover_text(
                                "Choose a folder to export watertight 3D-printable fixed_jaw.stl and movable_jaw.stl",
                            )
                            .clicked()
                        {
                            export_stl_clicked = true;
                        }
                    });

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .button("Close")
                            .on_hover_text("Close the Auto Soft-Jaw Wizard")
                            .clicked()
                        {
                            close_clicked = true;
                        }
                    });
                });
            });

        if close_clicked {
            keep_open = false;
        }

        let mut result = None;
        if export_stl_clicked
            && let Some(Ok(plan)) = &self.cached_preview
            && let (Some(fixed_bytes), Some(movable_bytes)) =
                (&plan.fixed_jaw_stl, &plan.movable_jaw_stl)
            && let Some(dir) = rfd::FileDialog::new()
                .set_title("Export Soft Jaws (fixed_jaw.stl & movable_jaw.stl)")
                .pick_folder()
        {
            let fixed_path = dir.join("fixed_jaw.stl");
            let movable_path = dir.join("movable_jaw.stl");
            match std::fs::write(&fixed_path, fixed_bytes)
                .and_then(|()| std::fs::write(&movable_path, movable_bytes))
            {
                Ok(()) => {
                    let msg = format!(
                        "Exported fixed_jaw.stl ({} tris) & movable_jaw.stl ({} tris) to {}",
                        plan.fixed_jaw_triangle_count,
                        plan.movable_jaw_triangle_count,
                        dir.display()
                    );
                    self.status_banner = Some((msg.clone(), false));
                    result = Some(SoftJawApplyResult {
                        new_setup_idx: None,
                        log_message: format!("[WORKHOLDING] {msg}"),
                    });
                }
                Err(err) => {
                    self.status_banner = Some((format!("STL export failed: {err}"), true));
                }
            }
        }

        if apply_setup_clicked && let Some(Ok(plan)) = self.cached_preview.take() {
            let grip_depth_mm = plan.grip_depth_mm;
            let part_clearance_mm = plan.part_clearance_mm;
            if plan.machining_setup.is_some() {
                history.record_before_edit("Apply Auto Soft-Jaw Wizard", None, project);
                if let Some(new_setup_idx) = apply_soft_jaw_plan(project, plan) {
                    let sj_name = project.setups[new_setup_idx].name.clone();
                    keep_open = false;
                    result = Some(SoftJawApplyResult {
                        new_setup_idx: Some(new_setup_idx),
                        log_message: format!(
                            "[WORKHOLDING] Applied Auto Soft-Jaw Wizard -> added '{sj_name}' (grip={grip_depth_mm:.2} mm, clearance={part_clearance_mm:.2} mm)."
                        ),
                    });
                }
            }
        }

        self.open = keep_open;
        result
    }

    fn sync_bolt_pattern(&mut self) {
        self.config.mounting_bolt_pattern = match self.bolt_preset_idx {
            0 => SoftJawBoltPattern::None,
            1 => SoftJawBoltPattern::Kurt6Inch,
            2 => SoftJawBoltPattern::Milo4Inch,
            3 => SoftJawBoltPattern::CarveraModVise,
            _ => SoftJawBoltPattern::Custom {
                spacing_mm: self.custom_spacing_mm,
                bolt_dia_mm: self.custom_bolt_dia_mm,
                cbore_dia_mm: self.custom_cbore_dia_mm.max(self.custom_bolt_dia_mm),
                cbore_depth_mm: self.custom_cbore_depth_mm,
            },
        };
    }

    fn recompute_preview(&mut self, project: &ProjectDocument, cad_doc: &CadDocument) {
        self.sync_bolt_pattern();
        let auto_tool_id = pick_flat_endmill_id(&project.tools);
        let mut cfg = self.config.clone();
        if cfg.roughing_tool_id.is_none() {
            cfg.roughing_tool_id = auto_tool_id.clone();
        }
        if cfg.finishing_tool_id.is_none() {
            cfg.finishing_tool_id = auto_tool_id;
        }
        cfg.setup_id = Some(next_unique_setup_id_with_base(project, "setup_soft_jaws"));
        cfg.wcs_slot = self
            .wcs_slot_override
            .unwrap_or_else(|| next_work_offset_slot(project));
        self.cached_preview = Some(
            build_soft_jaw_plan(
                cad_doc,
                &project.setups[self.setup_idx],
                &project.sketches,
                project.display_units,
                &cfg,
            )
            .map_err(|e| e.to_string()),
        );
        self.dirty_preview = false;
    }
}
