use chip_cad::CadDocument;
use chip_cam::WorkOffsetSlot;
use chip_project::{
    FlipAxis, FlipJigConfig, FlipJigPinCount, FlipJigPlan, ProjectDocument,
    STANDARD_PIN_DIAMETERS_MM, UndoHistory, apply_flip_jig_plan, build_two_sided_flip_jig,
    next_unique_setup_id_with_base, next_work_offset_slot, pick_pin_tool,
};

use super::{
    FlipJigApplyResult, edit_length_mm, render_optional_tool_combo, render_optional_wcs_combo,
};
use crate::{
    theme::ThemeTokens,
    widgets::{
        BannerKind, INPUT_COL_WIDTH, IconKind, grid_row, inline_banner, inline_icon,
        inspector_grid, primary_icon_button, static_card,
    },
};

/// Modal state for the **Two-Sided Flip-Jig Dowel-Pin Wizard**.
pub struct FlipJigWizardModalState {
    pub open: bool,
    pub setup_idx: usize,
    pub config: FlipJigConfig,
    pub setup2_wcs_slot_override: Option<WorkOffsetSlot>,
    pub spoilboard_wcs_slot_override: Option<WorkOffsetSlot>,
    cached_preview: Option<Result<FlipJigPlan, String>>,
    dirty_preview: bool,
}

impl Default for FlipJigWizardModalState {
    fn default() -> Self {
        Self {
            open: false,
            setup_idx: 0,
            config: FlipJigConfig::default(),
            setup2_wcs_slot_override: None,
            spoilboard_wcs_slot_override: None,
            cached_preview: None,
            dirty_preview: true,
        }
    }
}

impl FlipJigWizardModalState {
    pub fn open_for_setup(&mut self, setup_idx: usize) {
        self.setup_idx = setup_idx;
        self.open = true;
        self.dirty_preview = true;
    }

    pub fn show(
        &mut self,
        ctx: &egui::Context,
        project: &mut ProjectDocument,
        cad_doc: &CadDocument,
        history: &mut UndoHistory,
    ) -> Option<FlipJigApplyResult> {
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
        let mut apply_clicked = false;
        let mut close_clicked = false;
        let display_unit = project.display_units;

        egui::Window::new("Two-Sided Flip-Jig Dowel-Pin Wizard")
            .id(egui::Id::unique("chipcutter_flip_jig_wizard_modal"))
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .default_width(540.0)
            .open(&mut keep_open)
            .show(ctx, |ui| {
                let tok = ThemeTokens::from_ui(ui);
                ui.set_min_width(510.0);

                ui.horizontal(|ui| {
                    inline_icon(ui, IconKind::OrthoCube, tok.accent, 15.0);
                    ui.strong(format!(
                        "Source Setup: {}",
                        project.setups[self.setup_idx].name
                    ));
                });
                ui.add_space(4.0);

                static_card(ui, Some("Flip Axis & Dowel Pin Geometry"), |ui| {
                    let mut changed = false;
                    inspector_grid(ui, "flip_jig_wiz_grid", |ui| {
                        grid_row(
                            ui,
                            "Flip Axis",
                            Some("Axis around which the workpiece flips 180° between Setup 1 and Setup 2"),
                            |ui| {
                                let prev = self.config.flip_axis;
                                egui::ComboBox::from_id_salt("fj_axis_cb")
                                    .width(INPUT_COL_WIDTH + 60.0)
                                    .selected_text(self.config.flip_axis.label())
                                    .show_ui(ui, |ui| {
                                        for a in FlipAxis::ALL {
                                            ui.selectable_value(&mut self.config.flip_axis, a, a.label())
                                                .on_hover_text(a.label());
                                        }
                                    })
                                    .response
                                    .on_hover_text("Choose X-Axis (front-to-back) or Y-Axis (left-to-right) 180° flip");
                                if self.config.flip_axis != prev {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Pin Layout",
                            Some("2 pins directly on the flip centerline or 4 mirrored pins symmetrically across it"),
                            |ui| {
                                let prev = self.config.pin_count;
                                egui::ComboBox::from_id_salt("fj_count_cb")
                                    .width(INPUT_COL_WIDTH + 60.0)
                                    .selected_text(self.config.pin_count.label())
                                    .show_ui(ui, |ui| {
                                        for c in FlipJigPinCount::ALL {
                                            ui.selectable_value(&mut self.config.pin_count, c, c.label())
                                                .on_hover_text(c.label());
                                        }
                                    })
                                    .response
                                    .on_hover_text("Choose 2 on-axis pins or 4 symmetric mirrored pins");
                                if self.config.pin_count != prev {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Pin Preset",
                            Some("Standard metric and imperial dowel pin diameters"),
                            |ui| {
                                ui.horizontal_wrapped(|ui| {
                                    for &(lbl, dia) in STANDARD_PIN_DIAMETERS_MM {
                                        let active = (self.config.pin_diameter_mm - dia).abs() < 1e-3;
                                        if ui
                                            .selectable_label(active, lbl)
                                            .on_hover_text(format!("Set nominal dowel pin diameter to {lbl}"))
                                            .clicked()
                                        {
                                            self.config.pin_diameter_mm = dia;
                                            changed = true;
                                        }
                                    }
                                });
                            },
                        );

                        grid_row(
                            ui,
                            "Pin Diameter (mm)",
                            Some("Nominal hardened steel or brass dowel pin diameter"),
                            |ui| {
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut self.config.pin_diameter_mm)
                                            .speed(0.1)
                                            .range(1.0..=25.0)
                                            .suffix(" mm"),
                                    )
                                    .on_hover_text("Nominal dowel pin diameter in millimeters")
                                    .changed()
                                {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Fit Clearance (mm)",
                            Some("Extra hole diameter clearance for slip/press pin fit (default 0.02 mm)"),
                            |ui| {
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut self.config.pin_fit_clearance_mm)
                                            .speed(0.005)
                                            .range(0.0..=0.5)
                                            .fixed_decimals(3)
                                            .suffix(" mm"),
                                    )
                                    .on_hover_text("Clearance added to pin diameter for drilled/bored holes")
                                    .changed()
                                {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Edge Margin (mm)",
                            Some("Minimum clear distance outside target part 2D silhouette + pin radius"),
                            |ui| {
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut self.config.edge_margin_mm)
                                            .speed(0.2)
                                            .range(1.0..=50.0)
                                            .suffix(" mm"),
                                    )
                                    .on_hover_text("Clearance between dowel pin rim and part outer profile")
                                    .changed()
                                {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Poka-Yoke Offset",
                            Some("Optional asymmetric shift along the flip axis so the flipped part can ONLY seat in the single right orientation"),
                            |ui| {
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut self.config.poka_yoke_offset_mm)
                                            .speed(0.25)
                                            .range(-40.0..=40.0)
                                            .suffix(" mm"),
                                    )
                                    .on_hover_text("Asymmetric shift along the flip axis (0 = symmetric ends, non-zero = error-proof poka-yoke)")
                                    .changed()
                                {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Stock Pin Depth",
                            Some("0.0 mm drills through full stock thickness; positive value sets blind hole depth"),
                            |ui| {
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut self.config.stock_pin_depth_mm)
                                            .speed(0.25)
                                            .range(0.0..=100.0)
                                            .suffix(" mm"),
                                    )
                                    .on_hover_text("Pin hole depth into workpiece stock (0 = full through-stock)")
                                    .changed()
                                {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Spoilboard Depth",
                            Some("Depth of mating dowel registration holes drilled into the spoilboard"),
                            |ui| {
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut self.config.spoilboard_pin_depth_mm)
                                            .speed(0.25)
                                            .range(2.0..=40.0)
                                            .suffix(" mm"),
                                    )
                                    .on_hover_text("Mating spoilboard dowel hole depth in millimeters")
                                    .changed()
                                {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Pin Protrusion Above Jig",
                            Some("Height of dowel pins protruding upward above the spoilboard/jig into Setup 2 stock for keep-out collision avoidance"),
                            |ui| {
                                if edit_length_mm(
                                    ui,
                                    &mut self.config.pin_protrusion_z_mm,
                                    0.5..=60.0,
                                    0.25,
                                    2,
                                    display_unit,
                                    "",
                                    "Dowel pin protrusion height above jig surface for Setup 2 CylinderKeepOut fixtures",
                                ) {
                                    changed = true;
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Options",
                            Some("Automatically expand raw stock margins if pins fall outside and emit optional spoilboard setup"),
                            |ui| {
                                ui.vertical(|ui| {
                                    if ui
                                        .checkbox(
                                            &mut self.config.expand_stock_if_needed,
                                            "Auto-expand stock margins to enclose pins",
                                        )
                                        .on_hover_text("Automatically widen Setup 1 & Setup 2 stock if pins need sacrificial outer margin")
                                        .changed()
                                    {
                                        changed = true;
                                    }
                                    if ui
                                        .checkbox(
                                            &mut self.config.include_spoilboard_setup,
                                            "Create 'Spoilboard Dowel Holes' Setup",
                                        )
                                        .on_hover_text("Also add a dedicated Setup & Drill/Bore operation for the spoilboard registration holes")
                                        .changed()
                                    {
                                        changed = true;
                                    }
                                });
                            },
                        );
                    });

                    ui.add_space(4.0);
                    let auto_s2_slot = next_work_offset_slot(project);
                    let hole_dia = self.config.pin_diameter_mm + self.config.pin_fit_clearance_mm;
                    let (auto_pin_tid, _) = pick_pin_tool(&project.tools, hole_dia);
                    let auto_pin_hint = auto_pin_tid
                        .as_ref()
                        .and_then(|id| project.tool(id))
                        .map(|t| format!("T{}", t.tool_number))
                        .unwrap_or_else(|| "None".to_string());
                    egui::CollapsingHeader::new("Advanced Tool & WCS Overrides")
                        .id_salt("fj_adv_overrides")
                        .default_open(false)
                        .show(ui, |ui| {
                            inspector_grid(ui, "fj_adv_grid", |ui| {
                                grid_row(
                                    ui,
                                    "Pin Tool",
                                    Some("Optional explicit cutting tool for Op-1 and Spoilboard dowel-pin holes (Auto picks matching drill or smaller endmill)"),
                                    |ui| {
                                        if render_optional_tool_combo(
                                            ui,
                                            "fj_pin_tool_cb",
                                            &mut self.config.pin_tool_id,
                                            &project.tools,
                                            &auto_pin_hint,
                                            "Override the automatically selected dowel-pin drill or boring endmill",
                                        ) {
                                            changed = true;
                                        }
                                    },
                                );
                                grid_row(
                                    ui,
                                    "Setup 2 WCS",
                                    Some("Work coordinate offset slot assigned to the generated 180-deg flipped Setup 2"),
                                    |ui| {
                                        if render_optional_wcs_combo(
                                            ui,
                                            "fj_s2_wcs_cb",
                                            &mut self.setup2_wcs_slot_override,
                                            auto_s2_slot,
                                            "Override the work offset slot for Setup 2 (defaults to next unused G55..G59)",
                                        ) {
                                            if let Some(slot) = self.setup2_wcs_slot_override {
                                                self.config.setup2_wcs_slot = slot;
                                            }
                                            changed = true;
                                        }
                                    },
                                );
                                if self.config.include_spoilboard_setup {
                                    grid_row(
                                        ui,
                                        "Spoilboard WCS",
                                        Some("Work coordinate offset slot assigned to the optional Spoilboard Dowel Holes Setup"),
                                        |ui| {
                                            if render_optional_wcs_combo(
                                                ui,
                                                "fj_sb_wcs_cb",
                                                &mut self.spoilboard_wcs_slot_override,
                                                WorkOffsetSlot::G54,
                                                "Override the work offset slot for the Spoilboard Dowel Holes Setup (defaults to G54)",
                                            ) {
                                                if let Some(slot) = self.spoilboard_wcs_slot_override {
                                                    self.config.spoilboard_wcs_slot = slot;
                                                }
                                                changed = true;
                                            }
                                        },
                                    );
                                }
                            });
                        });

                    if changed {
                        self.dirty_preview = true;
                    }
                });

                ui.add_space(5.0);

                static_card(ui, Some("Live Preview & Symmetry Check"), |ui| match &self.cached_preview {
                    Some(Ok(plan)) => {
                        inline_banner(
                            ui,
                            BannerKind::Info,
                            format!(
                                "180° flip symmetry verified (err = {:.2e} mm < 1e-9 mm) · Min part clearance = {} · Hole Ø = {}",
                                plan.max_symmetry_error_mm,
                                display_unit.format_length(plan.min_part_clearance_mm),
                                display_unit.format_length(plan.hole_diameter_mm),
                            ),
                        );
                        ui.add_space(4.0);
                        for (idx, pt) in plan.pin_centers_setup1_wcs.iter().enumerate() {
                            ui.monospace(format!(
                                "  Pin #{}: X = {:+.3} mm, Y = {:+.3} mm (Op-1 WCS)",
                                idx + 1,
                                pt.x,
                                pt.y
                            ));
                        }
                        if plan.updated_setup1_stock.is_some() {
                            ui.small("Stock margins will expand symmetrically to enclose pin walls.");
                        }
                        ui.small(format!(
                            "Will create: Sketch '{}', Op-1 Pin Drill/Bore, '{}' ({} CylinderKeepOut pins, carryover from Op-1){}.",
                            plan.pin_sketch.name,
                            plan.setup2_flip.name,
                            plan.setup2_flip.fixtures.len(),
                            if plan.spoilboard_setup.is_some() {
                                ", and 'Spoilboard Dowel Holes' Setup"
                            } else {
                                ""
                            }
                        ));
                    }
                    Some(Err(err)) => {
                        inline_banner(ui, BannerKind::Error, err);
                    }
                    None => {
                        ui.weak("Computing flip-jig preview…");
                    }
                });

                ui.add_space(6.0);
                ui.separator();
                ui.horizontal(|ui| {
                    let can_apply = matches!(self.cached_preview, Some(Ok(_)));
                    ui.add_enabled_ui(can_apply, |ui| {
                        if primary_icon_button(
                            ui,
                            IconKind::Sparkles,
                            "Apply to Project",
                            "Insert Flip-Jig pin sketch, Op-1 pin op, paired 180° Op-2 Setup, and keep-out pins in a single Undo step",
                        )
                        .clicked()
                        {
                            apply_clicked = true;
                        }
                    });
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .button("Cancel")
                            .on_hover_text("Close the Flip-Jig Wizard without modifying the project")
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
        if apply_clicked && let Some(Ok(plan)) = self.cached_preview.take() {
            history.record_before_edit("Apply Flip-Jig Dowel-Pin Wizard", None, project);
            let s_idx = self.setup_idx;
            let s2_name = plan.setup2_flip.name.clone();
            let pin_count = plan.pin_centers_setup1_wcs.len();
            let flip_code = plan.flip_axis.short_code();
            let max_sym_err = plan.max_symmetry_error_mm;
            if let Some(new_setup_idx) = apply_flip_jig_plan(project, s_idx, plan) {
                keep_open = false;
                result = Some(FlipJigApplyResult {
                    new_setup_idx,
                    log_message: format!(
                        "[WORKHOLDING] Applied Flip-Jig Wizard ({flip_code} · {pin_count} pins · sym err {max_sym_err:.2e} mm) -> added '{s2_name}'."
                    ),
                });
            }
        }

        self.open = keep_open;
        result
    }

    fn recompute_preview(&mut self, project: &ProjectDocument, cad_doc: &CadDocument) {
        let hole_dia = self.config.pin_diameter_mm + self.config.pin_fit_clearance_mm;
        let (pin_tool_id, pin_tool_dia) = if let Some(tid) = &self.config.pin_tool_id {
            let dia = project.tool(tid).map(|t| t.geometry.cutting_diameter_mm());
            (Some(tid.clone()), dia)
        } else {
            pick_pin_tool(&project.tools, hole_dia)
        };
        let mut cfg = self.config.clone();
        cfg.pin_tool_id = pin_tool_id;
        cfg.pin_tool_diameter_mm = pin_tool_dia;
        cfg.setup2_id = Some(next_unique_setup_id_with_base(project, "setup_2_flip"));
        cfg.setup2_wcs_slot = self
            .setup2_wcs_slot_override
            .unwrap_or_else(|| next_work_offset_slot(project));
        cfg.spoilboard_setup_id = Some(next_unique_setup_id_with_base(
            project,
            "setup_spoilboard_pins",
        ));
        cfg.spoilboard_wcs_slot = self
            .spoilboard_wcs_slot_override
            .unwrap_or(WorkOffsetSlot::G54);
        self.cached_preview = Some(
            build_two_sided_flip_jig(
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
