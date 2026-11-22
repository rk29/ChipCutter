//! Interactive **Two-Sided Flip-Jig Wizard** and **Auto Soft-Jaw Generator Wizard** modals
//! (`chip-gui`, Milestone 63).
//!
//! Strict two-column layout with universal hover tooltips, live exact-geometry preview summary,
//! single-step `< 1 ms` Undo on Apply, and native / XDG-portal STL folder export.

use chip_cad::CadDocument;
use chip_cam::{
    FlipAxis, FlipJigConfig, FlipJigPinCount, FlipJigPlan, STANDARD_PIN_DIAMETERS_MM,
    SoftJawBoltPattern, SoftJawConfig, SoftJawOutputMode, SoftJawPlan, ViseClampAxis,
    WorkOffsetSlot, build_soft_jaw_plan, build_two_sided_flip_jig,
};
use chip_project::{ProjectDocument, UndoHistory};
use chip_tools::{CutterGeometry, Tool, ToolId};

use crate::{
    theme::ThemeTokens,
    widgets::{
        BannerKind, INPUT_COL_WIDTH, IconKind, grid_row, inline_banner, inline_icon,
        inspector_grid, primary_icon_button, static_card,
    },
};

/// Outcome returned when the user applies the Flip-Jig Wizard.
pub struct FlipJigApplyResult {
    pub new_setup_idx: usize,
    pub log_message: String,
}

/// Outcome returned when the user applies or exports from the Auto Soft-Jaw Wizard.
pub struct SoftJawApplyResult {
    pub new_setup_idx: Option<usize>,
    pub log_message: String,
}

/// Modal state for the **Two-Sided Flip-Jig Dowel-Pin Wizard**.
pub struct FlipJigWizardModalState {
    pub open: bool,
    pub setup_idx: usize,
    pub config: FlipJigConfig,
    cached_preview: Option<Result<FlipJigPlan, String>>,
    dirty_preview: bool,
}

impl Default for FlipJigWizardModalState {
    fn default() -> Self {
        Self {
            open: false,
            setup_idx: 0,
            config: FlipJigConfig::default(),
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
            if let Some(new_stock) = plan.updated_setup1_stock {
                project.setups[s_idx].stock = new_stock;
            }
            project.sketches.push(plan.pin_sketch);
            project.setups[s_idx]
                .operations
                .insert(0, plan.setup1_pin_op);
            for op in &mut project.setups[s_idx].operations {
                op.mark_dirty();
            }
            let new_setup_idx = project.setups.len();
            let s2_name = plan.setup2_flip.name.clone();
            let pin_count = plan.pin_centers_setup1_wcs.len();
            project.setups.push(plan.setup2_flip);
            if let Some(sb_sk) = plan.spoilboard_sketch {
                project.sketches.push(sb_sk);
            }
            if let Some(sb_setup) = plan.spoilboard_setup {
                project.setups.push(sb_setup);
            }
            keep_open = false;
            result = Some(FlipJigApplyResult {
                new_setup_idx,
                log_message: format!(
                    "[WORKHOLDING] Applied Flip-Jig Wizard ({} · {pin_count} pins · sym err {:.2e} mm) -> added '{s2_name}'.",
                    plan.flip_axis.short_code(),
                    plan.max_symmetry_error_mm
                ),
            });
        }

        self.open = keep_open;
        result
    }

    fn recompute_preview(&mut self, project: &ProjectDocument, cad_doc: &CadDocument) {
        let hole_dia = self.config.pin_diameter_mm + self.config.pin_fit_clearance_mm;
        let (pin_tool_id, pin_tool_dia) = pick_pin_tool(&project.tools, hole_dia);
        let mut cfg = self.config.clone();
        cfg.pin_tool_id = pin_tool_id;
        cfg.pin_tool_diameter_mm = pin_tool_dia;
        cfg.setup2_id = Some(next_unique_setup_id(project, "setup_2_flip"));
        cfg.setup2_wcs_slot = next_work_offset_slot(project);
        cfg.spoilboard_setup_id = Some(next_unique_setup_id(project, "setup_spoilboard_pins"));
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

/// Modal state for the **Auto Soft-Jaw Generator Wizard**.
pub struct SoftJawWizardModalState {
    pub open: bool,
    pub setup_idx: usize,
    pub config: SoftJawConfig,
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
                                "Custom Spacing / Ø",
                                Some("Custom 2-bolt center spacing and bolt/counterbore diameters (mm)"),
                                |ui| {
                                    ui.horizontal(|ui| {
                                        changed |= ui
                                            .add(
                                                egui::DragValue::new(&mut self.custom_spacing_mm)
                                                    .speed(0.5)
                                                    .range(10.0..=300.0)
                                                    .prefix("C-C ")
                                                    .suffix(" mm"),
                                            )
                                            .on_hover_text("Center-to-center bolt spacing")
                                            .changed();
                                        changed |= ui
                                            .add(
                                                egui::DragValue::new(&mut self.custom_bolt_dia_mm)
                                                    .speed(0.1)
                                                    .range(2.0..=25.0)
                                                    .prefix("Ø")
                                                    .suffix(" mm"),
                                            )
                                            .on_hover_text("Through-bolt clearance hole diameter")
                                            .changed();
                                        changed |= ui
                                            .add(
                                                egui::DragValue::new(&mut self.custom_cbore_dia_mm)
                                                    .speed(0.1)
                                                    .range(3.0..=35.0)
                                                    .prefix("CB Ø")
                                                    .suffix(" mm"),
                                            )
                                            .on_hover_text("Socket-head counterbore diameter")
                                            .changed();
                                    });
                                },
                            );
                        }
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

        if apply_setup_clicked
            && let Some(Ok(mut plan)) = self.cached_preview.take()
            && let Some(sj_setup) = plan.machining_setup.take()
        {
            history.record_before_edit("Apply Auto Soft-Jaw Wizard", None, project);
            if let Some(sk) = plan.pocket_sketch.take() {
                project.sketches.push(sk);
            }
            let new_setup_idx = project.setups.len();
            let sj_name = sj_setup.name.clone();
            project.setups.push(sj_setup);
            keep_open = false;
            result = Some(SoftJawApplyResult {
                new_setup_idx: Some(new_setup_idx),
                log_message: format!(
                    "[WORKHOLDING] Applied Auto Soft-Jaw Wizard -> added '{sj_name}' (grip={:.2} mm, clearance={:.2} mm).",
                    plan.grip_depth_mm, plan.part_clearance_mm
                ),
            });
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
        let tool_id = pick_flat_endmill_id(&project.tools);
        let mut cfg = self.config.clone();
        cfg.roughing_tool_id = tool_id.clone();
        cfg.finishing_tool_id = tool_id;
        cfg.setup_id = Some(next_unique_setup_id(project, "setup_soft_jaws"));
        cfg.wcs_slot = next_work_offset_slot(project);
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

fn next_unique_setup_id(proj: &ProjectDocument, base: &str) -> String {
    if !proj.setups.iter().any(|s| s.id == base) {
        return base.to_string();
    }
    let mut n = 2usize;
    loop {
        let cand = format!("{base}_{n}");
        if !proj.setups.iter().any(|s| s.id == cand) {
            return cand;
        }
        n += 1;
    }
}

fn next_work_offset_slot(proj: &ProjectDocument) -> WorkOffsetSlot {
    for slot in [
        WorkOffsetSlot::G55,
        WorkOffsetSlot::G56,
        WorkOffsetSlot::G57,
        WorkOffsetSlot::G58,
        WorkOffsetSlot::G59,
    ] {
        if !proj.setups.iter().any(|s| s.wcs_slot == slot) {
            return slot;
        }
    }
    WorkOffsetSlot::G55
}

fn pick_pin_tool(tools: &[Tool], hole_dia_mm: f64) -> (Option<ToolId>, Option<f64>) {
    if let Some(drill) = tools.iter().find(|t| {
        matches!(
            t.geometry,
            CutterGeometry::TwistDrill { diameter_mm, .. }
            if (diameter_mm - hole_dia_mm).abs() <= 0.15
        )
    }) {
        return (
            Some(drill.id.clone()),
            Some(drill.geometry.cutting_diameter_mm()),
        );
    }
    if let Some(em) = tools.iter().find(|t| {
        matches!(
            t.geometry,
            CutterGeometry::FlatEndmill { diameter_mm, .. }
            if diameter_mm < hole_dia_mm - 0.2
        )
    }) {
        return (Some(em.id.clone()), Some(em.geometry.cutting_diameter_mm()));
    }
    if let Some(first) = tools.first() {
        return (
            Some(first.id.clone()),
            Some(first.geometry.cutting_diameter_mm()),
        );
    }
    (None, None)
}

fn pick_flat_endmill_id(tools: &[Tool]) -> Option<ToolId> {
    tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { .. }))
        .or_else(|| tools.first())
        .map(|t| t.id.clone())
}
