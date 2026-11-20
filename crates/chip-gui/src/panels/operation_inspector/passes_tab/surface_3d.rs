use chip_cam::{
    Adaptive3DParams, AdaptiveCutDirection, MachiningBoundaryMode, Parallel3DParams,
    ParallelCutDirection, Pencil3DParams, Rotary2LinearParams, RotaryCutDirection,
    RotaryPatternMode, Scallop3DParams, ToolContainmentMode, Waterline3DParams, WaterlineOrderMode,
};

use super::{COMBO_COL_WIDTH, PassItemCtx};
use crate::widgets::section_card;

pub(super) fn show_waterline3d_params(
    ui: &mut egui::Ui,
    p: &mut Waterline3DParams,
    mut item: PassItemCtx<'_>,
) {
    let (setup_idx, op_idx) = item.ids();

    section_card(
        ui,
        ("wl3d_passes_card", setup_idx, op_idx),
        "3D Waterline / Z-Level Passes",
        true,
        |ui| {
            item.grid(ui, "wl3d_passes_grid", |f| {
                f.set_tip("Subdivide Z-levels automatically on shallow slopes to keep cusp height within target");
                f.check(
                    "Adaptive Stepdown",
                    &mut p.adaptive_stepdown,
                    "Subdivide shallow Z bands",
                    "Toggle Waterline Adaptive Stepdown",
                );
                if p.adaptive_stepdown {
                    f.set_tip("Minimum vertical Z-step clamp when subdividing shallow slopes");
                    f.len(
                        "Min Stepdown",
                        &mut p.min_stepdown,
                        "Edit Waterline Min Stepdown",
                    );
                    f.set_tip("Target maximum scallop/cusp height driving adaptive Z refinement");
                    f.len(
                        "Max Cusp Height",
                        &mut p.max_cusp_height,
                        "Edit Waterline Max Cusp Height",
                    );
                }
                f.set_tip("Automatically inject exact Z-levels on detected horizontal flat plateaus");
                f.check(
                    "Detect Flats",
                    &mut p.detect_flats,
                    "Land on flat plateaus",
                    "Toggle Waterline Detect Flats",
                );
                f.set_tip("Filter out shallow or vertical zones by surface slope angle (0° = flat, 90° = vertical)");
                f.row("Slope Range (°)", |ui, mutated| {
                    ui.horizontal(|ui| {
                        let r_min = ui.add(
                            egui::DragValue::new(&mut p.min_slope_deg)
                                .speed(1.0)
                                .range(0.0..=89.0)
                                .suffix("°"),
                        );
                        ui.label("to");
                        let r_max = ui.add(
                            egui::DragValue::new(&mut p.max_slope_deg)
                                .speed(1.0)
                                .range(p.min_slope_deg.max(1.0)..=90.0)
                                .suffix("°"),
                        );
                        if r_min.changed() || r_max.changed() {
                            if p.max_slope_deg < p.min_slope_deg {
                                p.max_slope_deg = p.min_slope_deg;
                            }
                            *mutated = Some("Edit Waterline Slope Range");
                        }
                    });
                });
                f.set_tip("Order waterline contours top-down by Z level or finish each island/boss before retracting");
                f.row("Pass Ordering", |ui, mutated| {
                    let prev = p.order_mode;
                    egui::ComboBox::from_id_salt(("wl3d_order_cb", setup_idx, op_idx))
                        .width(COMBO_COL_WIDTH)
                        .selected_text(p.order_mode.label())
                        .show_ui(ui, |ui| {
                            for m in WaterlineOrderMode::ALL {
                                ui.selectable_value(&mut p.order_mode, m, m.label());
                            }
                        });
                    if p.order_mode != prev {
                        *mutated = Some("Change Waterline Ordering");
                    }
                });
                f.set_tip("Climb milling keeps workpiece material to the right of tool travel");
                f.check(
                    "Cut Direction",
                    &mut p.climb_milling,
                    "Climb Milling",
                    "Toggle Waterline Climb Milling",
                );
                f.set_tip("Helical / contour-following ramp angle when descending to each Z level");
                f.row("Ramp Angle", |ui, mutated| {
                    if ui
                        .add(
                            egui::DragValue::new(&mut p.ramp_angle_deg)
                                .speed(0.25)
                                .range(0.5..=20.0)
                                .suffix("°"),
                        )
                        .changed()
                    {
                        *mutated = Some("Edit Waterline Ramp Angle");
                    }
                });
            });
        },
    );

    show_3d_boundary_card(
        ui,
        ("wl3d_boundary_card", setup_idx, op_idx),
        &mut p.boundary_mode,
        &mut p.tool_containment,
        &mut p.boundary_offset,
        &mut item,
    );
}

pub(super) fn show_adaptive3d_params(
    ui: &mut egui::Ui,
    p: &mut Adaptive3DParams,
    mut item: PassItemCtx<'_>,
) {
    let (setup_idx, op_idx) = item.ids();

    section_card(
        ui,
        ("a3d_passes_card", setup_idx, op_idx),
        "3D Adaptive Roughing",
        true,
        |ui| {
            item.grid(ui, "a3d_passes_grid", |f| {
                f.set_tip("Maximum radial engagement (ae) maintained across constant-load trochoidal peels");
                f.len(
                    "Optimal Load (ae)",
                    &mut p.adaptive.optimal_load,
                    "Edit 3D Adaptive Optimal Load",
                );
                f.set_tip("Primary coarse roughing Z stepdown (ap)");
                f.len(
                    "Rough Stepdown",
                    &mut p.max_roughing_stepdown,
                    "Edit 3D Adaptive Rough Stepdown",
                );
                f.set_tip("Intermediate fine Z step-up height peeling sloped terraces between coarse levels");
                f.len(
                    "Fine Step-Up",
                    &mut p.fine_stepdown,
                    "Edit 3D Adaptive Fine Step-Up",
                );
                f.set_tip("Automatically inject exact Z-levels on detected horizontal flat plateaus");
                f.check(
                    "Detect Flats",
                    &mut p.detect_flats,
                    "Land on flat plateaus",
                    "Toggle 3D Adaptive Detect Flats",
                );
                f.set_tip("Small Z micro-lift during high-speed non-cutting stay-down links");
                f.len(
                    "Micro-Lift Z",
                    &mut p.adaptive.micro_lift_z,
                    "Edit 3D Adaptive Micro-Lift",
                );
                f.set_tip("Boost feedrate automatically to maintain constant chip thickness at light radial loads");
                f.check(
                    "Chip Thinning",
                    &mut p.adaptive.auto_chip_thinning,
                    "Compensate Feedrate",
                    "Toggle 3D Adaptive Chip Thinning",
                );
                f.set_tip("Preferred milling direction along trochoidal peels");
                f.row("Cut Direction", |ui, mutated| {
                    let prev = p.adaptive.cut_direction;
                    egui::ComboBox::from_id_salt(("a3d_dir_cb", setup_idx, op_idx))
                        .width(COMBO_COL_WIDTH)
                        .selected_text(p.adaptive.cut_direction.label())
                        .show_ui(ui, |ui| {
                            for d in AdaptiveCutDirection::ALL {
                                ui.selectable_value(&mut p.adaptive.cut_direction, d, d.label());
                            }
                        });
                    if p.adaptive.cut_direction != prev {
                        *mutated = Some("Change 3D Adaptive Cut Direction");
                    }
                });
            });
        },
    );

    show_3d_boundary_card(
        ui,
        ("a3d_boundary_card", setup_idx, op_idx),
        &mut p.boundary_mode,
        &mut p.tool_containment,
        &mut p.boundary_offset,
        &mut item,
    );
}

pub(super) fn show_parallel3d_params(
    ui: &mut egui::Ui,
    p: &mut Parallel3DParams,
    mut item: PassItemCtx<'_>,
) {
    let (setup_idx, op_idx) = item.ids();

    section_card(
        ui,
        ("par3d_passes_card", setup_idx, op_idx),
        "3D Parallel / Raster Finishing",
        true,
        |ui| {
            item.grid(ui, "par3d_passes_grid", |f| {
                f.set_tip("Automatically compute raster stepover from target surface cusp height and ball/corner radius");
                f.check(
                    "Cusp Stepover",
                    &mut p.use_cusp_height,
                    "Use Cusp Height",
                    "Toggle Parallel Cusp-Driven Stepover",
                );
                if p.use_cusp_height {
                    f.set_tip("Target maximum scallop/cusp height between parallel raster passes");
                    f.len(
                        "Target Cusp Height",
                        &mut p.cusp_height,
                        "Edit Parallel Cusp Height",
                    );
                } else {
                    f.set_tip("Explicit horizontal XY spacing between consecutive raster scanlines");
                    f.len("Stepover", &mut p.stepover, "Edit Parallel Stepover");
                }
                f.set_tip("Scanline orientation angle in degrees (0° = along +X, 90° = along +Y)");
                f.row("Pass Angle", |ui, mutated| {
                    if ui
                        .add(
                            egui::DragValue::new(&mut p.pass_angle_deg)
                                .speed(1.0)
                                .range(-180.0..=180.0)
                                .suffix("°"),
                        )
                        .changed()
                    {
                        *mutated = Some("Edit Parallel Pass Angle");
                    }
                });
                f.set_tip("Add a second perpendicular raster pass at Pass Angle + 90°");
                f.check(
                    "Cross-Pass (+90°)",
                    &mut p.cross_pass,
                    "Add +90° Pass",
                    "Toggle Parallel Cross-Pass",
                );
                f.set_tip("Traversal direction along consecutive raster scanlines");
                f.row("Cut Direction", |ui, mutated| {
                    let prev = p.cut_direction;
                    egui::ComboBox::from_id_salt(("par3d_dir_cb", setup_idx, op_idx))
                        .width(COMBO_COL_WIDTH)
                        .selected_text(p.cut_direction.label())
                        .show_ui(ui, |ui| {
                            for d in ParallelCutDirection::ALL {
                                ui.selectable_value(&mut p.cut_direction, d, d.label());
                            }
                        });
                    if p.cut_direction != prev {
                        *mutated = Some("Change Parallel Cut Direction");
                    }
                });
                f.set_tip("Restrict raster finishing to shallow or steep slope ranges (0° = flat, 90° = vertical)");
                f.row("Slope Range (°)", |ui, mutated| {
                    ui.horizontal(|ui| {
                        let r_min = ui.add(
                            egui::DragValue::new(&mut p.min_slope_deg)
                                .speed(1.0)
                                .range(0.0..=89.0)
                                .suffix("°"),
                        );
                        ui.label("to");
                        let r_max = ui.add(
                            egui::DragValue::new(&mut p.max_slope_deg)
                                .speed(1.0)
                                .range(p.min_slope_deg.max(1.0)..=90.0)
                                .suffix("°"),
                        );
                        if r_min.changed() || r_max.changed() {
                            if p.max_slope_deg < p.min_slope_deg {
                                p.max_slope_deg = p.min_slope_deg;
                            }
                            *mutated = Some("Edit Parallel Slope Range");
                        }
                    });
                });
                f.set_tip("Along-pass Drop-Cutter sampling pitch");
                f.len("Sample Pitch", &mut p.sample_step, "Edit Parallel Sample Pitch");
                f.set_tip("3D chord decimation tolerance keeping G-code compact while preserving smooth curves");
                f.len(
                    "Smoothing Tol",
                    &mut p.smoothing_tolerance,
                    "Edit Parallel Smoothing Tolerance",
                );
            });
        },
    );

    show_3d_boundary_card(
        ui,
        ("par3d_boundary_card", setup_idx, op_idx),
        &mut p.boundary_mode,
        &mut p.tool_containment,
        &mut p.boundary_offset,
        &mut item,
    );
}

pub(super) fn show_scallop3d_params(
    ui: &mut egui::Ui,
    p: &mut Scallop3DParams,
    mut item: PassItemCtx<'_>,
) {
    let (setup_idx, op_idx) = item.ids();

    section_card(
        ui,
        ("sc3d_passes_card", setup_idx, op_idx),
        "3D Constant-Scallop Finishing",
        true,
        |ui| {
            item.grid(ui, "sc3d_passes_grid", |f| {
                f.set_tip("Target uniform 3D scallop/cusp height across shallow crests and steep walls");
                f.len(
                    "Target Cusp Height",
                    &mut p.cusp_height,
                    "Edit Scallop Cusp Height",
                );
                f.set_tip("Maximum 3D surface arc-length stepover clamp");
                f.len(
                    "Max Stepover",
                    &mut p.max_stepover,
                    "Edit Scallop Max Stepover",
                );
                f.set_tip("Minimum 3D surface arc-length stepover clamp");
                f.len(
                    "Min Stepover",
                    &mut p.min_stepover,
                    "Edit Scallop Min Stepover",
                );
                f.set_tip("Order iso-contours from interior peaks outward instead of boundary inward");
                f.check(
                    "Ordering",
                    &mut p.inside_out,
                    "Inside -> Out",
                    "Toggle Scallop Inside-Out Ordering",
                );
                f.set_tip("Climb milling keeps workpiece material to the right of tool travel");
                f.check(
                    "Cut Direction",
                    &mut p.climb_milling,
                    "Climb Milling",
                    "Toggle Scallop Climb Milling",
                );
                f.set_tip("Restrict finishing to specific surface slope angles (0° = flat, 90° = vertical)");
                f.row("Slope Range (°)", |ui, mutated| {
                    ui.horizontal(|ui| {
                        let r_min = ui.add(
                            egui::DragValue::new(&mut p.min_slope_deg)
                                .speed(1.0)
                                .range(0.0..=89.0)
                                .suffix("°"),
                        );
                        ui.label("to");
                        let r_max = ui.add(
                            egui::DragValue::new(&mut p.max_slope_deg)
                                .speed(1.0)
                                .range(p.min_slope_deg.max(1.0)..=90.0)
                                .suffix("°"),
                        );
                        if r_min.changed() || r_max.changed() {
                            if p.max_slope_deg < p.min_slope_deg {
                                p.max_slope_deg = p.min_slope_deg;
                            }
                            *mutated = Some("Edit Scallop Slope Range");
                        }
                    });
                });
                f.set_tip("3D chord decimation tolerance keeping G-code compact");
                f.len(
                    "Smoothing Tol",
                    &mut p.smoothing_tolerance,
                    "Edit Scallop Smoothing Tolerance",
                );
            });
        },
    );

    show_3d_boundary_card(
        ui,
        ("sc3d_boundary_card", setup_idx, op_idx),
        &mut p.boundary_mode,
        &mut p.tool_containment,
        &mut p.boundary_offset,
        &mut item,
    );
}

pub(super) fn show_pencil3d_params(
    ui: &mut egui::Ui,
    p: &mut Pencil3DParams,
    mut item: PassItemCtx<'_>,
) {
    let (setup_idx, op_idx) = item.ids();

    section_card(
        ui,
        ("pen3d_passes_card", setup_idx, op_idx),
        "3D Pencil / Fillet Cleanup",
        true,
        |ui| {
            item.grid(ui, "pen3d_passes_grid", |f| {
                f.set_tip("Diameter of the larger previous/reference ball endmill used to detect unmachined concave creases");
                f.len(
                    "Reference Tool Ø",
                    &mut p.reference_tool_diameter,
                    "Edit Pencil Reference Tool Diameter",
                );
                f.set_tip("Minimum residual fillet thickness required to trigger a concave valley pass");
                f.len(
                    "Valley Threshold",
                    &mut p.valley_threshold,
                    "Edit Pencil Valley Threshold",
                );
                f.set_tip("Optional extra lateral cleanup passes offset on each side of detected valley centerlines (0 = centerline only)");
                f.row("Offset Passes", |ui, mutated| {
                    if ui
                        .add(egui::DragValue::new(&mut p.num_offset_passes).range(0..=4))
                        .changed()
                    {
                        *mutated = Some("Edit Pencil Offset Pass Count");
                    }
                });
                if p.num_offset_passes > 0 {
                    f.set_tip("Lateral stepover distance between multi-pass pencil offsets");
                    f.len(
                        "Offset Stepover",
                        &mut p.offset_stepover,
                        "Edit Pencil Offset Stepover",
                    );
                }
                f.set_tip("3D chord decimation tolerance keeping G-code compact");
                f.len(
                    "Smoothing Tol",
                    &mut p.smoothing_tolerance,
                    "Edit Pencil Smoothing Tolerance",
                );
            });
        },
    );

    show_3d_boundary_card(
        ui,
        ("pen3d_boundary_card", setup_idx, op_idx),
        &mut p.boundary_mode,
        &mut p.tool_containment,
        &mut p.boundary_offset,
        &mut item,
    );
}

fn show_3d_boundary_card(
    ui: &mut egui::Ui,
    card_id: (&'static str, usize, usize),
    boundary_mode: &mut MachiningBoundaryMode,
    tool_containment: &mut ToolContainmentMode,
    boundary_offset: &mut chip_math::ParamLength,
    item: &mut PassItemCtx<'_>,
) {
    let (setup_idx, op_idx) = item.ids();
    section_card(ui, card_id, "3D Machining Boundary", true, |ui| {
        item.grid(ui, card_id.0, |f| {
            f.set_tip("2D XY containment boundary constraining 3D roughing/waterline motion");
            f.row("Boundary Source", |ui, mutated| {
                let prev = *boundary_mode;
                egui::ComboBox::from_id_salt((card_id.0, "bmode", setup_idx, op_idx))
                    .width(COMBO_COL_WIDTH)
                    .selected_text(boundary_mode.label())
                    .show_ui(ui, |ui| {
                        for m in MachiningBoundaryMode::ALL {
                            ui.selectable_value(boundary_mode, m, m.label());
                        }
                    });
                if *boundary_mode != prev {
                    *mutated = Some("Change 3D Boundary Source");
                }
            });
            f.set_tip("How the tool perimeter is positioned relative to the 2D boundary");
            f.row("Tool Containment", |ui, mutated| {
                let prev = *tool_containment;
                egui::ComboBox::from_id_salt((card_id.0, "tcont", setup_idx, op_idx))
                    .width(COMBO_COL_WIDTH)
                    .selected_text(tool_containment.label())
                    .show_ui(ui, |ui| {
                        for c in ToolContainmentMode::ALL {
                            ui.selectable_value(tool_containment, c, c.label());
                        }
                    });
                if *tool_containment != prev {
                    *mutated = Some("Change 3D Tool Containment");
                }
            });
            f.set_tip(
                "Additional outward (+) or inward (-) XY margin applied to the machining boundary",
            );
            f.len(
                "Boundary Offset",
                boundary_offset,
                "Edit 3D Boundary Offset",
            );
        });
    });
}

pub(super) fn show_rotary2linear_params(
    ui: &mut egui::Ui,
    p: &mut Rotary2LinearParams,
    mut item: PassItemCtx<'_>,
) {
    let (setup_idx, op_idx) = item.ids();

    section_card(
        ui,
        ("rot2lin_passes_card", setup_idx, op_idx),
        "4th-Axis Rotary Surfacing (Z+X / Z+Y)",
        true,
        |ui| {
            item.grid(ui, "rot2lin_passes_grid", |f| {
                f.set_tip("Continuous helical spiral around the shaft or lengthwise axial parallel passes");
                f.combo(
                    "Pattern",
                    &mut p.pattern,
                    (&RotaryPatternMode::ALL, RotaryPatternMode::label),
                    "Change Rotary Surfacing Pattern",
                );
                f.set_tip("Pass traversal ordering (Zig-Zag minimizes retracts; Climb/Conventional are unidirectional)");
                f.combo(
                    "Cut Direction",
                    &mut p.cut_direction,
                    (&RotaryCutDirection::ALL, RotaryCutDirection::label),
                    "Change Rotary Cut Direction",
                );
                f.set_tip("Derive helical/angular stepover automatically from target ball/bull-nose cusp height");
                f.check(
                    "Stepover Mode",
                    &mut p.use_cusp_height,
                    "Use Target Cusp Height",
                    "Toggle Rotary Cusp Stepover",
                );
                if p.use_cusp_height {
                    f.set_tip("Target surface scallop/cusp height across the cylindrical profile");
                    f.len("Target Cusp", &mut p.cusp_height, "Edit Rotary Cusp Height");
                } else {
                    f.set_tip("Axial pitch per revolution (Spiral) or circumferential lane spacing (Axial Parallel)");
                    f.len("Stepover", &mut p.stepover, "Edit Rotary Stepover");
                }
                f.set_tip("Angular machining range around the rotary axis (0° to 360° = full circumference)");
                f.row("Angle Window (°)", |ui, mutated| {
                    ui.horizontal(|ui| {
                        let r0 = ui.add(
                            egui::DragValue::new(&mut p.start_angle_deg)
                                .speed(2.0)
                                .range(-360.0..=360.0)
                                .suffix("°"),
                        );
                        ui.label("to");
                        let r1 = ui.add(
                            egui::DragValue::new(&mut p.end_angle_deg)
                                .speed(2.0)
                                .range((p.start_angle_deg + 1.0)..=720.0)
                                .suffix("°"),
                        );
                        if r0.changed() || r1.changed() {
                            if p.end_angle_deg <= p.start_angle_deg + 1.0 {
                                p.end_angle_deg = p.start_angle_deg + 1.0;
                            }
                            *mutated = Some("Edit Rotary Angle Window");
                        }
                    });
                });
                f.set_tip("Inward axial trim margin from the -U stock end (keeps cutter clear of the chuck)");
                f.len(
                    "Start Margin (-U)",
                    &mut p.axial_margin_start,
                    "Edit Rotary Start Margin",
                );
                f.set_tip("Inward axial trim margin from the +U stock end (e.g. tailstock clearance)");
                f.len(
                    "End Margin (+U)",
                    &mut p.axial_margin_end,
                    "Edit Rotary End Margin",
                );
                f.set_tip("Angular Drop-Cutter sampling resolution around the rotary axis");
                f.drag(
                    "Angular Sample",
                    &mut p.angular_sample_deg,
                    0.5..=15.0,
                    (0.25, "°"),
                    "Edit Rotary Angular Sample",
                );
                f.set_tip("Axial Drop-Cutter sampling pitch along the rotary axis");
                f.len(
                    "Axial Sample",
                    &mut p.axial_sample_step,
                    "Edit Rotary Axial Sample",
                );
                f.set_tip("Chord decimation tolerance keeping G93 4-axis code smooth and compact");
                f.len(
                    "Smoothing Tol",
                    &mut p.smoothing_tolerance,
                    "Edit Rotary Smoothing Tolerance",
                );
            });
        },
    );
}
