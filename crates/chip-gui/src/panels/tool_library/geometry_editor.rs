use chip_math::DisplayUnit;
use chip_tools::{CutterGeometry, Tool};

pub(super) fn show_cutter_geometry_editor(ui: &mut egui::Ui, tool: &mut Tool, unit: DisplayUnit) {
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
                .on_hover_text("Square-corner flat-bottom milling cutter")
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
                    matches!(tool.geometry, CutterGeometry::BullNoseEndmill { .. }),
                    "Bull-Nose Endmill",
                )
                .on_hover_text("Flat-bottom endmill with a defined corner fillet radius")
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
                .on_hover_text("Hemispherical ball-nose cutter for 3D surface finishing")
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
                .on_hover_text(
                    "Conical V-bit or chamfer cutter for bevels, spot drilling, and V-carving",
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
                .on_hover_text("Pointed twist drill for axial drilling and peck cycles")
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
                .on_hover_text("Large-diameter indexable face mill or fly cutter for stock facing")
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
            if ui
                .selectable_label(
                    matches!(tool.geometry, CutterGeometry::ThreadMill { .. }),
                    "Thread Mill",
                )
                .on_hover_text(
                    "Relieved-neck single-point or multi-form helical thread mill cutter",
                )
                .clicked()
            {
                let tm_d = d.max(1.0);
                let tm_oal = oal.max(25.0);
                let tm_fl = 1.25_f64.min(0.25 * tm_oal).max(0.5);
                let tm_neck_len = fl.max(tm_fl + 4.0).min(tm_oal - 4.0).max(tm_fl + 0.5);
                let tm_neck_d = (0.60 * tm_d).max(0.4).min(tm_d - 0.1);
                tool.geometry = CutterGeometry::ThreadMill {
                    diameter_mm: tm_d,
                    neck_diameter_mm: tm_neck_d,
                    neck_length_mm: tm_neck_len,
                    pitch_mm: 1.0,
                    teeth_count: 1,
                    thread_angle_deg: 60.0,
                    flute_length_mm: tm_fl,
                    overall_length_mm: tm_oal,
                    shank_diameter_mm: shk.max(tm_neck_d),
                    flutes: n.clamp(1, 12),
                };
            }
            if ui
                .selectable_label(
                    matches!(tool.geometry, CutterGeometry::LollipopEndmill { .. }),
                    "Lollipop Endmill",
                )
                .on_hover_text(
                    "Spherical ball head on a relieved slim neck for 3D underside deburring and concave undercuts",
                )
                .clicked()
            {
                let lp_d = d.max(2.0);
                let lp_neck_d = (0.50 * lp_d).max(0.5).min(lp_d - 0.2);
                let lp_fl = CutterGeometry::lollipop_sphere_height_mm(lp_d, lp_neck_d);
                let lp_oal = oal.max(lp_fl + 18.0);
                let lp_neck_len = fl.max(lp_fl + 8.0).min(lp_oal - 4.0).max(lp_fl + 0.5);
                tool.geometry = CutterGeometry::LollipopEndmill {
                    diameter_mm: lp_d,
                    neck_diameter_mm: lp_neck_d,
                    neck_length_mm: lp_neck_len,
                    flute_length_mm: lp_fl,
                    overall_length_mm: lp_oal,
                    shank_diameter_mm: shk.max(lp_neck_d),
                    flutes: n.clamp(2, 12),
                };
                tool.stickout_length_mm = tool.stickout_length_mm.max(lp_neck_len + 2.0);
            }
            if ui
                .selectable_label(
                    matches!(tool.geometry, CutterGeometry::SlotMill { .. }),
                    "Slot / T-Slot Mill",
                )
                .on_hover_text(
                    "Horizontal Woodruff / keyway / T-slot disc cutter with relieved cylindrical neck",
                )
                .clicked()
            {
                let sm_d = d.max(6.0);
                let sm_thick = (0.33 * sm_d).clamp(1.5, 6.0);
                let sm_neck_d = (0.42 * sm_d).max(1.5).min(sm_d - 0.5);
                let sm_oal = oal.max(sm_thick + 22.0);
                let sm_neck_len = fl.max(sm_thick + 10.0).min(sm_oal - 4.0).max(sm_thick + 0.5);
                tool.geometry = CutterGeometry::SlotMill {
                    diameter_mm: sm_d,
                    cutting_thickness_mm: sm_thick,
                    corner_radius_mm: (0.05 * sm_thick).min(0.25),
                    neck_diameter_mm: sm_neck_d,
                    neck_length_mm: sm_neck_len,
                    overall_length_mm: sm_oal,
                    shank_diameter_mm: shk.max(sm_neck_d),
                    flutes: n.clamp(2, 12),
                };
                tool.stickout_length_mm = tool.stickout_length_mm.max(sm_neck_len + 2.0);
            }
            if ui
                .selectable_label(
                    matches!(tool.geometry, CutterGeometry::DovetailCutter { .. }),
                    "Dovetail Cutter",
                )
                .on_hover_text(
                    "Inverted tapered frustum cutter with relieved neck for angled dovetail rails and slots",
                )
                .clicked()
            {
                let dt_d = d.max(6.0);
                let dt_angle = 60.0_f64;
                let dt_h = (0.40 * dt_d).clamp(2.0, 8.0);
                let dt_rc = (0.04 * dt_h).min(0.2);
                let dt_top_d =
                    CutterGeometry::dovetail_top_diameter_mm(dt_d, dt_angle, dt_h, dt_rc)
                        .max(1.0);
                let dt_neck_d = (0.88 * dt_top_d).max(1.0).min(dt_top_d - 0.05);
                let dt_oal = oal.max(dt_h + 22.0);
                let dt_neck_len = fl.max(dt_h + 10.0).min(dt_oal - 4.0).max(dt_h + 0.5);
                tool.geometry = CutterGeometry::DovetailCutter {
                    diameter_mm: dt_d,
                    side_angle_deg: dt_angle,
                    cutting_height_mm: dt_h,
                    corner_radius_mm: dt_rc,
                    neck_diameter_mm: dt_neck_d,
                    neck_length_mm: dt_neck_len,
                    overall_length_mm: dt_oal,
                    shank_diameter_mm: shk.max(dt_neck_d),
                    flutes: n.clamp(2, 12),
                };
                tool.stickout_length_mm = tool.stickout_length_mm.max(dt_neck_len + 2.0);
            }
        })
        .response
        .on_hover_text("Select physical cutter profile geometry");

    edit_cutter_geometry_fields(ui, &mut tool.geometry, unit);
    edit_mm_drag(ui, "Holder Stickout:", &mut tool.stickout_length_mm, unit);
}

pub(super) fn edit_offset_number_row(
    ui: &mut egui::Ui,
    label: &str,
    tooltip: &str,
    value: &mut u32,
) {
    ui.add(egui::Label::new(label).sense(egui::Sense::hover()))
        .on_hover_text(tooltip);
    ui.add(egui::DragValue::new(value).range(1..=999))
        .on_hover_text(tooltip);
    ui.end_row();
}

pub(super) fn edit_mm_drag(ui: &mut egui::Ui, label: &str, val_mm: &mut f64, unit: DisplayUnit) {
    let tip = format!(
        "Edit {} in current tool units ({})",
        label.trim_end_matches(':'),
        unit.suffix()
    );
    edit_mm_drag_with_tip(ui, label, &tip, val_mm, unit);
}

fn edit_mm_drag_with_tip(
    ui: &mut egui::Ui,
    label: &str,
    tooltip: &str,
    val_mm: &mut f64,
    unit: DisplayUnit,
) {
    let mut disp = unit.from_mm(*val_mm);
    ui.horizontal(|ui| {
        ui.add(egui::Label::new(label).sense(egui::Sense::hover()))
            .on_hover_text(tooltip);
        if ui
            .add(
                egui::DragValue::new(&mut disp)
                    .speed(0.05)
                    .max_decimals(4)
                    .range(0.0..=1_000.0)
                    .suffix(format!(" {}", unit.suffix())),
            )
            .on_hover_text(tooltip)
            .changed()
        {
            *val_mm = unit.to_mm(disp);
        }
    });
}

fn edit_mm_grid_row(
    ui: &mut egui::Ui,
    label: &str,
    tooltip: &str,
    val_mm: &mut f64,
    unit: DisplayUnit,
) {
    let mut disp = unit.from_mm(*val_mm);
    ui.add(egui::Label::new(label).sense(egui::Sense::hover()))
        .on_hover_text(tooltip);
    if ui
        .add(
            egui::DragValue::new(&mut disp)
                .speed(0.05)
                .max_decimals(4)
                .range(0.0..=1_000.0)
                .suffix(format!(" {}", unit.suffix())),
        )
        .on_hover_text(tooltip)
        .changed()
    {
        *val_mm = unit.to_mm(disp);
    }
    ui.end_row();
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
                ui.add(egui::DragValue::new(flutes).range(1..=24))
                    .on_hover_text("Number of cutting flutes on the tool");
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
                ui.add(egui::DragValue::new(flutes).range(1..=24))
                    .on_hover_text("Number of cutting flutes on the bull-nose cutter");
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
                )
                .on_hover_text("Full included cone angle of the chamfer mill or V-bit (e.g., 60 or 90 degrees)");
            });
            edit_mm_drag(ui, "Flute Length:", flute_length_mm, unit);
            edit_mm_drag(ui, "Overall Length:", overall_length_mm, unit);
            edit_mm_drag(ui, "Shank Diameter:", shank_diameter_mm, unit);
            ui.horizontal(|ui| {
                ui.label("Flutes:");
                ui.add(egui::DragValue::new(flutes).range(1..=24))
                    .on_hover_text("Number of cutting flutes on the chamfer/V-bit cutter");
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
                )
                .on_hover_text("Drill tip point angle (commonly 118 or 135 degrees)");
            });
            edit_mm_drag(ui, "Flute Length:", flute_length_mm, unit);
            edit_mm_drag(ui, "Overall Length:", overall_length_mm, unit);
            edit_mm_drag(ui, "Shank Diameter:", shank_diameter_mm, unit);
            ui.horizontal(|ui| {
                ui.label("Flutes:");
                ui.add(egui::DragValue::new(flutes).range(1..=8))
                    .on_hover_text("Number of flutes on the drill bit");
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
                ui.add(egui::DragValue::new(flutes).range(1..=16))
                    .on_hover_text("Number of carbide inserts or cutting teeth on the face mill");
            });
        }
        CutterGeometry::ThreadMill {
            diameter_mm,
            neck_diameter_mm,
            neck_length_mm,
            pitch_mm,
            teeth_count,
            thread_angle_deg,
            flute_length_mm,
            overall_length_mm,
            shank_diameter_mm,
            flutes,
        } => {
            edit_mm_drag_with_tip(
                ui,
                "Cutting Diameter:",
                "Outer crest cutting diameter of the thread mill teeth (must be smaller than internal minor diameter)",
                diameter_mm,
                unit,
            );
            edit_mm_drag_with_tip(
                ui,
                "Neck Diameter:",
                "Relieved slim neck diameter behind the cutting teeth for deep internal bore clearance",
                neck_diameter_mm,
                unit,
            );
            edit_mm_drag_with_tip(
                ui,
                "Neck Length (Reach):",
                "Usable relieved reach from tip to shank taper (maximum thread depth without shank collision)",
                neck_length_mm,
                unit,
            );
            edit_mm_drag_with_tip(
                ui,
                "Cutter Pitch:",
                "Nominal thread pitch spacing of the cutter form (multi-form cutters must match target pitch within 0.02 mm)",
                pitch_mm,
                unit,
            );
            ui.horizontal(|ui| {
                let teeth_tip = "Number of axial thread teeth: 1 = Single-Point (cuts any pitch), >1 = Multi-Form (cuts full depth in fewer turns)";
                ui.add(egui::Label::new("Teeth Count:").sense(egui::Sense::hover()))
                    .on_hover_text(teeth_tip);
                ui.add(egui::DragValue::new(teeth_count).range(1..=32))
                    .on_hover_text(teeth_tip);
                let mode_tag = if *teeth_count <= 1 {
                    "(Single-Point)"
                } else {
                    "(Multi-Form)"
                };
                ui.weak(mode_tag).on_hover_text(teeth_tip);
            });
            ui.horizontal(|ui| {
                let ang_tip =
                    "Included thread flank angle (60 deg for ISO Metric and Unified UNC/UNF)";
                ui.add(egui::Label::new("Thread Angle:").sense(egui::Sense::hover()))
                    .on_hover_text(ang_tip);
                ui.add(
                    egui::DragValue::new(thread_angle_deg)
                        .range(29.0..=90.0)
                        .suffix("°"),
                )
                .on_hover_text(ang_tip);
            });
            edit_mm_drag_with_tip(
                ui,
                "Flute Length:",
                "Axial length of the cutting tooth band at the cutter tip (must be <= Neck Length)",
                flute_length_mm,
                unit,
            );
            edit_mm_drag_with_tip(
                ui,
                "Overall Length:",
                "Total overall length (OAL) of the thread mill from tip to shank end",
                overall_length_mm,
                unit,
            );
            edit_mm_drag_with_tip(
                ui,
                "Shank Diameter:",
                "Cylindrical collet/holder shank diameter",
                shank_diameter_mm,
                unit,
            );
            ui.horizontal(|ui| {
                let fl_tip = "Number of radial cutting flutes around the thread mill circumference";
                ui.add(egui::Label::new("Flutes:").sense(egui::Sense::hover()))
                    .on_hover_text(fl_tip);
                ui.add(egui::DragValue::new(flutes).range(1..=12))
                    .on_hover_text(fl_tip);
            });
        }
        CutterGeometry::LollipopEndmill {
            diameter_mm,
            neck_diameter_mm,
            neck_length_mm,
            flute_length_mm,
            overall_length_mm,
            shank_diameter_mm,
            flutes,
        } => {
            egui::Grid::new("lollipop_geom_grid")
                .num_columns(2)
                .spacing([10.0, 4.0])
                .show(ui, |ui| {
                    edit_mm_grid_row(
                        ui,
                        "Cutting Diameter:",
                        "Full spherical cutting head diameter (D) of the lollipop endmill",
                        diameter_mm,
                        unit,
                    );
                    edit_mm_grid_row(
                        ui,
                        "Neck Diameter:",
                        "Relieved cylindrical neck diameter behind the spherical head (must be < Cutting Diameter)",
                        neck_diameter_mm,
                        unit,
                    );
                    edit_mm_grid_row(
                        ui,
                        "Neck Length (Reach):",
                        "Usable relieved reach from tip to shank taper (must be >= Flute Height and < Overall Length)",
                        neck_length_mm,
                        unit,
                    );
                    edit_mm_grid_row(
                        ui,
                        "Flute Height:",
                        "Axial height of the spherical cutting head up to the neck join (defaults to R + sqrt(R^2 - r_neck^2))",
                        flute_length_mm,
                        unit,
                    );
                    let join_h =
                        CutterGeometry::lollipop_sphere_height_mm(*diameter_mm, *neck_diameter_mm);
                    let join_tip = "Analytic height where the spherical head intersects the relieved neck cylinder";
                    ui.add(egui::Label::new("Sphere Join Height:").sense(egui::Sense::hover()))
                        .on_hover_text(join_tip);
                    ui.horizontal(|ui| {
                        ui.weak(unit.format_length(join_h)).on_hover_text(join_tip);
                        if ui
                            .small_button("Sync")
                            .on_hover_text("Set Flute Height to the exact analytic sphere-to-neck join height")
                            .clicked()
                        {
                            *flute_length_mm = join_h;
                        }
                    });
                    ui.end_row();
                    edit_mm_grid_row(
                        ui,
                        "Overall Length:",
                        "Total overall length (OAL) of the lollipop endmill from tip to shank end",
                        overall_length_mm,
                        unit,
                    );
                    edit_mm_grid_row(
                        ui,
                        "Shank Diameter:",
                        "Cylindrical collet/holder shank diameter",
                        shank_diameter_mm,
                        unit,
                    );
                    let fl_tip = "Number of cutting flutes around the spherical lollipop head";
                    ui.add(egui::Label::new("Flutes:").sense(egui::Sense::hover()))
                        .on_hover_text(fl_tip);
                    ui.add(egui::DragValue::new(flutes).range(1..=12))
                        .on_hover_text(fl_tip);
                    ui.end_row();
                });
        }
        CutterGeometry::SlotMill {
            diameter_mm,
            cutting_thickness_mm,
            corner_radius_mm,
            neck_diameter_mm,
            neck_length_mm,
            overall_length_mm,
            shank_diameter_mm,
            flutes,
        } => {
            egui::Grid::new("slot_mill_geom_grid")
                .num_columns(2)
                .spacing([10.0, 4.0])
                .show(ui, |ui| {
                    edit_mm_grid_row(
                        ui,
                        "Cutting Diameter:",
                        "Outer cutting disc diameter (D) of the horizontal T-slot / keyway mill",
                        diameter_mm,
                        unit,
                    );
                    edit_mm_grid_row(
                        ui,
                        "Cutting Thickness:",
                        "Axial disc cutting height / kerf width (H_cut) of the slot cutter",
                        cutting_thickness_mm,
                        unit,
                    );
                    edit_mm_grid_row(
                        ui,
                        "Corner Radius:",
                        "Top and bottom disc edge fillet radius (must be <= 0.5 * Cutting Thickness)",
                        corner_radius_mm,
                        unit,
                    );
                    edit_mm_grid_row(
                        ui,
                        "Neck Diameter:",
                        "Relieved cylindrical neck diameter above the cutting disc (must clear the pre-cut slot)",
                        neck_diameter_mm,
                        unit,
                    );
                    edit_mm_grid_row(
                        ui,
                        "Neck Length (Reach):",
                        "Usable relieved reach from tip to shank taper (must be >= Cutting Thickness and < Overall Length)",
                        neck_length_mm,
                        unit,
                    );
                    edit_mm_grid_row(
                        ui,
                        "Overall Length:",
                        "Total overall length (OAL) of the slot mill from tip to shank end",
                        overall_length_mm,
                        unit,
                    );
                    edit_mm_grid_row(
                        ui,
                        "Shank Diameter:",
                        "Cylindrical collet/holder shank diameter",
                        shank_diameter_mm,
                        unit,
                    );
                    let fl_tip = "Number of cutting teeth / flutes around the slot disc perimeter";
                    ui.add(egui::Label::new("Flutes:").sense(egui::Sense::hover()))
                        .on_hover_text(fl_tip);
                    ui.add(egui::DragValue::new(flutes).range(1..=16))
                        .on_hover_text(fl_tip);
                    ui.end_row();
                });
        }
        CutterGeometry::DovetailCutter {
            diameter_mm,
            side_angle_deg,
            cutting_height_mm,
            corner_radius_mm,
            neck_diameter_mm,
            neck_length_mm,
            overall_length_mm,
            shank_diameter_mm,
            flutes,
        } => {
            egui::Grid::new("dovetail_geom_grid")
                .num_columns(2)
                .spacing([10.0, 4.0])
                .show(ui, |ui| {
                    edit_mm_grid_row(
                        ui,
                        "Cutting Diameter:",
                        "Major bottom cutting diameter (D_bot) at the widest end of the dovetail cutter",
                        diameter_mm,
                        unit,
                    );
                    let ang_tip = "Dovetail flank angle measured from the horizontal floor (typically 45° or 60°, valid range 15°..85°)";
                    ui.add(egui::Label::new("Side Angle:").sense(egui::Sense::hover()))
                        .on_hover_text(ang_tip);
                    ui.add(
                        egui::DragValue::new(side_angle_deg)
                            .speed(0.5)
                            .range(15.5..=84.5)
                            .suffix("°"),
                    )
                    .on_hover_text(ang_tip);
                    ui.end_row();
                    edit_mm_grid_row(
                        ui,
                        "Cutting Height:",
                        "Axial height of the inverted conical dovetail flutes (H_cut)",
                        cutting_height_mm,
                        unit,
                    );
                    edit_mm_grid_row(
                        ui,
                        "Corner Radius:",
                        "Optional bottom outer corner radius (must be <= 0.5 * Cutting Height)",
                        corner_radius_mm,
                        unit,
                    );
                    let top_d = CutterGeometry::dovetail_top_diameter_mm(
                        *diameter_mm,
                        *side_angle_deg,
                        *cutting_height_mm,
                        *corner_radius_mm,
                    );
                    let top_tip = "Derived minor diameter at the top of the dovetail cutting cone (Neck Diameter must be <= Top Ø)";
                    ui.add(egui::Label::new("Top Ø (Derived):").sense(egui::Sense::hover()))
                        .on_hover_text(top_tip);
                    let tok = crate::theme::ThemeTokens::from_ui(ui);
                    let top_text = if top_d > 0.0 {
                        unit.format_length(top_d)
                    } else {
                        "Invalid (<= 0)".to_string()
                    };
                    let top_col = if top_d <= 1e-4 || *neck_diameter_mm > top_d + 1e-6 {
                        tok.error_fg
                    } else {
                        tok.info_fg
                    };
                    ui.colored_label(top_col, top_text).on_hover_text(top_tip);
                    ui.end_row();
                    edit_mm_grid_row(
                        ui,
                        "Neck Diameter:",
                        "Relieved cylindrical neck diameter above the dovetail cone (must be <= derived Top Ø)",
                        neck_diameter_mm,
                        unit,
                    );
                    edit_mm_grid_row(
                        ui,
                        "Neck Length (Reach):",
                        "Usable relieved reach from tip to shank taper (must be >= Cutting Height and < Overall Length)",
                        neck_length_mm,
                        unit,
                    );
                    edit_mm_grid_row(
                        ui,
                        "Overall Length:",
                        "Total overall length (OAL) of the dovetail cutter from tip to shank end",
                        overall_length_mm,
                        unit,
                    );
                    edit_mm_grid_row(
                        ui,
                        "Shank Diameter:",
                        "Cylindrical collet/holder shank diameter",
                        shank_diameter_mm,
                        unit,
                    );
                    let fl_tip = "Number of cutting flutes around the dovetail cutter";
                    ui.add(egui::Label::new("Flutes:").sense(egui::Sense::hover()))
                        .on_hover_text(fl_tip);
                    ui.add(egui::DragValue::new(flutes).range(1..=12))
                        .on_hover_text(fl_tip);
                    ui.end_row();
                });
        }
    }
}

pub(super) fn draw_cutter_schematic(ui: &mut egui::Ui, tool: &Tool) {
    let tok = crate::theme::ThemeTokens::from_ui(ui);
    let (rect, _) = ui.allocate_exact_size(egui::vec2(260.0, 128.0), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect(
        rect,
        4.0,
        tok.code_bg,
        egui::Stroke::new(1.0, tok.border),
        egui::StrokeKind::Inside,
    );

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

    // Fluted cutting body (gold/amber carbide tint)
    let shank_col = egui::Color32::from_rgb(122, 130, 142);
    let flute_col = egui::Color32::from_rgb(232, 182, 68);

    match tool.geometry {
        CutterGeometry::ThreadMill {
            neck_diameter_mm,
            neck_length_mm,
            pitch_mm,
            teeth_count,
            ..
        } => {
            draw_thread_mill_schematic(
                &painter,
                ThreadMillSchematicSpec {
                    cx,
                    top_y,
                    tip_y,
                    scale,
                    oal,
                    flute_len: fl,
                    r_shk,
                    r_cut,
                    neck_diameter_mm: neck_diameter_mm as f32,
                    neck_length_mm: neck_length_mm as f32,
                    pitch_mm: pitch_mm as f32,
                    teeth_count,
                    shank_col,
                    flute_col,
                },
            );
            return;
        }
        CutterGeometry::LollipopEndmill {
            diameter_mm,
            neck_diameter_mm,
            neck_length_mm,
            ..
        } => {
            draw_lollipop_schematic(
                &painter,
                UndercutSchematicBase {
                    cx,
                    top_y,
                    tip_y,
                    scale,
                    oal,
                    r_shk,
                    r_cut,
                    neck_diameter_mm: neck_diameter_mm as f32,
                    neck_length_mm: neck_length_mm as f32,
                    shank_col,
                    flute_col,
                },
                diameter_mm as f32,
            );
            return;
        }
        CutterGeometry::SlotMill {
            cutting_thickness_mm,
            corner_radius_mm,
            neck_diameter_mm,
            neck_length_mm,
            ..
        } => {
            draw_slot_mill_schematic(
                &painter,
                UndercutSchematicBase {
                    cx,
                    top_y,
                    tip_y,
                    scale,
                    oal,
                    r_shk,
                    r_cut,
                    neck_diameter_mm: neck_diameter_mm as f32,
                    neck_length_mm: neck_length_mm as f32,
                    shank_col,
                    flute_col,
                },
                cutting_thickness_mm as f32,
                corner_radius_mm as f32,
            );
            return;
        }
        CutterGeometry::DovetailCutter {
            diameter_mm,
            side_angle_deg,
            cutting_height_mm,
            corner_radius_mm,
            neck_diameter_mm,
            neck_length_mm,
            ..
        } => {
            let top_d = CutterGeometry::dovetail_top_diameter_mm(
                diameter_mm,
                side_angle_deg,
                cutting_height_mm,
                corner_radius_mm,
            )
            .max(neck_diameter_mm) as f32;
            draw_dovetail_schematic(
                &painter,
                UndercutSchematicBase {
                    cx,
                    top_y,
                    tip_y,
                    scale,
                    oal,
                    r_shk,
                    r_cut,
                    neck_diameter_mm: neck_diameter_mm as f32,
                    neck_length_mm: neck_length_mm as f32,
                    shank_col,
                    flute_col,
                },
                cutting_height_mm as f32,
                top_d,
            );
            return;
        }
        _ => {}
    }

    // Shank rectangle (steel gray)
    let shank_rect = egui::Rect::from_min_max(
        egui::pos2(cx - r_shk, top_y),
        egui::pos2(cx + r_shk, shank_bot_y),
    );
    painter.rect_filled(shank_rect, 1.0, shank_col);

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

struct ThreadMillSchematicSpec {
    cx: f32,
    top_y: f32,
    tip_y: f32,
    scale: f32,
    oal: f32,
    flute_len: f32,
    r_shk: f32,
    r_cut: f32,
    neck_diameter_mm: f32,
    neck_length_mm: f32,
    pitch_mm: f32,
    teeth_count: u32,
    shank_col: egui::Color32,
    flute_col: egui::Color32,
}

fn draw_thread_mill_schematic(painter: &egui::Painter, s: ThreadMillSchematicSpec) {
    let total_h = (s.tip_y - s.top_y).max(16.0);
    let neck_len = s.neck_length_mm.clamp(s.flute_len.max(0.5), s.oal - 1.5);
    let r_cut = s.r_cut.max(2.5);
    let r_neck = (0.5 * s.neck_diameter_mm.max(0.4) * s.scale)
        .max(1.2)
        .min((r_cut - 0.6).max(1.0));
    let r_root = r_neck.min(r_cut * 0.72);

    let neck_top_y = (s.top_y + (s.oal - neck_len) * s.scale).clamp(s.top_y + 6.0, s.tip_y - 6.0);
    let taper_h = (s.r_shk - r_neck)
        .abs()
        .clamp(2.5, 8.0)
        .min((neck_top_y - s.top_y) * 0.45);
    let shank_end_y = (neck_top_y - taper_h).max(s.top_y + 3.0);

    // 1. Cylindrical shank
    painter.rect_filled(
        egui::Rect::from_min_max(
            egui::pos2(s.cx - s.r_shk, s.top_y),
            egui::pos2(s.cx + s.r_shk, shank_end_y),
        ),
        1.0,
        s.shank_col,
    );

    // 2. Smooth shank-to-neck transition taper
    painter.add(egui::Shape::convex_polygon(
        vec![
            egui::pos2(s.cx - s.r_shk, shank_end_y),
            egui::pos2(s.cx + s.r_shk, shank_end_y),
            egui::pos2(s.cx + r_neck, neck_top_y),
            egui::pos2(s.cx - r_neck, neck_top_y),
        ],
        s.shank_col,
        egui::Stroke::NONE,
    ));

    // 3. Cutting tooth band height (ensure crisp readability for both single-point and multi-form)
    let teeth = s.teeth_count.clamp(1, 12) as usize;
    let phys_band_h = (s.flute_len.max(s.pitch_mm * (teeth as f32))) * s.scale;
    let min_band_h = if teeth == 1 {
        5.0_f32
    } else {
        (teeth as f32 * 3.4).min(total_h * 0.42)
    };
    let max_band_h = (s.tip_y - neck_top_y).max(4.5);
    let band_h = phys_band_h.max(min_band_h).min(max_band_h);
    let cut_top_y = (s.tip_y - band_h).max(neck_top_y);

    // 4. Relieved cylindrical neck
    let neck_col = egui::Color32::from_rgb(164, 172, 184);
    if cut_top_y > neck_top_y + 0.5 {
        painter.rect_filled(
            egui::Rect::from_min_max(
                egui::pos2(s.cx - r_neck, neck_top_y),
                egui::pos2(s.cx + r_neck, cut_top_y),
            ),
            0.0,
            neck_col,
        );
    }

    // 5. Core body behind V-teeth
    painter.rect_filled(
        egui::Rect::from_min_max(
            egui::pos2(s.cx - r_root, cut_top_y),
            egui::pos2(s.cx + r_root, s.tip_y),
        ),
        0.0,
        s.flute_col,
    );

    // 6. Sharp V-tooth silhouette(s)
    let tooth_h = (s.tip_y - cut_top_y) / (teeth as f32);
    let crest_stroke = egui::Stroke::new(0.8, egui::Color32::from_rgb(150, 108, 24));
    for i in 0..teeth {
        let y0 = cut_top_y + (i as f32) * tooth_h;
        let y1 = y0 + tooth_h;
        let ym = 0.5 * (y0 + y1);
        painter.add(egui::Shape::convex_polygon(
            vec![
                egui::pos2(s.cx - r_root, y0),
                egui::pos2(s.cx + r_root, y0),
                egui::pos2(s.cx + r_cut, ym),
                egui::pos2(s.cx + r_root, y1),
                egui::pos2(s.cx - r_root, y1),
                egui::pos2(s.cx - r_cut, ym),
            ],
            s.flute_col,
            egui::Stroke::NONE,
        ));
        painter.line_segment(
            [egui::pos2(s.cx - r_cut, ym), egui::pos2(s.cx + r_cut, ym)],
            crest_stroke,
        );
    }
}

struct UndercutSchematicBase {
    cx: f32,
    top_y: f32,
    tip_y: f32,
    scale: f32,
    oal: f32,
    r_shk: f32,
    r_cut: f32,
    neck_diameter_mm: f32,
    neck_length_mm: f32,
    shank_col: egui::Color32,
    flute_col: egui::Color32,
}

/// Draws the upper cylindrical shank, transition taper, and relieved cylindrical neck down to `head_top_y`.
/// Returns `(r_neck_px, neck_top_y)`.
fn draw_relieved_shank_and_neck(
    painter: &egui::Painter,
    b: &UndercutSchematicBase,
    head_top_y: f32,
) -> (f32, f32) {
    let r_cut = b.r_cut.max(3.0);
    let r_neck = (0.5 * b.neck_diameter_mm.max(0.4) * b.scale)
        .max(1.4)
        .min((r_cut - 0.8).max(1.2));
    let neck_len = b.neck_length_mm.clamp(2.0, b.oal - 1.5);
    let neck_top_y = (b.top_y + (b.oal - neck_len) * b.scale)
        .clamp(b.top_y + 8.0, (head_top_y - 4.0).max(b.top_y + 8.0));
    let taper_h = (b.r_shk - r_neck)
        .abs()
        .clamp(2.5, 8.0)
        .min((neck_top_y - b.top_y) * 0.45);
    let shank_end_y = (neck_top_y - taper_h).max(b.top_y + 3.0);

    // 1. Cylindrical shank
    painter.rect_filled(
        egui::Rect::from_min_max(
            egui::pos2(b.cx - b.r_shk, b.top_y),
            egui::pos2(b.cx + b.r_shk, shank_end_y),
        ),
        1.0,
        b.shank_col,
    );

    // 2. Smooth shank-to-neck transition taper
    painter.add(egui::Shape::convex_polygon(
        vec![
            egui::pos2(b.cx - b.r_shk, shank_end_y),
            egui::pos2(b.cx + b.r_shk, shank_end_y),
            egui::pos2(b.cx + r_neck, neck_top_y),
            egui::pos2(b.cx - r_neck, neck_top_y),
        ],
        b.shank_col,
        egui::Stroke::NONE,
    ));

    // 3. Relieved cylindrical neck down to the cutting head top
    let neck_col = egui::Color32::from_rgb(164, 172, 184);
    if head_top_y > neck_top_y + 0.5 {
        painter.rect_filled(
            egui::Rect::from_min_max(
                egui::pos2(b.cx - r_neck, neck_top_y),
                egui::pos2(b.cx + r_neck, head_top_y),
            ),
            0.0,
            neck_col,
        );
    }

    (r_neck, neck_top_y)
}

fn draw_lollipop_schematic(painter: &egui::Painter, b: UndercutSchematicBase, _diameter_mm: f32) {
    let r_ball = b.r_cut.clamp(5.0, 26.0);
    let ball_cy = b.tip_y - r_ball;
    let r_neck_est = (0.5 * b.neck_diameter_mm.max(0.4) * b.scale)
        .max(1.4)
        .min((r_ball - 1.0).max(1.2));
    let join_dy = (r_ball * r_ball - r_neck_est * r_neck_est).max(0.0).sqrt();
    let head_top_y = (ball_cy - join_dy).max(b.top_y + 14.0);

    let _ = draw_relieved_shank_and_neck(painter, &b, head_top_y + 1.0);

    // Spherical cutting ball head + subtle equatorial contour highlight
    painter.circle_filled(egui::pos2(b.cx, ball_cy), r_ball, b.flute_col);
    let accent = egui::Stroke::new(0.9, egui::Color32::from_rgb(156, 112, 24));
    painter.circle_stroke(egui::pos2(b.cx, ball_cy), r_ball, accent);
    painter.line_segment(
        [
            egui::pos2(b.cx - r_ball * 0.92, ball_cy),
            egui::pos2(b.cx + r_ball * 0.92, ball_cy),
        ],
        egui::Stroke::new(0.7, egui::Color32::from_rgb(184, 136, 36)),
    );
}

fn draw_slot_mill_schematic(
    painter: &egui::Painter,
    b: UndercutSchematicBase,
    cutting_thickness_mm: f32,
    corner_radius_mm: f32,
) {
    let r_disc = b.r_cut.clamp(6.0, 42.0);
    let disc_h = (cutting_thickness_mm.max(0.5) * b.scale).clamp(6.0, 28.0);
    let disc_top_y = (b.tip_y - disc_h).max(b.top_y + 16.0);
    let corner_px = (corner_radius_mm.max(0.0) * b.scale).min(0.45 * disc_h);

    let _ = draw_relieved_shank_and_neck(painter, &b, disc_top_y);

    // Horizontal cutting disc
    let disc_rect = egui::Rect::from_min_max(
        egui::pos2(b.cx - r_disc, disc_top_y),
        egui::pos2(b.cx + r_disc, b.tip_y),
    );
    let accent = egui::Stroke::new(0.9, egui::Color32::from_rgb(156, 112, 24));
    painter.rect(
        disc_rect,
        corner_px,
        b.flute_col,
        accent,
        egui::StrokeKind::Inside,
    );

    // Subtle tooth-flute hatch marks on the cutting disc perimeter
    let flute_stroke = egui::Stroke::new(0.8, egui::Color32::from_rgb(176, 130, 32));
    for frac in [-0.65_f32, -0.22, 0.22, 0.65] {
        let x = b.cx + frac * r_disc;
        painter.line_segment(
            [
                egui::pos2(x, disc_top_y + 1.5),
                egui::pos2(x, b.tip_y - 1.5),
            ],
            flute_stroke,
        );
    }
}

fn draw_dovetail_schematic(
    painter: &egui::Painter,
    b: UndercutSchematicBase,
    cutting_height_mm: f32,
    top_diameter_mm: f32,
) {
    let r_bot = b.r_cut.clamp(6.0, 42.0);
    let head_h = (cutting_height_mm.max(0.5) * b.scale).clamp(8.0, 34.0);
    let head_top_y = (b.tip_y - head_h).max(b.top_y + 16.0);

    let (r_neck, _) = draw_relieved_shank_and_neck(painter, &b, head_top_y);
    let r_top = (0.5 * top_diameter_mm.max(0.4) * b.scale)
        .max(r_neck)
        .min((r_bot - 1.2).max(r_neck));

    // Inverted conical dovetail frustum (narrower at top, flaring outward to major bottom diameter)
    let pts = vec![
        egui::pos2(b.cx - r_top, head_top_y),
        egui::pos2(b.cx + r_top, head_top_y),
        egui::pos2(b.cx + r_bot, b.tip_y),
        egui::pos2(b.cx - r_bot, b.tip_y),
    ];
    let accent = egui::Stroke::new(0.9, egui::Color32::from_rgb(156, 112, 24));
    painter.add(egui::Shape::convex_polygon(pts, b.flute_col, accent));

    // Subtle angled flute ridge lines
    let flute_stroke = egui::Stroke::new(0.75, egui::Color32::from_rgb(176, 130, 32));
    for frac in [-0.45_f32, 0.45] {
        painter.line_segment(
            [
                egui::pos2(b.cx + frac * r_top, head_top_y + 1.0),
                egui::pos2(b.cx + frac * r_bot, b.tip_y - 1.0),
            ],
            flute_stroke,
        );
    }
}
