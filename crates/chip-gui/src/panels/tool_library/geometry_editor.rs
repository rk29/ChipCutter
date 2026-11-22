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

    if let CutterGeometry::ThreadMill {
        neck_diameter_mm,
        neck_length_mm,
        pitch_mm,
        teeth_count,
        ..
    } = tool.geometry
    {
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
