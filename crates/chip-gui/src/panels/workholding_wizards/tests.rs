#![allow(clippy::unwrap_used, clippy::expect_used)]

use chip_cad::{CadKernel, MonstertruckKernel};
use chip_cam::{FixtureKind, WorkOffsetSlot};
use chip_fixtures::sample_machined_plate_step;
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit};
use chip_project::SoftJawBoltPattern;
use chip_project::{ProjectDocument, UndoHistory};
use chip_tools::{CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId};

use super::{FlipJigWizardModalState, SoftJawWizardModalState};

fn find_text_center(shape: &egui::Shape, needle: &str) -> Option<egui::Pos2> {
    match shape {
        egui::Shape::Text(t) if t.galley.text() == needle => Some(t.pos + t.galley.size() * 0.5),
        egui::Shape::Vec(v) => v.iter().find_map(|s| find_text_center(s, needle)),
        _ => None,
    }
}

fn find_text_containing(shape: &egui::Shape, needle: &str) -> Option<egui::Pos2> {
    match shape {
        egui::Shape::Text(t) if t.galley.text().contains(needle) => {
            Some(t.pos + t.galley.size() * 0.5)
        }
        egui::Shape::Vec(v) => v.iter().find_map(|s| find_text_containing(s, needle)),
        _ => None,
    }
}

fn click_at(pt: egui::Pos2) -> Vec<egui::Event> {
    vec![
        egui::Event::PointerMoved(pt),
        egui::Event::PointerButton {
            pos: pt,
            button: egui::PointerButton::Primary,
            pressed: true,
            modifiers: egui::Modifiers::NONE,
        },
        egui::Event::PointerButton {
            pos: pt,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: egui::Modifiers::NONE,
        },
    ]
}

fn type_replace_events(text: &str) -> Vec<egui::Event> {
    vec![
        egui::Event::Key {
            key: egui::Key::A,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::COMMAND,
        },
        egui::Event::Text(text.to_string()),
        egui::Event::Key {
            key: egui::Key::Enter,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::NONE,
        },
    ]
}

fn make_test_tool(id: &str, num: u32, name: &str, geometry: CutterGeometry) -> Tool {
    Tool {
        id: ToolId(id.to_string()),
        tool_number: num,
        length_offset_number: num,
        diameter_offset_number: num,
        name: name.to_string(),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::Mist,
        stickout_length_mm: 30.0,
        geometry,
        presets: Vec::new(),
    }
}

fn sample_doc_and_project() -> (chip_cad::CadDocument, ProjectDocument) {
    let step = sample_machined_plate_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("plate.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();
    let mut proj = ProjectDocument::new("m88_wizards_test");
    proj.tools = vec![
        make_test_tool(
            "t1_rough",
            1,
            "6mm Rougher",
            CutterGeometry::FlatEndmill {
                diameter_mm: 6.0,
                flutes: 3,
                flute_length_mm: 18.0,
                overall_length_mm: 55.0,
                shank_diameter_mm: 6.0,
            },
        ),
        make_test_tool(
            "t2_finish",
            2,
            "4mm Finisher",
            CutterGeometry::FlatEndmill {
                diameter_mm: 4.0,
                flutes: 4,
                flute_length_mm: 16.0,
                overall_length_mm: 50.0,
                shank_diameter_mm: 4.0,
            },
        ),
        make_test_tool(
            "t3_drill",
            3,
            "6mm Dowel Drill",
            CutterGeometry::TwistDrill {
                diameter_mm: 6.0,
                point_angle_deg: 118.0,
                flutes: 2,
                flute_length_mm: 30.0,
                overall_length_mm: 60.0,
                shank_diameter_mm: 6.0,
            },
        ),
    ];
    (doc, proj)
}

fn run_flip_jig_frame(
    ctx: &egui::Context,
    state: &mut FlipJigWizardModalState,
    project: &mut ProjectDocument,
    doc: &chip_cad::CadDocument,
    history: &mut UndoHistory,
    events: Vec<egui::Event>,
) -> (
    Vec<egui::epaint::ClippedShape>,
    Option<super::FlipJigApplyResult>,
) {
    let mut applied = None;
    let mut out = ctx.run_ui(
        egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(1000.0, 1200.0),
            )),
            events,
            ..Default::default()
        },
        |ui| {
            applied = state.show(ui.ctx(), project, doc, history);
        },
    );
    out.textures_delta.clear();
    (out.shapes, applied)
}

fn run_soft_jaw_frame(
    ctx: &egui::Context,
    state: &mut SoftJawWizardModalState,
    project: &mut ProjectDocument,
    doc: &chip_cad::CadDocument,
    history: &mut UndoHistory,
    events: Vec<egui::Event>,
) -> (
    Vec<egui::epaint::ClippedShape>,
    Option<super::SoftJawApplyResult>,
) {
    let mut applied = None;
    let mut out = ctx.run_ui(
        egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(1000.0, 1200.0),
            )),
            events,
            ..Default::default()
        },
        |ui| {
            applied = state.show(ui.ctx(), project, doc, history);
        },
    );
    out.textures_delta.clear();
    (out.shapes, applied)
}

fn find_number_on_row(
    shapes: &[egui::epaint::ClippedShape],
    row_y: f32,
    target: f64,
) -> Option<egui::Pos2> {
    fn visit(shape: &egui::Shape, row_y: f32, target: f64) -> Option<egui::Pos2> {
        match shape {
            egui::Shape::Text(t)
                if (t.pos.y + t.galley.size().y * 0.5 - row_y).abs() < 12.0
                    && t.galley
                        .text()
                        .trim()
                        .parse::<f64>()
                        .ok()
                        .is_some_and(|v| (v - target).abs() < 1e-4) =>
            {
                Some(t.pos + t.galley.size() * 0.5)
            }
            egui::Shape::Vec(v) => v.iter().find_map(|s| visit(s, row_y, target)),
            _ => None,
        }
    }
    shapes.iter().find_map(|cs| visit(&cs.shape, row_y, target))
}

#[test]
fn m88_flip_jig_wizard_protrusion_and_advanced_overrides() {
    let (doc, mut proj) = sample_doc_and_project();
    proj.display_units = DisplayUnit::Millimeter;
    let mut hist = UndoHistory::default();
    let ctx = egui::Context::default();
    let mut wiz = FlipJigWizardModalState::default();
    wiz.open_for_setup(0);

    let _ = run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let (s1, _) = run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());

    // 1. Verify "Pin Protrusion Above Jig" renders and edit its value from 6 mm -> 11 mm
    let prot_lbl = s1
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "Pin Protrusion Above Jig"))
        .expect("Flip-Jig Wizard must render 'Pin Protrusion Above Jig'");
    let prot_dv_pt = find_number_on_row(&s1, prot_lbl.y, 6.0)
        .expect("Pin Protrusion Above Jig DragValue must display 6.0 mm");
    let _ = run_flip_jig_frame(
        &ctx,
        &mut wiz,
        &mut proj,
        &doc,
        &mut hist,
        click_at(prot_dv_pt),
    );
    let _ = run_flip_jig_frame(
        &ctx,
        &mut wiz,
        &mut proj,
        &doc,
        &mut hist,
        type_replace_events("11"),
    );
    let _ = run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    assert!(
        (wiz.config.pin_protrusion_z_mm - 11.0).abs() < 1e-6,
        "Pin Protrusion Above Jig must update to 11.0 mm; got {}",
        wiz.config.pin_protrusion_z_mm
    );

    // 2. Expand "Advanced Tool & WCS Overrides" and override Pin Tool, Setup 2 WCS, and Spoilboard WCS
    let (s2, _) = run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let adv_pt = s2
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "Advanced Tool & WCS Overrides"))
        .expect("Flip-Jig Wizard must render 'Advanced Tool & WCS Overrides' collapsible header");
    let _ = run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, click_at(adv_pt));
    let (s_adv, _) = run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());

    // Override Pin Tool to T2 4mm Finisher
    let pin_tool_pt = s_adv
        .iter()
        .find_map(|cs| find_text_containing(&cs.shape, "Auto (T3)"))
        .expect("Pin Tool combo must display Auto (T3)");
    let _ = run_flip_jig_frame(
        &ctx,
        &mut wiz,
        &mut proj,
        &doc,
        &mut hist,
        click_at(pin_tool_pt),
    );
    let (s_pt_open, _) = run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let t2_pt = s_pt_open
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "T2 4mm Finisher"))
        .expect("Pin Tool dropdown must list T2 4mm Finisher");
    let _ = run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, click_at(t2_pt));
    assert_eq!(wiz.config.pin_tool_id, Some(ToolId("t2_finish".into())));

    // Override Setup 2 WCS to G58
    let (s_adv2, _) = run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let s2_wcs_pt = s_adv2
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "Auto (G55)"))
        .expect("Setup 2 WCS combo must display Auto (G55)");
    let _ = run_flip_jig_frame(
        &ctx,
        &mut wiz,
        &mut proj,
        &doc,
        &mut hist,
        click_at(s2_wcs_pt),
    );
    let (s_w2_open, _) = run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let g58_pt = s_w2_open
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, WorkOffsetSlot::G58.label()))
        .expect("Setup 2 WCS dropdown must list G58");
    let _ = run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, click_at(g58_pt));
    assert_eq!(wiz.setup2_wcs_slot_override, Some(WorkOffsetSlot::G58));

    // Override Spoilboard WCS to G59
    let (s_adv3, _) = run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let sb_wcs_pt = s_adv3
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "Auto (G54)"))
        .expect("Spoilboard WCS combo must display Auto (G54)");
    let _ = run_flip_jig_frame(
        &ctx,
        &mut wiz,
        &mut proj,
        &doc,
        &mut hist,
        click_at(sb_wcs_pt),
    );
    let (s_wsb_open, _) =
        run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let g59_pt = s_wsb_open
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, WorkOffsetSlot::G59.label()))
        .expect("Spoilboard WCS dropdown must list G59");
    let _ = run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, click_at(g59_pt));
    assert_eq!(wiz.spoilboard_wcs_slot_override, Some(WorkOffsetSlot::G59));

    // Apply and verify generated setups honor all overrides
    let (s_apply, _) = run_flip_jig_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let apply_pt = s_apply
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "Apply to Project"))
        .expect("Apply to Project button must be rendered");
    let (_, res) = run_flip_jig_frame(
        &ctx,
        &mut wiz,
        &mut proj,
        &doc,
        &mut hist,
        click_at(apply_pt),
    );
    assert!(res.is_some(), "Flip-Jig Wizard must apply cleanly");
    assert_eq!(
        proj.setups[0].operations[0].tool_id,
        Some(ToolId("t2_finish".into()))
    );
    assert_eq!(proj.setups[1].wcs_slot, WorkOffsetSlot::G58);
    assert_eq!(proj.setups[2].wcs_slot, WorkOffsetSlot::G59);
    let pin_fix = proj.setups[1]
        .fixtures
        .first()
        .expect("Setup 2 must contain CylinderKeepOut pin fixtures");
    if let FixtureKind::CylinderKeepOut {
        z_bottom, z_top, ..
    } = &pin_fix.kind
    {
        let expected_len = wiz.config.spoilboard_pin_depth_mm + 11.0;
        assert!(
            (z_top.value_mm - z_bottom.value_mm - expected_len).abs() < 1e-3,
            "Keep-out pin total height must equal spoilboard depth (10 mm) + protrusion (11 mm) = {expected_len} mm; got {}",
            z_top.value_mm - z_bottom.value_mm
        );
    } else {
        panic!("expected CylinderKeepOut");
    }
}

#[test]
fn m88_soft_jaw_wizard_cbore_depth_z_band_and_advanced_overrides() {
    let (doc, mut proj) = sample_doc_and_project();
    proj.display_units = DisplayUnit::Millimeter;
    let mut hist = UndoHistory::default();
    let ctx = egui::Context::default();
    let mut wiz = SoftJawWizardModalState::default();
    wiz.open_for_setup(0);

    let _ = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let (s1, _) = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());

    // 1. Edit "Grip Band Bottom Offset" (0 mm -> 2.5 mm)
    let zb_lbl = s1
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "Grip Band Bottom Offset"))
        .expect("Soft-Jaw Wizard must render 'Grip Band Bottom Offset'");
    let zb_dv_pt = find_number_on_row(&s1, zb_lbl.y, 0.0)
        .expect("Grip Band Bottom Offset DragValue must display 0.0 mm");
    let _ = run_soft_jaw_frame(
        &ctx,
        &mut wiz,
        &mut proj,
        &doc,
        &mut hist,
        click_at(zb_dv_pt),
    );
    let _ = run_soft_jaw_frame(
        &ctx,
        &mut wiz,
        &mut proj,
        &doc,
        &mut hist,
        type_replace_events("2.5"),
    );
    let _ = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    assert!(
        (wiz.config.z_band_bottom_offset_mm - 2.5).abs() < 1e-6,
        "z_band_bottom_offset_mm must update to 2.5; got {}",
        wiz.config.z_band_bottom_offset_mm
    );

    // 2. Switch Bolt Pattern to "Custom Bolt Pattern" and edit "Custom C'Bore Depth" (8.5 -> 12.0)
    let (s2, _) = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let bolt_cb_pt = s2
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, SoftJawBoltPattern::Milo4Inch.label()))
        .expect("Bolt Pattern combo must show Milo 4\" default");
    let _ = run_soft_jaw_frame(
        &ctx,
        &mut wiz,
        &mut proj,
        &doc,
        &mut hist,
        click_at(bolt_cb_pt),
    );
    let (s_bolt_open, _) =
        run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let custom_bolt_pt = s_bolt_open
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "Custom Bolt Pattern"))
        .expect("Bolt Pattern dropdown must list 'Custom Bolt Pattern'");
    let _ = run_soft_jaw_frame(
        &ctx,
        &mut wiz,
        &mut proj,
        &doc,
        &mut hist,
        click_at(custom_bolt_pt),
    );
    let (s_cb, _) = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let cb_lbl = s_cb
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "Custom C'Bore Depth"))
        .expect("Custom Bolt Pattern must render 'Custom C'Bore Depth' row");
    let cb_dv_pt = find_number_on_row(&s_cb, cb_lbl.y, 8.5)
        .expect("Custom C'Bore Depth DragValue must display 8.5 mm");
    let _ = run_soft_jaw_frame(
        &ctx,
        &mut wiz,
        &mut proj,
        &doc,
        &mut hist,
        click_at(cb_dv_pt),
    );
    let _ = run_soft_jaw_frame(
        &ctx,
        &mut wiz,
        &mut proj,
        &doc,
        &mut hist,
        type_replace_events("12"),
    );
    let _ = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    assert!(
        matches!(
            wiz.config.mounting_bolt_pattern,
            SoftJawBoltPattern::Custom { cbore_depth_mm, .. } if (cbore_depth_mm - 12.0).abs() < 1e-6
        ),
        "Custom cbore_depth_mm must sync to 12.0 mm; got {:?}",
        wiz.config.mounting_bolt_pattern
    );

    // 3. Expand "Advanced Tool & WCS Overrides", override Finishing Tool & Soft-Jaw WCS, and Apply
    let (s3, _) = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let adv_pt = s3
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "Advanced Tool & WCS Overrides"))
        .expect("Soft-Jaw Wizard must render 'Advanced Tool & WCS Overrides'");
    let _ = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, click_at(adv_pt));
    let (s_adv, _) = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());

    let fin_lbl = s_adv
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "Finishing Tool"))
        .expect("Advanced overrides must render 'Finishing Tool'");
    let fin_cb_pt = s_adv
        .iter()
        .filter_map(|cs| {
            fn find_on_row(shape: &egui::Shape, row_y: f32) -> Option<egui::Pos2> {
                match shape {
                    egui::Shape::Text(t)
                        if t.galley.text().contains("Auto (T1)")
                            && (t.pos.y + t.galley.size().y * 0.5 - row_y).abs() < 12.0 =>
                    {
                        Some(t.pos + t.galley.size() * 0.5)
                    }
                    egui::Shape::Vec(v) => v.iter().find_map(|s| find_on_row(s, row_y)),
                    _ => None,
                }
            }
            find_on_row(&cs.shape, fin_lbl.y)
        })
        .next()
        .expect("Finishing Tool combo must display Auto (T1)");
    let _ = run_soft_jaw_frame(
        &ctx,
        &mut wiz,
        &mut proj,
        &doc,
        &mut hist,
        click_at(fin_cb_pt),
    );
    let (s_f_open, _) = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let t2_pt = s_f_open
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "T2 4mm Finisher"))
        .expect("Finishing Tool dropdown must list T2 4mm Finisher");
    let _ = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, click_at(t2_pt));
    assert_eq!(
        wiz.config.finishing_tool_id,
        Some(ToolId("t2_finish".into()))
    );

    let (s_adv2, _) = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let wcs_pt = s_adv2
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "Auto (G55)"))
        .expect("Soft-Jaw WCS combo must display Auto (G55)");
    let _ = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, click_at(wcs_pt));
    let (s_w_open, _) = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let g57_pt = s_w_open
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, WorkOffsetSlot::G57.label()))
        .expect("Soft-Jaw WCS dropdown must list G57");
    let _ = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, click_at(g57_pt));
    assert_eq!(wiz.wcs_slot_override, Some(WorkOffsetSlot::G57));

    let (s_apply, _) = run_soft_jaw_frame(&ctx, &mut wiz, &mut proj, &doc, &mut hist, Vec::new());
    let apply_pt = s_apply
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "Apply to Project"))
        .expect("Apply to Project button must be rendered");
    let (_, res) = run_soft_jaw_frame(
        &ctx,
        &mut wiz,
        &mut proj,
        &doc,
        &mut hist,
        click_at(apply_pt),
    );
    let res = res.expect("Soft-Jaw Wizard must apply cleanly");
    let sj_setup = &proj.setups[res.new_setup_idx.unwrap()];
    assert_eq!(sj_setup.wcs_slot, WorkOffsetSlot::G57);
    assert_eq!(
        sj_setup.operations.last().unwrap().tool_id,
        Some(ToolId("t2_finish".into()))
    );

    // 4. Verify Inch DisplayUnit conversion on Grip Band Bottom Offset (0.25 in -> 6.35 mm)
    proj.display_units = DisplayUnit::Inch;
    let ctx_in = egui::Context::default();
    let mut wiz_in = SoftJawWizardModalState::default();
    wiz_in.config.z_band_bottom_offset_mm = 6.35;
    wiz_in.open_for_setup(0);
    let _ = run_soft_jaw_frame(&ctx_in, &mut wiz_in, &mut proj, &doc, &mut hist, Vec::new());
    let (s_in, _) =
        run_soft_jaw_frame(&ctx_in, &mut wiz_in, &mut proj, &doc, &mut hist, Vec::new());
    let zb_in_lbl = s_in
        .iter()
        .find_map(|cs| find_text_center(&cs.shape, "Grip Band Bottom Offset"))
        .expect("Inch mode must render 'Grip Band Bottom Offset'");
    let zb_in_pt = find_number_on_row(&s_in, zb_in_lbl.y, 0.25)
        .expect("Inch mode must display 6.35 mm as 0.25 in");
    let _ = run_soft_jaw_frame(
        &ctx_in,
        &mut wiz_in,
        &mut proj,
        &doc,
        &mut hist,
        click_at(zb_in_pt),
    );
    let _ = run_soft_jaw_frame(
        &ctx_in,
        &mut wiz_in,
        &mut proj,
        &doc,
        &mut hist,
        type_replace_events("0.5"),
    );
    let _ = run_soft_jaw_frame(&ctx_in, &mut wiz_in, &mut proj, &doc, &mut hist, Vec::new());
    assert!(
        (wiz_in.config.z_band_bottom_offset_mm - 12.7).abs() < 1e-6,
        "0.5 in Grip Band Bottom Offset must convert to 12.7 mm; got {}",
        wiz_in.config.z_band_bottom_offset_mm
    );
}
