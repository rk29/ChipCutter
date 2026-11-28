#![allow(clippy::unwrap_used, clippy::expect_used)]

use chip_cam::{
    OperationKind, ResolvedRotaryConfig, ResolvedStockGeometry, RotaryAxisKind, RotaryDirection,
    RotaryDriveMode, RotaryUnwindMode,
};
use chip_math::{Aabb3, DVec3, Ray3, RigidFrame3};

use super::*;
use crate::app::ChipCutterApp;
use crate::panels::{InspectorFocus, ViewportPickMode};
use crate::viewport::{HighlightState, gizmos::emit_stock_visuals};
use chip_cam::HeightReference;

fn sample_x_rotary() -> ResolvedRotaryConfig {
    ResolvedRotaryConfig {
        mode: RotaryDriveMode::Indexed3Plus1,
        axis: RotaryAxisKind::XAxis,
        direction: RotaryDirection::RightHandCcw,
        axis_letter: 'A',
        center_wcs: DVec3::new(0.0, 2.0, -3.0),
        stock_u_min: -45.0,
        stock_u_max: 45.0,
        stock_outer_radius_mm: 30.0,
        clearance_radius_mm: 40.0,
        chuck_end_positive: false,
        chuck_u_min: -65.0,
        chuck_u_max: -45.0,
        chuck_radius_mm: 35.0,
        unwind_mode: RotaryUnwindMode::ContinuousMultiTurn,
    }
}

#[test]
fn m84_indexed_auto_rotate_precedence_stock_wireframe_and_picking_ray_alignment() {
    let rot = sample_x_rotary();
    let mut op_idx90 = Operation::new("op90", "Index 90", OperationKind::Adaptive3D);
    op_idx90.rotary = OperationRotaryMode::Indexed3Plus1 { angle_deg: 90.0 };

    // 1. Auto-rotate enabled + no active sim -> rotates to 90° with HUD badge
    let (deg, badge) =
        resolve_effective_viewport_rotary(false, 0.0, true, Some(&op_idx90), Some(rot));
    assert!((deg - 90.0).abs() < 1e-9);
    assert_eq!(badge, Some(('A', 90.0)));

    // 2. Auto-rotate disabled -> stays at 0° with no badge
    let (deg_off, badge_off) =
        resolve_effective_viewport_rotary(false, 0.0, false, Some(&op_idx90), Some(rot));
    assert!((deg_off - 0.0).abs() < 1e-9);
    assert_eq!(badge_off, None);

    // 3. Simulation playback active -> strict precedence over auto-rotate!
    let (deg_sim, badge_sim) =
        resolve_effective_viewport_rotary(true, 137.5, true, Some(&op_idx90), Some(rot));
    assert!((deg_sim - 137.5).abs() < 1e-9);
    assert_eq!(badge_sim, None);

    // 4. `emit_stock_visuals` rotates asymmetric raw Box stock in lockstep with `sim_rotary_deg = 90°`
    let stock = ResolvedStockGeometry::Box {
        aabb_wcs: Aabb3::new(
            DVec3::new(-45.0, -10.0, -25.0),
            DVec3::new(45.0, 10.0, 25.0),
        ),
    };
    let mut rot_origin = rot;
    rot_origin.center_wcs = DVec3::ZERO;
    let hl90 = HighlightState {
        wcs_frame: RigidFrame3::IDENTITY,
        rotary: Some(rot_origin),
        sim_rotary_deg: 90.0,
        ..HighlightState::default()
    };
    let mut tris = Vec::new();
    let mut edges = Vec::new();
    emit_stock_visuals(
        &stock,
        &hl90.wcs_frame,
        hl90.stock_model_wcs_transform(),
        false,
        &mut tris,
        &mut edges,
    );
    assert_eq!(edges.len(), 12);
    let mut min_y = f32::INFINITY;
    let mut max_y = f32::NEG_INFINITY;
    let mut min_z = f32::INFINITY;
    let mut max_z = f32::NEG_INFINITY;
    for e in &edges {
        for p in [e.p0_mm, e.p1_mm] {
            min_y = min_y.min(p[1]);
            max_y = max_y.max(p[1]);
            min_z = min_z.min(p[2]);
            max_z = max_z.max(p[2]);
        }
    }
    assert!(
        (max_y - min_y - 50.0).abs() < 1e-3 && (max_z - min_z - 20.0).abs() < 1e-3,
        "Box stock rotated by 90° around X must swap Y (50 mm) and Z (20 mm) extents; got Y=[{min_y}, {max_y}], Z=[{min_z}, {max_z}]"
    );

    // 5. `world_ray_to_cad_model_space` keeps picking hits 100% aligned at 90°, 180°, 270°, and 45°
    let wcs_frame = RigidFrame3 {
        origin_mm: DVec3::new(12.0, -5.0, 7.0),
        x_axis: DVec3::X,
        y_axis: DVec3::Y,
        z_axis: DVec3::Z,
    };
    let sample_cad_pt = DVec3::new(18.0, 14.0, -9.0);
    for angle_deg in [45.0_f64, 90.0, 180.0, 270.0] {
        let hl = HighlightState {
            wcs_frame,
            rotary: Some(rot),
            sim_rotary_deg: angle_deg,
            ..HighlightState::default()
        };
        let displayed_world = hl.cad_world_transform().transform_point3(sample_cad_pt);
        // Shoot a ray directly from +Z above the displayed rotated point downward through it
        let ray_world =
            Ray3::new(displayed_world + DVec3::new(0.0, 0.0, 80.0), DVec3::NEG_Z).unwrap();
        let ray_cad = hl.world_ray_to_cad_model_space(&ray_world);
        let hit_on_ray = ray_cad.origin + ray_cad.dir * 80.0;
        assert!(
            hit_on_ray.distance(sample_cad_pt) < 1e-8,
            "picking ray at {angle_deg}° must map displayed point back onto exact unrotated CAD point (err = {:.3e} mm)",
            hit_on_ray.distance(sample_cad_pt)
        );
    }
}

#[test]
fn m88_select_xy_plane_x_edge_pick_mode_metadata() {
    let mode = ViewportPickMode::SetupWcsXyPlaneXEdge { setup_idx: 0 };
    let banner = mode
        .banner_text()
        .expect("SetupWcsXyPlaneXEdge must provide a viewport pick banner");
    assert!(banner.contains("+X Edge"));
    let (title, instr, done) = mode
        .guidance_info()
        .expect("SetupWcsXyPlaneXEdge must provide guidance HUD info");
    assert!(title.contains("+X Edge"));
    assert!(instr.contains("edge"));
    assert!(!done);
}

#[test]
fn m95_measure_state_machine_set_bottom_z_undo_and_headless_cards() {
    use chip_cad::{FaceId, MeasureSnapTarget, MeasurementResult};
    use chip_cam::{OperationKind, ReachAnalysisResult, ReachIssueKind, ReachSeverity};
    use chip_math::DisplayUnit;

    use crate::{
        panels::command_palette::CommandId,
        theme::{AppTheme, apply_theme},
        viewport::{draw_measure_readout_card, draw_reach_legend_overlay},
    };

    let ctx = egui::Context::default();
    let mut app = ChipCutterApp::new_headless_for_test(&ctx);
    app.add_standard_operation(OperationKind::Pocket2D);
    assert_eq!(
        app.selection.inspector_focus,
        InspectorFocus::Operation {
            setup_idx: 0,
            op_idx: 0
        }
    );

    // 1. Toggle Measure mode via Command Palette
    app.execute_palette_command(CommandId::ToggleMeasureTool, &ctx);
    assert_eq!(app.selection.pick_mode, ViewportPickMode::Measure);
    assert!(
        app.selection
            .pick_mode
            .banner_text()
            .unwrap()
            .contains("Measure")
    );

    // 2. Single pick -> SingleEntity
    let top_plane = MeasureSnapTarget::BRepPlanarFace {
        face_id: FaceId(1),
        point_wcs: DVec3::new(10.0, 10.0, 0.0),
        normal_wcs: DVec3::Z,
        z_wcs: 0.0,
        area_mm2: 500.0,
    };
    let floor_plane = MeasureSnapTarget::BRepPlanarFace {
        face_id: FaceId(2),
        point_wcs: DVec3::new(10.0, 10.0, -12.5),
        normal_wcs: DVec3::Z,
        z_wcs: -12.5,
        area_mm2: 200.0,
    };
    app.selection
        .measure_state
        .on_click_target(top_plane.clone());
    assert!(matches!(
        app.selection.measure_state.result,
        Some(MeasurementResult::SingleEntity { .. })
    ));
    assert!(app.selection.measure_state.anchor2.is_none());

    // 3. Two-point pick -> ParallelPlanes with exact 12.5 mm depth
    app.selection
        .measure_state
        .on_click_target(floor_plane.clone());
    match &app.selection.measure_state.result {
        Some(MeasurementResult::ParallelPlanes {
            perpendicular_gap_mm,
            ..
        }) => assert!((*perpendicular_gap_mm - 12.5).abs() < 1e-9),
        other => panic!("expected ParallelPlanes, got {other:?}"),
    }
    assert_eq!(
        app.selection.measure_state.suggested_bottom_z_wcs(),
        Some(-12.5)
    );

    // 4. Contextual "Set Op Bottom Z" records atomic Undo & mutates focused Op Bottom Height
    app.apply_measure_bottom_z_to_focused_op(-12.5);
    assert_eq!(
        app.project.setups[0].operations[0].heights.bottom.reference,
        HeightReference::WcsOrigin
    );
    assert!(
        (app.project.setups[0].operations[0]
            .heights
            .bottom
            .offset
            .value_mm
            + 12.5)
            .abs()
            < 1e-9
    );
    app.perform_undo();
    assert_eq!(
        app.project.setups[0].operations[0].heights.bottom.reference,
        HeightReference::FromSelection
    );

    // 5. Third click starts a fresh single-entity pick; clear resets anchors
    app.selection.measure_state.on_click_target(top_plane);
    assert!(matches!(
        app.selection.measure_state.result,
        Some(MeasurementResult::SingleEntity { .. })
    ));
    assert!(app.selection.measure_state.anchor2.is_none());
    app.selection.measure_state.clear_measurement();
    assert!(app.selection.measure_state.anchor1.is_none());
    assert!(app.selection.measure_state.result.is_none());

    // 6. Esc exits Measure mode cleanly; M hotkey toggles Measure mode on and off
    let mut out = ctx.run_ui(
        egui::RawInput {
            events: vec![egui::Event::Key {
                key: egui::Key::Escape,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::NONE,
            }],
            ..Default::default()
        },
        |ui| {
            let c = ui.ctx().clone();
            app.handle_global_shortcuts(&c);
        },
    );
    out.textures_delta.clear();
    assert_eq!(app.selection.pick_mode, ViewportPickMode::InspectEntity);

    let mut out_m = ctx.run_ui(
        egui::RawInput {
            events: vec![egui::Event::Key {
                key: egui::Key::M,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::NONE,
            }],
            ..Default::default()
        },
        |ui| {
            let c = ui.ctx().clone();
            app.handle_global_shortcuts(&c);
        },
    );
    out_m.textures_delta.clear();
    assert_eq!(app.selection.pick_mode, ViewportPickMode::Measure);

    let mut out_m2 = ctx.run_ui(
        egui::RawInput {
            events: vec![egui::Event::Key {
                key: egui::Key::M,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::NONE,
            }],
            ..Default::default()
        },
        |ui| {
            let c = ui.ctx().clone();
            app.handle_global_shortcuts(&c);
        },
    );
    out_m2.textures_delta.clear();
    assert_eq!(app.selection.pick_mode, ViewportPickMode::InspectEntity);

    // 7. Toggle Reach Heatmap via Command Palette + render floating cards in Dark & Light
    assert!(!app.highlight.show_reach_heatmap);
    app.execute_palette_command(CommandId::ToggleReachHeatmap, &ctx);
    assert!(app.highlight.show_reach_heatmap);

    app.selection.measure_state.on_click_target(floor_plane);
    let sample_reach = ReachAnalysisResult {
        tool_name: "6mm Flat".to_string(),
        tool_diameter_mm: 6.0,
        tool_flute_length_mm: 12.0,
        tool_effective_reach_mm: 12.0,
        tool_stickout_length_mm: 15.0,
        min_internal_radius_mm: Some(2.0),
        has_sharp_internal_corners: false,
        max_feature_depth_mm: 18.0,
        amber_count: 2,
        crimson_count: 1,
        tight_corner_count: 1,
        narrow_slot_count: 0,
        flute_exceeded_count: 1,
        stickout_exceeded_count: 1,
        highlight_faces: Vec::new(),
        highlight_edges: Vec::new(),
    };
    let _ = (ReachIssueKind::TightCorner, ReachSeverity::Warning);

    for theme in [AppTheme::Dark, AppTheme::Light] {
        apply_theme(&ctx, theme);
        let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(960.0, 640.0));
        for _frame in 0..3 {
            let mut frame_out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(rect),
                    ..Default::default()
                },
                |ui| {
                    let _ = draw_measure_readout_card(
                        ui,
                        rect,
                        &app.selection.measure_state,
                        DisplayUnit::Millimeter,
                        Some("Pocket 1"),
                    );
                    draw_reach_legend_overlay(
                        ui,
                        rect,
                        Some(&sample_reach),
                        false,
                        DisplayUnit::Millimeter,
                    );
                },
            );
            frame_out.textures_delta.clear();
            // Verify zero exotic non-ASCII glyphs in rendered text shapes
            for cs in &frame_out.shapes {
                if let egui::Shape::Text(ts) = &cs.shape {
                    assert!(
                        ts.galley.text().is_ascii(),
                        "measure/reach HUD text must be plain ASCII, found: {:?}",
                        ts.galley.text()
                    );
                }
            }
        }
    }
}
