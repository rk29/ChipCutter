#![allow(clippy::unwrap_used, clippy::expect_used)]

use chip_cad::{CadKernel, MonstertruckKernel};
use chip_cam::{ClEvent, MotionRole, Setup, ToolpathBuilder, WorkOffsetSlot};
use chip_fixtures::sample_cam_torture_block_step;
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec3, DisplayUnit, approx_eq};
use chip_tools::{CutterGeometry, default_starter_tool_library};

use chip_sim::chip_stock::*;
use chip_sim::*;

#[test]
fn radial_cutter_profiles_are_exact_for_all_six_types() {
    let lib = default_starter_tool_library();
    assert_eq!(lib.tools.len(), 10);

    // T1: 6mm Flat Endmill -> P(r) == 0 for 0 <= r <= 3
    let flat = RadialCutterProfile::from_tool(&lib.tools[0]);
    assert!(approx_eq(flat.tip_to_envelope_dz(0.0).unwrap(), 0.0, 1e-9));
    assert!(approx_eq(flat.tip_to_envelope_dz(3.0).unwrap(), 0.0, 1e-9));
    assert!(flat.tip_to_envelope_dz(3.01).is_none());

    // T2: Bull-Nose -> flat inside R - Rc, Rc at outer wall
    let bull = RadialCutterProfile::from_tool(&lib.tools[1]);
    let CutterGeometry::BullNoseEndmill {
        diameter_mm,
        corner_radius_mm,
        ..
    } = lib.tools[1].geometry
    else {
        panic!("expected bull nose");
    };
    assert!(approx_eq(bull.tip_to_envelope_dz(0.0).unwrap(), 0.0, 1e-9));
    assert!(approx_eq(
        bull.tip_to_envelope_dz(0.5 * diameter_mm).unwrap(),
        corner_radius_mm,
        1e-9
    ));

    // T4: 6mm Ball Endmill -> P(0) = 0, P(3) = 3
    let ball = RadialCutterProfile::from_tool(&lib.tools[3]);
    assert!(approx_eq(ball.tip_to_envelope_dz(0.0).unwrap(), 0.0, 1e-9));
    assert!(approx_eq(ball.tip_to_envelope_dz(3.0).unwrap(), 3.0, 1e-9));

    // T5: 6mm 90deg Chamfer (tip_d = 0.2 -> r_tip = 0.1) -> P(1.1) = 1.0
    let cham = RadialCutterProfile::from_tool(&lib.tools[4]);
    assert!(approx_eq(cham.tip_to_envelope_dz(0.05).unwrap(), 0.0, 1e-9));
    assert!(approx_eq(cham.tip_to_envelope_dz(1.1).unwrap(), 1.0, 1e-9));

    // T6: 6mm 118deg Twist Drill -> P(3) = 3 / tan(59deg)
    let drill = RadialCutterProfile::from_tool(&lib.tools[5]);
    let expected_drill_h = 3.0 / 59.0_f64.to_radians().tan();
    assert!(approx_eq(
        drill.tip_to_envelope_dz(3.0).unwrap(),
        expected_drill_h,
        1e-9
    ));

    // T7: 38mm Face Mill -> non-center-cutting, flat core = 0, outer insert edge > 0
    let face = RadialCutterProfile::from_tool(&lib.tools[6]);
    assert!(!face.is_center_cutting);
    assert!(approx_eq(face.tip_to_envelope_dz(10.0).unwrap(), 0.0, 1e-9));
    assert!(face.tip_to_envelope_dz(19.5).unwrap() > 0.8);

    // T8: 6mm 60deg V-Bit (sharp tip_d = 0.0 -> cot(30deg) = sqrt(3))
    let vbit = RadialCutterProfile::from_tool(&lib.tools[7]);
    assert!(approx_eq(
        vbit.conical_slope_cot_theta().unwrap(),
        3.0_f64.sqrt(),
        1e-9
    ));
    assert!(approx_eq(
        vbit.tip_to_envelope_dz(1.0).unwrap(),
        3.0_f64.sqrt(),
        1e-9
    ));
}

#[test]
fn trapezoidal_timeline_accounts_for_accel_dwell_tool_change_and_scrubs_cleanly() {
    let lib = default_starter_tool_library();
    let t1 = &lib.tools[0];
    let t5 = &lib.tools[4];

    let mut b1 = ToolpathBuilder::new("op_1");
    b1.push_event(ClEvent::SpindleStart {
        rpm: 12_000.0,
        direction: chip_tools::SpindleDirection::Cw,
    });
    b1.rapid_to(DVec3::new(0.0, 0.0, 10.0));
    b1.linear_to(DVec3::new(100.0, 0.0, 10.0), 1_200.0, MotionRole::Cut); // 100 mm @ 20 mm/s
    b1.push_event(ClEvent::Dwell { seconds: 1.5 });
    let tp1 = b1.finish();

    let mut b2 = ToolpathBuilder::new("op_2");
    b2.rapid_to(DVec3::new(100.0, 0.0, 10.0));
    b2.rapid_to(DVec3::new(0.0, 0.0, 10.0)); // 100 mm rapid
    let tp2 = b2.finish();

    let ops = [
        SimOperationInput {
            op_id: "op_1",
            op_name: "Cut Pass",
            tool: t1,
            fallback_spindle_rpm: 10_000.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp1,
        },
        SimOperationInput {
            op_id: "op_2",
            op_name: "Return Rapid",
            tool: t5,
            fallback_spindle_rpm: 8_000.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp2,
        },
    ];

    let kin = MachineKinematicsConfig::default();
    let tl = SimulationTimeline::from_operations(&ops, kin);

    // 100 mm @ 20 mm/s with accel 500 mm/s²: cruise = 5.0 s + accel/decel ramp (20 / 500 = 0.04 s) = 5.04 s
    assert!(approx_eq(tl.operations[0].cutting_time_sec, 5.04, 1e-3));
    assert!(approx_eq(tl.total_dwell_time_sec, 1.5, 1e-6));
    assert!(approx_eq(tl.total_tool_change_time_sec, 15.0, 1e-6));
    assert!(tl.total_time_sec > 21.5);

    let mid_snap = tl.sample_at_distance(50.0).unwrap();
    assert_eq!(mid_snap.motion_code, "G1");
    assert!(approx_eq(mid_snap.tip_pos_wcs.x, 50.0, 1e-6));
    assert!(approx_eq(mid_snap.spindle_rpm, 12_000.0, 1e-6));

    let next_op_t = tl.next_operation_time(0.0);
    assert!(next_op_t > 6.5);
    assert_eq!(format_cycle_time_hms(65.0), "00:01:05");

    // Verify M6 tool-change window animates retract, tool swap at midpoint, and descent
    let tc_start = tl.tool_change_times_sec[0];
    let tc_early = tl.sample_at_time(tc_start + 3.0).unwrap();
    assert_eq!(tc_early.motion_code, "M6");
    assert!(tc_early.tool_change.is_some());
    assert_eq!(tc_early.tool_number, t1.tool_number);
    assert!(
        tc_early.tip_pos_wcs.z > 15.0,
        "outgoing tool should lift smoothly during first half of M6"
    );
    let tc_late = tl.sample_at_time(tc_start + 12.0).unwrap();
    assert_eq!(tc_late.motion_code, "M6");
    assert!(tc_late.tool_change.is_some());
    assert_eq!(tc_late.tool_number, t5.tool_number);
    assert!(
        tc_late.tip_pos_wcs.z > 10.5,
        "incoming tool should lower smoothly during second half of M6"
    );

    // Verify 4-axis circumferential surface feedrate cycle-time accuracy:
    // Pure 90° rotary cut at R = 20 mm (arc = 31.416 mm) @ 600 mm/min (10 mm/s) -> >= 3.14 s!
    let mut b_rot = ToolpathBuilder::new("op_rot_time");
    b_rot.rotary_rapid_to(DVec3::new(0.0, 0.0, 20.0), 0.0);
    b_rot.rotary_linear_to(
        DVec3::new(0.0, 0.0, 20.0),
        90.0,
        600.0,
        20.0,
        MotionRole::Cut,
    );
    let tp_rot = b_rot.finish();
    let tl_rot = SimulationTimeline::from_operations(
        &[SimOperationInput {
            op_id: "op_rot_time",
            op_name: "Rotary Arc Timing",
            tool: t1,
            fallback_spindle_rpm: 10_000.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp_rot,
        }],
        kin,
    );
    let expected_arc_sec = (20.0 * std::f64::consts::FRAC_PI_2) / 10.0; // 3.1416 s
    assert!(
        approx_eq(
            tl_rot.operations[0].cutting_time_sec,
            expected_arc_sec,
            0.05
        ),
        "4-axis circumferential cut should take ~{expected_arc_sec:.3}s, got {:.3}s",
        tl_rot.operations[0].cutting_time_sec
    );
}

#[test]
fn fault_injection_catches_rapid_crash_target_gouge_flute_rub_holder_crash_and_unsafe_plunge() {
    let step_str = sample_cam_torture_block_step().unwrap();
    let cad_doc = MonstertruckKernel
        .load_step_bytes(
            "torture.step",
            step_str.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();

    let setup = Setup::new("s1", "Setup 1", WorkOffsetSlot::G54);
    // Default WCS origin is Stock Top Front-Left (`Z = 0` at stock top `21 mm`, so model top is `Z = -1.0 mm`!)
    let resolved = setup
        .resolve_immutable(&cad_doc, DisplayUnit::Millimeter)
        .unwrap();
    let stock_aabb = resolved.stock.aabb_wcs();
    let model_top_z = resolved.model_aabb_wcs.max.z;

    let lib = default_starter_tool_library();
    let mut short_flute_tool = lib.tools[2].clone(); // 1/8" flat: flute = 12mm, stickout = 16mm
    short_flute_tool.stickout_length_mm = 16.0;
    let face_mill = &lib.tools[6]; // non-center-cutting FaceMill

    // Op A: Deliberate G0 rapid straight through stock + Target Model Gouge + Flute Rub + Holder Collision!
    let mut b_bad = ToolpathBuilder::new("op_bad");
    b_bad.rapid_to(DVec3::new(
        stock_aabb.center().x,
        stock_aabb.center().y,
        5.0,
    ));
    // 1) G0 Rapid crash down to `stock_top - 3.0 mm` (`-3.0 mm`, Gouging model top `-1.0 mm` too!)
    b_bad.rapid_to(DVec3::new(
        stock_aabb.center().x,
        stock_aabb.center().y,
        model_top_z - 2.0,
    ));
    // 2) Deep cut `19 mm` below stock top (`> flute 12mm` AND `> stickout 16mm` -> Flute Rub + Holder Crash!)
    b_bad.linear_to(
        DVec3::new(
            stock_aabb.center().x + 8.0,
            stock_aabb.center().y,
            stock_aabb.max.z - 19.0,
        ),
        400.0,
        MotionRole::Cut,
    );
    let tp_bad = b_bad.finish();

    // Op B: Non-center-cutting FaceMill vertical plunge into uncut corner stock!
    let mut b_plunge = ToolpathBuilder::new("op_plunge");
    b_plunge.rapid_to(DVec3::new(
        stock_aabb.min.x + 25.0,
        stock_aabb.min.y + 20.0,
        5.0,
    ));
    b_plunge.linear_to(
        DVec3::new(
            stock_aabb.min.x + 25.0,
            stock_aabb.min.y + 20.0,
            stock_aabb.max.z - 0.5,
        ),
        200.0,
        MotionRole::Plunge,
    );
    let tp_plunge = b_plunge.finish();

    let ops = [
        SimOperationInput {
            op_id: "op_bad",
            op_name: "Deliberate Faults",
            tool: &short_flute_tool,
            fallback_spindle_rpm: 10_000.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp_bad,
        },
        SimOperationInput {
            op_id: "op_plunge",
            op_name: "Bad FaceMill Plunge",
            tool: face_mill,
            fallback_spindle_rpm: 4_000.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp_plunge,
        },
    ];

    let res = run_setup_simulation(&SetupSimulationInput {
        doc: &cad_doc,
        setup: &resolved,
        target_bodies: &setup.target_bodies,
        fixtures: &[],
        operations: &ops,
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        ..SetupSimulationInput::new(&cad_doc, &resolved, &[])
    })
    .unwrap();

    assert!(res.has_critical_errors());
    let kinds: Vec<SimIssueKind> = res.diagnostics.iter().map(|d| d.kind).collect();
    assert!(
        kinds.contains(&SimIssueKind::RapidIntoStock),
        "expected RapidIntoStock in {kinds:?}"
    );
    assert!(
        kinds.contains(&SimIssueKind::TargetModelGouge),
        "expected TargetModelGouge in {kinds:?}"
    );
    assert!(
        kinds.contains(&SimIssueKind::FluteLengthExceeded),
        "expected FluteLengthExceeded in {kinds:?}"
    );
    assert!(
        kinds.contains(&SimIssueKind::HolderCollision),
        "expected HolderCollision in {kinds:?}"
    );
    assert!(
        kinds.contains(&SimIssueKind::UnsafeVerticalPlunge),
        "expected UnsafeVerticalPlunge in {kinds:?}"
    );

    // Verify checkpoint scrubbing + crisp stock mesh triangulation (neutral & per-op colored)
    let mid_stock = res.reconstruct_stock_at_time(res.timeline.total_time_sec * 0.5);
    let mesh_neutral = res.extract_stock_mesh(&mid_stock, SimStockColorMode::Metallic);
    let mesh_colored = res.extract_stock_mesh(&res.final_stock, SimStockColorMode::ByOperation);
    assert!(!mesh_neutral.vertices.is_empty());
    assert_eq!(mesh_colored.indices.len() % 3, 0);
}

#[test]
fn fault_injection_catches_3d_ball_endmill_gouge_on_sculpted_dome_mesh() {
    use chip_cad::{MeshImportOptions, load_mesh_cad_document};
    use chip_cam::{
        BoxAnchor27, MotionRole, Setup, ToolpathBuilder, WcsOriginMode, WorkOffsetSlot,
    };
    use chip_fixtures::sample_sculpted_dome_stl_bytes;
    use chip_math::{DVec3, DisplayUnit};
    use chip_tools::default_starter_tool_library;

    let stl = sample_sculpted_dome_stl_bytes();
    let cad_doc =
        load_mesh_cad_document("sculpted_dome.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new("s_dome", "Dome Sim", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    let resolved = setup.resolve(&cad_doc, DisplayUnit::Millimeter).unwrap();
    // Stock top = 0.0, Dome apex = -1.0 mm

    let starter = default_starter_tool_library();
    let ball_6mm = starter
        .tools
        .iter()
        .find(|t| matches!(t.geometry, chip_tools::CutterGeometry::BallEndmill { .. }))
        .unwrap();

    // Deliberate 0.6 mm ball-endmill gouge straight across the dome apex (`Z = -1.6 mm` < `-1.0 mm`)!
    let mut b = ToolpathBuilder::new("op_gouge_dome");
    b.rapid_to(DVec3::new(-40.0, 0.0, resolved.retract_z_wcs));
    b.linear_to(DVec3::new(-40.0, 0.0, -1.6), 300.0, MotionRole::Plunge);
    b.linear_to(DVec3::new(40.0, 0.0, -1.6), 800.0, MotionRole::FinishCut);
    b.rapid_to(DVec3::new(40.0, 0.0, resolved.clearance_z_wcs));
    let tp_gouge = b.finish();

    let res = run_setup_simulation(&SetupSimulationInput {
        doc: &cad_doc,
        setup: &resolved,
        target_bodies: &[],
        fixtures: &[],
        operations: &[SimOperationInput {
            op_id: "op_gouge_dome",
            op_name: "Intentional Dome Gouge",
            tool: ball_6mm,
            fallback_spindle_rpm: 12_000.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp_gouge,
        }],
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        ..SetupSimulationInput::new(&cad_doc, &resolved, &[])
    })
    .unwrap();

    assert!(res.has_critical_errors());
    let gouge_diag = res
        .diagnostics
        .iter()
        .find(|d| d.kind == SimIssueKind::TargetModelGouge)
        .expect("3D target-mesh verifier must catch ball-endmill gouge on dome apex");
    assert!(
        (gouge_diag.max_excess_mm - 0.6).abs() < 0.08,
        "expected ~0.6 mm dome apex gouge, got {:.3} mm",
        gouge_diag.max_excess_mm
    );
}

#[test]
fn incremental_sync_stock_matches_full_reconstruction_mid_segment_and_across_scrubs() {
    use chip_cad::CadDocument;
    use chip_cam::{
        BoxAnchor27, MotionRole, Setup, StockConfig, ToolpathBuilder, WcsOriginMode,
        WorkOffsetSlot, XyArcMove,
    };
    use chip_math::{DVec2, DVec3, DisplayUnit, ParamLength};
    use chip_tools::default_starter_tool_library;

    let lib = default_starter_tool_library();
    let flat_6mm = &lib.tools[0];

    let mut setup = Setup::new("s_inc", "Incremental Sync", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(100.0),
        depth_y: ParamLength::from_mm(100.0),
        height_z: ParamLength::from_mm(20.0),
        xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
        z_align: chip_cam::FixedBoxZAlign::CenterZ,
    };
    let doc = CadDocument::empty();
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let mut b = ToolpathBuilder::new("op_inc");
    b.rapid_to(DVec3::new(-30.0, 0.0, 10.0));
    b.linear_to(DVec3::new(-30.0, 0.0, -4.0), 300.0, MotionRole::Plunge);
    // Long linear cut along X
    b.linear_to(DVec3::new(30.0, 0.0, -4.0), 600.0, MotionRole::Cut);
    // Semicircular XY arc around (0, 0) of radius 30 mm
    b.arc_xy_to(XyArcMove {
        end_wcs: DVec3::new(-30.0, 0.0, -4.0),
        center_xy: DVec2::new(0.0, 0.0),
        radius_mm: 30.0,
        ccw: true,
        turns: 0,
        feed_mm_min: 600.0,
        role: MotionRole::Cut,
    });
    // Retract in air
    b.rapid_to(DVec3::new(-30.0, 0.0, 10.0));
    let tp = b.finish();

    let ops = [SimOperationInput {
        op_id: "op_inc",
        op_name: "Linear + Arc Cut",
        tool: flat_6mm,
        fallback_spindle_rpm: 12_000.0,
        intentional_edge_break_mm: 0.0,
        toolpath: &tp,
    }];
    let sim = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved,
        target_bodies: &[],
        fixtures: &[],
        operations: &ops,
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        ..SetupSimulationInput::new(&doc, &resolved, &[])
    })
    .unwrap();

    let mut working = sim.final_stock.clone();
    let mut cursor: Option<StockPlaybackCursor> = None;

    // Sweep forward across both linear and arc segments, then scrub backward and forward again
    let total_t = sim.timeline.total_time_sec;
    let sample_times = [
        0.0,
        total_t * 0.12,
        total_t * 0.25,
        total_t * 0.38,
        total_t * 0.55,
        total_t * 0.72,
        total_t * 0.88,
        total_t * 0.30, // backward scrub
        total_t * 0.65, // forward jump
        total_t,
    ];

    for &t in &sample_times {
        let _ = sim.sync_stock_to_time(&mut working, &mut cursor, t);
        let reference = sim.reconstruct_stock_at_time(t);
        let w_hf = working.as_heightfield().unwrap();
        let r_hf = reference.as_heightfield().unwrap();
        assert_eq!(
            w_hf.z_stock.len(),
            r_hf.z_stock.len(),
            "grid dimensions must match"
        );
        for (i, (&w_z, &r_z)) in w_hf.z_stock.iter().zip(r_hf.z_stock.iter()).enumerate() {
            assert!(
                (w_z - r_z).abs() < 1e-4 || (w_z.is_infinite() && r_z.is_infinite()),
                "incremental sync diverged from full reconstruction at t={t:.3}s cell {i}: working={w_z}, ref={r_z}"
            );
        }
        assert_eq!(
            w_hf.last_op_idx, r_hf.last_op_idx,
            "last_op_idx mismatch at t={t:.3}s"
        );
    }
}

fn make_test_multiform_thread_mill() -> chip_tools::Tool {
    chip_tools::Tool {
        id: chip_tools::ToolId("tm_m8_4tooth".to_string()),
        tool_number: 11,
        length_offset_number: 11,
        diameter_offset_number: 11,
        name: "6.0mm 4-Tooth M8x1.25 Multi-Form Thread Mill".to_string(),
        vendor: Some("Test Carbide".to_string()),
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: chip_tools::SpindleDirection::Cw,
        default_coolant: chip_tools::CoolantMode::Mist,
        stickout_length_mm: 24.0,
        geometry: CutterGeometry::ThreadMill {
            diameter_mm: 6.0,
            neck_diameter_mm: 3.6,
            neck_length_mm: 18.0,
            pitch_mm: 1.25,
            teeth_count: 4,
            thread_angle_deg: 60.0,
            flute_length_mm: 5.0,
            overall_length_mm: 57.0,
            shank_diameter_mm: 6.0,
            flutes: 3,
        },
        presets: Vec::new(),
    }
}

fn make_test_drill_tool(diameter_mm: f64, flute_length_mm: f64) -> chip_tools::Tool {
    chip_tools::Tool {
        id: chip_tools::ToolId(format!("drill_{diameter_mm:.1}")),
        tool_number: 6,
        length_offset_number: 6,
        diameter_offset_number: 6,
        name: format!("{diameter_mm:.1}mm 118° Twist Drill"),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: chip_tools::SpindleDirection::Cw,
        default_coolant: chip_tools::CoolantMode::Flood,
        stickout_length_mm: flute_length_mm + 8.0,
        geometry: CutterGeometry::TwistDrill {
            diameter_mm,
            point_angle_deg: 118.0,
            flute_length_mm,
            overall_length_mm: flute_length_mm + 24.0,
            shank_diameter_mm: diameter_mm,
            flutes: 2,
        },
        presets: Vec::new(),
    }
}

fn build_internal_helical_thread_toolpath(
    op_id: &str,
    center: chip_math::DVec2,
    r_path: f64,
    pitch: f64,
    z_bottom: f64,
    n_turns: u32,
) -> chip_cam::GeneratedToolpath {
    use chip_cam::XyArcMove;
    use chip_math::DVec2;

    let mut b = ToolpathBuilder::new(op_id);
    b.push_event(ClEvent::SpindleStart {
        rpm: 10_000.0,
        direction: chip_tools::SpindleDirection::Cw,
    });
    b.rapid_to(DVec3::new(center.x, center.y, 8.0));
    b.rapid_to(DVec3::new(center.x, center.y, 2.0));
    // Plunge down pre-drilled hole centerline to bottom Z
    b.linear_to(
        DVec3::new(center.x, center.y, z_bottom),
        400.0,
        MotionRole::Plunge,
    );

    // 180° helical tangent lead-in (CCW Bottom-Up G3 Climb)
    let dz_lead = 0.5 * pitch;
    let r_lead = 0.5 * r_path;
    let c_lead = DVec2::new(center.x + r_lead, center.y);
    let z_helix_start = z_bottom + dz_lead;
    b.arc_xy_to(XyArcMove {
        end_wcs: DVec3::new(center.x + r_path, center.y, z_helix_start),
        center_xy: c_lead,
        radius_mm: r_lead,
        ccw: true,
        turns: 0,
        feed_mm_min: 300.0,
        role: MotionRole::LeadIn,
    });

    let mut cur_z = z_helix_start;
    for _ in 0..n_turns {
        let quads = [
            (
                DVec2::new(center.x, center.y + r_path),
                cur_z + 0.25 * pitch,
            ),
            (
                DVec2::new(center.x - r_path, center.y),
                cur_z + 0.50 * pitch,
            ),
            (
                DVec2::new(center.x, center.y - r_path),
                cur_z + 0.75 * pitch,
            ),
            (DVec2::new(center.x + r_path, center.y), cur_z + pitch),
        ];
        for (xy, z_end) in quads {
            b.arc_xy_to(XyArcMove {
                end_wcs: DVec3::new(xy.x, xy.y, z_end),
                center_xy: center,
                radius_mm: r_path,
                ccw: true,
                turns: 0,
                feed_mm_min: 400.0,
                role: MotionRole::FinishCut,
            });
        }
        cur_z += pitch;
    }

    // 180° helical tangent lead-out back to centerline
    b.arc_xy_to(XyArcMove {
        end_wcs: DVec3::new(center.x, center.y, cur_z + dz_lead),
        center_xy: c_lead,
        radius_mm: r_lead,
        ccw: true,
        turns: 0,
        feed_mm_min: 300.0,
        role: MotionRole::LeadOut,
    });
    b.rapid_to(DVec3::new(center.x, center.y, 8.0));
    b.finish()
}

#[test]
fn thread_mill_3d_cutter_mesh_and_non_monotonic_profile_are_exact() {
    let lib = default_starter_tool_library();
    let sp_tool = &lib.tools[8]; // T9: Ø4.75mm single-point 60° thread mill
    let mf_tool = make_test_multiform_thread_mill();

    // 1. Verify single-point `RadialCutterProfile` & `cutter_radius_at_z`
    let sp = RadialCutterProfile::from_tool(sp_tool);
    assert!(sp.is_thread_mill);
    assert!(!sp.is_center_cutting);
    assert!(approx_eq(sp.cutting_radius_mm, 2.375, 1e-9));
    assert!(approx_eq(sp.neck_radius_mm, 1.425, 1e-9));
    assert!(approx_eq(sp.neck_length_mm, 16.0, 1e-9));
    assert!(approx_eq(sp.effective_reach_mm(), 16.0, 1e-9));
    assert_eq!(sp.teeth_count, 1);

    assert!(sp.cutter_radius_at_z(-0.01).is_none());
    assert!(sp.cutter_radius_at_z(25.0).is_none());
    // Single-point crest at z = 0.6 mm reaches full cutting radius `2.375 mm`
    assert!(approx_eq(sp.cutter_radius_at_z(0.6).unwrap(), 2.375, 1e-9));
    // Flanks slope down smoothly at 60° toward `R_neck = 1.425 mm`
    let r_flank = sp.cutter_radius_at_z(0.3).unwrap();
    assert!(r_flank > sp.neck_radius_mm && r_flank < sp.cutting_radius_mm);
    // Relieved neck (`1.2 < z <= 16.0`) drops strictly below cutting radius!
    assert!(approx_eq(sp.cutter_radius_at_z(5.0).unwrap(), 1.425, 1e-9));
    assert!(approx_eq(sp.cutter_radius_at_z(15.5).unwrap(), 1.425, 1e-9));
    // Shank (`16.0 < z <= 22.0`) steps back up to `3.0 mm`
    assert!(approx_eq(sp.cutter_radius_at_z(18.0).unwrap(), 3.0, 1e-9));

    // 2. Verify multi-form 4-tooth `cutter_radius_at_z`
    let mf = RadialCutterProfile::from_tool(&mf_tool);
    assert!(mf.is_thread_mill);
    assert_eq!(mf.teeth_count, 4);
    for k in 0..4 {
        let z_crest = (k as f64 + 0.5) * 1.25;
        assert!(
            approx_eq(mf.cutter_radius_at_z(z_crest).unwrap(), 3.0, 1e-9),
            "multi-form tooth {k} crest at z={z_crest}"
        );
    }
    // Valley between teeth dips sharply below cutting radius (`< 2.0 mm`)
    let r_valley = mf.cutter_radius_at_z(1.25).unwrap();
    assert!(r_valley < 2.0 && r_valley >= mf.neck_radius_mm);
    // Relieved neck (`5.0 < z <= 18.0`) == `1.8 mm < 3.0 mm`
    assert!(approx_eq(mf.cutter_radius_at_z(10.0).unwrap(), 1.8, 1e-9));
    // Shank (`18.0 < z <= 24.0`) == `3.0 mm`
    assert!(approx_eq(mf.cutter_radius_at_z(21.0).unwrap(), 3.0, 1e-9));
}

#[test]
fn tridexel_internal_m8_helical_thread_mill_carves_real_undercut_grooves_with_zero_crashes() {
    use chip_cad::CadDocument;
    use chip_cam::{ResolvedSetup, ResolvedStockGeometry};
    use chip_math::{Aabb3, DVec2, RigidFrame3};

    let lib = default_starter_tool_library();
    let tm_475 = &lib.tools[8]; // T9: Ø4.75mm single-point 60° thread mill
    let pilot_drill = make_test_drill_tool(6.8, 22.0);

    // Compact 20 x 20 x 12 mm block centered at (0, 0), Z ∈ [-12, 0] -> ~0.125 mm Tri-Dexel pitch!
    let aabb = Aabb3::new(DVec3::new(-10.0, -10.0, -12.0), DVec3::new(10.0, 10.0, 0.0));
    let resolved = ResolvedSetup {
        frame: RigidFrame3::IDENTITY,
        model_aabb_wcs: aabb,
        stock: ResolvedStockGeometry::Box { aabb_wcs: aabb },
        rotary: None,
        clearance_z_wcs: 12.0,
        retract_z_wcs: 5.0,
    };
    let doc = CadDocument::empty();

    // Op 1: Through-drill Ø6.8 mm pilot bore at (0, 0)
    let mut b_drill = ToolpathBuilder::new("op_pilot");
    b_drill.rapid_to(DVec3::new(0.0, 0.0, 8.0));
    b_drill.linear_to(DVec3::new(0.0, 0.0, -15.0), 300.0, MotionRole::Plunge);
    b_drill.rapid_to(DVec3::new(0.0, 0.0, 8.0));
    let tp_drill = b_drill.finish();

    // Op 2: Internal M8 x 1.25 helical thread mill (`r_path = (8.0 - 4.75) / 2 = 1.625 mm`)
    let tp_thread =
        build_internal_helical_thread_toolpath("op_m8_thread", DVec2::ZERO, 1.625, 1.25, -10.0, 8);

    let sim_drill_only = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved,
        target_bodies: &[],
        fixtures: &[],
        operations: &[SimOperationInput {
            op_id: "op_pilot",
            op_name: "Pilot Drill Ø6.8",
            tool: &pilot_drill,
            fallback_spindle_rpm: 3_500.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp_drill,
        }],
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        engine: SimEngineKind::TriDexel,
        ..SetupSimulationInput::new(&doc, &resolved, &[])
    })
    .unwrap();

    let sim = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved,
        target_bodies: &[],
        fixtures: &[],
        operations: &[
            SimOperationInput {
                op_id: "op_pilot",
                op_name: "Pilot Drill Ø6.8",
                tool: &pilot_drill,
                fallback_spindle_rpm: 3_500.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp_drill,
            },
            SimOperationInput {
                op_id: "op_m8_thread",
                op_name: "M8x1.25 Thread Mill",
                tool: tm_475,
                fallback_spindle_rpm: 10_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp_thread,
            },
        ],
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        engine: SimEngineKind::TriDexel,
        ..SetupSimulationInput::new(&doc, &resolved, &[])
    })
    .unwrap();

    assert!(
        sim.diagnostics.is_empty(),
        "clean pre-drilled M8x1.25 thread mill must have 0 diagnostics, got: {:#?}",
        sim.diagnostics
    );
    let thread_removed = sim.removed_volume_mm3 - sim_drill_only.removed_volume_mm3;
    assert!(
        thread_removed > 15.0,
        "helical M8x1.25 thread grooves should remove > 15 mm³, removed {thread_removed:.2} mm³"
    );

    let td = sim.as_tridexel().unwrap();

    // Verify Z-dexel overhang splitting inside the thread wall (`x ≈ 3.70 mm ∈ (3.40, 4.00)`, `y ≈ 0`)
    let ix = ((3.70 - td.origin_wcs.x) / td.dx_mm()).floor() as usize;
    let iy = ((0.0 - td.origin_wcs.y) / td.dy_mm()).floor() as usize;
    let z_spans = td.z_plane().ray_spans(ix, iy);
    assert!(
        z_spans.len() >= 5,
        "Z-dexel inside internal thread wall must split into >= 5 overhanging thread crests, got {}: {:?}",
        z_spans.len(),
        &*z_spans
    );

    // Verify X-dexel radial thread profile oscillates between carved root (~4.0 mm) and retained crest (~3.4 mm)
    let j_mid = ((0.0 - td.x_plane().origin_u_mm) / td.x_plane().du_mm).floor() as usize;
    let mut min_wall_x = f64::INFINITY;
    let mut max_wall_x = f64::NEG_INFINITY;
    for k in 0..td.nz() {
        let pz = td.x_plane().center_v(k);
        if !(-8.0..=-2.0).contains(&pz) {
            continue;
        }
        let spans = td.x_plane().ray_spans(j_mid, k);
        // Positive-X solid wall starts after the hole gap (`min_mm > 0`)
        if let Some(pos_wall) = spans.iter().find(|s| s.min_mm > 0.5) {
            min_wall_x = min_wall_x.min(pos_wall.min_mm);
            max_wall_x = max_wall_x.max(pos_wall.min_mm);
        }
    }
    assert!(
        max_wall_x > 3.80,
        "carved thread root must reach > 3.80 mm (nominal 4.00 mm), got {max_wall_x:.3}"
    );
    assert!(
        min_wall_x < 3.55,
        "retained thread crest must stay < 3.55 mm (pilot 3.40 mm), got {min_wall_x:.3}"
    );

    let mesh = tessellate_tridexel_stock_mesh(td, true);
    assert!(mesh.vertices.len() > 200);
    assert_eq!(mesh.indices.len() % 3, 0);
}

#[test]
fn tridexel_external_m12_boss_thread_mill_carves_helical_grooves_with_zero_crashes() {
    use chip_cad::CadDocument;
    use chip_cam::{
        ResolvedSetup, ResolvedStockGeometry, StockAxis, ThreadFormKind, XyArcMove,
        compute_thread_minor_diameter_mm,
    };
    use chip_math::{Aabb3, DVec2, RigidFrame3};

    // External thread mill with ample neck relief (`(6.0 - 3.4)/2 = 1.30 mm > 1.0735 mm` M12x1.75 depth)
    let ext_tm = chip_tools::Tool {
        id: chip_tools::ToolId("tm_ext_m12".to_string()),
        tool_number: 12,
        length_offset_number: 12,
        diameter_offset_number: 12,
        name: "6.0mm Single-Point External Thread Mill".to_string(),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: chip_tools::SpindleDirection::Cw,
        default_coolant: chip_tools::CoolantMode::Mist,
        stickout_length_mm: 24.0,
        geometry: CutterGeometry::ThreadMill {
            diameter_mm: 6.0,
            neck_diameter_mm: 3.4,
            neck_length_mm: 18.0,
            pitch_mm: 1.75,
            teeth_count: 1,
            thread_angle_deg: 60.0,
            flute_length_mm: 1.6,
            overall_length_mm: 55.0,
            shank_diameter_mm: 6.0,
            flutes: 3,
        },
        presets: Vec::new(),
    };

    // Cylindrical boss stock: Ø12.0 mm (`R = 6.0 mm`), Z ∈ [-14.0, 0.0]
    let boss_r = 6.0;
    let boss_aabb = Aabb3::new(
        DVec3::new(-boss_r, -boss_r, -14.0),
        DVec3::new(boss_r, boss_r, 0.0),
    );
    let resolved = ResolvedSetup {
        frame: RigidFrame3::IDENTITY,
        model_aabb_wcs: boss_aabb,
        stock: ResolvedStockGeometry::Cylinder {
            axis: StockAxis::Z,
            center_wcs: DVec3::ZERO,
            radius_mm: boss_r,
            min_axial_mm: -14.0,
            max_axial_mm: 0.0,
            aabb_wcs: boss_aabb,
        },
        rotary: None,
        clearance_z_wcs: 12.0,
        retract_z_wcs: 5.0,
    };

    let pitch = 1.75;
    let d_minor = compute_thread_minor_diameter_mm(ThreadFormKind::ExternalBoss, 12.0, pitch, 60.0);
    let r_path = 0.5 * (d_minor + 6.0); // 4.927 + 3.0 = 7.927 mm
    let r_lead = 1.5;
    let center = DVec2::ZERO;
    let c_lead = DVec2::new(r_path + r_lead, 0.0);

    // Right-Hand External Climb = Top-Down CW (G2); external tangent lead-in bends CCW
    let mut b = ToolpathBuilder::new("op_ext_m12");
    b.rapid_to(DVec3::new(r_path + r_lead, r_lead, 8.0));
    b.linear_to(
        DVec3::new(r_path + r_lead, r_lead, 0.5),
        400.0,
        MotionRole::Plunge,
    );
    let z_helix_start = 0.5 - 0.25 * pitch;
    b.arc_xy_to(XyArcMove {
        end_wcs: DVec3::new(r_path, 0.0, z_helix_start),
        center_xy: c_lead,
        radius_mm: r_lead,
        ccw: true,
        turns: 0,
        feed_mm_min: 350.0,
        role: MotionRole::LeadIn,
    });
    let mut cur_z = z_helix_start;
    for _ in 0..6 {
        let quads = [
            (DVec2::new(0.0, -r_path), cur_z - 0.25 * pitch),
            (DVec2::new(-r_path, 0.0), cur_z - 0.50 * pitch),
            (DVec2::new(0.0, r_path), cur_z - 0.75 * pitch),
            (DVec2::new(r_path, 0.0), cur_z - pitch),
        ];
        for (xy, z_end) in quads {
            b.arc_xy_to(XyArcMove {
                end_wcs: DVec3::new(xy.x, xy.y, z_end),
                center_xy: center,
                radius_mm: r_path,
                ccw: false,
                turns: 0,
                feed_mm_min: 450.0,
                role: MotionRole::FinishCut,
            });
        }
        cur_z -= pitch;
    }
    b.arc_xy_to(XyArcMove {
        end_wcs: DVec3::new(r_path + r_lead, -r_lead, cur_z - 0.25 * pitch),
        center_xy: c_lead,
        radius_mm: r_lead,
        ccw: true,
        turns: 0,
        feed_mm_min: 350.0,
        role: MotionRole::LeadOut,
    });
    b.rapid_to(DVec3::new(r_path + r_lead, -r_lead, 8.0));
    let tp = b.finish();

    let doc = CadDocument::empty();
    let sim = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved,
        target_bodies: &[],
        fixtures: &[],
        operations: &[SimOperationInput {
            op_id: "op_ext_m12",
            op_name: "External M12x1.75 Thread",
            tool: &ext_tm,
            fallback_spindle_rpm: 10_000.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp,
        }],
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        engine: SimEngineKind::TriDexel,
        ..SetupSimulationInput::new(&doc, &resolved, &[])
    })
    .unwrap();

    assert!(
        sim.diagnostics.is_empty(),
        "external M12x1.75 boss thread mill must have 0 diagnostics, got: {:#?}",
        sim.diagnostics
    );
    assert!(
        sim.removed_volume_mm3 > 30.0,
        "external M12x1.75 thread grooves should remove > 30 mm³, got {:.2}",
        sim.removed_volume_mm3
    );

    let td = sim.as_tridexel().unwrap();
    // Z-ray inside the boss outer thread shell (`x ≈ 5.45 mm ∈ (4.927, 6.000)`, `y ≈ 0`)
    let ix = ((5.45 - td.origin_wcs.x) / td.dx_mm()).floor() as usize;
    let iy = ((0.0 - td.origin_wcs.y) / td.dy_mm()).floor() as usize;
    let z_spans = td.z_plane().ray_spans(ix, iy);
    assert!(
        z_spans.len() >= 4,
        "external boss Z-dexel must retain >= 4 overhanging thread crests, got {}: {:?}",
        z_spans.len(),
        &*z_spans
    );

    // X-ray outer boss radius along `y ≈ 0` oscillates between carved root (< 5.25) and crest (> 5.80)
    let j_mid = ((0.0 - td.x_plane().origin_u_mm) / td.x_plane().du_mm).floor() as usize;
    let mut min_outer_x = f64::INFINITY;
    let mut max_outer_x = f64::NEG_INFINITY;
    for k in 0..td.nz() {
        let pz = td.x_plane().center_v(k);
        if !(-9.0..=-2.0).contains(&pz) {
            continue;
        }
        let spans = td.x_plane().ray_spans(j_mid, k);
        if let Some(last) = spans.last() {
            min_outer_x = min_outer_x.min(last.max_mm);
            max_outer_x = max_outer_x.max(last.max_mm);
        }
    }
    assert!(
        min_outer_x < 5.25,
        "external thread root must carve < 5.25 mm (minor 4.927 mm), got {min_outer_x:.3}"
    );
    assert!(
        max_outer_x > 5.80,
        "external thread crest must remain > 5.80 mm (major 6.00 mm), got {max_outer_x:.3}"
    );
}

#[test]
fn thread_mill_fault_injection_catches_missing_pilot_plunge_and_neck_reach_exceeded_in_both_engines()
 {
    use chip_cad::CadDocument;
    use chip_cam::{ResolvedSetup, ResolvedStockGeometry};
    use chip_math::{Aabb3, DVec2, RigidFrame3};

    let lib = default_starter_tool_library();
    let tm_475 = &lib.tools[8]; // T9: Ø4.75mm, neck_length = 16.0mm, stickout = 22.0mm
    let pilot_drill = make_test_drill_tool(6.8, 26.0);

    let aabb = Aabb3::new(DVec3::new(-15.0, -15.0, -22.0), DVec3::new(15.0, 15.0, 0.0));
    let resolved = ResolvedSetup {
        frame: RigidFrame3::IDENTITY,
        model_aabb_wcs: aabb,
        stock: ResolvedStockGeometry::Box { aabb_wcs: aabb },
        rotary: None,
        clearance_z_wcs: 12.0,
        retract_z_wcs: 5.0,
    };
    let doc = CadDocument::empty();

    // Fault 1: Plunge straight into solid un-drilled stock (missing pilot drill!)
    let tp_no_pilot =
        build_internal_helical_thread_toolpath("op_no_pilot", DVec2::ZERO, 1.625, 1.25, -8.0, 4);

    // Fault 2: Pre-drill deep Ø6.8 hole, then thread down to Z = -18.5 mm (`> neck_length 16.0 mm`!)
    let mut b_deep_drill = ToolpathBuilder::new("op_deep_pilot");
    b_deep_drill.rapid_to(DVec3::new(0.0, 0.0, 8.0));
    b_deep_drill.linear_to(DVec3::new(0.0, 0.0, -25.0), 300.0, MotionRole::Plunge);
    b_deep_drill.rapid_to(DVec3::new(0.0, 0.0, 8.0));
    let tp_deep_drill = b_deep_drill.finish();
    let tp_too_deep =
        build_internal_helical_thread_toolpath("op_too_deep", DVec2::ZERO, 1.625, 1.25, -18.5, 2);

    for engine in [SimEngineKind::Heightmap, SimEngineKind::TriDexel] {
        let sim_no_pilot = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_no_pilot",
                op_name: "Un-Drilled Thread Plunge",
                tool: tm_475,
                fallback_spindle_rpm: 10_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp_no_pilot,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        })
        .unwrap();

        assert!(
            sim_no_pilot.has_critical_errors(),
            "{engine:?}: missing pilot drill plunge MUST raise a CriticalError, got: {:#?}",
            sim_no_pilot.diagnostics
        );
        assert!(
            sim_no_pilot.diagnostics.iter().any(|d| {
                d.kind == SimIssueKind::UnsafeVerticalPlunge
                    && d.severity == SimSeverity::CriticalError
            }),
            "{engine:?}: expected CriticalError UnsafeVerticalPlunge, got: {:#?}",
            sim_no_pilot.diagnostics
        );

        let sim_too_deep = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[
                SimOperationInput {
                    op_id: "op_deep_pilot",
                    op_name: "Deep Pilot Drill",
                    tool: &pilot_drill,
                    fallback_spindle_rpm: 3_500.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp_deep_drill,
                },
                SimOperationInput {
                    op_id: "op_too_deep",
                    op_name: "Over-Reach Thread Mill",
                    tool: tm_475,
                    fallback_spindle_rpm: 10_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp_too_deep,
                },
            ],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        })
        .unwrap();

        assert!(
            sim_too_deep
                .diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::FluteLengthExceeded),
            "{engine:?}: threading past neck_length (18.5 > 16.0 mm) must flag shank rub, got: {:#?}",
            sim_too_deep.diagnostics
        );
    }
}

#[test]
fn heightmap_and_tridexel_pre_drilled_step_hole_thread_mill_report_zero_critical_issues() {
    use std::sync::atomic::AtomicBool;

    use chip_cad::SurfaceKind;
    use chip_cam::{
        GeometrySelectionItem, HeightReference, HeightSpec, Operation, OperationGenRequest,
        OperationKind, StrategyParams, ThreadCutDirection, ThreadFormKind, ThreadHandedness,
        ThreadMillParams, generate_operation_toolpath_with_rest,
    };
    use chip_fixtures::sample_machined_plate_step;
    use chip_math::ParamLength;

    // Load real STEP plate (80x60x16 mm) containing an exact Ø8.0 mm cylindrical through-hole
    let step = sample_machined_plate_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("plate.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();

    let setup = Setup::new("s_step_tm", "STEP Thread Setup", WorkOffsetSlot::G54);
    let resolved = setup
        .resolve_immutable(&doc, DisplayUnit::Millimeter)
        .unwrap();

    // Find the Ø8.0 mm cylindrical bore face in the STEP B-Rep
    let hole_face = doc
        .faces
        .iter()
        .find(|f| {
            matches!(f.surface, SurfaceKind::Cylinder { radius_mm, .. } if (radius_mm - 4.0).abs() < 0.05)
        })
        .expect("sample_machined_plate_step must contain an Ø8.0 mm cylindrical hole");
    let binding = doc.bind_face(hole_face.id).unwrap();
    let hole_sel = vec![GeometrySelectionItem::CylindricalHole { binding }];

    let drill_8mm = make_test_drill_tool(8.0, 28.0);
    let lib = default_starter_tool_library();
    let tm_475 = &lib.tools[8]; // Ø4.75mm, neck Ø2.85mm, neck_length 16.0mm
    let cancel = AtomicBool::new(false);

    // Op 1: Drill Ø8.0 mm through-hole
    let mut op_drill = Operation::new("op_drill_8", "Drill Ø8.0 Pilot", OperationKind::DrillBore);
    op_drill.tool_id = Some(drill_8mm.id.clone());
    op_drill.selections = hole_sel.clone();
    op_drill.heights.bottom = HeightSpec::new(HeightReference::StockBottom, -3.5);

    let tp_drill = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_drill,
        tool: &drill_8mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();

    // Op 2: Thread Mill M9x1.0 (pilot Ø8.0 -> major Ø9.0) inside the STEP hole
    let mut op_tm = Operation::new("op_tm_m9", "Thread Mill M9x1.0", OperationKind::ThreadMill);
    op_tm.tool_id = Some(tm_475.id.clone());
    op_tm.selections = hole_sel;
    op_tm.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
    op_tm.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -10.0);
    op_tm.strategy = Some(StrategyParams::ThreadMill(ThreadMillParams {
        form: ThreadFormKind::InternalHole,
        handedness: ThreadHandedness::RightHand,
        cut_direction: ThreadCutDirection::Climb,
        standard_preset: None,
        major_diameter: ParamLength::from_mm(9.0),
        pitch: ParamLength::from_mm(1.0),
        pitch_diameter_offset: ParamLength::from_mm(0.0),
        radial_passes: 2,
        spring_passes: 1,
        lead_in_clearance: ParamLength::from_mm(0.5),
        compensate_centerline_feed: true,
        optimize_hole_order: true,
    }));

    let tp_tm = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_tm,
        tool: tm_475,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();

    let sim_ops = [
        SimOperationInput::from_operation(&op_drill, &drill_8mm, &tp_drill),
        SimOperationInput::from_operation(&op_tm, tm_475, &tp_tm),
    ];

    for engine in [SimEngineKind::Heightmap, SimEngineKind::TriDexel] {
        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &sim_ops,
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        })
        .unwrap();

        assert!(
            !sim.has_critical_errors(),
            "{engine:?}: pre-drilled + thread-milled STEP setup must report 0 critical errors, got: {:#?}",
            sim.diagnostics
        );
        assert!(
            !sim.diagnostics
                .iter()
                .any(|d| d.op_id == "op_tm_m9" && d.kind == SimIssueKind::FluteLengthExceeded),
            "{engine:?}: thread mill within neck_length must not report FluteLengthExceeded: {:#?}",
            sim.diagnostics
        );
        assert!(
            !sim.diagnostics
                .iter()
                .any(|d| d.op_id == "op_tm_m9" && d.kind == SimIssueKind::TargetModelGouge),
            "{engine:?}: intentional thread groove into STEP pilot wall must not report TargetModelGouge: {:#?}",
            sim.diagnostics
        );
    }
}

#[test]
fn m61_clamp_in_rapid_path_avoidance_and_cpu_gpu_parity_across_engines() {
    use chip_cam::{
        BoxAnchor27, FixedBoxXyAlign, FixedBoxZAlign, FixtureBody, MotionRole, Setup, StockConfig,
        ToolpathBuilder, WcsOriginMode, WorkOffsetSlot, resolve_setup_fixtures_parametric,
        route_toolpath_around_fixtures,
    };
    use chip_math::{DVec2, DVec3, DisplayUnit, ParamLength};
    use chip_sim::SimComputeBackendChoice;

    let lib = default_starter_tool_library();
    let flat_6mm = &lib.tools[0]; // Ø6mm flat endmill

    let mut setup = Setup::new("s_m61_rapid", "M61 Rapid Avoidance", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(120.0),
        depth_y: ParamLength::from_mm(100.0),
        height_z: ParamLength::from_mm(20.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::CenterZ,
    };
    // Place a ToeClamp with tall stud tower right across Y = 0 between the two pockets
    setup.fixtures.push(FixtureBody::default_toe_clamp(
        "center_clamp",
        "Center Toe Clamp",
        DVec2::new(0.0, -16.0),
        90.0,
    ));

    let doc = chip_cad::CadDocument::empty();
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let fixtures =
        resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();
    assert_eq!(fixtures.len(), 1);
    let clamp_top_z = fixtures[0].aabb_wcs.max.z;

    // Build two pocket cuts separated by a horizontal G0 rapid at Z = +10 mm (right through the clamp!)
    let mut tb = ToolpathBuilder::new("op_two_pockets");
    tb.rapid_to(DVec3::new(-35.0, 0.0, 10.0));
    tb.linear_to(DVec3::new(-35.0, 0.0, -5.0), 300.0, MotionRole::Plunge);
    tb.linear_to(DVec3::new(-25.0, 0.0, -5.0), 800.0, MotionRole::Cut);
    tb.rapid_to(DVec3::new(-25.0, 0.0, 10.0));
    tb.rapid_to(DVec3::new(25.0, 0.0, 10.0)); // Collides with Center Toe Clamp!
    tb.linear_to(DVec3::new(25.0, 0.0, -5.0), 300.0, MotionRole::Plunge);
    tb.linear_to(DVec3::new(35.0, 0.0, -5.0), 800.0, MotionRole::Cut);
    tb.rapid_to(DVec3::new(35.0, 0.0, 10.0));
    let tp_unrouted = tb.finish();

    let mut tp_routed = tp_unrouted.clone();
    let routing =
        route_toolpath_around_fixtures(&mut tp_routed, flat_6mm, &resolved, &fixtures, true, None);
    assert_eq!(routing.rapid_lifts_inserted, 1);
    assert!(routing.max_lifted_z_wcs >= clamp_top_z + fixtures[0].clearance_margin_mm + 2.0 - 1e-4);

    // 1. Unrouted toolpath (`auto_avoid = false`) MUST trigger identical `FixtureCollision`
    //    across Heightmap & TriDexel and across CPU & GPU backends!
    let mut baseline_fixture_diags: Option<Vec<chip_sim::SimDiagnostic>> = None;
    for engine in [SimEngineKind::Heightmap, SimEngineKind::TriDexel] {
        for choice in [SimComputeBackendChoice::Cpu, SimComputeBackendChoice::Auto] {
            let sim_bad = run_setup_simulation(&SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                fixtures: &fixtures,
                operations: &[SimOperationInput {
                    op_id: "op_two_pockets",
                    op_name: "Two Pockets",
                    tool: flat_6mm,
                    fallback_spindle_rpm: 12_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp_unrouted,
                }],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                initial_stock: None,
                initial_raw_volume_mm3: None,
                carried_over_from_setup: None,
                cancel: None,
                engine,
                backend_choice: choice,
                ..SetupSimulationInput::new(&doc, &resolved, &[])
            })
            .unwrap();
            let f_diags: Vec<_> = sim_bad
                .diagnostics
                .iter()
                .filter(|d| d.kind == SimIssueKind::FixtureCollision)
                .cloned()
                .collect();
            assert_eq!(
                f_diags.len(),
                1,
                "{engine:?}/{choice:?}: expected 1 FixtureCollision on unrouted rapid, got {f_diags:#?}"
            );
            assert_eq!(f_diags[0].severity, chip_sim::SimSeverity::CriticalError);
            assert_eq!(f_diags[0].fixture_id.as_deref(), Some("center_clamp"));
            assert_eq!(f_diags[0].fixture_name.as_deref(), Some("Center Toe Clamp"));
            assert!(f_diags[0].max_excess_mm > 1.0);
            if let Some(ref base) = baseline_fixture_diags {
                assert_eq!(
                    &f_diags, base,
                    "{engine:?}/{choice:?}: FixtureCollision diagnostics must be 100% identical across engines & CPU/GPU"
                );
            } else {
                baseline_fixture_diags = Some(f_diags);
            }

            // 2. Routed toolpath (`auto_avoid = true`) MUST have ZERO critical errors!
            let sim_ok = run_setup_simulation(&SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                fixtures: &fixtures,
                operations: &[SimOperationInput {
                    op_id: "op_two_pockets",
                    op_name: "Two Pockets",
                    tool: flat_6mm,
                    fallback_spindle_rpm: 12_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp_routed,
                }],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                initial_stock: None,
                initial_raw_volume_mm3: None,
                carried_over_from_setup: None,
                cancel: None,
                engine,
                backend_choice: choice,
                ..SetupSimulationInput::new(&doc, &resolved, &[])
            })
            .unwrap();
            assert!(
                !sim_ok.has_critical_errors(),
                "{engine:?}/{choice:?}: routed toolpath must report 0 critical errors, got {:#?}",
                sim_ok.diagnostics
            );
        }
    }
}

#[test]
fn m61_holder_vs_vise_jaw_spoilboard_overcut_clamp_in_cut_and_cylindrical_parity() {
    use chip_cam::{
        BoxAnchor27, FixedBoxXyAlign, FixedBoxZAlign, FixtureBody, FixtureKind, MotionRole,
        RotaryAxisKind, RotaryDriveMode, Setup, StockAxis, StockConfig, ToolpathBuilder,
        ViseClampAxis, ViseFixedSide, WcsOriginMode, WorkOffsetSlot,
        resolve_setup_fixtures_parametric, route_toolpath_around_fixtures,
    };
    use chip_math::{DVec2, DVec3, DisplayUnit, ParamLength};
    use chip_sim::SimComputeBackendChoice;
    use chip_tools::{CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId};

    let doc = chip_cad::CadDocument::empty();

    // -------------------------------------------------------------------------
    // A. Holder-vs-ViseJaws: tall vise jaw beside deep pocket — flute clears
    //    horizontally, wider collet/holder collides with accurate penetration!
    // -------------------------------------------------------------------------
    let short_3mm = Tool {
        id: ToolId("t_3mm_short".to_string()),
        tool_number: 2,
        length_offset_number: 2,
        diameter_offset_number: 2,
        name: "3mm Short Endmill".to_string(),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::Flood,
        stickout_length_mm: 16.0,
        geometry: CutterGeometry::FlatEndmill {
            diameter_mm: 3.0, // flute_r = 1.5 mm, shank_r = 1.5 mm, holder_nut_r = 12.0 mm
            flute_length_mm: 12.0,
            overall_length_mm: 45.0,
            shank_diameter_mm: 3.0,
            flutes: 3,
        },
        presets: Vec::new(),
    };

    let mut vise_setup = Setup::new("s_vise", "Vise Holder Test", WorkOffsetSlot::G54);
    vise_setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    // Stock is 100x100x30 (Y ∈ [-50, +50], Z ∈ [-30, 0])
    vise_setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(100.0),
        depth_y: ParamLength::from_mm(100.0),
        height_z: ParamLength::from_mm(30.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::CenterZ,
    };
    // Vise jaws grip 28 mm up from stock bottom -> jaw top is at Z = -2.0 mm, inner face at Y = ±50 mm!
    vise_setup.fixtures.push(FixtureBody {
        id: "vise_tall".to_string(),
        name: "Tall Soft Jaws".to_string(),
        enabled: true,
        clearance_margin: ParamLength::from_mm(1.0),
        allowed_overcut: ParamLength::from_mm(0.0),
        kind: FixtureKind::ViseJaws {
            clamp_axis: ViseClampAxis::Y,
            jaw_width: ParamLength::from_mm(100.0),
            jaw_thickness: ParamLength::from_mm(18.0),
            jaw_height: ParamLength::from_mm(40.0),
            grip_depth: ParamLength::from_mm(28.0),
            step_shelf_depth: ParamLength::from_mm(0.0),
            step_shelf_height: ParamLength::from_mm(0.0),
            include_parallels: false,
            parallel_thickness: ParamLength::from_mm(3.175),
            center_offset_along_jaw: ParamLength::from_mm(0.0),
            fixed_jaw_side: ViseFixedSide::Positive,
        },
        color_rgba: None,
    });
    let res_vise = vise_setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let vise_fix =
        resolve_setup_fixtures_parametric(&vise_setup, &res_vise, DisplayUnit::Millimeter).unwrap();

    // Tool cuts inside pocket at Y = +42.0 mm (8 mm clear of Y = +50 jaw inner face -> flute_r=1.5 clears!)
    // At Z_tip = -22.0 mm, holder bottom is at Z = -22.0 + 16.0 = -6.0 mm (< jaw top Z = -2.0 mm!)
    // Holder nut radius = 12.0 mm -> penetrates jaw by `12.0 - 8.0 = 4.0 mm` (axial = 4.0 mm)!
    let mut tb_vise = ToolpathBuilder::new("op_deep_pocket");
    tb_vise.rapid_to(DVec3::new(-10.0, 42.0, 15.0));
    tb_vise.linear_to(DVec3::new(-10.0, 42.0, -22.0), 300.0, MotionRole::Plunge);
    tb_vise.linear_to(DVec3::new(10.0, 42.0, -22.0), 700.0, MotionRole::Cut);
    tb_vise.rapid_to(DVec3::new(10.0, 42.0, 15.0));
    let tp_vise = tb_vise.finish();

    let sim_vise = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &res_vise,
        target_bodies: &[],
        fixtures: &vise_fix,
        operations: &[SimOperationInput {
            op_id: "op_deep_pocket",
            op_name: "Deep Pocket",
            tool: &short_3mm,
            fallback_spindle_rpm: 15_000.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp_vise,
        }],
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        ..SetupSimulationInput::new(&doc, &res_vise, &[])
    })
    .unwrap();
    let holder_diag = sim_vise
        .diagnostics
        .iter()
        .find(|d| d.kind == SimIssueKind::FixtureCollision)
        .expect("holder vs vise jaw must trigger FixtureCollision");
    assert_eq!(holder_diag.fixture_id.as_deref(), Some("vise_tall"));
    assert!(
        holder_diag.message.contains("Holder/collet"),
        "expected Holder/collet diagnostic message, got: {}",
        holder_diag.message
    );
    assert!(
        (holder_diag.max_excess_mm - 4.0).abs() < 0.1,
        "expected ~4.0 mm holder penetration, got {}",
        holder_diag.max_excess_mm
    );

    // -------------------------------------------------------------------------
    // B. Spoilboard `allowed_overcut`: 0.2 mm skim passes, 1.0 mm plunge fails!
    // -------------------------------------------------------------------------
    let lib = default_starter_tool_library();
    let flat_6mm = &lib.tools[0];
    let mut sb_setup = Setup::new("s_sb", "Spoilboard Test", WorkOffsetSlot::G54);
    sb_setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    sb_setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(100.0),
        depth_y: ParamLength::from_mm(100.0),
        height_z: ParamLength::from_mm(12.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::CenterZ,
    };
    let mut sb = FixtureBody::default_spoilboard("sb_mdf", "Sacrificial MDF");
    sb.allowed_overcut = ParamLength::from_mm(0.3);
    sb_setup.fixtures.push(sb);
    let res_sb = sb_setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let sb_fix =
        resolve_setup_fixtures_parametric(&sb_setup, &res_sb, DisplayUnit::Millimeter).unwrap();

    // 0.2 mm skim below stock bottom (Z = -12.2 mm) -> passes!
    let mut tb_skim = ToolpathBuilder::new("op_skim");
    tb_skim.rapid_to(DVec3::new(-15.0, 0.0, 5.0));
    tb_skim.linear_to(DVec3::new(-15.0, 0.0, -12.2), 300.0, MotionRole::Plunge);
    tb_skim.linear_to(DVec3::new(15.0, 0.0, -12.2), 800.0, MotionRole::Cut);
    tb_skim.rapid_to(DVec3::new(15.0, 0.0, 5.0));
    let tp_skim = tb_skim.finish();
    let sim_skim = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &res_sb,
        target_bodies: &[],
        fixtures: &sb_fix,
        operations: &[SimOperationInput {
            op_id: "op_skim",
            op_name: "Through Cut Skim",
            tool: flat_6mm,
            fallback_spindle_rpm: 12_000.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp_skim,
        }],
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        ..SetupSimulationInput::new(&doc, &res_sb, &[])
    })
    .unwrap();
    assert!(
        !sim_skim
            .diagnostics
            .iter()
            .any(|d| d.kind == SimIssueKind::FixtureCollision),
        "0.2 mm spoilboard skim within 0.3 mm tolerance must not flag FixtureCollision: {:#?}",
        sim_skim.diagnostics
    );

    // 1.0 mm deep plunge into spoilboard (Z = -13.0 mm) -> triggers FixtureCollision!
    let mut tb_deep = ToolpathBuilder::new("op_deep_sb");
    tb_deep.rapid_to(DVec3::new(-15.0, 0.0, 5.0));
    tb_deep.linear_to(DVec3::new(-15.0, 0.0, -13.0), 300.0, MotionRole::Plunge);
    tb_deep.linear_to(DVec3::new(15.0, 0.0, -13.0), 800.0, MotionRole::Cut);
    tb_deep.rapid_to(DVec3::new(15.0, 0.0, 5.0));
    let tp_deep = tb_deep.finish();
    let sim_deep = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &res_sb,
        target_bodies: &[],
        fixtures: &sb_fix,
        operations: &[SimOperationInput {
            op_id: "op_deep_sb",
            op_name: "Too Deep Cut",
            tool: flat_6mm,
            fallback_spindle_rpm: 12_000.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp_deep,
        }],
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        ..SetupSimulationInput::new(&doc, &res_sb, &[])
    })
    .unwrap();
    let sb_diag = sim_deep
        .diagnostics
        .iter()
        .find(|d| d.kind == SimIssueKind::FixtureCollision)
        .expect("1.0 mm plunge into spoilboard must trigger FixtureCollision");
    assert_eq!(sb_diag.fixture_id.as_deref(), Some("sb_mdf"));
    assert!((sb_diag.max_excess_mm - 0.7).abs() < 0.05);

    // -------------------------------------------------------------------------
    // C. Clamp-in-cut-path: rapid router must NOT mutate cutting pass, and
    //    chip-sim MUST block with FixtureCollision CriticalError!
    // -------------------------------------------------------------------------
    let mut cut_setup = Setup::new("s_cut", "Cut Collision Test", WorkOffsetSlot::G54);
    cut_setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    cut_setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(100.0),
        depth_y: ParamLength::from_mm(100.0),
        height_z: ParamLength::from_mm(12.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::CenterZ,
    };
    cut_setup.fixtures.push(FixtureBody::default_toe_clamp(
        "clamp_in_cut",
        "Strap Clamp On Cut",
        DVec2::new(0.0, 0.0),
        0.0,
    ));
    let res_cut = cut_setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let cut_fix =
        resolve_setup_fixtures_parametric(&cut_setup, &res_cut, DisplayUnit::Millimeter).unwrap();
    let mut tb_cut = ToolpathBuilder::new("op_cut_clamp");
    tb_cut.rapid_to(DVec3::new(-25.0, 0.0, 35.0));
    tb_cut.linear_to(DVec3::new(-25.0, 0.0, -3.0), 300.0, MotionRole::Plunge);
    tb_cut.linear_to(DVec3::new(25.0, 0.0, -3.0), 800.0, MotionRole::Cut);
    tb_cut.rapid_to(DVec3::new(25.0, 0.0, 35.0));
    let mut tp_cut = tb_cut.finish();
    let orig_cut_events = tp_cut.events.clone();
    let route_res =
        route_toolpath_around_fixtures(&mut tp_cut, flat_6mm, &res_cut, &cut_fix, true, None);
    assert_eq!(route_res.rapid_lifts_inserted, 0);
    assert_eq!(tp_cut.events, orig_cut_events);

    let sim_cut = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &res_cut,
        target_bodies: &[],
        fixtures: &cut_fix,
        operations: &[SimOperationInput {
            op_id: "op_cut_clamp",
            op_name: "Slot Through Clamp",
            tool: flat_6mm,
            fallback_spindle_rpm: 12_000.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp_cut,
        }],
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        ..SetupSimulationInput::new(&doc, &res_cut, &[])
    })
    .unwrap();
    assert!(sim_cut.has_critical_errors());
    assert!(sim_cut.diagnostics.iter().any(|d| {
        d.kind == SimIssueKind::FixtureCollision && d.fixture_id.as_deref() == Some("clamp_in_cut")
    }));

    // -------------------------------------------------------------------------
    // D. 4th-Axis Cylindrical CPU vs GPU FixtureCollision parity!
    // -------------------------------------------------------------------------
    let shaft_stl = chip_fixtures::sample_rotary_shaft_stl_bytes();
    let shaft_mesh = chip_cad::load_mesh_bytes(
        "rotary_shaft.stl",
        &shaft_stl,
        &chip_cad::MeshImportOptions::default(),
    )
    .unwrap();
    let shaft_doc = chip_cad::CadDocument::from_mesh("rotary_shaft.stl".to_string(), shaft_mesh);
    let mut rot_setup = Setup::new("s_rot", "4th Axis Fixture Parity", WorkOffsetSlot::G54);
    rot_setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27 {
            x: chip_cam::AxisAnchor::Center,
            y: chip_cam::AxisAnchor::Center,
            z: chip_cam::AxisAnchor::Center,
        },
    };
    rot_setup.stock = StockConfig::RelativeCylinder {
        axis: StockAxis::X,
        radial_offset: ParamLength::from_mm(1.0),
        top_offset: ParamLength::from_mm(0.0),
        bottom_offset: ParamLength::from_mm(0.0),
    };
    rot_setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
    rot_setup.rotary.axis = RotaryAxisKind::XAxis;
    rot_setup
        .fixtures
        .push(FixtureBody::default_cylinder_keepout(
            "tailstock_pin",
            "Tailstock Center Keep-Out",
            DVec2::new(20.0, 0.0),
            0.0,
            35.0,
        ));
    let res_rot = rot_setup
        .resolve(&shaft_doc, DisplayUnit::Millimeter)
        .unwrap();
    let rot_fix =
        resolve_setup_fixtures_parametric(&rot_setup, &res_rot, DisplayUnit::Millimeter).unwrap();
    let mut tb_rot = ToolpathBuilder::new("op_rot_hit");
    tb_rot.rotary_rapid_to(DVec3::new(-20.0, 0.0, 26.0), 0.0);
    tb_rot.rotary_rapid_to(DVec3::new(30.0, 0.0, 26.0), 90.0); // Hits tailstock pin!
    let tp_rot = tb_rot.finish();

    let cyl_cpu = run_setup_simulation(&SetupSimulationInput {
        doc: &shaft_doc,
        setup: &res_rot,
        target_bodies: &[],
        fixtures: &rot_fix,
        operations: &[SimOperationInput {
            op_id: "op_rot_hit",
            op_name: "Rotary Traverse",
            tool: flat_6mm,
            fallback_spindle_rpm: 12_000.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp_rot,
        }],
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        engine: SimEngineKind::Heightmap,
        backend_choice: SimComputeBackendChoice::Cpu,
        ..SetupSimulationInput::new(&shaft_doc, &res_rot, &[])
    })
    .unwrap();
    let cyl_gpu = run_setup_simulation(&SetupSimulationInput {
        doc: &shaft_doc,
        setup: &res_rot,
        target_bodies: &[],
        fixtures: &rot_fix,
        operations: &[SimOperationInput {
            op_id: "op_rot_hit",
            op_name: "Rotary Traverse",
            tool: flat_6mm,
            fallback_spindle_rpm: 12_000.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp_rot,
        }],
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        engine: SimEngineKind::Heightmap,
        backend_choice: SimComputeBackendChoice::Auto,
        ..SetupSimulationInput::new(&shaft_doc, &res_rot, &[])
    })
    .unwrap();
    assert!(cyl_cpu.is_cylindrical());
    assert!(cyl_gpu.is_cylindrical());
    let rot_cpu_f: Vec<_> = cyl_cpu
        .diagnostics
        .iter()
        .filter(|d| d.kind == SimIssueKind::FixtureCollision)
        .collect();
    let rot_gpu_f: Vec<_> = cyl_gpu
        .diagnostics
        .iter()
        .filter(|d| d.kind == SimIssueKind::FixtureCollision)
        .collect();
    assert_eq!(rot_cpu_f.len(), 1);
    assert_eq!(rot_cpu_f[0].fixture_id.as_deref(), Some("tailstock_pin"));
    assert_eq!(rot_cpu_f, rot_gpu_f);
}

#[test]
fn m64_undercut_sim_fault_detection() {
    use chip_cam::{
        BoxAnchor27, FixedBoxXyAlign, FixedBoxZAlign, MotionRole, Setup, StockConfig,
        ToolpathBuilder, WcsOriginMode, WorkOffsetSlot,
    };
    use chip_math::{DVec3, DisplayUnit, ParamLength};
    use chip_sim::SimComputeBackendChoice;
    use chip_tools::{starter_dovetail_60deg_12mm, starter_lollipop_6mm, starter_tslot_12mm};

    // End-to-end Tri-Dexel undercut simulation in `chip-sim`:
    // Pre-slot with 6mm Flat -> carve T-slot with 12mm SlotMill -> zero critical errors,
    // then fault-inject a vertical plunge of Dovetail into solid stock -> critical error on CPU & GPU!
    let doc = chip_cad::CadDocument::empty();
    let lib = default_starter_tool_library();
    let flat_6 = &lib.tools[0];
    let slot_12 = starter_tslot_12mm();
    let dove_12 = starter_dovetail_60deg_12mm();
    let lolli_6 = starter_lollipop_6mm();

    let mut setup = Setup::new("s_m64", "Undercut Sim", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(100.0),
        depth_y: ParamLength::from_mm(100.0),
        height_z: ParamLength::from_mm(18.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::CenterZ,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    // Op 0: Pre-slot along X from outside stock (-60 -> +60) at Y=0, Z=-12
    let mut tb_pre = ToolpathBuilder::new("op_preslot");
    tb_pre.rapid_to(DVec3::new(-60.0, 0.0, 5.0));
    tb_pre.linear_to(DVec3::new(-60.0, 0.0, -12.0), 400.0, MotionRole::Plunge);
    tb_pre.linear_to(DVec3::new(60.0, 0.0, -12.0), 800.0, MotionRole::Cut);
    tb_pre.rapid_to(DVec3::new(60.0, 0.0, 5.0));
    let tp_pre = tb_pre.finish();

    // Op 1: Clean lateral T-slot entry from outside stock (-62 -> +62) at Y=0, Z=-12
    let mut tb_tslot = ToolpathBuilder::new("op_tslot");
    tb_tslot.rapid_to(DVec3::new(-62.0, 0.0, 5.0));
    tb_tslot.linear_to(DVec3::new(-62.0, 0.0, -12.0), 300.0, MotionRole::Plunge);
    tb_tslot.linear_to(DVec3::new(62.0, 0.0, -12.0), 600.0, MotionRole::Cut);
    tb_tslot.rapid_to(DVec3::new(62.0, 0.0, 5.0));
    let tp_tslot = tb_tslot.finish();

    // Op 2: Clean lateral Lollipop undercut pass along pre-slotted channel
    let mut tb_lolli = ToolpathBuilder::new("op_lolli");
    tb_lolli.rapid_to(DVec3::new(-60.0, 0.0, 5.0));
    tb_lolli.linear_to(DVec3::new(-60.0, 0.0, -12.0), 300.0, MotionRole::Plunge);
    tb_lolli.linear_to(DVec3::new(60.0, 0.0, -12.0), 600.0, MotionRole::Cut);
    tb_lolli.rapid_to(DVec3::new(60.0, 0.0, 5.0));
    let tp_lolli = tb_lolli.finish();

    for backend in [SimComputeBackendChoice::Cpu, SimComputeBackendChoice::Auto] {
        let res_ok = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[
                SimOperationInput {
                    op_id: "op_preslot",
                    op_name: "Pre-Slot",
                    tool: flat_6,
                    fallback_spindle_rpm: 12_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp_pre,
                },
                SimOperationInput {
                    op_id: "op_tslot",
                    op_name: "T-Slot",
                    tool: &slot_12,
                    fallback_spindle_rpm: 8_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp_tslot,
                },
                SimOperationInput {
                    op_id: "op_lolli",
                    op_name: "Lollipop Pass",
                    tool: &lolli_6,
                    fallback_spindle_rpm: 10_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp_lolli,
                },
            ],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::TriDexel,
            backend_choice: backend,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        })
        .unwrap();
        assert!(
            !res_ok.has_critical_errors(),
            "clean pre-slot + T-slot + lollipop pass must have zero critical errors ({backend:?}): {:#?}",
            res_ok.diagnostics
        );

        // Fault-injection: vertical plunge of Dovetail directly into solid uncut stock
        let mut tb_bad = ToolpathBuilder::new("op_bad_dove");
        tb_bad.rapid_to(DVec3::new(0.0, 8.0, 5.0));
        tb_bad.linear_to(DVec3::new(0.0, 8.0, -12.0), 200.0, MotionRole::Plunge);
        tb_bad.rapid_to(DVec3::new(0.0, 8.0, 5.0));
        let tp_bad = tb_bad.finish();

        let res_bad = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_bad_dove",
                op_name: "Bad Dovetail Plunge",
                tool: &dove_12,
                fallback_spindle_rpm: 8_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp_bad,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::TriDexel,
            backend_choice: backend,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        })
        .unwrap();
        assert!(
            res_bad.has_critical_errors(),
            "vertical dovetail plunge into uncut solid must trigger critical error ({backend:?})"
        );
        assert!(
            res_bad.diagnostics.iter().any(|d| matches!(
                d.kind,
                SimIssueKind::FluteLengthExceeded | SimIssueKind::UnsafeVerticalPlunge
            )),
            "expected FluteLengthExceeded / UnsafeVerticalPlunge diagnostic ({backend:?}), got {:#?}",
            res_bad.diagnostics
        );
    }
}

#[test]
fn deburr3d_curved_nonplanar_edge_and_stepped_wall_trimming_cpu_and_gpu_tridexel() {
    use std::sync::atomic::AtomicBool;

    use chip_cad::{CadKernel, CurveKind, MonstertruckKernel};
    use chip_cam::{
        BoxAnchor27, Deburr3DParams, MotionRole, Operation, OperationKind, SegmentMotion3D, Setup,
        StockConfig, StrategyParams, WcsOriginMode, WorkOffsetSlot, generate_operation_toolpath,
    };
    use chip_fixtures::sample_sloped_boss_stepped_pocket_step;
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit, ParamLength};
    use chip_tools::{CutterGeometry, default_starter_tool_library};

    let step = sample_sloped_boss_stepped_pocket_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes(
            "m65-curved-stepped-boss.step",
            step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();

    // Find the exact curved arch CircleArc parameters on Y = 0 for chord-sag verification
    let arch_arc = doc
        .edges
        .iter()
        .find_map(|e| match e.curve {
            CurveKind::CircleArc {
                center_mm,
                radius_mm,
                ..
            } if e.aabb_mm.min.y.abs() < 0.1 && e.aabb_mm.max.y.abs() < 0.1 => {
                Some((center_mm, radius_mm))
            }
            _ => None,
        })
        .expect("expected curved 3D circular arc edge on front Y=0 face");
    let (arc_center, arc_radius) = arch_arc;

    let mut setup = Setup::new("s_m65_curve", "M65 Curved & Stepped", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
    };
    setup.stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(0.0),
        side_neg_x: ParamLength::from_mm(0.0),
        side_pos_y: ParamLength::from_mm(0.0),
        side_neg_y: ParamLength::from_mm(0.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(0.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(23.0),
        round_increment_mm: None,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let lib = default_starter_tool_library();
    let chamfer_90 = lib
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
        .expect("starter ChamferMill present");
    let ball_6 = lib
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::BallEndmill { .. }))
        .expect("starter BallEndmill present");

    let cancel = AtomicBool::new(false);

    for (label, tool) in [("ChamferMill", chamfer_90), ("BallEndmill", ball_6)] {
        let mut op = Operation::new(
            format!("op_deburr_{label}"),
            format!("3D Deburr ({label})"),
            OperationKind::Deburr3D,
        );
        op.tool_id = Some(tool.id.clone());
        op.strategy = Some(StrategyParams::Deburr3D(Deburr3DParams {
            chamfer_width: ParamLength::from_mm(0.5),
            tip_offset: ParamLength::from_mm(0.75),
            wall_clearance: ParamLength::from_mm(0.15),
            auto_detect_edges: true,
            min_dihedral_angle_deg: 15.0,
            exclude_vertical_edges: true,
            chord_tolerance_mm: 0.005,
            ..Deburr3DParams::default()
        }));

        let tp = generate_operation_toolpath(
            &doc,
            &resolved,
            &op,
            tool,
            DisplayUnit::Millimeter,
            true,
            &cancel,
        )
        .unwrap_or_else(|e| panic!("{label} Deburr3D failed: {e}"));

        // 1. Verify curved 3D arch Z tracking & chord sag <= 0.01 mm
        let mut arch_cut_z_min = f64::INFINITY;
        let mut arch_cut_z_max = f64::NEG_INFINITY;
        let mut max_chord_sag = 0.0_f64;
        let mut front_shelf_max_x = f64::NEG_INFINITY;

        for seg in &tp.segments_wcs {
            if !matches!(
                seg.motion,
                SegmentMotion3D::Linear {
                    role: MotionRole::Cut | MotionRole::FinishCut,
                    ..
                }
            ) {
                continue;
            }
            // Curved arch segments on front Y=0 side (`x ∈ [36, 79]`, `y < 0`)
            if seg.start_wcs.y < 0.0
                && seg.end_wcs.y < 0.0
                && seg.start_wcs.x > 37.0
                && seg.end_wcs.x > 37.0
                && seg.start_wcs.x < 79.0
                && seg.end_wcs.x < 79.0
            {
                arch_cut_z_min = arch_cut_z_min.min(seg.start_wcs.z).min(seg.end_wcs.z);
                arch_cut_z_max = arch_cut_z_max.max(seg.start_wcs.z).max(seg.end_wcs.z);
                // Measure sagitta in XZ relative to exact arch curvature
                let chord_len_xz =
                    (seg.end_wcs.x - seg.start_wcs.x).hypot(seg.end_wcs.z - seg.start_wcs.z);
                let eff_r = arc_radius.max(1.0);
                let sag = eff_r
                    - (eff_r * eff_r - 0.25 * chord_len_xz * chord_len_xz)
                        .max(0.0)
                        .sqrt();
                max_chord_sag = max_chord_sag.max(sag);
                let _ = arc_center;
            }
            // Lower shelf segments on front Y=0 side (`x < 35`, `y < 0`, `z < 14.5`)
            if seg.start_wcs.y < 0.0
                && seg.end_wcs.y < 0.0
                && seg.start_wcs.x > 1.0
                && seg.end_wcs.x < 35.0
                && seg.start_wcs.z < 14.5
                && seg.end_wcs.z < 14.5
            {
                front_shelf_max_x = front_shelf_max_x.max(seg.start_wcs.x).max(seg.end_wcs.x);
            }
        }

        assert!(
            arch_cut_z_max - arch_cut_z_min > 3.0,
            "{label}: expected smooth 3D arch Z tracking > 3 mm, got {arch_cut_z_min:.3}..{arch_cut_z_max:.3}"
        );
        assert!(
            max_chord_sag <= 0.01,
            "{label}: expected 3D curve chord sag <= 0.01 mm, got {max_chord_sag:.5} mm"
        );

        // 2. Verify automatic inside-corner wall trimming stops cleanly before vertical wall X=35.0
        //    preserving exact 2D Euclidean `wall_clearance = 0.15 mm` for the tool radius!
        let tool_r = 0.5 * tool.geometry.cutting_diameter_mm();
        let mut front_shelf_min_wall_dist = f64::INFINITY;
        for seg in &tp.segments_wcs {
            if !matches!(
                seg.motion,
                SegmentMotion3D::Linear {
                    role: MotionRole::Cut | MotionRole::FinishCut,
                    ..
                }
            ) {
                continue;
            }
            if seg.start_wcs.y < 0.0
                && seg.end_wcs.y < 0.0
                && seg.start_wcs.x > 1.0
                && seg.end_wcs.x < 35.0
                && seg.start_wcs.z < 14.5
                && seg.end_wcs.z < 14.5
            {
                for pt in [seg.start_wcs, seg.end_wcs] {
                    // Nearest point on vertical boss wall (`x = 35.0, y >= 0.0`)
                    let dx = (35.0 - pt.x).max(0.0);
                    let dy = (0.0 - pt.y).max(0.0);
                    front_shelf_min_wall_dist = front_shelf_min_wall_dist.min(dx.hypot(dy));
                }
            }
        }
        assert!(
            front_shelf_max_x.is_finite() && front_shelf_max_x > 10.0,
            "{label}: expected front lower shelf edge to be machined, got max_x={front_shelf_max_x}"
        );
        assert!(
            front_shelf_min_wall_dist >= tool_r + 0.15 - 0.025,
            "{label}: lower shelf edge must trim before vertical boss wall X=35 preserving 0.15 mm clearance (tool_r={tool_r:.2}, min_wall_dist={front_shelf_min_wall_dist:.4}, max_x={front_shelf_max_x:.3})"
        );

        // 3. Verify 0 critical errors on BOTH CPU & GPU Tri-Dexel starting from exact target solid!
        let template = chip_sim::TriDexelStock::from_stock_spec(
            &chip_sim::stock_shape_spec_from_cam(&resolved.stock),
            SimResolution::Standard,
        )
        .unwrap();
        let init_stock =
            chip_sim::extract_tridexel_target_from_cad(&doc, &resolved, &[], &template)
                .expect("build TriDexel stock matching target solid");

        for backend in [SimComputeBackendChoice::Cpu, SimComputeBackendChoice::Auto] {
            let sim_res = run_setup_simulation(&SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                fixtures: &[],
                operations: &[SimOperationInput::from_operation(&op, tool, &tp)],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                initial_stock: Some(&init_stock),
                initial_raw_volume_mm3: None,
                carried_over_from_setup: None,
                cancel: None,
                engine: SimEngineKind::TriDexel,
                backend_choice: backend,
                ..SetupSimulationInput::new(&doc, &resolved, &[])
            })
            .unwrap();

            assert!(
                !sim_res.has_critical_errors(),
                "{label} ({backend:?}) Deburr3D must have 0 critical errors, got: {:#?}",
                sim_res.diagnostics
            );
            assert!(
                sim_res.removed_volume_mm3 > 10.0,
                "{label} ({backend:?}) expected measurable chamfer bevel volume removed, got {:.3} mm³",
                sim_res.removed_volume_mm3
            );
        }
    }
}

#[test]
fn deburr3d_lollipop_underside_back_chamfer_cpu_and_gpu_tridexel() {
    use std::sync::atomic::AtomicBool;

    use chip_cad::{CadKernel, MonstertruckKernel};
    use chip_cam::{
        BoxAnchor27, Deburr3DParams, MotionRole, Operation, OperationKind, SegmentMotion3D, Setup,
        StockConfig, StrategyParams, WcsOriginMode, WorkOffsetSlot, generate_operation_toolpath,
    };
    use chip_fixtures::sample_overhang_flange_step;
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit, ParamLength};
    use chip_tools::starter_lollipop_6mm;

    let step = sample_overhang_flange_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes(
            "m65-overhang-flange.step",
            step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();

    let mut setup = Setup::new("s_m65_lolli", "M65 Lollipop Underside", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
    };
    setup.stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(0.0),
        side_neg_x: ParamLength::from_mm(0.0),
        side_pos_y: ParamLength::from_mm(0.0),
        side_neg_y: ParamLength::from_mm(0.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(0.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(18.0),
        round_increment_mm: None,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let lolli_6 = starter_lollipop_6mm();
    let mut op = Operation::new(
        "op_deburr_underside",
        "Lollipop Back-Chamfer",
        OperationKind::Deburr3D,
    );
    op.tool_id = Some(lolli_6.id.clone());
    op.strategy = Some(StrategyParams::Deburr3D(Deburr3DParams {
        chamfer_width: ParamLength::from_mm(0.5),
        wall_clearance: ParamLength::from_mm(0.15),
        auto_detect_edges: true,
        min_dihedral_angle_deg: 15.0,
        exclude_vertical_edges: true,
        allow_underside_back_chamfer: true,
        ..Deburr3DParams::default()
    }));

    let cancel = AtomicBool::new(false);
    let tp = generate_operation_toolpath(
        &doc,
        &resolved,
        &op,
        &lolli_6,
        DisplayUnit::Millimeter,
        true,
        &cancel,
    )
    .unwrap();

    // Verify underside overhang edges at Z = 12.0 mm are actively machined (`tip_z ≈ 12 - 1.87 - 3 = 7.13 mm`)!
    let underside_cut_count = tp
        .segments_wcs
        .iter()
        .filter(|s| {
            matches!(
                s.motion,
                SegmentMotion3D::Linear {
                    role: MotionRole::Cut | MotionRole::FinishCut,
                    ..
                }
            ) && s.start_wcs.z < 9.0
                && s.end_wcs.z < 9.0
        })
        .count();
    assert!(
        underside_cut_count >= 10,
        "expected Lollipop underside back-chamfer cuts along overhang rims (tip Z < 9 mm), got {underside_cut_count}"
    );

    // Verify 0 critical errors (zero neck rub, shank rub, rapid crash, or gouge) in CPU & GPU Tri-Dexel!
    let template = chip_sim::TriDexelStock::from_stock_spec(
        &chip_sim::stock_shape_spec_from_cam(&resolved.stock),
        SimResolution::Standard,
    )
    .unwrap();
    let init_stock = chip_sim::extract_tridexel_target_from_cad(&doc, &resolved, &[], &template)
        .expect("build TriDexel stock matching overhang flange");

    for backend in [SimComputeBackendChoice::Cpu, SimComputeBackendChoice::Auto] {
        let sim_res = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput::from_operation(&op, &lolli_6, &tp)],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: Some(&init_stock),
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::TriDexel,
            backend_choice: backend,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        })
        .unwrap();

        assert!(
            !sim_res.has_critical_errors(),
            "Lollipop underside back-chamfer ({backend:?}) must have 0 critical errors, got: {:#?}",
            sim_res.diagnostics
        );
        assert!(
            sim_res.removed_volume_mm3 > 10.0,
            "Lollipop underside back-chamfer ({backend:?}) expected removed bevel volume > 10 mm³, got {:.3}",
            sim_res.removed_volume_mm3
        );
    }
}

#[test]
fn deburr3d_auto_detect_sharp_edges_ignores_tangent_fillets_and_vertical_walls() {
    use std::sync::atomic::AtomicBool;

    use chip_cad::{CadKernel, MonstertruckKernel};
    use chip_cam::{
        BoxAnchor27, Deburr3DParams, MotionRole, Operation, OperationKind, SegmentMotion3D, Setup,
        StrategyParams, WcsOriginMode, WorkOffsetSlot, generate_operation_toolpath,
    };
    use chip_fixtures::sample_machined_plate_step;
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit, ParamLength};
    use chip_tools::{CutterGeometry, default_starter_tool_library};

    let step = sample_machined_plate_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("plate.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();

    let mut setup = Setup::new("s_m65_auto", "Auto-Detect Sharp Edges", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let lib = default_starter_tool_library();
    let chamfer_90 = lib
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
        .expect("starter ChamferMill present");

    let mut op = Operation::new("op_auto_deburr", "Auto Deburr", OperationKind::Deburr3D);
    op.tool_id = Some(chamfer_90.id.clone());
    op.strategy = Some(StrategyParams::Deburr3D(Deburr3DParams {
        chamfer_width: ParamLength::from_mm(0.5),
        tip_offset: ParamLength::from_mm(0.75),
        wall_clearance: ParamLength::from_mm(0.15),
        auto_detect_edges: true,
        min_dihedral_angle_deg: 15.0,
        exclude_vertical_edges: true,
        ..Deburr3DParams::default()
    }));

    let cancel = AtomicBool::new(false);
    let tp = generate_operation_toolpath(
        &doc,
        &resolved,
        &op,
        chamfer_90,
        DisplayUnit::Millimeter,
        true,
        &cancel,
    )
    .unwrap();

    // All cutting moves must lie cleanly at the top Z=16 mm chamfer tip height (`16 - 0.5 - 0.75 = 14.75 mm`),
    // confirming that pure vertical side-wall edges and smooth tangent fillet seams were ignored!
    let mut cut_count = 0usize;
    for seg in &tp.segments_wcs {
        if matches!(
            seg.motion,
            SegmentMotion3D::Linear {
                role: MotionRole::Cut | MotionRole::FinishCut,
                ..
            }
        ) {
            cut_count += 1;
            let dz = (seg.end_wcs.z - seg.start_wcs.z).abs();
            assert!(
                dz < 1e-3,
                "plate has only horizontal top sharp edges; vertical edges must be excluded (got dz={dz:.4})"
            );
            assert!(
                (seg.start_wcs.z - 14.75).abs() < 0.05,
                "expected chamfer tip height ~14.75 mm on Z=16 plate top, got {:.3}",
                seg.start_wcs.z
            );
        }
    }
    assert!(
        cut_count >= 20,
        "expected full top perimeter + hole rims deburred, got {cut_count}"
    );

    let template = chip_sim::TriDexelStock::from_stock_spec(
        &chip_sim::stock_shape_spec_from_cam(&resolved.stock),
        SimResolution::Standard,
    )
    .unwrap();
    let init_stock = chip_sim::extract_tridexel_target_from_cad(&doc, &resolved, &[], &template)
        .expect("build TriDexel stock matching plate");
    let sim_res = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved,
        target_bodies: &[],
        fixtures: &[],
        operations: &[SimOperationInput::from_operation(&op, chamfer_90, &tp)],
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: Some(&init_stock),
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        engine: SimEngineKind::TriDexel,
        backend_choice: SimComputeBackendChoice::Cpu,
        ..SetupSimulationInput::new(&doc, &resolved, &[])
    })
    .unwrap();
    assert!(
        !sim_res.has_critical_errors(),
        "auto-detected plate Deburr3D must have 0 critical errors, got: {:#?}",
        sim_res.diagnostics
    );
}

#[test]
fn m66_tslot_undercut_cpu_and_gpu_tridexel_parity() {
    use std::sync::atomic::AtomicBool;

    use chip_cam::{
        BoxAnchor27, FixedBoxXyAlign, FixedBoxZAlign, GeometrySelectionItem, HeightReference,
        HeightSpec, Operation, OperationGenRequest, OperationKind, SegmentMotion3D, Setup,
        SlotUndercutMode, SlotUndercutParams, SlotUndercutPreset, StockConfig, StrategyParams,
        UndercutEntryMode, WcsOriginMode, WorkOffsetSlot, generate_operation_toolpath_with_rest,
    };
    use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DisplayUnit, ParamLength};
    use chip_sim::SimComputeBackendChoice;
    use chip_sketch::{SketchEntity, SketchEntityEntry, SketchLayer};
    use chip_tools::{default_starter_tool_library, starter_tslot_12mm};

    let doc = chip_cad::CadDocument::empty();
    let lib = default_starter_tool_library();
    let flat_6 = &lib.tools[0]; // Ø6 mm Flat Endmill for pre-clearing 8.5 mm throat
    let slot_12 = starter_tslot_12mm(); // Ø12 mm, 4 mm thick, Ø5 mm neck, 18 mm neck length

    let mut setup = Setup::new("s_m66_tslot", "M66 M8 T-Slot", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(100.0),
        depth_y: ParamLength::from_mm(100.0),
        height_z: ParamLength::from_mm(20.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::CenterZ,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let mut sk = SketchLayer::new("sk_m8", "M8 T-Slot Axis", "s_m66_tslot");
    sk.entities.push(SketchEntityEntry {
        id: "e_axis".to_string(),
        name: "Slot Axis".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::new_polyline(ArcPolyline2D {
            segments: vec![ArcSegment2D::Line {
                start: DVec2::new(-50.0, 0.0),
                end: DVec2::new(50.0, 0.0),
            }],
            closed: false,
        }),
    });
    sk.rebuild_regions(None);
    let sel = vec![GeometrySelectionItem::SketchEntity {
        sketch_id: sk.id.clone(),
        entity_id: "e_axis".to_string(),
        reverse: false,
        side_flip: false,
    }];

    // Op 1: Pre-clear vertical 8.5 mm throat down to Z = -14.0 mm with Ø6 mm Flat Endmill (Keyway mode)
    let mut op_throat = Operation::new(
        "op_throat",
        "Pre-Clear 8.5mm Throat",
        OperationKind::SlotUndercut,
    );
    op_throat.tool_id = Some(flat_6.id.clone());
    op_throat.selections = sel.clone();
    op_throat.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_throat.heights.bottom = HeightSpec::new(HeightReference::StockTop, -14.0);
    op_throat.strategy = Some(StrategyParams::SlotUndercut(SlotUndercutParams {
        mode: SlotUndercutMode::Keyway,
        preset: SlotUndercutPreset::Custom,
        slot_width: ParamLength::from_mm(8.5),
        throat_width: ParamLength::from_mm(8.5),
        slot_height: ParamLength::from_mm(14.0),
        radial_stepover: ParamLength::from_mm(1.5),
        axial_stepdown: ParamLength::from_mm(7.0),
        finish_allowance: ParamLength::from_mm(0.0),
        spring_pass: false,
        entry_mode: UndercutEntryMode::BothEndsThrough,
        entry_extension: ParamLength::from_mm(6.0),
        climb_direction: true,
    }));

    // Op 2: Machine DIN 650 M8 T-Slot (14.5 mm wide × 6.0 mm tall beneath 8.5 mm throat) with SlotMill
    let mut p_m8 = SlotUndercutParams::default();
    p_m8.apply_preset(SlotUndercutPreset::TSlotM8);
    p_m8.entry_mode = UndercutEntryMode::BothEndsThrough;
    p_m8.entry_extension = ParamLength::from_mm(8.0);
    p_m8.spring_pass = true;
    let mut op_tslot = Operation::new(
        "op_tslot",
        "M8 T-Slot Undercut",
        OperationKind::SlotUndercut,
    );
    op_tslot.tool_id = Some(slot_12.id.clone());
    op_tslot.selections = sel;
    op_tslot.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_tslot.heights.bottom = HeightSpec::new(HeightReference::StockTop, -14.0);
    op_tslot.strategy = Some(StrategyParams::SlotUndercut(p_m8));

    let cancel = AtomicBool::new(false);
    let tp_throat = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: std::slice::from_ref(&sk),
        setup: &resolved,
        op: &op_throat,
        tool: flat_6,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();
    let tp_tslot = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[sk],
        setup: &resolved,
        op: &op_tslot,
        tool: &slot_12,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();

    // Verify ZERO vertical plunges or retracts beneath the T-slot roof (`|x| < 56 mm`)!
    for seg in &tp_tslot.segments_wcs {
        if (seg.end_wcs.z - seg.start_wcs.z).abs() > 1e-4 {
            assert!(
                seg.start_wcs.x.abs() >= 56.0 - 1e-3 && seg.end_wcs.x.abs() >= 56.0 - 1e-3,
                "T-slot must NEVER plunge or retract vertically inside roof (got {seg:?})"
            );
        }
        assert!(matches!(
            seg.motion,
            SegmentMotion3D::Rapid | SegmentMotion3D::Linear { .. } | SegmentMotion3D::Arc { .. }
        ));
    }

    let sim_ops = [
        SimOperationInput::from_operation(&op_throat, flat_6, &tp_throat),
        SimOperationInput::from_operation(&op_tslot, &slot_12, &tp_tslot),
    ];

    let mut cpu_vol: Option<f64> = None;
    for backend in [SimComputeBackendChoice::Cpu, SimComputeBackendChoice::Auto] {
        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &sim_ops,
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::TriDexel,
            backend_choice: backend,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        })
        .unwrap();

        assert!(
            !sim.has_critical_errors(),
            "M8 T-Slot ({backend:?}) must have 0 critical errors, got: {:#?}",
            sim.diagnostics
        );
        let td = sim.as_tridexel().expect("TriDexel stock present");

        // Probe exact cross-section at X = 0 using Y-axis rays!
        let y_plane = td.y_plane();
        let u_x0 = ((0.0 - y_plane.origin_u_mm) / y_plane.du_mm)
            .round()
            .clamp(0.0, (y_plane.nu - 1) as f64) as usize;

        // Helper: find Y-spans at `(x = 0, z)`
        let spans_at_z = |z_query: f64| {
            let v_z = ((z_query - y_plane.origin_v_mm) / y_plane.dv_mm)
                .round()
                .clamp(0.0, (y_plane.nv - 1) as f64) as usize;
            y_plane.ray_spans(u_x0, v_z)
        };

        // 1. Intact overhanging roof at Z = -4.0 mm: throat gap is `[-4.25, +4.25]` (`8.5 mm` wide!)
        let roof_spans = spans_at_z(-4.0);
        assert_eq!(
            roof_spans.len(),
            2,
            "roof at Z=-4 must split into 2 solid shoulders across the 8.5 mm throat ({backend:?}): {roof_spans:?}"
        );
        let throat_width_meas = roof_spans[1].min_mm - roof_spans[0].max_mm;
        assert!(
            (throat_width_meas - 8.5).abs() < 0.15,
            "expected 8.5 mm throat width at Z=-4 ({backend:?}), measured {throat_width_meas:.3} mm"
        );

        // 2. Undercut head cavity at Z = -11.0 mm: exact M8 width `[-7.25, +7.25]` (`14.5 mm` wide!)
        let cav_spans = spans_at_z(-11.0);
        assert_eq!(
            cav_spans.len(),
            2,
            "undercut cavity at Z=-11 must have 2 outer walls ({backend:?}): {cav_spans:?}"
        );
        let undercut_width_meas = cav_spans[1].min_mm - cav_spans[0].max_mm;
        assert!(
            (undercut_width_meas - 14.5).abs() < 0.15,
            "expected 14.5 mm M8 undercut width at Z=-11 ({backend:?}), measured {undercut_width_meas:.3} mm"
        );

        // 3. Solid uncut floor beneath Z = -14.0 mm (at Z = -17.0 mm)
        let floor_spans = spans_at_z(-17.0);
        assert_eq!(
            floor_spans.len(),
            1,
            "floor at Z=-17 must remain a single solid span ({backend:?}): {floor_spans:?}"
        );

        if let Some(v0) = cpu_vol {
            let rel = (sim.removed_volume_mm3 - v0).abs() / v0.max(1.0);
            assert!(
                rel < 0.005,
                "CPU vs GPU T-Slot removed volume mismatch: cpu={v0:.2}, gpu={:.2} (rel={:.4})",
                sim.removed_volume_mm3,
                rel
            );
        } else {
            cpu_vol = Some(sim.removed_volume_mm3);
        }
    }
}

#[test]
fn m66_dovetail_60deg_cpu_and_gpu_tridexel_parity() {
    use std::sync::atomic::AtomicBool;

    use chip_cam::{
        BoxAnchor27, CamError, DovetailMode, DovetailParams, FixedBoxXyAlign, FixedBoxZAlign,
        GeometrySelectionItem, HeightReference, HeightSpec, Operation, OperationGenRequest,
        OperationKind, Setup, SlotUndercutMode, SlotUndercutParams, SlotUndercutPreset,
        StockConfig, StrategyParams, UndercutEntryMode, WcsOriginMode, WorkOffsetSlot,
        generate_operation_toolpath_with_rest,
    };
    use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DisplayUnit, ParamLength};
    use chip_sim::SimComputeBackendChoice;
    use chip_sketch::{SketchEntity, SketchEntityEntry, SketchLayer};
    use chip_tools::{CutterGeometry, default_starter_tool_library, starter_dovetail_60deg_12mm};

    let doc = chip_cad::CadDocument::empty();
    let lib = default_starter_tool_library();
    let flat_6 = &lib.tools[0];
    let dove_60 = starter_dovetail_60deg_12mm(); // Ø12, 60°, H=5, Rc=0.2, neck=5.5, neck_len=18

    let mut setup = Setup::new("s_m66_dove", "M66 60deg Dovetail", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(100.0),
        depth_y: ParamLength::from_mm(100.0),
        height_z: ParamLength::from_mm(20.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::CenterZ,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let mut sk = SketchLayer::new("sk_dove", "Dovetail Axis", "s_m66_dove");
    sk.entities.push(SketchEntityEntry {
        id: "e_axis".to_string(),
        name: "Slot Axis".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::new_polyline(ArcPolyline2D {
            segments: vec![ArcSegment2D::Line {
                start: DVec2::new(-50.0, 0.0),
                end: DVec2::new(50.0, 0.0),
            }],
            closed: false,
        }),
    });
    sk.rebuild_regions(None);
    let sel = vec![GeometrySelectionItem::SketchEntity {
        sketch_id: sk.id.clone(),
        entity_id: "e_axis".to_string(),
        reverse: false,
        side_flip: false,
    }];

    // Op 1: Pre-clear vertical 8.5 mm throat down to Z = -12.0 mm
    let mut op_throat = Operation::new(
        "op_d_throat",
        "Pre-Clear Throat",
        OperationKind::SlotUndercut,
    );
    op_throat.tool_id = Some(flat_6.id.clone());
    op_throat.selections = sel.clone();
    op_throat.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_throat.heights.bottom = HeightSpec::new(HeightReference::StockTop, -12.0);
    op_throat.strategy = Some(StrategyParams::SlotUndercut(SlotUndercutParams {
        mode: SlotUndercutMode::Keyway,
        preset: SlotUndercutPreset::Custom,
        slot_width: ParamLength::from_mm(8.5),
        throat_width: ParamLength::from_mm(8.5),
        slot_height: ParamLength::from_mm(12.0),
        radial_stepover: ParamLength::from_mm(1.5),
        axial_stepdown: ParamLength::from_mm(6.0),
        finish_allowance: ParamLength::from_mm(0.0),
        spring_pass: false,
        entry_mode: UndercutEntryMode::BothEndsThrough,
        entry_extension: ParamLength::from_mm(6.0),
        climb_direction: true,
    }));

    // Op 2: Machine 60° internal dovetail slot (`top_throat = 8.5 mm`, `depth = 5.0 mm`, `z_bottom = -12.0 mm`)
    let mut op_dove = Operation::new("op_d_60", "60° Dovetail Slot", OperationKind::Dovetail);
    op_dove.tool_id = Some(dove_60.id.clone());
    op_dove.selections = sel;
    op_dove.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_dove.heights.bottom = HeightSpec::new(HeightReference::StockTop, -12.0);
    op_dove.strategy = Some(StrategyParams::Dovetail(DovetailParams {
        mode: DovetailMode::InternalSlot,
        top_throat_width: ParamLength::from_mm(8.5),
        dovetail_depth: ParamLength::from_mm(5.0),
        side_angle_deg: 60.0,
        radial_stepover: ParamLength::from_mm(0.6),
        finish_allowance: ParamLength::from_mm(0.1),
        spring_pass: true,
        entry_mode: UndercutEntryMode::BothEndsThrough,
        entry_extension: ParamLength::from_mm(8.0),
        climb_direction: true,
    }));

    let cancel = AtomicBool::new(false);

    // Angle mismatch check: 45° cutter on 60° slot MUST fail cleanly!
    let mut dove_45 = dove_60.clone();
    if let CutterGeometry::DovetailCutter { side_angle_deg, .. } = &mut dove_45.geometry {
        *side_angle_deg = 45.0;
    }
    let bad_err = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: std::slice::from_ref(&sk),
        setup: &resolved,
        op: &op_dove,
        tool: &dove_45,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap_err();
    assert!(matches!(bad_err, CamError::InvalidUndercutGeometry { .. }));

    let tp_throat = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: std::slice::from_ref(&sk),
        setup: &resolved,
        op: &op_throat,
        tool: flat_6,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();
    let tp_dove = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[sk],
        setup: &resolved,
        op: &op_dove,
        tool: &dove_60,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();

    let sim_ops = [
        SimOperationInput::from_operation(&op_throat, flat_6, &tp_throat),
        SimOperationInput::from_operation(&op_dove, &dove_60, &tp_dove),
    ];

    let tan60 = 60.0_f64.to_radians().tan();
    let mut cpu_vol: Option<f64> = None;
    for backend in [SimComputeBackendChoice::Cpu, SimComputeBackendChoice::Auto] {
        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &sim_ops,
            resolution: SimResolution::Fine,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::TriDexel,
            backend_choice: backend,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        })
        .unwrap();

        assert!(
            !sim.has_critical_errors(),
            "60° Dovetail ({backend:?}) must have 0 critical errors, got: {:#?}",
            sim.diagnostics
        );

        // Measure the exact angled flank in TriDexelStock Y-rays across multiple Z stations inside [-11.3, -7.5]!
        let td = sim.as_tridexel().expect("TriDexel stock present");
        let y_plane = td.y_plane();
        let u_x0 = ((0.0 - y_plane.origin_u_mm) / y_plane.du_mm)
            .round()
            .clamp(0.0, (y_plane.nu - 1) as f64) as usize;

        for z_probe in [-11.0_f64, -9.5, -8.0] {
            let v_z = ((z_probe - y_plane.origin_v_mm) / y_plane.dv_mm)
                .round()
                .clamp(0.0, (y_plane.nv - 1) as f64) as usize;
            let z_actual = y_plane.center_v(v_z);
            let dz_from_floor = z_actual - (-12.0);
            let expected_half_w = 0.5 * 8.5 + (5.0 - dz_from_floor) / tan60;

            let spans = y_plane.ray_spans(u_x0, v_z);
            assert_eq!(spans.len(), 2, "expected 2 walls at z={z_actual:.3}");
            let pos_wall = spans[1].min_mm;
            let neg_wall = -spans[0].max_mm;
            assert!(
                (pos_wall - expected_half_w).abs() < 0.05,
                "60° dovetail +Y flank at z={z_actual:.3} ({backend:?}): got {pos_wall:.4}, expected {expected_half_w:.4} (err={:.4} mm)",
                (pos_wall - expected_half_w).abs()
            );
            assert!(
                (neg_wall - expected_half_w).abs() < 0.05,
                "60° dovetail -Y flank at z={z_actual:.3} ({backend:?}): got {neg_wall:.4}, expected {expected_half_w:.4} (err={:.4} mm)",
                (neg_wall - expected_half_w).abs()
            );
        }

        if let Some(v0) = cpu_vol {
            let rel = (sim.removed_volume_mm3 - v0).abs() / v0.max(1.0);
            assert!(
                rel < 0.005,
                "CPU vs GPU Dovetail removed volume mismatch: cpu={v0:.2}, gpu={:.2}",
                sim.removed_volume_mm3
            );
        } else {
            cpu_vol = Some(sim.removed_volume_mm3);
        }
    }
}

#[test]
fn m66_undercut3d_lollipop_neck_clipping_and_tridexel_parity() {
    use std::sync::atomic::AtomicBool;

    use chip_cad::{MeshImportOptions, load_mesh_cad_document};
    use chip_cam::{
        BoxAnchor27, HeightReference, HeightSpec, MotionRole, Operation, OperationKind,
        SegmentMotion3D, Setup, StockConfig, StrategyParams, Undercut3DParams, WcsOriginMode,
        WorkOffsetSlot, generate_operation_toolpath,
    };
    use chip_math::{DVec3, DisplayUnit, ParamLength};
    use chip_sim::SimComputeBackendChoice;
    use chip_tools::{CutterGeometry, starter_lollipop_6mm};

    // Build a watertight 3D prism along X ∈ [0, 60] whose +Y front face has a continuous 45°
    // overhanging underside wall:
    // - Roof overhang top at Z = 20.0 mm extending out to Y = 20.0 mm across X ∈ [0, 60]
    // - Overhang lip drops vertically from Z = 20 to Z = 16 at Y = 20.0 mm
    // - From Z = 16 down to Z = 10, the wall slopes INWARD at 45° beneath the roof (`Y(z) = 20 - (16 - z)`):
    //   * At Z = 15.0: undercut depth = 1.0 mm (REACHABLE by Ø6/neck Ø3 Lollipop + 0.25 mm clr!)
    //   * At Z = 11.0: undercut depth = 5.0 mm (DELIBERATELY TOO DEEP — would ram the Ø3 mm neck
    //     into the overhanging lip at Y = 20.0 mm, so Undercut3D MUST clip it automatically!)
    let mut tris: Vec<[DVec3; 3]> = Vec::new();
    let push_quad = |tris: &mut Vec<[DVec3; 3]>, a: DVec3, b: DVec3, c: DVec3, d: DVec3| {
        tris.push([a, b, c]);
        tris.push([a, c, d]);
    };

    // Profile polygon in YZ plane (ordered CCW when looking down +X so extruding x0->x1 gives outward normals):
    // (y, z): (0,0) -> (14,0) -> (14,10) -> (20,16) -> (20,20) -> (0,20)
    let prof_yz: [(f64, f64); 6] = [
        (0.0, 0.0),
        (14.0, 0.0),
        (14.0, 10.0),
        (20.0, 16.0),
        (20.0, 20.0),
        (0.0, 20.0),
    ];
    let x0 = 0.0;
    let x1 = 60.0;
    let n_p = prof_yz.len();
    for i in 0..n_p {
        let (y_a, z_a) = prof_yz[i];
        let (y_b, z_b) = prof_yz[(i + 1) % n_p];
        // Outward normal for CCW profile in YZ extruded along +X: `(p_b - p_a) × (+X)`
        push_quad(
            &mut tris,
            DVec3::new(x0, y_a, z_a),
            DVec3::new(x0, y_b, z_b),
            DVec3::new(x1, y_b, z_b),
            DVec3::new(x1, y_a, z_a),
        );
    }
    // End caps at x0 (-X normal) and x1 (+X normal) via fan from interior point (7, 10)
    let c0 = DVec3::new(x0, 7.0, 10.0);
    let c1 = DVec3::new(x1, 7.0, 10.0);
    for i in 0..n_p {
        let (y_a, z_a) = prof_yz[i];
        let (y_b, z_b) = prof_yz[(i + 1) % n_p];
        tris.push([c0, DVec3::new(x0, y_b, z_b), DVec3::new(x0, y_a, z_a)]);
        tris.push([c1, DVec3::new(x1, y_a, z_a), DVec3::new(x1, y_b, z_b)]);
    }

    let mut stl = String::from("solid m66_overhang\n");
    for t in &tris {
        let n = (t[1] - t[0]).cross(t[2] - t[0]).normalize_or_zero();
        stl.push_str(&format!(
            "  facet normal {:.6} {:.6} {:.6}\n    outer loop\n      vertex {:.6} {:.6} {:.6}\n      vertex {:.6} {:.6} {:.6}\n      vertex {:.6} {:.6} {:.6}\n    endloop\n  endfacet\n",
            n.x, n.y, n.z,
            t[0].x, t[0].y, t[0].z,
            t[1].x, t[1].y, t[1].z,
            t[2].x, t[2].y, t[2].z,
        ));
    }
    stl.push_str("endsolid m66_overhang\n");

    let doc = load_mesh_cad_document(
        "m66_overhang_test.stl",
        stl.as_bytes(),
        &MeshImportOptions::default(),
    )
    .unwrap();

    let mut setup = Setup::new("s_m66_u3d", "M66 3D Undercut", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
    };
    setup.stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(0.0),
        side_neg_x: ParamLength::from_mm(0.0),
        side_pos_y: ParamLength::from_mm(0.0),
        side_neg_y: ParamLength::from_mm(0.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(0.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(20.0),
        round_increment_mm: None,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let lolli_6 = starter_lollipop_6mm(); // Ø6.0 (R=3.0), neck Ø3.0 (r_neck=1.5), reach=18.0
    let (r_sphere, r_neck) = match lolli_6.geometry {
        CutterGeometry::LollipopEndmill {
            diameter_mm,
            neck_diameter_mm,
            ..
        } => (0.5 * diameter_mm, 0.5 * neck_diameter_mm),
        _ => unreachable!(),
    };
    let neck_clr = 0.25;

    let mut op = Operation::new("op_u3d", "3D Lollipop Undercut", OperationKind::Undercut3D);
    op.tool_id = Some(lolli_6.id.clone());
    op.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
    op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
    op.strategy = Some(StrategyParams::Undercut3D(Undercut3DParams {
        z_stepdown: ParamLength::from_mm(0.35),
        use_cusp_height: false,
        target_cusp: ParamLength::from_mm(0.025),
        min_overhang_angle_deg: 5.0,
        neck_clearance: ParamLength::from_mm(neck_clr),
        stock_to_leave: ParamLength::from_mm(0.0),
        lead_out_distance: ParamLength::from_mm(4.0),
        climb_direction: true,
        ..Undercut3DParams::default()
    }));

    let cancel = AtomicBool::new(false);
    let tp = generate_operation_toolpath(
        &doc,
        &resolved,
        &op,
        &lolli_6,
        DisplayUnit::Millimeter,
        true,
        &cancel,
    )
    .unwrap();

    // 1. Verify active overhang cutting passes occurred AND that automatic neck-clearance clipping
    //    prevented ANY cutter center from moving closer than `Y = 20.0 + r_neck + neck_clr = 21.75 mm`
    //    while beneath the overhanging lip (`Z_lip = 16.0..20.0`)!
    let mut cut_count = 0usize;
    let mut min_cut_y = f64::INFINITY;
    for seg in &tp.segments_wcs {
        if matches!(
            seg.motion,
            SegmentMotion3D::Linear {
                role: MotionRole::FinishCut | MotionRole::Cut,
                ..
            }
        ) {
            cut_count += 1;
            min_cut_y = min_cut_y.min(seg.start_wcs.y).min(seg.end_wcs.y);
            // Neck outer edge `tip_y - r_neck` must NEVER intrude inside `Y = 20.0 + neck_clr`!
            assert!(
                seg.start_wcs.y >= 20.0 + r_neck + neck_clr - 0.02
                    && seg.end_wcs.y >= 20.0 + r_neck + neck_clr - 0.02,
                "neck-clearance clipping violated: lip at Y=20 requires tip_y >= {:.3}, got seg {:?}",
                20.0 + r_neck + neck_clr,
                seg
            );
        }
        // Every vertical plunge or retract MUST be strictly in open air (`y >= 20.0 + r_sphere + neck_clr`)!
        if (seg.end_wcs.z - seg.start_wcs.z).abs() > 1e-4 {
            assert!(
                seg.start_wcs.y >= 20.0 + r_sphere + neck_clr - 0.02,
                "vertical plunge/retract must occur in open air past overhang lip (Y >= {:.2}), got {seg:?}",
                20.0 + r_sphere + neck_clr
            );
        }
    }
    assert!(
        cut_count >= 5,
        "expected multiple Z-level overhang cuts along reachable 45° slope, got {cut_count}"
    );

    // 2. Simulate starting from a solid box matching the roof overhang (`Y ∈ [0, 20], Z ∈ [0, 20]`)
    //    so the Lollipop actively carves the reachable 45° overhang cavity underneath the intact roof!
    let mut cpu_vol: Option<f64> = None;
    for backend in [SimComputeBackendChoice::Cpu, SimComputeBackendChoice::Auto] {
        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput::from_operation(&op, &lolli_6, &tp)],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::TriDexel,
            backend_choice: backend,
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        })
        .unwrap();

        assert!(
            !sim.has_critical_errors(),
            "Undercut3D Lollipop ({backend:?}) must have 0 critical errors (0 neck rubs, 0 roof crashes, 0 gouges), got: {:#?}",
            sim.diagnostics
        );
        assert!(
            sim.removed_volume_mm3 > 20.0,
            "Undercut3D Lollipop ({backend:?}) must carve measurable overhang material, got {:.3} mm³",
            sim.removed_volume_mm3
        );

        // Verify roof above the overhang at `Z = 18.0 mm, X = 30.0 mm` stays completely intact out to `Y ≈ 20.0 mm`!
        let td = sim.as_tridexel().expect("TriDexel stock present");
        let y_plane = td.y_plane();
        let u_x30 = ((30.0 - y_plane.origin_u_mm) / y_plane.du_mm)
            .round()
            .clamp(0.0, (y_plane.nu - 1) as f64) as usize;
        let v_z18 = ((18.0 - y_plane.origin_v_mm) / y_plane.dv_mm)
            .round()
            .clamp(0.0, (y_plane.nv - 1) as f64) as usize;
        let roof_spans = y_plane.ray_spans(u_x30, v_z18);
        assert_eq!(roof_spans.len(), 1);
        assert!(
            roof_spans[0].max_mm > 19.8,
            "overhanging roof at Z=18 must remain intact out to Y=20.0 ({backend:?}), got {:.3}",
            roof_spans[0].max_mm
        );

        if let Some(v0) = cpu_vol {
            let rel = (sim.removed_volume_mm3 - v0).abs() / v0.max(1.0);
            assert!(
                rel < 0.01,
                "CPU vs GPU Undercut3D removed volume mismatch: cpu={v0:.2}, gpu={:.2}",
                sim.removed_volume_mm3
            );
        } else {
            cpu_vol = Some(sim.removed_volume_mm3);
        }
    }
}

#[test]
fn periodic_helical_sweep_step_parallel3d_respects_helix_geometry_with_zero_gouges() {
    use std::sync::atomic::AtomicBool;

    use chip_cad::{CadKernel, MonstertruckKernel};
    use chip_cam::{
        BoxAnchor27, MotionRole, Operation, OperationKind, Parallel3DParams, SegmentMotion3D,
        Setup, StrategyParams, WcsOriginMode, WorkOffsetSlot, generate_operation_toolpath,
    };
    use chip_fixtures::sample_periodic_helical_sweep_step;
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit, ParamLength};
    use chip_tools::{CutterGeometry, default_starter_tool_library};

    let step = sample_periodic_helical_sweep_step();
    let doc = MonstertruckKernel
        .load_step_bytes(
            "helical_sweep.step",
            step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();

    let mut setup = Setup::new("s_helix", "Helical Sweep 3D", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let lib = default_starter_tool_library();
    let mut ball_6 = lib
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::BallEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 1.0
            )
        })
        .expect("starter 6 mm ball endmill present")
        .clone();
    // Use long-reach stickout & flute length suitable for the 40 mm tall helical part.
    if let CutterGeometry::BallEndmill {
        flute_length_mm,
        overall_length_mm,
        ..
    } = &mut ball_6.geometry
    {
        *flute_length_mm = 45.0;
        *overall_length_mm = 70.0;
    }
    ball_6.stickout_length_mm = 55.0;

    let mut op = Operation::new(
        "op_helix_p3d",
        "Helix Parallel 3D",
        OperationKind::Parallel3D,
    );
    op.tool_id = Some(ball_6.id.clone());
    op.strategy = Some(StrategyParams::Parallel3D(Parallel3DParams {
        use_cusp_height: false,
        stepover: ParamLength::from_mm(3.0),
        pass_angle_deg: 0.0,
        ..Parallel3DParams::default()
    }));

    let cancel = AtomicBool::new(false);
    let tp = generate_operation_toolpath(
        &doc,
        &resolved,
        &op,
        &ball_6,
        DisplayUnit::Millimeter,
        true,
        &cancel,
    )
    .unwrap();

    // Verify cutting moves actively follow the helical tube at intermediate Z elevations
    // (10..35 mm), proving the toolpath sees the helical wall instead of dropping through it.
    let mid_helix_cuts = tp
        .segments_wcs
        .iter()
        .filter(|seg| {
            matches!(
                seg.motion,
                SegmentMotion3D::Linear {
                    role: MotionRole::Cut | MotionRole::FinishCut,
                    ..
                }
            ) && seg.end_wcs.z > 10.0
                && seg.end_wcs.z < 35.0
        })
        .count();
    assert!(
        mid_helix_cuts >= 10,
        "expected Parallel3D passes riding over mid-height helical geometry (Z ∈ 10..35 mm), got {mid_helix_cuts}"
    );

    let sim = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved,
        target_bodies: &[],
        fixtures: &[],
        operations: &[SimOperationInput::from_operation(&op, &ball_6, &tp)],
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        engine: SimEngineKind::Heightmap,
        backend_choice: SimComputeBackendChoice::Cpu,
        ..SetupSimulationInput::new(&doc, &resolved, &[])
    })
    .unwrap();

    assert!(
        !sim.has_critical_errors(),
        "Parallel3D over periodic helical sweep must produce 0 rapid collisions and 0 B-Rep gouges, got: {:#?}",
        sim.diagnostics
    );
}
