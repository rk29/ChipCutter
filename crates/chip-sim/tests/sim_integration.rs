#![allow(clippy::unwrap_used, clippy::expect_used)]

use chip_cad::{CadKernel, MonstertruckKernel, sample_cam_torture_block_step};
use chip_cam::{ClEvent, MotionRole, Setup, ToolpathBuilder, WorkOffsetSlot};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec3, DisplayUnit, approx_eq};
use chip_tools::{CutterGeometry, default_starter_tool_library};

use chip_sim::*;

#[test]
fn radial_cutter_profiles_and_3d_revolved_meshes_are_exact_for_all_six_types() {
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

    for t in &lib.tools {
        let mesh = build_cutter_assembly_mesh(t, 24, false);
        assert!(!mesh.vertices.is_empty());
        assert_eq!(mesh.indices.len() % 3, 0);
    }
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
    use chip_cad::{MeshImportOptions, load_mesh_cad_document, sample_sculpted_dome_stl_bytes};
    use chip_cam::{
        BoxAnchor27, MotionRole, Setup, ToolpathBuilder, WcsOriginMode, WorkOffsetSlot,
    };
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

    // 3. Verify 3D revolved cutter meshes for both single-point and multi-form
    for (tool, prof, expected_crests) in [(sp_tool, sp, 1usize), (&mf_tool, mf, 4usize)] {
        for crash_hl in [false, true] {
            let mesh = build_cutter_assembly_mesh(tool, 32, crash_hl);
            assert!(!mesh.vertices.is_empty());
            assert_eq!(mesh.indices.len() % 3, 0);
            for &idx in &mesh.indices {
                assert!((idx as usize) < mesh.vertices.len());
            }

            let mut max_r_flute = 0.0_f64;
            let mut max_r_neck = 0.0_f64;
            let mut max_r_shank = 0.0_f64;
            let mut crest_z_levels: Vec<i32> = Vec::new();

            for v in &mesh.vertices {
                let n_len = (f64::from(v.normal[0]).powi(2)
                    + f64::from(v.normal[1]).powi(2)
                    + f64::from(v.normal[2]).powi(2))
                .sqrt();
                assert!(
                    (n_len - 1.0).abs() < 1e-3,
                    "vertex normal must be unit length, got {n_len}"
                );
                let r = f64::from(v.pos_mm[0]).hypot(f64::from(v.pos_mm[1]));
                let z = f64::from(v.pos_mm[2]);
                if (0.0..=prof.flute_length_mm + 1e-4).contains(&z) {
                    max_r_flute = max_r_flute.max(r);
                    if (r - prof.cutting_radius_mm).abs() < 1e-3 {
                        let z_bucket = (z * 100.0).round() as i32;
                        if !crest_z_levels.contains(&z_bucket) {
                            crest_z_levels.push(z_bucket);
                        }
                    }
                } else if (z - prof.neck_length_mm).abs() < 1e-3 {
                    max_r_neck = max_r_neck.max(r);
                } else if (z - prof.stickout_length_mm).abs() < 1e-3
                    && r <= prof.shank_radius_mm + 1e-3
                {
                    max_r_shank = max_r_shank.max(r);
                }
            }

            assert!(
                approx_eq(max_r_flute, prof.cutting_radius_mm, 1e-3),
                "flute band max radius {max_r_flute} != {}",
                prof.cutting_radius_mm
            );
            assert!(
                approx_eq(max_r_neck, prof.neck_radius_mm, 1e-3),
                "relieved neck max radius {max_r_neck} != {}",
                prof.neck_radius_mm
            );
            assert!(
                max_r_neck < max_r_flute - 0.5,
                "relieved neck ({max_r_neck}) must be visibly narrower than tooth crest ({max_r_flute})"
            );
            assert!(
                approx_eq(max_r_shank, prof.shank_radius_mm, 1e-3),
                "shank max radius {max_r_shank} != {}",
                prof.shank_radius_mm
            );
            assert_eq!(
                crest_z_levels.len(),
                expected_crests,
                "expected {expected_crests} distinct V-tooth crest rings, got {crest_z_levels:?}"
            );
        }
    }
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

    let sim_drill_only = run_setup_simulation_with_engine(
        &SetupSimulationInput {
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
        },
        SimEngineKind::TriDexel,
    )
    .unwrap();

    let sim = run_setup_simulation_with_engine(
        &SetupSimulationInput {
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
        },
        SimEngineKind::TriDexel,
    )
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
    let sim = run_setup_simulation_with_engine(
        &SetupSimulationInput {
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
        },
        SimEngineKind::TriDexel,
    )
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
        let sim_no_pilot = run_setup_simulation_with_engine(
            &SetupSimulationInput {
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
            },
            engine,
        )
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

        let sim_too_deep = run_setup_simulation_with_engine(
            &SetupSimulationInput {
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
            },
            engine,
        )
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

    use chip_cad::{SurfaceKind, sample_machined_plate_step};
    use chip_cam::{
        GeometrySelectionItem, HeightReference, HeightSpec, Operation, OperationGenRequest,
        OperationKind, StrategyParams, ThreadCutDirection, ThreadFormKind, ThreadHandedness,
        ThreadMillParams, generate_operation_toolpath_with_rest,
    };
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
        cancel: &cancel,
    })
    .unwrap();

    let sim_ops = [
        SimOperationInput::from_operation(&op_drill, &drill_8mm, &tp_drill),
        SimOperationInput::from_operation(&op_tm, tm_475, &tp_tm),
    ];

    for engine in [SimEngineKind::Heightmap, SimEngineKind::TriDexel] {
        let sim = run_setup_simulation_with_engine(
            &SetupSimulationInput {
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
            },
            engine,
        )
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
            let sim_bad = run_setup_simulation_with_choice(
                &SetupSimulationInput {
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
                },
                engine,
                choice,
            )
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
            let sim_ok = run_setup_simulation_with_choice(
                &SetupSimulationInput {
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
                },
                engine,
                choice,
            )
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
    })
    .unwrap();
    assert!(sim_cut.has_critical_errors());
    assert!(sim_cut.diagnostics.iter().any(|d| {
        d.kind == SimIssueKind::FixtureCollision && d.fixture_id.as_deref() == Some("clamp_in_cut")
    }));

    // -------------------------------------------------------------------------
    // D. 4th-Axis Cylindrical CPU vs GPU FixtureCollision parity!
    // -------------------------------------------------------------------------
    let shaft_stl = chip_cad::sample_rotary_shaft_stl_bytes();
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

    let cyl_cpu = run_setup_simulation_with_choice(
        &SetupSimulationInput {
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
        },
        SimEngineKind::Heightmap,
        SimComputeBackendChoice::Cpu,
    )
    .unwrap();
    let cyl_gpu = run_setup_simulation_with_choice(
        &SetupSimulationInput {
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
        },
        SimEngineKind::Heightmap,
        SimComputeBackendChoice::Auto,
    )
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
