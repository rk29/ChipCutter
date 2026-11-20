//! 2.5D Z-Dexel heightmap stock material removal, trapezoidal cycle-time timeline,
//! 3D cutter/stock mesh builders, and 5-rule crash & gouge verifier (`chip-sim`).

pub mod cutter_profile;
pub mod cylindrical;
pub mod heightmap;
pub mod timeline;
pub mod verifier;

pub use cutter_profile::{
    RadialCutterProfile, SimColoredVertex, SimToolMesh, ToolMeshZone, build_cutter_assembly_mesh,
};
pub use cylindrical::{
    CylindricalCheckpoint, CylindricalStock, run_cylindrical_setup_simulation,
    tessellate_cylindrical_stock_mesh,
};
pub use heightmap::{
    HeightfieldStock, OPERATION_STOCK_PALETTE, SetupSimulationInput, SetupSimulationResult,
    SimError, SimResolution, SimStockMesh, StockCheckpoint, UNCUT_OP_IDX, run_setup_simulation,
    tessellate_stock_mesh,
};
pub use timeline::{
    MachineKinematicsConfig, MotionSpan, OperationTimelineSummary, ScrubSnapshot,
    SimOperationInput, SimulationTimeline, format_cycle_time_hms,
};
pub use verifier::{SimDiagnostic, SimIssueKind, SimSeverity, SimVerificationConfig};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::{CadKernel, MonstertruckKernel, sample_cam_torture_block_step};
    use chip_cam::{ClEvent, MotionRole, Setup, ToolpathBuilder, WorkOffsetSlot};
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec3, DisplayUnit, approx_eq};
    use chip_tools::{CutterGeometry, default_starter_tool_library};

    use super::*;

    #[test]
    fn radial_cutter_profiles_and_3d_revolved_meshes_are_exact_for_all_six_types() {
        let lib = default_starter_tool_library();
        assert_eq!(lib.tools.len(), 8);

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
            operations: &ops,
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
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
        let mid_stock = res.reconstruct_stock_at_time(&ops, res.timeline.total_time_sec * 0.5);
        let mesh_neutral = tessellate_stock_mesh(&mid_stock, false);
        let mesh_colored = tessellate_stock_mesh(&res.final_stock, true);
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
            load_mesh_cad_document("sculpted_dome.stl", &stl, &MeshImportOptions::default())
                .unwrap();

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
}
