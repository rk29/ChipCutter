#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::sync::atomic::AtomicBool;

use chip_cad::{
    CadKernel, MonstertruckKernel, SurfaceKind, find_same_diameter_holes,
    sample_cam_torture_block_step,
};
use chip_cam::{
    BoxAnchor27, CamError, ChamferSourceMode, ChamferTraceMode, ClEvent, ContourCompensation,
    GeometrySelectionItem, HeightReference, HeightSpec, HoleMachiningMode, MotionRole, Operation,
    OperationKind, PocketStrategyKind, SegmentMotion3D, Setup, StockConfig, StrategyParams,
    TabPlacementMode, TabProfileKind, WcsOriginMode, WorkOffsetSlot, generate_operation_toolpath,
};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec2, DisplayUnit, ParamLength, approx_eq};
use chip_tools::{CutterGeometry, Tool, default_starter_tool_library};

fn point_to_seg2d_dist(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let ab = b - a;
    let l2 = ab.length_squared();
    if l2 <= 1e-18 {
        return p.distance(a);
    }
    let t = ((p - a).dot(ab) / l2).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

/// Asserts that no `RapidMove` plunges below `min_safe_rapid_z` (except vertical chip-break/peck
/// retracts/resumes strictly coaxial with a hole center).
fn assert_no_plunge_rapids(
    tp: &chip_cam::GeneratedToolpath,
    min_safe_rapid_z: f64,
    allowed_hole_shafts: &[(DVec2, f64)],
) {
    for seg in &tp.segments_wcs {
        if matches!(seg.motion, SegmentMotion3D::Rapid) && seg.end_wcs.z < min_safe_rapid_z - 1e-3 {
            let end_xy = DVec2::new(seg.end_wcs.x, seg.end_wcs.y);
            let start_xy = DVec2::new(seg.start_wcs.x, seg.start_wcs.y);
            let in_drilled_shaft = start_xy.distance(end_xy) <= 1e-4
                && allowed_hole_shafts
                    .iter()
                    .any(|&(c, min_z)| end_xy.distance(c) <= 1e-3 && seg.end_wcs.z >= min_z - 1e-3);
            assert!(
                in_drilled_shaft,
                "UNSAFE G0 RAPID below feed/retract plane ({min_safe_rapid_z:.3} mm): {:?} -> {:?}",
                seg.start_wcs, seg.end_wcs
            );
        }
    }
}

#[test]
fn verifies_all_five_v1_strategies_against_torture_block_invariants() {
    let step = sample_cam_torture_block_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes(
            "torture_block.step",
            step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();

    // Anchor WCS at Model Bottom-Front-Left (0, 0, 0) so WCS coordinates match exact CAD coordinates:
    // Model Z ∈ [0..22], Stock Top Z = 23.0 mm (1 mm top facing stock), Stock XY = [-1..101] x [-1..81].
    let mut setup = Setup::new("setup_1", "Torture Block Op 1", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
    };
    setup.stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(1.0),
        side_neg_x: ParamLength::from_mm(1.0),
        side_pos_y: ParamLength::from_mm(1.0),
        side_neg_y: ParamLength::from_mm(1.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        round_increment_mm: None,
    };
    let resolved_setup = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let lib = default_starter_tool_library();
    let flat_6mm: &Tool = lib
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. } if approx_eq(diameter_mm, 6.0, 1e-3)
            )
        })
        .unwrap();
    let face_mill_38mm: &Tool = lib
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FaceMill { .. }))
        .unwrap();
    let chamfer_90deg: &Tool = lib
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
        .unwrap();
    let mut drill_6mm = lib
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::TwistDrill { .. }))
        .unwrap()
        .clone();
    if let CutterGeometry::TwistDrill { diameter_mm, .. } = &mut drill_6mm.geometry {
        *diameter_mm = 6.0;
    }

    let cancel = AtomicBool::new(false);

    // =========================================================================
    // 1. FACING: 38mm Face Mill surfacing Stock Top (Z=23) down to Model Top (Z=22)
    // =========================================================================
    let mut op_face = Operation::new("op_face", "Face Stock Top", OperationKind::Facing);
    op_face.tool_id = Some(face_mill_38mm.id.clone());
    let tp_face = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_face,
        face_mill_38mm,
        DisplayUnit::Millimeter,
        true,
        &cancel,
    )
    .unwrap();

    assert_no_plunge_rapids(&tp_face, 24.0, &[]);
    assert!(
        tp_face.stats.arc_move_count >= 2,
        "Facing Zig-Zag must emit smooth G2/G3 U-turn ArcMoves"
    );
    assert!(approx_eq(tp_face.stats.bounds_wcs.min.z, 22.0, 1e-3));

    // =========================================================================
    // 2A. DRILLING: 4x Ø6.0 mm through-holes (Z = 14 -> 0 mm) with 118° tip breakthrough
    // =========================================================================
    let r3_seed = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if approx_eq(radius_mm, 3.0, 1e-3)
            )
        })
        .unwrap()
        .id;
    let mut op_drill = Operation::new("op_drill", "4x M6 Clearance", OperationKind::DrillBore);
    op_drill.tool_id = Some(drill_6mm.id.clone());
    // Intentionally scramble selection order to verify Nearest-Neighbor + 2-Opt sorting!
    let mut hole_fids = find_same_diameter_holes(&doc, r3_seed, 1e-3, 1e-3);
    hole_fids.swap(0, 4);
    op_drill.selections = hole_fids
        .into_iter()
        .map(|fid| GeometrySelectionItem::CylindricalHole {
            binding: doc.bind_face(fid).unwrap(),
        })
        .collect();

    let tp_drill = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_drill,
        &drill_6mm,
        DisplayUnit::Millimeter,
        false,
        &cancel,
    )
    .unwrap();
    assert_eq!(tp_drill.stats.drill_cycle_count, 4);
    let drill_centers: Vec<(DVec2, f64)> = tp_drill
        .events
        .iter()
        .filter_map(|ev| match ev {
            ClEvent::DrillCycle { bottom_wcs, .. } => {
                Some((DVec2::new(bottom_wcs.x, bottom_wcs.y), bottom_wcs.z))
            }
            _ => None,
        })
        .collect();
    assert_no_plunge_rapids(&tp_drill, 15.0, &drill_centers);
    // Verify 118° tip breakthrough: 3.0 / tan(59°) + 0.5 = 1.8026 + 0.5 = 2.3026 mm below Z=0!
    let expected_bt_z = -(3.0 / 59.0_f64.to_radians().tan() + 0.5);
    assert!(approx_eq(drill_centers[0].1, expected_bt_z, 2e-2));
    // Verify monotonic Y path ordering along X = 88 (`14 -> 32 -> 48 -> 66` or reverse)
    let ys: Vec<f64> = drill_centers.iter().map(|(c, _)| c.y).collect();
    let monotonic_up = ys.windows(2).all(|w| w[1] > w[0]);
    let monotonic_dn = ys.windows(2).all(|w| w[1] < w[0]);
    assert!(
        monotonic_up || monotonic_dn,
        "hole TSP sorter failed to eliminate zig-zag rapids: {ys:?}"
    );

    // =========================================================================
    // 2B. HELICAL BORE: Wide Ø18.0 mm blind counterbore (Z = 20 -> 10 mm) with Ø6 mm flat endmill
    // =========================================================================
    let r9_bore = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if approx_eq(radius_mm, 9.0, 1e-3)
            )
        })
        .unwrap()
        .id;
    let mut op_bore = Operation::new("op_bore", "Ø18mm Counterbore", OperationKind::DrillBore);
    op_bore.tool_id = Some(flat_6mm.id.clone());
    op_bore.selections = vec![GeometrySelectionItem::CylindricalHole {
        binding: doc.bind_face(r9_bore).unwrap(),
    }];
    if let StrategyParams::DrillBore(p) = op_bore.strategy_mut() {
        p.mode = HoleMachiningMode::HelicalBore;
        p.bore_spring_pass = true;
    }

    let tp_bore = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_bore,
        flat_6mm,
        DisplayUnit::Millimeter,
        false,
        &cancel,
    )
    .unwrap();
    assert_no_plunge_rapids(&tp_bore, 21.0, &[]);
    assert!(tp_bore.stats.arc_move_count >= 12);

    // Verify ZERO bore wall gouge (`dist(tool_xy, (60, 22)) <= 6.005 mm`) AND 100% floor clearing (zero center pin!)
    let bore_center = DVec2::new(60.0, 22.0);
    let mut bore_bottom_samples: Vec<DVec2> = Vec::new();
    for seg in &tp_bore.segments_wcs {
        if !matches!(seg.motion, SegmentMotion3D::Rapid) {
            for k in 0..=8 {
                let pt = seg.sample_point(f64::from(k) / 8.0);
                if pt.z < 20.0 {
                    let r_xy = DVec2::new(pt.x, pt.y).distance(bore_center);
                    assert!(
                        r_xy <= 6.01,
                        "Helical bore gouged Ø18mm wall: tool center radius {r_xy:.4} > 6.0 mm"
                    );
                }
                if (pt.z - 10.0).abs() <= 1e-3 {
                    bore_bottom_samples.push(DVec2::new(pt.x, pt.y));
                }
            }
        }
    }
    // Check radial points from r = 0.0 (exact center!) out to r = 8.8 mm at Z = 10.0 mm
    for r_probe in [0.0, 1.5, 3.0, 4.5, 6.0, 7.5, 8.8] {
        let probe = bore_center + DVec2::new(r_probe, 0.0);
        let min_d = bore_bottom_samples
            .iter()
            .map(|&s| s.distance(probe))
            .fold(f64::INFINITY, f64::min);
        assert!(
            min_d <= 3.05,
            "Uncut pin/ring left in Ø18mm bore at radius {r_probe} mm (nearest tool center {min_d:.3} mm > 3.0 mm)"
        );
    }

    // =========================================================================
    // 3A. 2D POCKET (CLOSED + RAISED ISLAND): [12..46] x [16..64], Z=12..20, Ø10mm island at (29, 40)
    // =========================================================================
    let island_pkt_floor = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && approx_eq(origin_mm.z, 12.0, 1e-2)
            )
        })
        .unwrap()
        .id;

    for strat_kind in [
        PocketStrategyKind::ContourParallel,
        PocketStrategyKind::ZigZag {
            pass_angle_deg: 0.0,
        },
    ] {
        let mut op_pkt = Operation::new("op_pkt", "Island Cavity", OperationKind::Pocket2D);
        op_pkt.tool_id = Some(flat_6mm.id.clone());
        op_pkt.selections = vec![GeometrySelectionItem::PlanarFace {
            binding: doc.bind_face(island_pkt_floor).unwrap(),
        }];
        if let StrategyParams::Pocket2D(p) = op_pkt.strategy_mut() {
            p.strategy = strat_kind;
        }

        let tp_pkt = generate_operation_toolpath(
            &doc,
            &resolved_setup,
            &op_pkt,
            flat_6mm,
            DisplayUnit::Millimeter,
            false,
            &cancel,
        )
        .unwrap();
        assert_no_plunge_rapids(&tp_pkt, 21.0, &[]);
        assert!(
            tp_pkt.stats.arc_move_count >= 4,
            "Pocket with circular island must preserve true ArcMoves"
        );

        let island_c = DVec2::new(29.0, 40.0);
        let mut floor_segs: Vec<(DVec2, DVec2)> = Vec::new();
        for seg in &tp_pkt.segments_wcs {
            if !matches!(seg.motion, SegmentMotion3D::Rapid) {
                for k in 0..8 {
                    let p0 = seg.sample_point(f64::from(k) / 8.0);
                    let p1 = seg.sample_point(f64::from(k + 1) / 8.0);
                    if p0.z < 19.99 || p1.z < 19.99 {
                        for p in [p0, p1] {
                            let xy = DVec2::new(p.x, p.y);
                            // Outer wall gouge check: [12..46] x [16..64] eroded by R=3 -> [15..43] x [19..61]
                            assert!(
                                xy.x >= 14.98 && xy.x <= 43.02 && xy.y >= 18.98 && xy.y <= 61.02,
                                "Pocket outer wall gouge by {strat_kind:?} at {xy:?}"
                            );
                            // Raised Ø10mm (R=5) island boss gouge check: center dist >= 5 + 3 = 8.0 mm!
                            let d_isl = xy.distance(island_c);
                            assert!(
                                d_isl >= 7.98,
                                "Pocket gouged raised Ø10mm island boss ({strat_kind:?}): tool dist {d_isl:.4} < 8.0 mm"
                            );
                        }
                    }
                    if (p0.z - 12.0).abs() <= 1e-3 && (p1.z - 12.0).abs() <= 1e-3 {
                        floor_segs.push((DVec2::new(p0.x, p0.y), DVec2::new(p1.x, p1.y)));
                    }
                }
            }
        }

        // Floor completeness grid check across [15.5..42.5] x [19.5..60.5] outside R=8.5 around (29, 40)
        for ix in 0..=10 {
            let x = 15.5 + 27.0 * (f64::from(ix) / 10.0);
            for iy in 0..=12 {
                let y = 19.5 + 41.0 * (f64::from(iy) / 12.0);
                let pt = DVec2::new(x, y);
                if pt.distance(island_c) < 8.5 {
                    continue;
                }
                let min_d = floor_segs
                    .iter()
                    .map(|&(a, b)| point_to_seg2d_dist(pt, a, b))
                    .fold(f64::INFINITY, f64::min);
                assert!(
                    min_d <= 3.08,
                    "Uncut pocket floor sliver ({strat_kind:?}) at {pt:?}: nearest tool pass {min_d:.3} mm > 3.0 mm"
                );
            }
        }
    }

    // =========================================================================
    // 3B. 2D POCKET (OPEN-EDGE STEP): [76..100] x [0..80] at Z = 14.0 mm
    // =========================================================================
    let open_step_floor = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && approx_eq(origin_mm.z, 14.0, 1e-2)
            )
        })
        .unwrap()
        .id;
    let mut op_open_pkt = Operation::new("op_open_step", "Open Shoulder", OperationKind::Pocket2D);
    op_open_pkt.tool_id = Some(flat_6mm.id.clone());
    op_open_pkt.selections = vec![GeometrySelectionItem::PlanarFace {
        binding: doc.bind_face(open_step_floor).unwrap(),
    }];
    let tp_open = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_open_pkt,
        flat_6mm,
        DisplayUnit::Millimeter,
        false,
        &cancel,
    )
    .unwrap();
    assert_no_plunge_rapids(&tp_open, 21.0, &[]);

    // Verify first plunge enters in open air outside [76..100] x [0..80] AND X=76 wall is never gouged (`X >= 78.98`)!
    let first_plunge = tp_open
        .segments_wcs
        .iter()
        .find(|s| {
            matches!(
                s.motion,
                SegmentMotion3D::Linear {
                    role: MotionRole::Plunge,
                    ..
                }
            )
        })
        .unwrap();
    let fp_xy = DVec2::new(first_plunge.end_wcs.x, first_plunge.end_wcs.y);
    assert!(
        fp_xy.x > 100.1 || fp_xy.y < -0.1 || fp_xy.y > 80.1,
        "Open-edge step pocket should plunge in air outside stock boundary, got {fp_xy:?}"
    );
    for seg in &tp_open.segments_wcs {
        if !matches!(seg.motion, SegmentMotion3D::Rapid) {
            for k in 0..=4 {
                let p = seg.sample_point(f64::from(k) / 4.0);
                if p.z < 19.99 {
                    assert!(
                        p.x >= 78.95,
                        "Open-edge step pocket gouged rising X=76 wall: tool X = {:.4} < 79.0",
                        p.x
                    );
                }
            }
        }
    }

    // =========================================================================
    // 4. 2D CONTOUR / PROFILE + TRIANGULAR HOLDING TABS around Z = 0 outer profile
    // =========================================================================
    let bottom_face = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z < -0.99 && approx_eq(origin_mm.z, 0.0, 1e-2)
            )
        })
        .unwrap();
    let outer_loop_id = bottom_face.outer_loop.unwrap();
    let mut op_contour = Operation::new("op_cnt", "Outer Profile + Tabs", OperationKind::Contour2D);
    op_contour.tool_id = Some(flat_6mm.id.clone());
    // Note: bottom face normal is -Z, so its outer_loop winds CW when viewed from +Z;
    // with `LeftClimb`, walking CW around an outer profile offsets to the LEFT = OUTSIDE the part!
    op_contour.selections = vec![GeometrySelectionItem::ClosedLoop {
        binding: doc.bind_loop(outer_loop_id).unwrap(),
        reverse: false,
        side_flip: false,
    }];
    op_contour.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
    op_contour.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
    if let StrategyParams::Contour2D(p) = op_contour.strategy_mut() {
        p.compensation = ContourCompensation::LeftClimb;
        p.finish_passes = 1;
        p.tabs.enabled = true;
        p.tabs.placement = TabPlacementMode::Automatic {
            count: 4,
            by_spacing: false,
            target_spacing: ParamLength::from_mm(60.0),
        };
        p.tabs.profile = TabProfileKind::Triangular {
            ramp_angle_deg: 25.0,
        };
        p.tabs.height = ParamLength::from_mm(2.0);
    }

    let tp_cnt = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_contour,
        flat_6mm,
        DisplayUnit::Millimeter,
        false,
        &cancel,
    )
    .unwrap();
    assert_eq!(tp_cnt.tab_markers_wcs.len(), 4);
    assert!(tp_cnt.stats.arc_move_count >= 6);
    assert_no_plunge_rapids(&tp_cnt, 23.0, &[]);
    let has_tab_ramp = tp_cnt.segments_wcs.iter().any(|s| {
        matches!(
            s.motion,
            SegmentMotion3D::Linear {
                role: MotionRole::TabRamp,
                ..
            } | SegmentMotion3D::Arc {
                role: MotionRole::TabRamp,
                ..
            }
        ) && approx_eq(s.end_wcs.z.max(s.start_wcs.z), 2.0, 2e-2)
    });
    assert!(
        has_tab_ramp,
        "expected Triangular Holding Tabs to ramp smoothly up to Z = 2.0 mm"
    );

    // =========================================================================
    // 5. EDGE CHAMFERING: Sharp Step Edge + Tip Floor Guard + Modeled 45° Bevel Face
    // =========================================================================
    let step_top_edge = doc
        .edges
        .iter()
        .find(|e| {
            approx_eq(e.aabb_mm.min.z, 20.0, 1e-2)
                && approx_eq(e.aabb_mm.max.z, 20.0, 1e-2)
                && approx_eq(e.aabb_mm.min.x, 76.0, 1e-2)
                && approx_eq(e.aabb_mm.max.x, 76.0, 1e-2)
        })
        .unwrap()
        .id;
    let mut op_chamf = Operation::new("op_chamf", "Step Edge Chamfer", OperationKind::ChamferTrace);
    op_chamf.tool_id = Some(chamfer_90deg.id.clone());
    op_chamf.selections = vec![GeometrySelectionItem::OpenChain {
        body_id: doc.bodies[0].id,
        start_edge: doc.bind_edge(step_top_edge).unwrap(),
        end_edge: doc.bind_edge(step_top_edge).unwrap(),
        resolved_edges: vec![chip_cad::OrientedEdgeUse {
            edge_id: step_top_edge,
            forward: true,
        }],
        reverse: false,
        side_flip: true, // step drops off to +X (right of +Y edge)
    }];
    if let StrategyParams::ChamferTrace(p) = op_chamf.strategy_mut() {
        p.mode = ChamferTraceMode::EdgeChamfer;
        p.chamfer_source = ChamferSourceMode::SharpEdge {
            width: ParamLength::from_mm(0.5),
        };
        p.tip_offset = ParamLength::from_mm(0.8);
    }

    let tp_chamf = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_chamf,
        chamfer_90deg,
        DisplayUnit::Millimeter,
        false,
        &cancel,
    )
    .unwrap();
    // For 90° chamfer mill (θ = 45°), w = 0.5 mm, d_tip = 0.8 mm -> axial_drop = 0.5 + 0.8 = 1.3 mm -> tip Z = 18.7 mm!
    assert!(approx_eq(tp_chamf.stats.bounds_wcs.min.z, 18.7, 2e-2));

    // Excessive tip_offset (5.8 mm -> axial_drop = 6.3 mm -> tip Z = 13.7 mm < 14.0 mm step floor!) MUST error!
    let mut op_chamf_bad = op_chamf.clone();
    // Widen chamfer tool slightly in test so flute span check doesn't fire before floor check
    let mut big_chamfer = chamfer_90deg.clone();
    if let CutterGeometry::ChamferMill { diameter_mm, .. } = &mut big_chamfer.geometry {
        *diameter_mm = 16.0;
    }
    if let StrategyParams::ChamferTrace(p) = op_chamf_bad.strategy_mut() {
        p.tip_offset = ParamLength::from_mm(5.8);
    }
    let err = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_chamf_bad,
        &big_chamfer,
        DisplayUnit::Millimeter,
        false,
        &cancel,
    )
    .unwrap_err();
    assert!(
        matches!(err, CamError::ChamferTipCollision { .. }),
        "expected ChamferTipCollision against Z=14 step floor, got {err:?}"
    );

    // Modeled 45° x 2.0 mm chamfer face on the Z=20..22 boss pad (whose base sits flush on the Z=20.0 plateau!):
    // 1) Non-zero tip_offset (0.6 mm -> tip Z = 19.4 mm) would gouge 0.6 mm into the Z=20.0 plateau floor -> rejected!
    let modeled_bevel_face = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { normal, .. }
                if approx_eq(normal.z, std::f64::consts::FRAC_1_SQRT_2, 1e-2)
            )
        })
        .unwrap()
        .id;
    let mut op_mod_chamf = Operation::new(
        "op_mod_chamf",
        "Modeled Boss Chamfer",
        OperationKind::ChamferTrace,
    );
    op_mod_chamf.tool_id = Some(big_chamfer.id.clone());
    op_mod_chamf.selections = vec![GeometrySelectionItem::PlanarFace {
        binding: doc.bind_face(modeled_bevel_face).unwrap(),
    }];
    if let StrategyParams::ChamferTrace(p) = op_mod_chamf.strategy_mut() {
        p.chamfer_source = ChamferSourceMode::ModeledFace;
        p.tip_offset = ParamLength::from_mm(0.6);
    }
    let gouge_err = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_mod_chamf,
        &big_chamfer,
        DisplayUnit::Millimeter,
        false,
        &cancel,
    )
    .unwrap_err();
    assert!(matches!(gouge_err, CamError::ChamferTipCollision { .. }));

    // 2) With tip_offset = 0.0 mm, the cutter cone traces the 2.0 mm modeled bevel flush at Z = 20.0 mm with zero floor gouge!
    if let StrategyParams::ChamferTrace(p) = op_mod_chamf.strategy_mut() {
        p.tip_offset = ParamLength::from_mm(0.0);
    }
    let tp_mod_chamf = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_mod_chamf,
        &big_chamfer,
        DisplayUnit::Millimeter,
        false,
        &cancel,
    )
    .unwrap();
    assert!(approx_eq(tp_mod_chamf.stats.bounds_wcs.min.z, 20.0, 2e-2));
}
