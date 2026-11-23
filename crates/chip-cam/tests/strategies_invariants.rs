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
        sheet_thickness_z: ParamLength::from_mm(12.0),
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
        PocketStrategyKind::Adaptive(chip_cam::AdaptiveClearingParams::default()),
    ] {
        let mut op_pkt = Operation::new("op_pkt", "Island Cavity", OperationKind::Pocket2D);
        op_pkt.tool_id = Some(flat_6mm.id.clone());
        op_pkt.selections = vec![GeometrySelectionItem::PlanarFace {
            binding: doc.bind_face(island_pkt_floor).unwrap(),
        }];
        if let StrategyParams::Pocket2D(p) = op_pkt.strategy_mut() {
            p.strategy = strat_kind.clone();
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

    // =========================================================================
    // 6. PHYSICAL Z-DEXEL HEIGHTMAP ORACLE (`chip-sim`, AGENTS.md §4)
    // =========================================================================
    // Also machine the Z = 20.0 mm main plateau around the raised Z = 22.0 mm chamfered boss pad
    // so subsequent operations starting from Z = 20.0 mm enter cleanly!
    let plateau_z20_face = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && approx_eq(origin_mm.z, 20.0, 1e-2) && f.area_mm2 > 1500.0
            )
        })
        .unwrap()
        .id;
    let mut op_plateau = Operation::new(
        "op_plateau",
        "Plateau Deck Around Boss",
        OperationKind::Pocket2D,
    );
    op_plateau.tool_id = Some(flat_6mm.id.clone());
    op_plateau.selections = vec![GeometrySelectionItem::PlanarFace {
        binding: doc.bind_face(plateau_z20_face).unwrap(),
    }];
    let tp_plateau = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_plateau,
        flat_6mm,
        DisplayUnit::Millimeter,
        false,
        &cancel,
    )
    .unwrap();

    // Generate ContourParallel Island Pocket for physical simulation
    let mut op_pkt_sim = Operation::new("op_pkt_sim", "Island Cavity", OperationKind::Pocket2D);
    op_pkt_sim.tool_id = Some(flat_6mm.id.clone());
    op_pkt_sim.selections = vec![GeometrySelectionItem::PlanarFace {
        binding: doc.bind_face(island_pkt_floor).unwrap(),
    }];
    let tp_pkt_sim = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_pkt_sim,
        flat_6mm,
        DisplayUnit::Millimeter,
        false,
        &cancel,
    )
    .unwrap();

    let sim_ops = [
        chip_sim::SimOperationInput::from_operation(&op_face, face_mill_38mm, &tp_face),
        chip_sim::SimOperationInput::from_operation(&op_plateau, flat_6mm, &tp_plateau),
        chip_sim::SimOperationInput::from_operation(&op_open_pkt, flat_6mm, &tp_open),
        chip_sim::SimOperationInput::from_operation(&op_pkt_sim, flat_6mm, &tp_pkt_sim),
        chip_sim::SimOperationInput::from_operation(&op_bore, flat_6mm, &tp_bore),
        chip_sim::SimOperationInput::from_operation(&op_drill, &drill_6mm, &tp_drill),
        chip_sim::SimOperationInput::from_operation(&op_contour, flat_6mm, &tp_cnt),
        chip_sim::SimOperationInput::from_operation(&op_chamf, chamfer_90deg, &tp_chamf),
        chip_sim::SimOperationInput::from_operation(&op_mod_chamf, &big_chamfer, &tp_mod_chamf),
    ];

    let sim_res = chip_sim::run_setup_simulation(&chip_sim::SetupSimulationInput {
        doc: &doc,
        setup: &resolved_setup,
        target_bodies: &setup.target_bodies,
        fixtures: &[],
        operations: &sim_ops,
        resolution: chip_sim::SimResolution::Fine,
        kinematics: chip_sim::MachineKinematicsConfig::default(),
        verify_config: chip_sim::SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();

    assert_eq!(
        sim_res.critical_error_count(),
        0,
        "Expected ZERO critical rapid crashes, target gouges, or holder collisions across all 9 torture block operations, got: {:#?}",
        sim_res.diagnostics
    );

    // Verify exact heightmap Z values on machined features!
    // 1. Island Pocket Floor at (20, 25) -> Z == 12.00 ± 0.02 mm
    let z_pkt_floor = sim_res.final_stock.sample_stock_z(20.0, 25.0).unwrap();
    assert!(
        approx_eq(z_pkt_floor, 12.0, 0.02),
        "Island pocket floor should be at Z = 12.00 mm, got {z_pkt_floor:.4}"
    );
    // 2. Central Ø10mm Raised Island Boss at (29, 40) -> untouched at Z == 20.00 ± 0.02 mm!
    let z_island_top = sim_res.final_stock.sample_stock_z(29.0, 40.0).unwrap();
    assert!(
        approx_eq(z_island_top, 20.0, 0.02),
        "Central Ø10mm island boss top should remain at Z = 20.00 mm, got {z_island_top:.4}"
    );
    // 3. Ø18mm Blind Counterbore Floor at (60, 22) -> Z == 10.00 ± 0.02 mm!
    let z_bore_floor = sim_res.final_stock.sample_stock_z(60.0, 22.0).unwrap();
    assert!(
        approx_eq(z_bore_floor, 10.0, 0.02),
        "Ø18mm counterbore floor should be at Z = 10.00 mm, got {z_bore_floor:.4}"
    );
    // 4. Drilled Ø6mm through-hole center at (88, 14) -> completely Breakthrough (`None`)!
    assert!(
        sim_res.final_stock.sample_stock_z(88.0, 14.0).is_none(),
        "Through-hole center (88, 14) must be completely drilled through Z=0!"
    );
    // 5. Raised chamfered boss top pad at (60, 58) -> Z == 22.00 ± 0.02 mm!
    let z_boss_pad = sim_res.final_stock.sample_stock_z(60.0, 58.0).unwrap();
    assert!(
        approx_eq(z_boss_pad, 22.0, 0.02),
        "Chamfered boss pad top should be at Z = 22.00 mm, got {z_boss_pad:.4}"
    );
}

#[test]
fn verifies_adaptive_clearing_and_rest_machining_against_sim_oracle() {
    use std::sync::Arc;

    use chip_cam::{
        OperationGenRequest, RestMachiningConfig, RestMachiningMode, UpstreamSweptInput,
        generate_operation_toolpath_with_rest,
    };
    use chip_tools::{
        CoolantMode, CuttingPreset, SpindleDirection, SurfaceSpeedPresetSpec, Tool, ToolId,
    };

    let step = sample_cam_torture_block_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("torture.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();
    let starter = default_starter_tool_library();
    let face_mill_38mm = starter
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FaceMill { .. }))
        .unwrap();
    let flat_6mm = &starter.tools[0];

    let tool_10mm = Tool {
        id: ToolId("t10_rough".to_string()),
        tool_number: 10,
        length_offset_number: 10,
        diameter_offset_number: 10,
        name: "10mm 3FL Rougher".to_string(),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::AirBlast,
        stickout_length_mm: 28.0,
        geometry: CutterGeometry::FlatEndmill {
            diameter_mm: 10.0,
            flute_length_mm: 22.0,
            overall_length_mm: 65.0,
            shank_diameter_mm: 10.0,
            flutes: 3,
        },
        presets: vec![CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
            id: "al_rough",
            name: "Al Rough",
            material: "Al 6061",
            diameter_mm: 10.0,
            flutes: 3,
            surface_speed_m_min: 260.0,
            chipload_mm_tooth: 0.05,
            default_stepdown_mm: 10.0,
            default_stepover_mm: 2.0,
            coolant_override: None,
        })],
    };

    let tool_3mm = Tool {
        id: ToolId("t8_rest_3mm".to_string()),
        tool_number: 8,
        length_offset_number: 8,
        diameter_offset_number: 8,
        name: "3mm 3FL Rest".to_string(),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::AirBlast,
        stickout_length_mm: 18.0,
        geometry: CutterGeometry::FlatEndmill {
            diameter_mm: 3.0,
            flute_length_mm: 12.0,
            overall_length_mm: 45.0,
            shank_diameter_mm: 3.0,
            flutes: 3,
        },
        presets: vec![CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
            id: "al_rest",
            name: "Al Rest",
            material: "Al 6061",
            diameter_mm: 3.0,
            flutes: 3,
            surface_speed_m_min: 200.0,
            chipload_mm_tooth: 0.02,
            default_stepdown_mm: 4.5,
            default_stepover_mm: 0.6,
            coolant_override: None,
        })],
    };

    let mut setup = Setup::new("setup_adp", "Op 1", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
    };
    let resolved_setup = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let cancel = AtomicBool::new(false);

    // 1. Face Stock Top + Clear Z=20 Plateau around Boss Pad
    let mut op_face = Operation::new("op_face", "Face Top", OperationKind::Facing);
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

    let plateau_face = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && approx_eq(origin_mm.z, 20.0, 1e-2) && f.area_mm2 > 1500.0
            )
        })
        .unwrap()
        .id;
    let mut op_plat = Operation::new_adaptive_2d("op_plat", "Plateau Adaptive");
    op_plat.tool_id = Some(flat_6mm.id.clone());
    op_plat.selections = vec![GeometrySelectionItem::PlanarFace {
        binding: doc.bind_face(plateau_face).unwrap(),
    }];
    let tp_plat = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_plat,
        flat_6mm,
        DisplayUnit::Millimeter,
        false,
        &cancel,
    )
    .unwrap();

    // 2. Ø10 mm 2D Adaptive Roughing on Island Cavity `[12..46] x [16..64]` at Z = 12.0 mm
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
    let pkt_binding = doc.bind_face(island_pkt_floor).unwrap();

    let mut op_rough_10 = Operation::new_adaptive_2d("op_rough_10", "Island Ø10mm Adaptive");
    op_rough_10.tool_id = Some(tool_10mm.id.clone());
    op_rough_10.selections = vec![GeometrySelectionItem::PlanarFace {
        binding: pkt_binding.clone(),
    }];
    let tp_rough_10 = Arc::new(
        generate_operation_toolpath(
            &doc,
            &resolved_setup,
            &op_rough_10,
            &tool_10mm,
            DisplayUnit::Millimeter,
            false,
            &cancel,
        )
        .unwrap(),
    );

    // Verify radial chip-thinning boosted the cutting feedrate above nominal (`ae = 0.2 * D` -> `ratio = 0.8` -> `1.25x` boost!)
    let nom_feed_10 = op_rough_10
        .effective_cutting_params(&tool_10mm)
        .cutting_feed_mm_min;
    let has_boosted_cut = tp_rough_10.segments_wcs.iter().any(|s| {
        matches!(
            s.motion,
            SegmentMotion3D::Linear { feed_mm_min, role: chip_cam::MotionRole::Cut }
            | SegmentMotion3D::Arc { feed_mm_min, role: chip_cam::MotionRole::Cut, .. }
            if feed_mm_min > nom_feed_10 * 1.20
        )
    });
    assert!(
        has_boosted_cut,
        "2D Adaptive with auto_chip_thinning must apply ~1.25x feedrate compensation at ae = 0.20 * D"
    );

    // Compare full Ø3 mm Adaptive clear vs `FromPrecedingOperations` Ø3 mm Rest Adaptive clear!
    let mut op_full_3 = Operation::new_adaptive_2d("op_full_3", "Full Ø3mm Adaptive");
    op_full_3.tool_id = Some(tool_3mm.id.clone());
    op_full_3.selections = vec![GeometrySelectionItem::PlanarFace {
        binding: pkt_binding.clone(),
    }];
    let tp_full_3 = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_full_3,
        &tool_3mm,
        DisplayUnit::Millimeter,
        false,
        &cancel,
    )
    .unwrap();

    let mut op_rest_3 = Operation::new_adaptive_2d("op_rest_3", "Rest Ø3mm Adaptive");
    op_rest_3.tool_id = Some(tool_3mm.id.clone());
    op_rest_3.selections = vec![GeometrySelectionItem::PlanarFace {
        binding: pkt_binding.clone(),
    }];
    op_rest_3.passes.rest_machining = RestMachiningConfig {
        mode: RestMachiningMode::FromPrecedingOperations {
            source_op_ids: vec!["op_rough_10".to_string()],
        },
        rest_overlap: ParamLength::from_mm(0.25),
        min_rest_thickness: ParamLength::from_mm(0.06),
    };
    let upstream = [UpstreamSweptInput {
        op_id: "op_rough_10".to_string(),
        tool_diameter_mm: 10.0,
        corner_radius_mm: 0.0,
        toolpath: Arc::clone(&tp_rough_10),
        indexed_angle_deg: None,
        is_rotary_native: false,
    }];
    let tp_rest_3 = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_setup,
        op: &op_rest_3,
        tool: &tool_3mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &upstream,
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    assert!(
        tp_rest_3.stats.cutting_distance_mm < 0.35 * tp_full_3.stats.cutting_distance_mm,
        "Rest Machining cutting distance ({:.1} mm) must be < 35% of full Ø3mm clear ({:.1} mm)",
        tp_rest_3.stats.cutting_distance_mm,
        tp_full_3.stats.cutting_distance_mm,
    );

    // Also verify 2D Contour Rest Machining (`FromPreviousTool`) on the pocket outer loop cuts only the 4 corners!
    let pkt_outer_loop = doc.face(island_pkt_floor).unwrap().outer_loop.unwrap();
    let mut op_cnt_full =
        Operation::new("op_cnt_full", "Full Ø3mm Contour", OperationKind::Contour2D);
    op_cnt_full.tool_id = Some(tool_3mm.id.clone());
    op_cnt_full.selections = vec![GeometrySelectionItem::ClosedLoop {
        binding: doc.bind_loop(pkt_outer_loop).unwrap(),
        reverse: true,
        side_flip: false,
    }];
    if let StrategyParams::Contour2D(cp) = op_cnt_full.strategy_mut() {
        cp.compensation = ContourCompensation::LeftClimb;
    }
    let tp_cnt_full = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_cnt_full,
        &tool_3mm,
        DisplayUnit::Millimeter,
        false,
        &cancel,
    )
    .unwrap();

    let mut op_cnt_rest = op_cnt_full.clone();
    op_cnt_rest.passes.rest_machining = RestMachiningConfig {
        mode: RestMachiningMode::FromPreviousTool {
            prev_tool_diameter: ParamLength::from_mm(10.0),
            prev_stock_to_leave: ParamLength::from_mm(0.0),
        },
        rest_overlap: ParamLength::from_mm(0.30),
        min_rest_thickness: ParamLength::from_mm(0.05),
    };
    let tp_cnt_rest = generate_operation_toolpath(
        &doc,
        &resolved_setup,
        &op_cnt_rest,
        &tool_3mm,
        DisplayUnit::Millimeter,
        false,
        &cancel,
    )
    .unwrap();
    let cut_only_len = |tp: &chip_cam::GeneratedToolpath| -> f64 {
        tp.segments_wcs
            .iter()
            .filter(|s| {
                matches!(
                    s.motion,
                    SegmentMotion3D::Linear {
                        role: chip_cam::MotionRole::Cut | chip_cam::MotionRole::FinishCut,
                        ..
                    } | SegmentMotion3D::Arc {
                        role: chip_cam::MotionRole::Cut | chip_cam::MotionRole::FinishCut,
                        ..
                    }
                )
            })
            .map(|s| s.start_wcs.distance(s.end_wcs))
            .sum()
    };
    let full_cut_len = cut_only_len(&tp_cnt_full);
    let rest_cut_len = cut_only_len(&tp_cnt_rest);
    assert!(
        rest_cut_len < 0.30 * full_cut_len,
        "2D Contour Rest active wall cut distance ({rest_cut_len:.1} mm) must be < 30% of full contour ({full_cut_len:.1} mm)"
    );
    assert!(
        tp_cnt_rest.stats.cutting_distance_mm < 0.50 * tp_cnt_full.stats.cutting_distance_mm,
        "2D Contour Rest total feed distance ({:.1} mm) must be < 50% of full contour ({:.1} mm)",
        tp_cnt_rest.stats.cutting_distance_mm,
        tp_cnt_full.stats.cutting_distance_mm,
    );

    // 3. Sweep before & after Rest through `chip-sim` at Fine resolution!
    let sim_before_rest = chip_sim::run_setup_simulation(&chip_sim::SetupSimulationInput {
        doc: &doc,
        setup: &resolved_setup,
        target_bodies: &setup.target_bodies,
        fixtures: &[],
        operations: &[
            chip_sim::SimOperationInput::from_operation(&op_face, face_mill_38mm, &tp_face),
            chip_sim::SimOperationInput::from_operation(&op_plat, flat_6mm, &tp_plat),
            chip_sim::SimOperationInput::from_operation(&op_rough_10, &tool_10mm, &tp_rough_10),
        ],
        resolution: chip_sim::SimResolution::Fine,
        kinematics: chip_sim::MachineKinematicsConfig::default(),
        verify_config: chip_sim::SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(sim_before_rest.critical_error_count(), 0);
    // Tight corner `(13.6, 17.6)` is inside the R=5 cusp of Ø10mm (`(17-13.6)^2 + (21-17.6)^2 = 23.12 < 25` -> Wait: `sqrt(23.12) = 4.81 < 5.0`? Wait: at `(13.0, 17.0)`, `sqrt(4^2 + 4^2) = 5.657 > 5.0` -> uncut at Z=20!)
    let z_corner_before = sim_before_rest
        .final_stock
        .sample_stock_z(13.0, 17.0)
        .unwrap();
    assert!(
        z_corner_before > 19.5,
        "Tight corner (13.0, 17.0) is unreachable by Ø10mm rougher (`d=5.66 > R=5`), expected Z ~ 20, got {z_corner_before:.3}"
    );

    let sim_after_rest = chip_sim::run_setup_simulation(&chip_sim::SetupSimulationInput {
        doc: &doc,
        setup: &resolved_setup,
        target_bodies: &setup.target_bodies,
        fixtures: &[],
        operations: &[
            chip_sim::SimOperationInput::from_operation(&op_face, face_mill_38mm, &tp_face),
            chip_sim::SimOperationInput::from_operation(&op_plat, flat_6mm, &tp_plat),
            chip_sim::SimOperationInput::from_operation(&op_rough_10, &tool_10mm, &tp_rough_10),
            chip_sim::SimOperationInput::from_operation(&op_rest_3, &tool_3mm, &tp_rest_3),
        ],
        resolution: chip_sim::SimResolution::Fine,
        kinematics: chip_sim::MachineKinematicsConfig::default(),
        verify_config: chip_sim::SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim_after_rest.critical_error_count(),
        0,
        "Expected 0 critical errors after Adaptive + Rest sequence, got: {:#?}",
        sim_after_rest.diagnostics
    );
    // Corner `(13.0, 17.0)` IS reachable by Ø3mm (`R=1.5`, center at `(13.5, 17.5)` has `sqrt(0.5^2+0.5^2) = 0.71 <= 1.5`!), so it MUST now be at Z = 12.00 mm!
    let z_corner_after = sim_after_rest
        .final_stock
        .sample_stock_z(13.0, 17.0)
        .unwrap();
    assert!(
        approx_eq(z_corner_after, 12.0, 0.03),
        "Ø3mm Adaptive Rest must clear tight corner (13.0, 17.0) down to Z = 12.00 mm, got {z_corner_after:.4}"
    );
    // Central Ø10mm raised island boss at (29, 40) must remain untouched at Z = 20.00 mm!
    let z_island_after = sim_after_rest
        .final_stock
        .sample_stock_z(29.0, 40.0)
        .unwrap();
    assert!(approx_eq(z_island_after, 20.0, 0.02));
}

#[test]
fn verifies_vcarve_medial_corner_climbs_two_tool_flat_clearance_and_male_inlay_plug() {
    use std::sync::atomic::AtomicBool;

    use chip_cad::CadDocument;
    use chip_cam::{
        BoxAnchor27, FixedBoxXyAlign, FixedBoxZAlign, GeometrySelectionItem, Operation,
        OperationGenRequest, OperationKind, Setup, StockConfig, StrategyParams, VCarveClearingKind,
        VCarveInlayMode, WcsOriginMode, WorkOffsetSlot, generate_operation_toolpath_with_rest,
    };
    use chip_math::{DisplayUnit, ParamLength, approx_eq};
    use chip_sketch::{
        RectAnchorMode, SketchEntity, SketchEntityEntry, SketchFontSource, SketchLayer,
        SketchTextSpec,
    };
    use chip_tools::{
        CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId, default_starter_tool_library,
    };

    let empty_doc = CadDocument::empty();
    let cancel = AtomicBool::new(false);
    let starter = default_starter_tool_library();
    let flat_6mm = &starter.tools[0];
    let vbit_60 = starter
        .tools
        .iter()
        .find(|t| t.id.0 == "t8_6mm_60deg_vbit")
        .unwrap();

    // Sharp 90° V-bit with Ø24mm cone so a 20×20mm square carves in 1 pure medial pyramid to Z = -10.0 mm!
    let vbit_90_wide = Tool {
        id: ToolId("vbit_90_24mm".to_string()),
        tool_number: 12,
        length_offset_number: 12,
        diameter_offset_number: 12,
        name: "24mm 90° Sign V-Bit".to_string(),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::Disabled,
        stickout_length_mm: 24.0,
        geometry: CutterGeometry::ChamferMill {
            diameter_mm: 24.0,
            tip_diameter_mm: 0.0,
            included_angle_deg: 90.0,
            flute_length_mm: 14.0,
            overall_length_mm: 55.0,
            shank_diameter_mm: 6.35,
            flutes: 2,
        },
        presets: vbit_60.presets.clone(),
    };

    // 1. Pure Medial V-Carve on a 20×20mm square: center reaches Z = -10.0 mm and 4 corner bisectors climb to Z = 0.0 mm!
    let mut sk_sq = SketchLayer::new("sk_sq", "Square Test", "setup_1");
    sk_sq.entities.push(SketchEntityEntry {
        id: "sq_20".to_string(),
        name: "20x20 Square".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(0.0),
            y: ParamLength::from_mm(0.0),
            width: ParamLength::from_mm(20.0),
            height: ParamLength::from_mm(20.0),
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
        },
    });
    sk_sq.rebuild_regions(None);

    let mut setup = Setup::new("setup_1", "V-Carve Test", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(140.0),
        depth_y: ParamLength::from_mm(80.0),
        height_z: ParamLength::from_mm(15.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::OffsetFromTop {
            top_allowance: ParamLength::from_mm(0.0),
        },
    };
    let sketches_sq = [sk_sq];
    let resolved_sq = setup
        .resolve_with_sketches(&empty_doc, &sketches_sq, DisplayUnit::Millimeter)
        .unwrap();

    let mut op_pure = Operation::new("op_pure_vc", "Pure Square V-Carve", OperationKind::VCarve);
    op_pure.tool_id = Some(vbit_90_wide.id.clone());
    op_pure.passes.multiple_depths = false;
    op_pure
        .selections
        .push(GeometrySelectionItem::SketchRegion {
            sketch_id: "sk_sq".to_string(),
            region_id: sketches_sq[0].regions[0].id.clone(),
        });
    if let StrategyParams::VCarve(vp) = op_pure.strategy_mut() {
        vp.flat_depth = None;
    }

    let tp_pure = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &empty_doc,
        sketches: &sketches_sq,
        setup: &resolved_sq,
        op: &op_pure,
        tool: &vbit_90_wide,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    assert!(approx_eq(tp_pure.stats.bounds_wcs.min.z, -10.0, 0.05));
    for &(cx, cy) in &[(10.0, 10.0), (-10.0, 10.0), (10.0, -10.0), (-10.0, -10.0)] {
        let hit_corner = tp_pure.segments_wcs.iter().any(|s| {
            (s.end_wcs.x - cx).hypot(s.end_wcs.y - cy) <= 0.15 && s.end_wcs.z.abs() <= 0.15
        });
        assert!(
            hit_corner,
            "90° V-bit corner climb must reach sharp square corner ({cx}, {cy}, Z=0)"
        );
    }

    // Verify analytical conical 3D sweeper produces exact inverted pyramid (`z(5, 0) = -5.0 mm`, `z(0, 0) = -10.0 mm`, 0 crashes!)
    let sim_sq = chip_sim::run_setup_simulation(&chip_sim::SetupSimulationInput {
        doc: &empty_doc,
        setup: &resolved_sq,
        target_bodies: &[],
        fixtures: &[],
        operations: &[chip_sim::SimOperationInput::from_operation(
            &op_pure,
            &vbit_90_wide,
            &tp_pure,
        )],
        resolution: chip_sim::SimResolution::Fine,
        kinematics: chip_sim::MachineKinematicsConfig::default(),
        verify_config: chip_sim::SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(sim_sq.critical_error_count(), 0);
    let z_center = sim_sq.final_stock.sample_stock_z(0.0, 0.0).unwrap();
    let z_wall_mid = sim_sq.final_stock.sample_stock_z(5.0, 0.0).unwrap();
    assert!(approx_eq(z_center, -10.0, 0.08), "got z_center={z_center}");
    assert!(
        approx_eq(z_wall_mid, -5.0, 0.12),
        "analytical 45° bevel wall at x=5 must be Z=-5.0 mm, got {z_wall_mid}"
    );

    // 2. Two-Stage Flat-Bottom V-Carve (`T1` Ø6mm Flat Endmill + `T8` 60° V-Bit) on bold TrueType "B8O" + V-Inlay Male Plug!
    let mut sk_txt = SketchLayer::new("sk_b8o", "Bold Text", "setup_1");
    sk_txt.entities.push(SketchEntityEntry {
        id: "txt_b8o".to_string(),
        name: "B8O".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Text(SketchTextSpec {
            content: "B8O".to_string(),
            font: SketchFontSource::BuiltinSansBold,
            cap_height: ParamLength::from_mm(42.0),
            ..SketchTextSpec::default()
        }),
    });
    sk_txt.rebuild_regions(None);
    let sketches_txt = [sk_txt];
    let resolved_txt = setup
        .resolve_with_sketches(&empty_doc, &sketches_txt, DisplayUnit::Millimeter)
        .unwrap();

    let mut op_two_tool = Operation::new(
        "op_vcarve_2tool",
        "B8O Flat-Bottom V-Carve",
        OperationKind::VCarve,
    );
    op_two_tool.tool_id = Some(vbit_60.id.clone());
    op_two_tool
        .selections
        .push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sk_b8o".to_string(),
            entity_id: "txt_b8o".to_string(),
            reverse: false,
            side_flip: false,
        });
    if let StrategyParams::VCarve(vp) = op_two_tool.strategy_mut() {
        vp.flat_depth = Some(ParamLength::from_mm(2.0));
        vp.clearing_tool_id = Some(flat_6mm.id.clone());
        vp.clearing_strategy = VCarveClearingKind::ContourParallel;
    }

    let tp_two = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &empty_doc,
        sketches: &sketches_txt,
        setup: &resolved_txt,
        op: &op_two_tool,
        tool: vbit_60,
        clearing_tool: Some(flat_6mm),
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    let stage1 = tp_two
        .clearing_stage
        .as_ref()
        .expect("expected Stage 1 Flat Endmill clearing sub-toolpath");
    assert_eq!(stage1.tool_id, flat_6mm.id);
    assert!(!stage1.toolpath.segments_wcs.is_empty());

    let sim_two = chip_sim::run_setup_simulation(&chip_sim::SetupSimulationInput {
        doc: &empty_doc,
        setup: &resolved_txt,
        target_bodies: &[],
        fixtures: &[],
        operations: &[
            chip_sim::SimOperationInput::from_operation(&op_two_tool, flat_6mm, &stage1.toolpath),
            chip_sim::SimOperationInput::from_operation(&op_two_tool, vbit_60, &tp_two),
        ],
        resolution: chip_sim::SimResolution::Fine,
        kinematics: chip_sim::MachineKinematicsConfig::default(),
        verify_config: chip_sim::SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim_two.critical_error_count(),
        0,
        "Two-tool Flat-Bottom V-Carve must pass Fine heightmap sim with 0 crashes: {:?}",
        sim_two.diagnostics
    );

    // 3. Male V-Inlay Plug (`start_depth = 1.6 mm`, `glue_gap_bottom = 0.6 mm` => floor `Z = -2.2 mm`)
    let mut op_plug = Operation::new("op_inlay_plug", "B8O Male Plug", OperationKind::VCarve);
    op_plug.tool_id = Some(vbit_60.id.clone());
    op_plug.selections = op_two_tool.selections.clone();
    if let StrategyParams::VCarve(vp) = op_plug.strategy_mut() {
        vp.clearing_tool_id = Some(flat_6mm.id.clone());
        vp.inlay_mode = VCarveInlayMode::MalePlug {
            start_depth: ParamLength::from_mm(1.6),
            glue_gap_bottom: ParamLength::from_mm(0.6),
            border_margin: ParamLength::from_mm(4.0),
            auto_mirror_x: true,
        };
    }
    let tp_plug = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &empty_doc,
        sketches: &sketches_txt,
        setup: &resolved_txt,
        op: &op_plug,
        tool: vbit_60,
        clearing_tool: Some(flat_6mm),
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    assert!(approx_eq(tp_plug.stats.bounds_wcs.min.z, -2.2, 0.05));
    assert!(tp_plug.clearing_stage.is_some());
}

#[test]
fn verifies_adaptive_3d_and_waterline_3d_on_stl_obj_and_step_against_sim_oracle() {
    use chip_cad::{
        MeshImportOptions, load_mesh_cad_document, sample_relief_wave_obj_text,
        sample_sculpted_dome_stl_bytes,
    };
    use chip_cam::{
        HeightReference, HeightSpec, MachiningBoundaryMode, OperationGenRequest, SegmentMotion3D,
        StockConfig, StrategyParams, ToolContainmentMode, WaterlineOrderMode,
        generate_operation_toolpath_with_rest,
    };
    use chip_sim::{
        MachineKinematicsConfig, SetupSimulationInput, SimOperationInput, SimResolution,
        SimVerificationConfig, run_setup_simulation,
    };
    use chip_sketch::{RectAnchorMode, SketchEntity, SketchEntityEntry, SketchLayer};

    let starter = default_starter_tool_library();
    let flat_6mm = &starter.tools[0]; // 6mm flat endmill
    let ball_6mm = starter
        .tools
        .iter()
        .find(|t| matches!(t.geometry, chip_tools::CutterGeometry::BallEndmill { .. }))
        .unwrap();
    let cancel = AtomicBool::new(false);

    // === 1. Sculpted Dome (STL Mesh): Op 1 Adaptive3D (Flat) + Op 2 Waterline3D (Ball) ===
    let dome_stl = sample_sculpted_dome_stl_bytes();
    let dome_doc = load_mesh_cad_document(
        "sculpted_dome.stl",
        &dome_stl,
        &MeshImportOptions::default(),
    )
    .unwrap();
    let mut setup = Setup::new("setup_dome", "3D Dome Setup", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(2.0),
        side_neg_x: ParamLength::from_mm(2.0),
        side_pos_y: ParamLength::from_mm(2.0),
        side_neg_y: ParamLength::from_mm(2.0),
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(12.0),
        round_increment_mm: None,
    };
    let resolved_dome = setup.resolve(&dome_doc, DisplayUnit::Millimeter).unwrap();
    // Stock top = 0.0, Model top (dome apex) = -1.0, Pedestal flat top = -26.0, Model bottom = -34.0

    let mut op_adap3d = Operation::new("op_adap3d", "3D Adaptive Rough", OperationKind::Adaptive3D);
    op_adap3d.tool_id = Some(flat_6mm.id.clone());
    op_adap3d.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -25.0);
    op_adap3d.passes.stock_to_leave_radial = ParamLength::from_mm(0.5);
    op_adap3d.passes.stock_to_leave_axial = ParamLength::from_mm(0.3);
    if let StrategyParams::Adaptive3D(p) = op_adap3d.strategy_mut() {
        p.max_roughing_stepdown = ParamLength::from_mm(6.0);
        p.fine_stepdown = ParamLength::from_mm(2.0);
        p.adaptive.optimal_load = ParamLength::from_mm(1.5);
        p.detect_flats = true;
    }

    let tp_adap3d = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &dome_doc,
        sketches: &[],
        setup: &resolved_dome,
        op: &op_adap3d,
        tool: flat_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    assert!(
        tp_adap3d.segments_wcs.len() > 40,
        "Adaptive3D should generate rich multi-level trochoidal passes"
    );

    // Verify flat-plateau landing near pedestal top `Z = -26.0 + 0.3 = -25.7 mm`
    let landed_on_pedestal = tp_adap3d.segments_wcs.iter().any(|s| {
        matches!(s.motion, SegmentMotion3D::Linear { .. }) && (s.end_wcs.z - (-25.7)).abs() < 0.08
    });
    assert!(
        landed_on_pedestal,
        "Adaptive3D with detect_flats=true must land cleanly on pedestal plateau (-25.7 mm)"
    );

    let mut op_wl3d = Operation::new(
        "op_wl3d",
        "3D Waterline Semi-Finish",
        OperationKind::Waterline3D,
    );
    op_wl3d.tool_id = Some(ball_6mm.id.clone());
    op_wl3d.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -25.0);
    op_wl3d.passes.max_stepdown = ParamLength::from_mm(2.0);
    if let StrategyParams::Waterline3D(p) = op_wl3d.strategy_mut() {
        p.adaptive_stepdown = true;
        p.detect_flats = true;
        p.order_mode = WaterlineOrderMode::ByLevel;
    }

    let tp_wl3d = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &dome_doc,
        sketches: &[],
        setup: &resolved_dome,
        op: &op_wl3d,
        tool: ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    assert!(
        tp_wl3d.segments_wcs.len() > 40,
        "Waterline3D should emit smooth Z-level dome contours"
    );

    // Run full 5-rule `chip-sim` physical verification on Adaptive3D + Waterline3D together!
    let sim_ops = vec![
        SimOperationInput::from_operation(&op_adap3d, flat_6mm, &tp_adap3d),
        SimOperationInput::from_operation(&op_wl3d, ball_6mm, &tp_wl3d),
    ];
    let sim_res = run_setup_simulation(&SetupSimulationInput {
        doc: &dome_doc,
        setup: &resolved_dome,
        target_bodies: &[],
        fixtures: &[],
        operations: &sim_ops,
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim_res.critical_error_count(),
        0,
        "Sculpted Dome Adaptive3D + Waterline3D must pass 5-rule simulation with 0 crashes/gouges, got: {:#?}",
        sim_res.diagnostics
    );
    assert!(
        sim_res.removed_volume_mm3 > 25_000.0,
        "Expected substantial 3D stock removal around dome, got {:.1} mm³",
        sim_res.removed_volume_mm3
    );

    // === 2. Steep-Slope Filtering + 2D Sketch Containment Clipping on Waterline3D ===
    let mut sk = SketchLayer::new("sk_bound", "2D Window", "setup_dome");
    sk.entities.push(SketchEntityEntry {
        id: "rect_clip".to_string(),
        name: "Clip Rect".to_string(),
        is_construction: false,
        sub_layer: Some("DEFAULT".to_string()),
        entity: SketchEntity::Rectangle {
            x: ParamLength::from_mm(0.0),
            y: ParamLength::from_mm(0.0),
            width: ParamLength::from_mm(40.0),
            height: ParamLength::from_mm(40.0),
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
            anchor: RectAnchorMode::Center,
        },
    });
    sk.rebuild_regions(None);

    let mut op_wl_clip = Operation::new(
        "op_wl_clip",
        "Steep Clipped Waterline",
        OperationKind::Waterline3D,
    );
    op_wl_clip.tool_id = Some(ball_6mm.id.clone());
    op_wl_clip.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -24.0);
    op_wl_clip.passes.max_stepdown = ParamLength::from_mm(2.0);
    op_wl_clip
        .selections
        .push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sk_bound".to_string(),
            sub_layer: None,
            reverse: false,
            side_flip: false,
        });
    if let StrategyParams::Waterline3D(p) = op_wl_clip.strategy_mut() {
        p.boundary_mode = MachiningBoundaryMode::SelectedRegions;
        p.tool_containment = ToolContainmentMode::ToolInsideBoundary;
        p.boundary_offset = ParamLength::from_mm(0.0);
        p.min_slope_deg = 35.0;
        p.max_slope_deg = 90.0;
        p.adaptive_stepdown = false;
    }

    let tp_clip = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &dome_doc,
        sketches: &[sk],
        setup: &resolved_dome,
        op: &op_wl_clip,
        tool: ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    // With 40×40 rect (`±20 mm`) and `ToolInsideBoundary` (`R = 3 mm`), every cutting XY must stay inside `±17.1 mm`!
    for seg in &tp_clip.segments_wcs {
        if !matches!(seg.motion, SegmentMotion3D::Rapid) {
            assert!(
                seg.end_wcs.x.abs() <= 17.2 && seg.end_wcs.y.abs() <= 17.2,
                "Clipped Waterline3D escaped ToolInsideBoundary (±17 mm): {:?}",
                seg.end_wcs
            );
        }
    }
    // Slope >= 35° must skip shallow dome apex (`Z > -5 mm` has slope < 35°)
    let max_cut_z = tp_clip
        .segments_wcs
        .iter()
        .filter(|s| {
            matches!(
                s.motion,
                SegmentMotion3D::Linear {
                    role: chip_cam::MotionRole::FinishCut,
                    ..
                } | SegmentMotion3D::Arc {
                    role: chip_cam::MotionRole::FinishCut,
                    ..
                }
            )
        })
        .map(|s| s.end_wcs.z)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(
        max_cut_z < -3.0,
        "min_slope_deg=35° should skip shallow dome apex, got max_cut_z={max_cut_z:.2}"
    );

    // === 3. Relief Wave (OBJ) + 3D STEP Solid (`sample_cam_torture_block_step`) ===
    let wave_obj = sample_relief_wave_obj_text();
    let wave_doc = load_mesh_cad_document(
        "relief_wave.obj",
        wave_obj.as_bytes(),
        &MeshImportOptions::default(),
    )
    .unwrap();
    let mut wave_setup = Setup::new("setup_wave", "Wave Setup", WorkOffsetSlot::G54);
    let resolved_wave = wave_setup
        .resolve(&wave_doc, DisplayUnit::Millimeter)
        .unwrap();
    let mut op_wave = Operation::new("op_wave_a3d", "Wave 3D Adaptive", OperationKind::Adaptive3D);
    op_wave.tool_id = Some(flat_6mm.id.clone());
    op_wave.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -8.0);
    if let StrategyParams::Adaptive3D(p) = op_wave.strategy_mut() {
        p.max_roughing_stepdown = ParamLength::from_mm(4.0);
        p.fine_stepdown = ParamLength::from_mm(2.0);
        p.adaptive.optimal_load = ParamLength::from_mm(1.5);
    }
    let tp_wave = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &wave_doc,
        sketches: &[],
        setup: &resolved_wave,
        op: &op_wave,
        tool: flat_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    let sim_wave = run_setup_simulation(&SetupSimulationInput {
        doc: &wave_doc,
        setup: &resolved_wave,
        target_bodies: &[],
        fixtures: &[],
        operations: &[SimOperationInput::from_operation(
            &op_wave, flat_6mm, &tp_wave,
        )],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim_wave.critical_error_count(),
        0,
        "OBJ Relief Wave Adaptive3D sim errors: {:#?}",
        sim_wave.diagnostics
    );

    // 3D Waterline on STEP B-Rep solid (`sample_cam_torture_block_step`)
    let step_str = sample_cam_torture_block_step().unwrap();
    let step_doc = MonstertruckKernel
        .load_step_bytes(
            "torture.step",
            step_str.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    let mut step_setup = Setup::new("setup_step3d", "STEP 3D", WorkOffsetSlot::G54);
    let resolved_step = step_setup
        .resolve(&step_doc, DisplayUnit::Millimeter)
        .unwrap();
    let mut op_step_wl = Operation::new("op_step_wl", "STEP Waterline", OperationKind::Waterline3D);
    op_step_wl.tool_id = Some(ball_6mm.id.clone());
    op_step_wl.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -8.0);
    op_step_wl.passes.max_stepdown = ParamLength::from_mm(2.5);
    let tp_step_wl = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &step_doc,
        sketches: &[],
        setup: &resolved_step,
        op: &op_step_wl,
        tool: ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    assert!(!tp_step_wl.segments_wcs.is_empty());
}

#[test]
fn milestone_18_3d_finishing_parallel_scallop_pencil_and_rest_oracles() {
    use chip_cad::{
        MeshImportOptions, load_mesh_cad_document, sample_relief_wave_obj_text,
        sample_sculpted_dome_stl_bytes,
    };
    use chip_cam::{
        MachiningBoundaryMode, OperationGenRequest, ParallelCutDirection, RestMachiningConfig,
        RestMachiningMode, ToolContainmentMode, generate_operation_toolpath_with_rest,
    };
    use chip_sim::{
        MachineKinematicsConfig, SetupSimulationInput, SimOperationInput, SimResolution,
        SimVerificationConfig, run_setup_simulation,
    };
    use chip_tools::{
        CoolantMode, CutterGeometry, CuttingPreset, SpindleDirection, SurfaceSpeedPresetSpec, Tool,
        ToolId,
    };

    let mut ball_6mm = default_starter_tool_library()
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::BallEndmill { .. }))
        .unwrap()
        .clone();
    ball_6mm.stickout_length_mm = 32.0;

    let ball_3mm = Tool {
        id: ToolId("t_3mm_ball_finish".to_string()),
        tool_number: 11,
        length_offset_number: 11,
        diameter_offset_number: 11,
        name: "3mm 2FL Carbide Ball Endmill".to_string(),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::AirBlast,
        stickout_length_mm: 32.0,
        geometry: CutterGeometry::BallEndmill {
            diameter_mm: 3.0,
            flute_length_mm: 16.0,
            overall_length_mm: 55.0,
            shank_diameter_mm: 3.0,
            flutes: 2,
        },
        presets: vec![CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
            id: "al6061_finish",
            name: "Aluminum 6061 - 3D Finish",
            material: "Aluminum 6061",
            diameter_mm: 3.0,
            flutes: 2,
            surface_speed_m_min: 220.0,
            chipload_mm_tooth: 0.020,
            default_stepdown_mm: 0.8,
            default_stepover_mm: 0.5,
            coolant_override: Some(CoolantMode::Mist),
        })],
    };

    let cancel = AtomicBool::new(false);

    // === 1. Sculpted Dome (STL): Parallel3D (45°, cross_pass, shallow slope limit) ===
    let dome_stl = sample_sculpted_dome_stl_bytes();
    let dome_doc = load_mesh_cad_document(
        "sculpted_dome.stl",
        &dome_stl,
        &MeshImportOptions::default(),
    )
    .unwrap();
    let mut dome_setup = Setup::new("setup_dome_m18", "Dome M18 Setup", WorkOffsetSlot::G54);
    dome_setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    dome_setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(2.0),
        side_neg_x: ParamLength::from_mm(2.0),
        side_pos_y: ParamLength::from_mm(2.0),
        side_neg_y: ParamLength::from_mm(2.0),
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(12.0),
        round_increment_mm: None,
    };
    let resolved_dome = dome_setup
        .resolve(&dome_doc, DisplayUnit::Millimeter)
        .unwrap();

    // 1A. Parallel3D single direction vs cross_pass (doubles cutting distance)
    let mut op_par = Operation::new("op_par3d", "3D Parallel 45°", OperationKind::Parallel3D);
    op_par.tool_id = Some(ball_6mm.id.clone());
    op_par.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -25.0);
    if let StrategyParams::Parallel3D(p) = op_par.strategy_mut() {
        p.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.tool_containment = ToolContainmentMode::ToolInsideBoundary;
        p.use_cusp_height = true;
        p.cusp_height = ParamLength::from_mm(0.08);
        p.pass_angle_deg = 45.0;
        p.cross_pass = false;
        p.cut_direction = ParallelCutDirection::ZigZag;
    }
    let tp_par_single = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &dome_doc,
        sketches: &[],
        setup: &resolved_dome,
        op: &op_par,
        tool: &ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    let mut op_par_cross = op_par.clone();
    op_par_cross.id = "op_par3d_cross".to_string();
    if let StrategyParams::Parallel3D(p) = op_par_cross.strategy_mut() {
        p.cross_pass = true;
    }
    let tp_par_cross = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &dome_doc,
        sketches: &[],
        setup: &resolved_dome,
        op: &op_par_cross,
        tool: &ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    assert!(
        tp_par_cross.stats.cutting_distance_mm > 1.8 * tp_par_single.stats.cutting_distance_mm,
        "cross_pass=true should roughly double Parallel3D cutting distance ({:.1} vs {:.1})",
        tp_par_cross.stats.cutting_distance_mm,
        tp_par_single.stats.cutting_distance_mm
    );

    // 1B. Shallow slope filter (`0°..40°`) skips steep dome equator walls
    let mut op_par_shallow = op_par.clone();
    op_par_shallow.id = "op_par3d_shallow".to_string();
    if let StrategyParams::Parallel3D(p) = op_par_shallow.strategy_mut() {
        p.min_slope_deg = 0.0;
        p.max_slope_deg = 40.0;
    }
    let tp_par_shallow = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &dome_doc,
        sketches: &[],
        setup: &resolved_dome,
        op: &op_par_shallow,
        tool: &ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    let finish_dist = |tp: &chip_cam::GeneratedToolpath| -> f64 {
        tp.segments_wcs
            .iter()
            .filter(|s| {
                matches!(
                    s.motion,
                    SegmentMotion3D::Linear {
                        role: chip_cam::MotionRole::FinishCut,
                        ..
                    }
                )
            })
            .map(chip_cam::ToolpathSegment3D::length_mm)
            .sum()
    };
    let shallow_fin = finish_dist(&tp_par_shallow);
    let full_fin = finish_dist(&tp_par_single);
    assert!(
        shallow_fin < 0.80 * full_fin,
        "Shallow slope filter 0..40° should skip steep dome walls (finish_dist {shallow_fin:.1} vs {full_fin:.1})"
    );
    // Verify no FinishCut midpoints fall on the steep 60°..80° dome band (`r ∈ [21.5..24.5 mm]`)
    for seg in &tp_par_shallow.segments_wcs {
        if matches!(
            seg.motion,
            SegmentMotion3D::Linear {
                role: chip_cam::MotionRole::FinishCut,
                ..
            }
        ) {
            let mid = seg.start_wcs.lerp(seg.end_wcs, 0.5);
            let r = mid.x.hypot(mid.y);
            assert!(
                !(21.5..=24.5).contains(&r),
                "Shallow 0..40° Parallel3D cut steep dome wall at r={r:.2}, z={:.2}",
                mid.z
            );
        }
    }

    // 1C. 3D Rest Machining (`FromPreviousTool` Ø12mm -> Ø3mm) dramatically trims flat/convex areas
    let mut op_par_full_3mm = Operation::new(
        "op_par_full3",
        "Full 3mm Parallel",
        OperationKind::Parallel3D,
    );
    op_par_full_3mm.tool_id = Some(ball_3mm.id.clone());
    op_par_full_3mm.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -25.0);
    if let StrategyParams::Parallel3D(p) = op_par_full_3mm.strategy_mut() {
        p.use_cusp_height = false;
        p.stepover = ParamLength::from_mm(1.2);
    }
    let tp_full_3mm = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &dome_doc,
        sketches: &[],
        setup: &resolved_dome,
        op: &op_par_full_3mm,
        tool: &ball_3mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    let mut op_par_rest_3mm = op_par_full_3mm.clone();
    op_par_rest_3mm.id = "op_par_rest3".to_string();
    op_par_rest_3mm.passes.rest_machining = RestMachiningConfig {
        mode: RestMachiningMode::FromPreviousTool {
            prev_tool_diameter: ParamLength::from_mm(12.0),
            prev_stock_to_leave: ParamLength::from_mm(0.0),
        },
        rest_overlap: ParamLength::from_mm(0.5),
        min_rest_thickness: ParamLength::from_mm(0.1),
    };
    let tp_rest_3mm = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &dome_doc,
        sketches: &[],
        setup: &resolved_dome,
        op: &op_par_rest_3mm,
        tool: &ball_3mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    assert!(
        tp_rest_3mm.stats.cutting_distance_mm > 50.0
            && tp_rest_3mm.stats.cutting_distance_mm < 0.55 * tp_full_3mm.stats.cutting_distance_mm,
        "3D Rest Machining should focus only on unmachined concave crease ({:.1} vs full {:.1})",
        tp_rest_3mm.stats.cutting_distance_mm,
        tp_full_3mm.stats.cutting_distance_mm
    );

    // === 2. Scallop3D Slope-Adaptive Spacing Oracle on Sculpted Dome ===
    let mut op_scallop = Operation::new("op_scallop3d", "3D Scallop", OperationKind::Scallop3D);
    op_scallop.tool_id = Some(ball_6mm.id.clone());
    op_scallop.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -25.0);
    if let StrategyParams::Scallop3D(p) = op_scallop.strategy_mut() {
        p.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.tool_containment = ToolContainmentMode::ToolInsideBoundary;
        p.cusp_height = ParamLength::from_mm(0.08);
        p.max_stepover = ParamLength::from_mm(2.0);
        p.min_stepover = ParamLength::from_mm(0.2);
    }
    let tp_scallop = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &dome_doc,
        sketches: &[],
        setup: &resolved_dome,
        op: &op_scallop,
        tool: &ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    // Measure radial pass density along the +X axis (`|y| < 1.5 mm`, `x > 0`):
    // Steep wall annulus `r ∈ [16..26 mm]` must have noticeably tighter XY spacing (more ring crossings per mm)
    // than the shallow dome crest `r ∈ [2..12 mm]`!
    let mut steep_xs: Vec<f64> = Vec::new();
    let mut shallow_xs: Vec<f64> = Vec::new();
    for seg in &tp_scallop.segments_wcs {
        if matches!(
            seg.motion,
            SegmentMotion3D::Linear {
                role: chip_cam::MotionRole::FinishCut,
                ..
            }
        ) && (seg.start_wcs.y * seg.end_wcs.y <= 0.0)
            && seg.start_wcs.x > 0.0
            && seg.end_wcs.x > 0.0
        {
            let x_cross = 0.5 * (seg.start_wcs.x + seg.end_wcs.x);
            if (16.0..=26.0).contains(&x_cross) {
                steep_xs.push(x_cross);
            } else if (2.0..=12.0).contains(&x_cross) {
                shallow_xs.push(x_cross);
            }
        }
    }
    assert!(
        steep_xs.len() > shallow_xs.len(),
        "Scallop3D must pack tighter XY rings on steep dome wall (r=16..26: {} crossings) than shallow crest (r=2..12: {} crossings)",
        steep_xs.len(),
        shallow_xs.len()
    );

    // === 3. Pencil3D Exact Crease Radius Oracle on Sculpted Dome ===
    // Dome hemisphere R_d = 25 mm meets flat pedestal -> Ø3mm ball (R = 1.5 mm) exact bi-tangent
    // crease radius is `r = sqrt(25^2 + 2 * 25 * 1.5) = 26.46 mm`!
    let mut op_pencil = Operation::new("op_pencil3d", "3D Pencil Crease", OperationKind::Pencil3D);
    op_pencil.tool_id = Some(ball_3mm.id.clone());
    op_pencil.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -25.0);
    if let StrategyParams::Pencil3D(p) = op_pencil.strategy_mut() {
        p.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.tool_containment = ToolContainmentMode::ToolInsideBoundary;
        p.reference_tool_diameter = ParamLength::from_mm(6.0);
        p.valley_threshold = ParamLength::from_mm(0.15);
        p.num_offset_passes = 0;
    }
    let tp_pencil = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &dome_doc,
        sketches: &[],
        setup: &resolved_dome,
        op: &op_pencil,
        tool: &ball_3mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    let mut pencil_radii = Vec::new();
    for seg in &tp_pencil.segments_wcs {
        if matches!(
            seg.motion,
            SegmentMotion3D::Linear {
                role: chip_cam::MotionRole::FinishCut,
                ..
            }
        ) {
            pencil_radii.push(seg.end_wcs.x.hypot(seg.end_wcs.y));
        }
    }
    assert!(
        pencil_radii.len() >= 20,
        "Pencil3D should trace the full circular dome-pedestal crease, got {} segments",
        pencil_radii.len()
    );
    let mean_r = pencil_radii.iter().sum::<f64>() / pencil_radii.len() as f64;
    let exact_crease_r = (25.0_f64 * 25.0 + 2.0 * 25.0 * 1.5).sqrt(); // 26.4575 mm
    assert!(
        (mean_r - exact_crease_r).abs() < 0.6,
        "Pencil3D crease radius mean {mean_r:.3} mm should match exact bi-tangent radius {exact_crease_r:.3} mm"
    );

    // 3B. Pencil3D multi-pass lateral offsets (`num_offset_passes = 2`) increases cutting coverage cleanly
    let mut op_pencil_multi = op_pencil.clone();
    op_pencil_multi.id = "op_pencil3d_multi".to_string();
    if let StrategyParams::Pencil3D(p) = op_pencil_multi.strategy_mut() {
        p.num_offset_passes = 2;
        p.offset_stepover = ParamLength::from_mm(0.5);
    }
    let tp_pencil_multi = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &dome_doc,
        sketches: &[],
        setup: &resolved_dome,
        op: &op_pencil_multi,
        tool: &ball_3mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    assert!(
        tp_pencil_multi.stats.cutting_distance_mm > 3.0 * tp_pencil.stats.cutting_distance_mm,
        "Pencil3D num_offset_passes=2 should emit 5 passes (center + 2 each side), got {:.1} vs {:.1}",
        tp_pencil_multi.stats.cutting_distance_mm,
        tp_pencil.stats.cutting_distance_mm
    );

    // === 4. Full 5-Rule `chip-sim` Physical Verification (0 crashes / 0 gouges) ===
    // Verify Parallel3D + Pencil3D on Sculpted Dome, and Scallop3D on Relief Wave (OBJ)!
    let sim_dome = run_setup_simulation(&SetupSimulationInput {
        doc: &dome_doc,
        setup: &resolved_dome,
        target_bodies: &[],
        fixtures: &[],
        operations: &[
            SimOperationInput::from_operation(&op_par, &ball_6mm, &tp_par_single),
            SimOperationInput::from_operation(&op_pencil, &ball_3mm, &tp_pencil),
        ],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim_dome.critical_error_count(),
        0,
        "Dome Parallel3D + Pencil3D must pass 5-rule simulation with 0 crashes/gouges, got: {:#?}",
        sim_dome.diagnostics
    );

    let wave_obj = sample_relief_wave_obj_text();
    let wave_doc = load_mesh_cad_document(
        "relief_wave.obj",
        wave_obj.as_bytes(),
        &MeshImportOptions::default(),
    )
    .unwrap();
    let mut wave_setup = Setup::new("setup_wave_m18", "Wave M18", WorkOffsetSlot::G54);
    let resolved_wave = wave_setup
        .resolve(&wave_doc, DisplayUnit::Millimeter)
        .unwrap();

    let mut op_wave_par = Operation::new("op_wave_par", "Wave Parallel", OperationKind::Parallel3D);
    op_wave_par.tool_id = Some(ball_6mm.id.clone());
    if let StrategyParams::Parallel3D(p) = op_wave_par.strategy_mut() {
        p.use_cusp_height = true;
        p.cusp_height = ParamLength::from_mm(0.06);
        p.pass_angle_deg = 30.0;
    }
    let tp_wave_par = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &wave_doc,
        sketches: &[],
        setup: &resolved_wave,
        op: &op_wave_par,
        tool: &ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    let mut op_wave_sc = Operation::new("op_wave_sc", "Wave Scallop", OperationKind::Scallop3D);
    op_wave_sc.tool_id = Some(ball_6mm.id.clone());
    if let StrategyParams::Scallop3D(p) = op_wave_sc.strategy_mut() {
        p.cusp_height = ParamLength::from_mm(0.06);
    }
    let tp_wave_sc = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &wave_doc,
        sketches: &[],
        setup: &resolved_wave,
        op: &op_wave_sc,
        tool: &ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    let sim_wave = run_setup_simulation(&SetupSimulationInput {
        doc: &wave_doc,
        setup: &resolved_wave,
        target_bodies: &[],
        fixtures: &[],
        operations: &[
            SimOperationInput::from_operation(&op_wave_par, &ball_6mm, &tp_wave_par),
            SimOperationInput::from_operation(&op_wave_sc, &ball_6mm, &tp_wave_sc),
        ],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim_wave.critical_error_count(),
        0,
        "Wave Parallel3D + Scallop3D must pass 5-rule simulation with 0 crashes/gouges, got: {:#?}",
        sim_wave.diagnostics
    );
}

#[test]
fn rotary2linear_spiral_and_axial_parallel_on_x_and_y_axes_are_centerline_locked_and_crash_free() {
    use chip_cad::{MeshImportOptions, load_mesh_cad_document, sample_rotary_shaft_stl_bytes};
    use chip_cam::{
        AxisAnchor, OperationGenRequest, RotaryAxisKind, RotaryCutDirection, RotaryDriveMode,
        RotaryPatternMode, StockAxis, generate_operation_toolpath_with_rest,
    };
    use chip_sim::{
        MachineKinematicsConfig, SetupSimulationInput, SimOperationInput, SimResolution,
        SimVerificationConfig, run_setup_simulation,
    };

    let stl = sample_rotary_shaft_stl_bytes();
    let doc =
        load_mesh_cad_document("rotary_shaft.stl", &stl, &MeshImportOptions::default()).unwrap();

    let lib = default_starter_tool_library();
    let ball_6mm = lib
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::BallEndmill { diameter_mm, .. } if approx_eq(diameter_mm, 6.0, 1e-3)
            )
        })
        .expect("6mm ball endmill in starter library")
        .clone();

    // 1. X-Axis (A) 2-Linear + Rotary Setup (Z+X+A)
    let mut setup_x = Setup::new("setup_rot_x", "Rotary Shaft X/A", WorkOffsetSlot::G54);
    setup_x.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27 {
            x: AxisAnchor::Center,
            y: AxisAnchor::Center,
            z: AxisAnchor::Center,
        },
    };
    setup_x.stock = StockConfig::FixedCylinder {
        axis: StockAxis::X,
        diameter: ParamLength::from_mm(42.0),
        length: ParamLength::from_mm(80.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup_x.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
    setup_x.rotary.axis = RotaryAxisKind::XAxis;
    setup_x.rotary.clearance_radial_offset = ParamLength::from_mm(12.0);
    let resolved_x = setup_x.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let rot_x = resolved_x.rotary.unwrap();

    let cancel = AtomicBool::new(false);

    // Op A: Continuous Spiral over the turned central collar & 3-lobe cam (X in [-6..+32])
    let mut op_spiral = Operation::new(
        "op_rot_spiral",
        "Rotary Spiral",
        OperationKind::Rotary2Linear,
    );
    op_spiral.tool_id = Some(ball_6mm.id.clone());
    op_spiral.passes.multiple_depths = true;
    op_spiral.passes.max_stepdown = ParamLength::from_mm(3.0);
    if let StrategyParams::Rotary2Linear(p) = op_spiral.strategy_mut() {
        p.pattern = RotaryPatternMode::ContinuousSpiral;
        p.stepover = ParamLength::from_mm(1.5);
        p.axial_margin_start = ParamLength::from_mm(34.0);
        p.axial_margin_end = ParamLength::from_mm(8.0);
        p.angular_sample_deg = 5.0;
        p.axial_sample_step = ParamLength::from_mm(1.0);
    }
    let tp_spiral = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_x,
        op: &op_spiral,
        tool: &ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    assert!(
        tp_spiral.segments_wcs.len() > 200,
        "spiral should produce dense rotary motion, got {}",
        tp_spiral.segments_wcs.len()
    );
    for seg in &tp_spiral.segments_wcs {
        assert!(
            (seg.start_wcs.y - rot_x.center_wcs.y).abs() < 1e-9
                && (seg.end_wcs.y - rot_x.center_wcs.y).abs() < 1e-9,
            "Z+X+A spiral violated strict Y centerline lock: {:?} -> {:?}",
            seg.start_wcs,
            seg.end_wcs
        );
    }

    // Op B: Axial Parallel ZigZag
    let mut op_axial = Operation::new(
        "op_rot_axial",
        "Rotary Axial Parallel",
        OperationKind::Rotary2Linear,
    );
    op_axial.tool_id = Some(ball_6mm.id.clone());
    if let StrategyParams::Rotary2Linear(p) = op_axial.strategy_mut() {
        p.pattern = RotaryPatternMode::AxialParallel;
        p.cut_direction = RotaryCutDirection::ZigZag;
        p.stepover = ParamLength::from_mm(3.0);
        p.axial_margin_start = ParamLength::from_mm(34.0);
        p.axial_margin_end = ParamLength::from_mm(8.0);
        p.angular_sample_deg = 5.0;
        p.axial_sample_step = ParamLength::from_mm(1.0);
    }
    let tp_axial = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_x,
        op: &op_axial,
        tool: &ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    for seg in &tp_axial.segments_wcs {
        assert!(
            (seg.start_wcs.y - rot_x.center_wcs.y).abs() < 1e-9
                && (seg.end_wcs.y - rot_x.center_wcs.y).abs() < 1e-9,
            "Z+X+A axial parallel violated strict Y centerline lock: {:?} -> {:?}",
            seg.start_wcs,
            seg.end_wcs
        );
    }

    let sim_x = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved_x,
        target_bodies: &[],
        fixtures: &[],
        operations: &[
            SimOperationInput::from_operation(&op_spiral, &ball_6mm, &tp_spiral),
            SimOperationInput::from_operation(&op_axial, &ball_6mm, &tp_axial),
        ],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim_x.critical_error_count(),
        0,
        "Rotary2Linear Spiral + AxialParallel must pass CylindricalStock verification crash-free, got: {:#?}",
        sim_x.diagnostics
    );
    assert!(
        sim_x.removed_volume_mm3 > 1_500.0,
        "Expected substantial material removal on rotary shaft, got {:.1} mm3",
        sim_x.removed_volume_mm3
    );

    // 2. Y-Axis (B) 2-Linear + Rotary Setup (Z+Y+B) via 90-degree WCS clock
    let mut setup_y = Setup::new("setup_rot_y", "Rotary Shaft Y/B", WorkOffsetSlot::G54);
    setup_y.wcs.orientation = chip_cam::WcsOrientationMode::ModelOrientation {
        flip_z: false,
        flip_x: false,
        clock_90: 1,
        extra_z_rot_deg: 0.0,
    };
    setup_y.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27 {
            x: AxisAnchor::Center,
            y: AxisAnchor::Center,
            z: AxisAnchor::Center,
        },
    };
    setup_y.stock = StockConfig::FixedCylinder {
        axis: StockAxis::Y,
        diameter: ParamLength::from_mm(42.0),
        length: ParamLength::from_mm(80.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup_y.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
    setup_y.rotary.axis = RotaryAxisKind::YAxis;
    setup_y.rotary.clearance_radial_offset = ParamLength::from_mm(12.0);
    let resolved_y = setup_y.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let rot_y = resolved_y.rotary.unwrap();

    let tp_y = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_y,
        op: &op_spiral,
        tool: &ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    for seg in &tp_y.segments_wcs {
        assert!(
            (seg.start_wcs.x - rot_y.center_wcs.x).abs() < 1e-9
                && (seg.end_wcs.x - rot_y.center_wcs.x).abs() < 1e-9,
            "Z+Y+B spiral violated strict X centerline lock: {:?} -> {:?}",
            seg.start_wcs,
            seg.end_wcs
        );
    }
    let sim_y = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved_y,
        target_bodies: &[],
        fixtures: &[],
        operations: &[SimOperationInput::from_operation(
            &op_spiral, &ball_6mm, &tp_y,
        )],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim_y.critical_error_count(),
        0,
        "Z+Y+B Rotary2Linear must pass CylindricalStock verification crash-free, got: {:#?}",
        sim_y.diagnostics
    );
}

#[test]
fn wrap_cylinder_2d_pocket_respects_centerline_lock_arc_mapping_and_simulates_cleanly() {
    use chip_cad::CadDocument;
    use chip_cam::{
        AxisAnchor, OperationGenRequest, OperationRotaryMode, RotaryAxisKind, RotaryDriveMode,
        StockAxis, generate_operation_toolpath_with_rest,
    };
    use chip_sim::{
        MachineKinematicsConfig, SetupSimulationInput, SimOperationInput, SimResolution,
        SimVerificationConfig, run_setup_simulation,
    };
    use chip_sketch::{RectAnchorMode, SketchEntity, SketchEntityEntry, SketchLayer};

    let doc = CadDocument::empty();
    let mut setup = Setup::new("setup_wrap_2d", "Wrapped Cylinder", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27 {
            x: AxisAnchor::Center,
            y: AxisAnchor::Center,
            z: AxisAnchor::Center,
        },
    };
    setup.stock = StockConfig::FixedCylinder {
        axis: StockAxis::X,
        diameter: ParamLength::from_mm(40.0),
        length: ParamLength::from_mm(80.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
    setup.rotary.axis = RotaryAxisKind::XAxis;
    setup.rotary.clearance_radial_offset = ParamLength::from_mm(10.0);

    // 2D rectangular slot spanning X in [-12, +12], Y (circumferential arc) in [-10, +10]
    let mut sketch = SketchLayer::new("sk_wrap_slot", "Wrap Slot", setup.id.clone());
    sketch.entities.push(SketchEntityEntry {
        id: "slot_rect".to_string(),
        name: "Slot".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(0.0),
            y: ParamLength::from_mm(0.0),
            width: ParamLength::from_mm(24.0),
            height: ParamLength::from_mm(20.0),
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
        },
    });
    sketch.rebuild_regions(None);
    let sketches = vec![sketch];

    let resolved = setup
        .resolve_with_sketches(&doc, &sketches, DisplayUnit::Millimeter)
        .unwrap();
    let rot = resolved.rotary.unwrap();

    let lib = default_starter_tool_library();
    let flat_3mm = lib
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. } if diameter_mm < 4.0
            )
        })
        .expect("small flat endmill in starter library")
        .clone();
    let tool_r = 0.5 * flat_3mm.geometry.cutting_diameter_mm();

    let mut op = Operation::new("op_wrap_pocket", "Wrapped Slot", OperationKind::Pocket2D);
    op.tool_id = Some(flat_3mm.id.clone());
    op.selections.push(GeometrySelectionItem::SketchLayerAll {
        sketch_id: "sk_wrap_slot".to_string(),
        sub_layer: None,
        reverse: false,
        side_flip: false,
    });
    op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op.heights.bottom = HeightSpec::new(HeightReference::StockTop, -2.0);
    op.rotary = OperationRotaryMode::WrapCylinder {
        wrap_radius: ParamLength::from_mm(20.0),
        axial_offset: ParamLength::from_mm(0.0),
        angle_offset_deg: 0.0,
    };

    let cancel = AtomicBool::new(false);
    let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &sketches,
        setup: &resolved,
        op: &op,
        tool: &flat_3mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    let mut min_deg = f64::INFINITY;
    let mut max_deg = f64::NEG_INFINITY;
    let mut min_r = f64::INFINITY;
    for seg in &tp.segments_wcs {
        assert!(
            (seg.start_wcs.y - rot.center_wcs.y).abs() < 1e-9
                && (seg.end_wcs.y - rot.center_wcs.y).abs() < 1e-9,
            "WrapCylinder violated strict Y centerline lock: {:?} -> {:?}",
            seg.start_wcs,
            seg.end_wcs
        );
        min_deg = min_deg.min(seg.start_rotary_deg).min(seg.end_rotary_deg);
        max_deg = max_deg.max(seg.start_rotary_deg).max(seg.end_rotary_deg);
        min_r = min_r
            .min(seg.start_wcs.z - rot.center_wcs.z)
            .min(seg.end_wcs.z - rot.center_wcs.z);
    }
    // Pocket half-width = 10 - tool_r -> angle = ±((10 - tool_r)/20 rad)
    let expected_deg = ((10.0 - tool_r) / 20.0).to_degrees();
    assert!(
        (max_deg - expected_deg).abs() < 1.0 && (min_deg + expected_deg).abs() < 1.0,
        "Expected wrapped angular span approx ±{expected_deg:.2}°, got [{min_deg:.2}°, {max_deg:.2}°]"
    );
    assert!(
        (min_r - 18.0).abs() < 0.05,
        "Expected cut floor radius 18.0 mm (R20 - 2mm depth), got {min_r:.3}"
    );

    let sim = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved,
        target_bodies: &[],
        fixtures: &[],
        operations: &[SimOperationInput::from_operation(&op, &flat_3mm, &tp)],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim.critical_error_count(),
        0,
        "WrapCylinder 2D pocket must simulate with 0 critical errors, got: {:#?}",
        sim.diagnostics
    );
    assert!(
        sim.removed_volume_mm3 > 500.0,
        "Expected > 500 mm3 removed by wrapped pocket, got {:.1}",
        sim.removed_volume_mm3
    );
}

#[test]
fn indexed_3plus1_four_flats_on_rotary_shaft_retract_safely_and_verify_crash_free() {
    use chip_cad::{MeshImportOptions, load_mesh_cad_document, sample_rotary_shaft_stl_bytes};
    use chip_cam::{
        AxisAnchor, OperationGenRequest, OperationRotaryMode, RotaryAxisKind, RotaryDriveMode,
        StockAxis, generate_operation_toolpath_with_rest,
    };
    use chip_sim::{
        MachineKinematicsConfig, SetupSimulationInput, SimOperationInput, SimResolution,
        SimVerificationConfig, run_setup_simulation,
    };
    use chip_sketch::{RectAnchorMode, SketchEntity, SketchEntityEntry, SketchLayer};

    let stl = sample_rotary_shaft_stl_bytes();
    let doc =
        load_mesh_cad_document("rotary_shaft.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new("setup_idx_3p1", "Indexed Shaft", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27 {
            x: AxisAnchor::Center,
            y: AxisAnchor::Center,
            z: AxisAnchor::Center,
        },
    };
    setup.stock = StockConfig::FixedCylinder {
        axis: StockAxis::X,
        diameter: ParamLength::from_mm(40.0),
        length: ParamLength::from_mm(80.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
    setup.rotary.axis = RotaryAxisKind::XAxis;
    setup.rotary.clearance_radial_offset = ParamLength::from_mm(12.0);

    // 2D SetupXy pocket window strictly inside the 4-flat wrenching section (X in [-25, -6])
    let mut sketch = SketchLayer::new("sk_flats", "Wrench Flats", setup.id.clone());
    sketch.entities.push(SketchEntityEntry {
        id: "flat_rect".to_string(),
        name: "Flat Window".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(-15.5),
            y: ParamLength::from_mm(0.0),
            width: ParamLength::from_mm(18.0),
            height: ParamLength::from_mm(36.0),
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
        },
    });
    sketch.rebuild_regions(None);
    let sketches = vec![sketch];

    let resolved = setup
        .resolve_with_sketches(&doc, &sketches, DisplayUnit::Millimeter)
        .unwrap();
    let rot = resolved.rotary.unwrap();
    let safe_z = rot.center_wcs.z + rot.clearance_radius_mm;

    let lib = default_starter_tool_library();
    let flat_6mm = lib
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. } if approx_eq(diameter_mm, 6.0, 1e-3)
            )
        })
        .expect("6mm flat endmill in starter library")
        .clone();

    let cancel = AtomicBool::new(false);
    let mut ops = Vec::new();
    let mut tps = Vec::new();

    for (idx, &angle_deg) in [0.0_f64, 90.0, 180.0, 270.0].iter().enumerate() {
        let mut op = Operation::new(
            format!("op_flat_{idx}"),
            format!("Indexed Flat {angle_deg:.0}°"),
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(flat_6mm.id.clone());
        op.selections.push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sk_flats".to_string(),
            sub_layer: None,
            reverse: false,
            side_flip: false,
        });
        op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.5);
        op.heights.bottom = HeightSpec::new(HeightReference::StockTop, -5.0);
        op.passes.multiple_depths = true;
        op.passes.max_stepdown = ParamLength::from_mm(2.0);
        op.rotary = OperationRotaryMode::Indexed3Plus1 { angle_deg };

        let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: &sketches,
            setup: &resolved,
            op: &op,
            tool: &flat_6mm,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: idx == 0,
            upstream_rest: &[],
            carryover_stock: None,
            cancel: &cancel,
        })
        .unwrap();

        // Must emit ClEvent::RotaryIndex with exact target angle
        assert!(
            tp.events.iter().any(|e| matches!(
                e,
                ClEvent::RotaryIndex { angle_deg: a, .. } if (a - angle_deg).abs() < 1e-6
            )),
            "Indexed op at {angle_deg}° must emit ClEvent::RotaryIndex"
        );
        // Must enter and exit at or above rotary clearance radius
        assert!(tp.segments_wcs.first().unwrap().start_wcs.z >= safe_z - 1e-3);
        assert!(tp.segments_wcs.last().unwrap().end_wcs.z >= safe_z - 1e-3);
        // Every segment must carry constant angle_deg
        for seg in &tp.segments_wcs {
            assert!((seg.start_rotary_deg - angle_deg).abs() < 1e-9);
            assert!((seg.end_rotary_deg - angle_deg).abs() < 1e-9);
        }

        ops.push(op);
        tps.push(tp);
    }

    let sim_ops: Vec<SimOperationInput<'_>> = ops
        .iter()
        .zip(tps.iter())
        .map(|(op, tp)| SimOperationInput::from_operation(op, &flat_6mm, tp))
        .collect();

    let sim = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved,
        target_bodies: &[],
        fixtures: &[],
        operations: &sim_ops,
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim.critical_error_count(),
        0,
        "4-sided Indexed 3+1 flat milling must pass CylindricalStock verification crash-free, got: {:#?}",
        sim.diagnostics
    );
    assert!(
        sim.removed_volume_mm3 > 3_500.0,
        "Expected substantial material removed across 4 wrenching flats, got {:.1} mm3",
        sim.removed_volume_mm3
    );
}

#[test]
fn rotary4axis_sim_off_centerline_helical_flutes_on_x_and_y_are_smooth_and_crash_free() {
    use chip_cad::{
        MeshImportOptions, load_mesh_cad_document, sample_helical_flute_shaft_stl_bytes,
    };
    use chip_cam::{
        OperationGenRequest, Rotary4AxisPattern, RotaryAxisKind, RotaryCutDirection,
        RotaryDriveMode, RotaryTiltMode, StockAxis, generate_operation_toolpath_with_rest,
    };
    use chip_sim::{
        MachineKinematicsConfig, SetupSimulationInput, SimOperationInput, SimResolution,
        SimVerificationConfig, run_setup_simulation,
    };

    let stl = sample_helical_flute_shaft_stl_bytes();
    let doc = load_mesh_cad_document(
        "helical_flute_shaft.stl",
        &stl,
        &MeshImportOptions::default(),
    )
    .unwrap();
    assert!(doc.meshes[0].diagnostics.is_watertight);

    let lib = default_starter_tool_library();
    let ball_6mm = lib
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::BallEndmill { diameter_mm, .. } if approx_eq(diameter_mm, 6.0, 1e-3)
            )
        })
        .expect("6mm ball endmill in starter library")
        .clone();

    let cancel = AtomicBool::new(false);

    // 1. Verify Rotary4AxisSim cleanly rejects TwoLinearPlusRotary Setup
    let mut setup_2lin = Setup::new("s_2lin_reject", "2-Linear Only", WorkOffsetSlot::G54);
    setup_2lin.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::VOLUMETRIC_CENTER,
    };
    setup_2lin.stock = StockConfig::FixedCylinder {
        axis: StockAxis::X,
        diameter: ParamLength::from_mm(42.0),
        length: ParamLength::from_mm(80.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup_2lin.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
    setup_2lin.rotary.axis = RotaryAxisKind::XAxis;
    let res_2lin = setup_2lin.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let mut op_spiral = Operation::new(
        "op_4ax_spiral",
        "Simultaneous 4-Axis Spiral",
        OperationKind::Rotary4AxisSim,
    );
    op_spiral.tool_id = Some(ball_6mm.id.clone());
    op_spiral.passes.multiple_depths = true;
    op_spiral.passes.max_stepdown = ParamLength::from_mm(3.0);
    if let StrategyParams::Rotary4AxisSim(p) = op_spiral.strategy_mut() {
        p.pattern = Rotary4AxisPattern::SimultaneousSpiral;
        p.tilt_mode = RotaryTiltMode::SurfaceNormalAutoTilt;
        p.max_tilt_deg = 18.0;
        p.tilt_smoothing_window = 3;
        p.stepover = ParamLength::from_mm(2.0);
        p.axial_margin_start = ParamLength::from_mm(12.0);
        p.axial_margin_end = ParamLength::from_mm(12.0);
    }

    let err = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &res_2lin,
        op: &op_spiral,
        tool: &ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap_err();
    assert!(
        matches!(err, CamError::InvalidRotarySetup { .. }),
        "Expected InvalidRotarySetup for TwoLinearPlusRotary, got: {err:?}"
    );

    // 2. Full Simultaneous 4-Axis X/A (`X+Y+Z+A`) — Spiral + HelicalFlowline
    let mut setup_x = Setup::new("s_4ax_x", "Simultaneous 4-Axis X/A", WorkOffsetSlot::G54);
    setup_x.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::VOLUMETRIC_CENTER,
    };
    setup_x.stock = StockConfig::FixedCylinder {
        axis: StockAxis::X,
        diameter: ParamLength::from_mm(42.0),
        length: ParamLength::from_mm(80.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup_x.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
    setup_x.rotary.axis = RotaryAxisKind::XAxis;
    setup_x.rotary.clearance_radial_offset = ParamLength::from_mm(12.0);
    let resolved_x = setup_x.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let rot_x = resolved_x.rotary.unwrap();

    let tp_spiral = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_x,
        op: &op_spiral,
        tool: &ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    let mut max_abs_y = 0.0_f64;
    let mut max_dy_cut = 0.0_f64;
    for seg in &tp_spiral.segments_wcs {
        max_abs_y = max_abs_y
            .max((seg.start_wcs.y - rot_x.center_wcs.y).abs())
            .max((seg.end_wcs.y - rot_x.center_wcs.y).abs());
        if !matches!(seg.motion, SegmentMotion3D::Rapid) {
            max_dy_cut = max_dy_cut.max((seg.end_wcs.y - seg.start_wcs.y).abs());
        }
    }
    assert!(
        max_abs_y > 1.5,
        "Simultaneous 4-Axis auto-tilt must actively move cross-axis Y off centerline, got max |Y| = {max_abs_y:.3} mm"
    );
    assert!(
        max_dy_cut < 2.5,
        "Smoothed cross-axis Y step per segment must stay smooth, got max |dY| = {max_dy_cut:.3} mm"
    );

    let mut op_flow = Operation::new(
        "op_4ax_flow",
        "Simultaneous 4-Axis Flowline",
        OperationKind::Rotary4AxisSim,
    );
    op_flow.tool_id = Some(ball_6mm.id.clone());
    op_flow.passes.multiple_depths = false;
    if let StrategyParams::Rotary4AxisSim(p) = op_flow.strategy_mut() {
        p.pattern = Rotary4AxisPattern::HelicalFlowline;
        p.cut_direction = RotaryCutDirection::ZigZag;
        p.tilt_mode = RotaryTiltMode::FixedLateralOffset;
        p.lateral_offset = ParamLength::from_mm(2.5);
        p.stepover = ParamLength::from_mm(4.0);
        p.axial_margin_start = ParamLength::from_mm(12.0);
        p.axial_margin_end = ParamLength::from_mm(12.0);
    }
    let tp_flow = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_x,
        op: &op_flow,
        tool: &ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    assert!(tp_flow.segments_wcs.iter().all(|s| approx_eq(
        s.start_wcs.y,
        rot_x.center_wcs.y + 2.5,
        1e-6
    )));

    let sim_x = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved_x,
        target_bodies: &[],
        fixtures: &[],
        operations: &[
            SimOperationInput::from_operation(&op_spiral, &ball_6mm, &tp_spiral),
            SimOperationInput::from_operation(&op_flow, &ball_6mm, &tp_flow),
        ],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim_x.critical_error_count(),
        0,
        "Simultaneous 4-Axis X/A Spiral + Flowline must verify crash-free & gouge-free, got: {:#?}",
        sim_x.diagnostics
    );
    assert!(
        sim_x.removed_volume_mm3 > 2_500.0,
        "Expected substantial helical flute volume removal, got {:.1} mm3",
        sim_x.removed_volume_mm3
    );

    // 3. Simultaneous 4-Axis Y/B (`X+Y+Z+B`) via 90-degree WCS clock
    let mut setup_y = Setup::new("s_4ax_y", "Simultaneous 4-Axis Y/B", WorkOffsetSlot::G54);
    setup_y.wcs.orientation = chip_cam::WcsOrientationMode::ModelOrientation {
        flip_z: false,
        flip_x: false,
        clock_90: 1,
        extra_z_rot_deg: 0.0,
    };
    setup_y.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::VOLUMETRIC_CENTER,
    };
    setup_y.stock = StockConfig::FixedCylinder {
        axis: StockAxis::Y,
        diameter: ParamLength::from_mm(42.0),
        length: ParamLength::from_mm(80.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup_y.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
    setup_y.rotary.axis = RotaryAxisKind::YAxis;
    setup_y.rotary.clearance_radial_offset = ParamLength::from_mm(12.0);
    let resolved_y = setup_y.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let rot_y = resolved_y.rotary.unwrap();

    let tp_y = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_y,
        op: &op_spiral,
        tool: &ball_6mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    let max_abs_x = tp_y
        .segments_wcs
        .iter()
        .map(|s| {
            (s.start_wcs.x - rot_y.center_wcs.x)
                .abs()
                .max((s.end_wcs.x - rot_y.center_wcs.x).abs())
        })
        .fold(0.0_f64, f64::max);
    assert!(
        max_abs_x > 1.5,
        "Simultaneous 4-Axis Y/B must actively move cross-axis X off centerline, got {max_abs_x:.3} mm"
    );
    let sim_y = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved_y,
        target_bodies: &[],
        fixtures: &[],
        operations: &[SimOperationInput::from_operation(
            &op_spiral, &ball_6mm, &tp_y,
        )],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim_y.critical_error_count(),
        0,
        "Simultaneous 4-Axis Y/B must verify crash-free & gouge-free, got: {:#?}",
        sim_y.diagnostics
    );
}

fn push_box_stl_tris(tris: &mut Vec<([f32; 3], [f32; 3], [f32; 3])>, min: [f32; 3], max: [f32; 3]) {
    let [x0, y0, z0] = min;
    let [x1, y1, z1] = max;
    let p000 = [x0, y0, z0];
    let p100 = [x1, y0, z0];
    let p110 = [x1, y1, z0];
    let p010 = [x0, y1, z0];
    let p001 = [x0, y0, z1];
    let p101 = [x1, y0, z1];
    let p111 = [x1, y1, z1];
    let p011 = [x0, y1, z1];
    let quads = [
        (p000, p010, p110, p100), // -Z
        (p001, p101, p111, p011), // +Z
        (p000, p100, p101, p001), // -Y
        (p010, p011, p111, p110), // +Y
        (p000, p001, p011, p010), // -X
        (p100, p110, p111, p101), // +X
    ];
    for (a, b, c, d) in quads {
        tris.push((a, b, c));
        tris.push((a, c, d));
    }
}

fn build_binary_stl(tris: &[([f32; 3], [f32; 3], [f32; 3])]) -> Vec<u8> {
    let mut out = Vec::with_capacity(84 + tris.len() * 50);
    out.extend_from_slice(&[0u8; 80]);
    out.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for &(a, b, c) in tris {
        out.extend_from_slice(&[0u8; 12]); // normal recomputed on load
        for v in [a, b, c] {
            for coord in v {
                out.extend_from_slice(&coord.to_le_bytes());
            }
        }
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    out
}

fn push_cyl_stl_tris(
    tris: &mut Vec<([f32; 3], [f32; 3], [f32; 3])>,
    x0: f32,
    x1: f32,
    cy: f32,
    cz: f32,
    r: f32,
    segs: usize,
) {
    let mut ring0 = Vec::with_capacity(segs);
    let mut ring1 = Vec::with_capacity(segs);
    for i in 0..segs {
        let ang = (i as f32) * std::f32::consts::TAU / (segs as f32);
        let y = cy + r * ang.cos();
        let z = cz + r * ang.sin();
        ring0.push([x0, y, z]);
        ring1.push([x1, y, z]);
    }
    let c0 = [x0, cy, cz];
    let c1 = [x1, cy, cz];
    for i in 0..segs {
        let j = (i + 1) % segs;
        tris.push((ring0[i], ring1[i], ring1[j]));
        tris.push((ring0[i], ring1[j], ring0[j]));
        tris.push((c0, ring0[j], ring0[i]));
        tris.push((c1, ring1[i], ring1[j]));
    }
}

#[test]
fn rotary_4axis_and_2linear_clear_empty_center_without_leaving_stem() {
    use chip_cad::{MeshImportOptions, load_mesh_cad_document};
    use chip_cam::{
        OperationGenRequest, Rotary4AxisPattern, RotaryAxisKind, RotaryCutDirection,
        RotaryDriveMode, RotaryPatternMode, RotaryTiltMode, StockAxis,
        generate_operation_toolpath_with_rest,
    };
    use chip_sim::{
        MachineKinematicsConfig, SetupSimulationInput, SimComputeBackendChoice, SimEngineKind,
        SimOperationInput, SimResolution, SimVerificationConfig, run_setup_simulation,
        run_setup_simulation_with_choice,
    };

    // 1. Star-shaped watertight part: centered cylindrical base (`x ∈ [-10, 0], R = 9 mm`) plus
    // an eccentric lobe (`x ∈ [0, 10], center (0, 4.5), R = 4.5 mm`) tangent to the rotary axis,
    // leaving the entire `θ ∈ (90°, 270°)` half empty at the centerline (`r_target = -1`).
    let mut half_tris = Vec::new();
    push_cyl_stl_tris(&mut half_tris, -10.0, 0.0, 0.0, 0.0, 9.0, 72);
    push_cyl_stl_tris(&mut half_tris, 0.0, 10.0, 0.0, 4.5, 4.5, 72);
    let half_stl = build_binary_stl(&half_tris);
    let doc = load_mesh_cad_document("offset_cyl.stl", &half_stl, &MeshImportOptions::default())
        .expect("load synthetic eccentric-lobe mesh");

    let lib = default_starter_tool_library();
    let flat_3mm = lib
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. } if diameter_mm < 4.0
            )
        })
        .expect("small flat endmill in starter library")
        .clone();
    let mut ball_3mm = flat_3mm.clone();
    ball_3mm.id = chip_tools::ToolId("t_ball_3175".to_string());
    ball_3mm.name = "3.175mm Ball Endmill".to_string();
    ball_3mm.geometry = CutterGeometry::BallEndmill {
        diameter_mm: 3.175,
        flute_length_mm: 12.0,
        overall_length_mm: 38.0,
        shank_diameter_mm: 3.175,
        flutes: 2,
    };
    let cancel = AtomicBool::new(false);

    let mut setup_4ax = Setup::new("s_empty_4ax", "Empty Center 4-Axis", WorkOffsetSlot::G54);
    setup_4ax.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::VOLUMETRIC_CENTER,
    };
    setup_4ax.stock = StockConfig::FixedCylinder {
        axis: StockAxis::X,
        diameter: ParamLength::from_mm(30.0),
        length: ParamLength::from_mm(26.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup_4ax.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
    setup_4ax.rotary.axis = RotaryAxisKind::XAxis;
    setup_4ax.rotary.clearance_radial_offset = ParamLength::from_mm(10.0);
    let resolved_4ax = setup_4ax.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let rot_4ax = resolved_4ax.rotary.unwrap();

    // 1a. Simultaneous 4-Axis (`Rotary4AxisSim`) — Auto-Tilt Spiral (multi-depth)
    let mut op_autotilt = Operation::new(
        "op_empty_autotilt",
        "4-Axis Auto-Tilt Spiral",
        OperationKind::Rotary4AxisSim,
    );
    op_autotilt.tool_id = Some(flat_3mm.id.clone());
    op_autotilt.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
    op_autotilt.passes.multiple_depths = true;
    op_autotilt.passes.max_stepdown = ParamLength::from_mm(5.0);
    if let StrategyParams::Rotary4AxisSim(p) = op_autotilt.strategy_mut() {
        p.pattern = Rotary4AxisPattern::SimultaneousSpiral;
        p.tilt_mode = RotaryTiltMode::SurfaceNormalAutoTilt;
        p.max_tilt_deg = 16.0;
        p.tilt_smoothing_window = 3;
        p.stepover = ParamLength::from_mm(1.5);
        p.angular_sample_deg = 2.0;
    }

    let tp_autotilt = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_4ax,
        op: &op_autotilt,
        tool: &flat_3mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    let min_r_autotilt = tp_autotilt
        .segments_wcs
        .iter()
        .filter(|s| !matches!(s.motion, SegmentMotion3D::Rapid))
        .map(|s| (s.end_wcs.y - rot_4ax.center_wcs.y).hypot(s.end_wcs.z - rot_4ax.center_wcs.z))
        .fold(f64::INFINITY, f64::min);
    assert!(
        min_r_autotilt <= 1e-4,
        "Rotary4AxisSim Auto-Tilt must cut all the way to rotary centerline (r=0), got min_r={min_r_autotilt:.4} mm"
    );

    let sim_autotilt = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved_4ax,
        target_bodies: &[],
        fixtures: &[],
        operations: &[SimOperationInput::from_operation(
            &op_autotilt,
            &flat_3mm,
            &tp_autotilt,
        )],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim_autotilt.critical_error_count(),
        0,
        "Rotary4AxisSim Auto-Tilt empty-center must verify crash-free & gouge-free, got: {:#?}",
        sim_autotilt.diagnostics
    );
    let cyl_auto = sim_autotilt.as_cylindrical().unwrap();
    let mut empty_max_r = 0.0_f32;
    let mut empty_count = 0usize;
    for iu in 0..cyl_auto.nu {
        let u = cyl_auto.u_min + (iu as f64 + 0.5) * cyl_auto.du;
        // Sample inside the half-shaft zone (in WCS the step from full block to half-shaft is at x = 3.0)
        if !(6.0..=11.5).contains(&u) {
            continue;
        }
        for it in 0..cyl_auto.ntheta {
            let theta_deg = (it as f64 * cyl_auto.dtheta).to_degrees();
            if !(135.0..=225.0).contains(&theta_deg) {
                continue;
            }
            let idx = iu * cyl_auto.ntheta + it;
            if cyl_auto.r_target[idx] < 0.0 {
                empty_count += 1;
                empty_max_r = empty_max_r.max(cyl_auto.r_stock[idx]);
            }
        }
    }
    assert!(empty_count > 50, "expected empty-center target cells");
    assert!(
        empty_max_r < 0.05,
        "Rotary4AxisSim Auto-Tilt must not leave a center stem, got max r_stock={empty_max_r:.4} mm"
    );

    // 1b. Simultaneous 4-Axis (`Rotary4AxisSim`) — Fixed Lateral Offset (`2.5 mm`) must taper to center
    let mut op_fixed = Operation::new(
        "op_empty_fixed",
        "4-Axis Fixed Offset Flowline",
        OperationKind::Rotary4AxisSim,
    );
    op_fixed.tool_id = Some(flat_3mm.id.clone());
    op_fixed.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
    op_fixed.passes.multiple_depths = false;
    if let StrategyParams::Rotary4AxisSim(p) = op_fixed.strategy_mut() {
        p.pattern = Rotary4AxisPattern::HelicalFlowline;
        p.cut_direction = RotaryCutDirection::ZigZag;
        p.tilt_mode = RotaryTiltMode::FixedLateralOffset;
        p.lateral_offset = ParamLength::from_mm(2.5);
        p.stepover = ParamLength::from_mm(2.5);
    }
    let tp_fixed = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_4ax,
        op: &op_fixed,
        tool: &flat_3mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    let min_r_fixed = tp_fixed
        .segments_wcs
        .iter()
        .filter(|s| !matches!(s.motion, SegmentMotion3D::Rapid))
        .map(|s| (s.end_wcs.y - rot_4ax.center_wcs.y).hypot(s.end_wcs.z - rot_4ax.center_wcs.z))
        .fold(f64::INFINITY, f64::min);
    assert!(
        min_r_fixed <= 1e-4,
        "Rotary4AxisSim FixedLateralOffset must taper to centerline where no center material exists, got min_r={min_r_fixed:.4} mm"
    );
    let sim_fixed = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved_4ax,
        target_bodies: &[],
        fixtures: &[],
        operations: &[SimOperationInput::from_operation(
            &op_fixed, &flat_3mm, &tp_fixed,
        )],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim_fixed.critical_error_count(),
        0,
        "Rotary4AxisSim FixedLateralOffset empty-center must verify crash-free & gouge-free, got: {:#?}",
        sim_fixed.diagnostics
    );

    // 1c. Explicit positive Bottom Height (`WCS Origin + 3.0 mm`) must still preserve the requested core radius
    let mut op_core = op_autotilt.clone();
    op_core.id = "op_keep_core".to_string();
    op_core.heights.bottom = HeightSpec::new(HeightReference::WcsOrigin, 3.0);
    op_core.passes.multiple_depths = false;
    let tp_core = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_4ax,
        op: &op_core,
        tool: &flat_3mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    let min_z_core = tp_core
        .segments_wcs
        .iter()
        .filter(|s| !matches!(s.motion, SegmentMotion3D::Rapid))
        .map(|s| s.end_wcs.z - rot_4ax.center_wcs.z)
        .fold(f64::INFINITY, f64::min);
    assert!(
        min_z_core >= 3.0 - 1e-4,
        "Explicit positive Bottom Height (+3.0 mm) must be respected, got min_z_rel={min_z_core:.4} mm"
    );

    // 1d. 2-Linear + Rotary (`Rotary2Linear`) must also cut all the way to r = 0 without a stem
    let mut setup_2lin = setup_4ax.clone();
    setup_2lin.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
    let resolved_2lin = setup_2lin.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let rot_2lin = resolved_2lin.rotary.unwrap();

    let mut op_2lin = Operation::new(
        "op_empty_2lin",
        "Rotary 2-Linear Spiral",
        OperationKind::Rotary2Linear,
    );
    op_2lin.tool_id = Some(flat_3mm.id.clone());
    op_2lin.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
    op_2lin.passes.multiple_depths = true;
    op_2lin.passes.max_stepdown = ParamLength::from_mm(5.0);
    if let StrategyParams::Rotary2Linear(p) = op_2lin.strategy_mut() {
        p.pattern = RotaryPatternMode::ContinuousSpiral;
        p.stepover = ParamLength::from_mm(1.5);
    }
    let tp_2lin = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_2lin,
        op: &op_2lin,
        tool: &flat_3mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    let min_r_2lin = tp_2lin
        .segments_wcs
        .iter()
        .filter(|s| !matches!(s.motion, SegmentMotion3D::Rapid))
        .map(|s| s.end_wcs.z - rot_2lin.center_wcs.z)
        .fold(f64::INFINITY, f64::min);
    assert!(
        min_r_2lin <= 1e-6,
        "Rotary2Linear must cut all the way to rotary centerline (r=0), got min_r={min_r_2lin:.4} mm"
    );
    let sim_2lin = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved_2lin,
        target_bodies: &[],
        fixtures: &[],
        operations: &[SimOperationInput::from_operation(
            &op_2lin, &flat_3mm, &tp_2lin,
        )],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    })
    .unwrap();
    assert_eq!(
        sim_2lin.critical_error_count(),
        0,
        "Rotary2Linear empty-center must verify crash-free & gouge-free, got: {:#?}",
        sim_2lin.diagnostics
    );
    let cyl_2lin = sim_2lin.as_cylindrical().unwrap();
    let mut empty_max_r_2lin = 0.0_f32;
    for iu in 0..cyl_2lin.nu {
        let u = cyl_2lin.u_min + (iu as f64 + 0.5) * cyl_2lin.du;
        if !(6.0..=11.5).contains(&u) {
            continue;
        }
        for it in 0..cyl_2lin.ntheta {
            let theta_deg = (it as f64 * cyl_2lin.dtheta).to_degrees();
            if !(135.0..=225.0).contains(&theta_deg) {
                continue;
            }
            let idx = iu * cyl_2lin.ntheta + it;
            if cyl_2lin.r_target[idx] < 0.0 {
                empty_max_r_2lin = empty_max_r_2lin.max(cyl_2lin.r_stock[idx]);
            }
        }
    }
    assert!(
        empty_max_r_2lin < 0.05,
        "Rotary2Linear must not leave a center stem, got max r_stock={empty_max_r_2lin:.4} mm"
    );

    // 2. Two-prong ("bunny ears") hollow-center geometry with a ball endmill + Auto-Tilt smoothing:
    // base block + two symmetrically offset prongs (`|y| ∈ [6, 11]`) leaving a completely open
    // channel down the rotary centerline. Must cut out the center to r = 0 while preserving both
    // outer prongs without gouging on both CPU and GPU cylindrical backends.
    let mut prong_tris = Vec::new();
    push_box_stl_tris(&mut prong_tris, [-10.0, -9.0, -9.0], [0.0, 9.0, 9.0]);
    push_box_stl_tris(&mut prong_tris, [0.0, -11.0, -3.0], [10.0, -6.0, 3.0]);
    push_box_stl_tris(&mut prong_tris, [0.0, 6.0, -3.0], [10.0, 11.0, 3.0]);
    let prong_stl = build_binary_stl(&prong_tris);
    let prong_doc =
        load_mesh_cad_document("two_prongs.stl", &prong_stl, &MeshImportOptions::default())
            .expect("load two-prong mesh");

    let mut op_prongs = op_autotilt.clone();
    op_prongs.id = "op_prongs_ball".to_string();
    op_prongs.tool_id = Some(ball_3mm.id.clone());
    op_prongs.passes.multiple_depths = false;
    if let StrategyParams::Rotary4AxisSim(p) = op_prongs.strategy_mut() {
        p.stepover = ParamLength::from_mm(0.65);
        p.max_tilt_deg = 18.0;
        p.tilt_smoothing_window = 3;
        p.angular_sample_deg = 2.0;
    }

    let mut setup_prongs = setup_4ax.clone();
    let resolved_prongs = setup_prongs
        .resolve(&prong_doc, DisplayUnit::Millimeter)
        .unwrap();
    let rot_prongs = resolved_prongs.rotary.unwrap();
    let tp_prongs = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &prong_doc,
        sketches: &[],
        setup: &resolved_prongs,
        op: &op_prongs,
        tool: &ball_3mm,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    let min_r_prongs = tp_prongs
        .segments_wcs
        .iter()
        .filter(|s| !matches!(s.motion, SegmentMotion3D::Rapid))
        .map(|s| {
            (s.end_wcs.y - rot_prongs.center_wcs.y).hypot(s.end_wcs.z - rot_prongs.center_wcs.z)
        })
        .fold(f64::INFINITY, f64::min);
    assert!(
        min_r_prongs <= 1e-4,
        "Two-prong empty-center model must be cut to r = 0 without leaving a stem, got min_r={min_r_prongs:.4} mm"
    );

    let prong_sim_input = SetupSimulationInput {
        doc: &prong_doc,
        setup: &resolved_prongs,
        target_bodies: &[],
        fixtures: &[],
        operations: &[SimOperationInput::from_operation(
            &op_prongs, &ball_3mm, &tp_prongs,
        )],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
    };

    for backend in [SimComputeBackendChoice::Cpu, SimComputeBackendChoice::Auto] {
        let sim_prongs =
            run_setup_simulation_with_choice(&prong_sim_input, SimEngineKind::Heightmap, backend)
                .unwrap();
        assert_eq!(
            sim_prongs.critical_error_count(),
            0,
            "Two-prong ({backend:?}) toolpath must verify crash-free & gouge-free, got: {:#?}",
            sim_prongs.diagnostics
        );
        let cyl_prongs = sim_prongs.as_cylindrical().unwrap();
        let mut prong_empty_max_r = 0.0_f32;
        let mut prong_outer_min_r = f32::INFINITY;
        for iu in 0..cyl_prongs.nu {
            let u = cyl_prongs.u_min + (iu as f64 + 0.5) * cyl_prongs.du;
            if !(6.0..=11.5).contains(&u) {
                continue;
            }
            for it in 0..cyl_prongs.ntheta {
                let theta_deg = (it as f64 * cyl_prongs.dtheta).to_degrees();
                let idx = iu * cyl_prongs.ntheta + it;
                let in_open_channel =
                    theta_deg <= 20.0 || theta_deg >= 340.0 || (160.0..=200.0).contains(&theta_deg);
                if in_open_channel && cyl_prongs.r_target[idx] < 0.0 {
                    prong_empty_max_r = prong_empty_max_r.max(cyl_prongs.r_stock[idx]);
                }
                let on_prong_crown =
                    (85.0..=95.0).contains(&theta_deg) || (265.0..=275.0).contains(&theta_deg);
                if on_prong_crown && cyl_prongs.r_target[idx] > 10.0 {
                    prong_outer_min_r = prong_outer_min_r.min(cyl_prongs.r_stock[idx]);
                }
            }
        }
        assert!(
            prong_empty_max_r < 0.05,
            "Two-prong ({backend:?}) empty center must not retain a stem, got max r_stock={prong_empty_max_r:.4} mm"
        );
        assert!(
            prong_outer_min_r > 10.5,
            "Two-prong ({backend:?}) outer prongs must remain intact (not erased by center pass), got min r_stock={prong_outer_min_r:.4} mm"
        );
    }
}
