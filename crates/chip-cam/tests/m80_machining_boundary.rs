#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::sync::atomic::AtomicBool;

use chip_cad::{
    CadKernel, MeshImportOptions, MonstertruckKernel, load_mesh_cad_document,
    sample_machined_plate_step,
};
use chip_cam::{
    BoxAnchor27, Deburr3DParams, ExtractedSelectionSummary, GeometrySelectionItem, HeightReference,
    HeightSpec, MachiningBoundaryConfig, MachiningBoundaryMode, MotionRole, Operation,
    OperationGenRequest, OperationKind, ParallelCutDirection, SegmentMotion3D, Setup, StockAxis,
    StockConfig, StrategyParams, SurfaceMesh3D, ToolContainmentMode, Undercut3DParams,
    WcsOriginMode, WorkOffsetSlot, generate_operation_toolpath_with_rest,
    resolve_3d_containment_region_detailed, stock_xy_region,
};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec2, DVec3, DisplayUnit, ParamLength};
use chip_sim::{
    MachineKinematicsConfig, SetupSimulationInput, SimComputeBackendChoice, SimEngineKind,
    SimOperationInput, SimResolution, SimVerificationConfig, run_setup_simulation,
    run_setup_simulation_with_choice,
};
use chip_sketch::{RectAnchorMode, SketchEntity, SketchEntityEntry, SketchLayer};
use chip_tools::{CutterGeometry, default_starter_tool_library, starter_lollipop_6mm};

fn encode_test_stl(tris: &[[DVec3; 3]]) -> Vec<u8> {
    let mut out = Vec::with_capacity(84 + tris.len() * 50);
    out.extend_from_slice(&[0u8; 80]);
    out.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for &[a, b, c] in tris {
        let n = (b - a).cross(c - a).normalize_or_zero();
        for v in [n, a, b, c] {
            out.extend_from_slice(&(v.x as f32).to_le_bytes());
            out.extend_from_slice(&(v.y as f32).to_le_bytes());
            out.extend_from_slice(&(v.z as f32).to_le_bytes());
        }
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    out
}

/// Builds a watertight pure circular hemispherical dome (`R = radius_mm`) seated on a circular
/// cylindrical base (`R = radius_mm`, `Z ∈ [0, base_h]`) with NO square pedestal!
/// Its true 2D XY silhouette is a circle of area `π * R²` (NOT the `4 * R²` square AABB).
fn build_circular_dome_boss_stl(radius_mm: f64, base_h: f64) -> Vec<u8> {
    let n_theta = 72usize;
    let n_phi = 18usize;
    let mut tris: Vec<[DVec3; 3]> = Vec::new();

    // Bottom circular disk (`Z = 0`, normal `-Z`)
    let center_bot = DVec3::new(0.0, 0.0, 0.0);
    for i in 0..n_theta {
        let a0 = std::f64::consts::TAU * (i as f64) / (n_theta as f64);
        let a1 = std::f64::consts::TAU * ((i + 1) as f64) / (n_theta as f64);
        let p0 = DVec3::new(radius_mm * a0.cos(), radius_mm * a0.sin(), 0.0);
        let p1 = DVec3::new(radius_mm * a1.cos(), radius_mm * a1.sin(), 0.0);
        tris.push([center_bot, p1, p0]);

        // Vertical cylindrical wall (`Z ∈ [0, base_h]`)
        let q0 = DVec3::new(p0.x, p0.y, base_h);
        let q1 = DVec3::new(p1.x, p1.y, base_h);
        tris.push([p0, p1, q1]);
        tris.push([p0, q1, q0]);
    }

    // Upper hemispherical dome (`Z ∈ [base_h, base_h + radius_mm]`)
    for ip in 0..n_phi {
        let phi0 = (std::f64::consts::FRAC_PI_2) * (ip as f64) / (n_phi as f64);
        let phi1 = (std::f64::consts::FRAC_PI_2) * ((ip + 1) as f64) / (n_phi as f64);
        let r0 = radius_mm * phi0.cos();
        let z0 = base_h + radius_mm * phi0.sin();
        let r1 = radius_mm * phi1.cos();
        let z1 = base_h + radius_mm * phi1.sin();
        for it in 0..n_theta {
            let a0 = std::f64::consts::TAU * (it as f64) / (n_theta as f64);
            let a1 = std::f64::consts::TAU * ((it + 1) as f64) / (n_theta as f64);
            let p00 = DVec3::new(r0 * a0.cos(), r0 * a0.sin(), z0);
            let p10 = DVec3::new(r0 * a1.cos(), r0 * a1.sin(), z0);
            let p11 = DVec3::new(r1 * a1.cos(), r1 * a1.sin(), z1);
            let p01 = DVec3::new(r1 * a0.cos(), r1 * a0.sin(), z1);
            if ip + 1 == n_phi {
                let apex = DVec3::new(0.0, 0.0, base_h + radius_mm);
                tris.push([p00, p10, apex]);
            } else {
                tris.push([p00, p10, p11]);
                tris.push([p00, p11, p01]);
            }
        }
    }

    encode_test_stl(&tris)
}

/// Builds a watertight overhanging wall mesh spanning `X ∈ [0..40]`, with an overhanging `+Y`
/// bevel (`Y = 18 -> 20` across `Z = 11 -> 16`) beneath a vertical lip (`Y = 20`, `Z = 16..20`).
fn build_overhang_wall_stl() -> Vec<u8> {
    let prof_yz: [(f64, f64); 6] = [
        (0.0, 0.0),
        (18.0, 0.0),
        (18.0, 11.0),
        (20.0, 16.0),
        (20.0, 20.0),
        (0.0, 20.0),
    ];
    let mut tris: Vec<[DVec3; 3]> = Vec::new();
    for i in 0..prof_yz.len() {
        let (ya, za) = prof_yz[i];
        let (yb, zb) = prof_yz[(i + 1) % prof_yz.len()];
        tris.push([
            DVec3::new(0.0, ya, za),
            DVec3::new(0.0, yb, zb),
            DVec3::new(40.0, yb, zb),
        ]);
        tris.push([
            DVec3::new(0.0, ya, za),
            DVec3::new(40.0, yb, zb),
            DVec3::new(40.0, ya, za),
        ]);
    }
    // End caps at X = 0 (-X) and X = 40 (+X)
    let c0 = DVec3::new(0.0, 10.0, 10.0);
    let c1 = DVec3::new(40.0, 10.0, 10.0);
    for i in 0..prof_yz.len() {
        let (ya, za) = prof_yz[i];
        let (yb, zb) = prof_yz[(i + 1) % prof_yz.len()];
        tris.push([c0, DVec3::new(0.0, yb, zb), DVec3::new(0.0, ya, za)]);
        tris.push([c1, DVec3::new(40.0, ya, za), DVec3::new(40.0, yb, zb)]);
    }
    encode_test_stl(&tris)
}

fn make_rect_sketch(
    sketch_id: &str,
    setup_id: &str,
    cx: f64,
    cy: f64,
    w: f64,
    h: f64,
) -> SketchLayer {
    let mut sk = SketchLayer::new(sketch_id, "Boundary Sketch", setup_id);
    sk.entities.push(SketchEntityEntry {
        id: "rect_1".to_string(),
        name: "Boundary Rect".to_string(),
        is_construction: false,
        sub_layer: Some("DEFAULT".to_string()),
        entity: SketchEntity::Rectangle {
            x: ParamLength::from_mm(cx),
            y: ParamLength::from_mm(cy),
            width: ParamLength::from_mm(w),
            height: ParamLength::from_mm(h),
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
            anchor: RectAnchorMode::Center,
        },
    });
    sk.rebuild_regions(None);
    sk
}

#[test]
fn operation_machining_boundary_accessors_cover_facing_and_all_7_3d_strategies() {
    for kind in OperationKind::ALL {
        let mut op = Operation::new("op_test", kind.default_name(), kind);
        let expected_some = matches!(
            kind,
            OperationKind::Facing
                | OperationKind::Waterline3D
                | OperationKind::Adaptive3D
                | OperationKind::Parallel3D
                | OperationKind::Scallop3D
                | OperationKind::Pencil3D
                | OperationKind::Undercut3D
                | OperationKind::Deburr3D
        );
        assert_eq!(
            op.machining_boundary().is_some(),
            expected_some,
            "unexpected machining_boundary() presence for {kind:?}"
        );
        if expected_some {
            let b = op.machining_boundary_mut().unwrap();
            b.boundary_mode = MachiningBoundaryMode::SelectedRegions;
            b.tool_containment = ToolContainmentMode::ToolInsideBoundary;
            b.boundary_offset = ParamLength::from_mm(-1.25);
            let read_back = op.machining_boundary().unwrap();
            assert_eq!(
                *read_back,
                MachiningBoundaryConfig::new(
                    MachiningBoundaryMode::SelectedRegions,
                    ToolContainmentMode::ToolInsideBoundary,
                    -1.25
                )
            );
        }
    }

    // Verify Adaptive3DParams default preserves StockBox + ToolOutsideBoundary + 1.0 mm offset
    let op_a3d = Operation::new("op_a", "Adaptive3D", OperationKind::Adaptive3D);
    assert_eq!(
        *op_a3d.machining_boundary().unwrap(),
        MachiningBoundaryConfig::new(
            MachiningBoundaryMode::StockBox,
            ToolContainmentMode::ToolOutsideBoundary,
            1.0
        )
    );
}

#[test]
fn true_model_silhouette_is_circular_eliminates_corner_air_cuts_and_warns_on_empty_selection() {
    let radius = 20.0_f64;
    let stl = build_circular_dome_boss_stl(radius, 6.0);
    let doc =
        load_mesh_cad_document("circular_dome.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new("s_circ", "Circular Dome Setup", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(4.0),
        side_neg_x: ParamLength::from_mm(4.0),
        side_pos_y: ParamLength::from_mm(4.0),
        side_neg_y: ParamLength::from_mm(4.0),
        top_offset_z: ParamLength::from_mm(0.5),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(20.0),
        round_increment_mm: None,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let surface = SurfaceMesh3D::from_cad_document(&doc, &[], &resolved.frame);
    let empty_sel = ExtractedSelectionSummary::default();

    // 1. True ModelSilhouette area must match π*R² within 2% (NOT the 4*R² square AABB!)
    let res_sil = resolve_3d_containment_region_detailed(
        &surface,
        &resolved,
        &empty_sel,
        MachiningBoundaryMode::ModelSilhouette,
        ToolContainmentMode::ToolCenterOnBoundary,
        0.0,
        3.0,
    );
    assert!(res_sil.warnings.is_empty());
    let expected_area = std::f64::consts::PI * radius * radius;
    let aabb_area = (2.0 * radius) * (2.0 * radius);
    let actual_area = res_sil.nominal_region.total_area_mm2();
    let rel_err = (actual_area - expected_area).abs() / expected_area;
    assert!(
        rel_err < 0.02,
        "ModelSilhouette area ({actual_area:.1} mm²) must match πR² ({expected_area:.1} mm²) within 2% (AABB would be {aabb_area:.1} mm²), rel_err={rel_err:.4}"
    );
    // Empty AABB corners `(±18, ±18)` (radius 25.46 > 20.0) MUST be strictly outside silhouette!
    for corner in [
        DVec2::new(18.0, 18.0),
        DVec2::new(-18.0, 18.0),
        DVec2::new(18.0, -18.0),
        DVec2::new(-18.0, -18.0),
    ] {
        assert!(
            !res_sil.nominal_region.contains_point(corner),
            "empty AABB corner {corner:?} must NOT lie inside true circular ModelSilhouette"
        );
    }

    // 2. Empty SelectedRegions must emit a structured warning while falling back safely
    let res_empty_sel = resolve_3d_containment_region_detailed(
        &surface,
        &resolved,
        &empty_sel,
        MachiningBoundaryMode::SelectedRegions,
        ToolContainmentMode::ToolCenterOnBoundary,
        0.0,
        3.0,
    );
    assert!(
        !res_empty_sel.warnings.is_empty()
            && res_empty_sel.warnings[0].contains("Selected 2D Contours"),
        "expected warning on empty SelectedRegions, got: {:?}",
        res_empty_sel.warnings
    );

    // 3. Parallel3D and Scallop3D under ModelSilhouette must emit ZERO cutting moves in empty AABB corners
    let lib = default_starter_tool_library();
    let mut ball_6 = lib
        .tools
        .iter()
        .find(|t| {
            matches!(t.geometry, CutterGeometry::BallEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5)
        })
        .unwrap()
        .clone();
    ball_6.stickout_length_mm = 32.0;

    let cancel = AtomicBool::new(false);

    let mut op_par = Operation::new("op_par", "Parallel Circular", OperationKind::Parallel3D);
    op_par.tool_id = Some(ball_6.id.clone());
    op_par.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -18.0);
    if let StrategyParams::Parallel3D(p) = op_par.strategy_mut() {
        p.boundary = MachiningBoundaryConfig::new(
            MachiningBoundaryMode::ModelSilhouette,
            ToolContainmentMode::ToolInsideBoundary,
            0.0,
        );
        p.use_cusp_height = false;
        p.stepover = ParamLength::from_mm(2.0);
        p.cut_direction = ParallelCutDirection::ZigZag;
    }
    let tp_par = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_par,
        tool: &ball_6,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    assert!(tp_par.segments_wcs.len() > 20);
    let max_r_inside = radius - 3.0 + 0.35;
    for seg in &tp_par.segments_wcs {
        if !matches!(seg.motion, SegmentMotion3D::Rapid) {
            let r_s = seg.start_wcs.x.hypot(seg.start_wcs.y);
            let r_e = seg.end_wcs.x.hypot(seg.end_wcs.y);
            assert!(
                r_s <= max_r_inside && r_e <= max_r_inside,
                "Parallel3D emitted corner air-cut outside circular ModelSilhouette+ToolInside (max_r={max_r_inside:.2}): r_s={r_s:.2}, r_e={r_e:.2}"
            );
        }
    }

    let mut op_sc = Operation::new("op_sc", "Scallop Circular", OperationKind::Scallop3D);
    op_sc.tool_id = Some(ball_6.id.clone());
    op_sc.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -18.0);
    if let StrategyParams::Scallop3D(p) = op_sc.strategy_mut() {
        p.boundary = MachiningBoundaryConfig::new(
            MachiningBoundaryMode::ModelSilhouette,
            ToolContainmentMode::ToolInsideBoundary,
            0.0,
        );
        p.cusp_height = ParamLength::from_mm(0.15);
        p.max_stepover = ParamLength::from_mm(2.0);
        p.min_stepover = ParamLength::from_mm(0.5);
    }
    let tp_sc = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_sc,
        tool: &ball_6,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    assert!(tp_sc.segments_wcs.len() > 20);
    for seg in &tp_sc.segments_wcs {
        if !matches!(seg.motion, SegmentMotion3D::Rapid) {
            let r_s = seg.start_wcs.x.hypot(seg.start_wcs.y);
            let r_e = seg.end_wcs.x.hypot(seg.end_wcs.y);
            assert!(
                r_s <= max_r_inside + 0.3 && r_e <= max_r_inside + 0.3,
                "Scallop3D emitted corner air-cut outside circular ModelSilhouette+ToolInside: r_s={r_s:.2}, r_e={r_e:.2}"
            );
        }
    }

    // 4. Verify Parallel3D + Scallop3D warning forwarding when SelectedRegions has no selections
    let mut op_warn = op_par.clone();
    op_warn.machining_boundary_mut().unwrap().boundary_mode =
        MachiningBoundaryMode::SelectedRegions;
    let tp_warn = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_warn,
        tool: &ball_6,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    assert!(
        tp_warn
            .warnings
            .iter()
            .any(|w| w.contains("Selected 2D Contours")),
        "expected forwarded SelectedRegions warning in GeneratedToolpath::warnings, got: {:?}",
        tp_warn.warnings
    );

    // 5. Verify zero gouges / zero rapid collisions via chip-sim!
    let sim_ops = vec![
        SimOperationInput::from_operation(&op_par, &ball_6, &tp_par),
        SimOperationInput::from_operation(&op_sc, &ball_6, &tp_sc),
    ];
    let sim_res = run_setup_simulation(&SetupSimulationInput {
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
        sim_res.critical_error_count(),
        0,
        "Circular dome Parallel3D + Scallop3D must have 0 rapid/gouge errors, got: {:#?}",
        sim_res.diagnostics
    );
}

#[test]
fn adaptive3d_honors_stockbox_containment_negative_offset_and_cylindrical_z_stock() {
    let radius = 16.0_f64;
    let stl = build_circular_dome_boss_stl(radius, 4.0);
    let doc = load_mesh_cad_document("boss_cyl.stl", &stl, &MeshImportOptions::default()).unwrap();

    let lib = default_starter_tool_library();
    let mut flat_6 = lib
        .tools
        .iter()
        .find(|t| {
            matches!(t.geometry, CutterGeometry::FlatEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5)
        })
        .unwrap()
        .clone();
    flat_6.stickout_length_mm = 30.0;
    let tool_r = 3.0_f64;
    let cancel = AtomicBool::new(false);

    // === Part A: Box stock — verify ToolInside / ToolCenter / ToolOutside and negative offset ===
    let mut box_setup = Setup::new("s_box", "Box Stock", WorkOffsetSlot::G54);
    box_setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    box_setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(8.0),
        side_neg_x: ParamLength::from_mm(8.0),
        side_pos_y: ParamLength::from_mm(8.0),
        side_neg_y: ParamLength::from_mm(8.0),
        top_offset_z: ParamLength::from_mm(0.5),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(20.0),
        round_increment_mm: None,
    };
    let resolved_box = box_setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let half_stock = 24.0_f64; // radius 16 + 8 mm margin

    let run_a3d = |containment: ToolContainmentMode, offset_mm: f64| {
        let mut op = Operation::new("op_a3d", "Adaptive3D", OperationKind::Adaptive3D);
        op.tool_id = Some(flat_6.id.clone());
        op.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -12.0);
        op.passes.stock_to_leave_radial = ParamLength::from_mm(0.3);
        op.passes.stock_to_leave_axial = ParamLength::from_mm(0.2);
        if let StrategyParams::Adaptive3D(p) = op.strategy_mut() {
            p.boundary = MachiningBoundaryConfig::new(
                MachiningBoundaryMode::StockBox,
                containment,
                offset_mm,
            );
            p.max_roughing_stepdown = ParamLength::from_mm(6.0);
            p.fine_stepdown = ParamLength::from_mm(3.0);
            p.adaptive.optimal_load = ParamLength::from_mm(2.0);
        }
        let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: &[],
            setup: &resolved_box,
            op: &op,
            tool: &flat_6,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            cancel: &cancel,
        })
        .unwrap();
        let max_xy = tp
            .segments_wcs
            .iter()
            .filter(|s| !matches!(s.motion, SegmentMotion3D::Rapid))
            .map(|s| {
                s.start_wcs
                    .x
                    .abs()
                    .max(s.start_wcs.y.abs())
                    .max(s.end_wcs.x.abs())
                    .max(s.end_wcs.y.abs())
            })
            .fold(0.0_f64, f64::max);
        (op, tp, max_xy)
    };

    let (_op_in, _tp_in, max_in) = run_a3d(ToolContainmentMode::ToolInsideBoundary, 0.0);
    let (_op_ctr, _tp_ctr, max_ctr) = run_a3d(ToolContainmentMode::ToolCenterOnBoundary, 0.0);
    let (_op_out0, _tp_out0, max_out0) = run_a3d(ToolContainmentMode::ToolOutsideBoundary, 0.0);
    let (_op_neg, _tp_neg, max_neg) = run_a3d(ToolContainmentMode::ToolCenterOnBoundary, -2.5);

    // ToolInsideBoundary -> max_xy <= half_stock - tool_r = 21.0
    assert!(
        max_in <= half_stock - tool_r + 0.15,
        "Adaptive3D StockBox+ToolInside must stay <= {:.2}, got {max_in:.2}",
        half_stock - tool_r
    );
    // Negative offset (-2.5 with ToolCenter) -> max_xy <= half_stock - 2.5 = 21.5 (NOT clamped to +1.0!)
    assert!(
        max_neg <= half_stock - 2.5 + 0.15,
        "Adaptive3D StockBox negative offset (-2.5 mm) must NOT clamp to +1.0: expected <= {:.2}, got {max_neg:.2}",
        half_stock - 2.5
    );
    // ToolCenterOnBoundary + 0.0 -> max_xy <= half_stock = 24.0
    assert!(
        max_ctr <= half_stock + 0.15 && max_ctr > max_in + 1.5,
        "Adaptive3D StockBox+ToolCenter expected in ({:.2}, {:.2}], got {max_ctr:.2}",
        max_in + 1.5,
        half_stock
    );
    // ToolOutsideBoundary + 0.0 -> max_xy <= half_stock + tool_r = 27.0 (NOT forced to +1.0 = 28.0!)
    assert!(
        max_out0 <= half_stock + tool_r + 0.15 && max_out0 > max_ctr + 1.5,
        "Adaptive3D StockBox+ToolOutside (offset=0.0) expected in ({:.2}, {:.2}], got {max_out0:.2}",
        max_ctr + 1.5,
        half_stock + tool_r
    );

    // === Part B: Cylindrical Z-axis stock — true circular XY footprint + chip-sim verification ===
    let cyl_stock_r = 22.0_f64;
    let mut cyl_setup = Setup::new("s_cyl_z", "Cylindrical Z Stock", WorkOffsetSlot::G54);
    cyl_setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    cyl_setup.stock = StockConfig::FixedCylinder {
        axis: StockAxis::Z,
        diameter: ParamLength::from_mm(2.0 * cyl_stock_r),
        length: ParamLength::from_mm(21.0),
        axial_top_offset: ParamLength::from_mm(0.5),
    };
    let resolved_cyl = cyl_setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let cyl_reg = stock_xy_region(&resolved_cyl);
    let expected_cyl_area = std::f64::consts::PI * cyl_stock_r * cyl_stock_r;
    assert!(
        (cyl_reg.total_area_mm2() - expected_cyl_area).abs() / expected_cyl_area < 0.01,
        "stock_xy_region for Z-axis cylinder must be exact circle (expected {expected_cyl_area:.1} mm², got {:.1} mm²)",
        cyl_reg.total_area_mm2()
    );

    let mut op_cyl = Operation::new("op_a3d_cyl", "Adaptive3D Cyl Z", OperationKind::Adaptive3D);
    op_cyl.tool_id = Some(flat_6.id.clone());
    op_cyl.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -14.0);
    op_cyl.passes.stock_to_leave_radial = ParamLength::from_mm(0.3);
    op_cyl.passes.stock_to_leave_axial = ParamLength::from_mm(0.2);
    if let StrategyParams::Adaptive3D(p) = op_cyl.strategy_mut() {
        p.boundary = MachiningBoundaryConfig::new(
            MachiningBoundaryMode::StockBox,
            ToolContainmentMode::ToolOutsideBoundary,
            1.0,
        );
        p.max_roughing_stepdown = ParamLength::from_mm(7.0);
        p.fine_stepdown = ParamLength::from_mm(3.5);
        p.adaptive.optimal_load = ParamLength::from_mm(2.0);
    }
    let tp_cyl = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_cyl,
        op: &op_cyl,
        tool: &flat_6,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    assert!(tp_cyl.segments_wcs.len() > 20);
    // Max radial distance must stay within `cyl_stock_r + tool_r + 1.0 = 26.0 mm` (NOT `26 * sqrt(2) = 36.8 mm`!)
    let max_cyl_r = cyl_stock_r + tool_r + 1.0 + 0.25;
    for seg in &tp_cyl.segments_wcs {
        if !matches!(seg.motion, SegmentMotion3D::Rapid) {
            let r_e = seg.end_wcs.x.hypot(seg.end_wcs.y);
            assert!(
                r_e <= max_cyl_r,
                "Adaptive3D on cylindrical Z-stock escaped circular boundary (max_r={max_cyl_r:.2}): r={r_e:.2} at {:?}",
                seg.end_wcs
            );
        }
    }

    // Verify cylindrical Z-stock Adaptive3D in chip-sim (zero rapid collisions, zero gouges)!
    let sim_ops = vec![SimOperationInput::from_operation(&op_cyl, &flat_6, &tp_cyl)];
    let sim_res = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved_cyl,
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
        "Cylindrical Z-stock Adaptive3D must have 0 critical errors, got: {:#?}",
        sim_res.diagnostics
    );
}

#[test]
fn undercut3d_respects_selected_regions_containment_offset_and_passes_sim() {
    let stl = build_overhang_wall_stl();
    let doc =
        load_mesh_cad_document("overhang_u3d.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new("s_u3d_m80", "M80 Undercut3D", WorkOffsetSlot::G54);
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
    let lolli_6 = starter_lollipop_6mm(); // R = 3.0 mm
    let cancel = AtomicBool::new(false);

    // 2D boundary window centered at X=20, width=16 (`X ∈ [12, 28]`), Y ∈ [15, 29]
    let sk = make_rect_sketch("sk_u3d", "s_u3d_m80", 20.0, 22.0, 16.0, 14.0);

    let run_u3d = |containment: ToolContainmentMode, offset_mm: f64| {
        let mut op = Operation::new("op_u3d_sel", "Undercut Selected", OperationKind::Undercut3D);
        op.tool_id = Some(lolli_6.id.clone());
        op.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
        op.selections.push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sk_u3d".to_string(),
            sub_layer: None,
            reverse: false,
            side_flip: false,
        });
        op.strategy = Some(StrategyParams::Undercut3D(Undercut3DParams {
            boundary: MachiningBoundaryConfig::new(
                MachiningBoundaryMode::SelectedRegions,
                containment,
                offset_mm,
            ),
            z_stepdown: ParamLength::from_mm(0.5),
            ..Undercut3DParams::default()
        }));
        let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk),
            setup: &resolved,
            op: &op,
            tool: &lolli_6,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            cancel: &cancel,
        })
        .unwrap();
        let (mut min_x, mut max_x) = (f64::INFINITY, f64::NEG_INFINITY);
        for seg in &tp.segments_wcs {
            if matches!(
                seg.motion,
                SegmentMotion3D::Linear {
                    role: MotionRole::FinishCut | MotionRole::Cut,
                    ..
                }
            ) {
                min_x = min_x.min(seg.start_wcs.x).min(seg.end_wcs.x);
                max_x = max_x.max(seg.start_wcs.x).max(seg.end_wcs.x);
            }
        }
        (op, tp, min_x, max_x)
    };

    // ToolInsideBoundary (R=3): X ∈ [15, 25]
    let (op_in, tp_in, min_in, max_in) = run_u3d(ToolContainmentMode::ToolInsideBoundary, 0.0);
    assert!(
        min_in >= 14.9 && max_in <= 25.1 && max_in - min_in > 5.0,
        "Undercut3D ToolInsideBoundary expected X in [15, 25], got [{min_in:.2}, {max_in:.2}]"
    );

    // ToolOutsideBoundary (R=3, offset=+1.0): X ∈ [8, 32]
    let (_op_out, _tp_out, min_out, max_out) =
        run_u3d(ToolContainmentMode::ToolOutsideBoundary, 1.0);
    assert!(
        min_out >= 7.9 && max_out <= 32.1 && min_out < 10.0 && max_out > 30.0,
        "Undercut3D ToolOutsideBoundary + 1.0 expected X in [8, 32], got [{min_out:.2}, {max_out:.2}]"
    );

    // Verify TriDexel simulation has zero crashes / zero gouges / zero neck rubs
    let sim_ops = vec![SimOperationInput::from_operation(&op_in, &lolli_6, &tp_in)];
    let sim_res = run_setup_simulation_with_choice(
        &SetupSimulationInput {
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
        },
        SimEngineKind::TriDexel,
        SimComputeBackendChoice::Cpu,
    )
    .unwrap();
    assert_eq!(
        sim_res.critical_error_count(),
        0,
        "Undercut3D SelectedRegions must have 0 critical errors in TriDexel sim, got: {:#?}",
        sim_res.diagnostics
    );
}

#[test]
fn deburr3d_auto_detect_respects_selected_regions_and_passes_sim() {
    let step = sample_machined_plate_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("plate.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();

    let mut setup = Setup::new("s_deb_m80", "M80 Deburr3D", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let model_aabb = resolved.model_aabb_wcs;

    let lib = default_starter_tool_library();
    let chamfer_90 = lib
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
        .unwrap();
    let cancel = AtomicBool::new(false);

    // 1. Baseline: auto_detect_edges + default ModelSilhouette covers full X span
    let mut op_full = Operation::new("op_deb_full", "Deburr Full", OperationKind::Deburr3D);
    op_full.tool_id = Some(chamfer_90.id.clone());
    op_full.strategy = Some(StrategyParams::Deburr3D(Deburr3DParams {
        auto_detect_edges: true,
        ..Deburr3DParams::default()
    }));
    let tp_full = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_full,
        tool: chamfer_90,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    let full_max_x = tp_full
        .segments_wcs
        .iter()
        .filter(|s| {
            matches!(
                s.motion,
                SegmentMotion3D::Linear {
                    role: MotionRole::Cut | MotionRole::FinishCut,
                    ..
                }
            )
        })
        .map(|s| s.start_wcs.x.max(s.end_wcs.x))
        .fold(f64::NEG_INFINITY, f64::max);
    let mid_x = 0.5 * (model_aabb.min.x + model_aabb.max.x);
    assert!(
        full_max_x > mid_x + 10.0,
        "Full Deburr3D should cover past mid_x ({mid_x:.1}), got max_x={full_max_x:.1}"
    );

    // 2. SelectedRegions covering only the left half (`X ∈ [-10, mid_x]`)
    let left_w = (mid_x - (-10.0)).max(10.0);
    let left_cx = -10.0 + 0.5 * left_w;
    let sk_left = make_rect_sketch(
        "sk_deb_left",
        "s_deb_m80",
        left_cx,
        0.5 * (model_aabb.min.y + model_aabb.max.y),
        left_w,
        model_aabb.size().y + 20.0,
    );

    let mut op_left = Operation::new("op_deb_left", "Deburr Left Half", OperationKind::Deburr3D);
    op_left.tool_id = Some(chamfer_90.id.clone());
    op_left
        .selections
        .push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sk_deb_left".to_string(),
            sub_layer: None,
            reverse: false,
            side_flip: false,
        });
    op_left.strategy = Some(StrategyParams::Deburr3D(Deburr3DParams {
        boundary: MachiningBoundaryConfig::new(
            MachiningBoundaryMode::SelectedRegions,
            ToolContainmentMode::ToolCenterOnBoundary,
            0.0,
        ),
        auto_detect_edges: true,
        ..Deburr3DParams::default()
    }));

    let tp_left = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: std::slice::from_ref(&sk_left),
        setup: &resolved,
        op: &op_left,
        tool: chamfer_90,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    let mut left_cut_count = 0usize;
    for seg in &tp_left.segments_wcs {
        if matches!(
            seg.motion,
            SegmentMotion3D::Linear {
                role: MotionRole::Cut | MotionRole::FinishCut,
                ..
            }
        ) {
            left_cut_count += 1;
            assert!(
                seg.start_wcs.x <= mid_x + 0.1 && seg.end_wcs.x <= mid_x + 0.1,
                "Deburr3D SelectedRegions escaped X <= {mid_x:.2}: {:?}",
                seg
            );
        }
    }
    assert!(
        left_cut_count > 10,
        "Deburr3D SelectedRegions should emit cutting moves on left half, got {left_cut_count}"
    );

    // 3. Verify zero rapid collisions and zero target B-Rep gouges in chip-sim!
    let sim_ops = vec![SimOperationInput::from_operation(
        &op_left, chamfer_90, &tp_left,
    )];
    let sim_res = run_setup_simulation(&SetupSimulationInput {
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
        sim_res.critical_error_count(),
        0,
        "Deburr3D SelectedRegions must have 0 critical errors in sim, got: {:#?}",
        sim_res.diagnostics
    );
}
