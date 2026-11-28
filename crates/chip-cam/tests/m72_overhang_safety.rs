#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::sync::atomic::AtomicBool;

use chip_cad::{
    CadKernel, MeshImportOptions, MonstertruckKernel, SurfaceKind, find_same_diameter_holes,
    load_mesh_cad_document,
};
use chip_cam::{
    AxisAnchor, BoxAnchor27, ChamferSourceMode, ChamferTraceMode, ContourCompensation,
    GeometrySelectionItem, HeightReference, HeightSpec, HoleMachiningMode, MachiningBoundaryMode,
    Operation, OperationGenRequest, OperationKind, PocketStrategyKind, PushCutterField, Setup,
    StockConfig, StrategyParams, SurfaceCutterProfile, SurfaceMesh3D, ToolAssemblyPart,
    ToolContainmentMode, WcsOriginMode, WorkOffsetSlot, drop_cutter_z,
    drop_cutter_z_detailed_with_scratch, generate_operation_toolpath_with_rest,
};
use chip_fixtures::sample_multi_feature_bracket_step;
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec2, DVec3, DisplayUnit, ParamLength};
use chip_sim::{
    MachineKinematicsConfig, SetupSimulationInput, SimComputeBackendChoice, SimEngineKind,
    SimIssueKind, SimOperationInput, SimResolution, SimVerificationConfig, run_setup_simulation,
};
use chip_sketch::{
    RectAnchorMode, SketchAnchor, SketchEntity, SketchEntityEntry, SketchLayer, SketchPlaneRef,
};
use chip_tools::{
    CoolantMode, CutterGeometry, CuttingPreset, SpindleDirection, SurfaceSpeedPresetSpec, Tool,
    ToolId, default_starter_tool_library,
};

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

fn push_box_tris(tris: &mut Vec<[DVec3; 3]>, x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) {
    let v = [
        DVec3::new(x0, y0, z0),
        DVec3::new(x1, y0, z0),
        DVec3::new(x1, y1, z0),
        DVec3::new(x0, y1, z0),
        DVec3::new(x0, y0, z1),
        DVec3::new(x1, y0, z1),
        DVec3::new(x1, y1, z1),
        DVec3::new(x0, y1, z1),
    ];
    let quads = [
        [0, 3, 2, 1], // bottom (-Z)
        [4, 5, 6, 7], // top (+Z)
        [0, 1, 5, 4], // front (-Y)
        [2, 3, 7, 6], // back (+Y)
        [0, 4, 7, 3], // left (-X)
        [1, 2, 6, 5], // right (+X)
    ];
    for [a, b, c, d] in quads {
        tris.push([v[a], v[b], v[c]]);
        tris.push([v[a], v[c], v[d]]);
    }
}

fn make_reduced_flute_flat_tool(
    id: &str,
    tool_number: u32,
    cutting_d: f64,
    flute_len: f64,
    shank_d: f64,
    stickout: f64,
) -> Tool {
    Tool {
        id: ToolId(id.to_string()),
        tool_number,
        length_offset_number: tool_number,
        diameter_offset_number: tool_number,
        name: format!("Ø{cutting_d:.0}mm / Ø{shank_d:.0}mm Shank Flat Endmill"),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::AirBlast,
        stickout_length_mm: stickout,
        geometry: CutterGeometry::FlatEndmill {
            diameter_mm: cutting_d,
            flute_length_mm: flute_len,
            overall_length_mm: (stickout + 25.0).max(50.0),
            shank_diameter_mm: shank_d,
            flutes: 3,
        },
        presets: vec![CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
            id: "al6061_m72",
            name: "Aluminum 6061 - M72",
            material: "Aluminum 6061",
            diameter_mm: cutting_d,
            flutes: 3,
            surface_speed_m_min: 180.0,
            chipload_mm_tooth: 0.025,
            default_stepdown_mm: 2.0,
            default_stepover_mm: 1.5,
            coolant_override: Some(CoolantMode::AirBlast),
        })],
    }
}

fn make_twist_drill(id: &str, tool_number: u32, diameter_mm: f64, stickout: f64) -> Tool {
    Tool {
        id: ToolId(id.to_string()),
        tool_number,
        length_offset_number: tool_number,
        diameter_offset_number: tool_number,
        name: format!("Ø{diameter_mm:.0}mm Carbide Twist Drill"),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::AirBlast,
        stickout_length_mm: stickout,
        geometry: CutterGeometry::TwistDrill {
            diameter_mm,
            point_angle_deg: 118.0,
            flute_length_mm: 24.0,
            overall_length_mm: 60.0,
            shank_diameter_mm: diameter_mm,
            flutes: 2,
        },
        presets: vec![CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
            id: "al6061_drill",
            name: "Aluminum 6061 - Drill",
            material: "Aluminum 6061",
            diameter_mm,
            flutes: 2,
            surface_speed_m_min: 90.0,
            chipload_mm_tooth: 0.05,
            default_stepdown_mm: 3.0,
            default_stepover_mm: 0.0,
            coolant_override: Some(CoolantMode::AirBlast),
        })],
    }
}

fn build_overhanging_t_flange_stl(roof_top_z: f64) -> Vec<u8> {
    let mut tris = Vec::new();
    // Base pedestal `[0..2]`, central stem `[-4..4] x [-4..4] x [2..10]`,
    // wide overhanging roof `[-12..12] x [-8..8] x [10..roof_top_z]`
    push_box_tris(&mut tris, -22.0, -16.0, 0.0, 22.0, 16.0, 2.0);
    push_box_tris(&mut tris, -4.0, -4.0, 2.0, 4.0, 4.0, 10.0);
    push_box_tris(&mut tris, -12.0, -8.0, 10.0, 12.0, 8.0, roof_top_z);
    encode_test_stl(&tris)
}

fn build_stepped_overhanging_bridge_stl() -> Vec<u8> {
    let mut tris = Vec::new();
    // Lower base slab (`Z = 0..4`): open shelf floor at Z = 4.0
    push_box_tris(&mut tris, -24.0, -18.0, 0.0, 24.0, 18.0, 4.0);
    // Left and right support piers (`Z = 4..20`)
    push_box_tris(&mut tris, -24.0, -18.0, 4.0, -18.0, 18.0, 20.0);
    push_box_tris(&mut tris, 18.0, -18.0, 4.0, 24.0, 18.0, 20.0);
    // Overhanging bridge spanning across the center (`Y ∈ [-5..5]`, `Z ∈ [14..20]`)
    push_box_tris(&mut tris, -18.0, -5.0, 14.0, 18.0, 5.0, 20.0);
    encode_test_stl(&tris)
}

#[test]
fn drop_cutter_and_push_cutter_enforce_exact_shank_and_holder_tiers() {
    let mut tris = Vec::new();
    push_box_tris(&mut tris, -4.0, -4.0, 0.0, 4.0, 4.0, 10.0);
    push_box_tris(&mut tris, -12.0, -8.0, 10.0, 12.0, 8.0, 20.0);
    let surface = SurfaceMesh3D::from_raw_triangles(&tris);

    // Reduced-flute wider-shank cutter: cutting_d=4 (r=2), flute_length=8, shank_d=8 (r=4), stickout=18
    // holder_nut_r = max(4 * 2.2, 12.0, 4 + 4) = 12.0, holder_clearance = 0.5 -> r_nut_eff = 12.5
    let tool = make_reduced_flute_flat_tool("t_m72", 1, 4.0, 8.0, 8.0, 18.0);
    let prof_asm = SurfaceCutterProfile::from_tool(&tool, 0.0, 0.0, 0.5);
    let prof_flute = SurfaceCutterProfile::from_tool_geometry(&tool.geometry, 0.0, 0.0);

    // 1. At x = 15.0 (3.0 mm outside roof edge x=12): outside flute (r=2), INSIDE wider shank (r=4)!
    // Flute-only misses completely (None); full-assembly lifts to `20.0 - 8.0 = 12.0` (Shank)!
    assert!(drop_cutter_z(&surface, &prof_flute, 15.0, 0.0, 0.0).is_none());
    let mut scratch = Vec::new();
    let det_shank =
        drop_cutter_z_detailed_with_scratch(&surface, &prof_asm, 15.0, 0.0, 0.0, &mut scratch)
            .unwrap();
    assert_eq!(det_shank.limiting_part, ToolAssemblyPart::Shank);
    assert!(
        (det_shank.z_tip_wcs - 12.0).abs() < 1e-6,
        "Expected wider shank lift z_tip=12.0, got {}",
        det_shank.z_tip_wcs
    );

    // 2. At x = 19.0 (7.0 mm outside roof edge x=12): outside shank (r=4), INSIDE collet nut (r_eff=12.5)!
    // Collet nut starts at stickout=18.0 with 0.5 mm axial clearance -> `z_tip = 20.0 - 18.0 + 0.5 = 2.5`!
    let det_holder =
        drop_cutter_z_detailed_with_scratch(&surface, &prof_asm, 19.0, 0.0, 0.0, &mut scratch)
            .unwrap();
    assert_eq!(det_holder.limiting_part, ToolAssemblyPart::Holder);
    assert!(
        (det_holder.z_tip_wcs - 2.5).abs() < 1e-6,
        "Expected collet nut lift z_tip=2.5, got {}",
        det_holder.z_tip_wcs
    );

    // 3. Fast path parity: standard tool (`shank_d == cutting_d`) with stickout >= part height
    // produces bit-identical `drop_cutter_z` between `from_tool` and `from_tool_geometry`!
    let std_tool = make_reduced_flute_flat_tool("t_std", 2, 6.0, 18.0, 6.0, 30.0);
    let std_asm = SurfaceCutterProfile::from_tool(&std_tool, 0.2, 0.1, 0.5);
    let std_flute = SurfaceCutterProfile::from_tool_geometry(&std_tool.geometry, 0.2, 0.1);
    for &(x, y) in &[(0.0, 0.0), (10.0, 4.0), (14.5, 0.0), (18.0, 0.0)] {
        assert_eq!(
            drop_cutter_z(&surface, &std_asm, x, y, 0.0),
            drop_cutter_z(&surface, &std_flute, x, y, 0.0),
            "Standard tool with adequate stickout must take fast path identically at ({x}, {y})"
        );
    }

    // 4. PushCutterField monotonicity & full-assembly avoidance regions
    let field = PushCutterField::build_with_z_min(&surface, prof_asm, 0.0);
    let avoid_hi = field.avoidance_region_z(13.0);
    let avoid_mid = field.avoidance_region_z(6.0);
    let avoid_lo = field.avoidance_region_z(1.0);

    // At z=13.0, shank starts at z=21.0 > 20.0 -> x=15.0 is CLEAR!
    assert!(!avoid_hi.contains_point(DVec2::new(15.0, 0.0)));
    // At z=6.0, shank spans [14..24] -> x=15.0 is AVOIDED (shank shadow), x=19.0 is CLEAR!
    assert!(avoid_mid.contains_point(DVec2::new(15.0, 0.0)));
    assert!(!avoid_mid.contains_point(DVec2::new(19.0, 0.0)));
    // At z=1.0, collet nut starts at 19.0 < 20.0 -> BOTH x=15.0 and x=19.0 are AVOIDED!
    assert!(avoid_lo.contains_point(DVec2::new(15.0, 0.0)));
    assert!(avoid_lo.contains_point(DVec2::new(19.0, 0.0)));

    // Strict monotonicity across descending Z levels
    assert!(avoid_hi.difference(&avoid_mid).total_area_mm2() <= 1e-3);
    assert!(avoid_mid.difference(&avoid_lo).total_area_mm2() <= 1e-3);
}

#[test]
fn adaptive3d_and_waterline3d_wider_shank_overhang_pass_tridexel_zero_collisions() {
    let stl = build_overhanging_t_flange_stl(16.0);
    let doc = load_mesh_cad_document("t_flange.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new("setup_tflange", "T-Flange Setup", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27 {
            x: AxisAnchor::Center,
            y: AxisAnchor::Center,
            z: AxisAnchor::Min,
        },
    };
    setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(4.0),
        side_neg_x: ParamLength::from_mm(4.0),
        side_pos_y: ParamLength::from_mm(4.0),
        side_neg_y: ParamLength::from_mm(4.0),
        top_offset_z: ParamLength::from_mm(0.5),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(16.5),
        round_increment_mm: None,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    // Reduced-flute + wider-shank cutter: cutting_d=4, flute_length=8, shank_d=8, stickout=18
    let tool = make_reduced_flute_flat_tool("t_reduced", 1, 4.0, 8.0, 8.0, 18.0);
    let cancel = AtomicBool::new(false);

    // Op 1: Adaptive3D roughing down to pedestal top Z = 2.0 (stepdown = 4.0 <= flute_length,
    // fine_stepdown = 2.0 so upper stock down to Z = 8.0 clears cleanly before shank steps out!)
    let mut op_adap = Operation::new("op_adap3d", "Adaptive 3D Rough", OperationKind::Adaptive3D);
    op_adap.tool_id = Some(tool.id.clone());
    op_adap.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 2.0);
    op_adap.passes.stock_to_leave_radial = ParamLength::from_mm(0.2);
    op_adap.passes.stock_to_leave_axial = ParamLength::from_mm(0.1);
    if let StrategyParams::Adaptive3D(p) = op_adap.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::StockBox;
        p.max_roughing_stepdown = ParamLength::from_mm(4.0);
        p.fine_stepdown = ParamLength::from_mm(2.0);
        p.adaptive.optimal_load = ParamLength::from_mm(1.5);
        p.detect_flats = true;
    }

    let tp_adap = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_adap,
        tool: &tool,
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

    assert!(
        tp_adap
            .warnings
            .iter()
            .any(|w| w.contains("Reach-limit") && w.contains("shank")),
        "Adaptive3D must emit a reach-limit warning for wider-shank overhang clearance, got: {:?}",
        tp_adap.warnings
    );

    // Op 2: Waterline3D finishing down to Z = 2.5
    let mut op_wl = Operation::new("op_wl3d", "Waterline 3D Finish", OperationKind::Waterline3D);
    op_wl.tool_id = Some(tool.id.clone());
    op_wl.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 2.5);
    op_wl.passes.max_stepdown = ParamLength::from_mm(2.5);
    if let StrategyParams::Waterline3D(p) = op_wl.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::StockBox;
        p.boundary.tool_containment = ToolContainmentMode::ToolOutsideBoundary;
        p.adaptive_stepdown = false;
        p.detect_flats = true;
    }

    let tp_wl = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_wl,
        tool: &tool,
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

    assert!(
        tp_wl
            .warnings
            .iter()
            .any(|w| w.contains("Reach-limit") && w.contains("shank")),
        "Waterline3D must emit a reach-limit warning for wider-shank overhang clearance, got: {:?}",
        tp_wl.warnings
    );

    // Verify both operations together against `chip-sim` volumetric Tri-Dexel oracle!
    let sim_res = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved,
        target_bodies: &[],
        fixtures: &[],
        operations: &[
            SimOperationInput::from_operation(&op_adap, &tool, &tp_adap),
            SimOperationInput::from_operation(&op_wl, &tool, &tp_wl),
        ],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: Some(&cancel),
        engine: SimEngineKind::TriDexel,
        backend_choice: SimComputeBackendChoice::Auto,
        ..SetupSimulationInput::new(&doc, &resolved, &[])
    })
    .unwrap();

    let bad: Vec<_> = sim_res
        .diagnostics
        .iter()
        .filter(|d| {
            matches!(
                d.kind,
                SimIssueKind::FluteLengthExceeded
                    | SimIssueKind::HolderCollision
                    | SimIssueKind::TargetModelGouge
                    | SimIssueKind::RapidIntoStock
            )
        })
        .collect();
    assert!(
        bad.is_empty(),
        "Adaptive3D + Waterline3D with wider-shank avoidance must achieve 0 ShankRub, 0 HolderCollision, 0 TargetModelGouge, 0 RapidIntoStock; got: {bad:#?}"
    );
}

#[test]
fn adaptive3d_and_waterline3d_holder_clearance_avoids_tall_overhang_in_tridexel() {
    // Tall overhanging T-flange (`roof_top_z = 22.0`) with `stickout = 18.0`:
    // Below `Z = 4.5` (`22.0 - 18.0 + 0.5`), the collet nut (`r_nut = 12.0`) would crash into
    // the roof unless holder clearance lifts the avoidance envelope!
    let stl = build_overhanging_t_flange_stl(22.0);
    let doc =
        load_mesh_cad_document("t_flange_tall.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new("setup_tflange_tall", "Tall T-Flange", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27 {
            x: AxisAnchor::Center,
            y: AxisAnchor::Center,
            z: AxisAnchor::Min,
        },
    };
    setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(10.0),
        side_neg_x: ParamLength::from_mm(10.0),
        side_pos_y: ParamLength::from_mm(10.0),
        side_neg_y: ParamLength::from_mm(10.0),
        top_offset_z: ParamLength::from_mm(0.5),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(22.5),
        round_increment_mm: None,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    // Standard shank (`cutting_d = shank_d = 6.0`) with `flute_length = 14.0`, `stickout = 18.0`
    let tool = make_reduced_flute_flat_tool("t_holder_test", 3, 6.0, 14.0, 6.0, 18.0);
    let cancel = AtomicBool::new(false);

    let mut op_adap = Operation::new("op_adap_h", "Adaptive 3D Holder", OperationKind::Adaptive3D);
    op_adap.tool_id = Some(tool.id.clone());
    op_adap.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 2.0);
    op_adap.passes.stock_to_leave_radial = ParamLength::from_mm(0.2);
    op_adap.passes.stock_to_leave_axial = ParamLength::from_mm(0.1);
    if let StrategyParams::Adaptive3D(p) = op_adap.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::StockBox;
        p.max_roughing_stepdown = ParamLength::from_mm(6.0);
        p.fine_stepdown = ParamLength::from_mm(2.0);
        p.adaptive.optimal_load = ParamLength::from_mm(2.0);
        p.detect_flats = true;
    }

    let tp_adap = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_adap,
        tool: &tool,
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
    assert!(
        tp_adap
            .warnings
            .iter()
            .any(|w| w.contains("Reach-limit") && w.contains("holder")),
        "Adaptive3D must emit a reach-limit warning for collet/holder clearance, got: {:?}",
        tp_adap.warnings
    );

    let mut op_wl = Operation::new("op_wl_h", "Waterline 3D Holder", OperationKind::Waterline3D);
    op_wl.tool_id = Some(tool.id.clone());
    op_wl.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 2.5);
    op_wl.passes.max_stepdown = ParamLength::from_mm(3.0);
    if let StrategyParams::Waterline3D(p) = op_wl.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::StockBox;
        p.boundary.tool_containment = ToolContainmentMode::ToolOutsideBoundary;
        p.adaptive_stepdown = false;
        p.detect_flats = true;
    }

    let tp_wl = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_wl,
        tool: &tool,
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
    assert!(
        tp_wl
            .warnings
            .iter()
            .any(|w| w.contains("Reach-limit") && w.contains("holder")),
        "Waterline3D must emit a reach-limit warning for collet/holder clearance, got: {:?}",
        tp_wl.warnings
    );

    let sim_res = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved,
        target_bodies: &[],
        fixtures: &[],
        operations: &[
            SimOperationInput::from_operation(&op_adap, &tool, &tp_adap),
            SimOperationInput::from_operation(&op_wl, &tool, &tp_wl),
        ],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: Some(&cancel),
        engine: SimEngineKind::TriDexel,
        backend_choice: SimComputeBackendChoice::Auto,
        ..SetupSimulationInput::new(&doc, &resolved, &[])
    })
    .unwrap();

    let bad: Vec<_> = sim_res
        .diagnostics
        .iter()
        .filter(|d| {
            matches!(
                d.kind,
                SimIssueKind::HolderCollision
                    | SimIssueKind::TargetModelGouge
                    | SimIssueKind::RapidIntoStock
            )
        })
        .collect();
    assert!(
        bad.is_empty(),
        "Adaptive3D + Waterline3D with holder clearance must achieve 0 HolderCollision, 0 TargetModelGouge, 0 RapidIntoStock; got: {bad:#?}"
    );
}

#[test]
fn overhang_guard_warns_and_opt_in_avoidance_protects_25d_ops_beneath_bridge() {
    let stl = build_stepped_overhanging_bridge_stl();
    let doc =
        load_mesh_cad_document("bridge_overhang.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new("setup_bridge", "Bridge Setup", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27 {
            x: AxisAnchor::Center,
            y: AxisAnchor::Center,
            z: AxisAnchor::Min,
        },
    };
    setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(0.0),
        side_neg_x: ParamLength::from_mm(0.0),
        side_pos_y: ParamLength::from_mm(0.0),
        side_neg_y: ParamLength::from_mm(0.0),
        top_offset_z: ParamLength::from_mm(0.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(20.0),
        round_increment_mm: None,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    // Build a 2D Sketch Layer at Z = 4.0 mm (lower shelf beneath the Z=14..20 bridge spanning Y ∈ [-5..5])
    let mut layer = SketchLayer::new("sk_shelf", "Shelf Features", "setup_bridge");
    layer.anchor = SketchAnchor::SetupXy {
        setup_id: "setup_bridge".to_string(),
        plane_ref: SketchPlaneRef::WcsOrigin,
        z_offset: ParamLength::from_mm(4.0),
    };
    layer.entities.push(SketchEntityEntry {
        id: "e_pocket".to_string(),
        name: "Shelf Pocket".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(0.0),
            y: ParamLength::from_mm(0.0),
            width: ParamLength::from_mm(32.0),  // X ∈ [-16..16]
            height: ParamLength::from_mm(32.0), // Y ∈ [-16..16]
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
        },
    });
    layer.entities.push(SketchEntityEntry {
        id: "e_contour".to_string(),
        name: "Shelf Contour".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(0.0),
            y: ParamLength::from_mm(0.0),
            width: ParamLength::from_mm(24.0),  // X ∈ [-12..12]
            height: ParamLength::from_mm(26.0), // Y ∈ [-13..13]
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
        },
    });
    layer.entities.push(SketchEntityEntry {
        id: "e_hole_occ".to_string(),
        name: "Occluded Hole".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Circle {
            center_x: ParamLength::from_mm(0.0),
            center_y: ParamLength::from_mm(0.0), // Directly under the Y ∈ [-5..5] overhanging bridge!
            center_xy: None,
            diameter: ParamLength::from_mm(4.0),
        },
    });
    layer.entities.push(SketchEntityEntry {
        id: "e_hole_clr".to_string(),
        name: "Clear Hole".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Circle {
            center_x: ParamLength::from_mm(0.0),
            center_y: ParamLength::from_mm(11.5), // Strictly in clear open air (`y = 11.5 > 5 + 2 = 7`)!
            center_xy: None,
            diameter: ParamLength::from_mm(4.0),
        },
    });
    layer.rebuild_regions(None);
    let sketches = vec![layer];

    let flat_4 = make_reduced_flute_flat_tool("t_flat4", 1, 4.0, 18.0, 4.0, 28.0);
    let drill_4 = make_twist_drill("t_drill4", 2, 4.0, 28.0);
    let cancel = AtomicBool::new(false);

    // --- 1. Pocket2D beneath overhanging bridge ---
    let make_pocket = |avoid: bool| {
        let mut op = Operation::new("op_pkt", "Shelf Pocket", OperationKind::Pocket2D);
        op.tool_id = Some(flat_4.id.clone());
        op.selections.push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sk_shelf".to_string(),
            entity_id: "e_pocket".to_string(),
            reverse: false,
            side_flip: false,
        });
        op.heights.top = HeightSpec::new(HeightReference::ModelBottom, 6.0);
        op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 4.0);
        op.passes.multiple_depths = false;
        op.passes.avoid_model_overhangs = avoid;
        if let StrategyParams::Pocket2D(p) = op.strategy_mut() {
            p.strategy = PocketStrategyKind::ContourParallel;
            p.stepover = ParamLength::from_mm(1.8);
            p.finish_wall_pass = false;
        }
        op
    };

    let tp_pkt_warn = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &sketches,
        setup: &resolved,
        op: &make_pocket(false),
        tool: &flat_4,
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
    assert!(
        tp_pkt_warn
            .warnings
            .iter()
            .any(|w| w.contains("overhanging") && w.contains("Avoid Model Overhangs")),
        "Pocket2D with avoid_model_overhangs=false must emit overhang warning, got: {:?}",
        tp_pkt_warn.warnings
    );

    let op_pkt_safe = make_pocket(true);
    let tp_pkt_safe = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &sketches,
        setup: &resolved,
        op: &op_pkt_safe,
        tool: &flat_4,
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
    assert!(!tp_pkt_safe.segments_wcs.is_empty());

    // --- 2. Contour2D beneath overhanging bridge ---
    let make_contour = |avoid: bool| {
        let mut op = Operation::new("op_cnt", "Shelf Contour", OperationKind::Contour2D);
        op.tool_id = Some(flat_4.id.clone());
        op.selections.push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sk_shelf".to_string(),
            entity_id: "e_contour".to_string(),
            reverse: false,
            side_flip: false,
        });
        op.heights.top = HeightSpec::new(HeightReference::ModelBottom, 6.0);
        op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 4.0);
        op.passes.multiple_depths = false;
        op.passes.avoid_model_overhangs = avoid;
        if let StrategyParams::Contour2D(p) = op.strategy_mut() {
            p.compensation = ContourCompensation::LeftClimb;
        }
        op
    };

    let tp_cnt_warn = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &sketches,
        setup: &resolved,
        op: &make_contour(false),
        tool: &flat_4,
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
    assert!(
        tp_cnt_warn
            .warnings
            .iter()
            .any(|w| w.contains("overhanging")),
        "Contour2D with avoid_model_overhangs=false must emit overhang warning, got: {:?}",
        tp_cnt_warn.warnings
    );

    let op_cnt_safe = make_contour(true);
    let tp_cnt_safe = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &sketches,
        setup: &resolved,
        op: &op_cnt_safe,
        tool: &flat_4,
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
    assert!(!tp_cnt_safe.segments_wcs.is_empty());

    // --- 3. Facing beneath overhanging bridge ---
    let make_facing = |avoid: bool| {
        let mut op = Operation::new("op_face", "Shelf Facing", OperationKind::Facing);
        op.tool_id = Some(flat_4.id.clone());
        op.selections.push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sk_shelf".to_string(),
            entity_id: "e_pocket".to_string(),
            reverse: false,
            side_flip: false,
        });
        op.heights.top = HeightSpec::new(HeightReference::ModelBottom, 6.0);
        op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 4.0);
        op.passes.multiple_depths = false;
        op.passes.avoid_model_overhangs = avoid;
        if let StrategyParams::Facing(p) = op.strategy_mut() {
            p.stepover = ParamLength::from_mm(2.5);
        }
        op
    };

    let tp_face_warn = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &sketches,
        setup: &resolved,
        op: &make_facing(false),
        tool: &flat_4,
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
    assert!(
        tp_face_warn
            .warnings
            .iter()
            .any(|w| w.contains("overhanging")),
        "Facing with avoid_model_overhangs=false must emit overhang warning, got: {:?}",
        tp_face_warn.warnings
    );

    let op_face_safe = make_facing(true);
    let tp_face_safe = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &sketches,
        setup: &resolved,
        op: &op_face_safe,
        tool: &flat_4,
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
    assert!(!tp_face_safe.segments_wcs.is_empty());

    // --- 4. DrillBore with 1 occluded hole + 1 clear hole ---
    let make_drill = |avoid: bool| {
        let mut op = Operation::new("op_drill", "Shelf Holes", OperationKind::DrillBore);
        op.tool_id = Some(drill_4.id.clone());
        op.selections.push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sk_shelf".to_string(),
            entity_id: "e_hole_occ".to_string(),
            reverse: false,
            side_flip: false,
        });
        op.selections.push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sk_shelf".to_string(),
            entity_id: "e_hole_clr".to_string(),
            reverse: false,
            side_flip: false,
        });
        op.heights.top = HeightSpec::new(HeightReference::ModelBottom, 4.0);
        op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 4.0);
        op.passes.avoid_model_overhangs = avoid;
        if let StrategyParams::DrillBore(p) = op.strategy_mut() {
            p.mode = HoleMachiningMode::Drill;
            p.tip_breakthrough = false;
        }
        op
    };

    let tp_drill_warn = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &sketches,
        setup: &resolved,
        op: &make_drill(false),
        tool: &drill_4,
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
    assert!(
        tp_drill_warn
            .warnings
            .iter()
            .any(|w| w.contains("overhanging")),
        "DrillBore with avoid_model_overhangs=false must emit overhang warning, got: {:?}",
        tp_drill_warn.warnings
    );

    let op_drill_safe = make_drill(true);
    let tp_drill_safe = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &sketches,
        setup: &resolved,
        op: &op_drill_safe,
        tool: &drill_4,
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
    assert!(
        tp_drill_safe
            .warnings
            .iter()
            .any(|w| w.contains("Skipped hole") && w.contains("occluded")),
        "DrillBore with avoid_model_overhangs=true must skip occluded hole with per-hole warning, got: {:?}",
        tp_drill_safe.warnings
    );
    assert_eq!(
        tp_drill_safe.stats.pass_count, 1,
        "Only the 1 unobstructed hole at (0, 11.5) should be drilled"
    );

    // --- 5. Tri-Dexel verification of all 4 overhang-avoiding 2.5D toolpaths ---
    // Note: in `bridge_overhang.stl`, the lower shelf sits at Z=4 with open air above it up to Z=20;
    // verify zero TargetModelGouge and zero HolderCollision against the 3D overhanging bridge solid!
    let sim_res = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved,
        target_bodies: &[],
        fixtures: &[],
        operations: &[
            SimOperationInput::from_operation(&op_face_safe, &flat_4, &tp_face_safe),
            SimOperationInput::from_operation(&op_pkt_safe, &flat_4, &tp_pkt_safe),
            SimOperationInput::from_operation(&op_cnt_safe, &flat_4, &tp_cnt_safe),
            SimOperationInput::from_operation(&op_drill_safe, &drill_4, &tp_drill_safe),
        ],
        kinematics: MachineKinematicsConfig::default(),
        resolution: SimResolution::Standard,
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: Some(&cancel),
        engine: SimEngineKind::TriDexel,
        backend_choice: SimComputeBackendChoice::Auto,
        ..SetupSimulationInput::new(&doc, &resolved, &[])
    })
    .unwrap();

    let gouges_or_holder: Vec<_> = sim_res
        .diagnostics
        .iter()
        .filter(|d| {
            matches!(
                d.kind,
                SimIssueKind::TargetModelGouge | SimIssueKind::HolderCollision
            )
        })
        .collect();
    assert!(
        gouges_or_holder.is_empty(),
        "Opt-in avoid_model_overhangs 2.5D ops must have 0 TargetModelGouge and 0 HolderCollision against overhanging bridge; got: {gouges_or_holder:#?}"
    );
}

#[test]
fn normal_prismatic_step_bracket_emits_zero_false_overhang_warnings() {
    let step = sample_multi_feature_bracket_step(false).unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("bracket.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();

    let mut setup = Setup::new("setup_bracket", "Prismatic Bracket", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
    };
    setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(2.0),
        side_neg_x: ParamLength::from_mm(2.0),
        side_pos_y: ParamLength::from_mm(2.0),
        side_neg_y: ParamLength::from_mm(2.0),
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(21.0),
        round_increment_mm: None,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let starter = default_starter_tool_library();
    let flat_6 = starter
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.2
            )
        })
        .unwrap();
    let chamfer_tool = starter
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
        .unwrap();
    let drill_6 = make_twist_drill("t_drill_6", 8, 6.0, 28.0);
    let cancel = AtomicBool::new(false);

    // 1. Facing top face (Z = 20.0)
    let mut op_face = Operation::new("op_face", "Face Top", OperationKind::Facing);
    op_face.tool_id = Some(flat_6.id.clone());
    let tp_face = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_face,
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
    assert!(
        !tp_face.warnings.iter().any(|w| w.contains("overhanging")),
        "Normal Facing must not emit false overhang warning: {:?}",
        tp_face.warnings
    );

    // 2. Pocket2D on Pocket A floor (Z = 12.0)
    let pkt_floor = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && (origin_mm.z - 12.0).abs() < 0.05 && !f.inner_loops.is_empty()
            )
        })
        .unwrap();
    let mut op_pkt = Operation::new("op_pkt", "Pocket A", OperationKind::Pocket2D);
    op_pkt.tool_id = Some(flat_6.id.clone());
    op_pkt.selections.push(GeometrySelectionItem::PlanarFace {
        binding: doc.bind_face(pkt_floor.id).unwrap(),
    });
    let tp_pkt = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_pkt,
        tool: flat_6,
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
    assert!(
        !tp_pkt.warnings.iter().any(|w| w.contains("overhanging")),
        "Normal prismatic Pocket2D must not emit false overhang warning: {:?}",
        tp_pkt.warnings
    );

    // 3. Contour2D around outer top face (Z = 20 -> 0)
    let top_face = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && (origin_mm.z - 20.0).abs() < 0.05
            )
        })
        .unwrap();
    let mut op_cnt = Operation::new("op_cnt", "Outer Profile", OperationKind::Contour2D);
    op_cnt.tool_id = Some(flat_6.id.clone());
    op_cnt.selections.push(GeometrySelectionItem::PlanarFace {
        binding: doc.bind_face(top_face.id).unwrap(),
    });
    op_cnt.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
    let tp_cnt = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_cnt,
        tool: flat_6,
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
    assert!(
        !tp_cnt.warnings.iter().any(|w| w.contains("overhanging")),
        "Normal prismatic Contour2D must not emit false overhang warning: {:?}",
        tp_cnt.warnings
    );

    // 4. DrillBore on the 4 through-holes (Ø6 mm)
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
                } if (radius_mm - 3.0).abs() < 0.05
            )
        })
        .unwrap()
        .id;
    let mut op_drill = Operation::new("op_drill", "Drill Ø6 Holes", OperationKind::DrillBore);
    op_drill.tool_id = Some(drill_6.id.clone());
    for fid in find_same_diameter_holes(&doc, r3_seed, 1e-3, 1e-3) {
        op_drill
            .selections
            .push(GeometrySelectionItem::CylindricalHole {
                binding: doc.bind_face(fid).unwrap(),
            });
    }
    let tp_drill = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_drill,
        tool: &drill_6,
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
    assert!(
        !tp_drill.warnings.iter().any(|w| w.contains("overhanging")),
        "Normal prismatic DrillBore must not emit false overhang warning: {:?}",
        tp_drill.warnings
    );

    // 5. ChamferTrace (SharpEdge) on top face outer perimeter
    let mut op_ch = Operation::new("op_ch", "Top Edge Chamfer", OperationKind::ChamferTrace);
    op_ch.tool_id = Some(chamfer_tool.id.clone());
    op_ch.selections.push(GeometrySelectionItem::PlanarFace {
        binding: doc.bind_face(top_face.id).unwrap(),
    });
    if let StrategyParams::ChamferTrace(p) = op_ch.strategy_mut() {
        p.mode = ChamferTraceMode::EdgeChamfer;
        p.chamfer_source = ChamferSourceMode::SharpEdge {
            width: ParamLength::from_mm(0.5),
        };
    }
    let tp_ch = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_ch,
        tool: chamfer_tool,
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
    assert!(
        !tp_ch.warnings.iter().any(|w| w.contains("overhanging")),
        "Normal SharpEdge ChamferTrace must not emit false overhang warning: {:?}",
        tp_ch.warnings
    );
}
