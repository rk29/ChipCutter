#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::sync::atomic::AtomicBool;

use chip_cad::{
    MeshImportOptions, load_mesh_cad_document, sample_relief_wave_obj_text,
    sample_sculpted_dome_stl_bytes,
};
use chip_cam::{
    BoxAnchor27, HeightReference, HeightSpec, MachiningBoundaryMode, MotionRole, Operation,
    OperationGenRequest, OperationKind, ParallelCutDirection, RestMachiningConfig,
    RestMachiningMode, SegmentMotion3D, Setup, StockConfig, StrategyParams, ToolContainmentMode,
    WaterlineOrderMode, WcsOriginMode, WorkOffsetSlot, generate_operation_toolpath_with_rest,
};
use chip_math::{DVec3, DisplayUnit, ParamLength};
use chip_sim::{
    MachineKinematicsConfig, SetupSimulationInput, SimComputeBackendChoice, SimEngineKind,
    SimOperationInput, SimResolution, SimVerificationConfig, run_setup_simulation_with_choice,
};
use chip_tools::{
    CoolantMode, CutterGeometry, CuttingPreset, SpindleDirection, SurfaceSpeedPresetSpec, Tool,
    ToolId, default_starter_tool_library,
};

fn make_ball_tool(id: &str, tool_number: u32, diameter_mm: f64) -> Tool {
    Tool {
        id: ToolId(id.to_string()),
        tool_number,
        length_offset_number: tool_number,
        diameter_offset_number: tool_number,
        name: format!("{diameter_mm:.1}mm 2FL Carbide Ball Endmill"),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::AirBlast,
        stickout_length_mm: 30.0,
        geometry: CutterGeometry::BallEndmill {
            diameter_mm,
            flute_length_mm: 16.0,
            overall_length_mm: 55.0,
            shank_diameter_mm: diameter_mm,
            flutes: 2,
        },
        presets: vec![CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
            id: "al6061_3d_finish",
            name: "Aluminum 6061 - 3D Finish",
            material: "Aluminum 6061",
            diameter_mm,
            flutes: 2,
            surface_speed_m_min: 220.0,
            chipload_mm_tooth: 0.020,
            default_stepdown_mm: 0.8,
            default_stepover_mm: 0.5,
            coolant_override: Some(CoolantMode::Mist),
        })],
    }
}

/// Builds a closed watertight binary STL buffer from raw CCW outward triangles.
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

/// Builds a watertight diagonal concave V-trough solid whose exact analytical bi-tangent crease
/// lies along the 45° diagonal `y = x` (`z(x, y) = 4.0 + 0.65 * |x - y|` on `[-20, 20]^2`).
fn build_diagonal_concave_fillet_stl() -> Vec<u8> {
    let n = 40usize;
    let lo = -20.0_f64;
    let hi = 20.0_f64;
    let step = (hi - lo) / n as f64;
    let top_z = |x: f64, y: f64| -> f64 { 4.0 + 0.65 * (x - y).abs() };

    let mut tris: Vec<[DVec3; 3]> = Vec::new();
    for iy in 0..n {
        let y0 = lo + iy as f64 * step;
        let y1 = y0 + step;
        for ix in 0..n {
            let x0 = lo + ix as f64 * step;
            let x1 = x0 + step;
            let p00 = DVec3::new(x0, y0, top_z(x0, y0));
            let p10 = DVec3::new(x1, y0, top_z(x1, y0));
            let p11 = DVec3::new(x1, y1, top_z(x1, y1));
            let p01 = DVec3::new(x0, y1, top_z(x0, y1));
            // Split every quad along the `x - y = const` diagonal (`p00 -> p11`) so the exact
            // crease along `y = x` is represented by exact mesh edges with zero diagonal aliasing!
            tris.push([p00, p10, p11]);
            tris.push([p00, p11, p01]);
        }
    }

    // Bottom face (`z = 0`, outward `-Z`)
    let b00 = DVec3::new(lo, lo, 0.0);
    let b10 = DVec3::new(hi, lo, 0.0);
    let b11 = DVec3::new(hi, hi, 0.0);
    let b01 = DVec3::new(lo, hi, 0.0);
    tris.push([b00, b11, b10]);
    tris.push([b00, b01, b11]);

    // 4 perimeter side walls
    for i in 0..n {
        let s0 = lo + i as f64 * step;
        let s1 = s0 + step;
        // Front wall (`y = lo`, outward `-Y`)
        tris.push([
            DVec3::new(s0, lo, 0.0),
            DVec3::new(s1, lo, 0.0),
            DVec3::new(s1, lo, top_z(s1, lo)),
        ]);
        tris.push([
            DVec3::new(s0, lo, 0.0),
            DVec3::new(s1, lo, top_z(s1, lo)),
            DVec3::new(s0, lo, top_z(s0, lo)),
        ]);
        // Back wall (`y = hi`, outward `+Y`)
        tris.push([
            DVec3::new(s1, hi, 0.0),
            DVec3::new(s0, hi, 0.0),
            DVec3::new(s0, hi, top_z(s0, hi)),
        ]);
        tris.push([
            DVec3::new(s1, hi, 0.0),
            DVec3::new(s0, hi, top_z(s0, hi)),
            DVec3::new(s1, hi, top_z(s1, hi)),
        ]);
        // Left wall (`x = lo`, outward `-X`)
        tris.push([
            DVec3::new(lo, s1, 0.0),
            DVec3::new(lo, s0, 0.0),
            DVec3::new(lo, s0, top_z(lo, s0)),
        ]);
        tris.push([
            DVec3::new(lo, s1, 0.0),
            DVec3::new(lo, s0, top_z(lo, s0)),
            DVec3::new(lo, s1, top_z(lo, s1)),
        ]);
        // Right wall (`x = hi`, outward `+X`)
        tris.push([
            DVec3::new(hi, s0, 0.0),
            DVec3::new(hi, s1, 0.0),
            DVec3::new(hi, s1, top_z(hi, s1)),
        ]);
        tris.push([
            DVec3::new(hi, s0, 0.0),
            DVec3::new(hi, s1, top_z(hi, s1)),
            DVec3::new(hi, s0, top_z(hi, s0)),
        ]);
    }

    encode_test_stl(&tris)
}

/// Builds a single watertight `40 × 40 × 20 mm` vertical prism with a narrow `6 mm`-wide localized
/// shallow ramp protruding `6 mm` outward on the `+X` face between `Z = 10` and `Z = 14`
/// (changing total cross-sectional area by `< 2%`, while local radial offset is `6 mm`).
fn build_localized_shallow_shelf_stl() -> Vec<u8> {
    let mut tris: Vec<[DVec3; 3]> = Vec::new();
    let push_quad = |tris: &mut Vec<[DVec3; 3]>, a: DVec3, b: DVec3, c: DVec3, d: DVec3| {
        tris.push([a, b, c]);
        tris.push([a, c, d]);
    };

    // Build cleanly by sweeping the closed CCW 2D footprint across Z bands:
    // Band 1: Z ∈ [0, 10] -> box with +6mm tab at Y ∈ [-3, 3] (X = 26)
    // Band 2: Z ∈ [10, 14] -> tab slopes smoothly inward from X = 26 back to X = 20
    // Band 3: Z ∈ [14, 20] -> pure 40x40 box (X ∈ [-20, 20], Y ∈ [-20, 20])
    let profile_at_tab_x = |tab_x: f64| -> [(f64, f64); 8] {
        [
            (-20.0, -20.0),
            (20.0, -20.0),
            (20.0, -3.0),
            (tab_x, -3.0),
            (tab_x, 3.0),
            (20.0, 3.0),
            (20.0, 20.0),
            (-20.0, 20.0),
        ]
    };
    let z_rings = [
        (0.0_f64, 26.0_f64),
        (10.0_f64, 26.0_f64),
        (14.0_f64, 20.0_f64),
        (20.0_f64, 20.0_f64),
    ];

    for w in z_rings.windows(2) {
        let (za, xa) = w[0];
        let (zb, xb) = w[1];
        let pa = profile_at_tab_x(xa);
        let pb = profile_at_tab_x(xb);
        for i in 0..8 {
            let j = (i + 1) % 8;
            let a0 = DVec3::new(pa[i].0, pa[i].1, za);
            let a1 = DVec3::new(pa[j].0, pa[j].1, za);
            let b1 = DVec3::new(pb[j].0, pb[j].1, zb);
            let b0 = DVec3::new(pb[i].0, pb[i].1, zb);
            if a0.distance_squared(a1) > 1e-9 || b0.distance_squared(b1) > 1e-9 {
                push_quad(&mut tris, a0, a1, b1, b0);
            }
        }
    }

    // Bottom cap (`z = 0`, outward `-Z`): decompose concave footprint into 2 rectangles
    // Main box [-20..20] x [-20..20] + tab [20..26] x [-3..3]
    push_quad(
        &mut tris,
        DVec3::new(-20.0, -20.0, 0.0),
        DVec3::new(-20.0, 20.0, 0.0),
        DVec3::new(20.0, 20.0, 0.0),
        DVec3::new(20.0, -20.0, 0.0),
    );
    push_quad(
        &mut tris,
        DVec3::new(20.0, -3.0, 0.0),
        DVec3::new(20.0, 3.0, 0.0),
        DVec3::new(26.0, 3.0, 0.0),
        DVec3::new(26.0, -3.0, 0.0),
    );
    // Top cap (`z = 20`, outward `+Z`)
    push_quad(
        &mut tris,
        DVec3::new(-20.0, -20.0, 20.0),
        DVec3::new(20.0, -20.0, 20.0),
        DVec3::new(20.0, 20.0, 20.0),
        DVec3::new(-20.0, 20.0, 20.0),
    );

    encode_test_stl(&tris)
}

#[test]
fn relief_wave_09_finishing_tridexel_oracle_eliminates_gouge() {
    let obj_text = sample_relief_wave_obj_text();
    let doc = load_mesh_cad_document(
        "09-relief-wave.obj",
        obj_text.as_bytes(),
        &MeshImportOptions::default(),
    )
    .unwrap();

    let mut setup = Setup::new("setup_wave_m57", "Wave M57", WorkOffsetSlot::G54);
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
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let starter = default_starter_tool_library();
    let flat_6 = starter
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5
            )
        })
        .unwrap()
        .clone();
    let ball_6 = starter
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::BallEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5
            )
        })
        .unwrap()
        .clone();

    let cancel = AtomicBool::new(false);

    // Op 1: 3D Adaptive Roughing
    let mut op_rough = Operation::new("op_1_rough", "Adaptive Rough", OperationKind::Adaptive3D);
    op_rough.tool_id = Some(flat_6.id.clone());
    op_rough.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -9.0);
    op_rough.passes.stock_to_leave_radial = ParamLength::from_mm(0.4);
    op_rough.passes.stock_to_leave_axial = ParamLength::from_mm(0.25);
    if let StrategyParams::Adaptive3D(p) = op_rough.strategy_mut() {
        p.max_roughing_stepdown = ParamLength::from_mm(4.0);
        p.fine_stepdown = ParamLength::from_mm(2.0);
        p.adaptive.optimal_load = ParamLength::from_mm(1.5);
        p.detect_flats = true;
    }

    // Op 2: 3D Parallel Finishing (45° + Cross)
    let mut op_par = Operation::new("op_2_par", "Parallel Finish", OperationKind::Parallel3D);
    op_par.tool_id = Some(ball_6.id.clone());
    op_par.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -10.0);
    if let StrategyParams::Parallel3D(p) = op_par.strategy_mut() {
        p.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.tool_containment = ToolContainmentMode::ToolCenterOnBoundary;
        p.use_cusp_height = true;
        p.cusp_height = ParamLength::from_mm(0.04);
        p.pass_angle_deg = 45.0;
        p.cross_pass = true;
        p.cut_direction = ParallelCutDirection::ZigZag;
        p.min_slope_deg = 0.0;
        p.max_slope_deg = 60.0;
    }

    // Op 3: 3D Constant-Scallop (Rest)
    let mut op_sc = Operation::new("op_3_sc", "Scallop Finish", OperationKind::Scallop3D);
    op_sc.tool_id = Some(ball_6.id.clone());
    op_sc.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -10.0);
    op_sc.passes.rest_machining = RestMachiningConfig {
        mode: RestMachiningMode::FromPreviousTool {
            prev_tool_diameter: ParamLength::from_mm(12.0),
            prev_stock_to_leave: ParamLength::from_mm(0.25),
        },
        rest_overlap: ParamLength::from_mm(0.5),
        min_rest_thickness: ParamLength::from_mm(0.05),
    };
    if let StrategyParams::Scallop3D(p) = op_sc.strategy_mut() {
        p.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.tool_containment = ToolContainmentMode::ToolCenterOnBoundary;
        p.cusp_height = ParamLength::from_mm(0.04);
        p.max_stepover = ParamLength::from_mm(1.5);
        p.min_stepover = ParamLength::from_mm(0.25);
        p.inside_out = true;
        p.climb_milling = true;
    }

    let tp_rough = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_rough,
        tool: &flat_6,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    let tp_par = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_par,
        tool: &ball_6,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
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

    let sim = run_setup_simulation_with_choice(
        &SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[
                SimOperationInput::from_operation(&op_rough, &flat_6, &tp_rough),
                SimOperationInput::from_operation(&op_par, &ball_6, &tp_par),
                SimOperationInput::from_operation(&op_sc, &ball_6, &tp_sc),
            ],
            kinematics: MachineKinematicsConfig::default(),
            resolution: SimResolution::Standard,
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
        },
        SimEngineKind::TriDexel,
        SimComputeBackendChoice::Auto,
    )
    .unwrap();

    assert_eq!(
        sim.critical_error_count(),
        0,
        "09-3d-surface-finishing must pass Tri-Dexel with 0 critical errors, got: {:#?}",
        sim.diagnostics
    );

    let err = sim.error_report.as_ref().expect("Tri-Dexel error_report");
    assert!(
        err.gouge_volume_mm3 <= 1.0,
        "09-3d-surface-finishing gouge_volume_mm3 must be <= 1.0 mm³ (was 65.1 mm³), got {:.3} mm³",
        err.gouge_volume_mm3
    );
    assert!(
        err.max_gouge_mm <= 0.015,
        "09-3d-surface-finishing max_gouge_mm must be <= 0.015 mm (was 0.146 mm), got {:.4} mm",
        err.max_gouge_mm
    );
}

#[test]
fn pencil3d_diagonal_concave_fillet_subcell_centering_tracks_exact_crease_within_0_05mm() {
    let stl = build_diagonal_concave_fillet_stl();
    let doc =
        load_mesh_cad_document("diag_fillet.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new("setup_diag_pencil", "Diagonal Fillet", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let ball_3 = make_ball_tool("t_ball_3", 11, 3.0);
    let mut op = Operation::new("op_pencil_diag", "Diagonal Pencil", OperationKind::Pencil3D);
    op.tool_id = Some(ball_3.id.clone());
    if let StrategyParams::Pencil3D(p) = op.strategy_mut() {
        p.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.tool_containment = ToolContainmentMode::ToolInsideBoundary;
        p.reference_tool_diameter = ParamLength::from_mm(8.0);
        p.valley_threshold = ParamLength::from_mm(0.08);
        p.num_offset_passes = 0;
    }

    let cancel = AtomicBool::new(false);
    let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op,
        tool: &ball_3,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    // Exact diagonal bi-tangent crease in WCS is `y = x`; perpendicular lateral error is `|x - y| / sqrt(2)`.
    let mut lateral_errors = Vec::new();
    for seg in &tp.segments_wcs {
        if matches!(
            seg.motion,
            SegmentMotion3D::Linear {
                role: MotionRole::FinishCut,
                ..
            }
        ) {
            for pt in [seg.start_wcs, seg.end_wcs] {
                if pt.x.abs() <= 12.0 && pt.y.abs() <= 12.0 {
                    let err = (pt.x - pt.y).abs() / std::f64::consts::SQRT_2;
                    lateral_errors.push(err);
                }
            }
        }
    }

    assert!(
        lateral_errors.len() >= 10,
        "Expected pencil centerline points along the interior diagonal crease, got {}",
        lateral_errors.len()
    );
    let max_err = lateral_errors.iter().copied().fold(0.0_f64, f64::max);
    let mean_err = lateral_errors.iter().sum::<f64>() / lateral_errors.len() as f64;
    assert!(
        max_err < 0.05,
        "Pencil3D sub-cell ridge centering max lateral error on diagonal fillet must be < 0.05 mm, got max={max_err:.4} mm (mean={mean_err:.4} mm)"
    );
}

#[test]
fn waterline3d_localized_shallow_shelf_inserts_adaptive_z_levels_despite_small_area_change() {
    let stl = build_localized_shallow_shelf_stl();
    let doc =
        load_mesh_cad_document("local_shelf.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new("setup_local_shelf", "Localized Shelf", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27 {
            x: chip_cam::AxisAnchor::Center,
            y: chip_cam::AxisAnchor::Center,
            z: chip_cam::AxisAnchor::Min,
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
        sheet_thickness_z: ParamLength::from_mm(20.0),
        round_increment_mm: None,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let ball_6 = make_ball_tool("t_ball_6", 4, 6.0);
    let cancel = AtomicBool::new(false);

    let make_wl = |adaptive: bool| {
        let mut op = Operation::new(
            if adaptive {
                "op_wl_adapt"
            } else {
                "op_wl_fixed"
            },
            "Waterline Shelf",
            OperationKind::Waterline3D,
        );
        op.tool_id = Some(ball_6.id.clone());
        op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 2.0);
        op.passes.max_stepdown = ParamLength::from_mm(4.0);
        if let StrategyParams::Waterline3D(p) = op.strategy_mut() {
            p.boundary_mode = MachiningBoundaryMode::StockBox;
            p.tool_containment = ToolContainmentMode::ToolOutsideBoundary;
            p.adaptive_stepdown = adaptive;
            p.min_stepdown = ParamLength::from_mm(0.5);
            p.max_cusp_height = ParamLength::from_mm(0.05);
            p.detect_flats = false;
            p.order_mode = WaterlineOrderMode::ByLevel;
        }
        op
    };

    let op_fixed = make_wl(false);
    let op_adapt = make_wl(true);

    let tp_fixed = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_fixed,
        tool: &ball_6,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    let tp_adapt = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_adapt,
        tool: &ball_6,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    // Count distinct FinishCut Z levels inside the localized shallow shelf band `Z ∈ (9.5, 14.5)`
    let count_shelf_levels = |tp: &chip_cam::GeneratedToolpath| -> usize {
        let mut zs: Vec<i64> = tp
            .segments_wcs
            .iter()
            .filter(|s| {
                matches!(
                    s.motion,
                    SegmentMotion3D::Linear {
                        role: MotionRole::FinishCut,
                        ..
                    } | SegmentMotion3D::Arc {
                        role: MotionRole::FinishCut,
                        ..
                    }
                ) && (s.start_wcs.z - s.end_wcs.z).abs() < 1e-3
                    && s.end_wcs.z > 9.5
                    && s.end_wcs.z < 14.5
            })
            .map(|s| (s.end_wcs.z * 100.0).round() as i64)
            .collect();
        zs.sort_unstable();
        zs.dedup();
        zs.len()
    };

    let fixed_n = count_shelf_levels(&tp_fixed);
    let adapt_n = count_shelf_levels(&tp_adapt);
    assert!(
        adapt_n >= fixed_n + 3,
        "Local radial offset adaptive Waterline3D must insert intermediate Z-levels on the narrow shallow shelf (adapt={adapt_n} vs fixed={fixed_n})"
    );
}

#[test]
fn sculpted_dome_08_tridexel_oracle_zero_gouge_and_clean_crown() {
    let stl = sample_sculpted_dome_stl_bytes();
    let doc =
        load_mesh_cad_document("sculpted_dome.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new("setup_dome_m57", "Dome M57", WorkOffsetSlot::G54);
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
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let starter = default_starter_tool_library();
    let flat_6 = starter
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5
            )
        })
        .unwrap()
        .clone();
    let ball_6 = starter
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::BallEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5
            )
        })
        .unwrap()
        .clone();
    let ball_3 = make_ball_tool("t11_3mm_ball_2fl", 11, 3.0);
    let cancel = AtomicBool::new(false);

    let mut op_rough = Operation::new("op_1_rough", "Adaptive3D", OperationKind::Adaptive3D);
    op_rough.tool_id = Some(flat_6.id.clone());
    op_rough.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -25.0);
    op_rough.passes.stock_to_leave_radial = ParamLength::from_mm(0.5);
    op_rough.passes.stock_to_leave_axial = ParamLength::from_mm(0.3);
    if let StrategyParams::Adaptive3D(p) = op_rough.strategy_mut() {
        p.max_roughing_stepdown = ParamLength::from_mm(6.0);
        p.fine_stepdown = ParamLength::from_mm(2.0);
        p.adaptive.optimal_load = ParamLength::from_mm(1.5);
        p.detect_flats = true;
    }

    let mut op_wl = Operation::new("op_2_wl", "Waterline3D", OperationKind::Waterline3D);
    op_wl.tool_id = Some(ball_6.id.clone());
    op_wl.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -25.0);
    op_wl.passes.max_stepdown = ParamLength::from_mm(1.75);
    if let StrategyParams::Waterline3D(p) = op_wl.strategy_mut() {
        p.adaptive_stepdown = true;
        p.min_stepdown = ParamLength::from_mm(0.5);
        p.max_cusp_height = ParamLength::from_mm(0.08);
        p.detect_flats = true;
    }

    let mut op_pencil = Operation::new("op_3_pencil", "Pencil3D", OperationKind::Pencil3D);
    op_pencil.tool_id = Some(ball_3.id.clone());
    op_pencil.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -25.0);
    if let StrategyParams::Pencil3D(p) = op_pencil.strategy_mut() {
        p.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.tool_containment = ToolContainmentMode::ToolInsideBoundary;
        p.reference_tool_diameter = ParamLength::from_mm(6.0);
        p.valley_threshold = ParamLength::from_mm(0.15);
        p.num_offset_passes = 0;
    }

    let tp_rough = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_rough,
        tool: &flat_6,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    let tp_wl = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_wl,
        tool: &ball_6,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();
    let tp_pencil = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_pencil,
        tool: &ball_3,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    let sim = run_setup_simulation_with_choice(
        &SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[
                SimOperationInput::from_operation(&op_rough, &flat_6, &tp_rough),
                SimOperationInput::from_operation(&op_wl, &ball_6, &tp_wl),
                SimOperationInput::from_operation(&op_pencil, &ball_3, &tp_pencil),
            ],
            kinematics: MachineKinematicsConfig::default(),
            resolution: SimResolution::Standard,
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
        },
        SimEngineKind::TriDexel,
        SimComputeBackendChoice::Auto,
    )
    .unwrap();

    assert_eq!(
        sim.critical_error_count(),
        0,
        "Sculpted Dome must pass Tri-Dexel with 0 critical errors, got: {:#?}",
        sim.diagnostics
    );
    let err = sim.error_report.as_ref().expect("Tri-Dexel error_report");
    assert!(
        err.gouge_volume_mm3 <= 0.1,
        "Sculpted Dome gouge_volume_mm3 must be <= 0.1 mm³, got {:.4}",
        err.gouge_volume_mm3
    );
    assert!(
        err.max_gouge_mm <= 0.015,
        "Sculpted Dome max_gouge_mm must be <= 0.015 mm, got {:.4}",
        err.max_gouge_mm
    );
}

/// Builds a closed watertight 3-blade helical loft solid along the X axis (`x ∈ [-18, +18] mm`,
/// base radius `6.5 mm`, 3 helical fins reaching `R = 12.5 mm` with `180°` twist along X).
fn build_triple_helical_loft_stl() -> Vec<u8> {
    let nx = 24usize;
    let n_theta = 48usize;
    let x_min = -18.0_f64;
    let x_max = 18.0_f64;

    let radius_at = |x: f64, theta: f64| -> f64 {
        let u = (x - x_min) / (x_max - x_min);
        let twist = std::f64::consts::PI * u;
        let wave = (3.0 * (theta - twist)).cos();
        // Smooth axial envelope tapering the 3 helical lofts into clean cylindrical end hubs
        let env = (std::f64::consts::PI * u).sin().powf(0.45);
        let fin = (wave * 0.5 + 0.5).powi(2) * env;
        6.5 + 6.0 * fin
    };

    let mut rings: Vec<Vec<DVec3>> = Vec::with_capacity(nx + 1);
    for ix in 0..=nx {
        let x = x_min + (x_max - x_min) * (ix as f64 / nx as f64);
        let mut ring = Vec::with_capacity(n_theta);
        for it in 0..n_theta {
            let th = std::f64::consts::TAU * (it as f64 / n_theta as f64);
            let r = radius_at(x, th);
            ring.push(DVec3::new(x, r * th.cos(), r * th.sin()));
        }
        rings.push(ring);
    }

    let mut tris: Vec<[DVec3; 3]> = Vec::new();
    for ix in 0..nx {
        for it in 0..n_theta {
            let it_next = (it + 1) % n_theta;
            let p00 = rings[ix][it];
            let p10 = rings[ix + 1][it];
            let p11 = rings[ix + 1][it_next];
            let p01 = rings[ix][it_next];
            // Outward radial normal: (p01 - p00) x (p10 - p00) = (+theta) x (+X) = +radial
            tris.push([p00, p11, p10]);
            tris.push([p00, p01, p11]);
        }
    }
    // End caps (`-X` outward `-X`, `+X` outward `+X`)
    let c_neg = DVec3::new(x_min, 0.0, 0.0);
    let c_pos = DVec3::new(x_max, 0.0, 0.0);
    for it in 0..n_theta {
        let it_next = (it + 1) % n_theta;
        tris.push([c_neg, rings[0][it_next], rings[0][it]]);
        tris.push([c_pos, rings[nx][it], rings[nx][it_next]]);
    }

    encode_test_stl(&tris)
}

#[test]
fn adaptive3d_indexed_helical_loft_zero_gouge_and_zero_rapid_crashes() {
    use chip_cam::{
        OperationRotaryMode, RotaryAxisKind, RotaryDriveMode, StockAxis, SurfaceCutterProfile,
        SurfaceMesh3D, build_indexed_virtual_setup, drop_cutter_z,
    };

    let stl = build_triple_helical_loft_stl();
    let doc = load_mesh_cad_document("triple_helix_loft.stl", &stl, &MeshImportOptions::default())
        .unwrap();

    let mut ball_4 = make_ball_tool("t_ball_4_helix", 5, 4.0);
    ball_4.stickout_length_mm = 32.0;
    if let CutterGeometry::BallEndmill {
        flute_length_mm, ..
    } = &mut ball_4.geometry
    {
        *flute_length_mm = 26.0;
    }

    let cancel = AtomicBool::new(false);

    // 1. True volumetric Tri-Dexel verification of 3D Adaptive roughing on the overhanging triple helical loft
    let mut box_setup = Setup::new(
        "setup_helix_3ax",
        "Helical Loft 3-Axis",
        WorkOffsetSlot::G54,
    );
    box_setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    box_setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(3.0),
        side_neg_x: ParamLength::from_mm(3.0),
        side_pos_y: ParamLength::from_mm(3.0),
        side_neg_y: ParamLength::from_mm(3.0),
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(26.0),
        round_increment_mm: None,
    };
    let resolved_box = box_setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let mut op_3ax = Operation::new(
        "op_helix_3ax",
        "Adaptive 3D Helical",
        OperationKind::Adaptive3D,
    );
    op_3ax.tool_id = Some(ball_4.id.clone());
    op_3ax.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_3ax.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
    op_3ax.passes.stock_to_leave_radial = ParamLength::from_mm(0.4);
    op_3ax.passes.stock_to_leave_axial = ParamLength::from_mm(0.25);
    if let StrategyParams::Adaptive3D(p) = op_3ax.strategy_mut() {
        p.boundary_mode = MachiningBoundaryMode::StockBox;
        p.max_roughing_stepdown = ParamLength::from_mm(6.0);
        p.fine_stepdown = ParamLength::from_mm(3.0);
        p.adaptive.optimal_load = ParamLength::from_mm(1.8);
        p.detect_flats = true;
    }

    let tp_3ax = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_box,
        op: &op_3ax,
        tool: &ball_4,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    let sim_box = run_setup_simulation_with_choice(
        &SetupSimulationInput {
            doc: &doc,
            setup: &resolved_box,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput::from_operation(&op_3ax, &ball_4, &tp_3ax)],
            kinematics: MachineKinematicsConfig::default(),
            resolution: SimResolution::Standard,
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
        },
        SimEngineKind::TriDexel,
        SimComputeBackendChoice::Auto,
    )
    .unwrap();

    assert_eq!(
        sim_box.critical_error_count(),
        0,
        "3D Adaptive on overhanging triple helical loft must have 0 critical errors in Tri-Dexel, got: {:#?}",
        sim_box.diagnostics
    );
    let err_box = sim_box
        .error_report
        .as_ref()
        .expect("Tri-Dexel error_report");
    assert!(
        err_box.gouge_volume_mm3 <= 0.1,
        "3D Adaptive helical loft gouge_volume_mm3 must be <= 0.1 mm³, got {:.4}",
        err_box.gouge_volume_mm3
    );
    assert!(
        err_box.max_gouge_mm <= 0.015,
        "3D Adaptive helical loft max_gouge_mm must be <= 0.015 mm, got {:.4}",
        err_box.max_gouge_mm
    );

    // 2. Indexed 3+1 rotary Adaptive3D across 3 helical phases (0°, 120°, 240°) verified against
    //    exact analytical `drop_cutter_z` in each indexed virtual WCS frame
    let mut rot_setup = Setup::new(
        "setup_helix_idx",
        "Indexed Helical Roughing",
        WorkOffsetSlot::G54,
    );
    rot_setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27 {
            x: chip_cam::AxisAnchor::Center,
            y: chip_cam::AxisAnchor::Center,
            z: chip_cam::AxisAnchor::Center,
        },
    };
    rot_setup.stock = StockConfig::FixedCylinder {
        axis: StockAxis::X,
        diameter: ParamLength::from_mm(30.0),
        length: ParamLength::from_mm(40.0),
        axial_top_offset: ParamLength::from_mm(2.0),
    };
    rot_setup.rotary.mode = RotaryDriveMode::Indexed3Plus1;
    rot_setup.rotary.axis = RotaryAxisKind::XAxis;
    rot_setup.rotary.clearance_radial_offset = ParamLength::from_mm(10.0);
    rot_setup.rotary.chuck_length = ParamLength::from_mm(0.0);
    let resolved_rot = rot_setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let rot = resolved_rot.rotary.expect("rotary config resolved");

    for (idx, angle_deg) in [0.0_f64, 120.0_f64, 240.0_f64].into_iter().enumerate() {
        let mut op = Operation::new(
            format!("op_helix_{idx}"),
            format!("Indexed Adaptive 3D {angle_deg:.0}°"),
            OperationKind::Adaptive3D,
        );
        op.tool_id = Some(ball_4.id.clone());
        op.rotary = OperationRotaryMode::Indexed3Plus1 { angle_deg };
        op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
        op.passes.stock_to_leave_radial = ParamLength::from_mm(0.4);
        op.passes.stock_to_leave_axial = ParamLength::from_mm(0.25);
        if let StrategyParams::Adaptive3D(p) = op.strategy_mut() {
            p.boundary_mode = MachiningBoundaryMode::StockBox;
            p.max_roughing_stepdown = ParamLength::from_mm(6.0);
            p.fine_stepdown = ParamLength::from_mm(3.0);
            p.adaptive.optimal_load = ParamLength::from_mm(1.8);
            p.detect_flats = true;
        }

        let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: &[],
            setup: &resolved_rot,
            op: &op,
            tool: &ball_4,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: idx == 0,
            upstream_rest: &[],
            carryover_stock: None,
            cancel: &cancel,
        })
        .unwrap();

        let vsetup = build_indexed_virtual_setup(&resolved_rot, &rot, angle_deg);
        let surface = SurfaceMesh3D::from_cad_document(&doc, &[], &vsetup.frame);
        let nominal_cutter = SurfaceCutterProfile::from_tool_geometry(&ball_4.geometry, 0.0, 0.0);
        let mut max_gouge = 0.0_f64;
        for seg in &tp.segments_wcs {
            if matches!(seg.motion, SegmentMotion3D::Rapid) {
                continue;
            }
            for t in [0.0, 0.5, 1.0] {
                let p = seg.sample_point(t);
                if let Some(zd) = drop_cutter_z(&surface, &nominal_cutter, p.x, p.y, -100.0) {
                    max_gouge = max_gouge.max(zd - p.z);
                }
            }
        }
        assert!(
            max_gouge <= 0.02,
            "Indexed Adaptive3D at {angle_deg}° must not cut into overhanging helical lofts, max_gouge={max_gouge:.4} mm"
        );
    }
}
