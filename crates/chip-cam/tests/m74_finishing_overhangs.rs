#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::sync::atomic::AtomicBool;

use chip_cad::{MeshImportOptions, load_mesh_bytes, load_mesh_cad_document};
use chip_cam::{
    AxisAnchor, BoxAnchor27, HeightReference, HeightSpec, MachiningBoundaryMode, MotionRole,
    Operation, OperationGenRequest, OperationKind, ParallelCutDirection, RestMachiningConfig,
    RestMachiningMode, SegmentMotion3D, Setup, SoftJawBoltPattern, SoftJawConfig,
    SoftJawOutputMode, StockConfig, StrategyParams, SurfaceCutterProfile, SurfaceMesh3D,
    ToolContainmentMode, WcsOriginMode, WorkOffsetSlot, build_3d_rest_mask, build_soft_jaw_plan,
    generate_operation_toolpath_with_rest,
    surface3d::{finishing::split_chains_at_overhang_cliffs, rect_region_2d},
};
use chip_math::{DVec2, DVec3, DisplayUnit, ParamLength};
use chip_sim::{
    MachineKinematicsConfig, SetupSimulationInput, SimComputeBackendChoice, SimEngineKind,
    SimOperationInput, SimResolution, SimVerificationConfig, run_setup_simulation_with_choice,
};
use chip_tools::{
    CoolantMode, CutterGeometry, CuttingPreset, SpindleDirection, SurfaceSpeedPresetSpec, Tool,
    ToolId,
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
        stickout_length_mm: 32.0,
        geometry: CutterGeometry::BallEndmill {
            diameter_mm,
            flute_length_mm: 24.0,
            overall_length_mm: 60.0,
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

fn push_box(tris: &mut Vec<[DVec3; 3]>, x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) {
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

/// Builds a closed solid consisting of:
/// - Base plate `[-26, 26] × [-26, 26] × [-4, 0]` (floor top at `Z = 0`),
/// - Central cylindrical hub (`R = 7.0 mm`, `Z ∈ [0, 11]`) with a genuine `R = 2.0 mm` concave
///   base fillet transitioning smoothly to `R = 9.0 mm` on the `Z = 0` floor,
/// - Wide overhanging T-flange blade (`X ∈ [-18, 18], Y ∈ [-4.5, 4.5], Z ∈ [11, 15]`) cantilevering
///   `11 mm` past the hub on `-X` and `+X` over an `11 mm` open air void above `Z = 0`.
fn build_overhang_t_flange_with_hub_fillet_stl() -> Vec<u8> {
    let mut tris: Vec<[DVec3; 3]> = Vec::new();
    // 1. Base plate (`Z ∈ [-4, 0]`)
    push_box(&mut tris, -26.0, -26.0, -4.0, 26.0, 26.0, 0.0);

    // 2. Cylindrical hub (`R = 7.0`) + concave base fillet (`R_fillet = 2.0`, `r ∈ [7.0, 9.0]`, `z ∈ [0.0, 2.0]`)
    let n_theta = 48usize;
    let n_fillet = 8usize;
    let r_hub = 7.0_f64;
    let r_fil = 2.0_f64;
    let z_hub_top = 11.0_f64;

    let mut profile: Vec<(f64, f64)> = Vec::new();
    for k in 0..=n_fillet {
        let a = (std::f64::consts::FRAC_PI_2) * (k as f64 / n_fillet as f64);
        // Concave quarter-circle centered at `(r_hub + r_fil, r_fil) = (9.0, 2.0)`:
        // `a = 0` -> `(9.0, 0.0)` on floor, `a = pi/2` -> `(7.0, 2.0)` on hub wall.
        let r = (r_hub + r_fil) - r_fil * a.sin();
        let z = r_fil - r_fil * a.cos();
        profile.push((r, z));
    }
    profile.push((r_hub, z_hub_top));

    let ring_pt = |r: f64, z: f64, it: usize| -> DVec3 {
        let th = std::f64::consts::TAU * (it as f64 / n_theta as f64);
        DVec3::new(r * th.cos(), r * th.sin(), z)
    };

    for w in profile.windows(2) {
        let (r0, z0) = w[0];
        let (r1, z1) = w[1];
        for it in 0..n_theta {
            let it_n = (it + 1) % n_theta;
            let a0 = ring_pt(r0, z0, it);
            let a1 = ring_pt(r0, z0, it_n);
            let b1 = ring_pt(r1, z1, it_n);
            let b0 = ring_pt(r1, z1, it);
            tris.push([a0, a1, b1]);
            tris.push([a0, b1, b0]);
        }
    }
    // Hub top and bottom caps (making hub closed)
    let hub_top_c = DVec3::new(0.0, 0.0, z_hub_top);
    let hub_bot_c = DVec3::new(0.0, 0.0, 0.0);
    for it in 0..n_theta {
        let it_n = (it + 1) % n_theta;
        tris.push([
            hub_top_c,
            ring_pt(r_hub, z_hub_top, it),
            ring_pt(r_hub, z_hub_top, it_n),
        ]);
        tris.push([
            hub_bot_c,
            ring_pt(r_hub + r_fil, 0.0, it_n),
            ring_pt(r_hub + r_fil, 0.0, it),
        ]);
    }

    // 3. Wide overhanging T-flange blade (`Z ∈ [11, 15]`, cantilevering to `X = ±18`)
    push_box(&mut tris, -18.0, -4.5, 11.0, 18.0, 4.5, 15.0);

    encode_test_stl(&tris)
}

/// Builds a closed non-overhanging vertical boss (`[-14, 14] × [-8, 8] × [0, 15]`) sitting solidly
/// on a `[-26, 26] × [-26, 26] × [-4, 0]` base plate (no air void underneath).
fn build_solid_vertical_boss_stl() -> Vec<u8> {
    let mut tris: Vec<[DVec3; 3]> = Vec::new();
    push_box(&mut tris, -26.0, -26.0, -4.0, 26.0, 26.0, 0.0);
    push_box(&mut tris, -14.0, -8.0, 0.0, 14.0, 8.0, 15.0);
    encode_test_stl(&tris)
}

/// Builds a watertight continuously twisted `90°` helical blade loft along `Z ∈ [0, 12]`:
/// at every height `z`, the cross-section is a smooth super-elliptical blade (`a = 18 mm, b = 6 mm`)
/// rotated continuously by `θ(z) = (π / 2) * (z / 12)`.
fn build_twisted_90deg_helical_blade_stl() -> Vec<u8> {
    let nz = 48usize;
    let n_theta = 64usize;
    let z_max = 12.0_f64;
    let a = 18.0_f64;
    let b = 6.0_f64;

    let section_pt = |z: f64, phi: f64| -> DVec3 {
        let twist = std::f64::consts::FRAC_PI_2 * (z / z_max);
        let lx = a * phi.cos();
        let ly = b * phi.sin();
        let (st, ct) = twist.sin_cos();
        DVec3::new(lx * ct - ly * st, lx * st + ly * ct, z)
    };

    let mut rings: Vec<Vec<DVec3>> = Vec::with_capacity(nz + 1);
    for iz in 0..=nz {
        let z = z_max * (iz as f64 / nz as f64);
        let mut ring = Vec::with_capacity(n_theta);
        for it in 0..n_theta {
            let phi = std::f64::consts::TAU * (it as f64 / n_theta as f64);
            ring.push(section_pt(z, phi));
        }
        rings.push(ring);
    }

    let mut tris: Vec<[DVec3; 3]> = Vec::new();
    for iz in 0..nz {
        for it in 0..n_theta {
            let it_n = (it + 1) % n_theta;
            let p00 = rings[iz][it];
            let p01 = rings[iz][it_n];
            let p11 = rings[iz + 1][it_n];
            let p10 = rings[iz + 1][it];
            tris.push([p00, p01, p11]);
            tris.push([p00, p11, p10]);
        }
    }
    let c_bot = DVec3::new(0.0, 0.0, 0.0);
    let c_top = DVec3::new(0.0, 0.0, z_max);
    for it in 0..n_theta {
        let it_n = (it + 1) % n_theta;
        tris.push([c_bot, rings[0][it_n], rings[0][it]]);
        tris.push([c_top, rings[nz][it], rings[nz][it_n]]);
    }

    encode_test_stl(&tris)
}

#[test]
fn overhang_t_flange_pencil3d_and_rest_mask_suppress_shadow_while_preserving_hub_fillet() {
    let stl = build_overhang_t_flange_with_hub_fillet_stl();
    let doc =
        load_mesh_cad_document("t_flange_fillet.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new(
        "setup_m74_pencil",
        "M74 Overhang Pencil",
        WorkOffsetSlot::G54,
    );
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27 {
            x: AxisAnchor::Center,
            y: AxisAnchor::Center,
            z: AxisAnchor::Min,
        },
    };
    // Shift origin Z so floor top is at Z = 4.0 (model min Z = 0, floor top = 4.0, flange top = 19.0)
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let surface = SurfaceMesh3D::from_cad_document(&doc, &[], &resolved.frame);

    let ball_3 = make_ball_tool("t_ball_3", 11, 3.0);
    let cutter = SurfaceCutterProfile::from_tool_geometry(&ball_3.geometry, 0.0, 0.0);
    let containment = rect_region_2d(-24.0, -24.0, 24.0, 24.0);

    // 1. Verify `build_3d_rest_mask`: active on genuine concave hub fillet, silent on overhang shadow
    let rest_cfg = RestMachiningConfig {
        mode: RestMachiningMode::FromPreviousTool {
            prev_tool_diameter: ParamLength::from_mm(8.0),
            prev_stock_to_leave: ParamLength::from_mm(0.0),
        },
        rest_overlap: ParamLength::from_mm(0.2),
        min_rest_thickness: ParamLength::from_mm(0.08),
    };
    let mask = build_3d_rest_mask(&surface, &cutter, &containment, &rest_cfg, &[], 0.0)
        .expect("3D rest mask should be generated");

    // Genuine concave hub base fillet at `(0, ±9.0)` (away from the X-oriented flange overhang)
    let fillet_hits = [
        DVec2::new(0.0, 8.8),
        DVec2::new(0.0, 9.5),
        DVec2::new(0.0, -8.8),
        DVec2::new(0.0, -9.5),
    ]
    .into_iter()
    .filter(|&p| mask.contains_xy(p))
    .count();
    assert!(
        fillet_hits >= 2,
        "build_3d_rest_mask must remain active on the genuine concave hub base fillet, hits={fillet_hits}"
    );

    // Overhanging T-flange shadow zone just outside `X = ±18.0` (`|X| ∈ [19.6, 22.0], Y ∈ [-3.0, 3.0]`):
    // without overhang suppression, a Ø8 ref ball vs Ø3 ball creates a 2.5 mm tall false residual band here!
    for &shadow_pt in &[
        DVec2::new(20.0, 0.0),
        DVec2::new(21.0, 0.0),
        DVec2::new(-20.0, 0.0),
        DVec2::new(-21.0, 0.0),
        DVec2::new(20.0, 2.0),
        DVec2::new(-20.0, -2.0),
    ] {
        assert!(
            !mask.contains_xy(shadow_pt),
            "build_3d_rest_mask must suppress overhang shadow at {shadow_pt:?}"
        );
    }

    // 2. Verify `Pencil3D`: traces the genuine concave hub fillet and emits 0 spurious shadow passes
    let mut op = Operation::new(
        "op_pencil_m74",
        "Pencil3D Overhang",
        OperationKind::Pencil3D,
    );
    op.tool_id = Some(ball_3.id.clone());
    op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
    if let StrategyParams::Pencil3D(p) = op.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.boundary.tool_containment = ToolContainmentMode::ToolInsideBoundary;
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

    let mut fillet_cut_pts = 0usize;
    let mut spurious_shadow_pts = 0usize;
    for seg in &tp.segments_wcs {
        if matches!(
            seg.motion,
            SegmentMotion3D::Linear {
                role: MotionRole::FinishCut,
                ..
            }
        ) {
            for pt in [seg.start_wcs, seg.end_wcs] {
                let r_xy = pt.x.hypot(pt.y);
                // Genuine concave hub base fillet lies near `r ∈ [7.5, 11.5]` and `|y| > 5.5`
                if (7.5..=11.5).contains(&r_xy) && pt.y.abs() > 5.5 {
                    fillet_cut_pts += 1;
                }
                // Overhang blade shadow zone: outside `|x| > 15.0`, near the cantilevered flange
                if pt.x.abs() > 15.0 && pt.y.abs() < 7.5 {
                    spurious_shadow_pts += 1;
                }
            }
        }
    }

    assert!(
        fillet_cut_pts >= 8,
        "Pencil3D must trace the genuine concave hub base fillet, got only {fillet_cut_pts} points"
    );
    assert_eq!(
        spurious_shadow_pts, 0,
        "Pencil3D must emit 0 spurious passes along the overhanging T-flange shadow edge, got {spurious_shadow_pts}"
    );
}

#[test]
fn solid_vertical_boss_preserves_rest_mask_pencil_and_continuous_wall_chains() {
    let stl = build_solid_vertical_boss_stl();
    let doc =
        load_mesh_cad_document("solid_boss.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new("setup_solid_boss", "Solid Boss", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27 {
            x: AxisAnchor::Center,
            y: AxisAnchor::Center,
            z: AxisAnchor::Min,
        },
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let surface = SurfaceMesh3D::from_cad_document(&doc, &[], &resolved.frame);

    let ball_3 = make_ball_tool("t_ball_3", 11, 3.0);
    let cutter = SurfaceCutterProfile::from_tool_geometry(&ball_3.geometry, 0.0, 0.0);
    let containment = rect_region_2d(-24.0, -24.0, 24.0, 24.0);

    // Genuine solid vertical wall corner (`x = 14 + 1.5 = 15.5`, `y = 0`) MUST stay active in rest mask!
    let rest_cfg = RestMachiningConfig {
        mode: RestMachiningMode::FromPreviousTool {
            prev_tool_diameter: ParamLength::from_mm(8.0),
            prev_stock_to_leave: ParamLength::from_mm(0.0),
        },
        rest_overlap: ParamLength::from_mm(0.2),
        min_rest_thickness: ParamLength::from_mm(0.08),
    };
    let mask = build_3d_rest_mask(&surface, &cutter, &containment, &rest_cfg, &[], 0.0)
        .expect("3D rest mask should be generated on solid boss");
    assert!(
        mask.contains_xy(DVec2::new(15.6, 0.0)) || mask.contains_xy(DVec2::new(16.0, 0.0)),
        "Genuine solid vertical-wall-to-floor corner must NOT be suppressed in 3D rest mask"
    );

    // `split_chains_at_overhang_cliffs` must keep a steep chain on a non-overhanging boss intact!
    let continuous_chain = vec![
        DVec3::new(13.0, 0.0, 19.0),
        DVec3::new(15.4, 0.0, 18.0),
        DVec3::new(15.6, 0.0, 4.0),
        DVec3::new(18.0, 0.0, 4.0),
    ];
    let split = split_chains_at_overhang_cliffs(&surface, &cutter, continuous_chain);
    assert_eq!(
        split.len(),
        1,
        "Non-overhanging solid boss wall must NOT be split by overhang cliff splitter"
    );
}

#[test]
fn parallel3d_and_scallop3d_on_overhanging_flange_zero_gouge_zero_crash_no_cliff_dives() {
    let stl = build_overhang_t_flange_with_hub_fillet_stl();
    let doc =
        load_mesh_cad_document("t_flange_finish.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new(
        "setup_m74_finish",
        "M74 Overhang Finish",
        WorkOffsetSlot::G54,
    );
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(2.0),
        side_neg_x: ParamLength::from_mm(2.0),
        side_pos_y: ParamLength::from_mm(2.0),
        side_neg_y: ParamLength::from_mm(2.0),
        top_offset_z: ParamLength::from_mm(0.5),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(20.0),
        round_increment_mm: None,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let ball_4 = make_ball_tool("t_ball_4", 4, 4.0);
    let cancel = AtomicBool::new(false);

    // 1. Parallel3D (ZigZag 0° across the X overhang cliff + 90° cross-pass)
    let mut op_par = Operation::new(
        "op_par_m74",
        "Parallel3D Overhang",
        OperationKind::Parallel3D,
    );
    op_par.tool_id = Some(ball_4.id.clone());
    op_par.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 4.0);
    if let StrategyParams::Parallel3D(p) = op_par.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.boundary.tool_containment = ToolContainmentMode::ToolInsideBoundary;
        p.use_cusp_height = false;
        p.stepover = ParamLength::from_mm(1.5);
        p.pass_angle_deg = 0.0;
        p.cross_pass = true;
        p.cut_direction = ParallelCutDirection::ZigZag;
    }

    // 2. Scallop3D across the overhanging flange
    let mut op_sc = Operation::new("op_sc_m74", "Scallop3D Overhang", OperationKind::Scallop3D);
    op_sc.tool_id = Some(ball_4.id.clone());
    op_sc.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 4.0);
    if let StrategyParams::Scallop3D(p) = op_sc.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.boundary.tool_containment = ToolContainmentMode::ToolInsideBoundary;
        p.cusp_height = ParamLength::from_mm(0.10);
        p.max_stepover = ParamLength::from_mm(1.5);
        p.min_stepover = ParamLength::from_mm(0.4);
        p.inside_out = false;
        p.climb_milling = true;
    }

    let tp_par = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_par,
        tool: &ball_4,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
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
        tool: &ball_4,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    // Assert NO full-cut-feed near-vertical dives across overhang drop-offs in either strategy!
    const TAN_70: f64 = 2.747_477_419_454_622;
    let max_drop_allowed = 2.0_f64; // max(1.5, 0.5 * 4.0)
    let surface = SurfaceMesh3D::from_cad_document(&doc, &[], &resolved.frame);
    let cutter = SurfaceCutterProfile::from_tool_geometry(&ball_4.geometry, 0.0, 0.0);
    let mut cand_buf = Vec::new();
    let mut ray_buf = Vec::new();
    for (label, tp) in [("Parallel3D", &tp_par), ("Scallop3D", &tp_sc)] {
        let mut overhang_cliff_dives = 0usize;
        let mut plunge_count = 0usize;
        for seg in &tp.segments_wcs {
            match seg.motion {
                SegmentMotion3D::Linear {
                    role: MotionRole::FinishCut,
                    ..
                } => {
                    let dz_drop = seg.start_wcs.z - seg.end_wcs.z;
                    let dxy =
                        (seg.end_wcs.x - seg.start_wcs.x).hypot(seg.end_wcs.y - seg.start_wcs.y);
                    if dz_drop > max_drop_allowed && dz_drop > TAN_70 * dxy {
                        let is_oh = chip_cam::surface3d::finishing::is_overhang_shadow_transition(
                            &surface,
                            &cutter,
                            seg.start_wcs,
                            seg.end_wcs,
                            &mut cand_buf,
                            &mut ray_buf,
                        );
                        // Also check directly by known geometry: cantilevered flange wings lie at |x| > 7.5
                        let in_wing = seg.start_wcs.x.abs() > 7.5 || seg.end_wcs.x.abs() > 7.5;
                        if is_oh || in_wing {
                            overhang_cliff_dives += 1;
                        }
                    }
                }
                SegmentMotion3D::Linear {
                    role: MotionRole::Plunge,
                    ..
                } => {
                    plunge_count += 1;
                }
                _ => {}
            }
        }
        assert_eq!(
            overhang_cliff_dives, 0,
            "{label} must split at overhang cliffs instead of diving near-vertically at full cut_feed (found {overhang_cliff_dives} overhang dives)"
        );
        assert!(
            plunge_count >= 2,
            "{label} should emit safe Plunge re-entries after overhang cliff splits (got {plunge_count})"
        );
    }

    // Verify both strategies against chip-sim's volumetric Tri-Dexel oracle: 0 crashes, <= 0.015 mm gouge!
    let sim = run_setup_simulation_with_choice(
        &SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[
                SimOperationInput::from_operation(&op_par, &ball_4, &tp_par),
                SimOperationInput::from_operation(&op_sc, &ball_4, &tp_sc),
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
        "Parallel3D + Scallop3D on overhanging flange must have 0 rapid crashes / critical errors, got: {:#?}",
        sim.diagnostics
    );
    let err = sim.error_report.as_ref().expect("Tri-Dexel error_report");
    assert!(
        err.max_gouge_mm <= 0.015,
        "Parallel3D + Scallop3D on overhanging flange max_gouge_mm must be <= 0.015 mm, got {:.4} mm",
        err.max_gouge_mm
    );
}

#[test]
fn soft_jaw_wizard_90deg_twisted_helical_loft_zero_scalloping_interference() {
    let stl = build_twisted_90deg_helical_blade_stl();
    let doc = load_mesh_cad_document(
        "twisted_90deg_helix.stl",
        &stl,
        &MeshImportOptions::default(),
    )
    .unwrap();

    let mut setup = Setup::new("setup_helical_jaw", "Helical Soft Jaw", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27 {
            x: AxisAnchor::Center,
            y: AxisAnchor::Center,
            z: AxisAnchor::Min,
        },
    };
    let grip_depth_mm = 10.0;
    let cfg = SoftJawConfig {
        jaw_width_mm: 80.0,
        jaw_depth_mm: 30.0,
        jaw_height_mm: 36.0,
        jaw_gap_mm: 4.0,
        grip_depth_mm,
        part_clearance_mm: 0.05,
        corner_relief_radius_mm: 0.0,
        mounting_bolt_pattern: SoftJawBoltPattern::CarveraModVise,
        output_mode: SoftJawOutputMode::Both,
        z_band_bottom_offset_mm: 0.0,
        ..SoftJawConfig::default()
    };

    let plan = build_soft_jaw_plan(&doc, &setup, &[], DisplayUnit::Millimeter, &cfg).unwrap();

    // Sample 101 continuous Z heights inside `[0, grip_depth_mm]` (including every intermediate
    // phase between coarse 9-slice levels) around the entire twisted helical blade perimeter.
    let z_max_part = 12.0_f64;
    let a = 18.0_f64;
    let b = 6.0_f64;
    let mut outside_sil_count = 0usize;
    let mut outside_jaw_pocket_count = 0usize;
    let mut max_miss_mm = 0.0_f64;
    let half_gap = 0.5 * cfg.jaw_gap_mm;

    for iz in 0..=100 {
        let z = grip_depth_mm * (iz as f64 / 100.0);
        let twist = std::f64::consts::FRAC_PI_2 * (z / z_max_part);
        let (st, ct) = twist.sin_cos();
        for it in 0..64 {
            let phi = std::f64::consts::TAU * (it as f64 / 64.0);
            // Slightly inward by 0.002 mm to test exact interior of the faceted 64-gon section
            let lx = (a - 0.002) * phi.cos();
            let ly = (b - 0.002) * phi.sin();
            let pt = DVec2::new(lx * ct - ly * st, lx * st + ly * ct);

            if !plan.part_silhouette_2d.contains_point(pt) {
                let d = plan.part_silhouette_2d.min_distance_to_boundary(pt);
                if d > 0.01 {
                    outside_sil_count += 1;
                    max_miss_mm = max_miss_mm.max(d);
                }
            }

            // Inside the fixed jaw (`y > half_gap + 0.05`) or movable jaw (`y < -half_gap - 0.05`),
            // every point of the twisted helical blade MUST lie strictly inside the jaw pocket!
            if pt.y > half_gap + 0.05 && !plan.fixed_pocket_2d.contains_point(pt) {
                outside_jaw_pocket_count += 1;
            }
            if pt.y < -(half_gap + 0.05) && !plan.movable_pocket_2d.contains_point(pt) {
                outside_jaw_pocket_count += 1;
            }
        }
    }

    assert_eq!(
        outside_sil_count, 0,
        "Continuous swept helical silhouette must contain every twisted blade point in [0, {grip_depth_mm}] (missed {outside_sil_count} pts, max_miss={max_miss_mm:.4} mm)"
    );
    assert_eq!(
        outside_jaw_pocket_count, 0,
        "Generated soft-jaw pockets must have zero scalloping interference with the 90° twisted helical loft (missed {outside_jaw_pocket_count} pts)"
    );

    // Verify both generated STL jaws are 100% watertight
    for (label, bytes) in [
        ("helical_fixed.stl", plan.fixed_jaw_stl.as_ref().unwrap()),
        (
            "helical_movable.stl",
            plan.movable_jaw_stl.as_ref().unwrap(),
        ),
    ] {
        let reloaded = load_mesh_bytes(label, bytes, &MeshImportOptions::default()).unwrap();
        assert!(
            reloaded.diagnostics.is_watertight,
            "{label} must be watertight: {:?}",
            reloaded.diagnostics
        );
        assert_eq!(reloaded.diagnostics.open_boundary_edges, 0);
        assert_eq!(reloaded.diagnostics.non_manifold_edges, 0);
    }
}

#[test]
fn parallel3d_and_scallop3d_on_twisted_overhanging_helical_loft_zero_gouge_zero_crash() {
    let stl = build_twisted_90deg_helical_blade_stl();
    let doc = load_mesh_cad_document(
        "twisted_90deg_helix_finish.stl",
        &stl,
        &MeshImportOptions::default(),
    )
    .unwrap();

    let mut setup = Setup::new(
        "setup_helix_finish",
        "Helical Loft Finish",
        WorkOffsetSlot::G54,
    );
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(3.0),
        side_neg_x: ParamLength::from_mm(3.0),
        side_pos_y: ParamLength::from_mm(3.0),
        side_neg_y: ParamLength::from_mm(3.0),
        top_offset_z: ParamLength::from_mm(0.5),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(13.0),
        round_increment_mm: None,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let ball_4 = make_ball_tool("t_ball_4_helix", 4, 4.0);
    let cancel = AtomicBool::new(false);

    let mut op_par = Operation::new(
        "op_par_helix",
        "Parallel3D Helix",
        OperationKind::Parallel3D,
    );
    op_par.tool_id = Some(ball_4.id.clone());
    op_par.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
    if let StrategyParams::Parallel3D(p) = op_par.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.boundary.tool_containment = ToolContainmentMode::ToolCenterOnBoundary;
        p.use_cusp_height = false;
        p.stepover = ParamLength::from_mm(2.0);
        p.pass_angle_deg = 0.0;
        p.cross_pass = false;
        p.cut_direction = ParallelCutDirection::ZigZag;
    }

    let mut op_sc = Operation::new("op_sc_helix", "Scallop3D Helix", OperationKind::Scallop3D);
    op_sc.tool_id = Some(ball_4.id.clone());
    op_sc.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
    if let StrategyParams::Scallop3D(p) = op_sc.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.boundary.tool_containment = ToolContainmentMode::ToolCenterOnBoundary;
        p.cusp_height = ParamLength::from_mm(0.15);
        p.max_stepover = ParamLength::from_mm(2.0);
        p.min_stepover = ParamLength::from_mm(0.8);
        p.inside_out = false;
        p.climb_milling = true;
    }

    let tp_par = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_par,
        tool: &ball_4,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
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
        tool: &ball_4,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        cancel: &cancel,
    })
    .unwrap();

    const TAN_70: f64 = 2.747_477_419_454_622;
    let surface = SurfaceMesh3D::from_cad_document(&doc, &[], &resolved.frame);
    let cutter = SurfaceCutterProfile::from_tool_geometry(&ball_4.geometry, 0.0, 0.0);
    let mut cand_buf = Vec::new();
    let mut ray_buf = Vec::new();
    for (label, tp) in [("Parallel3D Helix", &tp_par), ("Scallop3D Helix", &tp_sc)] {
        let mut overhang_dives = 0usize;
        for seg in &tp.segments_wcs {
            if matches!(
                seg.motion,
                SegmentMotion3D::Linear {
                    role: MotionRole::FinishCut,
                    ..
                }
            ) {
                let dz_drop = seg.start_wcs.z - seg.end_wcs.z;
                let dxy = (seg.end_wcs.x - seg.start_wcs.x).hypot(seg.end_wcs.y - seg.start_wcs.y);
                if dz_drop > 2.0
                    && dz_drop > TAN_70 * dxy
                    && chip_cam::surface3d::finishing::is_overhang_shadow_transition(
                        &surface,
                        &cutter,
                        seg.start_wcs,
                        seg.end_wcs,
                        &mut cand_buf,
                        &mut ray_buf,
                    )
                {
                    overhang_dives += 1;
                }
            }
        }
        assert_eq!(
            overhang_dives, 0,
            "{label} must have 0 full-cut-feed dives across helical overhang drop-offs"
        );
    }

    let sim = run_setup_simulation_with_choice(
        &SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[
                SimOperationInput::from_operation(&op_par, &ball_4, &tp_par),
                SimOperationInput::from_operation(&op_sc, &ball_4, &tp_sc),
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
        "Parallel3D + Scallop3D on twisted overhanging helical loft must have 0 critical errors, got: {:#?}",
        sim.diagnostics
    );
    let err = sim.error_report.as_ref().expect("Tri-Dexel error_report");
    assert!(
        err.max_gouge_mm <= 0.015,
        "Parallel3D + Scallop3D on twisted overhanging helical loft max_gouge_mm must be <= 0.015 mm, got {:.4} mm",
        err.max_gouge_mm
    );
}
