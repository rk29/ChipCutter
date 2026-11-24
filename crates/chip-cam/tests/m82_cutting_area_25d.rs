#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::sync::atomic::AtomicBool;

use chip_cad::{CadDocument, MeshImportOptions, load_mesh_cad_document};
use chip_cam::{
    AdaptiveClearingParams, BoxAnchor27, Contour2DParams, ContourCompensation, FacingParams,
    FixedBoxXyAlign, FixedBoxZAlign, GeometrySelectionItem, HeightReference, HeightSpec,
    MachiningBoundaryMode, MotionRole, Operation, OperationGenRequest, OperationKind,
    Pocket2DParams, PocketBoundaryMode, PocketStrategyKind, SegmentMotion3D, Setup, StockConfig,
    StrategyParams, ToolContainmentMode, ToolpathSegment3D, WcsOriginMode, WorkOffsetSlot,
    generate_operation_toolpath_with_rest,
};
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3, DisplayUnit, ParamLength};
use chip_sim::{
    MachineKinematicsConfig, SetupSimulationInput, SimComputeBackendChoice, SimEngineKind,
    SimOperationInput, SimResolution, SimVerificationConfig, run_setup_simulation_with_choice,
};
use chip_sketch::{RectAnchorMode, SketchEntity, SketchEntityEntry, SketchLayer};
use chip_tools::{CutterGeometry, Tool, ToolId, default_starter_tool_library};

fn seg_role(seg: &ToolpathSegment3D) -> Option<MotionRole> {
    match seg.motion {
        SegmentMotion3D::Rapid => None,
        SegmentMotion3D::Linear { role, .. } | SegmentMotion3D::Arc { role, .. } => Some(role),
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
        [0, 3, 2, 1], // -Z
        [4, 5, 6, 7], // +Z
        [0, 1, 5, 4], // -Y
        [2, 3, 7, 6], // +Y
        [0, 4, 7, 3], // -X
        [1, 2, 6, 5], // +X
    ];
    for [a, b, c, d] in quads {
        tris.push([v[a], v[b], v[c]]);
        tris.push([v[a], v[c], v[d]]);
    }
}

fn push_cylinder_tris(
    tris: &mut Vec<[DVec3; 3]>,
    cx: f64,
    cy: f64,
    radius: f64,
    z0: f64,
    z1: f64,
    segments: usize,
) {
    let bot_c = DVec3::new(cx, cy, z0);
    let top_c = DVec3::new(cx, cy, z1);
    for i in 0..segments {
        let a0 = std::f64::consts::TAU * (i as f64) / (segments as f64);
        let a1 = std::f64::consts::TAU * ((i + 1) as f64) / (segments as f64);
        let b0 = DVec3::new(cx + radius * a0.cos(), cy + radius * a0.sin(), z0);
        let b1 = DVec3::new(cx + radius * a1.cos(), cy + radius * a1.sin(), z0);
        let t0 = DVec3::new(b0.x, b0.y, z1);
        let t1 = DVec3::new(b1.x, b1.y, z1);
        tris.push([bot_c, b1, b0]);
        tris.push([top_c, t0, t1]);
        tris.push([b0, b1, t1]);
        tris.push([b0, t1, t0]);
    }
}

fn flat_endmill(diameter_mm: f64) -> Tool {
    let lib = default_starter_tool_library();
    let mut tool = lib
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { .. }))
        .unwrap()
        .clone();
    tool.id = ToolId(format!("flat_{diameter_mm:.1}mm"));
    tool.name = format!("Ø{diameter_mm:.1}mm Flat Endmill");
    tool.geometry = CutterGeometry::FlatEndmill {
        diameter_mm,
        flute_length_mm: 24.0,
        overall_length_mm: 60.0,
        shank_diameter_mm: diameter_mm,
        flutes: 3,
    };
    tool.stickout_length_mm = 30.0;
    tool
}

fn make_l_shape_with_boss_sketch(setup_id: &str) -> SketchLayer {
    let mut sk = SketchLayer::new("sk_l_boss", "L-Shape + Raised Boss", setup_id);
    // L-shape outer boundary `[0..80] x [0..40] U [0..40] x [40..80]`
    let pts = [
        DVec2::new(0.0, 0.0),
        DVec2::new(80.0, 0.0),
        DVec2::new(80.0, 40.0),
        DVec2::new(40.0, 40.0),
        DVec2::new(40.0, 80.0),
        DVec2::new(0.0, 80.0),
    ];
    let mut segs = Vec::with_capacity(pts.len());
    for i in 0..pts.len() {
        segs.push(ArcSegment2D::Line {
            start: pts[i],
            end: pts[(i + 1) % pts.len()],
        });
    }
    sk.entities.push(SketchEntityEntry {
        id: "e_l_outer".to_string(),
        name: "L Outer".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::new_polyline(ArcPolyline2D {
            segments: segs,
            closed: true,
        }),
    });
    // Raised central boss `[14..26] x [14..26]` (center `(20, 20)`, size `12 x 12`)
    sk.entities.push(SketchEntityEntry {
        id: "e_boss".to_string(),
        name: "Raised Boss".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(20.0),
            y: ParamLength::from_mm(20.0),
            width: ParamLength::from_mm(12.0),
            height: ParamLength::from_mm(12.0),
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
        },
    });
    sk.rebuild_regions(None);
    sk
}

#[test]
fn facing_l_shape_with_raised_boss_clips_missing_quadrant_and_avoids_boss_gouges_in_sim() {
    // Watertight 3D solid matching the L-shape deck (`Z = 0..10`) + raised central boss (`Z = 10..16`)
    let mut tris = Vec::new();
    push_box_tris(&mut tris, 0.0, 0.0, 0.0, 80.0, 40.0, 10.0);
    push_box_tris(&mut tris, 0.0, 40.0, 0.0, 40.0, 80.0, 10.0);
    push_box_tris(&mut tris, 14.0, 14.0, 10.0, 26.0, 26.0, 16.0);
    let stl = encode_test_stl(&tris);
    let doc = load_mesh_cad_document("l_boss.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new("s_l_boss", "L-Boss Setup", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
    };
    setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(80.0),
        depth_y: ParamLength::from_mm(80.0),
        height_z: ParamLength::from_mm(16.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::OffsetFromBottom {
            bottom_allowance: ParamLength::from_mm(0.0),
        },
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let sk = make_l_shape_with_boss_sketch(&setup.id);
    let l_region = sk
        .regions
        .iter()
        .find(|r| !r.islands_cw.is_empty())
        .expect("L-shape sketch must detect outer L region with inner boss island");

    let tool = flat_endmill(6.0);
    let mut op = Operation::new(
        "op_face_l",
        "Face L-Deck Around Boss",
        OperationKind::Facing,
    );
    op.tool_id = Some(tool.id.clone());
    op.selections.push(GeometrySelectionItem::SketchRegion {
        sketch_id: sk.id.clone(),
        region_id: l_region.id.clone(),
    });
    op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 10.0);
    op.passes.multiple_depths = true;
    op.passes.max_stepdown = ParamLength::from_mm(2.0);
    op.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    op.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
    if let StrategyParams::Facing(p) = op.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::SelectedRegions;
        p.boundary.tool_containment = ToolContainmentMode::ToolOutsideBoundary;
        p.boundary.boundary_offset = ParamLength::from_mm(0.0);
        p.stepover = ParamLength::from_mm(4.0);
        p.pass_extension = ParamLength::from_mm(2.0);
        p.finishing_stepdown = Some(ParamLength::from_mm(0.4));
    }

    let cancel = AtomicBool::new(false);
    let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[sk],
        setup: &resolved,
        op: &op,
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
        tp.segments_wcs
            .iter()
            .any(|s| seg_role(s) == Some(MotionRole::FinishCut)),
        "finishing_stepdown must emit FinishCut segments"
    );

    // 1. Verify ZERO cutting motion enters the missing quadrant `(x > 46.5, y > 46.5)`
    //    and ZERO cutting motion penetrates the raised boss keep-out `[11.05..28.95]^2`!
    for seg in &tp.segments_wcs {
        if matches!(seg.motion, SegmentMotion3D::Rapid) {
            continue;
        }
        if seg.start_wcs.z > 15.9 && seg.end_wcs.z > 15.9 {
            continue;
        }
        for k in 0..=16 {
            let t = f64::from(k) / 16.0;
            let p = seg.sample_point(t);
            assert!(
                !(p.x > 47.5 && p.y > 47.5),
                "Facing pass intruded into missing quadrant at ({:.2}, {:.2}, {:.2})",
                p.x,
                p.y,
                p.z
            );
            assert!(
                !(p.x > 11.05 && p.x < 28.95 && p.y > 11.05 && p.y < 28.95),
                "Facing pass gouged raised boss keep-out at ({:.2}, {:.2}, {:.2})",
                p.x,
                p.y,
                p.z
            );
        }
    }

    // 2. Verify with `chip-sim` Heightmap oracle: zero rapid crashes, zero target gouges,
    //    boss intact at Z=16, L-shape arms cleanly cleared down to Z=10!
    let sim_ops = vec![SimOperationInput::from_operation(&op, &tool, &tp)];
    let sim = run_setup_simulation_with_choice(
        &SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &sim_ops,
            kinematics: MachineKinematicsConfig::default(),
            resolution: SimResolution::Fine,
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

    assert_eq!(
        sim.critical_error_count(),
        0,
        "L-shape + boss Facing must have zero simulation errors, got: {:#?}",
        sim.diagnostics
    );
    // Boss top stays intact at Z = 16 mm
    let z_boss = sim.final_stock.sample_stock_z(20.0, 20.0).unwrap();
    assert!(
        z_boss >= 15.9,
        "Raised boss center must remain uncut at Z=16, got {z_boss:.3}"
    );
    // Both L-shape arms are cleanly cleared down to Z = 10 mm
    for (pt_name, x, y) in [
        ("lower-right arm", 65.0, 20.0),
        ("upper-left arm", 20.0, 65.0),
        ("lower-left corner", 5.0, 5.0),
    ] {
        let z_deck = sim.final_stock.sample_stock_z(x, y).unwrap();
        assert!(
            (z_deck - 10.0).abs() <= 0.15,
            "L-shape {pt_name} ({x}, {y}) expected cleared to Z=10.0, got {z_deck:.3}"
        );
    }
    // Missing quadrant center `(65, 65)` remains untouched at stock top Z = 16 mm!
    let z_missing = sim.final_stock.sample_stock_z(65.0, 65.0).unwrap();
    assert!(
        z_missing >= 15.9,
        "Missing quadrant (65, 65) must remain completely untouched, got {z_missing:.3}"
    );
}

#[test]
fn facing_model_silhouette_and_stock_box_containment_modes_and_offset() {
    // Circular puck `R = 25 mm`, `Z = 0..10` inside `60 x 60 x 12` stock box (`[-30..30]^2`)
    let mut tris = Vec::new();
    push_cylinder_tris(&mut tris, 0.0, 0.0, 25.0, 0.0, 10.0, 72);
    let stl = encode_test_stl(&tris);
    let doc = load_mesh_cad_document("puck.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new("s_puck", "Puck Setup", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(60.0),
        depth_y: ParamLength::from_mm(60.0),
        height_z: ParamLength::from_mm(12.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::OffsetFromBottom {
            bottom_allowance: ParamLength::from_mm(0.0),
        },
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let tool = flat_endmill(8.0); // R = 4.0 mm
    let cancel = AtomicBool::new(false);

    let run_face =
        |mode: MachiningBoundaryMode, containment: ToolContainmentMode, offset_mm: f64| {
            let mut op = Operation::new("op_face", "Face Puck", OperationKind::Facing);
            op.tool_id = Some(tool.id.clone());
            op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
            op.heights.bottom = HeightSpec::new(HeightReference::ModelTop, 0.0);
            op.passes.multiple_depths = false;
            op.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
            if let StrategyParams::Facing(p) = op.strategy_mut() {
                p.boundary.boundary_mode = mode;
                p.boundary.tool_containment = containment;
                p.boundary.boundary_offset = ParamLength::from_mm(offset_mm);
                p.stepover = ParamLength::from_mm(4.0);
                p.pass_extension = ParamLength::from_mm(2.0);
            }
            generate_operation_toolpath_with_rest(&OperationGenRequest {
                doc: &doc,
                sketches: &[],
                setup: &resolved,
                op: &op,
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
            .unwrap()
        };

    // 1. ModelSilhouette + ToolCenterOnBoundary: tool center stays inside circular R=25 silhouette
    //    (zero cutting motion in square stock corners `(±24, ±24)` where `r = 33.9 > 25`!)
    let tp_sil = run_face(
        MachiningBoundaryMode::ModelSilhouette,
        ToolContainmentMode::ToolCenterOnBoundary,
        0.0,
    );
    let mut max_r_sil = 0.0_f64;
    for seg in &tp_sil.segments_wcs {
        if seg_role(seg) != Some(MotionRole::Cut) {
            continue;
        }
        for pt in [seg.start_wcs, seg.end_wcs] {
            max_r_sil = max_r_sil.max(pt.x.hypot(pt.y));
        }
    }
    assert!(
        max_r_sil > 20.0 && max_r_sil <= 25.35,
        "ModelSilhouette ToolCenterOnBoundary max radius expected in (20, 25.35], got {max_r_sil:.3}"
    );

    // 2. StockBox + ToolInsideBoundary + boundary_offset = -2.0 mm:
    //    Stock is `[-30..30]^2`, offset -2 -> `[-28..28]^2`, inside by R=4 -> tool center stays inside `[-24..24]^2`!
    let tp_inside = run_face(
        MachiningBoundaryMode::StockBox,
        ToolContainmentMode::ToolInsideBoundary,
        -2.0,
    );
    let mut max_abs_x = 0.0_f64;
    let mut max_abs_y = 0.0_f64;
    for seg in &tp_inside.segments_wcs {
        if seg_role(seg) != Some(MotionRole::Cut) {
            continue;
        }
        for pt in [seg.start_wcs, seg.end_wcs] {
            max_abs_x = max_abs_x.max(pt.x.abs());
            max_abs_y = max_abs_y.max(pt.y.abs());
        }
    }
    assert!(
        max_abs_x <= 24.05 && max_abs_y <= 24.05 && max_abs_x > 23.0,
        "StockBox ToolInsideBoundary (-2mm) expected tool centers within [-24..24], got max=({max_abs_x:.3}, {max_abs_y:.3})"
    );

    // 3. SelectedRegions with empty selections emits diagnostic warning and falls back to StockBox
    let tp_warn = run_face(
        MachiningBoundaryMode::SelectedRegions,
        ToolContainmentMode::ToolOutsideBoundary,
        0.0,
    );
    assert!(
        tp_warn
            .warnings
            .iter()
            .any(|w| w.contains("Selected 2D Contours")),
        "Empty SelectedRegions on Facing must emit fallback warning, got: {:?}",
        tp_warn.warnings
    );
}

#[test]
fn pocket2d_and_adaptive2d_clear_outside_selected_boss_to_stock_gouge_free() {
    // Circular boss `R = 12 mm` centered at `(30, 30)` rising from `Z = 7.5` to `Z = 16`
    // on a `60 x 60 x 7.5` baseplate inside `[0..60] x [0..60] x [0..16]` stock.
    let deck_z = 7.5_f64;
    let mut tris = Vec::new();
    push_box_tris(&mut tris, 0.0, 0.0, 0.0, 60.0, 60.0, deck_z);
    push_cylinder_tris(&mut tris, 30.0, 30.0, 12.0, deck_z, 16.0, 72);
    let stl = encode_test_stl(&tris);
    let doc = load_mesh_cad_document("boss_part.stl", &stl, &MeshImportOptions::default()).unwrap();

    let mut setup = Setup::new("s_boss", "Boss Clear Setup", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
    };
    setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(60.0),
        depth_y: ParamLength::from_mm(60.0),
        height_z: ParamLength::from_mm(16.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::OffsetFromBottom {
            bottom_allowance: ParamLength::from_mm(0.0),
        },
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    // Sketch containing ONLY the circular boss outline (no manual stock rectangle!)
    let mut sk = SketchLayer::new("sk_boss", "Boss Outline", &setup.id);
    sk.entities.push(SketchEntityEntry {
        id: "e_boss_circle".to_string(),
        name: "Boss Circle".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::new_circle(
            ParamLength::from_mm(30.0),
            ParamLength::from_mm(30.0),
            ParamLength::from_mm(24.0),
        ),
    });
    sk.rebuild_regions(None);
    let boss_reg = sk.regions.first().unwrap().clone();

    let tool = flat_endmill(6.0);
    let cancel = AtomicBool::new(false);

    for (label, strategy) in [
        ("ContourParallel", PocketStrategyKind::ContourParallel),
        (
            "Adaptive2D",
            PocketStrategyKind::Adaptive(AdaptiveClearingParams {
                optimal_load: ParamLength::from_mm(1.5),
                ..AdaptiveClearingParams::default()
            }),
        ),
    ] {
        let mut op = Operation::new(
            format!("op_boss_{label}"),
            format!("Clear Outside Boss ({label})"),
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(tool.id.clone());
        op.selections.push(GeometrySelectionItem::SketchRegion {
            sketch_id: sk.id.clone(),
            region_id: boss_reg.id.clone(),
        });
        op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, deck_z);
        op.passes.multiple_depths = true;
        op.passes.max_stepdown = ParamLength::from_mm(4.25);
        op.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        if let StrategyParams::Pocket2D(p) = op.strategy_mut() {
            p.boundary_mode = PocketBoundaryMode::ClearOutsideSelectedToStock;
            p.strategy = strategy;
            p.stepover = ParamLength::from_mm(2.4);
            p.finish_wall_pass = true;
        }

        let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk),
            setup: &resolved,
            op: &op,
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

        // First plunge must occur strictly in open air outside `[0..60] x [0..60]`!
        let first_plunge = tp
            .segments_wcs
            .iter()
            .find(|s| seg_role(s) == Some(MotionRole::Plunge))
            .expect("must emit a plunge into open air outside stock");
        let px = first_plunge.end_wcs.x;
        let py = first_plunge.end_wcs.y;
        assert!(
            px < -2.9 || px > 62.9 || py < -2.9 || py > 62.9,
            "{label}: first plunge ({px:.2}, {py:.2}) must occur strictly in open air outside stock [0..60]^2 + R=3"
        );

        // Verify in `chip-sim` Heightmap AND TriDexel: zero crashes, zero boss gouges, deck cleared!
        let sim_ops = vec![SimOperationInput::from_operation(&op, &tool, &tp)];
        for engine in [SimEngineKind::Heightmap, SimEngineKind::TriDexel] {
            let sim = run_setup_simulation_with_choice(
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
                engine,
                SimComputeBackendChoice::Cpu,
            )
            .unwrap();

            assert_eq!(
                sim.critical_error_count(),
                0,
                "{label} ({engine:?}): expected zero crashes/gouges, got: {:#?}",
                sim.diagnostics
            );

            let z_boss = sim.final_stock.sample_stock_z(30.0, 30.0).unwrap();
            assert!(
                z_boss >= 15.9,
                "{label} ({engine:?}): boss top must remain intact at Z=16, got {z_boss:.3}"
            );
            for (x, y) in [(5.0, 5.0), (55.0, 5.0), (55.0, 55.0), (5.0, 55.0)] {
                let z_corner = sim.final_stock.sample_stock_z(x, y).unwrap();
                assert!(
                    (z_corner - deck_z).abs() <= 0.15,
                    "{label} ({engine:?}): exterior corner ({x}, {y}) expected cleared to Z={deck_z}, got {z_corner:.3}"
                );
            }
        }
    }
}

#[test]
fn pocket2d_sketch_treat_outer_boundary_as_open_approaches_from_outside() {
    let doc = CadDocument::empty();
    let mut setup = Setup::new("s_2d_open", "2D Open Sketch Setup", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_FRONT_LEFT,
    };
    setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(60.0),
        depth_y: ParamLength::from_mm(50.0),
        height_z: ParamLength::from_mm(10.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::OffsetFromTop {
            top_allowance: ParamLength::from_mm(0.0),
        },
    };
    // Sketch rectangle `[15..45] x [15..35]` (center `(30, 25)`, size `30 x 20`)
    let mut sk = SketchLayer::new("sk_open", "Open Pocket Sketch", &setup.id);
    sk.entities.push(SketchEntityEntry {
        id: "e_rect".to_string(),
        name: "Pocket Rect".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(30.0),
            y: ParamLength::from_mm(25.0),
            width: ParamLength::from_mm(30.0),
            height: ParamLength::from_mm(20.0),
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
        },
    });
    sk.rebuild_regions(None);
    let resolved = setup
        .resolve_with_sketches(&doc, std::slice::from_ref(&sk), DisplayUnit::Millimeter)
        .unwrap();

    let tool = flat_endmill(6.0);
    let cancel = AtomicBool::new(false);

    let run_pocket = |open_outer: bool, strategy: PocketStrategyKind| {
        let mut op = Operation::new("op_pkt", "2D Sketch Pocket", OperationKind::Pocket2D);
        op.tool_id = Some(tool.id.clone());
        op.selections.push(GeometrySelectionItem::SketchEntity {
            sketch_id: sk.id.clone(),
            entity_id: "e_rect".to_string(),
            reverse: false,
            side_flip: false,
        });
        op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op.heights.bottom = HeightSpec::new(HeightReference::StockTop, -4.0);
        op.passes.multiple_depths = false;
        op.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        if let StrategyParams::Pocket2D(p) = op.strategy_mut() {
            p.treat_outer_boundary_as_open = open_outer;
            p.strategy = strategy;
            p.stepover = ParamLength::from_mm(2.4);
        }
        generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk),
            setup: &resolved,
            op: &op,
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
        .unwrap()
    };

    // Closed pocket: every cutting XY point must stay strictly inside `[18..42] x [18..32]`
    let tp_closed = run_pocket(false, PocketStrategyKind::ContourParallel);
    for seg in &tp_closed.segments_wcs {
        if matches!(seg.motion, SegmentMotion3D::Rapid) {
            continue;
        }
        for p in [seg.start_wcs, seg.end_wcs] {
            assert!(
                p.x >= 17.9 && p.x <= 42.1 && p.y >= 17.9 && p.y <= 32.1,
                "Closed pocket must stay inside inset boundary [18..42]x[18..32], got ({:.2}, {:.2})",
                p.x,
                p.y
            );
        }
    }

    // Open-outer pocket (ContourParallel & Adaptive): cutter must approach from outside `[15..45] x [15..35]`!
    for (label, strategy) in [
        ("ContourParallel", PocketStrategyKind::ContourParallel),
        (
            "Adaptive2D",
            PocketStrategyKind::Adaptive(AdaptiveClearingParams::default()),
        ),
    ] {
        let tp_open = run_pocket(true, strategy);
        let entered_outside = tp_open.segments_wcs.iter().any(|seg| {
            !matches!(seg.motion, SegmentMotion3D::Rapid)
                && (seg.start_wcs.x < 14.5
                    || seg.start_wcs.x > 45.5
                    || seg.start_wcs.y < 14.5
                    || seg.start_wcs.y > 35.5)
        });
        assert!(
            entered_outside,
            "{label} with treat_outer_boundary_as_open=true must approach from outside [15..45]x[15..35]"
        );
    }
}

#[test]
fn contour2d_open_chain_extension_and_clip_to_stock_bounds() {
    let mut tris = Vec::new();
    push_box_tris(&mut tris, 0.0, 0.0, 0.0, 50.0, 40.0, 10.0);
    let stl = encode_test_stl(&tris);
    let doc =
        load_mesh_cad_document("stock_plate.stl", &stl, &MeshImportOptions::default()).unwrap();
    let mut setup = Setup::new("s_cnt", "Contour Clip Setup", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
    };
    setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(50.0),
        depth_y: ParamLength::from_mm(40.0),
        height_z: ParamLength::from_mm(10.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::OffsetFromBottom {
            bottom_allowance: ParamLength::from_mm(0.0),
        },
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    // Open line `(5, 20) -> (45, 20)` plus an oversized line `(-12, 20) -> (62, 20)`
    let mut sk = SketchLayer::new("sk_cnt", "Open Chains", &setup.id);
    sk.entities.push(SketchEntityEntry {
        id: "e_open_line".to_string(),
        name: "Open Line".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::new_polyline(ArcPolyline2D {
            segments: vec![ArcSegment2D::Line {
                start: DVec2::new(5.0, 20.0),
                end: DVec2::new(45.0, 20.0),
            }],
            closed: false,
        }),
    });
    sk.entities.push(SketchEntityEntry {
        id: "e_oversized".to_string(),
        name: "Oversized Line".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::new_polyline(ArcPolyline2D {
            segments: vec![ArcSegment2D::Line {
                start: DVec2::new(-12.0, 20.0),
                end: DVec2::new(62.0, 20.0),
            }],
            closed: false,
        }),
    });
    sk.rebuild_regions(None);

    let tool = flat_endmill(6.0);
    let cancel = AtomicBool::new(false);

    // 1. Open-chain tangent extension (`6.0 mm` at both ends):
    //    `(5, 20) -> (45, 20)` extends to `(-1, 20) -> (51, 20)`!
    let mut op_ext = Operation::new("op_ext", "Extended Open Chain", OperationKind::Contour2D);
    op_ext.tool_id = Some(tool.id.clone());
    op_ext.selections.push(GeometrySelectionItem::SketchEntity {
        sketch_id: sk.id.clone(),
        entity_id: "e_open_line".to_string(),
        reverse: false,
        side_flip: false,
    });
    op_ext.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_ext.heights.bottom = HeightSpec::new(HeightReference::StockTop, -3.0);
    op_ext.passes.multiple_depths = false;
    op_ext.linking.lead_in.enabled = false;
    if let StrategyParams::Contour2D(p) = op_ext.strategy_mut() {
        p.compensation = ContourCompensation::CenterOff;
        p.open_chain_extension = ParamLength::from_mm(6.0);
    }

    let tp_ext = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: std::slice::from_ref(&sk),
        setup: &resolved,
        op: &op_ext,
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

    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    for seg in &tp_ext.segments_wcs {
        if seg_role(seg) == Some(MotionRole::Cut) {
            min_x = min_x.min(seg.start_wcs.x).min(seg.end_wcs.x);
            max_x = max_x.max(seg.start_wcs.x).max(seg.end_wcs.x);
        }
    }
    assert!(
        (min_x - (-1.0)).abs() < 1e-3 && (max_x - 51.0).abs() < 1e-3,
        "6mm tangent extension on [5..45] must span [-1..51], got [{min_x:.4}..{max_x:.4}]"
    );

    // 2. Clip to Stock XY Bounds (`[0..50] x [0..40]`):
    //    Oversized line `[-12..62]` must clip cleanly to `[0..50]`!
    let mut op_clip = Operation::new("op_clip", "Clipped Contour", OperationKind::Contour2D);
    op_clip.tool_id = Some(tool.id.clone());
    op_clip
        .selections
        .push(GeometrySelectionItem::SketchEntity {
            sketch_id: sk.id.clone(),
            entity_id: "e_oversized".to_string(),
            reverse: false,
            side_flip: false,
        });
    op_clip.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_clip.heights.bottom = HeightSpec::new(HeightReference::StockTop, -3.0);
    op_clip.passes.multiple_depths = false;
    op_clip.linking.lead_in.enabled = false;
    if let StrategyParams::Contour2D(p) = op_clip.strategy_mut() {
        p.compensation = ContourCompensation::CenterOff;
        p.clip_to_stock_bounds = true;
    }

    let tp_clip = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: std::slice::from_ref(&sk),
        setup: &resolved,
        op: &op_clip,
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

    let mut c_min_x = f64::INFINITY;
    let mut c_max_x = f64::NEG_INFINITY;
    for seg in &tp_clip.segments_wcs {
        if seg_role(seg) == Some(MotionRole::Cut) {
            c_min_x = c_min_x.min(seg.start_wcs.x).min(seg.end_wcs.x);
            c_max_x = c_max_x.max(seg.start_wcs.x).max(seg.end_wcs.x);
        }
    }
    assert!(
        c_min_x >= -1e-3 && c_max_x <= 50.0 + 1e-3 && (c_max_x - c_min_x - 50.0).abs() < 1e-2,
        "clip_to_stock_bounds on [-12..62] in [0..50] stock must clip to [0..50], got [{c_min_x:.4}..{c_max_x:.4}]"
    );
}

#[test]
fn m82_params_default_invariants() {
    let face = FacingParams::default();
    assert_eq!(face.boundary.boundary_mode, MachiningBoundaryMode::StockBox);
    assert_eq!(
        face.boundary.tool_containment,
        ToolContainmentMode::ToolOutsideBoundary
    );
    assert!((face.boundary.boundary_offset.value_mm - 0.0).abs() < 1e-9);

    let pkt = Pocket2DParams::default();
    assert_eq!(pkt.boundary_mode, PocketBoundaryMode::SelectedPockets);
    assert!(!pkt.treat_outer_boundary_as_open);

    let cnt = Contour2DParams::default();
    assert!(!cnt.clip_to_stock_bounds);
    assert!((cnt.open_chain_extension.value_mm - 0.0).abs() < 1e-9);
}
