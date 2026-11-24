#![allow(clippy::unwrap_used, clippy::expect_used)]

//! Milestone 85 integration & verification tests:
//! - Backward-compatible TOML round-trip (`ProjectDocument::avoid_regions` & `Operation::avoid`)
//! - Shared keep-out regions linked across multiple operations & setups, clearance overrides, `suppressed`, and removal pruning
//! - `OperationInputHash` cache invalidation on avoid/shared/fixture mutations
//! - Validation diagnostics (`validation.rs`) and associative STEP reload (`reload.rs`)
//! - Fixture-trimmed cutting areas (`Adaptive3D`, `Facing`, `Pocket2D`, `Parallel3D`) verified in
//!   `chip-sim` (`Heightmap` + `TriDexel`) with 0 Rule 6 `FixtureCollision` and 0 target gouges
//! - Sketch keep-out zones on `Contour2D`, `VCarve`, `DrillBore`, `Deburr3D`, `Waterline3D`, `Scallop3D`
//! - 4th-axis rotary axial keep-out bands on `Rotary2Linear` & `Rotary4AxisSim` verified against
//!   `CylindricalStock` + `TriDexelStock`
//! - `Indexed3Plus1` fixture & selection transformation into virtual setup frame

use std::sync::atomic::AtomicBool;

use chip_cad::{
    CadDocument, CadKernel, MeshImportOptions, MonstertruckKernel, SurfaceKind, TopologyGraph,
    find_same_diameter_holes, load_mesh_cad_document, sample_multi_feature_bracket_step,
    sample_sculpted_dome_stl_bytes,
};
use chip_cam::{
    Adaptive3DParams, AvoidRegionConfig, BoxAnchor27, CamError, Contour2DParams, Deburr3DParams,
    DrillBoreParams, FixedBoxXyAlign, FixedBoxZAlign, FixtureBody, GeometrySelectionItem,
    HeightReference, HeightSpec, HoleMachiningMode, MachiningBoundaryMode, Operation,
    OperationGenRequest, OperationKind, OperationRotaryMode, OperationStatus, Parallel3DParams,
    Pocket2DParams, PocketStrategyKind, Rotary2LinearParams, Rotary4AxisPattern,
    Rotary4AxisSimParams, RotaryAxisKind, RotaryDriveMode, RotaryPatternMode, Scallop3DParams,
    SegmentMotion3D, Setup, SharedAvoidRegion, StockAxis, StockConfig, StrategyParams,
    ToolContainmentMode, VCarveParams, Waterline3DParams, WcsOriginMode, WorkOffsetSlot,
    generate_operation_toolpath_with_rest, resolve_setup_fixtures_parametric,
};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec2, DVec3, DisplayUnit, ParamLength};
use chip_project::{
    ProjectDocument, SetupSimRequest, ToolpathCache, compute_operation_input_hash,
    generate_project_blocking, rebind_project_to_reloaded_cad, resolve_kinematics_for_key,
    simulate_project_setup_with_engine, validate_project,
};
use chip_sim::{SimEngineKind, SimIssueKind, SimResolution};
use chip_sketch::{RectAnchorMode, SketchEntity, SketchEntityEntry, SketchLayer};
use chip_tools::{CutterGeometry, Tool, ToolId, default_starter_tool_library};

fn flat_tool(id: &str, num: u32, d: f64, flute_len: f64, stickout: f64) -> Tool {
    let mut t = default_starter_tool_library()
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { .. }))
        .unwrap()
        .clone();
    t.id = ToolId(id.to_string());
    t.tool_number = num;
    t.length_offset_number = num;
    t.diameter_offset_number = num;
    t.name = format!("Flat Ø{d:.1}mm");
    t.stickout_length_mm = stickout;
    t.geometry = CutterGeometry::FlatEndmill {
        diameter_mm: d,
        flute_length_mm: flute_len,
        overall_length_mm: stickout + 20.0,
        shank_diameter_mm: d,
        flutes: 3,
    };
    t
}

fn ball_tool(id: &str, num: u32, d: f64, flute_len: f64, stickout: f64) -> Tool {
    let mut t = default_starter_tool_library()
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::BallEndmill { .. }))
        .unwrap()
        .clone();
    t.id = ToolId(id.to_string());
    t.tool_number = num;
    t.length_offset_number = num;
    t.diameter_offset_number = num;
    t.name = format!("Ball Ø{d:.1}mm");
    t.stickout_length_mm = stickout;
    t.geometry = CutterGeometry::BallEndmill {
        diameter_mm: d,
        flute_length_mm: flute_len,
        overall_length_mm: stickout + 20.0,
        shank_diameter_mm: d,
        flutes: 2,
    };
    t
}

fn vbit_tool(id: &str, num: u32, d: f64, angle_deg: f64) -> Tool {
    let mut t = default_starter_tool_library()
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
        .unwrap()
        .clone();
    t.id = ToolId(id.to_string());
    t.tool_number = num;
    t.length_offset_number = num;
    t.diameter_offset_number = num;
    t.name = format!("V-Bit Ø{d:.1}mm {angle_deg:.0}°");
    t.stickout_length_mm = 30.0;
    t.geometry = CutterGeometry::ChamferMill {
        diameter_mm: d,
        tip_diameter_mm: 0.2,
        included_angle_deg: angle_deg,
        flute_length_mm: 16.0,
        overall_length_mm: 50.0,
        shank_diameter_mm: d,
        flutes: 2,
    };
    t
}

fn drill_tool(id: &str, num: u32, d: f64) -> Tool {
    let mut t = default_starter_tool_library()
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::TwistDrill { .. }))
        .unwrap()
        .clone();
    t.id = ToolId(id.to_string());
    t.tool_number = num;
    t.length_offset_number = num;
    t.diameter_offset_number = num;
    t.name = format!("Drill Ø{d:.1}mm");
    t.stickout_length_mm = 40.0;
    t.geometry = CutterGeometry::TwistDrill {
        diameter_mm: d,
        point_angle_deg: 118.0,
        flute_length_mm: 30.0,
        overall_length_mm: 65.0,
        shank_diameter_mm: d,
        flutes: 2,
    };
    t
}

fn make_rect_sketch(
    id: &str,
    name: &str,
    setup_id: &str,
    x_min: f64,
    y_min: f64,
    width: f64,
    height: f64,
) -> SketchLayer {
    let mut sk = SketchLayer::new(id, name, setup_id);
    sk.entities.push(SketchEntityEntry {
        id: format!("{id}_rect"),
        name: name.to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::FrontLeft,
            x: ParamLength::from_mm(x_min),
            y: ParamLength::from_mm(y_min),
            width: ParamLength::from_mm(width),
            height: ParamLength::from_mm(height),
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
        },
    });
    sk.rebuild_regions(None);
    sk
}

fn build_rotary_cylinder_stl_bytes(x0: f64, x1: f64, r_base: f64, r_amp: f64) -> Vec<u8> {
    let nx = 20usize;
    let nt = 36usize;
    let mut tris: Vec<[DVec3; 3]> = Vec::new();

    let pt = |ix: usize, it: usize| -> DVec3 {
        let u = ix as f64 / nx as f64;
        let x = x0 + u * (x1 - x0);
        let th = (it % nt) as f64 * (std::f64::consts::TAU / nt as f64);
        let r = r_base + r_amp * (0.5 + 0.5 * (2.0 * th).cos());
        DVec3::new(x, r * th.cos(), r * th.sin())
    };

    for ix in 0..nx {
        for it in 0..nt {
            let p00 = pt(ix, it);
            let p10 = pt(ix + 1, it);
            let p11 = pt(ix + 1, it + 1);
            let p01 = pt(ix, it + 1);
            tris.push([p00, p10, p11]);
            tris.push([p00, p11, p01]);
        }
    }
    let c0 = DVec3::new(x0, 0.0, 0.0);
    let c1 = DVec3::new(x1, 0.0, 0.0);
    for it in 0..nt {
        tris.push([c0, pt(0, it + 1), pt(0, it)]);
        tris.push([c1, pt(nx, it), pt(nx, it + 1)]);
    }

    let mut bytes = Vec::with_capacity(84 + tris.len() * 50);
    bytes.extend_from_slice(&[0u8; 80]);
    bytes.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for [a, b, c] in tris {
        let n = (b - a).cross(c - a).normalize_or_zero();
        for v in [n, a, b, c] {
            bytes.extend_from_slice(&(v.x as f32).to_le_bytes());
            bytes.extend_from_slice(&(v.y as f32).to_le_bytes());
            bytes.extend_from_slice(&(v.z as f32).to_le_bytes());
        }
        bytes.extend_from_slice(&0u16.to_le_bytes());
    }
    bytes
}

fn box_stl_bytes(x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) -> Vec<u8> {
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
        [0, 3, 2, 1],
        [4, 5, 6, 7],
        [0, 1, 5, 4],
        [2, 3, 7, 6],
        [0, 4, 7, 3],
        [1, 2, 6, 5],
    ];
    let mut tris = Vec::new();
    for [a, b, c, d] in quads {
        tris.push([v[a], v[b], v[c]]);
        tris.push([v[a], v[c], v[d]]);
    }
    let mut bytes = Vec::with_capacity(84 + tris.len() * 50);
    bytes.extend_from_slice(&[0u8; 80]);
    bytes.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for [a, b, c] in tris {
        let n = (b - a).cross(c - a).normalize_or_zero();
        for p in [n, a, b, c] {
            bytes.extend_from_slice(&(p.x as f32).to_le_bytes());
            bytes.extend_from_slice(&(p.y as f32).to_le_bytes());
            bytes.extend_from_slice(&(p.z as f32).to_le_bytes());
        }
        bytes.extend_from_slice(&0u16.to_le_bytes());
    }
    bytes
}

/// Counts non-rapid cutting segments that come strictly closer than `min_clearance_mm` (exact
/// Euclidean distance to the keep-out rectangle `[rx0..rx1] x [ry0..ry1]`, matching circular
/// Minkowski disk dilation `tool_r + radial_clearance`).
fn count_cutting_samples_within_rect_clearance(
    tp: &chip_cam::GeneratedToolpath,
    rx0: f64,
    ry0: f64,
    rx1: f64,
    ry1: f64,
    min_clearance_mm: f64,
) -> usize {
    let mut hits = 0usize;
    for seg in &tp.segments_wcs {
        if matches!(seg.motion, SegmentMotion3D::Rapid) {
            continue;
        }
        let len = seg.length_mm();
        let steps = ((len / 0.25).ceil() as usize).clamp(2, 64);
        for k in 0..=steps {
            let p = seg.sample_point(k as f64 / steps as f64);
            let dx = (rx0 - p.x).max(0.0).max(p.x - rx1);
            let dy = (ry0 - p.y).max(0.0).max(p.y - ry1);
            if dx.hypot(dy) < min_clearance_mm {
                hits += 1;
                break;
            }
        }
    }
    hits
}

/// Checks whether any non-rapid cutting segment enters the axis-aligned rectangle `[x0..x1] x [y0..y1]`.
fn count_cutting_samples_inside_rect(
    tp: &chip_cam::GeneratedToolpath,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
) -> usize {
    count_cutting_samples_within_rect_clearance(tp, x0, y0, x1, y1, 1e-6)
}

#[test]
fn legacy_toml_roundtrip_omits_empty_avoid_and_shared_regions() {
    let mut proj = ProjectDocument::new("Legacy Compat");
    let tool = flat_tool("t1", 1, 6.0, 18.0, 30.0);
    proj.tools.push(tool.clone());
    let mut op = Operation::new("op_1", "Face Top", OperationKind::Facing);
    op.tool_id = Some(tool.id.clone());
    proj.setups[0].operations.push(op);

    let serialized = toml::to_string_pretty(&proj).unwrap();
    assert!(
        !serialized.contains("avoid_regions"),
        "empty avoid_regions must be omitted from .chip.toml"
    );
    assert!(
        !serialized.contains(".avoid]"),
        "empty Operation::avoid must be omitted from .chip.toml:\n{serialized}"
    );

    let reloaded: ProjectDocument = toml::from_str(&serialized).unwrap();
    assert!(reloaded.avoid_regions.is_empty());
    assert!(reloaded.setups[0].operations[0].avoid.is_empty());
    assert_eq!(reloaded, proj);

    // Populated shared avoid region & operation avoid round-trip losslessly
    let mut shared = SharedAvoidRegion::new("avoid_clamp_1", "North Toe Clamp");
    shared.fixture_ids.push("fix_1".to_string());
    shared.radial_clearance = Some(ParamLength::from_mm(1.25));
    shared.axial_clearance = Some(ParamLength::from_mm(0.75));
    proj.avoid_regions.push(shared);

    let op_mut = &mut proj.setups[0].operations[0];
    op_mut
        .avoid
        .shared_region_ids
        .push("avoid_clamp_1".to_string());
    op_mut.avoid.avoid_all_fixtures = true;

    let populated_toml = toml::to_string_pretty(&proj).unwrap();
    assert!(populated_toml.contains("avoid_regions"));
    assert!(populated_toml.contains("avoid_clamp_1"));
    let roundtripped: ProjectDocument = toml::from_str(&populated_toml).unwrap();
    assert_eq!(roundtripped, proj);
}

#[test]
fn shared_avoid_region_links_across_ops_and_setups_respects_overrides_and_suppressed() {
    let mut project = ProjectDocument::new("Shared Keep-Out Multi-Op");
    let tool = flat_tool("flat_6", 1, 6.0, 18.0, 30.0);
    project.tools.push(tool.clone());

    let make_setup = |id: &str, name: &str| {
        let mut s = Setup::new(id, name, WorkOffsetSlot::G54);
        s.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        s.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(100.0),
            depth_y: ParamLength::from_mm(80.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: FixedBoxXyAlign::CenterXy,
            z_align: FixedBoxZAlign::CenterZ,
        };
        s
    };

    let mut setup1 = make_setup("s1", "Setup 1");
    let mut setup2 = make_setup("s2", "Setup 2");

    // Pocket sketch covering [-40..40] x [-30..30]
    let sk_pocket = make_rect_sketch("sk_pkt", "Pocket Area", "s1", -40.0, -30.0, 80.0, 60.0);
    // Shared keep-out zone centered at [-10..10] x [-10..10]
    let sk_shared = make_rect_sketch(
        "sk_avoid_shared",
        "Center Boss",
        "s1",
        -10.0,
        -10.0,
        20.0,
        20.0,
    );
    // Operation-local extra keep-out at [25..40] x [-10..10]
    let sk_local = make_rect_sketch("sk_avoid_local", "Right Pin", "s1", 25.0, -10.0, 15.0, 20.0);
    project.sketches = vec![sk_pocket, sk_shared, sk_local];

    // Shared region with custom radial_clearance override = 2.0 mm (tool_r=3 + 2.0 = 5.0 mm -> [-15..15] x [-15..15])
    let mut shared = SharedAvoidRegion::new("sh_center", "Center Keep-Out");
    shared
        .selections
        .push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sk_avoid_shared".to_string(),
            sub_layer: None,
            reverse: false,
            side_flip: false,
        });
    shared.radial_clearance = Some(ParamLength::from_mm(2.0));
    project.avoid_regions.push(shared);

    // Setup 1 Op 1: Facing linked to shared + local keep-out
    let mut op_face1 = Operation::new("op_face_1", "Face S1", OperationKind::Facing);
    op_face1.tool_id = Some(tool.id.clone());
    op_face1.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_face1.heights.bottom = HeightSpec::new(HeightReference::StockTop, -1.0);
    op_face1
        .avoid
        .shared_region_ids
        .push("sh_center".to_string());
    op_face1
        .avoid
        .selections
        .push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sk_avoid_local".to_string(),
            sub_layer: None,
            reverse: false,
            side_flip: false,
        });
    op_face1.avoid.radial_clearance = ParamLength::from_mm(1.0);
    setup1.operations.push(op_face1);

    // Setup 1 Op 2: Pocket2D linked to shared keep-out
    let mut op_pkt = Operation::new("op_pkt_1", "Pocket S1", OperationKind::Pocket2D);
    op_pkt.tool_id = Some(tool.id.clone());
    op_pkt
        .selections
        .push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sk_pkt".to_string(),
            sub_layer: None,
            reverse: false,
            side_flip: false,
        });
    op_pkt.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_pkt.heights.bottom = HeightSpec::new(HeightReference::StockTop, -3.0);
    op_pkt.avoid.shared_region_ids.push("sh_center".to_string());
    setup1.operations.push(op_pkt);

    // Setup 2 Op 1: Facing linked to the SAME shared keep-out across setups!
    let mut op_face2 = Operation::new("op_face_2", "Face S2", OperationKind::Facing);
    op_face2.tool_id = Some(tool.id.clone());
    op_face2.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_face2.heights.bottom = HeightSpec::new(HeightReference::StockTop, -1.0);
    op_face2
        .avoid
        .shared_region_ids
        .push("sh_center".to_string());
    setup2.operations.push(op_face2);

    project.setups = vec![setup1, setup2];
    assert_eq!(project.avoid_region_usage_count("sh_center"), 3);

    let cad = CadDocument::empty();
    let mut cache = ToolpathCache::new(32);
    let rep = generate_project_blocking(&mut project, &cad, &mut cache, 1);
    assert!(rep.is_ok(), "generation errors: {:?}", rep.errors);

    // Verify ALL 3 operations strictly stay outside the shared keep-out + override clearance (tool_r=3 + 2.0 = 5.0 mm)
    for (s_idx, o_idx, label) in [(0, 0, "face1"), (0, 1, "pkt1"), (1, 0, "face2")] {
        let (_, tp) = cache
            .get_for_operation(&project, s_idx, o_idx, 1)
            .expect("toolpath cached");
        let shared_hits =
            count_cutting_samples_within_rect_clearance(&tp, -10.0, -10.0, 10.0, 10.0, 4.85);
        assert_eq!(
            shared_hits, 0,
            "{label} cut within tool_r + override_clearance (5.0 mm) of shared keep-out [-10..10]!"
        );
    }

    // Op 1 ALSO had a local keep-out at [25..40] x [-10..10] with local clearance 1.0 + tool_r 3.0 = 4.0 mm
    let (_, tp_face1) = cache.get_for_operation(&project, 0, 0, 1).unwrap();
    let local_hits =
        count_cutting_samples_within_rect_clearance(&tp_face1, 25.0, -10.0, 40.0, 10.0, 3.85);
    assert_eq!(
        local_hits, 0,
        "op_face_1 cut within tool_r + local_clearance (4.0 mm) of local keep-out!"
    );

    // Suppressing the shared keep-out immediately invalidates all 3 ops' hashes & lets them cut through center!
    let h_before: Vec<_> = [(0, 0), (0, 1), (1, 0)]
        .iter()
        .map(|&(s, o)| compute_operation_input_hash(&project, s, o, 1).unwrap())
        .collect();
    project.avoid_region_mut("sh_center").unwrap().suppressed = true;
    let h_after: Vec<_> = [(0, 0), (0, 1), (1, 0)]
        .iter()
        .map(|&(s, o)| compute_operation_input_hash(&project, s, o, 1).unwrap())
        .collect();
    for (i, (a, b)) in h_before.iter().zip(&h_after).enumerate() {
        assert_ne!(
            a, b,
            "suppressing shared avoid region must change hash for op #{i}"
        );
    }
    cache.reconcile_project_statuses(&mut project, 1, &Default::default());
    assert_eq!(
        project.setups[0].operations[0].runtime_status,
        OperationStatus::Dirty
    );
    assert_eq!(
        project.setups[0].operations[1].runtime_status,
        OperationStatus::Dirty
    );
    assert_eq!(
        project.setups[1].operations[0].runtime_status,
        OperationStatus::Dirty
    );

    let rep2 = generate_project_blocking(&mut project, &cad, &mut cache, 1);
    assert!(rep2.is_ok(), "suppressed regen errors: {:?}", rep2.errors);
    let (_, tp_face2_sup) = cache.get_for_operation(&project, 1, 0, 1).unwrap();
    let center_hits = count_cutting_samples_inside_rect(&tp_face2_sup, -5.0, -5.0, 5.0, 5.0);
    assert!(
        center_hits > 0,
        "suppressing shared avoid region should allow Face S2 to cut the center!"
    );

    // Removing the shared avoid region prunes stale shared_region_ids cleanly across all setups
    assert!(project.remove_avoid_region("sh_center"));
    assert_eq!(project.avoid_region_usage_count("sh_center"), 0);
    assert!(
        project.setups[0].operations[0]
            .avoid
            .shared_region_ids
            .is_empty()
    );
    assert!(
        project.setups[0].operations[1]
            .avoid
            .shared_region_ids
            .is_empty()
    );
    assert!(
        project.setups[1].operations[0]
            .avoid
            .shared_region_ids
            .is_empty()
    );
    assert_eq!(
        project.setups[1].operations[0].runtime_status,
        OperationStatus::Dirty
    );
}

#[test]
fn cache_hash_invalidation_and_validation_and_step_reload_for_avoid_regions() {
    let step = sample_multi_feature_bracket_step(false).unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("bracket.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();

    let mut project = ProjectDocument::new("Hash & Validation Test");
    let tool = flat_tool("flat_6", 1, 6.0, 18.0, 30.0);
    project.tools.push(tool.clone());
    project.setups[0].wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
    };
    project.setups[0]
        .fixtures
        .push(FixtureBody::default_toe_clamp(
            "clamp_1",
            "Clamp 1",
            DVec2::new(85.0, 35.0),
            0.0,
        ));
    let mut op = Operation::new("op_1", "Face", OperationKind::Facing);
    op.tool_id = Some(tool.id.clone());
    project.setups[0].operations.push(op);

    let h0 = compute_operation_input_hash(&project, 0, 0, 1).unwrap();

    // 1. Toggling avoid_all_fixtures changes hash
    project.setups[0].operations[0].avoid.avoid_all_fixtures = true;
    let h1 = compute_operation_input_hash(&project, 0, 0, 1).unwrap();
    assert_ne!(h0, h1);

    // 2. Moving referenced fixture changes hash
    if let chip_cam::FixtureKind::ToeClamp { anchor_xy, .. } =
        &mut project.setups[0].fixtures[0].kind
    {
        anchor_xy[0] = ParamLength::from_mm(75.0);
    }
    let h2 = compute_operation_input_hash(&project, 0, 0, 1).unwrap();
    assert_ne!(h1, h2);

    // 3. Adding explicit fixture_ids changes hash
    project.setups[0].operations[0]
        .avoid
        .fixture_ids
        .push("clamp_1".to_string());
    let h3 = compute_operation_input_hash(&project, 0, 0, 1).unwrap();
    assert_ne!(h2, h3);

    // 4. Adding local selection changes hash
    let top_face = doc
        .faces
        .iter()
        .find(|f| matches!(f.surface, SurfaceKind::Plane { normal, .. } if normal.z > 0.99))
        .unwrap();
    let face_sel = GeometrySelectionItem::PlanarFace {
        binding: doc.bind_face(top_face.id).unwrap(),
    };
    project.setups[0].operations[0]
        .avoid
        .selections
        .push(face_sel.clone());
    let h4 = compute_operation_input_hash(&project, 0, 0, 1).unwrap();
    assert_ne!(h3, h4);

    // 5. Linking & editing shared avoid region changes hash
    let mut sh = SharedAvoidRegion::new("sh_1", "Shared 1");
    sh.selections.push(face_sel);
    project.avoid_regions.push(sh);
    project.setups[0].operations[0]
        .avoid
        .shared_region_ids
        .push("sh_1".to_string());
    let h5 = compute_operation_input_hash(&project, 0, 0, 1).unwrap();
    assert_ne!(h4, h5);

    project.avoid_region_mut("sh_1").unwrap().radial_clearance = Some(ParamLength::from_mm(1.5));
    let h6 = compute_operation_input_hash(&project, 0, 0, 1).unwrap();
    assert_ne!(h5, h6);

    // 6. Associative STEP reload re-binds BOTH op.avoid.selections and project.avoid_regions[*].selections
    let topo = TopologyGraph::build(&doc);
    // Corrupt last_index slightly to prove GeometricSignature re-matching repairs it!
    if let GeometrySelectionItem::PlanarFace { binding } =
        &mut project.setups[0].operations[0].avoid.selections[0]
    {
        binding.last_index = 9999;
    }
    if let GeometrySelectionItem::PlanarFace { binding } =
        &mut project.avoid_regions[0].selections[0]
    {
        binding.last_index = 9999;
    }
    let reload_rep = rebind_project_to_reloaded_cad(&mut project, 0, &doc, &topo, "sha_v2", None);
    assert_eq!(
        reload_rep.needs_repick_count, 0,
        "reload report: {reload_rep:?}"
    );
    if let GeometrySelectionItem::PlanarFace { binding } =
        &project.setups[0].operations[0].avoid.selections[0]
    {
        assert_eq!(binding.last_index, top_face.id.0);
    }
    if let GeometrySelectionItem::PlanarFace { binding } = &project.avoid_regions[0].selections[0] {
        assert_eq!(binding.last_index, top_face.id.0);
    }

    // 7. Validation catches missing shared region, missing fixture IDs (local & shared), and negative clearances
    let mut bad_proj = project.clone();
    bad_proj.avoid_regions[0].radial_clearance = Some(ParamLength::from_mm(-0.5));
    bad_proj.avoid_regions[0]
        .fixture_ids
        .push("deleted_fix_shared".to_string());
    let bad_op = &mut bad_proj.setups[0].operations[0];
    bad_op
        .avoid
        .shared_region_ids
        .push("missing_shared_id".to_string());
    bad_op
        .avoid
        .fixture_ids
        .push("deleted_fix_local".to_string());
    bad_op.avoid.axial_clearance = ParamLength::from_mm(-1.0);

    let vrep = validate_project(&bad_proj, None, Some(&doc));
    let msgs: Vec<&str> = vrep
        .diagnostics
        .iter()
        .map(|d| d.message.as_str())
        .collect();
    assert!(
        msgs.iter().any(|m| m.contains("missing_shared_id")),
        "expected missing shared_region_id error: {msgs:?}"
    );
    assert!(
        msgs.iter().any(|m| m.contains("deleted_fix_local")),
        "expected missing local fixture_id error: {msgs:?}"
    );
    assert!(
        msgs.iter().any(|m| m.contains("deleted_fix_shared")),
        "expected missing shared fixture_id error: {msgs:?}"
    );
    assert!(
        msgs.iter().any(|m| m.contains("radial_clearance")),
        "expected negative shared radial_clearance error: {msgs:?}"
    );
    assert!(
        msgs.iter().any(|m| m.contains("axial_clearance")),
        "expected negative op axial_clearance error: {msgs:?}"
    );
}

#[test]
fn fixture_trimmed_cutting_areas_pass_chip_sim_zero_collision_and_zero_gouges() {
    // 1. Facing + Pocket2D on an 80x60x20 stock block with a ToeClamp overlapping the +X edge
    //    AND a Spoilboard sitting flush beneath stock bottom (verifying spoilboard is skipped!)
    {
        let mut project = ProjectDocument::new("Fixture Trimmed 2.5D");
        let tool = flat_tool("flat_6", 1, 6.0, 20.0, 40.0);
        project.tools.push(tool.clone());

        let box_bytes = box_stl_bytes(-40.0, -30.0, -20.0, 40.0, 30.0, -1.0);
        let cad_doc =
            load_mesh_cad_document("block.stl", &box_bytes, &MeshImportOptions::default()).unwrap();

        let setup = &mut project.setups[0];
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(80.0),
            depth_y: ParamLength::from_mm(60.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: FixedBoxXyAlign::CenterXy,
            z_align: FixedBoxZAlign::CenterZ,
        };
        setup.auto_avoid_fixtures_on_rapids = true;
        // Toe clamp gripping the +X edge at (40, 0) with 8 mm nose overhang onto stock top!
        let mut clamp = FixtureBody::default_toe_clamp(
            "clamp_east",
            "East Toe Clamp",
            DVec2::new(40.0, 0.0),
            0.0,
        );
        if let chip_cam::FixtureKind::ToeClamp {
            nose_overhang,
            strap_height,
            stud_height_above_strap,
            ..
        } = &mut clamp.kind
        {
            *nose_overhang = ParamLength::from_mm(8.0);
            *strap_height = ParamLength::from_mm(8.0);
            *stud_height_above_strap = ParamLength::from_mm(6.0);
        }
        setup.fixtures.push(clamp);
        // Spoilboard beneath stock bottom (Z = [-38, -20]) must NOT block Facing/Pocket2D!
        setup
            .fixtures
            .push(FixtureBody::default_spoilboard("spoil_1", "MDF Spoilboard"));

        let sk_pkt = make_rect_sketch("sk_pkt", "Top Pocket", &setup.id, -35.0, -22.0, 70.0, 44.0);
        project.sketches.push(sk_pkt);

        let mut op_face = Operation::new("op_face", "Face Around Clamp", OperationKind::Facing);
        op_face.tool_id = Some(tool.id.clone());
        op_face.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op_face.heights.bottom = HeightSpec::new(HeightReference::StockTop, -1.0);
        op_face.avoid.avoid_all_fixtures = true;
        op_face.avoid.radial_clearance = ParamLength::from_mm(1.0);
        project.setups[0].operations.push(op_face);

        let mut op_pkt = Operation::new("op_pkt", "Pocket Around Clamp", OperationKind::Pocket2D);
        op_pkt.tool_id = Some(tool.id.clone());
        op_pkt
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sk_pkt".to_string(),
                sub_layer: None,
                reverse: false,
                side_flip: false,
            });
        op_pkt.heights.top = HeightSpec::new(HeightReference::StockTop, -1.0);
        op_pkt.heights.bottom = HeightSpec::new(HeightReference::StockTop, -5.0);
        op_pkt.strategy = Some(StrategyParams::Pocket2D(Pocket2DParams {
            strategy: PocketStrategyKind::ContourParallel,
            stepover: ParamLength::from_mm(2.5),
            ..Pocket2DParams::default()
        }));
        op_pkt.avoid.avoid_all_fixtures = true;
        op_pkt.avoid.radial_clearance = ParamLength::from_mm(1.0);
        project.setups[0].operations.push(op_pkt);

        let mut cache = ToolpathCache::new(16);
        let gen_rep = generate_project_blocking(&mut project, &cad_doc, &mut cache, 1);
        assert!(
            gen_rep.is_ok(),
            "2.5D fixture-trimmed gen errors: {:?}",
            gen_rep.errors
        );

        let kin = resolve_kinematics_for_key(&project.setups[0].post_profile, None);
        for engine in [SimEngineKind::Heightmap, SimEngineKind::TriDexel] {
            let sim = simulate_project_setup_with_engine(
                &SetupSimRequest {
                    project: &project,
                    setup_idx: 0,
                    cad_doc: &cad_doc,
                    cad_revision: 1,
                    cache: &cache,
                    kinematics: kin,
                    resolution: SimResolution::Standard,
                    cancel: None,
                },
                engine,
            )
            .unwrap();
            let fix_hits: Vec<_> = sim
                .diagnostics
                .iter()
                .filter(|d| d.kind == SimIssueKind::FixtureCollision)
                .collect();
            let rapid_hits: Vec<_> = sim
                .diagnostics
                .iter()
                .filter(|d| d.kind == SimIssueKind::RapidIntoStock)
                .collect();
            assert!(
                fix_hits.is_empty(),
                "2.5D Facing+Pocket ({engine:?}): expected 0 FixtureCollision, got {fix_hits:#?}"
            );
            assert!(
                rapid_hits.is_empty(),
                "2.5D Facing+Pocket ({engine:?}): expected 0 RapidIntoStock, got {rapid_hits:#?}"
            );
            assert!(
                sim.removed_volume_mm3 > 5_000.0,
                "2.5D Facing+Pocket ({engine:?}): expected unblocked stock removed, got {:.1} mm³",
                sim.removed_volume_mm3
            );
        }
    }

    // 2. Adaptive3D + Parallel3D on sculpted_dome.stl with a ToeClamp overlapping the +X pedestal edge!
    {
        let dome_stl = sample_sculpted_dome_stl_bytes();
        let cad_doc = load_mesh_cad_document(
            "sculpted_dome.stl",
            &dome_stl,
            &MeshImportOptions::default(),
        )
        .unwrap();

        let mut project = ProjectDocument::new("Fixture Trimmed 3D");
        let rough_tool = flat_tool("flat_6", 1, 6.0, 26.0, 48.0);
        let fin_tool = ball_tool("ball_6", 2, 6.0, 26.0, 48.0);
        project.tools = vec![rough_tool.clone(), fin_tool.clone()];

        let setup = &mut project.setups[0];
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
        };
        setup.stock = StockConfig::RelativeBox {
            side_pos_x: ParamLength::from_mm(0.0),
            side_neg_x: ParamLength::from_mm(0.0),
            side_pos_y: ParamLength::from_mm(0.0),
            side_neg_y: ParamLength::from_mm(0.0),
            symmetric_xy: true,
            top_offset_z: ParamLength::from_mm(1.0),
            bottom_offset_z: ParamLength::from_mm(0.0),
            sheet_thickness_z: ParamLength::from_mm(12.0),
            round_increment_mm: None,
        };
        setup.auto_avoid_fixtures_on_rapids = true;

        // Dome is 70x70x33 mm in ModelBoxPoint BOTTOM_FRONT_LEFT -> [0..70] x [0..70] x [0..33],
        // pedestal top Z = 8.0, stock top Z = 34.0.
        // Place ToeClamp on the +X pedestal edge at (70, 35) anchored at Z = 8.0 (pedestal top)!
        let mut clamp = FixtureBody::default_toe_clamp(
            "clamp_dome_east",
            "Pedestal Toe Clamp",
            DVec2::new(70.0, 35.0),
            0.0,
        );
        if let chip_cam::FixtureKind::ToeClamp {
            z_reference,
            z_offset,
            nose_overhang,
            strap_height,
            stud_height_above_strap,
            ..
        } = &mut clamp.kind
        {
            *z_reference = chip_cam::FixtureZRef::StockBottom;
            *z_offset = ParamLength::from_mm(8.0);
            *nose_overhang = ParamLength::from_mm(10.0);
            *strap_height = ParamLength::from_mm(10.0);
            *stud_height_above_strap = ParamLength::from_mm(8.0);
        }
        setup.fixtures.push(clamp);

        let mut op_adap = Operation::new("op_adap3d", "Adaptive 3D", OperationKind::Adaptive3D);
        op_adap.tool_id = Some(rough_tool.id.clone());
        op_adap.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op_adap.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 8.0);
        op_adap.passes.stock_to_leave_radial = ParamLength::from_mm(0.5);
        op_adap.passes.stock_to_leave_axial = ParamLength::from_mm(0.3);
        let mut adap_p = Adaptive3DParams {
            max_roughing_stepdown: ParamLength::from_mm(6.0),
            fine_stepdown: ParamLength::from_mm(0.0),
            ..Default::default()
        };
        adap_p.boundary.boundary_mode = MachiningBoundaryMode::ModelBoundingBox;
        adap_p.boundary.tool_containment = ToolContainmentMode::ToolInsideBoundary;
        op_adap.strategy = Some(StrategyParams::Adaptive3D(adap_p));
        op_adap.avoid.avoid_all_fixtures = true;
        op_adap.avoid.radial_clearance = ParamLength::from_mm(1.0);
        project.setups[0].operations.push(op_adap);

        let mut op_par = Operation::new("op_par3d", "Parallel 3D", OperationKind::Parallel3D);
        op_par.tool_id = Some(fin_tool.id.clone());
        op_par.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op_par.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 8.0);
        let mut par_p = Parallel3DParams {
            stepover: ParamLength::from_mm(2.0),
            use_cusp_height: false,
            ..Default::default()
        };
        par_p.boundary.boundary_mode = MachiningBoundaryMode::ModelBoundingBox;
        par_p.boundary.tool_containment = ToolContainmentMode::ToolInsideBoundary;
        op_par.strategy = Some(StrategyParams::Parallel3D(par_p));
        op_par.avoid.avoid_all_fixtures = true;
        op_par.avoid.radial_clearance = ParamLength::from_mm(1.0);
        project.setups[0].operations.push(op_par);

        let mut cache = ToolpathCache::new(16);
        let gen_rep = generate_project_blocking(&mut project, &cad_doc, &mut cache, 1);
        assert!(
            gen_rep.is_ok(),
            "3D fixture-trimmed gen errors: {:?}",
            gen_rep.errors
        );

        let kin = resolve_kinematics_for_key(&project.setups[0].post_profile, None);
        for engine in [SimEngineKind::Heightmap, SimEngineKind::TriDexel] {
            let sim = simulate_project_setup_with_engine(
                &SetupSimRequest {
                    project: &project,
                    setup_idx: 0,
                    cad_doc: &cad_doc,
                    cad_revision: 1,
                    cache: &cache,
                    kinematics: kin,
                    resolution: SimResolution::Standard,
                    cancel: None,
                },
                engine,
            )
            .unwrap();
            let fix_hits: Vec<_> = sim
                .diagnostics
                .iter()
                .filter(|d| d.kind == SimIssueKind::FixtureCollision)
                .collect();
            let gouges: Vec<_> = sim
                .diagnostics
                .iter()
                .filter(|d| d.kind == SimIssueKind::TargetModelGouge)
                .collect();
            assert!(
                fix_hits.is_empty(),
                "3D Adaptive+Parallel ({engine:?}): expected 0 FixtureCollision, got {fix_hits:#?}"
            );
            assert!(
                gouges.is_empty(),
                "3D Adaptive+Parallel ({engine:?}): expected 0 TargetGouge, got {gouges:#?}"
            );
            assert!(
                sim.removed_volume_mm3 > 20_000.0,
                "3D Adaptive+Parallel ({engine:?}): expected unblocked stock removed, got {:.1} mm³",
                sim.removed_volume_mm3
            );
        }
    }
}

#[test]
fn sketch_keepout_excludes_contour2d_vcarve_drillbore_deburr3d_and_surface3d() {
    let cancel = AtomicBool::new(false);

    // 1. Contour2D + VCarve on a pure-2D sketch with a sketched keep-out rectangle
    {
        let mut setup = Setup::new("s1", "2D Keepout Setup", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(120.0),
            depth_y: ParamLength::from_mm(80.0),
            height_z: ParamLength::from_mm(15.0),
            xy_align: FixedBoxXyAlign::CenterXy,
            z_align: FixedBoxZAlign::CenterZ,
        };
        let sk_part = make_rect_sketch("sk_part", "Part Profile", "s1", -40.0, -25.0, 80.0, 50.0);
        // Keep-out rectangle covering the right side X=[30..50], Y=[-12..12]
        let sk_avoid = make_rect_sketch("sk_avoid", "Keep-Out", "s1", 30.0, -12.0, 20.0, 24.0);
        let sketches = vec![sk_part, sk_avoid];
        let cad = CadDocument::empty();
        let rs = setup
            .resolve_with_sketches(&cad, &sketches, DisplayUnit::Millimeter)
            .unwrap();

        // 1a. Contour2D (Ø6 mm, tool_r=3.0 + radial_clearance=1.5 = 4.5 mm -> X=[25.5..54.5], Y=[-16.5..16.5])
        let flat6 = flat_tool("flat_6", 1, 6.0, 18.0, 30.0);
        let mut op_cnt = Operation::new("op_cnt", "Contour Keepout", OperationKind::Contour2D);
        op_cnt
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sk_part".to_string(),
                sub_layer: None,
                reverse: false,
                side_flip: false,
            });
        op_cnt.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op_cnt.heights.bottom = HeightSpec::new(HeightReference::StockTop, -4.0);
        op_cnt.strategy = Some(StrategyParams::Contour2D(Contour2DParams::default()));
        op_cnt
            .avoid
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sk_avoid".to_string(),
                sub_layer: None,
                reverse: false,
                side_flip: false,
            });
        op_cnt.avoid.radial_clearance = ParamLength::from_mm(1.5);

        let tp_cnt = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &cad,
            sketches: &sketches,
            setup: &rs,
            op: &op_cnt,
            tool: &flat6,
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
        let ox = rs.frame.origin_mm.x;
        let oy = rs.frame.origin_mm.y;
        let cnt_hits = count_cutting_samples_within_rect_clearance(
            &tp_cnt,
            30.0 - ox,
            -12.0 - oy,
            50.0 - ox,
            12.0 - oy,
            4.35,
        );
        assert_eq!(cnt_hits, 0, "Contour2D cut inside inflated keep-out zone!");
        assert!(tp_cnt.stats.pass_count >= 1);

        // 1b. VCarve (Ø12 mm 90° V-bit -> tool_r=6.0 + radial_clearance=1.0 = 7.0 mm)
        // Strip sketch [-40..40] x [-4..4], keep-out at [-5..5] x [-10..10] -> effective X=[-12..12]
        let sk_v = make_rect_sketch("sk_vstrip", "V-Strip", "s1", -40.0, -4.0, 80.0, 8.0);
        let sk_v_avoid = make_rect_sketch("sk_vavoid", "V-Avoid", "s1", -5.0, -10.0, 10.0, 20.0);
        let v_sketches = vec![sk_v, sk_v_avoid];
        let rs_vc = setup
            .resolve_with_sketches(&cad, &v_sketches, DisplayUnit::Millimeter)
            .unwrap();
        let ox_vc = rs_vc.frame.origin_mm.x;
        let oy_vc = rs_vc.frame.origin_mm.y;
        let vbit = vbit_tool("v90", 2, 12.0, 90.0);
        let mut op_vc = Operation::new("op_vc", "VCarve Keepout", OperationKind::VCarve);
        op_vc
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sk_vstrip".to_string(),
                sub_layer: None,
                reverse: false,
                side_flip: false,
            });
        op_vc.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op_vc.heights.bottom = HeightSpec::new(HeightReference::StockTop, -4.0);
        op_vc.strategy = Some(StrategyParams::VCarve(VCarveParams::default()));
        op_vc
            .avoid
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sk_vavoid".to_string(),
                sub_layer: None,
                reverse: false,
                side_flip: false,
            });
        op_vc.avoid.radial_clearance = ParamLength::from_mm(1.0);

        let tp_vc = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &cad,
            sketches: &v_sketches,
            setup: &rs_vc,
            op: &op_vc,
            tool: &vbit,
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
        let vc_hits = count_cutting_samples_within_rect_clearance(
            &tp_vc,
            -5.0 - ox_vc,
            -10.0 - oy_vc,
            5.0 - ox_vc,
            10.0 - oy_vc,
            6.85,
        );
        assert_eq!(vc_hits, 0, "VCarve cut inside inflated keep-out zone!");
        assert!(
            tp_vc.stats.pass_count >= 2,
            "VCarve should carve both unblocked halves"
        );

        // Also test VCarve with a flat depth + clearing flat endmill (Stage 1 + Stage 2)
        let mut op_vc_flat = op_vc.clone();
        op_vc_flat.id = "op_vc_flat".to_string();
        let vc_flat_params = VCarveParams {
            flat_depth: Some(ParamLength::from_mm(2.0)),
            ..Default::default()
        };
        op_vc_flat.strategy = Some(StrategyParams::VCarve(vc_flat_params));
        let tp_vc_flat = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &cad,
            sketches: &v_sketches,
            setup: &rs_vc,
            op: &op_vc_flat,
            tool: &vbit,
            clearing_tool: Some(&flat6),
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap();
        let vc_flat_hits = count_cutting_samples_within_rect_clearance(
            &tp_vc_flat,
            -5.0 - ox_vc,
            -10.0 - oy_vc,
            5.0 - ox_vc,
            10.0 - oy_vc,
            3.85, // smaller clearing tool radius 3.0 + radial_clearance 1.0 = 4.0
        );
        assert_eq!(
            vc_flat_hits, 0,
            "VCarve with flat-depth clearing cut inside inflated keep-out zone!"
        );
    }

    // 2. DrillBore + Deburr3D on sample_multi_feature_bracket_step (90x70x20 mm, 4x Ø6mm corner holes)
    {
        let step = sample_multi_feature_bracket_step(false).unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("bracket.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let mut setup = Setup::new("s1", "Bracket Setup", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
        };
        // Find all 4 Ø6mm corner holes
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
                    } if (radius_mm - 3.0).abs() < 1e-3
                )
            })
            .unwrap()
            .id;
        let hole_items: Vec<GeometrySelectionItem> =
            find_same_diameter_holes(&doc, r3_seed, 1e-3, 1e-3)
                .into_iter()
                .map(|fid| GeometrySelectionItem::CylindricalHole {
                    binding: doc.bind_face(fid).unwrap(),
                })
                .collect();

        let rs_tmp = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let sum = chip_cam::extract_selection_features(&doc, &rs_tmp, &hole_items).unwrap();
        assert_eq!(sum.features.len(), 4);
        let first_hole_xy = match sum.features[0] {
            chip_cam::ExtractedFeature25D::CylindricalHole { center_wcs, .. } => center_wcs,
            _ => panic!("expected CylindricalHole"),
        };

        // Sketched keep-out centered on the first hole (`14x14 mm`)
        let sk_hole_avoid = make_rect_sketch(
            "sk_hole_avoid",
            "Clamp Over Hole 1",
            "s1",
            first_hole_xy.x - 7.0,
            first_hole_xy.y - 7.0,
            14.0,
            14.0,
        );
        let sketches = vec![sk_hole_avoid];
        let rs = setup
            .resolve_with_sketches(&doc, &sketches, DisplayUnit::Millimeter)
            .unwrap();

        // 2a. DrillBore skips the blocked hole and emits structured warning!
        let drill6 = drill_tool("d6", 3, 6.0);
        let mut op_drill = Operation::new("op_drill", "Drill 4 Holes", OperationKind::DrillBore);
        op_drill.selections = hole_items;
        op_drill.strategy = Some(StrategyParams::DrillBore(DrillBoreParams {
            mode: HoleMachiningMode::Drill,
            ..DrillBoreParams::default()
        }));
        op_drill
            .avoid
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sk_hole_avoid".to_string(),
                sub_layer: None,
                reverse: false,
                side_flip: false,
            });
        let tp_drill = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: &sketches,
            setup: &rs,
            op: &op_drill,
            tool: &drill6,
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
        assert_eq!(
            tp_drill.stats.pass_count, 3,
            "expected 3 of 4 holes drilled"
        );
        assert!(
            tp_drill
                .warnings
                .iter()
                .any(|w| w.contains("Skipped 1 hole(s) inside Avoid / Keep-Out region")),
            "expected structured skip warning, got {:?}",
            tp_drill.warnings
        );

        // 2b. Deburr3D excludes the keep-out corner around `first_hole_xy`!
        let chamf6 = vbit_tool("ch6", 4, 6.0, 90.0);
        let mut op_deb = Operation::new("op_deb", "Deburr Keepout", OperationKind::Deburr3D);
        op_deb.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op_deb.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
        op_deb.strategy = Some(StrategyParams::Deburr3D(Deburr3DParams::default()));
        op_deb
            .avoid
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sk_hole_avoid".to_string(),
                sub_layer: None,
                reverse: false,
                side_flip: false,
            });
        op_deb.avoid.radial_clearance = ParamLength::from_mm(1.0);
        let tp_deb = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: &sketches,
            setup: &rs,
            op: &op_deb,
            tool: &chamf6,
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
        // Inflated keepout = half 7 + tool_r 3 + clr 1 = 11 mm around first_hole_xy
        let deb_hits = count_cutting_samples_within_rect_clearance(
            &tp_deb,
            first_hole_xy.x - 7.0,
            first_hole_xy.y - 7.0,
            first_hole_xy.x + 7.0,
            first_hole_xy.y + 7.0,
            3.85,
        );
        assert_eq!(deb_hits, 0, "Deburr3D cut inside inflated keep-out zone!");
        assert!(tp_deb.stats.pass_count >= 1);
    }

    // 3. Waterline3D + Scallop3D + 100%-excluded EmptyToolpath check on sculpted_dome.stl
    {
        let dome_stl = sample_sculpted_dome_stl_bytes();
        let doc = load_mesh_cad_document(
            "sculpted_dome.stl",
            &dome_stl,
            &MeshImportOptions::default(),
        )
        .unwrap();
        let mut setup = Setup::new("s1", "Dome", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
        };
        let sk_half =
            make_rect_sketch("sk_right", "Right Half Avoid", "s1", 45.0, -5.0, 35.0, 80.0);
        let sk_full = make_rect_sketch("sk_full", "Full Avoid", "s1", -10.0, -10.0, 90.0, 90.0);
        let sketches = vec![sk_half, sk_full];
        let rs = setup
            .resolve_with_sketches(&doc, &sketches, DisplayUnit::Millimeter)
            .unwrap();
        let ball6 = ball_tool("ball_6", 1, 6.0, 24.0, 40.0);

        // Waterline3D excludes X >= 45 - 3 - 0.5 = 41.5
        let mut op_wl = Operation::new("op_wl", "Waterline Keepout", OperationKind::Waterline3D);
        op_wl.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op_wl.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 12.0);
        op_wl.passes.max_stepdown = ParamLength::from_mm(4.0);
        op_wl.strategy = Some(StrategyParams::Waterline3D(Waterline3DParams::default()));
        op_wl
            .avoid
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sk_right".to_string(),
                sub_layer: None,
                reverse: false,
                side_flip: false,
            });
        let tp_wl = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: &sketches,
            setup: &rs,
            op: &op_wl,
            tool: &ball6,
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
        assert_eq!(
            count_cutting_samples_inside_rect(&tp_wl, 41.7, -5.0, 80.0, 75.0),
            0,
            "Waterline3D cut inside keep-out!"
        );

        // Scallop3D with 100%-covering avoid returns EmptyToolpath cleanly
        let mut op_sc = Operation::new("op_sc", "Scallop Blocked", OperationKind::Scallop3D);
        op_sc.strategy = Some(StrategyParams::Scallop3D(Scallop3DParams::default()));
        op_sc
            .avoid
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sk_full".to_string(),
                sub_layer: None,
                reverse: false,
                side_flip: false,
            });
        let err = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: &sketches,
            setup: &rs,
            op: &op_sc,
            tool: &ball6,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap_err();
        assert!(
            matches!(err, CamError::EmptyToolpath { .. }),
            "100%-excluded Scallop3D must return EmptyToolpath, got {err:?}"
        );
    }
}

#[test]
fn rotary_axial_keepout_splits_passes_with_safe_retracts_and_passes_chip_sim() {
    let stl_bytes = build_rotary_cylinder_stl_bytes(-30.0, 30.0, 12.0, 3.0);
    let cad_doc =
        load_mesh_cad_document("rotary_lobe.stl", &stl_bytes, &MeshImportOptions::default())
            .unwrap();

    let mut project = ProjectDocument::new("Rotary Axial Keep-Out");
    let ball6 = ball_tool("ball_6", 1, 6.0, 24.0, 42.0);
    project.tools.push(ball6.clone());

    let setup = &mut project.setups[0];
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::VOLUMETRIC_CENTER,
    };
    setup.stock = StockConfig::FixedCylinder {
        axis: StockAxis::X,
        diameter: ParamLength::from_mm(34.0),
        length: ParamLength::from_mm(60.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
    setup.rotary.axis = RotaryAxisKind::XAxis;
    setup.rotary.chuck_length = ParamLength::from_mm(25.0);
    setup.rotary.chuck_diameter = ParamLength::from_mm(60.0);
    setup.rotary.clearance_radial_offset = ParamLength::from_mm(9.0);

    // Keep-out sketch band spanning X = [-4..+4] (with tool_r = 3.0 + axial_clearance = 1.0 -> effective [-8..+8])
    let sk_band = make_rect_sketch(
        "sk_band",
        "Steady Rest Band",
        &setup.id,
        -4.0,
        -15.0,
        8.0,
        30.0,
    );
    project.sketches.push(sk_band);

    // Op 1: Rotary2Linear (ContinuousSpiral)
    let mut op_2lin = Operation::new("op_r2l", "Rotary 2-Linear", OperationKind::Rotary2Linear);
    op_2lin.tool_id = Some(ball6.id.clone());
    op_2lin.passes.multiple_depths = false;
    let p2l = Rotary2LinearParams {
        pattern: RotaryPatternMode::ContinuousSpiral,
        stepover: ParamLength::from_mm(2.5),
        use_cusp_height: false,
        ..Default::default()
    };
    op_2lin.strategy = Some(StrategyParams::Rotary2Linear(p2l));
    op_2lin
        .avoid
        .selections
        .push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sk_band".to_string(),
            sub_layer: None,
            reverse: false,
            side_flip: false,
        });
    op_2lin.avoid.axial_clearance = ParamLength::from_mm(1.0);
    project.setups[0].operations.push(op_2lin);

    // Op 2: Rotary4AxisSim (HelicalFlowline)
    let mut op_4ax = Operation::new("op_r4a", "Rotary 4-Axis Sim", OperationKind::Rotary4AxisSim);
    op_4ax.tool_id = Some(ball6.id.clone());
    op_4ax.passes.multiple_depths = false;
    let p4a = Rotary4AxisSimParams {
        pattern: Rotary4AxisPattern::HelicalFlowline,
        stepover: ParamLength::from_mm(3.0),
        use_cusp_height: false,
        ..Default::default()
    };
    op_4ax.strategy = Some(StrategyParams::Rotary4AxisSim(p4a));
    op_4ax
        .avoid
        .selections
        .push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sk_band".to_string(),
            sub_layer: None,
            reverse: false,
            side_flip: false,
        });
    op_4ax.avoid.axial_clearance = ParamLength::from_mm(1.0);
    project.setups[0].operations.push(op_4ax);

    let mut cache = ToolpathCache::new(16);
    let gen_rep = generate_project_blocking(&mut project, &cad_doc, &mut cache, 1);
    assert!(
        gen_rep.is_ok(),
        "rotary avoid gen errors: {:?}",
        gen_rep.errors
    );

    // Verify BOTH rotary strategies stay strictly outside X in (-7.95, +7.95), cut BOTH sides,
    // and only cross [-8..+8] at safe radial retract Z >= 25.0 mm!
    for (o_idx, label) in [(0, "Rotary2Linear"), (1, "Rotary4AxisSim")] {
        let (_, tp) = cache.get_for_operation(&project, 0, o_idx, 1).unwrap();
        let mut cut_left = false;
        let mut cut_right = false;
        let mut crossed_at_safe_z = false;

        for seg in &tp.segments_wcs {
            let x0 = seg.start_wcs.x;
            let x1 = seg.end_wcs.x;
            if matches!(seg.motion, SegmentMotion3D::Rapid) {
                if (x0 < -7.5 && x1 > 7.5) || (x0 > 7.5 && x1 < -7.5) {
                    assert!(
                        seg.start_wcs.z >= 25.0 && seg.end_wcs.z >= 25.0,
                        "{label}: rapid across excluded band must stay at safe radial retract Z >= 25, got z0={:.2}, z1={:.2}",
                        seg.start_wcs.z,
                        seg.end_wcs.z
                    );
                    crossed_at_safe_z = true;
                }
            } else {
                let x_lo = x0.min(x1);
                let x_hi = x0.max(x1);
                assert!(
                    x_hi <= -7.95 || x_lo >= 7.95,
                    "{label}: cutting segment [{x0:.2}..{x1:.2}] intruded into excluded axial band (-8, +8)!"
                );
                if x_hi <= -7.95 {
                    cut_left = true;
                }
                if x_lo >= 7.95 {
                    cut_right = true;
                }
            }
        }
        assert!(
            cut_left && cut_right,
            "{label}: expected cuts on both sides of keep-out band"
        );
        assert!(
            crossed_at_safe_z,
            "{label}: expected safe radial rapid retract across excluded band"
        );
    }

    // Verify against BOTH CylindricalStock (`Heightmap`) and volumetric `TriDexel` in `chip-sim`!
    let kin = resolve_kinematics_for_key(&project.setups[0].post_profile, None);
    for engine in [SimEngineKind::Heightmap, SimEngineKind::TriDexel] {
        let sim = simulate_project_setup_with_engine(
            &SetupSimRequest {
                project: &project,
                setup_idx: 0,
                cad_doc: &cad_doc,
                cad_revision: 1,
                cache: &cache,
                kinematics: kin,
                resolution: SimResolution::Standard,
                cancel: None,
            },
            engine,
        )
        .unwrap();
        let bad: Vec<_> = sim
            .diagnostics
            .iter()
            .filter(|d| {
                matches!(
                    d.kind,
                    SimIssueKind::RapidIntoStock
                        | SimIssueKind::TargetModelGouge
                        | SimIssueKind::FixtureCollision
                )
            })
            .collect();
        assert!(
            bad.is_empty(),
            "Rotary avoid ({engine:?}): expected 0 crashes/gouges, got {bad:#?}"
        );
    }
}

#[test]
fn indexed_3plus1_transforms_fixtures_into_virtual_frame() {
    let cancel = AtomicBool::new(false);
    let box_bytes = box_stl_bytes(-20.0, -15.0, -15.0, 20.0, 15.0, 15.0);
    let cad_doc = load_mesh_cad_document(
        "indexed_block.stl",
        &box_bytes,
        &MeshImportOptions::default(),
    )
    .unwrap();

    let mut setup = Setup::new("s1", "Indexed 3+1", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27::VOLUMETRIC_CENTER,
    };
    setup.stock = StockConfig::FixedCylinder {
        axis: StockAxis::X,
        diameter: ParamLength::from_mm(34.0),
        length: ParamLength::from_mm(40.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup.rotary.mode = RotaryDriveMode::Indexed3Plus1;
    setup.rotary.axis = RotaryAxisKind::XAxis;
    setup.rotary.chuck_length = ParamLength::from_mm(20.0);
    setup.rotary.chuck_diameter = ParamLength::from_mm(50.0);
    setup.rotary.clearance_radial_offset = ParamLength::from_mm(9.0);

    // Place a box keep-out fixture strictly on the TOP (+Z) side of the stock at Z=[10..20], X=[-5..5], Y=[-5..5].
    let mut top_fix =
        FixtureBody::default_box_keepout("top_blk", "Top Clamp Block", DVec3::new(0.0, 0.0, 15.0));
    if let chip_cam::FixtureKind::BoxKeepOut { size_wcs, .. } = &mut top_fix.kind {
        *size_wcs = [
            ParamLength::from_mm(10.0),
            ParamLength::from_mm(10.0),
            ParamLength::from_mm(10.0),
        ];
    }
    setup.fixtures.push(top_fix);

    let rs = setup.resolve(&cad_doc, DisplayUnit::Millimeter).unwrap();
    let fixtures = resolve_setup_fixtures_parametric(&setup, &rs, DisplayUnit::Millimeter).unwrap();
    let flat6 = flat_tool("flat_6", 1, 6.0, 20.0, 35.0);

    // At A=0° (top side), the fixture at Z=[10..20] overlaps the facing pass at Z=[14..17] -> MUST be excluded!
    let mut op_0 = Operation::new("op_a0", "Face A0", OperationKind::Facing);
    op_0.rotary = OperationRotaryMode::Indexed3Plus1 { angle_deg: 0.0 };
    op_0.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_0.heights.bottom = HeightSpec::new(HeightReference::StockTop, -2.0);
    let mut face_p = chip_cam::FacingParams::default();
    face_p.boundary.boundary_mode = MachiningBoundaryMode::ModelBoundingBox;
    face_p.boundary.tool_containment = ToolContainmentMode::ToolInsideBoundary;
    face_p.boundary.model_bounds_offset_x_neg = ParamLength::from_mm(-2.0);
    op_0.strategy = Some(StrategyParams::Facing(face_p));
    op_0.avoid = AvoidRegionConfig {
        avoid_all_fixtures: true,
        radial_clearance: ParamLength::from_mm(0.5),
        axial_clearance: ParamLength::from_mm(0.5),
        ..AvoidRegionConfig::default()
    };
    let tp_0 = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &cad_doc,
        sketches: &[],
        setup: &rs,
        op: &op_0,
        tool: &flat6,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &fixtures,
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();
    let hits_a0 = count_cutting_samples_within_rect_clearance(&tp_0, -5.0, -5.0, 5.0, 5.0, 3.35);
    assert_eq!(
        hits_a0, 0,
        "Indexed A=0° must exclude the top-side fixture centered at (0, 0)!"
    );

    // At A=180° (opposite bottom side), the top-side fixture rotates to virtual Z=[-20..-10],
    // strictly BELOW virtual facing Z=[14..17] -> MUST NOT exclude the center!
    let mut op_180 = op_0.clone();
    op_180.id = "op_a180".to_string();
    op_180.rotary = OperationRotaryMode::Indexed3Plus1 { angle_deg: 180.0 };
    let tp_180 = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &cad_doc,
        sketches: &[],
        setup: &rs,
        op: &op_180,
        tool: &flat6,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &fixtures,
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();
    let hits_a180 = count_cutting_samples_inside_rect(&tp_180, -3.0, -3.0, 3.0, 3.0);
    assert!(
        hits_a180 > 0,
        "Indexed A=180° rotated the top fixture below virtual bottom Z, so center should be machined!"
    );
}
