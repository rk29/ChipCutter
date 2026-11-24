#![allow(clippy::unwrap_used, clippy::expect_used)]

//! Milestone 73 integration tests:
//! - Rotary-angle-scoped 2D rest filtering (`filter_upstream_for_indexed_angle`)
//! - In-setup cross-angle volumetric `TriDexelStock` carryover for `Indexed3Plus1` (`0°`, `120°`, `240°`)
//! - Same-angle `Indexed3Plus1` 2D rest preservation (`90°` Ø8 -> `90°` Ø3)
//! - Exact swept Z-slab target masking & cache invalidation
//! - Torture project sanity check (`torture/TripleHelix-Indexed.chip.toml` & `DoubleHelix-Indexed.chip.toml`)

use std::{
    collections::HashSet,
    path::PathBuf,
    time::{Duration, Instant},
};

use chip_cad::{MeshImportOptions, ModelTransform, load_mesh_cad_document};
use chip_cam::{
    AxisAnchor, BoxAnchor27, GeometrySelectionItem, HeightReference, HeightSpec,
    MachiningBoundaryMode, Operation, OperationGenRequest, OperationKind, OperationRotaryMode,
    OperationStatus, RestMachiningConfig, RestMachiningMode, RotaryAxisKind, RotaryDriveMode,
    Setup, StockAxis, StockConfig, StrategyParams, ToolContainmentMode, UpstreamSweptInput,
    WcsOriginMode, WorkOffsetSlot, generate_operation_toolpath_with_rest,
};
use chip_math::{DVec3, DisplayUnit, ParamLength};
use chip_project::{
    JobDispatcher, ModelEntry, ProjectDocument, SetupSimRequest, ToolpathCache,
    cache::{compute_operation_input_hash, op_needs_in_setup_rotary_carryover},
    compute_sha256_hex, generate_project_blocking, generate_project_blocking_in_dir,
    load_model_entry_cad_doc_transformed, resolve_kinematics_for_key,
    simulate_project_setup_with_choice,
};
use chip_sim::{SimComputeBackendChoice, SimEngineKind, SimIssueKind, SimResolution};
use chip_tools::{CutterGeometry, Tool, ToolId, default_starter_tool_library};

/// Builds binary STL bytes for a watertight 3-flute twisted helical loft along X ∈ `[-25, +25]`
/// (`r(θ, x) = r_core + amp * (0.5 + 0.5 * cos(3 * (θ - twist(x))))`), rotating `90°` end-to-end.
fn build_twisted_3flute_helical_loft_stl_bytes() -> Vec<u8> {
    let x0 = -24.0_f64;
    let x1 = 24.0_f64;
    let nx = 24usize;
    let nt = 48usize;
    let r_core = 8.0_f64;
    let r_lobe = 17.0_f64;
    let total_twist_rad = 90.0_f64.to_radians();

    let mut tris: Vec<[DVec3; 3]> = Vec::new();
    let sample_pt = |ix: usize, it: usize| -> DVec3 {
        let u = ix as f64 / nx as f64;
        let x = x0 + u * (x1 - x0);
        let theta = (it % nt) as f64 * (std::f64::consts::TAU / nt as f64);
        let phase = 3.0 * (theta - u * total_twist_rad);
        let lobe = (0.5 + 0.5 * phase.cos()).powf(1.5);
        let r = r_core + (r_lobe - r_core) * lobe;
        DVec3::new(x, r * theta.cos(), r * theta.sin())
    };

    // Cylindrical helical loft skin (outward normals)
    for ix in 0..nx {
        for it in 0..nt {
            let p00 = sample_pt(ix, it);
            let p10 = sample_pt(ix + 1, it);
            let p11 = sample_pt(ix + 1, it + 1);
            let p01 = sample_pt(ix, it + 1);
            tris.push([p00, p10, p11]);
            tris.push([p00, p11, p01]);
        }
    }

    // End caps (`-X` at ix=0, `+X` at ix=nx)
    let c0 = DVec3::new(x0, 0.0, 0.0);
    let c1 = DVec3::new(x1, 0.0, 0.0);
    for it in 0..nt {
        let a0 = sample_pt(0, it);
        let b0 = sample_pt(0, it + 1);
        tris.push([c0, b0, a0]);

        let a1 = sample_pt(nx, it);
        let b1 = sample_pt(nx, it + 1);
        tris.push([c1, a1, b1]);
    }

    let mut bytes = Vec::with_capacity(84 + tris.len() * 50);
    bytes.extend_from_slice(&[0u8; 80]);
    bytes.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for [v0, v1, v2] in tris {
        let n = (v1 - v0).cross(v2 - v0).normalize_or_zero();
        for c in [
            n.x, n.y, n.z, v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z,
        ] {
            bytes.extend_from_slice(&(c as f32).to_le_bytes());
        }
        bytes.extend_from_slice(&0u16.to_le_bytes());
    }
    bytes
}

const BOX_CENTER: BoxAnchor27 = BoxAnchor27 {
    x: AxisAnchor::Center,
    y: AxisAnchor::Center,
    z: AxisAnchor::Center,
};

fn make_flat_tool(id: &str, tool_number: u32, diameter_mm: f64, flute_mm: f64) -> Tool {
    let mut tool = default_starter_tool_library().tools[0].clone();
    tool.id = ToolId(id.to_string());
    tool.tool_number = tool_number;
    tool.length_offset_number = tool_number;
    tool.diameter_offset_number = tool_number;
    tool.name = format!("Flat Ø{diameter_mm}mm");
    tool.stickout_length_mm = flute_mm + 8.0;
    tool.geometry = CutterGeometry::FlatEndmill {
        diameter_mm,
        flute_length_mm: flute_mm,
        overall_length_mm: flute_mm + 24.0,
        shank_diameter_mm: diameter_mm,
        flutes: 3,
    };
    tool
}

fn build_3angle_helix_project(enable_rest: bool, sha: &str) -> ProjectDocument {
    let mut proj = ProjectDocument::new("M73 3-Angle Helical Loft");
    proj.models.push(ModelEntry {
        id: "helix_1".to_string(),
        path: "twisted_3flute.stl".to_string(),
        sha256: sha.to_string(),
        role: Default::default(),
        unit_override: None,
        transform: Default::default(),
    });
    let tool = make_flat_tool("t1_6mm_flat", 1, 6.0, 26.0);
    proj.tools = vec![tool];

    let mut setup = Setup::new("setup_1", "Rotary 3-Angle Indexed", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint { anchor: BOX_CENTER };
    setup.stock = StockConfig::FixedCylinder {
        axis: StockAxis::X,
        diameter: ParamLength::from_mm(40.0),
        length: ParamLength::from_mm(56.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup.rotary.mode = RotaryDriveMode::Indexed3Plus1;
    setup.rotary.axis = RotaryAxisKind::XAxis;
    setup.rotary.clearance_radial_offset = ParamLength::from_mm(10.0);
    // Full-stock StockBox indexed carryover test without a jaw keep-out collar (matching M57)
    setup.rotary.chuck_length = ParamLength::from_mm(0.0);
    setup.rotary.chuck_diameter = ParamLength::from_mm(55.0);

    for (idx, angle_deg) in [0.0_f64, 120.0, 240.0].into_iter().enumerate() {
        let mut op = Operation::new(
            format!("op_idx_{}", angle_deg as i32),
            format!("Adaptive3D {}°", angle_deg as i32),
            OperationKind::Adaptive3D,
        );
        op.tool_id = Some(ToolId("t1_6mm_flat".to_string()));
        op.rotary = OperationRotaryMode::Indexed3Plus1 { angle_deg };
        op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op.heights.bottom = HeightSpec::new(HeightReference::WcsOrigin, -2.0);
        op.passes.stock_to_leave_radial = ParamLength::from_mm(0.5);
        op.passes.stock_to_leave_axial = ParamLength::from_mm(0.3);
        if enable_rest && idx > 0 {
            op.passes.rest_machining = RestMachiningConfig {
                mode: RestMachiningMode::FromPrecedingOperations {
                    source_op_ids: Vec::new(),
                },
                rest_overlap: ParamLength::from_mm(0.25),
                min_rest_thickness: ParamLength::from_mm(0.05),
            };
        }
        if let Some(StrategyParams::Adaptive3D(ref mut p)) = op.strategy {
            p.boundary.boundary_mode = MachiningBoundaryMode::StockBox;
            p.boundary.tool_containment = ToolContainmentMode::ToolOutsideBoundary;
            p.max_roughing_stepdown = ParamLength::from_mm(6.0);
            p.fine_stepdown = ParamLength::from_mm(0.0);
            p.adaptive.optimal_load = ParamLength::from_mm(2.2);
        }
        setup.operations.push(op);
    }

    proj.setups = vec![setup];
    proj
}

#[test]
fn three_angle_indexed_adaptive3d_volumetric_carryover_reduces_air_cutting_with_zero_gouges_or_crashes()
 {
    let stl = build_twisted_3flute_helical_loft_stl_bytes();
    let cad_doc =
        load_mesh_cad_document("twisted_3flute.stl", &stl, &MeshImportOptions::default()).unwrap();
    let sha = compute_sha256_hex(&stl);

    // 1. Baseline: raw-stock roughing at 0°, 120°, 240° (Rest Machining Disabled)
    let mut proj_raw = build_3angle_helix_project(false, &sha);
    let mut cache_raw = ToolpathCache::new(32);
    let rep_raw = generate_project_blocking(&mut proj_raw, &cad_doc, &mut cache_raw, 1);
    assert!(
        rep_raw.is_ok(),
        "raw generation errors: {:?}",
        rep_raw.errors
    );
    assert_eq!(rep_raw.generated.len(), 3);

    let raw_120_240_cut_mm: f64 = rep_raw.generated[1..3]
        .iter()
        .map(|(_, _, tp)| tp.stats.cutting_distance_mm)
        .sum();

    // 1b. Direct cross-angle 2D rest isolation check:
    // Passing 0° upstream 2D toolpath into 120° op with `carryover_stock: None` MUST NOT subtract
    // any 0° XY capsules in 120° virtual XY space (cutting distance matches raw 120° identically!).
    let resolved_setup = proj_raw.setups[0]
        .resolve(&cad_doc, DisplayUnit::Millimeter)
        .unwrap();
    let cancel = std::sync::atomic::AtomicBool::new(false);
    let cross_angle_upstream = [UpstreamSweptInput {
        op_id: proj_raw.setups[0].operations[0].id.clone(),
        tool_diameter_mm: proj_raw.tools[0].geometry.cutting_diameter_mm(),
        corner_radius_mm: 0.0,
        toolpath: std::sync::Arc::clone(&rep_raw.generated[0].2),
        indexed_angle_deg: Some(0.0),
        is_rotary_native: false,
    }];
    let tp_120_with_0_upstream = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &cad_doc,
        sketches: &[],
        setup: &resolved_setup,
        op: &proj_raw.setups[0].operations[1],
        tool: &proj_raw.tools[0],
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &cross_angle_upstream,
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();
    assert!(
        (tp_120_with_0_upstream.stats.cutting_distance_mm
            - rep_raw.generated[1].2.stats.cutting_distance_mm)
            .abs()
            < 1e-3,
        "0° 2D swept capsules must NEVER be subtracted in 120° XY coordinates! got {:.2} vs raw {:.2}",
        tp_120_with_0_upstream.stats.cutting_distance_mm,
        rep_raw.generated[1].2.stats.cutting_distance_mm
    );

    // 2. M73: volumetric cross-angle Tri-Dexel carryover enabled on 120° and 240°
    let mut proj_rest = build_3angle_helix_project(true, &sha);
    assert!(!op_needs_in_setup_rotary_carryover(&proj_rest.setups[0], 0));
    assert!(op_needs_in_setup_rotary_carryover(&proj_rest.setups[0], 1));
    assert!(op_needs_in_setup_rotary_carryover(&proj_rest.setups[0], 2));

    let mut cache_rest = ToolpathCache::new(32);
    let rep_rest = generate_project_blocking(&mut proj_rest, &cad_doc, &mut cache_rest, 1);
    assert!(
        rep_rest.is_ok(),
        "carryover generation errors: {:?}",
        rep_rest.errors
    );
    assert_eq!(rep_rest.generated.len(), 3);

    // 0° op must match baseline identically
    let rest_0_cut_mm = cache_rest
        .get_for_operation(&proj_rest, 0, 0, 1)
        .unwrap()
        .1
        .stats
        .cutting_distance_mm;
    let raw_0_cut_mm = rep_raw.generated[0].2.stats.cutting_distance_mm;
    assert!(
        (rest_0_cut_mm - raw_0_cut_mm).abs() < 1e-3,
        "0° op should be unchanged: rest={rest_0_cut_mm:.2} vs raw={raw_0_cut_mm:.2}"
    );

    let rest_120_tp = cache_rest.get_for_operation(&proj_rest, 0, 1, 1).unwrap().1;
    let rest_240_tp = cache_rest.get_for_operation(&proj_rest, 0, 2, 1).unwrap().1;
    let rest_120_240_cut_mm =
        rest_120_tp.stats.cutting_distance_mm + rest_240_tp.stats.cutting_distance_mm;

    let reduction_pct = (1.0 - rest_120_240_cut_mm / raw_120_240_cut_mm) * 100.0;
    assert!(
        reduction_pct >= 25.0,
        "Volumetric carryover must reduce 120°+240° cutting distance by >= 25%, got {reduction_pct:.1}% (raw={raw_120_240_cut_mm:.1} mm -> rest={rest_120_240_cut_mm:.1} mm)"
    );

    // 3. Full multi-op rotary Tri-Dexel verification: ZERO gouges & ZERO rapid-into-stock crashes!
    let kin = resolve_kinematics_for_key(&proj_rest.setups[0].post_profile, None);
    let sim = simulate_project_setup_with_choice(
        &SetupSimRequest {
            project: &proj_rest,
            setup_idx: 0,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache_rest,
            kinematics: kin,
            resolution: SimResolution::Standard,
            cancel: None,
        },
        SimEngineKind::TriDexel,
        SimComputeBackendChoice::Cpu,
    )
    .unwrap();

    let rapid_crashes: Vec<_> = sim
        .diagnostics
        .iter()
        .filter(|d| d.kind == SimIssueKind::RapidIntoStock)
        .collect();
    let gouges: Vec<_> = sim
        .diagnostics
        .iter()
        .filter(|d| d.kind == SimIssueKind::TargetModelGouge)
        .collect();
    assert!(
        rapid_crashes.is_empty(),
        "Expected 0 RapidIntoStock crashes, got {}: {rapid_crashes:#?}",
        rapid_crashes.len()
    );
    assert!(
        gouges.is_empty(),
        "Expected 0 TargetModelGouge diagnostics, got {}: {gouges:#?}",
        gouges.len()
    );
    if let Some(err_rep) = &sim.error_report {
        assert!(
            err_rep.gouge_volume_mm3 <= 1e-6,
            "Expected 0.0 mm³ target gouge volume, got {:.4} mm³",
            err_rep.gouge_volume_mm3
        );
    }

    // 4. Cache invalidation: editing Op 0 (0°) must change hashes and mark 120° & 240° Dirty!
    let h1_before = compute_operation_input_hash(&proj_rest, 0, 1, 1).unwrap();
    let h2_before = compute_operation_input_hash(&proj_rest, 0, 2, 1).unwrap();
    proj_rest.setups[0].operations[0]
        .passes
        .stock_to_leave_radial = ParamLength::from_mm(0.8);
    let h1_after = compute_operation_input_hash(&proj_rest, 0, 1, 1).unwrap();
    let h2_after = compute_operation_input_hash(&proj_rest, 0, 2, 1).unwrap();
    assert_ne!(
        h1_before, h1_after,
        "Editing 0° op must invalidate 120° op hash"
    );
    assert_ne!(
        h2_before, h2_after,
        "Editing 0° op must invalidate 240° op hash"
    );

    cache_rest.reconcile_project_statuses(&mut proj_rest, 1, &HashSet::new());
    assert_eq!(
        proj_rest.setups[0].operations[0].runtime_status,
        OperationStatus::Dirty
    );
    assert_eq!(
        proj_rest.setups[0].operations[1].runtime_status,
        OperationStatus::Dirty
    );
    assert_eq!(
        proj_rest.setups[0].operations[2].runtime_status,
        OperationStatus::Dirty
    );

    // 5. Async JobDispatcher wave execution must also complete all 3 ops cleanly!
    let mut dispatcher = JobDispatcher::new(None);
    let cad_arc = std::sync::Arc::new(cad_doc);
    let queued =
        dispatcher.dispatch_operations(&mut proj_rest, cad_arc, 1, &cache_rest, None, None);
    assert_eq!(queued, 3);

    let started = Instant::now();
    let deadline = started + Duration::from_secs(300);
    while dispatcher.is_busy() {
        assert!(
            Instant::now() < deadline,
            "JobDispatcher timed out on 3-angle carryover wave after {:.1}s ({} in-flight/pending)",
            started.elapsed().as_secs_f64(),
            dispatcher.in_flight_count(),
        );
        let _ = dispatcher.poll_events(&mut proj_rest, &mut cache_rest, 1);
        std::thread::sleep(Duration::from_millis(10));
    }
    let _ = dispatcher.poll_events(&mut proj_rest, &mut cache_rest, 1);
    for (o_idx, op) in proj_rest.setups[0].operations.iter().enumerate() {
        assert_eq!(
            op.runtime_status,
            OperationStatus::Clean,
            "Async op #{o_idx} did not reach Clean"
        );
    }
}

#[test]
fn same_angle_indexed_2d_rest_preserves_exact_2d_swept_coverage() {
    // Prismatic block with a sharp-cornered rectangular pocket on the +Y face (indexed at 90°).
    // Two sequential Indexed3Plus1 ops at the SAME 90° angle: Ø8 rough -> Ø3 rest cleanup.
    // Must use exact 2D `swept_toolpath_coverage_2d` (no cross-angle Tri-Dexel carryover needed)
    // and cut drastically less than a full Ø3 pocket clear!
    let step = chip_cad::sample_multi_feature_bracket_step(false).unwrap();
    let raw_doc = chip_cad::MonstertruckKernel
        .load_step_bytes(
            "bracket.step",
            step.as_bytes(),
            chip_math::DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    // Rotate bracket by -90° around X so its +Z pocket face points along +Y_wcs, meaning
    // `Indexed3Plus1 { angle_deg: 90.0 }` rotates the pocket face straight up under the spindle!
    let cad_doc = raw_doc.transformed(&ModelTransform {
        rotate_deg: [-90.0, 0.0, 0.0],
        ..Default::default()
    });
    use chip_cad::CadKernel;

    // Pick Pocket A floor (now facing +Y_wcs with an inner island loop)
    let pkt_face = cad_doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                chip_cad::SurfaceKind::Plane { normal, .. }
                if normal.y > 0.99 && !f.inner_loops.is_empty()
            )
        })
        .unwrap();
    let pkt_bind = cad_doc.bind_face(pkt_face.id).unwrap();

    let make_proj = |use_rest: bool| -> ProjectDocument {
        let mut proj = ProjectDocument::new("Same-Angle 90° Rest");
        proj.tools = vec![
            make_flat_tool("t_8mm", 1, 8.0, 20.0),
            make_flat_tool("t_3mm", 2, 3.0, 14.0),
        ];
        let mut setup = Setup::new("s1", "Indexed 90° Setup", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint { anchor: BOX_CENTER };
        setup.rotary.mode = RotaryDriveMode::Indexed3Plus1;
        setup.rotary.axis = RotaryAxisKind::XAxis;

        let mut op1 = Operation::new_adaptive_2d("op_rough_90", "90° Ø8 Rough");
        op1.tool_id = Some(ToolId("t_8mm".to_string()));
        op1.rotary = OperationRotaryMode::Indexed3Plus1 { angle_deg: 90.0 };
        op1.selections = vec![GeometrySelectionItem::PlanarFace {
            binding: pkt_bind.clone(),
        }];

        let mut op2 = Operation::new_adaptive_2d("op_rest_90", "90° Ø3 Rest");
        op2.tool_id = Some(ToolId("t_3mm".to_string()));
        op2.rotary = OperationRotaryMode::Indexed3Plus1 { angle_deg: 90.0 };
        op2.selections = vec![GeometrySelectionItem::PlanarFace {
            binding: pkt_bind.clone(),
        }];
        if use_rest {
            op2.passes.rest_machining = RestMachiningConfig {
                mode: RestMachiningMode::FromPrecedingOperations {
                    source_op_ids: vec!["op_rough_90".to_string()],
                },
                rest_overlap: ParamLength::from_mm(0.25),
                min_rest_thickness: ParamLength::from_mm(0.06),
            };
        }
        setup.operations = vec![op1, op2];
        proj.setups = vec![setup];
        proj
    };

    let mut proj_full = make_proj(false);
    let mut cache_full = ToolpathCache::new(16);
    let rep_full = generate_project_blocking(&mut proj_full, &cad_doc, &mut cache_full, 1);
    assert!(rep_full.is_ok(), "full errors: {:?}", rep_full.errors);
    let full_3_cut = cache_full
        .get_for_operation(&proj_full, 0, 1, 1)
        .unwrap()
        .1
        .stats
        .cutting_distance_mm;

    let mut proj_rest = make_proj(true);
    // Same-angle 90° -> 90° MUST NOT trigger cross-angle Tri-Dexel carryover!
    assert!(!op_needs_in_setup_rotary_carryover(&proj_rest.setups[0], 1));

    let mut cache_rest = ToolpathCache::new(16);
    let rep_rest = generate_project_blocking(&mut proj_rest, &cad_doc, &mut cache_rest, 1);
    assert!(rep_rest.is_ok(), "rest errors: {:?}", rep_rest.errors);
    let rest_3_cut = cache_rest
        .get_for_operation(&proj_rest, 0, 1, 1)
        .unwrap()
        .1
        .stats
        .cutting_distance_mm;

    assert!(
        rest_3_cut < 0.45 * full_3_cut,
        "Same-angle 90° Ø8 -> Ø3 2D rest must cut < 45% of full Ø3 clear: rest={rest_3_cut:.1} mm vs full={full_3_cut:.1} mm"
    );
}

#[test]
fn torture_triple_and_double_helix_indexed_projects_sanity_check() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let torture_dir = manifest_dir.join("../../torture");

    for file_name in [
        "TripleHelix-Indexed.chip.toml",
        "DoubleHelix-Indexed.chip.toml",
    ] {
        let proj_path = torture_dir.join(file_name);
        if !proj_path.exists() {
            continue;
        }
        let toml_str = std::fs::read_to_string(&proj_path).unwrap();
        let mut proj = ProjectDocument::from_toml_str(&toml_str).unwrap();
        let model_entry = &proj.models[0];
        let model_path = torture_dir.join(&model_entry.path);
        let cad_doc = load_model_entry_cad_doc_transformed(
            &model_path,
            model_entry.unit_override,
            &model_entry.transform,
        )
        .unwrap();

        // Verify cross-angle rest detection on the real torture project configuration
        if file_name == "TripleHelix-Indexed.chip.toml" {
            assert!(!op_needs_in_setup_rotary_carryover(&proj.setups[0], 0));
            assert!(op_needs_in_setup_rotary_carryover(&proj.setups[0], 1));
            assert!(op_needs_in_setup_rotary_carryover(&proj.setups[0], 2));
            assert!(op_needs_in_setup_rotary_carryover(&proj.setups[0], 3));
        }

        // Run a fast 2-angle (`0°` -> `90°`) coarse check in debug builds so CI stays fast while
        // exercising the real STEP helical B-Rep end-to-end (full 4-angle torture benchmark lives
        // in `torture_triple_helix_full_4angle_indexed_carryover` below).
        let mut kept_indexed = 0usize;
        for op in &mut proj.setups[0].operations {
            if op.kind.is_rotary_native() {
                op.suppressed = true;
                continue;
            }
            if !op.suppressed {
                kept_indexed += 1;
                if kept_indexed > 2 {
                    op.suppressed = true;
                    continue;
                }
                if let Some(StrategyParams::Adaptive3D(ref mut p)) = op.strategy {
                    p.max_roughing_stepdown = ParamLength::from_mm(8.0);
                    p.fine_stepdown = ParamLength::from_mm(0.0);
                    p.adaptive.optimal_load = ParamLength::from_mm(1.6);
                }
            }
        }

        let mut cache = ToolpathCache::new(32);
        let rep = generate_project_blocking_in_dir(
            &mut proj,
            &cad_doc,
            Some(&torture_dir),
            &mut cache,
            1,
        );
        assert!(
            rep.is_ok(),
            "{file_name} generation errors: {:?}",
            rep.errors
        );
        assert!(
            !rep.generated.is_empty(),
            "{file_name} produced no generated ops"
        );

        let kin = resolve_kinematics_for_key(&proj.setups[0].post_profile, Some(&torture_dir));
        let sim = simulate_project_setup_with_choice(
            &SetupSimRequest {
                project: &proj,
                setup_idx: 0,
                cad_doc: &cad_doc,
                cad_revision: 1,
                cache: &cache,
                kinematics: kin,
                resolution: SimResolution::Standard,
                cancel: None,
            },
            SimEngineKind::TriDexel,
            SimComputeBackendChoice::Auto,
        )
        .unwrap();

        let rapid_crashes: Vec<_> = sim
            .diagnostics
            .iter()
            .filter(|d| d.kind == SimIssueKind::RapidIntoStock)
            .collect();
        let gouges: Vec<_> = sim
            .diagnostics
            .iter()
            .filter(|d| d.kind == SimIssueKind::TargetModelGouge)
            .collect();
        assert!(
            rapid_crashes.is_empty(),
            "{file_name}: expected 0 RapidIntoStock crashes, got {}: {rapid_crashes:#?}",
            rapid_crashes.len()
        );
        assert!(
            gouges.is_empty(),
            "{file_name}: expected 0 TargetModelGouge diagnostics, got {}: {gouges:#?}",
            gouges.len()
        );
    }
}

#[test]
#[ignore = "full unmodified 4-angle torture benchmark; run with --release --ignored"]
fn torture_triple_helix_full_4angle_indexed_carryover() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let torture_dir = manifest_dir.join("../../torture");
    let proj_path = torture_dir.join("TripleHelix-Indexed.chip.toml");
    if !proj_path.exists() {
        return;
    }
    let toml_str = std::fs::read_to_string(&proj_path).unwrap();
    let mut proj = ProjectDocument::from_toml_str(&toml_str).unwrap();
    let model_entry = &proj.models[0];
    let model_path = torture_dir.join(&model_entry.path);
    let cad_doc = load_model_entry_cad_doc_transformed(
        &model_path,
        model_entry.unit_override,
        &model_entry.transform,
    )
    .unwrap();

    for op in &mut proj.setups[0].operations {
        if op.kind.is_rotary_native() {
            op.suppressed = true;
        }
    }

    let mut cache = ToolpathCache::new(32);
    let rep =
        generate_project_blocking_in_dir(&mut proj, &cad_doc, Some(&torture_dir), &mut cache, 1);
    assert!(
        rep.is_ok(),
        "TripleHelix generation errors: {:?}",
        rep.errors
    );
    assert_eq!(rep.generated.len(), 4);

    let kin = resolve_kinematics_for_key(&proj.setups[0].post_profile, Some(&torture_dir));
    let sim = simulate_project_setup_with_choice(
        &SetupSimRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            kinematics: kin,
            resolution: SimResolution::Standard,
            cancel: None,
        },
        SimEngineKind::TriDexel,
        SimComputeBackendChoice::Auto,
    )
    .unwrap();

    assert!(
        !sim.diagnostics
            .iter()
            .any(|d| d.kind == SimIssueKind::RapidIntoStock),
        "TripleHelix full 4-angle must have 0 RapidIntoStock crashes"
    );
    assert!(
        !sim.diagnostics
            .iter()
            .any(|d| d.kind == SimIssueKind::TargetModelGouge),
        "TripleHelix full 4-angle must have 0 TargetModelGouge diagnostics"
    );
}
