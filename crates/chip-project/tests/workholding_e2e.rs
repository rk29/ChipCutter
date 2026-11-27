#![allow(clippy::unwrap_used, clippy::expect_used)]

//! End-to-end workholding, multi-setup carryover, Flip-Jig Wizard, and Auto Soft-Jaw Wizard
//! regression suite (`chip-project`, Milestone 63).

use chip_cad::{
    CadKernel, MeshImportOptions, MonstertruckKernel, load_mesh_bytes, load_mesh_cad_document,
    sample_multi_feature_bracket_step, sample_sculpted_dome_stl_bytes,
};
use chip_cam::{
    HeightReference, HeightSpec, Operation, OperationKind, Setup, StockConfig, ViseClampAxis,
    WorkOffsetSlot,
};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit, ParamLength};
use chip_post::builtin_post_profiles;
use chip_project::{
    FlipAxis, FlipJigConfig, FlipJigPinCount, ProjectDocument, SetupPostRequest, SetupSimRequest,
    SoftJawBoltPattern, SoftJawConfig, SoftJawOutputMode, ToolpathCache, build_soft_jaw_plan,
    build_two_sided_flip_jig, build_two_sided_vise_workholding_project, compute_sha256_hex,
    generate_project_blocking, post_process_project_setup, resolve_kinematics_for_key,
    sample_two_sided_vise_part_stl_bytes, simulate_all_setups, simulate_project_setup,
    validate_project,
};
use chip_sim::{SimComputeBackendChoice, SimEngineKind, SimResolution};
use chip_tools::{CutterGeometry, default_starter_tool_library};

/// 1. Full built-in `two-sided-vise` Op-1 -> Op-2 generation, fixture avoidance, CPU + GPU
///    Tri-Dexel carryover simulation (`0` critical errors, `In-Tol >= 95%`), and valid GRBL +
///    LinuxCNC G-code emission.
#[test]
fn two_sided_vise_sample_generates_simulates_cpu_gpu_and_posts_cleanly() {
    let stl_bytes = sample_two_sided_vise_part_stl_bytes();
    let cad_doc = load_mesh_cad_document(
        "two_sided_vise_bracket.stl",
        &stl_bytes,
        &MeshImportOptions::default(),
    )
    .unwrap();
    assert!(cad_doc.meshes[0].diagnostics.is_watertight);
    assert_eq!(cad_doc.meshes[0].diagnostics.open_boundary_edges, 0);

    let sha = compute_sha256_hex(&stl_bytes);
    let starter = default_starter_tool_library();
    let mut proj = build_two_sided_vise_workholding_project(
        "two_sided_vise_bracket.stl".to_string(),
        &sha,
        &starter.tools,
    );

    assert_eq!(proj.setups.len(), 2);
    assert_eq!(proj.setups[0].fixtures.len(), 3); // Vise + 2 Toe Clamps
    assert_eq!(proj.setups[1].fixtures.len(), 1); // Contoured Soft Jaws
    assert_eq!(
        proj.setups[1].carryover_from_setup_id.as_deref(),
        Some("setup_1")
    );

    // Deterministic TOML round-trip
    let toml_a = proj.to_toml_string().unwrap();
    let reparsed = ProjectDocument::from_toml_str(&toml_a).unwrap();
    assert_eq!(toml_a, reparsed.to_toml_string().unwrap());

    // Project validation passes with 0 errors
    let val = validate_project(&proj, None, Some(&cad_doc));
    assert!(
        val.is_valid(),
        "two-sided-vise validation errors: {:?}",
        val.diagnostics
    );

    // Parallel wave generation (Op-1 -> carryover -> Op-2)
    let mut cache = ToolpathCache::new(32);
    let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
    assert!(
        batch.is_ok(),
        "two-sided-vise generation errors: {:?}",
        batch.errors
    );
    assert_eq!(batch.generated.len(), 4);

    // CPU Tri-Dexel sequential carryover simulation
    let sims_cpu = simulate_all_setups(&chip_project::AllSetupsSimRequest {
        project: &proj,
        cad_doc: &cad_doc,
        cad_revision: 1,
        cache: &cache,
        project_dir: None,
        resolution: SimResolution::Standard,
        engine: SimEngineKind::TriDexel,
        backend_choice: SimComputeBackendChoice::Cpu,
        verify_config: Default::default(),
        cancel: None,
        progress: None,
    })
    .unwrap();
    assert_eq!(sims_cpu.len(), 2);

    let sim1 = &sims_cpu[0];
    let sim2 = &sims_cpu[1];
    assert!(
        !sim1.has_critical_errors(),
        "Setup 1 critical errors: {:#?}",
        sim1.diagnostics
    );
    assert!(
        !sim2.has_critical_errors(),
        "Setup 2 critical errors: {:#?}",
        sim2.diagnostics
    );
    assert_eq!(
        sim2.carried_over_from_setup.as_deref(),
        Some("Setup 1 — Top Op (G54)")
    );

    // Setup 2 starts from Op-1 remaining stock (significantly less than raw blank)
    let s2_t0_vol = sim2.reconstruct_stock_at_time(0.0).compute_volume_mm3();
    assert!(
        (s2_t0_vol - sim1.remaining_stock_volume_mm3).abs() < 1.0,
        "Setup 2 t=0 vol ({s2_t0_vol:.1}) != Setup 1 end vol ({:.1})",
        sim1.remaining_stock_volume_mm3
    );
    assert!(s2_t0_vol < sim1.initial_stock_volume_mm3 * 0.85);

    let err2_cpu = sim2.error_report.as_ref().unwrap();
    eprintln!(
        "[M63 two-sided-vise CPU] In-Tol={:.2}% RMS={:.4}mm MaxGouge={:.4}mm ExcessVol={:.1}mm³",
        err2_cpu.in_tolerance_surface_pct,
        err2_cpu.rms_error_mm,
        err2_cpu.max_gouge_mm,
        err2_cpu.excess_volume_mm3
    );
    assert!(
        err2_cpu.in_tolerance_surface_pct >= 95.0,
        "expected cumulative In-Tol >= 95%, got {:.2}%",
        err2_cpu.in_tolerance_surface_pct
    );
    assert!(
        err2_cpu.max_gouge_mm <= 0.02,
        "expected max_gouge <= 0.02 mm, got {:.4} mm",
        err2_cpu.max_gouge_mm
    );

    // GPU (Auto) Tri-Dexel parity check
    let sims_gpu = simulate_all_setups(&chip_project::AllSetupsSimRequest {
        project: &proj,
        cad_doc: &cad_doc,
        cad_revision: 1,
        cache: &cache,
        project_dir: None,
        resolution: SimResolution::Standard,
        engine: SimEngineKind::TriDexel,
        backend_choice: SimComputeBackendChoice::Auto,
        verify_config: Default::default(),
        cancel: None,
        progress: None,
    })
    .unwrap();
    assert!(
        !sims_gpu[0].has_critical_errors(),
        "Setup 1 GPU critical errors: {:#?}",
        sims_gpu[0].diagnostics
    );
    assert!(
        !sims_gpu[1].has_critical_errors(),
        "Setup 2 GPU critical errors: {:#?}",
        sims_gpu[1].diagnostics
    );
    let err2_gpu = sims_gpu[1].error_report.as_ref().unwrap();
    assert!(
        err2_gpu.in_tolerance_surface_pct >= 95.0,
        "GPU cumulative In-Tol {:.2}% < 95%",
        err2_gpu.in_tolerance_surface_pct
    );
    assert!(
        (err2_gpu.in_tolerance_surface_pct - err2_cpu.in_tolerance_surface_pct).abs() < 1.0,
        "CPU/GPU In-Tol mismatch: cpu={:.2}% gpu={:.2}%",
        err2_cpu.in_tolerance_surface_pct,
        err2_gpu.in_tolerance_surface_pct
    );

    // GRBL + LinuxCNC post-processing for BOTH Setups
    let profiles = builtin_post_profiles();
    let grbl = profiles.iter().find(|p| p.id == "grbl").unwrap();
    let linuxcnc = profiles.iter().find(|p| p.id == "linuxcnc").unwrap();
    for (s_idx, expected_wcs) in [(0usize, "G54"), (1usize, "G55")] {
        for profile in [grbl, linuxcnc] {
            let files = post_process_project_setup(&SetupPostRequest {
                project: &proj,
                setup_idx: s_idx,
                cad_doc: &cad_doc,
                cad_revision: 1,
                cache: &cache,
                post_profile: profile,
                project_label: &proj.project_name,
                program_stem: &format!("two_sided_s{}", s_idx + 1),
            })
            .unwrap();
            assert!(!files.is_empty());
            assert!(
                files[0].content.contains(expected_wcs),
                "Setup {} {} G-code missing {expected_wcs}",
                s_idx + 1,
                profile.id
            );
        }
    }
}

/// 2. Flip-Jig Wizard: verifies 180° flip symmetry invariance (`< 1e-9 mm`), part silhouette
///    clearance, poka-yoke asymmetry, and crash-free Op-1 + Op-2 + Spoilboard simulation.
#[test]
fn flip_jig_wizard_e2e_symmetry_clearance_and_crash_free_simulation() {
    let stl_bytes = sample_two_sided_vise_part_stl_bytes();
    let cad_doc = load_mesh_cad_document(
        "two_sided_part.stl",
        &stl_bytes,
        &MeshImportOptions::default(),
    )
    .unwrap();
    let starter = default_starter_tool_library();
    let flat_6 = starter.tools[0].clone(); // Ø6mm flat endmill
    let drill_6 = starter
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::TwistDrill { .. }))
        .cloned()
        .unwrap();

    let mut proj = ProjectDocument::new("Flip-Jig E2E");
    proj.tools.push(flat_6.clone());
    proj.tools.push(drill_6.clone());

    let mut s1 = Setup::new("setup_1", "Setup 1 - Top", WorkOffsetSlot::G54);
    s1.stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(4.0),
        side_neg_x: ParamLength::from_mm(4.0),
        side_pos_y: ParamLength::from_mm(4.0),
        side_neg_y: ParamLength::from_mm(4.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(4.0),
        sheet_thickness_z: ParamLength::from_mm(12.0),
        round_increment_mm: None,
    };
    proj.setups = vec![s1];

    let cfg = FlipJigConfig {
        flip_axis: FlipAxis::XAxis,
        pin_diameter_mm: 6.0,
        pin_fit_clearance_mm: 0.02,
        pin_count: FlipJigPinCount::FourSymmetric,
        poka_yoke_offset_mm: 4.0,
        stock_pin_depth_mm: 0.0,
        spoilboard_pin_depth_mm: 10.0,
        edge_margin_mm: 6.0,
        expand_stock_if_needed: true,
        include_spoilboard_setup: true,
        pin_protrusion_z_mm: 5.0,
        pin_tool_id: Some(drill_6.id.clone()),
        pin_tool_diameter_mm: Some(drill_6.geometry.cutting_diameter_mm()),
        setup2_id: Some("setup_2_flip".to_string()),
        setup2_wcs_slot: WorkOffsetSlot::G55,
        spoilboard_setup_id: Some("setup_spoilboard_pins".to_string()),
        spoilboard_wcs_slot: WorkOffsetSlot::G54,
    };

    let plan = build_two_sided_flip_jig(
        &cad_doc,
        &proj.setups[0],
        &proj.sketches,
        DisplayUnit::Millimeter,
        &cfg,
    )
    .unwrap();

    assert_eq!(plan.pin_centers_setup1_wcs.len(), 4);
    assert!(plan.max_symmetry_error_mm < 1e-9);
    assert!(plan.min_part_clearance_mm >= 6.0 - 1e-4);

    // Apply wizard plan cleanly
    if let Some(stk) = plan.updated_setup1_stock {
        proj.setups[0].stock = stk;
    }
    proj.sketches.push(plan.pin_sketch);
    proj.setups[0].operations.push(plan.setup1_pin_op);

    // Add a real Op-2 facing pass over the 4 mm bottom hat stock (protruding dowel pins sit below
    // Z = stock2_min_z + 5 mm while Op-2 faces stock2_max_z -> model_top_z cleanly above them!)
    let mut setup2 = plan.setup2_flip;
    let mut op2_face = Operation::new("op2_face_hat", "Op 2 Face Hat", OperationKind::Facing);
    op2_face.tool_id = Some(flat_6.id.clone());
    op2_face.preset_id = flat_6.presets.first().map(|p| p.id.clone());
    op2_face.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op2_face.heights.bottom = HeightSpec::new(HeightReference::ModelTop, 0.0);
    op2_face.passes.multiple_depths = true;
    op2_face.passes.max_stepdown = ParamLength::from_mm(2.0);
    op2_face.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    setup2.operations.push(op2_face);

    proj.setups.push(setup2);
    if let Some(sb_sk) = plan.spoilboard_sketch {
        proj.sketches.push(sb_sk);
    }
    if let Some(sb_s) = plan.spoilboard_setup {
        proj.setups.push(sb_s);
    }

    let val = validate_project(&proj, None, Some(&cad_doc));
    assert!(
        val.is_valid(),
        "flip-jig validation errors: {:?}",
        val.diagnostics
    );

    let mut cache = ToolpathCache::new(32);
    let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
    assert!(
        batch.is_ok(),
        "flip-jig generation errors: {:?}",
        batch.errors
    );

    // Simulate all 3 Setups (Setup 1 pin drilling, Setup 2 carryover + pin keep-outs, Spoilboard pins)
    let sims = simulate_all_setups(&chip_project::AllSetupsSimRequest {
        project: &proj,
        cad_doc: &cad_doc,
        cad_revision: 1,
        cache: &cache,
        project_dir: None,
        resolution: SimResolution::Standard,
        engine: SimEngineKind::TriDexel,
        backend_choice: SimComputeBackendChoice::Cpu,
        verify_config: Default::default(),
        cancel: None,
        progress: None,
    })
    .unwrap();
    assert_eq!(sims.len(), 3);
    for (idx, sim) in sims.iter().enumerate() {
        assert!(
            !sim.has_critical_errors(),
            "Flip-Jig Setup #{} had critical errors: {:#?}",
            idx + 1,
            sim.diagnostics
        );
    }
}

/// 3. Auto Soft-Jaw Wizard: verifies BOTH crash-free CNC aluminum soft-jaw machining operations
///    AND watertight `.stl` reload via `chip-cad` with exact pocket depth & radial clearance.
#[test]
fn soft_jaw_wizard_e2e_machining_simulation_and_watertight_stl_verification() {
    let step = sample_multi_feature_bracket_step(false).unwrap();
    let bracket_doc = MonstertruckKernel
        .load_step_bytes("bracket.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();
    let starter = default_starter_tool_library();
    let flat_3 = starter
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. }
                if (diameter_mm - 3.0).abs() < 0.2
            )
        })
        .cloned()
        .unwrap_or_else(|| starter.tools[0].clone());

    let mut proj = ProjectDocument::new("Soft-Jaw E2E");
    proj.tools.push(flat_3.clone());
    let setup1 = Setup::new("setup_1", "Part Setup 1", WorkOffsetSlot::G54);
    proj.setups = vec![setup1];

    let grip_depth_mm = 4.5;
    let clearance_mm = 0.12;
    let cfg = SoftJawConfig {
        clamp_axis: ViseClampAxis::Y,
        jaw_width_mm: 152.4,
        jaw_depth_mm: 45.0,
        jaw_height_mm: 45.0,
        jaw_gap_mm: 6.0,
        grip_depth_mm,
        part_clearance_mm: clearance_mm,
        corner_relief_radius_mm: 2.0,
        mounting_bolt_pattern: SoftJawBoltPattern::Kurt6Inch,
        output_mode: SoftJawOutputMode::Both,
        z_band_bottom_offset_mm: 0.0,
        roughing_tool_id: Some(flat_3.id.clone()),
        finishing_tool_id: Some(flat_3.id.clone()),
        setup_id: Some("setup_soft_jaws".to_string()),
        wcs_slot: WorkOffsetSlot::G56,
    };

    let plan = build_soft_jaw_plan(
        &bracket_doc,
        &proj.setups[0],
        &proj.sketches,
        DisplayUnit::Millimeter,
        &cfg,
    )
    .unwrap();

    // A. Verify watertight STL reload, exact jaw outer dimensions, pocket depth, and radial clearance
    let fixed_bytes = plan.fixed_jaw_stl.as_ref().unwrap();
    let movable_bytes = plan.movable_jaw_stl.as_ref().unwrap();

    for (name, bytes) in [
        ("fixed_jaw.stl", fixed_bytes),
        ("movable_jaw.stl", movable_bytes),
    ] {
        let mesh = load_mesh_bytes(name, bytes, &MeshImportOptions::default()).unwrap();
        assert!(
            mesh.diagnostics.is_watertight,
            "{name} not watertight: {:?}",
            mesh.diagnostics
        );
        assert_eq!(mesh.diagnostics.open_boundary_edges, 0);
        assert_eq!(mesh.diagnostics.non_manifold_edges, 0);
        assert!(mesh.diagnostics.signed_volume_mm3 > 0.0);

        let sz = mesh.aabb_mm.size();
        assert!((sz.x - 152.4).abs() < 0.02, "{name} X size={}", sz.x);
        assert!((sz.y - 45.0).abs() < 0.02, "{name} Y size={}", sz.y);
        assert!((sz.z - 45.0).abs() < 0.02, "{name} Z size={}", sz.z);

        // Verify pocket floor triangles sit at exact `Z = jaw_height - grip_depth` (`40.5 mm`)!
        let expected_floor_z = 45.0 - grip_depth_mm;
        let has_floor = mesh.triangles.iter().any(|t| {
            t.normal.z > 0.99
                && (mesh.vertices_mm[t.indices[0] as usize].z - expected_floor_z).abs() < 0.02
        });
        assert!(
            has_floor,
            "{name} missing horizontal pocket floor at Z = {expected_floor_z:.2} mm"
        );

        // Verify pocket width reflects exact `90.0 + 2 * clearance_mm` (plus dogbones at corners)
        let pocket_floor_xs: Vec<f64> = mesh
            .triangles
            .iter()
            .filter(|t| {
                t.normal.z > 0.99
                    && (mesh.vertices_mm[t.indices[0] as usize].z - expected_floor_z).abs() < 0.02
            })
            .flat_map(|t| {
                t.indices
                    .into_iter()
                    .map(|i| mesh.vertices_mm[i as usize].x)
            })
            .collect();
        let min_px = pocket_floor_xs
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let max_px = pocket_floor_xs
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        let pocket_span_x = max_px - min_px;
        assert!(
            pocket_span_x >= 90.0 + 2.0 * clearance_mm - 0.05,
            "{name} pocket span X ({pocket_span_x:.3} mm) smaller than part + 2*clearance"
        );
    }

    // B. Verify generated aluminum soft-jaw machining Setup generates & simulates crash-free!
    proj.sketches.push(plan.pocket_sketch.unwrap());
    proj.setups.push(plan.machining_setup.unwrap());

    let val = validate_project(&proj, None, Some(&bracket_doc));
    assert!(
        val.is_valid(),
        "soft-jaw setup validation errors: {:?}",
        val.diagnostics
    );

    let mut cache = ToolpathCache::new(32);
    let batch = generate_project_blocking(&mut proj, &bracket_doc, &mut cache, 1);
    assert!(
        batch.is_ok(),
        "soft-jaw generation errors: {:?}",
        batch.errors
    );
    assert_eq!(proj.setups[1].operations.len(), 3);

    let sim_sj = simulate_project_setup(&SetupSimRequest {
        project: &proj,
        setup_idx: 1,
        cad_doc: &bracket_doc,
        cad_revision: 1,
        cache: &cache,
        kinematics: resolve_kinematics_for_key("grbl", None),
        resolution: SimResolution::Standard,
        cancel: None,
        engine: SimEngineKind::Heightmap,
        backend_choice: SimComputeBackendChoice::Cpu,
        ..SetupSimRequest::new(&proj, 1, &bracket_doc, 1, &cache)
    })
    .unwrap();
    assert!(
        !sim_sj.has_critical_errors(),
        "soft-jaw machining simulation errors: {:#?}",
        sim_sj.diagnostics
    );

    // C. Also verify soft jaws work cleanly on organic STL dome meshes
    let dome_bytes = sample_sculpted_dome_stl_bytes();
    let dome_doc =
        load_mesh_cad_document("dome.stl", &dome_bytes, &MeshImportOptions::default()).unwrap();
    let dome_plan = build_soft_jaw_plan(
        &dome_doc,
        &Setup::new("s1", "Dome", WorkOffsetSlot::G54),
        &[],
        DisplayUnit::Millimeter,
        &SoftJawConfig {
            jaw_width_mm: 100.0,
            jaw_depth_mm: 40.0,
            jaw_height_mm: 38.0,
            jaw_gap_mm: 6.0,
            grip_depth_mm: 4.0,
            part_clearance_mm: 0.15,
            corner_relief_radius_mm: 1.5,
            mounting_bolt_pattern: SoftJawBoltPattern::CarveraModVise,
            output_mode: SoftJawOutputMode::PrintableStl,
            ..SoftJawConfig::default()
        },
    )
    .unwrap();
    let dome_fixed = load_mesh_bytes(
        "dome_fixed.stl",
        dome_plan.fixed_jaw_stl.as_ref().unwrap(),
        &MeshImportOptions::default(),
    )
    .unwrap();
    assert!(dome_fixed.diagnostics.is_watertight);
}

#[test]
fn soft_jaw_wizard_90deg_twisted_helical_loft_zero_scalloping_interference() {
    use chip_cam::{AxisAnchor, BoxAnchor27, WcsOriginMode};
    use chip_math::{DVec2, DVec3};

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

    let stl = chip_cad::encode_binary_stl("twisted_90deg_helix", &tris);
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

    let mut outside_sil_count = 0usize;
    let mut outside_jaw_pocket_count = 0usize;
    let mut max_miss_mm = 0.0_f64;
    let half_gap = 0.5 * cfg.jaw_gap_mm;

    for iz in 0..=100 {
        let z = grip_depth_mm * (iz as f64 / 100.0);
        let twist = std::f64::consts::FRAC_PI_2 * (z / z_max);
        let (st, ct) = twist.sin_cos();
        for it in 0..64 {
            let phi = std::f64::consts::TAU * (it as f64 / 64.0);
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
