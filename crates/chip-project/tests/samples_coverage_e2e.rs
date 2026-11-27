#![allow(clippy::unwrap_used, clippy::expect_used)]

//! All-samples crash-free E2E regression suite for Milestone 68's 6 gap-closing built-in sample
//! projects (`feed-opt-pocket`, `flip-jig-softjaws`, `curved-deburr-3d`, `dovetail-rail-keyway`,
//! `svg-badge`, `mesh-3mf-bracket`) plus byte-for-byte verification of checked-in minimal tutorial
//! starter assets under `docs/tutorials/assets/`.

use std::path::PathBuf;

use chip_cad::{
    CadDocument, CadKernel, MeshFormat, MeshImportOptions, MeshUnitKind, MonstertruckKernel,
    load_mesh_cad_document, sample_curved_deburr_overhang_step_bytes,
    sample_dovetail_rail_keyway_step_bytes, sample_indexed_4sided_bar_step_bytes,
    sample_keepout_clamp_plate_step_bytes, sample_overhang_flange_step,
    sample_sculpted_mount_3mf_bytes, sample_sloped_boss_stepped_pocket_step,
    sample_svg_badge_bytes, sample_undercut_deburr_block_step, tutorial_adaptive_pocket_step,
    tutorial_contour_plate_step,
};
use chip_cam::{
    DovetailMode, FixedBoxZAlign, FixtureKind, MachiningBoundaryMode, OperationKind,
    OperationRotaryMode, SlotUndercutMode, StockConfig, StrategyParams, UndercutEntryMode,
};
use chip_math::DEFAULT_TESS_CHORD_TOL_MM;
use chip_post::builtin_post_profiles;
use chip_project::{
    ProjectDocument, SetupPostRequest, SetupSimRequest, ToolpathCache,
    build_curved_deburr_3d_project, build_dovetail_rail_keyway_project,
    build_feed_opt_pocket_project, build_flip_jig_softjaws_project,
    build_indexed_model_bounds_project, build_keepout_clamps_project,
    build_mesh_3mf_bracket_project, build_svg_badge_sign_project, compute_sha256_hex,
    generate_project_blocking, post_process_project_setup, resolve_kinematics_for_key,
    simulate_all_setups, simulate_project_setup, validate_project,
};
use chip_sim::{SimComputeBackendChoice, SimEngineKind, SimIssueKind, SimResolution};
use chip_tools::default_starter_tool_library;

fn assert_posts_cleanly(
    proj: &ProjectDocument,
    cad_doc: &CadDocument,
    cache: &ToolpathCache,
    setup_idx: usize,
    expected_wcs: &str,
    stem: &str,
) -> (String, String) {
    let profiles = builtin_post_profiles();
    let grbl = profiles.iter().find(|p| p.id == "grbl").unwrap();
    let linuxcnc = profiles.iter().find(|p| p.id == "linuxcnc").unwrap();

    let mut outputs = Vec::new();
    for profile in [grbl, linuxcnc] {
        let files = post_process_project_setup(&SetupPostRequest {
            project: proj,
            setup_idx,
            cad_doc,
            cad_revision: 1,
            cache,
            post_profile: profile,
            project_label: &proj.project_name,
            program_stem: stem,
        })
        .unwrap_or_else(|e| panic!("{stem} setup {setup_idx} {} post failed: {e}", profile.id));
        assert!(
            !files.is_empty(),
            "{stem} setup {setup_idx} {} emitted no G-code files",
            profile.id
        );
        let combined: String = files.iter().map(|f| f.content.as_str()).collect();
        assert!(
            combined.contains(expected_wcs),
            "{stem} setup {setup_idx} {} missing {expected_wcs}",
            profile.id
        );
        assert!(
            combined.contains("G0"),
            "{stem} setup {setup_idx} {} missing G0 rapid",
            profile.id
        );
        assert!(
            combined.contains("G1"),
            "{stem} setup {setup_idx} {} missing G1 feed",
            profile.id
        );
        assert!(
            !combined.contains("NaN") && !combined.contains("inf") && !combined.contains("INF"),
            "{stem} setup {setup_idx} {} contains non-finite numeric output",
            profile.id
        );
        outputs.push(combined);
    }
    (outputs.remove(0), outputs.remove(0))
}

fn assert_toml_roundtrip_deterministic(proj: &ProjectDocument) {
    let toml_a = proj.to_toml_string().unwrap();
    let reparsed = ProjectDocument::from_toml_str(&toml_a).unwrap();
    assert_eq!(toml_a, reparsed.to_toml_string().unwrap());
}

/// `feed-opt-pocket`: Engagement-aware `FeedOptimizationConfig` produces `min_feed < max_feed`,
/// `0` critical errors, gouge-free stock removal, and clean GRBL + LinuxCNC G-code.
#[test]
fn sample_feed_opt_pocket_e2e_modulated_feed_and_zero_errors() {
    let step = tutorial_adaptive_pocket_step().unwrap();
    let cad_doc = MonstertruckKernel
        .load_step_bytes(
            "20-feed-opt-pocket.step",
            step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    let sha = compute_sha256_hex(step.as_bytes());
    let starter = default_starter_tool_library();

    let mut proj = build_feed_opt_pocket_project(
        &cad_doc,
        "20-feed-opt-pocket.step".to_string(),
        &sha,
        &starter.tools,
    );
    assert_eq!(proj.setups.len(), 1);
    assert_eq!(proj.setups[0].operations.len(), 2);
    assert!(
        proj.setups[0]
            .operations
            .iter()
            .all(|op| op.passes.feed_optimization.enabled)
    );
    assert_toml_roundtrip_deterministic(&proj);

    let val = validate_project(&proj, None, Some(&cad_doc));
    assert!(
        val.is_valid(),
        "feed-opt-pocket validation: {:?}",
        val.diagnostics
    );

    let mut cache = ToolpathCache::new(32);
    let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
    assert!(
        batch.is_ok(),
        "feed-opt-pocket generation: {:?}",
        batch.errors
    );

    let kin = resolve_kinematics_for_key(&proj.setups[0].post_profile, None);
    let sim = simulate_project_setup(&SetupSimRequest {
        project: &proj,
        setup_idx: 0,
        cad_doc: &cad_doc,
        cad_revision: 1,
        cache: &cache,
        kinematics: kin,
        resolution: SimResolution::Standard,
        cancel: None,
        engine: SimEngineKind::Heightmap,
        backend_choice: SimComputeBackendChoice::Cpu,
        ..SetupSimRequest::new(&proj, 0, &cad_doc, 1, &cache)
    })
    .unwrap();

    assert_eq!(
        sim.critical_error_count(),
        0,
        "feed-opt-pocket critical errors: {:#?}",
        sim.diagnostics
    );
    assert!(
        sim.timeline.feed_modulated_segments > 0,
        "expected feed-modulated segments on feed-opt-pocket"
    );
    assert!(
        sim.timeline.min_cutting_feed_mm_min + 1.0 < sim.timeline.max_cutting_feed_mm_min,
        "expected min_feed < max_feed: min={:.1}, max={:.1}",
        sim.timeline.min_cutting_feed_mm_min,
        sim.timeline.max_cutting_feed_mm_min
    );

    let err = sim.error_report.as_ref().unwrap();
    assert!(
        err.max_gouge_mm <= 0.02,
        "feed-opt-pocket max_gouge_mm {:.4} > 0.02",
        err.max_gouge_mm
    );

    assert_posts_cleanly(&proj, &cad_doc, &cache, 0, "G54", "feed_opt_pocket");
}

/// `flip-jig-softjaws`: Spoilboard + Flip-Jig `TwoOnAxis` registration pins (`CylinderKeepOut`
/// + poka-yoke) + Setup 2 `180°` flip carryover + Setup 3 generated Auto Soft-Jaw machining setup.
#[test]
fn sample_flip_jig_softjaws_e2e_carryover_pins_and_soft_jaws() {
    let step = tutorial_contour_plate_step().unwrap();
    let cad_doc = MonstertruckKernel
        .load_step_bytes(
            "02-contour-plate.step",
            step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    let sha = compute_sha256_hex(step.as_bytes());
    let starter = default_starter_tool_library();

    let mut proj = build_flip_jig_softjaws_project(
        &cad_doc,
        "02-contour-plate.step".to_string(),
        &sha,
        &starter.tools,
    );
    assert_eq!(
        proj.setups.len(),
        3,
        "expected 3 setups (Op1 + Flip Op2 + Auto Soft-Jaws)"
    );
    assert!(
        proj.setups[0]
            .fixtures
            .iter()
            .any(|f| matches!(f.kind, FixtureKind::Spoilboard { .. })),
        "Setup 1 must include a Spoilboard fixture"
    );
    assert_eq!(
        proj.setups[1].carryover_from_setup_id.as_deref(),
        Some("setup_1")
    );
    let pin_keepouts = proj.setups[1]
        .fixtures
        .iter()
        .filter(|f| matches!(f.kind, FixtureKind::CylinderKeepOut { .. }))
        .count();
    assert_eq!(
        pin_keepouts, 2,
        "Setup 2 must have 2 TwoOnAxis CylinderKeepOut pin fixtures"
    );
    assert_eq!(
        proj.setups[2].operations.len(),
        3,
        "Setup 3 Auto Soft-Jaw machining setup must have 3 operations"
    );
    assert_toml_roundtrip_deterministic(&proj);

    let val = validate_project(&proj, None, Some(&cad_doc));
    assert!(
        val.is_valid(),
        "flip-jig-softjaws validation: {:?}",
        val.diagnostics
    );

    let mut cache = ToolpathCache::new(64);
    let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
    assert!(
        batch.is_ok(),
        "flip-jig-softjaws generation: {:?}",
        batch.errors
    );

    // Simulate Setups 1 & 2 via sequential Tri-Dexel carryover (and Setup 3 soft jaws)
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
        assert_eq!(
            sim.critical_error_count(),
            0,
            "flip-jig-softjaws Setup #{} had critical errors: {:#?}",
            idx + 1,
            sim.diagnostics
        );
    }
    assert!(
        sims[1].carried_over_from_setup.is_some(),
        "Setup 2 must report carried_over_from_setup"
    );
    let s2_t0_vol = sims[1].reconstruct_stock_at_time(0.0).compute_volume_mm3();
    assert!(
        (s2_t0_vol - sims[0].remaining_stock_volume_mm3).abs() < 1.5,
        "Setup 2 t=0 vol ({s2_t0_vol:.1}) != Setup 1 end vol ({:.1})",
        sims[0].remaining_stock_volume_mm3
    );

    assert_posts_cleanly(&proj, &cad_doc, &cache, 0, "G54", "flip_jig_s1");
    assert_posts_cleanly(&proj, &cad_doc, &cache, 1, "G55", "flip_jig_s2");
    assert_posts_cleanly(&proj, &cad_doc, &cache, 2, "G56", "flip_jig_s3_softjaws");
}

/// `curved-deburr-3d`: Non-planar sloped arch + stepped boss wall trim + underside overhang lip
/// exercising `Deburr3D` across `ChamferMill`, `BallEndmill`, and `LollipopEndmill` with exact
/// CPU/GPU Tri-Dexel parity and `0` critical errors.
#[test]
fn sample_curved_deburr_3d_e2e_three_cutters_and_cpu_gpu_tridexel_parity() {
    let step_bytes = sample_curved_deburr_overhang_step_bytes().unwrap();
    let cad_doc = MonstertruckKernel
        .load_step_bytes(
            "sample_curved_deburr_overhang.step",
            &step_bytes,
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    let sha = compute_sha256_hex(&step_bytes);
    let starter = default_starter_tool_library();

    let mut proj = build_curved_deburr_3d_project(
        &cad_doc,
        "sample_curved_deburr_overhang.step".to_string(),
        &sha,
        &starter.tools,
    );
    assert_eq!(proj.setups.len(), 1);
    assert_eq!(proj.setups[0].operations.len(), 6);
    let deburr_count = proj.setups[0]
        .operations
        .iter()
        .filter(|op| op.kind == OperationKind::Deburr3D)
        .count();
    assert_eq!(
        deburr_count, 3,
        "curved-deburr-3d must contain 3 Deburr3D ops (ChamferMill, BallEndmill, Lollipop)"
    );
    assert_toml_roundtrip_deterministic(&proj);

    let val = validate_project(&proj, None, Some(&cad_doc));
    assert!(
        val.is_valid(),
        "curved-deburr-3d validation: {:?}",
        val.diagnostics
    );

    let mut cache = ToolpathCache::new(64);
    let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
    assert!(
        batch.is_ok(),
        "curved-deburr-3d generation: {:?}",
        batch.errors
    );
    assert_eq!(batch.generated.len(), 6);

    let kin = resolve_kinematics_for_key(&proj.setups[0].post_profile, None);
    let sim_cpu = simulate_project_setup(&SetupSimRequest {
        project: &proj,
        setup_idx: 0,
        cad_doc: &cad_doc,
        cad_revision: 1,
        cache: &cache,
        kinematics: kin,
        resolution: SimResolution::Standard,
        cancel: None,
        engine: SimEngineKind::TriDexel,
        backend_choice: SimComputeBackendChoice::Cpu,
        ..SetupSimRequest::new(&proj, 0, &cad_doc, 1, &cache)
    })
    .unwrap();
    let sim_gpu = simulate_project_setup(&SetupSimRequest {
        project: &proj,
        setup_idx: 0,
        cad_doc: &cad_doc,
        cad_revision: 1,
        cache: &cache,
        kinematics: kin,
        resolution: SimResolution::Standard,
        cancel: None,
        engine: SimEngineKind::TriDexel,
        backend_choice: SimComputeBackendChoice::Auto,
        ..SetupSimRequest::new(&proj, 0, &cad_doc, 1, &cache)
    })
    .unwrap();

    for (label, sim) in [("CPU", &sim_cpu), ("Auto/GPU", &sim_gpu)] {
        assert_eq!(
            sim.critical_error_count(),
            0,
            "curved-deburr-3d ({label}) critical errors: {:#?}",
            sim.diagnostics
        );
        let err = sim.error_report.as_ref().unwrap();
        assert!(
            err.in_tolerance_surface_pct >= 90.0,
            "curved-deburr-3d ({label}) In-Tol {:.2}% < 90%",
            err.in_tolerance_surface_pct
        );
    }

    let rel_vol = (sim_gpu.removed_volume_mm3 - sim_cpu.removed_volume_mm3).abs()
        / sim_cpu.removed_volume_mm3.max(1.0);
    assert!(
        rel_vol < 0.005,
        "curved-deburr-3d CPU/GPU removed volume parity mismatch: cpu={:.2}, gpu={:.2}",
        sim_cpu.removed_volume_mm3,
        sim_gpu.removed_volume_mm3
    );

    assert_posts_cleanly(&proj, &cad_doc, &cache, 0, "G54", "curved_deburr_3d");
}

/// `dovetail-rail-keyway`: Facing -> Ø8 mm Entry Bore -> Blind Keyway (`EntryBoreCenter`) ->
/// External `60°` Dovetail Rail (`DovetailMode::ExternalRail`) with CPU/GPU Tri-Dexel parity
/// and `0` critical errors.
#[test]
fn sample_dovetail_rail_keyway_e2e_external_rail_entry_bore_and_cpu_gpu_parity() {
    let step_bytes = sample_dovetail_rail_keyway_step_bytes().unwrap();
    let cad_doc = MonstertruckKernel
        .load_step_bytes(
            "sample_dovetail_rail_keyway.step",
            &step_bytes,
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    let sha = compute_sha256_hex(&step_bytes);
    let starter = default_starter_tool_library();

    let mut proj = build_dovetail_rail_keyway_project(
        &cad_doc,
        "sample_dovetail_rail_keyway.step".to_string(),
        &sha,
        &starter.tools,
    );
    assert_eq!(proj.setups.len(), 1);
    assert_eq!(proj.setups[0].operations.len(), 4);
    assert!(matches!(
        proj.setups[0].operations[2].strategy,
        Some(StrategyParams::SlotUndercut(ref p))
            if p.mode == SlotUndercutMode::Keyway
                && p.entry_mode == UndercutEntryMode::EntryBoreCenter
    ));
    assert!(matches!(
        proj.setups[0].operations[3].strategy,
        Some(StrategyParams::Dovetail(ref p)) if p.mode == DovetailMode::ExternalRail
    ));
    assert_toml_roundtrip_deterministic(&proj);

    let val = validate_project(&proj, None, Some(&cad_doc));
    assert!(
        val.is_valid(),
        "dovetail-rail-keyway validation: {:?}",
        val.diagnostics
    );

    let mut cache = ToolpathCache::new(64);
    let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
    assert!(
        batch.is_ok(),
        "dovetail-rail-keyway generation: {:?}",
        batch.errors
    );
    assert_eq!(batch.generated.len(), 4);

    let kin = resolve_kinematics_for_key(&proj.setups[0].post_profile, None);
    let sim_cpu = simulate_project_setup(&SetupSimRequest {
        project: &proj,
        setup_idx: 0,
        cad_doc: &cad_doc,
        cad_revision: 1,
        cache: &cache,
        kinematics: kin,
        resolution: SimResolution::Fine,
        cancel: None,
        engine: SimEngineKind::TriDexel,
        backend_choice: SimComputeBackendChoice::Cpu,
        ..SetupSimRequest::new(&proj, 0, &cad_doc, 1, &cache)
    })
    .unwrap();
    let sim_gpu = simulate_project_setup(&SetupSimRequest {
        project: &proj,
        setup_idx: 0,
        cad_doc: &cad_doc,
        cad_revision: 1,
        cache: &cache,
        kinematics: kin,
        resolution: SimResolution::Fine,
        cancel: None,
        engine: SimEngineKind::TriDexel,
        backend_choice: SimComputeBackendChoice::Auto,
        ..SetupSimRequest::new(&proj, 0, &cad_doc, 1, &cache)
    })
    .unwrap();

    for (label, sim) in [("CPU", &sim_cpu), ("Auto/GPU", &sim_gpu)] {
        assert_eq!(
            sim.critical_error_count(),
            0,
            "dovetail-rail-keyway ({label}) critical errors: {:#?}",
            sim.diagnostics
        );
        let err = sim.error_report.as_ref().unwrap();
        assert!(
            err.max_gouge_mm <= 0.02,
            "dovetail-rail-keyway ({label}) max_gouge_mm {:.4} > 0.02 at {:?} (gouge_vol={:.3}mm³)",
            err.max_gouge_mm,
            err.max_gouge_location_wcs,
            err.gouge_volume_mm3
        );
        assert!(
            err.in_tolerance_surface_pct >= 90.0,
            "dovetail-rail-keyway ({label}) In-Tol {:.2}% < 90%",
            err.in_tolerance_surface_pct
        );
    }

    let rel_vol = (sim_gpu.removed_volume_mm3 - sim_cpu.removed_volume_mm3).abs()
        / sim_cpu.removed_volume_mm3.max(1.0);
    assert!(
        rel_vol < 0.005,
        "dovetail-rail-keyway CPU/GPU removed volume parity mismatch: cpu={:.2}, gpu={:.2}",
        sim_cpu.removed_volume_mm3,
        sim_gpu.removed_volume_mm3
    );

    assert_posts_cleanly(&proj, &cad_doc, &cache, 0, "G54", "dovetail_rail_keyway");
}

/// `svg-badge`: Pure-2D 96 DPI cubic Bézier `.svg` import with counter-islands, Pocket +
/// V-Carve + Contour + Tabs, `0` critical errors, and true `G2`/`G3` biarc G-code emission.
#[test]
fn sample_svg_badge_e2e_biarc_g2_g3_emission_and_zero_errors() {
    let starter = default_starter_tool_library();
    let mut proj = build_svg_badge_sign_project(None, &starter.tools);
    assert!(proj.models.is_empty(), "svg-badge must be a Pure-2D job");
    assert_eq!(proj.sketches.len(), 1);
    assert_eq!(proj.setups.len(), 1);
    assert_eq!(proj.setups[0].operations.len(), 3);

    // Verify emblem pocket region carries its nested counter-island
    assert!(
        proj.sketches[0]
            .regions
            .iter()
            .any(|r| r.sub_layer.as_deref() == Some("EMBLEM_POCKET") && r.islands_cw.len() == 1),
        "expected EMBLEM_POCKET region with 1 nested counter-island"
    );
    assert_toml_roundtrip_deterministic(&proj);

    let cad_doc = CadDocument::empty();
    let val = validate_project(&proj, None, Some(&cad_doc));
    assert!(
        val.is_valid(),
        "svg-badge validation: {:?}",
        val.diagnostics
    );

    let mut cache = ToolpathCache::new(32);
    let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
    assert!(batch.is_ok(), "svg-badge generation: {:?}", batch.errors);
    assert_eq!(batch.generated.len(), 3);

    let kin = resolve_kinematics_for_key(&proj.setups[0].post_profile, None);
    let sim = simulate_project_setup(&SetupSimRequest {
        project: &proj,
        setup_idx: 0,
        cad_doc: &cad_doc,
        cad_revision: 1,
        cache: &cache,
        kinematics: kin,
        resolution: SimResolution::Standard,
        cancel: None,
        engine: SimEngineKind::Heightmap,
        backend_choice: SimComputeBackendChoice::Cpu,
        ..SetupSimRequest::new(&proj, 0, &cad_doc, 1, &cache)
    })
    .unwrap();
    assert_eq!(
        sim.critical_error_count(),
        0,
        "svg-badge critical errors: {:#?}",
        sim.diagnostics
    );

    let (grbl_gcode, linuxcnc_gcode) =
        assert_posts_cleanly(&proj, &cad_doc, &cache, 0, "G54", "svg_badge");
    for (dialect, gcode) in [("GRBL", &grbl_gcode), ("LinuxCNC", &linuxcnc_gcode)] {
        assert!(
            gcode.contains("G2 ") || gcode.contains("G3 "),
            "svg-badge {dialect} G-code must emit true circular G2/G3 biarc moves"
        );
    }
}

/// `mesh-3mf-bracket`: Watertight unit-aware `.3mf` mesh import machined with 3D Adaptive
/// Roughing + 3D Constant-Scallop Finishing, `0` critical errors, and clean GRBL/LinuxCNC G-code.
#[test]
fn sample_mesh_3mf_bracket_e2e_watertight_import_and_zero_errors() {
    let mf_bytes = sample_sculpted_mount_3mf_bytes();
    let cad_doc =
        load_mesh_cad_document("08-bracket.3mf", &mf_bytes, &MeshImportOptions::default()).unwrap();
    assert_eq!(cad_doc.meshes.len(), 1);
    assert_eq!(cad_doc.meshes[0].format, MeshFormat::ThreeMf);
    assert_eq!(cad_doc.meshes[0].applied_unit, MeshUnitKind::Millimeter);
    assert!(cad_doc.meshes[0].diagnostics.is_watertight);
    assert_eq!(cad_doc.meshes[0].diagnostics.open_boundary_edges, 0);

    let sha = compute_sha256_hex(&mf_bytes);
    let starter = default_starter_tool_library();
    let mut proj =
        build_mesh_3mf_bracket_project("08-bracket.3mf".to_string(), &sha, &starter.tools);
    assert_eq!(proj.setups.len(), 1);
    assert_eq!(proj.setups[0].operations.len(), 2);
    assert_toml_roundtrip_deterministic(&proj);

    let val = validate_project(&proj, None, Some(&cad_doc));
    assert!(
        val.is_valid(),
        "mesh-3mf-bracket validation: {:?}",
        val.diagnostics
    );

    let mut cache = ToolpathCache::new(32);
    let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
    assert!(
        batch.is_ok(),
        "mesh-3mf-bracket generation: {:?}",
        batch.errors
    );
    assert_eq!(batch.generated.len(), 2);

    let kin = resolve_kinematics_for_key(&proj.setups[0].post_profile, None);
    let sim = simulate_project_setup(&SetupSimRequest {
        project: &proj,
        setup_idx: 0,
        cad_doc: &cad_doc,
        cad_revision: 1,
        cache: &cache,
        kinematics: kin,
        resolution: SimResolution::Standard,
        cancel: None,
        engine: SimEngineKind::Heightmap,
        backend_choice: SimComputeBackendChoice::Cpu,
        ..SetupSimRequest::new(&proj, 0, &cad_doc, 1, &cache)
    })
    .unwrap();
    assert_eq!(
        sim.critical_error_count(),
        0,
        "mesh-3mf-bracket critical errors: {:#?}",
        sim.diagnostics
    );
    let err = sim.error_report.as_ref().unwrap();
    assert!(
        err.max_gouge_mm <= 0.03,
        "mesh-3mf-bracket max_gouge_mm {:.4} > 0.03",
        err.max_gouge_mm
    );

    assert_posts_cleanly(&proj, &cad_doc, &cache, 0, "G54", "mesh_3mf_bracket");
}

/// `keepout-clamps`: Shared Avoid / Keep-Out region bundling two overhanging `ToeClamp` fixtures
/// and a 2D sketched central boss keep-out shared across Facing, 3D Adaptive (`ModelBoundingBox` +
/// signed offsets), and 3D Parallel Finish, plus `Contour2D` with `avoid_all_fixtures = true` —
/// verified `0` critical errors, `0` `FixtureCollision`, `0` `TargetModelGouge`, `0` `RapidIntoStock`.
#[test]
fn sample_keepout_clamps_e2e_shared_avoid_and_zero_fixture_collisions() {
    let step_bytes = sample_keepout_clamp_plate_step_bytes().unwrap();
    let cad_doc = MonstertruckKernel
        .load_step_bytes(
            "sample_keepout_clamp_plate.step",
            &step_bytes,
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    let sha = compute_sha256_hex(&step_bytes);
    let starter = default_starter_tool_library();

    let mut proj = build_keepout_clamps_project(
        &cad_doc,
        "sample_keepout_clamp_plate.step".to_string(),
        &sha,
        &starter.tools,
    );

    assert_eq!(proj.setups.len(), 1);
    assert_eq!(proj.setups[0].operations.len(), 4);
    assert_eq!(proj.avoid_regions.len(), 1);
    assert_eq!(proj.avoid_regions[0].id, "avoid_clamps_boss");
    assert_eq!(proj.avoid_regions[0].fixture_ids.len(), 2);
    assert_eq!(proj.avoid_regions[0].selections.len(), 1);

    let shared_users = proj.setups[0]
        .operations
        .iter()
        .filter(|op| {
            op.avoid
                .shared_region_ids
                .iter()
                .any(|id| id == "avoid_clamps_boss")
        })
        .count();
    assert_eq!(
        shared_users, 3,
        "expected exactly 3 operations linked to shared keep-out 'avoid_clamps_boss'"
    );
    assert!(
        proj.setups[0].operations[3].avoid.avoid_all_fixtures,
        "Op 4 Contour2D must enable avoid_all_fixtures"
    );
    assert!(matches!(
        proj.setups[0].operations[1].strategy,
        Some(StrategyParams::Adaptive3D(ref p))
            if p.boundary.boundary_mode == MachiningBoundaryMode::ModelBoundingBox
                && p.boundary.model_bounds_offset_x_neg.value_mm < 0.0
                && p.boundary.model_bounds_offset_y_pos.value_mm > 0.0
    ));
    assert_toml_roundtrip_deterministic(&proj);

    let val = validate_project(&proj, None, Some(&cad_doc));
    assert!(
        val.is_valid(),
        "keepout-clamps validation: {:?}",
        val.diagnostics
    );

    let mut cache = ToolpathCache::new(64);
    let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
    assert!(
        batch.is_ok(),
        "keepout-clamps generation: {:?}",
        batch.errors
    );
    assert_eq!(batch.generated.len(), 4);

    let kin = resolve_kinematics_for_key(&proj.setups[0].post_profile, None);
    let sim = simulate_project_setup(&SetupSimRequest {
        project: &proj,
        setup_idx: 0,
        cad_doc: &cad_doc,
        cad_revision: 1,
        cache: &cache,
        kinematics: kin,
        resolution: SimResolution::Standard,
        cancel: None,
        engine: SimEngineKind::Heightmap,
        backend_choice: SimComputeBackendChoice::Cpu,
        ..SetupSimRequest::new(&proj, 0, &cad_doc, 1, &cache)
    })
    .unwrap();

    assert_eq!(
        sim.critical_error_count(),
        0,
        "keepout-clamps critical errors: {:#?}",
        sim.diagnostics
    );
    for kind in [
        SimIssueKind::FixtureCollision,
        SimIssueKind::TargetModelGouge,
        SimIssueKind::RapidIntoStock,
    ] {
        assert!(
            !sim.diagnostics.iter().any(|d| d.kind == kind),
            "keepout-clamps reported unexpected {kind:?}: {:#?}",
            sim.diagnostics
        );
    }

    let err = sim.error_report.as_ref().unwrap();
    assert!(
        err.max_gouge_mm <= 0.02,
        "keepout-clamps max_gouge_mm {:.4} > 0.02",
        err.max_gouge_mm
    );

    assert_posts_cleanly(&proj, &cad_doc, &cache, 0, "G54", "keepout_clamps");
}

/// `indexed-model-bounds`: 4-sided sculpted STEP bar in `FixedBox` with `FixedBoxZAlign::CenterZ`
/// on an `X`/`A` `Indexed3Plus1` rotary setup with `-X` chuck, exercising `ModelBoundingBox` with
/// both negative (`-X = -1.0 mm` chuck margin) and positive (`+X = +2.0 mm`, `±Y = +1.5 mm`)
/// per-side offsets across `0°`, `90°`, `180°`, and `270°` — verified `0` crashes, `0` gouges,
/// `0` chuck collisions, and clean 4-axis G-code.
#[test]
fn sample_indexed_model_bounds_e2e_center_z_offsets_and_zero_errors() {
    let step_bytes = sample_indexed_4sided_bar_step_bytes().unwrap();
    let cad_doc = MonstertruckKernel
        .load_step_bytes(
            "sample_indexed_4sided_bar.step",
            &step_bytes,
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    let sha = compute_sha256_hex(&step_bytes);
    let starter = default_starter_tool_library();

    let mut proj = build_indexed_model_bounds_project(
        "sample_indexed_4sided_bar.step".to_string(),
        &sha,
        &starter.tools,
    );

    assert_eq!(proj.setups.len(), 1);
    assert_eq!(proj.setups[0].operations.len(), 4);
    assert!(matches!(
        proj.setups[0].stock,
        StockConfig::FixedBox {
            z_align: FixedBoxZAlign::CenterZ,
            ..
        }
    ));

    let expected_angles = [0.0_f64, 90.0, 180.0, 270.0];
    for (op, expected_deg) in proj.setups[0].operations.iter().zip(expected_angles) {
        assert!(matches!(
            op.rotary,
            OperationRotaryMode::Indexed3Plus1 { angle_deg } if (angle_deg - expected_deg).abs() < 1e-6
        ));
        let boundary = match op.strategy.as_ref() {
            Some(StrategyParams::Adaptive3D(p)) => &p.boundary,
            Some(StrategyParams::Parallel3D(p)) => &p.boundary,
            other => panic!("unexpected strategy on {}: {other:?}", op.id),
        };
        assert_eq!(
            boundary.boundary_mode,
            MachiningBoundaryMode::ModelBoundingBox
        );
        assert!(
            boundary.model_bounds_offset_x_neg.value_mm < 0.0,
            "expected negative -X offset on {} for chuck clearance",
            op.id
        );
        assert!(
            boundary.model_bounds_offset_x_pos.value_mm > 0.0
                && boundary.model_bounds_offset_y_neg.value_mm > 0.0
                && boundary.model_bounds_offset_y_pos.value_mm > 0.0,
            "expected positive +X / ±Y offsets on {}",
            op.id
        );
    }
    assert_toml_roundtrip_deterministic(&proj);

    let val = validate_project(&proj, None, Some(&cad_doc));
    assert!(
        val.is_valid(),
        "indexed-model-bounds validation: {:?}",
        val.diagnostics
    );

    let mut cache = ToolpathCache::new(64);
    let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
    assert!(
        batch.is_ok(),
        "indexed-model-bounds generation: {:?}",
        batch.errors
    );
    assert_eq!(batch.generated.len(), 4);

    let kin = resolve_kinematics_for_key(&proj.setups[0].post_profile, None);
    let sim = simulate_project_setup(&SetupSimRequest {
        project: &proj,
        setup_idx: 0,
        cad_doc: &cad_doc,
        cad_revision: 1,
        cache: &cache,
        kinematics: kin,
        resolution: SimResolution::Standard,
        cancel: None,
        engine: SimEngineKind::Heightmap,
        backend_choice: SimComputeBackendChoice::Cpu,
        ..SetupSimRequest::new(&proj, 0, &cad_doc, 1, &cache)
    })
    .unwrap();

    assert_eq!(
        sim.critical_error_count(),
        0,
        "indexed-model-bounds critical errors: {:#?}",
        sim.diagnostics
    );
    for kind in [
        SimIssueKind::FixtureCollision,
        SimIssueKind::TargetModelGouge,
        SimIssueKind::RapidIntoStock,
    ] {
        assert!(
            !sim.diagnostics.iter().any(|d| d.kind == kind),
            "indexed-model-bounds reported unexpected {kind:?}: {:#?}",
            sim.diagnostics
        );
    }

    let err = sim.error_report.as_ref().unwrap();
    assert!(
        err.max_gouge_mm <= 0.02,
        "indexed-model-bounds max_gouge_mm {:.4} > 0.02",
        err.max_gouge_mm
    );

    // Post-process using linuxcnc_4axis to verify A-axis indexing G-code
    let profiles = builtin_post_profiles();
    let lcnc4 = profiles.iter().find(|p| p.id == "linuxcnc_4axis").unwrap();
    let files = post_process_project_setup(&SetupPostRequest {
        project: &proj,
        setup_idx: 0,
        cad_doc: &cad_doc,
        cad_revision: 1,
        cache: &cache,
        post_profile: lcnc4,
        project_label: &proj.project_name,
        program_stem: "indexed_model_bounds",
    })
    .unwrap();
    let combined: String = files.iter().map(|f| f.content.as_str()).collect();
    assert!(combined.contains("G54"), "missing G54");
    assert!(combined.contains("A90"), "missing A90 index move");
    assert!(combined.contains("A180"), "missing A180 index move");
    assert!(combined.contains("A270"), "missing A270 index move");
    assert!(
        !combined.contains("NaN") && !combined.contains("inf") && !combined.contains("INF"),
        "indexed-model-bounds 4-axis G-code contains non-finite numeric output"
    );
}

/// Verify all 6 minimal tutorial starter assets checked in under `docs/tutorials/assets/`
/// match their programmatic generators byte-for-byte.
#[test]
fn checked_in_m68_tutorial_starter_assets_match_generators_byte_for_byte() {
    let assets_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("docs")
        .join("tutorials")
        .join("assets");

    let expected: Vec<(&str, Vec<u8>)> = vec![
        (
            "20-feed-opt-pocket.step",
            tutorial_adaptive_pocket_step().unwrap().into_bytes(),
        ),
        (
            "21-sloped-boss-deburr.step",
            sample_sloped_boss_stepped_pocket_step()
                .unwrap()
                .into_bytes(),
        ),
        (
            "21-overhang-flange.step",
            sample_overhang_flange_step().unwrap().into_bytes(),
        ),
        (
            "22-undercut-deburr-block.step",
            sample_undercut_deburr_block_step().unwrap().into_bytes(),
        ),
        ("13-badge-logo.svg", sample_svg_badge_bytes()),
        ("08-bracket.3mf", sample_sculpted_mount_3mf_bytes()),
    ];

    for (filename, gen_bytes) in expected {
        let path = assets_dir.join(filename);
        let disk_bytes = std::fs::read(&path)
            .unwrap_or_else(|e| panic!("missing tutorial starter asset {}: {e}", path.display()));
        assert_eq!(
            disk_bytes,
            gen_bytes,
            "checked-in tutorial starter asset {} differs from programmatic generator",
            path.display()
        );
    }
}
