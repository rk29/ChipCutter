#![allow(clippy::unwrap_used, clippy::expect_used)]

//! Milestone 59 — Automated Quantitative Quality Regression Suite.
//!
//! Enforces strict error-volume, gouge-freedom, RMS, and in-tolerance surface accuracy gates
//! across ChipCutter's 4th-axis rotary, 3-axis surface, 2.5D adaptive/rest, helical thread
//! milling, and engagement-aware feedrate optimization workflows.

use chip_cad::{
    CadDocument, CadKernel, MeshImportOptions, MonstertruckKernel, load_mesh_cad_document,
    sample_helical_flute_shaft_stl_bytes, sample_relief_wave_obj_text,
    sample_rotary_shaft_stl_bytes, sample_sculpted_dome_stl_bytes, sample_threaded_manifold_step,
    tutorial_adaptive_pocket_step, tutorial_pocket_block_step,
};
use chip_cam::FeedOptimizationConfig;
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, ParamLength};
use chip_project::{
    ProjectDocument, SetupSimRequest, ToolpathCache, build_rotary_helical_flute_project,
    build_rotary_shaft_preview_project, build_sculpted_mesh_dome_project,
    build_thread_mill_manifold_project, build_tutorial_3d_adaptive_waterline_project,
    build_tutorial_3d_surface_finishing_project, build_tutorial_adaptive_rest_project,
    build_tutorial_pocket_project, compute_sha256_hex, generate_project_blocking,
    resolve_kinematics_for_key, simulate_project_setup,
};
use chip_sim::{SetupSimulationResult, SimComputeBackendChoice, SimEngineKind, SimResolution};
use chip_tools::default_starter_tool_library;

fn run_quality_sim(
    label: &str,
    proj: &mut ProjectDocument,
    cad_doc: &CadDocument,
    engine: SimEngineKind,
    resolution: SimResolution,
) -> SetupSimulationResult {
    let mut cache = ToolpathCache::new(32);
    let batch = generate_project_blocking(proj, cad_doc, &mut cache, 1);
    assert!(
        batch.is_ok(),
        "{label}: toolpath generation failed: {:?}",
        batch.errors
    );
    let post_key = proj.setups[0].post_profile.clone();
    let sim = simulate_project_setup(&SetupSimRequest {
        project: proj,
        setup_idx: 0,
        cad_doc,
        cad_revision: 1,
        cache: &cache,
        kinematics: resolve_kinematics_for_key(&post_key, None),
        resolution,
        cancel: None,
        engine,
        backend_choice: SimComputeBackendChoice::Auto,
        ..SetupSimRequest::new(proj, 0, cad_doc, 1, &cache)
    })
    .unwrap_or_else(|e| panic!("{label}: simulation failed: {e}"));

    if let Some(err) = &sim.error_report {
        eprintln!(
            "[M59 Quality] {label:<28} | crit={} | errVol={:7.1} mm3 (ex={:7.1}, gg={:6.2}) | RMS={:.4} mm | maxEx={:.3} | maxGg={:.3} | inTol={:5.1}% | segs={} | {:.0} ms",
            sim.critical_error_count(),
            err.total_error_volume_mm3,
            err.excess_volume_mm3,
            err.gouge_volume_mm3,
            err.rms_error_mm,
            err.max_excess_mm,
            err.max_gouge_mm,
            err.in_tolerance_surface_pct,
            sim.timing.total_segments,
            sim.timing.total_ms,
        );
    }
    sim
}

#[test]
fn quality_gate_simultaneous_4axis_helical_flute() {
    let starter = default_starter_tool_library();
    let stl = sample_helical_flute_shaft_stl_bytes();
    let doc = load_mesh_cad_document(
        "helical_flute_shaft.stl",
        &stl,
        &MeshImportOptions::default(),
    )
    .unwrap();
    let sha = compute_sha256_hex(&stl);
    let mut proj = build_rotary_helical_flute_project(
        "helical_flute_shaft.stl".to_string(),
        &sha,
        &starter.tools,
    );

    let sim = run_quality_sim(
        "rotary_4axis_sim",
        &mut proj,
        &doc,
        SimEngineKind::Heightmap,
        SimResolution::Standard,
    );
    assert_eq!(
        sim.critical_error_count(),
        0,
        "rotary_4axis_sim must have 0 critical errors: {:#?}",
        sim.diagnostics
    );
    let err = sim
        .error_report
        .as_ref()
        .expect("rotary_4axis_sim must produce StockErrorReport");
    assert!(
        err.max_gouge_mm <= 0.020,
        "rotary_4axis_sim max_gouge_mm={:.4} exceeded 0.020 mm",
        err.max_gouge_mm
    );
    assert!(
        err.gouge_volume_mm3 <= 1.0,
        "rotary_4axis_sim gouge_volume_mm3={:.3} exceeded 1.0 mm^3",
        err.gouge_volume_mm3
    );
    // Massively improved over v0.15.0 baseline (RMS 0.529 mm, In-Tol 17.8%)
    assert!(
        err.rms_error_mm <= 0.505,
        "rotary_4axis_sim rms_error_mm={:.4} regressed (v0.15.0 baseline was 0.529)",
        err.rms_error_mm
    );
    assert!(
        err.in_tolerance_surface_pct >= 62.0,
        "rotary_4axis_sim in_tolerance_surface_pct={:.1}% fell below 62% (v0.15.0 baseline was 17.8%)",
        err.in_tolerance_surface_pct
    );
}

#[test]
fn quality_gate_rotary_indexed_shaft() {
    let starter = default_starter_tool_library();
    let stl = sample_rotary_shaft_stl_bytes();
    let doc =
        load_mesh_cad_document("rotary_shaft.stl", &stl, &MeshImportOptions::default()).unwrap();
    let sha = compute_sha256_hex(&stl);
    let mut proj =
        build_rotary_shaft_preview_project("rotary_shaft.stl".to_string(), &sha, &starter.tools);

    let sim = run_quality_sim(
        "rotary_shaft",
        &mut proj,
        &doc,
        SimEngineKind::Heightmap,
        SimResolution::Standard,
    );
    assert_eq!(
        sim.critical_error_count(),
        0,
        "rotary_shaft must have 0 critical errors: {:#?}",
        sim.diagnostics
    );
    let err = sim
        .error_report
        .as_ref()
        .expect("rotary_shaft must produce StockErrorReport");
    assert!(
        err.max_gouge_mm <= 0.015,
        "rotary_shaft max_gouge_mm={:.4} exceeded 0.015 mm",
        err.max_gouge_mm
    );
    assert!(
        err.gouge_volume_mm3 <= 1.0,
        "rotary_shaft gouge_volume_mm3={:.3} exceeded 1.0 mm^3",
        err.gouge_volume_mm3
    );
    // Significantly improved over v0.15.0 baseline (RMS 0.884 mm, In-Tol 34.0%, ErrVol 5090.5 mm3)
    assert!(
        err.rms_error_mm <= 0.84,
        "rotary_shaft rms_error_mm={:.4} regressed (v0.15.0 baseline was 0.884)",
        err.rms_error_mm
    );
    assert!(
        err.in_tolerance_surface_pct >= 50.0,
        "rotary_shaft in_tolerance_surface_pct={:.1}% fell below 50% (v0.15.0 baseline was 34.0%)",
        err.in_tolerance_surface_pct
    );
}

#[test]
fn quality_gate_3d_surface_finishing_zero_gouge() {
    let starter = default_starter_tool_library();
    let obj = sample_relief_wave_obj_text();
    let doc = load_mesh_cad_document(
        "09-relief-wave.obj",
        obj.as_bytes(),
        &MeshImportOptions::default(),
    )
    .unwrap();
    let sha = compute_sha256_hex(obj.as_bytes());
    let mut proj = build_tutorial_3d_surface_finishing_project(
        "09-relief-wave.obj".to_string(),
        &sha,
        &starter.tools,
    );

    let sim = run_quality_sim(
        "09-3d-surface-finishing",
        &mut proj,
        &doc,
        SimEngineKind::TriDexel,
        SimResolution::Standard,
    );
    assert_eq!(
        sim.critical_error_count(),
        0,
        "09-3d-surface-finishing must have 0 critical errors: {:#?}",
        sim.diagnostics
    );
    let err = sim
        .error_report
        .as_ref()
        .expect("09-3d-surface-finishing must produce StockErrorReport");
    // Eliminates v0.15.0's 65.1 mm^3 / -0.146 mm gouge completely!
    assert!(
        err.gouge_volume_mm3 <= 1.0,
        "09-3d-surface-finishing gouge_volume_mm3={:.3} exceeded 1.0 mm^3 (v0.15.0 was 65.1)",
        err.gouge_volume_mm3
    );
    assert!(
        err.max_gouge_mm <= 0.015,
        "09-3d-surface-finishing max_gouge_mm={:.4} exceeded 0.015 mm (v0.15.0 was 0.146)",
        err.max_gouge_mm
    );
    assert!(
        err.in_tolerance_surface_pct >= 76.0,
        "09-3d-surface-finishing in_tolerance_surface_pct={:.1}% fell below 76.0%",
        err.in_tolerance_surface_pct
    );
}

#[test]
fn quality_gate_3d_mesh_surfacing() {
    let starter = default_starter_tool_library();
    let stl = sample_sculpted_dome_stl_bytes();
    let doc = load_mesh_cad_document("08-sculpted-dome.stl", &stl, &MeshImportOptions::default())
        .unwrap();
    let sha = compute_sha256_hex(&stl);

    // Sculpted Dome (`08-mesh-surfacing`: Adaptive3D + Waterline3D + Pencil3D)
    let mut dome_proj =
        build_sculpted_mesh_dome_project("08-sculpted-dome.stl".to_string(), &sha, &starter.tools);
    let dome_sim = run_quality_sim(
        "08-mesh-surfacing",
        &mut dome_proj,
        &doc,
        SimEngineKind::TriDexel,
        SimResolution::Fine,
    );
    assert_eq!(
        dome_sim.critical_error_count(),
        0,
        "08-mesh-surfacing must have 0 critical errors: {:#?}",
        dome_sim.diagnostics
    );
    let dome_err = dome_sim
        .error_report
        .as_ref()
        .expect("08-mesh-surfacing must produce StockErrorReport");
    assert!(
        dome_err.max_gouge_mm <= 0.015,
        "08-mesh-surfacing max_gouge_mm={:.4} exceeded 0.015 mm",
        dome_err.max_gouge_mm
    );
    assert!(
        dome_err.in_tolerance_surface_pct >= 54.0,
        "08-mesh-surfacing in_tolerance_surface_pct={:.1}% fell below 54.0%",
        dome_err.in_tolerance_surface_pct
    );
}

#[test]
fn quality_gate_3d_adaptive_waterline() {
    let starter = default_starter_tool_library();
    let stl = sample_sculpted_dome_stl_bytes();
    let doc = load_mesh_cad_document("08-sculpted-dome.stl", &stl, &MeshImportOptions::default())
        .unwrap();
    let sha = compute_sha256_hex(&stl);

    // Tutorial 08 (`08-3d-adaptive-waterline`: Adaptive3D + Adaptive Waterline3D)
    let mut wl_proj = build_tutorial_3d_adaptive_waterline_project(
        "08-sculpted-dome.stl".to_string(),
        &sha,
        &starter.tools,
    );
    let wl_sim = run_quality_sim(
        "08-3d-adaptive-waterline",
        &mut wl_proj,
        &doc,
        SimEngineKind::TriDexel,
        SimResolution::Standard,
    );
    assert_eq!(
        wl_sim.critical_error_count(),
        0,
        "08-3d-adaptive-waterline must have 0 critical errors: {:#?}",
        wl_sim.diagnostics
    );
    let wl_err = wl_sim
        .error_report
        .as_ref()
        .expect("08-3d-adaptive-waterline must produce StockErrorReport");
    assert!(
        wl_err.max_gouge_mm <= 0.015,
        "08-3d-adaptive-waterline max_gouge_mm={:.4} exceeded 0.015 mm",
        wl_err.max_gouge_mm
    );
    assert!(
        wl_err.in_tolerance_surface_pct >= 54.0,
        "08-3d-adaptive-waterline in_tolerance_surface_pct={:.1}% fell below 54.0%",
        wl_err.in_tolerance_surface_pct
    );
}

#[test]
fn quality_gate_2d_adaptive_and_rest_machining() {
    let step = tutorial_adaptive_pocket_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes(
            "05-adaptive-pocket.step",
            step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    let sha = compute_sha256_hex(step.as_bytes());
    let mut proj = build_tutorial_adaptive_rest_project(
        &doc,
        "05-adaptive-pocket.step".to_string(),
        &sha,
        false,
    );

    let sim = run_quality_sim(
        "05-adaptive-rest",
        &mut proj,
        &doc,
        SimEngineKind::TriDexel,
        SimResolution::Standard,
    );
    assert_eq!(
        sim.critical_error_count(),
        0,
        "05-adaptive-rest must have 0 critical errors: {:#?}",
        sim.diagnostics
    );
    let err = sim
        .error_report
        .as_ref()
        .expect("05-adaptive-rest must produce StockErrorReport");
    assert!(
        err.gouge_volume_mm3 <= 0.1,
        "05-adaptive-rest gouge_volume_mm3={:.4} exceeded 0.1 mm^3",
        err.gouge_volume_mm3
    );
    assert!(
        err.in_tolerance_surface_pct >= 99.0,
        "05-adaptive-rest in_tolerance_surface_pct={:.2}% fell below 99.0%",
        err.in_tolerance_surface_pct
    );
    assert!(
        err.total_error_volume_mm3 <= 40.0,
        "05-adaptive-rest total_error_volume_mm3={:.2} exceeded 40.0 mm^3 (v0.15.0 was 131.0)",
        err.total_error_volume_mm3
    );
}

#[test]
fn quality_gate_helical_thread_milling() {
    let starter = default_starter_tool_library();
    let step = sample_threaded_manifold_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes(
            "18-threaded-manifold.step",
            step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    let sha = compute_sha256_hex(step.as_bytes());
    let mut proj = build_thread_mill_manifold_project(
        &doc,
        "18-threaded-manifold.step".to_string(),
        &sha,
        &starter.tools,
    );

    let sim = run_quality_sim(
        "18-thread-milling",
        &mut proj,
        &doc,
        SimEngineKind::TriDexel,
        SimResolution::Standard,
    );
    assert_eq!(
        sim.critical_error_count(),
        0,
        "18-thread-milling must have 0 critical errors: {:#?}",
        sim.diagnostics
    );
    let err = sim
        .error_report
        .as_ref()
        .expect("18-thread-milling must produce StockErrorReport");
    assert!(
        err.in_tolerance_surface_pct >= 94.0,
        "18-thread-milling in_tolerance_surface_pct={:.1}% fell below 94.0%",
        err.in_tolerance_surface_pct
    );
}

#[test]
fn quality_gate_feedrate_optimization_slows_corners_with_exact_geometric_parity() {
    let starter = default_starter_tool_library();
    let step = tutorial_pocket_block_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes(
            "03-pocket-block.step",
            step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    let sha = compute_sha256_hex(step.as_bytes());

    let mut base_proj = build_tutorial_pocket_project(
        &doc,
        "03-pocket-block.step".to_string(),
        &sha,
        &starter.tools,
        false,
    );
    let mut opt_proj = base_proj.clone();
    for op in &mut opt_proj.setups[0].operations {
        op.passes.feed_optimization = FeedOptimizationConfig {
            enabled: true,
            corner_slowdown: true,
            min_feed_fraction: 0.35,
            corner_angle_threshold_deg: 25.0,
            slowdown_distance: ParamLength::from_mm(3.0),
            internal_arc_scaling: true,
            chip_thinning_boost: false,
            max_thinning_multiplier: 1.5,
        };
    }

    let base_sim = run_quality_sim(
        "pocket_feed_baseline",
        &mut base_proj,
        &doc,
        SimEngineKind::Heightmap,
        SimResolution::Standard,
    );
    let opt_sim = run_quality_sim(
        "pocket_feed_optimized",
        &mut opt_proj,
        &doc,
        SimEngineKind::Heightmap,
        SimResolution::Standard,
    );

    assert_eq!(base_sim.critical_error_count(), 0);
    assert_eq!(
        opt_sim.critical_error_count(),
        0,
        "Feed-optimized pocket must have 0 critical errors: {:#?}",
        opt_sim.diagnostics
    );

    // Corner slowdown & internal arc scaling must reduce minimum cutting feedrate cleanly
    assert!(
        opt_sim.timeline.feed_modulated_segments > 0,
        "expected feed-modulated slowdown segments on pocket with internal corners"
    );
    assert!(
        opt_sim.timeline.min_cutting_feed_mm_min < base_sim.timeline.min_cutting_feed_mm_min - 1.0,
        "expected lower min cutting feedrate with FeedOptimizationConfig enabled: opt={:.1}, base={:.1}",
        opt_sim.timeline.min_cutting_feed_mm_min,
        base_sim.timeline.min_cutting_feed_mm_min
    );

    // Exact geometric parity between baseline and feed-optimized stock removal
    let base_err = base_sim.error_report.as_ref().unwrap();
    let opt_err = opt_sim.error_report.as_ref().unwrap();
    assert!(
        opt_err.gouge_volume_mm3 <= 0.05 && opt_err.max_gouge_mm <= 0.015,
        "Feed-optimized pocket must remain gouge-free: ggVol={:.4}, maxGg={:.4}",
        opt_err.gouge_volume_mm3,
        opt_err.max_gouge_mm
    );
    assert!(
        (opt_err.total_error_volume_mm3 - base_err.total_error_volume_mm3).abs() <= 0.5,
        "Feed optimization must preserve exact geometric parity: opt_err={:.3}, base_err={:.3}",
        opt_err.total_error_volume_mm3,
        base_err.total_error_volume_mm3
    );
    assert!(
        (opt_err.rms_error_mm - base_err.rms_error_mm).abs() <= 0.002,
        "Feed optimization must preserve exact RMS parity: opt_rms={:.5}, base_rms={:.5}",
        opt_err.rms_error_mm,
        base_err.rms_error_mm
    );
}
