#![allow(clippy::unwrap_used, clippy::expect_used)]

//! End-to-end Phase 18 expanded machining regression, CPU-vs-GPU Tri-Dexel parity, and
//! undercut safety fault-injection suite (`chip-project`, Milestone 67).

use chip_cad::{CadKernel, MonstertruckKernel, sample_undercut_deburr_block_step_bytes};
use chip_cam::{MotionRole, ToolpathBuilder};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec3, DisplayUnit};
use chip_post::builtin_post_profiles;
use chip_project::{
    ProjectDocument, SetupPostRequest, SetupSimRequest, ToolpathCache,
    build_undercut_deburr_block_project, compute_sha256_hex, generate_project_blocking,
    post_process_project_setup, resolve_kinematics_for_key, simulate_project_setup,
    validate_project,
};
use chip_sim::{
    MachineKinematicsConfig, SetupSimulationInput, SimComputeBackendChoice, SimEngineKind,
    SimIssueKind, SimOperationInput, SimResolution, SimSeverity, SimVerificationConfig,
    run_setup_simulation,
};
use chip_tools::{default_starter_tool_library, starter_tslot_12mm};

/// 1. Full built-in `undercut-deburr` 6-operation pipeline:
///    - Deterministic `.chip.toml` roundtrip & `validate_project` (0 errors),
///    - Parallel generation of all 6 operations (`Face Top`, `Pre-Slot Throats`, `M8 T-Slot`,
///      `60° Dovetail`, `Lollipop Overhang Undercut`, `3D Edge Deburr & Back-Chamfer`),
///    - Full `TriDexel` (`Fine`) simulation on BOTH `CpuExactBackend` and `WgpuComputeBackend` (`Auto`),
///    - Asserts `0` critical errors, `max_gouge_mm <= 0.02 mm` on exact modeled B-Rep features (Ops 1–5),
///      `In-Tol >= 90%`, intact M8 T-slot roof & accurate `60°` dovetail flanks, exact CPU-vs-GPU
///      removed volume & `StockErrorReport` parity, and valid `GRBL` + `LinuxCNC` G-code output.
#[test]
fn undercut_deburr_sample_full_pipeline_cpu_gpu_tridexel_parity_and_posts() {
    let step_bytes = sample_undercut_deburr_block_step_bytes().unwrap();
    let cad_doc = MonstertruckKernel
        .load_step_bytes(
            "sample_undercut_deburr_block.step",
            &step_bytes,
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    let sha = compute_sha256_hex(&step_bytes);
    let starter = default_starter_tool_library();

    let mut proj = build_undercut_deburr_block_project(
        &cad_doc,
        "sample_undercut_deburr_block.step".to_string(),
        &sha,
        &starter.tools,
    );
    assert_eq!(proj.setups.len(), 1);
    assert_eq!(
        proj.setups[0].operations.len(),
        6,
        "undercut-deburr sample must have 6 sequenced operations"
    );

    // Deterministic TOML serialization roundtrip
    let toml_a = proj.to_toml_string().unwrap();
    let reparsed = ProjectDocument::from_toml_str(&toml_a).unwrap();
    assert_eq!(toml_a, reparsed.to_toml_string().unwrap());

    // Project validation passes cleanly with 0 errors
    let val = validate_project(&proj, None, Some(&cad_doc));
    assert!(
        val.is_valid(),
        "undercut-deburr validation failed: {:?}",
        val.diagnostics
    );

    // Generate all 6 operations cleanly
    let mut cache = ToolpathCache::new(64);
    let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
    assert!(
        batch.is_ok(),
        "undercut-deburr generation errors: {:?}",
        batch.errors
    );
    assert_eq!(batch.generated.len(), 6);

    let kin = resolve_kinematics_for_key(&proj.setups[0].post_profile, None);

    // A. Simulate exact B-Rep net-shape Ops 1–5 (prior to intentional unmodeled Deburr3D edge-break)
    //    at Fine Tri-Dexel resolution to assert strict `max_gouge_mm <= 0.02 mm`!
    let mut proj_ops1_5 = proj.clone();
    proj_ops1_5.setups[0].operations[5].suppressed = true;
    let sim_ops1_5 = simulate_project_setup(&SetupSimRequest {
        project: &proj_ops1_5,
        setup_idx: 0,
        cad_doc: &cad_doc,
        cad_revision: 1,
        cache: &cache,
        kinematics: kin,
        resolution: SimResolution::Fine,
        cancel: None,
        engine: SimEngineKind::TriDexel,
        backend_choice: SimComputeBackendChoice::Auto,
        ..SetupSimRequest::new(&proj_ops1_5, 0, &cad_doc, 1, &cache)
    })
    .unwrap();
    assert_eq!(
        sim_ops1_5.critical_error_count(),
        0,
        "Ops 1–5 must have 0 critical errors: {:#?}",
        sim_ops1_5.diagnostics
    );
    let err_1_5 = sim_ops1_5
        .error_report
        .as_ref()
        .expect("Ops 1–5 StockErrorReport present");
    eprintln!(
        "[M67 undercut-deburr Ops 1-5 Fine ({})] In-Tol={:.2}% RMS={:.4}mm MaxGouge={:.4}mm GougeVol={:.2}mm³ ExcessVol={:.1}mm³",
        if sim_ops1_5.active_backend.is_gpu {
            "GPU"
        } else {
            "CPU"
        },
        err_1_5.in_tolerance_surface_pct,
        err_1_5.rms_error_mm,
        err_1_5.max_gouge_mm,
        err_1_5.gouge_volume_mm3,
        err_1_5.excess_volume_mm3,
    );
    assert!(
        err_1_5.max_gouge_mm <= 0.02,
        "Ops 1–5 max_gouge_mm ({:.4} mm) exceeded 0.02 mm at {:?}",
        err_1_5.max_gouge_mm,
        err_1_5.max_gouge_location_wcs
    );
    assert!(
        err_1_5.in_tolerance_surface_pct >= 90.0,
        "Ops 1–5 In-Tol ({:.2}%) fell below 90.0%",
        err_1_5.in_tolerance_surface_pct
    );

    // B. Full 6-operation Tri-Dexel Fine simulation on BOTH CPU and GPU (Auto) backends
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
            "undercut-deburr ({label}) must have 0 critical errors: {:#?}",
            sim.diagnostics
        );
        let err = sim
            .error_report
            .as_ref()
            .expect("StockErrorReport must be present");
        eprintln!(
            "[M67 undercut-deburr 6-Op Fine ({label}, is_gpu={})] In-Tol={:.2}% RMS={:.4}mm MaxGouge={:.4}mm Removed={:.1}mm³",
            sim.active_backend.is_gpu,
            err.in_tolerance_surface_pct,
            err.rms_error_mm,
            err.max_gouge_mm,
            sim.removed_volume_mm3,
        );
        assert!(
            err.in_tolerance_surface_pct >= 90.0,
            "undercut-deburr ({label}) In-Tol ({:.2}%) fell below 90.0%",
            err.in_tolerance_surface_pct
        );
        // Op 6 removes measurable intentional 0.4 mm bevel material beyond Op 5 without gouging core body
        assert!(
            sim.removed_volume_mm3 > sim_ops1_5.removed_volume_mm3 + 5.0,
            "Op 6 Deburr3D ({label}) must remove measurable edge-break bevel volume (6-op={:.2} vs 5-op={:.2})",
            sim.removed_volume_mm3,
            sim_ops1_5.removed_volume_mm3
        );

        // Probe exact Tri-Dexel undercut geometry at X = 0 (WCS center):
        // Setup WCS origin = StockBoxPoint::TOP_CENTER => stock X ∈ [-50, 50], Y ∈ [-40, 40], Z ∈ [-29, 0]
        // Model Z = 28 -> WCS Z = -1.0; T-Slot roof Z_model ∈ [20, 28] -> WCS Z ∈ [-9, -1];
        // T-Slot head Z_model ∈ [14, 20] -> WCS Z ∈ [-15, -9], centered at Y_wcs = 20 - 40 = -20;
        // 60° Dovetail Z_model ∈ [14, 19] -> WCS Z ∈ [-15, -10], centered at Y_wcs = 52 - 40 = +12.
        let td = sim.as_tridexel().expect("TriDexel stock present");
        let y_plane = td.y_plane();
        let u_x0 = ((0.0 - y_plane.origin_u_mm) / y_plane.du_mm)
            .round()
            .clamp(0.0, (y_plane.nu - 1) as f64) as usize;
        let spans_at_z = |z_wcs: f64| {
            let v_z = ((z_wcs - y_plane.origin_v_mm) / y_plane.dv_mm)
                .round()
                .clamp(0.0, (y_plane.nv - 1) as f64) as usize;
            (y_plane.center_v(v_z), y_plane.ray_spans(u_x0, v_z))
        };

        // 1. Intact T-slot & dovetail roof bridges at Z_wcs = -5.0 (`Z_model = 24.0`):
        //    3 solid spans separated by the two 8.5 mm throats at Y = -20 and Y = +12!
        let (_, roof_spans) = spans_at_z(-5.0);
        assert_eq!(
            roof_spans.len(),
            3,
            "roof at Z_wcs=-5 ({label}) must stay intact as 3 solid bridges across both throats: {roof_spans:?}"
        );
        let tslot_throat_w = roof_spans[1].min_mm - roof_spans[0].max_mm;
        let dove_throat_w = roof_spans[2].min_mm - roof_spans[1].max_mm;
        assert!(
            (tslot_throat_w - 8.5).abs() < 0.15,
            "T-slot throat width ({label}): got {tslot_throat_w:.3}, expected 8.50"
        );
        assert!(
            (dove_throat_w - 8.5).abs() < 0.15,
            "Dovetail throat width ({label}): got {dove_throat_w:.3}, expected 8.50"
        );

        // 2. Undercut cavities at Z_wcs = -12.5 (`Z_model = 16.5`, mid-height of both T-slot & 60° dovetail!):
        let (z_actual, cav_spans) = spans_at_z(-12.5);
        assert_eq!(
            cav_spans.len(),
            3,
            "cavities at Z_wcs={z_actual:.2} ({label}) must have 3 solid walls: {cav_spans:?}"
        );
        let tslot_head_w = cav_spans[1].min_mm - cav_spans[0].max_mm;
        assert!(
            (tslot_head_w - 14.5).abs() < 0.15,
            "M8 T-slot undercut width ({label}): got {tslot_head_w:.3}, expected 14.50"
        );
        let dz_above_dove_floor = z_actual - (-15.0);
        let expected_dove_w = 8.5 + 2.0 * (5.0 - dz_above_dove_floor) / 60.0_f64.to_radians().tan();
        let dove_cav_w = cav_spans[2].min_mm - cav_spans[1].max_mm;
        assert!(
            (dove_cav_w - expected_dove_w).abs() < 0.15,
            "60° dovetail flank width at Z_wcs={z_actual:.2} ({label}): got {dove_cav_w:.3}, expected {expected_dove_w:.3}"
        );

        // 3. Solid uncut floor at Z_wcs = -20.0 (`Z_model = 9.0`): single solid span `[-40, +40]`
        let (_, floor_spans) = spans_at_z(-20.0);
        assert_eq!(
            floor_spans.len(),
            1,
            "floor at Z_wcs=-20 ({label}) must remain 1 solid span: {floor_spans:?}"
        );
    }

    // C. Exact CPU-vs-GPU volume & StockErrorReport parity
    let rel_vol_diff = (sim_gpu.removed_volume_mm3 - sim_cpu.removed_volume_mm3).abs()
        / sim_cpu.removed_volume_mm3.max(1.0);
    assert!(
        rel_vol_diff < 0.005,
        "CPU vs GPU removed volume parity exceeded 0.5%: cpu={:.2}, gpu={:.2} (rel={:.4})",
        sim_cpu.removed_volume_mm3,
        sim_gpu.removed_volume_mm3,
        rel_vol_diff
    );
    let err_cpu = sim_cpu.error_report.as_ref().unwrap();
    let err_gpu = sim_gpu.error_report.as_ref().unwrap();
    assert!(
        (err_gpu.in_tolerance_surface_pct - err_cpu.in_tolerance_surface_pct).abs() < 0.75,
        "CPU vs GPU In-Tol parity mismatch: cpu={:.2}%, gpu={:.2}%",
        err_cpu.in_tolerance_surface_pct,
        err_gpu.in_tolerance_surface_pct
    );
    assert!(
        (err_gpu.rms_error_mm - err_cpu.rms_error_mm).abs() < 0.005,
        "CPU vs GPU RMS parity mismatch: cpu={:.4}, gpu={:.4}",
        err_cpu.rms_error_mm,
        err_gpu.rms_error_mm
    );

    // D. Clean GRBL and LinuxCNC G-code post-processing
    let profiles = builtin_post_profiles();
    let grbl = profiles.iter().find(|p| p.id == "grbl").unwrap();
    let linuxcnc = profiles.iter().find(|p| p.id == "linuxcnc").unwrap();
    for profile in [grbl, linuxcnc] {
        let files = post_process_project_setup(&SetupPostRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            post_profile: profile,
            project_label: &proj.project_name,
            program_stem: "undercut_deburr",
        })
        .unwrap();
        assert!(!files.is_empty(), "{} emitted no G-code files", profile.id);
        let combined: String = files.iter().map(|f| f.content.as_str()).collect();
        assert!(combined.contains("G54"), "{} missing G54", profile.id);
        assert!(combined.contains("G0"), "{} missing G0 rapid", profile.id);
        assert!(combined.contains("G1"), "{} missing G1 feed", profile.id);
        assert!(
            !combined.contains("NaN") && !combined.contains("inf") && !combined.contains("INF"),
            "{} G-code contains non-finite numeric output!",
            profile.id
        );
    }
}

/// 2. Fault-injection: omitting Op 2 (`Pre-Slot Throats`) causes BOTH `SlotMill` (Op 3) AND
///    `DovetailCutter` (Op 4) to trigger critical neck/flute collision errors on BOTH CPU & GPU!
#[test]
fn fault_injection_missing_preslot_triggers_slotmill_and_dovetail_neck_collisions_cpu_and_gpu() {
    let step_bytes = sample_undercut_deburr_block_step_bytes().unwrap();
    let cad_doc = MonstertruckKernel
        .load_step_bytes(
            "sample_undercut_deburr_block.step",
            &step_bytes,
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    let sha = compute_sha256_hex(&step_bytes);
    let starter = default_starter_tool_library();

    let mut proj = build_undercut_deburr_block_project(
        &cad_doc,
        "sample_undercut_deburr_block.step".to_string(),
        &sha,
        &starter.tools,
    );
    // Deliberately suppress Op 2 (Pre-Slot Throats) while keeping Ops 1, 3 (SlotMill), and 4 (Dovetail)!
    proj.setups[0].operations[1].suppressed = true;
    proj.setups[0].operations[4].suppressed = true;
    proj.setups[0].operations[5].suppressed = true;

    let mut cache = ToolpathCache::new(32);
    let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
    assert!(batch.is_ok(), "generation errors: {:?}", batch.errors);

    let kin = resolve_kinematics_for_key(&proj.setups[0].post_profile, None);

    for (label, backend) in [
        ("CPU", SimComputeBackendChoice::Cpu),
        ("Auto/GPU", SimComputeBackendChoice::Auto),
    ] {
        let sim = simulate_project_setup(&SetupSimRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            kinematics: kin,
            resolution: SimResolution::Standard,
            cancel: None,
            engine: SimEngineKind::TriDexel,
            backend_choice: backend,
            ..SetupSimRequest::new(&proj, 0, &cad_doc, 1, &cache)
        })
        .unwrap();

        assert!(
            sim.has_critical_errors(),
            "omitting Op 2 pre-slot MUST trigger critical neck collision errors ({label})"
        );
        let has_slotmill_neck_crash = sim.diagnostics.iter().any(|d| {
            d.op_id == "op_3_tslot_m8"
                && d.severity == SimSeverity::CriticalError
                && d.kind == SimIssueKind::FluteLengthExceeded
        });
        let has_dovetail_neck_crash = sim.diagnostics.iter().any(|d| {
            d.op_id == "op_4_dovetail_60"
                && d.severity == SimSeverity::CriticalError
                && d.kind == SimIssueKind::FluteLengthExceeded
        });
        assert!(
            has_slotmill_neck_crash,
            "expected critical neck/flute collision on op_3_tslot_m8 ({label}), got: {:#?}",
            sim.diagnostics
        );
        assert!(
            has_dovetail_neck_crash,
            "expected critical neck/flute collision on op_4_dovetail_60 ({label}), got: {:#?}",
            sim.diagnostics
        );
    }
}

/// 3. Fault-injection: deliberate vertical retract through an undercut T-slot roof triggers
///    `RapidIntoStock` and `TargetModelGouge` critical errors on BOTH CPU & GPU!
#[test]
fn fault_injection_vertical_retract_through_undercut_roof_triggers_crash_and_gouge_cpu_and_gpu() {
    let step_bytes = sample_undercut_deburr_block_step_bytes().unwrap();
    let cad_doc = MonstertruckKernel
        .load_step_bytes(
            "sample_undercut_deburr_block.step",
            &step_bytes,
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    let sha = compute_sha256_hex(&step_bytes);
    let starter = default_starter_tool_library();

    let mut proj = build_undercut_deburr_block_project(
        &cad_doc,
        "sample_undercut_deburr_block.step".to_string(),
        &sha,
        &starter.tools,
    );
    // Keep only Op 1 (Face) + Op 2 (Pre-Slot Throats) so the M8 T-slot roof exists cleanly
    for op in proj.setups[0].operations.iter_mut().skip(2) {
        op.suppressed = true;
    }

    let mut cache = ToolpathCache::new(32);
    let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
    assert!(batch.is_ok(), "generation errors: {:?}", batch.errors);

    let resolved = proj.setups[0]
        .resolve(&cad_doc, DisplayUnit::Millimeter)
        .unwrap();
    let ready = chip_project::collect_ready_setup_operations(&proj, 0, &cache, 1);
    let slot_12 = starter_tslot_12mm();

    // Craft a deliberately unsafe T-slot toolpath that enters horizontally in open air (`X = -58`)
    // into the pre-slotted M8 channel (`Y_wcs = -20`, `Z_wcs = -15`), cuts to the middle (`X = 0`),
    // and then RETRACTS VERTICALLY (`G0 Z+5`) straight up through the overhanging aluminum roof!
    let mut bad_tb = ToolpathBuilder::new("op_bad_roof_retract");
    bad_tb.rapid_to(DVec3::new(-58.0, -20.0, 5.0));
    bad_tb.linear_to(DVec3::new(-58.0, -20.0, -15.0), 300.0, MotionRole::Plunge);
    bad_tb.linear_to(DVec3::new(0.0, -20.0, -15.0), 600.0, MotionRole::Cut);
    // FATAL BUG INJECTED: vertical G0 rapid straight up through the overhanging T-slot roof!
    bad_tb.rapid_to(DVec3::new(0.0, -20.0, 5.0));
    let bad_tp = bad_tb.finish();

    let mut sim_ops: Vec<SimOperationInput<'_>> = ready
        .iter()
        .map(|r| SimOperationInput::from_operation(r.op, r.tool, r.toolpath.as_ref()))
        .collect();
    sim_ops.push(SimOperationInput {
        op_id: "op_bad_roof_retract",
        op_name: "Unsafe Roof Retract",
        tool: &slot_12,
        fallback_spindle_rpm: 8_000.0,
        intentional_edge_break_mm: 0.0,
        toolpath: &bad_tp,
    });

    for (label, backend) in [
        ("CPU", SimComputeBackendChoice::Cpu),
        ("Auto/GPU", SimComputeBackendChoice::Auto),
    ] {
        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &cad_doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &sim_ops,
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::TriDexel,
            backend_choice: backend,
            ..SetupSimulationInput::new(&cad_doc, &resolved, &[])
        })
        .unwrap();

        assert!(
            sim.has_critical_errors(),
            "vertical retract through undercut roof MUST fail ({label})"
        );
        let has_rapid_crash = sim
            .diagnostics
            .iter()
            .any(|d| d.op_id == "op_bad_roof_retract" && d.kind == SimIssueKind::RapidIntoStock);
        let has_roof_gouge = sim
            .diagnostics
            .iter()
            .any(|d| d.op_id == "op_bad_roof_retract" && d.kind == SimIssueKind::TargetModelGouge);
        assert!(
            has_rapid_crash,
            "expected RapidIntoStock when retracting vertically through undercut roof ({label}), got: {:#?}",
            sim.diagnostics
        );
        assert!(
            has_roof_gouge,
            "expected TargetModelGouge when retracting vertically through undercut roof ({label}), got: {:#?}",
            sim.diagnostics
        );
    }
}
