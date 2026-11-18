use std::{path::PathBuf, process::ExitCode};

use chip_post::resolve_post_profile;
use chip_project::{
    DiagnosticSeverity, SetupPostRequest, SetupSimRequest, ToolpathCache,
    generate_project_blocking, kinematics_for_post_profile, post_process_project_setup,
    resolve_kinematics_for_key, simulate_project_setup, validate_project,
};
use chip_sim::{SetupSimulationResult, SimResolution, format_cycle_time_hms};

use crate::commands::{
    BuildFileSummaryJson, BuildSummaryJson, ProjectSimReportJson, SetupSimReportJson,
    load_primary_cad_for_project, load_project_or_chipz, parse_sim_resolution,
};

pub fn run_validate(
    project: PathBuf,
    generate: bool,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let loaded = load_project_or_chipz(&project)?;
    let mut proj_doc = loaded.proj;
    let proj_dir = loaded.dir.as_deref();
    let loaded_cad = load_primary_cad_for_project(&proj_doc, proj_dir).ok();

    let report = validate_project(&proj_doc, proj_dir, loaded_cad.as_ref());
    println!(
        "=== ChipCutter Project Validation: '{}' ===",
        proj_doc.project_name
    );
    println!(
        "Models: {} | Sketches: {} | Tools: {} | Setups: {}",
        proj_doc.models.len(),
        proj_doc.sketches.len(),
        proj_doc.tools.len(),
        proj_doc.setups.len()
    );
    for diag in &report.diagnostics {
        let tag = match diag.severity {
            DiagnosticSeverity::Warning => "WARN ",
            DiagnosticSeverity::Error => "ERROR",
        };
        println!("  [{tag}] {}: {}", diag.scope, diag.message);
    }

    let mut gen_failed = false;
    if generate {
        if let Some(cad_doc) = loaded_cad.as_ref() {
            let mut cache = ToolpathCache::new(64);
            let batch = generate_project_blocking(&mut proj_doc, cad_doc, &mut cache, 1);
            println!("=== Toolpath Generation Summary ===");
            for (scope, _id, tp) in &batch.generated {
                println!(
                    "  [OK  ] {}: {} pass(es), {} G1, {} G2/G3, {} drill, cut={:.1} mm, rapid={:.1} mm, est={:.1}s",
                    scope,
                    tp.stats.pass_count,
                    tp.stats.linear_move_count,
                    tp.stats.arc_move_count,
                    tp.stats.drill_cycle_count,
                    tp.stats.cutting_distance_mm,
                    tp.stats.rapid_distance_mm,
                    tp.stats.total_est_time_sec
                );
                for w in &tp.warnings {
                    println!("         [WARN] {w}");
                }
            }
            for (scope, err_msg) in &batch.errors {
                eprintln!("  [FAIL] {scope}: {err_msg}");
                gen_failed = true;
            }
        } else {
            eprintln!("  [FAIL] Cannot generate toolpaths: no valid STEP model loaded.");
            gen_failed = true;
        }
    }

    if report.is_valid() && !gen_failed {
        println!(
            "Validation PASSED ({} warning(s), 0 errors).",
            report.warning_count()
        );
        Ok(ExitCode::SUCCESS)
    } else {
        eprintln!(
            "Validation FAILED ({} validation error(s), {} warning(s)).",
            report.error_count(),
            report.warning_count()
        );
        Ok(ExitCode::FAILURE)
    }
}

pub fn run_simulate(
    project: PathBuf,
    resolution: String,
    report: Option<PathBuf>,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let sim_res_enum = parse_sim_resolution(&resolution);
    let loaded = load_project_or_chipz(&project)?;
    let mut proj_doc = loaded.proj;
    let proj_dir = loaded.dir.as_deref();
    let cad_doc = load_primary_cad_for_project(&proj_doc, proj_dir)?;

    let val_report = validate_project(&proj_doc, proj_dir, Some(&cad_doc));
    if !val_report.is_valid() {
        for d in &val_report.diagnostics {
            eprintln!(
                "  [{}] {}: {}",
                if d.severity == DiagnosticSeverity::Error {
                    "ERROR"
                } else {
                    "WARN"
                },
                d.scope,
                d.message
            );
        }
        return Ok(ExitCode::FAILURE);
    }

    let mut cache = ToolpathCache::new(128);
    let batch = generate_project_blocking(&mut proj_doc, &cad_doc, &mut cache, 1);
    if !batch.errors.is_empty() {
        for (scope, msg) in &batch.errors {
            eprintln!("  [GEN ERROR] {scope}: {msg}");
        }
        return Ok(ExitCode::FAILURE);
    }

    println!(
        "=== ChipCutter Heightmap Simulation: '{}' [{}] ===",
        proj_doc.project_name,
        sim_res_enum.label()
    );

    let mut setup_reports = Vec::with_capacity(proj_doc.setups.len());
    let mut any_critical = false;

    for (s_idx, setup) in proj_doc.setups.iter().enumerate() {
        let kin = resolve_kinematics_for_key(&setup.post_profile, proj_dir);
        let sim = simulate_project_setup(&SetupSimRequest {
            project: &proj_doc,
            setup_idx: s_idx,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            kinematics: kin,
            resolution: sim_res_enum,
            cancel: None,
        })?;

        let hms = format_cycle_time_hms(sim.timeline.total_time_sec);
        println!(
            "Setup '{}' [{}]: Cycle = {} ({:.1}s cut, {:.1}s rapid, {:.1}s ATC) | Stock: {:.0} -> {:.0} mm^3 (removed {:.0} mm^3)",
            setup.name,
            setup.wcs_slot.short_code(),
            hms,
            sim.timeline.total_cutting_time_sec,
            sim.timeline.total_rapid_time_sec,
            sim.timeline.total_tool_change_time_sec,
            sim.initial_stock_volume_mm3,
            sim.remaining_stock_volume_mm3,
            sim.removed_volume_mm3
        );
        for d in &sim.diagnostics {
            println!(
                "  {} [{}] {} (seg #{}, t={:.2}s): {}",
                d.severity.badge(),
                d.kind.label(),
                d.op_name,
                d.global_seg_idx,
                d.timeline_time_sec,
                d.message
            );
        }

        if sim.has_critical_errors() {
            any_critical = true;
        }
        setup_reports.push(build_sim_report_json(
            &setup.id,
            &setup.name,
            sim_res_enum,
            sim,
        ));
    }

    let full_report = ProjectSimReportJson {
        project_name: proj_doc.project_name.clone(),
        passed: !any_critical,
        setups: setup_reports,
    };

    if let Some(report_path) = report {
        if let Some(parent) = report_path.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(&report_path, serde_json::to_string_pretty(&full_report)?)?;
        println!("Wrote JSON simulation report to {}", report_path.display());
    }

    if any_critical {
        eprintln!("Simulation FAILED: Critical crash or gouge detected!");
        Ok(ExitCode::FAILURE)
    } else {
        println!("Simulation PASSED (0 critical crashes/gouges).");
        Ok(ExitCode::SUCCESS)
    }
}

pub struct BuildArgs {
    pub project: PathBuf,
    pub output_dir: PathBuf,
    pub setup_filter: Option<String>,
    pub post_override: Option<String>,
    pub allow_warnings: bool,
}

pub fn run_build(args: BuildArgs) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let loaded = load_project_or_chipz(&args.project)?;
    let mut proj_doc = loaded.proj;
    let proj_dir = loaded.dir.as_deref();
    let cad_doc = load_primary_cad_for_project(&proj_doc, proj_dir)?;

    let val_report = validate_project(&proj_doc, proj_dir, Some(&cad_doc));
    if !val_report.is_valid() {
        for d in &val_report.diagnostics {
            eprintln!("  [VALIDATION] {}: {}", d.scope, d.message);
        }
        return Ok(ExitCode::FAILURE);
    }

    let mut cache = ToolpathCache::new(128);
    let batch = generate_project_blocking(&mut proj_doc, &cad_doc, &mut cache, 1);
    if !batch.errors.is_empty() {
        for (scope, msg) in &batch.errors {
            eprintln!("  [GEN ERROR] {scope}: {msg}");
        }
        return Ok(ExitCode::FAILURE);
    }

    std::fs::create_dir_all(&args.output_dir)?;

    let proj_file_name = proj_doc.project_name.as_str();
    let proj_stem = args
        .project
        .file_stem()
        .and_then(|s| s.to_str())
        .map(|s| s.trim_end_matches(".chip"))
        .unwrap_or("program");

    let mut emitted_summaries = Vec::new();
    let mut sim_summaries = Vec::new();
    let mut matched_any_setup = false;

    for (s_idx, setup) in proj_doc.setups.iter().enumerate() {
        if let Some(filter) = &args.setup_filter {
            let f_low = filter.to_ascii_lowercase();
            if setup.name.to_ascii_lowercase() != f_low && setup.id.to_ascii_lowercase() != f_low {
                continue;
            }
        }
        matched_any_setup = true;

        let profile_key = args.post_override.as_deref().unwrap_or(&setup.post_profile);
        let post_profile = resolve_post_profile(profile_key, proj_dir)?;
        let kin = kinematics_for_post_profile(&post_profile);

        // Mandatory pre-post physical heightmap crash & gouge gate (`06-simulation.md` §4)!
        let sim = simulate_project_setup(&SetupSimRequest {
            project: &proj_doc,
            setup_idx: s_idx,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            kinematics: kin,
            resolution: SimResolution::Standard,
            cancel: None,
        })?;

        for d in &sim.diagnostics {
            eprintln!(
                "  {} [{}] {}: {}",
                d.severity.badge(),
                d.kind.label(),
                d.op_name,
                d.message
            );
        }
        if sim.has_critical_errors() {
            eprintln!(
                "Build ABORTED for Setup '{}': {} critical crash/gouge error(s) detected!",
                setup.name,
                sim.critical_error_count()
            );
            return Ok(ExitCode::FAILURE);
        }
        if sim.warning_count() > 0 && !args.allow_warnings {
            println!(
                "  [NOTE] Setup '{}' has {} simulation advisory warning(s) (continuing; pass --allow-warnings to acknowledge).",
                setup.name,
                sim.warning_count()
            );
        }

        let setup_stem = if proj_doc.setups.len() > 1 {
            format!("{proj_stem}_{}", setup.id)
        } else {
            proj_stem.to_string()
        };
        let nc_files = post_process_project_setup(&SetupPostRequest {
            project: &proj_doc,
            setup_idx: s_idx,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            post_profile: &post_profile,
            project_label: proj_file_name,
            program_stem: &setup_stem,
        })?;

        for nc in nc_files {
            let out_path = args.output_dir.join(&nc.filename);
            std::fs::write(&out_path, &nc.content)?;
            println!(
                "  [NC  ] Wrote {} ({} lines, max {} chars/line, tools {:?})",
                out_path.display(),
                nc.line_count,
                nc.max_line_chars,
                nc.tool_numbers
            );
            emitted_summaries.push(BuildFileSummaryJson {
                setup_name: setup.name.clone(),
                post_profile_id: post_profile.id.clone(),
                filename: nc.filename,
                line_count: nc.line_count,
                max_line_chars: nc.max_line_chars,
                tool_numbers: nc.tool_numbers,
                operation_names: nc.operation_names,
                est_time_sec: nc.est_time_sec,
            });
        }

        sim_summaries.push(build_sim_report_json(
            &setup.id,
            &setup.name,
            SimResolution::Standard,
            sim,
        ));
    }

    if !matched_any_setup {
        eprintln!("No Setup matched filter {:?}.", args.setup_filter);
        return Ok(ExitCode::FAILURE);
    }

    let summary_path = args.output_dir.join("build_summary.json");
    let summary = BuildSummaryJson {
        project_name: proj_doc.project_name,
        files: emitted_summaries,
        simulation: sim_summaries,
    };
    std::fs::write(&summary_path, serde_json::to_string_pretty(&summary)?)?;
    println!(
        "Build SUCCEEDED — summary written to {}",
        summary_path.display()
    );
    Ok(ExitCode::SUCCESS)
}

fn build_sim_report_json(
    setup_id: &str,
    setup_name: &str,
    resolution: SimResolution,
    sim: SetupSimulationResult,
) -> SetupSimReportJson {
    let hms = format_cycle_time_hms(sim.timeline.total_time_sec);
    let crit = sim.critical_error_count();
    let warn = sim.warning_count();
    SetupSimReportJson {
        setup_id: setup_id.to_string(),
        setup_name: setup_name.to_string(),
        resolution,
        total_time_sec: sim.timeline.total_time_sec,
        total_time_hms: hms,
        cutting_time_sec: sim.timeline.total_cutting_time_sec,
        rapid_time_sec: sim.timeline.total_rapid_time_sec,
        tool_change_time_sec: sim.timeline.total_tool_change_time_sec,
        initial_stock_volume_mm3: sim.initial_stock_volume_mm3,
        remaining_stock_volume_mm3: sim.remaining_stock_volume_mm3,
        removed_volume_mm3: sim.removed_volume_mm3,
        critical_error_count: crit,
        warning_count: warn,
        diagnostics: sim.diagnostics,
    }
}
