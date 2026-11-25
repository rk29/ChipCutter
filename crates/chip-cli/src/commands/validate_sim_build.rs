use std::{path::PathBuf, process::ExitCode};

use chip_cam::MachinabilitySeverity;
use chip_post::{
    PostProfileSource, resolve_post_profile, resolve_post_profile_cli_override,
    validate_safe_output_filename,
};
use chip_project::{
    DiagnosticSeverity, ProjectMachinabilityReport, SetupPostRequest, SetupSimRequest,
    ToolpathCache, ValidationDiagnostic, analyze_project_machinability,
    generate_project_blocking_in_dir, kinematics_for_post_profile, post_process_project_setup,
    simulate_all_setups_with_choice, simulate_project_setup_in_dir,
    simulate_project_setup_with_choice_in_dir, validate_project, validate_project_with_machine,
};
use chip_sim::{
    SetupSimulationResult, SimComputeBackendChoice, SimEngineKind, SimResolution, StockModelKind,
    format_cycle_time_hms,
};
use chip_tools::{
    builtin_generic_machine, resolve_machine_definition, resolve_machine_definition_cli_override,
};
use clap::Args;
use serde::Serialize;

use crate::commands::{
    BuildFileSummaryJson, BuildSummaryJson, ProjectSimReportJson, SetupSimReportJson,
    load_primary_cad_for_project, load_project_or_chipz, parse_sim_backend, parse_sim_engine,
    parse_sim_resolution,
};

#[derive(Debug)]
pub struct ValidateOptions {
    pub project: PathBuf,
    pub generate: bool,
    pub machine_override: Option<String>,
    pub physics: bool,
    pub strict_physics: bool,
    pub json: bool,
}

#[derive(Debug, Serialize)]
struct ValidateReportJson {
    project_name: String,
    machine_id: String,
    passed: bool,
    error_count: usize,
    warning_count: usize,
    diagnostics: Vec<ValidationDiagnostic>,
    #[serde(skip_serializing_if = "Option::is_none")]
    physics: Option<ProjectMachinabilityReport>,
}

pub fn run_validate(opts: ValidateOptions) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let loaded = load_project_or_chipz(&opts.project)?;
    let mut proj_doc = loaded.proj;
    let proj_dir = loaded.dir.as_deref();
    let loaded_cad = load_primary_cad_for_project(&proj_doc, proj_dir).ok();
    let explicit_machine = match opts.machine_override.as_deref() {
        Some(mid) => Some(resolve_machine_definition_cli_override(mid, proj_dir)?),
        None => None,
    };

    let report = validate_project_with_machine(
        &proj_doc,
        proj_dir,
        loaded_cad.as_ref(),
        explicit_machine.as_ref(),
    );
    let active_machine_label = explicit_machine
        .as_ref()
        .map_or(proj_doc.effective_machine_id(), |m| m.id.as_str())
        .to_string();

    if !opts.json {
        println!(
            "=== ChipCutter Project Validation: '{}' ===",
            proj_doc.project_name
        );
        println!(
            "Machine: {} | Models: {} | Sketches: {} | Tools: {} | Setups: {}",
            active_machine_label,
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
    }

    let mut gen_failed = false;
    if opts.generate {
        if let Some(cad_doc) = loaded_cad.as_ref() {
            let mut cache = ToolpathCache::new(64);
            let batch =
                generate_project_blocking_in_dir(&mut proj_doc, cad_doc, proj_dir, &mut cache, 1);
            if !opts.json {
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
            }
            for (scope, err_msg) in &batch.errors {
                if !opts.json {
                    eprintln!("  [FAIL] {scope}: {err_msg}");
                }
                gen_failed = true;
            }
        } else {
            if !opts.json {
                eprintln!("  [FAIL] Cannot generate toolpaths: no valid STEP model loaded.");
            }
            gen_failed = true;
        }
    }

    let physics_report = if opts.physics {
        let resolved_machine = match explicit_machine {
            Some(m) => m,
            None => resolve_machine_definition(proj_doc.effective_machine_id(), proj_dir)
                .unwrap_or_else(|_| builtin_generic_machine()),
        };
        let phys = analyze_project_machinability(&proj_doc, &resolved_machine, loaded_cad.as_ref());
        if !opts.json {
            println!(
                "=== Machinability & Cutting Physics (Material: {} | Machine: {}) ===",
                phys.material_name, phys.machine_name
            );
            for s in &phys.setups {
                for op in &s.operations {
                    if op.suppressed {
                        continue;
                    }
                    if let Some(m) = op.metrics {
                        let load_str = m
                            .spindle_load_pct
                            .map_or_else(|| "n/a".to_string(), |p| format!("{p:.0}%"));
                        let sf_str = if m.deflection.bending_safety_factor.is_finite() {
                            format!("{:.2}", m.deflection.bending_safety_factor)
                        } else {
                            "inf".to_string()
                        };
                        println!(
                            "  [{:<8}] {}/{}: MRR={:.2} cm^3/min | Load={} ({:.1}W) | Defl={:.1}um | SF={} | h_max={:.4}mm",
                            op.severity.label(),
                            s.setup_name,
                            op.op_name,
                            m.mrr_cm3_min,
                            load_str,
                            m.cutting_power_watts,
                            m.deflection.tip_deflection_um,
                            sf_str,
                            m.effective_chip_thickness_mm
                        );
                    }
                    for f in &op.findings {
                        let fix_note = f
                            .suggested_fix
                            .as_ref()
                            .map(|fx| format!(" -> Fix: {}", fx.label()))
                            .unwrap_or_default();
                        println!(
                            "             [{}] {}: {}{}",
                            f.severity.label(),
                            f.title,
                            f.detail,
                            fix_note
                        );
                    }
                }
            }
            println!(
                "Physics summary: {} critical, {} warning(s), {} info (overall: {}).",
                phys.critical_count,
                phys.warning_count,
                phys.info_count,
                phys.severity.label()
            );
        }
        Some(phys)
    } else {
        None
    };

    let strict_physics_failed = opts.strict_physics
        && physics_report
            .as_ref()
            .is_some_and(|p| p.severity == MachinabilitySeverity::Critical || p.critical_count > 0);
    let passed = report.is_valid() && !gen_failed && !strict_physics_failed;

    if opts.json {
        let payload = ValidateReportJson {
            project_name: proj_doc.project_name.clone(),
            machine_id: active_machine_label,
            passed,
            error_count: report.error_count(),
            warning_count: report.warning_count(),
            diagnostics: report.diagnostics,
            physics: physics_report,
        };
        println!("{}", serde_json::to_string_pretty(&payload)?);
    } else if passed {
        println!(
            "Validation PASSED ({} warning(s), 0 errors).",
            report.warning_count()
        );
    } else if strict_physics_failed {
        eprintln!(
            "Validation FAILED (--strict-physics: {} critical machinability finding(s)).",
            physics_report.as_ref().map_or(0, |p| p.critical_count)
        );
    } else {
        eprintln!(
            "Validation FAILED ({} validation error(s), {} warning(s)).",
            report.error_count(),
            report.warning_count()
        );
    }

    Ok(if passed {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    })
}

#[derive(Debug, Args)]
pub struct SimulateArgs {
    /// Path to `.chip.toml` project file (or `.chipz` bundle).
    pub project: PathBuf,
    /// Simulation engine (`heightmap` fast 2.5D Z-dexel, or `tri-dexel` true 3D volumetric).
    #[arg(long, default_value = "heightmap")]
    pub engine: String,
    /// Simulation compute backend (`auto` GPU with CPU fallback, `gpu`, or `cpu`).
    #[arg(long, default_value = "auto")]
    pub backend: String,
    /// Simulation resolution (`standard`, `fine`, `ultra`).
    #[arg(long, default_value = "standard")]
    pub resolution: String,
    /// Surface deviation pass tolerance band in millimeters for error metrics (`±tolerance`).
    #[arg(long, default_value_t = 0.05)]
    pub tolerance: f64,
    /// Print structured per-stage timing & throughput benchmark summary.
    #[arg(long)]
    pub benchmark: bool,
    /// Simulate all Setups sequentially with volumetric stock carryover and emit cumulative end-of-job metrology.
    #[arg(long)]
    pub all_setups: bool,
    /// Emit structured JSON simulation, backend, timing, and error-volume report to stdout.
    #[arg(long)]
    pub json: bool,
    /// Optional path to write structured JSON simulation & error-volume report.
    #[arg(long)]
    pub report: Option<PathBuf>,
}

pub fn run_simulate(args: SimulateArgs) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let sim_engine = parse_sim_engine(&args.engine)?;
    let sim_backend = parse_sim_backend(&args.backend)?;
    let sim_res_enum = parse_sim_resolution(&args.resolution);
    let tolerance_mm = args.tolerance.abs().max(1e-4);
    let loaded = load_project_or_chipz(&args.project)?;
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
    let batch = generate_project_blocking_in_dir(&mut proj_doc, &cad_doc, proj_dir, &mut cache, 1);
    if !batch.errors.is_empty() {
        for (scope, msg) in &batch.errors {
            eprintln!("  [GEN ERROR] {scope}: {msg}");
        }
        return Ok(ExitCode::FAILURE);
    }

    if !args.json {
        println!(
            "=== ChipCutter {} Simulation: '{}' [{}] ===",
            sim_engine.label(),
            proj_doc.project_name,
            sim_res_enum.label_for_engine(sim_engine)
        );
    }

    let all_sims = simulate_all_setups_with_choice(
        &proj_doc,
        &cad_doc,
        1,
        &cache,
        proj_dir,
        sim_res_enum,
        sim_engine,
        sim_backend,
        None,
    )?;

    let mut setup_reports = Vec::with_capacity(proj_doc.setups.len());
    let mut any_critical = false;
    let mut final_error_report = None;

    for (setup, mut sim) in proj_doc.setups.iter().zip(all_sims) {
        if (tolerance_mm - 0.05).abs() > 1e-6
            && let Some(recomputed) = sim.compute_error_report_at_tolerance(tolerance_mm)
        {
            sim.error_report = Some(recomputed);
        }

        if !args.json {
            let hms = format_cycle_time_hms(sim.timeline.total_time_sec);
            let backend_short = if sim.active_backend.is_gpu {
                format!("GPU/{}", sim.active_backend.backend_api)
            } else {
                "CPU".to_string()
            };
            let stock_tag = stock_kind_slug(sim.stock_kind());
            let carry_tag = sim
                .carried_over_from_setup
                .as_deref()
                .map(|src| format!(" · carried over from '{src}'"))
                .unwrap_or_default();
            println!(
                "Setup '{}' [{}] ({} · {}{}): Cycle = {} ({:.1}s cut, {:.1}s rapid, {:.1}s ATC) | Stock: {:.0} -> {:.0} mm^3 (removed {:.0} mm^3)",
                setup.name,
                setup.wcs_slot.short_code(),
                stock_tag,
                backend_short,
                carry_tag,
                hms,
                sim.timeline.total_cutting_time_sec,
                sim.timeline.total_rapid_time_sec,
                sim.timeline.total_tool_change_time_sec,
                sim.initial_stock_volume_mm3,
                sim.remaining_stock_volume_mm3,
                sim.removed_volume_mm3
            );
            if args.benchmark {
                let backend_mode = if sim.active_backend.is_gpu {
                    "gpu"
                } else {
                    "cpu"
                };
                println!(
                    "  Benchmark: backend={} | adapter='{}' ({}) | engine={} | stock={} | resolution={}",
                    backend_mode,
                    sim.active_backend.adapter_name,
                    sim.active_backend.backend_api,
                    sim.engine.short_slug(),
                    stock_tag,
                    sim.resolution.label_for_engine(sim.engine),
                );
                println!(
                    "  Timing: total={:.2} ms (target_setup={:.2} ms, sweep={:.2} ms, metrics={:.2} ms) | segments={} | throughput={:.1} seg/s",
                    sim.timing.total_ms,
                    sim.timing.target_setup_ms,
                    sim.timing.sweep_ms,
                    sim.timing.metrics_ms,
                    sim.timing.total_segments,
                    sim.timing.segments_per_sec,
                );
            }
            if let Some(err) = &sim.error_report {
                println!(
                    "  Error Volume: total={:.2} mm^3 ({:.2}% of target) | excess={:.2} mm^3 | gouge={:.2} mm^3 | efficiency={:.1}%",
                    err.total_error_volume_mm3,
                    err.error_volume_pct_of_target,
                    err.excess_volume_mm3,
                    err.gouge_volume_mm3,
                    err.material_removal_efficiency_pct
                );
                println!(
                    "  Surface Deviation (±{:.3} mm): RMS={:.4} mm | MAE={:.4} mm | Bias={:+.4} mm | P95={:.4} mm | P99={:.4} mm | In-Tol={:.1}% | MaxExcess=+{:.4} mm | MaxGouge=-{:.4} mm",
                    err.tolerance_mm,
                    err.rms_error_mm,
                    err.mean_abs_error_mm,
                    err.mean_signed_error_mm,
                    err.p95_abs_error_mm,
                    err.p99_abs_error_mm,
                    err.in_tolerance_surface_pct,
                    err.max_excess_mm,
                    err.max_gouge_mm
                );
            }
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
        }

        if sim.has_critical_errors() {
            any_critical = true;
        }
        if let Some(ref rep) = sim.error_report {
            final_error_report = Some(rep.clone());
        }
        setup_reports.push(build_sim_report_json(
            &setup.id,
            &setup.name,
            sim_res_enum,
            sim,
        ));
    }

    let cumulative_error_report = if args.all_setups {
        final_error_report
    } else {
        None
    };

    if !args.json
        && args.all_setups
        && let Some(ref cum) = cumulative_error_report
    {
        println!(
            "=== Cumulative Multi-Setup Metrology ({} Setup(s), ±{:.3} mm) ===",
            setup_reports.len(),
            cum.tolerance_mm
        );
        println!(
            "  Final Error Volume: total={:.2} mm^3 ({:.2}% of target) | excess={:.2} mm^3 | gouge={:.2} mm^3 | efficiency={:.1}%",
            cum.total_error_volume_mm3,
            cum.error_volume_pct_of_target,
            cum.excess_volume_mm3,
            cum.gouge_volume_mm3,
            cum.material_removal_efficiency_pct
        );
        println!(
            "  Final Surface Deviation: RMS={:.4} mm | In-Tol={:.1}% | MaxExcess=+{:.4} mm | MaxGouge=-{:.4} mm",
            cum.rms_error_mm, cum.in_tolerance_surface_pct, cum.max_excess_mm, cum.max_gouge_mm
        );
    }

    let full_report = ProjectSimReportJson {
        project_name: proj_doc.project_name.clone(),
        passed: !any_critical,
        setups: setup_reports,
        cumulative_error_report,
    };
    let json_text = if args.json || args.report.is_some() {
        Some(serde_json::to_string_pretty(&full_report)?)
    } else {
        None
    };

    if let Some(report_path) = args.report {
        if let Some(parent) = report_path.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        if let Some(text) = &json_text {
            std::fs::write(&report_path, text)?;
        }
        if !args.json {
            println!("Wrote JSON simulation report to {}", report_path.display());
        }
    }

    if args.json {
        if let Some(text) = &json_text {
            println!("{text}");
        }
    } else if any_critical {
        eprintln!("Simulation FAILED: Critical crash or gouge detected!");
    } else {
        println!("Simulation PASSED (0 critical crashes/gouges).");
    }

    if any_critical {
        Ok(ExitCode::FAILURE)
    } else {
        Ok(ExitCode::SUCCESS)
    }
}

#[derive(Debug, Args)]
pub struct BuildArgs {
    /// Path to `.chip.toml` project file (or `.chipz` bundle).
    pub project: PathBuf,
    /// Output directory for emitted `.nc`/`.ngc` G-code files and `build_summary.json`.
    #[arg(short, long, default_value = "out")]
    pub output_dir: PathBuf,
    /// Optional filter to build only a specific Setup (by Setup name or ID).
    #[arg(long = "setup")]
    pub setup_filter: Option<String>,
    /// Optional post-processor profile ID (`grbl`, `grblhal_atc`, `linuxcnc`) or `.post.toml` file path override.
    #[arg(long = "post")]
    pub post_override: Option<String>,
    /// Allow non-critical simulation warnings (critical crashes/gouges still block build!).
    #[arg(long)]
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
    let batch = generate_project_blocking_in_dir(&mut proj_doc, &cad_doc, proj_dir, &mut cache, 1);
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

        let post_profile = if let Some(cli_post) = args.post_override.as_deref() {
            resolve_post_profile_cli_override(cli_post, proj_dir)?
        } else {
            resolve_post_profile(&setup.post_profile, proj_dir)?
        };
        if post_profile.source == PostProfileSource::ProjectOrBundle {
            eprintln!(
                "  [POST WARN] Setup '{}' uses custom post profile '{}' loaded from project/bundle directory; review emitted header/footer G-code before machining.",
                setup.name, post_profile.id
            );
        }
        let kin = kinematics_for_post_profile(&post_profile);

        // Mandatory pre-post physical crash, gouge, & fixture gate (`06-simulation.md` §4)!
        // Automatically select full-3D Tri-Dexel when a Setup carries over volumetric stock or
        // runs undercut strategies / relieved-neck undercut cutters (`SlotMill`, `DovetailCutter`, `LollipopEndmill`).
        let needs_tridexel = setup.carryover_from_setup_id.is_some()
            || setup.operations.iter().any(|op| {
                if op.suppressed {
                    return false;
                }
                if matches!(
                    op.kind,
                    chip_cam::OperationKind::SlotUndercut
                        | chip_cam::OperationKind::Dovetail
                        | chip_cam::OperationKind::Undercut3D
                ) {
                    return true;
                }
                op.tool_id
                    .as_ref()
                    .and_then(|tid| proj_doc.tool(tid))
                    .is_some_and(|t| {
                        matches!(
                            t.geometry,
                            chip_tools::CutterGeometry::LollipopEndmill { .. }
                                | chip_tools::CutterGeometry::SlotMill { .. }
                                | chip_tools::CutterGeometry::DovetailCutter { .. }
                        )
                    })
            });
        let sim_req = SetupSimRequest {
            project: &proj_doc,
            setup_idx: s_idx,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            kinematics: kin,
            resolution: SimResolution::Standard,
            cancel: None,
        };
        let sim = if needs_tridexel {
            simulate_project_setup_with_choice_in_dir(
                &sim_req,
                proj_dir,
                SimEngineKind::TriDexel,
                SimComputeBackendChoice::Auto,
            )?
        } else {
            simulate_project_setup_in_dir(&sim_req, proj_dir)?
        };

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
            let safe_name = validate_safe_output_filename(&nc.filename)?;
            let out_path = args.output_dir.join(safe_name);
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

fn stock_kind_slug(kind: StockModelKind) -> &'static str {
    match kind {
        StockModelKind::Heightfield25D => "heightfield",
        StockModelKind::TriDexel3D => "tri_dexel",
        StockModelKind::Cylindrical4Axis(_) => "cylindrical",
    }
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
    let stock_kind = stock_kind_slug(sim.stock_kind()).to_string();
    SetupSimReportJson {
        setup_id: setup_id.to_string(),
        setup_name: setup_name.to_string(),
        carried_over_from_setup: sim.carried_over_from_setup,
        engine: sim.engine,
        stock_kind,
        resolution,
        backend: sim.active_backend,
        timing: sim.timing,
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
        error_report: sim.error_report,
    }
}
