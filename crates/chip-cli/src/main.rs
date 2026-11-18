use std::{
    path::{Path, PathBuf},
    process::ExitCode,
};

use chip_cad::{
    CadDocument, CadKernel, CurveKind, MonstertruckKernel, SurfaceKind, find_same_diameter_holes,
    sample_cam_torture_block_step, sample_machined_plate_step, sample_multi_feature_bracket_step,
};
use chip_cam::{
    ChamferSourceMode, ChamferTraceMode, GeometrySelectionItem, HeightReference, HeightSpec,
    HoleMachiningMode, Operation, OperationKind, StrategyParams, TabProfileKind,
};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit, ParamLength};
use chip_post::{
    OperationPostInput, SetupPostInput, describe_wcs_origin, post_process_setup,
    resolve_post_profile,
};
use chip_project::{
    DiagnosticSeverity, ModelEntry, ProjectDocument, ToolpathCache, compute_sha256_hex,
    generate_project_blocking, portable_model_path, resolve_model_path, validate_project,
};
use chip_sim::{
    MachineKinematicsConfig, SimDiagnostic, SimOperationInput, SimResolution, SimSeverity,
    SimVerificationConfig, format_cycle_time_hms, run_setup_simulation,
};
use chip_tools::{default_starter_tool_library, import_fusion_library_path};
use clap::{Parser, Subcommand};
use serde::Serialize;

#[derive(Debug, Parser)]
#[command(
    name = "chipcutter",
    version,
    about = "ChipCutter headless CAM CLI (STEP inspection, validation, simulation, G-code build, & tool import)"
)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Debug, Subcommand)]
enum Commands {
    /// Inspect an ISO 10303-21 STEP file, reporting normalized dimensions and B-Rep features.
    Inspect {
        /// Path to `.step` or `.stp` CAD model.
        path: PathBuf,
        /// Display unit for dimensions (`mm` or `in`).
        #[arg(long, default_value = "mm")]
        units: String,
    },
    /// Write ChipCutter's programmatic machined sample STEP file (and optional `.chip.toml` job) to disk.
    GenerateSampleStep {
        /// Output `.step` file path.
        output: PathBuf,
        /// Generate the Milestone 2 multi-feature bracket (with blind pocket/hole + 4 mounting holes).
        #[arg(long)]
        multi_feature: bool,
        /// Tilt the generated multi-feature bracket by 30° X / 25° Z to test WCS alignment.
        #[arg(long)]
        tilted: bool,
        /// Generate the Milestone 3 CAM torture block (island pocket, open step, wide bore, 45° bevel).
        #[arg(long)]
        cam_torture: bool,
        /// Optional path to also write a seeded `.chip.toml` CAM project referencing `output`.
        #[arg(long)]
        project_out: Option<PathBuf>,
    },
    /// Validate a `.chip.toml` project's STEP hashes, tools, Setups, WCS, selections, and Z heights.
    Validate {
        /// Path to `.chip.toml` project file.
        project: PathBuf,
        /// Also generate all enabled 2.5D toolpaths in parallel and verify non-empty CL-data output.
        #[arg(long)]
        generate: bool,
    },
    /// Run headless 2.5D Z-dexel heightmap stock removal & 5-rule crash/gouge verification (`06-simulation.md`).
    Simulate {
        /// Path to `.chip.toml` project file.
        project: PathBuf,
        /// Heightmap resolution (`standard` = 512, `fine` = 1024, `ultra` = 2048).
        #[arg(long, default_value = "standard")]
        resolution: String,
        /// Optional path to write structured JSON simulation report.
        #[arg(long)]
        report: Option<PathBuf>,
    },
    /// Generate toolpaths in parallel, verify zero critical crashes/gouges, and post-process G-code (`07-post-processor.md`).
    Build {
        /// Path to `.chip.toml` project file.
        project: PathBuf,
        /// Output directory for emitted `.nc`/`.ngc` G-code files and `build_summary.json`.
        #[arg(short, long, default_value = "out")]
        output_dir: PathBuf,
        /// Optional filter to build only a specific Setup (by Setup name or ID).
        #[arg(long)]
        setup: Option<String>,
        /// Optional post-processor profile ID (`grbl`, `grblhal_atc`, `linuxcnc`) or `.post.toml` file path override.
        #[arg(long)]
        post: Option<String>,
        /// Allow non-critical simulation warnings (critical crashes/gouges still block build!).
        #[arg(long)]
        allow_warnings: bool,
    },
    /// Tool library utilities (`import-fusion`).
    Tools {
        #[command(subcommand)]
        action: ToolCommands,
    },
}

#[derive(Debug, Subcommand)]
enum ToolCommands {
    /// Convert a Fusion 360 `.json` or `.tools` (ZIP) library into a native `.tools.toml` file.
    ImportFusion {
        /// Input Fusion 360 `.json` or `.tools` file.
        input: PathBuf,
        /// Output `.tools.toml` destination path.
        #[arg(short, long)]
        output: PathBuf,
        /// Optional custom library name.
        #[arg(long)]
        name: Option<String>,
    },
}

#[derive(Debug, Serialize)]
struct SetupSimReportJson {
    setup_id: String,
    setup_name: String,
    resolution: SimResolution,
    total_time_sec: f64,
    total_time_hms: String,
    cutting_time_sec: f64,
    rapid_time_sec: f64,
    tool_change_time_sec: f64,
    initial_stock_volume_mm3: f64,
    remaining_stock_volume_mm3: f64,
    removed_volume_mm3: f64,
    critical_error_count: usize,
    warning_count: usize,
    diagnostics: Vec<SimDiagnostic>,
}

#[derive(Debug, Serialize)]
struct ProjectSimReportJson {
    project_name: String,
    passed: bool,
    setups: Vec<SetupSimReportJson>,
}

#[derive(Debug, Serialize)]
struct BuildFileSummaryJson {
    setup_name: String,
    post_profile_id: String,
    filename: String,
    line_count: usize,
    max_line_chars: usize,
    tool_numbers: Vec<u32>,
    operation_names: Vec<String>,
    est_time_sec: f64,
}

#[derive(Debug, Serialize)]
struct BuildSummaryJson {
    project_name: String,
    files: Vec<BuildFileSummaryJson>,
    simulation: Vec<SetupSimReportJson>,
}

fn main() -> ExitCode {
    match run() {
        Ok(code) => code,
        Err(err) => {
            eprintln!("error: {err}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<ExitCode, Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    match cli.command {
        Commands::GenerateSampleStep {
            output,
            multi_feature,
            tilted,
            cam_torture,
            project_out,
        } => {
            let step = if cam_torture {
                sample_cam_torture_block_step()?
            } else if multi_feature || tilted {
                sample_multi_feature_bracket_step(tilted)?
            } else {
                sample_machined_plate_step()?
            };
            if let Some(parent) = output.parent()
                && !parent.as_os_str().is_empty()
            {
                std::fs::create_dir_all(parent)?;
            }
            std::fs::write(&output, &step)?;
            println!("Wrote sample machined STEP part to {}", output.display());

            if let Some(proj_path) = project_out {
                let doc = MonstertruckKernel.load_step_bytes(
                    "sample.step",
                    step.as_bytes(),
                    DEFAULT_TESS_CHORD_TOL_MM,
                )?;
                let sha = compute_sha256_hex(step.as_bytes());
                let proj = build_seeded_torture_project(&doc, &output, &proj_path, &sha);
                proj.save_to_path(&proj_path)?;
                println!(
                    "Wrote seeded CAM project ({} operations) to {}",
                    proj.setups[0].operations.len(),
                    proj_path.display()
                );
            }
            Ok(ExitCode::SUCCESS)
        }
        Commands::Inspect { path, units } => {
            let display_unit = match units.trim().to_ascii_lowercase().as_str() {
                "in" | "inch" | "inches" => DisplayUnit::Inch,
                _ => DisplayUnit::Millimeter,
            };
            let bytes = std::fs::read(&path)?;
            let name = path
                .file_name()
                .and_then(|s| s.to_str())
                .unwrap_or("model.step");

            let doc =
                MonstertruckKernel.load_step_bytes(name, &bytes, DEFAULT_TESS_CHORD_TOL_MM)?;
            let size = doc.aabb_mm.size();

            let mut planes = 0usize;
            let mut internal_holes = 0usize;
            let mut external_cyls = 0usize;
            let mut cones = 0usize;
            let mut other_faces = 0usize;
            for f in &doc.faces {
                match f.surface {
                    SurfaceKind::Plane { .. } => planes += 1,
                    SurfaceKind::Cylinder {
                        is_internal: true, ..
                    } => internal_holes += 1,
                    SurfaceKind::Cylinder {
                        is_internal: false, ..
                    } => external_cyls += 1,
                    SurfaceKind::Cone { .. } => cones += 1,
                    SurfaceKind::Torus { .. } | SurfaceKind::OtherNurbs => other_faces += 1,
                }
            }

            let mut line_edges = 0usize;
            let mut arc_edges = 0usize;
            let mut other_edges = 0usize;
            for e in &doc.edges {
                match e.curve {
                    CurveKind::Line { .. } => line_edges += 1,
                    CurveKind::CircleArc { .. } => arc_edges += 1,
                    CurveKind::EllipseArc | CurveKind::OtherNurbs => other_edges += 1,
                }
            }

            println!("=== ChipCutter STEP Inspection ===");
            println!("Source:         {}", doc.source_name);
            println!("STEP File Unit: {}", doc.detected_unit.label());
            println!("Solid Bodies:   {}", doc.bodies.len());
            println!(
                "Bounding Box:   X={}  Y={}  Z={}",
                display_unit.format_length(size.x),
                display_unit.format_length(size.y),
                display_unit.format_length(size.z),
            );
            println!(
                "Faces ({:>3}):    {} Planar, {} Cylindrical Hole Faces, {} External Cylinders, {} Cones, {} Freeform",
                doc.faces.len(),
                planes,
                internal_holes,
                external_cyls,
                cones,
                other_faces
            );
            println!("Loops:          {}", doc.loops.len());
            println!(
                "Edges ({:>3}):    {} Lines, {} Circular Arcs, {} Other",
                doc.edges.len(),
                line_edges,
                arc_edges,
                other_edges
            );
            println!(
                "Tessellation:   {} triangles, {} vertices",
                doc.render_mesh.indices.len() / 3,
                doc.render_mesh.vertices.len()
            );
            Ok(ExitCode::SUCCESS)
        }
        Commands::Validate { project, generate } => {
            let mut proj_doc = ProjectDocument::load_from_path(&project)?;
            let proj_dir = project.parent();
            let loaded_cad = load_primary_cad_for_project(&proj_doc, proj_dir).ok();

            let report = validate_project(&proj_doc, proj_dir, loaded_cad.as_ref());
            println!(
                "=== ChipCutter Project Validation: '{}' ===",
                proj_doc.project_name
            );
            println!(
                "Models: {} | Tools: {} | Setups: {}",
                proj_doc.models.len(),
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
        Commands::Simulate {
            project,
            resolution,
            report,
        } => {
            let sim_res_enum = parse_sim_resolution(&resolution);
            let mut proj_doc = ProjectDocument::load_from_path(&project)?;
            let proj_dir = project.parent();
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
                let resolved = setup.resolve_immutable(&cad_doc, proj_doc.display_units)?;
                let mut op_refs = Vec::new();
                for (o_idx, op) in setup.operations.iter().enumerate() {
                    if op.suppressed {
                        continue;
                    }
                    if let Some(tid) = &op.tool_id
                        && let Some(tool) = proj_doc.tools.iter().find(|t| &t.id == tid)
                        && let Some((_h, tp)) = cache.get_for_operation(&proj_doc, s_idx, o_idx, 1)
                    {
                        op_refs.push((op, tool, tp));
                    }
                }
                let sim_inputs: Vec<SimOperationInput<'_>> = op_refs
                    .iter()
                    .map(|(op, tool, tp)| SimOperationInput::from_operation(op, tool, tp.as_ref()))
                    .collect();

                let kin = resolve_post_profile(&setup.post_profile, proj_dir)
                    .map(|p| MachineKinematicsConfig {
                        rapid_rate_mm_min: p.rapid_rate_mm_min,
                        accel_mm_s2: p.accel_mm_s2,
                        junction_deviation_mm: 0.05,
                        tool_change_time_sec: p.tool_change_time_sec,
                    })
                    .unwrap_or_default();

                let sim = run_setup_simulation(
                    &cad_doc,
                    &resolved,
                    &setup.target_bodies,
                    &sim_inputs,
                    sim_res_enum,
                    kin,
                    SimVerificationConfig::default(),
                    None,
                )?;

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

                let crit = sim.critical_error_count();
                let warn = sim.warning_count();
                if crit > 0 {
                    any_critical = true;
                }
                setup_reports.push(SetupSimReportJson {
                    setup_id: setup.id.clone(),
                    setup_name: setup.name.clone(),
                    resolution: sim_res_enum,
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
                });
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
        Commands::Build {
            project,
            output_dir,
            setup: setup_filter,
            post: post_override,
            allow_warnings,
        } => {
            let mut proj_doc = ProjectDocument::load_from_path(&project)?;
            let proj_dir = project.parent();
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

            std::fs::create_dir_all(&output_dir)?;

            let proj_file_name = project
                .file_name()
                .and_then(|s| s.to_str())
                .unwrap_or(&proj_doc.project_name);
            let proj_stem = project
                .file_stem()
                .and_then(|s| s.to_str())
                .map(|s| s.trim_end_matches(".chip"))
                .unwrap_or("program");

            let mut emitted_summaries = Vec::new();
            let mut sim_summaries = Vec::new();
            let mut matched_any_setup = false;

            for (s_idx, setup) in proj_doc.setups.iter().enumerate() {
                if let Some(filter) = &setup_filter {
                    let f_low = filter.to_ascii_lowercase();
                    if setup.name.to_ascii_lowercase() != f_low
                        && setup.id.to_ascii_lowercase() != f_low
                    {
                        continue;
                    }
                }
                matched_any_setup = true;

                let profile_key = post_override.as_deref().unwrap_or(&setup.post_profile);
                let post_profile = resolve_post_profile(profile_key, proj_dir)?;
                let resolved = setup.resolve_immutable(&cad_doc, proj_doc.display_units)?;

                let mut op_refs = Vec::new();
                for (o_idx, op) in setup.operations.iter().enumerate() {
                    if op.suppressed {
                        continue;
                    }
                    if let Some(tid) = &op.tool_id
                        && let Some(tool) = proj_doc.tools.iter().find(|t| &t.id == tid)
                        && let Some((_h, tp)) = cache.get_for_operation(&proj_doc, s_idx, o_idx, 1)
                    {
                        op_refs.push((op, tool, tp));
                    }
                }

                // Mandatory pre-post physical heightmap crash & gouge gate (`06-simulation.md` §4)!
                let sim_inputs: Vec<SimOperationInput<'_>> = op_refs
                    .iter()
                    .map(|(op, tool, tp)| SimOperationInput::from_operation(op, tool, tp.as_ref()))
                    .collect();
                let kin = MachineKinematicsConfig {
                    rapid_rate_mm_min: post_profile.rapid_rate_mm_min,
                    accel_mm_s2: post_profile.accel_mm_s2,
                    junction_deviation_mm: 0.05,
                    tool_change_time_sec: post_profile.tool_change_time_sec,
                };
                let sim = run_setup_simulation(
                    &cad_doc,
                    &resolved,
                    &setup.target_bodies,
                    &sim_inputs,
                    SimResolution::Standard,
                    kin,
                    SimVerificationConfig::default(),
                    None,
                )?;

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
                let _ = SimSeverity::Warning;
                if sim.warning_count() > 0 && !allow_warnings {
                    println!(
                        "  [NOTE] Setup '{}' has {} simulation advisory warning(s) (continuing; pass --allow-warnings to acknowledge).",
                        setup.name,
                        sim.warning_count()
                    );
                }

                let post_ops: Vec<OperationPostInput<'_>> = op_refs
                    .iter()
                    .map(|(op, tool, tp)| OperationPostInput {
                        op,
                        tool,
                        toolpath: tp.as_ref(),
                    })
                    .collect();
                let post_input = SetupPostInput {
                    project_name: proj_file_name,
                    setup_name: &setup.name,
                    wcs_slot: setup.wcs_slot,
                    wcs_origin_description: describe_wcs_origin(&setup.wcs),
                    stock: resolved.stock,
                    clearance_z_wcs: resolved.clearance_z_wcs,
                    retract_z_wcs: resolved.retract_z_wcs,
                    operations: post_ops,
                };

                let setup_stem = if proj_doc.setups.len() > 1 {
                    format!("{proj_stem}_{}", setup.id)
                } else {
                    proj_stem.to_string()
                };
                let nc_files = post_process_setup(&post_input, &post_profile, &setup_stem)?;

                for nc in nc_files {
                    let out_path = output_dir.join(&nc.filename);
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

                let hms = format_cycle_time_hms(sim.timeline.total_time_sec);
                sim_summaries.push(SetupSimReportJson {
                    setup_id: setup.id.clone(),
                    setup_name: setup.name.clone(),
                    resolution: SimResolution::Standard,
                    total_time_sec: sim.timeline.total_time_sec,
                    total_time_hms: hms,
                    cutting_time_sec: sim.timeline.total_cutting_time_sec,
                    rapid_time_sec: sim.timeline.total_rapid_time_sec,
                    tool_change_time_sec: sim.timeline.total_tool_change_time_sec,
                    initial_stock_volume_mm3: sim.initial_stock_volume_mm3,
                    remaining_stock_volume_mm3: sim.remaining_stock_volume_mm3,
                    removed_volume_mm3: sim.removed_volume_mm3,
                    critical_error_count: sim.critical_error_count(),
                    warning_count: sim.warning_count(),
                    diagnostics: sim.diagnostics,
                });
            }

            if !matched_any_setup {
                eprintln!("No Setup matched filter {:?}.", setup_filter);
                return Ok(ExitCode::FAILURE);
            }

            let summary_path = output_dir.join("build_summary.json");
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
        Commands::Tools { action } => match action {
            ToolCommands::ImportFusion {
                input,
                output,
                name,
            } => {
                let report = import_fusion_library_path(&input, name.as_deref())?;
                report.library.save_to_path(&output)?;
                println!(
                    "Imported {} tool(s) and {} cutting preset(s) from '{}' -> '{}'.",
                    report.imported_tool_count,
                    report.imported_preset_count,
                    input.display(),
                    output.display()
                );
                for warn in &report.warnings {
                    println!("  [WARN] {warn}");
                }
                Ok(ExitCode::SUCCESS)
            }
        },
    }
}

fn parse_sim_resolution(raw: &str) -> SimResolution {
    match raw.trim().to_ascii_lowercase().as_str() {
        "fine" | "1024" => SimResolution::Fine,
        "ultra" | "2048" => SimResolution::Ultra,
        _ => SimResolution::Standard,
    }
}

fn load_primary_cad_for_project(
    proj: &ProjectDocument,
    proj_dir: Option<&Path>,
) -> Result<CadDocument, Box<dyn std::error::Error>> {
    let first = proj
        .models
        .first()
        .ok_or("project contains no CAD [[models]] entry")?;
    let step_path = resolve_model_path(proj_dir, &first.path);
    let bytes = std::fs::read(&step_path)
        .map_err(|e| format!("failed to read STEP model '{}': {e}", step_path.display()))?;
    let name = step_path
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("model.step");
    Ok(MonstertruckKernel.load_step_bytes(name, &bytes, DEFAULT_TESS_CHORD_TOL_MM)?)
}

fn build_seeded_torture_project(
    doc: &CadDocument,
    step_path: &Path,
    proj_path: &Path,
    sha256_hex: &str,
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("CAM Torture Block");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: portable_model_path(Some(proj_path), step_path),
        sha256: sha256_hex.to_string(),
    }];

    let starter = default_starter_tool_library();
    for t in &starter.tools {
        proj.ensure_tool_embedded(t);
    }

    let flat_6mm = proj
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                chip_tools::CutterGeometry::FlatEndmill { diameter_mm, .. }
                if (diameter_mm - 6.0).abs() < 0.05
            )
        })
        .cloned();
    let face_38mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, chip_tools::CutterGeometry::FaceMill { .. }))
        .cloned();
    let drill_6mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, chip_tools::CutterGeometry::TwistDrill { .. }))
        .cloned();
    let chamf_90 = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, chip_tools::CutterGeometry::ChamferMill { .. }))
        .cloned();

    if let Some(fm) = &face_38mm {
        let mut op = Operation::new("op_1_face", "Face Stock Top", OperationKind::Facing);
        op.tool_id = Some(fm.id.clone());
        op.preset_id = fm.presets.first().map(|p| p.id.clone());
        proj.setups[0].operations.push(op);
    }

    // Plateau Deck around Z=22 Boss Pad (`Z = 22 -> 20 mm`)
    if let Some(plat_face) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && (origin_mm.z - 20.0).abs() < 0.05 && f.area_mm2 > 1500.0
        )
    }) && let Some(bind) = doc.bind_face(plat_face.id)
        && let Some(em) = &flat_6mm
    {
        let mut op = Operation::new(
            "op_2_plateau",
            "Plateau Deck Around Boss",
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        proj.setups[0].operations.push(op);
    }

    // Open Shoulder Step (`Z = 14.0 mm`)
    if let Some(step_face) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && (origin_mm.z - 14.0).abs() < 0.05
        )
    }) && let Some(bind) = doc.bind_face(step_face.id)
        && let Some(em) = &flat_6mm
    {
        let mut op = Operation::new(
            "op_3_open_step",
            "Open Shoulder Step",
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        proj.setups[0].operations.push(op);
    }

    // Island Cavity Pocket (`Z = 12.0 mm` with Ø10mm raised island)
    if let Some(pkt_face) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && (origin_mm.z - 12.0).abs() < 0.05
        )
    }) && let Some(bind) = doc.bind_face(pkt_face.id)
        && let Some(em) = &flat_6mm
    {
        let mut op = Operation::new(
            "op_4_island_pocket",
            "Island Cavity Pocket",
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        proj.setups[0].operations.push(op);
    }

    // Helical Bore on Ø18mm Counterbore
    if let Some(r9_bore) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Cylinder {
                radius_mm,
                is_internal: true,
                ..
            } if (radius_mm - 9.0).abs() < 0.05
        )
    }) && let Some(bind) = doc.bind_face(r9_bore.id)
        && let Some(em) = &flat_6mm
    {
        let mut op = Operation::new(
            "op_5_helical_bore",
            "Ø18mm Helical Counterbore",
            OperationKind::DrillBore,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        if let StrategyParams::DrillBore(p) = op.strategy_mut() {
            p.mode = HoleMachiningMode::HelicalBore;
        }
        op.selections
            .push(GeometrySelectionItem::CylindricalHole { binding: bind });
        proj.setups[0].operations.push(op);
    }

    // Outer Contour + Triangular Holding Tabs (`T1` stays on `6mm` flat endmill before tool change!)
    if let Some(bot_face) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z < -0.99 && origin_mm.z.abs() < 0.05
        )
    }) && let Some(outer_lid) = bot_face.outer_loop
        && let Some(bind) = doc.bind_loop(outer_lid)
        && let Some(em) = &flat_6mm
    {
        let mut op = Operation::new(
            "op_6_outer_contour",
            "Outer Profile + Tabs",
            OperationKind::Contour2D,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.heights.top = HeightSpec::new(HeightReference::ModelTop, -2.0);
        op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
        op.selections.push(GeometrySelectionItem::ClosedLoop {
            binding: bind,
            reverse: false,
            side_flip: false,
        });
        if let StrategyParams::Contour2D(p) = op.strategy_mut() {
            p.tabs.enabled = true;
            p.tabs.profile = TabProfileKind::Triangular {
                ramp_angle_deg: 25.0,
            };
        }
        proj.setups[0].operations.push(op);
    }

    // 4x Ø6mm Shelf Through-Holes (`T6` Drill)
    if let Some(r3_seed) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Cylinder {
                radius_mm,
                is_internal: true,
                ..
            } if (radius_mm - 3.0).abs() < 0.05
        )
    }) && let Some(dr) = &drill_6mm
    {
        let mut op = Operation::new(
            "op_7_shelf_holes",
            "4x Shelf Through-Holes",
            OperationKind::DrillBore,
        );
        op.tool_id = Some(dr.id.clone());
        op.preset_id = dr.presets.first().map(|p| p.id.clone());
        for fid in find_same_diameter_holes(doc, r3_seed.id, 1e-3, 1e-3) {
            if let Some(b) = doc.bind_face(fid) {
                op.selections
                    .push(GeometrySelectionItem::CylindricalHole { binding: b });
            }
        }
        proj.setups[0].operations.push(op);
    }

    // Step Edge Chamfer (`T5` 90° Chamfer Mill)
    if let Some(step_edge) = doc.edges.iter().find(|e| {
        (e.aabb_mm.min.z - 20.0).abs() < 0.05
            && (e.aabb_mm.max.z - 20.0).abs() < 0.05
            && (e.aabb_mm.min.x - 76.0).abs() < 0.05
            && (e.aabb_mm.max.x - 76.0).abs() < 0.05
    }) && let Some(b) = doc.bind_edge(step_edge.id)
        && let Some(ch) = &chamf_90
    {
        let mut op = Operation::new(
            "op_8_step_chamfer",
            "Step Edge Chamfer",
            OperationKind::ChamferTrace,
        );
        op.tool_id = Some(ch.id.clone());
        op.preset_id = ch.presets.first().map(|p| p.id.clone());
        op.selections.push(GeometrySelectionItem::OpenChain {
            body_id: doc.bodies[0].id,
            start_edge: b.clone(),
            end_edge: b,
            resolved_edges: vec![chip_cad::OrientedEdgeUse {
                edge_id: step_edge.id,
                forward: true,
            }],
            reverse: false,
            side_flip: true,
        });
        if let StrategyParams::ChamferTrace(p) = op.strategy_mut() {
            p.mode = ChamferTraceMode::EdgeChamfer;
            p.chamfer_source = ChamferSourceMode::SharpEdge {
                width: ParamLength::from_mm(0.5),
            };
            p.tip_offset = ParamLength::from_mm(0.8);
        }
        proj.setups[0].operations.push(op);
    }

    proj
}
