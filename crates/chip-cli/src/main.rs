use std::{path::PathBuf, process::ExitCode};

use chip_cad::{
    CadKernel, CurveKind, MonstertruckKernel, SurfaceKind, sample_machined_plate_step,
    sample_multi_feature_bracket_step,
};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit};
use chip_project::{DiagnosticSeverity, ProjectDocument, resolve_model_path, validate_project};
use chip_tools::import_fusion_library_path;
use clap::{Parser, Subcommand};

#[derive(Debug, Parser)]
#[command(
    name = "chipcutter",
    version,
    about = "ChipCutter headless CAM CLI (STEP inspection, project validation, & Fusion 360 tool import)"
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
    /// Write ChipCutter's programmatic machined sample STEP file to disk.
    GenerateSampleStep {
        /// Output `.step` file path.
        output: PathBuf,
        /// Generate the Milestone 2 multi-feature bracket (with blind pocket/hole + 4 mounting holes).
        #[arg(long)]
        multi_feature: bool,
        /// Tilt the generated multi-feature bracket by 30° X / 25° Z to test WCS alignment.
        #[arg(long)]
        tilted: bool,
    },
    /// Validate a `.chip.toml` project's STEP hashes, tools, Setups, WCS, selections, and Z heights.
    Validate {
        /// Path to `.chip.toml` project file.
        project: PathBuf,
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
        } => {
            let step = if multi_feature || tilted {
                sample_multi_feature_bracket_step(tilted)?
            } else {
                sample_machined_plate_step()?
            };
            if let Some(parent) = output.parent()
                && !parent.as_os_str().is_empty()
            {
                std::fs::create_dir_all(parent)?;
            }
            std::fs::write(&output, step)?;
            println!("Wrote sample machined STEP part to {}", output.display());
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
        Commands::Validate { project } => {
            let proj_doc = ProjectDocument::load_from_path(&project)?;
            let proj_dir = project.parent();

            let mut loaded_cad = None;
            if let Some(first_model) = proj_doc.models.first() {
                let step_path = resolve_model_path(proj_dir, &first_model.path);
                if let Ok(bytes) = std::fs::read(&step_path) {
                    let name = step_path
                        .file_name()
                        .and_then(|s| s.to_str())
                        .unwrap_or("model.step");
                    if let Ok(doc) =
                        MonstertruckKernel.load_step_bytes(name, &bytes, DEFAULT_TESS_CHORD_TOL_MM)
                    {
                        loaded_cad = Some(doc);
                    }
                }
            }

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
            if report.is_valid() {
                println!(
                    "Validation PASSED ({} warning(s), 0 errors).",
                    report.warning_count()
                );
                Ok(ExitCode::SUCCESS)
            } else {
                eprintln!(
                    "Validation FAILED ({} error(s), {} warning(s)).",
                    report.error_count(),
                    report.warning_count()
                );
                Ok(ExitCode::FAILURE)
            }
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
