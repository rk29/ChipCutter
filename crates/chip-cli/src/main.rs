use std::path::PathBuf;

use chip_cad::{CadKernel, CurveKind, MonstertruckKernel, SurfaceKind, sample_machined_plate_step};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit};
use clap::{Parser, Subcommand};

#[derive(Debug, Parser)]
#[command(
    name = "chipcutter",
    version,
    about = "ChipCutter headless CAM CLI (Milestone 1 STEP inspector)"
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
    /// Write ChipCutter's programmatic machined sample plate STEP file to disk.
    GenerateSampleStep {
        /// Output `.step` file path.
        output: PathBuf,
    },
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    match cli.command {
        Commands::GenerateSampleStep { output } => {
            let step = sample_machined_plate_step()?;
            if let Some(parent) = output.parent()
                && !parent.as_os_str().is_empty()
            {
                std::fs::create_dir_all(parent)?;
            }
            std::fs::write(&output, step)?;
            println!("Wrote sample machined STEP part to {}", output.display());
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
                "Faces ({:>3}):    {} Planar, {} Cylindrical Holes, {} External Cylinders, {} Cones, {} Freeform",
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
        }
    }
    Ok(())
}
