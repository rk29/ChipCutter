use std::{path::PathBuf, process::ExitCode};

use chip_cad::{
    CadKernel, CurveKind, MonstertruckKernel, SurfaceKind, sample_cam_torture_block_step,
    sample_machined_plate_step, sample_multi_feature_bracket_step, sample_revised_part_pair_step,
};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit};
use chip_project::{build_cam_torture_project, compute_sha256_hex, portable_model_path};
use chip_tools::default_starter_tool_library;

pub struct GenerateSampleArgs {
    pub output: PathBuf,
    pub multi_feature: bool,
    pub tilted: bool,
    pub cam_torture: bool,
    pub revision_v1: bool,
    pub revision_v2: bool,
    pub project_out: Option<PathBuf>,
}

pub fn run_generate_sample_step(
    args: GenerateSampleArgs,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let step = if args.revision_v1 {
        sample_revised_part_pair_step()?.0
    } else if args.revision_v2 {
        sample_revised_part_pair_step()?.1
    } else if args.cam_torture {
        sample_cam_torture_block_step()?
    } else if args.multi_feature || args.tilted {
        sample_multi_feature_bracket_step(args.tilted)?
    } else {
        sample_machined_plate_step()?
    };

    if let Some(parent) = args.output.parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&args.output, &step)?;
    println!(
        "Wrote sample machined STEP part to {}",
        args.output.display()
    );

    if let Some(proj_path) = args.project_out {
        let doc = MonstertruckKernel.load_step_bytes(
            "sample.step",
            step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )?;
        let sha = compute_sha256_hex(step.as_bytes());
        let starter = default_starter_tool_library();
        let rel_path = portable_model_path(Some(&proj_path), &args.output);
        let proj = build_cam_torture_project(&doc, rel_path, &sha, &starter.tools);
        proj.save_to_path(&proj_path)?;
        println!(
            "Wrote seeded CAM project ({} operations) to {}",
            proj.setups[0].operations.len(),
            proj_path.display()
        );
    }

    Ok(ExitCode::SUCCESS)
}

pub fn run_inspect(path: PathBuf, units: String) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let display_unit = match units.trim().to_ascii_lowercase().as_str() {
        "in" | "inch" | "inches" => DisplayUnit::Inch,
        _ => DisplayUnit::Millimeter,
    };
    let bytes = std::fs::read(&path)?;
    let name = path
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("model.step");

    let doc = MonstertruckKernel.load_step_bytes(name, &bytes, DEFAULT_TESS_CHORD_TOL_MM)?;
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
