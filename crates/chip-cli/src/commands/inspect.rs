use std::{path::PathBuf, process::ExitCode};

use chip_cad::{
    CadKernel, CurveKind, MeshImportOptions, MeshUnitKind, MonstertruckKernel, SurfaceKind,
    is_mesh_extension, load_mesh_bytes,
};
use chip_math::{ArcSegment2D, DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit};
use chip_sketch::{
    DxfImportOptions, SvgImportOptions, VectorImportUnits, import_dxf_path, import_svg_path,
};

use super::import_mesh::parse_mesh_unit_override;

pub struct InspectArgs {
    pub path: PathBuf,
    pub units: String,
    pub mesh_units: String,
    pub svg_dpi: f64,
    pub join_tol: f64,
}

pub fn run_inspect(args: InspectArgs) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let InspectArgs {
        path,
        units,
        mesh_units,
        svg_dpi,
        join_tol,
    } = args;
    let display_unit = match units.trim().to_ascii_lowercase().as_str() {
        "in" | "inch" | "inches" => DisplayUnit::Inch,
        _ => DisplayUnit::Millimeter,
    };

    let ext = path
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase();
    if ext == "dxf" || ext == "svg" {
        return inspect_2d_vector(&path, &ext, display_unit, svg_dpi, join_tol);
    }
    if is_mesh_extension(&ext) {
        let unit_override = parse_mesh_unit_override(&mesh_units)?;
        return inspect_3d_mesh(&path, display_unit, unit_override);
    }

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

fn inspect_2d_vector(
    path: &std::path::Path,
    ext: &str,
    display_unit: DisplayUnit,
    svg_dpi: f64,
    join_tol: f64,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let (sketch, fmt_label, scale_info, layers) = if ext == "dxf" {
        let rep = import_dxf_path(
            path,
            DxfImportOptions {
                join_tolerance_mm: join_tol,
                units: VectorImportUnits::AutoDetect {
                    fallback: display_unit,
                },
                ..DxfImportOptions::default()
            },
        )?;
        (
            rep.sketch,
            "AutoCAD DXF",
            rep.detected_dxf_units,
            rep.available_layers,
        )
    } else {
        let rep = import_svg_path(
            path,
            SvgImportOptions {
                svg_dpi,
                join_tolerance_mm: join_tol,
                ..SvgImportOptions::default()
            },
        )?;
        let layers = rep.sketch.sub_layer_names();
        (rep.sketch, "W3C SVG", rep.scale_description, layers)
    };

    let aabb = sketch.exact_aabb();
    let sz = aabb.size();
    let mut arc_seg_count = 0usize;
    let mut line_seg_count = 0usize;
    for ent in &sketch.entities {
        for poly in ent.entity.to_arc_polylines(None) {
            for seg in poly.segments {
                match seg {
                    ArcSegment2D::Arc { .. } => arc_seg_count += 1,
                    ArcSegment2D::Line { .. } => line_seg_count += 1,
                }
            }
        }
    }
    let island_count: usize = sketch.regions.iter().map(|r| r.islands_cw.len()).sum();

    println!("=== ChipCutter 2D Vector Inspection ===");
    println!("Source:         {} ({fmt_label})", path.display());
    println!("Units / Scale:  {scale_info}");
    println!(
        "2D Bounds:      X={}  Y={}",
        display_unit.format_length(sz.x),
        display_unit.format_length(sz.y)
    );
    if !layers.is_empty() {
        println!("Layers ({:>2}):    {}", layers.len(), layers.join(", "));
    }
    println!(
        "Entities ({:>3}): {} Circles, {} Closed Loops, {} Open Chains ({line_seg_count} lines, {arc_seg_count} circular arcs)",
        sketch.entities.len(),
        sketch.circle_count(),
        sketch.closed_loop_count(),
        sketch.open_chain_count(),
    );
    println!(
        "Regions ({:>3}):  {} Planar Regions ({} interior island(s))",
        sketch.regions.len(),
        sketch.regions.len(),
        island_count
    );
    for d in &sketch.diagnostics {
        println!("  [DIAG] {}", d.message);
    }
    Ok(ExitCode::SUCCESS)
}

fn inspect_3d_mesh(
    path: &std::path::Path,
    display_unit: DisplayUnit,
    unit_override: Option<MeshUnitKind>,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let bytes = std::fs::read(path)?;
    let source_name = path
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("mesh.stl");
    let mesh = load_mesh_bytes(
        source_name,
        &bytes,
        &MeshImportOptions {
            unit_override,
            ..MeshImportOptions::default()
        },
    )?;
    let sz = mesh.aabb_mm.size();
    let d = &mesh.diagnostics;
    let detected_str = mesh
        .detected_unit
        .map_or("None (unitless)", MeshUnitKind::label);

    println!("=== ChipCutter 3D Mesh Inspection ===");
    println!(
        "Source:         {} ({})",
        path.display(),
        mesh.format.label()
    );
    println!("Mesh Name:      {}", mesh.name);
    println!("Detected Unit:  {detected_str}");
    println!("Applied Unit:   {}", mesh.applied_unit.label());
    println!(
        "Bounding Box:   X={}  Y={}  Z={}",
        display_unit.format_length(sz.x),
        display_unit.format_length(sz.y),
        display_unit.format_length(sz.z),
    );
    println!(
        "Topology:       {} triangles ({} raw, {} degenerate removed), {} welded vertices",
        d.valid_triangle_count,
        d.raw_triangle_count,
        d.degenerate_triangles_removed,
        d.welded_vertex_count
    );
    println!(
        "Manifold Check: {} (open boundary edges: {}, non-manifold edges: {})",
        if d.is_watertight {
            "Watertight"
        } else {
            "Not Watertight"
        },
        d.open_boundary_edges,
        d.non_manifold_edges
    );
    println!("Signed Volume:  {:.2} mm³", d.signed_volume_mm3);
    Ok(ExitCode::SUCCESS)
}
