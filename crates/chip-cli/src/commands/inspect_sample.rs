use std::{path::PathBuf, process::ExitCode};

use chip_cad::{
    CadKernel, CurveKind, MeshImportOptions, MeshUnitKind, MonstertruckKernel, SurfaceKind,
    is_mesh_extension, load_mesh_bytes, sample_cam_torture_block_step, sample_machined_plate_step,
    sample_multi_feature_bracket_step, sample_revised_part_pair_step,
    tutorial_adaptive_pocket_step, tutorial_chamfer_engrave_block_step,
    tutorial_contour_plate_step, tutorial_drill_bore_plate_step, tutorial_facing_billet_step,
    tutorial_pocket_block_step,
};
use chip_cam::{BoxAnchor27, StockConfig, WcsOriginMode};
use chip_math::{ArcSegment2D, DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit, ParamLength};
use chip_project::{
    ModelEntry, ProjectDocument, build_cam_torture_project, build_tutorial_adaptive_rest_project,
    build_tutorial_chamfer_trace_project, build_tutorial_contour_project,
    build_tutorial_drill_bore_project, build_tutorial_facing_project,
    build_tutorial_pocket_project, build_tutorial_vcarve_inlay_project, compute_sha256_hex,
    portable_model_path,
};
use chip_sketch::{
    DxfImportOptions, SketchLayer, SvgImportOptions, VectorImportUnits, import_dxf_path,
    import_svg_path,
};
use chip_tools::default_starter_tool_library;

pub struct GenerateSampleArgs {
    pub output: PathBuf,
    pub multi_feature: bool,
    pub tilted: bool,
    pub cam_torture: bool,
    pub revision_v1: bool,
    pub revision_v2: bool,
    pub tutorial: Option<String>,
    pub project_out: Option<PathBuf>,
    pub empty_project: bool,
}

pub fn run_generate_sample_step(
    args: GenerateSampleArgs,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let tutorial_slug = args
        .tutorial
        .as_deref()
        .map(|s| s.trim().to_ascii_lowercase());

    if matches!(
        tutorial_slug.as_deref(),
        Some("07-vcarve-medial" | "07-vcarve-inlay" | "vcarve-medial" | "vcarve-inlay")
    ) {
        let medial_only = matches!(
            tutorial_slug.as_deref(),
            Some("07-vcarve-medial" | "vcarve-medial")
        );
        let starter = default_starter_tool_library();
        let proj = build_tutorial_vcarve_inlay_project(&starter.tools, medial_only);
        let dest = args.project_out.unwrap_or(args.output);
        if let Some(parent) = dest.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        proj.save_to_path(&dest)?;
        println!(
            "Wrote Pure-2D V-Carve tutorial project ({} setup(s)) to {}",
            proj.setups.len(),
            dest.display()
        );
        return Ok(ExitCode::SUCCESS);
    }

    let step = if let Some(slug) = tutorial_slug.as_deref() {
        match slug {
            "01-facing" | "facing" => tutorial_facing_billet_step()?,
            "02-contour" | "02-2d-contour-tabs" | "contour" => tutorial_contour_plate_step()?,
            "03-pocket" | "03-2d-pocket" | "pocket" => tutorial_pocket_block_step()?,
            "04-drill-bore" | "04-drilling-helical-bore" | "drill-bore" => {
                tutorial_drill_bore_plate_step()?
            }
            "05-adaptive-rest" | "05-adaptive-rest-machining" | "adaptive-rest" => {
                tutorial_adaptive_pocket_step()?
            }
            "06-chamfer-trace" | "06-chamfer-curve-trace" | "chamfer-trace" => {
                tutorial_chamfer_engrave_block_step()?
            }
            other => {
                return Err(format!(
                    "unknown --tutorial '{other}'; expected one of: 01-facing, 02-contour, 03-pocket, 04-drill-bore, 05-adaptive-rest, 06-chamfer-trace, 07-vcarve-medial, 07-vcarve-inlay"
                )
                .into());
            }
        }
    } else if args.revision_v1 {
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
        let proj = match tutorial_slug.as_deref() {
            Some("01-facing" | "facing") => {
                build_tutorial_facing_project(rel_path, &sha, &starter.tools, args.empty_project)
            }
            Some("02-contour" | "02-2d-contour-tabs" | "contour") => {
                build_tutorial_contour_project(
                    &doc,
                    rel_path,
                    &sha,
                    &starter.tools,
                    args.empty_project,
                )
            }
            Some("03-pocket" | "03-2d-pocket" | "pocket") => build_tutorial_pocket_project(
                &doc,
                rel_path,
                &sha,
                &starter.tools,
                args.empty_project,
            ),
            Some("04-drill-bore" | "04-drilling-helical-bore" | "drill-bore") => {
                build_tutorial_drill_bore_project(
                    &doc,
                    rel_path,
                    &sha,
                    &starter.tools,
                    args.empty_project,
                )
            }
            Some("05-adaptive-rest" | "05-adaptive-rest-machining" | "adaptive-rest") => {
                build_tutorial_adaptive_rest_project(&doc, rel_path, &sha, args.empty_project)
            }
            Some("06-chamfer-trace" | "06-chamfer-curve-trace" | "chamfer-trace") => {
                build_tutorial_chamfer_trace_project(
                    &doc,
                    rel_path,
                    &sha,
                    &starter.tools,
                    args.empty_project,
                )
            }
            _ => build_cam_torture_project(&doc, rel_path, &sha, &starter.tools),
        };
        if let Some(parent) = proj_path.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        proj.save_to_path(&proj_path)?;
        println!(
            "Wrote seeded CAM project ({} operations) to {}",
            proj.setups[0].operations.len(),
            proj_path.display()
        );
    }

    Ok(ExitCode::SUCCESS)
}

pub struct InspectArgs {
    pub path: PathBuf,
    pub units: String,
    pub mesh_units: String,
    pub svg_dpi: f64,
    pub join_tol: f64,
}

pub struct ImportMeshArgs {
    pub input: PathBuf,
    pub output: PathBuf,
    pub mesh_units: String,
}

fn parse_mesh_unit_override(raw: &str) -> Result<Option<MeshUnitKind>, String> {
    let trimmed = raw.trim();
    if trimmed.is_empty() || trimmed.eq_ignore_ascii_case("auto") {
        return Ok(None);
    }
    MeshUnitKind::from_str_loose(trimmed)
        .map(Some)
        .ok_or_else(|| {
            format!("invalid --mesh-units '{raw}' (expected auto, mm, in, cm, m, ft, or um)")
        })
}

pub fn run_import_mesh(args: ImportMeshArgs) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let ext = args
        .input
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or_default();
    if !is_mesh_extension(ext) {
        return Err(format!(
            "unsupported 3D mesh file extension '{}' (expected .stl, .obj, or .3mf)",
            args.input.display()
        )
        .into());
    }
    let unit_override = parse_mesh_unit_override(&args.mesh_units)?;
    let bytes = std::fs::read(&args.input)?;
    let source_name = args
        .input
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

    let stem = args
        .input
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("Mesh Job");
    let mut proj = ProjectDocument::new(stem);
    let rel_path = portable_model_path(Some(&args.output), &args.input);
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: rel_path,
        sha256: compute_sha256_hex(&bytes),
        unit_override,
    }];
    proj.setups[0].wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    proj.setups[0].stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(2.0),
        side_neg_x: ParamLength::from_mm(2.0),
        side_pos_y: ParamLength::from_mm(2.0),
        side_neg_y: ParamLength::from_mm(2.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(12.0),
        round_increment_mm: None,
    };
    let starter = default_starter_tool_library();
    for t in &starter.tools {
        proj.ensure_tool_embedded(t);
    }

    proj.save_to_path(&args.output)?;
    let sz = mesh.aabb_mm.size();
    println!(
        "Imported {} '{}' ({}, {} triangles, {} vertices, {:.2} × {:.2} × {:.2} mm, {}).",
        mesh.format.label(),
        args.input.display(),
        mesh.applied_unit.short_label(),
        mesh.diagnostics.valid_triangle_count,
        mesh.diagnostics.welded_vertex_count,
        sz.x,
        sz.y,
        sz.z,
        if mesh.diagnostics.is_watertight {
            "Watertight"
        } else {
            "Open/Non-Manifold"
        }
    );
    println!(
        "Saved schema v{} project to {}",
        proj.schema_version,
        args.output.display()
    );
    Ok(ExitCode::SUCCESS)
}

pub struct Import2dArgs {
    pub input: PathBuf,
    pub output: PathBuf,
    pub existing_project: Option<PathBuf>,
    pub units: String,
    pub svg_dpi: f64,
    pub join_tol: f64,
    pub sheet_thickness_mm: f64,
}

fn parse_vector_units(raw: &str) -> VectorImportUnits {
    match raw.trim().to_ascii_lowercase().as_str() {
        "mm" | "millimeter" | "millimeters" => VectorImportUnits::Millimeters,
        "in" | "inch" | "inches" => VectorImportUnits::Inches,
        "cm" | "centimeter" | "centimeters" => VectorImportUnits::Centimeters,
        "auto_in" => VectorImportUnits::AutoDetect {
            fallback: DisplayUnit::Inch,
        },
        _ => VectorImportUnits::AutoDetect {
            fallback: DisplayUnit::Millimeter,
        },
    }
}

pub fn run_import_2d(args: Import2dArgs) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let ext = args
        .input
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase();
    let mut proj = match &args.existing_project {
        Some(existing) => ProjectDocument::load_from_path(existing)?,
        None => {
            let stem = args
                .input
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("Pure 2D Job");
            let mut fresh = ProjectDocument::new(stem);
            fresh.setups[0].wcs.origin = WcsOriginMode::StockBoxPoint {
                anchor: BoxAnchor27::TOP_FRONT_LEFT,
            };
            fresh.setups[0].stock = StockConfig::RelativeBox {
                side_pos_x: ParamLength::from_mm(2.0),
                side_neg_x: ParamLength::from_mm(2.0),
                side_pos_y: ParamLength::from_mm(2.0),
                side_neg_y: ParamLength::from_mm(2.0),
                symmetric_xy: true,
                top_offset_z: ParamLength::from_mm(0.0),
                bottom_offset_z: ParamLength::from_mm(0.0),
                sheet_thickness_z: ParamLength::from_mm(args.sheet_thickness_mm.max(0.5)),
                round_increment_mm: None,
            };
            let starter = default_starter_tool_library();
            for t in &starter.tools {
                fresh.ensure_tool_embedded(t);
            }
            fresh
        }
    };

    let setup_id = proj
        .setups
        .first()
        .map_or_else(|| "setup_1".to_string(), |s| s.id.clone());
    let next_id = format!("sketch_{}", proj.sketches.len() + 1);
    let rel_vec = portable_model_path(Some(&args.output), &args.input);
    let v_units = parse_vector_units(&args.units);

    let sketch: SketchLayer = match ext.as_str() {
        "dxf" => {
            let rep = import_dxf_path(
                &args.input,
                DxfImportOptions {
                    sketch_id: next_id,
                    setup_id,
                    source_path: Some(rel_vec),
                    units: v_units,
                    join_tolerance_mm: args.join_tol,
                    ..DxfImportOptions::default()
                },
            )?;
            rep.sketch
        }
        "svg" => {
            let units_override = match args.units.trim().to_ascii_lowercase().as_str() {
                "mm" | "in" | "cm" => Some(v_units),
                _ => None,
            };
            let rep = import_svg_path(
                &args.input,
                SvgImportOptions {
                    sketch_id: next_id,
                    setup_id,
                    source_path: Some(rel_vec),
                    svg_dpi: args.svg_dpi,
                    units_override,
                    join_tolerance_mm: args.join_tol,
                    ..SvgImportOptions::default()
                },
            )?;
            rep.sketch
        }
        _ => {
            return Err(format!(
                "unsupported 2D vector file extension '{}' (expected .dxf or .svg)",
                args.input.display()
            )
            .into());
        }
    };

    println!(
        "Imported '{}' -> Sketch '{}' ({} entities, {} planar regions, {} circle(s), {} open chain(s)).",
        args.input.display(),
        sketch.name,
        sketch.entities.len(),
        sketch.regions.len(),
        sketch.circle_count(),
        sketch.open_chain_count()
    );
    proj.sketches.push(sketch);
    proj.save_to_path(&args.output)?;
    println!("Saved project to {}", args.output.display());
    Ok(ExitCode::SUCCESS)
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
