use std::{path::PathBuf, process::ExitCode};

use chip_cad::{MeshImportOptions, MeshUnitKind, is_mesh_extension, load_mesh_bytes};
use chip_cam::{BoxAnchor27, StockConfig, WcsOriginMode};
use chip_math::ParamLength;
use chip_project::{ModelEntry, ProjectDocument, compute_sha256_hex, portable_model_path};
use chip_tools::default_starter_tool_library;

pub struct ImportMeshArgs {
    pub input: PathBuf,
    pub output: PathBuf,
    pub mesh_units: String,
    pub rot_x: f64,
    pub rot_y: f64,
    pub rot_z: f64,
    pub scale: f64,
    pub y_up_to_z_up: bool,
    pub center_xy: bool,
    pub drop_z0: bool,
}

pub(super) fn parse_mesh_unit_override(raw: &str) -> Result<Option<MeshUnitKind>, String> {
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
    let raw_mesh = load_mesh_bytes(
        source_name,
        &bytes,
        &MeshImportOptions {
            unit_override,
            ..MeshImportOptions::default()
        },
    )?;

    let mut xf = if args.y_up_to_z_up {
        chip_cad::ModelTransform::y_up_to_z_up()
    } else {
        chip_cad::ModelTransform::IDENTITY
    };
    if args.rot_x.abs() > 1e-9 || args.rot_y.abs() > 1e-9 || args.rot_z.abs() > 1e-9 {
        xf.rotate_deg = [
            xf.rotate_deg[0] + args.rot_x,
            xf.rotate_deg[1] + args.rot_y,
            xf.rotate_deg[2] + args.rot_z,
        ];
    }
    if (args.scale - 1.0).abs() > 1e-9 {
        xf = xf.with_uniform_scale(args.scale);
    }
    xf.validate()?;
    let mut mesh = raw_mesh.transformed(&xf);
    if args.center_xy {
        xf = xf.centered_xy_for_aabb(mesh.aabb_mm);
        mesh = raw_mesh.transformed(&xf);
    }
    if args.drop_z0 {
        xf = xf.dropped_bottom_to_z0_for_aabb(mesh.aabb_mm);
        mesh = raw_mesh.transformed(&xf);
    }

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
        transform: xf,
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
