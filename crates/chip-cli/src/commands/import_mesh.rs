use std::{path::PathBuf, process::ExitCode};

use chip_cad::{
    MeshImportOptions, MeshUnitKind, ModelTransform, is_mesh_extension, load_mesh_bytes,
};
use chip_project::{ModelEntry, ProjectDocument, compute_sha256_hex, portable_model_path};
use chip_tools::default_starter_tool_library;
use clap::Args;

#[derive(Debug, Args)]
pub struct ImportMeshArgs {
    /// Input `.stl`, `.obj`, or `.3mf` mesh file.
    pub input: PathBuf,
    /// Destination `.chip.toml` project file.
    #[arg(short, long)]
    pub output: PathBuf,
    /// Mesh unit override (`auto`, `mm`, `in`, `cm`, `m`, `ft`, `um`).
    #[arg(long, default_value = "auto")]
    pub mesh_units: String,
    /// Rotate imported mesh around X axis (degrees).
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    pub rot_x: f64,
    /// Rotate imported mesh around Y axis (degrees).
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    pub rot_y: f64,
    /// Rotate imported mesh around Z axis (degrees).
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    pub rot_z: f64,
    /// Uniform scale factor applied to imported mesh (`1.0` default).
    #[arg(long, default_value_t = 1.0)]
    pub scale: f64,
    /// Convert Y-up mesh to Z-up (`+90°` around X).
    #[arg(long)]
    pub y_up_to_z_up: bool,
    /// Center transformed mesh XY bounds on `(0, 0)`.
    #[arg(long)]
    pub center_xy: bool,
    /// Drop bottom of transformed mesh to `Z = 0`.
    #[arg(long)]
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

    let (xf, _) = ModelTransform::compose_for_mesh_import(
        &raw_mesh.vertices_mm,
        [args.rot_x, args.rot_y, args.rot_z],
        args.scale,
        args.y_up_to_z_up,
        args.center_xy,
        args.drop_z0,
    );
    xf.validate()?;
    let mesh = raw_mesh.transformed(&xf);

    let stem = args
        .input
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("Mesh Job");
    let rel_path = portable_model_path(Some(&args.output), &args.input);
    let starter = default_starter_tool_library();
    let proj = ProjectDocument::new_from_mesh(
        stem,
        ModelEntry {
            id: "model_1".to_string(),
            path: rel_path,
            sha256: compute_sha256_hex(&bytes),
            role: Default::default(),
            unit_override,
            transform: xf,
        },
        &starter.tools,
    );

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
