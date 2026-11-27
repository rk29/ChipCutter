use std::{path::PathBuf, process::ExitCode};

use chip_project::ProjectDocument;
use clap::Args;

#[derive(Debug, Args)]
pub struct TransformModelArgs {
    /// Path to `.chip.toml` project file.
    pub project: PathBuf,
    /// Optional output `.chip.toml` path (overwrites `project` in place if omitted).
    #[arg(short, long)]
    pub output: Option<PathBuf>,
    /// Set X rotation angle (degrees).
    #[arg(long, allow_hyphen_values = true)]
    pub rot_x: Option<f64>,
    /// Set Y rotation angle (degrees).
    #[arg(long, allow_hyphen_values = true)]
    pub rot_y: Option<f64>,
    /// Set Z rotation angle (degrees).
    #[arg(long, allow_hyphen_values = true)]
    pub rot_z: Option<f64>,
    /// Set uniform scale factor across X, Y, and Z.
    #[arg(long)]
    pub scale: Option<f64>,
    /// Set X scale factor.
    #[arg(long)]
    pub scale_x: Option<f64>,
    /// Set Y scale factor.
    #[arg(long)]
    pub scale_y: Option<f64>,
    /// Set Z scale factor.
    #[arg(long)]
    pub scale_z: Option<f64>,
    /// Uniformly scale so transformed X span equals `<mm>`.
    #[arg(long)]
    pub fit_size_x: Option<f64>,
    /// Uniformly scale so transformed Y span equals `<mm>`.
    #[arg(long)]
    pub fit_size_y: Option<f64>,
    /// Uniformly scale so transformed Z span equals `<mm>`.
    #[arg(long)]
    pub fit_size_z: Option<f64>,
    /// Set X translation (`mm`).
    #[arg(long, allow_hyphen_values = true)]
    pub move_x: Option<f64>,
    /// Set Y translation (`mm`).
    #[arg(long, allow_hyphen_values = true)]
    pub move_y: Option<f64>,
    /// Set Z translation (`mm`).
    #[arg(long, allow_hyphen_values = true)]
    pub move_z: Option<f64>,
    /// Apply `+90° X` (`Y-Up -> Z-Up`) world rotation around model center.
    #[arg(long)]
    pub y_up_to_z_up: bool,
    /// Center transformed XY bounds on `(0, 0)`.
    #[arg(long)]
    pub center_xy: bool,
    /// Drop bottom (`min Z`) to `Z = 0`.
    #[arg(long)]
    pub drop_z0: bool,
    /// Align top (`max Z`) to `Z = 0`.
    #[arg(long)]
    pub align_top_z0: bool,
    /// Reset transform to identity before applying any other flags.
    #[arg(long)]
    pub reset: bool,
}

pub fn run_transform_model(
    args: TransformModelArgs,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let mut proj = ProjectDocument::load_from_path(&args.project)?;
    let part_idx = proj
        .primary_part_model_index()
        .ok_or("project contains no target part [[models]] entry to transform")?;
    let first = proj.models[part_idx].clone();
    let model_path = chip_project::resolve_model_path(args.project.parent(), &first.path);
    let raw_doc = chip_cad::load_model_entry_cad_doc(&model_path, first.unit_override)?;

    let before_doc = raw_doc.transformed(&first.transform);
    let before_sz = before_doc.aabb_mm.size();

    let mut xf = if args.reset {
        chip_cad::ModelTransform::IDENTITY
    } else {
        first.transform
    };

    if args.y_up_to_z_up {
        let cur_center = raw_doc.transformed(&xf).aabb_mm.center();
        xf = xf.rotated_world_axis_deg(chip_math::DVec3::X, 90.0, cur_center);
    }
    if let Some(rx) = args.rot_x {
        xf.rotate_deg[0] = rx;
    }
    if let Some(ry) = args.rot_y {
        xf.rotate_deg[1] = ry;
    }
    if let Some(rz) = args.rot_z {
        xf.rotate_deg[2] = rz;
    }
    if let Some(s) = args.scale {
        xf = xf.with_uniform_scale(s);
    }
    if let Some(sx) = args.scale_x {
        xf.scale[0] = sx;
    }
    if let Some(sy) = args.scale_y {
        xf.scale[1] = sy;
    }
    if let Some(sz) = args.scale_z {
        xf.scale[2] = sz;
    }
    if let Some(tx) = args.move_x {
        xf.translate_mm[0] = tx;
    }
    if let Some(ty) = args.move_y {
        xf.translate_mm[1] = ty;
    }
    if let Some(tz) = args.move_z {
        xf.translate_mm[2] = tz;
    }
    xf.validate()?;

    let mut cur_doc = raw_doc.transformed(&xf);
    if let Some(target_x) = args.fit_size_x {
        xf = xf.scaled_uniformly_to_fit_axis(cur_doc.aabb_mm, 0, target_x);
        cur_doc = raw_doc.transformed(&xf);
    } else if let Some(target_y) = args.fit_size_y {
        xf = xf.scaled_uniformly_to_fit_axis(cur_doc.aabb_mm, 1, target_y);
        cur_doc = raw_doc.transformed(&xf);
    } else if let Some(target_z) = args.fit_size_z {
        xf = xf.scaled_uniformly_to_fit_axis(cur_doc.aabb_mm, 2, target_z);
        cur_doc = raw_doc.transformed(&xf);
    }
    if args.center_xy {
        xf = xf.centered_xy_for_aabb(cur_doc.aabb_mm);
        cur_doc = raw_doc.transformed(&xf);
    }
    if args.drop_z0 {
        xf = xf.dropped_bottom_to_z0_for_aabb(cur_doc.aabb_mm);
        cur_doc = raw_doc.transformed(&xf);
    } else if args.align_top_z0 {
        xf = xf.aligned_top_to_z0_for_aabb(cur_doc.aabb_mm);
        cur_doc = raw_doc.transformed(&xf);
    }
    xf.validate()?;

    proj.apply_model_transform(part_idx, xf);

    let dest = args.output.unwrap_or(args.project);
    proj.save_to_path(&dest)?;

    let after_sz = cur_doc.aabb_mm.size();
    println!("Transformed model '{}' in {}", first.id, dest.display());
    println!(
        "  Before: {:.2} × {:.2} × {:.2} mm (Z [{:.2} .. {:.2}])",
        before_sz.x, before_sz.y, before_sz.z, before_doc.aabb_mm.min.z, before_doc.aabb_mm.max.z
    );
    println!(
        "  After:  {:.2} × {:.2} × {:.2} mm (Z [{:.2} .. {:.2}])",
        after_sz.x, after_sz.y, after_sz.z, cur_doc.aabb_mm.min.z, cur_doc.aabb_mm.max.z
    );
    println!(
        "  Transform: rot=({:.2}°, {:.2}°, {:.2}°)  scale=({:.4}, {:.4}, {:.4})  move=({:.3}, {:.3}, {:.3}) mm",
        xf.rotate_deg[0],
        xf.rotate_deg[1],
        xf.rotate_deg[2],
        xf.scale[0],
        xf.scale[1],
        xf.scale[2],
        xf.translate_mm[0],
        xf.translate_mm[1],
        xf.translate_mm[2],
    );
    Ok(ExitCode::SUCCESS)
}
