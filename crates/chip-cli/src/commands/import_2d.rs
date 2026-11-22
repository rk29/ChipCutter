use std::{path::PathBuf, process::ExitCode};

use chip_cam::{BoxAnchor27, StockConfig, WcsOriginMode};
use chip_math::{DisplayUnit, ParamLength};
use chip_project::{ProjectDocument, portable_model_path};
use chip_sketch::{
    DxfImportOptions, SketchLayer, SvgImportOptions, VectorImportUnits, import_dxf_path,
    import_svg_path,
};
use chip_tools::default_starter_tool_library;

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
