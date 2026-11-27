use std::{path::PathBuf, process::ExitCode};

use chip_math::DisplayUnit;
use chip_project::{ProjectDocument, portable_model_path};
use chip_sketch::{
    DxfImportOptions, SketchLayer, SvgImportOptions, VectorImportUnits, import_dxf_path,
    import_svg_path,
};
use chip_tools::default_starter_tool_library;
use clap::Args;

#[derive(Debug, Args)]
pub struct Import2dArgs {
    /// Input `.dxf` or `.svg` vector file.
    pub input: PathBuf,
    /// Destination `.chip.toml` project file.
    #[arg(short, long)]
    pub output: PathBuf,
    /// Optional existing `.chip.toml` project to append the `SketchLayer` onto.
    #[arg(long = "project")]
    pub existing_project: Option<PathBuf>,
    /// Vector unit override (`auto`, `auto_in`, `mm`, `in`, `cm`).
    #[arg(long, default_value = "auto")]
    pub units: String,
    /// SVG unitless pixel DPI (`96` CSS/Inkscape, `72` Illustrator, `90` legacy Inkscape).
    #[arg(long, default_value_t = 96.0)]
    pub svg_dpi: f64,
    /// Endpoint gap-healing join tolerance in millimeters (`0.05` default).
    #[arg(long, default_value_t = 0.05)]
    pub join_tol: f64,
    /// Sheet Z thickness (`mm`) when initializing a new Pure-2D project.
    #[arg(long = "thickness", default_value_t = 6.0)]
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
            let starter = default_starter_tool_library();
            ProjectDocument::new_pure_2d_sheet(
                stem,
                args.sheet_thickness_mm,
                DisplayUnit::Millimeter,
                &starter.tools,
            )
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
