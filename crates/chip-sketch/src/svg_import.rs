//! W3C **SVG** (`usvg` + `kurbo`) vector importer with physical unit (`mm`/`in`/`cm`/`pt`) and
//! configurable `DPI` scaling, `+Y up` coordinate normalization, `<g transform=...>` flattening,
//! and circular biarc fitting (`11-sketches-vectors.md`).

use std::path::Path;

use chip_math::{DEFAULT_ARC_FIT_TOL_MM, DVec2, MM_PER_INCH};
use kurbo::{BezPath, Point as KPoint};
use thiserror::Error;
use usvg::{Group, Node, Options as UsvgOptions, Tree};

use crate::{
    curve_fit::fit_bezpath_to_arc_polylines,
    heal_chain::{RawImportedItem, heal_and_classify_sketch_geometry},
    model::{
        SketchAnchor, SketchLayer, SketchPlaneRef, VectorAssetFormat, VectorImportUnits,
        VectorSourceAsset, compute_sketch_sha256_hex,
    },
};

/// Errors encountered while reading or converting an `.svg` document.
#[derive(Debug, Error)]
pub enum SvgImportError {
    #[error("I/O error reading SVG '{path}': {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("SVG parse error: {0}")]
    SvgParse(String),
}

/// Configuration for importing an `.svg` file into a [`SketchLayer`].
#[derive(Debug, Clone)]
pub struct SvgImportOptions {
    pub sketch_id: String,
    pub sketch_name: String,
    pub setup_id: String,
    pub source_path: Option<String>,
    /// Dots-per-inch used to convert unitless SVG pixels (`px`) to millimeters (`25.4 / svg_dpi`).
    /// Standard CSS / Inkscape >= 0.92 is `96.0`; Adobe Illustrator is `72.0`; legacy Inkscape is `90.0`.
    pub svg_dpi: f64,
    pub units_override: Option<VectorImportUnits>,
    pub join_tolerance_mm: f64,
    pub arc_fit_tolerance_mm: f64,
}

impl Default for SvgImportOptions {
    fn default() -> Self {
        Self {
            sketch_id: "sketch_1".to_string(),
            sketch_name: "Imported SVG".to_string(),
            setup_id: "setup_1".to_string(),
            source_path: None,
            svg_dpi: 96.0,
            units_override: None,
            join_tolerance_mm: 0.05,
            arc_fit_tolerance_mm: DEFAULT_ARC_FIT_TOL_MM,
        }
    }
}

/// Summary metadata returned alongside an imported SVG [`SketchLayer`].
#[derive(Debug, Clone)]
pub struct SvgImportReport {
    pub sketch: SketchLayer,
    pub scale_description: String,
    pub canvas_size_mm: DVec2,
}

/// Imports an SVG file from disk into a healed [`SketchLayer`].
pub fn import_svg_path(
    path: &Path,
    mut opts: SvgImportOptions,
) -> Result<SvgImportReport, SvgImportError> {
    let bytes = std::fs::read(path).map_err(|source| SvgImportError::Io {
        path: path.display().to_string(),
        source,
    })?;
    if opts.source_path.is_none() {
        opts.source_path = Some(path.display().to_string());
    }
    if opts.sketch_name == "Imported SVG"
        && let Some(stem) = path.file_stem().and_then(|s| s.to_str())
    {
        opts.sketch_name = stem.to_string();
    }
    import_svg_bytes(&bytes, &opts)
}

/// Imports an SVG byte slice into a healed [`SketchLayer`].
pub fn import_svg_bytes(
    bytes: &[u8],
    opts: &SvgImportOptions,
) -> Result<SvgImportReport, SvgImportError> {
    let dpi = opts.svg_dpi.clamp(10.0, 2_400.0);
    // Note: `usvg` internally converts physical units (`mm`, `in`, `cm`, `pt`) to `px` using `usvg_opt.dpi`!
    // Setting `usvg_opt.dpi = dpi as f32` and then multiplying `usvg` user-space `px` by `25.4 / dpi`
    // recovers **exact** physical `1.0 mm == 1.0 mm` and `1.0 in == 25.4 mm` dimensions AND scales
    // unitless `px` SVGs at `25.4 / dpi` in a single unified formula!
    let usvg_opt = UsvgOptions {
        dpi: dpi as f32,
        ..UsvgOptions::default()
    };
    let tree =
        Tree::from_data(bytes, &usvg_opt).map_err(|e| SvgImportError::SvgParse(e.to_string()))?;

    let (px_to_mm, scale_desc) = match opts.units_override {
        Some(VectorImportUnits::Millimeters) => (1.0, "1 SVG px = 1.0 mm (forced)".to_string()),
        Some(VectorImportUnits::Inches) => (MM_PER_INCH, "1 SVG px = 1.0 in (forced)".to_string()),
        Some(VectorImportUnits::Centimeters) => (10.0, "1 SVG px = 1.0 cm (forced)".to_string()),
        _ => (
            MM_PER_INCH / dpi,
            format!(
                "Physical Units / {dpi:.0} DPI ({:.4} mm/px)",
                MM_PER_INCH / dpi
            ),
        ),
    };

    let sz = tree.size();
    let canvas_w_mm = f64::from(sz.width()) * px_to_mm;
    let canvas_h_mm = f64::from(sz.height()) * px_to_mm;

    let mut raw_items: Vec<RawImportedItem> = Vec::new();
    collect_svg_group(
        tree.root(),
        None,
        px_to_mm,
        canvas_h_mm,
        opts.arc_fit_tolerance_mm,
        &mut raw_items,
    );

    let healed = heal_and_classify_sketch_geometry(raw_items, opts.join_tolerance_mm);
    let sha = compute_sketch_sha256_hex(bytes);

    let mut sketch = SketchLayer::new(&opts.sketch_id, &opts.sketch_name, &opts.setup_id);
    sketch.anchor = SketchAnchor::SetupXy {
        setup_id: opts.setup_id.clone(),
        plane_ref: SketchPlaneRef::StockTop,
        z_offset: chip_math::ParamLength::from_mm(0.0),
    };
    sketch.source_asset = opts.source_path.as_ref().map(|p| VectorSourceAsset {
        path: p.clone(),
        sha256: sha,
        format: VectorAssetFormat::Svg,
        units: opts.units_override.unwrap_or_default(),
        svg_dpi: dpi,
        join_tolerance_mm: opts.join_tolerance_mm,
        arc_fit_tolerance_mm: opts.arc_fit_tolerance_mm,
    });
    sketch.entities = healed.entities;
    sketch.regions = healed.regions;
    sketch.diagnostics = healed.diagnostics;

    Ok(SvgImportReport {
        sketch,
        scale_description: scale_desc,
        canvas_size_mm: DVec2::new(canvas_w_mm, canvas_h_mm),
    })
}

fn collect_svg_group(
    group: &Group,
    inherited_layer: Option<&str>,
    px_to_mm: f64,
    canvas_h_mm: f64,
    arc_tol_mm: f64,
    out: &mut Vec<RawImportedItem>,
) {
    let grp_id = group.id().trim();
    let eff_layer = if !grp_id.is_empty() {
        Some(grp_id)
    } else {
        inherited_layer
    };

    for node in group.children() {
        match node {
            Node::Group(sub) => {
                collect_svg_group(sub, eff_layer, px_to_mm, canvas_h_mm, arc_tol_mm, out);
            }
            Node::Path(path) => {
                if !path.is_visible() {
                    continue;
                }
                let tf = path.abs_transform();
                let map_pt = |p: usvg::tiny_skia_path::Point| -> KPoint {
                    let x_px = f64::from(tf.sx) * f64::from(p.x)
                        + f64::from(tf.kx) * f64::from(p.y)
                        + f64::from(tf.tx);
                    let y_px = f64::from(tf.ky) * f64::from(p.x)
                        + f64::from(tf.sy) * f64::from(p.y)
                        + f64::from(tf.ty);
                    let x_mm = x_px * px_to_mm;
                    // Flip SVG `+Y down` screen space into right-handed CAM `+Y up`!
                    let y_mm = canvas_h_mm - y_px * px_to_mm;
                    KPoint::new(x_mm, y_mm)
                };

                let mut bez = BezPath::new();
                for seg in path.data().segments() {
                    match seg {
                        usvg::tiny_skia_path::PathSegment::MoveTo(p) => {
                            bez.move_to(map_pt(p));
                        }
                        usvg::tiny_skia_path::PathSegment::LineTo(p) => {
                            bez.line_to(map_pt(p));
                        }
                        usvg::tiny_skia_path::PathSegment::QuadTo(p1, p2) => {
                            bez.quad_to(map_pt(p1), map_pt(p2));
                        }
                        usvg::tiny_skia_path::PathSegment::CubicTo(p1, p2, p3) => {
                            bez.curve_to(map_pt(p1), map_pt(p2), map_pt(p3));
                        }
                        usvg::tiny_skia_path::PathSegment::Close => {
                            bez.close_path();
                        }
                    }
                }

                let sub_layer = eff_layer.map(str::to_string).or_else(|| {
                    let pid = path.id().trim();
                    if pid.is_empty() {
                        None
                    } else {
                        Some(pid.to_string())
                    }
                });

                for poly in fit_bezpath_to_arc_polylines(&bez, arc_tol_mm) {
                    out.push(RawImportedItem::Curve {
                        sub_layer: sub_layer.clone(),
                        polyline: poly,
                    });
                }
            }
            Node::Image(_) | Node::Text(_) => {}
        }
    }
}
