//! AutoCAD **DXF** (`dxf` crate) vector importer with `$INSUNITS` scaling, `LWPOLYLINE` bulge
//! preservation, `SPLINE`/`ELLIPSE` biarc fitting, recursive `INSERT` block expansion, and
//! deterministic sample router-panel fixture generation (`11-sketches-vectors.md`).

use std::{collections::HashSet, io::Cursor, path::Path};

use chip_math::{
    ArcPolyline2D, ArcSegment2D, DAffine2, DEFAULT_ARC_FIT_TOL_MM, DMat2, DVec2, DisplayUnit,
    MM_PER_INCH,
};
use dxf::{
    Drawing,
    entities::{Entity, EntityType},
    enums::{DrawingUnits, Units},
};
use kurbo::{Arc as KurboArc, BezPath, Point as KPoint, Vec2 as KVec2};
use thiserror::Error;

use crate::{
    curve_fit::fit_bezpath_to_arc_polylines,
    heal_chain::{RawImportedItem, heal_and_classify_sketch_geometry},
    model::{
        SketchAnchor, SketchLayer, VectorAssetFormat, VectorImportUnits, VectorSourceAsset,
        compute_sketch_sha256_hex,
    },
};

/// Maximum allowed geometric items emitted during a single DXF import (guards nested `INSERT` bombs).
pub const MAX_DXF_IMPORTED_ITEMS: usize = 250_000;
/// Maximum allowed recursive `INSERT` grid cell expansions during a single DXF import.
pub const MAX_DXF_INSERT_EXPANSIONS: usize = 100_000;

/// Errors encountered while reading or converting a `.dxf` drawing.
#[derive(Debug, Error)]
pub enum DxfImportError {
    #[error("I/O error reading DXF '{path}': {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("DXF parse error: {0}")]
    DxfParse(String),
}

/// Configuration for importing a `.dxf` file into a [`SketchLayer`].
#[derive(Debug, Clone)]
pub struct DxfImportOptions {
    pub sketch_id: String,
    pub sketch_name: String,
    pub setup_id: String,
    pub source_path: Option<String>,
    pub units: VectorImportUnits,
    pub join_tolerance_mm: f64,
    pub arc_fit_tolerance_mm: f64,
    /// Optional allow-list of DXF layer names (`None` imports all visible non-`DEFPOINTS` layers).
    pub include_layers: Option<HashSet<String>>,
}

impl Default for DxfImportOptions {
    fn default() -> Self {
        Self {
            sketch_id: "sketch_1".to_string(),
            sketch_name: "Imported DXF".to_string(),
            setup_id: "setup_1".to_string(),
            source_path: None,
            units: VectorImportUnits::default(),
            join_tolerance_mm: 0.05,
            arc_fit_tolerance_mm: DEFAULT_ARC_FIT_TOL_MM,
            include_layers: None,
        }
    }
}

/// Summary metadata returned alongside an imported DXF [`SketchLayer`].
#[derive(Debug, Clone)]
pub struct DxfImportReport {
    pub sketch: SketchLayer,
    pub detected_dxf_units: String,
    pub unit_scale_to_mm: f64,
    pub available_layers: Vec<String>,
}

/// Returns `Some(uniform_scale)` when the linear part of `xf` is isotropic (`sx == sy` and orthogonal).
fn affine2_uniform_scale(xf: &DAffine2) -> Option<f64> {
    let c0 = xf.matrix2.x_axis;
    let c1 = xf.matrix2.y_axis;
    let s0 = c0.length();
    let s1 = c1.length();
    if s0 > 1e-9 && (s0 - s1).abs() <= 1e-4 * s0 && c0.dot(c1).abs() <= 1e-4 * s0 * s1 {
        Some(s0)
    } else {
        None
    }
}

/// Imports a DXF file from disk into a healed [`SketchLayer`].
pub fn import_dxf_path(
    path: &Path,
    mut opts: DxfImportOptions,
) -> Result<DxfImportReport, DxfImportError> {
    let bytes = std::fs::read(path).map_err(|source| DxfImportError::Io {
        path: path.display().to_string(),
        source,
    })?;
    if opts.source_path.is_none() {
        opts.source_path = Some(path.display().to_string());
    }
    if opts.sketch_name == "Imported DXF"
        && let Some(stem) = path.file_stem().and_then(|s| s.to_str())
    {
        opts.sketch_name = stem.to_string();
    }
    import_dxf_bytes(&bytes, &opts)
}

/// Imports a DXF byte buffer into a healed [`SketchLayer`].
pub fn import_dxf_bytes(
    bytes: &[u8],
    opts: &DxfImportOptions,
) -> Result<DxfImportReport, DxfImportError> {
    let mut cursor = Cursor::new(bytes);
    let drawing =
        Drawing::load(&mut cursor).map_err(|e| DxfImportError::DxfParse(e.to_string()))?;

    let (unit_scale, detected_label) = resolve_dxf_unit_scale(&drawing, opts.units);

    // Collect hidden/frozen layer names from DXF layer table
    let mut hidden_layers: HashSet<String> = HashSet::new();
    hidden_layers.insert("DEFPOINTS".to_string());
    for layer in drawing.layers() {
        if !layer.is_layer_on {
            hidden_layers.insert(layer.name.to_ascii_uppercase());
        }
    }

    let mut available_layers_set: HashSet<String> = HashSet::new();
    for ent in drawing.entities() {
        let lname = ent.common.layer.trim();
        if !lname.is_empty() && !hidden_layers.contains(&lname.to_ascii_uppercase()) {
            available_layers_set.insert(lname.to_string());
        }
    }
    let mut available_layers: Vec<String> = available_layers_set.into_iter().collect();
    available_layers.sort();

    let root_xf = DAffine2::from_scale(DVec2::splat(unit_scale));
    let mut raw_items: Vec<RawImportedItem> = Vec::new();

    let env = DxfTraverseEnv {
        drawing: &drawing,
        hidden_layers: &hidden_layers,
        opts,
    };
    let mut insert_expansions = 0usize;
    let mut limit_exceeded = false;
    for ent in drawing.entities() {
        collect_dxf_entity_with_env(
            &env,
            ent,
            None,
            &root_xf,
            0,
            &mut insert_expansions,
            &mut limit_exceeded,
            &mut raw_items,
        );
        if limit_exceeded {
            return Err(DxfImportError::DxfParse(format!(
                "DXF exceeds complexity limit ({MAX_DXF_IMPORTED_ITEMS} items / {MAX_DXF_INSERT_EXPANSIONS} INSERT expansions)"
            )));
        }
    }

    let healed = heal_and_classify_sketch_geometry(raw_items, opts.join_tolerance_mm);
    let sha = compute_sketch_sha256_hex(bytes);

    let mut sketch = SketchLayer::new(&opts.sketch_id, &opts.sketch_name, &opts.setup_id);
    sketch.anchor = SketchAnchor::SetupXy {
        setup_id: opts.setup_id.clone(),
        plane_ref: crate::model::SketchPlaneRef::StockTop,
        z_offset: chip_math::ParamLength::from_mm(0.0),
    };
    sketch.source_asset = opts.source_path.as_ref().map(|p| VectorSourceAsset {
        path: p.clone(),
        sha256: sha,
        format: VectorAssetFormat::Dxf,
        units: opts.units,
        svg_dpi: 96.0,
        join_tolerance_mm: opts.join_tolerance_mm,
        arc_fit_tolerance_mm: opts.arc_fit_tolerance_mm,
    });
    sketch.entities = healed.entities;
    sketch.regions = healed.regions;
    sketch.diagnostics = healed.diagnostics;

    Ok(DxfImportReport {
        sketch,
        detected_dxf_units: detected_label,
        unit_scale_to_mm: unit_scale,
        available_layers,
    })
}

fn resolve_dxf_unit_scale(drawing: &Drawing, policy: VectorImportUnits) -> (f64, String) {
    match policy {
        VectorImportUnits::Millimeters => (1.0, "Millimeters (forced)".to_string()),
        VectorImportUnits::Inches => (MM_PER_INCH, "Inches (forced)".to_string()),
        VectorImportUnits::Centimeters => (10.0, "Centimeters (forced)".to_string()),
        VectorImportUnits::AutoDetect { fallback } => match drawing.header.default_drawing_units {
            Units::Millimeters => (1.0, "Millimeters ($INSUNITS=4)".to_string()),
            Units::Inches | Units::USSurveyInch => {
                (MM_PER_INCH, "Inches ($INSUNITS=1)".to_string())
            }
            Units::Centimeters => (10.0, "Centimeters ($INSUNITS=5)".to_string()),
            Units::Meters => (1_000.0, "Meters ($INSUNITS=6)".to_string()),
            Units::Feet | Units::USSurveyFeet => {
                (12.0 * MM_PER_INCH, "Feet ($INSUNITS=2)".to_string())
            }
            Units::Mils => (0.0254, "Mils ($INSUNITS=9)".to_string()),
            Units::Microns => (0.001, "Microns ($INSUNITS=13)".to_string()),
            Units::Unitless => {
                // Check `$MEASUREMENT` if present, otherwise honor explicit `fallback`
                if drawing.header.drawing_units == DrawingUnits::Metric {
                    (1.0, "Unitless ($MEASUREMENT=Metric -> mm)".to_string())
                } else {
                    match fallback {
                        DisplayUnit::Millimeter => (1.0, "Unitless (Fallback -> mm)".to_string()),
                        DisplayUnit::Inch => (MM_PER_INCH, "Unitless (Fallback -> in)".to_string()),
                    }
                }
            }
            other => (
                match fallback {
                    DisplayUnit::Millimeter => 1.0,
                    DisplayUnit::Inch => MM_PER_INCH,
                },
                format!("{other:?} (Fallback)"),
            ),
        },
    }
}

struct DxfTraverseEnv<'a> {
    drawing: &'a Drawing,
    hidden_layers: &'a HashSet<String>,
    opts: &'a DxfImportOptions,
}

#[allow(clippy::too_many_arguments)]
fn collect_dxf_entity_with_env(
    env: &DxfTraverseEnv<'_>,
    ent: &Entity,
    inherited_layer: Option<&str>,
    xf: &DAffine2,
    depth: u32,
    insert_expansions: &mut usize,
    limit_exceeded: &mut bool,
    out: &mut Vec<RawImportedItem>,
) {
    let drawing = env.drawing;
    let hidden_layers = env.hidden_layers;
    let opts = env.opts;
    if *limit_exceeded || out.len() >= MAX_DXF_IMPORTED_ITEMS {
        *limit_exceeded = true;
        return;
    }
    if depth > 8 {
        return;
    }
    let raw_layer = ent.common.layer.trim();
    let eff_layer = if (raw_layer.is_empty() || raw_layer == "0") && inherited_layer.is_some() {
        inherited_layer.unwrap_or("0")
    } else if raw_layer.is_empty() {
        "0"
    } else {
        raw_layer
    };

    if hidden_layers.contains(&eff_layer.to_ascii_uppercase()) {
        return;
    }

    // For `INSERT` block references, recurse before filtering by `include_layers` in case block entities
    // live on specific sub-layers!
    if let EntityType::Insert(ins) = &ent.specific {
        if let Some(block) = drawing.blocks().find(|b| b.name == ins.name) {
            let cols = ins.column_count.max(1) as usize;
            let rows = ins.row_count.max(1) as usize;
            let sx = if ins.x_scale_factor.abs() > 1e-9 {
                ins.x_scale_factor
            } else {
                1.0
            };
            let sy = if ins.y_scale_factor.abs() > 1e-9 {
                ins.y_scale_factor
            } else {
                1.0
            };
            let rot = ins.rotation.to_radians();
            let (sin_r, cos_r) = rot.sin_cos();
            let base_pt = DVec2::new(block.base_point.x, block.base_point.y);

            for r in 0..rows.min(64) {
                for c in 0..cols.min(64) {
                    *insert_expansions = insert_expansions.saturating_add(1);
                    if *insert_expansions > MAX_DXF_INSERT_EXPANSIONS
                        || out.len() >= MAX_DXF_IMPORTED_ITEMS
                    {
                        *limit_exceeded = true;
                        return;
                    }
                    let local_offset = DVec2::new(
                        ins.location.x + (c as f64) * ins.column_spacing,
                        ins.location.y + (r as f64) * ins.row_spacing,
                    );
                    let mat = DMat2::from_cols(
                        DVec2::new(cos_r * sx, sin_r * sx),
                        DVec2::new(-sin_r * sy, cos_r * sy),
                    );
                    let shifted_base = mat * base_pt;
                    let ins_xf =
                        *xf * DAffine2::from_mat2_translation(mat, local_offset - shifted_base);
                    for child in &block.entities {
                        collect_dxf_entity_with_env(
                            env,
                            child,
                            Some(eff_layer),
                            &ins_xf,
                            depth + 1,
                            insert_expansions,
                            limit_exceeded,
                            out,
                        );
                        if *limit_exceeded {
                            return;
                        }
                    }
                }
            }
        }
        return;
    }

    if let Some(allowed) = &opts.include_layers
        && !allowed.contains(eff_layer)
    {
        return;
    }
    let sub_layer = Some(eff_layer.to_string());

    match &ent.specific {
        EntityType::Line(line) => {
            let p0 = xf.transform_point2(DVec2::new(line.p1.x, line.p1.y));
            let p1 = xf.transform_point2(DVec2::new(line.p2.x, line.p2.y));
            if p0.distance(p1) > 1e-5 {
                out.push(RawImportedItem::Curve {
                    sub_layer,
                    polyline: ArcPolyline2D {
                        segments: vec![ArcSegment2D::Line { start: p0, end: p1 }],
                        closed: false,
                    },
                });
            }
        }
        EntityType::Circle(circ) => {
            let ocs_flip_x = if circ.normal.z < -0.5 { -1.0 } else { 1.0 };
            let c_local = DVec2::new(ocs_flip_x * circ.center.x, circ.center.y);
            if let Some(u_scale) = affine2_uniform_scale(xf) {
                let c = xf.transform_point2(c_local);
                let d = 2.0 * circ.radius.abs() * u_scale;
                if d > 1e-4 {
                    out.push(RawImportedItem::Circle {
                        sub_layer,
                        center_xy: c,
                        diameter_mm: d,
                    });
                }
            } else {
                // Non-uniform block scaling turns a circle into an ellipse: biarc-fit via `kurbo::Arc`!
                emit_transformed_arc(
                    LocalArcSpan {
                        center: c_local,
                        radius: circ.radius.abs(),
                        start_angle: 0.0,
                        sweep: std::f64::consts::TAU,
                    },
                    xf,
                    opts.arc_fit_tolerance_mm,
                    sub_layer,
                    out,
                );
            }
        }
        EntityType::Arc(arc) => {
            let ocs_flip_x = if arc.normal.z < -0.5 { -1.0 } else { 1.0 };
            let c_local = DVec2::new(ocs_flip_x * arc.center.x, arc.center.y);
            let a0 = arc.start_angle.to_radians();
            let a1 = arc.end_angle.to_radians();
            let mut sweep = (a1 - a0).rem_euclid(std::f64::consts::TAU);
            if sweep <= 1e-6 {
                sweep = std::f64::consts::TAU;
            }
            if let Some(u_scale) = affine2_uniform_scale(xf)
                && ocs_flip_x > 0.0
            {
                let p0 = xf.transform_point2(c_local + DVec2::new(a0.cos(), a0.sin()) * arc.radius);
                let p1 = xf.transform_point2(c_local + DVec2::new(a1.cos(), a1.sin()) * arc.radius);
                let center = xf.transform_point2(c_local);
                let radius = arc.radius.abs() * u_scale;
                let ccw = xf.matrix2.determinant() >= 0.0;
                // Split arcs > 180° into two halves so unambiguous arc endpoints are always well-conditioned
                if sweep > std::f64::consts::PI + 1e-4 {
                    let am = a0 + 0.5 * sweep;
                    let pm =
                        xf.transform_point2(c_local + DVec2::new(am.cos(), am.sin()) * arc.radius);
                    out.push(RawImportedItem::Curve {
                        sub_layer,
                        polyline: ArcPolyline2D {
                            segments: vec![
                                ArcSegment2D::Arc {
                                    start: p0,
                                    end: pm,
                                    center,
                                    radius,
                                    ccw,
                                },
                                ArcSegment2D::Arc {
                                    start: pm,
                                    end: p1,
                                    center,
                                    radius,
                                    ccw,
                                },
                            ],
                            closed: sweep >= std::f64::consts::TAU - 1e-4,
                        },
                    });
                } else {
                    out.push(RawImportedItem::Curve {
                        sub_layer,
                        polyline: ArcPolyline2D {
                            segments: vec![ArcSegment2D::Arc {
                                start: p0,
                                end: p1,
                                center,
                                radius,
                                ccw,
                            }],
                            closed: false,
                        },
                    });
                }
            } else {
                emit_transformed_arc(
                    LocalArcSpan {
                        center: c_local,
                        radius: arc.radius.abs(),
                        start_angle: a0,
                        sweep,
                    },
                    xf,
                    opts.arc_fit_tolerance_mm,
                    sub_layer,
                    out,
                );
            }
        }
        EntityType::LwPolyline(lwp) => {
            let ocs_flip_x = if lwp.extrusion_direction.z < -0.5 {
                -1.0
            } else {
                1.0
            };
            let verts: Vec<(DVec2, f64)> = lwp
                .vertices
                .iter()
                .map(|v| (DVec2::new(ocs_flip_x * v.x, v.y), ocs_flip_x * v.bulge))
                .collect();
            if let Some(poly) =
                build_bulge_polyline(&verts, lwp.is_closed(), xf, opts.arc_fit_tolerance_mm)
            {
                out.push(RawImportedItem::Curve {
                    sub_layer,
                    polyline: poly,
                });
            }
        }
        EntityType::Polyline(poly) => {
            let ocs_flip_x = if poly.normal.z < -0.5 { -1.0 } else { 1.0 };
            let verts: Vec<(DVec2, f64)> = poly
                .vertices()
                .map(|v| {
                    (
                        DVec2::new(ocs_flip_x * v.location.x, v.location.y),
                        ocs_flip_x * v.bulge,
                    )
                })
                .collect();
            if let Some(arc_poly) =
                build_bulge_polyline(&verts, poly.is_closed(), xf, opts.arc_fit_tolerance_mm)
            {
                out.push(RawImportedItem::Curve {
                    sub_layer,
                    polyline: arc_poly,
                });
            }
        }
        EntityType::Ellipse(ell) => {
            let c = DVec2::new(ell.center.x, ell.center.y);
            let maj = DVec2::new(ell.major_axis.x, ell.major_axis.y);
            let rx = maj.length();
            let ry = rx * ell.minor_axis_ratio.abs();
            if rx > 1e-5 && ry > 1e-5 {
                let x_rot = maj.y.atan2(maj.x);
                let mut sweep =
                    (ell.end_parameter - ell.start_parameter).rem_euclid(std::f64::consts::TAU);
                if sweep <= 1e-5 {
                    sweep = std::f64::consts::TAU;
                }
                let k_arc = KurboArc::new(
                    KPoint::new(c.x, c.y),
                    KVec2::new(rx, ry),
                    ell.start_parameter,
                    sweep,
                    x_rot,
                );
                let mut bez = BezPath::new();
                let mut first = true;
                k_arc.to_cubic_beziers(0.05, |p1, p2, p3| {
                    if first {
                        let s0 =
                            sample_ellipse(&c, &maj, ell.minor_axis_ratio, ell.start_parameter);
                        let ts0 = xf.transform_point2(s0);
                        bez.move_to(KPoint::new(ts0.x, ts0.y));
                        first = false;
                    }
                    let tp1 = xf.transform_point2(DVec2::new(p1.x, p1.y));
                    let tp2 = xf.transform_point2(DVec2::new(p2.x, p2.y));
                    let tp3 = xf.transform_point2(DVec2::new(p3.x, p3.y));
                    bez.curve_to(
                        KPoint::new(tp1.x, tp1.y),
                        KPoint::new(tp2.x, tp2.y),
                        KPoint::new(tp3.x, tp3.y),
                    );
                });
                if sweep >= std::f64::consts::TAU - 1e-4 {
                    bez.close_path();
                }
                for poly in fit_bezpath_to_arc_polylines(&bez, opts.arc_fit_tolerance_mm) {
                    out.push(RawImportedItem::Curve {
                        sub_layer: sub_layer.clone(),
                        polyline: poly,
                    });
                }
            }
        }
        EntityType::Spline(sp) => {
            if let Some(poly) = evaluate_dxf_spline(sp, xf, opts.arc_fit_tolerance_mm) {
                out.push(RawImportedItem::Curve {
                    sub_layer,
                    polyline: poly,
                });
            }
        }
        _ => {}
    }
}

fn sample_ellipse(center: &DVec2, major: &DVec2, minor_ratio: f64, param: f64) -> DVec2 {
    let minor = DVec2::new(-major.y, major.x) * minor_ratio;
    *center + *major * param.cos() + minor * param.sin()
}

#[derive(Clone, Copy)]
struct LocalArcSpan {
    center: DVec2,
    radius: f64,
    start_angle: f64,
    sweep: f64,
}

fn emit_transformed_arc(
    arc: LocalArcSpan,
    xf: &DAffine2,
    arc_tol: f64,
    sub_layer: Option<String>,
    out: &mut Vec<RawImportedItem>,
) {
    let LocalArcSpan {
        center,
        radius,
        start_angle,
        sweep,
    } = arc;
    if radius <= 1e-5 {
        return;
    }
    let k_arc = KurboArc::new(
        KPoint::new(center.x, center.y),
        KVec2::new(radius, radius),
        start_angle,
        sweep,
        0.0,
    );
    let mut bez = BezPath::new();
    let p0 =
        xf.transform_point2(center + DVec2::new(start_angle.cos(), start_angle.sin()) * radius);
    bez.move_to(KPoint::new(p0.x, p0.y));
    k_arc.to_cubic_beziers(0.05, |p1, p2, p3| {
        let tp1 = xf.transform_point2(DVec2::new(p1.x, p1.y));
        let tp2 = xf.transform_point2(DVec2::new(p2.x, p2.y));
        let tp3 = xf.transform_point2(DVec2::new(p3.x, p3.y));
        bez.curve_to(
            KPoint::new(tp1.x, tp1.y),
            KPoint::new(tp2.x, tp2.y),
            KPoint::new(tp3.x, tp3.y),
        );
    });
    if sweep >= std::f64::consts::TAU - 1e-4 {
        bez.close_path();
    }
    for poly in fit_bezpath_to_arc_polylines(&bez, arc_tol) {
        out.push(RawImportedItem::Curve {
            sub_layer: sub_layer.clone(),
            polyline: poly,
        });
    }
}

fn build_bulge_polyline(
    verts: &[(DVec2, f64)],
    is_closed_flag: bool,
    xf: &DAffine2,
    arc_tol: f64,
) -> Option<ArcPolyline2D> {
    if verts.len() < 2 {
        return None;
    }
    let is_closed = is_closed_flag
        || (verts.len() >= 3 && verts[0].0.distance(verts[verts.len() - 1].0) <= 1e-5);
    let seg_count = if is_closed {
        verts.len()
    } else {
        verts.len() - 1
    };

    let uniform = affine2_uniform_scale(xf);
    let flip = xf.matrix2.determinant() < 0.0;
    let mut segments = Vec::with_capacity(seg_count);

    for i in 0..seg_count {
        let (p0_local, bulge) = verts[i];
        let (p1_local, _) = verts[(i + 1) % verts.len()];
        if p0_local.distance(p1_local) <= 1e-6 {
            continue;
        }
        match ArcSegment2D::from_bulge(p0_local, p1_local, bulge) {
            ArcSegment2D::Line { .. } => {
                segments.push(ArcSegment2D::Line {
                    start: xf.transform_point2(p0_local),
                    end: xf.transform_point2(p1_local),
                });
            }
            ArcSegment2D::Arc {
                center: c_local,
                radius: rad_local,
                ccw,
                ..
            } => {
                if let Some(u_scale) = uniform {
                    segments.push(ArcSegment2D::Arc {
                        start: xf.transform_point2(p0_local),
                        end: xf.transform_point2(p1_local),
                        center: xf.transform_point2(c_local),
                        radius: rad_local * u_scale,
                        ccw: if flip { !ccw } else { ccw },
                    });
                } else {
                    let sweep = 4.0 * bulge.atan();
                    let a0 = (p0_local.y - c_local.y).atan2(p0_local.x - c_local.x);
                    let mut temp = Vec::new();
                    emit_transformed_arc(
                        LocalArcSpan {
                            center: c_local,
                            radius: rad_local,
                            start_angle: a0,
                            sweep,
                        },
                        xf,
                        arc_tol,
                        None,
                        &mut temp,
                    );
                    for item in temp {
                        if let RawImportedItem::Curve { polyline, .. } = item {
                            segments.extend(polyline.segments);
                        }
                    }
                }
            }
        }
    }

    if segments.is_empty() {
        None
    } else {
        let actual_closed = is_closed
            && segments
                .first()
                .zip(segments.last())
                .is_some_and(|(a, b)| a.start().distance(b.end()) <= 1e-3);
        Some(ArcPolyline2D {
            segments,
            closed: actual_closed,
        })
    }
}

/// Evaluates a DXF NURBS/B-spline (`Spline`) via Cox-de Boor over `[t_min, t_max]` and biarc-fits
/// smooth segments within `arc_tol_mm`.
fn evaluate_dxf_spline(
    sp: &dxf::entities::Spline,
    xf: &DAffine2,
    arc_tol_mm: f64,
) -> Option<ArcPolyline2D> {
    let ctrl: Vec<DVec2> = sp
        .control_points
        .iter()
        .map(|p| xf.transform_point2(DVec2::new(p.x, p.y)))
        .collect();

    if ctrl.len() < 2 {
        // Fallback to `fit_points` polyline if only fit points were stored
        let fits: Vec<DVec2> = sp
            .fit_points
            .iter()
            .map(|p| xf.transform_point2(DVec2::new(p.x, p.y)))
            .collect();
        if fits.len() < 2 {
            return None;
        }
        let mut segs = Vec::with_capacity(fits.len() - 1);
        for w in fits.windows(2) {
            if w[0].distance(w[1]) > 1e-5 {
                segs.push(ArcSegment2D::Line {
                    start: w[0],
                    end: w[1],
                });
            }
        }
        let closed = segs
            .first()
            .zip(segs.last())
            .is_some_and(|(a, b)| a.start().distance(b.end()) <= 1e-3);
        return Some(ArcPolyline2D {
            segments: segs,
            closed,
        });
    }

    let deg = (sp.degree_of_curve.max(1) as usize).min(ctrl.len() - 1);
    let n = ctrl.len();
    let knots: Vec<f64> = if sp.knot_values.len() == n + deg + 1 {
        sp.knot_values.clone()
    } else {
        // Clamped uniform knot vector fallback
        let mut kv = Vec::with_capacity(n + deg + 1);
        kv.extend(std::iter::repeat_n(0.0, deg + 1));
        let internal = n - deg - 1;
        for i in 1..=internal {
            kv.push((i as f64) / ((internal + 1) as f64));
        }
        kv.extend(std::iter::repeat_n(1.0, deg + 1));
        kv
    };

    let t_min = knots[deg];
    let t_max = knots[n];
    if t_max - t_min <= 1e-9 {
        return None;
    }

    let weights: Vec<f64> = if sp.weight_values.len() == n {
        sp.weight_values
            .iter()
            .map(|&w| if w > 1e-6 { w } else { 1.0 })
            .collect()
    } else {
        vec![1.0; n]
    };

    // Sample smooth Catmull/Hermite cubic spans and run our exact circular-arc fitter!
    let n_spans = (n * 6).clamp(16, 192);
    let mut pts = Vec::with_capacity(n_spans + 1);
    for k in 0..=n_spans {
        let u = if k == n_spans {
            t_max - 1e-9
        } else {
            t_min + (t_max - t_min) * ((k as f64) / (n_spans as f64))
        };
        if let Some(p) = eval_nurbs_point(deg, &knots, &ctrl, &weights, u) {
            pts.push(p);
        }
    }
    if pts.len() < 2 {
        return None;
    }

    // Convert sampled smooth curve into Catmull-Rom cubic BezPath and biarc-fit within `arc_tol_mm`!
    let mut bez = BezPath::new();
    bez.move_to(KPoint::new(pts[0].x, pts[0].y));
    for i in 0..(pts.len() - 1) {
        let p_prev = if i == 0 { pts[0] } else { pts[i - 1] };
        let p0 = pts[i];
        let p1 = pts[i + 1];
        let p_next = if i + 2 < pts.len() {
            pts[i + 2]
        } else {
            pts[pts.len() - 1]
        };
        let c1 = p0 + (p1 - p_prev) / 6.0;
        let c2 = p1 - (p_next - p0) / 6.0;
        bez.curve_to(
            KPoint::new(c1.x, c1.y),
            KPoint::new(c2.x, c2.y),
            KPoint::new(p1.x, p1.y),
        );
    }
    if pts[0].distance(*pts.last().unwrap_or(&pts[0])) <= 1e-3 {
        bez.close_path();
    }
    fit_bezpath_to_arc_polylines(&bez, arc_tol_mm)
        .into_iter()
        .next()
}

fn eval_nurbs_point(
    deg: usize,
    knots: &[f64],
    ctrl: &[DVec2],
    weights: &[f64],
    u: f64,
) -> Option<DVec2> {
    let n = ctrl.len();
    // Locate knot span `span` such that `knots[span] <= u < knots[span + 1]`
    let mut span = deg;
    for i in deg..n {
        if u >= knots[i] && u < knots[i + 1] {
            span = i;
            break;
        }
        if i + 1 == n {
            span = n - 1;
        }
    }

    let mut d_hom: Vec<(DVec2, f64)> = (0..=deg)
        .map(|j| {
            let idx = span + j - deg;
            let w = weights[idx];
            (ctrl[idx] * w, w)
        })
        .collect();

    for r in 1..=deg {
        for j in (r..=deg).rev() {
            let i = span + j - deg;
            let denom = knots[i + deg + 1 - r] - knots[i];
            let alpha = if denom.abs() > 1e-12 {
                (u - knots[i]) / denom
            } else {
                0.0
            };
            let prev = d_hom[j - 1];
            let curr = d_hom[j];
            d_hom[j] = (
                prev.0 * (1.0 - alpha) + curr.0 * alpha,
                prev.1 * (1.0 - alpha) + curr.1 * alpha,
            );
        }
    }

    let (num, w) = d_hom[deg];
    if w.abs() > 1e-12 { Some(num / w) } else { None }
}
