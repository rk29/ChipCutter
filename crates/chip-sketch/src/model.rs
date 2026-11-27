use chip_math::{
    Aabb2, Aabb3, ArcPolyline2D, DEFAULT_ARC_FIT_TOL_MM, DVec2, DVec3, DisplayUnit, EvalContext,
    ParamLength,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::{
    heal_chain::classify_planar_regions_from_candidates,
    primitives::{
        HolePoint2D, PointPatternSpec, PolygonEvalSpec, PolygonSizingMode, RectAnchorMode,
        RectEvalSpec, SlotLengthMode, build_circle_polyline, build_rectangle_polyline,
        build_regular_polygon_polyline, build_slot_polyline, transform_arc_polyline_2d,
    },
    typography::{CustomFontMap, SketchTextSpec},
};

fn default_true() -> bool {
    true
}

fn default_one() -> f64 {
    1.0
}

fn default_svg_dpi() -> f64 {
    96.0
}

fn default_join_tol_mm() -> f64 {
    0.05
}

fn default_arc_tol_mm() -> f64 {
    DEFAULT_ARC_FIT_TOL_MM
}

fn default_cut_depth() -> ParamLength {
    ParamLength::from_mm(6.0)
}

fn default_zero_length() -> ParamLength {
    ParamLength::from_mm(0.0)
}

fn default_setup_id() -> String {
    "setup_1".to_string()
}

/// Reference Z plane for a `SetupXy`-anchored [`SketchLayer`] (`11-sketches-vectors.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SketchPlaneRef {
    #[default]
    StockTop,
    ModelTop,
    StockBottom,
    WcsOrigin,
}

impl SketchPlaneRef {
    pub const ALL: [Self; 4] = [
        Self::StockTop,
        Self::ModelTop,
        Self::StockBottom,
        Self::WcsOrigin,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::StockTop => "Stock Top",
            Self::ModelTop => "Model Top",
            Self::StockBottom => "Stock Bottom",
            Self::WcsOrigin => "WCS Origin (Z0)",
        }
    }
}

/// Origin placement mode for [`SketchAnchor::PlanarFace`] (`11-sketches-vectors.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FaceSketchOriginMode {
    #[default]
    FaceCentroid,
    SetupWcsProjected,
}

impl FaceSketchOriginMode {
    pub const ALL: [Self; 2] = [Self::FaceCentroid, Self::SetupWcsProjected];

    pub const fn label(self) -> &'static str {
        match self {
            Self::FaceCentroid => "Face Centroid",
            Self::SetupWcsProjected => "Projected Setup WCS Origin",
        }
    }
}

/// CAD-kernel-independent persistent planar B-Rep face fingerprint for [`SketchAnchor::PlanarFace`].
///
/// Maps 1-to-1 with `chip_cad::SelectionBinding` (`EntitySignatureKind::PlanarFace`) during
/// associative STEP reload (`v1 -> v2`) without coupling `chip-sketch` to `monstertruck`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SketchFaceBinding {
    pub body_id: u32,
    pub last_face_index: u32,
    pub normal_world: DVec3,
    pub centroid_world_mm: DVec3,
    pub area_mm2: f64,
    pub aabb_mm: Aabb3,
}

/// Coordinate frame anchor of a 2D [`SketchLayer`] (`11-sketches-vectors.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SketchAnchor {
    SetupXy {
        setup_id: String,
        #[serde(default)]
        plane_ref: SketchPlaneRef,
        #[serde(default = "default_zero_length")]
        z_offset: ParamLength,
    },
    PlanarFace {
        #[serde(default = "default_setup_id")]
        setup_id: String,
        face_binding: SketchFaceBinding,
        #[serde(default)]
        origin_mode: FaceSketchOriginMode,
        #[serde(default)]
        x_axis_clock_deg: f64,
        #[serde(default = "default_zero_length")]
        z_offset: ParamLength,
    },
}

impl Default for SketchAnchor {
    fn default() -> Self {
        Self::SetupXy {
            setup_id: default_setup_id(),
            plane_ref: SketchPlaneRef::StockTop,
            z_offset: ParamLength::from_mm(0.0),
        }
    }
}

impl SketchAnchor {
    /// Returns the primary Setup ID associated with this sketch for variable scoping (`stock_x`, etc.).
    pub fn setup_id(&self) -> &str {
        match self {
            Self::SetupXy { setup_id, .. } | Self::PlanarFace { setup_id, .. } => setup_id,
        }
    }

    pub fn set_setup_id(&mut self, new_id: String) {
        match self {
            Self::SetupXy { setup_id, .. } | Self::PlanarFace { setup_id, .. } => {
                *setup_id = new_id;
            }
        }
    }

    pub fn z_offset_mut(&mut self) -> &mut ParamLength {
        match self {
            Self::SetupXy { z_offset, .. } | Self::PlanarFace { z_offset, .. } => z_offset,
        }
    }
}

/// Vector file format of an imported [`VectorSourceAsset`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum VectorAssetFormat {
    Dxf,
    Svg,
}

impl VectorAssetFormat {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Dxf => "DXF",
            Self::Svg => "SVG",
        }
    }
}

/// Unit interpretation policy when importing DXF or unitless SVG drawings (`11-sketches-vectors.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum VectorImportUnits {
    AutoDetect {
        #[serde(default)]
        fallback: DisplayUnit,
    },
    Millimeters,
    Inches,
    Centimeters,
}

impl Default for VectorImportUnits {
    fn default() -> Self {
        Self::AutoDetect {
            fallback: DisplayUnit::Millimeter,
        }
    }
}

impl VectorImportUnits {
    pub const fn label(self) -> &'static str {
        match self {
            Self::AutoDetect {
                fallback: DisplayUnit::Millimeter,
            } => "Auto ($INSUNITS / Fallback mm)",
            Self::AutoDetect {
                fallback: DisplayUnit::Inch,
            } => "Auto ($INSUNITS / Fallback in)",
            Self::Millimeters => "Millimeters (mm)",
            Self::Inches => "Inches (in)",
            Self::Centimeters => "Centimeters (cm)",
        }
    }
}

/// Provenance metadata for an external `.dxf` or `.svg` file imported into a [`SketchLayer`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VectorSourceAsset {
    pub path: String,
    pub sha256: String,
    pub format: VectorAssetFormat,
    #[serde(default)]
    pub units: VectorImportUnits,
    #[serde(default = "default_svg_dpi")]
    pub svg_dpi: f64,
    #[serde(default = "default_join_tol_mm")]
    pub join_tolerance_mm: f64,
    #[serde(default = "default_arc_tol_mm")]
    pub arc_fit_tolerance_mm: f64,
}

/// Parametric or imported 2D curve/primitive inside a [`SketchLayer`] (`11-sketches-vectors.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum SketchEntity {
    Rectangle {
        #[serde(default)]
        anchor: RectAnchorMode,
        #[serde(default = "default_zero_length")]
        x: ParamLength,
        #[serde(default = "default_zero_length")]
        y: ParamLength,
        width: ParamLength,
        height: ParamLength,
        #[serde(default = "default_zero_length")]
        corner_radius: ParamLength,
        #[serde(default)]
        rotation_deg: f64,
    },
    Circle {
        #[serde(default = "default_zero_length")]
        center_x: ParamLength,
        #[serde(default = "default_zero_length")]
        center_y: ParamLength,
        /// Backward-compatibility reader for Milestone 7 `center_xy = [x, y]` manifests.
        #[serde(default, skip_serializing)]
        center_xy: Option<DVec2>,
        diameter: ParamLength,
    },
    Slot {
        #[serde(default)]
        mode: SlotLengthMode,
        #[serde(default = "default_zero_length")]
        x: ParamLength,
        #[serde(default = "default_zero_length")]
        y: ParamLength,
        length: ParamLength,
        width: ParamLength,
        #[serde(default)]
        angle_deg: f64,
    },
    RegularPolygon {
        #[serde(default = "default_zero_length")]
        center_x: ParamLength,
        #[serde(default = "default_zero_length")]
        center_y: ParamLength,
        sides: u32,
        #[serde(default)]
        sizing: PolygonSizingMode,
        diameter: ParamLength,
        #[serde(default = "default_zero_length")]
        corner_radius: ParamLength,
        #[serde(default)]
        rotation_deg: f64,
    },
    Polyline {
        polyline: ArcPolyline2D,
        #[serde(default = "default_zero_length")]
        translate_x: ParamLength,
        #[serde(default = "default_zero_length")]
        translate_y: ParamLength,
        #[serde(default = "default_one")]
        scale: f64,
        #[serde(default)]
        rotation_deg: f64,
    },
    PointPattern(PointPatternSpec),
    Text(SketchTextSpec),
}

impl SketchEntity {
    /// Constructs a parametric `Circle` entity.
    pub fn new_circle(center_x: ParamLength, center_y: ParamLength, diameter: ParamLength) -> Self {
        Self::Circle {
            center_x,
            center_y,
            center_xy: None,
            diameter,
        }
    }

    /// Constructs an untransformed `Polyline` entity from `polyline`.
    pub fn new_polyline(polyline: ArcPolyline2D) -> Self {
        Self::Polyline {
            polyline,
            translate_x: ParamLength::from_mm(0.0),
            translate_y: ParamLength::from_mm(0.0),
            scale: 1.0,
            rotation_deg: 0.0,
        }
    }

    pub const fn kind_label(&self) -> &'static str {
        match self {
            Self::Rectangle { .. } => "Rectangle",
            Self::Circle { .. } => "Circle",
            Self::Slot { .. } => "Slot",
            Self::RegularPolygon { .. } => "Polygon",
            Self::Polyline { .. } => "Polyline",
            Self::PointPattern(_) => "Point Pattern",
            Self::Text(_) => "Text",
        }
    }

    /// Resolves the circle center (`mm`), honoring legacy M7 `center_xy` if present.
    pub fn circle_center_mm(&self) -> Option<DVec2> {
        match self {
            Self::Circle {
                center_x,
                center_y,
                center_xy,
                ..
            } => {
                Some(center_xy.unwrap_or_else(|| DVec2::new(center_x.value_mm, center_y.value_mm)))
            }
            _ => None,
        }
    }

    /// Evaluates hole target centers + diameters for `Circle` and `PointPattern` entities.
    pub fn evaluate_holes(&self) -> Vec<HolePoint2D> {
        match self {
            Self::Circle { diameter, .. } => {
                if let Some(c) = self.circle_center_mm() {
                    let d = diameter.value_mm.max(0.1);
                    vec![HolePoint2D {
                        center_xy: c,
                        diameter_mm: d,
                    }]
                } else {
                    Vec::new()
                }
            }
            Self::PointPattern(pat) => pat.evaluate_holes(),
            _ => Vec::new(),
        }
    }

    /// Returns characteristic center snap points (`mm`) for [`crate::snap::snap_sketch_point_2d`].
    pub fn snap_centers_2d(&self) -> Vec<DVec2> {
        match self {
            Self::Rectangle { x, y, .. } => vec![DVec2::new(x.value_mm, y.value_mm)],
            Self::Circle { .. } => self.circle_center_mm().into_iter().collect(),
            Self::Slot { x, y, .. } => vec![DVec2::new(x.value_mm, y.value_mm)],
            Self::RegularPolygon {
                center_x, center_y, ..
            } => vec![DVec2::new(center_x.value_mm, center_y.value_mm)],
            Self::Polyline {
                translate_x,
                translate_y,
                ..
            } => {
                let t = DVec2::new(translate_x.value_mm, translate_y.value_mm);
                if t.length_squared() > 1e-10 {
                    vec![t]
                } else {
                    Vec::new()
                }
            }
            Self::PointPattern(pat) => {
                let mut pts: Vec<DVec2> = pat
                    .evaluate_holes()
                    .into_iter()
                    .map(|h| h.center_xy)
                    .collect();
                if let PointPatternSpec::BoltCircle {
                    center_x, center_y, ..
                } = pat
                {
                    pts.push(DVec2::new(center_x.value_mm, center_y.value_mm));
                }
                pts
            }
            Self::Text(txt) => vec![txt.origin_mm()],
        }
    }

    /// Evaluates this entity into one or more canonical `f64` millimeter [`ArcPolyline2D`]s.
    pub fn to_arc_polylines(&self, custom_fonts: Option<&CustomFontMap>) -> Vec<ArcPolyline2D> {
        match self {
            Self::Rectangle {
                anchor,
                x,
                y,
                width,
                height,
                corner_radius,
                rotation_deg,
            } => {
                let poly = build_rectangle_polyline(RectEvalSpec {
                    anchor: *anchor,
                    anchor_xy: DVec2::new(x.value_mm, y.value_mm),
                    width_mm: width.value_mm,
                    height_mm: height.value_mm,
                    corner_radius_mm: corner_radius.value_mm,
                    rotation_deg: *rotation_deg,
                });
                if poly.segments.is_empty() {
                    Vec::new()
                } else {
                    vec![poly]
                }
            }
            Self::Circle { diameter, .. } => {
                let c = self.circle_center_mm().unwrap_or(DVec2::ZERO);
                let poly = build_circle_polyline(c, diameter.value_mm);
                if poly.segments.is_empty() {
                    Vec::new()
                } else {
                    vec![poly]
                }
            }
            Self::Slot {
                mode,
                x,
                y,
                length,
                width,
                angle_deg,
            } => {
                let poly = build_slot_polyline(
                    *mode,
                    DVec2::new(x.value_mm, y.value_mm),
                    length.value_mm,
                    width.value_mm,
                    *angle_deg,
                );
                if poly.segments.is_empty() {
                    Vec::new()
                } else {
                    vec![poly]
                }
            }
            Self::RegularPolygon {
                center_x,
                center_y,
                sides,
                sizing,
                diameter,
                corner_radius,
                rotation_deg,
            } => {
                let poly = build_regular_polygon_polyline(PolygonEvalSpec {
                    center_xy: DVec2::new(center_x.value_mm, center_y.value_mm),
                    sides: *sides,
                    sizing: *sizing,
                    diameter_mm: diameter.value_mm,
                    corner_radius_mm: corner_radius.value_mm,
                    rotation_deg: *rotation_deg,
                });
                if poly.segments.is_empty() {
                    Vec::new()
                } else {
                    vec![poly]
                }
            }
            Self::Polyline {
                polyline,
                translate_x,
                translate_y,
                scale,
                rotation_deg,
            } => {
                let t = DVec2::new(translate_x.value_mm, translate_y.value_mm);
                let poly = if t.length_squared() <= 1e-12
                    && (*scale - 1.0).abs() <= 1e-9
                    && rotation_deg.abs() <= 1e-9
                {
                    polyline.clone()
                } else {
                    transform_arc_polyline_2d(polyline, t, *scale, *rotation_deg)
                };
                if poly.segments.is_empty() {
                    Vec::new()
                } else {
                    vec![poly]
                }
            }
            Self::PointPattern(pat) => pat
                .evaluate_holes()
                .into_iter()
                .map(|h| build_circle_polyline(h.center_xy, h.diameter_mm))
                .filter(|p| !p.segments.is_empty())
                .collect(),
            Self::Text(txt) => txt.evaluate_polylines(custom_fonts),
        }
    }

    /// Evaluates the primary [`ArcPolyline2D`] of a single-contour entity (convenience wrapper
    /// over [`Self::to_arc_polylines`]).
    pub fn to_arc_polyline(&self) -> ArcPolyline2D {
        self.to_arc_polylines(None)
            .into_iter()
            .next()
            .unwrap_or_default()
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        match self {
            Self::Rectangle {
                x,
                y,
                width,
                height,
                corner_radius,
                ..
            } => {
                for p in [x, y, width, height, corner_radius] {
                    let _ = p.reevaluate(ctx);
                }
            }
            Self::Circle {
                center_x,
                center_y,
                center_xy,
                diameter,
            } => {
                if let Some(legacy) = center_xy.take() {
                    *center_x = ParamLength::from_mm(legacy.x);
                    *center_y = ParamLength::from_mm(legacy.y);
                }
                let _ = center_x.reevaluate(ctx);
                let _ = center_y.reevaluate(ctx);
                let _ = diameter.reevaluate(ctx);
            }
            Self::Slot {
                x,
                y,
                length,
                width,
                ..
            } => {
                for p in [x, y, length, width] {
                    let _ = p.reevaluate(ctx);
                }
            }
            Self::RegularPolygon {
                center_x,
                center_y,
                diameter,
                corner_radius,
                ..
            } => {
                for p in [center_x, center_y, diameter, corner_radius] {
                    let _ = p.reevaluate(ctx);
                }
            }
            Self::Polyline {
                translate_x,
                translate_y,
                ..
            } => {
                let _ = translate_x.reevaluate(ctx);
                let _ = translate_y.reevaluate(ctx);
            }
            Self::PointPattern(pat) => pat.reevaluate(ctx),
            Self::Text(txt) => txt.reevaluate(ctx),
        }
    }
}

/// Named, addressable 2D entity row inside a [`SketchLayer`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SketchEntityEntry {
    pub id: String,
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sub_layer: Option<String>,
    #[serde(default)]
    pub is_construction: bool,
    pub entity: SketchEntity,
}

/// Closed non-self-intersecting 2D planar region (`CCW` outer boundary + direct `CW` hole/island loops)
/// classified on a [`SketchLayer`] (`11-sketches-vectors.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlanarVectorRegion {
    pub id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sub_layer: Option<String>,
    pub outer_ccw: ArcPolyline2D,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub islands_cw: Vec<ArcPolyline2D>,
    pub area_mm2: f64,
}

impl PlanarVectorRegion {
    pub fn contains_point(&self, pt: DVec2) -> bool {
        chip_geom2d::region_contains_point(&self.outer_ccw, &self.islands_cw, pt)
    }
}

/// Classification of non-fatal warnings/notes produced during DXF/SVG import & gap-healing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SketchDiagnosticKind {
    HealedEndpointGap,
    DuplicateSegmentRemoved,
    OpenChainRemaining,
    DegenerateSkipped,
}

/// Diagnostic note produced during 2D vector import & topology healing (`11-sketches-vectors.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SketchImportDiagnostic {
    pub kind: SketchDiagnosticKind,
    pub location_xy: Option<DVec2>,
    pub message: String,
}

/// Planar 2D Sketch Layer stored in `.chip.toml` (`schema_version = 2`, `11-sketches-vectors.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SketchLayer {
    pub id: String,
    pub name: String,
    #[serde(default = "default_true")]
    pub visible: bool,
    #[serde(default)]
    pub anchor: SketchAnchor,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_asset: Option<VectorSourceAsset>,
    #[serde(default = "default_cut_depth")]
    pub cut_depth_default: ParamLength,
    #[serde(default)]
    pub entities: Vec<SketchEntityEntry>,
    #[serde(default)]
    pub regions: Vec<PlanarVectorRegion>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub diagnostics: Vec<SketchImportDiagnostic>,
}

impl SketchLayer {
    pub fn new(
        id: impl Into<String>,
        name: impl Into<String>,
        setup_id: impl Into<String>,
    ) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            visible: true,
            anchor: SketchAnchor::SetupXy {
                setup_id: setup_id.into(),
                plane_ref: SketchPlaneRef::StockTop,
                z_offset: ParamLength::from_mm(0.0),
            },
            source_asset: None,
            cut_depth_default: default_cut_depth(),
            entities: Vec::new(),
            regions: Vec::new(),
            diagnostics: Vec::new(),
        }
    }

    /// Rebuilds `self.regions` (`PlanarVectorRegion` hierarchy) from all non-construction closed
    /// curves, parametric primitives, and TrueType outline text entities on this layer.
    pub fn rebuild_regions(&mut self, custom_fonts: Option<&CustomFontMap>) {
        let mut candidates: Vec<(Option<String>, ArcPolyline2D)> = Vec::new();
        for ent in &self.entities {
            if ent.is_construction {
                continue;
            }
            match &ent.entity {
                SketchEntity::Text(txt) => {
                    // Preserve outline text on its own default sub-layer when ungrouped so glyphs
                    // inside an outer plaque remain directly pickable as recessed letter regions
                    // unless explicitly placed on the same named sub-layer!
                    let eff_layer = ent
                        .sub_layer
                        .clone()
                        .or_else(|| Some(format!("__text_{}", ent.id)));
                    for reg in txt.evaluate_outline_regions(eff_layer.as_deref(), custom_fonts) {
                        candidates.push((eff_layer.clone(), reg.outer_ccw));
                        for isl in reg.islands_cw {
                            candidates.push((eff_layer.clone(), isl));
                        }
                    }
                }
                other => {
                    for poly in other.to_arc_polylines(custom_fonts) {
                        if poly.closed && poly.segments.len() >= 2 {
                            candidates.push((ent.sub_layer.clone(), poly));
                        }
                    }
                }
            }
        }
        self.regions = classify_planar_regions_from_candidates(candidates);
    }

    /// Computes the tight 2D bounding box (`mm`) of all non-construction entities and regions on this layer.
    pub fn exact_aabb(&self) -> Aabb2 {
        let mut aabb = Aabb2::EMPTY;
        for ent in &self.entities {
            if ent.is_construction {
                continue;
            }
            for poly in ent.entity.to_arc_polylines(None) {
                let b = poly.exact_aabb();
                if b.is_valid() {
                    aabb = aabb.union(&b);
                }
            }
        }
        for reg in &self.regions {
            let b = reg.outer_ccw.exact_aabb();
            if b.is_valid() {
                aabb = aabb.union(&b);
            }
        }
        aabb
    }

    /// Returns distinct sorted sub-layer names present in `self.entities`.
    pub fn sub_layer_names(&self) -> Vec<String> {
        let mut names: Vec<String> = self
            .entities
            .iter()
            .filter_map(|e| e.sub_layer.clone())
            .collect();
        names.sort();
        names.dedup();
        names
    }

    pub fn circle_count(&self) -> usize {
        self.entities
            .iter()
            .filter(|e| !e.is_construction)
            .map(|e| match &e.entity {
                SketchEntity::Circle { .. } => 1,
                SketchEntity::PointPattern(pat) => pat.evaluate_holes().len(),
                _ => 0,
            })
            .sum()
    }

    pub fn closed_loop_count(&self) -> usize {
        self.entities
            .iter()
            .filter(|e| !e.is_construction)
            .map(|e| match &e.entity {
                SketchEntity::Rectangle { .. }
                | SketchEntity::Slot { .. }
                | SketchEntity::RegularPolygon { .. } => 1,
                SketchEntity::Polyline { polyline, .. } if polyline.closed => 1,
                _ => 0,
            })
            .sum()
    }

    pub fn open_chain_count(&self) -> usize {
        self.entities
            .iter()
            .filter(|e| !e.is_construction)
            .map(|e| match &e.entity {
                SketchEntity::Polyline { polyline, .. } if !polyline.closed => 1,
                SketchEntity::Text(txt) if txt.font.is_single_line() => txt
                    .evaluate_polylines(None)
                    .iter()
                    .filter(|p| !p.closed)
                    .count(),
                _ => 0,
            })
            .sum()
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.anchor.z_offset_mut().reevaluate(ctx);
        let _ = self.cut_depth_default.reevaluate(ctx);
        for ent in &mut self.entities {
            ent.entity.reevaluate(ctx);
        }
    }
}

/// Creates a default parametric [`SketchEntityEntry`] for quick-insert UI & CLI presets:
/// `spec`: `0` = Rectangle, `1` = Circle, `2` = Slot, `3` = Hexagon, `4` = Bolt Circle, `_` = Text.
pub fn create_default_sketch_entity(
    n: usize,
    spec: u8,
    sub_layer: Option<&str>,
) -> SketchEntityEntry {
    let mm = ParamLength::from_mm;
    let (p, title) = match spec {
        0 => ("rect", "Rectangle"),
        1 => ("circle", "Circle"),
        2 => ("slot", "Slot"),
        3 => ("poly", "Hexagon"),
        4 => ("bolts", "Bolt Circle"),
        _ => ("text", "Text"),
    };
    let entity = match spec {
        0 => SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: mm(0.0),
            y: mm(0.0),
            width: mm(50.0),
            height: mm(30.0),
            corner_radius: mm(4.0),
            rotation_deg: 0.0,
        },
        1 => SketchEntity::new_circle(mm(0.0), mm(0.0), mm(12.0)),
        2 => SketchEntity::Slot {
            mode: SlotLengthMode::CenterToCenter,
            x: mm(0.0),
            y: mm(0.0),
            length: mm(24.0),
            width: mm(8.0),
            angle_deg: 0.0,
        },
        3 => SketchEntity::RegularPolygon {
            center_x: mm(0.0),
            center_y: mm(0.0),
            sides: 6,
            sizing: PolygonSizingMode::AcrossFlats,
            diameter: mm(19.0),
            corner_radius: mm(0.0),
            rotation_deg: 0.0,
        },
        4 => SketchEntity::PointPattern(PointPatternSpec::BoltCircle {
            center_x: mm(0.0),
            center_y: mm(0.0),
            bolt_circle_diameter: mm(40.0),
            hole_diameter: mm(6.0),
            count: 6,
            start_angle_deg: 0.0,
        }),
        _ => SketchEntity::Text(SketchTextSpec::default()),
    };
    SketchEntityEntry {
        id: format!("{p}_{n}"),
        name: format!("{title} {n}"),
        sub_layer: sub_layer.map(str::to_string),
        is_construction: false,
        entity,
    }
}

/// Computes lowercase `64`-character SHA-256 hex digest of `bytes`.
pub fn compute_sketch_sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut out = String::with_capacity(64);
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for &b in digest.as_slice() {
        out.push(HEX[(b >> 4) as usize] as char);
        out.push(HEX[(b & 0x0f) as usize] as char);
    }
    out
}
