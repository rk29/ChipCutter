use chip_math::{
    Aabb2, ArcPolyline2D, ArcSegment2D, DEFAULT_ARC_FIT_TOL_MM, DVec2, DisplayUnit, EvalContext,
    ParamLength,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

fn default_true() -> bool {
    true
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

fn default_zero_length() -> ParamLength {
    ParamLength::from_mm(0.0)
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
}

impl Default for SketchAnchor {
    fn default() -> Self {
        Self::SetupXy {
            setup_id: "setup_1".to_string(),
            plane_ref: SketchPlaneRef::StockTop,
            z_offset: ParamLength::from_mm(0.0),
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
    Polyline {
        polyline: ArcPolyline2D,
    },
    Circle {
        center_xy: DVec2,
        diameter: ParamLength,
    },
}

impl SketchEntity {
    /// Evaluates this entity to a canonical `f64` millimeter [`ArcPolyline2D`].
    pub fn to_arc_polyline(&self) -> ArcPolyline2D {
        match self {
            Self::Polyline { polyline } => polyline.clone(),
            Self::Circle {
                center_xy,
                diameter,
            } => {
                let r = (0.5 * diameter.value_mm).max(0.0);
                if r <= 1e-6 || !center_xy.is_finite() {
                    return ArcPolyline2D::default();
                }
                let p_r = *center_xy + DVec2::new(r, 0.0);
                let p_l = *center_xy - DVec2::new(r, 0.0);
                ArcPolyline2D {
                    segments: vec![
                        ArcSegment2D::Arc {
                            start: p_r,
                            end: p_l,
                            center: *center_xy,
                            radius: r,
                            ccw: true,
                        },
                        ArcSegment2D::Arc {
                            start: p_l,
                            end: p_r,
                            center: *center_xy,
                            radius: r,
                            ccw: true,
                        },
                    ],
                    closed: true,
                }
            }
        }
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        if let Self::Circle { diameter, .. } = self {
            let _ = diameter.reevaluate(ctx);
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
        if !self.outer_ccw.contains_point(pt) {
            return false;
        }
        for isl in &self.islands_cw {
            if isl.contains_point(pt) && isl.distance_to_point(pt) > 1e-6 {
                return false;
            }
        }
        true
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

    /// Computes the tight 2D bounding box (`mm`) of all non-construction entities and regions on this layer.
    pub fn exact_aabb(&self) -> Aabb2 {
        let mut aabb = Aabb2::EMPTY;
        for ent in &self.entities {
            if ent.is_construction {
                continue;
            }
            let poly = ent.entity.to_arc_polyline();
            let b = poly.exact_aabb();
            if b.is_valid() {
                aabb = aabb.union(&b);
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
            .filter(|e| !e.is_construction && matches!(e.entity, SketchEntity::Circle { .. }))
            .count()
    }

    pub fn closed_loop_count(&self) -> usize {
        self.entities
            .iter()
            .filter(|e| {
                !e.is_construction
                    && matches!(&e.entity, SketchEntity::Polyline { polyline } if polyline.closed)
            })
            .count()
    }

    pub fn open_chain_count(&self) -> usize {
        self.entities
            .iter()
            .filter(|e| {
                !e.is_construction
                    && matches!(&e.entity, SketchEntity::Polyline { polyline } if !polyline.closed)
            })
            .count()
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        match &mut self.anchor {
            SketchAnchor::SetupXy { z_offset, .. } => {
                let _ = z_offset.reevaluate(ctx);
            }
        }
        let _ = self.cut_depth_default.reevaluate(ctx);
        for ent in &mut self.entities {
            ent.entity.reevaluate(ctx);
        }
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
