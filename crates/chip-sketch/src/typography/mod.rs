//! Dual-Mode CAM Typography (`SketchTextSpec`): TrueType / OpenType closed-outline fonts (`skrifa`)
//! for V-Carving, Pocketing, and Profiling alongside built-in single-line Hershey engraving fonts
//! for 1-pass scribed part numbers (`11-sketches-vectors.md`).

pub mod hershey;
pub mod ttf_outline;

use chip_math::{ArcPolyline2D, DEFAULT_ARC_FIT_TOL_MM, DVec2, EvalContext, ParamLength};
use serde::{Deserialize, Serialize};

pub use hershey::{HersheyFontKind, HersheyLayoutParams, evaluate_hershey_strokes};
pub use ttf_outline::{
    BUILTIN_DEJAVU_SANS_BOLD_TTF, CustomFontMap, TtfOutlineParams, evaluate_ttf_outline_regions,
};

use crate::model::PlanarVectorRegion;

fn default_line_spacing() -> f64 {
    1.35
}

fn default_text_arc_tol() -> f64 {
    0.008_f64.max(0.5 * DEFAULT_ARC_FIT_TOL_MM)
}

fn default_zero_len() -> ParamLength {
    ParamLength::from_mm(0.0)
}

/// Horizontal text alignment relative to `(x, y)` (`11-sketches-vectors.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TextHAlign {
    #[default]
    Left,
    Center,
    Right,
}

impl TextHAlign {
    pub const ALL: [Self; 3] = [Self::Left, Self::Center, Self::Right];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Left => "Left",
            Self::Center => "Center",
            Self::Right => "Right",
        }
    }
}

/// Vertical text alignment relative to `(x, y)` (`11-sketches-vectors.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TextVAlign {
    #[default]
    Baseline,
    Middle,
    Top,
}

impl TextVAlign {
    pub const ALL: [Self; 3] = [Self::Baseline, Self::Middle, Self::Top];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Baseline => "Baseline",
            Self::Middle => "Middle",
            Self::Top => "Top",
        }
    }
}

/// Font source for a [`SketchTextSpec`] (`11-sketches-vectors.md`).
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SketchFontSource {
    /// Bundled permissive bold sans-serif outline font (`DejaVuSans-Bold.ttf`).
    #[default]
    BuiltinSansBold,
    /// Project-referenced custom `.ttf` or `.otf` font file (bundled under `fonts/` in `.chipz`).
    CustomFile {
        path: String,
        #[serde(default)]
        sha256: String,
        #[serde(default)]
        face_index: u32,
    },
    /// Built-in public-domain single-line Hershey engraving stroke font.
    Hershey {
        #[serde(default)]
        font: HersheyFontKind,
    },
}

impl SketchFontSource {
    pub const fn is_single_line(&self) -> bool {
        matches!(self, Self::Hershey { .. })
    }

    pub fn label(&self) -> String {
        match self {
            Self::BuiltinSansBold => "DejaVu Sans Bold (Outline)".to_string(),
            Self::CustomFile { path, .. } => format!("Custom Font ({path})"),
            Self::Hershey { font } => font.label().to_string(),
        }
    }
}

/// Parametric 2D CAM Text specification (`11-sketches-vectors.md` §CAM Typography).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SketchTextSpec {
    pub content: String,
    #[serde(default)]
    pub font: SketchFontSource,
    pub cap_height: ParamLength,
    #[serde(default = "default_zero_len")]
    pub x: ParamLength,
    #[serde(default = "default_zero_len")]
    pub y: ParamLength,
    #[serde(default)]
    pub h_align: TextHAlign,
    #[serde(default)]
    pub v_align: TextVAlign,
    #[serde(default = "default_zero_len")]
    pub letter_spacing: ParamLength,
    #[serde(default = "default_line_spacing")]
    pub line_spacing_factor: f64,
    #[serde(default)]
    pub rotation_deg: f64,
    #[serde(default)]
    pub mirror_x: bool,
    #[serde(default = "default_text_arc_tol")]
    pub arc_fit_tolerance_mm: f64,
}

impl Default for SketchTextSpec {
    fn default() -> Self {
        Self {
            content: "CHIPCUTTER".to_string(),
            font: SketchFontSource::BuiltinSansBold,
            cap_height: ParamLength::from_mm(12.0),
            x: ParamLength::from_mm(0.0),
            y: ParamLength::from_mm(0.0),
            h_align: TextHAlign::Center,
            v_align: TextVAlign::Middle,
            letter_spacing: ParamLength::from_mm(0.0),
            line_spacing_factor: 1.35,
            rotation_deg: 0.0,
            mirror_x: false,
            arc_fit_tolerance_mm: default_text_arc_tol(),
        }
    }
}

impl SketchTextSpec {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.cap_height.reevaluate(ctx);
        let _ = self.x.reevaluate(ctx);
        let _ = self.y.reevaluate(ctx);
        let _ = self.letter_spacing.reevaluate(ctx);
    }

    pub fn origin_mm(&self) -> DVec2 {
        DVec2::new(self.x.value_mm, self.y.value_mm)
    }

    fn resolve_ttf_bytes<'a>(&self, custom_fonts: Option<&'a CustomFontMap>) -> (&'a [u8], u32) {
        match &self.font {
            SketchFontSource::BuiltinSansBold => (BUILTIN_DEJAVU_SANS_BOLD_TTF, 0),
            SketchFontSource::CustomFile {
                path,
                sha256,
                face_index,
            } => {
                if let Some(map) = custom_fonts
                    && let Some(buf) = map.get(path).or_else(|| map.get(sha256))
                {
                    return (buf.as_slice(), *face_index);
                }
                (BUILTIN_DEJAVU_SANS_BOLD_TTF, 0)
            }
            SketchFontSource::Hershey { .. } => (BUILTIN_DEJAVU_SANS_BOLD_TTF, 0),
        }
    }

    /// Evaluates closed [`PlanarVectorRegion`]s (outer glyph contours + `CW` counter-islands)
    /// when `self.font` is an **Outline** font; returns an empty vec for Hershey single-line fonts.
    pub fn evaluate_outline_regions(
        &self,
        sub_layer: Option<&str>,
        custom_fonts: Option<&CustomFontMap>,
    ) -> Vec<PlanarVectorRegion> {
        if self.font.is_single_line() {
            return Vec::new();
        }
        // Support direct disk read when `CustomFile` is used in headless CLI without pre-populated map
        let owned_disk: Option<Vec<u8>> = match &self.font {
            SketchFontSource::CustomFile { path, sha256, .. } => {
                let in_map =
                    custom_fonts.is_some_and(|m| m.contains_key(path) || m.contains_key(sha256));
                if !in_map {
                    std::fs::read(path).ok()
                } else {
                    None
                }
            }
            _ => None,
        };
        let (font_bytes, face_index) = match (&owned_disk, &self.font) {
            (Some(b), SketchFontSource::CustomFile { face_index, .. }) => {
                (b.as_slice(), *face_index)
            }
            _ => self.resolve_ttf_bytes(custom_fonts),
        };

        evaluate_ttf_outline_regions(&TtfOutlineParams {
            font_bytes,
            face_index,
            content: &self.content,
            cap_height_mm: self.cap_height.value_mm,
            origin_xy: self.origin_mm(),
            h_align: self.h_align,
            v_align: self.v_align,
            letter_spacing_mm: self.letter_spacing.value_mm,
            line_spacing_factor: self.line_spacing_factor,
            rotation_deg: self.rotation_deg,
            mirror_x: self.mirror_x,
            arc_fit_tolerance_mm: self.arc_fit_tolerance_mm,
            sub_layer,
        })
    }

    /// Evaluates all displayable & pickable [`ArcPolyline2D`]s for this text item:
    /// - For **Hershey** fonts: single-pass centerline stroke chains.
    /// - For **Outline** fonts: all outer `CCW` + counter-island `CW` glyph loops.
    pub fn evaluate_polylines(&self, custom_fonts: Option<&CustomFontMap>) -> Vec<ArcPolyline2D> {
        match &self.font {
            SketchFontSource::Hershey { font } => evaluate_hershey_strokes(&HersheyLayoutParams {
                content: &self.content,
                font: *font,
                cap_height_mm: self.cap_height.value_mm,
                origin_xy: self.origin_mm(),
                h_align: self.h_align,
                v_align: self.v_align,
                letter_spacing_mm: self.letter_spacing.value_mm,
                line_spacing_factor: self.line_spacing_factor,
                rotation_deg: self.rotation_deg,
                mirror_x: self.mirror_x,
            }),
            SketchFontSource::BuiltinSansBold | SketchFontSource::CustomFile { .. } => {
                let regions = self.evaluate_outline_regions(None, custom_fonts);
                let mut polys = Vec::with_capacity(regions.len() * 2);
                for r in regions {
                    polys.push(r.outer_ccw);
                    polys.extend(r.islands_cw);
                }
                polys
            }
        }
    }
}
