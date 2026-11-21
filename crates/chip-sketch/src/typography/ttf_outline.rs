//! TrueType / OpenType (`skrifa`) outline font layout, `kern` pair kerning, Bézier-to-biarc
//! circular arc fitting, `cavalier_contours` boolean glyph welding, and counter-island
//! `PlanarVectorRegion` extraction (`11-sketches-vectors.md` §CAM Typography).

use std::{collections::HashMap, sync::Arc};

use chip_geom2d::try_union_overlapping_ccw_loops;
use chip_math::{ArcPolyline2D, DVec2};
use kurbo::{BezPath, Point as KPoint};
use skrifa::{
    FontRef, GlyphId, MetadataProvider,
    instance::{LocationRef, Size},
    outline::{DrawSettings, OutlinePen},
    raw::TableProvider,
};

use crate::{
    curve_fit::fit_bezpath_to_arc_polylines,
    heal_chain::classify_planar_regions_from_candidates,
    model::PlanarVectorRegion,
    primitives::transform_arc_polyline_2d,
    typography::{TextHAlign, TextVAlign},
};

/// Permissively licensed embedded default proportional bold sans-serif font (`DejaVuSans-Bold.ttf`).
pub const BUILTIN_DEJAVU_SANS_BOLD_TTF: &[u8] = include_bytes!("../../assets/DejaVuSans-Bold.ttf");

/// Optional lookup table of project-loaded custom `.ttf`/`.otf` byte buffers keyed by `path` or `sha256`.
pub type CustomFontMap = HashMap<String, Arc<Vec<u8>>>;

/// Layout configuration passed to [`evaluate_ttf_outline_regions`].
#[derive(Debug, Clone, Copy)]
pub struct TtfOutlineParams<'a> {
    pub font_bytes: &'a [u8],
    pub face_index: u32,
    pub content: &'a str,
    pub cap_height_mm: f64,
    pub origin_xy: DVec2,
    pub h_align: TextHAlign,
    pub v_align: TextVAlign,
    pub letter_spacing_mm: f64,
    pub line_spacing_factor: f64,
    pub rotation_deg: f64,
    pub mirror_x: bool,
    pub arc_fit_tolerance_mm: f64,
    pub sub_layer: Option<&'a str>,
}

struct KurboPen {
    path: BezPath,
    offset_x: f64,
    offset_y: f64,
    scale: f64,
    mirror_x: bool,
}

impl KurboPen {
    fn map(&self, x: f32, y: f32) -> KPoint {
        let px = self.offset_x + f64::from(x) * self.scale;
        let py = self.offset_y + f64::from(y) * self.scale;
        KPoint::new(if self.mirror_x { -px } else { px }, py)
    }
}

impl OutlinePen for KurboPen {
    fn move_to(&mut self, x: f32, y: f32) {
        self.path.move_to(self.map(x, y));
    }

    fn line_to(&mut self, x: f32, y: f32) {
        self.path.line_to(self.map(x, y));
    }

    fn quad_to(&mut self, cx0: f32, cy0: f32, x: f32, y: f32) {
        self.path.quad_to(self.map(cx0, cy0), self.map(x, y));
    }

    fn curve_to(&mut self, cx0: f32, cy0: f32, cx1: f32, cy1: f32, x: f32, y: f32) {
        self.path
            .curve_to(self.map(cx0, cy0), self.map(cx1, cy1), self.map(x, y));
    }

    fn close(&mut self) {
        self.path.close_path();
    }
}

/// Evaluates TrueType / OpenType outline glyphs into welded, biarc-fitted [`PlanarVectorRegion`]s
/// (outer `CCW` contours paired with direct `CW` counter-islands inside `'A'`, `'B'`, `'O'`, `'8'`, `'e'`, etc.).
pub fn evaluate_ttf_outline_regions(params: &TtfOutlineParams<'_>) -> Vec<PlanarVectorRegion> {
    let cap_mm = params.cap_height_mm.max(0.2);
    if !cap_mm.is_finite() || params.content.is_empty() {
        return Vec::new();
    }
    let Ok(font) = FontRef::from_index(params.font_bytes, params.face_index) else {
        return Vec::new();
    };

    let loc = LocationRef::default();
    let global_metrics = font.metrics(Size::unscaled(), loc);
    let upem = f64::from(global_metrics.units_per_em.max(1));
    let charmap = font.charmap();
    let g_metrics = font.glyph_metrics(Size::unscaled(), loc);
    let outlines = font.outline_glyphs();

    let raw_cap_units = global_metrics
        .cap_height
        .map(f64::from)
        .filter(|&c| c > 1.0)
        .or_else(|| measure_glyph_cap_height(&charmap, &outlines, loc))
        .unwrap_or_else(|| (f64::from(global_metrics.ascent) * 0.78).max(upem * 0.70));

    let unit_scale = cap_mm / raw_cap_units;
    let line_step_mm = cap_mm * params.line_spacing_factor.clamp(0.8, 4.0);
    let arc_tol = params.arc_fit_tolerance_mm.clamp(0.002, 0.25);

    let lines: Vec<&str> = params.content.split('\n').collect();
    let n_lines = lines.len();

    // Resolve glyph sequences + kerning advances per line
    let mut positioned_lines: Vec<(f64, Vec<(GlyphId, f64)>)> = Vec::with_capacity(n_lines);
    for line in &lines {
        let gids: Vec<GlyphId> = line
            .chars()
            .map(|ch| charmap.map(ch).unwrap_or(GlyphId::NOTDEF))
            .collect();
        let mut cursor_mm = 0.0;
        let mut placed = Vec::with_capacity(gids.len());
        for (i, &gid) in gids.iter().enumerate() {
            if i > 0 {
                let kern_u = lookup_kern_pair(&font, gids[i - 1], gid);
                cursor_mm += kern_u * unit_scale + params.letter_spacing_mm;
            }
            placed.push((gid, cursor_mm));
            let adv_u = f64::from(g_metrics.advance_width(gid).unwrap_or(0.0));
            cursor_mm += adv_u * unit_scale;
        }
        positioned_lines.push((cursor_mm.max(0.0), placed));
    }

    let total_block_h = cap_mm + ((n_lines.saturating_sub(1)) as f64) * line_step_mm;
    let first_baseline_y = match params.v_align {
        TextVAlign::Baseline => 0.0,
        TextVAlign::Top => -cap_mm,
        TextVAlign::Middle => 0.5 * total_block_h - cap_mm,
    };

    // Extract per-glyph regions first, then union any overlapping adjacent glyph outer contours!
    let mut glyph_regions: Vec<PlanarVectorRegion> = Vec::new();
    let sub_layer_owned = params.sub_layer.map(str::to_string);

    for (li, (lw, placed)) in positioned_lines.iter().enumerate() {
        let start_x = match params.h_align {
            TextHAlign::Left => 0.0,
            TextHAlign::Center => -0.5 * *lw,
            TextHAlign::Right => -*lw,
        };
        let baseline_y = first_baseline_y - (li as f64) * line_step_mm;

        for &(gid, rel_x) in placed {
            let Some(outline) = outlines.get(gid) else {
                continue;
            };
            let mut pen = KurboPen {
                path: BezPath::new(),
                offset_x: start_x + rel_x,
                offset_y: baseline_y,
                scale: unit_scale,
                mirror_x: params.mirror_x,
            };
            if outline
                .draw(DrawSettings::unhinted(Size::unscaled(), loc), &mut pen)
                .is_err()
            {
                continue;
            }

            let raw_polys: Vec<(Option<String>, ArcPolyline2D)> =
                fit_bezpath_to_arc_polylines(&pen.path, arc_tol)
                    .into_iter()
                    .filter(|p| p.closed && p.segments.len() >= 2)
                    .map(|p| (sub_layer_owned.clone(), p))
                    .collect();

            if raw_polys.is_empty() {
                continue;
            }
            let per_glyph = classify_planar_regions_from_candidates(raw_polys);
            let welded = weld_intra_glyph_regions(per_glyph, sub_layer_owned.as_deref());
            glyph_regions.extend(welded);
        }
    }

    // Weld overlapping inter-glyph outer contours (e.g. tightly kerned pairs or touching script glyphs)
    let welded_all = weld_inter_glyph_overlaps(glyph_regions, sub_layer_owned.as_deref());

    // Finally apply `origin_xy` + `rotation_deg` and assign deterministic IDs
    welded_all
        .into_iter()
        .enumerate()
        .map(|(idx, mut reg)| {
            reg.id = format!("text_region_{}", idx + 1);
            reg.outer_ccw = transform_arc_polyline_2d(
                &reg.outer_ccw,
                params.origin_xy,
                1.0,
                params.rotation_deg,
            )
            .ensure_orientation(true);
            reg.islands_cw = reg
                .islands_cw
                .into_iter()
                .map(|isl| {
                    transform_arc_polyline_2d(&isl, params.origin_xy, 1.0, params.rotation_deg)
                        .ensure_orientation(false)
                })
                .collect();
            reg
        })
        .collect()
}

fn measure_glyph_cap_height(
    charmap: &skrifa::charmap::Charmap<'_>,
    outlines: &skrifa::outline::OutlineGlyphCollection<'_>,
    loc: LocationRef<'_>,
) -> Option<f64> {
    for probe in ['H', 'I', 'E', 'T'] {
        let Some(gid) = charmap.map(probe) else {
            continue;
        };
        let Some(outline) = outlines.get(gid) else {
            continue;
        };
        let mut pen = KurboPen {
            path: BezPath::new(),
            offset_x: 0.0,
            offset_y: 0.0,
            scale: 1.0,
            mirror_x: false,
        };
        if outline
            .draw(DrawSettings::unhinted(Size::unscaled(), loc), &mut pen)
            .is_ok()
        {
            let bbox = kurbo::Shape::bounding_box(&pen.path);
            if bbox.y1 > 1.0 {
                return Some(bbox.y1);
            }
        }
    }
    None
}

fn lookup_kern_pair(font: &FontRef<'_>, left: GlyphId, right: GlyphId) -> f64 {
    if let Ok(kern) = font.kern() {
        let mut total = 0_i32;
        for sub in kern.subtables() {
            if let Ok(st) = sub
                && st.is_horizontal()
                && !st.is_cross_stream()
                && let Ok(kind) = st.kind()
            {
                match kind {
                    skrifa::raw::tables::kern::SubtableKind::Format0(f0) => {
                        if let Some(v) = f0.kerning(left, right) {
                            total += v;
                        }
                    }
                    skrifa::raw::tables::kern::SubtableKind::Format2(f2) => {
                        if let Some(v) = f2.kerning(left, right) {
                            total += v;
                        }
                    }
                    _ => {}
                }
            }
        }
        if total != 0 {
            return f64::from(total);
        }
    }
    0.0
}

fn weld_intra_glyph_regions(
    regions: Vec<PlanarVectorRegion>,
    sub_layer: Option<&str>,
) -> Vec<PlanarVectorRegion> {
    if regions.len() <= 1 {
        return regions;
    }
    weld_inter_glyph_overlaps(regions, sub_layer)
}

fn weld_inter_glyph_overlaps(
    regions: Vec<PlanarVectorRegion>,
    sub_layer: Option<&str>,
) -> Vec<PlanarVectorRegion> {
    if regions.len() <= 1 {
        return regions;
    }

    let mut acc: Vec<PlanarVectorRegion> = Vec::with_capacity(regions.len());
    for reg in regions {
        let r_aabb = reg.outer_ccw.exact_aabb();
        let mut merged_into_idx: Option<usize> = None;

        for (i, existing) in acc.iter().enumerate() {
            if !existing.outer_ccw.exact_aabb().overlaps(&r_aabb) {
                continue;
            }
            // If the two outer contours genuinely overlapped and welded into a single loop, rebuild region!
            if let Some((new_outer, neg_holes)) =
                try_union_overlapping_ccw_loops(&existing.outer_ccw, &reg.outer_ccw)
            {
                let mut candidates: Vec<(Option<String>, ArcPolyline2D)> =
                    vec![(sub_layer.map(str::to_string), new_outer)];
                for hole in neg_holes {
                    candidates.push((sub_layer.map(str::to_string), hole));
                }
                for isl in &existing.islands_cw {
                    candidates.push((sub_layer.map(str::to_string), isl.clone()));
                }
                for isl in &reg.islands_cw {
                    candidates.push((sub_layer.map(str::to_string), isl.clone()));
                }
                let rebuilt = classify_planar_regions_from_candidates(candidates);
                if let Some(first) = rebuilt.into_iter().next() {
                    acc[i] = first;
                    merged_into_idx = Some(i);
                    break;
                }
            }
        }

        if merged_into_idx.is_none() {
            acc.push(reg);
        }
    }
    acc
}
