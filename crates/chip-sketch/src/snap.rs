//! UI-agnostic 2D Snapping Engine (`Origin`, `Endpoint`, `Midpoint`, `Center`, `Stock`,
//! `CadPoint`, and `Grid`; `11-sketches-vectors.md`).

use chip_math::{Aabb2, ArcSegment2D, DVec2};

use crate::model::SketchLayer;

/// Classification of the snapped 2D feature (`11-sketches-vectors.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnapKind {
    Origin,
    Endpoint,
    Midpoint,
    Center,
    StockCorner,
    StockCenter,
    CadFeature,
    Grid,
}

impl SnapKind {
    pub const fn badge(self) -> &'static str {
        match self {
            Self::Origin => "Origin",
            Self::Endpoint => "End",
            Self::Midpoint => "Mid",
            Self::Center => "Center",
            Self::StockCorner => "Stock Corner",
            Self::StockCenter => "Stock Center",
            Self::CadFeature => "B-Rep",
            Self::Grid => "Grid",
        }
    }
}

/// Snapped 2D coordinate returned by [`snap_sketch_point_2d`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SketchSnapResult {
    pub point_xy: DVec2,
    pub kind: Option<SnapKind>,
    pub distance_mm: f64,
}

/// Candidate geometric context passed into [`snap_sketch_point_2d`].
pub struct SketchSnapContext<'a> {
    pub sketch: &'a SketchLayer,
    pub stock_xy_bounds: Option<Aabb2>,
    pub projected_cad_points: &'a [DVec2],
    pub snap_radius_mm: f64,
    pub grid_step_mm: Option<f64>,
}

/// Snaps `query_xy` to the closest geometric feature within `ctx.snap_radius_mm`, falling back
/// to `ctx.grid_step_mm` (if enabled) or returning raw `query_xy`.
pub fn snap_sketch_point_2d(query_xy: DVec2, ctx: &SketchSnapContext<'_>) -> SketchSnapResult {
    if !query_xy.is_finite() {
        return SketchSnapResult {
            point_xy: DVec2::ZERO,
            kind: None,
            distance_mm: 0.0,
        };
    }

    let rad = ctx.snap_radius_mm.max(0.0);
    let mut best_geom: Option<(DVec2, SnapKind, f64)> = None;

    let mut consider = |pt: DVec2, kind: SnapKind| {
        if !pt.is_finite() {
            return;
        }
        let d = pt.distance(query_xy);
        if d <= rad && best_geom.is_none_or(|(_, _, prev_d)| d < prev_d - 1e-6) {
            best_geom = Some((pt, kind, d));
        }
    };

    // 1. Sketch Origin (0, 0)
    consider(DVec2::ZERO, SnapKind::Origin);

    // 2. Stock corners & center projected into Sketch XY
    if let Some(sb) = ctx.stock_xy_bounds
        && sb.is_valid()
    {
        consider(sb.center(), SnapKind::StockCenter);
        for corner in [
            sb.min,
            DVec2::new(sb.max.x, sb.min.y),
            sb.max,
            DVec2::new(sb.min.x, sb.max.y),
        ] {
            consider(corner, SnapKind::StockCorner);
        }
    }

    // 3. Projected 3D STEP B-Rep vertices / hole centers
    for &cp in ctx.projected_cad_points {
        consider(cp, SnapKind::CadFeature);
    }

    // 4. Existing Sketch entities (centers, endpoints, midpoints)
    for entry in &ctx.sketch.entities {
        for c in entry.entity.snap_centers_2d() {
            consider(c, SnapKind::Center);
        }
        for poly in entry.entity.to_arc_polylines(None) {
            for seg in &poly.segments {
                consider(seg.start(), SnapKind::Endpoint);
                consider(seg.end(), SnapKind::Endpoint);
                if let Some((mid, _)) = seg.eval_at(0.5) {
                    consider(mid, SnapKind::Midpoint);
                }
                if let ArcSegment2D::Arc { center, .. } = *seg {
                    consider(center, SnapKind::Center);
                }
            }
        }
    }

    if let Some((pt, kind, d)) = best_geom {
        return SketchSnapResult {
            point_xy: pt,
            kind: Some(kind),
            distance_mm: d,
        };
    }

    if let Some(step) = ctx.grid_step_mm
        && step.is_finite()
        && step > 1e-4
    {
        let gx = (query_xy.x / step).round() * step;
        let gy = (query_xy.y / step).round() * step;
        let gpt = DVec2::new(gx, gy);
        return SketchSnapResult {
            point_xy: gpt,
            kind: Some(SnapKind::Grid),
            distance_mm: gpt.distance(query_xy),
        };
    }

    SketchSnapResult {
        point_xy: query_xy,
        kind: None,
        distance_mm: 0.0,
    }
}
