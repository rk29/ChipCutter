//! Parametric 2D CAM primitive shapes (`Rectangle`, `Circle`, `Slot`, `RegularPolygon`,
//! `Polyline` transform, and `PointPattern`) evaluating to exact [`ArcPolyline2D`]s and hole lists.

use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, EvalContext, ParamLength};
use serde::{Deserialize, Serialize};

/// Anchor corner/center mode for [`crate::model::SketchEntity::Rectangle`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RectAnchorMode {
    #[default]
    Center,
    FrontLeft,
}

impl RectAnchorMode {
    pub const ALL: [Self; 2] = [Self::Center, Self::FrontLeft];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Center => "Center (X, Y)",
            Self::FrontLeft => "Front-Left Corner (X, Y)",
        }
    }
}

/// Length measurement convention for [`crate::model::SketchEntity::Slot`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SlotLengthMode {
    #[default]
    CenterToCenter,
    OverallLength,
}

impl SlotLengthMode {
    pub const ALL: [Self; 2] = [Self::CenterToCenter, Self::OverallLength];

    pub const fn label(self) -> &'static str {
        match self {
            Self::CenterToCenter => "Center-to-Center",
            Self::OverallLength => "Overall Length",
        }
    }
}

/// Sizing convention for [`crate::model::SketchEntity::RegularPolygon`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PolygonSizingMode {
    InscribedDiameter,
    #[default]
    CircumscribedDiameter,
    AcrossFlats,
}

impl PolygonSizingMode {
    pub const ALL: [Self; 3] = [
        Self::CircumscribedDiameter,
        Self::AcrossFlats,
        Self::InscribedDiameter,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::InscribedDiameter => "Inscribed Circle Ø (Across Flats)",
            Self::CircumscribedDiameter => "Circumscribed Circle Ø (Across Corners)",
            Self::AcrossFlats => "Width Across Flats (Hex/Nut)",
        }
    }
}

/// Evaluated 2D hole target emitted by [`PointPatternSpec`] or [`crate::model::SketchEntity::Circle`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HolePoint2D {
    pub center_xy: DVec2,
    pub diameter_mm: f64,
}

/// Parametric hole/point pattern (`Single`, `LinearGrid`, `BoltCircle`; `11-sketches-vectors.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "pattern", rename_all = "snake_case")]
pub enum PointPatternSpec {
    Single {
        x: ParamLength,
        y: ParamLength,
        hole_diameter: ParamLength,
    },
    LinearGrid {
        origin_x: ParamLength,
        origin_y: ParamLength,
        count_x: u32,
        count_y: u32,
        spacing_x: ParamLength,
        spacing_y: ParamLength,
        #[serde(default)]
        angle_deg: f64,
        hole_diameter: ParamLength,
    },
    BoltCircle {
        center_x: ParamLength,
        center_y: ParamLength,
        bolt_circle_diameter: ParamLength,
        hole_diameter: ParamLength,
        count: u32,
        #[serde(default)]
        start_angle_deg: f64,
    },
}

impl PointPatternSpec {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        match self {
            Self::Single {
                x,
                y,
                hole_diameter,
            } => {
                let _ = x.reevaluate(ctx);
                let _ = y.reevaluate(ctx);
                let _ = hole_diameter.reevaluate(ctx);
            }
            Self::LinearGrid {
                origin_x,
                origin_y,
                spacing_x,
                spacing_y,
                hole_diameter,
                ..
            } => {
                for p in [origin_x, origin_y, spacing_x, spacing_y, hole_diameter] {
                    let _ = p.reevaluate(ctx);
                }
            }
            Self::BoltCircle {
                center_x,
                center_y,
                bolt_circle_diameter,
                hole_diameter,
                ..
            } => {
                for p in [center_x, center_y, bolt_circle_diameter, hole_diameter] {
                    let _ = p.reevaluate(ctx);
                }
            }
        }
    }

    pub fn evaluate_holes(&self) -> Vec<HolePoint2D> {
        match self {
            Self::Single {
                x,
                y,
                hole_diameter,
            } => {
                let d = hole_diameter.value_mm.max(0.1);
                vec![HolePoint2D {
                    center_xy: DVec2::new(x.value_mm, y.value_mm),
                    diameter_mm: d,
                }]
            }
            Self::LinearGrid {
                origin_x,
                origin_y,
                count_x,
                count_y,
                spacing_x,
                spacing_y,
                angle_deg,
                hole_diameter,
            } => {
                let nx = (*count_x).clamp(1, 256);
                let ny = (*count_y).clamp(1, 256);
                let sx = spacing_x.value_mm;
                let sy = spacing_y.value_mm;
                let d = hole_diameter.value_mm.max(0.1);
                let origin = DVec2::new(origin_x.value_mm, origin_y.value_mm);
                let (sin_a, cos_a) = angle_deg.to_radians().sin_cos();
                let ux = DVec2::new(cos_a, sin_a);
                let uy = DVec2::new(-sin_a, cos_a);
                let mut out = Vec::with_capacity((nx * ny) as usize);
                for iy in 0..ny {
                    for ix in 0..nx {
                        // Snake row order minimizes drill rapid travel!
                        let col = if iy % 2 == 0 { ix } else { nx - 1 - ix };
                        let pt = origin + ux * ((col as f64) * sx) + uy * ((iy as f64) * sy);
                        if pt.is_finite() {
                            out.push(HolePoint2D {
                                center_xy: pt,
                                diameter_mm: d,
                            });
                        }
                    }
                }
                out
            }
            Self::BoltCircle {
                center_x,
                center_y,
                bolt_circle_diameter,
                hole_diameter,
                count,
                start_angle_deg,
            } => {
                let n = (*count).clamp(1, 360);
                let bcr = 0.5 * bolt_circle_diameter.value_mm.max(0.0);
                let d = hole_diameter.value_mm.max(0.1);
                let center = DVec2::new(center_x.value_mm, center_y.value_mm);
                let a0 = start_angle_deg.to_radians();
                let step = std::f64::consts::TAU / (n as f64);
                let mut out = Vec::with_capacity(n as usize);
                for i in 0..n {
                    let ang = a0 + (i as f64) * step;
                    let pt = center + DVec2::new(ang.cos(), ang.sin()) * bcr;
                    if pt.is_finite() {
                        out.push(HolePoint2D {
                            center_xy: pt,
                            diameter_mm: d,
                        });
                    }
                }
                out
            }
        }
    }
}

/// Builds a closed `CCW` 2-arc [`ArcPolyline2D`] for a circle of `diameter_mm` at `center_xy`.
pub fn build_circle_polyline(center_xy: DVec2, diameter_mm: f64) -> ArcPolyline2D {
    ArcPolyline2D::circle(center_xy, 0.5 * diameter_mm)
}

/// Configuration for [`build_rectangle_polyline`].
#[derive(Debug, Clone, Copy)]
pub struct RectEvalSpec {
    pub anchor: RectAnchorMode,
    pub anchor_xy: DVec2,
    pub width_mm: f64,
    pub height_mm: f64,
    pub corner_radius_mm: f64,
    pub rotation_deg: f64,
}

/// Builds a closed `CCW` rectangle [`ArcPolyline2D`] with exact `90°` corner arcs when `corner_radius_mm > 0`.
pub fn build_rectangle_polyline(spec: RectEvalSpec) -> ArcPolyline2D {
    let w = spec.width_mm;
    let h = spec.height_mm;
    if !w.is_finite() || !h.is_finite() || w <= 1e-5 || h <= 1e-5 || !spec.anchor_xy.is_finite() {
        return ArcPolyline2D::default();
    }
    let hw = 0.5 * w;
    let hh = 0.5 * h;
    let r = spec.corner_radius_mm.clamp(0.0, hw.min(hh));

    // Build unrotated centered around `(0, 0)` first, then rotate & translate to `center`!
    let mut segs = Vec::with_capacity(8);
    if r <= 1e-6 {
        let pts = [
            DVec2::new(-hw, -hh),
            DVec2::new(hw, -hh),
            DVec2::new(hw, hh),
            DVec2::new(-hw, hh),
        ];
        for i in 0..4 {
            segs.push(ArcSegment2D::Line {
                start: pts[i],
                end: pts[(i + 1) % 4],
            });
        }
    } else {
        let ix = (hw - r).max(0.0);
        let iy = (hh - r).max(0.0);

        let push_line = |s: DVec2, e: DVec2, out: &mut Vec<ArcSegment2D>| {
            if s.distance(e) > 1e-6 {
                out.push(ArcSegment2D::Line { start: s, end: e });
            }
        };

        // Bottom edge -> BR arc -> Right edge -> TR arc -> Top edge -> TL arc -> Left edge -> BL arc
        push_line(DVec2::new(-ix, -hh), DVec2::new(ix, -hh), &mut segs);
        segs.push(ArcSegment2D::Arc {
            start: DVec2::new(ix, -hh),
            end: DVec2::new(hw, -iy),
            center: DVec2::new(ix, -iy),
            radius: r,
            ccw: true,
        });
        push_line(DVec2::new(hw, -iy), DVec2::new(hw, iy), &mut segs);
        segs.push(ArcSegment2D::Arc {
            start: DVec2::new(hw, iy),
            end: DVec2::new(ix, hh),
            center: DVec2::new(ix, iy),
            radius: r,
            ccw: true,
        });
        push_line(DVec2::new(ix, hh), DVec2::new(-ix, hh), &mut segs);
        segs.push(ArcSegment2D::Arc {
            start: DVec2::new(-ix, hh),
            end: DVec2::new(-hw, iy),
            center: DVec2::new(-ix, iy),
            radius: r,
            ccw: true,
        });
        push_line(DVec2::new(-hw, iy), DVec2::new(-hw, -iy), &mut segs);
        segs.push(ArcSegment2D::Arc {
            start: DVec2::new(-hw, -iy),
            end: DVec2::new(-ix, -hh),
            center: DVec2::new(-ix, -iy),
            radius: r,
            ccw: true,
        });
    }

    let local_poly = ArcPolyline2D {
        segments: segs,
        closed: true,
    };
    let center_world = match spec.anchor {
        RectAnchorMode::Center => spec.anchor_xy,
        RectAnchorMode::FrontLeft => {
            let (s, c) = spec.rotation_deg.to_radians().sin_cos();
            spec.anchor_xy + DVec2::new(c * hw - s * hh, s * hw + c * hh)
        }
    };
    transform_arc_polyline_2d(&local_poly, center_world, 1.0, spec.rotation_deg)
}

/// Builds a closed `CCW` stadium slot [`ArcPolyline2D`] (`2` tangent lines + `2` `180°` end arcs).
pub fn build_slot_polyline(
    mode: SlotLengthMode,
    center_xy: DVec2,
    length_mm: f64,
    width_mm: f64,
    angle_deg: f64,
) -> ArcPolyline2D {
    if !width_mm.is_finite() || width_mm <= 1e-5 || !length_mm.is_finite() || !center_xy.is_finite()
    {
        return ArcPolyline2D::default();
    }
    let r = 0.5 * width_mm;
    let c2c = match mode {
        SlotLengthMode::CenterToCenter => length_mm.max(0.0),
        SlotLengthMode::OverallLength => (length_mm - width_mm).max(0.0),
    };
    if c2c <= 1e-6 {
        return build_circle_polyline(center_xy, width_mm);
    }
    let hc = 0.5 * c2c;
    let local = ArcPolyline2D {
        segments: vec![
            ArcSegment2D::Line {
                start: DVec2::new(-hc, -r),
                end: DVec2::new(hc, -r),
            },
            ArcSegment2D::Arc {
                start: DVec2::new(hc, -r),
                end: DVec2::new(hc, r),
                center: DVec2::new(hc, 0.0),
                radius: r,
                ccw: true,
            },
            ArcSegment2D::Line {
                start: DVec2::new(hc, r),
                end: DVec2::new(-hc, r),
            },
            ArcSegment2D::Arc {
                start: DVec2::new(-hc, r),
                end: DVec2::new(-hc, -r),
                center: DVec2::new(-hc, 0.0),
                radius: r,
                ccw: true,
            },
        ],
        closed: true,
    };
    transform_arc_polyline_2d(&local, center_xy, 1.0, angle_deg)
}

/// Configuration for [`build_regular_polygon_polyline`].
#[derive(Debug, Clone, Copy)]
pub struct PolygonEvalSpec {
    pub center_xy: DVec2,
    pub sides: u32,
    pub sizing: PolygonSizingMode,
    pub diameter_mm: f64,
    pub corner_radius_mm: f64,
    pub rotation_deg: f64,
}

/// Builds a closed `CCW` regular `N`-gon (`3..=64` sides) with optional corner fillet arcs.
pub fn build_regular_polygon_polyline(spec: PolygonEvalSpec) -> ArcPolyline2D {
    let n = spec.sides.clamp(3, 64) as usize;
    let d = spec.diameter_mm;
    if !d.is_finite() || d <= 1e-5 || !spec.center_xy.is_finite() {
        return ArcPolyline2D::default();
    }
    let half_wedge = std::f64::consts::PI / (n as f64);
    let circum_r = match spec.sizing {
        PolygonSizingMode::CircumscribedDiameter => 0.5 * d,
        PolygonSizingMode::InscribedDiameter | PolygonSizingMode::AcrossFlats => {
            0.5 * d / half_wedge.cos()
        }
    };

    let verts: Vec<DVec2> = (0..n)
        .map(|i| {
            let a = (i as f64) * (std::f64::consts::TAU / (n as f64));
            DVec2::new(a.cos(), a.sin()) * circum_r
        })
        .collect();

    // Interior half-angle at each vertex is `α = (π - 2π/n) / 2 = π/2 - π/n`
    let half_interior = std::f64::consts::FRAC_PI_2 - half_wedge;
    let side_len = 2.0 * circum_r * half_wedge.sin();
    // Trim distance along each side for fillet `r` is `t = r / tan(half_interior)` <= `0.5 * side_len`
    let max_r = 0.499 * side_len * half_interior.tan();
    let r = spec.corner_radius_mm.clamp(0.0, max_r);

    let mut segs = Vec::with_capacity(if r > 1e-6 { 2 * n } else { n });
    if r <= 1e-6 {
        for i in 0..n {
            segs.push(ArcSegment2D::Line {
                start: verts[i],
                end: verts[(i + 1) % n],
            });
        }
    } else {
        let trim = r / half_interior.tan();
        let center_inset = r / half_interior.sin();
        for i in 0..n {
            let v_prev = verts[(i + n - 1) % n];
            let v_curr = verts[i];
            let v_next = verts[(i + 1) % n];
            let v_next2 = verts[(i + 2) % n];

            let dir_out = (v_next - v_curr).normalize_or_zero();
            let dir_in = (v_prev - v_curr).normalize_or_zero();
            let arc_start = v_curr + dir_in * trim;
            let arc_end = v_curr + dir_out * trim;
            let bisector = (dir_in + dir_out).normalize_or_zero();
            let arc_center = v_curr + bisector * center_inset;

            segs.push(ArcSegment2D::Arc {
                start: arc_start,
                end: arc_end,
                center: arc_center,
                radius: r,
                ccw: true,
            });

            let next_dir_in = (v_curr - v_next).normalize_or_zero();
            let next_arc_start = v_next + next_dir_in * trim;
            let _ = v_next2;
            if arc_end.distance(next_arc_start) > 1e-6 {
                segs.push(ArcSegment2D::Line {
                    start: arc_end,
                    end: next_arc_start,
                });
            }
        }
    }

    let local = ArcPolyline2D {
        segments: segs,
        closed: true,
    };
    transform_arc_polyline_2d(&local, spec.center_xy, 1.0, spec.rotation_deg)
}

/// Applies uniform isotropic `scale`, counter-clockwise `rotation_deg`, and `translate_xy` to `poly`.
pub fn transform_arc_polyline_2d(
    poly: &ArcPolyline2D,
    translate_xy: DVec2,
    scale: f64,
    rotation_deg: f64,
) -> ArcPolyline2D {
    poly.transformed(translate_xy, scale, rotation_deg)
}
