use glam::DVec2;
use serde::{Deserialize, Serialize};

/// 2D axis-aligned bounding box in canonical `f64` millimeters.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Aabb2 {
    pub min: DVec2,
    pub max: DVec2,
}

impl Aabb2 {
    pub const EMPTY: Self = Self {
        min: DVec2::splat(f64::INFINITY),
        max: DVec2::splat(f64::NEG_INFINITY),
    };

    pub fn from_points(pts: impl IntoIterator<Item = DVec2>) -> Self {
        let mut b = Self::EMPTY;
        for p in pts {
            b.extend_point(p);
        }
        b
    }

    #[inline]
    pub fn extend_point(&mut self, p: DVec2) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }

    #[inline]
    pub fn union(&self, other: &Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    #[inline]
    pub fn is_valid(&self) -> bool {
        self.min.x <= self.max.x && self.min.y <= self.max.y
    }

    #[inline]
    pub fn center(&self) -> DVec2 {
        if self.is_valid() {
            0.5 * (self.min + self.max)
        } else {
            DVec2::ZERO
        }
    }

    #[inline]
    pub fn size(&self) -> DVec2 {
        if self.is_valid() {
            self.max - self.min
        } else {
            DVec2::ZERO
        }
    }
}

/// Exact 2D analytic curve segment preserving true circular arcs (`G2`/`G3`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ArcSegment2D {
    Line {
        start: DVec2,
        end: DVec2,
    },
    Arc {
        start: DVec2,
        end: DVec2,
        center: DVec2,
        radius: f64,
        ccw: bool,
    },
}

impl ArcSegment2D {
    pub fn start(&self) -> DVec2 {
        match *self {
            Self::Line { start, .. } | Self::Arc { start, .. } => start,
        }
    }

    pub fn end(&self) -> DVec2 {
        match *self {
            Self::Line { end, .. } | Self::Arc { end, .. } => end,
        }
    }

    /// Positive CCW or negative CW sweep angle in radians (`(0, 2π]` magnitude).
    pub fn signed_sweep_rad(&self) -> f64 {
        match *self {
            Self::Line { .. } => 0.0,
            Self::Arc {
                start,
                end,
                center,
                ccw,
                ..
            } => {
                let a0 = (start.y - center.y).atan2(start.x - center.x);
                let a1 = (end.y - center.y).atan2(end.x - center.x);
                if ccw {
                    let mut s = (a1 - a0).rem_euclid(std::f64::consts::TAU);
                    if s <= 1e-12 {
                        s = std::f64::consts::TAU;
                    }
                    s
                } else {
                    let mut s = (a0 - a1).rem_euclid(std::f64::consts::TAU);
                    if s <= 1e-12 {
                        s = std::f64::consts::TAU;
                    }
                    -s
                }
            }
        }
    }

    pub fn length(&self) -> f64 {
        match *self {
            Self::Line { start, end } => start.distance(end),
            Self::Arc { radius, .. } => radius * self.signed_sweep_rad().abs(),
        }
    }

    /// Reverses travel direction along this segment while preserving exact geometry.
    pub fn reverse(&self) -> Self {
        match *self {
            Self::Line { start, end } => Self::Line {
                start: end,
                end: start,
            },
            Self::Arc {
                start,
                end,
                center,
                radius,
                ccw,
            } => Self::Arc {
                start: end,
                end: start,
                center,
                radius,
                ccw: !ccw,
            },
        }
    }

    /// Exact tight 2D bounding box, including cardinal quadrant extrema swept by circular arcs.
    pub fn exact_aabb(&self) -> Aabb2 {
        let mut b = Aabb2::EMPTY;
        b.extend_point(self.start());
        b.extend_point(self.end());
        if let Self::Arc {
            start,
            center,
            radius,
            ccw,
            ..
        } = *self
        {
            let a0 = (start.y - center.y).atan2(start.x - center.x);
            let sweep_mag = self.signed_sweep_rad().abs();
            let cardinals = [
                (0.0, DVec2::new(radius, 0.0)),
                (std::f64::consts::FRAC_PI_2, DVec2::new(0.0, radius)),
                (std::f64::consts::PI, DVec2::new(-radius, 0.0)),
                (3.0 * std::f64::consts::FRAC_PI_2, DVec2::new(0.0, -radius)),
            ];
            for (angle, offset) in cardinals {
                let delta = if ccw {
                    (angle - a0).rem_euclid(std::f64::consts::TAU)
                } else {
                    (a0 - angle).rem_euclid(std::f64::consts::TAU)
                };
                if delta <= sweep_mag + 1e-12 {
                    b.extend_point(center + offset);
                }
            }
        }
        b
    }

    /// Evaluates position and unit tangent direction at normalized arc-length parameter `u ∈ [0, 1]`.
    pub fn eval_at(&self, u: f64) -> Option<(DVec2, DVec2)> {
        let u = u.clamp(0.0, 1.0);
        match *self {
            Self::Line { start, end } => {
                let d = end - start;
                let len = d.length();
                if len < 1e-12 {
                    None
                } else {
                    Some((start + d * u, d / len))
                }
            }
            Self::Arc {
                start,
                center,
                radius,
                ccw,
                ..
            } => {
                if radius < 1e-12 {
                    return None;
                }
                let a0 = (start.y - center.y).atan2(start.x - center.x);
                let theta = a0 + self.signed_sweep_rad() * u;
                let (s, c) = theta.sin_cos();
                let pt = center + DVec2::new(c, s) * radius;
                let tangent = if ccw {
                    DVec2::new(-s, c)
                } else {
                    DVec2::new(s, -c)
                };
                Some((pt, tangent))
            }
        }
    }
}

/// Connected sequence of 2D analytic line and circular arc segments in `f64` millimeters.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct ArcPolyline2D {
    pub segments: Vec<ArcSegment2D>,
    pub closed: bool,
}

impl ArcPolyline2D {
    pub fn perimeter(&self) -> f64 {
        self.segments.iter().map(ArcSegment2D::length).sum()
    }

    pub fn aabb_endpoints(&self) -> Aabb2 {
        let mut box2 = Aabb2::EMPTY;
        for seg in &self.segments {
            box2.extend_point(seg.start());
            box2.extend_point(seg.end());
        }
        box2
    }

    /// Exact 2D bounding box accounting for circular arc bulges beyond chord endpoints.
    pub fn exact_aabb(&self) -> Aabb2 {
        let mut box2 = Aabb2::EMPTY;
        for seg in &self.segments {
            box2 = box2.union(&seg.exact_aabb());
        }
        box2
    }

    /// Returns a reversed copy of this polyline (`end -> start` with inverted arc handedness).
    pub fn reverse(&self) -> Self {
        let segments = self
            .segments
            .iter()
            .rev()
            .map(ArcSegment2D::reverse)
            .collect();
        Self {
            segments,
            closed: self.closed,
        }
    }

    /// Exact signed area (`> 0` for CCW outer loops, `< 0` for CW hole/island loops).
    pub fn signed_area(&self) -> f64 {
        let mut double_area = 0.0;
        for seg in &self.segments {
            let p0 = seg.start();
            let p1 = seg.end();
            // Chord trapezoid/shoelace contribution
            double_area += p0.x * p1.y - p1.x * p0.y;
            // Circular segment cap contribution: 0.5 * r^2 * (θ - sin(θ)) => double_area += r^2 * (θ - sin(θ))
            if let ArcSegment2D::Arc { radius, .. } = *seg {
                let sweep = seg.signed_sweep_rad();
                double_area += radius * radius * (sweep - sweep.sin());
            }
        }
        0.5 * double_area
    }

    /// Evaluates `(point, unit_tangent)` at fractional distance `frac ∈ [0, 1]` along the perimeter.
    pub fn sample_at_fraction(&self, frac: f64) -> Option<(DVec2, DVec2)> {
        let total = self.perimeter();
        if total < 1e-12 {
            return None;
        }
        let target_s = total * frac.clamp(0.0, 1.0);
        let mut accum = 0.0;
        for seg in &self.segments {
            let seg_len = seg.length();
            if seg_len < 1e-12 {
                continue;
            }
            if accum + seg_len >= target_s {
                let local_u = ((target_s - accum) / seg_len).clamp(0.0, 1.0);
                return seg.eval_at(local_u);
            }
            accum += seg_len;
        }
        self.segments.last().and_then(|s| s.eval_at(1.0))
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::units::approx_eq;

    #[test]
    fn exact_arc_aabb_signed_area_and_reverse_are_accurate() {
        // Upper + lower 180° arcs forming a CCW circle of radius 10 centered at (5, -3)
        let center = DVec2::new(5.0, -3.0);
        let r = 10.0;
        let poly = ArcPolyline2D {
            segments: vec![
                ArcSegment2D::Arc {
                    start: center + DVec2::new(r, 0.0),
                    end: center + DVec2::new(-r, 0.0),
                    center,
                    radius: r,
                    ccw: true,
                },
                ArcSegment2D::Arc {
                    start: center + DVec2::new(-r, 0.0),
                    end: center + DVec2::new(r, 0.0),
                    center,
                    radius: r,
                    ccw: true,
                },
            ],
            closed: true,
        };

        let aabb = poly.exact_aabb();
        assert!(approx_eq(aabb.min.x, -5.0, 1e-12));
        assert!(approx_eq(aabb.max.x, 15.0, 1e-12));
        assert!(approx_eq(aabb.min.y, -13.0, 1e-12));
        assert!(approx_eq(aabb.max.y, 7.0, 1e-12));

        let expected_area = std::f64::consts::PI * r * r;
        assert!(approx_eq(poly.signed_area(), expected_area, 1e-9));

        let rev = poly.reverse();
        assert!(approx_eq(rev.signed_area(), -expected_area, 1e-9));
        assert!(approx_eq(rev.perimeter(), poly.perimeter(), 1e-9));
    }
}
