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

    #[inline]
    pub fn contains_point(&self, p: DVec2) -> bool {
        self.is_valid()
            && p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
    }

    #[inline]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.is_valid()
            && other.is_valid()
            && self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
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
    /// Constructs a line or circular arc segment from endpoints `p0 -> p1` and a polyline
    /// bulge factor `b = tan(Δθ / 4)` (`b > 0` CCW, `b < 0` CW, `|b| <= 1e-6` linear).
    ///
    /// Falls back to [`ArcSegment2D::Line`] for near-zero bulges, degenerate chords, or
    /// non-finite / out-of-range radii (`<= 1e-5` or `>= 1e7`).
    pub fn from_bulge(p0: DVec2, p1: DVec2, bulge: f64) -> Self {
        if !p0.is_finite() || !p1.is_finite() || !bulge.is_finite() || bulge.abs() <= 1e-6 {
            return Self::Line { start: p0, end: p1 };
        }
        let chord = p1 - p0;
        let chord_len = chord.length();
        if !chord_len.is_finite() || chord_len <= 1e-12 {
            return Self::Line { start: p0, end: p1 };
        }
        let bulge_squared = bulge * bulge;
        let inv_four_bulge = 1.0 / (4.0 * bulge);
        let radius = chord_len * (1.0 + bulge_squared) * inv_four_bulge.abs();
        let perp = DVec2::new(-chord.y, chord.x);
        let center = p0 + chord * 0.5 + perp * ((1.0 - bulge) * (1.0 + bulge) * inv_four_bulge);
        if radius.is_finite() && radius > 1e-5 && radius < 1e7 && center.is_finite() {
            Self::Arc {
                start: p0,
                end: p1,
                center,
                radius,
                ccw: bulge > 0.0,
            }
        } else {
            Self::Line { start: p0, end: p1 }
        }
    }

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

    /// Extracts the exact sub-segment over normalized arc-length interval `[u0, u1] ⊆ [0, 1]`.
    pub fn slice_u(&self, u0: f64, u1: f64) -> Option<Self> {
        let a = u0.clamp(0.0, 1.0);
        let b = u1.clamp(0.0, 1.0);
        if b - a <= 1e-12 {
            return None;
        }
        match *self {
            Self::Line { start, end } => {
                let d = end - start;
                let p0 = start + d * a;
                let p1 = start + d * b;
                if p0.distance(p1) <= 1e-12 {
                    None
                } else {
                    Some(Self::Line { start: p0, end: p1 })
                }
            }
            Self::Arc {
                start,
                center,
                radius,
                ccw,
                ..
            } => {
                if radius <= 1e-12 {
                    return None;
                }
                let a0 = (start.y - center.y).atan2(start.x - center.x);
                let sweep = self.signed_sweep_rad();
                let th0 = a0 + sweep * a;
                let th1 = a0 + sweep * b;
                let p0 = center + DVec2::new(th0.cos(), th0.sin()) * radius;
                let p1 = center + DVec2::new(th1.cos(), th1.sin()) * radius;
                if (sweep * (b - a)).abs() * radius <= 1e-12 {
                    None
                } else {
                    Some(Self::Arc {
                        start: p0,
                        end: p1,
                        center,
                        radius,
                        ccw,
                    })
                }
            }
        }
    }

    /// Splits this segment at normalized arc-length parameter `u ∈ [0, 1]`.
    pub fn split_at(&self, u: f64) -> (Option<Self>, Option<Self>) {
        (self.slice_u(0.0, u), self.slice_u(u, 1.0))
    }

    /// Returns `(closest_point, local_u ∈ [0, 1], distance_mm)` from `query` to this segment.
    pub fn closest_point(&self, query: DVec2) -> (DVec2, f64, f64) {
        match *self {
            Self::Line { start, end } => {
                let ab = end - start;
                let len_sq = ab.length_squared();
                if len_sq <= 1e-24 {
                    return (start, 0.0, query.distance(start));
                }
                let u = ((query - start).dot(ab) / len_sq).clamp(0.0, 1.0);
                let pt = start + ab * u;
                (pt, u, query.distance(pt))
            }
            Self::Arc {
                start,
                end,
                center,
                radius,
                ccw,
            } => {
                let rel = query - center;
                let rel_len = rel.length();
                let sweep_mag = self.signed_sweep_rad().abs();
                if radius <= 1e-12 || sweep_mag <= 1e-12 {
                    return (start, 0.0, query.distance(start));
                }
                if rel_len > 1e-12 {
                    let a0 = (start.y - center.y).atan2(start.x - center.x);
                    let aq = rel.y.atan2(rel.x);
                    let delta = if ccw {
                        (aq - a0).rem_euclid(std::f64::consts::TAU)
                    } else {
                        (a0 - aq).rem_euclid(std::f64::consts::TAU)
                    };
                    if delta <= sweep_mag + 1e-12 {
                        let u = (delta / sweep_mag).clamp(0.0, 1.0);
                        let pt = center + (rel / rel_len) * radius;
                        return (pt, u, query.distance(pt));
                    }
                }
                let d0 = query.distance(start);
                let d1 = query.distance(end);
                if d0 <= d1 {
                    (start, 0.0, d0)
                } else {
                    (end, 1.0, d1)
                }
            }
        }
    }
}

/// Closest-point projection result onto an [`ArcPolyline2D`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PolylineProjection {
    pub s_mm: f64,
    pub s_norm: f64,
    pub point: DVec2,
    pub tangent: DVec2,
    pub distance_mm: f64,
}

/// Connected sequence of 2D analytic line and circular arc segments in `f64` millimeters.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct ArcPolyline2D {
    pub segments: Vec<ArcSegment2D>,
    pub closed: bool,
}

impl ArcPolyline2D {
    /// Builds a closed `CCW` 2-arc circle of `radius` centered at `center`.
    ///
    /// Returns an empty [`ArcPolyline2D`] if `center` is non-finite or `radius <= 1e-6`.
    pub fn circle(center: DVec2, radius: f64) -> Self {
        if !center.is_finite() || !radius.is_finite() || radius <= 1e-6 {
            return Self::default();
        }
        let p_r = center + DVec2::new(radius, 0.0);
        let p_l = center - DVec2::new(radius, 0.0);
        Self {
            segments: vec![
                ArcSegment2D::Arc {
                    start: p_r,
                    end: p_l,
                    center,
                    radius,
                    ccw: true,
                },
                ArcSegment2D::Arc {
                    start: p_l,
                    end: p_r,
                    center,
                    radius,
                    ccw: true,
                },
            ],
            closed: true,
        }
    }

    /// Applies uniform isotropic `scale`, counter-clockwise `rotation_deg`, and `translate_xy` to `self`.
    pub fn transformed(&self, translate_xy: DVec2, scale: f64, rotation_deg: f64) -> Self {
        let s = if scale.is_finite() && scale.abs() > 1e-6 {
            scale
        } else {
            1.0
        };
        let (sin_r, cos_r) = rotation_deg.to_radians().sin_cos();
        let map_pt = |p: DVec2| -> DVec2 {
            let sp = p * s;
            DVec2::new(cos_r * sp.x - sin_r * sp.y, sin_r * sp.x + cos_r * sp.y) + translate_xy
        };
        let rad_scale = s.abs();
        let flip = s < 0.0;
        let mut segs = Vec::with_capacity(self.segments.len());
        for seg in &self.segments {
            match *seg {
                ArcSegment2D::Line { start, end } => {
                    segs.push(ArcSegment2D::Line {
                        start: map_pt(start),
                        end: map_pt(end),
                    });
                }
                ArcSegment2D::Arc {
                    start,
                    end,
                    center,
                    radius,
                    ccw,
                } => {
                    segs.push(ArcSegment2D::Arc {
                        start: map_pt(start),
                        end: map_pt(end),
                        center: map_pt(center),
                        radius: radius * rad_scale,
                        ccw: if flip { !ccw } else { ccw },
                    });
                }
            }
        }
        Self {
            segments: segs,
            closed: self.closed,
        }
    }

    pub fn perimeter(&self) -> f64 {
        self.segments.iter().map(ArcSegment2D::length).sum()
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

    /// Evaluates `(point, unit_tangent)` at arc-length `s_mm` from the start (wrapping modulo perimeter if `closed`).
    pub fn eval_at_distance(&self, s_mm: f64) -> Option<(DVec2, DVec2)> {
        let total = self.perimeter();
        if total < 1e-12 {
            return None;
        }
        let s = if self.closed {
            s_mm.rem_euclid(total)
        } else {
            s_mm.clamp(0.0, total)
        };
        self.sample_at_fraction(s / total)
    }

    /// Projects `query` onto the closest point along this polyline.
    pub fn project_point(&self, query: DVec2) -> Option<PolylineProjection> {
        let total = self.perimeter();
        if total < 1e-12 {
            return None;
        }
        let mut best: Option<PolylineProjection> = None;
        let mut accum = 0.0;
        for seg in &self.segments {
            let seg_len = seg.length();
            if seg_len < 1e-12 {
                continue;
            }
            let (pt, u, dist) = seg.closest_point(query);
            let is_better = match best {
                None => true,
                Some(prev) => dist < prev.distance_mm,
            };
            if is_better {
                let tangent = seg.eval_at(u).map_or(DVec2::X, |(_, t)| t);
                let s_mm = (accum + u * seg_len).clamp(0.0, total);
                best = Some(PolylineProjection {
                    s_mm,
                    s_norm: (s_mm / total).clamp(0.0, 1.0),
                    point: pt,
                    tangent,
                    distance_mm: dist,
                });
            }
            accum += seg_len;
        }
        best
    }

    /// Minimum Euclidean distance from `query` to any curve segment on this polyline.
    pub fn distance_to_point(&self, query: DVec2) -> f64 {
        self.project_point(query)
            .map_or(f64::INFINITY, |p| p.distance_mm)
    }

    /// Ensures a closed loop has `CCW` (`ccw == true`, `signed_area > 0`) or `CW` (`ccw == false`) orientation.
    pub fn ensure_orientation(&self, ccw: bool) -> Self {
        if !self.closed {
            return self.clone();
        }
        let is_ccw = self.signed_area() >= 0.0;
        if is_ccw == ccw {
            self.clone()
        } else {
            self.reverse()
        }
    }

    /// Extracts an open sub-polyline over `[s_start_mm, s_end_mm]`.
    /// When `self.closed` and `s_end_mm > s_start_mm`, supports seamless wrap-around past `perimeter()`.
    pub fn slice_interval(&self, s_start_mm: f64, s_end_mm: f64) -> Self {
        let total = self.perimeter();
        if total < 1e-12 || s_end_mm <= s_start_mm + 1e-12 {
            return Self {
                segments: Vec::new(),
                closed: false,
            };
        }

        if !self.closed {
            let a = s_start_mm.clamp(0.0, total);
            let b = s_end_mm.clamp(0.0, total);
            return Self {
                segments: self.slice_non_wrapping(a, b),
                closed: false,
            };
        }

        let span = (s_end_mm - s_start_mm).min(total);
        let a = s_start_mm.rem_euclid(total);
        if a + span <= total + 1e-12 {
            Self {
                segments: self.slice_non_wrapping(a, (a + span).min(total)),
                closed: false,
            }
        } else {
            let mut segs = self.slice_non_wrapping(a, total);
            segs.extend(self.slice_non_wrapping(0.0, (a + span - total).min(total)));
            Self {
                segments: segs,
                closed: false,
            }
        }
    }

    fn slice_non_wrapping(&self, a: f64, b: f64) -> Vec<ArcSegment2D> {
        let mut out = Vec::new();
        if b <= a + 1e-12 {
            return out;
        }
        let mut accum = 0.0;
        for seg in &self.segments {
            let len = seg.length();
            if len < 1e-12 {
                continue;
            }
            let seg_s0 = accum;
            let seg_s1 = accum + len;
            let ov0 = a.max(seg_s0);
            let ov1 = b.min(seg_s1);
            if ov1 > ov0 + 1e-12 {
                let u0 = ((ov0 - seg_s0) / len).clamp(0.0, 1.0);
                let u1 = ((ov1 - seg_s0) / len).clamp(0.0, 1.0);
                if let Some(sub) = seg.slice_u(u0, u1) {
                    out.push(sub);
                }
            }
            accum = seg_s1;
        }
        out
    }

    /// Shifts the start/end seam of a closed polyline to `new_start_s_mm` around its perimeter.
    pub fn rotate_closed_seam(&self, new_start_s_mm: f64) -> Self {
        if !self.closed {
            return self.clone();
        }
        let total = self.perimeter();
        if total < 1e-12 {
            return self.clone();
        }
        let s0 = new_start_s_mm.rem_euclid(total);
        if s0 <= 1e-9 || (total - s0) <= 1e-9 {
            return self.clone();
        }
        let mut sliced = self.slice_interval(s0, s0 + total);
        sliced.closed = true;
        sliced
    }

    /// Exact analytic point-in-region test for a closed [`ArcPolyline2D`], accounting for circular arc caps.
    pub fn contains_point(&self, pt: DVec2) -> bool {
        if !self.closed || self.segments.is_empty() {
            return false;
        }
        let aabb = self.exact_aabb();
        if pt.x < aabb.min.x - 1e-9
            || pt.x > aabb.max.x + 1e-9
            || pt.y < aabb.min.y - 1e-9
            || pt.y > aabb.max.y + 1e-9
        {
            return false;
        }
        if self.distance_to_point(pt) <= 1e-8 {
            return true;
        }

        // Subdivide any arc >= 150° into minor arcs (< 90°) so every circular cap is strictly convex
        let mut minor_segs = Vec::with_capacity(self.segments.len() + 4);
        for seg in &self.segments {
            if matches!(seg, ArcSegment2D::Arc { .. }) && seg.signed_sweep_rad().abs() > 2.6 {
                let (h0, h1) = seg.split_at(0.5);
                if let Some(s0) = h0 {
                    minor_segs.push(s0);
                }
                if let Some(s1) = h1 {
                    minor_segs.push(s1);
                }
            } else {
                minor_segs.push(*seg);
            }
        }

        // 1. Standard ray-casting parity across segment chords
        let mut inside = false;
        for seg in &minor_segs {
            let a = seg.start();
            let b = seg.end();
            if (a.y > pt.y) != (b.y > pt.y) {
                let x_intersect = a.x + (pt.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if pt.x < x_intersect {
                    inside = !inside;
                }
            }
        }

        // 2. Toggle parity for any strictly convex minor circular segment cap containing `pt`
        for seg in &minor_segs {
            if let ArcSegment2D::Arc {
                start,
                end,
                center,
                radius,
                ccw,
            } = *seg
                && pt.distance_squared(center) <= radius * radius
            {
                let chord = end - start;
                let rel = pt - start;
                let cross = chord.x * rel.y - chord.y * rel.x;
                // For a minor CCW arc, the circular cap lies to the RIGHT of chord `start -> end` (`cross < 0`);
                // for a minor CW arc, the circular cap lies to the LEFT of chord `start -> end` (`cross > 0`).
                let in_cap = if ccw { cross < 0.0 } else { cross > 0.0 };
                if in_cap {
                    inside = !inside;
                }
            }
        }

        inside
    }
}

/// Computes `(center, radius, ccw)` of the 2D circle passing through `a -> m -> b`.
pub fn circumcircle_through_three_points(
    a: DVec2,
    m: DVec2,
    b: DVec2,
) -> Option<(DVec2, f64, bool)> {
    let d1 = m - a;
    let d2 = b - m;
    let cross = d1.x * d2.y - d1.y * d2.x;
    if cross.abs() <= 1e-9 {
        return None;
    }

    let ax = a.x;
    let ay = a.y;
    let bx = m.x - ax;
    let by = m.y - ay;
    let cx = b.x - ax;
    let cy = b.y - ay;

    let denom = 2.0 * (bx * cy - by * cx);
    if denom.abs() <= 1e-11 {
        return None;
    }
    let b2 = bx * bx + by * by;
    let c2 = cx * cx + cy * cy;
    let ux = (cy * b2 - by * c2) / denom;
    let uy = (bx * c2 - cx * b2) / denom;
    let center = DVec2::new(ax + ux, ay + uy);
    let radius = center.distance(a);
    let ccw = cross > 0.0;
    Some((center, radius, ccw))
}

/// Exact `f64` 2D point-in-triangle test (orientation-independent, inclusive of edges within `1e-10`).
#[inline]
pub fn point_in_triangle_2d(p: DVec2, a: DVec2, b: DVec2, c: DVec2) -> bool {
    let area2 = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if area2.abs() <= 1e-14 {
        return false;
    }
    let inv = 1.0 / area2;
    let w0 = ((b.x - p.x) * (c.y - p.y) - (b.y - p.y) * (c.x - p.x)) * inv;
    let w1 = ((c.x - p.x) * (a.y - p.y) - (c.y - p.y) * (a.x - p.x)) * inv;
    let w2 = 1.0 - w0 - w1;
    w0 >= -1e-10 && w1 >= -1e-10 && w2 >= -1e-10
}

/// Closest point on 2D segment `(a, b)` to query point `p`.
#[inline]
pub fn closest_point_on_segment_2d(p: DVec2, a: DVec2, b: DVec2) -> DVec2 {
    let ab = b - a;
    let len_sq = ab.length_squared();
    if len_sq <= 1e-18 {
        return a;
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    a + ab * t
}

/// Exact squared Euclidean distance from 2D point `p` to 2D segment `(a, b)`.
#[inline]
pub fn point_to_segment_dist_sq_2d(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    p.distance_squared(closest_point_on_segment_2d(p, a, b))
}

/// Exact Euclidean distance from 2D point `p` to 2D segment `(a, b)`.
#[inline]
pub fn point_to_segment_dist_2d(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    p.distance(closest_point_on_segment_2d(p, a, b))
}

/// Exact minimum 2D distance between segments `(p0, p1)` and `(q0, q1)`, returning
/// `(distance, u_on_q)` where `q0.lerp(q1, u_on_q)` achieves the minimum distance.
pub fn segment_to_segment_dist_2d(p0: DVec2, p1: DVec2, q0: DVec2, q1: DVec2) -> (f64, f64) {
    let dp = p1 - p0;
    let dq = q1 - q0;
    let denom = dp.x * dq.y - dp.y * dq.x;
    if denom.abs() > 1e-12 {
        let r = q0 - p0;
        let t = (r.x * dq.y - r.y * dq.x) / denom;
        let u = (r.x * dp.y - r.y * dp.x) / denom;
        if (-1e-9..=1.0 + 1e-9).contains(&t) && (-1e-9..=1.0 + 1e-9).contains(&u) {
            return (0.0, u.clamp(0.0, 1.0));
        }
    }

    let mut best_d = f64::INFINITY;
    let mut best_u = 0.0;

    // 1. Endpoints of `p` projected onto `q`
    let q_len2 = dq.length_squared();
    for &ep in &[p0, p1] {
        let u = if q_len2 <= 1e-18 {
            0.0
        } else {
            ((ep - q0).dot(dq) / q_len2).clamp(0.0, 1.0)
        };
        let d = ep.distance(q0 + dq * u);
        if d < best_d {
            best_d = d;
            best_u = u;
        }
    }

    // 2. Endpoints of `q` projected onto `p`
    let p_len2 = dp.length_squared();
    for &(eq, u_fixed) in &[(q0, 0.0), (q1, 1.0)] {
        let t = if p_len2 <= 1e-18 {
            0.0
        } else {
            ((eq - p0).dot(dp) / p_len2).clamp(0.0, 1.0)
        };
        let d = eq.distance(p0 + dp * t);
        if d < best_d {
            best_d = d;
            best_u = u_fixed;
        }
    }

    (best_d, best_u)
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

        // Point-in-circle & cap tests
        assert!(poly.contains_point(center));
        assert!(poly.contains_point(center + DVec2::new(9.9, 0.0)));
        assert!(poly.contains_point(center + DVec2::new(7.0, 7.0))); // 9.899 < 10 -> inside circular cap!
        assert!(!poly.contains_point(center + DVec2::new(7.2, 7.2))); // 10.18 > 10 -> outside!

        // Seam rotation preserves area, perimeter, and exact arcs
        let rotated = poly.rotate_closed_seam(12.345);
        assert!(rotated.closed);
        assert!(approx_eq(rotated.perimeter(), poly.perimeter(), 1e-9));
        assert!(approx_eq(rotated.signed_area(), expected_area, 1e-8));

        // Closest-point projection onto arc
        let proj = poly.project_point(center + DVec2::new(0.0, 20.0)).unwrap();
        assert!(approx_eq(proj.distance_mm, 10.0, 1e-9));
        assert!(approx_eq(proj.point.x, center.x, 1e-9));
        assert!(approx_eq(proj.point.y, center.y + 10.0, 1e-9));
    }

    #[test]
    fn from_bulge_recovers_exact_minor_semi_and_major_arcs() {
        let r = 10.0;
        // 90° CCW arc from (R, 0) to (0, R): bulge = tan(pi/8) = sqrt(2) - 1
        let b90 = (std::f64::consts::FRAC_PI_2 * 0.25).tan();
        let seg_ccw = ArcSegment2D::from_bulge(DVec2::new(r, 0.0), DVec2::new(0.0, r), b90);
        match seg_ccw {
            ArcSegment2D::Arc {
                center,
                radius,
                ccw,
                ..
            } => {
                assert!(ccw);
                assert!(center.distance(DVec2::ZERO) < 1e-12);
                assert!(approx_eq(radius, r, 1e-12));
            }
            _ => panic!("expected Arc"),
        }

        // 180° CCW semicircle: bulge = 1.0
        let seg_semi = ArcSegment2D::from_bulge(DVec2::new(r, 0.0), DVec2::new(-r, 0.0), 1.0);
        match seg_semi {
            ArcSegment2D::Arc {
                center,
                radius,
                ccw,
                ..
            } => {
                assert!(ccw);
                assert!(center.distance(DVec2::ZERO) < 1e-12);
                assert!(approx_eq(radius, r, 1e-12));
            }
            _ => panic!("expected Arc"),
        }

        // 270° CW major arc from (R, 0) to (0, R): bulge = -tan(3*pi/8) = -(sqrt(2) + 1)
        let b270_cw = -(3.0 * std::f64::consts::FRAC_PI_2 * 0.25).tan();
        let seg_cw = ArcSegment2D::from_bulge(DVec2::new(r, 0.0), DVec2::new(0.0, r), b270_cw);
        match seg_cw {
            ArcSegment2D::Arc {
                center,
                radius,
                ccw,
                ..
            } => {
                assert!(!ccw);
                assert!(center.distance(DVec2::ZERO) < 1e-12);
                assert!(approx_eq(radius, r, 1e-12));
            }
            _ => panic!("expected Arc"),
        }

        // Zero bulge => Line
        assert!(matches!(
            ArcSegment2D::from_bulge(DVec2::ZERO, DVec2::new(5.0, 0.0), 0.0),
            ArcSegment2D::Line { .. }
        ));
    }

    #[test]
    fn circle_transformed_and_2d_primitives_are_accurate() {
        let c = ArcPolyline2D::circle(DVec2::new(2.0, 3.0), 5.0);
        assert!(approx_eq(
            c.signed_area(),
            std::f64::consts::PI * 25.0,
            1e-9
        ));
        let xf = c.transformed(DVec2::new(10.0, -4.0), 2.0, 90.0);
        assert!(approx_eq(
            xf.signed_area(),
            std::f64::consts::PI * 100.0,
            1e-9
        ));

        let (cc, cr, ccw) = circumcircle_through_three_points(
            DVec2::new(10.0, 0.0),
            DVec2::new(0.0, 10.0),
            DVec2::new(-10.0, 0.0),
        )
        .unwrap();
        assert!(cc.distance(DVec2::ZERO) < 1e-12);
        assert!(approx_eq(cr, 10.0, 1e-12));
        assert!(ccw);

        assert!(point_in_triangle_2d(
            DVec2::new(1.0, 1.0),
            DVec2::ZERO,
            DVec2::new(4.0, 0.0),
            DVec2::new(0.0, 4.0)
        ));
        assert!(!point_in_triangle_2d(
            DVec2::new(3.0, 3.0),
            DVec2::ZERO,
            DVec2::new(4.0, 0.0),
            DVec2::new(0.0, 4.0)
        ));
        assert!(approx_eq(
            point_to_segment_dist_2d(DVec2::new(5.0, 3.0), DVec2::ZERO, DVec2::new(10.0, 0.0)),
            3.0,
            1e-12
        ));
        let (d, u) = segment_to_segment_dist_2d(
            DVec2::new(0.0, 0.0),
            DVec2::new(10.0, 0.0),
            DVec2::new(5.0, -2.0),
            DVec2::new(5.0, 2.0),
        );
        assert!(approx_eq(d, 0.0, 1e-12));
        assert!(approx_eq(u, 0.5, 1e-12));
    }
}
