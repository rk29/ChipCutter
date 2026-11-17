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
    pub fn is_valid(&self) -> bool {
        self.min.x <= self.max.x && self.min.y <= self.max.y
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

    pub fn length(&self) -> f64 {
        match *self {
            Self::Line { start, end } => start.distance(end),
            Self::Arc {
                start,
                end,
                center,
                radius,
                ccw,
            } => {
                let a0 = (start.y - center.y).atan2(start.x - center.x);
                let a1 = (end.y - center.y).atan2(end.x - center.x);
                let mut sweep = if ccw { a1 - a0 } else { a0 - a1 };
                if sweep <= 1e-12 {
                    sweep += std::f64::consts::TAU;
                }
                radius * sweep
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
}
