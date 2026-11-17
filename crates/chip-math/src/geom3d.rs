use glam::DVec3;
use serde::{Deserialize, Serialize};

/// 3D axis-aligned bounding box in canonical `f64` millimeters.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Aabb3 {
    pub min: DVec3,
    pub max: DVec3,
}

impl Aabb3 {
    pub const EMPTY: Self = Self {
        min: DVec3::splat(f64::INFINITY),
        max: DVec3::splat(f64::NEG_INFINITY),
    };

    pub fn new(min: DVec3, max: DVec3) -> Self {
        Self { min, max }
    }

    pub fn from_points(pts: impl IntoIterator<Item = DVec3>) -> Self {
        let mut b = Self::EMPTY;
        for p in pts {
            b.extend_point(p);
        }
        b
    }

    #[inline]
    pub fn extend_point(&mut self, p: DVec3) {
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
        self.min.x <= self.max.x && self.min.y <= self.max.y && self.min.z <= self.max.z
    }

    #[inline]
    pub fn center(&self) -> DVec3 {
        if self.is_valid() {
            0.5 * (self.min + self.max)
        } else {
            DVec3::ZERO
        }
    }

    #[inline]
    pub fn size(&self) -> DVec3 {
        if self.is_valid() {
            self.max - self.min
        } else {
            DVec3::ZERO
        }
    }

    #[inline]
    pub fn diagonal(&self) -> f64 {
        self.size().length()
    }
}

/// Closest-approach result between a 3D picking ray and a 3D line segment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SegmentHit {
    /// Distance along the ray (`origin + ray_t * dir`), always `>= 0.0`.
    pub ray_t: f64,
    /// Parameter along segment `a..b` in `[0.0, 1.0]`.
    pub seg_u: f64,
    /// Perpendicular Euclidean distance (`mm`) between the ray and segment.
    pub distance_mm: f64,
    /// Closest point on the segment (`mm`).
    pub point_on_segment: DVec3,
}

/// 3D ray in canonical `f64` millimeters (`origin` + unit `dir`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ray3 {
    pub origin: DVec3,
    pub dir: DVec3,
}

impl Ray3 {
    pub fn new(origin: DVec3, dir: DVec3) -> Option<Self> {
        let len = dir.length();
        if len < 1e-12 || !len.is_finite() {
            None
        } else {
            Some(Self {
                origin,
                dir: dir / len,
            })
        }
    }

    #[inline]
    pub fn at(&self, t: f64) -> DVec3 {
        self.origin + self.dir * t
    }

    /// Slab intersection test against an `Aabb3` (expanded by `margin_mm`), returning entry `t`.
    pub fn intersect_aabb(&self, aabb: &Aabb3, margin_mm: f64) -> Option<f64> {
        if !aabb.is_valid() {
            return None;
        }
        let min = aabb.min - DVec3::splat(margin_mm);
        let max = aabb.max + DVec3::splat(margin_mm);

        let mut t_min = 0.0_f64;
        let mut t_max = f64::INFINITY;

        for axis in 0..3 {
            let o = self.origin[axis];
            let d = self.dir[axis];
            if d.abs() < 1e-14 {
                if o < min[axis] || o > max[axis] {
                    return None;
                }
            } else {
                let inv_d = 1.0 / d;
                let mut t1 = (min[axis] - o) * inv_d;
                let mut t2 = (max[axis] - o) * inv_d;
                if t1 > t2 {
                    std::mem::swap(&mut t1, &mut t2);
                }
                t_min = t_min.max(t1);
                t_max = t_max.min(t2);
                if t_min > t_max {
                    return None;
                }
            }
        }
        Some(t_min)
    }

    /// Two-sided Möller–Trumbore ray-triangle intersection in `f64`, returning ray parameter `t >= 0`.
    pub fn intersect_triangle(&self, v0: DVec3, v1: DVec3, v2: DVec3) -> Option<f64> {
        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let pvec = self.dir.cross(e2);
        let det = e1.dot(pvec);
        if det.abs() < 1e-12 {
            return None;
        }
        let inv_det = 1.0 / det;
        let tvec = self.origin - v0;
        let u = tvec.dot(pvec) * inv_det;
        if !(0.0..=1.0).contains(&u) {
            return None;
        }
        let qvec = tvec.cross(e1);
        let v = self.dir.dot(qvec) * inv_det;
        if v < 0.0 || u + v > 1.0 {
            return None;
        }
        let t = e2.dot(qvec) * inv_det;
        if t >= 0.0 { Some(t) } else { None }
    }

    /// Computes closest point between this forward ray (`t >= 0`) and line segment `a..b`.
    pub fn closest_to_segment(&self, a: DVec3, b: DVec3) -> SegmentHit {
        let seg = b - a;
        let w0 = self.origin - a;
        let seg_len_sq = seg.length_squared();

        if seg_len_sq < 1e-18 {
            let ray_t = (-w0.dot(self.dir)).max(0.0);
            let pt_ray = self.at(ray_t);
            return SegmentHit {
                ray_t,
                seg_u: 0.0,
                distance_mm: pt_ray.distance(a),
                point_on_segment: a,
            };
        }

        // Minimize |(origin + t*dir) - (a + u*seg)|^2 subject to t >= 0, u in [0, 1]
        let b_dot = self.dir.dot(seg);
        let d_dot = self.dir.dot(w0);
        let e_dot = seg.dot(w0);
        let denom = seg_len_sq - b_dot * b_dot;

        let (mut ray_t, mut seg_u) = if denom.abs() < 1e-14 {
            (0.0, (e_dot / seg_len_sq).clamp(0.0, 1.0))
        } else {
            let t = (b_dot * e_dot - seg_len_sq * d_dot) / denom;
            let u = (e_dot - b_dot * d_dot) / denom;
            (t, u)
        };

        if seg_u < 0.0 {
            seg_u = 0.0;
            ray_t = (-d_dot).max(0.0);
        } else if seg_u > 1.0 {
            seg_u = 1.0;
            ray_t = (b - self.origin).dot(self.dir).max(0.0);
        } else if ray_t < 0.0 {
            ray_t = 0.0;
            seg_u = (e_dot / seg_len_sq).clamp(0.0, 1.0);
        }

        let pt_ray = self.at(ray_t);
        let pt_seg = a + seg * seg_u;
        SegmentHit {
            ray_t,
            seg_u,
            distance_mm: pt_ray.distance(pt_seg),
            point_on_segment: pt_seg,
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::units::approx_eq;

    #[test]
    fn ray_triangle_and_segment_intersections_are_accurate() {
        let ray = Ray3::new(DVec3::new(2.0, 2.0, 10.0), DVec3::new(0.0, 0.0, -1.0)).unwrap();
        let t = ray
            .intersect_triangle(
                DVec3::new(0.0, 0.0, 3.5),
                DVec3::new(10.0, 0.0, 3.5),
                DVec3::new(0.0, 10.0, 3.5),
            )
            .unwrap();
        assert!(approx_eq(t, 6.5, 1e-12));

        let seg_hit =
            ray.closest_to_segment(DVec3::new(0.0, 2.25, 3.5), DVec3::new(5.0, 2.25, 3.5));
        assert!(approx_eq(seg_hit.distance_mm, 0.25, 1e-12));
        assert!(approx_eq(seg_hit.ray_t, 6.5, 1e-12));
        assert!(approx_eq(seg_hit.point_on_segment.x, 2.0, 1e-12));
    }
}
