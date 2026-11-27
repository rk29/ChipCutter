use glam::{DMat3, DVec3};
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

/// Computes the 3D circumcircle `(center, unit_normal, radius)` of three non-collinear points.
pub fn circumcircle_3d(a: DVec3, b: DVec3, c: DVec3) -> Option<(DVec3, DVec3, f64)> {
    let ab = b - a;
    let ac = c - a;
    let cross = ab.cross(ac);
    let cross_sq = cross.length_squared();
    if cross_sq < 1e-18 {
        return None;
    }
    let normal = cross / cross_sq.sqrt();
    let to_center = (cross.cross(ab) * ac.length_squared() + ac.cross(cross) * ab.length_squared())
        / (2.0 * cross_sq);
    let center = a + to_center;
    let radius = to_center.length();
    Some((center, normal, radius))
}

fn snap_angle_deg(deg: f64) -> f64 {
    let mut a = deg % 360.0;
    if a > 180.0 {
        a -= 360.0;
    } else if a <= -180.0 {
        a += 360.0;
    }
    // Snap cleanly to exact multiples of 90° within 1e-6° so repeated 90° clicks stay crisp
    let nearest_90 = (a / 90.0).round() * 90.0;
    if (a - nearest_90).abs() <= 1e-6 {
        if nearest_90.abs() <= 1e-6 {
            0.0
        } else if (nearest_90 + 180.0).abs() <= 1e-6 {
            180.0
        } else {
            nearest_90
        }
    } else if a.abs() <= 1e-9 {
        0.0
    } else {
        a
    }
}

/// Extracts canonical XYZ Euler angles `[rx_deg, ry_deg, rz_deg]` such that
/// `Rz(rz) * Ry(ry) * Rx(rx) == r`.
pub fn rotation_matrix_to_xyz_euler_deg(r: DMat3) -> [f64; 3] {
    let r20 = r.x_axis.z.clamp(-1.0, 1.0);
    let ry = (-r20).asin();
    let cos_y = ry.cos();
    let (rx, rz) = if cos_y.abs() > 1e-7 {
        let rx = r.y_axis.z.atan2(r.z_axis.z);
        let rz = r.x_axis.y.atan2(r.x_axis.x);
        (rx, rz)
    } else {
        let rz = 0.0;
        let rx = (-r.z_axis.y).atan2(r.y_axis.y);
        (rx, rz)
    };
    [
        snap_angle_deg(rx.to_degrees()),
        snap_angle_deg(ry.to_degrees()),
        snap_angle_deg(rz.to_degrees()),
    ]
}

/// Simplifies a 3D polyline using exact iterative 3D Ramer–Douglas–Peucker decimation
/// within `tol_mm` chord deviation (unclamped general tolerance `>= 0.0`).
pub fn simplify_3d_polyline_rdp(pts: &[DVec3], tol_mm: f64) -> Vec<DVec3> {
    if pts.len() <= 2 {
        return pts.to_vec();
    }
    let tol = if tol_mm.is_finite() {
        tol_mm.max(0.0)
    } else {
        0.0
    };
    let tol_sq = tol * tol;

    let mut keep = vec![false; pts.len()];
    keep[0] = true;
    keep[pts.len() - 1] = true;

    let mut stack = vec![(0usize, pts.len() - 1)];
    while let Some((start, end)) = stack.pop() {
        if end <= start + 1 {
            continue;
        }
        let a = pts[start];
        let b = pts[end];
        let ab = b - a;
        let ab_len_sq = ab.length_squared();

        let mut max_dist_sq = -1.0_f64;
        let mut max_idx = start;

        for (i, &p) in pts.iter().enumerate().take(end).skip(start + 1) {
            let dist_sq = if ab_len_sq > 1e-18 {
                let t = ((p - a).dot(ab) / ab_len_sq).clamp(0.0, 1.0);
                let proj = a + ab * t;
                (p - proj).length_squared()
            } else {
                (p - a).length_squared()
            };
            if dist_sq > max_dist_sq {
                max_dist_sq = dist_sq;
                max_idx = i;
            }
        }

        if max_dist_sq > tol_sq {
            keep[max_idx] = true;
            stack.push((start, max_idx));
            stack.push((max_idx, end));
        }
    }

    let mut out = Vec::with_capacity(pts.len());
    for (i, &p) in pts.iter().enumerate() {
        if keep[i] {
            out.push(p);
        }
    }
    out
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::units::approx_eq;

    #[test]
    fn ray_closest_to_segment_is_accurate() {
        let ray = Ray3::new(DVec3::new(2.0, 2.0, 10.0), DVec3::new(0.0, 0.0, -1.0)).unwrap();
        let seg_hit =
            ray.closest_to_segment(DVec3::new(0.0, 2.25, 3.5), DVec3::new(5.0, 2.25, 3.5));
        assert!(approx_eq(seg_hit.distance_mm, 0.25, 1e-12));
        assert!(approx_eq(seg_hit.ray_t, 6.5, 1e-12));
        assert!(approx_eq(seg_hit.point_on_segment.x, 2.0, 1e-12));
    }

    #[test]
    fn circumcircle_3d_euler_and_rdp_3d_are_accurate() {
        let (c, n, r) = circumcircle_3d(
            DVec3::new(5.0, 0.0, 2.0),
            DVec3::new(0.0, 5.0, 2.0),
            DVec3::new(-5.0, 0.0, 2.0),
        )
        .unwrap();
        assert!(c.distance(DVec3::new(0.0, 0.0, 2.0)) < 1e-12);
        assert!(n.distance(DVec3::Z) < 1e-12);
        assert!(approx_eq(r, 5.0, 1e-12));

        let rot = DMat3::from_rotation_z(90.0_f64.to_radians());
        let eul = rotation_matrix_to_xyz_euler_deg(rot);
        assert!(approx_eq(eul[0], 0.0, 1e-9));
        assert!(approx_eq(eul[1], 0.0, 1e-9));
        assert!(approx_eq(eul[2], 90.0, 1e-9));

        let pts = vec![
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(2.5, 0.0, 0.0),
            DVec3::new(5.0, 0.0, 2.0),
            DVec3::new(7.5, 0.0, 1.0),
            DVec3::new(10.0, 0.0, 0.0),
        ];
        let out = simplify_3d_polyline_rdp(&pts, 0.005);
        assert_eq!(out.len(), 4);
        assert!(out.contains(&DVec3::new(5.0, 0.0, 2.0)));
    }
}
