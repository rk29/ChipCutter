use chip_math::{Aabb3, DVec2, DVec3};
use rayon::prelude::*;

use crate::types::{
    Axis3, BatchSweepOutput, RadialCutterProfile, RotaryStockSpec, SpanRuleViolations,
    StockVerifyTolerances, SweepChord, SweepMotion, SweepToolSpec,
};

use super::grid::{
    DexelSpan, INLINE_DEXEL_SLOTS, TriDexelStock, load_row_ray_into, query_overlap_length,
    query_target_gouge_depth, subtract_dexel_interval, write_row_ray_from,
};

/// Canonical axis-aligned spindle orientation in workpiece-fixed coordinates (`0°`, `90°`, `180°`,
/// `270°` indexed 3+1 or standard 3-axis `+Z`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CardinalSpindle {
    PosZ = 0,
    NegZ = 1,
    PosY = 2,
    NegY = 3,
    PosX = 4,
    NegX = 5,
}

impl CardinalSpindle {
    #[inline]
    pub(crate) fn from_unit_vec(s: DVec3) -> Option<Self> {
        const TOL: f64 = 1e-5;
        if (s.z - 1.0).abs() <= TOL && s.x.abs() <= TOL && s.y.abs() <= TOL {
            Some(Self::PosZ)
        } else if (s.z + 1.0).abs() <= TOL && s.x.abs() <= TOL && s.y.abs() <= TOL {
            Some(Self::NegZ)
        } else if (s.y - 1.0).abs() <= TOL && s.x.abs() <= TOL && s.z.abs() <= TOL {
            Some(Self::PosY)
        } else if (s.y + 1.0).abs() <= TOL && s.x.abs() <= TOL && s.z.abs() <= TOL {
            Some(Self::NegY)
        } else if (s.x - 1.0).abs() <= TOL && s.y.abs() <= TOL && s.z.abs() <= TOL {
            Some(Self::PosX)
        } else if (s.x + 1.0).abs() <= TOL && s.y.abs() <= TOL && s.z.abs() <= TOL {
            Some(Self::NegX)
        } else {
            None
        }
    }

    #[cfg(feature = "gpu-compute")]
    #[inline]
    pub(crate) const fn code(self) -> u32 {
        self as u32
    }

    #[inline]
    pub(crate) const fn axis(self) -> Axis3 {
        match self {
            Self::PosZ | Self::NegZ => Axis3::Z,
            Self::PosY | Self::NegY => Axis3::Y,
            Self::PosX | Self::NegX => Axis3::X,
        }
    }

    #[inline]
    pub(crate) const fn sign(self) -> f64 {
        match self {
            Self::PosZ | Self::PosY | Self::PosX => 1.0,
            Self::NegZ | Self::NegY | Self::NegX => -1.0,
        }
    }
}

/// Single arbitrarily oriented rotary cutter pose in workpiece-fixed coordinates (`tip`, `spindle`)
/// alongside the original WCS tip position (`tip_wcs`) for diagnostic reporting.
#[derive(Debug, Clone, Copy)]
pub(crate) struct OrientedPose3D {
    pub(crate) tip: DVec3,
    pub(crate) spindle: DVec3,
    pub(crate) tip_wcs: DVec3,
}

/// Workpiece-fixed volumetric sweep primitive emitted by [`decompose_chord_for_tridexel`].
#[derive(Debug, Clone, Copy)]
pub(crate) enum SweptPrimitive {
    CardinalLinear {
        chord: LinearChord3D,
        spindle: CardinalSpindle,
        rot_deg: f64,
    },
    OrientedPose(OrientedPose3D),
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct LinearChord3D {
    pub(crate) p0: DVec3,
    pub(crate) p1: DVec3,
}

impl LinearChord3D {
    #[inline]
    fn min_z(self) -> f64 {
        self.p0.z.min(self.p1.z)
    }

    #[inline]
    fn max_z(self) -> f64 {
        self.p0.z.max(self.p1.z)
    }
}

/// Exact closed-form intersection of horizontal line `y = y_line` with the 2D stadium/capsule
/// of radius `r >= 0` swept along segment `p0 -> p1`.
#[inline]
pub(crate) fn intersect_horizontal_line_with_capsule_2d(
    p0: DVec2,
    p1: DVec2,
    r: f64,
    y_line: f64,
) -> Option<(f64, f64)> {
    if r < 0.0 || !r.is_finite() {
        return None;
    }
    let y_min = p0.y.min(p1.y) - r;
    let y_max = p0.y.max(p1.y) + r;
    if y_line < y_min || y_line > y_max {
        return None;
    }

    let r2 = r * r;
    let mut x_lo = f64::INFINITY;
    let mut x_hi = f64::NEG_INFINITY;

    let dy0 = y_line - p0.y;
    let d0_sq = r2 - dy0 * dy0;
    if d0_sq >= 0.0 {
        let dx = d0_sq.sqrt();
        x_lo = x_lo.min(p0.x - dx);
        x_hi = x_hi.max(p0.x + dx);
    }

    let dy1 = y_line - p1.y;
    let d1_sq = r2 - dy1 * dy1;
    if d1_sq >= 0.0 {
        let dx = d1_sq.sqrt();
        x_lo = x_lo.min(p1.x - dx);
        x_hi = x_hi.max(p1.x + dx);
    }

    let seg_v = p1 - p0;
    let len = seg_v.length();
    if len > 1e-12 {
        let n = DVec2::new(-seg_v.y / len, seg_v.x / len) * r;
        for sign in [-1.0, 1.0] {
            let a = p0 + n * sign;
            let b = p1 + n * sign;
            let dy = b.y - a.y;
            if dy.abs() > 1e-12 {
                let s = (y_line - a.y) / dy;
                if (0.0..=1.0).contains(&s) {
                    let xc = a.x + s * (b.x - a.x);
                    x_lo = x_lo.min(xc);
                    x_hi = x_hi.max(xc);
                }
            }
        }
    }

    if x_lo.is_finite() && x_hi.is_finite() && x_hi >= x_lo {
        Some((x_lo, x_hi))
    } else {
        None
    }
}

#[inline]
fn clamp_unit_interval(a: f64, b: f64) -> Option<(f64, f64)> {
    let lo = a.min(b);
    let hi = a.max(b);
    if hi < 0.0 || lo > 1.0 {
        return None;
    }
    Some((lo.max(0.0), hi.min(1.0)))
}

/// Computes the exact horizontal cut interval `[w_min, w_max]` along an X-ray (`swap_xy = false`,
/// fixed `y = fixed_u, z = pz`) or Y-ray (`swap_xy = true`, fixed `x = fixed_u, z = pz`) for a
/// vertical-spindle cutter moving linearly from `span.p0` to `span.p1`.
#[inline]
fn eval_horizontal_ray_cut_interval(
    span: LinearChord3D,
    prof: &RadialCutterProfile,
    radial_shrink_mm: f64,
    pz: f64,
    fixed_u: f64,
    swap_xy: bool,
    dz_step_mm: f64,
) -> Option<(f64, f64)> {
    let max_h = prof.horizontal_cut_max_height_mm();
    if pz < span.min_z() - 1e-9 || pz > span.max_z() + max_h + 1e-9 {
        return None;
    }

    let proj2 = |p: DVec3| -> DVec2 {
        if swap_xy {
            DVec2::new(p.y, p.x)
        } else {
            DVec2::new(p.x, p.y)
        }
    };

    let eval_r_at_dz = |dz: f64| -> Option<f64> {
        if prof.has_bounded_undercut_head {
            prof.cutting_radius_at_dz(dz)
        } else {
            prof.radius_at_height_dz(dz)
        }
    };

    let dz_span = span.p1.z - span.p0.z;
    if dz_span.abs() <= 1e-7 {
        let dz = pz - span.p0.z;
        if dz <= 1e-9 || dz > max_h + 1e-9 {
            return None;
        }
        let r = (eval_r_at_dz(dz)? - radial_shrink_mm).max(0.0);
        if r <= 1e-9 {
            return None;
        }
        return intersect_horizontal_line_with_capsule_2d(
            proj2(span.p0),
            proj2(span.p1),
            r,
            fixed_u,
        );
    }

    let t_at_z = |target_z: f64| (target_z - span.p0.z) / dz_span;
    let (t0, t1) = clamp_unit_interval(t_at_z(pz - max_h), t_at_z(pz))?;

    if prof.is_thread_mill {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        let mut merge_hit = |cand: Option<(f64, f64)>| {
            if let Some((c0, c1)) = cand {
                lo = lo.min(c0);
                hi = hi.max(c1);
            }
        };

        let r_neck = (prof.neck_radius_mm - radial_shrink_mm).max(0.0);
        if r_neck > 1e-9
            && prof.neck_length_mm > prof.flute_length_mm + 1e-9
            && let Some((n0, n1)) = clamp_unit_interval(
                t_at_z(pz - prof.neck_length_mm),
                t_at_z(pz - prof.flute_length_mm),
            )
        {
            let qn0 = span.p0.lerp(span.p1, n0);
            let qn1 = span.p0.lerp(span.p1, n1);
            merge_hit(intersect_horizontal_line_with_capsule_2d(
                proj2(qn0),
                proj2(qn1),
                r_neck,
                fixed_u,
            ));
        }

        if let Some((f0, f1)) = clamp_unit_interval(t_at_z(pz - prof.flute_length_mm), t_at_z(pz)) {
            let qf0 = span.p0.lerp(span.p1, f0);
            let qf1 = span.p0.lerp(span.p1, f1);
            let sub_dz = (qf1.z - qf0.z).abs();
            let (z_start, tooth_step, n_teeth, _tan_half) = prof.thread_mill_tooth_band_params();
            if sub_dz <= 1e-7 {
                let r_mid =
                    (prof.thread_mill_tooth_radius_at_z(pz - qf0.z) - radial_shrink_mm).max(0.0);
                if r_mid > 1e-9 {
                    merge_hit(intersect_horizontal_line_with_capsule_2d(
                        proj2(qf0),
                        proj2(qf1),
                        r_mid,
                        fixed_u,
                    ));
                }
            } else {
                let step = dz_step_mm.min(0.15 * tooth_step).max(0.02);
                let n_sub = ((sub_dz / step).ceil() as usize).clamp(2, 32);
                for s in 0..n_sub {
                    let u0 = s as f64 / n_sub as f64;
                    let u1 = (s + 1) as f64 / n_sub as f64;
                    let a = qf0.lerp(qf1, u0);
                    let b = qf0.lerp(qf1, u1);
                    let r_mid = (prof.thread_mill_tooth_radius_at_z(pz - 0.5 * (a.z + b.z))
                        - radial_shrink_mm)
                        .max(0.0);
                    if r_mid > 1e-9 {
                        merge_hit(intersect_horizontal_line_with_capsule_2d(
                            proj2(a),
                            proj2(b),
                            r_mid,
                            fixed_u,
                        ));
                    }
                }
                let r_crest = (prof.cutting_radius_mm - radial_shrink_mm).max(0.0);
                if r_crest > 1e-9 {
                    for k in 0..n_teeth {
                        let z_crest = z_start + (k as f64 + 0.5) * tooth_step;
                        let tc = t_at_z(pz - z_crest);
                        if tc >= f0 - 1e-9 && tc <= f1 + 1e-9 {
                            let pt = proj2(span.p0.lerp(span.p1, tc.clamp(f0, f1)));
                            merge_hit(intersect_horizontal_line_with_capsule_2d(
                                pt, pt, r_crest, fixed_u,
                            ));
                        }
                    }
                }
            }
        }

        return if lo.is_finite() && hi.is_finite() && hi > lo {
            Some((lo, hi))
        } else {
            None
        };
    }

    let q0 = span.p0.lerp(span.p1, t0);
    let q1 = span.p0.lerp(span.p1, t1);
    let r_at = |z_tip: f64| -> f64 {
        (eval_r_at_dz(pz - z_tip).unwrap_or(0.0) - radial_shrink_mm).max(0.0)
    };

    let r0 = r_at(q0.z);
    let r1 = r_at(q1.z);
    if !prof.has_relieved_neck && (r0 - r1).abs() <= 1e-7 && r0 > 1e-9 {
        return intersect_horizontal_line_with_capsule_2d(proj2(q0), proj2(q1), r0, fixed_u);
    }

    let sub_dz = (q1.z - q0.z).abs();
    let n_sub_z = (sub_dz / dz_step_mm.max(0.05)).ceil() as usize;
    let n_sub_r = ((r1 - r0).abs() / 0.015).ceil() as usize;
    let n_sub = n_sub_z.max(n_sub_r).clamp(2, 32);
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for s in 0..n_sub {
        let u0 = s as f64 / n_sub as f64;
        let u1 = (s + 1) as f64 / n_sub as f64;
        let a = q0.lerp(q1, u0);
        let b = q0.lerp(q1, u1);
        let r_mid = r_at(0.5 * (a.z + b.z));
        if r_mid > 1e-9
            && let Some((c0, c1)) =
                intersect_horizontal_line_with_capsule_2d(proj2(a), proj2(b), r_mid, fixed_u)
        {
            lo = lo.min(c0);
            hi = hi.max(c1);
        }
    }
    if let chip_tools::CutterEnvelopeShape::Lollipop {
        sphere_radius_mm, ..
    } = prof.envelope().shape()
    {
        let tc = t_at_z(pz - sphere_radius_mm);
        if tc >= t0 - 1e-9 && tc <= t1 + 1e-9 {
            let pt = proj2(span.p0.lerp(span.p1, tc.clamp(t0, t1)));
            let r_eq = (prof.cutting_radius_mm - radial_shrink_mm).max(0.0);
            if r_eq > 1e-9
                && let Some((c0, c1)) =
                    intersect_horizontal_line_with_capsule_2d(pt, pt, r_eq, fixed_u)
            {
                lo = lo.min(c0);
                hi = hi.max(c1);
            }
        }
    }
    if lo.is_finite() && hi.is_finite() && hi > lo {
        Some((lo, hi))
    } else {
        None
    }
}

#[inline]
fn eval_horizontal_ray_cylinder_interval(
    span: LinearChord3D,
    cyl_radius_mm: f64,
    cyl_height_mm: f64,
    pz: f64,
    fixed_u: f64,
    swap_xy: bool,
) -> Option<(f64, f64)> {
    if cyl_radius_mm <= 1e-9 || pz < span.min_z() - 1e-9 || pz > span.max_z() + cyl_height_mm + 1e-9
    {
        return None;
    }
    let proj2 = |p: DVec3| -> DVec2 {
        if swap_xy {
            DVec2::new(p.y, p.x)
        } else {
            DVec2::new(p.x, p.y)
        }
    };
    let dz_span = span.p1.z - span.p0.z;
    if dz_span.abs() <= 1e-7 {
        let dz = pz - span.p0.z;
        if dz <= 1e-9 || dz > cyl_height_mm + 1e-9 {
            return None;
        }
        return intersect_horizontal_line_with_capsule_2d(
            proj2(span.p0),
            proj2(span.p1),
            cyl_radius_mm,
            fixed_u,
        );
    }
    let t_at_z = |target_z: f64| (target_z - span.p0.z) / dz_span;
    let (t0, t1) = clamp_unit_interval(t_at_z(pz - cyl_height_mm), t_at_z(pz))?;
    let q0 = span.p0.lerp(span.p1, t0);
    let q1 = span.p0.lerp(span.p1, t1);
    intersect_horizontal_line_with_capsule_2d(proj2(q0), proj2(q1), cyl_radius_mm, fixed_u)
}

#[inline]
fn eval_z_ray_cylinder_interval(
    span: LinearChord3D,
    px: f64,
    py: f64,
    cyl_radius_mm: f64,
    cyl_height_mm: f64,
) -> Option<(f64, f64)> {
    if cyl_radius_mm <= 1e-9 {
        return None;
    }
    let p0_xy = DVec2::new(span.p0.x, span.p0.y);
    let p1_xy = DVec2::new(span.p1.x, span.p1.y);
    let q_xy = DVec2::new(px, py);
    let d_xy = p1_xy - p0_xy;
    let len2 = d_xy.length_squared();

    let (t0, t1) = if len2 <= 1e-12 {
        if (q_xy - p0_xy).length() > cyl_radius_mm + 1e-9 {
            return None;
        }
        (0.0, 1.0)
    } else {
        let f = p0_xy - q_xy;
        let b = 2.0 * f.dot(d_xy);
        let c = f.length_squared() - cyl_radius_mm * cyl_radius_mm;
        let disc = b * b - 4.0 * len2 * c;
        if disc < 0.0 {
            return None;
        }
        let sq = disc.sqrt();
        let inv2a = 0.5 / len2;
        let raw_t0 = (-b - sq) * inv2a;
        let raw_t1 = (-b + sq) * inv2a;
        if raw_t1 < 0.0 || raw_t0 > 1.0 {
            return None;
        }
        (raw_t0.max(0.0), raw_t1.min(1.0))
    };

    let dz_span = span.p1.z - span.p0.z;
    let z0 = span.p0.z + t0 * dz_span;
    let z1 = span.p0.z + t1 * dz_span;
    Some((z0.min(z1), z0.max(z1) + cyl_height_mm))
}

#[inline]
fn eval_z_ray_thread_mill_intervals(
    span: LinearChord3D,
    prof: &RadialCutterProfile,
    px: f64,
    py: f64,
    radius_limit: f64,
    out: &mut Vec<(f64, f64)>,
) {
    out.clear();
    let p0_xy = DVec2::new(span.p0.x, span.p0.y);
    let p1_xy = DVec2::new(span.p1.x, span.p1.y);
    let q_xy = DVec2::new(px, py);
    let d_xy = p1_xy - p0_xy;
    let len2 = d_xy.length_squared();

    let (t_enter, t_exit, t_closest) = if len2 <= 1e-12 {
        let r = (q_xy - p0_xy).length();
        if r > radius_limit + 1e-9 {
            return;
        }
        (0.0, 1.0, 0.0)
    } else {
        let f = p0_xy - q_xy;
        let b = 2.0 * f.dot(d_xy);
        let c = f.length_squared() - radius_limit * radius_limit;
        let disc = b * b - 4.0 * len2 * c;
        if disc < 0.0 {
            return;
        }
        let sq = disc.sqrt();
        let inv2a = 0.5 / len2;
        let raw_t0 = (-b - sq) * inv2a;
        let raw_t1 = (-b + sq) * inv2a;
        if raw_t1 < 0.0 || raw_t0 > 1.0 {
            return;
        }
        let t0 = raw_t0.max(0.0);
        let t1 = raw_t1.min(1.0);
        let tc = (-f.dot(d_xy) / len2).clamp(t0, t1);
        (t0, t1, tc)
    };

    let dz_span = span.p1.z - span.p0.z;
    if len2 <= 1e-12 || dz_span.abs() <= 1e-7 {
        let pt = p0_xy.lerp(p1_xy, t_closest);
        let r = (q_xy - pt).length();
        prof.thread_mill_axial_intervals_at_radius(r, out);
        let z_lo_tip = span.min_z();
        let z_hi_tip = span.max_z();
        for (a, b) in out.iter_mut() {
            *a += z_lo_tip;
            *b += z_hi_tip;
        }
        return;
    }

    let ts = [
        t_enter,
        t_enter + 0.25 * (t_exit - t_enter),
        t_enter + 0.50 * (t_exit - t_enter),
        t_enter + 0.75 * (t_exit - t_enter),
        t_exit,
        t_closest,
    ];
    let mut samples = [(f64::INFINITY, 0.0_f64); 6];
    let mut neck_lo = f64::INFINITY;
    let mut neck_hi = f64::NEG_INFINITY;
    for (idx, &t) in ts.iter().enumerate() {
        let pt = p0_xy.lerp(p1_xy, t);
        let r = (q_xy - pt).length();
        let z_tip = span.p0.z + t * dz_span;
        samples[idx] = (r, z_tip);
        if r <= prof.neck_radius_mm + 1e-9 {
            neck_lo = neck_lo.min(z_tip);
            neck_hi = neck_hi.max(z_tip + prof.neck_length_mm);
        }
    }

    let (z_start, tooth_step, n_teeth, tan_half) = prof.thread_mill_tooth_band_params();
    for k in 0..n_teeth {
        let z_k_lo = z_start + (k as f64) * tooth_step;
        let z_k_hi = z_k_lo + tooth_step;
        let z_crest = z_k_lo + 0.5 * tooth_step;
        let mut t_lo = f64::INFINITY;
        let mut t_hi = f64::NEG_INFINITY;
        for &(r, z_tip) in &samples {
            if r <= radius_limit + 1e-9 {
                let half_w = (prof.cutting_radius_mm - r).max(0.0) * tan_half;
                let lo = z_tip + (z_crest - half_w).max(z_k_lo);
                let hi = z_tip + (z_crest + half_w).min(z_k_hi);
                if hi > lo + 1e-9 {
                    t_lo = t_lo.min(lo);
                    t_hi = t_hi.max(hi);
                }
            }
        }
        if t_hi > t_lo + 1e-9 {
            out.push((t_lo, t_hi));
        }
    }
    if neck_hi > neck_lo + 1e-9 {
        out.push((neck_lo, neck_hi));
    }
}

#[inline]
fn eval_z_ray_cut_interval(
    span: LinearChord3D,
    prof: &RadialCutterProfile,
    is_flat_endmill: bool,
    px: f64,
    py: f64,
    radius_limit: f64,
) -> Option<(f64, f64, f64)> {
    let p0_xy = DVec2::new(span.p0.x, span.p0.y);
    let p1_xy = DVec2::new(span.p1.x, span.p1.y);
    let q_xy = DVec2::new(px, py);
    let d_xy = p1_xy - p0_xy;
    let len2 = d_xy.length_squared();

    let (t_enter, t_exit, t_closest) = if len2 <= 1e-12 {
        let r = (q_xy - p0_xy).length();
        if r > radius_limit + 1e-9 {
            return None;
        }
        (0.0, 1.0, 0.0)
    } else {
        let f = p0_xy - q_xy;
        let b = 2.0 * f.dot(d_xy);
        let c = f.length_squared() - radius_limit * radius_limit;
        let disc = b * b - 4.0 * len2 * c;
        if disc < 0.0 {
            return None;
        }
        let sq = disc.sqrt();
        let inv2a = 0.5 / len2;
        let raw_t0 = (-b - sq) * inv2a;
        let raw_t1 = (-b + sq) * inv2a;
        if raw_t1 < 0.0 || raw_t0 > 1.0 {
            return None;
        }
        let t0 = raw_t0.max(0.0);
        let t1 = raw_t1.min(1.0);
        let tc = (-f.dot(d_xy) / len2).clamp(t0, t1);
        (t0, t1, tc)
    };

    let dz_span = span.p1.z - span.p0.z;
    if dz_span.abs() <= 1e-7 {
        let pt = p0_xy.lerp(p1_xy, t_closest);
        let r = (q_xy - pt).length();
        let (dz_lo, dz_hi) = prof.cutting_z_interval_at_radius(r)?;
        let z_tip = span.p0.z;
        return Some((z_tip + dz_lo, z_tip + dz_hi, z_tip));
    }

    if is_flat_endmill {
        let t_min_z = if dz_span < 0.0 { t_exit } else { t_enter };
        let z_tip_lo = span.p0.z + t_min_z * dz_span;
        let t_max_z = if dz_span < 0.0 { t_enter } else { t_exit };
        let z_tip_hi = span.p0.z + t_max_z * dz_span;
        return Some((z_tip_lo, z_tip_hi + prof.flute_length_mm, z_tip_lo));
    }

    let mut best_z_cut = f64::INFINITY;
    let mut best_z_tip = f64::INFINITY;
    let mut max_z_hi = f64::NEG_INFINITY;
    let mut consider_t = |t: f64| {
        let pt = p0_xy.lerp(p1_xy, t);
        let r = (q_xy - pt).length();
        if let Some((dz_lo, dz_hi)) = prof.cutting_z_interval_at_radius(r) {
            let z_tip = span.p0.z + t * dz_span;
            let z_cut = z_tip + dz_lo;
            let z_hi = z_tip + dz_hi;
            if z_cut < best_z_cut {
                best_z_cut = z_cut;
                best_z_tip = z_tip;
            }
            if z_hi > max_z_hi {
                max_z_hi = z_hi;
            }
        }
    };
    let n_samples = 8usize;
    for s in 0..=n_samples {
        let u = s as f64 / n_samples as f64;
        consider_t(t_enter + u * (t_exit - t_enter));
    }
    consider_t(t_closest);
    if best_z_cut.is_finite() && max_z_hi.is_finite() {
        Some((best_z_cut, max_z_hi.max(best_z_cut), best_z_tip))
    } else {
        None
    }
}

struct SweepContext<'a> {
    prof: &'a RadialCutterProfile,
    is_flat_endmill: bool,
    is_rapid: bool,
    tol: StockVerifyTolerances,
    op_idx: u32,
    target: Option<&'a TriDexelStock>,
    rotary: Option<RotaryStockSpec>,
    rot_deg: f64,
}

impl SweepContext<'_> {
    #[inline]
    fn to_diag_wcs(&self, pt_part: DVec3) -> DVec3 {
        if let Some(rot) = self.rotary
            && self.rot_deg.abs() > 1e-6
        {
            rot.rotate_point_wcs(pt_part, self.rot_deg)
        } else {
            pt_part
        }
    }
}

#[inline]
fn aabb_overlaps(min_a: DVec3, max_a: DVec3, stock: Aabb3) -> bool {
    max_a.x >= stock.min.x
        && min_a.x <= stock.max.x
        && max_a.y >= stock.min.y
        && min_a.y <= stock.max.y
        && max_a.z >= stock.min.z
        && min_a.z <= stock.max.z
}

/// Computes the tight stock-intersecting world-space AABB `(aabb_min, aabb_max)` for an arbitrary
/// [`OrientedPose3D`] together with `check_holder` (`true` only when the holder/collet envelope
/// above `stickout_length` actually intersects `stock_aabb`). Returns `None` when no part of the
/// cutter/shank/holder intersects `stock_aabb` (e.g., clearance rapids above the workpiece).
#[inline]
pub(crate) fn oriented_pose_stock_aabb(
    pose: OrientedPose3D,
    prof: &RadialCutterProfile,
    stock_aabb: Aabb3,
    verify: bool,
) -> Option<(DVec3, DVec3, bool)> {
    let r_cut = prof.cutting_radius_mm.max(0.05);
    let max_h = prof.horizontal_cut_max_height_mm();
    let flute_top = pose.tip + pose.spindle * max_h;
    let mut bmin = pose.tip.min(flute_top) - DVec3::splat(r_cut);
    let mut bmax = pose.tip.max(flute_top) + DVec3::splat(r_cut);
    let mut active = aabb_overlaps(bmin, bmax, stock_aabb);
    let mut check_holder = false;

    if verify {
        let stickout = prof.stickout_length_mm.max(prof.flute_length_mm);
        let reach = prof.effective_reach_mm();
        if stickout > reach + 1e-4 {
            let s_lo = pose.tip + pose.spindle * reach;
            let s_hi = pose.tip + pose.spindle * stickout;
            let r_sh = prof.shank_radius_mm.max(r_cut);
            let smin = s_lo.min(s_hi) - DVec3::splat(r_sh);
            let smax = s_lo.max(s_hi) + DVec3::splat(r_sh);
            if aabb_overlaps(smin, smax, stock_aabb) {
                if active {
                    bmin = bmin.min(smin);
                    bmax = bmax.max(smax);
                } else {
                    bmin = smin;
                    bmax = smax;
                    active = true;
                }
            }
        }
        let h_lo = pose.tip + pose.spindle * stickout;
        let h_hi = pose.tip + pose.spindle * (stickout + 40.0);
        let r_h = prof.holder_radius_mm.max(r_cut);
        let hmin = h_lo.min(h_hi) - DVec3::splat(r_h);
        let hmax = h_lo.max(h_hi) + DVec3::splat(r_h);
        if aabb_overlaps(hmin, hmax, stock_aabb) {
            check_holder = true;
            if active {
                bmin = bmin.min(hmin);
                bmax = bmax.max(hmax);
            } else {
                bmin = hmin;
                bmax = hmax;
                active = true;
            }
        }
    }

    active.then_some((bmin, bmax, check_holder))
}

#[inline]
fn check_rotary_chuck_pose(
    rot: RotaryStockSpec,
    tip_wcs: DVec3,
    tip_part: DVec3,
    r_cut: f64,
    src_chord_idx: u32,
    violations: &mut SpanRuleViolations,
) {
    let (u_tip, _, r_tip) = rot.wcs_to_cylindrical(tip_part);
    let chuck_clearance_r = rot.chuck_radius_mm + r_cut;
    let u_overlaps_chuck = (u_tip + r_cut) >= rot.chuck_u_min && (u_tip - r_cut) <= rot.chuck_u_max;
    if u_overlaps_chuck && r_tip < chuck_clearance_r - 0.05 {
        let excess = chuck_clearance_r - r_tip;
        violations.record_holder_at(excess, tip_wcs, src_chord_idx);
    }
}

/// Decomposes a [`SweepChord`] into workpiece-fixed [`SweptPrimitive`] items shared identically
/// by both CPU `TriDexelStock` and GPU `GpuTriDexelState`, also checking rotary chuck keep-out
/// when `verify == true` and `rotary` is active.
#[allow(clippy::too_many_arguments)]
pub(crate) fn decompose_chord_for_tridexel(
    ch: &SweepChord,
    src_chord_idx: u32,
    rotary: Option<RotaryStockSpec>,
    min_pitch_mm: f64,
    prof: &RadialCutterProfile,
    verify: bool,
    skip_start_pose: bool,
    arc_buf: &mut Vec<LinearChord3D>,
    out_prims: &mut Vec<SweptPrimitive>,
    out_chuck_viol: &mut SpanRuleViolations,
) {
    let Some(rot) = rotary else {
        motion_to_linear_chords(ch.motion, arc_buf);
        for &lin in arc_buf.iter() {
            out_prims.push(SweptPrimitive::CardinalLinear {
                chord: lin,
                spindle: CardinalSpindle::PosZ,
                rot_deg: 0.0,
            });
        }
        return;
    };

    let has_chuck = verify && rot.chuck_u_max > rot.chuck_u_min + 1e-4;
    let r_cut = prof.cutting_radius_mm.max(0.05);
    let d_deg = ch.motion.rotary_delta_deg();

    if d_deg <= 1e-5 {
        let rot_deg = ch.motion.sample_rotary_deg(0.0);
        let spindle_part = rot.rotate_vector_wcs(DVec3::Z, -rot_deg);
        if let Some(cardinal) = CardinalSpindle::from_unit_vec(spindle_part) {
            motion_to_linear_chords(ch.motion, arc_buf);
            for &lin in arc_buf.iter() {
                let p0_part = if rot_deg.abs() > 1e-6 {
                    rot.rotate_point_wcs(lin.p0, -rot_deg)
                } else {
                    lin.p0
                };
                let p1_part = if rot_deg.abs() > 1e-6 {
                    rot.rotate_point_wcs(lin.p1, -rot_deg)
                } else {
                    lin.p1
                };
                if has_chuck {
                    check_rotary_chuck_pose(
                        rot,
                        lin.p0,
                        p0_part,
                        r_cut,
                        src_chord_idx,
                        out_chuck_viol,
                    );
                    let mid_wcs = lin.p0.lerp(lin.p1, 0.5);
                    let mid_part = p0_part.lerp(p1_part, 0.5);
                    check_rotary_chuck_pose(
                        rot,
                        mid_wcs,
                        mid_part,
                        r_cut,
                        src_chord_idx,
                        out_chuck_viol,
                    );
                    check_rotary_chuck_pose(
                        rot,
                        lin.p1,
                        p1_part,
                        r_cut,
                        src_chord_idx,
                        out_chuck_viol,
                    );
                }
                out_prims.push(SweptPrimitive::CardinalLinear {
                    chord: LinearChord3D {
                        p0: p0_part,
                        p1: p1_part,
                    },
                    spindle: cardinal,
                    rot_deg,
                });
            }
            return;
        }
    }

    // Non-cardinal constant angle (e.g. 30°/45° indexed) or simultaneous 4-axis rotary motion:
    // subdivide into workpiece-fixed oriented cutter poses scaled cleanly to voxel pitch and
    // effective rotary contact radius so scallops stay << voxel pitch without pose explosion.
    let step_lin = (0.5 * min_pitch_mm)
        .clamp(0.08, 0.30)
        .min((0.45 * r_cut).max(0.08));
    let p0_wcs = ch.motion.sample_point(0.0);
    let p1_wcs = ch.motion.sample_point(1.0);
    let (_, _, r0) = rot.wcs_to_cylindrical(p0_wcs);
    let (_, _, r1) = rot.wcs_to_cylindrical(p1_wcs);
    let r_ref = (r0.max(r1).min(rot.stock_outer_radius_mm) + r_cut)
        .clamp(r_cut.max(1.0), rot.stock_outer_radius_mm + r_cut);
    let step_deg = (step_lin / r_ref).to_degrees().clamp(0.35, 1.5);
    let lin_len = ch.motion.length_mm();
    let n_lin = (lin_len / step_lin).ceil() as usize;
    let n_rot = (d_deg / step_deg).ceil() as usize;
    let n_steps = n_lin.max(n_rot).clamp(1, 2_000);
    let k_start = usize::from(skip_start_pose);

    for k in k_start..=n_steps {
        let t = k as f64 / n_steps as f64;
        let tip_wcs = ch.motion.sample_point(t);
        let rot_deg = ch.motion.sample_rotary_deg(t);
        let tip_part = rot.rotate_point_wcs(tip_wcs, -rot_deg);
        let spindle_part = rot.rotate_vector_wcs(DVec3::Z, -rot_deg);
        if has_chuck {
            check_rotary_chuck_pose(rot, tip_wcs, tip_part, r_cut, src_chord_idx, out_chuck_viol);
        }
        out_prims.push(SweptPrimitive::OrientedPose(OrientedPose3D {
            tip: tip_part,
            spindle: spindle_part,
            tip_wcs,
        }));
    }
}

/// Solves ray `P(w) = p_base + w * d_axis` intersection with an oriented finite cylinder of
/// radius `radius` and axial span `h(w) ∈ [h_lo, h_hi]` along `spindle`.
#[inline]
#[allow(clippy::too_many_arguments)]
fn solve_oriented_cylinder_interval(
    a: f64,
    b: f64,
    c: f64,
    h0: f64,
    d_dot_s: f64,
    radius: f64,
    h_lo: f64,
    h_hi: f64,
) -> Option<(f64, f64)> {
    if radius <= 1e-9 || h_hi <= h_lo + 1e-9 {
        return None;
    }
    let c_eff = c - radius * radius;
    let (mut w_lo, mut w_hi) = if a <= 1e-12 {
        if c_eff <= 0.0 {
            (-1e6, 1e6)
        } else {
            return None;
        }
    } else {
        let disc = b * b - a * c_eff;
        if disc < 0.0 {
            return None;
        }
        let sd = disc.sqrt();
        ((-b - sd) / a, (-b + sd) / a)
    };

    if d_dot_s.abs() <= 1e-12 {
        if h0 < h_lo || h0 > h_hi {
            return None;
        }
    } else {
        let t0 = (h_lo - h0) / d_dot_s;
        let t1 = (h_hi - h0) / d_dot_s;
        w_lo = w_lo.max(t0.min(t1));
        w_hi = w_hi.min(t0.max(t1));
    }

    (w_hi > w_lo + 1e-6).then_some((w_lo, w_hi))
}

/// Exact ray-cutter cutting interval `[w_lo, w_hi]` along axis-aligned ray `P(w) = p_base + w * d`
/// for an arbitrarily oriented cutter pose `(tip, spindle)` and effective outer radius `r_lim`.
#[inline]
#[allow(clippy::too_many_arguments)]
fn eval_oriented_pose_cut_interval(
    v0: DVec3,
    d_axis: DVec3,
    spindle: DVec3,
    a: f64,
    b: f64,
    c: f64,
    h0: f64,
    d_dot_s: f64,
    prof: &RadialCutterProfile,
    r_lim: f64,
) -> Option<(f64, f64)> {
    let r_eff = r_lim.min(prof.cutting_radius_mm);
    if r_eff <= 1e-6 {
        return None;
    }
    let h_max = prof.horizontal_cut_max_height_mm();
    let (w_lo, w_hi) = solve_oriented_cylinder_interval(a, b, c, h0, d_dot_s, r_eff, 0.0, h_max)?;

    match prof.envelope().shape() {
        chip_tools::CutterEnvelopeShape::Flat { .. } => Some((w_lo, w_hi)),
        chip_tools::CutterEnvelopeShape::Ball { .. } => {
            let r_ball = prof.cutting_radius_mm;
            let mut out_lo = f64::INFINITY;
            let mut out_hi = f64::NEG_INFINITY;

            // Lower hemisphere: sphere centered at `tip + r_ball * spindle`, radius `r_eff`
            let q = v0 - spindle * r_ball;
            let bs = q.dot(d_axis);
            let cs = q.length_squared() - r_eff * r_eff;
            let disc = bs * bs - cs;
            if disc >= 0.0 {
                let sd = disc.sqrt();
                let mut s_lo = -bs - sd;
                let mut s_hi = -bs + sd;
                if d_dot_s.abs() > 1e-12 {
                    let t0 = -h0 / d_dot_s;
                    let t1 = (h_max - h0) / d_dot_s;
                    s_lo = s_lo.max(t0.min(t1));
                    s_hi = s_hi.min(t0.max(t1));
                } else if h0 < 0.0 || h0 > h_max {
                    s_hi = s_lo - 1.0;
                }
                if s_hi > s_lo + 1e-6 {
                    out_lo = out_lo.min(s_lo);
                    out_hi = out_hi.max(s_hi);
                }
            }
            // Upper cylindrical flutes `[r_ball, h_max]`
            if let Some((u_lo, u_hi)) =
                solve_oriented_cylinder_interval(a, b, c, h0, d_dot_s, r_eff, r_ball, h_max)
            {
                out_lo = out_lo.min(u_lo);
                out_hi = out_hi.max(u_hi);
            }
            (out_hi > out_lo + 1e-6).then_some((out_lo, out_hi))
        }
        _ => {
            let dr_skin = (prof.cutting_radius_mm - r_eff).max(0.0);
            let eval_inside = |w: f64| -> f64 {
                let rho = (a * w * w + 2.0 * b * w + c).max(0.0).sqrt() + dr_skin;
                let h = h0 + w * d_dot_s;
                let Some((dz_lo, dz_hi)) = prof.cutting_z_interval_at_radius(rho) else {
                    return -1e30;
                };
                (h - dz_lo).min(dz_hi - h)
            };

            let g_lo = eval_inside(w_lo);
            let g_hi = eval_inside(w_hi);
            if g_lo >= 0.0 && g_hi >= 0.0 {
                return Some((w_lo, w_hi));
            }
            if g_lo >= 0.0 {
                let mut lo = w_lo;
                let mut hi = w_hi;
                for _ in 0..11 {
                    let mid = 0.5 * (lo + hi);
                    if eval_inside(mid) >= 0.0 {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                return (lo > w_lo + 1e-6).then_some((w_lo, lo));
            }
            if g_hi >= 0.0 {
                let mut lo = w_lo;
                let mut hi = w_hi;
                for _ in 0..11 {
                    let mid = 0.5 * (lo + hi);
                    if eval_inside(mid) >= 0.0 {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                return (w_hi > hi + 1e-6).then_some((hi, w_hi));
            }

            // Both endpoints outside: probe interior for concave peak
            let w_min_rho = if a > 1e-12 {
                (-b / a).clamp(w_lo, w_hi)
            } else {
                0.5 * (w_lo + w_hi)
            };
            let probes = [
                w_min_rho,
                w_lo + 0.25 * (w_hi - w_lo),
                0.5 * (w_lo + w_hi),
                w_lo + 0.75 * (w_hi - w_lo),
            ];
            let mut best_w = w_min_rho;
            let mut best_g = f64::NEG_INFINITY;
            for wp in probes {
                let gv = eval_inside(wp);
                if gv > best_g {
                    best_g = gv;
                    best_w = wp;
                }
            }
            if best_g <= 0.0 {
                return None;
            }
            let mut lo_a = w_lo;
            let mut hi_a = best_w;
            for _ in 0..11 {
                let mid = 0.5 * (lo_a + hi_a);
                if eval_inside(mid) >= 0.0 {
                    hi_a = mid;
                } else {
                    lo_a = mid;
                }
            }
            let mut lo_b = best_w;
            let mut hi_b = w_hi;
            for _ in 0..11 {
                let mid = 0.5 * (lo_b + hi_b);
                if eval_inside(mid) >= 0.0 {
                    lo_b = mid;
                } else {
                    hi_b = mid;
                }
            }
            (lo_b > hi_a + 1e-6).then_some((hi_a, lo_b))
        }
    }
}

/// Evaluates an arbitrary [`OrientedPose3D`] against a single ray on plane `axis` at `(pu, pv)`.
#[allow(clippy::too_many_arguments)]
fn apply_oriented_pose_to_ray(
    axis: Axis3,
    pu: f64,
    pv: f64,
    pose: OrientedPose3D,
    prof: &RadialCutterProfile,
    is_rapid: bool,
    tol: StockVerifyTolerances,
    op_idx: u32,
    verify: bool,
    check_holder: bool,
    target_spans: Option<&[DexelSpan]>,
    scratch: &mut Vec<DexelSpan>,
) -> (bool, SpanRuleViolations) {
    let mut rv = SpanRuleViolations::default();
    if scratch.is_empty() {
        return (false, rv);
    }
    let p_base = axis.from_uvw(pu, pv, 0.0);
    let d_axis = axis.unit_vec();
    let spindle = pose.spindle;
    let v0 = p_base - pose.tip;
    let h0 = v0.dot(spindle);
    let d_dot_s = d_axis.dot(spindle);
    let v0_perp = v0 - spindle * h0;
    let d_perp = d_axis - spindle * d_dot_s;
    let a = d_perp.length_squared().max(0.0);
    let b = v0_perp.dot(d_perp);
    let c = v0_perp.length_squared().max(0.0);

    let r_cut = prof.cutting_radius_mm;
    let shank_skin = (prof.shank_radius_mm - tol.radial_wall_skin_mm).max(0.0);
    let neck_skin = (prof.neck_radius_mm - tol.radial_wall_skin_mm).max(0.0);
    let r_inner_skin = (r_cut - tol.radial_wall_skin_mm).max(0.0);

    if verify {
        let stickout = prof.stickout_length_mm.max(prof.flute_length_mm);
        if check_holder
            && let Some((hw_lo, hw_hi)) = solve_oriented_cylinder_interval(
                a,
                b,
                c,
                h0,
                d_dot_s,
                prof.holder_radius_mm,
                stickout,
                stickout + 40.0,
            )
        {
            let pen = query_overlap_length(scratch, hw_lo, hw_hi);
            if pen > 0.05 {
                rv.record_holder(pen, pose.tip_wcs);
            }
        }
        if !is_rapid {
            let reach = prof.effective_reach_mm();
            if stickout > reach + 1e-4
                && let Some((fw_lo, fw_hi)) = solve_oriented_cylinder_interval(
                    a, b, c, h0, d_dot_s, shank_skin, reach, stickout,
                )
            {
                let rub = query_overlap_length(scratch, fw_lo, fw_hi);
                if rub > 0.15 {
                    rv.record_flute(rub, pose.tip_wcs);
                }
            }
            if prof.has_relieved_neck
                && prof.neck_length_mm > prof.flute_length_mm + 0.05
                && let Some((nw_lo, nw_hi)) = solve_oriented_cylinder_interval(
                    a,
                    b,
                    c,
                    h0,
                    d_dot_s,
                    neck_skin,
                    prof.flute_length_mm + 0.05,
                    prof.neck_length_mm,
                )
            {
                let ov = query_overlap_length(scratch, nw_lo, nw_hi);
                if ov > 0.10 {
                    rv.record_neck(ov, pose.tip_wcs);
                }
            }
        }
    }

    let mut ray_mut = false;
    if let Some((w_lo, w_hi)) =
        eval_oriented_pose_cut_interval(v0, d_axis, spindle, a, b, c, h0, d_dot_s, prof, r_cut)
    {
        if verify && is_rapid {
            let ov = query_overlap_length(scratch, w_lo, w_hi);
            if ov > tol.rapid_crash_tol_mm {
                rv.record_rapid(ov, pose.tip_wcs);
            }
        }
        if verify
            && let Some(t_spans) = target_spans
            && let Some((w_in_lo, w_in_hi)) = eval_oriented_pose_cut_interval(
                v0,
                d_axis,
                spindle,
                a,
                b,
                c,
                h0,
                d_dot_s,
                prof,
                r_inner_skin,
            )
            && let Some((pen, _)) = query_target_gouge_depth(
                t_spans,
                w_in_lo,
                w_in_hi,
                tol.gouge_tol_mm + tol.radial_wall_skin_mm,
            )
        {
            rv.record_gouge(pen, pose.tip_wcs);
        }
        if subtract_dexel_interval(scratch, w_lo, w_hi, op_idx) > 0.0 {
            ray_mut = true;
        }
    }

    (ray_mut, rv)
}

#[inline]
fn cardinal_axial_chord(axis: Axis3, sign: f64, chord: LinearChord3D) -> LinearChord3D {
    let u_ax = axis.u_axis();
    let v_ax = axis.v_axis();
    LinearChord3D {
        p0: DVec3::new(
            u_ax.get(chord.p0),
            v_ax.get(chord.p0),
            sign * axis.get(chord.p0),
        ),
        p1: DVec3::new(
            u_ax.get(chord.p1),
            v_ax.get(chord.p1),
            sign * axis.get(chord.p1),
        ),
    }
}

#[inline]
fn cardinal_horizontal_chord(
    axis: Axis3,
    spindle: CardinalSpindle,
    chord: LinearChord3D,
) -> (LinearChord3D, bool) {
    let s_ax = spindle.axis();
    let s_sign = spindle.sign();
    let u_ax = axis.u_axis();
    let v_ax = axis.v_axis();
    let spindle_in_u = u_ax == s_ax;
    let other_ax = if spindle_in_u { v_ax } else { u_ax };
    let mapped = LinearChord3D {
        p0: DVec3::new(
            axis.get(chord.p0),
            other_ax.get(chord.p0),
            s_sign * s_ax.get(chord.p0),
        ),
        p1: DVec3::new(
            axis.get(chord.p1),
            other_ax.get(chord.p1),
            s_sign * s_ax.get(chord.p1),
        ),
    };
    (mapped, spindle_in_u)
}

#[inline]
fn map_cz_interval_to_w(cz_lo: f64, cz_hi: f64, sign: f64) -> (f64, f64) {
    if sign >= 0.0 {
        (cz_lo, cz_hi)
    } else {
        (-cz_hi, -cz_lo)
    }
}

pub(crate) fn motion_to_linear_chords(motion: SweepMotion, out: &mut Vec<LinearChord3D>) {
    out.clear();
    match motion {
        SweepMotion::Linear { p0, p1, .. } => {
            out.push(LinearChord3D { p0, p1 });
        }
        SweepMotion::PlanarArcXY {
            start,
            end,
            radius_mm,
            signed_sweep_rad,
            ..
        } => {
            let r = radius_mm.max(1e-6);
            let max_sag = 0.005;
            let cos_half = (1.0 - (max_sag / r).min(0.5)).clamp(-1.0, 1.0);
            let max_dtheta = (2.0 * cos_half.acos()).clamp(0.02, 0.05);
            let steps = ((signed_sweep_rad.abs() / max_dtheta).ceil() as usize).clamp(4, 2_048);
            let mut prev = start;
            for k in 1..=steps {
                let u = k as f64 / steps as f64;
                let next = if k == steps {
                    end
                } else {
                    motion.sample_point(u)
                };
                out.push(LinearChord3D { p0: prev, p1: next });
                prev = next;
            }
        }
    }
}

impl TriDexelStock {
    /// Sweeps a batch of [`SweepChord`] primitives across all 3 orthogonal dexel planes,
    /// supporting both 3-axis and 4th-axis rotary motions (`self.rotary`) and optionally
    /// evaluating Rules 1–4 against `target` (`verify = true`).
    pub fn sweep_chords(
        &mut self,
        chords: &[SweepChord],
        tools: &[SweepToolSpec],
        target: Option<&TriDexelStock>,
        verify: bool,
    ) -> BatchSweepOutput {
        let mut out = BatchSweepOutput::default();
        let mut arc_buf: Vec<LinearChord3D> = Vec::with_capacity(8);
        let mut prim_buf: Vec<SweptPrimitive> = Vec::with_capacity(16);
        let min_pitch = self.pitch_mm.x.min(self.pitch_mm.y).min(self.pitch_mm.z);
        let rotary = self.rotary;
        let mut prev_oriented_end: Option<(DVec3, f64, u32, usize, bool)> = None;

        for (raw_idx, ch) in chords.iter().enumerate() {
            let Some(tool) = tools.get(ch.profile_idx) else {
                continue;
            };
            let start_pt = ch.motion.sample_point(0.0);
            let start_rot = ch.motion.sample_rotary_deg(0.0);
            let skip_start = prev_oriented_end.is_some_and(|(pp, pr, pop, pprof, prap)| {
                pop == ch.op_idx
                    && pprof == ch.profile_idx
                    && prap == ch.is_rapid
                    && pp.distance_squared(start_pt) <= 1e-10
                    && (pr - start_rot).abs() <= 1e-6
            });
            prim_buf.clear();
            decompose_chord_for_tridexel(
                ch,
                raw_idx as u32,
                rotary,
                min_pitch,
                &tool.profile,
                verify,
                skip_start,
                &mut arc_buf,
                &mut prim_buf,
                &mut out.violations,
            );
            prev_oriented_end = if prim_buf
                .last()
                .is_some_and(|p| matches!(p, SweptPrimitive::OrientedPose(_)))
            {
                Some((
                    ch.motion.sample_point(1.0),
                    ch.motion.sample_rotary_deg(1.0),
                    ch.op_idx,
                    ch.profile_idx,
                    ch.is_rapid,
                ))
            } else {
                None
            };
            for &prim in &prim_buf {
                match prim {
                    SweptPrimitive::CardinalLinear {
                        chord,
                        spindle,
                        rot_deg,
                    } => {
                        let ctx = SweepContext {
                            prof: &tool.profile,
                            is_flat_endmill: tool.is_flat_endmill,
                            is_rapid: ch.is_rapid,
                            tol: ch.verify_tol,
                            op_idx: ch.op_idx,
                            target,
                            rotary,
                            rot_deg,
                        };
                        let (mutated, viol) =
                            self.apply_cardinal_linear_chord(chord, spindle, &ctx, verify);
                        out.mutated |= mutated;
                        out.violations.merge(viol);
                    }
                    SweptPrimitive::OrientedPose(pose) => {
                        let ctx = SweepContext {
                            prof: &tool.profile,
                            is_flat_endmill: tool.is_flat_endmill,
                            is_rapid: ch.is_rapid,
                            tol: ch.verify_tol,
                            op_idx: ch.op_idx,
                            target,
                            rotary,
                            rot_deg: 0.0,
                        };
                        let (mutated, viol) = self.apply_oriented_pose(pose, &ctx, verify);
                        out.mutated |= mutated;
                        out.violations.merge(viol);
                    }
                }
            }
        }

        out
    }

    fn apply_cardinal_linear_chord(
        &mut self,
        chord: LinearChord3D,
        spindle: CardinalSpindle,
        ctx: &SweepContext<'_>,
        verify: bool,
    ) -> (bool, SpanRuleViolations) {
        let mut violations = SpanRuleViolations::default();
        let mut mutated = false;
        let s_ax = spindle.axis();

        let (a_mut, a_viol) = self.sweep_axial_plane(s_ax, spindle.sign(), chord, ctx, verify);
        mutated |= a_mut;
        violations.merge(a_viol);

        for axis in Axis3::ALL {
            if axis == s_ax {
                continue;
            }
            let (h_mut, h_viol) = self.sweep_horizontal_plane(axis, spindle, chord, ctx, verify);
            mutated |= h_mut;
            violations.merge(h_viol);
        }

        (mutated, violations)
    }

    fn apply_oriented_pose(
        &mut self,
        pose: OrientedPose3D,
        ctx: &SweepContext<'_>,
        verify: bool,
    ) -> (bool, SpanRuleViolations) {
        let Some((aabb_min, aabb_max, check_holder)) =
            oriented_pose_stock_aabb(pose, ctx.prof, self.aabb_wcs(), verify)
        else {
            return (false, SpanRuleViolations::default());
        };

        let mut violations = SpanRuleViolations::default();
        let mut mutated = false;
        for axis in Axis3::ALL {
            let (p_mut, p_viol) = self.sweep_oriented_pose_plane(
                axis,
                pose,
                aabb_min,
                aabb_max,
                ctx,
                verify,
                check_holder,
            );
            mutated |= p_mut;
            violations.merge(p_viol);
        }

        (mutated, violations)
    }

    #[allow(clippy::too_many_arguments)]
    fn sweep_oriented_pose_plane(
        &mut self,
        axis: Axis3,
        pose: OrientedPose3D,
        aabb_min: DVec3,
        aabb_max: DVec3,
        ctx: &SweepContext<'_>,
        verify: bool,
        check_holder: bool,
    ) -> (bool, SpanRuleViolations) {
        let u_ax = axis.u_axis();
        let v_ax = axis.v_axis();
        let plane = self.plane_mut(axis);
        let (Some((u0, u1)), Some((v0, v1))) = (
            plane.index_span_u(u_ax.get(aabb_min), u_ax.get(aabb_max)),
            plane.index_span_v(v_ax.get(aabb_min), v_ax.get(aabb_max)),
        ) else {
            return (false, SpanRuleViolations::default());
        };

        let nu = plane.nu;
        let ou = plane.origin_u_mm;
        let ov = plane.origin_v_mm;
        let ow = plane.origin_w_mm;
        let du = plane.du_mm;
        let dv = plane.dv_mm;
        let row_slot_stride = nu * INLINE_DEXEL_SLOTS;
        let target_plane = ctx.target.map(|t| t.plane(axis));
        let span_rays = (u1 - u0 + 1) * (v1 - v0 + 1);

        let slots_slice = &mut plane.slots[v0 * row_slot_stride..(v1 + 1) * row_slot_stride];
        let counts_slice = &mut plane.counts[v0 * nu..(v1 + 1) * nu];
        let overflow_ref = &plane.overflow;

        let (p_mut, p_viol, overflow_deltas) = if span_rays < 2048 {
            let mut p_mut = false;
            let mut p_viol = SpanRuleViolations::default();
            let mut ov_deltas: Vec<(u32, Option<Vec<DexelSpan>>)> = Vec::new();
            let mut scratch: Vec<DexelSpan> = Vec::with_capacity(8);

            for (v_off, (row_slots, row_counts)) in slots_slice
                .chunks_mut(row_slot_stride)
                .zip(counts_slice.chunks_mut(nu))
                .enumerate()
            {
                let iv = v0 + v_off;
                let pv = ov + (iv as f64 + 0.5) * dv;
                let row_base = iv * nu;

                for iu in u0..=u1 {
                    if row_counts[iu] == 0 {
                        continue;
                    }
                    let ray_idx = (row_base + iu) as u32;
                    let was_ov = load_row_ray_into(
                        iu,
                        ray_idx,
                        row_slots,
                        row_counts,
                        overflow_ref,
                        ow,
                        &mut scratch,
                    );
                    if scratch.is_empty() {
                        continue;
                    }
                    let pu = ou + (iu as f64 + 0.5) * du;
                    let target_spans_owned =
                        target_plane.map(|tp| tp.ray_spans_by_index(row_base + iu));
                    let (ray_mut, ray_viol) = apply_oriented_pose_to_ray(
                        axis,
                        pu,
                        pv,
                        pose,
                        ctx.prof,
                        ctx.is_rapid,
                        ctx.tol,
                        ctx.op_idx,
                        verify,
                        check_holder,
                        target_spans_owned.as_deref(),
                        &mut scratch,
                    );
                    p_viol.merge(ray_viol);
                    if ray_mut {
                        p_mut = true;
                        write_row_ray_from(
                            iu,
                            ray_idx,
                            row_slots,
                            row_counts,
                            ow,
                            &scratch,
                            was_ov,
                            &mut ov_deltas,
                        );
                    }
                }
            }
            (p_mut, p_viol, ov_deltas)
        } else {
            slots_slice
                .par_chunks_mut(row_slot_stride)
                .zip(counts_slice.par_chunks_mut(nu))
                .enumerate()
                .map(|(v_off, (row_slots, row_counts))| {
                    let iv = v0 + v_off;
                    let pv = ov + (iv as f64 + 0.5) * dv;
                    let row_base = iv * nu;
                    let mut row_mut = false;
                    let mut rv = SpanRuleViolations::default();
                    let mut row_ov_deltas: Vec<(u32, Option<Vec<DexelSpan>>)> = Vec::new();
                    let mut scratch: Vec<DexelSpan> = Vec::with_capacity(8);

                    for iu in u0..=u1 {
                        if row_counts[iu] == 0 {
                            continue;
                        }
                        let ray_idx = (row_base + iu) as u32;
                        let was_ov = load_row_ray_into(
                            iu,
                            ray_idx,
                            row_slots,
                            row_counts,
                            overflow_ref,
                            ow,
                            &mut scratch,
                        );
                        if scratch.is_empty() {
                            continue;
                        }
                        let pu = ou + (iu as f64 + 0.5) * du;
                        let target_spans_owned =
                            target_plane.map(|tp| tp.ray_spans_by_index(row_base + iu));
                        let (ray_mut, ray_viol) = apply_oriented_pose_to_ray(
                            axis,
                            pu,
                            pv,
                            pose,
                            ctx.prof,
                            ctx.is_rapid,
                            ctx.tol,
                            ctx.op_idx,
                            verify,
                            check_holder,
                            target_spans_owned.as_deref(),
                            &mut scratch,
                        );
                        rv.merge(ray_viol);
                        if ray_mut {
                            row_mut = true;
                            write_row_ray_from(
                                iu,
                                ray_idx,
                                row_slots,
                                row_counts,
                                ow,
                                &scratch,
                                was_ov,
                                &mut row_ov_deltas,
                            );
                        }
                    }
                    (row_mut, rv, row_ov_deltas)
                })
                .reduce(
                    || (false, SpanRuleViolations::default(), Vec::new()),
                    |(m0, mut v0, mut d0), (m1, v1, mut d1)| {
                        v0.merge(v1);
                        d0.append(&mut d1);
                        (m0 || m1, v0, d0)
                    },
                )
        };

        for (ray_idx, entry) in overflow_deltas {
            match entry {
                Some(vec) => {
                    plane.overflow.insert(ray_idx, vec);
                }
                None => {
                    plane.overflow.remove(&ray_idx);
                }
            }
        }

        (p_mut, p_viol)
    }

    fn sweep_axial_plane(
        &mut self,
        axis: Axis3,
        spindle_sign: f64,
        chord: LinearChord3D,
        ctx: &SweepContext<'_>,
        verify: bool,
    ) -> (bool, SpanRuleViolations) {
        let mapped = cardinal_axial_chord(axis, spindle_sign, chord);
        let r_cut = ctx.prof.cutting_radius_mm;
        let r_check = if verify {
            r_cut.max(ctx.prof.holder_radius_mm)
        } else {
            r_cut
        };

        let min_u = mapped.p0.x.min(mapped.p1.x) - r_check;
        let max_u = mapped.p0.x.max(mapped.p1.x) + r_check;
        let min_v = mapped.p0.y.min(mapped.p1.y) - r_check;
        let max_v = mapped.p0.y.max(mapped.p1.y) + r_check;
        let neck_skin = (ctx.prof.neck_radius_mm - ctx.tol.radial_wall_skin_mm).max(0.0);
        let shank_skin = (ctx.prof.shank_radius_mm - ctx.tol.radial_wall_skin_mm).max(0.0);
        let r_inner_skin = (r_cut - ctx.tol.radial_wall_skin_mm).max(0.0);

        let plane = self.plane_mut(axis);
        let (Some((i0, i1)), Some((j0, j1))) = (
            plane.index_span_u(min_u, max_u),
            plane.index_span_v(min_v, max_v),
        ) else {
            return (false, SpanRuleViolations::default());
        };

        let nu = plane.nu;
        let ou = plane.origin_u_mm;
        let ov = plane.origin_v_mm;
        let ow = plane.origin_w_mm;
        let du = plane.du_mm;
        let dv = plane.dv_mm;
        let row_slot_stride = nu * INLINE_DEXEL_SLOTS;
        let target_plane = ctx.target.map(|t| t.plane(axis));

        let slots_slice = &mut plane.slots[j0 * row_slot_stride..(j1 + 1) * row_slot_stride];
        let counts_slice = &mut plane.counts[j0 * nu..(j1 + 1) * nu];
        let overflow_ref = &plane.overflow;

        let (z_mut, z_viol, overflow_deltas) = slots_slice
            .par_chunks_mut(row_slot_stride)
            .zip(counts_slice.par_chunks_mut(nu))
            .enumerate()
            .map(|(j_off, (row_slots, row_counts))| {
                let j = j0 + j_off;
                let py = ov + (j as f64 + 0.5) * dv;
                let row_base = j * nu;
                let mut row_mut = false;
                let mut rv = SpanRuleViolations::default();
                let mut row_ov_deltas: Vec<(u32, Option<Vec<DexelSpan>>)> = Vec::new();
                let mut scratch: Vec<DexelSpan> = Vec::with_capacity(8);
                let mut tm_intervals: Vec<(f64, f64)> = Vec::new();

                for i in i0..=i1 {
                    if row_counts[i] == 0 {
                        continue;
                    }
                    let ray_idx = (row_base + i) as u32;
                    let was_ov = load_row_ray_into(
                        i,
                        ray_idx,
                        row_slots,
                        row_counts,
                        overflow_ref,
                        ow,
                        &mut scratch,
                    );
                    if scratch.is_empty() {
                        continue;
                    }
                    let px = ou + (i as f64 + 0.5) * du;

                    if verify {
                        let p0_xy = DVec2::new(mapped.p0.x, mapped.p0.y);
                        let p1_xy = DVec2::new(mapped.p1.x, mapped.p1.y);
                        let d_xy = p1_xy - p0_xy;
                        let l2 = d_xy.length_squared();
                        let t_c = if l2 > 1e-12 {
                            ((DVec2::new(px, py) - p0_xy).dot(d_xy) / l2).clamp(0.0, 1.0)
                        } else {
                            0.0
                        };
                        let c_xy = p0_xy.lerp(p1_xy, t_c);
                        let dist_xy = (DVec2::new(px, py) - c_xy).length();
                        let z_tip_here = if l2 > 1e-12 {
                            mapped.p0.z + t_c * (mapped.p1.z - mapped.p0.z)
                        } else {
                            mapped.p0.z.min(mapped.p1.z)
                        };
                        let top_cz = if spindle_sign >= 0.0 {
                            scratch.last().map(|s| s.max_mm)
                        } else {
                            scratch.first().map(|s| -s.min_mm)
                        };
                        if let Some(top_z) = top_cz {
                            if dist_xy <= ctx.prof.holder_radius_mm {
                                let holder_z = z_tip_here + ctx.prof.stickout_length_mm;
                                let pen = top_z - holder_z;
                                if pen > 0.05 {
                                    let loc = ctx.to_diag_wcs(axis.from_uvw(
                                        px,
                                        py,
                                        spindle_sign * holder_z,
                                    ));
                                    rv.record_holder(pen, loc);
                                }
                            }
                            if !ctx.is_rapid && dist_xy <= shank_skin {
                                let reach_top = z_tip_here + ctx.prof.effective_reach_mm();
                                let rub = top_z - reach_top;
                                if rub > 0.15 {
                                    let loc = ctx.to_diag_wcs(axis.from_uvw(
                                        px,
                                        py,
                                        spindle_sign * reach_top,
                                    ));
                                    rv.record_flute(rub, loc);
                                }
                            }
                        }
                        if ctx.prof.has_relieved_neck && !ctx.is_rapid && dist_xy <= neck_skin {
                            let neck_lo = z_tip_here + ctx.prof.flute_length_mm + 0.05;
                            let neck_hi = z_tip_here + ctx.prof.neck_length_mm;
                            if neck_hi > neck_lo {
                                let (w_lo, w_hi) =
                                    map_cz_interval_to_w(neck_lo, neck_hi, spindle_sign);
                                let ov = query_overlap_length(&scratch, w_lo, w_hi);
                                if ov > 0.10 {
                                    let loc = ctx.to_diag_wcs(axis.from_uvw(
                                        px,
                                        py,
                                        spindle_sign * neck_lo,
                                    ));
                                    rv.record_neck(ov, loc);
                                }
                            }
                        }
                    }

                    let mut ray_mut = false;
                    if ctx.prof.is_thread_mill {
                        eval_z_ray_thread_mill_intervals(
                            mapped,
                            ctx.prof,
                            px,
                            py,
                            r_cut,
                            &mut tm_intervals,
                        );
                        if !tm_intervals.is_empty() {
                            if verify && ctx.is_rapid {
                                for &(cz_lo, cz_hi) in &tm_intervals {
                                    let (w_lo, w_hi) =
                                        map_cz_interval_to_w(cz_lo, cz_hi, spindle_sign);
                                    let ov = query_overlap_length(&scratch, w_lo, w_hi);
                                    if ov > ctx.tol.rapid_crash_tol_mm {
                                        rv.record_rapid(
                                            ov,
                                            ctx.to_diag_wcs(axis.from_uvw(px, py, w_lo)),
                                        );
                                    }
                                }
                            }
                            if verify
                                && let Some(tz_plane) = target_plane
                                && let Some((cz_lo_in, cz_hi_in)) = eval_z_ray_cylinder_interval(
                                    mapped,
                                    px,
                                    py,
                                    neck_skin,
                                    ctx.prof.neck_length_mm,
                                )
                            {
                                let (w_lo_in, w_hi_in) =
                                    map_cz_interval_to_w(cz_lo_in, cz_hi_in, spindle_sign);
                                if let Some((pen, w_at)) = query_target_gouge_depth(
                                    &tz_plane.ray_spans_by_index(row_base + i),
                                    w_lo_in,
                                    w_hi_in,
                                    ctx.tol.gouge_tol_mm,
                                ) {
                                    rv.record_gouge(
                                        pen,
                                        ctx.to_diag_wcs(axis.from_uvw(px, py, w_at)),
                                    );
                                }
                            }
                            for &(cz_lo, cz_hi) in &tm_intervals {
                                let (w_lo, w_hi) = map_cz_interval_to_w(cz_lo, cz_hi, spindle_sign);
                                if subtract_dexel_interval(&mut scratch, w_lo, w_hi, ctx.op_idx)
                                    > 0.0
                                {
                                    ray_mut = true;
                                }
                            }
                        }
                    } else if let Some((cz_lo, cz_hi, _)) = eval_z_ray_cut_interval(
                        mapped,
                        ctx.prof,
                        ctx.is_flat_endmill,
                        px,
                        py,
                        r_cut,
                    ) {
                        let (w_lo, w_hi) = map_cz_interval_to_w(cz_lo, cz_hi, spindle_sign);
                        if verify && ctx.is_rapid {
                            let ov = query_overlap_length(&scratch, w_lo, w_hi);
                            if ov > ctx.tol.rapid_crash_tol_mm {
                                rv.record_rapid(ov, ctx.to_diag_wcs(axis.from_uvw(px, py, w_lo)));
                            }
                        }
                        if verify
                            && let Some(tz_plane) = target_plane
                            && let Some((cz_lo_in, cz_hi_in, _)) = eval_z_ray_cut_interval(
                                mapped,
                                ctx.prof,
                                ctx.is_flat_endmill,
                                px,
                                py,
                                r_inner_skin,
                            )
                        {
                            let (w_lo_in, w_hi_in) =
                                map_cz_interval_to_w(cz_lo_in, cz_hi_in, spindle_sign);
                            if let Some((pen, w_at)) = query_target_gouge_depth(
                                &tz_plane.ray_spans_by_index(row_base + i),
                                w_lo_in,
                                w_hi_in,
                                ctx.tol.gouge_tol_mm,
                            ) {
                                rv.record_gouge(pen, ctx.to_diag_wcs(axis.from_uvw(px, py, w_at)));
                            }
                        }
                        if subtract_dexel_interval(&mut scratch, w_lo, w_hi, ctx.op_idx) > 0.0 {
                            ray_mut = true;
                        }
                    }

                    if ray_mut {
                        row_mut = true;
                        write_row_ray_from(
                            i,
                            ray_idx,
                            row_slots,
                            row_counts,
                            ow,
                            &scratch,
                            was_ov,
                            &mut row_ov_deltas,
                        );
                    }
                }

                (row_mut, rv, row_ov_deltas)
            })
            .reduce(
                || (false, SpanRuleViolations::default(), Vec::new()),
                |(m0, mut v0, mut d0), (m1, v1, mut d1)| {
                    v0.merge(v1);
                    d0.append(&mut d1);
                    (m0 || m1, v0, d0)
                },
            );

        for (ray_idx, entry) in overflow_deltas {
            match entry {
                Some(vec) => {
                    plane.overflow.insert(ray_idx, vec);
                }
                None => {
                    plane.overflow.remove(&ray_idx);
                }
            }
        }

        (z_mut, z_viol)
    }

    /// Axis-generic horizontal dexel sweep orthogonal to any [`CardinalSpindle`] axis.
    fn sweep_horizontal_plane(
        &mut self,
        axis: Axis3,
        spindle: CardinalSpindle,
        chord: LinearChord3D,
        ctx: &SweepContext<'_>,
        verify: bool,
    ) -> (bool, SpanRuleViolations) {
        let (mapped, spindle_in_u) = cardinal_horizontal_chord(axis, spindle, chord);
        let s_ax = spindle.axis();
        let s_sign = spindle.sign();
        let r_cut = ctx.prof.cutting_radius_mm;
        let max_h = ctx.prof.horizontal_cut_max_height_mm();
        let min_lat = mapped.p0.y.min(mapped.p1.y) - r_cut;
        let max_lat = mapped.p0.y.max(mapped.p1.y) + r_cut;
        let (min_s, max_s) = if s_sign >= 0.0 {
            (
                s_ax.get(chord.p0).min(s_ax.get(chord.p1)),
                s_ax.get(chord.p0).max(s_ax.get(chord.p1)) + max_h,
            )
        } else {
            (
                s_ax.get(chord.p0).min(s_ax.get(chord.p1)) - max_h,
                s_ax.get(chord.p0).max(s_ax.get(chord.p1)),
            )
        };
        let (min_u_cut, max_u_cut, min_v_cut, max_v_cut) = if spindle_in_u {
            (min_s, max_s, min_lat, max_lat)
        } else {
            (min_lat, max_lat, min_s, max_s)
        };

        let dz_step = 0.25 * self.pitch(s_ax);
        let neck_skin = (ctx.prof.neck_radius_mm - ctx.tol.radial_wall_skin_mm).max(0.0);
        let wall_skin = ctx.tol.gouge_tol_mm + ctx.tol.radial_wall_skin_mm;
        let flute_wall_skin = ctx.tol.radial_wall_skin_mm + (ctx.tol.gouge_tol_mm - 0.02).max(0.0);

        let plane = self.plane_mut(axis);
        let (Some((u0, u1)), Some((k0, k1))) = (
            plane.index_span_u(min_u_cut, max_u_cut),
            plane.index_span_v(min_v_cut, max_v_cut),
        ) else {
            return (false, SpanRuleViolations::default());
        };

        let nu = plane.nu;
        let ou = plane.origin_u_mm;
        let ov = plane.origin_v_mm;
        let ow = plane.origin_w_mm;
        let du = plane.du_mm;
        let dv = plane.dv_mm;
        let row_slot_stride = nu * INLINE_DEXEL_SLOTS;
        let target_plane = ctx.target.map(|t| t.plane(axis));

        let slots_slice = &mut plane.slots[k0 * row_slot_stride..(k1 + 1) * row_slot_stride];
        let counts_slice = &mut plane.counts[k0 * nu..(k1 + 1) * nu];
        let overflow_ref = &plane.overflow;

        let (h_mut, h_viol, overflow_deltas) = slots_slice
            .par_chunks_mut(row_slot_stride)
            .zip(counts_slice.par_chunks_mut(nu))
            .enumerate()
            .map(|(k_off, (row_slots, row_counts))| {
                let k = k0 + k_off;
                let pv = ov + (k as f64 + 0.5) * dv;
                let row_base = k * nu;
                let mut row_mut = false;
                let mut rv = SpanRuleViolations::default();
                let mut row_ov_deltas: Vec<(u32, Option<Vec<DexelSpan>>)> = Vec::new();
                let mut scratch: Vec<DexelSpan> = Vec::with_capacity(8);

                for iu in u0..=u1 {
                    if row_counts[iu] == 0 {
                        continue;
                    }
                    let ray_idx = (row_base + iu) as u32;
                    let was_ov = load_row_ray_into(
                        iu,
                        ray_idx,
                        row_slots,
                        row_counts,
                        overflow_ref,
                        ow,
                        &mut scratch,
                    );
                    if scratch.is_empty() {
                        continue;
                    }
                    let pu = ou + (iu as f64 + 0.5) * du;
                    let (pz_c, pu_c) = if spindle_in_u {
                        (s_sign * pu, pv)
                    } else {
                        (s_sign * pv, pu)
                    };
                    if let Some((w_lo, w_hi)) = eval_horizontal_ray_cut_interval(
                        mapped, ctx.prof, 0.0, pz_c, pu_c, false, dz_step,
                    ) {
                        if verify && ctx.is_rapid {
                            let ov = query_overlap_length(&scratch, w_lo, w_hi);
                            if ov > ctx.tol.rapid_crash_tol_mm {
                                rv.record_rapid(
                                    ov,
                                    ctx.to_diag_wcs(axis.from_uvw(pu, pv, 0.5 * (w_lo + w_hi))),
                                );
                            }
                        }
                        if verify
                            && pz_c >= mapped.min_z() + ctx.tol.gouge_tol_mm
                            && let Some(tp) = target_plane
                            && let Some((w_lo_in, w_hi_in)) = if ctx.prof.is_thread_mill {
                                eval_horizontal_ray_cylinder_interval(
                                    mapped,
                                    neck_skin,
                                    ctx.prof.neck_length_mm,
                                    pz_c,
                                    pu_c,
                                    false,
                                )
                            } else {
                                eval_horizontal_ray_cut_interval(
                                    mapped,
                                    ctx.prof,
                                    flute_wall_skin,
                                    pz_c,
                                    pu_c,
                                    false,
                                    dz_step,
                                )
                            }
                            && let Some((pen, w_at)) = query_target_gouge_depth(
                                &tp.ray_spans_by_index(row_base + iu),
                                w_lo_in,
                                w_hi_in,
                                wall_skin,
                            )
                        {
                            rv.record_gouge(pen, ctx.to_diag_wcs(axis.from_uvw(pu, pv, w_at)));
                        }
                        if subtract_dexel_interval(&mut scratch, w_lo, w_hi, ctx.op_idx) > 0.0 {
                            row_mut = true;
                            write_row_ray_from(
                                iu,
                                ray_idx,
                                row_slots,
                                row_counts,
                                ow,
                                &scratch,
                                was_ov,
                                &mut row_ov_deltas,
                            );
                        }
                    }
                }
                (row_mut, rv, row_ov_deltas)
            })
            .reduce(
                || (false, SpanRuleViolations::default(), Vec::new()),
                |(m0, mut v0, mut d0), (m1, v1, mut d1)| {
                    v0.merge(v1);
                    d0.append(&mut d1);
                    (m0 || m1, v0, d0)
                },
            );

        for (ray_idx, entry) in overflow_deltas {
            match entry {
                Some(vec) => {
                    plane.overflow.insert(ray_idx, vec);
                }
                None => {
                    plane.overflow.remove(&ray_idx);
                }
            }
        }

        (h_mut, h_viol)
    }
}

/// Exact `f64` single-ray replay of one [`SweptPrimitive`] on plane `axis` at ray center `(pu, pv)`.
/// Shared by `WgpuComputeBackend`'s lossless hybrid overflow safety valve so rare `> K = 4`-span
/// rays execute identical subtraction and Rules 1–4 verification to the CPU Tri-Dexel oracle across
/// both 3-axis and 4th-axis rotary sweeps.
#[cfg(feature = "gpu-compute")]
#[allow(clippy::too_many_arguments)]
pub(crate) fn replay_single_ray_primitive(
    axis: Axis3,
    pu: f64,
    pv: f64,
    pitch_mm: DVec3,
    prim: SweptPrimitive,
    rotary: Option<RotaryStockSpec>,
    tool: &SweepToolSpec,
    is_rapid: bool,
    tol: StockVerifyTolerances,
    op_idx: u32,
    verify: bool,
    target_spans: Option<&[DexelSpan]>,
    scratch: &mut Vec<DexelSpan>,
) -> (bool, SpanRuleViolations) {
    let mut rv = SpanRuleViolations::default();
    if scratch.is_empty() {
        return (false, rv);
    }
    let prof = &tool.profile;
    match prim {
        SweptPrimitive::OrientedPose(pose) => apply_oriented_pose_to_ray(
            axis,
            pu,
            pv,
            pose,
            prof,
            is_rapid,
            tol,
            op_idx,
            verify,
            verify,
            target_spans,
            scratch,
        ),
        SweptPrimitive::CardinalLinear {
            chord,
            spindle,
            rot_deg,
        } => {
            let to_diag_wcs = |pt_part: DVec3| -> DVec3 {
                if let Some(rot) = rotary
                    && rot_deg.abs() > 1e-6
                {
                    rot.rotate_point_wcs(pt_part, rot_deg)
                } else {
                    pt_part
                }
            };
            let r_cut = prof.cutting_radius_mm;
            let neck_skin = (prof.neck_radius_mm - tol.radial_wall_skin_mm).max(0.0);
            let shank_skin = (prof.shank_radius_mm - tol.radial_wall_skin_mm).max(0.0);
            let r_inner_skin = (r_cut - tol.radial_wall_skin_mm).max(0.0);
            let s_ax = spindle.axis();
            let s_sign = spindle.sign();

            if axis == s_ax {
                let mapped = cardinal_axial_chord(axis, s_sign, chord);
                let px = pu;
                let py = pv;
                let r_check = if verify {
                    r_cut.max(prof.holder_radius_mm)
                } else {
                    r_cut
                };
                if px < mapped.p0.x.min(mapped.p1.x) - r_check - 1e-6
                    || px > mapped.p0.x.max(mapped.p1.x) + r_check + 1e-6
                    || py < mapped.p0.y.min(mapped.p1.y) - r_check - 1e-6
                    || py > mapped.p0.y.max(mapped.p1.y) + r_check + 1e-6
                {
                    return (false, rv);
                }

                if verify {
                    let p0_xy = DVec2::new(mapped.p0.x, mapped.p0.y);
                    let p1_xy = DVec2::new(mapped.p1.x, mapped.p1.y);
                    let d_xy = p1_xy - p0_xy;
                    let l2 = d_xy.length_squared();
                    let t_c = if l2 > 1e-12 {
                        ((DVec2::new(px, py) - p0_xy).dot(d_xy) / l2).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };
                    let c_xy = p0_xy.lerp(p1_xy, t_c);
                    let dist_xy = (DVec2::new(px, py) - c_xy).length();
                    let z_tip_here = if l2 > 1e-12 {
                        mapped.p0.z + t_c * (mapped.p1.z - mapped.p0.z)
                    } else {
                        mapped.p0.z.min(mapped.p1.z)
                    };
                    let top_cz = if s_sign >= 0.0 {
                        scratch.last().map(|s| s.max_mm)
                    } else {
                        scratch.first().map(|s| -s.min_mm)
                    };
                    if let Some(top_z) = top_cz {
                        if dist_xy <= prof.holder_radius_mm {
                            let holder_z = z_tip_here + prof.stickout_length_mm;
                            let pen = top_z - holder_z;
                            if pen > 0.05 {
                                rv.record_holder(
                                    pen,
                                    to_diag_wcs(axis.from_uvw(px, py, s_sign * holder_z)),
                                );
                            }
                        }
                        if !is_rapid && dist_xy <= shank_skin {
                            let reach_top = z_tip_here + prof.effective_reach_mm();
                            let rub = top_z - reach_top;
                            if rub > 0.15 {
                                rv.record_flute(
                                    rub,
                                    to_diag_wcs(axis.from_uvw(px, py, s_sign * reach_top)),
                                );
                            }
                        }
                    }
                    if prof.has_relieved_neck && !is_rapid && dist_xy <= neck_skin {
                        let neck_lo = z_tip_here + prof.flute_length_mm + 0.05;
                        let neck_hi = z_tip_here + prof.neck_length_mm;
                        if neck_hi > neck_lo {
                            let (w_lo, w_hi) = map_cz_interval_to_w(neck_lo, neck_hi, s_sign);
                            let ov = query_overlap_length(scratch, w_lo, w_hi);
                            if ov > 0.10 {
                                rv.record_neck(
                                    ov,
                                    to_diag_wcs(axis.from_uvw(px, py, s_sign * neck_lo)),
                                );
                            }
                        }
                    }
                }

                let mut ray_mut = false;
                if prof.is_thread_mill {
                    let mut tm_intervals = Vec::new();
                    eval_z_ray_thread_mill_intervals(
                        mapped,
                        prof,
                        px,
                        py,
                        r_cut,
                        &mut tm_intervals,
                    );
                    if !tm_intervals.is_empty() {
                        if verify && is_rapid {
                            for &(cz_lo, cz_hi) in &tm_intervals {
                                let (w_lo, w_hi) = map_cz_interval_to_w(cz_lo, cz_hi, s_sign);
                                let ov = query_overlap_length(scratch, w_lo, w_hi);
                                if ov > tol.rapid_crash_tol_mm {
                                    rv.record_rapid(ov, to_diag_wcs(axis.from_uvw(px, py, w_lo)));
                                }
                            }
                        }
                        if verify
                            && let Some(t_spans) = target_spans
                            && let Some((cz_lo_in, cz_hi_in)) = eval_z_ray_cylinder_interval(
                                mapped,
                                px,
                                py,
                                neck_skin,
                                prof.neck_length_mm,
                            )
                        {
                            let (w_lo_in, w_hi_in) =
                                map_cz_interval_to_w(cz_lo_in, cz_hi_in, s_sign);
                            if let Some((pen, w_at)) = query_target_gouge_depth(
                                t_spans,
                                w_lo_in,
                                w_hi_in,
                                tol.gouge_tol_mm,
                            ) {
                                rv.record_gouge(pen, to_diag_wcs(axis.from_uvw(px, py, w_at)));
                            }
                        }
                        for &(cz_lo, cz_hi) in &tm_intervals {
                            let (w_lo, w_hi) = map_cz_interval_to_w(cz_lo, cz_hi, s_sign);
                            if subtract_dexel_interval(scratch, w_lo, w_hi, op_idx) > 0.0 {
                                ray_mut = true;
                            }
                        }
                    }
                } else if let Some((cz_lo, cz_hi, _)) =
                    eval_z_ray_cut_interval(mapped, prof, tool.is_flat_endmill, px, py, r_cut)
                {
                    let (w_lo, w_hi) = map_cz_interval_to_w(cz_lo, cz_hi, s_sign);
                    if verify && is_rapid {
                        let ov = query_overlap_length(scratch, w_lo, w_hi);
                        if ov > tol.rapid_crash_tol_mm {
                            rv.record_rapid(ov, to_diag_wcs(axis.from_uvw(px, py, w_lo)));
                        }
                    }
                    if verify
                        && let Some(t_spans) = target_spans
                        && let Some((cz_lo_in, cz_hi_in, _)) = eval_z_ray_cut_interval(
                            mapped,
                            prof,
                            tool.is_flat_endmill,
                            px,
                            py,
                            r_inner_skin,
                        )
                    {
                        let (w_lo_in, w_hi_in) = map_cz_interval_to_w(cz_lo_in, cz_hi_in, s_sign);
                        if let Some((pen, w_at)) =
                            query_target_gouge_depth(t_spans, w_lo_in, w_hi_in, tol.gouge_tol_mm)
                        {
                            rv.record_gouge(pen, to_diag_wcs(axis.from_uvw(px, py, w_at)));
                        }
                    }
                    if subtract_dexel_interval(scratch, w_lo, w_hi, op_idx) > 0.0 {
                        ray_mut = true;
                    }
                }
                (ray_mut, rv)
            } else {
                let (mapped, spindle_in_u) = cardinal_horizontal_chord(axis, spindle, chord);
                let (pz_c, pu_c) = if spindle_in_u {
                    (s_sign * pu, pv)
                } else {
                    (s_sign * pv, pu)
                };
                let lat0 = mapped.p0.y.min(mapped.p1.y) - r_cut;
                let lat1 = mapped.p0.y.max(mapped.p1.y) + r_cut;
                if pu_c < lat0 - 1e-6
                    || pu_c > lat1 + 1e-6
                    || pz_c < mapped.min_z() - 1e-6
                    || pz_c > mapped.max_z() + prof.horizontal_cut_max_height_mm() + 1e-6
                {
                    return (false, rv);
                }
                let dz_step_mm = 0.25 * s_ax.get(pitch_mm);
                let wall_skin = tol.gouge_tol_mm + tol.radial_wall_skin_mm;
                let flute_wall_skin = tol.radial_wall_skin_mm + (tol.gouge_tol_mm - 0.02).max(0.0);
                let mut ray_mut = false;
                if let Some((w_lo, w_hi)) = eval_horizontal_ray_cut_interval(
                    mapped, prof, 0.0, pz_c, pu_c, false, dz_step_mm,
                ) {
                    if verify && is_rapid {
                        let ov = query_overlap_length(scratch, w_lo, w_hi);
                        if ov > tol.rapid_crash_tol_mm {
                            rv.record_rapid(
                                ov,
                                to_diag_wcs(axis.from_uvw(pu, pv, 0.5 * (w_lo + w_hi))),
                            );
                        }
                    }
                    if verify
                        && pz_c >= mapped.min_z() + tol.gouge_tol_mm
                        && let Some(t_spans) = target_spans
                        && let Some((w_lo_in, w_hi_in)) = if prof.is_thread_mill {
                            eval_horizontal_ray_cylinder_interval(
                                mapped,
                                neck_skin,
                                prof.neck_length_mm,
                                pz_c,
                                pu_c,
                                false,
                            )
                        } else {
                            eval_horizontal_ray_cut_interval(
                                mapped,
                                prof,
                                flute_wall_skin,
                                pz_c,
                                pu_c,
                                false,
                                dz_step_mm,
                            )
                        }
                        && let Some((pen, w_at)) =
                            query_target_gouge_depth(t_spans, w_lo_in, w_hi_in, wall_skin)
                    {
                        rv.record_gouge(pen, to_diag_wcs(axis.from_uvw(pu, pv, w_at)));
                    }
                    if subtract_dexel_interval(scratch, w_lo, w_hi, op_idx) > 0.0 {
                        ray_mut = true;
                    }
                }
                (ray_mut, rv)
            }
        }
    }
}
