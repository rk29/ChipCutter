use chip_math::{DVec2, DVec3};
use rayon::prelude::*;

use crate::types::{
    Axis3, BatchSweepOutput, RadialCutterProfile, SpanRuleViolations, StockVerifyTolerances,
    SweepChord, SweepMotion, SweepToolSpec,
};

use super::grid::{
    DexelSpan, INLINE_DEXEL_SLOTS, TriDexelStock, load_row_ray_into, query_overlap_length,
    query_target_gouge_depth, subtract_dexel_interval, write_row_ray_from,
};

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
    let n_sub = ((sub_dz / dz_step_mm.max(0.05)).ceil() as usize).clamp(2, 24);
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
    /// optionally evaluating Rules 1–4 against `target` (`verify = true`).
    pub fn sweep_chords(
        &mut self,
        chords: &[SweepChord],
        tools: &[SweepToolSpec],
        target: Option<&TriDexelStock>,
        verify: bool,
    ) -> BatchSweepOutput {
        let mut out = BatchSweepOutput::default();
        let mut linear_buf: Vec<LinearChord3D> = Vec::with_capacity(8);

        for ch in chords {
            let Some(tool) = tools.get(ch.profile_idx) else {
                continue;
            };
            let ctx = SweepContext {
                prof: &tool.profile,
                is_flat_endmill: tool.is_flat_endmill,
                is_rapid: ch.is_rapid,
                tol: ch.verify_tol,
                op_idx: ch.op_idx,
                target,
            };
            motion_to_linear_chords(ch.motion, &mut linear_buf);
            for &lin in &linear_buf {
                let (mutated, viol) = self.apply_linear_chord(lin, &ctx, verify);
                out.mutated |= mutated;
                out.violations.merge(viol);
            }
        }

        out
    }

    fn apply_linear_chord(
        &mut self,
        chord: LinearChord3D,
        ctx: &SweepContext<'_>,
        verify: bool,
    ) -> (bool, SpanRuleViolations) {
        let mut violations = SpanRuleViolations::default();
        let mut mutated = false;

        let (z_mut, z_viol) = self.sweep_z_plane(chord, ctx, verify);
        mutated |= z_mut;
        violations.merge(z_viol);

        // Axis-symmetric horizontal plane sweeps (X and Y) via a single unified kernel!
        for axis in [Axis3::X, Axis3::Y] {
            let (h_mut, h_viol) = self.sweep_horizontal_plane(axis, chord, ctx, verify);
            mutated |= h_mut;
            violations.merge(h_viol);
        }

        (mutated, violations)
    }

    fn sweep_z_plane(
        &mut self,
        chord: LinearChord3D,
        ctx: &SweepContext<'_>,
        verify: bool,
    ) -> (bool, SpanRuleViolations) {
        let r_cut = ctx.prof.cutting_radius_mm;
        let r_check = if verify {
            r_cut.max(ctx.prof.holder_radius_mm)
        } else {
            r_cut
        };

        let min_x = chord.p0.x.min(chord.p1.x) - r_check;
        let max_x = chord.p0.x.max(chord.p1.x) + r_check;
        let min_y = chord.p0.y.min(chord.p1.y) - r_check;
        let max_y = chord.p0.y.max(chord.p1.y) + r_check;
        let neck_skin = (ctx.prof.neck_radius_mm - ctx.tol.radial_wall_skin_mm).max(0.0);
        let shank_skin = (ctx.prof.shank_radius_mm - ctx.tol.radial_wall_skin_mm).max(0.0);
        let r_inner_skin = (r_cut - ctx.tol.radial_wall_skin_mm).max(0.0);

        let plane = self.plane_mut(Axis3::Z);
        let (Some((i0, i1)), Some((j0, j1))) = (
            plane.index_span_u(min_x, max_x),
            plane.index_span_v(min_y, max_y),
        ) else {
            return (false, SpanRuleViolations::default());
        };

        let nx = plane.nu;
        let ox = plane.origin_u_mm;
        let oy = plane.origin_v_mm;
        let oz = plane.origin_w_mm;
        let dx = plane.du_mm;
        let dy = plane.dv_mm;
        let row_slot_stride = nx * INLINE_DEXEL_SLOTS;
        let target_z = ctx.target.map(|t| t.plane(Axis3::Z));

        let slots_slice = &mut plane.slots[j0 * row_slot_stride..(j1 + 1) * row_slot_stride];
        let counts_slice = &mut plane.counts[j0 * nx..(j1 + 1) * nx];
        let overflow_ref = &plane.overflow;

        let (z_mut, z_viol, overflow_deltas) = slots_slice
            .par_chunks_mut(row_slot_stride)
            .zip(counts_slice.par_chunks_mut(nx))
            .enumerate()
            .map(|(j_off, (row_slots, row_counts))| {
                let j = j0 + j_off;
                let py = oy + (j as f64 + 0.5) * dy;
                let row_base = j * nx;
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
                        oz,
                        &mut scratch,
                    );
                    if scratch.is_empty() {
                        continue;
                    }
                    let px = ox + (i as f64 + 0.5) * dx;

                    if verify {
                        let p0_xy = DVec2::new(chord.p0.x, chord.p0.y);
                        let p1_xy = DVec2::new(chord.p1.x, chord.p1.y);
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
                            chord.p0.z + t_c * (chord.p1.z - chord.p0.z)
                        } else {
                            chord.p0.z.min(chord.p1.z)
                        };
                        if let Some(top_span) = scratch.last() {
                            let top_z = top_span.max_mm;
                            if dist_xy <= ctx.prof.holder_radius_mm {
                                let holder_z = z_tip_here + ctx.prof.stickout_length_mm;
                                let pen = top_z - holder_z;
                                if pen > 0.05 {
                                    rv.record_holder(pen, DVec3::new(px, py, holder_z));
                                }
                            }
                            if !ctx.is_rapid && dist_xy <= shank_skin {
                                let reach_top = z_tip_here + ctx.prof.effective_reach_mm();
                                let rub = top_z - reach_top;
                                if rub > 0.15 {
                                    rv.record_flute(rub, DVec3::new(px, py, reach_top));
                                }
                            }
                        }
                        if ctx.prof.has_relieved_neck && !ctx.is_rapid && dist_xy <= neck_skin {
                            let neck_lo = z_tip_here + ctx.prof.flute_length_mm + 0.05;
                            let neck_hi = z_tip_here + ctx.prof.neck_length_mm;
                            if neck_hi > neck_lo {
                                let ov = query_overlap_length(&scratch, neck_lo, neck_hi);
                                if ov > 0.10 {
                                    rv.record_neck(ov, DVec3::new(px, py, neck_lo));
                                }
                            }
                        }
                    }

                    let mut ray_mut = false;
                    if ctx.prof.is_thread_mill {
                        eval_z_ray_thread_mill_intervals(
                            chord,
                            ctx.prof,
                            px,
                            py,
                            r_cut,
                            &mut tm_intervals,
                        );
                        if !tm_intervals.is_empty() {
                            if verify && ctx.is_rapid {
                                for &(z_lo, z_hi) in &tm_intervals {
                                    let ov = query_overlap_length(&scratch, z_lo, z_hi);
                                    if ov > ctx.tol.rapid_crash_tol_mm {
                                        rv.record_rapid(ov, DVec3::new(px, py, z_lo));
                                    }
                                }
                            }
                            if verify
                                && let Some(tz_plane) = target_z
                                && let Some((z_lo_in, z_hi_in)) = eval_z_ray_cylinder_interval(
                                    chord,
                                    px,
                                    py,
                                    neck_skin,
                                    ctx.prof.neck_length_mm,
                                )
                                && let Some((pen, z_at)) = query_target_gouge_depth(
                                    &tz_plane.ray_spans_by_index(row_base + i),
                                    z_lo_in,
                                    z_hi_in,
                                    ctx.tol.gouge_tol_mm,
                                )
                            {
                                rv.record_gouge(pen, DVec3::new(px, py, z_at));
                            }
                            for &(z_lo, z_hi) in &tm_intervals {
                                if subtract_dexel_interval(&mut scratch, z_lo, z_hi, ctx.op_idx)
                                    > 0.0
                                {
                                    ray_mut = true;
                                }
                            }
                        }
                    } else if let Some((z_lo, z_hi, _z_tip)) =
                        eval_z_ray_cut_interval(chord, ctx.prof, ctx.is_flat_endmill, px, py, r_cut)
                    {
                        if verify && ctx.is_rapid {
                            let ov = query_overlap_length(&scratch, z_lo, z_hi);
                            if ov > ctx.tol.rapid_crash_tol_mm {
                                rv.record_rapid(ov, DVec3::new(px, py, z_lo));
                            }
                        }
                        if verify
                            && let Some(tz_plane) = target_z
                            && let Some((z_lo_in, z_hi_in, _)) = eval_z_ray_cut_interval(
                                chord,
                                ctx.prof,
                                ctx.is_flat_endmill,
                                px,
                                py,
                                r_inner_skin,
                            )
                            && let Some((pen, z_at)) = query_target_gouge_depth(
                                &tz_plane.ray_spans_by_index(row_base + i),
                                z_lo_in,
                                z_hi_in,
                                ctx.tol.gouge_tol_mm,
                            )
                        {
                            rv.record_gouge(pen, DVec3::new(px, py, z_at));
                        }
                        if subtract_dexel_interval(&mut scratch, z_lo, z_hi, ctx.op_idx) > 0.0 {
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
                            oz,
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

    /// Axis-generic horizontal dexel sweep parameterized for both `Axis3::X` (`u=Y, v=Z, w=X`)
    /// and `Axis3::Y` (`u=X, v=Z, w=Y`) without duplicated loops.
    fn sweep_horizontal_plane(
        &mut self,
        axis: Axis3,
        chord: LinearChord3D,
        ctx: &SweepContext<'_>,
        verify: bool,
    ) -> (bool, SpanRuleViolations) {
        let u_ax = axis.u_axis();
        let swap_xy = axis == Axis3::Y;
        let r_cut = ctx.prof.cutting_radius_mm;
        let min_u_cut = u_ax.get(chord.p0).min(u_ax.get(chord.p1)) - r_cut;
        let max_u_cut = u_ax.get(chord.p0).max(u_ax.get(chord.p1)) + r_cut;
        let min_z = chord.min_z();
        let max_z = chord.max_z() + ctx.prof.horizontal_cut_max_height_mm();
        let dz_step = 0.25 * self.dz_mm();
        let neck_skin = (ctx.prof.neck_radius_mm - ctx.tol.radial_wall_skin_mm).max(0.0);
        let wall_skin = ctx.tol.gouge_tol_mm + ctx.tol.radial_wall_skin_mm;
        let flute_wall_skin = ctx.tol.radial_wall_skin_mm + (ctx.tol.gouge_tol_mm - 0.02).max(0.0);

        let plane = self.plane_mut(axis);
        let (Some((u0, u1)), Some((k0, k1))) = (
            plane.index_span_u(min_u_cut, max_u_cut),
            plane.index_span_v(min_z, max_z),
        ) else {
            return (false, SpanRuleViolations::default());
        };

        let nu = plane.nu;
        let ou = plane.origin_u_mm;
        let oz = plane.origin_v_mm;
        let ow = plane.origin_w_mm;
        let du = plane.du_mm;
        let dz = plane.dv_mm;
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
                let pz = oz + (k as f64 + 0.5) * dz;
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
                    if let Some((w_lo, w_hi)) = eval_horizontal_ray_cut_interval(
                        chord, ctx.prof, 0.0, pz, pu, swap_xy, dz_step,
                    ) {
                        if verify && ctx.is_rapid {
                            let ov = query_overlap_length(&scratch, w_lo, w_hi);
                            if ov > ctx.tol.rapid_crash_tol_mm {
                                rv.record_rapid(ov, axis.from_uvw(pu, pz, 0.5 * (w_lo + w_hi)));
                            }
                        }
                        if verify
                            && pz >= chord.min_z() + ctx.tol.gouge_tol_mm
                            && let Some(tp) = target_plane
                            && let Some((w_lo_in, w_hi_in)) = if ctx.prof.is_thread_mill {
                                eval_horizontal_ray_cylinder_interval(
                                    chord,
                                    neck_skin,
                                    ctx.prof.neck_length_mm,
                                    pz,
                                    pu,
                                    swap_xy,
                                )
                            } else {
                                eval_horizontal_ray_cut_interval(
                                    chord,
                                    ctx.prof,
                                    flute_wall_skin,
                                    pz,
                                    pu,
                                    swap_xy,
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
                            rv.record_gouge(pen, axis.from_uvw(pu, pz, w_at));
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

/// Exact `f64` single-ray replay of one [`LinearChord3D`] on plane `axis` at ray center `(pu, pv)`.
/// Shared by `WgpuComputeBackend`'s lossless hybrid overflow safety valve so rare `> K = 4`-span
/// rays execute identical subtraction and Rules 1–4 verification to the CPU Tri-Dexel oracle.
#[cfg(feature = "gpu-compute")]
#[allow(clippy::too_many_arguments)]
pub(crate) fn replay_single_ray_linear_chord(
    axis: Axis3,
    pu: f64,
    pv: f64,
    dz_step_mm: f64,
    chord: LinearChord3D,
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
    let r_cut = prof.cutting_radius_mm;
    let neck_skin = (prof.neck_radius_mm - tol.radial_wall_skin_mm).max(0.0);
    let shank_skin = (prof.shank_radius_mm - tol.radial_wall_skin_mm).max(0.0);
    let r_inner_skin = (r_cut - tol.radial_wall_skin_mm).max(0.0);

    match axis {
        Axis3::Z => {
            let px = pu;
            let py = pv;
            let r_check = if verify {
                r_cut.max(prof.holder_radius_mm)
            } else {
                r_cut
            };
            if px < chord.p0.x.min(chord.p1.x) - r_check - 1e-6
                || px > chord.p0.x.max(chord.p1.x) + r_check + 1e-6
                || py < chord.p0.y.min(chord.p1.y) - r_check - 1e-6
                || py > chord.p0.y.max(chord.p1.y) + r_check + 1e-6
            {
                return (false, rv);
            }

            if verify {
                let p0_xy = DVec2::new(chord.p0.x, chord.p0.y);
                let p1_xy = DVec2::new(chord.p1.x, chord.p1.y);
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
                    chord.p0.z + t_c * (chord.p1.z - chord.p0.z)
                } else {
                    chord.p0.z.min(chord.p1.z)
                };
                if let Some(top_span) = scratch.last() {
                    let top_z = top_span.max_mm;
                    if dist_xy <= prof.holder_radius_mm {
                        let holder_z = z_tip_here + prof.stickout_length_mm;
                        let pen = top_z - holder_z;
                        if pen > 0.05 {
                            rv.record_holder(pen, DVec3::new(px, py, holder_z));
                        }
                    }
                    if !is_rapid && dist_xy <= shank_skin {
                        let reach_top = z_tip_here + prof.effective_reach_mm();
                        let rub = top_z - reach_top;
                        if rub > 0.15 {
                            rv.record_flute(rub, DVec3::new(px, py, reach_top));
                        }
                    }
                }
                if prof.has_relieved_neck && !is_rapid && dist_xy <= neck_skin {
                    let neck_lo = z_tip_here + prof.flute_length_mm + 0.05;
                    let neck_hi = z_tip_here + prof.neck_length_mm;
                    if neck_hi > neck_lo {
                        let ov = query_overlap_length(scratch, neck_lo, neck_hi);
                        if ov > 0.10 {
                            rv.record_neck(ov, DVec3::new(px, py, neck_lo));
                        }
                    }
                }
            }

            let mut ray_mut = false;
            if prof.is_thread_mill {
                let mut tm_intervals = Vec::new();
                eval_z_ray_thread_mill_intervals(chord, prof, px, py, r_cut, &mut tm_intervals);
                if !tm_intervals.is_empty() {
                    if verify && is_rapid {
                        for &(z_lo, z_hi) in &tm_intervals {
                            let ov = query_overlap_length(scratch, z_lo, z_hi);
                            if ov > tol.rapid_crash_tol_mm {
                                rv.record_rapid(ov, DVec3::new(px, py, z_lo));
                            }
                        }
                    }
                    if verify
                        && let Some(t_spans) = target_spans
                        && let Some((z_lo_in, z_hi_in)) = eval_z_ray_cylinder_interval(
                            chord,
                            px,
                            py,
                            neck_skin,
                            prof.neck_length_mm,
                        )
                        && let Some((pen, z_at)) =
                            query_target_gouge_depth(t_spans, z_lo_in, z_hi_in, tol.gouge_tol_mm)
                    {
                        rv.record_gouge(pen, DVec3::new(px, py, z_at));
                    }
                    for &(z_lo, z_hi) in &tm_intervals {
                        if subtract_dexel_interval(scratch, z_lo, z_hi, op_idx) > 0.0 {
                            ray_mut = true;
                        }
                    }
                }
            } else if let Some((z_lo, z_hi, _)) =
                eval_z_ray_cut_interval(chord, prof, tool.is_flat_endmill, px, py, r_cut)
            {
                if verify && is_rapid {
                    let ov = query_overlap_length(scratch, z_lo, z_hi);
                    if ov > tol.rapid_crash_tol_mm {
                        rv.record_rapid(ov, DVec3::new(px, py, z_lo));
                    }
                }
                if verify
                    && let Some(t_spans) = target_spans
                    && let Some((z_lo_in, z_hi_in, _)) = eval_z_ray_cut_interval(
                        chord,
                        prof,
                        tool.is_flat_endmill,
                        px,
                        py,
                        r_inner_skin,
                    )
                    && let Some((pen, z_at)) =
                        query_target_gouge_depth(t_spans, z_lo_in, z_hi_in, tol.gouge_tol_mm)
                {
                    rv.record_gouge(pen, DVec3::new(px, py, z_at));
                }
                if subtract_dexel_interval(scratch, z_lo, z_hi, op_idx) > 0.0 {
                    ray_mut = true;
                }
            }
            (ray_mut, rv)
        }
        Axis3::X | Axis3::Y => {
            let swap_xy = axis == Axis3::Y;
            let u_ax = axis.u_axis();
            let pz = pv;
            let u0 = u_ax.get(chord.p0).min(u_ax.get(chord.p1)) - r_cut;
            let u1 = u_ax.get(chord.p0).max(u_ax.get(chord.p1)) + r_cut;
            if pu < u0 - 1e-6
                || pu > u1 + 1e-6
                || pz < chord.min_z() - 1e-6
                || pz > chord.max_z() + prof.horizontal_cut_max_height_mm() + 1e-6
            {
                return (false, rv);
            }
            let wall_skin = tol.gouge_tol_mm + tol.radial_wall_skin_mm;
            let flute_wall_skin = tol.radial_wall_skin_mm + (tol.gouge_tol_mm - 0.02).max(0.0);
            let mut ray_mut = false;
            if let Some((w_lo, w_hi)) =
                eval_horizontal_ray_cut_interval(chord, prof, 0.0, pz, pu, swap_xy, dz_step_mm)
            {
                if verify && is_rapid {
                    let ov = query_overlap_length(scratch, w_lo, w_hi);
                    if ov > tol.rapid_crash_tol_mm {
                        rv.record_rapid(ov, axis.from_uvw(pu, pz, 0.5 * (w_lo + w_hi)));
                    }
                }
                if verify
                    && pz >= chord.min_z() + tol.gouge_tol_mm
                    && let Some(t_spans) = target_spans
                    && let Some((w_lo_in, w_hi_in)) = if prof.is_thread_mill {
                        eval_horizontal_ray_cylinder_interval(
                            chord,
                            neck_skin,
                            prof.neck_length_mm,
                            pz,
                            pu,
                            swap_xy,
                        )
                    } else {
                        eval_horizontal_ray_cut_interval(
                            chord,
                            prof,
                            flute_wall_skin,
                            pz,
                            pu,
                            swap_xy,
                            dz_step_mm,
                        )
                    }
                    && let Some((pen, w_at)) =
                        query_target_gouge_depth(t_spans, w_lo_in, w_hi_in, wall_skin)
                {
                    rv.record_gouge(pen, axis.from_uvw(pu, pz, w_at));
                }
                if subtract_dexel_interval(scratch, w_lo, w_hi, op_idx) > 0.0 {
                    ray_mut = true;
                }
            }
            (ray_mut, rv)
        }
    }
}
