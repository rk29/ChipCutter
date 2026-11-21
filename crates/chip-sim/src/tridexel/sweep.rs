use std::sync::atomic::Ordering;

use chip_cam::{ArcPlane, SegmentMotion3D, ToolpathSegment3D};
use chip_math::{DVec2, DVec3};
use chip_tools::CutterGeometry;
use rayon::prelude::*;

use crate::{
    cutter_profile::RadialCutterProfile,
    heightmap::{
        HeightfieldStock, SetupSimulationInput, SetupSimulationResult, SimError,
        StockPlaybackCursor, UNCUT_OP_IDX, slice_segment_range, target_cursor_from_snap,
    },
    timeline::{MotionSpan, ScrubSnapshot, SimulationTimeline},
    verifier::{SimDiagnostic, SimIssueKind, SimVerificationConfig},
};

use super::grid::{
    SimEngineKind, TriDexelCheckpoint, TriDexelStock, query_overlap_length,
    query_target_gouge_depth, subtract_dexel_interval,
};

#[derive(Clone, Copy)]
struct LinearSpan3D {
    p0: DVec3,
    p1: DVec3,
}

impl LinearSpan3D {
    #[inline]
    fn min_z(self) -> f64 {
        self.p0.z.min(self.p1.z)
    }

    #[inline]
    fn max_z(self) -> f64 {
        self.p0.z.max(self.p1.z)
    }
}

/// Decomposes a [`ToolpathSegment3D`] (linear or circular/helical arc) into linear micro-chords
/// whose chord sag is at most `max_sag_mm`.
fn decompose_to_linear_chords(seg: &ToolpathSegment3D, max_sag_mm: f64) -> Vec<LinearSpan3D> {
    match seg.motion {
        SegmentMotion3D::Rapid | SegmentMotion3D::Linear { .. } => {
            vec![LinearSpan3D {
                p0: seg.start_wcs,
                p1: seg.end_wcs,
            }]
        }
        SegmentMotion3D::Arc {
            center_wcs,
            plane,
            radius_mm,
            signed_sweep_rad,
            ..
        } => {
            let r = radius_mm.max(1e-6);
            let a0 = match plane {
                ArcPlane::XY => {
                    let s = seg.start_wcs - center_wcs;
                    s.y.atan2(s.x)
                }
                ArcPlane::XZ => {
                    let s = seg.start_wcs - center_wcs;
                    s.z.atan2(s.x)
                }
                ArcPlane::YZ => {
                    let s = seg.start_wcs - center_wcs;
                    s.z.atan2(s.y)
                }
            };
            let sweep = signed_sweep_rad;
            // Chord sag s = r * (1 - cos(dtheta / 2)) <= max_sag_mm
            let cos_half = (1.0 - (max_sag_mm / r).min(0.5)).clamp(-1.0, 1.0);
            let max_dtheta = (2.0 * cos_half.acos()).clamp(0.02, 0.25);
            let steps = ((sweep.abs() / max_dtheta).ceil() as usize).clamp(4, 256);

            let mut out = Vec::with_capacity(steps);
            let mut prev = seg.start_wcs;
            for k in 1..=steps {
                let u = k as f64 / steps as f64;
                let ang = a0 + u * sweep;
                let (s, c) = ang.sin_cos();
                let next = if k == steps {
                    seg.end_wcs
                } else {
                    match plane {
                        ArcPlane::XY => DVec3::new(
                            center_wcs.x + r * c,
                            center_wcs.y + r * s,
                            seg.start_wcs.z + u * (seg.end_wcs.z - seg.start_wcs.z),
                        ),
                        ArcPlane::XZ => DVec3::new(
                            center_wcs.x + r * c,
                            seg.start_wcs.y + u * (seg.end_wcs.y - seg.start_wcs.y),
                            center_wcs.z + r * s,
                        ),
                        ArcPlane::YZ => DVec3::new(
                            seg.start_wcs.x + u * (seg.end_wcs.x - seg.start_wcs.x),
                            center_wcs.y + r * c,
                            center_wcs.z + r * s,
                        ),
                    }
                };
                out.push(LinearSpan3D { p0: prev, p1: next });
                prev = next;
            }
            out
        }
    }
}

/// Exact closed-form intersection of horizontal line `y = y_line` with the 2D stadium/capsule
/// of radius `r >= 0` swept along segment `p0 -> p1`.
///
/// Returns `Some((x_min, x_max))` when the line intersects the capsule.
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

    // 1. Endcap circles at p0 and p1
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

    // 2. Two tangent side segments connecting circle(p0, r) and circle(p1, r)
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

/// Computes the exact horizontal cut interval `[u_min, u_max]` along an X-ray (`swap_xy = false`,
/// fixed `y = fixed_v, z = pz`) or Y-ray (`swap_xy = true`, fixed `x = fixed_v, z = pz`) for a
/// vertical-spindle cutter moving linearly from `span.p0` to `span.p1`.
#[inline]
fn eval_horizontal_ray_cut_interval(
    span: LinearSpan3D,
    prof: &RadialCutterProfile,
    radial_shrink_mm: f64,
    pz: f64,
    fixed_v: f64,
    swap_xy: bool,
    dz_step_mm: f64,
) -> Option<(f64, f64)> {
    let max_h = prof.flute_length_mm;
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

    let dz_span = span.p1.z - span.p0.z;
    // Fast exact path for level moves.
    // Half-open lower bound (`dz > 1e-9`) avoids double-counting a ray lying exactly on a flat floor.
    if dz_span.abs() <= 1e-7 {
        let dz = pz - span.p0.z;
        if dz <= 1e-9 || dz > max_h + 1e-9 {
            return None;
        }
        let r = (prof.radius_at_height_dz(dz)? - radial_shrink_mm).max(0.0);
        if r <= 1e-9 {
            return None;
        }
        return intersect_horizontal_line_with_capsule_2d(
            proj2(span.p0),
            proj2(span.p1),
            r,
            fixed_v,
        );
    }

    // Sloped Z move: restrict parameter t ∈ [0, 1] to where 0 <= pz - z(t) <= max_h
    let t_at_z = |target_z: f64| (target_z - span.p0.z) / dz_span;
    let t_low = t_at_z(pz - max_h);
    let t_high = t_at_z(pz);
    let t0 = t_low.min(t_high).clamp(0.0, 1.0);
    let t1 = t_low.max(t_high).clamp(0.0, 1.0);
    if t1 < t0 {
        return None;
    }

    let q0 = span.p0.lerp(span.p1, t0);
    let q1 = span.p0.lerp(span.p1, t1);
    let r_at = |z_tip: f64| -> f64 {
        (prof.radius_at_height_dz(pz - z_tip).unwrap_or(0.0) - radial_shrink_mm).max(0.0)
    };

    let r0 = r_at(q0.z);
    let r1 = r_at(q1.z);
    // If radius is constant across [t0, t1] (e.g., flat endmill or cylinder wall above tip radius),
    // a single exact 2D capsule suffices with zero subdivision!
    if (r0 - r1).abs() <= 1e-7 && r0 > 1e-9 {
        return intersect_horizontal_line_with_capsule_2d(proj2(q0), proj2(q1), r0, fixed_v);
    }

    // Otherwise subdivide smoothly across the curved/conical tip transition
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
                intersect_horizontal_line_with_capsule_2d(proj2(a), proj2(b), r_mid, fixed_v)
        {
            lo = lo.min(c0);
            hi = hi.max(c1);
        }
    }
    if lo.is_finite() && hi.is_finite() && hi > lo {
        Some((lo, hi))
    } else {
        None
    }
}

/// Computes the exact vertical cut interval `[z_min_cut, z_max_cut]` along a Z-ray at `(px, py)`
/// for a vertical-spindle cutter moving linearly along `span`.
#[inline]
fn eval_z_ray_cut_interval(
    span: LinearSpan3D,
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
        let t0 = ((-b - sq) * inv2a).clamp(0.0, 1.0);
        let t1 = ((-b + sq) * inv2a).clamp(0.0, 1.0);
        if t1 < t0 {
            return None;
        }
        let tc = (-f.dot(d_xy) / len2).clamp(t0, t1);
        (t0, t1, tc)
    };

    let dz_span = span.p1.z - span.p0.z;
    if dz_span.abs() <= 1e-7 {
        let pt = p0_xy.lerp(p1_xy, t_closest);
        let r = (q_xy - pt).length();
        let dz = prof.tip_to_envelope_dz(r)?;
        let z_tip = span.p0.z;
        return Some((z_tip + dz, z_tip + prof.flute_length_mm, z_tip));
    }

    if is_flat_endmill {
        let t_min_z = if dz_span < 0.0 { t_exit } else { t_enter };
        let z_tip_lo = span.p0.z + t_min_z * dz_span;
        let t_max_z = if dz_span < 0.0 { t_enter } else { t_exit };
        let z_tip_hi = span.p0.z + t_max_z * dz_span;
        return Some((z_tip_lo, z_tip_hi + prof.flute_length_mm, z_tip_lo));
    }

    // Curved / conical cutter on a sloped 3D move: sample dense stations across [t_enter, t_exit]
    let mut best_z_cut = f64::INFINITY;
    let mut best_z_tip = f64::INFINITY;
    let mut max_z_tip = f64::NEG_INFINITY;
    let n_samples = 8usize;
    for s in 0..=n_samples {
        let u = s as f64 / n_samples as f64;
        let t = t_enter + u * (t_exit - t_enter);
        let pt = p0_xy.lerp(p1_xy, t);
        let r = (q_xy - pt).length();
        if let Some(dz) = prof.tip_to_envelope_dz(r) {
            let z_tip = span.p0.z + t * dz_span;
            let z_cut = z_tip + dz;
            if z_cut < best_z_cut {
                best_z_cut = z_cut;
                best_z_tip = z_tip;
            }
            if z_tip > max_z_tip {
                max_z_tip = z_tip;
            }
        }
    }
    if best_z_cut.is_finite() {
        Some((best_z_cut, max_z_tip + prof.flute_length_mm, best_z_tip))
    } else {
        None
    }
}

struct SweepContext<'a> {
    prof: &'a RadialCutterProfile,
    is_flat_endmill: bool,
    is_rapid: bool,
    cfg: &'a SimVerificationConfig,
    op_idx: u16,
    target: Option<&'a TriDexelStock>,
}

#[derive(Default)]
struct SpanRuleViolations {
    worst_rapid: Option<(f64, DVec3)>,
    worst_gouge: Option<(f64, DVec3)>,
    worst_flute: Option<(f64, DVec3)>,
    worst_holder: Option<(f64, DVec3)>,
}

impl SpanRuleViolations {
    fn merge(&mut self, other: Self) {
        if let Some((d, p)) = other.worst_rapid
            && self.worst_rapid.is_none_or(|(b, _)| d > b)
        {
            self.worst_rapid = Some((d, p));
        }
        if let Some((d, p)) = other.worst_gouge
            && self.worst_gouge.is_none_or(|(b, _)| d > b)
        {
            self.worst_gouge = Some((d, p));
        }
        if let Some((d, p)) = other.worst_flute
            && self.worst_flute.is_none_or(|(b, _)| d > b)
        {
            self.worst_flute = Some((d, p));
        }
        if let Some((d, p)) = other.worst_holder
            && self.worst_holder.is_none_or(|(b, _)| d > b)
        {
            self.worst_holder = Some((d, p));
        }
    }
}

impl TriDexelStock {
    /// Mutation-only sweep of a single [`ToolpathSegment3D`] onto all 3 orthogonal dexel planes
    /// (used for interactive timeline scrubbing).
    pub fn sweep_segment_mutate_only(
        &mut self,
        seg: &ToolpathSegment3D,
        prof: &RadialCutterProfile,
        is_flat_endmill: bool,
        op_idx: u16,
    ) -> bool {
        let max_sag = 0.005;
        let chords = decompose_to_linear_chords(seg, max_sag);
        let mut mutated = false;
        let cfg = SimVerificationConfig::default();
        let ctx = SweepContext {
            prof,
            is_flat_endmill,
            is_rapid: matches!(seg.motion, SegmentMotion3D::Rapid),
            cfg: &cfg,
            op_idx,
            target: None,
        };
        for ch in chords {
            let (changed, _) = self.apply_linear_chord(ch, &ctx, false);
            mutated |= changed;
        }
        mutated
    }

    fn apply_linear_chord(
        &mut self,
        chord: LinearSpan3D,
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
        let min_z = chord.min_z();
        let max_z = chord.max_z() + ctx.prof.flute_length_mm;

        let mut violations = SpanRuleViolations::default();
        let mut mutated = false;

        // 1. Z-rays (X, Y) -> Z intervals + Rules 1/2/3/4
        if let (Some((i0, i1)), Some((j0, j1))) = (
            self.z_rays.index_span_x(min_x, max_x),
            self.z_rays.index_span_y(min_y, max_y),
        ) {
            let nx = self.z_rays.nu;
            let ox = self.z_rays.origin_u_mm;
            let oy = self.z_rays.origin_v_mm;
            let dx = self.z_rays.du_mm;
            let dy = self.z_rays.dv_mm;
            let r_inner_skin = (r_cut - ctx.cfg.radial_wall_skin_mm).max(0.0);
            let target_z_rays = ctx.target.map(|t| &t.z_rays.rays);

            let (z_mut, z_viol) = self.z_rays.rays[j0 * nx..(j1 + 1) * nx]
                .par_chunks_mut(nx)
                .enumerate()
                .map(|(j_off, row)| {
                    let j = j0 + j_off;
                    let py = oy + (j as f64 + 0.5) * dy;
                    let mut row_mut = false;
                    let mut rv = SpanRuleViolations::default();
                    let row_base = j * nx;

                    for (i, spans) in row.iter_mut().enumerate().take(i1 + 1).skip(i0) {
                        if spans.is_empty() {
                            continue;
                        }
                        let px = ox + (i as f64 + 0.5) * dx;

                        // Holder & shank checks (Rules 3 & 4)
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
                            let z_tip_here = chord.p0.z + t_c * (chord.p1.z - chord.p0.z);
                            if let Some(top_span) = spans.last() {
                                let top_z = top_span.max_mm;
                                // Rule 4: Holder collision
                                if dist_xy <= ctx.prof.holder_radius_mm {
                                    let holder_z = z_tip_here + ctx.prof.stickout_length_mm;
                                    let pen = top_z - holder_z;
                                    if pen > 0.05 && rv.worst_holder.is_none_or(|(b, _)| pen > b) {
                                        rv.worst_holder = Some((pen, DVec3::new(px, py, holder_z)));
                                    }
                                }
                                // Rule 3: Flute length exceeded (non-cutting shank rub)
                                if !ctx.is_rapid && dist_xy <= ctx.prof.shank_radius_mm {
                                    let flute_top = z_tip_here + ctx.prof.flute_length_mm;
                                    let rub = top_z - flute_top;
                                    if rub > 0.15 && rv.worst_flute.is_none_or(|(b, _)| rub > b) {
                                        rv.worst_flute = Some((rub, DVec3::new(px, py, flute_top)));
                                    }
                                }
                            }
                        }

                        if let Some((z_lo, z_hi, _z_tip)) = eval_z_ray_cut_interval(
                            chord,
                            ctx.prof,
                            ctx.is_flat_endmill,
                            px,
                            py,
                            r_cut,
                        ) {
                            // Rule 1: G0 rapid into stock
                            if verify && ctx.is_rapid {
                                let ov = query_overlap_length(spans, z_lo, z_hi);
                                if ov > ctx.cfg.rapid_crash_tol_mm
                                    && rv.worst_rapid.is_none_or(|(b, _)| ov > b)
                                {
                                    rv.worst_rapid = Some((ov, DVec3::new(px, py, z_lo)));
                                }
                            }

                            // Rule 2: Target model gouge along Z (using inner wall skin)
                            if verify
                                && let Some(tz_all) = target_z_rays
                                && let Some((z_lo_in, z_hi_in, _)) = eval_z_ray_cut_interval(
                                    chord,
                                    ctx.prof,
                                    ctx.is_flat_endmill,
                                    px,
                                    py,
                                    r_inner_skin,
                                )
                                && let Some((pen, z_at)) = query_target_gouge_depth(
                                    &tz_all[row_base + i],
                                    z_lo_in,
                                    z_hi_in,
                                    ctx.cfg.gouge_tol_mm,
                                )
                                && rv.worst_gouge.is_none_or(|(b, _)| pen > b)
                            {
                                rv.worst_gouge = Some((pen, DVec3::new(px, py, z_at)));
                            }

                            let rem = subtract_dexel_interval(spans, z_lo, z_hi, ctx.op_idx);
                            if rem > 0.0 {
                                row_mut = true;
                            }
                        }
                    }
                    (row_mut, rv)
                })
                .reduce(
                    || (false, SpanRuleViolations::default()),
                    |(m0, mut v0), (m1, v1)| {
                        v0.merge(v1);
                        (m0 || m1, v0)
                    },
                );
            mutated |= z_mut;
            violations.merge(z_viol);
        }

        // 2. X-rays (Y, Z) -> X intervals (exact vertical wall & undercut fidelity!)
        let min_y_cut = chord.p0.y.min(chord.p1.y) - r_cut;
        let max_y_cut = chord.p0.y.max(chord.p1.y) + r_cut;
        let dz_step = 0.25 * self.dz_mm;
        if let (Some((j0, j1)), Some((k0, k1))) = (
            self.x_rays.index_span_u(min_y_cut, max_y_cut),
            self.x_rays.index_span_v(min_z, max_z),
        ) {
            let ny = self.x_rays.nu;
            let oy = self.x_rays.origin_u_mm;
            let oz = self.x_rays.origin_v_mm;
            let dy = self.x_rays.du_mm;
            let dz = self.x_rays.dv_mm;
            let target_x_rays = ctx.target.map(|t| &t.x_rays.rays);
            let wall_skin = ctx.cfg.gouge_tol_mm + ctx.cfg.radial_wall_skin_mm;

            let (x_mut, x_viol) = self.x_rays.rays[k0 * ny..(k1 + 1) * ny]
                .par_chunks_mut(ny)
                .enumerate()
                .map(|(k_off, row)| {
                    let k = k0 + k_off;
                    let pz = oz + (k as f64 + 0.5) * dz;
                    let mut row_mut = false;
                    let mut rv = SpanRuleViolations::default();
                    let row_base = k * ny;

                    for (j, spans) in row.iter_mut().enumerate().take(j1 + 1).skip(j0) {
                        if spans.is_empty() {
                            continue;
                        }
                        let py = oy + (j as f64 + 0.5) * dy;
                        if let Some((x_lo, x_hi)) = eval_horizontal_ray_cut_interval(
                            chord, ctx.prof, 0.0, pz, py, false, dz_step,
                        ) {
                            if verify && ctx.is_rapid {
                                let ov = query_overlap_length(spans, x_lo, x_hi);
                                if ov > ctx.cfg.rapid_crash_tol_mm
                                    && rv.worst_rapid.is_none_or(|(b, _)| ov > b)
                                {
                                    rv.worst_rapid =
                                        Some((ov, DVec3::new(0.5 * (x_lo + x_hi), py, pz)));
                                }
                            }
                            if verify
                                && pz >= chord.min_z() + ctx.cfg.gouge_tol_mm
                                && let Some(tx_all) = target_x_rays
                                && let Some((x_lo_in, x_hi_in)) = eval_horizontal_ray_cut_interval(
                                    chord,
                                    ctx.prof,
                                    ctx.cfg.radial_wall_skin_mm,
                                    pz,
                                    py,
                                    false,
                                    dz_step,
                                )
                                && let Some((pen, x_at)) = query_target_gouge_depth(
                                    &tx_all[row_base + j],
                                    x_lo_in,
                                    x_hi_in,
                                    wall_skin,
                                )
                                && rv.worst_gouge.is_none_or(|(b, _)| pen > b)
                            {
                                rv.worst_gouge = Some((pen, DVec3::new(x_at, py, pz)));
                            }
                            let rem = subtract_dexel_interval(spans, x_lo, x_hi, ctx.op_idx);
                            if rem > 0.0 {
                                row_mut = true;
                            }
                        }
                    }
                    (row_mut, rv)
                })
                .reduce(
                    || (false, SpanRuleViolations::default()),
                    |(m0, mut v0), (m1, v1)| {
                        v0.merge(v1);
                        (m0 || m1, v0)
                    },
                );
            mutated |= x_mut;
            violations.merge(x_viol);
        }

        // 3. Y-rays (X, Z) -> Y intervals (exact vertical wall & undercut fidelity!)
        let min_x_cut = chord.p0.x.min(chord.p1.x) - r_cut;
        let max_x_cut = chord.p0.x.max(chord.p1.x) + r_cut;
        if let (Some((i0, i1)), Some((k0, k1))) = (
            self.y_rays.index_span_u(min_x_cut, max_x_cut),
            self.y_rays.index_span_v(min_z, max_z),
        ) {
            let nx = self.y_rays.nu;
            let ox = self.y_rays.origin_u_mm;
            let oz = self.y_rays.origin_v_mm;
            let dx = self.y_rays.du_mm;
            let dz = self.y_rays.dv_mm;
            let target_y_rays = ctx.target.map(|t| &t.y_rays.rays);
            let wall_skin = ctx.cfg.gouge_tol_mm + ctx.cfg.radial_wall_skin_mm;

            let (y_mut, y_viol) = self.y_rays.rays[k0 * nx..(k1 + 1) * nx]
                .par_chunks_mut(nx)
                .enumerate()
                .map(|(k_off, row)| {
                    let k = k0 + k_off;
                    let pz = oz + (k as f64 + 0.5) * dz;
                    let mut row_mut = false;
                    let mut rv = SpanRuleViolations::default();
                    let row_base = k * nx;

                    for (i, spans) in row.iter_mut().enumerate().take(i1 + 1).skip(i0) {
                        if spans.is_empty() {
                            continue;
                        }
                        let px = ox + (i as f64 + 0.5) * dx;
                        if let Some((y_lo, y_hi)) = eval_horizontal_ray_cut_interval(
                            chord, ctx.prof, 0.0, pz, px, true, dz_step,
                        ) {
                            if verify && ctx.is_rapid {
                                let ov = query_overlap_length(spans, y_lo, y_hi);
                                if ov > ctx.cfg.rapid_crash_tol_mm
                                    && rv.worst_rapid.is_none_or(|(b, _)| ov > b)
                                {
                                    rv.worst_rapid =
                                        Some((ov, DVec3::new(px, 0.5 * (y_lo + y_hi), pz)));
                                }
                            }
                            if verify
                                && pz >= chord.min_z() + ctx.cfg.gouge_tol_mm
                                && let Some(ty_all) = target_y_rays
                                && let Some((y_lo_in, y_hi_in)) = eval_horizontal_ray_cut_interval(
                                    chord,
                                    ctx.prof,
                                    ctx.cfg.radial_wall_skin_mm,
                                    pz,
                                    px,
                                    true,
                                    dz_step,
                                )
                                && let Some((pen, y_at)) = query_target_gouge_depth(
                                    &ty_all[row_base + i],
                                    y_lo_in,
                                    y_hi_in,
                                    wall_skin,
                                )
                                && rv.worst_gouge.is_none_or(|(b, _)| pen > b)
                            {
                                rv.worst_gouge = Some((pen, DVec3::new(px, y_at, pz)));
                            }
                            let rem = subtract_dexel_interval(spans, y_lo, y_hi, ctx.op_idx);
                            if rem > 0.0 {
                                row_mut = true;
                            }
                        }
                    }
                    (row_mut, rv)
                })
                .reduce(
                    || (false, SpanRuleViolations::default()),
                    |(m0, mut v0), (m1, v1)| {
                        v0.merge(v1);
                        (m0 || m1, v0)
                    },
                );
            mutated |= y_mut;
            violations.merge(y_viol);
        }

        (mutated, violations)
    }

    /// Incrementally advances or reconstructs `self` to match the timeline playback position `snap`.
    pub fn sync_to_scrub_snapshot(
        &mut self,
        cursor: &mut StockPlaybackCursor,
        result: &SetupSimulationResult,
        snap: &ScrubSnapshot,
    ) -> bool {
        let total_spans = result.timeline.motion_spans.len();
        let target = target_cursor_from_snap(snap, total_spans);
        if *cursor == target {
            return false;
        }
        let going_backward = target.seg_idx < cursor.seg_idx
            || (target.seg_idx == cursor.seg_idx && target.seg_u + 1e-9 < cursor.seg_u);

        let mut mutated = false;
        if going_backward {
            if let Some(cp) = result
                .tridexel_checkpoints
                .iter()
                .rev()
                .find(|c| c.completed_segments <= target.seg_idx)
            {
                *self = cp.stock.clone();
                *cursor = StockPlaybackCursor {
                    seg_idx: cp.completed_segments,
                    seg_u: 0.0,
                };
                mutated = true;
            }
        } else if let Some(cp) = result.tridexel_checkpoints.iter().rev().find(|c| {
            c.completed_segments <= target.seg_idx && c.completed_segments > cursor.seg_idx
        }) {
            *self = cp.stock.clone();
            *cursor = StockPlaybackCursor {
                seg_idx: cp.completed_segments,
                seg_u: 0.0,
            };
            mutated = true;
        }

        while cursor.seg_idx < target.seg_idx {
            let span = &result.timeline.motion_spans[cursor.seg_idx];
            if let Some(&(prof, is_flat)) = result.operation_profiles.get(span.op_idx) {
                let seg_opt = if cursor.seg_u > 1e-9 {
                    slice_segment_range(&span.segment, cursor.seg_u, 1.0)
                } else {
                    Some(span.segment)
                };
                if let Some(seg) = seg_opt {
                    mutated |=
                        self.sweep_segment_mutate_only(&seg, &prof, is_flat, span.op_idx as u16);
                }
            }
            cursor.seg_idx += 1;
            cursor.seg_u = 0.0;
        }

        if target.seg_idx < total_spans && target.seg_u > cursor.seg_u + 1e-9 {
            let span = &result.timeline.motion_spans[target.seg_idx];
            if let Some(&(prof, is_flat)) = result.operation_profiles.get(span.op_idx)
                && let Some(seg) = slice_segment_range(&span.segment, cursor.seg_u, target.seg_u)
            {
                mutated |= self.sweep_segment_mutate_only(&seg, &prof, is_flat, span.op_idx as u16);
            }
            cursor.seg_u = target.seg_u;
        }

        mutated
    }

    /// Exports a projected Z-heightfield view from this Tri-Dexel stock so legacy heightmap readers
    /// have a valid `HeightfieldStock` populated on `SetupSimulationResult`.
    pub fn to_projected_heightfield(&self) -> HeightfieldStock {
        let nx = self.nx;
        let ny = self.ny;
        let total = nx * ny;
        let mut in_stock = vec![false; total];
        let mut z_stock = vec![f32::NEG_INFINITY; total];
        let mut z_bottom = vec![f32::NEG_INFINITY; total];
        let z_target = vec![f32::NEG_INFINITY; total];
        let mut last_op_idx = vec![UNCUT_OP_IDX; total];

        for j in 0..ny {
            let row = j * nx;
            for i in 0..nx {
                let spans = self.z_rays.ray(i, j);
                if let (Some(first), Some(last)) = (spans.first(), spans.last()) {
                    let idx = row + i;
                    in_stock[idx] = true;
                    z_bottom[idx] = first.min_mm as f32;
                    z_stock[idx] = last.max_mm as f32;
                    last_op_idx[idx] = last.max_op_idx;
                }
            }
        }

        HeightfieldStock {
            nx,
            ny,
            origin_x_mm: self.origin_mm.x,
            origin_y_mm: self.origin_mm.y,
            dx_mm: self.dx_mm,
            dy_mm: self.dy_mm,
            max_initial_top_z_mm: self.origin_mm.z + (self.nz as f64) * self.dz_mm,
            min_bottom_z_mm: self.origin_mm.z,
            in_stock,
            z_stock,
            z_bottom,
            z_target,
            last_op_idx,
        }
    }
}

/// Helper alias so `index_span_x/y` work cleanly on `z_rays`.
impl super::grid::DexelPlane {
    #[inline]
    pub(crate) fn index_span_x(&self, min_x: f64, max_x: f64) -> Option<(usize, usize)> {
        self.index_span_u(min_x, max_x)
    }

    #[inline]
    pub(crate) fn index_span_y(&self, min_y: f64, max_y: f64) -> Option<(usize, usize)> {
        self.index_span_v(min_y, max_y)
    }
}

/// Runs a full 3D **Tri-Dexel volumetric stock simulation & 5-rule verification** for a Setup.
pub fn run_tridexel_setup_simulation(
    input: &SetupSimulationInput<'_>,
) -> Result<SetupSimulationResult, SimError> {
    let timeline = SimulationTimeline::from_operations(input.operations, input.kinematics);
    let mut stock = TriDexelStock::from_setup_stock(&input.setup.stock, input.resolution)?;
    let initial_vol = stock.compute_volume_mm3();

    let has_target_geo = !input.doc.render_mesh.pick_triangles.is_empty();
    let target_stock = if has_target_geo {
        Some(TriDexelStock::from_target_cad(
            input.doc,
            input.setup,
            input.target_bodies,
            &stock,
        ))
    } else {
        None
    };

    let operation_profiles: Vec<(RadialCutterProfile, bool)> = input
        .operations
        .iter()
        .map(|op| {
            (
                RadialCutterProfile::from_tool(op.tool),
                matches!(op.tool.geometry, CutterGeometry::FlatEndmill { .. }),
            )
        })
        .collect();

    let total_spans = timeline.motion_spans.len();
    let checkpoint_interval = (total_spans / 10).max(120);
    let mut tridexel_checkpoints = vec![TriDexelCheckpoint {
        completed_segments: 0,
        stock: stock.clone(),
    }];

    let max_sag = 0.005;
    let mut diagnostics: Vec<SimDiagnostic> = Vec::new();
    let mut recorded_op_kinds: Vec<(usize, SimIssueKind)> = Vec::new();
    let mut per_op_errors: Vec<crate::metrics::OperationErrorSnapshot> = Vec::new();
    let mut prev_vol = initial_vol;

    for (span_idx, span) in timeline.motion_spans.iter().enumerate() {
        if let Some(cancel) = input.cancel
            && cancel.load(Ordering::Relaxed)
        {
            return Err(SimError::Cancelled);
        }

        let op_in = &input.operations[span.op_idx];
        let (prof, is_flat) = operation_profiles[span.op_idx];
        let mut op_cfg = input.verify_config;
        op_cfg.gouge_tol_mm += op_in.intentional_edge_break_mm.max(0.0);

        let is_rapid = matches!(span.segment.motion, SegmentMotion3D::Rapid);

        // Rule 5: Vertical Plunge check
        if !is_rapid {
            let dxy = (span.segment.end_wcs.x - span.segment.start_wcs.x)
                .hypot(span.segment.end_wcs.y - span.segment.start_wcs.y);
            if dxy <= 1e-4
                && span.segment.end_wcs.z < span.segment.start_wcs.z - 1e-4
                && let Some(top_z) =
                    stock.sample_top_z(span.segment.start_wcs.x, span.segment.start_wcs.y)
            {
                let entry_z = top_z.min(span.segment.start_wcs.z);
                let plunge_in_mat = entry_z - span.segment.end_wcs.z;
                if plunge_in_mat > 0.05 {
                    let trigger_detail = if !prof.is_center_cutting {
                        Some((
                            plunge_in_mat,
                            format!(
                                "Non-center-cutting tool T{} plunged vertically {plunge_in_mat:.2} mm into stock (use Helical or Linear ramp).",
                                op_in.tool.tool_number
                            ),
                        ))
                    } else if plunge_in_mat > prof.max_safe_plunge_depth_mm + 0.05 {
                        Some((
                            plunge_in_mat - prof.max_safe_plunge_depth_mm,
                            format!(
                                "Vertical plunge depth ({plunge_in_mat:.2} mm) exceeds safe axial plunge threshold ({:.2} mm) on T{}.",
                                prof.max_safe_plunge_depth_mm, op_in.tool.tool_number
                            ),
                        ))
                    } else {
                        None
                    };
                    if let Some((excess, msg)) = trigger_detail {
                        push_dedup_diag(
                            &mut diagnostics,
                            &mut recorded_op_kinds,
                            span,
                            op_in,
                            SimIssueKind::UnsafeVerticalPlunge,
                            excess,
                            span.segment.end_wcs,
                            msg,
                        );
                    }
                }
            }
        }

        let ctx = SweepContext {
            prof: &prof,
            is_flat_endmill: is_flat,
            is_rapid,
            cfg: &op_cfg,
            op_idx: span.op_idx as u16,
            target: target_stock.as_ref(),
        };

        let chords = decompose_to_linear_chords(&span.segment, max_sag);
        let mut span_viol = SpanRuleViolations::default();
        for ch in chords {
            let (_, v) = stock.apply_linear_chord(ch, &ctx, true);
            span_viol.merge(v);
        }

        if let Some((pen, loc)) = span_viol.worst_rapid {
            push_dedup_diag(
                &mut diagnostics,
                &mut recorded_op_kinds,
                span,
                op_in,
                SimIssueKind::RapidIntoStock,
                pen,
                loc,
                format!(
                    "G0 rapid move crashed {pen:.2} mm into unmachined stock at WCS ({:.2}, {:.2}, {:.2}).",
                    loc.x, loc.y, loc.z
                ),
            );
        }
        if let Some((pen, loc)) = span_viol.worst_gouge {
            push_dedup_diag(
                &mut diagnostics,
                &mut recorded_op_kinds,
                span,
                op_in,
                SimIssueKind::TargetModelGouge,
                pen,
                loc,
                format!(
                    "Tool T{} gouged target model by {pen:.3} mm (tolerance {:.3} mm) at WCS ({:.2}, {:.2}, {:.2}).",
                    op_in.tool.tool_number, op_cfg.gouge_tol_mm, loc.x, loc.y, loc.z
                ),
            );
        }
        if let Some((rub, loc)) = span_viol.worst_flute {
            push_dedup_diag(
                &mut diagnostics,
                &mut recorded_op_kinds,
                span,
                op_in,
                SimIssueKind::FluteLengthExceeded,
                rub,
                loc,
                format!(
                    "Cut depth exceeded T{} flute length ({:.2} mm) by {rub:.2} mm — non-cutting shank rub.",
                    op_in.tool.tool_number, prof.flute_length_mm
                ),
            );
        }
        if let Some((pen, loc)) = span_viol.worst_holder {
            push_dedup_diag(
                &mut diagnostics,
                &mut recorded_op_kinds,
                span,
                op_in,
                SimIssueKind::HolderCollision,
                pen,
                loc,
                format!(
                    "Collet/holder collided {pen:.2} mm with uncut stock above T{} stickout ({:.2} mm).",
                    op_in.tool.tool_number, prof.stickout_length_mm
                ),
            );
        }

        let completed = span_idx + 1;
        let is_op_end = timeline.operations[span.op_idx].end_global_seg == completed;
        if is_op_end {
            let cur_vol = stock.compute_volume_mm3();
            let op_removed = (prev_vol - cur_vol).max(0.0);
            prev_vol = cur_vol;
            if let Some(target_ref) = target_stock.as_ref() {
                let (ex_v, g_v, rms) =
                    crate::metrics::compute_tridexel_quick_error_summary(&stock, target_ref);
                per_op_errors.push(crate::metrics::OperationErrorSnapshot {
                    op_idx: span.op_idx,
                    op_id: op_in.op_id.to_string(),
                    op_name: op_in.op_name.to_string(),
                    removed_volume_mm3: op_removed,
                    excess_volume_mm3: ex_v,
                    gouge_volume_mm3: g_v,
                    total_error_volume_mm3: ex_v + g_v,
                    rms_error_mm: rms,
                });
            }
        }
        if (completed % checkpoint_interval == 0 || is_op_end) && completed < total_spans {
            tridexel_checkpoints.push(TriDexelCheckpoint {
                completed_segments: completed,
                stock: stock.clone(),
            });
        }
    }

    let remaining_vol = stock.compute_volume_mm3();
    let removed_vol = (initial_vol - remaining_vol).max(0.0);
    let error_report = target_stock.as_ref().map(|target_ref| {
        crate::metrics::compute_tridexel_error_report(
            &stock,
            target_ref,
            initial_vol,
            0.05,
            per_op_errors,
        )
    });
    let projected_hf = stock.to_projected_heightfield();

    Ok(SetupSimulationResult {
        engine: SimEngineKind::TriDexel,
        resolution: input.resolution,
        timeline,
        initial_stock_volume_mm3: initial_vol,
        remaining_stock_volume_mm3: remaining_vol,
        removed_volume_mm3: removed_vol,
        final_stock: projected_hf,
        final_cylindrical_stock: None,
        final_tridexel_stock: Some(stock),
        target_tridexel_stock: target_stock,
        checkpoints: Vec::new(),
        cylindrical_checkpoints: Vec::new(),
        tridexel_checkpoints,
        operation_profiles,
        diagnostics,
        error_report,
    })
}

#[allow(clippy::too_many_arguments)]
fn push_dedup_diag(
    diagnostics: &mut Vec<SimDiagnostic>,
    recorded_op_kinds: &mut Vec<(usize, SimIssueKind)>,
    span: &MotionSpan,
    op_in: &crate::timeline::SimOperationInput<'_>,
    kind: SimIssueKind,
    excess_mm: f64,
    location_wcs: DVec3,
    message: String,
) {
    if let Some(existing) = diagnostics
        .iter_mut()
        .find(|d| d.op_idx == span.op_idx && d.kind == kind)
    {
        if excess_mm > existing.max_excess_mm {
            existing.max_excess_mm = excess_mm;
            existing.location_wcs = location_wcs;
            existing.global_seg_idx = span.global_seg_idx;
            existing.timeline_time_sec = span.t_start_sec;
            existing.message = message;
        }
        return;
    }
    recorded_op_kinds.push((span.op_idx, kind));
    diagnostics.push(SimDiagnostic {
        severity: kind.default_severity(),
        kind,
        op_idx: span.op_idx,
        op_id: op_in.op_id.to_string(),
        op_name: op_in.op_name.to_string(),
        tool_number: op_in.tool.tool_number,
        global_seg_idx: span.global_seg_idx,
        timeline_time_sec: span.t_start_sec,
        location_wcs,
        max_excess_mm: excess_mm,
        message,
    });
}
