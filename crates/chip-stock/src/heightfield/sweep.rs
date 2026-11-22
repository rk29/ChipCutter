use chip_math::{Aabb2, DVec2, DVec3};
use rayon::prelude::*;

use crate::types::{
    BatchSweepOutput, RadialCutterProfile, SpanRuleViolations, StockVerifyTolerances, SweepChord,
    SweepMotion, SweepToolSpec,
};

use super::HeightfieldStock;

#[derive(Clone, Copy)]
struct SweepEnv<'a> {
    i0: usize,
    i1: usize,
    ox: f64,
    oz: f64,
    dx: f64,
    prim: SweepMotion,
    prof: &'a RadialCutterProfile,
    is_flat_endmill: bool,
    is_rapid: bool,
    check_holder: bool,
    r_inner_skin: f64,
    tol: StockVerifyTolerances,
    op_idx: u32,
}

struct RowCellsMut<'a> {
    z_bottom: &'a [f32],
    z_target: &'a [f32],
    z_stock: &'a mut [f32],
    last_op: &'a mut [u32],
}

impl HeightfieldStock {
    /// Sweeps a batch of [`SweepChord`] primitives onto this heightfield, optionally evaluating
    /// Rules 1–4 (`verify = true`) and returning combined mutation status & worst-case violations.
    pub fn sweep_chords(
        &mut self,
        chords: &[SweepChord],
        tools: &[SweepToolSpec],
        verify: bool,
    ) -> BatchSweepOutput {
        let mut out = BatchSweepOutput::default();
        for ch in chords {
            let Some(tool) = tools.get(ch.profile_idx) else {
                continue;
            };
            if verify {
                let (mutated, viol) = self.apply_motion_verified(
                    ch.motion,
                    &tool.profile,
                    tool.is_flat_endmill,
                    ch.is_rapid,
                    ch.verify_tol,
                    ch.op_idx,
                );
                out.mutated |= mutated;
                out.violations.merge(viol);
            } else {
                out.mutated |= self.apply_motion_mutate(
                    ch.motion,
                    &tool.profile,
                    tool.is_flat_endmill,
                    ch.op_idx,
                );
            }
        }
        out
    }

    fn apply_motion_mutate(
        &mut self,
        prim: SweepMotion,
        prof: &RadialCutterProfile,
        is_flat_endmill: bool,
        op_idx: u32,
    ) -> bool {
        let r_cut = prof.cutting_radius_mm;
        let aabb = motion_xy_bounds(&prim, r_cut);
        let (Some((i0, i1)), Some((j0, j1))) = (
            self.index_span_x(aabb.min.x, aabb.max.x),
            self.index_span_y(aabb.min.y, aabb.max.y),
        ) else {
            return false;
        };

        let oz = self.origin_wcs.z;
        let min_possible_z_local = (motion_min_tip_z(&prim) - oz) as f32;
        let mut mutated = false;
        for j in j0..=j1 {
            let py = self.origin_wcs.y + (j as f64 + 0.5) * self.dy_mm;
            let row = j * self.nx;
            for i in i0..=i1 {
                let idx = row + i;
                let zb = self.z_bottom[idx];
                if !zb.is_finite() || self.z_stock[idx] <= min_possible_z_local {
                    continue;
                }
                let px = self.origin_wcs.x + (i as f64 + 0.5) * self.dx_mm;
                if let Some(hit) = eval_cutter_at_xy(&prim, px, py, prof, is_flat_endmill) {
                    let z_cut_local = (hit.z_cut_mm - oz) as f32;
                    if z_cut_local < self.z_stock[idx] {
                        self.z_stock[idx] = if z_cut_local <= zb {
                            f32::NEG_INFINITY
                        } else {
                            z_cut_local
                        };
                        self.last_op_idx[idx] = op_idx;
                        mutated = true;
                    }
                }
            }
        }
        mutated
    }

    fn apply_motion_verified(
        &mut self,
        prim: SweepMotion,
        prof: &RadialCutterProfile,
        is_flat_endmill: bool,
        is_rapid: bool,
        tol: StockVerifyTolerances,
        op_idx: u32,
    ) -> (bool, SpanRuleViolations) {
        let check_holder =
            motion_min_tip_z(&prim) + prof.stickout_length_mm < self.max_initial_top_z_mm - 0.02;
        let sweep_r = if check_holder {
            prof.max_assembly_radius_mm()
        } else {
            prof.cutting_radius_mm.max(prof.shank_radius_mm)
        };
        let aabb = motion_xy_bounds(&prim, sweep_r);
        let (Some((i0, i1)), Some((j0, j1))) = (
            self.index_span_x(aabb.min.x, aabb.max.x),
            self.index_span_y(aabb.min.y, aabb.max.y),
        ) else {
            return (false, SpanRuleViolations::default());
        };

        let row_count = j1 - j0 + 1;
        let ox = self.origin_wcs.x;
        let oy = self.origin_wcs.y;
        let oz = self.origin_wcs.z;
        let dx = self.dx_mm;
        let dy = self.dy_mm;
        let nx = self.nx;
        let r_inner_skin = (prof.cutting_radius_mm - tol.radial_wall_skin_mm).max(0.0);

        let env = SweepEnv {
            i0,
            i1,
            ox,
            oz,
            dx,
            prim,
            prof,
            is_flat_endmill,
            is_rapid,
            check_holder,
            r_inner_skin,
            tol,
            op_idx,
        };

        let mut mutated = false;
        let mut violations = SpanRuleViolations::default();

        if row_count >= 24 {
            let z_stock_slice = &mut self.z_stock[j0 * nx..(j1 + 1) * nx];
            let last_op_slice = &mut self.last_op_idx[j0 * nx..(j1 + 1) * nx];
            let z_bot_ref = &self.z_bottom[j0 * nx..(j1 + 1) * nx];
            let z_tgt_ref = &self.z_target[j0 * nx..(j1 + 1) * nx];

            let row_reducers: Vec<(bool, SpanRuleViolations)> = z_stock_slice
                .par_chunks_mut(nx)
                .zip(last_op_slice.par_chunks_mut(nx))
                .enumerate()
                .map(|(rel_j, (row_zs, row_op))| {
                    let j = j0 + rel_j;
                    let py = oy + (j as f64 + 0.5) * dy;
                    let row_base = rel_j * nx;
                    eval_row_cells(
                        py,
                        RowCellsMut {
                            z_bottom: &z_bot_ref[row_base..row_base + nx],
                            z_target: &z_tgt_ref[row_base..row_base + nx],
                            z_stock: row_zs,
                            last_op: row_op,
                        },
                        &env,
                    )
                })
                .collect();

            for (rm, rr) in row_reducers {
                mutated |= rm;
                violations.merge(rr);
            }
        } else {
            for j in j0..=j1 {
                let py = oy + (j as f64 + 0.5) * dy;
                let base = j * nx;
                let (rm, rr) = eval_row_cells(
                    py,
                    RowCellsMut {
                        z_bottom: &self.z_bottom[base..base + nx],
                        z_target: &self.z_target[base..base + nx],
                        z_stock: &mut self.z_stock[base..base + nx],
                        last_op: &mut self.last_op_idx[base..base + nx],
                    },
                    &env,
                );
                mutated |= rm;
                violations.merge(rr);
            }
        }

        (mutated, violations)
    }
}

fn eval_row_cells(py: f64, row: RowCellsMut<'_>, env: &SweepEnv<'_>) -> (bool, SpanRuleViolations) {
    let mut mutated = false;
    let mut out = SpanRuleViolations::default();
    let shank_skin = (env.prof.shank_radius_mm - env.tol.radial_wall_skin_mm).max(0.0);
    let neck_skin = (env.prof.neck_radius_mm - env.tol.radial_wall_skin_mm).max(0.0);
    let holder_skin = (env.prof.holder_radius_mm - env.tol.radial_wall_skin_mm).max(0.0);
    let reach_mm = env.prof.effective_reach_mm();
    let oz = env.oz;

    for i in env.i0..=env.i1 {
        let px = env.ox + (i as f64 + 0.5) * env.dx;
        let zs_before = row.z_stock[i];
        let zb = row.z_bottom[i];
        let has_stock = zb.is_finite() && zs_before > zb;
        let zt = row.z_target[i];
        let has_target = zt.is_finite();

        if !has_stock && !has_target {
            continue;
        }

        // Check holder collision when deep enough (`r <= holder_skin`)
        if env.check_holder
            && has_stock
            && let Some((r_axis, z_tip_min)) =
                min_tip_z_within_radius(&env.prim, px, py, holder_skin)
            && r_axis <= holder_skin
        {
            let holder_z = z_tip_min + env.prof.stickout_length_mm;
            let pen = (oz + f64::from(zs_before)) - holder_z;
            if pen > 0.03 {
                out.record_holder(pen, DVec3::new(px, py, holder_z));
            }
        }

        // Evaluate cutter cutting envelope
        if let Some(hit) = eval_cutter_at_xy(&env.prim, px, py, env.prof, env.is_flat_endmill) {
            let z_cut = hit.z_cut_mm;

            // Rule 2: Target Model Gouge (for ThreadMill, check only relieved-neck core `r <= neck_skin`
            // so intentional helical thread-groove engagement into smooth pilot walls is exempted).
            let gouge_r_limit = if env.prof.is_thread_mill {
                neck_skin
            } else {
                env.r_inner_skin
            };
            if has_target && hit.r_mm <= gouge_r_limit {
                let gouge = (oz + f64::from(zt)) - z_cut;
                if gouge > env.tol.gouge_tol_mm {
                    out.record_gouge(gouge, DVec3::new(px, py, z_cut));
                }
            }

            if has_stock {
                let stock_z_wcs = oz + f64::from(zs_before);
                let cut_depth = stock_z_wcs - z_cut;

                // Rule 1: Rapid-into-Stock Crash
                if env.is_rapid
                    && hit.r_mm <= env.r_inner_skin
                    && cut_depth > env.tol.rapid_crash_tol_mm
                {
                    out.record_rapid(cut_depth, DVec3::new(px, py, z_cut));
                }

                // Relieved-neck clearance check for ThreadMill (`r <= neck_skin` above cutting teeth)
                if env.prof.is_thread_mill
                    && !env.is_rapid
                    && hit.r_mm <= neck_skin
                    && cut_depth > 0.02
                {
                    let neck_bottom_z = hit.z_tip_mm + env.prof.flute_length_mm;
                    let neck_pen = stock_z_wcs - neck_bottom_z;
                    if neck_pen > 0.15 {
                        out.record_neck(neck_pen, DVec3::new(px, py, neck_bottom_z));
                    }
                }

                // Rule 3: Shank rub above usable reach (`neck_length_mm` for ThreadMill, `flute_length_mm` otherwise)
                if hit.r_mm <= env.r_inner_skin.max(shank_skin) && cut_depth > 0.02 {
                    let engagement = stock_z_wcs - hit.z_tip_mm;
                    let excess_flute = engagement - reach_mm;
                    if excess_flute > 0.05 {
                        out.record_flute(excess_flute, DVec3::new(px, py, hit.z_tip_mm));
                    }
                }

                // Mutate remaining heightmap stock in stock-local `f32`!
                let z_cut_local = (z_cut - oz) as f32;
                if z_cut_local < zs_before {
                    row.z_stock[i] = if z_cut_local <= zb {
                        f32::NEG_INFINITY
                    } else {
                        z_cut_local
                    };
                    row.last_op[i] = env.op_idx;
                    mutated = true;
                }
            }
        } else if has_stock && env.prof.shank_radius_mm > env.prof.cutting_radius_mm + 1e-4 {
            // Reduced-neck / small cutter with wider shank (`cutting_r < r <= shank_skin`)
            if let Some((r_axis, z_tip_min)) =
                min_tip_z_within_radius(&env.prim, px, py, shank_skin)
                && r_axis <= shank_skin
            {
                let shank_bottom_z = z_tip_min + reach_mm;
                let rub = (oz + f64::from(zs_before)) - shank_bottom_z;
                if rub > 0.05 {
                    out.record_flute(rub, DVec3::new(px, py, shank_bottom_z));
                }
            }
        }
    }

    (mutated, out)
}

#[derive(Clone, Copy)]
struct CutterCellHit {
    z_cut_mm: f64,
    z_tip_mm: f64,
    r_mm: f64,
}

#[inline]
fn motion_min_tip_z(prim: &SweepMotion) -> f64 {
    match *prim {
        SweepMotion::Linear { p0, p1, .. } => p0.z.min(p1.z),
        SweepMotion::PlanarArcXY { start, end, .. } => start.z.min(end.z),
    }
}

#[inline]
fn motion_xy_bounds(prim: &SweepMotion, expand_r: f64) -> Aabb2 {
    match *prim {
        SweepMotion::Linear { p0, p1, .. } => Aabb2 {
            min: DVec2::new(p0.x.min(p1.x) - expand_r, p0.y.min(p1.y) - expand_r),
            max: DVec2::new(p0.x.max(p1.x) + expand_r, p0.y.max(p1.y) + expand_r),
        },
        SweepMotion::PlanarArcXY {
            center_xy,
            radius_mm,
            ..
        } => {
            let r_tot = radius_mm + expand_r;
            Aabb2 {
                min: DVec2::new(center_xy.x - r_tot, center_xy.y - r_tot),
                max: DVec2::new(center_xy.x + r_tot, center_xy.y + r_tot),
            }
        }
    }
}

fn min_tip_z_within_radius(prim: &SweepMotion, px: f64, py: f64, r_max: f64) -> Option<(f64, f64)> {
    if r_max <= 0.0 {
        return None;
    }
    match *prim {
        SweepMotion::Linear { p0, p1, .. } => {
            let dx = p1.x - p0.x;
            let dy = p1.y - p0.y;
            let l2 = dx * dx + dy * dy;
            if l2 <= 1e-12 {
                let r = (px - p0.x).hypot(py - p0.y);
                return (r <= r_max).then(|| (r, p0.z.min(p1.z)));
            }
            let tc = ((px - p0.x) * dx + (py - p0.y) * dy) / l2;
            let qx = p0.x + tc * dx;
            let qy = p0.y + tc * dy;
            let d_perp_sq = (px - qx).powi(2) + (py - qy).powi(2);
            let r2 = r_max * r_max;
            if d_perp_sq > r2 {
                return None;
            }
            let dt = ((r2 - d_perp_sq) / l2).max(0.0).sqrt();
            let t_lo = (tc - dt).clamp(0.0, 1.0);
            let t_hi = (tc + dt).clamp(0.0, 1.0);
            if t_lo > t_hi {
                return None;
            }
            let t_best = if p1.z >= p0.z { t_lo } else { t_hi };
            let t_clamped = tc.clamp(0.0, 1.0);
            let r_min = (px - (p0.x + t_clamped * dx)).hypot(py - (p0.y + t_clamped * dy));
            let z_min = p0.z + t_best * (p1.z - p0.z);
            Some((r_min, z_min))
        }
        SweepMotion::PlanarArcXY {
            start,
            end,
            center_xy,
            radius_mm,
            a0_rad,
            signed_sweep_rad,
            ..
        } => {
            let r = planar_arc_xy_distance(
                px,
                py,
                start,
                end,
                center_xy,
                radius_mm,
                a0_rad,
                signed_sweep_rad,
            );
            (r <= r_max).then_some((r, start.z))
        }
    }
}

fn eval_cutter_at_xy(
    prim: &SweepMotion,
    px: f64,
    py: f64,
    prof: &RadialCutterProfile,
    is_flat_endmill: bool,
) -> Option<CutterCellHit> {
    let r_max = prof.cutting_radius_mm;
    match *prim {
        SweepMotion::Linear {
            p0: start, p1: end, ..
        } => {
            let dx = end.x - start.x;
            let dy = end.y - start.y;
            let dz = end.z - start.z;
            let l2 = dx * dx + dy * dy;

            if l2 <= 1e-12 {
                // Pure vertical move
                let r = (px - start.x).hypot(py - start.y);
                let p_dz = prof.tip_to_envelope_dz(r)?;
                let z_tip = start.z.min(end.z);
                return Some(CutterCellHit {
                    z_cut_mm: z_tip + p_dz,
                    z_tip_mm: z_tip,
                    r_mm: r,
                });
            }

            if dz.abs() <= 1e-8 {
                // Horizontal 2D linear move (exact closed-form)
                let t = (((px - start.x) * dx + (py - start.y) * dy) / l2).clamp(0.0, 1.0);
                let qx = start.x + t * dx;
                let qy = start.y + t * dy;
                let r = (px - qx).hypot(py - qy);
                let p_dz = prof.tip_to_envelope_dz(r)?;
                return Some(CutterCellHit {
                    z_cut_mm: start.z + p_dz,
                    z_tip_mm: start.z,
                    r_mm: r,
                });
            }

            // Sloped 3D linear move (exact `[t_lo, t_hi]` interval inside cylinder of radius `r_max`)
            let tc = ((px - start.x) * dx + (py - start.y) * dy) / l2;
            let qx_inf = start.x + tc * dx;
            let qy_inf = start.y + tc * dy;
            let d_perp_sq = (px - qx_inf).powi(2) + (py - qy_inf).powi(2);
            let r2 = r_max * r_max;
            if d_perp_sq > r2 {
                return None;
            }
            let dt = ((r2 - d_perp_sq) / l2).max(0.0).sqrt();
            let t_lo = (tc - dt).clamp(0.0, 1.0);
            let t_hi = (tc + dt).clamp(0.0, 1.0);
            if t_lo > t_hi {
                return None;
            }

            let t_min_r = tc.clamp(t_lo, t_hi);
            let min_r = (px - (start.x + t_min_r * dx)).hypot(py - (start.y + t_min_r * dy));

            if is_flat_endmill {
                let t_opt = if dz >= 0.0 { t_lo } else { t_hi };
                let z_tip = start.z + t_opt * dz;
                return Some(CutterCellHit {
                    z_cut_mm: z_tip,
                    z_tip_mm: z_tip,
                    r_mm: min_r,
                });
            }

            // Shaped cutter on sloped 3D ramp: evaluate `[t_lo, t_hi]`, `t_min_r`, AND exact
            // closed-form stationary point `t*` for conical V-bits/chamfer cutters!
            let mut best: Option<CutterCellHit> = None;
            let mut consider_t = |t_cand: f64| {
                let t = t_cand.clamp(t_lo, t_hi);
                let r = (px - (start.x + t * dx)).hypot(py - (start.y + t * dy));
                if let Some(p_dz) = prof.tip_to_envelope_dz(r) {
                    let z_tip = start.z + t * dz;
                    let z_cut = z_tip + p_dz;
                    if best.is_none_or(|b| z_cut < b.z_cut_mm) {
                        best = Some(CutterCellHit {
                            z_cut_mm: z_cut,
                            z_tip_mm: z_tip,
                            r_mm: min_r.min(r),
                        });
                    }
                }
            };

            let samples = 6usize;
            for s in 0..=samples {
                consider_t(t_lo + (t_hi - t_lo) * (s as f64 / samples as f64));
            }
            consider_t(t_min_r);

            if let Some(k_cot) = prof.conical_slope_cot_theta() {
                let denom_sq = k_cot * k_cot * l2 - dz * dz;
                if denom_sq > 1e-12 {
                    let d_perp = d_perp_sq.max(0.0).sqrt();
                    let l_xy = l2.sqrt();
                    let t_star = tc - dz.signum() * dz.abs() * d_perp / (l_xy * denom_sq.sqrt());
                    consider_t(t_star);
                }
            }
            best
        }
        SweepMotion::PlanarArcXY {
            start,
            end,
            center_xy,
            radius_mm,
            a0_rad,
            signed_sweep_rad,
            ..
        } => {
            let r = planar_arc_xy_distance(
                px,
                py,
                start,
                end,
                center_xy,
                radius_mm,
                a0_rad,
                signed_sweep_rad,
            );
            let p_dz = prof.tip_to_envelope_dz(r)?;
            Some(CutterCellHit {
                z_cut_mm: start.z + p_dz,
                z_tip_mm: start.z,
                r_mm: r,
            })
        }
    }
}

#[allow(clippy::too_many_arguments)]
#[inline]
fn planar_arc_xy_distance(
    px: f64,
    py: f64,
    start: DVec3,
    end: DVec3,
    center_xy: DVec2,
    radius_mm: f64,
    a0_rad: f64,
    signed_sweep_rad: f64,
) -> f64 {
    let sweep_abs = signed_sweep_rad.abs();
    let dx = px - center_xy.x;
    let dy = py - center_xy.y;
    let dist_c = dx.hypot(dy);

    if sweep_abs >= std::f64::consts::TAU - 1e-6 {
        return (dist_c - radius_mm).abs();
    }
    if dist_c > 1e-9 {
        let ang = dy.atan2(dx);
        let rel = if signed_sweep_rad >= 0.0 {
            (ang - a0_rad).rem_euclid(std::f64::consts::TAU)
        } else {
            (a0_rad - ang).rem_euclid(std::f64::consts::TAU)
        };
        if rel <= sweep_abs {
            return (dist_c - radius_mm).abs();
        }
    }
    let d0 = (px - start.x).hypot(py - start.y);
    let d1 = (px - end.x).hypot(py - end.y);
    d0.min(d1)
}
