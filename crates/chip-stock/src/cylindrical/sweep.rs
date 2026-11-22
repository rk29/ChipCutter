use std::f64::consts::TAU;

use chip_math::DVec3;

use crate::types::{
    Axis3, BatchSweepOutput, RadialCutterProfile, SpanRuleViolations, StockVerifyTolerances,
    SweepChord, SweepToolSpec,
};

use super::CylindricalStock;

struct PoseVerifyCtx<'a> {
    is_rapid: bool,
    verify_tol: StockVerifyTolerances,
    violations: &'a mut SpanRuleViolations,
}

impl CylindricalStock {
    /// Sweeps a batch of [`SweepChord`] primitives across this polar stock, optionally evaluating
    /// 4th-axis rapid, target-gouge, flute-length, and chuck/holder collisions (`verify = true`).
    pub fn sweep_chords(
        &mut self,
        chords: &[SweepChord],
        tools: &[SweepToolSpec],
        verify: bool,
    ) -> BatchSweepOutput {
        let mut out = BatchSweepOutput::default();
        let step_u = (0.5 * self.du).max(0.15);
        let step_deg = (0.5 * self.dtheta.to_degrees()).max(0.75);

        for ch in chords {
            let Some(tool) = tools.get(ch.profile_idx) else {
                continue;
            };
            let lin_len = ch.motion.length_mm();
            let d_deg = ch.motion.rotary_delta_deg();
            let n_lin = (lin_len / step_u).ceil() as usize;
            let n_rot = (d_deg / step_deg).ceil() as usize;
            let n_steps = n_lin.max(n_rot).clamp(1, 2_000);

            for k in 0..=n_steps {
                let t = k as f64 / n_steps as f64;
                let tip_wcs = ch.motion.sample_point(t);
                let rot_deg = ch.motion.sample_rotary_deg(t);

                if verify {
                    let mut vctx = PoseVerifyCtx {
                        is_rapid: ch.is_rapid,
                        verify_tol: ch.verify_tol,
                        violations: &mut out.violations,
                    };
                    out.mutated |= self.sweep_pose_inner(
                        tip_wcs,
                        rot_deg,
                        &tool.profile,
                        ch.op_idx,
                        Some(&mut vctx),
                    );
                } else {
                    out.mutated |=
                        self.sweep_pose_inner(tip_wcs, rot_deg, &tool.profile, ch.op_idx, None);
                }
            }
        }

        out
    }

    fn sweep_pose_inner(
        &mut self,
        tip_wcs: DVec3,
        rot_deg: f64,
        profile: &RadialCutterProfile,
        op_idx: u32,
        mut verify: Option<&mut PoseVerifyCtx<'_>>,
    ) -> bool {
        // Transform cutter tip and spindle up-axis (+Z_wcs) into workpiece-fixed coordinates (`-rot_deg`)
        let tip_part = self.rotary.rotate_point_wcs(tip_wcs, -rot_deg);
        let spindle_part = self.rotary.rotate_vector_wcs(DVec3::Z, -rot_deg);

        let r_cut = profile.cutting_radius_mm.max(0.05);
        let r_holder = profile.holder_radius_mm.max(r_cut);
        let stickout = profile.stickout_length_mm.max(profile.flute_length_mm);
        let u_vec = self.rotary.axis_unit();
        let (u_tip, theta_tip, r_tip) = self.rotary.wcs_to_cylindrical(tip_part);

        // 1. Check direct collision with the rotary chuck keep-out cylinder (verification mode only)
        if let Some(v) = verify.as_deref_mut()
            && self.rotary.chuck_u_max > self.rotary.chuck_u_min + 1e-4
        {
            let chuck_clearance_r = self.rotary.chuck_radius_mm + r_cut;
            let u_overlaps_chuck = (u_tip + r_cut) >= self.rotary.chuck_u_min
                && (u_tip - r_cut) <= self.rotary.chuck_u_max;
            if u_overlaps_chuck && r_tip < chuck_clearance_r - 0.05 {
                let excess = chuck_clearance_r - r_tip;
                v.violations.record_holder(excess, tip_wcs);
            }
        }

        let check_holder = verify.is_some();
        let max_possible_r =
            self.rotary.stock_outer_radius_mm + if check_holder { r_holder } else { r_cut } + 2.0;
        let tip_h_above_axis = (tip_part - self.rotary.center_wcs).dot(spindle_part);
        if tip_h_above_axis > max_possible_r {
            return false;
        }

        let holder_can_reach_stock = check_holder
            && (tip_h_above_axis + stickout) <= self.rotary.stock_outer_radius_mm + 1.0;
        let r_footprint = if holder_can_reach_stock {
            r_holder
        } else {
            r_cut
        };

        let u_reach = r_footprint
            + if holder_can_reach_stock {
                (stickout + 10.0) * spindle_part.dot(u_vec).abs()
            } else {
                profile.flute_length_mm * spindle_part.dot(u_vec).abs()
            }
            + self.du;
        let u_lo = (u_tip - u_reach).max(self.u_min);
        let u_hi = (u_tip + u_reach).min(self.u_max);
        if u_lo > u_hi {
            return false;
        }
        let iu_min = (((u_lo - self.u_min) / self.du).floor() as isize)
            .clamp(0, self.nu as isize - 1) as usize;
        let iu_max = (((u_hi - self.u_min) / self.du).ceil() as isize)
            .clamp(0, self.nu as isize - 1) as usize;

        let (e1, e2, _) = self.rotary.basis();
        let pol = self.rotary.direction_sign;
        let rel_tip = tip_part - self.rotary.center_wcs;
        let radial_tip = rel_tip - u_vec * rel_tip.dot(u_vec);
        let c_off_center = radial_tip.cross(spindle_part).length();
        let full_circle = r_tip <= r_footprint + 1.0 || self.rotary.axis == Axis3::Z;
        let (it_center, it_half_span) = if full_circle {
            (0isize, self.ntheta as isize)
        } else {
            let off_center_extra = (c_off_center / r_tip.max(1.0)).clamp(0.0, 1.0).asin();
            let half_angle =
                ((r_footprint + 1.0) / r_tip).clamp(0.0, 1.0).asin() + off_center_extra + 0.12;
            let center_norm = theta_tip.rem_euclid(TAU);
            let ic = (center_norm / self.dtheta).round() as isize;
            let hs = (half_angle / self.dtheta).ceil() as isize + 1;
            (ic, hs.min((self.ntheta / 2) as isize))
        };

        let u_center = self.rotary.center_wcs.dot(u_vec);
        let it_start = if full_circle {
            0
        } else {
            it_center - it_half_span
        };
        let it_end = if full_circle {
            self.ntheta as isize - 1
        } else {
            it_center + it_half_span
        };

        let mut mutated = false;
        for iu in iu_min..=iu_max {
            let u = self.u_min + (iu as f64 + 0.5) * self.du;
            let axis_pt = self.rotary.center_wcs + u_vec * (u - u_center);
            let v0 = axis_pt - tip_part;
            let h0 = v0.dot(spindle_part);
            let v0_sq = v0.length_squared();

            for it_raw in it_start..=it_end {
                let it = it_raw.rem_euclid(self.ntheta as isize) as usize;
                let idx = iu * self.ntheta + it;
                let r_cur = f64::from(self.r_stock[idx]);
                if r_cur <= 1e-4 {
                    continue;
                }

                let theta = it as f64 * self.dtheta;
                let (sin_t, cos_t) = theta.sin_cos();
                let d_rad = e1 * cos_t + e2 * (sin_t * pol);

                let d_dot_s = d_rad.dot(spindle_part);
                let v0_dot_d = v0.dot(d_rad);

                let a = (1.0 - d_dot_s * d_dot_s).max(0.0);
                let b = v0_dot_d - h0 * d_dot_s;
                let c = (v0_sq - h0 * h0).max(0.0);

                if let Some(v) = verify.as_deref_mut()
                    && let Some((hr_lo, hr_hi)) = solve_cylinder_ray_interval(a, b, c, r_holder)
                    && hr_hi > 0.0
                    && hr_lo < r_cur
                {
                    let r_chk = r_cur.min(hr_hi);
                    let h_chk = h0 + r_chk * d_dot_s;
                    if h_chk > stickout + 0.05 {
                        let excess = (r_cur - hr_lo.max(0.0)).min(h_chk - stickout);
                        if excess > 0.05 {
                            v.violations.record_holder(excess, tip_wcs);
                        }
                    }
                }

                let Some((cr_lo, cr_hi)) = solve_cylinder_ray_interval(a, b, c, r_cut) else {
                    continue;
                };
                if cr_hi <= 0.0 || cr_lo >= r_cur {
                    continue;
                }

                let r_enter = r_cur.min(cr_hi);
                let r_exit = cr_lo.max(0.0);
                if r_enter <= r_exit + 1e-6 {
                    continue;
                }

                let eval_clearance = |r: f64| -> f64 {
                    let h = h0 + r * d_dot_s;
                    let rho = (a * r * r + 2.0 * b * r + c).max(0.0).sqrt();
                    let env_dz = profile.tip_to_envelope_dz(rho).unwrap_or(f64::INFINITY);
                    h - env_dz
                };

                let c_enter = eval_clearance(r_enter);
                if c_enter < 0.0 {
                    continue;
                }

                if let Some(v) = verify.as_deref_mut() {
                    let h_enter = h0 + r_enter * d_dot_s;
                    if h_enter > profile.effective_reach_mm() + 0.05 {
                        v.violations
                            .record_flute(h_enter - profile.effective_reach_mm(), tip_wcs);
                    }
                }

                let c_exit = eval_clearance(r_exit);
                let shape = profile.envelope().shape();
                let is_flat = matches!(shape, chip_tools::CutterEnvelopeShape::Flat { .. });
                let is_ball = matches!(shape, chip_tools::CutterEnvelopeShape::Ball { .. });
                let r_new = if c_exit >= 0.0 {
                    r_exit
                } else if is_flat && d_dot_s > 1e-9 {
                    (-h0 / d_dot_s).clamp(r_exit, r_enter)
                } else {
                    let mut lo = if !is_ball && d_dot_s > 1e-6 {
                        r_exit.max(-h0 / d_dot_s).min(r_enter)
                    } else {
                        r_exit
                    };
                    let mut hi = r_enter;
                    for _ in 0..11 {
                        let mid = 0.5 * (lo + hi);
                        if eval_clearance(mid) >= 0.0 {
                            hi = mid;
                        } else {
                            lo = mid;
                        }
                    }
                    hi
                };

                if r_new < r_cur - 1e-5 {
                    if let Some(v) = verify.as_deref_mut() {
                        let cut_depth = r_cur - r_new;
                        if v.is_rapid && cut_depth > v.verify_tol.rapid_crash_tol_mm {
                            v.violations.record_rapid(cut_depth, tip_wcs);
                        }
                        let r_tgt = f64::from(self.r_target[idx]);
                        if r_tgt >= 0.0 && r_new < r_tgt - v.verify_tol.gouge_tol_mm {
                            v.violations.record_gouge(r_tgt - r_new, tip_wcs);
                        }
                    }

                    self.r_stock[idx] = r_new.max(0.0) as f32;
                    self.last_op_idx[idx] = op_idx;
                    mutated = true;
                }
            }
        }
        mutated
    }
}

/// Solves `a*r^2 + 2*b*r + (c - R^2) <= 0` for the ray intersection interval `[r_lo, r_hi]`.
#[inline]
fn solve_cylinder_ray_interval(a: f64, b: f64, c: f64, radius: f64) -> Option<(f64, f64)> {
    let c_eff = c - radius * radius;
    if a <= 1e-12 {
        if c_eff <= 0.0 { Some((0.0, 1e6)) } else { None }
    } else {
        let disc = b * b - a * c_eff;
        if disc < 0.0 {
            None
        } else {
            let sd = disc.sqrt();
            Some(((-b - sd) / a, (-b + sd) / a))
        }
    }
}
