//! Polar / Cylindrical Heightmap Stock Simulation (`(u_axial, θ) -> r_max`) and 5-Rule 4th-Axis
//! Rotary Verification (`chip-sim` Milestone 19).
//!
//! Supports rotary axes aligned along `X (A)`, `Y (B)`, or `Z (C)`, both cylindrical bar stock
//! and chucked rectangular/square bar stock, analytical sweeping of all 6 cutter types across
//! centerline-locked and off-centerline 4-axis motion, polar target-model gouge checking against
//! STEP B-Rep solids or STL/OBJ/3MF meshes, chuck/holder collision checks, and watertight
//! color-by-op cylindrical mesh tessellation.

use std::{f64::consts::TAU, sync::atomic::Ordering};

use chip_cad::CadDocument;
use chip_cam::{
    ResolvedRotaryConfig, ResolvedSetup, ResolvedStockGeometry, RotaryAxisKind, RotaryDirection,
    SegmentMotion3D, SurfaceMesh3D, ToolpathSegment3D, rotate_point_about_axis,
};
use chip_math::{Aabb3, DVec3, Ray3};
use chip_tools::CutterGeometry;
use rayon::prelude::*;

use crate::{
    cutter_profile::{RadialCutterProfile, SimColoredVertex},
    heightmap::{
        HeightfieldStock, OPERATION_STOCK_PALETTE, SetupSimulationInput, SetupSimulationResult,
        SimError, SimResolution, SimStockMesh, StockPlaybackCursor, UNCUT_OP_IDX,
        slice_segment_range, target_cursor_from_snap,
    },
    timeline::{SimOperationInput, SimulationTimeline},
    verifier::{SimDiagnostic, SimIssueKind, SimSeverity, SimVerificationConfig},
};

/// Uncut raw metallic aluminum color (sRGB RGBA) matching 3-axis heightmap stock.
const UNCUT_STOCK_RGBA: [f32; 4] = [0.64, 0.67, 0.72, 1.0];

/// Polar / cylindrical heightmap stock parameterized over `(u_axial, θ) -> r_stock(u, θ)`.
#[derive(Debug, Clone)]
pub struct CylindricalStock {
    pub rotary: ResolvedRotaryConfig,
    pub u_min: f64,
    pub u_max: f64,
    pub du: f64,
    pub dtheta: f64,
    pub nu: usize,
    pub ntheta: usize,
    /// Current machined outer radius (`mm`) from rotary centerline at cell `(iu, itheta)`.
    pub r_stock: Vec<f32>,
    /// Initial raw stock outer radius (`mm`) at cell `(iu, itheta)`.
    pub r_init: Vec<f32>,
    /// Polar-rasterized target CAD/mesh outermost radius (`mm`), or `-1.0` where no solid exists.
    pub r_target: Vec<f32>,
    /// Operation index that last machined cell `(iu, itheta)`, or [`UNCUT_OP_IDX`].
    pub last_op_idx: Vec<u16>,
}

/// Snapshot of [`CylindricalStock`] after completing `completed_segments` motion spans.
#[derive(Debug, Clone)]
pub struct CylindricalCheckpoint {
    pub completed_segments: usize,
    pub r_stock: Vec<f32>,
    pub last_op_idx: Vec<u16>,
}

impl CylindricalStock {
    /// Initializes a polar [`CylindricalStock`] from `setup.stock` and `rotary`.
    pub fn from_setup(
        setup: &ResolvedSetup,
        rotary: ResolvedRotaryConfig,
        resolution: SimResolution,
    ) -> Result<Self, SimError> {
        let u_min = rotary.stock_u_min;
        let u_max = rotary.stock_u_max;
        let length = u_max - u_min;
        if !length.is_finite() || length <= 1e-3 || rotary.stock_outer_radius_mm <= 1e-3 {
            return Err(SimError::InvalidStockBounds);
        }

        let (nu, ntheta) = match resolution {
            SimResolution::Standard => (256usize, 360usize),
            SimResolution::Fine => (400usize, 540usize),
            SimResolution::Ultra => (512usize, 720usize),
        };
        let du = length / nu as f64;
        let dtheta = TAU / ntheta as f64;
        let total = nu * ntheta;

        let mut r_init = vec![0.0_f32; total];
        for iu in 0..nu {
            let u = u_min + (iu as f64 + 0.5) * du;
            for it in 0..ntheta {
                let theta = it as f64 * dtheta;
                let r = eval_initial_stock_radius(&setup.stock, &rotary, u, theta);
                r_init[iu * ntheta + it] = r.max(0.0) as f32;
            }
        }

        let r_stock = r_init.clone();
        let r_target = vec![-1.0_f32; total];
        let last_op_idx = vec![UNCUT_OP_IDX; total];

        Ok(Self {
            rotary,
            u_min,
            u_max,
            du,
            dtheta,
            nu,
            ntheta,
            r_stock,
            r_init,
            r_target,
            last_op_idx,
        })
    }

    /// Computes total cylindrical volume (`mm³`) of the current `r_stock` polar field:
    /// `V = ∑ 0.5 * r(u, θ)² * dθ * du`.
    pub fn current_volume_mm3(&self) -> f64 {
        let factor = 0.5 * self.du * self.dtheta;
        self.r_stock
            .iter()
            .map(|&r| {
                let rf = f64::from(r.max(0.0));
                rf * rf * factor
            })
            .sum()
    }

    /// Computes initial raw stock volume (`mm³`) from `r_init`.
    pub fn initial_volume_mm3(&self) -> f64 {
        let factor = 0.5 * self.du * self.dtheta;
        self.r_init
            .iter()
            .map(|&r| {
                let rf = f64::from(r.max(0.0));
                rf * rf * factor
            })
            .sum()
    }

    /// Rasterizes the target CAD/mesh solid (`doc`) into `self.r_target` via parallel inward raycasts.
    pub fn rasterize_target_surface(&mut self, doc: &CadDocument, setup: &ResolvedSetup) {
        if !doc.has_3d_geometry() {
            return;
        }
        let surface = SurfaceMesh3D::from_cad_document(doc, &[], &setup.frame);
        if surface.triangles.is_empty() {
            return;
        }
        let r_cast = self.rotary.stock_outer_radius_mm + 5.0;
        let nu = self.nu;
        let ntheta = self.ntheta;
        let u_min = self.u_min;
        let du = self.du;
        let dtheta = self.dtheta;
        let rotary = self.rotary;

        let target_rows: Vec<Vec<f32>> = (0..nu)
            .into_par_iter()
            .map(|iu| {
                let u = u_min + (iu as f64 + 0.5) * du;
                let mut row = vec![-1.0_f32; ntheta];
                for (it, cell_r) in row.iter_mut().enumerate() {
                    let theta = it as f64 * dtheta;
                    let p_outer = rotary.cylindrical_to_wcs(u, theta, r_cast);
                    let p_axis = rotary.cylindrical_to_wcs(u, theta, 0.0);
                    let dir = (p_axis - p_outer).normalize_or_zero();
                    if let Some(ray) = Ray3::new(p_outer, dir)
                        && let Some(hit) =
                            surface
                                .bvh
                                .raycast(&surface.vertices_wcs, &surface.triangles, &ray)
                        && hit.ray_t <= r_cast
                    {
                        let r_hit = (r_cast - hit.ray_t).max(0.0);
                        *cell_r = r_hit as f32;
                    }
                }
                row
            })
            .collect();

        for (iu, row) in target_rows.into_iter().enumerate() {
            let start = iu * ntheta;
            self.r_target[start..start + ntheta].copy_from_slice(&row);
        }
    }

    /// Applies a single [`ToolpathSegment3D`] sweep to this polar stock and records any 5-rule violations.
    #[allow(clippy::too_many_arguments)]
    pub fn sweep_segment(
        &mut self,
        seg: &ToolpathSegment3D,
        profile: &RadialCutterProfile,
        op_idx: usize,
        op: &SimOperationInput<'_>,
        global_seg_idx: usize,
        seg_start_time_sec: f64,
        verify_cfg: &SimVerificationConfig,
        diagnostics: &mut Vec<SimDiagnostic>,
    ) {
        let is_rapid = matches!(seg.motion, SegmentMotion3D::Rapid);
        let lin_len = seg.length_mm();
        let d_deg = (seg.end_rotary_deg - seg.start_rotary_deg).abs();
        let step_u = (0.5 * self.du).max(0.15);
        let step_deg = (0.5 * self.dtheta.to_degrees()).max(0.75);
        let n_lin = (lin_len / step_u).ceil() as usize;
        let n_rot = (d_deg / step_deg).ceil() as usize;
        let n_steps = n_lin.max(n_rot).clamp(1, 2_000);

        let mut worst_rapid: Option<(f64, DVec3)> = None;
        let mut worst_gouge: Option<(f64, DVec3)> = None;
        let mut worst_flute: Option<(f64, DVec3)> = None;
        let mut worst_holder: Option<(f64, DVec3)> = None;

        for k in 0..=n_steps {
            let t = k as f64 / n_steps as f64;
            let tip_wcs = seg.sample_point(t);
            let rot_deg = seg.sample_rotary_deg(t);

            self.sweep_pose_at(
                tip_wcs,
                rot_deg,
                profile,
                op_idx,
                is_rapid,
                op.intentional_edge_break_mm,
                verify_cfg,
                &mut worst_rapid,
                &mut worst_gouge,
                &mut worst_flute,
                &mut worst_holder,
            );
        }

        // Emit at most one diagnostic per issue kind per segment (matching 3-axis heightmap verifier)
        if let Some((excess, loc)) = worst_rapid {
            diagnostics.push(SimDiagnostic {
                severity: SimSeverity::CriticalError,
                kind: SimIssueKind::RapidIntoStock,
                op_idx,
                op_id: op.op_id.to_string(),
                op_name: op.op_name.to_string(),
                tool_number: op.tool.tool_number,
                global_seg_idx,
                timeline_time_sec: seg_start_time_sec,
                location_wcs: loc,
                max_excess_mm: excess,
                message: format!(
                    "G0 rapid collided with rotary stock by {excess:.3} mm at ({:.2}, {:.2}, {:.2}).",
                    loc.x, loc.y, loc.z
                ),
            });
        }
        if let Some((excess, loc)) = worst_gouge {
            diagnostics.push(SimDiagnostic {
                severity: SimSeverity::CriticalError,
                kind: SimIssueKind::TargetModelGouge,
                op_idx,
                op_id: op.op_id.to_string(),
                op_name: op.op_name.to_string(),
                tool_number: op.tool.tool_number,
                global_seg_idx,
                timeline_time_sec: seg_start_time_sec,
                location_wcs: loc,
                max_excess_mm: excess,
                message: format!(
                    "Cutter gouged rotary target model by {excess:.3} mm at ({:.2}, {:.2}, {:.2}).",
                    loc.x, loc.y, loc.z
                ),
            });
        }
        if let Some((excess, loc)) = worst_holder {
            diagnostics.push(SimDiagnostic {
                severity: SimSeverity::CriticalError,
                kind: SimIssueKind::HolderCollision,
                op_idx,
                op_id: op.op_id.to_string(),
                op_name: op.op_name.to_string(),
                tool_number: op.tool.tool_number,
                global_seg_idx,
                timeline_time_sec: seg_start_time_sec,
                location_wcs: loc,
                max_excess_mm: excess,
                message: format!(
                    "Tool holder / chuck collision by {excess:.3} mm at ({:.2}, {:.2}, {:.2}).",
                    loc.x, loc.y, loc.z
                ),
            });
        }
        if let Some((excess, loc)) = worst_flute {
            diagnostics.push(SimDiagnostic {
                severity: SimSeverity::Warning,
                kind: SimIssueKind::FluteLengthExceeded,
                op_idx,
                op_id: op.op_id.to_string(),
                op_name: op.op_name.to_string(),
                tool_number: op.tool.tool_number,
                global_seg_idx,
                timeline_time_sec: seg_start_time_sec,
                location_wcs: loc,
                max_excess_mm: excess,
                message: format!(
                    "Cutting depth exceeded flute length by {excess:.3} mm at ({:.2}, {:.2}, {:.2}).",
                    loc.x, loc.y, loc.z
                ),
            });
        }
    }

    /// Fast mutation-only polar sweep used when reconstructing intermediate timeline scrub
    /// positions. Returns `true` if at least one cell's outer radius decreased.
    pub fn sweep_segment_mutate_only(
        &mut self,
        seg: &ToolpathSegment3D,
        profile: &RadialCutterProfile,
        op_idx: usize,
    ) -> bool {
        let lin_len = seg.length_mm();
        let d_deg = (seg.end_rotary_deg - seg.start_rotary_deg).abs();
        let step_u = (0.5 * self.du).max(0.15);
        let step_deg = (0.5 * self.dtheta.to_degrees()).max(0.75);
        let n_lin = (lin_len / step_u).ceil() as usize;
        let n_rot = (d_deg / step_deg).ceil() as usize;
        let n_steps = n_lin.max(n_rot).clamp(1, 2_000);

        let mut mutated = false;
        for k in 0..=n_steps {
            let t = k as f64 / n_steps as f64;
            let tip_wcs = seg.sample_point(t);
            let rot_deg = seg.sample_rotary_deg(t);
            mutated |= self.sweep_pose_inner(tip_wcs, rot_deg, profile, op_idx, None);
        }
        mutated
    }

    #[allow(clippy::too_many_arguments)]
    fn sweep_pose_at(
        &mut self,
        tip_wcs: DVec3,
        rot_deg: f64,
        profile: &RadialCutterProfile,
        op_idx: usize,
        is_rapid: bool,
        edge_break_mm: f64,
        verify_cfg: &SimVerificationConfig,
        worst_rapid: &mut Option<(f64, DVec3)>,
        worst_gouge: &mut Option<(f64, DVec3)>,
        worst_flute: &mut Option<(f64, DVec3)>,
        worst_holder: &mut Option<(f64, DVec3)>,
    ) {
        let mut verify = PoseVerifyCtx {
            is_rapid,
            edge_break_mm,
            verify_cfg,
            worst_rapid,
            worst_gouge,
            worst_flute,
            worst_holder,
        };
        let _ = self.sweep_pose_inner(tip_wcs, rot_deg, profile, op_idx, Some(&mut verify));
    }

    fn sweep_pose_inner(
        &mut self,
        tip_wcs: DVec3,
        rot_deg: f64,
        profile: &RadialCutterProfile,
        op_idx: usize,
        mut verify: Option<&mut PoseVerifyCtx<'_>>,
    ) -> bool {
        // Transform cutter tip and spindle up-axis (+Z_wcs) into workpiece-fixed coordinates (`-rot_deg`)
        let tip_part = self.rotary.rotate_point_wcs(tip_wcs, -rot_deg);
        let spindle_part =
            rotate_vector_about_axis(DVec3::Z, self.rotary.axis, -rot_deg, self.rotary.direction);

        let r_cut = profile.cutting_radius_mm.max(0.05);
        let r_holder = profile.holder_radius_mm.max(r_cut);
        let stickout = profile.stickout_length_mm.max(profile.flute_length_mm);
        let u_vec = self.rotary.axis.axis_unit();
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
                update_worst(v.worst_holder, excess, tip_wcs);
            }
        }

        // Fast reject if tool tip is completely outside stock radial envelope + tool radius
        let check_holder = verify.is_some();
        let max_possible_r =
            self.rotary.stock_outer_radius_mm + if check_holder { r_holder } else { r_cut } + 2.0;
        let tip_h_above_axis = (tip_part - self.rotary.center_wcs).dot(spindle_part);
        if tip_h_above_axis > max_possible_r {
            return false;
        }

        // Only expand the radial/axial cell search window to `r_holder` when verifying and the
        // holder bottom actually descends into the stock's outer radial envelope.
        let holder_can_reach_stock = check_holder
            && (tip_h_above_axis + stickout) <= self.rotary.stock_outer_radius_mm + 1.0;
        let r_footprint = if holder_can_reach_stock {
            r_holder
        } else {
            r_cut
        };

        // Axial cell range `[iu_min..=iu_max]`
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

        // Angular wedge range around `theta_tip` (full circle if cutter approaches centerline,
        // and expanded for off-centerline simultaneous 4-axis poses where the spindle axis sweeps
        // angularly from `tip_part` up through the stock outer envelope).
        let (e1, e2, _) = self.rotary.axis.basis();
        let pol = self.rotary.direction.sign();
        let rel_tip = tip_part - self.rotary.center_wcs;
        let radial_tip = rel_tip - u_vec * rel_tip.dot(u_vec);
        let c_off_center = radial_tip.cross(spindle_part).length();
        let full_circle = r_tip <= r_footprint + 1.0 || self.rotary.axis == RotaryAxisKind::ZAxis;
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

        let op_tag = op_idx.min(u16::MAX as usize - 1) as u16;
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

                // Radial distance squared from spindle axis along ray `P(r) = axis_pt + r * d_rad`:
                // `rho^2(r) = a*r^2 + 2*b*r + c`
                let a = (1.0 - d_dot_s * d_dot_s).max(0.0);
                let b = v0_dot_d - h0 * d_dot_s;
                let c = (v0_sq - h0 * h0).max(0.0);

                // Holder collision check against outer holder cylinder `rho <= r_holder` above `stickout`
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
                            update_worst(v.worst_holder, excess, tip_wcs);
                        }
                    }
                }

                // Cutter envelope intersection (`rho <= r_cut`)
                let Some((cr_lo, cr_hi)) = solve_cylinder_ray_interval(a, b, c, r_cut) else {
                    continue;
                };
                if cr_hi <= 0.0 || cr_lo >= r_cur {
                    continue;
                }

                // Only engage the ray side facing the spindle (or traversing across the spindle)
                let r_enter = r_cur.min(cr_hi);
                let r_exit = cr_lo.max(0.0);
                if r_enter <= r_exit + 1e-6 {
                    continue;
                }

                let eval_clearance = |r: f64| -> f64 {
                    let h = h0 + r * d_dot_s;
                    let rho = (a * r * r + 2.0 * b * r + c).max(0.0).sqrt();
                    let env_dz = profile.tip_to_envelope_dz(rho).unwrap_or(f64::INFINITY);
                    h - env_dz // >= 0 means point is INSIDE the cutter body
                };

                let c_enter = eval_clearance(r_enter);
                if c_enter < 0.0 {
                    // Even the outermost point on the stock ray is below the cutter tip
                    continue;
                }

                // Check flute-length engagement on outer uncut stock surface
                if let Some(v) = verify.as_deref_mut() {
                    let h_enter = h0 + r_enter * d_dot_s;
                    if h_enter > profile.flute_length_mm + 0.05 {
                        update_worst(v.worst_flute, h_enter - profile.flute_length_mm, tip_wcs);
                    }
                }

                // Find exact lowest radius `r_new` where `eval_clearance(r) == 0` via fast bisection
                let c_exit = eval_clearance(r_exit);
                let r_new = if c_exit >= 0.0 {
                    r_exit
                } else {
                    let mut lo = r_exit;
                    let mut hi = r_enter;
                    for _ in 0..10 {
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
                        if v.is_rapid && cut_depth > v.verify_cfg.rapid_crash_tol_mm {
                            update_worst(v.worst_rapid, cut_depth, tip_wcs);
                        }
                        let gouge_tol = v.verify_cfg.gouge_tol_mm + v.edge_break_mm;
                        let r_tgt = f64::from(self.r_target[idx]);
                        if r_tgt >= 0.0 && r_new < r_tgt - gouge_tol {
                            update_worst(v.worst_gouge, r_tgt - r_new, tip_wcs);
                        }
                    }

                    self.r_stock[idx] = r_new.max(0.0) as f32;
                    self.last_op_idx[idx] = op_tag;
                    mutated = true;
                }
            }
        }
        mutated
    }
}

struct PoseVerifyCtx<'a> {
    is_rapid: bool,
    edge_break_mm: f64,
    verify_cfg: &'a SimVerificationConfig,
    worst_rapid: &'a mut Option<(f64, DVec3)>,
    worst_gouge: &'a mut Option<(f64, DVec3)>,
    worst_flute: &'a mut Option<(f64, DVec3)>,
    worst_holder: &'a mut Option<(f64, DVec3)>,
}

impl SetupSimulationResult {
    /// Reconstructs the exact 4th-axis [`CylindricalStock`] state at `time_sec` by restoring the
    /// nearest preceding [`CylindricalCheckpoint`] and sweeping the remaining segments up to
    /// `time_sec` (including fractional intra-segment motion).
    pub fn reconstruct_cylindrical_stock_at_time(&self, time_sec: f64) -> Option<CylindricalStock> {
        let mut stock = self.final_cylindrical_stock.as_ref()?.clone();
        let mut cursor = None;
        self.sync_cylindrical_stock_to_time(&mut stock, &mut cursor, time_sec);
        Some(stock)
    }

    /// Synchronizes a reusable working [`CylindricalStock`] to the exact timeline state at
    /// `time_sec`, advancing incrementally when moving forward and restoring from the nearest
    /// [`CylindricalCheckpoint`] when scrubbing backward or jumping past a checkpoint.
    ///
    /// Returns `true` if `stock` changed (and any display mesh should be re-tessellated).
    pub fn sync_cylindrical_stock_to_time(
        &self,
        stock: &mut CylindricalStock,
        cursor: &mut Option<StockPlaybackCursor>,
        time_sec: f64,
    ) -> bool {
        let Some(cyl_final) = &self.final_cylindrical_stock else {
            return false;
        };
        let total_spans = self.timeline.motion_spans.len();
        let target = self.timeline.sample_at_time(time_sec).map_or(
            StockPlaybackCursor {
                seg_idx: total_spans,
                seg_u: 0.0,
            },
            |s| target_cursor_from_snap(&s, total_spans),
        );

        if let Some(cur) = *cursor
            && cur.seg_idx == target.seg_idx
            && (target.seg_u - cur.seg_u).abs() <= 1e-6
        {
            return false;
        }

        if target.seg_idx >= total_spans {
            stock.r_stock.clone_from(&cyl_final.r_stock);
            stock.last_op_idx.clone_from(&cyl_final.last_op_idx);
            *cursor = Some(StockPlaybackCursor {
                seg_idx: total_spans,
                seg_u: 0.0,
            });
            return true;
        }

        let cp = self
            .cylindrical_checkpoints
            .iter()
            .rfind(|c| c.completed_segments <= target.seg_idx);

        let (mut cur_seg, cur_u, mut changed) = if let Some(cur) = *cursor
            && (target.seg_idx > cur.seg_idx
                || (target.seg_idx == cur.seg_idx && target.seg_u >= cur.seg_u))
        {
            if let Some(cp) = cp
                && cp.completed_segments > cur.seg_idx
            {
                stock.r_stock.clone_from(&cp.r_stock);
                stock.last_op_idx.clone_from(&cp.last_op_idx);
                (cp.completed_segments, 0.0, true)
            } else {
                (cur.seg_idx, cur.seg_u, false)
            }
        } else if let Some(cp) = cp {
            stock.r_stock.clone_from(&cp.r_stock);
            stock.last_op_idx.clone_from(&cp.last_op_idx);
            (cp.completed_segments, 0.0, true)
        } else {
            stock.r_stock.clone_from(&cyl_final.r_stock);
            stock.last_op_idx.clone_from(&cyl_final.last_op_idx);
            (0, 0.0, true)
        };

        let profiles = &self.operation_profiles;
        if cur_seg == target.seg_idx {
            if let Some(span) = self.timeline.motion_spans.get(cur_seg)
                && let Some(&(prof, _)) = profiles.get(span.op_idx)
                && let Some(sub_seg) = slice_segment_range(&span.segment, cur_u, target.seg_u)
            {
                changed |= stock.sweep_segment_mutate_only(&sub_seg, &prof, span.op_idx);
            }
        } else {
            if cur_u > 1e-6 {
                if let Some(span) = self.timeline.motion_spans.get(cur_seg)
                    && let Some(&(prof, _)) = profiles.get(span.op_idx)
                    && let Some(sub_seg) = slice_segment_range(&span.segment, cur_u, 1.0)
                {
                    changed |= stock.sweep_segment_mutate_only(&sub_seg, &prof, span.op_idx);
                }
                cur_seg += 1;
            }

            for span in &self.timeline.motion_spans[cur_seg..target.seg_idx] {
                if let Some(&(prof, _)) = profiles.get(span.op_idx) {
                    changed |= stock.sweep_segment_mutate_only(&span.segment, &prof, span.op_idx);
                }
            }

            if target.seg_u > 1e-6
                && let Some(span) = self.timeline.motion_spans.get(target.seg_idx)
                && let Some(&(prof, _)) = profiles.get(span.op_idx)
                && let Some(sub_seg) = slice_segment_range(&span.segment, 0.0, target.seg_u)
            {
                changed |= stock.sweep_segment_mutate_only(&sub_seg, &prof, span.op_idx);
            }
        }

        *cursor = Some(target);
        changed
    }
}

/// Executes full polar/cylindrical 4th-axis setup simulation with checkpoints and 5-rule verification.
pub fn run_cylindrical_setup_simulation(
    input: &SetupSimulationInput<'_>,
    rotary: ResolvedRotaryConfig,
) -> Result<SetupSimulationResult, SimError> {
    let mut stock = CylindricalStock::from_setup(input.setup, rotary, input.resolution)?;
    stock.rasterize_target_surface(input.doc, input.setup);
    let initial_vol = stock.initial_volume_mm3();

    let timeline = SimulationTimeline::from_operations(input.operations, input.kinematics);
    let total_spans = timeline.motion_spans.len();
    let checkpoint_stride = (total_spans / 24).max(16);

    let mut checkpoints = vec![CylindricalCheckpoint {
        completed_segments: 0,
        r_stock: stock.r_stock.clone(),
        last_op_idx: stock.last_op_idx.clone(),
    }];
    let mut diagnostics = Vec::new();

    let profiles: Vec<(RadialCutterProfile, bool)> = input
        .operations
        .iter()
        .map(|op| {
            (
                RadialCutterProfile::from_tool(op.tool),
                matches!(op.tool.geometry, CutterGeometry::FlatEndmill { .. }),
            )
        })
        .collect();

    for (span_idx, span) in timeline.motion_spans.iter().enumerate() {
        if input.cancel.is_some_and(|c| c.load(Ordering::Relaxed)) {
            return Err(SimError::Cancelled);
        }
        let op = &input.operations[span.op_idx];
        stock.sweep_segment(
            &span.segment,
            &profiles[span.op_idx].0,
            span.op_idx,
            op,
            span.global_seg_idx,
            span.t_start_sec,
            &input.verify_config,
            &mut diagnostics,
        );

        let completed = span_idx + 1;
        if completed.is_multiple_of(checkpoint_stride) || completed == total_spans {
            checkpoints.push(CylindricalCheckpoint {
                completed_segments: completed,
                r_stock: stock.r_stock.clone(),
                last_op_idx: stock.last_op_idx.clone(),
            });
        }
    }

    let remaining_vol = stock.current_volume_mm3();
    let removed_vol = (initial_vol - remaining_vol).max(0.0);

    // Minimal placeholder 3-axis heightfield so existing callers inspecting `final_stock` never panic
    let placeholder_hf =
        HeightfieldStock::from_setup_stock(&input.setup.stock, SimResolution::Standard)
            .unwrap_or_else(|_| {
                let aabb = Aabb3::new(DVec3::splat(-10.0), DVec3::splat(10.0));
                HeightfieldStock::from_setup_stock(
                    &ResolvedStockGeometry::Box { aabb_wcs: aabb },
                    SimResolution::Standard,
                )
                .unwrap_or_else(|_| unreachable!())
            });

    Ok(SetupSimulationResult {
        resolution: input.resolution,
        timeline,
        initial_stock_volume_mm3: initial_vol,
        remaining_stock_volume_mm3: remaining_vol,
        removed_volume_mm3: removed_vol,
        final_stock: placeholder_hf,
        final_cylindrical_stock: Some(stock),
        checkpoints: Vec::new(),
        cylindrical_checkpoints: checkpoints,
        operation_profiles: profiles,
        diagnostics,
    })
}

/// Tessellates a [`CylindricalStock`] into a smooth, watertight 3D [`SimStockMesh`] (including
/// both end caps) optionally rotated live around the rotary axis by `rotary_angle_deg` for
/// interactive 4th-axis playback.
pub fn tessellate_cylindrical_stock_mesh(
    stock: &CylindricalStock,
    color_by_op: bool,
    rotary_angle_deg: f64,
) -> SimStockMesh {
    let nu = stock.nu;
    let nt = stock.ntheta;
    if nu < 2 || nt < 3 {
        return SimStockMesh::default();
    }

    // Downsample slightly for ultra-responsive 60 FPS scrubbing while keeping smooth fidelity
    let step_u = (nu / 180).max(1);
    let step_t = (nt / 180).max(1);
    let su = nu.div_ceil(step_u);
    let st = nt.div_ceil(step_t);

    let sample_r = |iu: usize, it: usize| -> f64 {
        let cu = (iu * step_u).min(nu - 1);
        let ct = (it * step_t) % nt;
        f64::from(stock.r_stock[cu * nt + ct].max(0.0))
    };
    let sample_color = |iu: usize, it: usize| -> [f32; 4] {
        if !color_by_op {
            return UNCUT_STOCK_RGBA;
        }
        let cu = (iu * step_u).min(nu - 1);
        let ct = (it * step_t) % nt;
        let op_idx = stock.last_op_idx[cu * nt + ct];
        if op_idx == UNCUT_OP_IDX {
            UNCUT_STOCK_RGBA
        } else {
            OPERATION_STOCK_PALETTE[usize::from(op_idx) % OPERATION_STOCK_PALETTE.len()]
        }
    };

    let mut positions: Vec<DVec3> = Vec::with_capacity(su * st);
    let mut colors: Vec<[f32; 4]> = Vec::with_capacity(su * st);

    for iu in 0..su {
        let u = if iu + 1 == su {
            stock.u_max
        } else if iu == 0 {
            stock.u_min
        } else {
            stock.u_min + ((iu * step_u) as f64 + 0.5) * stock.du
        };
        for it in 0..st {
            let theta = (it * step_t) as f64 * stock.dtheta;
            let r = sample_r(iu, it);
            let p_part = stock.rotary.cylindrical_to_wcs(u, theta, r);
            let p_rot = if rotary_angle_deg.abs() > 1e-6 {
                stock.rotary.rotate_point_wcs(p_part, rotary_angle_deg)
            } else {
                p_part
            };
            positions.push(p_rot);
            colors.push(sample_color(iu, it));
        }
    }

    // Build lateral skin triangles & accumulate area-weighted smooth vertex normals
    let mut normals = vec![DVec3::ZERO; su * st];
    let mut indices: Vec<u32> = Vec::with_capacity((su - 1) * st * 6 + st * 6);

    for iu in 0..(su - 1) {
        for it in 0..st {
            let it_next = (it + 1) % st;
            let i00 = iu * st + it;
            let i10 = (iu + 1) * st + it;
            let i11 = (iu + 1) * st + it_next;
            let i01 = iu * st + it_next;

            let p00 = positions[i00];
            let p10 = positions[i10];
            let p11 = positions[i11];
            let p01 = positions[i01];

            let n1 = (p10 - p00).cross(p11 - p00);
            let n2 = (p11 - p00).cross(p01 - p00);
            normals[i00] += n1 + n2;
            normals[i10] += n1;
            normals[i11] += n1 + n2;
            normals[i01] += n2;

            indices.extend_from_slice(&[
                i00 as u32, i10 as u32, i11 as u32, i00 as u32, i11 as u32, i01 as u32,
            ]);
        }
    }

    let mut vertices: Vec<SimColoredVertex> = Vec::with_capacity(su * st + 2 * (st + 1));
    for i in 0..(su * st) {
        let p = positions[i];
        let n = normals[i].normalize_or_zero();
        vertices.push(SimColoredVertex {
            pos_mm: [p.x as f32, p.y as f32, p.z as f32],
            normal: [n.x as f32, n.y as f32, n.z as f32],
            color_rgba: colors[i],
        });
    }

    // Watertight end caps at `u_min` (normal `-u_axis`) and `u_max` (normal `+u_axis`)
    let u_vec = stock.rotary.axis.axis_unit();
    for (is_max_end, ring_iu, cap_u, cap_normal) in [
        (false, 0usize, stock.u_min, -u_vec),
        (true, su - 1, stock.u_max, u_vec),
    ] {
        let c_part = stock.rotary.cylindrical_to_wcs(cap_u, 0.0, 0.0);
        let c_rot = if rotary_angle_deg.abs() > 1e-6 {
            stock.rotary.rotate_point_wcs(c_part, rotary_angle_deg)
        } else {
            c_part
        };
        let center_idx = vertices.len() as u32;
        vertices.push(SimColoredVertex {
            pos_mm: [c_rot.x as f32, c_rot.y as f32, c_rot.z as f32],
            normal: [
                cap_normal.x as f32,
                cap_normal.y as f32,
                cap_normal.z as f32,
            ],
            color_rgba: UNCUT_STOCK_RGBA,
        });

        let ring_base = vertices.len() as u32;
        for it in 0..st {
            let p = positions[ring_iu * st + it];
            vertices.push(SimColoredVertex {
                pos_mm: [p.x as f32, p.y as f32, p.z as f32],
                normal: [
                    cap_normal.x as f32,
                    cap_normal.y as f32,
                    cap_normal.z as f32,
                ],
                color_rgba: UNCUT_STOCK_RGBA,
            });
        }
        for it in 0..st {
            let a = ring_base + it as u32;
            let b = ring_base + ((it + 1) % st) as u32;
            if is_max_end {
                indices.extend_from_slice(&[center_idx, b, a]);
            } else {
                indices.extend_from_slice(&[center_idx, a, b]);
            }
        }
    }

    SimStockMesh { vertices, indices }
}

/// Evaluates the exact raw stock outer radius (`mm`) from `rotary.center_wcs` at `(u, theta)`.
fn eval_initial_stock_radius(
    stock: &ResolvedStockGeometry,
    rotary: &ResolvedRotaryConfig,
    u: f64,
    theta: f64,
) -> f64 {
    let (e1, e2, u_vec) = rotary.axis.basis();
    let pol = rotary.direction.sign();
    let d_rad = e1 * theta.cos() + e2 * (theta.sin() * pol);
    let u_center = rotary.center_wcs.dot(u_vec);
    let origin = rotary.center_wcs + u_vec * (u - u_center);

    match *stock {
        ResolvedStockGeometry::Cylinder {
            axis,
            center_wcs,
            radius_mm,
            min_axial_mm,
            max_axial_mm,
            ..
        } if axis == rotary.axis.to_stock_axis() => {
            if u < min_axial_mm - 1e-4 || u > max_axial_mm + 1e-4 {
                return 0.0;
            }
            // Ray-circle intersection in the perpendicular plane (handles centered & eccentric bars)
            let rel = origin - center_wcs;
            let perp_o = rel - u_vec * rel.dot(u_vec);
            let b = perp_o.dot(d_rad);
            let c = perp_o.length_squared() - radius_mm * radius_mm;
            let disc = b * b - c;
            if disc < 0.0 {
                0.0
            } else {
                (-b + disc.sqrt()).max(0.0)
            }
        }
        ResolvedStockGeometry::Box { aabb_wcs }
        | ResolvedStockGeometry::Cylinder { aabb_wcs, .. } => {
            ray_exit_aabb_distance(origin, d_rad, &aabb_wcs).unwrap_or(0.0)
        }
    }
}

/// Computes the exit distance `t_max >= 0` of ray `origin + t * dir` leaving `aabb`.
fn ray_exit_aabb_distance(origin: DVec3, dir: DVec3, aabb: &Aabb3) -> Option<f64> {
    let mut t_min = f64::NEG_INFINITY;
    let mut t_max = f64::INFINITY;
    for axis in 0..3 {
        let (o, d, bmin, bmax) = match axis {
            0 => (origin.x, dir.x, aabb.min.x, aabb.max.x),
            1 => (origin.y, dir.y, aabb.min.y, aabb.max.y),
            _ => (origin.z, dir.z, aabb.min.z, aabb.max.z),
        };
        if d.abs() < 1e-12 {
            if o < bmin - 1e-6 || o > bmax + 1e-6 {
                return None;
            }
        } else {
            let inv = 1.0 / d;
            let mut t0 = (bmin - o) * inv;
            let mut t1 = (bmax - o) * inv;
            if t0 > t1 {
                std::mem::swap(&mut t0, &mut t1);
            }
            t_min = t_min.max(t0);
            t_max = t_max.min(t1);
            if t_min > t_max + 1e-9 {
                return None;
            }
        }
    }
    if t_max >= 0.0 { Some(t_max) } else { None }
}

/// Solves `a*r^2 + 2*b*r + (c - R^2) <= 0` for the ray intersection interval `[r_lo, r_hi]`.
#[inline]
fn solve_cylinder_ray_interval(a: f64, b: f64, c: f64, radius: f64) -> Option<(f64, f64)> {
    let c_eff = c - radius * radius;
    if a <= 1e-12 {
        // Ray is parallel to spindle axis
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

#[inline]
fn rotate_vector_about_axis(
    vec: DVec3,
    axis: RotaryAxisKind,
    angle_deg: f64,
    direction: RotaryDirection,
) -> DVec3 {
    rotate_point_about_axis(vec, DVec3::ZERO, axis, angle_deg, direction).normalize_or_zero()
}

#[inline]
fn update_worst(slot: &mut Option<(f64, DVec3)>, excess: f64, loc: DVec3) {
    if slot.is_none_or(|(prev, _)| excess > prev) {
        *slot = Some((excess, loc));
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::f64::consts::PI;

    use chip_cad::{
        CadDocument, MeshImportOptions, load_mesh_cad_document, sample_rotary_shaft_stl_bytes,
    };
    use chip_cam::{
        AxisAnchor, BoxAnchor27, MotionRole, RotaryAxisKind, RotaryDriveMode, Setup, StockAxis,
        StockConfig, ToolpathBuilder, WcsOriginMode, WorkOffsetSlot,
    };
    use chip_math::{DisplayUnit, ParamLength, approx_eq};
    use chip_tools::default_starter_tool_library;

    use super::*;
    use crate::{MachineKinematicsConfig, SimVerificationConfig, run_setup_simulation};

    #[test]
    fn cylindrical_stock_turning_volume_and_chucked_square_bar_are_exact() {
        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];

        // 1. Round bar Ø44 × 80 mm along X (A-axis), turn a 20 mm wide groove from R=22 down to R=18 mm!
        //    Exact removed volume = pi * (22^2 - 18^2) * (20 + 6) ... with Ø6mm flat endmill moving
        //    from X=-10 to X=+10, total swept groove width = 26 mm -> V = pi * (484 - 324) * 26 = 13,069 mm³!
        let mut setup = Setup::new("s_rot", "Rotary A", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27 {
                x: AxisAnchor::Center,
                y: AxisAnchor::Center,
                z: AxisAnchor::Center,
            },
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(44.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
        setup.rotary.axis = RotaryAxisKind::XAxis;

        let doc = CadDocument::empty();
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        assert!(resolved.rotary.is_some());

        let mut b = ToolpathBuilder::new("op_turn");
        b.rapid_to(DVec3::new(-10.0, 0.0, 30.0));
        b.linear_to(DVec3::new(-10.0, 0.0, 18.0), 400.0, MotionRole::Plunge);
        // Full 360° circular ring at left end X = -10 mm
        b.rotary_linear_to(
            DVec3::new(-10.0, 0.0, 18.0),
            360.0,
            800.0,
            18.0,
            MotionRole::Cut,
        );
        // Continuous helical spiral turning from X=-10 to X=+10 over 10 revolutions (2 mm/rev pitch < 6 mm tool!)
        let n_turns = 10usize;
        let steps = n_turns * 36;
        for s in 1..=steps {
            let frac = s as f64 / steps as f64;
            let x = -10.0 + 20.0 * frac;
            let deg = 360.0 + 360.0 * n_turns as f64 * frac;
            b.rotary_linear_to(DVec3::new(x, 0.0, 18.0), deg, 800.0, 18.0, MotionRole::Cut);
        }
        // Finish circular cleanup at right end X = +10 mm
        b.rotary_linear_to(
            DVec3::new(10.0, 0.0, 18.0),
            360.0 * (n_turns as f64 + 2.0),
            800.0,
            18.0,
            MotionRole::FinishCut,
        );
        b.rapid_to(DVec3::new(10.0, 0.0, 30.0));
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            operations: &[SimOperationInput {
                op_id: "op_turn",
                op_name: "Rotary Turn Groove",
                tool: flat_6mm,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            cancel: None,
        })
        .unwrap();

        assert_eq!(
            sim.critical_error_count(),
            0,
            "Clean rotary turning groove should have 0 crashes, got: {:#?}",
            sim.diagnostics
        );
        let exact_v = PI * (22.0_f64.powi(2) - 18.0_f64.powi(2)) * 26.0;
        let err_pct = ((sim.removed_volume_mm3 - exact_v).abs() / exact_v) * 100.0;
        assert!(
            err_pct < 3.0,
            "Removed volume {:.1} mm³ should match analytical {:.1} mm³ within 3% (err={err_pct:.2}%)",
            sim.removed_volume_mm3,
            exact_v
        );

        // Tessellated watertight cylindrical stock mesh builds cleanly at any rotary angle
        let cyl_stock = sim.final_cylindrical_stock.as_ref().unwrap();
        let mesh = tessellate_cylindrical_stock_mesh(cyl_stock, true, 45.0);
        assert!(mesh.vertices.len() > 1_000);
        assert_eq!(mesh.indices.len() % 3, 0);

        // 2. Square 100 × 100 × 120 mm bar chucked in rotary 4-jaw chuck: volume == 1,200,000 mm³
        let mut sq_setup = Setup::new("s_sq", "Square Bar", WorkOffsetSlot::G54);
        sq_setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27 {
                x: AxisAnchor::Center,
                y: AxisAnchor::Center,
                z: AxisAnchor::Center,
            },
        };
        sq_setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(120.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(100.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::CenterZ,
        };
        sq_setup.rotary.mode = RotaryDriveMode::Indexed3Plus1;
        sq_setup.rotary.axis = RotaryAxisKind::XAxis;
        let sq_res = sq_setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let sq_cyl =
            CylindricalStock::from_setup(&sq_res, sq_res.rotary.unwrap(), SimResolution::Standard)
                .unwrap();
        assert!(
            approx_eq(sq_cyl.initial_volume_mm3(), 1_200_000.0, 2_000.0),
            "Square bar polar volume should be ~1,200,000 mm³, got {:.1}",
            sq_cyl.initial_volume_mm3()
        );
    }

    #[test]
    fn fault_injection_catches_rotary_rapid_crash_target_gouge_and_chuck_collision() {
        let stl = sample_rotary_shaft_stl_bytes();
        let doc = load_mesh_cad_document("rotary_shaft.stl", &stl, &MeshImportOptions::default())
            .unwrap();
        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_shaft", "Rotary Shaft", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27 {
                x: AxisAnchor::Center,
                y: AxisAnchor::Center,
                z: AxisAnchor::Center,
            },
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(44.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup.rotary.axis = RotaryAxisKind::XAxis;
        setup.rotary.chuck_length = ParamLength::from_mm(20.0);
        setup.rotary.chuck_diameter = ParamLength::from_mm(65.0);
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        // Inject 3 distinct faults:
        // (1) Rapid plunge straight into Ø44 bar at X=0, Z=17
        // (2) Target model gouge cutting Z=10 into the R=14 mm central groove (`X=0`)
        // (3) Chuck collision moving into X=-48, Z=25 (inside chuck `X ∈ [-60, -40]`, `R=32.5`)
        let mut b = ToolpathBuilder::new("op_fault");
        b.rapid_to(DVec3::new(0.0, 0.0, 35.0));
        b.rapid_to(DVec3::new(0.0, 0.0, 17.0)); // Fault 1: RapidIntoStock
        b.linear_to(DVec3::new(0.0, 0.0, 10.0), 400.0, MotionRole::Cut); // Fault 2: TargetModelGouge
        b.rapid_to(DVec3::new(0.0, 0.0, 35.0));
        b.rapid_to(DVec3::new(-48.0, 0.0, 25.0)); // Fault 3: Chuck HolderCollision
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            operations: &[SimOperationInput {
                op_id: "op_fault",
                op_name: "Fault Injection",
                tool: flat_6mm,
                fallback_spindle_rpm: 10_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            cancel: None,
        })
        .unwrap();

        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::RapidIntoStock),
            "Must detect rotary RapidIntoStock, got: {:#?}",
            sim.diagnostics
        );
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::TargetModelGouge),
            "Must detect rotary TargetModelGouge on shaft STL, got: {:#?}",
            sim.diagnostics
        );
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::HolderCollision),
            "Must detect rotary chuck HolderCollision, got: {:#?}",
            sim.diagnostics
        );
    }

    #[test]
    fn off_centerline_simultaneous_4axis_fault_injection_catches_flank_gouge_and_holder_crash() {
        let stl = chip_cad::sample_helical_flute_shaft_stl_bytes();
        let doc = load_mesh_cad_document(
            "helical_flute_shaft.stl",
            &stl,
            &MeshImportOptions::default(),
        )
        .unwrap();
        assert!(doc.meshes[0].diagnostics.is_watertight);
        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_helical", "Helical Shaft", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(42.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup.rotary.axis = RotaryAxisKind::XAxis;
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        // Off-centerline simultaneous 4-axis fault injection:
        // (1) Sideways off-centerline cut at Y=+8mm, Z=9mm (gouges helical flute core!)
        // (2) Deep off-centerline plunge at Y=-7mm, Z=-25mm driving tool holder straight into Ø42 bar!
        let mut b = ToolpathBuilder::new("op_4ax_fault");
        b.rotary_rapid_to(DVec3::new(0.0, 8.0, 35.0), 0.0);
        b.rotary_linear_to(DVec3::new(0.0, 8.0, 9.0), 0.0, 400.0, 12.0, MotionRole::Cut);
        b.rotary_linear_to(
            DVec3::new(10.0, 8.0, 9.0),
            45.0,
            400.0,
            12.0,
            MotionRole::Cut,
        );
        b.rotary_rapid_to(DVec3::new(10.0, 8.0, 35.0), 45.0);
        b.rotary_rapid_to(DVec3::new(-32.0, -7.0, 35.0), 45.0);
        b.rotary_linear_to(
            DVec3::new(-32.0, -7.0, -25.0),
            45.0,
            300.0,
            20.0,
            MotionRole::Plunge,
        );
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            operations: &[SimOperationInput {
                op_id: "op_4ax_fault",
                op_name: "Off-Centerline Faults",
                tool: flat_6mm,
                fallback_spindle_rpm: 10_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            cancel: None,
        })
        .unwrap();

        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::TargetModelGouge),
            "Off-centerline 4-axis verifier must catch helical flank gouge, got: {:#?}",
            sim.diagnostics
        );
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::HolderCollision),
            "Off-centerline 4-axis verifier must catch deep holder crash, got: {:#?}",
            sim.diagnostics
        );
    }
}
