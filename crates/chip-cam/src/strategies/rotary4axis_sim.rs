//! Simultaneous 4-Axis Milling (`X + Y + Z + A` / `X + Y + Z + B`) Strategy
//! (`OperationKind::Rotary4AxisSim`).
//!
//! Drives all three linear axes simultaneously with the 4th rotary axis using either a fixed
//! off-centerline lateral shift or smoothed automatic surface-normal tilt (`c_cross(u, θ)`),
//! followed by exact `f64` off-centerline Drop-Cutter contact evaluation in parallel (`rayon`),
//! concentric multi-depth roughing without mid-air unwinding, and gouge-safe chord decimation.

use chip_math::DVec3;
use rayon::prelude::*;

use crate::{
    cl_data::MotionRole,
    params::{Rotary4AxisPattern, Rotary4AxisSimParams, RotaryCutDirection, RotaryTiltMode},
    rotary::{
        ResolvedRotaryConfig, RotaryAxisKind, RotaryDriveMode, build_rotated_strip_mesh,
        resolve_rotary_axial_window,
    },
    strategies::{StrategyExecContext, compute_z_levels},
    surface3d::{
        SurfaceCutterProfile, SurfaceMesh3D, drop_cutter_z_with_scratch, stepover_for_cusp_height,
    },
    toolpath::ToolpathBuilder,
    wcs::CamError,
};

/// Generates a full simultaneous 4-axis (`X + Y + Z + Rotary`) toolpath directly onto `builder`.
pub fn generate_rotary4axis_sim(
    ctx: &StrategyExecContext<'_>,
    params: &Rotary4AxisSimParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    ctx.check_cancel()?;

    let Some(rot) = ctx.setup.rotary else {
        return Err(CamError::InvalidRotarySetup {
            op_name: ctx.op.name.clone(),
            reason: "Simultaneous 4-Axis Milling requires a Setup with 4th-Axis Rotary enabled"
                .to_string(),
        });
    };
    if rot.mode != RotaryDriveMode::Simultaneous4Axis {
        return Err(CamError::InvalidRotarySetup {
            op_name: ctx.op.name.clone(),
            reason: format!(
                "Simultaneous 4-Axis Milling requires Setup Rotary Mode 'Simultaneous 4-Axis (X+Y+Z+Rotary)' (currently '{}')",
                rot.mode.label()
            ),
        });
    }
    if rot.axis == RotaryAxisKind::ZAxis {
        return Err(CamError::InvalidRotarySetup {
            op_name: ctx.op.name.clone(),
            reason: "Simultaneous 4-Axis Milling requires a horizontal rotary axis (A/X or B/Y)"
                .to_string(),
        });
    }

    let base_mesh = SurfaceMesh3D::from_cad_document(ctx.doc, &[], &ctx.setup.frame);
    if base_mesh.triangles.is_empty() {
        return Err(CamError::EmptySelection {
            op_name: ctx.op.name.clone(),
            op_kind: ctx.op.kind.label(),
        });
    }

    let tool_d = ctx.tool.geometry.cutting_diameter_mm().max(0.1);
    let (u_start, u_end) = resolve_rotary_axial_window(
        &rot,
        &base_mesh,
        tool_d,
        params.axial_margin_start.value_mm,
        params.axial_margin_end.value_mm,
        &ctx.op.name,
    )?;

    let start_deg = params.start_angle_deg;
    let end_deg = params.end_angle_deg;
    if !start_deg.is_finite() || !end_deg.is_finite() || end_deg <= start_deg + 1e-3 {
        return Err(CamError::InvalidParameter {
            param: "end_angle_deg",
            value: end_deg,
            reason: "must be finite and greater than start_angle_deg",
        });
    }
    let angular_span_deg = end_deg - start_deg;
    let is_full_turn = (angular_span_deg - 360.0).abs() <= 1e-3;

    // Fine internal Drop-Cutter sampling resolution keeping convex helical flanks gouge-free
    let axial_sample = params.axial_sample_step.value_mm.clamp(0.1, 0.40);
    let n_u = (((u_end - u_start) / axial_sample).ceil() as usize).clamp(4, 2_000);
    let du = (u_end - u_start) / (n_u as f64);
    let u_samples: Vec<f64> = (0..=n_u).map(|i| u_start + (i as f64) * du).collect();

    let d_theta_req = params.angular_sample_deg.clamp(0.5, 2.0);
    let n_theta = ((angular_span_deg / d_theta_req).ceil() as usize).clamp(8, 720);
    let d_theta = angular_span_deg / (n_theta as f64);
    let theta_count = if is_full_turn { n_theta } else { n_theta + 1 };

    let cutter = SurfaceCutterProfile::from_tool_geometry(
        &ctx.tool.geometry,
        ctx.passes.stock_to_leave_radial.value_mm,
        ctx.passes.stock_to_leave_axial.value_mm,
    );
    let r_max = cutter.max_radius_mm();
    let r_floor = (ctx.heights.bottom_z_wcs - rot.center_wcs.z).max(0.5);
    let z_floor = rot.center_wcs.z + r_floor;
    let r_outer_est = rot.stock_outer_radius_mm.max(2.0);
    let max_safe_cross = (r_outer_est * 0.65).max(0.5);

    // Stage A: Compute smoothed off-centerline cross-axis offset field `c_cross[it][iu]`
    let c_cross_grid = compute_cross_offset_field(
        &base_mesh,
        &rot,
        &cutter,
        params,
        &u_samples,
        start_deg,
        d_theta,
        theta_count,
        is_full_turn,
        u_start,
        u_end,
        r_max,
        r_floor,
        z_floor,
        max_safe_cross,
    );
    ctx.check_cancel()?;

    let mut max_abs_cross = 0.0_f64;
    for row in &c_cross_grid {
        for &c in row {
            max_abs_cross = max_abs_cross.max(c.abs());
        }
    }

    // Stage B: Exact f64 off-centerline Drop-Cutter evaluation in parallel across angular stations
    let z_slices: Vec<Vec<f64>> = (0..theta_count)
        .into_par_iter()
        .map(|it| {
            let theta_deg = start_deg + (it as f64) * d_theta;
            let rot_mesh = build_rotated_strip_mesh(
                &base_mesh,
                &rot,
                theta_deg,
                u_start,
                u_end,
                max_abs_cross + r_max,
                z_floor,
            );
            let mut scratch = Vec::with_capacity(64);
            let mut row = Vec::with_capacity(u_samples.len());
            for (iu, &u_val) in u_samples.iter().enumerate() {
                let c_off = c_cross_grid[it][iu];
                let (qx, qy) = match rot.axis {
                    RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => {
                        (u_val, rot.center_wcs.y + c_off)
                    }
                    RotaryAxisKind::YAxis => (rot.center_wcs.x + c_off, u_val),
                };
                let z_hit = if rot_mesh.triangles.is_empty() {
                    None
                } else {
                    drop_cutter_z_with_scratch(&rot_mesh, &cutter, qx, qy, z_floor, &mut scratch)
                };
                row.push((z_hit.unwrap_or(z_floor) - rot.center_wcs.z).max(r_floor));
            }
            row
        })
        .collect();

    ctx.check_cancel()?;

    let mut r_min_surf = f64::INFINITY;
    let mut r_max_surf = 0.0_f64;
    for (it, row) in z_slices.iter().enumerate() {
        for (iu, &z_rel) in row.iter().enumerate() {
            let r_eff = z_rel.hypot(c_cross_grid[it][iu]);
            r_min_surf = r_min_surf.min(r_eff);
            r_max_surf = r_max_surf.max(r_eff);
        }
    }
    if !r_min_surf.is_finite() {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    let grid = Rotary4AxisGrid {
        u_start,
        du,
        n_u,
        start_deg,
        d_theta,
        n_theta,
        is_full_turn,
        z_rel_slices: z_slices,
        c_cross_slices: c_cross_grid,
    };

    let stepover_mm = if params.use_cusp_height {
        let cusp = params.cusp_height.value_mm.clamp(0.002, 2.0);
        stepover_for_cusp_height(&cutter, cusp, params.stepover.value_mm.max(0.05))
            .clamp(0.1, tool_d * 0.75)
    } else {
        params.stepover.value_mm.max(0.05)
    };

    let r_outer = rot.stock_outer_radius_mm.max(r_max_surf);
    let radial_levels = compute_z_levels(
        r_outer,
        r_min_surf,
        ctx.passes.multiple_depths,
        ctx.passes.max_stepdown.value_mm,
        ctx.passes.even_stepdowns,
    )?;

    let safe_z_wcs = rot.center_wcs.z + rot.clearance_radius_mm.max(r_outer + 5.0);
    let cut_feed = ctx.cutting.cutting_feed_mm_min;
    let plunge_feed = ctx.cutting.plunge_feed_mm_min.min(cut_feed).max(10.0);
    let smooth_tol = params.smoothing_tolerance.value_mm.clamp(0.0, 0.25);

    match params.pattern {
        Rotary4AxisPattern::SimultaneousSpiral => {
            emit_simultaneous_spiral(
                ctx,
                &rot,
                &grid,
                &radial_levels,
                u_start,
                u_end,
                start_deg,
                end_deg,
                is_full_turn,
                stepover_mm,
                d_theta,
                params.cut_direction,
                safe_z_wcs,
                cut_feed,
                plunge_feed,
                smooth_tol,
                builder,
            )?;
        }
        Rotary4AxisPattern::HelicalFlowline => {
            emit_helical_flowline(
                ctx,
                &rot,
                &grid,
                &radial_levels,
                &u_samples,
                start_deg,
                end_deg,
                is_full_turn,
                r_outer,
                stepover_mm,
                params.cut_direction,
                safe_z_wcs,
                cut_feed,
                plunge_feed,
                smooth_tol,
                builder,
            )?;
        }
    }

    if let Some(last) = builder.current_pos()
        && last.z < safe_z_wcs - 1e-4
    {
        builder.rotary_rapid_to(
            DVec3::new(last.x, last.y, safe_z_wcs),
            builder.current_rotary_deg(),
        );
    }

    Ok(())
}

/// Computes the smooth cross-axis lateral offset grid `c_cross[it][iu]` (`mm`).
#[allow(clippy::too_many_arguments)]
fn compute_cross_offset_field(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    params: &Rotary4AxisSimParams,
    u_samples: &[f64],
    start_deg: f64,
    d_theta: f64,
    theta_count: usize,
    is_full_turn: bool,
    u_start: f64,
    u_end: f64,
    r_max: f64,
    r_floor: f64,
    z_floor: f64,
    max_safe_cross: f64,
) -> Vec<Vec<f64>> {
    let nu = u_samples.len();
    match params.tilt_mode {
        RotaryTiltMode::FixedLateralOffset => {
            let c_fixed = params
                .lateral_offset
                .value_mm
                .clamp(-max_safe_cross, max_safe_cross);
            vec![vec![c_fixed; nu]; theta_count]
        }
        RotaryTiltMode::SurfaceNormalAutoTilt => {
            let max_tilt_rad = params.max_tilt_deg.clamp(0.0, 40.0).to_radians();
            if max_tilt_rad <= 1e-4 {
                return vec![vec![0.0; nu]; theta_count];
            }

            // Coarse centerline drop-cutter pass to estimate local circumferential flank slope `dr/(r*dθ)`
            let r_center: Vec<Vec<f64>> = (0..theta_count)
                .into_par_iter()
                .map(|it| {
                    let theta_deg = start_deg + (it as f64) * d_theta;
                    let rot_mesh = build_rotated_strip_mesh(
                        base_mesh, rot, theta_deg, u_start, u_end, r_max, z_floor,
                    );
                    let mut scratch = Vec::with_capacity(64);
                    let mut row = Vec::with_capacity(nu);
                    for &u_val in u_samples {
                        let (qx, qy) = match rot.axis {
                            RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => {
                                (u_val, rot.center_wcs.y)
                            }
                            RotaryAxisKind::YAxis => (rot.center_wcs.x, u_val),
                        };
                        let z_hit = if rot_mesh.triangles.is_empty() {
                            None
                        } else {
                            drop_cutter_z_with_scratch(
                                &rot_mesh,
                                cutter,
                                qx,
                                qy,
                                z_floor,
                                &mut scratch,
                            )
                        };
                        row.push((z_hit.unwrap_or(z_floor) - rot.center_wcs.z).max(r_floor));
                    }
                    row
                })
                .collect();

            // Convert circumferential slope into bounded cross-axis offset aligning cutter with flank normal
            let d_th_rad = d_theta.to_radians().max(1e-4);
            let axis_cross_sign = match rot.axis {
                // XAxis: e2 = -Y -> +theta moves toward -Y (for CCW); positive dr/dθ faces -theta (+Y)
                RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => rot.direction.sign(),
                // YAxis: e2 = +X -> +theta moves toward +X (for CCW); positive dr/dθ faces -theta (-X)
                RotaryAxisKind::YAxis => -rot.direction.sign(),
            };

            let mut raw = vec![vec![0.0_f64; nu]; theta_count];
            for it in 0..theta_count {
                let it_prev = if is_full_turn {
                    (it + theta_count - 1) % theta_count
                } else {
                    it.saturating_sub(1)
                };
                let it_next = if is_full_turn {
                    (it + 1) % theta_count
                } else {
                    (it + 1).min(theta_count - 1)
                };
                let span_rad = if !is_full_turn && (it == 0 || it + 1 == theta_count) {
                    d_th_rad
                } else {
                    2.0 * d_th_rad
                };
                for iu in 0..nu {
                    let r_here = r_center[it][iu].max(1.0);
                    let dr_dtheta = (r_center[it_next][iu] - r_center[it_prev][iu]) / span_rad;
                    let slope_angle = (dr_dtheta / r_here).atan();
                    let clamped_alpha = slope_angle.clamp(-max_tilt_rad, max_tilt_rad);
                    let c_val = (axis_cross_sign * r_here * clamped_alpha.sin())
                        .clamp(-max_safe_cross, max_safe_cross);
                    raw[it][iu] = c_val;
                }
            }

            // Smooth cross-axis tilt field so simultaneous Y/X & rotary motion stays C1-smooth
            let smooth_passes = params.tilt_smoothing_window.clamp(0, 12) as usize;
            let mut cur = raw;
            let mut next = vec![vec![0.0_f64; nu]; theta_count];
            for _ in 0..smooth_passes {
                for it in 0..theta_count {
                    let itp = if is_full_turn {
                        (it + theta_count - 1) % theta_count
                    } else {
                        it.saturating_sub(1)
                    };
                    let itn = if is_full_turn {
                        (it + 1) % theta_count
                    } else {
                        (it + 1).min(theta_count - 1)
                    };
                    for iu in 0..nu {
                        let iup = iu.saturating_sub(1);
                        let iun = (iu + 1).min(nu - 1);
                        // 2D separable-style 5-point smoothing kernel
                        let val = 0.40 * cur[it][iu]
                            + 0.20 * cur[itp][iu]
                            + 0.20 * cur[itn][iu]
                            + 0.10 * cur[it][iup]
                            + 0.10 * cur[it][iun];
                        next[it][iu] = val.clamp(-max_safe_cross, max_safe_cross);
                    }
                }
                std::mem::swap(&mut cur, &mut next);
            }
            cur
        }
    }
}

struct Rotary4AxisGrid {
    u_start: f64,
    du: f64,
    n_u: usize,
    start_deg: f64,
    d_theta: f64,
    n_theta: usize,
    is_full_turn: bool,
    z_rel_slices: Vec<Vec<f64>>,
    c_cross_slices: Vec<Vec<f64>>,
}

impl Rotary4AxisGrid {
    /// Returns `(c_cross_mm, z_rel_mm)` at `(u, theta_deg)` with shoulder-safe Z lifting across
    /// sharp steps and smooth bilinear cross-axis interpolation.
    fn sample_safe(&self, u: f64, theta_deg: f64) -> (f64, f64) {
        let fu = ((u - self.u_start) / self.du).clamp(0.0, self.n_u as f64);
        let iu0 = (fu.floor() as usize).min(self.n_u);
        let iu1 = (iu0 + 1).min(self.n_u);
        let tu = (fu - iu0 as f64).clamp(0.0, 1.0);

        let (it0, it1, tt) = if self.is_full_turn {
            let wrapped = (theta_deg - self.start_deg).rem_euclid(360.0);
            let ft = wrapped / self.d_theta;
            let i0 = (ft.floor() as usize) % self.n_theta;
            let i1 = (i0 + 1) % self.n_theta;
            (i0, i1, (ft - ft.floor()).clamp(0.0, 1.0))
        } else {
            let max_idx = self.z_rel_slices.len().saturating_sub(1);
            let ft = ((theta_deg - self.start_deg) / self.d_theta).clamp(0.0, max_idx as f64);
            let i0 = (ft.floor() as usize).min(max_idx);
            let i1 = (i0 + 1).min(max_idx);
            (i0, i1, (ft - i0 as f64).clamp(0.0, 1.0))
        };

        let c00 = self.c_cross_slices[it0][iu0];
        let c01 = self.c_cross_slices[it0][iu1];
        let c10 = self.c_cross_slices[it1][iu0];
        let c11 = self.c_cross_slices[it1][iu1];
        let c0 = c00 + (c01 - c00) * tu;
        let c1 = c10 + (c11 - c10) * tu;
        let c_bi = c0 + (c1 - c0) * tt;

        let z00 = self.z_rel_slices[it0][iu0];
        let z01 = self.z_rel_slices[it0][iu1];
        let z10 = self.z_rel_slices[it1][iu0];
        let z11 = self.z_rel_slices[it1][iu1];
        let z0 = z00 + (z01 - z00) * tu;
        let z1 = z10 + (z11 - z10) * tu;
        let z_bi = z0 + (z1 - z0) * tt;
        let z_max = z00.max(z01).max(z10).max(z11);

        let c_min = c00.min(c01).min(c10).min(c11);
        let c_max = c00.max(c01).max(c10).max(c11);
        let c_spread = c_max - c_min;
        let z_safe = if (z_max - z_bi) > 0.008 || c_spread > 0.10 {
            z_max + 0.06 * c_spread
        } else {
            z_bi
        };
        (c_bi, z_safe)
    }

    /// Evaluates `(c_cross, z_rel)` clamped to concentric cylindrical roughing shell `r_ceil`.
    #[inline]
    fn sample_at_shell(&self, u: f64, theta_deg: f64, r_ceil: f64) -> (f64, f64) {
        let (c, z_surf) = self.sample_safe(u, theta_deg);
        let z_shell = (r_ceil * r_ceil - c * c).max(0.25).sqrt();
        (c, z_surf.max(z_shell))
    }
}

#[inline]
fn off_center_wcs(rot: &ResolvedRotaryConfig, u_axial: f64, c_cross: f64, z_rel: f64) -> DVec3 {
    let z_wcs = rot.center_wcs.z + z_rel;
    match rot.axis {
        RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => {
            DVec3::new(u_axial, rot.center_wcs.y + c_cross, z_wcs)
        }
        RotaryAxisKind::YAxis => DVec3::new(rot.center_wcs.x + c_cross, u_axial, z_wcs),
    }
}

#[allow(clippy::too_many_arguments)]
fn emit_simultaneous_spiral(
    ctx: &StrategyExecContext<'_>,
    rot: &ResolvedRotaryConfig,
    grid: &Rotary4AxisGrid,
    radial_levels: &[f64],
    u_start: f64,
    u_end: f64,
    start_deg: f64,
    end_deg: f64,
    is_full_turn: bool,
    stepover_mm: f64,
    d_theta: f64,
    cut_direction: RotaryCutDirection,
    safe_z_wcs: f64,
    cut_feed: f64,
    plunge_feed: f64,
    smooth_tol: f64,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    let n_levels = radial_levels.len();
    let mut continuous_theta = start_deg;

    for (li, &r_ceil) in radial_levels.iter().enumerate() {
        ctx.check_cancel()?;
        let role = if li + 1 == n_levels {
            MotionRole::FinishCut
        } else {
            MotionRole::Cut
        };

        if is_full_turn {
            let n_turns = ((u_end - u_start) / stepover_mm).ceil().max(1.0);
            let steps_per_rev = ((360.0 / d_theta).ceil() as usize).clamp(24, 360);
            let rev_step_deg = 360.0 / (steps_per_rev as f64);
            let level_start_deg = continuous_theta;

            let mut raw_pts: Vec<(f64, f64, f64, f64)> =
                Vec::with_capacity((n_turns as usize + 3) * steps_per_rev);

            // 1. Start 360° cleanup ring at u_start
            for k in 0..=steps_per_rev {
                let th = level_start_deg + (k as f64) * rev_step_deg;
                let (c, z) = grid.sample_at_shell(u_start, th, r_ceil);
                raw_pts.push((u_start, th, c, z));
            }

            // 2. Continuous simultaneous 4-axis helix
            let helix_steps =
                ((n_turns * (steps_per_rev as f64)).round() as usize).max(steps_per_rev);
            let th_helix_start = level_start_deg + 360.0;
            let th_helix_span = n_turns * 360.0;
            for k in 1..=helix_steps {
                let t = (k as f64) / (helix_steps as f64);
                let u = u_start + t * (u_end - u_start);
                let th = th_helix_start + t * th_helix_span;
                let (c, z) = grid.sample_at_shell(u, th, r_ceil);
                raw_pts.push((u, th, c, z));
            }

            // 3. End 360° cleanup ring at u_end
            let th_end_start = th_helix_start + th_helix_span;
            for k in 1..=steps_per_rev {
                let th = th_end_start + (k as f64) * rev_step_deg;
                let (c, z) = grid.sample_at_shell(u_end, th, r_ceil);
                raw_pts.push((u_end, th, c, z));
            }
            continuous_theta = th_end_start + 360.0;

            let pts = decimate_4axis_strip(&raw_pts, smooth_tol);
            if let Some(&(u0, th0, c0, z0)) = pts.first() {
                let p0 = off_center_wcs(rot, u0, c0, z0);
                builder.rotary_rapid_to(DVec3::new(p0.x, p0.y, safe_z_wcs), th0);
                builder.rotary_linear_to(
                    p0,
                    th0,
                    plunge_feed,
                    c0.hypot(z0).max(1.0),
                    MotionRole::Plunge,
                );
                for &(u, th, c, z) in pts.iter().skip(1) {
                    builder.rotary_linear_to(
                        off_center_wcs(rot, u, c, z),
                        th,
                        cut_feed,
                        c.hypot(z).max(1.0),
                        role,
                    );
                }
                if let Some(&(ul, thl, cl, zl)) = pts.last() {
                    let pl = off_center_wcs(rot, ul, cl, zl);
                    builder.rotary_rapid_to(DVec3::new(pl.x, pl.y, safe_z_wcs), thl);
                }
                builder.pass_count += 1;
            }
        } else {
            let n_lanes = (((u_end - u_start) / stepover_mm).ceil() as usize).max(1);
            let lane_du = (u_end - u_start) / (n_lanes as f64);
            let n_ang = (((end_deg - start_deg) / d_theta).ceil() as usize).max(4);
            let ang_step = (end_deg - start_deg) / (n_ang as f64);

            for lane in 0..=n_lanes {
                ctx.check_cancel()?;
                let u = u_start + (lane as f64) * lane_du;
                let reverse = match cut_direction {
                    RotaryCutDirection::ZigZag => lane % 2 == 1,
                    RotaryCutDirection::Climb => false,
                    RotaryCutDirection::Conventional => true,
                };
                let mut raw_pts = Vec::with_capacity(n_ang + 1);
                for k in 0..=n_ang {
                    let kk = if reverse { n_ang - k } else { k };
                    let th = start_deg + (kk as f64) * ang_step;
                    let (c, z) = grid.sample_at_shell(u, th, r_ceil);
                    raw_pts.push((u, th, c, z));
                }
                let pts = decimate_4axis_strip(&raw_pts, smooth_tol);
                if let Some(&(u0, th0, c0, z0)) = pts.first() {
                    let p0 = off_center_wcs(rot, u0, c0, z0);
                    let need_retract = lane == 0 || cut_direction != RotaryCutDirection::ZigZag;
                    if need_retract {
                        builder.rotary_rapid_to(DVec3::new(p0.x, p0.y, safe_z_wcs), th0);
                        builder.rotary_linear_to(
                            p0,
                            th0,
                            plunge_feed,
                            c0.hypot(z0).max(1.0),
                            MotionRole::Plunge,
                        );
                    } else {
                        builder.rotary_linear_to(
                            p0,
                            th0,
                            cut_feed,
                            c0.hypot(z0).max(1.0),
                            MotionRole::LinkTransition,
                        );
                    }
                    for &(uu, th, c, z) in pts.iter().skip(1) {
                        builder.rotary_linear_to(
                            off_center_wcs(rot, uu, c, z),
                            th,
                            cut_feed,
                            c.hypot(z).max(1.0),
                            role,
                        );
                    }
                    if cut_direction != RotaryCutDirection::ZigZag
                        && let Some(&(ul, thl, cl, zl)) = pts.last()
                    {
                        let pl = off_center_wcs(rot, ul, cl, zl);
                        builder.rotary_rapid_to(DVec3::new(pl.x, pl.y, safe_z_wcs), thl);
                    }
                    builder.pass_count += 1;
                }
            }
            if let Some(last) = builder.current_pos()
                && last.z < safe_z_wcs - 1e-4
            {
                builder.rotary_rapid_to(
                    DVec3::new(last.x, last.y, safe_z_wcs),
                    builder.current_rotary_deg(),
                );
            }
        }
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn emit_helical_flowline(
    ctx: &StrategyExecContext<'_>,
    rot: &ResolvedRotaryConfig,
    grid: &Rotary4AxisGrid,
    radial_levels: &[f64],
    u_samples: &[f64],
    start_deg: f64,
    end_deg: f64,
    is_full_turn: bool,
    r_outer: f64,
    stepover_mm: f64,
    cut_direction: RotaryCutDirection,
    safe_z_wcs: f64,
    cut_feed: f64,
    plunge_feed: f64,
    smooth_tol: f64,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    let angular_span = end_deg - start_deg;
    let delta_deg = (stepover_mm / r_outer.max(2.0))
        .to_degrees()
        .clamp(0.5, 45.0);
    let n_lanes = ((angular_span / delta_deg).ceil() as usize).max(4);
    let lane_step_deg = angular_span / (n_lanes as f64);
    let lane_count = if is_full_turn { n_lanes } else { n_lanes + 1 };
    let n_levels = radial_levels.len();

    for (li, &r_ceil) in radial_levels.iter().enumerate() {
        ctx.check_cancel()?;
        let role = if li + 1 == n_levels {
            MotionRole::FinishCut
        } else {
            MotionRole::Cut
        };

        for lane in 0..lane_count {
            ctx.check_cancel()?;
            let th = start_deg + (lane as f64) * lane_step_deg;
            let reverse = match cut_direction {
                RotaryCutDirection::ZigZag => lane % 2 == 1,
                RotaryCutDirection::Climb => false,
                RotaryCutDirection::Conventional => true,
            };

            let mut raw_pts = Vec::with_capacity(u_samples.len());
            if reverse {
                for &u in u_samples.iter().rev() {
                    let (c, z) = grid.sample_at_shell(u, th, r_ceil);
                    raw_pts.push((u, th, c, z));
                }
            } else {
                for &u in u_samples {
                    let (c, z) = grid.sample_at_shell(u, th, r_ceil);
                    raw_pts.push((u, th, c, z));
                }
            }

            let pts = decimate_4axis_strip(&raw_pts, smooth_tol);
            if let Some(&(u0, th0, c0, z0)) = pts.first() {
                let p0 = off_center_wcs(rot, u0, c0, z0);
                if lane == 0 || cut_direction != RotaryCutDirection::ZigZag {
                    builder.rotary_rapid_to(DVec3::new(p0.x, p0.y, safe_z_wcs), th0);
                    builder.rotary_linear_to(
                        p0,
                        th0,
                        plunge_feed,
                        c0.hypot(z0).max(1.0),
                        MotionRole::Plunge,
                    );
                } else {
                    let prev_th = start_deg + ((lane - 1) as f64) * lane_step_deg;
                    let link_steps = ((lane_step_deg / 2.0).ceil() as usize).clamp(2, 16);
                    for s in 1..=link_steps {
                        let t = (s as f64) / (link_steps as f64);
                        let th_mid = prev_th + t * (th0 - prev_th);
                        let (c_mid, z_mid) = grid.sample_at_shell(u0, th_mid, r_ceil);
                        builder.rotary_linear_to(
                            off_center_wcs(rot, u0, c_mid, z_mid),
                            th_mid,
                            cut_feed,
                            c_mid.hypot(z_mid).max(1.0),
                            MotionRole::LinkTransition,
                        );
                    }
                }

                for &(u, theta, c, z) in pts.iter().skip(1) {
                    builder.rotary_linear_to(
                        off_center_wcs(rot, u, c, z),
                        theta,
                        cut_feed,
                        c.hypot(z).max(1.0),
                        role,
                    );
                }

                if cut_direction != RotaryCutDirection::ZigZag
                    && let Some(&(ul, thl, cl, zl)) = pts.last()
                {
                    let pl = off_center_wcs(rot, ul, cl, zl);
                    builder.rotary_rapid_to(DVec3::new(pl.x, pl.y, safe_z_wcs), thl);
                }
                builder.pass_count += 1;
            }
        }

        if let Some(last) = builder.current_pos()
            && last.z < safe_z_wcs - 1e-4
        {
            builder.rotary_rapid_to(
                DVec3::new(last.x, last.y, safe_z_wcs),
                builder.current_rotary_deg(),
            );
        }
    }

    Ok(())
}

/// Conservative 4-axis chord decimation in `(u, theta_deg, c_cross, z_rel)` space:
/// never dips > 3 µm below convex ridges in `z`, preserves cross-axis `c_cross` accuracy within
/// `tol_mm`, and keeps angular spans `<= 2.5°` on curved contours.
fn decimate_4axis_strip(pts: &[(f64, f64, f64, f64)], tol_mm: f64) -> Vec<(f64, f64, f64, f64)> {
    if pts.len() <= 2 || tol_mm <= 1e-7 {
        return pts.to_vec();
    }
    let mut out = Vec::with_capacity(pts.len());
    out.push(pts[0]);
    let mut anchor_idx = 0usize;

    for i in 1..(pts.len() - 1) {
        let a = pts[anchor_idx];
        let b = pts[i + 1];
        let max_ang = if (b.3 - a.3).abs() > 1e-3 || (b.2 - a.2).abs() > 1e-3 || a.2.abs() > 0.5 {
            1.8
        } else {
            6.0
        };
        if (b.1 - a.1).abs() > max_ang {
            out.push(pts[i]);
            anchor_idx = i;
            continue;
        }
        let du = b.0 - a.0;
        let dth = b.1 - a.1;
        let denom = du.hypot(dth);
        let mut exceeds = false;
        for &(pu, pth, pc, pz) in &pts[(anchor_idx + 1)..=i] {
            let t = if denom > 1e-12 {
                (((pu - a.0) * du + (pth - a.1) * dth) / (denom * denom)).clamp(0.0, 1.0)
            } else {
                0.5
            };
            let c_interp = a.2 + t * (b.2 - a.2);
            let z_interp = a.3 + t * (b.3 - a.3);
            if (pz - z_interp) > 0.003 || (z_interp - pz) > tol_mm || (pc - c_interp).abs() > tol_mm
            {
                exceeds = true;
                break;
            }
        }
        if exceeds {
            out.push(pts[i]);
            anchor_idx = i;
        }
    }
    out.push(pts[pts.len() - 1]);
    out
}
