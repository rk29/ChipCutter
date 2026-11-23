//! Simultaneous 4-Axis Milling (`X + Y + Z + A` / `X + Y + Z + B`) Strategy
//! (`OperationKind::Rotary4AxisSim`).
//!
//! Drives all three linear axes simultaneously with the 4th rotary axis using either a fixed
//! off-centerline lateral shift or smoothed automatic surface-normal tilt (`c_cross(u, θ)`),
//! followed by exact `f64` direct off-centerline Drop-Cutter contact evaluation in parallel
//! (`rayon`), adaptive gouge-free chord-midpoint subdivision, concentric multi-depth roughing
//! without mid-air unwinding, and tight convex-safe chord decimation.

use chip_math::DVec3;
use rayon::prelude::*;

use crate::{
    cl_data::MotionRole,
    params::{Rotary4AxisPattern, Rotary4AxisSimParams, RotaryCutDirection, RotaryTiltMode},
    rotary::{
        ResolvedRotaryConfig, RotaryAxisKind, RotaryDriveMode,
        detect_rotary_axial_undercut_warning, drop_cutter_rotary_z_rel,
        estimate_rotary_flank_cos_phi, mesh_has_steep_circumferential_overhang,
        resolve_rotary_axial_window,
    },
    strategies::{StrategyExecContext, compute_z_levels},
    surface3d::{SurfaceCutterProfile, SurfaceMesh3D, stepover_for_cusp_height},
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

    // Fine direct sampling bounds keeping curved helical flanks accurate prior to adaptive midpoint refinement
    let axial_sample = params.axial_sample_step.value_mm.clamp(0.08, 0.25);
    let n_u = (((u_end - u_start) / axial_sample).ceil() as usize).clamp(4, 2_000);
    let du = (u_end - u_start) / (n_u as f64);
    let u_samples: Vec<f64> = (0..=n_u).map(|i| u_start + (i as f64) * du).collect();

    let d_theta_req = params.angular_sample_deg.clamp(0.5, 1.5);
    let n_theta = ((angular_span_deg / d_theta_req).ceil() as usize).clamp(8, 720);
    let d_theta = angular_span_deg / (n_theta as f64);

    let cutter = SurfaceCutterProfile::from_tool_geometry(
        &ctx.tool.geometry,
        ctx.passes.stock_to_leave_radial.value_mm,
        ctx.passes.stock_to_leave_axial.value_mm,
    );
    let r_floor = (ctx.heights.bottom_z_wcs - rot.center_wcs.z).max(0.0);
    let r_outer_est = rot.stock_outer_radius_mm.max(2.0);
    let max_safe_cross = (r_outer_est * 0.65).max(0.5);

    // Stage A: Compute smooth C1-continuous cross-axis offset field `c_cross(u, θ)`
    let c_field = compute_cross_offset_field(
        &base_mesh,
        &rot,
        &cutter,
        params,
        u_start,
        u_end,
        start_deg,
        angular_span_deg,
        is_full_turn,
        r_floor,
        max_safe_cross,
    );
    ctx.check_cancel()?;

    // Slope-compensated cusp-height stepover scaling when enabled
    let stepover_mm = if params.use_cusp_height {
        let cusp = params.cusp_height.value_mm.clamp(0.002, 2.0);
        let base_step = stepover_for_cusp_height(&cutter, cusp, params.stepover.value_mm.max(0.05));
        let cos_phi = estimate_rotary_flank_cos_phi(
            &base_mesh,
            &rot,
            &cutter,
            u_start,
            u_end,
            start_deg,
            end_deg,
            is_full_turn,
            r_floor,
            matches!(params.pattern, Rotary4AxisPattern::HelicalFlowline),
        );
        (base_step * cos_phi).clamp(0.08, tool_d * 0.75)
    } else {
        params.stepover.value_mm.max(0.05)
    };

    let cut_feed = ctx.cutting.cutting_feed_mm_min;
    let plunge_feed = ctx.cutting.plunge_feed_mm_min.min(cut_feed).max(10.0);
    let smooth_tol = params.smoothing_tolerance.value_mm.clamp(0.0, 0.25);

    // Stage B: Exact direct f64 off-centerline drop-cutter evaluation + adaptive chord refinement
    match params.pattern {
        Rotary4AxisPattern::SimultaneousSpiral => {
            emit_simultaneous_spiral(
                ctx,
                &base_mesh,
                &rot,
                &cutter,
                &c_field,
                u_start,
                u_end,
                start_deg,
                end_deg,
                is_full_turn,
                r_floor,
                stepover_mm,
                d_theta,
                params.cut_direction,
                cut_feed,
                plunge_feed,
                smooth_tol,
                builder,
            )?;
        }
        Rotary4AxisPattern::HelicalFlowline => {
            emit_helical_flowline(
                ctx,
                &base_mesh,
                &rot,
                &cutter,
                &c_field,
                &u_samples,
                start_deg,
                end_deg,
                is_full_turn,
                r_floor,
                r_outer_est,
                stepover_mm,
                params.cut_direction,
                cut_feed,
                plunge_feed,
                smooth_tol,
                builder,
            )?;
        }
    }

    let safe_z_wcs = rot.center_wcs.z + rot.clearance_radius_mm.max(r_outer_est + 5.0);
    if let Some(last) = builder.current_pos()
        && last.z < safe_z_wcs - 1e-4
    {
        builder.rotary_rapid_to(
            DVec3::new(last.x, last.y, safe_z_wcs),
            builder.current_rotary_deg(),
        );
    }

    if let Some(warn) =
        detect_rotary_axial_undercut_warning(&base_mesh, &rot, u_start, u_end, r_floor)
    {
        builder.warnings.push(warn);
    }

    Ok(())
}

/// Smooth cross-axis lateral offset field `c_cross(u, θ)` (`mm`).
enum CrossOffsetField {
    Fixed(f64),
    Grid {
        u_start: f64,
        du: f64,
        n_u: usize,
        start_deg: f64,
        d_theta: f64,
        n_theta: usize,
        is_full_turn: bool,
        c_cross_slices: Vec<Vec<f64>>,
    },
}

impl CrossOffsetField {
    #[inline]
    fn sample(&self, u: f64, theta_deg: f64) -> f64 {
        match self {
            Self::Fixed(c) => *c,
            Self::Grid {
                u_start,
                du,
                n_u,
                start_deg,
                d_theta,
                n_theta,
                is_full_turn,
                c_cross_slices,
            } => {
                let fu = ((u - *u_start) / *du).clamp(0.0, *n_u as f64);
                let iu0 = (fu.floor() as usize).min(*n_u);
                let iu1 = (iu0 + 1).min(*n_u);
                let tu = (fu - iu0 as f64).clamp(0.0, 1.0);

                let (it0, it1, tt) = if *is_full_turn {
                    let wrapped = (theta_deg - *start_deg).rem_euclid(360.0);
                    let ft = wrapped / *d_theta;
                    let i0 = (ft.floor() as usize) % *n_theta;
                    let i1 = (i0 + 1) % *n_theta;
                    (i0, i1, (ft - ft.floor()).clamp(0.0, 1.0))
                } else {
                    let max_idx = c_cross_slices.len().saturating_sub(1);
                    let ft = ((theta_deg - *start_deg) / *d_theta).clamp(0.0, max_idx as f64);
                    let i0 = (ft.floor() as usize).min(max_idx);
                    let i1 = (i0 + 1).min(max_idx);
                    (i0, i1, (ft - i0 as f64).clamp(0.0, 1.0))
                };

                let c00 = c_cross_slices[it0][iu0];
                let c01 = c_cross_slices[it0][iu1];
                let c10 = c_cross_slices[it1][iu0];
                let c11 = c_cross_slices[it1][iu1];
                let c0 = c00 + (c01 - c00) * tu;
                let c1 = c10 + (c11 - c10) * tu;
                c0 + (c1 - c0) * tt
            }
        }
    }
}

/// Samples the centerline drop-cutter radius grid `r_center[it][iu]` over the rotary window.
#[allow(clippy::too_many_arguments)]
fn sample_centerline_radius_grid(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    params: &Rotary4AxisSimParams,
    u_start: f64,
    u_end: f64,
    start_deg: f64,
    angular_span_deg: f64,
    is_full_turn: bool,
    r_floor: f64,
) -> (usize, f64, usize, f64, Vec<Vec<f64>>) {
    let axial_tilt = params.axial_sample_step.value_mm.clamp(0.20, 0.40);
    let n_u = (((u_end - u_start) / axial_tilt).ceil() as usize).clamp(4, 1_000);
    let du = (u_end - u_start) / (n_u as f64);
    let u_samples: Vec<f64> = (0..=n_u).map(|i| u_start + (i as f64) * du).collect();

    let d_theta_req = params.angular_sample_deg.clamp(1.0, 2.0);
    let n_theta = ((angular_span_deg / d_theta_req).ceil() as usize).clamp(8, 360);
    let d_theta = angular_span_deg / (n_theta as f64);
    let theta_count = if is_full_turn { n_theta } else { n_theta + 1 };

    let r_center: Vec<Vec<f64>> = (0..theta_count)
        .into_par_iter()
        .map(|it| {
            let theta_deg = start_deg + (it as f64) * d_theta;
            u_samples
                .iter()
                .map(|&u_val| {
                    drop_cutter_rotary_z_rel(base_mesh, rot, cutter, u_val, 0.0, theta_deg, r_floor)
                })
                .collect()
        })
        .collect();

    (n_u, du, n_theta, d_theta, r_center)
}

/// Applies `smooth_passes` of bounded 5-point stencil smoothing to a `(theta, u)` cross-offset grid,
/// enforcing `|c_cross| <= min(r_center, max_safe_cross)` at every cell so smoothing never bleeds
/// non-zero lateral offsets into empty-center voids (`r_center == 0`).
fn smooth_cross_offset_grid(
    mut cur: Vec<Vec<f64>>,
    r_center: &[Vec<f64>],
    is_full_turn: bool,
    smooth_passes: usize,
    max_safe_cross: f64,
) -> Vec<Vec<f64>> {
    let theta_count = cur.len();
    let nu = cur.first().map_or(0, Vec::len);
    if theta_count == 0 || nu == 0 || smooth_passes == 0 {
        return cur;
    }
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
                let val = 0.40 * cur[it][iu]
                    + 0.20 * cur[itp][iu]
                    + 0.20 * cur[itn][iu]
                    + 0.10 * cur[it][iup]
                    + 0.10 * cur[it][iun];
                let cell_limit = r_center[it][iu].max(0.0).min(max_safe_cross);
                next[it][iu] = val.clamp(-cell_limit, cell_limit);
            }
        }
        std::mem::swap(&mut cur, &mut next);
    }
    cur
}

/// Computes the smooth cross-axis lateral offset field `c_cross(u, θ)` (`mm`) using direct
/// exact `f64` centerline drop-cutter queries.
#[allow(clippy::too_many_arguments)]
fn compute_cross_offset_field(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    params: &Rotary4AxisSimParams,
    u_start: f64,
    u_end: f64,
    start_deg: f64,
    angular_span_deg: f64,
    is_full_turn: bool,
    r_floor: f64,
    max_safe_cross: f64,
) -> CrossOffsetField {
    let smooth_passes = params.tilt_smoothing_window.clamp(0, 12) as usize;
    match params.tilt_mode {
        RotaryTiltMode::FixedLateralOffset => {
            let c_fixed = params
                .lateral_offset
                .value_mm
                .clamp(-max_safe_cross, max_safe_cross);
            if c_fixed.abs() <= 1e-6 {
                return CrossOffsetField::Fixed(0.0);
            }

            let (n_u, du, n_theta, d_theta, r_center) = sample_centerline_radius_grid(
                base_mesh,
                rot,
                cutter,
                params,
                u_start,
                u_end,
                start_deg,
                angular_span_deg,
                is_full_turn,
                r_floor,
            );
            let c_abs = c_fixed.abs();
            let needs_center_taper = r_center
                .iter()
                .any(|row| row.iter().any(|&r| r < c_abs - 1e-4));
            if !needs_center_taper {
                return CrossOffsetField::Fixed(c_fixed);
            }

            let raw: Vec<Vec<f64>> = r_center
                .iter()
                .map(|row| {
                    row.iter()
                        .map(|&r| c_fixed * (r.max(0.0) / c_abs).clamp(0.0, 1.0))
                        .collect()
                })
                .collect();
            let cur = smooth_cross_offset_grid(raw, &r_center, is_full_turn, smooth_passes, c_abs);

            CrossOffsetField::Grid {
                u_start,
                du,
                n_u,
                start_deg,
                d_theta,
                n_theta,
                is_full_turn,
                c_cross_slices: cur,
            }
        }
        RotaryTiltMode::SurfaceNormalAutoTilt => {
            let max_tilt_rad = params.max_tilt_deg.clamp(0.0, 40.0).to_radians();
            if max_tilt_rad <= 1e-4 {
                return CrossOffsetField::Fixed(0.0);
            }

            // Exact centerline drop-cutter pass to estimate local circumferential flank slope `dr/(r*dθ)`
            let (n_u, du, n_theta, d_theta, r_center) = sample_centerline_radius_grid(
                base_mesh,
                rot,
                cutter,
                params,
                u_start,
                u_end,
                start_deg,
                angular_span_deg,
                is_full_turn,
                r_floor,
            );
            let theta_count = r_center.len();
            let nu = n_u + 1;

            // Convert circumferential slope into bounded cross-axis offset aligning cutter with flank normal
            let d_th_rad = d_theta.to_radians().max(1e-4);
            let axis_cross_sign = match rot.axis {
                // XAxis: e2 = -Y -> +theta moves toward -Y (for CCW); positive dr/dθ faces -theta (+Y)
                RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => rot.direction.sign(),
                // YAxis: e2 = +X -> +theta moves toward +X (for CCW); positive dr/dθ faces -theta (-X)
                RotaryAxisKind::YAxis => -rot.direction.sign(),
            };

            let mut raw = vec![vec![0.0_f64; nu]; theta_count];
            let mut local_slope = vec![vec![0.0_f64; nu]; theta_count];
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
                    let r_raw = r_center[it][iu].max(0.0);
                    let r_denom = r_raw.max(1.0);
                    let dr_dtheta = (r_center[it_next][iu] - r_center[it_prev][iu]) / span_rad;
                    let slope = dr_dtheta / r_denom;
                    local_slope[it][iu] = slope;
                    let slope_angle = slope.atan();
                    let clamped_alpha = slope_angle.clamp(-max_tilt_rad, max_tilt_rad);
                    let c_val = (axis_cross_sign * r_raw * clamped_alpha.sin())
                        .clamp(-max_safe_cross, max_safe_cross);
                    raw[it][iu] = c_val;
                }
            }

            // Overhang-aware multi-offset circumferential flank probing: activates strictly when
            // the target mesh contains steep (>70°) or overhanging circumferential blade walls so
            // non-overhanging flutes/cams (and Phase 5 goldens) remain 100% bit-identical.
            let probed_overhangs = mesh_has_steep_circumferential_overhang(
                base_mesh,
                rot,
                u_start,
                u_end,
                cutter.max_radius_mm(),
            );
            if probed_overhangs {
                const CLIFF_SLOPE_THRESH: f64 = 0.700_207_538_209_709_7; // tan(35°)
                let look_steps =
                    ((params.max_tilt_deg.clamp(5.0, 40.0) * 0.90) / d_theta).ceil() as usize;
                let look_steps = look_steps.clamp(2, 24);
                let sin_max_tilt = max_tilt_rad.sin();

                let probed: Vec<Vec<f64>> = (0..theta_count)
                    .into_par_iter()
                    .map(|it| {
                        let theta_deg = start_deg + (it as f64) * d_theta;
                        let mut row = raw[it].clone();

                        for (iu, c_out) in row.iter_mut().enumerate() {
                            let r_raw = r_center[it][iu].max(0.0);
                            if r_raw <= r_floor + 1e-4 {
                                continue;
                            }
                            let c_lim = (r_raw * sin_max_tilt).min(r_raw).min(max_safe_cross);
                            if c_lim <= 0.05 {
                                continue;
                            }

                            let c_slope = *c_out;
                            let is_cliff = local_slope[it][iu].abs() > CLIFF_SLOPE_THRESH;

                            // Local crest check + nearby overhanging cliff search within ±look_steps
                            let mut r_local_max = r_raw;
                            for delta in 1..=3usize {
                                if is_full_turn {
                                    let im = (it + theta_count - delta % theta_count) % theta_count;
                                    let ip = (it + delta) % theta_count;
                                    r_local_max =
                                        r_local_max.max(r_center[im][iu]).max(r_center[ip][iu]);
                                } else {
                                    if it >= delta {
                                        r_local_max = r_local_max.max(r_center[it - delta][iu]);
                                    }
                                    if it + delta < theta_count {
                                        r_local_max = r_local_max.max(r_center[it + delta][iu]);
                                    }
                                }
                            }

                            let mut neg_dist: Option<usize> = None;
                            let mut pos_dist: Option<usize> = None;
                            for step in 1..=look_steps {
                                if neg_dist.is_none() {
                                    let im_opt = if is_full_turn {
                                        Some((it + theta_count - step % theta_count) % theta_count)
                                    } else {
                                        it.checked_sub(step)
                                    };
                                    if let Some(im) = im_opt
                                        && local_slope[im][iu].abs() > CLIFF_SLOPE_THRESH
                                        && r_center[im][iu] > r_raw + 0.5
                                    {
                                        neg_dist = Some(step);
                                    }
                                }
                                if pos_dist.is_none() {
                                    let ip_opt = if is_full_turn {
                                        Some((it + step) % theta_count)
                                    } else if it + step < theta_count {
                                        Some(it + step)
                                    } else {
                                        None
                                    };
                                    if let Some(ip) = ip_opt
                                        && local_slope[ip][iu].abs() > CLIFF_SLOPE_THRESH
                                        && r_center[ip][iu] > r_raw + 0.5
                                    {
                                        pos_dist = Some(step);
                                    }
                                }
                                if neg_dist.is_some() && pos_dist.is_some() {
                                    break;
                                }
                            }

                            let has_nearby_cliff = neg_dist.is_some() || pos_dist.is_some();
                            if !is_cliff && !has_nearby_cliff {
                                continue;
                            }

                            // Symmetric 7-sample lateral probe fan c ∈ [-c_lim, +c_lim]
                            let u_val = u_start + (iu as f64) * du;
                            let mut samples = [(0.0_f64, r_raw, r_raw); 7];
                            let mut r_eff_min = r_raw;
                            for (idx, &k) in [-3_i32, -2, -1, 0, 1, 2, 3].iter().enumerate() {
                                if k == 0 {
                                    samples[idx] = (0.0, r_raw, r_raw);
                                } else {
                                    let c_cand = (f64::from(k) / 3.0) * c_lim;
                                    let z_cand = drop_cutter_rotary_z_rel(
                                        base_mesh, rot, cutter, u_val, c_cand, theta_deg, r_floor,
                                    );
                                    let r_eff = z_cand.hypot(c_cand);
                                    samples[idx] = (c_cand, z_cand, r_eff);
                                    if z_cand >= r_floor {
                                        r_eff_min = r_eff_min.min(r_eff);
                                    }
                                }
                            }

                            let on_blade_crest = r_raw >= r_local_max - 0.75;
                            let mut c_target = c_slope;

                            if !on_blade_crest && r_eff_min < r_raw - 0.12 {
                                // Flank or narrow overhanging groove: pick candidate reaching deepest
                                // into the groove while preferring smoothness with `c_slope`.
                                let mut best_c = c_slope;
                                let mut best_cost = f64::INFINITY;
                                for &(c_cand, z_cand, r_eff) in &samples {
                                    if z_cand < r_floor || r_eff > r_eff_min + 0.08 {
                                        continue;
                                    }
                                    let cost =
                                        (r_eff - r_eff_min) + 0.02 * (c_cand - c_slope).abs();
                                    if cost < best_cost {
                                        best_cost = cost;
                                        best_c = c_cand;
                                    }
                                }
                                c_target = 0.75 * best_c + 0.25 * c_slope;
                            } else if has_nearby_cliff {
                                // Recessed groove floor beside an overhanging helical crest: tilt
                                // toward the closer overhanging cliff as far as stays clear of the
                                // overhanging blade tip (`r_eff <= r_eff_min + 0.35 mm`).
                                let (overhang_dir, dist) = match (neg_dist, pos_dist) {
                                    (Some(dn), Some(dp)) => {
                                        if dn <= dp {
                                            (-axis_cross_sign, dn)
                                        } else {
                                            (axis_cross_sign, dp)
                                        }
                                    }
                                    (Some(dn), None) => (-axis_cross_sign, dn),
                                    (None, Some(dp)) => (axis_cross_sign, dp),
                                    (None, None) => (0.0, look_steps),
                                };
                                if overhang_dir.abs() > 0.5 {
                                    let mut best_reach_c = 0.0_f64;
                                    for &(c_cand, z_cand, r_eff) in &samples {
                                        if z_cand >= r_floor
                                            && c_cand * overhang_dir > 0.0
                                            && r_eff <= r_eff_min + 0.35
                                            && c_cand.abs() > best_reach_c.abs()
                                        {
                                            best_reach_c = c_cand;
                                        }
                                    }
                                    let prox = (1.0 - (dist as f64) / ((look_steps + 2) as f64))
                                        .clamp(0.25, 1.0);
                                    let c_reach = best_reach_c * prox;
                                    c_target = if is_cliff {
                                        0.75 * c_reach + 0.25 * c_slope
                                    } else {
                                        c_reach
                                    };
                                }
                            }

                            *c_out = c_target.clamp(-c_lim, c_lim);
                        }
                        row
                    })
                    .collect();
                raw = probed;
            }

            // Smooth cross-axis tilt field so simultaneous Y/X & rotary motion stays C1-smooth
            // with zero sudden cross-axis jerks across steep overhang transitions.
            let cur = if probed_overhangs {
                let mut g = smooth_cross_offset_grid(
                    raw,
                    &r_center,
                    is_full_turn,
                    smooth_passes.max(8),
                    max_safe_cross,
                );
                // Enforce a strict angular & axial rate-of-change bound (`<= 0.45 mm/deg`) via
                // iterative projection so steep overhang reversals never create cross-axis jerks.
                let max_dc_th = (0.45 * d_theta).max(0.15);
                let max_dc_u = (0.60 * du).max(0.15);
                for _ in 0..8 {
                    let mut changed = false;
                    for it in 0..theta_count {
                        let it_next = if is_full_turn {
                            (it + 1) % theta_count
                        } else if it + 1 < theta_count {
                            it + 1
                        } else {
                            continue;
                        };
                        for iu in 0..nu {
                            let diff = g[it_next][iu] - g[it][iu];
                            if diff.abs() > max_dc_th {
                                let excess = 0.5 * (diff.abs() - max_dc_th) * diff.signum();
                                g[it][iu] += excess;
                                g[it_next][iu] -= excess;
                                changed = true;
                            }
                        }
                    }
                    for row in &mut g {
                        for iu in 0..(nu.saturating_sub(1)) {
                            let diff = row[iu + 1] - row[iu];
                            if diff.abs() > max_dc_u {
                                let excess = 0.5 * (diff.abs() - max_dc_u) * diff.signum();
                                row[iu] += excess;
                                row[iu + 1] -= excess;
                                changed = true;
                            }
                        }
                    }
                    if !changed {
                        break;
                    }
                }
                smooth_cross_offset_grid(g, &r_center, is_full_turn, 2, max_safe_cross)
            } else {
                smooth_cross_offset_grid(
                    raw,
                    &r_center,
                    is_full_turn,
                    smooth_passes,
                    max_safe_cross,
                )
            };

            CrossOffsetField::Grid {
                u_start,
                du,
                n_u,
                start_deg,
                d_theta,
                n_theta,
                is_full_turn,
                c_cross_slices: cur,
            }
        }
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

#[inline]
fn clamp_to_shell(c_cross: f64, z_surf: f64, r_ceil: f64) -> f64 {
    let z_shell = (r_ceil * r_ceil - c_cross * c_cross).max(0.0).sqrt();
    z_surf.max(z_shell)
}

/// Evaluates exact `(u, theta_deg, c_cross, z_rel)` in parallel along a sequence of `(u, theta_deg)`
/// coordinates, then recursively refines every chord whose midpoint deviates from exact `f64`
/// off-centerline drop-cutter contact (guarding both convex peaks `> 4 µm` and concave valleys).
fn evaluate_and_refine_4axis_strip(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    c_field: &CrossOffsetField,
    coords: &[(f64, f64)],
    r_floor: f64,
    smooth_tol: f64,
) -> Vec<(f64, f64, f64, f64)> {
    if coords.is_empty() {
        return Vec::new();
    }

    let evaluated: Vec<(f64, f64, f64, f64)> = coords
        .par_iter()
        .map(|&(u, th)| {
            let c = c_field.sample(u, th);
            let z = drop_cutter_rotary_z_rel(base_mesh, rot, cutter, u, c, th, r_floor);
            (u, th, c, z)
        })
        .collect();

    if evaluated.len() <= 1 {
        return evaluated;
    }

    let concave_tol = smooth_tol.clamp(0.004, 0.015);
    let inserted: Vec<Vec<(f64, f64, f64, f64)>> = (0..(evaluated.len() - 1))
        .into_par_iter()
        .map(|i| {
            let mut mid_pts = Vec::new();
            refine_4axis_segment(
                base_mesh,
                rot,
                cutter,
                c_field,
                r_floor,
                evaluated[i],
                evaluated[i + 1],
                concave_tol,
                0,
                &mut mid_pts,
            );
            mid_pts
        })
        .collect();

    let extra: usize = inserted.iter().map(Vec::len).sum();
    let mut out = Vec::with_capacity(evaluated.len() + extra);
    for (i, seg_mids) in inserted.into_iter().enumerate() {
        out.push(evaluated[i]);
        out.extend(seg_mids);
    }
    if let Some(&last) = evaluated.last() {
        out.push(last);
    }
    out
}

#[allow(clippy::too_many_arguments)]
fn refine_4axis_segment(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    c_field: &CrossOffsetField,
    r_floor: f64,
    a: (f64, f64, f64, f64),
    b: (f64, f64, f64, f64),
    concave_tol: f64,
    depth: u8,
    out: &mut Vec<(f64, f64, f64, f64)>,
) {
    let max_depth = if (b.3 - a.3).abs() > 0.25 { 8 } else { 4 };
    if depth >= max_depth {
        return;
    }
    let d_u = (b.0 - a.0).abs();
    let d_th = (b.1 - a.1).abs();
    let d_z = (b.3 - a.3).abs();
    let d_c = (b.2 - a.2).abs();
    if d_th <= 0.35 && d_z <= 0.015 && d_c <= 0.02 && (d_th > 1e-6 || d_u <= 0.05) {
        return;
    }

    let u_mid = 0.5 * (a.0 + b.0);
    let th_mid = 0.5 * (a.1 + b.1);
    let c_mid = c_field.sample(u_mid, th_mid);
    let z_mid = drop_cutter_rotary_z_rel(base_mesh, rot, cutter, u_mid, c_mid, th_mid, r_floor);

    let c_chord = 0.5 * (a.2 + b.2);
    let z_chord = 0.5 * (a.3 + b.3);
    // Also verify exact contact at the linearly interpolated cross-axis offset `c_chord`
    let z_at_chord_c = if (c_mid - c_chord).abs() > 1e-4 {
        drop_cutter_rotary_z_rel(base_mesh, rot, cutter, u_mid, c_chord, th_mid, r_floor)
    } else {
        z_mid
    };

    let convex_err = (z_mid - z_chord).max(z_at_chord_c - z_chord);
    let concave_err = z_chord - z_mid;
    let cross_err = (c_mid - c_chord).abs();

    let mut needs_split =
        convex_err > 0.0025 || concave_err > concave_tol || cross_err > concave_tol.min(0.006);
    // On pure-axial strips/links (`d_th == 0`) crossing steep vertical cliffs, also check
    // quarter-points so asymmetric shoulders never clip (leaves full-turn spirals bit-identical).
    if !needs_split && d_th <= 1e-6 && d_z > 0.25 {
        for t in [0.25_f64, 0.75] {
            let u_q = a.0 + t * (b.0 - a.0);
            let th_q = a.1 + t * (b.1 - a.1);
            let c_q = a.2 + t * (b.2 - a.2);
            let z_q = drop_cutter_rotary_z_rel(base_mesh, rot, cutter, u_q, c_q, th_q, r_floor);
            let z_c = a.3 + t * (b.3 - a.3);
            if (z_q - z_c) > 0.0025 {
                needs_split = true;
                break;
            }
        }
    }

    if needs_split {
        let mid = (u_mid, th_mid, c_mid, z_mid);
        refine_4axis_segment(
            base_mesh,
            rot,
            cutter,
            c_field,
            r_floor,
            a,
            mid,
            concave_tol,
            depth + 1,
            out,
        );
        out.push(mid);
        refine_4axis_segment(
            base_mesh,
            rot,
            cutter,
            c_field,
            r_floor,
            mid,
            b,
            concave_tol,
            depth + 1,
            out,
        );
    }
}

fn surface_radius_bounds(pts: &[(f64, f64, f64, f64)]) -> (f64, f64) {
    let mut r_min = f64::INFINITY;
    let mut r_max = 0.0_f64;
    for &(_, _, c, z) in pts {
        let r_eff = z.hypot(c);
        r_min = r_min.min(r_eff);
        r_max = r_max.max(r_eff);
    }
    (r_min, r_max)
}

#[allow(clippy::too_many_arguments)]
fn emit_simultaneous_spiral(
    ctx: &StrategyExecContext<'_>,
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    c_field: &CrossOffsetField,
    u_start: f64,
    u_end: f64,
    start_deg: f64,
    end_deg: f64,
    is_full_turn: bool,
    r_floor: f64,
    stepover_mm: f64,
    d_theta: f64,
    cut_direction: RotaryCutDirection,
    cut_feed: f64,
    plunge_feed: f64,
    smooth_tol: f64,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    if is_full_turn {
        let n_turns = ((u_end - u_start) / stepover_mm).ceil().max(1.0);
        let steps_per_rev = ((360.0 / d_theta).ceil() as usize).clamp(48, 720);
        let rev_step_deg = 360.0 / (steps_per_rev as f64);

        let helix_steps = ((n_turns * (steps_per_rev as f64)).round() as usize).max(steps_per_rev);
        let th_helix_start = start_deg + 360.0;
        let th_helix_span = n_turns * 360.0;
        let th_end_start = th_helix_start + th_helix_span;
        let total_level_deg = (n_turns + 2.0) * 360.0;

        let mut base_coords: Vec<(f64, f64)> =
            Vec::with_capacity(2 * steps_per_rev + helix_steps + 2);
        // 1. Start 360° cleanup ring at u_start
        for k in 0..=steps_per_rev {
            base_coords.push((u_start, start_deg + (k as f64) * rev_step_deg));
        }
        // 2. Continuous simultaneous 4-axis helix
        for k in 1..=helix_steps {
            let t = (k as f64) / (helix_steps as f64);
            let u = u_start + t * (u_end - u_start);
            let th = th_helix_start + t * th_helix_span;
            base_coords.push((u, th));
        }
        // 3. End 360° cleanup ring at u_end
        for k in 1..=steps_per_rev {
            base_coords.push((u_end, th_end_start + (k as f64) * rev_step_deg));
        }

        let base_refined = evaluate_and_refine_4axis_strip(
            base_mesh,
            rot,
            cutter,
            c_field,
            &base_coords,
            r_floor,
            smooth_tol,
        );
        ctx.check_cancel()?;

        let (r_min_surf, r_max_surf) = surface_radius_bounds(&base_refined);
        if !r_min_surf.is_finite() {
            return Err(CamError::EmptyToolpath {
                op_name: ctx.op.name.clone(),
            });
        }
        let r_outer = rot.stock_outer_radius_mm.max(r_max_surf);
        let radial_levels = compute_z_levels(
            r_outer,
            r_min_surf,
            ctx.passes.multiple_depths,
            ctx.passes.max_stepdown.value_mm,
            ctx.passes.even_stepdowns,
        )?;
        let safe_z_wcs = rot.center_wcs.z + rot.clearance_radius_mm.max(r_outer + 5.0);
        let n_levels = radial_levels.len();

        for (li, &r_ceil) in radial_levels.iter().enumerate() {
            ctx.check_cancel()?;
            let role = if li + 1 == n_levels {
                MotionRole::FinishCut
            } else {
                MotionRole::Cut
            };
            let level_th_offset = (li as f64) * total_level_deg;
            let level_pts: Vec<(f64, f64, f64, f64)> = base_refined
                .iter()
                .map(|&(u, th, c, z)| (u, th + level_th_offset, c, clamp_to_shell(c, z, r_ceil)))
                .collect();

            let pts = decimate_4axis_strip(&level_pts, smooth_tol);
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
        }
    } else {
        let n_lanes = (((u_end - u_start) / stepover_mm).ceil() as usize).max(1);
        let lane_du = (u_end - u_start) / (n_lanes as f64);
        let n_ang = (((end_deg - start_deg) / d_theta).ceil() as usize).max(4);
        let ang_step = (end_deg - start_deg) / (n_ang as f64);

        let refined_lanes: Vec<Vec<(f64, f64, f64, f64)>> = (0..=n_lanes)
            .map(|lane| {
                let u = u_start + (lane as f64) * lane_du;
                let reverse = match cut_direction {
                    RotaryCutDirection::ZigZag => lane % 2 == 1,
                    RotaryCutDirection::Climb => false,
                    RotaryCutDirection::Conventional => true,
                };
                let coords: Vec<(f64, f64)> = (0..=n_ang)
                    .map(|k| {
                        let kk = if reverse { n_ang - k } else { k };
                        (u, start_deg + (kk as f64) * ang_step)
                    })
                    .collect();
                evaluate_and_refine_4axis_strip(
                    base_mesh, rot, cutter, c_field, &coords, r_floor, smooth_tol,
                )
            })
            .collect();
        ctx.check_cancel()?;

        // Precompute exact gouge-free ZigZag axial link transitions (`u_prev -> u_cur`) at the
        // shared sector endpoint angle `th0` so diagonal helical crests are never clipped.
        let link_steps = ((lane_du / 0.25).ceil() as usize).clamp(2, 64);
        let lane_links: Vec<Vec<(f64, f64, f64, f64)>> = (0..=n_lanes)
            .map(|lane| {
                if lane == 0 || cut_direction != RotaryCutDirection::ZigZag {
                    return Vec::new();
                }
                let Some(&(u0, th0, _, _)) = refined_lanes[lane].first() else {
                    return Vec::new();
                };
                let u_prev = u_start + ((lane - 1) as f64) * lane_du;
                let coords: Vec<(f64, f64)> = (0..=link_steps)
                    .map(|s| {
                        let t = (s as f64) / (link_steps as f64);
                        (u_prev + t * (u0 - u_prev), th0)
                    })
                    .collect();
                evaluate_and_refine_4axis_strip(
                    base_mesh, rot, cutter, c_field, &coords, r_floor, smooth_tol,
                )
            })
            .collect();
        ctx.check_cancel()?;

        let mut r_min_surf = f64::INFINITY;
        let mut r_max_surf = 0.0_f64;
        for lane in &refined_lanes {
            let (lo, hi) = surface_radius_bounds(lane);
            r_min_surf = r_min_surf.min(lo);
            r_max_surf = r_max_surf.max(hi);
        }
        for link in &lane_links {
            let (lo, hi) = surface_radius_bounds(link);
            r_min_surf = r_min_surf.min(lo);
            r_max_surf = r_max_surf.max(hi);
        }
        if !r_min_surf.is_finite() {
            return Err(CamError::EmptyToolpath {
                op_name: ctx.op.name.clone(),
            });
        }
        let r_outer = rot.stock_outer_radius_mm.max(r_max_surf);
        let radial_levels = compute_z_levels(
            r_outer,
            r_min_surf,
            ctx.passes.multiple_depths,
            ctx.passes.max_stepdown.value_mm,
            ctx.passes.even_stepdowns,
        )?;
        let safe_z_wcs = rot.center_wcs.z + rot.clearance_radius_mm.max(r_outer + 5.0);
        let n_levels = radial_levels.len();

        for (li, &r_ceil) in radial_levels.iter().enumerate() {
            ctx.check_cancel()?;
            let role = if li + 1 == n_levels {
                MotionRole::FinishCut
            } else {
                MotionRole::Cut
            };

            for (lane, base_lane) in refined_lanes.iter().enumerate() {
                let level_pts: Vec<(f64, f64, f64, f64)> = base_lane
                    .iter()
                    .map(|&(u, th, c, z)| (u, th, c, clamp_to_shell(c, z, r_ceil)))
                    .collect();
                let pts = decimate_4axis_strip(&level_pts, smooth_tol);
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
                    } else if lane_links[lane].len() >= 2 {
                        for &(lu, lth, lc, lz) in lane_links[lane].iter().skip(1) {
                            let z_clamped = clamp_to_shell(lc, lz, r_ceil);
                            builder.rotary_linear_to(
                                off_center_wcs(rot, lu, lc, z_clamped),
                                lth,
                                cut_feed,
                                lc.hypot(z_clamped).max(1.0),
                                MotionRole::LinkTransition,
                            );
                        }
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
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    c_field: &CrossOffsetField,
    u_samples: &[f64],
    start_deg: f64,
    end_deg: f64,
    is_full_turn: bool,
    r_floor: f64,
    r_outer_est: f64,
    stepover_mm: f64,
    cut_direction: RotaryCutDirection,
    cut_feed: f64,
    plunge_feed: f64,
    smooth_tol: f64,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    let angular_span = end_deg - start_deg;
    let delta_deg = (stepover_mm / r_outer_est.max(2.0))
        .to_degrees()
        .clamp(0.5, 45.0);
    let n_lanes = ((angular_span / delta_deg).ceil() as usize).max(4);
    let lane_step_deg = angular_span / (n_lanes as f64);
    let lane_count = if is_full_turn { n_lanes } else { n_lanes + 1 };

    let refined_lanes: Vec<Vec<(f64, f64, f64, f64)>> = (0..lane_count)
        .map(|lane| {
            let th = start_deg + (lane as f64) * lane_step_deg;
            let reverse = match cut_direction {
                RotaryCutDirection::ZigZag => lane % 2 == 1,
                RotaryCutDirection::Climb => false,
                RotaryCutDirection::Conventional => true,
            };
            let coords: Vec<(f64, f64)> = if reverse {
                u_samples.iter().rev().map(|&u| (u, th)).collect()
            } else {
                u_samples.iter().map(|&u| (u, th)).collect()
            };
            evaluate_and_refine_4axis_strip(
                base_mesh, rot, cutter, c_field, &coords, r_floor, smooth_tol,
            )
        })
        .collect();
    ctx.check_cancel()?;

    let mut r_min_surf = f64::INFINITY;
    let mut r_max_surf = 0.0_f64;
    for lane in &refined_lanes {
        let (lo, hi) = surface_radius_bounds(lane);
        r_min_surf = r_min_surf.min(lo);
        r_max_surf = r_max_surf.max(hi);
    }
    if !r_min_surf.is_finite() {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    let r_outer = rot.stock_outer_radius_mm.max(r_max_surf);
    let radial_levels = compute_z_levels(
        r_outer,
        r_min_surf,
        ctx.passes.multiple_depths,
        ctx.passes.max_stepdown.value_mm,
        ctx.passes.even_stepdowns,
    )?;
    let safe_z_wcs = rot.center_wcs.z + rot.clearance_radius_mm.max(r_outer + 5.0);
    let n_levels = radial_levels.len();

    // Precompute exact gouge-free ZigZag link transitions between adjacent lanes
    let link_steps = ((lane_step_deg / 1.5).ceil() as usize).clamp(2, 24);
    let lane_links: Vec<Vec<(f64, f64, f64, f64)>> = (0..lane_count)
        .map(|lane| {
            if lane == 0 || cut_direction != RotaryCutDirection::ZigZag {
                return Vec::new();
            }
            let Some(&(u0, th0, _, _)) = refined_lanes[lane].first() else {
                return Vec::new();
            };
            let prev_th = start_deg + ((lane - 1) as f64) * lane_step_deg;
            let coords: Vec<(f64, f64)> = (0..=link_steps)
                .map(|s| {
                    let t = (s as f64) / (link_steps as f64);
                    (u0, prev_th + t * (th0 - prev_th))
                })
                .collect();
            evaluate_and_refine_4axis_strip(
                base_mesh, rot, cutter, c_field, &coords, r_floor, smooth_tol,
            )
        })
        .collect();

    for (li, &r_ceil) in radial_levels.iter().enumerate() {
        ctx.check_cancel()?;
        let role = if li + 1 == n_levels {
            MotionRole::FinishCut
        } else {
            MotionRole::Cut
        };

        for (lane, base_lane) in refined_lanes.iter().enumerate() {
            ctx.check_cancel()?;
            let level_pts: Vec<(f64, f64, f64, f64)> = base_lane
                .iter()
                .map(|&(u, th, c, z)| (u, th, c, clamp_to_shell(c, z, r_ceil)))
                .collect();
            let pts = decimate_4axis_strip(&level_pts, smooth_tol);
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
                    for &(lu, lth, lc, lz) in lane_links[lane].iter().skip(1) {
                        let z_clamped = clamp_to_shell(lc, lz, r_ceil);
                        builder.rotary_linear_to(
                            off_center_wcs(rot, lu, lc, z_clamped),
                            lth,
                            cut_feed,
                            lc.hypot(z_clamped).max(1.0),
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
/// never dips `> 2 µm` below convex ridges in `z`, preserves cross-axis `c_cross` accuracy within
/// `tol_mm`, and keeps angular spans `<= 1.5°` on curved contours (`<= 4.0°` on constant-radius zones).
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
            1.5
        } else {
            4.0
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
            if (pz - z_interp) > 0.0015
                || (z_interp - pz) > tol_mm
                || (pc - c_interp).abs() > tol_mm.min(0.005)
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
