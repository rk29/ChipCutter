//! 2-Linear + Rotary (`Z+X+A` / `Z+Y+B`) 3D Surfacing Strategy (`OperationKind::Rotary2Linear`).
//!
//! Strictly locks the perpendicular cross-axis (`Y = center.y` for `X/A`, or `X = center.x` for `Y/B`)
//! while computing exact `f64` direct Drop-Cutter radial contact heights along every emitted spiral
//! or axial-parallel strip in parallel (`rayon`), followed by adaptive gouge-free chord-midpoint
//! subdivision and multi-depth concentric radial roughing.

use chip_math::DVec3;
use rayon::prelude::*;

use crate::{
    cl_data::MotionRole,
    params::{Rotary2LinearParams, RotaryCutDirection, RotaryPatternMode},
    rotary::{
        ResolvedRotaryConfig, RotaryAxisKind, drop_cutter_rotary_z_rel,
        estimate_rotary_flank_cos_phi, resolve_rotary_axial_window,
    },
    strategies::{StrategyExecContext, compute_z_levels},
    surface3d::{SurfaceCutterProfile, SurfaceMesh3D, stepover_for_cusp_height},
    toolpath::ToolpathBuilder,
    wcs::CamError,
};

/// Generates a 2-linear + rotary 3D surface toolpath (`Z+X+A` or `Z+Y+B`) directly onto `builder`.
pub fn generate_rotary2linear(
    ctx: &StrategyExecContext<'_>,
    params: &Rotary2LinearParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    ctx.check_cancel()?;

    let Some(rot) = ctx.setup.rotary else {
        return Err(CamError::InvalidRotarySetup {
            op_name: ctx.op.name.clone(),
            reason: "Rotary 2-Linear Surfacing requires a Setup with 4th-Axis Rotary enabled"
                .to_string(),
        });
    };
    if rot.axis == RotaryAxisKind::ZAxis {
        return Err(CamError::InvalidRotarySetup {
            op_name: ctx.op.name.clone(),
            reason: "Rotary 2-Linear Surfacing requires a horizontal rotary axis (A/X or B/Y)"
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

    // Angular window [start_deg, end_deg]
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

    // Tight direct sampling bounds keeping convex bevels & multi-lobe cams smooth prior to midpoint refinement
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
    let r_floor = (ctx.heights.bottom_z_wcs - rot.center_wcs.z).max(0.5);
    let r_outer_est = rot.stock_outer_radius_mm.max(2.0);

    // Effective stepover (either explicit mm or slope-compensated cusp-height derived)
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
            matches!(params.pattern, RotaryPatternMode::AxialParallel),
        );
        (base_step * cos_phi).clamp(0.08, tool_d * 0.75)
    } else {
        params.stepover.value_mm.max(0.05)
    };

    let cut_feed = ctx.cutting.cutting_feed_mm_min;
    let plunge_feed = ctx.cutting.plunge_feed_mm_min.min(cut_feed).max(10.0);
    let smooth_tol = params.smoothing_tolerance.value_mm.clamp(0.0, 0.25);

    match params.pattern {
        RotaryPatternMode::ContinuousSpiral => {
            emit_continuous_spiral(
                ctx,
                &base_mesh,
                &rot,
                &cutter,
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
        RotaryPatternMode::AxialParallel => {
            emit_axial_parallel(
                ctx,
                &base_mesh,
                &rot,
                &cutter,
                &u_samples,
                start_deg,
                end_deg,
                is_full_turn,
                r_outer_est,
                r_floor,
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

    Ok(())
}

#[inline]
fn centerline_wcs(rot: &ResolvedRotaryConfig, u_axial: f64, radius_mm: f64) -> DVec3 {
    let z_wcs = rot.center_wcs.z + radius_mm;
    match rot.axis {
        RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => {
            DVec3::new(u_axial, rot.center_wcs.y, z_wcs)
        }
        RotaryAxisKind::YAxis => DVec3::new(rot.center_wcs.x, u_axial, z_wcs),
    }
}

/// Evaluates exact `(u, theta_deg, r_exact)` in parallel along `coords`, then recursively inserts
/// exact chord midpoints whenever convex peak gouging (`> 4 µm`) or concave valley faceting
/// (`> smooth_tol.clamp(0.004, 0.015)`) exceeds tolerance (up to depth 4).
fn evaluate_and_refine_rotary_strip(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    coords: &[(f64, f64)],
    r_floor: f64,
    smooth_tol: f64,
) -> Vec<(f64, f64, f64)> {
    if coords.is_empty() {
        return Vec::new();
    }

    let evaluated: Vec<(f64, f64, f64)> = coords
        .par_iter()
        .map(|&(u, th)| {
            let r = drop_cutter_rotary_z_rel(base_mesh, rot, cutter, u, 0.0, th, r_floor);
            (u, th, r)
        })
        .collect();

    if evaluated.len() <= 1 {
        return evaluated;
    }

    let concave_tol = smooth_tol.clamp(0.004, 0.015);
    let inserted: Vec<Vec<(f64, f64, f64)>> = (0..(evaluated.len() - 1))
        .into_par_iter()
        .map(|i| {
            let mut mid_pts = Vec::new();
            refine_rotary_segment(
                base_mesh,
                rot,
                cutter,
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
fn refine_rotary_segment(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    r_floor: f64,
    a: (f64, f64, f64),
    b: (f64, f64, f64),
    concave_tol: f64,
    depth: u8,
    out: &mut Vec<(f64, f64, f64)>,
) {
    if depth >= 4 {
        return;
    }
    let d_th = (b.1 - a.1).abs();
    let d_r = (b.2 - a.2).abs();
    if d_th <= 0.5 && d_r <= 0.02 {
        return;
    }

    let u_mid = 0.5 * (a.0 + b.0);
    let th_mid = 0.5 * (a.1 + b.1);
    let r_mid = drop_cutter_rotary_z_rel(base_mesh, rot, cutter, u_mid, 0.0, th_mid, r_floor);
    let r_chord = 0.5 * (a.2 + b.2);

    if (r_mid - r_chord) > 0.004 || (r_chord - r_mid) > concave_tol {
        let mid = (u_mid, th_mid, r_mid);
        refine_rotary_segment(
            base_mesh,
            rot,
            cutter,
            r_floor,
            a,
            mid,
            concave_tol,
            depth + 1,
            out,
        );
        out.push(mid);
        refine_rotary_segment(
            base_mesh,
            rot,
            cutter,
            r_floor,
            mid,
            b,
            concave_tol,
            depth + 1,
            out,
        );
    }
}

fn radius_bounds(pts: &[(f64, f64, f64)]) -> (f64, f64) {
    let mut r_min = f64::INFINITY;
    let mut r_max = 0.0_f64;
    for &(_, _, r) in pts {
        r_min = r_min.min(r);
        r_max = r_max.max(r);
    }
    (r_min, r_max)
}

#[allow(clippy::too_many_arguments)]
fn emit_continuous_spiral(
    ctx: &StrategyExecContext<'_>,
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
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
        // 1. Start cleanup ring at u_start: [start_deg .. start_deg + 360]
        for k in 0..=steps_per_rev {
            base_coords.push((u_start, start_deg + (k as f64) * rev_step_deg));
        }
        // 2. Helical body: u_start -> u_end over n_turns revolutions
        for k in 1..=helix_steps {
            let t = (k as f64) / (helix_steps as f64);
            let u = u_start + t * (u_end - u_start);
            let th = th_helix_start + t * th_helix_span;
            base_coords.push((u, th));
        }
        // 3. End cleanup ring at u_end: extra 360°
        for k in 1..=steps_per_rev {
            base_coords.push((u_end, th_end_start + (k as f64) * rev_step_deg));
        }

        let base_refined = evaluate_and_refine_rotary_strip(
            base_mesh,
            rot,
            cutter,
            &base_coords,
            r_floor,
            smooth_tol,
        );
        ctx.check_cancel()?;

        let (r_min_surf, r_max_surf) = radius_bounds(&base_refined);
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
            let level_pts: Vec<(f64, f64, f64)> = base_refined
                .iter()
                .map(|&(u, th, r)| (u, th + level_th_offset, r.max(r_ceil)))
                .collect();

            let pts = decimate_rotary_strip(&level_pts, smooth_tol);
            if let Some(&(u0, th0, r0)) = pts.first() {
                let p0 = centerline_wcs(rot, u0, r0);
                builder.rotary_rapid_to(DVec3::new(p0.x, p0.y, safe_z_wcs), th0);
                builder.rotary_linear_to(p0, th0, plunge_feed, r0.max(1.0), MotionRole::Plunge);
                for &(u, th, r) in pts.iter().skip(1) {
                    builder.rotary_linear_to(
                        centerline_wcs(rot, u, r),
                        th,
                        cut_feed,
                        r.max(1.0),
                        role,
                    );
                }
                if let Some(&(ul, thl, rl)) = pts.last() {
                    let p_end = centerline_wcs(rot, ul, rl);
                    builder.rotary_rapid_to(DVec3::new(p_end.x, p_end.y, safe_z_wcs), thl);
                }
                builder.pass_count += 1;
            }
        }
    } else {
        // Partial angular sector: axial lanes stepping along u with angular sweeps across [start_deg, end_deg]
        let n_lanes = (((u_end - u_start) / stepover_mm).ceil() as usize).max(1);
        let lane_du = (u_end - u_start) / (n_lanes as f64);
        let n_ang = (((end_deg - start_deg) / d_theta).ceil() as usize).max(4);
        let ang_step = (end_deg - start_deg) / (n_ang as f64);

        let refined_lanes: Vec<Vec<(f64, f64, f64)>> = (0..=n_lanes)
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
                evaluate_and_refine_rotary_strip(
                    base_mesh, rot, cutter, &coords, r_floor, smooth_tol,
                )
            })
            .collect();
        ctx.check_cancel()?;

        let mut r_min_surf = f64::INFINITY;
        let mut r_max_surf = 0.0_f64;
        for lane in &refined_lanes {
            let (lo, hi) = radius_bounds(lane);
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
                let level_pts: Vec<(f64, f64, f64)> = base_lane
                    .iter()
                    .map(|&(u, th, r)| (u, th, r.max(r_ceil)))
                    .collect();
                let pts = decimate_rotary_strip(&level_pts, smooth_tol);
                if let Some(&(u0, th0, r0)) = pts.first() {
                    let p0 = centerline_wcs(rot, u0, r0);
                    let need_retract = lane == 0 || cut_direction != RotaryCutDirection::ZigZag;
                    if need_retract {
                        builder.rotary_rapid_to(DVec3::new(p0.x, p0.y, safe_z_wcs), th0);
                        builder.rotary_linear_to(
                            p0,
                            th0,
                            plunge_feed,
                            r0.max(1.0),
                            MotionRole::Plunge,
                        );
                    } else {
                        builder.rotary_linear_to(
                            p0,
                            th0,
                            cut_feed,
                            r0.max(1.0),
                            MotionRole::LinkTransition,
                        );
                    }
                    for &(uu, th, r) in pts.iter().skip(1) {
                        builder.rotary_linear_to(
                            centerline_wcs(rot, uu, r),
                            th,
                            cut_feed,
                            r.max(1.0),
                            role,
                        );
                    }
                    if cut_direction != RotaryCutDirection::ZigZag
                        && let Some(&(ul, thl, rl)) = pts.last()
                    {
                        let pl = centerline_wcs(rot, ul, rl);
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
fn emit_axial_parallel(
    ctx: &StrategyExecContext<'_>,
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    u_samples: &[f64],
    start_deg: f64,
    end_deg: f64,
    is_full_turn: bool,
    r_outer_est: f64,
    r_floor: f64,
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

    let refined_lanes: Vec<Vec<(f64, f64, f64)>> = (0..lane_count)
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
            evaluate_and_refine_rotary_strip(base_mesh, rot, cutter, &coords, r_floor, smooth_tol)
        })
        .collect();
    ctx.check_cancel()?;

    let mut r_min_surf = f64::INFINITY;
    let mut r_max_surf = 0.0_f64;
    for lane in &refined_lanes {
        let (lo, hi) = radius_bounds(lane);
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

    // Precompute exact gouge-free ZigZag link transitions between adjacent angular lanes
    let link_steps = ((lane_step_deg / 1.5).ceil() as usize).clamp(2, 24);
    let lane_links: Vec<Vec<(f64, f64, f64)>> = (0..lane_count)
        .map(|lane| {
            if lane == 0 || cut_direction != RotaryCutDirection::ZigZag {
                return Vec::new();
            }
            let Some(&(u0, th0, _)) = refined_lanes[lane].first() else {
                return Vec::new();
            };
            let prev_th = start_deg + ((lane - 1) as f64) * lane_step_deg;
            let coords: Vec<(f64, f64)> = (0..=link_steps)
                .map(|s| {
                    let t = (s as f64) / (link_steps as f64);
                    (u0, prev_th + t * (th0 - prev_th))
                })
                .collect();
            evaluate_and_refine_rotary_strip(base_mesh, rot, cutter, &coords, r_floor, smooth_tol)
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
            let level_pts: Vec<(f64, f64, f64)> = base_lane
                .iter()
                .map(|&(u, th, r)| (u, th, r.max(r_ceil)))
                .collect();
            let pts = decimate_rotary_strip(&level_pts, smooth_tol);
            if let Some(&(u0, th0, r0)) = pts.first() {
                let p0 = centerline_wcs(rot, u0, r0);
                if lane == 0 || cut_direction != RotaryCutDirection::ZigZag {
                    builder.rotary_rapid_to(DVec3::new(p0.x, p0.y, safe_z_wcs), th0);
                    builder.rotary_linear_to(p0, th0, plunge_feed, r0.max(1.0), MotionRole::Plunge);
                } else {
                    for &(lu, lth, lr) in lane_links[lane].iter().skip(1) {
                        let r_clamped = lr.max(r_ceil);
                        builder.rotary_linear_to(
                            centerline_wcs(rot, lu, r_clamped),
                            lth,
                            cut_feed,
                            r_clamped.max(1.0),
                            MotionRole::LinkTransition,
                        );
                    }
                }

                for &(u, theta, r) in pts.iter().skip(1) {
                    builder.rotary_linear_to(
                        centerline_wcs(rot, u, r),
                        theta,
                        cut_feed,
                        r.max(1.0),
                        role,
                    );
                }

                if cut_direction != RotaryCutDirection::ZigZag
                    && let Some(&(ul, thl, rl)) = pts.last()
                {
                    let pl = centerline_wcs(rot, ul, rl);
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

/// Conservative chord decimation in `(u_mm, theta_deg, r_mm)` space: preserves all endpoints,
/// drops interior points only when the linear interpolation error in `r` stays within `tol_mm`,
/// never dips below any convex peak/shoulder (`pr - r_interp <= 0.002 mm`), and keeps the angular
/// span between retained vertices `<= 1.5°` on curved profiles (`<= 4.0°` on constant-radius zones).
fn decimate_rotary_strip(pts: &[(f64, f64, f64)], tol_mm: f64) -> Vec<(f64, f64, f64)> {
    if pts.len() <= 2 || tol_mm <= 1e-7 {
        return pts.to_vec();
    }
    let mut out = Vec::with_capacity(pts.len());
    out.push(pts[0]);
    let mut anchor_idx = 0usize;

    for i in 1..(pts.len() - 1) {
        let a = pts[anchor_idx];
        let b = pts[i + 1];
        let max_ang = if (b.2 - a.2).abs() > 1e-3 { 1.5 } else { 4.0 };
        if (b.1 - a.1).abs() > max_ang {
            out.push(pts[i]);
            anchor_idx = i;
            continue;
        }
        // Check max radial deviation of all interior points in (anchor_idx..=i) against segment a->b
        let du = b.0 - a.0;
        let dth = b.1 - a.1;
        let denom = du.hypot(dth);
        let mut exceeds = false;
        for &(pu, pth, pr) in &pts[(anchor_idx + 1)..=i] {
            let t = if denom > 1e-12 {
                (((pu - a.0) * du + (pth - a.1) * dth) / (denom * denom)).clamp(0.0, 1.0)
            } else {
                0.5
            };
            let r_interp = a.2 + t * (b.2 - a.2);
            // Strictly forbid dipping below convex surfaces by more than 2 microns
            if (pr - r_interp) > 0.002 || (r_interp - pr) > tol_mm {
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
