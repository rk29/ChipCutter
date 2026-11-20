//! 2-Linear + Rotary (`Z+X+A` / `Z+Y+B`) 3D Surfacing Strategy (`OperationKind::Rotary2Linear`).
//!
//! Strictly locks the perpendicular cross-axis (`Y = center.y` for `X/A`, or `X = center.x` for `Y/B`)
//! while computing exact `f64` Drop-Cutter radial contact heights across the 3D workpiece mesh at
//! every angular station in parallel (`rayon`), generating crash-free continuous spiral or axial-parallel
//! rotary toolpaths with multi-depth radial roughing.

use chip_cad::{MeshBvh, MeshTriangle};
use chip_math::{Aabb3, DVec3};
use rayon::prelude::*;

use crate::{
    cl_data::MotionRole,
    params::{Rotary2LinearParams, RotaryCutDirection, RotaryPatternMode},
    rotary::{ResolvedRotaryConfig, RotaryAxisKind},
    strategies::{StrategyExecContext, compute_z_levels},
    surface3d::{
        SurfaceCutterProfile, SurfaceMesh3D, drop_cutter_z_with_scratch, stepover_for_cusp_height,
    },
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

    // Determine axial span [u_start, u_end] trimmed by margins, clamped to model & clear of chuck
    let (_, _, u_vec) = rot.axis.basis();
    let mut mesh_u_min = f64::INFINITY;
    let mut mesh_u_max = f64::NEG_INFINITY;
    for &v in &base_mesh.vertices_wcs {
        let u = v.dot(u_vec);
        mesh_u_min = mesh_u_min.min(u);
        mesh_u_max = mesh_u_max.max(u);
    }

    let tool_d = ctx.tool.geometry.cutting_diameter_mm().max(0.1);
    let tool_r = 0.5 * tool_d;
    let chuck_guard = tool_r + 0.5;

    let mut u_start = rot.stock_u_min + params.axial_margin_start.value_mm.max(0.0);
    let mut u_end = rot.stock_u_max - params.axial_margin_end.value_mm.max(0.0);
    if mesh_u_min.is_finite() && mesh_u_max.is_finite() {
        u_start = u_start.max(mesh_u_min);
        u_end = u_end.min(mesh_u_max);
    }
    if rot.chuck_end_positive {
        u_end = u_end.min(rot.chuck_u_min - chuck_guard);
    } else {
        u_start = u_start.max(rot.chuck_u_max + chuck_guard);
    }

    if !u_start.is_finite() || !u_end.is_finite() || u_end <= u_start + 1e-3 {
        return Err(CamError::InvalidRotarySetup {
            op_name: ctx.op.name.clone(),
            reason: format!(
                "Axial machining window [{u_start:.2}, {u_end:.2}] mm is empty after chuck clearance and margins"
            ),
        });
    }

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

    // Sampling grid dimensions: enforce fine internal axial & angular resolution so convex
    // bevel fillets and multi-lobe cams never exhibit chord-sagitta gouges; flat regions are
    // subsequently compacted cleanly by `decimate_rotary_strip`.
    let axial_sample = params.axial_sample_step.value_mm.clamp(0.1, 0.35);
    let n_u = (((u_end - u_start) / axial_sample).ceil() as usize).clamp(4, 2_000);
    let du = (u_end - u_start) / (n_u as f64);
    let u_samples: Vec<f64> = (0..=n_u).map(|i| u_start + (i as f64) * du).collect();

    let d_theta_req = params.angular_sample_deg.clamp(0.5, 2.5);
    let n_theta = ((angular_span_deg / d_theta_req).ceil() as usize).clamp(8, 720);
    let d_theta = angular_span_deg / (n_theta as f64);
    let theta_count = if is_full_turn { n_theta } else { n_theta + 1 };

    let cutter = SurfaceCutterProfile::from_tool_geometry(
        &ctx.tool.geometry,
        ctx.passes.stock_to_leave_radial.value_mm,
        ctx.passes.stock_to_leave_axial.value_mm,
    );
    let r_floor = (ctx.heights.bottom_z_wcs - rot.center_wcs.z).max(0.5);
    let z_floor = rot.center_wcs.z + r_floor;

    let r_max = cutter.max_radius_mm();

    // Evaluate exact f64 rotary drop-cutter heights in parallel across angular stations
    let r_slices: Vec<Vec<f64>> = (0..theta_count)
        .into_par_iter()
        .map(|it| {
            let theta_deg = start_deg + (it as f64) * d_theta;
            let rot_mesh = build_rotated_strip_mesh(
                &base_mesh, &rot, theta_deg, u_start, u_end, r_max, z_floor,
            );
            let mut scratch = Vec::with_capacity(64);
            let mut row = Vec::with_capacity(u_samples.len());
            for &u_val in &u_samples {
                let (qx, qy) = match rot.axis {
                    RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => (u_val, rot.center_wcs.y),
                    RotaryAxisKind::YAxis => (rot.center_wcs.x, u_val),
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
    for row in &r_slices {
        for &r in row {
            r_min_surf = r_min_surf.min(r);
            r_max_surf = r_max_surf.max(r);
        }
    }
    if !r_min_surf.is_finite() {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    let grid = RotaryRadiusGrid {
        u_start,
        du,
        n_u,
        start_deg,
        d_theta,
        n_theta,
        is_full_turn,
        r_slices,
    };

    // Effective stepover (either explicit mm or cusp-height derived)
    let stepover_mm = if params.use_cusp_height {
        let cusp = params.cusp_height.value_mm.clamp(0.002, 2.0);
        stepover_for_cusp_height(&cutter, cusp, params.stepover.value_mm.max(0.05))
            .clamp(0.1, tool_d * 0.75)
    } else {
        params.stepover.value_mm.max(0.05)
    };

    // Multi-depth radial roughing envelope levels
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
        RotaryPatternMode::ContinuousSpiral => {
            emit_continuous_spiral(
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
        RotaryPatternMode::AxialParallel => {
            emit_axial_parallel(
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

struct RotaryRadiusGrid {
    u_start: f64,
    du: f64,
    n_u: usize,
    start_deg: f64,
    d_theta: f64,
    n_theta: usize,
    is_full_turn: bool,
    r_slices: Vec<Vec<f64>>,
}

impl RotaryRadiusGrid {
    /// Shoulder-safe lookup of the required cutter tip radius at `(u, theta_deg)`.
    /// Uses bilinear interpolation on smooth regions and lifts to the maximum of the 4 surrounding
    /// grid corners across sharp axial shoulders/flutes so diagonal chords cannot clip steps.
    fn sample_safe(&self, u: f64, theta_deg: f64) -> f64 {
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
            let max_idx = self.r_slices.len().saturating_sub(1);
            let ft = ((theta_deg - self.start_deg) / self.d_theta).clamp(0.0, max_idx as f64);
            let i0 = (ft.floor() as usize).min(max_idx);
            let i1 = (i0 + 1).min(max_idx);
            (i0, i1, (ft - i0 as f64).clamp(0.0, 1.0))
        };

        let r00 = self.r_slices[it0][iu0];
        let r01 = self.r_slices[it0][iu1];
        let r10 = self.r_slices[it1][iu0];
        let r11 = self.r_slices[it1][iu1];

        let r0 = r00 + (r01 - r00) * tu;
        let r1 = r10 + (r11 - r10) * tu;
        let r_bi = r0 + (r1 - r0) * tt;
        let r_max = r00.max(r01).max(r10).max(r11);

        if r_max - r_bi > 0.015 { r_max } else { r_bi }
    }
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

/// Builds a rotated [`SurfaceMesh3D`] pre-filtered to ONLY triangles intersecting the exact
/// centerline cutter strip (`|cross - center| <= r_max`, `u ∈ [u_start - r_max, u_end + r_max]`,
/// `max.z >= z_floor`), eliminating ~85% of unreachable triangles prior to BVH construction with
/// zero approximation error.
fn build_rotated_strip_mesh(
    base: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    theta_deg: f64,
    u_start: f64,
    u_end: f64,
    r_max: f64,
    z_floor: f64,
) -> SurfaceMesh3D {
    let vertices_wcs: Vec<DVec3> = base
        .vertices_wcs
        .iter()
        .map(|&v| rot.rotate_point_wcs(v, theta_deg))
        .collect();

    let pad = r_max + 1e-4;
    let u_lo = u_start - pad;
    let u_hi = u_end + pad;
    let z_lo = z_floor - 1e-4;

    let mut aabb_wcs = Aabb3::EMPTY;
    let mut triangles = Vec::with_capacity(base.triangles.len() / 4);
    for tri in &base.triangles {
        let [i0, i1, i2] = tri.indices;
        let v0 = vertices_wcs[i0 as usize];
        let v1 = vertices_wcs[i1 as usize];
        let v2 = vertices_wcs[i2 as usize];
        let tri_min = v0.min(v1).min(v2);
        let tri_max = v0.max(v1).max(v2);
        if tri_max.z < z_lo {
            continue;
        }
        let overlaps_strip = match rot.axis {
            RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => {
                tri_max.x >= u_lo
                    && tri_min.x <= u_hi
                    && tri_max.y >= rot.center_wcs.y - pad
                    && tri_min.y <= rot.center_wcs.y + pad
            }
            RotaryAxisKind::YAxis => {
                tri_max.y >= u_lo
                    && tri_min.y <= u_hi
                    && tri_max.x >= rot.center_wcs.x - pad
                    && tri_min.x <= rot.center_wcs.x + pad
            }
        };
        if !overlaps_strip {
            continue;
        }
        let n = (v1 - v0).cross(v2 - v0).normalize_or_zero();
        let tri_aabb = Aabb3::new(tri_min, tri_max);
        aabb_wcs = aabb_wcs.union(&tri_aabb);
        triangles.push(MeshTriangle {
            indices: tri.indices,
            normal: n,
            aabb: tri_aabb,
        });
    }
    let bvh = MeshBvh::build(&triangles);
    SurfaceMesh3D {
        vertices_wcs,
        triangles,
        aabb_wcs,
        bvh,
    }
}

#[allow(clippy::too_many_arguments)]
fn emit_continuous_spiral(
    ctx: &StrategyExecContext<'_>,
    rot: &ResolvedRotaryConfig,
    grid: &RotaryRadiusGrid,
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

            // Continuous multi-turn spiral with full 360° cleanup rings at both ends
            let mut raw_pts: Vec<(f64, f64, f64)> =
                Vec::with_capacity((n_turns as usize + 3) * steps_per_rev);

            // 1. Start cleanup ring at u_start: [level_start_deg .. level_start_deg + 360]
            for k in 0..=steps_per_rev {
                let th = level_start_deg + (k as f64) * rev_step_deg;
                let r = grid.sample_safe(u_start, th).max(r_ceil);
                raw_pts.push((u_start, th, r));
            }

            // 2. Helical body: u_start -> u_end over n_turns revolutions
            let helix_steps =
                ((n_turns * (steps_per_rev as f64)).round() as usize).max(steps_per_rev);
            let th_helix_start = level_start_deg + 360.0;
            let th_helix_span = n_turns * 360.0;
            for k in 1..=helix_steps {
                let t = (k as f64) / (helix_steps as f64);
                let u = u_start + t * (u_end - u_start);
                let th = th_helix_start + t * th_helix_span;
                let r = grid.sample_safe(u, th).max(r_ceil);
                raw_pts.push((u, th, r));
            }

            // 3. End cleanup ring at u_end: extra 360°
            let th_end_start = th_helix_start + th_helix_span;
            for k in 1..=steps_per_rev {
                let th = th_end_start + (k as f64) * rev_step_deg;
                let r = grid.sample_safe(u_end, th).max(r_ceil);
                raw_pts.push((u_end, th, r));
            }
            continuous_theta = th_end_start + 360.0;

            let pts = decimate_rotary_strip(&raw_pts, smooth_tol);
            if let Some(&(u0, th0, r0)) = pts.first() {
                let entry_safe = DVec3::new(
                    centerline_wcs(rot, u0, r0).x,
                    centerline_wcs(rot, u0, r0).y,
                    safe_z_wcs,
                );
                builder.rotary_rapid_to(entry_safe, th0);
                builder.rotary_linear_to(
                    centerline_wcs(rot, u0, r0),
                    th0,
                    plunge_feed,
                    r0.max(1.0),
                    MotionRole::Plunge,
                );
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
        } else {
            // Partial angular sector: axial lanes stepping along u with angular sweeps across [start_deg, end_deg]
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
                    let r = grid.sample_safe(u, th).max(r_ceil);
                    raw_pts.push((u, th, r));
                }
                let pts = decimate_rotary_strip(&raw_pts, smooth_tol);
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
                        // Safe axial link at the same angle
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
    rot: &ResolvedRotaryConfig,
    grid: &RotaryRadiusGrid,
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
                    let r = grid.sample_safe(u, th).max(r_ceil);
                    raw_pts.push((u, th, r));
                }
            } else {
                for &u in u_samples {
                    let r = grid.sample_safe(u, th).max(r_ceil);
                    raw_pts.push((u, th, r));
                }
            }

            let pts = decimate_rotary_strip(&raw_pts, smooth_tol);
            if let Some(&(u0, th0, r0)) = pts.first() {
                let p0 = centerline_wcs(rot, u0, r0);
                if lane == 0 || cut_direction != RotaryCutDirection::ZigZag {
                    builder.rotary_rapid_to(DVec3::new(p0.x, p0.y, safe_z_wcs), th0);
                    builder.rotary_linear_to(p0, th0, plunge_feed, r0.max(1.0), MotionRole::Plunge);
                } else {
                    // Surface-following angular index link at u0 from previous angle to th0
                    let prev_th = start_deg + ((lane - 1) as f64) * lane_step_deg;
                    let link_steps = ((lane_step_deg / 2.0).ceil() as usize).clamp(2, 16);
                    for s in 1..=link_steps {
                        let t = (s as f64) / (link_steps as f64);
                        let th_mid = prev_th + t * (th0 - prev_th);
                        let r_mid = grid.sample_safe(u0, th_mid).max(r_ceil);
                        builder.rotary_linear_to(
                            centerline_wcs(rot, u0, r_mid),
                            th_mid,
                            cut_feed,
                            r_mid.max(1.0),
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
/// never dips below any convex peak/shoulder (`pr - r_interp <= 0.003 mm`), and keeps the angular
/// span between retained vertices `<= 2.5°` on curved profiles (`<= 6.0°` on constant-radius zones).
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
        let max_ang = if (b.2 - a.2).abs() > 1e-3 { 2.5 } else { 6.0 };
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
            // Strictly forbid dipping below convex surfaces by more than 3 microns
            if (pr - r_interp) > 0.003 || (r_interp - pr) > tol_mm {
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
