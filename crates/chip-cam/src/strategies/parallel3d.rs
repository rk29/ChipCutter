//! **3D Parallel / Raster Finishing** (`OperationKind::Parallel3D`).
//!
//! Generates exact `f64` Drop-Cutter parallel scanline finishing passes across imported triangle
//! meshes (`STL`/`OBJ`/`3MF`) and tessellated STEP B-Rep solids, supporting configurable pass
//! angle, optional perpendicular cross-pass (`+90°`), cusp-height stepover calculation, slope-range
//! filtering (`0°..90°`), 3D Rest Machining boundaries, 3D Ramer–Douglas–Peucker chord decimation,
//! and collision-free surface-hugging linking.

use chip_math::{DVec2, DVec3};
use rayon::prelude::*;

use crate::{
    cl_data::MotionRole,
    params::{Parallel3DParams, ParallelCutDirection},
    rest2d::RegionSet2D,
    strategies::StrategyExecContext,
    surface3d::{
        RestMaskGrid2D, SurfaceCutterProfile, SurfaceLinkConfig, SurfaceMesh3D, build_3d_rest_mask,
        drop_cutter_z_with_scratch, emit_final_surface_retract, emit_surface_safe_link,
        finishing::{simplify_3d_polyline_gouge_safe, split_chains_at_overhang_cliffs},
        levels::{resolve_3d_boundary_config_detailed, sample_surface_slope_deg},
        refine_convex_chords_3d, region_aabb_2d, stepover_for_cusp_height,
    },
    toolpath::ToolpathBuilder,
    wcs::CamError,
};

/// Generates **3D Parallel / Raster Finishing** toolpaths across `ctx.doc`.
pub fn generate_parallel3d(
    ctx: &StrategyExecContext<'_>,
    params: &Parallel3DParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    ctx.check_cancel()?;
    if !ctx.doc.has_3d_geometry() {
        return Err(CamError::EmptyCadDocument);
    }

    let surface = SurfaceMesh3D::from_cad_document(ctx.doc, &[], &ctx.setup.frame);
    if surface.triangles.is_empty() || !surface.aabb_wcs.is_valid() {
        return Err(CamError::EmptyCadDocument);
    }

    let radial_stock = ctx.passes.stock_to_leave_radial.value_mm.max(0.0);
    let axial_stock = ctx.passes.stock_to_leave_axial.value_mm.max(0.0);
    let cutter = SurfaceCutterProfile::from_tool(ctx.tool, radial_stock, axial_stock, 0.5);
    let tool_d = ctx.tool.geometry.cutting_diameter_mm().max(0.1);
    let tool_r = 0.5 * tool_d;

    let z_bottom = ctx
        .heights
        .bottom_z_wcs
        .max(surface.aabb_wcs.min.z + axial_stock);
    let z_top = ctx
        .heights
        .top_z_wcs
        .min(surface.aabb_wcs.max.z + axial_stock);
    if z_top < z_bottom - 1e-4 {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    let resolved_boundary = resolve_3d_boundary_config_detailed(
        &surface,
        ctx.setup,
        &ctx.selection,
        &params.boundary,
        tool_r,
    );
    builder.warnings.extend(resolved_boundary.warnings);
    let containment = resolved_boundary.effective_region;
    let Some(bounds) = region_aabb_2d(&containment) else {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    };

    let rest_mask = build_3d_rest_mask(
        &surface,
        &cutter,
        &containment,
        &ctx.passes.rest_machining,
        ctx.upstream_rest,
        z_bottom,
    );

    let stepover = if params.use_cusp_height {
        stepover_for_cusp_height(
            &cutter,
            params.cusp_height.value_mm,
            params.stepover.value_mm,
        )
    } else {
        params.stepover.value_mm.clamp(0.05, 0.75 * tool_d)
    };
    let adaptive_sample_cap = (0.06 * tool_d).clamp(0.15, 0.25);
    let sample_step = params
        .sample_step
        .value_mm
        .clamp(0.08, 1.0)
        .min(adaptive_sample_cap);
    let smooth_tol = params.smoothing_tolerance.value_mm.clamp(0.001, 0.010);

    let slope_min = params.min_slope_deg.clamp(0.0, 90.0);
    let slope_max = params.max_slope_deg.clamp(slope_min, 90.0);
    let filter_slopes = slope_min > 0.1 || slope_max < 89.9;

    let mut angles_deg = vec![params.pass_angle_deg];
    if params.cross_pass {
        angles_deg.push(params.pass_angle_deg + 90.0);
    }

    let retract_z = ctx.safe_retract_z();
    let stock_top_z = ctx.setup.stock.aabb_wcs().max.z;
    let cut_feed = ctx.cutting.cutting_feed_mm_min;
    let lead_feed = ctx.cutting.lead_feed_mm_min.max(0.4 * cut_feed);
    let plunge_feed = ctx.cutting.plunge_feed_mm_min.max(0.25 * cut_feed);
    let stay_down_limit = ctx
        .linking
        .safe_stay_down_distance
        .value_mm
        .clamp(stepover * 1.5, (3.0 * tool_d).max(stepover * 2.5));

    let link_cfg = SurfaceLinkConfig {
        retract_z,
        stock_top_z,
        z_floor: z_bottom,
        stay_down_limit_mm: stay_down_limit,
        sample_step_mm: sample_step,
        lead_feed,
        plunge_feed,
    };

    let corners = [
        DVec2::new(bounds.min.x, bounds.min.y),
        DVec2::new(bounds.max.x, bounds.min.y),
        DVec2::new(bounds.max.x, bounds.max.y),
        DVec2::new(bounds.min.x, bounds.max.y),
    ];

    let mut bvh_scratch = Vec::with_capacity(64);
    let mut emitted_passes = 0usize;

    for angle_deg in angles_deg {
        ctx.check_cancel()?;
        let rad = angle_deg.to_radians();
        let u_dir = DVec2::new(rad.cos(), rad.sin());
        let v_dir = DVec2::new(-rad.sin(), rad.cos());

        let mut u_min = f64::INFINITY;
        let mut u_max = f64::NEG_INFINITY;
        let mut v_min = f64::INFINITY;
        let mut v_max = f64::NEG_INFINITY;
        for c in corners {
            let u = c.dot(u_dir);
            let v = c.dot(v_dir);
            u_min = u_min.min(u);
            u_max = u_max.max(u);
            v_min = v_min.min(v);
            v_max = v_max.max(v);
        }

        let span_v = (v_max - v_min).max(0.0);
        let span_u = (u_max - u_min).max(0.0);
        if span_u <= 1e-4 {
            continue;
        }

        let n_lines = ((span_v / stepover).floor() as usize) + 1;
        let v_used = (n_lines.saturating_sub(1)) as f64 * stepover;
        let v_start = v_min + 0.5 * (span_v - v_used).max(0.0);
        let n_samples = ((span_u / sample_step).ceil() as usize).max(2);

        let line_v_coords: Vec<(usize, f64)> = (0..n_lines)
            .map(|i| (i, v_start + i as f64 * stepover))
            .collect();

        let line_chains: Vec<(usize, Vec<Vec<DVec3>>)> = line_v_coords
            .par_iter()
            .map(|&(line_idx, v)| {
                let chains = sample_parallel_scanline(
                    &surface,
                    &cutter,
                    &containment,
                    rest_mask.as_ref(),
                    u_dir,
                    v_dir,
                    u_min,
                    u_max,
                    v,
                    n_samples,
                    z_bottom,
                    z_top,
                    filter_slopes,
                    slope_min,
                    slope_max,
                    smooth_tol,
                );
                (line_idx, chains)
            })
            .collect();

        for (line_idx, mut chains) in line_chains {
            ctx.check_cancel()?;
            if chains.is_empty() {
                continue;
            }
            let reverse_line = match params.cut_direction {
                ParallelCutDirection::ZigZag => line_idx % 2 == 1,
                ParallelCutDirection::OneWayClimb => false,
                ParallelCutDirection::OneWayConventional => true,
            };
            if reverse_line {
                chains.reverse();
                for c in &mut chains {
                    c.reverse();
                }
            }

            for chain in chains {
                for sub_chain in split_chains_at_overhang_cliffs(&surface, &cutter, chain) {
                    emit_surface_safe_link(
                        &surface,
                        &cutter,
                        sub_chain[0],
                        &link_cfg,
                        &mut bvh_scratch,
                        builder,
                    );
                    for &pt in &sub_chain[1..] {
                        builder.linear_to(pt, cut_feed, MotionRole::FinishCut);
                    }
                    builder.pass_count += 1;
                    emitted_passes += 1;
                }
            }
        }
    }

    if emitted_passes == 0 {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    emit_final_surface_retract(retract_z, lead_feed, builder);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn sample_parallel_scanline(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    containment: &RegionSet2D,
    rest_mask: Option<&RestMaskGrid2D>,
    u_dir: DVec2,
    v_dir: DVec2,
    u_min: f64,
    u_max: f64,
    v: f64,
    n_samples: usize,
    z_bottom: f64,
    z_top: f64,
    filter_slopes: bool,
    slope_min: f64,
    slope_max: f64,
    smooth_tol: f64,
) -> Vec<Vec<DVec3>> {
    let mut scratch = Vec::with_capacity(64);
    let mut chains: Vec<Vec<DVec3>> = Vec::new();
    let mut current: Vec<DVec3> = Vec::new();

    for k in 0..=n_samples {
        let t = k as f64 / n_samples as f64;
        let u = u_min + (u_max - u_min) * t;
        let pt_xy = u_dir * u + v_dir * v;

        let in_region = containment.contains_point(pt_xy)
            || containment.min_distance_to_boundary(pt_xy) <= 1e-3;
        let in_rest = rest_mask.is_none_or(|rm| rm.contains_xy(pt_xy));

        let valid_z = if in_region && in_rest {
            drop_cutter_z_with_scratch(surface, cutter, pt_xy.x, pt_xy.y, z_bottom, &mut scratch)
                .filter(|&z| z <= z_top + 1e-3)
                .and_then(|z| {
                    if filter_slopes {
                        let s_deg = sample_surface_slope_deg(
                            surface,
                            cutter,
                            pt_xy,
                            z_bottom,
                            &mut scratch,
                        );
                        if s_deg >= slope_min - 1.0 && s_deg <= slope_max + 1.0 {
                            Some(z)
                        } else {
                            None
                        }
                    } else {
                        Some(z)
                    }
                })
        } else {
            None
        };

        if let Some(z) = valid_z {
            current.push(DVec3::new(pt_xy.x, pt_xy.y, z));
        } else if current.len() >= 2 {
            chains.push(std::mem::take(&mut current));
        } else {
            current.clear();
        }
    }
    if current.len() >= 2 {
        chains.push(current);
    }

    chains
        .into_iter()
        .filter_map(|raw| {
            let refined =
                refine_convex_chords_3d(surface, cutter, z_bottom, z_top, &raw, smooth_tol);
            let simplified =
                simplify_3d_polyline_gouge_safe(surface, cutter, z_bottom, &refined, smooth_tol);
            if simplified.len() >= 2 {
                Some(simplified)
            } else {
                None
            }
        })
        .collect()
}
