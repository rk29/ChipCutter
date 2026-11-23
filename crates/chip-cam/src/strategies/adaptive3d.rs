//! **3D Adaptive Clearing / Roughing** (`OperationKind::Adaptive3D`).
//!
//! Combines primary coarse roughing Z-levels (`max_roughing_stepdown`), automatic
//! **fine step-up passes** (`fine_stepdown`) peeling terraced slopes between coarse levels
//! with zero air-cutting, exact horizontal **flat-plateau detection**, and Milestone 6's
//! constant-engagement trochoidal wavefront engine ([`generate_adaptive_region_slice`]).

use chip_math::DVec3;

use crate::{
    offset2d::PocketRegion2D,
    params::{Adaptive3DParams, MachiningBoundaryMode, Pocket2DParams},
    rest2d::{RegionSet2D, swept_toolpath_coverage_2d},
    strategies::{
        StrategyExecContext,
        adaptive2d::{AdaptiveSliceSpec, generate_adaptive_region_slice},
    },
    surface3d::{
        PushCutterField, SurfaceCutterProfile, SurfaceMesh3D,
        levels::{detect_flat_plateaus, resolve_3d_containment_region, stock_xy_region},
    },
    toolpath::ToolpathBuilder,
    wcs::CamError,
};

/// Generates **3D Adaptive Roughing** toolpaths across `ctx.doc` (triangle mesh or STEP solid).
pub fn generate_adaptive3d(
    ctx: &StrategyExecContext<'_>,
    params: &Adaptive3DParams,
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
    let cutter =
        SurfaceCutterProfile::from_tool_geometry(&ctx.tool.geometry, radial_stock, axial_stock);
    let tool_d = ctx.tool.geometry.cutting_diameter_mm().max(0.1);
    let tool_r = 0.5 * tool_d;

    let z_top = ctx.heights.top_z_wcs;
    let z_bottom = ctx
        .heights
        .bottom_z_wcs
        .max(surface.aabb_wcs.min.z + axial_stock);
    if z_top <= z_bottom + 1e-4 {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    // Resolve 2D tool-center containment boundary.
    // For `StockBox` roughing, expand as a sharp rectangle past the physical stock edge (`tool_r + 1.5 mm`)
    // so `generate_adaptive_region_slice` has a continuous connected open-air frame on all 4 sides & corners!
    let containment = match params.boundary_mode {
        MachiningBoundaryMode::StockBox => {
            let sa = ctx.setup.stock.aabb_wcs();
            let pad = tool_r + params.boundary_offset.value_mm.max(1.0);
            crate::surface3d::levels::rect_region_2d(
                sa.min.x - pad,
                sa.min.y - pad,
                sa.max.x + pad,
                sa.max.y + pad,
            )
        }
        _ => resolve_3d_containment_region(
            &surface,
            ctx.setup,
            &ctx.selection,
            params.boundary_mode,
            params.tool_containment,
            params.boundary_offset.value_mm,
            tool_r,
        ),
    };
    if containment.is_empty() {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    let stock_outer_region = stock_xy_region(ctx.setup);
    let rough_step = params
        .max_roughing_stepdown
        .value_mm
        .clamp(0.1, ctx.tool.geometry.flute_length_mm().max(1.0));
    let fine_step = params.fine_stepdown.value_mm.clamp(0.0, rough_step);

    // Detect exact flat plateaus inside `[z_bottom, z_top]`
    let flat_targets: Vec<f64> = if params.detect_flats {
        detect_flat_plateaus(
            &surface,
            z_bottom - axial_stock,
            z_top - axial_stock,
            params.flat_area_min_mm2,
        )
        .into_iter()
        .map(|zf| zf + axial_stock)
        .filter(|&zt| zt >= z_bottom - 1e-4 && zt <= z_top - 0.05)
        .collect()
    } else {
        Vec::new()
    };

    // Build primary coarse roughing Z-levels descending from `z_top` down to `z_bottom`
    let coarse_levels = build_coarse_levels(z_top, z_bottom, rough_step, &flat_targets);
    let pocket_defaults = Pocket2DParams::default();
    let retract_z = ctx.safe_retract_z();

    use crate::surface3d::slice_surface_contours_z;
    let push_field = PushCutterField::build(&surface, cutter);
    let mut prev_coarse_z = z_top;
    let mut emitted_any = false;

    let carryover_excess_masks = |td: &chip_stock::TriDexelStock,
                                  slab_lo: f64,
                                  slab_hi: f64|
     -> Option<(RegionSet2D, RegionSet2D)> {
        let raw_polys = td.extract_remaining_material_regions_in_z_slab(slab_lo, slab_hi, 0.0);
        let raw_mat = crate::rest2d::contours_to_region_set(&raw_polys);
        if raw_mat.is_empty() {
            return None;
        }
        // Subtract the target part solid cross-section (+ 1-cell Tri-Dexel quantization halo)
        // so already-finished walls/bosses from Setup 1 are NOT mistaken for uncut excess stock!
        let xy_pitch = td.pitch_mm.x.max(td.pitch_mm.y);
        let halo = radial_stock + 1.05 * xy_pitch;
        let mut part_slices = Vec::new();
        let z_lo_s = (slab_lo - axial_stock)
            .clamp(surface.aabb_wcs.min.z + 1e-4, surface.aabb_wcs.max.z - 1e-4);
        let z_hi_s = (slab_hi - axial_stock)
            .clamp(surface.aabb_wcs.min.z + 1e-4, surface.aabb_wcs.max.z - 1e-4);
        if slab_lo - axial_stock < surface.aabb_wcs.max.z - 1e-4
            && slab_hi - axial_stock > surface.aabb_wcs.min.z + 1e-4
        {
            for z_s in [z_lo_s, 0.5 * (z_lo_s + z_hi_s)] {
                let c = slice_surface_contours_z(&surface, z_s);
                let r = crate::rest2d::contours_to_region_set(&c);
                if !r.is_empty() {
                    part_slices.push(r);
                }
            }
        }
        let excess_mat = if part_slices.is_empty() {
            raw_mat
        } else {
            let part_solid = RegionSet2D::union_many(&mut part_slices).dilate_outward(halo);
            raw_mat.difference(&part_solid)
        };
        if excess_mat.is_empty() || excess_mat.total_area_mm2() <= 0.1 {
            return None;
        }
        let mat_mask = excess_mat.dilate_outward(tool_r + radial_stock + 0.5);
        let tight_mask = excess_mat.dilate_outward(tool_r + 0.05);
        Some((mat_mask, tight_mask))
    };

    for &z_curr in &coarse_levels {
        ctx.check_cancel()?;

        // Compute clearable tool-center region at `z_curr` (monotonically includes full vertical
        // overhang shadow for all part geometry above `z_curr`)
        let avoid_curr = push_field.avoidance_region_z(z_curr);

        // Multi-setup carried-over stock rest filtering: skip completely empty Z slabs
        // and clip candidate roughing regions to remaining uncut excess stock!
        if let Some(td) = ctx.carryover_stock
            && !td.has_material_in_z_slab(z_curr + 0.01, prev_coarse_z + 0.05, 0.02)
        {
            prev_coarse_z = z_curr;
            continue;
        }

        let mut clearable_curr = if avoid_curr.is_empty() {
            containment.clone()
        } else {
            containment.difference(&avoid_curr)
        };

        // Optional Rest Machining from preceding operations
        let mut upstream_air = if !ctx.upstream_rest.is_empty() {
            swept_toolpath_coverage_2d(ctx.upstream_rest, z_curr).erode_inward(tool_r * 0.98)
        } else {
            RegionSet2D::empty()
        };
        if !upstream_air.is_empty() {
            clearable_curr = clearable_curr.difference(&upstream_air);
        }

        // Clip to carried-over remaining excess material mask (inflated by safe entry margin)
        if let Some(td) = ctx.carryover_stock {
            let slab_lo = z_curr + 0.01;
            let slab_hi = prev_coarse_z + 0.05;
            if let Some((mat_mask, tight_mask)) = carryover_excess_masks(td, slab_lo, slab_hi) {
                let carryover_air = clearable_curr.difference(&tight_mask);
                if !carryover_air.is_empty() {
                    upstream_air = upstream_air.union(&carryover_air);
                }
                clearable_curr = clearable_curr.intersect(&mat_mask);
            } else {
                clearable_curr = RegionSet2D::empty();
            }
        }

        if !clearable_curr.is_empty() && clearable_curr.total_area_mm2() > 0.08 {
            clear_region_set_adaptive(
                ctx,
                &clearable_curr,
                &upstream_air,
                &stock_outer_region,
                &pocket_defaults,
                params,
                prev_coarse_z,
                z_curr,
                builder,
            )?;
            emitted_any = true;
        }

        // Fine step-up passes climbing the terraced slope inside `(z_curr, prev_coarse_z)`
        // Executed top-down from `prev_coarse_z` toward `z_curr` using `clearable_curr` as
        // already-cleared open air (`cleared_air_centers`) so axial load stays `<= fine_step`
        // and ZERO air-cutting occurs over already-cleared stock!
        let band_h = prev_coarse_z - z_curr;
        if fine_step > 0.05 && band_h > fine_step * 1.15 {
            let mut step_ups =
                build_fine_stepup_levels(prev_coarse_z, z_curr, fine_step, &flat_targets);
            step_ups.sort_by(|a, b| b.total_cmp(a));

            let mut step_prev_z = prev_coarse_z;
            let cleared_base = clearable_curr.union(&upstream_air);

            for z_up in step_ups {
                ctx.check_cancel()?;
                if let Some(td) = ctx.carryover_stock
                    && !td.has_material_in_z_slab(z_up + 0.01, step_prev_z + 0.05, 0.02)
                {
                    step_prev_z = z_up;
                    continue;
                }
                let avoid_up = push_field.avoidance_region_z(z_up);
                let mut clearable_up = if avoid_up.is_empty() {
                    containment.clone()
                } else {
                    containment.difference(&avoid_up)
                };
                if let Some(td) = ctx.carryover_stock {
                    if let Some((mat_mask, _)) =
                        carryover_excess_masks(td, z_up + 0.01, step_prev_z + 0.05)
                    {
                        clearable_up = clearable_up.intersect(&mat_mask);
                    } else {
                        clearable_up = RegionSet2D::empty();
                    }
                }
                let newly_exposed = clearable_up.difference(&cleared_base);
                if newly_exposed.is_empty() || newly_exposed.total_area_mm2() <= 0.08 {
                    step_prev_z = z_up;
                    continue;
                }

                // Retract safely before climbing up into a higher terrace band
                if let Some(cur) = builder.current_pos()
                    && cur.z < z_up - 1e-4
                {
                    let safe_z = retract_z.max(z_up + 1.0);
                    builder.rapid_to(DVec3::new(cur.x, cur.y, safe_z));
                }

                clear_region_set_adaptive(
                    ctx,
                    &clearable_up,
                    &cleared_base,
                    &stock_outer_region,
                    &pocket_defaults,
                    params,
                    step_prev_z,
                    z_up,
                    builder,
                )?;
                emitted_any = true;
                step_prev_z = z_up;
            }
        }

        prev_coarse_z = z_curr;
    }

    if !emitted_any {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    if let Some(cur) = builder.current_pos()
        && cur.z < retract_z - 1e-4
    {
        builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn clear_region_set_adaptive(
    ctx: &StrategyExecContext<'_>,
    clearable: &RegionSet2D,
    cleared_air: &RegionSet2D,
    stock_outer_region: &RegionSet2D,
    pocket_defaults: &Pocket2DParams,
    params: &Adaptive3DParams,
    prev_level_z: f64,
    z_cut: f64,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    // Process largest components first for stable deterministic ordering
    let mut regions: Vec<&PocketRegion2D> = clearable.regions.iter().collect();
    regions.sort_by(|a, b| b.net_area_mm2().total_cmp(&a.net_area_mm2()));

    for sub_reg in regions {
        if sub_reg.net_area_mm2() <= 0.08 {
            continue;
        }
        // Does this region reach outside the physical stock XY footprint (open-air entry)?
        let sub_set = RegionSet2D::from_single(sub_reg.clone());
        let has_open_air = !sub_set.difference(stock_outer_region).is_empty();

        let spec = AdaptiveSliceSpec {
            sub_reg,
            finish_reg: None,
            raw_floor_outer_ccw: &sub_reg.outer_ccw,
            has_open_air,
            cleared_air_centers: cleared_air,
            pocket_params: pocket_defaults,
            adaptive_params: &params.adaptive,
            prev_level_z,
            z_cut,
        };
        generate_adaptive_region_slice(ctx, &spec, builder)?;
    }
    Ok(())
}

fn build_coarse_levels(
    z_top: f64,
    z_bottom: f64,
    rough_step: f64,
    flat_targets: &[f64],
) -> Vec<f64> {
    let span = (z_top - z_bottom).max(0.0);
    let n = (span / rough_step).ceil().max(1.0) as usize;
    let mut levels = Vec::with_capacity(n + flat_targets.len());
    for k in 1..=n {
        levels.push(z_top - (k as f64 / n as f64) * span);
    }
    // Inject significant flat plateaus directly into primary Z levels, snapping accurately within 0.005 mm
    for &zf in flat_targets {
        if zf < z_top - 0.005 && zf > z_bottom + 0.005 {
            if let Some(existing) = levels.iter_mut().find(|z| (**z - zf).abs() <= 0.25) {
                *existing = zf;
            } else {
                levels.push(zf);
            }
        }
    }
    levels.sort_by(|a, b| b.total_cmp(a));
    levels.dedup_by(|a, b| (*a - *b).abs() <= 0.005);
    levels
}

fn build_fine_stepup_levels(
    z_hi: f64,
    z_lo: f64,
    fine_step: f64,
    flat_targets: &[f64],
) -> Vec<f64> {
    let span = z_hi - z_lo;
    let n = (span / fine_step).ceil().max(1.0) as usize;
    let mut levels = Vec::new();
    for k in 1..n {
        let z = z_hi - (k as f64 / n as f64) * span;
        if z < z_hi - 0.005 && z > z_lo + 0.005 {
            levels.push(z);
        }
    }
    let snap_tol = (0.25 * fine_step).clamp(0.05, 0.25);
    for &zf in flat_targets {
        if zf < z_hi - 0.005 && zf > z_lo + 0.005 {
            if let Some(existing) = levels.iter_mut().find(|z| (**z - zf).abs() <= snap_tol) {
                *existing = zf;
            } else {
                levels.push(zf);
            }
        }
    }
    levels.sort_by(|a, b| b.total_cmp(a));
    levels.dedup_by(|a, b| (*a - *b).abs() <= 0.005);
    levels
}
