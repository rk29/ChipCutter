//! **3D Lollipop Undercut Finishing** (`OperationKind::Undercut3D`).
//!
//! Identifies overhanging downward-facing 3D surfaces (`n_z <= -sin(min_overhang_angle)`) on the
//! target `TriMesh3D` (`FlatBvh`) within the active Z height band and optional 2D containment
//! boundary, slices horizontal Z-levels, solves exact `f64` `LollipopEndmill` upper-hemisphere
//! tangential contact (`p_center = p_surf + (R + stock_to_leave) * n_surf`), verifies relieved-neck
//! (`r_neck + neck_clearance`), shank (`r_shank + neck_clearance`), and sphere head clearance
//! against all overhanging geometry via `FlatBvh` (trimming unreachable deep recesses cleanly),
//! and emits verified horizontal open-air `lead_out_distance` pull-aways before every `+Z` retract.

use std::collections::HashMap;

use chip_bvh::closest_point_on_triangle;
use chip_math::{Aabb3, DVec2, DVec3};
use chip_tools::CutterGeometry;

use crate::{
    cl_data::MotionRole,
    params::Undercut3DParams,
    rest2d::RegionSet2D,
    strategies::StrategyExecContext,
    surface3d::{SurfaceMesh3D, resolve_3d_boundary_config_detailed},
    toolpath::ToolpathBuilder,
    wcs::CamError,
};

#[derive(Debug, Clone, Copy)]
struct OverhangStation {
    tip_wcs: DVec3,
    outward_xy: DVec2,
}

#[derive(Debug, Clone)]
struct LollipopSpecs {
    sphere_r: f64,
    eff_r: f64,
    neck_r: f64,
    shank_r: f64,
    z_join: f64,
    z_neck_top: f64,
    z_stickout: f64,
    neck_clearance: f64,
}

impl LollipopSpecs {
    /// Minimum horizontal normal magnitude `hypot(n_x, n_y)` reachable on the upper spherical
    /// hemisphere before hitting the relieved neck join (`r_contact >= r_neck`).
    #[inline]
    fn min_contact_nxy(&self) -> f64 {
        (self.neck_r / self.sphere_r.max(1e-6)).clamp(0.0, 0.999)
    }

    /// Exact upper-hemisphere tangential contact solution for surface point `p_surf` and outward
    /// unit normal `n_surf` (`n_surf.z < 0`).
    #[inline]
    fn solve_tip(&self, p_surf: DVec3, n_surf: DVec3) -> Option<OverhangStation> {
        let n = n_surf.normalize_or_zero();
        if n.length_squared() < 0.5 || n.z >= -1e-4 {
            return None;
        }
        let n_xy_len = n.x.hypot(n.y);
        if n_xy_len + 1e-4 < self.min_contact_nxy() || n_xy_len <= 1e-5 {
            return None;
        }
        let center = p_surf + n * self.eff_r;
        let tip = DVec3::new(center.x, center.y, center.z - self.sphere_r);
        if !tip.is_finite() {
            return None;
        }
        Some(OverhangStation {
            tip_wcs: tip,
            outward_xy: DVec2::new(n.x / n_xy_len, n.y / n_xy_len),
        })
    }
}

/// Generates a **3D Lollipop Undercut** contour finishing toolpath (`OperationKind::Undercut3D`).
pub fn generate_undercut3d(
    ctx: &StrategyExecContext<'_>,
    params: &Undercut3DParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    ctx.check_cancel()?;

    // 1. Validate LollipopEndmill geometry
    let CutterGeometry::LollipopEndmill {
        diameter_mm,
        neck_diameter_mm,
        neck_length_mm,
        shank_diameter_mm,
        ..
    } = ctx.tool.geometry
    else {
        return Err(CamError::IncompatibleTool {
            op_name: ctx.op.name.clone(),
            op_kind: ctx.op.kind.label(),
            tool_name: ctx.tool.name.clone(),
            tool_kind: ctx.tool.geometry.kind_label(),
        });
    };

    // 2. Validate numeric parameters
    if !params.use_cusp_height
        && (!params.z_stepdown.value_mm.is_finite() || params.z_stepdown.value_mm <= 1e-4)
    {
        return Err(CamError::InvalidParameter {
            param: "z_stepdown",
            value: params.z_stepdown.value_mm,
            reason: "must be finite and > 0",
        });
    }
    if params.use_cusp_height
        && (!params.target_cusp.value_mm.is_finite() || params.target_cusp.value_mm <= 1e-5)
    {
        return Err(CamError::InvalidParameter {
            param: "target_cusp",
            value: params.target_cusp.value_mm,
            reason: "must be finite and > 0",
        });
    }
    if !params.min_overhang_angle_deg.is_finite()
        || params.min_overhang_angle_deg <= 0.0
        || params.min_overhang_angle_deg >= 89.0
    {
        return Err(CamError::InvalidParameter {
            param: "min_overhang_angle_deg",
            value: params.min_overhang_angle_deg,
            reason: "must be finite and within (0°, 89°)",
        });
    }
    if !params.neck_clearance.value_mm.is_finite() || params.neck_clearance.value_mm < 0.0 {
        return Err(CamError::InvalidParameter {
            param: "neck_clearance",
            value: params.neck_clearance.value_mm,
            reason: "must be finite and >= 0",
        });
    }
    if !params.stock_to_leave.value_mm.is_finite() || params.stock_to_leave.value_mm < 0.0 {
        return Err(CamError::InvalidParameter {
            param: "stock_to_leave",
            value: params.stock_to_leave.value_mm,
            reason: "must be finite and >= 0",
        });
    }
    if !params.lead_out_distance.value_mm.is_finite() || params.lead_out_distance.value_mm < 0.0 {
        return Err(CamError::InvalidParameter {
            param: "lead_out_distance",
            value: params.lead_out_distance.value_mm,
            reason: "must be finite and >= 0",
        });
    }

    if !ctx.doc.has_3d_geometry() {
        return Err(CamError::EmptyCadDocument);
    }
    let surface = SurfaceMesh3D::from_cad_document(ctx.doc, &[], &ctx.setup.frame);
    if surface.triangles.is_empty() || !surface.bounds.is_valid() {
        return Err(CamError::EmptyCadDocument);
    }

    let sphere_r = 0.5 * diameter_mm;
    let extra_stock = params
        .stock_to_leave
        .value_mm
        .max(ctx.passes.stock_to_leave_radial.value_mm)
        .max(0.0);
    let specs = LollipopSpecs {
        sphere_r,
        eff_r: sphere_r + extra_stock,
        neck_r: 0.5 * neck_diameter_mm,
        shank_r: 0.5 * shank_diameter_mm,
        z_join: CutterGeometry::lollipop_sphere_height_mm(diameter_mm, neck_diameter_mm),
        z_neck_top: neck_length_mm,
        z_stickout: ctx.tool.stickout_length_mm.max(neck_length_mm + 1.0),
        neck_clearance: params.neck_clearance.value_mm.max(0.0),
    };

    // Resolve 2D XY tool-center containment boundary from `params.boundary` minus Avoid / Keep-Out regions
    let mut resolved_boundary = resolve_3d_boundary_config_detailed(
        &surface,
        ctx.setup,
        &ctx.selection,
        &params.boundary,
        specs.eff_r,
    );
    let avoid_2d = ctx.resolve_avoid_2d(ctx.heights.bottom_z_wcs, ctx.heights.top_z_wcs);
    let had_avoid = !avoid_2d.effective_region.is_empty();
    resolved_boundary.apply_avoid(avoid_2d);
    builder.warnings.extend(resolved_boundary.warnings);
    if resolved_boundary.effective_region.is_empty() {
        if had_avoid {
            builder
                .warnings
                .push("Avoid / Keep-Out regions excluded 100% of the cutting area.".to_string());
        }
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }
    let containment_opt: Option<RegionSet2D> = Some(resolved_boundary.effective_region);

    // 3. Identify overhanging downward-facing triangles (`n_z <= -sin(min_overhang_angle)`)
    let sin_min = params.min_overhang_angle_deg.to_radians().sin();
    let min_nxy = specs.min_contact_nxy();
    let mut overhang_tris: Vec<usize> = Vec::new();
    let mut ov_z_min = f64::INFINITY;
    let mut ov_z_max = f64::NEG_INFINITY;

    for (ti, tri) in surface.triangles.iter().enumerate() {
        let n = tri.normal;
        if n.z <= -sin_min && n.x.hypot(n.y) + 1e-4 >= min_nxy {
            // Skip bottom-most base plate floor triangles resting on model bottom (`n_z < -0.999`)
            if n.z < -0.999 {
                continue;
            }
            if tri.aabb.max.z < ctx.heights.bottom_z_wcs - 1e-4
                || tri.aabb.min.z > ctx.heights.top_z_wcs + 1e-4
            {
                continue;
            }
            overhang_tris.push(ti);
            ov_z_min = ov_z_min.min(tri.aabb.min.z);
            ov_z_max = ov_z_max.max(tri.aabb.max.z);
        }
    }

    if overhang_tris.is_empty() {
        return Err(CamError::InvalidUndercutGeometry {
            reason: format!(
                "no reachable overhanging surfaces found (n_z <= -{sin_min:.3}) within Z [{:.2}, {:.2}] mm",
                ctx.heights.bottom_z_wcs, ctx.heights.top_z_wcs
            ),
        });
    }

    let z_lo = ov_z_min.max(ctx.heights.bottom_z_wcs);
    let z_hi = ov_z_max.min(ctx.heights.top_z_wcs);
    if z_hi <= z_lo + 1e-4 {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    // 4. Generate horizontal slice Z-levels across the overhanging surface band
    let dz = if params.use_cusp_height {
        let h = params.target_cusp.value_mm.clamp(1e-4, sphere_r * 0.5);
        (2.0 * (2.0 * sphere_r * h - h * h).max(1e-8).sqrt()).clamp(0.05, sphere_r * 0.75)
    } else {
        params.z_stepdown.value_mm.clamp(0.02, 20.0)
    };
    let z_levels = build_overhang_z_slices(z_lo, z_hi, dz);

    let safe_z = ctx.safe_retract_z();
    let feed_cut = ctx.cutting.cutting_feed_mm_min;
    let feed_lead = ctx.cutting.lead_feed_mm_min.min(feed_cut).max(1.0);
    let feed_plunge = ctx.cutting.plunge_feed_mm_min.min(feed_cut).max(1.0);
    let min_pull = params.lead_out_distance.value_mm.max(0.5);
    let mut cand_buf: Vec<usize> = Vec::with_capacity(64);
    let mut emitted_any = false;

    // 5. Slice each Z-level, solve exact upper-hemisphere contact, trim neck/shank collisions, and emit!
    for &z_slice in &z_levels {
        ctx.check_cancel()?;

        // Tiny deterministic offset if `z_slice` lands directly on a mesh vertex
        let mut z_eval = z_slice;
        let hits_v = overhang_tris.iter().any(|&ti| {
            let tri = &surface.triangles[ti];
            if tri.aabb.max.z < z_eval - 1e-7 || tri.aabb.min.z > z_eval + 1e-7 {
                return false;
            }
            tri.indices
                .iter()
                .any(|&vi| (surface.vertices[vi as usize].z - z_eval).abs() <= 1e-7)
        });
        if hits_v {
            z_eval += 1e-5;
        }

        let raw_segs = slice_overhang_triangles_at_z(&surface, &overhang_tris, z_eval);
        if raw_segs.is_empty() {
            continue;
        }

        let chains = chain_overhang_segments(&raw_segs);
        for chain in chains {
            ctx.check_cancel()?;
            let dense = densify_surface_chain(&chain, 0.5);
            if dense.len() < 2 {
                continue;
            }

            // Evaluate contact & full revolved neck/shank/head clearance at every station
            let subchains = trim_chain_for_neck_and_head_clearance(
                &dense,
                &specs,
                &surface,
                containment_opt.as_ref(),
                &mut cand_buf,
            );

            for mut sub in subchains {
                if sub.len() < 2 {
                    continue;
                }
                orient_overhang_subchain(&mut sub, params.climb_direction);

                let first = sub[0];
                let last = sub[sub.len() - 1];

                // Compute verified open-air horizontal lead-in and lead-out points BEFORE any vertical move!
                let Some(p_lead_in) = find_safe_open_air_pull_point(
                    first.tip_wcs,
                    first.outward_xy,
                    min_pull,
                    &specs,
                    safe_z,
                    &surface,
                    &mut cand_buf,
                ) else {
                    continue;
                };
                let Some(p_lead_out) = find_safe_open_air_pull_point(
                    last.tip_wcs,
                    last.outward_xy,
                    min_pull,
                    &specs,
                    safe_z,
                    &surface,
                    &mut cand_buf,
                ) else {
                    continue;
                };

                builder.pass_count += 1;
                emitted_any = true;

                // 1. Rapid in safe clearance Z above verified open-air lead-in column
                builder.rapid_to(DVec3::new(p_lead_in.x, p_lead_in.y, safe_z));
                // 2. Plunge vertically strictly in verified open air down to lead-in height
                builder.linear_to(p_lead_in, feed_plunge, MotionRole::Plunge);
                // 3. Feed horizontally inward under the overhang to first contact station
                builder.linear_to(first.tip_wcs, feed_lead, MotionRole::LeadIn);
                // 4. Cut smoothly along collision-free upper-hemisphere tangential contour
                for st in &sub[1..] {
                    builder.linear_to(st.tip_wcs, feed_cut, MotionRole::FinishCut);
                }
                // 5. Pull horizontally outward away from the overhang into verified open air
                builder.linear_to(p_lead_out, feed_lead, MotionRole::LeadOut);
                // 6. Retract vertically `+Z` strictly in open air!
                builder.rapid_to(DVec3::new(p_lead_out.x, p_lead_out.y, safe_z));
            }
        }
    }

    if !emitted_any {
        return Err(CamError::InvalidUndercutGeometry {
            reason: "all overhanging surface regions were unreachable without neck/shank collision (reduce neck_clearance or use a smaller-neck Lollipop Endmill)".to_string(),
        });
    }

    Ok(())
}

fn build_overhang_z_slices(z_lo: f64, z_hi: f64, dz: f64) -> Vec<f64> {
    let span = (z_hi - z_lo).max(0.0);
    let margin = (0.05_f64).min(0.25 * span);
    let top = z_hi - margin;
    let bot = z_lo + margin;
    if top <= bot + 1e-4 {
        return vec![0.5 * (z_lo + z_hi)];
    }
    let eff_span = top - bot;
    let n = (eff_span / dz).ceil().max(1.0) as usize;
    let step = eff_span / (n as f64);
    (0..=n).map(|i| top - step * (i as f64)).collect()
}

#[derive(Debug, Clone, Copy)]
struct OverhangSliceSeg {
    p0: DVec3,
    p1: DVec3,
    normal: DVec3,
}

fn slice_overhang_triangles_at_z(
    surface: &SurfaceMesh3D,
    overhang_tris: &[usize],
    z_plane: f64,
) -> Vec<OverhangSliceSeg> {
    let mut segs = Vec::new();
    for &ti in overhang_tris {
        let tri = &surface.triangles[ti];
        if tri.aabb.max.z < z_plane || tri.aabb.min.z > z_plane {
            continue;
        }
        let [i0, i1, i2] = tri.indices;
        let v0 = surface.vertices[i0 as usize];
        let v1 = surface.vertices[i1 as usize];
        let v2 = surface.vertices[i2 as usize];

        let d0 = v0.z - z_plane;
        let d1 = v1.z - z_plane;
        let d2 = v2.z - z_plane;
        let a0 = d0 > 0.0;
        let a1 = d1 > 0.0;
        let a2 = d2 > 0.0;
        if (a0 && a1 && a2) || (!a0 && !a1 && !a2) {
            continue;
        }

        let mut pts = [DVec3::ZERO; 2];
        let mut n_pts = 0usize;
        for (p, q, dp, dq, ap, aq) in [
            (v0, v1, d0, d1, a0, a1),
            (v1, v2, d1, d2, a1, a2),
            (v2, v0, d2, d0, a2, a0),
        ] {
            if ap != aq && n_pts < 2 {
                let t = dp / (dp - dq);
                pts[n_pts] = DVec3::new(p.x + t * (q.x - p.x), p.y + t * (q.y - p.y), z_plane);
                n_pts += 1;
            }
        }
        if n_pts != 2 || pts[0].distance_squared(pts[1]) <= 1e-10 {
            continue;
        }

        // Orient segment so the outward horizontal normal points to the LEFT of `p0 -> p1` (Climb for M3!)
        let mut p0 = pts[0];
        let mut p1 = pts[1];
        let seg = DVec2::new(p1.x - p0.x, p1.y - p0.y);
        let left_dir = DVec2::new(-seg.y, seg.x);
        let n_xy = DVec2::new(tri.normal.x, tri.normal.y);
        if left_dir.dot(n_xy) < 0.0 {
            std::mem::swap(&mut p0, &mut p1);
        }
        segs.push(OverhangSliceSeg {
            p0,
            p1,
            normal: tri.normal,
        });
    }
    segs
}

/// Chains directed overhang slice segments into ordered `(surface_pos, outward_normal)` polylines.
fn chain_overhang_segments(segs: &[OverhangSliceSeg]) -> Vec<Vec<(DVec3, DVec3)>> {
    if segs.is_empty() {
        return Vec::new();
    }
    const WELD_TOL: f64 = 1e-3;
    let inv_tol = 1.0 / WELD_TOL;
    let weld_sq = WELD_TOL * WELD_TOL;

    let mut verts: Vec<DVec3> = Vec::with_capacity(segs.len() * 2);
    let mut grid: HashMap<(i64, i64), Vec<usize>> = HashMap::with_capacity(segs.len() * 2);

    let mut intern = |p: DVec3| -> usize {
        let gx = (p.x * inv_tol).floor() as i64;
        let gy = (p.y * inv_tol).floor() as i64;
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(bucket) = grid.get(&(gx + dx, gy + dy)) {
                    for &idx in bucket {
                        if verts[idx].distance_squared(p) <= weld_sq {
                            return idx;
                        }
                    }
                }
            }
        }
        let idx = verts.len();
        verts.push(p);
        grid.entry((gx, gy)).or_default().push(idx);
        idx
    };

    let mut edges: Vec<(usize, usize, DVec3)> = Vec::with_capacity(segs.len());
    let mut outgoing: HashMap<usize, Vec<usize>> = HashMap::with_capacity(segs.len());
    let mut incoming_count: HashMap<usize, usize> = HashMap::with_capacity(segs.len());
    for s in segs {
        let ia = intern(s.p0);
        let ib = intern(s.p1);
        if ia == ib {
            continue;
        }
        let eid = edges.len();
        edges.push((ia, ib, s.normal));
        outgoing.entry(ia).or_default().push(eid);
        *incoming_count.entry(ib).or_insert(0) += 1;
    }

    // Prefer starting open chains from vertices with in-degree 0 so open strips aren't fragmented!
    let mut start_order: Vec<usize> = (0..edges.len()).collect();
    start_order.sort_by_key(|&eid| incoming_count.get(&edges[eid].0).copied().unwrap_or(0));

    let mut used = vec![false; edges.len()];
    let mut chains = Vec::new();

    for start_eid in start_order {
        if used[start_eid] {
            continue;
        }
        used[start_eid] = true;
        let (start_v, mut cur_v, mut cur_n) = edges[start_eid];
        let mut pts = vec![(verts[start_v], cur_n), (verts[cur_v], cur_n)];

        while cur_v != start_v {
            let next_edge = outgoing.get(&cur_v).and_then(|list| {
                list.iter()
                    .copied()
                    .filter(|&eid| !used[eid])
                    .max_by(|&a, &b| edges[a].2.dot(cur_n).total_cmp(&edges[b].2.dot(cur_n)))
            });
            let Some(eid) = next_edge else {
                break;
            };
            // Break chain across sharp normal jumps (> 40°) so distinct overhang walls stay clean
            if edges[eid].2.dot(cur_n) < 0.766 {
                break;
            }
            used[eid] = true;
            cur_v = edges[eid].1;
            cur_n = edges[eid].2;
            // Smooth shared vertex normal across adjacent triangles on the same curved wall
            if let Some(prev) = pts.last_mut() {
                let avg = (prev.1 + cur_n).normalize_or_zero();
                if avg.length_squared() > 0.5 {
                    prev.1 = avg;
                }
            }
            pts.push((verts[cur_v], cur_n));
        }

        if pts.len() >= 2 {
            chains.push(pts);
        }
    }

    chains
}

/// Subdivides a surface chain so consecutive stations are at most `max_step_mm` apart.
fn densify_surface_chain(chain: &[(DVec3, DVec3)], max_step_mm: f64) -> Vec<(DVec3, DVec3)> {
    let mut out = Vec::with_capacity(chain.len() * 2);
    if let Some(&first) = chain.first() {
        out.push(first);
    }
    for win in chain.windows(2) {
        let (p0, n0) = win[0];
        let (p1, n1) = win[1];
        let dist = p0.distance(p1);
        if dist <= 1e-6 {
            continue;
        }
        let n_sub = (dist / max_step_mm.max(0.05)).ceil().clamp(1.0, 256.0) as usize;
        for k in 1..=n_sub {
            let u = k as f64 / n_sub as f64;
            let p = p0.lerp(p1, u);
            let n = n0.lerp(n1, u).normalize_or_zero();
            out.push((p, if n.length_squared() > 0.5 { n } else { n0 }));
        }
    }
    out
}

/// Evaluates every station along `dense`, filters by optional 2D containment + exact `FlatBvh`
/// relieved-neck/shank/head clearance, and splits into collision-free subchains with binary
/// boundary refinement (`< 0.01 mm`).
fn trim_chain_for_neck_and_head_clearance(
    dense: &[(DVec3, DVec3)],
    specs: &LollipopSpecs,
    surface: &SurfaceMesh3D,
    containment: Option<&RegionSet2D>,
    cand_buf: &mut Vec<usize>,
) -> Vec<Vec<OverhangStation>> {
    let n = dense.len();
    let mut solved: Vec<Option<OverhangStation>> = Vec::with_capacity(n);
    let mut free: Vec<bool> = Vec::with_capacity(n);

    for &(p_surf, n_surf) in dense {
        let sol = specs.solve_tip(p_surf, n_surf);
        let ok = sol.is_some_and(|st| {
            let in_xy = containment.is_none_or(|reg| {
                let tip_xy = DVec2::new(st.tip_wcs.x, st.tip_wcs.y);
                reg.contains_point(tip_xy) || reg.min_distance_to_boundary(tip_xy) <= 0.02
            });
            in_xy && is_lollipop_station_clear(st.tip_wcs, specs, surface, cand_buf)
        });
        solved.push(sol);
        free.push(ok);
    }

    let mut subchains = Vec::new();
    let mut i = 0usize;
    while i < n {
        if !free[i] {
            i += 1;
            continue;
        }
        let start = i;
        while i + 1 < n && free[i + 1] {
            i += 1;
        }
        let end = i;
        i += 1;

        let mut pts = Vec::with_capacity(end - start + 3);
        if start > 0
            && let Some(ref_st) = binary_refine_overhang_boundary(
                dense[start],
                dense[start - 1],
                specs,
                surface,
                containment,
                cand_buf,
            )
            && solved[start].is_none_or(|s| ref_st.tip_wcs.distance(s.tip_wcs) > 1e-4)
        {
            pts.push(ref_st);
        }
        for st in solved[start..=end].iter().flatten() {
            pts.push(*st);
        }
        if end + 1 < n
            && let Some(ref_st) = binary_refine_overhang_boundary(
                dense[end],
                dense[end + 1],
                specs,
                surface,
                containment,
                cand_buf,
            )
            && pts
                .last()
                .is_none_or(|last| ref_st.tip_wcs.distance(last.tip_wcs) > 1e-4)
        {
            pts.push(ref_st);
        }

        let total_len: f64 = pts
            .windows(2)
            .map(|w| w[0].tip_wcs.distance(w[1].tip_wcs))
            .sum();
        if pts.len() >= 2 && total_len >= 0.05 {
            subchains.push(pts);
        }
    }

    subchains
}

fn binary_refine_overhang_boundary(
    good: (DVec3, DVec3),
    bad: (DVec3, DVec3),
    specs: &LollipopSpecs,
    surface: &SurfaceMesh3D,
    containment: Option<&RegionSet2D>,
    cand_buf: &mut Vec<usize>,
) -> Option<OverhangStation> {
    let mut lo = 0.0_f64;
    let mut hi = 1.0_f64;
    let mut best = None;
    for _ in 0..8 {
        let mid = 0.5 * (lo + hi);
        let p = good.0.lerp(bad.0, mid);
        let n = good.1.lerp(bad.1, mid).normalize_or_zero();
        let ok = if let Some(st) = specs.solve_tip(p, n) {
            let in_xy = containment.is_none_or(|reg| {
                let tip_xy = DVec2::new(st.tip_wcs.x, st.tip_wcs.y);
                reg.contains_point(tip_xy) || reg.min_distance_to_boundary(tip_xy) <= 0.02
            });
            if in_xy && is_lollipop_station_clear(st.tip_wcs, specs, surface, cand_buf) {
                best = Some(st);
                true
            } else {
                false
            }
        } else {
            false
        };
        if ok {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    best
}

/// Checks full revolved Lollipop cutter clearance against `surface.bvh`:
/// 1. Spherical cutting head centered at `(tip.x, tip.y, tip.z + sphere_r)` must not gouge
///    `surface` deeper than `0.025 mm` inside `eff_r`.
/// 2. Relieved neck (`dz ∈ [z_join, z_neck_top]`) must maintain radial clearance `>= neck_r + neck_clearance`.
/// 3. Upper shank (`dz ∈ (z_neck_top, z_stickout]`) must maintain radial clearance `>= shank_r + neck_clearance`.
fn is_lollipop_station_clear(
    tip: DVec3,
    specs: &LollipopSpecs,
    surface: &SurfaceMesh3D,
    cand_buf: &mut Vec<usize>,
) -> bool {
    let r_query = specs
        .eff_r
        .max(specs.shank_r + specs.neck_clearance)
        .max(specs.neck_r + specs.neck_clearance)
        + 0.05;
    let query_aabb = Aabb3::new(
        DVec3::new(tip.x - r_query, tip.y - r_query, tip.z - 0.05),
        DVec3::new(tip.x + r_query, tip.y + r_query, tip.z + specs.z_stickout),
    );
    surface.bvh.query_aabb_into(&query_aabb, cand_buf);

    let sphere_center = DVec3::new(tip.x, tip.y, tip.z + specs.sphere_r);
    let head_min_r = (specs.eff_r - 0.025).max(0.05);
    let head_min_r2 = head_min_r * head_min_r;
    let neck_req = specs.neck_r + specs.neck_clearance;
    let shank_req = specs.shank_r + specs.neck_clearance;
    let tip_xy = DVec2::new(tip.x, tip.y);

    for &ti in cand_buf.iter() {
        let tri = &surface.triangles[ti];
        let v0 = surface.vertices[tri.indices[0] as usize];
        let v1 = surface.vertices[tri.indices[1] as usize];
        let v2 = surface.vertices[tri.indices[2] as usize];

        // 1. Spherical head gouge test (`closest_point_on_triangle` in exact f64!)
        if tri.aabb.min.z <= sphere_center.z + specs.eff_r
            && tri.aabb.max.z >= sphere_center.z - specs.eff_r
        {
            let (_, d2, _, _) = closest_point_on_triangle(sphere_center, v0, v1, v2);
            if d2 < head_min_r2 {
                return false;
            }
        }

        // 2. Relieved neck & shank vertical cylinder clearance above `z_join`
        let z_cyl_lo = tip.z + specs.z_join;
        let z_cyl_hi = tip.z + specs.z_stickout;
        let z_lo = tri.aabb.min.z.max(z_cyl_lo);
        let z_hi = tri.aabb.max.z.min(z_cyl_hi);
        if z_hi < z_lo - 1e-6 {
            continue;
        }

        // Check critical Z planes (`z_lo`, `z_hi`, `z_neck_top`) plus uniform 0.4 mm sub-samples
        let z_neck_top_abs = tip.z + specs.z_neck_top;
        let check_plane = |zk: f64| -> bool {
            let Some(q3) = chip_bvh::closest_point_on_triangle_at_z(v0, v1, v2, zk, tip_xy) else {
                return true;
            };
            let q = DVec2::new(q3.x, q3.y);
            let dz = zk - tip.z;
            let req_r = if dz <= specs.z_neck_top + 1e-6 {
                neck_req
            } else {
                shank_req
            };
            q.distance(tip_xy) >= req_r - 1e-4
        };

        if !check_plane(z_lo) || !check_plane(z_hi) {
            return false;
        }
        if z_neck_top_abs > z_lo + 1e-5
            && z_neck_top_abs < z_hi - 1e-5
            && (!check_plane(z_neck_top_abs - 1e-4) || !check_plane(z_neck_top_abs + 1e-4))
        {
            return false;
        }
        let span = (z_hi - z_lo).max(0.0);
        let n_sub = ((span / 0.4).ceil() as usize).clamp(1, 64);
        for k in 1..n_sub {
            let zk = z_lo + span * (k as f64 / n_sub as f64);
            if !check_plane(zk) {
                return false;
            }
        }
    }

    true
}

/// Checks whether a vertical plunge/retract column above `tip` up to `safe_z` with full cutter
/// radius (`eff_r + neck_clearance`) is 100% clear of `surface` (zero roof crashes!).
fn is_vertical_column_open_air(
    tip: DVec3,
    safe_z: f64,
    specs: &LollipopSpecs,
    surface: &SurfaceMesh3D,
    cand_buf: &mut Vec<usize>,
) -> bool {
    let req_r = specs.eff_r.max(specs.shank_r) + specs.neck_clearance;
    let query_aabb = Aabb3::new(
        DVec3::new(tip.x - req_r - 0.05, tip.y - req_r - 0.05, tip.z - 0.05),
        DVec3::new(
            tip.x + req_r + 0.05,
            tip.y + req_r + 0.05,
            safe_z.max(tip.z + specs.z_stickout),
        ),
    );
    surface.bvh.query_aabb_into(&query_aabb, cand_buf);
    let xy = DVec2::new(tip.x, tip.y);

    for &ti in cand_buf.iter() {
        let tri = &surface.triangles[ti];
        let v0 = surface.vertices[tri.indices[0] as usize];
        let v1 = surface.vertices[tri.indices[1] as usize];
        let v2 = surface.vertices[tri.indices[2] as usize];
        let z_lo = tri.aabb.min.z.max(tip.z);
        let z_hi = tri.aabb.max.z.min(safe_z.max(tip.z + specs.z_stickout));
        if z_hi < z_lo - 1e-6 {
            continue;
        }
        let span = (z_hi - z_lo).max(0.0);
        let n_sub = ((span / 0.5).ceil() as usize).clamp(1, 48);
        for k in 0..=n_sub {
            let zk = z_lo + span * (k as f64 / n_sub as f64);
            if let Some(q3) = chip_bvh::closest_point_on_triangle_at_z(v0, v1, v2, zk, xy)
                && DVec2::new(q3.x, q3.y).distance(xy) < req_r - 1e-4
            {
                return false;
            }
        }
    }
    true
}

/// Finds a verified open-air horizontal pull-away point `p_pull` away from `tip` along `outward_xy`
/// (with slight angle fan fallback) where both the horizontal transition and the full vertical
/// column up to `safe_z` are 100% collision-free against `surface`!
fn find_safe_open_air_pull_point(
    tip: DVec3,
    outward_xy: DVec2,
    min_pull: f64,
    specs: &LollipopSpecs,
    safe_z: f64,
    surface: &SurfaceMesh3D,
    cand_buf: &mut Vec<usize>,
) -> Option<DVec3> {
    let base_dir = outward_xy.normalize_or_zero();
    if base_dir.length_squared() < 0.5 {
        return None;
    }
    let angles_deg = [0.0_f64, 20.0, -20.0, 40.0, -40.0];
    let max_extra = specs.eff_r * 3.0 + 12.0;

    for &ang_deg in &angles_deg {
        let (s, c) = ang_deg.to_radians().sin_cos();
        let dir = DVec2::new(
            base_dir.x * c - base_dir.y * s,
            base_dir.x * s + base_dir.y * c,
        );
        let mut d = min_pull;
        while d <= min_pull + max_extra + 1e-6 {
            let cand = DVec3::new(tip.x + dir.x * d, tip.y + dir.y * d, tip.z);
            if is_vertical_column_open_air(cand, safe_z, specs, surface, cand_buf) {
                // Verify intermediate horizontal lead segment stations are also collision-free
                let n_chk = ((d / 0.75).ceil() as usize).clamp(1, 24);
                let mut path_clear = true;
                for k in 1..=n_chk {
                    let u = k as f64 / n_chk as f64;
                    let mid = tip.lerp(cand, u);
                    if !is_lollipop_station_clear(mid, specs, surface, cand_buf) {
                        path_clear = false;
                        break;
                    }
                }
                if path_clear {
                    return Some(cand);
                }
            }
            d += 0.75;
        }
    }
    None
}

/// Orients an overhang contour subchain for Climb (`true`) vs Conventional (`false`) milling.
/// For CW (`M3`) spindle: Climb keeps material on the RIGHT, meaning outward air normal `outward_xy`
/// points to the LEFT of travel `(-t.y, t.x)`.
fn orient_overhang_subchain(sub: &mut [OverhangStation], climb_direction: bool) {
    let mut climb_score = 0.0_f64;
    for win in sub.windows(2) {
        let seg = DVec2::new(
            win[1].tip_wcs.x - win[0].tip_wcs.x,
            win[1].tip_wcs.y - win[0].tip_wcs.y,
        );
        let left_n = DVec2::new(-seg.y, seg.x);
        climb_score += left_n.dot(win[0].outward_xy);
    }
    let currently_climb = climb_score >= 0.0;
    if currently_climb != climb_direction {
        sub.reverse();
    }
}

/// Computes the closest XY point on the horizontal cross-section `triangle ∩ {z = z_plane}` to `target_xy`.
#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::sync::atomic::AtomicBool;

    use chip_cad::{MeshImportOptions, load_mesh_cad_document};
    use chip_math::{DVec2, DVec3, DisplayUnit, ParamLength};
    use chip_tools::starter_lollipop_6mm;

    use crate::{
        BoxAnchor27, FixtureBody, HeightReference, HeightSpec, Operation, OperationGenRequest,
        OperationKind, Setup, StockConfig, StrategyParams, Undercut3DParams, WcsOriginMode,
        WorkOffsetSlot, generate_operation_toolpath_with_fixtures,
        resolve_setup_fixtures_parametric,
    };

    #[test]
    fn undercut3d_cusp_mode_feed_opt_and_fixture_avoidance() {
        let u3d_p = Undercut3DParams {
            use_cusp_height: true,
            target_cusp: ParamLength::from_mm(0.03),
            ..Undercut3DParams::default()
        };

        // Verify Undercut3D cusp-height slicing flows through M58 feed_opt & M61 fixture avoidance
        // Steep ~68° overhang (`dx_y = 2.0` over `dz = 5.0`, `Z ∈ [11..16]`) gives 3+ reachable cusp passes!
        let prof_yz: [(f64, f64); 6] = [
            (0.0, 0.0),
            (18.0, 0.0),
            (18.0, 11.0),
            (20.0, 16.0),
            (20.0, 20.0),
            (0.0, 20.0),
        ];
        let mut stl = String::from("solid overhang_u3d\n");
        let mut push_tri = |a: DVec3, b: DVec3, c: DVec3| {
            let n = (b - a).cross(c - a).normalize_or_zero();
            stl.push_str(&format!(
                "  facet normal {:.6} {:.6} {:.6}\n    outer loop\n      vertex {:.6} {:.6} {:.6}\n      vertex {:.6} {:.6} {:.6}\n      vertex {:.6} {:.6} {:.6}\n    endloop\n  endfacet\n",
                n.x, n.y, n.z, a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z
            ));
        };
        for i in 0..prof_yz.len() {
            let (ya, za) = prof_yz[i];
            let (yb, zb) = prof_yz[(i + 1) % prof_yz.len()];
            push_tri(
                DVec3::new(0.0, ya, za),
                DVec3::new(0.0, yb, zb),
                DVec3::new(40.0, yb, zb),
            );
            push_tri(
                DVec3::new(0.0, ya, za),
                DVec3::new(40.0, yb, zb),
                DVec3::new(40.0, ya, za),
            );
        }
        stl.push_str("endsolid overhang_u3d\n");

        let doc = load_mesh_cad_document(
            "overhang_u3d.stl",
            stl.as_bytes(),
            &MeshImportOptions::default(),
        )
        .unwrap();

        let mut setup = Setup::new("s_u3d", "U3D Fixture Test", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
        };
        setup.stock = StockConfig::RelativeBox {
            side_pos_x: ParamLength::from_mm(0.0),
            side_neg_x: ParamLength::from_mm(0.0),
            side_pos_y: ParamLength::from_mm(0.0),
            side_neg_y: ParamLength::from_mm(0.0),
            symmetric_xy: true,
            top_offset_z: ParamLength::from_mm(0.0),
            bottom_offset_z: ParamLength::from_mm(0.0),
            sheet_thickness_z: ParamLength::from_mm(20.0),
            round_increment_mm: None,
        };
        setup.fixtures.push(FixtureBody::default_toe_clamp(
            "clamp_top",
            "Top Clamp",
            DVec2::new(20.0, 26.0),
            0.0,
        ));
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let fixtures =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();

        let lolli = starter_lollipop_6mm();
        let mut op = Operation::new("op_u3d_cusp", "Cusp Undercut", OperationKind::Undercut3D);
        op.tool_id = Some(lolli.id.clone());
        op.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
        op.passes.feed_optimization.enabled = true;
        op.strategy = Some(StrategyParams::Undercut3D(u3d_p));

        let cancel = AtomicBool::new(false);
        let tp = generate_operation_toolpath_with_fixtures(
            &OperationGenRequest {
                doc: &doc,
                sketches: &[],
                setup: &resolved,
                op: &op,
                tool: &lolli,
                clearing_tool: None,
                display_unit: DisplayUnit::Millimeter,
                is_first_tool_in_setup: true,
                upstream_rest: &[],
                carryover_stock: None,
                fixtures: &[],
                shared_avoid_regions: &[],
                cancel: &cancel,
            },
            &fixtures,
            true,
            None,
        )
        .unwrap();
        assert!(tp.stats.pass_count >= 2);
        assert!(tp.rapid_lifts_inserted >= 1);
    }
}
