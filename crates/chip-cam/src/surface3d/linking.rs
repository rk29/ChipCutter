//! Surface-safe collision-free 3D linking, overhang shadow residual suppression, and
//! overhang-cliff chain splitting.

use chip_math::{DVec2, DVec3, point_in_triangle_2d};
use rayon::prelude::*;

use crate::{
    cl_data::MotionRole,
    rest2d::RegionSet2D,
    surface3d::{
        SurfaceCutterProfile, SurfaceMesh3D,
        drop_cutter::{drop_cutter_highest_contact, drop_cutter_triangle_contact_shape},
        drop_cutter_z_with_scratch,
    },
    toolpath::ToolpathBuilder,
};

/// Parameters controlling gouge-free 3D linking between consecutive finishing passes.
#[derive(Debug, Clone, Copy)]
pub struct SurfaceLinkConfig {
    pub retract_z: f64,
    pub stock_top_z: f64,
    pub z_floor: f64,
    pub stay_down_limit_mm: f64,
    pub sample_step_mm: f64,
    pub lead_feed: f64,
    pub plunge_feed: f64,
}

/// Emits a gouge-proof link from `builder.current_pos()` to `to_wcs`, either via a short
/// surface-hugging stay-down link or a safe retract + rapid traverse + vertical approach, forcing
/// a safe retract whenever the XY chord crosses `avoid_region`.
pub fn emit_surface_safe_link_with_avoid(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    to_wcs: DVec3,
    cfg: &SurfaceLinkConfig,
    avoid_region: Option<&RegionSet2D>,
    bvh_scratch: &mut Vec<usize>,
    builder: &mut ToolpathBuilder,
) {
    let safe_rapid_approach_z = (cfg.stock_top_z + 1.0)
        .max(to_wcs.z + 1.0)
        .min(cfg.retract_z);

    if let Some(from_wcs) = builder.current_pos() {
        let dist_xy = (to_wcs.x - from_wcs.x).hypot(to_wcs.y - from_wcs.y);
        if dist_xy <= 1e-4 && (to_wcs.z - from_wcs.z).abs() <= 1e-4 {
            return;
        }

        // Try gouge-proof surface-hugging stay-down link for short adjacent stepovers,
        // except when descending steeply off a split overhang cliff or crossing an Avoid / Keep-Out region.
        let steep_cliff_drop =
            (from_wcs.z - to_wcs.z) > 1.5 && dist_xy < (from_wcs.z - to_wcs.z) * 0.5;
        let crosses_avoid = avoid_region.is_some_and(|av| {
            !av.is_empty()
                && !crate::surface3d::is_2d_segment_clear_of_avoid(
                    DVec2::new(from_wcs.x, from_wcs.y),
                    DVec2::new(to_wcs.x, to_wcs.y),
                    av,
                )
        });
        if !steep_cliff_drop
            && !crosses_avoid
            && dist_xy <= cfg.stay_down_limit_mm
            && dist_xy > 1e-4
        {
            let step = cfg.sample_step_mm.clamp(0.10, 0.25);
            let n = (dist_xy / step).ceil().max(3.0) as usize;
            let mut raw_link = Vec::with_capacity(n + 3);
            let base_min_z = from_wcs.z.min(to_wcs.z);
            let max_allowed_rise = (2.0 * cutter.max_radius_mm()).clamp(2.5, 4.5);
            let link_clearance = 0.25_f64;

            // Smooth vertical micro-lift off `from_wcs` so turnaround links never graze the surface
            let from_lift_z = (from_wcs.z + link_clearance).min(cfg.retract_z);
            let to_lift_z = (to_wcs.z + link_clearance).min(cfg.retract_z);
            raw_link.push(DVec3::new(from_wcs.x, from_wcs.y, from_lift_z));

            for k in 1..n {
                let t = k as f64 / n as f64;
                let x = from_wcs.x + (to_wcs.x - from_wcs.x) * t;
                let y = from_wcs.y + (to_wcs.y - from_wcs.y) * t;
                let lerp_z = from_lift_z + (to_lift_z - from_lift_z) * t;
                let z_surf =
                    drop_cutter_z_with_scratch(surface, cutter, x, y, cfg.z_floor, bvh_scratch)
                        .unwrap_or(cfg.z_floor);
                let z = lerp_z.max(z_surf + link_clearance).min(cfg.retract_z);
                raw_link.push(DVec3::new(x, y, z));
            }
            raw_link.push(DVec3::new(to_wcs.x, to_wcs.y, to_lift_z));

            // Recursively verify every link chord clears exact `drop_cutter_z + 0.15 mm`
            let mut safe_link = Vec::with_capacity(raw_link.len() * 2 + 2);
            safe_link.push(raw_link[0]);
            for win in raw_link.windows(2) {
                refine_link_chord_recursive(
                    surface,
                    cutter,
                    cfg.z_floor,
                    win[0],
                    win[1],
                    0.15,
                    0,
                    bvh_scratch,
                    &mut safe_link,
                );
            }
            safe_link.push(to_wcs);

            let max_rise = safe_link
                .iter()
                .map(|p| p.z - base_min_z)
                .fold(0.0_f64, f64::max);

            if max_rise <= max_allowed_rise {
                for &p in &safe_link {
                    builder.linear_to(p, cfg.lead_feed, MotionRole::LinkTransition);
                }
                return;
            }
        }

        // Safe retract above stock/part before rapid traverse
        let pull_off_z = (from_wcs.z + 0.5).min(cfg.retract_z);
        if pull_off_z > from_wcs.z + 1e-4 {
            builder.linear_to(
                DVec3::new(from_wcs.x, from_wcs.y, pull_off_z),
                cfg.lead_feed,
                MotionRole::LeadOut,
            );
        }
        if pull_off_z < cfg.retract_z - 1e-4 {
            builder.rapid_to(DVec3::new(from_wcs.x, from_wcs.y, cfg.retract_z));
        }
    }

    // Rapid horizontally at safe retract height, rapid down only to guaranteed stock clearance,
    // then feed cleanly down onto `to_wcs`.
    builder.rapid_to(DVec3::new(to_wcs.x, to_wcs.y, cfg.retract_z));
    if safe_rapid_approach_z < cfg.retract_z - 1e-4 {
        builder.rapid_to(DVec3::new(to_wcs.x, to_wcs.y, safe_rapid_approach_z));
    }
    if to_wcs.z < safe_rapid_approach_z - 1e-4 {
        builder.linear_to(to_wcs, cfg.plunge_feed, MotionRole::Plunge);
    }
}

/// Emits a clean vertical pull-off and rapid retract to `retract_z` at the end of a 3D op.
pub fn emit_final_surface_retract(retract_z: f64, lead_feed: f64, builder: &mut ToolpathBuilder) {
    if let Some(last) = builder.current_pos()
        && last.z < retract_z - 1e-4
    {
        let pull_off_z = (last.z + 0.5).min(retract_z);
        if pull_off_z > last.z + 1e-4 {
            builder.linear_to(
                DVec3::new(last.x, last.y, pull_off_z),
                lead_feed,
                MotionRole::LeadOut,
            );
        }
        if pull_off_z < retract_z - 1e-4 {
            builder.rapid_to(DVec3::new(last.x, last.y, retract_z));
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn refine_link_chord_recursive(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    z_floor: f64,
    a: DVec3,
    b: DVec3,
    clearance_mm: f64,
    depth: u32,
    scratch: &mut Vec<usize>,
    out: &mut Vec<DVec3>,
) {
    let xy_dist = (b.x - a.x).hypot(b.y - a.y);
    if depth < 4 && xy_dist > 0.04 {
        let mid_x = 0.5 * (a.x + b.x);
        let mid_y = 0.5 * (a.y + b.y);
        let chord_mid_z = 0.5 * (a.z + b.z);
        if let Some(z_raw) =
            drop_cutter_z_with_scratch(surface, cutter, mid_x, mid_y, z_floor, scratch)
        {
            let req_z = z_raw.max(z_floor) + clearance_mm;
            if req_z > chord_mid_z + 0.002 {
                let mid = DVec3::new(mid_x, mid_y, req_z);
                refine_link_chord_recursive(
                    surface,
                    cutter,
                    z_floor,
                    a,
                    mid,
                    clearance_mm,
                    depth + 1,
                    scratch,
                    out,
                );
                refine_link_chord_recursive(
                    surface,
                    cutter,
                    z_floor,
                    mid,
                    b,
                    clearance_mm,
                    depth + 1,
                    scratch,
                    out,
                );
                return;
            }
        }
    }
    out.push(b);
}

/// Suppresses spurious 3D rest / pencil residual cells caused by disconnected overhanging roofs
/// above an open air void, while preserving 100% of genuine concave fillets, V-grooves,
/// vertical-wall-to-floor corners, and sculpted relief valleys.
///
/// For any cell `(x, y)` with `residual > 0`, locates the highest reference-ball surface contact
/// `c_ref` inside `r <= ref_radius_mm` and probes vertically beneath `c_ref`:
/// - If solid material connects `z_cur` continuously up to `c_ref`, the residual is genuine and
///   remains untouched.
/// - If `c_ref` sits on an overhanging structure above an open air gap underneath a downward-facing
///   surface (`normal.z < -0.15`), recomputes the maximum residual from non-overhanging connected
///   surfaces below the overhang ceiling (zeroing pure overhang-shadow bands while preserving any
///   genuine lower fillet nearby).
#[allow(clippy::too_many_arguments)]
pub fn suppress_overhang_shadow_residuals(
    surface: &SurfaceMesh3D,
    current_cutter: &SurfaceCutterProfile,
    ref_radius_mm: f64,
    z_cur: &[Option<f64>],
    residual: &mut [f64],
    min_x: f64,
    min_y: f64,
    cell_mm: f64,
    nx: usize,
    ny: usize,
) {
    if nx == 0 || ny == 0 || surface.triangles.is_empty() || residual.len() < nx * ny {
        return;
    }

    // Fast path: if the mesh has no downward-facing triangles strictly above its lowest floor,
    // no overhang shadow can exist anywhere on the part.
    let z_min_part = surface.bounds.min.z;
    let has_any_overhang = surface
        .triangles
        .iter()
        .any(|t| t.normal.z < -0.15 && t.aabb.max.z > z_min_part + 0.05);
    if !has_any_overhang {
        return;
    }

    let ref_r = ref_radius_mm.max(current_cutter.max_radius_mm());
    let ref_d = (2.0 * ref_r).max(0.1);
    let ref_ball_geom = chip_tools::CutterGeometry::BallEndmill {
        diameter_mm: ref_d,
        flute_length_mm: current_cutter.flute_length_mm.max(ref_d * 2.0),
        overall_length_mm: current_cutter.flute_length_mm.max(ref_d * 2.0) + 20.0,
        shank_diameter_mm: ref_d,
        flutes: 2,
    };
    let ref_cutter = SurfaceCutterProfile::from_tool_geometry(
        &ref_ball_geom,
        0.0,
        current_cutter.stock_axial_mm,
    );

    residual
        .par_chunks_mut(nx)
        .enumerate()
        .for_each(|(iy, row)| {
            if iy >= ny {
                return;
            }
            let y = min_y + iy as f64 * cell_mm;
            let mut cand_buf = Vec::with_capacity(64);
            let mut ray_buf = Vec::with_capacity(32);

            for (ix, res_val) in row.iter_mut().enumerate() {
                if *res_val <= 1e-4 {
                    continue;
                }
                let Some(zc) = z_cur[iy * nx + ix] else {
                    continue;
                };
                let x = min_x + ix as f64 * cell_mm;
                let p_xy = DVec2::new(x, y);
                let z_surf_cur = zc - current_cutter.stock_axial_mm;

                let Some((_best_z, best_pt, best_tri)) =
                    drop_cutter_highest_contact(surface, &ref_cutter, p_xy, &mut cand_buf)
                else {
                    continue;
                };

                let Some(z_overhang_bot) = overhang_ceiling_above_air_gap(
                    surface,
                    best_pt,
                    best_tri,
                    p_xy,
                    z_surf_cur,
                    &mut ray_buf,
                ) else {
                    continue;
                };

                // Highest `z_ref` contact rests on an overhanging roof above an open air gap.
                // Check whether any connected (non-overhanging) triangle below the overhang
                // ceiling still leaves a genuine concave residual at `(x, y)`.
                let mut best_connected_z = f64::NEG_INFINITY;
                for &ti in &cand_buf {
                    let tri = &surface.triangles[ti];
                    if tri.aabb.min.z >= z_overhang_bot - 0.05 {
                        continue;
                    }
                    let [i0, i1, i2] = tri.indices;
                    let v0 = surface.vertices[i0 as usize];
                    let v1 = surface.vertices[i1 as usize];
                    let v2 = surface.vertices[i2 as usize];
                    if let Some((z_tri, c_pt)) =
                        drop_cutter_triangle_contact_shape(&ref_cutter.shape, p_xy, v0, v1, v2)
                        && z_tri > best_connected_z
                        && overhang_ceiling_above_air_gap(
                            surface,
                            c_pt,
                            ti,
                            p_xy,
                            z_surf_cur,
                            &mut ray_buf,
                        )
                        .is_none()
                    {
                        best_connected_z = z_tri;
                    }
                }

                let max_connected_res = if best_connected_z.is_finite() {
                    (best_connected_z + ref_cutter.stock_axial_mm - zc).max(0.0)
                } else {
                    0.0
                };
                *res_val = (*res_val).min(max_connected_res);
            }
        });
}

/// Returns `true` when the transition between two nearby 3D cutter-tip points `a` and `b`
/// crosses a vertical overhang-shadow air gap (upper point resting on a disconnected overhanging
/// roof above a downward-facing underside `normal.z < -0.15`, lower point on the lower floor/hub),
/// rather than a continuous steep wall.
pub fn is_overhang_shadow_transition(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    a: DVec3,
    b: DVec3,
    cand_buf: &mut Vec<usize>,
    ray_buf: &mut Vec<usize>,
) -> bool {
    let (hi, lo) = if a.z >= b.z { (a, b) } else { (b, a) };
    if hi.z - lo.z <= 0.15 {
        return false;
    }
    let hi_xy = DVec2::new(hi.x, hi.y);
    let lo_xy = DVec2::new(lo.x, lo.y);
    let z_surf_lo = lo.z - cutter.stock_axial_mm;

    let Some((_z_hit, contact_pt, tri_idx)) =
        drop_cutter_highest_contact(surface, cutter, hi_xy, cand_buf)
    else {
        return false;
    };

    overhang_ceiling_above_air_gap(surface, contact_pt, tri_idx, lo_xy, z_surf_lo, ray_buf)
        .is_some()
}

/// Splits a 3D finishing cutting chain at steep downward overhang-shadow drop-offs so the cutter
/// retracts/approaches cleanly and plunges at `plunge_feed` via [`emit_surface_safe_link`] instead
/// of diving nearly vertically at full `cut_feed`, while keeping continuous steep walls intact.
pub fn split_chains_at_overhang_cliffs(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    chain: Vec<DVec3>,
) -> Vec<Vec<DVec3>> {
    if chain.len() < 2 {
        return Vec::new();
    }

    let z_min_part = surface.bounds.min.z;
    let has_any_overhang = surface
        .triangles
        .iter()
        .any(|t| t.normal.z < -0.15 && t.aabb.max.z > z_min_part + 0.05);
    if !has_any_overhang {
        return vec![chain];
    }

    let tool_d = 2.0 * cutter.max_radius_mm();
    let max_drop_mm = (0.5 * tool_d).max(1.5);
    // tan(70°) ≈ 2.747477419454622
    const TAN_70_DEG: f64 = 2.747_477_419_454_622;

    let mut cand_buf = Vec::with_capacity(64);
    let mut ray_buf = Vec::with_capacity(32);
    let mut out: Vec<Vec<DVec3>> = Vec::new();
    let mut cur: Vec<DVec3> = Vec::with_capacity(chain.len());
    cur.push(chain[0]);

    for &b in &chain[1..] {
        let a = *cur.last().unwrap_or(&b);
        let dz_drop = a.z - b.z;
        let dz_rise = b.z - a.z;
        let dxy = (b.x - a.x).hypot(b.y - a.y);
        let is_steep_drop = dz_drop > max_drop_mm && dz_drop > TAN_70_DEG * dxy;
        if is_steep_drop
            && is_overhang_shadow_transition(surface, cutter, a, b, &mut cand_buf, &mut ray_buf)
        {
            if cur.len() >= 2 {
                out.push(std::mem::take(&mut cur));
            } else {
                cur.clear();
            }
            cur.push(b);
        } else if dz_drop > 0.05 && dz_drop > TAN_70_DEG * dxy && dxy > 1e-4 {
            // Non-overhanging steep wall descent: step clear horizontally at high Z before descending
            cur.push(DVec3::new(b.x, b.y, a.z));
            cur.push(b);
        } else if dz_rise > 0.05 && dz_rise > TAN_70_DEG * dxy && dxy > 1e-4 {
            // Steep vertical wall / overhang climb: rise straight up in clear air before stepping onto crest
            cur.push(DVec3::new(a.x, a.y, b.z));
            cur.push(b);
        } else {
            cur.push(b);
        }
    }

    if cur.len() >= 2 {
        out.push(cur);
    }
    out
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ColumnGapCheck {
    OverhangGap(f64),
    SolidConnected,
    MissedUpper,
}

fn overhang_ceiling_above_air_gap(
    surface: &SurfaceMesh3D,
    contact_pt: DVec3,
    tri_idx: usize,
    ref_xy: DVec2,
    z_surf_lo: f64,
    ray_buf: &mut Vec<usize>,
) -> Option<f64> {
    if contact_pt.z <= z_surf_lo + 0.08 {
        return None;
    }
    let tri = &surface.triangles[tri_idx];
    let [i0, i1, i2] = tri.indices;
    let v0 = surface.vertices[i0 as usize];
    let v1 = surface.vertices[i1 as usize];
    let v2 = surface.vertices[i2 as usize];

    let c_xy = DVec2::new(contact_pt.x, contact_pt.y);
    let centroid_xy = DVec2::new((v0.x + v1.x + v2.x) / 3.0, (v0.y + v1.y + v2.y) / 3.0);
    let dir_in = (c_xy - ref_xy).normalize_or_zero();
    let has_dir = dir_in.length_squared() > 0.5;
    let perp = DVec2::new(-dir_in.y, dir_in.x);

    let mut probes = [DVec2::ZERO; 7];
    let mut n_probes = 0usize;
    if has_dir {
        probes[n_probes] = c_xy + dir_in * 0.08;
        n_probes += 1;
        probes[n_probes] = c_xy + dir_in * 0.02;
        n_probes += 1;
        // Grazing-corner fallback: step slightly to either lateral side of `dir_in` so rays
        // aligned directly with a roof corner edge still enter the overhanging solid cleanly.
        probes[n_probes] = c_xy + dir_in * 0.08 + perp * 0.06;
        n_probes += 1;
        probes[n_probes] = c_xy + dir_in * 0.08 - perp * 0.06;
        n_probes += 1;
    } else {
        probes[n_probes] = c_xy;
        n_probes += 1;
    }
    if tri.normal.z.abs() > 0.1 {
        probes[n_probes] = c_xy.lerp(centroid_xy, 0.25);
        n_probes += 1;
    }
    if has_dir {
        probes[n_probes] = c_xy + dir_in * 0.25;
        n_probes += 1;
    }

    for &q_xy in &probes[..n_probes] {
        match column_overhang_bottom_above_gap(
            surface,
            q_xy,
            z_surf_lo,
            contact_pt.z,
            tri.aabb.max.z,
            ray_buf,
        ) {
            ColumnGapCheck::OverhangGap(z_bot) => return Some(z_bot),
            ColumnGapCheck::SolidConnected => return None,
            ColumnGapCheck::MissedUpper => {}
        }
    }
    None
}

fn column_overhang_bottom_above_gap(
    surface: &SurfaceMesh3D,
    q_xy: DVec2,
    z_surf_lo: f64,
    z_contact: f64,
    z_tri_max: f64,
    ray_buf: &mut Vec<usize>,
) -> ColumnGapCheck {
    // Tiny deterministic XY perturbation prevents vertical rays from slipping through shared edges
    let q = DVec2::new(q_xy.x + 1.13e-7, q_xy.y + 2.37e-7);
    surface.bvh.query_xy_circle_into(q.x, q.y, 1e-5, ray_buf);
    if ray_buf.is_empty() {
        return ColumnGapCheck::MissedUpper;
    }

    let mut hits: Vec<(f64, f64)> = Vec::with_capacity(ray_buf.len().min(16));
    for &ti in ray_buf.iter() {
        let tri = &surface.triangles[ti];
        let nz = tri.normal.z;
        if nz.abs() <= 1e-5 {
            continue;
        }
        let [i0, i1, i2] = tri.indices;
        let v0 = surface.vertices[i0 as usize];
        let v1 = surface.vertices[i1 as usize];
        let v2 = surface.vertices[i2 as usize];
        let v0_2 = DVec2::new(v0.x, v0.y);
        let v1_2 = DVec2::new(v1.x, v1.y);
        let v2_2 = DVec2::new(v2.x, v2.y);
        if !point_in_triangle_2d(q, v0_2, v1_2, v2_2) {
            continue;
        }
        let n = tri.normal;
        let z_hit = v0.z - (n.x * (q.x - v0.x) + n.y * (q.y - v0.y)) / nz;
        if z_hit.is_finite() {
            hits.push((z_hit, nz));
        }
    }
    if hits.is_empty() {
        return ColumnGapCheck::MissedUpper;
    }

    // Sort ascending by Z; break near-coplanar ties with upward faces (`nz > 0`) first so flush
    // stacked-box internal seams (`+Z` coplanar with `-Z`) never register as an air gap.
    hits.sort_by(|a, b| {
        if (a.0 - b.0).abs() <= 1e-4 {
            b.1.total_cmp(&a.1)
        } else {
            a.0.total_cmp(&b.0)
        }
    });

    let z_upper_limit = z_contact.max(z_tri_max) + 0.15;
    let mut highest_below = z_surf_lo;
    let mut saw_upper_solid = false;

    for &(z_hit, nz) in &hits {
        if z_hit <= z_surf_lo + 0.05 {
            continue;
        }
        if z_hit > z_upper_limit {
            break;
        }
        if nz < -0.15 {
            // Confirm no coplanar upward-facing seam sits within 0.05 mm of this downward face
            let has_coplanar_up = hits
                .iter()
                .any(|&(zh, nzh)| nzh > 0.05 && (zh - z_hit).abs() <= 0.05);
            if !has_coplanar_up && z_hit - highest_below > 0.05 {
                return ColumnGapCheck::OverhangGap(z_hit);
            }
        } else if nz > 0.05 {
            if z_hit >= z_contact - 0.15 {
                saw_upper_solid = true;
                break;
            }
            highest_below = highest_below.max(z_hit);
        }
    }

    if saw_upper_solid {
        ColumnGapCheck::SolidConnected
    } else {
        ColumnGapCheck::MissedUpper
    }
}
