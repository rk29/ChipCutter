use chip_bvh::TriMesh3D;
use chip_math::{Aabb3, COINCIDENCE_TOL_MM, DVec2, DVec3};
use chip_tools::{AnalyticCutterEnvelope, CutterEnvelopeShape};

use crate::rest2d::RegionSet2D;

use super::{
    DeburrContactSolution, DeburrEdgeChain, DeburrSample3D, SolvedStation, TrimmedSubChain,
    solve_deburr_tool_tip,
};

#[inline]
pub(super) fn tip_xy_in_containment(
    tip_wcs: DVec3,
    xy_containment: Option<&RegionSet2D>,
    avoid_keepout: Option<&RegionSet2D>,
) -> bool {
    let pt = DVec2::new(tip_wcs.x, tip_wcs.y);
    if avoid_keepout.is_some_and(|av| av.contains_point(pt)) {
        return false;
    }
    xy_containment
        .is_none_or(|reg| reg.contains_point(pt) || reg.min_distance_to_boundary(pt) <= 0.02)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn trim_chain_against_target(
    chain: &DeburrEdgeChain,
    tool: &chip_tools::Tool,
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    target_mesh: &TriMesh3D,
    chamfer_w: f64,
    tip_offset: f64,
    wall_clearance: f64,
    allow_underside: bool,
    xy_containment: Option<&RegionSet2D>,
    avoid_keepout: Option<&RegionSet2D>,
    cand_buf: &mut Vec<usize>,
) -> Vec<TrimmedSubChain> {
    let n = chain.samples.len();
    if n < 2 {
        return Vec::new();
    }

    let mut solved: Vec<Option<DeburrContactSolution>> = Vec::with_capacity(n);
    let mut free: Vec<bool> = Vec::with_capacity(n);

    for s in &chain.samples {
        let sol = solve_deburr_tool_tip(
            s,
            &tool.geometry,
            chamfer_w,
            tip_offset,
            wall_clearance,
            allow_underside,
        );
        let ok = if let Some(cs) = sol {
            tip_xy_in_containment(cs.tip_wcs, xy_containment, avoid_keepout)
                && is_station_collision_free(
                    s,
                    &cs,
                    envelope,
                    stickout_mm,
                    target_mesh,
                    chamfer_w,
                    wall_clearance,
                    cand_buf,
                )
        } else {
            false
        };
        solved.push(sol);
        free.push(ok);
    }

    let all_free = free.iter().all(|&ok| ok);
    if all_free {
        let stations: Vec<SolvedStation> = chain
            .samples
            .iter()
            .zip(solved.iter())
            .filter_map(|(&sample, sol)| sol.map(|sol| SolvedStation { sample, sol }))
            .collect();
        if chain_length(&stations) >= 0.05 {
            return vec![TrimmedSubChain {
                stations,
                closed: chain.closed,
                is_underside: chain.is_underside,
            }];
        }
        return Vec::new();
    }

    // Split into contiguous collision-free runs with binary boundary refinement (`<= 0.02 mm`)!
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

        let mut stations = Vec::with_capacity(end - start + 3);

        // Refine entering boundary (`start - 1` colliding -> `start` collision-free)
        if start > 0
            && let Some(refined) = binary_refine_boundary(
                &chain.samples[start],
                &chain.samples[start - 1],
                tool,
                envelope,
                stickout_mm,
                target_mesh,
                chamfer_w,
                tip_offset,
                wall_clearance,
                allow_underside,
                xy_containment,
                avoid_keepout,
                cand_buf,
            )
            && refined.sol.tip_wcs.distance(
                solved[start]
                    .map(|s| s.tip_wcs)
                    .unwrap_or(refined.sol.tip_wcs),
            ) > 1e-4
        {
            stations.push(refined);
        }

        for k in start..=end {
            if let Some(sol) = solved[k] {
                stations.push(SolvedStation {
                    sample: chain.samples[k],
                    sol,
                });
            }
        }

        // Refine exiting boundary (`end` collision-free -> `end + 1` colliding)
        if end + 1 < n
            && let Some(refined) = binary_refine_boundary(
                &chain.samples[end],
                &chain.samples[end + 1],
                tool,
                envelope,
                stickout_mm,
                target_mesh,
                chamfer_w,
                tip_offset,
                wall_clearance,
                allow_underside,
                xy_containment,
                avoid_keepout,
                cand_buf,
            )
            && let Some(last) = stations.last()
            && refined.sol.tip_wcs.distance(last.sol.tip_wcs) > 1e-4
        {
            stations.push(refined);
        }

        if stations.len() >= 2 && chain_length(&stations) >= 0.05 {
            subchains.push(TrimmedSubChain {
                stations,
                closed: false,
                is_underside: chain.is_underside,
            });
        }
    }

    // If the original chain was closed and both the first and last subchains touch the seam,
    // merge them into one seamless open subchain!
    if chain.closed
        && subchains.len() >= 2
        && let (Some(first_pt), Some(last_pt)) = (
            subchains.first().and_then(|s| s.stations.first()),
            subchains.last().and_then(|s| s.stations.last()),
        )
        && first_pt.sample.pos_wcs.distance(last_pt.sample.pos_wcs) <= COINCIDENCE_TOL_MM * 4.0
    {
        let mut tail = subchains.pop().unwrap_or(TrimmedSubChain {
            stations: Vec::new(),
            closed: false,
            is_underside: chain.is_underside,
        });
        let head = subchains.remove(0);
        if let (Some(tail_last), Some(head_first)) = (tail.stations.last(), head.stations.first()) {
            let same_normal = tail_last.sol.outward_xy.dot(head_first.sol.outward_xy) >= 0.995;
            if same_normal {
                tail.stations.extend_from_slice(&head.stations[1..]);
            } else {
                // Seam sits on a sharp convex corner: keep `head.stations[0]` so `emit_deburr_subchain`
                // sees `corner_dist <= 0.25` and emits a smooth `emit_corner_roll` around the miter!
                tail.stations.extend_from_slice(&head.stations);
            }
        }
        subchains.insert(0, tail);
    }

    subchains
}

/// Binary-refines between a collision-free sample `good` (`u = 0`) and a colliding sample `bad`
/// (`u = 1`) down to `< 0.005 mm` (8 iterations on `<= 0.5 mm` step).
#[allow(clippy::too_many_arguments)]
fn binary_refine_boundary(
    good: &DeburrSample3D,
    bad: &DeburrSample3D,
    tool: &chip_tools::Tool,
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    target_mesh: &TriMesh3D,
    chamfer_w: f64,
    tip_offset: f64,
    wall_clearance: f64,
    allow_underside: bool,
    xy_containment: Option<&RegionSet2D>,
    avoid_keepout: Option<&RegionSet2D>,
    cand_buf: &mut Vec<usize>,
) -> Option<SolvedStation> {
    let mut lo = 0.0_f64;
    let mut hi = 1.0_f64;
    let mut best: Option<SolvedStation> = None;

    for _ in 0..8 {
        let mid_u = 0.5 * (lo + hi);
        let sample = lerp_sample(good, bad, mid_u);
        let ok = if let Some(sol) = solve_deburr_tool_tip(
            &sample,
            &tool.geometry,
            chamfer_w,
            tip_offset,
            wall_clearance,
            allow_underside,
        ) {
            if tip_xy_in_containment(sol.tip_wcs, xy_containment, avoid_keepout)
                && is_station_collision_free(
                    &sample,
                    &sol,
                    envelope,
                    stickout_mm,
                    target_mesh,
                    chamfer_w,
                    wall_clearance,
                    cand_buf,
                )
            {
                best = Some(SolvedStation { sample, sol });
                true
            } else {
                false
            }
        } else {
            false
        };
        if ok {
            lo = mid_u;
        } else {
            hi = mid_u;
        }
    }

    best
}

fn lerp_sample(a: &DeburrSample3D, b: &DeburrSample3D, u: f64) -> DeburrSample3D {
    let pos_wcs = a.pos_wcs.lerp(b.pos_wcs, u);
    let tangent = a.tangent.lerp(b.tangent, u).normalize_or_zero();
    let n1 = a.n1.lerp(b.n1, u).normalize_or_zero();
    let n2 = a.n2.lerp(b.n2, u).normalize_or_zero();
    let bisector_out = (n1 + n2).normalize_or_zero();
    DeburrSample3D {
        pos_wcs,
        tangent: if tangent.length_squared() > 0.25 {
            tangent
        } else {
            a.tangent
        },
        n1: if n1.length_squared() > 0.25 { n1 } else { a.n1 },
        n2: if n2.length_squared() > 0.25 { n2 } else { a.n2 },
        bisector_out: if bisector_out.length_squared() > 0.25 {
            bisector_out
        } else {
            a.bisector_out
        },
        is_underside: a.is_underside,
        face1: a.face1,
        face2: a.face2,
    }
}

fn chain_length(stations: &[SolvedStation]) -> f64 {
    stations
        .windows(2)
        .map(|w| w[0].sol.tip_wcs.distance(w[1].sol.tip_wcs))
        .sum()
}

/// Checks whether the full revolved cutter assembly (flutes + relieved neck + cylindrical shank)
/// at `sol.tip_wcs` clears all non-bevel geometry of `target_mesh` by at least `wall_clearance`.
#[allow(clippy::too_many_arguments)]
fn is_station_collision_free(
    sample: &DeburrSample3D,
    sol: &DeburrContactSolution,
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    target_mesh: &TriMesh3D,
    chamfer_w: f64,
    wall_clearance: f64,
    cand_buf: &mut Vec<usize>,
) -> bool {
    if target_mesh.triangles.is_empty() {
        return true;
    }

    let tip = sol.tip_wcs;
    let r_max =
        envelope.cutting_radius_mm().max(envelope.shank_radius_mm()) + wall_clearance + 0.05;
    let query_aabb = Aabb3::new(
        DVec3::new(tip.x - r_max, tip.y - r_max, tip.z - wall_clearance - 0.05),
        DVec3::new(tip.x + r_max, tip.y + r_max, tip.z + stickout_mm),
    );
    target_mesh.bvh.query_aabb_into(&query_aabb, cand_buf);

    let f1_id = sample.face1.0;
    let f2_id = sample.face2.0;
    let flute_len = envelope.flute_length_mm();
    let neck_reach = envelope.effective_reach_mm();
    let tip_profile_h = envelope.tip_profile_height_mm();
    let max_chord_excl = sol.contact_chord_mm + wall_clearance + 0.25;

    for &tri_idx in cand_buf.iter() {
        let tri = target_mesh.triangles[tri_idx];
        let v0 = target_mesh.vertices[tri.indices[0] as usize];
        let v1 = target_mesh.vertices[tri.indices[1] as usize];
        let v2 = target_mesh.vertices[tri.indices[2] as usize];

        let tri_z_min = v0.z.min(v1.z).min(v2.z);
        let tri_z_max = v0.z.max(v1.z).max(v2.z);
        let z_lo = tri_z_min.max(tip.z - wall_clearance);
        let z_hi = tri_z_max.min(tip.z + stickout_mm);
        if z_hi < z_lo - 1e-6 {
            continue;
        }

        let is_adj_face = tri.face_id == f1_id || tri.face_id == f2_id;
        let same_adj_side =
            is_adj_face && (tri.normal.dot(sample.n1) > 0.35 || tri.normal.dot(sample.n2) > 0.35);

        // Collect key Z test planes across `[z_lo, z_hi]`
        let mut z_planes = [0.0_f64; 8];
        let mut n_key = 0usize;
        z_planes[n_key] = z_lo;
        n_key += 1;
        if (z_hi - z_lo).abs() > 1e-6 {
            z_planes[n_key] = z_hi;
            n_key += 1;
        }
        for dz_crit in [0.0, tip_profile_h, flute_len, neck_reach] {
            let zc = tip.z + dz_crit;
            if zc > z_lo + 1e-5 && zc < z_hi - 1e-5 && n_key < z_planes.len() {
                z_planes[n_key] = zc;
                n_key += 1;
            }
        }

        // Uniform Z sub-samples every <= 0.5 mm across the triangle's active overlap
        let z_span = (z_hi - z_lo).max(0.0);
        let n_sub = ((z_span / 0.5).ceil() as usize).clamp(1, 64);

        let check_z = |zk: f64| -> bool {
            let Some(q) =
                chip_bvh::closest_point_on_triangle_at_z(v0, v1, v2, zk, DVec2::new(tip.x, tip.y))
            else {
                return true;
            };
            let dz = zk - tip.z;
            let r_axis = (q.x - tip.x).hypot(q.y - tip.y);

            let Some(r_lim) =
                cutter_inflated_radius_at_dz(envelope, stickout_mm, dz, wall_clearance)
            else {
                return true;
            };
            if r_axis >= r_lim - 1e-5 {
                return true;
            }

            // Non-cutting relieved neck or cylindrical shank is NEVER exempted from collision!
            if dz > flute_len + 1e-4 {
                return false;
            }

            // Below cutter tip (vertical floor clearance check)
            if dz < -1e-4 {
                return false;
            }

            // Inside the active flute zone: check whether `q` lies strictly inside the intentional
            // bevel wedge of the active edge!
            if is_point_in_intentional_bevel_wedge(
                q,
                sample,
                sol,
                same_adj_side,
                chamfer_w,
                max_chord_excl,
            ) {
                return true;
            }

            false
        };

        for &zk in &z_planes[..n_key] {
            if !check_z(zk) {
                return false;
            }
        }
        if n_sub > 1 {
            for s in 1..n_sub {
                let zk = z_lo + z_span * (s as f64) / (n_sub as f64);
                if !check_z(zk) {
                    return false;
                }
            }
        }
    }

    true
}

/// Checks whether a purely vertical plunge/retract column `(xy, z_start ..= z_top)` is completely
/// collision-free against `target_mesh` (with zero bevel exemption!).
pub(super) fn is_vertical_column_clear(
    tip_bottom: DVec3,
    z_top: f64,
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    target_mesh: &TriMesh3D,
    wall_clearance: f64,
    cand_buf: &mut Vec<usize>,
) -> bool {
    if target_mesh.triangles.is_empty() {
        return true;
    }
    let r_max =
        envelope.cutting_radius_mm().max(envelope.shank_radius_mm()) + wall_clearance + 0.05;
    let z_lo_all = tip_bottom.z - wall_clearance;
    let z_hi_all = z_top.max(tip_bottom.z) + stickout_mm;
    let query_aabb = Aabb3::new(
        DVec3::new(tip_bottom.x - r_max, tip_bottom.y - r_max, z_lo_all),
        DVec3::new(tip_bottom.x + r_max, tip_bottom.y + r_max, z_hi_all),
    );
    target_mesh.bvh.query_aabb_into(&query_aabb, cand_buf);
    if cand_buf.is_empty() {
        return true;
    }

    // Check swept vertical column: at any Z >= tip_bottom.z, the maximum radius of any part of
    // the cutter passing through Z during the vertical stroke `[tip_bottom.z, z_top]` is
    // `max(cutting_radius, shank_radius)`!
    let r_swept = envelope.cutting_radius_mm().max(envelope.shank_radius_mm()) + wall_clearance;
    let xy = DVec2::new(tip_bottom.x, tip_bottom.y);

    for &tri_idx in cand_buf.iter() {
        let tri = target_mesh.triangles[tri_idx];
        let v0 = target_mesh.vertices[tri.indices[0] as usize];
        let v1 = target_mesh.vertices[tri.indices[1] as usize];
        let v2 = target_mesh.vertices[tri.indices[2] as usize];
        let tri_z_max = v0.z.max(v1.z).max(v2.z);
        if tri_z_max < z_lo_all {
            continue;
        }
        // Project triangle to XY: if its XY distance to `xy` is < r_swept while above `z_lo_all`,
        // test closest 3D point on the triangle above `z_lo_all`!
        let tri_z_min = v0.z.min(v1.z).min(v2.z);
        let z_a = tri_z_min.max(z_lo_all);
        let z_b = tri_z_max.min(z_hi_all);
        if z_b < z_a - 1e-6 {
            continue;
        }
        for zk in [z_a, 0.5 * (z_a + z_b), z_b] {
            if let Some(q) = chip_bvh::closest_point_on_triangle_at_z(v0, v1, v2, zk, xy) {
                let r_axis = (q.x - xy.x).hypot(q.y - xy.y);
                let r_lim = if zk < tip_bottom.z {
                    cutter_inflated_radius_at_dz(
                        envelope,
                        stickout_mm,
                        zk - tip_bottom.z,
                        wall_clearance,
                    )
                    .unwrap_or(0.0)
                } else {
                    r_swept
                };
                if r_axis < r_lim - 1e-5 {
                    return false;
                }
            }
        }
    }

    true
}

/// Checks whether a horizontal transit point `tip` (outside the cut, e.g. on a lead-in/lead-out
/// arc or approach bridge) clears the target mesh completely with zero bevel exemption.
pub(super) fn is_air_position_clear(
    tip: DVec3,
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    target_mesh: &TriMesh3D,
    wall_clearance: f64,
    cand_buf: &mut Vec<usize>,
) -> bool {
    if target_mesh.triangles.is_empty() {
        return true;
    }
    let r_max =
        envelope.cutting_radius_mm().max(envelope.shank_radius_mm()) + wall_clearance + 0.05;
    let query_aabb = Aabb3::new(
        DVec3::new(tip.x - r_max, tip.y - r_max, tip.z - wall_clearance - 0.05),
        DVec3::new(tip.x + r_max, tip.y + r_max, tip.z + stickout_mm),
    );
    target_mesh.bvh.query_aabb_into(&query_aabb, cand_buf);
    let xy = DVec2::new(tip.x, tip.y);

    for &tri_idx in cand_buf.iter() {
        let tri = target_mesh.triangles[tri_idx];
        let v0 = target_mesh.vertices[tri.indices[0] as usize];
        let v1 = target_mesh.vertices[tri.indices[1] as usize];
        let v2 = target_mesh.vertices[tri.indices[2] as usize];
        let z_lo = v0.z.min(v1.z).min(v2.z).max(tip.z - wall_clearance);
        let z_hi = v0.z.max(v1.z).max(v2.z).min(tip.z + stickout_mm);
        if z_hi < z_lo - 1e-6 {
            continue;
        }
        let z_span = (z_hi - z_lo).max(0.0);
        let n_sub = ((z_span / 0.5).ceil() as usize).clamp(1, 32);
        for s in 0..=n_sub {
            let zk = z_lo + z_span * (s as f64) / (n_sub as f64);
            if let Some(q) = chip_bvh::closest_point_on_triangle_at_z(v0, v1, v2, zk, xy) {
                let dz = zk - tip.z;
                if let Some(r_lim) =
                    cutter_inflated_radius_at_dz(envelope, stickout_mm, dz, wall_clearance)
                    && (q.x - xy.x).hypot(q.y - xy.y) < r_lim - 1e-5
                {
                    return false;
                }
            }
        }
    }
    true
}

/// Evaluates the cutter's revolved radius at height `dz` above `tip_wcs`, inflated outward in 3D by
/// `wall_clearance` (handling Ball, Lollipop sphere + relieved neck + shank, and Conical flanks).
fn cutter_inflated_radius_at_dz(
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    dz: f64,
    wall_clearance: f64,
) -> Option<f64> {
    let wc = wall_clearance.max(0.0);
    if !dz.is_finite() || dz < -wc - 1e-6 || dz > stickout_mm + 1e-6 {
        return None;
    }

    match envelope.shape() {
        CutterEnvelopeShape::Ball { radius_mm } => {
            if dz < radius_mm {
                let r_inf = radius_mm + wc;
                let dy = dz - radius_mm;
                let disc = r_inf * r_inf - dy * dy;
                if disc <= 0.0 { None } else { Some(disc.sqrt()) }
            } else if dz <= envelope.flute_length_mm() + 1e-6 {
                Some(radius_mm + wc)
            } else {
                Some(envelope.shank_radius_mm() + wc)
            }
        }
        CutterEnvelopeShape::Lollipop {
            sphere_radius_mm,
            neck_radius_mm,
            neck_length_mm,
            z_neck_join_mm,
            ..
        } => {
            let r_inf = sphere_radius_mm + wc;
            let dy = dz - sphere_radius_mm;
            let sphere_r = if dy.abs() < r_inf {
                Some((r_inf * r_inf - dy * dy).max(0.0).sqrt())
            } else {
                None
            };
            if dz <= z_neck_join_mm + wc {
                // Take the union of the inflated sphere and the relieved neck above the join
                let neck_r = if dz >= z_neck_join_mm - 1e-6 {
                    Some(neck_radius_mm + wc)
                } else {
                    None
                };
                match (sphere_r, neck_r) {
                    (Some(a), Some(b)) => Some(a.max(b)),
                    (Some(a), None) => Some(a),
                    (None, Some(b)) => Some(b),
                    (None, None) => None,
                }
            } else if dz <= neck_length_mm + 1e-6 {
                Some(neck_radius_mm + wc)
            } else {
                Some(envelope.shank_radius_mm() + wc)
            }
        }
        CutterEnvelopeShape::Conical {
            tip_radius_mm,
            upper_radius_mm,
            half_angle_rad,
            tan_half_angle,
        } => {
            if dz < 0.0 {
                // Within `wall_clearance` directly below the tip
                Some(tip_radius_mm + wc)
            } else if dz <= envelope.flute_length_mm() + 1e-6 {
                let radial_infl = wc / half_angle_rad.cos().max(0.1);
                Some((tip_radius_mm + dz * tan_half_angle + radial_infl).min(upper_radius_mm + wc))
            } else {
                Some(envelope.shank_radius_mm() + wc)
            }
        }
        _ => {
            if dz < 0.0 {
                Some(envelope.cutting_radius_mm() + wc)
            } else {
                envelope
                    .cutter_radius_at_z_within(dz, stickout_mm)
                    .map(|r| r + wc)
            }
        }
    }
}

/// Convex-Corner Bevel-Wedge Lemma: returns `true` iff `q` lies strictly inside the intentional
/// chamfered corner material around `sample.pos_wcs` (never exempting protruding neighbor walls,
/// inside pocket corners, raised stepped bosses, lower floors, or opposite bore walls!).
fn is_point_in_intentional_bevel_wedge(
    q: DVec3,
    sample: &DeburrSample3D,
    sol: &DeburrContactSolution,
    same_adj_side: bool,
    chamfer_w: f64,
    max_chord_excl: f64,
) -> bool {
    let dist_to_edge = q.distance(sample.pos_wcs);
    if dist_to_edge > max_chord_excl {
        return false;
    }

    let delta = q - sample.pos_wcs;
    let d1 = delta.dot(sample.n1);
    let d2 = delta.dot(sample.n2);
    let bevel_signed = (q - sol.bevel_point_wcs).dot(sol.bevel_normal_wcs);

    if same_adj_side {
        // Active adjacent face (`f1` or `f2`, same orientation side): allow slight positive sag
        // along curved internal hole rims while still verifying `q` is inside the bevel cut depth.
        let curv_allow = 0.08 + 0.15 * chamfer_w;
        d1 <= curv_allow && d2 <= curv_allow && bevel_signed >= -(0.12 + 0.25 * chamfer_w)
    } else {
        // Non-adjacent face (e.g., perpendicular side face at an outer convex box corner):
        // ONLY exempt when `q` lies strictly on the SOLID side of BOTH `n1` and `n2` AND inside
        // the bevel wedge! Protruding inside pocket walls and raised bosses fail `d1 <= 0.03` or
        // `d2 <= 0.03` immediately!
        d1 <= 0.03 && d2 <= 0.03 && bevel_signed >= -0.06
    }
}
