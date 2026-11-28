use chip_math::{Aabb3, DVec2, DVec3};
use chip_tools::{Tool, ToolAssemblyPart};
use rayon::prelude::*;

use crate::{
    toolpath::{SegmentMotion3D, ToolpathSegment3D},
    workholding::ResolvedFixtureSolid,
};

use super::{FixtureHit, MARGIN_CONTACT_EPS_MM, PHYSICAL_CONTACT_EPS_MM, ToolAssemblyProfile};

#[derive(Debug, Clone, Copy)]
struct AxialZoneBand {
    dz_lo: f64,
    dz_hi: f64,
    radius_mm: f64,
    part: ToolAssemblyPart,
    /// When `true`, uses exact lower profile `envelope.profile_dz_at_radius` for non-flat flutes.
    exact_flute_tip: bool,
    /// When `true`, uses linear cone inversion between `holder_cone_z0` and `holder_cone_z1`.
    exact_holder_cone: bool,
}

fn assembly_zone_bands(assembly: &ToolAssemblyProfile) -> ([AxialZoneBand; 4], usize) {
    let mut bands = [AxialZoneBand {
        dz_lo: 0.0,
        dz_hi: 0.0,
        radius_mm: 0.0,
        part: ToolAssemblyPart::Flute,
        exact_flute_tip: false,
        exact_holder_cone: false,
    }; 4];
    let mut n = 0usize;

    // 1. Cutting flutes
    bands[n] = AxialZoneBand {
        dz_lo: 0.0,
        dz_hi: assembly.flute_length_mm,
        radius_mm: assembly.cutting_radius_mm,
        part: ToolAssemblyPart::Flute,
        exact_flute_tip: true,
        exact_holder_cone: false,
    };
    n += 1;

    // 2. Non-cutting shank above flutes (when stickout > flute_length)
    if assembly.stickout_length_mm > assembly.flute_length_mm + 1e-6 {
        let shank_r = if assembly.has_relieved_neck {
            assembly.shank_radius_mm.max(assembly.neck_radius_mm)
        } else {
            assembly.shank_radius_mm
        };
        bands[n] = AxialZoneBand {
            dz_lo: assembly.flute_length_mm,
            dz_hi: assembly.stickout_length_mm,
            radius_mm: shank_r,
            part: ToolAssemblyPart::Shank,
            exact_flute_tip: false,
            exact_holder_cone: false,
        };
        n += 1;
    }

    // 3. Collet nut cylinder `[stickout, holder_cone_z0]`
    bands[n] = AxialZoneBand {
        dz_lo: assembly.stickout_length_mm,
        dz_hi: assembly.holder_cone_z0,
        radius_mm: assembly.holder_nut_radius_mm,
        part: ToolAssemblyPart::Holder,
        exact_flute_tip: false,
        exact_holder_cone: false,
    };
    n += 1;

    // 4. Tapered holder cone & upper spindle/holder body `[holder_cone_z0, +∞)`
    bands[n] = AxialZoneBand {
        dz_lo: assembly.holder_cone_z0,
        dz_hi: f64::INFINITY,
        radius_mm: assembly.holder_body_radius_mm,
        part: ToolAssemblyPart::Holder,
        exact_flute_tip: false,
        exact_holder_cone: true,
    };
    n += 1;

    (bands, n)
}

/// Checks whether a linear 3D tool-tip chord `p0 -> p1` collides with any fixture in `fixtures`.
///
/// - When `include_clearance_margin` is `true` (used by automatic rapid routing), each fixture's
///   `clearance_margin_mm` safety halo is enforced radially and vertically (with `0.0 mm` vertical
///   halo above a sacrificial spoilboard's top surface so stay-down links above the spoilboard do
///   not falsely lift).
/// - When `include_clearance_margin` is `false` (used by `chip-sim` Rule 6 verification), exact
///   physical contact is tested and `allowed_overcut_mm` is honored for non-rapid cutting flute
///   engagement into sacrificial fixtures such as [`crate::FixtureKind::Spoilboard`].
pub fn check_chord_fixture_collision(
    p0: DVec3,
    p1: DVec3,
    assembly: &ToolAssemblyProfile,
    fixtures: &[ResolvedFixtureSolid],
    include_clearance_margin: bool,
    is_rapid: bool,
) -> Option<FixtureHit> {
    if fixtures.is_empty() || !p0.is_finite() || !p1.is_finite() {
        return None;
    }
    let mut cand_buf = Vec::with_capacity(64);
    let mut best: Option<FixtureHit> = None;
    for fix in fixtures {
        if let Some(hit) = check_chord_vs_single_fixture(
            p0,
            p1,
            assembly,
            fix,
            include_clearance_margin,
            is_rapid,
            &mut cand_buf,
        ) && is_worse_hit(&hit, best.as_ref())
        {
            best = Some(hit);
        }
    }
    best
}

/// Checks a single [`ToolpathSegment3D`] (rapid, linear, or circular/helical arc) against all
/// `fixtures`, returning all distinct per-fixture worst [`FixtureHit`]s (at most one per fixture).
pub fn check_segment_fixture_collisions_all(
    seg: &ToolpathSegment3D,
    tool: &Tool,
    fixtures: &[ResolvedFixtureSolid],
    include_clearance_margin: bool,
) -> Vec<FixtureHit> {
    if fixtures.is_empty() {
        return Vec::new();
    }
    let assembly = ToolAssemblyProfile::from_tool(tool);
    check_segment_fixture_collisions_with_profile(
        seg,
        &assembly,
        fixtures,
        include_clearance_margin,
    )
}

/// Checks a single [`ToolpathSegment3D`] against `fixtures` using a precomputed
/// [`ToolAssemblyProfile`], returning the single worst [`FixtureHit`] if any fixture is hit.
pub fn check_segment_fixture_collision(
    seg: &ToolpathSegment3D,
    tool: &Tool,
    fixtures: &[ResolvedFixtureSolid],
    include_clearance_margin: bool,
) -> Option<FixtureHit> {
    let hits = check_segment_fixture_collisions_all(seg, tool, fixtures, include_clearance_margin);
    hits.into_iter()
        .reduce(|acc, h| if is_worse_hit(&h, Some(&acc)) { h } else { acc })
}

/// Fast precomputed-profile version returning at most one worst [`FixtureHit`] per colliding
/// fixture for `seg` (used by `chip-sim`'s parallel Rule 6 verifier).
pub fn check_segment_fixture_collisions_with_profile(
    seg: &ToolpathSegment3D,
    assembly: &ToolAssemblyProfile,
    fixtures: &[ResolvedFixtureSolid],
    include_clearance_margin: bool,
) -> Vec<FixtureHit> {
    if fixtures.is_empty() {
        return Vec::new();
    }
    let is_rapid = matches!(seg.motion, SegmentMotion3D::Rapid);
    let mut cand_buf = Vec::with_capacity(64);
    let mut per_fixture: Vec<Option<FixtureHit>> = vec![None; fixtures.len()];

    match seg.motion {
        SegmentMotion3D::Rapid | SegmentMotion3D::Linear { .. } => {
            for (fi, fix) in fixtures.iter().enumerate() {
                if let Some(hit) = check_chord_vs_single_fixture(
                    seg.start_wcs,
                    seg.end_wcs,
                    assembly,
                    fix,
                    include_clearance_margin,
                    is_rapid,
                    &mut cand_buf,
                ) {
                    per_fixture[fi] = Some(hit);
                }
            }
        }
        SegmentMotion3D::Arc {
            radius_mm,
            signed_sweep_rad,
            ..
        } => {
            let r = radius_mm.max(1e-6);
            let cos_half = (1.0 - (0.02 / r).min(0.5)).clamp(-1.0, 1.0);
            let max_dtheta = (2.0 * cos_half.acos()).clamp(0.02, 0.12);
            let steps = ((signed_sweep_rad.abs() / max_dtheta).ceil() as usize).clamp(4, 1_024);

            let mut prev = seg.start_wcs;
            for k in 1..=steps {
                let next = if k == steps {
                    seg.end_wcs
                } else {
                    seg.sample_point(k as f64 / steps as f64)
                };
                for (fi, fix) in fixtures.iter().enumerate() {
                    if let Some(hit) = check_chord_vs_single_fixture(
                        prev,
                        next,
                        assembly,
                        fix,
                        include_clearance_margin,
                        is_rapid,
                        &mut cand_buf,
                    ) && is_worse_hit(&hit, per_fixture[fi].as_ref())
                    {
                        per_fixture[fi] = Some(hit);
                    }
                }
                prev = next;
            }
        }
    }

    per_fixture.into_iter().flatten().collect()
}

/// Evaluates full-assembly swept collision checks in parallel (`rayon`) across a slice of
/// `(span_index, op_index, segment)` items against `fixtures`, returning `(span_index, hit)` pairs
/// in deterministic ascending `span_index` order.
///
/// Exact zero-cost no-op returning an empty `Vec` when `fixtures.is_empty()`.
pub fn check_segments_fixture_collisions_par(
    items: &[(usize, usize, ToolpathSegment3D)],
    profiles_by_op: &[ToolAssemblyProfile],
    fixtures: &[ResolvedFixtureSolid],
    include_clearance_margin: bool,
) -> Vec<(usize, FixtureHit)> {
    if fixtures.is_empty() || items.is_empty() {
        return Vec::new();
    }
    items
        .par_iter()
        .flat_map_iter(|&(span_idx, op_idx, ref seg)| {
            let hits = if let Some(assembly) = profiles_by_op.get(op_idx) {
                check_segment_fixture_collisions_with_profile(
                    seg,
                    assembly,
                    fixtures,
                    include_clearance_margin,
                )
            } else {
                Vec::new()
            };
            hits.into_iter().map(move |h| (span_idx, h))
        })
        .collect()
}

/// Computes the minimum safe tip Z (`mm` in Setup WCS) required for the full revolved cutter
/// assembly to traverse horizontally from `p0` to `p1` cleanly over all `fixtures` (including
/// each fixture's `clearance_margin_mm`).
///
/// Returns `None` if no fixture lies within the swept radial footprint of `p0 -> p1`.
pub(super) fn min_safe_z_over_fixtures_with_profile(
    p0: DVec3,
    p1: DVec3,
    assembly: &ToolAssemblyProfile,
    fixtures: &[ResolvedFixtureSolid],
) -> Option<f64> {
    if fixtures.is_empty() || !p0.is_finite() || !p1.is_finite() {
        return None;
    }

    let s0 = DVec2::new(p0.x, p0.y);
    let s1 = DVec2::new(p1.x, p1.y);
    let mut cand_buf = Vec::with_capacity(64);
    let mut best_z: Option<f64> = None;

    for fix in fixtures {
        if fix.mesh_wcs.is_empty() || !fix.aabb_wcs.is_valid() {
            continue;
        }
        let r_margin = fix.clearance_margin_mm.max(0.0);
        let v_margin = if fix.is_spoilboard && fix.allowed_overcut_mm > 0.0 {
            0.0
        } else {
            r_margin
        };
        let r_max = assembly.max_assembly_radius_mm() + r_margin;

        let swept_xy = Aabb3::new(
            DVec3::new(
                p0.x.min(p1.x) - r_max,
                p0.y.min(p1.y) - r_max,
                fix.aabb_wcs.min.z - 1.0,
            ),
            DVec3::new(
                p0.x.max(p1.x) + r_max,
                p0.y.max(p1.y) + r_max,
                fix.aabb_wcs.max.z + 1.0,
            ),
        );
        if !aabb_overlaps(&swept_xy, &fix.aabb_wcs) {
            continue;
        }

        fix.mesh_wcs.query_aabb_into(&swept_xy, &mut cand_buf);
        for &ti in &cand_buf {
            let tri = &fix.mesh_wcs.triangles[ti];
            if !aabb_overlaps(&swept_xy, &tri.aabb) {
                continue;
            }
            let v0 = fix.mesh_wcs.vertices[tri.indices[0] as usize];
            let v1 = fix.mesh_wcs.vertices[tri.indices[1] as usize];
            let v2 = fix.mesh_wcs.vertices[tri.indices[2] as usize];

            // Fast XY distance check to full triangle
            let poly = [v0, v1, v2];
            let (d_xy, _) = segment_to_convex_poly_xy_dist(s0, s1, &poly);
            if d_xy >= r_max {
                continue;
            }

            // Compute exact required lift Z across Flute, Shank, and Holder radial footprints
            let tri_top_z = tri.aabb.max.z;
            let r_eff = (d_xy - r_margin).max(0.0);
            if let Some((dz_min, _)) = assembly.min_dz_and_part_for_radius(r_eff) {
                let req = tri_top_z + v_margin - dz_min;
                if req.is_finite() && best_z.is_none_or(|b| req > b) {
                    best_z = Some(req);
                }
            }
        }
    }

    best_z
}

#[inline]
fn is_worse_hit(cand: &FixtureHit, current: Option<&FixtureHit>) -> bool {
    match current {
        None => true,
        Some(cur) => cand.penetration_mm > cur.penetration_mm + 1e-6,
    }
}

#[inline]
fn aabb_overlaps(a: &Aabb3, b: &Aabb3) -> bool {
    a.is_valid()
        && b.is_valid()
        && a.min.x <= b.max.x
        && a.max.x >= b.min.x
        && a.min.y <= b.max.y
        && a.max.y >= b.min.y
        && a.min.z <= b.max.z
        && a.max.z >= b.min.z
}

#[inline]
fn aabb_contains_point(a: &Aabb3, p: DVec3) -> bool {
    a.is_valid()
        && p.x >= a.min.x
        && p.x <= a.max.x
        && p.y >= a.min.y
        && p.y <= a.max.y
        && p.z >= a.min.z
        && p.z <= a.max.z
}

fn check_chord_vs_single_fixture(
    p0: DVec3,
    p1: DVec3,
    assembly: &ToolAssemblyProfile,
    fix: &ResolvedFixtureSolid,
    include_clearance_margin: bool,
    is_rapid: bool,
    cand_buf: &mut Vec<usize>,
) -> Option<FixtureHit> {
    if fix.mesh_wcs.is_empty() || !fix.aabb_wcs.is_valid() {
        return None;
    }

    let r_margin = if include_clearance_margin {
        fix.clearance_margin_mm.max(0.0)
    } else {
        0.0
    };
    let v_margin = if include_clearance_margin {
        if fix.is_spoilboard && fix.allowed_overcut_mm > 0.0 {
            0.0
        } else {
            fix.clearance_margin_mm.max(0.0)
        }
    } else {
        0.0
    };
    let contact_eps = if include_clearance_margin {
        MARGIN_CONTACT_EPS_MM
    } else {
        PHYSICAL_CONTACT_EPS_MM
    };
    let dxy = (p1.x - p0.x).hypot(p1.y - p0.y);
    let is_upward_retract = dxy <= 1e-5 && p1.z > p0.z;
    let overcut_tol = if !include_clearance_margin
        && (!is_rapid || is_upward_retract)
        && fix.allowed_overcut_mm > 0.0
    {
        fix.allowed_overcut_mm
    } else {
        0.0
    };

    let r_max = assembly.max_assembly_radius_mm() + r_margin;
    let z_tip_min = p0.z.min(p1.z);
    let swept_aabb = Aabb3::new(
        DVec3::new(
            p0.x.min(p1.x) - r_max,
            p0.y.min(p1.y) - r_max,
            z_tip_min - v_margin,
        ),
        DVec3::new(
            p0.x.max(p1.x) + r_max,
            p0.y.max(p1.y) + r_max,
            fix.aabb_wcs.max.z.max(z_tip_min) + v_margin,
        ),
    );
    if !aabb_overlaps(&swept_aabb, &fix.aabb_wcs) {
        return None;
    }

    let (bands, n_bands) = assembly_zone_bands(assembly);
    let active_bands = &bands[..n_bands];

    let dz_chord = (p1.z - p0.z).abs();
    let is_sloped = dxy > 1e-5 && dz_chord > 1e-5;

    fix.mesh_wcs.query_aabb_into(&swept_aabb, cand_buf);

    let mut best: Option<FixtureHit> = None;

    for &ti in cand_buf.iter() {
        let tri = &fix.mesh_wcs.triangles[ti];
        if tri.aabb.max.z + v_margin <= z_tip_min + contact_eps {
            continue;
        }
        if !aabb_overlaps(&swept_aabb, &tri.aabb) {
            continue;
        }
        let v0 = fix.mesh_wcs.vertices[tri.indices[0] as usize];
        let v1 = fix.mesh_wcs.vertices[tri.indices[1] as usize];
        let v2 = fix.mesh_wcs.vertices[tri.indices[2] as usize];

        if !is_sloped {
            // Horizontal or purely vertical chord: exact closed-form Z-slab polygon clip per band!
            let s0 = DVec2::new(p0.x, p0.y);
            let s1 = DVec2::new(p1.x, p1.y);
            evaluate_flat_or_vertical_chord_on_tri(
                s0,
                s1,
                z_tip_min,
                v0,
                v1,
                v2,
                assembly,
                active_bands,
                fix,
                r_margin,
                v_margin,
                overcut_tol,
                contact_eps,
                &mut best,
            );
        } else {
            // Sloped diagonal chord: narrow to XY overlap interval `[t_a, t_b]` and evaluate
            // exact sub-chords so `z_tip(t)` tracks the slope tightly without false triggers!
            let s0 = DVec2::new(p0.x, p0.y);
            let s1 = DVec2::new(p1.x, p1.y);
            let full_tri = [v0, v1, v2];
            let (d_all, _) = segment_to_convex_poly_xy_dist(s0, s1, &full_tri);
            if d_all >= r_max {
                continue;
            }
            let Some((t_a, t_b)) = segment_xy_aabb_interval(s0, s1, &tri.aabb, r_max) else {
                continue;
            };
            let za = p0.z + (p1.z - p0.z) * t_a;
            let zb = p0.z + (p1.z - p0.z) * t_b;
            let n_sub = (((zb - za).abs() / 0.25).ceil() as usize).clamp(1, 64);
            for k in 0..n_sub {
                let u0 = t_a + (t_b - t_a) * (k as f64 / n_sub as f64);
                let u1 = t_a + (t_b - t_a) * ((k + 1) as f64 / n_sub as f64);
                let sub_p0 = p0.lerp(p1, u0);
                let sub_p1 = p0.lerp(p1, u1);
                let sub_z = 0.5 * (sub_p0.z + sub_p1.z);
                evaluate_flat_or_vertical_chord_on_tri(
                    DVec2::new(sub_p0.x, sub_p0.y),
                    DVec2::new(sub_p1.x, sub_p1.y),
                    sub_z,
                    v0,
                    v1,
                    v2,
                    assembly,
                    active_bands,
                    fix,
                    r_margin,
                    v_margin,
                    overcut_tol,
                    contact_eps,
                    &mut best,
                );
            }
        }
    }

    // Watertight interior containment safeguard: if the tool tip sits completely inside a huge
    // fixture volume (e.g. deep plunge inside a box keep-out), an upward `+Z` raycast with odd
    // parity catches it cleanly and computes the exact top surface Z above the tool!
    if best.is_none()
        && (aabb_contains_point(&fix.aabb_wcs, p0)
            || aabb_contains_point(&fix.aabb_wcs, p1)
            || aabb_contains_point(&fix.aabb_wcs, p0.lerp(p1, 0.5)))
    {
        let mut ray_hits = Vec::with_capacity(8);
        for probe in [p0, p1, p0.lerp(p1, 0.5)] {
            if !aabb_contains_point(&fix.aabb_wcs, probe) {
                continue;
            }
            fix.mesh_wcs
                .raycast_all_hits(probe, DVec3::Z, f64::INFINITY, &mut ray_hits);
            // Deduplicate coincident shared-edge ray hits (`|t_i - t_prev| <= 1e-6`)
            let mut unique_ts: Vec<f64> = Vec::with_capacity(ray_hits.len());
            for rh in &ray_hits {
                if rh.t > 1e-6
                    && unique_ts
                        .last()
                        .is_none_or(|&prev| (rh.t - prev).abs() > 1e-5)
                {
                    unique_ts.push(rh.t);
                }
            }
            if unique_ts.len() % 2 == 1
                && let Some(&top_t) = unique_ts.last()
            {
                let top_z = probe.z + top_t;
                let mut pen = top_z + v_margin - probe.z;
                if overcut_tol > 0.0 {
                    let overcut_depth = fix.aabb_wcs.max.z - probe.z;
                    if overcut_depth <= overcut_tol + 1e-4 {
                        continue;
                    }
                    pen = (overcut_depth - overcut_tol).min(pen);
                }
                if pen > contact_eps {
                    let cand = FixtureHit {
                        fixture_id: fix.fixture_id.clone(),
                        fixture_name: fix.name.clone(),
                        contact_part: ToolAssemblyPart::Flute,
                        penetration_mm: pen,
                        required_lift_z_wcs: top_z + v_margin,
                        hit_point_wcs: probe,
                    };
                    if is_worse_hit(&cand, best.as_ref()) {
                        best = Some(cand);
                    }
                }
            }
        }
    }

    best
}

#[allow(clippy::too_many_arguments)]
fn evaluate_flat_or_vertical_chord_on_tri(
    s0: DVec2,
    s1: DVec2,
    z_tip: f64,
    v0: DVec3,
    v1: DVec3,
    v2: DVec3,
    assembly: &ToolAssemblyProfile,
    bands: &[AxialZoneBand],
    fix: &ResolvedFixtureSolid,
    r_margin: f64,
    v_margin: f64,
    overcut_tol: f64,
    contact_eps: f64,
    best: &mut Option<FixtureHit>,
) {
    let mut clipped = [DVec3::ZERO; 7];

    for band in bands {
        let z_lo = z_tip + band.dz_lo;
        let z_hi = if band.dz_hi.is_finite() {
            z_tip + band.dz_hi
        } else {
            f64::INFINITY
        };
        let slab_lo = z_lo - v_margin;
        let n_poly = chip_bvh::clip_triangle_to_z_slab(v0, v1, v2, slab_lo, z_hi, &mut clipped);
        if n_poly < 3 {
            continue;
        }
        let poly = &clipped[..n_poly];
        let r_allow = band.radius_mm + r_margin;
        let (d_xy, witness) = segment_to_convex_poly_xy_dist(s0, s1, poly);
        if d_xy >= r_allow - contact_eps {
            continue;
        }

        // Exact axial clearance for non-flat flute tips and tapered holder cones
        let r_eff = (d_xy - r_margin).max(0.0);
        let eff_dz_lo = if band.exact_flute_tip {
            match assembly.envelope.profile_dz_at_radius(r_eff) {
                Some(dz) => dz,
                None => continue,
            }
        } else if band.exact_holder_cone {
            if r_eff <= assembly.holder_nut_radius_mm {
                band.dz_lo
            } else {
                let dr = (assembly.holder_body_radius_mm - assembly.holder_nut_radius_mm).max(1e-9);
                let frac = ((r_eff - assembly.holder_nut_radius_mm) / dr).clamp(0.0, 1.0);
                assembly.holder_cone_z0 + frac * (assembly.holder_cone_z1 - assembly.holder_cone_z0)
            }
        } else {
            band.dz_lo
        };

        // Highest Z among clipped polygon points inside `r_allow` of the chord
        let mut poly_max_z = witness.z;
        for &pt in poly {
            let dp = chip_math::point_to_segment_dist_2d(DVec2::new(pt.x, pt.y), s0, s1);
            if dp <= r_allow + 1e-6 && pt.z > poly_max_z {
                poly_max_z = pt.z;
            }
        }

        let axial_pen = poly_max_z + v_margin - (z_tip + eff_dz_lo);
        if axial_pen <= contact_eps {
            continue;
        }

        // Exact band radius at witness height (for holder cone / conical flute)
        let dz_witness = (witness.z - z_tip).max(eff_dz_lo);
        let eff_band_r = if band.exact_flute_tip {
            assembly
                .envelope
                .radius_at_dz(dz_witness.min(assembly.flute_length_mm))
                .unwrap_or(band.radius_mm)
        } else if band.exact_holder_cone {
            assembly.holder_radius_at_dz(dz_witness)
        } else if band.part == ToolAssemblyPart::Shank
            && assembly.has_relieved_neck
            && dz_witness <= assembly.neck_length_mm
        {
            assembly.neck_radius_mm
        } else {
            band.radius_mm
        };

        let radial_pen = (eff_band_r + r_margin) - d_xy;
        if radial_pen <= contact_eps {
            continue;
        }

        let mut pen = radial_pen.min(axial_pen);

        // Honor sacrificial `allowed_overcut_mm` for cutting Flute contact (e.g. MDF spoilboard)
        if band.part == ToolAssemblyPart::Flute && overcut_tol > 0.0 {
            let overcut_depth = fix.aabb_wcs.max.z - z_tip;
            if overcut_depth <= overcut_tol + 1e-4 {
                continue;
            }
            pen = pen.min(overcut_depth - overcut_tol);
            if pen <= contact_eps {
                continue;
            }
        }

        let required_lift_z_wcs = poly_max_z + v_margin - eff_dz_lo;
        let cand = FixtureHit {
            fixture_id: fix.fixture_id.clone(),
            fixture_name: fix.name.clone(),
            contact_part: band.part,
            penetration_mm: pen,
            required_lift_z_wcs,
            hit_point_wcs: witness,
        };
        if is_worse_hit(&cand, best.as_ref()) {
            *best = Some(cand);
        }
    }
}

/// Computes the exact minimum 2D XY distance between 2D segment `(s0, s1)` and a 3D convex polygon
/// `poly` (3..=5 vertices, coplanar), along with a 3D witness point on `poly` achieving that
/// distance (choosing the highest-Z witness when `distance == 0.0`).
fn segment_to_convex_poly_xy_dist(s0: DVec2, s1: DVec2, poly: &[DVec3]) -> (f64, DVec3) {
    let n = poly.len();
    debug_assert!(n >= 3);

    // 1. Check if either segment endpoint lies inside the 2D projection of `poly`
    let mut best_zero_z: Option<DVec3> = None;
    let mut consider_zero = |pt: DVec3| {
        if best_zero_z.is_none_or(|b| pt.z > b.z) {
            best_zero_z = Some(pt);
        }
    };

    for &ep in &[s0, s1] {
        if let Some(z) = eval_point_in_convex_poly_xy(ep, poly) {
            consider_zero(DVec3::new(ep.x, ep.y, z));
        }
    }

    // 2. Check all polygon edges against `(s0, s1)` for exact 2D intersection or closest approach
    let mut best_dist = f64::INFINITY;
    let mut best_pt = poly[0];

    for i in 0..n {
        let a3 = poly[i];
        let b3 = poly[(i + 1) % n];
        let a2 = DVec2::new(a3.x, a3.y);
        let b2 = DVec2::new(b3.x, b3.y);
        let (d, u_edge) = chip_math::segment_to_segment_dist_2d(s0, s1, a2, b2);
        let q = a3.lerp(b3, u_edge);
        if d <= 1e-9 {
            consider_zero(q);
        } else if d < best_dist {
            best_dist = d;
            best_pt = q;
        }
    }

    if let Some(hit_pt) = best_zero_z {
        (0.0, hit_pt)
    } else {
        (best_dist, best_pt)
    }
}

/// Evaluates whether 2D point `p` lies inside (or on the boundary of) the 2D XY projection of
/// coplanar convex polygon `poly` (fan-triangulated around `poly[0]`), returning interpolated `z`.
fn eval_point_in_convex_poly_xy(p: DVec2, poly: &[DVec3]) -> Option<f64> {
    let v0 = poly[0];
    let p0 = DVec2::new(v0.x, v0.y);
    for i in 1..(poly.len() - 1) {
        let v1 = poly[i];
        let v2 = poly[i + 1];
        let p1 = DVec2::new(v1.x, v1.y);
        let p2 = DVec2::new(v2.x, v2.y);
        let e0 = p1 - p0;
        let e1 = p2 - p0;
        let area2 = e0.x * e1.y - e0.y * e1.x;
        if area2.abs() <= 1e-12 {
            continue;
        }
        let d = p - p0;
        let u = (d.x * e1.y - d.y * e1.x) / area2;
        let v = (e0.x * d.y - e0.y * d.x) / area2;
        let w = 1.0 - u - v;
        if u >= -1e-9 && v >= -1e-9 && w >= -1e-9 {
            return Some(w * v0.z + u * v1.z + v * v2.z);
        }
    }
    None
}

/// Computes the normalized parameter interval `[t_min, t_max] ⊆ [0, 1]` where 2D segment `(s0, s1)`
/// overlaps the XY AABB `aabb` inflated by `pad`.
fn segment_xy_aabb_interval(s0: DVec2, s1: DVec2, aabb: &Aabb3, pad: f64) -> Option<(f64, f64)> {
    let mut t_lo = 0.0_f64;
    let mut t_hi = 1.0_f64;
    let d = s1 - s0;
    let mins = [aabb.min.x - pad, aabb.min.y - pad];
    let maxs = [aabb.max.x + pad, aabb.max.y + pad];
    let orig = [s0.x, s0.y];
    let dir = [d.x, d.y];

    for axis in 0..2 {
        if dir[axis].abs() <= 1e-12 {
            if orig[axis] < mins[axis] || orig[axis] > maxs[axis] {
                return None;
            }
        } else {
            let inv = 1.0 / dir[axis];
            let mut ta = (mins[axis] - orig[axis]) * inv;
            let mut tb = (maxs[axis] - orig[axis]) * inv;
            if ta > tb {
                std::mem::swap(&mut ta, &mut tb);
            }
            t_lo = t_lo.max(ta);
            t_hi = t_hi.min(tb);
            if t_lo > t_hi {
                return None;
            }
        }
    }
    Some((t_lo, t_hi))
}
