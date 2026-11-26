//! Full-assembly swept-cutter vs. 3D workholding fixture `FlatBvh` collision kernel and
//! automatic safe-rapid / non-cutting link routing (`M61`).
//!
//! All calculations operate in exact `f64` Setup WCS millimeters against [`ResolvedFixtureSolid`]
//! watertight triangle meshes and their [`chip_bvh::FlatBvh`] spatial indexes—strictly UI-agnostic.

use chip_math::{Aabb3, DVec2, DVec3};
use chip_tools::{AnalyticCutterEnvelope, Tool};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::{
    cl_data::{ClEvent, MotionRole},
    feed_opt::rebuild_segments_from_events,
    setup::ResolvedSetup,
    toolpath::{GeneratedToolpath, SegmentMotion3D, ToolpathSegment3D},
    workholding::ResolvedFixtureSolid,
};

/// Physical portion of the revolved cutter assembly that contacted a workholding fixture.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolAssemblyPart {
    Flute,
    Shank,
    Holder,
}

impl ToolAssemblyPart {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Flute => "Cutter flutes",
            Self::Shank => "Non-cutting shank",
            Self::Holder => "Holder/collet",
        }
    }
}

/// Exact full-assembly collision finding against a [`ResolvedFixtureSolid`].
#[derive(Debug, Clone, PartialEq)]
pub struct FixtureHit {
    pub fixture_id: String,
    pub fixture_name: String,
    pub contact_part: ToolAssemblyPart,
    pub penetration_mm: f64,
    pub required_lift_z_wcs: f64,
    pub hit_point_wcs: DVec3,
}

/// Summary telemetry produced by [`route_toolpath_around_fixtures`].
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FixtureRoutingSummary {
    pub rapid_lifts_inserted: usize,
    pub max_lifted_z_wcs: f64,
}

/// Full revolved tool assembly profile (cutting flutes, non-cutting shank/neck, collet nut, and
/// tapered holder body) derived from [`Tool`] and [`AnalyticCutterEnvelope`].
#[derive(Debug, Clone, Copy)]
pub struct ToolAssemblyProfile {
    pub envelope: AnalyticCutterEnvelope,
    pub cutting_radius_mm: f64,
    pub flute_length_mm: f64,
    pub neck_radius_mm: f64,
    pub neck_length_mm: f64,
    pub is_thread_mill: bool,
    pub has_relieved_neck: bool,
    pub shank_radius_mm: f64,
    pub stickout_length_mm: f64,
    pub holder_nut_radius_mm: f64,
    pub holder_body_radius_mm: f64,
    pub holder_cone_z0: f64,
    pub holder_cone_z1: f64,
}

impl ToolAssemblyProfile {
    /// Builds the full revolved assembly envelope matching `chip-stock`'s `RadialCutterProfile`
    /// and `chip-sim`'s 3D cutter visualization (`nut_r = max(shank_r * 2.2, 12.0, shank_r + 4.0)`).
    pub fn from_tool(tool: &Tool) -> Self {
        let envelope = AnalyticCutterEnvelope::from_geometry(&tool.geometry);
        let cutting_radius_mm = envelope.cutting_radius_mm();
        let flute_length_mm = envelope.flute_length_mm();
        let neck_radius_mm = envelope.neck_radius_mm();
        let neck_length_mm = envelope.effective_reach_mm();
        let is_thread_mill = envelope.is_thread_mill();
        let has_relieved_neck = envelope.has_relieved_neck();
        let shank_radius_mm = envelope.shank_radius_mm();
        let stickout_length_mm = tool.stickout_length_mm.max(flute_length_mm);
        let holder_nut_radius_mm = (shank_radius_mm * 2.2).max(12.0).max(shank_radius_mm + 4.0);
        let holder_body_radius_mm = holder_nut_radius_mm * 1.25;
        let holder_cone_z0 = stickout_length_mm + 18.0;
        let holder_cone_z1 = stickout_length_mm + 28.0;

        Self {
            envelope,
            cutting_radius_mm,
            flute_length_mm,
            neck_radius_mm,
            neck_length_mm,
            is_thread_mill,
            has_relieved_neck,
            shank_radius_mm,
            stickout_length_mm,
            holder_nut_radius_mm,
            holder_body_radius_mm,
            holder_cone_z0,
            holder_cone_z1,
        }
    }

    #[inline]
    pub fn max_assembly_radius_mm(&self) -> f64 {
        self.cutting_radius_mm
            .max(self.shank_radius_mm)
            .max(self.holder_body_radius_mm)
    }

    /// Evaluates the collet nut / tapered holder cone radius at `dz >= stickout_length_mm`.
    #[inline]
    pub fn holder_radius_at_dz(&self, dz: f64) -> f64 {
        if dz <= self.holder_cone_z0 {
            self.holder_nut_radius_mm
        } else if dz < self.holder_cone_z1 {
            let span = (self.holder_cone_z1 - self.holder_cone_z0).max(1e-6);
            let u = ((dz - self.holder_cone_z0) / span).clamp(0.0, 1.0);
            self.holder_nut_radius_mm + (self.holder_body_radius_mm - self.holder_nut_radius_mm) * u
        } else {
            self.holder_body_radius_mm
        }
    }

    /// Returns the lowest axial height `dz >= 0` above the tip where the assembly reaches radial
    /// distance `r_eff` from the spindle axis, paired with the corresponding [`ToolAssemblyPart`].
    #[inline]
    pub fn min_dz_and_part_for_radius(&self, r_eff: f64) -> Option<(f64, ToolAssemblyPart)> {
        if !r_eff.is_finite() || r_eff < 0.0 {
            return None;
        }
        if r_eff <= self.cutting_radius_mm + 1e-9
            && let Some(dz) = self.envelope.profile_dz_at_radius(r_eff)
        {
            return Some((dz, ToolAssemblyPart::Flute));
        }
        if self.stickout_length_mm > self.flute_length_mm + 1e-6
            && r_eff <= self.shank_radius_mm + 1e-9
        {
            let shank_z = if self.has_relieved_neck {
                if r_eff <= self.neck_radius_mm + 1e-9 {
                    self.flute_length_mm
                } else {
                    self.neck_length_mm
                }
            } else {
                self.flute_length_mm
            };
            return Some((shank_z, ToolAssemblyPart::Shank));
        }
        if r_eff <= self.holder_nut_radius_mm + 1e-9 {
            return Some((self.stickout_length_mm, ToolAssemblyPart::Holder));
        }
        if r_eff <= self.holder_body_radius_mm + 1e-9 {
            let dr = (self.holder_body_radius_mm - self.holder_nut_radius_mm).max(1e-9);
            let frac = ((r_eff - self.holder_nut_radius_mm) / dr).clamp(0.0, 1.0);
            let dz = self.holder_cone_z0 + frac * (self.holder_cone_z1 - self.holder_cone_z0);
            return Some((dz, ToolAssemblyPart::Holder));
        }
        None
    }
}

/// Physical grazing tolerance (`mm`) used when `include_clearance_margin == false` so flush tangent
/// contact (e.g. a contour finishing exactly on top of a spoilboard or flush beside a vise jaw)
/// never triggers a false collision.
const PHYSICAL_CONTACT_EPS_MM: f64 = 5e-3;
/// Margin-mode threshold (`mm`) used for automatic safe-rapid routing.
const MARGIN_CONTACT_EPS_MM: f64 = 1e-4;
/// Extra vertical clearance (`mm`) added on top of `max_required_lift_z` for safe-rapid bridges.
const SAFE_RAPID_EXTRA_LIFT_MM: f64 = 2.0;

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
fn min_safe_z_over_fixtures_with_profile(
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
        let n_poly = clip_triangle_to_z_slab(v0, v1, v2, slab_lo, z_hi, &mut clipped);
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
            let dp = point_to_segment_dist_2d(DVec2::new(pt.x, pt.y), s0, s1);
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

/// Automatically re-routes colliding non-cutting transitions (`RapidMove`, `RotaryRapidMove`,
/// `MotionRole::LinkTransition`) cleanly up and over enabled 3D workholding fixtures while
/// **NEVER** altering cutting moves (`Cut`, `FinishCut`, `Plunge`, `Ramp`, `LeadIn`, `LeadOut`,
/// `TabRamp`).
///
/// Exact no-op when `!auto_avoid_fixtures_on_rapids` or `resolved_fixtures.is_empty()`.
pub fn route_toolpath_around_fixtures(
    toolpath: &mut GeneratedToolpath,
    tool: &Tool,
    setup: &ResolvedSetup,
    resolved_fixtures: &[ResolvedFixtureSolid],
    auto_avoid_fixtures_on_rapids: bool,
    machine_z_travel_mm: Option<f64>,
) -> FixtureRoutingSummary {
    if !auto_avoid_fixtures_on_rapids || resolved_fixtures.is_empty() {
        return FixtureRoutingSummary::default();
    }

    let assembly = ToolAssemblyProfile::from_tool(tool);
    let stock_top_z = setup.stock.aabb_wcs().max.z;
    let stock_min_z = setup.stock.aabb_wcs().min.z;

    let mut out: Vec<ClEvent> = Vec::with_capacity(toolpath.events.len() + 16);
    let mut cur_pos: Option<DVec3> = None;
    let mut cur_rotary_deg = 0.0_f64;
    let mut lifts_inserted = 0usize;
    let mut max_lifted_z = f64::NEG_INFINITY;

    for ev in &toolpath.events {
        match ev {
            ClEvent::RapidMove { end_wcs } => {
                if let Some(p0) = cur_pos
                    && (end_wcs.x - p0.x).hypot(end_wcs.y - p0.y) > 1e-5
                    && check_chord_fixture_collision(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                        true,
                        true,
                    )
                    .is_some()
                {
                    let req_z = min_safe_z_over_fixtures_with_profile(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                    )
                    .unwrap_or(setup.clearance_z_wcs);
                    let z_safe = setup
                        .clearance_z_wcs
                        .max(req_z + SAFE_RAPID_EXTRA_LIFT_MM)
                        .max(p0.z)
                        .max(end_wcs.z);

                    push_safe_rapid_bridge_3axis(
                        &mut out,
                        p0,
                        *end_wcs,
                        z_safe,
                        None,
                        stock_top_z,
                        setup.retract_z_wcs,
                    );
                    lifts_inserted += 1;
                    max_lifted_z = max_lifted_z.max(z_safe);
                    cur_pos = Some(*end_wcs);
                    continue;
                }
                cur_pos = Some(*end_wcs);
                out.push(ev.clone());
            }
            ClEvent::LinearMove {
                end_wcs,
                feed_mm_min,
                role,
            } if *role == MotionRole::LinkTransition => {
                if let Some(p0) = cur_pos
                    && (end_wcs.x - p0.x).hypot(end_wcs.y - p0.y) > 1e-5
                    && check_chord_fixture_collision(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                        true,
                        true,
                    )
                    .is_some()
                {
                    let req_z = min_safe_z_over_fixtures_with_profile(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                    )
                    .unwrap_or(setup.clearance_z_wcs);
                    let z_safe = setup
                        .clearance_z_wcs
                        .max(req_z + SAFE_RAPID_EXTRA_LIFT_MM)
                        .max(p0.z)
                        .max(end_wcs.z);

                    push_safe_rapid_bridge_3axis(
                        &mut out,
                        p0,
                        *end_wcs,
                        z_safe,
                        Some(*feed_mm_min),
                        stock_top_z,
                        setup.retract_z_wcs,
                    );
                    lifts_inserted += 1;
                    max_lifted_z = max_lifted_z.max(z_safe);
                    cur_pos = Some(*end_wcs);
                    continue;
                }
                cur_pos = Some(*end_wcs);
                out.push(ev.clone());
            }
            ClEvent::RotaryRapidMove {
                end_wcs,
                rotary_deg,
            } => {
                if let Some(p0) = cur_pos
                    && (end_wcs.x - p0.x).hypot(end_wcs.y - p0.y) > 1e-5
                    && check_chord_fixture_collision(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                        true,
                        true,
                    )
                    .is_some()
                {
                    let req_z = min_safe_z_over_fixtures_with_profile(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                    )
                    .unwrap_or(setup.clearance_z_wcs);
                    let rot_clear_z = setup.rotary.map_or(f64::NEG_INFINITY, |r| {
                        r.center_wcs.z + r.clearance_radius_mm
                    });
                    let z_safe = setup
                        .clearance_z_wcs
                        .max(rot_clear_z)
                        .max(req_z + SAFE_RAPID_EXTRA_LIFT_MM)
                        .max(p0.z)
                        .max(end_wcs.z);

                    if p0.z < z_safe - 1e-4 {
                        out.push(ClEvent::RotaryRapidMove {
                            end_wcs: DVec3::new(p0.x, p0.y, z_safe),
                            rotary_deg: cur_rotary_deg,
                        });
                    }
                    out.push(ClEvent::RotaryRapidMove {
                        end_wcs: DVec3::new(end_wcs.x, end_wcs.y, z_safe),
                        rotary_deg: *rotary_deg,
                    });
                    if end_wcs.z < z_safe - 1e-4 {
                        out.push(ClEvent::RotaryRapidMove {
                            end_wcs: *end_wcs,
                            rotary_deg: *rotary_deg,
                        });
                    }
                    lifts_inserted += 1;
                    max_lifted_z = max_lifted_z.max(z_safe);
                    cur_pos = Some(*end_wcs);
                    cur_rotary_deg = *rotary_deg;
                    continue;
                }
                cur_pos = Some(*end_wcs);
                cur_rotary_deg = *rotary_deg;
                out.push(ev.clone());
            }
            ClEvent::RotaryLinearMove {
                end_wcs,
                rotary_deg,
                feed_mm_min,
                effective_radius_mm,
                role,
            } if *role == MotionRole::LinkTransition => {
                if let Some(p0) = cur_pos
                    && (end_wcs.x - p0.x).hypot(end_wcs.y - p0.y) > 1e-5
                    && check_chord_fixture_collision(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                        true,
                        true,
                    )
                    .is_some()
                {
                    let req_z = min_safe_z_over_fixtures_with_profile(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                    )
                    .unwrap_or(setup.clearance_z_wcs);
                    let rot_clear_z = setup.rotary.map_or(f64::NEG_INFINITY, |r| {
                        r.center_wcs.z + r.clearance_radius_mm
                    });
                    let z_safe = setup
                        .clearance_z_wcs
                        .max(rot_clear_z)
                        .max(req_z + SAFE_RAPID_EXTRA_LIFT_MM)
                        .max(p0.z)
                        .max(end_wcs.z);

                    if p0.z < z_safe - 1e-4 {
                        out.push(ClEvent::RotaryRapidMove {
                            end_wcs: DVec3::new(p0.x, p0.y, z_safe),
                            rotary_deg: cur_rotary_deg,
                        });
                    }
                    out.push(ClEvent::RotaryRapidMove {
                        end_wcs: DVec3::new(end_wcs.x, end_wcs.y, z_safe),
                        rotary_deg: *rotary_deg,
                    });
                    if end_wcs.z < z_safe - 1e-4 {
                        out.push(ClEvent::RotaryLinearMove {
                            end_wcs: *end_wcs,
                            rotary_deg: *rotary_deg,
                            feed_mm_min: *feed_mm_min,
                            effective_radius_mm: *effective_radius_mm,
                            role: MotionRole::LinkTransition,
                        });
                    }
                    lifts_inserted += 1;
                    max_lifted_z = max_lifted_z.max(z_safe);
                    cur_pos = Some(*end_wcs);
                    cur_rotary_deg = *rotary_deg;
                    continue;
                }
                cur_pos = Some(*end_wcs);
                cur_rotary_deg = *rotary_deg;
                out.push(ev.clone());
            }
            ClEvent::RotaryIndex { angle_deg, .. } => {
                if let Some(p0) = cur_pos
                    && let Some(rot) = setup.rotary
                {
                    let rot_clear_z = rot.center_wcs.z + rot.clearance_radius_mm;
                    let needs_rot_lift = p0.z < rot_clear_z - 1e-4
                        && check_chord_fixture_collision(
                            p0,
                            p0,
                            &assembly,
                            resolved_fixtures,
                            true,
                            true,
                        )
                        .is_some();
                    if needs_rot_lift {
                        let req_z = min_safe_z_over_fixtures_with_profile(
                            p0,
                            p0,
                            &assembly,
                            resolved_fixtures,
                        )
                        .unwrap_or(rot_clear_z);
                        let z_safe = setup
                            .clearance_z_wcs
                            .max(rot_clear_z)
                            .max(req_z + SAFE_RAPID_EXTRA_LIFT_MM);
                        out.push(ClEvent::RapidMove {
                            end_wcs: DVec3::new(p0.x, p0.y, z_safe),
                        });
                        cur_pos = Some(DVec3::new(p0.x, p0.y, z_safe));
                        lifts_inserted += 1;
                        max_lifted_z = max_lifted_z.max(z_safe);
                    }
                }
                cur_rotary_deg = *angle_deg;
                out.push(ev.clone());
            }
            // Cutting moves (`Cut`, `FinishCut`, `Plunge`, `Ramp`, `LeadIn`, `LeadOut`, `TabRamp`)
            // are NEVER altered or skipped — only track current tool tip position!
            ClEvent::LinearMove { end_wcs, .. } | ClEvent::ArcMove { end_wcs, .. } => {
                cur_pos = Some(*end_wcs);
                out.push(ev.clone());
            }
            ClEvent::RotaryLinearMove {
                end_wcs,
                rotary_deg,
                ..
            } => {
                cur_pos = Some(*end_wcs);
                cur_rotary_deg = *rotary_deg;
                out.push(ev.clone());
            }
            ClEvent::DrillCycle {
                bottom_wcs,
                clear_z_wcs,
                r_plane_z_wcs,
                ..
            } => {
                cur_pos = Some(DVec3::new(
                    bottom_wcs.x,
                    bottom_wcs.y,
                    clear_z_wcs.max(*r_plane_z_wcs),
                ));
                out.push(ev.clone());
            }
            _ => {
                out.push(ev.clone());
            }
        }
    }

    if lifts_inserted == 0 {
        return FixtureRoutingSummary::default();
    }

    toolpath.events = out;
    toolpath.segments_wcs = rebuild_segments_from_events(&toolpath.events);
    toolpath.recompute_stats();
    toolpath.rapid_lifts_inserted = lifts_inserted;
    toolpath.max_lifted_z_wcs = if max_lifted_z.is_finite() {
        max_lifted_z
    } else {
        0.0
    };

    if let Some(z_travel) = machine_z_travel_mm
        && z_travel.is_finite()
        && z_travel > 0.0
        && max_lifted_z.is_finite()
    {
        let min_z = toolpath.stats.bounds_wcs.min.z.min(stock_min_z);
        let z_span = max_lifted_z - min_z;
        if z_span > z_travel + 0.01 {
            toolpath.warnings.push(format!(
                "Safe-rapid lift over fixtures (max Z = {max_lifted_z:+.1} mm, Z span {z_span:.1} mm) exceeds machine Z travel limit ({z_travel:.1} mm); lower fixture height or adjust Setup Z clearance."
            ));
        }
    }

    FixtureRoutingSummary {
        rapid_lifts_inserted: lifts_inserted,
        max_lifted_z_wcs: toolpath.max_lifted_z_wcs,
    }
}

/// Emits a clean 3-move vertical lift -> horizontal bridge -> vertical descent (with safe
/// feed-plane guard when a `LinkTransition` descends below stock top).
fn push_safe_rapid_bridge_3axis(
    out: &mut Vec<ClEvent>,
    p0: DVec3,
    p1: DVec3,
    z_safe: f64,
    link_feed_mm_min: Option<f64>,
    stock_top_z: f64,
    retract_z_wcs: f64,
) {
    // 1. Vertical lift to `(p0.x, p0.y, z_safe)` (coalescing if the immediately preceding move
    //    was already a vertical RapidMove at `(p0.x, p0.y)`!)
    if p0.z < z_safe - 1e-4 {
        let lift_pt = DVec3::new(p0.x, p0.y, z_safe);
        let merged_prev = if let Some(ClEvent::RapidMove { end_wcs }) = out.last_mut()
            && (end_wcs.x - p0.x).abs() <= 1e-5
            && (end_wcs.y - p0.y).abs() <= 1e-5
            && end_wcs.z <= z_safe + 1e-4
        {
            end_wcs.z = z_safe;
            true
        } else {
            false
        };
        if !merged_prev {
            out.push(ClEvent::RapidMove { end_wcs: lift_pt });
        }
    }

    // 2. Horizontal safe clearance bridge to `(p1.x, p1.y, z_safe)`
    out.push(ClEvent::RapidMove {
        end_wcs: DVec3::new(p1.x, p1.y, z_safe),
    });

    // 3. Vertical descent to `p1` (with safe feed-plane guard for sub-stock `LinkTransition`s!)
    if p1.z < z_safe - 1e-4 {
        if let Some(feed) = link_feed_mm_min
            && p1.z < stock_top_z - 1e-4
        {
            let feed_guard_z = retract_z_wcs.max(stock_top_z + 1.0).min(z_safe);
            if feed_guard_z < z_safe - 1e-4 && feed_guard_z > p1.z + 1e-4 {
                out.push(ClEvent::RapidMove {
                    end_wcs: DVec3::new(p1.x, p1.y, feed_guard_z),
                });
            }
            out.push(ClEvent::LinearMove {
                end_wcs: p1,
                feed_mm_min: feed.max(1.0),
                role: MotionRole::LinkTransition,
            });
        } else {
            out.push(ClEvent::RapidMove { end_wcs: p1 });
        }
    }
}

// ---------------------------------------------------------------------------
// Exact 2D / 3D Computational Geometry Primitives
// ---------------------------------------------------------------------------

/// Clips 3D triangle `(v0, v1, v2)` to the horizontal Z-slab `[z_lo, z_hi]` via exact
/// Sutherland-Hodgman plane clipping. Returns the number of output vertices (`0` or `3..=5`).
fn clip_triangle_to_z_slab(
    v0: DVec3,
    v1: DVec3,
    v2: DVec3,
    z_lo: f64,
    z_hi: f64,
    out: &mut [DVec3; 7],
) -> usize {
    let mut buf_a = [
        v0,
        v1,
        v2,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
    ];
    let mut n = 3usize;

    // Clip against `z >= z_lo`
    if z_lo.is_finite() {
        let mut next_n = 0usize;
        let mut prev = buf_a[n - 1];
        let mut prev_in = prev.z >= z_lo;
        for &cur in &buf_a[..n] {
            let cur_in = cur.z >= z_lo;
            if cur_in != prev_in {
                let denom = cur.z - prev.z;
                if denom.abs() > 1e-15 && next_n < out.len() {
                    let t = ((z_lo - prev.z) / denom).clamp(0.0, 1.0);
                    out[next_n] = prev.lerp(cur, t);
                    out[next_n].z = z_lo;
                    next_n += 1;
                }
            }
            if cur_in && next_n < out.len() {
                out[next_n] = cur;
                next_n += 1;
            }
            prev = cur;
            prev_in = cur_in;
        }
        if next_n < 3 {
            return 0;
        }
        buf_a[..next_n].copy_from_slice(&out[..next_n]);
        n = next_n;
    }

    // Clip against `z <= z_hi`
    if z_hi.is_finite() {
        let mut next_n = 0usize;
        let mut prev = buf_a[n - 1];
        let mut prev_in = prev.z <= z_hi;
        for &cur in &buf_a[..n] {
            let cur_in = cur.z <= z_hi;
            if cur_in != prev_in {
                let denom = cur.z - prev.z;
                if denom.abs() > 1e-15 && next_n < out.len() {
                    let t = ((z_hi - prev.z) / denom).clamp(0.0, 1.0);
                    out[next_n] = prev.lerp(cur, t);
                    out[next_n].z = z_hi;
                    next_n += 1;
                }
            }
            if cur_in && next_n < out.len() {
                out[next_n] = cur;
                next_n += 1;
            }
            prev = cur;
            prev_in = cur_in;
        }
        if next_n < 3 {
            return 0;
        }
        next_n
    } else {
        out[..n].copy_from_slice(&buf_a[..n]);
        n
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
        let (d, u_edge) = segment_to_segment_dist_2d(s0, s1, a2, b2);
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

/// Exact distance from 2D point `p` to 2D segment `(a, b)`.
#[inline]
fn point_to_segment_dist_2d(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let ab = b - a;
    let len2 = ab.length_squared();
    if len2 <= 1e-18 {
        return p.distance(a);
    }
    let t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

/// Exact minimum 2D distance between segments `(p0, p1)` and `(q0, q1)`, returning
/// `(distance, u_on_q)` where `q0.lerp(q1, u_on_q)` achieves the minimum distance.
fn segment_to_segment_dist_2d(p0: DVec2, p1: DVec2, q0: DVec2, q1: DVec2) -> (f64, f64) {
    let dp = p1 - p0;
    let dq = q1 - q0;
    let denom = dp.x * dq.y - dp.y * dq.x;
    if denom.abs() > 1e-12 {
        let r = q0 - p0;
        let t = (r.x * dq.y - r.y * dq.x) / denom;
        let u = (r.x * dp.y - r.y * dp.x) / denom;
        if (-1e-9..=1.0 + 1e-9).contains(&t) && (-1e-9..=1.0 + 1e-9).contains(&u) {
            return (0.0, u.clamp(0.0, 1.0));
        }
    }

    let mut best_d = f64::INFINITY;
    let mut best_u = 0.0;

    // 1. Endpoints of `p` projected onto `q`
    let q_len2 = dq.length_squared();
    for &ep in &[p0, p1] {
        let u = if q_len2 <= 1e-18 {
            0.0
        } else {
            ((ep - q0).dot(dq) / q_len2).clamp(0.0, 1.0)
        };
        let d = ep.distance(q0 + dq * u);
        if d < best_d {
            best_d = d;
            best_u = u;
        }
    }

    // 2. Endpoints of `q` projected onto `p`
    let p_len2 = dp.length_squared();
    for &(eq, u_fixed) in &[(q0, 0.0), (q1, 1.0)] {
        let t = if p_len2 <= 1e-18 {
            0.0
        } else {
            ((eq - p0).dot(dp) / p_len2).clamp(0.0, 1.0)
        };
        let d = eq.distance(p0 + dp * t);
        if d < best_d {
            best_d = d;
            best_u = u_fixed;
        }
    }

    (best_d, best_u)
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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::CadDocument;
    use chip_math::{DisplayUnit, ParamLength, approx_eq};
    use chip_tools::{CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId};

    use super::*;
    use crate::{
        BoxAnchor27, FixtureBody, FixtureKind, Setup, StockConfig, ToolpathBuilder, WcsOriginMode,
        WorkOffsetSlot, resolve_setup_fixtures_parametric,
    };

    fn test_endmill_6mm() -> Tool {
        Tool {
            id: ToolId("t1_6mm".to_string()),
            tool_number: 1,
            length_offset_number: 1,
            diameter_offset_number: 1,
            name: "6mm Flat Endmill".to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::Flood,
            stickout_length_mm: 24.0,
            geometry: CutterGeometry::FlatEndmill {
                diameter_mm: 6.0,
                flute_length_mm: 18.0,
                overall_length_mm: 55.0,
                shank_diameter_mm: 6.0,
                flutes: 3,
            },
            presets: Vec::new(),
        }
    }

    fn make_box_setup() -> (Setup, ResolvedSetup) {
        let mut setup = Setup::new("s1", "Setup 1", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(100.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: crate::FixedBoxXyAlign::CenterXy,
            z_align: crate::FixedBoxZAlign::CenterZ,
        };
        let doc = CadDocument::empty();
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        (setup, resolved)
    }

    #[test]
    fn rapid_routing_lifts_cleanly_over_toe_clamp_and_preserves_cutting_moves() {
        let (mut setup, resolved) = make_box_setup();
        // Place a ToeClamp across X = 0 (pointing +Y, anchor at Y = -16 so the tall stud tower sits directly on Y = 0!)
        let clamp =
            FixtureBody::default_toe_clamp("c1", "Center Clamp", DVec2::new(0.0, -16.0), 90.0);
        setup.fixtures.push(clamp);
        let fixtures =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();
        let clamp_top_z = fixtures[0].aabb_wcs.max.z;
        assert!(
            clamp_top_z > 20.0,
            "expected stud top > 20 mm, got {clamp_top_z}"
        );

        let tool = test_endmill_6mm();
        let mut tb = ToolpathBuilder::new("op_1");
        tb.rapid_to(DVec3::new(-30.0, 0.0, 10.0));
        tb.linear_to(DVec3::new(-30.0, 0.0, -5.0), 300.0, MotionRole::Plunge);
        tb.linear_to(DVec3::new(-20.0, 0.0, -5.0), 800.0, MotionRole::Cut);
        tb.rapid_to(DVec3::new(-20.0, 0.0, 10.0));
        // Colliding horizontal rapid across the toe clamp at Z = 10 mm
        tb.rapid_to(DVec3::new(30.0, 0.0, 10.0));
        tb.linear_to(DVec3::new(30.0, 0.0, -5.0), 300.0, MotionRole::Plunge);
        tb.linear_to(DVec3::new(40.0, 0.0, -5.0), 800.0, MotionRole::Cut);
        let mut tp = tb.finish();

        // 1. No-op when `auto_avoid = false`
        let orig_events = tp.events.clone();
        let sum_off =
            route_toolpath_around_fixtures(&mut tp, &tool, &resolved, &fixtures, false, None);
        assert_eq!(sum_off.rapid_lifts_inserted, 0);
        assert_eq!(tp.events, orig_events);

        // 2. Active routing lifts the colliding rapid cleanly above clamp_top + clearance + 2.0
        let sum_on =
            route_toolpath_around_fixtures(&mut tp, &tool, &resolved, &fixtures, true, Some(30.0));
        assert_eq!(sum_on.rapid_lifts_inserted, 1);
        assert!(
            sum_on.max_lifted_z_wcs >= clamp_top_z + fixtures[0].clearance_margin_mm + 2.0 - 1e-4
        );
        assert_eq!(tp.rapid_lifts_inserted, 1);
        assert!(approx_eq(
            tp.max_lifted_z_wcs,
            sum_on.max_lifted_z_wcs,
            1e-9
        ));
        // Machine Z travel (30 mm) was exceeded by `max_lifted_z - stock_min_z` (~45 mm) -> warning!
        assert!(
            tp.warnings
                .iter()
                .any(|w| w.contains("exceeds machine Z travel limit")),
            "expected Z travel warning, got {:?}",
            tp.warnings
        );

        // 3. Every rapid/linear segment now passes full-assembly collision check cleanly!
        for seg in &tp.segments_wcs {
            let hit = check_segment_fixture_collision(seg, &tool, &fixtures, false);
            assert!(
                hit.is_none(),
                "expected zero fixture hits after routing, got {hit:?}"
            );
        }
    }

    #[test]
    fn holder_collision_caught_when_flute_clears_tall_jaw() {
        let (mut setup, resolved) = make_box_setup();
        // Tall box keep-out standing 8 mm away from Y = 0 (Y = 8..28, Z = -20..+35)
        // Tool Ø6 mm (flute_r = 3, shank_r = 3, stickout = 24, holder_nut_r = 12) moving along Y = 0 at Z = 0:
        // - Flute & shank are at distance 8 mm > 3 mm -> clear!
        // - Holder nut starts at Z = 0 + 24 = +24 mm with radius 12 mm > 8 mm -> penetrates 4.0 mm!
        let mut jaw = FixtureBody::default_box_keepout(
            "tall_jaw",
            "Tall Vise Jaw",
            DVec3::new(0.0, 18.0, 7.5),
        );
        jaw.kind = FixtureKind::BoxKeepOut {
            center_wcs: [
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(18.0),
                ParamLength::from_mm(7.5),
            ],
            size_wcs: [
                ParamLength::from_mm(60.0),
                ParamLength::from_mm(20.0),
                ParamLength::from_mm(55.0), // Z = -20 .. +35
            ],
            yaw_deg: 0.0,
        };
        setup.fixtures.push(jaw);
        let fixtures =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();

        let tool = test_endmill_6mm();
        let seg = ToolpathSegment3D {
            start_wcs: DVec3::new(-20.0, 0.0, 0.0),
            end_wcs: DVec3::new(20.0, 0.0, 0.0),
            motion: SegmentMotion3D::Linear {
                feed_mm_min: 800.0,
                role: MotionRole::Cut,
            },
            start_rotary_deg: 0.0,
            end_rotary_deg: 0.0,
        };
        let hit = check_segment_fixture_collision(&seg, &tool, &fixtures, false)
            .expect("holder must collide with tall jaw");
        assert_eq!(hit.fixture_id, "tall_jaw");
        assert_eq!(hit.contact_part, ToolAssemblyPart::Holder);
        assert!(
            approx_eq(hit.penetration_mm, 4.0, 0.05),
            "expected ~4.0 mm holder radial penetration, got {}",
            hit.penetration_mm
        );
    }

    #[test]
    fn spoilboard_allowed_overcut_permits_skim_and_flags_deep_plunge() {
        let (mut setup, resolved) = make_box_setup();
        let mut sb = FixtureBody::default_spoilboard("sb1", "MDF Spoilboard");
        sb.allowed_overcut = ParamLength::from_mm(0.3);
        setup.fixtures.push(sb);
        let fixtures =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();
        let sb_top = fixtures[0].aabb_wcs.max.z; // stock min Z = -20.0

        let tool = test_endmill_6mm();
        // 0.2 mm skim cut -> allowed!
        let skim = ToolpathSegment3D {
            start_wcs: DVec3::new(-10.0, 0.0, sb_top - 0.2),
            end_wcs: DVec3::new(10.0, 0.0, sb_top - 0.2),
            motion: SegmentMotion3D::Linear {
                feed_mm_min: 600.0,
                role: MotionRole::Cut,
            },
            start_rotary_deg: 0.0,
            end_rotary_deg: 0.0,
        };
        assert!(
            check_segment_fixture_collision(&skim, &tool, &fixtures, false).is_none(),
            "0.2 mm skim into spoilboard with 0.3 mm allowance must pass cleanly"
        );

        // 1.0 mm deep cut -> triggers FixtureCollision!
        let deep = ToolpathSegment3D {
            start_wcs: DVec3::new(0.0, 0.0, sb_top + 1.0),
            end_wcs: DVec3::new(0.0, 0.0, sb_top - 1.0),
            motion: SegmentMotion3D::Linear {
                feed_mm_min: 300.0,
                role: MotionRole::Plunge,
            },
            start_rotary_deg: 0.0,
            end_rotary_deg: 0.0,
        };
        let hit = check_segment_fixture_collision(&deep, &tool, &fixtures, false)
            .expect("1.0 mm plunge into spoilboard must collide");
        assert_eq!(hit.fixture_id, "sb1");
        assert_eq!(hit.contact_part, ToolAssemblyPart::Flute);
        assert!(
            approx_eq(hit.penetration_mm, 0.7, 0.05),
            "expected ~0.7 mm excess over 0.3 mm allowance, got {}",
            hit.penetration_mm
        );
    }

    #[test]
    fn clamp_in_cut_path_is_never_mutated_by_rapid_router() {
        let (mut setup, resolved) = make_box_setup();
        setup.fixtures.push(FixtureBody::default_toe_clamp(
            "c_cut",
            "Blocking Clamp",
            DVec2::new(0.0, 0.0),
            0.0,
        ));
        let fixtures =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();
        let tool = test_endmill_6mm();

        let mut tb = ToolpathBuilder::new("op_cut");
        tb.rapid_to(DVec3::new(-25.0, 0.0, 35.0));
        tb.linear_to(DVec3::new(-25.0, 0.0, -2.0), 300.0, MotionRole::Plunge);
        // Cutting move straight through the clamp nose at Z = -2 mm!
        tb.linear_to(DVec3::new(25.0, 0.0, -2.0), 800.0, MotionRole::Cut);
        tb.rapid_to(DVec3::new(25.0, 0.0, 35.0));
        let mut tp = tb.finish();
        let orig_events = tp.events.clone();

        let summary =
            route_toolpath_around_fixtures(&mut tp, &tool, &resolved, &fixtures, true, None);
        assert_eq!(summary.rapid_lifts_inserted, 0);
        assert_eq!(
            tp.events, orig_events,
            "cutting moves must NEVER be mutated"
        );
    }
}
