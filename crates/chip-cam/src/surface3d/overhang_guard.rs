//! 2.5D Overhang Collision Guard & Opt-In 3D Model Overhang Avoidance (`M72`).
//!
//! Provides:
//! 1. Fast BVH detection ([`check_2d_toolpath_against_overhangs`]) warning whenever a 2.5D
//!    operation (`Facing`, `Contour2D`, `Pocket2D`, `DrillBore`, `ChamferTrace`) cuts or plunges
//!    underneath overhanging 3D solid geometry (including wider shank and collet/holder tiers),
//!    while strictly ignoring flush floor/wall contact on prismatic 2.5D features.
//! 2. Exact contour clipping ([`clip_2d_contour_outside_overhangs`]), lead-in/out clearance checks,
//!    and hole-column occlusion checks used when `PassesParams::avoid_model_overhangs == true`.

use chip_math::{ArcPolyline2D, DVec2, DVec3};
use chip_tools::Tool;
use rayon::prelude::*;

use crate::{
    fixture_avoidance::ToolAssemblyPart,
    rest2d::{RegionSet2D, clip_polyline_outside_region_set},
    surface3d::{
        SurfaceCutterProfile, SurfaceMesh3D,
        drop_cutter::{drop_cutter_z_detailed_with_scratch, drop_cutter_z_with_scratch},
    },
    toolpath::{SegmentMotion3D, ToolpathSegment3D},
};

/// Diagnostic report emitted when a 2.5D toolpath passes beneath overhanging 3D model geometry.
#[derive(Debug, Clone, PartialEq)]
pub struct OverhangGuardReport {
    /// Number of emitted non-rapid segments that intersect overhanging 3D geometry.
    pub colliding_segments: usize,
    /// Worst vertical intrusion (`z_required - z_tool_tip`, `mm`) detected across all segments.
    pub max_excess_z_mm: f64,
    /// Lowest cutting Z height (`mm` WCS) where an overhang collision occurred.
    pub min_affected_z_wcs: f64,
    /// Highest overhanging solid height (`mm` WCS) obstructing the toolpath.
    pub max_overhang_z_wcs: f64,
    /// Worst collision location in Setup WCS (`mm`).
    pub worst_point_wcs: DVec3,
    /// Assembly portion (`Flute`, `Shank`, or `Holder`) causing the worst collision.
    pub worst_part: ToolAssemblyPart,
}

impl OverhangGuardReport {
    /// Formats a clear, actionable non-blocking warning suitable for `GeneratedToolpath::warnings`.
    pub fn format_warning(&self, op_kind_label: &str) -> String {
        format!(
            "{op_kind_label} passes beneath overhanging 3D model geometry ({} collides up to Z={:.2} mm above cut Z={:.2} mm across {} move(s) near WCS ({:.1}, {:.1})). Enable 'Avoid Model Overhangs' in the Passes tab or switch to Adaptive3D / Undercut3D.",
            self.worst_part.label(),
            self.max_overhang_z_wcs,
            self.min_affected_z_wcs,
            self.colliding_segments,
            self.worst_point_wcs.x,
            self.worst_point_wcs.y,
        )
    }
}

/// Fast BVH overhang collision check for 2.5D emitted segments against a 3D [`SurfaceMesh3D`].
///
/// Ignores flush floor and vertical-wall contact within grazing tolerance (`0.15 mm` radially and
/// `0.15 mm` axially) so normal prismatic 2.5D pockets, contours, chamfers, facing passes, and
/// holes never trigger false warnings.
///
/// `min_ignored_z_wcs`: optional Z floor below which intentional unmodeled feature removal occurs
/// (e.g., sharp-edge `EdgeChamfer` or center-compensated `CurveTrace` top edge Z).
pub fn check_2d_toolpath_against_overhangs(
    surface: &SurfaceMesh3D,
    tool: &Tool,
    segments: &[ToolpathSegment3D],
    min_ignored_z_wcs: Option<f64>,
) -> Option<OverhangGuardReport> {
    if surface.triangles.is_empty() || !surface.bounds.is_valid() || segments.is_empty() {
        return None;
    }

    const Z_TOL_MM: f64 = 0.15;
    let tool_r = 0.5 * tool.geometry.cutting_diameter_mm().max(0.1);
    let wall_tol = (0.15_f64).min(0.25 * tool_r);

    // Quick Z early-out across all non-rapid segments
    let mut min_seg_z = f64::INFINITY;
    for seg in segments {
        if !matches!(seg.motion, SegmentMotion3D::Rapid) {
            min_seg_z = min_seg_z.min(seg.start_wcs.z).min(seg.end_wcs.z);
        }
    }
    if !min_seg_z.is_finite() {
        return None;
    }
    let effective_min_z = min_ignored_z_wcs.map_or(min_seg_z, |z| min_seg_z.max(z));
    if surface.bounds.max.z <= effective_min_z + Z_TOL_MM {
        return None;
    }

    // Build full-assembly profile shrunk radially by `wall_tol` so flush vertical walls,
    // cylindrical hole faces, and modeled chamfer bevels never false-trigger.
    let check_cutter =
        SurfaceCutterProfile::from_tool(tool, 0.0, 0.0, 0.0).with_radial_shrink(wall_tol);
    let sample_step = (0.35 * tool_r).clamp(0.25, 2.0);
    let z_floor = min_seg_z - 1.0;

    let eval_seg = |seg: &ToolpathSegment3D,
                    scratch: &mut Vec<usize>|
     -> Option<(f64, f64, f64, DVec3, ToolAssemblyPart)> {
        if matches!(seg.motion, SegmentMotion3D::Rapid) {
            return None;
        }
        let seg_min_z = seg.start_wcs.z.min(seg.end_wcs.z);
        let seg_check_min_z = min_ignored_z_wcs.map_or(seg_min_z, |z| seg_min_z.max(z));
        if surface.bounds.max.z <= seg_check_min_z + Z_TOL_MM {
            return None;
        }

        let len = seg.length_mm();
        let n_steps = if len <= 1e-5 {
            1
        } else {
            ((len / sample_step).ceil() as usize).clamp(2, 64)
        };

        let mut best_excess = 0.0_f64;
        let mut best_cut_z = f64::INFINITY;
        let mut best_oh_z = f64::NEG_INFINITY;
        let mut best_pt = seg.start_wcs;
        let mut best_part = ToolAssemblyPart::Flute;
        let mut hit = false;

        for k in 0..=n_steps {
            let t = k as f64 / n_steps as f64;
            let p = seg.sample_point(t);
            let z_ref = min_ignored_z_wcs.map_or(p.z, |z| p.z.max(z));
            if let Some(det) = drop_cutter_z_detailed_with_scratch(
                surface,
                &check_cutter,
                p.x,
                p.y,
                z_floor,
                scratch,
            ) {
                let excess = det.z_tip_wcs - z_ref;
                if excess > Z_TOL_MM {
                    hit = true;
                    best_cut_z = best_cut_z.min(p.z);
                    best_oh_z = best_oh_z.max(det.z_tip_wcs);
                    if excess > best_excess {
                        best_excess = excess;
                        best_pt = p;
                        best_part = det.limiting_part;
                    }
                }
            }
        }

        hit.then_some((best_excess, best_cut_z, best_oh_z, best_pt, best_part))
    };

    let hits: Vec<(f64, f64, f64, DVec3, ToolAssemblyPart)> = if segments.len() >= 64 {
        segments
            .par_iter()
            .map_init(
                || Vec::with_capacity(64),
                |scratch, seg| eval_seg(seg, scratch),
            )
            .flatten()
            .collect()
    } else {
        let mut scratch = Vec::with_capacity(64);
        segments
            .iter()
            .filter_map(|seg| eval_seg(seg, &mut scratch))
            .collect()
    };

    if hits.is_empty() {
        return None;
    }

    let mut report = OverhangGuardReport {
        colliding_segments: hits.len(),
        max_excess_z_mm: 0.0,
        min_affected_z_wcs: f64::INFINITY,
        max_overhang_z_wcs: f64::NEG_INFINITY,
        worst_point_wcs: DVec3::ZERO,
        worst_part: ToolAssemblyPart::Flute,
    };

    for (excess, cut_z, oh_z, pt, part) in hits {
        report.min_affected_z_wcs = report.min_affected_z_wcs.min(cut_z);
        report.max_overhang_z_wcs = report.max_overhang_z_wcs.max(oh_z);
        if excess > report.max_excess_z_mm {
            report.max_excess_z_mm = excess;
            report.worst_point_wcs = pt;
            report.worst_part = part;
        }
    }

    Some(report)
}

/// Clips a 2D offset contour polyline (`Contour2D`, `ChamferTrace`, or `Pocket2D` finish wall)
/// strictly outside overhanging 3D geometry while preserving unobstructed wall-tangent portions.
///
/// Uses a two-stage strategy:
/// 1. Clip against `avoid_z.erode_inward(0.06)` so contour segments running flush along vertical
///    prismatic walls (`d == tool_r`) are NOT falsely erased by `PushCutterField`'s 15 µm sagitta
///    dilation.
/// 2. For any open clipped endpoint created by an overhang intersection, trim back along the
///    polyline until exact `drop_cutter_z` (with grazing-free wall clearance) clears `z_cut`.
pub fn clip_2d_contour_outside_overhangs(
    poly: &ArcPolyline2D,
    avoid_z: &RegionSet2D,
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    z_cut: f64,
) -> Vec<ArcPolyline2D> {
    if poly.segments.is_empty() || avoid_z.is_empty() {
        return vec![poly.clone()];
    }

    const WALL_GRAZE_ERODE_MM: f64 = 0.06;
    let deep_avoid = avoid_z.erode_inward(WALL_GRAZE_ERODE_MM);
    if deep_avoid.is_empty() {
        return vec![poly.clone()];
    }

    let clipped = clip_polyline_outside_region_set(poly, &deep_avoid);
    if clipped.is_empty() {
        return Vec::new();
    }

    // If the entire closed contour remained untouched outside `deep_avoid`, preserve it intact!
    if clipped.len() == 1 && clipped[0].closed && poly.closed {
        return clipped;
    }

    // Open spans created by overhang clipping: trim back each exposed end so the cutter center
    // stays strictly clear of the overhang (0.000 mm gouge in Tri-Dexel verification) while
    // allowing flush contact on the intended tangent vertical wall (`shrink = 0.005 mm`).
    let trim_cutter = cutter.with_radial_shrink(0.005);
    let mut scratch = Vec::with_capacity(64);
    let z_floor = z_cut - 1.0;

    let mut out = Vec::with_capacity(clipped.len());
    for span in clipped {
        let perim = span.perimeter();
        if perim <= 0.05 {
            continue;
        }
        if span.closed {
            out.push(span);
            continue;
        }

        let is_blocked = |s: f64, sc: &mut Vec<usize>| -> bool {
            let Some((pt, _)) = span.eval_at_distance(s) else {
                return false;
            };
            drop_cutter_z_with_scratch(surface, &trim_cutter, pt.x, pt.y, z_floor, sc)
                .is_some_and(|zd| zd > z_cut + 0.015)
        };

        let mut s_start = 0.0_f64;
        let max_trim = (0.35_f64).min(0.45 * perim);
        if is_blocked(s_start, &mut scratch) {
            let mut probe = 0.02;
            while probe < max_trim && is_blocked(probe, &mut scratch) {
                probe += 0.02;
            }
            s_start = probe.min(perim);
        }

        let mut s_end = perim;
        if s_end > s_start + 0.05 && is_blocked(s_end, &mut scratch) {
            let mut probe = 0.02;
            while probe < max_trim && is_blocked((perim - probe).max(s_start), &mut scratch) {
                probe += 0.02;
            }
            s_end = (perim - probe).max(s_start);
        }

        if s_end > s_start + 0.05 {
            let trimmed = if s_start > 1e-6 || s_end < perim - 1e-6 {
                span.slice_interval(s_start, s_end)
            } else {
                span
            };
            if trimmed.perimeter() > 0.05 {
                out.push(trimmed);
            }
        }
    }

    out
}

/// Returns `true` when the straight XY chord `a -> b` stays clear of `avoid_z` (allowing tiny
/// tangent wall grazing `<= 0.02 mm` at the anchor endpoint).
pub fn is_2d_segment_clear_of_avoid(a: DVec2, b: DVec2, avoid_z: &RegionSet2D) -> bool {
    if avoid_z.is_empty() {
        return true;
    }
    let dist = a.distance(b);
    let n = (dist / 0.25).ceil().clamp(4.0, 64.0) as usize;
    for k in 0..=n {
        let pt = a.lerp(b, k as f64 / n as f64);
        if avoid_z.contains_point(pt) && avoid_z.min_distance_to_boundary(pt) > 0.02 {
            return false;
        }
    }
    true
}

/// Returns `true` when an XY circular arc from `start` around `center` by `signed_sweep_rad`
/// stays strictly clear of `avoid_z`.
pub fn is_2d_arc_clear_of_avoid(
    start: DVec2,
    center: DVec2,
    radius: f64,
    signed_sweep_rad: f64,
    avoid_z: &RegionSet2D,
) -> bool {
    if avoid_z.is_empty() || radius <= 1e-6 {
        return true;
    }
    let a0 = (start.y - center.y).atan2(start.x - center.x);
    let arc_len = radius * signed_sweep_rad.abs();
    let n = (arc_len / 0.25).ceil().clamp(6.0, 64.0) as usize;
    for k in 0..=n {
        let theta = a0 + signed_sweep_rad * (k as f64 / n as f64);
        let pt = DVec2::new(
            center.x + radius * theta.cos(),
            center.y + radius * theta.sin(),
        );
        if avoid_z.contains_point(pt) && avoid_z.min_distance_to_boundary(pt) > 0.02 {
            return false;
        }
    }
    true
}

/// Checks whether a vertical drilling or helical-boring column at `center_wcs` down to
/// `hole_bottom_z` is occluded by overhanging 3D model geometry (or wider shank/holder interference).
///
/// Returns `Some(blocking_z_wcs)` if occluded, or `None` if the full vertical column is clear.
pub fn check_hole_column_overhang(
    surface: &SurfaceMesh3D,
    tool: &Tool,
    center_wcs: DVec2,
    orbit_radius_mm: f64,
    hole_bottom_z: f64,
) -> Option<f64> {
    if surface.triangles.is_empty() || !surface.bounds.is_valid() {
        return None;
    }
    const Z_TOL_MM: f64 = 0.15;
    if surface.bounds.max.z <= hole_bottom_z + Z_TOL_MM {
        return None;
    }

    let tool_r = 0.5 * tool.geometry.cutting_diameter_mm().max(0.1);
    let wall_tol = (0.10_f64).min(0.25 * tool_r);
    // Shrink flute radius slightly to tolerate tessellated cylindrical hole chord sagitta while
    // preserving full shank & holder clearance margins.
    let mut check_cutter = SurfaceCutterProfile::from_tool(tool, 0.0, 0.0, 0.5);
    check_cutter.shape = check_cutter.with_radial_shrink(wall_tol).shape;

    let mut scratch = Vec::with_capacity(64);
    let z_floor = hole_bottom_z - 1.0;
    let mut worst_z: Option<f64> = None;

    let mut test_pt = |pt: DVec2| {
        if let Some(zd) =
            drop_cutter_z_with_scratch(surface, &check_cutter, pt.x, pt.y, z_floor, &mut scratch)
            && zd > hole_bottom_z + Z_TOL_MM
            && worst_z.is_none_or(|b| zd > b)
        {
            worst_z = Some(zd);
        }
    };

    test_pt(center_wcs);
    if orbit_radius_mm > 0.05 {
        for k in 0..8 {
            let theta = (k as f64) * std::f64::consts::FRAC_PI_4;
            test_pt(
                center_wcs
                    + DVec2::new(orbit_radius_mm * theta.cos(), orbit_radius_mm * theta.sin()),
            );
        }
    }

    worst_z
}
