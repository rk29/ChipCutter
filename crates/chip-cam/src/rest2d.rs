//! Exact 2D Boolean Region Sets ([`RegionSet2D`]), 2D Swept Toolpath Coverage, and Gouge-Proof
//! **2.5D Rest Machining** (`05-operations.md` §Rest Machining).

use std::sync::Arc;

pub use chip_geom2d::region::{
    RegionSet2D, clip_polyline_by_region_set, clip_polyline_outside_region_set,
    contours_to_region_set, swept_segment_capsule_2d,
};
use chip_geom2d::{PocketRegion2D, offset_polyline};
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2};

use crate::{
    cl_data::ArcPlane,
    params::{RestMachiningConfig, RestMachiningMode},
    toolpath::{GeneratedToolpath, SegmentMotion3D},
};

/// Upstream operation toolpath + cutter envelope supplied to `RestMachiningMode::FromPrecedingOperations`.
#[derive(Debug, Clone)]
pub struct UpstreamSweptInput {
    pub op_id: String,
    pub tool_diameter_mm: f64,
    pub corner_radius_mm: f64,
    pub toolpath: Arc<GeneratedToolpath>,
    /// Normalized rotary index angle in `[0°, 360°)` for `OperationRotaryMode::Indexed3Plus1`
    /// operations, or `None` for standard 3-axis / non-indexed operations.
    pub indexed_angle_deg: Option<f64>,
    /// `true` for rotary-native simultaneous/turning operations (`Rotary2Linear`, `Rotary4AxisSim`)
    /// or `OperationRotaryMode::WrapCylinder` wrapped operations whose `segments_wcs` are non-planar
    /// and must never be projected into 2D planar `swept_toolpath_coverage_2d`.
    pub is_rotary_native: bool,
}

/// Normalizes an indexed rotary angle (`deg`) into canonical `[0.0, 360.0)`.
#[inline]
pub fn normalize_indexed_angle_deg(angle_deg: f64) -> f64 {
    if !angle_deg.is_finite() {
        return 0.0;
    }
    let m = angle_deg.rem_euclid(360.0);
    if m.abs() < 1e-9 || (360.0 - m).abs() < 1e-9 {
        0.0
    } else {
        m
    }
}

/// Returns `true` when two rotary index angles `a_deg` and `b_deg` coincide modulo `360°` within
/// `tol_deg`.
#[inline]
pub fn indexed_angles_match(a_deg: f64, b_deg: f64, tol_deg: f64) -> bool {
    if !a_deg.is_finite() || !b_deg.is_finite() || !tol_deg.is_finite() {
        return false;
    }
    let diff = (a_deg - b_deg).rem_euclid(360.0);
    diff.min(360.0 - diff) <= tol_deg.max(0.0)
}

/// Filters `inputs` to only upstream operations whose 2D planar XY coordinates live in the exact
/// same frame as `target_angle_deg`:
/// - Always excludes rotary-native / cylindrical-wrapped inputs (`is_rotary_native == true`).
/// - Matches `(Some(u_ang), Some(t_ang))` iff `indexed_angles_match(u_ang, t_ang, 1e-3)`.
/// - Matches `(None, None)` for standard 3-axis planar operations.
/// - Excludes cross-angle or mixed indexed/non-indexed pairs so 2D swept capsules from one rotary
///   index angle can never corrupt another angle's XY coordinate plane.
pub fn filter_upstream_for_indexed_angle(
    inputs: &[UpstreamSweptInput],
    target_angle_deg: Option<f64>,
) -> Vec<UpstreamSweptInput> {
    const ANGLE_TOL_DEG: f64 = 1e-3;
    let target_norm = target_angle_deg.map(normalize_indexed_angle_deg);
    inputs
        .iter()
        .filter(|inp| {
            if inp.is_rotary_native {
                return false;
            }
            match (inp.indexed_angle_deg, target_norm) {
                (Some(u_ang), Some(t_ang)) => indexed_angles_match(u_ang, t_ang, ANGLE_TOL_DEG),
                (None, None) => true,
                _ => false,
            }
        })
        .cloned()
        .collect()
}

fn effective_radius_at_height(
    tool_diameter_mm: f64,
    corner_radius_mm: f64,
    dz_above_tip: f64,
) -> f64 {
    let r = 0.5 * tool_diameter_mm;
    let rc = corner_radius_mm.clamp(0.0, r);
    if rc <= 1e-5 || dz_above_tip >= rc {
        r
    } else if dz_above_tip <= 0.0 {
        (r - rc).max(0.0)
    } else {
        (r - rc)
            + (2.0 * rc * dz_above_tip - dz_above_tip * dz_above_tip)
                .max(0.0)
                .sqrt()
    }
}

/// Computes the exact 2D material footprint (`RegionSet2D`) cleared at `z_level_wcs` by a slice of
/// upstream [`UpstreamSweptInput`] operations.
pub fn swept_toolpath_coverage_2d(inputs: &[UpstreamSweptInput], z_level_wcs: f64) -> RegionSet2D {
    let mut capsules: Vec<RegionSet2D> = Vec::new();
    let z_tol = 0.03;

    for inp in inputs {
        if inp.is_rotary_native || inp.tool_diameter_mm <= 1e-4 {
            continue;
        }
        for seg3 in &inp.toolpath.segments_wcs {
            if matches!(seg3.motion, SegmentMotion3D::Rapid) {
                continue;
            }
            // Simultaneous rotary motion is never a flat 2D XY capsule in a single planar frame.
            if (seg3.start_rotary_deg - seg3.end_rotary_deg).abs() > 1e-3 {
                continue;
            }
            let seg_max_z = seg3.start_wcs.z.max(seg3.end_wcs.z);
            // Only segments whose entire span stays at or below `z_level_wcs + z_tol` clear the slice cleanly
            if seg_max_z > z_level_wcs + z_tol {
                continue;
            }
            let dz = (z_level_wcs - seg_max_z).max(0.0);
            let r_eff = effective_radius_at_height(inp.tool_diameter_mm, inp.corner_radius_mm, dz);
            if r_eff <= 1e-4 {
                continue;
            }

            let s2 = DVec2::new(seg3.start_wcs.x, seg3.start_wcs.y);
            let e2 = DVec2::new(seg3.end_wcs.x, seg3.end_wcs.y);

            let seg2_opt = match seg3.motion {
                SegmentMotion3D::Rapid => None,
                SegmentMotion3D::Linear { .. } => Some(ArcSegment2D::Line { start: s2, end: e2 }),
                SegmentMotion3D::Arc {
                    plane: ArcPlane::XY,
                    center_wcs,
                    radius_mm,
                    signed_sweep_rad,
                    ..
                } => {
                    let center_xy = DVec2::new(center_wcs.x, center_wcs.y);
                    if signed_sweep_rad.abs() >= std::f64::consts::TAU - 1e-3 {
                        // Full revolution bore/helix clears full annular ring!
                        let outer = RegionSet2D::from_circle(center_xy, radius_mm + r_eff);
                        let ring = if radius_mm > r_eff + 1e-4 {
                            outer
                                .difference(&RegionSet2D::from_circle(center_xy, radius_mm - r_eff))
                        } else {
                            outer
                        };
                        capsules.push(ring);
                        None
                    } else {
                        Some(ArcSegment2D::Arc {
                            start: s2,
                            end: e2,
                            center: center_xy,
                            radius: radius_mm,
                            ccw: signed_sweep_rad >= 0.0,
                        })
                    }
                }
                SegmentMotion3D::Arc { .. } => Some(ArcSegment2D::Line { start: s2, end: e2 }),
            };

            if let Some(seg2) = seg2_opt
                && let Some(cap) = swept_segment_capsule_2d(&seg2, r_eff)
            {
                capsules.push(RegionSet2D::from_single(cap));
            }
        }
    }

    RegionSet2D::union_many(&mut capsules)
}

/// Rest-machined tool-center region paired with its adjacent already-cleared air locus.
#[derive(Debug, Clone)]
pub struct RestPocketWorkItem {
    /// Gouge-proof tool-center domain `Ω_rest = Ω_safe \ Ω_air`.
    pub rest_tool_region: PocketRegion2D,
    /// Already-cleared tool-center air domain `Ω_air` (enables horizontal air entry into corners!).
    pub cleared_air_centers: RegionSet2D,
}

/// Parameters for computing gouge-proof 2.5D Pocket / Adaptive Rest Machining sub-regions.
pub struct PocketRestQuery<'a> {
    pub raw_pocket: &'a PocketRegion2D,
    pub safe_tool_regions: &'a [PocketRegion2D],
    pub current_tool_radius_mm: f64,
    pub radial_stock_to_leave_mm: f64,
    pub rest_cfg: &'a RestMachiningConfig,
    pub upstream: &'a [UpstreamSweptInput],
    pub z_level_wcs: f64,
}

/// Computes gouge-proof Rest Machining tool-center sub-regions (`05-operations.md` §Rest Machining).
pub fn compute_rest_pocket_work_items(query: &PocketRestQuery<'_>) -> Vec<RestPocketWorkItem> {
    let r_curr = query.current_tool_radius_mm.max(0.01);
    let overlap = query.rest_cfg.rest_overlap.value_mm.max(0.0);
    let min_thick = query.rest_cfg.min_rest_thickness.value_mm.max(0.01);

    let cleared_material = match &query.rest_cfg.mode {
        RestMachiningMode::Disabled => return Vec::new(),
        RestMachiningMode::FromPreviousTool {
            prev_tool_diameter,
            prev_stock_to_leave,
        } => {
            let r_prev = 0.5 * prev_tool_diameter.value_mm.max(0.02);
            let s_prev = prev_stock_to_leave.value_mm.max(0.0);
            let prev_centers = RegionSet2D {
                regions: query.raw_pocket.erode_inward(r_prev + s_prev),
            };
            prev_centers.dilate_outward(r_prev)
        }
        RestMachiningMode::FromPrecedingOperations { source_op_ids } => {
            let filtered: Vec<UpstreamSweptInput> = if source_op_ids.is_empty() {
                query.upstream.to_vec()
            } else {
                query
                    .upstream
                    .iter()
                    .filter(|u| source_op_ids.iter().any(|id| id == &u.op_id))
                    .cloned()
                    .collect()
            };
            swept_toolpath_coverage_2d(&filtered, query.z_level_wcs)
        }
    };

    if cleared_material.is_empty() {
        return query
            .safe_tool_regions
            .iter()
            .cloned()
            .map(|r| RestPocketWorkItem {
                rest_tool_region: r,
                cleared_air_centers: RegionSet2D::empty(),
            })
            .collect();
    }

    // Strict zero-engagement air centers (safe for zero-load air entry & micro-lifts!)
    let strict_air_centers = cleared_material.erode_inward((r_curr - 0.02).max(0.01));

    // Exact uncut material thicker than `min_thick` (eliminates 0.000 mm flush straight walls cleanly!):
    let target_mat = RegionSet2D {
        regions: query
            .raw_pocket
            .erode_inward(query.radial_stock_to_leave_mm.max(0.0)),
    };
    let meaningful_uncut_mat = target_mat.difference(&cleared_material.dilate_outward(min_thick));
    if meaningful_uncut_mat.is_empty() {
        return Vec::new();
    }

    // Gouge-proof Rest Tool-Center Domain:
    // `Ω_rest = Ω_safe ∩ dilate_outward(M_uncut, R_curr + overlap)`
    let active_center_mask = meaningful_uncut_mat.dilate_outward(r_curr + overlap);
    let safe_set = RegionSet2D {
        regions: query.safe_tool_regions.to_vec(),
    };
    let rest_set = safe_set.intersect(&active_center_mask);

    let mut items: Vec<RestPocketWorkItem> = rest_set
        .regions
        .into_iter()
        .map(|reg| RestPocketWorkItem {
            rest_tool_region: reg,
            cleared_air_centers: strict_air_centers.clone(),
        })
        .collect();

    // Order disconnected corner regions via nearest-neighbor TSP to minimize rapid travel
    sort_work_items_tsp(&mut items);
    items
}

fn sort_work_items_tsp(items: &mut Vec<RestPocketWorkItem>) {
    if items.len() <= 2 {
        return;
    }
    let mut ordered = Vec::with_capacity(items.len());
    ordered.push(items.remove(0));
    while !items.is_empty() {
        let last_c = ordered
            .last()
            .map_or(DVec2::ZERO, |w| w.rest_tool_region.exact_aabb().center());
        let mut best_i = 0usize;
        let mut best_d = f64::INFINITY;
        for (i, cand) in items.iter().enumerate() {
            let d = last_c.distance_squared(cand.rest_tool_region.exact_aabb().center());
            if d < best_d {
                best_d = d;
                best_i = i;
            }
        }
        ordered.push(items.remove(best_i));
    }
    *items = ordered;
}

/// Parameters for clipping a 2D Contour offset path to uncut Rest Machining spans.
pub struct ContourRestClipQuery<'a> {
    pub raw_contour: &'a ArcPolyline2D,
    pub offset_sign: f64,
    pub candidate_path: &'a ArcPolyline2D,
    pub current_tool_radius_mm: f64,
    pub rest_cfg: &'a RestMachiningConfig,
    pub upstream: &'a [UpstreamSweptInput],
    pub z_level_wcs: f64,
}

/// Clips a 2D Contour tool-center path `candidate_path` to only the corner/channel spans where
/// uncut material remains outside `rest_cfg`'s reference tool or preceding operations.
pub fn clip_contour_path_to_rest(query: &ContourRestClipQuery<'_>) -> Vec<ArcPolyline2D> {
    let r_curr = query.current_tool_radius_mm.max(0.01);
    let overlap = query.rest_cfg.rest_overlap.value_mm.max(0.05);
    let min_thick = query.rest_cfg.min_rest_thickness.value_mm.max(0.02);

    let cleared_material = match &query.rest_cfg.mode {
        RestMachiningMode::Disabled => return vec![query.candidate_path.clone()],
        RestMachiningMode::FromPreviousTool {
            prev_tool_diameter,
            prev_stock_to_leave,
        } => {
            let r_prev = 0.5 * prev_tool_diameter.value_mm.max(0.02);
            let s_prev = prev_stock_to_leave.value_mm.max(0.0);
            let prev_paths =
                offset_polyline(query.raw_contour, query.offset_sign * (r_prev + s_prev));
            let mut caps = Vec::new();
            for p in &prev_paths {
                for seg in &p.segments {
                    if let Some(c) = swept_segment_capsule_2d(seg, r_prev) {
                        caps.push(RegionSet2D::from_single(c));
                    }
                }
            }
            RegionSet2D::union_many(&mut caps)
        }
        RestMachiningMode::FromPrecedingOperations { source_op_ids } => {
            let filtered: Vec<UpstreamSweptInput> = if source_op_ids.is_empty() {
                query.upstream.to_vec()
            } else {
                query
                    .upstream
                    .iter()
                    .filter(|u| source_op_ids.iter().any(|id| id == &u.op_id))
                    .cloned()
                    .collect()
            };
            swept_toolpath_coverage_2d(&filtered, query.z_level_wcs)
        }
    };

    if cleared_material.is_empty() {
        return vec![query.candidate_path.clone()];
    }

    // Compute the swept material band along `candidate_path` that was NOT already cleared by `cleared_material`:
    let mut cand_caps = Vec::new();
    for seg in &query.candidate_path.segments {
        if let Some(c) = swept_segment_capsule_2d(seg, r_curr) {
            cand_caps.push(RegionSet2D::from_single(c));
        }
    }
    let cand_swept = RegionSet2D::union_many(&mut cand_caps);
    let uncut_along_contour = cand_swept.difference(&cleared_material.dilate_outward(min_thick));
    if uncut_along_contour.is_empty() {
        return Vec::new();
    }

    let active_mask = uncut_along_contour.dilate_outward(r_curr + overlap);
    clip_polyline_by_region_set(query.candidate_path, &active_mask, true)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{ParamLength, approx_eq};

    use super::*;

    fn rect_ccw(x0: f64, y0: f64, x1: f64, y1: f64) -> ArcPolyline2D {
        let p0 = DVec2::new(x0, y0);
        let p1 = DVec2::new(x1, y0);
        let p2 = DVec2::new(x1, y1);
        let p3 = DVec2::new(x0, y1);
        ArcPolyline2D {
            segments: vec![
                ArcSegment2D::Line { start: p0, end: p1 },
                ArcSegment2D::Line { start: p1, end: p2 },
                ArcSegment2D::Line { start: p2, end: p3 },
                ArcSegment2D::Line { start: p3, end: p0 },
            ],
            closed: true,
        }
    }

    #[test]
    fn region_set_booleans_and_from_previous_tool_isolate_four_pocket_corners() {
        // 40 x 30 mm sharp-cornered pocket roughed with a Ø12 mm endmill (R = 6 mm) and
        // rest-machined with a Ø4 mm endmill (R = 2 mm):
        // The central floor is already cleared; ONLY the 4 sharp corners (where R=6 leaves cusps
        // that R=2 can reach!) should remain as 4 disconnected gouge-proof Rest regions!
        let raw_pocket = PocketRegion2D::new(rect_ccw(0.0, 0.0, 40.0, 30.0), Vec::new());
        let safe_r2 = raw_pocket.erode_inward(2.0);
        assert_eq!(safe_r2.len(), 1);

        let rest_cfg = RestMachiningConfig {
            mode: RestMachiningMode::FromPreviousTool {
                prev_tool_diameter: ParamLength::from_mm(12.0),
                prev_stock_to_leave: ParamLength::from_mm(0.0),
            },
            rest_overlap: ParamLength::from_mm(0.25),
            min_rest_thickness: ParamLength::from_mm(0.05),
        };

        let items = compute_rest_pocket_work_items(&PocketRestQuery {
            raw_pocket: &raw_pocket,
            safe_tool_regions: &safe_r2,
            current_tool_radius_mm: 2.0,
            radial_stock_to_leave_mm: 0.0,
            rest_cfg: &rest_cfg,
            upstream: &[],
            z_level_wcs: -5.0,
        });

        assert_eq!(
            items.len(),
            4,
            "Ø12mm -> Ø4mm rest machining on a rectangle must isolate the 4 corner regions"
        );
        // Pocket center (20, 15) must be excluded from all rest tool regions (zero central air-cutting!)
        for item in &items {
            assert!(
                !item
                    .rest_tool_region
                    .contains_interior_point(DVec2::new(20.0, 15.0))
            );
            // Every rest region must stay strictly inside `[2..38] x [2..28]` (gouge-proof!)
            let b = item.rest_tool_region.exact_aabb();
            assert!(b.min.x >= 2.0 - 1e-3 && b.max.x <= 38.0 + 1e-3);
            assert!(b.min.y >= 2.0 - 1e-3 && b.max.y <= 28.0 + 1e-3);
        }

        // Verify union area identity on disjoint + overlapping disks
        let d1 = RegionSet2D::from_circle(DVec2::new(0.0, 0.0), 5.0);
        let d2 = RegionSet2D::from_circle(DVec2::new(20.0, 0.0), 5.0);
        let u = d1.union(&d2);
        assert!(approx_eq(
            u.total_area_mm2(),
            2.0 * std::f64::consts::PI * 25.0,
            0.2
        ));
    }

    #[test]
    fn filter_upstream_for_indexed_angle_scopes_by_mod_360_and_excludes_rotary_native() {
        use crate::{cl_data::MotionRole, toolpath::ToolpathBuilder};
        use chip_math::DVec3;

        let make_tp = |id: &str, start_deg: f64, end_deg: f64| {
            let mut b = ToolpathBuilder::new(id);
            b.rotary_linear_to(
                DVec3::new(0.0, 0.0, -5.0),
                start_deg,
                500.0,
                10.0,
                MotionRole::Cut,
            );
            b.rotary_linear_to(
                DVec3::new(20.0, 0.0, -5.0),
                end_deg,
                500.0,
                10.0,
                MotionRole::Cut,
            );
            Arc::new(b.finish())
        };

        let inputs = vec![
            UpstreamSweptInput {
                op_id: "plain_3axis".to_string(),
                tool_diameter_mm: 6.0,
                corner_radius_mm: 0.0,
                toolpath: make_tp("plain_3axis", 0.0, 0.0),
                indexed_angle_deg: None,
                is_rotary_native: false,
            },
            UpstreamSweptInput {
                op_id: "idx_0".to_string(),
                tool_diameter_mm: 6.0,
                corner_radius_mm: 0.0,
                toolpath: make_tp("idx_0", 0.0, 0.0),
                indexed_angle_deg: Some(0.0),
                is_rotary_native: false,
            },
            UpstreamSweptInput {
                op_id: "idx_359_9995".to_string(),
                tool_diameter_mm: 6.0,
                corner_radius_mm: 0.0,
                toolpath: make_tp("idx_359_9995", 359.9995, 359.9995),
                indexed_angle_deg: Some(359.9995),
                is_rotary_native: false,
            },
            UpstreamSweptInput {
                op_id: "idx_neg90".to_string(),
                tool_diameter_mm: 6.0,
                corner_radius_mm: 0.0,
                toolpath: make_tp("idx_neg90", -90.0, -90.0),
                indexed_angle_deg: Some(-90.0),
                is_rotary_native: false,
            },
            UpstreamSweptInput {
                op_id: "idx_120".to_string(),
                tool_diameter_mm: 6.0,
                corner_radius_mm: 0.0,
                toolpath: make_tp("idx_120", 120.0, 120.0),
                indexed_angle_deg: Some(120.0),
                is_rotary_native: false,
            },
            UpstreamSweptInput {
                op_id: "rotary_sim".to_string(),
                tool_diameter_mm: 6.0,
                corner_radius_mm: 0.0,
                toolpath: make_tp("rotary_sim", 0.0, 90.0),
                indexed_angle_deg: Some(0.0),
                is_rotary_native: true,
            },
        ];

        let f_none: Vec<_> = filter_upstream_for_indexed_angle(&inputs, None)
            .into_iter()
            .map(|u| u.op_id)
            .collect();
        assert_eq!(f_none, vec!["plain_3axis"]);

        let f_0: Vec<_> = filter_upstream_for_indexed_angle(&inputs, Some(0.0))
            .into_iter()
            .map(|u| u.op_id)
            .collect();
        assert_eq!(f_0, vec!["idx_0", "idx_359_9995"]);

        let f_270: Vec<_> = filter_upstream_for_indexed_angle(&inputs, Some(270.0))
            .into_iter()
            .map(|u| u.op_id)
            .collect();
        assert_eq!(f_270, vec!["idx_neg90"]);

        let f_240 = filter_upstream_for_indexed_angle(&inputs, Some(240.0));
        assert!(f_240.is_empty());

        // Defensive check: simultaneous rotary segment with `start_rotary_deg != end_rotary_deg`
        // must be skipped by `swept_toolpath_coverage_2d` even if `is_rotary_native == false`.
        let sim_unmarked = [UpstreamSweptInput {
            op_id: "sim_unmarked".to_string(),
            tool_diameter_mm: 6.0,
            corner_radius_mm: 0.0,
            toolpath: make_tp("sim_unmarked", 0.0, 45.0),
            indexed_angle_deg: None,
            is_rotary_native: false,
        }];
        assert!(swept_toolpath_coverage_2d(&sim_unmarked, -5.0).is_empty());
    }
}
