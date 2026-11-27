use std::collections::BTreeMap;

use chip_geom2d::PocketRegion2D;
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2};

use crate::{
    params::{MachiningBoundaryConfig, MachiningBoundaryMode, ToolContainmentMode},
    rest2d::RegionSet2D,
    selection::{ExtractedFeature25D, ExtractedSelectionSummary},
    setup::ResolvedSetup,
    stock::ResolvedStockGeometry,
    surface3d::{
        SurfaceCutterProfile, SurfaceMesh3D, drop_cutter_z_with_scratch,
        slab_projection::project_surface_z_slab_xy,
    },
};

/// Structured result of resolving a 3D operation's 2D XY machining containment boundary.
///
/// Shared between all 3D toolpath strategies and viewport boundary preview so both render and cut
/// against the exact same nominal and effective regions.
#[derive(Debug, Clone)]
pub struct ResolvedBoundary2D {
    /// Raw nominal boundary loops before tool-radius / offset dilation (for viewport display).
    pub nominal_region: RegionSet2D,
    /// Effective tool-center containment region after `tool_containment` + `boundary_offset`
    /// (and after subtracting `avoid_effective_region` when keep-out regions are active).
    pub effective_region: RegionSet2D,
    /// Raw nominal Avoid / Keep-Out footprints (for 3D viewport keep-out overlay display).
    pub avoid_nominal_region: RegionSet2D,
    /// Effective tool-center Avoid / Keep-Out footprints subtracted from `effective_region`.
    pub avoid_effective_region: RegionSet2D,
    /// Non-fatal diagnostic warnings (e.g., `SelectedRegions` chosen with no selections).
    pub warnings: Vec<String>,
}

impl ResolvedBoundary2D {
    /// Subtracts `avoid.effective_region` from `self.effective_region` and stores both nominal
    /// and effective keep-out footprints plus any warnings onto `self`.
    pub fn apply_avoid(&mut self, avoid: crate::avoid_regions::ResolvedAvoidRegion2D) {
        self.warnings.extend(avoid.warnings);
        if !avoid.effective_region.is_empty() && !self.effective_region.is_empty() {
            self.effective_region = self.effective_region.difference(&avoid.effective_region);
        }
        self.avoid_nominal_region = avoid.nominal_region;
        self.avoid_effective_region = avoid.effective_region;
    }
}

/// Detects significant upward-facing horizontal flat plateaus on `surface` inside `[z_min, z_max]`,
/// returning deduplicated plateau Z heights sorted descending (top-down).
pub fn detect_flat_plateaus(
    surface: &SurfaceMesh3D,
    z_min: f64,
    z_max: f64,
    min_area_mm2: f64,
) -> Vec<f64> {
    if !z_min.is_finite() || !z_max.is_finite() || z_max < z_min {
        return Vec::new();
    }
    let min_area = min_area_mm2.max(0.25);

    // Bucket upward horizontal triangles (`normal.z > 0.995`) into `0.01 mm` Z bins
    let mut buckets: BTreeMap<i64, (f64, f64)> = BTreeMap::new(); // key -> (area_sum, area_weighted_z)
    for tri in &surface.triangles {
        if tri.normal.z < 0.995 {
            continue;
        }
        let [i0, i1, i2] = tri.indices;
        let v0 = surface.vertices[i0 as usize];
        let v1 = surface.vertices[i1 as usize];
        let v2 = surface.vertices[i2 as usize];
        let z_avg = (v0.z + v1.z + v2.z) / 3.0;
        if z_avg < z_min - 1e-4 || z_avg > z_max + 1e-4 {
            continue;
        }
        let area = 0.5 * (v1 - v0).cross(v2 - v0).length();
        if !area.is_finite() || area <= 1e-8 {
            continue;
        }
        let key = (z_avg * 100.0).round() as i64;
        let entry = buckets.entry(key).or_insert((0.0, 0.0));
        entry.0 += area;
        entry.1 += area * z_avg;
    }

    // Merge adjacent buckets within `0.02 mm` and keep clusters with total area >= `min_area`
    let mut clusters: Vec<(f64, f64)> = Vec::new(); // (area_sum, weighted_z_sum)
    for (_key, (area, wz)) in buckets {
        let z_bin = wz / area;
        if let Some((last_a, last_wz)) = clusters.last_mut() {
            let last_z = *last_wz / *last_a;
            if (z_bin - last_z).abs() <= 0.025 {
                *last_a += area;
                *last_wz += wz;
                continue;
            }
        }
        clusters.push((area, wz));
    }

    let mut heights: Vec<f64> = clusters
        .into_iter()
        .filter(|(a, _)| *a >= min_area)
        .map(|(a, wz)| wz / a)
        .collect();
    heights.sort_by(|a, b| b.total_cmp(a));
    heights
}

/// Projects the true 2D XY silhouette of `surface` onto the active Setup XY plane using
/// [`project_surface_z_slab_xy`] across the full model Z span, clamping outer AABB boundaries to
/// exact `surface.aabb_wcs` coordinates and falling back to [`rect_region_2d`] only if projection
/// yields an empty polygon on a degenerate mesh.
///
/// When `fill_holes == true`, interior through-hole voids (`holes_cw`) are filled so machining
/// boundaries cover the full outer silhouette rather than leaving uncut stock pillars inside
/// through-holes.
fn model_silhouette_xy_region(
    surface: &SurfaceMesh3D,
    setup: &ResolvedSetup,
    fill_holes: bool,
) -> RegionSet2D {
    if !surface.bounds.is_valid() {
        return stock_xy_region(setup);
    }
    let aabb_rect = rect_region_2d(
        surface.bounds.min.x,
        surface.bounds.min.y,
        surface.bounds.max.x,
        surface.bounds.max.y,
    );
    let span_x = (surface.bounds.max.x - surface.bounds.min.x).max(0.1);
    let span_y = (surface.bounds.max.y - surface.bounds.min.y).max(0.1);
    let cell_mm = (span_x.max(span_y) / 300.0).clamp(0.05, 0.25);

    let projected = project_surface_z_slab_xy(
        surface,
        surface.bounds.min.z - 1.0,
        surface.bounds.max.z + 1.0,
        cell_mm,
    );
    if projected.is_empty() {
        return aabb_rect;
    }
    let projected = if fill_holes && projected.regions.iter().any(|r| !r.islands_cw.is_empty()) {
        let mut outer_sets: Vec<RegionSet2D> = projected
            .regions
            .iter()
            .map(|r| RegionSet2D::from_single(PocketRegion2D::new(r.outer_ccw.clone(), Vec::new())))
            .collect();
        let filled = RegionSet2D::union_many(&mut outer_sets);
        if filled.is_empty() { projected } else { filled }
    } else {
        projected
    };
    let clipped = projected.intersect(&aabb_rect);
    if clipped.is_empty() {
        projected
    } else {
        clipped
    }
}

/// Resolves a [`MachiningBoundaryConfig`] into both its raw `nominal_region` (before tool radius
/// and offset dilation) and its tool-center `effective_region`, along with any non-fatal warnings.
pub fn resolve_3d_boundary_config_detailed(
    surface: &SurfaceMesh3D,
    setup: &ResolvedSetup,
    selection: &ExtractedSelectionSummary,
    config: &MachiningBoundaryConfig,
    tool_radius_mm: f64,
) -> ResolvedBoundary2D {
    resolve_3d_containment_region_impl(
        surface,
        setup,
        selection,
        config.boundary_mode,
        config.tool_containment,
        config.boundary_offset.value_mm,
        config.silhouette_fill_holes,
        [
            config.model_bounds_offset_x_neg.value_mm,
            config.model_bounds_offset_x_pos.value_mm,
            config.model_bounds_offset_y_neg.value_mm,
            config.model_bounds_offset_y_pos.value_mm,
        ],
        tool_radius_mm,
    )
}

/// Resolves the 2D XY machining boundary into both its raw `nominal_region` (before tool radius
/// and offset dilation) and its tool-center `effective_region`, along with any non-fatal warnings.
pub fn resolve_3d_containment_region_detailed(
    surface: &SurfaceMesh3D,
    setup: &ResolvedSetup,
    selection: &ExtractedSelectionSummary,
    mode: MachiningBoundaryMode,
    containment: ToolContainmentMode,
    extra_offset_mm: f64,
    tool_radius_mm: f64,
) -> ResolvedBoundary2D {
    resolve_3d_containment_region_impl(
        surface,
        setup,
        selection,
        mode,
        containment,
        extra_offset_mm,
        true,
        [0.0; 4],
        tool_radius_mm,
    )
}

#[allow(clippy::too_many_arguments)]
fn resolve_3d_containment_region_impl(
    surface: &SurfaceMesh3D,
    setup: &ResolvedSetup,
    selection: &ExtractedSelectionSummary,
    mode: MachiningBoundaryMode,
    containment: ToolContainmentMode,
    extra_offset_mm: f64,
    silhouette_fill_holes: bool,
    model_bounds_offsets_mm: [f64; 4],
    tool_radius_mm: f64,
) -> ResolvedBoundary2D {
    let mut warnings = Vec::new();
    let mut model_box_bounds: Option<(f64, f64, f64, f64)> = None;
    let mut model_box_collapsed = false;
    let nominal_region = match mode {
        MachiningBoundaryMode::StockBox => stock_xy_region(setup),
        MachiningBoundaryMode::ModelBoundingBox => {
            let finite_or_zero = |v: f64| if v.is_finite() { v } else { 0.0 };
            let off_x_neg = finite_or_zero(model_bounds_offsets_mm[0]);
            let off_x_pos = finite_or_zero(model_bounds_offsets_mm[1]);
            let off_y_neg = finite_or_zero(model_bounds_offsets_mm[2]);
            let off_y_pos = finite_or_zero(model_bounds_offsets_mm[3]);
            if surface.bounds.is_valid() {
                let x_min = surface.bounds.min.x - off_x_neg;
                let x_max = surface.bounds.max.x + off_x_pos;
                let y_min = surface.bounds.min.y - off_y_neg;
                let y_max = surface.bounds.max.y + off_y_pos;
                if x_max <= x_min + 1e-4 || y_max <= y_min + 1e-4 {
                    model_box_collapsed = true;
                    warnings.push(format!(
                        "Model Bounds collapsed to an empty XY region (X: [{x_min:.2}, {x_max:.2}], Y: [{y_min:.2}, {y_max:.2}] mm) because negative per-side offsets exceed the model span."
                    ));
                    RegionSet2D::empty()
                } else {
                    model_box_bounds = Some((x_min, y_min, x_max, y_max));
                    rect_region_2d(x_min, y_min, x_max, y_max)
                }
            } else {
                stock_xy_region(setup)
            }
        }
        MachiningBoundaryMode::ModelSilhouette => {
            model_silhouette_xy_region(surface, setup, silhouette_fill_holes)
        }
        MachiningBoundaryMode::SelectedRegions => {
            let sel_reg = selected_features_xy_region(selection);
            if sel_reg.is_empty() {
                warnings.push(
                    "Machining boundary is set to 'Selected 2D Contours' but no closed 2D contours or planar faces are selected; falling back to Model Silhouette."
                        .to_string(),
                );
                model_silhouette_xy_region(surface, setup, silhouette_fill_holes)
            } else {
                sel_reg
            }
        }
    };

    let tool_shift = match containment {
        ToolContainmentMode::ToolInsideBoundary => -tool_radius_mm,
        ToolContainmentMode::ToolCenterOnBoundary => 0.0,
        ToolContainmentMode::ToolOutsideBoundary => tool_radius_mm,
    };
    let delta = tool_shift
        + if extra_offset_mm.is_finite() {
            extra_offset_mm
        } else {
            0.0
        };

    let effective_region = if model_box_collapsed {
        RegionSet2D::empty()
    } else if let Some((x_min, y_min, x_max, y_max)) = model_box_bounds {
        let ex_min = x_min - delta;
        let ex_max = x_max + delta;
        let ey_min = y_min - delta;
        let ey_max = y_max + delta;
        if ex_max <= ex_min + 1e-4 || ey_max <= ey_min + 1e-4 {
            warnings.push(
                "Effective Model Bounds collapsed to an empty XY region after applying Tool Containment and Boundary Offset."
                    .to_string(),
            );
            RegionSet2D::empty()
        } else {
            rect_region_2d(ex_min, ey_min, ex_max, ey_max)
        }
    } else if mode == MachiningBoundaryMode::StockBox {
        match setup.stock {
            ResolvedStockGeometry::Cylinder {
                axis: crate::stock::StockAxis::Z,
                center_wcs,
                radius_mm,
                ..
            } => {
                RegionSet2D::from_circle(DVec2::new(center_wcs.x, center_wcs.y), radius_mm + delta)
            }
            ResolvedStockGeometry::Box { aabb_wcs }
            | ResolvedStockGeometry::Cylinder { aabb_wcs, .. } => rect_region_2d(
                aabb_wcs.min.x - delta,
                aabb_wcs.min.y - delta,
                aabb_wcs.max.x + delta,
                aabb_wcs.max.y + delta,
            ),
        }
    } else if delta > 1e-5 {
        nominal_region.dilate_outward(delta)
    } else if delta < -1e-5 {
        nominal_region.erode_inward(-delta)
    } else {
        nominal_region.clone()
    };

    ResolvedBoundary2D {
        nominal_region,
        effective_region,
        avoid_nominal_region: RegionSet2D::empty(),
        avoid_effective_region: RegionSet2D::empty(),
        warnings,
    }
}

/// Evaluates the effective local surface slope angle in degrees (`0° = flat horizontal`,
/// `90° = vertical wall`) seen by `cutter` at `(pt_xy, z_level)` using exact Drop-Cutter
/// central differences.
pub fn sample_surface_slope_deg(
    surface: &SurfaceMesh3D,
    cutter: &SurfaceCutterProfile,
    pt_xy: DVec2,
    z_floor: f64,
    scratch: &mut Vec<usize>,
) -> f64 {
    let h = (0.15 * cutter.max_radius_mm()).clamp(0.05, 0.25);
    let z_c = drop_cutter_z_with_scratch(surface, cutter, pt_xy.x, pt_xy.y, z_floor, scratch)
        .unwrap_or(z_floor);
    let z_px = drop_cutter_z_with_scratch(surface, cutter, pt_xy.x + h, pt_xy.y, z_floor, scratch)
        .unwrap_or(z_c);
    let z_nx = drop_cutter_z_with_scratch(surface, cutter, pt_xy.x - h, pt_xy.y, z_floor, scratch)
        .unwrap_or(z_c);
    let z_py = drop_cutter_z_with_scratch(surface, cutter, pt_xy.x, pt_xy.y + h, z_floor, scratch)
        .unwrap_or(z_c);
    let z_ny = drop_cutter_z_with_scratch(surface, cutter, pt_xy.x, pt_xy.y - h, z_floor, scratch)
        .unwrap_or(z_c);

    // Use max of one-sided and central difference so sharp cliff edges report steep slopes accurately
    let dx = ((z_px - z_nx) / (2.0 * h))
        .abs()
        .max(((z_px - z_c) / h).abs())
        .max(((z_c - z_nx) / h).abs());
    let dy = ((z_py - z_ny) / (2.0 * h))
        .abs()
        .max(((z_py - z_c) / h).abs())
        .max(((z_c - z_ny) / h).abs());
    let grad = dx.hypot(dy);
    grad.atan().to_degrees().clamp(0.0, 90.0)
}

/// Builds a CCW rectangular [`RegionSet2D`] covering `[min_x..max_x] × [min_y..max_y]`.
pub fn rect_region_2d(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> RegionSet2D {
    if max_x - min_x <= 1e-4 || max_y - min_y <= 1e-4 {
        return RegionSet2D::empty();
    }
    let p0 = DVec2::new(min_x, min_y);
    let p1 = DVec2::new(max_x, min_y);
    let p2 = DVec2::new(max_x, max_y);
    let p3 = DVec2::new(min_x, max_y);
    let outer = ArcPolyline2D {
        segments: vec![
            ArcSegment2D::Line { start: p0, end: p1 },
            ArcSegment2D::Line { start: p1, end: p2 },
            ArcSegment2D::Line { start: p2, end: p3 },
            ArcSegment2D::Line { start: p3, end: p0 },
        ],
        closed: true,
    };
    RegionSet2D::from_single(PocketRegion2D::new(outer, Vec::new()))
}

/// Returns the exact 2D XY stock footprint [`RegionSet2D`] of `setup`.
pub fn stock_xy_region(setup: &ResolvedSetup) -> RegionSet2D {
    match setup.stock {
        ResolvedStockGeometry::Box { aabb_wcs } => rect_region_2d(
            aabb_wcs.min.x,
            aabb_wcs.min.y,
            aabb_wcs.max.x,
            aabb_wcs.max.y,
        ),
        ResolvedStockGeometry::Cylinder {
            axis: crate::stock::StockAxis::Z,
            center_wcs,
            radius_mm,
            ..
        } => RegionSet2D::from_circle(DVec2::new(center_wcs.x, center_wcs.y), radius_mm),
        ResolvedStockGeometry::Cylinder { aabb_wcs, .. } => rect_region_2d(
            aabb_wcs.min.x,
            aabb_wcs.min.y,
            aabb_wcs.max.x,
            aabb_wcs.max.y,
        ),
    }
}

fn selected_features_xy_region(selection: &ExtractedSelectionSummary) -> RegionSet2D {
    let mut sets = Vec::new();
    for feat in &selection.features {
        match feat {
            ExtractedFeature25D::PlanarFace {
                outer_boundary_wcs,
                inner_islands_wcs,
                ..
            } => {
                let outer = normalize_ccw(outer_boundary_wcs);
                let islands: Vec<ArcPolyline2D> =
                    inner_islands_wcs.iter().map(normalize_cw).collect();
                let reg = RegionSet2D::from_single(PocketRegion2D::new(outer, islands));
                if !reg.is_empty() {
                    sets.push(reg);
                }
            }
            ExtractedFeature25D::Contour { polyline_wcs, .. } if polyline_wcs.closed => {
                let outer = normalize_ccw(polyline_wcs);
                let reg = RegionSet2D::from_single(PocketRegion2D::new(outer, Vec::new()));
                if !reg.is_empty() {
                    sets.push(reg);
                }
            }
            _ => {}
        }
    }
    RegionSet2D::union_many(&mut sets)
}

/// Clips `poly` to keep only portions lying inside `region` (within `1e-4 mm` tolerance),
/// returning either the original closed loop if fully inside or clean open sub-polylines.
pub fn clip_polyline_to_region(
    poly: &ArcPolyline2D,
    region: &RegionSet2D,
    sample_step_mm: f64,
) -> Vec<ArcPolyline2D> {
    if poly.segments.is_empty() || region.is_empty() {
        return Vec::new();
    }
    let step = sample_step_mm.clamp(0.1, 2.0);
    let inside_pt =
        |pt: DVec2| region.contains_point(pt) || region.min_distance_to_boundary(pt) <= 1e-3;

    // Fast path: check if every sampled point along `poly` is inside `region`
    let mut all_inside = true;
    for seg in &poly.segments {
        let len = seg.length();
        let n = (len / step).ceil().max(1.0) as usize;
        for k in 0..=n {
            let t = k as f64 / n as f64;
            if !inside_pt(sample_seg(seg, t)) {
                all_inside = false;
                break;
            }
        }
        if !all_inside {
            break;
        }
    }
    if all_inside {
        return vec![poly.clone()];
    }

    // Subdivide & keep contiguous inside runs
    let mut chains: Vec<Vec<DVec2>> = Vec::new();
    let mut current: Vec<DVec2> = Vec::new();

    for seg in &poly.segments {
        let len = seg.length();
        let n = (len / step).ceil().max(1.0) as usize;
        for k in 0..n {
            let t0 = k as f64 / n as f64;
            let t1 = (k + 1) as f64 / n as f64;
            let p0 = sample_seg(seg, t0);
            let p1 = sample_seg(seg, t1);
            let mid = sample_seg(seg, 0.5 * (t0 + t1));
            if inside_pt(p0) && inside_pt(p1) && inside_pt(mid) {
                if current.is_empty() {
                    current.push(p0);
                }
                if current
                    .last()
                    .is_none_or(|l| l.distance_squared(p1) > 1e-10)
                {
                    current.push(p1);
                }
            } else if current.len() >= 2 {
                chains.push(std::mem::take(&mut current));
            } else {
                current.clear();
            }
        }
    }
    if current.len() >= 2 {
        chains.push(current);
    }

    // Merge wrap-around first & last chain if original polyline was closed
    if poly.closed
        && chains.len() >= 2
        && let (Some(first_pt), Some(last_pt)) = (
            chains.first().and_then(|c| c.first()).copied(),
            chains.last().and_then(|c| c.last()).copied(),
        )
        && first_pt.distance(last_pt) <= 1e-3
        && let Some(mut tail) = chains.pop()
    {
        tail.pop();
        tail.extend(chains[0].iter().copied());
        chains[0] = tail;
    }

    chains
        .into_iter()
        .filter_map(|pts| points_to_polyline(&pts, false))
        .collect()
}

pub(crate) fn sample_seg(seg: &ArcSegment2D, t: f64) -> DVec2 {
    match *seg {
        ArcSegment2D::Line { start, end } => start.lerp(end, t),
        ArcSegment2D::Arc {
            start,
            end,
            center,
            radius,
            ccw,
        } => {
            let a0 = (start.y - center.y).atan2(start.x - center.x);
            let a1 = (end.y - center.y).atan2(end.x - center.x);
            let mut sweep = a1 - a0;
            if ccw {
                if sweep <= 0.0 {
                    sweep += std::f64::consts::TAU;
                }
            } else if sweep >= 0.0 {
                sweep -= std::f64::consts::TAU;
            }
            let ang = a0 + sweep * t;
            center + DVec2::new(radius * ang.cos(), radius * ang.sin())
        }
    }
}

pub(crate) fn points_to_polyline(pts: &[DVec2], closed: bool) -> Option<ArcPolyline2D> {
    if pts.len() < 2 {
        return None;
    }
    let mut segs = Vec::with_capacity(pts.len());
    for w in pts.windows(2) {
        if w[0].distance_squared(w[1]) > 1e-10 {
            segs.push(ArcSegment2D::Line {
                start: w[0],
                end: w[1],
            });
        }
    }
    if closed
        && let (Some(&first), Some(&last)) = (pts.first(), pts.last())
        && first.distance_squared(last) > 1e-10
    {
        segs.push(ArcSegment2D::Line {
            start: last,
            end: first,
        });
    }
    if segs.is_empty() {
        None
    } else {
        Some(ArcPolyline2D {
            segments: segs,
            closed,
        })
    }
}

pub(crate) fn polyline_signed_area(poly: &ArcPolyline2D) -> f64 {
    let mut area2 = 0.0f64;
    for seg in &poly.segments {
        let a = seg.start();
        let b = seg.end();
        area2 += a.x * b.y - b.x * a.y;
    }
    0.5 * area2
}

pub(crate) fn reverse_arc_polyline(poly: &ArcPolyline2D) -> ArcPolyline2D {
    let segments = poly
        .segments
        .iter()
        .rev()
        .map(|seg| match *seg {
            ArcSegment2D::Line { start, end } => ArcSegment2D::Line {
                start: end,
                end: start,
            },
            ArcSegment2D::Arc {
                start,
                end,
                center,
                radius,
                ccw,
            } => ArcSegment2D::Arc {
                start: end,
                end: start,
                center,
                radius,
                ccw: !ccw,
            },
        })
        .collect();
    ArcPolyline2D {
        segments,
        closed: poly.closed,
    }
}

pub(crate) fn normalize_ccw(poly: &ArcPolyline2D) -> ArcPolyline2D {
    if polyline_signed_area(poly) < 0.0 {
        reverse_arc_polyline(poly)
    } else {
        poly.clone()
    }
}

pub(crate) fn normalize_cw(poly: &ArcPolyline2D) -> ArcPolyline2D {
    if polyline_signed_area(poly) > 0.0 {
        reverse_arc_polyline(poly)
    } else {
        poly.clone()
    }
}
