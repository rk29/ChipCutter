//! Segment-Voronoi interior medial axis & maximum inscribed circle pole extraction (`boostvoronoi`).
//!
//! Used by **2D Adaptive Clearing** (for locating optimal helical entry poles inside narrow, curved,
//! or Rest-Machined corner cavities) and forms the geometric foundation for Milestone 9 **V-Carving**.

use boostvoronoi::{
    builder::Builder,
    geometry::{Line, Point},
};
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2};

use crate::offset2d::{HelicalEntryPole, PocketRegion2D, find_helical_entry_pole};

/// Interior medial-axis sample node with its exact clearance radius to the region boundary.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MedialNode2D {
    pub pos_wcs: DVec2,
    pub clearance_radius_mm: f64,
}

/// Discretizes an [`ArcPolyline2D`] into closed non-degenerate linear chords within `chord_tol_mm`
/// suitable for integer-scaled segment Voronoi construction.
fn discretize_loop_chords(poly: &ArcPolyline2D, chord_tol_mm: f64) -> Vec<(DVec2, DVec2)> {
    let mut pts: Vec<DVec2> = Vec::with_capacity(poly.segments.len() * 4);
    let tol = chord_tol_mm.clamp(0.005, 0.25);

    for seg in &poly.segments {
        match *seg {
            ArcSegment2D::Line { start, end } => {
                if start.distance(end) > 1e-4 {
                    push_unique_pt(&mut pts, start, 2e-3);
                }
            }
            ArcSegment2D::Arc { radius, .. } => {
                let sweep = seg.signed_sweep_rad().abs();
                let step_ang = if radius > tol {
                    (2.0 * (1.0 - tol / radius).clamp(-1.0, 1.0).acos()).clamp(0.08, 0.45)
                } else {
                    0.45
                };
                let steps = ((sweep / step_ang).ceil() as usize).clamp(2, 96);
                for k in 0..steps {
                    let t = (k as f64) / (steps as f64);
                    if let Some((p, _)) = seg.eval_at(t) {
                        push_unique_pt(&mut pts, p, 2e-3);
                    }
                }
            }
        }
    }

    if pts.len() < 3 {
        return Vec::new();
    }
    if let (Some(&first), Some(&last)) = (pts.first(), pts.last())
        && first.distance(last) <= 2e-3
    {
        pts.pop();
    }
    if pts.len() < 3 {
        return Vec::new();
    }

    let mut segs = Vec::with_capacity(pts.len());
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        if a.distance(b) > 2e-3 {
            segs.push((a, b));
        }
    }
    segs
}

fn push_unique_pt(pts: &mut Vec<DVec2>, p: DVec2, min_dist: f64) {
    if !p.is_finite() {
        return;
    }
    if pts.last().is_none_or(|prev| prev.distance(p) > min_dist) {
        pts.push(p);
    }
}

/// Extracts interior medial-axis circle-event vertices of `region` using `boostvoronoi`.
pub fn extract_interior_medial_nodes(
    region: &PocketRegion2D,
    chord_tol_mm: f64,
) -> Vec<MedialNode2D> {
    let aabb = region.exact_aabb();
    if !aabb.is_valid() {
        return Vec::new();
    }
    let center = aabb.center();
    let max_span = aabb.size().x.max(aabb.size().y).max(1.0);
    // Map region coordinates into safe 32-bit integer space (`[-50_000_000, +50_000_000]`)
    let scale = (50_000_000.0 / max_span).clamp(100.0, 100_000.0);

    let mut raw_chords = discretize_loop_chords(&region.outer_ccw, chord_tol_mm);
    for isl in &region.islands_cw {
        raw_chords.extend(discretize_loop_chords(isl, chord_tol_mm));
    }
    if raw_chords.len() < 3 {
        return Vec::new();
    }

    let to_i32 = |p: DVec2| -> Point<i32> {
        let ix = ((p.x - center.x) * scale).round().clamp(-1.0e8, 1.0e8) as i32;
        let iy = ((p.y - center.y) * scale).round().clamp(-1.0e8, 1.0e8) as i32;
        Point { x: ix, y: iy }
    };

    let mut int_lines: Vec<Line<i32>> = Vec::with_capacity(raw_chords.len());
    for (a, b) in raw_chords {
        let pa = to_i32(a);
        let pb = to_i32(b);
        if pa != pb {
            int_lines.push(Line::new(pa, pb));
        }
    }
    if int_lines.len() < 3 {
        return Vec::new();
    }

    let Ok(builder_with_segs) = Builder::<i32>::default().with_segments(int_lines.iter()) else {
        return Vec::new();
    };
    let Ok(diagram) = builder_with_segs.build() else {
        return Vec::new();
    };

    let inv_scale = 1.0 / scale;
    let mut nodes = Vec::new();
    for v in diagram.vertices() {
        if v.is_site_point() {
            continue;
        }
        let vx = center.x + v.x() * inv_scale;
        let vy = center.y + v.y() * inv_scale;
        let pt = DVec2::new(vx, vy);
        if !pt.is_finite() || !aabb.contains_point(pt) {
            continue;
        }
        if region.contains_interior_point(pt) {
            let clr = region.min_boundary_distance(pt);
            if clr > 1e-5 {
                nodes.push(MedialNode2D {
                    pos_wcs: pt,
                    clearance_radius_mm: clr,
                });
            }
        }
    }

    // Also sample midpoints of strictly interior Voronoi edges so constant-width channels (like slots
    // or annular rings between concentric circles, which have few circle-event vertices!) still yield
    // high-accuracy medial poles!
    for idx in 0..diagram.num_edges() {
        let eid = diagram.edge_index_unchecked(idx);
        let Ok(Some(v0_id)) = diagram.edge_get_vertex0(eid) else {
            continue;
        };
        let Ok(Some(v1_id)) = diagram.edge_get_vertex1(eid) else {
            continue;
        };
        if v0_id.usize() >= v1_id.usize() {
            continue;
        }
        let (Ok(v0), Ok(v1)) = (diagram.vertex(v0_id), diagram.vertex(v1_id)) else {
            continue;
        };
        let p0 = DVec2::new(center.x + v0.x() * inv_scale, center.y + v0.y() * inv_scale);
        let p1 = DVec2::new(center.x + v1.x() * inv_scale, center.y + v1.y() * inv_scale);
        let mid = 0.5 * (p0 + p1);
        if mid.is_finite() && aabb.contains_point(mid) && region.contains_interior_point(mid) {
            let clr = region.min_boundary_distance(mid);
            if clr > 1e-5 {
                nodes.push(MedialNode2D {
                    pos_wcs: mid,
                    clearance_radius_mm: clr,
                });
            }
        }
    }

    nodes
}

/// Finds the maximum inscribed circle pole inside `tool_center_region` by combining exact
/// `boostvoronoi` medial-axis circle events with local pattern refinement and grid fallback.
pub fn find_medial_inscribed_pole(tool_center_region: &PocketRegion2D) -> Option<HelicalEntryPole> {
    let medial_nodes = extract_interior_medial_nodes(tool_center_region, 0.05);
    let fallback = find_helical_entry_pole(tool_center_region);

    let best_medial = medial_nodes
        .into_iter()
        .max_by(|a, b| a.clearance_radius_mm.total_cmp(&b.clearance_radius_mm));

    let (mut best_center, mut best_r) = match (best_medial, fallback) {
        (Some(m), Some(f)) => {
            if m.clearance_radius_mm >= f.max_helix_radius_mm {
                (m.pos_wcs, m.clearance_radius_mm)
            } else {
                (f.center_xy, f.max_helix_radius_mm)
            }
        }
        (Some(m), None) => (m.pos_wcs, m.clearance_radius_mm),
        (None, Some(f)) => return Some(f),
        (None, None) => return None,
    };

    // Fine sub-micron pattern polish on exact analytic `ArcPolyline2D` boundary distances
    // (removing any tiny chord-discretization error on curved boundaries!)
    let mut step = (0.15 * best_r).clamp(0.005, 0.25);
    for _ in 0..8 {
        let mut improved = false;
        let d = std::f64::consts::FRAC_1_SQRT_2;
        for (dx, dy) in [
            (1.0, 0.0),
            (-1.0, 0.0),
            (0.0, 1.0),
            (0.0, -1.0),
            (d, d),
            (-d, d),
            (d, -d),
            (-d, -d),
        ] {
            let cand = best_center + DVec2::new(dx * step, dy * step);
            if tool_center_region.contains_interior_point(cand) {
                let clr = tool_center_region.min_boundary_distance(cand);
                if clr > best_r {
                    best_r = clr;
                    best_center = cand;
                    improved = true;
                }
            }
        }
        if !improved {
            step *= 0.5;
        }
    }

    Some(HelicalEntryPole {
        center_xy: best_center,
        max_helix_radius_mm: best_r.max(0.0),
    })
}
