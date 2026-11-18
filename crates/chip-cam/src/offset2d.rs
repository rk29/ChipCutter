use cavalier_contours::{
    core::math::Vector2,
    polyline::{
        BooleanOp, PlineBooleanOptions, PlineOffsetOptions, PlineOrientation, PlineSource,
        PlineSourceMut, PlineVertex, Polyline, seg_arc_radius_and_center,
    },
    shape_algorithms::{Shape, ShapeOffsetOptions},
};
use chip_math::{Aabb2, ArcPolyline2D, ArcSegment2D, DVec2};

/// Losslessly converts an [`ArcPolyline2D`] into a `cavalier_contours::Polyline<f64>` using
/// exact bulge factors `b = tan(Δθ / 4)` (splitting arcs `> 170°` so `|b| < 1`).
pub fn to_cavalier_polyline(poly: &ArcPolyline2D) -> Polyline<f64> {
    let mut normalized: Vec<ArcSegment2D> = Vec::with_capacity(poly.segments.len() + 4);
    for seg in &poly.segments {
        if seg.length() <= 1e-6 {
            continue;
        }
        if matches!(seg, ArcSegment2D::Arc { .. }) && seg.signed_sweep_rad().abs() > 2.95 {
            let (h0, h1) = seg.split_at(0.5);
            if let Some(s0) = h0 {
                normalized.push(s0);
            }
            if let Some(s1) = h1 {
                normalized.push(s1);
            }
        } else {
            normalized.push(*seg);
        }
    }

    let mut out = if poly.closed {
        Polyline::new_closed()
    } else {
        Polyline::new()
    };

    if normalized.is_empty() {
        return out;
    }

    for seg in &normalized {
        let s = seg.start();
        let bulge = match *seg {
            ArcSegment2D::Line { .. } => 0.0,
            ArcSegment2D::Arc { .. } => (0.25 * seg.signed_sweep_rad()).tan(),
        };
        if s.is_finite() && bulge.is_finite() {
            out.add(s.x, s.y, bulge);
        }
    }

    if !poly.closed
        && let Some(last) = normalized.last()
    {
        let e = last.end();
        if e.is_finite() {
            out.add(e.x, e.y, 0.0);
        }
    }

    if let Some(cleaned) = out.remove_repeat_pos(1e-5) {
        cleaned
    } else {
        out
    }
}

/// Losslessly converts a `cavalier_contours::Polyline<f64>` back into an exact [`ArcPolyline2D`],
/// preserving circular arcs (`ArcSegment2D::Arc`) rather than discretizing into line segments.
pub fn from_cavalier_polyline(pline: &Polyline<f64>) -> ArcPolyline2D {
    let n = pline.vertex_count();
    let closed = pline.is_closed();
    if n < 2 {
        return ArcPolyline2D {
            segments: Vec::new(),
            closed,
        };
    }

    let seg_count = if closed { n } else { n - 1 };
    let mut segments = Vec::with_capacity(seg_count);

    for i in 0..seg_count {
        let j = (i + 1) % n;
        let Some(v1) = pline.get(i) else {
            continue;
        };
        let Some(v2) = pline.get(j) else {
            continue;
        };
        let p1 = DVec2::new(v1.x, v1.y);
        let p2 = DVec2::new(v2.x, v2.y);
        if !p1.is_finite() || !p2.is_finite() {
            continue;
        }
        let chord_len = p1.distance(p2);
        if chord_len <= 1e-5 {
            continue;
        }

        if v1.bulge_is_zero() || v1.bulge.abs() <= 1e-6 {
            segments.push(ArcSegment2D::Line { start: p1, end: p2 });
        } else {
            let (radius, c_vec) = seg_arc_radius_and_center(v1, v2);
            let center = DVec2::new(c_vec.x, c_vec.y);
            if radius.is_finite() && radius > 1e-5 && radius < 1e7 && center.is_finite() {
                segments.push(ArcSegment2D::Arc {
                    start: p1,
                    end: p2,
                    center,
                    radius,
                    ccw: v1.bulge > 0.0,
                });
            } else {
                segments.push(ArcSegment2D::Line { start: p1, end: p2 });
            }
        }
    }

    ArcPolyline2D { segments, closed }
}

/// Computes parallel offset polylines (`signed_left_offset_mm > 0` offsets to the LEFT of travel,
/// `< 0` offsets to the RIGHT of travel), handling self-intersections and preserving true arcs.
pub fn offset_polyline(poly: &ArcPolyline2D, signed_left_offset_mm: f64) -> Vec<ArcPolyline2D> {
    if !signed_left_offset_mm.is_finite() {
        return Vec::new();
    }
    if signed_left_offset_mm.abs() <= 1e-7 {
        return if poly.segments.is_empty() {
            Vec::new()
        } else {
            vec![poly.clone()]
        };
    }

    let pl = to_cavalier_polyline(poly);
    if pl.vertex_count() < 2 {
        return Vec::new();
    }

    let opts = PlineOffsetOptions {
        handle_self_intersects: true,
        pos_equal_eps: 1e-5,
        offset_dist_eps: 1e-4,
        ..Default::default()
    };

    pl.parallel_offset_opt(signed_left_offset_mm, &opts)
        .iter()
        .map(from_cavalier_polyline)
        .filter(|r| !r.segments.is_empty() && r.perimeter() > 1e-4)
        .collect()
}

/// Planar 2D region bounded by one `CCW` outer boundary and zero or more `CW` interior island holes.
#[derive(Debug, Clone, PartialEq)]
pub struct PocketRegion2D {
    pub outer_ccw: ArcPolyline2D,
    pub islands_cw: Vec<ArcPolyline2D>,
}

impl PocketRegion2D {
    pub fn new(outer: ArcPolyline2D, islands: Vec<ArcPolyline2D>) -> Self {
        let outer_ccw = outer.ensure_orientation(true);
        let islands_cw = islands
            .into_iter()
            .filter(|isl| isl.closed && !isl.segments.is_empty())
            .map(|isl| isl.ensure_orientation(false))
            .collect();
        Self {
            outer_ccw,
            islands_cw,
        }
    }

    /// Constructs a [`PocketRegion2D`] from a planar face boundary, expanding any open-air outer segments
    /// outward by `open_shift_mm` while keeping rising closed walls strictly untouched!
    pub fn from_face_with_open_extensions(
        outer: &ArcPolyline2D,
        open_flags: &[bool],
        raised_islands: &[ArcPolyline2D],
        open_shift_mm: f64,
    ) -> Self {
        let (mut outer_ccw, flags_ccw): (ArcPolyline2D, Vec<bool>) = if outer.signed_area() >= 0.0 {
            (outer.clone(), open_flags.to_vec())
        } else {
            let mut rev_flags = open_flags.to_vec();
            rev_flags.reverse();
            (outer.reverse(), rev_flags)
        };

        if open_shift_mm > 1e-4
            && flags_ccw.len() == outer_ccw.segments.len()
            && flags_ccw.iter().any(|&b| b)
        {
            outer_ccw = expand_open_segments_outward(&outer_ccw, &flags_ccw, open_shift_mm);
        }

        Self::new(outer_ccw, raised_islands.to_vec())
    }

    pub fn exact_aabb(&self) -> Aabb2 {
        self.outer_ccw.exact_aabb()
    }

    /// Exact point-in-pocket test (`inside outer_ccw` AND `strictly outside every island`).
    pub fn contains_interior_point(&self, pt: DVec2) -> bool {
        if !self.outer_ccw.contains_point(pt) {
            return false;
        }
        for isl in &self.islands_cw {
            if isl.contains_point(pt) && isl.distance_to_point(pt) > 1e-6 {
                return false;
            }
        }
        true
    }

    /// Minimum Euclidean distance from `pt` to any boundary curve (outer wall or island wall).
    pub fn min_boundary_distance(&self, pt: DVec2) -> f64 {
        let mut d = self.outer_ccw.distance_to_point(pt);
        for isl in &self.islands_cw {
            d = d.min(isl.distance_to_point(pt));
        }
        d
    }

    /// Dilates this region outward by `outward_dist_mm` (`outer` expands outward, `islands` shrink inward).
    pub fn dilate_outward(&self, outward_dist_mm: f64) -> Vec<Self> {
        if !outward_dist_mm.is_finite() {
            return Vec::new();
        }
        self.erode_inward(-outward_dist_mm)
    }

    /// Approximate net planar area (`mm²`) of `outer_ccw` minus all `islands_cw`.
    pub fn net_area_mm2(&self) -> f64 {
        let outer_a = self.outer_ccw.signed_area().abs();
        let isl_a: f64 = self.islands_cw.iter().map(|i| i.signed_area().abs()).sum();
        (outer_a - isl_a).max(0.0)
    }

    /// Erodes this region inward by `inward_dist_mm` (`outer` shrinks inward, `islands` expand outward),
    /// returning 0, 1, or multiple disconnected [`PocketRegion2D`] sub-regions.
    pub fn erode_inward(&self, inward_dist_mm: f64) -> Vec<Self> {
        if !inward_dist_mm.is_finite() {
            return Vec::new();
        }
        if inward_dist_mm.abs() <= 1e-7 {
            return vec![self.clone()];
        }

        let has_micro_arc = |p: &ArcPolyline2D| {
            p.segments
                .iter()
                .any(|s| matches!(s, ArcSegment2D::Arc { .. }) && s.signed_sweep_rad().abs() < 0.05)
        };
        if self.islands_cw.len() > 1
            || self.outer_ccw.segments.len() > 16
            || self.islands_cw.iter().any(|isl| isl.segments.len() > 16)
            || has_micro_arc(&self.outer_ccw)
            || self.islands_cw.iter().any(has_micro_arc)
        {
            return crate::rest2d::offset_single_region_shape(self, inward_dist_mm);
        }

        let mut plines = Vec::with_capacity(1 + self.islands_cw.len());
        let mut outer_pl = to_cavalier_polyline(&self.outer_ccw);
        if outer_pl.vertex_count() < 2 {
            return Vec::new();
        }
        if outer_pl.orientation() != PlineOrientation::CounterClockwise {
            outer_pl.invert_direction_mut();
        }
        plines.push(outer_pl);

        for isl in &self.islands_cw {
            let mut isl_pl = to_cavalier_polyline(isl);
            if isl_pl.vertex_count() >= 2 {
                if isl_pl.orientation() != PlineOrientation::Clockwise {
                    isl_pl.invert_direction_mut();
                }
                plines.push(isl_pl);
            }
        }

        let shape = Shape::from_plines(plines);
        let offset_shape = shape.parallel_offset(inward_dist_mm, &ShapeOffsetOptions::default());

        let outers: Vec<ArcPolyline2D> = offset_shape
            .ccw_plines
            .iter()
            .map(|ip| from_cavalier_polyline(&ip.polyline).ensure_orientation(true))
            .filter(|p| p.closed && p.segments.len() >= 2 && p.signed_area() > 1e-5)
            .collect();

        if outers.is_empty() {
            return Vec::new();
        }

        let holes: Vec<ArcPolyline2D> = offset_shape
            .cw_plines
            .iter()
            .map(|ip| from_cavalier_polyline(&ip.polyline).ensure_orientation(false))
            .filter(|p| p.closed && p.segments.len() >= 2 && p.signed_area().abs() > 1e-5)
            .collect();

        let mut regions: Vec<Self> = outers
            .into_iter()
            .map(|o| Self {
                outer_ccw: o,
                islands_cw: Vec::new(),
            })
            .collect();

        for hole in holes {
            if let Some((sample_pt, _)) = hole.sample_at_fraction(0.13) {
                let mut best_idx: Option<(usize, f64)> = None;
                for (idx, reg) in regions.iter().enumerate() {
                    if reg.outer_ccw.contains_point(sample_pt) {
                        let area = reg.outer_ccw.signed_area().abs();
                        if best_idx.is_none_or(|(_, prev_a)| area < prev_a) {
                            best_idx = Some((idx, area));
                        }
                    }
                }
                if let Some((idx, _)) = best_idx {
                    regions[idx].islands_cw.push(hole);
                }
            }
        }

        regions
    }
}

/// Node in the **Offset Region Tree** (`05-operations.md` §3):
/// When an inward pocket stepover splits around an island or narrow neck into disconnected
/// sub-pockets, `children` holds each independent branch so every sub-pocket clears completely
/// before retracting to its sibling!
#[derive(Debug, Clone)]
pub struct OffsetRegionNode {
    pub region: PocketRegion2D,
    pub depth_index: usize,
    pub children: Vec<OffsetRegionNode>,
}

/// Recursively builds the inward Offset Region Tree starting from `outermost_tool_region`.
pub fn build_offset_region_tree(
    outermost_tool_region: &PocketRegion2D,
    stepover_mm: f64,
    max_depth: usize,
) -> OffsetRegionNode {
    let step = stepover_mm.max(0.05);
    build_node_recursive(outermost_tool_region.clone(), step, 0, max_depth.min(512))
}

fn build_node_recursive(
    region: PocketRegion2D,
    stepover_mm: f64,
    depth_index: usize,
    max_depth: usize,
) -> OffsetRegionNode {
    if depth_index >= max_depth {
        return OffsetRegionNode {
            region,
            depth_index,
            children: Vec::new(),
        };
    }
    let next_regions = region.erode_inward(stepover_mm);
    let children = next_regions
        .into_iter()
        .map(|child_reg| build_node_recursive(child_reg, stepover_mm, depth_index + 1, max_depth))
        .collect();
    OffsetRegionNode {
        region,
        depth_index,
        children,
    }
}

/// Optimal collision-free center point and clearance radius inside a tool-center [`PocketRegion2D`]
/// for helical ramp entry (`05-operations.md` §3).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HelicalEntryPole {
    pub center_xy: DVec2,
    /// Maximum distance (`mm`) the tool center can orbit around `center_xy` while staying strictly
    /// inside `tool_center_region` (i.e. maximum safe helix path radius `R_helix`).
    pub max_helix_radius_mm: f64,
}

/// Locates the largest inscribed circle pole inside `tool_center_region` via coarse-to-fine
/// inward erosion + interior grid refinement.
pub fn find_helical_entry_pole(tool_center_region: &PocketRegion2D) -> Option<HelicalEntryPole> {
    let aabb = tool_center_region.exact_aabb();
    if !aabb.is_valid() {
        return None;
    }
    let size = aabb.size();
    let max_possible_r = 0.5 * size.x.min(size.y);

    let mut best_pt: Option<DVec2> = None;
    let mut best_r = -1.0_f64;

    let consider = |pt: DVec2, best_r: &mut f64, best_pt: &mut Option<DVec2>| {
        if tool_center_region.contains_interior_point(pt) {
            let clr = tool_center_region.min_boundary_distance(pt);
            if clr > *best_r {
                *best_r = clr;
                *best_pt = Some(pt);
            }
        }
    };

    // 1. Sample 14 x 14 grid across the region's bounding box
    let nx = 14usize;
    let ny = 14usize;
    for ix in 0..=nx {
        let fx = (f64::from(ix as u32) + 0.5) / f64::from((nx + 1) as u32);
        let x = aabb.min.x + size.x * fx;
        for iy in 0..=ny {
            let fy = (f64::from(iy as u32) + 0.5) / f64::from((ny + 1) as u32);
            let y = aabb.min.y + size.y * fy;
            consider(DVec2::new(x, y), &mut best_r, &mut best_pt);
        }
    }

    // 2. Also probe deepest inward-eroded ring centroids/vertices via binary search on erosion depth
    if max_possible_r > 1e-3 {
        let mut lo = 0.0_f64;
        let mut hi = max_possible_r;
        for _ in 0..10 {
            let mid = 0.5 * (lo + hi);
            let sub = tool_center_region.erode_inward(mid);
            if let Some(first) = sub.first() {
                lo = mid;
                consider(first.exact_aabb().center(), &mut best_r, &mut best_pt);
                if let Some((p, _)) = first.outer_ccw.sample_at_fraction(0.0) {
                    consider(p, &mut best_r, &mut best_pt);
                }
                if let Some((p, _)) = first.outer_ccw.sample_at_fraction(0.5) {
                    consider(p, &mut best_r, &mut best_pt);
                }
            } else {
                hi = mid;
            }
        }
    }

    // 3. Coordinate-pattern refinement around best candidate
    if let Some(mut center) = best_pt {
        let mut step = 0.12 * size.x.min(size.y).max(0.2);
        for _ in 0..8 {
            for (dx, dy) in [
                (1.0, 0.0),
                (-1.0, 0.0),
                (0.0, 1.0),
                (0.0, -1.0),
                (0.707, 0.707),
                (-0.707, 0.707),
                (0.707, -0.707),
                (-0.707, -0.707),
            ] {
                consider(
                    center + DVec2::new(dx * step, dy * step),
                    &mut best_r,
                    &mut best_pt,
                );
            }
            if let Some(updated) = best_pt {
                center = updated;
            }
            step *= 0.5;
        }
    }

    best_pt.map(|center_xy| HelicalEntryPole {
        center_xy,
        max_helix_radius_mm: best_r.max(0.0),
    })
}

/// Clips an arbitrary 2D line segment `p0 -> p1` against `region` (`outer_ccw` minus `islands_cw`),
/// returning ordered interior sub-segments `(a, b)` along the direction `p0 -> p1`.
pub fn clip_scanline_to_region(
    p0: DVec2,
    p1: DVec2,
    region: &PocketRegion2D,
) -> Vec<(DVec2, DVec2)> {
    let dir = p1 - p0;
    let len = dir.length();
    if len <= 1e-7 {
        return Vec::new();
    }

    let mut ts = vec![0.0_f64, 1.0_f64];
    collect_polyline_line_intersections(p0, p1, &region.outer_ccw, &mut ts);
    for isl in &region.islands_cw {
        collect_polyline_line_intersections(p0, p1, isl, &mut ts);
    }

    ts.sort_by(f64::total_cmp);
    ts.dedup_by(|a, b| (*a - *b).abs() <= 1e-7);

    let mut spans = Vec::new();
    for w in ts.windows(2) {
        let t0 = w[0];
        let t1 = w[1];
        if (t1 - t0) * len <= 1e-4 {
            continue;
        }
        let mid = p0 + dir * (0.5 * (t0 + t1));
        if region.contains_interior_point(mid) {
            spans.push((p0 + dir * t0, p0 + dir * t1));
        }
    }
    spans
}

fn collect_polyline_line_intersections(
    p0: DVec2,
    p1: DVec2,
    poly: &ArcPolyline2D,
    ts_out: &mut Vec<f64>,
) {
    let d = p1 - p0;
    for seg in &poly.segments {
        match *seg {
            ArcSegment2D::Line { start, end } => {
                let s = end - start;
                let denom = d.x * s.y - d.y * s.x;
                if denom.abs() > 1e-12 {
                    let diff = start - p0;
                    let t = (diff.x * s.y - diff.y * s.x) / denom;
                    let u = (diff.x * d.y - diff.y * d.x) / denom;
                    if (-1e-7..=1.0 + 1e-7).contains(&t) && (-1e-7..=1.0 + 1e-7).contains(&u) {
                        ts_out.push(t.clamp(0.0, 1.0));
                    }
                }
            }
            ArcSegment2D::Arc {
                start,
                center,
                radius,
                ccw,
                ..
            } => {
                let f = p0 - center;
                let a = d.length_squared();
                let b = 2.0 * f.dot(d);
                let c = f.length_squared() - radius * radius;
                let disc = b * b - 4.0 * a * c;
                if disc >= 0.0 && a > 1e-12 {
                    let sqrt_d = disc.sqrt();
                    let a0 = (start.y - center.y).atan2(start.x - center.x);
                    let sweep_mag = seg.signed_sweep_rad().abs();
                    for sign in [-1.0, 1.0] {
                        let t = (-b + sign * sqrt_d) / (2.0 * a);
                        if (-1e-7..=1.0 + 1e-7).contains(&t) {
                            let hit = p0 + d * t;
                            let ah = (hit.y - center.y).atan2(hit.x - center.x);
                            let delta = if ccw {
                                (ah - a0).rem_euclid(std::f64::consts::TAU)
                            } else {
                                (a0 - ah).rem_euclid(std::f64::consts::TAU)
                            };
                            if delta <= sweep_mag + 1e-5 {
                                ts_out.push(t.clamp(0.0, 1.0));
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Expands open-air segments of a `CCW` outer loop outward by `shift_mm` using boolean union of
/// outward buffer bands aligned with adjacent closed-wall tangents so rising walls are never crossed.
fn expand_open_segments_outward(
    outer_ccw: &ArcPolyline2D,
    open_flags: &[bool],
    shift_mm: f64,
) -> ArcPolyline2D {
    let n = outer_ccw.segments.len();
    if n == 0 {
        return outer_ccw.clone();
    }

    let mut accum_pline = to_cavalier_polyline(outer_ccw);
    if accum_pline.orientation() != PlineOrientation::CounterClockwise {
        accum_pline.invert_direction_mut();
    }

    let bool_opts = PlineBooleanOptions {
        pos_equal_eps: 1e-5,
        ..Default::default()
    };

    for i in 0..n {
        if !open_flags[i] {
            continue;
        }
        let prev_i = (i + n - 1) % n;
        let next_i = (i + 1) % n;
        let seg = outer_ccw.segments[i];
        let Some((_, t_start)) = seg.eval_at(0.0) else {
            continue;
        };
        let Some((_, t_end)) = seg.eval_at(1.0) else {
            continue;
        };

        // Outward normal of a CCW loop points to the RIGHT of travel: `(t.y, -t.x)`
        let out_n_start = DVec2::new(t_start.y, -t_start.x);
        let out_n_end = DVec2::new(t_end.y, -t_end.x);

        // When an open segment meets a closed wall at its start/end vertex, slide along the closed wall's
        // tangent ray rather than crossing behind the closed wall!
        let dir_start = if !open_flags[prev_i]
            && let Some((_, prev_t_end)) = outer_ccw.segments[prev_i].eval_at(1.0)
        {
            let ray = prev_t_end.normalize_or_zero();
            let proj = ray.dot(out_n_start);
            if proj > 0.25 {
                ray * (shift_mm / proj).min(shift_mm * 2.5)
            } else {
                out_n_start * shift_mm
            }
        } else {
            out_n_start * shift_mm
        };

        let dir_end = if !open_flags[next_i]
            && let Some((_, next_t_start)) = outer_ccw.segments[next_i].eval_at(0.0)
        {
            let ray = (-next_t_start).normalize_or_zero();
            let proj = ray.dot(out_n_end);
            if proj > 0.25 {
                ray * (shift_mm / proj).min(shift_mm * 2.5)
            } else {
                out_n_end * shift_mm
            }
        } else {
            out_n_end * shift_mm
        };

        // Build a CCW outward strip quad/arc patch for segment `i` (inset slightly into the pocket
        // by `0.05 mm` so `BooleanOp::Or` overlaps cleanly rather than touching along a hairline edge!)
        let p0 = seg.start();
        let p1 = seg.end();
        let in0 = p0 - out_n_start * 0.05;
        let in1 = p1 - out_n_end * 0.05;
        let out1 = p1 + dir_end;
        let out0 = p0 + dir_start;

        let mut patch = Polyline::new_closed();
        patch.add_vertex(PlineVertex::new(in0.x, in0.y, 0.0));
        patch.add_vertex(PlineVertex::new(out0.x, out0.y, 0.0));
        patch.add_vertex(PlineVertex::new(out1.x, out1.y, 0.0));
        patch.add_vertex(PlineVertex::new(in1.x, in1.y, 0.0));
        if patch.area() < 0.0 {
            patch.invert_direction_mut();
        }
        if patch.area() > 1e-4 {
            let union_res = accum_pline.boolean_opt(&patch, BooleanOp::Or, &bool_opts);
            if let Some(best) = union_res
                .pos_plines
                .into_iter()
                .map(|bp| bp.pline)
                .max_by(|a, b| a.area().abs().total_cmp(&b.area().abs()))
            {
                accum_pline = best;
            }
        }

        // If two consecutive segments `i` and `next_i` are BOTH open, add an outward corner fan/box at `p1`
        // so convex open corners expand seamlessly without leaving a notch!
        if open_flags[next_i]
            && let Some((_, next_t_start)) = outer_ccw.segments[next_i].eval_at(0.0)
        {
            let next_out_n = DVec2::new(next_t_start.y, -next_t_start.x);
            let corner_diag = (out_n_end + next_out_n).normalize_or_zero();
            if corner_diag.length_squared() > 0.2 {
                let c_in = p1 - corner_diag * 0.05;
                let c_a = p1 + out_n_end * shift_mm;
                let c_mid = p1 + corner_diag * (shift_mm * 1.42);
                let c_b = p1 + next_out_n * shift_mm;
                let mut cap = Polyline::new_closed();
                for v in [
                    Vector2::new(c_in.x, c_in.y),
                    Vector2::new(c_a.x, c_a.y),
                    Vector2::new(c_mid.x, c_mid.y),
                    Vector2::new(c_b.x, c_b.y),
                ] {
                    cap.add(v.x, v.y, 0.0);
                }
                if cap.area() < 0.0 {
                    cap.invert_direction_mut();
                }
                if cap.area() > 1e-4 {
                    let union_res = accum_pline.boolean_opt(&cap, BooleanOp::Or, &bool_opts);
                    if let Some(best) = union_res
                        .pos_plines
                        .into_iter()
                        .map(|bp| bp.pline)
                        .max_by(|a, b| a.area().abs().total_cmp(&b.area().abs()))
                    {
                        accum_pline = best;
                    }
                }
            }
        }
    }

    from_cavalier_polyline(&accum_pline).ensure_orientation(true)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::approx_eq;

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

    fn circle_ccw(center: DVec2, radius: f64) -> ArcPolyline2D {
        let p_r = center + DVec2::new(radius, 0.0);
        let p_l = center - DVec2::new(radius, 0.0);
        ArcPolyline2D {
            segments: vec![
                ArcSegment2D::Arc {
                    start: p_r,
                    end: p_l,
                    center,
                    radius,
                    ccw: true,
                },
                ArcSegment2D::Arc {
                    start: p_l,
                    end: p_r,
                    center,
                    radius,
                    ccw: true,
                },
            ],
            closed: true,
        }
    }

    #[test]
    fn preserves_true_circular_arcs_across_cavalier_offsetting() {
        let c = DVec2::new(10.0, 15.0);
        let circle = circle_ccw(c, 20.0);
        let inward = offset_polyline(&circle, 3.0);
        assert_eq!(inward.len(), 1);
        let aabb = inward[0].exact_aabb();
        assert!(approx_eq(aabb.size().x, 34.0, 1e-4));
        assert!(approx_eq(aabb.size().y, 34.0, 1e-4));
        // Every segment must remain an exact analytic circular arc (zero micro-segment faceting!)
        assert!(
            inward[0]
                .segments
                .iter()
                .all(|s| matches!(s, ArcSegment2D::Arc { .. }))
        );
    }

    #[test]
    fn erodes_pocket_with_island_splits_region_tree_and_expands_open_step() {
        // 1. Pocket [0..40] x [0..24] with a central Ø14mm (R=7) island at (20, 12):
        //    Gap at top/bottom between island (Y=5..19) and walls (Y=0, 24) is only 5 mm!
        //    So eroding by 3.0 mm pinches the top & bottom gaps closed and splits into LEFT and RIGHT sub-pockets!
        let outer = rect_ccw(0.0, 0.0, 40.0, 24.0);
        let island = circle_ccw(DVec2::new(20.0, 12.0), 7.0);
        let pkt = PocketRegion2D::new(outer, vec![island]);

        let split_subs = pkt.erode_inward(3.0);
        assert_eq!(
            split_subs.len(),
            2,
            "narrow gap around central island must split into 2 disconnected sub-pockets"
        );
        for sub in &split_subs {
            let pole = find_helical_entry_pole(sub).unwrap();
            assert!(sub.contains_interior_point(pole.center_xy));
            assert!(pole.max_helix_radius_mm > 2.5);
        }

        // 2. Open-edge step [76..100] x [0..80] where segs [0] (Y=0), [1] (X=100), [2] (Y=80) are open
        //    and seg [3] (X=76) is a rising wall:
        let step_outer = rect_ccw(76.0, 0.0, 100.0, 80.0);
        let open_flags = [true, true, true, false];
        // Shift open edges out by tool_r (3.0) + overhang (1.0) = 4.0, then erode by tool_r (3.0):
        let expanded =
            PocketRegion2D::from_face_with_open_extensions(&step_outer, &open_flags, &[], 4.0);
        let tool_regions = expanded.erode_inward(3.0);
        assert_eq!(tool_regions.len(), 1);
        let tr_aabb = tool_regions[0].exact_aabb();
        // Rising wall at X=76 MUST keep exact X >= 79.0 clearance, while open edges extend 1.0 mm past stock!
        assert!(approx_eq(tr_aabb.min.x, 79.0, 2e-2));
        assert!(tr_aabb.max.x > 100.8);
        assert!(tr_aabb.min.y < -0.8);
        assert!(tr_aabb.max.y > 80.8);
    }
}
