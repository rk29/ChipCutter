//! Exact 2D Boolean Region Sets ([`RegionSet2D`]), swept segment capsules, polyline clipping, and
//! even-odd contour nesting.

use cavalier_contours::polyline::{
    BooleanOp, PlineBooleanOptions, PlineOrientation, PlineSource, PlineSourceMut, Polyline,
};
use chip_math::{Aabb2, ArcPolyline2D, ArcSegment2D, DVec2};

use crate::offset::{
    PocketRegion2D, from_cavalier_polyline, offset_polyline, to_cavalier_polyline,
};

/// Disjoint collection of planar [`PocketRegion2D`] polygons supporting exact arc-preserving
/// `union`, `difference`, `intersect`, `dilate_outward`, and `erode_inward` operations.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RegionSet2D {
    pub regions: Vec<PocketRegion2D>,
}

impl RegionSet2D {
    pub fn empty() -> Self {
        Self {
            regions: Vec::new(),
        }
    }

    pub fn from_single(region: PocketRegion2D) -> Self {
        if region.outer_ccw.segments.len() < 2 || region.net_area_mm2() <= 1e-5 {
            Self::empty()
        } else {
            Self {
                regions: vec![region],
            }
        }
    }

    pub fn from_regions(regions: Vec<PocketRegion2D>) -> Self {
        let mut iter = regions
            .into_iter()
            .filter(|r| r.outer_ccw.segments.len() >= 2 && r.net_area_mm2() > 1e-5)
            .map(Self::from_single)
            .collect::<Vec<_>>();
        Self::union_many(&mut iter)
    }

    /// Constructs a circular disk region centered at `center` with `radius_mm`.
    pub fn from_circle(center: DVec2, radius_mm: f64) -> Self {
        if !center.is_finite() || !radius_mm.is_finite() || radius_mm <= 1e-5 {
            return Self::empty();
        }
        Self::from_single(PocketRegion2D::new(
            ArcPolyline2D::circle(center, radius_mm),
            Vec::new(),
        ))
    }

    /// Exact combined 2D axis-aligned bounding box of all outer contours in this region set.
    pub fn exact_aabb(&self) -> Option<Aabb2> {
        let mut acc = Aabb2::EMPTY;
        for r in &self.regions {
            let b = r.exact_aabb();
            if b.is_valid() {
                acc = acc.union(&b);
            }
        }
        acc.is_valid().then_some(acc)
    }

    pub fn is_empty(&self) -> bool {
        self.regions.is_empty()
    }

    pub fn total_area_mm2(&self) -> f64 {
        self.regions.iter().map(PocketRegion2D::net_area_mm2).sum()
    }

    pub fn contains_point(&self, pt: DVec2) -> bool {
        self.regions.iter().any(|r| r.contains_interior_point(pt))
    }

    pub fn min_distance_to_boundary(&self, pt: DVec2) -> f64 {
        self.regions
            .iter()
            .map(|r| r.min_boundary_distance(pt))
            .fold(f64::INFINITY, f64::min)
    }

    /// Erodes every region inward by `inward_dist_mm` (`outer` shrinks, `islands` expand).
    pub fn erode_inward(&self, inward_dist_mm: f64) -> Self {
        if !inward_dist_mm.is_finite() || self.is_empty() {
            return Self::empty();
        }
        if inward_dist_mm.abs() <= 1e-7 {
            return self.clone();
        }
        if inward_dist_mm < 0.0 {
            return self.dilate_outward(-inward_dist_mm);
        }
        let mut out = Vec::new();
        for reg in &self.regions {
            out.extend(reg.erode_inward(inward_dist_mm));
        }
        Self { regions: out }
    }

    /// Dilates every region outward by `outward_dist_mm` and unions any newly touching components.
    pub fn dilate_outward(&self, outward_dist_mm: f64) -> Self {
        if !outward_dist_mm.is_finite() || self.is_empty() {
            return Self::empty();
        }
        if outward_dist_mm.abs() <= 1e-7 {
            return self.clone();
        }
        if outward_dist_mm < 0.0 {
            return self.erode_inward(-outward_dist_mm);
        }
        let mut sets: Vec<Self> = self
            .regions
            .iter()
            .map(|reg| Self {
                regions: offset_single_region_shape(reg, -outward_dist_mm),
            })
            .collect();
        Self::union_many(&mut sets)
    }

    /// Computes the exact boolean union `self ∪ other`.
    pub fn union(&self, other: &Self) -> Self {
        if self.is_empty() {
            return other.clone();
        }
        if other.is_empty() {
            return self.clone();
        }
        let mut acc = self.clone();
        for reg in &other.regions {
            acc = acc.union_region(reg);
        }
        acc
    }

    /// Computes the exact boolean difference `self \ clip`.
    pub fn difference(&self, clip: &Self) -> Self {
        if self.is_empty() || clip.is_empty() {
            return self.clone();
        }
        let mut acc = self.clone();
        for q in &clip.regions {
            // De Morgan identity for region with islands `Q = O_Q \ (H_1 ∪ ... ∪ H_k)`:
            // `S \ Q = (S \ O_Q) ∪ (S ∩ H_1) ∪ ... ∪ (S ∩ H_k)`
            let q_aabb = q.exact_aabb();
            if !acc.regions.iter().any(|r| r.exact_aabb().overlaps(&q_aabb)) {
                continue;
            }
            let outer_ccw = to_ccw_pline(&q.outer_ccw);
            if outer_ccw.vertex_count() < 2 {
                continue;
            }
            let mut next = subtract_simple_ccw_from_set(&acc, &outer_ccw, &q_aabb);
            for hole_cw in &q.islands_cw {
                let hole_ccw = to_ccw_pline(hole_cw);
                let hole_aabb = hole_cw.exact_aabb();
                if hole_ccw.vertex_count() >= 2 {
                    let preserved = intersect_set_with_simple_ccw(&acc, &hole_ccw, &hole_aabb);
                    // `(acc \ O_Q)` lies outside `O_Q` while `(acc ∩ H_k)` lies strictly inside
                    // `H_k ⊂ int(O_Q)` (and `H_k` are pairwise disjoint), so their closures are
                    // strictly disjoint—appending directly avoids nested-island recursion!
                    next.regions.extend(preserved.regions);
                }
            }
            acc = next;
            if acc.is_empty() {
                break;
            }
        }
        acc
    }

    /// Computes the exact boolean intersection `self ∩ mask`.
    pub fn intersect(&self, mask: &Self) -> Self {
        if self.is_empty() || mask.is_empty() {
            return Self::empty();
        }
        let mut pieces = Vec::new();
        for m in &mask.regions {
            let m_aabb = m.exact_aabb();
            let outer_ccw = to_ccw_pline(&m.outer_ccw);
            if outer_ccw.vertex_count() < 2 {
                continue;
            }
            let mut sub = intersect_set_with_simple_ccw(self, &outer_ccw, &m_aabb);
            for hole_cw in &m.islands_cw {
                let hole_ccw = to_ccw_pline(hole_cw);
                let hole_aabb = hole_cw.exact_aabb();
                if hole_ccw.vertex_count() >= 2 {
                    sub = subtract_simple_ccw_from_set(&sub, &hole_ccw, &hole_aabb);
                }
            }
            if !sub.is_empty() {
                pieces.push(sub);
            }
        }
        Self::union_many(&mut pieces)
    }

    /// Pairwise divide-and-conquer union in $O(N \log N)$ boolean passes.
    pub fn union_many(sets: &mut Vec<Self>) -> Self {
        sets.retain(|s| !s.is_empty());
        if sets.is_empty() {
            return Self::empty();
        }
        while sets.len() > 1 {
            let mut next_wave = Vec::with_capacity(sets.len().div_ceil(2));
            let drained = std::mem::take(sets);
            let mut iter = drained.into_iter();
            while let Some(a) = iter.next() {
                if let Some(b) = iter.next() {
                    next_wave.push(a.union(&b));
                } else {
                    next_wave.push(a);
                }
            }
            *sets = next_wave;
        }
        sets.pop().unwrap_or_else(Self::empty)
    }

    fn union_region(&self, reg: &PocketRegion2D) -> Self {
        if self.is_empty() {
            return Self::from_single(reg.clone());
        }
        let r_aabb = reg.exact_aabb();
        let mut untouched = Vec::new();
        let mut overlapping = Vec::new();
        for existing in &self.regions {
            if existing.exact_aabb().overlaps(&r_aabb) {
                overlapping.push(existing.clone());
            } else {
                untouched.push(existing.clone());
            }
        }
        if overlapping.is_empty() {
            untouched.push(reg.clone());
            return Self { regions: untouched };
        }

        // Union all outer CCW loops `U = (∪ O_A) ∪ O_R`, then carve remaining holes that lie
        // outside both `A` and `R`.
        let mut outer_plines: Vec<Polyline<f64>> = overlapping
            .iter()
            .map(|r| to_ccw_pline(&r.outer_ccw))
            .collect();
        outer_plines.push(to_ccw_pline(&reg.outer_ccw));

        let (merged_outers, courtyard_holes) = union_simple_ccw_loops(outer_plines);
        let mut merged_set = assemble_regions_from_outers_and_holes(merged_outers, courtyard_holes);

        for (i, a_reg) in overlapping.iter().enumerate() {
            for isl_cw in &a_reg.islands_cw {
                let mut hole_rem = Self::from_single(PocketRegion2D::new(
                    isl_cw.ensure_orientation(true),
                    Vec::new(),
                ));
                hole_rem = hole_rem.difference(&Self::from_single(reg.clone()));
                for (j, other_a) in overlapping.iter().enumerate() {
                    if i != j && !hole_rem.is_empty() {
                        hole_rem = hole_rem.difference(&Self::from_single(other_a.clone()));
                    }
                }
                if !hole_rem.is_empty() {
                    merged_set = merged_set.difference(&hole_rem);
                }
            }
        }
        for isl_cw in &reg.islands_cw {
            let mut hole_rem = Self::from_single(PocketRegion2D::new(
                isl_cw.ensure_orientation(true),
                Vec::new(),
            ));
            for a_reg in &overlapping {
                if !hole_rem.is_empty() {
                    hole_rem = hole_rem.difference(&Self::from_single(a_reg.clone()));
                }
            }
            if !hole_rem.is_empty() {
                merged_set = merged_set.difference(&hole_rem);
            }
        }

        untouched.extend(merged_set.regions);
        Self { regions: untouched }
    }
}

fn scale_pline(src: &Polyline<f64>, s: f64) -> Polyline<f64> {
    let mut dst = if src.is_closed() {
        Polyline::new_closed()
    } else {
        Polyline::new()
    };
    for v in &src.vertex_data {
        dst.add(v.x * s, v.y * s, v.bulge);
    }
    dst
}

fn clean_pline_for_fallback(src: &Polyline<f64>) -> Polyline<f64> {
    let mut clean = src.clone();
    for v in &mut clean.vertex_data {
        if v.bulge.abs() < 1e-5 {
            v.bulge = 0.0;
        }
    }
    clean.remove_redundant(1e-4).unwrap_or(clean)
}

/// Panic-safe, power-of-two scaled wrapper around `cavalier_contours::Polyline::boolean_opt`.
///
/// Mirrors `offset_polyline`'s bit-exact `WORK_SCALE` strategy (`16.0`) so unnormalized
/// `perp_dot` predicates in `cavalier_contours` don't misclassify short near-collinear segments
/// and debug-mode `PlineViewData::validate_for_source` assertions never panic on curved/helical slices.
fn safe_pline_boolean(
    a: &Polyline<f64>,
    b: &Polyline<f64>,
    op: BooleanOp,
) -> (Vec<Polyline<f64>>, Vec<Polyline<f64>>) {
    const WORK_SCALE: f64 = 16.0;
    const INV_WORK_SCALE: f64 = 1.0 / WORK_SCALE;

    let run_scaled = |pa: &Polyline<f64>, pb: &Polyline<f64>, scale: f64, eps: f64| {
        let sa = scale_pline(pa, scale);
        let sb = scale_pline(pb, scale);
        let opts = PlineBooleanOptions {
            pos_equal_eps: eps,
            ..Default::default()
        };
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            sa.boolean_opt(&sb, op, &opts)
        }))
        .ok()
        .map(|res| {
            let inv = 1.0 / scale;
            let pos = res
                .pos_plines
                .into_iter()
                .map(|bp| scale_pline(&bp.pline, inv))
                .collect();
            let neg = res
                .neg_plines
                .into_iter()
                .map(|bp| scale_pline(&bp.pline, inv))
                .collect();
            (pos, neg)
        })
    };

    // Fast path: bit-exact 16x working scale
    if let Some(res) = run_scaled(a, b, WORK_SCALE, 1e-5) {
        return res;
    }

    // Fallback: snap micro-bulges, remove redundant vertices, and retry across power-of-two scales
    let ca = clean_pline_for_fallback(a);
    let cb = clean_pline_for_fallback(b);
    for (scale, eps) in [(256.0, 1e-5), (1024.0, 1e-5), (1.0, 1e-4)] {
        if let Some(res) = run_scaled(&ca, &cb, scale, eps) {
            return res;
        }
    }
    let _ = INV_WORK_SCALE;

    // Conservative last-resort fallback if every boolean scale failed
    match op {
        BooleanOp::Or => (vec![a.clone(), b.clone()], Vec::new()),
        BooleanOp::Not => (vec![a.clone()], Vec::new()),
        BooleanOp::And | BooleanOp::Xor => (Vec::new(), Vec::new()),
    }
}

fn to_ccw_pline(poly: &ArcPolyline2D) -> Polyline<f64> {
    let mut pl = to_cavalier_polyline(&poly.ensure_orientation(true));
    if pl.vertex_count() >= 2 && pl.orientation() != PlineOrientation::CounterClockwise {
        pl.invert_direction_mut();
    }
    pl
}

pub(crate) fn offset_single_region_shape(
    reg: &PocketRegion2D,
    signed_inward_mm: f64,
) -> Vec<PocketRegion2D> {
    if !signed_inward_mm.is_finite() {
        return Vec::new();
    }
    if signed_inward_mm.abs() <= 1e-7 {
        return vec![reg.clone()];
    }

    let outer_loops = offset_polyline(&reg.outer_ccw.ensure_orientation(true), signed_inward_mm);
    let mut outers = Vec::new();
    let mut holes = Vec::new();
    for lp in outer_loops {
        if !lp.closed || lp.segments.len() < 2 {
            continue;
        }
        let area = lp.signed_area();
        if area > 1e-5 {
            outers.push(lp.ensure_orientation(true));
        } else if area < -1e-5 {
            holes.push(lp.ensure_orientation(false));
        }
    }

    let mut set = RegionSet2D {
        regions: assign_holes_to_outers(outers, holes),
    };
    if set.is_empty() || reg.islands_cw.is_empty() {
        return set.regions;
    }

    let mut hole_sets: Vec<RegionSet2D> = Vec::with_capacity(reg.islands_cw.len());
    for isl in &reg.islands_cw {
        let isl_loops = offset_polyline(&isl.ensure_orientation(true), -signed_inward_mm);
        let mut i_outers = Vec::new();
        let mut i_holes = Vec::new();
        for lp in isl_loops {
            if !lp.closed || lp.segments.len() < 2 {
                continue;
            }
            let area = lp.signed_area();
            if area > 1e-5 {
                i_outers.push(lp.ensure_orientation(true));
            } else if area < -1e-5 {
                i_holes.push(lp.ensure_orientation(false));
            }
        }
        let isl_regs = assign_holes_to_outers(i_outers, i_holes);
        if !isl_regs.is_empty() {
            hole_sets.push(RegionSet2D { regions: isl_regs });
        }
    }

    if !hole_sets.is_empty() {
        let merged_holes = RegionSet2D::union_many(&mut hole_sets);
        set = set.difference(&merged_holes);
    }
    set.regions
}

fn assign_holes_to_outers(
    outers: Vec<ArcPolyline2D>,
    holes: Vec<ArcPolyline2D>,
) -> Vec<PocketRegion2D> {
    if outers.is_empty() {
        return Vec::new();
    }
    let mut regions: Vec<PocketRegion2D> = outers
        .into_iter()
        .map(|o| PocketRegion2D {
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
                regions[idx].islands_cw.push(hole.ensure_orientation(false));
            }
        }
    }
    regions
        .into_iter()
        .filter(|r| r.net_area_mm2() > 1e-5)
        .collect()
}

fn assemble_regions_from_outers_and_holes(
    outers_ccw: Vec<Polyline<f64>>,
    holes: Vec<Polyline<f64>>,
) -> RegionSet2D {
    let out_arcs: Vec<ArcPolyline2D> = outers_ccw
        .iter()
        .map(|p| from_cavalier_polyline(p).ensure_orientation(true))
        .filter(|p| p.closed && p.segments.len() >= 2 && p.signed_area() > 1e-5)
        .collect();
    let hole_arcs: Vec<ArcPolyline2D> = holes
        .iter()
        .map(|p| from_cavalier_polyline(p).ensure_orientation(false))
        .filter(|p| p.closed && p.segments.len() >= 2 && p.signed_area().abs() > 1e-5)
        .collect();
    RegionSet2D {
        regions: assign_holes_to_outers(out_arcs, hole_arcs),
    }
}

/// Unions a list of simple `CCW` loops, returning `(merged_outer_ccw_plines, enclosed_courtyard_holes)`.
fn union_simple_ccw_loops(
    mut loops: Vec<Polyline<f64>>,
) -> (Vec<Polyline<f64>>, Vec<Polyline<f64>>) {
    loops.retain(|p| p.vertex_count() >= 2 && p.area().abs() > 1e-5);
    let mut outers: Vec<Polyline<f64>> = Vec::new();
    let mut holes: Vec<Polyline<f64>> = Vec::new();

    for mut cand in loops {
        if cand.vertex_count() < 2 || cand.area().abs() <= 1e-5 {
            continue;
        }
        if cand.orientation() != PlineOrientation::CounterClockwise {
            cand.invert_direction_mut();
        }
        let mut current_outers = Vec::with_capacity(outers.len() + 1);
        for existing in outers {
            let (pos_plines, neg_plines) = safe_pline_boolean(&existing, &cand, BooleanOp::Or);
            if pos_plines.len() == 1 {
                cand = pos_plines.into_iter().next().unwrap_or(cand);
                if cand.vertex_count() >= 2
                    && cand.orientation() != PlineOrientation::CounterClockwise
                {
                    cand.invert_direction_mut();
                }
                for neg in neg_plines {
                    if neg.vertex_count() >= 2 {
                        holes.push(neg);
                    }
                }
            } else {
                // Disjoint: keep `existing` separate and continue testing `cand` against remaining loops
                current_outers.push(existing);
            }
        }
        // Filter any previously formed courtyard holes that were covered by `cand`
        let mut surviving_holes = Vec::new();
        for mut h in holes {
            if h.vertex_count() < 2 {
                continue;
            }
            if h.orientation() != PlineOrientation::CounterClockwise {
                h.invert_direction_mut();
            }
            let (pos_plines, _) = safe_pline_boolean(&h, &cand, BooleanOp::Not);
            for pos in pos_plines {
                if pos.vertex_count() >= 2 && pos.area().abs() > 1e-5 {
                    surviving_holes.push(pos);
                }
            }
        }
        holes = surviving_holes;
        if cand.vertex_count() >= 2 && cand.area().abs() > 1e-5 {
            current_outers.push(cand);
        }
        outers = current_outers;
    }
    (outers, holes)
}

fn subtract_simple_ccw_from_set(
    set: &RegionSet2D,
    cut_ccw: &Polyline<f64>,
    cut_aabb: &Aabb2,
) -> RegionSet2D {
    let mut out_regions = Vec::new();

    for reg in &set.regions {
        if !reg.exact_aabb().overlaps(cut_aabb) {
            out_regions.push(reg.clone());
            continue;
        }
        let outer_pl = to_ccw_pline(&reg.outer_ccw);
        if outer_pl.vertex_count() < 2 {
            continue;
        }

        // Identity: `(O \ (I_1 ∪ ... ∪ I_m)) \ C == O \ (I_1 ∪ ... ∪ I_m ∪ C)`
        let mut obstacles = Vec::with_capacity(reg.islands_cw.len() + 1);
        for isl in &reg.islands_cw {
            obstacles.push(to_ccw_pline(isl));
        }
        obstacles.push(cut_ccw.clone());

        let (obs_outers, obs_courtyards) = union_simple_ccw_loops(obstacles);

        // Start from `O`, subtract each merged obstacle outer `U_k`, then add back `O ∩ courtyard`!
        let mut cur_outers = vec![outer_pl.clone()];
        let mut cur_holes: Vec<Polyline<f64>> = Vec::new();

        for obs in &obs_outers {
            let mut next_outers = Vec::new();
            for o in cur_outers {
                let (pos_plines, neg_plines) = safe_pline_boolean(&o, obs, BooleanOp::Not);
                for pos in pos_plines {
                    if pos.vertex_count() >= 2 && pos.area().abs() > 1e-5 {
                        next_outers.push(pos);
                    }
                }
                for neg in neg_plines {
                    if neg.vertex_count() >= 2 && neg.area().abs() > 1e-5 {
                        cur_holes.push(neg);
                    }
                }
            }
            cur_outers = next_outers;
        }

        let mut base = assemble_regions_from_outers_and_holes(cur_outers, cur_holes);
        for mut cy in obs_courtyards {
            if cy.vertex_count() < 2 {
                continue;
            }
            if cy.orientation() != PlineOrientation::CounterClockwise {
                cy.invert_direction_mut();
            }
            let (pos_plines, neg_plines) = safe_pline_boolean(&outer_pl, &cy, BooleanOp::And);
            let cy_set = assemble_regions_from_outers_and_holes(pos_plines, neg_plines);
            base = base.union(&cy_set);
        }
        out_regions.extend(base.regions);
    }

    RegionSet2D {
        regions: out_regions,
    }
}

fn intersect_set_with_simple_ccw(
    set: &RegionSet2D,
    mask_ccw: &Polyline<f64>,
    mask_aabb: &Aabb2,
) -> RegionSet2D {
    let mut out_regions = Vec::new();

    for reg in &set.regions {
        if !reg.exact_aabb().overlaps(mask_aabb) {
            continue;
        }
        let outer_pl = to_ccw_pline(&reg.outer_ccw);
        if outer_pl.vertex_count() < 2 {
            continue;
        }
        let (pos_plines, neg_plines) = safe_pline_boolean(&outer_pl, mask_ccw, BooleanOp::And);
        let mut sub = assemble_regions_from_outers_and_holes(pos_plines, neg_plines);
        for isl in &reg.islands_cw {
            if !sub.is_empty() {
                let isl_ccw = to_ccw_pline(isl);
                let isl_aabb = isl.exact_aabb();
                sub = subtract_simple_ccw_from_set(&sub, &isl_ccw, &isl_aabb);
            }
        }
        out_regions.extend(sub.regions);
    }

    RegionSet2D {
        regions: out_regions,
    }
}

/// Converts a list of closed 2D slice contours into a nested [`RegionSet2D`] using exact
/// containment-depth parity (outer solids = even depth, interior cavities = odd depth).
pub fn contours_to_region_set(contours: &[ArcPolyline2D]) -> RegionSet2D {
    let mut closed_loops: Vec<(f64, ArcPolyline2D)> = contours
        .iter()
        .filter(|c| c.closed && c.segments.len() >= 2)
        .filter_map(|c| {
            let area = c.signed_area();
            if area.abs() <= 1e-5 {
                return None;
            }
            let ccw = if area > 0.0 { c.clone() } else { c.reverse() };
            Some((area.abs(), ccw))
        })
        .collect();

    if closed_loops.is_empty() {
        return RegionSet2D::empty();
    }

    // Sort largest area first so parent loops always precede contained child loops
    closed_loops.sort_by(|a, b| b.0.total_cmp(&a.0));

    let mut depths = vec![0usize; closed_loops.len()];
    for i in 0..closed_loops.len() {
        let sample_pt = closed_loops[i].1.segments[0].start();
        let mut depth = 0usize;
        for (_, parent_loop) in closed_loops.iter().take(i) {
            if parent_loop.contains_point(sample_pt) {
                depth += 1;
            }
        }
        depths[i] = depth;
    }

    let mut result = RegionSet2D::empty();
    for (idx, (_, loop_ccw)) in closed_loops.into_iter().enumerate() {
        let loop_reg = RegionSet2D::from_single(PocketRegion2D::new(loop_ccw, Vec::new()));
        if depths[idx].is_multiple_of(2) {
            result = if result.is_empty() {
                loop_reg
            } else {
                result.union(&loop_reg)
            };
        } else {
            result = result.difference(&loop_reg);
        }
    }
    result
}

/// Organizes already-deduplicated simple closed `CCW` loops into Even-Odd nested
/// [`PocketRegion2D`]s (`outer_ccw` paired with direct `CW` child islands).
///
/// Expects loops with positive area and valid sample points; loops are sorted descending by
/// absolute area internally so parent loops always precede contained child loops.
pub fn nest_closed_loops_even_odd(ccw_loops: &[ArcPolyline2D]) -> Vec<PocketRegion2D> {
    struct Entry<'a> {
        ccw: &'a ArcPolyline2D,
        abs_area: f64,
        sample_pt: DVec2,
    }

    let mut entries: Vec<Entry<'_>> = ccw_loops
        .iter()
        .filter_map(|ccw| {
            let abs_area = ccw.signed_area().abs();
            if abs_area <= 1e-4 {
                return None;
            }
            let (sample_pt, _) = ccw.sample_at_fraction(0.137)?;
            Some(Entry {
                ccw,
                abs_area,
                sample_pt,
            })
        })
        .collect();
    entries.sort_by(|a, b| b.abs_area.total_cmp(&a.abs_area));

    let n = entries.len();
    let mut parent: Vec<Option<usize>> = vec![None; n];
    let mut depth: Vec<usize> = vec![0; n];

    for i in 0..n {
        let mut direct_parent: Option<usize> = None;
        for j in (0..i).rev() {
            if entries[j].ccw.contains_point(entries[i].sample_pt) {
                direct_parent = Some(j);
                break;
            }
        }
        parent[i] = direct_parent;
        depth[i] = direct_parent.map_or(0, |p| depth[p] + 1);
    }

    let mut out = Vec::new();
    for i in 0..n {
        if !depth[i].is_multiple_of(2) {
            continue;
        }
        let mut islands_cw = Vec::new();
        let mut isl_area_sum = 0.0;
        for child in (i + 1)..n {
            if parent[child] == Some(i) {
                isl_area_sum += entries[child].abs_area;
                islands_cw.push(entries[child].ccw.ensure_orientation(false));
            }
        }
        let net_area = (entries[i].abs_area - isl_area_sum).max(0.0);
        if net_area > 1e-4 {
            out.push(PocketRegion2D {
                outer_ccw: entries[i].ccw.ensure_orientation(true),
                islands_cw,
            });
        }
    }
    out
}

/// Computes the exact 2D CCW convex hull of `points` via Andrew's monotone chain (`O(N log N)`)
/// and wraps it as a single-region [`RegionSet2D`].
pub fn region_from_convex_hull_2d(points: &[DVec2]) -> Option<RegionSet2D> {
    if points.len() < 3 {
        return None;
    }
    let mut pts: Vec<DVec2> = points.iter().copied().filter(|p| p.is_finite()).collect();
    pts.sort_by(|a, b| a.x.total_cmp(&b.x).then_with(|| a.y.total_cmp(&b.y)));
    pts.dedup_by(|a, b| a.distance_squared(*b) <= 1e-10);
    if pts.len() < 3 {
        return None;
    }

    let cross =
        |o: DVec2, a: DVec2, b: DVec2| (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    let mut lower: Vec<DVec2> = Vec::with_capacity(pts.len());
    for &p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 1e-9 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<DVec2> = Vec::with_capacity(pts.len());
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 1e-9 {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    if lower.len() < 3 {
        return None;
    }

    let mut segs = Vec::with_capacity(lower.len());
    for i in 0..lower.len() {
        let a = lower[i];
        let b = lower[(i + 1) % lower.len()];
        if a.distance_squared(b) > 1e-10 {
            segs.push(ArcSegment2D::Line { start: a, end: b });
        }
    }
    if segs.len() < 3 {
        return None;
    }
    let poly = ArcPolyline2D {
        segments: segs,
        closed: true,
    }
    .ensure_orientation(true);
    let reg = RegionSet2D::from_single(PocketRegion2D::new(poly, Vec::new()));
    (!reg.is_empty()).then_some(reg)
}

/// Computes the exact 2D capsule (`CCW` stadium or annular sector) swept by a disk of `radius_mm`
/// moving along `seg`.
pub fn swept_segment_capsule_2d(seg: &ArcSegment2D, radius_mm: f64) -> Option<PocketRegion2D> {
    if !radius_mm.is_finite() || radius_mm <= 1e-5 {
        return None;
    }
    match *seg {
        ArcSegment2D::Line { start, end } => {
            let delta = end - start;
            let len = delta.length();
            if len <= 1e-5 {
                return RegionSet2D::from_circle(start, radius_mm)
                    .regions
                    .into_iter()
                    .next();
            }
            let u = delta / len;
            let n = DVec2::new(-u.y, u.x); // Left normal
            let r_start = start - n * radius_mm;
            let r_end = end - n * radius_mm;
            let l_end = end + n * radius_mm;
            let l_start = start + n * radius_mm;
            let stadium = ArcPolyline2D {
                segments: vec![
                    ArcSegment2D::Line {
                        start: r_start,
                        end: r_end,
                    },
                    ArcSegment2D::Arc {
                        start: r_end,
                        end: l_end,
                        center: end,
                        radius: radius_mm,
                        ccw: true,
                    },
                    ArcSegment2D::Line {
                        start: l_end,
                        end: l_start,
                    },
                    ArcSegment2D::Arc {
                        start: l_start,
                        end: r_start,
                        center: start,
                        radius: radius_mm,
                        ccw: true,
                    },
                ],
                closed: true,
            };
            Some(PocketRegion2D::new(stadium, Vec::new()))
        }
        ArcSegment2D::Arc {
            start,
            end,
            center,
            radius: arc_r,
            ccw,
        } => {
            let sweep = seg.signed_sweep_rad().abs();
            // Full or near-full circle: annular ring (or filled disk if `arc_r <= radius_mm`)
            if sweep >= std::f64::consts::TAU - 1e-3 {
                let outer = RegionSet2D::from_circle(center, arc_r + radius_mm);
                if arc_r > radius_mm + 1e-4 {
                    let inner = RegionSet2D::from_circle(center, arc_r - radius_mm);
                    return outer.difference(&inner).regions.into_iter().next();
                }
                return outer.regions.into_iter().next();
            }

            // When arc radius exceeds cutter radius and sweep <= 170°, construct exact 4-arc curved stadium;
            // otherwise union overlapping sub-chord stadia + endpoint disks for rock-solid robustness!
            if arc_r > radius_mm + 0.05 && sweep <= 2.95 {
                let (p0, p1) = if ccw { (start, end) } else { (end, start) };
                let u0 = (p0 - center).normalize_or_zero();
                let u1 = (p1 - center).normalize_or_zero();
                if u0.length_squared() > 0.5 && u1.length_squared() > 0.5 {
                    let out0 = center + u0 * (arc_r + radius_mm);
                    let out1 = center + u1 * (arc_r + radius_mm);
                    let in1 = center + u1 * (arc_r - radius_mm);
                    let in0 = center + u0 * (arc_r - radius_mm);
                    let curved_stadium = ArcPolyline2D {
                        segments: vec![
                            ArcSegment2D::Arc {
                                start: out0,
                                end: out1,
                                center,
                                radius: arc_r + radius_mm,
                                ccw: true,
                            },
                            ArcSegment2D::Arc {
                                start: out1,
                                end: in1,
                                center: p1,
                                radius: radius_mm,
                                ccw: true,
                            },
                            ArcSegment2D::Arc {
                                start: in1,
                                end: in0,
                                center,
                                radius: arc_r - radius_mm,
                                ccw: false,
                            },
                            ArcSegment2D::Arc {
                                start: in0,
                                end: out0,
                                center: p0,
                                radius: radius_mm,
                                ccw: true,
                            },
                        ],
                        closed: true,
                    };
                    return Some(PocketRegion2D::new(curved_stadium, Vec::new()));
                }
            }

            let steps = ((sweep / 0.45).ceil() as usize).clamp(2, 16);
            let mut pieces = Vec::with_capacity(steps);
            for k in 0..steps {
                let t0 = (k as f64) / (steps as f64);
                let t1 = ((k + 1) as f64) / (steps as f64);
                if let (Some((pa, _)), Some((pb, _))) = (seg.eval_at(t0), seg.eval_at(t1))
                    && let Some(cap) = swept_segment_capsule_2d(
                        &ArcSegment2D::Line { start: pa, end: pb },
                        radius_mm,
                    )
                {
                    pieces.push(RegionSet2D::from_single(cap));
                }
            }
            RegionSet2D::union_many(&mut pieces)
                .regions
                .into_iter()
                .next()
        }
    }
}

/// Subtracts `air_set` from an open or closed `ArcPolyline2D`, returning ordered active sub-chains
/// (merging seam-wrapping spans on closed loops!).
pub fn clip_polyline_outside_region_set(
    poly: &ArcPolyline2D,
    air_set: &RegionSet2D,
) -> Vec<ArcPolyline2D> {
    clip_polyline_by_region_set(poly, air_set, false)
}

/// Clips an open or closed [`ArcPolyline2D`] against `set`, retaining spans either inside
/// (`keep_inside == true`) or outside (`keep_inside == false`) the region set.
pub fn clip_polyline_by_region_set(
    poly: &ArcPolyline2D,
    set: &RegionSet2D,
    keep_inside: bool,
) -> Vec<ArcPolyline2D> {
    if poly.segments.is_empty() {
        return Vec::new();
    }
    if set.is_empty() {
        return if keep_inside {
            Vec::new()
        } else {
            vec![poly.clone()]
        };
    }

    let mut active_segs: Vec<ArcSegment2D> = Vec::new();
    for seg in &poly.segments {
        let len = seg.length();
        if len <= 1e-5 {
            continue;
        }
        // Adaptive fine parameter subdivision along segment to isolate boundary crossings cleanly
        let n_samples = ((len / 0.25).ceil() as usize).clamp(4, 128);
        let mut span_start: Option<f64> = None;

        for k in 0..n_samples {
            let t0 = (k as f64) / (n_samples as f64);
            let t1 = ((k + 1) as f64) / (n_samples as f64);
            let t_mid = 0.5 * (t0 + t1);
            let Some((p_mid, _)) = seg.eval_at(t_mid) else {
                continue;
            };
            let in_set = set.contains_point(p_mid);
            let keep = if keep_inside { in_set } else { !in_set };
            match (span_start, keep) {
                (None, true) => span_start = Some(t0),
                (Some(s0), false) => {
                    if let Some(sub) = extract_subsegment(seg, s0, t0) {
                        active_segs.push(sub);
                    }
                    span_start = None;
                }
                _ => {}
            }
        }
        if let Some(s0) = span_start
            && let Some(sub) = extract_subsegment(seg, s0, 1.0)
        {
            active_segs.push(sub);
        }
    }

    if active_segs.is_empty() {
        return Vec::new();
    }

    // Group contiguous segments into chains
    let mut chains: Vec<ArcPolyline2D> = Vec::new();
    let mut cur: Vec<ArcSegment2D> = Vec::new();
    for s in active_segs {
        if let Some(prev) = cur.last()
            && prev.end().distance(s.start()) > 1e-3
        {
            chains.push(finalize_chain(std::mem::take(&mut cur)));
        }
        cur.push(s);
    }
    if !cur.is_empty() {
        chains.push(finalize_chain(cur));
    }

    // If `poly` was closed and the first and last chains meet across the loop seam (without forming
    // the entire closed loop), stitch `[last, first]` into one seamless corner pass!
    if poly.closed
        && chains.len() >= 2
        && let (Some(first_pt), Some(last_pt)) = (
            chains[0].segments.first().map(ArcSegment2D::start),
            chains
                .last()
                .and_then(|c| c.segments.last())
                .map(ArcSegment2D::end),
        )
        && first_pt.distance(last_pt) <= 1e-3
    {
        let first = chains.remove(0);
        if let Some(last) = chains.last_mut() {
            last.segments.extend(first.segments);
            if let (Some(s), Some(e)) = (
                last.segments.first().map(ArcSegment2D::start),
                last.segments.last().map(ArcSegment2D::end),
            ) && s.distance(e) <= 1e-3
            {
                last.closed = true;
            }
        }
    }

    chains
        .into_iter()
        .filter(|c| c.perimeter() > 0.10)
        .collect()
}

fn finalize_chain(segs: Vec<ArcSegment2D>) -> ArcPolyline2D {
    let closed = segs.len() >= 2
        && segs
            .first()
            .zip(segs.last())
            .is_some_and(|(a, b)| a.start().distance(b.end()) <= 1e-3);
    ArcPolyline2D {
        segments: segs,
        closed,
    }
}

fn extract_subsegment(seg: &ArcSegment2D, t0: f64, t1: f64) -> Option<ArcSegment2D> {
    if t1 - t0 <= 1e-6 {
        return None;
    }
    if t0 <= 1e-6 && t1 >= 1.0 - 1e-6 {
        return Some(*seg);
    }
    let tail = if t0 <= 1e-6 {
        *seg
    } else {
        seg.split_at(t0).1?
    };
    let rem_frac = ((t1 - t0) / (1.0 - t0).max(1e-9)).clamp(0.0, 1.0);
    if rem_frac >= 1.0 - 1e-6 {
        Some(tail)
    } else {
        tail.split_at(rem_frac).0
    }
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

    #[test]
    fn region_set_booleans_difference_intersection_and_even_odd_nesting() {
        let outer = rect_ccw(0.0, 0.0, 40.0, 40.0);
        let cavity = rect_ccw(8.0, 8.0, 32.0, 32.0);
        let inner_island = rect_ccw(16.0, 16.0, 24.0, 24.0);

        // Even-odd nesting: outer (depth 0, solid) - cavity (depth 1, hole) + island (depth 2, solid)
        let nested = contours_to_region_set(&[inner_island, outer, cavity]);
        // Area = 40*40 - 24*24 + 8*8 = 1600 - 576 + 64 = 1088
        assert!(approx_eq(nested.total_area_mm2(), 1088.0, 1e-2));
        assert!(nested.contains_point(DVec2::new(4.0, 4.0)));
        assert!(!nested.contains_point(DVec2::new(10.0, 10.0)));
        assert!(nested.contains_point(DVec2::new(20.0, 20.0)));

        // Intersect with left half [0..20] x [0..40]: exact symmetry => half the area (544)
        let left_half =
            RegionSet2D::from_single(PocketRegion2D::new(rect_ccw(0.0, 0.0, 20.0, 40.0), vec![]));
        let inter = nested.intersect(&left_half);
        assert!(approx_eq(inter.total_area_mm2(), 544.0, 5e-2));

        // Disjoint + overlapping disk union area
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
    fn swept_capsules_and_polyline_clipping_match_analytical_areas() {
        // Line capsule length 10, radius 2 => 10*4 + pi*4
        let line = ArcSegment2D::Line {
            start: DVec2::new(0.0, 0.0),
            end: DVec2::new(10.0, 0.0),
        };
        let cap = swept_segment_capsule_2d(&line, 2.0).unwrap();
        let expected_line_area = 40.0 + std::f64::consts::PI * 4.0;
        assert!(approx_eq(cap.net_area_mm2(), expected_line_area, 0.05));

        // Quarter-circle arc capsule R=10, tool r=2 => sector area (pi/4)*((12^2)-(8^2)) + pi*2^2
        let arc = ArcSegment2D::Arc {
            start: DVec2::new(10.0, 0.0),
            end: DVec2::new(0.0, 10.0),
            center: DVec2::ZERO,
            radius: 10.0,
            ccw: true,
        };
        let arc_cap = swept_segment_capsule_2d(&arc, 2.0).unwrap();
        let expected_arc_area =
            0.25 * std::f64::consts::PI * (144.0 - 64.0) + std::f64::consts::PI * 4.0;
        assert!(approx_eq(arc_cap.net_area_mm2(), expected_arc_area, 0.1));

        // Clip a line segment [0..20] outside a central air block [6..14] => two outside pieces
        let path = ArcPolyline2D {
            segments: vec![ArcSegment2D::Line {
                start: DVec2::new(0.0, 5.0),
                end: DVec2::new(20.0, 5.0),
            }],
            closed: false,
        };
        let air =
            RegionSet2D::from_single(PocketRegion2D::new(rect_ccw(6.0, 0.0, 14.0, 10.0), vec![]));
        let outside = clip_polyline_outside_region_set(&path, &air);
        assert_eq!(outside.len(), 2);
        let total_len: f64 = outside.iter().map(ArcPolyline2D::perimeter).sum();
        assert!(approx_eq(total_len, 12.0, 0.5));
    }
}
