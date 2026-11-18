//! Exact 2D Boolean Region Sets (`RegionSet2D`), 2D Swept Toolpath Coverage, and Gouge-Proof
//! **2.5D Rest Machining** (`05-operations.md` §Rest Machining).

use std::sync::Arc;

use cavalier_contours::polyline::{
    BooleanOp, PlineBooleanOptions, PlineOrientation, PlineSource, PlineSourceMut, Polyline,
};
use chip_math::{Aabb2, ArcPolyline2D, ArcSegment2D, DVec2};

use crate::{
    cl_data::ArcPlane,
    offset2d::{PocketRegion2D, from_cavalier_polyline, offset_polyline, to_cavalier_polyline},
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
}

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
        let p_r = center + DVec2::new(radius_mm, 0.0);
        let p_l = center - DVec2::new(radius_mm, 0.0);
        let circle = ArcPolyline2D {
            segments: vec![
                ArcSegment2D::Arc {
                    start: p_r,
                    end: p_l,
                    center,
                    radius: radius_mm,
                    ccw: true,
                },
                ArcSegment2D::Arc {
                    start: p_l,
                    end: p_r,
                    center,
                    radius: radius_mm,
                    ccw: true,
                },
            ],
            closed: true,
        };
        Self::from_single(PocketRegion2D::new(circle, Vec::new()))
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

        // `(A ∪ R)`: first subtract `R` from overlapping `A` (`A \ R`), then append `R`?
        // Careful: keeping `A \ R` and `R` as separate touching polygons would leave an internal seam!
        // Instead: union all outer CCW loops `U = (∪ O_A) ∪ O_R`, and compute remaining holes:
        // Any point in a hole of `A ∪ R` must be BOTH outside `A` AND outside `R`!
        let bool_opts = bool_options();
        let mut outer_plines: Vec<Polyline<f64>> = overlapping
            .iter()
            .map(|r| to_ccw_pline(&r.outer_ccw))
            .collect();
        outer_plines.push(to_ccw_pline(&reg.outer_ccw));

        let (merged_outers, courtyard_holes) = union_simple_ccw_loops(outer_plines, &bool_opts);
        let mut merged_set = assemble_regions_from_outers_and_holes(merged_outers, courtyard_holes);

        // Existing island `H_A` of `A_i` survives ONLY where it is outside `R` AND outside all other `A_j`:
        // Equivalently: collect all holes from `overlapping` + `reg` + `courtyard_holes`;
        // Actually, for `S_ov = overlapping`, `H_ov_total` of `A_i` outside `R` is `(H_A \ R)` intersected with `(∩_{j != i} complement(A_j))`.
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

fn bool_options() -> PlineBooleanOptions<'static> {
    PlineBooleanOptions {
        pos_equal_eps: 1e-5,
        ..Default::default()
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

    let outer_loops =
        crate::offset2d::offset_polyline(&reg.outer_ccw.ensure_orientation(true), signed_inward_mm);
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
        let isl_loops =
            crate::offset2d::offset_polyline(&isl.ensure_orientation(true), -signed_inward_mm);
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
    opts: &PlineBooleanOptions,
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
            let res = existing.boolean_opt(&cand, BooleanOp::Or, opts);
            if res.pos_plines.len() == 1 {
                cand = res
                    .pos_plines
                    .into_iter()
                    .next()
                    .map(|b| b.pline)
                    .unwrap_or(cand);
                if cand.vertex_count() >= 2
                    && cand.orientation() != PlineOrientation::CounterClockwise
                {
                    cand.invert_direction_mut();
                }
                for neg in res.neg_plines {
                    if neg.pline.vertex_count() >= 2 {
                        holes.push(neg.pline);
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
            let diff = h.boolean_opt(&cand, BooleanOp::Not, opts);
            for pos in diff.pos_plines {
                if pos.pline.vertex_count() >= 2 && pos.pline.area().abs() > 1e-5 {
                    surviving_holes.push(pos.pline);
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
    let opts = bool_options();
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

        let (obs_outers, obs_courtyards) = union_simple_ccw_loops(obstacles, &opts);

        // Start from `O`, subtract each merged obstacle outer `U_k`, then add back `O ∩ courtyard`!
        let mut cur_outers = vec![outer_pl.clone()];
        let mut cur_holes: Vec<Polyline<f64>> = Vec::new();

        for obs in &obs_outers {
            let mut next_outers = Vec::new();
            for o in cur_outers {
                let diff = o.boolean_opt(obs, BooleanOp::Not, &opts);
                for pos in diff.pos_plines {
                    if pos.pline.vertex_count() >= 2 && pos.pline.area().abs() > 1e-5 {
                        next_outers.push(pos.pline);
                    }
                }
                for neg in diff.neg_plines {
                    if neg.pline.vertex_count() >= 2 && neg.pline.area().abs() > 1e-5 {
                        cur_holes.push(neg.pline);
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
            let inter = outer_pl.boolean_opt(&cy, BooleanOp::And, &opts);
            let cy_set = assemble_regions_from_outers_and_holes(
                inter.pos_plines.into_iter().map(|b| b.pline).collect(),
                inter.neg_plines.into_iter().map(|b| b.pline).collect(),
            );
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
    let opts = bool_options();
    let mut out_regions = Vec::new();

    for reg in &set.regions {
        if !reg.exact_aabb().overlaps(mask_aabb) {
            continue;
        }
        let outer_pl = to_ccw_pline(&reg.outer_ccw);
        if outer_pl.vertex_count() < 2 {
            continue;
        }
        let inter = outer_pl.boolean_opt(mask_ccw, BooleanOp::And, &opts);
        let mut sub = assemble_regions_from_outers_and_holes(
            inter.pos_plines.into_iter().map(|b| b.pline).collect(),
            inter.neg_plines.into_iter().map(|b| b.pline).collect(),
        );
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
        if inp.tool_diameter_mm <= 1e-4 {
            continue;
        }
        for seg3 in &inp.toolpath.segments_wcs {
            if matches!(seg3.motion, SegmentMotion3D::Rapid) {
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
    // Guarantees:
    // 1. Every tool center lies inside `Ω_safe` (zero wall/island gouges).
    // 2. Straight walls already machined flush to target vanish from `M_uncut`, isolating true uncut
    //    corners/channels instead of leaving a hairline `overlap` picture-frame around the whole pocket!
    // 3. Each rest corner blends smoothly by `overlap` into the already-cleared cavity!
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

/// Subtracts `air_set` from an open or closed `ArcPolyline2D`, returning ordered active sub-chains
/// (merging seam-wrapping spans on closed loops!).
pub fn clip_polyline_outside_region_set(
    poly: &ArcPolyline2D,
    air_set: &RegionSet2D,
) -> Vec<ArcPolyline2D> {
    clip_polyline_by_region_set(poly, air_set, false)
}

fn clip_polyline_by_region_set(
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
    // the entire closed loop), Stitch `[last, first]` into one seamless corner pass!
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
}
