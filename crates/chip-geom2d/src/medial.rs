//! Segment-Voronoi interior medial axis graph, maximum inscribed circle pole extraction, and
//! pruned 2D medial-axis chains (`boostvoronoi`).

use std::collections::{HashMap, HashSet};

use boostvoronoi::{
    builder::Builder,
    diagram::{CellIndex, Diagram, SourceCategory},
    geometry::{Line, Point},
};
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2};

use crate::offset::{HelicalEntryPole, PocketRegion2D, find_helical_entry_pole};

/// Interior medial-axis sample node with its exact clearance radius to the region boundary.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MedialNode2D {
    pub pos_wcs: DVec2,
    pub clearance_radius_mm: f64,
}

/// Ordered continuous chain of interior medial-axis samples (`(pos_wcs, clearance_radius_mm)`).
#[derive(Debug, Clone, PartialEq)]
pub struct MedialChain2D {
    pub samples: Vec<MedialNode2D>,
    pub closed: bool,
}

/// Discretizes an [`ArcPolyline2D`] into closed non-degenerate linear chords within `chord_tol_mm`
/// suitable for integer-scaled segment Voronoi construction.
fn discretize_loop_chords(poly: &ArcPolyline2D, chord_tol_mm: f64) -> Vec<(DVec2, DVec2)> {
    let mut pts: Vec<DVec2> = Vec::with_capacity(poly.segments.len() * 4);
    let tol = chord_tol_mm.clamp(0.004, 0.25);

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
                    (2.0 * (1.0 - tol / radius).clamp(-1.0, 1.0).acos()).clamp(0.06, 0.40)
                } else {
                    0.40
                };
                let steps = ((sweep / step_ang).ceil() as usize).clamp(2, 128);
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

struct BuiltVoronoi {
    diagram: Diagram,
    center: DVec2,
    inv_scale: f64,
    used_segs: Vec<(DVec2, DVec2)>,
}

fn orient2d_i64(a: Point<i32>, b: Point<i32>, c: Point<i32>) -> i64 {
    (i64::from(b.x) - i64::from(a.x)) * (i64::from(c.y) - i64::from(a.y))
        - (i64::from(b.y) - i64::from(a.y)) * (i64::from(c.x) - i64::from(a.x))
}

fn on_segment_i64(a: Point<i32>, b: Point<i32>, p: Point<i32>) -> bool {
    orient2d_i64(a, b, p) == 0
        && p.x >= a.x.min(b.x)
        && p.x <= a.x.max(b.x)
        && p.y >= a.y.min(b.y)
        && p.y <= a.y.max(b.y)
}

fn int_segments_conflict(s1: &Line<i32>, s2: &Line<i32>) -> bool {
    let a = s1.start;
    let b = s1.end;
    let c = s2.start;
    let d = s2.end;

    // Fast AABB reject
    if a.x.max(b.x) < c.x.min(d.x)
        || c.x.max(d.x) < a.x.min(b.x)
        || a.y.max(b.y) < c.y.min(d.y)
        || c.y.max(d.y) < a.y.min(b.y)
    {
        return false;
    }

    // Exact or reversed duplicate segment
    if (a == c && b == d) || (a == d && b == c) {
        return true;
    }

    // Shared single endpoint is allowed UNLESS collinear & folded back over one another!
    if a == c || a == d || b == c || b == d {
        if orient2d_i64(a, b, c) == 0 && orient2d_i64(a, b, d) == 0 {
            let shared = if a == c || a == d { a } else { b };
            let o1 = if a == shared { b } else { a };
            let o2 = if c == shared { d } else { c };
            let dot = (i64::from(o1.x) - i64::from(shared.x))
                * (i64::from(o2.x) - i64::from(shared.x))
                + (i64::from(o1.y) - i64::from(shared.y)) * (i64::from(o2.y) - i64::from(shared.y));
            return dot > 0;
        }
        return false;
    }

    let o1 = orient2d_i64(a, b, c);
    let o2 = orient2d_i64(a, b, d);
    let o3 = orient2d_i64(c, d, a);
    let o4 = orient2d_i64(c, d, b);

    if (o1 > 0 && o2 < 0 || o1 < 0 && o2 > 0) && (o3 > 0 && o4 < 0 || o3 < 0 && o4 > 0) {
        return true;
    }
    on_segment_i64(a, b, c)
        || on_segment_i64(a, b, d)
        || on_segment_i64(c, d, a)
        || on_segment_i64(c, d, b)
}

fn build_region_voronoi(region: &PocketRegion2D, chord_tol_mm: f64) -> Option<BuiltVoronoi> {
    let aabb = region.exact_aabb();
    if !aabb.is_valid() {
        return None;
    }
    let center = aabb.center();
    let max_span = aabb.size().x.max(aabb.size().y).max(1.0);
    let scale = (2_000_000.0 / max_span).clamp(100.0, 20_000.0);

    let mut raw_chords = discretize_loop_chords(&region.outer_ccw, chord_tol_mm);
    for isl in &region.islands_cw {
        raw_chords.extend(discretize_loop_chords(isl, chord_tol_mm));
    }
    if raw_chords.len() < 3 {
        return None;
    }

    let to_i32 = |p: DVec2| -> Point<i32> {
        let ix = ((p.x - center.x) * scale).round().clamp(-1.0e7, 1.0e7) as i32;
        let iy = ((p.y - center.y) * scale).round().clamp(-1.0e7, 1.0e7) as i32;
        Point { x: ix, y: iy }
    };

    let mut int_lines: Vec<Line<i32>> = Vec::with_capacity(raw_chords.len());
    let mut used_segs: Vec<(DVec2, DVec2)> = Vec::with_capacity(raw_chords.len());
    for (a, b) in raw_chords {
        let pa = to_i32(a);
        let pb = to_i32(b);
        if pa == pb {
            continue;
        }
        // Guard `boostvoronoi` non-intersection precondition: reject any candidate chord that
        // overlaps, T-junctions, or crosses an already-accepted `Line<i32>` except at shared endpoints!
        if int_lines
            .iter()
            .any(|existing| int_segments_conflict(existing, &Line::new(pa, pb)))
        {
            continue;
        }
        int_lines.push(Line::new(pa, pb));
        used_segs.push((a, b));
    }
    if int_lines.len() < 3 {
        return None;
    }

    let builder_with_segs = Builder::<i32>::default()
        .with_segments(int_lines.iter())
        .ok()?;
    let diagram = builder_with_segs.build().ok()?;
    Some(BuiltVoronoi {
        diagram,
        center,
        inv_scale: 1.0 / scale,
        used_segs,
    })
}

/// Extracts interior medial-axis circle-event vertices of `region` using `boostvoronoi`.
pub fn extract_interior_medial_nodes(
    region: &PocketRegion2D,
    chord_tol_mm: f64,
) -> Vec<MedialNode2D> {
    let aabb = region.exact_aabb();
    let Some(bv) = build_region_voronoi(region, chord_tol_mm) else {
        return Vec::new();
    };
    let BuiltVoronoi {
        diagram,
        center,
        inv_scale,
        ..
    } = bv;

    let mut nodes = Vec::new();
    for v in diagram.vertices() {
        if v.is_site_point() {
            continue;
        }
        let pt = DVec2::new(center.x + v.x() * inv_scale, center.y + v.y() * inv_scale);
        if pt.is_finite() && aabb.contains_point(pt) && region.contains_interior_point(pt) {
            let clr = region.min_boundary_distance(pt);
            if clr > 1e-5 {
                nodes.push(MedialNode2D {
                    pos_wcs: pt,
                    clearance_radius_mm: clr,
                });
            }
        }
    }

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

/// Extracts pruned interior **2D Medial Axis Chains** ([`MedialChain2D`]):
/// 1. Retains primary finite interior Voronoi edges (`edge.is_primary()`, pruning reflex-vertex
///    perpendicular separator rays automatically).
/// 2. Samples curved parabolic Voronoi arcs (`point` vs. `segment` equidistance parabola) accurately.
/// 3. Iteratively prunes smooth-arc chord-discretization comb-teeth ($\Delta r / \Delta s > 0.968$,
///    corresponding to smooth chord joints $\alpha > 151^\circ$) while preserving all genuine salient
///    corner-climb bisectors ($90^\circ$ square corners, acute serifs, hexagons, etc.).
/// 4. Chains degree-2 paths into continuous strokes and evaluates exact `ArcPolyline2D` clearance
///    radii at every sample!
#[derive(Clone)]
struct GraphEdge {
    u: usize,
    v: usize,
    pts: Vec<DVec2>,
    alive: bool,
}

pub fn extract_medial_axis_chains(
    region: &PocketRegion2D,
    chord_tol_mm: f64,
) -> Vec<MedialChain2D> {
    let aabb = region.exact_aabb();
    let Some(bv) = build_region_voronoi(region, chord_tol_mm) else {
        return Vec::new();
    };
    let BuiltVoronoi {
        diagram,
        center,
        inv_scale,
        used_segs,
    } = bv;

    let n_verts = diagram.vertices().len();
    let mut vert_pos: Vec<DVec2> = vec![DVec2::ZERO; n_verts];
    let mut vert_clr: Vec<f64> = vec![0.0; n_verts];
    for (vi, v) in diagram.vertices().iter().enumerate() {
        let p = DVec2::new(center.x + v.x() * inv_scale, center.y + v.y() * inv_scale);
        vert_pos[vi] = p;
        if p.is_finite() {
            vert_clr[vi] = region.min_boundary_distance(p);
        }
    }

    let mut edges: Vec<GraphEdge> = Vec::new();
    let mut seen_undirected: HashSet<(usize, usize)> = HashSet::new();

    for idx in 0..diagram.num_edges() {
        let eid = diagram.edge_index_unchecked(idx);
        let Ok(edge) = diagram.edge(eid) else {
            continue;
        };
        if !edge.is_primary() || diagram.edge_is_infinite(eid).unwrap_or(true) {
            continue;
        }
        let (Ok(Some(v0_id)), Ok(Some(v1_id))) =
            (diagram.edge_get_vertex0(eid), diagram.edge_get_vertex1(eid))
        else {
            continue;
        };
        let u = v0_id.usize();
        let v = v1_id.usize();
        if u == v || u >= n_verts || v >= n_verts {
            continue;
        }
        let pair = if u < v { (u, v) } else { (v, u) };
        if !seen_undirected.insert(pair) {
            continue;
        }

        let p0 = vert_pos[u];
        let p1 = vert_pos[v];
        if !p0.is_finite() || !p1.is_finite() || p0.distance(p1) <= 1e-5 {
            continue;
        }

        let pts = if edge.is_curved()
            && let Ok(twin_id) = diagram.edge_get_twin(eid)
            && let (Ok(c0_id), Ok(c1_id)) =
                (edge.cell(), diagram.edge(twin_id).and_then(|te| te.cell()))
            && let Some(curved_pts) =
                sample_parabolic_voronoi_edge(&diagram, &used_segs, c0_id, c1_id, p0, p1, 0.25)
        {
            curved_pts
        } else {
            subdivide_linear_span(p0, p1, 0.5)
        };

        let mid = pts[pts.len() / 2];
        // Test interior containment at midpoint (and quarter points so exterior chords never sneak in!)
        let q1 = pts.get(pts.len() / 4).copied().unwrap_or((p0 + mid) * 0.5);
        let q3 = pts
            .get((3 * pts.len()) / 4)
            .copied()
            .unwrap_or((mid + p1) * 0.5);
        if !aabb.contains_point(mid)
            || !region.contains_interior_point(mid)
            || (!region.contains_interior_point(q1) && vert_clr[u] > 1e-3)
            || (!region.contains_interior_point(q3) && vert_clr[v] > 1e-3)
        {
            continue;
        }

        edges.push(GraphEdge {
            u,
            v,
            pts,
            alive: true,
        });
    }

    if edges.is_empty() {
        if let Some(pole) = find_medial_inscribed_pole(region)
            && pole.max_helix_radius_mm > 1e-4
        {
            return vec![MedialChain2D {
                samples: vec![MedialNode2D {
                    pos_wcs: pole.center_xy,
                    clearance_radius_mm: pole.max_helix_radius_mm,
                }],
                closed: false,
            }];
        }
        return Vec::new();
    }

    // Track highest-clearance vertex in case a pure circular dot prunes all its chord-joint spokes!
    let best_hub_vid = edges
        .iter()
        .flat_map(|e| [e.u, e.v])
        .max_by(|&a, &b| vert_clr[a].total_cmp(&vert_clr[b]));

    // Iteratively prune smooth-arc discretization comb-teeth (`dr / ds > sin(75.5°) = 0.968`)
    // touching the boundary (`r_leaf <= boundary_eps`), while preserving genuine corner bisectors!
    let boundary_eps = (3.0 * chord_tol_mm).max(0.06);
    const SMOOTH_ARC_SIN_HALF_THRESHOLD: f64 = 0.968;
    loop {
        let mut deg: HashMap<usize, usize> = HashMap::new();
        for e in &edges {
            if e.alive {
                *deg.entry(e.u).or_insert(0) += 1;
                *deg.entry(e.v).or_insert(0) += 1;
            }
        }
        let mut pruned_any = false;
        for e in &mut edges {
            if !e.alive {
                continue;
            }
            let du = deg.get(&e.u).copied().unwrap_or(0);
            let dv = deg.get(&e.v).copied().unwrap_or(0);
            let leaf_pair = if du == 1 && vert_clr[e.u] <= boundary_eps {
                Some((e.u, e.v))
            } else if dv == 1 && vert_clr[e.v] <= boundary_eps {
                Some((e.v, e.u))
            } else {
                None
            };
            if let Some((leaf_v, inner_v)) = leaf_pair {
                let chord_len = vert_pos[leaf_v].distance(vert_pos[inner_v]);
                let dr = (vert_clr[inner_v] - vert_clr[leaf_v]).max(0.0);
                let ratio = if chord_len > 1e-6 {
                    dr / chord_len
                } else {
                    1.0
                };
                // Prune smooth-wall discretization spoke OR microscopic sub-tolerance whisker
                if ratio > SMOOTH_ARC_SIN_HALF_THRESHOLD
                    || (chord_len <= boundary_eps && dr <= boundary_eps)
                {
                    e.alive = false;
                    pruned_any = true;
                }
            }
        }
        if !pruned_any {
            break;
        }
    }

    let alive_edges: Vec<GraphEdge> = edges.into_iter().filter(|e| e.alive).collect();
    if alive_edges.is_empty() {
        if let Some(vid) = best_hub_vid
            && vert_clr[vid] > 1e-4
        {
            return vec![MedialChain2D {
                samples: vec![MedialNode2D {
                    pos_wcs: vert_pos[vid],
                    clearance_radius_mm: vert_clr[vid],
                }],
                closed: false,
            }];
        }
        return Vec::new();
    }

    // Build adjacency list and stitch maximal continuous chains across degree-2 interior nodes
    let mut adj: HashMap<usize, Vec<usize>> = HashMap::new();
    for (ei, e) in alive_edges.iter().enumerate() {
        adj.entry(e.u).or_default().push(ei);
        adj.entry(e.v).or_default().push(ei);
    }

    let mut used_edge = vec![false; alive_edges.len()];
    let mut raw_chains: Vec<(Vec<DVec2>, bool)> = Vec::new();

    // 1. Walk open/tree chains starting at non-degree-2 vertices (corners & branch junctions)
    let mut terminals: Vec<usize> = adj
        .iter()
        .filter_map(|(&vid, inc)| (inc.len() != 2).then_some(vid))
        .collect();
    terminals.sort_unstable();

    for start_v in terminals {
        let Some(inc_list) = adj.get(&start_v).cloned() else {
            continue;
        };
        for first_ei in inc_list {
            if used_edge[first_ei] {
                continue;
            }
            used_edge[first_ei] = true;
            let mut chain_pts: Vec<DVec2> = Vec::new();
            let mut cur_v = append_oriented_edge(&alive_edges[first_ei], start_v, &mut chain_pts);

            while adj.get(&cur_v).is_some_and(|inc| inc.len() == 2) {
                let next_opt = adj[&cur_v].iter().copied().find(|&ei| !used_edge[ei]);
                let Some(next_ei) = next_opt else {
                    break;
                };
                used_edge[next_ei] = true;
                cur_v = append_oriented_edge(&alive_edges[next_ei], cur_v, &mut chain_pts);
                if cur_v == start_v {
                    break;
                }
            }
            let is_closed = chain_pts.len() >= 3
                && chain_pts
                    .first()
                    .zip(chain_pts.last())
                    .is_some_and(|(a, b)| a.distance(*b) <= 1e-4);
            raw_chains.push((chain_pts, is_closed));
        }
    }

    // 2. Walk any remaining pure degree-2 closed cycles (e.g. annular 'O' medial ring!)
    for start_ei in 0..alive_edges.len() {
        if used_edge[start_ei] {
            continue;
        }
        used_edge[start_ei] = true;
        let start_v = alive_edges[start_ei].u;
        let mut chain_pts: Vec<DVec2> = Vec::new();
        let mut cur_v = append_oriented_edge(&alive_edges[start_ei], start_v, &mut chain_pts);
        while cur_v != start_v {
            let next_opt = adj
                .get(&cur_v)
                .and_then(|inc| inc.iter().copied().find(|&ei| !used_edge[ei]));
            let Some(next_ei) = next_opt else {
                break;
            };
            used_edge[next_ei] = true;
            cur_v = append_oriented_edge(&alive_edges[next_ei], cur_v, &mut chain_pts);
        }
        let is_closed = cur_v == start_v && chain_pts.len() >= 3;
        raw_chains.push((chain_pts, is_closed));
    }

    // Order chains by greedy nearest-endpoint traversal to minimize rapid travel!
    let ordered_chains = order_chains_nearest(raw_chains);

    ordered_chains
        .into_iter()
        .filter_map(|(pts, closed)| {
            let samples: Vec<MedialNode2D> = pts
                .into_iter()
                .map(|p| MedialNode2D {
                    pos_wcs: p,
                    clearance_radius_mm: region.min_boundary_distance(p).max(0.0),
                })
                .collect();
            // Single-pole dimple for isolated circular dots whose spokes pruned to center
            (samples.len() >= 2 || (samples.len() == 1 && samples[0].clearance_radius_mm > 1e-4))
                .then_some(MedialChain2D { samples, closed })
        })
        .collect()
}

fn append_oriented_edge(edge: &GraphEdge, from_v: usize, out: &mut Vec<DVec2>) -> usize {
    let skip = usize::from(!out.is_empty());
    if from_v == edge.u {
        out.extend(edge.pts.iter().skip(skip).copied());
        edge.v
    } else {
        out.extend(edge.pts.iter().rev().skip(skip).copied());
        edge.u
    }
}

fn subdivide_linear_span(p0: DVec2, p1: DVec2, max_step_mm: f64) -> Vec<DVec2> {
    let len = p0.distance(p1);
    let n = ((len / max_step_mm.max(0.05)).ceil() as usize).clamp(1, 256);
    (0..=n)
        .map(|i| p0.lerp(p1, (i as f64) / (n as f64)))
        .collect()
}

fn sample_parabolic_voronoi_edge(
    diagram: &Diagram,
    used_segs: &[(DVec2, DVec2)],
    c0_id: CellIndex,
    c1_id: CellIndex,
    p0: DVec2,
    p1: DVec2,
    max_step_mm: f64,
) -> Option<Vec<DVec2>> {
    let c0 = diagram.cell(c0_id).ok()?;
    let c1 = diagram.cell(c1_id).ok()?;
    let (pt_cell, seg_cell) = if c0.contains_point() && c1.contains_segment() {
        (c0, c1)
    } else if c1.contains_point() && c0.contains_segment() {
        (c1, c0)
    } else {
        return None;
    };

    let focus = {
        let seg = used_segs.get(pt_cell.source_index().usize())?;
        match pt_cell.source_category() {
            SourceCategory::SegmentStart => seg.0,
            SourceCategory::SegmentEnd | SourceCategory::Segment | SourceCategory::SinglePoint => {
                seg.1
            }
        }
    };
    let (l0, l1) = *used_segs.get(seg_cell.source_index().usize())?;
    let t_hat = (l1 - l0).normalize_or_zero();
    if t_hat.length_squared() < 0.5 {
        return None;
    }
    let mut n_hat = DVec2::new(-t_hat.y, t_hat.x);
    let mut p_dist = (focus - l0).dot(n_hat);
    if p_dist < 0.0 {
        n_hat = -n_hat;
        p_dist = -p_dist;
    }
    if p_dist <= 1e-6 {
        return Some(subdivide_linear_span(p0, p1, max_step_mm));
    }
    let f_proj = focus - n_hat * p_dist;
    let u0 = (p0 - f_proj).dot(t_hat);
    let u1 = (p1 - f_proj).dot(t_hat);
    let span = p0.distance(p1);
    let steps = ((span / max_step_mm.max(0.08)).ceil() as usize).clamp(2, 128);
    let mut out = Vec::with_capacity(steps + 1);
    out.push(p0);
    for k in 1..steps {
        let frac = (k as f64) / (steps as f64);
        let u = u0 + (u1 - u0) * frac;
        let w = (u * u + p_dist * p_dist) / (2.0 * p_dist);
        out.push(f_proj + t_hat * u + n_hat * w);
    }
    out.push(p1);
    Some(out)
}

fn order_chains_nearest(mut chains: Vec<(Vec<DVec2>, bool)>) -> Vec<(Vec<DVec2>, bool)> {
    if chains.len() <= 1 {
        return chains;
    }
    let mut out = Vec::with_capacity(chains.len());
    let first = chains.remove(0);
    let mut cur_pos = first.0.last().copied().unwrap_or(DVec2::ZERO);
    out.push(first);

    while !chains.is_empty() {
        let mut best_idx = 0usize;
        let mut best_rev = false;
        let mut best_d2 = f64::INFINITY;
        for (i, (pts, closed)) in chains.iter().enumerate() {
            if let Some(&s) = pts.first() {
                let d2 = cur_pos.distance_squared(s);
                if d2 < best_d2 {
                    best_d2 = d2;
                    best_idx = i;
                    best_rev = false;
                }
            }
            if !*closed && let Some(&e) = pts.last() {
                let d2 = cur_pos.distance_squared(e);
                if d2 < best_d2 {
                    best_d2 = d2;
                    best_idx = i;
                    best_rev = true;
                }
            }
        }
        let (mut chosen_pts, closed) = chains.swap_remove(best_idx);
        if best_rev {
            chosen_pts.reverse();
        }
        if let Some(&last) = chosen_pts.last() {
            cur_pos = last;
        }
        out.push((chosen_pts, closed));
    }
    out
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
    fn medial_axis_extracts_exact_pole_and_corner_bisectors_for_rectangle_and_annulus() {
        // 40 x 20 mm rectangle: inscribed radius == 10.0 mm along the spine y=10, x in [10..30]
        let rect = PocketRegion2D::new(rect_ccw(0.0, 0.0, 40.0, 20.0), Vec::new());
        let pole = find_medial_inscribed_pole(&rect).unwrap();
        assert!(approx_eq(pole.max_helix_radius_mm, 10.0, 1e-2));
        assert!(approx_eq(pole.center_xy.y, 10.0, 0.1));

        let chains = extract_medial_axis_chains(&rect, 0.05);
        assert!(!chains.is_empty());
        let max_clr = chains
            .iter()
            .flat_map(|c| c.samples.iter())
            .map(|n| n.clearance_radius_mm)
            .fold(0.0_f64, f64::max);
        assert!(approx_eq(max_clr, 10.0, 5e-2));

        // Annular 'O' ring (outer R=15, inner R=7): medial ring at R=11 with clearance == 4.0 mm
        let ring = PocketRegion2D::new(
            circle_ccw(DVec2::ZERO, 15.0),
            vec![circle_ccw(DVec2::ZERO, 7.0)],
        );
        let ring_chains = extract_medial_axis_chains(&ring, 0.05);
        assert!(!ring_chains.is_empty());
        assert!(
            ring_chains.iter().any(|c| c.closed),
            "annulus medial axis must contain a closed medial cycle"
        );
        let ring_pole = find_medial_inscribed_pole(&ring).unwrap();
        assert!(approx_eq(ring_pole.max_helix_radius_mm, 4.0, 0.05));
    }
}
