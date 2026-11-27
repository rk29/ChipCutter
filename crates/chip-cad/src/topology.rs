use std::{cmp::Ordering, collections::BinaryHeap};

use chip_math::{COINCIDENCE_TOL_MM, DVec3};

use crate::model::{CadDocument, CurveKind, EdgeId, FaceId, OrientedEdgeUse, SurfaceKind};

/// Welded topological vertex identifier inside a [`TopologyGraph`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VertexId(pub u32);

/// Start/end vertex and unit tangent metadata for a single [`EdgeId`].
#[derive(Debug, Clone, Copy)]
pub struct EdgeEndpointInfo {
    pub start_vertex: VertexId,
    pub end_vertex: VertexId,
    pub start_pos_mm: DVec3,
    pub end_pos_mm: DVec3,
    /// Unit tangent pointing away from `start_pos_mm` along forward edge travel.
    pub start_tangent: DVec3,
    /// Unit tangent arriving at `end_pos_mm` along forward edge travel.
    pub end_tangent: DVec3,
    pub is_closed: bool,
}

/// Axial span and blind-vs-through classification for an internal cylindrical hole along a WCS `+Z` axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylindricalHoleSpan {
    /// Uppermost rim projection (`p · z_axis_world`) in millimeters.
    pub top_proj_mm: f64,
    /// Lowermost rim projection (`p · z_axis_world`) in millimeters.
    pub bottom_proj_mm: f64,
    /// `true` if the hole opens through the bottom/exterior rather than terminating on an upward-facing floor.
    pub is_through_hole: bool,
}

/// Logical cylindrical bore grouping coaxial split B-Rep cylinder faces (e.g., two `180°` STEP half-cylinders)
/// so viewport highlighting covers the full `360°` wall and CAM operations never drill the same hole twice.
#[derive(Debug, Clone, PartialEq)]
pub struct LogicalCylindricalHole {
    pub primary_face: FaceId,
    pub faces: Vec<FaceId>,
    pub axis_origin_mm: DVec3,
    pub axis_dir: DVec3,
    pub radius_mm: f64,
    pub span: CylindricalHoleSpan,
}

/// Welded vertex-edge incidence graph enabling open-chain pathfinding and adjacency queries.
#[derive(Debug, Clone, Default)]
pub struct TopologyGraph {
    pub vertices_mm: Vec<DVec3>,
    pub vertex_edges: Vec<Vec<EdgeId>>,
    pub edge_endpoints: Vec<Option<EdgeEndpointInfo>>,
}

#[derive(Clone, Copy)]
struct ChainState {
    cost: f64,
    node: usize,
}

impl PartialEq for ChainState {
    fn eq(&self, other: &Self) -> bool {
        self.cost.to_bits() == other.cost.to_bits() && self.node == other.node
    }
}
impl Eq for ChainState {}

impl Ord for ChainState {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap on `cost`
        other
            .cost
            .total_cmp(&self.cost)
            .then_with(|| other.node.cmp(&self.node))
    }
}

impl PartialOrd for ChainState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl TopologyGraph {
    /// Builds a welded vertex-edge topology graph across all edges in `doc`.
    pub fn build(doc: &CadDocument) -> Self {
        let mut vertices_mm: Vec<DVec3> = Vec::new();
        let mut vertex_edges: Vec<Vec<EdgeId>> = Vec::new();
        let mut edge_endpoints = vec![None; doc.edges.len()];

        let mut poly_by_edge: Vec<Option<&[DVec3]>> = vec![None; doc.edges.len()];
        for ep in &doc.render_mesh.edge_polylines {
            if let Some(slot) = poly_by_edge.get_mut(ep.edge_id.0 as usize) {
                *slot = Some(ep.points_mm.as_slice());
            }
        }

        for edge in &doc.edges {
            let idx = edge.id.0 as usize;
            let poly = poly_by_edge.get(idx).copied().flatten();
            let Some((p0, p1, t0, t1, is_closed)) = extract_edge_kinematics(&edge.curve, poly)
            else {
                continue;
            };
            let v0 = weld_vertex(&mut vertices_mm, &mut vertex_edges, p0);
            let v1 = if is_closed {
                v0
            } else {
                weld_vertex(&mut vertices_mm, &mut vertex_edges, p1)
            };

            if let Some(list) = vertex_edges.get_mut(v0.0 as usize)
                && !list.contains(&edge.id)
            {
                list.push(edge.id);
            }
            if v1 != v0
                && let Some(list) = vertex_edges.get_mut(v1.0 as usize)
                && !list.contains(&edge.id)
            {
                list.push(edge.id);
            }

            if let Some(slot) = edge_endpoints.get_mut(idx) {
                *slot = Some(EdgeEndpointInfo {
                    start_vertex: v0,
                    end_vertex: v1,
                    start_pos_mm: p0,
                    end_pos_mm: p1,
                    start_tangent: t0,
                    end_tangent: t1,
                    is_closed,
                });
            }
        }

        Self {
            vertices_mm,
            vertex_edges,
            edge_endpoints,
        }
    }

    #[inline]
    pub fn endpoint_info(&self, edge_id: EdgeId) -> Option<EdgeEndpointInfo> {
        self.edge_endpoints
            .get(edge_id.0 as usize)
            .copied()
            .flatten()
    }

    /// Finds the optimal connected open (or closed) chain of oriented B-Rep edges from `start_edge`
    /// to `end_edge` on the same body.
    ///
    /// Uses Dijkstra over directed edge states `(EdgeId, forward)` with a tangent-kink + plane-change
    /// penalty so smooth `G1` filleted contours are strongly preferred over branching off onto vertical walls.
    pub fn find_open_edge_chain(
        &self,
        doc: &CadDocument,
        start_edge: EdgeId,
        end_edge: EdgeId,
    ) -> Option<Vec<OrientedEdgeUse>> {
        let start_rec = doc.edge(start_edge)?;
        let end_rec = doc.edge(end_edge)?;
        if start_rec.body_id != end_rec.body_id {
            return None;
        }
        if start_edge == end_edge {
            return Some(vec![OrientedEdgeUse {
                edge_id: start_edge,
                forward: true,
            }]);
        }

        let n_edges = doc.edges.len();
        let n_states = n_edges * 2; // state = edge_idx * 2 + (forward as usize)
        if n_states == 0 {
            return None;
        }

        let mut dist = vec![f64::INFINITY; n_states];
        let mut prev: Vec<Option<usize>> = vec![None; n_states];
        let mut heap = BinaryHeap::new();

        for fwd in [true, false] {
            let s_idx = (start_edge.0 as usize) * 2 + usize::from(fwd);
            if self.endpoint_info(start_edge).is_some() {
                let init_cost = start_rec.length_mm.max(1e-3);
                dist[s_idx] = init_cost;
                heap.push(ChainState {
                    cost: init_cost,
                    node: s_idx,
                });
            }
        }

        let mut best_goal_state: Option<usize> = None;

        while let Some(ChainState { cost, node }) = heap.pop() {
            if cost > dist[node] + 1e-12 {
                continue;
            }
            let cur_edge = EdgeId((node / 2) as u32);
            let cur_fwd = (node % 2) == 1;

            if cur_edge == end_edge {
                best_goal_state = Some(node);
                break;
            }

            let Some(cur_ep) = self.endpoint_info(cur_edge) else {
                continue;
            };
            let Some(cur_rec) = doc.edge(cur_edge) else {
                continue;
            };

            let (arrive_v, arrive_tan) = if cur_fwd {
                (cur_ep.end_vertex, cur_ep.end_tangent)
            } else {
                (cur_ep.start_vertex, -cur_ep.start_tangent)
            };

            let Some(incident) = self.vertex_edges.get(arrive_v.0 as usize) else {
                continue;
            };

            for &next_edge in incident {
                if next_edge == cur_edge {
                    continue;
                }
                let Some(next_rec) = doc.edge(next_edge) else {
                    continue;
                };
                if next_rec.body_id != start_rec.body_id {
                    continue;
                }
                let Some(next_ep) = self.endpoint_info(next_edge) else {
                    continue;
                };
                if next_ep.is_closed {
                    continue;
                }

                // Candidate orientations of `next_edge` departing from `arrive_v`
                let mut candidates = Vec::with_capacity(2);
                if next_ep.start_vertex == arrive_v {
                    candidates.push((true, next_ep.start_tangent));
                }
                if next_ep.end_vertex == arrive_v {
                    candidates.push((false, -next_ep.end_tangent));
                }

                let shares_planar_face = shares_common_plane(doc, cur_rec, next_rec);

                for (next_fwd, depart_tan) in candidates {
                    let cos_kink = arrive_tan.dot(depart_tan).clamp(-1.0, 1.0);
                    // Reject direct 180° hairpin foldbacks
                    if cos_kink < -0.999 {
                        continue;
                    }
                    let kink_angle = cos_kink.acos();
                    let plane_penalty = if shares_planar_face { 0.0 } else { 18.0 };
                    let step_cost =
                        next_rec.length_mm.max(1e-3) + kink_angle * 25.0 + plane_penalty;

                    let next_node = (next_edge.0 as usize) * 2 + usize::from(next_fwd);
                    let next_cost = cost + step_cost;
                    if next_cost + 1e-12 < dist[next_node] {
                        dist[next_node] = next_cost;
                        prev[next_node] = Some(node);
                        heap.push(ChainState {
                            cost: next_cost,
                            node: next_node,
                        });
                    }
                }
            }
        }

        let goal = best_goal_state?;
        let mut chain = Vec::new();
        let mut cur = Some(goal);
        let mut guard = 0usize;
        while let Some(st) = cur {
            if guard > n_states {
                return None;
            }
            guard += 1;
            chain.push(OrientedEdgeUse {
                edge_id: EdgeId((st / 2) as u32),
                forward: (st % 2) == 1,
            });
            cur = prev[st];
        }
        chain.reverse();
        Some(chain)
    }
}

fn weld_vertex(
    vertices_mm: &mut Vec<DVec3>,
    vertex_edges: &mut Vec<Vec<EdgeId>>,
    pt: DVec3,
) -> VertexId {
    for (idx, existing) in vertices_mm.iter().enumerate() {
        if existing.distance(pt) <= COINCIDENCE_TOL_MM * 2.5 {
            return VertexId(idx as u32);
        }
    }
    let id = VertexId(vertices_mm.len() as u32);
    vertices_mm.push(pt);
    vertex_edges.push(Vec::new());
    id
}

fn shares_common_plane(
    doc: &CadDocument,
    e1: &crate::model::BRepEdge,
    e2: &crate::model::BRepEdge,
) -> bool {
    for &f1 in &e1.adjacent_faces {
        if e2.adjacent_faces.contains(&f1)
            && let Some(face) = doc.face(f1)
            && matches!(face.surface, SurfaceKind::Plane { .. })
        {
            return true;
        }
    }
    false
}

fn extract_edge_kinematics(
    curve: &CurveKind,
    poly: Option<&[DVec3]>,
) -> Option<(DVec3, DVec3, DVec3, DVec3, bool)> {
    match *curve {
        CurveKind::Line { start_mm, end_mm } => {
            let dir = end_mm - start_mm;
            let len = dir.length();
            if len < 1e-12 {
                return None;
            }
            let t = dir / len;
            // Align orientation with discretized polyline if reversed during STEP trimming
            if let Some(pts) = poly
                && let Some(&first) = pts.first()
                && first.distance(end_mm) < first.distance(start_mm)
            {
                return Some((end_mm, start_mm, -t, -t, false));
            }
            Some((start_mm, end_mm, t, t, false))
        }
        CurveKind::CircleArc {
            center_mm,
            normal,
            start_mm,
            end_mm,
            is_full_circle,
            ..
        } => {
            if is_full_circle {
                let r_vec = (start_mm - center_mm).normalize_or_zero();
                let tan = normal.cross(r_vec).normalize_or_zero();
                return Some((start_mm, start_mm, tan, tan, true));
            }
            // Use polyline orientation to ensure curve sense matches trimmed edge endpoints
            if let Some(pts) = poly
                && pts.len() >= 2
            {
                let p0 = pts[0];
                let p1 = pts[pts.len() - 1];
                let t0 = (pts[1] - pts[0]).normalize_or_zero();
                let t1 = (pts[pts.len() - 1] - pts[pts.len() - 2]).normalize_or_zero();
                if t0.length_squared() > 0.5 && t1.length_squared() > 0.5 {
                    return Some((p0, p1, t0, t1, false));
                }
            }
            let r0 = (start_mm - center_mm).normalize_or_zero();
            let r1 = (end_mm - center_mm).normalize_or_zero();
            let t0 = normal.cross(r0).normalize_or_zero();
            let t1 = normal.cross(r1).normalize_or_zero();
            Some((start_mm, end_mm, t0, t1, false))
        }
        CurveKind::EllipseArc | CurveKind::OtherNurbs => {
            let pts = poly?;
            if pts.len() < 2 {
                return None;
            }
            let p0 = pts[0];
            let p1 = pts[pts.len() - 1];
            let is_closed = p0.distance(p1) <= COINCIDENCE_TOL_MM;
            let t0 = (pts[1] - pts[0]).normalize_or_zero();
            let t1 = (pts[pts.len() - 1] - pts[pts.len() - 2]).normalize_or_zero();
            Some((p0, p1, t0, t1, is_closed))
        }
    }
}

/// Finds all internal cylindrical hole faces on the same body sharing `seed_face_id`'s axis orientation
/// (parallel within `axis_tol_rad`) and radius (within `radius_tol_mm`).
pub fn find_same_diameter_holes(
    doc: &CadDocument,
    seed_face_id: FaceId,
    axis_tol_rad: f64,
    radius_tol_mm: f64,
) -> Vec<FaceId> {
    let Some(seed) = doc.face(seed_face_id) else {
        return Vec::new();
    };
    let SurfaceKind::Cylinder {
        axis_dir: seed_axis,
        radius_mm: seed_radius,
        is_internal: true,
        ..
    } = seed.surface
    else {
        return Vec::new();
    };

    let min_cos = axis_tol_rad.cos();
    doc.faces
        .iter()
        .filter_map(|f| {
            if f.body_id != seed.body_id {
                return None;
            }
            if let SurfaceKind::Cylinder {
                axis_dir,
                radius_mm,
                is_internal: true,
                ..
            } = f.surface
                && seed_axis.dot(axis_dir).abs() >= min_cos
                && (radius_mm - seed_radius).abs() <= radius_tol_mm
            {
                Some(f.id)
            } else {
                None
            }
        })
        .collect()
}

/// Finds all planar faces on the same body coplanar with `seed_face_id` (matching outward normal
/// direction within `normal_tol_rad` and signed plane offset within `height_tol_mm`).
pub fn find_coplanar_faces(
    doc: &CadDocument,
    seed_face_id: FaceId,
    normal_tol_rad: f64,
    height_tol_mm: f64,
) -> Vec<FaceId> {
    let Some(seed) = doc.face(seed_face_id) else {
        return Vec::new();
    };
    let SurfaceKind::Plane {
        origin_mm: seed_origin,
        normal: seed_normal,
    } = seed.surface
    else {
        return Vec::new();
    };

    let seed_n = seed_normal.normalize_or_zero();
    let seed_d = seed_n.dot(seed_origin);
    let min_cos = normal_tol_rad.cos();

    doc.faces
        .iter()
        .filter_map(|f| {
            if f.body_id != seed.body_id {
                return None;
            }
            if let SurfaceKind::Plane { origin_mm, normal } = f.surface {
                let n = normal.normalize_or_zero();
                if seed_n.dot(n) >= min_cos
                    && (seed_n.dot(origin_mm) - seed_d).abs() <= height_tol_mm
                {
                    return Some(f.id);
                }
            }
            None
        })
        .collect()
}

/// Computes the exact axial projection span (`top_proj_mm`, `bottom_proj_mm`) along unit vector
/// `spindle_up_world` and classifies whether the hole breaks through or terminates on a blind floor.
pub fn analyze_cylindrical_hole(
    doc: &CadDocument,
    cyl_face_id: FaceId,
    spindle_up_world: DVec3,
) -> Option<CylindricalHoleSpan> {
    let face = doc.face(cyl_face_id)?;
    if !matches!(
        face.surface,
        SurfaceKind::Cylinder {
            is_internal: true,
            ..
        }
    ) {
        return None;
    }
    let up = spindle_up_world.normalize_or_zero();
    if up.length_squared() < 0.5 {
        return None;
    }

    let mut min_proj = f64::INFINITY;
    let mut max_proj = f64::NEG_INFINITY;
    let mut face_edges = Vec::new();

    for loop_id in face.outer_loop.iter().chain(face.inner_loops.iter()) {
        if let Some(lp) = doc.brep_loop(*loop_id) {
            for use_rec in &lp.edges {
                if !face_edges.contains(&use_rec.edge_id) {
                    face_edges.push(use_rec.edge_id);
                }
                if let Some(edge) = doc.edge(use_rec.edge_id) {
                    match edge.curve {
                        CurveKind::Line { start_mm, end_mm } => {
                            for p in [start_mm, end_mm] {
                                let pr = p.dot(up);
                                min_proj = min_proj.min(pr);
                                max_proj = max_proj.max(pr);
                            }
                        }
                        CurveKind::CircleArc {
                            center_mm,
                            normal,
                            radius_mm,
                            is_full_circle,
                            start_mm,
                            end_mm,
                        } => {
                            if is_full_circle {
                                let sin_tilt = (1.0 - normal.normalize_or_zero().dot(up).powi(2))
                                    .max(0.0)
                                    .sqrt();
                                let c_proj = center_mm.dot(up);
                                min_proj = min_proj.min(c_proj - radius_mm * sin_tilt);
                                max_proj = max_proj.max(c_proj + radius_mm * sin_tilt);
                            } else {
                                for p in [start_mm, end_mm] {
                                    let pr = p.dot(up);
                                    min_proj = min_proj.min(pr);
                                    max_proj = max_proj.max(pr);
                                }
                            }
                        }
                        CurveKind::EllipseArc | CurveKind::OtherNurbs => {}
                    }
                }
            }
        }
    }

    // Also include discretized edge polyline vertices for general trimmed cylinders
    for ep in &doc.render_mesh.edge_polylines {
        if face_edges.contains(&ep.edge_id) {
            for &pt in &ep.points_mm {
                let pr = pt.dot(up);
                min_proj = min_proj.min(pr);
                max_proj = max_proj.max(pr);
            }
        }
    }

    if !min_proj.is_finite() || !max_proj.is_finite() {
        return None;
    }

    // Check faces bordering the bottom rim (`proj ≈ min_proj`): if an adjacent face is an
    // upward-pointing floor (`normal · up > 0.35`) or conical drill point at the bottom, it's blind!
    let mut has_blind_floor = false;
    let rim_band = 0.08_f64;
    for &eid in &face_edges {
        let Some(edge) = doc.edge(eid) else {
            continue;
        };
        let edge_center_proj = edge.aabb_mm.center().dot(up);
        let edge_min_proj = [
            DVec3::new(edge.aabb_mm.min.x, edge.aabb_mm.min.y, edge.aabb_mm.min.z),
            DVec3::new(edge.aabb_mm.max.x, edge.aabb_mm.max.y, edge.aabb_mm.max.z),
        ]
        .into_iter()
        .map(|p| p.dot(up))
        .fold(f64::INFINITY, f64::min);

        if (edge_center_proj - min_proj).abs() <= rim_band
            || (edge_min_proj - min_proj).abs() <= rim_band
                && (edge.aabb_mm.size().dot(up.abs())) <= rim_band * 2.0
        {
            for &adj_fid in &edge.adjacent_faces {
                if adj_fid == cyl_face_id {
                    continue;
                }
                if let Some(adj_face) = doc.face(adj_fid) {
                    match adj_face.surface {
                        SurfaceKind::Plane { normal, .. }
                            if normal.normalize_or_zero().dot(up) > 0.35 =>
                        {
                            has_blind_floor = true;
                        }
                        SurfaceKind::Cone { apex_mm, .. }
                            if apex_mm.dot(up) <= min_proj + rim_band =>
                        {
                            has_blind_floor = true;
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    Some(CylindricalHoleSpan {
        top_proj_mm: max_proj,
        bottom_proj_mm: min_proj,
        is_through_hole: !has_blind_floor,
    })
}

/// Expands an internal cylindrical `FaceId` to all coaxial internal cylinder faces sharing the
/// same bore (for example, when a STEP exporter splits a `360°` hole into two `180°` half-cylinders).
pub fn expand_coaxial_hole_faces(doc: &CadDocument, seed_face_id: FaceId) -> Vec<FaceId> {
    let Some(seed) = doc.face(seed_face_id) else {
        return vec![seed_face_id];
    };
    let SurfaceKind::Cylinder {
        axis_origin_mm: seed_orig,
        axis_dir: seed_dir,
        radius_mm: seed_r,
        is_internal: true,
    } = seed.surface
    else {
        return vec![seed_face_id];
    };
    let u = seed_dir.normalize_or_zero();
    if u.length_squared() < 0.5 {
        return vec![seed_face_id];
    }
    let Some(seed_span) = analyze_cylindrical_hole(doc, seed_face_id, u) else {
        return vec![seed_face_id];
    };

    let mut out = Vec::new();
    for f in &doc.faces {
        if f.body_id != seed.body_id {
            continue;
        }
        if let SurfaceKind::Cylinder {
            axis_origin_mm,
            axis_dir,
            radius_mm,
            is_internal: true,
        } = f.surface
        {
            if (radius_mm - seed_r).abs() > 1e-3 {
                continue;
            }
            let dir_n = axis_dir.normalize_or_zero();
            if u.dot(dir_n).abs() < 0.9999 {
                continue;
            }
            let delta = axis_origin_mm - seed_orig;
            let perp_dist = (delta - u * delta.dot(u)).length();
            if perp_dist > 1e-2 {
                continue;
            }
            if let Some(sp) = analyze_cylindrical_hole(doc, f.id, u) {
                let overlap = sp.top_proj_mm.min(seed_span.top_proj_mm)
                    - sp.bottom_proj_mm.max(seed_span.bottom_proj_mm);
                if overlap >= -1e-2 {
                    out.push(f.id);
                }
            }
        }
    }
    if out.is_empty() {
        out.push(seed_face_id);
    }
    out
}

/// Deduplicates a list of internal cylindrical `FaceId`s into distinct coaxial [`LogicalCylindricalHole`]s.
pub fn group_coaxial_holes(
    doc: &CadDocument,
    face_ids: &[FaceId],
    spindle_up_world: DVec3,
) -> Vec<LogicalCylindricalHole> {
    let mut visited = Vec::<FaceId>::new();
    let mut holes = Vec::new();

    for &fid in face_ids {
        if visited.contains(&fid) {
            continue;
        }
        let Some(face) = doc.face(fid) else {
            continue;
        };
        let SurfaceKind::Cylinder {
            axis_origin_mm,
            axis_dir,
            radius_mm,
            is_internal: true,
        } = face.surface
        else {
            continue;
        };

        let group = expand_coaxial_hole_faces(doc, fid);
        let mut top_proj = f64::NEG_INFINITY;
        let mut bot_proj = f64::INFINITY;
        let mut all_through = true;

        for &gfid in &group {
            if !visited.contains(&gfid) {
                visited.push(gfid);
            }
            if let Some(sp) = analyze_cylindrical_hole(doc, gfid, spindle_up_world) {
                top_proj = top_proj.max(sp.top_proj_mm);
                bot_proj = bot_proj.min(sp.bottom_proj_mm);
                if !sp.is_through_hole {
                    all_through = false;
                }
            }
        }

        if top_proj.is_finite() && bot_proj.is_finite() {
            holes.push(LogicalCylindricalHole {
                primary_face: group[0],
                faces: group,
                axis_origin_mm,
                axis_dir: axis_dir.normalize_or_zero(),
                radius_mm,
                span: CylindricalHoleSpan {
                    top_proj_mm: top_proj,
                    bottom_proj_mm: bot_proj,
                    is_through_hole: all_through,
                },
            });
        }
    }

    holes
}
