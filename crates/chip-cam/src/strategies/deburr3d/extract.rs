use std::collections::{HashMap, HashSet};

use chip_cad::{
    BRepEdge, BRepFace, CadDocument, CurveKind, EdgeId, FaceId, LoopId, OrientedEdgeUse,
    SurfaceKind, group_coaxial_holes,
};
use chip_math::{COINCIDENCE_TOL_MM, DVec2, DVec3, RigidFrame3};
use chip_tools::CutterGeometry;

use crate::{
    params::Deburr3DParams, selection::GeometrySelectionItem, strategies::StrategyExecContext,
    wcs::CamError,
};

use super::{DeburrEdgeChain, DeburrSample3D};

/// Pre-indexed target B-Rep triangle lookup by `FaceId` (world coordinates) for exact normal
/// fallback and geometric convexity checks.
pub(super) struct FaceTriLookup<'a> {
    by_face: HashMap<FaceId, Vec<usize>>,
    doc: &'a CadDocument,
}

impl<'a> FaceTriLookup<'a> {
    pub(super) fn build(doc: &'a CadDocument) -> Self {
        let mut by_face: HashMap<FaceId, Vec<usize>> = HashMap::new();
        for (idx, tri) in doc.render_mesh.pick_triangles.iter().enumerate() {
            by_face.entry(tri.face_id).or_default().push(idx);
        }
        Self { by_face, doc }
    }

    /// Evaluates the exact outward unit normal of `face_id` at world point `p_world`.
    fn outward_normal_world(&self, face_id: FaceId, p_world: DVec3) -> Option<DVec3> {
        let face = self.doc.face(face_id)?;
        match face.surface {
            SurfaceKind::Plane { normal, .. } => {
                let n = normal.normalize_or_zero();
                (n.length_squared() > 0.5).then_some(n)
            }
            SurfaceKind::Cylinder {
                axis_origin_mm,
                axis_dir,
                is_internal,
                ..
            } => {
                let a = axis_dir.normalize_or_zero();
                let v = p_world - axis_origin_mm;
                let radial = (v - a * v.dot(a)).normalize_or_zero();
                if radial.length_squared() < 0.5 {
                    return self.nearest_triangle_normal_world(face_id, p_world);
                }
                Some(if is_internal { -radial } else { radial })
            }
            SurfaceKind::Cone {
                apex_mm,
                axis_dir,
                half_angle_rad,
            } => {
                let a = axis_dir.normalize_or_zero();
                let v = p_world - apex_mm;
                let radial = (v - a * v.dot(a)).normalize_or_zero();
                if radial.length_squared() < 0.5 {
                    return self.nearest_triangle_normal_world(face_id, p_world);
                }
                let axial_sign = if v.dot(a) >= 0.0 { 1.0 } else { -1.0 };
                let (sin_h, cos_h) = half_angle_rad.sin_cos();
                let mut n = (radial * cos_h - a * (axial_sign * sin_h)).normalize_or_zero();
                if let Some(tri_n) = self.nearest_triangle_normal_world(face_id, p_world)
                    && n.dot(tri_n) < 0.0
                {
                    n = -n;
                }
                (n.length_squared() > 0.5).then_some(n)
            }
            SurfaceKind::Torus {
                center_mm,
                axis_dir,
                major_radius_mm,
                ..
            } => {
                let a = axis_dir.normalize_or_zero();
                let v = p_world - center_mm;
                let proj = (v - a * v.dot(a)).normalize_or_zero();
                if proj.length_squared() < 0.5 {
                    return self.nearest_triangle_normal_world(face_id, p_world);
                }
                let major_pt = center_mm + proj * major_radius_mm;
                let mut n = (p_world - major_pt).normalize_or_zero();
                if let Some(tri_n) = self.nearest_triangle_normal_world(face_id, p_world)
                    && n.dot(tri_n) < 0.0
                {
                    n = -n;
                }
                (n.length_squared() > 0.5).then_some(n)
            }
            SurfaceKind::OtherNurbs => self.nearest_triangle_normal_world(face_id, p_world),
        }
    }

    fn nearest_triangle_normal_world(&self, face_id: FaceId, p_world: DVec3) -> Option<DVec3> {
        let tris = self.by_face.get(&face_id)?;
        let mut best_d2 = f64::INFINITY;
        let mut best_n = DVec3::ZERO;
        for &idx in tris {
            let t = self.doc.render_mesh.pick_triangles[idx];
            let cp = chip_bvh::closest_point_on_triangle(p_world, t.v0_mm, t.v1_mm, t.v2_mm).0;
            let d2 = cp.distance_squared(p_world);
            if d2 < best_d2 {
                let n = (t.v1_mm - t.v0_mm)
                    .cross(t.v2_mm - t.v0_mm)
                    .normalize_or_zero();
                if n.length_squared() > 0.5 {
                    best_d2 = d2;
                    best_n = n;
                }
            }
        }
        (best_n.length_squared() > 0.5).then_some(best_n)
    }

    /// Returns a world-space point strictly inside `face_id` near `p_world` (displaced away from
    /// the edge tangent `edge_tan_world`) to determine exact edge convexity vs. concavity.
    fn interior_witness_point_world(
        &self,
        face_id: FaceId,
        p_world: DVec3,
        edge_tan_world: DVec3,
    ) -> Option<DVec3> {
        let tris = self.by_face.get(&face_id)?;
        let mut best_score = f64::INFINITY;
        let mut best_q = None;

        for &idx in tris {
            let t = self.doc.render_mesh.pick_triangles[idx];
            let cp = chip_bvh::closest_point_on_triangle(p_world, t.v0_mm, t.v1_mm, t.v2_mm).0;
            let dist_to_edge_pt = cp.distance(p_world);
            let centroid = (t.v0_mm + t.v1_mm + t.v2_mm) / 3.0;
            let delta = centroid - p_world;
            let perp = delta - edge_tan_world * delta.dot(edge_tan_world);
            if perp.length_squared() < 1e-10 {
                continue;
            }
            if dist_to_edge_pt < best_score {
                best_score = dist_to_edge_pt;
                best_q = Some(centroid);
            }
        }
        best_q
    }
}

/// Automatically detects all sharp convex 3D B-Rep edges accessible to the current tool and chains
/// connected edges into maximal continuous 3D paths.
pub(super) fn auto_detect_sharp_edge_chains(
    ctx: &StrategyExecContext<'_>,
    params: &Deburr3DParams,
    lookup: &FaceTriLookup<'_>,
    poly_by_edge: &HashMap<EdgeId, &[DVec3]>,
    chord_tol_mm: f64,
) -> Result<Vec<DeburrEdgeChain>, CamError> {
    let cos_min_dihedral = params.min_dihedral_angle_deg.to_radians().cos();
    let allow_underside = params.allow_underside_back_chamfer
        && matches!(ctx.tool.geometry, CutterGeometry::LollipopEndmill { .. });
    let model_min_z = ctx.setup.model_aabb_wcs.min.z;

    let mut candidate_segments: Vec<Vec<DeburrSample3D>> = Vec::new();
    let mut closed_chains: Vec<DeburrEdgeChain> = Vec::new();

    for edge in &ctx.doc.edges {
        ctx.check_cancel()?;
        let Some((f1, f2)) = distinct_adjacent_face_pair(edge) else {
            continue;
        };
        let Some(samples) = sample_brep_edge_3d(
            ctx.doc,
            edge,
            true,
            f1,
            f2,
            &ctx.setup.frame,
            lookup,
            poly_by_edge.get(&edge.id).copied(),
            chord_tol_mm,
        ) else {
            continue;
        };
        if samples.len() < 2 {
            continue;
        }

        // Check convexity, dihedral angle, vertical filter, and top/underside accessibility
        let mid = &samples[samples.len() / 2];
        let mid_world = ctx.setup.frame.local_to_world_point(mid.pos_wcs);
        let tan_world = ctx.setup.frame.local_to_world_dir(mid.tangent);
        let n1_world = ctx.setup.frame.local_to_world_dir(mid.n1);
        let n2_world = ctx.setup.frame.local_to_world_dir(mid.n2);

        if !is_edge_convex_world(lookup, f1, f2, mid_world, tan_world, n1_world, n2_world) {
            continue;
        }

        // Dihedral angle check across samples (skip smooth tangent seams)
        let dot_n = mid.n1.dot(mid.n2).clamp(-1.0, 1.0);
        if dot_n >= cos_min_dihedral {
            continue;
        }

        // Exclude pure vertical edges when requested
        if params.exclude_vertical_edges && samples.iter().all(|s| s.tangent.z.abs() > 0.95) {
            continue;
        }

        // Accessibility classification (top vs underside)
        let is_underside = mid.is_underside;
        if is_underside {
            if !allow_underside {
                continue;
            }
            // Skip bottom resting edges flush with the part bottom (table/vise seating face)
            if samples.iter().all(|s| s.pos_wcs.z <= model_min_z + 0.05) {
                continue;
            }
            if DVec2::new(mid.bisector_out.x, mid.bisector_out.y).length() < 0.15 {
                continue;
            }
        } else if mid.bisector_out.z < 0.05 {
            continue;
        }

        // Respect Operation Z height bounds (`[bottom_z_wcs, top_z_wcs]`)
        let z_ok = samples.iter().any(|s| {
            s.pos_wcs.z >= ctx.heights.bottom_z_wcs - 0.05
                && s.pos_wcs.z <= ctx.heights.top_z_wcs + 0.05
        });
        if !z_ok {
            continue;
        }

        let is_closed = is_sample_list_closed(&samples);
        if is_closed {
            closed_chains.push(DeburrEdgeChain {
                samples,
                closed: true,
                is_underside,
                user_reversed: false,
            });
        } else {
            candidate_segments.push(samples);
        }
    }

    let mut out = closed_chains;
    out.extend(stitch_open_edge_segments(candidate_segments));
    Ok(out)
}

/// Extracts explicitly selected B-Rep loops, open chains, planar face rims, or hole rims into
/// 3D edge chains in Setup WCS.
pub(super) fn extract_selected_edge_chains(
    ctx: &StrategyExecContext<'_>,
    params: &Deburr3DParams,
    lookup: &FaceTriLookup<'_>,
    poly_by_edge: &HashMap<EdgeId, &[DVec3]>,
    chord_tol_mm: f64,
) -> Result<Vec<DeburrEdgeChain>, CamError> {
    let mut chains = Vec::new();
    let allow_underside = params.allow_underside_back_chamfer
        && matches!(ctx.tool.geometry, CutterGeometry::LollipopEndmill { .. });

    for item in &ctx.op.selections {
        ctx.check_cancel()?;
        match item {
            GeometrySelectionItem::ClosedLoop {
                binding, reverse, ..
            } => {
                let lid = LoopId(binding.last_index);
                let lp = ctx
                    .doc
                    .brep_loop(lid)
                    .ok_or(CamError::MissingSelectedEntity {
                        index: binding.last_index,
                    })?;
                if let Some(ch) = build_chain_from_edge_uses(
                    ctx.doc,
                    &lp.edges,
                    true,
                    *reverse,
                    allow_underside,
                    &ctx.setup.frame,
                    lookup,
                    poly_by_edge,
                    chord_tol_mm,
                ) {
                    chains.push(ch);
                }
            }
            GeometrySelectionItem::OpenChain {
                resolved_edges,
                reverse,
                start_edge,
                ..
            } => {
                if resolved_edges.is_empty() {
                    return Err(CamError::MissingSelectedEntity {
                        index: start_edge.last_index,
                    });
                }
                if let Some(ch) = build_chain_from_edge_uses(
                    ctx.doc,
                    resolved_edges,
                    false,
                    *reverse,
                    allow_underside,
                    &ctx.setup.frame,
                    lookup,
                    poly_by_edge,
                    chord_tol_mm,
                ) {
                    chains.push(ch);
                }
            }
            GeometrySelectionItem::PlanarFace { binding } => {
                let fid = FaceId(binding.last_index);
                let face = ctx.doc.face(fid).ok_or(CamError::MissingSelectedEntity {
                    index: binding.last_index,
                })?;
                for lid in face.outer_loop.iter().chain(face.inner_loops.iter()) {
                    if let Some(lp) = ctx.doc.brep_loop(*lid)
                        && let Some(ch) = build_chain_from_edge_uses(
                            ctx.doc,
                            &lp.edges,
                            true,
                            false,
                            allow_underside,
                            &ctx.setup.frame,
                            lookup,
                            poly_by_edge,
                            chord_tol_mm,
                        )
                    {
                        chains.push(ch);
                    }
                }
            }
            GeometrySelectionItem::CylindricalHole { binding } => {
                let fid = FaceId(binding.last_index);
                let grouped = group_coaxial_holes(ctx.doc, &[fid], ctx.setup.frame.z_axis);
                let mut seen_edges = HashSet::new();
                let mut hole_segs = Vec::new();
                for h in grouped {
                    for gfid in h.faces {
                        let Some(face) = ctx.doc.face(gfid) else {
                            continue;
                        };
                        for lid in face.outer_loop.iter().chain(face.inner_loops.iter()) {
                            let Some(lp) = ctx.doc.brep_loop(*lid) else {
                                continue;
                            };
                            for u in &lp.edges {
                                if !seen_edges.insert(u.edge_id) {
                                    continue;
                                }
                                let Some(edge) = ctx.doc.edge(u.edge_id) else {
                                    continue;
                                };
                                let Some((f1, f2)) = distinct_adjacent_face_pair(edge) else {
                                    continue;
                                };
                                let Some(samples) = sample_brep_edge_3d(
                                    ctx.doc,
                                    edge,
                                    u.forward,
                                    f1,
                                    f2,
                                    &ctx.setup.frame,
                                    lookup,
                                    poly_by_edge.get(&edge.id).copied(),
                                    chord_tol_mm,
                                ) else {
                                    continue;
                                };
                                if samples.len() < 2 {
                                    continue;
                                }
                                let mid = &samples[samples.len() / 2];
                                if mid.is_underside && !allow_underside {
                                    continue;
                                }
                                if !mid.is_underside && mid.bisector_out.z < 0.05 {
                                    continue;
                                }
                                if is_sample_list_closed(&samples) {
                                    chains.push(DeburrEdgeChain {
                                        is_underside: mid.is_underside,
                                        samples,
                                        closed: true,
                                        user_reversed: false,
                                    });
                                } else {
                                    hole_segs.push(samples);
                                }
                            }
                        }
                    }
                }
                chains.extend(stitch_open_edge_segments(hole_segs));
            }
            GeometrySelectionItem::SketchRegion { .. }
            | GeometrySelectionItem::SketchEntity { .. }
            | GeometrySelectionItem::SketchLayerAll { .. } => {}
        }
    }

    Ok(chains)
}

#[allow(clippy::too_many_arguments)]
fn build_chain_from_edge_uses(
    doc: &CadDocument,
    uses: &[OrientedEdgeUse],
    hint_closed: bool,
    reverse: bool,
    allow_underside: bool,
    frame: &RigidFrame3,
    lookup: &FaceTriLookup<'_>,
    poly_by_edge: &HashMap<EdgeId, &[DVec3]>,
    chord_tol_mm: f64,
) -> Option<DeburrEdgeChain> {
    let mut combined: Vec<DeburrSample3D> = Vec::new();
    let mut underside_votes = 0usize;
    let mut total_edges = 0usize;

    for u in uses {
        let edge = doc.edge(u.edge_id)?;
        let (f1, f2) = distinct_adjacent_face_pair(edge)?;
        let samples = sample_brep_edge_3d(
            doc,
            edge,
            u.forward,
            f1,
            f2,
            frame,
            lookup,
            poly_by_edge.get(&edge.id).copied(),
            chord_tol_mm,
        )?;
        if samples.len() < 2 {
            continue;
        }
        let mid = &samples[samples.len() / 2];
        let mid_w = frame.local_to_world_point(mid.pos_wcs);
        let tan_w = frame.local_to_world_dir(mid.tangent);
        let n1_w = frame.local_to_world_dir(mid.n1);
        let n2_w = frame.local_to_world_dir(mid.n2);
        if !is_edge_convex_world(lookup, f1, f2, mid_w, tan_w, n1_w, n2_w) {
            continue;
        }
        if mid.is_underside {
            if !allow_underside {
                continue;
            }
            underside_votes += 1;
        }
        total_edges += 1;
        append_connected_samples(&mut combined, &samples);
    }

    if combined.len() < 2 || total_edges == 0 {
        return None;
    }

    let closed = hint_closed || is_sample_list_closed(&combined);
    let is_underside = underside_votes * 2 > total_edges;

    if reverse {
        reverse_sample_chain(&mut combined);
    }

    Some(DeburrEdgeChain {
        samples: combined,
        closed,
        is_underside,
        user_reversed: reverse,
    })
}

fn distinct_adjacent_face_pair(edge: &BRepEdge) -> Option<(FaceId, FaceId)> {
    let f1 = *edge.adjacent_faces.first()?;
    let f2 = edge.adjacent_faces.iter().copied().find(|&f| f != f1)?;
    Some((f1, f2))
}

fn is_edge_convex_world(
    lookup: &FaceTriLookup<'_>,
    f1: FaceId,
    f2: FaceId,
    p_world: DVec3,
    edge_tan_world: DVec3,
    n1_world: DVec3,
    n2_world: DVec3,
) -> bool {
    let Some(q1) = lookup.interior_witness_point_world(f1, p_world, edge_tan_world) else {
        return false;
    };
    let Some(q2) = lookup.interior_witness_point_world(f2, p_world, edge_tan_world) else {
        return false;
    };
    let v1 = (q1 - p_world).normalize_or_zero();
    let v2 = (q2 - p_world).normalize_or_zero();
    let s1 = v1.dot(n2_world);
    let s2 = v2.dot(n1_world);
    s1 < -1e-3 && s2 < -1e-3
}

/// Samples a single B-Rep edge in exact `f64` Setup WCS coordinates with adaptive chord-sagitta
/// subdivision (`<= chord_tol_mm`), evaluating exact adjacent-face normals `(n1, n2)` at every
/// sample point.
#[allow(clippy::too_many_arguments)]
fn sample_brep_edge_3d(
    doc: &CadDocument,
    edge: &BRepEdge,
    forward: bool,
    f1: FaceId,
    f2: FaceId,
    frame: &RigidFrame3,
    lookup: &FaceTriLookup<'_>,
    poly_pts: Option<&[DVec3]>,
    chord_tol_mm: f64,
) -> Option<Vec<DeburrSample3D>> {
    let tol = chord_tol_mm.clamp(0.0005, 0.25);
    let max_step_mm = 0.5_f64;

    let mut world_pts = match edge.curve {
        CurveKind::Line { start_mm, end_mm } => {
            let mut p0 = start_mm;
            let mut p1 = end_mm;
            if let Some(pts) = poly_pts
                && let Some(&first) = pts.first()
                && first.distance(end_mm) < first.distance(start_mm)
            {
                std::mem::swap(&mut p0, &mut p1);
            }
            let len = p0.distance(p1);
            if len < 1e-6 {
                return None;
            }
            let n = (len / max_step_mm).ceil().max(2.0) as usize;
            (0..=n)
                .map(|i| p0.lerp(p1, (i as f64) / (n as f64)))
                .collect::<Vec<_>>()
        }
        CurveKind::CircleArc {
            center_mm,
            normal,
            radius_mm,
            start_mm,
            end_mm,
            is_full_circle,
        } => sample_circle_arc_3d(
            center_mm,
            normal,
            radius_mm,
            start_mm,
            end_mm,
            is_full_circle,
            poly_pts,
            tol,
            max_step_mm,
        )?,
        CurveKind::EllipseArc | CurveKind::OtherNurbs => {
            let raw = poly_pts?;
            if raw.len() < 2 {
                return None;
            }
            refine_freeform_edge_3d(doc, f1, f2, raw, tol, max_step_mm)
        }
    };

    if !forward {
        world_pts.reverse();
    }
    if world_pts.len() < 2 {
        return None;
    }

    let n_pts = world_pts.len();
    let mut samples = Vec::with_capacity(n_pts);
    for i in 0..n_pts {
        let p_w = world_pts[i];
        let t_w = if i == 0 {
            (world_pts[1] - world_pts[0]).normalize_or_zero()
        } else if i + 1 == n_pts {
            (world_pts[n_pts - 1] - world_pts[n_pts - 2]).normalize_or_zero()
        } else {
            let t_prev = (world_pts[i] - world_pts[i - 1]).normalize_or_zero();
            let t_next = (world_pts[i + 1] - world_pts[i]).normalize_or_zero();
            (t_prev + t_next).normalize_or_zero()
        };
        if t_w.length_squared() < 0.25 {
            continue;
        }
        let n1_w = lookup.outward_normal_world(f1, p_w)?;
        let n2_w = lookup.outward_normal_world(f2, p_w)?;
        let pos_wcs = frame.world_to_local_point(p_w);
        let tangent = frame.world_to_local_dir(t_w).normalize_or_zero();
        let n1 = frame.world_to_local_dir(n1_w).normalize_or_zero();
        let n2 = frame.world_to_local_dir(n2_w).normalize_or_zero();
        let bisector_out = (n1 + n2).normalize_or_zero();
        if bisector_out.length_squared() < 0.25 {
            continue;
        }
        let is_underside = bisector_out.z < -0.15;
        samples.push(DeburrSample3D {
            pos_wcs,
            tangent,
            n1,
            n2,
            bisector_out,
            is_underside,
            face1: f1,
            face2: f2,
        });
    }

    (samples.len() >= 2).then_some(samples)
}

#[allow(clippy::too_many_arguments)]
fn sample_circle_arc_3d(
    center_mm: DVec3,
    normal: DVec3,
    radius_mm: f64,
    start_mm: DVec3,
    end_mm: DVec3,
    is_full_circle: bool,
    poly_pts: Option<&[DVec3]>,
    chord_tol_mm: f64,
    max_step_mm: f64,
) -> Option<Vec<DVec3>> {
    if !radius_mm.is_finite() || radius_mm <= 1e-5 {
        return None;
    }
    let n = normal.normalize_or_zero();
    if n.length_squared() < 0.5 {
        return None;
    }

    // Align start/end endpoints and sweep sense with the trimmed polyline when available
    let (p_start, p_end) = if is_full_circle {
        (start_mm, start_mm)
    } else if let Some(pts) = poly_pts
        && pts.len() >= 2
    {
        (pts[0], pts[pts.len() - 1])
    } else {
        (start_mm, end_mm)
    };

    let u = (p_start - center_mm - n * (p_start - center_mm).dot(n)).normalize_or_zero();
    if u.length_squared() < 0.5 {
        return None;
    }
    let mut v = n.cross(u).normalize_or_zero();

    // Determine exact sweep angle in `[0, 2π]` matching `poly_pts` midpoint if partial arc
    let sweep = if is_full_circle {
        if let Some(pts) = poly_pts
            && pts.len() >= 3
        {
            let d0 = (pts[1] - pts[0]).normalize_or_zero();
            if d0.dot(v) < 0.0 {
                v = -v;
            }
        }
        std::f64::consts::TAU
    } else {
        let r_end = (p_end - center_mm - n * (p_end - center_mm).dot(n)).normalize_or_zero();
        let mut ang_pos = r_end.dot(v).atan2(r_end.dot(u));
        if ang_pos <= 1e-9 {
            ang_pos += std::f64::consts::TAU;
        }
        if let Some(pts) = poly_pts
            && pts.len() >= 3
        {
            let mid_poly = pts[pts.len() / 2];
            let mid_pos =
                center_mm + (u * (0.5 * ang_pos).cos() + v * (0.5 * ang_pos).sin()) * radius_mm;
            let ang_neg = std::f64::consts::TAU - ang_pos;
            let mid_neg =
                center_mm + (u * (0.5 * ang_neg).cos() - v * (0.5 * ang_neg).sin()) * radius_mm;
            if mid_poly.distance_squared(mid_neg) < mid_poly.distance_squared(mid_pos) {
                v = -v;
                ang_neg
            } else {
                ang_pos
            }
        } else {
            ang_pos
        }
    };

    let d_theta_sag = if chord_tol_mm < radius_mm {
        2.0 * (1.0 - chord_tol_mm / radius_mm).clamp(-1.0, 1.0).acos()
    } else {
        0.25
    };
    let d_theta_len = (max_step_mm / radius_mm).clamp(0.01, 0.5);
    let d_theta = d_theta_sag.min(d_theta_len).max(1e-3);
    let n_steps = (sweep / d_theta).ceil().clamp(4.0, 2048.0) as usize;

    let mut pts = Vec::with_capacity(n_steps + 1);
    for i in 0..=n_steps {
        let ang = sweep * (i as f64) / (n_steps as f64);
        let (s, c) = ang.sin_cos();
        pts.push(center_mm + (u * c + v * s) * radius_mm);
    }
    if is_full_circle
        && let Some(&first) = pts.first()
        && let Some(last) = pts.last_mut()
    {
        *last = first;
    }
    Some(pts)
}

/// Refines a freeform/ellipse edge polyline to satisfy `chord_tol_mm` and `max_step_mm`.
/// When the two adjacent B-Rep faces are an analytic `Plane` and `Cylinder`, midpoints are
/// projected onto the exact `Plane ∩ Cylinder` intersection ellipse in `f64`!
fn refine_freeform_edge_3d(
    doc: &CadDocument,
    f1: FaceId,
    f2: FaceId,
    raw: &[DVec3],
    chord_tol_mm: f64,
    max_step_mm: f64,
) -> Vec<DVec3> {
    let plane_cyl = extract_plane_cylinder_pair(doc.face(f1), doc.face(f2));

    let mut work = raw.to_vec();
    if let Some((p_orig, p_norm, c_orig, c_axis, c_rad)) = plane_cyl {
        // Recursive midpoint subdivision projected onto exact `Plane ∩ Cylinder`!
        let mut out = Vec::with_capacity(work.len() * 4);
        if let Some(&first) = work.first() {
            out.push(project_onto_plane_cylinder(
                first, p_orig, p_norm, c_orig, c_axis, c_rad,
            ));
        }
        for seg in work.windows(2) {
            subdivide_plane_cylinder(
                seg[0],
                seg[1],
                p_orig,
                p_norm,
                c_orig,
                c_axis,
                c_rad,
                chord_tol_mm,
                max_step_mm,
                0,
                &mut out,
            );
        }
        return out;
    }

    // General NURBS / ellipse polyline: densify long chords to `<= max_step_mm`
    let mut out = Vec::with_capacity(work.len() * 2);
    if let Some(&first) = work.first() {
        out.push(first);
    }
    for seg in work.windows(2) {
        let len = seg[0].distance(seg[1]);
        if len < 1e-7 {
            continue;
        }
        let n = (len / max_step_mm).ceil().max(1.0) as usize;
        for k in 1..=n {
            out.push(seg[0].lerp(seg[1], (k as f64) / (n as f64)));
        }
    }
    work = out;
    work
}

fn extract_plane_cylinder_pair(
    fa: Option<&BRepFace>,
    fb: Option<&BRepFace>,
) -> Option<(DVec3, DVec3, DVec3, DVec3, f64)> {
    let (fa, fb) = (fa?, fb?);
    match (fa.surface, fb.surface) {
        (
            SurfaceKind::Plane { origin_mm, normal },
            SurfaceKind::Cylinder {
                axis_origin_mm,
                axis_dir,
                radius_mm,
                ..
            },
        )
        | (
            SurfaceKind::Cylinder {
                axis_origin_mm,
                axis_dir,
                radius_mm,
                ..
            },
            SurfaceKind::Plane { origin_mm, normal },
        ) => {
            let n = normal.normalize_or_zero();
            let a = axis_dir.normalize_or_zero();
            if n.dot(a).abs() > 0.05 && radius_mm > 1e-4 {
                Some((origin_mm, n, axis_origin_mm, a, radius_mm))
            } else {
                None
            }
        }
        _ => None,
    }
}

fn project_onto_plane_cylinder(
    pt: DVec3,
    plane_origin: DVec3,
    plane_normal: DVec3,
    cyl_origin: DVec3,
    cyl_axis: DVec3,
    cyl_radius: f64,
) -> DVec3 {
    // 1. Project radially onto exact cylinder surface at radius `cyl_radius`
    let v = pt - cyl_origin;
    let radial = v - cyl_axis * v.dot(cyl_axis);
    let r_dir = radial.normalize_or_zero();
    if r_dir.length_squared() < 0.5 {
        return pt;
    }
    let q_cyl = cyl_origin + r_dir * cyl_radius;
    // 2. Intersect cylinder generator line `q_cyl + t * cyl_axis` with `plane`
    let denom = cyl_axis.dot(plane_normal);
    if denom.abs() < 1e-8 {
        return pt;
    }
    let t = (plane_origin - q_cyl).dot(plane_normal) / denom;
    q_cyl + cyl_axis * t
}

#[allow(clippy::too_many_arguments)]
fn subdivide_plane_cylinder(
    a: DVec3,
    b: DVec3,
    plane_origin: DVec3,
    plane_normal: DVec3,
    cyl_origin: DVec3,
    cyl_axis: DVec3,
    cyl_radius: f64,
    chord_tol_mm: f64,
    max_step_mm: f64,
    depth: u32,
    out: &mut Vec<DVec3>,
) {
    let mid_lin = 0.5 * (a + b);
    let mid_exact = project_onto_plane_cylinder(
        mid_lin,
        plane_origin,
        plane_normal,
        cyl_origin,
        cyl_axis,
        cyl_radius,
    );
    let sag = mid_exact.distance(mid_lin);
    let len = a.distance(b);
    if depth < 10 && (sag > chord_tol_mm || len > max_step_mm) {
        subdivide_plane_cylinder(
            a,
            mid_exact,
            plane_origin,
            plane_normal,
            cyl_origin,
            cyl_axis,
            cyl_radius,
            chord_tol_mm,
            max_step_mm,
            depth + 1,
            out,
        );
        subdivide_plane_cylinder(
            mid_exact,
            b,
            plane_origin,
            plane_normal,
            cyl_origin,
            cyl_axis,
            cyl_radius,
            chord_tol_mm,
            max_step_mm,
            depth + 1,
            out,
        );
    } else {
        let b_exact = project_onto_plane_cylinder(
            b,
            plane_origin,
            plane_normal,
            cyl_origin,
            cyl_axis,
            cyl_radius,
        );
        out.push(b_exact);
    }
}

fn is_sample_list_closed(samples: &[DeburrSample3D]) -> bool {
    match (samples.first(), samples.last()) {
        (Some(a), Some(b)) => a.pos_wcs.distance(b.pos_wcs) <= COINCIDENCE_TOL_MM * 3.0,
        _ => false,
    }
}

fn append_connected_samples(dst: &mut Vec<DeburrSample3D>, src: &[DeburrSample3D]) {
    if src.is_empty() {
        return;
    }
    if let Some(last) = dst.last()
        && let Some(first) = src.first()
        && last.pos_wcs.distance(first.pos_wcs) <= COINCIDENCE_TOL_MM * 3.0
        && last.bisector_out.dot(first.bisector_out) >= 0.995
    {
        dst.extend_from_slice(&src[1..]);
    } else {
        dst.extend_from_slice(src);
    }
}

fn reverse_sample_chain(samples: &mut [DeburrSample3D]) {
    samples.reverse();
    for s in samples.iter_mut() {
        s.tangent = -s.tangent;
    }
}

/// Stitches open auto-detected 3D edge segments sharing endpoints (within coincidence tolerance)
/// into continuous maximal chains, preferring smooth tangent continuity and matching top/underside
/// classification.
fn stitch_open_edge_segments(mut segs: Vec<Vec<DeburrSample3D>>) -> Vec<DeburrEdgeChain> {
    let mut used = vec![false; segs.len()];
    let mut chains = Vec::new();
    let weld_tol = COINCIDENCE_TOL_MM * 4.0;

    for i in 0..segs.len() {
        if used[i] || segs[i].len() < 2 {
            continue;
        }
        used[i] = true;
        let mut cur = std::mem::take(&mut segs[i]);
        let is_underside = cur[cur.len() / 2].is_underside;

        // Extend forward
        while let Some(&tail) = cur.last() {
            if cur.len() > 2 && tail.pos_wcs.distance(cur[0].pos_wcs) <= weld_tol {
                break;
            }
            let mut best: Option<(usize, bool, f64)> = None;
            for (j, cand) in segs.iter().enumerate() {
                if used[j] || cand.len() < 2 || cand[cand.len() / 2].is_underside != is_underside {
                    continue;
                }
                if let Some(&c_first) = cand.first()
                    && c_first.pos_wcs.distance(tail.pos_wcs) <= weld_tol
                {
                    let score = tail.tangent.dot(c_first.tangent);
                    if best.is_none_or(|b| score > b.2) {
                        best = Some((j, false, score));
                    }
                }
                if let Some(&c_last) = cand.last()
                    && c_last.pos_wcs.distance(tail.pos_wcs) <= weld_tol
                {
                    let score = tail.tangent.dot(-c_last.tangent);
                    if best.is_none_or(|b| score > b.2) {
                        best = Some((j, true, score));
                    }
                }
            }
            let Some((best_j, flip, _)) = best else {
                break;
            };
            used[best_j] = true;
            let mut next_seg = std::mem::take(&mut segs[best_j]);
            if flip {
                reverse_sample_chain(&mut next_seg);
            }
            append_connected_samples(&mut cur, &next_seg);
        }

        // Extend backward (prepend)
        while let Some(&head) = cur.first() {
            if cur.len() > 2
                && let Some(&tail) = cur.last()
                && tail.pos_wcs.distance(head.pos_wcs) <= weld_tol
            {
                break;
            }
            let mut best: Option<(usize, bool, f64)> = None;
            for (j, cand) in segs.iter().enumerate() {
                if used[j] || cand.len() < 2 || cand[cand.len() / 2].is_underside != is_underside {
                    continue;
                }
                if let Some(&c_last) = cand.last()
                    && c_last.pos_wcs.distance(head.pos_wcs) <= weld_tol
                {
                    let score = c_last.tangent.dot(head.tangent);
                    if best.is_none_or(|b| score > b.2) {
                        best = Some((j, false, score));
                    }
                }
                if let Some(&c_first) = cand.first()
                    && c_first.pos_wcs.distance(head.pos_wcs) <= weld_tol
                {
                    let score = (-c_first.tangent).dot(head.tangent);
                    if best.is_none_or(|b| score > b.2) {
                        best = Some((j, true, score));
                    }
                }
            }
            let Some((best_j, flip, _)) = best else {
                break;
            };
            used[best_j] = true;
            let mut prev_seg = std::mem::take(&mut segs[best_j]);
            if flip {
                reverse_sample_chain(&mut prev_seg);
            }
            append_connected_samples(&mut prev_seg, &cur);
            cur = prev_seg;
        }

        let closed = is_sample_list_closed(&cur);
        chains.push(DeburrEdgeChain {
            samples: cur,
            closed,
            is_underside,
            user_reversed: false,
        });
    }

    chains
}

/// Ensures chain travel direction matches `climb_direction` (unless explicitly reversed by user).
pub(super) fn orient_chain_for_cut_direction(chain: &mut DeburrEdgeChain, climb_direction: bool) {
    if chain.user_reversed || chain.samples.len() < 2 {
        return;
    }
    // For CW spindle (`M3`), Climb milling puts the outward air normal to the LEFT of travel:
    // `(tangent_xy × bisector_xy).z > 0`.
    let mut score = 0.0_f64;
    for s in &chain.samples {
        score += s.tangent.x * s.bisector_out.y - s.tangent.y * s.bisector_out.x;
    }
    let want_reverse = if climb_direction {
        score < -1e-6
    } else {
        score > 1e-6
    };
    if want_reverse {
        reverse_sample_chain(&mut chain.samples);
    }
}

pub(super) fn apply_open_chain_extension(chain: &mut DeburrEdgeChain, ext_mm: f64) {
    if chain.samples.len() < 2 || ext_mm <= 1e-4 {
        return;
    }
    let steps = (ext_mm / 0.25).ceil().clamp(1.0, 40.0) as usize;
    let first = chain.samples[0];
    let last = chain.samples[chain.samples.len() - 1];

    let mut prefix = Vec::with_capacity(steps);
    for k in (1..=steps).rev() {
        let d = ext_mm * (k as f64) / (steps as f64);
        let mut s = first;
        s.pos_wcs = first.pos_wcs - first.tangent * d;
        prefix.push(s);
    }
    let mut suffix = Vec::with_capacity(steps);
    for k in 1..=steps {
        let d = ext_mm * (k as f64) / (steps as f64);
        let mut s = last;
        s.pos_wcs = last.pos_wcs + last.tangent * d;
        suffix.push(s);
    }

    let mut extended = Vec::with_capacity(prefix.len() + chain.samples.len() + suffix.len());
    extended.extend(prefix);
    extended.extend_from_slice(&chain.samples);
    extended.extend(suffix);
    chain.samples = extended;
}
