//! Minimum-Radius & Flute/Stickout Reach Machinability Analyzer (`chip-cam`).
//!
//! Pure, `Send + Sync`, cacheable geometric analysis scanning B-Rep concave vertical fillets,
//! small holes, narrow opposing slots, sharp internal corners, closed 2D Sketch corners/arcs,
//! and feature Z depths against the active cutter's diameter, flute length, relieved neck, and
//! stickout length.

use std::{
    collections::{HashMap, HashSet},
    f64::consts::{PI, SQRT_2, TAU},
    hash::{Hash as _, Hasher as _},
};

use chip_cad::{CadDocument, CurveKind, EdgeId, FaceId, SurfaceKind, expand_coaxial_hole_faces};
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3, RigidFrame3};
use chip_sketch::{SketchEntity, SketchLayer};
use chip_tools::{CutterGeometry, Tool};

use crate::setup::{ResolvedSetup, Setup, resolve_sketch_world_frame_with_doc};

const CORNER_FIT_TOL_MM: f64 = 0.02;
const DEPTH_REACH_TOL_MM: f64 = 0.02;
const VERTICAL_AXIS_COS_MIN: f64 = 0.9659258262890683; // cos(15 deg)
const VERTICAL_WALL_NORMAL_Z_MAX: f64 = 0.25881904510252074; // sin(15 deg)
const SHARP_CORNER_SIN_MIN: f64 = 0.17364817766693033; // sin(10 deg)

/// Specific machinability / cutter-fit or reach issue detected on a CAD or Sketch feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReachIssueKind {
    /// Concave internal corner radius `r < tool_r - 0.02 mm` (leaves rest stock).
    TightCorner,
    /// Opposing vertical walls or hole diameter `< tool_d + 0.02 mm` (cutter cannot enter).
    NarrowSlotBlocked,
    /// Feature depth exceeds flute length (and any relieved neck length) -> shank rubbing risk.
    ExceedsFluteLength,
    /// Feature depth exceeds tool stickout length -> collet/holder collision.
    ExceedsStickout,
}

impl ReachIssueKind {
    pub const fn severity(self) -> ReachSeverity {
        match self {
            Self::TightCorner | Self::ExceedsFluteLength => ReachSeverity::Warning,
            Self::NarrowSlotBlocked | Self::ExceedsStickout => ReachSeverity::Critical,
        }
    }

    pub const fn short_label(self) -> &'static str {
        match self {
            Self::TightCorner => "Tight Corner",
            Self::NarrowSlotBlocked => "Blocked Slot/Hole",
            Self::ExceedsFluteLength => "Exceeds LOC",
            Self::ExceedsStickout => "Exceeds Stickout",
        }
    }
}

/// Visual heatmap severity bucket rendered in the 3D viewport.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ReachSeverity {
    /// Amber overlay: tight fillet corner or flute-length rub risk.
    Warning,
    /// Crimson overlay: physically blocked slot/hole or stickout-unreachable collision.
    Critical,
}

/// Highlighted 3D B-Rep face region with pre-built World-space triangles for zero-cost rendering.
#[derive(Debug, Clone, PartialEq)]
pub struct ReachHighlightFace {
    pub face_id: Option<FaceId>,
    pub kind: ReachIssueKind,
    pub severity: ReachSeverity,
    /// Pre-built `(triangle_vertices_world_mm, outward_normal_world)` tuples.
    pub triangles_world: Vec<([DVec3; 3], DVec3)>,
    pub measured_value_mm: f64,
    pub limit_value_mm: f64,
    pub detail: String,
}

/// Highlighted 3D edge / 2D sketch arc / corner segment in World coordinates.
#[derive(Debug, Clone, PartialEq)]
pub struct ReachHighlightEdge {
    pub edge_id: Option<EdgeId>,
    pub kind: ReachIssueKind,
    pub severity: ReachSeverity,
    pub segments_world: Vec<(DVec3, DVec3)>,
    pub measured_value_mm: f64,
    pub limit_value_mm: f64,
    pub detail: String,
}

/// Complete cached Minimum-Radius & Reach analysis result for a `(CAD, Setup, Tool)` triple.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ReachAnalysisResult {
    pub tool_name: String,
    pub tool_diameter_mm: f64,
    pub tool_flute_length_mm: f64,
    pub tool_effective_reach_mm: f64,
    pub tool_stickout_length_mm: f64,
    pub min_internal_radius_mm: Option<f64>,
    pub has_sharp_internal_corners: bool,
    pub max_feature_depth_mm: f64,
    pub amber_count: usize,
    pub crimson_count: usize,
    pub tight_corner_count: usize,
    pub narrow_slot_count: usize,
    pub flute_exceeded_count: usize,
    pub stickout_exceeded_count: usize,
    pub highlight_faces: Vec<ReachHighlightFace>,
    pub highlight_edges: Vec<ReachHighlightEdge>,
}

/// Computes the maximum non-rubbing vertical cutting/neck reach of `tool` in millimeters.
///
/// Respects relieved necks (`neck_diameter_mm < cutting_diameter_mm` up to `neck_length_mm`) and
/// reduced shanks (`shank_diameter_mm < cutting_diameter_mm` up to `stickout_length_mm`).
pub fn effective_non_rubbing_reach_mm(tool: &Tool) -> f64 {
    let flute = tool.geometry.flute_length_mm().max(0.0);
    let stickout = tool.stickout_length_mm.max(flute);
    let d_cut = tool.geometry.cutting_diameter_mm();

    match tool.geometry {
        CutterGeometry::ThreadMill {
            neck_diameter_mm,
            neck_length_mm,
            ..
        }
        | CutterGeometry::LollipopEndmill {
            neck_diameter_mm,
            neck_length_mm,
            ..
        }
        | CutterGeometry::SlotMill {
            neck_diameter_mm,
            neck_length_mm,
            ..
        }
        | CutterGeometry::DovetailCutter {
            neck_diameter_mm,
            neck_length_mm,
            ..
        } => {
            if neck_diameter_mm + 1e-3 < d_cut {
                neck_length_mm.max(flute).min(stickout)
            } else {
                flute
            }
        }
        _ => {
            let shank = tool.geometry.shank_diameter_mm();
            if shank > 0.0 && shank + 1e-3 < d_cut {
                stickout
            } else {
                flute
            }
        }
    }
}

/// Computes a deterministic cache key over `(cad_revision, setup, resolved_setup, tool, sketches)`
/// so idle 60 FPS viewport frames perform zero redundant reach analysis work.
pub fn compute_reach_cache_key(
    cad_revision: u64,
    setup: &Setup,
    resolved_setup: &ResolvedSetup,
    tool: &Tool,
    sketches: &[SketchLayer],
) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    cad_revision.hash(&mut h);
    setup.id.hash(&mut h);
    setup.target_bodies.hash(&mut h);

    for v in [
        resolved_setup.frame.origin_mm,
        resolved_setup.frame.x_axis,
        resolved_setup.frame.y_axis,
        resolved_setup.frame.z_axis,
    ] {
        v.x.to_bits().hash(&mut h);
        v.y.to_bits().hash(&mut h);
        v.z.to_bits().hash(&mut h);
    }
    let st = resolved_setup.stock.aabb_wcs();
    for p in [st.min, st.max] {
        p.x.to_bits().hash(&mut h);
        p.y.to_bits().hash(&mut h);
        p.z.to_bits().hash(&mut h);
    }

    tool.id.0.hash(&mut h);
    tool.geometry.cutting_diameter_mm().to_bits().hash(&mut h);
    tool.geometry.flute_length_mm().to_bits().hash(&mut h);
    tool.geometry.shank_diameter_mm().to_bits().hash(&mut h);
    effective_non_rubbing_reach_mm(tool).to_bits().hash(&mut h);
    tool.stickout_length_mm.to_bits().hash(&mut h);
    std::mem::discriminant(&tool.geometry).hash(&mut h);

    for sk in sketches {
        if !sk.visible {
            continue;
        }
        sk.id.hash(&mut h);
        sk.entities.len().hash(&mut h);
        sk.regions.len().hash(&mut h);
        let b = sk.exact_aabb();
        if b.is_valid() {
            b.min.x.to_bits().hash(&mut h);
            b.min.y.to_bits().hash(&mut h);
            b.max.x.to_bits().hash(&mut h);
            b.max.y.to_bits().hash(&mut h);
        }
    }

    h.finish()
}

/// Analyzes minimum internal corner radii, blocked narrow slots/holes, and flute/stickout vertical
/// reach depths across `cad_doc` and `sketches` in active Setup WCS for `tool`.
///
/// Pure and `Send + Sync` — intended to be dispatched on a background `rayon` thread and cached by
/// [`compute_reach_cache_key`].
pub fn analyze_machinability_reach(
    cad_doc: &CadDocument,
    sketches: &[SketchLayer],
    setup: &Setup,
    resolved_setup: &ResolvedSetup,
    tool: &Tool,
) -> ReachAnalysisResult {
    let tool_d = tool.geometry.cutting_diameter_mm().max(1e-6);
    let tool_r = 0.5 * tool_d;
    let flute_len = tool.geometry.flute_length_mm().max(0.0);
    let effective_reach = effective_non_rubbing_reach_mm(tool);
    let stickout = tool.stickout_length_mm.max(flute_len);
    let is_drill = matches!(tool.geometry, CutterGeometry::TwistDrill { .. });
    let hole_min_d = if is_drill {
        (tool_d - CORNER_FIT_TOL_MM).max(0.0)
    } else {
        tool_d + CORNER_FIT_TOL_MM
    };

    let frame = &resolved_setup.frame;
    let stock_aabb = resolved_setup.stock.aabb_wcs();
    let stock_top_z_wcs = if stock_aabb.is_valid() {
        stock_aabb.max.z
    } else if resolved_setup.model_aabb_wcs.is_valid() {
        resolved_setup.model_aabb_wcs.max.z
    } else {
        0.0
    };

    let mut result = ReachAnalysisResult {
        tool_name: tool.name.clone(),
        tool_diameter_mm: tool_d,
        tool_flute_length_mm: flute_len,
        tool_effective_reach_mm: effective_reach,
        tool_stickout_length_mm: stickout,
        ..ReachAnalysisResult::default()
    };

    let include_body = |bid| setup.target_bodies.is_empty() || setup.target_bodies.contains(&bid);

    // Pre-index B-Rep face triangles & edge polylines once in O(T + E)
    let mut tris_by_face: Vec<Vec<([DVec3; 3], DVec3)>> = vec![Vec::new(); cad_doc.faces.len()];
    for tri in &cad_doc.render_mesh.pick_triangles {
        let idx = tri.face_id.0 as usize;
        if let Some(slot) = tris_by_face.get_mut(idx) {
            let n = (tri.v1_mm - tri.v0_mm)
                .cross(tri.v2_mm - tri.v0_mm)
                .normalize_or_zero();
            let n = if n.length_squared() > 0.25 {
                n
            } else {
                DVec3::Z
            };
            slot.push(([tri.v0_mm, tri.v1_mm, tri.v2_mm], n));
        }
    }

    let mut segs_by_edge: Vec<Vec<(DVec3, DVec3)>> = vec![Vec::new(); cad_doc.edges.len()];
    for ep in &cad_doc.render_mesh.edge_polylines {
        let idx = ep.edge_id.0 as usize;
        if let Some(slot) = segs_by_edge.get_mut(idx) {
            for w in ep.points_mm.windows(2) {
                slot.push((w[0], w[1]));
            }
        }
    }

    // Track worst-severity highlight per FaceId so translucent triangles never double-stack
    let mut face_issues: HashMap<FaceId, ReachHighlightFace> = HashMap::new();
    let mut record_face_issue = |fid: FaceId,
                                 kind: ReachIssueKind,
                                 measured_value_mm: f64,
                                 limit_value_mm: f64,
                                 detail: String| {
        let severity = kind.severity();
        let replace = face_issues
            .get(&fid)
            .is_none_or(|prev| severity > prev.severity);
        if replace {
            let triangles_world = tris_by_face
                .get(fid.0 as usize)
                .cloned()
                .unwrap_or_default();
            face_issues.insert(
                fid,
                ReachHighlightFace {
                    face_id: Some(fid),
                    kind,
                    severity,
                    triangles_world,
                    measured_value_mm,
                    limit_value_mm,
                    detail,
                },
            );
        }
    };

    // ------------------------------------------------------------------------
    // 1. B-Rep Concave Cylindrical Faces (Internal Corner Fillets & Small Holes)
    // ------------------------------------------------------------------------
    let mut visited_cyl_faces: HashSet<FaceId> = HashSet::new();
    for face in &cad_doc.faces {
        if !include_body(face.body_id) || visited_cyl_faces.contains(&face.id) {
            continue;
        }
        let SurfaceKind::Cylinder {
            axis_origin_mm,
            axis_dir,
            radius_mm,
            is_internal: true,
        } = face.surface
        else {
            continue;
        };

        let axis_wcs = frame.world_to_local_dir(axis_dir).normalize_or_zero();
        if axis_wcs.z.abs() < VERTICAL_AXIS_COS_MIN {
            continue;
        }

        let group = expand_coaxial_hole_faces(cad_doc, face.id);
        for &gfid in &group {
            visited_cyl_faces.insert(gfid);
        }

        // Record smallest internal radius
        result.min_internal_radius_mm = Some(
            result
                .min_internal_radius_mm
                .map_or(radius_mm, |prev| prev.min(radius_mm)),
        );

        let coverage_rad = estimate_cylinder_group_sweep_rad(
            &group,
            &tris_by_face,
            axis_origin_mm,
            axis_dir.normalize_or_zero(),
        );
        let is_full_hole = coverage_rad >= 1.45 * PI;
        let dia_mm = 2.0 * radius_mm;

        if is_full_hole {
            if dia_mm < hole_min_d {
                result.narrow_slot_count += 1;
                let detail = format!("Hole Dia {:.3} mm < cutter Dia {:.3} mm", dia_mm, tool_d);
                for &gfid in &group {
                    record_face_issue(
                        gfid,
                        ReachIssueKind::NarrowSlotBlocked,
                        dia_mm,
                        tool_d,
                        detail.clone(),
                    );
                }
            }
        } else if radius_mm < tool_r - CORNER_FIT_TOL_MM {
            result.tight_corner_count += 1;
            let radial_cusp = (tool_r - radius_mm) * (SQRT_2 - 1.0);
            let detail = format!(
                "Concave fillet R{:.3} mm < tool R{:.3} mm (cusp {:.2} mm; max cutter Dia {:.2} mm)",
                radius_mm, tool_r, radial_cusp, dia_mm
            );
            for &gfid in &group {
                record_face_issue(
                    gfid,
                    ReachIssueKind::TightCorner,
                    radius_mm,
                    tool_r,
                    detail.clone(),
                );
            }
        }
    }

    // ------------------------------------------------------------------------
    // 2. B-Rep Opposing Vertical Planar Walls (Narrow Slot Blocked)
    // ------------------------------------------------------------------------
    struct VerticalWallInfo {
        face_id: FaceId,
        normal_wcs: DVec3,
        tangent_wcs: DVec3,
        origin_wcs: DVec3,
        t_min: f64,
        t_max: f64,
        z_min: f64,
        z_max: f64,
    }

    let mut vertical_walls: Vec<VerticalWallInfo> = Vec::new();
    for face in &cad_doc.faces {
        if !include_body(face.body_id) {
            continue;
        }
        let SurfaceKind::Plane { origin_mm, normal } = face.surface else {
            continue;
        };
        let n_wcs = frame.world_to_local_dir(normal).normalize_or_zero();
        if n_wcs.length_squared() < 0.5 || n_wcs.z.abs() > VERTICAL_WALL_NORMAL_Z_MAX {
            continue;
        }
        let n_xy = DVec3::new(n_wcs.x, n_wcs.y, 0.0).normalize_or_zero();
        if n_xy.length_squared() < 0.5 {
            continue;
        }
        let tangent_wcs = DVec3::Z.cross(n_xy).normalize_or_zero();
        let Some(tris) = tris_by_face.get(face.id.0 as usize) else {
            continue;
        };
        if tris.is_empty() {
            continue;
        }
        let mut t_min = f64::INFINITY;
        let mut t_max = f64::NEG_INFINITY;
        let mut z_min = f64::INFINITY;
        let mut z_max = f64::NEG_INFINITY;
        for (verts, _) in tris {
            for &v in verts {
                let p = frame.world_to_local_point(v);
                let tp = p.dot(tangent_wcs);
                t_min = t_min.min(tp);
                t_max = t_max.max(tp);
                z_min = z_min.min(p.z);
                z_max = z_max.max(p.z);
            }
        }
        if t_max > t_min + 0.1 && z_max > z_min + 0.1 {
            vertical_walls.push(VerticalWallInfo {
                face_id: face.id,
                normal_wcs: n_xy,
                tangent_wcs,
                origin_wcs: frame.world_to_local_point(origin_mm),
                t_min,
                t_max,
                z_min,
                z_max,
            });
        }
    }

    let mut flagged_slot_pairs: HashSet<(FaceId, FaceId)> = HashSet::new();
    for i in 0..vertical_walls.len() {
        for j in (i + 1)..vertical_walls.len() {
            let a = &vertical_walls[i];
            let b = &vertical_walls[j];
            // Anti-parallel walls facing toward each other
            if a.normal_wcs.dot(b.normal_wcs) > -0.985 {
                continue;
            }
            let gap_ab = (b.origin_wcs - a.origin_wcs).dot(a.normal_wcs);
            let gap_ba = (a.origin_wcs - b.origin_wcs).dot(b.normal_wcs);
            if gap_ab <= 0.05 || gap_ba <= 0.05 {
                continue;
            }
            let gap = 0.5 * (gap_ab + gap_ba);
            if gap >= tool_d + CORNER_FIT_TOL_MM {
                continue;
            }
            // Check Z overlap and lateral overlap along tangent_a
            let z_ov = a.z_max.min(b.z_max) - a.z_min.max(b.z_min);
            if z_ov < 0.1 {
                continue;
            }
            // Project b's tangent span onto a.tangent_wcs (opposite orientation since normals oppose)
            let b_t_min = -b.t_max;
            let b_t_max = -b.t_min;
            let t_ov = a.t_max.min(b_t_max) - a.t_min.max(b_t_min);
            if t_ov < 0.5 {
                continue;
            }
            // Verify no intermediate wall separates a and b in this overlapping span
            let ov_t0 = a.t_min.max(b_t_min);
            let ov_t1 = a.t_max.min(b_t_max);
            let ov_z0 = a.z_min.max(b.z_min);
            let ov_z1 = a.z_max.min(b.z_max);
            let blocked_by_mid_wall = vertical_walls.iter().enumerate().any(|(k, c)| {
                if k == i || k == j {
                    return false;
                }
                if c.normal_wcs.dot(a.normal_wcs).abs() < 0.985 {
                    return false;
                }
                let d_from_a = (c.origin_wcs - a.origin_wcs).dot(a.normal_wcs);
                if d_from_a <= 0.02 || d_from_a >= gap - 0.02 {
                    return false;
                }
                let z_hit = c.z_max.min(ov_z1) - c.z_min.max(ov_z0) > 0.05;
                let (ct0, ct1) = if c.tangent_wcs.dot(a.tangent_wcs) >= 0.0 {
                    (c.t_min, c.t_max)
                } else {
                    (-c.t_max, -c.t_min)
                };
                let t_hit = ct1.min(ov_t1) - ct0.max(ov_t0) > 0.05;
                z_hit && t_hit
            });
            if blocked_by_mid_wall {
                continue;
            }

            if flagged_slot_pairs.insert((a.face_id, b.face_id)) {
                result.narrow_slot_count += 1;
                let detail = format!("Slot width {:.3} mm < cutter Dia {:.3} mm", gap, tool_d);
                record_face_issue(
                    a.face_id,
                    ReachIssueKind::NarrowSlotBlocked,
                    gap,
                    tool_d,
                    detail.clone(),
                );
                record_face_issue(
                    b.face_id,
                    ReachIssueKind::NarrowSlotBlocked,
                    gap,
                    tool_d,
                    detail,
                );
            }
        }
    }

    // ------------------------------------------------------------------------
    // 3. Sharp Interior Vertical B-Rep Corners (Concave Edge between 2 Vertical Walls)
    // ------------------------------------------------------------------------
    for edge in &cad_doc.edges {
        if !include_body(edge.body_id) || edge.adjacent_faces.len() < 2 {
            continue;
        }
        let CurveKind::Line { start_mm, end_mm } = edge.curve else {
            continue;
        };
        let dir_wcs = frame
            .world_to_local_dir(end_mm - start_mm)
            .normalize_or_zero();
        if dir_wcs.z.abs() < VERTICAL_AXIS_COS_MIN || edge.length_mm < 0.1 {
            continue;
        }
        let edge_mid_wcs = frame.world_to_local_point(0.5 * (start_mm + end_mm));

        let mut is_concave_corner = false;
        'pair: for a_idx in 0..edge.adjacent_faces.len() {
            let Some(fa) = cad_doc.face(edge.adjacent_faces[a_idx]) else {
                continue;
            };
            let SurfaceKind::Plane { normal: na_w, .. } = fa.surface else {
                continue;
            };
            let na = frame.world_to_local_dir(na_w).normalize_or_zero();
            if na.z.abs() > VERTICAL_WALL_NORMAL_Z_MAX {
                continue;
            }
            let ca = frame.world_to_local_point(fa.signature.centroid_mm);

            for &fb_id in edge.adjacent_faces.iter().skip(a_idx + 1) {
                let Some(fb) = cad_doc.face(fb_id) else {
                    continue;
                };
                let SurfaceKind::Plane { normal: nb_w, .. } = fb.surface else {
                    continue;
                };
                let nb = frame.world_to_local_dir(nb_w).normalize_or_zero();
                if nb.z.abs() > VERTICAL_WALL_NORMAL_Z_MAX {
                    continue;
                }
                if na.dot(nb) > 0.985 {
                    continue;
                }
                let cb = frame.world_to_local_point(fb.signature.centroid_mm);
                // Interior (concave) corner: each face's centroid sits on the outward-normal side of the other
                if (cb - edge_mid_wcs).dot(na) > 1e-3 && (ca - edge_mid_wcs).dot(nb) > 1e-3 {
                    is_concave_corner = true;
                    break 'pair;
                }
            }
        }

        if is_concave_corner && tool_r > CORNER_FIT_TOL_MM {
            result.has_sharp_internal_corners = true;
            result.tight_corner_count += 1;
            let segs = segs_by_edge
                .get(edge.id.0 as usize)
                .filter(|s| !s.is_empty())
                .cloned()
                .unwrap_or_else(|| vec![(start_mm, end_mm)]);
            result.highlight_edges.push(ReachHighlightEdge {
                edge_id: Some(edge.id),
                kind: ReachIssueKind::TightCorner,
                severity: ReachSeverity::Warning,
                segments_world: segs,
                measured_value_mm: 0.0,
                limit_value_mm: tool_r,
                detail: format!("Sharp interior corner R0.000 mm < tool R{:.3} mm", tool_r),
            });
        }
    }

    // ------------------------------------------------------------------------
    // 4. Flute-Length & Stickout-Reach Depth Check on B-Rep Faces
    // ------------------------------------------------------------------------
    for face in &cad_doc.faces {
        if !include_body(face.body_id) {
            continue;
        }
        // Skip downward-facing exterior undersides (`normal_wcs.z < -0.15`)
        if let SurfaceKind::Plane { normal, .. } = face.surface {
            let n_wcs = frame.world_to_local_dir(normal).normalize_or_zero();
            if n_wcs.z < -0.15 {
                continue;
            }
        }

        let min_z_wcs = face_min_z_wcs(face, tris_by_face.get(face.id.0 as usize), frame);
        let Some(z_min) = min_z_wcs else {
            continue;
        };
        let depth_mm = (stock_top_z_wcs - z_min).max(0.0);
        if depth_mm <= 1e-3 {
            continue;
        }
        result.max_feature_depth_mm = result.max_feature_depth_mm.max(depth_mm);

        if depth_mm > stickout + DEPTH_REACH_TOL_MM {
            result.stickout_exceeded_count += 1;
            let detail = format!(
                "Depth {:.3} mm > stickout {:.3} mm (holder collision)",
                depth_mm, stickout
            );
            record_face_issue(
                face.id,
                ReachIssueKind::ExceedsStickout,
                depth_mm,
                stickout,
                detail,
            );
        } else if depth_mm > effective_reach + DEPTH_REACH_TOL_MM {
            result.flute_exceeded_count += 1;
            let detail = format!(
                "Depth {:.3} mm > effective flute/neck reach {:.3} mm (shank rub risk)",
                depth_mm, effective_reach
            );
            record_face_issue(
                face.id,
                ReachIssueKind::ExceedsFluteLength,
                depth_mm,
                effective_reach,
                detail,
            );
        }
    }

    // ------------------------------------------------------------------------
    // 5. Closed 2D Sketch Concave Corners, Small Arcs, & Blocked Circles
    // ------------------------------------------------------------------------
    for sk in sketches {
        if !sk.visible {
            continue;
        }
        let sk_setup_id = sk.anchor.setup_id();
        if !sk_setup_id.is_empty() && sk_setup_id != setup.id {
            continue;
        }
        let wf = resolve_sketch_world_frame_with_doc(cad_doc, sk, resolved_setup);

        for entry in &sk.entities {
            if entry.is_construction {
                continue;
            }
            match &entry.entity {
                SketchEntity::Circle { diameter, .. } => {
                    let d = diameter.value_mm;
                    let r = 0.5 * d;
                    if r > 1e-6 {
                        result.min_internal_radius_mm =
                            Some(result.min_internal_radius_mm.map_or(r, |prev| prev.min(r)));
                    }
                    if d < hole_min_d
                        && let Some(c) = entry.entity.circle_center_mm()
                    {
                        result.narrow_slot_count += 1;
                        result.highlight_edges.push(ReachHighlightEdge {
                            edge_id: None,
                            kind: ReachIssueKind::NarrowSlotBlocked,
                            severity: ReachSeverity::Critical,
                            segments_world: discretize_circle_world(c, r, &wf, 24),
                            measured_value_mm: d,
                            limit_value_mm: tool_d,
                            detail: format!(
                                "Sketch circle Dia {:.3} mm < cutter Dia {:.3} mm",
                                d, tool_d
                            ),
                        });
                    }
                }
                SketchEntity::PointPattern(_) => {
                    for hole in entry.entity.evaluate_holes() {
                        let d = hole.diameter_mm;
                        let r = 0.5 * d;
                        if r > 1e-6 {
                            result.min_internal_radius_mm =
                                Some(result.min_internal_radius_mm.map_or(r, |prev| prev.min(r)));
                        }
                        if d < hole_min_d {
                            result.narrow_slot_count += 1;
                            result.highlight_edges.push(ReachHighlightEdge {
                                edge_id: None,
                                kind: ReachIssueKind::NarrowSlotBlocked,
                                severity: ReachSeverity::Critical,
                                segments_world: discretize_circle_world(hole.center_xy, r, &wf, 24),
                                measured_value_mm: d,
                                limit_value_mm: tool_d,
                                detail: format!(
                                    "Pattern hole Dia {:.3} mm < cutter Dia {:.3} mm",
                                    d, tool_d
                                ),
                            });
                        }
                    }
                }
                _ => {
                    let poly = entry.entity.to_arc_polyline();
                    if poly.closed && !poly.segments.is_empty() {
                        analyze_closed_sketch_loop(&poly, &wf, tool_r, &mut result);
                    }
                }
            }
        }
    }

    // Finalize deterministic sorted face highlights & aggregate Amber / Crimson totals
    let mut faces_vec: Vec<ReachHighlightFace> = face_issues.into_values().collect();
    faces_vec.sort_by_key(|f| f.face_id.map_or(u32::MAX, |id| id.0));

    for hf in &faces_vec {
        match hf.severity {
            ReachSeverity::Warning => result.amber_count += 1,
            ReachSeverity::Critical => result.crimson_count += 1,
        }
    }
    for he in &result.highlight_edges {
        match he.severity {
            ReachSeverity::Warning => result.amber_count += 1,
            ReachSeverity::Critical => result.crimson_count += 1,
        }
    }
    result.highlight_faces = faces_vec;

    if result.min_internal_radius_mm.is_none() && result.has_sharp_internal_corners {
        result.min_internal_radius_mm = Some(0.0);
    }

    result
}

fn face_min_z_wcs(
    face: &chip_cad::BRepFace,
    tris: Option<&Vec<([DVec3; 3], DVec3)>>,
    frame: &RigidFrame3,
) -> Option<f64> {
    if let Some(tri_list) = tris
        && !tri_list.is_empty()
    {
        let mut min_z = f64::INFINITY;
        for (verts, _) in tri_list {
            for &v in verts {
                min_z = min_z.min(frame.world_to_local_point(v).z);
            }
        }
        if min_z.is_finite() {
            return Some(min_z);
        }
    }
    if face.aabb_mm.is_valid() {
        let mut min_z = f64::INFINITY;
        for x in [face.aabb_mm.min.x, face.aabb_mm.max.x] {
            for y in [face.aabb_mm.min.y, face.aabb_mm.max.y] {
                for z in [face.aabb_mm.min.z, face.aabb_mm.max.z] {
                    min_z = min_z.min(frame.world_to_local_point(DVec3::new(x, y, z)).z);
                }
            }
        }
        if min_z.is_finite() {
            return Some(min_z);
        }
    }
    None
}

fn estimate_cylinder_group_sweep_rad(
    group: &[FaceId],
    tris_by_face: &[Vec<([DVec3; 3], DVec3)>],
    axis_origin: DVec3,
    axis_dir: DVec3,
) -> f64 {
    if axis_dir.length_squared() < 0.5 {
        return 0.0;
    }
    let ref_v = if axis_dir.x.abs() < 0.9 {
        DVec3::X
    } else {
        DVec3::Y
    };
    let u = axis_dir.cross(ref_v).normalize_or_zero();
    let v = axis_dir.cross(u).normalize_or_zero();

    let mut angles: Vec<f64> = Vec::new();
    for &fid in group {
        if let Some(tris) = tris_by_face.get(fid.0 as usize) {
            for (verts, _) in tris {
                for &p in verts {
                    let d = p - axis_origin;
                    let proj = d - axis_dir * d.dot(axis_dir);
                    if proj.length_squared() > 1e-10 {
                        angles.push(proj.dot(v).atan2(proj.dot(u)));
                    }
                }
            }
        }
    }
    if angles.len() < 3 {
        return 0.0;
    }
    angles.sort_by(|a, b| a.total_cmp(b));
    let mut max_gap = 0.0_f64;
    for w in angles.windows(2) {
        max_gap = max_gap.max(w[1] - w[0]);
    }
    if let (Some(&first), Some(&last)) = (angles.first(), angles.last()) {
        max_gap = max_gap.max((first + TAU) - last);
    }
    (TAU - max_gap).clamp(0.0, TAU)
}

fn analyze_closed_sketch_loop(
    poly: &ArcPolyline2D,
    wf: &RigidFrame3,
    tool_r: f64,
    result: &mut ReachAnalysisResult,
) {
    let area = poly.signed_area();
    if area.abs() < 1e-6 {
        return;
    }
    let is_ccw = area > 0.0;
    let to_world = |p: DVec2| wf.local_to_world_point(DVec3::new(p.x, p.y, 0.05));

    // 1. Curved concave corner arcs (same winding as loop -> internal fillet of a pocket)
    for seg in &poly.segments {
        if let ArcSegment2D::Arc {
            start,
            end,
            center,
            radius,
            ccw,
        } = *seg
            && ccw == is_ccw
            && radius > 1e-6
        {
            result.min_internal_radius_mm = Some(
                result
                    .min_internal_radius_mm
                    .map_or(radius, |prev| prev.min(radius)),
            );
            if radius < tool_r - CORNER_FIT_TOL_MM {
                result.tight_corner_count += 1;
                let segs_w = discretize_arc_world(start, end, center, radius, ccw, wf, 12);
                result.highlight_edges.push(ReachHighlightEdge {
                    edge_id: None,
                    kind: ReachIssueKind::TightCorner,
                    severity: ReachSeverity::Warning,
                    segments_world: segs_w,
                    measured_value_mm: radius,
                    limit_value_mm: tool_r,
                    detail: format!(
                        "Sketch corner arc R{:.3} mm < tool R{:.3} mm",
                        radius, tool_r
                    ),
                });
            }
        }
    }

    // 2. Sharp internal vertices between consecutive segments
    let n = poly.segments.len();
    for i in 0..n {
        let s_prev = &poly.segments[i];
        let s_next = &poly.segments[(i + 1) % n];
        let t_in = segment_end_tangent(s_prev);
        let t_out = segment_start_tangent(s_next);
        let cross = t_in.perp_dot(t_out);
        let is_internal_turn = if is_ccw {
            cross > SHARP_CORNER_SIN_MIN
        } else {
            cross < -SHARP_CORNER_SIN_MIN
        };
        if is_internal_turn && tool_r > CORNER_FIT_TOL_MM {
            result.has_sharp_internal_corners = true;
            result.tight_corner_count += 1;
            let corner_xy = segment_start_point(s_next);
            let tick = (tool_r * 0.45).clamp(0.4, 3.0);
            let p_a = to_world(corner_xy - t_in * tick);
            let p_c = to_world(corner_xy);
            let p_b = to_world(corner_xy + t_out * tick);
            result.highlight_edges.push(ReachHighlightEdge {
                edge_id: None,
                kind: ReachIssueKind::TightCorner,
                severity: ReachSeverity::Warning,
                segments_world: vec![(p_a, p_c), (p_c, p_b)],
                measured_value_mm: 0.0,
                limit_value_mm: tool_r,
                detail: format!("Sharp sketch corner R0.000 mm < tool R{:.3} mm", tool_r),
            });
        }
    }
}

fn segment_start_point(seg: &ArcSegment2D) -> DVec2 {
    match *seg {
        ArcSegment2D::Line { start, .. } | ArcSegment2D::Arc { start, .. } => start,
    }
}

fn segment_start_tangent(seg: &ArcSegment2D) -> DVec2 {
    match *seg {
        ArcSegment2D::Line { start, end } => (end - start).normalize_or_zero(),
        ArcSegment2D::Arc {
            start, center, ccw, ..
        } => {
            let r = (start - center).normalize_or_zero();
            if ccw {
                DVec2::new(-r.y, r.x)
            } else {
                DVec2::new(r.y, -r.x)
            }
        }
    }
}

fn segment_end_tangent(seg: &ArcSegment2D) -> DVec2 {
    match *seg {
        ArcSegment2D::Line { start, end } => (end - start).normalize_or_zero(),
        ArcSegment2D::Arc {
            end, center, ccw, ..
        } => {
            let r = (end - center).normalize_or_zero();
            if ccw {
                DVec2::new(-r.y, r.x)
            } else {
                DVec2::new(r.y, -r.x)
            }
        }
    }
}

fn discretize_circle_world(
    center: DVec2,
    radius: f64,
    wf: &RigidFrame3,
    steps: usize,
) -> Vec<(DVec3, DVec3)> {
    let n = steps.max(8);
    let mut segs = Vec::with_capacity(n);
    let mut prev = wf.local_to_world_point(DVec3::new(center.x + radius, center.y, 0.05));
    for i in 1..=n {
        let a = (i as f64 / n as f64) * TAU;
        let cur = wf.local_to_world_point(DVec3::new(
            center.x + radius * a.cos(),
            center.y + radius * a.sin(),
            0.05,
        ));
        segs.push((prev, cur));
        prev = cur;
    }
    segs
}

fn discretize_arc_world(
    start: DVec2,
    end: DVec2,
    center: DVec2,
    radius: f64,
    ccw: bool,
    wf: &RigidFrame3,
    steps: usize,
) -> Vec<(DVec3, DVec3)> {
    let a0 = (start.y - center.y).atan2(start.x - center.x);
    let a1 = (end.y - center.y).atan2(end.x - center.x);
    let mut sweep = a1 - a0;
    if ccw {
        if sweep <= 1e-9 {
            sweep += TAU;
        }
    } else if sweep >= -1e-9 {
        sweep -= TAU;
    }
    let n = steps.max(4);
    let mut segs = Vec::with_capacity(n);
    let mut prev = wf.local_to_world_point(DVec3::new(start.x, start.y, 0.05));
    for i in 1..=n {
        let a = a0 + sweep * (i as f64 / n as f64);
        let cur = wf.local_to_world_point(DVec3::new(
            center.x + radius * a.cos(),
            center.y + radius * a.sin(),
            0.05,
        ));
        segs.push((prev, cur));
        prev = cur;
    }
    segs
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::{
        BRepFace, BodyId, EntitySignatureKind, FaceTriangle, GeometricSignature, RenderMesh,
        SolidBody, StepUnitKind,
    };
    use chip_math::{Aabb3, DisplayUnit, ParamLength};
    use chip_sketch::{RectAnchorMode, SketchEntity, SketchEntityEntry, SketchLayer};
    use chip_tools::{CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId};

    use super::*;
    use crate::{BoxAnchor27, StockConfig, WcsOriginMode, WorkOffsetSlot};

    fn make_flat_endmill(d_mm: f64, flute_mm: f64, stickout_mm: f64) -> Tool {
        Tool {
            id: ToolId(format!("flat_{d_mm}_{flute_mm}_{stickout_mm}")),
            tool_number: 1,
            length_offset_number: 1,
            diameter_offset_number: 1,
            name: format!("D{d_mm} Flat"),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::Disabled,
            stickout_length_mm: stickout_mm,
            geometry: CutterGeometry::FlatEndmill {
                diameter_mm: d_mm,
                flute_length_mm: flute_mm,
                overall_length_mm: (stickout_mm + 20.0).max(50.0),
                shank_diameter_mm: d_mm,
                flutes: 3,
            },
            presets: Vec::new(),
        }
    }

    fn make_relieved_neck_tool(
        d_mm: f64,
        neck_d_mm: f64,
        flute_mm: f64,
        neck_len_mm: f64,
        stickout_mm: f64,
    ) -> Tool {
        Tool {
            id: ToolId("relieved_neck".to_string()),
            tool_number: 2,
            length_offset_number: 2,
            diameter_offset_number: 2,
            name: "Relieved Neck Lollipop".to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::Disabled,
            stickout_length_mm: stickout_mm,
            geometry: CutterGeometry::LollipopEndmill {
                diameter_mm: d_mm,
                neck_diameter_mm: neck_d_mm,
                neck_length_mm: neck_len_mm,
                flute_length_mm: flute_mm,
                overall_length_mm: stickout_mm + 20.0,
                shank_diameter_mm: d_mm,
                flutes: 3,
            },
            presets: Vec::new(),
        }
    }

    /// Synthesizes a minimal B-Rep CadDocument with:
    /// - Stock/part top at `Z = 20.0 mm`, pocket floor at `Z = 2.0 mm` (`18.0 mm` depth!)
    /// - A 90-degree concave vertical cylindrical corner fillet of `R = 2.0 mm` spanning `Z = [2.0, 20.0]`.
    fn synthetic_filleted_deep_pocket_doc(fillet_r_mm: f64, pocket_depth_mm: f64) -> CadDocument {
        let top_z = 20.0;
        let floor_z = top_z - pocket_depth_mm;

        // Build 90-degree arc triangles for the concave fillet face (FaceId(0))
        let mut pick_triangles = Vec::new();
        let center = DVec3::new(10.0, 10.0, 0.0);
        let segs = 8;
        for i in 0..segs {
            let a0 = (i as f64 / segs as f64) * (0.5 * PI);
            let a1 = ((i + 1) as f64 / segs as f64) * (0.5 * PI);
            let p0_b = DVec3::new(
                center.x + fillet_r_mm * a0.cos(),
                center.y + fillet_r_mm * a0.sin(),
                floor_z,
            );
            let p1_b = DVec3::new(
                center.x + fillet_r_mm * a1.cos(),
                center.y + fillet_r_mm * a1.sin(),
                floor_z,
            );
            let p0_t = DVec3::new(p0_b.x, p0_b.y, top_z);
            let p1_t = DVec3::new(p1_b.x, p1_b.y, top_z);
            pick_triangles.push(FaceTriangle {
                face_id: FaceId(0),
                v0_mm: p0_b,
                v1_mm: p1_b,
                v2_mm: p1_t,
            });
            pick_triangles.push(FaceTriangle {
                face_id: FaceId(0),
                v0_mm: p0_b,
                v1_mm: p1_t,
                v2_mm: p0_t,
            });
        }

        // Upward-facing pocket floor at `floor_z` (FaceId(1))
        pick_triangles.push(FaceTriangle {
            face_id: FaceId(1),
            v0_mm: DVec3::new(0.0, 0.0, floor_z),
            v1_mm: DVec3::new(40.0, 0.0, floor_z),
            v2_mm: DVec3::new(40.0, 40.0, floor_z),
        });
        pick_triangles.push(FaceTriangle {
            face_id: FaceId(1),
            v0_mm: DVec3::new(0.0, 0.0, floor_z),
            v1_mm: DVec3::new(40.0, 40.0, floor_z),
            v2_mm: DVec3::new(0.0, 40.0, floor_z),
        });

        // Top deck face at `top_z` (FaceId(2))
        pick_triangles.push(FaceTriangle {
            face_id: FaceId(2),
            v0_mm: DVec3::new(0.0, 0.0, top_z),
            v1_mm: DVec3::new(40.0, 0.0, top_z),
            v2_mm: DVec3::new(40.0, 40.0, top_z),
        });

        let aabb = Aabb3::new(DVec3::new(0.0, 0.0, floor_z), DVec3::new(40.0, 40.0, top_z));
        let make_sig = |kind, normal_or_axis, centroid_mm, measure| GeometricSignature {
            kind,
            normal_or_axis,
            centroid_mm,
            characteristic_measure: measure,
            aabb_mm: aabb,
        };

        let faces = vec![
            BRepFace {
                id: FaceId(0),
                body_id: BodyId(0),
                surface: SurfaceKind::Cylinder {
                    axis_origin_mm: DVec3::new(center.x, center.y, floor_z),
                    axis_dir: DVec3::Z,
                    radius_mm: fillet_r_mm,
                    is_internal: true,
                },
                area_mm2: 0.5 * PI * fillet_r_mm * pocket_depth_mm,
                aabb_mm: aabb,
                outer_loop: None,
                inner_loops: Vec::new(),
                signature: make_sig(
                    EntitySignatureKind::InternalCylinderFace,
                    DVec3::Z,
                    DVec3::new(center.x, center.y, 0.5 * (top_z + floor_z)),
                    fillet_r_mm,
                ),
            },
            BRepFace {
                id: FaceId(1),
                body_id: BodyId(0),
                surface: SurfaceKind::Plane {
                    origin_mm: DVec3::new(20.0, 20.0, floor_z),
                    normal: DVec3::Z,
                },
                area_mm2: 1600.0,
                aabb_mm: Aabb3::new(
                    DVec3::new(0.0, 0.0, floor_z),
                    DVec3::new(40.0, 40.0, floor_z),
                ),
                outer_loop: None,
                inner_loops: Vec::new(),
                signature: make_sig(
                    EntitySignatureKind::PlanarFace,
                    DVec3::Z,
                    DVec3::new(20.0, 20.0, floor_z),
                    1600.0,
                ),
            },
            BRepFace {
                id: FaceId(2),
                body_id: BodyId(0),
                surface: SurfaceKind::Plane {
                    origin_mm: DVec3::new(20.0, 20.0, top_z),
                    normal: DVec3::Z,
                },
                area_mm2: 800.0,
                aabb_mm: Aabb3::new(DVec3::new(0.0, 0.0, top_z), DVec3::new(40.0, 40.0, top_z)),
                outer_loop: None,
                inner_loops: Vec::new(),
                signature: make_sig(
                    EntitySignatureKind::PlanarFace,
                    DVec3::Z,
                    DVec3::new(20.0, 20.0, top_z),
                    800.0,
                ),
            },
        ];

        CadDocument {
            source_name: "synthetic_pocket.step".to_string(),
            detected_unit: StepUnitKind::Millimeter,
            bodies: vec![SolidBody {
                id: BodyId(0),
                name: "PocketBody".to_string(),
                faces: vec![FaceId(0), FaceId(1), FaceId(2)],
                edges: Vec::new(),
                aabb_mm: aabb,
            }],
            meshes: Vec::new(),
            faces,
            loops: Vec::new(),
            edges: Vec::new(),
            aabb_mm: aabb,
            render_mesh: RenderMesh {
                vertices: Vec::new(),
                indices: Vec::new(),
                pick_triangles,
                edge_polylines: Vec::new(),
            },
        }
    }

    fn zero_margin_setup(doc: &CadDocument) -> (Setup, ResolvedSetup) {
        let mut setup = Setup::new("setup_1", "Op 1", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup.stock = StockConfig::RelativeBox {
            side_pos_x: ParamLength::from_mm(0.0),
            side_neg_x: ParamLength::from_mm(0.0),
            side_pos_y: ParamLength::from_mm(0.0),
            side_neg_y: ParamLength::from_mm(0.0),
            symmetric_xy: true,
            top_offset_z: ParamLength::from_mm(0.0),
            bottom_offset_z: ParamLength::from_mm(0.0),
            sheet_thickness_z: ParamLength::from_mm(12.0),
            round_increment_mm: None,
        };
        let rs = setup.resolve(doc, DisplayUnit::Millimeter).unwrap();
        (setup, rs)
    }

    #[test]
    fn concave_r2_fillet_flags_tight_corner_for_6mm_cutter_and_clears_for_3_175mm_cutter() {
        let doc = synthetic_filleted_deep_pocket_doc(2.0, 10.0);
        let (setup, rs) = zero_margin_setup(&doc);

        let tool_6mm = make_flat_endmill(6.0, 18.0, 25.0);
        let res_6 = analyze_machinability_reach(&doc, &[], &setup, &rs, &tool_6mm);
        assert_eq!(res_6.min_internal_radius_mm, Some(2.0));
        assert_eq!(res_6.tight_corner_count, 1);
        assert_eq!(res_6.narrow_slot_count, 0);
        assert_eq!(res_6.crimson_count, 0);
        assert!(res_6.amber_count >= 1);
        assert!(res_6.highlight_faces.iter().any(|f| {
            f.face_id == Some(FaceId(0))
                && f.kind == ReachIssueKind::TightCorner
                && f.severity == ReachSeverity::Warning
        }));

        let tool_1_8in = make_flat_endmill(3.175, 18.0, 25.0);
        let res_small = analyze_machinability_reach(&doc, &[], &setup, &rs, &tool_1_8in);
        assert_eq!(res_small.min_internal_radius_mm, Some(2.0));
        assert_eq!(res_small.tight_corner_count, 0);
        assert_eq!(res_small.amber_count, 0);
        assert_eq!(res_small.crimson_count, 0);
    }

    #[test]
    fn deep_18mm_pocket_flags_loc_and_stickout_while_passing_relieved_neck_cutter() {
        let doc = synthetic_filleted_deep_pocket_doc(4.0, 18.0);
        let (setup, rs) = zero_margin_setup(&doc);

        // 1. 12 mm LOC, 25 mm stickout => ExceedsFluteLength (Amber), 0 Crimson
        let loc_short = make_flat_endmill(6.0, 12.0, 25.0);
        let r_loc = analyze_machinability_reach(&doc, &[], &setup, &rs, &loc_short);
        assert!((r_loc.max_feature_depth_mm - 18.0).abs() < 1e-6);
        assert!(r_loc.flute_exceeded_count >= 1);
        assert_eq!(r_loc.stickout_exceeded_count, 0);
        assert_eq!(r_loc.crimson_count, 0);
        assert!(r_loc.amber_count >= 1);

        // 2. 12 mm LOC, 15 mm stickout => ExceedsStickout (Crimson)
        let stickout_short = make_flat_endmill(6.0, 12.0, 15.0);
        let r_stick = analyze_machinability_reach(&doc, &[], &setup, &rs, &stickout_short);
        assert!(r_stick.stickout_exceeded_count >= 1);
        assert!(r_stick.crimson_count >= 1);

        // 3. Relieved-neck cutter (6 mm head, 4 mm neck, 5 mm flute, 20 mm neck reach, 25 mm stickout)
        //    clears the 18 mm pocket depth cleanly!
        let relieved = make_relieved_neck_tool(6.0, 4.0, 5.0, 20.0, 25.0);
        let r_rel = analyze_machinability_reach(&doc, &[], &setup, &rs, &relieved);
        assert_eq!(r_rel.flute_exceeded_count, 0);
        assert_eq!(r_rel.stickout_exceeded_count, 0);
        assert_eq!(r_rel.amber_count, 0);
        assert_eq!(r_rel.crimson_count, 0);
    }

    #[test]
    fn sketch_rounded_rect_and_small_circle_flag_tight_corners_and_blocked_hole() {
        let mut sk = SketchLayer::new("sk_1", "Panel", "setup_1");
        sk.entities.push(SketchEntityEntry {
            id: "r1".to_string(),
            name: "Pocket".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::Rectangle {
                anchor: RectAnchorMode::FrontLeft,
                x: ParamLength::from_mm(0.0),
                y: ParamLength::from_mm(0.0),
                width: ParamLength::from_mm(50.0),
                height: ParamLength::from_mm(30.0),
                corner_radius: ParamLength::from_mm(2.0),
                rotation_deg: 0.0,
            },
        });
        sk.entities.push(SketchEntityEntry {
            id: "c1".to_string(),
            name: "Small Hole".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::new_circle(
                ParamLength::from_mm(25.0),
                ParamLength::from_mm(15.0),
                ParamLength::from_mm(4.0),
            ),
        });

        let setup = Setup::new("setup_1", "Pure 2D", WorkOffsetSlot::G54);
        let rs = setup
            .resolve_with_sketches(
                &CadDocument::empty(),
                &[sk.clone()],
                DisplayUnit::Millimeter,
            )
            .unwrap();
        let tool_6mm = make_flat_endmill(6.0, 15.0, 25.0);

        let res = analyze_machinability_reach(&CadDocument::empty(), &[sk], &setup, &rs, &tool_6mm);
        assert_eq!(res.min_internal_radius_mm, Some(2.0));
        assert_eq!(res.tight_corner_count, 4);
        assert_eq!(res.narrow_slot_count, 1);
        assert_eq!(res.amber_count, 4);
        assert_eq!(res.crimson_count, 1);
    }
}
