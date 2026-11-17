use chip_cad::{
    BodyId, CadDocument, CurveKind, EdgeId, FaceId, LoopId, OrientedEdgeUse, SelectionBinding,
    SurfaceKind, group_coaxial_holes, highest_adjacent_wall_top,
};
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3, RigidFrame3};
use serde::{Deserialize, Serialize};

use crate::{setup::ResolvedSetup, wcs::CamError};

/// Persistent interactive 3D geometry selection item inside an [`crate::operation::Operation`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum GeometrySelectionItem {
    PlanarFace {
        binding: SelectionBinding,
    },
    ClosedLoop {
        binding: SelectionBinding,
        #[serde(default)]
        reverse: bool,
        #[serde(default)]
        side_flip: bool,
    },
    OpenChain {
        body_id: BodyId,
        start_edge: SelectionBinding,
        end_edge: SelectionBinding,
        resolved_edges: Vec<OrientedEdgeUse>,
        #[serde(default)]
        reverse: bool,
        #[serde(default)]
        side_flip: bool,
    },
    CylindricalHole {
        binding: SelectionBinding,
    },
}

impl GeometrySelectionItem {
    pub fn short_label(&self) -> String {
        match self {
            Self::PlanarFace { binding } => format!("Planar Face #{}", binding.last_index),
            Self::ClosedLoop {
                binding,
                reverse,
                side_flip,
            } => {
                let dir = if *reverse { "Rev" } else { "Fwd" };
                let side = if *side_flip { "Right" } else { "Left" };
                format!("Closed Loop #{} ({dir}, {side})", binding.last_index)
            }
            Self::OpenChain {
                start_edge,
                end_edge,
                resolved_edges,
                reverse,
                side_flip,
                ..
            } => {
                let dir = if *reverse { "Rev" } else { "Fwd" };
                let side = if *side_flip { "Right" } else { "Left" };
                format!(
                    "Open Chain E#{}->E#{} ({} edges, {dir}, {side})",
                    start_edge.last_index,
                    end_edge.last_index,
                    resolved_edges.len()
                )
            }
            Self::CylindricalHole { binding } => {
                format!("Cylindrical Hole #{}", binding.last_index)
            }
        }
    }
}

/// Extracted 2.5D CAM geometry in canonical Setup WCS millimeters (`f64`).
#[derive(Debug, Clone, PartialEq)]
pub enum ExtractedFeature25D {
    PlanarFace {
        face_id: FaceId,
        outer_boundary_wcs: ArcPolyline2D,
        inner_islands_wcs: Vec<ArcPolyline2D>,
        face_z_wcs: f64,
        inferred_top_z_wcs: f64,
        area_mm2: f64,
    },
    Contour {
        polyline_wcs: ArcPolyline2D,
        min_z_wcs: f64,
        max_z_wcs: f64,
        reverse: bool,
        side_flip: bool,
    },
    CylindricalHole {
        primary_face: FaceId,
        coaxial_faces: Vec<FaceId>,
        center_wcs: DVec2,
        diameter_mm: f64,
        top_z_wcs: f64,
        bottom_z_wcs: f64,
        is_through_hole: bool,
    },
}

/// 3D arrow indicator rendered in the viewport to show chain travel direction and offset side.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChainDirectionArrow3D {
    pub anchor_world_mm: DVec3,
    pub tangent_world: DVec3,
    pub side_normal_world: DVec3,
}

/// Aggregated result of extracting all selections of an operation in active Setup WCS coordinates.
#[derive(Debug, Clone, Default)]
pub struct ExtractedSelectionSummary {
    pub features: Vec<ExtractedFeature25D>,
    pub highlight_faces: Vec<FaceId>,
    pub highlight_edges: Vec<EdgeId>,
    pub direction_arrows: Vec<ChainDirectionArrow3D>,
    pub inferred_top_z_wcs: f64,
    pub inferred_bottom_z_wcs: f64,
    /// Optional primary hole diameter (`mm`) injected as `hole_d` into `EvalContext`.
    pub primary_hole_diameter_mm: Option<f64>,
}

/// Projects and validates an operation's [`GeometrySelectionItem`] list into active Setup WCS
/// `f64` features and infers default `Top` / `Bottom` Z heights (`02-cad-selection.md`).
pub fn extract_selection_features(
    doc: &CadDocument,
    setup: &ResolvedSetup,
    selections: &[GeometrySelectionItem],
) -> Result<ExtractedSelectionSummary, CamError> {
    let stock_top_z = setup.stock.aabb_wcs().max.z;
    let model_top_z = setup.model_aabb_wcs.max.z;

    if selections.is_empty() {
        return Ok(ExtractedSelectionSummary {
            features: Vec::new(),
            highlight_faces: Vec::new(),
            highlight_edges: Vec::new(),
            direction_arrows: Vec::new(),
            inferred_top_z_wcs: stock_top_z,
            inferred_bottom_z_wcs: model_top_z,
            primary_hole_diameter_mm: None,
        });
    }

    let mut features = Vec::new();
    let mut highlight_faces = Vec::new();
    let mut highlight_edges = Vec::new();
    let mut direction_arrows = Vec::new();
    let mut top_candidates = Vec::new();
    let mut bottom_candidates = Vec::new();
    let mut raw_hole_faces = Vec::new();

    for item in selections {
        match item {
            GeometrySelectionItem::PlanarFace { binding } => {
                let fid = FaceId(binding.last_index);
                let face = doc.face(fid).ok_or(CamError::MissingSelectedEntity {
                    index: binding.last_index,
                })?;
                let SurfaceKind::Plane { origin_mm, normal } = face.surface else {
                    return Err(CamError::FaceNotHorizontalInWcs {
                        face_id: fid.0,
                        cos_z: 0.0,
                    });
                };
                let n_wcs = setup.frame.world_to_local_dir(normal.normalize_or_zero());
                if n_wcs.z.abs() < 0.999 {
                    return Err(CamError::FaceNotHorizontalInWcs {
                        face_id: fid.0,
                        cos_z: n_wcs.z.abs(),
                    });
                }
                let face_z_wcs = setup.frame.world_to_local_point(origin_mm).z;
                let outer_boundary_wcs = match face.outer_loop {
                    Some(lid) => project_loop_to_wcs_xy(doc, &setup.frame, lid)?.0,
                    None => ArcPolyline2D::default(),
                };
                let mut inner_islands_wcs = Vec::with_capacity(face.inner_loops.len());
                for &ilid in &face.inner_loops {
                    inner_islands_wcs.push(project_loop_to_wcs_xy(doc, &setup.frame, ilid)?.0);
                }

                // Pocket Top Z inference: highest adjacent wall top Z in WCS (or Stock Top if open)
                let inferred_top = highest_adjacent_wall_top(doc, fid, setup.frame.z_axis)
                    .map(|world_proj| {
                        let wcs_origin_proj = setup.frame.origin_mm.dot(setup.frame.z_axis);
                        world_proj - wcs_origin_proj
                    })
                    .unwrap_or(stock_top_z);

                if !highlight_faces.contains(&fid) {
                    highlight_faces.push(fid);
                }
                top_candidates.push(inferred_top);
                bottom_candidates.push(face_z_wcs);
                features.push(ExtractedFeature25D::PlanarFace {
                    face_id: fid,
                    outer_boundary_wcs,
                    inner_islands_wcs,
                    face_z_wcs,
                    inferred_top_z_wcs: inferred_top,
                    area_mm2: face.area_mm2,
                });
            }
            GeometrySelectionItem::ClosedLoop {
                binding,
                reverse,
                side_flip,
            } => {
                let lid = LoopId(binding.last_index);
                let lp = doc.brep_loop(lid).ok_or(CamError::MissingSelectedEntity {
                    index: binding.last_index,
                })?;
                for u in &lp.edges {
                    if !highlight_edges.contains(&u.edge_id) {
                        highlight_edges.push(u.edge_id);
                    }
                }
                let (mut poly, min_z, max_z) = project_loop_to_wcs_xy(doc, &setup.frame, lid)?;
                if *reverse {
                    poly = poly.reverse();
                }
                if let Some(arrow) =
                    build_chain_arrow(&poly, 0.5 * (min_z + max_z), *side_flip, &setup.frame)
                {
                    direction_arrows.push(arrow);
                }

                let top_z = if min_z < model_top_z - 1e-4 {
                    model_top_z
                } else {
                    stock_top_z
                };
                top_candidates.push(top_z);
                bottom_candidates.push(min_z);
                features.push(ExtractedFeature25D::Contour {
                    polyline_wcs: poly,
                    min_z_wcs: min_z,
                    max_z_wcs: max_z,
                    reverse: *reverse,
                    side_flip: *side_flip,
                });
            }
            GeometrySelectionItem::OpenChain {
                resolved_edges,
                reverse,
                side_flip,
                start_edge,
                ..
            } => {
                if resolved_edges.is_empty() {
                    return Err(CamError::MissingSelectedEntity {
                        index: start_edge.last_index,
                    });
                }
                for u in resolved_edges {
                    if !highlight_edges.contains(&u.edge_id) {
                        highlight_edges.push(u.edge_id);
                    }
                }
                let (mut poly, min_z, max_z) =
                    project_oriented_edges_to_wcs_xy(doc, &setup.frame, resolved_edges, false)?;
                if *reverse {
                    poly = poly.reverse();
                }
                if let Some(arrow) =
                    build_chain_arrow(&poly, 0.5 * (min_z + max_z), *side_flip, &setup.frame)
                {
                    direction_arrows.push(arrow);
                }

                let top_z = if min_z < model_top_z - 1e-4 {
                    model_top_z
                } else {
                    stock_top_z
                };
                top_candidates.push(top_z);
                bottom_candidates.push(min_z);
                features.push(ExtractedFeature25D::Contour {
                    polyline_wcs: poly,
                    min_z_wcs: min_z,
                    max_z_wcs: max_z,
                    reverse: *reverse,
                    side_flip: *side_flip,
                });
            }
            GeometrySelectionItem::CylindricalHole { binding } => {
                let fid = FaceId(binding.last_index);
                let face = doc.face(fid).ok_or(CamError::MissingSelectedEntity {
                    index: binding.last_index,
                })?;
                let SurfaceKind::Cylinder {
                    axis_dir,
                    is_internal: true,
                    ..
                } = face.surface
                else {
                    return Err(CamError::HoleNotAlignedWithWcsZ {
                        face_id: fid.0,
                        cos_z: 0.0,
                    });
                };
                let axis_wcs = setup.frame.world_to_local_dir(axis_dir.normalize_or_zero());
                if axis_wcs.z.abs() < 0.999 {
                    return Err(CamError::HoleNotAlignedWithWcsZ {
                        face_id: fid.0,
                        cos_z: axis_wcs.z.abs(),
                    });
                }
                raw_hole_faces.push(fid);
            }
        }
    }

    // Group coaxial half-cylinder hole faces so split STEP holes produce 1 logical Hole25D!
    let mut primary_hole_diameter_mm = None;
    if !raw_hole_faces.is_empty() {
        let grouped = group_coaxial_holes(doc, &raw_hole_faces, setup.frame.z_axis);
        let origin_proj = setup.frame.origin_mm.dot(setup.frame.z_axis);
        for h in grouped {
            for &gfid in &h.faces {
                if !highlight_faces.contains(&gfid) {
                    highlight_faces.push(gfid);
                }
            }
            let c_wcs = setup.frame.world_to_local_point(h.axis_origin_mm);
            let top_z_wcs = h.span.top_proj_mm - origin_proj;
            let bottom_z_wcs = h.span.bottom_proj_mm - origin_proj;
            let diameter_mm = 2.0 * h.radius_mm;
            if primary_hole_diameter_mm.is_none() {
                primary_hole_diameter_mm = Some(diameter_mm);
            }
            top_candidates.push(top_z_wcs);
            bottom_candidates.push(bottom_z_wcs);
            features.push(ExtractedFeature25D::CylindricalHole {
                primary_face: h.primary_face,
                coaxial_faces: h.faces,
                center_wcs: DVec2::new(c_wcs.x, c_wcs.y),
                diameter_mm,
                top_z_wcs,
                bottom_z_wcs,
                is_through_hole: h.span.is_through_hole,
            });
        }
    }

    let inferred_top_z_wcs = top_candidates
        .into_iter()
        .reduce(f64::max)
        .unwrap_or(stock_top_z);
    let inferred_bottom_z_wcs = bottom_candidates
        .into_iter()
        .reduce(f64::min)
        .unwrap_or(model_top_z);

    Ok(ExtractedSelectionSummary {
        features,
        highlight_faces,
        highlight_edges,
        direction_arrows,
        inferred_top_z_wcs,
        inferred_bottom_z_wcs,
        primary_hole_diameter_mm,
    })
}

fn build_chain_arrow(
    poly_wcs: &ArcPolyline2D,
    z_wcs: f64,
    side_flip: bool,
    frame: &RigidFrame3,
) -> Option<ChainDirectionArrow3D> {
    let (pt2, tan2) = poly_wcs.sample_at_fraction(0.25)?;
    let left_n2 = DVec2::new(-tan2.y, tan2.x);
    let side2 = if side_flip { -left_n2 } else { left_n2 };

    let anchor_world_mm = frame.local_to_world_point(DVec3::new(pt2.x, pt2.y, z_wcs));
    let tangent_world = frame
        .local_to_world_dir(DVec3::new(tan2.x, tan2.y, 0.0))
        .normalize_or_zero();
    let side_normal_world = frame
        .local_to_world_dir(DVec3::new(side2.x, side2.y, 0.0))
        .normalize_or_zero();

    Some(ChainDirectionArrow3D {
        anchor_world_mm,
        tangent_world,
        side_normal_world,
    })
}

fn project_loop_to_wcs_xy(
    doc: &CadDocument,
    frame: &RigidFrame3,
    loop_id: LoopId,
) -> Result<(ArcPolyline2D, f64, f64), CamError> {
    let lp = doc
        .brep_loop(loop_id)
        .ok_or(CamError::MissingSelectedEntity { index: loop_id.0 })?;
    project_oriented_edges_to_wcs_xy(doc, frame, &lp.edges, true)
}

fn project_oriented_edges_to_wcs_xy(
    doc: &CadDocument,
    frame: &RigidFrame3,
    uses: &[OrientedEdgeUse],
    closed: bool,
) -> Result<(ArcPolyline2D, f64, f64), CamError> {
    let mut segments = Vec::with_capacity(uses.len());
    let mut min_z = f64::INFINITY;
    let mut max_z = f64::NEG_INFINITY;

    for u in uses {
        let edge = doc
            .edge(u.edge_id)
            .ok_or(CamError::MissingSelectedEntity { index: u.edge_id.0 })?;

        // Check discretized polyline endpoints to determine whether `CurveKind` direction matches
        // the mesh/trimmed edge's intrinsic orientation!
        let poly_pts = doc
            .render_mesh
            .edge_polylines
            .iter()
            .find(|ep| ep.edge_id == u.edge_id)
            .map(|ep| ep.points_mm.as_slice());

        match edge.curve {
            CurveKind::Line { start_mm, end_mm } => {
                let mut p_start = start_mm;
                let mut p_end = end_mm;
                if let Some(pts) = poly_pts
                    && let Some(&first) = pts.first()
                    && first.distance(end_mm) < first.distance(start_mm)
                {
                    std::mem::swap(&mut p_start, &mut p_end);
                }
                if !u.forward {
                    std::mem::swap(&mut p_start, &mut p_end);
                }
                let s_wcs = frame.world_to_local_point(p_start);
                let e_wcs = frame.world_to_local_point(p_end);
                min_z = min_z.min(s_wcs.z).min(e_wcs.z);
                max_z = max_z.max(s_wcs.z).max(e_wcs.z);
                let s2 = DVec2::new(s_wcs.x, s_wcs.y);
                let e2 = DVec2::new(e_wcs.x, e_wcs.y);
                if s2.distance(e2) > 1e-6 {
                    segments.push(ArcSegment2D::Line { start: s2, end: e2 });
                }
            }
            CurveKind::CircleArc {
                center_mm,
                normal,
                radius_mm,
                start_mm,
                end_mm,
                is_full_circle,
            } => {
                let n_wcs = frame.world_to_local_dir(normal.normalize_or_zero());
                if n_wcs.z.abs() >= 0.999 {
                    // True horizontal circular arc in WCS XY -> preserve exact `ArcSegment2D::Arc`!
                    let c_wcs = frame.world_to_local_point(center_mm);
                    min_z = min_z.min(c_wcs.z);
                    max_z = max_z.max(c_wcs.z);
                    let c2 = DVec2::new(c_wcs.x, c_wcs.y);

                    let intrinsic_ccw = determine_arc_ccw_in_wcs(c2, n_wcs.z, poly_pts, frame);
                    let ccw = if u.forward {
                        intrinsic_ccw
                    } else {
                        !intrinsic_ccw
                    };

                    if is_full_circle {
                        // Split full 360° circle into two exact 180° semicircles so start != end!
                        let s_wcs = frame.world_to_local_point(start_mm);
                        let mut p0 = DVec2::new(s_wcs.x, s_wcs.y);
                        if (p0 - c2).length() < 1e-6 {
                            p0 = c2 + DVec2::new(radius_mm, 0.0);
                        } else {
                            p0 = c2 + (p0 - c2).normalize() * radius_mm;
                        }
                        let p_opp = c2 - (p0 - c2);
                        segments.push(ArcSegment2D::Arc {
                            start: p0,
                            end: p_opp,
                            center: c2,
                            radius: radius_mm,
                            ccw,
                        });
                        segments.push(ArcSegment2D::Arc {
                            start: p_opp,
                            end: p0,
                            center: c2,
                            radius: radius_mm,
                            ccw,
                        });
                    } else {
                        let (mut raw_s, mut raw_e) = (start_mm, end_mm);
                        if let Some(pts) = poly_pts
                            && let (Some(&first), Some(&last)) = (pts.first(), pts.last())
                            && first.distance(end_mm) < first.distance(start_mm)
                        {
                            raw_s = last;
                            raw_e = first;
                            std::mem::swap(&mut raw_s, &mut raw_e);
                        } else if let Some(pts) = poly_pts
                            && let (Some(&first), Some(&last)) = (pts.first(), pts.last())
                        {
                            raw_s = first;
                            raw_e = last;
                        }
                        if !u.forward {
                            std::mem::swap(&mut raw_s, &mut raw_e);
                        }
                        let s_wcs = frame.world_to_local_point(raw_s);
                        let e_wcs = frame.world_to_local_point(raw_e);
                        min_z = min_z.min(s_wcs.z).min(e_wcs.z);
                        max_z = max_z.max(s_wcs.z).max(e_wcs.z);
                        segments.push(ArcSegment2D::Arc {
                            start: DVec2::new(s_wcs.x, s_wcs.y),
                            end: DVec2::new(e_wcs.x, e_wcs.y),
                            center: c2,
                            radius: radius_mm,
                            ccw,
                        });
                    }
                } else if let Some(pts) = poly_pts {
                    append_discretized_edge(
                        pts,
                        u.forward,
                        frame,
                        &mut segments,
                        &mut min_z,
                        &mut max_z,
                    );
                }
            }
            CurveKind::EllipseArc | CurveKind::OtherNurbs => {
                if let Some(pts) = poly_pts {
                    append_discretized_edge(
                        pts,
                        u.forward,
                        frame,
                        &mut segments,
                        &mut min_z,
                        &mut max_z,
                    );
                }
            }
        }
    }

    if !min_z.is_finite() {
        min_z = 0.0;
        max_z = 0.0;
    }

    Ok((ArcPolyline2D { segments, closed }, min_z, max_z))
}

fn determine_arc_ccw_in_wcs(
    center_xy: DVec2,
    normal_z: f64,
    poly_pts: Option<&[DVec3]>,
    frame: &RigidFrame3,
) -> bool {
    if let Some(pts) = poly_pts
        && pts.len() >= 2
    {
        let p0 = frame.world_to_local_point(pts[0]);
        let p1 = frame.world_to_local_point(pts[1]);
        let r0 = DVec2::new(p0.x, p0.y) - center_xy;
        let step = DVec2::new(p1.x - p0.x, p1.y - p0.y);
        let cross_z = r0.x * step.y - r0.y * step.x;
        if cross_z.abs() > 1e-9 {
            return cross_z > 0.0;
        }
    }
    normal_z > 0.0
}

fn append_discretized_edge(
    pts_world: &[DVec3],
    forward: bool,
    frame: &RigidFrame3,
    segments: &mut Vec<ArcSegment2D>,
    min_z: &mut f64,
    max_z: &mut f64,
) {
    let iter: Box<dyn Iterator<Item = &DVec3>> = if forward {
        Box::new(pts_world.iter())
    } else {
        Box::new(pts_world.iter().rev())
    };
    let projected: Vec<DVec3> = iter.map(|&p| frame.world_to_local_point(p)).collect();
    for p in &projected {
        *min_z = min_z.min(p.z);
        *max_z = max_z.max(p.z);
    }
    for w in projected.windows(2) {
        let s2 = DVec2::new(w[0].x, w[0].y);
        let e2 = DVec2::new(w[1].x, w[1].y);
        if s2.distance(e2) > 1e-6 {
            segments.push(ArcSegment2D::Line { start: s2, end: e2 });
        }
    }
}
