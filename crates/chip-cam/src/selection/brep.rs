use std::collections::HashSet;

use chip_cad::{
    CadDocument, CurveKind, EdgeId, FaceId, LoopId, OrientedEdgeUse, SelectionBinding, SurfaceKind,
    group_coaxial_holes,
};
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3, RigidFrame3};

use crate::{setup::ResolvedSetup, wcs::CamError};

use super::{ChainDirectionArrow3D, ExtractedFeature25D, build_chain_arrow};

pub(super) struct BRepExtractAccum<'a> {
    pub(super) doc: &'a CadDocument,
    pub(super) setup: &'a ResolvedSetup,
    pub(super) stock_top_z: f64,
    pub(super) model_top_z: f64,
    pub(super) wcs_origin_proj: f64,
    pub(super) features: &'a mut Vec<ExtractedFeature25D>,
    pub(super) highlight_faces: &'a mut Vec<FaceId>,
    pub(super) highlight_edges: &'a mut Vec<EdgeId>,
    pub(super) direction_arrows: &'a mut Vec<ChainDirectionArrow3D>,
    pub(super) top_candidates: &'a mut Vec<f64>,
    pub(super) bottom_candidates: &'a mut Vec<f64>,
    pub(super) raw_hole_faces: &'a mut Vec<FaceId>,
    pub(super) primary_hole_diameter_mm: &'a mut Option<f64>,
}

pub(super) fn extract_planar_face(
    acc: &mut BRepExtractAccum<'_>,
    binding: &SelectionBinding,
) -> Result<(), CamError> {
    let doc = acc.doc;
    let setup = acc.setup;
    let stock_top_z = acc.stock_top_z;
    let wcs_origin_proj = acc.wcs_origin_proj;
    let highlight_faces = &mut *acc.highlight_faces;
    let top_candidates = &mut *acc.top_candidates;
    let bottom_candidates = &mut *acc.bottom_candidates;
    let features = &mut *acc.features;
    let fid = FaceId(binding.last_index);
    let face = doc.face(fid).ok_or(CamError::MissingSelectedEntity {
        index: binding.last_index,
    })?;

    // Check horizontal planar face first; otherwise check modeled angled chamfer face!
    let horizontal_plane = match face.surface {
        SurfaceKind::Plane { origin_mm, normal } => {
            let n_wcs = setup.frame.world_to_local_dir(normal.normalize_or_zero());
            if n_wcs.z.abs() >= 0.999 {
                Some(origin_mm)
            } else {
                None
            }
        }
        _ => None,
    };

    if let Some(origin_mm) = horizontal_plane {
        let face_z_wcs = setup.frame.world_to_local_point(origin_mm).z;
        let cls = classify_planar_face_loops(doc, fid, setup.frame.z_axis);
        let mut outer_edge_ids = Vec::new();
        let (outer_boundary_wcs, open_outer_segments) = match face.outer_loop {
            Some(lid) => {
                if let Some(lp) = doc.brep_loop(lid) {
                    outer_edge_ids.extend(lp.edges.iter().map(|u| u.edge_id));
                }
                let (p, flags, _, _) = project_loop_to_wcs_xy_with_open_flags(
                    doc,
                    &setup.frame,
                    lid,
                    Some(&cls.open_outer_edges),
                )?;
                (p, flags)
            }
            None => (ArcPolyline2D::default(), Vec::new()),
        };
        let mut inner_islands_wcs = Vec::with_capacity(face.inner_loops.len());
        let mut raised_islands_wcs = Vec::new();
        for &ilid in &face.inner_loops {
            let poly = project_loop_to_wcs_xy(doc, &setup.frame, ilid)?.0;
            if cls.raised_inner_loops.contains(&ilid) {
                raised_islands_wcs.push(poly.clone());
            }
            inner_islands_wcs.push(poly);
        }

        // Pocket Top Z inference: highest adjacent wall top Z in WCS (or Stock Top if open)
        let inferred_top = highest_adjacent_wall_top(doc, fid, setup.frame.z_axis)
            .map(|world_proj| world_proj - wcs_origin_proj)
            .unwrap_or(stock_top_z);

        let adjacent_lower_floor_z_wcs = if outer_edge_ids.is_empty() {
            None
        } else {
            highest_adjacent_lower_floor_proj(
                doc,
                &outer_edge_ids,
                face_z_wcs + wcs_origin_proj,
                setup.frame.z_axis,
            )
            .map(|p| p - wcs_origin_proj)
        };

        if !highlight_faces.contains(&fid) {
            highlight_faces.push(fid);
        }
        top_candidates.push(inferred_top);
        bottom_candidates.push(face_z_wcs);
        features.push(ExtractedFeature25D::PlanarFace {
            source_face_id: Some(fid),
            outer_boundary_wcs,
            open_outer_segments,
            inner_islands_wcs,
            raised_islands_wcs,
            face_z_wcs,
            inferred_top_z_wcs: inferred_top,
            adjacent_lower_floor_z_wcs,
            area_mm2: face.area_mm2,
        });
    } else if let Some(strip) = extract_modeled_chamfer_strip(doc, fid, setup.frame.z_axis) {
        let top_z_wcs = strip.top_proj_mm - wcs_origin_proj;
        let bottom_z_wcs = strip.bottom_proj_mm - wcs_origin_proj;
        let upper_closed = is_oriented_uses_closed(doc, &strip.upper_edges);
        let lower_closed = is_oriented_uses_closed(doc, &strip.lower_edges);
        let upper_polyline_wcs = if strip.upper_edges.is_empty() {
            ArcPolyline2D::default()
        } else {
            project_oriented_edges_to_wcs_xy(doc, &setup.frame, &strip.upper_edges, upper_closed)?.0
        };
        let lower_polyline_wcs = if strip.lower_edges.is_empty() {
            ArcPolyline2D::default()
        } else {
            project_oriented_edges_to_wcs_xy(doc, &setup.frame, &strip.lower_edges, lower_closed)?.0
        };
        let mut edge_ids = Vec::new();
        for u in strip.upper_edges.iter().chain(strip.lower_edges.iter()) {
            if !edge_ids.contains(&u.edge_id) {
                edge_ids.push(u.edge_id);
            }
        }
        let outward_normal_xy_wcs = match face.surface {
            SurfaceKind::Plane { normal, .. } => {
                let n_wcs = setup.frame.world_to_local_dir(normal.normalize_or_zero());
                let xy = DVec2::new(n_wcs.x, n_wcs.y).normalize_or_zero();
                if xy.length_squared() > 1e-12 {
                    Some(xy)
                } else {
                    None
                }
            }
            _ => None,
        };
        let adjacent_lower_floor_z_wcs = if edge_ids.is_empty() {
            None
        } else {
            highest_adjacent_lower_floor_proj(
                doc,
                &edge_ids,
                top_z_wcs + wcs_origin_proj,
                setup.frame.z_axis,
            )
            .map(|p| p - wcs_origin_proj)
        };
        if !highlight_faces.contains(&fid) {
            highlight_faces.push(fid);
        }
        top_candidates.push(top_z_wcs);
        bottom_candidates.push(bottom_z_wcs);
        features.push(ExtractedFeature25D::ModeledChamfer {
            source_face_id: Some(fid),
            upper_polyline_wcs,
            lower_polyline_wcs,
            radial_width_mm: strip.radial_width_mm,
            axial_drop_mm: strip.axial_drop_mm,
            bevel_half_angle_rad: strip.bevel_half_angle_rad,
            top_z_wcs,
            bottom_z_wcs,
            outward_normal_xy_wcs,
            adjacent_lower_floor_z_wcs,
            edge_ids,
        });
    } else {
        let cos_z = match face.surface {
            SurfaceKind::Plane { normal, .. } => setup
                .frame
                .world_to_local_dir(normal.normalize_or_zero())
                .z
                .abs(),
            _ => 0.0,
        };
        return Err(CamError::FaceNotHorizontalInWcs {
            face_id: fid.0,
            cos_z,
        });
    }

    Ok(())
}

pub(super) fn extract_closed_loop(
    acc: &mut BRepExtractAccum<'_>,
    binding: &SelectionBinding,
    reverse: bool,
    side_flip: bool,
) -> Result<(), CamError> {
    let doc = acc.doc;
    let setup = acc.setup;
    let stock_top_z = acc.stock_top_z;
    let model_top_z = acc.model_top_z;
    let wcs_origin_proj = acc.wcs_origin_proj;
    let highlight_edges = &mut *acc.highlight_edges;
    let direction_arrows = &mut *acc.direction_arrows;
    let top_candidates = &mut *acc.top_candidates;
    let bottom_candidates = &mut *acc.bottom_candidates;
    let features = &mut *acc.features;
    let reverse = &reverse;
    let side_flip = &side_flip;
    let lid = LoopId(binding.last_index);
    let lp = doc.brep_loop(lid).ok_or(CamError::MissingSelectedEntity {
        index: binding.last_index,
    })?;
    let mut source_edge_ids = Vec::with_capacity(lp.edges.len());
    for u in &lp.edges {
        source_edge_ids.push(u.edge_id);
        if !highlight_edges.contains(&u.edge_id) {
            highlight_edges.push(u.edge_id);
        }
    }
    let (mut poly, min_z, max_z) = project_loop_to_wcs_xy(doc, &setup.frame, lid)?;
    // Normalize closed loop winding relative to solid material in WCS XY before applying `reverse`:
    // - Outer face loops and raised boss base loops wind CW (`want_ccw = false`) so `LeftClimb`
    //   offsets outward into open air (Climb Milling around the exterior).
    // - Internal pocket/hole cutout loops wind CCW (`want_ccw = true`) so `LeftClimb`
    //   offsets inward into the cutout!
    if poly.closed && poly.signed_area().abs() > 1e-6 {
        let want_ccw = if lp.is_outer {
            false
        } else {
            let cls = classify_planar_face_loops(doc, lp.face_id, setup.frame.z_axis);
            !cls.raised_inner_loops.contains(&lid)
        };
        poly = poly.ensure_orientation(want_ccw);
    }
    if *reverse {
        poly = poly.reverse();
    }
    if let Some(arrow) = build_chain_arrow(&poly, 0.5 * (min_z + max_z), *side_flip, &setup.frame) {
        direction_arrows.push(arrow);
    }

    let adjacent_lower_floor_z_wcs = if source_edge_ids.is_empty() {
        None
    } else {
        highest_adjacent_lower_floor_proj(
            doc,
            &source_edge_ids,
            max_z + wcs_origin_proj,
            setup.frame.z_axis,
        )
        .map(|p| p - wcs_origin_proj)
    };

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
        source_edge_ids,
        adjacent_lower_floor_z_wcs,
    });

    Ok(())
}

pub(super) fn extract_open_chain(
    acc: &mut BRepExtractAccum<'_>,
    start_edge: &SelectionBinding,
    resolved_edges: &[OrientedEdgeUse],
    reverse: bool,
    side_flip: bool,
) -> Result<(), CamError> {
    let doc = acc.doc;
    let setup = acc.setup;
    let stock_top_z = acc.stock_top_z;
    let model_top_z = acc.model_top_z;
    let wcs_origin_proj = acc.wcs_origin_proj;
    let highlight_edges = &mut *acc.highlight_edges;
    let direction_arrows = &mut *acc.direction_arrows;
    let top_candidates = &mut *acc.top_candidates;
    let bottom_candidates = &mut *acc.bottom_candidates;
    let features = &mut *acc.features;
    let reverse = &reverse;
    let side_flip = &side_flip;
    if resolved_edges.is_empty() {
        return Err(CamError::MissingSelectedEntity {
            index: start_edge.last_index,
        });
    }
    let mut source_edge_ids = Vec::with_capacity(resolved_edges.len());
    for u in resolved_edges {
        source_edge_ids.push(u.edge_id);
        if !highlight_edges.contains(&u.edge_id) {
            highlight_edges.push(u.edge_id);
        }
    }
    let (mut poly, min_z, max_z) =
        project_oriented_edges_to_wcs_xy(doc, &setup.frame, resolved_edges, false)?;
    if *reverse {
        poly = poly.reverse();
    }
    if let Some(arrow) = build_chain_arrow(&poly, 0.5 * (min_z + max_z), *side_flip, &setup.frame) {
        direction_arrows.push(arrow);
    }

    let adjacent_lower_floor_z_wcs = if source_edge_ids.is_empty() {
        None
    } else {
        highest_adjacent_lower_floor_proj(
            doc,
            &source_edge_ids,
            max_z + wcs_origin_proj,
            setup.frame.z_axis,
        )
        .map(|p| p - wcs_origin_proj)
    };

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
        source_edge_ids,
        adjacent_lower_floor_z_wcs,
    });

    Ok(())
}

pub(super) fn collect_cylindrical_hole(
    acc: &mut BRepExtractAccum<'_>,
    binding: &SelectionBinding,
) -> Result<(), CamError> {
    let doc = acc.doc;
    let setup = acc.setup;
    let raw_hole_faces = &mut *acc.raw_hole_faces;
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

    Ok(())
}

pub(super) fn finalize_cylindrical_holes(acc: &mut BRepExtractAccum<'_>) {
    let doc = acc.doc;
    let setup = acc.setup;
    let wcs_origin_proj = acc.wcs_origin_proj;
    let raw_hole_faces = &*acc.raw_hole_faces;
    let highlight_faces = &mut *acc.highlight_faces;
    let top_candidates = &mut *acc.top_candidates;
    let bottom_candidates = &mut *acc.bottom_candidates;
    let features = &mut *acc.features;
    let primary_hole_diameter_mm = &mut *acc.primary_hole_diameter_mm;
    // Group coaxial half-cylinder hole faces so split STEP holes produce 1 logical Hole25D!
    if !raw_hole_faces.is_empty() {
        let grouped = group_coaxial_holes(doc, raw_hole_faces, setup.frame.z_axis);
        for h in grouped {
            for &gfid in &h.faces {
                if !highlight_faces.contains(&gfid) {
                    highlight_faces.push(gfid);
                }
            }
            let c_wcs = setup.frame.world_to_local_point(h.axis_origin_mm);
            let top_z_wcs = h.span.top_proj_mm - wcs_origin_proj;
            let bottom_z_wcs = h.span.bottom_proj_mm - wcs_origin_proj;
            let diameter_mm = 2.0 * h.radius_mm;
            if primary_hole_diameter_mm.is_none() {
                *primary_hole_diameter_mm = Some(diameter_mm);
            }
            top_candidates.push(top_z_wcs);
            bottom_candidates.push(bottom_z_wcs);
            features.push(ExtractedFeature25D::CylindricalHole {
                primary_face: Some(h.primary_face),
                coaxial_faces: h.faces,
                center_wcs: DVec2::new(c_wcs.x, c_wcs.y),
                diameter_mm,
                top_z_wcs,
                bottom_z_wcs,
                is_through_hole: h.span.is_through_hole,
            });
        }
    }
}

fn is_oriented_uses_closed(doc: &CadDocument, uses: &[OrientedEdgeUse]) -> bool {
    if uses.is_empty() {
        return false;
    }
    if uses.len() == 1
        && let Some(e) = doc.edge(uses[0].edge_id)
        && matches!(
            e.curve,
            CurveKind::CircleArc {
                is_full_circle: true,
                ..
            }
        )
    {
        return true;
    }
    let endpoint = |u: OrientedEdgeUse, start: bool| -> Option<DVec3> {
        let ep = doc
            .render_mesh
            .edge_polylines
            .iter()
            .find(|ep| ep.edge_id == u.edge_id)?;
        if start == u.forward {
            ep.points_mm.first().copied()
        } else {
            ep.points_mm.last().copied()
        }
    };
    match (
        uses.first().and_then(|&u| endpoint(u, true)),
        uses.last().and_then(|&u| endpoint(u, false)),
    ) {
        (Some(p0), Some(p1)) => p0.distance(p1) <= 1e-3,
        _ => false,
    }
}

fn project_loop_to_wcs_xy(
    doc: &CadDocument,
    frame: &RigidFrame3,
    loop_id: LoopId,
) -> Result<(ArcPolyline2D, f64, f64), CamError> {
    let (poly, _, min_z, max_z) =
        project_loop_to_wcs_xy_with_open_flags(doc, frame, loop_id, None)?;
    Ok((poly, min_z, max_z))
}

fn project_loop_to_wcs_xy_with_open_flags(
    doc: &CadDocument,
    frame: &RigidFrame3,
    loop_id: LoopId,
    open_edges: Option<&HashSet<EdgeId>>,
) -> Result<(ArcPolyline2D, Vec<bool>, f64, f64), CamError> {
    let lp = doc
        .brep_loop(loop_id)
        .ok_or(CamError::MissingSelectedEntity { index: loop_id.0 })?;
    project_oriented_edges_with_flags(doc, frame, &lp.edges, true, open_edges)
}

fn project_oriented_edges_to_wcs_xy(
    doc: &CadDocument,
    frame: &RigidFrame3,
    uses: &[OrientedEdgeUse],
    closed: bool,
) -> Result<(ArcPolyline2D, f64, f64), CamError> {
    let (poly, _, min_z, max_z) =
        project_oriented_edges_with_flags(doc, frame, uses, closed, None)?;
    Ok((poly, min_z, max_z))
}

fn project_oriented_edges_with_flags(
    doc: &CadDocument,
    frame: &RigidFrame3,
    uses: &[OrientedEdgeUse],
    closed: bool,
    open_edges: Option<&HashSet<EdgeId>>,
) -> Result<(ArcPolyline2D, Vec<bool>, f64, f64), CamError> {
    let mut segments = Vec::with_capacity(uses.len());
    let mut open_flags = Vec::with_capacity(uses.len());
    let mut min_z = f64::INFINITY;
    let mut max_z = f64::NEG_INFINITY;

    for u in uses {
        let is_open = open_edges.is_some_and(|s| s.contains(&u.edge_id));
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
        open_flags.resize(segments.len(), is_open);
    }

    if !min_z.is_finite() {
        min_z = 0.0;
        max_z = 0.0;
    }

    Ok((ArcPolyline2D { segments, closed }, open_flags, min_z, max_z))
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

/// Finds the highest projection (`p · spindle_up_world`) among side walls directly adjacent to
/// a planar floor face (used for automatic Pocket `Top Height` inference).
pub fn highest_adjacent_wall_top(
    doc: &CadDocument,
    floor_face_id: FaceId,
    spindle_up_world: DVec3,
) -> Option<f64> {
    let floor = doc.face(floor_face_id)?;
    let up = spindle_up_world.normalize_or_zero();
    if up.length_squared() < 0.5 {
        return None;
    }

    let floor_proj = match floor.surface {
        SurfaceKind::Plane { origin_mm, .. } => origin_mm.dot(up),
        _ => floor.aabb_mm.center().dot(up),
    };

    let mut max_wall_top: Option<f64> = None;

    for loop_id in floor.outer_loop.iter().chain(floor.inner_loops.iter()) {
        let Some(lp) = doc.brep_loop(*loop_id) else {
            continue;
        };
        for use_rec in &lp.edges {
            let Some(edge) = doc.edge(use_rec.edge_id) else {
                continue;
            };
            for &adj_fid in &edge.adjacent_faces {
                if adj_fid == floor_face_id {
                    continue;
                }
                // Compute exact max projection of this adjacent wall's triangles/edges along `up`
                let mut adj_max = f64::NEG_INFINITY;
                for tri in &doc.render_mesh.pick_triangles {
                    if tri.face_id == adj_fid {
                        for v in [tri.v0_mm, tri.v1_mm, tri.v2_mm] {
                            adj_max = adj_max.max(v.dot(up));
                        }
                    }
                }
                if adj_max.is_finite() && adj_max > floor_proj + 1e-3 {
                    max_wall_top = Some(max_wall_top.map_or(adj_max, |cur| cur.max(adj_max)));
                }
            }
        }
    }

    max_wall_top
}

/// Topological wall/air/island classification for a planar pocket or step floor face (`05-operations.md`).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PlanarFaceLoopClassification {
    /// Outer loop edges whose neighboring face drops downward/away (open to air/stock) rather than rising into a wall.
    pub open_outer_edges: HashSet<EdgeId>,
    /// Inner loops that bound a raised island/boss (`adj_max_proj > floor_proj`), as opposed to downward holes.
    pub raised_inner_loops: HashSet<LoopId>,
}

/// Classifies outer-loop open-air edges and inner-loop raised bosses vs. downward holes on `floor_face_id`.
pub fn classify_planar_face_loops(
    doc: &CadDocument,
    floor_face_id: FaceId,
    spindle_up_world: DVec3,
) -> PlanarFaceLoopClassification {
    let mut out = PlanarFaceLoopClassification::default();
    let Some(floor) = doc.face(floor_face_id) else {
        return out;
    };
    let up = spindle_up_world.normalize_or_zero();
    if up.length_squared() < 0.5 {
        return out;
    }

    let floor_proj = match floor.surface {
        SurfaceKind::Plane { origin_mm, .. } => origin_mm.dot(up),
        _ => floor.aabb_mm.center().dot(up),
    };

    let face_rises_above_floor = |adj_fid: FaceId| -> bool {
        let mut adj_max = f64::NEG_INFINITY;
        for tri in &doc.render_mesh.pick_triangles {
            if tri.face_id == adj_fid {
                for v in [tri.v0_mm, tri.v1_mm, tri.v2_mm] {
                    adj_max = adj_max.max(v.dot(up));
                }
            }
        }
        adj_max.is_finite() && adj_max > floor_proj + 1e-2
    };

    if let Some(outer_lid) = floor.outer_loop
        && let Some(lp) = doc.brep_loop(outer_lid)
    {
        for u in &lp.edges {
            let Some(edge) = doc.edge(u.edge_id) else {
                continue;
            };
            let has_rising_wall = edge
                .adjacent_faces
                .iter()
                .copied()
                .filter(|&fid| fid != floor_face_id)
                .any(face_rises_above_floor);
            if !has_rising_wall {
                out.open_outer_edges.insert(u.edge_id);
            }
        }
    }

    for &ilid in &floor.inner_loops {
        let Some(lp) = doc.brep_loop(ilid) else {
            continue;
        };
        let has_rising_island_wall = lp.edges.iter().any(|u| {
            doc.edge(u.edge_id).is_some_and(|edge| {
                edge.adjacent_faces
                    .iter()
                    .copied()
                    .filter(|&fid| fid != floor_face_id)
                    .any(face_rises_above_floor)
            })
        });
        if has_rising_island_wall {
            out.raised_inner_loops.insert(ilid);
        }
    }

    out
}

/// Extracted geometry for an angled B-Rep chamfer/bevel face (`05-operations.md` §5A).
#[derive(Debug, Clone, PartialEq)]
pub struct ModeledChamferStrip {
    pub chamfer_face_id: FaceId,
    pub upper_edges: Vec<OrientedEdgeUse>,
    pub lower_edges: Vec<OrientedEdgeUse>,
    pub radial_width_mm: f64,
    pub axial_drop_mm: f64,
    /// Angle `θ` measured from spindle `+Z` axis to the chamfer face surface (`π/4` for a `45°` chamfer).
    pub bevel_half_angle_rad: f64,
    pub top_proj_mm: f64,
    pub bottom_proj_mm: f64,
}

/// Extracts upper/lower rim edges, chamfer width `w`, and bevel angle `θ` from a modeled angled planar/conical face.
pub fn extract_modeled_chamfer_strip(
    doc: &CadDocument,
    chamfer_face_id: FaceId,
    spindle_up_world: DVec3,
) -> Option<ModeledChamferStrip> {
    let face = doc.face(chamfer_face_id)?;
    let up = spindle_up_world.normalize_or_zero();
    if up.length_squared() < 0.5 {
        return None;
    }

    // Determine bevel half-angle from vertical (`θ`):
    // For an upward-tilted planar bevel with outward unit normal `n`, `cos_up = n · up ∈ (0.05, 0.995)`,
    // and the surface tangent makes angle `θ = arcsin(n · up)` with spindle vertical (`cos_up = sin(45°) = 0.707` -> `θ = 45°`).
    let bevel_half_angle_rad = match face.surface {
        SurfaceKind::Plane { normal, .. } => {
            let cos_up = normal.normalize_or_zero().dot(up).abs();
            if !(0.05..=0.995).contains(&cos_up) {
                return None;
            }
            cos_up.asin()
        }
        SurfaceKind::Cone {
            axis_dir,
            half_angle_rad,
            ..
        } => {
            if axis_dir.normalize_or_zero().dot(up).abs() < 0.999 {
                return None;
            }
            let ha = half_angle_rad.abs();
            if !(0.05..=1.52).contains(&ha) {
                return None;
            }
            ha
        }
        _ => return None,
    };

    let mut min_proj = f64::INFINITY;
    let mut max_proj = f64::NEG_INFINITY;
    for tri in &doc.render_mesh.pick_triangles {
        if tri.face_id == chamfer_face_id {
            for v in [tri.v0_mm, tri.v1_mm, tri.v2_mm] {
                let pr = v.dot(up);
                min_proj = min_proj.min(pr);
                max_proj = max_proj.max(pr);
            }
        }
    }
    let axial_drop_mm = max_proj - min_proj;
    if !axial_drop_mm.is_finite() || axial_drop_mm <= 1e-3 {
        return None;
    }

    let radial_width_mm = axial_drop_mm * bevel_half_angle_rad.tan();
    let band = (0.12 * axial_drop_mm).max(0.05);
    let mut upper_edges = Vec::new();
    let mut lower_edges = Vec::new();

    for lid in face.outer_loop.iter().chain(face.inner_loops.iter()) {
        let Some(lp) = doc.brep_loop(*lid) else {
            continue;
        };
        for u in &lp.edges {
            if doc.edge(u.edge_id).is_none() {
                continue;
            }
            // Check whether this edge sits entirely along the upper rim or lower rim
            let mut e_min = f64::INFINITY;
            let mut e_max = f64::NEG_INFINITY;
            if let Some(ep) = doc
                .render_mesh
                .edge_polylines
                .iter()
                .find(|ep| ep.edge_id == u.edge_id)
            {
                for &pt in &ep.points_mm {
                    let pr = pt.dot(up);
                    e_min = e_min.min(pr);
                    e_max = e_max.max(pr);
                }
            }
            if !e_min.is_finite() {
                continue;
            }
            if (e_min - max_proj).abs() <= band && (e_max - max_proj).abs() <= band {
                upper_edges.push(*u);
            } else if (e_min - min_proj).abs() <= band && (e_max - min_proj).abs() <= band {
                lower_edges.push(*u);
            }
        }
    }

    if upper_edges.is_empty() && lower_edges.is_empty() {
        return None;
    }

    Some(ModeledChamferStrip {
        chamfer_face_id,
        upper_edges,
        lower_edges,
        radial_width_mm,
        axial_drop_mm,
        bevel_half_angle_rad,
        top_proj_mm: max_proj,
        bottom_proj_mm: min_proj,
    })
}

/// Finds the highest upward-facing horizontal step floor directly below any vertical/angled wall
/// bordering `chamfer_edges` (`proj < ref_top_proj_mm - 1e-2`), used to guard against `tip_offset` floor gouges.
pub fn highest_adjacent_lower_floor_proj(
    doc: &CadDocument,
    chamfer_edges: &[EdgeId],
    ref_top_proj_mm: f64,
    spindle_up_world: DVec3,
) -> Option<f64> {
    let up = spindle_up_world.normalize_or_zero();
    if up.length_squared() < 0.5 {
        return None;
    }

    // Collect side walls directly touching `chamfer_edges`
    let mut side_wall_faces = HashSet::new();
    for &eid in chamfer_edges {
        if let Some(edge) = doc.edge(eid) {
            for &fid in &edge.adjacent_faces {
                if let Some(face) = doc.face(fid) {
                    let is_top_cap = matches!(
                        face.surface,
                        SurfaceKind::Plane { origin_mm, normal }
                        if normal.normalize_or_zero().dot(up).abs() > 0.99
                            && (origin_mm.dot(up) - ref_top_proj_mm).abs() <= 0.05
                    );
                    if !is_top_cap {
                        side_wall_faces.insert(fid);
                    }
                }
            }
        }
    }

    // Check every edge of those side walls for an upward-pointing horizontal floor (`n · up > 0.98`)
    let mut highest_floor: Option<f64> = None;
    for wall_fid in side_wall_faces {
        let Some(wall) = doc.face(wall_fid) else {
            continue;
        };
        for lid in wall.outer_loop.iter().chain(wall.inner_loops.iter()) {
            let Some(lp) = doc.brep_loop(*lid) else {
                continue;
            };
            for u in &lp.edges {
                let Some(edge) = doc.edge(u.edge_id) else {
                    continue;
                };
                for &adj_fid in &edge.adjacent_faces {
                    if adj_fid == wall_fid {
                        continue;
                    }
                    if let Some(adj_face) = doc.face(adj_fid)
                        && let SurfaceKind::Plane { origin_mm, normal } = adj_face.surface
                    {
                        let cos_up = normal.normalize_or_zero().dot(up);
                        let f_proj = origin_mm.dot(up);
                        if cos_up > 0.98 && f_proj < ref_top_proj_mm - 0.05 {
                            highest_floor =
                                Some(highest_floor.map_or(f_proj, |cur| cur.max(f_proj)));
                        }
                    }
                }
            }
        }
    }

    highest_floor
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::{
        CadKernel, MonstertruckKernel, SurfaceKind, sample_cam_torture_block_step,
        sample_multi_feature_bracket_step, sample_threaded_manifold_step,
        tutorial_chamfer_engrave_block_step,
    };
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec3, approx_eq};

    use super::*;

    #[test]
    fn brep_spindle_feature_helpers_classify_walls_steps_islands_chamfers_and_floors() {
        // 1. Multi-feature bracket adjacent wall top Z = 20.0 mm above Z = 12.0 mm pocket floor
        let bracket_step = sample_multi_feature_bracket_step(false).unwrap();
        let bracket_doc = MonstertruckKernel
            .load_step_bytes(
                "multi_bracket.step",
                bracket_step.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .unwrap();
        let z12_floor = bracket_doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { origin_mm, normal }
                    if normal.z > 0.99 && approx_eq(origin_mm.z, 12.0, 1e-2)
                )
            })
            .unwrap()
            .id;
        let wall_top = highest_adjacent_wall_top(&bracket_doc, z12_floor, DVec3::Z).unwrap();
        assert!(approx_eq(wall_top, 20.0, 0.05));

        // 2. Torture block open step, raised island, modeled 45° x 2.0 mm chamfer, and lower floor
        let torture_step = sample_cam_torture_block_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes(
                "torture_block.step",
                torture_step.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .unwrap();
        let step_floor = doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { origin_mm, normal }
                    if normal.z > 0.99 && approx_eq(origin_mm.z, 14.0, 1e-2)
                )
            })
            .unwrap()
            .id;
        let step_cls = classify_planar_face_loops(&doc, step_floor, DVec3::Z);
        assert_eq!(step_cls.open_outer_edges.len(), 3);
        assert!(step_cls.raised_inner_loops.is_empty());

        let island_pkt_floor = doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { origin_mm, normal }
                    if normal.z > 0.99 && approx_eq(origin_mm.z, 12.0, 1e-2)
                )
            })
            .unwrap()
            .id;
        let pkt_cls = classify_planar_face_loops(&doc, island_pkt_floor, DVec3::Z);
        assert!(pkt_cls.open_outer_edges.is_empty());
        assert_eq!(pkt_cls.raised_inner_loops.len(), 1);

        let chamfer_face = doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { normal, .. }
                    if approx_eq(normal.z, std::f64::consts::FRAC_1_SQRT_2, 1e-2)
                )
            })
            .unwrap()
            .id;
        let strip = extract_modeled_chamfer_strip(&doc, chamfer_face, DVec3::Z).unwrap();
        assert!(approx_eq(
            strip.bevel_half_angle_rad,
            std::f64::consts::FRAC_PI_4,
            1e-2
        ));
        assert!(approx_eq(strip.axial_drop_mm, 2.0, 2e-2));
        assert!(approx_eq(strip.radial_width_mm, 2.0, 2e-2));
        assert!(!strip.upper_edges.is_empty());
        assert!(!strip.lower_edges.is_empty());

        let step_top_edge = doc
            .edges
            .iter()
            .find(|e| {
                approx_eq(e.aabb_mm.min.z, 20.0, 1e-2)
                    && approx_eq(e.aabb_mm.max.z, 20.0, 1e-2)
                    && approx_eq(e.aabb_mm.min.x, 76.0, 1e-2)
                    && approx_eq(e.aabb_mm.max.x, 76.0, 1e-2)
            })
            .unwrap()
            .id;
        let lower_floor =
            highest_adjacent_lower_floor_proj(&doc, &[step_top_edge], 20.0, DVec3::Z).unwrap();
        assert!(approx_eq(lower_floor, 14.0, 2e-2));

        // 3. Tutorial chamfer block 45° x 1.5 mm front bevel
        let ch_step = tutorial_chamfer_engrave_block_step().unwrap();
        let ch_doc = MonstertruckKernel
            .load_step_bytes(
                "06-chamfer-engrave-block.step",
                ch_step.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .unwrap();
        let bevel_face = ch_doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { normal, .. }
                    if normal.z > 0.65 && normal.z < 0.76
                )
            })
            .unwrap()
            .id;
        let ch_strip = extract_modeled_chamfer_strip(&ch_doc, bevel_face, DVec3::Z).unwrap();
        assert!(approx_eq(
            ch_strip.bevel_half_angle_rad,
            std::f64::consts::FRAC_PI_4,
            1e-2
        ));
        assert!(approx_eq(ch_strip.radial_width_mm, 1.5, 2e-2));
        assert!(approx_eq(ch_strip.axial_drop_mm, 1.5, 2e-2));

        // 4. Threaded manifold deck at Z = 12 mm has 4 open outer edges + 1 raised Ø12 mm boss
        let man_step = sample_threaded_manifold_step().unwrap();
        let man_doc = MonstertruckKernel
            .load_step_bytes(
                "18-threaded-manifold.step",
                man_step.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .unwrap();
        let deck_z12 = man_doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { origin_mm, normal }
                    if normal.z > 0.99 && approx_eq(origin_mm.z, 12.0, 1e-2)
                )
            })
            .unwrap()
            .id;
        let deck_cls = classify_planar_face_loops(&man_doc, deck_z12, DVec3::Z);
        assert_eq!(deck_cls.open_outer_edges.len(), 4);
        assert_eq!(deck_cls.raised_inner_loops.len(), 1);
    }
}
