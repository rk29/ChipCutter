use std::collections::HashSet;

use chip_cad::{
    BodyId, CadDocument, CurveKind, EdgeId, FaceId, LoopId, OrientedEdgeUse, SelectionBinding,
    SurfaceKind, classify_planar_face_loops, extract_modeled_chamfer_strip, group_coaxial_holes,
    highest_adjacent_lower_floor_proj, highest_adjacent_wall_top,
};
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3, RigidFrame3};
use chip_sketch::{PlanarVectorRegion, SketchEntity, SketchEntityEntry, SketchLayer};
use serde::{Deserialize, Serialize};

use crate::{
    setup::{ResolvedSetup, resolve_sketch_world_frame},
    wcs::CamError,
};

/// Persistent interactive 3D B-Rep or 2D Sketch geometry selection item inside an [`crate::operation::Operation`].
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
    SketchRegion {
        sketch_id: String,
        region_id: String,
    },
    SketchEntity {
        sketch_id: String,
        entity_id: String,
        #[serde(default)]
        reverse: bool,
        #[serde(default)]
        side_flip: bool,
    },
    SketchLayerAll {
        sketch_id: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        sub_layer: Option<String>,
        #[serde(default)]
        reverse: bool,
        #[serde(default)]
        side_flip: bool,
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
            Self::SketchRegion {
                sketch_id,
                region_id,
            } => format!("Sketch Region {sketch_id}/{region_id}"),
            Self::SketchEntity {
                sketch_id,
                entity_id,
                reverse,
                side_flip,
            } => {
                let dir = if *reverse { "Rev" } else { "Fwd" };
                let side = if *side_flip { "Right" } else { "Left" };
                format!("Sketch Curve {sketch_id}/{entity_id} ({dir}, {side})")
            }
            Self::SketchLayerAll {
                sketch_id,
                sub_layer,
                reverse,
                side_flip,
            } => {
                let dir = if *reverse { "Rev" } else { "Fwd" };
                let side = if *side_flip { "Right" } else { "Left" };
                match sub_layer {
                    Some(layer) => format!("Sketch Layer {sketch_id} [{layer}] ({dir}, {side})"),
                    None => format!("All of Sketch {sketch_id} ({dir}, {side})"),
                }
            }
        }
    }
}

/// Extracted 2.5D CAM geometry in canonical Setup WCS millimeters (`f64`).
#[derive(Debug, Clone, PartialEq)]
pub enum ExtractedFeature25D {
    PlanarFace {
        source_face_id: Option<FaceId>,
        outer_boundary_wcs: ArcPolyline2D,
        /// Per-segment flag (`outer_boundary_wcs.segments.len()`) true when touching open air/exterior step.
        open_outer_segments: Vec<bool>,
        inner_islands_wcs: Vec<ArcPolyline2D>,
        /// Subset of `inner_islands_wcs` that bound true raised islands/bosses (`Z > face_z_wcs`).
        raised_islands_wcs: Vec<ArcPolyline2D>,
        face_z_wcs: f64,
        inferred_top_z_wcs: f64,
        adjacent_lower_floor_z_wcs: Option<f64>,
        area_mm2: f64,
    },
    Contour {
        polyline_wcs: ArcPolyline2D,
        min_z_wcs: f64,
        max_z_wcs: f64,
        reverse: bool,
        side_flip: bool,
        source_edge_ids: Vec<EdgeId>,
        adjacent_lower_floor_z_wcs: Option<f64>,
    },
    ModeledChamfer {
        source_face_id: Option<FaceId>,
        upper_polyline_wcs: ArcPolyline2D,
        lower_polyline_wcs: ArcPolyline2D,
        radial_width_mm: f64,
        axial_drop_mm: f64,
        bevel_half_angle_rad: f64,
        top_z_wcs: f64,
        bottom_z_wcs: f64,
        outward_normal_xy_wcs: Option<DVec2>,
        adjacent_lower_floor_z_wcs: Option<f64>,
        edge_ids: Vec<EdgeId>,
    },
    CylindricalHole {
        primary_face: Option<FaceId>,
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
    pub highlight_sketch_regions: Vec<(String, String)>,
    pub highlight_sketch_entities: Vec<(String, String)>,
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
    extract_selection_features_with_sketches(doc, &[], setup, selections)
}

/// Projects both 3D B-Rep and 2D [`SketchLayer`] selections into unified [`ExtractedFeature25D`]s.
pub fn extract_selection_features_with_sketches(
    doc: &CadDocument,
    sketches: &[SketchLayer],
    setup: &ResolvedSetup,
    selections: &[GeometrySelectionItem],
) -> Result<ExtractedSelectionSummary, CamError> {
    let stock_top_z = setup.stock.aabb_wcs().max.z;
    let model_top_z = setup.model_aabb_wcs.max.z;
    let doc_has_bodies = !doc.bodies.is_empty();

    if selections.is_empty() {
        return Ok(ExtractedSelectionSummary {
            features: Vec::new(),
            highlight_faces: Vec::new(),
            highlight_edges: Vec::new(),
            highlight_sketch_regions: Vec::new(),
            highlight_sketch_entities: Vec::new(),
            direction_arrows: Vec::new(),
            inferred_top_z_wcs: stock_top_z,
            inferred_bottom_z_wcs: model_top_z,
            primary_hole_diameter_mm: None,
        });
    }

    let mut features = Vec::new();
    let mut highlight_faces = Vec::new();
    let mut highlight_edges = Vec::new();
    let mut highlight_sketch_regions = Vec::new();
    let mut highlight_sketch_entities = Vec::new();
    let mut direction_arrows = Vec::new();
    let mut top_candidates = Vec::new();
    let mut bottom_candidates = Vec::new();
    let mut raw_hole_faces = Vec::new();
    let mut primary_hole_diameter_mm: Option<f64> = None;
    let wcs_origin_proj = setup.frame.origin_mm.dot(setup.frame.z_axis);

    for item in selections {
        match item {
            GeometrySelectionItem::PlanarFace { binding } => {
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
                } else if let Some(strip) =
                    extract_modeled_chamfer_strip(doc, fid, setup.frame.z_axis)
                {
                    let top_z_wcs = strip.top_proj_mm - wcs_origin_proj;
                    let bottom_z_wcs = strip.bottom_proj_mm - wcs_origin_proj;
                    let upper_closed = is_oriented_uses_closed(doc, &strip.upper_edges);
                    let lower_closed = is_oriented_uses_closed(doc, &strip.lower_edges);
                    let upper_polyline_wcs = if strip.upper_edges.is_empty() {
                        ArcPolyline2D::default()
                    } else {
                        project_oriented_edges_to_wcs_xy(
                            doc,
                            &setup.frame,
                            &strip.upper_edges,
                            upper_closed,
                        )?
                        .0
                    };
                    let lower_polyline_wcs = if strip.lower_edges.is_empty() {
                        ArcPolyline2D::default()
                    } else {
                        project_oriented_edges_to_wcs_xy(
                            doc,
                            &setup.frame,
                            &strip.lower_edges,
                            lower_closed,
                        )?
                        .0
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
                if let Some(arrow) =
                    build_chain_arrow(&poly, 0.5 * (min_z + max_z), *side_flip, &setup.frame)
                {
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
                if let Some(arrow) =
                    build_chain_arrow(&poly, 0.5 * (min_z + max_z), *side_flip, &setup.frame)
                {
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
            GeometrySelectionItem::SketchRegion {
                sketch_id,
                region_id,
            } => {
                let sk = sketches
                    .iter()
                    .find(|s| s.id == *sketch_id)
                    .ok_or_else(|| CamError::MissingSelectedSketch {
                        sketch_id: sketch_id.clone(),
                        item_id: region_id.clone(),
                    })?;
                let reg = sk
                    .regions
                    .iter()
                    .find(|r| r.id == *region_id)
                    .ok_or_else(|| CamError::MissingSelectedSketch {
                        sketch_id: sketch_id.clone(),
                        item_id: region_id.clone(),
                    })?;
                let mut acc = SketchExtractAccum {
                    doc_has_bodies,
                    setup,
                    features: &mut features,
                    direction_arrows: &mut direction_arrows,
                    top_candidates: &mut top_candidates,
                    bottom_candidates: &mut bottom_candidates,
                    primary_hole_diameter_mm: &mut primary_hole_diameter_mm,
                };
                extract_sketch_region(&mut acc, sk, reg);
                let key = (sketch_id.clone(), region_id.clone());
                if !highlight_sketch_regions.contains(&key) {
                    highlight_sketch_regions.push(key);
                }
            }
            GeometrySelectionItem::SketchEntity {
                sketch_id,
                entity_id,
                reverse,
                side_flip,
            } => {
                let sk = sketches
                    .iter()
                    .find(|s| s.id == *sketch_id)
                    .ok_or_else(|| CamError::MissingSelectedSketch {
                        sketch_id: sketch_id.clone(),
                        item_id: entity_id.clone(),
                    })?;
                let ent = sk
                    .entities
                    .iter()
                    .find(|e| e.id == *entity_id)
                    .ok_or_else(|| CamError::MissingSelectedSketch {
                        sketch_id: sketch_id.clone(),
                        item_id: entity_id.clone(),
                    })?;
                let mut acc = SketchExtractAccum {
                    doc_has_bodies,
                    setup,
                    features: &mut features,
                    direction_arrows: &mut direction_arrows,
                    top_candidates: &mut top_candidates,
                    bottom_candidates: &mut bottom_candidates,
                    primary_hole_diameter_mm: &mut primary_hole_diameter_mm,
                };
                extract_sketch_entity(&mut acc, sk, ent, *reverse, *side_flip, true);
                let key = (sketch_id.clone(), entity_id.clone());
                if !highlight_sketch_entities.contains(&key) {
                    highlight_sketch_entities.push(key);
                }
            }
            GeometrySelectionItem::SketchLayerAll {
                sketch_id,
                sub_layer,
                reverse,
                side_flip,
            } => {
                let sk = sketches
                    .iter()
                    .find(|s| s.id == *sketch_id)
                    .ok_or_else(|| CamError::MissingSelectedSketch {
                        sketch_id: sketch_id.clone(),
                        item_id: sub_layer.clone().unwrap_or_else(|| "*".to_string()),
                    })?;
                let mut acc = SketchExtractAccum {
                    doc_has_bodies,
                    setup,
                    features: &mut features,
                    direction_arrows: &mut direction_arrows,
                    top_candidates: &mut top_candidates,
                    bottom_candidates: &mut bottom_candidates,
                    primary_hole_diameter_mm: &mut primary_hole_diameter_mm,
                };
                extract_sketch_layer_all(
                    &mut acc,
                    sk,
                    sub_layer.as_deref(),
                    *reverse,
                    *side_flip,
                    &mut highlight_sketch_regions,
                    &mut highlight_sketch_entities,
                );
            }
        }
    }

    // Group coaxial half-cylinder hole faces so split STEP holes produce 1 logical Hole25D!
    if !raw_hole_faces.is_empty() {
        let grouped = group_coaxial_holes(doc, &raw_hole_faces, setup.frame.z_axis);
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
                primary_hole_diameter_mm = Some(diameter_mm);
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
        highlight_sketch_regions,
        highlight_sketch_entities,
        direction_arrows,
        inferred_top_z_wcs,
        inferred_bottom_z_wcs,
        primary_hole_diameter_mm,
    })
}

struct SketchExtractAccum<'a> {
    doc_has_bodies: bool,
    setup: &'a ResolvedSetup,
    features: &'a mut Vec<ExtractedFeature25D>,
    direction_arrows: &'a mut Vec<ChainDirectionArrow3D>,
    top_candidates: &'a mut Vec<f64>,
    bottom_candidates: &'a mut Vec<f64>,
    primary_hole_diameter_mm: &'a mut Option<f64>,
}

fn transform_sketch_polyline_to_wcs(
    poly: &ArcPolyline2D,
    sk_world_frame: &RigidFrame3,
    setup_frame: &RigidFrame3,
) -> (ArcPolyline2D, f64) {
    let map_pt = |p: DVec2| -> (DVec2, f64) {
        let w = sk_world_frame.local_to_world_point(DVec3::new(p.x, p.y, 0.0));
        let l = setup_frame.world_to_local_point(w);
        (DVec2::new(l.x, l.y), l.z)
    };
    let z_orient = sk_world_frame.z_axis.dot(setup_frame.z_axis);
    let flip_ccw = z_orient < 0.0;
    let mut z_val = 0.0;
    let mut segs = Vec::with_capacity(poly.segments.len());
    for seg in &poly.segments {
        match *seg {
            ArcSegment2D::Line { start, end } => {
                let (s, zs) = map_pt(start);
                let (e, _) = map_pt(end);
                z_val = zs;
                segs.push(ArcSegment2D::Line { start: s, end: e });
            }
            ArcSegment2D::Arc {
                start,
                end,
                center,
                radius,
                ccw,
            } => {
                let (s, zs) = map_pt(start);
                let (e, _) = map_pt(end);
                let (c, _) = map_pt(center);
                z_val = zs;
                segs.push(ArcSegment2D::Arc {
                    start: s,
                    end: e,
                    center: c,
                    radius,
                    ccw: if flip_ccw { !ccw } else { ccw },
                });
            }
        }
    }
    (
        ArcPolyline2D {
            segments: segs,
            closed: poly.closed,
        },
        z_val,
    )
}

fn infer_sketch_z_span(
    doc_has_bodies: bool,
    setup: &ResolvedSetup,
    plane_z_wcs: f64,
) -> (f64, f64) {
    let stock_aabb = setup.stock.aabb_wcs();
    let stock_top_z = stock_aabb.max.z;
    let top_z = if plane_z_wcs <= stock_top_z + 1e-4 {
        stock_top_z
    } else {
        plane_z_wcs
    };
    let bottom_z = if plane_z_wcs < stock_top_z - 1e-4 {
        plane_z_wcs
    } else if !doc_has_bodies {
        setup.model_aabb_wcs.min.z.min(top_z - 0.5)
    } else {
        (top_z - 1.0).max(stock_aabb.min.z)
    };
    (top_z, bottom_z)
}

fn extract_sketch_region(
    acc: &mut SketchExtractAccum<'_>,
    sk: &SketchLayer,
    reg: &PlanarVectorRegion,
) {
    let sk_frame = resolve_sketch_world_frame(acc.doc_has_bodies, sk, acc.setup);
    let (outer_raw, plane_z_wcs) =
        transform_sketch_polyline_to_wcs(&reg.outer_ccw, &sk_frame, &acc.setup.frame);
    let outer_boundary_wcs = outer_raw.ensure_orientation(true);
    let open_outer_segments = vec![false; outer_boundary_wcs.segments.len()];
    let mut inner_islands_wcs = Vec::with_capacity(reg.islands_cw.len());
    for isl in &reg.islands_cw {
        let (isl_raw, _) = transform_sketch_polyline_to_wcs(isl, &sk_frame, &acc.setup.frame);
        inner_islands_wcs.push(isl_raw.ensure_orientation(false));
    }
    let raised_islands_wcs = inner_islands_wcs.clone();
    let (inferred_top, inferred_bot) =
        infer_sketch_z_span(acc.doc_has_bodies, acc.setup, plane_z_wcs);
    acc.top_candidates.push(inferred_top);
    acc.bottom_candidates.push(inferred_bot);
    acc.features.push(ExtractedFeature25D::PlanarFace {
        source_face_id: None,
        outer_boundary_wcs,
        open_outer_segments,
        inner_islands_wcs,
        raised_islands_wcs,
        face_z_wcs: inferred_bot,
        inferred_top_z_wcs: inferred_top,
        adjacent_lower_floor_z_wcs: None,
        area_mm2: reg.area_mm2,
    });
}

fn extract_sketch_entity(
    acc: &mut SketchExtractAccum<'_>,
    sk: &SketchLayer,
    entry: &SketchEntityEntry,
    reverse: bool,
    side_flip: bool,
    emit_contour_for_circle: bool,
) {
    let sk_frame = resolve_sketch_world_frame(acc.doc_has_bodies, sk, acc.setup);
    match &entry.entity {
        SketchEntity::Circle {
            center_xy,
            diameter,
        } => {
            let d_mm = diameter.value_mm;
            if d_mm <= 1e-4 {
                return;
            }
            let c_world = sk_frame.local_to_world_point(DVec3::new(center_xy.x, center_xy.y, 0.0));
            let c_wcs3 = acc.setup.frame.world_to_local_point(c_world);
            let (top_z, bot_z) = infer_sketch_z_span(acc.doc_has_bodies, acc.setup, c_wcs3.z);
            let hole_bot_z = if !acc.doc_has_bodies {
                acc.setup.stock.aabb_wcs().min.z
            } else {
                bot_z
            };
            if acc.primary_hole_diameter_mm.is_none() {
                *acc.primary_hole_diameter_mm = Some(d_mm);
            }
            acc.top_candidates.push(top_z);
            acc.bottom_candidates.push(hole_bot_z);
            acc.features.push(ExtractedFeature25D::CylindricalHole {
                primary_face: None,
                coaxial_faces: Vec::new(),
                center_wcs: DVec2::new(c_wcs3.x, c_wcs3.y),
                diameter_mm: d_mm,
                top_z_wcs: top_z,
                bottom_z_wcs: hole_bot_z,
                is_through_hole: !acc.doc_has_bodies,
            });
            if emit_contour_for_circle {
                let raw_poly = entry.entity.to_arc_polyline();
                let (mut poly_wcs, _) =
                    transform_sketch_polyline_to_wcs(&raw_poly, &sk_frame, &acc.setup.frame);
                poly_wcs = poly_wcs.ensure_orientation(false);
                if reverse {
                    poly_wcs = poly_wcs.reverse();
                }
                if let Some(arrow) =
                    build_chain_arrow(&poly_wcs, c_wcs3.z, side_flip, &acc.setup.frame)
                {
                    acc.direction_arrows.push(arrow);
                }
                acc.features.push(ExtractedFeature25D::Contour {
                    polyline_wcs: poly_wcs,
                    min_z_wcs: bot_z,
                    max_z_wcs: top_z,
                    reverse,
                    side_flip,
                    source_edge_ids: Vec::new(),
                    adjacent_lower_floor_z_wcs: None,
                });
            }
        }
        SketchEntity::Polyline { polyline } => {
            if polyline.segments.is_empty() {
                return;
            }
            let (mut poly_wcs, plane_z_wcs) =
                transform_sketch_polyline_to_wcs(polyline, &sk_frame, &acc.setup.frame);
            if poly_wcs.closed && poly_wcs.signed_area().abs() > 1e-6 {
                // Default closed sketch contour winds CW so `LeftClimb` profiles outside in Climb Milling!
                poly_wcs = poly_wcs.ensure_orientation(false);
            }
            if reverse {
                poly_wcs = poly_wcs.reverse();
            }
            if let Some(arrow) =
                build_chain_arrow(&poly_wcs, plane_z_wcs, side_flip, &acc.setup.frame)
            {
                acc.direction_arrows.push(arrow);
            }
            let (top_z, bot_z) = infer_sketch_z_span(acc.doc_has_bodies, acc.setup, plane_z_wcs);
            acc.top_candidates.push(top_z);
            acc.bottom_candidates.push(bot_z);
            acc.features.push(ExtractedFeature25D::Contour {
                polyline_wcs: poly_wcs,
                min_z_wcs: bot_z,
                max_z_wcs: top_z,
                reverse,
                side_flip,
                source_edge_ids: Vec::new(),
                adjacent_lower_floor_z_wcs: None,
            });
        }
    }
}

fn extract_sketch_layer_all(
    acc: &mut SketchExtractAccum<'_>,
    sk: &SketchLayer,
    sub_layer_filter: Option<&str>,
    reverse: bool,
    side_flip: bool,
    highlight_regions: &mut Vec<(String, String)>,
    highlight_entities: &mut Vec<(String, String)>,
) {
    let matches_layer = |item_layer: Option<&str>| -> bool {
        match sub_layer_filter {
            Some(target) => item_layer.is_some_and(|l| l.eq_ignore_ascii_case(target)),
            None => true,
        }
    };

    // 1. Emit `CylindricalHole` for all circles on the layer (enables 1-click Drilling of `HOLES` layer!)
    for ent in &sk.entities {
        if ent.is_construction || !matches_layer(ent.sub_layer.as_deref()) {
            continue;
        }
        if matches!(ent.entity, SketchEntity::Circle { .. }) {
            extract_sketch_entity(acc, sk, ent, reverse, side_flip, false);
            let key = (sk.id.clone(), ent.id.clone());
            if !highlight_entities.contains(&key) {
                highlight_entities.push(key);
            }
        }
    }

    // 2. Closed planar regions on the layer: if `reverse || side_flip` is requested, emit oriented
    //    `Contour`s so inside/outside profile direction is honored; otherwise emit `PlanarFace`s!
    let sk_frame = resolve_sketch_world_frame(acc.doc_has_bodies, sk, acc.setup);
    for reg in &sk.regions {
        if !matches_layer(reg.sub_layer.as_deref()) {
            continue;
        }
        if reverse || side_flip {
            let (mut poly_wcs, plane_z_wcs) =
                transform_sketch_polyline_to_wcs(&reg.outer_ccw, &sk_frame, &acc.setup.frame);
            poly_wcs = poly_wcs.ensure_orientation(false);
            if reverse {
                poly_wcs = poly_wcs.reverse();
            }
            if let Some(arrow) =
                build_chain_arrow(&poly_wcs, plane_z_wcs, side_flip, &acc.setup.frame)
            {
                acc.direction_arrows.push(arrow);
            }
            let (top_z, bot_z) = infer_sketch_z_span(acc.doc_has_bodies, acc.setup, plane_z_wcs);
            acc.top_candidates.push(top_z);
            acc.bottom_candidates.push(bot_z);
            acc.features.push(ExtractedFeature25D::Contour {
                polyline_wcs: poly_wcs,
                min_z_wcs: bot_z,
                max_z_wcs: top_z,
                reverse,
                side_flip,
                source_edge_ids: Vec::new(),
                adjacent_lower_floor_z_wcs: None,
            });
        } else {
            extract_sketch_region(acc, sk, reg);
        }
        let key = (sk.id.clone(), reg.id.clone());
        if !highlight_regions.contains(&key) {
            highlight_regions.push(key);
        }
    }

    // 3. Open chains on the layer -> emit `Contour`s
    for ent in &sk.entities {
        if ent.is_construction || !matches_layer(ent.sub_layer.as_deref()) {
            continue;
        }
        if let SketchEntity::Polyline { polyline } = &ent.entity
            && !polyline.closed
        {
            extract_sketch_entity(acc, sk, ent, reverse, side_flip, false);
            let key = (sk.id.clone(), ent.id.clone());
            if !highlight_entities.contains(&key) {
                highlight_entities.push(key);
            }
        }
    }
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
