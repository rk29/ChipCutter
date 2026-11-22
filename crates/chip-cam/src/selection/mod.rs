mod brep;
mod sketch;

use chip_cad::{BodyId, CadDocument, EdgeId, FaceId, OrientedEdgeUse, SelectionBinding};
use chip_math::{ArcPolyline2D, DVec2, DVec3, RigidFrame3};
use chip_sketch::SketchLayer;
use serde::{Deserialize, Serialize};

use crate::{setup::ResolvedSetup, wcs::CamError};

use self::{
    brep::{
        BRepExtractAccum, collect_cylindrical_hole, extract_closed_loop, extract_open_chain,
        extract_planar_face, finalize_cylindrical_holes,
    },
    sketch::{
        SketchExtractAccum, extract_sketch_entity, extract_sketch_layer_all, extract_sketch_region,
        verify_sketch_aligned_with_setup,
    },
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
    let doc_has_bodies = doc.has_3d_geometry();

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
                let mut acc = BRepExtractAccum {
                    doc,
                    setup,
                    stock_top_z,
                    model_top_z,
                    wcs_origin_proj,
                    features: &mut features,
                    highlight_faces: &mut highlight_faces,
                    highlight_edges: &mut highlight_edges,
                    direction_arrows: &mut direction_arrows,
                    top_candidates: &mut top_candidates,
                    bottom_candidates: &mut bottom_candidates,
                    raw_hole_faces: &mut raw_hole_faces,
                    primary_hole_diameter_mm: &mut primary_hole_diameter_mm,
                };
                extract_planar_face(&mut acc, binding)?;
            }
            GeometrySelectionItem::ClosedLoop {
                binding,
                reverse,
                side_flip,
            } => {
                let mut acc = BRepExtractAccum {
                    doc,
                    setup,
                    stock_top_z,
                    model_top_z,
                    wcs_origin_proj,
                    features: &mut features,
                    highlight_faces: &mut highlight_faces,
                    highlight_edges: &mut highlight_edges,
                    direction_arrows: &mut direction_arrows,
                    top_candidates: &mut top_candidates,
                    bottom_candidates: &mut bottom_candidates,
                    raw_hole_faces: &mut raw_hole_faces,
                    primary_hole_diameter_mm: &mut primary_hole_diameter_mm,
                };
                extract_closed_loop(&mut acc, binding, *reverse, *side_flip)?;
            }
            GeometrySelectionItem::OpenChain {
                resolved_edges,
                reverse,
                side_flip,
                start_edge,
                ..
            } => {
                let mut acc = BRepExtractAccum {
                    doc,
                    setup,
                    stock_top_z,
                    model_top_z,
                    wcs_origin_proj,
                    features: &mut features,
                    highlight_faces: &mut highlight_faces,
                    highlight_edges: &mut highlight_edges,
                    direction_arrows: &mut direction_arrows,
                    top_candidates: &mut top_candidates,
                    bottom_candidates: &mut bottom_candidates,
                    raw_hole_faces: &mut raw_hole_faces,
                    primary_hole_diameter_mm: &mut primary_hole_diameter_mm,
                };
                extract_open_chain(&mut acc, start_edge, resolved_edges, *reverse, *side_flip)?;
            }
            GeometrySelectionItem::CylindricalHole { binding } => {
                let mut acc = BRepExtractAccum {
                    doc,
                    setup,
                    stock_top_z,
                    model_top_z,
                    wcs_origin_proj,
                    features: &mut features,
                    highlight_faces: &mut highlight_faces,
                    highlight_edges: &mut highlight_edges,
                    direction_arrows: &mut direction_arrows,
                    top_candidates: &mut top_candidates,
                    bottom_candidates: &mut bottom_candidates,
                    raw_hole_faces: &mut raw_hole_faces,
                    primary_hole_diameter_mm: &mut primary_hole_diameter_mm,
                };
                collect_cylindrical_hole(&mut acc, binding)?;
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
                let sk_frame = verify_sketch_aligned_with_setup(doc, sk, setup)?;
                let mut acc = SketchExtractAccum {
                    doc_has_bodies,
                    setup,
                    sk_frame,
                    features: &mut features,
                    direction_arrows: &mut direction_arrows,
                    top_candidates: &mut top_candidates,
                    bottom_candidates: &mut bottom_candidates,
                    primary_hole_diameter_mm: &mut primary_hole_diameter_mm,
                };
                extract_sketch_region(&mut acc, reg);
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
                let sk_frame = verify_sketch_aligned_with_setup(doc, sk, setup)?;
                let mut acc = SketchExtractAccum {
                    doc_has_bodies,
                    setup,
                    sk_frame,
                    features: &mut features,
                    direction_arrows: &mut direction_arrows,
                    top_candidates: &mut top_candidates,
                    bottom_candidates: &mut bottom_candidates,
                    primary_hole_diameter_mm: &mut primary_hole_diameter_mm,
                };
                extract_sketch_entity(&mut acc, ent, *reverse, *side_flip, true);
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
                let sk_frame = verify_sketch_aligned_with_setup(doc, sk, setup)?;
                let mut acc = SketchExtractAccum {
                    doc_has_bodies,
                    setup,
                    sk_frame,
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

    let mut brep_acc = BRepExtractAccum {
        doc,
        setup,
        stock_top_z,
        model_top_z,
        wcs_origin_proj,
        features: &mut features,
        highlight_faces: &mut highlight_faces,
        highlight_edges: &mut highlight_edges,
        direction_arrows: &mut direction_arrows,
        top_candidates: &mut top_candidates,
        bottom_candidates: &mut bottom_candidates,
        raw_hole_faces: &mut raw_hole_faces,
        primary_hole_diameter_mm: &mut primary_hole_diameter_mm,
    };
    finalize_cylindrical_holes(&mut brep_acc);

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

pub(super) fn build_chain_arrow(
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
