use chip_cad::CadDocument;
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3, RigidFrame3};
use chip_sketch::{PlanarVectorRegion, SketchEntity, SketchEntityEntry, SketchLayer};

use crate::{
    setup::{ResolvedSetup, resolve_sketch_world_frame_with_doc},
    wcs::CamError,
};

use super::{ChainDirectionArrow3D, ExtractedFeature25D, build_chain_arrow};

pub(super) fn verify_sketch_aligned_with_setup(
    doc: &CadDocument,
    sk: &SketchLayer,
    setup: &ResolvedSetup,
) -> Result<RigidFrame3, CamError> {
    let sk_frame = resolve_sketch_world_frame_with_doc(doc, sk, setup);
    let cos_z = sk_frame.z_axis.dot(setup.frame.z_axis);
    if cos_z < 0.999 {
        return Err(CamError::SketchNotAlignedWithSetupZ {
            sketch_name: sk.name.clone(),
            cos_z,
        });
    }
    Ok(sk_frame)
}

pub(super) struct SketchExtractAccum<'a> {
    pub(super) doc_has_bodies: bool,
    pub(super) setup: &'a ResolvedSetup,
    pub(super) sk_frame: RigidFrame3,
    pub(super) features: &'a mut Vec<ExtractedFeature25D>,
    pub(super) direction_arrows: &'a mut Vec<ChainDirectionArrow3D>,
    pub(super) top_candidates: &'a mut Vec<f64>,
    pub(super) bottom_candidates: &'a mut Vec<f64>,
    pub(super) primary_hole_diameter_mm: &'a mut Option<f64>,
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

pub(super) fn extract_sketch_region(acc: &mut SketchExtractAccum<'_>, reg: &PlanarVectorRegion) {
    let sk_frame = acc.sk_frame;
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

fn emit_sketch_polyline_contour(
    acc: &mut SketchExtractAccum<'_>,
    raw_poly: &ArcPolyline2D,
    reverse: bool,
    side_flip: bool,
) {
    if raw_poly.segments.is_empty() {
        return;
    }
    let (mut poly_wcs, plane_z_wcs) =
        transform_sketch_polyline_to_wcs(raw_poly, &acc.sk_frame, &acc.setup.frame);
    if poly_wcs.closed && poly_wcs.signed_area().abs() > 1e-6 {
        // Default closed sketch contour winds CW so `LeftClimb` profiles outside in Climb Milling!
        poly_wcs = poly_wcs.ensure_orientation(false);
    }
    if reverse {
        poly_wcs = poly_wcs.reverse();
    }
    if let Some(arrow) = build_chain_arrow(&poly_wcs, plane_z_wcs, side_flip, &acc.setup.frame) {
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

fn emit_sketch_hole_points(
    acc: &mut SketchExtractAccum<'_>,
    holes: &[chip_sketch::HolePoint2D],
    reverse: bool,
    side_flip: bool,
    emit_contours: bool,
) {
    for h in holes {
        if h.diameter_mm <= 1e-4 {
            continue;
        }
        let c_world =
            acc.sk_frame
                .local_to_world_point(DVec3::new(h.center_xy.x, h.center_xy.y, 0.0));
        let c_wcs3 = acc.setup.frame.world_to_local_point(c_world);
        let (top_z, bot_z) = infer_sketch_z_span(acc.doc_has_bodies, acc.setup, c_wcs3.z);
        let hole_bot_z = if !acc.doc_has_bodies {
            acc.setup.stock.aabb_wcs().min.z
        } else {
            bot_z
        };
        if acc.primary_hole_diameter_mm.is_none() {
            *acc.primary_hole_diameter_mm = Some(h.diameter_mm);
        }
        acc.top_candidates.push(top_z);
        acc.bottom_candidates.push(hole_bot_z);
        acc.features.push(ExtractedFeature25D::CylindricalHole {
            primary_face: None,
            coaxial_faces: Vec::new(),
            center_wcs: DVec2::new(c_wcs3.x, c_wcs3.y),
            diameter_mm: h.diameter_mm,
            top_z_wcs: top_z,
            bottom_z_wcs: hole_bot_z,
            is_through_hole: !acc.doc_has_bodies,
        });
        if emit_contours {
            let circle_poly = chip_sketch::build_circle_polyline(h.center_xy, h.diameter_mm);
            emit_sketch_polyline_contour(acc, &circle_poly, reverse, side_flip);
        }
    }
}

pub(super) fn extract_sketch_entity(
    acc: &mut SketchExtractAccum<'_>,
    entry: &SketchEntityEntry,
    reverse: bool,
    side_flip: bool,
    emit_contour_for_circle: bool,
) {
    match &entry.entity {
        SketchEntity::Circle { .. } | SketchEntity::PointPattern(_) => {
            let holes = entry.entity.evaluate_holes();
            emit_sketch_hole_points(acc, &holes, reverse, side_flip, emit_contour_for_circle);
        }
        SketchEntity::Rectangle { .. }
        | SketchEntity::Slot { .. }
        | SketchEntity::RegularPolygon { .. }
        | SketchEntity::Polyline { .. } => {
            let raw_poly = entry.entity.to_arc_polyline();
            emit_sketch_polyline_contour(acc, &raw_poly, reverse, side_flip);
        }
        SketchEntity::Text(txt) => {
            if txt.font.is_single_line() {
                for stroke in txt.evaluate_polylines(None) {
                    emit_sketch_polyline_contour(acc, &stroke, reverse, side_flip);
                }
            } else {
                for reg in txt.evaluate_outline_regions(entry.sub_layer.as_deref(), None) {
                    if reverse || side_flip {
                        emit_sketch_polyline_contour(acc, &reg.outer_ccw, reverse, side_flip);
                        for isl in &reg.islands_cw {
                            emit_sketch_polyline_contour(acc, isl, reverse, !side_flip);
                        }
                    } else {
                        extract_sketch_region(acc, &reg);
                    }
                }
            }
        }
    }
}

pub(super) fn extract_sketch_layer_all(
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

    // 1. Emit `CylindricalHole` for all circles and point-patterns on the layer
    for ent in &sk.entities {
        if ent.is_construction || !matches_layer(ent.sub_layer.as_deref()) {
            continue;
        }
        if matches!(
            ent.entity,
            SketchEntity::Circle { .. } | SketchEntity::PointPattern(_)
        ) {
            extract_sketch_entity(acc, ent, reverse, side_flip, false);
            let key = (sk.id.clone(), ent.id.clone());
            if !highlight_entities.contains(&key) {
                highlight_entities.push(key);
            }
        }
    }

    // 2. Closed planar regions on the layer: if `reverse || side_flip` is requested, emit oriented
    //    `Contour`s so inside/outside profile direction is honored; otherwise emit `PlanarFace`s!
    let sk_frame = acc.sk_frame;
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
            extract_sketch_region(acc, reg);
        }
        let key = (sk.id.clone(), reg.id.clone());
        if !highlight_regions.contains(&key) {
            highlight_regions.push(key);
        }
    }

    // 3. Open chains and single-line Hershey text strokes on the layer -> emit `Contour`s
    for ent in &sk.entities {
        if ent.is_construction || !matches_layer(ent.sub_layer.as_deref()) {
            continue;
        }
        let is_open_or_hershey = match &ent.entity {
            SketchEntity::Polyline { polyline, .. } => !polyline.closed,
            SketchEntity::Text(txt) => txt.font.is_single_line(),
            _ => false,
        };
        if is_open_or_hershey {
            extract_sketch_entity(acc, ent, reverse, side_flip, false);
            let key = (sk.id.clone(), ent.id.clone());
            if !highlight_entities.contains(&key) {
                highlight_entities.push(key);
            }
        }
    }
}
