//! Interactive 2D Sketch entity & planar region hit-testing (`SketchPicker2D`).

use chip_math::DVec2;

use crate::model::SketchLayer;

/// Hit result from [`pick_sketch_layer_2d`].
#[derive(Debug, Clone, PartialEq)]
pub enum SketchPickHit {
    /// Cursor hit a curve or circle boundary within `edge_tol_mm`.
    Entity {
        sketch_id: String,
        entity_id: String,
        distance_mm: f64,
    },
    /// Cursor clicked inside a closed [`crate::model::PlanarVectorRegion`] (outside its interior islands).
    Region {
        sketch_id: String,
        region_id: String,
        area_mm2: f64,
    },
}

/// Picks the highest-priority entity edge (within `edge_tol_mm`) or smallest-area enclosing
/// [`crate::model::PlanarVectorRegion`] at `query_xy` on `sketch`.
pub fn pick_sketch_layer_2d(
    sketch: &SketchLayer,
    query_xy: DVec2,
    edge_tol_mm: f64,
    prefer_regions: bool,
) -> Option<SketchPickHit> {
    if !sketch.visible || !query_xy.is_finite() {
        return None;
    }

    // 1. Closest curve/circle/stroke boundary within `edge_tol_mm`
    let mut best_edge: Option<(String, f64)> = None;
    for ent in &sketch.entities {
        if ent.is_construction {
            continue;
        }
        for poly in ent.entity.to_arc_polylines(None) {
            if poly.segments.is_empty() {
                continue;
            }
            let d = poly.distance_to_point(query_xy);
            if d <= edge_tol_mm && best_edge.as_ref().is_none_or(|(_, prev_d)| d < *prev_d) {
                best_edge = Some((ent.id.clone(), d));
            }
        }
    }

    // 2. Smallest-area enclosing closed region
    let mut best_region: Option<(String, f64)> = None;
    for reg in &sketch.regions {
        if reg.contains_point(query_xy)
            && best_region
                .as_ref()
                .is_none_or(|(_, prev_a)| reg.area_mm2 < *prev_a)
        {
            best_region = Some((reg.id.clone(), reg.area_mm2));
        }
    }

    if prefer_regions && let Some((region_id, area_mm2)) = best_region {
        return Some(SketchPickHit::Region {
            sketch_id: sketch.id.clone(),
            region_id,
            area_mm2,
        });
    }

    if let Some((entity_id, distance_mm)) = best_edge {
        return Some(SketchPickHit::Entity {
            sketch_id: sketch.id.clone(),
            entity_id,
            distance_mm,
        });
    }

    best_region.map(|(region_id, area_mm2)| SketchPickHit::Region {
        sketch_id: sketch.id.clone(),
        region_id,
        area_mm2,
    })
}
