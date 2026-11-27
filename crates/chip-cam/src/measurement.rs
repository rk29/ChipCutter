//! Exact `f64` 2D sketch measurement snap adapter in active Setup WCS coordinates (`chip-cam`).
//!
//! Core B-Rep/mesh measurement types and geometry kernels live in `chip_cad::measurement`.

use chip_cad::MeasureSnapTarget;
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3, RigidFrame3};
use chip_sketch::{SketchEntity, SketchLayer};

/// Snaps a 2D point on a [`SketchLayer`] into an exact `f64` [`MeasureSnapTarget`] in Setup WCS.
pub fn snap_sketch_to_measure_target(
    sketch: &SketchLayer,
    sketch_world_frame: &RigidFrame3,
    setup_frame: &RigidFrame3,
    query_xy: DVec2,
    snap_radius_mm: f64,
) -> Option<MeasureSnapTarget> {
    let tol = snap_radius_mm.max(0.25);
    let to_wcs = |p2: DVec2| -> DVec3 {
        let pw = sketch_world_frame.local_to_world_point(DVec3::new(p2.x, p2.y, 0.0));
        setup_frame.world_to_local_point(pw)
    };

    let mut best_circle: Option<(f64, DVec2, f64)> = None;
    let mut best_point: Option<(f64, DVec2)> = None;

    for entry in &sketch.entities {
        match &entry.entity {
            SketchEntity::Circle { diameter, .. } => {
                if let Some(c) = entry.entity.circle_center_mm() {
                    let r = 0.5 * diameter.value_mm;
                    let d_center = query_xy.distance(c);
                    let d_rim = (d_center - r).abs();
                    let d = d_center.min(d_rim);
                    if d <= tol && best_circle.is_none_or(|(bd, _, _)| d < bd) {
                        best_circle = Some((d, c, diameter.value_mm));
                    }
                }
            }
            SketchEntity::PointPattern(_) => {
                for hole in entry.entity.evaluate_holes() {
                    let r = 0.5 * hole.diameter_mm;
                    let d_center = query_xy.distance(hole.center_xy);
                    let d_rim = (d_center - r).abs();
                    let d = d_center.min(d_rim);
                    if d <= tol && best_circle.is_none_or(|(bd, _, _)| d < bd) {
                        best_circle = Some((d, hole.center_xy, hole.diameter_mm));
                    }
                }
            }
            _ => {
                let poly = entry.entity.to_arc_polyline();
                collect_polyline_snap_candidates(
                    &poly,
                    query_xy,
                    tol,
                    &mut best_circle,
                    &mut best_point,
                );
            }
        }
    }

    for region in &sketch.regions {
        collect_polyline_snap_candidates(
            &region.outer_ccw,
            query_xy,
            tol,
            &mut best_circle,
            &mut best_point,
        );
        for isl in &region.islands_cw {
            collect_polyline_snap_candidates(isl, query_xy, tol, &mut best_circle, &mut best_point);
        }
    }

    if let Some((_, c, dia)) = best_circle {
        return Some(MeasureSnapTarget::SketchCircle {
            sketch_id: sketch.id.clone(),
            center_wcs: to_wcs(c),
            diameter_mm: dia,
        });
    }
    if let Some((_, pt)) = best_point {
        return Some(MeasureSnapTarget::SketchPoint {
            sketch_id: sketch.id.clone(),
            pos_wcs: to_wcs(pt),
        });
    }

    None
}

fn collect_polyline_snap_candidates(
    poly: &ArcPolyline2D,
    query_xy: DVec2,
    tol: f64,
    best_circle: &mut Option<(f64, DVec2, f64)>,
    best_point: &mut Option<(f64, DVec2)>,
) {
    for seg in &poly.segments {
        match *seg {
            ArcSegment2D::Line { start, end } => {
                let mid = 0.5 * (start + end);
                for pt in [start, end, mid] {
                    let d = query_xy.distance(pt);
                    if d <= tol && best_point.is_none_or(|(bd, _)| d < bd) {
                        *best_point = Some((d, pt));
                    }
                }
            }
            ArcSegment2D::Arc {
                start,
                end,
                center,
                radius,
                ..
            } => {
                let d_center = query_xy.distance(center);
                let d_rim = (d_center - radius).abs();
                if d_center <= tol && best_circle.is_none_or(|(bd, _, _)| d_center < bd) {
                    *best_circle = Some((d_center, center, 2.0 * radius));
                } else if d_rim <= tol * 0.5 && best_circle.is_none_or(|(bd, _, _)| d_rim < bd) {
                    *best_circle = Some((d_rim, center, 2.0 * radius));
                }
                for pt in [start, end] {
                    let d = query_xy.distance(pt);
                    if d <= tol && best_point.is_none_or(|(bd, _)| d < bd) {
                        *best_point = Some((d, pt));
                    }
                }
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::{CadDocument, MeasurementResult, measure_pair};
    use chip_math::{DisplayUnit, ParamLength};
    use chip_sketch::{SketchEntity, SketchEntityEntry, SketchLayer};

    use super::*;
    use crate::{Setup, WorkOffsetSlot, resolve_sketch_world_frame};

    #[test]
    fn sketch_circle_snap_and_center_to_center_pitch_measurement() {
        let mut sk = SketchLayer::new("sk_1", "Holes", "setup_1");
        sk.entities.push(SketchEntityEntry {
            id: "c1".to_string(),
            name: "C1".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::new_circle(
                ParamLength::from_mm(10.0),
                ParamLength::from_mm(20.0),
                ParamLength::from_mm(8.0),
            ),
        });
        sk.entities.push(SketchEntityEntry {
            id: "c2".to_string(),
            name: "C2".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::new_circle(
                ParamLength::from_mm(40.0),
                ParamLength::from_mm(60.0),
                ParamLength::from_mm(10.0),
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
        let wf = resolve_sketch_world_frame(false, &sk, &rs);
        let t1 = snap_sketch_to_measure_target(&sk, &wf, &rs.frame, DVec2::new(10.2, 20.1), 1.0)
            .unwrap();
        let t2 = snap_sketch_to_measure_target(&sk, &wf, &rs.frame, DVec2::new(40.0, 60.0), 1.0)
            .unwrap();
        match measure_pair(&t1, &t2) {
            MeasurementResult::CylinderToCylinder {
                diameter1_mm,
                diameter2_mm,
                center_distance_xy_mm,
                min_wall_thickness_mm,
                ..
            } => {
                assert!((diameter1_mm - 8.0).abs() < 1e-6);
                assert!((diameter2_mm - 10.0).abs() < 1e-6);
                // 3-4-5 triangle: dx=30, dy=40 => pitch=50 mm; wall = 50 - 4 - 5 = 41 mm
                assert!((center_distance_xy_mm - 50.0).abs() < 1e-6);
                assert!((min_wall_thickness_mm.unwrap() - 41.0).abs() < 1e-6);
            }
            other => panic!("expected CylinderToCylinder for sketch circles, got {other:?}"),
        }
    }
}
