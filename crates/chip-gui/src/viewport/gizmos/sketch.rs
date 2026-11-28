use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3, RigidFrame3};
use chip_sketch::{SketchDiagnosticKind, SketchEntity, SketchPickHit};

use crate::{
    camera::CadCamera,
    viewport::{
        gpu_types::EdgeInstanceGpu,
        highlight::{HighlightState, dvec3_f32},
    },
};

pub(super) fn emit_sketch_layer_visuals(
    highlight: &HighlightState,
    camera: &CadCamera,
    out: &mut Vec<EdgeInstanceGpu>,
) {
    let diag_tick = (camera.scene_radius_mm * 0.025).clamp(0.8, 4.0);
    for vis in &highlight.sketch_visuals {
        let sk = &vis.sketch;
        if !sk.visible {
            continue;
        }
        let wf = &vis.world_frame;
        // Slight +0.04 mm lift along sketch +Z normal so 2D curves sit crisply above Stock Top / Machined Stock!
        let lift = wf.z_axis * 0.04;

        // 1. Base sketch entities (primitives, hole patterns, outline & Hershey text, polylines)
        for ent in &sk.entities {
            let key = (sk.id.clone(), ent.id.clone());
            let is_sel = highlight.extra_selected_sketch_entities.contains(&key)
                || matches!(
                    &highlight.selected_sketch,
                    Some(SketchPickHit::Entity {
                        sketch_id,
                        entity_id,
                        ..
                    }) if sketch_id == &sk.id && entity_id == &ent.id
                );
            let is_hov = matches!(
                &highlight.hovered_sketch,
                Some(SketchPickHit::Entity {
                    sketch_id,
                    entity_id,
                    ..
                }) if sketch_id == &sk.id && entity_id == &ent.id
            );

            let is_light = highlight.theme.is_light();
            for poly in ent.entity.to_arc_polylines(None) {
                let (col, width_px) = if is_sel {
                    if is_light {
                        ([0.86, 0.48, 0.02, 1.0], 3.4)
                    } else {
                        ([1.0, 0.80, 0.20, 1.0], 3.4)
                    }
                } else if is_hov {
                    if is_light {
                        ([0.04, 0.56, 0.88, 1.0], 3.1)
                    } else {
                        ([0.25, 0.96, 1.0, 1.0], 3.1)
                    }
                } else if ent.is_construction {
                    if is_light {
                        ([0.40, 0.47, 0.58, 0.72], 1.65)
                    } else {
                        ([0.56, 0.65, 0.78, 0.65], 1.65)
                    }
                } else if matches!(
                    ent.entity,
                    SketchEntity::Circle { .. } | SketchEntity::PointPattern(_)
                ) {
                    if is_light {
                        ([0.06, 0.54, 0.44, 0.96], 2.2)
                    } else {
                        ([0.42, 0.92, 0.78, 0.96], 2.2)
                    }
                } else if poly.closed {
                    if is_light {
                        ([0.08, 0.42, 0.80, 0.96], 2.05)
                    } else {
                        ([0.38, 0.78, 0.98, 0.94], 2.05)
                    }
                } else if is_light {
                    ([0.82, 0.34, 0.12, 0.96], 2.2)
                } else {
                    ([1.0, 0.56, 0.34, 0.96], 2.2)
                };
                emit_arc_polyline_edges(&poly, wf, lift, col, width_px, out);
            }
        }

        // 2. Highlighted / hovered `PlanarVectorRegion` outer + island boundaries
        for reg in &sk.regions {
            let key = (sk.id.clone(), reg.id.clone());
            let is_sel = highlight.extra_selected_sketch_regions.contains(&key)
                || matches!(
                    &highlight.selected_sketch,
                    Some(SketchPickHit::Region {
                        sketch_id,
                        region_id,
                        ..
                    }) if sketch_id == &sk.id && region_id == &reg.id
                );
            let is_hov = matches!(
                &highlight.hovered_sketch,
                Some(SketchPickHit::Region {
                    sketch_id,
                    region_id,
                    ..
                }) if sketch_id == &sk.id && region_id == &reg.id
            );
            if !is_sel && !is_hov {
                continue;
            }
            let (col, width_px) = if is_sel {
                ([1.0, 0.80, 0.20, 1.0], 3.5)
            } else {
                ([0.25, 0.96, 1.0, 1.0], 3.2)
            };
            emit_arc_polyline_edges(&reg.outer_ccw, wf, lift, col, width_px, out);
            for isl in &reg.islands_cw {
                emit_arc_polyline_edges(isl, wf, lift, col, width_px, out);
            }
        }

        // 3. Diagnostic markers at remaining unclosed open-chain endpoints
        for diag in &sk.diagnostics {
            if diag.kind == SketchDiagnosticKind::OpenChainRemaining
                && let Some(pt) = diag.location_xy
            {
                let c = wf.local_to_world_point(DVec3::new(pt.x, pt.y, 0.0)) + lift;
                let dx = wf.x_axis * diag_tick;
                let dy = wf.y_axis * diag_tick;
                for (a, b) in [
                    (c - dx, c + dy),
                    (c + dy, c + dx),
                    (c + dx, c - dy),
                    (c - dy, c - dx),
                ] {
                    out.push(EdgeInstanceGpu {
                        p0_mm: dvec3_f32(a),
                        p1_mm: dvec3_f32(b),
                        color: [1.0, 0.36, 0.36, 1.0],
                        width_px: 2.3,
                    });
                }
            }
        }
    }

    // Live interactive 2D Sketch rubber-band preview + 2D Snap diamond marker
    for (wf, preview_poly) in &highlight.sketch_preview_polys {
        let lift = wf.z_axis * 0.08;
        emit_arc_polyline_edges(preview_poly, wf, lift, [0.22, 0.96, 1.0, 0.98], 2.6, out);
    }
    if let Some((snap_world, _, _)) = highlight.sketch_snap_marker {
        let (_, _, cam_r, cam_u) = camera.basis();
        let dx = cam_r * (diag_tick * 1.15);
        let dy = cam_u * (diag_tick * 1.15);
        for (a, b) in [
            (snap_world - dx, snap_world + dy),
            (snap_world + dy, snap_world + dx),
            (snap_world + dx, snap_world - dy),
            (snap_world - dy, snap_world - dx),
            (snap_world - dx * 1.4, snap_world + dx * 1.4),
            (snap_world - dy * 1.4, snap_world + dy * 1.4),
        ] {
            out.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(a),
                p1_mm: dvec3_f32(b),
                color: [1.0, 0.84, 0.18, 1.0],
                width_px: 2.5,
            });
        }
    }
}

pub(super) fn emit_arc_polyline_edges(
    poly: &ArcPolyline2D,
    wf: &RigidFrame3,
    lift: DVec3,
    color: [f32; 4],
    width_px: f32,
    out: &mut Vec<EdgeInstanceGpu>,
) {
    let to_w = |p: DVec2| wf.local_to_world_point(DVec3::new(p.x, p.y, 0.0)) + lift;
    for seg in &poly.segments {
        match *seg {
            ArcSegment2D::Line { start, end } => {
                out.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(to_w(start)),
                    p1_mm: dvec3_f32(to_w(end)),
                    color,
                    width_px,
                });
            }
            ArcSegment2D::Arc { start, end, .. } => {
                let sweep = seg.signed_sweep_rad().abs();
                let steps = ((sweep / (7.5_f64.to_radians())).ceil() as usize).clamp(4, 64);
                let mut prev = to_w(start);
                for k in 1..=steps {
                    let next = if k == steps {
                        to_w(end)
                    } else if let Some((pt, _)) = seg.eval_at((k as f64) / (steps as f64)) {
                        to_w(pt)
                    } else {
                        to_w(end)
                    };
                    out.push(EdgeInstanceGpu {
                        p0_mm: dvec3_f32(prev),
                        p1_mm: dvec3_f32(next),
                        color,
                        width_px,
                    });
                    prev = next;
                }
            }
        }
    }
}
