use chip_cad::CurveKind;
use chip_cam::{resolve_sketch_world_frame, resolve_sketch_world_frame_with_doc};
use chip_math::{Aabb2, DVec2, DVec3, ParamLength};
use chip_sketch::{
    PointPatternSpec, PolygonEvalSpec, PolygonSizingMode, RectAnchorMode, RectEvalSpec,
    SketchEntity, SketchEntityEntry, SketchSnapContext, SketchTextSpec, SlotLengthMode,
    build_circle_polyline, build_rectangle_polyline, build_regular_polygon_polyline,
    build_slot_polyline, snap_sketch_point_2d,
};

use super::SketchAuthorInteractionInput;
use crate::{
    app::ChipCutterApp,
    panels::{SketchAuthorTool, ViewportPickMode},
};

impl ChipCutterApp {
    pub(crate) fn handle_sketch_author_viewport_interaction(
        &mut self,
        input: SketchAuthorInteractionInput,
    ) {
        let SketchAuthorInteractionInput {
            sketch_idx,
            tool,
            ray,
            mm_per_px,
            primary_click,
            right_click,
        } = input;

        if right_click {
            if self.selection.sketch_author_start_xy.take().is_none() {
                self.selection.pick_mode = ViewportPickMode::InspectEntity;
            }
            return;
        }
        if sketch_idx >= self.project.sketches.len() {
            self.selection.pick_mode = ViewportPickMode::InspectEntity;
            return;
        }
        let Some(Ok(rs)) = self.active_resolved_setup() else {
            return;
        };
        let wf = if let Some(m) = &self.assets.model {
            resolve_sketch_world_frame_with_doc(&m.doc, &self.project.sketches[sketch_idx], &rs)
        } else {
            resolve_sketch_world_frame(false, &self.project.sketches[sketch_idx], &rs)
        };

        let denom = ray.dir.dot(wf.z_axis);
        if denom.abs() <= 1e-6 {
            return;
        }
        let t = (wf.origin_mm - ray.origin).dot(wf.z_axis) / denom;
        if t <= 0.0 {
            return;
        }
        let hit_world = ray.origin + ray.dir * t;
        let hit_local = wf.world_to_local_point(hit_world);
        let raw_xy = DVec2::new(hit_local.x, hit_local.y);

        // Project stock corners & 3D STEP vertices/circle centers into Sketch XY for 2D Snapping!
        let st_wcs = rs.stock.aabb_wcs();
        let stock_xy = if st_wcs.is_valid() {
            let mut b = Aabb2::EMPTY;
            for p_wcs in [
                DVec3::new(st_wcs.min.x, st_wcs.min.y, st_wcs.max.z),
                DVec3::new(st_wcs.max.x, st_wcs.min.y, st_wcs.max.z),
                DVec3::new(st_wcs.max.x, st_wcs.max.y, st_wcs.max.z),
                DVec3::new(st_wcs.min.x, st_wcs.max.y, st_wcs.max.z),
            ] {
                let pw = rs.frame.local_to_world_point(p_wcs);
                let pl = wf.world_to_local_point(pw);
                b.extend_point(DVec2::new(pl.x, pl.y));
            }
            b.is_valid().then_some(b)
        } else {
            None
        };

        let mut cad_pts_2d: Vec<DVec2> = Vec::new();
        if let Some(m) = &self.assets.model {
            for edge in &m.doc.edges {
                match edge.curve {
                    CurveKind::Line { start_mm, end_mm } => {
                        for pw in [start_mm, end_mm] {
                            let pl = wf.world_to_local_point(pw);
                            if pl.z.abs() <= 25.0 {
                                cad_pts_2d.push(DVec2::new(pl.x, pl.y));
                            }
                        }
                    }
                    CurveKind::CircleArc {
                        center_mm,
                        start_mm,
                        end_mm,
                        ..
                    } => {
                        for pw in [center_mm, start_mm, end_mm] {
                            let pl = wf.world_to_local_point(pw);
                            if pl.z.abs() <= 25.0 {
                                cad_pts_2d.push(DVec2::new(pl.x, pl.y));
                            }
                        }
                    }
                    _ => {}
                }
            }
        }

        let snap_res = snap_sketch_point_2d(
            raw_xy,
            &SketchSnapContext {
                sketch: &self.project.sketches[sketch_idx],
                stock_xy_bounds: stock_xy,
                projected_cad_points: &cad_pts_2d,
                snap_radius_mm: (12.0 * mm_per_px).clamp(0.5, 10.0),
                grid_step_mm: self.selection.sketch_snap_grid_mm,
            },
        );
        let pt_xy = snap_res.point_xy;
        let snap_world = wf.local_to_world_point(DVec3::new(pt_xy.x, pt_xy.y, 0.06));
        let badge = snap_res.kind.map_or("XY", |k| k.badge());
        self.highlight.sketch_snap_marker = Some((snap_world, badge, pt_xy));

        // Build live rubber-band preview when first point is active
        if let Some(p0) = self.selection.sketch_author_start_xy {
            let preview_polys = build_rubber_band_polys(tool, p0, pt_xy);
            for poly in preview_polys {
                if !poly.segments.is_empty() {
                    self.highlight.sketch_preview_polys.push((wf, poly));
                }
            }
        }

        if !primary_click {
            return;
        }

        if tool == SketchAuthorTool::PlaceText {
            self.history
                .record_before_edit("Place CAM Text", None, &self.project);
            let sk = &mut self.project.sketches[sketch_idx];
            let n = sk.entities.len() + 1;
            let spec = SketchTextSpec {
                x: ParamLength::from_mm(pt_xy.x),
                y: ParamLength::from_mm(pt_xy.y),
                ..SketchTextSpec::default()
            };
            sk.entities.push(SketchEntityEntry {
                id: format!("text_{n}"),
                name: format!("Text {n}"),
                sub_layer: None,
                is_construction: false,
                entity: SketchEntity::Text(spec),
            });
            sk.rebuild_regions(None);
            self.selection.selected_sketch_entity_idx = Some(sk.entities.len() - 1);
            self.workers.cached_frame_geo = None;
            return;
        }

        let Some(p0) = self.selection.sketch_author_start_xy else {
            self.selection.sketch_author_start_xy = Some(pt_xy);
            return;
        };

        let delta = pt_xy - p0;
        let dist = delta.length();
        if dist <= 0.25 {
            return;
        }
        self.selection.sketch_author_start_xy = None;
        self.history.record_before_edit(
            format!("Draw Sketch {}", tool.short_label()),
            None,
            &self.project,
        );
        let sk = &mut self.project.sketches[sketch_idx];
        let n = sk.entities.len() + 1;
        let new_entry = match tool {
            SketchAuthorTool::Rectangle => {
                let c = 0.5 * (p0 + pt_xy);
                let w = delta.x.abs().max(1.0);
                let h = delta.y.abs().max(1.0);
                SketchEntityEntry {
                    id: format!("rect_{n}"),
                    name: format!("Rectangle {n}"),
                    sub_layer: None,
                    is_construction: false,
                    entity: SketchEntity::Rectangle {
                        anchor: RectAnchorMode::Center,
                        x: ParamLength::from_mm(c.x),
                        y: ParamLength::from_mm(c.y),
                        width: ParamLength::from_mm(w),
                        height: ParamLength::from_mm(h),
                        corner_radius: ParamLength::from_mm(0.0),
                        rotation_deg: 0.0,
                    },
                }
            }
            SketchAuthorTool::Circle => SketchEntityEntry {
                id: format!("circle_{n}"),
                name: format!("Circle {n}"),
                sub_layer: None,
                is_construction: false,
                entity: SketchEntity::new_circle(
                    ParamLength::from_mm(p0.x),
                    ParamLength::from_mm(p0.y),
                    ParamLength::from_mm(2.0 * dist),
                ),
            },
            SketchAuthorTool::Slot => {
                let c = 0.5 * (p0 + pt_xy);
                let ang = delta.y.atan2(delta.x).to_degrees();
                SketchEntityEntry {
                    id: format!("slot_{n}"),
                    name: format!("Slot {n}"),
                    sub_layer: None,
                    is_construction: false,
                    entity: SketchEntity::Slot {
                        mode: SlotLengthMode::CenterToCenter,
                        x: ParamLength::from_mm(c.x),
                        y: ParamLength::from_mm(c.y),
                        length: ParamLength::from_mm(dist),
                        width: ParamLength::from_mm(8.0),
                        angle_deg: ang,
                    },
                }
            }
            SketchAuthorTool::Polygon => {
                let ang = delta.y.atan2(delta.x).to_degrees();
                SketchEntityEntry {
                    id: format!("poly_{n}"),
                    name: format!("Hexagon {n}"),
                    sub_layer: None,
                    is_construction: false,
                    entity: SketchEntity::RegularPolygon {
                        center_x: ParamLength::from_mm(p0.x),
                        center_y: ParamLength::from_mm(p0.y),
                        sides: 6,
                        sizing: PolygonSizingMode::CircumscribedDiameter,
                        diameter: ParamLength::from_mm(2.0 * dist),
                        corner_radius: ParamLength::from_mm(0.0),
                        rotation_deg: ang,
                    },
                }
            }
            SketchAuthorTool::BoltCircle | SketchAuthorTool::PlaceText => {
                let ang = delta.y.atan2(delta.x).to_degrees();
                SketchEntityEntry {
                    id: format!("bolts_{n}"),
                    name: format!("Bolt Circle {n}"),
                    sub_layer: Some("HOLES".to_string()),
                    is_construction: false,
                    entity: SketchEntity::PointPattern(PointPatternSpec::BoltCircle {
                        center_x: ParamLength::from_mm(p0.x),
                        center_y: ParamLength::from_mm(p0.y),
                        bolt_circle_diameter: ParamLength::from_mm(2.0 * dist),
                        hole_diameter: ParamLength::from_mm(6.0),
                        count: 6,
                        start_angle_deg: ang,
                    }),
                }
            }
        };
        sk.entities.push(new_entry);
        sk.rebuild_regions(None);
        self.selection.selected_sketch_entity_idx = Some(sk.entities.len() - 1);
        self.workers.cached_frame_geo = None;
    }
}

fn build_rubber_band_polys(
    tool: SketchAuthorTool,
    p0: DVec2,
    p1: DVec2,
) -> Vec<chip_math::ArcPolyline2D> {
    let delta = p1 - p0;
    let dist = delta.length();
    if dist <= 0.1 {
        return Vec::new();
    }
    match tool {
        SketchAuthorTool::Rectangle => {
            let c = 0.5 * (p0 + p1);
            vec![build_rectangle_polyline(RectEvalSpec {
                anchor: RectAnchorMode::Center,
                anchor_xy: c,
                width_mm: delta.x.abs(),
                height_mm: delta.y.abs(),
                corner_radius_mm: 0.0,
                rotation_deg: 0.0,
            })]
        }
        SketchAuthorTool::Circle => vec![build_circle_polyline(p0, 2.0 * dist)],
        SketchAuthorTool::Slot => {
            let c = 0.5 * (p0 + p1);
            let ang = delta.y.atan2(delta.x).to_degrees();
            vec![build_slot_polyline(
                SlotLengthMode::CenterToCenter,
                c,
                dist,
                8.0,
                ang,
            )]
        }
        SketchAuthorTool::Polygon => {
            let ang = delta.y.atan2(delta.x).to_degrees();
            vec![build_regular_polygon_polyline(PolygonEvalSpec {
                center_xy: p0,
                sides: 6,
                sizing: PolygonSizingMode::CircumscribedDiameter,
                diameter_mm: 2.0 * dist,
                corner_radius_mm: 0.0,
                rotation_deg: ang,
            })]
        }
        SketchAuthorTool::BoltCircle => {
            let mut out = vec![build_circle_polyline(p0, 2.0 * dist)];
            let pat = PointPatternSpec::BoltCircle {
                center_x: ParamLength::from_mm(p0.x),
                center_y: ParamLength::from_mm(p0.y),
                bolt_circle_diameter: ParamLength::from_mm(2.0 * dist),
                hole_diameter: ParamLength::from_mm(6.0),
                count: 6,
                start_angle_deg: delta.y.atan2(delta.x).to_degrees(),
            };
            for h in pat.evaluate_holes() {
                out.push(build_circle_polyline(h.center_xy, h.diameter_mm));
            }
            out
        }
        SketchAuthorTool::PlaceText => Vec::new(),
    }
}
