use chip_cad::CadDocument;
use chip_cam::{
    MeasureSnapTarget, MeasurementResult, ReachAnalysisResult, ReachSeverity, RegionSet2D,
    ResolvedRotaryConfig, ResolvedStockGeometry, RotaryAxisKind, StockAxis, TabMarker3D,
};
use chip_math::{Aabb3, ArcPolyline2D, ArcSegment2D, DMat4, DVec2, DVec3, RigidFrame3};
use chip_sketch::{SketchDiagnosticKind, SketchEntity, SketchPickHit};

use super::{
    boundary_preview::BoundaryPreviewOverlay,
    gpu_types::{EdgeInstanceGpu, OverlayVertexGpu},
    highlight::{HighlightState, dvec3_f32},
};
use crate::camera::CadCamera;

pub(crate) fn build_edge_and_scene_instances(
    doc: Option<&CadDocument>,
    highlight: &HighlightState,
    camera: &CadCamera,
    overlay_tris: &mut Vec<OverlayVertexGpu>,
) -> Vec<EdgeInstanceGpu> {
    let mut out = Vec::new();
    let (hov_edges, sel_edges) = highlight.collect_edge_sets();

    // 1. B-Rep topological edges (hidden when solid machined stock replaces B-Rep unless ghost target is on)
    if (highlight.sim_stock_source.is_none() || highlight.ghost_part)
        && let Some(cad_doc) = doc
    {
        let has_cad_xf = highlight.has_active_cad_rotary_transform();
        let cad_xf = if has_cad_xf {
            highlight.cad_world_transform()
        } else {
            DMat4::IDENTITY
        };
        for poly in &cad_doc.render_mesh.edge_polylines {
            let (color, width_px) = if sel_edges.contains(&poly.edge_id) {
                ([1.0, 0.78, 0.18, 1.0], 3.4)
            } else if hov_edges.contains(&poly.edge_id) {
                ([0.28, 0.88, 1.0, 1.0], 3.1)
            } else {
                ([0.11, 0.13, 0.16, 0.90], 1.55)
            };

            for w in poly.points_mm.windows(2) {
                let (p0, p1) = if has_cad_xf {
                    (cad_xf.transform_point3(w[0]), cad_xf.transform_point3(w[1]))
                } else {
                    (w[0], w[1])
                };
                out.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(p0),
                    p1_mm: dvec3_f32(p1),
                    color,
                    width_px,
                });
            }
        }
    }

    // 1b. 2D Vector Sketch Layers (`chip-sketch`, rendered in both Pure-2D and Hybrid 3D+2D modes)
    if !highlight.sketch_visuals.is_empty() {
        emit_sketch_layer_visuals(highlight, camera, &mut out);
    }

    // 2. Stock translucent shell + wireframe in active Setup WCS (when not replaced by 3D machined stock)
    let stock_rot_wcs = highlight.stock_model_wcs_transform();
    if highlight.show_stock
        && highlight.sim_stock_source.is_none()
        && let Some(stock) = &highlight.stock
    {
        emit_stock_visuals(
            stock,
            &highlight.wcs_frame,
            stock_rot_wcs,
            highlight.theme.is_light(),
            overlay_tris,
            &mut out,
        );
    }

    // 2b. 4th-Axis Rotary Centerline, Chuck Collar, Direction Ring, & Clearance Cylinder Gizmo
    if let Some(rot) = &highlight.rotary {
        emit_rotary_gizmo_visuals(
            rot,
            &highlight.wcs_frame,
            highlight.theme.is_light(),
            overlay_tris,
            &mut out,
        );
    }

    // 2c. Crisp Silhouette / Feature-Crease Wireframe for 3D Workholding Fixtures
    if highlight.show_fixtures && !highlight.fixture_meshes.is_empty() {
        super::scene_builders::emit_fixture_wireframe_edges(highlight, &mut out);
    }

    // 3. Translucent Z-Height reference planes across Stock XY footprint (honoring rotary rotation)
    if !highlight.z_planes_wcs.is_empty() {
        let footprint = highlight
            .stock
            .map(|s| {
                let raw = s.aabb_wcs();
                if highlight.has_active_cad_rotary_transform() && raw.is_valid() {
                    let mut rot_aabb = Aabb3::EMPTY;
                    for x in [raw.min.x, raw.max.x] {
                        for y in [raw.min.y, raw.max.y] {
                            for z in [raw.min.z, raw.max.z] {
                                rot_aabb.extend_point(
                                    stock_rot_wcs.transform_point3(DVec3::new(x, y, z)),
                                );
                            }
                        }
                    }
                    rot_aabb
                } else {
                    raw
                }
            })
            .unwrap_or_else(|| {
                Aabb3::new(DVec3::new(-40.0, -30.0, 0.0), DVec3::new(40.0, 30.0, 10.0))
            });
        let pad = 3.0;
        let x0 = footprint.min.x - pad;
        let x1 = footprint.max.x + pad;
        let y0 = footprint.min.y - pad;
        let y1 = footprint.max.y + pad;

        for plane in &highlight.z_planes_wcs {
            let z = plane.z_wcs;
            let p00 = highlight
                .wcs_frame
                .local_to_world_point(DVec3::new(x0, y0, z));
            let p10 = highlight
                .wcs_frame
                .local_to_world_point(DVec3::new(x1, y0, z));
            let p11 = highlight
                .wcs_frame
                .local_to_world_point(DVec3::new(x1, y1, z));
            let p01 = highlight
                .wcs_frame
                .local_to_world_point(DVec3::new(x0, y1, z));
            push_quad_tris(overlay_tris, p00, p10, p11, p01, plane.fill_rgba);
            for (a, b) in [(p00, p10), (p10, p11), (p11, p01), (p01, p00)] {
                out.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(a),
                    p1_mm: dvec3_f32(b),
                    color: plane.border_rgba,
                    width_px: 1.8,
                });
            }
        }
    }

    // 3b. Live Cutting-Area / Machining-Boundary Preview (when OperationTab::Geometry is active)
    if let Some(preview) = &highlight.boundary_preview {
        emit_boundary_preview_visuals(
            preview,
            &highlight.wcs_frame,
            highlight.sim_rotary_deg,
            highlight.theme.is_light(),
            overlay_tris,
            &mut out,
        );
    }

    // 4. Active Setup WCS XYZ Origin Triad with 3D Arrowheads
    let triad_len = (camera.scene_radius_mm * 0.30).clamp(8.0, 48.0);
    let head = triad_len * 0.18;
    let origin_w = highlight.wcs_frame.origin_mm;
    let triad_axes = [
        (
            highlight.wcs_frame.x_axis,
            highlight.wcs_frame.y_axis,
            [0.96, 0.30, 0.30, 1.0],
        ),
        (
            highlight.wcs_frame.y_axis,
            highlight.wcs_frame.x_axis,
            [0.28, 0.90, 0.42, 1.0],
        ),
        (
            highlight.wcs_frame.z_axis,
            highlight.wcs_frame.x_axis,
            [0.34, 0.62, 1.0, 1.0],
        ),
    ];
    for (dir, side, color) in triad_axes {
        let tip = origin_w + dir * triad_len;
        out.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(origin_w),
            p1_mm: dvec3_f32(tip),
            color,
            width_px: 2.9,
        });
        let wing_a = tip - dir * head + side * (0.45 * head);
        let wing_b = tip - dir * head - side * (0.45 * head);
        out.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(tip),
            p1_mm: dvec3_f32(wing_a),
            color,
            width_px: 2.4,
        });
        out.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(tip),
            p1_mm: dvec3_f32(wing_b),
            color,
            width_px: 2.4,
        });
    }

    // 5. 3D Chain Travel Direction & Offset-Side Indicators
    let arrow_scale = (camera.scene_radius_mm * 0.14).clamp(4.0, 18.0);
    let has_arrow_xf = highlight.has_active_cad_rotary_transform();
    let arrow_xf = if has_arrow_xf {
        highlight.cad_world_transform()
    } else {
        DMat4::IDENTITY
    };
    for arrow in &highlight.chain_arrows {
        let (anchor, tangent, side_normal) = if has_arrow_xf {
            (
                arrow_xf.transform_point3(arrow.anchor_world_mm),
                arrow_xf
                    .transform_vector3(arrow.tangent_world)
                    .normalize_or_zero(),
                arrow_xf
                    .transform_vector3(arrow.side_normal_world)
                    .normalize_or_zero(),
            )
        } else {
            (
                arrow.anchor_world_mm,
                arrow.tangent_world,
                arrow.side_normal_world,
            )
        };
        let p0 = anchor - tangent * (0.4 * arrow_scale);
        let p_tip = anchor + tangent * (0.6 * arrow_scale);
        let dir_col = [0.18, 0.96, 0.62, 1.0];
        out.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(p0),
            p1_mm: dvec3_f32(p_tip),
            color: dir_col,
            width_px: 3.2,
        });
        let w_len = 0.35 * arrow_scale;
        for sign in [-1.0, 1.0] {
            let wing = p_tip - tangent * w_len + side_normal * (sign * 0.55 * w_len);
            out.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(p_tip),
                p1_mm: dvec3_f32(wing),
                color: dir_col,
                width_px: 2.6,
            });
        }
        // Perpendicular offset-side tick (amber/magenta) showing Left vs Right cutter compensation side
        let side_tip = anchor + side_normal * (0.55 * arrow_scale);
        out.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(anchor),
            p1_mm: dvec3_f32(side_tip),
            color: [1.0, 0.45, 0.78, 1.0],
            width_px: 2.6,
        });
    }

    // 6. 3D Holding Tab Bridge Markers (from generated toolpaths + live interactive cursor preview)
    if highlight.show_toolpaths {
        for entry in &highlight.toolpaths {
            let alpha_scale = if entry.is_focused { 1.0 } else { 0.45 };
            for tab in &entry.toolpath.tab_markers_wcs {
                emit_tab_marker_visual(
                    tab,
                    &highlight.wcs_frame,
                    false,
                    alpha_scale,
                    overlay_tris,
                    &mut out,
                );
            }
        }
    }
    if let Some(preview_tab) = &highlight.preview_tab_marker_wcs {
        emit_tab_marker_visual(
            preview_tab,
            &highlight.wcs_frame,
            true,
            1.0,
            overlay_tris,
            &mut out,
        );
    }

    // 7. Minimum-Radius & Flute/Stickout Reach Heatmap Overlay (Amber & Crimson)
    if highlight.show_reach_heatmap
        && let Some(reach) = &highlight.reach_analysis
    {
        emit_reach_heatmap_visuals(
            reach,
            highlight.has_active_cad_rotary_transform(),
            if highlight.has_active_cad_rotary_transform() {
                highlight.cad_world_transform()
            } else {
                DMat4::IDENTITY
            },
            highlight.theme.is_light(),
            overlay_tris,
            &mut out,
        );
    }

    // 8. Interactive Click-to-Measure Snap Anchors, Dimension Leader Line, Ticks, & dX/dY/dZ Legs
    if let Some(measure) = &highlight.measure_state {
        emit_measure_visuals(
            measure,
            &highlight.wcs_frame,
            camera,
            highlight.theme.is_light(),
            &mut out,
        );
    }

    out
}

fn emit_reach_heatmap_visuals(
    reach: &ReachAnalysisResult,
    has_cad_xf: bool,
    cad_xf: DMat4,
    is_light: bool,
    overlay_tris: &mut Vec<OverlayVertexGpu>,
    out: &mut Vec<EdgeInstanceGpu>,
) {
    // High-contrast Amber (Warning) & Crimson (Critical) palettes tuned for both Studio Dark & Daylight Light
    let face_rgba = |sev: ReachSeverity| -> [f32; 4] {
        match (sev, is_light) {
            (ReachSeverity::Warning, false) => [0.98, 0.68, 0.10, 0.44],
            (ReachSeverity::Warning, true) => [0.90, 0.48, 0.04, 0.48],
            (ReachSeverity::Critical, false) => [0.96, 0.16, 0.22, 0.50],
            (ReachSeverity::Critical, true) => [0.86, 0.08, 0.14, 0.54],
        }
    };
    let edge_rgba = |sev: ReachSeverity| -> [f32; 4] {
        match (sev, is_light) {
            (ReachSeverity::Warning, false) => [1.0, 0.78, 0.18, 0.98],
            (ReachSeverity::Warning, true) => [0.84, 0.44, 0.02, 0.98],
            (ReachSeverity::Critical, false) => [1.0, 0.24, 0.30, 0.98],
            (ReachSeverity::Critical, true) => [0.84, 0.06, 0.14, 0.98],
        }
    };

    let nudge_mm = 0.03;
    for hf in &reach.highlight_faces {
        let col = face_rgba(hf.severity);
        for &(verts, normal) in &hf.triangles_world {
            let n = if has_cad_xf {
                cad_xf.transform_vector3(normal).normalize_or_zero()
            } else {
                normal
            };
            let offset = n * nudge_mm;
            for v in verts {
                let p = if has_cad_xf {
                    cad_xf.transform_point3(v) + offset
                } else {
                    v + offset
                };
                overlay_tris.push(OverlayVertexGpu {
                    pos_mm: dvec3_f32(p),
                    color: col,
                });
            }
        }
    }

    for he in &reach.highlight_edges {
        let col = edge_rgba(he.severity);
        for &(a, b) in &he.segments_world {
            let (p0, p1) = if has_cad_xf {
                (cad_xf.transform_point3(a), cad_xf.transform_point3(b))
            } else {
                (a, b)
            };
            out.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(p0),
                p1_mm: dvec3_f32(p1),
                color: col,
                width_px: 3.4,
            });
        }
    }
}

fn emit_measure_visuals(
    measure: &super::highlight::MeasureToolState,
    wcs_frame: &RigidFrame3,
    camera: &CadCamera,
    is_light: bool,
    out: &mut Vec<EdgeInstanceGpu>,
) {
    let scale = (camera.scene_radius_mm * 0.035).clamp(0.8, 6.0);
    let hover_col = if is_light {
        [0.02, 0.56, 0.78, 0.95]
    } else {
        [0.24, 0.92, 1.0, 0.98]
    };
    let anchor_col = if is_light {
        [0.86, 0.42, 0.02, 1.0]
    } else {
        [1.0, 0.82, 0.22, 1.0]
    };
    let leader_col = if is_light {
        [0.04, 0.44, 0.86, 1.0]
    } else {
        [0.32, 0.94, 1.0, 1.0]
    };

    if let Some(hov) = &measure.hovered_snap {
        emit_measure_snap_marker(hov, wcs_frame, scale, hover_col, 2.4, out);
    }
    if let Some(a1) = &measure.anchor1 {
        emit_measure_snap_marker(a1, wcs_frame, scale * 1.15, anchor_col, 3.0, out);
    }
    if let Some(a2) = &measure.anchor2 {
        emit_measure_snap_marker(a2, wcs_frame, scale * 1.15, anchor_col, 3.0, out);
    }

    if let Some(res) = &measure.result
        && let Some((p1_wcs, p2_wcs)) = res.leader_endpoints_wcs()
    {
        let w1 = wcs_frame.local_to_world_point(p1_wcs);
        let w2 = wcs_frame.local_to_world_point(p2_wcs);
        let diff = w2 - w1;
        let dist = diff.length();

        if dist > 1e-6 {
            // Main 3D dimension leader line
            out.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(w1),
                p1_mm: dvec3_f32(w2),
                color: leader_col,
                width_px: 3.2,
            });

            // Perpendicular end ticks at both endpoints
            let dir = diff / dist;
            let ref_axis = if dir.dot(wcs_frame.z_axis).abs() < 0.9 {
                wcs_frame.z_axis
            } else {
                wcs_frame.x_axis
            };
            let tick_dir = dir.cross(ref_axis).normalize_or_zero();
            let tick_half = (scale * 0.85).min(dist * 0.35);
            if tick_dir.length_squared() > 0.5 && tick_half > 1e-4 {
                for end_w in [w1, w2] {
                    out.push(EdgeInstanceGpu {
                        p0_mm: dvec3_f32(end_w - tick_dir * tick_half),
                        p1_mm: dvec3_f32(end_w + tick_dir * tick_half),
                        color: leader_col,
                        width_px: 2.6,
                    });
                }
            }

            // Orthogonal dX / dY / dZ projection legs in Setup WCS for point/cylinder/plane pairs
            let show_legs = matches!(
                res,
                MeasurementResult::PointToPoint { .. }
                    | MeasurementResult::CylinderToCylinder { .. }
                    | MeasurementResult::PointToPlane { .. }
            );
            if show_legs {
                let d = p2_wcs - p1_wcs;
                let corner_x = DVec3::new(p2_wcs.x, p1_wcs.y, p1_wcs.z);
                let corner_xy = DVec3::new(p2_wcs.x, p2_wcs.y, p1_wcs.z);
                let wx = wcs_frame.local_to_world_point(corner_x);
                let wxy = wcs_frame.local_to_world_point(corner_xy);

                let (col_x, col_y, col_z) = if is_light {
                    (
                        [0.82, 0.18, 0.18, 0.82],
                        [0.12, 0.64, 0.24, 0.82],
                        [0.14, 0.42, 0.88, 0.82],
                    )
                } else {
                    (
                        [0.98, 0.36, 0.36, 0.82],
                        [0.32, 0.92, 0.46, 0.82],
                        [0.38, 0.68, 1.0, 0.82],
                    )
                };

                if d.x.abs() > 1e-4 {
                    out.push(EdgeInstanceGpu {
                        p0_mm: dvec3_f32(w1),
                        p1_mm: dvec3_f32(wx),
                        color: col_x,
                        width_px: 1.9,
                    });
                }
                if d.y.abs() > 1e-4 {
                    out.push(EdgeInstanceGpu {
                        p0_mm: dvec3_f32(wx),
                        p1_mm: dvec3_f32(wxy),
                        color: col_y,
                        width_px: 1.9,
                    });
                }
                if d.z.abs() > 1e-4 {
                    out.push(EdgeInstanceGpu {
                        p0_mm: dvec3_f32(wxy),
                        p1_mm: dvec3_f32(w2),
                        color: col_z,
                        width_px: 1.9,
                    });
                }
            }
        }
    }
}

fn emit_measure_snap_marker(
    target: &MeasureSnapTarget,
    wcs_frame: &RigidFrame3,
    scale: f64,
    color: [f32; 4],
    width_px: f32,
    out: &mut Vec<EdgeInstanceGpu>,
) {
    let anchor_w = wcs_frame.local_to_world_point(target.anchor_wcs());

    // 3D crosshair at canonical anchor
    for axis in [wcs_frame.x_axis, wcs_frame.y_axis, wcs_frame.z_axis] {
        out.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(anchor_w - axis * scale),
            p1_mm: dvec3_f32(anchor_w + axis * scale),
            color,
            width_px,
        });
    }

    match target {
        MeasureSnapTarget::BRepCylindricalFace {
            axis_origin_wcs,
            axis_dir_wcs,
            diameter_mm,
            ..
        } => {
            let r = (0.5 * *diameter_mm).max(0.2);
            let axis_w = wcs_frame
                .local_to_world_dir(*axis_dir_wcs)
                .normalize_or_zero();
            emit_circle_ring_edges(
                wcs_frame.local_to_world_point(*axis_origin_wcs),
                if axis_w.length_squared() > 0.5 {
                    axis_w
                } else {
                    wcs_frame.z_axis
                },
                r,
                24,
                color,
                width_px,
                out,
            );
        }
        MeasureSnapTarget::SketchCircle {
            center_wcs,
            diameter_mm,
            ..
        } => {
            let r = (0.5 * *diameter_mm).max(0.2);
            emit_circle_ring_edges(
                wcs_frame.local_to_world_point(*center_wcs),
                wcs_frame.z_axis,
                r,
                24,
                color,
                width_px,
                out,
            );
        }
        MeasureSnapTarget::BRepEdge {
            arc_radius_mm: Some(r),
            arc_center_wcs: Some(c),
            ..
        } => {
            emit_circle_ring_edges(
                wcs_frame.local_to_world_point(*c),
                wcs_frame.z_axis,
                (*r).max(0.2),
                20,
                color,
                width_px * 0.85,
                out,
            );
        }
        MeasureSnapTarget::BRepPlanarFace {
            point_wcs,
            normal_wcs,
            ..
        } => {
            let p_w = wcs_frame.local_to_world_point(*point_wcs);
            let n_w = wcs_frame
                .local_to_world_dir(*normal_wcs)
                .normalize_or_zero();
            if n_w.length_squared() > 0.5 {
                // Small square frame on plane + outward normal indicator
                let ref_v = if n_w.x.abs() < 0.9 {
                    DVec3::X
                } else {
                    DVec3::Y
                };
                let u = n_w.cross(ref_v).normalize_or_zero() * (scale * 1.4);
                let v = n_w.cross(u).normalize_or_zero() * (scale * 1.4);
                let c0 = p_w - u - v;
                let c1 = p_w + u - v;
                let c2 = p_w + u + v;
                let c3 = p_w - u + v;
                for (a, b) in [(c0, c1), (c1, c2), (c2, c3), (c3, c0)] {
                    out.push(EdgeInstanceGpu {
                        p0_mm: dvec3_f32(a),
                        p1_mm: dvec3_f32(b),
                        color,
                        width_px: width_px * 0.85,
                    });
                }
                out.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(p_w),
                    p1_mm: dvec3_f32(p_w + n_w * (scale * 2.0)),
                    color,
                    width_px,
                });
            }
        }
        _ => {}
    }
}

fn emit_circle_ring_edges(
    center_world: DVec3,
    normal_world: DVec3,
    radius_mm: f64,
    steps: usize,
    color: [f32; 4],
    width_px: f32,
    out: &mut Vec<EdgeInstanceGpu>,
) {
    let ref_v = if normal_world.x.abs() < 0.9 {
        DVec3::X
    } else {
        DVec3::Y
    };
    let u = normal_world.cross(ref_v).normalize_or_zero();
    let v = normal_world.cross(u).normalize_or_zero();
    let n = steps.max(12);
    let mut prev = center_world + u * radius_mm;
    for i in 1..=n {
        let a = (i as f64 / n as f64) * std::f64::consts::TAU;
        let cur = center_world + (u * a.cos() + v * a.sin()) * radius_mm;
        out.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(prev),
            p1_mm: dvec3_f32(cur),
            color,
            width_px,
        });
        prev = cur;
    }
}

fn emit_sketch_layer_visuals(
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

fn emit_arc_polyline_edges(
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

fn emit_tab_marker_visual(
    tab: &TabMarker3D,
    wcs_frame: &RigidFrame3,
    is_hover_preview: bool,
    alpha_scale: f32,
    overlay_tris: &mut Vec<OverlayVertexGpu>,
    edges: &mut Vec<EdgeInstanceGpu>,
) {
    let tan = DVec2::new(tab.tangent_xy.x, tab.tangent_xy.y).normalize_or_zero();
    let t2 = if tan.length_squared() < 0.2 {
        DVec2::X
    } else {
        tan
    };
    let n2 = DVec2::new(-t2.y, t2.x);
    let half_w = 0.5 * tab.width_mm.max(0.8);
    let half_t = 1.1; // visual bridge thickness across contour edge
    let z0 = tab.center_wcs.z;
    let z1 = z0 + tab.height_mm.max(0.3);

    let fill_col = if is_hover_preview {
        [0.24, 0.98, 0.72, 0.36 * alpha_scale]
    } else {
        [0.98, 0.76, 0.22, 0.28 * alpha_scale]
    };
    let wire_col = if is_hover_preview {
        [0.24, 0.98, 0.72, 0.98 * alpha_scale]
    } else {
        [1.0, 0.82, 0.26, 0.95 * alpha_scale]
    };

    let c2 = DVec2::new(tab.center_wcs.x, tab.center_wcs.y);
    let to_w = |xy: DVec2, z: f64| wcs_frame.local_to_world_point(DVec3::new(xy.x, xy.y, z));

    if tab.is_triangular {
        // Triangular prism bridge peaking at `(c2, z1)`
        let bl0 = to_w(c2 - t2 * half_w - n2 * half_t, z0);
        let bl1 = to_w(c2 - t2 * half_w + n2 * half_t, z0);
        let br0 = to_w(c2 + t2 * half_w - n2 * half_t, z0);
        let br1 = to_w(c2 + t2 * half_w + n2 * half_t, z0);
        let ap0 = to_w(c2 - n2 * half_t, z1);
        let ap1 = to_w(c2 + n2 * half_t, z1);

        push_quad_tris(overlay_tris, bl0, ap0, ap1, bl1, fill_col);
        push_quad_tris(overlay_tris, ap0, br0, br1, ap1, fill_col);
        for (a, b) in [
            (bl0, ap0),
            (ap0, br0),
            (bl1, ap1),
            (ap1, br1),
            (ap0, ap1),
            (bl0, bl1),
            (br0, br1),
        ] {
            edges.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(a),
                p1_mm: dvec3_f32(b),
                color: wire_col,
                width_px: 2.3,
            });
        }
    } else {
        // Rectangular bridge box
        let b00 = to_w(c2 - t2 * half_w - n2 * half_t, z0);
        let b10 = to_w(c2 + t2 * half_w - n2 * half_t, z0);
        let b11 = to_w(c2 + t2 * half_w + n2 * half_t, z0);
        let b01 = to_w(c2 - t2 * half_w + n2 * half_t, z0);
        let t00 = to_w(c2 - t2 * half_w - n2 * half_t, z1);
        let t10 = to_w(c2 + t2 * half_w - n2 * half_t, z1);
        let t11 = to_w(c2 + t2 * half_w + n2 * half_t, z1);
        let t01 = to_w(c2 - t2 * half_w + n2 * half_t, z1);

        push_quad_tris(overlay_tris, t00, t10, t11, t01, fill_col);
        for (a, b) in [
            (t00, t10),
            (t10, t11),
            (t11, t01),
            (t01, t00),
            (b00, t00),
            (b10, t10),
            (b11, t11),
            (b01, t01),
        ] {
            edges.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(a),
                p1_mm: dvec3_f32(b),
                color: wire_col,
                width_px: 2.3,
            });
        }
    }
}

/// Emits 3D 4th-axis rotary kinematics gizmo overlays in Setup WCS:
/// 1. Dashed rotary centerline extending past stock & chuck,
/// 2. Rotary chuck collar + 3-jaw markers at `[chuck_u_min, chuck_u_max]`,
/// 3. Circular rotation-direction arrow ring at the free end showing `+θ` polarity,
/// 4. Subtle rotary clearance cylinder wireframe at `clearance_radius_mm`.
fn emit_rotary_gizmo_visuals(
    rot: &ResolvedRotaryConfig,
    frame: &RigidFrame3,
    is_light: bool,
    overlay_tris: &mut Vec<OverlayVertexGpu>,
    edges: &mut Vec<EdgeInstanceGpu>,
) {
    let to_w = |p_wcs: DVec3| frame.local_to_world_point(p_wcs);
    let u_vec = rot.axis.axis_unit();

    let u_lo = rot.stock_u_min.min(rot.chuck_u_min) - 14.0;
    let u_hi = rot.stock_u_max.max(rot.chuck_u_max) + 14.0;

    let axis_col = if is_light {
        [0.05, 0.56, 0.72, 0.95]
    } else {
        [0.24, 0.88, 1.0, 0.96]
    };
    let clr_col = if is_light {
        [0.14, 0.62, 0.78, 0.42]
    } else {
        [0.26, 0.82, 0.98, 0.36]
    };
    let chuck_edge_col = if is_light {
        [0.28, 0.33, 0.40, 0.92]
    } else {
        [0.64, 0.70, 0.78, 0.88]
    };
    let chuck_fill_col = if is_light {
        [0.34, 0.38, 0.46, 0.22]
    } else {
        [0.44, 0.50, 0.58, 0.24]
    };
    let arrow_col = if is_light {
        [0.86, 0.44, 0.04, 0.96]
    } else {
        [1.0, 0.68, 0.16, 0.96]
    };

    // 1. Dashed rotary centerline axis
    let dash = 4.5_f64;
    let gap = 2.5_f64;
    let mut u_cur = u_lo;
    while u_cur < u_hi {
        let u_next = (u_cur + dash).min(u_hi);
        let p0 = to_w(rot.cylindrical_to_wcs(u_cur, 0.0, 0.0));
        let p1 = to_w(rot.cylindrical_to_wcs(u_next, 0.0, 0.0));
        edges.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(p0),
            p1_mm: dvec3_f32(p1),
            color: axis_col,
            width_px: 2.2,
        });
        u_cur += dash + gap;
    }

    // 2. Rotary Chuck collar cylinder + 3-jaw radial markers
    let segs = 36usize;
    let chuck_len = rot.chuck_u_max - rot.chuck_u_min;
    if chuck_len > 0.5 && rot.chuck_radius_mm > 1.0 {
        let r_ch = rot.chuck_radius_mm;
        let mut ring0 = Vec::with_capacity(segs);
        let mut ring1 = Vec::with_capacity(segs);
        for k in 0..segs {
            let th = (k as f64 / segs as f64) * std::f64::consts::TAU;
            ring0.push(to_w(rot.cylindrical_to_wcs(rot.chuck_u_min, th, r_ch)));
            ring1.push(to_w(rot.cylindrical_to_wcs(rot.chuck_u_max, th, r_ch)));
        }
        for k in 0..segs {
            let kn = (k + 1) % segs;
            push_quad_tris(
                overlay_tris,
                ring0[k],
                ring1[k],
                ring1[kn],
                ring0[kn],
                chuck_fill_col,
            );
            edges.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(ring0[k]),
                p1_mm: dvec3_f32(ring0[kn]),
                color: chuck_edge_col,
                width_px: 1.9,
            });
            edges.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(ring1[k]),
                p1_mm: dvec3_f32(ring1[kn]),
                color: chuck_edge_col,
                width_px: 1.9,
            });
            if k % 6 == 0 {
                edges.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(ring0[k]),
                    p1_mm: dvec3_f32(ring1[k]),
                    color: chuck_edge_col,
                    width_px: 1.3,
                });
            }
        }

        // 3 radial chuck jaw bars gripping from `stock_outer_radius` to `chuck_radius` at chuck face
        let jaw_u = if rot.chuck_end_positive {
            rot.chuck_u_min
        } else {
            rot.chuck_u_max
        };
        let r_inner = rot.stock_outer_radius_mm.min(r_ch * 0.95);
        for j in 0..3 {
            let th = (j as f64 / 3.0) * std::f64::consts::TAU;
            let p_in = to_w(rot.cylindrical_to_wcs(jaw_u, th, r_inner));
            let p_out = to_w(rot.cylindrical_to_wcs(jaw_u, th, r_ch));
            edges.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(p_in),
                p1_mm: dvec3_f32(p_out),
                color: chuck_edge_col,
                width_px: 3.2,
            });
        }
    }

    // 3. Rotary Clearance Cylinder wireframe (2 end rings + 4 axial generator lines)
    if rot.clearance_radius_mm > 1.0 && rot.stock_u_max > rot.stock_u_min + 0.5 {
        let r_clr = rot.clearance_radius_mm;
        for u_ring in [rot.stock_u_min, rot.stock_u_max] {
            for k in 0..segs {
                let th0 = (k as f64 / segs as f64) * std::f64::consts::TAU;
                let th1 = ((k + 1) as f64 / segs as f64) * std::f64::consts::TAU;
                let p0 = to_w(rot.cylindrical_to_wcs(u_ring, th0, r_clr));
                let p1 = to_w(rot.cylindrical_to_wcs(u_ring, th1, r_clr));
                edges.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(p0),
                    p1_mm: dvec3_f32(p1),
                    color: clr_col,
                    width_px: 1.35,
                });
            }
        }
        for q in 0..4 {
            let th = (q as f64 * 0.25) * std::f64::consts::TAU;
            let p0 = to_w(rot.cylindrical_to_wcs(rot.stock_u_min, th, r_clr));
            let p1 = to_w(rot.cylindrical_to_wcs(rot.stock_u_max, th, r_clr));
            edges.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(p0),
                p1_mm: dvec3_f32(p1),
                color: clr_col,
                width_px: 1.2,
            });
        }
    }

    // 4. Positive-rotation polarity arrow arc (270° sweep around the free end of the bar)
    let arrow_u = if rot.chuck_end_positive {
        rot.stock_u_min - 6.0
    } else {
        rot.stock_u_max + 6.0
    };
    let r_arc = (rot.stock_outer_radius_mm * 0.72).clamp(8.0, 28.0);
    let arc_steps = 24usize;
    let sweep_rad = 260.0_f64.to_radians();
    for k in 0..arc_steps {
        let th0 = (k as f64 / arc_steps as f64) * sweep_rad;
        let th1 = ((k + 1) as f64 / arc_steps as f64) * sweep_rad;
        let p0 = to_w(rot.cylindrical_to_wcs(arrow_u, th0, r_arc));
        let p1 = to_w(rot.cylindrical_to_wcs(arrow_u, th1, r_arc));
        edges.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(p0),
            p1_mm: dvec3_f32(p1),
            color: arrow_col,
            width_px: 2.6,
        });
    }
    // Arrowhead at end of positive-rotation arc
    let tip_wcs = rot.cylindrical_to_wcs(arrow_u, sweep_rad, r_arc);
    let prev_wcs = rot.cylindrical_to_wcs(arrow_u, sweep_rad - 0.15, r_arc);
    let tangent = (tip_wcs - prev_wcs).normalize_or_zero();
    let radial = (tip_wcs - rot.cylindrical_to_wcs(arrow_u, 0.0, 0.0)).normalize_or_zero();
    let head_len = (r_arc * 0.25).clamp(2.2, 5.0);
    let tip_w = to_w(tip_wcs);
    let wing_1 = to_w(tip_wcs - tangent * head_len + radial * (0.45 * head_len));
    let wing_2 = to_w(tip_wcs - tangent * head_len - radial * (0.45 * head_len));
    let _ = u_vec;
    edges.push(EdgeInstanceGpu {
        p0_mm: dvec3_f32(tip_w),
        p1_mm: dvec3_f32(wing_1),
        color: arrow_col,
        width_px: 2.6,
    });
    edges.push(EdgeInstanceGpu {
        p0_mm: dvec3_f32(tip_w),
        p1_mm: dvec3_f32(wing_2),
        color: arrow_col,
        width_px: 2.6,
    });
}

fn push_quad_tris(
    tris: &mut Vec<OverlayVertexGpu>,
    p0: DVec3,
    p1: DVec3,
    p2: DVec3,
    p3: DVec3,
    color: [f32; 4],
) {
    for p in [p0, p1, p2, p0, p2, p3] {
        tris.push(OverlayVertexGpu {
            pos_mm: dvec3_f32(p),
            color,
        });
    }
}

pub(crate) fn emit_stock_visuals(
    stock: &ResolvedStockGeometry,
    wcs_frame: &RigidFrame3,
    stock_rot_wcs: DMat4,
    is_light: bool,
    overlay_tris: &mut Vec<OverlayVertexGpu>,
    edges: &mut Vec<EdgeInstanceGpu>,
) {
    let fill_col = if is_light {
        [0.84, 0.58, 0.10, 0.095]
    } else {
        [0.92, 0.74, 0.28, 0.085]
    };
    let wire_col = if is_light {
        [0.72, 0.46, 0.06, 0.78]
    } else {
        [0.95, 0.78, 0.32, 0.62]
    };

    let map_wcs = |p: DVec3| wcs_frame.local_to_world_point(stock_rot_wcs.transform_point3(p));

    match *stock {
        ResolvedStockGeometry::Box { aabb_wcs } => {
            if !aabb_wcs.is_valid() {
                return;
            }
            let min = aabb_wcs.min;
            let max = aabb_wcs.max;
            let corners_wcs = [
                DVec3::new(min.x, min.y, min.z),
                DVec3::new(max.x, min.y, min.z),
                DVec3::new(max.x, max.y, min.z),
                DVec3::new(min.x, max.y, min.z),
                DVec3::new(min.x, min.y, max.z),
                DVec3::new(max.x, min.y, max.z),
                DVec3::new(max.x, max.y, max.z),
                DVec3::new(min.x, max.y, max.z),
            ];
            let c: [DVec3; 8] = corners_wcs.map(map_wcs);

            let quads = [
                (0, 1, 2, 3),
                (4, 5, 6, 7),
                (0, 1, 5, 4),
                (1, 2, 6, 5),
                (2, 3, 7, 6),
                (3, 0, 4, 7),
            ];
            for (i0, i1, i2, i3) in quads {
                push_quad_tris(overlay_tris, c[i0], c[i1], c[i2], c[i3], fill_col);
            }

            let box_edges = [
                (0, 1),
                (1, 2),
                (2, 3),
                (3, 0),
                (4, 5),
                (5, 6),
                (6, 7),
                (7, 4),
                (0, 4),
                (1, 5),
                (2, 6),
                (3, 7),
            ];
            for (a, b) in box_edges {
                edges.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(c[a]),
                    p1_mm: dvec3_f32(c[b]),
                    color: wire_col,
                    width_px: 1.5,
                });
            }
        }
        ResolvedStockGeometry::Cylinder {
            axis,
            center_wcs,
            radius_mm,
            min_axial_mm,
            max_axial_mm,
            ..
        } => {
            const N: usize = 36;
            let map_cyl = |theta: f64, ax: f64| -> DVec3 {
                let (s, c) = theta.sin_cos();
                let u = radius_mm * c;
                let v = radius_mm * s;
                let p_wcs = match axis {
                    StockAxis::Z => DVec3::new(center_wcs.x + u, center_wcs.y + v, ax),
                    StockAxis::X => DVec3::new(ax, center_wcs.y + u, center_wcs.z + v),
                    StockAxis::Y => DVec3::new(center_wcs.x + u, ax, center_wcs.z + v),
                };
                map_wcs(p_wcs)
            };

            for i in 0..N {
                let a0 = (i as f64) * std::f64::consts::TAU / (N as f64);
                let a1 = ((i + 1) as f64) * std::f64::consts::TAU / (N as f64);
                let b0 = map_cyl(a0, min_axial_mm);
                let b1 = map_cyl(a1, min_axial_mm);
                let t0 = map_cyl(a0, max_axial_mm);
                let t1 = map_cyl(a1, max_axial_mm);

                push_quad_tris(overlay_tris, b0, b1, t1, t0, fill_col);
                edges.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(b0),
                    p1_mm: dvec3_f32(b1),
                    color: wire_col,
                    width_px: 1.5,
                });
                edges.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(t0),
                    p1_mm: dvec3_f32(t1),
                    color: wire_col,
                    width_px: 1.5,
                });
                if i % 9 == 0 {
                    edges.push(EdgeInstanceGpu {
                        p0_mm: dvec3_f32(b0),
                        p1_mm: dvec3_f32(t0),
                        color: wire_col,
                        width_px: 1.3,
                    });
                }
            }
        }
    }
}

/// Emits live 3D viewport cutting-area / machining-boundary preview overlays when
/// `OperationTab::Geometry` is active (`08-ui-viewport.md`).
fn emit_boundary_preview_visuals(
    preview: &BoundaryPreviewOverlay,
    wcs_frame: &RigidFrame3,
    sim_rotary_deg: f64,
    is_light: bool,
    overlay_tris: &mut Vec<OverlayVertexGpu>,
    edges: &mut Vec<EdgeInstanceGpu>,
) {
    match preview {
        BoundaryPreviewOverlay::Planar2D {
            nominal_region,
            effective_region,
            show_effective,
            avoid_nominal_region,
            avoid_effective_region,
            top_z_wcs,
            bottom_z_wcs,
            ..
        } => {
            let nominal_col = if is_light {
                [0.02, 0.48, 0.80, 0.98]
            } else {
                [0.18, 0.90, 1.0, 0.98]
            };
            let nominal_bottom_col = if is_light {
                [0.02, 0.48, 0.80, 0.36]
            } else {
                [0.18, 0.90, 1.0, 0.36]
            };
            let drop_col = if is_light {
                [0.04, 0.46, 0.76, 0.30]
            } else {
                [0.20, 0.84, 0.96, 0.30]
            };
            let effective_col = if is_light {
                [0.86, 0.40, 0.02, 0.98]
            } else {
                [1.0, 0.74, 0.18, 0.98]
            };

            let z_top = *top_z_wcs + 0.08;
            let z_bot = *bottom_z_wcs;
            let has_depth = (*top_z_wcs - *bottom_z_wcs).abs() > 0.25;

            if !nominal_region.is_empty() {
                emit_region_set_loops(
                    nominal_region,
                    wcs_frame,
                    z_top,
                    nominal_col,
                    3.0,
                    false,
                    edges,
                );
                if has_depth {
                    emit_region_set_loops(
                        nominal_region,
                        wcs_frame,
                        z_bot,
                        nominal_bottom_col,
                        1.5,
                        false,
                        edges,
                    );
                    emit_region_vertical_ticks(
                        nominal_region,
                        wcs_frame,
                        z_top,
                        z_bot,
                        drop_col,
                        1.3,
                        edges,
                    );
                }
            }
            if *show_effective && !effective_region.is_empty() {
                emit_region_set_loops(
                    effective_region,
                    wcs_frame,
                    *top_z_wcs + 0.14,
                    effective_col,
                    2.6,
                    true,
                    edges,
                );
            }
            emit_avoid_region_2d_visuals(
                avoid_nominal_region,
                avoid_effective_region,
                wcs_frame,
                *top_z_wcs,
                *bottom_z_wcs,
                is_light,
                overlay_tris,
                edges,
            );
        }
        BoundaryPreviewOverlay::StockOuterEnvelope {
            stock_region,
            avoid_nominal_region,
            avoid_effective_region,
            top_z_wcs,
            bottom_z_wcs,
            ..
        } => {
            let top_col = if is_light {
                [0.04, 0.56, 0.48, 0.98]
            } else {
                [0.20, 0.94, 0.78, 0.98]
            };
            let bot_col = if is_light {
                [0.04, 0.56, 0.48, 0.42]
            } else {
                [0.20, 0.94, 0.78, 0.42]
            };
            let z_top = *top_z_wcs + 0.08;
            emit_region_set_loops(stock_region, wcs_frame, z_top, top_col, 2.8, false, edges);
            if (*top_z_wcs - *bottom_z_wcs).abs() > 0.25 {
                emit_region_set_loops(
                    stock_region,
                    wcs_frame,
                    *bottom_z_wcs,
                    bot_col,
                    1.6,
                    true,
                    edges,
                );
                emit_region_vertical_ticks(
                    stock_region,
                    wcs_frame,
                    z_top,
                    *bottom_z_wcs,
                    bot_col,
                    1.3,
                    edges,
                );
            }
            emit_avoid_region_2d_visuals(
                avoid_nominal_region,
                avoid_effective_region,
                wcs_frame,
                *top_z_wcs,
                *bottom_z_wcs,
                is_light,
                overlay_tris,
                edges,
            );
        }
        BoundaryPreviewOverlay::RotarySector {
            rotary,
            window,
            avoid_axial_bands,
            ..
        } => {
            emit_rotary_sector_preview(
                rotary,
                window,
                avoid_axial_bands,
                wcs_frame,
                sim_rotary_deg,
                is_light,
                overlay_tris,
                edges,
            );
        }
    }
}

/// Emits crisp coral/red 2.5D & 3-axis Avoid / Keep-Out overlays:
/// translucent top-plane fill + solid nominal outline + subtle vertical prism ticks +
/// dashed outer tool-center clearance ring (`avoid_effective_region`).
#[allow(clippy::too_many_arguments)]
fn emit_avoid_region_2d_visuals(
    avoid_nominal_region: &RegionSet2D,
    avoid_effective_region: &RegionSet2D,
    wcs_frame: &RigidFrame3,
    top_z_wcs: f64,
    bottom_z_wcs: f64,
    is_light: bool,
    overlay_tris: &mut Vec<OverlayVertexGpu>,
    edges: &mut Vec<EdgeInstanceGpu>,
) {
    if avoid_nominal_region.is_empty() && avoid_effective_region.is_empty() {
        return;
    }

    let nominal_col = if is_light {
        [0.86, 0.14, 0.18, 0.98]
    } else {
        [1.0, 0.32, 0.34, 0.98]
    };
    let nominal_bot_col = if is_light {
        [0.86, 0.14, 0.18, 0.38]
    } else {
        [1.0, 0.32, 0.34, 0.38]
    };
    let fill_col = if is_light {
        [0.90, 0.16, 0.20, 0.20]
    } else {
        [1.0, 0.24, 0.28, 0.24]
    };
    let effective_col = if is_light {
        [0.78, 0.22, 0.10, 0.90]
    } else {
        [1.0, 0.56, 0.44, 0.90]
    };

    let z_top = top_z_wcs + 0.12;
    let z_bot = bottom_z_wcs;
    let has_depth = (top_z_wcs - bottom_z_wcs).abs() > 0.25;

    if !avoid_nominal_region.is_empty() {
        emit_region_set_top_fill(
            avoid_nominal_region,
            wcs_frame,
            z_top,
            fill_col,
            overlay_tris,
        );
        emit_region_set_loops(
            avoid_nominal_region,
            wcs_frame,
            z_top + 0.03,
            nominal_col,
            3.0,
            false,
            edges,
        );
        if has_depth {
            emit_region_set_loops(
                avoid_nominal_region,
                wcs_frame,
                z_bot,
                nominal_bot_col,
                1.5,
                false,
                edges,
            );
            emit_region_vertical_ticks(
                avoid_nominal_region,
                wcs_frame,
                z_top,
                z_bot,
                nominal_bot_col,
                1.3,
                edges,
            );
        }
    }

    if !avoid_effective_region.is_empty() {
        emit_region_set_loops(
            avoid_effective_region,
            wcs_frame,
            top_z_wcs + 0.20,
            effective_col,
            2.3,
            true,
            edges,
        );
    }
}

/// Samples a closed [`ArcPolyline2D`] into a simple 2D polygon point list for translucent fill.
fn sample_closed_polyline_points(poly: &ArcPolyline2D) -> Vec<DVec2> {
    let mut pts = Vec::new();
    for seg in &poly.segments {
        match *seg {
            ArcSegment2D::Line { start, .. } => {
                if pts
                    .last()
                    .is_none_or(|p: &DVec2| p.distance_squared(start) > 1e-8)
                {
                    pts.push(start);
                }
            }
            ArcSegment2D::Arc { start, .. } => {
                let sweep = seg.signed_sweep_rad().abs();
                let steps = ((sweep / (10.0_f64.to_radians())).ceil() as usize).clamp(4, 48);
                if pts
                    .last()
                    .is_none_or(|p: &DVec2| p.distance_squared(start) > 1e-8)
                {
                    pts.push(start);
                }
                for k in 1..steps {
                    if let Some((pt, _)) = seg.eval_at((k as f64) / (steps as f64))
                        && pts
                            .last()
                            .is_none_or(|p: &DVec2| p.distance_squared(pt) > 1e-8)
                    {
                        pts.push(pt);
                    }
                }
            }
        }
    }
    if pts.len() >= 2
        && let (Some(first), Some(last)) = (pts.first(), pts.last())
        && first.distance_squared(*last) <= 1e-8
    {
        pts.pop();
    }
    pts
}

fn emit_region_set_top_fill(
    region: &RegionSet2D,
    wcs_frame: &RigidFrame3,
    z_wcs: f64,
    color: [f32; 4],
    overlay_tris: &mut Vec<OverlayVertexGpu>,
) {
    let to_w = |p: DVec2| wcs_frame.local_to_world_point(DVec3::new(p.x, p.y, z_wcs));
    for r in &region.regions {
        let pts = sample_closed_polyline_points(&r.outer_ccw);
        if pts.len() < 3 {
            continue;
        }
        for [i0, i1, i2] in chip_geom2d::ear_clip_ccw_2d(&pts) {
            for p in [pts[i0], pts[i1], pts[i2]] {
                overlay_tris.push(OverlayVertexGpu {
                    pos_mm: dvec3_f32(to_w(p)),
                    color,
                });
            }
        }
    }
}

fn emit_region_set_loops(
    region: &RegionSet2D,
    wcs_frame: &RigidFrame3,
    z_wcs: f64,
    color: [f32; 4],
    width_px: f32,
    dashed: bool,
    out: &mut Vec<EdgeInstanceGpu>,
) {
    let lift = wcs_frame.z_axis * z_wcs;
    for r in &region.regions {
        if dashed {
            emit_dashed_arc_polyline_edges(&r.outer_ccw, wcs_frame, lift, color, width_px, out);
            for isl in &r.islands_cw {
                emit_dashed_arc_polyline_edges(isl, wcs_frame, lift, color, width_px, out);
            }
        } else {
            emit_arc_polyline_edges(&r.outer_ccw, wcs_frame, lift, color, width_px, out);
            for isl in &r.islands_cw {
                emit_arc_polyline_edges(isl, wcs_frame, lift, color, width_px, out);
            }
        }
    }
}

fn emit_dashed_arc_polyline_edges(
    poly: &ArcPolyline2D,
    wf: &RigidFrame3,
    lift: DVec3,
    color: [f32; 4],
    width_px: f32,
    out: &mut Vec<EdgeInstanceGpu>,
) {
    let perim = poly.perimeter().max(1.0);
    let dash_mm = (perim / 44.0).clamp(1.6, 5.5);
    let gap_mm = (dash_mm * 0.62).clamp(1.0, 3.4);
    let cycle = dash_mm + gap_mm;
    let to_w = |p: DVec2| wf.local_to_world_point(DVec3::new(p.x, p.y, 0.0)) + lift;

    let mut pts: Vec<DVec2> = Vec::new();
    for seg in &poly.segments {
        match *seg {
            ArcSegment2D::Line { start, end } => {
                if pts.is_empty() {
                    pts.push(start);
                }
                pts.push(end);
            }
            ArcSegment2D::Arc { start, end, .. } => {
                let sweep = seg.signed_sweep_rad().abs();
                let steps = ((sweep / (7.5_f64.to_radians())).ceil() as usize).clamp(4, 64);
                if pts.is_empty() {
                    pts.push(start);
                }
                for k in 1..=steps {
                    let next = if k == steps {
                        end
                    } else if let Some((pt, _)) = seg.eval_at((k as f64) / (steps as f64)) {
                        pt
                    } else {
                        end
                    };
                    pts.push(next);
                }
            }
        }
    }

    let mut dist = 0.0_f64;
    for w in pts.windows(2) {
        let a = w[0];
        let b = w[1];
        let seg_len = (b - a).length();
        if !seg_len.is_finite() || seg_len <= 1e-6 {
            continue;
        }
        let mut local = 0.0_f64;
        let max_iters = ((seg_len / cycle).ceil() as usize + 4) * 4;
        for _ in 0..max_iters {
            if local >= seg_len - 1e-6 {
                break;
            }
            let phase = (dist + local).rem_euclid(cycle);
            if phase < dash_mm - 1e-9 {
                let step = (dash_mm - phase).min(seg_len - local).max(1e-5);
                let next_local = (local + step).min(seg_len);
                let t0 = local / seg_len;
                let t1 = next_local / seg_len;
                let p0 = to_w(a.lerp(b, t0));
                let p1 = to_w(a.lerp(b, t1));
                out.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(p0),
                    p1_mm: dvec3_f32(p1),
                    color,
                    width_px,
                });
                local = next_local;
            } else {
                let skip = (cycle - phase).min(seg_len - local).max(1e-5);
                local = (local + skip).min(seg_len);
            }
        }
        dist += seg_len;
    }
}

fn emit_region_vertical_ticks(
    region: &RegionSet2D,
    wcs_frame: &RigidFrame3,
    z_top: f64,
    z_bot: f64,
    color: [f32; 4],
    width_px: f32,
    out: &mut Vec<EdgeInstanceGpu>,
) {
    for r in &region.regions {
        let seg_count = r.outer_ccw.segments.len();
        if seg_count == 0 {
            continue;
        }
        let stride = (seg_count / 8).max(1);
        for (i, seg) in r.outer_ccw.segments.iter().enumerate() {
            if i % stride != 0 {
                continue;
            }
            let pt = match *seg {
                ArcSegment2D::Line { start, .. } | ArcSegment2D::Arc { start, .. } => start,
            };
            let p_top = wcs_frame.local_to_world_point(DVec3::new(pt.x, pt.y, z_top));
            let p_bot = wcs_frame.local_to_world_point(DVec3::new(pt.x, pt.y, z_bot));
            out.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(p_top),
                p1_mm: dvec3_f32(p_bot),
                color,
                width_px,
            });
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn emit_rotary_sector_preview(
    rot: &ResolvedRotaryConfig,
    window: &chip_cam::ResolvedRotaryWindow,
    avoid_axial_bands: &[(f64, f64)],
    wcs_frame: &RigidFrame3,
    sim_rotary_deg: f64,
    is_light: bool,
    overlay_tris: &mut Vec<OverlayVertexGpu>,
    edges: &mut Vec<EdgeInstanceGpu>,
) {
    let (e1, _, u_vec) = rot.axis.basis();
    let u_center = rot.center_wcs.dot(u_vec);
    let radial_ref = if rot.axis == RotaryAxisKind::ZAxis {
        e1
    } else {
        DVec3::Z
    };

    // Map commanded rotary angle `theta_deg` onto the workpiece in exact lockstep with
    // `toolpath_instances.rs` (`net_deg = -theta_deg + sim_rotary_deg`).
    let sector_world = |u: f64, theta_deg: f64, r: f64| -> DVec3 {
        let p_top = rot.center_wcs + u_vec * (u - u_center) + radial_ref * r;
        let p_wcs = rot.rotate_point_wcs(p_top, -theta_deg + sim_rotary_deg);
        wcs_frame.local_to_world_point(p_wcs)
    };

    let sweep_deg = (window.end_angle_deg - window.start_angle_deg).clamp(1.0, 360.0);
    let is_full = sweep_deg >= 359.5;
    let segs = ((sweep_deg / 7.5).ceil() as usize).clamp(8, 72);
    let r_sleeve = window.r_ceiling + 0.18;

    let fill_col = if is_light {
        [0.04, 0.56, 0.74, 0.18]
    } else {
        [0.16, 0.88, 0.98, 0.15]
    };
    let ring_col = if is_light {
        [0.02, 0.48, 0.78, 0.98]
    } else {
        [0.20, 0.94, 1.0, 0.98]
    };
    let rib_col = if is_light {
        [0.04, 0.52, 0.78, 0.55]
    } else {
        [0.20, 0.90, 1.0, 0.50]
    };
    let floor_col = if is_light {
        [0.04, 0.58, 0.46, 0.78]
    } else {
        [0.24, 0.94, 0.72, 0.78]
    };

    // 1. Cylindrical cutting sleeve + crisp -U / +U endcap rings
    let mut ring_u0 = Vec::with_capacity(segs + 1);
    let mut ring_u1 = Vec::with_capacity(segs + 1);
    for k in 0..=segs {
        let th = window.start_angle_deg + sweep_deg * (k as f64 / segs as f64);
        ring_u0.push(sector_world(window.u_start, th, r_sleeve));
        ring_u1.push(sector_world(window.u_end, th, r_sleeve));
    }

    for k in 0..segs {
        push_quad_tris(
            overlay_tris,
            ring_u0[k],
            ring_u1[k],
            ring_u1[k + 1],
            ring_u0[k + 1],
            fill_col,
        );
        edges.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(ring_u0[k]),
            p1_mm: dvec3_f32(ring_u0[k + 1]),
            color: ring_col,
            width_px: 3.0,
        });
        edges.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(ring_u1[k]),
            p1_mm: dvec3_f32(ring_u1[k + 1]),
            color: ring_col,
            width_px: 3.0,
        });
    }

    // Axial generator ribs + prominent sector boundary edges
    let rib_stride = (segs / 6).max(1);
    for k in 0..=segs {
        let is_boundary = !is_full && (k == 0 || k == segs);
        if is_boundary || (k < segs && k % rib_stride == 0) {
            edges.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(ring_u0[k]),
                p1_mm: dvec3_f32(ring_u1[k]),
                color: if is_boundary { ring_col } else { rib_col },
                width_px: if is_boundary { 2.7 } else { 1.35 },
            });
        }
    }

    // Partial-sector radial wedge edges at start/end angles
    if !is_full {
        let r_in = window.r_floor.max(0.5);
        for &th in &[window.start_angle_deg, window.end_angle_deg] {
            for &u in &[window.u_start, window.u_end] {
                let p_in = sector_world(u, th, r_in);
                let p_out = sector_world(u, th, r_sleeve);
                edges.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(p_in),
                    p1_mm: dvec3_f32(p_out),
                    color: ring_col,
                    width_px: 2.2,
                });
            }
        }
    }

    // 2. Inner r_floor reference rings at -U / +U when r_floor > 0.5 mm
    if window.r_floor > 0.5 {
        for &u in &[window.u_start, window.u_end] {
            for k in 0..segs {
                if k % 2 == 1 {
                    continue;
                }
                let th0 = window.start_angle_deg + sweep_deg * (k as f64 / segs as f64);
                let th1 = window.start_angle_deg + sweep_deg * ((k + 1) as f64 / segs as f64);
                let p0 = sector_world(u, th0, window.r_floor);
                let p1 = sector_world(u, th1, window.r_floor);
                edges.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(p0),
                    p1_mm: dvec3_f32(p1),
                    color: floor_col,
                    width_px: 1.85,
                });
            }
        }
    }

    // 3. Rotary Chuck Keep-Out Plane at `window.chuck_guard_u` whenever a chuck is modeled
    let chuck_len = (rot.chuck_u_max - rot.chuck_u_min).abs();
    if chuck_len > 0.5 {
        let r_guard = rot.chuck_radius_mm.max(r_sleeve + 3.0);
        let r_inner = window
            .r_floor
            .max(rot.stock_outer_radius_mm * 0.25)
            .min(r_guard * 0.75);
        let (guard_wire, guard_fill, guard_width) = if window.clamped_by_chuck {
            (
                if is_light {
                    [0.88, 0.12, 0.14, 1.0]
                } else {
                    [1.0, 0.26, 0.26, 1.0]
                },
                [1.0, 0.20, 0.22, 0.18],
                3.6,
            )
        } else {
            (
                if is_light {
                    [0.86, 0.44, 0.02, 0.95]
                } else {
                    [1.0, 0.68, 0.14, 0.92]
                },
                [1.0, 0.66, 0.14, 0.09],
                2.6,
            )
        };

        let guard_segs = 48usize;
        for k in 0..guard_segs {
            let a0 = (k as f64 / guard_segs as f64) * 360.0;
            let a1 = ((k + 1) as f64 / guard_segs as f64) * 360.0;
            let o0 = sector_world(window.chuck_guard_u, a0, r_guard);
            let o1 = sector_world(window.chuck_guard_u, a1, r_guard);
            let i0 = sector_world(window.chuck_guard_u, a0, r_inner);
            let i1 = sector_world(window.chuck_guard_u, a1, r_inner);
            push_quad_tris(overlay_tris, i0, o0, o1, i1, guard_fill);
            edges.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(o0),
                p1_mm: dvec3_f32(o1),
                color: guard_wire,
                width_px: guard_width,
            });
            if k % 6 == 0 {
                edges.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(i0),
                    p1_mm: dvec3_f32(o0),
                    color: [guard_wire[0], guard_wire[1], guard_wire[2], 0.65],
                    width_px: 1.6,
                });
            }
        }
    }

    // 4. Translucent coral/red rotary axial Avoid / Keep-Out sleeve bands (`[u_lo, u_hi]`)
    if !avoid_axial_bands.is_empty() {
        let r_avoid = r_sleeve + 0.42;
        let band_segs = 36usize;
        let band_fill = if is_light {
            [0.90, 0.16, 0.20, 0.26]
        } else {
            [1.0, 0.26, 0.30, 0.28]
        };
        let band_ring = if is_light {
            [0.86, 0.12, 0.16, 0.98]
        } else {
            [1.0, 0.34, 0.36, 0.98]
        };
        let band_rib = if is_light {
            [0.86, 0.16, 0.18, 0.62]
        } else {
            [1.0, 0.42, 0.40, 0.60]
        };

        for &(u_lo, u_hi) in avoid_axial_bands {
            if !u_lo.is_finite() || !u_hi.is_finite() || u_hi <= u_lo + 1e-4 {
                continue;
            }
            let mut b0 = Vec::with_capacity(band_segs + 1);
            let mut b1 = Vec::with_capacity(band_segs + 1);
            for k in 0..=band_segs {
                let th = (k as f64 / band_segs as f64) * 360.0;
                b0.push(sector_world(u_lo, th, r_avoid));
                b1.push(sector_world(u_hi, th, r_avoid));
            }
            for k in 0..band_segs {
                push_quad_tris(overlay_tris, b0[k], b1[k], b1[k + 1], b0[k + 1], band_fill);
                edges.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(b0[k]),
                    p1_mm: dvec3_f32(b0[k + 1]),
                    color: band_ring,
                    width_px: 2.8,
                });
                edges.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(b1[k]),
                    p1_mm: dvec3_f32(b1[k + 1]),
                    color: band_ring,
                    width_px: 2.8,
                });
                if k % 4 == 0 {
                    edges.push(EdgeInstanceGpu {
                        p0_mm: dvec3_f32(b0[k]),
                        p1_mm: dvec3_f32(b1[k]),
                        color: band_rib,
                        width_px: 1.5,
                    });
                }
            }
        }
    }
}
