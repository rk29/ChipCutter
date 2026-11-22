use chip_cad::CadDocument;
use chip_cam::{ResolvedRotaryConfig, ResolvedStockGeometry, StockAxis, TabMarker3D};
use chip_math::{Aabb3, ArcPolyline2D, ArcSegment2D, DMat4, DVec2, DVec3, RigidFrame3};
use chip_sketch::{SketchDiagnosticKind, SketchEntity, SketchPickHit};

use super::{
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
    if highlight.show_stock
        && highlight.sim_stock_source.is_none()
        && let Some(stock) = &highlight.stock
    {
        emit_stock_visuals(
            stock,
            &highlight.wcs_frame,
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

    // 3. Translucent Z-Height reference planes across Stock XY footprint
    if !highlight.z_planes_wcs.is_empty() {
        let footprint = highlight.stock.map(|s| s.aabb_wcs()).unwrap_or_else(|| {
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
    for arrow in &highlight.chain_arrows {
        let p0 = arrow.anchor_world_mm - arrow.tangent_world * (0.4 * arrow_scale);
        let p_tip = arrow.anchor_world_mm + arrow.tangent_world * (0.6 * arrow_scale);
        let dir_col = [0.18, 0.96, 0.62, 1.0];
        out.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(p0),
            p1_mm: dvec3_f32(p_tip),
            color: dir_col,
            width_px: 3.2,
        });
        let w_len = 0.35 * arrow_scale;
        for sign in [-1.0, 1.0] {
            let wing = p_tip - arrow.tangent_world * w_len
                + arrow.side_normal_world * (sign * 0.55 * w_len);
            out.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(p_tip),
                p1_mm: dvec3_f32(wing),
                color: dir_col,
                width_px: 2.6,
            });
        }
        // Perpendicular offset-side tick (amber/magenta) showing Left vs Right cutter compensation side
        let side_tip = arrow.anchor_world_mm + arrow.side_normal_world * (0.55 * arrow_scale);
        out.push(EdgeInstanceGpu {
            p0_mm: dvec3_f32(arrow.anchor_world_mm),
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

    out
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

fn emit_stock_visuals(
    stock: &ResolvedStockGeometry,
    wcs_frame: &RigidFrame3,
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
            let c: [DVec3; 8] = corners_wcs.map(|p| wcs_frame.local_to_world_point(p));

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
                wcs_frame.local_to_world_point(p_wcs)
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
