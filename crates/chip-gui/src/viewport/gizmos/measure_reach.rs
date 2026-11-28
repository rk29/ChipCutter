use chip_cad::{MeasureSnapTarget, MeasurementResult};
use chip_cam::{ReachAnalysisResult, ReachSeverity};
use chip_math::{DMat4, DVec3, RigidFrame3};

use super::emit_circle_ring_edges;
use crate::{
    camera::CadCamera,
    viewport::{
        gpu_types::{EdgeInstanceGpu, OverlayVertexGpu},
        highlight::{MeasureToolState, dvec3_f32},
    },
};

pub(super) fn emit_reach_heatmap_visuals(
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

pub(super) fn emit_measure_visuals(
    measure: &MeasureToolState,
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
