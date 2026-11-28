use chip_cam::{ResolvedStockGeometry, StockAxis, TabMarker3D};
use chip_math::{Aabb3, DMat4, DVec2, DVec3, RigidFrame3};

use super::push_quad_tris;
use crate::{
    camera::CadCamera,
    viewport::{
        gpu_types::{EdgeInstanceGpu, OverlayVertexGpu},
        highlight::{HighlightState, dvec3_f32},
    },
};

pub(super) fn emit_tab_marker_visual(
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

pub(super) fn emit_z_planes_visuals(
    highlight: &HighlightState,
    stock_rot_wcs: DMat4,
    overlay_tris: &mut Vec<OverlayVertexGpu>,
    out: &mut Vec<EdgeInstanceGpu>,
) {
    let footprint = highlight
        .stock
        .map(|s| {
            let raw = s.aabb_wcs();
            if highlight.has_active_cad_rotary_transform() && raw.is_valid() {
                let mut rot_aabb = Aabb3::EMPTY;
                for x in [raw.min.x, raw.max.x] {
                    for y in [raw.min.y, raw.max.y] {
                        for z in [raw.min.z, raw.max.z] {
                            rot_aabb
                                .extend_point(stock_rot_wcs.transform_point3(DVec3::new(x, y, z)));
                        }
                    }
                }
                rot_aabb
            } else {
                raw
            }
        })
        .unwrap_or_else(|| Aabb3::new(DVec3::new(-40.0, -30.0, 0.0), DVec3::new(40.0, 30.0, 10.0)));
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

pub(super) fn emit_wcs_triad_visuals(
    wcs_frame: &RigidFrame3,
    camera: &CadCamera,
    out: &mut Vec<EdgeInstanceGpu>,
) {
    let triad_len = (camera.scene_radius_mm * 0.30).clamp(8.0, 48.0);
    let head = triad_len * 0.18;
    let origin_w = wcs_frame.origin_mm;
    let triad_axes = [
        (wcs_frame.x_axis, wcs_frame.y_axis, [0.96, 0.30, 0.30, 1.0]),
        (wcs_frame.y_axis, wcs_frame.x_axis, [0.28, 0.90, 0.42, 1.0]),
        (wcs_frame.z_axis, wcs_frame.x_axis, [0.34, 0.62, 1.0, 1.0]),
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
}

pub(super) fn emit_chain_arrows_visuals(
    highlight: &HighlightState,
    camera: &CadCamera,
    out: &mut Vec<EdgeInstanceGpu>,
) {
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
}
