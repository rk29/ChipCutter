use chip_cam::ResolvedRotaryConfig;
use chip_math::{DVec3, RigidFrame3};

use super::push_quad_tris;
use crate::viewport::{
    gpu_types::{EdgeInstanceGpu, OverlayVertexGpu},
    highlight::dvec3_f32,
};

/// Emits 3D 4th-axis rotary kinematics gizmo overlays in Setup WCS:
/// 1. Dashed rotary centerline extending past stock & chuck,
/// 2. Rotary chuck collar + 3-jaw markers at `[chuck_u_min, chuck_u_max]`,
/// 3. Circular rotation-direction arrow ring at the free end showing `+θ` polarity,
/// 4. Subtle rotary clearance cylinder wireframe at `clearance_radius_mm`.
pub(super) fn emit_rotary_gizmo_visuals(
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
