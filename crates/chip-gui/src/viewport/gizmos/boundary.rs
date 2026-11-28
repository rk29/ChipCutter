use chip_cam::{BoundaryPreviewOverlay, RegionSet2D, ResolvedRotaryConfig, RotaryAxisKind};
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3, RigidFrame3};

use super::{push_quad_tris, sketch::emit_arc_polyline_edges};
use crate::viewport::{
    gpu_types::{EdgeInstanceGpu, OverlayVertexGpu},
    highlight::dvec3_f32,
};

/// Emits live 3D viewport cutting-area / machining-boundary preview overlays when
/// `OperationTab::Geometry` is active (`08-ui-viewport.md`).
pub(super) fn emit_boundary_preview_visuals(
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
