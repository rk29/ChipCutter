use std::{
    collections::HashSet,
    hash::{Hash as _, Hasher as _},
    sync::Arc,
};

use chip_cad::{CadDocument, EdgeId, FaceId, PickTarget, expand_coaxial_hole_faces};
use chip_cam::{
    ChainDirectionArrow3D, GeneratedToolpath, MotionRole, ResolvedStockGeometry, SegmentMotion3D,
    StockAxis, TabMarker3D,
};
use chip_math::{Aabb3, DMat4, DVec2, DVec3, DisplayUnit, MM_PER_INCH, RigidFrame3};
use chip_sim::{SimStockMesh, SimToolMesh};

use super::gpu_types::{EdgeInstanceGpu, OverlayVertexGpu};
use crate::camera::CadCamera;

/// Visual Z-height reference plane rendered across the active Setup's stock XY footprint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZPlaneVisual {
    pub z_wcs: f64,
    pub fill_rgba: [f32; 4],
    pub border_rgba: [f32; 4],
}

/// Cached generated toolpath item queued for Pass #5 GPU-instanced rendering (`08-ui-viewport.md`).
#[derive(Debug, Clone)]
pub struct ToolpathRenderEntry {
    pub op_id: String,
    pub cache_key: [u8; 32],
    pub toolpath: Arc<GeneratedToolpath>,
    pub is_focused: bool,
}

/// Active hover, multi-selection, WCS, Stock, Z-plane, and Toolpath scene state fed into the 3D CAD/CAM viewport.
#[derive(Debug, Clone)]
pub struct HighlightState {
    pub hovered: Option<PickTarget>,
    pub selected: Option<PickTarget>,
    pub extra_selected_faces: HashSet<FaceId>,
    pub extra_selected_edges: HashSet<EdgeId>,
    pub chain_arrows: Vec<ChainDirectionArrow3D>,
    pub wcs_frame: RigidFrame3,
    pub stock: Option<ResolvedStockGeometry>,
    pub show_stock: bool,
    pub z_planes_wcs: Vec<ZPlaneVisual>,
    pub ghost_part: bool,
    pub show_grid: bool,
    pub toolpaths: Vec<ToolpathRenderEntry>,
    pub show_toolpaths: bool,
    pub show_rapids: bool,
    pub preview_tab_marker_wcs: Option<TabMarker3D>,
    /// Pass #6: Optional 3D machined `HeightfieldStock` mesh (`(mesh_rev, Arc<SimStockMesh>)`).
    pub sim_stock_mesh: Option<(u64, Arc<SimStockMesh>)>,
    /// Pass #6: Optional 3D revolved Flute+Shank+Holder mesh (`(tool_rev, Arc<SimToolMesh>)`) + WCS tip position.
    pub sim_tool_mesh: Option<(u64, Arc<SimToolMesh>, DVec3)>,
    /// Optional global motion segment limit for progressive toolpath trail during playback.
    pub sim_trail_limit: Option<usize>,
}

impl Default for HighlightState {
    fn default() -> Self {
        Self {
            hovered: None,
            selected: None,
            extra_selected_faces: HashSet::new(),
            extra_selected_edges: HashSet::new(),
            chain_arrows: Vec::new(),
            wcs_frame: RigidFrame3::IDENTITY,
            stock: None,
            show_stock: true,
            z_planes_wcs: Vec::new(),
            ghost_part: false,
            show_grid: true,
            toolpaths: Vec::new(),
            show_toolpaths: true,
            show_rapids: true,
            preview_tab_marker_wcs: None,
            sim_stock_mesh: None,
            sim_tool_mesh: None,
            sim_trail_limit: None,
        }
    }
}

impl HighlightState {
    pub(crate) fn build_face_states(&self, doc: Option<&CadDocument>) -> (u32, Vec<u32>) {
        let face_count = doc.map_or(64, |d| d.faces.len().max(64));
        let mut states = vec![0_u32; face_count];

        let mut hovered_fid = u32::MAX;
        if let Some(PickTarget::Face(fid)) = &self.hovered {
            hovered_fid = fid.0;
            if let Some(cad) = doc {
                for gfid in expand_coaxial_hole_faces(cad, *fid) {
                    if let Some(slot) = states.get_mut(gfid.0 as usize) {
                        *slot = 1;
                    }
                }
            } else if let Some(slot) = states.get_mut(fid.0 as usize) {
                *slot = 1;
            }
        }

        if let Some(PickTarget::Face(fid)) = &self.selected {
            if let Some(cad) = doc {
                for gfid in expand_coaxial_hole_faces(cad, *fid) {
                    if let Some(slot) = states.get_mut(gfid.0 as usize) {
                        *slot = 2;
                    }
                }
            } else if let Some(slot) = states.get_mut(fid.0 as usize) {
                *slot = 2;
            }
        }

        for &fid in &self.extra_selected_faces {
            if let Some(slot) = states.get_mut(fid.0 as usize) {
                *slot = 2;
            }
        }

        (hovered_fid, states)
    }

    fn collect_edge_sets(&self) -> (HashSet<EdgeId>, HashSet<EdgeId>) {
        let mut hov = HashSet::new();
        let mut sel = self.extra_selected_edges.clone();
        match &self.hovered {
            Some(PickTarget::Edge(eid)) => {
                hov.insert(*eid);
            }
            Some(PickTarget::Loop { edges, .. }) => {
                hov.extend(edges.iter().copied());
            }
            _ => {}
        }
        match &self.selected {
            Some(PickTarget::Edge(eid)) => {
                sel.insert(*eid);
            }
            Some(PickTarget::Loop { edges, .. }) => {
                sel.extend(edges.iter().copied());
            }
            _ => {}
        }
        (hov, sel)
    }
}

pub(crate) fn dmat4_to_f32_2d(m: DMat4) -> [[f32; 4]; 4] {
    let cols = m.to_cols_array_2d();
    let mut out = [[0.0_f32; 4]; 4];
    for c in 0..4 {
        for r in 0..4 {
            out[c][r] = cols[c][r] as f32;
        }
    }
    out
}

fn dvec3_f32(v: DVec3) -> [f32; 3] {
    [v.x as f32, v.y as f32, v.z as f32]
}

pub(crate) fn adaptive_grid_steps(
    camera: &CadCamera,
    height_px: f32,
    unit: DisplayUnit,
) -> (f32, f32) {
    let mm_per_px = camera.mm_per_pixel(height_px);
    let target_minor_mm = mm_per_px * 18.0;
    match unit {
        DisplayUnit::Millimeter => {
            let candidates = [0.1, 0.5, 1.0, 5.0, 10.0, 25.0, 50.0, 100.0];
            let minor = candidates
                .into_iter()
                .find(|&c| c >= target_minor_mm)
                .unwrap_or(100.0);
            (minor as f32, (minor * 10.0) as f32)
        }
        DisplayUnit::Inch => {
            let candidates_in = [0.01, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0];
            let target_in = target_minor_mm / MM_PER_INCH;
            let minor_in = candidates_in
                .into_iter()
                .find(|&c| c >= target_in)
                .unwrap_or(5.0);
            let minor_mm = minor_in * MM_PER_INCH;
            let major_mm = minor_mm * 10.0;
            (minor_mm as f32, major_mm as f32)
        }
    }
}

pub(crate) fn build_edge_and_scene_instances(
    doc: Option<&CadDocument>,
    highlight: &HighlightState,
    camera: &CadCamera,
    overlay_tris: &mut Vec<OverlayVertexGpu>,
) -> Vec<EdgeInstanceGpu> {
    let mut out = Vec::new();
    let (hov_edges, sel_edges) = highlight.collect_edge_sets();

    // 1. B-Rep topological edges (hidden when solid machined stock replaces B-Rep unless ghost target is on)
    if (highlight.sim_stock_mesh.is_none() || highlight.ghost_part)
        && let Some(cad_doc) = doc
    {
        for poly in &cad_doc.render_mesh.edge_polylines {
            let (color, width_px) = if sel_edges.contains(&poly.edge_id) {
                ([1.0, 0.78, 0.18, 1.0], 3.4)
            } else if hov_edges.contains(&poly.edge_id) {
                ([0.28, 0.88, 1.0, 1.0], 3.1)
            } else {
                ([0.11, 0.13, 0.16, 0.90], 1.55)
            };

            for w in poly.points_mm.windows(2) {
                out.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(w[0]),
                    p1_mm: dvec3_f32(w[1]),
                    color,
                    width_px,
                });
            }
        }
    }

    // 2. Stock translucent shell + wireframe in active Setup WCS (when not replaced by 3D machined stock)
    if highlight.show_stock
        && highlight.sim_stock_mesh.is_none()
        && let Some(stock) = &highlight.stock
    {
        emit_stock_visuals(stock, &highlight.wcs_frame, overlay_tris, &mut out);
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

pub(crate) fn compute_toolpath_gpu_signature(highlight: &HighlightState) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    highlight.show_toolpaths.hash(&mut h);
    highlight.show_rapids.hash(&mut h);
    highlight.sim_trail_limit.hash(&mut h);
    for v in [
        highlight.wcs_frame.origin_mm,
        highlight.wcs_frame.x_axis,
        highlight.wcs_frame.y_axis,
        highlight.wcs_frame.z_axis,
    ] {
        v.x.to_bits().hash(&mut h);
        v.y.to_bits().hash(&mut h);
        v.z.to_bits().hash(&mut h);
    }
    if highlight.show_toolpaths {
        highlight.toolpaths.len().hash(&mut h);
        for item in &highlight.toolpaths {
            item.op_id.hash(&mut h);
            item.cache_key.hash(&mut h);
            item.is_focused.hash(&mut h);
        }
    }
    h.finish()
}

pub(crate) fn build_toolpath_gpu_instances(highlight: &HighlightState) -> Vec<EdgeInstanceGpu> {
    if !highlight.show_toolpaths || highlight.toolpaths.is_empty() {
        return Vec::new();
    }

    let mut out = Vec::new();
    let frame = &highlight.wcs_frame;
    let mut global_seg_counter = 0usize;

    for entry in &highlight.toolpaths {
        let alpha = if entry.is_focused { 0.96_f32 } else { 0.36_f32 };
        let width_scale = if entry.is_focused { 1.0_f32 } else { 0.80_f32 };

        for seg in &entry.toolpath.segments_wcs {
            let cur_global = global_seg_counter;
            global_seg_counter += 1;
            if let Some(limit) = highlight.sim_trail_limit
                && cur_global > limit
            {
                continue;
            }
            let (color, width_px, arc_steps) = match seg.motion {
                SegmentMotion3D::Rapid => {
                    if !highlight.show_rapids {
                        continue;
                    }
                    ([0.98, 0.75, 0.16, 0.82 * alpha], 1.5 * width_scale, 1usize)
                }
                SegmentMotion3D::Linear { role, .. } => {
                    let (rgb, w) = role_color_and_width(role);
                    ([rgb[0], rgb[1], rgb[2], alpha], w * width_scale, 1usize)
                }
                SegmentMotion3D::Arc {
                    radius_mm,
                    signed_sweep_rad,
                    role,
                    ..
                } => {
                    let (rgb, w) = role_color_and_width(role);
                    // Discretize circular/helical arcs strictly on GPU upload to <= ~0.025 mm chord error
                    let step_angle = if radius_mm > 0.05 {
                        (2.0 * (1.0 - (0.025 / radius_mm).min(0.5)).acos()).clamp(0.06, 0.32)
                    } else {
                        0.25
                    };
                    let n = ((signed_sweep_rad.abs() / step_angle).ceil() as usize).clamp(4, 96);
                    ([rgb[0], rgb[1], rgb[2], alpha], w * width_scale, n)
                }
            };

            if arc_steps <= 1 {
                let p0 = frame.local_to_world_point(seg.start_wcs);
                let p1 = frame.local_to_world_point(seg.end_wcs);
                out.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(p0),
                    p1_mm: dvec3_f32(p1),
                    color,
                    width_px,
                });
            } else {
                let mut prev = frame.local_to_world_point(seg.start_wcs);
                for k in 1..=arc_steps {
                    let t = (k as f64) / (arc_steps as f64);
                    let cur = frame.local_to_world_point(seg.sample_point(t));
                    out.push(EdgeInstanceGpu {
                        p0_mm: dvec3_f32(prev),
                        p1_mm: dvec3_f32(cur),
                        color,
                        width_px,
                    });
                    prev = cur;
                }
            }
        }
    }

    out
}

fn role_color_and_width(role: MotionRole) -> ([f32; 3], f32) {
    match role {
        MotionRole::Cut => ([0.22, 0.74, 0.98], 2.25),
        MotionRole::FinishCut => ([0.14, 0.90, 0.94], 2.45),
        MotionRole::LeadIn | MotionRole::LeadOut => ([0.20, 0.84, 0.60], 1.95),
        MotionRole::Plunge | MotionRole::Ramp => ([0.98, 0.44, 0.52], 2.15),
        MotionRole::TabRamp => ([0.98, 0.80, 0.24], 2.35),
        MotionRole::LinkTransition => ([0.56, 0.78, 0.98], 1.75),
    }
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
    overlay_tris: &mut Vec<OverlayVertexGpu>,
    edges: &mut Vec<EdgeInstanceGpu>,
) {
    let fill_col = [0.92, 0.74, 0.28, 0.085];
    let wire_col = [0.95, 0.78, 0.32, 0.62];

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
