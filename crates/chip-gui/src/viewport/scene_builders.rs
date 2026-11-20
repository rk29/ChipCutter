use std::{
    collections::HashSet,
    hash::{Hash as _, Hasher as _},
    sync::Arc,
};

use chip_cad::{CadDocument, EdgeId, FaceId, PickTarget, expand_coaxial_hole_faces};
use chip_cam::{
    ChainDirectionArrow3D, GeneratedToolpath, MotionRole, ResolvedRotaryConfig,
    ResolvedStockGeometry, SegmentMotion3D, StockAxis, TabMarker3D,
};
use chip_math::{
    Aabb3, ArcPolyline2D, ArcSegment2D, DMat4, DVec2, DVec3, DisplayUnit, MM_PER_INCH, RigidFrame3,
};
use chip_sim::{SimStockMesh, SimToolMesh};
use chip_sketch::{SketchDiagnosticKind, SketchEntity, SketchLayer, SketchPickHit};

use super::gpu_types::{EdgeInstanceGpu, OverlayVertexGpu};
use crate::{camera::CadCamera, theme::AppTheme};

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

/// Visible 2D [`SketchLayer`] paired with its resolved 3D World [`RigidFrame3`] for GPU curve rendering.
#[derive(Debug, Clone)]
pub struct SketchLayerVisual {
    pub sketch: SketchLayer,
    pub world_frame: RigidFrame3,
}

/// Active hover, multi-selection, WCS, Stock, Z-plane, 2D Sketch, and Toolpath scene state fed into the 3D CAD/CAM viewport.
#[derive(Debug, Clone)]
pub struct HighlightState {
    pub theme: AppTheme,
    pub hovered: Option<PickTarget>,
    pub selected: Option<PickTarget>,
    pub hovered_sketch: Option<SketchPickHit>,
    pub selected_sketch: Option<SketchPickHit>,
    pub extra_selected_faces: HashSet<FaceId>,
    pub extra_selected_edges: HashSet<EdgeId>,
    pub extra_selected_sketch_regions: HashSet<(String, String)>,
    pub extra_selected_sketch_entities: HashSet<(String, String)>,
    pub sketch_visuals: Vec<SketchLayerVisual>,
    pub sketch_preview_polys: Vec<(RigidFrame3, ArcPolyline2D)>,
    pub sketch_snap_marker: Option<(DVec3, &'static str, DVec2)>,
    pub chain_arrows: Vec<ChainDirectionArrow3D>,
    pub wcs_frame: RigidFrame3,
    pub stock: Option<ResolvedStockGeometry>,
    pub rotary: Option<ResolvedRotaryConfig>,
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
    /// Current 4th-axis rotary angle (`deg`) during simulation playback (rotates wrapped toolpaths in sync with stock).
    pub sim_rotary_deg: f64,
}

impl Default for HighlightState {
    fn default() -> Self {
        Self {
            theme: AppTheme::Dark,
            hovered: None,
            selected: None,
            hovered_sketch: None,
            selected_sketch: None,
            extra_selected_faces: HashSet::new(),
            extra_selected_edges: HashSet::new(),
            extra_selected_sketch_regions: HashSet::new(),
            extra_selected_sketch_entities: HashSet::new(),
            sketch_visuals: Vec::new(),
            sketch_preview_polys: Vec::new(),
            sketch_snap_marker: None,
            chain_arrows: Vec::new(),
            wcs_frame: RigidFrame3::IDENTITY,
            stock: None,
            rotary: None,
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
            sim_rotary_deg: 0.0,
        }
    }
}

impl HighlightState {
    /// Clears all project-scoped selection, overlay, toolpath, sketch, and simulation visual state
    /// while preserving user viewport display toggles and theme settings.
    pub fn clear_project_scene_state(&mut self) {
        self.hovered = None;
        self.selected = None;
        self.hovered_sketch = None;
        self.selected_sketch = None;
        self.extra_selected_faces.clear();
        self.extra_selected_edges.clear();
        self.extra_selected_sketch_regions.clear();
        self.extra_selected_sketch_entities.clear();
        self.sketch_visuals.clear();
        self.sketch_preview_polys.clear();
        self.sketch_snap_marker = None;
        self.chain_arrows.clear();
        self.wcs_frame = RigidFrame3::IDENTITY;
        self.stock = None;
        self.rotary = None;
        self.z_planes_wcs.clear();
        self.toolpaths.clear();
        self.preview_tab_marker_wcs = None;
        self.sim_stock_mesh = None;
        self.sim_tool_mesh = None;
        self.sim_trail_limit = None;
        self.sim_rotary_deg = 0.0;
    }

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

    // 1b. 2D Vector Sketch Layers (`chip-sketch`, rendered in both Pure-2D and Hybrid 3D+2D modes)
    if !highlight.sketch_visuals.is_empty() {
        emit_sketch_layer_visuals(highlight, camera, &mut out);
    }

    // 2. Stock translucent shell + wireframe in active Setup WCS (when not replaced by 3D machined stock)
    if highlight.show_stock
        && highlight.sim_stock_mesh.is_none()
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

pub(crate) fn compute_toolpath_gpu_signature(highlight: &HighlightState) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    highlight.theme.hash(&mut h);
    highlight.show_toolpaths.hash(&mut h);
    highlight.show_rapids.hash(&mut h);
    highlight.sim_trail_limit.hash(&mut h);
    highlight.sim_rotary_deg.to_bits().hash(&mut h);
    if let Some(rot) = highlight.rotary {
        true.hash(&mut h);
        rot.axis.hash(&mut h);
        rot.direction.hash(&mut h);
        rot.center_wcs.x.to_bits().hash(&mut h);
        rot.center_wcs.y.to_bits().hash(&mut h);
        rot.center_wcs.z.to_bits().hash(&mut h);
    } else {
        false.hash(&mut h);
    }
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

    let is_light = highlight.theme.is_light();
    for entry in &highlight.toolpaths {
        let alpha = if entry.is_focused { 0.96_f32 } else { 0.42_f32 };
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
                    let r_col = if is_light {
                        [0.82, 0.48, 0.04, 0.86 * alpha]
                    } else {
                        [0.98, 0.75, 0.16, 0.82 * alpha]
                    };
                    (r_col, 1.5 * width_scale, 1usize)
                }
                SegmentMotion3D::Linear { role, .. } => {
                    let (rgb, w) = role_color_and_width(role, is_light);
                    ([rgb[0], rgb[1], rgb[2], alpha], w * width_scale, 1usize)
                }
                SegmentMotion3D::Arc {
                    radius_mm,
                    signed_sweep_rad,
                    role,
                    ..
                } => {
                    let (rgb, w) = role_color_and_width(role, is_light);
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

            let rot_steps = if highlight.rotary.is_some() {
                let d_deg = (seg.end_rotary_deg - seg.start_rotary_deg).abs();
                if d_deg > 2.5 {
                    ((d_deg / 2.5).ceil() as usize).clamp(2, 128)
                } else {
                    1
                }
            } else {
                1
            };
            let steps = arc_steps.max(rot_steps);
            let sim_rot = highlight.sim_rotary_deg;
            let map_wcs = |p_wcs: DVec3, seg_rot_deg: f64| -> DVec3 {
                let pt = if let Some(rot) = highlight.rotary {
                    let net_deg = -seg_rot_deg + sim_rot;
                    if net_deg.abs() > 1e-6 {
                        rot.rotate_point_wcs(p_wcs, net_deg)
                    } else {
                        p_wcs
                    }
                } else {
                    p_wcs
                };
                frame.local_to_world_point(pt)
            };

            if steps <= 1 {
                let p0 = map_wcs(seg.start_wcs, seg.start_rotary_deg);
                let p1 = map_wcs(seg.end_wcs, seg.end_rotary_deg);
                out.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(p0),
                    p1_mm: dvec3_f32(p1),
                    color,
                    width_px,
                });
            } else {
                let mut prev = map_wcs(seg.start_wcs, seg.start_rotary_deg);
                for k in 1..=steps {
                    let t = (k as f64) / (steps as f64);
                    let cur = map_wcs(seg.sample_point(t), seg.sample_rotary_deg(t));
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

fn role_color_and_width(role: MotionRole, is_light: bool) -> ([f32; 3], f32) {
    if is_light {
        match role {
            MotionRole::Cut => ([0.04, 0.44, 0.84], 2.3),
            MotionRole::FinishCut => ([0.02, 0.58, 0.66], 2.5),
            MotionRole::LeadIn | MotionRole::LeadOut => ([0.06, 0.56, 0.36], 2.0),
            MotionRole::Plunge | MotionRole::Ramp => ([0.82, 0.18, 0.28], 2.2),
            MotionRole::TabRamp => ([0.82, 0.50, 0.04], 2.4),
            MotionRole::LinkTransition => ([0.18, 0.46, 0.80], 1.8),
        }
    } else {
        match role {
            MotionRole::Cut => ([0.22, 0.74, 0.98], 2.25),
            MotionRole::FinishCut => ([0.14, 0.90, 0.94], 2.45),
            MotionRole::LeadIn | MotionRole::LeadOut => ([0.20, 0.84, 0.60], 1.95),
            MotionRole::Plunge | MotionRole::Ramp => ([0.98, 0.44, 0.52], 2.15),
            MotionRole::TabRamp => ([0.98, 0.80, 0.24], 2.35),
            MotionRole::LinkTransition => ([0.56, 0.78, 0.98], 1.75),
        }
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
