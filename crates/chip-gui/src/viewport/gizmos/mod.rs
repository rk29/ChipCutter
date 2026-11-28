mod boundary;
mod measure_reach;
mod rotary;
mod sketch;
mod stock_wcs;

use chip_cad::CadDocument;
use chip_math::{DMat4, DVec3};

pub(crate) use stock_wcs::emit_stock_visuals;

use self::{
    boundary::emit_boundary_preview_visuals,
    measure_reach::{emit_measure_visuals, emit_reach_heatmap_visuals},
    rotary::emit_rotary_gizmo_visuals,
    sketch::emit_sketch_layer_visuals,
    stock_wcs::{
        emit_chain_arrows_visuals, emit_tab_marker_visual, emit_wcs_triad_visuals,
        emit_z_planes_visuals,
    },
};
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
        emit_z_planes_visuals(highlight, stock_rot_wcs, overlay_tris, &mut out);
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
    emit_wcs_triad_visuals(&highlight.wcs_frame, camera, &mut out);

    // 5. 3D Chain Travel Direction & Offset-Side Indicators
    if !highlight.chain_arrows.is_empty() {
        emit_chain_arrows_visuals(highlight, camera, &mut out);
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

pub(super) fn emit_circle_ring_edges(
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

pub(super) fn push_quad_tris(
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
