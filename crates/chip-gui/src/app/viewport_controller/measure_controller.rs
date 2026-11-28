use chip_cad::{PickFilter, snap_brep_pick_to_measure_target, snap_mesh_hit_to_measure_target};
use chip_cam::{
    HeightReference, HeightSpec, analyze_machinability_reach, compute_reach_cache_key,
    resolve_sketch_world_frame, resolve_sketch_world_frame_with_doc, snap_sketch_to_measure_target,
};
use chip_math::{DMat4, DVec2, Ray3, RigidFrame3};
use chip_sketch::SketchAnchor;

use crate::{
    app::ChipCutterApp,
    panels::{
        InspectorFocus, ViewportPickMode, operation_inspector::avoid_card::mark_ops_dirty_with_rest,
    },
};

impl ChipCutterApp {
    /// Toggles [`ViewportPickMode::Measure`] on or off cleanly.
    pub(crate) fn toggle_measure_mode(&mut self) {
        if self.selection.pick_mode == ViewportPickMode::Measure {
            self.selection.measure_state.reset_all();
            self.selection.pick_mode = ViewportPickMode::InspectEntity;
            self.highlight.measure_state = None;
        } else {
            self.selection.sketch_author_start_xy = None;
            self.selection.measure_state.reset_all();
            self.selection.pick_mode = ViewportPickMode::Measure;
            self.highlight.hovered = None;
            self.highlight.hovered_sketch = None;
            self.highlight.measure_state = Some(self.selection.measure_state.clone());
        }
    }

    /// Resolves the active cutter used for Minimum-Radius & Reach Heatmap analysis.
    pub(crate) fn active_tool_for_reach(&self) -> Option<chip_tools::Tool> {
        if let InspectorFocus::Operation { setup_idx, op_idx } = self.selection.inspector_focus
            && let Some(op) = self
                .project
                .setups
                .get(setup_idx)
                .and_then(|s| s.operations.get(op_idx))
            && let Some(tid) = &op.tool_id
            && let Some(t) = self
                .project
                .tool(tid)
                .or_else(|| self.global_tools.tool(tid))
        {
            return Some(t.clone());
        }
        if let InspectorFocus::ProjectTool { tool_idx } = self.selection.inspector_focus
            && let Some(t) = self.project.tools.get(tool_idx)
        {
            return Some(t.clone());
        }
        if let Some(setup) = self.project.setups.get(self.selection.active_setup_idx) {
            for op in &setup.operations {
                if let Some(tid) = &op.tool_id
                    && let Some(t) = self
                        .project
                        .tool(tid)
                        .or_else(|| self.global_tools.tool(tid))
                {
                    return Some(t.clone());
                }
            }
        }
        self.project
            .tools
            .first()
            .or_else(|| self.global_tools.tools.first())
            .cloned()
    }

    /// Dispatches [`analyze_machinability_reach`] on a background `rayon` task when the Reach /
    /// Min-R overlay is enabled and the `(cad_revision, setup_hash, tool_hash)` cache key changed.
    /// Idle frames with a matching cache key do zero work.
    pub(crate) fn sync_reach_heatmap_worker(&mut self, egui_ctx: &egui::Context) {
        if !self.highlight.show_reach_heatmap {
            return;
        }
        let Some((cad_doc, cad_rev)) = self.effective_cad_arc() else {
            self.workers.reach_result = None;
            self.workers.reach_cache_key = None;
            return;
        };
        let Some(tool) = self.active_tool_for_reach() else {
            self.workers.reach_result = None;
            self.workers.reach_cache_key = None;
            return;
        };
        let Some(Ok(rs)) = self.active_resolved_setup() else {
            return;
        };
        let Some(setup) = self
            .project
            .setups
            .get(self.selection.active_setup_idx)
            .cloned()
        else {
            return;
        };

        let key = compute_reach_cache_key(cad_rev, &setup, &rs, &tool, &self.project.sketches);
        if self.workers.reach_cache_key == Some(key)
            || self.workers.reach_in_flight_key == Some(key)
        {
            return;
        }

        self.workers.reach_in_flight_key = Some(key);
        let sketches = self.project.sketches.clone();
        let tx = self.workers.reach_job_tx.clone();
        let repaint_ctx = egui_ctx.clone();
        rayon::spawn(move || {
            let res = analyze_machinability_reach(&cad_doc, &sketches, &setup, &rs, &tool);
            let _ = tx.send((key, std::sync::Arc::new(res)));
            repaint_ctx.request_repaint();
        });
    }

    /// Handles 3D viewport hover snapping and click progression inside [`ViewportPickMode::Measure`].
    pub(crate) fn handle_measure_viewport_interaction(
        &mut self,
        ray: &Ray3,
        mm_per_px: f64,
        primary_click: bool,
        right_click: bool,
    ) {
        if right_click {
            if self.selection.measure_state.result.is_some() {
                self.selection.measure_state.clear_measurement();
                self.highlight.measure_state = Some(self.selection.measure_state.clone());
            } else {
                self.toggle_measure_mode();
            }
            return;
        }

        let resolved_opt = self.active_resolved_setup().and_then(Result::ok);
        let setup_frame = resolved_opt.map_or(RigidFrame3::IDENTITY, |rs| rs.frame);
        let snap_radius_mm = (mm_per_px * 10.0).clamp(0.3, 6.0);

        let mut best_t = f64::INFINITY;
        let mut best_target = None;

        // 1. Exact B-Rep pick + vertex/edge/plane/cylinder snap
        if let Some(m) = &self.assets.model {
            let pick_ray = self.highlight.world_ray_to_cad_model_space(ray);
            if let Some(hit) = m.picker.pick(&m.doc, &pick_ray, mm_per_px, PickFilter::Any)
                && let Some(t) = snap_brep_pick_to_measure_target(
                    &m.doc,
                    &m.topo,
                    &setup_frame,
                    &hit,
                    snap_radius_mm,
                )
            {
                best_t = hit.ray_t;
                best_target = Some(t);
            }
        }

        // 2. Exact 2D Sketch vertex / circle center snap (wins if closer along camera ray)
        if let Some(rs) = &resolved_opt {
            let doc_has_bodies = self.assets.model.is_some();
            let has_cad_rot = self.highlight.has_active_cad_rotary_transform();
            let cad_xf = if has_cad_rot {
                self.highlight.cad_world_transform()
            } else {
                DMat4::IDENTITY
            };
            for sk in &self.project.sketches {
                if !sk.visible {
                    continue;
                }
                let mut wf = if let Some(m) = &self.assets.model {
                    resolve_sketch_world_frame_with_doc(&m.doc, sk, rs)
                } else {
                    resolve_sketch_world_frame(doc_has_bodies, sk, rs)
                };
                if has_cad_rot && matches!(sk.anchor, SketchAnchor::PlanarFace { .. }) {
                    wf = wf.transformed_by(&cad_xf);
                }
                let denom = ray.dir.dot(wf.z_axis);
                if denom.abs() <= 1e-6 {
                    continue;
                }
                let t = (wf.origin_mm - ray.origin).dot(wf.z_axis) / denom;
                if t <= 0.0 || t >= best_t + snap_radius_mm {
                    continue;
                }
                let hit_world = ray.origin + ray.dir * t;
                let hit_local = wf.world_to_local_point(hit_world);
                if let Some(sk_target) = snap_sketch_to_measure_target(
                    sk,
                    &wf,
                    &setup_frame,
                    DVec2::new(hit_local.x, hit_local.y),
                    snap_radius_mm,
                ) {
                    best_t = t;
                    best_target = Some(sk_target);
                }
            }
        }

        // 3. Fallback to triangle mesh surface point for STL/OBJ/3MF when no B-Rep/Sketch target hit
        if best_target.is_none()
            && let Some(m) = &self.assets.model
        {
            let pick_ray = self.highlight.world_ray_to_cad_model_space(ray);
            if let Some((pt_world, normal_world)) = m.picker.pick_surface_normal(&m.doc, &pick_ray)
            {
                best_target = Some(snap_mesh_hit_to_measure_target(
                    pt_world,
                    normal_world,
                    &setup_frame,
                ));
            }
        }

        self.selection.measure_state.hovered_snap = best_target.clone();
        if primary_click && let Some(target) = best_target {
            self.selection.measure_state.on_click_target(target);
        }
        self.highlight.measure_state = Some(self.selection.measure_state.clone());
    }

    /// Applies a measured WCS Z coordinate to the focused Operation's Bottom Height with atomic Undo.
    pub(crate) fn apply_measure_bottom_z_to_focused_op(&mut self, z_wcs: f64) {
        if !z_wcs.is_finite() {
            return;
        }
        if let InspectorFocus::Operation { setup_idx, op_idx } = self.selection.inspector_focus
            && setup_idx < self.project.setups.len()
            && op_idx < self.project.setups[setup_idx].operations.len()
        {
            let op_name = self.project.setups[setup_idx].operations[op_idx]
                .name
                .clone();
            self.history.record_before_edit(
                "Set Operation Bottom Z from Measure",
                None,
                &self.project,
            );
            self.project.setups[setup_idx].operations[op_idx]
                .heights
                .bottom = HeightSpec::new(HeightReference::WcsOrigin, z_wcs);
            mark_ops_dirty_with_rest(&mut self.project.setups[setup_idx], &[op_idx]);
            self.workers.cached_frame_geo = None;
            self.logs.push(format!(
                "Set Bottom Height of '{}' to WCS Z = {}.",
                op_name,
                self.project.display_units.format_length(z_wcs)
            ));
        }
    }
}
