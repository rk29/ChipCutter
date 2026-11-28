use chip_cad::{MeasureSnapTarget, SurfaceKind};
use chip_cam::apply_machinability_fix;

use crate::{
    app::ChipCutterApp,
    panels::{
        InspectorFocus, ViewportPickMode,
        operation_inspector::{
            avoid_card::mark_ops_dirty_with_rest,
            machinability_card::evaluate_operation_machinability,
        },
    },
};

impl ChipCutterApp {
    pub(crate) fn handle_self_test(&mut self, ctx: &egui::Context) {
        let is_busy = self.workers.dispatcher.is_busy()
            || self.sim_panel.is_busy()
            || self.sim_panel.is_remeshing()
            || self.workers.reach_in_flight_key.is_some();
        let has_geo = self.has_geometry();
        let ready = has_geo
            || self.modals.welcome.open
            || self.modals.sample_gallery.open
            || self.modals.tool_library.open
            || self.modals.machine_manager.open
            || self.modals.post_profile_inspector.open
            || self.modals.command_palette.open
            || self.modals.shortcut_cheatsheet.open;
        let Some(st) = &mut self.self_test else {
            if is_busy {
                ctx.request_repaint();
            }
            return;
        };
        if self.workers.loading_in_flight || is_busy {
            ctx.request_repaint();
            return;
        }
        if !ready && st.idle_wait_frames < 8 {
            st.idle_wait_frames += 1;
            ctx.request_repaint();
            return;
        }
        if st.capture.auto_generate && !st.generate_triggered && has_geo {
            st.generate_triggered = true;
            if let Some(si) = st.capture.setup_idx
                && si < self.project.setups.len()
            {
                self.selection.active_setup_idx = si;
                self.selection.inspector_focus = InspectorFocus::Setup { setup_idx: si };
                self.workers.cached_frame_geo = None;
            }
            if let Some(eng) = st.capture.sim_engine {
                self.sim_panel.engine = eng;
            }
            if let Some(backend) = st.capture.sim_backend {
                self.sim_panel.backend_choice = backend;
            }
            if let Some(res) = st.capture.sim_resolution {
                self.sim_panel.resolution = res;
            }
            self.sim_panel.show_error_heatmap = st.capture.sim_heatmap;
            self.overlay_prefs.auto_sim_after_gen = true;
            self.trigger_generate_dirty(None, None);
            ctx.request_repaint();
            return;
        }
        let capture_to_apply = if !st.capture_applied {
            st.capture_applied = true;
            Some(st.capture.clone())
        } else {
            None
        };
        if let Some(capture) = capture_to_apply {
            let needs_resim = self.apply_capture_spec(&capture);
            if needs_resim {
                if let Some(st) = &mut self.self_test {
                    st.capture_applied = false;
                }
                self.trigger_active_setup_simulation(ctx);
            }
            ctx.request_repaint();
            return;
        }
        let Some(st) = &mut self.self_test else {
            return;
        };
        st.frames_rendered += 1;
        if st.frames_rendered < 6 {
            ctx.request_repaint();
            return;
        }
        if !st.requested {
            st.requested = true;
            let out_path = st.screenshot_path.clone();
            let thumb_path = st.capture.thumb_path.clone();
            let ctx_clone = ctx.clone();
            ctx.request_screenshot(move |image| {
                let w = image.width() as u32;
                let h = image.height() as u32;
                let rgba: Vec<u8> = image
                    .pixels
                    .iter()
                    .flat_map(|c| [c.r(), c.g(), c.b(), c.a()])
                    .collect();
                if let Some(parent) = out_path.parent()
                    && !parent.as_os_str().is_empty()
                {
                    let _ = std::fs::create_dir_all(parent);
                }
                if let Err(e) = image::save_buffer(&out_path, &rgba, w, h, image::ColorType::Rgba8)
                {
                    tracing::error!("failed to save screenshot: {e}");
                }
                if let Some(t_path) = thumb_path
                    && let Some(buf) = image::RgbaImage::from_raw(w, h, rgba)
                {
                    if let Some(parent) = t_path.parent()
                        && !parent.as_os_str().is_empty()
                    {
                        let _ = std::fs::create_dir_all(parent);
                    }
                    let tw = 480_u32.min(w.max(1));
                    let th = ((u64::from(h) * u64::from(tw)) / u64::from(w.max(1))).max(1) as u32;
                    let resized = image::imageops::resize(
                        &buf,
                        tw,
                        th,
                        image::imageops::FilterType::Lanczos3,
                    );
                    if let Err(e) = resized.save(&t_path) {
                        tracing::error!("failed to save thumbnail: {e}");
                    }
                }
                ctx_clone.send_viewport_cmd(egui::ViewportCommand::Close);
                ctx_clone.request_repaint();
            });
            ctx.request_repaint();
        } else {
            ctx.request_repaint();
        }
    }

    fn apply_capture_spec(&mut self, capture: &crate::capture::CaptureSpec) -> bool {
        use crate::capture::{CaptureFocus, CaptureModal, CapturePane, CaptureView};
        let mut needs_resim = false;
        let prev_setup_idx = self.selection.active_setup_idx;

        if let Some(si) = capture.setup_idx
            && si < self.project.setups.len()
        {
            self.selection.active_setup_idx = si;
            if capture.focus.is_none() {
                self.selection.inspector_focus = InspectorFocus::Setup { setup_idx: si };
            }
            self.workers.cached_frame_geo = None;
        }

        if let Some(focus) = capture.focus {
            match focus {
                CaptureFocus::Project => {
                    self.selection.inspector_focus = InspectorFocus::Project;
                    self.workers.cached_frame_geo = None;
                }
                CaptureFocus::Setup { setup_idx, tab } => {
                    if setup_idx < self.project.setups.len() {
                        self.selection.active_setup_idx = setup_idx;
                        self.selection.inspector_focus = InspectorFocus::Setup { setup_idx };
                        self.selection.setup_tab = tab;
                        self.workers.cached_frame_geo = None;
                    }
                }
                CaptureFocus::Operation {
                    setup_idx,
                    op_idx,
                    tab,
                } => {
                    if let Some(setup) = self.project.setups.get(setup_idx)
                        && op_idx < setup.operations.len()
                    {
                        self.selection.active_setup_idx = setup_idx;
                        self.selection.inspector_focus =
                            InspectorFocus::Operation { setup_idx, op_idx };
                        self.selection.op_tab = tab;
                        self.workers.cached_frame_geo = None;
                    }
                }
                CaptureFocus::Sketch { sketch_idx } => {
                    if sketch_idx < self.project.sketches.len() {
                        self.selection.inspector_focus = InspectorFocus::Sketch { sketch_idx };
                        self.workers.cached_frame_geo = None;
                    }
                }
                CaptureFocus::Model { model_idx } => {
                    if model_idx < self.project.models.len() {
                        self.selection.inspector_focus = InspectorFocus::Model { model_idx };
                        self.workers.cached_frame_geo = None;
                    }
                }
            }
        }

        let wants_sim = capture.auto_generate
            || capture.sim_after_op.is_some()
            || capture.sim_progress.is_some();
        let sim_matches_setup = self.sim_panel.active_result.is_some()
            && self.sim_panel.active_setup_idx == self.selection.active_setup_idx;
        if (self.selection.active_setup_idx != prev_setup_idx
            && self.sim_panel.active_result.is_some())
            || (wants_sim && !sim_matches_setup)
        {
            needs_resim = true;
        }
        if capture.isolate_op {
            self.overlay_prefs.show_all_setup_toolpaths = false;
        }
        if let Some(z_planes) = capture.show_z_planes {
            self.overlay_prefs.show_z_planes = z_planes;
        }
        if let Some(dots) = capture.show_27pt_dots {
            self.overlay_prefs.show_27pt_dots = dots;
        }
        if let Some(show_fix) = capture.show_fixtures {
            self.highlight.show_fixtures = show_fix;
        }
        if let Some(ghost) = capture.ghost_part {
            self.highlight.ghost_part = ghost;
        }
        if capture.reach_heatmap {
            self.highlight.show_reach_heatmap = true;
        }
        if capture.apply_safe_fixes
            && let InspectorFocus::Operation { setup_idx, op_idx } = self.selection.inspector_focus
        {
            let active_machine = self.active_machine().cloned();
            let rs_opt = self
                .ensure_frame_geo_cached()
                .resolved_setup
                .as_ref()
                .and_then(|r| r.as_ref().ok())
                .cloned();
            for _ in 0..4 {
                let cad_ref = self.assets.model.as_ref().map(|m| m.doc.as_ref());
                let fix_opt = evaluate_operation_machinability(
                    &self.project,
                    setup_idx,
                    op_idx,
                    &self.global_tools,
                    active_machine.as_ref(),
                    rs_opt.as_ref(),
                    cad_ref,
                )
                .and_then(|rep| {
                    rep.primary_fixable_finding()
                        .and_then(|f| f.suggested_fix.clone())
                });
                let Some(fix) = fix_opt else {
                    break;
                };
                if !apply_machinability_fix(
                    &mut self.project.setups[setup_idx].operations[op_idx],
                    &fix,
                ) {
                    break;
                }
                mark_ops_dirty_with_rest(&mut self.project.setups[setup_idx], &[op_idx]);
            }
            self.workers.cached_frame_geo = None;
        }
        if capture.measure_demo {
            self.seed_capture_measure_demo();
        }
        if let Some(show_stock) = capture.show_sim_stock {
            self.sim_panel.show_machined_stock = show_stock;
        }
        if let Some(show_cutter) = capture.show_sim_cutter {
            self.sim_panel.show_cutter_assembly = show_cutter;
        }
        self.sim_panel.show_error_heatmap = capture.sim_heatmap;
        if let Some(eng) = capture.sim_engine
            && self.sim_panel.engine != eng
        {
            self.sim_panel.engine = eng;
            if self.sim_panel.active_result.is_some() {
                needs_resim = true;
            }
        }
        if let Some(backend) = capture.sim_backend
            && self.sim_panel.backend_choice != backend
        {
            self.sim_panel.backend_choice = backend;
            if self.sim_panel.active_result.is_some() {
                needs_resim = true;
            }
        }
        if let Some(sim_res) = capture.sim_resolution
            && self.sim_panel.resolution != sim_res
        {
            self.sim_panel.resolution = sim_res;
            if self.sim_panel.active_result.is_some() {
                needs_resim = true;
            }
        }
        if let Some(res) = &self.sim_panel.active_result {
            if let Some(eng) = capture.sim_engine
                && res.engine != eng
            {
                needs_resim = true;
            }
            if let Some(sim_res) = capture.sim_resolution
                && res.resolution != sim_res
            {
                needs_resim = true;
            }
            if let Some(backend) = capture.sim_backend {
                let want_gpu = backend != chip_sim::SimComputeBackendChoice::Cpu
                    && self.sim_panel.has_gpu_context();
                if res.active_backend.is_gpu != want_gpu {
                    needs_resim = true;
                }
            }
            let op_id_for_index = capture.sim_after_op.and_then(|oi| {
                self.project
                    .setups
                    .get(self.selection.active_setup_idx)
                    .and_then(|s| s.operations.get(oi))
                    .map(|op| op.id.as_str())
            });
            self.sim_panel.current_time_sec =
                capture.resolve_sim_timestamp(&res.timeline, op_id_for_index);
            self.sim_panel.is_playing = false;
        }
        if let Some(post_id) = &capture.post_profile
            && let Some(setup) = self.project.setups.get_mut(self.selection.active_setup_idx)
            && !setup.post_profile.eq_ignore_ascii_case(post_id)
        {
            setup.post_profile = post_id.clone();
            self.gcode_panel.invalidate_preview();
            needs_resim = self.sim_panel.active_result.is_some();
        }

        if let Some(pane) = capture.pane {
            crate::layout::focus_workspace_pane(&mut self.tree, pane.to_workspace_pane());
            if pane == CapturePane::GCode {
                let proj_dir = self
                    .session
                    .project_file_path
                    .as_deref()
                    .and_then(std::path::Path::parent)
                    .or(self.session.bundle_extract_dir.as_deref());
                self.gcode_panel.ensure_profiles_loaded(proj_dir);
            }
        }

        if let Some(modal) = capture.modal {
            self.modals.welcome.open = modal == CaptureModal::Welcome;
            self.modals.sample_gallery.open = modal == CaptureModal::SampleGallery;
            self.modals.tool_library.open = modal == CaptureModal::ToolLibrary;
            if modal == CaptureModal::ToolLibrary
                && let InspectorFocus::Operation { setup_idx, op_idx } =
                    self.selection.inspector_focus
                && let Some(op_tool_id) = self
                    .project
                    .setups
                    .get(setup_idx)
                    .and_then(|s| s.operations.get(op_idx))
                    .and_then(|op| op.tool_id.as_ref())
                    .cloned()
            {
                if !self.global_tools.tools.iter().any(|t| t.id == op_tool_id) {
                    for pt in &self.project.tools {
                        if !self.global_tools.tools.iter().any(|t| t.id == pt.id) {
                            self.global_tools.tools.push(pt.clone());
                        }
                    }
                }
                if let Some(idx) = self
                    .global_tools
                    .tools
                    .iter()
                    .position(|t| t.id == op_tool_id)
                {
                    self.modals.tool_library.selected_tool_idx = idx;
                    self.modals.tool_library.selected_preset_idx = 0;
                    self.modals.tool_library.scroll_to_selected = true;
                }
            }
            if modal == CaptureModal::Machines {
                let mid = self.project.effective_machine_id().to_string();
                self.modals
                    .machine_manager
                    .open_selecting(&self.machines, &mid);
            } else {
                self.modals.machine_manager.open = false;
            }
            if modal == CaptureModal::FlipJigWizard {
                self.modals
                    .flip_jig_wizard
                    .open_for_setup(self.selection.active_setup_idx);
            } else {
                self.modals.flip_jig_wizard.open = false;
            }
            if modal == CaptureModal::SoftJawWizard {
                self.modals
                    .soft_jaw_wizard
                    .open_for_setup(self.selection.active_setup_idx);
            } else {
                self.modals.soft_jaw_wizard.open = false;
            }
            self.modals.shortcut_cheatsheet.open = modal == CaptureModal::ShortcutCheatsheet;
            self.modals.about.open = modal == CaptureModal::About;
        }

        if let Some(view) = capture.view {
            match view {
                CaptureView::Standard(v) => {
                    self.camera.set_standard_view(v);
                    self.fit_scene_to_viewport();
                }
                CaptureView::SetupTop => self.align_camera_to_active_setup(),
                CaptureView::ActiveSketch => {
                    let sk_idx = match self.selection.inspector_focus {
                        InspectorFocus::Sketch { sketch_idx } => sketch_idx,
                        _ => 0,
                    };
                    self.align_camera_to_sketch(sk_idx);
                }
            }
        }
        needs_resim
    }

    fn seed_capture_measure_demo(&mut self) {
        self.selection.pick_mode = ViewportPickMode::Measure;
        self.selection.measure_state.reset_all();
        let rs_opt = self
            .ensure_frame_geo_cached()
            .resolved_setup
            .as_ref()
            .and_then(|r| r.as_ref().ok())
            .cloned();
        let (Some(model), Some(rs)) = (&self.assets.model, rs_opt) else {
            return;
        };
        let doc = &model.doc;

        // Prefer two distinct internal cylindrical holes (center-to-center hole pitch)
        let mut holes: Vec<MeasureSnapTarget> = Vec::new();
        for face in &doc.faces {
            if let SurfaceKind::Cylinder {
                axis_origin_mm,
                axis_dir,
                radius_mm,
                is_internal: true,
            } = face.surface
            {
                let origin_wcs = rs.frame.world_to_local_point(axis_origin_mm);
                let dir_wcs = rs.frame.world_to_local_dir(axis_dir).normalize_or_zero();
                let dup = holes.iter().any(|h| match h {
                    MeasureSnapTarget::BRepCylindricalFace {
                        axis_origin_wcs, ..
                    } => {
                        (axis_origin_wcs.x - origin_wcs.x).hypot(axis_origin_wcs.y - origin_wcs.y)
                            < 0.25
                    }
                    _ => false,
                });
                if !dup {
                    let bmin = rs.frame.world_to_local_point(face.aabb_mm.min);
                    let bmax = rs.frame.world_to_local_point(face.aabb_mm.max);
                    let top_z = bmin.z.max(bmax.z);
                    let bot_z = bmin.z.min(bmax.z);
                    holes.push(MeasureSnapTarget::BRepCylindricalFace {
                        face_id: face.id,
                        axis_origin_wcs: chip_math::DVec3::new(origin_wcs.x, origin_wcs.y, top_z),
                        axis_dir_wcs: dir_wcs,
                        diameter_mm: radius_mm * 2.0,
                        depth_mm: (top_z - bot_z).abs(),
                        is_internal: true,
                        top_z_wcs: top_z,
                        bottom_z_wcs: bot_z,
                    });
                }
            }
        }
        if holes.len() >= 2 {
            let mut best = (0usize, 1usize, -1.0_f64);
            for i in 0..holes.len() {
                for j in (i + 1)..holes.len() {
                    let d = holes[i].anchor_wcs().distance(holes[j].anchor_wcs());
                    if d > best.2 {
                        best = (i, j, d);
                    }
                }
            }
            self.selection
                .measure_state
                .on_click_target(holes[best.0].clone());
            self.selection
                .measure_state
                .on_click_target(holes[best.1].clone());
            self.highlight.measure_state = Some(self.selection.measure_state.clone());
            return;
        }

        // Fallback: top horizontal planar face vs deepest horizontal pocket floor
        let mut planes: Vec<MeasureSnapTarget> = Vec::new();
        for face in &doc.faces {
            if let SurfaceKind::Plane { origin_mm, normal } = face.surface {
                let n_wcs = rs.frame.world_to_local_dir(normal).normalize_or_zero();
                if n_wcs.z > 0.99 {
                    let c_world = 0.5 * (face.aabb_mm.min + face.aabb_mm.max);
                    let pt_wcs = rs.frame.world_to_local_point(c_world);
                    let z_wcs = rs.frame.world_to_local_point(origin_mm).z;
                    planes.push(MeasureSnapTarget::BRepPlanarFace {
                        face_id: face.id,
                        point_wcs: chip_math::DVec3::new(pt_wcs.x, pt_wcs.y, z_wcs),
                        normal_wcs: n_wcs,
                        z_wcs,
                        area_mm2: face.area_mm2,
                    });
                }
            }
        }
        planes.sort_by(|a, b| {
            let za = a.anchor_wcs().z;
            let zb = b.anchor_wcs().z;
            zb.total_cmp(&za)
        });
        if let (Some(top), Some(bot)) = (planes.first(), planes.last())
            && (top.anchor_wcs().z - bot.anchor_wcs().z).abs() > 0.1
        {
            self.selection.measure_state.on_click_target(top.clone());
            self.selection.measure_state.on_click_target(bot.clone());
            self.highlight.measure_state = Some(self.selection.measure_state.clone());
        }
    }
}
