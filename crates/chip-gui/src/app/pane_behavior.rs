use std::path::Path;

use chip_cad::{CurveKind, PickFilter, PickTarget, SurfaceKind};
use chip_cam::WcsOriginMode;
use chip_sim::SimDiagnostic;

use crate::{
    app::ChipCutterApp,
    layout::{WorkspacePane, focus_workspace_pane},
    panels::{
        InspectorFocus, SetupTab, ViewportPickMode,
        browser::{BrowserAction, CamBrowserCtx, show_cam_browser},
        diagnostics::show_diagnostics_panel,
        gcode_preview::{GCodePanelCtx, GCodeUiAction},
        operation_inspector::{OperationInspectorCtx, show_operation_inspector},
        setup_inspector::{SetupInspectorCtx, show_setup_inspector},
        simulation::{SimPanelCtx, SimulationUiAction},
    },
    viewport::{draw_view_cube_overlay, draw_wcs_origin_27pt_overlay},
};

pub(crate) struct PaneBehavior<'a> {
    pub app: &'a mut ChipCutterApp,
}

impl egui_tiles::Behavior<WorkspacePane> for PaneBehavior<'_> {
    fn tab_title_for_pane(&mut self, pane: &WorkspacePane) -> egui::WidgetText {
        pane.title().into()
    }

    fn pane_ui(
        &mut self,
        ui: &mut egui::Ui,
        _tile_id: egui_tiles::TileId,
        pane: &mut WorkspacePane,
    ) -> egui_tiles::UiResponse {
        match *pane {
            WorkspacePane::CamBrowser => self.ui_cam_browser(ui),
            WorkspacePane::Viewport3D => self.ui_viewport_3d(ui),
            WorkspacePane::Inspector => self.ui_inspector(ui),
            WorkspacePane::SimulationTimeline => self.ui_simulation(ui),
            WorkspacePane::GCodePreview => self.ui_gcode_preview(ui),
            WorkspacePane::DiagnosticsLog => self.ui_diagnostics(ui),
        }
        egui_tiles::UiResponse::None
    }
}

impl PaneBehavior<'_> {
    fn ui_cam_browser(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        egui::Frame::NONE.inner_margin(8.0).show(ui, |ui| {
            let cad_ref = self.app.model.as_ref().map(|m| m.doc.as_ref());
            let cad_rev = self.app.model.as_ref().map_or(0, |m| m.revision);
            let action = show_cam_browser(
                ui,
                CamBrowserCtx {
                    project: &mut self.app.project,
                    history: &mut self.app.history,
                    toolpath_cache: &self.app.toolpath_cache,
                    cad_doc: cad_ref,
                    cad_revision: cad_rev,
                    active_setup_idx: &mut self.app.active_setup_idx,
                    focus: &mut self.app.inspector_focus,
                    setup_tab: &mut self.app.setup_tab,
                    op_tab: &mut self.app.op_tab,
                    pick_mode: &mut self.app.pick_mode,
                },
            );

            if let Some(act) = action {
                match act {
                    BrowserAction::OpenStepDialog => self.app.open_step_dialog(&ctx),
                    BrowserAction::ReloadStepModel => self.app.reload_active_step_model(&ctx),
                    BrowserAction::ReplaceStepDialog => self.app.replace_step_dialog(&ctx),
                    BrowserAction::LoadMultiFeatureSample { tilted } => {
                        self.app
                            .spawn_multi_feature_sample_load(tilted, !tilted, ctx.clone());
                    }
                    BrowserAction::LoadCamTortureBlock => {
                        self.app.spawn_cam_torture_sample_load(true, ctx.clone());
                    }
                    BrowserAction::LoadAdaptiveRestDemo => {
                        self.app.spawn_adaptive_rest_demo_load(ctx.clone());
                    }
                    BrowserAction::LoadRevisionDemoV1 => {
                        self.app.spawn_revision_demo_load(false, ctx.clone());
                    }
                    BrowserAction::ReloadRevisionDemoV2 => {
                        self.app.spawn_revision_demo_load(true, ctx.clone());
                    }
                    BrowserAction::OpenToolLibrary => {
                        self.app.tool_library_modal.open = true;
                    }
                    BrowserAction::AlignCameraToActiveSetup => {
                        self.app.align_camera_to_active_setup();
                    }
                    BrowserAction::GenerateSetup(s_idx) => {
                        self.app.trigger_generate_dirty(Some(s_idx), None);
                    }
                    BrowserAction::GenerateSingleOp { setup_idx, op_id } => {
                        self.app
                            .trigger_generate_dirty(Some(setup_idx), Some(&op_id));
                    }
                    BrowserAction::Log(msg) => {
                        self.app.logs.push(msg);
                    }
                }
            }
        });
    }

    fn ui_viewport_3d(&mut self, ui: &mut egui::Ui) {
        let avail = ui.available_rect_before_wrap();
        let response = ui.allocate_rect(avail, egui::Sense::click_and_drag());

        self.app.camera.handle_viewport_input(ui, &response);
        self.app.highlight.preview_tab_marker_wcs = None;

        if let ViewportPickMode::PlaceContourTab { setup_idx, op_idx } = self.app.pick_mode {
            if let Some(cursor_pos) = response.hover_pos()
                && let Some(ray) = self.app.camera.screen_ray(cursor_pos, avail)
            {
                let mm_per_px = self.app.camera.mm_per_pixel(avail.height());
                let primary_click = response.clicked_by(egui::PointerButton::Primary);
                let right_click = response.clicked_by(egui::PointerButton::Secondary);
                self.app.handle_manual_tab_viewport_interaction(
                    setup_idx,
                    op_idx,
                    &ray,
                    mm_per_px,
                    primary_click,
                    right_click,
                );
            }
        } else {
            let filter = self.app.effective_pick_filter();
            let mut clicked_target = None;
            let mut was_primary_click = false;

            if let Some(m) = &self.app.model {
                if let Some(cursor_pos) = response.hover_pos()
                    && let Some(ray) = self.app.camera.screen_ray(cursor_pos, avail)
                {
                    let mm_per_px = self.app.camera.mm_per_pixel(avail.height());
                    let hit = m.picker.pick(&m.doc, &ray, mm_per_px, filter);
                    self.app.highlight.hovered = hit.map(|h| h.target);
                }
                if response.clicked_by(egui::PointerButton::Primary) {
                    was_primary_click = true;
                    clicked_target = self.app.highlight.hovered.clone();
                }
            }

            if was_primary_click {
                self.app.handle_viewport_pick_click(clicked_target);
                self.app.sync_viewport_scene_state();
            }
        }

        let ppp = ui.ctx().pixels_per_point();
        let width_px = ((avail.width() * ppp).round() as u32).max(2);
        let height_px = ((avail.height() * ppp).round() as u32).max(2);

        if let (Some(vp), Some(rs)) = (
            self.app.viewport_renderer.as_mut(),
            self.app.wgpu_render_state.as_ref(),
        ) {
            let doc_ref = self
                .app
                .model
                .as_ref()
                .map(|m| (m.doc.as_ref(), m.revision));
            let tex_id = vp.render_for_egui(
                rs,
                [width_px, height_px],
                &self.app.camera,
                doc_ref,
                &self.app.highlight,
                self.app.project.display_units,
            );
            ui.painter().image(
                tex_id,
                avail,
                egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                egui::Color32::WHITE,
            );
        }

        if self.app.show_27pt_dots
            && let InspectorFocus::Setup { setup_idx } = self.app.inspector_focus
            && self.app.setup_tab == SetupTab::Wcs
            && setup_idx == self.app.active_setup_idx
            && let Some(Ok(rs)) = self.app.active_resolved_setup()
            && let Some(setup) = self.app.project.setups.get(setup_idx)
        {
            let target_box_and_anchor = match setup.wcs.origin {
                WcsOriginMode::StockBoxPoint { anchor } => {
                    Some((rs.stock.aabb_wcs(), anchor, true))
                }
                WcsOriginMode::ModelBoxPoint { anchor } => Some((rs.model_aabb_wcs, anchor, false)),
                WcsOriginMode::SelectedCadFeature { .. } => None,
            };
            if let Some((box_wcs, active_anc, is_stock)) = target_box_and_anchor
                && let Some(new_anchor) = draw_wcs_origin_27pt_overlay(
                    ui,
                    avail,
                    &self.app.camera,
                    &rs.frame,
                    &box_wcs,
                    active_anc,
                )
            {
                self.app.history.record_before_edit(
                    format!("Set WCS Origin to {}", new_anchor.label()),
                    None,
                    &self.app.project,
                );
                self.app.project.setups[setup_idx].wcs.origin = if is_stock {
                    WcsOriginMode::StockBoxPoint { anchor: new_anchor }
                } else {
                    WcsOriginMode::ModelBoxPoint { anchor: new_anchor }
                };
            }
        }

        draw_view_cube_overlay(ui, avail, &mut self.app.camera);

        if let Some(banner) = self.app.pick_mode.banner_text() {
            let banner_pos = egui::pos2(avail.left() + 12.0, avail.top() + 14.0);
            ui.painter().text(
                banner_pos,
                egui::Align2::LEFT_TOP,
                format!(">> {banner}"),
                egui::FontId::proportional(13.0),
                egui::Color32::from_rgb(108, 224, 255),
            );
        }

        let setup_lbl = self
            .app
            .project
            .setups
            .get(self.app.active_setup_idx)
            .map(|s| format!("{} [{}]", s.name, s.wcs_slot.short_code()))
            .unwrap_or_else(|| "No Setup".to_string());

        let hud_pos = egui::pos2(avail.left() + 10.0, avail.bottom() - 24.0);
        ui.painter().text(
            hud_pos,
            egui::Align2::LEFT_CENTER,
            format!(
                "{setup_lbl} | {} | {} | Units: {}",
                if self.app.camera.projection.is_orthographic() {
                    "Ortho"
                } else {
                    "Persp"
                },
                self.app.camera.mouse_preset.short_label(),
                self.app.project.display_units.suffix()
            ),
            egui::FontId::proportional(11.5),
            egui::Color32::from_rgba_unmultiplied(210, 220, 232, 195),
        );
    }

    fn ui_inspector(&mut self, ui: &mut egui::Ui) {
        let _ = self.app.ensure_frame_geo_cached();
        // Temporarily take `cached_frame_geo` out of `self.app` so we can borrow its pre-computed
        // `ResolvedSetup` & `ExtractedSelectionSummary` alongside `&mut self.app.project` with zero clones!
        let mut frame_geo = self.app.cached_frame_geo.take();

        egui::Frame::NONE
            .inner_margin(8.0)
            .show(ui, |ui| match self.app.inspector_focus {
                InspectorFocus::Setup { setup_idx } => {
                    let cad_ref = self.app.model.as_ref().map(|m| m.doc.as_ref());
                    let resolved_ref = frame_geo
                        .as_ref()
                        .and_then(|fg| fg.resolved_setup.as_ref())
                        .map(|r| r.as_ref().map_err(String::as_str));
                    let align = show_setup_inspector(
                        ui,
                        SetupInspectorCtx {
                            project: &mut self.app.project,
                            setup_idx,
                            history: &mut self.app.history,
                            cad_doc: cad_ref,
                            resolved: resolved_ref,
                            tab: &mut self.app.setup_tab,
                            pick_mode: &mut self.app.pick_mode,
                            show_27pt_dots: &mut self.app.show_27pt_dots,
                            show_z_planes: &mut self.app.show_z_planes,
                        },
                    );
                    if align {
                        self.app.cached_frame_geo = frame_geo.take();
                        self.app.align_camera_to_active_setup();
                    }
                }
                InspectorFocus::Operation { setup_idx, op_idx } => {
                    let cad_ref = self.app.model.as_ref().map(|m| m.doc.as_ref());
                    let resolved_opt = frame_geo
                        .as_ref()
                        .and_then(|fg| fg.resolved_setup.as_ref())
                        .and_then(|r| r.as_ref().ok());
                    let extracted_ref = frame_geo
                        .as_ref()
                        .and_then(|fg| fg.focused_extracted.as_ref())
                        .map(|r| r.as_ref().map_err(String::as_str));
                    show_operation_inspector(
                        ui,
                        OperationInspectorCtx {
                            project: &mut self.app.project,
                            setup_idx,
                            op_idx,
                            global_tools: &self.app.global_tools,
                            history: &mut self.app.history,
                            cad_doc: cad_ref,
                            resolved_setup: resolved_opt,
                            extracted: extracted_ref,
                            tab: &mut self.app.op_tab,
                            pick_mode: &mut self.app.pick_mode,
                            lock_chipload: &mut self.app.lock_chipload,
                            show_z_planes: &mut self.app.show_z_planes,
                            open_tool_manager: &mut self.app.tool_library_modal.open,
                        },
                    );
                }
                InspectorFocus::CadEntity => {
                    self.ui_cad_entity_inspector(ui);
                }
            });

        if self.app.cached_frame_geo.is_none() {
            self.app.cached_frame_geo = frame_geo;
        }
    }

    fn ui_cad_entity_inspector(&mut self, ui: &mut egui::Ui) {
        let u = self.app.project.display_units;
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading("B-Rep Entity Inspector");
                egui::ComboBox::from_id_salt("cad_pick_filter")
                    .selected_text(self.app.pick_filter.label())
                    .show_ui(ui, |ui| {
                        for f in PickFilter::ALL {
                            ui.selectable_value(&mut self.app.pick_filter, f, f.label());
                        }
                    });
            });
            ui.separator();

            match (&self.app.model, &self.app.highlight.selected) {
                (Some(m), Some(PickTarget::Face(fid))) => {
                    if let Some(face) = m.doc.face(*fid) {
                        ui.strong(format!("Face #{} — {}", fid.0, face.surface.short_label()));
                        ui.add_space(4.0);
                        egui::Grid::new("face_props_grid")
                            .num_columns(2)
                            .spacing([12.0, 4.0])
                            .show(ui, |ui| {
                                match face.surface {
                                    SurfaceKind::Plane { origin_mm, normal } => {
                                        ui.label("Normal:");
                                        ui.monospace(format!(
                                            "({:+.4}, {:+.4}, {:+.4})",
                                            normal.x, normal.y, normal.z
                                        ));
                                        ui.end_row();
                                        ui.label("World Z:");
                                        ui.monospace(u.format_length(origin_mm.z));
                                        ui.end_row();
                                    }
                                    SurfaceKind::Cylinder {
                                        axis_origin_mm,
                                        axis_dir,
                                        radius_mm,
                                        is_internal,
                                    } => {
                                        ui.label(if is_internal {
                                            "Hole Diameter (Ø):"
                                        } else {
                                            "Cylinder Diameter (Ø):"
                                        });
                                        ui.monospace(u.format_length(radius_mm * 2.0));
                                        ui.end_row();
                                        ui.label("Center XY:");
                                        ui.monospace(format!(
                                            "({}, {})",
                                            u.format_length(axis_origin_mm.x),
                                            u.format_length(axis_origin_mm.y)
                                        ));
                                        ui.end_row();
                                        ui.label("Axis:");
                                        ui.monospace(format!(
                                            "({:+.3}, {:+.3}, {:+.3})",
                                            axis_dir.x, axis_dir.y, axis_dir.z
                                        ));
                                        ui.end_row();
                                    }
                                    _ => {}
                                }
                                ui.label("Surface Area:");
                                ui.monospace(format!(
                                    "{:.2} {}²",
                                    u.area_from_mm2(face.area_mm2),
                                    u.suffix()
                                ));
                                ui.end_row();
                            });
                    }
                }
                (Some(m), Some(PickTarget::Edge(eid))) => {
                    if let Some(edge) = m.doc.edge(*eid) {
                        ui.strong(format!("Edge #{} — {}", eid.0, edge.curve.short_label()));
                        ui.label(format!("Length: {}", u.format_length(edge.length_mm)));
                        if let CurveKind::CircleArc { radius_mm, .. } = edge.curve {
                            ui.label(format!(
                                "Arc R {} / Ø {}",
                                u.format_length(radius_mm),
                                u.format_length(radius_mm * 2.0)
                            ));
                        }
                    }
                }
                (Some(m), Some(PickTarget::Loop { loop_id, edges, .. })) => {
                    ui.strong(format!(
                        "Edge Loop #{} ({} chained edges)",
                        loop_id.0,
                        edges.len()
                    ));
                    if let Some(lp) = m.doc.brep_loop(*loop_id) {
                        ui.label(format!(
                            "Perimeter: {}",
                            u.format_length(lp.signature.characteristic_measure)
                        ));
                    }
                }
                _ => {
                    ui.weak(
                        "Click any B-Rep face, edge, loop, or cylindrical bore in the 3D viewport.",
                    );
                }
            }

            ui.add_space(14.0);
            ui.heading("Parametric Expression Scratchpad");
            ui.separator();

            let mut ctx = self.app.eval_context();
            ui.label("Tool Diameter (`tool_d`):");
            if self.app.wb_tool_dia.show(ui, "wb_tool_dia", &ctx) {
                self.app.refresh_dependent_expressions();
                ctx = self.app.eval_context();
            }
            ui.add_space(4.0);
            ui.label("Stepover (`ae`):");
            let _ = self.app.wb_stepover.show(ui, "wb_stepover", &ctx);
            ui.add_space(4.0);
            ui.label("Clearance Height (`Z`):");
            let _ = self.app.wb_clearance_z.show(ui, "wb_clearance_z", &ctx);
        });
    }

    fn ui_simulation(&mut self, ui: &mut egui::Ui) {
        let mut pending_action: Option<SimulationUiAction> = None;
        egui::Frame::NONE.inner_margin(6.0).show(ui, |ui| {
            let cad_doc = self.app.model.as_ref().map(|m| &m.doc);
            let cad_rev = self.app.model.as_ref().map_or(0, |m| m.revision);
            let proj_dir = self
                .app
                .project_file_path
                .as_deref()
                .and_then(Path::parent)
                .or(self.app.bundle_extract_dir.as_deref());
            pending_action = self.app.sim_panel.show_ui(
                ui,
                SimPanelCtx {
                    project: &self.app.project,
                    setup_idx: self.app.active_setup_idx,
                    cad_doc,
                    cad_revision: cad_rev,
                    cache: &self.app.toolpath_cache,
                    project_dir: proj_dir,
                    logs: &mut self.app.logs,
                },
            );
        });
        if let Some(act) = pending_action {
            match act {
                SimulationUiAction::FocusDiagnosticsTab => {
                    focus_workspace_pane(&mut self.app.tree, WorkspacePane::DiagnosticsLog);
                }
                SimulationUiAction::CenterCameraWcs(pt_wcs) => {
                    if let Some(Ok(rs)) = self.app.active_resolved_setup() {
                        self.app.camera.target_mm = rs.frame.local_to_world_point(pt_wcs);
                    }
                }
            }
        }
    }

    fn ui_gcode_preview(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        let mut pending_action: Option<GCodeUiAction> = None;
        egui::Frame::NONE.inner_margin(6.0).show(ui, |ui| {
            let cad_doc = self.app.model.as_ref().map(|m| &m.doc);
            let cad_rev = self.app.model.as_ref().map_or(0, |m| m.revision);
            let virtual_proj_path = self.app.virtual_project_file_path();
            pending_action = self.app.gcode_panel.show_ui(
                ui,
                GCodePanelCtx {
                    project: &mut self.app.project,
                    history: &mut self.app.history,
                    setup_idx: self.app.active_setup_idx,
                    cad_doc,
                    cad_revision: cad_rev,
                    cache: &self.app.toolpath_cache,
                    project_file_path: virtual_proj_path.as_deref(),
                    sim_state: &self.app.sim_panel,
                    logs: &mut self.app.logs,
                },
            );
        });
        if let Some(act) = pending_action {
            match act {
                GCodeUiAction::RunSimulationFirst => {
                    focus_workspace_pane(&mut self.app.tree, WorkspacePane::SimulationTimeline);
                    self.app.trigger_active_setup_simulation(&ctx);
                }
                GCodeUiAction::JumpToCrash(diag) => {
                    self.app.jump_to_sim_diagnostic(&diag);
                }
            }
        }
    }

    fn ui_diagnostics(&mut self, ui: &mut egui::Ui) {
        let mut jump_diag: Option<SimDiagnostic> = None;
        egui::Frame::NONE.inner_margin(6.0).show(ui, |ui| {
            let cad_rev = self.app.model.as_ref().map_or(0, |m| m.revision);
            jump_diag = show_diagnostics_panel(
                ui,
                &self.app.project,
                self.app.active_setup_idx,
                &self.app.toolpath_cache,
                cad_rev,
                self.app.sim_panel.active_result.as_deref(),
                &self.app.logs,
            );
        });
        if let Some(diag) = jump_diag {
            self.app.jump_to_sim_diagnostic(&diag);
        }
    }
}

impl ChipCutterApp {
    pub(crate) fn handle_self_test(&mut self, ctx: &egui::Context) {
        let is_busy = self.dispatcher.is_busy() || self.sim_panel.is_busy();
        let Some(st) = &mut self.self_test else {
            if is_busy {
                ctx.request_repaint();
            }
            return;
        };
        if self.loading_in_flight || self.model.is_none() || is_busy {
            ctx.request_repaint();
            return;
        }
        st.frames_rendered += 1;
        if st.frames_rendered == 1
            && let Some(res) = &self.sim_panel.active_result
        {
            self.sim_panel.current_time_sec = res.timeline.total_time_sec * 0.82;
        }
        if st.frames_rendered < 5 {
            ctx.request_repaint();
            return;
        }
        if !st.requested {
            st.requested = true;
            let out_path = st.screenshot_path.clone();
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
                ctx_clone.send_viewport_cmd(egui::ViewportCommand::Close);
            });
        }
    }
}
