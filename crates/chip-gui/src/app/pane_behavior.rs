use std::path::Path;

use chip_cad::{CurveKind, PickFilter, PickTarget, SurfaceKind};
use chip_cam::WcsOriginMode;
use chip_sim::SimDiagnostic;

use crate::{
    app::{ChipCutterApp, viewport_controller::SketchAuthorInteractionInput},
    layout::{WorkspacePane, focus_workspace_pane},
    panels::{
        InspectorFocus, OperationTab, SetupTab, ViewportPickMode,
        browser::{BrowserAction, CamBrowserCtx, show_cam_browser},
        diagnostics::show_diagnostics_panel,
        gcode_preview::{GCodePanelCtx, GCodeUiAction},
        operation_inspector::{OperationInspectorCtx, show_operation_inspector},
        setup_inspector::{SetupInspectorCtx, show_setup_inspector},
        simulation::{SimPanelCtx, SimulationUiAction},
        sketch_inspector::{SketchInspectorAction, SketchInspectorCtx, show_sketch_inspector},
    },
    viewport::{
        GuidanceHudAction, ViewportHudAction, draw_view_cube_overlay, draw_viewport_bottom_hud,
        draw_viewport_guidance_hud, draw_wcs_origin_27pt_overlay,
    },
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
            let eff_cad = self.app.effective_cad_arc();
            let cad_ref = self.app.model.as_ref().map(|m| m.doc.as_ref());
            let cad_rev = eff_cad.as_ref().map_or(0, |(_, r)| *r);
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
                    BrowserAction::OpenMeshImportModal { browse_file } => {
                        if browse_file {
                            self.app.mesh_import_modal.open_with_file_picker();
                        } else {
                            self.app.mesh_import_modal.open = true;
                        }
                    }
                    BrowserAction::OpenVectorImportModal { browse_file } => {
                        if browse_file {
                            self.app.vector_import_modal.open_with_file_picker();
                        } else {
                            self.app.vector_import_modal.open = true;
                        }
                    }
                    BrowserAction::ReloadStepModel => self.app.reload_active_step_model(&ctx),
                    BrowserAction::ReplaceStepDialog => self.app.replace_step_dialog(&ctx),
                    BrowserAction::OpenSampleGallery => {
                        self.app.sample_gallery_modal.open = true;
                    }
                    BrowserAction::NewSetupXySketch => {
                        self.app.create_new_setup_xy_sketch();
                    }
                    BrowserAction::StartPickSketchHostFace => {
                        self.app.pick_mode = ViewportPickMode::PickSketchHostFace {
                            setup_idx: self.app.active_setup_idx,
                        };
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
        self.app.highlight.sketch_preview_polys.clear();
        self.app.highlight.sketch_snap_marker = None;

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
        } else if let ViewportPickMode::SketchAuthor { sketch_idx, tool } = self.app.pick_mode {
            if let Some(cursor_pos) = response.hover_pos()
                && let Some(ray) = self.app.camera.screen_ray(cursor_pos, avail)
            {
                let mm_per_px = self.app.camera.mm_per_pixel(avail.height());
                let primary_click = response.clicked_by(egui::PointerButton::Primary);
                let right_click = response.clicked_by(egui::PointerButton::Secondary);
                self.app
                    .handle_sketch_author_viewport_interaction(SketchAuthorInteractionInput {
                        sketch_idx,
                        tool,
                        ray,
                        mm_per_px,
                        primary_click,
                        right_click,
                    });
                if primary_click || right_click {
                    self.app.sync_viewport_scene_state();
                }
            }
        } else {
            let filter = self.app.effective_pick_filter();
            let picking_3d_host_face = matches!(
                self.app.pick_mode,
                ViewportPickMode::PickSketchHostFace { .. }
                    | ViewportPickMode::RebindSketchHostFace { .. }
            );
            let mut clicked_target = None;
            let mut clicked_sketch = None;
            let mut was_primary_click = false;

            if let Some(cursor_pos) = response.hover_pos()
                && let Some(ray) = self.app.camera.screen_ray(cursor_pos, avail)
            {
                let mm_per_px = self.app.camera.mm_per_pixel(avail.height());
                let sketch_hit = if picking_3d_host_face {
                    None
                } else {
                    self.app.pick_sketch_in_viewport(&ray, mm_per_px)
                };
                self.app.highlight.hovered_sketch = sketch_hit;

                if let Some(m) = &self.app.model {
                    let hit = m.picker.pick(&m.doc, &ray, mm_per_px, filter);
                    self.app.highlight.hovered = hit.map(|h| h.target);
                } else {
                    self.app.highlight.hovered = None;
                }
            }

            if response.clicked_by(egui::PointerButton::Primary) {
                was_primary_click = true;
                clicked_sketch = self.app.highlight.hovered_sketch.clone();
                clicked_target = self.app.highlight.hovered.clone();
            }

            if was_primary_click {
                if let Some(sk_hit) = clicked_sketch {
                    self.app.handle_viewport_sketch_click(sk_hit);
                } else if self.app.model.is_some() {
                    self.app.handle_viewport_pick_click(clicked_target);
                } else if self.app.pick_mode == ViewportPickMode::InspectEntity {
                    self.app.highlight.selected_sketch = None;
                }
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

        let tok = crate::theme::ThemeTokens::from_ui(ui);
        if let Some((snap_world, badge, xy_mm)) = self.app.highlight.sketch_snap_marker
            && let Some(screen_pt) = self.app.camera.project_world_to_screen(snap_world, avail)
        {
            let u = self.app.project.display_units;
            let lbl = format!(
                "[{badge}] X={} Y={}",
                u.format_length(xy_mm.x),
                u.format_length(xy_mm.y)
            );
            ui.painter().text(
                egui::pos2(screen_pt.x + 12.0, screen_pt.y - 12.0),
                egui::Align2::LEFT_BOTTOM,
                lbl,
                egui::FontId::monospace(12.0),
                tok.snap_badge_fg,
            );
        }

        let guidance_badge = match self.app.pick_mode {
            ViewportPickMode::OpPlanarFaces { setup_idx, op_idx }
            | ViewportPickMode::OpClosedLoops { setup_idx, op_idx }
            | ViewportPickMode::OpOpenChain {
                setup_idx, op_idx, ..
            }
            | ViewportPickMode::OpCylindricalHoles { setup_idx, op_idx } => self
                .app
                .project
                .setups
                .get(setup_idx)
                .and_then(|s| s.operations.get(op_idx))
                .map(|op| format!("{} selected", op.selections.len())),
            ViewportPickMode::PlaceContourTab { setup_idx, op_idx } => self
                .app
                .project
                .setups
                .get(setup_idx)
                .and_then(|s| s.operations.get(op_idx))
                .and_then(|op| match &op.strategy {
                    Some(chip_cam::StrategyParams::Contour2D(p)) => {
                        Some(format!("{} tab(s)", p.tabs.manual_anchors.len()))
                    }
                    _ => None,
                }),
            ViewportPickMode::SketchAuthor { .. } => {
                if self.app.sketch_author_start_xy.is_some() {
                    Some("1st point set".to_string())
                } else {
                    self.app
                        .sketch_snap_grid_mm
                        .map(|g| format!("Grid: {g} mm"))
                }
            }
            _ => None,
        };
        if let Some(g_act) =
            draw_viewport_guidance_hud(ui, avail, self.app.pick_mode, guidance_badge.as_deref())
        {
            match g_act {
                GuidanceHudAction::Done | GuidanceHudAction::Cancel => {
                    self.app.sketch_author_start_xy = None;
                    self.app.pick_mode = ViewportPickMode::InspectEntity;
                    self.app.highlight.preview_tab_marker_wcs = None;
                }
                GuidanceHudAction::ResetChainStart => {
                    if let ViewportPickMode::OpOpenChain {
                        setup_idx, op_idx, ..
                    } = self.app.pick_mode
                    {
                        self.app.pick_mode = ViewportPickMode::OpOpenChain {
                            setup_idx,
                            op_idx,
                            start_edge: None,
                        };
                    }
                }
            }
        }

        let setup_lbl = self
            .app
            .project
            .setups
            .get(self.app.active_setup_idx)
            .map(|s| {
                let code = s.wcs_slot.short_code();
                if s.name.contains(code) {
                    format!("{} · {}", s.name, self.app.project.display_units.suffix())
                } else {
                    format!(
                        "{} [{}] · {}",
                        s.name,
                        code,
                        self.app.project.display_units.suffix()
                    )
                }
            })
            .unwrap_or_else(|| "No Setup".to_string());

        if let Some(hud_act) = draw_viewport_bottom_hud(
            ui,
            avail,
            &mut self.app.camera,
            &mut self.app.highlight,
            &setup_lbl,
        ) {
            match hud_act {
                ViewportHudAction::FitToViewport => self.app.fit_scene_to_viewport(),
                ViewportHudAction::AlignToSetupTop => self.app.align_camera_to_active_setup(),
            }
        }
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
                    let selected_face_normal_wcs =
                        match (&self.app.highlight.selected, cad_ref, resolved_opt) {
                            (Some(chip_cad::PickTarget::Face(fid)), Some(doc), Some(rs)) => {
                                doc.face(*fid).and_then(|f| match f.surface {
                                    chip_cad::SurfaceKind::Plane { normal, .. } => Some(
                                        rs.frame.world_to_local_dir(normal).normalize_or_zero(),
                                    ),
                                    _ => None,
                                })
                            }
                            _ => None,
                        };
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
                            selected_face_normal_wcs,
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
                InspectorFocus::Sketch { sketch_idx } => {
                    let invalidated = self.ui_sketch_inspector(ui, sketch_idx);
                    if invalidated {
                        frame_geo = None;
                    }
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

            if let Some(m) = &self.app.model
                && !m.doc.meshes.is_empty()
                && m.doc.bodies.is_empty()
            {
                for mesh in &m.doc.meshes {
                    let sz = mesh.aabb_mm.size();
                    let d = &mesh.diagnostics;
                    ui.strong(format!("{} ({})", mesh.name, mesh.format.label()));
                    ui.add_space(4.0);
                    egui::Grid::new(format!("mesh_props_grid_{}", mesh.id.0))
                        .num_columns(2)
                        .spacing([12.0, 4.0])
                        .show(ui, |ui| {
                            ui.label("Applied Unit:");
                            ui.monospace(mesh.applied_unit.label());
                            ui.end_row();
                            ui.label("Dimensions:");
                            ui.monospace(format!(
                                "{} × {} × {}",
                                u.format_length(sz.x),
                                u.format_length(sz.y),
                                u.format_length(sz.z)
                            ));
                            ui.end_row();
                            ui.label("Triangles:");
                            ui.monospace(format!(
                                "{} ({} degenerate removed)",
                                d.valid_triangle_count, d.degenerate_triangles_removed
                            ));
                            ui.end_row();
                            ui.label("Welded Vertices:");
                            ui.monospace(format!("{}", d.welded_vertex_count));
                            ui.end_row();
                            ui.label("Manifold Check:");
                            ui.monospace(if d.is_watertight {
                                "Watertight (0 open, 0 non-manifold)".to_string()
                            } else {
                                format!(
                                    "{} open, {} non-manifold edges",
                                    d.open_boundary_edges, d.non_manifold_edges
                                )
                            });
                            ui.end_row();
                            ui.label("Signed Volume:");
                            ui.monospace(format!("{:.2} cm³", d.signed_volume_mm3 / 1000.0));
                            ui.end_row();
                        });
                }
                return;
            }

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

    fn ui_sketch_inspector(&mut self, ui: &mut egui::Ui, sketch_idx: usize) -> bool {
        let eval_ctx = self.app.eval_context();
        let has_cad_model = self.app.model.is_some();
        let proj_path = self.app.virtual_project_file_path();
        let act = show_sketch_inspector(
            ui,
            SketchInspectorCtx {
                project: &mut self.app.project,
                sketch_idx,
                active_setup_idx: self.app.active_setup_idx,
                selected_entity_idx: &mut self.app.selected_sketch_entity_idx,
                snap_grid_mm: &mut self.app.sketch_snap_grid_mm,
                author_start_xy: &mut self.app.sketch_author_start_xy,
                history: &mut self.app.history,
                pick_mode: &mut self.app.pick_mode,
                eval_ctx: &eval_ctx,
                has_cad_model,
                project_file: proj_path.as_deref(),
            },
        );
        let mut invalidated = false;
        if let Some(action) = act {
            match action {
                SketchInspectorAction::LookAtSketch(idx) => {
                    self.app.cached_frame_geo = None;
                    invalidated = true;
                    self.app.align_camera_to_sketch(idx);
                }
                SketchInspectorAction::InvalidateFrameCache => {
                    self.app.cached_frame_geo = None;
                    invalidated = true;
                }
                SketchInspectorAction::FocusOperation { setup_idx, op_idx } => {
                    self.app.cached_frame_geo = None;
                    invalidated = true;
                    self.app.inspector_focus = InspectorFocus::Operation { setup_idx, op_idx };
                    self.app.op_tab = OperationTab::Geometry;
                }
                SketchInspectorAction::Log(msg) => {
                    self.app.logs.push(msg);
                }
            }
        }
        invalidated
    }

    fn ui_simulation(&mut self, ui: &mut egui::Ui) {
        let mut pending_action: Option<SimulationUiAction> = None;
        egui::Frame::NONE.inner_margin(6.0).show(ui, |ui| {
            let eff_cad = self.app.effective_cad_arc();
            let cad_doc = eff_cad.as_ref().map(|(d, _)| d);
            let cad_rev = eff_cad.as_ref().map_or(0, |(_, r)| *r);
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
            let eff_cad = self.app.effective_cad_arc();
            let cad_doc = eff_cad.as_ref().map(|(d, _)| d);
            let cad_rev = eff_cad.as_ref().map_or(0, |(_, r)| *r);
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
            let cad_rev = self.app.effective_cad_arc().map_or(0, |(_, r)| r);
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
        let has_geo = self.has_geometry();
        let ready = has_geo
            || self.welcome_modal.open
            || self.sample_gallery_modal.open
            || self.tool_library_modal.open
            || self.command_palette.open
            || self.shortcut_cheatsheet.open;
        let Some(st) = &mut self.self_test else {
            if is_busy {
                ctx.request_repaint();
            }
            return;
        };
        if self.loading_in_flight || is_busy {
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
            self.auto_sim_after_gen = true;
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
        let prev_setup_idx = self.active_setup_idx;

        if let Some(focus) = capture.focus {
            match focus {
                CaptureFocus::Setup { setup_idx, tab } => {
                    if setup_idx < self.project.setups.len() {
                        self.active_setup_idx = setup_idx;
                        self.inspector_focus = InspectorFocus::Setup { setup_idx };
                        self.setup_tab = tab;
                        self.cached_frame_geo = None;
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
                        self.active_setup_idx = setup_idx;
                        self.inspector_focus = InspectorFocus::Operation { setup_idx, op_idx };
                        self.op_tab = tab;
                        self.cached_frame_geo = None;
                    }
                }
                CaptureFocus::Sketch { sketch_idx } => {
                    if sketch_idx < self.project.sketches.len() {
                        self.inspector_focus = InspectorFocus::Sketch { sketch_idx };
                        self.cached_frame_geo = None;
                    }
                }
            }
        }

        if self.active_setup_idx != prev_setup_idx && self.sim_panel.active_result.is_some() {
            needs_resim = true;
        }
        if capture.isolate_op {
            self.show_all_setup_toolpaths = false;
        }
        if let Some(z_planes) = capture.show_z_planes {
            self.show_z_planes = z_planes;
        }
        if let Some(dots) = capture.show_27pt_dots {
            self.show_27pt_dots = dots;
        }
        if let Some(ghost) = capture.ghost_part {
            self.highlight.ghost_part = ghost;
        }
        if let Some(show_stock) = capture.show_sim_stock {
            self.sim_panel.show_machined_stock = show_stock;
        }
        if let Some(show_cutter) = capture.show_sim_cutter {
            self.sim_panel.show_cutter_assembly = show_cutter;
        }
        if let Some(res) = &self.sim_panel.active_result {
            self.sim_panel.current_time_sec =
                res.timeline.total_time_sec * capture.effective_sim_progress();
        }
        if let Some(post_id) = &capture.post_profile
            && let Some(setup) = self.project.setups.get_mut(self.active_setup_idx)
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
                    .project_file_path
                    .as_deref()
                    .and_then(std::path::Path::parent)
                    .or(self.bundle_extract_dir.as_deref());
                self.gcode_panel.ensure_profiles_loaded(proj_dir);
            }
        }

        if let Some(modal) = capture.modal {
            self.welcome_modal.open = modal == CaptureModal::Welcome;
            self.sample_gallery_modal.open = modal == CaptureModal::SampleGallery;
            self.tool_library_modal.open = modal == CaptureModal::ToolLibrary;
            self.shortcut_cheatsheet.open = modal == CaptureModal::ShortcutCheatsheet;
            self.about_modal.open = modal == CaptureModal::About;
        }

        if let Some(view) = capture.view {
            match view {
                CaptureView::Standard(v) => {
                    self.camera.set_standard_view(v);
                    self.fit_scene_to_viewport();
                }
                CaptureView::SetupTop => self.align_camera_to_active_setup(),
                CaptureView::ActiveSketch => {
                    let sk_idx = match self.inspector_focus {
                        InspectorFocus::Sketch { sketch_idx } => sketch_idx,
                        _ => 0,
                    };
                    self.align_camera_to_sketch(sk_idx);
                }
            }
        }
        needs_resim
    }
}
