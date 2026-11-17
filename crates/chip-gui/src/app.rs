use std::path::PathBuf;

use chip_cad::{
    CadDocument, CadKernel, CadPicker, CurveKind, MonstertruckKernel, PickFilter, PickTarget,
    SurfaceKind, sample_machined_plate_step,
};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit, EvalContext};
use crossbeam_channel::{Receiver, Sender};

use crate::{
    camera::{CadCamera, MousePreset, OrbitMode, StandardView},
    layout::{WorkspacePane, create_default_workspace_tree},
    viewport::{HighlightState, ViewportRenderer, draw_view_cube_overlay},
    widgets::ExpressionInput,
};

enum LoadJobResult {
    Loaded {
        payload: Box<(CadDocument, CadPicker)>,
        elapsed_ms: u128,
    },
    Failed(String),
}

struct LoadedModelState {
    doc: CadDocument,
    picker: CadPicker,
    revision: u64,
}

/// Self-test automation configuration for headless CI screenshot capture.
pub struct SelfTestConfig {
    pub screenshot_path: PathBuf,
    frames_rendered: u32,
    requested: bool,
}

impl SelfTestConfig {
    pub fn new(screenshot_path: PathBuf) -> Self {
        Self {
            screenshot_path,
            frames_rendered: 0,
            requested: false,
        }
    }
}

/// Main ChipCutter desktop application state (`eframe` + `wgpu` + `egui_tiles`).
pub struct ChipCutterApp {
    tree: egui_tiles::Tree<WorkspacePane>,
    camera: CadCamera,
    viewport_renderer: Option<ViewportRenderer>,
    wgpu_render_state: Option<egui_wgpu::RenderState>,

    model: Option<LoadedModelState>,
    next_revision: u64,
    loading_in_flight: bool,
    job_tx: Sender<LoadJobResult>,
    job_rx: Receiver<LoadJobResult>,

    pick_filter: PickFilter,
    highlight: HighlightState,
    display_unit: DisplayUnit,

    // Parametric expression workbench demonstrating live mixed-unit formulas
    wb_tool_dia: ExpressionInput,
    wb_stepover: ExpressionInput,
    wb_clearance_z: ExpressionInput,

    logs: Vec<String>,
    self_test: Option<SelfTestConfig>,
}

impl ChipCutterApp {
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        initial_step: Option<PathBuf>,
        self_test: Option<SelfTestConfig>,
    ) -> Self {
        let mut visuals = egui::Visuals::dark();
        visuals.panel_fill = egui::Color32::from_rgb(24, 27, 32);
        visuals.window_fill = egui::Color32::from_rgb(28, 32, 38);
        cc.egui_ctx.set_visuals(visuals);

        let wgpu_render_state = cc.wgpu_render_state.clone();
        let viewport_renderer = wgpu_render_state
            .as_ref()
            .map(|rs| ViewportRenderer::new(&rs.device));

        let (job_tx, job_rx) = crossbeam_channel::unbounded();

        let mut app = Self {
            tree: create_default_workspace_tree(),
            camera: CadCamera::default(),
            viewport_renderer,
            wgpu_render_state,
            model: None,
            next_revision: 1,
            loading_in_flight: false,
            job_tx,
            job_rx,
            pick_filter: PickFilter::Any,
            highlight: HighlightState {
                hovered: None,
                selected: None,
                show_grid: true,
            },
            display_unit: DisplayUnit::Millimeter,
            wb_tool_dia: ExpressionInput::new(6.35, Some("1/4 in")),
            wb_stepover: ExpressionInput::new(2.54, Some("tool_d * 0.4")),
            wb_clearance_z: ExpressionInput::new(26.0, Some("model_top_z + 10 mm")),
            logs: vec![
                "ChipCutter v0.1.0 initialized (Milestone 1: STEP Kernel & wgpu CAD Viewport)."
                    .to_string(),
            ],
            self_test,
        };

        if let Some(path) = initial_step {
            app.spawn_step_file_load(path, cc.egui_ctx.clone());
        } else {
            // Load the built-in machined plate fixture immediately so the viewport is ready out-of-the-box!
            app.spawn_sample_plate_load(cc.egui_ctx.clone());
        }

        app
    }

    fn eval_context(&self) -> EvalContext {
        let (stock_x, stock_y, stock_z, top_z, bot_z) = match &self.model {
            Some(m) if m.doc.aabb_mm.is_valid() => {
                let sz = m.doc.aabb_mm.size();
                (sz.x, sz.y, sz.z, m.doc.aabb_mm.max.z, m.doc.aabb_mm.min.z)
            }
            _ => (80.0, 60.0, 16.0, 16.0, 0.0),
        };
        let tool_d = self.wb_tool_dia.value_mm();
        EvalContext::new(self.display_unit)
            .with_var("tool_d", tool_d)
            .with_var("tool_r", 0.5 * tool_d)
            .with_var("stock_x", stock_x)
            .with_var("stock_y", stock_y)
            .with_var("stock_z", stock_z)
            .with_var("model_top_z", top_z)
            .with_var("model_bottom_z", bot_z)
    }

    fn refresh_dependent_expressions(&mut self) {
        let ctx = self.eval_context();
        self.wb_stepover.refresh(&ctx);
        self.wb_clearance_z.refresh(&ctx);
    }

    fn spawn_step_file_load(&mut self, path: PathBuf, egui_ctx: egui::Context) {
        self.loading_in_flight = true;
        self.logs.push(format!(
            "Loading STEP file '{}' on background worker…",
            path.display()
        ));
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = std::time::Instant::now();
            let res = match std::fs::read(&path) {
                Ok(bytes) => {
                    let name = path
                        .file_name()
                        .and_then(|s| s.to_str())
                        .unwrap_or("part.step")
                        .to_string();
                    match MonstertruckKernel.load_step_bytes(
                        &name,
                        &bytes,
                        DEFAULT_TESS_CHORD_TOL_MM,
                    ) {
                        Ok(doc) => {
                            let picker = CadPicker::build(&doc);
                            LoadJobResult::Loaded {
                                payload: Box::new((doc, picker)),
                                elapsed_ms: t0.elapsed().as_millis(),
                            }
                        }
                        Err(e) => LoadJobResult::Failed(e.to_string()),
                    }
                }
                Err(e) => LoadJobResult::Failed(format!("failed to read {}: {e}", path.display())),
            };
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    fn spawn_sample_plate_load(&mut self, egui_ctx: egui::Context) {
        self.loading_in_flight = true;
        self.logs
            .push("Generating & tessellating sample CNC mounting plate STEP model…".to_string());
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = std::time::Instant::now();
            let res = match sample_machined_plate_step() {
                Ok(step_str) => match MonstertruckKernel.load_step_bytes(
                    "sample_machined_plate.step",
                    step_str.as_bytes(),
                    DEFAULT_TESS_CHORD_TOL_MM,
                ) {
                    Ok(doc) => {
                        let picker = CadPicker::build(&doc);
                        LoadJobResult::Loaded {
                            payload: Box::new((doc, picker)),
                            elapsed_ms: t0.elapsed().as_millis(),
                        }
                    }
                    Err(e) => LoadJobResult::Failed(e.to_string()),
                },
                Err(e) => LoadJobResult::Failed(e.to_string()),
            };
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    fn poll_background_jobs(&mut self) {
        while let Ok(msg) = self.job_rx.try_recv() {
            self.loading_in_flight = false;
            match msg {
                LoadJobResult::Loaded {
                    payload,
                    elapsed_ms,
                } => {
                    let (doc, picker) = *payload;
                    let size = doc.aabb_mm.size();
                    self.logs.push(format!(
                        "Loaded '{}' [{}] in {} ms — {} faces, {} loops, {} edges (X={:.2} Y={:.2} Z={:.2} mm).",
                        doc.source_name,
                        doc.detected_unit.label(),
                        elapsed_ms,
                        doc.faces.len(),
                        doc.loops.len(),
                        doc.edges.len(),
                        size.x,
                        size.y,
                        size.z,
                    ));
                    self.camera.fit_aabb(&doc.aabb_mm);
                    self.highlight.hovered = None;
                    // Default-select the first internal cylindrical hole (if present) so the Inspector showcases exact analytic B-Rep data immediately!
                    self.highlight.selected = doc
                        .faces
                        .iter()
                        .find(|f| {
                            matches!(
                                f.surface,
                                SurfaceKind::Cylinder {
                                    is_internal: true,
                                    ..
                                }
                            )
                        })
                        .map(|f| PickTarget::Face(f.id));

                    let rev = self.next_revision;
                    self.next_revision += 1;
                    self.model = Some(LoadedModelState {
                        doc,
                        picker,
                        revision: rev,
                    });
                    self.refresh_dependent_expressions();
                }
                LoadJobResult::Failed(err) => {
                    self.logs.push(format!("ERROR loading STEP model: {err}"));
                }
            }
        }
    }

    fn open_step_dialog(&mut self, ctx: &egui::Context) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("STEP 3D Models", &["step", "stp", "STEP", "STP"])
            .pick_file()
        {
            self.spawn_step_file_load(path, ctx.clone());
        }
    }

    fn draw_top_bars(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();

        // Keyboard shortcuts (when not typing inside a text input)
        if !ctx.egui_wants_keyboard_input() {
            if ctx.input(|i| i.key_pressed(egui::Key::P)) {
                self.camera.toggle_projection();
            }
            if ctx.input(|i| i.key_pressed(egui::Key::F))
                && let Some(m) = &self.model
            {
                self.camera.fit_aabb(&m.doc.aabb_mm);
            }
        }

        egui::Panel::top("chip_menu_bar").show(ui, |ui| {
            egui::MenuBar::new().ui(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("Open STEP Model…").clicked() {
                        ui.close();
                        self.open_step_dialog(&ctx);
                    }
                    if ui.button("Load Sample Machined Plate").clicked() {
                        ui.close();
                        self.spawn_sample_plate_load(ctx.clone());
                    }
                    ui.separator();
                    if ui.button("Quit").clicked() {
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                });

                ui.menu_button("View", |ui| {
                    if ui.button("Reset Default Workspace Layout").clicked() {
                        self.tree = create_default_workspace_tree();
                        ui.close();
                    }
                    ui.separator();
                    ui.checkbox(&mut self.highlight.show_grid, "Show WCS XY Grid");
                    if ui.button("Toggle Orthographic / Perspective (P)").clicked() {
                        self.camera.toggle_projection();
                        ui.close();
                    }
                    if ui.button("Fit Model to Viewport (F)").clicked() {
                        if let Some(m) = &self.model {
                            self.camera.fit_aabb(&m.doc.aabb_mm);
                        }
                        ui.close();
                    }
                    if ui.button("Align View to Setup Top (+Z)").clicked() {
                        self.camera.set_standard_view(StandardView::Top);
                        ui.close();
                    }
                });

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if self.loading_in_flight {
                        ui.label("Loading STEP…");
                        ui.spinner();
                    } else if let Some(m) = &self.model {
                        ui.weak(format!(
                            "{} | {} faces, {} edges",
                            m.doc.source_name,
                            m.doc.faces.len(),
                            m.doc.edges.len()
                        ));
                    }
                });
            });
        });

        egui::Panel::top("chip_toolbar").show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                if ui.button("Open STEP…").clicked() {
                    self.open_step_dialog(&ctx);
                }
                if ui.button("Sample Plate").clicked() {
                    self.spawn_sample_plate_load(ctx.clone());
                }

                ui.separator();
                ui.label("Pick:");
                egui::ComboBox::from_id_salt("pick_filter_combo")
                    .selected_text(self.pick_filter.label())
                    .show_ui(ui, |ui| {
                        for f in PickFilter::ALL {
                            ui.selectable_value(&mut self.pick_filter, f, f.label());
                        }
                    });

                ui.separator();
                let proj_label = if self.camera.projection.is_orthographic() {
                    "Ortho (P)"
                } else {
                    "Persp (P)"
                };
                if ui.button(proj_label).clicked() {
                    self.camera.toggle_projection();
                }
                if ui.button("Fit (F)").clicked()
                    && let Some(m) = &self.model
                {
                    self.camera.fit_aabb(&m.doc.aabb_mm);
                }
                if ui.button("Look at Top").clicked() {
                    self.camera.set_standard_view(StandardView::Top);
                }

                ui.separator();
                ui.label("Mouse:");
                egui::ComboBox::from_id_salt("mouse_preset_combo")
                    .selected_text(self.camera.mouse_preset.short_label())
                    .show_ui(ui, |ui| {
                        for preset in MousePreset::ALL {
                            ui.selectable_value(
                                &mut self.camera.mouse_preset,
                                preset,
                                preset.label(),
                            );
                        }
                    });

                let next_orbit = match self.camera.orbit_mode {
                    OrbitMode::TurntableZUp => OrbitMode::FreeTrackball,
                    OrbitMode::FreeTrackball => OrbitMode::TurntableZUp,
                };
                if ui.button(self.camera.orbit_mode.label()).clicked() {
                    self.camera.set_orbit_mode(next_orbit);
                }

                ui.separator();
                ui.label("Units:");
                let mut u_sel = self.display_unit;
                ui.selectable_value(&mut u_sel, DisplayUnit::Millimeter, "mm");
                ui.selectable_value(&mut u_sel, DisplayUnit::Inch, "in");
                if u_sel != self.display_unit {
                    self.display_unit = u_sel;
                    self.refresh_dependent_expressions();
                }
            });
        });
    }

    fn handle_self_test(&mut self, ctx: &egui::Context) {
        let Some(st) = &mut self.self_test else {
            return;
        };
        if self.loading_in_flight || self.model.is_none() {
            ctx.request_repaint();
            return;
        }
        st.frames_rendered += 1;
        if st.frames_rendered < 4 {
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

impl eframe::App for ChipCutterApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.poll_background_jobs();
        self.draw_top_bars(ui);

        let ctx = ui.ctx().clone();
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE.fill(egui::Color32::from_rgb(18, 20, 24)))
            .show(ui, |ui| {
                // Temporarily move `tree` out of `self` so `PaneBehavior` can mutably borrow the rest of `self` cleanly
                let mut tree = std::mem::replace(
                    &mut self.tree,
                    egui_tiles::Tree::empty(egui::Id::unique("placeholder")),
                );
                let mut behavior = PaneBehavior { app: self };
                tree.ui(&mut behavior, ui);
                self.tree = tree;
            });

        self.handle_self_test(&ctx);
    }
}

struct PaneBehavior<'a> {
    app: &'a mut ChipCutterApp,
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
            WorkspacePane::DiagnosticsLog => self.ui_diagnostics(ui),
        }
        egui_tiles::UiResponse::None
    }
}

impl PaneBehavior<'_> {
    fn ui_cam_browser(&mut self, ui: &mut egui::Ui) {
        egui::Frame::NONE.inner_margin(8.0).show(ui, |ui| {
            ui.heading("CAM Browser");
            ui.separator();
            egui::ScrollArea::vertical().show(ui, |ui| {
                let Some(m) = &self.app.model else {
                    ui.weak("No STEP model loaded.");
                    return;
                };
                let doc = &m.doc;
                let mut next_selection: Option<PickTarget> = None;

                egui::CollapsingHeader::new(format!("[STEP] {}", doc.source_name))
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.small(format!("STEP Unit: {}", doc.detected_unit.label()));
                        for body in &doc.bodies {
                            egui::CollapsingHeader::new(format!(
                                "{} ({} faces)",
                                body.name,
                                body.faces.len()
                            ))
                            .default_open(true)
                            .show(ui, |ui| {
                                for &fid in &body.faces {
                                    if let Some(face) = doc.face(fid) {
                                        let is_sel = self.app.highlight.selected
                                            == Some(PickTarget::Face(fid));
                                        let label = format!(
                                            "Face #{} — {}",
                                            fid.0,
                                            face.surface.short_label()
                                        );
                                        let resp = ui.selectable_label(is_sel, label);
                                        if resp.hovered() {
                                            self.app.highlight.hovered =
                                                Some(PickTarget::Face(fid));
                                        }
                                        if resp.clicked() {
                                            next_selection = Some(PickTarget::Face(fid));
                                        }
                                    }
                                }
                            });
                        }
                    });

                if let Some(sel) = next_selection {
                    self.app.highlight.selected = Some(sel);
                }

                ui.add_space(10.0);
                ui.separator();
                ui.weak("Setups & 2.5D Operations land in Milestones 2–3.");
            });
        });
    }

    fn ui_viewport_3d(&mut self, ui: &mut egui::Ui) {
        let avail = ui.available_rect_before_wrap();
        let response = ui.allocate_rect(avail, egui::Sense::click_and_drag());

        self.app.camera.handle_viewport_input(ui, &response);

        // Raycast hover and click picking in exact f64 millimeters
        if let Some(m) = &self.app.model {
            if let Some(cursor_pos) = response.hover_pos() {
                if let Some(ray) = self.app.camera.screen_ray(cursor_pos, avail) {
                    let mm_per_px = self.app.camera.mm_per_pixel(avail.height());
                    let hit = m.picker.pick(&m.doc, &ray, mm_per_px, self.app.pick_filter);
                    self.app.highlight.hovered = hit.map(|h| h.target);
                }
            } else if !ui.rect_contains_pointer(avail) {
                // Keep tree-hovered highlight if cursor isn't inside viewport
            }

            if response.clicked_by(egui::PointerButton::Primary) {
                self.app.highlight.selected = self.app.highlight.hovered.clone();
            }
        }

        // Render 4x MSAA 3D viewport texture and paint into `avail`
        let ppp = ui.ctx().pixels_per_point();
        let width_px = ((avail.width() * ppp).round() as u32).max(2);
        let height_px = ((avail.height() * ppp).round() as u32).max(2);

        if let (Some(vp), Some(rs)) = (
            self.app.viewport_renderer.as_mut(),
            self.app.wgpu_render_state.as_ref(),
        ) {
            let doc_ref = self.app.model.as_ref().map(|m| (&m.doc, m.revision));
            let tex_id = vp.render_for_egui(
                rs,
                [width_px, height_px],
                &self.app.camera,
                doc_ref,
                &self.app.highlight,
                self.app.display_unit,
            );
            ui.painter().image(
                tex_id,
                avail,
                egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                egui::Color32::WHITE,
            );
        }

        draw_view_cube_overlay(ui, avail, &mut self.app.camera);

        // Subtle HUD status pill in bottom-left of viewport
        let hud_pos = egui::pos2(avail.left() + 10.0, avail.bottom() - 24.0);
        ui.painter().text(
            hud_pos,
            egui::Align2::LEFT_CENTER,
            format!(
                "{} | {} | Units: {}",
                if self.app.camera.projection.is_orthographic() {
                    "Orthographic"
                } else {
                    "Perspective"
                },
                self.app.camera.mouse_preset.short_label(),
                self.app.display_unit.suffix()
            ),
            egui::FontId::proportional(11.5),
            egui::Color32::from_rgba_unmultiplied(210, 220, 232, 195),
        );
    }

    fn ui_inspector(&mut self, ui: &mut egui::Ui) {
        let u = self.app.display_unit;
        egui::Frame::NONE.inner_margin(8.0).show(ui, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.heading("Selection Inspector");
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
                                            ui.label("Inferred Z Height:");
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
                                            ui.label("Radius:");
                                            ui.monospace(u.format_length(radius_mm));
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
                                            ui.label("Z Span (Top / Bot):");
                                            ui.monospace(format!(
                                                "{} -> {}",
                                                u.format_length(face.aabb_mm.max.z),
                                                u.format_length(face.aabb_mm.min.z)
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
                                    ui.label("Inner Hole Loops:");
                                    ui.monospace(format!("{}", face.inner_loops.len()));
                                    ui.end_row();
                                });
                        }
                    }
                    (Some(m), Some(PickTarget::Edge(eid))) => {
                        if let Some(edge) = m.doc.edge(*eid) {
                            ui.strong(format!("Edge #{} — {}", eid.0, edge.curve.short_label()));
                            ui.add_space(4.0);
                            egui::Grid::new("edge_props_grid")
                                .num_columns(2)
                                .spacing([12.0, 4.0])
                                .show(ui, |ui| {
                                    ui.label("Length:");
                                    ui.monospace(u.format_length(edge.length_mm));
                                    ui.end_row();
                                    if let CurveKind::CircleArc {
                                        center_mm,
                                        radius_mm,
                                        ..
                                    } = edge.curve
                                    {
                                        ui.label("Arc Radius / Ø:");
                                        ui.monospace(format!(
                                            "R {} / Ø {}",
                                            u.format_length(radius_mm),
                                            u.format_length(radius_mm * 2.0)
                                        ));
                                        ui.end_row();
                                        ui.label("Center:");
                                        ui.monospace(format!(
                                            "({}, {}, {})",
                                            u.format_length(center_mm.x),
                                            u.format_length(center_mm.y),
                                            u.format_length(center_mm.z)
                                        ));
                                        ui.end_row();
                                    }
                                    ui.label("Adjacent Faces:");
                                    ui.monospace(format!("{}", edge.adjacent_faces.len()));
                                    ui.end_row();
                                });
                        }
                    }
                    (
                        Some(m),
                        Some(PickTarget::Loop {
                            loop_id, edges, ..
                        }),
                    ) => {
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
                            ui.label(format!(
                                "Role: {}",
                                if lp.is_outer {
                                    "Outer Boundary"
                                } else {
                                    "Inner Island / Hole"
                                }
                            ));
                        }
                    }
                    _ => {
                        ui.weak("Click any planar face, edge, loop, or cylindrical bore in the 3D viewport.");
                    }
                }

                ui.add_space(14.0);
                ui.heading("Parametric Expression Preview");
                ui.small("Supports mixed units (`1/4 in`, `10 thou`) & variables (`tool_d`, `model_top_z`).");
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
        });
    }

    fn ui_diagnostics(&mut self, ui: &mut egui::Ui) {
        egui::Frame::NONE.inner_margin(6.0).show(ui, |ui| {
            egui::ScrollArea::vertical()
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    for entry in &self.app.logs {
                        ui.monospace(entry);
                    }
                });
        });
    }
}
