pub mod loader;
pub mod menus_toolbar;
pub mod pane_behavior;
pub mod viewport_controller;

use std::{
    hash::{Hash as _, Hasher as _},
    path::{Path, PathBuf},
    sync::Arc,
    time::Instant,
};

use chip_cad::{CadDocument, CadPicker, PickFilter, TopologyGraph};
use chip_cam::{
    ExtractedSelectionSummary, OperationStatus, ResolvedSetup,
    extract_selection_features_with_sketches,
};
use chip_math::{Aabb3, DVec3, EvalContext};
use chip_project::{JobDispatcher, ProjectDocument, ToolpathCache, UndoHistory, is_chipz_path};
use chip_sim::SimDiagnostic;
use chip_tools::{ToolLibrary, load_or_default_global_library};
use crossbeam_channel::{Receiver, Sender};

use crate::{
    app::{loader::LoadJobResult, pane_behavior::PaneBehavior},
    camera::CadCamera,
    layout::{WorkspacePane, create_default_workspace_tree, focus_workspace_pane},
    panels::{
        InspectorFocus, OperationTab, SetupTab, ViewportPickMode,
        gcode_preview::GCodePreviewPanelState,
        simulation::{SimTriggerArgs, SimulationPanelState},
        tool_library::ToolLibraryModalState,
        vector_import_modal::{VectorImportModalState, VectorModalAction, VectorModalCtx},
    },
    theme::{AppTheme, ThemeTokens, apply_theme},
    viewport::{HighlightState, ViewportRenderer},
    widgets::ExpressionInput,
};

pub(crate) struct LoadedModelState {
    pub doc: Arc<CadDocument>,
    pub picker: CadPicker,
    pub topo: TopologyGraph,
    pub raw_step_bytes: Vec<u8>,
    pub source_path: Option<PathBuf>,
    pub revision: u64,
}

/// Cached Setup WCS + focused Operation 2.5D selection extraction so idle 60 FPS frames perform
/// zero redundant B-Rep loop chaining or stock/WCS recomputations.
#[derive(Clone)]
pub(crate) struct CachedFrameGeo {
    pub sig: u64,
    pub resolved_setup: Option<Result<ResolvedSetup, String>>,
    pub focused_extracted: Option<Result<ExtractedSelectionSummary, String>>,
}

/// Optional startup demo mode for CLI flags (`--demo-pure-2d`, `--demo-nameplate`, `--demo-vcarve`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum StartupDemoMode {
    #[default]
    None,
    Pure2dPanel,
    EngravedNameplate,
    VCarveSign,
}

/// Self-test automation configuration for headless CI screenshot capture.
pub struct SelfTestConfig {
    pub screenshot_path: PathBuf,
    pub(crate) frames_rendered: u32,
    pub(crate) requested: bool,
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
    pub(crate) tree: egui_tiles::Tree<WorkspacePane>,
    pub(crate) theme: AppTheme,
    pub(crate) camera: CadCamera,
    pub(crate) viewport_renderer: Option<ViewportRenderer>,
    pub(crate) wgpu_render_state: Option<egui_wgpu::RenderState>,

    pub(crate) project: ProjectDocument,
    pub(crate) project_file_path: Option<PathBuf>,
    pub(crate) bundle_extract_dir: Option<PathBuf>,
    pub(crate) clean_undo_len: Option<usize>,
    pub(crate) last_window_title: String,

    pub(crate) history: UndoHistory,
    pub(crate) toolpath_cache: ToolpathCache,
    pub(crate) dispatcher: JobDispatcher,
    pub(crate) global_tools: ToolLibrary,
    pub(crate) tool_library_modal: ToolLibraryModalState,
    pub(crate) vector_import_modal: VectorImportModalState,
    pub(crate) sim_panel: SimulationPanelState,
    pub(crate) gcode_panel: GCodePreviewPanelState,
    pub(crate) auto_sim_after_gen: bool,

    pub(crate) active_setup_idx: usize,
    pub(crate) inspector_focus: InspectorFocus,
    pub(crate) setup_tab: SetupTab,
    pub(crate) op_tab: OperationTab,
    pub(crate) pick_mode: ViewportPickMode,
    pub(crate) selected_sketch_entity_idx: Option<usize>,
    pub(crate) sketch_snap_grid_mm: Option<f64>,
    pub(crate) sketch_author_start_xy: Option<chip_math::DVec2>,
    pub(crate) lock_chipload: bool,
    pub(crate) show_27pt_dots: bool,
    pub(crate) show_z_planes: bool,
    pub(crate) show_all_setup_toolpaths: bool,

    pub(crate) model: Option<LoadedModelState>,
    pub(crate) empty_cad: Arc<CadDocument>,
    pub(crate) next_revision: u64,
    pub(crate) loading_in_flight: bool,
    pub(crate) job_tx: Sender<LoadJobResult>,
    pub(crate) job_rx: Receiver<LoadJobResult>,

    pub(crate) last_disk_check: Instant,
    pub(crate) external_step_modified: bool,
    pub(crate) last_reload_banner: Option<(String, bool)>,
    pub(crate) cached_frame_geo: Option<CachedFrameGeo>,

    pub(crate) pick_filter: PickFilter,
    pub(crate) highlight: HighlightState,

    pub(crate) wb_tool_dia: ExpressionInput,
    pub(crate) wb_stepover: ExpressionInput,
    pub(crate) wb_clearance_z: ExpressionInput,

    pub(crate) logs: Vec<String>,
    pub(crate) self_test: Option<SelfTestConfig>,
}

impl Drop for ChipCutterApp {
    fn drop(&mut self) {
        if let Some(dir) = self.bundle_extract_dir.take() {
            let _ = std::fs::remove_dir_all(dir);
        }
    }
}

impl ChipCutterApp {
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        initial_path: Option<PathBuf>,
        self_test: Option<SelfTestConfig>,
    ) -> Self {
        Self::new_with_demo(cc, initial_path, StartupDemoMode::None, self_test)
    }

    pub fn new_with_mode(
        cc: &eframe::CreationContext<'_>,
        initial_path: Option<PathBuf>,
        demo_pure_2d: bool,
        self_test: Option<SelfTestConfig>,
    ) -> Self {
        let mode = if demo_pure_2d {
            StartupDemoMode::Pure2dPanel
        } else {
            StartupDemoMode::None
        };
        Self::new_with_demo(cc, initial_path, mode, self_test)
    }

    pub fn new_with_demo(
        cc: &eframe::CreationContext<'_>,
        initial_path: Option<PathBuf>,
        demo_mode: StartupDemoMode,
        self_test: Option<SelfTestConfig>,
    ) -> Self {
        Self::new_with_demo_and_theme(cc, initial_path, demo_mode, AppTheme::Dark, self_test)
    }

    pub fn new_with_demo_and_theme(
        cc: &eframe::CreationContext<'_>,
        initial_path: Option<PathBuf>,
        demo_mode: StartupDemoMode,
        initial_theme: AppTheme,
        self_test: Option<SelfTestConfig>,
    ) -> Self {
        apply_theme(&cc.egui_ctx, initial_theme);

        let wgpu_render_state = cc.wgpu_render_state.clone();
        let viewport_renderer = wgpu_render_state
            .as_ref()
            .map(|rs| ViewportRenderer::new(&rs.device));

        let (job_tx, job_rx) = crossbeam_channel::unbounded();
        let global_tools = load_or_default_global_library();
        let repaint_ctx = cc.egui_ctx.clone();
        let dispatcher = JobDispatcher::new(Some(Arc::new(move || {
            repaint_ctx.request_repaint();
        })));

        let mut app = Self {
            tree: create_default_workspace_tree(),
            theme: initial_theme,
            camera: CadCamera::default(),
            viewport_renderer,
            wgpu_render_state,
            project: ProjectDocument::new("CAM Torture Block Job"),
            project_file_path: None,
            bundle_extract_dir: None,
            clean_undo_len: Some(0),
            last_window_title: String::new(),
            history: UndoHistory::new(100),
            toolpath_cache: ToolpathCache::new(128),
            dispatcher,
            global_tools,
            tool_library_modal: ToolLibraryModalState::new(),
            vector_import_modal: VectorImportModalState::default(),
            sim_panel: SimulationPanelState::default(),
            gcode_panel: GCodePreviewPanelState::default(),
            auto_sim_after_gen: true,
            active_setup_idx: 0,
            inspector_focus: InspectorFocus::Setup { setup_idx: 0 },
            setup_tab: SetupTab::Wcs,
            op_tab: OperationTab::Passes,
            pick_mode: ViewportPickMode::InspectEntity,
            selected_sketch_entity_idx: None,
            sketch_snap_grid_mm: None,
            sketch_author_start_xy: None,
            lock_chipload: true,
            show_27pt_dots: true,
            show_z_planes: true,
            show_all_setup_toolpaths: true,
            model: None,
            empty_cad: Arc::new(CadDocument::empty()),
            next_revision: 1,
            loading_in_flight: false,
            job_tx,
            job_rx,
            last_disk_check: Instant::now(),
            external_step_modified: false,
            last_reload_banner: None,
            cached_frame_geo: None,
            pick_filter: PickFilter::Any,
            highlight: HighlightState {
                theme: initial_theme,
                ..HighlightState::default()
            },
            wb_tool_dia: ExpressionInput::new(6.35, Some("1/4 in")),
            wb_stepover: ExpressionInput::new(2.54, Some("tool_d * 0.4")),
            wb_clearance_z: ExpressionInput::new(26.0, Some("model_top_z + 10 mm")),
            logs: vec![
                "ChipCutter v0.2.0 initialized (v2 Complete: 2D Adaptive & Rest, DXF/SVG & Sketches, CAM Typography, Two-Tool V-Carving & V-Inlays)."
                    .to_string(),
            ],
            self_test,
        };

        if let Some(path) = initial_path {
            if is_chipz_path(&path) {
                app.load_chipz_from_disk(&path, cc.egui_ctx.clone());
            } else if path
                .to_string_lossy()
                .to_ascii_lowercase()
                .ends_with(".toml")
            {
                app.load_project_from_disk(&path, cc.egui_ctx.clone());
            } else {
                let ext_low = path
                    .extension()
                    .and_then(|s| s.to_str())
                    .unwrap_or_default()
                    .to_ascii_lowercase();
                if ext_low == "dxf" || ext_low == "svg" {
                    app.vector_import_modal.load_from_disk_path(path);
                    app.vector_import_modal.open = true;
                } else {
                    app.spawn_step_file_load(path, cc.egui_ctx.clone());
                }
            }
        } else {
            match demo_mode {
                StartupDemoMode::Pure2dPanel => app.load_pure2d_router_panel_demo(&cc.egui_ctx),
                StartupDemoMode::EngravedNameplate => {
                    app.load_engraved_nameplate_demo(&cc.egui_ctx)
                }
                StartupDemoMode::VCarveSign => app.load_vcarve_sign_demo(&cc.egui_ctx),
                StartupDemoMode::None => {
                    app.spawn_cam_torture_sample_load(true, cc.egui_ctx.clone())
                }
            }
        }

        app
    }

    pub fn set_theme(&mut self, ctx: &egui::Context, theme: AppTheme) {
        self.theme = theme;
        self.highlight.theme = theme;
        apply_theme(ctx, theme);
        ctx.request_repaint();
    }

    pub(crate) fn active_project_dir(&self) -> Option<&Path> {
        self.project_file_path
            .as_deref()
            .and_then(Path::parent)
            .or(self.bundle_extract_dir.as_deref())
    }

    pub(crate) fn virtual_project_file_path(&self) -> Option<PathBuf> {
        self.project_file_path.clone().or_else(|| {
            self.bundle_extract_dir
                .as_ref()
                .map(|d| d.join("project.chip.toml"))
        })
    }

    pub(crate) fn clear_bundle_extract_dir(&mut self) {
        if let Some(dir) = self.bundle_extract_dir.take() {
            let _ = std::fs::remove_dir_all(dir);
        }
    }

    pub(crate) fn eval_context(&self) -> EvalContext {
        let (stock_x, stock_y, stock_z, top_z, bot_z) = match &self.model {
            Some(m) if m.doc.aabb_mm.is_valid() => {
                let sz = m.doc.aabb_mm.size();
                (sz.x, sz.y, sz.z, m.doc.aabb_mm.max.z, m.doc.aabb_mm.min.z)
            }
            _ => (90.0, 70.0, 20.0, 20.0, 0.0),
        };
        let tool_d = self.wb_tool_dia.value_mm();
        EvalContext::new(self.project.display_units)
            .with_var("tool_d", tool_d)
            .with_var("tool_r", 0.5 * tool_d)
            .with_var("stock_x", stock_x)
            .with_var("stock_y", stock_y)
            .with_var("stock_z", stock_z)
            .with_var("model_top_z", top_z)
            .with_var("model_bottom_z", bot_z)
    }

    pub(crate) fn refresh_dependent_expressions(&mut self) {
        let ctx = self.eval_context();
        self.wb_stepover.refresh(&ctx);
        self.wb_clearance_z.refresh(&ctx);
    }

    pub(crate) fn has_geometry(&self) -> bool {
        self.model.is_some() || !self.project.sketches.is_empty()
    }

    /// Returns `(Arc<CadDocument>, cad_revision)` for either the loaded 3D STEP model OR Pure-2D (`empty_cad`).
    pub(crate) fn effective_cad_arc(&self) -> Option<(Arc<CadDocument>, u64)> {
        if let Some(m) = &self.model {
            Some((Arc::clone(&m.doc), m.revision))
        } else if !self.project.sketches.is_empty() {
            Some((Arc::clone(&self.empty_cad), 0))
        } else {
            None
        }
    }

    pub(crate) fn fit_scene_to_viewport(&mut self) {
        if let Some(m) = &self.model {
            self.camera.fit_aabb(&m.doc.aabb_mm);
        } else if let Some(Ok(rs)) = self.active_resolved_setup() {
            let b = rs.stock.aabb_wcs();
            let mut world_aabb = Aabb3::EMPTY;
            for x in [b.min.x, b.max.x] {
                for y in [b.min.y, b.max.y] {
                    for z in [b.min.z, b.max.z] {
                        world_aabb.extend_point(rs.frame.local_to_world_point(DVec3::new(x, y, z)));
                    }
                }
            }
            if world_aabb.is_valid() {
                self.camera.fit_aabb(&world_aabb);
            }
        }
    }

    fn compute_frame_geo_signature(&self) -> u64 {
        let mut h = std::collections::hash_map::DefaultHasher::new();
        self.model.as_ref().map_or(0, |m| m.revision).hash(&mut h);
        self.active_setup_idx.hash(&mut h);
        format!("{:?}", self.project.display_units).hash(&mut h);
        format!("{:?}", self.inspector_focus).hash(&mut h);
        for sk in &self.project.sketches {
            sk.id.hash(&mut h);
            sk.visible.hash(&mut h);
            format!("{:?}", sk.anchor).hash(&mut h);
            sk.entities.len().hash(&mut h);
            sk.regions.len().hash(&mut h);
        }
        if let Some(setup) = self.project.setups.get(self.active_setup_idx) {
            format!("{:?}", setup.wcs).hash(&mut h);
            format!("{:?}", setup.stock).hash(&mut h);
            format!("{:?}", setup.heights).hash(&mut h);
        }
        if let InspectorFocus::Operation { setup_idx, op_idx } = self.inspector_focus
            && let Some(setup) = self.project.setups.get(setup_idx)
            && let Some(op) = setup.operations.get(op_idx)
        {
            format!("{:?}", op.selections).hash(&mut h);
            matches!(op.runtime_status, OperationStatus::NeedsRepick { .. }).hash(&mut h);
        }
        h.finish()
    }

    pub(crate) fn ensure_frame_geo_cached(&mut self) -> &CachedFrameGeo {
        let sig = self.compute_frame_geo_signature();
        if self.cached_frame_geo.as_ref().map(|c| c.sig) != Some(sig) {
            let eff_cad = self.effective_cad_arc();
            let resolved_setup = match (
                eff_cad.as_ref(),
                self.project.setups.get(self.active_setup_idx),
            ) {
                (Some((doc, _)), Some(setup)) => Some(
                    setup
                        .resolve_with_sketches(
                            doc,
                            &self.project.sketches,
                            self.project.display_units,
                        )
                        .map_err(|e| e.to_string()),
                ),
                _ => None,
            };

            let focused_extracted = match (eff_cad.as_ref(), &resolved_setup, self.inspector_focus)
            {
                (Some((doc, _)), Some(Ok(rs)), InspectorFocus::Operation { setup_idx, op_idx }) => {
                    if let Some(setup) = self.project.setups.get(setup_idx)
                        && let Some(op) = setup.operations.get(op_idx)
                        && !matches!(op.runtime_status, OperationStatus::NeedsRepick { .. })
                    {
                        Some(
                            extract_selection_features_with_sketches(
                                doc,
                                &self.project.sketches,
                                rs,
                                &op.selections,
                            )
                            .map_err(|e| e.to_string()),
                        )
                    } else {
                        None
                    }
                }
                _ => None,
            };

            self.cached_frame_geo = Some(CachedFrameGeo {
                sig,
                resolved_setup,
                focused_extracted,
            });
        }
        // Safe: populated immediately above when None or stale
        self.cached_frame_geo
            .as_ref()
            .unwrap_or_else(|| unreachable!())
    }

    pub(crate) fn active_resolved_setup(&mut self) -> Option<Result<ResolvedSetup, String>> {
        self.ensure_frame_geo_cached().resolved_setup.clone()
    }

    pub(crate) fn align_camera_to_active_setup(&mut self) {
        if let Some(Ok(rs)) = self.active_resolved_setup() {
            self.camera
                .align_to_setup_top(&rs.frame, &rs.stock.aabb_wcs());
        }
    }

    pub(crate) fn align_camera_to_sketch(&mut self, sketch_idx: usize) {
        let Some(Ok(rs)) = self.active_resolved_setup() else {
            return;
        };
        let Some(sk) = self.project.sketches.get(sketch_idx) else {
            return;
        };
        let wf = if let Some(m) = &self.model {
            chip_cam::resolve_sketch_world_frame_with_doc(&m.doc, sk, &rs)
        } else {
            chip_cam::resolve_sketch_world_frame(false, sk, &rs)
        };
        let mut bounds_2d = sk.exact_aabb();
        if !bounds_2d.is_valid() || bounds_2d.size().length() < 1.0 {
            let st = rs.stock.aabb_wcs();
            if st.is_valid() {
                bounds_2d = chip_math::Aabb2::from_points([
                    chip_math::DVec2::new(st.min.x, st.min.y),
                    chip_math::DVec2::new(st.max.x, st.max.y),
                ]);
            }
        }
        self.camera.align_to_sketch_plane(&wf, &bounds_2d);
    }

    pub(crate) fn create_new_setup_xy_sketch(&mut self) {
        self.history
            .record_before_edit("New 2D Sketch", None, &self.project);
        let setup_id = self
            .project
            .setups
            .get(self.active_setup_idx)
            .map_or_else(|| "setup_1".to_string(), |s| s.id.clone());
        let n = self.project.sketches.len() + 1;
        let sk = chip_sketch::SketchLayer::new(
            format!("sketch_{n}"),
            format!("Sketch {n} (Setup XY)"),
            setup_id,
        );
        self.project.sketches.push(sk);
        let sk_idx = self.project.sketches.len() - 1;
        self.inspector_focus = InspectorFocus::Sketch { sketch_idx: sk_idx };
        self.selected_sketch_entity_idx = None;
        self.pick_mode = ViewportPickMode::InspectEntity;
        self.cached_frame_geo = None;
    }

    pub(crate) fn trigger_active_setup_simulation(&mut self, egui_ctx: &egui::Context) {
        let Some((cad_doc, cad_rev)) = self.effective_cad_arc() else {
            self.logs
                .push("Cannot simulate: load a STEP model or 2D Sketch first.".to_string());
            return;
        };
        let proj_dir = self
            .project_file_path
            .as_deref()
            .and_then(Path::parent)
            .or(self.bundle_extract_dir.as_deref());
        self.sim_panel.trigger_simulation(SimTriggerArgs {
            project: &self.project,
            setup_idx: self.active_setup_idx,
            cad_doc,
            cad_revision: cad_rev,
            cache: &self.toolpath_cache,
            project_dir: proj_dir,
            egui_ctx: egui_ctx.clone(),
            logs: &mut self.logs,
        });
    }

    pub(crate) fn jump_to_sim_diagnostic(&mut self, diag: &SimDiagnostic) {
        self.sim_panel.jump_to_time(diag.timeline_time_sec);
        focus_workspace_pane(&mut self.tree, WorkspacePane::SimulationTimeline);
        if let Some(Ok(rs)) = self.active_resolved_setup() {
            self.camera.target_mm = rs.frame.local_to_world_point(diag.location_wcs);
        }
    }

    pub(crate) fn trigger_generate_dirty(
        &mut self,
        only_setup_idx: Option<usize>,
        only_op_id: Option<&str>,
    ) {
        let Some((cad_doc, cad_rev)) = self.effective_cad_arc() else {
            self.logs.push(
                "Cannot generate toolpaths: load a STEP model or 2D Sketch first.".to_string(),
            );
            return;
        };
        let spawned = self.dispatcher.dispatch_operations(
            &mut self.project,
            cad_doc,
            cad_rev,
            &self.toolpath_cache,
            only_setup_idx,
            only_op_id,
        );
        if spawned > 0 {
            self.logs.push(format!(
                "Dispatched {spawned} operation(s) onto background rayon pool…"
            ));
        } else {
            self.logs
                .push("All enabled operations are already up-to-date (Clean).".to_string());
        }
    }

    pub(crate) fn perform_undo(&mut self) {
        if let Some(lbl) = self.history.undo(&mut self.project) {
            if let Some((_, cad_rev)) = self.effective_cad_arc() {
                self.dispatcher.cancel_stale_jobs(&self.project, cad_rev);
                let inflight = self.dispatcher.in_flight_ids();
                self.toolpath_cache.reconcile_project_statuses(
                    &mut self.project,
                    cad_rev,
                    &inflight,
                );
            }
            self.cached_frame_geo = None;
            self.logs.push(format!("Undo: {lbl}"));
        }
    }

    pub(crate) fn perform_redo(&mut self) {
        if let Some(lbl) = self.history.redo(&mut self.project) {
            if let Some((_, cad_rev)) = self.effective_cad_arc() {
                self.dispatcher.cancel_stale_jobs(&self.project, cad_rev);
                let inflight = self.dispatcher.in_flight_ids();
                self.toolpath_cache.reconcile_project_statuses(
                    &mut self.project,
                    cad_rev,
                    &inflight,
                );
            }
            self.cached_frame_geo = None;
            self.logs.push(format!("Redo: {lbl}"));
        }
    }
}

impl eframe::App for ChipCutterApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.poll_background_jobs(&ctx);
        self.sim_panel.update_playback_and_shortcuts(&ctx);
        self.sync_viewport_scene_state();
        self.draw_top_bars(ui);

        self.tool_library_modal.show(
            &ctx,
            &mut self.global_tools,
            &mut self.project,
            &mut self.history,
            &mut self.logs,
        );

        let proj_path_clone = self.project_file_path.clone();
        if let Some(vec_act) = self.vector_import_modal.show(
            &ctx,
            VectorModalCtx {
                project: &mut self.project,
                project_file_path: proj_path_clone.as_deref(),
                active_setup_idx: self.active_setup_idx,
                global_tools: &self.global_tools,
                history: &mut self.history,
                logs: &mut self.logs,
            },
        ) {
            match vec_act {
                VectorModalAction::AppendedSketch { .. } => {
                    self.cached_frame_geo = None;
                    if self.model.is_none() {
                        self.fit_scene_to_viewport();
                    }
                }
                VectorModalAction::CreatedPure2dJob { .. } => {
                    self.dispatcher.cancel_all();
                    self.sim_panel.cancel_in_flight();
                    self.sim_panel.active_result = None;
                    self.gcode_panel.invalidate_preview();
                    self.clear_bundle_extract_dir();
                    self.project_file_path = None;
                    self.history.clear();
                    self.clean_undo_len = Some(0);
                    self.model = None;
                    self.active_setup_idx = 0;
                    self.inspector_focus = InspectorFocus::Sketch { sketch_idx: 0 };
                    self.cached_frame_geo = None;
                    self.fit_scene_to_viewport();
                }
            }
        }

        let tok = ThemeTokens::for_theme(self.theme);
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE.fill(tok.app_bg))
            .show(ui, |ui| {
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
