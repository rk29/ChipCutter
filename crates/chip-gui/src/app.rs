use std::{
    path::{Path, PathBuf},
    sync::Arc,
};

use chip_cad::{
    CadDocument, CadKernel, CadPicker, CurveKind, MonstertruckKernel, PickFilter, PickTarget,
    SurfaceKind, TopologyGraph, find_same_diameter_holes, sample_cam_torture_block_step,
    sample_machined_plate_step, sample_multi_feature_bracket_step,
};
use chip_cam::{
    ChamferSourceMode, ChamferTraceMode, ExtractedFeature25D, GeometrySelectionItem,
    HeightReference, HeightSpec, HoleMachiningMode, ManualTabAnchor, Operation, OperationKind,
    ResolvedHeights, ResolvedSetup, Setup, StrategyParams, TabMarker3D, TabProfileKind, WcsAxisRef,
    WcsFeaturePointRef, WcsOrientationMode, WcsOriginMode, WorkOffsetSlot,
    extract_selection_features,
};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec2, DisplayUnit, EvalContext, ParamLength};
use chip_project::{
    JobDispatcher, ModelEntry, ProjectDocument, ToolpathCache, UndoHistory, compute_sha256_hex,
    portable_model_path, resolve_model_path, validate_project,
};
use chip_sim::SimDiagnostic;
use chip_tools::{ToolLibrary, load_or_default_global_library};
use crossbeam_channel::{Receiver, Sender};

use crate::{
    camera::{CadCamera, MousePreset, OrbitMode},
    layout::{WorkspacePane, create_default_workspace_tree, focus_workspace_pane},
    panels::{
        InspectorFocus, OperationTab, SetupTab, ViewportPickMode,
        browser::{BrowserAction, default_pick_mode_for_op, show_cam_browser},
        diagnostics::show_diagnostics_panel,
        gcode_preview::{GCodePreviewPanelState, GCodeUiAction},
        operation_inspector::show_operation_inspector,
        setup_inspector::show_setup_inspector,
        simulation::{SimulationPanelState, SimulationUiAction},
        tool_library::ToolLibraryModalState,
    },
    viewport::{
        HighlightState, ToolpathRenderEntry, ViewportRenderer, ZPlaneVisual,
        draw_view_cube_overlay, draw_wcs_origin_27pt_overlay,
    },
    widgets::ExpressionInput,
};

#[derive(Clone, Copy, PartialEq, Eq)]
enum SeedDemoKind {
    None,
    MultiFeatureBracket,
    CamTortureBlock,
}

struct LoadedPayload {
    doc: CadDocument,
    picker: CadPicker,
    topo: TopologyGraph,
    source_path: Option<PathBuf>,
    sha256_hex: String,
    seed_demo: SeedDemoKind,
}

enum LoadJobResult {
    Loaded {
        payload: Box<LoadedPayload>,
        elapsed_ms: u128,
    },
    Failed(String),
}

struct LoadedModelState {
    doc: Arc<CadDocument>,
    picker: CadPicker,
    topo: TopologyGraph,
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

    project: ProjectDocument,
    project_file_path: Option<PathBuf>,
    history: UndoHistory,
    toolpath_cache: ToolpathCache,
    dispatcher: JobDispatcher,
    global_tools: ToolLibrary,
    tool_library_modal: ToolLibraryModalState,
    sim_panel: SimulationPanelState,
    gcode_panel: GCodePreviewPanelState,
    auto_sim_after_gen: bool,

    active_setup_idx: usize,
    inspector_focus: InspectorFocus,
    setup_tab: SetupTab,
    op_tab: OperationTab,
    pick_mode: ViewportPickMode,
    lock_chipload: bool,
    show_27pt_dots: bool,
    show_z_planes: bool,
    show_all_setup_toolpaths: bool,

    model: Option<LoadedModelState>,
    next_revision: u64,
    loading_in_flight: bool,
    job_tx: Sender<LoadJobResult>,
    job_rx: Receiver<LoadJobResult>,

    pick_filter: PickFilter,
    highlight: HighlightState,

    wb_tool_dia: ExpressionInput,
    wb_stepover: ExpressionInput,
    wb_clearance_z: ExpressionInput,

    logs: Vec<String>,
    self_test: Option<SelfTestConfig>,
}

impl ChipCutterApp {
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        initial_path: Option<PathBuf>,
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
        let global_tools = load_or_default_global_library();
        let repaint_ctx = cc.egui_ctx.clone();
        let dispatcher = JobDispatcher::new(Some(Arc::new(move || {
            repaint_ctx.request_repaint();
        })));

        let mut app = Self {
            tree: create_default_workspace_tree(),
            camera: CadCamera::default(),
            viewport_renderer,
            wgpu_render_state,
            project: ProjectDocument::new("CAM Torture Block Job"),
            project_file_path: None,
            history: UndoHistory::new(100),
            toolpath_cache: ToolpathCache::new(128),
            dispatcher,
            global_tools,
            tool_library_modal: ToolLibraryModalState::new(),
            sim_panel: SimulationPanelState::default(),
            gcode_panel: GCodePreviewPanelState::default(),
            auto_sim_after_gen: true,
            active_setup_idx: 0,
            inspector_focus: InspectorFocus::Setup { setup_idx: 0 },
            setup_tab: SetupTab::Wcs,
            op_tab: OperationTab::Passes,
            pick_mode: ViewportPickMode::InspectEntity,
            lock_chipload: true,
            show_27pt_dots: true,
            show_z_planes: true,
            show_all_setup_toolpaths: true,
            model: None,
            next_revision: 1,
            loading_in_flight: false,
            job_tx,
            job_rx,
            pick_filter: PickFilter::Any,
            highlight: HighlightState::default(),
            wb_tool_dia: ExpressionInput::new(6.35, Some("1/4 in")),
            wb_stepover: ExpressionInput::new(2.54, Some("tool_d * 0.4")),
            wb_clearance_z: ExpressionInput::new(26.0, Some("model_top_z + 10 mm")),
            logs: vec![
                "ChipCutter v0.1.0 initialized (Milestone 4: Heightmap Simulation, Crash Verifier, & TOML Post-Processors)."
                    .to_string(),
            ],
            self_test,
        };

        if let Some(path) = initial_path {
            let is_chip_proj = path
                .to_string_lossy()
                .to_ascii_lowercase()
                .ends_with(".toml");
            if is_chip_proj {
                app.load_project_from_disk(&path, cc.egui_ctx.clone());
            } else {
                app.spawn_step_file_load(path, cc.egui_ctx.clone());
            }
        } else {
            app.spawn_cam_torture_sample_load(true, cc.egui_ctx.clone());
        }

        app
    }

    fn eval_context(&self) -> EvalContext {
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

    fn refresh_dependent_expressions(&mut self) {
        let ctx = self.eval_context();
        self.wb_stepover.refresh(&ctx);
        self.wb_clearance_z.refresh(&ctx);
    }

    fn load_project_from_disk(&mut self, path: &Path, egui_ctx: egui::Context) {
        match ProjectDocument::load_from_path(path) {
            Ok(doc) => {
                self.history.clear();
                self.project = doc;
                self.project_file_path = Some(path.to_path_buf());
                self.active_setup_idx = 0;
                self.inspector_focus = InspectorFocus::Setup { setup_idx: 0 };
                self.pick_mode = ViewportPickMode::InspectEntity;
                self.logs
                    .push(format!("Opened project '{}'.", path.display()));
                if let Some(first_model) = self.project.models.first() {
                    let step_path = resolve_model_path(path.parent(), &first_model.path);
                    self.spawn_step_file_load_internal(step_path, false, egui_ctx);
                }
            }
            Err(err) => {
                self.logs
                    .push(format!("ERROR opening project '{}': {err}", path.display()));
            }
        }
    }

    fn save_project_to_disk(&mut self, path: &Path) {
        for entry in &mut self.project.models {
            let cur = PathBuf::from(&entry.path);
            if cur.is_absolute() {
                entry.path = portable_model_path(Some(path), &cur);
            }
        }
        match self.project.save_to_path(path) {
            Ok(()) => {
                self.project_file_path = Some(path.to_path_buf());
                self.logs
                    .push(format!("Saved project to '{}'.", path.display()));
            }
            Err(err) => {
                self.logs
                    .push(format!("ERROR saving project '{}': {err}", path.display()));
            }
        }
    }

    fn spawn_step_file_load(&mut self, path: PathBuf, egui_ctx: egui::Context) {
        self.spawn_step_file_load_internal(path, true, egui_ctx);
    }

    fn spawn_step_file_load_internal(
        &mut self,
        path: PathBuf,
        update_model_entry: bool,
        egui_ctx: egui::Context,
    ) {
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
                    let sha256_hex = compute_sha256_hex(&bytes);
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
                            let topo = TopologyGraph::build(&doc);
                            LoadJobResult::Loaded {
                                payload: Box::new(LoadedPayload {
                                    doc,
                                    picker,
                                    topo,
                                    source_path: if update_model_entry { Some(path) } else { None },
                                    sha256_hex,
                                    seed_demo: SeedDemoKind::None,
                                }),
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
                Ok(step_str) => {
                    let sha256_hex = compute_sha256_hex(step_str.as_bytes());
                    match MonstertruckKernel.load_step_bytes(
                        "sample_machined_plate.step",
                        step_str.as_bytes(),
                        DEFAULT_TESS_CHORD_TOL_MM,
                    ) {
                        Ok(doc) => {
                            let picker = CadPicker::build(&doc);
                            let topo = TopologyGraph::build(&doc);
                            LoadJobResult::Loaded {
                                payload: Box::new(LoadedPayload {
                                    doc,
                                    picker,
                                    topo,
                                    source_path: None,
                                    sha256_hex,
                                    seed_demo: SeedDemoKind::None,
                                }),
                                elapsed_ms: t0.elapsed().as_millis(),
                            }
                        }
                        Err(e) => LoadJobResult::Failed(e.to_string()),
                    }
                }
                Err(e) => LoadJobResult::Failed(e.to_string()),
            };
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    fn spawn_multi_feature_sample_load(
        &mut self,
        tilted: bool,
        seed_demo_ops: bool,
        egui_ctx: egui::Context,
    ) {
        self.loading_in_flight = true;
        self.logs.push(format!(
            "Generating Milestone 2 multi-feature bracket STEP fixture (tilted={tilted})…"
        ));
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = std::time::Instant::now();
            let res = match sample_multi_feature_bracket_step(tilted) {
                Ok(step_str) => {
                    let sha256_hex = compute_sha256_hex(step_str.as_bytes());
                    let name = if tilted {
                        "sample_multi_feature_tilted.step"
                    } else {
                        "sample_multi_feature_bracket.step"
                    };
                    match MonstertruckKernel.load_step_bytes(
                        name,
                        step_str.as_bytes(),
                        DEFAULT_TESS_CHORD_TOL_MM,
                    ) {
                        Ok(doc) => {
                            let picker = CadPicker::build(&doc);
                            let topo = TopologyGraph::build(&doc);
                            LoadJobResult::Loaded {
                                payload: Box::new(LoadedPayload {
                                    doc,
                                    picker,
                                    topo,
                                    source_path: None,
                                    sha256_hex,
                                    seed_demo: if seed_demo_ops {
                                        SeedDemoKind::MultiFeatureBracket
                                    } else {
                                        SeedDemoKind::None
                                    },
                                }),
                                elapsed_ms: t0.elapsed().as_millis(),
                            }
                        }
                        Err(e) => LoadJobResult::Failed(e.to_string()),
                    }
                }
                Err(e) => LoadJobResult::Failed(e.to_string()),
            };
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    fn spawn_cam_torture_sample_load(&mut self, seed_demo_ops: bool, egui_ctx: egui::Context) {
        self.loading_in_flight = true;
        self.logs.push(
            "Generating Milestone 3 CAM Torture Block STEP fixture (island pocket, open step, Ø18 bore, 45° bevel)…"
                .to_string(),
        );
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = std::time::Instant::now();
            let res = match sample_cam_torture_block_step() {
                Ok(step_str) => {
                    let sha256_hex = compute_sha256_hex(step_str.as_bytes());
                    match MonstertruckKernel.load_step_bytes(
                        "sample_cam_torture_block.step",
                        step_str.as_bytes(),
                        DEFAULT_TESS_CHORD_TOL_MM,
                    ) {
                        Ok(doc) => {
                            let picker = CadPicker::build(&doc);
                            let topo = TopologyGraph::build(&doc);
                            LoadJobResult::Loaded {
                                payload: Box::new(LoadedPayload {
                                    doc,
                                    picker,
                                    topo,
                                    source_path: None,
                                    sha256_hex,
                                    seed_demo: if seed_demo_ops {
                                        SeedDemoKind::CamTortureBlock
                                    } else {
                                        SeedDemoKind::None
                                    },
                                }),
                                elapsed_ms: t0.elapsed().as_millis(),
                            }
                        }
                        Err(e) => LoadJobResult::Failed(e.to_string()),
                    }
                }
                Err(e) => LoadJobResult::Failed(e.to_string()),
            };
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    fn poll_background_jobs(&mut self, egui_ctx: &egui::Context) {
        while let Ok(msg) = self.job_rx.try_recv() {
            self.loading_in_flight = false;
            match msg {
                LoadJobResult::Loaded {
                    payload,
                    elapsed_ms,
                } => {
                    let LoadedPayload {
                        doc,
                        picker,
                        topo,
                        source_path,
                        sha256_hex,
                        seed_demo,
                    } = *payload;
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

                    self.dispatcher.cancel_all();

                    match seed_demo {
                        SeedDemoKind::MultiFeatureBracket => {
                            self.seed_initial_demo_job(&doc);
                        }
                        SeedDemoKind::CamTortureBlock => {
                            self.seed_cam_torture_demo_job(&doc, &sha256_hex);
                        }
                        SeedDemoKind::None => {}
                    }

                    if let Some(step_path) = source_path {
                        let rel =
                            portable_model_path(self.project_file_path.as_deref(), &step_path);
                        self.project.models = vec![ModelEntry {
                            id: "model_1".to_string(),
                            path: rel,
                            sha256: sha256_hex,
                        }];
                    } else if self.project.models.is_empty() {
                        self.project.models = vec![ModelEntry {
                            id: "model_1".to_string(),
                            path: doc.source_name.clone(),
                            sha256: sha256_hex,
                        }];
                    }

                    self.camera.fit_aabb(&doc.aabb_mm);
                    self.highlight.hovered = None;
                    self.highlight.selected = None;

                    let rev = self.next_revision;
                    self.next_revision += 1;
                    self.model = Some(LoadedModelState {
                        doc: Arc::new(doc),
                        picker,
                        topo,
                        revision: rev,
                    });
                    self.refresh_dependent_expressions();

                    if seed_demo == SeedDemoKind::CamTortureBlock {
                        self.auto_sim_after_gen = true;
                        self.trigger_generate_dirty(None, None);
                    }
                }
                LoadJobResult::Failed(err) => {
                    self.logs.push(format!("ERROR loading STEP model: {err}"));
                }
            }
        }

        // Poll background toolpath generation worker events (`JobDispatcher`)
        if let Some(m) = &self.model {
            self.dispatcher.cancel_stale_jobs(&self.project, m.revision);
            let gen_logs = self.dispatcher.poll_events(
                &mut self.project,
                &mut self.toolpath_cache,
                m.revision,
            );
            let had_gen_completion = !gen_logs.is_empty();
            self.logs.extend(gen_logs);

            if self.auto_sim_after_gen && had_gen_completion && !self.dispatcher.is_busy() {
                self.auto_sim_after_gen = false;
                self.trigger_active_setup_simulation(egui_ctx);
            }
        }

        // Poll background heightmap simulation worker
        let _ = self.sim_panel.poll_worker(&mut self.logs);
    }

    fn trigger_active_setup_simulation(&mut self, egui_ctx: &egui::Context) {
        let Some(m) = &self.model else {
            self.logs
                .push("Cannot simulate: load a STEP model first.".to_string());
            return;
        };
        let proj_dir = self.project_file_path.as_deref().and_then(Path::parent);
        self.sim_panel.trigger_simulation(
            &self.project,
            self.active_setup_idx,
            Arc::clone(&m.doc),
            m.revision,
            &self.toolpath_cache,
            proj_dir,
            egui_ctx.clone(),
            &mut self.logs,
        );
    }

    fn jump_to_sim_diagnostic(&mut self, diag: &SimDiagnostic) {
        self.sim_panel.jump_to_time(diag.timeline_time_sec);
        focus_workspace_pane(&mut self.tree, WorkspacePane::SimulationTimeline);
        if let Some(Ok(rs)) = self.active_resolved_setup() {
            self.camera.target_mm = rs.frame.local_to_world_point(diag.location_wcs);
        }
    }

    fn trigger_generate_dirty(&mut self, only_setup_idx: Option<usize>, only_op_id: Option<&str>) {
        let Some(m) = &self.model else {
            self.logs
                .push("Cannot generate toolpaths: load a STEP model first.".to_string());
            return;
        };
        let spawned = self.dispatcher.dispatch_operations(
            &mut self.project,
            Arc::clone(&m.doc),
            m.revision,
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

    fn seed_initial_demo_job(&mut self, doc: &CadDocument) {
        self.project = ProjectDocument::new("Multi-Feature Bracket");
        if let Some(t1) = self.global_tools.tools.first() {
            self.project.ensure_tool_embedded(t1);
        }
        if let Some(drill) = self
            .global_tools
            .tools
            .iter()
            .find(|t| matches!(t.geometry, chip_tools::CutterGeometry::TwistDrill { .. }))
        {
            self.project.ensure_tool_embedded(drill);
        }

        // Operation 1: 2D Pocket on the Z = 12.0 mm recessed pocket floor
        if let Some(pkt_floor) = doc.faces.iter().find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && (origin_mm.z - 12.0).abs() < 0.05 && !f.inner_loops.is_empty()
            )
        }) && let Some(bind) = doc.bind_face(pkt_floor.id)
        {
            let mut op_pkt = Operation::new(
                "op_1_pocket",
                "Center Cavity Pocket",
                OperationKind::Pocket2D,
            );
            if let Some(t1) = self.project.tools.first() {
                op_pkt.tool_id = Some(t1.id.clone());
                op_pkt.preset_id = t1.presets.first().map(|p| p.id.clone());
            }
            op_pkt
                .selections
                .push(GeometrySelectionItem::PlanarFace { binding: bind });
            self.project.setups[0].operations.push(op_pkt);
        }

        // Operation 2: Drill all 4x Ø6mm mounting through-holes
        if let Some(r3_seed) = doc.faces.iter().find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if (radius_mm - 3.0).abs() < 1e-2
            )
        }) {
            let mut op_drill = Operation::new(
                "op_2_holes",
                "4x Ø6mm Mounting Holes",
                OperationKind::DrillBore,
            );
            if let Some(drill) = self
                .project
                .tools
                .iter()
                .find(|t| matches!(t.geometry, chip_tools::CutterGeometry::TwistDrill { .. }))
            {
                op_drill.tool_id = Some(drill.id.clone());
                op_drill.preset_id = drill.presets.first().map(|p| p.id.clone());
            }
            for fid in find_same_diameter_holes(doc, r3_seed.id, 1e-3, 1e-3) {
                if let Some(b) = doc.bind_face(fid) {
                    op_drill
                        .selections
                        .push(GeometrySelectionItem::CylindricalHole { binding: b });
                }
            }
            self.project.setups[0].operations.push(op_drill);
        }

        self.active_setup_idx = 0;
        if !self.project.setups[0].operations.is_empty() {
            self.inspector_focus = InspectorFocus::Operation {
                setup_idx: 0,
                op_idx: 0,
            };
            self.op_tab = OperationTab::Geometry;
        }
        self.history.clear();
    }

    fn seed_cam_torture_demo_job(&mut self, doc: &CadDocument, sha256_hex: &str) {
        self.project = ProjectDocument::new("CAM Torture Block");
        self.project.models = vec![ModelEntry {
            id: "model_1".to_string(),
            path: doc.source_name.clone(),
            sha256: sha256_hex.to_string(),
        }];

        for tool in &self.global_tools.tools {
            self.project.ensure_tool_embedded(tool);
        }

        let flat_6mm = self
            .project
            .tools
            .iter()
            .find(|t| {
                matches!(
                    t.geometry,
                    chip_tools::CutterGeometry::FlatEndmill { diameter_mm, .. }
                    if (diameter_mm - 6.0).abs() < 0.05
                )
            })
            .cloned();
        let face_38mm = self
            .project
            .tools
            .iter()
            .find(|t| matches!(t.geometry, chip_tools::CutterGeometry::FaceMill { .. }))
            .cloned();
        let drill_5mm = self
            .project
            .tools
            .iter()
            .find(|t| matches!(t.geometry, chip_tools::CutterGeometry::TwistDrill { .. }))
            .cloned();
        let chamf_90 = self
            .project
            .tools
            .iter()
            .find(|t| matches!(t.geometry, chip_tools::CutterGeometry::ChamferMill { .. }))
            .cloned();

        // 1. Facing Top Stock (`Z = 23 -> 22`)
        if let Some(fm) = &face_38mm {
            let mut op_face = Operation::new("op_1_face", "Face Stock Top", OperationKind::Facing);
            op_face.tool_id = Some(fm.id.clone());
            op_face.preset_id = fm.presets.first().map(|p| p.id.clone());
            self.project.setups[0].operations.push(op_face);
        }

        // 2. Plateau Deck around Z=22 Boss Pad (`Z = 22 -> 20 mm`)
        if let Some(plat_face) = doc.faces.iter().find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && (origin_mm.z - 20.0).abs() < 0.05 && f.area_mm2 > 1500.0
            )
        }) && let Some(bind) = doc.bind_face(plat_face.id)
            && let Some(em) = &flat_6mm
        {
            let mut op_plat = Operation::new(
                "op_2_plateau",
                "Plateau Deck Around Boss",
                OperationKind::Pocket2D,
            );
            op_plat.tool_id = Some(em.id.clone());
            op_plat.preset_id = em.presets.first().map(|p| p.id.clone());
            op_plat
                .selections
                .push(GeometrySelectionItem::PlanarFace { binding: bind });
            self.project.setups[0].operations.push(op_plat);
        }

        // 3. Closed Island Pocket (Z = 12.0 mm floor with Ø10mm raised island boss)
        if let Some(pkt_floor) = doc.faces.iter().find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && (origin_mm.z - 12.0).abs() < 0.05
            )
        }) && let Some(bind) = doc.bind_face(pkt_floor.id)
            && let Some(em) = &flat_6mm
        {
            let mut op_pkt = Operation::new(
                "op_2_island_pocket",
                "Island Cavity Pocket",
                OperationKind::Pocket2D,
            );
            op_pkt.tool_id = Some(em.id.clone());
            op_pkt.preset_id = em.presets.first().map(|p| p.id.clone());
            op_pkt
                .selections
                .push(GeometrySelectionItem::PlanarFace { binding: bind });
            self.project.setups[0].operations.push(op_pkt);
        }

        // 3. Open-Edge Step Shoulder Pocket (Z = 14.0 mm shelf)
        if let Some(step_floor) = doc.faces.iter().find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && (origin_mm.z - 14.0).abs() < 0.05
            )
        }) && let Some(bind) = doc.bind_face(step_floor.id)
            && let Some(em) = &flat_6mm
        {
            let mut op_step = Operation::new(
                "op_3_open_step",
                "Open Shoulder Step",
                OperationKind::Pocket2D,
            );
            op_step.tool_id = Some(em.id.clone());
            op_step.preset_id = em.presets.first().map(|p| p.id.clone());
            op_step
                .selections
                .push(GeometrySelectionItem::PlanarFace { binding: bind });
            self.project.setups[0].operations.push(op_step);
        }

        // 4. Helical Bore on Ø18mm Counterbore + 4x Ø6mm Shelf Through-Holes
        if let Some(r9_bore) = doc.faces.iter().find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if (radius_mm - 9.0).abs() < 0.05
            )
        }) && let Some(bind) = doc.bind_face(r9_bore.id)
            && let Some(em) = &flat_6mm
        {
            let mut op_bore = Operation::new(
                "op_4_helical_bore",
                "Ø18mm Helical Counterbore",
                OperationKind::DrillBore,
            );
            op_bore.tool_id = Some(em.id.clone());
            op_bore.preset_id = em.presets.first().map(|p| p.id.clone());
            if let StrategyParams::DrillBore(p) = op_bore.strategy_mut() {
                p.mode = HoleMachiningMode::HelicalBore;
            }
            op_bore
                .selections
                .push(GeometrySelectionItem::CylindricalHole { binding: bind });
            self.project.setups[0].operations.push(op_bore);
        }

        if let Some(r3_seed) = doc.faces.iter().find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if (radius_mm - 3.0).abs() < 0.05
            )
        }) && let Some(dr) = &drill_5mm
        {
            let mut op_drill = Operation::new(
                "op_5_shelf_holes",
                "4x Shelf Through-Holes",
                OperationKind::DrillBore,
            );
            op_drill.tool_id = Some(dr.id.clone());
            op_drill.preset_id = dr.presets.first().map(|p| p.id.clone());
            for fid in find_same_diameter_holes(doc, r3_seed.id, 1e-3, 1e-3) {
                if let Some(b) = doc.bind_face(fid) {
                    op_drill
                        .selections
                        .push(GeometrySelectionItem::CylindricalHole { binding: b });
                }
            }
            self.project.setups[0].operations.push(op_drill);
        }

        // 5. Outer 2D Contour + Ramped Triangular Holding Tabs
        if let Some(bot_face) = doc.faces.iter().find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z < -0.99 && origin_mm.z.abs() < 0.05
            )
        }) && let Some(outer_lid) = bot_face.outer_loop
            && let Some(bind) = doc.bind_loop(outer_lid)
            && let Some(em) = &flat_6mm
        {
            let mut op_cnt = Operation::new(
                "op_6_outer_contour",
                "Outer Profile + Tabs",
                OperationKind::Contour2D,
            );
            op_cnt.tool_id = Some(em.id.clone());
            op_cnt.preset_id = em.presets.first().map(|p| p.id.clone());
            op_cnt.heights.top = HeightSpec::new(HeightReference::ModelTop, -2.0);
            op_cnt.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
            op_cnt.selections.push(GeometrySelectionItem::ClosedLoop {
                binding: bind,
                reverse: false,
                side_flip: false,
            });
            if let StrategyParams::Contour2D(p) = op_cnt.strategy_mut() {
                p.tabs.enabled = true;
                p.tabs.profile = TabProfileKind::Triangular {
                    ramp_angle_deg: 25.0,
                };
            }
            self.project.setups[0].operations.push(op_cnt);
        }

        // 6. Sharp Step Edge Chamfer
        if let Some(step_edge) = doc.edges.iter().find(|e| {
            (e.aabb_mm.min.z - 20.0).abs() < 0.05
                && (e.aabb_mm.max.z - 20.0).abs() < 0.05
                && (e.aabb_mm.min.x - 76.0).abs() < 0.05
                && (e.aabb_mm.max.x - 76.0).abs() < 0.05
        }) && let Some(eb) = doc.bind_edge(step_edge.id)
            && let Some(cm) = &chamf_90
        {
            let mut op_ch = Operation::new(
                "op_7_step_chamfer",
                "Step Edge Chamfer",
                OperationKind::ChamferTrace,
            );
            op_ch.tool_id = Some(cm.id.clone());
            op_ch.preset_id = cm.presets.first().map(|p| p.id.clone());
            op_ch.selections.push(GeometrySelectionItem::OpenChain {
                body_id: doc.bodies[0].id,
                start_edge: eb.clone(),
                end_edge: eb,
                resolved_edges: vec![chip_cad::OrientedEdgeUse {
                    edge_id: step_edge.id,
                    forward: true,
                }],
                reverse: false,
                side_flip: true,
            });
            if let StrategyParams::ChamferTrace(p) = op_ch.strategy_mut() {
                p.mode = ChamferTraceMode::EdgeChamfer;
                p.chamfer_source = ChamferSourceMode::SharpEdge {
                    width: ParamLength::from_mm(0.5),
                };
                p.tip_offset = ParamLength::from_mm(0.8);
            }
            self.project.setups[0].operations.push(op_ch);
        }

        self.active_setup_idx = 0;
        if self.project.setups[0].operations.len() > 1 {
            self.inspector_focus = InspectorFocus::Operation {
                setup_idx: 0,
                op_idx: 1,
            };
            self.op_tab = OperationTab::Passes;
        }
        self.history.clear();
    }

    fn active_resolved_setup(&self) -> Option<Result<ResolvedSetup, String>> {
        let m = self.model.as_ref()?;
        let setup = self.project.setups.get(self.active_setup_idx)?;
        Some(
            setup
                .resolve_immutable(&m.doc, self.project.display_units)
                .map_err(|e| e.to_string()),
        )
    }

    fn align_camera_to_active_setup(&mut self) {
        if let Some(Ok(rs)) = self.active_resolved_setup() {
            self.camera
                .align_to_setup_top(&rs.frame, &rs.stock.aabb_wcs());
        }
    }

    fn sync_viewport_scene_state(&mut self) {
        self.highlight.extra_selected_faces.clear();
        self.highlight.extra_selected_edges.clear();
        self.highlight.chain_arrows.clear();
        self.highlight.z_planes_wcs.clear();

        let resolved_opt = self.active_resolved_setup().and_then(Result::ok);
        if let Some(rs) = &resolved_opt {
            self.highlight.wcs_frame = rs.frame;
            self.camera.turntable_frame = rs.frame;
            self.highlight.stock = Some(rs.stock);
        } else {
            self.highlight.wcs_frame = chip_math::RigidFrame3::IDENTITY;
            self.highlight.stock = None;
        }

        // Populate multi-selection highlights + Z-height planes based on active Inspector focus
        match self.inspector_focus {
            InspectorFocus::Setup { setup_idx } => {
                if self.show_z_planes
                    && self.setup_tab == SetupTab::Heights
                    && setup_idx == self.active_setup_idx
                    && let Some(rs) = &resolved_opt
                {
                    self.highlight.z_planes_wcs.push(ZPlaneVisual {
                        z_wcs: rs.clearance_z_wcs,
                        fill_rgba: [0.24, 0.72, 1.0, 0.11],
                        border_rgba: [0.34, 0.80, 1.0, 0.85],
                    });
                    self.highlight.z_planes_wcs.push(ZPlaneVisual {
                        z_wcs: rs.retract_z_wcs,
                        fill_rgba: [1.0, 0.72, 0.22, 0.10],
                        border_rgba: [1.0, 0.78, 0.28, 0.85],
                    });
                }
            }
            InspectorFocus::Operation { setup_idx, op_idx } => {
                if let (Some(m), Some(rs)) = (&self.model, &resolved_opt)
                    && let Some(setup) = self.project.setups.get(setup_idx)
                    && let Some(op) = setup.operations.get(op_idx)
                    && let Ok(sum) = extract_selection_features(&m.doc, rs, &op.selections)
                {
                    for fid in sum.highlight_faces {
                        self.highlight.extra_selected_faces.insert(fid);
                    }
                    for eid in sum.highlight_edges {
                        self.highlight.extra_selected_edges.insert(eid);
                    }
                    self.highlight.chain_arrows = sum.direction_arrows;

                    if self.show_z_planes && self.op_tab == OperationTab::Heights {
                        let mut ctx = rs.eval_context(self.project.display_units);
                        if let Some(tid) = &op.tool_id
                            && let Some(t) = self.project.tool(tid)
                        {
                            t.populate_eval_context(&mut ctx);
                        }
                        ctx.set_var("feature_top_z", sum.inferred_top_z_wcs);
                        ctx.set_var("feature_bottom_z", sum.inferred_bottom_z_wcs);
                        let mut h_eval = op.heights.clone();
                        h_eval.reevaluate(&ctx);
                        if let Ok(rh) = ResolvedHeights::resolve(
                            &h_eval,
                            &rs.model_aabb_wcs,
                            &rs.stock.aabb_wcs(),
                            rs.clearance_z_wcs,
                            rs.retract_z_wcs,
                            sum.inferred_top_z_wcs,
                            sum.inferred_bottom_z_wcs,
                        ) {
                            self.highlight.z_planes_wcs = vec![
                                ZPlaneVisual {
                                    z_wcs: rh.clearance_z_wcs,
                                    fill_rgba: [0.24, 0.72, 1.0, 0.10],
                                    border_rgba: [0.34, 0.80, 1.0, 0.85],
                                },
                                ZPlaneVisual {
                                    z_wcs: rh.retract_z_wcs,
                                    fill_rgba: [1.0, 0.72, 0.22, 0.09],
                                    border_rgba: [1.0, 0.78, 0.28, 0.85],
                                },
                                ZPlaneVisual {
                                    z_wcs: rh.feed_z_wcs,
                                    fill_rgba: [0.26, 0.90, 0.46, 0.09],
                                    border_rgba: [0.30, 0.94, 0.50, 0.85],
                                },
                                ZPlaneVisual {
                                    z_wcs: rh.top_z_wcs,
                                    fill_rgba: [0.78, 0.52, 1.0, 0.10],
                                    border_rgba: [0.84, 0.60, 1.0, 0.88],
                                },
                                ZPlaneVisual {
                                    z_wcs: rh.bottom_z_wcs,
                                    fill_rgba: [1.0, 0.36, 0.52, 0.12],
                                    border_rgba: [1.0, 0.44, 0.58, 0.92],
                                },
                            ];
                        }
                    }
                }
            }
            InspectorFocus::CadEntity => {}
        }

        if let ViewportPickMode::OpOpenChain {
            start_edge: Some(se),
            ..
        } = self.pick_mode
        {
            self.highlight.extra_selected_edges.insert(se);
        }

        // Populate Pass #5 cached Clean toolpaths for the active Setup
        self.highlight.toolpaths.clear();
        if let Some(m) = &self.model
            && let Some(setup) = self.project.setups.get(self.active_setup_idx)
        {
            let focused_op_idx = match self.inspector_focus {
                InspectorFocus::Operation { setup_idx, op_idx }
                    if setup_idx == self.active_setup_idx =>
                {
                    Some(op_idx)
                }
                _ => None,
            };

            for (o_idx, op) in setup.operations.iter().enumerate() {
                if op.suppressed {
                    continue;
                }
                let is_focused = focused_op_idx.is_none_or(|f_idx| f_idx == o_idx);
                if !self.show_all_setup_toolpaths && !is_focused {
                    continue;
                }
                if let Some((hash, tp)) = self.toolpath_cache.get_for_operation(
                    &self.project,
                    self.active_setup_idx,
                    o_idx,
                    m.revision,
                ) {
                    self.highlight.toolpaths.push(ToolpathRenderEntry {
                        op_id: op.id.clone(),
                        cache_key: hash.0,
                        toolpath: tp,
                        is_focused,
                    });
                }
            }

            let (sim_stock, sim_tool, sim_trail) = self.sim_panel.resolve_viewport_visuals(
                &self.project,
                self.active_setup_idx,
                &self.toolpath_cache,
                m.revision,
            );
            self.highlight.sim_stock_mesh = sim_stock;
            self.highlight.sim_tool_mesh = sim_tool;
            self.highlight.sim_trail_limit = sim_trail;
        } else {
            self.highlight.sim_stock_mesh = None;
            self.highlight.sim_tool_mesh = None;
            self.highlight.sim_trail_limit = None;
        }
    }

    fn effective_pick_filter(&self) -> PickFilter {
        match self.pick_mode {
            ViewportPickMode::InspectEntity => self.pick_filter,
            ViewportPickMode::SetupWcsZ { .. } | ViewportPickMode::SetupWcsX { .. } => {
                PickFilter::Any
            }
            ViewportPickMode::SetupWcsXyPlane { .. } | ViewportPickMode::OpPlanarFaces { .. } => {
                PickFilter::FacesOnly
            }
            ViewportPickMode::SetupWcsOriginFeature { .. } => PickFilter::Any,
            ViewportPickMode::OpClosedLoops { .. } => PickFilter::EdgeLoops,
            ViewportPickMode::OpOpenChain { .. } | ViewportPickMode::PlaceContourTab { .. } => {
                PickFilter::EdgesOnly
            }
            ViewportPickMode::OpCylindricalHoles { .. } => PickFilter::CylindricalHolesOnly,
        }
    }

    fn handle_viewport_pick_click(&mut self, picked: Option<PickTarget>) {
        let Some(target) = picked else {
            if self.pick_mode == ViewportPickMode::InspectEntity {
                self.highlight.selected = None;
            }
            return;
        };

        let Some(m) = &self.model else {
            return;
        };
        let doc = &m.doc;

        match self.pick_mode {
            ViewportPickMode::InspectEntity => {
                self.highlight.selected = Some(target);
                self.inspector_focus = InspectorFocus::CadEntity;
            }
            ViewportPickMode::SetupWcsZ { setup_idx } => {
                if setup_idx < self.project.setups.len() {
                    let axis_opt = match target {
                        PickTarget::Face(fid) => doc.bind_face(fid).map(WcsAxisRef::FaceAxis),
                        PickTarget::Edge(eid) | PickTarget::Loop { seed_edge: eid, .. } => {
                            doc.bind_edge(eid).map(WcsAxisRef::EdgeDirection)
                        }
                    };
                    if let Some(z_axis_ref) = axis_opt {
                        self.history.record_before_edit(
                            "Pick Setup WCS Z Axis",
                            None,
                            &self.project,
                        );
                        if let WcsOrientationMode::SelectZAndX { z_ref, .. } =
                            &mut self.project.setups[setup_idx].wcs.orientation
                        {
                            *z_ref = z_axis_ref;
                        }
                        self.pick_mode = ViewportPickMode::InspectEntity;
                    }
                }
            }
            ViewportPickMode::SetupWcsX { setup_idx } => {
                if setup_idx < self.project.setups.len() {
                    let axis_opt = match target {
                        PickTarget::Edge(eid) | PickTarget::Loop { seed_edge: eid, .. } => {
                            doc.bind_edge(eid).map(WcsAxisRef::EdgeDirection)
                        }
                        PickTarget::Face(fid) => doc.bind_face(fid).map(WcsAxisRef::FaceAxis),
                    };
                    if let Some(x_axis_ref) = axis_opt {
                        self.history.record_before_edit(
                            "Pick Setup WCS X Axis",
                            None,
                            &self.project,
                        );
                        if let WcsOrientationMode::SelectZAndX { x_ref, .. } =
                            &mut self.project.setups[setup_idx].wcs.orientation
                        {
                            *x_ref = Some(x_axis_ref);
                        }
                        self.pick_mode = ViewportPickMode::InspectEntity;
                    }
                }
            }
            ViewportPickMode::SetupWcsXyPlane { setup_idx } => {
                if let PickTarget::Face(fid) = target
                    && setup_idx < self.project.setups.len()
                    && let Some(bind) = doc.bind_face(fid)
                {
                    self.history
                        .record_before_edit("Pick Setup WCS XY Plane", None, &self.project);
                    if let WcsOrientationMode::SelectXYPlane { face_ref, .. } =
                        &mut self.project.setups[setup_idx].wcs.orientation
                    {
                        *face_ref = bind;
                    }
                    self.pick_mode = ViewportPickMode::InspectEntity;
                }
            }
            ViewportPickMode::SetupWcsOriginFeature { setup_idx } => {
                if setup_idx < self.project.setups.len() {
                    let feat_opt = match target {
                        PickTarget::Face(fid) => {
                            if let Some(f) = doc.face(fid)
                                && matches!(
                                    f.surface,
                                    SurfaceKind::Cylinder {
                                        is_internal: true,
                                        ..
                                    }
                                )
                            {
                                doc.bind_face(fid)
                                    .map(|b| WcsFeaturePointRef::HoleAxisCenter {
                                        hole_face: b,
                                        top_rim: true,
                                    })
                            } else {
                                None
                            }
                        }
                        PickTarget::Edge(eid) | PickTarget::Loop { seed_edge: eid, .. } => {
                            doc.edge(eid).and_then(|e| {
                                let b = doc.bind_edge(eid)?;
                                if matches!(e.curve, CurveKind::CircleArc { .. }) {
                                    Some(WcsFeaturePointRef::CircleCenter(b))
                                } else {
                                    Some(WcsFeaturePointRef::EdgeMidpoint(b))
                                }
                            })
                        }
                    };
                    if let Some(feature) = feat_opt {
                        self.history.record_before_edit(
                            "Pick WCS Origin Feature",
                            None,
                            &self.project,
                        );
                        self.project.setups[setup_idx].wcs.origin =
                            WcsOriginMode::SelectedCadFeature { feature };
                        self.pick_mode = ViewportPickMode::InspectEntity;
                    }
                }
            }
            ViewportPickMode::OpPlanarFaces { setup_idx, op_idx } => {
                if let PickTarget::Face(fid) = target
                    && let Some(bind) = doc.bind_face(fid)
                    && let Some(setup) = self.project.setups.get(setup_idx)
                    && op_idx < setup.operations.len()
                {
                    self.history.record_before_edit(
                        "Toggle Planar Face Selection",
                        None,
                        &self.project,
                    );
                    let sels = &mut self.project.setups[setup_idx].operations[op_idx].selections;
                    if let Some(pos) = sels.iter().position(|s| {
                        matches!(s, GeometrySelectionItem::PlanarFace { binding } if binding.last_index == fid.0)
                    }) {
                        sels.remove(pos);
                    } else {
                        sels.push(GeometrySelectionItem::PlanarFace { binding: bind });
                    }
                    self.project.setups[setup_idx].operations[op_idx].mark_dirty();
                }
            }
            ViewportPickMode::OpClosedLoops { setup_idx, op_idx } => {
                let lid_opt = match target {
                    PickTarget::Loop { loop_id, .. } => Some(loop_id),
                    PickTarget::Edge(eid) => doc.edge(eid).and_then(|e| e.loops.first().copied()),
                    _ => None,
                };
                if let Some(lid) = lid_opt
                    && let Some(bind) = doc.bind_loop(lid)
                    && let Some(setup) = self.project.setups.get(setup_idx)
                    && op_idx < setup.operations.len()
                {
                    self.history.record_before_edit(
                        "Toggle Closed Loop Selection",
                        None,
                        &self.project,
                    );
                    let sels = &mut self.project.setups[setup_idx].operations[op_idx].selections;
                    if let Some(pos) = sels.iter().position(|s| {
                        matches!(s, GeometrySelectionItem::ClosedLoop { binding, .. } if binding.last_index == lid.0)
                    }) {
                        sels.remove(pos);
                    } else {
                        sels.push(GeometrySelectionItem::ClosedLoop {
                            binding: bind,
                            reverse: false,
                            side_flip: false,
                        });
                    }
                    self.project.setups[setup_idx].operations[op_idx].mark_dirty();
                }
            }
            ViewportPickMode::OpOpenChain {
                setup_idx,
                op_idx,
                start_edge,
            } => {
                let eid_opt = match target {
                    PickTarget::Edge(eid) => Some(eid),
                    PickTarget::Loop { seed_edge, .. } => Some(seed_edge),
                    _ => None,
                };
                if let Some(eid) = eid_opt {
                    match start_edge {
                        None => {
                            self.pick_mode = ViewportPickMode::OpOpenChain {
                                setup_idx,
                                op_idx,
                                start_edge: Some(eid),
                            };
                        }
                        Some(se) => {
                            if let Some(chain_uses) = m.topo.find_open_edge_chain(doc, se, eid)
                                && let (Some(sb), Some(eb), Some(s_rec)) =
                                    (doc.bind_edge(se), doc.bind_edge(eid), doc.edge(se))
                                && let Some(setup) = self.project.setups.get(setup_idx)
                                && op_idx < setup.operations.len()
                            {
                                self.history.record_before_edit(
                                    "Add Open Edge Chain",
                                    None,
                                    &self.project,
                                );
                                self.project.setups[setup_idx].operations[op_idx]
                                    .selections
                                    .push(GeometrySelectionItem::OpenChain {
                                        body_id: s_rec.body_id,
                                        start_edge: sb,
                                        end_edge: eb,
                                        resolved_edges: chain_uses,
                                        reverse: false,
                                        side_flip: false,
                                    });
                                self.project.setups[setup_idx].operations[op_idx].mark_dirty();
                            } else {
                                self.logs.push(format!(
                                    "Could not find connected open edge path from Edge #{} to Edge #{}.",
                                    se.0, eid.0
                                ));
                            }
                            self.pick_mode = ViewportPickMode::OpOpenChain {
                                setup_idx,
                                op_idx,
                                start_edge: None,
                            };
                        }
                    }
                }
            }
            ViewportPickMode::OpCylindricalHoles { setup_idx, op_idx } => {
                if let PickTarget::Face(fid) = target
                    && let Some(bind) = doc.bind_face(fid)
                    && let Some(setup) = self.project.setups.get(setup_idx)
                    && op_idx < setup.operations.len()
                {
                    self.history
                        .record_before_edit("Toggle Cylindrical Hole", None, &self.project);
                    let sels = &mut self.project.setups[setup_idx].operations[op_idx].selections;
                    if let Some(pos) = sels.iter().position(|s| {
                        matches!(s, GeometrySelectionItem::CylindricalHole { binding } if binding.last_index == fid.0)
                    }) {
                        sels.remove(pos);
                    } else {
                        sels.push(GeometrySelectionItem::CylindricalHole { binding: bind });
                    }
                    self.project.setups[setup_idx].operations[op_idx].mark_dirty();
                }
            }
            ViewportPickMode::PlaceContourTab { .. } => {}
        }
    }

    fn handle_manual_tab_viewport_interaction(
        &mut self,
        setup_idx: usize,
        op_idx: usize,
        ray: &chip_math::Ray3,
        mm_per_px: f64,
        primary_click: bool,
        right_click: bool,
    ) {
        let Some(m) = &self.model else {
            return;
        };
        let Some(Ok(rs)) = self.active_resolved_setup() else {
            return;
        };
        let Some(setup) = self.project.setups.get(setup_idx) else {
            return;
        };
        let Some(op) = setup.operations.get(op_idx) else {
            return;
        };
        let Ok(sum) = extract_selection_features(&m.doc, &rs, &op.selections) else {
            return;
        };

        let ray_orig_wcs = rs.frame.world_to_local_point(ray.origin);
        let ray_dir_wcs = rs.frame.world_to_local_dir(ray.dir);
        if ray_dir_wcs.z.abs() <= 1e-6 {
            return;
        }

        // Collect closed source contours matching `contour2d.rs` contour_index ordering
        let mut closed_contours = Vec::new();
        for feat in &sum.features {
            match feat {
                ExtractedFeature25D::Contour {
                    polyline_wcs,
                    min_z_wcs,
                    ..
                } if polyline_wcs.closed => {
                    closed_contours.push((polyline_wcs.clone(), *min_z_wcs));
                }
                ExtractedFeature25D::PlanarFace {
                    outer_boundary_wcs,
                    face_z_wcs,
                    ..
                } if outer_boundary_wcs.closed => {
                    closed_contours
                        .push((outer_boundary_wcs.ensure_orientation(false), *face_z_wcs));
                }
                _ => {}
            }
        }

        let snap_radius_mm = (14.0 * mm_per_px).clamp(1.0, 12.0);
        let mut best_snap: Option<(usize, f64, DVec2, DVec2, f64, f64)> = None;
        for (c_idx, (poly, z_plane)) in closed_contours.iter().enumerate() {
            let t = (z_plane - ray_orig_wcs.z) / ray_dir_wcs.z;
            if t <= 0.0 {
                continue;
            }
            let hit_wcs = ray_orig_wcs + ray_dir_wcs * t;
            let hit_xy = DVec2::new(hit_wcs.x, hit_wcs.y);
            if let Some(proj) = poly.project_point(hit_xy)
                && proj.distance_mm <= snap_radius_mm
                && best_snap.is_none_or(|(_, _, _, _, _, prev_d)| proj.distance_mm < prev_d)
            {
                best_snap = Some((
                    c_idx,
                    proj.s_norm,
                    proj.point,
                    proj.tangent,
                    *z_plane,
                    proj.distance_mm,
                ));
            }
        }

        let Some((contour_index, s_norm, pt_xy, tan_xy, z_wcs, _)) = best_snap else {
            return;
        };

        let (width_mm, height_mm, is_tri) = match op.resolved_strategy() {
            StrategyParams::Contour2D(p) => (
                p.tabs.width.value_mm,
                p.tabs.height.value_mm,
                matches!(p.tabs.profile, TabProfileKind::Triangular { .. }),
            ),
            _ => return,
        };

        self.highlight.preview_tab_marker_wcs = Some(TabMarker3D {
            center_wcs: chip_math::DVec3::new(pt_xy.x, pt_xy.y, z_wcs),
            tangent_xy: tan_xy,
            width_mm,
            height_mm,
            is_triangular: is_tri,
            contour_index,
            s_norm,
        });

        if primary_click {
            let world_pt = rs
                .frame
                .local_to_world_point(chip_math::DVec3::new(pt_xy.x, pt_xy.y, z_wcs));
            self.history
                .record_before_edit("Add Manual Holding Tab", None, &self.project);
            let op_mut = &mut self.project.setups[setup_idx].operations[op_idx];
            if let StrategyParams::Contour2D(p) = op_mut.strategy_mut() {
                p.tabs.enabled = true;
                p.tabs.placement = chip_cam::TabPlacementMode::Manual;
                p.tabs.manual_anchors.push(ManualTabAnchor {
                    contour_index,
                    s_norm,
                    world_point_mm: world_pt,
                });
            }
            op_mut.mark_dirty();
        } else if right_click {
            let mut removed = false;
            let pre = self.project.clone();
            let op_mut = &mut self.project.setups[setup_idx].operations[op_idx];
            if let StrategyParams::Contour2D(p) = op_mut.strategy_mut()
                && let Some(pos) = p.tabs.manual_anchors.iter().position(|a| {
                    a.contour_index == contour_index && (a.s_norm - s_norm).abs() <= 0.06
                })
            {
                p.tabs.manual_anchors.remove(pos);
                removed = true;
            }
            if removed {
                self.history
                    .record_before_edit("Remove Manual Holding Tab", None, &pre);
                self.project.setups[setup_idx].operations[op_idx].mark_dirty();
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

    fn open_project_dialog(&mut self, ctx: &egui::Context) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("ChipCutter Project (.chip.toml)", &["toml"])
            .pick_file()
        {
            self.load_project_from_disk(&path, ctx.clone());
        }
    }

    fn save_project_dialog(&mut self) {
        if let Some(existing) = self.project_file_path.clone() {
            self.save_project_to_disk(&existing);
        } else if let Some(path) = rfd::FileDialog::new()
            .set_file_name("job.chip.toml")
            .save_file()
        {
            self.save_project_to_disk(&path);
        }
    }

    fn perform_undo(&mut self) {
        if let Some(lbl) = self.history.undo(&mut self.project) {
            if let Some(m) = &self.model {
                self.dispatcher.cancel_stale_jobs(&self.project, m.revision);
                let inflight = self.dispatcher.in_flight_ids();
                self.toolpath_cache.reconcile_project_statuses(
                    &mut self.project,
                    m.revision,
                    &inflight,
                );
            }
            self.logs.push(format!("Undo: {lbl}"));
        }
    }

    fn perform_redo(&mut self) {
        if let Some(lbl) = self.history.redo(&mut self.project) {
            if let Some(m) = &self.model {
                self.dispatcher.cancel_stale_jobs(&self.project, m.revision);
                let inflight = self.dispatcher.in_flight_ids();
                self.toolpath_cache.reconcile_project_statuses(
                    &mut self.project,
                    m.revision,
                    &inflight,
                );
            }
            self.logs.push(format!("Redo: {lbl}"));
        }
    }

    fn draw_top_bars(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();

        // Global shortcuts (`Ctrl+G`, `Ctrl+Z`, `Ctrl+Y`, `Ctrl+S`, `Ctrl+O`, `Esc`, `P`, `F`)
        let (gen_shortcut, undo_shortcut, redo_shortcut, save_shortcut, open_shortcut, esc_key) =
            ctx.input(|i| {
                let cmd = i.modifiers.command || i.modifiers.ctrl;
                (
                    cmd && i.key_pressed(egui::Key::G),
                    cmd && !i.modifiers.shift && i.key_pressed(egui::Key::Z),
                    (cmd && i.key_pressed(egui::Key::Y))
                        || (cmd && i.modifiers.shift && i.key_pressed(egui::Key::Z)),
                    cmd && i.key_pressed(egui::Key::S),
                    cmd && i.key_pressed(egui::Key::O),
                    i.key_pressed(egui::Key::Escape),
                )
            });

        if gen_shortcut {
            self.trigger_generate_dirty(Some(self.active_setup_idx), None);
        }
        if undo_shortcut && !ctx.egui_wants_keyboard_input() {
            self.perform_undo();
        }
        if redo_shortcut && !ctx.egui_wants_keyboard_input() {
            self.perform_redo();
        }
        if save_shortcut {
            self.save_project_dialog();
        }
        if open_shortcut {
            self.open_project_dialog(&ctx);
        }
        if esc_key {
            if self.pick_mode != ViewportPickMode::InspectEntity {
                self.pick_mode = ViewportPickMode::InspectEntity;
                self.highlight.preview_tab_marker_wcs = None;
            } else if self.dispatcher.is_busy() {
                let n = self.dispatcher.cancel_all();
                self.logs
                    .push(format!("Cancelled {n} in-flight generation job(s)."));
            }
        }

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
                    if ui.button("New Project").clicked() {
                        self.history.clear();
                        self.project = ProjectDocument::new("Untitled Job");
                        self.project_file_path = None;
                        self.active_setup_idx = 0;
                        self.inspector_focus = InspectorFocus::Setup { setup_idx: 0 };
                        ui.close();
                    }
                    if ui.button("Open Project (.chip.toml)… (Ctrl+O)").clicked() {
                        ui.close();
                        self.open_project_dialog(&ctx);
                    }
                    if ui.button("Save Project (Ctrl+S)").clicked() {
                        ui.close();
                        self.save_project_dialog();
                    }
                    if ui.button("Save Project As…").clicked() {
                        ui.close();
                        if let Some(path) = rfd::FileDialog::new()
                            .set_file_name("job.chip.toml")
                            .save_file()
                        {
                            self.save_project_to_disk(&path);
                        }
                    }
                    if ui.button("Export G-Code (.nc / .ngc)…").clicked() {
                        ui.close();
                        focus_workspace_pane(&mut self.tree, WorkspacePane::GCodePreview);
                        let cad_doc = self.model.as_ref().map(|m| &m.doc);
                        let cad_rev = self.model.as_ref().map_or(0, |m| m.revision);
                        let proj_dir = self.project_file_path.as_deref().and_then(Path::parent);
                        self.gcode_panel.ensure_profiles_loaded(proj_dir);
                        self.gcode_panel.rebuild_preview_if_needed(
                            &self.project,
                            self.active_setup_idx,
                            cad_doc,
                            cad_rev,
                            &self.toolpath_cache,
                            self.project_file_path.as_deref(),
                        );
                        self.gcode_panel.trigger_export_action(
                            &self.project,
                            self.active_setup_idx,
                            &self.toolpath_cache,
                            cad_rev,
                            &self.sim_panel,
                            &mut self.logs,
                        );
                    }
                    ui.separator();
                    if ui.button("Import STEP Model…").clicked() {
                        ui.close();
                        self.open_step_dialog(&ctx);
                    }
                    if ui.button("Load CAM Torture Block").clicked() {
                        ui.close();
                        self.spawn_cam_torture_sample_load(true, ctx.clone());
                    }
                    if ui.button("Load Multi-Feature Sample Bracket").clicked() {
                        ui.close();
                        self.spawn_multi_feature_sample_load(false, true, ctx.clone());
                    }
                    if ui.button("Load 30°-Tilted Sample Bracket").clicked() {
                        ui.close();
                        self.spawn_multi_feature_sample_load(true, false, ctx.clone());
                    }
                    if ui.button("Load M1 Sample Plate").clicked() {
                        ui.close();
                        self.spawn_sample_plate_load(ctx.clone());
                    }
                    ui.separator();
                    if ui.button("Quit").clicked() {
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                });

                ui.menu_button("Edit", |ui| {
                    let undo_lbl = match self.history.peek_undo_label() {
                        Some(l) => format!("Undo {l} (Ctrl+Z)"),
                        None => "Undo (Ctrl+Z)".to_string(),
                    };
                    if ui
                        .add_enabled(self.history.can_undo(), egui::Button::new(undo_lbl))
                        .clicked()
                    {
                        self.perform_undo();
                        ui.close();
                    }
                    let redo_lbl = match self.history.peek_redo_label() {
                        Some(l) => format!("Redo {l} (Ctrl+Y)"),
                        None => "Redo (Ctrl+Y)".to_string(),
                    };
                    if ui
                        .add_enabled(self.history.can_redo(), egui::Button::new(redo_lbl))
                        .clicked()
                    {
                        self.perform_redo();
                        ui.close();
                    }
                });

                ui.menu_button("Setup", |ui| {
                    if ui.button("New Setup").clicked() {
                        self.history
                            .record_before_edit("Add Setup", None, &self.project);
                        let n = self.project.setups.len() + 1;
                        self.project.setups.push(Setup::new(
                            format!("setup_{n}"),
                            format!("Setup {n} — Op {n}"),
                            WorkOffsetSlot::G55,
                        ));
                        self.active_setup_idx = self.project.setups.len() - 1;
                        self.inspector_focus = InspectorFocus::Setup {
                            setup_idx: self.active_setup_idx,
                        };
                        ui.close();
                    }
                    if ui.button("Align View to Active Setup Top").clicked() {
                        self.align_camera_to_active_setup();
                        ui.close();
                    }
                    if ui.button("Validate Project Invariants").clicked() {
                        let rep = validate_project(
                            &self.project,
                            self.project_file_path.as_deref().and_then(Path::parent),
                            self.model.as_ref().map(|m| m.doc.as_ref()),
                        );
                        self.logs.push(format!(
                            "Validation check: {} error(s), {} warning(s).",
                            rep.error_count(),
                            rep.warning_count()
                        ));
                        for d in rep.diagnostics {
                            self.logs
                                .push(format!("  [{:?}] {}: {}", d.severity, d.scope, d.message));
                        }
                        ui.close();
                    }
                });

                ui.menu_button("Operations", |ui| {
                    if ui
                        .button("Generate Dirty in Active Setup (Ctrl+G)")
                        .clicked()
                    {
                        self.trigger_generate_dirty(Some(self.active_setup_idx), None);
                        ui.close();
                    }
                    if ui.button("Generate All Setups").clicked() {
                        self.trigger_generate_dirty(None, None);
                        ui.close();
                    }
                    if self.dispatcher.is_busy() && ui.button("Cancel Generation (Esc)").clicked() {
                        self.dispatcher.cancel_all();
                        ui.close();
                    }
                    ui.separator();
                    for kind in OperationKind::ALL {
                        if ui.button(format!("+ {}", kind.label())).clicked()
                            && self.active_setup_idx < self.project.setups.len()
                        {
                            self.history.record_before_edit(
                                format!("Add {}", kind.label()),
                                None,
                                &self.project,
                            );
                            let s_idx = self.active_setup_idx;
                            let n = self.project.setups[s_idx].operations.len() + 1;
                            let mut op = Operation::new(
                                format!("op_{s_idx}_{n}"),
                                format!("{} {n}", kind.default_name()),
                                kind,
                            );
                            if let Some(t) = self
                                .project
                                .tools
                                .iter()
                                .find(|t| kind.is_compatible_with_tool(t))
                            {
                                op.tool_id = Some(t.id.clone());
                                op.preset_id = t.presets.first().map(|p| p.id.clone());
                            }
                            self.project.setups[s_idx].operations.push(op);
                            let o_idx = self.project.setups[s_idx].operations.len() - 1;
                            self.inspector_focus = InspectorFocus::Operation {
                                setup_idx: s_idx,
                                op_idx: o_idx,
                            };
                            self.op_tab = if kind == OperationKind::Facing {
                                OperationTab::Passes
                            } else {
                                OperationTab::Geometry
                            };
                            self.pick_mode =
                                default_pick_mode_for_op(s_idx, o_idx, kind, self.op_tab);
                            ui.close();
                        }
                    }
                });

                ui.menu_button("Tools", |ui| {
                    if ui.button("Tool Library Manager…").clicked() {
                        self.tool_library_modal.open = true;
                        ui.close();
                    }
                });

                ui.menu_button("View", |ui| {
                    if ui.button("Reset Default Workspace Layout").clicked() {
                        self.tree = create_default_workspace_tree();
                        ui.close();
                    }
                    ui.separator();
                    ui.checkbox(
                        &mut self.highlight.show_toolpaths,
                        "Show 3D Generated Toolpaths",
                    );
                    ui.checkbox(&mut self.highlight.show_rapids, "Show Rapid Moves (G0)");
                    ui.checkbox(
                        &mut self.show_all_setup_toolpaths,
                        "Show All Operations in Active Setup",
                    );
                    ui.separator();
                    ui.checkbox(
                        &mut self.highlight.show_grid,
                        "Show Active Setup WCS XY Grid",
                    );
                    ui.checkbox(&mut self.highlight.show_stock, "Show Translucent Stock");
                    ui.checkbox(&mut self.highlight.ghost_part, "Ghost Part Translucency");
                    ui.checkbox(
                        &mut self.show_z_planes,
                        "Show Z-Height Planes in Heights Tab",
                    );
                    ui.separator();
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
                    if ui.button("Align View to Active Setup Top (+Z)").clicked() {
                        self.align_camera_to_active_setup();
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
                if ui
                    .button("Open")
                    .on_hover_text("Open .chip.toml Project (Ctrl+O)")
                    .clicked()
                {
                    self.open_project_dialog(&ctx);
                }
                if ui
                    .button("Save")
                    .on_hover_text("Save .chip.toml Project (Ctrl+S)")
                    .clicked()
                {
                    self.save_project_dialog();
                }
                if ui
                    .add_enabled(self.history.can_undo(), egui::Button::new("Undo"))
                    .clicked()
                {
                    self.perform_undo();
                }
                if ui
                    .add_enabled(self.history.can_redo(), egui::Button::new("Redo"))
                    .clicked()
                {
                    self.perform_redo();
                }

                ui.separator();
                if self.dispatcher.is_busy() {
                    ui.spinner();
                    if ui
                        .button(format!("Cancel ({})", self.dispatcher.in_flight_count()))
                        .on_hover_text("Cancel in-flight toolpath generation (Esc)")
                        .clicked()
                    {
                        self.dispatcher.cancel_all();
                    }
                } else if ui
                    .button("Generate (Ctrl+G)")
                    .on_hover_text("Generate dirty 2.5D toolpaths in active Setup on rayon pool")
                    .clicked()
                {
                    self.trigger_generate_dirty(Some(self.active_setup_idx), None);
                }

                if ui
                    .button("Simulate")
                    .on_hover_text("Run 2.5D Z-Dexel Heightmap Stock Removal & Crash Checks")
                    .clicked()
                {
                    focus_workspace_pane(&mut self.tree, WorkspacePane::SimulationTimeline);
                    self.trigger_active_setup_simulation(&ctx);
                }
                if ui
                    .button("Post G-Code")
                    .on_hover_text("Open G-Code Preview & Safety-Gated Export")
                    .clicked()
                {
                    focus_workspace_pane(&mut self.tree, WorkspacePane::GCodePreview);
                }

                if ui.button("Tool Library").clicked() {
                    self.tool_library_modal.open = true;
                }
                if ui
                    .button("Setup Top")
                    .on_hover_text("Align camera straight down Active Setup +Z")
                    .clicked()
                {
                    self.align_camera_to_active_setup();
                }
                ui.toggle_value(&mut self.highlight.show_toolpaths, "Toolpaths");
                ui.toggle_value(&mut self.highlight.show_rapids, "Rapids");
                ui.toggle_value(&mut self.highlight.show_stock, "Stock");
                ui.toggle_value(&mut self.highlight.ghost_part, "Ghost");

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
                let mut u_sel = self.project.display_units;
                ui.selectable_value(&mut u_sel, DisplayUnit::Millimeter, "mm");
                ui.selectable_value(&mut u_sel, DisplayUnit::Inch, "in");
                if u_sel != self.project.display_units {
                    self.history
                        .record_before_edit("Change Display Units", None, &self.project);
                    self.project.display_units = u_sel;
                    self.refresh_dependent_expressions();
                }
            });
        });
    }

    fn handle_self_test(&mut self, ctx: &egui::Context) {
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
            // Position scrubber near 82% so most pockets/bores are machined AND the 3D cutter assembly is actively cutting!
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

        egui::CentralPanel::default()
            .frame(egui::Frame::NONE.fill(egui::Color32::from_rgb(18, 20, 24)))
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
                &mut self.app.project,
                &mut self.app.history,
                &self.app.toolpath_cache,
                cad_ref,
                cad_rev,
                &mut self.app.active_setup_idx,
                &mut self.app.inspector_focus,
                &mut self.app.setup_tab,
                &mut self.app.op_tab,
                &mut self.app.pick_mode,
            );

            if let Some(act) = action {
                match act {
                    BrowserAction::OpenStepDialog => self.app.open_step_dialog(&ctx),
                    BrowserAction::LoadMultiFeatureSample { tilted } => {
                        self.app
                            .spawn_multi_feature_sample_load(tilted, !tilted, ctx.clone());
                    }
                    BrowserAction::LoadCamTortureBlock => {
                        self.app.spawn_cam_torture_sample_load(true, ctx.clone());
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

        // Interactive Manual Holding Tab placement/removal on 3D Contour edges!
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
            // Raycast hover and click picking in exact f64 millimeters
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

        // Interactive 27-point Stock/Model WCS Origin overlay when editing Setup WCS
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

        // Active Pick Mode banner at top-left of viewport
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

        // Bottom-left HUD status pill
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
        egui::Frame::NONE
            .inner_margin(8.0)
            .show(ui, |ui| match self.app.inspector_focus {
                InspectorFocus::Setup { setup_idx } => {
                    let resolved = self.app.active_resolved_setup();
                    let cad_ref = self.app.model.as_ref().map(|m| m.doc.as_ref());
                    let align = show_setup_inspector(
                        ui,
                        &mut self.app.project,
                        setup_idx,
                        &mut self.app.history,
                        cad_ref,
                        resolved
                            .as_ref()
                            .map(|r| r.as_ref().map_err(String::as_str)),
                        &mut self.app.setup_tab,
                        &mut self.app.pick_mode,
                        &mut self.app.show_27pt_dots,
                        &mut self.app.show_z_planes,
                    );
                    if align {
                        self.app.align_camera_to_active_setup();
                    }
                }
                InspectorFocus::Operation { setup_idx, op_idx } => {
                    let resolved_opt = self.app.active_resolved_setup().and_then(Result::ok);
                    let cad_ref = self.app.model.as_ref().map(|m| m.doc.as_ref());
                    show_operation_inspector(
                        ui,
                        &mut self.app.project,
                        setup_idx,
                        op_idx,
                        &self.app.global_tools,
                        &mut self.app.history,
                        cad_ref,
                        resolved_opt.as_ref(),
                        &mut self.app.op_tab,
                        &mut self.app.pick_mode,
                        &mut self.app.lock_chipload,
                        &mut self.app.show_z_planes,
                        &mut self.app.tool_library_modal.open,
                    );
                }
                InspectorFocus::CadEntity => {
                    self.ui_cad_entity_inspector(ui);
                }
            });
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
            let proj_dir = self.app.project_file_path.as_deref().and_then(Path::parent);
            pending_action = self.app.sim_panel.show_ui(
                ui,
                &self.app.project,
                self.app.active_setup_idx,
                cad_doc,
                cad_rev,
                &self.app.toolpath_cache,
                proj_dir,
                &mut self.app.logs,
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
            pending_action = self.app.gcode_panel.show_ui(
                ui,
                &mut self.app.project,
                &mut self.app.history,
                self.app.active_setup_idx,
                cad_doc,
                cad_rev,
                &self.app.toolpath_cache,
                self.app.project_file_path.as_deref(),
                &self.app.sim_panel,
                &mut self.app.logs,
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
