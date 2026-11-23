pub mod commands;
pub mod loader;
pub mod menus;
pub mod modals;
pub mod pane_behavior;
pub mod shortcuts;
pub mod toolbar;
pub mod viewport_controller;

use std::{
    collections::HashMap,
    hash::{Hash as _, Hasher as _},
    path::{Path, PathBuf},
    sync::Arc,
    time::Instant,
};

use chip_cad::{CadDocument, CadPicker, PickFilter, TopologyGraph};
use chip_cam::{
    ExtractedSelectionSummary, FixtureBody, OperationStatus, ResolvedFixtureSolid, ResolvedSetup,
    TriMesh3D, extract_selection_features_with_sketches, resolve_fixture_body,
};
use chip_math::{Aabb3, DVec3, EvalContext};
use chip_project::{
    JobDispatcher, ModelEntry, ProjectDocument, ToolpathCache, UndoHistory, compute_sha256_hex,
    is_chipz_path, is_mesh_path, load_model_entry_cad_doc_transformed, portable_model_path,
};
use chip_sim::SimDiagnostic;
use chip_tools::{ToolLibrary, load_or_default_global_library};
use crossbeam_channel::{Receiver, Sender};

use crate::{
    app::{loader::LoadJobResult, modals::ModalStates, pane_behavior::PaneBehavior},
    camera::CadCamera,
    capture::CaptureSpec,
    layout::{
        WorkspacePane, create_workspace_tree_with_shares, extract_workspace_shares,
        focus_workspace_pane,
    },
    panels::{
        InspectorFocus, OperationTab, SetupTab, ViewportPickMode,
        about::APP_VERSION,
        gcode_preview::GCodePreviewPanelState,
        simulation::{SimTriggerArgs, SimulationPanelState},
        welcome_gallery::SampleProjectId,
    },
    prefs::{UserPreferences, load_or_default_prefs, save_prefs},
    theme::{AppTheme, ThemeTokens, apply_theme},
    viewport::{HighlightState, ViewportRenderer},
    widgets::ExpressionInput,
};

pub(crate) struct LoadedModelState {
    pub raw_doc: Arc<CadDocument>,
    pub applied_transform: chip_cad::ModelTransform,
    pub doc: Arc<CadDocument>,
    pub picker: CadPicker,
    pub topo: TopologyGraph,
    pub raw_step_bytes: Vec<u8>,
    pub source_path: Option<PathBuf>,
    pub revision: u64,
}

/// Cached Setup WCS + resolved 3D workholding fixtures + focused Operation 2.5D selection extraction
/// so idle 60 FPS frames perform zero redundant B-Rep loop chaining, stock/WCS, or fixture recomputations.
#[derive(Clone)]
pub(crate) struct CachedFrameGeo {
    pub sig: u64,
    pub resolved_setup: Option<Result<ResolvedSetup, String>>,
    pub resolved_fixtures: Vec<ResolvedFixtureSolid>,
    pub fixture_mesh_arcs: Vec<Arc<TriMesh3D>>,
    pub focused_extracted: Option<Result<ExtractedSelectionSummary, String>>,
}

/// Optional startup demo mode for CLI flags (`--sample`, `--demo-*`, `--welcome`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum StartupDemoMode {
    #[default]
    None,
    CamTorture,
    Pure2dPanel,
    EngravedNameplate,
    VCarveSign,
    Sample(SampleProjectId),
    WelcomeScreen,
}

/// Self-test automation configuration for headless CI & tutorial screenshot capture.
pub struct SelfTestConfig {
    pub screenshot_path: PathBuf,
    pub capture: CaptureSpec,
    pub(crate) frames_rendered: u32,
    pub(crate) idle_wait_frames: u32,
    pub(crate) generate_triggered: bool,
    pub(crate) capture_applied: bool,
    pub(crate) requested: bool,
}

impl SelfTestConfig {
    pub fn new(screenshot_path: PathBuf) -> Self {
        Self::with_capture(screenshot_path, CaptureSpec::default())
    }

    pub fn with_capture(screenshot_path: PathBuf, capture: CaptureSpec) -> Self {
        Self {
            screenshot_path,
            capture,
            frames_rendered: 0,
            idle_wait_frames: 0,
            generate_triggered: false,
            capture_applied: false,
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
    pub(crate) allow_close_without_prompt: bool,
    pub(crate) last_window_title: String,

    pub(crate) history: UndoHistory,
    pub(crate) toolpath_cache: ToolpathCache,
    pub(crate) dispatcher: JobDispatcher,
    pub(crate) machines: Vec<chip_tools::MachineDefinition>,
    pub(crate) global_tools: ToolLibrary,
    pub(crate) prefs: UserPreferences,
    pub(crate) last_saved_prefs: UserPreferences,
    pub(crate) modals: ModalStates,
    pub(crate) sim_panel: SimulationPanelState,
    pub(crate) gcode_panel: GCodePreviewPanelState,
    pub(crate) auto_sim_after_gen: bool,

    pub(crate) active_setup_idx: usize,
    pub(crate) inspector_focus: InspectorFocus,
    pub(crate) setup_tab: SetupTab,
    pub(crate) op_tab: OperationTab,
    pub(crate) pick_mode: ViewportPickMode,
    pub(crate) selected_fixture_idx: Option<usize>,
    pub(crate) selected_sketch_entity_idx: Option<usize>,
    pub(crate) sketch_snap_grid_mm: Option<f64>,
    pub(crate) sketch_author_start_xy: Option<chip_math::DVec2>,
    pub(crate) lock_chipload: bool,
    pub(crate) lock_model_uniform_scale: bool,
    pub(crate) show_27pt_dots: bool,
    pub(crate) show_z_planes: bool,
    pub(crate) show_all_setup_toolpaths: bool,
    pub(crate) hidden_operation_ids: std::collections::HashSet<String>,

    pub(crate) model: Option<LoadedModelState>,
    pub(crate) fixture_models: HashMap<String, Arc<CadDocument>>,
    pub(crate) fixture_models_revision: u64,
    pub(crate) empty_cad: Arc<CadDocument>,
    pub(crate) next_revision: u64,
    pub(crate) loading_in_flight: bool,
    pub(crate) load_generation: u64,
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
        Self::new_with_options(cc, initial_path, demo_mode, None, self_test)
    }

    pub fn new_with_demo_and_theme(
        cc: &eframe::CreationContext<'_>,
        initial_path: Option<PathBuf>,
        demo_mode: StartupDemoMode,
        initial_theme: AppTheme,
        self_test: Option<SelfTestConfig>,
    ) -> Self {
        Self::new_with_options(cc, initial_path, demo_mode, Some(initial_theme), self_test)
    }

    pub fn new_with_options(
        cc: &eframe::CreationContext<'_>,
        initial_path: Option<PathBuf>,
        demo_mode: StartupDemoMode,
        theme_override: Option<AppTheme>,
        self_test: Option<SelfTestConfig>,
    ) -> Self {
        let mut prefs = if self_test.is_some() {
            UserPreferences::default()
        } else {
            load_or_default_prefs()
        };
        let active_theme = theme_override.unwrap_or(prefs.theme);
        prefs.theme = active_theme;
        apply_theme(&cc.egui_ctx, active_theme);

        let wgpu_render_state = cc.wgpu_render_state.clone();
        let viewport_renderer = wgpu_render_state
            .as_ref()
            .map(|rs| ViewportRenderer::new(&rs.device));
        #[cfg(feature = "gpu-compute")]
        let shared_gpu_ctx = wgpu_render_state.as_ref().map(|rs| {
            let info = rs.adapter.get_info();
            Arc::new(chip_sim::GpuSimContext::from_shared(
                Arc::new(rs.device.clone()),
                Arc::new(rs.queue.clone()),
                chip_sim::ActiveSimBackendInfo {
                    is_gpu: true,
                    adapter_name: info.name,
                    backend_api: format!("{:?}", info.backend),
                },
            ))
        });

        let (job_tx, job_rx) = crossbeam_channel::unbounded();
        let machines = chip_tools::load_available_machines(None);
        let default_mid = prefs.default_machine_id.clone();
        let initial_machine = machines
            .iter()
            .find(|m| m.id.eq_ignore_ascii_case(&default_mid))
            .or_else(|| machines.first());
        let global_tools = initial_machine
            .map(chip_tools::MachineDefinition::to_tool_library)
            .unwrap_or_else(load_or_default_global_library);
        let repaint_ctx = cc.egui_ctx.clone();
        let dispatcher = JobDispatcher::new(Some(Arc::new(move || {
            repaint_ctx.request_repaint();
        })));

        let mut camera = CadCamera {
            mouse_preset: prefs.mouse_preset,
            ..CadCamera::default()
        };
        camera.set_orbit_mode(prefs.orbit_mode);
        if !prefs.orthographic && camera.projection.is_orthographic() {
            camera.toggle_projection();
        }

        let mut initial_project = ProjectDocument::new("Untitled Job");
        initial_project.display_units = prefs.default_units;
        if let Some(m) = initial_machine
            && m.id != chip_tools::DEFAULT_MACHINE_ID
        {
            initial_project.apply_machine_defaults(m, true);
        }

        let mut app = Self {
            tree: create_workspace_tree_with_shares(prefs.layout_shares),
            theme: active_theme,
            camera,
            viewport_renderer,
            wgpu_render_state,
            project: initial_project,
            project_file_path: None,
            bundle_extract_dir: None,
            clean_undo_len: Some(0),
            allow_close_without_prompt: false,
            last_window_title: String::new(),
            history: UndoHistory::new(100),
            toolpath_cache: ToolpathCache::new(128),
            dispatcher,
            machines,
            global_tools,
            last_saved_prefs: prefs.clone(),
            #[cfg(feature = "gpu-compute")]
            sim_panel: SimulationPanelState::with_prefs_and_gpu(
                prefs.sim_engine,
                prefs.sim_backend,
                shared_gpu_ctx,
            ),
            #[cfg(not(feature = "gpu-compute"))]
            sim_panel: SimulationPanelState::with_prefs(prefs.sim_engine, prefs.sim_backend),
            prefs,
            modals: ModalStates::new(),
            gcode_panel: GCodePreviewPanelState::default(),
            auto_sim_after_gen: true,
            active_setup_idx: 0,
            inspector_focus: InspectorFocus::Setup { setup_idx: 0 },
            setup_tab: SetupTab::Wcs,
            op_tab: OperationTab::Passes,
            pick_mode: ViewportPickMode::InspectEntity,
            selected_fixture_idx: None,
            selected_sketch_entity_idx: None,
            sketch_snap_grid_mm: None,
            sketch_author_start_xy: None,
            lock_chipload: true,
            lock_model_uniform_scale: true,
            show_27pt_dots: true,
            show_z_planes: true,
            show_all_setup_toolpaths: true,
            hidden_operation_ids: std::collections::HashSet::new(),
            model: None,
            fixture_models: HashMap::new(),
            fixture_models_revision: 0,
            empty_cad: Arc::new(CadDocument::empty()),
            next_revision: 1,
            loading_in_flight: false,
            load_generation: 0,
            job_tx,
            job_rx,
            last_disk_check: Instant::now(),
            external_step_modified: false,
            last_reload_banner: None,
            cached_frame_geo: None,
            pick_filter: PickFilter::Any,
            highlight: HighlightState {
                theme: active_theme,
                show_toolpaths: true,
                show_rapids: true,
                show_stock: true,
                ghost_part: false,
                ..HighlightState::default()
            },
            wb_tool_dia: ExpressionInput::new(6.35, Some("1/4 in")),
            wb_stepover: ExpressionInput::new(2.54, Some("tool_d * 0.4")),
            wb_clearance_z: ExpressionInput::new(26.0, Some("model_top_z + 10 mm")),
            logs: vec![format!("ChipCutter v{APP_VERSION} initialized.")],
            self_test,
        };
        app.highlight.show_toolpaths = app.prefs.show_toolpaths;
        app.highlight.show_rapids = app.prefs.show_rapids;
        app.highlight.show_stock = app.prefs.show_stock;
        app.highlight.show_fixtures = app.prefs.show_fixtures;
        app.highlight.ghost_part = app.prefs.ghost_part;
        if let Some(st) = &app.self_test {
            if let Some(eng) = st.capture.sim_engine {
                app.sim_panel.engine = eng;
            }
            if let Some(backend) = st.capture.sim_backend {
                app.sim_panel.backend_choice = backend;
            }
            if let Some(res) = st.capture.sim_resolution {
                app.sim_panel.resolution = res;
            }
            app.sim_panel.show_error_heatmap = st.capture.sim_heatmap;
        }

        if let Some(path) = initial_path {
            app.open_any_path(&path, cc.egui_ctx.clone());
        } else {
            match demo_mode {
                StartupDemoMode::Pure2dPanel => app.load_pure2d_router_panel_demo(&cc.egui_ctx),
                StartupDemoMode::EngravedNameplate => {
                    app.load_engraved_nameplate_demo(&cc.egui_ctx)
                }
                StartupDemoMode::VCarveSign => app.load_vcarve_sign_demo(&cc.egui_ctx),
                StartupDemoMode::CamTorture => {
                    app.spawn_cam_torture_sample_load(true, cc.egui_ctx.clone())
                }
                StartupDemoMode::Sample(sample_id) => {
                    app.load_sample_project(sample_id, &cc.egui_ctx)
                }
                StartupDemoMode::WelcomeScreen => {
                    app.modals.welcome.open = true;
                }
                StartupDemoMode::None => {
                    if app.self_test.is_some() {
                        app.spawn_cam_torture_sample_load(true, cc.egui_ctx.clone());
                    } else if app.prefs.show_welcome_on_startup {
                        app.modals.welcome.open = true;
                    }
                }
            }
        }

        app
    }

    pub(crate) fn open_any_path(&mut self, path: &Path, egui_ctx: egui::Context) {
        if is_chipz_path(path) {
            self.load_chipz_from_disk(path, egui_ctx);
        } else if path
            .to_string_lossy()
            .to_ascii_lowercase()
            .ends_with(".toml")
        {
            self.load_project_from_disk(path, egui_ctx);
        } else {
            let ext_low = path
                .extension()
                .and_then(|s| s.to_str())
                .unwrap_or_default()
                .to_ascii_lowercase();
            if ext_low == "dxf" || ext_low == "svg" {
                self.record_recent_file(path);
                self.modals
                    .vector_import
                    .load_from_disk_path(path.to_path_buf());
                self.modals.vector_import.open = true;
            } else if is_mesh_path(path) {
                if self.self_test.is_some() {
                    self.spawn_step_file_load(path.to_path_buf(), egui_ctx);
                } else {
                    self.record_recent_file(path);
                    self.modals
                        .mesh_import
                        .load_from_disk_path(path.to_path_buf());
                    self.modals.mesh_import.open = true;
                }
            } else {
                self.spawn_step_file_load(path.to_path_buf(), egui_ctx);
            }
        }
    }

    #[cfg(test)]
    pub(crate) fn new_headless_for_test(egui_ctx: &egui::Context) -> Self {
        let prefs = UserPreferences::default();
        let active_theme = prefs.theme;
        let (job_tx, job_rx) = crossbeam_channel::unbounded();
        let machines = chip_tools::load_available_machines(None);
        let global_tools = machines
            .first()
            .map(chip_tools::MachineDefinition::to_tool_library)
            .unwrap_or_else(load_or_default_global_library);
        let repaint_ctx = egui_ctx.clone();
        let dispatcher = JobDispatcher::new(Some(Arc::new(move || {
            repaint_ctx.request_repaint();
        })));
        let initial_project = ProjectDocument::new("Untitled Job");
        Self {
            tree: create_workspace_tree_with_shares(prefs.layout_shares),
            theme: active_theme,
            camera: CadCamera::default(),
            viewport_renderer: None,
            wgpu_render_state: None,
            project: initial_project,
            project_file_path: None,
            bundle_extract_dir: None,
            clean_undo_len: Some(0),
            allow_close_without_prompt: false,
            last_window_title: String::new(),
            history: UndoHistory::new(100),
            toolpath_cache: ToolpathCache::new(128),
            dispatcher,
            machines,
            global_tools,
            last_saved_prefs: prefs.clone(),
            sim_panel: SimulationPanelState::with_prefs(prefs.sim_engine, prefs.sim_backend),
            prefs,
            modals: ModalStates::new(),
            gcode_panel: GCodePreviewPanelState::default(),
            auto_sim_after_gen: false,
            active_setup_idx: 0,
            inspector_focus: InspectorFocus::Setup { setup_idx: 0 },
            setup_tab: SetupTab::Wcs,
            op_tab: OperationTab::Passes,
            pick_mode: ViewportPickMode::InspectEntity,
            selected_fixture_idx: None,
            selected_sketch_entity_idx: None,
            sketch_snap_grid_mm: None,
            sketch_author_start_xy: None,
            lock_chipload: true,
            lock_model_uniform_scale: true,
            show_27pt_dots: true,
            show_z_planes: true,
            show_all_setup_toolpaths: true,
            hidden_operation_ids: std::collections::HashSet::new(),
            model: None,
            fixture_models: HashMap::new(),
            fixture_models_revision: 0,
            empty_cad: Arc::new(CadDocument::empty()),
            next_revision: 1,
            loading_in_flight: false,
            load_generation: 0,
            job_tx,
            job_rx,
            last_disk_check: Instant::now(),
            external_step_modified: false,
            last_reload_banner: None,
            cached_frame_geo: None,
            pick_filter: PickFilter::Any,
            highlight: HighlightState {
                theme: active_theme,
                show_toolpaths: true,
                show_rapids: true,
                show_stock: true,
                ghost_part: false,
                ..HighlightState::default()
            },
            wb_tool_dia: ExpressionInput::new(6.35, Some("1/4 in")),
            wb_stepover: ExpressionInput::new(2.54, Some("tool_d * 0.4")),
            wb_clearance_z: ExpressionInput::new(26.0, Some("model_top_z + 10 mm")),
            logs: Vec::new(),
            self_test: None,
        }
    }

    pub(crate) fn record_recent_file(&mut self, path: &Path) {
        self.prefs.push_recent_file(path);
        if self.self_test.is_none() && !cfg!(test) {
            let _ = save_prefs(&self.prefs);
            self.last_saved_prefs = self.prefs.clone();
        }
    }

    pub(crate) fn load_sample_project(&mut self, id: SampleProjectId, ctx: &egui::Context) {
        match id {
            SampleProjectId::CamTortureBlock => {
                self.spawn_cam_torture_sample_load(true, ctx.clone());
            }
            SampleProjectId::AdaptiveRest => {
                self.spawn_adaptive_rest_demo_load(ctx.clone());
            }
            SampleProjectId::Pure2dPanel => {
                self.load_pure2d_router_panel_demo(ctx);
            }
            SampleProjectId::EngravedNameplate => {
                self.load_engraved_nameplate_demo(ctx);
            }
            SampleProjectId::VCarveSignInlay => {
                self.load_vcarve_sign_demo(ctx);
            }
            SampleProjectId::MultiFeatureBracket => {
                self.spawn_multi_feature_sample_load(false, true, ctx.clone());
            }
            SampleProjectId::Tilted30Bracket => {
                self.spawn_multi_feature_sample_load(true, false, ctx.clone());
            }
            SampleProjectId::CadRevisionV1 => {
                self.spawn_revision_demo_load(false, ctx.clone());
            }
            SampleProjectId::CadRevisionV2Reload => {
                self.spawn_revision_demo_load(true, ctx.clone());
            }
            SampleProjectId::SamplePlate => {
                self.spawn_sample_plate_load(ctx.clone());
            }
            SampleProjectId::SculptedMeshDome => {
                self.spawn_sculpted_mesh_dome_load(ctx.clone());
            }
            SampleProjectId::ReliefWaveMesh => {
                self.spawn_relief_wave_mesh_load(ctx.clone());
            }
            SampleProjectId::RotaryIndexedShaft => {
                self.spawn_rotary_shaft_load(ctx.clone());
            }
            SampleProjectId::RotaryWrappedEngraving => {
                self.load_rotary_wrapped_engraving_demo(ctx);
            }
            SampleProjectId::RotaryHelicalFlute => {
                self.spawn_rotary_helical_flute_load(ctx.clone());
            }
            SampleProjectId::ThreadMillManifold => {
                self.spawn_thread_mill_manifold_load(ctx.clone());
            }
            SampleProjectId::TwoSidedViseWorkholding => {
                self.spawn_two_sided_vise_load(ctx.clone());
            }
            SampleProjectId::UndercutAndDeburrBlock => {
                self.spawn_undercut_deburr_load(ctx.clone());
            }
            SampleProjectId::FeedOptPocket => {
                self.spawn_feed_opt_pocket_load(ctx.clone());
            }
            SampleProjectId::FlipJigSoftJaws => {
                self.spawn_flip_jig_softjaws_load(ctx.clone());
            }
            SampleProjectId::CurvedDeburr3D => {
                self.spawn_curved_deburr_3d_load(ctx.clone());
            }
            SampleProjectId::DovetailRailKeyway => {
                self.spawn_dovetail_rail_keyway_load(ctx.clone());
            }
            SampleProjectId::SvgBadgeSign => {
                self.load_svg_badge_sign_demo(ctx);
            }
            SampleProjectId::Mesh3mfBracket => {
                self.spawn_mesh_3mf_bracket_load(ctx.clone());
            }
        }
    }

    pub(crate) fn sync_and_save_prefs_if_changed(&mut self, pointer_down: bool) {
        if self.self_test.is_some() || pointer_down || cfg!(test) {
            return;
        }
        self.prefs.theme = self.theme;
        self.prefs.mouse_preset = self.camera.mouse_preset;
        self.prefs.orbit_mode = self.camera.orbit_mode;
        self.prefs.orthographic = self.camera.projection.is_orthographic();
        self.prefs.show_toolpaths = self.highlight.show_toolpaths;
        self.prefs.show_rapids = self.highlight.show_rapids;
        self.prefs.show_stock = self.highlight.show_stock;
        self.prefs.show_fixtures = self.highlight.show_fixtures;
        self.prefs.ghost_part = self.highlight.ghost_part;
        self.prefs.sim_engine = self.sim_panel.engine;
        self.prefs.sim_backend = self.sim_panel.backend_choice;
        let shares = extract_workspace_shares(&self.tree);
        if !shares.approx_eq(self.prefs.layout_shares) {
            self.prefs.layout_shares = shares;
        }
        if self.prefs != self.last_saved_prefs {
            let _ = save_prefs(&self.prefs);
            self.last_saved_prefs = self.prefs.clone();
        }
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
        self.fixture_models_revision.hash(&mut h);
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
            format!("{:?}", setup.fixtures).hash(&mut h);
            setup.auto_avoid_fixtures_on_rapids.hash(&mut h);
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

            let mut resolved_fixtures = Vec::new();
            let mut fixture_mesh_arcs = Vec::new();
            if let (Some(Ok(rs)), Some(setup)) = (
                &resolved_setup,
                self.project.setups.get(self.active_setup_idx),
            ) && !setup.fixtures.is_empty()
            {
                let eval_ctx = rs.eval_context(self.project.display_units);
                let stock_aabb = rs.stock.aabb_wcs();
                for fix in &setup.fixtures {
                    if !fix.enabled {
                        continue;
                    }
                    if let Ok(sol) = resolve_fixture_body(fix, &stock_aabb, &eval_ctx, &mut |id| {
                        self.fixture_models.get(id).map(Arc::as_ref)
                    }) {
                        let mesh_arc = Arc::new(sol.mesh_wcs.clone());
                        fixture_mesh_arcs.push(mesh_arc);
                        resolved_fixtures.push(sol);
                    }
                }
            }

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
                resolved_fixtures,
                fixture_mesh_arcs,
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
        let machine_z = self.active_machine().map(|m| m.travel_mm.z_mm);
        let fix_docs: std::collections::BTreeMap<String, Arc<CadDocument>> = self
            .fixture_models
            .iter()
            .map(|(k, v)| (k.clone(), Arc::clone(v)))
            .collect();
        self.dispatcher.set_workholding_context(fix_docs, machine_z);
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

    pub(crate) fn active_machine(&self) -> Option<&chip_tools::MachineDefinition> {
        let mid = self.project.effective_machine_id();
        self.machines
            .iter()
            .find(|m| m.id.eq_ignore_ascii_case(mid))
            .or_else(|| self.machines.first())
    }

    pub(crate) fn sync_active_machine_tools(&mut self) {
        if let Some(m) = self.active_machine().cloned() {
            self.global_tools = m.to_tool_library();
        }
    }

    pub(crate) fn reload_fixture_models_from_disk(&mut self) {
        self.fixture_models.clear();
        let proj_dir = self.active_project_dir().map(Path::to_path_buf);
        for (id, doc) in self.project.load_fixture_cad_documents(proj_dir.as_deref()) {
            self.fixture_models.insert(id, Arc::new(doc));
        }
        self.fixture_models_revision = self.fixture_models_revision.wrapping_add(1);
        self.cached_frame_geo = None;
    }

    pub(crate) fn import_fixture_model_dialog(&mut self, setup_idx: usize) {
        if setup_idx >= self.project.setups.len() {
            return;
        }
        let Some(path) = rfd::FileDialog::new()
            .add_filter(
                "3D Fixture Models (.step, .stp, .stl, .obj, .3mf)",
                &[
                    "step", "stp", "stl", "obj", "3mf", "STEP", "STP", "STL", "OBJ", "3MF",
                ],
            )
            .pick_file()
        else {
            return;
        };

        let bytes = match std::fs::read(&path) {
            Ok(b) => b,
            Err(err) => {
                self.logs.push(format!(
                    "ERROR reading fixture model '{}': {err}",
                    path.display()
                ));
                return;
            }
        };
        let cad_doc = match load_model_entry_cad_doc_transformed(
            &path,
            None,
            &chip_cad::ModelTransform::IDENTITY,
        ) {
            Ok(d) => d,
            Err(err) => {
                self.logs.push(format!(
                    "ERROR parsing fixture model '{}': {err}",
                    path.display()
                ));
                return;
            }
        };

        let sha = compute_sha256_hex(&bytes);
        let rel_path = portable_model_path(self.project_file_path.as_deref(), &path);
        let stem = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("Fixture")
            .to_string();

        let mut m_num = self.project.models.len() + 1;
        let model_id = loop {
            let cand = format!("fixture_model_{m_num}");
            if self.project.model_by_id(&cand).is_none() {
                break cand;
            }
            m_num += 1;
        };
        let mut f_num = self.project.setups[setup_idx].fixtures.len() + 1;
        let fix_id = loop {
            let cand = format!("imported_{f_num}");
            if !self.project.setups[setup_idx]
                .fixtures
                .iter()
                .any(|f| f.id == cand)
            {
                break cand;
            }
            f_num += 1;
        };

        self.history
            .record_before_edit("Import 3D Fixture Model", None, &self.project);
        self.project
            .models
            .push(ModelEntry::new_fixture(model_id.clone(), rel_path, sha));
        self.project.setups[setup_idx]
            .fixtures
            .push(FixtureBody::default_imported_model(
                fix_id,
                stem.clone(),
                model_id.clone(),
            ));
        for op in &mut self.project.setups[setup_idx].operations {
            op.mark_dirty();
        }
        self.selected_fixture_idx = Some(self.project.setups[setup_idx].fixtures.len() - 1);
        self.highlight.show_fixtures = true;
        self.fixture_models.insert(model_id, Arc::new(cad_doc));
        self.fixture_models_revision = self.fixture_models_revision.wrapping_add(1);
        self.cached_frame_geo = None;
        self.logs
            .push(format!("Imported 3D fixture model '{stem}'."));
    }

    pub(crate) fn sync_loaded_model_transform(&mut self) -> bool {
        let Some(desired_xf) = self.project.primary_part_model().map(|m| m.transform) else {
            return false;
        };
        let needs_rebuild = self
            .model
            .as_ref()
            .is_some_and(|m| m.applied_transform != desired_xf);
        if !needs_rebuild {
            return false;
        }
        let Some(loaded) = self.model.as_mut() else {
            return false;
        };
        let new_doc = if desired_xf.is_identity() {
            Arc::clone(&loaded.raw_doc)
        } else {
            Arc::new(loaded.raw_doc.transformed(&desired_xf))
        };
        loaded.picker = CadPicker::build(&new_doc);
        loaded.topo = TopologyGraph::build(&new_doc);
        loaded.doc = new_doc;
        loaded.applied_transform = desired_xf;
        loaded.revision = self.next_revision;
        self.next_revision += 1;

        let new_rev = loaded.revision;
        self.dispatcher.cancel_stale_jobs(&self.project, new_rev);
        self.sim_panel.cancel_in_flight();
        self.sim_panel.active_result = None;
        self.gcode_panel.invalidate_preview();
        self.highlight.hovered = None;
        self.highlight.selected = None;
        self.cached_frame_geo = None;
        self.refresh_dependent_expressions();
        true
    }

    pub(crate) fn is_project_dirty(&self) -> bool {
        self.clean_undo_len != Some(self.history.undo_len())
    }

    pub(crate) fn should_intercept_close(&self) -> bool {
        self.is_project_dirty() && !self.allow_close_without_prompt && self.self_test.is_none()
    }

    pub(crate) fn request_app_close(&mut self, ctx: &egui::Context) {
        if self.should_intercept_close() {
            self.modals.show_unsaved_close = true;
        } else {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }

    pub(crate) fn handle_viewport_close_guard(&mut self, ctx: &egui::Context) {
        if ctx.input(|i| i.viewport().close_requested()) && self.should_intercept_close() {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.modals.show_unsaved_close = true;
        }
    }

    pub(crate) fn perform_undo(&mut self) {
        if let Some(lbl) = self.history.undo(&mut self.project) {
            self.sync_active_machine_tools();
            self.sync_loaded_model_transform();
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
            self.sync_active_machine_tools();
            self.sync_loaded_model_transform();
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
        self.sync_loaded_model_transform();
        self.sim_panel.update_playback_and_shortcuts(&ctx);
        self.sync_viewport_scene_state();
        self.draw_top_bars(ui);
        self.show_primary_modals(&ctx);

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

        self.show_overlay_modals(&ctx);
        self.handle_viewport_close_guard(&ctx);
        self.show_unsaved_close_prompt(&ctx);

        let ptr_down = ctx.input(|i| i.pointer.any_down());
        self.sync_and_save_prefs_if_changed(ptr_down);
        self.handle_self_test(&ctx);
    }
}
