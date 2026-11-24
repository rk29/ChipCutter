mod cad_apply;
mod sample_loaders;

use std::{
    path::{Path, PathBuf},
    time::Instant,
};

use chip_cad::{CadDocument, CadPicker, MeshUnitKind, TopologyGraph};
use chip_project::{
    ProjectDocument, compute_sha256_hex, create_private_temp_extract_dir, export_chipz_bundle,
    external_or_suspicious_assets_referenced, is_chipz_path, is_flatpak_document_portal_path,
    load_model_bytes_cad_doc, portable_model_path, resolve_model_path, unpack_chipz_bundle,
};

use crate::{
    app::ChipCutterApp,
    panels::{InspectorFocus, OperationTab, SetupTab, ViewportPickMode},
};

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum LoadPurpose {
    FreshImport { update_model_entry: bool },
    SeedBlankSample,
    SeedMultiFeatureBracket,
    SeedCamTortureBlock,
    SeedAdaptiveRestDemo,
    SeedRevisionDemoV1,
    SeedSculptedMeshDome,
    SeedReliefWaveMesh,
    SeedRotaryIndexedShaft,
    SeedRotaryHelicalFlute,
    SeedThreadMillManifold,
    SeedTwoSidedViseWorkholding,
    SeedUndercutAndDeburrBlock,
    SeedFeedOptPocket,
    SeedFlipJigSoftJaws,
    SeedCurvedDeburr3D,
    SeedDovetailRailKeyway,
    SeedMesh3mfBracket,
    ReloadRebind { update_path: bool },
}

pub(crate) struct LoadedPayload {
    pub doc: CadDocument,
    pub picker: CadPicker,
    pub topo: TopologyGraph,
    pub raw_step_bytes: Vec<u8>,
    pub source_path: Option<PathBuf>,
    pub unit_override: Option<MeshUnitKind>,
    pub initial_transform: Option<chip_cad::ModelTransform>,
    pub sha256_hex: String,
    pub purpose: LoadPurpose,
    pub pre_seeded_project: Option<ProjectDocument>,
}

pub(crate) enum LoadJobResult {
    Loaded {
        generation: u64,
        payload: Box<LoadedPayload>,
        elapsed_ms: u128,
    },
    Failed {
        generation: u64,
        error: String,
    },
}

impl ChipCutterApp {
    pub(crate) fn begin_model_load_generation(&mut self) -> u64 {
        self.load_generation = self.load_generation.wrapping_add(1);
        self.loading_in_flight = true;
        self.load_generation
    }

    pub(crate) fn cancel_in_flight_model_loads(&mut self) {
        self.load_generation = self.load_generation.wrapping_add(1);
        self.loading_in_flight = false;
        while self.job_rx.try_recv().is_ok() {}
    }

    /// Cancels all background work and resets every piece of project-scoped runtime state
    /// (loaded 3D model, toolpath cache, simulation results, G-code preview, undo history,
    /// viewport selections/overlays, bundle extract dir, and machine/post definitions) before
    /// installing a replacement `ProjectDocument`.
    pub(crate) fn prepare_for_project_replacement(
        &mut self,
        project_dir: Option<&Path>,
        new_bundle_extract_dir: Option<PathBuf>,
    ) {
        self.cancel_in_flight_model_loads();
        self.dispatcher.reset();
        self.toolpath_cache.clear();
        self.sim_panel.reset_for_new_project();
        self.gcode_panel.reset_for_project(project_dir);
        self.auto_sim_after_gen = false;
        self.clear_bundle_extract_dir();
        self.bundle_extract_dir = new_bundle_extract_dir;
        self.history.clear();
        self.clean_undo_len = Some(0);
        self.modals.show_unsaved_close = false;
        self.allow_close_without_prompt = false;
        self.external_step_modified = false;
        self.last_reload_banner = None;
        self.model = None;
        self.fixture_models.clear();
        self.fixture_models_revision = self.fixture_models_revision.wrapping_add(1);
        self.selected_fixture_idx = None;
        self.cached_frame_geo = None;
        self.boundary_preview_cache.clear();
        self.active_setup_idx = 0;
        self.hidden_operation_ids.clear();
        self.inspector_focus = InspectorFocus::Setup { setup_idx: 0 };
        self.setup_tab = SetupTab::Wcs;
        self.op_tab = OperationTab::Passes;
        self.pick_mode = ViewportPickMode::InspectEntity;
        self.selected_sketch_entity_idx = None;
        self.sketch_author_start_xy = None;
        self.highlight.clear_project_scene_state();
        self.modals.mesh_import.open = false;
        self.modals.vector_import.open = false;
        self.machines = chip_tools::load_available_machines(project_dir);
    }

    pub(crate) fn open_step_dialog(&mut self, ctx: &egui::Context) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("STEP 3D Models", &["step", "stp", "STEP", "STP"])
            .pick_file()
        {
            self.spawn_step_file_load(path, ctx.clone());
        }
    }

    pub(crate) fn open_project_dialog(&mut self, ctx: &egui::Context) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter(
                "ChipCutter Project & Portable Bundle (.chip.toml, .chipz)",
                &["toml", "TOML", "chipz", "CHIPZ"],
            )
            .pick_file()
        {
            if is_chipz_path(&path) {
                self.load_chipz_from_disk(&path, ctx.clone());
            } else {
                self.load_project_from_disk(&path, ctx.clone());
            }
        }
    }

    pub(crate) fn save_project_dialog(&mut self) -> bool {
        if let Some(existing) = self.project_file_path.clone() {
            self.save_project_to_disk(&existing);
        } else if let Some(path) = rfd::FileDialog::new()
            .set_file_name("job.chip.toml")
            .save_file()
        {
            self.save_project_to_disk(&path);
        }
        !self.is_project_dirty()
    }

    pub(crate) fn load_project_from_disk(&mut self, path: &Path, egui_ctx: egui::Context) {
        match ProjectDocument::load_from_path(path) {
            Ok(doc) => {
                self.prepare_for_project_replacement(path.parent(), None);
                self.project = doc;
                self.project_file_path = Some(path.to_path_buf());
                self.record_recent_file(path);
                self.sync_active_machine_tools();
                self.refresh_dependent_expressions();
                self.reload_fixture_models_from_disk();
                self.logs
                    .push(format!("Opened project '{}'.", path.display()));
                if let Some(first_model) = self.project.primary_part_model() {
                    let step_path = resolve_model_path(path.parent(), &first_model.path);
                    let unit_override = first_model.unit_override;
                    self.spawn_model_file_load_with_options(
                        step_path,
                        unit_override,
                        LoadPurpose::FreshImport {
                            update_model_entry: false,
                        },
                        egui_ctx,
                    );
                } else {
                    self.fit_scene_to_viewport();
                }
            }
            Err(err) => {
                self.logs
                    .push(format!("ERROR opening project '{}': {err}", path.display()));
            }
        }
    }

    pub(crate) fn load_chipz_from_disk(&mut self, archive_path: &Path, egui_ctx: egui::Context) {
        let extract_dir = match create_private_temp_extract_dir("chipcutter_gui_chipz") {
            Ok(dir) => dir,
            Err(err) => {
                self.logs.push(format!(
                    "ERROR creating private temp directory for '{}': {err}",
                    archive_path.display()
                ));
                return;
            }
        };
        match unpack_chipz_bundle(archive_path, &extract_dir) {
            Ok(unpacked) => {
                self.prepare_for_project_replacement(Some(&extract_dir), Some(extract_dir.clone()));
                self.project = unpacked.project;
                self.project_file_path = None;
                self.record_recent_file(archive_path);
                self.sync_active_machine_tools();
                self.refresh_dependent_expressions();
                self.reload_fixture_models_from_disk();
                self.logs.push(format!(
                    "Unpacked portable bundle '{}' ({} file(s)).",
                    archive_path.display(),
                    unpacked.extracted_files.len()
                ));
                if let Some(first_model) = self.project.primary_part_model() {
                    let step_path = resolve_model_path(Some(&extract_dir), &first_model.path);
                    let unit_override = first_model.unit_override;
                    self.spawn_model_file_load_with_options(
                        step_path,
                        unit_override,
                        LoadPurpose::FreshImport {
                            update_model_entry: false,
                        },
                        egui_ctx,
                    );
                } else {
                    self.fit_scene_to_viewport();
                }
            }
            Err(err) => {
                let _ = std::fs::remove_dir_all(&extract_dir);
                self.logs.push(format!(
                    "ERROR unpacking portable bundle '{}': {err}",
                    archive_path.display()
                ));
            }
        }
    }

    pub(crate) fn save_project_to_disk(&mut self, path: &Path) {
        for entry in &mut self.project.models {
            let cur = PathBuf::from(&entry.path);
            if cur.is_absolute() {
                entry.path = portable_model_path(Some(path), &cur);
            }
        }
        for sk in &mut self.project.sketches {
            if let Some(asset) = &mut sk.source_asset {
                let cur = PathBuf::from(&asset.path);
                if cur.is_absolute() {
                    asset.path = portable_model_path(Some(path), &cur);
                }
            }
        }
        match self.project.save_to_path(path) {
            Ok(()) => {
                self.project_file_path = Some(path.to_path_buf());
                self.record_recent_file(path);
                self.clean_undo_len = Some(self.history.undo_len());
                self.logs
                    .push(format!("Saved project to '{}'.", path.display()));
            }
            Err(err) => {
                self.logs
                    .push(format!("ERROR saving project '{}': {err}", path.display()));
            }
        }
    }

    pub(crate) fn export_chipz_dialog(&mut self) {
        let default_stem = self
            .project_file_path
            .as_deref()
            .and_then(Path::file_stem)
            .and_then(|s| s.to_str())
            .map(|s| s.trim_end_matches(".chip"))
            .unwrap_or("job");
        let Some(dest_path) = rfd::FileDialog::new()
            .add_filter("ChipCutter Portable Bundle (.chipz)", &["chipz"])
            .set_file_name(format!("{default_stem}.chipz"))
            .save_file()
        else {
            return;
        };

        let mut fallback: Vec<(&str, &[u8])> = Vec::new();
        if let (Some(m), Some(first)) = (&self.model, self.project.primary_part_model()) {
            fallback.push((first.id.as_str(), m.raw_step_bytes.as_slice()));
            fallback.push((first.path.as_str(), m.raw_step_bytes.as_slice()));
        }
        let proj_dir = self
            .project_file_path
            .as_deref()
            .and_then(Path::parent)
            .or(self.bundle_extract_dir.as_deref());
        let external_assets = external_or_suspicious_assets_referenced(&self.project, proj_dir);
        if !external_assets.is_empty() {
            let names: Vec<String> = external_assets
                .iter()
                .map(|p| p.display().to_string())
                .collect();
            self.logs.push(format!(
                "WARN exporting .chipz: project references external/non-standard local file(s): {}",
                names.join(", ")
            ));
        }
        match export_chipz_bundle(
            &self.project,
            proj_dir,
            &fallback,
            self.gcode_panel.custom_profiles(),
            &dest_path,
        ) {
            Ok(sum) => {
                self.logs.push(format!(
                    "Exported portable bundle '{}' ({} bytes, {} model(s), {} vector(s), {} custom post(s), {} custom machine(s)).",
                    dest_path.display(),
                    sum.archive_size_bytes,
                    sum.model_files.len(),
                    sum.vector_files.len(),
                    sum.custom_post_files.len(),
                    sum.custom_machine_files.len()
                ));
            }
            Err(err) => {
                self.logs.push(format!(
                    "ERROR exporting portable bundle '{}': {err}",
                    dest_path.display()
                ));
            }
        }
    }

    pub(crate) fn reload_active_step_model(&mut self, egui_ctx: &egui::Context) {
        let disk_path = self
            .model
            .as_ref()
            .and_then(|m| m.source_path.clone())
            .or_else(|| {
                let first = self.project.primary_part_model()?;
                let resolved = resolve_model_path(self.active_project_dir(), &first.path);
                resolved.is_file().then_some(resolved)
            });

        if let Some(step_path) = disk_path {
            self.spawn_step_file_load_with_purpose(
                step_path,
                LoadPurpose::ReloadRebind { update_path: false },
                egui_ctx.clone(),
            );
        } else {
            self.logs.push(
                "Current STEP model is an in-memory sample; choose a STEP file on disk to associatively re-bind (or try '-> Rev v2')."
                    .to_string(),
            );
            self.replace_step_dialog(egui_ctx);
        }
    }

    pub(crate) fn replace_step_dialog(&mut self, ctx: &egui::Context) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("STEP 3D Models", &["step", "stp", "STEP", "STP"])
            .pick_file()
        {
            self.spawn_step_file_load_with_purpose(
                path,
                LoadPurpose::ReloadRebind { update_path: true },
                ctx.clone(),
            );
        }
    }

    pub(crate) fn spawn_step_file_load(&mut self, path: PathBuf, egui_ctx: egui::Context) {
        self.record_recent_file(&path);
        self.spawn_step_file_load_with_purpose(
            path,
            LoadPurpose::FreshImport {
                update_model_entry: true,
            },
            egui_ctx,
        );
    }

    pub(crate) fn spawn_step_file_load_with_purpose(
        &mut self,
        path: PathBuf,
        purpose: LoadPurpose,
        egui_ctx: egui::Context,
    ) {
        self.spawn_model_file_load_with_options(path, None, purpose, egui_ctx);
    }

    pub(crate) fn spawn_model_file_load_with_options(
        &mut self,
        path: PathBuf,
        unit_override: Option<MeshUnitKind>,
        purpose: LoadPurpose,
        egui_ctx: egui::Context,
    ) {
        let generation = self.begin_model_load_generation();
        self.logs.push(format!(
            "Loading 3D model '{}' on background worker…",
            path.display()
        ));
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let res = match std::fs::read(&path) {
                Ok(bytes) => {
                    let name = path
                        .file_name()
                        .and_then(|s| s.to_str())
                        .unwrap_or("part.step")
                        .to_string();
                    parse_model_bytes_job(
                        generation,
                        name,
                        bytes,
                        Some(path),
                        unit_override,
                        purpose,
                        t0,
                    )
                }
                Err(e) => LoadJobResult::Failed {
                    generation,
                    error: format_model_read_error(&path, &e, is_running_in_flatpak_sandbox()),
                },
            };
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub(crate) fn spawn_mesh_bytes_load(
        &mut self,
        source_name: String,
        raw_bytes: Vec<u8>,
        source_path: Option<PathBuf>,
        unit_override: Option<MeshUnitKind>,
        initial_transform: chip_cad::ModelTransform,
        egui_ctx: egui::Context,
    ) {
        if let Some(p) = &source_path {
            self.record_recent_file(p);
        }
        let generation = self.begin_model_load_generation();
        self.logs.push(format!(
            "Importing 3D mesh '{source_name}' on background worker…"
        ));
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let mut res = parse_model_bytes_job(
                generation,
                source_name,
                raw_bytes,
                source_path,
                unit_override,
                LoadPurpose::FreshImport {
                    update_model_entry: true,
                },
                t0,
            );
            if let LoadJobResult::Loaded { payload, .. } = &mut res {
                payload.initial_transform = Some(initial_transform);
            }
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub(crate) fn poll_background_jobs(&mut self, egui_ctx: &egui::Context) {
        while let Ok(msg) = self.job_rx.try_recv() {
            let msg_gen = match &msg {
                LoadJobResult::Loaded { generation, .. }
                | LoadJobResult::Failed { generation, .. } => *generation,
            };
            if !self.loading_in_flight || msg_gen != self.load_generation {
                continue;
            }
            self.loading_in_flight = false;
            match msg {
                LoadJobResult::Loaded {
                    payload,
                    elapsed_ms,
                    ..
                } => {
                    self.apply_loaded_cad_payload(*payload, elapsed_ms);
                }
                LoadJobResult::Failed { error, .. } => {
                    self.logs.push(format!("ERROR loading 3D model: {error}"));
                }
            }
        }

        // Throttled (every 2s) external STEP modification check on disk (`09-persistence-cli.md` §1)
        if !self.loading_in_flight && self.last_disk_check.elapsed().as_secs_f32() >= 2.0 {
            self.last_disk_check = Instant::now();
            if let (Some(m), Some(first)) = (&self.model, self.project.primary_part_model())
                && let Some(disk_path) = &m.source_path
                && let Ok(meta) = std::fs::metadata(disk_path)
                && meta.is_file()
                && meta.len() <= 64 * 1024 * 1024
                && let Ok(bytes) = std::fs::read(disk_path)
            {
                let disk_sha = compute_sha256_hex(&bytes);
                self.external_step_modified = disk_sha != first.sha256;
            }
        }

        // Poll background toolpath generation worker events (`JobDispatcher`)
        if let Some((_, cad_rev)) = self.effective_cad_arc() {
            self.dispatcher.cancel_stale_jobs(&self.project, cad_rev);
            let gen_logs =
                self.dispatcher
                    .poll_events(&mut self.project, &mut self.toolpath_cache, cad_rev);
            let had_gen_completion = !gen_logs.is_empty();
            self.logs.extend(gen_logs);

            if self.auto_sim_after_gen && had_gen_completion && !self.dispatcher.is_busy() {
                self.auto_sim_after_gen = false;
                self.trigger_active_setup_simulation(egui_ctx);
            }
        }

        let _ = self.sim_panel.poll_worker(&mut self.logs);
    }
}

fn is_running_in_flatpak_sandbox() -> bool {
    std::env::var_os("FLATPAK_ID").is_some() || Path::new("/.flatpak-info").exists()
}

fn format_model_read_error(path: &Path, err: &std::io::Error, in_flatpak_sandbox: bool) -> String {
    let base = format!("failed to read {}: {err}", path.display());
    if is_flatpak_document_portal_path(path)
        || (in_flatpak_sandbox
            && matches!(
                err.kind(),
                std::io::ErrorKind::NotFound | std::io::ErrorKind::PermissionDenied
            ))
    {
        format!(
            "{base} (Hint: Flatpak single-file portal mounts cannot access sibling STEP/model files referenced by a .chip.toml project. Grant host filesystem access with `flatpak override --user --filesystem=host io.github.rk29.ChipCutter`, open a self-contained `.chipz` bundle, or use File -> Replace STEP Model to locate the model.)"
        )
    } else {
        base
    }
}

pub(super) fn parse_step_bytes_job(
    generation: u64,
    name: String,
    raw_step_bytes: Vec<u8>,
    source_path: Option<PathBuf>,
    purpose: LoadPurpose,
    t0: Instant,
) -> LoadJobResult {
    parse_model_bytes_job(
        generation,
        name,
        raw_step_bytes,
        source_path,
        None,
        purpose,
        t0,
    )
}

pub(super) fn parse_model_bytes_job(
    generation: u64,
    name: String,
    raw_step_bytes: Vec<u8>,
    source_path: Option<PathBuf>,
    unit_override: Option<MeshUnitKind>,
    purpose: LoadPurpose,
    t0: Instant,
) -> LoadJobResult {
    let sha256_hex = compute_sha256_hex(&raw_step_bytes);
    match load_model_bytes_cad_doc(&name, &raw_step_bytes, unit_override) {
        Ok(doc) => {
            let picker = CadPicker::build(&doc);
            let topo = TopologyGraph::build(&doc);
            LoadJobResult::Loaded {
                generation,
                payload: Box::new(LoadedPayload {
                    doc,
                    picker,
                    topo,
                    raw_step_bytes,
                    source_path,
                    unit_override,
                    initial_transform: None,
                    sha256_hex,
                    purpose,
                    pre_seeded_project: None,
                }),
                elapsed_ms: t0.elapsed().as_millis(),
            }
        }
        Err(e) => LoadJobResult::Failed {
            generation,
            error: e.to_string(),
        },
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::{
        sync::Arc,
        time::{Duration, Instant},
    };

    use chip_cad::{
        FaceId, PickTarget, sample_relief_wave_obj_text, sample_sculpted_dome_stl_bytes,
    };
    use chip_cam::{MotionRole, ToolpathBuilder};
    use chip_math::DVec3;
    use chip_post::PostOutputFile;
    use chip_project::{
        ModelEntry, ProjectDocument, build_pure2d_router_panel_demo_project,
        build_relief_wave_mesh_project, compute_sha256_hex, create_private_temp_extract_dir,
        export_chipz_bundle,
    };
    use chip_sim::{
        MachineKinematicsConfig, SetupSimulationInput, SimOperationInput, SimResolution,
        SimVerificationConfig, run_setup_simulation,
    };

    use super::*;

    fn wait_for_model_load(app: &mut ChipCutterApp, ctx: &egui::Context) {
        let deadline = Instant::now() + Duration::from_secs(15);
        while app.loading_in_flight && Instant::now() < deadline {
            app.poll_background_jobs(ctx);
            std::thread::sleep(Duration::from_millis(5));
        }
        app.poll_background_jobs(ctx);
        assert!(
            !app.loading_in_flight,
            "background model load did not finish before timeout"
        );
    }

    fn seed_active_stl_project_with_sim(app: &mut ChipCutterApp, ctx: &egui::Context) {
        let stl_bytes = sample_sculpted_dome_stl_bytes();
        app.spawn_mesh_bytes_load(
            "previous_part.stl".to_string(),
            stl_bytes,
            None,
            None,
            chip_cad::ModelTransform::IDENTITY,
            ctx.clone(),
        );
        wait_for_model_load(app, ctx);
        assert_eq!(
            app.model.as_ref().expect("STL should load").doc.source_name,
            "previous_part.stl"
        );

        // Populate history, highlights, G-code cache, and active simulation result from the STL project.
        app.history
            .record_before_edit("STL Edit", None, &app.project);
        app.highlight.selected = Some(PickTarget::Face(FaceId(0)));
        app.highlight.hovered = Some(PickTarget::Face(FaceId(0)));
        app.gcode_panel.cached_files.push(PostOutputFile {
            filename: "previous_part.nc".to_string(),
            content: "G0 Z10\nM30\n".to_string(),
            line_count: 2,
            max_line_chars: 6,
            tool_numbers: vec![1],
            operation_names: vec!["STL Cut".to_string()],
            est_time_sec: 12.0,
            warnings: Vec::new(),
        });
        app.auto_sim_after_gen = true;

        let tool = app.global_tools.tools[0].clone();
        app.project.ensure_tool_embedded(&tool);
        let doc = Arc::clone(&app.model.as_ref().unwrap().doc);
        let resolved = app.project.setups[0]
            .resolve(&doc, app.project.display_units)
            .unwrap();
        let mut b = ToolpathBuilder::new("stl_op");
        b.rapid_to(DVec3::new(0.0, 0.0, 45.0));
        b.linear_to(DVec3::new(0.0, 0.0, 30.0), 400.0, MotionRole::Plunge);
        b.linear_to(DVec3::new(10.0, 0.0, 30.0), 800.0, MotionRole::Cut);
        let tp = b.finish();
        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "stl_op",
                op_name: "STL Cut",
                tool: &tool,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
        })
        .unwrap();
        app.sim_panel.active_result = Some(Arc::new(sim));
        app.sync_viewport_scene_state();
        app.sim_panel.flush_remesh_sync();
        app.sync_viewport_scene_state();
        assert!(app.highlight.sim_stock_source.is_some());
    }

    #[test]
    fn loading_pure2d_chip_toml_clears_previous_stl_model_sim_gcode_and_stale_loads() {
        let ctx = egui::Context::default();
        let mut app = ChipCutterApp::new_headless_for_test(&ctx);
        seed_active_stl_project_with_sim(&mut app, &ctx);

        // Queue a stale in-flight STL result from the previous generation to ensure it cannot resurrect.
        let stale_gen = app.load_generation;
        let stale_stl = sample_sculpted_dome_stl_bytes();
        let stale_msg = parse_model_bytes_job(
            stale_gen,
            "stale_previous.stl".to_string(),
            stale_stl,
            None,
            None,
            LoadPurpose::FreshImport {
                update_model_entry: true,
            },
            Instant::now(),
        );

        let tmp = create_private_temp_extract_dir("chipcutter_test_toml_pure2d").unwrap();
        let toml_path = tmp.join("pure2d_panel.chip.toml");
        let pure2d = build_pure2d_router_panel_demo_project(None, &app.global_tools.tools);
        pure2d.save_to_path(&toml_path).unwrap();

        app.load_project_from_disk(&toml_path, ctx.clone());
        let _ = app.job_tx.send(stale_msg);
        app.poll_background_jobs(&ctx);
        app.sync_viewport_scene_state();

        assert!(
            app.model.is_none(),
            "previous STL model must be cleared when loading a Pure-2D .chip.toml"
        );
        assert_eq!(app.project.project_name, pure2d.project_name);
        assert!(app.project.models.is_empty());
        assert!(app.sim_panel.active_result.is_none());
        assert!(app.highlight.sim_stock_source.is_none());
        assert!(app.highlight.sim_tool_mesh.is_none());
        assert!(app.highlight.selected.is_none());
        assert!(app.highlight.hovered.is_none());
        assert!(app.gcode_panel.cached_files.is_empty());
        assert!(!app.history.can_undo());
        assert!(!app.auto_sim_after_gen);

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn loading_chip_toml_with_missing_model_does_not_leave_previous_stl_active() {
        let ctx = egui::Context::default();
        let mut app = ChipCutterApp::new_headless_for_test(&ctx);
        seed_active_stl_project_with_sim(&mut app, &ctx);

        let tmp = create_private_temp_extract_dir("chipcutter_test_toml_missing_model").unwrap();
        let toml_path = tmp.join("missing_model.chip.toml");
        let mut proj = ProjectDocument::new("Project With Missing Model");
        proj.models.push(ModelEntry {
            id: "model_1".to_string(),
            path: "does_not_exist.step".to_string(),
            sha256: "0".repeat(64),
            role: Default::default(),
            unit_override: None,
            transform: chip_cad::ModelTransform::IDENTITY,
        });
        proj.save_to_path(&toml_path).unwrap();

        app.load_project_from_disk(&toml_path, ctx.clone());
        assert!(
            app.model.is_none(),
            "previous STL model must be cleared immediately before spawning the new project's model load"
        );
        assert!(app.sim_panel.active_result.is_none());

        wait_for_model_load(&mut app, &ctx);
        app.sync_viewport_scene_state();

        assert!(
            app.model.is_none(),
            "previous STL must stay cleared even when the new project's referenced model fails to load"
        );
        assert!(app.highlight.sim_stock_source.is_none());
        assert!(
            app.logs
                .iter()
                .any(|l| l.contains("ERROR loading 3D model")),
            "expected model load error in logs, got: {:?}",
            app.logs
        );

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn loading_chipz_bundles_replaces_previous_stl_and_clears_sim_state() {
        let ctx = egui::Context::default();
        let mut app = ChipCutterApp::new_headless_for_test(&ctx);
        seed_active_stl_project_with_sim(&mut app, &ctx);

        let tmp = create_private_temp_extract_dir("chipcutter_test_chipz_switch").unwrap();

        // 1. Export a 3D .chipz containing an OBJ model (distinct from previous_part.stl)
        let obj_bytes = sample_relief_wave_obj_text().into_bytes();
        let obj_sha = compute_sha256_hex(&obj_bytes);
        let obj_proj = build_relief_wave_mesh_project(
            "relief_wave.obj".to_string(),
            &obj_sha,
            &app.global_tools.tools,
        );
        let obj_chipz = tmp.join("relief_wave.chipz");
        export_chipz_bundle(
            &obj_proj,
            None,
            &[("model_1", obj_bytes.as_slice())],
            &[],
            &obj_chipz,
        )
        .unwrap();

        app.load_chipz_from_disk(&obj_chipz, ctx.clone());
        assert!(
            app.model.is_none(),
            "previous STL must be cleared immediately when opening a .chipz bundle"
        );
        assert!(app.sim_panel.active_result.is_none());
        assert!(app.gcode_panel.cached_files.is_empty());

        wait_for_model_load(&mut app, &ctx);
        app.sync_viewport_scene_state();

        assert_eq!(
            app.model
                .as_ref()
                .expect("bundled OBJ should load")
                .doc
                .source_name,
            "relief_wave.obj"
        );
        assert!(
            app.sim_panel.active_result.is_none(),
            "previous STL simulation must not remain active after loading a .chipz bundle"
        );
        assert!(app.highlight.sim_stock_source.is_none());
        assert!(app.highlight.sim_tool_mesh.is_none());

        // 2. Now load a Pure-2D .chipz bundle and confirm the 3D model is completely cleared
        let pure2d = build_pure2d_router_panel_demo_project(None, &app.global_tools.tools);
        let pure2d_chipz = tmp.join("pure2d.chipz");
        export_chipz_bundle(&pure2d, None, &[], &[], &pure2d_chipz).unwrap();

        app.load_chipz_from_disk(&pure2d_chipz, ctx.clone());
        app.poll_background_jobs(&ctx);
        app.sync_viewport_scene_state();

        assert!(app.model.is_none());
        assert!(app.project.models.is_empty());
        assert!(app.sim_panel.active_result.is_none());
        assert!(app.highlight.sim_stock_source.is_none());

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn loading_blank_sample_replaces_previous_stl_project_cleanly() {
        let ctx = egui::Context::default();
        let mut app = ChipCutterApp::new_headless_for_test(&ctx);
        seed_active_stl_project_with_sim(&mut app, &ctx);

        app.spawn_sample_plate_load(ctx.clone());
        wait_for_model_load(&mut app, &ctx);
        app.sync_viewport_scene_state();

        assert_eq!(
            app.model
                .as_ref()
                .expect("sample plate should load")
                .doc
                .source_name,
            "sample_machined_plate.step"
        );
        assert_eq!(app.project.models.len(), 1);
        assert_eq!(app.project.models[0].path, "sample_machined_plate.step");
        assert!(app.sim_panel.active_result.is_none());
        assert!(app.highlight.sim_stock_source.is_none());
        assert!(app.gcode_panel.cached_files.is_empty());
    }

    #[test]
    fn unsaved_changes_guard_prompts_when_dirty_and_clears_after_save() {
        let ctx = egui::Context::default();
        let mut app = ChipCutterApp::new_headless_for_test(&ctx);
        assert!(!app.is_project_dirty());
        assert!(!app.should_intercept_close());

        // Making an edit marks the project dirty and activates the close guard.
        app.history
            .record_before_edit("Rename Project", None, &app.project);
        app.project.project_name = "Modified Part".to_string();
        assert!(app.is_project_dirty());
        assert!(app.should_intercept_close());

        app.request_app_close(&ctx);
        assert!(app.modals.show_unsaved_close);

        // Saving to disk clears the dirty flag so closing proceeds without prompting.
        let tmp = create_private_temp_extract_dir("chipcutter_test_unsaved_guard").unwrap();
        let path = tmp.join("saved.chip.toml");
        app.save_project_to_disk(&path);
        assert!(!app.is_project_dirty());
        assert!(!app.should_intercept_close());

        // Subsequent edit makes it dirty again, and explicit discard bypasses the guard.
        app.history
            .record_before_edit("Second Edit", None, &app.project);
        app.project.project_name = "Modified Again".to_string();
        assert!(app.should_intercept_close());
        app.allow_close_without_prompt = true;
        assert!(!app.should_intercept_close());

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn flatpak_manifest_grants_host_filesystem_for_project_step_models() {
        let manifest_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../packaging/flatpak/io.github.rk29.ChipCutter.yml");
        let manifest = std::fs::read_to_string(&manifest_path)
            .expect("Flatpak manifest must exist in packaging/flatpak/");
        assert!(
            manifest
                .lines()
                .any(|line| line.trim() == "- --filesystem=host"),
            "Flatpak manifest must grant `--filesystem=host` in `finish-args` so `.chip.toml` projects can open sibling/referenced STEP models"
        );
    }

    #[test]
    fn model_read_error_includes_flatpak_hint_only_for_portal_or_sandbox_failures() {
        let enoent = std::io::Error::new(std::io::ErrorKind::NotFound, "No such file or directory");

        let portal_msg = format_model_read_error(
            Path::new("/run/user/1000/doc/abc12345/part.step"),
            &enoent,
            false,
        );
        assert!(portal_msg.contains("Flatpak single-file portal mounts"));
        assert!(portal_msg.contains("Replace STEP Model"));

        let sandbox_msg =
            format_model_read_error(Path::new("/home/user/projects/part.step"), &enoent, true);
        assert!(sandbox_msg.contains("Flatpak single-file portal mounts"));

        let regular_msg =
            format_model_read_error(Path::new("/home/user/projects/part.step"), &enoent, false);
        assert!(!regular_msg.contains("Flatpak"));
        assert!(regular_msg.contains("failed to read /home/user/projects/part.step"));
    }

    #[test]
    fn missing_project_model_can_be_recovered_via_associative_rebind() {
        let ctx = egui::Context::default();
        let mut app = ChipCutterApp::new_headless_for_test(&ctx);

        let tmp = create_private_temp_extract_dir("chipcutter_test_missing_model_rebind").unwrap();
        let toml_path = tmp.join("missing_step.chip.toml");
        let mut proj = ProjectDocument::new("Project Missing STEP");
        proj.models.push(ModelEntry {
            id: "model_1".to_string(),
            path: "missing_original.step".to_string(),
            sha256: "0".repeat(64),
            role: Default::default(),
            unit_override: None,
            transform: chip_cad::ModelTransform::IDENTITY,
        });
        proj.save_to_path(&toml_path).unwrap();

        app.load_project_from_disk(&toml_path, ctx.clone());
        wait_for_model_load(&mut app, &ctx);
        assert!(app.model.is_none());
        assert!(
            app.project.primary_part_model().is_some(),
            "project should still retain its primary part model entry for Replace STEP Model recovery"
        );

        let recovered_step_path = tmp.join("recovered_part.step");
        std::fs::write(
            &recovered_step_path,
            chip_cad::sample_machined_plate_step().unwrap(),
        )
        .unwrap();

        app.spawn_step_file_load_with_purpose(
            recovered_step_path.clone(),
            LoadPurpose::ReloadRebind { update_path: true },
            ctx.clone(),
        );
        wait_for_model_load(&mut app, &ctx);

        assert_eq!(
            app.model
                .as_ref()
                .expect("recovered STEP should load via Replace STEP Model re-bind")
                .doc
                .source_name,
            "recovered_part.step"
        );
        assert_eq!(
            app.project.primary_part_model().unwrap().path,
            "recovered_part.step"
        );

        let _ = std::fs::remove_dir_all(&tmp);
    }
}
