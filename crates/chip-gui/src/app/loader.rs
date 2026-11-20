use std::{
    path::{Path, PathBuf},
    sync::Arc,
    time::Instant,
};

use chip_cad::{
    CadDocument, CadPicker, MeshUnitKind, TopologyGraph, sample_cam_torture_block_step,
    sample_helical_flute_shaft_stl_bytes, sample_machined_plate_step,
    sample_multi_feature_bracket_step, sample_relief_wave_obj_text, sample_revised_part_pair_step,
    sample_rotary_shaft_stl_bytes, sample_sculpted_dome_stl_bytes,
};
use chip_project::{
    ModelEntry, OperationReloadOutcome, ProjectDocument, build_adaptive_rest_demo_project,
    build_cam_torture_project, build_multi_feature_bracket_project,
    build_pure2d_router_panel_demo_project, build_relief_wave_mesh_project,
    build_revision_v1_demo_project, build_rotary_helical_flute_project,
    build_rotary_shaft_preview_project, build_rotary_wrapped_engraving_project,
    build_sculpted_mesh_dome_project, compute_sha256_hex, create_private_temp_extract_dir,
    export_chipz_bundle, external_or_suspicious_assets_referenced, is_chipz_path,
    load_model_bytes_cad_doc, portable_model_path, rebind_project_to_reloaded_cad,
    resolve_model_path, unpack_chipz_bundle,
};

use crate::{
    app::{ChipCutterApp, LoadedModelState},
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
        self.external_step_modified = false;
        self.last_reload_banner = None;
        self.model = None;
        self.cached_frame_geo = None;
        self.active_setup_idx = 0;
        self.inspector_focus = InspectorFocus::Setup { setup_idx: 0 };
        self.setup_tab = SetupTab::Wcs;
        self.op_tab = OperationTab::Passes;
        self.pick_mode = ViewportPickMode::InspectEntity;
        self.selected_sketch_entity_idx = None;
        self.sketch_author_start_xy = None;
        self.highlight.clear_project_scene_state();
        self.mesh_import_modal.open = false;
        self.vector_import_modal.open = false;
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

    pub(crate) fn save_project_dialog(&mut self) {
        if let Some(existing) = self.project_file_path.clone() {
            self.save_project_to_disk(&existing);
        } else if let Some(path) = rfd::FileDialog::new()
            .set_file_name("job.chip.toml")
            .save_file()
        {
            self.save_project_to_disk(&path);
        }
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
                self.logs
                    .push(format!("Opened project '{}'.", path.display()));
                if let Some(first_model) = self.project.models.first() {
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
                self.logs.push(format!(
                    "Unpacked portable bundle '{}' ({} file(s)).",
                    archive_path.display(),
                    unpacked.extracted_files.len()
                ));
                if let Some(first_model) = self.project.models.first() {
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
        if let (Some(m), Some(first)) = (&self.model, self.project.models.first()) {
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
                let first = self.project.models.first()?;
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
                    error: format!("failed to read {}: {e}", path.display()),
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

    pub(crate) fn spawn_sculpted_mesh_dome_load(&mut self, egui_ctx: egui::Context) {
        let generation = self.begin_model_load_generation();
        self.logs
            .push("Generating watertight Sculpted Dome (STL Mesh) sample…".to_string());
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let stl_bytes = sample_sculpted_dome_stl_bytes();
            let res = parse_model_bytes_job(
                generation,
                "sculpted_dome.stl".to_string(),
                stl_bytes,
                None,
                None,
                LoadPurpose::SeedSculptedMeshDome,
                t0,
            );
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub(crate) fn spawn_relief_wave_mesh_load(&mut self, egui_ctx: egui::Context) {
        let generation = self.begin_model_load_generation();
        self.logs
            .push("Generating Relief Wave (OBJ Mesh) 3D finishing sample…".to_string());
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let obj_bytes = sample_relief_wave_obj_text().into_bytes();
            let res = parse_model_bytes_job(
                generation,
                "relief_wave.obj".to_string(),
                obj_bytes,
                None,
                None,
                LoadPurpose::SeedReliefWaveMesh,
                t0,
            );
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub(crate) fn spawn_rotary_shaft_load(&mut self, egui_ctx: egui::Context) {
        let generation = self.begin_model_load_generation();
        self.logs
            .push("Generating Rotary Indexed Shaft (4th-Axis A) sample…".to_string());
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let stl_bytes = sample_rotary_shaft_stl_bytes();
            let res = parse_model_bytes_job(
                generation,
                "rotary_shaft.stl".to_string(),
                stl_bytes,
                None,
                None,
                LoadPurpose::SeedRotaryIndexedShaft,
                t0,
            );
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub(crate) fn spawn_rotary_helical_flute_load(&mut self, egui_ctx: egui::Context) {
        let generation = self.begin_model_load_generation();
        self.logs
            .push("Generating Simultaneous 4-Axis Helical Flute (X+Y+Z+A) sample…".to_string());
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let stl_bytes = sample_helical_flute_shaft_stl_bytes();
            let res = parse_model_bytes_job(
                generation,
                "helical_flute_shaft.stl".to_string(),
                stl_bytes,
                None,
                None,
                LoadPurpose::SeedRotaryHelicalFlute,
                t0,
            );
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub(crate) fn spawn_sample_plate_load(&mut self, egui_ctx: egui::Context) {
        let generation = self.begin_model_load_generation();
        self.logs
            .push("Generating & tessellating sample CNC mounting plate STEP model…".to_string());
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let res = match sample_machined_plate_step() {
                Ok(step_str) => parse_step_bytes_job(
                    generation,
                    "sample_machined_plate.step".to_string(),
                    step_str.into_bytes(),
                    None,
                    LoadPurpose::SeedBlankSample,
                    t0,
                ),
                Err(e) => LoadJobResult::Failed {
                    generation,
                    error: e.to_string(),
                },
            };
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub(crate) fn spawn_multi_feature_sample_load(
        &mut self,
        tilted: bool,
        seed_demo_ops: bool,
        egui_ctx: egui::Context,
    ) {
        let generation = self.begin_model_load_generation();
        self.logs.push(format!(
            "Generating Milestone 2 multi-feature bracket STEP fixture (tilted={tilted})…"
        ));
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let res = match sample_multi_feature_bracket_step(tilted) {
                Ok(step_str) => {
                    let name = if tilted {
                        "sample_multi_feature_tilted.step"
                    } else {
                        "sample_multi_feature_bracket.step"
                    };
                    let purpose = if seed_demo_ops {
                        LoadPurpose::SeedMultiFeatureBracket
                    } else {
                        LoadPurpose::SeedBlankSample
                    };
                    parse_step_bytes_job(
                        generation,
                        name.to_string(),
                        step_str.into_bytes(),
                        None,
                        purpose,
                        t0,
                    )
                }
                Err(e) => LoadJobResult::Failed {
                    generation,
                    error: e.to_string(),
                },
            };
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub(crate) fn spawn_cam_torture_sample_load(
        &mut self,
        seed_demo_ops: bool,
        egui_ctx: egui::Context,
    ) {
        let generation = self.begin_model_load_generation();
        self.logs.push(
            "Generating Milestone 3 CAM Torture Block STEP fixture (island pocket, open step, Ø18 bore, 45° bevel)…"
                .to_string(),
        );
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let res = match sample_cam_torture_block_step() {
                Ok(step_str) => {
                    let purpose = if seed_demo_ops {
                        LoadPurpose::SeedCamTortureBlock
                    } else {
                        LoadPurpose::SeedBlankSample
                    };
                    parse_step_bytes_job(
                        generation,
                        "sample_cam_torture_block.step".to_string(),
                        step_str.into_bytes(),
                        None,
                        purpose,
                        t0,
                    )
                }
                Err(e) => LoadJobResult::Failed {
                    generation,
                    error: e.to_string(),
                },
            };
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub(crate) fn spawn_adaptive_rest_demo_load(&mut self, egui_ctx: egui::Context) {
        let generation = self.begin_model_load_generation();
        self.logs
            .push("Generating Milestone 6 2D Adaptive + Rest Machining STEP job…".to_string());
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let res = match sample_cam_torture_block_step() {
                Ok(step_str) => parse_step_bytes_job(
                    generation,
                    "sample_cam_torture_block.step".to_string(),
                    step_str.into_bytes(),
                    None,
                    LoadPurpose::SeedAdaptiveRestDemo,
                    t0,
                ),
                Err(e) => LoadJobResult::Failed {
                    generation,
                    error: e.to_string(),
                },
            };
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub fn load_rotary_wrapped_engraving_demo(&mut self, egui_ctx: &egui::Context) {
        self.prepare_for_project_replacement(None, None);
        self.project_file_path = None;
        self.project.machine_id = None;
        self.sync_active_machine_tools();
        self.project = build_rotary_wrapped_engraving_project(&self.global_tools.tools);
        self.sync_active_machine_tools();
        self.inspector_focus = InspectorFocus::Operation {
            setup_idx: 0,
            op_idx: 0,
        };
        self.op_tab = OperationTab::Passes;
        self.refresh_dependent_expressions();
        self.fit_scene_to_viewport();
        self.logs.push(
            "Loaded Wrapped Rotary Engraving (Z+X+A — Ø40×80mm cylinder, wrapped 'CHIP' V-Carve + circumferential flute pocket)."
                .to_string(),
        );
        self.auto_sim_after_gen = true;
        self.trigger_generate_dirty(None, None);
        egui_ctx.request_repaint();
    }

    pub fn load_pure2d_router_panel_demo(&mut self, egui_ctx: &egui::Context) {
        self.prepare_for_project_replacement(None, None);
        self.project_file_path = None;
        self.project.machine_id = None;
        self.sync_active_machine_tools();
        self.project = build_pure2d_router_panel_demo_project(None, &self.global_tools.tools);
        self.sync_active_machine_tools();
        self.inspector_focus = InspectorFocus::Operation {
            setup_idx: 0,
            op_idx: 1,
        };
        self.op_tab = OperationTab::Geometry;
        self.refresh_dependent_expressions();
        self.fit_scene_to_viewport();
        self.logs.push(
            "Loaded Pure-2D CNC Router Front Panel (.dxf, zero STEP files — 4 operations: Holes, Island Pocket, Vent Slot, Outer Profile + Tabs)."
                .to_string(),
        );
        self.auto_sim_after_gen = true;
        self.trigger_generate_dirty(None, None);
        egui_ctx.request_repaint();
    }

    pub fn load_engraved_nameplate_demo(&mut self, egui_ctx: &egui::Context) {
        self.prepare_for_project_replacement(None, None);
        self.project_file_path = None;
        self.project.machine_id = None;
        self.sync_active_machine_tools();
        self.project =
            chip_project::build_engraved_nameplate_demo_project(&self.global_tools.tools);
        self.sync_active_machine_tools();
        self.inspector_focus = InspectorFocus::Sketch { sketch_idx: 0 };
        self.selected_sketch_entity_idx = Some(3);
        self.refresh_dependent_expressions();
        self.fit_scene_to_viewport();
        self.logs.push(
            "Loaded Engraved Nameplate + Dual-Mode Typography Demo (Bolt Circle Drill, Ø22 Bore, DejaVuSans-Bold 'CHIP' Pocket, Hershey 1-Pass 'S/N 2026-08' Trace, R10 Plaque + Tabs)."
                .to_string(),
        );
        self.auto_sim_after_gen = true;
        self.trigger_generate_dirty(None, None);
        egui_ctx.request_repaint();
    }

    pub fn load_vcarve_sign_demo(&mut self, egui_ctx: &egui::Context) {
        self.prepare_for_project_replacement(None, None);
        self.project_file_path = None;
        self.project.machine_id = None;
        self.sync_active_machine_tools();
        self.project =
            chip_project::build_vcarve_sign_and_inlay_demo_project(&self.global_tools.tools);
        self.sync_active_machine_tools();
        self.inspector_focus = InspectorFocus::Operation {
            setup_idx: 0,
            op_idx: 0,
        };
        self.op_tab = OperationTab::Passes;
        self.refresh_dependent_expressions();
        self.fit_scene_to_viewport();
        self.logs.push(
            "Loaded V-Carve Sign & Mated V-Inlay Plug Demo (Setup 1: Two-Stage Ø6mm + 60° V-Bit Crest & 'CHIP' + Medial 'CAM v2' + Plaque Tabs; Setup 2: Mirrored Male V-Inlay Plug)."
                .to_string(),
        );
        self.auto_sim_after_gen = true;
        self.trigger_generate_dirty(None, None);
        egui_ctx.request_repaint();
    }

    pub(crate) fn spawn_revision_demo_load(
        &mut self,
        load_v2_rebind: bool,
        egui_ctx: egui::Context,
    ) {
        let generation = self.begin_model_load_generation();
        self.logs.push(if load_v2_rebind {
            "Reloading CAD Revision v2 STEP fixture (resized pocket, shifted Ø14 bore, 1 deleted Ø6 hole)…"
                .to_string()
        } else {
            "Generating Milestone 5 CAD Revision v1 STEP fixture & seeding CAM job…".to_string()
        });
        let seed_v1_tools =
            if load_v2_rebind && self.project.setups.iter().all(|s| s.operations.is_empty()) {
                Some(self.global_tools.tools.clone())
            } else {
                None
            };
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let res = match sample_revised_part_pair_step() {
                Ok((v1_str, v2_str)) => {
                    let pre_seeded = if let Some(tools) = seed_v1_tools {
                        let v1_bytes = v1_str.as_bytes();
                        let v1_sha = compute_sha256_hex(v1_bytes);
                        match load_model_bytes_cad_doc(
                            "sample_revision_part_v1.step",
                            v1_bytes,
                            None,
                        ) {
                            Ok(v1_doc) => {
                                let v1_topo = TopologyGraph::build(&v1_doc);
                                Some(build_revision_v1_demo_project(
                                    &v1_doc,
                                    &v1_topo,
                                    "sample_revision_part_v1.step".to_string(),
                                    &v1_sha,
                                    &tools,
                                ))
                            }
                            Err(e) => {
                                let _ = tx.send(LoadJobResult::Failed {
                                    generation,
                                    error: e.to_string(),
                                });
                                egui_ctx.request_repaint();
                                return;
                            }
                        }
                    } else {
                        None
                    };
                    let (name, step_str, purpose) = if load_v2_rebind {
                        (
                            "sample_revision_part_v2.step",
                            v2_str,
                            LoadPurpose::ReloadRebind { update_path: false },
                        )
                    } else {
                        (
                            "sample_revision_part_v1.step",
                            v1_str,
                            LoadPurpose::SeedRevisionDemoV1,
                        )
                    };
                    let mut job = parse_step_bytes_job(
                        generation,
                        name.to_string(),
                        step_str.into_bytes(),
                        None,
                        purpose,
                        t0,
                    );
                    if let LoadJobResult::Loaded { payload, .. } = &mut job {
                        payload.pre_seeded_project = pre_seeded;
                    }
                    job
                }
                Err(e) => LoadJobResult::Failed {
                    generation,
                    error: e.to_string(),
                },
            };
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
            if let (Some(m), Some(first)) = (&self.model, self.project.models.first())
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

    fn apply_loaded_cad_payload(&mut self, payload: LoadedPayload, elapsed_ms: u128) {
        let LoadedPayload {
            doc,
            picker,
            topo,
            raw_step_bytes,
            source_path,
            unit_override,
            initial_transform,
            sha256_hex,
            purpose,
            pre_seeded_project,
        } = payload;
        let size = doc.aabb_mm.size();
        if let Some(mesh) = doc.meshes.first() {
            self.logs.push(format!(
                "Loaded {} '{}' [{}] in {} ms — {} triangles, {} vertices, {} (X={:.2} Y={:.2} Z={:.2} mm).",
                mesh.format.label(),
                doc.source_name,
                mesh.applied_unit.label(),
                elapsed_ms,
                mesh.diagnostics.valid_triangle_count,
                mesh.diagnostics.welded_vertex_count,
                if mesh.diagnostics.is_watertight {
                    "Watertight"
                } else {
                    "Open/Non-Manifold"
                },
                size.x,
                size.y,
                size.z,
            ));
        } else {
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
        }

        self.dispatcher.reset();
        self.external_step_modified = false;
        let was_model_empty = self.model.is_none();
        let mut fit_camera = true;

        match purpose {
            LoadPurpose::SeedBlankSample => {
                self.prepare_for_project_replacement(None, None);
                self.project_file_path = None;
                let mut proj = ProjectDocument::new("Untitled Job");
                proj.display_units = self.prefs.default_units;
                if self.prefs.default_machine_id != chip_tools::DEFAULT_MACHINE_ID
                    && let Some(m) = self
                        .machines
                        .iter()
                        .find(|m| m.id.eq_ignore_ascii_case(&self.prefs.default_machine_id))
                        .cloned()
                {
                    proj.apply_machine_defaults(&m, true);
                }
                proj.models = vec![ModelEntry {
                    id: "model_1".to_string(),
                    path: doc.source_name.clone(),
                    sha256: sha256_hex.clone(),
                    unit_override,
                    transform: chip_cad::ModelTransform::IDENTITY,
                }];
                self.project = proj;
                self.sync_active_machine_tools();
            }
            LoadPurpose::SeedMultiFeatureBracket => {
                self.prepare_for_project_replacement(None, None);
                self.project_file_path = None;
                self.project.machine_id = None;
                self.sync_active_machine_tools();
                self.project = build_multi_feature_bracket_project(
                    &doc,
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.sync_active_machine_tools();
                if !self.project.setups[0].operations.is_empty() {
                    self.inspector_focus = InspectorFocus::Operation {
                        setup_idx: 0,
                        op_idx: 0,
                    };
                    self.op_tab = OperationTab::Geometry;
                }
            }
            LoadPurpose::SeedCamTortureBlock => {
                self.prepare_for_project_replacement(None, None);
                self.project_file_path = None;
                self.project.machine_id = None;
                self.sync_active_machine_tools();
                self.project = build_cam_torture_project(
                    &doc,
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.sync_active_machine_tools();
                if self.project.setups[0].operations.len() > 1 {
                    self.inspector_focus = InspectorFocus::Operation {
                        setup_idx: 0,
                        op_idx: 1,
                    };
                    self.op_tab = OperationTab::Passes;
                }
            }
            LoadPurpose::SeedAdaptiveRestDemo => {
                self.prepare_for_project_replacement(None, None);
                self.project_file_path = None;
                self.project.machine_id = None;
                self.sync_active_machine_tools();
                self.project = build_adaptive_rest_demo_project(
                    &doc,
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.sync_active_machine_tools();
                if self.project.setups[0].operations.len() > 3 {
                    self.inspector_focus = InspectorFocus::Operation {
                        setup_idx: 0,
                        op_idx: 3,
                    };
                    self.op_tab = OperationTab::Passes;
                }
            }
            LoadPurpose::SeedRevisionDemoV1 => {
                self.prepare_for_project_replacement(None, None);
                self.project_file_path = None;
                self.project.machine_id = None;
                self.sync_active_machine_tools();
                self.project = build_revision_v1_demo_project(
                    &doc,
                    &topo,
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.sync_active_machine_tools();
                self.inspector_focus = InspectorFocus::Operation {
                    setup_idx: 0,
                    op_idx: 1,
                };
                self.op_tab = OperationTab::Passes;
            }
            LoadPurpose::SeedSculptedMeshDome => {
                self.prepare_for_project_replacement(None, None);
                self.project_file_path = None;
                self.project.machine_id = None;
                self.sync_active_machine_tools();
                self.project = build_sculpted_mesh_dome_project(
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.sync_active_machine_tools();
                if !self.project.setups[0].operations.is_empty() {
                    self.inspector_focus = InspectorFocus::Operation {
                        setup_idx: 0,
                        op_idx: self.project.setups[0].operations.len().saturating_sub(1),
                    };
                    self.op_tab = OperationTab::Passes;
                } else {
                    self.inspector_focus = InspectorFocus::Setup { setup_idx: 0 };
                    self.setup_tab = SetupTab::Wcs;
                }
                self.show_27pt_dots = false;
            }
            LoadPurpose::SeedReliefWaveMesh => {
                self.prepare_for_project_replacement(None, None);
                self.project_file_path = None;
                self.project.machine_id = None;
                self.sync_active_machine_tools();
                self.project = build_relief_wave_mesh_project(
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.sync_active_machine_tools();
                if self.project.setups[0].operations.len() > 1 {
                    self.inspector_focus = InspectorFocus::Operation {
                        setup_idx: 0,
                        op_idx: 1,
                    };
                    self.op_tab = OperationTab::Passes;
                } else {
                    self.inspector_focus = InspectorFocus::Setup { setup_idx: 0 };
                    self.setup_tab = SetupTab::Wcs;
                }
                self.show_27pt_dots = false;
            }
            LoadPurpose::SeedRotaryIndexedShaft => {
                self.prepare_for_project_replacement(None, None);
                self.project_file_path = None;
                self.project.machine_id = None;
                self.sync_active_machine_tools();
                self.project = build_rotary_shaft_preview_project(
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.sync_active_machine_tools();
                self.inspector_focus = InspectorFocus::Operation {
                    setup_idx: 0,
                    op_idx: 0,
                };
                self.op_tab = OperationTab::Passes;
                self.show_27pt_dots = false;
            }
            LoadPurpose::SeedRotaryHelicalFlute => {
                self.prepare_for_project_replacement(None, None);
                self.project_file_path = None;
                self.project.machine_id = None;
                self.sync_active_machine_tools();
                self.project = build_rotary_helical_flute_project(
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.sync_active_machine_tools();
                self.inspector_focus = InspectorFocus::Operation {
                    setup_idx: 0,
                    op_idx: 1,
                };
                self.op_tab = OperationTab::Passes;
                self.show_27pt_dots = false;
            }
            LoadPurpose::FreshImport { update_model_entry } => {
                self.sim_panel.reset_for_new_project();
                self.gcode_panel.invalidate_preview();
                self.auto_sim_after_gen = false;
                self.last_reload_banner = None;
                self.highlight.preview_tab_marker_wcs = None;
                let init_xf = initial_transform.unwrap_or(chip_cad::ModelTransform::IDENTITY);
                if update_model_entry {
                    let stored_path = if let Some(model_path) = &source_path {
                        portable_model_path(self.project_file_path.as_deref(), model_path)
                    } else {
                        doc.source_name.clone()
                    };
                    self.project.models = vec![ModelEntry {
                        id: "model_1".to_string(),
                        path: stored_path,
                        sha256: sha256_hex.clone(),
                        unit_override,
                        transform: init_xf,
                    }];
                    for setup in &mut self.project.setups {
                        for op in &mut setup.operations {
                            op.mark_dirty();
                        }
                    }
                } else if self.project.models.is_empty() {
                    self.project.models = vec![ModelEntry {
                        id: "model_1".to_string(),
                        path: doc.source_name.clone(),
                        sha256: sha256_hex.clone(),
                        unit_override,
                        transform: init_xf,
                    }];
                }
            }
            LoadPurpose::ReloadRebind { update_path } => {
                if let Some(seeded) = pre_seeded_project {
                    self.prepare_for_project_replacement(None, None);
                    self.project_file_path = None;
                    self.project = seeded;
                    self.sync_active_machine_tools();
                }
                fit_camera = was_model_empty;
                let preserved_xf = self
                    .project
                    .models
                    .first()
                    .map_or(chip_cad::ModelTransform::IDENTITY, |m| m.transform);
                if preserved_xf.is_identity() {
                    self.apply_associative_reload_rebind(
                        &doc,
                        &topo,
                        &sha256_hex,
                        source_path.as_deref(),
                        update_path,
                    );
                } else {
                    let transformed_for_rebind = doc.transformed(&preserved_xf);
                    let transformed_topo = TopologyGraph::build(&transformed_for_rebind);
                    self.apply_associative_reload_rebind(
                        &transformed_for_rebind,
                        &transformed_topo,
                        &sha256_hex,
                        source_path.as_deref(),
                        update_path,
                    );
                }
            }
        }

        let raw_doc = Arc::new(doc);
        let applied_transform = self
            .project
            .models
            .first()
            .map_or(chip_cad::ModelTransform::IDENTITY, |m| m.transform);
        let (final_doc, final_picker, final_topo) = if applied_transform.is_identity() {
            (Arc::clone(&raw_doc), picker, topo)
        } else {
            let td = Arc::new(raw_doc.transformed(&applied_transform));
            let tp = CadPicker::build(&td);
            let tt = TopologyGraph::build(&td);
            (td, tp, tt)
        };

        if fit_camera {
            self.camera.fit_aabb(&final_doc.aabb_mm);
        }
        self.highlight.hovered = None;
        self.highlight.selected = None;

        let rev = self.next_revision;
        self.next_revision += 1;
        self.model = Some(LoadedModelState {
            raw_doc,
            applied_transform,
            doc: final_doc,
            picker: final_picker,
            topo: final_topo,
            raw_step_bytes,
            source_path,
            revision: rev,
        });
        self.cached_frame_geo = None;
        self.refresh_dependent_expressions();

        if matches!(
            purpose,
            LoadPurpose::SeedCamTortureBlock
                | LoadPurpose::SeedAdaptiveRestDemo
                | LoadPurpose::SeedRevisionDemoV1
                | LoadPurpose::SeedSculptedMeshDome
                | LoadPurpose::SeedReliefWaveMesh
                | LoadPurpose::SeedRotaryIndexedShaft
                | LoadPurpose::SeedRotaryHelicalFlute
        ) {
            self.auto_sim_after_gen = true;
            self.trigger_generate_dirty(None, None);
        }
    }

    fn apply_associative_reload_rebind(
        &mut self,
        doc: &CadDocument,
        topo: &TopologyGraph,
        sha256_hex: &str,
        source_path: Option<&Path>,
        update_path: bool,
    ) {
        self.history
            .record_before_edit("Reload STEP Model", None, &self.project);
        let new_rel = if update_path {
            source_path.map(|sp| portable_model_path(self.project_file_path.as_deref(), sp))
        } else {
            None
        };
        let report =
            rebind_project_to_reloaded_cad(&mut self.project, 0, doc, topo, sha256_hex, new_rel);
        let summary = report.summary_line();
        let has_warn = report.needs_repick_count > 0 || !report.setup_wcs_warnings.is_empty();
        self.logs.push(summary.clone());
        for w in &report.setup_wcs_warnings {
            self.logs.push(format!("  [WCS WARN] {w}"));
        }
        let mut first_repick: Option<(usize, usize)> = None;
        for out in &report.operation_outcomes {
            match out {
                OperationReloadOutcome::ReboundDirty {
                    setup_name,
                    op_name,
                    updated_indices,
                    ..
                } => {
                    self.logs.push(format!(
                        "  [REBOUND ] {setup_name} -> {op_name} ({updated_indices} topology update(s))"
                    ));
                }
                OperationReloadOutcome::NeedsRepick {
                    setup_name,
                    op_id,
                    op_name,
                    reason,
                } => {
                    self.logs
                        .push(format!("  [REPICK! ] {setup_name} -> {op_name}: {reason}"));
                    if first_repick.is_none() {
                        for (si, s) in self.project.setups.iter().enumerate() {
                            if let Some(oi) = s.operations.iter().position(|op| &op.id == op_id) {
                                first_repick = Some((si, oi));
                                break;
                            }
                        }
                    }
                }
                OperationReloadOutcome::SuppressedSkipped { .. } => {}
            }
        }
        if let Some((si, oi)) = first_repick {
            self.active_setup_idx = si;
            self.inspector_focus = InspectorFocus::Operation {
                setup_idx: si,
                op_idx: oi,
            };
            self.op_tab = OperationTab::Geometry;
        }
        self.sim_panel.reset_for_new_project();
        self.gcode_panel.invalidate_preview();
        self.auto_sim_after_gen = false;
        self.last_reload_banner = Some((summary, has_warn));
    }
}

fn parse_step_bytes_job(
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

fn parse_model_bytes_job(
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
    use std::time::{Duration, Instant};

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
            cancel: None,
        })
        .unwrap();
        app.sim_panel.active_result = Some(Arc::new(sim));
        app.sync_viewport_scene_state();
        assert!(app.highlight.sim_stock_mesh.is_some());
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
        assert!(app.highlight.sim_stock_mesh.is_none());
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
        assert!(app.highlight.sim_stock_mesh.is_none());
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
        assert!(app.highlight.sim_stock_mesh.is_none());
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
        assert!(app.highlight.sim_stock_mesh.is_none());

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
        assert!(app.highlight.sim_stock_mesh.is_none());
        assert!(app.gcode_panel.cached_files.is_empty());
    }
}
