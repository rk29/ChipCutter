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
    build_sculpted_mesh_dome_project, compute_sha256_hex, export_chipz_bundle, is_chipz_path,
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
    pub sha256_hex: String,
    pub purpose: LoadPurpose,
    pub pre_seeded_project: Option<ProjectDocument>,
}

pub(crate) enum LoadJobResult {
    Loaded {
        payload: Box<LoadedPayload>,
        elapsed_ms: u128,
    },
    Failed(String),
}

impl ChipCutterApp {
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
                &["toml", "chipz", "CHIPZ"],
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
                self.clear_bundle_extract_dir();
                self.history.clear();
                self.clean_undo_len = Some(0);
                self.external_step_modified = false;
                self.last_reload_banner = None;
                self.project = doc;
                self.project_file_path = Some(path.to_path_buf());
                self.record_recent_file(path);
                self.active_setup_idx = 0;
                self.inspector_focus = InspectorFocus::Setup { setup_idx: 0 };
                self.setup_tab = SetupTab::Wcs;
                self.pick_mode = ViewportPickMode::InspectEntity;
                self.gcode_panel.refresh_profiles(path.parent());
                self.machines = chip_tools::load_available_machines(path.parent());
                self.sync_active_machine_tools();
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
                    self.model = None;
                    self.cached_frame_geo = None;
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
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos());
        let extract_dir = std::env::temp_dir().join(format!(
            "chipcutter_gui_chipz_{}_{nanos}",
            std::process::id()
        ));
        match unpack_chipz_bundle(archive_path, &extract_dir) {
            Ok(unpacked) => {
                self.clear_bundle_extract_dir();
                self.bundle_extract_dir = Some(extract_dir.clone());
                self.history.clear();
                self.clean_undo_len = Some(0);
                self.external_step_modified = false;
                self.last_reload_banner = None;
                self.project = unpacked.project;
                self.project_file_path = None;
                self.record_recent_file(archive_path);
                self.active_setup_idx = 0;
                self.inspector_focus = InspectorFocus::Setup { setup_idx: 0 };
                self.pick_mode = ViewportPickMode::InspectEntity;
                self.gcode_panel.refresh_profiles(Some(&extract_dir));
                self.machines = chip_tools::load_available_machines(Some(&extract_dir));
                self.sync_active_machine_tools();
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
                    self.model = None;
                    self.cached_frame_geo = None;
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
        self.loading_in_flight = true;
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
                    parse_model_bytes_job(name, bytes, Some(path), unit_override, purpose, t0)
                }
                Err(e) => LoadJobResult::Failed(format!("failed to read {}: {e}", path.display())),
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
        egui_ctx: egui::Context,
    ) {
        if let Some(p) = &source_path {
            self.record_recent_file(p);
        }
        self.loading_in_flight = true;
        self.logs.push(format!(
            "Importing 3D mesh '{source_name}' on background worker…"
        ));
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let res = parse_model_bytes_job(
                source_name,
                raw_bytes,
                source_path,
                unit_override,
                LoadPurpose::FreshImport {
                    update_model_entry: true,
                },
                t0,
            );
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub(crate) fn spawn_sculpted_mesh_dome_load(&mut self, egui_ctx: egui::Context) {
        self.loading_in_flight = true;
        self.logs
            .push("Generating watertight Sculpted Dome (STL Mesh) sample…".to_string());
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let stl_bytes = sample_sculpted_dome_stl_bytes();
            let res = parse_model_bytes_job(
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
        self.loading_in_flight = true;
        self.logs
            .push("Generating Relief Wave (OBJ Mesh) 3D finishing sample…".to_string());
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let obj_bytes = sample_relief_wave_obj_text().into_bytes();
            let res = parse_model_bytes_job(
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
        self.loading_in_flight = true;
        self.logs
            .push("Generating Rotary Indexed Shaft (4th-Axis A) sample…".to_string());
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let stl_bytes = sample_rotary_shaft_stl_bytes();
            let res = parse_model_bytes_job(
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
        self.loading_in_flight = true;
        self.logs
            .push("Generating Simultaneous 4-Axis Helical Flute (X+Y+Z+A) sample…".to_string());
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let stl_bytes = sample_helical_flute_shaft_stl_bytes();
            let res = parse_model_bytes_job(
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
        self.loading_in_flight = true;
        self.logs
            .push("Generating & tessellating sample CNC mounting plate STEP model…".to_string());
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let res = match sample_machined_plate_step() {
                Ok(step_str) => parse_step_bytes_job(
                    "sample_machined_plate.step".to_string(),
                    step_str.into_bytes(),
                    None,
                    LoadPurpose::FreshImport {
                        update_model_entry: false,
                    },
                    t0,
                ),
                Err(e) => LoadJobResult::Failed(e.to_string()),
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
        self.loading_in_flight = true;
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
                        LoadPurpose::FreshImport {
                            update_model_entry: false,
                        }
                    };
                    parse_step_bytes_job(name.to_string(), step_str.into_bytes(), None, purpose, t0)
                }
                Err(e) => LoadJobResult::Failed(e.to_string()),
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
        self.loading_in_flight = true;
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
                        LoadPurpose::FreshImport {
                            update_model_entry: false,
                        }
                    };
                    parse_step_bytes_job(
                        "sample_cam_torture_block.step".to_string(),
                        step_str.into_bytes(),
                        None,
                        purpose,
                        t0,
                    )
                }
                Err(e) => LoadJobResult::Failed(e.to_string()),
            };
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub(crate) fn spawn_adaptive_rest_demo_load(&mut self, egui_ctx: egui::Context) {
        self.loading_in_flight = true;
        self.logs
            .push("Generating Milestone 6 2D Adaptive + Rest Machining STEP job…".to_string());
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let res = match sample_cam_torture_block_step() {
                Ok(step_str) => parse_step_bytes_job(
                    "sample_cam_torture_block.step".to_string(),
                    step_str.into_bytes(),
                    None,
                    LoadPurpose::SeedAdaptiveRestDemo,
                    t0,
                ),
                Err(e) => LoadJobResult::Failed(e.to_string()),
            };
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub fn load_rotary_wrapped_engraving_demo(&mut self, egui_ctx: &egui::Context) {
        self.loading_in_flight = false;
        while self.job_rx.try_recv().is_ok() {}
        self.dispatcher.cancel_all();
        self.sim_panel.cancel_in_flight();
        self.sim_panel.active_result = None;
        self.gcode_panel.invalidate_preview();
        self.clear_bundle_extract_dir();
        self.project_file_path = None;
        self.history.clear();
        self.clean_undo_len = Some(0);
        self.last_reload_banner = None;
        self.external_step_modified = false;
        self.model = None;
        self.project = build_rotary_wrapped_engraving_project(&self.global_tools.tools);
        self.active_setup_idx = 0;
        self.inspector_focus = InspectorFocus::Operation {
            setup_idx: 0,
            op_idx: 0,
        };
        self.op_tab = OperationTab::Passes;
        self.highlight.hovered = None;
        self.highlight.selected = None;
        self.highlight.hovered_sketch = None;
        self.highlight.selected_sketch = None;
        self.cached_frame_geo = None;
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
        self.loading_in_flight = false;
        while self.job_rx.try_recv().is_ok() {}
        self.dispatcher.cancel_all();
        self.sim_panel.cancel_in_flight();
        self.sim_panel.active_result = None;
        self.gcode_panel.invalidate_preview();
        self.clear_bundle_extract_dir();
        self.project_file_path = None;
        self.history.clear();
        self.clean_undo_len = Some(0);
        self.last_reload_banner = None;
        self.external_step_modified = false;
        self.model = None;
        self.project = build_pure2d_router_panel_demo_project(None, &self.global_tools.tools);
        self.active_setup_idx = 0;
        self.inspector_focus = InspectorFocus::Operation {
            setup_idx: 0,
            op_idx: 1,
        };
        self.op_tab = OperationTab::Geometry;
        self.highlight.hovered = None;
        self.highlight.selected = None;
        self.highlight.hovered_sketch = None;
        self.highlight.selected_sketch = None;
        self.cached_frame_geo = None;
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
        self.loading_in_flight = false;
        while self.job_rx.try_recv().is_ok() {}
        self.dispatcher.cancel_all();
        self.sim_panel.cancel_in_flight();
        self.sim_panel.active_result = None;
        self.gcode_panel.invalidate_preview();
        self.clear_bundle_extract_dir();
        self.project_file_path = None;
        self.history.clear();
        self.clean_undo_len = Some(0);
        self.last_reload_banner = None;
        self.external_step_modified = false;
        self.model = None;
        self.project =
            chip_project::build_engraved_nameplate_demo_project(&self.global_tools.tools);
        self.active_setup_idx = 0;
        self.inspector_focus = InspectorFocus::Sketch { sketch_idx: 0 };
        self.selected_sketch_entity_idx = Some(3);
        self.pick_mode = ViewportPickMode::InspectEntity;
        self.sketch_author_start_xy = None;
        self.highlight.hovered = None;
        self.highlight.selected = None;
        self.highlight.hovered_sketch = None;
        self.highlight.selected_sketch = None;
        self.cached_frame_geo = None;
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
        self.loading_in_flight = false;
        while self.job_rx.try_recv().is_ok() {}
        self.dispatcher.cancel_all();
        self.sim_panel.cancel_in_flight();
        self.sim_panel.active_result = None;
        self.gcode_panel.invalidate_preview();
        self.clear_bundle_extract_dir();
        self.project_file_path = None;
        self.history.clear();
        self.clean_undo_len = Some(0);
        self.last_reload_banner = None;
        self.external_step_modified = false;
        self.model = None;
        self.project =
            chip_project::build_vcarve_sign_and_inlay_demo_project(&self.global_tools.tools);
        self.active_setup_idx = 0;
        self.inspector_focus = InspectorFocus::Operation {
            setup_idx: 0,
            op_idx: 0,
        };
        self.op_tab = OperationTab::Passes;
        self.pick_mode = ViewportPickMode::InspectEntity;
        self.sketch_author_start_xy = None;
        self.highlight.hovered = None;
        self.highlight.selected = None;
        self.highlight.hovered_sketch = None;
        self.highlight.selected_sketch = None;
        self.cached_frame_geo = None;
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
        self.loading_in_flight = true;
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
                                let _ = tx.send(LoadJobResult::Failed(e.to_string()));
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
                Err(e) => LoadJobResult::Failed(e.to_string()),
            };
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub(crate) fn poll_background_jobs(&mut self, egui_ctx: &egui::Context) {
        while let Ok(msg) = self.job_rx.try_recv() {
            if !self.loading_in_flight {
                continue;
            }
            self.loading_in_flight = false;
            match msg {
                LoadJobResult::Loaded {
                    payload,
                    elapsed_ms,
                } => {
                    self.apply_loaded_cad_payload(*payload, elapsed_ms);
                }
                LoadJobResult::Failed(err) => {
                    self.logs.push(format!("ERROR loading STEP model: {err}"));
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

        self.dispatcher.cancel_all();
        self.external_step_modified = false;
        let was_model_empty = self.model.is_none();
        let mut fit_camera = true;

        match purpose {
            LoadPurpose::SeedMultiFeatureBracket => {
                self.clear_bundle_extract_dir();
                self.project_file_path = None;
                self.history.clear();
                self.clean_undo_len = Some(0);
                self.last_reload_banner = None;
                self.project = build_multi_feature_bracket_project(
                    &doc,
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.active_setup_idx = 0;
                if !self.project.setups[0].operations.is_empty() {
                    self.inspector_focus = InspectorFocus::Operation {
                        setup_idx: 0,
                        op_idx: 0,
                    };
                    self.op_tab = OperationTab::Geometry;
                }
            }
            LoadPurpose::SeedCamTortureBlock => {
                self.clear_bundle_extract_dir();
                self.project_file_path = None;
                self.history.clear();
                self.clean_undo_len = Some(0);
                self.last_reload_banner = None;
                self.project = build_cam_torture_project(
                    &doc,
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.active_setup_idx = 0;
                if self.project.setups[0].operations.len() > 1 {
                    self.inspector_focus = InspectorFocus::Operation {
                        setup_idx: 0,
                        op_idx: 1,
                    };
                    self.op_tab = OperationTab::Passes;
                }
            }
            LoadPurpose::SeedAdaptiveRestDemo => {
                self.clear_bundle_extract_dir();
                self.project_file_path = None;
                self.history.clear();
                self.clean_undo_len = Some(0);
                self.last_reload_banner = None;
                self.project = build_adaptive_rest_demo_project(
                    &doc,
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.active_setup_idx = 0;
                if self.project.setups[0].operations.len() > 3 {
                    self.inspector_focus = InspectorFocus::Operation {
                        setup_idx: 0,
                        op_idx: 3,
                    };
                    self.op_tab = OperationTab::Passes;
                }
            }
            LoadPurpose::SeedRevisionDemoV1 => {
                self.clear_bundle_extract_dir();
                self.project_file_path = None;
                self.history.clear();
                self.clean_undo_len = Some(0);
                self.last_reload_banner = None;
                self.project = build_revision_v1_demo_project(
                    &doc,
                    &topo,
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.active_setup_idx = 0;
                self.inspector_focus = InspectorFocus::Operation {
                    setup_idx: 0,
                    op_idx: 1,
                };
                self.op_tab = OperationTab::Passes;
            }
            LoadPurpose::SeedSculptedMeshDome => {
                self.clear_bundle_extract_dir();
                self.project_file_path = None;
                self.history.clear();
                self.clean_undo_len = Some(0);
                self.last_reload_banner = None;
                self.project = build_sculpted_mesh_dome_project(
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.active_setup_idx = 0;
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
                self.clear_bundle_extract_dir();
                self.project_file_path = None;
                self.history.clear();
                self.clean_undo_len = Some(0);
                self.last_reload_banner = None;
                self.project = build_relief_wave_mesh_project(
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.active_setup_idx = 0;
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
                self.clear_bundle_extract_dir();
                self.project_file_path = None;
                self.history.clear();
                self.clean_undo_len = Some(0);
                self.last_reload_banner = None;
                self.project = build_rotary_shaft_preview_project(
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.active_setup_idx = 0;
                self.inspector_focus = InspectorFocus::Operation {
                    setup_idx: 0,
                    op_idx: 0,
                };
                self.op_tab = OperationTab::Passes;
                self.show_27pt_dots = false;
            }
            LoadPurpose::SeedRotaryHelicalFlute => {
                self.clear_bundle_extract_dir();
                self.project_file_path = None;
                self.history.clear();
                self.clean_undo_len = Some(0);
                self.last_reload_banner = None;
                self.project = build_rotary_helical_flute_project(
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.active_setup_idx = 0;
                self.inspector_focus = InspectorFocus::Operation {
                    setup_idx: 0,
                    op_idx: 1,
                };
                self.op_tab = OperationTab::Passes;
                self.show_27pt_dots = false;
            }
            LoadPurpose::FreshImport { update_model_entry } => {
                self.last_reload_banner = None;
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
                    }];
                } else if self.project.models.is_empty() {
                    self.project.models = vec![ModelEntry {
                        id: "model_1".to_string(),
                        path: doc.source_name.clone(),
                        sha256: sha256_hex.clone(),
                        unit_override,
                    }];
                }
            }
            LoadPurpose::ReloadRebind { update_path } => {
                if let Some(seeded) = pre_seeded_project {
                    self.clear_bundle_extract_dir();
                    self.project_file_path = None;
                    self.history.clear();
                    self.clean_undo_len = Some(0);
                    self.project = seeded;
                    self.active_setup_idx = 0;
                }
                fit_camera = was_model_empty;
                self.apply_associative_reload_rebind(
                    &doc,
                    &topo,
                    &sha256_hex,
                    source_path.as_deref(),
                    update_path,
                );
            }
        }

        if fit_camera {
            self.camera.fit_aabb(&doc.aabb_mm);
        }
        self.highlight.hovered = None;
        self.highlight.selected = None;

        let rev = self.next_revision;
        self.next_revision += 1;
        self.model = Some(LoadedModelState {
            doc: Arc::new(doc),
            picker,
            topo,
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
        self.sim_panel.cancel_in_flight();
        self.sim_panel.active_result = None;
        self.gcode_panel.invalidate_preview();
        self.last_reload_banner = Some((summary, has_warn));
    }
}

fn parse_step_bytes_job(
    name: String,
    raw_step_bytes: Vec<u8>,
    source_path: Option<PathBuf>,
    purpose: LoadPurpose,
    t0: Instant,
) -> LoadJobResult {
    parse_model_bytes_job(name, raw_step_bytes, source_path, None, purpose, t0)
}

fn parse_model_bytes_job(
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
                payload: Box::new(LoadedPayload {
                    doc,
                    picker,
                    topo,
                    raw_step_bytes,
                    source_path,
                    unit_override,
                    sha256_hex,
                    purpose,
                    pre_seeded_project: None,
                }),
                elapsed_ms: t0.elapsed().as_millis(),
            }
        }
        Err(e) => LoadJobResult::Failed(e.to_string()),
    }
}
