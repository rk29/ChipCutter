use std::time::Instant;

use chip_cad::{
    TopologyGraph, sample_cam_torture_block_step, sample_helical_flute_shaft_stl_bytes,
    sample_machined_plate_step, sample_multi_feature_bracket_step, sample_relief_wave_obj_text,
    sample_revised_part_pair_step, sample_rotary_shaft_stl_bytes, sample_sculpted_dome_stl_bytes,
    sample_threaded_manifold_step, sample_undercut_deburr_block_step_bytes,
};
use chip_project::{
    build_pure2d_router_panel_demo_project, build_revision_v1_demo_project,
    build_rotary_wrapped_engraving_project, compute_sha256_hex, load_model_bytes_cad_doc,
    sample_two_sided_vise_part_stl_bytes,
};

use crate::{
    app::ChipCutterApp,
    panels::{InspectorFocus, OperationTab},
};

use super::{LoadJobResult, LoadPurpose, parse_model_bytes_job, parse_step_bytes_job};

impl ChipCutterApp {
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

    pub(crate) fn spawn_two_sided_vise_load(&mut self, egui_ctx: egui::Context) {
        let generation = self.begin_model_load_generation();
        self.logs.push(
            "Generating Two-Sided Vise & Soft-Jaw Workholding sample (Setup 1 G54 Vise + Toe Clamps -> Setup 2 G55 Flip Carryover)…".to_string(),
        );
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let stl_bytes = sample_two_sided_vise_part_stl_bytes();
            let res = parse_model_bytes_job(
                generation,
                "two_sided_vise_bracket.stl".to_string(),
                stl_bytes,
                None,
                None,
                LoadPurpose::SeedTwoSidedViseWorkholding,
                t0,
            );
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    pub(crate) fn spawn_undercut_deburr_load(&mut self, egui_ctx: egui::Context) {
        let generation = self.begin_model_load_generation();
        self.logs.push(
            "Generating Undercut & 3D Deburr Block STEP fixture (M8 T-Slot, 60° Dovetail, Lollipop Overhang, & 3D Deburr)…"
                .to_string(),
        );
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let res = match sample_undercut_deburr_block_step_bytes() {
                Ok(step_bytes) => parse_step_bytes_job(
                    generation,
                    "sample_undercut_deburr_block.step".to_string(),
                    step_bytes,
                    None,
                    LoadPurpose::SeedUndercutAndDeburrBlock,
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

    pub(crate) fn spawn_thread_mill_manifold_load(&mut self, egui_ctx: egui::Context) {
        let generation = self.begin_model_load_generation();
        self.logs.push(
            "Generating Threaded Manifold Block STEP fixture (M8×1.25, 1/4-20 UNC, External M12×1.75)…"
                .to_string(),
        );
        let tx = self.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let res = match sample_threaded_manifold_step() {
                Ok(step_str) => parse_step_bytes_job(
                    generation,
                    "18-threaded-manifold.step".to_string(),
                    step_str.into_bytes(),
                    None,
                    LoadPurpose::SeedThreadMillManifold,
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
}
