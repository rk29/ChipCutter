use std::time::Instant;

use chip_cad::{TopologyGraph, load_model_bytes_cad_doc};
use chip_demos::{SampleBuildContext, SampleFocusHint, SampleId, build_revision_v1_demo_project};
use chip_fixtures::sample_revised_part_pair_step;
use chip_project::compute_sha256_hex;

use crate::{
    app::ChipCutterApp,
    panels::{InspectorFocus, OperationTab},
};

use super::{LoadJobResult, LoadPurpose, parse_model_bytes_job, parse_step_bytes_job};

impl ChipCutterApp {
    /// Spawns a background load for any 3D STEP/Mesh sample registered in [`chip_demos::SampleCatalog`].
    pub(crate) fn spawn_catalog_sample_load(
        &mut self,
        sample_id: SampleId,
        seed_demo_ops: bool,
        egui_ctx: egui::Context,
    ) {
        let spec = sample_id.spec();
        let generation = self.begin_model_load_generation();
        self.logs.push(spec.load_status_message.to_string());
        let purpose = if seed_demo_ops {
            LoadPurpose::SeedSample(sample_id)
        } else {
            LoadPurpose::SeedBlankSample
        };
        let tx = self.workers.job_tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let res = match sample_id.generate_asset_bytes() {
                Ok(Some(bytes)) => {
                    if spec.asset_kind.is_3d_mesh() {
                        parse_model_bytes_job(
                            generation,
                            spec.default_filename.to_string(),
                            bytes,
                            None,
                            None,
                            purpose,
                            t0,
                        )
                    } else {
                        parse_step_bytes_job(
                            generation,
                            spec.default_filename.to_string(),
                            bytes,
                            None,
                            purpose,
                            t0,
                        )
                    }
                }
                Ok(None) => LoadJobResult::Failed {
                    generation,
                    error: format!(
                        "sample '{}' has no 3D CAD asset to load",
                        spec.canonical_slug
                    ),
                },
                Err(e) => LoadJobResult::Failed {
                    generation,
                    error: e.to_string(),
                },
            };
            let _ = tx.send(res);
            egui_ctx.request_repaint();
        });
    }

    /// Loads any pure-2D / procedural sample registered in [`chip_demos::SampleCatalog`].
    pub(crate) fn load_catalog_pure2d_demo(
        &mut self,
        sample_id: SampleId,
        egui_ctx: &egui::Context,
    ) {
        let spec = sample_id.spec();
        self.prepare_for_project_replacement(None, None);
        self.session.project_file_path = None;
        self.project.machine_id = None;
        self.sync_active_machine_tools();
        if let Ok(proj) = sample_id.build_project(&SampleBuildContext {
            doc: None,
            topo: None,
            asset_path: None,
            sha256_hex: "",
            available_tools: &self.global_tools.tools,
            empty_project: false,
        }) {
            self.project = proj;
        }
        self.sync_active_machine_tools();
        match spec.focus_hint {
            SampleFocusHint::Operation {
                op_idx,
                geometry_tab,
            } => {
                self.selection.inspector_focus = InspectorFocus::Operation {
                    setup_idx: 0,
                    op_idx,
                };
                self.selection.op_tab = if geometry_tab {
                    OperationTab::Geometry
                } else {
                    OperationTab::Passes
                };
            }
            SampleFocusHint::SketchEntity {
                sketch_idx,
                entity_idx,
            } => {
                self.selection.inspector_focus = InspectorFocus::Sketch { sketch_idx };
                self.selection.selected_sketch_entity_idx = Some(entity_idx);
            }
            _ => {}
        }
        self.refresh_dependent_expressions();
        self.fit_scene_to_viewport();
        self.logs.push(spec.load_status_message.to_string());
        self.overlay_prefs.auto_sim_after_gen = spec.auto_generate_and_sim;
        if spec.auto_generate_and_sim {
            self.trigger_generate_dirty(None, None);
        }
        egui_ctx.request_repaint();
    }

    pub(crate) fn spawn_sculpted_mesh_dome_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::SculptedMeshDome, true, egui_ctx);
    }

    pub(crate) fn spawn_relief_wave_mesh_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::ReliefWaveMesh, true, egui_ctx);
    }

    pub(crate) fn spawn_rotary_shaft_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::RotaryIndexedShaft, true, egui_ctx);
    }

    pub(crate) fn spawn_rotary_helical_flute_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::RotaryHelicalFlute, true, egui_ctx);
    }

    pub(crate) fn spawn_sample_plate_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::MachinedPlate, false, egui_ctx);
    }

    pub(crate) fn spawn_multi_feature_sample_load(
        &mut self,
        tilted: bool,
        seed_demo_ops: bool,
        egui_ctx: egui::Context,
    ) {
        let id = if tilted {
            SampleId::MultiFeatureBracketTilted
        } else {
            SampleId::MultiFeatureBracket
        };
        self.spawn_catalog_sample_load(id, seed_demo_ops, egui_ctx);
    }

    pub(crate) fn spawn_cam_torture_sample_load(
        &mut self,
        seed_demo_ops: bool,
        egui_ctx: egui::Context,
    ) {
        self.spawn_catalog_sample_load(SampleId::CamTortureBlock, seed_demo_ops, egui_ctx);
    }

    pub(crate) fn spawn_adaptive_rest_demo_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::AdaptiveRestDemo, true, egui_ctx);
    }

    pub(crate) fn spawn_two_sided_vise_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::TwoSidedViseWorkholding, true, egui_ctx);
    }

    pub(crate) fn spawn_undercut_deburr_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::UndercutAndDeburrBlock, true, egui_ctx);
    }

    pub(crate) fn spawn_feed_opt_pocket_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::FeedOptPocket, true, egui_ctx);
    }

    pub(crate) fn spawn_flip_jig_softjaws_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::FlipJigSoftJaws, true, egui_ctx);
    }

    pub(crate) fn spawn_curved_deburr_3d_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::CurvedDeburr3D, true, egui_ctx);
    }

    pub(crate) fn spawn_dovetail_rail_keyway_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::DovetailRailKeyway, true, egui_ctx);
    }

    pub(crate) fn spawn_mesh_3mf_bracket_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::Mesh3mfBracket, true, egui_ctx);
    }

    pub(crate) fn spawn_keepout_clamps_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::KeepOutClamps, true, egui_ctx);
    }

    pub(crate) fn spawn_indexed_model_bounds_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::IndexedModelBounds, true, egui_ctx);
    }

    pub fn load_svg_badge_sign_demo(&mut self, egui_ctx: &egui::Context) {
        self.load_catalog_pure2d_demo(SampleId::SvgBadgeSign, egui_ctx);
    }

    pub(crate) fn spawn_thread_mill_manifold_load(&mut self, egui_ctx: egui::Context) {
        self.spawn_catalog_sample_load(SampleId::ThreadMillManifold, true, egui_ctx);
    }

    pub fn load_rotary_wrapped_engraving_demo(&mut self, egui_ctx: &egui::Context) {
        self.load_catalog_pure2d_demo(SampleId::RotaryWrappedEngraving, egui_ctx);
    }

    pub fn load_pure2d_router_panel_demo(&mut self, egui_ctx: &egui::Context) {
        self.load_catalog_pure2d_demo(SampleId::Pure2dRouterPanel, egui_ctx);
    }

    pub fn load_engraved_nameplate_demo(&mut self, egui_ctx: &egui::Context) {
        self.load_catalog_pure2d_demo(SampleId::EngravedNameplate, egui_ctx);
    }

    pub fn load_vcarve_sign_demo(&mut self, egui_ctx: &egui::Context) {
        self.load_catalog_pure2d_demo(SampleId::VCarveSignAndInlay, egui_ctx);
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
        let tx = self.workers.job_tx.clone();
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
                            LoadPurpose::SeedSample(SampleId::RevisionDemoV1),
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
