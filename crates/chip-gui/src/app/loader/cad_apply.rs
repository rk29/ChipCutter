use std::{path::Path, sync::Arc};

use chip_cad::{CadDocument, CadPicker, TopologyGraph};
use chip_project::{
    ModelEntry, OperationReloadOutcome, ProjectDocument, build_adaptive_rest_demo_project,
    build_cam_torture_project, build_multi_feature_bracket_project, build_relief_wave_mesh_project,
    build_revision_v1_demo_project, build_rotary_helical_flute_project,
    build_rotary_shaft_preview_project, build_sculpted_mesh_dome_project,
    build_thread_mill_manifold_project, build_two_sided_vise_workholding_project,
    build_undercut_deburr_block_project, portable_model_path, rebind_project_to_reloaded_cad,
};

use crate::{
    app::{ChipCutterApp, LoadedModelState},
    panels::{InspectorFocus, OperationTab, SetupTab},
};

use super::{LoadPurpose, LoadedPayload};

impl ChipCutterApp {
    pub(super) fn apply_loaded_cad_payload(&mut self, payload: LoadedPayload, elapsed_ms: u128) {
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
                    role: Default::default(),
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
            LoadPurpose::SeedThreadMillManifold => {
                self.prepare_for_project_replacement(None, None);
                self.project_file_path = None;
                self.project.machine_id = None;
                self.sync_active_machine_tools();
                self.project = build_thread_mill_manifold_project(
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
                self.show_27pt_dots = false;
            }
            LoadPurpose::SeedTwoSidedViseWorkholding => {
                self.prepare_for_project_replacement(None, None);
                self.project_file_path = None;
                self.project.machine_id = None;
                self.sync_active_machine_tools();
                self.project = build_two_sided_vise_workholding_project(
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.sync_active_machine_tools();
                self.inspector_focus = InspectorFocus::Setup { setup_idx: 0 };
                self.setup_tab = SetupTab::Fixtures;
                self.highlight.show_fixtures = true;
                self.show_27pt_dots = false;
            }
            LoadPurpose::SeedUndercutAndDeburrBlock => {
                self.prepare_for_project_replacement(None, None);
                self.project_file_path = None;
                self.project.machine_id = None;
                self.sync_active_machine_tools();
                self.project = build_undercut_deburr_block_project(
                    &doc,
                    doc.source_name.clone(),
                    &sha256_hex,
                    &self.global_tools.tools,
                );
                self.sync_active_machine_tools();
                self.sim_panel.engine = chip_sim::SimEngineKind::TriDexel;
                if self.project.setups[0].operations.len() > 3 {
                    self.inspector_focus = InspectorFocus::Operation {
                        setup_idx: 0,
                        op_idx: 3,
                    };
                    self.op_tab = OperationTab::Passes;
                }
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
                    let new_part = ModelEntry {
                        id: "model_1".to_string(),
                        path: stored_path,
                        sha256: sha256_hex.clone(),
                        role: chip_project::ModelRole::Part,
                        unit_override,
                        transform: init_xf,
                    };
                    // Preserve any imported workholding fixture models (`ModelRole::Fixture`)
                    self.project.models.retain(|m| m.role.is_fixture());
                    self.project.models.insert(0, new_part);
                    for setup in &mut self.project.setups {
                        for op in &mut setup.operations {
                            op.mark_dirty();
                        }
                    }
                } else if self.project.primary_part_model().is_none() {
                    self.project.models.insert(
                        0,
                        ModelEntry {
                            id: "model_1".to_string(),
                            path: doc.source_name.clone(),
                            sha256: sha256_hex.clone(),
                            role: chip_project::ModelRole::Part,
                            unit_override,
                            transform: init_xf,
                        },
                    );
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
                    .primary_part_model()
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
            .primary_part_model()
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
                | LoadPurpose::SeedThreadMillManifold
                | LoadPurpose::SeedTwoSidedViseWorkholding
                | LoadPurpose::SeedUndercutAndDeburrBlock
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
        let part_idx = self.project.primary_part_model_index().unwrap_or(0);
        let report = rebind_project_to_reloaded_cad(
            &mut self.project,
            part_idx,
            doc,
            topo,
            sha256_hex,
            new_rel,
        );
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
