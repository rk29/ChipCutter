use std::{path::Path, sync::Arc};

use chip_cad::{CadDocument, CadPicker, TopologyGraph};
use chip_demos::{SampleBuildContext, SampleFocusHint};
use chip_project::{
    ModelEntry, OperationReloadOutcome, ProjectDocument, portable_model_path,
    rebind_project_to_reloaded_cad,
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
        let mut auto_generate_and_sim = false;

        match purpose {
            LoadPurpose::SeedBlankSample => {
                self.prepare_for_project_replacement(None, None);
                self.project_file_path = None;
                let mut proj = ProjectDocument::new("Untitled Job");
                proj.display_units = self.prefs.default_units;
                if self.prefs.default_machine_id != chip_machines::DEFAULT_MACHINE_ID
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
            LoadPurpose::SeedSample(sample_id) => {
                let spec = sample_id.spec();
                self.prepare_for_project_replacement(None, None);
                self.project_file_path = None;
                self.project.machine_id = None;
                self.sync_active_machine_tools();
                match sample_id.build_project(&SampleBuildContext {
                    doc: Some(&doc),
                    topo: Some(&topo),
                    asset_path: Some(doc.source_name.clone()),
                    sha256_hex: &sha256_hex,
                    available_tools: &self.global_tools.tools,
                    empty_project: false,
                }) {
                    Ok(proj) => self.project = proj,
                    Err(err) => {
                        self.logs.push(format!(
                            "Failed to seed sample '{}': {err}",
                            spec.canonical_slug
                        ));
                    }
                }
                self.sync_active_machine_tools();
                if spec.prefer_tridexel_sim {
                    self.sim_panel.engine = chip_sim::SimEngineKind::TriDexel;
                }
                if spec.show_fixtures {
                    self.highlight.show_fixtures = true;
                }
                if spec.hide_27pt_dots {
                    self.show_27pt_dots = false;
                }
                let op_count = self
                    .project
                    .setups
                    .first()
                    .map_or(0, |s| s.operations.len());
                match spec.focus_hint {
                    SampleFocusHint::Operation {
                        op_idx,
                        geometry_tab,
                    } if op_idx < op_count => {
                        self.inspector_focus = InspectorFocus::Operation {
                            setup_idx: 0,
                            op_idx,
                        };
                        self.op_tab = if geometry_tab {
                            OperationTab::Geometry
                        } else {
                            OperationTab::Passes
                        };
                    }
                    SampleFocusHint::LastOperationPasses if op_count > 0 => {
                        self.inspector_focus = InspectorFocus::Operation {
                            setup_idx: 0,
                            op_idx: op_count.saturating_sub(1),
                        };
                        self.op_tab = OperationTab::Passes;
                    }
                    SampleFocusHint::SetupFixtures => {
                        self.inspector_focus = InspectorFocus::Setup { setup_idx: 0 };
                        self.setup_tab = SetupTab::Fixtures;
                    }
                    SampleFocusHint::SketchEntity {
                        sketch_idx,
                        entity_idx,
                    } => {
                        self.inspector_focus = InspectorFocus::Sketch { sketch_idx };
                        self.selected_sketch_entity_idx = Some(entity_idx);
                    }
                    _ => {
                        if op_count == 0
                            && matches!(
                                spec.focus_hint,
                                SampleFocusHint::LastOperationPasses
                                    | SampleFocusHint::Operation { .. }
                            )
                        {
                            self.inspector_focus = InspectorFocus::Setup { setup_idx: 0 };
                            self.setup_tab = SetupTab::Wcs;
                        }
                    }
                }
                auto_generate_and_sim = spec.auto_generate_and_sim;
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
            LoadPurpose::ReloadMeshUnits => {
                self.sim_panel.reset_for_new_project();
                self.gcode_panel.invalidate_preview();
                self.auto_sim_after_gen = false;
                self.highlight.preview_tab_marker_wcs = None;
                fit_camera = true;
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
            applied_unit_override: unit_override,
            doc: final_doc,
            picker: final_picker,
            topo: final_topo,
            raw_step_bytes,
            source_path,
            revision: rev,
        });
        self.cached_frame_geo = None;
        self.refresh_dependent_expressions();

        if auto_generate_and_sim {
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
