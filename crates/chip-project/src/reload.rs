use std::collections::{HashMap, HashSet};

use chip_cad::{
    BodyId, CadDocument, EntitySignatureKind, FaceId, GeometricSignature, RematchOutcome,
    SelectionBinding, TopologyGraph, expand_coaxial_hole_faces, rematch_edge, rematch_face,
    rematch_loop,
};
use chip_cam::{
    GeometrySelectionItem, OperationStatus, WcsAxisRef, WcsFeaturePointRef, WcsOrientationMode,
    WcsOriginMode, extract_selection_features_with_sketches,
};
use chip_sketch::{SketchAnchor, SketchFaceBinding};
use serde::{Deserialize, Serialize};

use crate::document::{ModelEntry, ProjectDocument};

/// Converts a CAD-kernel-independent [`SketchFaceBinding`] into a [`SelectionBinding`] (`PlanarFace`).
pub fn sketch_face_to_cad_binding(b: &SketchFaceBinding) -> SelectionBinding {
    SelectionBinding {
        body_id: BodyId(b.body_id),
        last_index: b.last_face_index,
        signature: GeometricSignature {
            kind: EntitySignatureKind::PlanarFace,
            normal_or_axis: b.normal_world,
            centroid_mm: b.centroid_world_mm,
            characteristic_measure: b.area_mm2,
            aabb_mm: b.aabb_mm,
        },
    }
}

/// Converts a planar [`SelectionBinding`] (`chip-cad`) into a serializable [`SketchFaceBinding`].
pub fn cad_binding_to_sketch_face(b: &SelectionBinding) -> SketchFaceBinding {
    SketchFaceBinding {
        body_id: b.body_id.0,
        last_face_index: b.last_index,
        normal_world: b.signature.normal_or_axis,
        centroid_world_mm: b.signature.centroid_mm,
        area_mm2: b.signature.characteristic_measure,
        aabb_mm: b.signature.aabb_mm,
    }
}

/// Per-operation outcome after reloading a revised STEP model (`02-cad-selection.md` §CAD Reload).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum OperationReloadOutcome {
    ReboundDirty {
        setup_name: String,
        op_id: String,
        op_name: String,
        updated_indices: usize,
    },
    NeedsRepick {
        setup_name: String,
        op_id: String,
        op_name: String,
        reason: String,
    },
    SuppressedSkipped {
        setup_name: String,
        op_id: String,
        op_name: String,
    },
}

/// Summary report returned by [`rebind_project_to_reloaded_cad`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CadReloadReport {
    pub model_id: String,
    pub new_sha256: String,
    pub setup_wcs_warnings: Vec<String>,
    pub operation_outcomes: Vec<OperationReloadOutcome>,
    pub rebound_dirty_count: usize,
    pub needs_repick_count: usize,
}

impl CadReloadReport {
    pub fn summary_line(&self) -> String {
        if self.needs_repick_count == 0 && self.setup_wcs_warnings.is_empty() {
            format!(
                "Reloaded CAD & re-bound {} operation(s) cleanly (Dirty — press Ctrl+G to regenerate).",
                self.rebound_dirty_count
            )
        } else {
            format!(
                "Reloaded CAD: {} operation(s) re-bound cleanly, {} need re-pick ({} WCS warning(s)).",
                self.rebound_dirty_count,
                self.needs_repick_count,
                self.setup_wcs_warnings.len()
            )
        }
    }
}

/// Re-binds all Setup WCS references and Operation geometry selections in `project` against a
/// revised [`CadDocument`] (`02-cad-selection.md` §Re-Matching Algorithm).
///
/// 1. Updates `project.models[model_idx].sha256`.
/// 2. Re-binds Setup WCS alignment & feature origin entities first.
/// 3. Re-binds every Operation's `GeometrySelectionItem`s (including `OpenChain` healing via `topo`),
///    marking matched operations `Dirty` and flagging missing/ambiguous selections `NeedsRepick`
///    without losing tool, feed/speed, height, pass, or holding-tab parameters!
pub fn rebind_project_to_reloaded_cad(
    project: &mut ProjectDocument,
    model_idx: usize,
    new_doc: &CadDocument,
    topo: &TopologyGraph,
    new_sha256: &str,
    new_stored_path: Option<String>,
) -> CadReloadReport {
    if project.models.is_empty() {
        project.models.push(ModelEntry {
            id: "model_1".to_string(),
            path: new_stored_path.unwrap_or_else(|| new_doc.source_name.clone()),
            sha256: new_sha256.to_string(),
            unit_override: None,
            transform: chip_cad::ModelTransform::IDENTITY,
        });
    } else if let Some(entry) = project.models.get_mut(model_idx) {
        entry.sha256 = new_sha256.to_string();
        if let Some(p) = new_stored_path {
            entry.path = p;
        }
    }

    let model_id = project
        .models
        .get(model_idx)
        .map_or_else(|| "model_1".to_string(), |m| m.id.clone());

    let mut report = CadReloadReport {
        model_id,
        new_sha256: new_sha256.to_string(),
        ..CadReloadReport::default()
    };

    // Step 0: Re-bind associative `SketchAnchor::PlanarFace` sketches on revised STEP geometry!
    let mut broken_sketch_reasons: HashMap<String, String> = HashMap::new();
    let mut updated_sketch_indices: HashSet<String> = HashSet::new();
    for sk in &mut project.sketches {
        if let SketchAnchor::PlanarFace { face_binding, .. } = &mut sk.anchor {
            let cad_bind = sketch_face_to_cad_binding(face_binding);
            match rematch_face(new_doc, &cad_bind) {
                RematchOutcome::Matched {
                    updated_binding,
                    index_changed,
                    ..
                } => {
                    *face_binding = cad_binding_to_sketch_face(&updated_binding);
                    if index_changed {
                        updated_sketch_indices.insert(sk.id.clone());
                    }
                }
                RematchOutcome::Ambiguous { message, .. } | RematchOutcome::Missing { message } => {
                    let msg = format!(
                        "Sketch '{}' host planar face could not re-bind: {message}",
                        sk.name
                    );
                    report.setup_wcs_warnings.push(msg.clone());
                    broken_sketch_reasons.insert(sk.id.clone(), msg);
                }
            }
        }
    }

    let display_unit = project.display_units;
    let project_tools = project.tools.clone();
    let project_sketches = project.sketches.clone();

    for setup in &mut project.setups {
        // Clamp target_bodies if body count shrank
        setup
            .target_bodies
            .retain(|b| (b.0 as usize) < new_doc.bodies.len());

        // Step 1: Re-bind Setup WCS orientation & origin feature references first
        rebind_setup_wcs(setup, new_doc, &mut report.setup_wcs_warnings);

        let resolved_setup_opt = setup
            .resolve_with_sketches(new_doc, &project_sketches, display_unit)
            .ok();

        // Step 2: Re-bind Operation geometry selections
        for op in &mut setup.operations {
            if op.suppressed {
                report
                    .operation_outcomes
                    .push(OperationReloadOutcome::SuppressedSkipped {
                        setup_name: setup.name.clone(),
                        op_id: op.id.clone(),
                        op_name: op.name.clone(),
                    });
                continue;
            }

            if op.selections.is_empty() {
                if let Some(rs) = &resolved_setup_opt {
                    let mut eval_ctx = rs.eval_context(display_unit);
                    if let Some(tid) = &op.tool_id
                        && let Some(t) = project_tools.iter().find(|t| &t.id == tid)
                    {
                        t.populate_eval_context(&mut eval_ctx);
                    }
                    op.reevaluate_all(&eval_ctx);
                }
                op.runtime_status = OperationStatus::Dirty;
                report.rebound_dirty_count += 1;
                report
                    .operation_outcomes
                    .push(OperationReloadOutcome::ReboundDirty {
                        setup_name: setup.name.clone(),
                        op_id: op.id.clone(),
                        op_name: op.name.clone(),
                        updated_indices: 0,
                    });
                continue;
            }

            let mut updated_selections: Vec<GeometrySelectionItem> =
                Vec::with_capacity(op.selections.len());
            let mut claimed_faces: HashSet<FaceId> = HashSet::new();
            let mut claimed_coaxial_reps: HashSet<FaceId> = HashSet::new();
            let mut changed_index_count = 0usize;
            let mut failure_reasons: Vec<String> = Vec::new();

            for item in &op.selections {
                match item {
                    GeometrySelectionItem::PlanarFace { binding } => {
                        match rematch_face(new_doc, binding) {
                            RematchOutcome::Matched {
                                id,
                                updated_binding,
                                index_changed,
                                ..
                            } => {
                                if !claimed_faces.insert(id) {
                                    failure_reasons.push(format!(
                                        "Planar Face #{} merged into an already-selected face #{}.",
                                        binding.last_index, id.0
                                    ));
                                } else {
                                    if index_changed {
                                        changed_index_count += 1;
                                    }
                                    updated_selections.push(GeometrySelectionItem::PlanarFace {
                                        binding: updated_binding,
                                    });
                                }
                            }
                            RematchOutcome::Ambiguous { message, .. }
                            | RematchOutcome::Missing { message } => {
                                failure_reasons.push(message);
                            }
                        }
                    }
                    GeometrySelectionItem::ClosedLoop {
                        binding,
                        reverse,
                        side_flip,
                    } => match rematch_loop(new_doc, binding) {
                        RematchOutcome::Matched {
                            updated_binding,
                            index_changed,
                            ..
                        } => {
                            if index_changed {
                                changed_index_count += 1;
                            }
                            updated_selections.push(GeometrySelectionItem::ClosedLoop {
                                binding: updated_binding,
                                reverse: *reverse,
                                side_flip: *side_flip,
                            });
                        }
                        RematchOutcome::Ambiguous { message, .. }
                        | RematchOutcome::Missing { message } => {
                            failure_reasons.push(message);
                        }
                    },
                    GeometrySelectionItem::OpenChain {
                        start_edge,
                        end_edge,
                        reverse,
                        side_flip,
                        ..
                    } => {
                        let s_out = rematch_edge(new_doc, start_edge);
                        let e_out = rematch_edge(new_doc, end_edge);
                        match (s_out, e_out) {
                            (
                                RematchOutcome::Matched {
                                    id: s_eid,
                                    updated_binding: s_bind,
                                    index_changed: s_chg,
                                    ..
                                },
                                RematchOutcome::Matched {
                                    id: e_eid,
                                    updated_binding: e_bind,
                                    index_changed: e_chg,
                                    ..
                                },
                            ) => {
                                if let Some(chained) =
                                    topo.find_open_edge_chain(new_doc, s_eid, e_eid)
                                {
                                    if s_chg || e_chg {
                                        changed_index_count += 1;
                                    }
                                    updated_selections.push(GeometrySelectionItem::OpenChain {
                                        body_id: s_bind.body_id,
                                        start_edge: s_bind,
                                        end_edge: e_bind,
                                        resolved_edges: chained,
                                        reverse: *reverse,
                                        side_flip: *side_flip,
                                    });
                                } else {
                                    failure_reasons.push(format!(
                                        "Open chain between re-matched Edge #{} and Edge #{} is no longer connected.",
                                        s_eid.0, e_eid.0
                                    ));
                                }
                            }
                            (
                                RematchOutcome::Ambiguous { message, .. }
                                | RematchOutcome::Missing { message },
                                _,
                            )
                            | (
                                _,
                                RematchOutcome::Ambiguous { message, .. }
                                | RematchOutcome::Missing { message },
                            ) => {
                                failure_reasons.push(message);
                            }
                        }
                    }
                    GeometrySelectionItem::CylindricalHole { binding } => {
                        match rematch_face(new_doc, binding) {
                            RematchOutcome::Matched {
                                id,
                                updated_binding,
                                index_changed,
                                ..
                            } => {
                                let group = expand_coaxial_hole_faces(new_doc, id);
                                let rep = group.iter().copied().min().unwrap_or(id);
                                // Note: if the original operation stored both half-cylinder faces of a single hole,
                                // check whether `binding` was already coaxial with an earlier item OR a distinct hole!
                                // Allow coaxial siblings only if their old centroids matched within `0.1 mm`; otherwise guard 1-to-1!
                                let was_same_coaxial_in_old = op.selections.iter().any(|other| {
                                    if let GeometrySelectionItem::CylindricalHole { binding: ob } =
                                        other
                                    {
                                        ob.last_index != binding.last_index
                                            && (ob.signature.centroid_mm
                                                - binding.signature.centroid_mm)
                                                .length()
                                                <= 0.25
                                    } else {
                                        false
                                    }
                                });

                                if !was_same_coaxial_in_old && !claimed_coaxial_reps.insert(rep) {
                                    failure_reasons.push(format!(
                                        "Hole #{} collided onto already-matched Hole #{}; original hole was likely deleted.",
                                        binding.last_index, id.0
                                    ));
                                } else {
                                    claimed_coaxial_reps.insert(rep);
                                    if index_changed {
                                        changed_index_count += 1;
                                    }
                                    updated_selections.push(
                                        GeometrySelectionItem::CylindricalHole {
                                            binding: updated_binding,
                                        },
                                    );
                                }
                            }
                            RematchOutcome::Ambiguous { message, .. }
                            | RematchOutcome::Missing { message } => {
                                failure_reasons.push(message);
                            }
                        }
                    }
                    GeometrySelectionItem::SketchRegion { sketch_id, .. }
                    | GeometrySelectionItem::SketchEntity { sketch_id, .. }
                    | GeometrySelectionItem::SketchLayerAll { sketch_id, .. } => {
                        if let Some(err) = broken_sketch_reasons.get(sketch_id) {
                            failure_reasons.push(err.clone());
                        } else {
                            if updated_sketch_indices.contains(sketch_id) {
                                changed_index_count += 1;
                            }
                            updated_selections.push(item.clone());
                        }
                    }
                }
            }

            // Always commit surviving updated bindings so valid selections stay fresh!
            if !updated_selections.is_empty() {
                op.selections = updated_selections;
            }

            if failure_reasons.is_empty() {
                if let Some(rs) = &resolved_setup_opt {
                    let mut eval_ctx = rs.eval_context(display_unit);
                    if let Some(tid) = &op.tool_id
                        && let Some(t) = project_tools.iter().find(|t| &t.id == tid)
                    {
                        t.populate_eval_context(&mut eval_ctx);
                    }
                    if let Ok(sum) = extract_selection_features_with_sketches(
                        new_doc,
                        &project_sketches,
                        rs,
                        &op.selections,
                    ) {
                        eval_ctx.set_var("feature_top_z", sum.inferred_top_z_wcs);
                        eval_ctx.set_var("feature_bottom_z", sum.inferred_bottom_z_wcs);
                        if let Some(hd) = sum.primary_hole_diameter_mm {
                            eval_ctx.set_var("hole_d", hd);
                        }
                    }
                    op.reevaluate_all(&eval_ctx);
                }
                op.runtime_status = OperationStatus::Dirty;
                report.rebound_dirty_count += 1;
                report
                    .operation_outcomes
                    .push(OperationReloadOutcome::ReboundDirty {
                        setup_name: setup.name.clone(),
                        op_id: op.id.clone(),
                        op_name: op.name.clone(),
                        updated_indices: changed_index_count,
                    });
            } else {
                let reason = failure_reasons.join(" ");
                op.runtime_status = OperationStatus::NeedsRepick {
                    message: reason.clone(),
                };
                report.needs_repick_count += 1;
                report
                    .operation_outcomes
                    .push(OperationReloadOutcome::NeedsRepick {
                        setup_name: setup.name.clone(),
                        op_id: op.id.clone(),
                        op_name: op.name.clone(),
                        reason,
                    });
            }
        }
    }

    report
}

fn rebind_setup_wcs(
    setup: &mut chip_cam::Setup,
    new_doc: &CadDocument,
    warnings: &mut Vec<String>,
) {
    match &mut setup.wcs.orientation {
        WcsOrientationMode::ModelOrientation { .. } => {}
        WcsOrientationMode::SelectZAndX { z_ref, x_ref, .. } => {
            rebind_axis_ref(z_ref, new_doc, &setup.name, "+Z axis", warnings);
            if let Some(xr) = x_ref {
                rebind_axis_ref(xr, new_doc, &setup.name, "+X axis", warnings);
            }
        }
        WcsOrientationMode::SelectXYPlane {
            face_ref,
            x_edge_ref,
            ..
        } => {
            rebind_face_binding(face_ref, new_doc, &setup.name, "XY plane face", warnings);
            if let Some(er) = x_edge_ref {
                rebind_edge_binding(er, new_doc, &setup.name, "X alignment edge", warnings);
            }
        }
    }

    if let WcsOriginMode::SelectedCadFeature { feature } = &mut setup.wcs.origin {
        match feature {
            WcsFeaturePointRef::EdgeStart(b)
            | WcsFeaturePointRef::EdgeMidpoint(b)
            | WcsFeaturePointRef::EdgeEnd(b)
            | WcsFeaturePointRef::CircleCenter(b) => {
                rebind_edge_binding(b, new_doc, &setup.name, "WCS origin edge", warnings);
            }
            WcsFeaturePointRef::HoleAxisCenter { hole_face, .. } => {
                rebind_face_binding(hole_face, new_doc, &setup.name, "WCS origin hole", warnings);
            }
        }
    }
}

fn rebind_axis_ref(
    axis_ref: &mut WcsAxisRef,
    new_doc: &CadDocument,
    setup_name: &str,
    label: &str,
    warnings: &mut Vec<String>,
) {
    match axis_ref {
        WcsAxisRef::FaceAxis(b) => rebind_face_binding(b, new_doc, setup_name, label, warnings),
        WcsAxisRef::EdgeDirection(b) => {
            rebind_edge_binding(b, new_doc, setup_name, label, warnings)
        }
    }
}

fn rebind_face_binding(
    binding: &mut SelectionBinding,
    new_doc: &CadDocument,
    setup_name: &str,
    label: &str,
    warnings: &mut Vec<String>,
) {
    match rematch_face(new_doc, binding) {
        RematchOutcome::Matched {
            updated_binding, ..
        } => {
            *binding = updated_binding;
        }
        RematchOutcome::Ambiguous { message, .. } | RematchOutcome::Missing { message } => {
            warnings.push(format!("Setup '{setup_name}' {label}: {message}"));
        }
    }
}

fn rebind_edge_binding(
    binding: &mut SelectionBinding,
    new_doc: &CadDocument,
    setup_name: &str,
    label: &str,
    warnings: &mut Vec<String>,
) {
    match rematch_edge(new_doc, binding) {
        RematchOutcome::Matched {
            updated_binding, ..
        } => {
            *binding = updated_binding;
        }
        RematchOutcome::Ambiguous { message, .. } | RematchOutcome::Missing { message } => {
            warnings.push(format!("Setup '{setup_name}' {label}: {message}"));
        }
    }
}
