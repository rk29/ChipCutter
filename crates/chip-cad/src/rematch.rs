use crate::{
    model::{
        CadDocument, EdgeId, EntitySignatureKind, FaceId, GeometricSignature, LoopId,
        SelectionBinding, SurfaceKind,
    },
    topology::expand_coaxial_hole_faces,
};

/// Result of re-matching a stored [`SelectionBinding`] against a newly loaded [`CadDocument`]
/// (`02-cad-selection.md` §CAD Reload & Geometric Signature Re-Matching).
#[derive(Debug, Clone, PartialEq)]
pub enum RematchOutcome<Id> {
    Matched {
        id: Id,
        updated_binding: SelectionBinding,
        cost: f64,
        index_changed: bool,
    },
    Ambiguous {
        candidate_indices: Vec<u32>,
        message: String,
    },
    Missing {
        message: String,
    },
}

const MAX_ACCEPT_COST: f64 = 0.85;
const AMBIGUITY_MARGIN: f64 = 0.045;

/// Re-matches a face [`SelectionBinding`] (`PlanarFace`, `InternalCylinderFace`, `ConicalFace`, etc.)
/// against `doc`.
pub fn rematch_face(doc: &CadDocument, binding: &SelectionBinding) -> RematchOutcome<FaceId> {
    let old_sig = &binding.signature;

    // 1. Exact fast-path if topology index & signature did not change at all
    if let Some(same_idx_face) = doc.face(FaceId(binding.last_index))
        && signatures_nearly_identical(&same_idx_face.signature, old_sig)
        && let Some(updated) = doc.bind_face(same_idx_face.id)
    {
        return RematchOutcome::Matched {
            id: same_idx_face.id,
            updated_binding: updated,
            cost: 0.0,
            index_changed: false,
        };
    }

    let body_diag = doc.aabb_mm.diagonal().max(25.0);
    let old_n = old_sig.normal_or_axis.normalize_or_zero();
    let is_cylinder = matches!(
        old_sig.kind,
        EntitySignatureKind::InternalCylinderFace | EntitySignatureKind::ExternalCylinderFace
    );

    struct FaceCandidate {
        face_id: FaceId,
        coaxial_group_rep: FaceId,
        cost: f64,
    }

    let mut candidates: Vec<FaceCandidate> = Vec::new();

    for face in &doc.faces {
        if face.signature.kind != old_sig.kind {
            continue;
        }

        let cand_n = face.signature.normal_or_axis.normalize_or_zero();
        let cos_align = match old_sig.kind {
            EntitySignatureKind::PlanarFace => cand_n.dot(old_n),
            EntitySignatureKind::InternalCylinderFace
            | EntitySignatureKind::ExternalCylinderFace
            | EntitySignatureKind::ConicalFace => cand_n.dot(old_n).abs(),
            _ => cand_n.dot(old_n),
        };
        if cos_align < 0.998 {
            continue;
        }
        let orient_cost = (1.0 - cos_align.min(1.0)) * 20.0;

        if is_cylinder {
            let r_old = old_sig.characteristic_measure.max(0.2);
            let r_new = face.signature.characteristic_measure.max(0.2);
            let rel_r = (r_new - r_old).abs() / r_old;
            if rel_r > 0.32 {
                continue;
            }

            let group = expand_coaxial_hole_faces(doc, face.id);
            let group_rep = group.iter().copied().min().unwrap_or(face.id);

            // `GeometricSignature::centroid_mm` on cylinders stores the exact 3D axis midpoint `(cx, cy, z_mid)`,
            // making cylinder matching 100% invariant to STEP half-cylinder seam orientation!
            let pos_shift_mm = if let SurfaceKind::Cylinder {
                axis_origin_mm,
                axis_dir,
                ..
            } = face.surface
            {
                let u = axis_dir.normalize_or_zero();
                let v = old_sig.centroid_mm - axis_origin_mm;
                let radial_axis_shift = (v - u * v.dot(u)).length();
                let mut grp_aabb = face.aabb_mm;
                for &gfid in &group {
                    if let Some(gf) = doc.face(gfid) {
                        grp_aabb = grp_aabb.union(&gf.aabb_mm);
                    }
                }
                let axial_shift = (grp_aabb.center() - old_sig.centroid_mm).dot(u).abs();
                radial_axis_shift.hypot(axial_shift)
            } else {
                (face.signature.centroid_mm - old_sig.centroid_mm).length()
            };

            let max_hole_shift_mm = (3.2 * r_old).clamp(6.0, 22.0);
            if pos_shift_mm > max_hole_shift_mm {
                continue;
            }

            let aabb_tie =
                ((face.aabb_mm.center() - old_sig.aabb_mm.center()).length() / body_diag) * 0.002;
            let cost =
                orient_cost + 1.4 * rel_r + 0.55 * (pos_shift_mm / max_hole_shift_mm) + aabb_tie;
            if cost <= MAX_ACCEPT_COST {
                candidates.push(FaceCandidate {
                    face_id: face.id,
                    coaxial_group_rep: group_rep,
                    cost,
                });
            }
        } else {
            let a_old = old_sig.characteristic_measure.max(1.0);
            let a_new = face.signature.characteristic_measure.max(1.0);
            let log_area_diff = (a_new / a_old).ln().abs();
            if log_area_diff > 1.30 {
                continue;
            }

            let shift_mm = (face.signature.centroid_mm - old_sig.centroid_mm).length();
            let max_face_shift = (0.35 * body_diag).max(16.0);
            if shift_mm > max_face_shift {
                continue;
            }

            let aabb_center_shift =
                (face.signature.aabb_mm.center() - old_sig.aabb_mm.center()).length();
            let cost = orient_cost
                + 0.38 * log_area_diff
                + 0.42 * (shift_mm / max_face_shift)
                + 0.20 * (aabb_center_shift / max_face_shift);
            if cost <= MAX_ACCEPT_COST {
                candidates.push(FaceCandidate {
                    face_id: face.id,
                    coaxial_group_rep: face.id,
                    cost,
                });
            }
        }
    }

    candidates.sort_by(|a, b| a.cost.total_cmp(&b.cost));
    let Some(best) = candidates.first() else {
        return RematchOutcome::Missing {
            message: format!(
                "Face #{} ({:?}) no longer exists in revised STEP model.",
                binding.last_index, old_sig.kind
            ),
        };
    };

    // Check runner-up from a *distinct* topological feature (ignoring coaxial half-cylinder siblings of the same hole!)
    if let Some(runner_up) = candidates
        .iter()
        .find(|c| c.coaxial_group_rep != best.coaxial_group_rep)
        && (runner_up.cost - best.cost) < AMBIGUITY_MARGIN
    {
        return RematchOutcome::Ambiguous {
            candidate_indices: vec![best.face_id.0, runner_up.face_id.0],
            message: format!(
                "Face #{} matched multiple ambiguous candidates (#{} cost {:.3} vs #{} cost {:.3}); please re-pick.",
                binding.last_index, best.face_id.0, best.cost, runner_up.face_id.0, runner_up.cost
            ),
        };
    }

    let Some(updated_binding) = doc.bind_face(best.face_id) else {
        return RematchOutcome::Missing {
            message: format!("Failed to bind matched Face #{}.", best.face_id.0),
        };
    };

    RematchOutcome::Matched {
        id: best.face_id,
        updated_binding,
        cost: best.cost,
        index_changed: best.face_id.0 != binding.last_index,
    }
}

/// Re-matches a closed boundary loop [`SelectionBinding`] against `doc`.
pub fn rematch_loop(doc: &CadDocument, binding: &SelectionBinding) -> RematchOutcome<LoopId> {
    let old_sig = &binding.signature;
    if let Some(same_lp) = doc.brep_loop(LoopId(binding.last_index))
        && signatures_nearly_identical(&same_lp.signature, old_sig)
        && let Some(updated) = doc.bind_loop(same_lp.id)
    {
        return RematchOutcome::Matched {
            id: same_lp.id,
            updated_binding: updated,
            cost: 0.0,
            index_changed: false,
        };
    }

    let body_diag = doc.aabb_mm.diagonal().max(25.0);
    let max_shift = (0.35 * body_diag).max(16.0);
    let old_n = old_sig.normal_or_axis.normalize_or_zero();
    let p_old = old_sig.characteristic_measure.max(1.0);

    let mut scored: Vec<(LoopId, f64)> = Vec::new();
    for lp in &doc.loops {
        let cand_n = lp.signature.normal_or_axis.normalize_or_zero();
        let cos_align = cand_n.dot(old_n);
        if cos_align < 0.995 {
            continue;
        }
        let p_new = lp.signature.characteristic_measure.max(1.0);
        let rel_perim = (p_new - p_old).abs() / p_old;
        if rel_perim > 0.45 {
            continue;
        }
        let shift_mm = (lp.signature.centroid_mm - old_sig.centroid_mm).length();
        if shift_mm > max_shift {
            continue;
        }
        let cost =
            (1.0 - cos_align.min(1.0)) * 20.0 + 0.50 * rel_perim + 0.50 * (shift_mm / max_shift);
        if cost <= MAX_ACCEPT_COST {
            scored.push((lp.id, cost));
        }
    }

    scored.sort_by(|a, b| a.1.total_cmp(&b.1));
    let Some(&(best_id, best_cost)) = scored.first() else {
        return RematchOutcome::Missing {
            message: format!(
                "Edge Loop #{} no longer exists in revised STEP model.",
                binding.last_index
            ),
        };
    };

    if let Some(&(second_id, second_cost)) = scored.get(1)
        && (second_cost - best_cost) < AMBIGUITY_MARGIN
    {
        return RematchOutcome::Ambiguous {
            candidate_indices: vec![best_id.0, second_id.0],
            message: format!(
                "Loop #{} matched multiple ambiguous loops (#{} vs #{}); please re-pick.",
                binding.last_index, best_id.0, second_id.0
            ),
        };
    }

    let Some(updated_binding) = doc.bind_loop(best_id) else {
        return RematchOutcome::Missing {
            message: format!("Failed to bind matched Loop #{}.", best_id.0),
        };
    };

    RematchOutcome::Matched {
        id: best_id,
        updated_binding,
        cost: best_cost,
        index_changed: best_id.0 != binding.last_index,
    }
}

/// Re-matches a topological edge [`SelectionBinding`] against `doc`.
pub fn rematch_edge(doc: &CadDocument, binding: &SelectionBinding) -> RematchOutcome<EdgeId> {
    let old_sig = &binding.signature;
    if let Some(same_edge) = doc.edge(EdgeId(binding.last_index))
        && signatures_nearly_identical(&same_edge.signature, old_sig)
        && let Some(updated) = doc.bind_edge(same_edge.id)
    {
        return RematchOutcome::Matched {
            id: same_edge.id,
            updated_binding: updated,
            cost: 0.0,
            index_changed: false,
        };
    }

    let body_diag = doc.aabb_mm.diagonal().max(25.0);
    let max_shift = (0.28 * body_diag).max(12.0);
    let old_dir = old_sig.normal_or_axis.normalize_or_zero();
    let m_old = old_sig.characteristic_measure.max(0.25);

    let mut scored: Vec<(EdgeId, f64)> = Vec::new();
    for edge in &doc.edges {
        if edge.signature.kind != old_sig.kind {
            continue;
        }
        let cand_dir = edge.signature.normal_or_axis.normalize_or_zero();
        let cos_align = cand_dir.dot(old_dir).abs();
        if cos_align < 0.995 {
            continue;
        }
        let m_new = edge.signature.characteristic_measure.max(0.25);
        let rel_m = (m_new - m_old).abs() / m_old;
        if rel_m > 0.45 {
            continue;
        }
        let shift_mm = (edge.signature.centroid_mm - old_sig.centroid_mm).length();
        if shift_mm > max_shift {
            continue;
        }
        let cost = (1.0 - cos_align.min(1.0)) * 20.0 + 0.40 * rel_m + 0.60 * (shift_mm / max_shift);
        if cost <= MAX_ACCEPT_COST {
            scored.push((edge.id, cost));
        }
    }

    scored.sort_by(|a, b| a.1.total_cmp(&b.1));
    let Some(&(best_id, best_cost)) = scored.first() else {
        return RematchOutcome::Missing {
            message: format!(
                "Edge #{} ({:?}) no longer exists in revised STEP model.",
                binding.last_index, old_sig.kind
            ),
        };
    };

    if let Some(&(second_id, second_cost)) = scored.get(1)
        && (second_cost - best_cost) < AMBIGUITY_MARGIN
    {
        return RematchOutcome::Ambiguous {
            candidate_indices: vec![best_id.0, second_id.0],
            message: format!(
                "Edge #{} matched multiple ambiguous edges (#{} vs #{}); please re-pick.",
                binding.last_index, best_id.0, second_id.0
            ),
        };
    }

    let Some(updated_binding) = doc.bind_edge(best_id) else {
        return RematchOutcome::Missing {
            message: format!("Failed to bind matched Edge #{}.", best_id.0),
        };
    };

    RematchOutcome::Matched {
        id: best_id,
        updated_binding,
        cost: best_cost,
        index_changed: best_id.0 != binding.last_index,
    }
}

fn signatures_nearly_identical(a: &GeometricSignature, b: &GeometricSignature) -> bool {
    a.kind == b.kind
        && (a.normal_or_axis - b.normal_or_axis).length() <= 1e-4
        && (a.centroid_mm - b.centroid_mm).length() <= 1e-3
        && (a.characteristic_measure - b.characteristic_measure).abs()
            <= 1e-3 * b.characteristic_measure.abs().max(1.0)
        && (a.aabb_mm.min - b.aabb_mm.min).length() <= 1e-3
        && (a.aabb_mm.max - b.aabb_mm.max).length() <= 1e-3
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, approx_eq};

    use super::*;
    use crate::{
        CadKernel, MonstertruckKernel, SurfaceKind, sample_ambiguous_twin_hole_step,
        sample_revised_part_pair_step,
    };

    #[test]
    fn rematches_resized_pocket_moved_holes_permuted_ids_and_detects_missing_or_ambiguous_features()
    {
        let (v1_step, v2_step) = sample_revised_part_pair_step().unwrap();
        let twins_step = sample_ambiguous_twin_hole_step().unwrap();

        let doc_v1 = MonstertruckKernel
            .load_step_bytes("v1.step", v1_step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let doc_v2 = MonstertruckKernel
            .load_step_bytes("v2.step", v2_step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let doc_twins = MonstertruckKernel
            .load_step_bytes(
                "twins.step",
                twins_step.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .unwrap();

        // 1. Pocket floor on v1 (`Z = 12.0 mm`) -> re-matches to deepened/widened v2 pocket (`Z = 11.0 mm`) even with permuted FaceId!
        let v1_pkt = doc_v1
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { origin_mm, normal }
                    if normal.z > 0.99 && approx_eq(origin_mm.z, 12.0, 1e-2)
                )
            })
            .unwrap();
        let pkt_bind = doc_v1.bind_face(v1_pkt.id).unwrap();

        // Exact fast-path on reloading v1
        assert!(matches!(
            rematch_face(&doc_v1, &pkt_bind),
            RematchOutcome::Matched {
                index_changed: false,
                cost,
                ..
            } if cost == 0.0
        ));

        let RematchOutcome::Matched {
            id: v2_pkt_id,
            index_changed,
            ..
        } = rematch_face(&doc_v2, &pkt_bind)
        else {
            panic!("expected pocket floor to re-match on v2");
        };
        assert!(
            index_changed,
            "v2 permuted shell order must shift FaceId to exercise true geometric matching"
        );
        let v2_pkt_face = doc_v2.face(v2_pkt_id).unwrap();
        assert!(matches!(
            v2_pkt_face.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && approx_eq(origin_mm.z, 11.0, 1e-2)
        ));

        // 2. Ø14mm bore at (68, 24) -> matches shifted (71, 26) on v2, and triggers Ambiguous on `doc_twins`!
        let v1_bore = doc_v1
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Cylinder {
                        radius_mm,
                        is_internal: true,
                        ..
                    } if approx_eq(radius_mm, 7.0, 1e-2)
                )
            })
            .unwrap();
        let bore_bind = doc_v1.bind_face(v1_bore.id).unwrap();

        let RematchOutcome::Matched { id: v2_bore_id, .. } = rematch_face(&doc_v2, &bore_bind)
        else {
            panic!("expected moved Ø14mm bore to re-match on v2");
        };
        let SurfaceKind::Cylinder { axis_origin_mm, .. } = doc_v2.face(v2_bore_id).unwrap().surface
        else {
            panic!("expected cylinder");
        };
        assert!(approx_eq(axis_origin_mm.x, 71.0, 0.1));
        assert!(approx_eq(axis_origin_mm.y, 26.0, 0.1));

        assert!(
            matches!(
                rematch_face(&doc_twins, &bore_bind),
                RematchOutcome::Ambiguous { .. }
            ),
            "equidistant twin Ø14mm bores must trigger Ambiguous"
        );

        // 3. Ø6mm hole #1 at (68, 48) matches (70, 47), while deleted hole #2 at (68, 60) returns Missing!
        let v1_hole_keep = doc_v1
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Cylinder {
                        axis_origin_mm,
                        radius_mm,
                        is_internal: true,
                        ..
                    } if approx_eq(radius_mm, 3.0, 1e-2) && approx_eq(axis_origin_mm.y, 48.0, 0.2)
                )
            })
            .unwrap();
        let v1_hole_del = doc_v1
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Cylinder {
                        axis_origin_mm,
                        radius_mm,
                        is_internal: true,
                        ..
                    } if approx_eq(radius_mm, 3.0, 1e-2) && approx_eq(axis_origin_mm.y, 60.0, 0.2)
                )
            })
            .unwrap();

        assert!(matches!(
            rematch_face(&doc_v2, &doc_v1.bind_face(v1_hole_keep.id).unwrap()),
            RematchOutcome::Matched { .. }
        ));
        assert!(matches!(
            rematch_face(&doc_v2, &doc_v1.bind_face(v1_hole_del.id).unwrap()),
            RematchOutcome::Missing { .. }
        ));

        // 4. Outer bottom loop & top chamfer edge re-match cleanly on v2
        let v1_bot_face = doc_v1
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { origin_mm, normal }
                    if normal.z < -0.99 && approx_eq(origin_mm.z, 0.0, 1e-2)
                )
            })
            .unwrap();
        let bot_loop_bind = doc_v1.bind_loop(v1_bot_face.outer_loop.unwrap()).unwrap();
        assert!(matches!(
            rematch_loop(&doc_v2, &bot_loop_bind),
            RematchOutcome::Matched { .. }
        ));

        let v1_chamf_edge = doc_v1
            .edges
            .iter()
            .find(|e| {
                approx_eq(e.aabb_mm.min.z, 20.0, 1e-2)
                    && approx_eq(e.aabb_mm.min.x, 80.0, 1e-2)
                    && approx_eq(e.aabb_mm.max.x, 90.0, 1e-2)
            })
            .unwrap();
        let edge_bind = doc_v1.bind_edge(v1_chamf_edge.id).unwrap();
        let RematchOutcome::Matched { id: v2_edge_id, .. } = rematch_edge(&doc_v2, &edge_bind)
        else {
            panic!("expected top chamfer edge to re-match on v2");
        };
        assert!(approx_eq(
            doc_v2.edge(v2_edge_id).unwrap().aabb_mm.max.x,
            92.0,
            1e-2
        ));
    }
}
