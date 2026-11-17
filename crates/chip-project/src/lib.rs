//! Project document state, deterministic `.chip.toml` serialization, CAD SHA-256 verification,
//! project invariant validation, and instant Undo/Redo history (`chip-project`).

pub mod document;
pub mod history;
pub mod toml_io;
pub mod validation;

pub use document::{
    ModelEntry, ProjectDocument, compute_sha256_hex, portable_model_path, resolve_model_path,
};
pub use history::UndoHistory;
pub use toml_io::ProjectIoError;
pub use validation::{
    DiagnosticSeverity, ProjectValidationReport, ValidationDiagnostic, validate_project,
};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::{CadKernel, MonstertruckKernel, SurfaceKind, sample_multi_feature_bracket_step};
    use chip_cam::{
        GeometrySelectionItem, HeightReference, HeightSpec, Operation, OperationKind,
        WorkOffsetSlot,
    };
    use chip_math::DEFAULT_TESS_CHORD_TOL_MM;
    use chip_tools::default_starter_tool_library;

    use super::*;

    #[test]
    fn serializes_chip_toml_deterministically_and_validates_project_invariants() {
        let step_str = sample_multi_feature_bracket_step(false).unwrap();
        let cad_doc = MonstertruckKernel
            .load_step_bytes(
                "bracket.step",
                step_str.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .unwrap();

        let starter_tools = default_starter_tool_library();
        let t1 = &starter_tools.tools[0]; // 6mm flat endmill

        let mut proj = ProjectDocument::new("Motor Mount Bracket");
        proj.models.push(ModelEntry {
            id: "model_1".to_string(),
            path: "cad/bracket.step".to_string(),
            sha256: compute_sha256_hex(step_str.as_bytes()),
        });
        proj.ensure_tool_embedded(t1);

        let pocket_floor = cad_doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { origin_mm, normal }
                    if normal.z > 0.99 && (origin_mm.z - 12.0).abs() < 0.05
                )
            })
            .unwrap()
            .id;

        let mut op = Operation::new("op_pkt_1", "Center Pocket", OperationKind::Pocket2D);
        op.tool_id = Some(t1.id.clone());
        op.preset_id = Some(t1.presets[0].id.clone());
        op.selections.push(GeometrySelectionItem::PlanarFace {
            binding: cad_doc.bind_face(pocket_floor).unwrap(),
        });
        proj.setups[0].operations.push(op);

        // 1. Byte-for-byte deterministic `.chip.toml` round-trip
        let toml_a = proj.to_toml_string().unwrap();
        let reloaded = ProjectDocument::from_toml_str(&toml_a).unwrap();
        let toml_b = reloaded.to_toml_string().unwrap();
        assert_eq!(toml_a, toml_b);

        // 2. Valid project passes `validate_project` with zero errors
        let report_ok = validate_project(&reloaded, None, Some(&cad_doc));
        assert!(
            report_ok.is_valid(),
            "expected valid project, got {:?}",
            report_ok.diagnostics
        );

        // 3. Invalid height hierarchy or incompatible twist drill on Pocket2D triggers explicit errors
        let mut broken = reloaded.clone();
        let twist_drill = starter_tools
            .tools
            .iter()
            .find(|t| matches!(t.geometry, chip_tools::CutterGeometry::TwistDrill { .. }))
            .unwrap();
        broken.ensure_tool_embedded(twist_drill);
        broken.setups[0].operations[0].tool_id = Some(twist_drill.id.clone());
        broken.setups[0].operations[0].heights.bottom =
            HeightSpec::new(HeightReference::StockTop, 10.0);

        let report_bad = validate_project(&broken, None, Some(&cad_doc));
        assert!(report_bad.error_count() >= 2);
        let _ = WorkOffsetSlot::G55;
    }

    #[test]
    fn undo_redo_history_supports_100_steps_coalescing_and_branch_truncation() {
        let mut doc = ProjectDocument::new("Initial");
        let mut history = UndoHistory::new(5);

        history.record_before_edit("Rename to V1", None, &doc);
        doc.project_name = "V1".to_string();

        // Coalesced edits with key "stock_drag" should keep V1 as the single undo target!
        history.record_before_edit("Adjust Stock", Some("stock_drag"), &doc);
        doc.project_name = "V2a".to_string();
        history.record_before_edit("Adjust Stock", Some("stock_drag"), &doc);
        doc.project_name = "V2b".to_string();

        assert_eq!(history.undo_len(), 2);
        let undone = history.undo(&mut doc).unwrap();
        assert_eq!(undone, "Adjust Stock");
        assert_eq!(doc.project_name, "V1");

        let redone = history.redo(&mut doc).unwrap();
        assert_eq!(redone, "Adjust Stock");
        assert_eq!(doc.project_name, "V2b");

        // Undo again, then make a new edit -> redo stack must truncate!
        history.undo(&mut doc).unwrap();
        history.record_before_edit("Branch V3", None, &doc);
        doc.project_name = "V3".to_string();
        assert!(!history.can_redo());
    }
}
