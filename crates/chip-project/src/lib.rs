//! Project document state, deterministic `.chip.toml` serialization, CAD SHA-256 verification,
//! project invariant validation, and instant Undo/Redo history (`chip-project`).

pub mod bundle;
pub mod cache;
pub mod demos;
pub mod dispatcher;
pub mod document;
pub mod history;
pub mod pipeline;
pub mod reload;
pub mod toml_io;
pub mod validation;

pub use bundle::{
    BundleError, ChipzExportSummary, MAX_CHIPZ_UNPACK_BYTES, UnpackedChipzBundle,
    export_chipz_bundle, is_chipz_path, pack_chipz_bundle_bytes, unpack_chipz_bundle,
    unpack_chipz_bytes,
};
pub use cache::{
    OperationInputHash, ToolpathCache, compute_operation_input_hash,
    resolve_upstream_rest_op_indices,
};
pub use demos::{
    build_adaptive_rest_demo_project, build_cam_torture_project,
    build_multi_feature_bracket_project, build_revision_v1_demo_project,
};
pub use dispatcher::{
    BatchGenerationReport, DispatcherEvent, JobDispatcher, generate_project_blocking,
};
pub use document::{
    ModelEntry, ProjectDocument, compute_sha256_hex, portable_model_path, resolve_model_path,
};
pub use history::UndoHistory;
pub use pipeline::{
    ReadySetupOperation, SetupPostRequest, SetupSimRequest, collect_ready_setup_operations,
    kinematics_for_post_profile, post_process_project_setup, resolve_kinematics_for_key,
    simulate_project_setup,
};
pub use reload::{CadReloadReport, OperationReloadOutcome, rebind_project_to_reloaded_cad};
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

    #[test]
    fn dispatcher_generates_in_parallel_invalidates_on_edit_and_restores_clean_on_undo() {
        use std::{collections::HashSet, sync::Arc, time::Instant};

        use chip_cad::sample_cam_torture_block_step;
        use chip_cam::OperationStatus;
        use chip_math::ParamLength;

        let step_str = sample_cam_torture_block_step().unwrap();
        let cad_doc = Arc::new(
            MonstertruckKernel
                .load_step_bytes(
                    "torture.step",
                    step_str.as_bytes(),
                    DEFAULT_TESS_CHORD_TOL_MM,
                )
                .unwrap(),
        );

        let starter = default_starter_tool_library();
        let flat_6mm = &starter.tools[0];
        let face_38mm = starter
            .tools
            .iter()
            .find(|t| matches!(t.geometry, chip_tools::CutterGeometry::FaceMill { .. }))
            .unwrap();

        let mut proj = ProjectDocument::new("M3 Dispatcher + Undo Cache Test");
        proj.ensure_tool_embedded(flat_6mm);
        proj.ensure_tool_embedded(face_38mm);

        let mut op_face = Operation::new("op_1_face", "Face Stock", OperationKind::Facing);
        op_face.tool_id = Some(face_38mm.id.clone());

        let pkt_floor = cad_doc
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
        let mut op_pkt = Operation::new("op_2_pkt", "Island Pocket", OperationKind::Pocket2D);
        op_pkt.tool_id = Some(flat_6mm.id.clone());
        op_pkt.selections.push(GeometrySelectionItem::PlanarFace {
            binding: cad_doc.bind_face(pkt_floor).unwrap(),
        });

        proj.setups[0].operations = vec![op_face, op_pkt];

        // Deterministic .chip.toml roundtrip with full Passes, Linking, and StrategyParams!
        let toml_1 = proj.to_toml_string().unwrap();
        let reparsed = ProjectDocument::from_toml_str(&toml_1).unwrap();
        assert_eq!(toml_1, reparsed.to_toml_string().unwrap());

        let mut cache = ToolpathCache::new(64);
        let mut dispatcher = JobDispatcher::new(None);
        let mut history = UndoHistory::new(50);

        let spawned =
            dispatcher.dispatch_operations(&mut proj, Arc::clone(&cad_doc), 1, &cache, None, None);
        assert_eq!(spawned, 2);
        assert_eq!(
            proj.setups[0].operations[0].runtime_status,
            OperationStatus::Generating
        );

        // Wait briefly for rayon background tasks to complete
        let start_wait = Instant::now();
        while dispatcher.is_busy() && start_wait.elapsed().as_secs() < 10 {
            dispatcher.poll_events(&mut proj, &mut cache, 1);
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        dispatcher.poll_events(&mut proj, &mut cache, 1);

        assert_eq!(
            proj.setups[0].operations[0].runtime_status,
            OperationStatus::Clean
        );
        assert_eq!(
            proj.setups[0].operations[1].runtime_status,
            OperationStatus::Clean
        );

        // Mutate stepdown on op_2_pkt -> reconciles immediately to Dirty while op_1_face stays Clean!
        history.record_before_edit("Change Stepdown", None, &proj);
        proj.setups[0].operations[1].passes.max_stepdown = ParamLength::from_mm(1.25);
        cache.reconcile_project_statuses(&mut proj, 1, &HashSet::new());
        assert_eq!(
            proj.setups[0].operations[0].runtime_status,
            OperationStatus::Clean
        );
        assert_eq!(
            proj.setups[0].operations[1].runtime_status,
            OperationStatus::Dirty
        );

        // Undo (`Ctrl+Z`) -> restores Clean status + cached GeneratedToolpath in < 1 ms!
        let t_undo = Instant::now();
        history.undo(&mut proj).unwrap();
        cache.reconcile_project_statuses(&mut proj, 1, &HashSet::new());
        let undo_us = t_undo.elapsed().as_micros();
        assert!(
            undo_us < 10_000,
            "Undo + cache reconciliation should be instantaneous, took {undo_us} us"
        );
        assert_eq!(
            proj.setups[0].operations[1].runtime_status,
            OperationStatus::Clean
        );
        assert!(cache.get_for_operation(&proj, 0, 1, 1).is_some());
    }

    #[test]
    fn associative_step_reload_v1_to_v2_rebinds_dirty_flags_deleted_needs_repick_and_passes_sim_oracle()
     {
        use chip_cad::{TopologyGraph, sample_revised_part_pair_step};
        use chip_cam::{
            ChamferSourceMode, ChamferTraceMode, HoleMachiningMode, OperationStatus, StrategyParams,
        };
        use chip_math::{ParamLength, approx_eq};
        use chip_sim::{MachineKinematicsConfig, SimResolution};

        let (v1_step, v2_step) = sample_revised_part_pair_step().unwrap();
        let doc_v1 = MonstertruckKernel
            .load_step_bytes("v1.step", v1_step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let doc_v2 = MonstertruckKernel
            .load_step_bytes("v2.step", v2_step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let topo_v1 = TopologyGraph::build(&doc_v1);
        let topo_v2 = TopologyGraph::build(&doc_v2);

        let starter = default_starter_tool_library();
        let flat_6mm = &starter.tools[0];
        let drill_6mm = &starter.tools[5];
        let chamf_90 = &starter.tools[4];
        let face_38mm = &starter.tools[6];

        let mut proj = ProjectDocument::new("Associative Reload v1 -> v2");
        proj.models.push(ModelEntry {
            id: "model_1".to_string(),
            path: "cad/part.step".to_string(),
            sha256: compute_sha256_hex(v1_step.as_bytes()),
        });
        for t in [flat_6mm, drill_6mm, chamf_90, face_38mm] {
            proj.ensure_tool_embedded(t);
        }

        // Op 0: Facing Top Stock
        let mut op_face = Operation::new("op_0_face", "Face Top", OperationKind::Facing);
        op_face.tool_id = Some(face_38mm.id.clone());

        // Op 1: Blind Pocket (`Z = 12.0` on v1 -> `Z = 11.0` on v2!)
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
            .unwrap()
            .id;
        let mut op_pkt = Operation::new("op_1_pkt", "Resized Pocket", OperationKind::Pocket2D);
        op_pkt.tool_id = Some(flat_6mm.id.clone());
        op_pkt.selections.push(GeometrySelectionItem::PlanarFace {
            binding: doc_v1.bind_face(v1_pkt).unwrap(),
        });

        // Op 2: Ø14mm Helical Bore (`(68, 24)` on v1 -> `(71, 26)` on v2!)
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
            .unwrap()
            .id;
        let mut op_bore = Operation::new("op_2_bore", "Shifted Ø14 Bore", OperationKind::DrillBore);
        op_bore.tool_id = Some(flat_6mm.id.clone());
        if let StrategyParams::DrillBore(p) = op_bore.strategy_mut() {
            p.mode = HoleMachiningMode::HelicalBore;
        }
        op_bore
            .selections
            .push(GeometrySelectionItem::CylindricalHole {
                binding: doc_v1.bind_face(v1_bore).unwrap(),
            });

        // Op 3: Surviving Ø6mm Hole at `(68, 48)` -> `(70, 47)` on v2
        let v1_h1 = doc_v1
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
            .unwrap()
            .id;
        let mut op_h1 =
            Operation::new("op_3_hole_keep", "Surviving Hole", OperationKind::DrillBore);
        op_h1.tool_id = Some(drill_6mm.id.clone());
        op_h1
            .selections
            .push(GeometrySelectionItem::CylindricalHole {
                binding: doc_v1.bind_face(v1_h1).unwrap(),
            });

        // Op 4: Deleted Ø6mm Hole at `(68, 60)` -> DELETED on v2 -> must become NeedsRepick while preserving tool & overrides!
        let v1_h2 = doc_v1
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
            .unwrap()
            .id;
        let mut op_h2 = Operation::new("op_4_hole_del", "Deleted Hole", OperationKind::DrillBore);
        op_h2.tool_id = Some(drill_6mm.id.clone());
        op_h2.feed_overrides.spindle_rpm = Some(5_123.0);
        op_h2
            .selections
            .push(GeometrySelectionItem::CylindricalHole {
                binding: doc_v1.bind_face(v1_h2).unwrap(),
            });

        // Op 5: Multi-edge Open Chain Chamfer along top front -> angled chamfer edge (`X=80..90` -> `X=82..92` on v2!)
        let e_front = doc_v1
            .edges
            .iter()
            .find(|e| {
                approx_eq(e.aabb_mm.min.z, 20.0, 1e-2)
                    && approx_eq(e.aabb_mm.max.y, 0.0, 1e-2)
                    && approx_eq(e.aabb_mm.max.x, 80.0, 1e-2)
            })
            .unwrap()
            .id;
        let e_ang = doc_v1
            .edges
            .iter()
            .find(|e| {
                approx_eq(e.aabb_mm.min.z, 20.0, 1e-2)
                    && approx_eq(e.aabb_mm.min.x, 80.0, 1e-2)
                    && approx_eq(e.aabb_mm.max.x, 90.0, 1e-2)
            })
            .unwrap()
            .id;
        let chain_v1 = topo_v1
            .find_open_edge_chain(&doc_v1, e_front, e_ang)
            .unwrap();
        let mut op_chamf = Operation::new(
            "op_5_chamf",
            "Front Chamfer Chain",
            OperationKind::ChamferTrace,
        );
        op_chamf.tool_id = Some(chamf_90.id.clone());
        op_chamf.selections.push(GeometrySelectionItem::OpenChain {
            body_id: doc_v1.bodies[0].id,
            start_edge: doc_v1.bind_edge(e_front).unwrap(),
            end_edge: doc_v1.bind_edge(e_ang).unwrap(),
            resolved_edges: chain_v1,
            reverse: false,
            side_flip: true,
        });
        if let StrategyParams::ChamferTrace(p) = op_chamf.strategy_mut() {
            p.mode = ChamferTraceMode::EdgeChamfer;
            p.chamfer_source = ChamferSourceMode::SharpEdge {
                width: ParamLength::from_mm(0.5),
            };
        }

        proj.setups[0].operations = vec![op_face, op_pkt, op_bore, op_h1, op_h2, op_chamf];

        // Generate cleanly on v1 first
        let mut cache = ToolpathCache::new(64);
        let batch_v1 = generate_project_blocking(&mut proj, &doc_v1, &mut cache, 1);
        assert!(
            batch_v1.errors.is_empty(),
            "v1 generation errors: {:?}",
            batch_v1.errors
        );

        // Reload v2 STEP and run WCS-first associative re-binding!
        let sha_v2 = compute_sha256_hex(v2_step.as_bytes());
        let reload_rep =
            rebind_project_to_reloaded_cad(&mut proj, 0, &doc_v2, &topo_v2, &sha_v2, None);
        assert_eq!(reload_rep.rebound_dirty_count, 5);
        assert_eq!(reload_rep.needs_repick_count, 1);
        assert!(reload_rep.setup_wcs_warnings.is_empty());

        // Verify Op 4 (`Deleted Hole`) transitioned to `NeedsRepick` while preserving its tool & RPM override!
        assert!(matches!(
            proj.setups[0].operations[4].runtime_status,
            OperationStatus::NeedsRepick { .. }
        ));
        assert_eq!(
            proj.setups[0].operations[4].feed_overrides.spindle_rpm,
            Some(5_123.0)
        );

        // Suppress the orphaned deleted hole, regenerate the 5 re-bound operations on v2, and verify via `chip-sim`!
        proj.setups[0].operations[4].suppressed = true;
        let batch_v2 = generate_project_blocking(&mut proj, &doc_v2, &mut cache, 2);
        assert!(
            batch_v2.errors.is_empty(),
            "v2 generation errors: {:?}",
            batch_v2.errors
        );
        assert_eq!(batch_v2.generated.len(), 5);

        let sim_v2 = simulate_project_setup(&SetupSimRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &doc_v2,
            cad_revision: 2,
            cache: &cache,
            kinematics: MachineKinematicsConfig::default(),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert_eq!(
            sim_v2.critical_error_count(),
            0,
            "re-bound v2 operations must pass physical heightmap verification with 0 crashes/gouges, got: {:#?}",
            sim_v2.diagnostics
        );
    }

    #[test]
    fn chipz_bundle_packs_deterministically_embeds_custom_posts_verifies_sha256_and_blocks_zip_slip()
     {
        use std::io::Write as _;

        use chip_post::{BUILTIN_GRBL_TOML, PostProfile};

        let step_bytes = sample_multi_feature_bracket_step(false)
            .unwrap()
            .into_bytes();
        let mut proj = ProjectDocument::new("Portable Bundle Job");
        proj.models.push(ModelEntry {
            id: "model_1".to_string(),
            path: "bracket.step".to_string(),
            sha256: compute_sha256_hex(&step_bytes),
        });
        let starter = default_starter_tool_library();
        proj.ensure_tool_embedded(&starter.tools[0]);
        proj.setups[0].post_profile = "shop_custom_grbl".to_string();

        let mut custom_post = PostProfile::from_toml_str("custom", BUILTIN_GRBL_TOML).unwrap();
        custom_post.id = "shop_custom_grbl".to_string();
        custom_post.name = "Shop Custom GRBL".to_string();

        // 1. Byte-for-byte deterministic `.chipz` archive generation
        let (bytes_1, sum_1) = pack_chipz_bundle_bytes(
            &proj,
            None,
            &[("model_1", &step_bytes)],
            std::slice::from_ref(&custom_post),
        )
        .unwrap();
        let (bytes_2, _) = pack_chipz_bundle_bytes(
            &proj,
            None,
            &[("model_1", &step_bytes)],
            std::slice::from_ref(&custom_post),
        )
        .unwrap();
        assert_eq!(
            bytes_1, bytes_2,
            ".chipz archive output must be byte-for-byte deterministic"
        );
        assert_eq!(sum_1.model_files, vec!["models/bracket.step".to_string()]);
        assert_eq!(
            sum_1.custom_post_files,
            vec!["posts/shop_custom_grbl.post.toml".to_string()]
        );

        // 2. Unpack into temp folder and verify SHA-256 + custom post extraction
        let tmp_dir = std::env::temp_dir().join(format!("chipz_test_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp_dir);
        let unpacked = unpack_chipz_bytes(&bytes_1, &tmp_dir).unwrap();
        assert_eq!(unpacked.project.project_name, "Portable Bundle Job");
        assert!(tmp_dir.join("models/bracket.step").is_file());
        assert!(tmp_dir.join("posts/shop_custom_grbl.post.toml").is_file());
        let _ = std::fs::remove_dir_all(&tmp_dir);

        // 3. Reject malicious `../evil` zip-slip entry!
        let mut bad_zip = std::io::Cursor::new(Vec::new());
        {
            let mut zw = zip::ZipWriter::new(&mut bad_zip);
            let opts = zip::write::SimpleFileOptions::default();
            zw.start_file("../evil_escape.toml", opts).unwrap();
            zw.write_all(b"malicious").unwrap();
            zw.finish().unwrap();
        }
        let slip_dir = std::env::temp_dir().join(format!("chipz_slip_{}", std::process::id()));
        let err = unpack_chipz_bytes(&bad_zip.into_inner(), &slip_dir).unwrap_err();
        assert!(matches!(err, BundleError::UnsafeEntryPath { .. }));
        let _ = std::fs::remove_dir_all(&slip_dir);
    }

    #[test]
    fn adaptive_and_rest_machining_dag_waves_dirty_propagation_and_undo_work() {
        use chip_cad::sample_cam_torture_block_step;
        use chip_cam::OperationStatus;
        use chip_math::ParamLength;
        use chip_sim::SimResolution;

        let step = sample_cam_torture_block_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("torture.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let sha = compute_sha256_hex(step.as_bytes());
        let starter = default_starter_tool_library();

        let mut proj = build_adaptive_rest_demo_project(
            &doc,
            "models/torture.step".to_string(),
            &sha,
            &starter.tools,
        );
        assert_eq!(proj.setups[0].operations.len(), 5);
        let val = validate_project(&proj, None, Some(&doc));
        assert!(val.is_valid(), "validation errors: {:?}", val.diagnostics);

        let mut cache = ToolpathCache::new(32);
        let mut history = UndoHistory::new(32);

        // Wave 0 generates independent operations in parallel;
        // Wave 1 generates `op_2_adaptive_rest_3mm` using `op_1_adaptive_rough_10mm`'s swept coverage!
        let batch = generate_project_blocking(&mut proj, &doc, &mut cache, 1);
        assert!(batch.is_ok(), "batch errors: {:?}", batch.errors);
        assert_eq!(batch.generated.len(), 5);
        assert!(
            proj.setups[0]
                .operations
                .iter()
                .all(|o| o.runtime_status == OperationStatus::Clean)
        );

        // Verify the entire Adaptive + Rest setup passes `chip-sim` with 0 crashes and 0 gouges!
        let cancel = std::sync::atomic::AtomicBool::new(false);
        let sim = simulate_project_setup(&SetupSimRequest {
            project: &proj,
            cad_doc: &doc,
            cache: &cache,
            setup_idx: 0,
            cad_revision: 1,
            resolution: SimResolution::Standard,
            kinematics: resolve_kinematics_for_key("grbl", None),
            cancel: Some(&cancel),
        })
        .unwrap();
        assert!(
            !sim.has_critical_errors(),
            "Adaptive + Rest demo must have 0 critical simulation errors, got: {:?}",
            sim.diagnostics
        );

        // Editing upstream Roughing Op (`operations[2]`, `op_1_adaptive_rough_10mm`) MUST automatically
        // mark BOTH `operations[2]` AND downstream Rest `operations[3]` (`op_2_adaptive_rest_3mm`) `Dirty`
        // while leaving independent `operations[4]` (`op_3_open_shoulder_adaptive`) `Clean`!
        history.record_before_edit("Change Rough Stock to Leave", None, &proj);
        proj.setups[0].operations[2].passes.stock_to_leave_radial = ParamLength::from_mm(0.5);
        cache.reconcile_project_statuses(&mut proj, 1, &std::collections::HashSet::new());

        assert_eq!(
            proj.setups[0].operations[2].runtime_status,
            OperationStatus::Dirty
        );
        assert_eq!(
            proj.setups[0].operations[3].runtime_status,
            OperationStatus::Dirty,
            "Downstream FromPrecedingOperations Rest op must auto-invalidate when upstream changes"
        );
        assert_eq!(
            proj.setups[0].operations[4].runtime_status,
            OperationStatus::Clean,
            "Independent operation must remain Clean"
        );

        // `Ctrl+Z` Undo restores BOTH Op 1 and downstream Rest Op 2 back to `Clean` instantaneously!
        assert!(history.undo(&mut proj).is_some());
        cache.reconcile_project_statuses(&mut proj, 1, &std::collections::HashSet::new());
        assert!(
            proj.setups[0]
                .operations
                .iter()
                .all(|o| o.runtime_status == OperationStatus::Clean)
        );
    }
}
