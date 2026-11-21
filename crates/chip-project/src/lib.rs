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
    BundleError, ChipzExportSummary, ChipzPackOptions, MAX_CHIPZ_ENTRIES, MAX_CHIPZ_UNPACK_BYTES,
    UnpackedChipzBundle, create_private_temp_extract_dir, export_chipz_bundle,
    export_chipz_bundle_with_options, external_or_suspicious_assets_referenced, is_chipz_path,
    pack_chipz_bundle_bytes, pack_chipz_bundle_with_options, unpack_chipz_bundle,
    unpack_chipz_bytes,
};
pub use cache::{
    OperationInputHash, ToolpathCache, compute_operation_input_hash,
    resolve_upstream_rest_op_indices,
};
pub use demos::{
    build_adaptive_rest_demo_project, build_cam_torture_project,
    build_engraved_nameplate_demo_project, build_multi_feature_bracket_project,
    build_pure2d_router_panel_demo_project, build_relief_wave_mesh_project,
    build_revision_v1_demo_project, build_rotary_helical_flute_project,
    build_rotary_shaft_preview_project, build_rotary_wrapped_engraving_project,
    build_sculpted_mesh_dome_project, build_tutorial_3d_adaptive_waterline_project,
    build_tutorial_3d_surface_finishing_project, build_tutorial_16_tilted_step_project,
    build_tutorial_16_yup_stl_project, build_tutorial_adaptive_rest_project,
    build_tutorial_chamfer_trace_project, build_tutorial_contour_project,
    build_tutorial_drill_bore_project, build_tutorial_facing_project,
    build_tutorial_indexed_step_multiface_project, build_tutorial_pocket_project,
    build_tutorial_rotary_4axis_flowline_project, build_tutorial_rotary_axial_parallel_project,
    build_tutorial_vcarve_inlay_project, build_vcarve_sign_and_inlay_demo_project,
};
pub use dispatcher::{
    BatchGenerationReport, DispatcherEvent, JobDispatcher, generate_project_blocking,
};
pub use document::{
    ModelEntry, ProjectDocument, compute_sha256_hex, is_mesh_path, load_model_bytes_cad_doc,
    load_model_bytes_cad_doc_transformed, load_model_entry_cad_doc,
    load_model_entry_cad_doc_transformed, portable_model_path, resolve_model_path,
};
pub use history::UndoHistory;
pub use pipeline::{
    ReadySetupOperation, SetupPostRequest, SetupSimRequest, collect_ready_setup_operations,
    kinematics_for_post_profile, post_process_project_setup, resolve_kinematics_for_key,
    simulate_project_setup, simulate_project_setup_with_engine,
};
pub use reload::{
    CadReloadReport, OperationReloadOutcome, cad_binding_to_sketch_face,
    rebind_project_to_reloaded_cad, sketch_face_to_cad_binding,
};
pub use toml_io::ProjectIoError;
pub use validation::{
    DiagnosticSeverity, ProjectValidationReport, ValidationDiagnostic, validate_project,
    validate_project_with_machine,
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
            unit_override: None,
            transform: chip_cad::ModelTransform::IDENTITY,
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
        while dispatcher.is_busy() && start_wait.elapsed().as_secs() < 60 {
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

        // Undo (`Ctrl+Z`) -> restores Clean status + cached GeneratedToolpath without re-dispatching!
        let t_undo = Instant::now();
        history.undo(&mut proj).unwrap();
        cache.reconcile_project_statuses(&mut proj, 1, &HashSet::new());
        let undo_ms = t_undo.elapsed().as_millis();
        assert!(
            undo_ms < 500,
            "Undo + cache reconciliation should not trigger recomputation, took {undo_ms} ms"
        );
        assert!(!dispatcher.is_busy());
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
            unit_override: None,
            transform: chip_cad::ModelTransform::IDENTITY,
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
            unit_override: None,
            transform: chip_cad::ModelTransform::IDENTITY,
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

    #[test]
    fn pure2d_dxf_router_panel_migrates_v1_schema_packs_vectors_and_passes_sim_oracle() {
        use chip_cad::CadDocument;
        use chip_sim::SimResolution;
        use chip_sketch::sample_router_panel_dxf;

        // 1. Verify lossless v1 -> v2 schema migration
        let v1_toml = r#"
schema_version = 1
project_name = "Legacy V1 Job"
display_units = "mm"
models = []
tools = []
setups = []
"#;
        let migrated = ProjectDocument::from_toml_str(v1_toml).unwrap();
        assert_eq!(migrated.schema_version, 3);
        assert!(migrated.sketches.is_empty());

        // 2. Build Pure-2D Router Panel demo (`models = []`, 0 STEP files!)
        let starter = default_starter_tool_library();
        let dxf_bytes = sample_router_panel_dxf();
        let mut proj =
            build_pure2d_router_panel_demo_project(Some("panel.dxf".to_string()), &starter.tools);
        assert!(proj.models.is_empty());
        assert_eq!(proj.sketches.len(), 1);
        assert_eq!(proj.setups[0].operations.len(), 4);

        let empty_cad = CadDocument::empty();
        let val = validate_project(&proj, None, Some(&empty_cad));
        assert!(
            val.is_valid(),
            "pure-2D validation errors: {:?}",
            val.diagnostics
        );

        let mut cache = ToolpathCache::new(32);
        let batch = generate_project_blocking(&mut proj, &empty_cad, &mut cache, 1);
        assert!(batch.is_ok(), "pure-2D batch errors: {:?}", batch.errors);
        assert_eq!(batch.generated.len(), 4);

        // 3. Verify 0 crashes/gouges in `chip-sim` against the 6mm 2D sheet stock!
        let sim = simulate_project_setup(&SetupSimRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &empty_cad,
            cad_revision: 1,
            cache: &cache,
            kinematics: resolve_kinematics_for_key("grbl", None),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert!(
            !sim.has_critical_errors(),
            "Pure-2D DXF panel must pass heightmap verification with 0 critical errors, got: {:?}",
            sim.diagnostics
        );

        // 4. Verify `.chipz` packs `vectors/panel.dxf` deterministically and verifies SHA-256 on unpack!
        let (bundle_bytes, sum) = pack_chipz_bundle_with_options(&ChipzPackOptions {
            project: &proj,
            project_dir: None,
            fallback_model_bytes: &[],
            fallback_vector_bytes: &[("sketch_panel", &dxf_bytes)],
            extra_custom_posts: &[],
        })
        .unwrap();
        assert!(sum.model_files.is_empty());
        assert_eq!(sum.vector_files, vec!["vectors/panel.dxf".to_string()]);

        let tmp = std::env::temp_dir().join(format!("chipz_pure2d_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp);
        let unpacked = unpack_chipz_bytes(&bundle_bytes, &tmp).unwrap();
        assert!(unpacked.project.models.is_empty());
        assert_eq!(unpacked.project.sketches.len(), 1);
        assert!(tmp.join("vectors/panel.dxf").is_file());
        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn associative_planar_face_sketch_rebinds_across_step_v1_to_v2_and_checks_setup_z_alignment() {
        use chip_cad::{TopologyGraph, sample_revised_part_pair_step};
        use chip_cam::{
            CamError, ChamferTraceMode, StrategyParams, extract_selection_features_with_sketches,
        };
        use chip_math::ParamLength;
        use chip_sim::SimResolution;
        use chip_sketch::{
            FaceSketchOriginMode, HersheyFontKind, SketchAnchor, SketchEntity, SketchEntityEntry,
            SketchFontSource, SketchLayer, SketchTextSpec, SlotLengthMode,
        };

        let (step_v1, step_v2) = sample_revised_part_pair_step().unwrap();
        let doc_v1 = MonstertruckKernel
            .load_step_bytes("rev_v1.step", step_v1.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let doc_v2 = MonstertruckKernel
            .load_step_bytes("rev_v2.step", step_v2.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let topo_v2 = TopologyGraph::build(&doc_v2);

        let starter = default_starter_tool_library();
        let mut proj = ProjectDocument::new("Associative PlanarFace Sketch Test");
        proj.models.push(ModelEntry {
            id: "model_1".to_string(),
            path: "rev_v1.step".to_string(),
            sha256: compute_sha256_hex(step_v1.as_bytes()),
            unit_override: None,
            transform: chip_cad::ModelTransform::IDENTITY,
        });
        let ch_tool = starter
            .tools
            .iter()
            .find(|t| matches!(t.geometry, chip_tools::CutterGeometry::ChamferMill { .. }))
            .unwrap();
        proj.ensure_tool_embedded(ch_tool);

        // Anchor a 2D Sketch onto the top planar STEP face (`+Z`, `z = 20.0 mm`)
        let top_face_v1 = doc_v1
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { origin_mm, normal }
                    if normal.z > 0.99 && (origin_mm.z - 20.0).abs() < 0.05
                )
            })
            .unwrap();
        let top_bind_v1 = cad_binding_to_sketch_face(&doc_v1.bind_face(top_face_v1.id).unwrap());

        let mut sk = SketchLayer::new("sk_top_face", "Top Face Serial & Slot", "setup_1");
        sk.anchor = SketchAnchor::PlanarFace {
            setup_id: "setup_1".to_string(),
            face_binding: top_bind_v1,
            origin_mode: FaceSketchOriginMode::FaceCentroid,
            x_axis_clock_deg: 0.0,
            z_offset: ParamLength::from_mm(0.0),
        };
        sk.entities.push(SketchEntityEntry {
            id: "slot_1".to_string(),
            name: "Keyway Slot".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::Slot {
                mode: SlotLengthMode::CenterToCenter,
                x: ParamLength::from_mm(0.0),
                y: ParamLength::from_mm(-18.0),
                length: ParamLength::from_mm(16.0),
                width: ParamLength::from_mm(6.0),
                angle_deg: 0.0,
            },
        });
        sk.entities.push(SketchEntityEntry {
            id: "ser_1".to_string(),
            name: "Part Number".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::Text(SketchTextSpec {
                content: "REV-A".to_string(),
                font: SketchFontSource::Hershey {
                    font: HersheyFontKind::Mono,
                },
                cap_height: ParamLength::from_mm(4.5),
                y: ParamLength::from_mm(18.0),
                ..SketchTextSpec::default()
            }),
        });
        sk.rebuild_regions(None);
        proj.sketches.push(sk);

        let mut op_trace = Operation::new(
            "op_face_scribe",
            "Top Face Scribe",
            OperationKind::ChamferTrace,
        );
        op_trace.tool_id = Some(ch_tool.id.clone());
        op_trace.preset_id = ch_tool.presets.first().map(|p| p.id.clone());
        op_trace.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -0.3);
        op_trace
            .selections
            .push(GeometrySelectionItem::SketchEntity {
                sketch_id: "sk_top_face".to_string(),
                entity_id: "ser_1".to_string(),
                reverse: false,
                side_flip: false,
            });
        if let StrategyParams::ChamferTrace(p) = op_trace.strategy_mut() {
            p.mode = ChamferTraceMode::CurveTrace;
        }
        proj.setups[0].operations.push(op_trace);

        let mut cache = ToolpathCache::new(16);
        let batch_v1 = generate_project_blocking(&mut proj, &doc_v1, &mut cache, 1);
        assert!(batch_v1.is_ok(), "v1 errors: {:?}", batch_v1.errors);

        // Reload `v2` STEP -> associative `PlanarFace` sketch re-binds cleanly and regenerates with 0 sim crashes!
        let reload_rep = rebind_project_to_reloaded_cad(
            &mut proj,
            0,
            &doc_v2,
            &topo_v2,
            &compute_sha256_hex(step_v2.as_bytes()),
            Some("rev_v2.step".to_string()),
        );
        assert_eq!(reload_rep.needs_repick_count, 0);
        assert!(reload_rep.setup_wcs_warnings.is_empty());

        let batch_v2 = generate_project_blocking(&mut proj, &doc_v2, &mut cache, 2);
        assert!(batch_v2.is_ok(), "v2 errors: {:?}", batch_v2.errors);

        let sim_v2 = simulate_project_setup(&SetupSimRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &doc_v2,
            cad_revision: 2,
            cache: &cache,
            kinematics: resolve_kinematics_for_key("grbl", None),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert!(!sim_v2.has_critical_errors());

        // Also verify a side-face `PlanarFace` sketch (`normal ≈ +X`) is rejected when selected in a `+Z` Setup!
        let side_face = doc_v2
            .faces
            .iter()
            .find(|f| matches!(f.surface, SurfaceKind::Plane { normal, .. } if normal.x > 0.99))
            .unwrap();
        let mut side_sk = SketchLayer::new("sk_side", "Side Wall Sketch", "setup_1");
        side_sk.anchor = SketchAnchor::PlanarFace {
            setup_id: "setup_1".to_string(),
            face_binding: cad_binding_to_sketch_face(&doc_v2.bind_face(side_face.id).unwrap()),
            origin_mode: FaceSketchOriginMode::FaceCentroid,
            x_axis_clock_deg: 0.0,
            z_offset: ParamLength::from_mm(0.0),
        };
        side_sk.entities.push(SketchEntityEntry {
            id: "c_side".to_string(),
            name: "Side Hole".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::new_circle(
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(6.0),
            ),
        });
        let rs_top = proj.setups[0]
            .resolve_with_sketches(&doc_v2, &[side_sk.clone()], proj.display_units)
            .unwrap();
        let err = extract_selection_features_with_sketches(
            &doc_v2,
            &[side_sk],
            &rs_top,
            &[GeometrySelectionItem::SketchEntity {
                sketch_id: "sk_side".to_string(),
                entity_id: "c_side".to_string(),
                reverse: false,
                side_flip: false,
            }],
        )
        .unwrap_err();
        assert!(matches!(err, CamError::SketchNotAlignedWithSetupZ { .. }));
    }

    #[test]
    fn engraved_nameplate_demo_passes_sim_and_chipz_bundles_custom_ttf_fonts() {
        use chip_cad::CadDocument;
        use chip_sim::SimResolution;
        use chip_sketch::{BUILTIN_DEJAVU_SANS_BOLD_TTF, SketchEntity, SketchFontSource};

        let starter = default_starter_tool_library();
        let mut proj = build_engraved_nameplate_demo_project(&starter.tools);
        assert!(proj.models.is_empty());
        assert_eq!(proj.sketches.len(), 1);
        assert_eq!(proj.setups[0].operations.len(), 5);

        let empty_cad = CadDocument::empty();
        let mut cache = ToolpathCache::new(32);
        let batch = generate_project_blocking(&mut proj, &empty_cad, &mut cache, 1);
        assert!(batch.is_ok(), "nameplate batch errors: {:?}", batch.errors);
        assert_eq!(batch.generated.len(), 5);

        let sim = simulate_project_setup(&SetupSimRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &empty_cad,
            cad_revision: 1,
            cache: &cache,
            kinematics: resolve_kinematics_for_key("grbl", None),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert!(
            !sim.has_critical_errors(),
            "Nameplate + Text demo must pass heightfield sim with 0 crashes/gouges, got: {:?}",
            sim.diagnostics
        );

        // Verify custom `.ttf` font asset packing & SHA-256 verification inside `.chipz` (`fonts/*`)
        let tmp_src = std::env::temp_dir().join(format!("chipz_font_src_{}", std::process::id()));
        let tmp_dst = std::env::temp_dir().join(format!("chipz_font_dst_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp_src);
        let _ = std::fs::remove_dir_all(&tmp_dst);
        std::fs::create_dir_all(&tmp_src).unwrap();
        let custom_ttf_path = tmp_src.join("CustomSansBold.ttf");
        std::fs::write(&custom_ttf_path, BUILTIN_DEJAVU_SANS_BOLD_TTF).unwrap();

        if let Some(ent) = proj.sketches[0]
            .entities
            .iter_mut()
            .find(|e| e.id == "ent_title_ttf")
            && let SketchEntity::Text(txt) = &mut ent.entity
        {
            txt.font = SketchFontSource::CustomFile {
                path: "CustomSansBold.ttf".to_string(),
                sha256: String::new(),
                face_index: 0,
            };
        }

        let (bundle_bytes, sum) = pack_chipz_bundle_bytes(&proj, Some(&tmp_src), &[], &[]).unwrap();
        assert_eq!(sum.font_files, vec!["fonts/CustomSansBold.ttf".to_string()]);

        let unpacked = unpack_chipz_bytes(&bundle_bytes, &tmp_dst).unwrap();
        assert!(tmp_dst.join("fonts/CustomSansBold.ttf").is_file());
        assert!(!unpacked.project.sketches[0].regions.is_empty());

        let _ = std::fs::remove_dir_all(&tmp_src);
        let _ = std::fs::remove_dir_all(&tmp_dst);
    }

    #[test]
    fn vcarve_sign_and_inlay_demo_passes_sim_and_posts_atc_and_grbl_split_files() {
        use chip_cad::CadDocument;
        use chip_post::resolve_post_profile;
        use chip_sim::SimResolution;

        let starter = default_starter_tool_library();
        let mut proj = build_vcarve_sign_and_inlay_demo_project(&starter.tools);
        assert_eq!(proj.setups.len(), 2);

        let empty_cad = CadDocument::empty();
        let val = validate_project(&proj, None, Some(&empty_cad));
        assert!(
            val.is_valid(),
            "expected valid vcarve demo project, got: {:?}",
            val.diagnostics
        );

        let mut cache = ToolpathCache::new(32);
        let batch = generate_project_blocking(&mut proj, &empty_cad, &mut cache, 1);
        assert!(batch.is_ok(), "vcarve batch errors: {:?}", batch.errors);
        assert_eq!(batch.generated.len(), 4);

        // Setup 1 (`Op 1` 2-stage + `Op 2` medial + `Op 3` contour = 4 ReadySetupOperation items!)
        let ready_s0 = collect_ready_setup_operations(&proj, 0, &cache, 1);
        assert_eq!(ready_s0.len(), 4);
        assert_eq!(ready_s0[0].tool.tool_number, 1); // Stage 1 Ø6mm Flat Endmill
        assert_eq!(ready_s0[1].tool.tool_number, 8); // Stage 2 60° V-Bit

        // Both Setup 1 (Female Sign Plaque) and Setup 2 (Male V-Inlay Plug) must pass 3D sim cleanly!
        for setup_idx in [0, 1] {
            let sim = simulate_project_setup(&SetupSimRequest {
                project: &proj,
                setup_idx,
                cad_doc: &empty_cad,
                cad_revision: 1,
                cache: &cache,
                kinematics: resolve_kinematics_for_key("linuxcnc", None),
                resolution: SimResolution::Standard,
                cancel: None,
            })
            .unwrap();
            assert!(
                !sim.has_critical_errors(),
                "V-Carve demo setup {setup_idx} must pass heightfield sim with 0 crashes/gouges, got: {:?}",
                sim.diagnostics
            );
        }

        // Verify ATC (`linuxcnc`) emits 1 unified program with `T1 M6` + `T8 M6`!
        let linuxcnc = resolve_post_profile("linuxcnc", None).unwrap();
        let atc_files = post_process_project_setup(&SetupPostRequest {
            project: &proj,
            setup_idx: 1,
            cad_doc: &empty_cad,
            cad_revision: 1,
            cache: &cache,
            post_profile: &linuxcnc,
            project_label: &proj.project_name,
            program_stem: "inlay_plug",
        })
        .unwrap();
        assert_eq!(atc_files.len(), 1);
        assert!(atc_files[0].content.contains("T1 M6"));
        assert!(atc_files[0].content.contains("T8 M6"));

        // Verify GRBL (`split_files_by_tool = true`) splits the single Two-Tool V-Carve op into 2 files!
        let grbl = resolve_post_profile("grbl", None).unwrap();
        let grbl_files = post_process_project_setup(&SetupPostRequest {
            project: &proj,
            setup_idx: 1,
            cad_doc: &empty_cad,
            cad_revision: 1,
            cache: &cache,
            post_profile: &grbl,
            project_label: &proj.project_name,
            program_stem: "inlay_plug",
        })
        .unwrap();
        assert_eq!(
            grbl_files.len(),
            2,
            "GRBL must split Two-Tool V-Carve into Stage 1 (T1) and Stage 2 (T8) files!"
        );
        assert!(grbl_files[0].filename.contains("T1"));
        assert!(grbl_files[1].filename.contains("T8"));
    }

    #[test]
    fn minimal_tutorial_projects_pass_heightmap_sim_with_zero_crashes() {
        use chip_cad::{
            tutorial_adaptive_pocket_step, tutorial_chamfer_engrave_block_step,
            tutorial_contour_plate_step, tutorial_drill_bore_plate_step,
            tutorial_facing_billet_step, tutorial_pocket_block_step,
        };
        use chip_post::resolve_post_profile;
        use chip_sim::SimResolution;

        let kernel = MonstertruckKernel;
        let starter = default_starter_tool_library();
        let grbl = resolve_post_profile("grbl", None).unwrap();
        let linuxcnc = resolve_post_profile("linuxcnc", None).unwrap();

        let cases: Vec<(&str, String, usize)> = vec![
            ("01-facing", tutorial_facing_billet_step().unwrap(), 1),
            ("02-contour", tutorial_contour_plate_step().unwrap(), 1),
            ("03-pocket", tutorial_pocket_block_step().unwrap(), 2),
            (
                "04-drill-bore",
                tutorial_drill_bore_plate_step().unwrap(),
                2,
            ),
            (
                "05-adaptive-rest",
                tutorial_adaptive_pocket_step().unwrap(),
                2,
            ),
            (
                "06-chamfer-trace",
                tutorial_chamfer_engrave_block_step().unwrap(),
                3,
            ),
        ];

        for (slug, step, expected_ops) in cases {
            let doc = kernel
                .load_step_bytes(slug, step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
                .unwrap();
            let sha = compute_sha256_hex(step.as_bytes());
            let mut proj = match slug {
                "01-facing" => {
                    build_tutorial_facing_project(slug.to_string(), &sha, &starter.tools, false)
                }
                "02-contour" => build_tutorial_contour_project(
                    &doc,
                    slug.to_string(),
                    &sha,
                    &starter.tools,
                    false,
                ),
                "03-pocket" => build_tutorial_pocket_project(
                    &doc,
                    slug.to_string(),
                    &sha,
                    &starter.tools,
                    false,
                ),
                "04-drill-bore" => build_tutorial_drill_bore_project(
                    &doc,
                    slug.to_string(),
                    &sha,
                    &starter.tools,
                    false,
                ),
                "05-adaptive-rest" => {
                    build_tutorial_adaptive_rest_project(&doc, slug.to_string(), &sha, false)
                }
                "06-chamfer-trace" => build_tutorial_chamfer_trace_project(
                    &doc,
                    slug.to_string(),
                    &sha,
                    &starter.tools,
                    false,
                ),
                _ => unreachable!(),
            };
            assert_eq!(proj.setups[0].operations.len(), expected_ops);

            let mut cache = ToolpathCache::new(32);
            let report = generate_project_blocking(&mut proj, &doc, &mut cache, 1);
            assert!(
                report.errors.is_empty(),
                "tutorial '{slug}' generation failed: {:?}",
                report.errors
            );

            let sim = simulate_project_setup(&SetupSimRequest {
                project: &proj,
                setup_idx: 0,
                cad_doc: &doc,
                cad_revision: 1,
                cache: &cache,
                kinematics: resolve_kinematics_for_key("linuxcnc", None),
                resolution: SimResolution::Standard,
                cancel: None,
            })
            .unwrap();
            assert!(
                !sim.has_critical_errors(),
                "tutorial '{slug}' must pass heightmap sim with 0 crashes/gouges, got: {:?}",
                sim.diagnostics
            );

            for profile in [&grbl, &linuxcnc] {
                let files = post_process_project_setup(&SetupPostRequest {
                    project: &proj,
                    setup_idx: 0,
                    cad_doc: &doc,
                    cad_revision: 1,
                    cache: &cache,
                    post_profile: profile,
                    project_label: &proj.project_name,
                    program_stem: slug,
                })
                .unwrap();
                assert!(!files.is_empty());
            }
        }

        // 07 V-Carve (pure medial-axis + two-stage flat clearance & mated Male V-Inlay plug)
        let empty_cad = chip_cad::CadDocument::empty();
        for medial_only in [true, false] {
            let mut vc_proj = build_tutorial_vcarve_inlay_project(&starter.tools, medial_only);
            let mut cache = ToolpathCache::new(32);
            let report = generate_project_blocking(&mut vc_proj, &empty_cad, &mut cache, 1);
            assert!(
                report.errors.is_empty(),
                "tutorial 07 (medial_only={medial_only}) generation failed: {:?}",
                report.errors
            );
            for setup_idx in 0..vc_proj.setups.len() {
                let sim = simulate_project_setup(&SetupSimRequest {
                    project: &vc_proj,
                    setup_idx,
                    cad_doc: &empty_cad,
                    cad_revision: 1,
                    cache: &cache,
                    kinematics: resolve_kinematics_for_key("linuxcnc", None),
                    resolution: SimResolution::Standard,
                    cancel: None,
                })
                .unwrap();
                assert!(
                    !sim.has_critical_errors(),
                    "tutorial 07 (medial_only={medial_only}, setup={setup_idx}) sim errors: {:?}",
                    sim.diagnostics
                );
                let grbl_files = post_process_project_setup(&SetupPostRequest {
                    project: &vc_proj,
                    setup_idx,
                    cad_doc: &empty_cad,
                    cad_revision: 1,
                    cache: &cache,
                    post_profile: &grbl,
                    project_label: &vc_proj.project_name,
                    program_stem: "07_vcarve",
                })
                .unwrap();
                if medial_only {
                    assert_eq!(grbl_files.len(), 1);
                } else {
                    assert_eq!(
                        grbl_files.len(),
                        2,
                        "GRBL must split Two-Stage V-Carve into Stage 1 clearing + Stage 2 V-bit files"
                    );
                }
            }
        }
    }

    #[test]
    fn schema_v3_migrates_v1_and_v2_and_bundles_stl_obj_3mf_meshes() {
        use chip_cad::{
            MeshUnitKind, sample_box_3mf_bytes, sample_relief_wave_obj_text,
            sample_sculpted_dome_stl_bytes,
        };

        // 1. Lossless v1 -> v3 and v2 -> v3 migration
        for ver in [1u32, 2u32] {
            let legacy_toml = format!(
                r#"
schema_version = {ver}
project_name = "Legacy v{ver}"
display_units = "mm"
tools = []
setups = []

[[models]]
id = "model_1"
path = "part.step"
sha256 = "abc"
"#
            );
            let parsed = ProjectDocument::from_toml_str(&legacy_toml).unwrap();
            assert_eq!(
                parsed.schema_version,
                ProjectDocument::CURRENT_SCHEMA_VERSION
            );
            assert_eq!(parsed.models[0].unit_override, None);
            // STEP ModelEntry must omit `unit_override` from serialized TOML
            let reserialized = parsed.to_toml_string().unwrap();
            assert!(!reserialized.contains("unit_override"));
        }

        // 2. Mesh ModelEntry with explicit `unit_override = "inch"` serializes cleanly
        let stl_bytes = sample_sculpted_dome_stl_bytes();
        let obj_bytes = sample_relief_wave_obj_text().into_bytes();
        let mf_bytes = sample_box_3mf_bytes(MeshUnitKind::Inch, None);

        let mut proj = ProjectDocument::new("Phase 5 Mesh Bundle");
        proj.models = vec![
            ModelEntry {
                id: "dome_stl".to_string(),
                path: "sculpted_dome.stl".to_string(),
                sha256: compute_sha256_hex(&stl_bytes),
                unit_override: Some(MeshUnitKind::Millimeter),
                transform: chip_cad::ModelTransform::IDENTITY,
            },
            ModelEntry {
                id: "wave_obj".to_string(),
                path: "relief_wave.obj".to_string(),
                sha256: compute_sha256_hex(&obj_bytes),
                unit_override: None,
                transform: chip_cad::ModelTransform::IDENTITY,
            },
            ModelEntry {
                id: "box_3mf".to_string(),
                path: "box.3mf".to_string(),
                sha256: compute_sha256_hex(&mf_bytes),
                unit_override: None,
                transform: chip_cad::ModelTransform::IDENTITY,
            },
        ];

        let toml_out = proj.to_toml_string().unwrap();
        assert!(toml_out.contains("unit_override = \"millimeter\""));
        let reparsed = ProjectDocument::from_toml_str(&toml_out).unwrap();
        assert_eq!(
            reparsed.models[0].unit_override,
            Some(MeshUnitKind::Millimeter)
        );
        assert_eq!(reparsed.models[1].unit_override, None);

        // 3. `.chipz` bundle packs all 3 mesh formats under `models/*` and verifies SHA-256 on unpack!
        let (chipz_bytes, summary) = pack_chipz_bundle_bytes(
            &proj,
            None,
            &[
                ("dome_stl", stl_bytes.as_slice()),
                ("wave_obj", obj_bytes.as_slice()),
                ("box_3mf", mf_bytes.as_slice()),
            ],
            &[],
        )
        .unwrap();
        assert_eq!(summary.model_files.len(), 3);

        let tmp = std::env::temp_dir().join(format!("chipz_mesh_v3_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp);
        let unpacked = unpack_chipz_bytes(&chipz_bytes, &tmp).unwrap();
        assert_eq!(unpacked.project.schema_version, 3);
        for entry in &unpacked.project.models {
            let p = tmp.join(&entry.path);
            assert!(is_mesh_path(&p));
            let doc = load_model_entry_cad_doc(&p, entry.unit_override).unwrap();
            assert!(doc.has_3d_geometry());
            assert_eq!(doc.meshes.len(), 1);
            assert!(doc.meshes[0].diagnostics.is_watertight);
        }
        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn sculpted_mesh_dome_3d_adaptive_and_waterline_roundtrips_simulates_and_posts_gcode() {
        use chip_cad::{MeshImportOptions, load_mesh_cad_document, sample_sculpted_dome_stl_bytes};
        use chip_post::builtin_post_profiles;
        use chip_sim::SimResolution;

        let stl_bytes = sample_sculpted_dome_stl_bytes();
        let cad_doc = load_mesh_cad_document(
            "sculpted_dome.stl",
            &stl_bytes,
            &MeshImportOptions::default(),
        )
        .unwrap();
        let sha = compute_sha256_hex(&stl_bytes);
        let starter = default_starter_tool_library();

        let mut proj =
            build_sculpted_mesh_dome_project("sculpted_dome.stl".to_string(), &sha, &starter.tools);
        assert_eq!(proj.schema_version, 3);
        assert_eq!(proj.setups[0].operations.len(), 3);

        // 1. Deterministic Schema v3 TOML round-trip
        let toml_1 = proj.to_toml_string().unwrap();
        let reparsed = ProjectDocument::from_toml_str(&toml_1).unwrap();
        assert_eq!(toml_1, reparsed.to_toml_string().unwrap());

        // 2. Project validation passes with 0 errors
        let val = validate_project(&proj, None, Some(&cad_doc));
        assert!(
            val.is_valid(),
            "dome validation errors: {:?}",
            val.diagnostics
        );

        // 3. Parallel generation & 5-rule heightmap simulation pass with 0 critical errors
        let mut cache = ToolpathCache::new(16);
        let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
        assert!(batch.is_ok(), "dome generation errors: {:?}", batch.errors);
        assert_eq!(batch.generated.len(), 3);

        let sim = simulate_project_setup(&SetupSimRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            kinematics: resolve_kinematics_for_key("grbl", None),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert!(
            !sim.has_critical_errors(),
            "Sculpted Dome sample must pass simulation with 0 critical errors, got: {:#?}",
            sim.diagnostics
        );

        // 4. Both GRBL (3 split tool files: T1 Flat + T4 Ø6mm Ball + T11 Ø3mm Ball) and LinuxCNC (1 ATC file) post cleanly
        let profiles = builtin_post_profiles();
        let grbl = profiles.iter().find(|p| p.id == "grbl").unwrap();
        let linuxcnc = profiles.iter().find(|p| p.id == "linuxcnc").unwrap();

        let grbl_files = post_process_project_setup(&SetupPostRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            post_profile: grbl,
            project_label: &proj.project_name,
            program_stem: "sculpted_dome",
        })
        .unwrap();
        assert_eq!(
            grbl_files.len(),
            3,
            "GRBL must split T1 Adaptive3D, T4 Waterline3D, and T11 Pencil3D into 3 separate .nc files"
        );

        let lcnc_files = post_process_project_setup(&SetupPostRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            post_profile: linuxcnc,
            project_label: &proj.project_name,
            program_stem: "sculpted_dome",
        })
        .unwrap();
        assert_eq!(lcnc_files.len(), 1);
        assert!(lcnc_files[0].content.contains("M6"));
    }

    #[test]
    fn relief_wave_mesh_sample_round_trips_simulates_and_posts_cleanly() {
        use chip_cad::{MeshImportOptions, load_mesh_cad_document, sample_relief_wave_obj_text};
        use chip_post::builtin_post_profiles;
        use chip_sim::SimResolution;

        let obj_text = sample_relief_wave_obj_text();
        let obj_bytes = obj_text.as_bytes();
        let cad_doc =
            load_mesh_cad_document("relief_wave.obj", obj_bytes, &MeshImportOptions::default())
                .unwrap();
        let sha = compute_sha256_hex(obj_bytes);
        let starter = default_starter_tool_library();

        let mut proj =
            build_relief_wave_mesh_project("relief_wave.obj".to_string(), &sha, &starter.tools);
        assert_eq!(proj.schema_version, 3);
        assert_eq!(proj.setups[0].operations.len(), 3);

        // 1. Deterministic Schema v3 TOML round-trip
        let toml_1 = proj.to_toml_string().unwrap();
        let reparsed = ProjectDocument::from_toml_str(&toml_1).unwrap();
        assert_eq!(toml_1, reparsed.to_toml_string().unwrap());

        // 2. Project validation passes with 0 errors
        let val = validate_project(&proj, None, Some(&cad_doc));
        assert!(
            val.is_valid(),
            "wave validation errors: {:?}",
            val.diagnostics
        );

        // 3. Generation & 5-rule heightmap simulation pass with 0 critical errors
        let mut cache = ToolpathCache::new(16);
        let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
        assert!(batch.is_ok(), "wave generation errors: {:?}", batch.errors);
        assert_eq!(batch.generated.len(), 3);

        let sim = simulate_project_setup(&SetupSimRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            kinematics: resolve_kinematics_for_key("grbl", None),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert!(
            !sim.has_critical_errors(),
            "Relief Wave sample must pass simulation with 0 critical errors, got: {:#?}",
            sim.diagnostics
        );

        // 4. Both GRBL (2 split tool files: T1 Flat + T4 Ball) and LinuxCNC post cleanly
        let profiles = builtin_post_profiles();
        let grbl = profiles.iter().find(|p| p.id == "grbl").unwrap();
        let linuxcnc = profiles.iter().find(|p| p.id == "linuxcnc").unwrap();

        let grbl_files = post_process_project_setup(&SetupPostRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            post_profile: grbl,
            project_label: &proj.project_name,
            program_stem: "relief_wave",
        })
        .unwrap();
        assert_eq!(
            grbl_files.len(),
            2,
            "GRBL must split T1 Adaptive3D and T4 Parallel3D/Scallop3D into 2 separate .nc files"
        );

        let lcnc_files = post_process_project_setup(&SetupPostRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            post_profile: linuxcnc,
            project_label: &proj.project_name,
            program_stem: "relief_wave",
        })
        .unwrap();
        assert_eq!(lcnc_files.len(), 1);
        assert!(lcnc_files[0].content.contains("M6"));
    }

    #[test]
    fn tutorial_3d_surfacing_projects_roundtrip_simulate_crash_free_and_post() {
        use chip_cad::{
            MeshImportOptions, load_mesh_cad_document, sample_relief_wave_obj_text,
            sample_sculpted_dome_stl_bytes,
        };
        use chip_post::builtin_post_profiles;
        use chip_sim::SimResolution;

        let starter = default_starter_tool_library();
        let profiles = builtin_post_profiles();
        let grbl = profiles.iter().find(|p| p.id == "grbl").unwrap();
        let linuxcnc = profiles.iter().find(|p| p.id == "linuxcnc").unwrap();

        // ── Tutorial 08: 3D Adaptive Roughing & Waterline / Z-Level (08-sculpted-dome.stl) ──
        let dome_bytes = sample_sculpted_dome_stl_bytes();
        let dome_doc = load_mesh_cad_document(
            "08-sculpted-dome.stl",
            &dome_bytes,
            &MeshImportOptions::default(),
        )
        .unwrap();
        let dome_sha = compute_sha256_hex(&dome_bytes);
        let mut proj_08 = build_tutorial_3d_adaptive_waterline_project(
            "08-sculpted-dome.stl".to_string(),
            &dome_sha,
            &starter.tools,
        );
        assert_eq!(proj_08.schema_version, 3);
        assert_eq!(proj_08.sketches.len(), 1);
        assert_eq!(proj_08.setups[0].operations.len(), 2);

        let toml_08_a = proj_08.to_toml_string().unwrap();
        let toml_08_b = ProjectDocument::from_toml_str(&toml_08_a)
            .unwrap()
            .to_toml_string()
            .unwrap();
        assert_eq!(toml_08_a, toml_08_b);

        let val_08 = validate_project(&proj_08, None, Some(&dome_doc));
        assert!(
            val_08.is_valid(),
            "Tutorial 08 validation errors: {:?}",
            val_08.diagnostics
        );

        let mut cache_08 = ToolpathCache::new(16);
        let batch_08 = generate_project_blocking(&mut proj_08, &dome_doc, &mut cache_08, 1);
        assert!(
            batch_08.is_ok(),
            "Tutorial 08 generation errors: {:?}",
            batch_08.errors
        );
        assert_eq!(batch_08.generated.len(), 2);

        let sim_08 = simulate_project_setup(&SetupSimRequest {
            project: &proj_08,
            setup_idx: 0,
            cad_doc: &dome_doc,
            cad_revision: 1,
            cache: &cache_08,
            kinematics: resolve_kinematics_for_key("grbl", None),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert_eq!(
            sim_08.critical_error_count(),
            0,
            "Tutorial 08 must simulate crash-free with 0 critical errors, got: {:#?}",
            sim_08.diagnostics
        );

        let grbl_08 = post_process_project_setup(&SetupPostRequest {
            project: &proj_08,
            setup_idx: 0,
            cad_doc: &dome_doc,
            cad_revision: 1,
            cache: &cache_08,
            post_profile: grbl,
            project_label: &proj_08.project_name,
            program_stem: "08_adaptive_waterline",
        })
        .unwrap();
        assert_eq!(
            grbl_08.len(),
            2,
            "GRBL must split T1 Adaptive3D and T4 Waterline3D into 2 separate .nc files"
        );

        let lcnc_08 = post_process_project_setup(&SetupPostRequest {
            project: &proj_08,
            setup_idx: 0,
            cad_doc: &dome_doc,
            cad_revision: 1,
            cache: &cache_08,
            post_profile: linuxcnc,
            project_label: &proj_08.project_name,
            program_stem: "08_adaptive_waterline",
        })
        .unwrap();
        assert_eq!(lcnc_08.len(), 1);
        assert!(lcnc_08[0].content.contains("M6"));

        // ── Tutorial 09: 3D Surface Finishing (09-relief-wave.obj) ──
        let wave_text = sample_relief_wave_obj_text();
        let wave_bytes = wave_text.as_bytes();
        let wave_doc = load_mesh_cad_document(
            "09-relief-wave.obj",
            wave_bytes,
            &MeshImportOptions::default(),
        )
        .unwrap();
        let wave_sha = compute_sha256_hex(wave_bytes);
        let mut proj_09 = build_tutorial_3d_surface_finishing_project(
            "09-relief-wave.obj".to_string(),
            &wave_sha,
            &starter.tools,
        );
        assert_eq!(proj_09.schema_version, 3);
        assert_eq!(proj_09.setups[0].operations.len(), 3);

        let toml_09_a = proj_09.to_toml_string().unwrap();
        let toml_09_b = ProjectDocument::from_toml_str(&toml_09_a)
            .unwrap()
            .to_toml_string()
            .unwrap();
        assert_eq!(toml_09_a, toml_09_b);

        let val_09 = validate_project(&proj_09, None, Some(&wave_doc));
        assert!(
            val_09.is_valid(),
            "Tutorial 09 validation errors: {:?}",
            val_09.diagnostics
        );

        let mut cache_09 = ToolpathCache::new(16);
        let batch_09 = generate_project_blocking(&mut proj_09, &wave_doc, &mut cache_09, 1);
        assert!(
            batch_09.is_ok(),
            "Tutorial 09 generation errors: {:?}",
            batch_09.errors
        );
        assert_eq!(batch_09.generated.len(), 3);

        let sim_09 = simulate_project_setup(&SetupSimRequest {
            project: &proj_09,
            setup_idx: 0,
            cad_doc: &wave_doc,
            cad_revision: 1,
            cache: &cache_09,
            kinematics: resolve_kinematics_for_key("grbl", None),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert_eq!(
            sim_09.critical_error_count(),
            0,
            "Tutorial 09 must simulate crash-free with 0 critical errors, got: {:#?}",
            sim_09.diagnostics
        );

        let grbl_09 = post_process_project_setup(&SetupPostRequest {
            project: &proj_09,
            setup_idx: 0,
            cad_doc: &wave_doc,
            cad_revision: 1,
            cache: &cache_09,
            post_profile: grbl,
            project_label: &proj_09.project_name,
            program_stem: "09_surface_finishing",
        })
        .unwrap();
        assert_eq!(
            grbl_09.len(),
            2,
            "GRBL must split T1 Adaptive3D and T4 Parallel3D/Scallop3D into 2 separate .nc files"
        );

        let lcnc_09 = post_process_project_setup(&SetupPostRequest {
            project: &proj_09,
            setup_idx: 0,
            cad_doc: &wave_doc,
            cad_revision: 1,
            cache: &cache_09,
            post_profile: linuxcnc,
            project_label: &proj_09.project_name,
            program_stem: "09_surface_finishing",
        })
        .unwrap();
        assert_eq!(lcnc_09.len(), 1);
        assert!(lcnc_09[0].content.contains("M6"));
    }

    #[test]
    fn rotary_setup_roundtrips_toml_omits_when_disabled_and_validates_post_compatibility() {
        use chip_cad::{
            CadDocument, MeshImportOptions, load_mesh_cad_document, sample_rotary_shaft_stl_bytes,
        };
        use chip_post::builtin_post_profiles;
        use chip_sim::SimResolution;

        let stl_bytes = sample_rotary_shaft_stl_bytes();
        let cad_doc = load_mesh_cad_document(
            "rotary_shaft.stl",
            &stl_bytes,
            &MeshImportOptions::default(),
        )
        .unwrap();
        assert!(cad_doc.meshes[0].diagnostics.is_watertight);
        let sha = compute_sha256_hex(&stl_bytes);
        let starter = default_starter_tool_library();

        // 1. 3-axis default project omits `[setups.rotary]` completely (zero diff for existing files)
        let plain = ProjectDocument::new("Plain 3-Axis");
        let plain_toml = plain.to_toml_string().unwrap();
        assert!(
            !plain_toml.contains("rotary"),
            "Disabled rotary must be omitted from .chip.toml:\n{plain_toml}"
        );

        // 2. Rotary Shaft preview project round-trips deterministically & validates cleanly with `linuxcnc_4axis`
        let mut proj = build_rotary_shaft_preview_project(
            "rotary_shaft.stl".to_string(),
            &sha,
            &starter.tools,
        );
        let toml_1 = proj.to_toml_string().unwrap();
        assert!(toml_1.contains("simultaneous4_axis"));
        assert!(toml_1.contains("rotary_2linear"));
        assert!(toml_1.contains("indexed3_plus1"));
        let reparsed = ProjectDocument::from_toml_str(&toml_1).unwrap();
        assert_eq!(toml_1, reparsed.to_toml_string().unwrap());

        let val_ok = validate_project(&proj, None, Some(&cad_doc));
        assert!(
            val_ok.is_valid() && val_ok.warning_count() == 0,
            "Rotary shaft with linuxcnc_4axis should have 0 errors/warnings: {:?}",
            val_ok.diagnostics
        );

        // 3. Generate all 5 ops (Rotary Spiral + 4 Indexed 3+1 Flats), verify 0 sim crashes, and post-process!
        let mut cache = ToolpathCache::new(16);
        let batch = generate_project_blocking(&mut proj, &cad_doc, &mut cache, 1);
        assert!(
            batch.is_ok() && batch.generated.len() == 5,
            "Expected all 5 rotary shaft ops to generate cleanly, got errors={:?}, gen={}",
            batch.errors,
            batch.generated.len()
        );
        let sim = simulate_project_setup(&SetupSimRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            kinematics: resolve_kinematics_for_key("linuxcnc_4axis", None),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert!(
            !sim.has_critical_errors(),
            "Rotary Indexed Shaft sample must verify crash-free in CylindricalStock, got: {:#?}",
            sim.diagnostics
        );
        let profiles = builtin_post_profiles();
        let linux_4ax = profiles.iter().find(|p| p.id == "linuxcnc_4axis").unwrap();
        let grbl_wrap = profiles
            .iter()
            .find(|p| p.id == "grbl_rotary_wrap")
            .unwrap();
        let post_files = post_process_project_setup(&SetupPostRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &cad_doc,
            cad_revision: 1,
            cache: &cache,
            post_profile: linux_4ax,
            project_label: &proj.project_name,
            program_stem: "rotary_shaft",
        })
        .unwrap();
        assert_eq!(post_files.len(), 1);
        assert!(
            post_files[0].content.contains("G93") && post_files[0].content.contains("A90.000"),
            "Expected G93 inverse-time and A90.000 indexed moves in LinuxCNC 4-axis output"
        );

        // 4. Switching to 3-axis `grbl` post triggers a helpful compatibility warning
        proj.setups[0].post_profile = "grbl".to_string();
        let val_warn = validate_project(&proj, None, Some(&cad_doc));
        assert!(val_warn.is_valid());
        assert!(
            val_warn
                .diagnostics
                .iter()
                .any(|d| d.severity == DiagnosticSeverity::Warning && d.message.contains("3-axis")),
            "Expected 3-axis post warning on rotary setup: {:?}",
            val_warn.diagnostics
        );

        // 5. Wrapped Rotary Engraving (`Z+X+A`) round-trips, validates, simulates crash-free,
        //    and posts cleanly to both `linuxcnc_4axis` (G93) and `grbl_rotary_wrap` (Y-swapped)!
        let mut wrap_proj = build_rotary_wrapped_engraving_project(&starter.tools);
        let wrap_toml = wrap_proj.to_toml_string().unwrap();
        assert!(wrap_toml.contains("wrap_cylinder"));
        assert_eq!(
            wrap_toml,
            ProjectDocument::from_toml_str(&wrap_toml)
                .unwrap()
                .to_toml_string()
                .unwrap()
        );
        let empty_cad = CadDocument::empty();
        let wrap_val = validate_project(&wrap_proj, None, Some(&empty_cad));
        assert!(
            wrap_val.is_valid() && wrap_val.warning_count() == 0,
            "Wrapped Rotary Engraving sample should validate cleanly: {:?}",
            wrap_val.diagnostics
        );
        let mut wrap_cache = ToolpathCache::new(16);
        let wrap_batch = generate_project_blocking(&mut wrap_proj, &empty_cad, &mut wrap_cache, 1);
        assert!(
            wrap_batch.is_ok() && wrap_batch.generated.len() == 2,
            "Expected both wrapped rotary ops to generate cleanly, got errors={:?}, gen={}",
            wrap_batch.errors,
            wrap_batch.generated.len()
        );
        let wrap_sim = simulate_project_setup(&SetupSimRequest {
            project: &wrap_proj,
            setup_idx: 0,
            cad_doc: &empty_cad,
            cad_revision: 1,
            cache: &wrap_cache,
            kinematics: resolve_kinematics_for_key("linuxcnc_4axis", None),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert!(
            !wrap_sim.has_critical_errors(),
            "Wrapped Rotary Engraving sample must verify crash-free, got: {:#?}",
            wrap_sim.diagnostics
        );
        let wrap_post_linux = post_process_project_setup(&SetupPostRequest {
            project: &wrap_proj,
            setup_idx: 0,
            cad_doc: &empty_cad,
            cad_revision: 1,
            cache: &wrap_cache,
            post_profile: linux_4ax,
            project_label: &wrap_proj.project_name,
            program_stem: "rotary_wrap",
        })
        .unwrap();
        assert!(wrap_post_linux.iter().any(|f| f.content.contains("G93")));

        let wrap_post_grbl = post_process_project_setup(&SetupPostRequest {
            project: &wrap_proj,
            setup_idx: 0,
            cad_doc: &empty_cad,
            cad_revision: 1,
            cache: &wrap_cache,
            post_profile: grbl_wrap,
            project_label: &wrap_proj.project_name,
            program_stem: "rotary_wrap",
        })
        .unwrap();
        assert!(!wrap_post_grbl.is_empty());
        for f in &wrap_post_grbl {
            let motion_lines_have_a = f.content.lines().any(|line| {
                let trimmed = line.trim();
                !trimmed.starts_with('(')
                    && !trimmed.starts_with(';')
                    && trimmed.split_whitespace().any(|tok| tok.starts_with('A'))
            });
            assert!(
                !motion_lines_have_a && f.content.contains('Y'),
                "grbl_rotary_wrap must swap A onto Y and emit zero A motion words in {}",
                f.filename
            );
        }

        // 6. Simultaneous 4-Axis Helical Flute (`X+Y+Z+A`) round-trips, validates, simulates
        //    crash-free, and posts simultaneous `G93 X Y Z A` G-code on linuxcnc_4axis & grblhal_4axis!
        let flute_stl = chip_cad::sample_helical_flute_shaft_stl_bytes();
        let flute_doc = load_mesh_cad_document(
            "helical_flute_shaft.stl",
            &flute_stl,
            &MeshImportOptions::default(),
        )
        .unwrap();
        let flute_sha = compute_sha256_hex(&flute_stl);
        let mut flute_proj = build_rotary_helical_flute_project(
            "helical_flute_shaft.stl".to_string(),
            &flute_sha,
            &starter.tools,
        );
        let flute_toml = flute_proj.to_toml_string().unwrap();
        assert!(flute_toml.contains("rotary_4axis_sim"));
        assert_eq!(
            flute_toml,
            ProjectDocument::from_toml_str(&flute_toml)
                .unwrap()
                .to_toml_string()
                .unwrap()
        );
        let flute_val = validate_project(&flute_proj, None, Some(&flute_doc));
        assert!(
            flute_val.is_valid() && flute_val.warning_count() == 0,
            "Simultaneous 4-Axis Helical Flute should validate cleanly: {:?}",
            flute_val.diagnostics
        );
        let mut flute_cache = ToolpathCache::new(16);
        let flute_batch =
            generate_project_blocking(&mut flute_proj, &flute_doc, &mut flute_cache, 1);
        assert!(
            flute_batch.is_ok() && flute_batch.generated.len() == 2,
            "Expected both simultaneous 4-axis ops to generate cleanly: {:?}",
            flute_batch.errors
        );
        let flute_sim = simulate_project_setup(&SetupSimRequest {
            project: &flute_proj,
            setup_idx: 0,
            cad_doc: &flute_doc,
            cad_revision: 1,
            cache: &flute_cache,
            kinematics: resolve_kinematics_for_key("linuxcnc_4axis", None),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert!(
            !flute_sim.has_critical_errors(),
            "Simultaneous 4-Axis Helical Flute must verify crash-free, got: {:#?}",
            flute_sim.diagnostics
        );
        let grblhal_4ax = profiles.iter().find(|p| p.id == "grblhal_4axis").unwrap();
        for prof in [linux_4ax, grblhal_4ax] {
            let files = post_process_project_setup(&SetupPostRequest {
                project: &flute_proj,
                setup_idx: 0,
                cad_doc: &flute_doc,
                cad_revision: 1,
                cache: &flute_cache,
                post_profile: prof,
                project_label: &flute_proj.project_name,
                program_stem: "helical_flute",
            })
            .unwrap();
            assert_eq!(files.len(), 1);
            let gcode = &files[0].content;
            assert!(gcode.contains("G93"), "{} must emit G93", prof.id);
            let has_simultaneous_xyza = gcode.lines().any(|line| {
                let t = line.trim();
                !t.starts_with('(')
                    && !t.starts_with(';')
                    && t.contains('X')
                    && t.contains('Y')
                    && t.contains('Z')
                    && t.contains('A')
                    && t.contains('F')
            });
            assert!(
                has_simultaneous_xyza,
                "{} must emit simultaneous X Y Z A F blocks",
                prof.id
            );
        }
    }

    #[test]
    fn tutorial_rotary_projects_roundtrip_simulate_crash_free_and_post() {
        use chip_cad::{
            MeshImportOptions, load_mesh_cad_document, sample_helical_flute_shaft_stl_bytes,
            sample_rotary_shaft_stl_bytes, tutorial_indexed_4side_block_step,
        };
        use chip_post::builtin_post_profiles;
        use chip_sim::SimResolution;

        let starter = default_starter_tool_library();
        let profiles = builtin_post_profiles();
        let linux_4ax = profiles.iter().find(|p| p.id == "linuxcnc_4axis").unwrap();
        let grblhal_4ax = profiles.iter().find(|p| p.id == "grblhal_4axis").unwrap();
        let grbl_wrap = profiles
            .iter()
            .find(|p| p.id == "grbl_rotary_wrap")
            .unwrap();

        // 1. Tutorial 10 — Rotary Axial Parallel (-60°..+60°) + Continuous Spiral (Z+X+A)
        let shaft_stl = sample_rotary_shaft_stl_bytes();
        let shaft_doc = load_mesh_cad_document(
            "09-rotary-shaft.stl",
            &shaft_stl,
            &MeshImportOptions::default(),
        )
        .unwrap();
        let shaft_sha = compute_sha256_hex(&shaft_stl);
        let mut axial_proj = build_tutorial_rotary_axial_parallel_project(
            "09-rotary-shaft.stl".to_string(),
            &shaft_sha,
            &starter.tools,
        );
        let axial_toml = axial_proj.to_toml_string().unwrap();
        assert!(axial_toml.contains("axial_parallel"));
        assert!(axial_toml.contains("continuous_spiral"));
        assert_eq!(
            axial_toml,
            ProjectDocument::from_toml_str(&axial_toml)
                .unwrap()
                .to_toml_string()
                .unwrap()
        );
        let axial_val = validate_project(&axial_proj, None, Some(&shaft_doc));
        assert!(
            axial_val.is_valid() && axial_val.warning_count() == 0,
            "Tutorial 10 Axial Parallel validation issues: {:?}",
            axial_val.diagnostics
        );
        let mut axial_cache = ToolpathCache::new(16);
        let axial_batch =
            generate_project_blocking(&mut axial_proj, &shaft_doc, &mut axial_cache, 1);
        assert!(
            axial_batch.is_ok() && axial_batch.generated.len() == 2,
            "Tutorial 10 Axial Parallel generation errors: {:?}",
            axial_batch.errors
        );
        let axial_sim = simulate_project_setup(&SetupSimRequest {
            project: &axial_proj,
            setup_idx: 0,
            cad_doc: &shaft_doc,
            cad_revision: 1,
            cache: &axial_cache,
            kinematics: resolve_kinematics_for_key("linuxcnc_4axis", None),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert!(
            !axial_sim.has_critical_errors(),
            "Tutorial 10 Axial Parallel must have 0 critical errors in CylindricalStock: {:#?}",
            axial_sim.diagnostics
        );
        let axial_post_lcnc = post_process_project_setup(&SetupPostRequest {
            project: &axial_proj,
            setup_idx: 0,
            cad_doc: &shaft_doc,
            cad_revision: 1,
            cache: &axial_cache,
            post_profile: linux_4ax,
            project_label: &axial_proj.project_name,
            program_stem: "rotary_axial",
        })
        .unwrap();
        assert!(axial_post_lcnc[0].content.contains("G93"));
        let axial_post_wrap = post_process_project_setup(&SetupPostRequest {
            project: &axial_proj,
            setup_idx: 0,
            cad_doc: &shaft_doc,
            cad_revision: 1,
            cache: &axial_cache,
            post_profile: grbl_wrap,
            project_label: &axial_proj.project_name,
            program_stem: "rotary_axial",
        })
        .unwrap();
        assert!(!axial_post_wrap.is_empty());
        for f in &axial_post_wrap {
            let has_a_word = f.content.lines().any(|line| {
                let t = line.trim();
                !t.starts_with('(')
                    && !t.starts_with(';')
                    && t.split_whitespace().any(|tok| tok.starts_with('A'))
            });
            assert!(
                !has_a_word && f.content.contains('Y'),
                "grbl_rotary_wrap must swap A onto Y with zero A words in {}",
                f.filename
            );
        }

        // 2. Tutorial 11 — Indexed 3+1 Multi-Face STEP Block (0°, 45°, 90°, 180°, 270°)
        let block_step = tutorial_indexed_4side_block_step().unwrap();
        let block_doc = MonstertruckKernel
            .load_step_bytes(
                "11-indexed-multiface-block.step",
                block_step.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .unwrap();
        let block_sha = compute_sha256_hex(block_step.as_bytes());
        let mut idx_proj = build_tutorial_indexed_step_multiface_project(
            &block_doc,
            "11-indexed-multiface-block.step".to_string(),
            &block_sha,
            &starter.tools,
        );
        assert_eq!(idx_proj.setups[0].operations.len(), 5);
        let idx_toml = idx_proj.to_toml_string().unwrap();
        assert!(idx_toml.contains("indexed3_plus1"));
        assert_eq!(
            idx_toml,
            ProjectDocument::from_toml_str(&idx_toml)
                .unwrap()
                .to_toml_string()
                .unwrap()
        );
        let idx_val = validate_project(&idx_proj, None, Some(&block_doc));
        assert!(
            idx_val.is_valid() && idx_val.warning_count() == 0,
            "Tutorial 11 Indexed STEP validation issues: {:?}",
            idx_val.diagnostics
        );
        let mut idx_cache = ToolpathCache::new(16);
        let idx_batch = generate_project_blocking(&mut idx_proj, &block_doc, &mut idx_cache, 1);
        assert!(
            idx_batch.is_ok() && idx_batch.generated.len() == 5,
            "Tutorial 11 Indexed STEP generation errors: {:?}",
            idx_batch.errors
        );
        let idx_sim = simulate_project_setup(&SetupSimRequest {
            project: &idx_proj,
            setup_idx: 0,
            cad_doc: &block_doc,
            cad_revision: 1,
            cache: &idx_cache,
            kinematics: resolve_kinematics_for_key("linuxcnc_4axis", None),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert!(
            !idx_sim.has_critical_errors(),
            "Tutorial 11 Indexed STEP must have 0 critical errors in CylindricalStock: {:#?}",
            idx_sim.diagnostics
        );
        let idx_post = post_process_project_setup(&SetupPostRequest {
            project: &idx_proj,
            setup_idx: 0,
            cad_doc: &block_doc,
            cad_revision: 1,
            cache: &idx_cache,
            post_profile: linux_4ax,
            project_label: &idx_proj.project_name,
            program_stem: "indexed_multiface",
        })
        .unwrap();
        assert_eq!(idx_post.len(), 1);
        for word in ["A45.000", "A90.000", "A180.000", "A270.000"] {
            assert!(
                idx_post[0].content.contains(word),
                "Tutorial 11 Indexed STEP G-code must contain {word}"
            );
        }

        // 3. Tutorial 12 — Simultaneous 4-Axis Auto-Tilt Spiral vs. Fixed-Offset Helical Flowline
        let flute_stl = sample_helical_flute_shaft_stl_bytes();
        let flute_doc = load_mesh_cad_document(
            "10-helical-flute-shaft.stl",
            &flute_stl,
            &MeshImportOptions::default(),
        )
        .unwrap();
        let flute_sha = compute_sha256_hex(&flute_stl);
        let mut flow_proj = build_tutorial_rotary_4axis_flowline_project(
            "10-helical-flute-shaft.stl".to_string(),
            &flute_sha,
            &starter.tools,
        );
        assert_eq!(flow_proj.setups[0].operations.len(), 3);
        let flow_toml = flow_proj.to_toml_string().unwrap();
        assert!(flow_toml.contains("surface_normal_auto_tilt"));
        assert!(flow_toml.contains("fixed_lateral_offset"));
        assert!(flow_toml.contains("helical_flowline"));
        assert_eq!(
            flow_toml,
            ProjectDocument::from_toml_str(&flow_toml)
                .unwrap()
                .to_toml_string()
                .unwrap()
        );
        let flow_val = validate_project(&flow_proj, None, Some(&flute_doc));
        assert!(
            flow_val.is_valid() && flow_val.warning_count() == 0,
            "Tutorial 12 4-Axis Flowline validation issues: {:?}",
            flow_val.diagnostics
        );
        let mut flow_cache = ToolpathCache::new(16);
        let flow_batch = generate_project_blocking(&mut flow_proj, &flute_doc, &mut flow_cache, 1);
        assert!(
            flow_batch.is_ok() && flow_batch.generated.len() == 3,
            "Tutorial 12 4-Axis Flowline generation errors: {:?}",
            flow_batch.errors
        );
        let flow_sim = simulate_project_setup(&SetupSimRequest {
            project: &flow_proj,
            setup_idx: 0,
            cad_doc: &flute_doc,
            cad_revision: 1,
            cache: &flow_cache,
            kinematics: resolve_kinematics_for_key("grblhal_4axis", None),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert!(
            !flow_sim.has_critical_errors(),
            "Tutorial 12 4-Axis Flowline must have 0 critical errors in CylindricalStock: {:#?}",
            flow_sim.diagnostics
        );
        for prof in [linux_4ax, grblhal_4ax] {
            let files = post_process_project_setup(&SetupPostRequest {
                project: &flow_proj,
                setup_idx: 0,
                cad_doc: &flute_doc,
                cad_revision: 1,
                cache: &flow_cache,
                post_profile: prof,
                project_label: &flow_proj.project_name,
                program_stem: "rotary_4axis_flowline",
            })
            .unwrap();
            assert_eq!(files.len(), 1);
            assert!(files[0].content.contains("G93"));
            assert!(
                files[0].content.lines().any(|line| {
                    let t = line.trim();
                    !t.starts_with('(')
                        && !t.starts_with(';')
                        && t.contains('X')
                        && t.contains('Y')
                        && t.contains('Z')
                        && t.contains('A')
                        && t.contains('F')
                }),
                "{} must emit simultaneous X Y Z A F blocks",
                prof.id
            );
        }
    }

    #[test]
    fn machine_id_roundtrip_validation_warnings_and_chipz_bundling() {
        use chip_tools::{
            DEFAULT_MACHINE_ID, builtin_generic_machine, clone_machine_definition,
            save_user_machine_in,
        };

        let starter = default_starter_tool_library();
        let mut proj =
            build_tutorial_facing_project("billet.step".to_string(), "", &starter.tools, false);
        // 1. Default `machine_id: None` omits field in TOML and resolves to `"generic"`
        assert_eq!(proj.effective_machine_id(), DEFAULT_MACHINE_ID);
        let toml_no_machine = proj.to_toml_string().unwrap();
        assert!(
            !toml_no_machine.contains("machine_id"),
            "machine_id must be omitted when None for backward compatibility"
        );

        // 2. Validate against a compact 3-axis custom machine -> catches travel, RPM, feed, & rotary!
        let generic = builtin_generic_machine();
        let mut tiny = clone_machine_definition(&generic, "tiny-3ax", "Tiny 3-Axis Mill").unwrap();
        tiny.travel_mm.x_mm = 50.0;
        tiny.spindle.min_rpm = 10_000.0;
        tiny.spindle.max_rpm = 12_000.0;
        tiny.kinematics.max_cut_feed_xy_mm_min = 400.0;
        tiny.kinematics.max_cut_feed_z_mm_min = 100.0;
        tiny.rotary = None;

        // Force op feeds above tiny machine's limits
        let op = &mut proj.setups[0].operations[0];
        op.feed_overrides.spindle_rpm = Some(20_000.0);
        op.feed_overrides.cutting_feed_mm_min = Some(1_200.0);
        op.feed_overrides.plunge_feed_mm_min = Some(300.0);

        let step = chip_cad::tutorial_facing_billet_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("billet.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let report = validate_project_with_machine(&proj, None, Some(&doc), Some(&tiny));
        assert!(
            report.is_valid(),
            "machine checks must be non-blocking warnings"
        );
        let msgs: Vec<&str> = report
            .diagnostics
            .iter()
            .map(|d| d.message.as_str())
            .collect();
        assert!(
            msgs.iter().any(|m| m.contains("travel envelope")),
            "expected travel envelope warning in {msgs:?}"
        );
        assert!(
            msgs.iter().any(|m| m.contains("spindle range")),
            "expected spindle RPM warning in {msgs:?}"
        );
        assert!(
            msgs.iter().any(|m| m.contains("XY limit")),
            "expected XY feedrate warning in {msgs:?}"
        );
        assert!(
            msgs.iter().any(|m| m.contains("Z limit")),
            "expected Z plunge feedrate warning in {msgs:?}"
        );

        // 3. Rotary enabled on 3-axis-only machine -> emits rotary capability warning
        let rot_proj = build_rotary_wrapped_engraving_project(&starter.tools);
        let rot_report = validate_project_with_machine(&rot_proj, None, None, Some(&tiny));
        assert!(
            rot_report
                .diagnostics
                .iter()
                .any(|d| d.message.contains("3-axis only")),
            "expected 3-axis-only rotary warning: {:?}",
            rot_report.diagnostics
        );

        // 4. Custom machine `.machine.toml` is bundled cleanly into `.chipz` and resolved on unpack
        let tmp =
            std::env::temp_dir().join(format!("chipcutter_m29_bundle_test_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp);
        let proj_dir = tmp.join("job");
        let machines_dir = proj_dir.join("machines");
        std::fs::create_dir_all(&machines_dir).unwrap();
        save_user_machine_in(&tiny, &machines_dir).unwrap();
        std::fs::write(proj_dir.join("billet.step"), step.as_bytes()).unwrap();

        proj.apply_machine_defaults(&tiny, false);
        assert_eq!(proj.effective_machine_id(), "tiny-3ax");
        let (zip_bytes, summary) =
            pack_chipz_bundle_bytes(&proj, Some(&proj_dir), &[], &[]).unwrap();
        assert_eq!(
            summary.custom_machine_files,
            vec!["machines/tiny-3ax.machine.toml".to_string()]
        );

        let unpacked_dir = tmp.join("unpacked");
        let unpacked = unpack_chipz_bytes(&zip_bytes, &unpacked_dir).unwrap();
        assert_eq!(unpacked.project.effective_machine_id(), "tiny-3ax");
        assert!(
            unpacked_dir
                .join("machines/tiny-3ax.machine.toml")
                .is_file()
        );
        let resolved =
            chip_tools::resolve_machine_definition("tiny-3ax", Some(&unpacked_dir)).unwrap();
        assert_eq!(resolved.id, "tiny-3ax");
        assert_eq!(resolved.tools.len(), tiny.tools.len());

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn milo_1_5_and_carvera_air_end_to_end_machining_simulates_crash_free_with_zero_warnings() {
        use chip_cad::{sample_rotary_shaft_stl_bytes, tutorial_contour_plate_step};
        use chip_cam::{OperationKind, StrategyParams};
        use chip_math::ParamLength;
        use chip_sim::{MachineKinematicsConfig, SimResolution};
        use chip_tools::{builtin_carvera_air_machine, builtin_milo_1_5_machine};

        // 1. Millennium Milo v1.5 (LDO Kit): 2D Contour plate using Milo T1 (Ø6mm 1FL DLC Flat)
        let milo = builtin_milo_1_5_machine();
        let plate_step = tutorial_contour_plate_step().unwrap();
        let plate_doc = MonstertruckKernel
            .load_step_bytes(
                "plate.step",
                plate_step.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .unwrap();
        let mut milo_proj = build_tutorial_contour_project(
            &plate_doc,
            "plate.step".to_string(),
            "",
            &milo.tools,
            false,
        );
        milo_proj.tools.clear();
        let milo_t1 = &milo.tools[0];
        milo_proj.ensure_tool_embedded(milo_t1);
        milo_proj.setups[0].operations[0].tool_id = Some(milo_t1.id.clone());
        milo_proj.setups[0].operations[0].preset_id = Some(milo_t1.presets[0].id.clone());
        milo_proj.setups[0].operations[0].feed_overrides.clear_all();
        milo_proj.apply_machine_defaults(&milo, true);

        let milo_val = validate_project(&milo_proj, None, Some(&plate_doc));
        assert!(
            milo_val.is_valid() && milo_val.warning_count() == 0,
            "Milo v1.5 job must pass with 0 errors and 0 warnings, got: {:?}",
            milo_val.diagnostics
        );
        let mut milo_cache = ToolpathCache::new(16);
        let milo_batch = generate_project_blocking(&mut milo_proj, &plate_doc, &mut milo_cache, 1);
        assert!(
            milo_batch.errors.is_empty(),
            "Milo gen errors: {:?}",
            milo_batch.errors
        );
        let milo_sim = simulate_project_setup(&SetupSimRequest {
            project: &milo_proj,
            setup_idx: 0,
            cad_doc: &plate_doc,
            cad_revision: 1,
            cache: &milo_cache,
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::from_machine_spec(&milo.kinematics),
            cancel: None,
        })
        .unwrap();
        assert_eq!(
            milo_sim.critical_error_count(),
            0,
            "Milo simulation must be crash-free: {:?}",
            milo_sim.diagnostics
        );

        // 2. Makera Carvera Air: 4th-Axis Rotary Surfacing using Carvera T5 (Ø3.175mm 2FL Ball)
        let carvera = builtin_carvera_air_machine();
        let shaft_stl = sample_rotary_shaft_stl_bytes();
        let shaft_doc = load_model_bytes_cad_doc("shaft.stl", &shaft_stl, None).unwrap();
        let mut carvera_proj = build_tutorial_rotary_axial_parallel_project(
            "shaft.stl".to_string(),
            "",
            &carvera.tools,
        );
        carvera_proj.tools.clear();
        let carvera_ball = carvera.tools.iter().find(|t| t.tool_number == 5).unwrap();
        carvera_proj.ensure_tool_embedded(carvera_ball);
        let rot_op = &mut carvera_proj.setups[0].operations[0];
        assert_eq!(rot_op.kind, OperationKind::Rotary2Linear);
        rot_op.tool_id = Some(carvera_ball.id.clone());
        rot_op.preset_id = Some(carvera_ball.presets[0].id.clone());
        rot_op.feed_overrides.clear_all();
        if let StrategyParams::Rotary2Linear(p) = rot_op.strategy_mut() {
            p.stepover = ParamLength::from_mm(4.0);
        }
        carvera_proj.apply_machine_defaults(&carvera, true);

        let carvera_val = validate_project(&carvera_proj, None, Some(&shaft_doc));
        assert!(
            carvera_val.is_valid() && carvera_val.warning_count() == 0,
            "Carvera Air 4th-axis job must pass with 0 errors and 0 warnings, got: {:?}",
            carvera_val.diagnostics
        );
        let mut carvera_cache = ToolpathCache::new(16);
        let carvera_batch =
            generate_project_blocking(&mut carvera_proj, &shaft_doc, &mut carvera_cache, 1);
        assert!(
            carvera_batch.errors.is_empty(),
            "Carvera gen errors: {:?}",
            carvera_batch.errors
        );
        let carvera_sim = simulate_project_setup(&SetupSimRequest {
            project: &carvera_proj,
            setup_idx: 0,
            cad_doc: &shaft_doc,
            cad_revision: 1,
            cache: &carvera_cache,
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::from_machine_spec(&carvera.kinematics),
            cancel: None,
        })
        .unwrap();
        assert_eq!(
            carvera_sim.critical_error_count(),
            0,
            "Carvera Air 4th-axis simulation must be crash-free: {:?}",
            carvera_sim.diagnostics
        );
        let post4 =
            chip_post::resolve_post_profile(&carvera_proj.setups[0].post_profile, None).unwrap();
        assert_eq!(post4.id, "grblhal_4axis");
        let files = post_process_project_setup(&SetupPostRequest {
            project: &carvera_proj,
            setup_idx: 0,
            cad_doc: &shaft_doc,
            cad_revision: 1,
            cache: &carvera_cache,
            post_profile: &post4,
            project_label: &carvera_proj.project_name,
            program_stem: "carvera_rotary",
        })
        .unwrap();
        assert_eq!(files.len(), 1);
        assert!(files[0].content.contains("G93"));
    }

    #[test]
    fn milestone_33_private_temp_dir_and_external_asset_detection_and_entry_cap() {
        use std::io::Write as _;

        // 1. Private exclusive temp dir creation (`0o700` on Unix)
        let priv_dir = create_private_temp_extract_dir("chipcutter_m33_test").unwrap();
        assert!(priv_dir.is_dir());
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt as _;
            let mode = std::fs::metadata(&priv_dir).unwrap().permissions().mode() & 0o777;
            assert_eq!(mode, 0o700, "temp extract dir must be owner-only 0700");
        }

        // 2. Archive with > MAX_CHIPZ_ENTRIES is rejected cleanly
        let mut many_zip = std::io::Cursor::new(Vec::new());
        {
            let mut zw = zip::ZipWriter::new(&mut many_zip);
            let opts = zip::write::SimpleFileOptions::default();
            for i in 0..=MAX_CHIPZ_ENTRIES {
                zw.start_file(format!("f_{i}.txt"), opts).unwrap();
                zw.write_all(b"x").unwrap();
            }
            zw.finish().unwrap();
        }
        let err = unpack_chipz_bytes(&many_zip.into_inner(), &priv_dir).unwrap_err();
        assert!(matches!(err, BundleError::ArchiveTooLarge));

        // 3. External / non-standard assets are flagged before packing
        let mut proj = ProjectDocument::new("Untrusted Project");
        proj.models.push(ModelEntry {
            id: "m_ext".to_string(),
            path: "/etc/passwd".to_string(),
            sha256: String::new(),
            unit_override: None,
            transform: chip_cad::ModelTransform::IDENTITY,
        });
        let flagged = external_or_suspicious_assets_referenced(&proj, Some(&priv_dir));
        assert_eq!(flagged.len(), 1);

        let _ = std::fs::remove_dir_all(&priv_dir);
    }

    #[test]
    fn milestone_35_model_transform_toml_omission_chipz_roundtrip_and_reload_preservation() {
        // 1. Identity transform is omitted from `.chip.toml` for 100% backward compatibility
        let mut proj = ProjectDocument::new("Transform Test");
        proj.models.push(ModelEntry::new("m1", "part.step", "abc"));
        let toml_id = proj.to_toml_string().unwrap();
        assert!(
            !toml_id.contains("transform"),
            "identity transform must be omitted from TOML:\n{toml_id}"
        );

        // 2. Non-identity transform (scale + rotate + translate) roundtrips cleanly through TOML & `.chipz`
        let xf = chip_cad::ModelTransform {
            translate_mm: [12.5, -4.0, 2.0],
            rotate_deg: [90.0, 0.0, 45.0],
            scale: [1.5, 1.5, 1.5],
        };
        proj.models[0].transform = xf;
        let toml_xf = proj.to_toml_string().unwrap();
        let parsed = ProjectDocument::from_toml_str(&toml_xf).unwrap();
        assert_eq!(parsed.models[0].transform, xf);

        let step_text = sample_multi_feature_bracket_step(false).unwrap();
        let step_bytes = step_text.as_bytes();
        let (chipz_bytes, _sum) =
            pack_chipz_bundle_bytes(&proj, None, &[("m1", step_bytes)], &[]).unwrap();
        let tmp = create_private_temp_extract_dir("chipcutter_m35_chipz").unwrap();
        let unpacked = unpack_chipz_bytes(&chipz_bytes, &tmp).unwrap();
        assert_eq!(unpacked.project.models[0].transform, xf);

        // 3. Associative reload preserves `ModelEntry.transform`
        let raw_doc = MonstertruckKernel
            .load_step_bytes("part.step", step_bytes, 0.05)
            .unwrap();
        let t_doc = raw_doc.transformed(&xf);
        let topo = chip_cad::TopologyGraph::build(&t_doc);
        let mut reloaded_proj = unpacked.project.clone();
        let _rep = rebind_project_to_reloaded_cad(
            &mut reloaded_proj,
            0,
            &t_doc,
            &topo,
            &compute_sha256_hex(step_bytes),
            None,
        );
        assert_eq!(reloaded_proj.models[0].transform, xf);
        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn milestone_37_tutorial_16_yup_stl_and_tilted_step_simulate_crash_free() {
        use chip_cad::{MeshImportOptions, load_mesh_cad_document, tutorial_yup_bracket_stl_bytes};
        use chip_sim::{MachineKinematicsConfig, SimResolution};

        let starter = default_starter_tool_library();

        // 1. Verify Tutorial 16 Y-up STL fixture is watertight, transforms to 90x60x27 mm at Z>=0,
        // validates cleanly, and simulates crash-free with 0 errors.
        let stl_bytes = tutorial_yup_bracket_stl_bytes();
        let raw_stl = load_mesh_cad_document(
            "16-yup-bracket.stl",
            &stl_bytes,
            &MeshImportOptions::default(),
        )
        .unwrap();
        assert!(raw_stl.meshes[0].diagnostics.is_watertight);
        let sha_stl = compute_sha256_hex(&stl_bytes);
        let mut stl_proj = build_tutorial_16_yup_stl_project(
            "16-yup-bracket.stl".to_string(),
            &sha_stl,
            &starter.tools,
            true,
        );
        let transformed_stl = raw_stl.transformed(&stl_proj.models[0].transform);
        let sz = transformed_stl.aabb_mm.size();
        assert!((sz.x - 90.0).abs() < 0.05, "sz={sz:?}");
        assert!((sz.y - 60.0).abs() < 0.05, "sz={sz:?}");
        assert!((sz.z - 27.0).abs() < 0.05, "sz={sz:?}");
        assert!(transformed_stl.aabb_mm.min.z.abs() < 1e-4);

        let val_stl = validate_project(&stl_proj, None, Some(&transformed_stl));
        assert_eq!(
            val_stl.error_count(),
            0,
            "Tutorial 16 STL validation failed: {:?}",
            val_stl.diagnostics
        );
        let mut cache_stl = ToolpathCache::new(32);
        let gen_stl = generate_project_blocking(&mut stl_proj, &transformed_stl, &mut cache_stl, 1);
        assert!(
            gen_stl.errors.is_empty(),
            "STL generation errors: {gen_stl:?}"
        );
        let sim_stl = simulate_project_setup(&SetupSimRequest {
            project: &stl_proj,
            setup_idx: 0,
            cad_doc: &transformed_stl,
            cad_revision: 1,
            cache: &cache_stl,
            kinematics: MachineKinematicsConfig::default(),
            resolution: SimResolution::Standard,
            cancel: None,
        })
        .unwrap();
        assert!(
            !sim_stl.has_critical_errors(),
            "STL sim diagnostics: {:?}",
            sim_stl.diagnostics
        );

        // 2. Verify Tutorial 16 Tilted STEP 2-Setup project (Setup 1 G54 + Setup 2 G55 flip)
        // omits zero extra_z_rot_deg in TOML, validates cleanly, and simulates both Setups crash-free.
        let step_str = sample_multi_feature_bracket_step(true).unwrap();
        let step_doc = MonstertruckKernel
            .load_step_bytes(
                "16-tilted-bracket.step",
                step_str.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .unwrap();
        let sha_step = compute_sha256_hex(step_str.as_bytes());
        let mut step_proj = build_tutorial_16_tilted_step_project(
            &step_doc,
            "16-tilted-bracket.step".to_string(),
            &sha_step,
            &starter.tools,
        );
        let toml_step = step_proj.to_toml_string().unwrap();
        assert!(
            !toml_step.contains("extra_z_rot_deg"),
            "zero extra_z_rot_deg should be omitted from TOML"
        );
        assert_eq!(step_proj.setups.len(), 2);
        let val_step = validate_project(&step_proj, None, Some(&step_doc));
        assert_eq!(
            val_step.error_count(),
            0,
            "Tutorial 16 STEP validation failed: {:?}",
            val_step.diagnostics
        );
        let mut cache_step = ToolpathCache::new(32);
        let gen_step = generate_project_blocking(&mut step_proj, &step_doc, &mut cache_step, 1);
        assert!(
            gen_step.errors.is_empty(),
            "STEP generation errors: {gen_step:?}"
        );
        for s_idx in 0..2 {
            let sim_step = simulate_project_setup(&SetupSimRequest {
                project: &step_proj,
                setup_idx: s_idx,
                cad_doc: &step_doc,
                cad_revision: 1,
                cache: &cache_step,
                kinematics: MachineKinematicsConfig::default(),
                resolution: SimResolution::Standard,
                cancel: None,
            })
            .unwrap();
            assert!(
                !sim_step.has_critical_errors(),
                "STEP setup {s_idx} sim diagnostics: {:?}",
                sim_step.diagnostics
            );
        }
    }
}
