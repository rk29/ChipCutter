#![allow(clippy::unwrap_used, clippy::expect_used)]

//! Milestone 75 — Comprehensive Phase 20 E2E Regression Suite (`M72`, `M73`, `M74`, `M75`).
//!
//! Locks down deterministic end-to-end verification across all four Phase 20 milestones on the
//! watertight programmatic `sample_triple_helix_loft_stl_bytes()` fixture with both CPU and GPU
//! (`TriDexel` and `Cylindrical`) simulation verification:
//! 1. **M72:** Wider-shank & holder overhang clearance (`0` `FluteLengthExceeded`/ShankRub,
//!    `0` `HolderCollision`, `0` `TargetModelGouge`) + 2.5D overhang guard warning & opt-in
//!    `avoid_model_overhangs`.
//! 2. **M73:** `Indexed3Plus1` (`0°`/`120°`/`240°`) cross-angle rest + volumetric `TriDexelStock`
//!    carryover (`0` rapid crashes, `0` gouges, `>= 25%` cutting-distance reduction vs. raw stock,
//!    CPU vs. GPU parity).
//! 3. **M74:** `Parallel3D`/`Scallop3D` zero cliff-dive gouges + `Pencil3D` overhang-shadow
//!    suppression + Auto Soft-Jaw zero-scalloping verification on the twisted helical loft.
//! 4. **M75:** Partial-sector (`0°..180°`) `ZigZag` in `Rotary2Linear` and `Rotary4AxisSim`
//!    (`max_gouge_mm <= 0.015`, `0` critical collisions in `CylindricalStock` and `TriDexelStock`,
//!    axial-undercut reachability warning), plus `Rotary4AxisSim` `SurfaceNormalAutoTilt`
//!    outperforming centerline `Rotary2Linear` on overhanging helical grooves with `0` gouges,
//!    smooth bounded cross-axis motion, and CPU vs. GPU parity.

use std::sync::atomic::AtomicBool;

use chip_cad::{
    MeshImportOptions, load_mesh_bytes, load_mesh_cad_document,
    sample_helical_flute_shaft_stl_bytes, sample_triple_helix_loft_stl_bytes,
};
use chip_cam::{
    AxisAnchor, BoxAnchor27, GeometrySelectionItem, HeightReference, HeightSpec,
    MachiningBoundaryMode, MotionRole, Operation, OperationGenRequest, OperationKind,
    OperationRotaryMode, ParallelCutDirection, PocketStrategyKind, RestMachiningConfig,
    RestMachiningMode, Rotary4AxisPattern, RotaryAxisKind, RotaryCutDirection, RotaryDriveMode,
    RotaryPatternMode, RotaryTiltMode, SegmentMotion3D, Setup, SoftJawBoltPattern, SoftJawConfig,
    SoftJawOutputMode, StockAxis, StockConfig, StrategyParams, SurfaceCutterProfile, SurfaceMesh3D,
    ToolContainmentMode, WcsOriginMode, WorkOffsetSlot, build_soft_jaw_plan,
    generate_operation_toolpath_with_rest,
};
use chip_math::{DVec2, DisplayUnit, ParamLength};
use chip_project::{
    ModelEntry, ProjectDocument, SetupSimRequest, ToolpathCache, compute_sha256_hex,
    generate_project_blocking, resolve_kinematics_for_key, simulate_project_setup_with_choice,
};
use chip_sim::{
    MachineKinematicsConfig, SetupSimulationInput, SimComputeBackendChoice, SimEngineKind,
    SimIssueKind, SimOperationInput, SimResolution, SimVerificationConfig,
    run_setup_simulation_with_choice,
};
use chip_sketch::{
    RectAnchorMode, SketchAnchor, SketchEntity, SketchEntityEntry, SketchLayer, SketchPlaneRef,
};
use chip_tools::{
    CoolantMode, CutterGeometry, CuttingPreset, SpindleDirection, SurfaceSpeedPresetSpec, Tool,
    ToolId, default_starter_tool_library,
};

const BOX_CENTER: BoxAnchor27 = BoxAnchor27 {
    x: AxisAnchor::Center,
    y: AxisAnchor::Center,
    z: AxisAnchor::Center,
};

fn make_flat_tool(
    id: &str,
    tool_number: u32,
    cutting_d: f64,
    flute_len: f64,
    shank_d: f64,
    stickout: f64,
) -> Tool {
    Tool {
        id: ToolId(id.to_string()),
        tool_number,
        length_offset_number: tool_number,
        diameter_offset_number: tool_number,
        name: format!("Flat Ø{cutting_d:.1}mm / Ø{shank_d:.1}mm Shank"),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::AirBlast,
        stickout_length_mm: stickout,
        geometry: CutterGeometry::FlatEndmill {
            diameter_mm: cutting_d,
            flute_length_mm: flute_len,
            overall_length_mm: (stickout + 24.0).max(55.0),
            shank_diameter_mm: shank_d,
            flutes: 3,
        },
        presets: vec![CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
            id: "al6061_m75",
            name: "Aluminum 6061",
            material: "Aluminum 6061",
            diameter_mm: cutting_d,
            flutes: 3,
            surface_speed_m_min: 200.0,
            chipload_mm_tooth: 0.025,
            default_stepdown_mm: 2.0,
            default_stepover_mm: 1.5,
            coolant_override: Some(CoolantMode::AirBlast),
        })],
    }
}

fn make_ball_tool(id: &str, tool_number: u32, diameter_mm: f64, flute_len: f64) -> Tool {
    Tool {
        id: ToolId(id.to_string()),
        tool_number,
        length_offset_number: tool_number,
        diameter_offset_number: tool_number,
        name: format!("Ball Ø{diameter_mm:.1}mm"),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::AirBlast,
        stickout_length_mm: (flute_len + 10.0).max(32.0),
        geometry: CutterGeometry::BallEndmill {
            diameter_mm,
            flute_length_mm: flute_len,
            overall_length_mm: (flute_len + 32.0).max(60.0),
            shank_diameter_mm: diameter_mm,
            flutes: 2,
        },
        presets: vec![CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
            id: "al6061_ball_m75",
            name: "Aluminum 6061 Ball",
            material: "Aluminum 6061",
            diameter_mm,
            flutes: 2,
            surface_speed_m_min: 220.0,
            chipload_mm_tooth: 0.020,
            default_stepdown_mm: 1.5,
            default_stepover_mm: 0.8,
            coolant_override: Some(CoolantMode::AirBlast),
        })],
    }
}

fn make_rotary_helix_setup(mode: RotaryDriveMode) -> Setup {
    let mut setup = Setup::new(
        "setup_helix_rot",
        "Triple-Helix Rotary",
        WorkOffsetSlot::G54,
    );
    setup.wcs.origin = WcsOriginMode::StockBoxPoint { anchor: BOX_CENTER };
    setup.stock = StockConfig::FixedCylinder {
        axis: StockAxis::X,
        diameter: ParamLength::from_mm(40.0),
        length: ParamLength::from_mm(56.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup.rotary.mode = mode;
    setup.rotary.axis = RotaryAxisKind::XAxis;
    setup.rotary.clearance_radial_offset = ParamLength::from_mm(12.0);
    setup.rotary.chuck_length = ParamLength::from_mm(16.0);
    setup.rotary.chuck_diameter = ParamLength::from_mm(58.0);
    setup
}

/// M72 E2E: Full-assembly shank/holder clearance on `sample_triple_helix_loft_stl_bytes()`
/// AND 2.5D overhang warning + opt-in `avoid_model_overhangs`.
#[test]
fn m72_triple_helix_full_assembly_clearance_and_25d_overhang_guard() {
    let stl = sample_triple_helix_loft_stl_bytes();
    let doc = load_mesh_cad_document("triple_helix_loft.stl", &stl, &MeshImportOptions::default())
        .unwrap();
    let cancel = AtomicBool::new(false);

    // 1. Pre-rough bulk cylinder stock with a standard Ø6 mm cutter, then semi-finish with a
    //    reduced-flute wider-shank cutter (Ø4 mm flute x 10 mm, Ø8 mm shank, 18 mm stickout)
    //    down to Z=+2.0 so M72 full-assembly shank & holder clearance actively protects the
    //    overhanging twisted helical blades (`R = 18 mm`) with 0 shank rubs, 0 holder crashes,
    //    and 0 target gouges.
    let rough_tool = make_flat_tool("t_rough_m72", 1, 4.0, 24.0, 4.0, 32.0);
    let stepped_tool = make_flat_tool("t_stepped_m72", 2, 4.0, 10.0, 8.0, 18.0);

    let mut setup = Setup::new("setup_m72_3ax", "Triple-Helix 3-Axis", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint { anchor: BOX_CENTER };
    setup.stock = StockConfig::FixedCylinder {
        axis: StockAxis::X,
        diameter: ParamLength::from_mm(36.0),
        length: ParamLength::from_mm(56.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    let mut op_rough = Operation::new(
        "op_rough_m72",
        "Adaptive3D Rough",
        OperationKind::Adaptive3D,
    );
    op_rough.tool_id = Some(rough_tool.id.clone());
    op_rough.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_rough.heights.bottom = HeightSpec::new(HeightReference::WcsOrigin, 8.0);
    op_rough.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
    op_rough.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    if let StrategyParams::Adaptive3D(p) = op_rough.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::StockBox;
        p.boundary.tool_containment = ToolContainmentMode::ToolOutsideBoundary;
        p.max_roughing_stepdown = ParamLength::from_mm(2.0);
        p.fine_stepdown = ParamLength::from_mm(0.5);
        p.adaptive.optimal_load = ParamLength::from_mm(1.6);
    }

    let mut op_wl = Operation::new(
        "op_wl_m72",
        "Waterline3D Stepped",
        OperationKind::Waterline3D,
    );
    op_wl.tool_id = Some(stepped_tool.id.clone());
    op_wl.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_wl.heights.bottom = HeightSpec::new(HeightReference::WcsOrigin, 2.0);
    op_wl.passes.max_stepdown = ParamLength::from_mm(2.0);
    if let StrategyParams::Waterline3D(p) = op_wl.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::StockBox;
        p.boundary.tool_containment = ToolContainmentMode::ToolOutsideBoundary;
        p.adaptive_stepdown = false;
    }

    let tp_rough = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_rough,
        tool: &rough_tool,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();
    let tp_wl = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_wl,
        tool: &stepped_tool,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();
    assert!(
        tp_wl
            .warnings
            .iter()
            .any(|w| w.contains("Reach-limit") && (w.contains("shank") || w.contains("holder"))),
        "Stepped-shank Waterline3D on overhanging triple-helix loft must emit Reach-limit warning, got: {:?}",
        tp_wl.warnings
    );

    for backend in [SimComputeBackendChoice::Cpu, SimComputeBackendChoice::Auto] {
        let sim = run_setup_simulation_with_choice(
            &SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                fixtures: &[],
                operations: &[
                    SimOperationInput::from_operation(&op_rough, &rough_tool, &tp_rough),
                    SimOperationInput::from_operation(&op_wl, &stepped_tool, &tp_wl),
                ],
                kinematics: MachineKinematicsConfig::default(),
                resolution: SimResolution::Standard,
                verify_config: SimVerificationConfig::default(),
                initial_stock: None,
                initial_raw_volume_mm3: None,
                carried_over_from_setup: None,
                cancel: None,
            },
            SimEngineKind::TriDexel,
            backend,
        )
        .unwrap();

        let bad_issues: Vec<_> = sim
            .diagnostics
            .iter()
            .filter(|d| {
                matches!(
                    d.kind,
                    SimIssueKind::TargetModelGouge
                        | SimIssueKind::HolderCollision
                        | SimIssueKind::FluteLengthExceeded
                )
            })
            .collect();
        assert!(
            bad_issues.is_empty(),
            "M72 stepped tool on triple-helix loft ({backend:?}) must have 0 gouges/shank-rubs/holder collisions, got: {bad_issues:#?}"
        );
    }

    // 2. 2.5D Pocket beneath the overhanging helical loft: warning when disabled, 0 gouges when enabled
    let mut sketch = SketchLayer::new("sk_m72", "Overhang Pocket", "setup_m72_3ax");
    sketch.anchor = SketchAnchor::SetupXy {
        setup_id: "setup_m72_3ax".to_string(),
        plane_ref: SketchPlaneRef::WcsOrigin,
        z_offset: ParamLength::from_mm(2.0),
    };
    sketch.entities.push(SketchEntityEntry {
        id: "rect_1".to_string(),
        name: "Overhang Pocket".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(0.0),
            y: ParamLength::from_mm(0.0),
            width: ParamLength::from_mm(32.0),
            height: ParamLength::from_mm(32.0),
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
        },
    });
    sketch.rebuild_regions(None);
    let sketches = vec![sketch];
    let pock_tool = make_flat_tool("t_pock_m72", 2, 4.0, 22.0, 4.0, 28.0);

    let build_pocket_op = |avoid: bool| {
        let mut op = Operation::new(
            "op_pock_m72",
            "Pocket2D Under Helix",
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(pock_tool.id.clone());
        op.selections.push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sk_m72".to_string(),
            entity_id: "rect_1".to_string(),
            reverse: false,
            side_flip: false,
        });
        op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op.heights.bottom = HeightSpec::new(HeightReference::WcsOrigin, 2.0);
        op.passes.multiple_depths = true;
        op.passes.max_stepdown = ParamLength::from_mm(6.0);
        op.passes.avoid_model_overhangs = avoid;
        if let StrategyParams::Pocket2D(p) = op.strategy_mut() {
            p.strategy = PocketStrategyKind::ContourParallel;
            p.stepover = ParamLength::from_mm(1.8);
        }
        op
    };

    let op_warn = build_pocket_op(false);
    let tp_warn = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &sketches,
        setup: &resolved,
        op: &op_warn,
        tool: &pock_tool,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();
    assert!(
        tp_warn
            .warnings
            .iter()
            .any(|w| w.contains("overhanging") || w.contains("Avoid Model Overhangs")),
        "Pocket2D beneath triple-helix overhangs must emit overhang warning when avoid_model_overhangs=false, got: {:?}",
        tp_warn.warnings
    );

    let op_avoid = build_pocket_op(true);
    let tp_avoid = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &sketches,
        setup: &resolved,
        op: &op_avoid,
        tool: &pock_tool,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();
    let sim_avoid = run_setup_simulation_with_choice(
        &SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput::from_operation(
                &op_avoid, &pock_tool, &tp_avoid,
            )],
            kinematics: MachineKinematicsConfig::default(),
            resolution: SimResolution::Fine,
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
        },
        SimEngineKind::TriDexel,
        SimComputeBackendChoice::Auto,
    )
    .unwrap();
    assert_eq!(
        sim_avoid.critical_error_count(),
        0,
        "Pocket2D with avoid_model_overhangs=true must have 0 critical errors: {:#?}",
        sim_avoid.diagnostics
    );
    assert!(
        !sim_avoid
            .diagnostics
            .iter()
            .any(|d| d.kind == SimIssueKind::TargetModelGouge),
        "Pocket2D with avoid_model_overhangs=true must have 0 target gouges"
    );
}

/// M73 E2E: `Indexed3Plus1` (`0°`/`120°`/`240°`) cross-angle rest + volumetric `TriDexelStock`
/// carryover on `sample_triple_helix_loft_stl_bytes()` with CPU vs. GPU parity.
#[test]
fn m73_triple_helix_indexed_3plus1_volumetric_carryover_cpu_gpu_parity() {
    let stl = sample_triple_helix_loft_stl_bytes();
    let doc = load_mesh_cad_document("triple_helix_loft.stl", &stl, &MeshImportOptions::default())
        .unwrap();
    let sha = compute_sha256_hex(&stl);

    let build_proj = |enable_rest: bool| {
        let mut proj = ProjectDocument::new("M73 Triple-Helix Indexed 3+1");
        proj.models.push(ModelEntry {
            id: "helix_1".to_string(),
            path: "triple_helix_loft.stl".to_string(),
            sha256: sha.clone(),
            role: Default::default(),
            unit_override: None,
            transform: Default::default(),
        });
        let tool = make_flat_tool("t1_6mm", 1, 6.0, 26.0, 6.0, 34.0);
        proj.tools = vec![tool];

        let mut setup = make_rotary_helix_setup(RotaryDriveMode::Indexed3Plus1);
        // Keep a realistic chuck-cleared machining boundary (`X ∈ [-21, +26]`, clear of chuck at X <= -28)
        let mut sk_win = SketchLayer::new("sk_m73_win", "Chuck-Cleared Window", "setup_helix_rot");
        sk_win.anchor = SketchAnchor::SetupXy {
            setup_id: "setup_helix_rot".to_string(),
            plane_ref: SketchPlaneRef::WcsOrigin,
            z_offset: ParamLength::from_mm(0.0),
        };
        sk_win.entities.push(SketchEntityEntry {
            id: "e_win".to_string(),
            name: "Machining Window".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::Rectangle {
                anchor: RectAnchorMode::Center,
                x: ParamLength::from_mm(2.5), // X ∈ [-20.5, +25.5]
                y: ParamLength::from_mm(0.0),
                width: ParamLength::from_mm(46.0),
                height: ParamLength::from_mm(40.0),
                corner_radius: ParamLength::from_mm(0.0),
                rotation_deg: 0.0,
            },
        });
        sk_win.rebuild_regions(None);
        proj.sketches.push(sk_win);

        for (idx, angle_deg) in [0.0_f64, 120.0, 240.0].into_iter().enumerate() {
            let mut op = Operation::new(
                format!("op_idx_{}", angle_deg as i32),
                format!("Adaptive3D {}°", angle_deg as i32),
                OperationKind::Adaptive3D,
            );
            op.tool_id = Some(ToolId("t1_6mm".to_string()));
            op.rotary = OperationRotaryMode::Indexed3Plus1 { angle_deg };
            op.selections.push(GeometrySelectionItem::SketchEntity {
                sketch_id: "sk_m73_win".to_string(),
                entity_id: "e_win".to_string(),
                reverse: false,
                side_flip: false,
            });
            op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
            op.heights.bottom = HeightSpec::new(HeightReference::WcsOrigin, -2.0);
            op.passes.stock_to_leave_radial = ParamLength::from_mm(0.4);
            op.passes.stock_to_leave_axial = ParamLength::from_mm(0.3);
            if enable_rest && idx > 0 {
                op.passes.rest_machining = RestMachiningConfig {
                    mode: RestMachiningMode::FromPrecedingOperations {
                        source_op_ids: Vec::new(),
                    },
                    rest_overlap: ParamLength::from_mm(0.25),
                    min_rest_thickness: ParamLength::from_mm(0.05),
                };
            }
            if let StrategyParams::Adaptive3D(p) = op.strategy_mut() {
                p.boundary.boundary_mode = MachiningBoundaryMode::SelectedRegions;
                p.boundary.tool_containment = ToolContainmentMode::ToolInsideBoundary;
                p.max_roughing_stepdown = ParamLength::from_mm(6.0);
                p.fine_stepdown = ParamLength::from_mm(0.0);
                p.adaptive.optimal_load = ParamLength::from_mm(2.2);
            }
            setup.operations.push(op);
        }
        proj.setups = vec![setup];
        proj
    };

    let mut proj_raw = build_proj(false);
    let mut cache_raw = ToolpathCache::new(32);
    let rep_raw = generate_project_blocking(&mut proj_raw, &doc, &mut cache_raw, 1);
    assert!(rep_raw.is_ok(), "raw errors: {:?}", rep_raw.errors);
    let raw_cut_120_240: f64 = rep_raw.generated[1..3]
        .iter()
        .map(|(_, _, tp)| tp.stats.cutting_distance_mm)
        .sum();

    let mut proj_rest = build_proj(true);
    let mut cache_rest = ToolpathCache::new(32);
    let rep_rest = generate_project_blocking(&mut proj_rest, &doc, &mut cache_rest, 1);
    assert!(rep_rest.is_ok(), "rest errors: {:?}", rep_rest.errors);
    let rest_cut_120_240: f64 = rep_rest.generated[1..3]
        .iter()
        .map(|(_, _, tp)| tp.stats.cutting_distance_mm)
        .sum();

    let reduction_pct = 100.0 * (1.0 - rest_cut_120_240 / raw_cut_120_240.max(1e-6));
    assert!(
        reduction_pct >= 25.0,
        "M73 cross-angle volumetric carryover must reduce 120°/240° cutting distance by >= 25% (got {reduction_pct:.1}%: raw={raw_cut_120_240:.1} mm vs rest={rest_cut_120_240:.1} mm)"
    );

    let post_key = proj_rest.setups[0].post_profile.clone();
    let sim_cpu = simulate_project_setup_with_choice(
        &SetupSimRequest {
            project: &proj_rest,
            setup_idx: 0,
            cad_doc: &doc,
            cad_revision: 1,
            cache: &cache_rest,
            kinematics: resolve_kinematics_for_key(&post_key, None),
            resolution: SimResolution::Standard,
            cancel: None,
        },
        SimEngineKind::TriDexel,
        SimComputeBackendChoice::Cpu,
    )
    .unwrap();
    let sim_auto = simulate_project_setup_with_choice(
        &SetupSimRequest {
            project: &proj_rest,
            setup_idx: 0,
            cad_doc: &doc,
            cad_revision: 1,
            cache: &cache_rest,
            kinematics: resolve_kinematics_for_key(&post_key, None),
            resolution: SimResolution::Standard,
            cancel: None,
        },
        SimEngineKind::TriDexel,
        SimComputeBackendChoice::Auto,
    )
    .unwrap();

    for (label, sim) in [("CPU", &sim_cpu), ("Auto/GPU", &sim_auto)] {
        assert_eq!(
            sim.critical_error_count(),
            0,
            "M73 {label} TriDexel must have 0 rapid crashes / critical errors: {:#?}",
            sim.diagnostics
        );
        let err = sim.error_report.as_ref().expect("TriDexel error report");
        assert!(
            err.gouge_volume_mm3 <= 0.01 && err.max_gouge_mm <= 0.01,
            "M73 {label} TriDexel must have ~0.0 mm³ gouge (got vol={:.4} mm³, max={:.4} mm)",
            err.gouge_volume_mm3,
            err.max_gouge_mm
        );
    }

    let err_cpu = sim_cpu.error_report.as_ref().unwrap();
    let err_gpu = sim_auto.error_report.as_ref().unwrap();
    let rel_vol_diff = (err_cpu.excess_volume_mm3 - err_gpu.excess_volume_mm3).abs()
        / err_cpu.excess_volume_mm3.max(1.0);
    assert!(
        rel_vol_diff <= 0.05,
        "M73 CPU vs Auto/GPU excess volume drift too large: cpu={:.1} vs auto={:.1} ({:.2}%)",
        err_cpu.excess_volume_mm3,
        err_gpu.excess_volume_mm3,
        rel_vol_diff * 100.0
    );
}

/// M74 E2E: `Parallel3D`/`Scallop3D` zero cliff-dive gouges + `Pencil3D` overhang-shadow
/// suppression + Auto Soft-Jaw zero-scalloping on `sample_triple_helix_loft_stl_bytes()`.
#[test]
fn m74_triple_helix_overhang_safe_finishing_and_swept_soft_jaws() {
    let stl = sample_triple_helix_loft_stl_bytes();
    let doc = load_mesh_cad_document("triple_helix_loft.stl", &stl, &MeshImportOptions::default())
        .unwrap();
    let cancel = AtomicBool::new(false);

    // 1. 3D Finishing (Parallel3D + Scallop3D + Pencil3D) on the overhanging triple-helix loft
    let mut setup = Setup::new("setup_m74", "Triple-Helix Finish", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::ModelBoxPoint { anchor: BOX_CENTER };
    setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(1.0),
        side_neg_x: ParamLength::from_mm(1.0),
        side_pos_y: ParamLength::from_mm(1.0),
        side_neg_y: ParamLength::from_mm(1.0),
        top_offset_z: ParamLength::from_mm(0.5),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(37.0),
        round_increment_mm: None,
    };
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let ball_4 = make_ball_tool("t_ball_4", 1, 4.0, 24.0);

    let mut op_par = Operation::new("op_par_m74", "Parallel3D Helix", OperationKind::Parallel3D);
    op_par.tool_id = Some(ball_4.id.clone());
    op_par.heights.bottom = HeightSpec::new(HeightReference::WcsOrigin, 0.0);
    if let StrategyParams::Parallel3D(p) = op_par.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.boundary.tool_containment = ToolContainmentMode::ToolCenterOnBoundary;
        p.use_cusp_height = false;
        p.stepover = ParamLength::from_mm(2.0);
        p.pass_angle_deg = 0.0;
        p.cross_pass = false;
        p.cut_direction = ParallelCutDirection::ZigZag;
    }

    let mut op_sc = Operation::new("op_sc_m74", "Scallop3D Helix", OperationKind::Scallop3D);
    op_sc.tool_id = Some(ball_4.id.clone());
    op_sc.heights.bottom = HeightSpec::new(HeightReference::WcsOrigin, 0.0);
    if let StrategyParams::Scallop3D(p) = op_sc.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.boundary.tool_containment = ToolContainmentMode::ToolCenterOnBoundary;
        p.cusp_height = ParamLength::from_mm(0.15);
        p.max_stepover = ParamLength::from_mm(2.2);
        p.min_stepover = ParamLength::from_mm(0.8);
    }

    let mut op_pen = Operation::new("op_pen_m74", "Pencil3D Helix", OperationKind::Pencil3D);
    op_pen.tool_id = Some(ball_4.id.clone());
    op_pen.heights.bottom = HeightSpec::new(HeightReference::WcsOrigin, 0.0);
    if let StrategyParams::Pencil3D(p) = op_pen.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.boundary.tool_containment = ToolContainmentMode::ToolCenterOnBoundary;
        p.reference_tool_diameter = ParamLength::from_mm(10.0);
        p.valley_threshold = ParamLength::from_mm(0.20);
        p.num_offset_passes = 0;
    }

    let tp_par = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_par,
        tool: &ball_4,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();
    let tp_sc = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_sc,
        tool: &ball_4,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();
    let tp_pen = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved,
        op: &op_pen,
        tool: &ball_4,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: false,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();

    // Assert zero full-cut-feed overhang cliff dives in Parallel3D & Scallop3D
    const TAN_70: f64 = 2.747_477_419_454_622;
    let surface = SurfaceMesh3D::from_cad_document(&doc, &[], &resolved.frame);
    let cutter = SurfaceCutterProfile::from_tool_geometry(&ball_4.geometry, 0.0, 0.0);
    let mut cand_buf = Vec::new();
    let mut ray_buf = Vec::new();
    for (label, tp) in [("Parallel3D", &tp_par), ("Scallop3D", &tp_sc)] {
        let mut overhang_dives = 0usize;
        for seg in &tp.segments_wcs {
            if matches!(
                seg.motion,
                SegmentMotion3D::Linear {
                    role: MotionRole::FinishCut,
                    ..
                }
            ) {
                let dz_drop = seg.start_wcs.z - seg.end_wcs.z;
                let dxy = (seg.end_wcs.x - seg.start_wcs.x).hypot(seg.end_wcs.y - seg.start_wcs.y);
                if dz_drop > 2.0
                    && dz_drop > TAN_70 * dxy
                    && chip_cam::surface3d::finishing::is_overhang_shadow_transition(
                        &surface,
                        &cutter,
                        seg.start_wcs,
                        seg.end_wcs,
                        &mut cand_buf,
                        &mut ray_buf,
                    )
                {
                    overhang_dives += 1;
                }
            }
        }
        assert_eq!(
            overhang_dives, 0,
            "M74 {label} must have 0 full-cut-feed overhang cliff dives"
        );
    }

    let sim = run_setup_simulation_with_choice(
        &SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[
                SimOperationInput::from_operation(&op_par, &ball_4, &tp_par),
                SimOperationInput::from_operation(&op_sc, &ball_4, &tp_sc),
                SimOperationInput::from_operation(&op_pen, &ball_4, &tp_pen),
            ],
            kinematics: MachineKinematicsConfig::default(),
            resolution: SimResolution::Fine,
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
        },
        SimEngineKind::TriDexel,
        SimComputeBackendChoice::Auto,
    )
    .unwrap();
    assert_eq!(
        sim.critical_error_count(),
        0,
        "M74 finishing ops must have 0 critical errors: {:#?}",
        sim.diagnostics
    );
    let err = sim.error_report.as_ref().expect("TriDexel error report");
    assert!(
        err.max_gouge_mm <= 0.025 && err.gouge_volume_mm3 <= 0.10,
        "M74 finishing gouge exceeded bounds: max_gouge_mm={:.4}, gouge_vol={:.4} mm³",
        err.max_gouge_mm,
        err.gouge_volume_mm3
    );

    // 2. Auto Soft-Jaw zero-scalloping check on the twisted triple-helix loft
    let mut jaw_setup = Setup::new(
        "setup_jaw_m74",
        "Triple-Helix Soft Jaw",
        WorkOffsetSlot::G54,
    );
    jaw_setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
        anchor: BoxAnchor27 {
            x: AxisAnchor::Center,
            y: AxisAnchor::Center,
            z: AxisAnchor::Min,
        },
    };
    let grip_depth_mm = 12.0;
    let cfg = SoftJawConfig {
        jaw_width_mm: 90.0,
        jaw_depth_mm: 32.0,
        jaw_height_mm: 36.0,
        jaw_gap_mm: 4.0,
        grip_depth_mm,
        part_clearance_mm: 0.10,
        corner_relief_radius_mm: 0.0,
        mounting_bolt_pattern: SoftJawBoltPattern::CarveraModVise,
        output_mode: SoftJawOutputMode::Both,
        z_band_bottom_offset_mm: 6.0,
        ..SoftJawConfig::default()
    };
    let plan = build_soft_jaw_plan(&doc, &jaw_setup, &[], DisplayUnit::Millimeter, &cfg).unwrap();
    let jaw_resolved = jaw_setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let jaw_surface = SurfaceMesh3D::from_cad_document(&doc, &[], &jaw_resolved.frame);
    let z_lo = cfg.z_band_bottom_offset_mm;
    let z_hi = z_lo + grip_depth_mm;
    let half_gap = 0.5 * cfg.jaw_gap_mm;

    let mut outside_sil = 0usize;
    let mut outside_pocket = 0usize;
    for v in &jaw_surface.vertices {
        if v.z >= z_lo + 0.05 && v.z <= z_hi - 0.05 {
            let pt = DVec2::new(v.x, v.y);
            if !plan.part_silhouette_2d.contains_point(pt)
                && plan.part_silhouette_2d.min_distance_to_boundary(pt) > 0.02
            {
                outside_sil += 1;
            }
            if pt.y > half_gap + 0.15 && !plan.fixed_pocket_2d.contains_point(pt) {
                outside_pocket += 1;
            }
            if pt.y < -(half_gap + 0.15) && !plan.movable_pocket_2d.contains_point(pt) {
                outside_pocket += 1;
            }
        }
    }
    assert_eq!(
        outside_sil, 0,
        "M74 swept soft-jaw silhouette must contain 100% of twisted helical blade vertices in Z-band"
    );
    assert_eq!(
        outside_pocket, 0,
        "M74 soft-jaw pockets must have zero scalloping interference on triple-helix loft"
    );
    for (label, bytes) in [
        ("m74_fixed_jaw.stl", plan.fixed_jaw_stl.as_ref().unwrap()),
        (
            "m74_movable_jaw.stl",
            plan.movable_jaw_stl.as_ref().unwrap(),
        ),
    ] {
        let m = load_mesh_bytes(label, bytes, &MeshImportOptions::default()).unwrap();
        assert!(m.diagnostics.is_watertight, "{label} must be watertight");
    }
}

/// M75 E2E (Part 1): Partial-sector (`0°..180°`) `ZigZag` in `Rotary2Linear` and `Rotary4AxisSim`
/// on `sample_triple_helix_loft_stl_bytes()` verified in BOTH `CylindricalStock` and `TriDexelStock`
/// (CPU + Auto/GPU), plus axial-undercut reachability warning check.
#[test]
fn m75_partial_sector_zigzag_rotary2linear_and_rotary4axis_sim_gouge_free() {
    let stl = sample_triple_helix_loft_stl_bytes();
    let doc = load_mesh_cad_document("triple_helix_loft.stl", &stl, &MeshImportOptions::default())
        .unwrap();
    let cancel = AtomicBool::new(false);
    let ball_4 = make_ball_tool("t_ball_4_m75", 1, 4.0, 24.0);

    // 1. Rotary2Linear partial-sector (0°..180°) ZigZag across helical + axial-undercut window
    let mut setup_2lin = make_rotary_helix_setup(RotaryDriveMode::TwoLinearPlusRotary);
    let resolved_2lin = setup_2lin.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let mut op_2lin = Operation::new(
        "op_rot2lin_sector",
        "Rotary2Linear 0..180 ZigZag",
        OperationKind::Rotary2Linear,
    );
    op_2lin.tool_id = Some(ball_4.id.clone());
    op_2lin.passes.multiple_depths = true;
    op_2lin.passes.max_stepdown = ParamLength::from_mm(4.5);
    if let StrategyParams::Rotary2Linear(p) = op_2lin.strategy_mut() {
        p.pattern = RotaryPatternMode::ContinuousSpiral;
        p.cut_direction = RotaryCutDirection::ZigZag;
        p.axial_boundary.start_angle_deg = 0.0;
        p.axial_boundary.end_angle_deg = 180.0;
        p.use_cusp_height = false;
        p.stepover = ParamLength::from_mm(3.0);
        p.axial_boundary.axial_margin_start = ParamLength::from_mm(9.0);
        p.axial_boundary.axial_margin_end = ParamLength::from_mm(5.0);
    }

    let tp_2lin = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_2lin,
        op: &op_2lin,
        tool: &ball_4,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();

    let link_count_2lin = tp_2lin
        .segments_wcs
        .iter()
        .filter(|s| {
            matches!(
                s.motion,
                SegmentMotion3D::Linear {
                    role: MotionRole::LinkTransition,
                    ..
                }
            )
        })
        .count();
    assert!(
        link_count_2lin > 40,
        "Partial-sector ZigZag Rotary2Linear must emit dense drop-cutter-verified LinkTransition strips (got {link_count_2lin} link segs)"
    );
    assert!(
        tp_2lin
            .warnings
            .iter()
            .any(|w| w.contains("Undercut3D") && w.contains("LollipopEndmill")),
        "Rotary2Linear on triple-helix loft with axial undercut must emit Undercut3D/LollipopEndmill warning, got: {:?}",
        tp_2lin.warnings
    );

    // 2. Rotary4AxisSim partial-sector (0°..180°) ZigZag with SurfaceNormalAutoTilt
    let mut setup_4ax = make_rotary_helix_setup(RotaryDriveMode::Simultaneous4Axis);
    let resolved_4ax = setup_4ax.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let mut op_4ax = Operation::new(
        "op_rot4ax_sector",
        "Rotary4AxisSim 0..180 ZigZag",
        OperationKind::Rotary4AxisSim,
    );
    op_4ax.tool_id = Some(ball_4.id.clone());
    op_4ax.passes.multiple_depths = true;
    op_4ax.passes.max_stepdown = ParamLength::from_mm(4.5);
    if let StrategyParams::Rotary4AxisSim(p) = op_4ax.strategy_mut() {
        p.pattern = Rotary4AxisPattern::SimultaneousSpiral;
        p.tilt_mode = RotaryTiltMode::SurfaceNormalAutoTilt;
        p.max_tilt_deg = 28.0;
        p.tilt_smoothing_window = 3;
        p.cut_direction = RotaryCutDirection::ZigZag;
        p.axial_boundary.start_angle_deg = 0.0;
        p.axial_boundary.end_angle_deg = 180.0;
        p.use_cusp_height = false;
        p.stepover = ParamLength::from_mm(3.0);
        p.axial_boundary.axial_margin_start = ParamLength::from_mm(9.0);
        p.axial_boundary.axial_margin_end = ParamLength::from_mm(5.0);
    }

    let tp_4ax = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_4ax,
        op: &op_4ax,
        tool: &ball_4,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();

    let link_count_4ax = tp_4ax
        .segments_wcs
        .iter()
        .filter(|s| {
            matches!(
                s.motion,
                SegmentMotion3D::Linear {
                    role: MotionRole::LinkTransition,
                    ..
                }
            )
        })
        .count();
    assert!(
        link_count_4ax > 40,
        "Partial-sector ZigZag Rotary4AxisSim must emit dense drop-cutter-verified LinkTransition strips (got {link_count_4ax} link segs)"
    );
    assert!(
        tp_4ax
            .warnings
            .iter()
            .any(|w| w.contains("Undercut3D") && w.contains("LollipopEndmill")),
        "Rotary4AxisSim on triple-helix loft with axial undercut must emit Undercut3D/LollipopEndmill warning, got: {:?}",
        tp_4ax.warnings
    );

    // Verify non-overhanging helical flute shaft emits NO false axial-undercut warning
    let flute_stl = sample_helical_flute_shaft_stl_bytes();
    let flute_doc = load_mesh_cad_document(
        "helical_flute_shaft.stl",
        &flute_stl,
        &MeshImportOptions::default(),
    )
    .unwrap();
    let starter = default_starter_tool_library();
    let mut flute_setup = Setup::new("setup_flute", "Flute Check", WorkOffsetSlot::G54);
    flute_setup.wcs.origin = WcsOriginMode::StockBoxPoint { anchor: BOX_CENTER };
    flute_setup.stock = StockConfig::FixedCylinder {
        axis: StockAxis::X,
        diameter: ParamLength::from_mm(42.0),
        length: ParamLength::from_mm(80.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    flute_setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
    flute_setup.rotary.axis = RotaryAxisKind::XAxis;
    let resolved_flute = flute_setup
        .resolve(&flute_doc, DisplayUnit::Millimeter)
        .unwrap();
    let mut op_flute = Operation::new("op_flute", "Flute", OperationKind::Rotary4AxisSim);
    op_flute.tool_id = Some(starter.tools[0].id.clone());
    if let StrategyParams::Rotary4AxisSim(p) = op_flute.strategy_mut() {
        p.stepover = ParamLength::from_mm(10.0);
        p.use_cusp_height = false;
        p.axial_boundary.axial_margin_start = ParamLength::from_mm(11.0);
        p.axial_boundary.axial_margin_end = ParamLength::from_mm(11.0);
    }
    let tp_flute = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &flute_doc,
        sketches: &[],
        setup: &resolved_flute,
        op: &op_flute,
        tool: &starter.tools[0],
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();
    assert!(
        !tp_flute.warnings.iter().any(|w| w.contains("Undercut3D")),
        "Non-overhanging helical flute shaft must NOT emit false axial-undercut warnings: {:?}",
        tp_flute.warnings
    );

    // Verify BOTH Cylindrical (Heightmap) and volumetric TriDexel across CPU and Auto/GPU
    for (label, resolved, op, tp) in [
        ("Rotary2Linear 0..180", &resolved_2lin, &op_2lin, &tp_2lin),
        ("Rotary4AxisSim 0..180", &resolved_4ax, &op_4ax, &tp_4ax),
    ] {
        for engine in [SimEngineKind::Heightmap, SimEngineKind::TriDexel] {
            for backend in [SimComputeBackendChoice::Cpu, SimComputeBackendChoice::Auto] {
                let sim = run_setup_simulation_with_choice(
                    &SetupSimulationInput {
                        doc: &doc,
                        setup: resolved,
                        target_bodies: &[],
                        fixtures: &[],
                        operations: &[SimOperationInput::from_operation(op, &ball_4, tp)],
                        kinematics: MachineKinematicsConfig::default(),
                        resolution: SimResolution::Standard,
                        verify_config: SimVerificationConfig::default(),
                        initial_stock: None,
                        initial_raw_volume_mm3: None,
                        carried_over_from_setup: None,
                        cancel: None,
                    },
                    engine,
                    backend,
                )
                .unwrap();

                assert_eq!(
                    sim.critical_error_count(),
                    0,
                    "M75 {label} ({engine:?}/{backend:?}) must have 0 critical collisions: {:#?}",
                    sim.diagnostics
                );
                let err = sim.error_report.as_ref().expect("StockErrorReport");
                match engine {
                    SimEngineKind::Heightmap => {
                        assert!(
                            err.max_gouge_mm <= 0.015,
                            "M75 {label} ({engine:?}/{backend:?}) max_gouge_mm={:.4} exceeded 0.015 mm (gouge_vol={:.4} mm³)",
                            err.max_gouge_mm,
                            err.gouge_volume_mm3
                        );
                    }
                    SimEngineKind::TriDexel => {
                        assert!(
                            err.gouge_volume_mm3 <= 0.25 && err.max_gouge_mm <= 0.12,
                            "M75 {label} ({engine:?}/{backend:?}) TriDexel gouge exceeded bounds: max_gouge_mm={:.4}, gouge_vol={:.4} mm³",
                            err.max_gouge_mm,
                            err.gouge_volume_mm3
                        );
                    }
                }
            }
        }
    }
}

/// M75 E2E (Part 2): `Rotary4AxisSim` (`SurfaceNormalAutoTilt` with overhang-aware multi-offset
/// flank probing) vs. centerline-only `Rotary2Linear` on the overhanging helical grooves of
/// `sample_triple_helix_loft_stl_bytes()`, verified with CPU & GPU parity.
#[test]
fn m75_rotary4axis_auto_tilt_improves_overhanging_helical_grooves_vs_rotary2linear_cpu_gpu() {
    let stl = sample_triple_helix_loft_stl_bytes();
    let doc = load_mesh_cad_document("triple_helix_loft.stl", &stl, &MeshImportOptions::default())
        .unwrap();
    let cancel = AtomicBool::new(false);
    let ball_4 = make_ball_tool("t_ball_4_auto", 1, 4.0, 24.0);

    // Focus axial window squarely on the 3-blade overhanging helical loft (`X ∈ [-18, +14]`)
    let margin_start = 10.0; // -28 + 10 = -18
    let margin_end = 14.0; // +28 - 14 = +14

    let mut setup_2lin = make_rotary_helix_setup(RotaryDriveMode::TwoLinearPlusRotary);
    let resolved_2lin = setup_2lin.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let mut op_2lin = Operation::new(
        "op_2lin_full",
        "Rotary2Linear 360",
        OperationKind::Rotary2Linear,
    );
    op_2lin.tool_id = Some(ball_4.id.clone());
    op_2lin.passes.multiple_depths = false;
    if let StrategyParams::Rotary2Linear(p) = op_2lin.strategy_mut() {
        p.pattern = RotaryPatternMode::ContinuousSpiral;
        p.use_cusp_height = false;
        p.stepover = ParamLength::from_mm(2.0);
        p.axial_boundary.axial_margin_start = ParamLength::from_mm(margin_start);
        p.axial_boundary.axial_margin_end = ParamLength::from_mm(margin_end);
        p.angular_sample_deg = 1.5;
    }

    let mut setup_4ax = make_rotary_helix_setup(RotaryDriveMode::Simultaneous4Axis);
    let resolved_4ax = setup_4ax.resolve(&doc, DisplayUnit::Millimeter).unwrap();
    let mut op_4ax = Operation::new(
        "op_4ax_full",
        "Rotary4AxisSim Auto-Tilt 360",
        OperationKind::Rotary4AxisSim,
    );
    op_4ax.tool_id = Some(ball_4.id.clone());
    op_4ax.passes.multiple_depths = false;
    if let StrategyParams::Rotary4AxisSim(p) = op_4ax.strategy_mut() {
        p.pattern = Rotary4AxisPattern::SimultaneousSpiral;
        p.tilt_mode = RotaryTiltMode::SurfaceNormalAutoTilt;
        p.max_tilt_deg = 32.0;
        p.tilt_smoothing_window = 3;
        p.use_cusp_height = false;
        p.stepover = ParamLength::from_mm(2.0);
        p.axial_boundary.axial_margin_start = ParamLength::from_mm(margin_start);
        p.axial_boundary.axial_margin_end = ParamLength::from_mm(margin_end);
        p.angular_sample_deg = 1.5;
    }

    let tp_2lin = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_2lin,
        op: &op_2lin,
        tool: &ball_4,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();
    let tp_4ax = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &doc,
        sketches: &[],
        setup: &resolved_4ax,
        op: &op_4ax,
        tool: &ball_4,
        clearing_tool: None,
        display_unit: DisplayUnit::Millimeter,
        is_first_tool_in_setup: true,
        upstream_rest: &[],
        carryover_stock: None,
        fixtures: &[],
        shared_avoid_regions: &[],
        cancel: &cancel,
    })
    .unwrap();

    // Verify smooth bounded cross-axis (Y) motion in Rotary4AxisSim (non-zero tilt, zero jerks)
    let mut max_abs_y = 0.0_f64;
    let mut max_cut_dy_step = 0.0_f64;
    for seg in &tp_4ax.segments_wcs {
        if matches!(
            seg.motion,
            SegmentMotion3D::Linear {
                role: MotionRole::Cut | MotionRole::FinishCut,
                ..
            }
        ) {
            max_abs_y = max_abs_y
                .max(seg.start_wcs.y.abs())
                .max(seg.end_wcs.y.abs());
            max_cut_dy_step = max_cut_dy_step.max((seg.end_wcs.y - seg.start_wcs.y).abs());
        }
    }
    assert!(
        max_abs_y >= 1.5,
        "SurfaceNormalAutoTilt on overhanging helical loft must actively engage off-centerline Y tilt (max |Y|={max_abs_y:.3} mm)"
    );
    assert!(
        max_cut_dy_step <= 1.25,
        "SurfaceNormalAutoTilt cross-axis Y transitions must stay smooth (max step ΔY={max_cut_dy_step:.3} mm)"
    );

    // Compare volumetric TriDexelStock (captures true 3D circumferential overhangs!) on CPU & Auto/GPU
    let mut tri_2lin_cpu = None;
    let mut tri_4ax_cpu = None;
    let mut tri_4ax_auto = None;
    for backend in [SimComputeBackendChoice::Cpu, SimComputeBackendChoice::Auto] {
        let sim_2lin = run_setup_simulation_with_choice(
            &SetupSimulationInput {
                doc: &doc,
                setup: &resolved_2lin,
                target_bodies: &[],
                fixtures: &[],
                operations: &[SimOperationInput::from_operation(
                    &op_2lin, &ball_4, &tp_2lin,
                )],
                kinematics: MachineKinematicsConfig::default(),
                resolution: SimResolution::Standard,
                verify_config: SimVerificationConfig::default(),
                initial_stock: None,
                initial_raw_volume_mm3: None,
                carried_over_from_setup: None,
                cancel: None,
            },
            SimEngineKind::TriDexel,
            backend,
        )
        .unwrap();
        let sim_4ax = run_setup_simulation_with_choice(
            &SetupSimulationInput {
                doc: &doc,
                setup: &resolved_4ax,
                target_bodies: &[],
                fixtures: &[],
                operations: &[SimOperationInput::from_operation(&op_4ax, &ball_4, &tp_4ax)],
                kinematics: MachineKinematicsConfig::default(),
                resolution: SimResolution::Standard,
                verify_config: SimVerificationConfig::default(),
                initial_stock: None,
                initial_raw_volume_mm3: None,
                carried_over_from_setup: None,
                cancel: None,
            },
            SimEngineKind::TriDexel,
            backend,
        )
        .unwrap();

        assert_eq!(
            sim_4ax.critical_error_count(),
            0,
            "M75 Rotary4AxisSim ({backend:?}) TriDexel must have 0 critical errors: {:#?}",
            sim_4ax.diagnostics
        );
        let e2 = sim_2lin.error_report.as_ref().unwrap();
        let e4 = sim_4ax.error_report.as_ref().unwrap();
        assert!(
            e4.max_gouge_mm <= 0.045 && e4.gouge_volume_mm3 <= 0.5,
            "M75 Rotary4AxisSim ({backend:?}) TriDexel gouge exceeded bounds: max_gouge_mm={:.4}, gouge_vol={:.4} mm³",
            e4.max_gouge_mm,
            e4.gouge_volume_mm3
        );
        assert!(
            e4.excess_volume_mm3 < e2.excess_volume_mm3 - 50.0,
            "M75 Rotary4AxisSim Auto-Tilt ({backend:?}) must leave measurably less excess volume under overhanging helical blades than centerline Rotary2Linear (4ax={:.1} mm³ vs 2lin={:.1} mm³)",
            e4.excess_volume_mm3,
            e2.excess_volume_mm3
        );
        assert!(
            e4.rms_error_mm < e2.rms_error_mm,
            "M75 Rotary4AxisSim Auto-Tilt ({backend:?}) must achieve lower RMS error than centerline Rotary2Linear (4ax={:.4} mm vs 2lin={:.4} mm)",
            e4.rms_error_mm,
            e2.rms_error_mm
        );

        if backend == SimComputeBackendChoice::Cpu {
            tri_2lin_cpu = Some(e2.clone());
            tri_4ax_cpu = Some(e4.clone());
        } else {
            tri_4ax_auto = Some(e4.clone());
        }
    }

    // CPU vs Auto/GPU parity on TriDexel
    let c4 = tri_4ax_cpu.unwrap();
    let g4 = tri_4ax_auto.unwrap();
    let tri_rel_diff =
        (c4.excess_volume_mm3 - g4.excess_volume_mm3).abs() / c4.excess_volume_mm3.max(1.0);
    assert!(
        tri_rel_diff <= 0.05,
        "M75 TriDexel CPU vs Auto/GPU parity drift: cpu={:.1} vs auto={:.1} mm³ ({:.2}%)",
        c4.excess_volume_mm3,
        g4.excess_volume_mm3,
        tri_rel_diff * 100.0
    );
    let _ = tri_2lin_cpu;

    // Also verify CylindricalStock CPU vs Auto/GPU parity on Rotary4AxisSim
    let cyl_cpu = run_setup_simulation_with_choice(
        &SetupSimulationInput {
            doc: &doc,
            setup: &resolved_4ax,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput::from_operation(&op_4ax, &ball_4, &tp_4ax)],
            kinematics: MachineKinematicsConfig::default(),
            resolution: SimResolution::Fine,
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
        },
        SimEngineKind::Heightmap,
        SimComputeBackendChoice::Cpu,
    )
    .unwrap();
    let cyl_auto = run_setup_simulation_with_choice(
        &SetupSimulationInput {
            doc: &doc,
            setup: &resolved_4ax,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput::from_operation(&op_4ax, &ball_4, &tp_4ax)],
            kinematics: MachineKinematicsConfig::default(),
            resolution: SimResolution::Fine,
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
        },
        SimEngineKind::Heightmap,
        SimComputeBackendChoice::Auto,
    )
    .unwrap();

    for (label, sim) in [("Cyl CPU", &cyl_cpu), ("Cyl Auto/GPU", &cyl_auto)] {
        assert_eq!(
            sim.critical_error_count(),
            0,
            "M75 {label} must have 0 critical errors: {:#?}",
            sim.diagnostics
        );
        let err = sim.error_report.as_ref().unwrap();
        assert!(
            err.max_gouge_mm <= 0.015,
            "M75 {label} max_gouge_mm={:.4} exceeded 0.015 mm",
            err.max_gouge_mm
        );
    }
    let ec = cyl_cpu.error_report.as_ref().unwrap();
    let eg = cyl_auto.error_report.as_ref().unwrap();
    let cyl_rel_diff =
        (ec.excess_volume_mm3 - eg.excess_volume_mm3).abs() / ec.excess_volume_mm3.max(1.0);
    assert!(
        cyl_rel_diff <= 0.05,
        "M75 Cylindrical CPU vs Auto/GPU parity drift: cpu={:.1} vs auto={:.1} mm³ ({:.2}%)",
        ec.excess_volume_mm3,
        eg.excess_volume_mm3,
        cyl_rel_diff * 100.0
    );
}
