#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::sync::atomic::AtomicBool;

use chip_cad::CadDocument;
use chip_cam::{
    ClEvent, DrillBoreParams, FixedBoxXyAlign, FixedBoxZAlign, GeometrySelectionItem,
    HeightReference, HeightSpec, HoleMachiningMode, MachinabilityCheckKind, MachinabilitySeverity,
    MotionRole, Operation, OperationGenRequest, OperationKind, StockConfig, StrategyParams,
    apply_machinability_fix, generate_operation_toolpath_with_rest,
};
use chip_math::{DisplayUnit, ParamLength};
use chip_project::{
    ProjectDocument, SetupSimRequest, ToolpathCache, analyze_project_machinability,
    generate_project_blocking_in_dir, simulate_project_setup_with_choice,
};
use chip_sim::{MachineKinematicsConfig, SimComputeBackendChoice, SimEngineKind, SimResolution};
use chip_sketch::{SketchEntity, SketchEntityEntry, SketchLayer};
use chip_tools::{
    ALUMINUM_6061_T6_ID, ALUMINUM_6063_T5_ID, ProjectMaterialSelection,
    builtin_carvera_air_machine, builtin_carvera_z1_machine, builtin_milo_1_5_machine,
};

#[test]
fn m94_bartrim_helical_bore_inner_arc_comp_chip_clear_and_tridexel_crash_free() {
    let machine = builtin_carvera_air_machine();
    let t2 = machine
        .tools
        .iter()
        .find(|t| t.tool_number == 2)
        .unwrap()
        .clone();

    let mut proj = ProjectDocument::new("BarTrim Helical Bore Regression");
    proj.machine_id = Some(machine.id.clone());
    proj.workpiece_material = Some(ProjectMaterialSelection::builtin(ALUMINUM_6063_T5_ID));
    proj.tools = vec![t2.clone()];

    let mut sk = SketchLayer::new("sk_bore", "Ø5 Bore", "setup_1");
    sk.entities.push(SketchEntityEntry {
        id: "hole_5mm".to_string(),
        name: "Ø5.0 mm Through-Hole".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::new_circle(
            ParamLength::from_mm(0.0),
            ParamLength::from_mm(0.0),
            ParamLength::from_mm(5.0),
        ),
    });
    proj.sketches.push(sk);

    proj.setups[0].stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(30.0),
        depth_y: ParamLength::from_mm(30.0),
        height_z: ParamLength::from_mm(20.2),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::OffsetFromTop {
            top_allowance: ParamLength::from_mm(0.0),
        },
    };

    // Newly created DrillBore defaults bore_inner_arc_feed_comp = true
    let mut op_bore = Operation::new(
        "op_bore",
        "Ø5 x 20.2 Through-Bore",
        OperationKind::DrillBore,
    );
    assert!(matches!(
        &op_bore.strategy,
        Some(StrategyParams::DrillBore(p)) if p.bore_inner_arc_feed_comp && p.bore_chip_clear_depth.is_none()
    ));

    // Legacy TOML deserialization defaults bore_inner_arc_feed_comp = false & bore_chip_clear_depth = None
    let legacy_params: DrillBoreParams = toml::from_str("mode = \"helical_bore\"\n").unwrap();
    assert!(!legacy_params.bore_inner_arc_feed_comp);
    assert!(legacy_params.bore_chip_clear_depth.is_none());

    // 1. Reproduce BarTrim failure: uncompensated 309 mm/min + bore_finish_pass = true
    op_bore.tool_id = Some(t2.id.clone());
    op_bore.preset_id = Some("carvera_al6061_deep".to_string());
    op_bore.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_bore.heights.bottom = HeightSpec::new(HeightReference::StockTop, -20.2);
    op_bore
        .selections
        .push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sk_bore".to_string(),
            entity_id: "hole_5mm".to_string(),
            reverse: false,
            side_flip: false,
        });
    if let StrategyParams::DrillBore(p) = op_bore.strategy_mut() {
        p.mode = HoleMachiningMode::HelicalBore;
        p.bore_finish_pass = true;
        p.bore_inner_arc_feed_comp = false;
        p.bore_chip_clear_depth = None;
    }
    proj.setups[0].operations.push(op_bore);

    let cad = CadDocument::empty();
    let bad_report = analyze_project_machinability(&proj, &machine, Some(&cad));
    assert_eq!(bad_report.severity, MachinabilitySeverity::Critical);
    let bad_findings = &bad_report.setups[0].operations[0].findings;
    assert!(bad_findings.iter().any(
        |f| f.kind == MachinabilityCheckKind::HelicalInnerArcOverload
            && f.severity == MachinabilitySeverity::Critical
    ));
    assert!(bad_findings.iter().any(
        |f| f.kind == MachinabilityCheckKind::DeepBoreFullDepthFinish
            && f.severity == MachinabilitySeverity::Critical
    ));
    assert!(
        bad_findings
            .iter()
            .any(|f| f.kind == MachinabilityCheckKind::DeepHoleChipEvacuation
                && f.severity == MachinabilitySeverity::Warning)
    );

    // 2. Tune via 1-click fixes: bore_finish_pass = false, bore_inner_arc_feed_comp = true, bore_chip_clear_depth = 10 mm
    for f in bad_findings {
        if let Some(fix) = &f.suggested_fix {
            apply_machinability_fix(&mut proj.setups[0].operations[0], fix);
        }
    }
    if let StrategyParams::DrillBore(p) = proj.setups[0].operations[0].strategy_mut() {
        assert!(!p.bore_finish_pass);
        assert!(p.bore_inner_arc_feed_comp);
        assert!(p.bore_chip_clear_depth.as_ref().unwrap().value_mm > 5.0);
        p.bore_chip_clear_depth = Some(ParamLength::from_mm(10.0));
    }

    let tuned_report = analyze_project_machinability(&proj, &machine, Some(&cad));
    assert_eq!(
        tuned_report.critical_count, 0,
        "tuned bore must have 0 critical findings: {:?}",
        tuned_report.setups[0].operations[0].findings
    );
    assert_eq!(
        tuned_report.warning_count, 0,
        "tuned bore must have 0 warning findings: {:?}",
        tuned_report.setups[0].operations[0].findings
    );
    assert_eq!(tuned_report.severity, MachinabilitySeverity::Ok);

    // 3. Verify emitted helical bore CL-data has exact inner-arc feed compensation (±5% of target wall chipload)
    //    AND intermediate Z-staged chip-clearing retracts to r_plane_z!
    let rs = proj.setups[0]
        .resolve_with_sketches(&cad, &proj.sketches, DisplayUnit::Millimeter)
        .unwrap();
    let cancel = AtomicBool::new(false);
    let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc: &cad,
        sketches: &proj.sketches,
        setup: &rs,
        op: &proj.setups[0].operations[0],
        tool: &t2,
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

    let cutting = proj.setups[0].operations[0].effective_cutting_params(&t2);
    let r_hole = 2.5_f64;
    let r_tool = 0.5 * t2.geometry.cutting_diameter_mm();
    let r_orbit = r_hole - r_tool;
    let target_wall_chipload = cutting.cutting_feed_mm_min / cutting.spindle_rpm;
    let mut saw_cut_arc = false;
    let mut saw_ramp_arc = false;
    let mut chip_clear_retracts = 0usize;

    for ev in &tp.events {
        match *ev {
            ClEvent::ArcMove {
                role: MotionRole::Cut,
                feed_mm_min,
                ..
            } => {
                saw_cut_arc = true;
                let actual_wall_feed = feed_mm_min * (r_hole / r_orbit);
                let actual_wall_chipload = actual_wall_feed / cutting.spindle_rpm;
                let rel_err =
                    ((actual_wall_chipload - target_wall_chipload) / target_wall_chipload).abs();
                assert!(
                    rel_err < 0.05,
                    "wall chipload {actual_wall_chipload:.5} drifted >5% from target {target_wall_chipload:.5}"
                );
            }
            ClEvent::ArcMove {
                role: MotionRole::Ramp,
                feed_mm_min,
                ..
            } => {
                saw_ramp_arc = true;
                let expected_ramp = cutting.ramp_feed_mm_min * (r_orbit / r_hole).clamp(0.20, 1.0);
                assert!(
                    (feed_mm_min - expected_ramp).abs() < 1e-3,
                    "helical ramp feed {feed_mm_min:.3} != expected {expected_ramp:.3}"
                );
            }
            ClEvent::RapidMove { end_wcs } => {
                // Intermediate retracts to r_plane_z (`Z = 2.0 mm`) at hole center (not initial start_xy)
                if end_wcs.z > 0.5 && end_wcs.z < 4.5 {
                    chip_clear_retracts += 1;
                }
            }
            _ => {}
        }
    }
    assert!(saw_cut_arc && saw_ramp_arc);
    // 1 initial plunge-to-r_plane + 2 intermediate stages * (retract to r_plane + move to start_xy at r_plane) >= 5 rapid moves at r_plane_z!
    assert!(
        chip_clear_retracts >= 5,
        "20.2 mm bore with 10 mm chip-clear depth must emit >= 2 intermediate retracts to r_plane_z, got {chip_clear_retracts} r-plane rapids"
    );

    // 4. Verify tuned staged helical bore is 100% crash-free in `chip-sim` Tri-Dexel!
    let mut cache = ToolpathCache::new(16);
    let batch = generate_project_blocking_in_dir(&mut proj, &cad, None, &mut cache, 1);
    assert!(batch.errors.is_empty(), "gen errors: {:?}", batch.errors);

    let sim = simulate_project_setup_with_choice(
        &SetupSimRequest {
            project: &proj,
            setup_idx: 0,
            cad_doc: &cad,
            cad_revision: 1,
            cache: &cache,
            kinematics: MachineKinematicsConfig::default(),
            resolution: SimResolution::Standard,
            cancel: None,
        },
        SimEngineKind::TriDexel,
        SimComputeBackendChoice::Cpu,
    )
    .unwrap();
    assert_eq!(
        sim.critical_error_count(),
        0,
        "Tri-Dexel simulation of tuned staged helical bore must be crash-free, got {:?}",
        sim.diagnostics
    );
    assert!(sim.removed_volume_mm3 > 300.0);
}

#[test]
fn m94_project_machinability_bartrim_facing_and_carvera_z1_vs_milo() {
    let air = builtin_carvera_air_machine();
    let z1 = builtin_carvera_z1_machine();
    let milo = builtin_milo_1_5_machine();

    // 1. BarTrim Facing: 4.04 x 0.50 -> Critical, 2.02 x 0.30 -> Ok
    let t2 = air
        .tools
        .iter()
        .find(|t| t.tool_number == 2)
        .unwrap()
        .clone();
    let mut proj = ProjectDocument::new("BarTrim Facing");
    proj.machine_id = Some(air.id.clone());
    proj.workpiece_material = Some(ProjectMaterialSelection::builtin(ALUMINUM_6063_T5_ID));
    proj.tools = vec![t2.clone()];
    proj.setups[0].stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(80.0),
        depth_y: ParamLength::from_mm(25.0),
        height_z: ParamLength::from_mm(20.2),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::OffsetFromTop {
            top_allowance: ParamLength::from_mm(0.0),
        },
    };

    let mut op_face = Operation::new("op_face", "Face Stock", OperationKind::Facing);
    op_face.tool_id = Some(t2.id.clone());
    op_face.preset_id = Some("carvera_al6061_deep".to_string());
    op_face.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_face.heights.bottom = HeightSpec::new(HeightReference::StockTop, -8.08);
    op_face.passes.max_stepdown = ParamLength::from_mm(4.04);
    if let StrategyParams::Facing(p) = op_face.strategy_mut() {
        p.stepover = ParamLength::from_mm(0.50);
    }
    proj.setups[0].operations.push(op_face);

    let rep_bad = analyze_project_machinability(&proj, &air, None);
    assert_eq!(rep_bad.severity, MachinabilitySeverity::Critical);

    proj.setups[0].operations[0].passes.max_stepdown = ParamLength::from_mm(2.02);
    if let StrategyParams::Facing(p) = proj.setups[0].operations[0].strategy_mut() {
        p.stepover = ParamLength::from_mm(0.30);
    }
    let rep_ok = analyze_project_machinability(&proj, &air, None);
    assert_eq!(rep_ok.severity, MachinabilitySeverity::Ok);

    // 2. Heavy Ø6 mm Slotting: stalls Carvera Z1 (150 W), passes Milo (1.5 kW)
    let em6 = milo
        .tools
        .iter()
        .find(|t| t.tool_number == 1)
        .unwrap()
        .clone();
    let mut slot_proj = ProjectDocument::new("Heavy Slotting");
    slot_proj.workpiece_material = Some(ProjectMaterialSelection::builtin(ALUMINUM_6061_T6_ID));
    slot_proj.tools = vec![em6.clone()];
    let mut op_slot = Operation::new("op_slot", "Ø6 Full Slot", OperationKind::Contour2D);
    op_slot.tool_id = Some(em6.id.clone());
    op_slot.feed_overrides.spindle_rpm = Some(12_000.0);
    op_slot.feed_overrides.cutting_feed_mm_min = Some(800.0);
    op_slot.passes.max_stepdown = ParamLength::from_mm(3.0);
    slot_proj.setups[0].operations.push(op_slot);

    let rep_z1 = analyze_project_machinability(&slot_proj, &z1, None);
    let rep_milo = analyze_project_machinability(&slot_proj, &milo, None);
    assert_eq!(rep_z1.severity, MachinabilitySeverity::Critical);
    assert!(
        rep_z1.setups[0].operations[0]
            .findings
            .iter()
            .any(|f| f.kind == MachinabilityCheckKind::SpindlePowerStall
                && f.severity == MachinabilitySeverity::Critical)
    );
    assert!(
        !rep_milo.setups[0].operations[0]
            .findings
            .iter()
            .any(|f| f.kind == MachinabilityCheckKind::SpindlePowerStall)
    );
}
