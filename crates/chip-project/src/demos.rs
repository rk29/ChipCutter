use chip_cad::{
    CadDocument, OrientedEdgeUse, SurfaceKind, TopologyGraph, find_same_diameter_holes,
};
use chip_cam::{
    AdaptiveClearingParams, BoxAnchor27, ChamferSourceMode, ChamferTraceMode, ContourCompensation,
    FixedBoxXyAlign, FixedBoxZAlign, GeometrySelectionItem, HeightReference, HeightSpec,
    HoleMachiningMode, Operation, OperationKind, PocketStrategyKind, RestMachiningConfig,
    RestMachiningMode, Setup, StockConfig, StrategyParams, TabProfileKind, VCarveClearingKind,
    VCarveInlayMode, WcsOriginMode, WorkOffsetSlot,
};
use chip_math::{DisplayUnit, EvalContext, ParamLength, approx_eq};
use chip_sketch::{
    DxfImportOptions, HersheyFontKind, PointPatternSpec, RectAnchorMode, SketchEntity,
    SketchEntityEntry, SketchFontSource, SketchLayer, SketchTextSpec, TextHAlign, TextVAlign,
    import_dxf_bytes, sample_router_panel_dxf,
};
use chip_tools::{
    CoolantMode, CutterGeometry, CuttingPreset, SpindleDirection, SurfaceSpeedPresetSpec, Tool,
    ToolId,
};

use crate::document::{ModelEntry, ProjectDocument};

/// Builds a seeded Milestone 2 **Multi-Feature Bracket** [`ProjectDocument`] (`Center Cavity Pocket` + `4x Ø6mm Holes`).
pub fn build_multi_feature_bracket_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Multi-Feature Bracket");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        unit_override: None,
    }];

    if let Some(t1) = available_tools.first() {
        proj.ensure_tool_embedded(t1);
    }
    if let Some(drill) = available_tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::TwistDrill { .. }))
    {
        proj.ensure_tool_embedded(drill);
    }

    if let Some(pkt_floor) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && (origin_mm.z - 12.0).abs() < 0.05 && !f.inner_loops.is_empty()
        )
    }) && let Some(bind) = doc.bind_face(pkt_floor.id)
    {
        let mut op_pkt = Operation::new(
            "op_1_pocket",
            "Center Cavity Pocket",
            OperationKind::Pocket2D,
        );
        if let Some(t1) = proj.tools.first() {
            op_pkt.tool_id = Some(t1.id.clone());
            op_pkt.preset_id = t1.presets.first().map(|p| p.id.clone());
        }
        op_pkt
            .selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        proj.setups[0].operations.push(op_pkt);
    }

    if let Some(r3_seed) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Cylinder {
                radius_mm,
                is_internal: true,
                ..
            } if (radius_mm - 3.0).abs() < 1e-2
        )
    }) {
        let mut op_drill = Operation::new(
            "op_2_holes",
            "4x Ø6mm Mounting Holes",
            OperationKind::DrillBore,
        );
        if let Some(drill) = proj
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::TwistDrill { .. }))
        {
            op_drill.tool_id = Some(drill.id.clone());
            op_drill.preset_id = drill.presets.first().map(|p| p.id.clone());
        }
        for fid in find_same_diameter_holes(doc, r3_seed.id, 1e-3, 1e-3) {
            if let Some(b) = doc.bind_face(fid) {
                op_drill
                    .selections
                    .push(GeometrySelectionItem::CylindricalHole { binding: b });
            }
        }
        proj.setups[0].operations.push(op_drill);
    }

    proj
}

/// Builds the canonical 8-operation **CAM Torture Block** [`ProjectDocument`] shared by `chip-cli`
/// golden tests and `chip-gui` startup demo seeding.
pub fn build_cam_torture_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("CAM Torture Block");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        unit_override: None,
    }];

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

    let flat_6mm = proj
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. }
                if (diameter_mm - 6.0).abs() < 0.05
            )
        })
        .cloned();
    let face_38mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FaceMill { .. }))
        .cloned();
    let drill_6mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::TwistDrill { .. }))
        .cloned();
    let chamf_90 = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
        .cloned();

    if let Some(fm) = &face_38mm {
        let mut op = Operation::new("op_1_face", "Face Stock Top", OperationKind::Facing);
        op.tool_id = Some(fm.id.clone());
        op.preset_id = fm.presets.first().map(|p| p.id.clone());
        proj.setups[0].operations.push(op);
    }

    // Plateau Deck around Z=22 Boss Pad (`Z = 22 -> 20 mm`)
    if let Some(plat_face) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && (origin_mm.z - 20.0).abs() < 0.05 && f.area_mm2 > 1500.0
        )
    }) && let Some(bind) = doc.bind_face(plat_face.id)
        && let Some(em) = &flat_6mm
    {
        let mut op = Operation::new(
            "op_2_plateau",
            "Plateau Deck Around Boss",
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        proj.setups[0].operations.push(op);
    }

    // Open Shoulder Step (`Z = 14.0 mm`)
    if let Some(step_face) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && (origin_mm.z - 14.0).abs() < 0.05
        )
    }) && let Some(bind) = doc.bind_face(step_face.id)
        && let Some(em) = &flat_6mm
    {
        let mut op = Operation::new(
            "op_3_open_step",
            "Open Shoulder Step",
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        proj.setups[0].operations.push(op);
    }

    // Island Cavity Pocket (`Z = 12.0 mm` with Ø10mm raised island)
    if let Some(pkt_face) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && (origin_mm.z - 12.0).abs() < 0.05
        )
    }) && let Some(bind) = doc.bind_face(pkt_face.id)
        && let Some(em) = &flat_6mm
    {
        let mut op = Operation::new(
            "op_4_island_pocket",
            "Island Cavity Pocket",
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        proj.setups[0].operations.push(op);
    }

    // Helical Bore on Ø18mm Counterbore
    if let Some(r9_bore) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Cylinder {
                radius_mm,
                is_internal: true,
                ..
            } if (radius_mm - 9.0).abs() < 0.05
        )
    }) && let Some(bind) = doc.bind_face(r9_bore.id)
        && let Some(em) = &flat_6mm
    {
        let mut op = Operation::new(
            "op_5_helical_bore",
            "Ø18mm Helical Counterbore",
            OperationKind::DrillBore,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        if let StrategyParams::DrillBore(p) = op.strategy_mut() {
            p.mode = HoleMachiningMode::HelicalBore;
        }
        op.selections
            .push(GeometrySelectionItem::CylindricalHole { binding: bind });
        proj.setups[0].operations.push(op);
    }

    // Outer Contour + Triangular Holding Tabs (`T1` stays on `6mm` flat endmill before tool change!)
    if let Some(bot_face) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z < -0.99 && origin_mm.z.abs() < 0.05
        )
    }) && let Some(outer_lid) = bot_face.outer_loop
        && let Some(bind) = doc.bind_loop(outer_lid)
        && let Some(em) = &flat_6mm
    {
        let mut op = Operation::new(
            "op_6_outer_contour",
            "Outer Profile + Tabs",
            OperationKind::Contour2D,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.heights.top = HeightSpec::new(HeightReference::ModelTop, -2.0);
        op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
        op.selections.push(GeometrySelectionItem::ClosedLoop {
            binding: bind,
            reverse: false,
            side_flip: false,
        });
        if let StrategyParams::Contour2D(p) = op.strategy_mut() {
            p.tabs.enabled = true;
            p.tabs.profile = TabProfileKind::Triangular {
                ramp_angle_deg: 25.0,
            };
        }
        proj.setups[0].operations.push(op);
    }

    // 4x Ø6mm Shelf Through-Holes (`T6` Drill)
    if let Some(r3_seed) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Cylinder {
                radius_mm,
                is_internal: true,
                ..
            } if (radius_mm - 3.0).abs() < 0.05
        )
    }) && let Some(dr) = &drill_6mm
    {
        let mut op = Operation::new(
            "op_7_shelf_holes",
            "4x Shelf Through-Holes",
            OperationKind::DrillBore,
        );
        op.tool_id = Some(dr.id.clone());
        op.preset_id = dr.presets.first().map(|p| p.id.clone());
        for fid in find_same_diameter_holes(doc, r3_seed.id, 1e-3, 1e-3) {
            if let Some(b) = doc.bind_face(fid) {
                op.selections
                    .push(GeometrySelectionItem::CylindricalHole { binding: b });
            }
        }
        proj.setups[0].operations.push(op);
    }

    // Step Edge Chamfer (`T5` 90° Chamfer Mill)
    if let Some(step_edge) = doc.edges.iter().find(|e| {
        (e.aabb_mm.min.z - 20.0).abs() < 0.05
            && (e.aabb_mm.max.z - 20.0).abs() < 0.05
            && (e.aabb_mm.min.x - 76.0).abs() < 0.05
            && (e.aabb_mm.max.x - 76.0).abs() < 0.05
    }) && let Some(b) = doc.bind_edge(step_edge.id)
        && let Some(ch) = &chamf_90
    {
        let mut op = Operation::new(
            "op_8_step_chamfer",
            "Step Edge Chamfer",
            OperationKind::ChamferTrace,
        );
        op.tool_id = Some(ch.id.clone());
        op.preset_id = ch.presets.first().map(|p| p.id.clone());
        op.selections.push(GeometrySelectionItem::OpenChain {
            body_id: doc.bodies[0].id,
            start_edge: b.clone(),
            end_edge: b,
            resolved_edges: vec![OrientedEdgeUse {
                edge_id: step_edge.id,
                forward: true,
            }],
            reverse: false,
            side_flip: true,
        });
        if let StrategyParams::ChamferTrace(p) = op.strategy_mut() {
            p.mode = ChamferTraceMode::EdgeChamfer;
            p.chamfer_source = ChamferSourceMode::SharpEdge {
                width: ParamLength::from_mm(0.5),
            };
            p.tip_offset = ParamLength::from_mm(0.8);
        }
        proj.setups[0].operations.push(op);
    }

    proj
}

/// Builds a Milestone 6 **2D Adaptive + Rest Machining** [`ProjectDocument`] on the CAM Torture Block:
/// 1. `Op 1` (`Ø10 mm` 3FL Flat Endmill): Deep-axial **2D Adaptive Roughing** on the Island Cavity (`Z = 12.0 mm`).
/// 2. `Op 2` (`Ø3 mm` 3FL Flat Endmill): **2D Adaptive Rest Machining** (`FromPrecedingOperations`) cleaning out
///    the tight pocket corners and narrow island-wall channel skipped by the `Ø10 mm` rougher.
/// 3. `Op 3` (`Ø6 mm` 3FL Flat Endmill): **Open Shoulder 2D Adaptive** entering horizontally from outside-stock air.
pub fn build_adaptive_rest_demo_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Adaptive & Rest Machining Demo");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        unit_override: None,
    }];

    // T10: Ø10.0 mm 3FL Rougher + T8: Ø3.0 mm 3FL Rest Endmill + starter T1 Ø6.0 mm
    let t10_rough = Tool {
        id: ToolId("t10_10mm_adaptive_3fl".to_string()),
        tool_number: 10,
        length_offset_number: 10,
        diameter_offset_number: 10,
        name: "10mm 3FL Carbide Adaptive Rougher".to_string(),
        vendor: Some("Generic Carbide".to_string()),
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::AirBlast,
        stickout_length_mm: 30.0,
        geometry: CutterGeometry::FlatEndmill {
            diameter_mm: 10.0,
            flute_length_mm: 22.0,
            overall_length_mm: 65.0,
            shank_diameter_mm: 10.0,
            flutes: 3,
        },
        presets: vec![
            CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
                id: "al6061_adaptive",
                name: "Aluminum 6061 - Adaptive Roughing",
                material: "Aluminum 6061",
                diameter_mm: 10.0,
                flutes: 3,
                surface_speed_m_min: 280.0,
                chipload_mm_tooth: 0.055,
                default_stepdown_mm: 12.0,
                default_stepover_mm: 2.0,
                coolant_override: Some(CoolantMode::AirBlast),
            })
            .with_optimal_load_mm(2.0),
        ],
    };

    let t8_rest = Tool {
        id: ToolId("t8_3mm_rest_3fl".to_string()),
        tool_number: 8,
        length_offset_number: 8,
        diameter_offset_number: 8,
        name: "3mm 3FL Carbide Rest Endmill".to_string(),
        vendor: Some("Generic Carbide".to_string()),
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::AirBlast,
        stickout_length_mm: 18.0,
        geometry: CutterGeometry::FlatEndmill {
            diameter_mm: 3.0,
            flute_length_mm: 12.0,
            overall_length_mm: 45.0,
            shank_diameter_mm: 3.0,
            flutes: 3,
        },
        presets: vec![
            CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
                id: "al6061_rest",
                name: "Aluminum 6061 - Rest Cleanup",
                material: "Aluminum 6061",
                diameter_mm: 3.0,
                flutes: 3,
                surface_speed_m_min: 200.0,
                chipload_mm_tooth: 0.022,
                default_stepdown_mm: 4.5,
                default_stepover_mm: 0.6,
                coolant_override: Some(CoolantMode::AirBlast),
            })
            .with_optimal_load_mm(0.6),
        ],
    };

    proj.ensure_tool_embedded(&t10_rough);
    proj.ensure_tool_embedded(&t8_rest);
    if let Some(fm) = available_tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FaceMill { .. }))
    {
        proj.ensure_tool_embedded(fm);
        let mut op_face = Operation::new("op_0_face_top", "Face Stock Top", OperationKind::Facing);
        op_face.tool_id = Some(fm.id.clone());
        op_face.preset_id = fm.presets.first().map(|p| p.id.clone());
        proj.setups[0].operations.push(op_face);
    }
    if let Some(t1) = available_tools.first() {
        proj.ensure_tool_embedded(t1);
    }

    // Plateau Deck around Z=22 Boss Pad (`Z = 22 -> 20 mm`) — Open-Air 2D Adaptive!
    if let Some(plat_face) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && (origin_mm.z - 20.0).abs() < 0.05 && f.area_mm2 > 1500.0
        )
    }) && let Some(bind) = doc.bind_face(plat_face.id)
    {
        let mut op_plat = Operation::new_adaptive_2d(
            "op_0b_plateau_adaptive",
            "Plateau Deck — Ø10mm Open Adaptive",
        );
        op_plat.tool_id = Some(t10_rough.id.clone());
        op_plat.preset_id = Some("al6061_adaptive".to_string());
        op_plat
            .selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        proj.setups[0].operations.push(op_plat);
    }

    // Island Cavity floor (`Z = 12.0 mm`)
    if let Some(island_floor) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && approx_eq(origin_mm.z, 12.0, 1e-2)
        )
    }) && let Some(bind) = doc.bind_face(island_floor.id)
    {
        // Op 1: Ø10mm 2D Adaptive Roughing (single full-depth 8mm axial cut!)
        let mut op_rough = Operation::new_adaptive_2d(
            "op_1_adaptive_rough_10mm",
            "Island Cavity — Ø10mm Adaptive Rough",
        );
        op_rough.tool_id = Some(t10_rough.id.clone());
        op_rough.preset_id = Some("al6061_adaptive".to_string());
        op_rough.selections.push(GeometrySelectionItem::PlanarFace {
            binding: bind.clone(),
        });
        proj.setups[0].operations.push(op_rough);

        // Op 2: Ø3mm 2D Adaptive Rest Cleanup (`FromPrecedingOperations`)
        let mut op_rest = Operation::new_adaptive_2d(
            "op_2_adaptive_rest_3mm",
            "Island Cavity — Ø3mm Rest Corners",
        );
        op_rest.tool_id = Some(t8_rest.id.clone());
        op_rest.preset_id = Some("al6061_rest".to_string());
        op_rest
            .selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        op_rest.passes.rest_machining = RestMachiningConfig {
            mode: RestMachiningMode::FromPrecedingOperations {
                source_op_ids: vec!["op_1_adaptive_rough_10mm".to_string()],
            },
            rest_overlap: ParamLength::from_mm(0.30),
            min_rest_thickness: ParamLength::from_mm(0.08),
        };
        if let StrategyParams::Pocket2D(p) = op_rest.strategy_mut() {
            p.strategy = PocketStrategyKind::Adaptive(AdaptiveClearingParams::default());
        }
        proj.setups[0].operations.push(op_rest);
    }

    // Open Shoulder floor (`Z = 14.0 mm`): Ø6mm Open-Air 2D Adaptive
    if let Some(open_floor) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && approx_eq(origin_mm.z, 14.0, 1e-2)
        )
    }) && let Some(bind) = doc.bind_face(open_floor.id)
        && let Some(t1) = available_tools.first()
    {
        let mut op_open = Operation::new_adaptive_2d(
            "op_3_open_shoulder_adaptive",
            "Open Shoulder — Ø6mm Air-Entry Adaptive",
        );
        op_open.tool_id = Some(t1.id.clone());
        op_open.preset_id = t1.presets.first().map(|p| p.id.clone());
        op_open
            .selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        proj.setups[0].operations.push(op_open);
    }

    proj
}

/// Builds the Milestone 5 **CAD Revision v1** [`ProjectDocument`] (`v1 -> v2` associative reload demo).
pub fn build_revision_v1_demo_project(
    doc: &CadDocument,
    topo: &TopologyGraph,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("CAD Revision Demo (v1 -> v2)");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        unit_override: None,
    }];

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

    let flat_6mm = proj
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. }
                if (diameter_mm - 6.0).abs() < 0.05
            )
        })
        .cloned();
    let face_38mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FaceMill { .. }))
        .cloned();
    let drill_6mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::TwistDrill { .. }))
        .cloned();
    let chamf_90 = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
        .cloned();

    if let Some(fm) = &face_38mm {
        let mut op = Operation::new("op_0_face", "Face Stock Top", OperationKind::Facing);
        op.tool_id = Some(fm.id.clone());
        op.preset_id = fm.presets.first().map(|p| p.id.clone());
        proj.setups[0].operations.push(op);
    }

    if let Some(pkt_face) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && approx_eq(origin_mm.z, 12.0, 1e-2)
        )
    }) && let Some(bind) = doc.bind_face(pkt_face.id)
        && let Some(em) = &flat_6mm
    {
        let mut op = Operation::new("op_1_pkt", "Resized Blind Pocket", OperationKind::Pocket2D);
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        proj.setups[0].operations.push(op);
    }

    if let Some(bore_face) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Cylinder {
                radius_mm,
                is_internal: true,
                ..
            } if approx_eq(radius_mm, 7.0, 1e-2)
        )
    }) && let Some(bind) = doc.bind_face(bore_face.id)
        && let Some(em) = &flat_6mm
    {
        let mut op = Operation::new(
            "op_2_bore",
            "Shifted Ø14mm Helical Bore",
            OperationKind::DrillBore,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        if let StrategyParams::DrillBore(p) = op.strategy_mut() {
            p.mode = HoleMachiningMode::HelicalBore;
        }
        op.selections
            .push(GeometrySelectionItem::CylindricalHole { binding: bind });
        proj.setups[0].operations.push(op);
    }

    if let Some(h_keep) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Cylinder {
                axis_origin_mm,
                radius_mm,
                is_internal: true,
                ..
            } if approx_eq(radius_mm, 3.0, 1e-2) && approx_eq(axis_origin_mm.y, 48.0, 0.2)
        )
    }) && let Some(bind) = doc.bind_face(h_keep.id)
        && let Some(dr) = &drill_6mm
    {
        let mut op = Operation::new(
            "op_3_hole_keep",
            "Surviving Ø6mm Hole",
            OperationKind::DrillBore,
        );
        op.tool_id = Some(dr.id.clone());
        op.preset_id = dr.presets.first().map(|p| p.id.clone());
        op.selections
            .push(GeometrySelectionItem::CylindricalHole { binding: bind });
        proj.setups[0].operations.push(op);
    }

    if let Some(h_del) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Cylinder {
                axis_origin_mm,
                radius_mm,
                is_internal: true,
                ..
            } if approx_eq(radius_mm, 3.0, 1e-2) && approx_eq(axis_origin_mm.y, 60.0, 0.2)
        )
    }) && let Some(bind) = doc.bind_face(h_del.id)
        && let Some(dr) = &drill_6mm
    {
        let mut op = Operation::new(
            "op_4_hole_deleted",
            "Mounting Hole (Deleted in v2)",
            OperationKind::DrillBore,
        );
        op.tool_id = Some(dr.id.clone());
        op.preset_id = dr.presets.first().map(|p| p.id.clone());
        op.selections
            .push(GeometrySelectionItem::CylindricalHole { binding: bind });
        proj.setups[0].operations.push(op);
    }

    if let (Some(e_front), Some(e_ang), Some(cm)) = (
        doc.edges.iter().find(|e| {
            approx_eq(e.aabb_mm.min.z, 20.0, 1e-2)
                && approx_eq(e.aabb_mm.max.y, 0.0, 1e-2)
                && approx_eq(e.aabb_mm.max.x, 80.0, 1e-2)
        }),
        doc.edges.iter().find(|e| {
            approx_eq(e.aabb_mm.min.z, 20.0, 1e-2)
                && approx_eq(e.aabb_mm.min.x, 80.0, 1e-2)
                && approx_eq(e.aabb_mm.max.x, 90.0, 1e-2)
        }),
        &chamf_90,
    ) && let Some(chain_uses) = topo.find_open_edge_chain(doc, e_front.id, e_ang.id)
        && let (Some(sb), Some(eb)) = (doc.bind_edge(e_front.id), doc.bind_edge(e_ang.id))
    {
        let mut op = Operation::new(
            "op_5_front_chamf",
            "Front Chamfer Open Chain",
            OperationKind::ChamferTrace,
        );
        op.tool_id = Some(cm.id.clone());
        op.preset_id = cm.presets.first().map(|p| p.id.clone());
        op.selections.push(GeometrySelectionItem::OpenChain {
            body_id: doc.bodies[0].id,
            start_edge: sb,
            end_edge: eb,
            resolved_edges: chain_uses,
            reverse: false,
            side_flip: true,
        });
        if let StrategyParams::ChamferTrace(p) = op.strategy_mut() {
            p.mode = ChamferTraceMode::EdgeChamfer;
            p.chamfer_source = ChamferSourceMode::SharpEdge {
                width: ParamLength::from_mm(0.5),
            };
        }
        proj.setups[0].operations.push(op);
    }

    proj
}

/// Builds a Milestone 7 **Pure-2D CNC Router Front Panel (DXF)** [`ProjectDocument`] (`models = []`, zero `.step` files!)
/// seeded from [`sample_router_panel_dxf`]:
/// 1. `Op 1` (`T6` Ø6mm Twist Drill): **4x Ø6mm Corner Mounting Holes** (`HOLES` DXF layer).
/// 2. `Op 2` (`T1` Ø6mm Flat Endmill): **Display Recess Pocket + Boss Island** (`POCKETS` DXF layer, `3.0 mm` deep).
/// 3. `Op 3` (`T1` Ø6mm Flat Endmill): **Vent Slot Through-Cutout** (`CUTOUTS` `INSERT`ed block layer).
/// 4. `Op 4` (`T1` Ø6mm Flat Endmill): **Outer Rounded Panel Profile + Triangular Holding Tabs** (`OUTLINE` DXF layer).
pub fn build_pure2d_router_panel_demo_project(
    source_dxf_path: Option<String>,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("2D Router Electronics Panel (DXF)");
    proj.models.clear();

    let dxf_bytes = sample_router_panel_dxf();
    let opts = DxfImportOptions {
        sketch_id: "sketch_panel".to_string(),
        sketch_name: "Router Front Panel (DXF)".to_string(),
        setup_id: "setup_1".to_string(),
        source_path: source_dxf_path,
        ..DxfImportOptions::default()
    };
    if let Ok(rep) = import_dxf_bytes(&dxf_bytes, &opts) {
        proj.sketches.push(rep.sketch);
    }

    // Configure 6.0 mm sheet stock (`168 × 108 × 6 mm`, `4 mm` XY perimeter margin for clean air-entry)
    proj.setups[0].name = "Setup 1 — 6mm Aluminum Sheet (G54)".to_string();
    proj.setups[0].wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_FRONT_LEFT,
    };
    proj.setups[0].stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(4.0),
        side_neg_x: ParamLength::from_mm(4.0),
        side_pos_y: ParamLength::from_mm(4.0),
        side_neg_y: ParamLength::from_mm(4.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(0.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(6.0),
        round_increment_mm: None,
    };

    let flat_6mm = available_tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. }
                if (diameter_mm - 6.0).abs() < 0.05
            )
        })
        .cloned();
    let drill_6mm = available_tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::TwistDrill { .. }))
        .cloned();

    if let Some(em) = &flat_6mm {
        proj.ensure_tool_embedded(em);
    }
    if let Some(dr) = &drill_6mm {
        proj.ensure_tool_embedded(dr);
    }

    // Op 1: 4x Ø6mm Corner Mounting Holes (`HOLES` layer)
    if let Some(dr) = &drill_6mm {
        let mut op_holes = Operation::new(
            "op_1_panel_holes",
            "4x Ø6mm Mounting Holes",
            OperationKind::DrillBore,
        );
        op_holes.tool_id = Some(dr.id.clone());
        op_holes.preset_id = dr.presets.first().map(|p| p.id.clone());
        op_holes
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sketch_panel".to_string(),
                sub_layer: Some("HOLES".to_string()),
                reverse: false,
                side_flip: false,
            });
        proj.setups[0].operations.push(op_holes);
    }

    // Op 2: Central Display Recess Pocket + Boss Island (`POCKETS` layer, 3.0 mm deep)
    if let Some(em) = &flat_6mm {
        let mut op_pkt = Operation::new(
            "op_2_display_pocket",
            "Display Recess + Boss Island",
            OperationKind::Pocket2D,
        );
        op_pkt.tool_id = Some(em.id.clone());
        op_pkt.preset_id = em.presets.first().map(|p| p.id.clone());
        op_pkt.heights.bottom = HeightSpec::new(HeightReference::StockTop, -3.0);
        op_pkt
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sketch_panel".to_string(),
                sub_layer: Some("POCKETS".to_string()),
                reverse: false,
                side_flip: false,
            });
        proj.setups[0].operations.push(op_pkt);

        // Op 3: Vent Slot Cutout (`CUTOUTS` layer, full 6.0 mm sheet depth via helical-entry pocket)
        let mut op_slot = Operation::new(
            "op_3_vent_slot",
            "Vent Slot Through-Cutout",
            OperationKind::Pocket2D,
        );
        op_slot.tool_id = Some(em.id.clone());
        op_slot.preset_id = em.presets.first().map(|p| p.id.clone());
        op_slot.heights.bottom = HeightSpec::new(HeightReference::StockBottom, 0.0);
        op_slot
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sketch_panel".to_string(),
                sub_layer: Some("CUTOUTS".to_string()),
                reverse: false,
                side_flip: false,
            });
        proj.setups[0].operations.push(op_slot);

        // Op 4: Outer Rounded Panel Profile + Triangular Holding Tabs (`OUTLINE` layer)
        let mut op_outer = Operation::new(
            "op_4_panel_outline",
            "Outer Panel Profile + Tabs",
            OperationKind::Contour2D,
        );
        op_outer.tool_id = Some(em.id.clone());
        op_outer.preset_id = em.presets.first().map(|p| p.id.clone());
        op_outer.heights.bottom = HeightSpec::new(HeightReference::StockBottom, 0.0);
        op_outer
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sketch_panel".to_string(),
                sub_layer: Some("OUTLINE".to_string()),
                reverse: false,
                side_flip: false,
            });
        if let StrategyParams::Contour2D(p) = op_outer.strategy_mut() {
            p.tabs.enabled = true;
            p.tabs.profile = TabProfileKind::Triangular {
                ramp_angle_deg: 25.0,
            };
        }
        proj.setups[0].operations.push(op_outer);
    }

    proj
}

/// Builds a Milestone 8 **Parametric Industrial Nameplate & Dual-Mode CAM Typography**
/// [`ProjectDocument`] (`models = []`) showcasing:
/// 1. Parametric `Rectangle` plaque (`width = "stock_x - 10 mm"`, `height = "stock_y - 10 mm"`, `R10` fillets),
///    `Circle` gauge cutout, `PointPattern::BoltCircle` (`6× Ø6.0 mm` flange holes), TrueType Outline
///    `Text` (`"CHIP"` — `DejaVuSans-Bold` preserving `'P'`'s counter-island), and single-line
///    `HersheySimplexSans` `Text` (`"S/N 2026-08"`).
/// 2. `Op 1` (`T6` Ø6mm Twist Drill): **6× Ø6mm Flange Bolt Circle** (`PointPattern::BoltCircle`).
/// 3. `Op 2` (`T1` Ø6mm Flat Endmill): **Ø22mm Gauge Port Through-Pocket** (`Circle`).
/// 4. `Op 3` (`T9` Ø2mm 2FL Carbide Endmill): **TrueType Outline Text Pocket (`"CHIP"`, `1.5 mm` deep)**.
/// 5. `Op 4` (`T5` 90° Chamfer / Engraving Mill): **Hershey 1-Pass Serial Engraving (`"S/N 2026-08"`, `0.4 mm` deep)**.
/// 6. `Op 5` (`T1` Ø6mm Flat Endmill): **Outer Filleted Plaque Profile + Triangular Holding Tabs**.
pub fn build_engraved_nameplate_demo_project(available_tools: &[Tool]) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Engraved Industrial Nameplate (2D Sketch + Text)");
    proj.models.clear();

    proj.setups[0].name = "Setup 1 — 6mm Anodized Plate (G54 Top Center)".to_string();
    proj.setups[0].wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    proj.setups[0].stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(180.0),
        depth_y: ParamLength::from_mm(84.0),
        height_z: ParamLength::from_mm(6.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::OffsetFromTop {
            top_allowance: ParamLength::from_mm(0.0),
        },
    };

    let mut eval_ctx = EvalContext::new(DisplayUnit::Millimeter);
    eval_ctx.set_var("stock_x", 180.0);
    eval_ctx.set_var("stock_y", 84.0);
    eval_ctx.set_var("stock_z", 6.0);

    let mut sk = SketchLayer::new("sketch_nameplate", "Nameplate Layout & Text", "setup_1");
    sk.entities.push(SketchEntityEntry {
        id: "ent_plaque_rect".to_string(),
        name: "Outer Plaque (R10 Fillets)".to_string(),
        sub_layer: Some("OUTLINE".to_string()),
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(0.0),
            y: ParamLength::from_mm(0.0),
            width: ParamLength::parse("stock_x - 10 mm", &eval_ctx)
                .unwrap_or_else(|_| ParamLength::from_mm(170.0)),
            height: ParamLength::parse("stock_y - 10 mm", &eval_ctx)
                .unwrap_or_else(|_| ParamLength::from_mm(74.0)),
            corner_radius: ParamLength::from_mm(10.0),
            rotation_deg: 0.0,
        },
    });
    sk.entities.push(SketchEntityEntry {
        id: "ent_gauge_bore".to_string(),
        name: "Ø22mm Gauge Window".to_string(),
        sub_layer: Some("CUTOUTS".to_string()),
        is_construction: false,
        entity: SketchEntity::new_circle(
            ParamLength::from_mm(-52.0),
            ParamLength::from_mm(0.0),
            ParamLength::from_mm(22.0),
        ),
    });
    sk.entities.push(SketchEntityEntry {
        id: "ent_flange_bolts".to_string(),
        name: "6x Ø6mm Flange Bolt Circle".to_string(),
        sub_layer: Some("HOLES".to_string()),
        is_construction: false,
        entity: SketchEntity::PointPattern(PointPatternSpec::BoltCircle {
            center_x: ParamLength::from_mm(-52.0),
            center_y: ParamLength::from_mm(0.0),
            bolt_circle_diameter: ParamLength::from_mm(46.0),
            hole_diameter: ParamLength::from_mm(6.0),
            count: 6,
            start_angle_deg: 0.0,
        }),
    });
    sk.entities.push(SketchEntityEntry {
        id: "ent_title_ttf".to_string(),
        name: "Logo Text 'CHIP' (Outline)".to_string(),
        sub_layer: Some("LOGO_TEXT".to_string()),
        is_construction: false,
        entity: SketchEntity::Text(SketchTextSpec {
            content: "CHIP".to_string(),
            font: SketchFontSource::BuiltinSansBold,
            cap_height: ParamLength::from_mm(24.0),
            x: ParamLength::from_mm(26.0),
            y: ParamLength::from_mm(10.0),
            h_align: TextHAlign::Center,
            v_align: TextVAlign::Middle,
            letter_spacing: ParamLength::from_mm(1.2),
            ..SketchTextSpec::default()
        }),
    });
    sk.entities.push(SketchEntityEntry {
        id: "ent_serial_hershey".to_string(),
        name: "Serial 'S/N 2026-08' (1-Line)".to_string(),
        sub_layer: Some("SERIAL_ENGRAVE".to_string()),
        is_construction: false,
        entity: SketchEntity::Text(SketchTextSpec {
            content: "S/N 2026-08".to_string(),
            font: SketchFontSource::Hershey {
                font: HersheyFontKind::SimplexSans,
            },
            cap_height: ParamLength::from_mm(6.5),
            x: ParamLength::from_mm(26.0),
            y: ParamLength::from_mm(-18.0),
            h_align: TextHAlign::Center,
            v_align: TextVAlign::Middle,
            letter_spacing: ParamLength::from_mm(0.6),
            ..SketchTextSpec::default()
        }),
    });
    sk.rebuild_regions(None);
    proj.sketches.push(sk);

    let flat_6mm = available_tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. }
                if (diameter_mm - 6.0).abs() < 0.05
            )
        })
        .cloned();
    let drill_6mm = available_tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::TwistDrill { .. }))
        .cloned();
    let chamf_90 = available_tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
        .cloned();

    let t9_2mm = Tool {
        id: ToolId("t9_2mm_flat_2fl".to_string()),
        tool_number: 9,
        length_offset_number: 9,
        diameter_offset_number: 9,
        name: "2mm 2FL Carbide Detail Endmill".to_string(),
        vendor: Some("Generic Carbide".to_string()),
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::AirBlast,
        stickout_length_mm: 14.0,
        geometry: CutterGeometry::FlatEndmill {
            diameter_mm: 2.0,
            flute_length_mm: 8.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 3.175,
            flutes: 2,
        },
        presets: vec![CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
            id: "al6061_detail",
            name: "Aluminum 6061 - Fine Lettering",
            material: "Aluminum 6061",
            diameter_mm: 2.0,
            flutes: 2,
            surface_speed_m_min: 150.0,
            chipload_mm_tooth: 0.015,
            default_stepdown_mm: 0.8,
            default_stepover_mm: 0.9,
            coolant_override: Some(CoolantMode::AirBlast),
        })],
    };
    proj.ensure_tool_embedded(&t9_2mm);
    if let Some(em) = &flat_6mm {
        proj.ensure_tool_embedded(em);
    }
    if let Some(dr) = &drill_6mm {
        proj.ensure_tool_embedded(dr);
    }
    if let Some(ch) = &chamf_90 {
        proj.ensure_tool_embedded(ch);
    }

    // Op 1: 6x Ø6mm Flange Bolt Circle (`Drilling`)
    if let Some(dr) = &drill_6mm {
        let mut op_bolts = Operation::new(
            "op_1_bolt_circle",
            "6x Ø6mm Flange Bolt Circle",
            OperationKind::DrillBore,
        );
        op_bolts.tool_id = Some(dr.id.clone());
        op_bolts.preset_id = dr.presets.first().map(|p| p.id.clone());
        op_bolts
            .selections
            .push(GeometrySelectionItem::SketchEntity {
                sketch_id: "sketch_nameplate".to_string(),
                entity_id: "ent_flange_bolts".to_string(),
                reverse: false,
                side_flip: false,
            });
        proj.setups[0].operations.push(op_bolts);
    }

    // Op 2: Ø22mm Gauge Port Through-Cutout (`2D Pocket`)
    if let Some(em) = &flat_6mm {
        let mut op_bore = Operation::new(
            "op_2_gauge_window",
            "Ø22mm Gauge Window Cutout",
            OperationKind::Pocket2D,
        );
        op_bore.tool_id = Some(em.id.clone());
        op_bore.preset_id = em.presets.first().map(|p| p.id.clone());
        op_bore.heights.bottom = HeightSpec::new(HeightReference::StockBottom, 0.0);
        op_bore
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sketch_nameplate".to_string(),
                sub_layer: Some("CUTOUTS".to_string()),
                reverse: false,
                side_flip: false,
            });
        proj.setups[0].operations.push(op_bore);
    }

    // Op 3: Recessed TrueType Outline Text `"CHIP"` (`1.5 mm` deep, preserving `'P'` counter-island!)
    let mut op_logo = Operation::new(
        "op_3_logo_text_pocket",
        "Outline Text 'CHIP' Pocket",
        OperationKind::Pocket2D,
    );
    op_logo.tool_id = Some(t9_2mm.id.clone());
    op_logo.preset_id = t9_2mm.presets.first().map(|p| p.id.clone());
    op_logo.heights.bottom = HeightSpec::new(HeightReference::StockTop, -1.5);
    op_logo
        .selections
        .push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sketch_nameplate".to_string(),
            entity_id: "ent_title_ttf".to_string(),
            reverse: false,
            side_flip: false,
        });
    if let StrategyParams::Pocket2D(p) = op_logo.strategy_mut() {
        p.stepover = ParamLength::from_mm(0.85);
    }
    proj.setups[0].operations.push(op_logo);

    // Op 4: 1-Pass Hershey Serial Engraving `"S/N 2026-08"` (`CurveTrace`, `0.4 mm` deep)
    if let Some(ch) = &chamf_90 {
        let mut op_ser = Operation::new(
            "op_4_serial_engrave",
            "1-Pass Serial 'S/N 2026-08' Trace",
            OperationKind::ChamferTrace,
        );
        op_ser.tool_id = Some(ch.id.clone());
        op_ser.preset_id = ch.presets.first().map(|p| p.id.clone());
        op_ser.heights.bottom = HeightSpec::new(HeightReference::StockTop, -0.4);
        op_ser.selections.push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sketch_nameplate".to_string(),
            entity_id: "ent_serial_hershey".to_string(),
            reverse: false,
            side_flip: false,
        });
        if let StrategyParams::ChamferTrace(p) = op_ser.strategy_mut() {
            p.mode = ChamferTraceMode::CurveTrace;
            p.trace_compensation = ContourCompensation::CenterOff;
            p.trace_axial_offset = ParamLength::from_mm(0.0);
        }
        proj.setups[0].operations.push(op_ser);
    }

    // Op 5: Outer Filleted Plaque Profile + Triangular Holding Tabs (`OUTLINE`)
    if let Some(em) = &flat_6mm {
        let mut op_out = Operation::new(
            "op_5_plaque_profile",
            "Outer Filleted Plaque + Tabs",
            OperationKind::Contour2D,
        );
        op_out.tool_id = Some(em.id.clone());
        op_out.preset_id = em.presets.first().map(|p| p.id.clone());
        op_out.heights.bottom = HeightSpec::new(HeightReference::StockBottom, 0.0);
        op_out.selections.push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sketch_nameplate".to_string(),
            entity_id: "ent_plaque_rect".to_string(),
            reverse: false,
            side_flip: false,
        });
        if let StrategyParams::Contour2D(p) = op_out.strategy_mut() {
            p.tabs.enabled = true;
            p.tabs.profile = TabProfileKind::Triangular {
                ramp_angle_deg: 25.0,
            };
        }
        proj.setups[0].operations.push(op_out);
    }

    proj
}

/// Builds a Milestone 9 **V-Carved Hardwood Sign & Mated V-Inlay Plug** [`ProjectDocument`]
/// (`models = []`) showcasing:
/// 1. **Setup 1 (`G54` — Female Sign Plaque, `180 × 84 × 12 mm`):**
///    - `Op 1` (`T1` Ø6mm Flat Endmill Stage 1 + `T8` 60° V-Bit Stage 2): **Two-Stage Flat-Bottom
///      V-Carving** of a `36 × 36 mm` filleted crest emblem (with an inner `Ø12 mm` boss island)
///      and TrueType `"CHIP"` title (`flat_depth = 3.0 mm`).
///    - `Op 2` (`T8` 60° V-Bit): **Pure Medial Variable-Depth V-Carve** subtitle `"CAM v2"` with
///      simultaneous 3-axis `(X, Y, Z)` strokes and sharp 3D corner climbs.
///    - `Op 3` (`T1` Ø6mm Flat Endmill): **Outer Filleted Sign Plaque Profile + Triangular Tabs**.
/// 2. **Setup 2 (`G55` — Mated Male V-Inlay Plug, `180 × 84 × 12 mm`):**
///    - `Op 1` (`T1` Ø6mm Flat Endmill Stage 1 + `T8` 60° V-Bit Stage 2): **Mirrored Male V-Inlay
///      Plug** (`start_depth = 2.0 mm`, `glue_gap_bottom = 1.0 mm`, `border_margin = 5.0 mm`,
///      `auto_mirror_x = true`) mated to the Setup 1 crest emblem!
pub fn build_vcarve_sign_and_inlay_demo_project(available_tools: &[Tool]) -> ProjectDocument {
    let mut proj = ProjectDocument::new("V-Carve Sign & Mated V-Inlay Plug");
    proj.models.clear();

    let stock_cfg = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(180.0),
        depth_y: ParamLength::from_mm(84.0),
        height_z: ParamLength::from_mm(12.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::OffsetFromTop {
            top_allowance: ParamLength::from_mm(0.0),
        },
    };

    proj.setups[0].name = "Setup 1 — Female Sign Plaque (G54)".to_string();
    proj.setups[0].wcs_slot = WorkOffsetSlot::G54;
    proj.setups[0].wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    proj.setups[0].stock = stock_cfg.clone();

    let mut setup_plug = Setup::new(
        "setup_2_plug",
        "Setup 2 — Mated Male V-Inlay Plug (G55)",
        WorkOffsetSlot::G55,
    );
    setup_plug.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup_plug.stock = stock_cfg;

    let mut sk = SketchLayer::new("sketch_vcarve_sign", "V-Carve Sign & Crest", "setup_1");
    sk.entities.push(SketchEntityEntry {
        id: "ent_sign_plaque_rect".to_string(),
        name: "Outer Sign Plaque (R8 Fillets)".to_string(),
        sub_layer: Some("OUTLINE".to_string()),
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(0.0),
            y: ParamLength::from_mm(0.0),
            width: ParamLength::from_mm(166.0),
            height: ParamLength::from_mm(70.0),
            corner_radius: ParamLength::from_mm(8.0),
            rotation_deg: 0.0,
        },
    });
    sk.entities.push(SketchEntityEntry {
        id: "ent_crest_outer".to_string(),
        name: "Crest Shield Outer (36x36mm)".to_string(),
        sub_layer: Some("VCARVE_CREST".to_string()),
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(-50.0),
            y: ParamLength::from_mm(0.0),
            width: ParamLength::from_mm(36.0),
            height: ParamLength::from_mm(36.0),
            corner_radius: ParamLength::from_mm(4.0),
            rotation_deg: 0.0,
        },
    });
    sk.entities.push(SketchEntityEntry {
        id: "ent_crest_boss".to_string(),
        name: "Crest Center Boss (Ø12mm)".to_string(),
        sub_layer: Some("VCARVE_CREST".to_string()),
        is_construction: false,
        entity: SketchEntity::new_circle(
            ParamLength::from_mm(-50.0),
            ParamLength::from_mm(0.0),
            ParamLength::from_mm(12.0),
        ),
    });
    sk.entities.push(SketchEntityEntry {
        id: "ent_vcarve_title".to_string(),
        name: "Title 'CHIP' (28mm Bold)".to_string(),
        sub_layer: Some("VCARVE_TITLE".to_string()),
        is_construction: false,
        entity: SketchEntity::Text(SketchTextSpec {
            content: "CHIP".to_string(),
            font: SketchFontSource::BuiltinSansBold,
            cap_height: ParamLength::from_mm(28.0),
            x: ParamLength::from_mm(22.0),
            y: ParamLength::from_mm(8.0),
            h_align: TextHAlign::Center,
            v_align: TextVAlign::Middle,
            letter_spacing: ParamLength::from_mm(1.5),
            ..SketchTextSpec::default()
        }),
    });
    sk.entities.push(SketchEntityEntry {
        id: "ent_vcarve_subtitle".to_string(),
        name: "Subtitle 'CAM v2' (11mm Medial)".to_string(),
        sub_layer: Some("VCARVE_MEDIAL".to_string()),
        is_construction: false,
        entity: SketchEntity::Text(SketchTextSpec {
            content: "CAM v2".to_string(),
            font: SketchFontSource::BuiltinSansBold,
            cap_height: ParamLength::from_mm(11.0),
            x: ParamLength::from_mm(22.0),
            y: ParamLength::from_mm(-17.0),
            h_align: TextHAlign::Center,
            v_align: TextVAlign::Middle,
            letter_spacing: ParamLength::from_mm(1.0),
            ..SketchTextSpec::default()
        }),
    });
    sk.rebuild_regions(None);
    proj.sketches.push(sk);

    let flat_6mm = available_tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. }
                if (diameter_mm - 6.0).abs() < 0.05
            )
        })
        .cloned();
    let vbit_60 = available_tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::ChamferMill {
                    tip_diameter_mm,
                    included_angle_deg,
                    ..
                } if tip_diameter_mm <= 0.05 && (included_angle_deg - 60.0).abs() < 1.0
            )
        })
        .or_else(|| {
            available_tools
                .iter()
                .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
        })
        .cloned();

    if let Some(em) = &flat_6mm {
        proj.ensure_tool_embedded(em);
    }
    if let Some(vb) = &vbit_60 {
        proj.ensure_tool_embedded(vb);
    }

    // Setup 1 Op 1: Two-Stage Flat-Bottom V-Carving (`T1` Ø6mm Clear + `T8` 60° V-Bit)
    if let Some(vb) = &vbit_60 {
        let mut op_female = Operation::new(
            "op_1_female_vcarve",
            "Crest & 'CHIP' — 2-Tool Flat V-Carve",
            OperationKind::VCarve,
        );
        op_female.tool_id = Some(vb.id.clone());
        op_female.preset_id = vb.presets.first().map(|p| p.id.clone());
        op_female
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sketch_vcarve_sign".to_string(),
                sub_layer: Some("VCARVE_CREST".to_string()),
                reverse: false,
                side_flip: false,
            });
        op_female
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sketch_vcarve_sign".to_string(),
                sub_layer: Some("VCARVE_TITLE".to_string()),
                reverse: false,
                side_flip: false,
            });
        if let StrategyParams::VCarve(p) = op_female.strategy_mut() {
            p.flat_depth = Some(ParamLength::from_mm(3.0));
            if let Some(em) = &flat_6mm {
                p.clearing_tool_id = Some(em.id.clone());
                p.clearing_preset_id = em.presets.first().map(|pr| pr.id.clone());
            }
            p.clearing_strategy = VCarveClearingKind::ContourParallel;
        }
        proj.setups[0].operations.push(op_female);

        // Setup 1 Op 2: Pure Medial Variable-Depth V-Carve Subtitle `'CAM v2'`
        let mut op_medial = Operation::new(
            "op_2_subtitle_medial",
            "Subtitle 'CAM v2' — Medial V-Carve",
            OperationKind::VCarve,
        );
        op_medial.tool_id = Some(vb.id.clone());
        op_medial.preset_id = vb.presets.first().map(|p| p.id.clone());
        op_medial
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sketch_vcarve_sign".to_string(),
                sub_layer: Some("VCARVE_MEDIAL".to_string()),
                reverse: false,
                side_flip: false,
            });
        proj.setups[0].operations.push(op_medial);

        // Setup 2 Op 1: Mated Male V-Inlay Plug (`G55`)
        let mut op_plug = Operation::new(
            "op_1_male_inlay_plug",
            "Crest Male V-Inlay Plug (Mirrored)",
            OperationKind::VCarve,
        );
        op_plug.tool_id = Some(vb.id.clone());
        op_plug.preset_id = vb.presets.first().map(|p| p.id.clone());
        op_plug
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sketch_vcarve_sign".to_string(),
                sub_layer: Some("VCARVE_CREST".to_string()),
                reverse: false,
                side_flip: false,
            });
        if let StrategyParams::VCarve(p) = op_plug.strategy_mut() {
            p.flat_depth = Some(ParamLength::from_mm(3.0));
            if let Some(em) = &flat_6mm {
                p.clearing_tool_id = Some(em.id.clone());
                p.clearing_preset_id = em.presets.first().map(|pr| pr.id.clone());
            }
            p.inlay_mode = VCarveInlayMode::MalePlug {
                start_depth: ParamLength::from_mm(2.0),
                glue_gap_bottom: ParamLength::from_mm(1.0),
                border_margin: ParamLength::from_mm(5.0),
                auto_mirror_x: true,
            };
        }
        setup_plug.operations.push(op_plug);
    }

    // Setup 1 Op 3: Outer Filleted Sign Plaque Profile + Triangular Tabs
    if let Some(em) = &flat_6mm {
        let mut op_out = Operation::new(
            "op_3_sign_profile",
            "Outer Sign Plaque + Tabs",
            OperationKind::Contour2D,
        );
        op_out.tool_id = Some(em.id.clone());
        op_out.preset_id = em.presets.first().map(|p| p.id.clone());
        op_out.heights.bottom = HeightSpec::new(HeightReference::StockBottom, 0.0);
        op_out.selections.push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sketch_vcarve_sign".to_string(),
            entity_id: "ent_sign_plaque_rect".to_string(),
            reverse: false,
            side_flip: false,
        });
        if let StrategyParams::Contour2D(p) = op_out.strategy_mut() {
            p.tabs.enabled = true;
            p.tabs.profile = TabProfileKind::Triangular {
                ramp_angle_deg: 25.0,
            };
        }
        proj.setups[0].operations.push(op_out);
    }

    proj.setups.push(setup_plug);
    proj
}

/// Helper to initialize a clean single-model tutorial [`ProjectDocument`] with exact stock margins
/// and zero operations.
fn init_tutorial_step_project(
    name: &str,
    model_path: String,
    sha256_hex: &str,
    side_margin_mm: f64,
    top_allowance_mm: f64,
) -> ProjectDocument {
    let mut proj = ProjectDocument::new(name);
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        unit_override: None,
    }];
    proj.setups[0].stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(side_margin_mm),
        side_neg_x: ParamLength::from_mm(side_margin_mm),
        side_pos_y: ParamLength::from_mm(side_margin_mm),
        side_neg_y: ParamLength::from_mm(side_margin_mm),
        top_offset_z: ParamLength::from_mm(top_allowance_mm),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(12.0),
        round_increment_mm: None,
    };
    proj
}

/// Builds the minimal **Tutorial 01 — Facing** project for `01-facing-billet.step`
/// (`100 × 80 × 20 mm` plain billet, `2.0 mm` stock top allowance, ONLY `1. Face Stock Top` using `T7`).
/// When `empty_ops = true`, returns a 0-operation starter project for the initial Stock setup screenshot.
pub fn build_tutorial_facing_project(
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
    empty_ops: bool,
) -> ProjectDocument {
    let mut proj =
        init_tutorial_step_project("Tutorial 01 — Facing", model_path, sha256_hex, 0.0, 2.0);
    if empty_ops {
        return proj;
    }
    if let Some(fm) = available_tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FaceMill { .. }))
    {
        proj.ensure_tool_embedded(fm);
        let mut op = Operation::new("op_1_face", "Face Stock Top", OperationKind::Facing);
        op.tool_id = Some(fm.id.clone());
        op.preset_id = fm.presets.first().map(|p| p.id.clone());
        proj.setups[0].operations.push(op);
    }
    proj
}

/// Builds the minimal **Tutorial 02 — 2D Contour & Holding Tabs** project for `02-contour-plate.step`
/// (`100 × 70 × 12 mm` rounded/chamfered plate inside `4.0 mm` side stock margin, ONLY `1. Outer Profile + Tabs`).
pub fn build_tutorial_contour_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
    empty_ops: bool,
) -> ProjectDocument {
    let mut proj = init_tutorial_step_project(
        "Tutorial 02 — 2D Contour & Tabs",
        model_path,
        sha256_hex,
        4.0,
        0.0,
    );
    if empty_ops {
        return proj;
    }
    if let Some(em) = available_tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.2))
        && let Some(bot_face) = doc.faces.iter().find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z < -0.99 && origin_mm.z.abs() < 0.05
            )
        })
        && let Some(outer_lid) = bot_face.outer_loop
        && let Some(bind) = doc.bind_loop(outer_lid)
    {
        proj.ensure_tool_embedded(em);
        let mut op = Operation::new(
            "op_1_outer_contour",
            "Outer Profile + Tabs",
            OperationKind::Contour2D,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
        op.selections.push(GeometrySelectionItem::ClosedLoop {
            binding: bind,
            reverse: false,
            side_flip: false,
        });
        if let StrategyParams::Contour2D(p) = op.strategy_mut() {
            p.tabs.enabled = true;
            p.tabs.profile = TabProfileKind::Triangular {
                ramp_angle_deg: 25.0,
            };
        }
        proj.setups[0].operations.push(op);
    }
    proj
}

/// Builds the minimal **Tutorial 03 — 2D Pocket** project for `03-pocket-block.step`
/// (`100 × 80 × 20 mm` pre-squared block, ONLY `1. Island Cavity Pocket` and `2. Open Shoulder Step`).
pub fn build_tutorial_pocket_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
    empty_ops: bool,
) -> ProjectDocument {
    let mut proj =
        init_tutorial_step_project("Tutorial 03 — 2D Pocket", model_path, sha256_hex, 0.0, 0.0);
    if empty_ops {
        return proj;
    }
    let flat_6mm = available_tools.iter().find(|t| {
        matches!(t.geometry, CutterGeometry::FlatEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.2)
    });
    if let Some(em) = flat_6mm {
        proj.ensure_tool_embedded(em);

        // Op 1: Closed multi-island pocket (Z = 12 mm with raised Ø14mm island)
        if let Some(pkt_face) = doc.faces.iter().find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && (origin_mm.z - 12.0).abs() < 0.05
            )
        }) && let Some(bind) = doc.bind_face(pkt_face.id)
        {
            let mut op = Operation::new(
                "op_1_island_pocket",
                "Island Cavity Pocket",
                OperationKind::Pocket2D,
            );
            op.tool_id = Some(em.id.clone());
            op.preset_id = em.presets.first().map(|p| p.id.clone());
            op.selections
                .push(GeometrySelectionItem::PlanarFace { binding: bind });
            proj.setups[0].operations.push(op);
        }

        // Op 2: Open-shoulder step shelf (Z = 14 mm, open on 3 air sides)
        if let Some(step_face) = doc.faces.iter().find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && (origin_mm.z - 14.0).abs() < 0.05
            )
        }) && let Some(bind) = doc.bind_face(step_face.id)
        {
            let mut op = Operation::new(
                "op_2_open_step",
                "Open Shoulder Step",
                OperationKind::Pocket2D,
            );
            op.tool_id = Some(em.id.clone());
            op.preset_id = em.presets.first().map(|p| p.id.clone());
            op.selections
                .push(GeometrySelectionItem::PlanarFace { binding: bind });
            proj.setups[0].operations.push(op);
        }
    }
    proj
}

/// Builds the minimal **Tutorial 04 — Drilling & Helical Bore** project for `04-drill-bore-plate.step`
/// (`100 × 70 × 16 mm` pre-squared plate, ONLY `1. 4x Ø6mm Through-Holes` and `2. Ø18mm Helical Counterbore`).
pub fn build_tutorial_drill_bore_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
    empty_ops: bool,
) -> ProjectDocument {
    let mut proj = init_tutorial_step_project(
        "Tutorial 04 — Drilling & Helical Bore",
        model_path,
        sha256_hex,
        0.0,
        0.0,
    );
    if empty_ops {
        return proj;
    }

    let drill_6mm = available_tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::TwistDrill { .. }));
    let flat_6mm = available_tools.iter().find(|t| {
        matches!(t.geometry, CutterGeometry::FlatEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.2)
    });

    // Op 1: 4x Ø6mm corner through-holes (`T6` 6.0mm 118° Stub Drill)
    if let Some(dr) = drill_6mm
        && let Some(r3_seed) = doc.faces.iter().find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if (radius_mm - 3.0).abs() < 0.05
            )
        })
    {
        proj.ensure_tool_embedded(dr);
        let mut op = Operation::new(
            "op_1_through_holes",
            "4x Ø6mm Through-Holes",
            OperationKind::DrillBore,
        );
        op.tool_id = Some(dr.id.clone());
        op.preset_id = dr.presets.first().map(|p| p.id.clone());
        for fid in find_same_diameter_holes(doc, r3_seed.id, 1e-3, 1e-3) {
            if let Some(b) = doc.bind_face(fid) {
                op.selections
                    .push(GeometrySelectionItem::CylindricalHole { binding: b });
            }
        }
        proj.setups[0].operations.push(op);
    }

    // Op 2: Central Ø18mm blind counterbore (`T1` 6mm Flat Endmill in Helical Bore mode)
    if let Some(em) = flat_6mm
        && let Some(r9_bore) = doc.faces.iter().find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if (radius_mm - 9.0).abs() < 0.05
            )
        })
        && let Some(bind) = doc.bind_face(r9_bore.id)
    {
        proj.ensure_tool_embedded(em);
        let mut op = Operation::new(
            "op_2_helical_bore",
            "Ø18mm Helical Counterbore",
            OperationKind::DrillBore,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        if let StrategyParams::DrillBore(p) = op.strategy_mut() {
            p.mode = HoleMachiningMode::HelicalBore;
            p.tip_breakthrough = false;
        }
        op.selections
            .push(GeometrySelectionItem::CylindricalHole { binding: bind });
        proj.setups[0].operations.push(op);
    }

    proj
}

/// Builds the minimal **Tutorial 05 — 2D Adaptive Clearing & 2.5D Rest Machining** project for
/// `05-adaptive-pocket.step` (`100 × 80 × 20 mm` pre-squared block with one deep `10 mm` sharp-corner
/// pocket + `Ø16 mm` central boss):
/// - `Op 1`: `Ø10mm Adaptive Rough` (single full-depth `10 mm` axial cut, `Optimal Load = 2.0 mm`, `Radial Stock = 0.2 mm`, RCT enabled),
/// - `Op 2`: `Ø3mm Rest Corners` (`FromPrecedingOperations`, clearing only the 4 uncut `R5 mm` corner wedges & wall stock).
pub fn build_tutorial_adaptive_rest_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    empty_ops: bool,
) -> ProjectDocument {
    let mut proj = init_tutorial_step_project(
        "Tutorial 05 — 2D Adaptive & Rest",
        model_path,
        sha256_hex,
        0.0,
        0.0,
    );
    if empty_ops {
        return proj;
    }

    let t10_rough = Tool {
        id: ToolId("t10_10mm_adaptive_3fl".to_string()),
        tool_number: 10,
        length_offset_number: 10,
        diameter_offset_number: 10,
        name: "10mm 3FL Carbide Adaptive Rougher".to_string(),
        vendor: Some("Generic Carbide".to_string()),
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::AirBlast,
        stickout_length_mm: 30.0,
        geometry: CutterGeometry::FlatEndmill {
            diameter_mm: 10.0,
            flute_length_mm: 22.0,
            overall_length_mm: 65.0,
            shank_diameter_mm: 10.0,
            flutes: 3,
        },
        presets: vec![
            CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
                id: "al6061_adaptive",
                name: "Aluminum 6061 - Adaptive Roughing",
                material: "Aluminum 6061",
                diameter_mm: 10.0,
                flutes: 3,
                surface_speed_m_min: 280.0,
                chipload_mm_tooth: 0.055,
                default_stepdown_mm: 12.0,
                default_stepover_mm: 2.0,
                coolant_override: Some(CoolantMode::AirBlast),
            })
            .with_optimal_load_mm(2.0),
        ],
    };

    let t3_rest = Tool {
        id: ToolId("t3_3mm_rest_3fl".to_string()),
        tool_number: 3,
        length_offset_number: 3,
        diameter_offset_number: 3,
        name: "3mm 3FL Carbide Rest Endmill".to_string(),
        vendor: Some("Generic Carbide".to_string()),
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::AirBlast,
        stickout_length_mm: 18.0,
        geometry: CutterGeometry::FlatEndmill {
            diameter_mm: 3.0,
            flute_length_mm: 12.0,
            overall_length_mm: 45.0,
            shank_diameter_mm: 3.0,
            flutes: 3,
        },
        presets: vec![
            CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
                id: "al6061_rest",
                name: "Aluminum 6061 - Rest Cleanup",
                material: "Aluminum 6061",
                diameter_mm: 3.0,
                flutes: 3,
                surface_speed_m_min: 200.0,
                chipload_mm_tooth: 0.022,
                default_stepdown_mm: 5.0,
                default_stepover_mm: 0.6,
                coolant_override: Some(CoolantMode::AirBlast),
            })
            .with_optimal_load_mm(0.6),
        ],
    };

    proj.ensure_tool_embedded(&t10_rough);
    proj.ensure_tool_embedded(&t3_rest);

    if let Some(pkt_floor) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && approx_eq(origin_mm.z, 10.0, 1e-2)
        )
    }) && let Some(bind) = doc.bind_face(pkt_floor.id)
    {
        // Op 1: Ø10mm 2D Adaptive Roughing (single full-depth 10mm axial cut + 0.2mm wall allowance)
        let mut op_rough =
            Operation::new_adaptive_2d("op_1_adaptive_rough_10mm", "Ø10mm Adaptive Rough");
        op_rough.tool_id = Some(t10_rough.id.clone());
        op_rough.preset_id = Some("al6061_adaptive".to_string());
        op_rough.passes.stock_to_leave_radial = ParamLength::from_mm(0.2);
        op_rough.selections.push(GeometrySelectionItem::PlanarFace {
            binding: bind.clone(),
        });
        proj.setups[0].operations.push(op_rough);

        // Op 2: Ø3mm 2D Adaptive Rest Cleanup (`FromPrecedingOperations`)
        let mut op_rest = Operation::new_adaptive_2d("op_2_adaptive_rest_3mm", "Ø3mm Rest Corners");
        op_rest.tool_id = Some(t3_rest.id.clone());
        op_rest.preset_id = Some("al6061_rest".to_string());
        op_rest
            .selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        op_rest.passes.rest_machining = RestMachiningConfig {
            mode: RestMachiningMode::FromPrecedingOperations {
                source_op_ids: vec!["op_1_adaptive_rough_10mm".to_string()],
            },
            rest_overlap: ParamLength::from_mm(0.30),
            min_rest_thickness: ParamLength::from_mm(0.08),
        };
        if let StrategyParams::Pocket2D(p) = op_rest.strategy_mut() {
            p.strategy = PocketStrategyKind::Adaptive(AdaptiveClearingParams::default());
        }
        proj.setups[0].operations.push(op_rest);
    }

    proj
}

/// Builds the minimal **Tutorial 06 — Edge Chamfering & 2D Curve Trace / Engraving** project for
/// `06-chamfer-engrave-block.step` (`100 × 70 × 18 mm` pre-squared block with a modeled `45° × 1.5 mm`
/// front bevel, sharp `90°` back edge, and clean top face):
/// - `Op 1`: `Modeled 45° Front Bevel` (`EdgeChamfer`, `ModeledFace`, `T5`),
/// - `Op 2`: `Sharp Back Edge Deburr` (`EdgeChamfer`, `SharpEdge { width: 0.75 mm }`, `T5`),
/// - `Op 3`: `Hershey Text Engraving` (`CurveTrace`, `CenterOff`, `0.4 mm` deep single-line `"CHIP-06"`, `T5`).
pub fn build_tutorial_chamfer_trace_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
    empty_ops: bool,
) -> ProjectDocument {
    let mut proj = init_tutorial_step_project(
        "Tutorial 06 — Chamfer & Trace",
        model_path,
        sha256_hex,
        0.0,
        0.0,
    );
    if empty_ops {
        return proj;
    }

    let chamf_90 = available_tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::ChamferMill {
                    included_angle_deg,
                    ..
                } if (included_angle_deg - 90.0).abs() < 1.0
            )
        })
        .or_else(|| {
            available_tools
                .iter()
                .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
        });

    // Attach 1 single-line Hershey engraving sketch on Setup XY (which coincides with the Z=18 top face!)
    let mut sk = SketchLayer::new("sketch_hershey_label", "Top Face Engraving", "setup_1");
    sk.entities.push(SketchEntityEntry {
        id: "ent_hershey_chip06".to_string(),
        name: "Serial 'CHIP-06' (Hershey 1-Line)".to_string(),
        sub_layer: Some("ENGRAVE".to_string()),
        is_construction: false,
        entity: SketchEntity::Text(SketchTextSpec {
            content: "CHIP-06".to_string(),
            font: SketchFontSource::Hershey {
                font: HersheyFontKind::SimplexSans,
            },
            cap_height: ParamLength::from_mm(12.0),
            x: ParamLength::from_mm(50.0),
            y: ParamLength::from_mm(36.0),
            h_align: TextHAlign::Center,
            v_align: TextVAlign::Middle,
            letter_spacing: ParamLength::from_mm(1.2),
            ..SketchTextSpec::default()
        }),
    });
    sk.rebuild_regions(None);
    proj.sketches.push(sk);

    let Some(ch) = chamf_90 else {
        return proj;
    };
    proj.ensure_tool_embedded(ch);

    // Op 1: Modeled 45° x 1.5 mm front bevel face
    if let Some(bevel_face) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { normal, .. }
            if normal.z > 0.65 && normal.z < 0.76
        )
    }) && let Some(bind) = doc.bind_face(bevel_face.id)
    {
        let mut op = Operation::new(
            "op_1_modeled_bevel",
            "Modeled 45° Front Bevel",
            OperationKind::ChamferTrace,
        );
        op.tool_id = Some(ch.id.clone());
        op.preset_id = ch.presets.first().map(|p| p.id.clone());
        if let StrategyParams::ChamferTrace(p) = op.strategy_mut() {
            p.mode = ChamferTraceMode::EdgeChamfer;
            p.chamfer_source = ChamferSourceMode::ModeledFace;
            p.tip_offset = ParamLength::from_mm(0.5);
            p.check_adjacent_walls = true;
        }
        op.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        proj.setups[0].operations.push(op);
    }

    // Op 2: Sharp unmodeled back edge deburr (Y = 70, Z = 18)
    if let Some(back_edge) = doc.edges.iter().find(|e| {
        approx_eq(e.aabb_mm.min.z, 18.0, 1e-2)
            && approx_eq(e.aabb_mm.max.z, 18.0, 1e-2)
            && approx_eq(e.aabb_mm.min.y, 70.0, 1e-2)
            && approx_eq(e.aabb_mm.max.y, 70.0, 1e-2)
    }) && let Some(b) = doc.bind_edge(back_edge.id)
    {
        let mut op = Operation::new(
            "op_2_sharp_deburr",
            "Sharp Back Edge Deburr",
            OperationKind::ChamferTrace,
        );
        op.tool_id = Some(ch.id.clone());
        op.preset_id = ch.presets.first().map(|p| p.id.clone());
        if let StrategyParams::ChamferTrace(p) = op.strategy_mut() {
            p.mode = ChamferTraceMode::EdgeChamfer;
            p.chamfer_source = ChamferSourceMode::SharpEdge {
                width: ParamLength::from_mm(0.75),
            };
            p.tip_offset = ParamLength::from_mm(0.5);
            p.check_adjacent_walls = true;
        }
        op.selections.push(GeometrySelectionItem::OpenChain {
            body_id: doc.bodies[0].id,
            start_edge: b.clone(),
            end_edge: b,
            resolved_edges: vec![chip_cad::OrientedEdgeUse {
                edge_id: back_edge.id,
                forward: true,
            }],
            reverse: false,
            side_flip: false,
        });
        proj.setups[0].operations.push(op);
    }

    // Op 3: 1-Pass Single-Line Hershey Text Engraving (`CurveTrace`, `CenterOff`, `0.4 mm` deep)
    let mut op_trace = Operation::new(
        "op_3_hershey_trace",
        "Hershey Text Engraving",
        OperationKind::ChamferTrace,
    );
    op_trace.tool_id = Some(ch.id.clone());
    op_trace.preset_id = ch.presets.first().map(|p| p.id.clone());
    op_trace.heights.bottom = HeightSpec::new(HeightReference::StockTop, -0.4);
    op_trace
        .selections
        .push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sketch_hershey_label".to_string(),
            entity_id: "ent_hershey_chip06".to_string(),
            reverse: false,
            side_flip: false,
        });
    if let StrategyParams::ChamferTrace(p) = op_trace.strategy_mut() {
        p.mode = ChamferTraceMode::CurveTrace;
        p.trace_compensation = ContourCompensation::CenterOff;
        p.trace_axial_offset = ParamLength::from_mm(0.0);
    }
    proj.setups[0].operations.push(op_trace);

    proj
}

/// Builds the minimal Pure-2D **Tutorial 07 — 3D V-Carving, Flat Clearance, & Mated V-Inlays**
/// project (`models = []`) with a single clean TrueType `"CHIP"` sketch on `120 × 55 × 15 mm` stock:
/// - When `medial_only = true`: Setup 1 (`G54`) has ONLY `1. Full-Depth Medial V-Carve` (`T8` `60° V-Bit`).
/// - When `medial_only = false`:
///   - `Setup 1 [G54]`: ONLY `1. Two-Stage Flat V-Carve` (`T8` `60° V-Bit` + `T3` `1/8 in` Flat Endmill at `flat_depth = 3.0 mm`).
///   - `Setup 2 [G55]`: ONLY `1. Male V-Inlay Plug` (`T8` + `T3`, `start_depth = 2.0 mm`, `glue_gap = 1.0 mm`, `auto_mirror_x = true`).
pub fn build_tutorial_vcarve_inlay_project(
    available_tools: &[Tool],
    medial_only: bool,
) -> ProjectDocument {
    let mut proj = ProjectDocument::new(if medial_only {
        "Tutorial 07 — Medial V-Carve"
    } else {
        "Tutorial 07 — V-Carve & V-Inlay"
    });
    proj.models.clear();

    let stock_cfg = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(120.0),
        depth_y: ParamLength::from_mm(55.0),
        height_z: ParamLength::from_mm(15.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::OffsetFromTop {
            top_allowance: ParamLength::from_mm(0.0),
        },
    };

    proj.setups[0].name = "Setup 1 — Female Plaque (G54)".to_string();
    proj.setups[0].wcs_slot = WorkOffsetSlot::G54;
    proj.setups[0].wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    proj.setups[0].stock = stock_cfg.clone();

    let mut sk = SketchLayer::new("sketch_vcarve_word", "V-Carve Text 'CHIP'", "setup_1");
    sk.entities.push(SketchEntityEntry {
        id: "ent_vcarve_chip".to_string(),
        name: "Title 'CHIP' (24mm Bold)".to_string(),
        sub_layer: Some("VCARVE".to_string()),
        is_construction: false,
        entity: SketchEntity::Text(SketchTextSpec {
            content: "CHIP".to_string(),
            font: SketchFontSource::BuiltinSansBold,
            cap_height: ParamLength::from_mm(24.0),
            x: ParamLength::from_mm(0.0),
            y: ParamLength::from_mm(0.0),
            h_align: TextHAlign::Center,
            v_align: TextVAlign::Middle,
            letter_spacing: ParamLength::from_mm(1.5),
            ..SketchTextSpec::default()
        }),
    });
    sk.rebuild_regions(None);
    proj.sketches.push(sk);

    let flat_clearing = available_tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. }
                if (diameter_mm - 3.175).abs() < 0.2
            )
        })
        .or_else(|| {
            available_tools
                .iter()
                .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { .. }))
        })
        .cloned();
    let vbit_60 = available_tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::ChamferMill {
                    tip_diameter_mm,
                    included_angle_deg,
                    ..
                } if tip_diameter_mm <= 0.05 && (included_angle_deg - 60.0).abs() < 1.0
            )
        })
        .cloned();

    let Some(vb) = vbit_60 else {
        return proj;
    };
    proj.ensure_tool_embedded(&vb);

    if medial_only {
        let mut op_medial = Operation::new(
            "op_1_medial_vcarve",
            "Full-Depth Medial V-Carve",
            OperationKind::VCarve,
        );
        op_medial.tool_id = Some(vb.id.clone());
        op_medial.preset_id = vb.presets.first().map(|p| p.id.clone());
        op_medial
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sketch_vcarve_word".to_string(),
                sub_layer: Some("VCARVE".to_string()),
                reverse: false,
                side_flip: false,
            });
        if let StrategyParams::VCarve(p) = op_medial.strategy_mut() {
            p.flat_depth = None;
            p.clearing_tool_id = None;
        }
        proj.setups[0].operations.push(op_medial);
        return proj;
    }

    if let Some(em) = &flat_clearing {
        proj.ensure_tool_embedded(em);
    }

    // Setup 1 Op 1: Two-Stage Flat-Bottom V-Carve (`T3` clearing + `T8` 60° V-Bit)
    let mut op_female = Operation::new(
        "op_1_two_stage_vcarve",
        "Two-Stage Flat V-Carve",
        OperationKind::VCarve,
    );
    op_female.tool_id = Some(vb.id.clone());
    op_female.preset_id = vb.presets.first().map(|p| p.id.clone());
    op_female
        .selections
        .push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sketch_vcarve_word".to_string(),
            sub_layer: Some("VCARVE".to_string()),
            reverse: false,
            side_flip: false,
        });
    if let StrategyParams::VCarve(p) = op_female.strategy_mut() {
        p.flat_depth = Some(ParamLength::from_mm(3.0));
        if let Some(em) = &flat_clearing {
            p.clearing_tool_id = Some(em.id.clone());
            p.clearing_preset_id = em.presets.first().map(|pr| pr.id.clone());
        }
        p.clearing_strategy = VCarveClearingKind::ContourParallel;
    }
    proj.setups[0].operations.push(op_female);

    // Setup 2 (G55): Mirrored Mated Male V-Inlay Plug
    let mut setup_plug = Setup::new(
        "setup_2_plug",
        "Setup 2 — Male V-Inlay Plug (G55)",
        WorkOffsetSlot::G55,
    );
    setup_plug.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup_plug.stock = stock_cfg;

    let mut op_plug = Operation::new(
        "op_1_male_v_inlay",
        "Male V-Inlay Plug (Mirrored)",
        OperationKind::VCarve,
    );
    op_plug.tool_id = Some(vb.id.clone());
    op_plug.preset_id = vb.presets.first().map(|p| p.id.clone());
    op_plug
        .selections
        .push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sketch_vcarve_word".to_string(),
            sub_layer: Some("VCARVE".to_string()),
            reverse: false,
            side_flip: false,
        });
    if let StrategyParams::VCarve(p) = op_plug.strategy_mut() {
        p.flat_depth = Some(ParamLength::from_mm(3.0));
        if let Some(em) = &flat_clearing {
            p.clearing_tool_id = Some(em.id.clone());
            p.clearing_preset_id = em.presets.first().map(|pr| pr.id.clone());
        }
        p.clearing_strategy = VCarveClearingKind::ContourParallel;
        p.inlay_mode = VCarveInlayMode::MalePlug {
            start_depth: ParamLength::from_mm(2.0),
            glue_gap_bottom: ParamLength::from_mm(1.0),
            border_margin: ParamLength::from_mm(5.0),
            auto_mirror_x: true,
        };
    }
    setup_plug.operations.push(op_plug);
    proj.setups.push(setup_plug);

    proj
}

/// Builds the Phase 5 **Sculpted Dome (STL Mesh)** sample [`ProjectDocument`] (`70 × 70 × 33 mm`
/// watertight `R = 25 mm` hemisphere seated on an `8 mm` base pedestal), seeded with:
/// - **Op 1:** `3D Adaptive Roughing` (Ø6 mm Flat Endmill, `6 mm` rough / `2 mm` fine step-up, `0.5 mm` radial / `0.3 mm` axial stock-to-leave),
/// - **Op 2:** `3D Waterline Semi-Finish` (Ø6 mm Ball Endmill, `1.75 mm` adaptive Z-level contours with flat-plateau detection),
/// - **Op 3:** `3D Pencil Fillet Cleanup` (Ø3 mm Ball Endmill tracing the concave dome-to-pedestal crease left by the Ø6 mm ball).
pub fn build_sculpted_mesh_dome_project(
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Sculpted Dome (STL Mesh)");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        unit_override: None,
    }];
    proj.setups[0].name = "Setup 1 — 3D Dome Mesh".to_string();
    proj.setups[0].wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    proj.setups[0].stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(2.0),
        side_neg_x: ParamLength::from_mm(2.0),
        side_pos_y: ParamLength::from_mm(2.0),
        side_neg_y: ParamLength::from_mm(2.0),
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(12.0),
        round_increment_mm: None,
    };
    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

    // Ensure a long-reach Ø3 mm Ball Endmill is available for Op 3: 3D Pencil Fillet Cleanup
    let ball_3mm_tool = Tool {
        id: chip_tools::ToolId("t11_3mm_ball_2fl".to_string()),
        tool_number: 11,
        length_offset_number: 11,
        diameter_offset_number: 11,
        name: "3mm 2FL Carbide Ball Endmill".to_string(),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: chip_tools::SpindleDirection::Cw,
        default_coolant: chip_tools::CoolantMode::AirBlast,
        stickout_length_mm: 30.0,
        geometry: CutterGeometry::BallEndmill {
            diameter_mm: 3.0,
            flute_length_mm: 16.0,
            overall_length_mm: 55.0,
            shank_diameter_mm: 3.0,
            flutes: 2,
        },
        presets: vec![chip_tools::CuttingPreset::from_surface_speed(
            chip_tools::SurfaceSpeedPresetSpec {
                id: "al6061_3d_pencil",
                name: "Aluminum 6061 - 3D Finish",
                material: "Aluminum 6061",
                diameter_mm: 3.0,
                flutes: 2,
                surface_speed_m_min: 220.0,
                chipload_mm_tooth: 0.020,
                default_stepdown_mm: 0.8,
                default_stepover_mm: 0.5,
                coolant_override: Some(chip_tools::CoolantMode::Mist),
            },
        )],
    };
    proj.ensure_tool_embedded(&ball_3mm_tool);

    let flat_6mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5))
        .or_else(|| proj.tools.first())
        .cloned();
    let ball_6mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::BallEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5))
        .cloned();

    // Op 1: 3D Adaptive Roughing down to the `Z = 8 mm` pedestal top (`ModelTop - 25.0 mm`)
    let mut op_rough = Operation::new(
        "op_1_adaptive_3d",
        "3D Adaptive Roughing",
        OperationKind::Adaptive3D,
    );
    if let Some(t) = &flat_6mm {
        op_rough.tool_id = Some(t.id.clone());
        op_rough.preset_id = t.presets.first().map(|p| p.id.clone());
    }
    op_rough.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -25.0);
    op_rough.passes.stock_to_leave_radial = ParamLength::from_mm(0.5);
    op_rough.passes.stock_to_leave_axial = ParamLength::from_mm(0.3);
    if let StrategyParams::Adaptive3D(p) = op_rough.strategy_mut() {
        p.max_roughing_stepdown = ParamLength::from_mm(6.0);
        p.fine_stepdown = ParamLength::from_mm(2.0);
        p.adaptive.optimal_load = ParamLength::from_mm(1.5);
        p.detect_flats = true;
    }

    // Op 2: 3D Waterline Semi-Finish across the sculpted dome
    let mut op_wl = Operation::new(
        "op_2_waterline_3d",
        "3D Waterline Semi-Finish",
        OperationKind::Waterline3D,
    );
    if let Some(t) = ball_6mm.as_ref().or(flat_6mm.as_ref()) {
        op_wl.tool_id = Some(t.id.clone());
        op_wl.preset_id = t.presets.first().map(|p| p.id.clone());
    }
    op_wl.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -25.0);
    op_wl.passes.max_stepdown = ParamLength::from_mm(1.75);
    if let StrategyParams::Waterline3D(p) = op_wl.strategy_mut() {
        p.adaptive_stepdown = true;
        p.min_stepdown = ParamLength::from_mm(0.5);
        p.max_cusp_height = ParamLength::from_mm(0.08);
        p.detect_flats = true;
    }

    // Op 3: 3D Pencil Fillet Cleanup tracing the concave dome-to-pedestal crease
    let mut op_pencil = Operation::new(
        "op_3_pencil_3d",
        "3D Pencil Fillet Cleanup",
        OperationKind::Pencil3D,
    );
    op_pencil.tool_id = Some(ball_3mm_tool.id.clone());
    op_pencil.preset_id = ball_3mm_tool.presets.first().map(|p| p.id.clone());
    op_pencil.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -25.0);
    if let StrategyParams::Pencil3D(p) = op_pencil.strategy_mut() {
        p.boundary_mode = chip_cam::MachiningBoundaryMode::ModelSilhouette;
        p.tool_containment = chip_cam::ToolContainmentMode::ToolInsideBoundary;
        p.reference_tool_diameter = ParamLength::from_mm(6.0);
        p.valley_threshold = ParamLength::from_mm(0.15);
        p.num_offset_passes = 0;
    }

    proj.setups[0].operations = vec![op_rough, op_wl, op_pencil];
    proj
}

/// Builds the Phase 5 **Relief Wave (OBJ Mesh)** sample [`ProjectDocument`] (`80 × 60 × 15 mm`
/// sculpted sinusoidal relief wave), seeded with:
/// - **Op 1:** `3D Adaptive Roughing` (Ø6 mm Flat Endmill, `4 mm` rough / `2 mm` fine, `0.4 mm` radial / `0.25 mm` axial stock-to-leave),
/// - **Op 2:** `3D Parallel Raster Finish` (Ø6 mm Ball Endmill, `0.035 mm` cusp height, `45°` raster angle),
/// - **Op 3:** `3D Constant-Scallop Finish` (Ø6 mm Ball Endmill, `0.04 mm` uniform 3D cusp height).
pub fn build_relief_wave_mesh_project(
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Relief Wave (OBJ Mesh)");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        unit_override: None,
    }];
    proj.setups[0].name = "Setup 1 — 3D Relief Wave (OBJ)".to_string();
    proj.setups[0].wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    proj.setups[0].stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(2.0),
        side_neg_x: ParamLength::from_mm(2.0),
        side_pos_y: ParamLength::from_mm(2.0),
        side_neg_y: ParamLength::from_mm(2.0),
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(12.0),
        round_increment_mm: None,
    };
    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

    let flat_6mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5))
        .or_else(|| proj.tools.first())
        .cloned();
    let ball_6mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::BallEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5))
        .cloned();

    // Op 1: 3D Adaptive Roughing down to wave trough depth (`ModelTop - 9.0 mm`)
    let mut op_rough = Operation::new(
        "op_1_wave_adaptive_3d",
        "3D Adaptive Roughing",
        OperationKind::Adaptive3D,
    );
    if let Some(t) = &flat_6mm {
        op_rough.tool_id = Some(t.id.clone());
        op_rough.preset_id = t.presets.first().map(|p| p.id.clone());
    }
    op_rough.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -9.0);
    op_rough.passes.stock_to_leave_radial = ParamLength::from_mm(0.4);
    op_rough.passes.stock_to_leave_axial = ParamLength::from_mm(0.25);
    if let StrategyParams::Adaptive3D(p) = op_rough.strategy_mut() {
        p.max_roughing_stepdown = ParamLength::from_mm(4.0);
        p.fine_stepdown = ParamLength::from_mm(2.0);
        p.adaptive.optimal_load = ParamLength::from_mm(1.5);
        p.detect_flats = true;
    }

    // Op 2: 3D Parallel Raster Finishing at 45° with cusp-height stepover
    let mut op_par = Operation::new(
        "op_2_wave_parallel_3d",
        "3D Parallel Raster Finish (45°)",
        OperationKind::Parallel3D,
    );
    if let Some(t) = ball_6mm.as_ref().or(flat_6mm.as_ref()) {
        op_par.tool_id = Some(t.id.clone());
        op_par.preset_id = t.presets.first().map(|p| p.id.clone());
    }
    op_par.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -10.0);
    if let StrategyParams::Parallel3D(p) = op_par.strategy_mut() {
        p.boundary_mode = chip_cam::MachiningBoundaryMode::ModelSilhouette;
        p.tool_containment = chip_cam::ToolContainmentMode::ToolCenterOnBoundary;
        p.use_cusp_height = true;
        p.cusp_height = ParamLength::from_mm(0.035);
        p.pass_angle_deg = 45.0;
        p.cross_pass = false;
        p.cut_direction = chip_cam::ParallelCutDirection::ZigZag;
    }

    // Op 3: 3D Constant-Scallop Finishing across the wave surface
    let mut op_scallop = Operation::new(
        "op_3_wave_scallop_3d",
        "3D Constant-Scallop Finish",
        OperationKind::Scallop3D,
    );
    if let Some(t) = ball_6mm.as_ref().or(flat_6mm.as_ref()) {
        op_scallop.tool_id = Some(t.id.clone());
        op_scallop.preset_id = t.presets.first().map(|p| p.id.clone());
    }
    op_scallop.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -10.0);
    if let StrategyParams::Scallop3D(p) = op_scallop.strategy_mut() {
        p.boundary_mode = chip_cam::MachiningBoundaryMode::ModelSilhouette;
        p.tool_containment = chip_cam::ToolContainmentMode::ToolCenterOnBoundary;
        p.cusp_height = ParamLength::from_mm(0.04);
        p.max_stepover = ParamLength::from_mm(1.5);
        p.min_stepover = ParamLength::from_mm(0.2);
    }

    proj.setups[0].operations = vec![op_rough, op_par, op_scallop];
    proj
}

/// Builds the Phase 5 **Rotary Indexed Shaft (4th-Axis A)** [`ProjectDocument`]
/// (`Ø40 mm × 80 mm` shaft aligned along WCS `X / A-Axis`, chucked in `Ø42 × 80 mm` cylindrical
/// bar stock with `linuxcnc_4axis` post profile, continuous `Z+X+A` spiral surfacing across the
/// reduced central groove & 3-lobe cam, plus 4-sided `Indexed 3+1` wrench-flat milling at
/// `0° / 90° / 180° / 270°`).
pub fn build_rotary_shaft_preview_project(
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Rotary Indexed Shaft (4th-Axis A)");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        unit_override: None,
    }];
    let setup_id = proj.setups[0].id.clone();

    let setup = &mut proj.setups[0];
    setup.name = "Setup 1 — 4th-Axis A-Chuck (X-Axis)".to_string();
    setup.post_profile = "linuxcnc_4axis".to_string();
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::VOLUMETRIC_CENTER,
    };
    setup.stock = StockConfig::FixedCylinder {
        axis: chip_cam::StockAxis::X,
        diameter: ParamLength::from_mm(42.0),
        length: ParamLength::from_mm(80.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup.rotary.mode = chip_cam::RotaryDriveMode::Simultaneous4Axis;
    setup.rotary.axis = chip_cam::RotaryAxisKind::XAxis;
    setup.rotary.direction = chip_cam::RotaryDirection::RightHandCcw;
    setup.rotary.clearance_radial_offset = ParamLength::from_mm(12.0);
    setup.rotary.chuck_length = ParamLength::from_mm(20.0);
    setup.rotary.chuck_diameter = ParamLength::from_mm(65.0);

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

    let ball_6mm = available_tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::BallEndmill { diameter_mm, .. } if approx_eq(diameter_mm, 6.0, 0.5)
            )
        })
        .or_else(|| {
            available_tools
                .iter()
                .find(|t| matches!(t.geometry, CutterGeometry::BallEndmill { .. }))
        })
        .map(|t| t.id.clone());

    let flat_6mm = available_tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. } if approx_eq(diameter_mm, 6.0, 0.5)
            )
        })
        .or_else(|| {
            available_tools
                .iter()
                .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { .. }))
        })
        .map(|t| t.id.clone());

    // 2D SetupXy sketch window isolating the 4-flat wrenching zone (X in [-25, -6])
    let mut sk_flats = SketchLayer::new("sk_wrench_flats", "Wrench Flat Window", setup_id);
    sk_flats.entities.push(SketchEntityEntry {
        id: "ent_flat_window".to_string(),
        name: "Flat Pocket Window".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(-15.5),
            y: ParamLength::from_mm(0.0),
            width: ParamLength::from_mm(18.0),
            height: ParamLength::from_mm(36.0),
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
        },
    });
    sk_flats.rebuild_regions(None);
    proj.sketches.push(sk_flats);

    // Op 1: Continuous 2-Linear + Rotary Spiral Surfacing (Z+X+A) over Collar & 3-Lobe Cam
    let mut op_spiral = Operation::new(
        "op_1_rotary_spiral",
        "Rotary Spiral — Collar & 3-Lobe Cam",
        OperationKind::Rotary2Linear,
    );
    op_spiral.tool_id = ball_6mm;
    op_spiral.passes.multiple_depths = true;
    op_spiral.passes.max_stepdown = ParamLength::from_mm(3.0);
    if let StrategyParams::Rotary2Linear(p) = op_spiral.strategy_mut() {
        p.pattern = chip_cam::RotaryPatternMode::ContinuousSpiral;
        p.stepover = ParamLength::from_mm(1.6);
        p.axial_margin_start = ParamLength::from_mm(34.0);
        p.axial_margin_end = ParamLength::from_mm(8.0);
        p.angular_sample_deg = 5.0;
        p.axial_sample_step = ParamLength::from_mm(1.0);
    }

    let mut ops = vec![op_spiral];

    // Ops 2..5: Indexed 3+1 Multi-Face Wrenching Flats (0°, 90°, 180°, 270°)
    for (idx, &angle_deg) in [0.0_f64, 90.0, 180.0, 270.0].iter().enumerate() {
        let mut op_flat = Operation::new(
            format!("op_{}_indexed_flat_{}", idx + 2, angle_deg as i32),
            format!("Indexed 3+1 Flat ({angle_deg:.0}°)"),
            OperationKind::Pocket2D,
        );
        op_flat.tool_id = flat_6mm.clone();
        op_flat
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sk_wrench_flats".to_string(),
                sub_layer: None,
                reverse: false,
                side_flip: false,
            });
        op_flat.heights.top = HeightSpec::new(HeightReference::StockTop, 0.5);
        op_flat.heights.bottom = HeightSpec::new(HeightReference::StockTop, -6.0);
        op_flat.passes.multiple_depths = true;
        op_flat.passes.max_stepdown = ParamLength::from_mm(2.0);
        op_flat.rotary = chip_cam::OperationRotaryMode::Indexed3Plus1 { angle_deg };
        ops.push(op_flat);
    }

    proj.setups[0].operations = ops;
    proj
}

/// Builds the Phase 5 **Wrapped Rotary Engraving (`Z+X+A`)** [`ProjectDocument`]:
/// wraps bold TrueType `"CHIP"` V-Carving and a rounded circumferential flute pocket onto a
/// `Ø40 × 80 mm` cylindrical blank with strict cross-axis centerline locking (`Y = 0`),
/// compatible with both `linuxcnc_4axis` (`G93`) and 3-axis `grbl_rotary_wrap` swapped-axis CNCs.
pub fn build_rotary_wrapped_engraving_project(available_tools: &[Tool]) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Wrapped Rotary Engraving (Z+X+A)");
    let setup_id = proj.setups[0].id.clone();

    let setup = &mut proj.setups[0];
    setup.name = "Setup 1 — Wrapped Cylinder (Z+X+A)".to_string();
    setup.post_profile = "linuxcnc_4axis".to_string();
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::VOLUMETRIC_CENTER,
    };
    setup.stock = StockConfig::FixedCylinder {
        axis: chip_cam::StockAxis::X,
        diameter: ParamLength::from_mm(40.0),
        length: ParamLength::from_mm(80.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup.rotary.mode = chip_cam::RotaryDriveMode::TwoLinearPlusRotary;
    setup.rotary.axis = chip_cam::RotaryAxisKind::XAxis;
    setup.rotary.direction = chip_cam::RotaryDirection::RightHandCcw;
    setup.rotary.clearance_radial_offset = ParamLength::from_mm(10.0);
    setup.rotary.chuck_length = ParamLength::from_mm(20.0);
    setup.rotary.chuck_diameter = ParamLength::from_mm(65.0);

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

    let vbit_id = available_tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
        .map(|t| t.id.clone());
    let flat_small_id = available_tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. } if diameter_mm <= 4.0
            )
        })
        .or_else(|| {
            available_tools
                .iter()
                .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { .. }))
        })
        .map(|t| t.id.clone());

    // Sketch 1: Bold "CHIP" outline wrapped around the cylinder circumference (along Y)
    let mut sk_logo = SketchLayer::new("sk_wrap_logo", "Wrapped Logo 'CHIP'", setup_id.clone());
    sk_logo.entities.push(SketchEntityEntry {
        id: "ent_wrap_chip".to_string(),
        name: "Logo 'CHIP'".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Text(SketchTextSpec {
            content: "CHIP".to_string(),
            font: SketchFontSource::BuiltinSansBold,
            cap_height: ParamLength::from_mm(14.0),
            x: ParamLength::from_mm(12.0),
            y: ParamLength::from_mm(0.0),
            rotation_deg: 90.0,
            h_align: TextHAlign::Center,
            v_align: TextVAlign::Middle,
            letter_spacing: ParamLength::from_mm(1.5),
            ..SketchTextSpec::default()
        }),
    });
    sk_logo.rebuild_regions(None);

    // Sketch 2: Rounded decorative circumferential flute pocket
    let mut sk_flute = SketchLayer::new("sk_wrap_flute", "Circumferential Flute", setup_id);
    sk_flute.entities.push(SketchEntityEntry {
        id: "ent_wrap_flute".to_string(),
        name: "Flute Pocket".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(-14.0),
            y: ParamLength::from_mm(0.0),
            width: ParamLength::from_mm(12.0),
            height: ParamLength::from_mm(34.0),
            corner_radius: ParamLength::from_mm(4.0),
            rotation_deg: 0.0,
        },
    });
    sk_flute.rebuild_regions(None);

    proj.sketches = vec![sk_logo, sk_flute];

    let wrap_mode = chip_cam::OperationRotaryMode::WrapCylinder {
        wrap_radius: ParamLength::from_mm(20.0),
        axial_offset: ParamLength::from_mm(0.0),
        angle_offset_deg: 0.0,
    };

    // Op 1: Wrapped V-Carve "CHIP"
    let mut op_vcarve = Operation::new(
        "op_1_wrap_vcarve",
        "Wrapped V-Carve 'CHIP'",
        OperationKind::VCarve,
    );
    op_vcarve.tool_id = vbit_id;
    op_vcarve
        .selections
        .push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sk_wrap_logo".to_string(),
            sub_layer: None,
            reverse: false,
            side_flip: false,
        });
    op_vcarve.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_vcarve.heights.bottom = HeightSpec::new(HeightReference::StockTop, -2.5);
    op_vcarve.rotary = wrap_mode.clone();

    // Op 2: Wrapped Decorative Flute Pocket
    let mut op_flute = Operation::new(
        "op_2_wrap_flute",
        "Wrapped Flute Pocket",
        OperationKind::Pocket2D,
    );
    op_flute.tool_id = flat_small_id;
    op_flute
        .selections
        .push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sk_wrap_flute".to_string(),
            sub_layer: None,
            reverse: false,
            side_flip: false,
        });
    op_flute.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_flute.heights.bottom = HeightSpec::new(HeightReference::StockTop, -1.8);
    op_flute.rotary = wrap_mode;

    proj.setups[0].operations = vec![op_vcarve, op_flute];
    proj
}

/// Builds the Phase 5 **Simultaneous 4-Axis Helical Flute (`X+Y+Z+A`)** [`ProjectDocument`]:
/// `Ø40 × 80 mm` 3-start helical flute shaft chucked on `X / A` (`Simultaneous4Axis`,
/// `linuxcnc_4axis`) with simultaneous off-centerline auto-tilted roughing and finishing passes.
pub fn build_rotary_helical_flute_project(
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Simultaneous 4-Axis Helical Flute (X+Y+Z+A)");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        unit_override: None,
    }];

    let setup = &mut proj.setups[0];
    setup.name = "Setup 1 — Simultaneous 4-Axis A-Chuck".to_string();
    setup.post_profile = "linuxcnc_4axis".to_string();
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::VOLUMETRIC_CENTER,
    };
    setup.stock = StockConfig::FixedCylinder {
        axis: chip_cam::StockAxis::X,
        diameter: ParamLength::from_mm(42.0),
        length: ParamLength::from_mm(80.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup.rotary.mode = chip_cam::RotaryDriveMode::Simultaneous4Axis;
    setup.rotary.axis = chip_cam::RotaryAxisKind::XAxis;
    setup.rotary.direction = chip_cam::RotaryDirection::RightHandCcw;
    setup.rotary.clearance_radial_offset = ParamLength::from_mm(12.0);
    setup.rotary.chuck_length = ParamLength::from_mm(20.0);
    setup.rotary.chuck_diameter = ParamLength::from_mm(65.0);

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

    let ball_6mm = available_tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::BallEndmill { diameter_mm, .. } if approx_eq(diameter_mm, 6.0, 0.5)
            )
        })
        .or_else(|| {
            available_tools
                .iter()
                .find(|t| matches!(t.geometry, CutterGeometry::BallEndmill { .. }))
        })
        .map(|t| t.id.clone());

    // Op 1: Multi-depth Simultaneous 4-Axis Helical Roughing
    let mut op_rough = Operation::new(
        "op_1_4ax_rough",
        "4-Axis Helical Roughing (Auto-Tilt)",
        OperationKind::Rotary4AxisSim,
    );
    op_rough.tool_id = ball_6mm.clone();
    op_rough.passes.multiple_depths = true;
    op_rough.passes.max_stepdown = ParamLength::from_mm(3.0);
    op_rough.passes.stock_to_leave_radial = ParamLength::from_mm(0.25);
    op_rough.passes.stock_to_leave_axial = ParamLength::from_mm(0.25);
    if let StrategyParams::Rotary4AxisSim(p) = op_rough.strategy_mut() {
        p.pattern = chip_cam::Rotary4AxisPattern::SimultaneousSpiral;
        p.tilt_mode = chip_cam::RotaryTiltMode::SurfaceNormalAutoTilt;
        p.max_tilt_deg = 16.0;
        p.tilt_smoothing_window = 3;
        p.stepover = ParamLength::from_mm(2.6);
        p.axial_margin_start = ParamLength::from_mm(11.0);
        p.axial_margin_end = ParamLength::from_mm(11.0);
    }

    // Op 2: Simultaneous 4-Axis Helical Finishing (off-centerline smoothed auto-tilt)
    let mut op_finish = Operation::new(
        "op_2_4ax_finish",
        "4-Axis Helical Finishing (Auto-Tilt)",
        OperationKind::Rotary4AxisSim,
    );
    op_finish.tool_id = ball_6mm;
    op_finish.passes.multiple_depths = false;
    op_finish.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
    op_finish.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    if let StrategyParams::Rotary4AxisSim(p) = op_finish.strategy_mut() {
        p.pattern = chip_cam::Rotary4AxisPattern::SimultaneousSpiral;
        p.tilt_mode = chip_cam::RotaryTiltMode::SurfaceNormalAutoTilt;
        p.max_tilt_deg = 18.0;
        p.tilt_smoothing_window = 3;
        p.stepover = ParamLength::from_mm(1.5);
        p.axial_margin_start = ParamLength::from_mm(11.0);
        p.axial_margin_end = ParamLength::from_mm(11.0);
    }

    proj.setups[0].operations = vec![op_rough, op_finish];
    proj
}
