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
        transform: chip_cad::ModelTransform::IDENTITY,
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
        transform: chip_cad::ModelTransform::IDENTITY,
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
        transform: chip_cad::ModelTransform::IDENTITY,
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
        transform: chip_cad::ModelTransform::IDENTITY,
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

/// Builds the Milestone 47 **Threaded Manifold Block** (`--sample thread-mill`) [`ProjectDocument`]
/// on [`chip_cad::sample_threaded_manifold_step`] (`80 × 60 × 20 mm`, 6 operations in `G54`):
/// 1. `Face Stock Top` (`Facing`, `T7` Ø38 mm Face Mill): decks raw billet from `Z = 21 -> 20 mm`.
/// 2. `Clear Deck Around Boss` (`Pocket2D`, `T1` Ø6 mm Flat Endmill): clears open manifold deck
///    `Z = 20 -> 12 mm` leaving the proud `Ø12.0 mm` circular boss standing cleanly.
/// 3. `Bore Pilot Holes` (`DrillBore` / `HelicalBore`, `T3` 1/8" Flat Endmill): helically bores
///    2× `Ø6.80 mm` (`M8 × 1.25`) and 2× `Ø5.11 mm` (`1/4-20 UNC`) through pilot holes.
/// 4. `Internal M8 x 1.25 Thread Mill` (`ThreadMill`, `T9` Ø4.75 mm Single-Point): bottom-up climb
///    (`G3`), 2 radial passes + 1 spring pass.
/// 5. `Internal 1/4-20 UNC Thread Mill` (`ThreadMill`, `T9` Ø4.75 mm Single-Point): bottom-up climb
///    (`G3`), 2 radial passes + 1 spring pass.
/// 6. `External M12 x 1.75 Boss Thread` (`ThreadMill`, `T11` Ø8.0 mm Single-Point): top-down climb
///    (`G2`) around the proud `Ø12.0 mm` boss, 2 radial passes + 1 spring pass.
pub fn build_thread_mill_manifold_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    use chip_cam::{ThreadCutDirection, ThreadFormKind, ThreadHandedness, ThreadMillParams};

    let mut proj = ProjectDocument::new("Threaded Manifold Block (Helical Thread Milling)");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    }];

    let mut setup = Setup::new(
        "setup_1",
        "Setup 1 — Threaded Manifold (G54)",
        WorkOffsetSlot::G54,
    );
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_FRONT_LEFT,
    };
    setup.stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(0.0),
        side_neg_x: ParamLength::from_mm(0.0),
        side_pos_y: ParamLength::from_mm(0.0),
        side_neg_y: ParamLength::from_mm(0.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(20.0),
        round_increment_mm: None,
    };

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

    // Dedicated Ø8.0 mm single-point thread mill with 1.60 mm radial tooth height for coarse
    // external M12 x 1.75 boss threads (keeps relieved neck Ø4.8 mm > 0.5 mm clear of boss OD).
    let t11_ext_tm = Tool {
        id: ToolId("t11_8mm_single_thread_mill".to_string()),
        tool_number: 11,
        length_offset_number: 11,
        diameter_offset_number: 11,
        name: "8.0mm 3FL 60° Single-Point Thread Mill (M10–M24 / Ext)".to_string(),
        vendor: Some("Generic Carbide".to_string()),
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::Mist,
        stickout_length_mm: 26.0,
        geometry: CutterGeometry::ThreadMill {
            diameter_mm: 8.0,
            neck_diameter_mm: 4.8,
            neck_length_mm: 18.0,
            pitch_mm: 1.75,
            teeth_count: 1,
            thread_angle_deg: 60.0,
            flute_length_mm: 2.0,
            overall_length_mm: 55.0,
            shank_diameter_mm: 8.0,
            flutes: 3,
        },
        presets: vec![CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
            id: "al6061_ext_thread",
            name: "Aluminum 6061 - External Threading",
            material: "Aluminum 6061",
            diameter_mm: 8.0,
            flutes: 3,
            surface_speed_m_min: 190.0,
            chipload_mm_tooth: 0.025,
            default_stepdown_mm: 1.75,
            default_stepover_mm: 0.6,
            coolant_override: Some(CoolantMode::Mist),
        })],
    };
    proj.ensure_tool_embedded(&t11_ext_tm);

    let face_38mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FaceMill { .. }))
        .cloned();
    let flat_6mm = proj
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. } if approx_eq(diameter_mm, 6.0, 0.1)
            )
        })
        .cloned();
    let flat_3175mm = proj
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. } if (2.5..4.5).contains(&diameter_mm)
            )
        })
        .cloned();
    let tm_475mm = proj
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::ThreadMill {
                    diameter_mm,
                    teeth_count: 1,
                    ..
                } if (4.0..4.95).contains(&diameter_mm)
            )
        })
        .cloned();

    // Op 1: Face Stock Top (Z = 21 -> 20 mm)
    if let Some(fm) = &face_38mm {
        let mut op_face = Operation::new("op_1_face_top", "Face Stock Top", OperationKind::Facing);
        op_face.tool_id = Some(fm.id.clone());
        op_face.preset_id = fm.presets.first().map(|p| p.id.clone());
        setup.operations.push(op_face);
    }

    // Op 2: Clear Manifold Deck Around Proud Ø12 mm Boss (Z = 20 -> 12 mm)
    if let Some(deck_face) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && approx_eq(origin_mm.z, 12.0, 0.05)
        )
    }) && let Some(bind) = doc.bind_face(deck_face.id)
        && let Some(em) = &flat_6mm
    {
        let mut op_deck = Operation::new(
            "op_2_clear_boss_deck",
            "Clear Deck Around Boss",
            OperationKind::Pocket2D,
        );
        op_deck.tool_id = Some(em.id.clone());
        op_deck.preset_id = em.presets.first().map(|p| p.id.clone());
        op_deck.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op_deck.heights.bottom = HeightSpec::new(HeightReference::FromSelection, 0.0);
        op_deck.passes.multiple_depths = true;
        op_deck.passes.max_stepdown = ParamLength::from_mm(4.0);
        op_deck.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op_deck.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        op_deck
            .selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        setup.operations.push(op_deck);
    }

    // Collect Ø6.80 mm (M8) and Ø5.11 mm (1/4-20 UNC) pilot hole faces
    let m8_hole_faces: Vec<_> = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if approx_eq(radius_mm, 3.40, 0.03)
            )
        })
        .map(|seed| find_same_diameter_holes(doc, seed.id, 1e-3, 1e-3))
        .unwrap_or_default();

    let unc_hole_faces: Vec<_> = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if approx_eq(radius_mm, 2.555, 0.03)
            )
        })
        .map(|seed| find_same_diameter_holes(doc, seed.id, 1e-3, 1e-3))
        .unwrap_or_default();

    // Op 3: Helical Bore all 4 Pilot Holes (2× Ø6.80 mm + 2× Ø5.11 mm) with T3 1/8" Flat Endmill
    if let Some(em_small) = &flat_3175mm
        && (!m8_hole_faces.is_empty() || !unc_hole_faces.is_empty())
    {
        let mut op_bore = Operation::new(
            "op_3_bore_pilots",
            "Bore Pilot Holes (Ø6.8 & Ø5.1 mm)",
            OperationKind::DrillBore,
        );
        op_bore.tool_id = Some(em_small.id.clone());
        op_bore.preset_id = em_small.presets.first().map(|p| p.id.clone());
        op_bore.heights.top = HeightSpec::new(HeightReference::FromSelection, 0.0);
        op_bore.heights.bottom = HeightSpec::new(HeightReference::FromSelection, 0.0);
        op_bore.passes.multiple_depths = true;
        op_bore.passes.max_stepdown = ParamLength::from_mm(4.0);
        if let StrategyParams::DrillBore(p) = op_bore.strategy_mut() {
            p.mode = HoleMachiningMode::HelicalBore;
            p.bore_ramp_angle_deg = 4.0;
            p.bore_finish_pass = true;
            p.bore_finish_stepover = ParamLength::from_mm(0.15);
            p.optimize_hole_order = true;
        }
        for &fid in m8_hole_faces.iter().chain(unc_hole_faces.iter()) {
            if let Some(bind) = doc.bind_face(fid) {
                op_bore
                    .selections
                    .push(GeometrySelectionItem::CylindricalHole { binding: bind });
            }
        }
        setup.operations.push(op_bore);
    }

    // Op 4: Internal M8 x 1.25 Thread Mill (single-point, bottom-up climb G3, 2 radial + 1 spring)
    if let Some(tm) = &tm_475mm
        && !m8_hole_faces.is_empty()
    {
        let mut op_m8 = Operation::new(
            "op_4_thread_m8",
            "Internal M8 x 1.25 Thread Mill",
            OperationKind::ThreadMill,
        );
        op_m8.tool_id = Some(tm.id.clone());
        op_m8.preset_id = tm.presets.first().map(|p| p.id.clone());
        op_m8.heights.top = HeightSpec::new(HeightReference::FromSelection, 0.0);
        op_m8.heights.bottom = HeightSpec::new(HeightReference::FromSelection, 2.0);
        op_m8.strategy = Some(StrategyParams::ThreadMill(ThreadMillParams {
            form: ThreadFormKind::InternalHole,
            handedness: ThreadHandedness::RightHand,
            cut_direction: ThreadCutDirection::Climb,
            standard_preset: Some("M8 x 1.25".to_string()),
            major_diameter: ParamLength::from_mm(8.0),
            pitch: ParamLength::from_mm(1.25),
            pitch_diameter_offset: ParamLength::from_mm(0.0),
            radial_passes: 2,
            spring_passes: 1,
            lead_in_clearance: ParamLength::from_mm(0.5),
            compensate_centerline_feed: true,
            optimize_hole_order: true,
        }));
        for &fid in &m8_hole_faces {
            if let Some(bind) = doc.bind_face(fid) {
                op_m8
                    .selections
                    .push(GeometrySelectionItem::CylindricalHole { binding: bind });
            }
        }
        setup.operations.push(op_m8);
    }

    // Op 5: Internal 1/4-20 UNC Thread Mill (single-point, bottom-up climb G3, 2 radial + 1 spring)
    if let Some(tm) = &tm_475mm
        && !unc_hole_faces.is_empty()
    {
        let mut op_unc = Operation::new(
            "op_5_thread_unc",
            "Internal 1/4-20 UNC Thread Mill",
            OperationKind::ThreadMill,
        );
        op_unc.tool_id = Some(tm.id.clone());
        op_unc.preset_id = tm.presets.first().map(|p| p.id.clone());
        op_unc.heights.top = HeightSpec::new(HeightReference::FromSelection, 0.0);
        op_unc.heights.bottom = HeightSpec::new(HeightReference::FromSelection, 2.0);
        op_unc.strategy = Some(StrategyParams::ThreadMill(ThreadMillParams {
            form: ThreadFormKind::InternalHole,
            handedness: ThreadHandedness::RightHand,
            cut_direction: ThreadCutDirection::Climb,
            standard_preset: Some("1/4-20 UNC".to_string()),
            major_diameter: ParamLength::from_mm(6.35),
            pitch: ParamLength::from_mm(1.27),
            pitch_diameter_offset: ParamLength::from_mm(0.0),
            radial_passes: 2,
            spring_passes: 1,
            lead_in_clearance: ParamLength::from_mm(0.5),
            compensate_centerline_feed: true,
            optimize_hole_order: true,
        }));
        for &fid in &unc_hole_faces {
            if let Some(bind) = doc.bind_face(fid) {
                op_unc
                    .selections
                    .push(GeometrySelectionItem::CylindricalHole { binding: bind });
            }
        }
        setup.operations.push(op_unc);
    }

    // Op 6: External M12 x 1.75 Boss Thread (top-down climb G2, 2 radial + 1 spring)
    if let Some(boss_top) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && approx_eq(origin_mm.z, 20.0, 0.05)
        )
    }) && let Some(bind) = doc.bind_face(boss_top.id)
    {
        let mut op_ext = Operation::new(
            "op_6_thread_m12_boss",
            "External M12 x 1.75 Boss Thread",
            OperationKind::ThreadMill,
        );
        op_ext.tool_id = Some(t11_ext_tm.id.clone());
        op_ext.preset_id = t11_ext_tm.presets.first().map(|p| p.id.clone());
        op_ext.heights.top = HeightSpec::new(HeightReference::FromSelection, 0.0);
        op_ext.heights.bottom = HeightSpec::new(HeightReference::FromSelection, 1.0);
        op_ext.strategy = Some(StrategyParams::ThreadMill(ThreadMillParams {
            form: ThreadFormKind::ExternalBoss,
            handedness: ThreadHandedness::RightHand,
            cut_direction: ThreadCutDirection::Climb,
            standard_preset: Some("M12 x 1.75".to_string()),
            major_diameter: ParamLength::from_mm(12.0),
            pitch: ParamLength::from_mm(1.75),
            pitch_diameter_offset: ParamLength::from_mm(0.0),
            radial_passes: 2,
            spring_passes: 1,
            lead_in_clearance: ParamLength::from_mm(0.5),
            compensate_centerline_feed: true,
            optimize_hole_order: true,
        }));
        op_ext
            .selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        setup.operations.push(op_ext);
    }

    proj.setups = vec![setup];
    proj
}
