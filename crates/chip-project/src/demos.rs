use chip_cad::{
    CadDocument, OrientedEdgeUse, SurfaceKind, TopologyGraph, find_same_diameter_holes,
};
use chip_cam::{
    ChamferSourceMode, ChamferTraceMode, GeometrySelectionItem, HeightReference, HeightSpec,
    HoleMachiningMode, Operation, OperationKind, StrategyParams, TabProfileKind,
};
use chip_math::{ParamLength, approx_eq};
use chip_tools::{CutterGeometry, Tool};

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
