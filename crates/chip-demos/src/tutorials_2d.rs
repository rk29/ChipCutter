use chip_cad::{CadDocument, SurfaceKind, find_same_diameter_holes};
use chip_cam::{
    AdaptiveClearingParams, BoxAnchor27, ChamferSourceMode, ChamferTraceMode, ContourCompensation,
    FixedBoxXyAlign, FixedBoxZAlign, GeometrySelectionItem, HeightReference, HeightSpec,
    HoleMachiningMode, Operation, OperationKind, PocketStrategyKind, RestMachiningConfig,
    RestMachiningMode, Setup, StockConfig, StrategyParams, TabProfileKind, VCarveClearingKind,
    VCarveInlayMode, WcsOriginMode, WorkOffsetSlot,
};
use chip_math::{DisplayUnit, ParamLength, approx_eq};
use chip_sketch::{
    HersheyFontKind, SketchEntity, SketchEntityEntry, SketchFontSource, SketchLayer,
    SketchTextSpec, TextHAlign, TextVAlign,
};
use chip_tools::{
    CoolantMode, CutterGeometry, CuttingPreset, SpindleDirection, SurfaceSpeedPresetSpec, Tool,
    ToolId,
};

use chip_project::{ModelEntry, ProjectDocument};

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
        role: Default::default(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
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
