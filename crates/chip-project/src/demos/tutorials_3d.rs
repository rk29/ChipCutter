use chip_cad::{CadDocument, SurfaceKind};
use chip_cam::{
    BoxAnchor27, GeometrySelectionItem, HeightReference, HeightSpec, Operation, OperationKind,
    RestMachiningConfig, RestMachiningMode, Setup, StockConfig, StrategyParams, WcsOriginMode,
    WorkOffsetSlot,
};
use chip_math::{ParamLength, approx_eq};
use chip_sketch::{RectAnchorMode, SketchEntity, SketchEntityEntry, SketchLayer};
use chip_tools::{CutterGeometry, Tool};

use crate::document::{ModelEntry, ProjectDocument};

/// Builds the dedicated **Tutorial 08 — 3D Adaptive Roughing & Waterline / Z-Level**
/// [`ProjectDocument`] for `08-sculpted-dome.stl` (`70 × 70 × 33 mm` watertight hemisphere
/// `R = 25 mm` on an `8 mm` square pedestal):
/// - 2D `SetupXy` containment sketch (`Dome Window`, `68 × 68 mm` rounded rectangle centered on WCS),
/// - **Op 1:** `3D Adaptive Roughing` (`T1` Ø6 mm Flat Endmill, `SelectedRegions`, flat-area
///   detection, `0.4 mm` radial / `0.25 mm` axial stock-to-leave),
/// - **Op 2:** `3D Waterline Finishing` (`T4` Ø6 mm Ball Endmill, `Adaptive Shallow Z-Stepdowns = true`,
///   `WaterlineOrderMode::ByLevel` top-down, `0.0 mm` stock-to-leave).
pub fn build_tutorial_3d_adaptive_waterline_project(
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Tutorial 08 — 3D Adaptive & Waterline");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        role: Default::default(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    }];
    let setup_id = proj.setups[0].id.clone();
    proj.setups[0].name = "Setup 1 — Sculpted Dome (3D Mesh)".to_string();
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

    // 2D SetupXy containment boundary sketch centered on the dome pedestal
    let mut sk_window = SketchLayer::new("sketch_dome_window", "Dome Window", setup_id);
    sk_window.entities.push(SketchEntityEntry {
        id: "ent_dome_window".to_string(),
        name: "Dome Containment Window (68x68mm)".to_string(),
        sub_layer: Some("BOUNDARY".to_string()),
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(0.0),
            y: ParamLength::from_mm(0.0),
            width: ParamLength::from_mm(68.0),
            height: ParamLength::from_mm(68.0),
            corner_radius: ParamLength::from_mm(4.0),
            rotation_deg: 0.0,
        },
    });
    sk_window.rebuild_regions(None);
    proj.sketches.push(sk_window);

    let flat_6mm = proj
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5
            )
        })
        .or_else(|| proj.tools.first())
        .cloned();
    let ball_6mm = proj
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::BallEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5
            )
        })
        .cloned();

    // Op 1: 3D Adaptive Roughing constrained by the 2D `Dome Window` sketch
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
    op_rough.passes.stock_to_leave_radial = ParamLength::from_mm(0.4);
    op_rough.passes.stock_to_leave_axial = ParamLength::from_mm(0.25);
    op_rough
        .selections
        .push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sketch_dome_window".to_string(),
            sub_layer: None,
            reverse: false,
            side_flip: false,
        });
    if let StrategyParams::Adaptive3D(p) = op_rough.strategy_mut() {
        p.boundary_mode = chip_cam::MachiningBoundaryMode::SelectedRegions;
        p.tool_containment = chip_cam::ToolContainmentMode::ToolOutsideBoundary;
        p.boundary_offset = ParamLength::from_mm(0.5);
        p.max_roughing_stepdown = ParamLength::from_mm(5.0);
        p.fine_stepdown = ParamLength::from_mm(1.5);
        p.adaptive.optimal_load = ParamLength::from_mm(1.8);
        p.detect_flats = true;
    }

    // Op 2: 3D Waterline / Z-Level Finishing (`Adaptive Stepdown = true`, `ByLevel` top-down)
    let mut op_wl = Operation::new(
        "op_2_waterline_3d",
        "3D Waterline Finishing",
        OperationKind::Waterline3D,
    );
    if let Some(t) = ball_6mm.as_ref().or(flat_6mm.as_ref()) {
        op_wl.tool_id = Some(t.id.clone());
        op_wl.preset_id = t.presets.first().map(|p| p.id.clone());
    }
    op_wl.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -25.0);
    op_wl.passes.max_stepdown = ParamLength::from_mm(1.5);
    op_wl.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
    op_wl.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    if let StrategyParams::Waterline3D(p) = op_wl.strategy_mut() {
        p.boundary_mode = chip_cam::MachiningBoundaryMode::ModelSilhouette;
        p.tool_containment = chip_cam::ToolContainmentMode::ToolOutsideBoundary;
        p.boundary_offset = ParamLength::from_mm(0.5);
        p.adaptive_stepdown = true;
        p.min_stepdown = ParamLength::from_mm(0.4);
        p.max_cusp_height = ParamLength::from_mm(0.05);
        p.detect_flats = true;
        p.order_mode = chip_cam::WaterlineOrderMode::ByLevel;
        p.min_slope_deg = 0.0;
        p.max_slope_deg = 90.0;
        p.climb_milling = true;
    }

    proj.setups[0].operations = vec![op_rough, op_wl];
    proj
}

/// Builds the dedicated **Tutorial 09 — 3D Surface Finishing (`Parallel3D`, `Scallop3D`, & 3D Rest)**
/// [`ProjectDocument`] for `09-relief-wave.obj` (`80 × 60 × 13.5 mm` sculpted sinusoidal relief):
/// - **Op 1:** `3D Adaptive Roughing` (`T1` Ø6 mm Flat Endmill, `0.4 mm` radial / `0.25 mm` axial stock-to-leave),
/// - **Op 2:** `3D Parallel Finishing (45° + Cross)` (`T4` Ø6 mm Ball Endmill, `Use Target Cusp Height = 0.04 mm`,
///   `Pass Angle = 45°`, `Cross-Pass +90° = true`, `Slope Range 0°–60°`),
/// - **Op 3:** `3D Constant-Scallop (Rest)` (`T4` Ø6 mm Ball Endmill, `Inside -> Out = true`,
///   `Min/Max Stepover = 0.25 / 1.5 mm`, and `3D Rest Machining` `FromPreviousTool`).
pub fn build_tutorial_3d_surface_finishing_project(
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Tutorial 09 — 3D Surface Finishing");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        role: Default::default(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    }];
    proj.setups[0].name = "Setup 1 — Relief Wave (3D Finishing)".to_string();
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
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5
            )
        })
        .or_else(|| proj.tools.first())
        .cloned();
    let ball_6mm = proj
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::BallEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5
            )
        })
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

    // Op 2: 3D Parallel Finishing (`45°`, cusp height `0.04 mm`, `Cross-Pass +90°`, `0°–60°` slope)
    let mut op_par = Operation::new(
        "op_2_wave_parallel_3d",
        "3D Parallel Finishing (45° + Cross)",
        OperationKind::Parallel3D,
    );
    if let Some(t) = ball_6mm.as_ref().or(flat_6mm.as_ref()) {
        op_par.tool_id = Some(t.id.clone());
        op_par.preset_id = t.presets.first().map(|p| p.id.clone());
    }
    op_par.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -10.0);
    op_par.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
    op_par.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    if let StrategyParams::Parallel3D(p) = op_par.strategy_mut() {
        p.boundary_mode = chip_cam::MachiningBoundaryMode::ModelSilhouette;
        p.tool_containment = chip_cam::ToolContainmentMode::ToolCenterOnBoundary;
        p.use_cusp_height = true;
        p.cusp_height = ParamLength::from_mm(0.04);
        p.pass_angle_deg = 45.0;
        p.cross_pass = true;
        p.cut_direction = chip_cam::ParallelCutDirection::ZigZag;
        p.min_slope_deg = 0.0;
        p.max_slope_deg = 60.0;
    }

    // Op 3: 3D Constant-Scallop (`Inside -> Out`, `Min/Max Stepover`, `3D Rest Machining` `FromPreviousTool`)
    let mut op_scallop = Operation::new(
        "op_3_wave_scallop_3d",
        "3D Constant-Scallop (Rest)",
        OperationKind::Scallop3D,
    );
    if let Some(t) = ball_6mm.as_ref().or(flat_6mm.as_ref()) {
        op_scallop.tool_id = Some(t.id.clone());
        op_scallop.preset_id = t.presets.first().map(|p| p.id.clone());
    }
    op_scallop.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -10.0);
    op_scallop.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
    op_scallop.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    op_scallop.passes.rest_machining = RestMachiningConfig {
        mode: RestMachiningMode::FromPreviousTool {
            prev_tool_diameter: ParamLength::from_mm(12.0),
            prev_stock_to_leave: ParamLength::from_mm(0.25),
        },
        rest_overlap: ParamLength::from_mm(0.5),
        min_rest_thickness: ParamLength::from_mm(0.05),
    };
    if let StrategyParams::Scallop3D(p) = op_scallop.strategy_mut() {
        p.boundary_mode = chip_cam::MachiningBoundaryMode::ModelSilhouette;
        p.tool_containment = chip_cam::ToolContainmentMode::ToolCenterOnBoundary;
        p.cusp_height = ParamLength::from_mm(0.04);
        p.max_stepover = ParamLength::from_mm(1.5);
        p.min_stepover = ParamLength::from_mm(0.25);
        p.inside_out = true;
        p.climb_milling = true;
    }

    proj.setups[0].operations = vec![op_rough, op_par, op_scallop];
    proj
}

/// Builds the **Tutorial 16 (Part A)** project for `16-yup-bracket.stl`:
/// - When `oriented_and_scaled == false`: raw Y-up sideways STL before orientation (empty ops).
/// - When `oriented_and_scaled == true`: applies `Y-Up -> Z-Up (+90° X)`, scales `1.5×`
///   (`90 × 60 × 27 mm`), centers XY, drops bottom to `Z = 0`, and seeds a 3D Adaptive Roughing +
///   3D Parallel Finish setup ready to simulate and post-process.
pub fn build_tutorial_16_yup_stl_project(
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
    oriented_and_scaled: bool,
) -> ProjectDocument {
    use chip_cad::{ModelTransform, load_mesh_cad_document, tutorial_yup_bracket_stl_bytes};

    let mut proj = ProjectDocument::new("Tutorial 16 — Y-Up STL Model Transform & Scaling");
    proj.schema_version = 3;

    let transform = if oriented_and_scaled {
        let base_doc = load_mesh_cad_document(
            "16-yup-bracket.stl",
            &tutorial_yup_bracket_stl_bytes(),
            &chip_cad::MeshImportOptions::default(),
        )
        .ok();
        let mut xf = ModelTransform::y_up_to_z_up().with_uniform_scale(1.5);
        if let Some(raw) = base_doc {
            let td = raw.transformed(&xf);
            xf = xf
                .centered_xy_for_aabb(td.aabb_mm)
                .dropped_bottom_to_z0_for_aabb(td.aabb_mm);
        }
        xf
    } else {
        ModelTransform::IDENTITY
    };

    proj.models.push(ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        role: Default::default(),
        unit_override: Some(chip_cad::MeshUnitKind::Millimeter),
        transform,
    });

    let mut setup = Setup::new(
        "setup_1",
        "Setup 1 — 3D Sculpted Boss (G54)",
        WorkOffsetSlot::G54,
    );
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(3.0),
        side_neg_x: ParamLength::from_mm(3.0),
        side_pos_y: ParamLength::from_mm(3.0),
        side_neg_y: ParamLength::from_mm(3.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(4.0),
        round_increment_mm: None,
        sheet_thickness_z: ParamLength::from_mm(20.0),
    };

    if oriented_and_scaled {
        let flat_6 = available_tools
            .iter()
            .find(|t| {
                matches!(
                    t.geometry,
                    CutterGeometry::FlatEndmill { diameter_mm, .. } if approx_eq(diameter_mm, 6.0, 0.3)
                )
            })
            .or_else(|| {
                available_tools
                    .iter()
                    .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { .. }))
            });
        let ball_6 = available_tools
            .iter()
            .find(|t| {
                matches!(
                    t.geometry,
                    CutterGeometry::BallEndmill { diameter_mm, .. } if approx_eq(diameter_mm, 6.0, 0.3)
                )
            })
            .or_else(|| {
                available_tools
                    .iter()
                    .find(|t| matches!(t.geometry, CutterGeometry::BallEndmill { .. }))
            });

        if let Some(em) = flat_6 {
            proj.ensure_tool_embedded(em);
            let mut rough = Operation::new(
                "op_1_adaptive_3d",
                "3D Adaptive Roughing",
                OperationKind::Adaptive3D,
            );
            rough.tool_id = Some(em.id.clone());
            rough.preset_id = em.presets.first().map(|p| p.id.clone());
            rough.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
            rough.passes.stock_to_leave_radial = ParamLength::from_mm(0.3);
            rough.passes.stock_to_leave_axial = ParamLength::from_mm(0.25);
            if let StrategyParams::Adaptive3D(p) = rough.strategy_mut() {
                p.max_roughing_stepdown = ParamLength::from_mm(6.0);
                p.fine_stepdown = ParamLength::from_mm(3.0);
                p.adaptive.optimal_load = ParamLength::from_mm(2.2);
            }
            setup.operations.push(rough);
        }
        if let Some(bm) = ball_6 {
            proj.ensure_tool_embedded(bm);
            let mut fin = Operation::new(
                "op_2_parallel_3d",
                "3D Parallel Surface Finish",
                OperationKind::Parallel3D,
            );
            fin.tool_id = Some(bm.id.clone());
            fin.preset_id = bm.presets.first().map(|p| p.id.clone());
            fin.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
            if let StrategyParams::Parallel3D(p) = fin.strategy_mut() {
                p.boundary_mode = chip_cam::MachiningBoundaryMode::ModelSilhouette;
                p.tool_containment = chip_cam::ToolContainmentMode::ToolInsideBoundary;
                p.use_cusp_height = false;
                p.stepover = ParamLength::from_mm(2.0);
                p.pass_angle_deg = 0.0;
                p.cut_direction = chip_cam::ParallelCutDirection::ZigZag;
            }
            setup.operations.push(fin);
        }
    }

    proj.setups = vec![setup];
    proj
}

/// Builds the **Tutorial 16 (Part C)** 2-Setup project for `16-tilted-bracket.step`:
/// - **Setup 1 (`G54` — Op 1 Top Features):** Aligns WCS `+Z` & `+X` to the tilted `30° X / 25° Z`
///   bracket via `SelectZAndX`, with Facing + Top Pocket + 4× Mounting Holes.
/// - **Setup 2 (`G55` — Op 2 Flipped Bottom Face):** Flips `+Z` (`flip_z: false`) onto the underside
///   with a bottom Facing pass so both sides are machined from a single untouched STEP file.
pub fn build_tutorial_16_tilted_step_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    use chip_cam::{WcsAxisRef, WcsOrientationMode};

    let mut proj = ProjectDocument::new("Tutorial 16 — Tilted STEP WCS & Op-2 Flip");
    proj.models.push(ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        role: Default::default(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    });

    let bottom_face = doc
        .faces
        .iter()
        .filter(|f| matches!(f.surface, SurfaceKind::Plane { .. }))
        .max_by(|a, b| a.area_mm2.total_cmp(&b.area_mm2));

    let longest_bottom_edge = bottom_face.and_then(|bf| {
        let lp = doc.brep_loop(bf.outer_loop?)?;
        lp.edges
            .iter()
            .filter_map(|u| doc.edge(u.edge_id))
            .max_by(|a, b| a.length_mm.total_cmp(&b.length_mm))
    });

    let mut setup1 = Setup::new(
        "setup_1",
        "Setup 1 — Op 1 Top Pocket & Holes (G54)",
        WorkOffsetSlot::G54,
    );
    let mut setup2 = Setup::new(
        "setup_2",
        "Setup 2 — Op 2 Flipped Underside (G55)",
        WorkOffsetSlot::G55,
    );

    if let (Some(bf), Some(be)) = (bottom_face, longest_bottom_edge)
        && let (Some(bf_bind), Some(be_bind)) = (doc.bind_face(bf.id), doc.bind_edge(be.id))
    {
        setup1.wcs.orientation = WcsOrientationMode::SelectZAndX {
            z_ref: WcsAxisRef::FaceAxis(bf_bind.clone()),
            flip_z: true,
            x_ref: Some(WcsAxisRef::EdgeDirection(be_bind.clone())),
            flip_x: false,
            clock_90: 0,
            extra_z_rot_deg: 0.0,
        };
        setup2.wcs.orientation = WcsOrientationMode::SelectZAndX {
            z_ref: WcsAxisRef::FaceAxis(bf_bind),
            flip_z: false,
            x_ref: Some(WcsAxisRef::EdgeDirection(be_bind)),
            flip_x: false,
            clock_90: 0,
            extra_z_rot_deg: 0.0,
        };
    }

    setup1.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup1.stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(2.0),
        side_neg_x: ParamLength::from_mm(2.0),
        side_pos_y: ParamLength::from_mm(2.0),
        side_neg_y: ParamLength::from_mm(2.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(1.5),
        bottom_offset_z: ParamLength::from_mm(2.0),
        round_increment_mm: None,
        sheet_thickness_z: ParamLength::from_mm(20.0),
    };

    setup2.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup2.stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(0.0),
        side_neg_x: ParamLength::from_mm(0.0),
        side_pos_y: ParamLength::from_mm(0.0),
        side_neg_y: ParamLength::from_mm(0.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(2.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        round_increment_mm: None,
        sheet_thickness_z: ParamLength::from_mm(20.0),
    };

    let flat_6 = available_tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. } if approx_eq(diameter_mm, 6.0, 0.3)
            )
        })
        .or_else(|| {
            available_tools
                .iter()
                .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { .. }))
        });
    let face_mill = available_tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FaceMill { .. }))
        .or(flat_6);
    let drill_6 = available_tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::TwistDrill { diameter_mm, .. } if approx_eq(diameter_mm, 6.0, 0.5)
            )
        })
        .or_else(|| {
            available_tools
                .iter()
                .find(|t| matches!(t.geometry, CutterGeometry::TwistDrill { .. }))
        });

    if let Some(fm) = face_mill {
        proj.ensure_tool_embedded(fm);
        let mut face_op1 = Operation::new("op_1_face_top", "Face Top Stock", OperationKind::Facing);
        face_op1.tool_id = Some(fm.id.clone());
        face_op1.preset_id = fm.presets.first().map(|p| p.id.clone());
        setup1.operations.push(face_op1);

        let mut face_op2 = Operation::new(
            "op_2_face_underside",
            "Face Underside Hat Stock",
            OperationKind::Facing,
        );
        face_op2.tool_id = Some(fm.id.clone());
        face_op2.preset_id = fm.presets.first().map(|p| p.id.clone());
        setup2.operations.push(face_op2);
    }

    // Resolve Setup 1 frame so we can locate the horizontal pocket floor & top-aligned holes
    if let Ok(rs1) = setup1.resolve(doc, proj.display_units) {
        if let Some(em) = flat_6 {
            proj.ensure_tool_embedded(em);
            let mut pocket_faces: Vec<(&chip_cad::BRepFace, f64)> = doc
                .faces
                .iter()
                .filter_map(|f| match f.surface {
                    SurfaceKind::Plane { origin_mm, normal } => {
                        let n_wcs = rs1.frame.world_to_local_dir(normal).normalize_or_zero();
                        let p_wcs = rs1.frame.world_to_local_point(origin_mm);
                        if n_wcs.z > 0.99
                            && p_wcs.z > rs1.model_aabb_wcs.min.z + 1.0
                            && p_wcs.z < rs1.model_aabb_wcs.max.z - 1.0
                        {
                            Some((f, f.area_mm2))
                        } else {
                            None
                        }
                    }
                    _ => None,
                })
                .collect();
            pocket_faces.sort_by(|a, b| b.1.total_cmp(&a.1));
            if let Some((pf, _)) = pocket_faces.first()
                && let Some(bind) = doc.bind_face(pf.id)
            {
                let mut pkt =
                    Operation::new("op_2_top_pocket", "Center Pocket", OperationKind::Pocket2D);
                pkt.tool_id = Some(em.id.clone());
                pkt.preset_id = em.presets.first().map(|p| p.id.clone());
                pkt.selections
                    .push(GeometrySelectionItem::PlanarFace { binding: bind });
                setup1.operations.push(pkt);
            }
        }

        if let Some(dr) = drill_6 {
            proj.ensure_tool_embedded(dr);
            let mut dr_op = Operation::new(
                "op_3_drill_holes",
                "Drill Mounting Holes",
                OperationKind::DrillBore,
            );
            dr_op.tool_id = Some(dr.id.clone());
            dr_op.preset_id = dr.presets.first().map(|p| p.id.clone());
            for f in &doc.faces {
                if let SurfaceKind::Cylinder {
                    axis_dir,
                    radius_mm,
                    is_internal: true,
                    ..
                } = f.surface
                {
                    let a_wcs = rs1.frame.world_to_local_dir(axis_dir).normalize_or_zero();
                    if a_wcs.z.abs() > 0.99
                        && (radius_mm * 2.0 - 6.0).abs() < 0.8
                        && let Some(bind) = doc.bind_face(f.id)
                    {
                        dr_op
                            .selections
                            .push(GeometrySelectionItem::CylindricalHole { binding: bind });
                    }
                }
            }
            if !dr_op.selections.is_empty() {
                setup1.operations.push(dr_op);
            }
        }
    }

    proj.setups = vec![setup1, setup2];
    proj
}
