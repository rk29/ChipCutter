//! Gap-closing built-in sample projects (Milestone 68):
//! 1. [`build_feed_opt_pocket_project`] (`--sample feed-opt-pocket`):
//!    Phase 16 engagement-aware `FeedOptimizationConfig` (sharp-corner lookahead deceleration,
//!    internal-arc centerline scaling, radial chip-thinning boost).
//! 2. [`build_flip_jig_softjaws_project`] (`--sample flip-jig-softjaws`):
//!    Phase 17 `Spoilboard` + **Flip-Jig Dowel-Pin Wizard** (`TwoOnAxis` + poka-yoke + `CylinderKeepOut`)
//!    + `Setup 2` `180°` flip carryover + `Setup 3` generated **Auto Soft-Jaw Machining Setup**.
//! 3. [`build_curved_deburr_3d_project`] (`--sample curved-deburr-3d`):
//!    Phase 18 non-planar sloped-boss `Deburr3D` across `ChamferMill` (automatic `FlatBvh`
//!    inside-corner wall trimming), `BallEndmill`, and `LollipopEndmill` underside back-chamfering.
//! 4. [`build_dovetail_rail_keyway_project`] (`--sample dovetail-rail-keyway`):
//!    Phase 18 `DovetailMode::ExternalRail` + blind `SlotUndercutMode::Keyway` with
//!    `UndercutEntryMode::EntryBoreCenter` in `Tri-Dexel` mode.
//! 5. [`build_svg_badge_sign_project`] (`--sample svg-badge`):
//!    Phase 2 Pure-2D `.svg` import (`96 DPI`, cubic Bézier biarc `G2`/`G3` fitting, counter-islands)
//!    with Pocket + V-Carve accent + outer Contour + Tabs.
//! 6. [`build_mesh_3mf_bracket_project`] (`--sample mesh-3mf-bracket`):
//!    Phase 5 unit-aware `.3mf` mesh import with 3D Adaptive Roughing + 3D Constant-Scallop Finish.

use chip_cad::{CadDocument, CurveKind, OrientedEdgeUse, SurfaceKind};
use chip_cam::{
    BoxAnchor27, Deburr3DParams, DovetailMode, DovetailParams, FeedOptimizationConfig,
    FixedBoxXyAlign, FixedBoxZAlign, FixtureBody, FixtureKind, FixtureZRef, GeometrySelectionItem,
    HeightReference, HeightSpec, HoleMachiningMode, MachiningBoundaryMode, Operation,
    OperationKind, OperationRotaryMode, PocketStrategyKind, RestMachiningConfig, RestMachiningMode,
    RotaryAxisKind, RotaryDirection, RotaryDriveMode, Setup, SharedAvoidRegion, SlotUndercutMode,
    SlotUndercutParams, SlotUndercutPreset, StockConfig, StrategyParams, TabProfileKind,
    ToolContainmentMode, UndercutEntryMode, ViseClampAxis, WcsOriginMode, WorkOffsetSlot,
};
use chip_fixtures::sample_svg_badge_bytes;
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DisplayUnit, ParamLength, approx_eq};
use chip_sketch::{
    RectAnchorMode, SketchAnchor, SketchEntity, SketchEntityEntry, SketchLayer, SketchPlaneRef,
    SvgImportOptions, import_svg_bytes,
};
use chip_tools::{CutterGeometry, Tool, starter_undercut_tools};

use chip_project::{
    document::{ModelEntry, ProjectDocument},
    wizards::{
        FlipAxis, FlipJigConfig, FlipJigPinCount, SoftJawBoltPattern, SoftJawConfig,
        SoftJawOutputMode, build_soft_jaw_plan, build_two_sided_flip_jig,
    },
};

fn find_flat_endmill(tools: &[Tool], target_dia_mm: f64, tol_mm: f64) -> Option<Tool> {
    tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::FlatEndmill { diameter_mm, .. }
                if (diameter_mm - target_dia_mm).abs() <= tol_mm
            )
        })
        .cloned()
}

/// Builds the **Engagement-Aware Feed-Optimized Pocket** sample (`--sample feed-opt-pocket`)
/// on `tutorial_adaptive_pocket_step()` (`100 × 80 × 20 mm`, sharp-corner pocket `[14..86] × [14..66]`
/// at `Z = 10 mm` with raised `Ø16 mm` central island boss).
pub fn build_feed_opt_pocket_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Feed-Optimized Adaptive Pocket");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        role: Default::default(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    }];

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

    let flat_6mm = find_flat_endmill(&proj.tools, 6.0, 0.2).or_else(|| proj.tools.first().cloned());

    let mut setup = Setup::new(
        "setup_1",
        "Setup 1 — Feed-Optimized Pocket (G54)",
        WorkOffsetSlot::G54,
    );
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_FRONT_LEFT,
    };
    setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(0.0),
        side_neg_x: ParamLength::from_mm(0.0),
        side_pos_y: ParamLength::from_mm(0.0),
        side_neg_y: ParamLength::from_mm(0.0),
        top_offset_z: ParamLength::from_mm(0.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(20.0),
        round_increment_mm: None,
    };

    let feed_opt = FeedOptimizationConfig {
        enabled: true,
        corner_slowdown: true,
        min_feed_fraction: 0.35,
        corner_angle_threshold_deg: 25.0,
        slowdown_distance: ParamLength::from_mm(3.0),
        internal_arc_scaling: true,
        chip_thinning_boost: true,
        max_thinning_multiplier: 1.5,
    };

    let pkt_floor = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && approx_eq(origin_mm.z, 10.0, 0.05)
        )
    });

    if let (Some(em), Some(floor)) = (&flat_6mm, pkt_floor)
        && let Some(bind) = doc.bind_face(floor.id)
    {
        // Op 1: 2D Adaptive Clearing with radial chip-thinning boost + internal-arc scaling + corner ramps
        let mut op1 = Operation::new_adaptive_2d(
            "op_1_adaptive_feed_opt",
            "1. 2D Adaptive Rough (Feed-Optimized)",
        );
        op1.tool_id = Some(em.id.clone());
        op1.preset_id = em.presets.first().map(|p| p.id.clone());
        op1.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op1.heights.bottom = HeightSpec::new(HeightReference::FromSelection, 0.0);
        op1.passes.multiple_depths = true;
        op1.passes.max_stepdown = ParamLength::from_mm(5.0);
        op1.passes.stock_to_leave_radial = ParamLength::from_mm(0.25);
        op1.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        op1.passes.feed_optimization = feed_opt.clone();
        op1.selections.push(GeometrySelectionItem::PlanarFace {
            binding: bind.clone(),
        });
        if let StrategyParams::Pocket2D(p) = op1.strategy_mut()
            && let PocketStrategyKind::Adaptive(ref mut adp) = p.strategy
        {
            adp.optimal_load = ParamLength::from_mm(1.5);
        }
        setup.operations.push(op1);

        // Op 2: 2D Pocket Finish (sharp 90° corners + Ø16 mm boss contour with corner slowdown & arc scaling)
        let mut op2 = Operation::new(
            "op_2_pocket_finish_feed_opt",
            "2. Sharp-Corner & Boss Finish (Feed-Optimized)",
            OperationKind::Pocket2D,
        );
        op2.tool_id = Some(em.id.clone());
        op2.preset_id = em.presets.first().map(|p| p.id.clone());
        op2.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op2.heights.bottom = HeightSpec::new(HeightReference::FromSelection, 0.0);
        op2.passes.multiple_depths = true;
        op2.passes.max_stepdown = ParamLength::from_mm(5.0);
        op2.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op2.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        op2.passes.feed_optimization = feed_opt;
        op2.passes.rest_machining = RestMachiningConfig {
            mode: RestMachiningMode::FromPrecedingOperations {
                source_op_ids: vec!["op_1_adaptive_feed_opt".to_string()],
            },
            ..RestMachiningConfig::default()
        };
        op2.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        setup.operations.push(op2);
    }

    proj.setups = vec![setup];
    proj
}

/// Builds the **Flip-Jig Dowel Pins & Auto Soft-Jaw Setup** sample (`--sample flip-jig-softjaws`)
/// on `tutorial_contour_plate_step()` (`02-contour-plate`: `100 × 70 × 12 mm` with `R10` fillet & `45°` chamfer):
/// - **Setup 1 (`G54`):** Mounted on a `Spoilboard` with **Flip-Jig Wizard** `TwoOnAxis` registration
///   pins (`+5.0 mm` poka-yoke offset), Op 1.1 dowel-pin boring, Op 1.2 top facing, and Op 1.3 top contour.
/// - **Setup 2 (`G55`):** `180°` flip around WCS X with `carryover_from_setup_id = Some("setup_1")`
///   and `CylinderKeepOut` registration pin fixtures + `Spoilboard`, facing bottom hat stock & finishing skirt.
/// - **Setup 3 (`G56`):** Auto-generated **Soft-Jaw Machining Setup** from `build_soft_jaw_plan`.
pub fn build_flip_jig_softjaws_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Flip-Jig Pins & Auto Soft-Jaw Setup");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        role: Default::default(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    }];

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

    let flat_6mm = find_flat_endmill(&proj.tools, 6.0, 0.2).or_else(|| proj.tools.first().cloned());
    let flat_3mm = find_flat_endmill(&proj.tools, 3.175, 0.4).or_else(|| flat_6mm.clone());

    // Setup 1 — Top Op on Spoilboard with Flip-Jig Registration Pins
    let mut setup_1 = Setup::new(
        "setup_1",
        "Setup 1 — Top Op & Flip-Jig Pins (G54)",
        WorkOffsetSlot::G54,
    );
    setup_1.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup_1.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(14.0),
        side_neg_x: ParamLength::from_mm(14.0),
        side_pos_y: ParamLength::from_mm(14.0),
        side_neg_y: ParamLength::from_mm(14.0),
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(3.0),
        sheet_thickness_z: ParamLength::from_mm(12.0),
        round_increment_mm: None,
    };
    setup_1.auto_avoid_fixtures_on_rapids = true;
    setup_1.fixtures.push(FixtureBody::default_spoilboard(
        "spoilboard_s1",
        "MDF Spoilboard (Setup 1)",
    ));

    // Generate TwoOnAxis Flip-Jig Wizard plan with poka-yoke asymmetry (`5.0 mm`) & carryover!
    let pin_tool = flat_3mm.as_ref();
    let flip_cfg = FlipJigConfig {
        flip_axis: FlipAxis::XAxis,
        pin_diameter_mm: 6.0,
        pin_fit_clearance_mm: 0.02,
        pin_count: FlipJigPinCount::TwoOnAxis,
        poka_yoke_offset_mm: 5.0,
        stock_pin_depth_mm: 0.0,
        spoilboard_pin_depth_mm: 10.0,
        edge_margin_mm: 6.0,
        expand_stock_if_needed: true,
        include_spoilboard_setup: false,
        pin_protrusion_z_mm: 4.5,
        pin_tool_id: pin_tool.map(|t| t.id.clone()),
        pin_tool_diameter_mm: pin_tool.map(|t| t.geometry.cutting_diameter_mm()),
        setup2_id: Some("setup_2".to_string()),
        setup2_wcs_slot: WorkOffsetSlot::G55,
        spoilboard_setup_id: None,
        spoilboard_wcs_slot: WorkOffsetSlot::G54,
    };

    let flip_plan = build_two_sided_flip_jig(
        doc,
        &setup_1,
        &proj.sketches,
        DisplayUnit::Millimeter,
        &flip_cfg,
    );

    let outer_loop_binding = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z < -0.99 && origin_mm.z.abs() < 0.05
            )
        })
        .and_then(|f| f.outer_loop)
        .and_then(|lid| doc.bind_loop(lid));

    let mut setup_2_opt = None;
    if let Ok(plan) = flip_plan {
        if let Some(updated_stock) = plan.updated_setup1_stock {
            setup_1.stock = updated_stock;
        }
        proj.sketches.push(plan.pin_sketch);
        let mut pin_op = plan.setup1_pin_op;
        if let Some(pt) = pin_tool {
            pin_op.preset_id = pt.presets.first().map(|p| p.id.clone());
        }
        setup_1.operations.push(pin_op);
        setup_2_opt = Some(plan.setup2_flip);
    }

    if let Some(em) = &flat_6mm {
        // Setup 1 Op 1.2: Face 1.0 mm top stock allowance down to ModelTop
        let mut op1_face =
            Operation::new("op1_face_top", "2. Face Stock Top", OperationKind::Facing);
        op1_face.tool_id = Some(em.id.clone());
        op1_face.preset_id = em.presets.first().map(|p| p.id.clone());
        op1_face.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op1_face.heights.bottom = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op1_face.passes.multiple_depths = false;
        op1_face.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        if let StrategyParams::Facing(p) = op1_face.strategy_mut() {
            p.pass_extension = ParamLength::from_mm(1.5);
        }
        setup_1.operations.push(op1_face);

        // Setup 1 Op 1.3: Top Outer Contour down to ModelBottom + 2.0 mm (clear of bottom hat & pins)
        if let Some(bind) = &outer_loop_binding {
            let mut op1_contour = Operation::new(
                "op1_top_contour",
                "3. Top Outer Contour",
                OperationKind::Contour2D,
            );
            op1_contour.tool_id = Some(em.id.clone());
            op1_contour.preset_id = em.presets.first().map(|p| p.id.clone());
            op1_contour.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
            op1_contour.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 2.0);
            op1_contour.passes.multiple_depths = true;
            op1_contour.passes.max_stepdown = ParamLength::from_mm(4.0);
            op1_contour.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
            op1_contour.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
            op1_contour
                .selections
                .push(GeometrySelectionItem::ClosedLoop {
                    binding: bind.clone(),
                    reverse: false,
                    side_flip: false,
                });
            setup_1.operations.push(op1_contour);
        }
    }

    // Setup 2 — 180° Flipped Bottom Op (G55) with Carryover + CylinderKeepOut Pins + Spoilboard
    let mut setups = vec![setup_1.clone()];
    if let Some(mut setup_2) = setup_2_opt {
        setup_2.name = "Setup 2 — 180° Flip Bottom Op (G55)".to_string();
        setup_2.fixtures.push(FixtureBody::default_spoilboard(
            "spoilboard_s2",
            "MDF Spoilboard (Flipped)",
        ));
        if let Some(em) = &flat_6mm {
            // Op 2.1: Face off the 3.0 mm bottom hat stock cleanly above the protruding dowel pins
            let mut op2_face = Operation::new(
                "op2_face_hat",
                "1. Face Bottom Hat Stock",
                OperationKind::Facing,
            );
            op2_face.tool_id = Some(em.id.clone());
            op2_face.preset_id = em.presets.first().map(|p| p.id.clone());
            op2_face.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
            op2_face.heights.bottom = HeightSpec::new(HeightReference::ModelTop, 0.0);
            op2_face.passes.multiple_depths = true;
            op2_face.passes.max_stepdown = ParamLength::from_mm(1.5);
            op2_face.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
            if let StrategyParams::Facing(p) = op2_face.strategy_mut() {
                p.pass_extension = ParamLength::from_mm(1.5);
            }
            setup_2.operations.push(op2_face);

            // Op 2.2: Finish the remaining 2.0 mm bottom skirt contour (strictly above protruding pins)
            if let Some(bind) = &outer_loop_binding {
                let mut op2_skirt = Operation::new(
                    "op2_skirt_contour",
                    "2. Finish Bottom Skirt Contour",
                    OperationKind::Contour2D,
                );
                op2_skirt.tool_id = Some(em.id.clone());
                op2_skirt.preset_id = em.presets.first().map(|p| p.id.clone());
                op2_skirt.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
                op2_skirt.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -2.5);
                op2_skirt.passes.multiple_depths = false;
                op2_skirt.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
                op2_skirt.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
                op2_skirt
                    .selections
                    .push(GeometrySelectionItem::ClosedLoop {
                        binding: bind.clone(),
                        reverse: false,
                        side_flip: false,
                    });
                setup_2.operations.push(op2_skirt);
            }
        }
        setups.push(setup_2);
    }

    // Setup 3 — Generated Auto Soft-Jaw Machining Setup (G56)
    let sj_cfg = SoftJawConfig {
        clamp_axis: ViseClampAxis::Y,
        jaw_width_mm: 152.4,
        jaw_depth_mm: 45.0,
        jaw_height_mm: 45.0,
        jaw_gap_mm: 6.0,
        grip_depth_mm: 4.0,
        part_clearance_mm: 0.12,
        corner_relief_radius_mm: 2.0,
        mounting_bolt_pattern: SoftJawBoltPattern::Kurt6Inch,
        output_mode: SoftJawOutputMode::MachiningSetup,
        z_band_bottom_offset_mm: 0.0,
        roughing_tool_id: flat_3mm.as_ref().map(|t| t.id.clone()),
        finishing_tool_id: flat_3mm.as_ref().map(|t| t.id.clone()),
        setup_id: Some("setup_3_soft_jaws".to_string()),
        wcs_slot: WorkOffsetSlot::G56,
    };
    if let Ok(sj_plan) = build_soft_jaw_plan(
        doc,
        &setup_1,
        &proj.sketches,
        DisplayUnit::Millimeter,
        &sj_cfg,
    ) {
        if let Some(sj_sketch) = sj_plan.pocket_sketch {
            proj.sketches.push(sj_sketch);
        }
        if let Some(mut sj_setup) = sj_plan.machining_setup {
            sj_setup.name = "Setup 3 — Auto Soft-Jaw Machining (G56)".to_string();
            if let Some(ft) = &flat_3mm {
                let preset = ft.presets.first().map(|p| p.id.clone());
                for op in &mut sj_setup.operations {
                    if op.preset_id.is_none() {
                        op.preset_id = preset.clone();
                    }
                }
            }
            setups.push(sj_setup);
        }
    }

    proj.setups = setups;
    proj
}

/// Builds the **3D Curved & Underside Deburring** sample (`--sample curved-deburr-3d`)
/// on `sample_curved_deburr_overhang_step_bytes()` (`80 × 50 × 23 mm`), exercising `Deburr3D` across:
/// - `ChamferMill` (curved non-planar sloped arch + automatic `FlatBvh` inside-corner wall trimming at `X = 35`),
/// - `BallEndmill` (curved 3D circular-arc arch rims), and
/// - `LollipopEndmill` (underside back-chamfering on the `+X` overhang lip at `Z = 9.5 mm`)
///
/// Runs in `Tri-Dexel` mode with `0` critical errors.
pub fn build_curved_deburr_3d_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("3D Curved & Underside Deburring");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        role: Default::default(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    }];

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }
    for t in starter_undercut_tools() {
        proj.ensure_tool_embedded(&t);
    }

    let flat_6mm = find_flat_endmill(&proj.tools, 6.0, 0.2);
    let ball_6mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::BallEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5))
        .cloned();
    let chamfer_90 = proj
        .tools
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
            proj.tools
                .iter()
                .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
        })
        .cloned();
    let slot_12mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::SlotMill { .. }))
        .cloned();
    let lollipop_6mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::LollipopEndmill { .. }))
        .cloned();

    // Setup 1 — G54 Top Center (`X ∈ [-40, 40]`, `Y ∈ [-25, 25]`, `Z ∈ [-23, 0]`)
    let mut setup = Setup::new(
        "setup_1",
        "Setup 1 — Curved Boss & Overhang Deburr (G54)",
        WorkOffsetSlot::G54,
    );
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(0.0),
        side_neg_x: ParamLength::from_mm(0.0),
        side_pos_y: ParamLength::from_mm(0.0),
        side_neg_y: ParamLength::from_mm(0.0),
        top_offset_z: ParamLength::from_mm(0.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(23.0),
        round_increment_mm: None,
    };

    // Companion sketch line along `+X` overhang relief groove (`X_wcs = +43.5`, `Y_wcs ∈ [-25, +25]`)
    // so the Ø12×4 mm SlotMill (`R = 6.0`, `r_neck = 2.5`) clears `X_model = 77.5..80.0`, `Z = 3.0..9.5`
    // cleanly before Lollipop underside back-chamfering!
    let mut sk_groove = SketchLayer::new(
        "sketch_overhang_groove",
        "Side Overhang Groove Axis",
        "setup_1",
    );
    sk_groove.anchor = SketchAnchor::SetupXy {
        setup_id: "setup_1".to_string(),
        plane_ref: SketchPlaneRef::StockTop,
        z_offset: ParamLength::from_mm(0.0),
    };
    sk_groove.entities.push(SketchEntityEntry {
        id: "ent_groove_axis".to_string(),
        name: "+X Overhang Groove Centerline".to_string(),
        sub_layer: Some("GROOVE".to_string()),
        is_construction: false,
        entity: SketchEntity::new_polyline(ArcPolyline2D {
            segments: vec![ArcSegment2D::Line {
                start: DVec2::new(43.5, -25.0),
                end: DVec2::new(43.5, 25.0),
            }],
            closed: false,
        }),
    });
    sk_groove.rebuild_regions(None);
    proj.sketches.push(sk_groove);

    // Op 1: 3D Adaptive Roughing of the stepped shelf (Z = 14.0) and curved circular-arc arch
    if let Some(em) = &flat_6mm {
        let mut op1 = Operation::new(
            "op_1_adaptive_3d",
            "1. 3D Adaptive Roughing",
            OperationKind::Adaptive3D,
        );
        op1.tool_id = Some(em.id.clone());
        op1.preset_id = em.presets.first().map(|p| p.id.clone());
        op1.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op1.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 14.0);
        op1.passes.stock_to_leave_radial = ParamLength::from_mm(0.25);
        op1.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        if let StrategyParams::Adaptive3D(p) = op1.strategy_mut() {
            p.boundary.boundary_mode = MachiningBoundaryMode::StockBox;
            p.max_roughing_stepdown = ParamLength::from_mm(4.5);
            p.fine_stepdown = ParamLength::from_mm(1.5);
            p.detect_flats = true;
            p.adaptive.optimal_load = ParamLength::from_mm(1.8);
        }
        setup.operations.push(op1);
    }

    // Op 2: 3D Parallel Raster Finishing across the curved arch & stepped shelf
    if let Some(ball) = &ball_6mm {
        let mut op2 = Operation::new(
            "op_2_parallel_3d",
            "2. 3D Parallel Arch & Shelf Finish",
            OperationKind::Parallel3D,
        );
        op2.tool_id = Some(ball.id.clone());
        op2.preset_id = ball.presets.first().map(|p| p.id.clone());
        op2.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op2.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 14.0);
        op2.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op2.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        if let StrategyParams::Parallel3D(p) = op2.strategy_mut() {
            p.boundary.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
            p.boundary.tool_containment = ToolContainmentMode::ToolCenterOnBoundary;
            p.use_cusp_height = true;
            p.cusp_height = ParamLength::from_mm(0.03);
            p.pass_angle_deg = 0.0;
        }
        setup.operations.push(op2);
    }

    // Op 3: Side Overhang Relief Groove (`SlotUndercutMode::SideGroove`, `Z_model = 3.0..9.5`)
    if let Some(sm) = &slot_12mm {
        let mut op3 = Operation::new(
            "op_3_side_overhang_groove",
            "3. Side Overhang Relief Groove",
            OperationKind::SlotUndercut,
        );
        op3.tool_id = Some(sm.id.clone());
        op3.preset_id = sm.presets.first().map(|p| p.id.clone());
        op3.heights.top = HeightSpec::new(HeightReference::ModelBottom, 9.5);
        op3.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 3.0);
        op3.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op3.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        op3.selections.push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sketch_overhang_groove".to_string(),
            entity_id: "ent_groove_axis".to_string(),
            reverse: false,
            side_flip: false,
        });
        op3.strategy = Some(StrategyParams::SlotUndercut(SlotUndercutParams {
            mode: SlotUndercutMode::SideGroove,
            preset: SlotUndercutPreset::Custom,
            slot_width: ParamLength::from_mm(12.0),
            throat_width: ParamLength::from_mm(12.0),
            slot_height: ParamLength::from_mm(6.5),
            radial_stepover: ParamLength::from_mm(1.0),
            axial_stepdown: ParamLength::from_mm(3.5),
            finish_allowance: ParamLength::from_mm(0.0),
            spring_pass: false,
            entry_mode: UndercutEntryMode::BothEndsThrough,
            entry_extension: ParamLength::from_mm(8.0),
            climb_direction: true,
        }));
        setup.operations.push(op3);
    }

    // Op 4: 3D Deburr — ChamferMill tracing the curved sloped arch & stepped shelf with automatic
    // FlatBvh inside-corner wall collision trimming at the vertical X = 35 step wall!
    if let Some(ch) = &chamfer_90 {
        let mut op4 = Operation::new(
            "op_4_deburr_chamfer_mill",
            "4. 3D Deburr — Chamfer Mill (Wall-Trimmed)",
            OperationKind::Deburr3D,
        );
        op4.tool_id = Some(ch.id.clone());
        op4.preset_id = ch.presets.first().map(|p| p.id.clone());
        op4.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op4.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 13.0);
        op4.strategy = Some(StrategyParams::Deburr3D(Deburr3DParams {
            chamfer_width: ParamLength::from_mm(0.45),
            tip_offset: ParamLength::from_mm(0.65),
            wall_clearance: ParamLength::from_mm(0.15),
            edge_extension: ParamLength::from_mm(0.0),
            auto_detect_edges: true,
            min_dihedral_angle_deg: 15.0,
            exclude_vertical_edges: true,
            allow_underside_back_chamfer: false,
            climb_direction: true,
            chord_tolerance_mm: 0.005,
            multiple_passes: 1,
            spring_pass: false,
            ..Deburr3DParams::default()
        }));
        setup.operations.push(op4);
    }

    // Locate the 2 non-planar curved 3D CircleArc arch edges along Y = 0 and Y = 50
    let arch_edges: Vec<_> = doc
        .edges
        .iter()
        .filter(|e| matches!(e.curve, CurveKind::CircleArc { .. }))
        .collect();

    // Op 5: 3D Deburr — BallEndmill curved rim break along the two non-planar 3D circular arcs
    if let Some(ball) = &ball_6mm
        && !arch_edges.is_empty()
    {
        let mut op5 = Operation::new(
            "op_5_deburr_ball_arch",
            "5. 3D Deburr — Ball Endmill (Curved Arch)",
            OperationKind::Deburr3D,
        );
        op5.tool_id = Some(ball.id.clone());
        op5.preset_id = ball.presets.first().map(|p| p.id.clone());
        op5.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op5.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 13.0);
        let body_id = doc.bodies[0].id;
        for edge in &arch_edges {
            if let Some(b) = doc.bind_edge(edge.id) {
                op5.selections.push(GeometrySelectionItem::OpenChain {
                    body_id,
                    start_edge: b.clone(),
                    end_edge: b,
                    resolved_edges: vec![OrientedEdgeUse {
                        edge_id: edge.id,
                        forward: true,
                    }],
                    reverse: false,
                    side_flip: false,
                });
            }
        }
        op5.strategy = Some(StrategyParams::Deburr3D(Deburr3DParams {
            chamfer_width: ParamLength::from_mm(0.45),
            tip_offset: ParamLength::from_mm(0.5),
            wall_clearance: ParamLength::from_mm(0.15),
            edge_extension: ParamLength::from_mm(0.0),
            auto_detect_edges: false,
            min_dihedral_angle_deg: 15.0,
            exclude_vertical_edges: true,
            allow_underside_back_chamfer: false,
            climb_direction: true,
            chord_tolerance_mm: 0.005,
            multiple_passes: 1,
            spring_pass: false,
            ..Deburr3DParams::default()
        }));
        setup.operations.push(op5);
    }

    // Locate the sharp +X underside overhang lip edge at X = 80.0, Z = 9.5 (Y ∈ [0..50])
    let underside_edge = doc.edges.iter().find(|e| {
        approx_eq(e.aabb_mm.min.x, 80.0, 0.05)
            && approx_eq(e.aabb_mm.max.x, 80.0, 0.05)
            && approx_eq(e.aabb_mm.min.z, 9.5, 0.05)
            && approx_eq(e.aabb_mm.max.z, 9.5, 0.05)
            && e.aabb_mm.size().y > 40.0
    });

    // Op 6: 3D Deburr — LollipopEndmill underside back-chamfering on the +X overhang rim
    if let (Some(lolli), Some(edge)) = (&lollipop_6mm, underside_edge)
        && let Some(b) = doc.bind_edge(edge.id)
    {
        let mut op6 = Operation::new(
            "op_6_deburr_lollipop_underside",
            "6. 3D Deburr — Lollipop Underside Back-Chamfer",
            OperationKind::Deburr3D,
        );
        op6.tool_id = Some(lolli.id.clone());
        op6.preset_id = lolli.presets.first().map(|p| p.id.clone());
        op6.heights.top = HeightSpec::new(HeightReference::ModelBottom, 11.0);
        op6.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 2.5);
        op6.selections.push(GeometrySelectionItem::OpenChain {
            body_id: doc.bodies[0].id,
            start_edge: b.clone(),
            end_edge: b,
            resolved_edges: vec![OrientedEdgeUse {
                edge_id: edge.id,
                forward: true,
            }],
            reverse: false,
            side_flip: false,
        });
        op6.strategy = Some(StrategyParams::Deburr3D(Deburr3DParams {
            chamfer_width: ParamLength::from_mm(0.4),
            tip_offset: ParamLength::from_mm(0.5),
            wall_clearance: ParamLength::from_mm(0.15),
            edge_extension: ParamLength::from_mm(0.0),
            auto_detect_edges: false,
            min_dihedral_angle_deg: 15.0,
            exclude_vertical_edges: true,
            allow_underside_back_chamfer: true,
            climb_direction: true,
            chord_tolerance_mm: 0.005,
            multiple_passes: 1,
            spring_pass: false,
            ..Deburr3DParams::default()
        }));
        setup.operations.push(op6);
    }

    proj.setups = vec![setup];
    proj
}

/// Builds the **External Dovetail Rail & Blind Entry-Bore Keyway** sample (`--sample dovetail-rail-keyway`)
/// on `sample_dovetail_rail_keyway_step_bytes()` (`90 × 50 × 22 mm`), sequencing:
/// 1. `Face Stock Top` (`Facing`, Ø6 mm Flat Endmill),
/// 2. `Pre-Drill Keyway Entry Bore` (`DrillBore` / `HelicalBore`, Ø6 mm Flat Endmill on Ø8.0 mm bore),
/// 3. `Blind Top Keyway (Entry Bore)` (`SlotUndercut` `Keyway` + `EntryBoreCenter`, Ø6 mm Flat Endmill),
/// 4. `External 60° Dovetail Rail` (`Dovetail` `ExternalRail`, Ø12 mm 60° DovetailCutter).
///
/// Runs in `Tri-Dexel` mode with `0` critical errors.
pub fn build_dovetail_rail_keyway_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("External 60° Dovetail Rail & Blind Keyway");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        role: Default::default(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    }];

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }
    for t in starter_undercut_tools() {
        proj.ensure_tool_embedded(&t);
    }

    let flat_6mm = find_flat_endmill(&proj.tools, 6.0, 0.2);
    let dovetail_60 = proj
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::DovetailCutter { side_angle_deg, .. }
                if (side_angle_deg - 60.0).abs() < 0.5
            )
        })
        .cloned();

    // Setup 1 — G54 Top Center (`90 × 50 × 23 mm` raw block with 1.0 mm facing allowance)
    let mut setup = Setup::new(
        "setup_1",
        "Setup 1 — Dovetail Rail & Blind Keyway (G54)",
        WorkOffsetSlot::G54,
    );
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(0.0),
        side_neg_x: ParamLength::from_mm(0.0),
        side_pos_y: ParamLength::from_mm(0.0),
        side_neg_y: ParamLength::from_mm(0.0),
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(22.0),
        round_increment_mm: None,
    };

    // Locate the Ø8.0 mm (R = 4.0 mm) cylindrical entry bore face & Z = 16.0 mm blind keyway floor
    let entry_bore_face = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Cylinder {
                radius_mm,
                is_internal: true,
                ..
            } if approx_eq(radius_mm, 4.0, 0.05)
        )
    });
    let keyway_floor = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && approx_eq(origin_mm.z, 16.0, 0.05)
        )
    });

    // Op 1: Face Stock Top (1.0 mm allowance down to Z = 22.0 mm)
    if let Some(em) = &flat_6mm {
        let mut op1 = Operation::new("op_1_face_top", "1. Face Stock Top", OperationKind::Facing);
        op1.tool_id = Some(em.id.clone());
        op1.preset_id = em.presets.first().map(|p| p.id.clone());
        op1.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op1.heights.bottom = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op1.passes.multiple_depths = false;
        op1.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        if let StrategyParams::Facing(p) = op1.strategy_mut() {
            p.pass_extension = ParamLength::from_mm(2.0);
        }
        setup.operations.push(op1);
    }

    // Op 2: Helical Entry Bore (`Ø8.0 mm` at `(24, 25)` down to `Z = 14.0 mm` via `DrillBore` / `HelicalBore`)
    if let (Some(em), Some(bore_face)) = (&flat_6mm, entry_bore_face)
        && let Some(bore_bind) = doc.bind_face(bore_face.id)
    {
        let mut op2 = Operation::new(
            "op_2_entry_bore",
            "2. Pre-Drill Keyway Entry Bore (Ø8 mm)",
            OperationKind::DrillBore,
        );
        op2.tool_id = Some(em.id.clone());
        op2.preset_id = em.presets.first().map(|p| p.id.clone());
        op2.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op2.heights.bottom = HeightSpec::new(HeightReference::FromSelection, 0.0);
        op2.passes.stock_to_leave_radial = ParamLength::from_mm(0.06);
        op2.selections
            .push(GeometrySelectionItem::CylindricalHole { binding: bore_bind });
        if let StrategyParams::DrillBore(p) = op2.strategy_mut() {
            p.mode = HoleMachiningMode::HelicalBore;
        }
        setup.operations.push(op2);
    }

    // Op 3: Blind Top Keyway (`SlotUndercutMode::Keyway` + `UndercutEntryMode::EntryBoreCenter`)
    if let (Some(em), Some(kw_face), Some(bore_face)) = (&flat_6mm, keyway_floor, entry_bore_face)
        && let (Some(kw_bind), Some(bore_bind)) =
            (doc.bind_face(kw_face.id), doc.bind_face(bore_face.id))
    {
        let mut op3 = Operation::new(
            "op_3_blind_keyway",
            "3. Blind Top Keyway (Entry Bore)",
            OperationKind::SlotUndercut,
        );
        op3.tool_id = Some(em.id.clone());
        op3.preset_id = em.presets.first().map(|p| p.id.clone());
        op3.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op3.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -6.0);
        op3.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op3.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        op3.selections
            .push(GeometrySelectionItem::PlanarFace { binding: kw_bind });
        op3.selections
            .push(GeometrySelectionItem::CylindricalHole { binding: bore_bind });
        op3.strategy = Some(StrategyParams::SlotUndercut(SlotUndercutParams {
            mode: SlotUndercutMode::Keyway,
            preset: SlotUndercutPreset::Custom,
            slot_width: ParamLength::from_mm(10.0),
            throat_width: ParamLength::from_mm(10.0),
            slot_height: ParamLength::from_mm(6.0),
            radial_stepover: ParamLength::from_mm(1.0),
            axial_stepdown: ParamLength::from_mm(3.0),
            finish_allowance: ParamLength::from_mm(0.1),
            spring_pass: true,
            entry_mode: UndercutEntryMode::EntryBoreCenter,
            entry_extension: ParamLength::from_mm(6.0),
            climb_direction: true,
        }));
        setup.operations.push(op3);
    }

    // Op 4: External 60° Dovetail Rail (`DovetailMode::ExternalRail`) along both `-Y` (Y=0) & `+Y` (Y=50) flanks!
    if let Some(dt) = &dovetail_60 {
        let mut rail_selections = Vec::new();
        let body_id = doc.bodies[0].id;

        // `-Y` top rail edge (Y = 0, Z = 22): travel in `-X` so LEFT (`+1`) points `-Y` (outward air)
        if let Some(edge_neg_y) = doc.edges.iter().find(|e| {
            approx_eq(e.aabb_mm.min.z, 22.0, 0.05)
                && approx_eq(e.aabb_mm.max.z, 22.0, 0.05)
                && approx_eq(e.aabb_mm.min.y, 0.0, 0.05)
                && approx_eq(e.aabb_mm.max.y, 0.0, 0.05)
                && e.aabb_mm.size().x > 80.0
        }) && let Some(b) = doc.bind_edge(edge_neg_y.id)
        {
            let forward = match edge_neg_y.curve {
                CurveKind::Line { start_mm, end_mm } => start_mm.x > end_mm.x,
                _ => true,
            };
            rail_selections.push(GeometrySelectionItem::OpenChain {
                body_id,
                start_edge: b.clone(),
                end_edge: b,
                resolved_edges: vec![OrientedEdgeUse {
                    edge_id: edge_neg_y.id,
                    forward,
                }],
                reverse: false,
                side_flip: false,
            });
        }

        // `+Y` top rail edge (Y = 50, Z = 22): travel in `+X` so LEFT (`+1`) points `+Y` (outward air)
        if let Some(edge_pos_y) = doc.edges.iter().find(|e| {
            approx_eq(e.aabb_mm.min.z, 22.0, 0.05)
                && approx_eq(e.aabb_mm.max.z, 22.0, 0.05)
                && approx_eq(e.aabb_mm.min.y, 50.0, 0.05)
                && approx_eq(e.aabb_mm.max.y, 50.0, 0.05)
                && e.aabb_mm.size().x > 80.0
        }) && let Some(b) = doc.bind_edge(edge_pos_y.id)
        {
            let forward = match edge_pos_y.curve {
                CurveKind::Line { start_mm, end_mm } => start_mm.x < end_mm.x,
                _ => true,
            };
            rail_selections.push(GeometrySelectionItem::OpenChain {
                body_id,
                start_edge: b.clone(),
                end_edge: b,
                resolved_edges: vec![OrientedEdgeUse {
                    edge_id: edge_pos_y.id,
                    forward,
                }],
                reverse: false,
                side_flip: false,
            });
        }

        if !rail_selections.is_empty() {
            let mut op4 = Operation::new(
                "op_4_external_dovetail_rail",
                "4. External 60° Dovetail Rail Flanks",
                OperationKind::Dovetail,
            );
            op4.tool_id = Some(dt.id.clone());
            op4.preset_id = dt.presets.first().map(|p| p.id.clone());
            op4.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
            op4.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -5.0);
            op4.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
            op4.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
            op4.selections = rail_selections;
            op4.strategy = Some(StrategyParams::Dovetail(DovetailParams {
                mode: DovetailMode::ExternalRail,
                top_throat_width: ParamLength::from_mm(10.0),
                dovetail_depth: ParamLength::from_mm(5.0),
                side_angle_deg: 60.0,
                radial_stepover: ParamLength::from_mm(0.6),
                finish_allowance: ParamLength::from_mm(0.1),
                spring_pass: true,
                entry_mode: UndercutEntryMode::BothEndsThrough,
                entry_extension: ParamLength::from_mm(8.0),
                climb_direction: true,
            }));
            setup.operations.push(op4);
        }
    }

    proj.setups = vec![setup];
    proj
}

/// Builds the **Pure-2D SVG Badge Sign** sample (`--sample svg-badge`) imported directly from
/// `sample_svg_badge_bytes()` (`96 DPI`, cubic Bézier biarc `G2`/`G3` fitting, counter-islands):
/// 1. `Emblem Ring Pocket + Island` (`Pocket2D`, Ø3.175 mm Flat Endmill),
/// 2. `Flanking V-Carve Accents` (`VCarve`, 60° V-Bit),
/// 3. `Outer Badge Contour + Tabs` (`Contour2D`, Ø6 mm Flat Endmill + Triangular Tabs).
pub fn build_svg_badge_sign_project(
    source_svg_path: Option<String>,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Pure-2D SVG Badge Sign (96 DPI Bézier)");
    proj.models.clear();

    let svg_bytes = sample_svg_badge_bytes();
    let opts = SvgImportOptions {
        sketch_id: "sketch_svg_badge".to_string(),
        sketch_name: "SVG Badge Emblem (96 DPI)".to_string(),
        setup_id: "setup_1".to_string(),
        source_path: source_svg_path,
        svg_dpi: 96.0,
        ..SvgImportOptions::default()
    };
    if let Ok(rep) = import_svg_bytes(&svg_bytes, &opts) {
        proj.sketches.push(rep.sketch);
    }

    proj.setups[0].name = "Setup 1 — 8mm Hardwood Badge Blank (G54)".to_string();
    proj.setups[0].wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_FRONT_LEFT,
    };
    proj.setups[0].stock = StockConfig::RelativeBox {
        symmetric_xy: true,
        side_pos_x: ParamLength::from_mm(6.0),
        side_neg_x: ParamLength::from_mm(6.0),
        side_pos_y: ParamLength::from_mm(6.0),
        side_neg_y: ParamLength::from_mm(6.0),
        top_offset_z: ParamLength::from_mm(0.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(8.0),
        round_increment_mm: None,
    };

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

    let flat_3175 = find_flat_endmill(&proj.tools, 3.175, 0.4);
    let flat_6mm = find_flat_endmill(&proj.tools, 6.0, 0.2);
    let vbit_60 = proj
        .tools
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
            proj.tools
                .iter()
                .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
        })
        .cloned();

    // Op 1: Emblem Ring Pocket + Counter-Island (cubic Bézier -> true G2/G3 arcs!)
    if let Some(em3) = &flat_3175 {
        let mut op1 = Operation::new(
            "op_1_svg_emblem_pocket",
            "1. Emblem Ring Pocket + Island",
            OperationKind::Pocket2D,
        );
        op1.tool_id = Some(em3.id.clone());
        op1.preset_id = em3.presets.first().map(|p| p.id.clone());
        op1.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op1.heights.bottom = HeightSpec::new(HeightReference::StockTop, -2.5);
        op1.passes.multiple_depths = false;
        op1.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op1.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        op1.selections.push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sketch_svg_badge".to_string(),
            sub_layer: Some("EMBLEM_POCKET".to_string()),
            reverse: false,
            side_flip: false,
        });
        proj.setups[0].operations.push(op1);
    }

    // Op 2: Decorative Flanking V-Carve Accents (`VCARVE_ACCENT`)
    if let Some(vb) = &vbit_60 {
        let mut op2 = Operation::new(
            "op_2_svg_vcarve_accents",
            "2. Flanking V-Carve Accents",
            OperationKind::VCarve,
        );
        op2.tool_id = Some(vb.id.clone());
        op2.preset_id = vb.presets.first().map(|p| p.id.clone());
        op2.selections.push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sketch_svg_badge".to_string(),
            sub_layer: Some("VCARVE_ACCENT".to_string()),
            reverse: false,
            side_flip: false,
        });
        proj.setups[0].operations.push(op2);
    }

    // Op 3: Outer Badge Contour + Triangular Holding Tabs (`OUTLINE`, cubic Bézier R10 corners -> G2/G3!)
    if let Some(em6) = &flat_6mm {
        let mut op3 = Operation::new(
            "op_3_svg_outer_contour",
            "3. Outer Badge Contour + Tabs",
            OperationKind::Contour2D,
        );
        op3.tool_id = Some(em6.id.clone());
        op3.preset_id = em6.presets.first().map(|p| p.id.clone());
        op3.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op3.heights.bottom = HeightSpec::new(HeightReference::StockBottom, 0.0);
        op3.passes.multiple_depths = true;
        op3.passes.max_stepdown = ParamLength::from_mm(4.0);
        op3.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op3.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        op3.selections.push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sketch_svg_badge".to_string(),
            sub_layer: Some("OUTLINE".to_string()),
            reverse: false,
            side_flip: false,
        });
        if let StrategyParams::Contour2D(p) = op3.strategy_mut() {
            p.tabs.enabled = true;
            p.tabs.profile = TabProfileKind::Triangular {
                ramp_angle_deg: 25.0,
            };
        }
        proj.setups[0].operations.push(op3);
    }

    proj
}

/// Builds the **Sculpted Mount Bracket (3MF Mesh)** sample (`--sample mesh-3mf-bracket`)
/// imported directly from `sample_sculpted_mount_3mf_bytes()` (`80 × 56 × 20 mm` unit-aware `.3mf`),
/// machined with:
/// 1. `3D Adaptive Roughing` (`Ø6 mm` Flat Endmill, `0.4 mm` radial / `0.25 mm` axial stock-to-leave),
/// 2. `3D Constant-Scallop Finish` (`Ø6 mm` Ball Endmill, `0.04 mm` uniform 3D cusp height).
pub fn build_mesh_3mf_bracket_project(
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Sculpted Mount Bracket (3MF Mesh)");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        role: Default::default(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    }];
    proj.setups[0].name = "Setup 1 — 3D Sculpted Mount (3MF)".to_string();
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
        sheet_thickness_z: ParamLength::from_mm(20.0),
        round_increment_mm: None,
    };

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

    let flat_6mm = find_flat_endmill(&proj.tools, 6.0, 0.5).or_else(|| proj.tools.first().cloned());
    let ball_6mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::BallEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5))
        .cloned();

    // Op 1: 3D Adaptive Roughing down to the Z = 8.0 mm base flange deck (`ModelTop - 12.0 mm`)
    if let Some(t) = &flat_6mm {
        let mut op_rough = Operation::new(
            "op_1_3mf_adaptive_3d",
            "1. 3D Adaptive Roughing",
            OperationKind::Adaptive3D,
        );
        op_rough.tool_id = Some(t.id.clone());
        op_rough.preset_id = t.presets.first().map(|p| p.id.clone());
        op_rough.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -12.0);
        op_rough.passes.stock_to_leave_radial = ParamLength::from_mm(0.4);
        op_rough.passes.stock_to_leave_axial = ParamLength::from_mm(0.25);
        if let StrategyParams::Adaptive3D(p) = op_rough.strategy_mut() {
            p.max_roughing_stepdown = ParamLength::from_mm(4.0);
            p.fine_stepdown = ParamLength::from_mm(2.0);
            p.adaptive.optimal_load = ParamLength::from_mm(1.5);
            p.detect_flats = true;
        }
        proj.setups[0].operations.push(op_rough);
    }

    // Op 2: 3D Constant-Scallop Finishing across the sculpted bracket mount
    if let Some(t) = ball_6mm.as_ref().or(flat_6mm.as_ref()) {
        let mut op_scallop = Operation::new(
            "op_2_3mf_scallop_3d",
            "2. 3D Constant-Scallop Finish",
            OperationKind::Scallop3D,
        );
        op_scallop.tool_id = Some(t.id.clone());
        op_scallop.preset_id = t.presets.first().map(|p| p.id.clone());
        op_scallop.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -12.0);
        if let StrategyParams::Scallop3D(p) = op_scallop.strategy_mut() {
            p.boundary.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
            p.boundary.tool_containment = ToolContainmentMode::ToolCenterOnBoundary;
            p.cusp_height = ParamLength::from_mm(0.04);
            p.max_stepover = ParamLength::from_mm(1.5);
            p.min_stepover = ParamLength::from_mm(0.25);
        }
        proj.setups[0].operations.push(op_scallop);
    }

    proj
}

/// Builds the Phase 23 **Clamp & Boss Keep-Out Plate (Avoid / Keep-Out Regions)** sample
/// (`--sample keepout-clamps`) on `sample_keepout_clamp_plate_step_bytes()` (`100 × 70 × 18 mm`
/// filleted flange plate with raised `30 × 26 × 8 mm` central boss):
/// - Sits on an MDF `Spoilboard` with two `ToeClamp` fixtures overhanging the `-X` and `+X` stock
///   and flange edges plus a 2D sketch (`Keep-Out Zones`) surrounding the central boss,
/// - Defines a reusable [`SharedAvoidRegion`] (`"avoid_clamps_boss"`) bundling both Toe Clamps
///   and the sketched boss keep-out rectangle (`radial_clearance = 1.0 mm`), shared across:
///   1. `1. Face Stock Top (Avoid Clamps & Boss)` (`Facing`),
///   2. `2. 3D Adaptive Roughing (Model Bounds + Offsets)` (`Adaptive3D` + `ModelBoundingBox`),
///   3. `3. 3D Parallel Finish (Shared Keep-Out)` (`Parallel3D`),
/// - Plus `4. Outer Flange Contour (Avoid All Fixtures)` (`Contour2D`, `avoid_all_fixtures = true`),
///   verified `0` `FixtureCollision`, `0` gouges, and `0` rapid crashes in `chip-sim`.
pub fn build_keepout_clamps_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Clamp & Boss Keep-Out Plate (Avoid / Keep-Out Regions)");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        role: Default::default(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    }];

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

    let flat_6mm = find_flat_endmill(&proj.tools, 6.0, 0.5).or_else(|| proj.tools.first().cloned());

    let mut setup = Setup::new(
        "setup_1",
        "Setup 1 — Clamped Plate & Boss Keep-Out (G54)",
        WorkOffsetSlot::G54,
    );
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::RelativeBox {
        symmetric_xy: false,
        side_pos_x: ParamLength::from_mm(12.0),
        side_neg_x: ParamLength::from_mm(12.0),
        side_pos_y: ParamLength::from_mm(8.0),
        side_neg_y: ParamLength::from_mm(8.0),
        top_offset_z: ParamLength::from_mm(2.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(18.0),
        round_increment_mm: None,
    };
    setup.heights.clearance = HeightSpec::new(HeightReference::StockTop, 24.0);
    setup.auto_avoid_fixtures_on_rapids = true;

    // 1. Sacrificial MDF Spoilboard sitting directly beneath StockBottom
    setup.fixtures.push(FixtureBody::default_spoilboard(
        "spoilboard",
        "MDF Spoilboard",
    ));

    // 2. Two Toe Clamps overhanging the -X and +X stock & outer flange edges (nose at X = ±47 mm)
    for (id, name, anchor_xy, yaw_deg) in [
        (
            "clamp_neg_x",
            "Left Toe Clamp (-X)",
            DVec2::new(-62.0, 0.0),
            180.0,
        ),
        (
            "clamp_pos_x",
            "Right Toe Clamp (+X)",
            DVec2::new(62.0, 0.0),
            0.0,
        ),
    ] {
        let mut clamp = FixtureBody::default_toe_clamp(id, name, anchor_xy, yaw_deg);
        clamp.clearance_margin = ParamLength::from_mm(1.0);
        if let FixtureKind::ToeClamp {
            z_reference,
            z_offset,
            strap_length,
            strap_width,
            strap_height,
            nose_overhang,
            stud_diameter,
            stud_height_above_strap,
            ..
        } = &mut clamp.kind
        {
            *z_reference = FixtureZRef::StockTop;
            *z_offset = ParamLength::from_mm(0.0);
            *strap_length = ParamLength::from_mm(42.0);
            *strap_width = ParamLength::from_mm(16.0);
            *strap_height = ParamLength::from_mm(8.0);
            *nose_overhang = ParamLength::from_mm(15.0);
            *stud_diameter = ParamLength::from_mm(8.0);
            *stud_height_above_strap = ParamLength::from_mm(10.0);
        }
        setup.fixtures.push(clamp);
    }

    // 3. 2D Sketch layer containing the protective Keep-Out rectangle around the central boss
    let mut sk_keepout = SketchLayer::new("sk_keepout_zones", "Keep-Out Zones", "setup_1");
    sk_keepout.anchor = SketchAnchor::SetupXy {
        setup_id: "setup_1".to_string(),
        plane_ref: SketchPlaneRef::StockTop,
        z_offset: ParamLength::from_mm(0.0),
    };
    sk_keepout.entities.push(SketchEntityEntry {
        id: "ent_boss_keepout".to_string(),
        name: "Central Boss Keep-Out".to_string(),
        sub_layer: Some("KEEPOUT".to_string()),
        is_construction: false,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(0.0),
            y: ParamLength::from_mm(0.0),
            width: ParamLength::from_mm(34.0),
            height: ParamLength::from_mm(30.0),
            corner_radius: ParamLength::from_mm(2.0),
            rotation_deg: 0.0,
        },
    });
    sk_keepout.rebuild_regions(None);
    proj.sketches.push(sk_keepout);

    // 4. Shared Project Avoid / Keep-Out Region bundling both Toe Clamps + sketched central boss
    let mut shared_avoid = SharedAvoidRegion::new("avoid_clamps_boss", "Clamps & Boss Keep-Out");
    shared_avoid.fixture_ids = vec!["clamp_neg_x".to_string(), "clamp_pos_x".to_string()];
    shared_avoid
        .selections
        .push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sk_keepout_zones".to_string(),
            entity_id: "ent_boss_keepout".to_string(),
            reverse: false,
            side_flip: false,
        });
    shared_avoid.radial_clearance = Some(ParamLength::from_mm(1.0));
    proj.avoid_regions.push(shared_avoid);

    let outer_loop_binding = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z < -0.99 && origin_mm.z.abs() < 0.05
            )
        })
        .and_then(|f| f.outer_loop)
        .and_then(|lid| doc.bind_loop(lid));

    if let Some(em) = &flat_6mm {
        // Op 1: Face Stock Top (`StockBox` + shared `"avoid_clamps_boss"`)
        let mut op1 = Operation::new(
            "op_1_face_avoid_clamps_boss",
            "Face Stock Top (Avoid Clamps & Boss)",
            OperationKind::Facing,
        );
        op1.tool_id = Some(em.id.clone());
        op1.preset_id = em.presets.first().map(|p| p.id.clone());
        op1.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op1.heights.bottom = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op1.passes.multiple_depths = false;
        op1.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        op1.avoid
            .shared_region_ids
            .push("avoid_clamps_boss".to_string());
        if let StrategyParams::Facing(p) = op1.strategy_mut() {
            p.boundary.boundary_mode = MachiningBoundaryMode::StockBox;
            p.pass_extension = ParamLength::from_mm(1.5);
        }
        setup.operations.push(op1);

        // Op 2: 3D Adaptive Roughing (`ModelBoundingBox` + signed per-side offsets + shared keep-out)
        let mut op2 = Operation::new(
            "op_2_adaptive_model_bounds",
            "3D Adaptive Roughing (Model Bounds + Offsets)",
            OperationKind::Adaptive3D,
        );
        op2.tool_id = Some(em.id.clone());
        op2.preset_id = em.presets.first().map(|p| p.id.clone());
        op2.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op2.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 10.0);
        op2.passes.stock_to_leave_radial = ParamLength::from_mm(0.25);
        op2.passes.stock_to_leave_axial = ParamLength::from_mm(0.2);
        op2.avoid
            .shared_region_ids
            .push("avoid_clamps_boss".to_string());
        if let StrategyParams::Adaptive3D(p) = op2.strategy_mut() {
            p.boundary.boundary_mode = MachiningBoundaryMode::ModelBoundingBox;
            p.boundary.model_bounds_offset_x_neg = ParamLength::from_mm(-1.0);
            p.boundary.model_bounds_offset_x_pos = ParamLength::from_mm(-1.0);
            p.boundary.model_bounds_offset_y_neg = ParamLength::from_mm(3.0);
            p.boundary.model_bounds_offset_y_pos = ParamLength::from_mm(3.0);
            p.max_roughing_stepdown = ParamLength::from_mm(4.0);
            p.fine_stepdown = ParamLength::from_mm(2.0);
            p.detect_flats = true;
            p.adaptive.optimal_load = ParamLength::from_mm(2.0);
        }
        setup.operations.push(op2);

        // Op 3: 3D Parallel Finish (linked to the exact same shared `"avoid_clamps_boss"`)
        let mut op3 = Operation::new(
            "op_3_parallel_shared_keepout",
            "3D Parallel Finish (Shared Keep-Out)",
            OperationKind::Parallel3D,
        );
        op3.tool_id = Some(em.id.clone());
        op3.preset_id = em.presets.first().map(|p| p.id.clone());
        op3.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op3.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 10.0);
        op3.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op3.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        op3.avoid
            .shared_region_ids
            .push("avoid_clamps_boss".to_string());
        if let StrategyParams::Parallel3D(p) = op3.strategy_mut() {
            p.boundary.boundary_mode = MachiningBoundaryMode::ModelBoundingBox;
            p.boundary.model_bounds_offset_x_neg = ParamLength::from_mm(-1.0);
            p.boundary.model_bounds_offset_x_pos = ParamLength::from_mm(-1.0);
            p.boundary.model_bounds_offset_y_neg = ParamLength::from_mm(2.0);
            p.boundary.model_bounds_offset_y_pos = ParamLength::from_mm(2.0);
            p.boundary.tool_containment = ToolContainmentMode::ToolCenterOnBoundary;
            p.use_cusp_height = false;
            p.stepover = ParamLength::from_mm(1.8);
            p.pass_angle_deg = 0.0;
        }
        setup.operations.push(op3);

        // Op 4: Outer Flange Contour (`avoid_all_fixtures = true` automatically clips around clamps!)
        if let Some(bind) = outer_loop_binding {
            let mut op4 = Operation::new(
                "op_4_contour_avoid_fixtures",
                "Outer Flange Contour (Avoid All Fixtures)",
                OperationKind::Contour2D,
            );
            op4.tool_id = Some(em.id.clone());
            op4.preset_id = em.presets.first().map(|p| p.id.clone());
            op4.heights.top = HeightSpec::new(HeightReference::ModelBottom, 10.0);
            op4.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.5);
            op4.passes.multiple_depths = true;
            op4.passes.max_stepdown = ParamLength::from_mm(3.5);
            op4.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
            op4.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
            op4.avoid.avoid_all_fixtures = true;
            op4.avoid.radial_clearance = ParamLength::from_mm(1.0);
            op4.selections.push(GeometrySelectionItem::ClosedLoop {
                binding: bind,
                reverse: false,
                side_flip: false,
            });
            setup.operations.push(op4);
        }
    }

    proj.setups = vec![setup];
    proj
}

/// Builds the Phase 23 **Indexed 4-Sided Bar (Model Bounds & Auto-Rotate 3+1)** sample
/// (`--sample indexed-model-bounds`) on `sample_indexed_4sided_bar_step_bytes()` (`70 × 30 × 30 mm`
/// 4-sided sculpted bar with distinct features on `0°`, `90°`, `180°`, and `270°` faces):
/// - Held in `FixedBox` stock (`82 × 34 × 34 mm`) using **`FixedBoxZAlign::CenterZ`** squarely
///   centered on the `X` (`A`) rotary axis with a `-X` chuck (`chuck_length = 20 mm`,
///   `chuck_diameter = 65 mm`),
/// - Sequences 4 `Indexed3Plus1` operations (`0°`, `90°`, `180°` `Adaptive3D` + `270°` `Parallel3D`
///   finishing pass), each using `MachiningBoundaryMode::ModelBoundingBox` with explicit signed
///   per-side offsets (`-X = -1.0 mm` keeping extra clearance from the chuck, `+X = +2.0 mm`,
///   `-Y/+Y = +1.5 mm`) and automatic viewport upright rotation on operation selection.
pub fn build_indexed_model_bounds_project(
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Indexed 4-Sided Bar (Model Bounds & Auto-Rotate 3+1)");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        role: Default::default(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    }];

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

    let flat_6mm = find_flat_endmill(&proj.tools, 6.0, 0.5).or_else(|| proj.tools.first().cloned());
    let ball_6mm = proj
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::BallEndmill { diameter_mm, .. } if (diameter_mm - 6.0).abs() < 0.5
            )
        })
        .cloned()
        .or_else(|| flat_6mm.clone());

    let mut setup = Setup::new(
        "setup_1",
        "Setup 1 — Indexed 3+1 Bar (Center-Z Stock)",
        WorkOffsetSlot::G54,
    );
    setup.post_profile = "linuxcnc_4axis".to_string();
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::VOLUMETRIC_CENTER,
    };
    setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(88.0),
        depth_y: ParamLength::from_mm(34.0),
        height_z: ParamLength::from_mm(34.0),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::CenterZ,
    };
    setup.rotary.mode = RotaryDriveMode::Indexed3Plus1;
    setup.rotary.axis = RotaryAxisKind::XAxis;
    setup.rotary.direction = RotaryDirection::RightHandCcw;
    setup.rotary.clearance_radial_offset = ParamLength::from_mm(10.0);
    setup.rotary.chuck_end_positive = false;
    setup.rotary.chuck_length = ParamLength::from_mm(20.0);
    setup.rotary.chuck_diameter = ParamLength::from_mm(65.0);

    // Ops 1..3: Indexed 0°, 90°, 180° Adaptive3D Roughing with ModelBoundingBox + signed offsets
    if let Some(em) = &flat_6mm {
        for (id, name, angle_deg) in [
            (
                "op_1_idx0_adaptive",
                "Indexed 0° Adaptive Rough (Model Bounds)",
                0.0_f64,
            ),
            (
                "op_2_idx90_adaptive",
                "Indexed 90° Adaptive Rough (Model Bounds)",
                90.0_f64,
            ),
            (
                "op_3_idx180_adaptive",
                "Indexed 180° Adaptive Rough (Model Bounds)",
                180.0_f64,
            ),
        ] {
            let mut op = Operation::new(id, name, OperationKind::Adaptive3D);
            op.tool_id = Some(em.id.clone());
            op.preset_id = em.presets.first().map(|p| p.id.clone());
            op.rotary = OperationRotaryMode::Indexed3Plus1 { angle_deg };
            op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
            op.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -4.0);
            op.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
            op.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
            if let StrategyParams::Adaptive3D(p) = op.strategy_mut() {
                p.boundary.boundary_mode = MachiningBoundaryMode::ModelBoundingBox;
                p.boundary.model_bounds_offset_x_neg = ParamLength::from_mm(-1.0);
                p.boundary.model_bounds_offset_x_pos = ParamLength::from_mm(2.0);
                p.boundary.model_bounds_offset_y_neg = ParamLength::from_mm(1.5);
                p.boundary.model_bounds_offset_y_pos = ParamLength::from_mm(1.5);
                p.max_roughing_stepdown = ParamLength::from_mm(3.0);
                p.fine_stepdown = ParamLength::from_mm(1.5);
                p.detect_flats = true;
                p.adaptive.optimal_load = ParamLength::from_mm(1.8);
            }
            setup.operations.push(op);
        }
    }

    // Op 4: Indexed 270° Parallel3D Finishing over the convex circular-arc ridge (Model Bounds)
    if let Some(ball) = &ball_6mm {
        let mut op4 = Operation::new(
            "op_4_idx270_parallel_finish",
            "Indexed 270° Parallel Finish (Model Bounds)",
            OperationKind::Parallel3D,
        );
        op4.tool_id = Some(ball.id.clone());
        op4.preset_id = ball.presets.first().map(|p| p.id.clone());
        op4.rotary = OperationRotaryMode::Indexed3Plus1 { angle_deg: 270.0 };
        op4.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op4.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -4.0);
        op4.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op4.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        if let StrategyParams::Parallel3D(p) = op4.strategy_mut() {
            p.boundary.boundary_mode = MachiningBoundaryMode::ModelBoundingBox;
            p.boundary.model_bounds_offset_x_neg = ParamLength::from_mm(-1.0);
            p.boundary.model_bounds_offset_x_pos = ParamLength::from_mm(2.0);
            p.boundary.model_bounds_offset_y_neg = ParamLength::from_mm(1.5);
            p.boundary.model_bounds_offset_y_pos = ParamLength::from_mm(1.5);
            p.boundary.tool_containment = ToolContainmentMode::ToolCenterOnBoundary;
            p.use_cusp_height = true;
            p.cusp_height = ParamLength::from_mm(0.03);
            p.pass_angle_deg = 0.0;
        }
        setup.operations.push(op4);
    }

    proj.setups = vec![setup];
    proj
}
