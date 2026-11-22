use chip_cad::{CadDocument, SurfaceKind};
use chip_cam::{
    BoxAnchor27, FixedBoxXyAlign, FixedBoxZAlign, GeometrySelectionItem, HeightReference,
    HeightSpec, Operation, OperationKind, StockConfig, StrategyParams, WcsOriginMode,
};
use chip_math::{DisplayUnit, ParamLength, approx_eq};
use chip_sketch::{
    RectAnchorMode, SketchEntity, SketchEntityEntry, SketchFontSource, SketchLayer, SketchTextSpec,
    TextHAlign, TextVAlign,
};
use chip_tools::{CutterGeometry, Tool};

use crate::document::{ModelEntry, ProjectDocument};

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
        transform: chip_cad::ModelTransform::IDENTITY,
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
        transform: chip_cad::ModelTransform::IDENTITY,
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

/// Builds the Milestone 23 **Tutorial 10 — Rotary Wrapping & Surfacing (`AxialParallel` +
/// `ContinuousSpiral`)** [`ProjectDocument`] on `09-rotary-shaft.stl` (`Ø40 × 80 mm` shaft in
/// `Ø42 × 80 mm` cylindrical stock on `X/A`, `TwoLinearPlusRotary`, `linuxcnc_4axis`):
/// - **Op 1:** `Rotary Continuous Spiral (360°)` roughing & finishing the collar + 3-lobe cam,
/// - **Op 2:** `Rotary Axial Parallel (-60°..+60°)` lengthwise raster finishing over a partial
///   angular sector (`-60°` to `+60°`).
pub fn build_tutorial_rotary_axial_parallel_project(
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Tutorial 10 — Rotary Axial Parallel & Spiral");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    }];

    let setup = &mut proj.setups[0];
    setup.name = "Setup 1 — 2-Linear Rotary (Z+X+A)".to_string();
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
    setup.rotary.mode = chip_cam::RotaryDriveMode::TwoLinearPlusRotary;
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

    // Op 1: Full-circumference Continuous Helical Spiral (0°..360°)
    let mut op_spiral = Operation::new(
        "op_1_rotary_spiral",
        "Rotary Continuous Spiral (360°)",
        OperationKind::Rotary2Linear,
    );
    op_spiral.tool_id = ball_6mm.clone();
    op_spiral.passes.multiple_depths = true;
    op_spiral.passes.max_stepdown = ParamLength::from_mm(3.0);
    if let StrategyParams::Rotary2Linear(p) = op_spiral.strategy_mut() {
        p.pattern = chip_cam::RotaryPatternMode::ContinuousSpiral;
        p.cut_direction = chip_cam::RotaryCutDirection::ZigZag;
        p.stepover = ParamLength::from_mm(1.6);
        p.start_angle_deg = 0.0;
        p.end_angle_deg = 360.0;
        p.axial_margin_start = ParamLength::from_mm(34.0);
        p.axial_margin_end = ParamLength::from_mm(8.0);
        p.angular_sample_deg = 5.0;
        p.axial_sample_step = ParamLength::from_mm(1.0);
    }

    // Op 2: Partial-sector Axial Parallel lengthwise raster (-60°..+60°)
    let mut op_axial = Operation::new(
        "op_2_rotary_axial_sector",
        "Rotary Axial Parallel (-60°..+60°)",
        OperationKind::Rotary2Linear,
    );
    op_axial.tool_id = ball_6mm;
    op_axial.passes.multiple_depths = false;
    if let StrategyParams::Rotary2Linear(p) = op_axial.strategy_mut() {
        p.pattern = chip_cam::RotaryPatternMode::AxialParallel;
        p.cut_direction = chip_cam::RotaryCutDirection::ZigZag;
        p.stepover = ParamLength::from_mm(1.2);
        p.start_angle_deg = -60.0;
        p.end_angle_deg = 60.0;
        p.axial_margin_start = ParamLength::from_mm(34.0);
        p.axial_margin_end = ParamLength::from_mm(8.0);
        p.angular_sample_deg = 4.0;
        p.axial_sample_step = ParamLength::from_mm(1.0);
    }

    proj.setups[0].operations = vec![op_spiral, op_axial];
    proj
}

/// Builds the Milestone 23 **Tutorial 11 — Indexed `3+1` Multi-Face STEP Block** [`ProjectDocument`]
/// for `11-indexed-multiface-block.step` (`70 × 32 × 32 mm` prismatic block with 4 cardinal faces
/// plus a `45°` beveled mounting facet) chucked in `90 × 36 × 36 mm` square bar stock on `X/A`
/// (`Indexed3Plus1`, `linuxcnc_4axis`).
///
/// Every operation's rotary index angle (`0°`, `45°`, `90°`, `180°`, `270°`) is computed directly
/// from the STEP face normal via [`chip_cam::compute_index_angle_for_normal`].
pub fn build_tutorial_indexed_step_multiface_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Tutorial 11 — Indexed 3+1 Multi-Face STEP");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    }];

    {
        let setup = &mut proj.setups[0];
        setup.name = "Setup 1 — Indexed 3+1 Square Bar (X/A)".to_string();
        setup.post_profile = "linuxcnc_4axis".to_string();
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(90.0),
            depth_y: ParamLength::from_mm(36.0),
            height_z: ParamLength::from_mm(36.0),
            xy_align: FixedBoxXyAlign::CenterXy,
            z_align: FixedBoxZAlign::CenterZ,
        };
        setup.rotary.mode = chip_cam::RotaryDriveMode::Indexed3Plus1;
        setup.rotary.axis = chip_cam::RotaryAxisKind::XAxis;
        setup.rotary.direction = chip_cam::RotaryDirection::RightHandCcw;
        setup.rotary.clearance_radial_offset = ParamLength::from_mm(10.0);
        setup.rotary.chuck_length = ParamLength::from_mm(20.0);
        setup.rotary.chuck_diameter = ParamLength::from_mm(65.0);
    }

    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }

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
        .cloned();

    let resolved = proj.setups[0].resolve(doc, DisplayUnit::Millimeter).ok();
    let rot_cfg = resolved.as_ref().and_then(|r| r.rotary);
    let setup_frame = resolved.as_ref().map(|r| r.frame);

    let index_angle_for_world_normal = |world_n: chip_math::DVec3, fallback_deg: f64| -> f64 {
        match (setup_frame, rot_cfg) {
            (Some(frame), Some(rot)) => {
                let n_wcs = frame.world_to_local_dir(world_n);
                chip_cam::compute_index_angle_for_normal(n_wcs, &rot).unwrap_or(fallback_deg)
            }
            _ => fallback_deg,
        }
    };

    let Some(em) = flat_6mm else {
        return proj;
    };

    let mut ops = Vec::new();

    // Op 1 (0°): Face +Z top land (Z = +16 mm) removing 2 mm bar-stock allowance
    if let Some((face_id, world_n)) = doc.faces.iter().find_map(|f| match f.surface {
        SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && approx_eq(origin_mm.z, 16.0, 0.05) =>
        {
            Some((f.id, normal))
        }
        _ => None,
    }) && let Some(bind) = doc.bind_face(face_id)
    {
        let angle_deg = index_angle_for_world_normal(world_n, 0.0);
        let mut op = Operation::new(
            "op_1_face_plus_z_0deg",
            "Indexed 0° — Face +Z Top",
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        op.passes.multiple_depths = true;
        op.passes.max_stepdown = ParamLength::from_mm(2.0);
        op.rotary = chip_cam::OperationRotaryMode::Indexed3Plus1 { angle_deg };
        ops.push(op);
    }

    // Op 2 (45°): Mill angled 45° beveled mounting facet via `compute_index_angle_for_normal`!
    if let Some((face_id, world_n)) = doc.faces.iter().find_map(|f| match f.surface {
        SurfaceKind::Plane { normal, .. } if normal.y > 0.65 && normal.z > 0.65 => {
            Some((f.id, normal))
        }
        _ => None,
    }) && let Some(bind) = doc.bind_face(face_id)
    {
        let angle_deg = index_angle_for_world_normal(world_n, 45.0);
        let mut op = Operation::new(
            "op_2_bevel_45deg",
            "Indexed 45° — Angled Mounting Bevel",
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        op.passes.multiple_depths = true;
        op.passes.max_stepdown = ParamLength::from_mm(2.5);
        op.rotary = chip_cam::OperationRotaryMode::Indexed3Plus1 { angle_deg };
        ops.push(op);
    }

    // Op 3 (90°): Mill +Y cardinal face (Y = +16 mm)
    if let Some((face_id, world_n)) = doc.faces.iter().find_map(|f| match f.surface {
        SurfaceKind::Plane { origin_mm, normal }
            if normal.y > 0.99 && approx_eq(origin_mm.y, 16.0, 0.05) =>
        {
            Some((f.id, normal))
        }
        _ => None,
    }) && let Some(bind) = doc.bind_face(face_id)
    {
        let angle_deg = index_angle_for_world_normal(world_n, 90.0);
        let mut op = Operation::new(
            "op_3_face_plus_y_90deg",
            "Indexed 90° — Face +Y Side",
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        op.passes.multiple_depths = true;
        op.passes.max_stepdown = ParamLength::from_mm(2.0);
        op.rotary = chip_cam::OperationRotaryMode::Indexed3Plus1 { angle_deg };
        ops.push(op);
    }

    // Op 4 (180°): Mill -Z cardinal face (Z = -16 mm)
    if let Some((face_id, world_n)) = doc.faces.iter().find_map(|f| match f.surface {
        SurfaceKind::Plane { origin_mm, normal }
            if normal.z < -0.99 && approx_eq(origin_mm.z, -16.0, 0.05) =>
        {
            Some((f.id, normal))
        }
        _ => None,
    }) && let Some(bind) = doc.bind_face(face_id)
    {
        let angle_deg = index_angle_for_world_normal(world_n, 180.0);
        let mut op = Operation::new(
            "op_4_face_minus_z_180deg",
            "Indexed 180° — Face -Z Bottom",
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        op.passes.multiple_depths = true;
        op.passes.max_stepdown = ParamLength::from_mm(2.0);
        op.rotary = chip_cam::OperationRotaryMode::Indexed3Plus1 { angle_deg };
        ops.push(op);
    }

    // Op 5 (270°): Mill -Y cardinal face (Y = -16 mm)
    if let Some((face_id, world_n)) = doc.faces.iter().find_map(|f| match f.surface {
        SurfaceKind::Plane { origin_mm, normal }
            if normal.y < -0.99 && approx_eq(origin_mm.y, -16.0, 0.05) =>
        {
            Some((f.id, normal))
        }
        _ => None,
    }) && let Some(bind) = doc.bind_face(face_id)
    {
        let angle_deg = index_angle_for_world_normal(world_n, 270.0);
        let mut op = Operation::new(
            "op_5_face_minus_y_270deg",
            "Indexed 270° — Face -Y Side",
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(em.id.clone());
        op.preset_id = em.presets.first().map(|p| p.id.clone());
        op.selections
            .push(GeometrySelectionItem::PlanarFace { binding: bind });
        op.passes.multiple_depths = true;
        op.passes.max_stepdown = ParamLength::from_mm(2.0);
        op.rotary = chip_cam::OperationRotaryMode::Indexed3Plus1 { angle_deg };
        ops.push(op);
    }

    proj.setups[0].operations = ops;
    proj
}

/// Builds the Milestone 23 **Tutorial 12 — Simultaneous 4-Axis Milling (`Auto Surface-Normal Tilt`
/// vs. `Fixed Lateral Offset` & `SimultaneousSpiral` vs. `HelicalFlowline`)** [`ProjectDocument`]
/// on `10-helical-flute-shaft.stl` (`Ø40 × 80 mm` 3-start helical flutes in `Ø42 × 80 mm` stock):
/// - **Op 1:** `4-Axis Helical Roughing (Auto-Tilt)` (`SimultaneousSpiral`, `16°` max auto tilt, `3 mm` stepdowns),
/// - **Op 2:** `4-Axis Spiral Finish (Auto-Tilt 18°)` (`SimultaneousSpiral`, `SurfaceNormalAutoTilt`, `18°`, `3 cells`),
/// - **Op 3:** `4-Axis Flowline (Fixed Offset 2.5 mm)` (`HelicalFlowline`, `FixedLateralOffset = 2.5 mm`).
pub fn build_tutorial_rotary_4axis_flowline_project(
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Tutorial 12 — Simultaneous 4-Axis Auto-Tilt & Flowline");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    }];

    let setup = &mut proj.setups[0];
    setup.name = "Setup 1 — Simultaneous 4-Axis A-Chuck".to_string();
    setup.post_profile = "grblhal_4axis".to_string();
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
        "op_1_4ax_rough_spiral",
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

    // Op 2: Simultaneous Helical Spiral Finish with Auto Surface-Normal Tilt (18°, 3 cells)
    let mut op_spiral_fin = Operation::new(
        "op_2_4ax_finish_spiral",
        "4-Axis Spiral Finish (Auto-Tilt 18°)",
        OperationKind::Rotary4AxisSim,
    );
    op_spiral_fin.tool_id = ball_6mm.clone();
    op_spiral_fin.passes.multiple_depths = false;
    op_spiral_fin.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
    op_spiral_fin.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    if let StrategyParams::Rotary4AxisSim(p) = op_spiral_fin.strategy_mut() {
        p.pattern = chip_cam::Rotary4AxisPattern::SimultaneousSpiral;
        p.tilt_mode = chip_cam::RotaryTiltMode::SurfaceNormalAutoTilt;
        p.max_tilt_deg = 18.0;
        p.tilt_smoothing_window = 3;
        p.stepover = ParamLength::from_mm(1.5);
        p.axial_margin_start = ParamLength::from_mm(11.0);
        p.axial_margin_end = ParamLength::from_mm(11.0);
    }

    // Op 3: Axial / Helical Flowline with Fixed Lateral Offset (2.5 mm off centerline)
    let mut op_flowline = Operation::new(
        "op_3_4ax_flowline_offset",
        "4-Axis Flowline (Fixed Offset 2.5 mm)",
        OperationKind::Rotary4AxisSim,
    );
    op_flowline.tool_id = ball_6mm;
    op_flowline.passes.multiple_depths = false;
    op_flowline.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
    op_flowline.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    if let StrategyParams::Rotary4AxisSim(p) = op_flowline.strategy_mut() {
        p.pattern = chip_cam::Rotary4AxisPattern::HelicalFlowline;
        p.tilt_mode = chip_cam::RotaryTiltMode::FixedLateralOffset;
        p.lateral_offset = ParamLength::from_mm(2.5);
        p.stepover = ParamLength::from_mm(2.2);
        p.axial_margin_start = ParamLength::from_mm(11.0);
        p.axial_margin_end = ParamLength::from_mm(11.0);
    }

    proj.setups[0].operations = vec![op_rough, op_spiral_fin, op_flowline];
    proj
}
