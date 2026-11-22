use chip_cam::{
    BoxAnchor27, HeightReference, HeightSpec, Operation, OperationKind, StockConfig,
    StrategyParams, WcsOriginMode,
};
use chip_math::{DisplayUnit, ParamLength};
use chip_tools::{CutterGeometry, Tool};

use crate::document::{ModelEntry, ProjectDocument};

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
        transform: chip_cad::ModelTransform::IDENTITY,
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
        transform: chip_cad::ModelTransform::IDENTITY,
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
