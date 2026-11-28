//! Built-in **Undercut & 3D Deburr Block** sample project (`--sample undercut-deburr`,
//! Milestone 67).
//!
//! Showcases the full Phase 18 expanded machining suite on a watertight `100 × 80 × 28 mm`
//! aluminum manifold/fixture block in a single crash-free Setup (`G54`, `Tri-Dexel` engine):
//! 1. **Face Top** (`OperationKind::Facing` + Ø6 mm Flat Endmill)
//! 2. **Pre-Slot Throats (T-Slot & Dovetail)** (`OperationKind::SlotUndercut` Keyway + Ø6 mm Flat Endmill)
//! 3. **M8 T-Slot Undercut** (`OperationKind::SlotUndercut` DIN 650 M8 + Ø12×4 mm SlotMill)
//! 4. **60° Dovetail Guideway** (`OperationKind::Dovetail` + Ø12 mm 60° DovetailCutter)
//! 5. **Lollipop Overhang Undercut** (`OperationKind::Undercut3D` + Ø6 mm LollipopEndmill)
//! 6. **3D Edge Deburr & Back-Chamfer** (`OperationKind::Deburr3D` + Ø6 mm LollipopEndmill)

use chip_cad::{CadDocument, SurfaceKind};
use chip_cam::{
    BoxAnchor27, Deburr3DParams, DovetailMode, DovetailParams, GeometrySelectionItem,
    HeightReference, HeightSpec, Operation, OperationKind, Setup, SlotUndercutMode,
    SlotUndercutParams, SlotUndercutPreset, StockConfig, StrategyParams, Undercut3DParams,
    UndercutEntryMode, WcsOriginMode, WorkOffsetSlot,
};
use chip_math::{ParamLength, approx_eq};
use chip_tools::{CutterGeometry, Tool, starter_undercut_tools};

use chip_project::{ModelEntry, ProjectDocument};

/// Builds the built-in **Undercut & 3D Deburr Block** [`ProjectDocument`]
/// (`--sample undercut-deburr`) for `sample_undercut_deburr_block_step_bytes()`.
pub fn build_undercut_deburr_block_project(
    doc: &CadDocument,
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Undercut & 3D Deburr Block");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        role: Default::default(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
    }];

    // Embed available starter tools plus the Phase 18 undercut cutter trio (Lollipop, SlotMill, Dovetail).
    for t in available_tools {
        proj.ensure_tool_embedded(t);
    }
    for t in starter_undercut_tools() {
        proj.ensure_tool_embedded(&t);
    }

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
    let slot_12mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::SlotMill { .. }))
        .cloned();
    let dovetail_60 = proj
        .tools
        .iter()
        .find(|t| {
            matches!(
                t.geometry,
                CutterGeometry::DovetailCutter { side_angle_deg, .. } if (side_angle_deg - 60.0).abs() < 0.5
            )
        })
        .cloned();
    let lollipop_6mm = proj
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::LollipopEndmill { .. }))
        .cloned();

    // Setup 1 — Top Op (G54): raw 100 × 80 × 29 mm aluminum block (`1.0 mm` top facing allowance)
    let mut setup = Setup::new("setup_1", "Setup 1 — Top Op (G54)", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup.stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(0.0),
        side_neg_x: ParamLength::from_mm(0.0),
        side_pos_y: ParamLength::from_mm(0.0),
        side_neg_y: ParamLength::from_mm(0.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(0.0),
        sheet_thickness_z: ParamLength::from_mm(28.0),
        round_increment_mm: None,
    };

    // Locate exact B-Rep slot floor faces at Z = 14.0 mm:
    //  - M8 T-Slot floor centered around Y = 20.0 mm
    //  - 60° Dovetail guideway floor centered around Y = 52.0 mm
    let tslot_floor = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99
                && approx_eq(origin_mm.z, 14.0, 0.05)
                && f.aabb_mm.center().y < 36.0
        )
    });
    let dovetail_floor = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99
                && approx_eq(origin_mm.z, 14.0, 0.05)
                && f.aabb_mm.center().y >= 36.0
        )
    });

    // Op 1: Face Top (`StockTop -> ModelTop`, 1.0 mm facing allowance with Ø6 mm Flat Endmill)
    if let Some(em) = &flat_6mm {
        let mut op1 = Operation::new("op_1_face_top", "Face Top", OperationKind::Facing);
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

    // Op 2: Pre-Slot Throats (T-Slot & Dovetail) — clears both 8.5 mm vertical throats down to Z=14
    // BEFORE either relieved-neck undercut tool enters!
    if let Some(em) = &flat_6mm {
        let mut op2 = Operation::new(
            "op_2_preslot_throats",
            "Pre-Slot Throats (T-Slot & Dovetail)",
            OperationKind::SlotUndercut,
        );
        op2.tool_id = Some(em.id.clone());
        op2.preset_id = em.presets.first().map(|p| p.id.clone());
        op2.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op2.heights.bottom = HeightSpec::new(HeightReference::FromSelection, 0.0);
        op2.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op2.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        if let Some(f) = tslot_floor
            && let Some(b) = doc.bind_face(f.id)
        {
            op2.selections
                .push(GeometrySelectionItem::PlanarFace { binding: b });
        }
        if let Some(f) = dovetail_floor
            && let Some(b) = doc.bind_face(f.id)
        {
            op2.selections
                .push(GeometrySelectionItem::PlanarFace { binding: b });
        }
        op2.strategy = Some(StrategyParams::SlotUndercut(SlotUndercutParams {
            mode: SlotUndercutMode::Keyway,
            preset: SlotUndercutPreset::Custom,
            slot_width: ParamLength::from_mm(8.5),
            throat_width: ParamLength::from_mm(8.5),
            slot_height: ParamLength::from_mm(14.0),
            radial_stepover: ParamLength::from_mm(1.5),
            axial_stepdown: ParamLength::from_mm(7.0),
            finish_allowance: ParamLength::from_mm(0.0),
            spring_pass: false,
            entry_mode: UndercutEntryMode::BothEndsThrough,
            entry_extension: ParamLength::from_mm(6.0),
            climb_direction: true,
        }));
        setup.operations.push(op2);
    }

    // Op 3: M8 T-Slot Undercut (`OperationKind::SlotUndercut` + `SlotMill`, DIN 650 M8 preset)
    if let (Some(sm), Some(f)) = (&slot_12mm, tslot_floor)
        && let Some(b) = doc.bind_face(f.id)
    {
        let mut p_m8 = SlotUndercutParams::default();
        p_m8.apply_preset(SlotUndercutPreset::TSlotM8);
        p_m8.radial_stepover = ParamLength::from_mm(0.75);
        p_m8.axial_stepdown = ParamLength::from_mm(3.0);
        p_m8.finish_allowance = ParamLength::from_mm(0.15);
        p_m8.spring_pass = true;
        p_m8.entry_mode = UndercutEntryMode::BothEndsThrough;
        p_m8.entry_extension = ParamLength::from_mm(8.0);
        p_m8.climb_direction = true;

        let mut op3 = Operation::new(
            "op_3_tslot_m8",
            "M8 T-Slot Undercut",
            OperationKind::SlotUndercut,
        );
        op3.tool_id = Some(sm.id.clone());
        op3.preset_id = sm.presets.first().map(|p| p.id.clone());
        op3.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op3.heights.bottom = HeightSpec::new(HeightReference::FromSelection, 0.0);
        op3.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op3.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        op3.selections
            .push(GeometrySelectionItem::PlanarFace { binding: b });
        op3.strategy = Some(StrategyParams::SlotUndercut(p_m8));
        setup.operations.push(op3);
    }

    // Op 4: 60° Dovetail Guideway (`OperationKind::Dovetail` + `DovetailCutter`)
    if let (Some(dt), Some(f)) = (&dovetail_60, dovetail_floor)
        && let Some(b) = doc.bind_face(f.id)
    {
        let mut op4 = Operation::new(
            "op_4_dovetail_60",
            "60° Dovetail Guideway",
            OperationKind::Dovetail,
        );
        op4.tool_id = Some(dt.id.clone());
        op4.preset_id = dt.presets.first().map(|p| p.id.clone());
        op4.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op4.heights.bottom = HeightSpec::new(HeightReference::FromSelection, 0.0);
        op4.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op4.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        op4.selections
            .push(GeometrySelectionItem::PlanarFace { binding: b });
        op4.strategy = Some(StrategyParams::Dovetail(DovetailParams {
            mode: DovetailMode::InternalSlot,
            top_throat_width: ParamLength::from_mm(8.5),
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

    // Op 5: Lollipop Overhang Undercut (`OperationKind::Undercut3D` + `LollipopEndmill`)
    // Z-band `[ModelTop - 8.2, ModelTop - 3.5]` (`Z_model ∈ [19.8, 24.5]`) isolates the +Y side
    // overhang groove cleanly above the Z=14..19 dovetail cavity.
    if let Some(lolli) = &lollipop_6mm {
        let mut op5 = Operation::new(
            "op_5_lollipop_undercut",
            "Lollipop Overhang Undercut",
            OperationKind::Undercut3D,
        );
        op5.tool_id = Some(lolli.id.clone());
        op5.preset_id = lolli.presets.first().map(|p| p.id.clone());
        op5.heights.top = HeightSpec::new(HeightReference::ModelTop, -3.5);
        op5.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -8.2);
        op5.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op5.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        op5.strategy = Some(StrategyParams::Undercut3D(Undercut3DParams {
            z_stepdown: ParamLength::from_mm(0.25),
            use_cusp_height: false,
            target_cusp: ParamLength::from_mm(0.02),
            min_overhang_angle_deg: 5.0,
            neck_clearance: ParamLength::from_mm(0.25),
            stock_to_leave: ParamLength::from_mm(0.0),
            lead_out_distance: ParamLength::from_mm(4.0),
            climb_direction: true,
            ..Undercut3DParams::default()
        }));
        setup.operations.push(op5);
    }

    // Op 6: 3D Edge Deburr & Back-Chamfer (`OperationKind::Deburr3D` + `LollipopEndmill`)
    // Breaks all sharp top perimeter & slot-throat edges plus the +Y underside overhang lip.
    if let Some(lolli) = &lollipop_6mm {
        let mut op6 = Operation::new(
            "op_6_deburr_3d",
            "3D Edge Deburr & Back-Chamfer",
            OperationKind::Deburr3D,
        );
        op6.tool_id = Some(lolli.id.clone());
        op6.preset_id = lolli.presets.first().map(|p| p.id.clone());
        op6.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op6.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -6.5);
        op6.strategy = Some(StrategyParams::Deburr3D(Deburr3DParams {
            chamfer_width: ParamLength::from_mm(0.4),
            tip_offset: ParamLength::from_mm(0.5),
            wall_clearance: ParamLength::from_mm(0.15),
            edge_extension: ParamLength::from_mm(0.0),
            auto_detect_edges: true,
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
