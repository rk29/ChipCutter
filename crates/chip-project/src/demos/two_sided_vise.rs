//! Built-in **Two-Sided Vise & Soft-Jaw Workholding** sample project (`--sample two-sided-vise`,
//! Milestone 63).
//!
//! Showcases the complete Phase 17 shop-floor workholding & multi-setup volumetric carryover
//! workflow on a watertight stepped aluminum bracket/manifold (`64 × 50 × 16 mm`):
//! - **`Setup 1 — Top Op (G54)`:** Held in a parametric `ViseJaws` fixture (with under-stock
//!   parallels) plus two `ToeClamp`s on the sacrificial side ears, demonstrating automatic
//!   safe-rapid lifting over clamps and zero `FixtureCollision` errors.
//! - **`Setup 2 — Flip Bottom Op (G55)`:** Flipped `180°` around WCS X with
//!   `carryover_from_setup_id = Some("setup_1")`, held in stepped soft-jaw `ViseJaws`,
//!   running rest-aware hat-stock facing and skirt profiling with cumulative `In-Tol >= 95%`
//!   and `0` critical errors.

use chip_cam::{
    BoxAnchor27, FixtureBody, FixtureKind, FixtureZRef, HeightReference, HeightSpec,
    MachiningBoundaryMode, Operation, OperationKind, Setup, StockConfig, StrategyParams,
    ViseClampAxis, ViseFixedSide, WcsOrientationMode, WcsOriginMode, WorkOffsetSlot,
};
use chip_math::{DVec2, ParamLength};
use chip_tools::{CutterGeometry, Tool};

use crate::document::{ModelEntry, ProjectDocument};

/// Generates a deterministic watertight binary STL of the built-in two-sided aluminum manifold
/// bracket (`64 × 50 × 16 mm`):
/// - Lower flange: `X ∈ [0, 64], Y ∈ [0, 50], Z ∈ [0, 10]`
/// - Upper centered stepped boss: `X ∈ [14, 50], Y ∈ [11, 39], Z ∈ [10, 16]`
pub fn sample_two_sided_vise_part_stl_bytes() -> Vec<u8> {
    let mut tris: Vec<([f32; 3], [f32; 3], [f32; 3])> = Vec::new();
    let mut push_quad = |a: [f32; 3], b: [f32; 3], c: [f32; 3], d: [f32; 3]| {
        tris.push((a, b, c));
        tris.push((a, c, d));
    };

    let (x0, x1, y0, y1, z0, z1) = (0.0_f32, 64.0_f32, 0.0_f32, 50.0_f32, 0.0_f32, 10.0_f32);
    let (bx0, bx1, by0, by1, bz1) = (14.0_f32, 50.0_f32, 11.0_f32, 39.0_f32, 16.0_f32);
    let xs = [x0, bx0, bx1, x1];
    let ys = [y0, by0, by1, y1];

    // Bottom face (Z = 0, normal -Z) on the exact 3x3 grid so every edge matches the side walls
    for ix in 0..3 {
        for iy in 0..3 {
            push_quad(
                [xs[ix], ys[iy], z0],
                [xs[ix], ys[iy + 1], z0],
                [xs[ix + 1], ys[iy + 1], z0],
                [xs[ix + 1], ys[iy], z0],
            );
        }
    }

    // Lower flange outer walls (Z = 0..10), split cleanly at bx0/bx1 and by0/by1 (zero T-junctions!)
    for ix in 0..3 {
        push_quad(
            [xs[ix], y0, z0],
            [xs[ix + 1], y0, z0],
            [xs[ix + 1], y0, z1],
            [xs[ix], y0, z1],
        ); // -Y
        push_quad(
            [xs[ix + 1], y1, z0],
            [xs[ix], y1, z0],
            [xs[ix], y1, z1],
            [xs[ix + 1], y1, z1],
        ); // +Y
    }
    for iy in 0..3 {
        push_quad(
            [x0, ys[iy + 1], z0],
            [x0, ys[iy], z0],
            [x0, ys[iy], z1],
            [x0, ys[iy + 1], z1],
        ); // -X
        push_quad(
            [x1, ys[iy], z0],
            [x1, ys[iy + 1], z0],
            [x1, ys[iy + 1], z1],
            [x1, ys[iy], z1],
        ); // +X
    }

    // Horizontal stepped flange shelf at Z = 10 surrounding upper boss (8 outer cells of 3x3 grid)
    for ix in 0..3 {
        for iy in 0..3 {
            if ix == 1 && iy == 1 {
                continue;
            }
            push_quad(
                [xs[ix], ys[iy], z1],
                [xs[ix + 1], ys[iy], z1],
                [xs[ix + 1], ys[iy + 1], z1],
                [xs[ix], ys[iy + 1], z1],
            );
        }
    }

    // Upper boss walls (Z = 10..16)
    push_quad(
        [bx0, by0, z1],
        [bx1, by0, z1],
        [bx1, by0, bz1],
        [bx0, by0, bz1],
    );
    push_quad(
        [bx1, by1, z1],
        [bx0, by1, z1],
        [bx0, by1, bz1],
        [bx1, by1, bz1],
    );
    push_quad(
        [bx0, by1, z1],
        [bx0, by0, z1],
        [bx0, by0, bz1],
        [bx0, by1, bz1],
    );
    push_quad(
        [bx1, by0, z1],
        [bx1, by1, z1],
        [bx1, by1, bz1],
        [bx1, by0, bz1],
    );

    // Upper boss top face (Z = 16, normal +Z)
    push_quad(
        [bx0, by0, bz1],
        [bx1, by0, bz1],
        [bx1, by1, bz1],
        [bx0, by1, bz1],
    );

    let mut bytes = Vec::with_capacity(84 + tris.len() * 50);
    let mut header = [0u8; 80];
    let title = b"ChipCutter Two-Sided Vise Bracket (64x50x16mm)";
    header[..title.len()].copy_from_slice(title);
    bytes.extend_from_slice(&header);
    bytes.extend_from_slice(&(tris.len() as u32).to_le_bytes());

    for (v0, v1, v2) in tris {
        let e1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
        let e2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];
        let nx = e1[1] * e2[2] - e1[2] * e2[1];
        let ny = e1[2] * e2[0] - e1[0] * e2[2];
        let nz = e1[0] * e2[1] - e1[1] * e2[0];
        let len = (nx * nx + ny * ny + nz * nz).sqrt().max(1e-12);
        for c in [nx / len, ny / len, nz / len] {
            bytes.extend_from_slice(&c.to_le_bytes());
        }
        for v in [v0, v1, v2] {
            for c in v {
                bytes.extend_from_slice(&c.to_le_bytes());
            }
        }
        bytes.extend_from_slice(&0u16.to_le_bytes());
    }
    bytes
}

/// Builds the built-in **Two-Sided Vise & Soft-Jaw Workholding** [`ProjectDocument`]
/// (`--sample two-sided-vise`) with Setup 1 (Vise + Parallels + Toe Clamps) and Setup 2
/// (180° Flipped Op with `carryover_from_setup_id = Some("setup_1")` & Stepped Soft Jaws).
pub fn build_two_sided_vise_workholding_project(
    model_path: String,
    sha256_hex: &str,
    available_tools: &[Tool],
) -> ProjectDocument {
    let mut proj = ProjectDocument::new("Two-Sided Vise & Soft-Jaw Bracket");
    proj.models = vec![ModelEntry {
        id: "model_1".to_string(),
        path: model_path,
        sha256: sha256_hex.to_string(),
        role: Default::default(),
        unit_override: Some(chip_cad::MeshUnitKind::Millimeter),
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
        .cloned()
        .or_else(|| proj.tools.first().cloned());

    // ── Setup 1 — Top Op (G54) ────────────────────────────────────────────────────────────────
    // Raw 70 × 56 × 22 mm aluminum billet: 3 mm side margins, 1 mm top facing allowance,
    // 5 mm bottom hat stock held in the machinist vise + parallels, plus 2 end toe clamps
    // positioned clear of the top boss/flange envelope while triggering safe-rapid lifts.
    let mut setup_1 = Setup::new("setup_1", "Setup 1 — Top Op (G54)", WorkOffsetSlot::G54);
    setup_1.wcs.orientation = WcsOrientationMode::ModelOrientation {
        flip_z: false,
        flip_x: false,
        clock_90: 0,
        extra_z_rot_deg: 0.0,
    };
    setup_1.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup_1.stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(3.0),
        side_neg_x: ParamLength::from_mm(3.0),
        side_pos_y: ParamLength::from_mm(3.0),
        side_neg_y: ParamLength::from_mm(3.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(5.0),
        sheet_thickness_z: ParamLength::from_mm(12.0),
        round_increment_mm: None,
    };
    setup_1.auto_avoid_fixtures_on_rapids = true;

    // 1. Machinist Vise + Parallels gripping the 5 mm bottom hat stock along Y
    let mut vise_op1 = FixtureBody::default_vise("vise_op1", "Machinist Vise + Parallels");
    vise_op1.clearance_margin = ParamLength::from_mm(1.0);
    if let FixtureKind::ViseJaws {
        clamp_axis,
        jaw_width,
        jaw_thickness,
        jaw_height,
        grip_depth,
        include_parallels,
        fixed_jaw_side,
        ..
    } = &mut vise_op1.kind
    {
        *clamp_axis = ViseClampAxis::Y;
        *jaw_width = ParamLength::from_mm(100.0);
        *jaw_thickness = ParamLength::from_mm(18.0);
        *jaw_height = ParamLength::from_mm(38.0);
        *grip_depth = ParamLength::from_mm(4.5);
        *include_parallels = true;
        *fixed_jaw_side = ViseFixedSide::Positive;
    }
    setup_1.fixtures.push(vise_op1);

    // 2. Two Toe Clamps anchored on the left/right vise-jaw shoulders outside the cutter
    //    sweep envelope (keeping Op-1 cuts collision-free while tall stud towers demonstrate
    //    automatic G0 rapid-lift avoidance!)
    for (id, name, anchor_xy, yaw_deg) in [
        (
            "clamp_right",
            "End Toe Clamp (+X)",
            DVec2::new(48.5, 0.0),
            0.0,
        ),
        (
            "clamp_left",
            "End Toe Clamp (-X)",
            DVec2::new(-48.5, 0.0),
            180.0,
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
            *z_offset = ParamLength::from_mm(-4.0);
            *strap_length = ParamLength::from_mm(38.0);
            *strap_width = ParamLength::from_mm(14.0);
            *strap_height = ParamLength::from_mm(9.0);
            *nose_overhang = ParamLength::from_mm(1.5);
            *stud_diameter = ParamLength::from_mm(8.0);
            *stud_height_above_strap = ParamLength::from_mm(14.0);
        }
        setup_1.fixtures.push(clamp);
    }

    if let Some(em) = &flat_6mm {
        // Op 1.1: Face 1 mm top allowance cleanly down to ModelTop
        let mut op1_face =
            Operation::new("op1_face_top", "1. Face Stock Top", OperationKind::Facing);
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

        // Op 1.2: 3D Adaptive Top Boss & Flange down to ModelBottom + 2 mm (leaving 2 mm safe
        // skirt above the 5 mm vise-gripped hat stock!)
        let mut op1_adapt = Operation::new(
            "op1_adaptive_top",
            "2. 3D Adaptive Top Boss & Flange",
            OperationKind::Adaptive3D,
        );
        op1_adapt.tool_id = Some(em.id.clone());
        op1_adapt.preset_id = em.presets.first().map(|p| p.id.clone());
        op1_adapt.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op1_adapt.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 2.0);
        op1_adapt.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op1_adapt.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        if let StrategyParams::Adaptive3D(p) = op1_adapt.strategy_mut() {
            p.boundary_mode = MachiningBoundaryMode::StockBox;
            p.max_roughing_stepdown = ParamLength::from_mm(5.0);
            p.fine_stepdown = ParamLength::from_mm(2.0);
            p.detect_flats = true;
            p.adaptive.optimal_load = ParamLength::from_mm(2.2);
        }
        setup_1.operations.push(op1_adapt);
    }

    // ── Setup 2 — Flip Bottom Op (G55) ────────────────────────────────────────────────────────
    // Flipped 180° around WCS X so the 5 mm hat stock sits on top (+Z), with volumetric Tri-Dexel
    // stock carryover from Setup 1 and stepped soft-jaw ViseJaws gripping the finished Op-1 boss!
    let mut setup_2 = Setup::new(
        "setup_2",
        "Setup 2 — Flip Bottom Op (G55)",
        WorkOffsetSlot::G55,
    );
    setup_2.wcs.orientation = WcsOrientationMode::ModelOrientation {
        flip_z: true,
        flip_x: false,
        clock_90: 0,
        extra_z_rot_deg: 0.0,
    };
    setup_2.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    setup_2.stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(3.0),
        side_neg_x: ParamLength::from_mm(3.0),
        side_pos_y: ParamLength::from_mm(3.0),
        side_neg_y: ParamLength::from_mm(3.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(5.0),
        bottom_offset_z: ParamLength::from_mm(1.0),
        sheet_thickness_z: ParamLength::from_mm(12.0),
        round_increment_mm: None,
    };
    setup_2.carryover_from_setup_id = Some("setup_1".to_string());
    setup_2.auto_avoid_fixtures_on_rapids = true;

    // Stepped Soft-Jaw Vise fixture gripping the already-machined Op-1 boss below Z = ModelBottom + 6 mm
    let mut soft_jaws = FixtureBody::default_vise("soft_jaws_op2", "Contoured Soft Jaws (Op 2)");
    soft_jaws.clearance_margin = ParamLength::from_mm(1.0);
    soft_jaws.color_rgba = Some([0.36, 0.62, 0.78, 0.40]);
    if let FixtureKind::ViseJaws {
        clamp_axis,
        jaw_width,
        jaw_thickness,
        jaw_height,
        grip_depth,
        step_shelf_depth,
        step_shelf_height,
        include_parallels,
        fixed_jaw_side,
        ..
    } = &mut soft_jaws.kind
    {
        *clamp_axis = ViseClampAxis::Y;
        *jaw_width = ParamLength::from_mm(100.0);
        *jaw_thickness = ParamLength::from_mm(22.0);
        *jaw_height = ParamLength::from_mm(38.0);
        *grip_depth = ParamLength::from_mm(7.0);
        *step_shelf_depth = ParamLength::from_mm(4.0);
        *step_shelf_height = ParamLength::from_mm(4.0);
        *include_parallels = false;
        *fixed_jaw_side = ViseFixedSide::Positive;
    }
    setup_2.fixtures.push(soft_jaws);

    if let Some(em) = &flat_6mm {
        // Op 2.1: Face off the 5 mm bottom hat stock cleanly down to flipped ModelTop
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
        op2_face.passes.max_stepdown = ParamLength::from_mm(2.5);
        op2_face.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        if let StrategyParams::Facing(p) = op2_face.strategy_mut() {
            p.pass_extension = ParamLength::from_mm(1.5);
        }
        setup_2.operations.push(op2_face);

        // Op 2.2: Rest-aware 3D Adaptive skirt profiling — carries over Op-1's machined Tri-Dexel
        // solid so it clears ONLY the remaining 2 mm flange skirt and skips the 14 mm already
        // finished in Op-1 (avoiding both air cuts and the soft jaws below!)
        let mut op2_rest = Operation::new(
            "op2_rest_skirt",
            "2. Rest Profile Bottom Skirt",
            OperationKind::Adaptive3D,
        );
        op2_rest.tool_id = Some(em.id.clone());
        op2_rest.preset_id = em.presets.first().map(|p| p.id.clone());
        op2_rest.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
        op2_rest.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -3.5);
        op2_rest.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
        op2_rest.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
        if let StrategyParams::Adaptive3D(p) = op2_rest.strategy_mut() {
            p.boundary_mode = MachiningBoundaryMode::StockBox;
            p.max_roughing_stepdown = ParamLength::from_mm(3.5);
            p.fine_stepdown = ParamLength::from_mm(2.0);
            p.detect_flats = true;
            p.adaptive.optimal_load = ParamLength::from_mm(2.2);
        }
        setup_2.operations.push(op2_rest);
    }

    proj.setups = vec![setup_1, setup_2];
    proj
}
