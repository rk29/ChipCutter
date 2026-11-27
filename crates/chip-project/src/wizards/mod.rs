//! Shop-floor workholding automation wizards (`chip-project`, Milestone 63):
//!
//! 1. **Two-Sided Flip-Jig Dowel-Pin Wizard** ([`build_two_sided_flip_jig`]):
//!    Computes exact `f64` collision-free dowel-pin registration holes strictly outside the target
//!    part's 2D silhouette and strictly symmetric under `180°` flip about the chosen flip axis,
//!    emitting a `"Flip-Jig Registration Pins"` [`SketchLayer`], Op-1 pin drilling/boring op,
//!    optional spoilboard pin-hole setup, paired `180°` Op-2 Setup (`G55`) with volumetric stock
//!    carryover (`carryover_from_setup_id`), and matching `FixtureKind::CylinderKeepOut` pin bodies.
//! 2. **Auto Soft-Jaw Generator Wizard** ([`build_soft_jaw_plan`]):
//!    Projects the target part's exact `f64` 2D silhouette across the clamping Z band, applies
//!    radial clearance and sharp-corner dogbone reliefs, and intersects with fixed & movable vise
//!    jaws—generating both a ready-to-machine `"Soft Jaws — Fixed & Movable"` Setup + sketch +
//!    Facing/Pocket2D/Contour2D ops AND watertight 3D-printable binary `.stl` buffers.

mod flip_jig;
mod silhouette;
mod soft_jaw;
mod soft_jaw_mesh;

use chip_cam::{CamError, Operation, Setup, StockConfig, ViseClampAxis, WorkOffsetSlot};
use chip_geom2d::RegionSet2D;
use chip_math::{Aabb2, DVec2};
use chip_sketch::SketchLayer;
use chip_tools::ToolId;
use serde::{Deserialize, Serialize};

pub use self::{
    flip_jig::build_two_sided_flip_jig, silhouette::extract_part_silhouette_2d,
    soft_jaw::build_soft_jaw_plan,
};

/// Standard dowel pin diameter presets (`mm`) for the Two-Sided Flip-Jig Wizard.
pub const STANDARD_PIN_DIAMETERS_MM: &[(&str, f64)] = &[
    ("4 mm", 4.0),
    ("6 mm", 6.0),
    ("8 mm", 8.0),
    ("1/8\" (3.175 mm)", 3.175),
    ("1/4\" (6.35 mm)", 6.35),
];

/// Flip axis around which the workpiece is rotated `180°` between Setup 1 and Setup 2.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FlipAxis {
    /// Flip front-to-back around `Y = y_mid` (`X2 = +X1, Y2 = -Y1, Z2 = -Z1`).
    #[default]
    XAxis,
    /// Flip left-to-right around `X = x_mid` (`X2 = -X1, Y2 = +Y1, Z2 = -Z1`).
    YAxis,
}

impl FlipAxis {
    pub const ALL: [Self; 2] = [Self::XAxis, Self::YAxis];

    pub const fn label(self) -> &'static str {
        match self {
            Self::XAxis => "X-Axis (Flip Front-to-Back)",
            Self::YAxis => "Y-Axis (Flip Left-to-Right)",
        }
    }

    pub const fn short_code(self) -> &'static str {
        match self {
            Self::XAxis => "X",
            Self::YAxis => "Y",
        }
    }
}

/// Number and layout of symmetric dowel-pin registration holes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FlipJigPinCount {
    /// 2 pins lying directly on the flip centerline (`v = v_mid`) for guaranteed `180°` invariance.
    #[default]
    TwoOnAxis,
    /// 4 pins arranged in two mirrored pairs (`v = v_mid ± dv`) across the flip axis.
    FourSymmetric,
}

impl FlipJigPinCount {
    pub const ALL: [Self; 2] = [Self::TwoOnAxis, Self::FourSymmetric];

    pub const fn label(self) -> &'static str {
        match self {
            Self::TwoOnAxis => "2 Pins (On Flip Centerline)",
            Self::FourSymmetric => "4 Pins (Mirrored Symmetric Pairs)",
        }
    }

    pub const fn count(self) -> usize {
        match self {
            Self::TwoOnAxis => 2,
            Self::FourSymmetric => 4,
        }
    }
}

/// Configuration for [`build_two_sided_flip_jig`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FlipJigConfig {
    pub flip_axis: FlipAxis,
    /// Nominal dowel pin diameter (`mm`, e.g., `6.0`).
    pub pin_diameter_mm: f64,
    /// Fit clearance added to hole diameter (`mm`, default `0.02` slip/press tuning).
    pub pin_fit_clearance_mm: f64,
    pub pin_count: FlipJigPinCount,
    /// Optional asymmetric offset (`mm`) along the flip axis (`poka-yoke`) so the flipped part
    /// can only seat in the single right orientation while remaining strictly `180°`-flip invariant.
    pub poka_yoke_offset_mm: f64,
    /// Pin hole depth into stock (`<= 0.0` = through-stock, `> 0.0` = blind depth from stock top).
    pub stock_pin_depth_mm: f64,
    /// Mating dowel hole depth into the sacrificial spoilboard (`mm`, default `10.0`).
    pub spoilboard_pin_depth_mm: f64,
    /// Minimum clear margin (`mm`) between pin outer rim and part 2D silhouette.
    pub edge_margin_mm: f64,
    /// Expand Setup 1 / Setup 2 stock margins automatically if current stock is too tight for pins.
    pub expand_stock_if_needed: bool,
    /// Emit an optional `"Spoilboard Dowel Holes"` Setup + drilling/boring operation.
    pub include_spoilboard_setup: bool,
    /// Protrusion height (`mm`) of dowel pins upward from Setup 2 stock bottom for keep-out safety.
    pub pin_protrusion_z_mm: f64,
    /// Optional tool ID pre-assigned to generated pin drilling/boring operations.
    pub pin_tool_id: Option<ToolId>,
    /// Optional cutter diameter (`mm`) corresponding to `pin_tool_id` to pick Drill vs. HelicalBore.
    pub pin_tool_diameter_mm: Option<f64>,
    /// Optional explicit ID for Setup 2 (defaults to `"setup_2_flip"`).
    pub setup2_id: Option<String>,
    /// Controller work offset for Setup 2 (defaults to `G55`).
    pub setup2_wcs_slot: WorkOffsetSlot,
    /// Optional explicit ID for the optional spoilboard setup (defaults to `"setup_spoilboard_pins"`).
    pub spoilboard_setup_id: Option<String>,
    /// Controller work offset for the spoilboard setup (defaults to `G54`).
    pub spoilboard_wcs_slot: WorkOffsetSlot,
}

impl Default for FlipJigConfig {
    fn default() -> Self {
        Self {
            flip_axis: FlipAxis::XAxis,
            pin_diameter_mm: 6.0,
            pin_fit_clearance_mm: 0.02,
            pin_count: FlipJigPinCount::TwoOnAxis,
            poka_yoke_offset_mm: 0.0,
            stock_pin_depth_mm: 0.0,
            spoilboard_pin_depth_mm: 10.0,
            edge_margin_mm: 6.0,
            expand_stock_if_needed: true,
            include_spoilboard_setup: true,
            pin_protrusion_z_mm: 6.0,
            pin_tool_id: None,
            pin_tool_diameter_mm: None,
            setup2_id: None,
            setup2_wcs_slot: WorkOffsetSlot::G55,
            spoilboard_setup_id: None,
            spoilboard_wcs_slot: WorkOffsetSlot::G54,
        }
    }
}

/// Concrete ready-to-apply plan emitted by [`build_two_sided_flip_jig`].
#[derive(Debug, Clone)]
pub struct FlipJigPlan {
    pub flip_axis: FlipAxis,
    pub flip_centerline_mm: f64,
    pub hole_diameter_mm: f64,
    pub pin_centers_setup1_wcs: Vec<DVec2>,
    pub pin_centers_setup2_wcs: Vec<DVec2>,
    /// Maximum deviation (`mm`) between `mirror(pin_set)` and `pin_set` (`< 1e-9 mm`).
    pub max_symmetry_error_mm: f64,
    /// Minimum clear distance (`mm`) from any pin rim to the target part 2D silhouette.
    pub min_part_clearance_mm: f64,
    /// Updated `StockConfig` for Setup 1 whenever stock margins were expanded to fit the pins.
    pub updated_setup1_stock: Option<StockConfig>,
    /// `"Flip-Jig Registration Pins"` sketch anchored to Setup 1 XY with exact `Circle` entities.
    pub pin_sketch: SketchLayer,
    /// Op-1 stock dowel-pin drilling / helical-boring operation referencing `pin_sketch`.
    pub setup1_pin_op: Operation,
    /// Paired `180°`-flipped Setup 2 (`G55`) with `carryover_from_setup_id` and `CylinderKeepOut` pins.
    pub setup2_flip: Setup,
    /// Optional `"Spoilboard Dowel Holes"` Setup and companion sketch (when `include_spoilboard_setup`).
    pub spoilboard_setup: Option<Setup>,
    pub spoilboard_sketch: Option<SketchLayer>,
}

/// Mounting bolt pattern presets for Auto Soft Jaws.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SoftJawBoltPattern {
    None,
    /// Standard Kurt 6" vise (`3.875"` / `98.425 mm` spacing, 1/2"-13 SHCS).
    Kurt6Inch,
    /// Standard 4" benchtop/Milo vise (`2.500"` / `63.5 mm` spacing, M8 SHCS).
    #[default]
    Milo4Inch,
    /// Carvera modular vise (`50.0 mm` spacing, M5 SHCS).
    CarveraModVise,
    /// Custom 2-bolt pattern.
    Custom {
        spacing_mm: f64,
        bolt_dia_mm: f64,
        cbore_dia_mm: f64,
        cbore_depth_mm: f64,
    },
}

impl SoftJawBoltPattern {
    pub const fn label(self) -> &'static str {
        match self {
            Self::None => "None (Blank Jaws)",
            Self::Kurt6Inch => "Kurt 6\" Vise (98.425 mm · 1/2\")",
            Self::Milo4Inch => "4\" Mill / Milo Vise (63.5 mm · M8)",
            Self::CarveraModVise => "Carvera Mod Vise (50.0 mm · M5)",
            Self::Custom { .. } => "Custom Bolt Pattern",
        }
    }

    /// Returns `(spacing_mm, bolt_dia_mm, cbore_dia_mm, cbore_depth_mm, height_from_bottom_mm)`.
    pub fn dimensions_mm(self) -> Option<(f64, f64, f64, f64, f64)> {
        match self {
            Self::None => None,
            Self::Kurt6Inch => Some((98.425, 13.5, 20.5, 13.0, 23.8125)),
            Self::Milo4Inch => Some((63.5, 8.5, 14.0, 8.5, 16.0)),
            Self::CarveraModVise => Some((50.0, 5.5, 10.0, 6.0, 12.0)),
            Self::Custom {
                spacing_mm,
                bolt_dia_mm,
                cbore_dia_mm,
                cbore_depth_mm,
            } => Some((
                spacing_mm,
                bolt_dia_mm,
                cbore_dia_mm,
                cbore_depth_mm,
                0.0, // centered in lower solid
            )),
        }
    }
}

/// Output artifact mode for [`build_soft_jaw_plan`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SoftJawOutputMode {
    MachiningSetup,
    PrintableStl,
    #[default]
    Both,
}

impl SoftJawOutputMode {
    pub const ALL: [Self; 3] = [Self::Both, Self::MachiningSetup, Self::PrintableStl];

    pub const fn label(self) -> &'static str {
        match self {
            Self::MachiningSetup => "CNC Machining Setup (Aluminum Jaws)",
            Self::PrintableStl => "3D-Printable STL Files Only",
            Self::Both => "Both CNC Setup & Printable STLs",
        }
    }
}

/// Configuration for [`build_soft_jaw_plan`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SoftJawConfig {
    pub clamp_axis: ViseClampAxis,
    /// Width of each soft jaw plate along the non-clamping axis (`mm`).
    pub jaw_width_mm: f64,
    /// Depth / thickness of each soft jaw plate along the clamping axis (`mm`).
    pub jaw_depth_mm: f64,
    /// Total vertical height of each soft jaw blank (`mm`).
    pub jaw_height_mm: f64,
    /// Closed spacer gap between fixed & movable jaws during machining/clamping (`mm`).
    pub jaw_gap_mm: f64,
    /// Vertical pocket step depth cut into the top of the soft jaws (`mm`).
    pub grip_depth_mm: f64,
    /// Radial slip-fit clearance added outward around the part silhouette (`mm`).
    pub part_clearance_mm: f64,
    /// Dogbone / corner relief radius at sharp convex part corners (`mm`, `0` disables).
    pub corner_relief_radius_mm: f64,
    pub mounting_bolt_pattern: SoftJawBoltPattern,
    pub output_mode: SoftJawOutputMode,
    /// Optional offset upward from `model_aabb.min.z` where clamping Z band starts (`mm`).
    pub z_band_bottom_offset_mm: f64,
    /// Optional tool ID for Facing & Pocket2D roughing on the generated CNC setup.
    pub roughing_tool_id: Option<ToolId>,
    /// Optional tool ID for Contour2D wall finishing on the generated CNC setup.
    pub finishing_tool_id: Option<ToolId>,
    /// Optional explicit ID for the generated soft-jaw Setup.
    pub setup_id: Option<String>,
    /// Work offset slot for the generated soft-jaw Setup.
    pub wcs_slot: WorkOffsetSlot,
}

impl Default for SoftJawConfig {
    fn default() -> Self {
        Self {
            clamp_axis: ViseClampAxis::Y,
            jaw_width_mm: 100.0,
            jaw_depth_mm: 25.0,
            jaw_height_mm: 38.0,
            jaw_gap_mm: 6.0,
            grip_depth_mm: 4.0,
            part_clearance_mm: 0.05,
            corner_relief_radius_mm: 1.5,
            mounting_bolt_pattern: SoftJawBoltPattern::Milo4Inch,
            output_mode: SoftJawOutputMode::Both,
            z_band_bottom_offset_mm: 0.0,
            roughing_tool_id: None,
            finishing_tool_id: None,
            setup_id: None,
            wcs_slot: WorkOffsetSlot::G56,
        }
    }
}

/// Concrete ready-to-apply / export plan emitted by [`build_soft_jaw_plan`].
#[derive(Debug, Clone)]
pub struct SoftJawPlan {
    pub part_silhouette_2d: RegionSet2D,
    pub fixed_pocket_2d: RegionSet2D,
    pub movable_pocket_2d: RegionSet2D,
    pub jaw_width_mm: f64,
    pub jaw_depth_mm: f64,
    pub jaw_height_mm: f64,
    pub grip_depth_mm: f64,
    pub part_clearance_mm: f64,
    pub corner_relief_count: usize,
    pub machining_setup: Option<Setup>,
    pub pocket_sketch: Option<SketchLayer>,
    pub fixed_jaw_stl: Option<Vec<u8>>,
    pub movable_jaw_stl: Option<Vec<u8>>,
    pub fixed_jaw_triangle_count: usize,
    pub movable_jaw_triangle_count: usize,
}

pub(super) fn region_set_aabb(set: &RegionSet2D) -> Option<Aabb2> {
    set.exact_aabb()
}

pub(super) fn validate_positive_wiz(name: &str, field: &str, v: f64) -> Result<f64, CamError> {
    if !v.is_finite() || v <= 1e-6 {
        return Err(CamError::InvalidFixture {
            fixture_name: name.to_string(),
            reason: format!("'{field}' must be finite and > 0 (got {v})"),
        });
    }
    Ok(v)
}

pub(super) fn validate_non_negative_wiz(name: &str, field: &str, v: f64) -> Result<f64, CamError> {
    if !v.is_finite() || v < 0.0 {
        return Err(CamError::InvalidFixture {
            fixture_name: name.to_string(),
            reason: format!("'{field}' must be finite and >= 0 (got {v})"),
        });
    }
    Ok(v)
}

pub(super) fn validate_finite_wiz(name: &str, field: &str, v: f64) -> Result<f64, CamError> {
    if !v.is_finite() {
        return Err(CamError::InvalidFixture {
            fixture_name: name.to_string(),
            reason: format!("'{field}' must be finite (got {v})"),
        });
    }
    Ok(v)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::{
        CadKernel, MeshImportOptions, MonstertruckKernel, load_mesh_bytes,
        sample_multi_feature_bracket_step, sample_sculpted_dome_stl_bytes,
    };
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit};

    use super::*;

    #[test]
    fn flip_jig_wizard_guarantees_180_degree_symmetry_and_part_clearance() {
        let step = sample_multi_feature_bracket_step(false).unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("bracket.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let setup1 = Setup::new("setup_1", "Setup 1 - Top", WorkOffsetSlot::G54);

        for axis in FlipAxis::ALL {
            for count in FlipJigPinCount::ALL {
                for poka in [0.0, 5.0] {
                    let cfg = FlipJigConfig {
                        flip_axis: axis,
                        pin_diameter_mm: 6.0,
                        pin_fit_clearance_mm: 0.02,
                        pin_count: count,
                        poka_yoke_offset_mm: poka,
                        edge_margin_mm: 6.0,
                        ..FlipJigConfig::default()
                    };
                    let plan =
                        build_two_sided_flip_jig(&doc, &setup1, &[], DisplayUnit::Millimeter, &cfg)
                            .unwrap();
                    assert_eq!(plan.pin_centers_setup1_wcs.len(), count.count());
                    assert_eq!(plan.pin_centers_setup2_wcs.len(), count.count());
                    assert!(
                        plan.max_symmetry_error_mm < 1e-9,
                        "axis={axis:?} count={count:?} poka={poka} sym_err={}",
                        plan.max_symmetry_error_mm
                    );
                    assert!(
                        plan.min_part_clearance_mm >= 6.0 - 1e-4,
                        "axis={axis:?} count={count:?} clearance={}",
                        plan.min_part_clearance_mm
                    );
                    assert_eq!(
                        plan.setup2_flip.carryover_from_setup_id.as_deref(),
                        Some("setup_1")
                    );
                    assert_eq!(plan.setup2_flip.fixtures.len(), count.count());
                    assert_eq!(plan.pin_sketch.entities.len(), count.count());
                    assert!(plan.spoilboard_setup.is_some());
                }
            }
        }
    }

    #[test]
    fn soft_jaw_wizard_emits_watertight_stls_for_step_bracket_and_organic_stl_dome() {
        // 1. Prismatic STEP bracket (90x70x20 mm) with Kurt 6" counterbored bolt pattern
        let step = sample_multi_feature_bracket_step(false).unwrap();
        let bracket_doc = MonstertruckKernel
            .load_step_bytes("bracket.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let setup1 = Setup::new("setup_1", "Op 1", WorkOffsetSlot::G54);
        let cfg_bracket = SoftJawConfig {
            jaw_width_mm: 152.4,
            jaw_depth_mm: 40.0,
            jaw_height_mm: 45.0,
            jaw_gap_mm: 6.0,
            grip_depth_mm: 4.0,
            part_clearance_mm: 0.1,
            corner_relief_radius_mm: 2.0,
            mounting_bolt_pattern: SoftJawBoltPattern::Kurt6Inch,
            output_mode: SoftJawOutputMode::Both,
            ..SoftJawConfig::default()
        };
        let plan_b = build_soft_jaw_plan(
            &bracket_doc,
            &setup1,
            &[],
            DisplayUnit::Millimeter,
            &cfg_bracket,
        )
        .unwrap();
        assert!(plan_b.corner_relief_count >= 4);
        assert!(plan_b.machining_setup.is_some());
        assert!(plan_b.pocket_sketch.is_some());

        for (label, bytes) in [
            ("fixed_jaw.stl", plan_b.fixed_jaw_stl.as_ref().unwrap()),
            ("movable_jaw.stl", plan_b.movable_jaw_stl.as_ref().unwrap()),
        ] {
            let reloaded = load_mesh_bytes(label, bytes, &MeshImportOptions::default()).unwrap();
            assert!(
                reloaded.diagnostics.is_watertight,
                "{label} must be watertight: {:?}",
                reloaded.diagnostics
            );
            assert_eq!(reloaded.diagnostics.open_boundary_edges, 0);
            assert_eq!(reloaded.diagnostics.non_manifold_edges, 0);
            assert!(reloaded.diagnostics.signed_volume_mm3 > 0.0);
            let sz = reloaded.aabb_mm.size();
            assert!((sz.x - 152.4).abs() < 0.02);
            assert!((sz.y - 40.0).abs() < 0.02);
            assert!((sz.z - 45.0).abs() < 0.02);
        }

        // 2. Organic STL mesh (70x70x33 mm Sculpted Dome) with Milo 4" bolt pattern
        let dome_bytes = sample_sculpted_dome_stl_bytes();
        let dome_doc = chip_cad::load_mesh_cad_document(
            "sculpted_dome.stl",
            &dome_bytes,
            &MeshImportOptions::default(),
        )
        .unwrap();
        let cfg_dome = SoftJawConfig {
            jaw_width_mm: 100.0,
            jaw_depth_mm: 40.0,
            jaw_height_mm: 38.0,
            jaw_gap_mm: 6.0,
            grip_depth_mm: 4.5,
            part_clearance_mm: 0.15,
            corner_relief_radius_mm: 1.5,
            mounting_bolt_pattern: SoftJawBoltPattern::Milo4Inch,
            output_mode: SoftJawOutputMode::PrintableStl,
            ..SoftJawConfig::default()
        };
        let plan_d =
            build_soft_jaw_plan(&dome_doc, &setup1, &[], DisplayUnit::Millimeter, &cfg_dome)
                .unwrap();
        for (label, bytes) in [
            ("dome_fixed.stl", plan_d.fixed_jaw_stl.as_ref().unwrap()),
            ("dome_movable.stl", plan_d.movable_jaw_stl.as_ref().unwrap()),
        ] {
            let reloaded = load_mesh_bytes(label, bytes, &MeshImportOptions::default()).unwrap();
            assert!(
                reloaded.diagnostics.is_watertight,
                "{label} must be watertight: {:?}",
                reloaded.diagnostics
            );
            assert_eq!(reloaded.diagnostics.open_boundary_edges, 0);
            assert_eq!(reloaded.diagnostics.non_manifold_edges, 0);
            assert!(reloaded.diagnostics.signed_volume_mm3 > 0.0);
        }
    }
}
