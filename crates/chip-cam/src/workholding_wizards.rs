//! Shop-floor workholding automation wizards (`chip-cam`, Milestone 63):
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

use std::f64::consts::TAU;

use chip_cad::CadDocument;
use chip_geom2d::{PocketRegion2D, RegionSet2D, contours_to_region_set};
use chip_math::{
    Aabb2, Aabb3, ArcPolyline2D, ArcSegment2D, DVec2, DVec3, DisplayUnit, ParamLength,
};
use chip_sketch::{
    RectAnchorMode, SketchAnchor, SketchEntity, SketchEntityEntry, SketchLayer, SketchPlaneRef,
};
use chip_tools::ToolId;
use serde::{Deserialize, Serialize};

use crate::{
    cl_data::DrillCycleKind,
    heights::{HeightReference, HeightSpec},
    operation::{Operation, OperationKind},
    params::{ContourCompensation, HoleMachiningMode, PocketStrategyKind, StrategyParams},
    selection::GeometrySelectionItem,
    setup::{ResolvedSetup, Setup},
    stock::{FixedBoxXyAlign, FixedBoxZAlign, StockConfig},
    surface3d::{SurfaceMesh3D, rect_region_2d, slice_surface_contours_z, stock_xy_region},
    wcs::{BoxAnchor27, CamError, WcsOrientationMode, WcsOriginMode, WorkOffsetSlot},
    workholding::{FixtureBody, FixtureKind, ViseClampAxis},
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

/// Projects the target part's exact `f64` outer 2D silhouette [`RegionSet2D`] in Setup WCS,
/// optionally restricted to a vertical clamping Z band `[z_lo, z_hi]`.
pub fn extract_part_silhouette_2d(
    doc: &CadDocument,
    setup: &Setup,
    resolved: &ResolvedSetup,
    z_band: Option<(f64, f64)>,
) -> Result<RegionSet2D, CamError> {
    if !doc.has_3d_geometry() {
        let b = resolved.model_aabb_wcs;
        if b.is_valid() {
            return Ok(rect_region_2d(b.min.x, b.min.y, b.max.x, b.max.y));
        }
        return Ok(stock_xy_region(resolved));
    }

    let surface = SurfaceMesh3D::from_cad_document(doc, &setup.target_bodies, &resolved.frame);
    if surface.triangles.is_empty() || !surface.aabb_wcs.is_valid() {
        let b = resolved.model_aabb_wcs;
        if b.is_valid() {
            return Ok(rect_region_2d(b.min.x, b.min.y, b.max.x, b.max.y));
        }
        return Err(CamError::EmptyCadDocument);
    }

    let z_min_part = surface.aabb_wcs.min.z;
    let z_max_part = surface.aabb_wcs.max.z;
    let (z_lo, z_hi) = match z_band {
        Some((lo, hi)) => (
            lo.clamp(z_min_part, z_max_part),
            hi.clamp(z_min_part, z_max_part),
        ),
        None => (z_min_part, z_max_part),
    };
    let (z_lo, z_hi) = if z_hi > z_lo + 1e-4 {
        (z_lo, z_hi)
    } else {
        (z_min_part, z_max_part)
    };

    let span = (z_hi - z_lo).max(1e-3);
    let eps = (0.01 * span).clamp(0.005, 0.05);
    let mut z_samples: Vec<f64> = Vec::with_capacity(24);
    if z_hi - z_lo > 2.5 * eps {
        z_samples.push(z_lo + eps);
        z_samples.push(z_hi - eps);
    } else {
        z_samples.push(0.5 * (z_lo + z_hi));
    }
    const N_SAMPLES: usize = 9;
    for k in 1..N_SAMPLES {
        let frac = (k as f64) / (N_SAMPLES as f64);
        z_samples.push(z_lo + frac * span);
    }

    // Include Z heights just above/below horizontal B-Rep planes inside the band so stepped
    // flanges and ledges are captured to full precision.
    for face in &doc.faces {
        if let chip_cad::SurfaceKind::Plane { origin_mm, normal } = face.surface {
            let n_wcs = resolved.frame.world_to_local_dir(normal);
            if n_wcs.z.abs() > 0.99 {
                let z_f = resolved.frame.world_to_local_point(origin_mm).z;
                if z_f >= z_lo - 1e-3 && z_f <= z_hi + 1e-3 {
                    let z_above = z_f + eps;
                    let z_below = z_f - eps;
                    if z_above > z_lo && z_above < z_hi {
                        z_samples.push(z_above);
                    }
                    if z_below > z_lo && z_below < z_hi {
                        z_samples.push(z_below);
                    }
                }
            }
        }
    }

    z_samples.sort_by(f64::total_cmp);
    z_samples.dedup_by(|a, b| (*a - *b).abs() <= 0.02);

    let mut slice_sets: Vec<RegionSet2D> = Vec::with_capacity(z_samples.len());
    for z in z_samples {
        let contours = slice_surface_contours_z(&surface, z);
        let reg = contours_to_region_set(&contours);
        if reg.is_empty() {
            continue;
        }
        // Keep filled outer boundaries so interior mounting holes in the part never create
        // floating pins inside soft jaws or false-clearance pin cavities.
        let filled_hulls: Vec<PocketRegion2D> = reg
            .regions
            .into_iter()
            .map(|r| PocketRegion2D::new(r.outer_ccw, Vec::new()))
            .collect();
        let hull_set = RegionSet2D::from_regions(filled_hulls);
        if !hull_set.is_empty() {
            slice_sets.push(hull_set);
        }
    }

    let merged = RegionSet2D::union_many(&mut slice_sets);
    if !merged.is_empty() {
        Ok(merged)
    } else {
        Ok(rect_region_2d(
            surface.aabb_wcs.min.x,
            surface.aabb_wcs.min.y,
            surface.aabb_wcs.max.x,
            surface.aabb_wcs.max.y,
        ))
    }
}

/// Builds a complete **Two-Sided Flip-Jig Dowel-Pin** plan from `setup1` and `doc`.
pub fn build_two_sided_flip_jig(
    doc: &CadDocument,
    setup1: &Setup,
    sketches: &[SketchLayer],
    display_unit: DisplayUnit,
    config: &FlipJigConfig,
) -> Result<FlipJigPlan, CamError> {
    validate_positive_wiz("Flip-Jig Wizard", "pin_diameter_mm", config.pin_diameter_mm)?;
    validate_non_negative_wiz(
        "Flip-Jig Wizard",
        "pin_fit_clearance_mm",
        config.pin_fit_clearance_mm,
    )?;
    validate_non_negative_wiz("Flip-Jig Wizard", "edge_margin_mm", config.edge_margin_mm)?;
    validate_positive_wiz(
        "Flip-Jig Wizard",
        "spoilboard_pin_depth_mm",
        config.spoilboard_pin_depth_mm,
    )?;
    validate_finite_wiz(
        "Flip-Jig Wizard",
        "poka_yoke_offset_mm",
        config.poka_yoke_offset_mm,
    )?;
    validate_finite_wiz(
        "Flip-Jig Wizard",
        "stock_pin_depth_mm",
        config.stock_pin_depth_mm,
    )?;
    validate_non_negative_wiz(
        "Flip-Jig Wizard",
        "pin_protrusion_z_mm",
        config.pin_protrusion_z_mm,
    )?;

    let resolved1 = setup1.resolve_with_sketches(doc, sketches, display_unit)?;
    let part_sil = extract_part_silhouette_2d(doc, setup1, &resolved1, None)?;
    let part_aabb = region_set_aabb(&part_sil).unwrap_or_else(|| Aabb2 {
        min: DVec2::new(
            resolved1.model_aabb_wcs.min.x,
            resolved1.model_aabb_wcs.min.y,
        ),
        max: DVec2::new(
            resolved1.model_aabb_wcs.max.x,
            resolved1.model_aabb_wcs.max.y,
        ),
    });
    let stock1_aabb = resolved1.stock.aabb_wcs();
    let hole_diameter_mm = config.pin_diameter_mm + config.pin_fit_clearance_mm;
    let pin_radius = 0.5 * hole_diameter_mm;
    let req_center_dist = pin_radius + config.edge_margin_mm;

    // Flip centerline passes through the part's exact center perpendicular to the flip axis so
    // symmetric stock & flipped Setup 2 align cleanly with zero drift.
    let part_center = part_aabb.center();
    let v_flip = match config.flip_axis {
        FlipAxis::XAxis => part_center.y,
        FlipAxis::YAxis => part_center.x,
    };
    let (u_min_part, u_max_part, v_span_part) = match config.flip_axis {
        FlipAxis::XAxis => (part_aabb.min.x, part_aabb.max.x, part_aabb.size().y),
        FlipAxis::YAxis => (part_aabb.min.y, part_aabb.max.y, part_aabb.size().x),
    };

    let make_xy = |u: f64, v: f64| -> DVec2 {
        match config.flip_axis {
            FlipAxis::XAxis => DVec2::new(u, v),
            FlipAxis::YAxis => DVec2::new(v, u),
        }
    };

    // Place pins outside the part silhouette along `u` (applying optional poka-yoke outward shift
    // on one end while maintaining exact `180°` flip invariance around `v = v_flip`).
    let mut u_neg = u_min_part - req_center_dist - (-config.poka_yoke_offset_mm).max(0.0);
    let mut u_pos = u_max_part + req_center_dist + config.poka_yoke_offset_mm.max(0.0);

    let clear_of_part = |pt: DVec2| -> bool {
        !part_sil.contains_point(pt)
            && part_sil.min_distance_to_boundary(pt) >= req_center_dist - 1e-6
    };

    let pin_centers_setup1_wcs: Vec<DVec2> = match config.pin_count {
        FlipJigPinCount::TwoOnAxis => {
            for _ in 0..64 {
                if clear_of_part(make_xy(u_neg, v_flip)) {
                    break;
                }
                u_neg -= 0.5;
            }
            for _ in 0..64 {
                if clear_of_part(make_xy(u_pos, v_flip)) {
                    break;
                }
                u_pos += 0.5;
            }
            vec![make_xy(u_neg, v_flip), make_xy(u_pos, v_flip)]
        }
        FlipJigPinCount::FourSymmetric => {
            let dv = (0.35 * v_span_part).max(config.pin_diameter_mm * 1.5);
            for _ in 0..64 {
                if clear_of_part(make_xy(u_neg, v_flip - dv))
                    && clear_of_part(make_xy(u_neg, v_flip + dv))
                {
                    break;
                }
                u_neg -= 0.5;
            }
            for _ in 0..64 {
                if clear_of_part(make_xy(u_pos, v_flip - dv))
                    && clear_of_part(make_xy(u_pos, v_flip + dv))
                {
                    break;
                }
                u_pos += 0.5;
            }
            vec![
                make_xy(u_neg, v_flip - dv),
                make_xy(u_neg, v_flip + dv),
                make_xy(u_pos, v_flip - dv),
                make_xy(u_pos, v_flip + dv),
            ]
        }
    };

    // Verify exact 180° flip symmetry: `mirror(pin_set) == pin_set` within `< 1e-9 mm`
    let max_symmetry_error_mm =
        compute_flip_symmetry_error(&pin_centers_setup1_wcs, config.flip_axis, v_flip);
    if max_symmetry_error_mm > 1e-9 {
        return Err(CamError::InvalidFixture {
            fixture_name: "Flip-Jig Registration Pins".to_string(),
            reason: format!(
                "pin layout symmetry error {max_symmetry_error_mm:.3e} mm exceeded 1e-9 mm"
            ),
        });
    }

    let min_part_clearance_mm = pin_centers_setup1_wcs
        .iter()
        .map(|&p| part_sil.min_distance_to_boundary(p) - pin_radius)
        .fold(f64::INFINITY, f64::min);

    // Ensure raw stock encloses all pins with at least `pin_radius + 4.0 mm` outer wall margin
    let wall_margin = pin_radius + 4.0;
    let mut pins_aabb = Aabb2::EMPTY;
    for &p in &pin_centers_setup1_wcs {
        pins_aabb.extend_point(p - DVec2::splat(wall_margin));
        pins_aabb.extend_point(p + DVec2::splat(wall_margin));
    }

    let needs_expand = pins_aabb.min.x < stock1_aabb.min.x - 1e-4
        || pins_aabb.max.x > stock1_aabb.max.x + 1e-4
        || pins_aabb.min.y < stock1_aabb.min.y - 1e-4
        || pins_aabb.max.y > stock1_aabb.max.y + 1e-4;

    let mut effective_setup1 = setup1.clone();
    let updated_setup1_stock = if needs_expand {
        if !config.expand_stock_if_needed {
            return Err(CamError::InvalidFixture {
                fixture_name: "Flip-Jig Registration Pins".to_string(),
                reason: "dowel pins fall outside current stock footprint; enable stock expansion or widen stock margins".to_string(),
            });
        }
        let expanded = expand_stock_for_pins(
            &setup1.stock,
            &resolved1.model_aabb_wcs,
            &stock1_aabb,
            &pins_aabb,
            config.flip_axis,
        );
        effective_setup1.stock = expanded.clone();
        Some(expanded)
    } else {
        None
    };

    // Re-resolve Setup 1 with expanded stock so pin coordinates in Setup 1 WCS stay exact even if
    // Setup 1's WCS origin is anchored to a StockBoxPoint corner!
    let resolved1_eff = effective_setup1.resolve_with_sketches(doc, sketches, display_unit)?;
    let origin_shift_xy = {
        let p_world_ref = resolved1.frame.local_to_world_point(DVec3::new(
            0.0,
            0.0,
            resolved1.model_aabb_wcs.min.z,
        ));
        let p_in_new = resolved1_eff.frame.world_to_local_point(p_world_ref);
        DVec2::new(p_in_new.x, p_in_new.y)
    };
    let pin_centers_s1_final: Vec<DVec2> = pin_centers_setup1_wcs
        .iter()
        .map(|&p| p + origin_shift_xy)
        .collect();
    let v_flip_final = v_flip
        + match config.flip_axis {
            FlipAxis::XAxis => origin_shift_xy.y,
            FlipAxis::YAxis => origin_shift_xy.x,
        };
    let final_sym_err =
        compute_flip_symmetry_error(&pin_centers_s1_final, config.flip_axis, v_flip_final);

    // 1. Build `"Flip-Jig Registration Pins"` SketchLayer on Setup 1 XY
    let sketch_id = format!("sketch_flip_jig_pins_{}", setup1.id);
    let mut pin_sketch = SketchLayer::new(
        sketch_id.clone(),
        "Flip-Jig Registration Pins",
        setup1.id.clone(),
    );
    pin_sketch.anchor = SketchAnchor::SetupXy {
        setup_id: setup1.id.clone(),
        plane_ref: SketchPlaneRef::StockTop,
        z_offset: ParamLength::from_mm(0.0),
    };
    let mut pin_selections = Vec::with_capacity(pin_centers_s1_final.len());
    for (idx, &pt) in pin_centers_s1_final.iter().enumerate() {
        let ent_id = format!("pin_{}", idx + 1);
        pin_sketch.entities.push(SketchEntityEntry {
            id: ent_id.clone(),
            name: format!("Dowel Pin {} (Ø{:.2} mm)", idx + 1, hole_diameter_mm),
            sub_layer: Some("PINS".to_string()),
            is_construction: false,
            entity: SketchEntity::new_circle(
                ParamLength::from_mm(pt.x),
                ParamLength::from_mm(pt.y),
                ParamLength::from_mm(hole_diameter_mm),
            ),
        });
        pin_selections.push(GeometrySelectionItem::SketchEntity {
            sketch_id: sketch_id.clone(),
            entity_id: ent_id,
            reverse: false,
            side_flip: false,
        });
    }
    pin_sketch.rebuild_regions(None);

    // 2. Build Op-1 Stock Dowel-Pin Drilling / Helical Boring operation
    let mut setup1_pin_op = Operation::new(
        "op_flip_jig_pins",
        "Flip-Jig Dowel Pin Holes",
        OperationKind::DrillBore,
    );
    setup1_pin_op.tool_id = config.pin_tool_id.clone();
    setup1_pin_op.selections = pin_selections;
    setup1_pin_op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    setup1_pin_op.heights.bottom = if config.stock_pin_depth_mm > 1e-4 {
        HeightSpec::new(HeightReference::StockTop, -config.stock_pin_depth_mm)
    } else {
        HeightSpec::new(HeightReference::StockBottom, 0.0)
    };
    configure_pin_drill_params(
        &mut setup1_pin_op,
        hole_diameter_mm,
        config.pin_tool_diameter_mm,
    );

    // 3. Build paired `Setup 2 — Flip Op (G55)` rotated 180° around `flip_axis` with stock carryover
    let setup2_id = config
        .setup2_id
        .clone()
        .unwrap_or_else(|| "setup_2_flip".to_string());
    let mut setup2_flip = Setup::new(
        setup2_id.clone(),
        format!(
            "Setup 2 — Flip Op ({})",
            config.setup2_wcs_slot.short_code()
        ),
        config.setup2_wcs_slot,
    );
    setup2_flip.target_bodies = setup1.target_bodies.clone();
    setup2_flip.post_profile = setup1.post_profile.clone();
    setup2_flip.heights = setup1.heights.clone();
    setup2_flip.auto_avoid_fixtures_on_rapids = true;
    setup2_flip.carryover_from_setup_id = Some(setup1.id.clone());
    setup2_flip.wcs.orientation = flip_wcs_orientation(&setup1.wcs.orientation, config.flip_axis);
    setup2_flip.wcs.origin = mirror_wcs_origin(&setup1.wcs.origin, config.flip_axis);
    setup2_flip.stock = mirror_stock_for_flip(&effective_setup1.stock, config.flip_axis);

    let resolved2 = setup2_flip.resolve_with_sketches(doc, sketches, display_unit)?;
    let stock1_eff_aabb = resolved1_eff.stock.aabb_wcs();
    let stock2_aabb = resolved2.stock.aabb_wcs();

    // Transform each Setup-1 pin center rigidly into Setup-2 WCS so CylinderKeepOut bodies
    // align to `< 1e-9 mm` with the drilled stock holes after the 180° flip!
    let mut pin_centers_setup2_wcs = Vec::with_capacity(pin_centers_s1_final.len());
    let pin_z_bottom = stock2_aabb.min.z - config.spoilboard_pin_depth_mm;
    let pin_z_top = (stock2_aabb.min.z + config.pin_protrusion_z_mm.max(1.0))
        .min(stock2_aabb.max.z - 0.5)
        .max(stock2_aabb.min.z + 1.0);
    for (idx, &p1) in pin_centers_s1_final.iter().enumerate() {
        let p_world =
            resolved1_eff
                .frame
                .local_to_world_point(DVec3::new(p1.x, p1.y, stock1_eff_aabb.min.z));
        let p2_3d = resolved2.frame.world_to_local_point(p_world);
        let p2 = DVec2::new(p2_3d.x, p2_3d.y);
        pin_centers_setup2_wcs.push(p2);

        let mut pin_fix = FixtureBody::default_cylinder_keepout(
            format!("flip_pin_{}", idx + 1),
            format!("Flip-Jig Dowel Pin {}", idx + 1),
            p2,
            pin_z_bottom,
            pin_z_top,
        );
        pin_fix.clearance_margin = ParamLength::from_mm(1.0);
        if let FixtureKind::CylinderKeepOut { diameter, .. } = &mut pin_fix.kind {
            *diameter = ParamLength::from_mm(config.pin_diameter_mm);
        }
        setup2_flip.fixtures.push(pin_fix);
    }

    // 4. Optional `"Spoilboard Dowel Holes"` Setup + companion sketch/operation
    let (spoilboard_setup, spoilboard_sketch) = if config.include_spoilboard_setup {
        let sb_id = config
            .spoilboard_setup_id
            .clone()
            .unwrap_or_else(|| "setup_spoilboard_pins".to_string());
        let mut sb_setup = Setup::new(
            sb_id.clone(),
            "Spoilboard Dowel Holes",
            config.spoilboard_wcs_slot,
        );
        sb_setup.target_bodies = setup1.target_bodies.clone();
        sb_setup.post_profile = setup1.post_profile.clone();
        sb_setup.wcs = setup1.wcs.clone();
        sb_setup.stock = effective_setup1.stock.clone();
        sb_setup.heights = setup1.heights.clone();

        let sb_sketch_id = format!("sketch_{sb_id}");
        let mut sb_sketch = SketchLayer::new(
            sb_sketch_id.clone(),
            "Spoilboard Dowel Holes",
            sb_id.clone(),
        );
        sb_sketch.anchor = SketchAnchor::SetupXy {
            setup_id: sb_id,
            plane_ref: SketchPlaneRef::StockTop,
            z_offset: ParamLength::from_mm(0.0),
        };
        let mut sb_sels = Vec::with_capacity(pin_centers_s1_final.len());
        for (idx, &pt) in pin_centers_s1_final.iter().enumerate() {
            let ent_id = format!("sb_pin_{}", idx + 1);
            sb_sketch.entities.push(SketchEntityEntry {
                id: ent_id.clone(),
                name: format!("Spoilboard Pin {} (Ø{:.2} mm)", idx + 1, hole_diameter_mm),
                sub_layer: Some("SPOILBOARD_PINS".to_string()),
                is_construction: false,
                entity: SketchEntity::new_circle(
                    ParamLength::from_mm(pt.x),
                    ParamLength::from_mm(pt.y),
                    ParamLength::from_mm(hole_diameter_mm),
                ),
            });
            sb_sels.push(GeometrySelectionItem::SketchEntity {
                sketch_id: sb_sketch_id.clone(),
                entity_id: ent_id,
                reverse: false,
                side_flip: false,
            });
        }
        sb_sketch.rebuild_regions(None);

        let mut sb_op = Operation::new(
            "op_spoilboard_dowel_holes",
            "Drill/Bore Spoilboard Dowel Holes",
            OperationKind::DrillBore,
        );
        sb_op.tool_id = config.pin_tool_id.clone();
        sb_op.selections = sb_sels;
        sb_op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        sb_op.heights.bottom =
            HeightSpec::new(HeightReference::StockTop, -config.spoilboard_pin_depth_mm);
        configure_pin_drill_params(&mut sb_op, hole_diameter_mm, config.pin_tool_diameter_mm);
        sb_setup.operations.push(sb_op);
        (Some(sb_setup), Some(sb_sketch))
    } else {
        (None, None)
    };

    Ok(FlipJigPlan {
        flip_axis: config.flip_axis,
        flip_centerline_mm: v_flip_final,
        hole_diameter_mm,
        pin_centers_setup1_wcs: pin_centers_s1_final,
        pin_centers_setup2_wcs,
        max_symmetry_error_mm: final_sym_err,
        min_part_clearance_mm,
        updated_setup1_stock,
        pin_sketch,
        setup1_pin_op,
        setup2_flip,
        spoilboard_setup,
        spoilboard_sketch,
    })
}

/// Builds a complete **Auto Soft-Jaw** plan (CNC machining setup + sketch + ops and/or watertight
/// 3D-printable binary `.stl` files) from `setup` and `doc`.
pub fn build_soft_jaw_plan(
    doc: &CadDocument,
    setup: &Setup,
    sketches: &[SketchLayer],
    display_unit: DisplayUnit,
    config: &SoftJawConfig,
) -> Result<SoftJawPlan, CamError> {
    validate_positive_wiz("Soft-Jaw Wizard", "jaw_width_mm", config.jaw_width_mm)?;
    validate_positive_wiz("Soft-Jaw Wizard", "jaw_depth_mm", config.jaw_depth_mm)?;
    validate_positive_wiz("Soft-Jaw Wizard", "jaw_height_mm", config.jaw_height_mm)?;
    validate_non_negative_wiz("Soft-Jaw Wizard", "jaw_gap_mm", config.jaw_gap_mm)?;
    validate_positive_wiz("Soft-Jaw Wizard", "grip_depth_mm", config.grip_depth_mm)?;
    validate_non_negative_wiz(
        "Soft-Jaw Wizard",
        "part_clearance_mm",
        config.part_clearance_mm,
    )?;
    validate_non_negative_wiz(
        "Soft-Jaw Wizard",
        "corner_relief_radius_mm",
        config.corner_relief_radius_mm,
    )?;
    if config.grip_depth_mm >= config.jaw_height_mm - 2.0 {
        return Err(CamError::InvalidFixture {
            fixture_name: "Soft-Jaw Wizard".to_string(),
            reason: format!(
                "grip_depth_mm ({:.2}) must leave at least 2.0 mm base below pocket (jaw_height_mm = {:.2})",
                config.grip_depth_mm, config.jaw_height_mm
            ),
        });
    }

    let resolved = setup.resolve_with_sketches(doc, sketches, display_unit)?;
    let z_bot = resolved.model_aabb_wcs.min.z + config.z_band_bottom_offset_mm;
    let z_top = z_bot + config.grip_depth_mm;
    let part_sil = extract_part_silhouette_2d(doc, setup, &resolved, Some((z_bot, z_top)))?;
    let part_aabb = region_set_aabb(&part_sil).ok_or_else(|| CamError::InvalidFixture {
        fixture_name: "Soft-Jaw Wizard".to_string(),
        reason: "could not extract non-empty 2D clamping silhouette from target part".to_string(),
    })?;
    let part_center = part_aabb.center();

    // 1. Apply radial slip-fit clearance outward around the clamping silhouette
    let cleared_part = if config.part_clearance_mm > 1e-6 {
        part_sil.dilate_outward(config.part_clearance_mm)
    } else {
        part_sil.clone()
    };

    // 2. Add internal sharp-corner dogbone reliefs detected on the pre-dilation part silhouette
    //    (before `dilate_outward` rounds sharp vertices with tiny `part_clearance_mm` corner arcs!)
    let (pocket_with_relief, corner_relief_count) = add_convex_corner_reliefs(
        &part_sil,
        &cleared_part,
        config.part_clearance_mm,
        config.corner_relief_radius_mm,
    );

    // 3. Intersect against Fixed Jaw & Movable Jaw rectangles separated by `jaw_gap_mm`
    let half_w = 0.5 * config.jaw_width_mm;
    let half_gap = 0.5 * config.jaw_gap_mm;
    let depth = config.jaw_depth_mm;

    let (fixed_rect, movable_rect) = match config.clamp_axis {
        ViseClampAxis::Y => (
            rect_region_2d(
                part_center.x - half_w,
                part_center.y + half_gap,
                part_center.x + half_w,
                part_center.y + half_gap + depth,
            ),
            rect_region_2d(
                part_center.x - half_w,
                part_center.y - half_gap - depth,
                part_center.x + half_w,
                part_center.y - half_gap,
            ),
        ),
        ViseClampAxis::X => (
            rect_region_2d(
                part_center.x + half_gap,
                part_center.y - half_w,
                part_center.x + half_gap + depth,
                part_center.y + half_w,
            ),
            rect_region_2d(
                part_center.x - half_gap - depth,
                part_center.y - half_w,
                part_center.x - half_gap,
                part_center.y + half_w,
            ),
        ),
    };

    let fixed_pocket_wcs = pocket_with_relief.intersect(&fixed_rect);
    let movable_pocket_wcs = pocket_with_relief.intersect(&movable_rect);

    if fixed_pocket_wcs.is_empty() && movable_pocket_wcs.is_empty() {
        return Err(CamError::InvalidFixture {
            fixture_name: "Soft-Jaw Wizard".to_string(),
            reason: "part clamping silhouette does not overlap either soft jaw; reduce jaw_gap_mm or increase jaw_depth_mm".to_string(),
        });
    }

    // 4. Optional CNC-Machinable Soft-Jaw Setup + Sketch + Facing / Pocket2D / Contour2D ops
    let (machining_setup, pocket_sketch) = if matches!(
        config.output_mode,
        SoftJawOutputMode::MachiningSetup | SoftJawOutputMode::Both
    ) {
        let (sj_setup, sj_sketch) = build_soft_jaw_machining_setup(
            setup,
            &resolved,
            part_center,
            &fixed_pocket_wcs,
            &movable_pocket_wcs,
            config,
        );
        (Some(sj_setup), Some(sj_sketch))
    } else {
        (None, None)
    };

    // 5. Optional Watertight 3D-Printable Binary STL Buffers (`fixed_jaw.stl` & `movable_jaw.stl`)
    let (fixed_jaw_stl, fixed_jaw_triangle_count, movable_jaw_stl, movable_jaw_triangle_count) = if matches!(
        config.output_mode,
        SoftJawOutputMode::PrintableStl | SoftJawOutputMode::Both
    ) {
        let fixed_local = map_region_to_jaw_local(
            &fixed_pocket_wcs,
            part_center,
            config.clamp_axis,
            true,
            config.jaw_width_mm,
            config.jaw_gap_mm,
        );
        let movable_local = map_region_to_jaw_local(
            &movable_pocket_wcs,
            part_center,
            config.clamp_axis,
            false,
            config.jaw_width_mm,
            config.jaw_gap_mm,
        );

        let (fixed_bytes, fixed_tris) = build_watertight_soft_jaw_stl_bytes(
            "ChipCutter Fixed Soft Jaw",
            config.jaw_width_mm,
            config.jaw_depth_mm,
            config.jaw_height_mm,
            config.grip_depth_mm,
            &fixed_local,
            config.mounting_bolt_pattern,
        )?;
        let (movable_bytes, movable_tris) = build_watertight_soft_jaw_stl_bytes(
            "ChipCutter Movable Soft Jaw",
            config.jaw_width_mm,
            config.jaw_depth_mm,
            config.jaw_height_mm,
            config.grip_depth_mm,
            &movable_local,
            config.mounting_bolt_pattern,
        )?;
        (
            Some(fixed_bytes),
            fixed_tris,
            Some(movable_bytes),
            movable_tris,
        )
    } else {
        (None, 0, None, 0)
    };

    Ok(SoftJawPlan {
        part_silhouette_2d: part_sil,
        fixed_pocket_2d: fixed_pocket_wcs,
        movable_pocket_2d: movable_pocket_wcs,
        jaw_width_mm: config.jaw_width_mm,
        jaw_depth_mm: config.jaw_depth_mm,
        jaw_height_mm: config.jaw_height_mm,
        grip_depth_mm: config.grip_depth_mm,
        part_clearance_mm: config.part_clearance_mm,
        corner_relief_count,
        machining_setup,
        pocket_sketch,
        fixed_jaw_stl,
        movable_jaw_stl,
        fixed_jaw_triangle_count,
        movable_jaw_triangle_count,
    })
}

// ── Flip-Jig Helpers ───────────────────────────────────────────────────────────────────────────

fn configure_pin_drill_params(op: &mut Operation, hole_dia_mm: f64, tool_dia_mm: Option<f64>) {
    if let StrategyParams::DrillBore(p) = op.strategy_mut() {
        p.mode = match tool_dia_mm {
            Some(td) if td < hole_dia_mm - 0.15 => HoleMachiningMode::HelicalBore,
            Some(_) => HoleMachiningMode::Drill,
            None => HoleMachiningMode::AutoByTool,
        };
        p.cycle_kind = DrillCycleKind::DeepPeck;
        p.peck_depth = ParamLength::from_mm(3.0);
        p.bore_finish_pass = true;
        p.bore_finish_stepover = ParamLength::from_mm(0.1);
        p.tip_breakthrough = false;
    }
}

fn compute_flip_symmetry_error(pins: &[DVec2], axis: FlipAxis, v_flip: f64) -> f64 {
    let mut max_err = 0.0_f64;
    for &p in pins {
        let mirrored = match axis {
            FlipAxis::XAxis => DVec2::new(p.x, 2.0 * v_flip - p.y),
            FlipAxis::YAxis => DVec2::new(2.0 * v_flip - p.x, p.y),
        };
        let nearest = pins
            .iter()
            .map(|&q| q.distance(mirrored))
            .fold(f64::INFINITY, f64::min);
        if nearest > max_err {
            max_err = nearest;
        }
    }
    max_err
}

fn expand_stock_for_pins(
    stock: &StockConfig,
    model_aabb: &Aabb3,
    stock_aabb: &Aabb3,
    pins_aabb: &Aabb2,
    flip_axis: FlipAxis,
) -> StockConfig {
    let cur_neg_x = (model_aabb.min.x - stock_aabb.min.x).max(0.0);
    let cur_pos_x = (stock_aabb.max.x - model_aabb.max.x).max(0.0);
    let cur_neg_y = (model_aabb.min.y - stock_aabb.min.y).max(0.0);
    let cur_pos_y = (stock_aabb.max.y - model_aabb.max.y).max(0.0);
    let cur_top_z = (stock_aabb.max.z - model_aabb.max.z).max(0.0);
    let cur_bot_z = (model_aabb.min.z - stock_aabb.min.z).max(0.0);

    let mut neg_x = cur_neg_x.max(model_aabb.min.x - pins_aabb.min.x);
    let mut pos_x = cur_pos_x.max(pins_aabb.max.x - model_aabb.max.x);
    let mut neg_y = cur_neg_y.max(model_aabb.min.y - pins_aabb.min.y);
    let mut pos_y = cur_pos_y.max(pins_aabb.max.y - model_aabb.max.y);

    // Keep the axis perpendicular to `flip_axis` symmetric so the stock centerline remains
    // coincident with the part flip centerline.
    match flip_axis {
        FlipAxis::XAxis => {
            let sym_y = neg_y.max(pos_y);
            neg_y = sym_y;
            pos_y = sym_y;
        }
        FlipAxis::YAxis => {
            let sym_x = neg_x.max(pos_x);
            neg_x = sym_x;
            pos_x = sym_x;
        }
    }

    let symmetric_xy = (pos_x - neg_x).abs() <= 1e-6 && (pos_y - neg_y).abs() <= 1e-6;
    let sheet_z = match stock {
        StockConfig::RelativeBox {
            sheet_thickness_z, ..
        } => sheet_thickness_z.clone(),
        _ => ParamLength::from_mm(stock_aabb.size().z.max(1.0)),
    };

    StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(pos_x),
        side_neg_x: ParamLength::from_mm(neg_x),
        side_pos_y: ParamLength::from_mm(pos_y),
        side_neg_y: ParamLength::from_mm(neg_y),
        symmetric_xy,
        top_offset_z: ParamLength::from_mm(cur_top_z),
        bottom_offset_z: ParamLength::from_mm(cur_bot_z),
        sheet_thickness_z: sheet_z,
        round_increment_mm: None,
    }
}

fn flip_wcs_orientation(orig: &WcsOrientationMode, axis: FlipAxis) -> WcsOrientationMode {
    match orig {
        WcsOrientationMode::ModelOrientation {
            flip_z,
            flip_x,
            clock_90,
            extra_z_rot_deg,
        } => {
            let inv_clock = (4 - (*clock_90 % 4)) % 4;
            WcsOrientationMode::ModelOrientation {
                flip_z: !*flip_z,
                flip_x: match axis {
                    FlipAxis::XAxis => *flip_x,
                    FlipAxis::YAxis => !*flip_x,
                },
                clock_90: inv_clock,
                extra_z_rot_deg: -*extra_z_rot_deg,
            }
        }
        WcsOrientationMode::SelectZAndX {
            z_ref,
            flip_z,
            x_ref,
            flip_x,
            clock_90,
            extra_z_rot_deg,
        } => {
            let inv_clock = (4 - (*clock_90 % 4)) % 4;
            WcsOrientationMode::SelectZAndX {
                z_ref: z_ref.clone(),
                flip_z: !*flip_z,
                x_ref: x_ref.clone(),
                flip_x: match axis {
                    FlipAxis::XAxis => *flip_x,
                    FlipAxis::YAxis => !*flip_x,
                },
                clock_90: inv_clock,
                extra_z_rot_deg: -*extra_z_rot_deg,
            }
        }
        WcsOrientationMode::SelectXYPlane {
            face_ref,
            flip_z,
            x_edge_ref,
            clock_90,
            extra_z_rot_deg,
        } => {
            let base_clock = (4 - (*clock_90 % 4)) % 4;
            let inv_clock = match axis {
                FlipAxis::XAxis => base_clock,
                FlipAxis::YAxis => (base_clock + 2) % 4,
            };
            WcsOrientationMode::SelectXYPlane {
                face_ref: face_ref.clone(),
                flip_z: !*flip_z,
                x_edge_ref: x_edge_ref.clone(),
                clock_90: inv_clock,
                extra_z_rot_deg: -*extra_z_rot_deg,
            }
        }
    }
}

fn mirror_wcs_origin(origin: &WcsOriginMode, _axis: FlipAxis) -> WcsOriginMode {
    match origin {
        WcsOriginMode::StockBoxPoint { .. } => WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        },
        WcsOriginMode::ModelBoxPoint { .. } => WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        },
        WcsOriginMode::SelectedCadFeature { .. } => WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        },
    }
}

fn mirror_stock_for_flip(stock: &StockConfig, axis: FlipAxis) -> StockConfig {
    match stock {
        StockConfig::RelativeBox {
            side_pos_x,
            side_neg_x,
            side_pos_y,
            side_neg_y,
            symmetric_xy,
            top_offset_z,
            bottom_offset_z,
            sheet_thickness_z,
            round_increment_mm,
        } => {
            let (px, nx, py, ny) = match axis {
                FlipAxis::XAxis => (
                    side_pos_x.clone(),
                    side_neg_x.clone(),
                    side_neg_y.clone(),
                    side_pos_y.clone(),
                ),
                FlipAxis::YAxis => (
                    side_neg_x.clone(),
                    side_pos_x.clone(),
                    side_pos_y.clone(),
                    side_neg_y.clone(),
                ),
            };
            StockConfig::RelativeBox {
                side_pos_x: px,
                side_neg_x: nx,
                side_pos_y: py,
                side_neg_y: ny,
                symmetric_xy: *symmetric_xy,
                top_offset_z: bottom_offset_z.clone(),
                bottom_offset_z: top_offset_z.clone(),
                sheet_thickness_z: sheet_thickness_z.clone(),
                round_increment_mm: *round_increment_mm,
            }
        }
        StockConfig::FixedBox {
            width_x,
            depth_y,
            height_z,
            xy_align,
            z_align,
        } => {
            let mirrored_z = match z_align {
                FixedBoxZAlign::OffsetFromTop { top_allowance } => {
                    FixedBoxZAlign::OffsetFromBottom {
                        bottom_allowance: top_allowance.clone(),
                    }
                }
                FixedBoxZAlign::OffsetFromBottom { bottom_allowance } => {
                    FixedBoxZAlign::OffsetFromTop {
                        top_allowance: bottom_allowance.clone(),
                    }
                }
                FixedBoxZAlign::CenterZ => FixedBoxZAlign::CenterZ,
            };
            StockConfig::FixedBox {
                width_x: width_x.clone(),
                depth_y: depth_y.clone(),
                height_z: height_z.clone(),
                xy_align: xy_align.clone(),
                z_align: mirrored_z,
            }
        }
        other => other.clone(),
    }
}

// ── Soft-Jaw Helpers ───────────────────────────────────────────────────────────────────────────

/// Adds circular dogbone corner reliefs (`radius = relief_r`) at sharp convex corners detected on
/// `raw_part_sil` (before clearance dilation rounds sharp corners with tiny tangent arcs) so
/// square part corners seat dead-flat inside CNC-machined or 3D-printed soft jaws.
fn add_convex_corner_reliefs(
    raw_part_sil: &RegionSet2D,
    cleared_part: &RegionSet2D,
    clearance_mm: f64,
    relief_r: f64,
) -> (RegionSet2D, usize) {
    if !relief_r.is_finite() || relief_r <= 0.05 || raw_part_sil.is_empty() {
        return (cleared_part.clone(), 0);
    }

    let mut relief_disks: Vec<RegionSet2D> = Vec::new();
    let min_edge_len = (2.0 * relief_r).max(1.5);
    let eff_r = relief_r + clearance_mm.max(0.0);

    for reg in &raw_part_sil.regions {
        let segs = &reg.outer_ccw.segments;
        let n = segs.len();
        if n < 3 {
            continue;
        }
        for i in 0..n {
            let prev = &segs[(i + n - 1) % n];
            let curr = &segs[i];
            if prev.length() < min_edge_len || curr.length() < min_edge_len {
                continue;
            }
            let (Some((_, t_in)), Some((_, t_out))) = (prev.eval_at(1.0), curr.eval_at(0.0)) else {
                continue;
            };
            let cross = t_in.x * t_out.y - t_in.y * t_out.x;
            let dot = t_in.dot(t_out).clamp(-1.0, 1.0);
            let turn_deg = cross.atan2(dot).to_degrees();
            // Convex corner on CCW outer loop turns left (`turn_deg >= 25°`).
            if (25.0..=165.0).contains(&turn_deg) {
                let corner_pt = curr.start();
                let disk = RegionSet2D::from_circle(corner_pt, eff_r);
                if !disk.is_empty() {
                    relief_disks.push(disk);
                }
            }
        }
    }

    let count = relief_disks.len();
    if count == 0 {
        return (cleared_part.clone(), 0);
    }
    let merged_disks = RegionSet2D::union_many(&mut relief_disks);
    (cleared_part.union(&merged_disks), count)
}

fn build_soft_jaw_machining_setup(
    source_setup: &Setup,
    resolved: &ResolvedSetup,
    part_center: DVec2,
    fixed_pocket_wcs: &RegionSet2D,
    movable_pocket_wcs: &RegionSet2D,
    config: &SoftJawConfig,
) -> (Setup, SketchLayer) {
    let setup_id = config
        .setup_id
        .clone()
        .unwrap_or_else(|| "setup_soft_jaws".to_string());
    let mut sj_setup = Setup::new(
        setup_id.clone(),
        "Soft Jaws — Fixed & Movable",
        config.wcs_slot,
    );
    sj_setup.target_bodies = source_setup.target_bodies.clone();
    sj_setup.post_profile = source_setup.post_profile.clone();
    sj_setup.wcs.orientation = source_setup.wcs.orientation.clone();
    sj_setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };

    let (raw_span_x, raw_span_y) = match config.clamp_axis {
        ViseClampAxis::Y => (
            config.jaw_width_mm,
            2.0 * config.jaw_depth_mm + config.jaw_gap_mm,
        ),
        ViseClampAxis::X => (
            2.0 * config.jaw_depth_mm + config.jaw_gap_mm,
            config.jaw_width_mm,
        ),
    };

    // Ensure FixedBox stock encloses the project's 3D target model (if any) and positions the
    // model safely below the soft-jaw pocket cut depth so simulation never reports false gouges.
    let model_sz = if resolved.model_aabb_wcs.is_valid() {
        resolved.model_aabb_wcs.size()
    } else {
        DVec3::ZERO
    };
    let skim_mm = 0.5;
    let total_cut_depth = skim_mm + config.grip_depth_mm;
    let top_allow_above_model = total_cut_depth + 1.5;

    let stock_x = raw_span_x.max(model_sz.x + 2.0);
    let stock_y = raw_span_y.max(model_sz.y + 2.0);
    let stock_z = config
        .jaw_height_mm
        .max(model_sz.z + top_allow_above_model + 1.0);

    sj_setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(stock_x),
        depth_y: ParamLength::from_mm(stock_y),
        height_z: ParamLength::from_mm(stock_z),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::OffsetFromTop {
            top_allowance: ParamLength::from_mm(top_allow_above_model),
        },
    };

    let sketch_id = format!("sketch_{setup_id}");
    let mut sketch = SketchLayer::new(
        sketch_id.clone(),
        "Soft-Jaw Pocket Contours",
        setup_id.clone(),
    );
    sketch.anchor = SketchAnchor::SetupXy {
        setup_id,
        plane_ref: SketchPlaneRef::StockTop,
        z_offset: ParamLength::from_mm(0.0),
    };
    sketch.cut_depth_default = ParamLength::from_mm(config.grip_depth_mm);

    // Construction outlines for Fixed & Movable jaw blanks centered at `(0, 0)`
    let half_gap = 0.5 * config.jaw_gap_mm;
    let (fixed_cx, fixed_cy, mov_cx, mov_cy, rect_w, rect_h) = match config.clamp_axis {
        ViseClampAxis::Y => (
            0.0,
            half_gap + 0.5 * config.jaw_depth_mm,
            0.0,
            -(half_gap + 0.5 * config.jaw_depth_mm),
            config.jaw_width_mm,
            config.jaw_depth_mm,
        ),
        ViseClampAxis::X => (
            half_gap + 0.5 * config.jaw_depth_mm,
            0.0,
            -(half_gap + 0.5 * config.jaw_depth_mm),
            0.0,
            config.jaw_depth_mm,
            config.jaw_width_mm,
        ),
    };
    sketch.entities.push(SketchEntityEntry {
        id: "fixed_jaw_outline".to_string(),
        name: "Fixed Jaw Blank Outline".to_string(),
        sub_layer: Some("JAW_BLANKS".to_string()),
        is_construction: true,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(fixed_cx),
            y: ParamLength::from_mm(fixed_cy),
            width: ParamLength::from_mm(rect_w),
            height: ParamLength::from_mm(rect_h),
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
        },
    });
    sketch.entities.push(SketchEntityEntry {
        id: "movable_jaw_outline".to_string(),
        name: "Movable Jaw Blank Outline".to_string(),
        sub_layer: Some("JAW_BLANKS".to_string()),
        is_construction: true,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(mov_cx),
            y: ParamLength::from_mm(mov_cy),
            width: ParamLength::from_mm(rect_w),
            height: ParamLength::from_mm(rect_h),
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
        },
    });

    // Insert exact shifted pocket contours into the sketch layer
    let mut pocket_entity_ids: Vec<String> = Vec::new();
    for (jaw_tag, jaw_label, reg_set) in [
        ("fixed", "Fixed Jaw Pocket", fixed_pocket_wcs),
        ("movable", "Movable Jaw Pocket", movable_pocket_wcs),
    ] {
        for (idx, reg) in reg_set.regions.iter().enumerate() {
            let shifted = shift_polyline(&reg.outer_ccw, -part_center).ensure_orientation(true);
            let ent_id = format!("jaw_pocket_{jaw_tag}_{idx}");
            sketch.entities.push(SketchEntityEntry {
                id: ent_id.clone(),
                name: format!("{jaw_label} {}", idx + 1),
                sub_layer: Some("POCKETS".to_string()),
                is_construction: false,
                entity: SketchEntity::new_polyline(shifted),
            });
            pocket_entity_ids.push(ent_id);
        }
    }
    sketch.rebuild_regions(None);

    // Op 1: Skim-face soft jaw tops (`0.5 mm`)
    let mut op_face = Operation::new(
        "op_sj_1_face",
        "1. Face Soft-Jaw Tops",
        OperationKind::Facing,
    );
    op_face.tool_id = config.roughing_tool_id.clone();
    op_face.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_face.heights.bottom = HeightSpec::new(HeightReference::StockTop, -skim_mm);
    op_face.passes.multiple_depths = false;
    op_face.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    sj_setup.operations.push(op_face);

    // Op 2: Pocket2D roughing of contoured soft-jaw step pockets
    let mut op_pocket = Operation::new(
        "op_sj_2_pocket",
        "2. Rough Soft-Jaw Pockets",
        OperationKind::Pocket2D,
    );
    op_pocket.tool_id = config.roughing_tool_id.clone();
    op_pocket.heights.top = HeightSpec::new(HeightReference::StockTop, -skim_mm);
    op_pocket.heights.bottom = HeightSpec::new(HeightReference::StockTop, -total_cut_depth);
    op_pocket.passes.multiple_depths = config.grip_depth_mm > 2.5;
    op_pocket.passes.max_stepdown = ParamLength::from_mm(2.0);
    op_pocket.passes.stock_to_leave_radial = ParamLength::from_mm(0.15);
    op_pocket.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    if let StrategyParams::Pocket2D(p) = op_pocket.strategy_mut() {
        p.strategy = PocketStrategyKind::ContourParallel;
    }
    if !sketch.regions.is_empty() {
        for reg in &sketch.regions {
            op_pocket
                .selections
                .push(GeometrySelectionItem::SketchRegion {
                    sketch_id: sketch_id.clone(),
                    region_id: reg.id.clone(),
                });
        }
    } else {
        for ent_id in &pocket_entity_ids {
            op_pocket
                .selections
                .push(GeometrySelectionItem::SketchEntity {
                    sketch_id: sketch_id.clone(),
                    entity_id: ent_id.clone(),
                    reverse: false,
                    side_flip: false,
                });
        }
    }
    sj_setup.operations.push(op_pocket);

    // Op 3: Contour2D wall finishing pass (`reverse = true` -> CCW -> LeftClimb cuts INSIDE pocket!)
    let mut op_finish = Operation::new(
        "op_sj_3_finish",
        "3. Finish Soft-Jaw Pocket Walls",
        OperationKind::Contour2D,
    );
    op_finish.tool_id = config
        .finishing_tool_id
        .clone()
        .or_else(|| config.roughing_tool_id.clone());
    op_finish.heights.top = HeightSpec::new(HeightReference::StockTop, -skim_mm);
    op_finish.heights.bottom = HeightSpec::new(HeightReference::StockTop, -total_cut_depth);
    op_finish.passes.multiple_depths = false;
    op_finish.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
    op_finish.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    if let StrategyParams::Contour2D(p) = op_finish.strategy_mut() {
        p.compensation = ContourCompensation::LeftClimb;
        p.roughing_passes = 0;
        p.finish_passes = 1;
        p.finish_stepover = ParamLength::from_mm(0.15);
    }
    for ent_id in &pocket_entity_ids {
        op_finish
            .selections
            .push(GeometrySelectionItem::SketchEntity {
                sketch_id: sketch_id.clone(),
                entity_id: ent_id.clone(),
                reverse: true,
                side_flip: false,
            });
    }
    sj_setup.operations.push(op_finish);

    (sj_setup, sketch)
}

fn shift_polyline(poly: &ArcPolyline2D, delta: DVec2) -> ArcPolyline2D {
    let segments = poly
        .segments
        .iter()
        .map(|seg| match *seg {
            ArcSegment2D::Line { start, end } => ArcSegment2D::Line {
                start: start + delta,
                end: end + delta,
            },
            ArcSegment2D::Arc {
                start,
                end,
                center,
                radius,
                ccw,
            } => ArcSegment2D::Arc {
                start: start + delta,
                end: end + delta,
                center: center + delta,
                radius,
                ccw,
            },
        })
        .collect();
    ArcPolyline2D {
        segments,
        closed: poly.closed,
    }
}

/// Maps a Setup-WCS jaw pocket [`RegionSet2D`] into jaw-local `[0..W] × [0..D]` coordinates where
/// `Y = 0` is always the inner clamping face (facing the workpiece) and `Y = D` is the outer back
/// face bolted to the vise master jaw.
fn map_region_to_jaw_local(
    region_wcs: &RegionSet2D,
    part_center: DVec2,
    clamp_axis: ViseClampAxis,
    is_fixed_positive_side: bool,
    jaw_width_mm: f64,
    jaw_gap_mm: f64,
) -> RegionSet2D {
    let half_w = 0.5 * jaw_width_mm;
    let half_gap = 0.5 * jaw_gap_mm;

    let map_pt = |p: DVec2| -> DVec2 {
        let rel = p - part_center;
        match (clamp_axis, is_fixed_positive_side) {
            // Fixed (+Y): X_local = rel.x + W/2, Y_local = rel.y - gap/2
            (ViseClampAxis::Y, true) => DVec2::new(rel.x + half_w, rel.y - half_gap),
            // Movable (-Y): rotated 180° in XY so inner clamping face is still Y_local = 0!
            (ViseClampAxis::Y, false) => DVec2::new(-rel.x + half_w, -rel.y - half_gap),
            // Fixed (+X): rotate -90° so X_local = rel.y + W/2, Y_local = rel.x - gap/2 (Wait: preserves orientation: det = +1?
            // Let's keep det = +1 (rigid rotation): X_local = -rel.y + W/2, Y_local = rel.x - gap/2!)
            (ViseClampAxis::X, true) => DVec2::new(-rel.y + half_w, rel.x - half_gap),
            // Movable (-X): rigid +90° rotation: X_local = rel.y + W/2, Y_local = -rel.x - gap/2!
            (ViseClampAxis::X, false) => DVec2::new(rel.y + half_w, -rel.x - half_gap),
        }
    };

    let map_poly = |poly: &ArcPolyline2D| -> ArcPolyline2D {
        let segments = poly
            .segments
            .iter()
            .map(|seg| match *seg {
                ArcSegment2D::Line { start, end } => ArcSegment2D::Line {
                    start: map_pt(start),
                    end: map_pt(end),
                },
                ArcSegment2D::Arc {
                    start,
                    end,
                    center,
                    radius,
                    ccw,
                } => ArcSegment2D::Arc {
                    start: map_pt(start),
                    end: map_pt(end),
                    center: map_pt(center),
                    radius,
                    ccw,
                },
            })
            .collect();
        ArcPolyline2D {
            segments,
            closed: poly.closed,
        }
    };

    let regions = region_wcs
        .regions
        .iter()
        .map(|r| {
            PocketRegion2D::new(
                map_poly(&r.outer_ccw),
                r.islands_cw.iter().map(map_poly).collect(),
            )
        })
        .collect();
    RegionSet2D::from_regions(regions)
}

// ── Watertight Binary STL Soft-Jaw Mesh Builder ────────────────────────────────────────────────

/// Discretizes an `ArcPolyline2D` into a closed 2D vertex ring (`<= 0.02 mm` arc chord sag),
/// snapping points within `5e-3 mm` of the jaw boundary rectangle `[0..W] × [0..D]` cleanly onto
/// the boundary and quantizing onto a `0.001 mm` (`1 µm`) grid for exact manifold edge sharing.
fn discretize_closed_loop(poly: &ArcPolyline2D, w: f64, d: f64) -> Vec<DVec2> {
    let mut pts = Vec::new();
    const CHORD_TOL: f64 = 0.02;
    for seg in &poly.segments {
        match *seg {
            ArcSegment2D::Line { start, .. } => pts.push(snap_jaw_pt(start, w, d)),
            ArcSegment2D::Arc {
                start,
                center,
                radius,
                ccw,
                ..
            } => {
                let sweep = seg.signed_sweep_rad().abs();
                let step_angle = if radius > CHORD_TOL {
                    2.0 * (1.0 - CHORD_TOL / radius).clamp(-1.0, 1.0).acos()
                } else {
                    0.5
                };
                let steps = ((sweep / step_angle.max(0.05)).ceil() as usize).clamp(2, 64);
                let a0 = (start.y - center.y).atan2(start.x - center.x);
                let dir = if ccw { 1.0 } else { -1.0 };
                for k in 0..steps {
                    let ang = a0 + dir * sweep * (k as f64 / steps as f64);
                    let p = center + DVec2::new(ang.cos(), ang.sin()) * radius;
                    pts.push(snap_jaw_pt(p, w, d));
                }
            }
        }
    }
    clean_ring_vertices(&mut pts);
    pts
}

fn snap_jaw_pt(p: DVec2, w: f64, d: f64) -> DVec2 {
    let snap = |val: f64, lo: f64, hi: f64| -> f64 {
        if (val - lo).abs() <= 5e-3 {
            lo
        } else if (val - hi).abs() <= 5e-3 {
            hi
        } else {
            (val * 1000.0).round() * 0.001
        }
    };
    DVec2::new(snap(p.x, 0.0, w), snap(p.y, 0.0, d))
}

fn clean_ring_vertices(pts: &mut Vec<DVec2>) {
    let mut out: Vec<DVec2> = Vec::with_capacity(pts.len());
    for &p in pts.iter() {
        if out.last().is_none_or(|prev| prev.distance(p) > 2e-3) {
            out.push(p);
        }
    }
    while out.len() > 2 && out[0].distance(out[out.len() - 1]) <= 2e-3 {
        out.pop();
    }
    *pts = out;
}

fn ring_to_line_polyline(pts: &[DVec2]) -> ArcPolyline2D {
    let n = pts.len();
    if n < 3 {
        return ArcPolyline2D::default();
    }
    let mut segments = Vec::with_capacity(n);
    for i in 0..n {
        segments.push(ArcSegment2D::Line {
            start: pts[i],
            end: pts[(i + 1) % n],
        });
    }
    ArcPolyline2D {
        segments,
        closed: true,
    }
}

fn signed_area_ring(pts: &[DVec2]) -> f64 {
    let n = pts.len();
    if n < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        sum += a.x * b.y - b.x * a.y;
    }
    0.5 * sum
}

fn ensure_ring_orientation(mut pts: Vec<DVec2>, ccw: bool) -> Vec<DVec2> {
    let area = signed_area_ring(&pts);
    if (ccw && area < 0.0) || (!ccw && area > 0.0) {
        pts.reverse();
    }
    pts
}

/// Inserts any point from `candidates` that lies strictly on an edge `(a, b)` of `ring` so
/// adjacent planar faces on `Z = Z_floor` share identical half-edges with zero T-junctions.
fn subdivide_ring_collinear(ring: &mut Vec<DVec2>, candidates: &[DVec2]) {
    let n = ring.len();
    if n < 2 {
        return;
    }
    let mut out = Vec::with_capacity(n + 8);
    for i in 0..n {
        let a = ring[i];
        let b = ring[(i + 1) % n];
        out.push(a);
        let ab = b - a;
        let len2 = ab.length_squared();
        if len2 <= 1e-8 {
            continue;
        }
        let len = len2.sqrt();
        let mut splits: Vec<(f64, DVec2)> = Vec::new();
        for &c in candidates {
            if c.distance(a) <= 2e-3 || c.distance(b) <= 2e-3 {
                continue;
            }
            let t = (c - a).dot(ab) / len2;
            if t <= 1e-5 || t >= 1.0 - 1e-5 {
                continue;
            }
            let proj = a + ab * t;
            if proj.distance(c) <= 2e-3 && (t * len) > 2e-3 && ((1.0 - t) * len) > 2e-3 {
                splits.push((t, c));
            }
        }
        splits.sort_by(|x, y| x.0.total_cmp(&y.0));
        splits.dedup_by(|x, y| x.1.distance(y.1) <= 2e-3);
        for (_, pt) in splits {
            out.push(pt);
        }
    }
    clean_ring_vertices(&mut out);
    *ring = out;
}

/// Exact `f64` 2D ear-clipping triangulator supporting a CCW outer ring with zero or more CW holes.
///
/// Holes are spliced into the outer boundary via non-intersecting mutual-visibility bridges
/// (David Eberly's rightmost-vertex bridge algorithm) before ear clipping.
fn triangulate_polygon_with_holes_2d(outer: &[DVec2], holes: &[Vec<DVec2>]) -> Vec<[DVec2; 3]> {
    let mut poly = ensure_ring_orientation(outer.to_vec(), true);
    if poly.len() < 3 {
        return Vec::new();
    }

    // Sort holes descending by rightmost X so bridges never cross unprocessed holes
    let mut ordered_holes: Vec<(f64, usize, Vec<DVec2>)> = holes
        .iter()
        .filter(|h| h.len() >= 3)
        .filter_map(|h| {
            let cw = ensure_ring_orientation(h.clone(), false);
            let (idx, max_x) = cw
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.x.total_cmp(&b.1.x))
                .map(|(i, p)| (i, p.x))?;
            Some((max_x, idx, cw))
        })
        .collect();
    ordered_holes.sort_by(|a, b| b.0.total_cmp(&a.0));

    for (_, hole_right_idx, hole) in ordered_holes {
        let m = hole[hole_right_idx];
        if let Some(outer_idx) = find_visible_bridge_vertex(&poly, m) {
            let mut merged = Vec::with_capacity(poly.len() + hole.len() + 2);
            merged.extend_from_slice(&poly[..=outer_idx]);
            for k in 0..=hole.len() {
                merged.push(hole[(hole_right_idx + k) % hole.len()]);
            }
            merged.push(poly[outer_idx]);
            merged.extend_from_slice(&poly[(outer_idx + 1)..]);
            poly = merged;
        }
    }

    ear_clip_simple_ccw(&poly)
}

fn find_visible_bridge_vertex(poly: &[DVec2], m: DVec2) -> Option<usize> {
    let n = poly.len();
    // 1. Cast horizontal ray `+X` from `m` to find closest intersection with an edge of `poly`
    let mut best_x = f64::INFINITY;
    let mut best_edge_idx: Option<usize> = None;
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        // Ray crosses upward or downward edge with y straddling m.y
        let crosses = (a.y <= m.y && b.y > m.y) || (b.y <= m.y && a.y > m.y);
        if !crosses {
            continue;
        }
        let t = (m.y - a.y) / (b.y - a.y);
        let x_hit = a.x + t * (b.x - a.x);
        if x_hit >= m.x - 1e-9 && x_hit < best_x {
            best_x = x_hit;
            best_edge_idx = Some(i);
        }
    }

    let edge_i = best_edge_idx?;
    let a = poly[edge_i];
    let b = poly[(edge_i + 1) % n];
    let p_idx = if a.x >= b.x { edge_i } else { (edge_i + 1) % n };
    let p = poly[p_idx];
    let i_pt = DVec2::new(best_x, m.y);

    // 2. Check if any reflex vertex of `poly` lies strictly inside triangle `(m, i_pt, p)`
    let mut candidate_idx = p_idx;
    let mut best_angle = f64::INFINITY;
    let mut best_dist = f64::INFINITY;

    for (idx, &v) in poly.iter().enumerate() {
        if idx == p_idx {
            continue;
        }
        if v.distance_squared(m) < 1e-12 || v.distance_squared(p) < 1e-12 {
            continue;
        }
        if point_in_triangle_2d(v, m, i_pt, p) {
            let dir = v - m;
            let ang = dir.y.atan2(dir.x).abs();
            let dist = dir.length_squared();
            if ang < best_angle - 1e-9 || ((ang - best_angle).abs() <= 1e-9 && dist < best_dist) {
                best_angle = ang;
                best_dist = dist;
                candidate_idx = idx;
            }
        }
    }

    Some(candidate_idx)
}

fn ear_clip_simple_ccw(poly: &[DVec2]) -> Vec<[DVec2; 3]> {
    let n = poly.len();
    if n < 3 {
        return Vec::new();
    }
    let mut indices: Vec<usize> = (0..n).collect();
    let mut tris = Vec::with_capacity(n.saturating_sub(2));

    let mut guard = n * n + 16;
    while indices.len() > 3 && guard > 0 {
        guard -= 1;
        let m = indices.len();
        let mut clipped = false;

        // Pick the best strictly-convex ear (preferring larger minimum angle / non-sliver)
        let mut best_ear: Option<(usize, f64)> = None;
        for i in 0..m {
            let i_prev = indices[(i + m - 1) % m];
            let i_curr = indices[i];
            let i_next = indices[(i + 1) % m];
            let a = poly[i_prev];
            let b = poly[i_curr];
            let c = poly[i_next];

            let area2 = cross2d(b - a, c - a);
            if area2 <= 1e-10 {
                continue;
            }
            let mut contains_other = false;
            for (k_pos, &idx) in indices.iter().enumerate() {
                if k_pos == i || k_pos == (i + m - 1) % m || k_pos == (i + 1) % m {
                    continue;
                }
                let p = poly[idx];
                if p.distance_squared(a) <= 1e-12
                    || p.distance_squared(b) <= 1e-12
                    || p.distance_squared(c) <= 1e-12
                {
                    continue;
                }
                if point_in_triangle_2d(p, a, b, c) {
                    contains_other = true;
                    break;
                }
            }
            if contains_other {
                continue;
            }
            let perim2 =
                a.distance_squared(b) + b.distance_squared(c) + c.distance_squared(a) + 1e-12;
            let quality = area2 / perim2;
            if best_ear.is_none_or(|(_, q)| quality > q) {
                best_ear = Some((i, quality));
            }
        }

        if let Some((ear_pos, _)) = best_ear {
            let i_prev = indices[(ear_pos + m - 1) % m];
            let i_curr = indices[ear_pos];
            let i_next = indices[(ear_pos + 1) % m];
            tris.push([poly[i_prev], poly[i_curr], poly[i_next]]);
            indices.remove(ear_pos);
            clipped = true;
        }

        if !clipped {
            // Remove collinear / zero-area spikes if any remain
            let mut removed_deg = false;
            for i in 0..m {
                let a = poly[indices[(i + m - 1) % m]];
                let b = poly[indices[i]];
                let c = poly[indices[(i + 1) % m]];
                if cross2d(b - a, c - a).abs() <= 1e-8 {
                    indices.remove(i);
                    removed_deg = true;
                    break;
                }
            }
            if !removed_deg {
                // Fallback: clip first positive-area ear
                for i in 0..m {
                    let a = poly[indices[(i + m - 1) % m]];
                    let b = poly[indices[i]];
                    let c = poly[indices[(i + 1) % m]];
                    if cross2d(b - a, c - a) > 0.0 {
                        tris.push([a, b, c]);
                        indices.remove(i);
                        removed_deg = true;
                        break;
                    }
                }
                if !removed_deg {
                    break;
                }
            }
        }
    }

    if indices.len() == 3 {
        let a = poly[indices[0]];
        let b = poly[indices[1]];
        let c = poly[indices[2]];
        if cross2d(b - a, c - a) > 1e-10 {
            tris.push([a, b, c]);
        }
    }

    tris
}

#[inline]
fn cross2d(u: DVec2, v: DVec2) -> f64 {
    u.x * v.y - u.y * v.x
}

fn point_in_triangle_2d(p: DVec2, a: DVec2, b: DVec2, c: DVec2) -> bool {
    let area = cross2d(b - a, c - a);
    let (b2, c2) = if area >= 0.0 { (b, c) } else { (c, b) };
    let c1 = cross2d(b2 - a, p - a);
    let c2v = cross2d(c2 - b2, p - b2);
    let c3 = cross2d(a - c2, p - c2);
    c1 >= -1e-9 && c2v >= -1e-9 && c3 >= -1e-9
}

/// Generates a guaranteed-watertight binary STL byte buffer for a single soft jaw in jaw-local
/// coordinates `[0..w] × [0..d] × [0..h]` with contoured step pocket (`grip_depth` deep from `z = h`)
/// and optional horizontal counterbored mounting bolt holes piercing `Y = 0..d`.
fn build_watertight_soft_jaw_stl_bytes(
    header_label: &str,
    w: f64,
    d: f64,
    h: f64,
    grip_depth: f64,
    pocket_local: &RegionSet2D,
    bolt_pattern: SoftJawBoltPattern,
) -> Result<(Vec<u8>, usize), CamError> {
    let z_floor = ((h - grip_depth) * 1000.0).round() * 0.001;
    let w_q = (w * 1000.0).round() * 0.001;
    let d_q = (d * 1000.0).round() * 0.001;
    let h_q = (h * 1000.0).round() * 0.001;

    // 1. Discretize pocket polygons into straight-segment regions so `jaw_rect \ pocket` shares
    //    exact vertices along every pocket wall and jaw perimeter boundary!
    let mut disc_pocket_regs: Vec<PocketRegion2D> = Vec::new();
    for reg in &pocket_local.regions {
        let outer_pts =
            ensure_ring_orientation(discretize_closed_loop(&reg.outer_ccw, w_q, d_q), true);
        if outer_pts.len() < 3 || signed_area_ring(&outer_pts) <= 1e-4 {
            continue;
        }
        let mut islands = Vec::new();
        for isl in &reg.islands_cw {
            let isl_pts = ensure_ring_orientation(discretize_closed_loop(isl, w_q, d_q), false);
            if isl_pts.len() >= 3 && signed_area_ring(&isl_pts).abs() > 1e-4 {
                islands.push(ring_to_line_polyline(&isl_pts));
            }
        }
        disc_pocket_regs.push(PocketRegion2D::new(
            ring_to_line_polyline(&outer_pts),
            islands,
        ));
    }

    let jaw_rect = rect_region_2d(0.0, 0.0, w_q, d_q);
    let pocket_poly_set = RegionSet2D::from_regions(disc_pocket_regs).intersect(&jaw_rect);
    let top_land_set = jaw_rect.difference(&pocket_poly_set);

    let extract_rings = |set: &RegionSet2D| -> Vec<(Vec<DVec2>, Vec<Vec<DVec2>>)> {
        let mut out = Vec::new();
        for reg in &set.regions {
            let outer =
                ensure_ring_orientation(discretize_closed_loop(&reg.outer_ccw, w_q, d_q), true);
            if outer.len() < 3 || signed_area_ring(&outer) <= 1e-4 {
                continue;
            }
            let mut holes = Vec::new();
            for isl in &reg.islands_cw {
                let h_ring = ensure_ring_orientation(discretize_closed_loop(isl, w_q, d_q), false);
                if h_ring.len() >= 3 && signed_area_ring(&h_ring).abs() > 1e-4 {
                    holes.push(h_ring);
                }
            }
            out.push((outer, holes));
        }
        out
    };

    let mut pocket_rings = extract_rings(&pocket_poly_set);
    let mut land_rings = extract_rings(&top_land_set);

    // 2. Snap all vertices across pocket_rings, land_rings, and jaw corners onto one canonical 2D
    //    vertex table (`5e-3 mm` weld radius) so boolean-output vertices coincide bit-for-bit,
    //    then subdivide collinear segments so zero T-junctions can ever exist on `Z = Z_floor`.
    let mut all_z_floor_pts: Vec<DVec2> = vec![
        DVec2::new(0.0, 0.0),
        DVec2::new(w_q, 0.0),
        DVec2::new(w_q, d_q),
        DVec2::new(0.0, d_q),
    ];
    let mut intern_2d = |p: DVec2| -> DVec2 {
        for &existing in &all_z_floor_pts {
            if existing.distance(p) <= 5e-3 {
                return existing;
            }
        }
        all_z_floor_pts.push(p);
        p
    };
    for (o, hs) in pocket_rings.iter_mut().chain(land_rings.iter_mut()) {
        for pt in o.iter_mut() {
            *pt = intern_2d(*pt);
        }
        clean_ring_vertices(o);
        for hr in hs.iter_mut() {
            for pt in hr.iter_mut() {
                *pt = intern_2d(*pt);
            }
            clean_ring_vertices(hr);
        }
    }

    for (o, hs) in &mut pocket_rings {
        subdivide_ring_collinear(o, &all_z_floor_pts);
        for hr in hs {
            subdivide_ring_collinear(hr, &all_z_floor_pts);
        }
    }
    for (o, hs) in &mut land_rings {
        subdivide_ring_collinear(o, &all_z_floor_pts);
        for hr in hs {
            subdivide_ring_collinear(hr, &all_z_floor_pts);
        }
    }

    let mut tris: Vec<[DVec3; 3]> = Vec::with_capacity(512);
    let mut push_tri = |a: DVec3, b: DVec3, c: DVec3| {
        if (b - a).cross(c - a).length_squared() > 1e-12 {
            tris.push([a, b, c]);
        }
    };

    // 3. Upper solid (`Z ∈ [z_floor, h_q]`): top face at `z = h_q` (+Z) and vertical walls
    for (outer, holes) in &land_rings {
        for [a, b, c] in triangulate_polygon_with_holes_2d(outer, holes) {
            push_tri(
                DVec3::new(a.x, a.y, h_q),
                DVec3::new(b.x, b.y, h_q),
                DVec3::new(c.x, c.y, h_q),
            );
        }
        // Vertical walls: for CCW outer ring, outward normal is to the RIGHT of `p0 -> p1`.
        // Quad `(p0_bot, p1_bot, p1_top, p0_top)` has outward normal `(p1 - p0) × +Z` (right!).
        for ring in std::iter::once(outer).chain(holes.iter()) {
            let n = ring.len();
            for i in 0..n {
                let p0 = ring[i];
                let p1 = ring[(i + 1) % n];
                let b0 = DVec3::new(p0.x, p0.y, z_floor);
                let b1 = DVec3::new(p1.x, p1.y, z_floor);
                let t1 = DVec3::new(p1.x, p1.y, h_q);
                let t0 = DVec3::new(p0.x, p0.y, h_q);
                push_tri(b0, b1, t1);
                push_tri(b0, t1, t0);
            }
        }
    }

    // 4. Pocket floor face at `Z = z_floor` (normal `+Z`)
    for (outer, holes) in &pocket_rings {
        for [a, b, c] in triangulate_polygon_with_holes_2d(outer, holes) {
            push_tri(
                DVec3::new(a.x, a.y, z_floor),
                DVec3::new(b.x, b.y, z_floor),
                DVec3::new(c.x, c.y, z_floor),
            );
        }
    }

    // 5. Bottom face at `Z = 0.0` (normal `-Z`)
    push_tri(
        DVec3::new(0.0, 0.0, 0.0),
        DVec3::new(0.0, d_q, 0.0),
        DVec3::new(w_q, d_q, 0.0),
    );
    push_tri(
        DVec3::new(0.0, 0.0, 0.0),
        DVec3::new(w_q, d_q, 0.0),
        DVec3::new(w_q, 0.0, 0.0),
    );

    // 6. Lower side walls (`Z ∈ [0, z_floor]`) sharing exact subdivided `Z = z_floor` vertices!
    let mut y0_xs = vec![0.0, w_q];
    let mut yd_xs = vec![0.0, w_q];
    let mut x0_ys = vec![0.0, d_q];
    let mut xw_ys = vec![0.0, d_q];
    for &p in &all_z_floor_pts {
        if p.y.abs() <= 2e-3 {
            y0_xs.push(p.x);
        }
        if (p.y - d_q).abs() <= 2e-3 {
            yd_xs.push(p.x);
        }
        if p.x.abs() <= 2e-3 {
            x0_ys.push(p.y);
        }
        if (p.x - w_q).abs() <= 2e-3 {
            xw_ys.push(p.y);
        }
    }
    let sort_dedup = |v: &mut Vec<f64>| {
        v.sort_by(f64::total_cmp);
        v.dedup_by(|a, b| (*a - *b).abs() <= 2e-3);
    };
    sort_dedup(&mut y0_xs);
    sort_dedup(&mut yd_xs);
    sort_dedup(&mut x0_ys);
    sort_dedup(&mut xw_ys);

    // Left lower wall (`X = 0`, normal `-X`) in `(Y, Z)` plane: CCW in `(-Y, +Z)` -> fan/triangulate
    {
        let mut outer_yz = vec![DVec2::new(d_q, 0.0), DVec2::new(0.0, 0.0)];
        for &y in &x0_ys {
            outer_yz.push(DVec2::new(y, z_floor));
        }
        // `(u = y, v = z)`: CCW in `(y, z)` gives normal `+X`, so reverse for `-X`
        let outer_ccw = ensure_ring_orientation(outer_yz, true);
        for [a, b, c] in triangulate_polygon_with_holes_2d(&outer_ccw, &[]) {
            push_tri(
                DVec3::new(0.0, a.x, a.y),
                DVec3::new(0.0, c.x, c.y),
                DVec3::new(0.0, b.x, b.y),
            );
        }
    }
    // Right lower wall (`X = w_q`, normal `+X`)
    {
        let mut outer_yz = vec![DVec2::new(0.0, 0.0), DVec2::new(d_q, 0.0)];
        for &y in xw_ys.iter().rev() {
            outer_yz.push(DVec2::new(y, z_floor));
        }
        let outer_ccw = ensure_ring_orientation(outer_yz, true);
        for [a, b, c] in triangulate_polygon_with_holes_2d(&outer_ccw, &[]) {
            push_tri(
                DVec3::new(w_q, a.x, a.y),
                DVec3::new(w_q, b.x, b.y),
                DVec3::new(w_q, c.x, c.y),
            );
        }
    }

    // 7. Determine whether horizontal mounting bolt holes + counterbores fit cleanly inside lower body
    let active_bolts: Option<(Vec<f64>, f64, f64, f64, f64)> = bolt_pattern
        .dimensions_mm()
        .and_then(|(spacing, bolt_dia, cbore_dia, cbore_depth, raw_z)| {
            if spacing <= 0.0 || bolt_dia <= 0.5 || cbore_dia < bolt_dia {
                return None;
            }
            let cbore_r = 0.5 * cbore_dia;
            let bolt_z = if raw_z > cbore_r + 1.5 && raw_z + cbore_r + 1.5 <= z_floor {
                raw_z
            } else {
                0.5 * z_floor
            };
            let bolt_z_q = (bolt_z * 1000.0).round() * 0.001;
            let half_sp = 0.5 * spacing;
            let x_l = 0.5 * w_q - half_sp;
            let x_r = 0.5 * w_q + half_sp;
            let fits_x =
                x_l - cbore_r >= 2.0 && x_r + cbore_r <= w_q - 2.0 && spacing > cbore_dia + 2.0;
            let fits_z = bolt_z_q - cbore_r >= 1.5 && bolt_z_q + cbore_r <= z_floor - 1.5;
            let eff_cbore_depth = cbore_depth.min(d_q - 3.0).max(0.0);
            if fits_x && fits_z && d_q > 4.0 {
                Some((
                    vec![
                        (x_l * 1000.0).round() * 0.001,
                        (x_r * 1000.0).round() * 0.001,
                    ],
                    bolt_z_q,
                    0.5 * bolt_dia,
                    cbore_r,
                    (eff_cbore_depth * 1000.0).round() * 0.001,
                ))
            } else {
                None
            }
        });

    const BOLT_SEGS: usize = 24;
    let circle_xz = |cx: f64, cz: f64, r: f64| -> Vec<DVec2> {
        (0..BOLT_SEGS)
            .map(|k| {
                let ang = TAU * (k as f64) / (BOLT_SEGS as f64);
                DVec2::new(
                    ((cx + r * ang.cos()) * 1000.0).round() * 0.001,
                    ((cz + r * ang.sin()) * 1000.0).round() * 0.001,
                )
            })
            .collect()
    };

    // Front lower wall (`Y = 0`, normal `-Y`), with counterbore holes when active
    {
        let mut outer_xz = vec![DVec2::new(0.0, 0.0), DVec2::new(w_q, 0.0)];
        for &x in y0_xs.iter().rev() {
            outer_xz.push(DVec2::new(x, z_floor));
        }
        let outer_ccw = ensure_ring_orientation(outer_xz, true);
        let front_holes: Vec<Vec<DVec2>> = match &active_bolts {
            Some((xs, bz, bolt_r, cbore_r, cbore_d)) => {
                let r_front = if *cbore_d > 0.25 { *cbore_r } else { *bolt_r };
                xs.iter()
                    .map(|&bx| ensure_ring_orientation(circle_xz(bx, *bz, r_front), false))
                    .collect()
            }
            None => Vec::new(),
        };
        // In `(X, Z)` plane, CCW `(a, b, c)` has `(b-a)×(c-a)` along `-Y` (`X × Z = -Y`)!
        for [a, b, c] in triangulate_polygon_with_holes_2d(&outer_ccw, &front_holes) {
            push_tri(
                DVec3::new(a.x, 0.0, a.y),
                DVec3::new(b.x, 0.0, b.y),
                DVec3::new(c.x, 0.0, c.y),
            );
        }
    }

    // Back lower wall (`Y = d_q`, normal `+Y`), with through-bolt holes when active
    {
        let mut outer_xz = vec![DVec2::new(0.0, 0.0), DVec2::new(w_q, 0.0)];
        for &x in yd_xs.iter().rev() {
            outer_xz.push(DVec2::new(x, z_floor));
        }
        let outer_ccw = ensure_ring_orientation(outer_xz, true);
        let back_holes: Vec<Vec<DVec2>> = match &active_bolts {
            Some((xs, bz, bolt_r, _, _)) => xs
                .iter()
                .map(|&bx| ensure_ring_orientation(circle_xz(bx, *bz, *bolt_r), false))
                .collect(),
            None => Vec::new(),
        };
        // Reverse `(a, c, b)` so normal points `+Y`!
        for [a, b, c] in triangulate_polygon_with_holes_2d(&outer_ccw, &back_holes) {
            push_tri(
                DVec3::new(a.x, d_q, a.y),
                DVec3::new(c.x, d_q, c.y),
                DVec3::new(b.x, d_q, b.y),
            );
        }
    }

    // Horizontal bolt sleeves & counterbore shoulder rings (inward normals into the bore cavity)
    if let Some((xs, bz, bolt_r, cbore_r, cbore_d)) = active_bolts {
        for bx in xs {
            let ring_bolt = circle_xz(bx, bz, bolt_r); // CCW in (X, Z)
            if cbore_d > 0.25 && cbore_r > bolt_r + 0.05 {
                let ring_cbore = circle_xz(bx, bz, cbore_r);
                // 1. Counterbore sleeve `Y = 0 -> cbore_d`:
                //    Front hole on `Y = 0` traverses CW (`a1 -> a0`), so sleeve traverses CCW (`a0 -> a1`)
                //    with inward normal `(a1 - a0) × (b1 - a0)` pointing toward the bore centerline!
                for k in 0..BOLT_SEGS {
                    let k_next = (k + 1) % BOLT_SEGS;
                    let p0 = ring_cbore[k];
                    let p1 = ring_cbore[k_next];
                    let a0 = DVec3::new(p0.x, 0.0, p0.y);
                    let a1 = DVec3::new(p1.x, 0.0, p1.y);
                    let b1 = DVec3::new(p1.x, cbore_d, p1.y);
                    let b0 = DVec3::new(p0.x, cbore_d, p0.y);
                    push_tri(a0, a1, b1);
                    push_tri(a0, b1, b0);
                }
                // 2. Counterbore shoulder annulus at `Y = cbore_d` (normal `-Y` facing front mouth):
                //    Outer edge `c0 -> c1` pairs with counterbore sleeve back edge `b1 -> b0` (`c1 -> c0`);
                //    Inner edge `i1 -> i0` pairs with through-bolt sleeve front edge `a0 -> a1` (`i0 -> i1`).
                for k in 0..BOLT_SEGS {
                    let k_next = (k + 1) % BOLT_SEGS;
                    let c0 = DVec3::new(ring_cbore[k].x, cbore_d, ring_cbore[k].y);
                    let c1 = DVec3::new(ring_cbore[k_next].x, cbore_d, ring_cbore[k_next].y);
                    let i0 = DVec3::new(ring_bolt[k].x, cbore_d, ring_bolt[k].y);
                    let i1 = DVec3::new(ring_bolt[k_next].x, cbore_d, ring_bolt[k_next].y);
                    push_tri(c0, c1, i1);
                    push_tri(c0, i1, i0);
                }
                // 3. Through-bolt sleeve `Y = cbore_d -> d_q`:
                //    Back hole on `Y = d_q` traverses CCW (`b0 -> b1`), pairing with sleeve back edge `b1 -> b0`.
                for k in 0..BOLT_SEGS {
                    let k_next = (k + 1) % BOLT_SEGS;
                    let p0 = ring_bolt[k];
                    let p1 = ring_bolt[k_next];
                    let a0 = DVec3::new(p0.x, cbore_d, p0.y);
                    let a1 = DVec3::new(p1.x, cbore_d, p1.y);
                    let b1 = DVec3::new(p1.x, d_q, p1.y);
                    let b0 = DVec3::new(p0.x, d_q, p0.y);
                    push_tri(a0, a1, b1);
                    push_tri(a0, b1, b0);
                }
            } else {
                // Straight through-bolt sleeve `Y = 0 -> d_q`
                for k in 0..BOLT_SEGS {
                    let k_next = (k + 1) % BOLT_SEGS;
                    let p0 = ring_bolt[k];
                    let p1 = ring_bolt[k_next];
                    let a0 = DVec3::new(p0.x, 0.0, p0.y);
                    let a1 = DVec3::new(p1.x, 0.0, p1.y);
                    let b1 = DVec3::new(p1.x, d_q, p1.y);
                    let b0 = DVec3::new(p0.x, d_q, p0.y);
                    push_tri(a0, a1, b1);
                    push_tri(a0, b1, b0);
                }
            }
        }
    }

    if tris.is_empty() {
        return Err(CamError::InvalidFixture {
            fixture_name: header_label.to_string(),
            reason: "soft jaw STL generator produced zero triangles".to_string(),
        });
    }

    let bytes = encode_binary_stl(header_label, &tris);
    Ok((bytes, tris.len()))
}

fn encode_binary_stl(header_label: &str, tris: &[[DVec3; 3]]) -> Vec<u8> {
    let mut out = Vec::with_capacity(84 + tris.len() * 50);
    let mut header = [0u8; 80];
    let label_bytes = header_label.as_bytes();
    let copy_len = label_bytes.len().min(79);
    header[..copy_len].copy_from_slice(&label_bytes[..copy_len]);
    out.extend_from_slice(&header);
    out.extend_from_slice(&(tris.len() as u32).to_le_bytes());

    for &[v0, v1, v2] in tris {
        let n = (v1 - v0).cross(v2 - v0).normalize_or_zero();
        for c in [n.x as f32, n.y as f32, n.z as f32] {
            out.extend_from_slice(&c.to_le_bytes());
        }
        for v in [v0, v1, v2] {
            for c in [v.x as f32, v.y as f32, v.z as f32] {
                out.extend_from_slice(&c.to_le_bytes());
            }
        }
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    out
}

fn region_set_aabb(set: &RegionSet2D) -> Option<Aabb2> {
    let mut acc = Aabb2::EMPTY;
    for r in &set.regions {
        let b = r.exact_aabb();
        if b.is_valid() {
            acc = acc.union(&b);
        }
    }
    acc.is_valid().then_some(acc)
}

fn validate_positive_wiz(name: &str, field: &str, v: f64) -> Result<f64, CamError> {
    if !v.is_finite() || v <= 1e-6 {
        return Err(CamError::InvalidFixture {
            fixture_name: name.to_string(),
            reason: format!("'{field}' must be finite and > 0 (got {v})"),
        });
    }
    Ok(v)
}

fn validate_non_negative_wiz(name: &str, field: &str, v: f64) -> Result<f64, CamError> {
    if !v.is_finite() || v < 0.0 {
        return Err(CamError::InvalidFixture {
            fixture_name: name.to_string(),
            reason: format!("'{field}' must be finite and >= 0 (got {v})"),
        });
    }
    Ok(v)
}

fn validate_finite_wiz(name: &str, field: &str, v: f64) -> Result<f64, CamError> {
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
