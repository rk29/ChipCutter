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

use chip_bvh::FlatBvh;
use chip_cad::CadDocument;
use chip_geom2d::{
    PocketRegion2D, RegionSet2D, contours_to_region_set, simplify_rdp_2d,
    triangulate_polygon_with_holes_2d,
};
use chip_math::{
    Aabb2, Aabb3, ArcPolyline2D, ArcSegment2D, DVec2, DVec3, DisplayUnit, ParamLength,
};
use chip_sketch::{
    RectAnchorMode, SketchAnchor, SketchEntity, SketchEntityEntry, SketchLayer, SketchPlaneRef,
};
use chip_tools::ToolId;
use rayon::prelude::*;
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
    if surface.triangles.is_empty() || !surface.bounds.is_valid() {
        let b = resolved.model_aabb_wcs;
        if b.is_valid() {
            return Ok(rect_region_2d(b.min.x, b.min.y, b.max.x, b.max.y));
        }
        return Err(CamError::EmptyCadDocument);
    }

    let z_min_part = surface.bounds.min.z;
    let z_max_part = surface.bounds.max.z;
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

    let swept = extract_swept_clamping_band_silhouette(&surface, z_lo, z_hi);
    if !swept.is_empty() {
        Ok(swept)
    } else {
        Ok(rect_region_2d(
            surface.bounds.min.x,
            surface.bounds.min.y,
            surface.bounds.max.x,
            surface.bounds.max.y,
        ))
    }
}

/// Exact continuous swept XY silhouette of `surface` across the vertical clamping Z band `[z_lo, z_hi]`.
///
/// Captures the full continuous horizontal projection of every triangle clipped to `[z_lo, z_hi]`
/// (eliminating discrete Z-slice scalloping on continuously twisting helical blades) combined with
/// exact mid-slab cross-section occupancy for purely vertical prism interiors, extracted via a fine
/// parallel occupancy grid + Marching Squares with analytical boundary bisection, convex-chord
/// refinement, and sharp-corner restoration.
fn extract_swept_clamping_band_silhouette(
    surface: &SurfaceMesh3D,
    z_lo: f64,
    z_hi: f64,
) -> RegionSet2D {
    if surface.triangles.is_empty() || !surface.bounds.is_valid() {
        return RegionSet2D::empty();
    }

    let z_min_part = surface.bounds.min.z;
    let z_max_part = surface.bounds.max.z;
    // Include coplanar part bottom/top caps when the band reaches the part's extreme Z, while
    // excluding a wider lower ledge ending strictly at `z_lo` when gripping an elevated band.
    let z_lo_clip = if z_lo <= z_min_part + 1e-5 {
        z_lo - 1e-6
    } else {
        z_lo + 1e-6
    };
    let z_hi_clip = if z_hi >= z_max_part - 1e-5 {
        z_hi + 1e-6
    } else {
        z_hi - 1e-6
    };
    let (z_lo_clip, z_hi_clip) = if z_hi_clip > z_lo_clip {
        (z_lo_clip, z_hi_clip)
    } else {
        (z_lo - 1e-6, z_hi + 1e-6)
    };

    // 1. Mid-slab horizontal cross-section covers interior XY columns of purely vertical prisms
    //    whose vertical side walls have zero projected XY area inside `[z_lo, z_hi]`.
    let z_mid = 0.5 * (z_lo + z_hi);
    let mid_contours = slice_surface_contours_z(surface, z_mid);
    let mid_slice_region = contours_to_region_set(&mid_contours);

    // 2. Clip every triangle intersecting `[z_lo_clip, z_hi_clip]` to the Z slab and project the
    //    resulting convex polygon (3..5 vertices) into 2D XY triangles accelerated by a `FlatBvh`.
    let slab_tris = surface.bvh.query_z_slab(z_lo_clip, z_hi_clip);
    let mut proj_tris: Vec<[DVec2; 3]> = Vec::with_capacity(slab_tris.len() * 2);
    let mut proj_aabbs: Vec<Aabb3> = Vec::with_capacity(slab_tris.len() * 2);
    let mut swept_aabb = region_set_aabb(&mid_slice_region).unwrap_or(Aabb2::EMPTY);

    for &ti in &slab_tris {
        let tri = &surface.triangles[ti];
        if tri.aabb.max.z < z_lo_clip || tri.aabb.min.z > z_hi_clip {
            continue;
        }
        let [i0, i1, i2] = tri.indices;
        let v0 = surface.vertices[i0 as usize];
        let v1 = surface.vertices[i1 as usize];
        let v2 = surface.vertices[i2 as usize];

        let mut clipped_buf = [DVec3::ZERO; 7];
        let clipped_len =
            chip_bvh::clip_triangle_to_z_slab(v0, v1, v2, z_lo_clip, z_hi_clip, &mut clipped_buf);
        if clipped_len < 3 {
            continue;
        }
        let clipped = &clipped_buf[..clipped_len];
        let p0 = DVec2::new(clipped[0].x, clipped[0].y);
        for k in 1..(clipped.len() - 1) {
            let p1 = DVec2::new(clipped[k].x, clipped[k].y);
            let p2 = DVec2::new(clipped[k + 1].x, clipped[k + 1].y);
            let area2 = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
            if area2.abs() <= 1e-10 {
                continue;
            }
            let min_x = p0.x.min(p1.x).min(p2.x);
            let max_x = p0.x.max(p1.x).max(p2.x);
            let min_y = p0.y.min(p1.y).min(p2.y);
            let max_y = p0.y.max(p1.y).max(p2.y);
            swept_aabb.extend_point(DVec2::new(min_x, min_y));
            swept_aabb.extend_point(DVec2::new(max_x, max_y));
            proj_tris.push([p0, p1, p2]);
            proj_aabbs.push(Aabb3::new(
                DVec3::new(min_x, min_y, 0.0),
                DVec3::new(max_x, max_y, 0.0),
            ));
        }
    }

    if !swept_aabb.is_valid() {
        return RegionSet2D::empty();
    }

    // Fast path: if all geometry inside the slab is purely vertical (no non-vertical triangles
    // projected beyond `mid_slice_region`), return the exact filled outer hull of `mid_slice_region`.
    if proj_tris.is_empty() {
        let filled_hulls: Vec<PocketRegion2D> = mid_slice_region
            .regions
            .into_iter()
            .map(|r| PocketRegion2D::new(r.outer_ccw, Vec::new()))
            .collect();
        return RegionSet2D::from_regions(filled_hulls);
    }

    let bvh_2d = FlatBvh::build(&proj_aabbs);
    // Tiny 5 µm conservative radial margin guarantees simplified curved chords never undercut
    // continuously swept helical blade crests while keeping prismatic dimensions exact to < 0.01 mm.
    const CONSERVATIVE_MARGIN_MM: f64 = 0.005;
    const MARGIN_SQ: f64 = CONSERVATIVE_MARGIN_MM * CONSERVATIVE_MARGIN_MM;

    let is_occupied = |pt: DVec2, scratch: &mut Vec<usize>| -> bool {
        if mid_slice_region.contains_point(pt) {
            return true;
        }
        bvh_2d.query_xy_circle_into(pt.x, pt.y, CONSERVATIVE_MARGIN_MM + 1e-6, scratch);
        for &idx in scratch.iter() {
            let [a, b, c] = proj_tris[idx];
            if point_in_or_near_triangle_2d(pt, a, b, c, MARGIN_SQ) {
                return true;
            }
        }
        false
    };

    let span_x = (swept_aabb.max.x - swept_aabb.min.x).max(0.1);
    let span_y = (swept_aabb.max.y - swept_aabb.min.y).max(0.1);
    let base_cell = (0.005 * span_x.max(span_y)).clamp(0.12, 0.35);
    const MAX_CELLS: f64 = 120_000.0;
    let cell_mm = base_cell.max(((span_x * span_y) / MAX_CELLS).sqrt());

    // Pad by `2 * cell_mm` so every outer border node is guaranteed outside, forcing closed loops.
    let pad = 2.0 * cell_mm;
    let min_x = swept_aabb.min.x - pad;
    let min_y = swept_aabb.min.y - pad;
    let max_x = swept_aabb.max.x + pad;
    let max_y = swept_aabb.max.y + pad;

    let nx = (((max_x - min_x) / cell_mm).ceil() as usize).max(4) + 1;
    let ny = (((max_y - min_y) / cell_mm).ceil() as usize).max(4) + 1;
    let grid_xy = |ix: usize, iy: usize| -> DVec2 {
        DVec2::new(min_x + ix as f64 * cell_mm, min_y + iy as f64 * cell_mm)
    };

    // 3. Evaluate exact swept 2D occupancy across the grid in parallel
    let inside: Vec<bool> = (0..ny)
        .into_par_iter()
        .flat_map_iter(|iy| {
            let mut row = vec![false; nx];
            let mut scratch = Vec::with_capacity(64);
            let y = min_y + iy as f64 * cell_mm;
            for (ix, flag) in row.iter_mut().enumerate() {
                let x = min_x + ix as f64 * cell_mm;
                *flag = is_occupied(DVec2::new(x, y), &mut scratch);
            }
            row
        })
        .collect();

    let h_cols = nx - 1;
    let h_count = h_cols * ny;
    let v_count = nx * (ny - 1);
    let total_edges = h_count + v_count;
    let h_edge = |ix: usize, iy: usize| -> usize { iy * h_cols + ix };
    let v_edge = |ix: usize, iy: usize| -> usize { h_count + iy * nx + ix };

    let mut crossing_specs: Vec<(usize, DVec2, DVec2)> = Vec::new();
    for iy in 0..ny {
        let row = iy * nx;
        for ix in 0..h_cols {
            let a_in = inside[row + ix];
            let b_in = inside[row + ix + 1];
            if a_in != b_in {
                let pa = grid_xy(ix, iy);
                let pb = grid_xy(ix + 1, iy);
                let (p_in, p_out) = if a_in { (pa, pb) } else { (pb, pa) };
                crossing_specs.push((h_edge(ix, iy), p_in, p_out));
            }
        }
    }
    for iy in 0..(ny - 1) {
        let row0 = iy * nx;
        let row1 = (iy + 1) * nx;
        for ix in 0..nx {
            let a_in = inside[row0 + ix];
            let b_in = inside[row1 + ix];
            if a_in != b_in {
                let pa = grid_xy(ix, iy);
                let pb = grid_xy(ix, iy + 1);
                let (p_in, p_out) = if a_in { (pa, pb) } else { (pb, pa) };
                crossing_specs.push((v_edge(ix, iy), p_in, p_out));
            }
        }
    }
    if crossing_specs.is_empty() {
        return RegionSet2D::empty();
    }

    // 4. Analytical boundary bisection in parallel (8 steps -> `< cell_mm / 256`, conservative `t_hi`)
    let refined_points: Vec<(usize, DVec2)> = crossing_specs
        .par_iter()
        .map_init(
            || Vec::with_capacity(64),
            |scratch, &(eid, p_in, p_out)| {
                let mut t_lo = 0.0_f64;
                let mut t_hi = 1.0_f64;
                for _ in 0..8 {
                    let t_mid = 0.5 * (t_lo + t_hi);
                    let p_mid = p_in.lerp(p_out, t_mid);
                    if is_occupied(p_mid, scratch) {
                        t_lo = t_mid;
                    } else {
                        t_hi = t_mid;
                    }
                }
                let t_safe = t_hi.clamp(1e-3, 1.0 - 1e-3);
                (eid, p_in.lerp(p_out, t_safe))
            },
        )
        .collect();

    let mut edge_pos = vec![DVec2::ZERO; total_edges];
    for (eid, pt) in refined_points {
        edge_pos[eid] = pt;
    }

    // 5. Marching Squares directed edge transitions (`inside` on LEFT -> outer CCW, holes CW)
    let mut next_edge = vec![u32::MAX; total_edges];
    let mut active_edges = Vec::with_capacity(crossing_specs.len());
    let mut saddle_scratch = Vec::with_capacity(64);

    let mut link = |from: usize, to: usize| {
        next_edge[from] = to as u32;
        active_edges.push(from);
    };

    for iy in 0..(ny - 1) {
        let row0 = iy * nx;
        let row1 = (iy + 1) * nx;
        for ix in 0..h_cols {
            let mask = u8::from(inside[row0 + ix])
                | (u8::from(inside[row0 + ix + 1]) << 1)
                | (u8::from(inside[row1 + ix + 1]) << 2)
                | (u8::from(inside[row1 + ix]) << 3);
            if mask == 0 || mask == 15 {
                continue;
            }
            let eb = h_edge(ix, iy);
            let er = v_edge(ix + 1, iy);
            let et = h_edge(ix, iy + 1);
            let el = v_edge(ix, iy);

            match mask {
                1 => link(eb, el),
                2 => link(er, eb),
                3 => link(er, el),
                4 => link(et, er),
                5 => {
                    let center = DVec2::new(
                        min_x + (ix as f64 + 0.5) * cell_mm,
                        min_y + (iy as f64 + 0.5) * cell_mm,
                    );
                    if is_occupied(center, &mut saddle_scratch) {
                        link(eb, er);
                        link(et, el);
                    } else {
                        link(eb, el);
                        link(et, er);
                    }
                }
                6 => link(et, eb),
                7 => link(et, el),
                8 => link(el, et),
                9 => link(eb, et),
                10 => {
                    let center = DVec2::new(
                        min_x + (ix as f64 + 0.5) * cell_mm,
                        min_y + (iy as f64 + 0.5) * cell_mm,
                    );
                    if is_occupied(center, &mut saddle_scratch) {
                        link(el, eb);
                        link(er, et);
                    } else {
                        link(er, eb);
                        link(el, et);
                    }
                }
                11 => link(er, et),
                12 => link(el, er),
                13 => link(eb, er),
                14 => link(el, eb),
                _ => {}
            }
        }
    }

    // 6. Trace closed loops, refine convex chords, simplify, restore sharp corners, and keep
    //    filled outer CCW hulls so interior mounting holes never become floating soft-jaw pins.
    let mut visited = vec![false; total_edges];
    let mut polylines: Vec<ArcPolyline2D> = Vec::new();
    let mut refine_scratch = Vec::with_capacity(64);

    for start_e in active_edges {
        if visited[start_e] || next_edge[start_e] == u32::MAX {
            continue;
        }
        let mut loop_pts: Vec<DVec2> = Vec::new();
        let mut cur = start_e;
        let mut closed = false;
        while cur < total_edges && !visited[cur] {
            visited[cur] = true;
            let p = edge_pos[cur];
            if loop_pts
                .last()
                .is_none_or(|prev| prev.distance_squared(p) > 1e-6)
            {
                loop_pts.push(p);
            }
            let nxt = next_edge[cur];
            if nxt == u32::MAX {
                break;
            }
            cur = nxt as usize;
            if cur == start_e {
                closed = true;
                break;
            }
        }
        if !closed || loop_pts.len() < 3 {
            continue;
        }
        if loop_pts
            .first()
            .zip(loop_pts.last())
            .is_some_and(|(a, b)| a.distance_squared(*b) <= 1e-6)
        {
            loop_pts.pop();
        }
        if loop_pts.len() < 3 {
            continue;
        }

        // Refine any chord whose midpoint dips inside the swept silhouette (eliminates sagitta undercut)
        let mut refined: Vec<DVec2> = Vec::with_capacity(loop_pts.len());
        let n_pts = loop_pts.len();
        for i in 0..n_pts {
            let a = loop_pts[i];
            let b = loop_pts[(i + 1) % n_pts];
            refined.push(a);
            refine_swept_chord(
                a,
                b,
                cell_mm,
                0,
                &is_occupied,
                &mut refine_scratch,
                &mut refined,
            );
        }

        if let Some((min_idx, _)) = refined
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.x.total_cmp(&b.x).then_with(|| a.y.total_cmp(&b.y)))
        {
            refined.rotate_left(min_idx);
        }
        if refined.len() >= 4 {
            refined = simplify_rdp_2d(&refined, true, 0.003);
        }
        if refined.len() >= 4 {
            refined =
                restore_sharp_corners_2d(&refined, cell_mm, &is_occupied, &mut refine_scratch);
        }
        if refined.len() < 3 {
            continue;
        }

        if let Some((min_idx, _)) = refined
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.x.total_cmp(&b.x).then_with(|| a.y.total_cmp(&b.y)))
        {
            refined.rotate_left(min_idx);
        }

        let mut segs = Vec::with_capacity(refined.len());
        for w in refined.windows(2) {
            if w[0].distance_squared(w[1]) > 1e-6 {
                segs.push(ArcSegment2D::Line {
                    start: w[0],
                    end: w[1],
                });
            }
        }
        if let (Some(&first), Some(&last)) = (refined.first(), refined.last())
            && first.distance_squared(last) > 1e-6
        {
            segs.push(ArcSegment2D::Line {
                start: last,
                end: first,
            });
        }
        if segs.len() >= 3 {
            polylines.push(ArcPolyline2D {
                segments: segs,
                closed: true,
            });
        }
    }

    let raw_reg = contours_to_region_set(&polylines);
    if raw_reg.is_empty() {
        return RegionSet2D::empty();
    }
    // Preserve filled outer boundaries so interior mounting holes never become floating pins
    // inside soft jaws or false-clearance cavities in flip-jigs.
    let filled_hulls: Vec<PocketRegion2D> = raw_reg
        .regions
        .into_iter()
        .map(|r| PocketRegion2D::new(r.outer_ccw, Vec::new()))
        .collect();
    RegionSet2D::from_regions(filled_hulls)
}

#[inline]
fn point_in_or_near_triangle_2d(p: DVec2, a: DVec2, b: DVec2, c: DVec2, margin_sq: f64) -> bool {
    if chip_math::point_in_triangle_2d(p, a, b, c) {
        return true;
    }
    if margin_sq <= 0.0 {
        return false;
    }
    chip_math::point_to_segment_dist_sq_2d(p, a, b) <= margin_sq
        || chip_math::point_to_segment_dist_sq_2d(p, b, c) <= margin_sq
        || chip_math::point_to_segment_dist_sq_2d(p, c, a) <= margin_sq
}

fn refine_swept_chord<F>(
    a: DVec2,
    b: DVec2,
    cell_mm: f64,
    depth: u32,
    is_occupied: &F,
    scratch: &mut Vec<usize>,
    out: &mut Vec<DVec2>,
) where
    F: Fn(DVec2, &mut Vec<usize>) -> bool,
{
    if depth >= 3 {
        return;
    }
    let seg = b - a;
    let len = seg.length();
    if len <= 0.15 {
        return;
    }
    let mid = 0.5 * (a + b);
    if !is_occupied(mid, scratch) {
        return;
    }
    // Occupied region lies to the LEFT of `a -> b`, so the outward direction is the RIGHT normal.
    let out_n = DVec2::new(seg.y / len, -seg.x / len);
    let mut step_hi = 0.02_f64;
    let max_step = cell_mm.max(0.5);
    while step_hi < max_step && is_occupied(mid + out_n * step_hi, scratch) {
        step_hi *= 2.0;
    }
    if is_occupied(mid + out_n * step_hi, scratch) {
        return;
    }
    let mut lo = 0.0_f64;
    let mut hi = step_hi;
    for _ in 0..7 {
        let m = 0.5 * (lo + hi);
        if is_occupied(mid + out_n * m, scratch) {
            lo = m;
        } else {
            hi = m;
        }
    }
    let p_safe = mid + out_n * hi;
    if p_safe.distance_squared(a) <= 1e-6 || p_safe.distance_squared(b) <= 1e-6 {
        return;
    }
    refine_swept_chord(a, p_safe, cell_mm, depth + 1, is_occupied, scratch, out);
    out.push(p_safe);
    refine_swept_chord(p_safe, b, cell_mm, depth + 1, is_occupied, scratch, out);
}

/// Collapses Marching-Squares single-cell corner bevels between adjacent long straight edges back
/// into their exact sharp intersection vertex when the true silhouette has a sharp corner (leaving
/// genuine rounded fillets untouched).
fn restore_sharp_corners_2d<F>(
    pts: &[DVec2],
    cell_mm: f64,
    is_occupied: &F,
    scratch: &mut Vec<usize>,
) -> Vec<DVec2>
where
    F: Fn(DVec2, &mut Vec<usize>) -> bool,
{
    let n = pts.len();
    if n < 5 {
        return pts.to_vec();
    }
    let min_long_len = (4.0 * cell_mm).max(1.5);
    let max_corner_dist = (2.5 * cell_mm).max(0.5);

    // Identify indices of "long" edges `i -> (i+1)%n`
    let mut long_edges: Vec<usize> = Vec::new();
    for i in 0..n {
        if pts[i].distance(pts[(i + 1) % n]) >= min_long_len {
            long_edges.push(i);
        }
    }
    if long_edges.len() < 2 {
        return pts.to_vec();
    }

    // Track corner replacements `(gap_start_vert_inclusive, gap_end_vert_inclusive, sharp_pt)`
    let mut corner_map: std::collections::HashMap<usize, (usize, DVec2)> =
        std::collections::HashMap::new();
    let mut skip_vert = vec![false; n];

    for k in 0..long_edges.len() {
        let e1 = long_edges[k];
        let e2 = long_edges[(k + 1) % long_edges.len()];
        let v_after_e1 = (e1 + 1) % n;
        let gap_verts = (e2 + n - v_after_e1) % n;
        if gap_verts == 0 || gap_verts > 6 {
            continue;
        }

        let a0 = pts[e1];
        let a1 = pts[v_after_e1];
        let b0 = pts[e2];
        let b1 = pts[(e2 + 1) % n];

        let d1 = (a1 - a0).normalize_or_zero();
        let d2 = (b1 - b0).normalize_or_zero();
        let denom = d1.x * d2.y - d1.y * d2.x;
        if denom.abs() < 0.34 {
            // Nearly collinear (< ~20° turn): not a sharp corner
            continue;
        }
        let diff = b0 - a1;
        let t1 = (diff.x * d2.y - diff.y * d2.x) / denom;
        let t2 = (diff.x * d1.y - diff.y * d1.x) / denom;
        // Intersection must lie forward of `a1` and backward of `b0` within `max_corner_dist`
        if t1 < -0.02 || t1 > max_corner_dist || t2 > 0.02 || -t2 > max_corner_dist {
            continue;
        }
        let corner = a1 + d1 * t1;

        // Verify every intermediate bevel vertex lies close to `corner`
        let mut all_near = true;
        for step in 0..=gap_verts {
            let vi = (v_after_e1 + step) % n;
            if pts[vi].distance(corner) > max_corner_dist {
                all_near = false;
                break;
            }
        }
        if !all_near {
            continue;
        }

        // Distinguish a genuine sharp corner from a true rounded fillet: an inward bisector probe
        // just 0.04 mm inside `corner` MUST lie inside the occupied swept silhouette!
        let inward = (DVec2::new(-d1.y, d1.x) + DVec2::new(-d2.y, d2.x)).normalize_or_zero();
        let probe = corner + inward * 0.04;
        if !is_occupied(corner, scratch) && !is_occupied(probe, scratch) {
            continue;
        }

        corner_map.insert(v_after_e1, (e2, corner));
        for step in 1..=gap_verts {
            let vi = (v_after_e1 + step) % n;
            skip_vert[vi] = true;
        }
    }

    if corner_map.is_empty() {
        return pts.to_vec();
    }

    let mut out = Vec::with_capacity(n);
    for (i, &p) in pts.iter().enumerate() {
        if let Some(&(_, sharp)) = corner_map.get(&i) {
            out.push(sharp);
        } else if !skip_vert[i] {
            out.push(p);
        }
    }
    if out.len() >= 3 { out } else { pts.to_vec() }
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

    let bytes = chip_cad::encode_binary_stl(header_label, &tris);
    Ok((bytes, tris.len()))
}

fn region_set_aabb(set: &RegionSet2D) -> Option<Aabb2> {
    set.exact_aabb()
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
