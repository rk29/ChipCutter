//! 3D Workholding fixtures, vises, toe/strap clamps, spoilboards, keep-out zones, and imported
//! fixture solids (`chip-cam`).
//!
//! All geometry is computed in exact `f64` millimeters in Setup WCS coordinates, producing
//! watertight [`TriMesh3D`] meshes with [`chip_bvh::FlatBvh`] spatial indexes and exact 2D XY
//! [`RegionSet2D`] keep-out footprints—strictly UI-agnostic with zero `egui`/`wgpu` dependencies.

use std::f64::consts::TAU;

use chip_bvh::TriMesh3D;
use chip_cad::CadDocument;
use chip_geom2d::{PocketRegion2D, RegionSet2D};
use chip_math::{
    Aabb3, ArcPolyline2D, ArcSegment2D, DMat3, DVec2, DVec3, DisplayUnit, EvalContext, ParamLength,
};
use serde::{Deserialize, Serialize};

use crate::{
    setup::{ResolvedSetup, Setup},
    wcs::CamError,
};

fn default_true() -> bool {
    true
}

fn default_clearance_margin() -> ParamLength {
    ParamLength::from_mm(1.0)
}

fn default_zero_length() -> ParamLength {
    ParamLength::from_mm(0.0)
}

fn default_zero_xyz() -> [ParamLength; 3] {
    [
        ParamLength::from_mm(0.0),
        ParamLength::from_mm(0.0),
        ParamLength::from_mm(0.0),
    ]
}

fn is_zero_param_length(p: &ParamLength) -> bool {
    p.expr.as_deref().is_none_or(|s| s.trim().is_empty()) && p.value_mm.abs() <= 1e-9
}

fn is_zero_xyz(v: &[ParamLength; 3]) -> bool {
    v.iter().all(is_zero_param_length)
}

fn is_zero_angles(a: &[f64; 3]) -> bool {
    a.iter().all(|v| v.abs() <= 1e-12)
}

fn is_zero_angle(v: &f64) -> bool {
    v.abs() <= 1e-12
}

fn default_z_ref_stock_top() -> FixtureZRef {
    FixtureZRef::StockTop
}

fn default_z_ref_stock_bottom() -> FixtureZRef {
    FixtureZRef::StockBottom
}

/// Principal Setup WCS axis along which vise jaws clamp the workpiece.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ViseClampAxis {
    /// Front (`-Y`) and Rear (`+Y`) jaws clamping along WCS Y (standard vertical mill vise).
    #[default]
    Y,
    /// Left (`-X`) and Right (`+X`) jaws clamping along WCS X.
    X,
}

impl ViseClampAxis {
    pub const ALL: [Self; 2] = [Self::Y, Self::X];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Y => "Y-Axis (Front / Rear Jaws)",
            Self::X => "X-Axis (Left / Right Jaws)",
        }
    }
}

/// Which side of the vise carries the stationary fixed jaw.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ViseFixedSide {
    /// Rear (`+Y`) or Right (`+X`) fixed jaw (standard Kurt-style mill vise).
    #[default]
    Positive,
    /// Front (`-Y`) or Left (`-X`) fixed jaw.
    Negative,
}

impl ViseFixedSide {
    pub const ALL: [Self; 2] = [Self::Positive, Self::Negative];

    pub const fn label(self, axis: ViseClampAxis) -> &'static str {
        match (self, axis) {
            (Self::Positive, ViseClampAxis::Y) => "Rear (+Y) Fixed Jaw",
            (Self::Negative, ViseClampAxis::Y) => "Front (-Y) Fixed Jaw",
            (Self::Positive, ViseClampAxis::X) => "Right (+X) Fixed Jaw",
            (Self::Negative, ViseClampAxis::X) => "Left (-X) Fixed Jaw",
        }
    }
}

/// Setup-relative Z datum used to position toe clamps and spoilboards parametrically.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FixtureZRef {
    #[default]
    StockTop,
    StockBottom,
    WcsOrigin,
}

impl FixtureZRef {
    pub const ALL: [Self; 3] = [Self::StockTop, Self::StockBottom, Self::WcsOrigin];

    pub const fn label(self) -> &'static str {
        match self {
            Self::StockTop => "Stock Top",
            Self::StockBottom => "Stock Bottom",
            Self::WcsOrigin => "WCS Origin (Z = 0)",
        }
    }

    #[inline]
    pub fn resolve_z_wcs(self, stock_aabb: &Aabb3) -> f64 {
        match self {
            Self::StockTop => stock_aabb.max.z,
            Self::StockBottom => stock_aabb.min.z,
            Self::WcsOrigin => 0.0,
        }
    }
}

/// Parametric or imported 3D workholding hardware specification attached to a [`Setup`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum FixtureKind {
    /// Two-jaw machinist vise with optional stepped soft-jaw shelves and under-stock parallels.
    ViseJaws {
        #[serde(default)]
        clamp_axis: ViseClampAxis,
        jaw_width: ParamLength,
        jaw_thickness: ParamLength,
        jaw_height: ParamLength,
        /// Height of stock bottom gripped upward inside the jaws (`jaw_top_z = stock_min_z + grip_depth`).
        grip_depth: ParamLength,
        /// Optional inward step-shelf depth under stock bottom (`0` = flat jaws).
        #[serde(
            default = "default_zero_length",
            skip_serializing_if = "is_zero_param_length"
        )]
        step_shelf_depth: ParamLength,
        /// Optional step-shelf vertical thickness below stock bottom (`0` = flat jaws).
        #[serde(
            default = "default_zero_length",
            skip_serializing_if = "is_zero_param_length"
        )]
        step_shelf_height: ParamLength,
        /// Include a pair of machinist parallels sitting beneath stock bottom against the inner jaw faces.
        #[serde(default)]
        include_parallels: bool,
        /// Thickness of each parallel bar (`mm`, default `3.175 mm` / `1/8 in`).
        #[serde(default = "default_parallel_thickness")]
        parallel_thickness: ParamLength,
        /// Shift jaws along the jaw-width axis relative to stock center.
        #[serde(
            default = "default_zero_length",
            skip_serializing_if = "is_zero_param_length"
        )]
        center_offset_along_jaw: ParamLength,
        #[serde(default)]
        fixed_jaw_side: ViseFixedSide,
    },
    /// Toe / strap clamp with nose overhang, heel riser block, and vertical T-slot stud + nut tower.
    ToeClamp {
        /// Clamp nose contact center in Setup WCS XY.
        anchor_xy: [ParamLength; 2],
        #[serde(default = "default_z_ref_stock_top")]
        z_reference: FixtureZRef,
        #[serde(
            default = "default_zero_length",
            skip_serializing_if = "is_zero_param_length"
        )]
        z_offset: ParamLength,
        /// Outward approach yaw angle (`deg` in Setup XY; `0°` = clamp body extends toward `+X`).
        #[serde(default, skip_serializing_if = "is_zero_angle")]
        yaw_deg: f64,
        strap_length: ParamLength,
        strap_width: ParamLength,
        strap_height: ParamLength,
        /// How far the clamp nose overlaps inward onto the stock edge.
        nose_overhang: ParamLength,
        /// Clamping stud / bolt outer diameter (`0` omits stud tower).
        stud_diameter: ParamLength,
        /// Height of clamping stud + nut protruding above the top of the strap.
        stud_height_above_strap: ParamLength,
    },
    /// Sacrificial MDF / aluminum spoilboard or tooling plate beneath the workpiece.
    Spoilboard {
        #[serde(default = "default_z_ref_stock_bottom")]
        z_top_ref: FixtureZRef,
        #[serde(
            default = "default_zero_length",
            skip_serializing_if = "is_zero_param_length"
        )]
        z_offset: ParamLength,
        thickness: ParamLength,
        /// XY overhang extending outward beyond the stock XY footprint.
        margin_xy: ParamLength,
    },
    /// Generic oriented rectangular 3D keep-out volume in Setup WCS.
    BoxKeepOut {
        center_wcs: [ParamLength; 3],
        size_wcs: [ParamLength; 3],
        #[serde(default, skip_serializing_if = "is_zero_angle")]
        yaw_deg: f64,
    },
    /// Vertical cylindrical keep-out pillar (dowel pin, bolt head, cam clamp, or rotary chuck boss).
    CylinderKeepOut {
        center_xy_wcs: [ParamLength; 2],
        z_bottom: ParamLength,
        z_top: ParamLength,
        diameter: ParamLength,
    },
    /// Imported 3D STEP/STL/OBJ/3MF fixture solid referencing a `ModelEntry` (`role = "fixture"`).
    ImportedModel {
        model_id: String,
        #[serde(default = "default_zero_xyz", skip_serializing_if = "is_zero_xyz")]
        offset_wcs: [ParamLength; 3],
        #[serde(default, skip_serializing_if = "is_zero_angles")]
        rotate_deg: [f64; 3],
    },
}

fn default_parallel_thickness() -> ParamLength {
    ParamLength::from_mm(3.175)
}

impl FixtureKind {
    pub const fn short_badge(&self) -> &'static str {
        match self {
            Self::ViseJaws { .. } => "Vise",
            Self::ToeClamp { .. } => "Clamp",
            Self::Spoilboard { .. } => "Spoilboard",
            Self::BoxKeepOut { .. } => "Box",
            Self::CylinderKeepOut { .. } => "Cylinder",
            Self::ImportedModel { .. } => "Model",
        }
    }

    pub const fn label(&self) -> &'static str {
        match self {
            Self::ViseJaws { .. } => "Vise Jaws + Parallels",
            Self::ToeClamp { .. } => "Toe / Strap Clamp",
            Self::Spoilboard { .. } => "Spoilboard / Tooling Plate",
            Self::BoxKeepOut { .. } => "Box Keep-Out Zone",
            Self::CylinderKeepOut { .. } => "Cylinder Keep-Out Pin",
            Self::ImportedModel { .. } => "Imported 3D Fixture Model",
        }
    }

    pub const fn default_color_rgba(&self) -> [f32; 4] {
        match self {
            // Machined steel gray for vise jaws
            Self::ViseJaws { .. } => [0.50, 0.56, 0.64, 0.38],
            // Anodized shop red for toe clamps
            Self::ToeClamp { .. } => [0.88, 0.26, 0.24, 0.44],
            // Warm MDF tan for spoilboards
            Self::Spoilboard { .. } => [0.78, 0.61, 0.40, 0.34],
            // High-visibility amber for keep-out volumes
            Self::BoxKeepOut { .. } | Self::CylinderKeepOut { .. } => [0.94, 0.68, 0.16, 0.38],
            // Slate blue for imported 3D fixture solids
            Self::ImportedModel { .. } => [0.40, 0.58, 0.82, 0.40],
        }
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        match self {
            Self::ViseJaws {
                jaw_width,
                jaw_thickness,
                jaw_height,
                grip_depth,
                step_shelf_depth,
                step_shelf_height,
                parallel_thickness,
                center_offset_along_jaw,
                ..
            } => {
                let _ = jaw_width.reevaluate(ctx);
                let _ = jaw_thickness.reevaluate(ctx);
                let _ = jaw_height.reevaluate(ctx);
                let _ = grip_depth.reevaluate(ctx);
                let _ = step_shelf_depth.reevaluate(ctx);
                let _ = step_shelf_height.reevaluate(ctx);
                let _ = parallel_thickness.reevaluate(ctx);
                let _ = center_offset_along_jaw.reevaluate(ctx);
            }
            Self::ToeClamp {
                anchor_xy,
                z_offset,
                strap_length,
                strap_width,
                strap_height,
                nose_overhang,
                stud_diameter,
                stud_height_above_strap,
                ..
            } => {
                for v in anchor_xy {
                    let _ = v.reevaluate(ctx);
                }
                let _ = z_offset.reevaluate(ctx);
                let _ = strap_length.reevaluate(ctx);
                let _ = strap_width.reevaluate(ctx);
                let _ = strap_height.reevaluate(ctx);
                let _ = nose_overhang.reevaluate(ctx);
                let _ = stud_diameter.reevaluate(ctx);
                let _ = stud_height_above_strap.reevaluate(ctx);
            }
            Self::Spoilboard {
                z_offset,
                thickness,
                margin_xy,
                ..
            } => {
                let _ = z_offset.reevaluate(ctx);
                let _ = thickness.reevaluate(ctx);
                let _ = margin_xy.reevaluate(ctx);
            }
            Self::BoxKeepOut {
                center_wcs,
                size_wcs,
                ..
            } => {
                for v in center_wcs {
                    let _ = v.reevaluate(ctx);
                }
                for v in size_wcs {
                    let _ = v.reevaluate(ctx);
                }
            }
            Self::CylinderKeepOut {
                center_xy_wcs,
                z_bottom,
                z_top,
                diameter,
            } => {
                for v in center_xy_wcs {
                    let _ = v.reevaluate(ctx);
                }
                let _ = z_bottom.reevaluate(ctx);
                let _ = z_top.reevaluate(ctx);
                let _ = diameter.reevaluate(ctx);
            }
            Self::ImportedModel { offset_wcs, .. } => {
                for v in offset_wcs {
                    let _ = v.reevaluate(ctx);
                }
            }
        }
    }
}

/// First-class 3D workholding fixture record persisted inside `Setup.fixtures`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FixtureBody {
    pub id: String,
    pub name: String,
    #[serde(default = "default_true")]
    pub enabled: bool,
    /// Safety clearance halo (`mm`, default `1.0 mm`) enforced for rapid avoidance & collision checks.
    #[serde(default = "default_clearance_margin")]
    pub clearance_margin: ParamLength,
    /// Permitted sacrificial overcut (`mm`, default `0.0 mm`; e.g. `0.5 mm` on a spoilboard).
    #[serde(
        default = "default_zero_length",
        skip_serializing_if = "is_zero_param_length"
    )]
    pub allowed_overcut: ParamLength,
    pub kind: FixtureKind,
    /// Optional RGBA color override for 3D viewport rendering (uses deterministic kind default if `None`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub color_rgba: Option<[f32; 4]>,
}

impl FixtureBody {
    /// Creates a standard 100 mm (4 in) Kurt-style mill vise clamping along WCS Y with parallels.
    pub fn default_vise(id: impl Into<String>, name: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            enabled: true,
            clearance_margin: default_clearance_margin(),
            allowed_overcut: default_zero_length(),
            kind: FixtureKind::ViseJaws {
                clamp_axis: ViseClampAxis::Y,
                jaw_width: ParamLength::from_mm(100.0),
                jaw_thickness: ParamLength::from_mm(18.0),
                jaw_height: ParamLength::from_mm(38.0),
                grip_depth: ParamLength::from_mm(5.0),
                step_shelf_depth: ParamLength::from_mm(0.0),
                step_shelf_height: ParamLength::from_mm(0.0),
                include_parallels: true,
                parallel_thickness: default_parallel_thickness(),
                center_offset_along_jaw: ParamLength::from_mm(0.0),
                fixed_jaw_side: ViseFixedSide::Positive,
            },
            color_rgba: None,
        }
    }

    /// Creates a standard toe/strap clamp anchored at `anchor_xy_mm` pointing outward along `yaw_deg`.
    pub fn default_toe_clamp(
        id: impl Into<String>,
        name: impl Into<String>,
        anchor_xy_mm: DVec2,
        yaw_deg: f64,
    ) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            enabled: true,
            clearance_margin: default_clearance_margin(),
            allowed_overcut: default_zero_length(),
            kind: FixtureKind::ToeClamp {
                anchor_xy: [
                    ParamLength::from_mm(anchor_xy_mm.x),
                    ParamLength::from_mm(anchor_xy_mm.y),
                ],
                z_reference: FixtureZRef::StockTop,
                z_offset: ParamLength::from_mm(0.0),
                yaw_deg,
                strap_length: ParamLength::from_mm(45.0),
                strap_width: ParamLength::from_mm(16.0),
                strap_height: ParamLength::from_mm(10.0),
                nose_overhang: ParamLength::from_mm(6.0),
                stud_diameter: ParamLength::from_mm(8.0),
                stud_height_above_strap: ParamLength::from_mm(12.0),
            },
            color_rgba: None,
        }
    }

    /// Creates a sacrificial MDF spoilboard sitting flush beneath stock bottom with `0.5 mm` overcut allowance.
    pub fn default_spoilboard(id: impl Into<String>, name: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            enabled: true,
            clearance_margin: ParamLength::from_mm(0.5),
            allowed_overcut: ParamLength::from_mm(0.5),
            kind: FixtureKind::Spoilboard {
                z_top_ref: FixtureZRef::StockBottom,
                z_offset: ParamLength::from_mm(0.0),
                thickness: ParamLength::from_mm(18.0),
                margin_xy: ParamLength::from_mm(25.0),
            },
            color_rgba: None,
        }
    }

    /// Creates a generic rectangular keep-out block in Setup WCS.
    pub fn default_box_keepout(
        id: impl Into<String>,
        name: impl Into<String>,
        center_wcs: DVec3,
    ) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            enabled: true,
            clearance_margin: default_clearance_margin(),
            allowed_overcut: default_zero_length(),
            kind: FixtureKind::BoxKeepOut {
                center_wcs: [
                    ParamLength::from_mm(center_wcs.x),
                    ParamLength::from_mm(center_wcs.y),
                    ParamLength::from_mm(center_wcs.z),
                ],
                size_wcs: [
                    ParamLength::from_mm(25.0),
                    ParamLength::from_mm(25.0),
                    ParamLength::from_mm(15.0),
                ],
                yaw_deg: 0.0,
            },
            color_rgba: None,
        }
    }

    /// Creates a cylindrical keep-out pillar (bolt head, dowel pin, or cam clamp) in Setup WCS.
    pub fn default_cylinder_keepout(
        id: impl Into<String>,
        name: impl Into<String>,
        center_xy_wcs: DVec2,
        z_bottom_mm: f64,
        z_top_mm: f64,
    ) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            enabled: true,
            clearance_margin: default_clearance_margin(),
            allowed_overcut: default_zero_length(),
            kind: FixtureKind::CylinderKeepOut {
                center_xy_wcs: [
                    ParamLength::from_mm(center_xy_wcs.x),
                    ParamLength::from_mm(center_xy_wcs.y),
                ],
                z_bottom: ParamLength::from_mm(z_bottom_mm),
                z_top: ParamLength::from_mm(z_top_mm),
                diameter: ParamLength::from_mm(14.0),
            },
            color_rgba: None,
        }
    }

    /// Creates an imported 3D fixture referencing `model_id` (`ModelRole::Fixture`).
    pub fn default_imported_model(
        id: impl Into<String>,
        name: impl Into<String>,
        model_id: impl Into<String>,
    ) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            enabled: true,
            clearance_margin: default_clearance_margin(),
            allowed_overcut: default_zero_length(),
            kind: FixtureKind::ImportedModel {
                model_id: model_id.into(),
                offset_wcs: default_zero_xyz(),
                rotate_deg: [0.0, 0.0, 0.0],
            },
            color_rgba: None,
        }
    }

    pub fn effective_color_rgba(&self) -> [f32; 4] {
        self.color_rgba
            .unwrap_or_else(|| self.kind.default_color_rgba())
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.clearance_margin.reevaluate(ctx);
        let _ = self.allowed_overcut.reevaluate(ctx);
        self.kind.reevaluate(ctx);
    }
}

/// Concrete evaluated 3D workholding solid in Setup WCS with exact `f64` [`TriMesh3D`] +
/// [`chip_bvh::FlatBvh`] spatial index and exact 2D XY keep-out footprint.
#[derive(Debug, Clone)]
pub struct ResolvedFixtureSolid {
    pub fixture_id: String,
    pub name: String,
    pub mesh_wcs: TriMesh3D,
    pub aabb_wcs: Aabb3,
    pub footprint_2d: RegionSet2D,
    pub clearance_margin_mm: f64,
    pub allowed_overcut_mm: f64,
    pub is_spoilboard: bool,
    pub color_rgba: [f32; 4],
}

/// Resolves all enabled fixtures on `setup` into exact `f64` Setup-WCS [`ResolvedFixtureSolid`]s.
///
/// `lookup_model` resolves `FixtureKind::ImportedModel { model_id, .. }` references to their
/// loaded [`CadDocument`]s (ignoring disabled fixtures).
pub fn resolve_setup_fixtures<'a, F>(
    setup: &Setup,
    resolved: &ResolvedSetup,
    display_unit: DisplayUnit,
    mut lookup_model: F,
) -> Result<Vec<ResolvedFixtureSolid>, CamError>
where
    F: FnMut(&str) -> Option<&'a CadDocument>,
{
    if setup.fixtures.is_empty() {
        return Ok(Vec::new());
    }
    let eval_ctx = resolved.eval_context(display_unit);
    let stock_aabb = resolved.stock.aabb_wcs();
    let mut out = Vec::with_capacity(setup.fixtures.len());
    for fixture in &setup.fixtures {
        if !fixture.enabled {
            continue;
        }
        out.push(resolve_fixture_body(
            fixture,
            &stock_aabb,
            &eval_ctx,
            &mut lookup_model,
        )?);
    }
    Ok(out)
}

/// Convenience wrapper for setups containing only parametric fixtures (no `ImportedModel` lookup).
pub fn resolve_setup_fixtures_parametric(
    setup: &Setup,
    resolved: &ResolvedSetup,
    display_unit: DisplayUnit,
) -> Result<Vec<ResolvedFixtureSolid>, CamError> {
    resolve_setup_fixtures(setup, resolved, display_unit, |_| None)
}

/// Resolves a single [`FixtureBody`] into an exact `f64` Setup-WCS [`ResolvedFixtureSolid`].
pub fn resolve_fixture_body<'a, F>(
    fixture: &FixtureBody,
    stock_aabb: &Aabb3,
    eval_ctx: &EvalContext,
    lookup_model: &mut F,
) -> Result<ResolvedFixtureSolid, CamError>
where
    F: FnMut(&str) -> Option<&'a CadDocument>,
{
    let mut eval_fix = fixture.clone();
    eval_fix.reevaluate(eval_ctx);

    let fname = eval_fix.name.as_str();
    let clearance_margin_mm = validate_non_negative(
        fname,
        "clearance_margin",
        eval_fix.clearance_margin.value_mm,
    )?;
    let allowed_overcut_mm =
        validate_non_negative(fname, "allowed_overcut", eval_fix.allowed_overcut.value_mm)?;

    let (mesh_wcs, footprint_2d) = match &eval_fix.kind {
        FixtureKind::ViseJaws {
            clamp_axis,
            jaw_width,
            jaw_thickness,
            jaw_height,
            grip_depth,
            step_shelf_depth,
            step_shelf_height,
            include_parallels,
            parallel_thickness,
            center_offset_along_jaw,
            fixed_jaw_side,
        } => build_vise_jaws_geometry(
            fname,
            stock_aabb,
            ViseBuildParams {
                clamp_axis: *clamp_axis,
                jaw_width: validate_positive(fname, "jaw_width", jaw_width.value_mm)?,
                jaw_thickness: validate_positive(fname, "jaw_thickness", jaw_thickness.value_mm)?,
                jaw_height: validate_positive(fname, "jaw_height", jaw_height.value_mm)?,
                grip_depth: validate_positive(fname, "grip_depth", grip_depth.value_mm)?,
                step_shelf_depth: validate_non_negative(
                    fname,
                    "step_shelf_depth",
                    step_shelf_depth.value_mm,
                )?,
                step_shelf_height: validate_non_negative(
                    fname,
                    "step_shelf_height",
                    step_shelf_height.value_mm,
                )?,
                include_parallels: *include_parallels,
                parallel_thickness: validate_non_negative(
                    fname,
                    "parallel_thickness",
                    parallel_thickness.value_mm,
                )?,
                center_offset_along_jaw: validate_finite(
                    fname,
                    "center_offset_along_jaw",
                    center_offset_along_jaw.value_mm,
                )?,
                fixed_jaw_side: *fixed_jaw_side,
            },
        )?,
        FixtureKind::ToeClamp {
            anchor_xy,
            z_reference,
            z_offset,
            yaw_deg,
            strap_length,
            strap_width,
            strap_height,
            nose_overhang,
            stud_diameter,
            stud_height_above_strap,
        } => build_toe_clamp_geometry(
            fname,
            stock_aabb,
            ToeClampBuildParams {
                anchor_xy: DVec2::new(
                    validate_finite(fname, "anchor_xy[0]", anchor_xy[0].value_mm)?,
                    validate_finite(fname, "anchor_xy[1]", anchor_xy[1].value_mm)?,
                ),
                z_contact: z_reference.resolve_z_wcs(stock_aabb)
                    + validate_finite(fname, "z_offset", z_offset.value_mm)?,
                yaw_rad: validate_finite(fname, "yaw_deg", *yaw_deg)?.to_radians(),
                strap_length: validate_positive(fname, "strap_length", strap_length.value_mm)?,
                strap_width: validate_positive(fname, "strap_width", strap_width.value_mm)?,
                strap_height: validate_positive(fname, "strap_height", strap_height.value_mm)?,
                nose_overhang: validate_non_negative(
                    fname,
                    "nose_overhang",
                    nose_overhang.value_mm,
                )?,
                stud_diameter: validate_non_negative(
                    fname,
                    "stud_diameter",
                    stud_diameter.value_mm,
                )?,
                stud_height_above_strap: validate_non_negative(
                    fname,
                    "stud_height_above_strap",
                    stud_height_above_strap.value_mm,
                )?,
            },
        )?,
        FixtureKind::Spoilboard {
            z_top_ref,
            z_offset,
            thickness,
            margin_xy,
        } => build_spoilboard_geometry(
            fname,
            stock_aabb,
            z_top_ref.resolve_z_wcs(stock_aabb)
                + validate_finite(fname, "z_offset", z_offset.value_mm)?,
            validate_positive(fname, "thickness", thickness.value_mm)?,
            validate_non_negative(fname, "margin_xy", margin_xy.value_mm)?,
        )?,
        FixtureKind::BoxKeepOut {
            center_wcs,
            size_wcs,
            yaw_deg,
        } => build_box_keepout_geometry(
            DVec3::new(
                validate_finite(fname, "center_wcs[0]", center_wcs[0].value_mm)?,
                validate_finite(fname, "center_wcs[1]", center_wcs[1].value_mm)?,
                validate_finite(fname, "center_wcs[2]", center_wcs[2].value_mm)?,
            ),
            DVec3::new(
                validate_positive(fname, "size_wcs[0]", size_wcs[0].value_mm)?,
                validate_positive(fname, "size_wcs[1]", size_wcs[1].value_mm)?,
                validate_positive(fname, "size_wcs[2]", size_wcs[2].value_mm)?,
            ),
            validate_finite(fname, "yaw_deg", *yaw_deg)?.to_radians(),
        ),
        FixtureKind::CylinderKeepOut {
            center_xy_wcs,
            z_bottom,
            z_top,
            diameter,
        } => {
            let cx = validate_finite(fname, "center_xy_wcs[0]", center_xy_wcs[0].value_mm)?;
            let cy = validate_finite(fname, "center_xy_wcs[1]", center_xy_wcs[1].value_mm)?;
            let zb = validate_finite(fname, "z_bottom", z_bottom.value_mm)?;
            let zt = validate_finite(fname, "z_top", z_top.value_mm)?;
            if zt <= zb + 1e-6 {
                return Err(CamError::InvalidFixture {
                    fixture_name: fname.to_string(),
                    reason: format!(
                        "cylinder z_top ({zt:.3} mm) must exceed z_bottom ({zb:.3} mm)"
                    ),
                });
            }
            let dia = validate_positive(fname, "diameter", diameter.value_mm)?;
            build_cylinder_keepout_geometry(DVec2::new(cx, cy), zb, zt, dia)
        }
        FixtureKind::ImportedModel {
            model_id,
            offset_wcs,
            rotate_deg,
        } => {
            let off = DVec3::new(
                validate_finite(fname, "offset_wcs[0]", offset_wcs[0].value_mm)?,
                validate_finite(fname, "offset_wcs[1]", offset_wcs[1].value_mm)?,
                validate_finite(fname, "offset_wcs[2]", offset_wcs[2].value_mm)?,
            );
            let rot = [
                validate_finite(fname, "rotate_deg[0]", rotate_deg[0])?,
                validate_finite(fname, "rotate_deg[1]", rotate_deg[1])?,
                validate_finite(fname, "rotate_deg[2]", rotate_deg[2])?,
            ];
            let Some(cad_doc) = lookup_model(model_id.as_str()) else {
                return Err(CamError::InvalidFixture {
                    fixture_name: fname.to_string(),
                    reason: format!("referenced fixture model_id '{model_id}' is not loaded"),
                });
            };
            build_imported_model_geometry(fname, cad_doc, off, rot)?
        }
    };

    if mesh_wcs.is_empty() || !mesh_wcs.bounds().is_valid() {
        return Err(CamError::InvalidFixture {
            fixture_name: fname.to_string(),
            reason: "resolved fixture solid contains no valid triangles".to_string(),
        });
    }

    let color_rgba = eval_fix.effective_color_rgba();
    let is_spoilboard = matches!(eval_fix.kind, FixtureKind::Spoilboard { .. });
    let aabb_wcs = mesh_wcs.bounds();
    Ok(ResolvedFixtureSolid {
        fixture_id: eval_fix.id,
        name: eval_fix.name,
        mesh_wcs,
        aabb_wcs,
        footprint_2d,
        clearance_margin_mm,
        allowed_overcut_mm,
        is_spoilboard,
        color_rgba,
    })
}

fn validate_finite(fixture_name: &str, field: &'static str, v: f64) -> Result<f64, CamError> {
    if v.is_finite() {
        Ok(v)
    } else {
        Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: format!("field '{field}' must be finite (got {v})"),
        })
    }
}

fn validate_positive(fixture_name: &str, field: &'static str, v: f64) -> Result<f64, CamError> {
    if v.is_finite() && v > 1e-6 {
        Ok(v)
    } else {
        Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: format!("field '{field}' must be finite and > 0 (got {v:.4} mm)"),
        })
    }
}

fn validate_non_negative(fixture_name: &str, field: &'static str, v: f64) -> Result<f64, CamError> {
    if v.is_finite() && v >= -1e-9 {
        Ok(v.max(0.0))
    } else {
        Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: format!("field '{field}' must be finite and >= 0 (got {v:.4} mm)"),
        })
    }
}

struct ViseBuildParams {
    clamp_axis: ViseClampAxis,
    jaw_width: f64,
    jaw_thickness: f64,
    jaw_height: f64,
    grip_depth: f64,
    step_shelf_depth: f64,
    step_shelf_height: f64,
    include_parallels: bool,
    parallel_thickness: f64,
    center_offset_along_jaw: f64,
    fixed_jaw_side: ViseFixedSide,
}

fn build_vise_jaws_geometry(
    fixture_name: &str,
    stock_aabb: &Aabb3,
    p: ViseBuildParams,
) -> Result<(TriMesh3D, RegionSet2D), CamError> {
    if !stock_aabb.is_valid() {
        return Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: "cannot resolve ViseJaws without valid stock bounds".to_string(),
        });
    }
    if p.jaw_height <= p.grip_depth + 0.05 {
        return Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: format!(
                "jaw_height ({:.2} mm) must exceed grip_depth ({:.2} mm)",
                p.jaw_height, p.grip_depth
            ),
        });
    }

    let stock_c = stock_aabb.center();
    let (clamp_min, clamp_max, along_center) = match p.clamp_axis {
        ViseClampAxis::Y => (
            stock_aabb.min.y,
            stock_aabb.max.y,
            stock_c.x + p.center_offset_along_jaw,
        ),
        ViseClampAxis::X => (
            stock_aabb.min.x,
            stock_aabb.max.x,
            stock_c.y + p.center_offset_along_jaw,
        ),
    };
    let clamp_span = (clamp_max - clamp_min).max(0.0);
    let along_min = along_center - 0.5 * p.jaw_width;
    let along_max = along_center + 0.5 * p.jaw_width;

    let z_stock_bot = stock_aabb.min.z;
    let z_jaw_top = z_stock_bot + p.grip_depth;
    let z_jaw_bot = z_jaw_top - p.jaw_height;

    let has_shelf = p.step_shelf_depth > 1e-4 && p.step_shelf_height > 1e-4;
    let shelf_d = if has_shelf {
        p.step_shelf_depth.min(0.45 * clamp_span)
    } else {
        0.0
    };
    let shelf_bot_z = if has_shelf {
        (z_stock_bot - p.step_shelf_height).max(z_jaw_bot)
    } else {
        z_stock_bot
    };

    let mut mb = MeshBuilder::new();
    let mut fp_sets: Vec<RegionSet2D> = Vec::new();

    // Helper: map `(clamp_coord, along_coord, z)` -> Setup WCS `DVec3`
    let to_wcs = |u_clamp: f64, v_along: f64, z: f64| -> DVec3 {
        match p.clamp_axis {
            ViseClampAxis::Y => DVec3::new(v_along, u_clamp, z),
            ViseClampAxis::X => DVec3::new(u_clamp, v_along, z),
        }
    };
    let rect_fp = |u0: f64, u1: f64, v0: f64, v1: f64| -> RegionSet2D {
        let (x0, x1, y0, y1) = match p.clamp_axis {
            ViseClampAxis::Y => (v0.min(v1), v0.max(v1), u0.min(u1), u0.max(u1)),
            ViseClampAxis::X => (u0.min(u1), u0.max(u1), v0.min(v1), v0.max(v1)),
        };
        region_from_rect(x0, y0, x1, y1)
    };

    // Build Positive (+side) and Negative (-side) jaws
    for (is_pos, face_edge, outward_sign) in
        [(true, clamp_max, 1.0_f64), (false, clamp_min, -1.0_f64)]
    {
        let is_fixed = (is_pos && p.fixed_jaw_side == ViseFixedSide::Positive)
            || (!is_pos && p.fixed_jaw_side == ViseFixedSide::Negative);
        let face_id = if is_fixed { 1 } else { 2 };

        let u_inner = face_edge;
        let u_outer = face_edge + outward_sign * p.jaw_thickness;

        if has_shelf && shelf_d > 1e-4 {
            // Watertight single-solid stepped jaw extruded along `along_min..along_max`.
            // 2D cross-section in `(r, z)` where `r = 0` at `u_inner`, `+` outward, `-` inward:
            let profile_rz: Vec<(f64, f64)> = if shelf_bot_z > z_jaw_bot + 1e-5 {
                vec![
                    (0.0, z_jaw_bot),
                    (p.jaw_thickness, z_jaw_bot),
                    (p.jaw_thickness, z_jaw_top),
                    (0.0, z_jaw_top),
                    (0.0, z_stock_bot),
                    (-shelf_d, z_stock_bot),
                    (-shelf_d, shelf_bot_z),
                    (0.0, shelf_bot_z),
                ]
            } else {
                vec![
                    (-shelf_d, z_jaw_bot),
                    (p.jaw_thickness, z_jaw_bot),
                    (p.jaw_thickness, z_jaw_top),
                    (0.0, z_jaw_top),
                    (0.0, z_stock_bot),
                    (-shelf_d, z_stock_bot),
                ]
            };
            mb.push_extruded_jaw_profile(
                &profile_rz,
                face_edge,
                outward_sign,
                along_min,
                along_max,
                p.clamp_axis,
                face_id,
            );
            let u_shelf = face_edge - outward_sign * shelf_d;
            fp_sets.push(rect_fp(u_shelf, u_outer, along_min, along_max));
        } else {
            let p_min = to_wcs(u_inner.min(u_outer), along_min, z_jaw_bot);
            let p_max = to_wcs(u_inner.max(u_outer), along_max, z_jaw_top);
            mb.push_aabb_box(p_min, p_max, face_id);
            fp_sets.push(rect_fp(u_inner, u_outer, along_min, along_max));
        }

        // Optional machinist parallels tucked under the stock bottom against each jaw face
        if p.include_parallels && p.parallel_thickness > 1e-4 && z_stock_bot > z_jaw_bot + 0.5 {
            let par_top_z = if has_shelf && shelf_d > 1e-4 {
                shelf_bot_z
            } else {
                z_stock_bot
            };
            if par_top_z > z_jaw_bot + 0.5 {
                let par_t = p.parallel_thickness.min(0.45 * clamp_span);
                let par_u0 = face_edge;
                let par_u1 = face_edge - outward_sign * par_t;
                let p_min = to_wcs(par_u0.min(par_u1), along_min, z_jaw_bot);
                let p_max = to_wcs(par_u0.max(par_u1), along_max, par_top_z);
                mb.push_aabb_box(p_min, p_max, 3);
                fp_sets.push(rect_fp(par_u0, par_u1, along_min, along_max));
            }
        }
    }

    Ok((mb.finish(), RegionSet2D::union_many(&mut fp_sets)))
}

struct ToeClampBuildParams {
    anchor_xy: DVec2,
    z_contact: f64,
    yaw_rad: f64,
    strap_length: f64,
    strap_width: f64,
    strap_height: f64,
    nose_overhang: f64,
    stud_diameter: f64,
    stud_height_above_strap: f64,
}

fn build_toe_clamp_geometry(
    fixture_name: &str,
    stock_aabb: &Aabb3,
    p: ToeClampBuildParams,
) -> Result<(TriMesh3D, RegionSet2D), CamError> {
    if p.nose_overhang >= p.strap_length - 1e-4 {
        return Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: format!(
                "nose_overhang ({:.2} mm) must be smaller than strap_length ({:.2} mm)",
                p.nose_overhang, p.strap_length
            ),
        });
    }

    let (sin_y, cos_y) = p.yaw_rad.sin_cos();
    let u_dir = DVec2::new(cos_y, sin_y); // Outward along clamp strap away from stock
    let v_dir = DVec2::new(-sin_y, cos_y);

    let u_front = -p.nose_overhang;
    let u_back = p.strap_length - p.nose_overhang;
    let half_w = 0.5 * p.strap_width;

    let mut mb = MeshBuilder::new();
    let mut fp_sets = Vec::new();

    // 1. Strap bar body (u in [u_front, u_back], v in [-half_w, half_w], z in [z_contact, z_contact + strap_height])
    let strap_center_u = 0.5 * (u_front + u_back);
    let strap_center_xy = p.anchor_xy + u_dir * strap_center_u;
    mb.push_oriented_box(
        DVec3::new(
            strap_center_xy.x,
            strap_center_xy.y,
            p.z_contact + 0.5 * p.strap_height,
        ),
        DVec3::new(p.strap_length, p.strap_width, p.strap_height),
        p.yaw_rad,
        1,
    );
    fp_sets.push(region_from_oriented_rect(
        strap_center_xy,
        p.strap_length,
        p.strap_width,
        p.yaw_rad,
    ));

    // 2. Heel / step-block riser supporting rear end of strap down to stock bottom
    let riser_bot_z = if stock_aabb.is_valid() {
        stock_aabb.min.z
    } else {
        p.z_contact - 15.0
    };
    if p.z_contact > riser_bot_z + 0.5 {
        let heel_len = (p.strap_length * 0.30).clamp(4.0, 18.0).min(u_back);
        if heel_len > 0.5 {
            let heel_u_center = u_back - 0.5 * heel_len;
            let heel_xy = p.anchor_xy + u_dir * heel_u_center;
            let heel_h = p.z_contact - riser_bot_z;
            mb.push_oriented_box(
                DVec3::new(heel_xy.x, heel_xy.y, riser_bot_z + 0.5 * heel_h),
                DVec3::new(heel_len, p.strap_width, heel_h),
                p.yaw_rad,
                2,
            );
        }
    }

    // 3. Clamping T-slot stud + nut tower protruding above strap top
    if p.stud_diameter > 1e-4 && p.stud_height_above_strap > 1e-4 {
        let stud_r = 0.5 * p.stud_diameter;
        let stud_u = (0.45 * u_back).clamp(stud_r.min(0.5 * u_back), (u_back - stud_r).max(0.0));
        let stud_xy = p.anchor_xy + u_dir * stud_u;
        let z0 = p.z_contact + p.strap_height;
        let z1 = z0 + p.stud_height_above_strap;
        mb.push_cylinder_z(stud_xy, z0, z1, stud_r, 24, 3);
        fp_sets.push(RegionSet2D::from_circle(stud_xy, stud_r));
    }

    let _ = (v_dir, half_w);
    Ok((mb.finish(), RegionSet2D::union_many(&mut fp_sets)))
}

fn build_spoilboard_geometry(
    fixture_name: &str,
    stock_aabb: &Aabb3,
    z_top: f64,
    thickness: f64,
    margin_xy: f64,
) -> Result<(TriMesh3D, RegionSet2D), CamError> {
    if !stock_aabb.is_valid() {
        return Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: "cannot resolve Spoilboard without valid stock bounds".to_string(),
        });
    }
    let min = DVec3::new(
        stock_aabb.min.x - margin_xy,
        stock_aabb.min.y - margin_xy,
        z_top - thickness,
    );
    let max = DVec3::new(
        stock_aabb.max.x + margin_xy,
        stock_aabb.max.y + margin_xy,
        z_top,
    );
    let mut mb = MeshBuilder::new();
    mb.push_aabb_box(min, max, 1);
    let fp = region_from_rect(min.x, min.y, max.x, max.y);
    Ok((mb.finish(), fp))
}

fn build_box_keepout_geometry(
    center_wcs: DVec3,
    size_wcs: DVec3,
    yaw_rad: f64,
) -> (TriMesh3D, RegionSet2D) {
    let mut mb = MeshBuilder::new();
    mb.push_oriented_box(center_wcs, size_wcs, yaw_rad, 1);
    let fp = region_from_oriented_rect(
        DVec2::new(center_wcs.x, center_wcs.y),
        size_wcs.x,
        size_wcs.y,
        yaw_rad,
    );
    (mb.finish(), fp)
}

fn build_cylinder_keepout_geometry(
    center_xy: DVec2,
    z_bottom: f64,
    z_top: f64,
    diameter: f64,
) -> (TriMesh3D, RegionSet2D) {
    let radius = 0.5 * diameter;
    let mut mb = MeshBuilder::new();
    mb.push_cylinder_z(center_xy, z_bottom, z_top, radius, 32, 1);
    let fp = RegionSet2D::from_circle(center_xy, radius);
    (mb.finish(), fp)
}

fn build_imported_model_geometry(
    fixture_name: &str,
    cad_doc: &CadDocument,
    offset_wcs: DVec3,
    rotate_deg: [f64; 3],
) -> Result<(TriMesh3D, RegionSet2D), CamError> {
    let rot_mat = DMat3::from_rotation_z(rotate_deg[2].to_radians())
        * DMat3::from_rotation_y(rotate_deg[1].to_radians())
        * DMat3::from_rotation_x(rotate_deg[0].to_radians());
    let xform = |p: DVec3| rot_mat * p + offset_wcs;

    let mut vertices: Vec<DVec3> = Vec::new();
    let mut indexed_faces: Vec<([u32; 3], u32)> = Vec::new();

    if !cad_doc.meshes.is_empty() {
        for (m_idx, mesh_body) in cad_doc.meshes.iter().enumerate() {
            let Ok(base) = u32::try_from(vertices.len()) else {
                break;
            };
            vertices.reserve(mesh_body.vertices_mm.len());
            for &v in &mesh_body.vertices_mm {
                vertices.push(xform(v));
            }
            indexed_faces.reserve(mesh_body.triangles.len());
            for tri in &mesh_body.triangles {
                indexed_faces.push((
                    [
                        base + tri.indices[0],
                        base + tri.indices[1],
                        base + tri.indices[2],
                    ],
                    (m_idx + 1) as u32,
                ));
            }
        }
    } else if !cad_doc.render_mesh.indices.is_empty() {
        let rm = &cad_doc.render_mesh;
        vertices.reserve(rm.vertices.len());
        for v in &rm.vertices {
            let p = DVec3::new(
                f64::from(v.position_mm[0]),
                f64::from(v.position_mm[1]),
                f64::from(v.position_mm[2]),
            );
            vertices.push(xform(p));
        }
        indexed_faces.reserve(rm.indices.len() / 3);
        for &[i0, i1, i2] in rm.indices.as_chunks::<3>().0 {
            let fid = rm.vertices.get(i0 as usize).map_or(0, |v| v.face_id);
            indexed_faces.push(([i0, i1, i2], fid));
        }
    }

    if indexed_faces.is_empty() {
        return Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: format!(
                "imported fixture model '{}' contains no triangles",
                cad_doc.source_name
            ),
        });
    }

    let pts_2d: Vec<DVec2> = vertices
        .iter()
        .filter(|v| v.is_finite())
        .map(|v| DVec2::new(v.x, v.y))
        .collect();
    let mesh = TriMesh3D::from_indexed_with_faces(vertices, &indexed_faces);
    let footprint = region_from_convex_hull_2d(&pts_2d).unwrap_or_else(|| {
        let b = mesh.bounds();
        region_from_rect(b.min.x, b.min.y, b.max.x, b.max.y)
    });
    Ok((mesh, footprint))
}

// ============================================================================
// Exact Watertight Parametric Mesh Builder
// ============================================================================

struct MeshBuilder {
    vertices: Vec<DVec3>,
    indexed_faces: Vec<([u32; 3], u32)>,
}

impl MeshBuilder {
    fn new() -> Self {
        Self {
            vertices: Vec::new(),
            indexed_faces: Vec::new(),
        }
    }

    fn push_aabb_box(&mut self, min: DVec3, max: DVec3, face_id: u32) {
        let Ok(base) = u32::try_from(self.vertices.len()) else {
            return;
        };
        let corners = [
            DVec3::new(min.x, min.y, min.z), // 0
            DVec3::new(max.x, min.y, min.z), // 1
            DVec3::new(max.x, max.y, min.z), // 2
            DVec3::new(min.x, max.y, min.z), // 3
            DVec3::new(min.x, min.y, max.z), // 4
            DVec3::new(max.x, min.y, max.z), // 5
            DVec3::new(max.x, max.y, max.z), // 6
            DVec3::new(min.x, max.y, max.z), // 7
        ];
        self.vertices.extend_from_slice(&corners);
        // Strictly outward-facing CCW winding
        let tris: [[u32; 3]; 12] = [
            // Bottom (-Z)
            [0, 2, 1],
            [0, 3, 2],
            // Top (+Z)
            [4, 5, 6],
            [4, 6, 7],
            // Front (-Y)
            [0, 1, 5],
            [0, 5, 4],
            // Back (+Y)
            [3, 7, 6],
            [3, 6, 2],
            // Left (-X)
            [0, 4, 7],
            [0, 7, 3],
            // Right (+X)
            [1, 2, 6],
            [1, 6, 5],
        ];
        for [a, b, c] in tris {
            self.indexed_faces
                .push(([base + a, base + b, base + c], face_id));
        }
    }

    fn push_oriented_box(&mut self, center: DVec3, size: DVec3, yaw_rad: f64, face_id: u32) {
        let Ok(base) = u32::try_from(self.vertices.len()) else {
            return;
        };
        let (s, c) = yaw_rad.sin_cos();
        let hx = 0.5 * size.x;
        let hy = 0.5 * size.y;
        let hz = 0.5 * size.z;
        let local = [
            DVec3::new(-hx, -hy, -hz),
            DVec3::new(hx, -hy, -hz),
            DVec3::new(hx, hy, -hz),
            DVec3::new(-hx, hy, -hz),
            DVec3::new(-hx, -hy, hz),
            DVec3::new(hx, -hy, hz),
            DVec3::new(hx, hy, hz),
            DVec3::new(-hx, hy, hz),
        ];
        for lp in local {
            let wp = DVec3::new(
                center.x + lp.x * c - lp.y * s,
                center.y + lp.x * s + lp.y * c,
                center.z + lp.z,
            );
            self.vertices.push(wp);
        }
        let tris: [[u32; 3]; 12] = [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [3, 7, 6],
            [3, 6, 2],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ];
        for [a, b, c] in tris {
            self.indexed_faces
                .push(([base + a, base + b, base + c], face_id));
        }
    }

    /// Extrudes a simple CCW 2D cross-section polygon `profile_rz` (in `(r, z)` where
    /// `u_clamp = face_edge + outward_sign * r`) along `along_min..along_max` into a 100%
    /// watertight manifold mesh with strictly outward normals.
    #[allow(clippy::too_many_arguments)]
    fn push_extruded_jaw_profile(
        &mut self,
        profile_rz: &[(f64, f64)],
        face_edge: f64,
        outward_sign: f64,
        along_min: f64,
        along_max: f64,
        clamp_axis: ViseClampAxis,
        face_id: u32,
    ) {
        let n = profile_rz.len();
        if n < 3 {
            return;
        }
        let Ok(base) = u32::try_from(self.vertices.len()) else {
            return;
        };
        let to_wcs = |r: f64, z: f64, v_along: f64| -> DVec3 {
            let u = face_edge + outward_sign * r;
            match clamp_axis {
                ViseClampAxis::Y => DVec3::new(v_along, u, z),
                ViseClampAxis::X => DVec3::new(u, v_along, z),
            }
        };

        // Determine orientation parity of the mapping `(r, z, v_along) -> (x, y, z)`:
        // - For `ViseClampAxis::Y`: e_r = (0, s, 0), e_z = (0, 0, 1), e_v = (1, 0, 0) -> det = +s.
        // - For `ViseClampAxis::X`: e_r = (s, 0, 0), e_z = (0, 0, 1), e_v = (0, 1, 0) -> det = -s.
        let right_handed = match clamp_axis {
            ViseClampAxis::Y => outward_sign > 0.0,
            ViseClampAxis::X => outward_sign < 0.0,
        };

        // Front ring (`v = along_min`, indices `base..base+n`) and Back ring (`v = along_max`, indices `base+n..base+2n`)
        for &(r, z) in profile_rz {
            self.vertices.push(to_wcs(r, z, along_min));
        }
        for &(r, z) in profile_rz {
            self.vertices.push(to_wcs(r, z, along_max));
        }

        let n_u32 = n as u32;
        // Side quads around extruded perimeter
        for i in 0..n_u32 {
            let j = (i + 1) % n_u32;
            let f0 = base + i;
            let f1 = base + j;
            let b0 = base + n_u32 + i;
            let b1 = base + n_u32 + j;
            if right_handed {
                // Outward normal for CCW `(r, z)` extruded along `+e_v` points along `(e_r_edge) x (+e_v)` -> (f0, f1, b1), (f0, b1, b0)
                self.indexed_faces.push(([f0, f1, b1], face_id));
                self.indexed_faces.push(([f0, b1, b0], face_id));
            } else {
                self.indexed_faces.push(([f0, b1, f1], face_id));
                self.indexed_faces.push(([f0, b0, b1], face_id));
            }
        }

        // End caps via exact ear-clipping triangulation of simple CCW 2D polygon `profile_rz`
        let cap_tris = ear_clip_ccw_2d(profile_rz);
        for [i0, i1, i2] in cap_tris {
            let a = i0 as u32;
            let b = i1 as u32;
            let c = i2 as u32;
            if right_handed {
                // Front cap (`along_min`, outward normal `-e_v`) -> CW in `(r, z)`
                self.indexed_faces
                    .push(([base + a, base + c, base + b], face_id));
                // Back cap (`along_max`, outward normal `+e_v`) -> CCW in `(r, z)`
                self.indexed_faces.push((
                    [base + n_u32 + a, base + n_u32 + b, base + n_u32 + c],
                    face_id,
                ));
            } else {
                self.indexed_faces
                    .push(([base + a, base + b, base + c], face_id));
                self.indexed_faces.push((
                    [base + n_u32 + a, base + n_u32 + c, base + n_u32 + b],
                    face_id,
                ));
            }
        }
    }

    fn push_cylinder_z(
        &mut self,
        center_xy: DVec2,
        z_min: f64,
        z_max: f64,
        radius: f64,
        segs: usize,
        face_id: u32,
    ) {
        let segs = segs.max(8);
        let Ok(base) = u32::try_from(self.vertices.len()) else {
            return;
        };
        let n = segs as u32;
        // Bottom ring: base .. base + n
        // Top ring:    base + n .. base + 2*n
        // Bottom ctr:  base + 2*n
        // Top ctr:     base + 2*n + 1
        for k in 0..segs {
            let th = TAU * (k as f64) / (segs as f64);
            let x = center_xy.x + radius * th.cos();
            let y = center_xy.y + radius * th.sin();
            self.vertices.push(DVec3::new(x, y, z_min));
        }
        for k in 0..segs {
            let th = TAU * (k as f64) / (segs as f64);
            let x = center_xy.x + radius * th.cos();
            let y = center_xy.y + radius * th.sin();
            self.vertices.push(DVec3::new(x, y, z_max));
        }
        let c_bot = base + 2 * n;
        let c_top = base + 2 * n + 1;
        self.vertices
            .push(DVec3::new(center_xy.x, center_xy.y, z_min));
        self.vertices
            .push(DVec3::new(center_xy.x, center_xy.y, z_max));

        for k in 0..n {
            let kn = (k + 1) % n;
            let b0 = base + k;
            let b1 = base + kn;
            let t0 = base + n + k;
            let t1 = base + n + kn;
            // Side wall (outward normal)
            self.indexed_faces.push(([b0, b1, t1], face_id));
            self.indexed_faces.push(([b0, t1, t0], face_id));
            // Bottom cap (-Z normal)
            self.indexed_faces.push(([c_bot, b1, b0], face_id));
            // Top cap (+Z normal)
            self.indexed_faces.push(([c_top, t0, t1], face_id));
        }
    }

    fn finish(self) -> TriMesh3D {
        TriMesh3D::from_indexed_with_faces(self.vertices, &self.indexed_faces)
    }
}

/// Exact ear-clipping triangulation for a simple CCW 2D polygon (`(x, y)` pairs).
fn ear_clip_ccw_2d(pts: &[(f64, f64)]) -> Vec<[usize; 3]> {
    let n = pts.len();
    if n < 3 {
        return Vec::new();
    }
    let mut rem: Vec<usize> = (0..n).collect();
    let mut tris = Vec::with_capacity(n.saturating_sub(2));

    let cross2 = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| -> f64 {
        (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
    };
    let point_in_tri = |p: (f64, f64), a: (f64, f64), b: (f64, f64), c: (f64, f64)| -> bool {
        let c1 = cross2(a, b, p);
        let c2 = cross2(b, c, p);
        let c3 = cross2(c, a, p);
        c1 >= -1e-10 && c2 >= -1e-10 && c3 >= -1e-10
    };

    let mut guard = n * n + 4;
    while rem.len() > 3 && guard > 0 {
        guard -= 1;
        let m = rem.len();
        let mut clipped = false;
        for i in 0..m {
            let prev = rem[(i + m - 1) % m];
            let curr = rem[i];
            let next = rem[(i + 1) % m];
            let a = pts[prev];
            let b = pts[curr];
            let c = pts[next];
            if cross2(a, b, c) <= 1e-10 {
                continue;
            }
            let any_inside = rem.iter().copied().any(|idx| {
                idx != prev && idx != curr && idx != next && point_in_tri(pts[idx], a, b, c)
            });
            if !any_inside {
                tris.push([prev, curr, next]);
                rem.remove(i);
                clipped = true;
                break;
            }
        }
        if !clipped {
            break;
        }
    }
    if rem.len() == 3 {
        tris.push([rem[0], rem[1], rem[2]]);
    }
    tris
}

// ============================================================================
// Exact 2D Keep-Out Footprint Helpers (`RegionSet2D`)
// ============================================================================

fn region_from_rect(x0: f64, y0: f64, x1: f64, y1: f64) -> RegionSet2D {
    let min_x = x0.min(x1);
    let max_x = x0.max(x1);
    let min_y = y0.min(y1);
    let max_y = y0.max(y1);
    if max_x - min_x <= 1e-5 || max_y - min_y <= 1e-5 {
        return RegionSet2D::empty();
    }
    region_from_ccw_ring(&[
        DVec2::new(min_x, min_y),
        DVec2::new(max_x, min_y),
        DVec2::new(max_x, max_y),
        DVec2::new(min_x, max_y),
    ])
}

fn region_from_oriented_rect(center: DVec2, size_x: f64, size_y: f64, yaw_rad: f64) -> RegionSet2D {
    let (s, c) = yaw_rad.sin_cos();
    let hx = 0.5 * size_x;
    let hy = 0.5 * size_y;
    let rot = |lx: f64, ly: f64| DVec2::new(center.x + lx * c - ly * s, center.y + lx * s + ly * c);
    region_from_ccw_ring(&[rot(-hx, -hy), rot(hx, -hy), rot(hx, hy), rot(-hx, hy)])
}

fn region_from_ccw_ring(pts: &[DVec2]) -> RegionSet2D {
    if pts.len() < 3 {
        return RegionSet2D::empty();
    }
    let mut segs = Vec::with_capacity(pts.len());
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        if a.distance_squared(b) > 1e-10 {
            segs.push(ArcSegment2D::Line { start: a, end: b });
        }
    }
    if segs.len() < 3 {
        return RegionSet2D::empty();
    }
    let poly = ArcPolyline2D {
        segments: segs,
        closed: true,
    }
    .ensure_orientation(true);
    RegionSet2D::from_single(PocketRegion2D::new(poly, Vec::new()))
}

/// Computes the exact 2D CCW convex hull of `points` via Andrew's monotone chain (`O(N log N)`).
fn region_from_convex_hull_2d(points: &[DVec2]) -> Option<RegionSet2D> {
    if points.len() < 3 {
        return None;
    }
    let mut pts: Vec<DVec2> = points.iter().copied().filter(|p| p.is_finite()).collect();
    pts.sort_by(|a, b| a.x.total_cmp(&b.x).then_with(|| a.y.total_cmp(&b.y)));
    pts.dedup_by(|a, b| a.distance_squared(*b) <= 1e-10);
    if pts.len() < 3 {
        return None;
    }

    let cross =
        |o: DVec2, a: DVec2, b: DVec2| (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    let mut lower: Vec<DVec2> = Vec::with_capacity(pts.len());
    for &p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 1e-9 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<DVec2> = Vec::with_capacity(pts.len());
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 1e-9 {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    if lower.len() < 3 {
        return None;
    }
    let reg = region_from_ccw_ring(&lower);
    (!reg.is_empty()).then_some(reg)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::collections::HashMap;

    use chip_cad::{
        CadKernel, MeshImportOptions, MonstertruckKernel, load_mesh_cad_document,
        sample_machined_plate_step, sample_sculpted_dome_stl_bytes,
    };
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, approx_eq};

    use super::*;
    use crate::{
        stock::{FixedBoxXyAlign, FixedBoxZAlign, StockConfig},
        wcs::{BoxAnchor27, WcsOriginMode, WorkOffsetSlot},
    };

    /// Asserts that `mesh` is a strictly watertight, manifold, consistently outward-oriented solid:
    /// every undirected edge has exactly 2 opposite half-edges, and signed volume > 0.
    fn assert_watertight_outward_solid(mesh: &TriMesh3D, label: &str) {
        assert!(!mesh.is_empty(), "{label}: mesh is empty");
        let mut half_edges: HashMap<(u32, u32), usize> = HashMap::new();
        let mut undir_edges: HashMap<(u32, u32), usize> = HashMap::new();
        let mut signed_vol6 = 0.0_f64;

        for tri in &mesh.triangles {
            let [i0, i1, i2] = tri.indices;
            let v0 = mesh.vertices[i0 as usize];
            let v1 = mesh.vertices[i1 as usize];
            let v2 = mesh.vertices[i2 as usize];
            signed_vol6 += v0.dot(v1.cross(v2));

            for (a, b) in [(i0, i1), (i1, i2), (i2, i0)] {
                *half_edges.entry((a, b)).or_insert(0) += 1;
                let key = if a < b { (a, b) } else { (b, a) };
                *undir_edges.entry(key).or_insert(0) += 1;
            }
        }

        for (&(a, b), &count) in &undir_edges {
            assert_eq!(
                count, 2,
                "{label}: edge ({a}, {b}) has incident count {count} (expected 2 for watertight manifold)"
            );
            assert_eq!(
                half_edges.get(&(a, b)).copied().unwrap_or(0),
                1,
                "{label}: half-edge ({a}->{b}) orientation mismatch"
            );
            assert_eq!(
                half_edges.get(&(b, a)).copied().unwrap_or(0),
                1,
                "{label}: half-edge ({b}->{a}) orientation mismatch"
            );
        }

        assert!(
            signed_vol6 > 1e-3,
            "{label}: expected positive outward signed volume, got {}",
            signed_vol6 / 6.0
        );
    }

    fn make_box_setup(wx: f64, wy: f64, wz: f64) -> (Setup, ResolvedSetup) {
        let mut setup = Setup::new("s1", "Test Setup", WorkOffsetSlot::G54);
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(wx),
            depth_y: ParamLength::from_mm(wy),
            height_z: ParamLength::from_mm(wz),
            xy_align: FixedBoxXyAlign::CenterXy,
            z_align: FixedBoxZAlign::CenterZ,
        };
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        let resolved = setup
            .resolve_immutable(&CadDocument::empty(), DisplayUnit::Millimeter)
            .unwrap();
        // Stock spans X: [-wx/2, wx/2], Y: [-wy/2, wy/2], Z: [-wz, 0]
        (setup, resolved)
    }

    #[test]
    fn vise_jaws_flat_stepped_and_parallels_are_watertight_with_exact_bounds_and_raycasts() {
        let (mut setup, resolved) = make_box_setup(120.0, 100.0, 20.0);

        // 1. Flat jaws + parallels (Y clamp axis)
        let vise_flat = FixtureBody::default_vise("fix_vise_1", "4in Kurt Vise");
        setup.fixtures = vec![vise_flat];
        let solids =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();
        assert_eq!(solids.len(), 1);
        let s = &solids[0];
        assert_watertight_outward_solid(&s.mesh_wcs, "flat vise + parallels");
        // X: [-50, 50], Y: [-50 - 18, 50 + 18] = [-68, 68], Z: [-20 + 5 - 38, -20 + 5] = [-53, -15]
        assert!(approx_eq(s.aabb_wcs.min.x, -50.0, 1e-6));
        assert!(approx_eq(s.aabb_wcs.max.x, 50.0, 1e-6));
        assert!(approx_eq(s.aabb_wcs.min.y, -68.0, 1e-6));
        assert!(approx_eq(s.aabb_wcs.max.y, 68.0, 1e-6));
        assert!(approx_eq(s.aabb_wcs.min.z, -53.0, 1e-6));
        assert!(approx_eq(s.aabb_wcs.max.z, -15.0, 1e-6));

        // Downward ray onto +Y jaw top at (0, 59, 10) hits at z = -15.0 (t = 25.0)
        let hit_jaw = s
            .mesh_wcs
            .raycast(DVec3::new(0.0, 59.0, 10.0), DVec3::NEG_Z, f64::INFINITY)
            .unwrap();
        assert!(approx_eq(hit_jaw.point.z, -15.0, 1e-9));
        // Downward ray onto parallel inside +Y jaw at (0, 48.5, 10) hits at stock bottom z = -20.0
        let hit_par = s
            .mesh_wcs
            .raycast(DVec3::new(0.0, 48.5, 10.0), DVec3::NEG_Z, f64::INFINITY)
            .unwrap();
        assert!(approx_eq(hit_par.point.z, -20.0, 1e-9));
        // Downward ray through open vise center (0, 0, 10) misses completely
        assert!(
            s.mesh_wcs
                .raycast(DVec3::new(0.0, 0.0, 10.0), DVec3::NEG_Z, f64::INFINITY)
                .is_none()
        );
        assert!(s.footprint_2d.contains_point(DVec2::new(0.0, 59.0)));
        assert!(!s.footprint_2d.contains_point(DVec2::new(0.0, 0.0)));

        // 2. Stepped soft jaws (X clamp axis, with step shelf & no parallels, plus expression jaw_width)
        let mut stepped = FixtureBody::default_vise("fix_vise_2", "Stepped Soft Jaws");
        if let FixtureKind::ViseJaws {
            clamp_axis,
            jaw_width,
            step_shelf_depth,
            step_shelf_height,
            include_parallels,
            ..
        } = &mut stepped.kind
        {
            *clamp_axis = ViseClampAxis::X;
            *jaw_width = ParamLength {
                value_mm: 50.0,
                expr: Some("stock_y + 20 mm".to_string()),
            };
            *step_shelf_depth = ParamLength::from_mm(4.0);
            *step_shelf_height = ParamLength::from_mm(6.0);
            *include_parallels = false;
        }
        setup.fixtures = vec![stepped];
        let solids_step =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();
        let ss = &solids_step[0];
        assert_watertight_outward_solid(&ss.mesh_wcs, "stepped soft jaws X-axis");
        // Jaw width evaluated from `stock_y + 20 mm` = 120 mm -> Y in [-60, 60]
        assert!(approx_eq(ss.aabb_wcs.min.y, -60.0, 1e-6));
        assert!(approx_eq(ss.aabb_wcs.max.y, 60.0, 1e-6));
        // X spans [-60 - 18, 60 + 18] = [-78, 78]
        assert!(approx_eq(ss.aabb_wcs.min.x, -78.0, 1e-6));
        assert!(approx_eq(ss.aabb_wcs.max.x, 78.0, 1e-6));
        // Ray on step shelf at (58, 0, 10) hits at stock bottom z = -20.0!
        let hit_shelf = ss
            .mesh_wcs
            .raycast(DVec3::new(58.0, 0.0, 10.0), DVec3::NEG_Z, f64::INFINITY)
            .unwrap();
        assert!(approx_eq(hit_shelf.point.z, -20.0, 1e-9));
    }

    #[test]
    fn toe_clamp_rotated_cardinals_is_watertight_and_catches_stud_tower() {
        let (mut setup, resolved) = make_box_setup(120.0, 100.0, 20.0);

        for (idx, (anchor, yaw_deg)) in [
            (DVec2::new(60.0, 0.0), 0.0),
            (DVec2::new(0.0, 50.0), 90.0),
            (DVec2::new(-60.0, 0.0), 180.0),
            (DVec2::new(0.0, -50.0), 270.0),
        ]
        .into_iter()
        .enumerate()
        {
            let clamp = FixtureBody::default_toe_clamp(
                format!("clamp_{idx}"),
                format!("Clamp {yaw_deg}°"),
                anchor,
                yaw_deg,
            );
            setup.fixtures = vec![clamp];
            let solids =
                resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter)
                    .unwrap();
            let s = &solids[0];
            assert_watertight_outward_solid(&s.mesh_wcs, &format!("toe clamp {yaw_deg}°"));

            // Z spans from stock bottom (-20) up to strap (10) + stud (12) = +22.0 mm!
            assert!(approx_eq(s.aabb_wcs.min.z, -20.0, 1e-6));
            assert!(approx_eq(s.aabb_wcs.max.z, 22.0, 1e-6));

            // Ray descending directly over the stud tower catches z = +22.0 mm (above strap z = +10.0 mm!)
            let dir = DVec2::new(yaw_deg.to_radians().cos(), yaw_deg.to_radians().sin());
            let stud_u = 0.45 * (45.0 - 6.0);
            let stud_xy = anchor + dir * stud_u;
            let hit_stud = s
                .mesh_wcs
                .raycast(
                    DVec3::new(stud_xy.x, stud_xy.y, 40.0),
                    DVec3::NEG_Z,
                    f64::INFINITY,
                )
                .unwrap();
            assert!(
                approx_eq(hit_stud.point.z, 22.0, 1e-6),
                "yaw={yaw_deg}: expected stud top z=22.0, got {}",
                hit_stud.point.z
            );
            assert!(s.footprint_2d.contains_point(stud_xy));
        }
    }

    #[test]
    fn spoilboard_box_cylinder_and_imported_step_stl_resolve_cleanly() {
        let (mut setup, resolved) = make_box_setup(120.0, 100.0, 20.0);

        let sb = FixtureBody::default_spoilboard("sb_1", "MDF Spoilboard");
        let mut bx = FixtureBody::default_box_keepout(
            "bx_1",
            "Rotated Stop Block",
            DVec3::new(-75.0, 0.0, -5.0),
        );
        if let FixtureKind::BoxKeepOut { yaw_deg, .. } = &mut bx.kind {
            *yaw_deg = 45.0;
        }
        let cyl = FixtureBody::default_cylinder_keepout(
            "cyl_1",
            "Dowel Pin",
            DVec2::new(70.0, 20.0),
            -20.0,
            15.0,
        );
        let mut imp_step = FixtureBody::default_imported_model("imp_1", "STEP Fixture", "fix_step");
        if let FixtureKind::ImportedModel {
            offset_wcs,
            rotate_deg,
            ..
        } = &mut imp_step.kind
        {
            *offset_wcs = [
                ParamLength::from_mm(100.0),
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(-20.0),
            ];
            *rotate_deg = [0.0, 0.0, 90.0];
        }
        let imp_stl = FixtureBody::default_imported_model("imp_2", "STL Fixture", "fix_stl");

        let step_txt = sample_machined_plate_step().unwrap();
        let step_doc = MonstertruckKernel
            .load_step_bytes(
                "fixture_plate.step",
                step_txt.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .unwrap();
        let stl_bytes = sample_sculpted_dome_stl_bytes();
        let stl_doc = load_mesh_cad_document(
            "fixture_dome.stl",
            &stl_bytes,
            &MeshImportOptions::default(),
        )
        .unwrap();

        let mut docs: HashMap<&str, CadDocument> = HashMap::new();
        docs.insert("fix_step", step_doc);
        docs.insert("fix_stl", stl_doc);

        setup.fixtures = vec![sb, bx, cyl, imp_step, imp_stl];
        let solids = resolve_setup_fixtures(&setup, &resolved, DisplayUnit::Millimeter, |id| {
            docs.get(id)
        })
        .unwrap();
        assert_eq!(solids.len(), 5);

        // Spoilboard (stock X: [-60, 60] + 25 margin = [-85, 85])
        assert_watertight_outward_solid(&solids[0].mesh_wcs, "spoilboard");
        assert!(approx_eq(solids[0].aabb_wcs.min.x, -85.0, 1e-6));
        assert!(approx_eq(solids[0].aabb_wcs.max.x, 85.0, 1e-6));
        assert!(approx_eq(solids[0].aabb_wcs.max.z, -20.0, 1e-6));
        assert!(approx_eq(solids[0].aabb_wcs.min.z, -38.0, 1e-6));
        assert!(approx_eq(solids[0].allowed_overcut_mm, 0.5, 1e-9));

        // BoxKeepOut (rotated 45°)
        assert_watertight_outward_solid(&solids[1].mesh_wcs, "rotated box keep-out");
        let diag_half = 25.0 * std::f64::consts::FRAC_1_SQRT_2;
        assert!(approx_eq(solids[1].aabb_wcs.min.x, -75.0 - diag_half, 1e-6));
        assert!(approx_eq(solids[1].aabb_wcs.max.x, -75.0 + diag_half, 1e-6));
        assert!(
            solids[1]
                .footprint_2d
                .contains_point(DVec2::new(-75.0, 0.0))
        );

        // CylinderKeepOut
        assert_watertight_outward_solid(&solids[2].mesh_wcs, "cylinder keep-out");
        assert!(approx_eq(solids[2].aabb_wcs.min.z, -20.0, 1e-6));
        assert!(approx_eq(solids[2].aabb_wcs.max.z, 15.0, 1e-6));
        assert!(
            solids[2]
                .footprint_2d
                .contains_point(DVec2::new(70.0, 20.0))
        );
        assert!(
            !solids[2]
                .footprint_2d
                .contains_point(DVec2::new(80.0, 20.0))
        );

        // Imported STEP & STL fixtures
        assert!(solids[3].mesh_wcs.len() >= 12);
        assert!(solids[3].aabb_wcs.is_valid());
        assert!(!solids[3].footprint_2d.is_empty());
        assert!(solids[4].mesh_wcs.len() >= 100);
        assert!(solids[4].aabb_wcs.is_valid());
        assert!(!solids[4].footprint_2d.is_empty());
    }

    #[test]
    fn rejects_invalid_fixture_dimensions_and_missing_models() {
        let (mut setup, resolved) = make_box_setup(120.0, 100.0, 20.0);

        let mut neg_margin = FixtureBody::default_spoilboard("s1", "Bad Margin");
        neg_margin.clearance_margin = ParamLength::from_mm(-1.0);
        setup.fixtures = vec![neg_margin];
        assert!(matches!(
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter),
            Err(CamError::InvalidFixture { .. })
        ));

        let bad_cyl =
            FixtureBody::default_cylinder_keepout("c1", "Bad Cyl", DVec2::ZERO, 10.0, 5.0);
        setup.fixtures = vec![bad_cyl];
        assert!(matches!(
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter),
            Err(CamError::InvalidFixture { .. })
        ));

        let missing_imp = FixtureBody::default_imported_model("m1", "Missing", "nonexistent");
        setup.fixtures = vec![missing_imp];
        assert!(matches!(
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter),
            Err(CamError::InvalidFixture { .. })
        ));

        // Disabled invalid fixture is skipped cleanly
        setup.fixtures[0].enabled = false;
        let skipped =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();
        assert!(skipped.is_empty());
    }
}
