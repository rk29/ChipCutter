use chip_bvh::TriMesh3D;
use chip_geom2d::RegionSet2D;
use chip_math::{Aabb3, DVec2, DVec3, EvalContext, ParamLength};
use serde::{Deserialize, Serialize};

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
