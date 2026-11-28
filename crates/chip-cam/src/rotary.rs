//! Configurable 4th-axis rotary kinematics, cylindrical coordinate transforms, inverse-time (`G93`)
//! and angular (`G94`) feedrate helpers, and multi-turn / shortest-path unwinding (`03-setups-wcs.md`
//! & Milestone 19).

use chip_bvh::FlatBvhNode;
use chip_cad::{CadDocument, CurveKind, EdgeId, FaceId, LoopId, MeshTriangle};
use chip_math::{Aabb3, DVec2, DVec3, EvalContext, ParamLength, RigidFrame3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::{
    params::{RotaryAxialBoundaryConfig, RotaryAxialBoundaryMode, ToolContainmentMode},
    selection::{ExtractedFeature25D, ExtractedSelectionSummary, GeometrySelectionItem},
    setup::ResolvedSetup,
    stock::{ResolvedStockGeometry, StockAxis},
    surface3d::{SurfaceCutterProfile, SurfaceMesh3D, drop_cutter::drop_cutter_triangle},
    toolpath::{GeneratedToolpath, SegmentMotion3D},
    wcs::CamError,
};

/// Principal Setup WCS axis around which the 4th-axis rotary chuck rotates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RotaryAxisKind {
    /// Rotary axis aligned along WCS `X` (standard `A`-axis).
    #[default]
    XAxis,
    /// Rotary axis aligned along WCS `Y` (standard `B`-axis).
    YAxis,
    /// Rotary axis aligned along WCS `Z` (standard `C`-axis rotary table).
    ZAxis,
}

impl RotaryAxisKind {
    pub const ALL: [Self; 3] = [Self::XAxis, Self::YAxis, Self::ZAxis];

    pub const fn label(self) -> &'static str {
        match self {
            Self::XAxis => "X Axis (A-Axis)",
            Self::YAxis => "Y Axis (B-Axis)",
            Self::ZAxis => "Z Axis (C-Axis)",
        }
    }

    pub const fn default_letter(self) -> char {
        match self {
            Self::XAxis => 'A',
            Self::YAxis => 'B',
            Self::ZAxis => 'C',
        }
    }

    pub const fn to_stock_axis(self) -> StockAxis {
        match self {
            Self::XAxis => StockAxis::X,
            Self::YAxis => StockAxis::Y,
            Self::ZAxis => StockAxis::Z,
        }
    }

    /// Right-handed orthonormal basis `(e1, e2, u_axis)` such that `e1 × e2 = u_axis`.
    ///
    /// For `XAxis` (`A`) and `YAxis` (`B`), `e1 = +Z` so `θ = 0°` always points straight up
    /// toward the spindle. For `ZAxis` (`C`), `e1 = +X` and `e2 = +Y`.
    pub const fn basis(self) -> (DVec3, DVec3, DVec3) {
        match self {
            Self::XAxis => (
                DVec3::new(0.0, 0.0, 1.0),
                DVec3::new(0.0, -1.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
            ),
            Self::YAxis => (
                DVec3::new(0.0, 0.0, 1.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            ),
            Self::ZAxis => (
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
                DVec3::new(0.0, 0.0, 1.0),
            ),
        }
    }

    /// Unit vector along the rotary axis in Setup WCS coordinates.
    pub const fn axis_unit(self) -> DVec3 {
        self.basis().2
    }

    /// Human-readable labels for the two perpendicular centerline offset coordinates `(e1, e2)`.
    pub const fn offset_axis_labels(self) -> (&'static str, &'static str) {
        match self {
            Self::XAxis => ("Center Y", "Center Z"),
            Self::YAxis => ("Center X", "Center Z"),
            Self::ZAxis => ("Center X", "Center Y"),
        }
    }
}

/// Positive angular rotation polarity around the configured rotary axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RotaryDirection {
    /// Standard ISO right-hand rule (`+θ` curls CCW when looking from `+axis` toward origin).
    #[default]
    RightHandCcw,
    /// Inverted polarity (`+θ` curls CW when looking from `+axis` toward origin).
    LeftHandCw,
}

impl RotaryDirection {
    pub const ALL: [Self; 2] = [Self::RightHandCcw, Self::LeftHandCw];

    pub const fn label(self) -> &'static str {
        match self {
            Self::RightHandCcw => "Right-Hand Rule (CCW)",
            Self::LeftHandCw => "Inverted (CW)",
        }
    }

    #[inline]
    pub const fn sign(self) -> f64 {
        match self {
            Self::RightHandCcw => 1.0,
            Self::LeftHandCw => -1.0,
        }
    }
}

/// Active 4th-axis kinematic drive mode on a [`crate::setup::Setup`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RotaryDriveMode {
    /// Standard 3-axis `X/Y/Z` setup (no rotary axis active).
    #[default]
    Disabled3Axis,
    /// Centerline-locked rotary mode driving two linear axes + rotary (`Z+X+A` or `Z+Y+B`).
    TwoLinearPlusRotary,
    /// Positional `3+1` multi-face indexing between 2.5D/3-axis operations.
    Indexed3Plus1,
    /// Full simultaneous 4-axis milling operating `X + Y + Z + Rotary` together.
    Simultaneous4Axis,
}

impl RotaryDriveMode {
    pub const ALL: [Self; 4] = [
        Self::Disabled3Axis,
        Self::TwoLinearPlusRotary,
        Self::Indexed3Plus1,
        Self::Simultaneous4Axis,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Disabled3Axis => "Disabled (3-Axis)",
            Self::TwoLinearPlusRotary => "2-Linear + Rotary (Z+X / Z+Y)",
            Self::Indexed3Plus1 => "Indexed 3+1 Positioning",
            Self::Simultaneous4Axis => "Simultaneous 4-Axis (X+Y+Z+R)",
        }
    }

    #[inline]
    pub const fn is_enabled(self) -> bool {
        !matches!(self, Self::Disabled3Axis)
    }
}

pub use chip_cl::RotaryUnwindMode;

fn default_zero_len() -> ParamLength {
    ParamLength::from_mm(0.0)
}

fn default_rotary_clearance_offset() -> ParamLength {
    ParamLength::from_mm(10.0)
}

fn default_chuck_length() -> ParamLength {
    ParamLength::from_mm(20.0)
}

fn default_chuck_diameter() -> ParamLength {
    ParamLength::from_mm(65.0)
}

fn default_wrap_radius() -> ParamLength {
    ParamLength {
        expr: Some("rotary_radius".to_string()),
        value_mm: 20.0,
    }
}

/// Per-operation 4th-axis rotary orientation or cylindrical wrapping configuration (`Operation::rotary`).
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum OperationRotaryMode {
    /// Standard 3-axis (or `0°` unrotated) operation.
    #[default]
    None,
    /// Positional **`3+1` Indexed** machining: rotates the rotary axis to `angle_deg` at safe
    /// clearance radius before executing the operation along the indexed `+Z` spindle direction.
    Indexed3Plus1 {
        #[serde(default)]
        angle_deg: f64,
    },
    /// **2-Linear + Rotary Cylindrical Wrapping** (`Z+X+A` or `Z+Y+B`): wraps a flat 2D operation
    /// (`Contour2D`, `Pocket2D`, `ChamferTrace`, `VCarve`, `DrillBore`) onto `wrap_radius` while
    /// strictly locking the cross-axis at the rotary centerline.
    WrapCylinder {
        #[serde(default = "default_wrap_radius")]
        wrap_radius: ParamLength,
        #[serde(default = "default_zero_len")]
        axial_offset: ParamLength,
        #[serde(default)]
        angle_offset_deg: f64,
    },
}

impl OperationRotaryMode {
    #[inline]
    pub const fn is_none(&self) -> bool {
        matches!(self, Self::None)
    }

    pub const fn label(&self) -> &'static str {
        match self {
            Self::None => "None (0° / Standard)",
            Self::Indexed3Plus1 { .. } => "Indexed 3+1 Angle",
            Self::WrapCylinder { .. } => "Wrap 2D on Cylinder (Z+X / Z+Y)",
        }
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        if let Self::WrapCylinder {
            wrap_radius,
            axial_offset,
            ..
        } = self
        {
            let _ = wrap_radius.reevaluate(ctx);
            let _ = axial_offset.reevaluate(ctx);
        }
    }
}

/// Computes the exact rotary index angle (`deg ∈ [0, 360)`) that rotates `normal_wcs` directly
/// upward to `+Z_wcs` (spindle axis) around `rotary.axis` (`XAxis` or `YAxis`).
///
/// Returns `None` if `rotary.axis == ZAxis` or `normal_wcs` is nearly parallel to the rotary axis.
pub fn compute_index_angle_for_normal(
    normal_wcs: DVec3,
    rotary: &ResolvedRotaryConfig,
) -> Option<f64> {
    if rotary.axis == RotaryAxisKind::ZAxis {
        return None;
    }
    let (e1, e2, _) = rotary.axis.basis();
    let c1 = normal_wcs.dot(e1);
    let c2 = normal_wcs.dot(e2) * rotary.direction.sign();
    if c1.hypot(c2) <= 1e-4 {
        return None;
    }
    let phi_deg = c2.atan2(c1).to_degrees();
    let idx = (-phi_deg).rem_euclid(360.0);
    Some(if idx.abs() < 1e-7 || (idx - 360.0).abs() < 1e-7 {
        0.0
    } else {
        idx
    })
}

/// Constructs a virtual [`ResolvedSetup`] whose WCS frame is rotated by `-angle_deg` around
/// `rotary` so that running any standard 2.5D or 3-axis strategy in that virtual Setup produces
/// coordinates directly in machine WCS for `Indexed3Plus1` machining at `angle_deg`.
pub fn build_indexed_virtual_setup(
    setup: &ResolvedSetup,
    rotary: &ResolvedRotaryConfig,
    angle_deg: f64,
) -> ResolvedSetup {
    let inv_rot_vec = |v: DVec3| {
        rotate_point_about_axis(v, DVec3::ZERO, rotary.axis, -angle_deg, rotary.direction)
            .normalize_or_zero()
    };
    let inv_rot_pt = |p: DVec3| {
        rotate_point_about_axis(
            p,
            rotary.center_wcs,
            rotary.axis,
            -angle_deg,
            rotary.direction,
        )
    };
    let fwd_rot_pt = |p: DVec3| {
        rotate_point_about_axis(
            p,
            rotary.center_wcs,
            rotary.axis,
            angle_deg,
            rotary.direction,
        )
    };

    let vx = setup.frame.local_to_world_dir(inv_rot_vec(DVec3::X));
    let vy = setup.frame.local_to_world_dir(inv_rot_vec(DVec3::Y));
    let vz = setup.frame.local_to_world_dir(inv_rot_vec(DVec3::Z));
    let v_origin_world = setup.frame.local_to_world_point(inv_rot_pt(DVec3::ZERO));

    let virtual_frame = RigidFrame3 {
        origin_mm: v_origin_world,
        x_axis: vx,
        y_axis: vy,
        z_axis: vz,
    };

    // Transform model AABB corners into rotated virtual WCS
    let mut rot_model_aabb = Aabb3::EMPTY;
    if setup.model_aabb_wcs.is_valid() {
        for x in [setup.model_aabb_wcs.min.x, setup.model_aabb_wcs.max.x] {
            for y in [setup.model_aabb_wcs.min.y, setup.model_aabb_wcs.max.y] {
                for z in [setup.model_aabb_wcs.min.z, setup.model_aabb_wcs.max.z] {
                    rot_model_aabb.extend_point(fwd_rot_pt(DVec3::new(x, y, z)));
                }
            }
        }
    }

    // Transform stock geometry into rotated virtual WCS
    let rot_stock = match setup.stock {
        ResolvedStockGeometry::Cylinder {
            axis,
            center_wcs,
            radius_mm,
            min_axial_mm,
            max_axial_mm,
            aabb_wcs,
        } if axis == rotary.axis.to_stock_axis() => {
            let new_c = fwd_rot_pt(center_wcs);
            let mut new_aabb = Aabb3::EMPTY;
            for x in [aabb_wcs.min.x, aabb_wcs.max.x] {
                for y in [aabb_wcs.min.y, aabb_wcs.max.y] {
                    for z in [aabb_wcs.min.z, aabb_wcs.max.z] {
                        new_aabb.extend_point(fwd_rot_pt(DVec3::new(x, y, z)));
                    }
                }
            }
            ResolvedStockGeometry::Cylinder {
                axis,
                center_wcs: new_c,
                radius_mm,
                min_axial_mm,
                max_axial_mm,
                aabb_wcs: new_aabb,
            }
        }
        other => {
            let aabb = other.aabb_wcs();
            let mut new_aabb = Aabb3::EMPTY;
            for x in [aabb.min.x, aabb.max.x] {
                for y in [aabb.min.y, aabb.max.y] {
                    for z in [aabb.min.z, aabb.max.z] {
                        new_aabb.extend_point(fwd_rot_pt(DVec3::new(x, y, z)));
                    }
                }
            }
            ResolvedStockGeometry::Box { aabb_wcs: new_aabb }
        }
    };

    let safe_r_z = rotary.center_wcs.z + rotary.clearance_radius_mm;
    ResolvedSetup {
        frame: virtual_frame,
        model_aabb_wcs: rot_model_aabb,
        stock: rot_stock,
        rotary: Some(*rotary),
        clearance_z_wcs: setup.clearance_z_wcs.max(safe_r_z + 2.0),
        retract_z_wcs: setup.retract_z_wcs.max(safe_r_z),
    }
}

/// Setup-level 4th-axis rotary kinematics configuration (`Setup::rotary`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RotaryConfig {
    #[serde(default)]
    pub mode: RotaryDriveMode,
    #[serde(default)]
    pub axis: RotaryAxisKind,
    #[serde(default)]
    pub direction: RotaryDirection,
    /// Optional explicit G-code axis letter override (`'A'`, `'B'`, or `'C'`); `None` uses `axis.default_letter()`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub axis_letter_override: Option<char>,
    /// Perpendicular WCS offset of the rotary centerline from `(0, 0, 0)` along the two non-rotary axes:
    /// - `XAxis`: `[center_y_wcs, center_z_wcs]`
    /// - `YAxis`: `[center_x_wcs, center_z_wcs]`
    /// - `ZAxis`: `[center_x_wcs, center_y_wcs]`
    #[serde(default = "default_center_offset_pair")]
    pub center_offset_wcs: [ParamLength; 2],
    /// Extra radial clearance (`mm`) added outside the stock's outer radius for safe index rotations.
    #[serde(default = "default_rotary_clearance_offset")]
    pub clearance_radial_offset: ParamLength,
    /// When `false` (default), the rotary chuck sits at the `-u` stock end; when `true`, at `+u`.
    #[serde(default)]
    pub chuck_end_positive: bool,
    /// Axial keep-out collar length (`mm`) representing the rotary chuck jaws.
    #[serde(default = "default_chuck_length")]
    pub chuck_length: ParamLength,
    /// Outer diameter (`mm`) of the rotary chuck body for viewport gizmo & collision checks.
    #[serde(default = "default_chuck_diameter")]
    pub chuck_diameter: ParamLength,
    #[serde(default)]
    pub unwind_mode: RotaryUnwindMode,
}

fn default_center_offset_pair() -> [ParamLength; 2] {
    [default_zero_len(), default_zero_len()]
}

impl Default for RotaryConfig {
    fn default() -> Self {
        Self {
            mode: RotaryDriveMode::Disabled3Axis,
            axis: RotaryAxisKind::XAxis,
            direction: RotaryDirection::RightHandCcw,
            axis_letter_override: None,
            center_offset_wcs: default_center_offset_pair(),
            clearance_radial_offset: default_rotary_clearance_offset(),
            chuck_end_positive: false,
            chuck_length: default_chuck_length(),
            chuck_diameter: default_chuck_diameter(),
            unwind_mode: RotaryUnwindMode::ContinuousMultiTurn,
        }
    }
}

impl RotaryConfig {
    #[inline]
    pub const fn is_disabled(&self) -> bool {
        !self.mode.is_enabled()
    }

    #[inline]
    pub const fn is_enabled(&self) -> bool {
        self.mode.is_enabled()
    }

    /// Effective rotary axis letter (`'A'`, `'B'`, or `'C'`).
    pub fn effective_axis_letter(&self) -> char {
        self.axis_letter_override
            .map(|c| c.to_ascii_uppercase())
            .filter(|c| matches!(c, 'A' | 'B' | 'C'))
            .unwrap_or_else(|| self.axis.default_letter())
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.center_offset_wcs[0].reevaluate(ctx);
        let _ = self.center_offset_wcs[1].reevaluate(ctx);
        let _ = self.clearance_radial_offset.reevaluate(ctx);
        let _ = self.chuck_length.reevaluate(ctx);
        let _ = self.chuck_diameter.reevaluate(ctx);
    }

    /// Resolves this [`RotaryConfig`] against the evaluated Setup WCS stock bounds.
    pub fn resolve(
        &self,
        stock: &ResolvedStockGeometry,
        eval_ctx: &EvalContext,
    ) -> Option<ResolvedRotaryConfig> {
        if self.is_disabled() {
            return None;
        }
        let mut cfg = self.clone();
        cfg.reevaluate(eval_ctx);

        let a = cfg.center_offset_wcs[0].value_mm;
        let b = cfg.center_offset_wcs[1].value_mm;
        let center_wcs = match cfg.axis {
            RotaryAxisKind::XAxis => DVec3::new(0.0, a, b),
            RotaryAxisKind::YAxis => DVec3::new(a, 0.0, b),
            RotaryAxisKind::ZAxis => DVec3::new(a, b, 0.0),
        };

        let (stock_u_min, stock_u_max, stock_outer_radius_mm) =
            compute_stock_rotary_envelope(stock, cfg.axis, center_wcs);
        let clearance_offset = cfg.clearance_radial_offset.value_mm.max(0.0);
        let clearance_radius_mm = stock_outer_radius_mm + clearance_offset;
        let chuck_len = cfg.chuck_length.value_mm.max(0.0);
        let chuck_radius_mm = (0.5 * cfg.chuck_diameter.value_mm).max(stock_outer_radius_mm);
        let (chuck_u_min, chuck_u_max) = if cfg.chuck_end_positive {
            (stock_u_max, stock_u_max + chuck_len)
        } else {
            (stock_u_min - chuck_len, stock_u_min)
        };

        Some(ResolvedRotaryConfig {
            mode: cfg.mode,
            axis: cfg.axis,
            direction: cfg.direction,
            axis_letter: cfg.effective_axis_letter(),
            center_wcs,
            stock_u_min,
            stock_u_max,
            stock_outer_radius_mm,
            clearance_radius_mm,
            chuck_end_positive: cfg.chuck_end_positive,
            chuck_u_min,
            chuck_u_max,
            chuck_radius_mm,
            unwind_mode: cfg.unwind_mode,
        })
    }
}

/// Concrete resolved 4th-axis rotary kinematics in canonical Setup WCS millimeters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolvedRotaryConfig {
    pub mode: RotaryDriveMode,
    pub axis: RotaryAxisKind,
    pub direction: RotaryDirection,
    pub axis_letter: char,
    /// Point lying on the rotary centerline in Setup WCS (`mm`).
    pub center_wcs: DVec3,
    /// Axial span `[u_min, u_max]` of the raw stock along `axis.axis_unit()`.
    pub stock_u_min: f64,
    pub stock_u_max: f64,
    /// Maximum outer radius (`mm`) of the raw stock relative to `center_wcs`.
    pub stock_outer_radius_mm: f64,
    /// Safe radial clearance radius (`mm`) for indexed rotary positioning moves.
    pub clearance_radius_mm: f64,
    pub chuck_end_positive: bool,
    pub chuck_u_min: f64,
    pub chuck_u_max: f64,
    pub chuck_radius_mm: f64,
    pub unwind_mode: RotaryUnwindMode,
}

impl ResolvedRotaryConfig {
    /// Rotates a WCS point around this rotary axis by `angle_deg` honoring `self.direction`.
    #[inline]
    pub fn rotate_point_wcs(&self, pt_wcs: DVec3, angle_deg: f64) -> DVec3 {
        rotate_point_about_axis(
            pt_wcs,
            self.center_wcs,
            self.axis,
            angle_deg,
            self.direction,
        )
    }

    /// Transforms a WCS point into cylindrical coordinates `(u_axial_mm, theta_rad, radius_mm)`
    /// relative to this rotary centerline (`θ = 0` along `e1`, increasing with `self.direction`).
    #[inline]
    pub fn wcs_to_cylindrical(&self, pt_wcs: DVec3) -> (f64, f64, f64) {
        wcs_to_cylindrical(pt_wcs, self)
    }

    /// Converts cylindrical coordinates `(u_axial_mm, theta_rad, radius_mm)` back into Setup WCS.
    #[inline]
    pub fn cylindrical_to_wcs(&self, u_axial: f64, theta_rad: f64, radius_mm: f64) -> DVec3 {
        cylindrical_to_wcs(u_axial, theta_rad, radius_mm, self)
    }

    /// Axial midpoint of the stock along the rotary axis in Setup WCS.
    #[inline]
    pub fn stock_center_wcs(&self) -> DVec3 {
        let u_mid = 0.5 * (self.stock_u_min + self.stock_u_max);
        let u_vec = self.axis.axis_unit();
        self.center_wcs + u_vec * (u_mid - self.center_wcs.dot(u_vec))
    }
}

/// Computes `(u_min, u_max, max_radius_mm)` of `stock` relative to `center_wcs` along `axis`.
pub fn compute_stock_rotary_envelope(
    stock: &ResolvedStockGeometry,
    axis: RotaryAxisKind,
    center_wcs: DVec3,
) -> (f64, f64, f64) {
    let (e1, e2, _u_vec) = axis.basis();
    match *stock {
        ResolvedStockGeometry::Cylinder {
            axis: stock_axis,
            center_wcs: cyl_center,
            radius_mm,
            min_axial_mm,
            max_axial_mm,
            ..
        } if stock_axis == axis.to_stock_axis() => {
            let perp = cyl_center - center_wcs;
            let ecc = perp.dot(e1).hypot(perp.dot(e2));
            (min_axial_mm, max_axial_mm, radius_mm + ecc)
        }
        ResolvedStockGeometry::Box { aabb_wcs }
        | ResolvedStockGeometry::Cylinder { aabb_wcs, .. } => {
            aabb_rotary_envelope(&aabb_wcs, axis, center_wcs)
        }
    }
}

fn aabb_rotary_envelope(aabb: &Aabb3, axis: RotaryAxisKind, center_wcs: DVec3) -> (f64, f64, f64) {
    let (e1, e2, u_vec) = axis.basis();
    let mut u_min = f64::INFINITY;
    let mut u_max = f64::NEG_INFINITY;
    let mut r_max = 0.0_f64;

    for x in [aabb.min.x, aabb.max.x] {
        for y in [aabb.min.y, aabb.max.y] {
            for z in [aabb.min.z, aabb.max.z] {
                let p = DVec3::new(x, y, z);
                let u = p.dot(u_vec);
                let rel = p - center_wcs;
                let r = rel.dot(e1).hypot(rel.dot(e2));
                u_min = u_min.min(u);
                u_max = u_max.max(u);
                r_max = r_max.max(r);
            }
        }
    }
    if !u_min.is_finite() {
        (0.0, 0.0, 0.0)
    } else {
        (u_min, u_max, r_max)
    }
}

/// Rotates `pt_wcs` around the line passing through `center_wcs` with direction `axis.axis_unit()`
/// by `angle_deg` honoring `direction` polarity.
#[inline]
pub fn rotate_point_about_axis(
    pt_wcs: DVec3,
    center_wcs: DVec3,
    axis: RotaryAxisKind,
    angle_deg: f64,
    direction: RotaryDirection,
) -> DVec3 {
    let (e1, e2, u_vec) = axis.basis();
    let rel = pt_wcs - center_wcs;
    let u = rel.dot(u_vec);
    let c1 = rel.dot(e1);
    let c2 = rel.dot(e2);
    let a = (angle_deg * direction.sign()).to_radians();
    let (sin_a, cos_a) = a.sin_cos();
    let r1 = c1 * cos_a - c2 * sin_a;
    let r2 = c1 * sin_a + c2 * cos_a;
    center_wcs + u_vec * u + e1 * r1 + e2 * r2
}

/// Converts a Setup WCS point into cylindrical coordinates `(u_axial_mm, theta_rad, radius_mm)`.
///
/// `u_axial_mm` is the absolute projection `pt_wcs · u_axis`, `theta_rad ∈ (-π, π]` follows
/// `rotary.direction`, and `radius_mm >= 0` is perpendicular distance to the rotary centerline.
#[inline]
pub fn wcs_to_cylindrical(pt_wcs: DVec3, rotary: &ResolvedRotaryConfig) -> (f64, f64, f64) {
    let (e1, e2, u_vec) = rotary.axis.basis();
    let u_axial = pt_wcs.dot(u_vec);
    let rel = pt_wcs - rotary.center_wcs;
    let c1 = rel.dot(e1);
    let c2 = rel.dot(e2) * rotary.direction.sign();
    let r = c1.hypot(c2);
    let theta_rad = c2.atan2(c1);
    (u_axial, theta_rad, r)
}

/// Converts cylindrical coordinates `(u_axial_mm, theta_rad, radius_mm)` back into Setup WCS.
#[inline]
pub fn cylindrical_to_wcs(
    u_axial: f64,
    theta_rad: f64,
    radius_mm: f64,
    rotary: &ResolvedRotaryConfig,
) -> DVec3 {
    let (e1, e2, u_vec) = rotary.axis.basis();
    let (sin_t, cos_t) = theta_rad.sin_cos();
    let c1 = radius_mm * cos_t;
    let c2 = radius_mm * sin_t * rotary.direction.sign();
    let u_center = rotary.center_wcs.dot(u_vec);
    rotary.center_wcs + u_vec * (u_axial - u_center) + e1 * c1 + e2 * c2
}

/// Minimum axial safety clearance (`mm`) added beyond the cutter radius from the rotary chuck face.
pub const ROTARY_CHUCK_GUARD_CLEARANCE_MM: f64 = 0.5;

/// Computes the hard rotary chuck keep-out boundary plane coordinate (`mm` along `rot.axis`)
/// for a cutter of `tool_diameter_mm`:
/// - When `!rot.chuck_end_positive` (chuck at `-U`): `chuck_u_max + tool_r + 0.5 mm`
///   (tool centerline must satisfy `u >= chuck_guard_u`).
/// - When `rot.chuck_end_positive` (chuck at `+U`): `chuck_u_min - (tool_r + 0.5 mm)`
///   (tool centerline must satisfy `u <= chuck_guard_u`).
#[inline]
pub fn rotary_chuck_guard_u(rot: &ResolvedRotaryConfig, tool_diameter_mm: f64) -> f64 {
    let tool_r = 0.5 * tool_diameter_mm.max(0.1);
    let guard = tool_r + ROTARY_CHUCK_GUARD_CLEARANCE_MM;
    if rot.chuck_end_positive {
        rot.chuck_u_min - guard
    } else {
        rot.chuck_u_max + guard
    }
}

/// Fully resolved 4th-axis rotary axial (`-U`/`+U`), angular (`θ`), and radial (`r_floor..r_ceiling`)
/// machining window shared by native rotary toolpath kernels and live 3D viewport previews.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedRotaryWindow {
    /// Nominal axial bounds before tool-containment & margin shifts (`mm` along rotary axis).
    pub nominal_u_min: f64,
    pub nominal_u_max: f64,
    /// Final chuck-safe tool-center axial span `[u_start, u_end]` (`mm`).
    pub u_start: f64,
    pub u_end: f64,
    /// Effective angular span `[start_angle_deg, end_angle_deg]` (`deg`).
    pub start_angle_deg: f64,
    pub end_angle_deg: f64,
    /// Effective radial envelope `[r_floor, r_ceiling]` (`mm` from rotary centerline).
    pub r_floor: f64,
    pub r_ceiling: f64,
    /// Hard chuck keep-out boundary plane coordinate (`mm`) along the rotary axis.
    pub chuck_guard_u: f64,
    /// Whether the chuck keep-out guard actively clamped `u_start` or `u_end`.
    pub clamped_by_chuck: bool,
    pub warnings: Vec<String>,
}

/// Input bundle for [`resolve_rotary_window`].
pub struct RotaryWindowInput<'a> {
    pub rot: &'a ResolvedRotaryConfig,
    pub frame: &'a RigidFrame3,
    pub doc: Option<&'a CadDocument>,
    pub base_mesh: &'a SurfaceMesh3D,
    pub selections: &'a [GeometrySelectionItem],
    pub extracted_selection: Option<&'a ExtractedSelectionSummary>,
    pub boundary: &'a RotaryAxialBoundaryConfig,
    pub tool_diameter_mm: f64,
    pub top_z_wcs: f64,
    pub bottom_z_wcs: f64,
    pub op_name: &'a str,
}

fn extend_u_from_aabb_world(
    aabb_world: &Aabb3,
    frame: &RigidFrame3,
    u_vec: DVec3,
    u_min: &mut f64,
    u_max: &mut f64,
) {
    if !aabb_world.is_valid() {
        return;
    }
    for x in [aabb_world.min.x, aabb_world.max.x] {
        for y in [aabb_world.min.y, aabb_world.max.y] {
            for z in [aabb_world.min.z, aabb_world.max.z] {
                let p_wcs = frame.world_to_local_point(DVec3::new(x, y, z));
                let u = p_wcs.dot(u_vec);
                *u_min = (*u_min).min(u);
                *u_max = (*u_max).max(u);
            }
        }
    }
}

fn extend_u_from_edge(
    doc: &CadDocument,
    eid: EdgeId,
    frame: &RigidFrame3,
    u_vec: DVec3,
    u_min: &mut f64,
    u_max: &mut f64,
) {
    let mut found_pts = false;
    if let Some(ep) = doc
        .render_mesh
        .edge_polylines
        .iter()
        .find(|p| p.edge_id == eid)
    {
        for &pt_world in &ep.points_mm {
            let u = frame.world_to_local_point(pt_world).dot(u_vec);
            *u_min = (*u_min).min(u);
            *u_max = (*u_max).max(u);
            found_pts = true;
        }
    }
    if let Some(edge) = doc.edge(eid) {
        match edge.curve {
            CurveKind::Line { start_mm, end_mm } => {
                for p in [start_mm, end_mm] {
                    let u = frame.world_to_local_point(p).dot(u_vec);
                    *u_min = (*u_min).min(u);
                    *u_max = (*u_max).max(u);
                    found_pts = true;
                }
            }
            CurveKind::CircleArc {
                center_mm,
                normal,
                radius_mm,
                start_mm,
                end_mm,
                is_full_circle,
            } => {
                for p in [start_mm, end_mm] {
                    let u = frame.world_to_local_point(p).dot(u_vec);
                    *u_min = (*u_min).min(u);
                    *u_max = (*u_max).max(u);
                    found_pts = true;
                }
                if is_full_circle {
                    // Exact axial projection of a 3D circle of radius `r` and unit normal `n`:
                    // `u(θ) = c · u_vec ± r * sqrt(1 - (n · u_vec)^2)`.
                    let c_wcs = frame.world_to_local_point(center_mm);
                    let n_wcs = frame.world_to_local_dir(normal.normalize_or_zero());
                    let cos_nu = n_wcs.dot(u_vec).clamp(-1.0, 1.0);
                    let sin_nu = (1.0 - cos_nu * cos_nu).max(0.0).sqrt();
                    let cu = c_wcs.dot(u_vec);
                    let r_u = radius_mm.abs() * sin_nu;
                    *u_min = (*u_min).min(cu - r_u);
                    *u_max = (*u_max).max(cu + r_u);
                    found_pts = true;
                }
            }
            _ => {}
        }
        if !found_pts {
            extend_u_from_aabb_world(&edge.aabb_mm, frame, u_vec, u_min, u_max);
        }
    }
}

fn extend_u_from_face(
    doc: &CadDocument,
    fid: FaceId,
    frame: &RigidFrame3,
    u_vec: DVec3,
    u_min: &mut f64,
    u_max: &mut f64,
) {
    let mut found_pts = false;
    for tri in &doc.render_mesh.pick_triangles {
        if tri.face_id == fid {
            for p in [tri.v0_mm, tri.v1_mm, tri.v2_mm] {
                let u = frame.world_to_local_point(p).dot(u_vec);
                *u_min = (*u_min).min(u);
                *u_max = (*u_max).max(u);
            }
            found_pts = true;
        }
    }
    if let Some(face) = doc.face(fid) {
        if let Some(lid) = face.outer_loop
            && let Some(lp) = doc.brep_loop(lid)
        {
            for eu in &lp.edges {
                extend_u_from_edge(doc, eu.edge_id, frame, u_vec, u_min, u_max);
                found_pts = true;
            }
        }
        for &ilid in &face.inner_loops {
            if let Some(lp) = doc.brep_loop(ilid) {
                for eu in &lp.edges {
                    extend_u_from_edge(doc, eu.edge_id, frame, u_vec, u_min, u_max);
                    found_pts = true;
                }
            }
        }
        if !found_pts {
            extend_u_from_aabb_world(&face.aabb_mm, frame, u_vec, u_min, u_max);
        }
    }
}

/// Projects all selected 3D B-Rep faces/loops/chains/holes and extracted 2D sketch features
/// onto the rotary axis unit vector `u_vec` to obtain `[sel_u_min, sel_u_max]` (`mm`).
///
/// Returns `None` if no selections are present or none project to finite coordinates.
pub fn project_selections_onto_rotary_axis(
    doc: Option<&CadDocument>,
    frame: &RigidFrame3,
    selections: &[GeometrySelectionItem],
    extracted_selection: Option<&ExtractedSelectionSummary>,
    rot: &ResolvedRotaryConfig,
) -> Option<(f64, f64)> {
    let (_, _, u_vec) = rot.axis.basis();
    let mut u_min = f64::INFINITY;
    let mut u_max = f64::NEG_INFINITY;

    let extend_xy_box = |box2: chip_math::Aabb2, u_min: &mut f64, u_max: &mut f64| {
        if box2.is_valid() {
            for x in [box2.min.x, box2.max.x] {
                for y in [box2.min.y, box2.max.y] {
                    let u = DVec3::new(x, y, 0.0).dot(u_vec);
                    *u_min = (*u_min).min(u);
                    *u_max = (*u_max).max(u);
                }
            }
        }
    };

    if let Some(summary) = extracted_selection {
        for feat in &summary.features {
            match feat {
                ExtractedFeature25D::PlanarFace {
                    outer_boundary_wcs,
                    inner_islands_wcs,
                    ..
                } => {
                    extend_xy_box(outer_boundary_wcs.exact_aabb(), &mut u_min, &mut u_max);
                    for isl in inner_islands_wcs {
                        extend_xy_box(isl.exact_aabb(), &mut u_min, &mut u_max);
                    }
                }
                ExtractedFeature25D::Contour { polyline_wcs, .. } => {
                    extend_xy_box(polyline_wcs.exact_aabb(), &mut u_min, &mut u_max);
                }
                ExtractedFeature25D::ModeledChamfer {
                    upper_polyline_wcs,
                    lower_polyline_wcs,
                    ..
                } => {
                    extend_xy_box(upper_polyline_wcs.exact_aabb(), &mut u_min, &mut u_max);
                    extend_xy_box(lower_polyline_wcs.exact_aabb(), &mut u_min, &mut u_max);
                }
                ExtractedFeature25D::CylindricalHole {
                    center_wcs,
                    diameter_mm,
                    ..
                } => {
                    let r = 0.5 * diameter_mm.abs();
                    let cu = DVec3::new(center_wcs.x, center_wcs.y, 0.0).dot(u_vec);
                    u_min = u_min.min(cu - r);
                    u_max = u_max.max(cu + r);
                }
            }
        }
    }

    if let Some(cad) = doc {
        for item in selections {
            match item {
                GeometrySelectionItem::PlanarFace { binding }
                | GeometrySelectionItem::CylindricalHole { binding } => {
                    let fid = FaceId(binding.last_index);
                    if cad.face(fid).is_some() {
                        extend_u_from_face(cad, fid, frame, u_vec, &mut u_min, &mut u_max);
                    } else {
                        extend_u_from_aabb_world(
                            &binding.signature.aabb_mm,
                            frame,
                            u_vec,
                            &mut u_min,
                            &mut u_max,
                        );
                    }
                }
                GeometrySelectionItem::ClosedLoop { binding, .. } => {
                    let lid = LoopId(binding.last_index);
                    if let Some(lp) = cad.brep_loop(lid) {
                        for eu in &lp.edges {
                            extend_u_from_edge(
                                cad, eu.edge_id, frame, u_vec, &mut u_min, &mut u_max,
                            );
                        }
                    } else {
                        extend_u_from_aabb_world(
                            &binding.signature.aabb_mm,
                            frame,
                            u_vec,
                            &mut u_min,
                            &mut u_max,
                        );
                    }
                }
                GeometrySelectionItem::OpenChain {
                    start_edge,
                    resolved_edges,
                    ..
                } => {
                    if resolved_edges.is_empty() {
                        let eid = EdgeId(start_edge.last_index);
                        if cad.edge(eid).is_some() {
                            extend_u_from_edge(cad, eid, frame, u_vec, &mut u_min, &mut u_max);
                        } else {
                            extend_u_from_aabb_world(
                                &start_edge.signature.aabb_mm,
                                frame,
                                u_vec,
                                &mut u_min,
                                &mut u_max,
                            );
                        }
                    } else {
                        for eu in resolved_edges {
                            extend_u_from_edge(
                                cad, eu.edge_id, frame, u_vec, &mut u_min, &mut u_max,
                            );
                        }
                    }
                }
                GeometrySelectionItem::SketchRegion { .. }
                | GeometrySelectionItem::SketchEntity { .. }
                | GeometrySelectionItem::SketchLayerAll { .. } => {
                    // Handled via `extracted_selection` above.
                }
            }
        }
    }

    if u_min.is_finite() && u_max.is_finite() {
        Some((u_min, u_max))
    } else {
        None
    }
}

/// Resolves the complete 4th-axis rotary machining window (`-U`/`+U` axial bounds, tool
/// containment, signed margins, mandatory hard chuck keep-out guard, angular sector, and
/// radial floor/ceiling envelope) for [`crate::strategies::rotary2linear`] and
/// [`crate::strategies::rotary4axis_sim`].
pub fn resolve_rotary_window(
    input: &RotaryWindowInput<'_>,
) -> Result<ResolvedRotaryWindow, CamError> {
    let rot = input.rot;
    let boundary = input.boundary;
    let (_, _, u_vec) = rot.axis.basis();

    let mut mesh_u_min = f64::INFINITY;
    let mut mesh_u_max = f64::NEG_INFINITY;
    for &v in &input.base_mesh.vertices {
        let u = v.dot(u_vec);
        mesh_u_min = mesh_u_min.min(u);
        mesh_u_max = mesh_u_max.max(u);
    }

    let model_nominal = if mesh_u_min.is_finite() && mesh_u_max.is_finite() {
        (
            mesh_u_min.max(rot.stock_u_min),
            mesh_u_max.min(rot.stock_u_max),
        )
    } else {
        (rot.stock_u_min, rot.stock_u_max)
    };

    let mut warnings = Vec::new();
    let (nominal_u_min, nominal_u_max) = match boundary.axial_boundary_mode {
        RotaryAxialBoundaryMode::StockBounds => (rot.stock_u_min, rot.stock_u_max),
        RotaryAxialBoundaryMode::ModelBounds => model_nominal,
        RotaryAxialBoundaryMode::SelectedGeometry => {
            if let Some((sel_min, sel_max)) = project_selections_onto_rotary_axis(
                input.doc,
                input.frame,
                input.selections,
                input.extracted_selection,
                rot,
            ) {
                (sel_min, sel_max)
            } else {
                warnings.push(
                    "Rotary axial boundary mode 'Selected Geometry' has no projectable selections; falling back to Model Bounds."
                        .to_string(),
                );
                model_nominal
            }
        }
        RotaryAxialBoundaryMode::ExplicitRange => {
            let e_start = boundary.explicit_u_start.value_mm;
            let e_end = boundary.explicit_u_end.value_mm;
            if !e_start.is_finite() {
                return Err(CamError::InvalidParameter {
                    param: "explicit_u_start",
                    value: e_start,
                    reason: "must be finite",
                });
            }
            if !e_end.is_finite() {
                return Err(CamError::InvalidParameter {
                    param: "explicit_u_end",
                    value: e_end,
                    reason: "must be finite",
                });
            }
            if e_start > e_end + 1e-9 {
                warnings.push(format!(
                    "Explicit rotary axial range start ({e_start:.2} mm) > end ({e_end:.2} mm); normalized to [{e_end:.2}, {e_start:.2}] mm."
                ));
            }
            (e_start.min(e_end), e_start.max(e_end))
        }
    };

    let margin_start = boundary.axial_margin_start.value_mm;
    let margin_end = boundary.axial_margin_end.value_mm;
    if !margin_start.is_finite() {
        return Err(CamError::InvalidParameter {
            param: "axial_margin_start",
            value: margin_start,
            reason: "must be finite",
        });
    }
    if !margin_end.is_finite() {
        return Err(CamError::InvalidParameter {
            param: "axial_margin_end",
            value: margin_end,
            reason: "must be finite",
        });
    }

    let tool_d = input.tool_diameter_mm.max(0.1);
    let tool_r = 0.5 * tool_d;

    // 1. Apply axial tool containment relative to nominal [-U, +U] ends
    let (mut u_start, mut u_end) = match boundary.axial_containment {
        ToolContainmentMode::ToolInsideBoundary => (nominal_u_min + tool_r, nominal_u_max - tool_r),
        ToolContainmentMode::ToolCenterOnBoundary => (nominal_u_min, nominal_u_max),
        ToolContainmentMode::ToolOutsideBoundary => {
            (nominal_u_min - tool_r, nominal_u_max + tool_r)
        }
    };

    // 2. Apply signed margins (> 0 trims inward, < 0 extends outward past boundary)
    u_start += margin_start;
    u_end -= margin_end;

    // 3. Enforce mandatory hard rotary chuck keep-out guard AFTER containment and margins
    let chuck_guard_u = rotary_chuck_guard_u(rot, tool_d);
    let mut clamped_by_chuck = false;
    if rot.chuck_end_positive {
        if u_end > chuck_guard_u + 1e-9 {
            let requested = u_end;
            u_end = chuck_guard_u;
            clamped_by_chuck = true;
            warnings.push(format!(
                "Rotary axial +U limit ({requested:.2} mm) clamped to {chuck_guard_u:.2} mm by mandatory chuck keep-out guard (tool radius + {ROTARY_CHUCK_GUARD_CLEARANCE_MM:.1} mm)."
            ));
        }
    } else if u_start < chuck_guard_u - 1e-9 {
        let requested = u_start;
        u_start = chuck_guard_u;
        clamped_by_chuck = true;
        warnings.push(format!(
            "Rotary axial -U limit ({requested:.2} mm) clamped to {chuck_guard_u:.2} mm by mandatory chuck keep-out guard (tool radius + {ROTARY_CHUCK_GUARD_CLEARANCE_MM:.1} mm)."
        ));
    }

    if !u_start.is_finite() || !u_end.is_finite() || u_end <= u_start + 1e-3 {
        return Err(CamError::InvalidRotarySetup {
            op_name: input.op_name.to_string(),
            reason: format!(
                "Axial machining window [{u_start:.2}, {u_end:.2}] mm is empty after containment, margins, and chuck clearance"
            ),
        });
    }

    let start_angle_deg = boundary.start_angle_deg;
    let end_angle_deg = boundary.end_angle_deg;
    if !start_angle_deg.is_finite()
        || !end_angle_deg.is_finite()
        || end_angle_deg <= start_angle_deg + 1e-3
    {
        return Err(CamError::InvalidParameter {
            param: "end_angle_deg",
            value: end_angle_deg,
            reason: "must be finite and greater than start_angle_deg",
        });
    }

    let r_floor = (input.bottom_z_wcs - rot.center_wcs.z).max(0.0);
    let stock_top_z = rot.center_wcs.z + rot.stock_outer_radius_mm;
    let r_ceiling = if input.top_z_wcs < stock_top_z - 1e-6 {
        (input.top_z_wcs - rot.center_wcs.z).clamp(r_floor, rot.stock_outer_radius_mm.max(r_floor))
    } else {
        rot.stock_outer_radius_mm.max(r_floor)
    };

    Ok(ResolvedRotaryWindow {
        nominal_u_min,
        nominal_u_max,
        u_start,
        u_end,
        start_angle_deg,
        end_angle_deg,
        r_floor,
        r_ceiling,
        chuck_guard_u,
        clamped_by_chuck,
        warnings,
    })
}

/// Validates that every motion segment of `tp` (and any nested clearing stage) respects the
/// mandatory rotary chuck axial keep-out plane (`rotary_chuck_guard_u`), returning a structured
/// [`CamError::InvalidRotarySetup`] if any move intrudes into the chuck keep-out zone.
pub fn validate_toolpath_rotary_chuck_clearance(
    tp: &GeneratedToolpath,
    rot: &ResolvedRotaryConfig,
    tool_diameter_mm: f64,
    clearing_tool_diameter_mm: Option<f64>,
    op_name: &str,
) -> Result<(), CamError> {
    if rot.axis == RotaryAxisKind::ZAxis || (rot.chuck_u_max - rot.chuck_u_min).abs() <= 1e-6 {
        return Ok(());
    }
    let guard_u = rotary_chuck_guard_u(rot, tool_diameter_mm);
    let (_, _, u_vec) = rot.axis.basis();
    let chuck_top_z = rot.center_wcs.z + rot.chuck_radius_mm + ROTARY_CHUCK_GUARD_CLEARANCE_MM;
    let mut worst_violation: Option<f64> = None;

    for seg in &tp.segments_wcs {
        let is_rapid = matches!(seg.motion, SegmentMotion3D::Rapid);
        let min_z = seg.start_wcs.z.min(seg.end_wcs.z);
        if is_rapid && min_z > chuck_top_z {
            continue;
        }
        let u0 = seg.start_wcs.dot(u_vec);
        let u1 = seg.end_wcs.dot(u_vec);
        if rot.chuck_end_positive {
            let u_hi = u0.max(u1);
            if u_hi > guard_u + 1e-4 {
                worst_violation = Some(worst_violation.map_or(u_hi, |w| w.max(u_hi)));
            }
        } else {
            let u_lo = u0.min(u1);
            if u_lo < guard_u - 1e-4 {
                worst_violation = Some(worst_violation.map_or(u_lo, |w| w.min(u_lo)));
            }
        }
    }

    if let Some(offending_u) = worst_violation {
        let (side_label, cmp_label) = if rot.chuck_end_positive {
            ("+U", "<=")
        } else {
            ("-U", ">=")
        };
        return Err(CamError::InvalidRotarySetup {
            op_name: op_name.to_string(),
            reason: format!(
                "Toolpath reaches U = {offending_u:.2} mm and intrudes into the {side_label} rotary chuck keep-out zone (must stay {cmp_label} {guard_u:.2} mm for Ø{:.2} mm cutter + {ROTARY_CHUCK_GUARD_CLEARANCE_MM:.1} mm clearance)",
                tool_diameter_mm.max(0.1)
            ),
        });
    }

    if let Some(cs) = &tp.clearing_stage {
        let clear_d = clearing_tool_diameter_mm.unwrap_or(tool_diameter_mm);
        validate_toolpath_rotary_chuck_clearance(&cs.toolpath, rot, clear_d, None, op_name)?;
    }

    Ok(())
}

/// Builds a rotated [`SurfaceMesh3D`] pre-filtered to ONLY triangles intersecting the exact
/// cutter strip (`|cross - center| <= cross_half_width`, `u ∈ [u_start - cross_half_width, u_end + cross_half_width]`,
/// `max.z >= z_floor`), eliminating ~85% of unreachable triangles prior to BVH construction with
/// zero approximation error.
pub fn build_rotated_strip_mesh(
    base: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    theta_deg: f64,
    u_start: f64,
    u_end: f64,
    cross_half_width: f64,
    z_floor: f64,
) -> SurfaceMesh3D {
    let vertices_wcs: Vec<DVec3> = base
        .vertices
        .iter()
        .map(|&v| rot.rotate_point_wcs(v, theta_deg))
        .collect();

    let pad = cross_half_width + 1e-4;
    let u_lo = u_start - pad;
    let u_hi = u_end + pad;
    let z_lo = z_floor - 1e-4;

    let mut aabb_wcs = Aabb3::EMPTY;
    let mut triangles = Vec::with_capacity(base.triangles.len() / 3);
    for tri in &base.triangles {
        let [i0, i1, i2] = tri.indices;
        let v0 = vertices_wcs[i0 as usize];
        let v1 = vertices_wcs[i1 as usize];
        let v2 = vertices_wcs[i2 as usize];
        let tri_min = v0.min(v1).min(v2);
        let tri_max = v0.max(v1).max(v2);
        if tri_max.z < z_lo {
            continue;
        }
        let overlaps_strip = match rot.axis {
            RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => {
                tri_max.x >= u_lo
                    && tri_min.x <= u_hi
                    && tri_max.y >= rot.center_wcs.y - pad
                    && tri_min.y <= rot.center_wcs.y + pad
            }
            RotaryAxisKind::YAxis => {
                tri_max.y >= u_lo
                    && tri_min.y <= u_hi
                    && tri_max.x >= rot.center_wcs.x - pad
                    && tri_min.x <= rot.center_wcs.x + pad
            }
        };
        if !overlaps_strip {
            continue;
        }
        let n = (v1 - v0).cross(v2 - v0).normalize_or_zero();
        let tri_aabb = Aabb3::new(tri_min, tri_max);
        aabb_wcs = aabb_wcs.union(&tri_aabb);
        triangles.push(MeshTriangle {
            indices: tri.indices,
            normal: n,
            aabb: tri_aabb,
        });
    }
    SurfaceMesh3D::from_indexed_triangles(vertices_wcs, triangles, aabb_wcs)
}

/// Evaluates the exact analytical `f64` Drop-Cutter contact Z height (`z_tip_wcs` in mm) at an
/// arbitrary rotary angle `theta_deg`, axial coordinate `u_axial`, and cross-axis offset `c_cross`
/// directly against `base_mesh`'s `FlatBvh` without rebuilding or rotating the full mesh.
///
/// Uses exact oriented-strip interval bounds on `FlatBvhNode` AABBs with top-down highest-Z-first
/// traversal and zero heap allocations, achieving `< 1e-9 mm` agreement with
/// [`build_rotated_strip_mesh`] + [`crate::surface3d::drop_cutter_z_with_scratch`].
pub fn drop_cutter_rotary_at(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    u_axial: f64,
    c_cross: f64,
    theta_deg: f64,
    z_floor: f64,
) -> Option<f64> {
    if rot.axis == RotaryAxisKind::ZAxis
        || base_mesh.bvh.is_empty()
        || !u_axial.is_finite()
        || !c_cross.is_finite()
        || !theta_deg.is_finite()
        || !z_floor.is_finite()
    {
        return None;
    }

    let r_max = cutter.max_radius_mm();
    let r_pad = r_max + 1e-5;
    let r_xy_sq = (r_max + 1e-6) * (r_max + 1e-6);

    let angle_rad = (theta_deg * rot.direction.sign()).to_radians();
    let (sin_a, cos_a) = angle_rad.sin_cos();
    let cx = rot.center_wcs.x;
    let cy = rot.center_wcs.y;
    let cz = rot.center_wcs.z;
    let is_x_axis = rot.axis == RotaryAxisKind::XAxis;

    let (qx, qy) = if is_x_axis {
        (u_axial, cy + c_cross)
    } else {
        (cx + c_cross, u_axial)
    };
    let c_xy = DVec2::new(qx, qy);

    let u_lo = u_axial - r_pad;
    let u_hi = u_axial + r_pad;
    let c_lo = c_cross - r_pad;
    let c_hi = c_cross + r_pad;
    let z_min_cutoff = z_floor - cutter.stock_axial_mm.max(0.0) - 1e-4;

    let node_z_max_if_overlaps = |node: &FlatBvhNode, z_cut: f64| -> Option<f64> {
        if is_x_axis {
            let nu_min = f64::from(node.aabb_min[0]);
            let nu_max = f64::from(node.aabb_max[0]);
            if nu_max < u_lo || nu_min > u_hi {
                return None;
            }
            let dy0 = f64::from(node.aabb_min[1]) - cy;
            let dy1 = f64::from(node.aabb_max[1]) - cy;
            let dz0 = f64::from(node.aabb_min[2]) - cz;
            let dz1 = f64::from(node.aabb_max[2]) - cz;

            let (cy_lo, cy_hi) = if cos_a >= 0.0 {
                (dy0 * cos_a, dy1 * cos_a)
            } else {
                (dy1 * cos_a, dy0 * cos_a)
            };
            let neg_sin = -sin_a;
            let (cz_lo, cz_hi) = if neg_sin >= 0.0 {
                (dz0 * neg_sin, dz1 * neg_sin)
            } else {
                (dz1 * neg_sin, dz0 * neg_sin)
            };
            if cy_hi + cz_hi < c_lo || cy_lo + cz_lo > c_hi {
                return None;
            }

            let z_max =
                cz + if sin_a >= 0.0 {
                    dy1 * sin_a
                } else {
                    dy0 * sin_a
                } + if cos_a >= 0.0 {
                    dz1 * cos_a
                } else {
                    dz0 * cos_a
                };
            (z_max >= z_cut).then_some(z_max)
        } else {
            let nu_min = f64::from(node.aabb_min[1]);
            let nu_max = f64::from(node.aabb_max[1]);
            if nu_max < u_lo || nu_min > u_hi {
                return None;
            }
            let dx0 = f64::from(node.aabb_min[0]) - cx;
            let dx1 = f64::from(node.aabb_max[0]) - cx;
            let dz0 = f64::from(node.aabb_min[2]) - cz;
            let dz1 = f64::from(node.aabb_max[2]) - cz;

            let (cx_lo, cx_hi) = if cos_a >= 0.0 {
                (dx0 * cos_a, dx1 * cos_a)
            } else {
                (dx1 * cos_a, dx0 * cos_a)
            };
            let (cz_lo, cz_hi) = if sin_a >= 0.0 {
                (dz0 * sin_a, dz1 * sin_a)
            } else {
                (dz1 * sin_a, dz0 * sin_a)
            };
            if cx_hi + cz_hi < c_lo || cx_lo + cz_lo > c_hi {
                return None;
            }

            let neg_sin = -sin_a;
            let z_max =
                cz + if neg_sin >= 0.0 {
                    dx1 * neg_sin
                } else {
                    dx0 * neg_sin
                } + if cos_a >= 0.0 {
                    dz1 * cos_a
                } else {
                    dz0 * cos_a
                };
            (z_max >= z_cut).then_some(z_max)
        }
    };

    let rot_pt = |p: DVec3| -> DVec3 {
        if is_x_axis {
            let dy = p.y - cy;
            let dz = p.z - cz;
            DVec3::new(
                p.x,
                cy + dy * cos_a - dz * sin_a,
                cz + dy * sin_a + dz * cos_a,
            )
        } else {
            let dx = p.x - cx;
            let dz = p.z - cz;
            DVec3::new(
                cx + dx * cos_a + dz * sin_a,
                p.y,
                cz - dx * sin_a + dz * cos_a,
            )
        }
    };

    let nodes = base_mesh.bvh.nodes();
    let prim_indices = base_mesh.bvh.prim_indices();
    let root_z_max = node_z_max_if_overlaps(&nodes[0], z_min_cutoff)?;

    let mut best_z = f64::NEG_INFINITY;
    let mut hit_any = false;

    let mut stack: [(u32, f64); 64] = [(0, 0.0); 64];
    let mut sp = 1usize;
    stack[0] = (0, root_z_max);

    while sp > 0 {
        sp -= 1;
        let (node_idx, node_z_max) = stack[sp];
        if hit_any && node_z_max <= best_z {
            continue;
        }
        let node = &nodes[node_idx as usize];
        if node.is_leaf() {
            let first = node.left_or_first as usize;
            let end = first + node.tri_count as usize;
            for &pi in &prim_indices[first..end] {
                let tri = &base_mesh.triangles[pi as usize];
                let [i0, i1, i2] = tri.indices;
                let v0 = rot_pt(base_mesh.vertices[i0 as usize]);
                let v1 = rot_pt(base_mesh.vertices[i1 as usize]);
                let v2 = rot_pt(base_mesh.vertices[i2 as usize]);

                let tz_max = v0.z.max(v1.z).max(v2.z);
                if tz_max < z_min_cutoff || (hit_any && tz_max <= best_z) {
                    continue;
                }

                let tx_min = v0.x.min(v1.x).min(v2.x);
                let tx_max = v0.x.max(v1.x).max(v2.x);
                let ty_min = v0.y.min(v1.y).min(v2.y);
                let ty_max = v0.y.max(v1.y).max(v2.y);
                let dx = if qx < tx_min {
                    tx_min - qx
                } else if qx > tx_max {
                    qx - tx_max
                } else {
                    0.0
                };
                let dy = if qy < ty_min {
                    ty_min - qy
                } else if qy > ty_max {
                    qy - ty_max
                } else {
                    0.0
                };
                if dx * dx + dy * dy > r_xy_sq {
                    continue;
                }

                if let Some(z_tri) = drop_cutter_triangle(cutter, c_xy, v0, v1, v2)
                    && z_tri.is_finite()
                    && (!hit_any || z_tri > best_z)
                {
                    best_z = z_tri;
                    hit_any = true;
                }
            }
        } else {
            let z_cut = if hit_any {
                best_z.max(z_min_cutoff)
            } else {
                z_min_cutoff
            };
            let left_idx = node.left_or_first;
            let right_idx = left_idx + 1;
            let zl = node_z_max_if_overlaps(&nodes[left_idx as usize], z_cut);
            let zr = node_z_max_if_overlaps(&nodes[right_idx as usize], z_cut);
            match (zl, zr) {
                (Some(l), Some(r)) => {
                    if sp + 2 <= stack.len() {
                        // Push smaller Z first so larger Z is popped and visited first
                        if l >= r {
                            stack[sp] = (right_idx, r);
                            stack[sp + 1] = (left_idx, l);
                        } else {
                            stack[sp] = (left_idx, l);
                            stack[sp + 1] = (right_idx, r);
                        }
                        sp += 2;
                    }
                }
                (Some(l), None) => {
                    if sp < stack.len() {
                        stack[sp] = (left_idx, l);
                        sp += 1;
                    }
                }
                (None, Some(r)) => {
                    if sp < stack.len() {
                        stack[sp] = (right_idx, r);
                        sp += 1;
                    }
                }
                (None, None) => {}
            }
        }
    }

    if hit_any {
        Some((best_z + cutter.stock_axial_mm).max(z_floor))
    } else {
        None
    }
}

/// Evaluates the exact `f64` Drop-Cutter tip height relative to `rot.center_wcs.z` at
/// `(u_axial, c_cross, theta_deg)`, floored at `r_floor`.
#[inline]
pub fn drop_cutter_rotary_z_rel(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    u_axial: f64,
    c_cross: f64,
    theta_deg: f64,
    r_floor: f64,
) -> f64 {
    let z_floor = rot.center_wcs.z + r_floor;
    let z_hit = drop_cutter_rotary_at(base_mesh, rot, cutter, u_axial, c_cross, theta_deg, z_floor);
    (z_hit.unwrap_or(z_floor) - rot.center_wcs.z).max(r_floor)
}

/// Estimates the effective flank slope factor `cos(φ_eff) ∈ [0.45, 1.0]` for slope-compensated
/// cusp-height stepover scaling (`Δ_step = Δ_flat * cos(φ_eff)`).
///
/// - When `circumferential == false` (spiral patterns stepping along `u`), measures axial flank
///   slopes `|dr/du|`.
/// - When `circumferential == true` (axial-parallel / flowline patterns stepping along `θ`),
///   measures circumferential flank slopes `|dr / (r * dθ)|`.
#[allow(clippy::too_many_arguments)]
pub fn estimate_rotary_flank_cos_phi(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    u_start: f64,
    u_end: f64,
    start_deg: f64,
    end_deg: f64,
    is_full_turn: bool,
    r_floor: f64,
    circumferential: bool,
) -> f64 {
    let nu = (((u_end - u_start) / 0.8).ceil() as usize).clamp(8, 80);
    let du = (u_end - u_start) / (nu as f64);
    let span_deg = (end_deg - start_deg).max(1e-3);
    let nt = ((span_deg / 4.0).ceil() as usize).clamp(8, 90);
    let d_theta = span_deg / (nt as f64);
    let d_th_rad = d_theta.to_radians().max(1e-4);
    let theta_count = if is_full_turn { nt } else { nt + 1 };

    let grid: Vec<Vec<f64>> = (0..theta_count)
        .into_par_iter()
        .map(|it| {
            let th = start_deg + (it as f64) * d_theta;
            (0..=nu)
                .map(|iu| {
                    let u = u_start + (iu as f64) * du;
                    drop_cutter_rotary_z_rel(base_mesh, rot, cutter, u, 0.0, th, r_floor)
                })
                .collect()
        })
        .collect();

    let mut slopes = Vec::with_capacity(theta_count * nu);
    if circumferential {
        for it in 0..theta_count {
            let it_next = if is_full_turn {
                (it + 1) % theta_count
            } else if it + 1 < theta_count {
                it + 1
            } else {
                continue;
            };
            for iu in 0..=nu {
                let r_avg = (0.5 * (grid[it][iu] + grid[it_next][iu])).max(1.0);
                let s = (grid[it_next][iu] - grid[it][iu]).abs() / (r_avg * d_th_rad);
                if s.is_finite() {
                    slopes.push(s);
                }
            }
        }
    } else if du > 1e-6 {
        for row in &grid {
            for iu in 0..nu {
                let s = (row[iu + 1] - row[iu]).abs() / du;
                if s.is_finite() {
                    slopes.push(s);
                }
            }
        }
    }

    if slopes.is_empty() {
        return 1.0;
    }
    slopes.sort_by(f64::total_cmp);
    // Use 90th percentile slope so steep helical flanks/bevels get tight cusp control
    // without letting isolated 90° step walls collapse the entire stepover.
    let p90_idx = ((slopes.len() as f64 - 1.0) * 0.90).round() as usize;
    let slope_eff = slopes[p90_idx.min(slopes.len() - 1)];
    (1.0 / (1.0 + slope_eff * slope_eff).sqrt()).clamp(0.45, 1.0)
}

/// Returns `-1.0` if `base_mesh` is globally inward-wound (negative signed volume, as in some
/// legacy STL fixtures) or `+1.0` for standard outward-oriented meshes.
fn mesh_outward_winding_sign(base_mesh: &SurfaceMesh3D) -> f64 {
    let mut signed_vol6 = 0.0_f64;
    for tri in &base_mesh.triangles {
        let [i0, i1, i2] = tri.indices;
        let v0 = base_mesh.vertices[i0 as usize];
        let v1 = base_mesh.vertices[i1 as usize];
        let v2 = base_mesh.vertices[i2 as usize];
        signed_vol6 += v0.dot(v1.cross(v2));
    }
    if signed_vol6 < -1e-6 { -1.0 } else { 1.0 }
}

/// Returns `true` when `base_mesh` contains significant steep circumferential flanks (`> ~70°`
/// from radial) or genuine circumferential overhangs (`n · e_r <= 0`) inside the axial window
/// `[u_start - margin_mm, u_end + margin_mm]`.
///
/// Used by [`crate::strategies::rotary4axis_sim`] so multi-offset circumferential flank probing
/// activates cleanly on overhanging helical blades while leaving non-overhanging flutes/cams
/// (and Phase 5 golden snapshots) 100% bit-identical.
pub fn mesh_has_steep_circumferential_overhang(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    u_start: f64,
    u_end: f64,
    margin_mm: f64,
) -> bool {
    if rot.axis == RotaryAxisKind::ZAxis || base_mesh.triangles.is_empty() {
        return false;
    }
    let winding_sign = mesh_outward_winding_sign(base_mesh);
    let (_, _, u_vec) = rot.axis.basis();
    let u_lo = u_start - margin_mm.max(0.0);
    let u_hi = u_end + margin_mm.max(0.0);
    let mut steep_area = 0.0_f64;

    for tri in &base_mesh.triangles {
        let [i0, i1, i2] = tri.indices;
        let v0 = base_mesh.vertices[i0 as usize];
        let v1 = base_mesh.vertices[i1 as usize];
        let v2 = base_mesh.vertices[i2 as usize];
        let centroid = (v0 + v1 + v2) / 3.0;
        let u = centroid.dot(u_vec);
        if u < u_lo || u > u_hi {
            continue;
        }
        let rel = centroid - rot.center_wcs;
        let radial_vec = rel - u_vec * rel.dot(u_vec);
        let r_len = radial_vec.length();
        if r_len <= 0.25 {
            continue;
        }
        let e_r = radial_vec / r_len;
        let cross = (v1 - v0).cross(v2 - v0);
        let area2 = cross.length();
        if area2 <= 1e-8 {
            continue;
        }
        let n = (cross / area2) * winding_sign;
        let n_perp = n - u_vec * n.dot(u_vec);
        let n_perp_len = n_perp.length();
        if n_perp_len <= 0.25 {
            continue;
        }
        let cos_radial = n_perp.dot(e_r) / n_perp_len;
        // Steep circumferential wall (> ~70° from radial) or true circumferential overhang (<= 0)
        if cos_radial < 0.35 {
            steep_area += 0.5 * area2;
            if steep_area > 1.0 {
                return true;
            }
        }
    }
    false
}

/// Fast post-generation reachability check detecting axial (`u`-direction) overhang / undercut
/// surface area on `base_mesh` inside `[u_start, u_end]` that cannot be reached from any vertical
/// cylindrical pose `(u, c, θ)`.
///
/// Returns an actionable non-blocking warning suggesting an indexed `Undercut3D` (`LollipopEndmill`)
/// pass whenever significant unreachable axial-undercut area (`>= 2.0 mm²`) is found.
pub fn detect_rotary_axial_undercut_warning(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    u_start: f64,
    u_end: f64,
    r_floor: f64,
) -> Option<String> {
    if rot.axis == RotaryAxisKind::ZAxis || base_mesh.triangles.is_empty() {
        return None;
    }
    let winding_sign = mesh_outward_winding_sign(base_mesh);
    let (_, _, u_vec) = rot.axis.basis();
    let mut mesh_u_min = f64::INFINITY;
    let mut mesh_u_max = f64::NEG_INFINITY;
    for &v in &base_mesh.vertices {
        let u = v.dot(u_vec);
        mesh_u_min = mesh_u_min.min(u);
        mesh_u_max = mesh_u_max.max(u);
    }
    if !mesh_u_min.is_finite() || !mesh_u_max.is_finite() {
        return None;
    }

    let u_lo = (u_start + 0.5).max(mesh_u_min + 0.35);
    let u_hi = (u_end - 0.5).min(mesh_u_max - 0.35);
    if u_hi <= u_lo {
        return None;
    }

    let max_ray_t = (rot.stock_outer_radius_mm * 2.5).max(100.0);
    // Test a ±40° fan of perpendicular-plane spindle approach rays `(e_r, e_theta)`
    const APPROACH_ANGLES_DEG: [f64; 5] = [-40.0, -20.0, 0.0, 20.0, 40.0];
    let approach_sc: [(f64, f64); 5] = APPROACH_ANGLES_DEG.map(|deg| deg.to_radians().sin_cos());

    let unreachable_area: f64 = base_mesh
        .triangles
        .par_iter()
        .map(|tri| {
            let [i0, i1, i2] = tri.indices;
            let v0 = base_mesh.vertices[i0 as usize];
            let v1 = base_mesh.vertices[i1 as usize];
            let v2 = base_mesh.vertices[i2 as usize];
            let centroid = (v0 + v1 + v2) / 3.0;
            let u = centroid.dot(u_vec);
            if u < u_lo || u > u_hi {
                return 0.0;
            }
            let rel = centroid - rot.center_wcs;
            let radial_vec = rel - u_vec * rel.dot(u_vec);
            let r_len = radial_vec.length();
            if r_len <= r_floor + 0.25 {
                return 0.0;
            }
            let cross = (v1 - v0).cross(v2 - v0);
            let area2 = cross.length();
            if area2 <= 1e-6 {
                return 0.0;
            }
            let n = (cross / area2) * winding_sign;
            let e_r = radial_vec / r_len;
            let n_u_abs = n.dot(u_vec).abs();
            let n_r = n.dot(e_r);

            // Only evaluate triangles with an axial orientation component that either face inward
            // toward the rotary axis or sit beneath an axial roof (normal non-radial).
            if n_u_abs < 0.15 || n_r > 0.35 {
                return 0.0;
            }

            let e_theta = u_vec.cross(e_r).normalize_or_zero();
            let origin = centroid + n * 0.05;

            // Reachable if ANY vertical cylindrical approach ray in the cross-sectional plane
            // faces the triangle (`n · dir >= -0.10`) AND escapes without hitting an overhanging roof.
            let mut reachable = false;
            for &(sin_a, cos_a) in &approach_sc {
                let dir = (e_r * cos_a + e_theta * sin_a).normalize_or_zero();
                if n.dot(dir) >= -0.10 && base_mesh.raycast(origin, dir, max_ray_t).is_none() {
                    reachable = true;
                    break;
                }
            }
            if reachable { 0.0 } else { 0.5 * area2 }
        })
        .sum();

    if unreachable_area >= 2.0 {
        Some(format!(
            "Rotary toolpath detected ~{unreachable_area:.1} mm² of axial overhang/undercut surface area unreachable from vertical cylindrical poses; consider adding an indexed Undercut3D pass with a LollipopEndmill."
        ))
    } else {
        None
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{DisplayUnit, approx_eq};

    use super::*;

    fn dummy_resolved(
        axis: RotaryAxisKind,
        dir: RotaryDirection,
        center: DVec3,
    ) -> ResolvedRotaryConfig {
        ResolvedRotaryConfig {
            mode: RotaryDriveMode::Simultaneous4Axis,
            axis,
            direction: dir,
            axis_letter: axis.default_letter(),
            center_wcs: center,
            stock_u_min: -40.0,
            stock_u_max: 40.0,
            stock_outer_radius_mm: 22.0,
            clearance_radius_mm: 32.0,
            chuck_end_positive: false,
            chuck_u_min: -60.0,
            chuck_u_max: -40.0,
            chuck_radius_mm: 32.5,
            unwind_mode: RotaryUnwindMode::ContinuousMultiTurn,
        }
    }

    #[test]
    fn cylindrical_transforms_and_axis_rotations_roundtrip_exact_across_a_b_c() {
        let test_pt = DVec3::new(14.5, -7.25, 21.75);
        for axis in RotaryAxisKind::ALL {
            let (e1, e2, u) = axis.basis();
            // Verify strict right-handedness `e1 x e2 == u`
            assert!(e1.cross(e2).distance(u) < 1e-12);

            for dir in RotaryDirection::ALL {
                let rot = dummy_resolved(axis, dir, DVec3::new(1.5, -2.0, 3.0));
                let (u_val, theta, r) = rot.wcs_to_cylindrical(test_pt);
                let back = rot.cylindrical_to_wcs(u_val, theta, r);
                assert!(
                    back.distance(test_pt) < 1e-9,
                    "roundtrip failed for {axis:?}/{dir:?}: {back:?} vs {test_pt:?}"
                );

                // Rotating by +37° then -37° returns exact original point
                let p_fwd = rot.rotate_point_wcs(test_pt, 37.0);
                let p_back = rot.rotate_point_wcs(p_fwd, -37.0);
                assert!(p_back.distance(test_pt) < 1e-9);

                // Rotating by +90° preserves axial coordinate `u` and radius `r`
                let p90 = rot.rotate_point_wcs(test_pt, 90.0);
                let (u90, _, r90) = rot.wcs_to_cylindrical(p90);
                assert!(approx_eq(u90, u_val, 1e-9));
                assert!(approx_eq(r90, r, 1e-9));
            }
        }
    }

    #[test]
    fn rotary_config_clearance_and_index_angle_oracles_are_exact() {
        // RotaryConfig resolves clearance radius cleanly around cylindrical stock
        let cfg = RotaryConfig {
            mode: RotaryDriveMode::Indexed3Plus1,
            axis: RotaryAxisKind::XAxis,
            clearance_radial_offset: ParamLength::from_mm(12.0),
            ..Default::default()
        };
        let cyl = ResolvedStockGeometry::Cylinder {
            axis: StockAxis::X,
            center_wcs: DVec3::ZERO,
            radius_mm: 22.0,
            min_axial_mm: -40.0,
            max_axial_mm: 40.0,
            aabb_wcs: Aabb3::new(
                DVec3::new(-40.0, -22.0, -22.0),
                DVec3::new(40.0, 22.0, 22.0),
            ),
        };
        let resolved = cfg
            .resolve(&cyl, &EvalContext::new(DisplayUnit::Millimeter))
            .unwrap();
        assert!(approx_eq(resolved.stock_outer_radius_mm, 22.0, 1e-9));
        assert!(approx_eq(resolved.clearance_radius_mm, 34.0, 1e-9));
        assert_eq!(resolved.axis_letter, 'A');

        // Face-normal index angle oracle for X (A-axis) and Y (B-axis)
        let rot_a = dummy_resolved(
            RotaryAxisKind::XAxis,
            RotaryDirection::RightHandCcw,
            DVec3::ZERO,
        );
        assert!(approx_eq(
            compute_index_angle_for_normal(DVec3::Z, &rot_a).unwrap(),
            0.0,
            1e-9
        ));
        assert!(approx_eq(
            compute_index_angle_for_normal(DVec3::Y, &rot_a).unwrap(),
            90.0,
            1e-9
        ));
        assert!(approx_eq(
            compute_index_angle_for_normal(-DVec3::Z, &rot_a).unwrap(),
            180.0,
            1e-9
        ));
        assert!(approx_eq(
            compute_index_angle_for_normal(-DVec3::Y, &rot_a).unwrap(),
            270.0,
            1e-9
        ));
        assert!(compute_index_angle_for_normal(DVec3::X, &rot_a).is_none());

        // Verify rotating +Y face normal by +90 deg lands directly on +Z
        let rotated_n = rot_a.rotate_point_wcs(DVec3::Y, 90.0);
        assert!(rotated_n.distance(DVec3::Z) < 1e-9);

        let rot_b = dummy_resolved(
            RotaryAxisKind::YAxis,
            RotaryDirection::RightHandCcw,
            DVec3::ZERO,
        );
        let ang_b =
            compute_index_angle_for_normal(DVec3::new(1.0, 0.0, 1.0).normalize(), &rot_b).unwrap();
        let rotated_b = rot_b.rotate_point_wcs(DVec3::new(1.0, 0.0, 1.0).normalize(), ang_b);
        assert!(rotated_b.distance(DVec3::Z) < 1e-9);
    }

    #[test]
    fn drop_cutter_rotary_at_matches_rotated_strip_mesh_exact() {
        use chip_cad::{MeshImportOptions, load_mesh_cad_document};
        use chip_fixtures::sample_helical_flute_shaft_stl_bytes;
        use chip_tools::CutterGeometry;

        use crate::surface3d::drop_cutter_z_with_scratch;

        let stl = sample_helical_flute_shaft_stl_bytes();
        let doc = load_mesh_cad_document("flute.stl", &stl, &MeshImportOptions::default()).unwrap();
        let base_mesh_x = SurfaceMesh3D::from_cad_document(&doc, &[], &RigidFrame3::IDENTITY);
        // Also build a Y-axis rotated version of the shaft (swap X <-> Y) with a non-zero center offset
        let shift = DVec3::new(1.25, -2.5, 3.5);
        let y_verts: Vec<DVec3> = base_mesh_x
            .vertices
            .iter()
            .map(|&v| DVec3::new(v.y, v.x, v.z) + shift)
            .collect();
        let y_tris: Vec<MeshTriangle> = base_mesh_x
            .triangles
            .iter()
            .map(|t| {
                let v0 = y_verts[t.indices[0] as usize];
                let v1 = y_verts[t.indices[1] as usize];
                let v2 = y_verts[t.indices[2] as usize];
                MeshTriangle {
                    indices: t.indices,
                    normal: (v1 - v0).cross(v2 - v0).normalize_or_zero(),
                    aabb: Aabb3::from_points([v0, v1, v2]),
                }
            })
            .collect();
        let y_aabb = y_tris
            .iter()
            .fold(Aabb3::EMPTY, |acc, t| acc.union(&t.aabb));
        let base_mesh_y = SurfaceMesh3D::from_indexed_triangles(y_verts, y_tris, y_aabb);

        let cutters = [
            SurfaceCutterProfile::from_tool_geometry(
                &CutterGeometry::BallEndmill {
                    diameter_mm: 6.0,
                    flute_length_mm: 24.0,
                    overall_length_mm: 60.0,
                    shank_diameter_mm: 6.0,
                    flutes: 2,
                },
                0.0,
                0.0,
            ),
            SurfaceCutterProfile::from_tool_geometry(
                &CutterGeometry::BullNoseEndmill {
                    diameter_mm: 8.0,
                    corner_radius_mm: 1.5,
                    flute_length_mm: 24.0,
                    overall_length_mm: 60.0,
                    shank_diameter_mm: 8.0,
                    flutes: 3,
                },
                0.2,
                0.15,
            ),
            SurfaceCutterProfile::from_tool_geometry(
                &CutterGeometry::FlatEndmill {
                    diameter_mm: 6.0,
                    flute_length_mm: 24.0,
                    overall_length_mm: 60.0,
                    shank_diameter_mm: 6.0,
                    flutes: 3,
                },
                0.0,
                0.0,
            ),
        ];

        for (axis, mesh, center, u_vals) in [
            (
                RotaryAxisKind::XAxis,
                &base_mesh_x,
                DVec3::ZERO,
                [-26.0_f64, -12.3, 0.0, 14.7, 25.5],
            ),
            (
                RotaryAxisKind::YAxis,
                &base_mesh_y,
                shift,
                [
                    -26.0 + shift.y,
                    -12.3 + shift.y,
                    shift.y,
                    14.7 + shift.y,
                    25.5 + shift.y,
                ],
            ),
        ] {
            for dir in RotaryDirection::ALL {
                let rot = dummy_resolved(axis, dir, center);
                let z_floor = center.z + 2.0;
                for cutter in &cutters {
                    let r_max = cutter.max_radius_mm();
                    for &theta_deg in &[0.0_f64, 17.5, 45.0, 123.4, 210.0, 312.7] {
                        let rot_mesh = build_rotated_strip_mesh(
                            mesh,
                            &rot,
                            theta_deg,
                            u_vals[0] - 2.0,
                            u_vals[u_vals.len() - 1] + 2.0,
                            5.0 + r_max,
                            z_floor - cutter.stock_axial_mm,
                        );
                        let mut scratch = Vec::with_capacity(64);
                        for &u in &u_vals {
                            for &c_cross in &[-3.5_f64, 0.0, 2.75] {
                                let (qx, qy) = match axis {
                                    RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => {
                                        (u, center.y + c_cross)
                                    }
                                    RotaryAxisKind::YAxis => (center.x + c_cross, u),
                                };
                                let z_ref = drop_cutter_z_with_scratch(
                                    &rot_mesh,
                                    cutter,
                                    qx,
                                    qy,
                                    z_floor,
                                    &mut scratch,
                                );
                                let z_direct = drop_cutter_rotary_at(
                                    mesh,
                                    rot_ref(&rot),
                                    cutter,
                                    u,
                                    c_cross,
                                    theta_deg,
                                    z_floor,
                                );
                                match (z_ref, z_direct) {
                                    (Some(a), Some(b)) => {
                                        assert!(
                                            (a - b).abs() < 1e-9,
                                            "mismatch axis={axis:?} dir={dir:?} th={theta_deg} u={u} c={c_cross}: ref={a} direct={b} diff={}",
                                            (a - b).abs()
                                        );
                                    }
                                    (None, None) => {}
                                    (a, b) => panic!(
                                        "hit/miss mismatch axis={axis:?} dir={dir:?} th={theta_deg} u={u} c={c_cross}: ref={a:?} direct={b:?}"
                                    ),
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[inline]
    fn rot_ref(r: &ResolvedRotaryConfig) -> &ResolvedRotaryConfig {
        r
    }

    #[test]
    fn m81_resolve_rotary_window_all_four_modes_containment_and_chuck_guard_on_x_and_y_axes() {
        use chip_cad::{CadKernel, MonstertruckKernel};
        use chip_fixtures::tutorial_indexed_4side_block_step;
        use chip_math::{ArcPolyline2D, ArcSegment2D, DEFAULT_TESS_CHORD_TOL_MM};

        use crate::operation::{Operation, OperationKind};

        // Helper building a simple 1-triangle dummy mesh spanning `u ∈ [-20.0, +25.0]` along `axis`
        let make_mesh = |axis: RotaryAxisKind| -> SurfaceMesh3D {
            let (v0, v1, v2) = match axis {
                RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => (
                    DVec3::new(-20.0, -5.0, 12.0),
                    DVec3::new(25.0, 5.0, 12.0),
                    DVec3::new(0.0, 0.0, 18.0),
                ),
                RotaryAxisKind::YAxis => (
                    DVec3::new(-5.0, -20.0, 12.0),
                    DVec3::new(5.0, 25.0, 12.0),
                    DVec3::new(0.0, 0.0, 18.0),
                ),
            };
            let verts = vec![v0, v1, v2];
            let aabb = Aabb3::from_points([v0, v1, v2]);
            let tri = MeshTriangle {
                indices: [0, 1, 2],
                normal: (v1 - v0).cross(v2 - v0).normalize_or_zero(),
                aabb,
            };
            SurfaceMesh3D::from_indexed_triangles(verts, vec![tri], aabb)
        };

        let tool_d = 6.0_f64;
        let tool_r = 3.0_f64;

        for axis in [RotaryAxisKind::XAxis, RotaryAxisKind::YAxis] {
            let mesh = make_mesh(axis);
            for chuck_pos in [false, true] {
                let mut rot = dummy_resolved(axis, RotaryDirection::RightHandCcw, DVec3::ZERO);
                rot.chuck_end_positive = chuck_pos;
                if chuck_pos {
                    rot.chuck_u_min = 40.0;
                    rot.chuck_u_max = 60.0;
                } else {
                    rot.chuck_u_min = -60.0;
                    rot.chuck_u_max = -40.0;
                }
                let expected_guard = if chuck_pos {
                    40.0 - (tool_r + ROTARY_CHUCK_GUARD_CLEARANCE_MM)
                } else {
                    -40.0 + (tool_r + ROTARY_CHUCK_GUARD_CLEARANCE_MM)
                };

                // 1. StockBounds: nominal = [-40, +40], chuck end clamped with warning
                let b_stock = RotaryAxialBoundaryConfig {
                    axial_boundary_mode: RotaryAxialBoundaryMode::StockBounds,
                    ..Default::default()
                };
                let w_stock = resolve_rotary_window(&RotaryWindowInput {
                    rot: &rot,
                    frame: &RigidFrame3::IDENTITY,
                    doc: None,
                    base_mesh: &mesh,
                    selections: &[],
                    extracted_selection: None,
                    boundary: &b_stock,
                    tool_diameter_mm: tool_d,
                    top_z_wcs: 22.0,
                    bottom_z_wcs: 5.0,
                    op_name: "test_stock",
                })
                .unwrap();
                assert!(approx_eq(w_stock.nominal_u_min, -40.0, 1e-9));
                assert!(approx_eq(w_stock.nominal_u_max, 40.0, 1e-9));
                assert!(approx_eq(w_stock.chuck_guard_u, expected_guard, 1e-9));
                assert!(w_stock.clamped_by_chuck);
                assert!(!w_stock.warnings.is_empty());
                if chuck_pos {
                    assert!(approx_eq(w_stock.u_start, -40.0, 1e-9));
                    assert!(approx_eq(w_stock.u_end, expected_guard, 1e-9));
                } else {
                    assert!(approx_eq(w_stock.u_start, expected_guard, 1e-9));
                    assert!(approx_eq(w_stock.u_end, 40.0, 1e-9));
                }
                assert!(approx_eq(w_stock.r_floor, 5.0, 1e-9));
                assert!(approx_eq(w_stock.r_ceiling, 22.0, 1e-9));

                // 2. ModelBounds + ToolInside / ToolCenter / ToolOutside exact ±tool_r shifts
                for (containment, d_lo, d_hi) in [
                    (ToolContainmentMode::ToolInsideBoundary, tool_r, -tool_r),
                    (ToolContainmentMode::ToolCenterOnBoundary, 0.0, 0.0),
                    (ToolContainmentMode::ToolOutsideBoundary, -tool_r, tool_r),
                ] {
                    let b_model = RotaryAxialBoundaryConfig {
                        axial_boundary_mode: RotaryAxialBoundaryMode::ModelBounds,
                        axial_containment: containment,
                        ..Default::default()
                    };
                    let w_model = resolve_rotary_window(&RotaryWindowInput {
                        rot: &rot,
                        frame: &RigidFrame3::IDENTITY,
                        doc: None,
                        base_mesh: &mesh,
                        selections: &[],
                        extracted_selection: None,
                        boundary: &b_model,
                        tool_diameter_mm: tool_d,
                        top_z_wcs: 16.0, // lowered below stock_outer_radius_mm = 22.0
                        bottom_z_wcs: 4.0,
                        op_name: "test_model",
                    })
                    .unwrap();
                    assert!(approx_eq(w_model.u_start, -20.0 + d_lo, 1e-9));
                    assert!(approx_eq(w_model.u_end, 25.0 + d_hi, 1e-9));
                    assert!(!w_model.clamped_by_chuck);
                    assert!(approx_eq(w_model.r_ceiling, 16.0, 1e-9));
                    assert!(approx_eq(w_model.r_floor, 4.0, 1e-9));
                }

                // 3. Signed negative margin extends past model for shoulder overrun, AND
                //    excessive negative margin toward chuck hard-clamps cleanly at chuck_guard_u!
                let b_overrun = RotaryAxialBoundaryConfig {
                    axial_boundary_mode: RotaryAxialBoundaryMode::ModelBounds,
                    axial_containment: ToolContainmentMode::ToolCenterOnBoundary,
                    axial_margin_start: ParamLength::from_mm(-5.0),
                    axial_margin_end: ParamLength::from_mm(-4.0),
                    ..Default::default()
                };
                let w_over = resolve_rotary_window(&RotaryWindowInput {
                    rot: &rot,
                    frame: &RigidFrame3::IDENTITY,
                    doc: None,
                    base_mesh: &mesh,
                    selections: &[],
                    extracted_selection: None,
                    boundary: &b_overrun,
                    tool_diameter_mm: tool_d,
                    top_z_wcs: 22.0,
                    bottom_z_wcs: 0.0,
                    op_name: "test_overrun",
                })
                .unwrap();
                assert!(approx_eq(w_over.u_start, -25.0, 1e-9));
                assert!(approx_eq(w_over.u_end, 29.0, 1e-9));
                assert!(!w_over.clamped_by_chuck);

                let b_extreme = RotaryAxialBoundaryConfig {
                    axial_boundary_mode: RotaryAxialBoundaryMode::ModelBounds,
                    axial_containment: ToolContainmentMode::ToolOutsideBoundary,
                    axial_margin_start: ParamLength::from_mm(-30.0),
                    axial_margin_end: ParamLength::from_mm(-30.0),
                    ..Default::default()
                };
                let w_ext = resolve_rotary_window(&RotaryWindowInput {
                    rot: &rot,
                    frame: &RigidFrame3::IDENTITY,
                    doc: None,
                    base_mesh: &mesh,
                    selections: &[],
                    extracted_selection: None,
                    boundary: &b_extreme,
                    tool_diameter_mm: tool_d,
                    top_z_wcs: 22.0,
                    bottom_z_wcs: 0.0,
                    op_name: "test_extreme",
                })
                .unwrap();
                assert!(w_ext.clamped_by_chuck);
                if chuck_pos {
                    assert!(approx_eq(w_ext.u_end, expected_guard, 1e-9));
                } else {
                    assert!(approx_eq(w_ext.u_start, expected_guard, 1e-9));
                }

                // 4. SelectedGeometry: extracted contour projects cleanly onto `u_vec`,
                //    and empty selections fall back to ModelBounds with a warning
                let (p0, p1) = match axis {
                    RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => {
                        (DVec2::new(-8.5, -4.0), DVec2::new(14.5, 4.0))
                    }
                    RotaryAxisKind::YAxis => (DVec2::new(-4.0, -8.5), DVec2::new(4.0, 14.5)),
                };
                let extracted = ExtractedSelectionSummary {
                    features: vec![ExtractedFeature25D::Contour {
                        polyline_wcs: ArcPolyline2D {
                            segments: vec![ArcSegment2D::Line { start: p0, end: p1 }],
                            closed: false,
                        },
                        min_z_wcs: 0.0,
                        max_z_wcs: 10.0,
                        reverse: false,
                        side_flip: false,
                        source_edge_ids: Vec::new(),
                        adjacent_lower_floor_z_wcs: None,
                    }],
                    ..Default::default()
                };
                let b_sel = RotaryAxialBoundaryConfig {
                    axial_boundary_mode: RotaryAxialBoundaryMode::SelectedGeometry,
                    ..Default::default()
                };
                let w_sel = resolve_rotary_window(&RotaryWindowInput {
                    rot: &rot,
                    frame: &RigidFrame3::IDENTITY,
                    doc: None,
                    base_mesh: &mesh,
                    selections: &[],
                    extracted_selection: Some(&extracted),
                    boundary: &b_sel,
                    tool_diameter_mm: tool_d,
                    top_z_wcs: 22.0,
                    bottom_z_wcs: 0.0,
                    op_name: "test_sel",
                })
                .unwrap();
                assert!(approx_eq(w_sel.u_start, -8.5, 1e-9));
                assert!(approx_eq(w_sel.u_end, 14.5, 1e-9));
                assert!(w_sel.warnings.is_empty());

                let w_sel_empty = resolve_rotary_window(&RotaryWindowInput {
                    rot: &rot,
                    frame: &RigidFrame3::IDENTITY,
                    doc: None,
                    base_mesh: &mesh,
                    selections: &[],
                    extracted_selection: None,
                    boundary: &b_sel,
                    tool_diameter_mm: tool_d,
                    top_z_wcs: 22.0,
                    bottom_z_wcs: 0.0,
                    op_name: "test_sel_empty",
                })
                .unwrap();
                assert!(approx_eq(w_sel_empty.u_start, -20.0, 1e-9));
                assert!(approx_eq(w_sel_empty.u_end, 25.0, 1e-9));
                assert!(
                    w_sel_empty
                        .warnings
                        .iter()
                        .any(|w| w.contains("falling back to Model Bounds"))
                );

                // 5. ExplicitRange: exact bounds, inverted bounds normalized with warning, empty errors
                let b_exp = RotaryAxialBoundaryConfig {
                    axial_boundary_mode: RotaryAxialBoundaryMode::ExplicitRange,
                    explicit_u_start: ParamLength::from_mm(-12.0),
                    explicit_u_end: ParamLength::from_mm(18.0),
                    ..Default::default()
                };
                let w_exp = resolve_rotary_window(&RotaryWindowInput {
                    rot: &rot,
                    frame: &RigidFrame3::IDENTITY,
                    doc: None,
                    base_mesh: &mesh,
                    selections: &[],
                    extracted_selection: None,
                    boundary: &b_exp,
                    tool_diameter_mm: tool_d,
                    top_z_wcs: 22.0,
                    bottom_z_wcs: 0.0,
                    op_name: "test_exp",
                })
                .unwrap();
                assert!(approx_eq(w_exp.u_start, -12.0, 1e-9));
                assert!(approx_eq(w_exp.u_end, 18.0, 1e-9));

                let b_exp_inv = RotaryAxialBoundaryConfig {
                    axial_boundary_mode: RotaryAxialBoundaryMode::ExplicitRange,
                    explicit_u_start: ParamLength::from_mm(18.0),
                    explicit_u_end: ParamLength::from_mm(-12.0),
                    ..Default::default()
                };
                let w_exp_inv = resolve_rotary_window(&RotaryWindowInput {
                    rot: &rot,
                    frame: &RigidFrame3::IDENTITY,
                    doc: None,
                    base_mesh: &mesh,
                    selections: &[],
                    extracted_selection: None,
                    boundary: &b_exp_inv,
                    tool_diameter_mm: tool_d,
                    top_z_wcs: 22.0,
                    bottom_z_wcs: 0.0,
                    op_name: "test_exp_inv",
                })
                .unwrap();
                assert!(approx_eq(w_exp_inv.u_start, -12.0, 1e-9));
                assert!(approx_eq(w_exp_inv.u_end, 18.0, 1e-9));
                assert!(!w_exp_inv.warnings.is_empty());

                let b_exp_empty = RotaryAxialBoundaryConfig {
                    axial_boundary_mode: RotaryAxialBoundaryMode::ExplicitRange,
                    explicit_u_start: ParamLength::from_mm(5.0),
                    explicit_u_end: ParamLength::from_mm(5.0),
                    ..Default::default()
                };
                let err_empty = resolve_rotary_window(&RotaryWindowInput {
                    rot: &rot,
                    frame: &RigidFrame3::IDENTITY,
                    doc: None,
                    base_mesh: &mesh,
                    selections: &[],
                    extracted_selection: None,
                    boundary: &b_exp_empty,
                    tool_diameter_mm: tool_d,
                    top_z_wcs: 22.0,
                    bottom_z_wcs: 0.0,
                    op_name: "test_exp_empty",
                })
                .unwrap_err();
                assert!(matches!(err_empty, CamError::InvalidRotarySetup { .. }));
            }
        }

        // 6. SelectedGeometry with real 3D STEP faces (including non-horizontal side/bevel face)
        let step = tutorial_indexed_4side_block_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("block.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let target_face = doc
            .faces
            .iter()
            .find(|f| (f.aabb_mm.max.x - f.aabb_mm.min.x) > 5.0)
            .unwrap();
        let binding = doc.bind_face(target_face.id).unwrap();
        let rot_x = dummy_resolved(
            RotaryAxisKind::XAxis,
            RotaryDirection::RightHandCcw,
            DVec3::ZERO,
        );
        let projected = project_selections_onto_rotary_axis(
            Some(&doc),
            &RigidFrame3::IDENTITY,
            &[GeometrySelectionItem::PlanarFace { binding }],
            None,
            &rot_x,
        )
        .unwrap();
        assert!(approx_eq(projected.0, target_face.aabb_mm.min.x, 0.05));
        assert!(approx_eq(projected.1, target_face.aabb_mm.max.x, 0.05));

        // 7. Operation::rotary_axial_boundary / _mut accessors
        let mut op_2lin = Operation::new("op_r2", "Rot2", OperationKind::Rotary2Linear);
        assert!(op_2lin.rotary_axial_boundary().is_some());
        op_2lin
            .rotary_axial_boundary_mut()
            .unwrap()
            .axial_boundary_mode = RotaryAxialBoundaryMode::ExplicitRange;
        assert_eq!(
            op_2lin.rotary_axial_boundary().unwrap().axial_boundary_mode,
            RotaryAxialBoundaryMode::ExplicitRange
        );

        let mut op_4ax = Operation::new("op_r4", "Rot4", OperationKind::Rotary4AxisSim);
        assert!(op_4ax.rotary_axial_boundary().is_some());
        op_4ax
            .rotary_axial_boundary_mut()
            .unwrap()
            .axial_containment = ToolContainmentMode::ToolInsideBoundary;
        assert_eq!(
            op_4ax.rotary_axial_boundary().unwrap().axial_containment,
            ToolContainmentMode::ToolInsideBoundary
        );

        let mut op_2d = Operation::new("op_2d", "Pocket", OperationKind::Pocket2D);
        assert!(op_2d.rotary_axial_boundary().is_none());
        assert!(op_2d.rotary_axial_boundary_mut().is_none());
    }
}
