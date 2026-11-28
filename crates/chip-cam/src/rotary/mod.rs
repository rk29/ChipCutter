//! Configurable 4th-axis rotary kinematics, cylindrical coordinate transforms, inverse-time (`G93`)
//! and angular (`G94`) feedrate helpers, and multi-turn / shortest-path unwinding (`03-setups-wcs.md`
//! & Milestone 19).

mod indexed;
mod surface_drop;
mod window;

use chip_math::{Aabb3, DVec3, EvalContext, ParamLength};
use serde::{Deserialize, Serialize};

use crate::stock::{ResolvedStockGeometry, StockAxis};

pub use indexed::{build_indexed_virtual_setup, compute_index_angle_for_normal};
pub use surface_drop::{
    build_rotated_strip_mesh, detect_rotary_axial_undercut_warning, drop_cutter_rotary_at,
    drop_cutter_rotary_z_rel, estimate_rotary_flank_cos_phi,
    mesh_has_steep_circumferential_overhang,
};
pub use window::{
    ResolvedRotaryWindow, RotaryWindowInput, project_selections_onto_rotary_axis,
    resolve_rotary_window, validate_toolpath_rotary_chuck_clearance,
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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::MeshTriangle;
    use chip_math::{DVec2, DisplayUnit, RigidFrame3, approx_eq};

    use super::*;
    use crate::{
        params::{RotaryAxialBoundaryConfig, RotaryAxialBoundaryMode, ToolContainmentMode},
        selection::{ExtractedFeature25D, ExtractedSelectionSummary, GeometrySelectionItem},
        surface3d::{SurfaceCutterProfile, SurfaceMesh3D},
        wcs::CamError,
    };

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
