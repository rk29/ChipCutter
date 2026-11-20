//! Configurable 4th-axis rotary kinematics, cylindrical coordinate transforms, inverse-time (`G93`)
//! and angular (`G94`) feedrate helpers, and multi-turn / shortest-path unwinding (`03-setups-wcs.md`
//! & Milestone 19).

use chip_math::{Aabb3, DVec3, EvalContext, ParamLength};
use serde::{Deserialize, Serialize};

use crate::stock::{ResolvedStockGeometry, StockAxis};

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

/// Angular unwinding / rollover strategy when commanding rotary target angles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RotaryUnwindMode {
    /// Allow continuous multi-turn coordinates (e.g., `0° -> 720° -> 1440°` for helical spirals).
    #[default]
    ContinuousMultiTurn,
    /// Move via shortest signed angular delta `[-180°, +180°]` from current angle.
    WrapShortestPath,
    /// Fold all target angles into `[0°, 360°)`.
    Modulo0To360,
}

impl RotaryUnwindMode {
    pub const ALL: [Self; 3] = [
        Self::ContinuousMultiTurn,
        Self::WrapShortestPath,
        Self::Modulo0To360,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::ContinuousMultiTurn => "Continuous Multi-Turn",
            Self::WrapShortestPath => "Shortest Path (±180°)",
            Self::Modulo0To360 => "Modulo [0°, 360°)",
        }
    }
}

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

/// Resolves the commanded target rotary angle (`deg`) from `current_deg` to `target_deg`
/// according to `unwind_mode`.
pub fn resolve_rotary_target_deg(
    current_deg: f64,
    target_deg: f64,
    unwind_mode: RotaryUnwindMode,
) -> f64 {
    if !current_deg.is_finite() || !target_deg.is_finite() {
        return 0.0;
    }
    match unwind_mode {
        RotaryUnwindMode::ContinuousMultiTurn => target_deg,
        RotaryUnwindMode::WrapShortestPath => {
            let mut delta = (target_deg - current_deg).rem_euclid(360.0);
            if delta > 180.0 {
                delta -= 360.0;
            }
            current_deg + delta
        }
        RotaryUnwindMode::Modulo0To360 => {
            let m = target_deg.rem_euclid(360.0);
            if (m - 360.0).abs() < 1e-9 { 0.0 } else { m }
        }
    }
}

/// Computes the exact NIST RS274NGC / LinuxCNC / grblHAL **`G93` Inverse-Time Feedrate** (`1/min`)
/// for a combined linear + rotary motion segment so the cutter moves along the workpiece surface
/// at `target_surface_feed_mm_min`.
///
/// Returns `None` if motion duration is zero (degenerate zero-length segment).
pub fn compute_inverse_time_feed_g93(
    start_wcs: DVec3,
    end_wcs: DVec3,
    start_deg: f64,
    end_deg: f64,
    effective_radius_mm: f64,
    target_surface_feed_mm_min: f64,
    max_rotary_feed_deg_min: f64,
) -> Option<f64> {
    if !target_surface_feed_mm_min.is_finite() || target_surface_feed_mm_min <= 1e-6 {
        return None;
    }
    let linear_dist = start_wcs.distance(end_wcs);
    let d_deg = (end_deg - start_deg).abs();
    let d_rad = d_deg.to_radians();
    let r_eff = effective_radius_mm.max(0.0);
    let rotary_arc_dist = r_eff * d_rad;
    let combined_dist = linear_dist.hypot(rotary_arc_dist);

    let dt_surface_min = combined_dist / target_surface_feed_mm_min;
    let dt_rotary_min = if d_deg > 1e-9 && max_rotary_feed_deg_min > 1e-6 {
        d_deg / max_rotary_feed_deg_min
    } else {
        0.0
    };
    let dt_min = dt_surface_min.max(dt_rotary_min);
    if !dt_min.is_finite() || dt_min <= 1e-9 {
        return None;
    }
    Some((1.0 / dt_min).clamp(0.01, 99_999.0))
}

/// Computes the **`G94` pure angular feedrate** (`deg/min`) matching `surface_feed_mm_min` at
/// `effective_radius_mm`, clamped to `max_deg_min`.
pub fn compute_rotary_angular_feed_deg_min(
    surface_feed_mm_min: f64,
    effective_radius_mm: f64,
    max_deg_min: f64,
) -> f64 {
    let max_f = max_deg_min.max(1.0);
    if !surface_feed_mm_min.is_finite() || surface_feed_mm_min <= 1e-6 {
        return max_f;
    }
    let r = effective_radius_mm.max(0.25);
    let deg_min = (surface_feed_mm_min / r).to_degrees();
    deg_min.clamp(1.0, max_f)
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
    fn g93_inverse_time_feed_and_unwinding_modes_are_exact() {
        // Pure rotary 90° move at R = 20 mm -> arc length = 20 * pi/2 = 31.4159 mm
        // At 628.3185 mm/min -> dt = 0.05 min -> F_g93 = 20.0 (1/min)
        let f_g93 = compute_inverse_time_feed_g93(
            DVec3::new(0.0, 0.0, 20.0),
            DVec3::new(0.0, 0.0, 20.0),
            0.0,
            90.0,
            20.0,
            20.0 * std::f64::consts::PI * 10.0,
            36_000.0,
        )
        .unwrap();
        assert!(approx_eq(f_g93, 20.0, 1e-6), "expected 20.0, got {f_g93}");

        // Combined axial (30 mm) + rotary arc (40 mm) -> hypotenuse 50 mm @ 500 mm/min -> dt=0.1 -> F=10.0
        let arc_deg = (40.0_f64 / 20.0).to_degrees();
        let f_comb = compute_inverse_time_feed_g93(
            DVec3::new(0.0, 0.0, 20.0),
            DVec3::new(30.0, 0.0, 20.0),
            0.0,
            arc_deg,
            20.0,
            500.0,
            36_000.0,
        )
        .unwrap();
        assert!(approx_eq(f_comb, 10.0, 1e-6), "expected 10.0, got {f_comb}");

        // Unwinding policies
        assert!(approx_eq(
            resolve_rotary_target_deg(350.0, 10.0, RotaryUnwindMode::WrapShortestPath),
            370.0,
            1e-9
        ));
        assert!(approx_eq(
            resolve_rotary_target_deg(10.0, 350.0, RotaryUnwindMode::WrapShortestPath),
            -10.0,
            1e-9
        ));
        assert!(approx_eq(
            resolve_rotary_target_deg(720.0, 1080.0, RotaryUnwindMode::ContinuousMultiTurn),
            1080.0,
            1e-9
        ));
        assert!(approx_eq(
            resolve_rotary_target_deg(350.0, -90.0, RotaryUnwindMode::Modulo0To360),
            270.0,
            1e-9
        ));

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
    }
}
