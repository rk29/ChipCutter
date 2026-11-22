use chip_math::{EvalContext, ParamLength};
use serde::{Deserialize, Serialize};

use super::surface_3d::{
    default_finishing_cusp_height, default_finishing_smooth_tol, default_zero_length,
};

/// Machining pattern for centerline-locked 2-linear + rotary 3D surfacing (`OperationKind::Rotary2Linear`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RotaryPatternMode {
    /// Continuous multi-turn helical spiral along the rotary axis (zero stepover retracts).
    #[default]
    ContinuousSpiral,
    /// Axial raster passes sweeping back and forth along `u_axial`, stepping around `θ`.
    AxialParallel,
}

impl RotaryPatternMode {
    pub const ALL: [Self; 2] = [Self::ContinuousSpiral, Self::AxialParallel];

    pub const fn label(self) -> &'static str {
        match self {
            Self::ContinuousSpiral => "Continuous Helical Spiral",
            Self::AxialParallel => "Axial Parallel (Index & Sweep)",
        }
    }
}

/// Traversal direction for [`OperationKind::Rotary2Linear`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RotaryCutDirection {
    #[default]
    ZigZag,
    Climb,
    Conventional,
}

impl RotaryCutDirection {
    pub const ALL: [Self; 3] = [Self::ZigZag, Self::Climb, Self::Conventional];

    pub const fn label(self) -> &'static str {
        match self {
            Self::ZigZag => "Both Ways (Zig-Zag)",
            Self::Climb => "Climb (One-Way)",
            Self::Conventional => "Conventional (One-Way)",
        }
    }
}

fn default_rotary_stepover() -> ParamLength {
    ParamLength::from_mm(1.2)
}

fn default_rotary_end_angle_deg() -> f64 {
    360.0
}

fn default_rotary_angular_sample_deg() -> f64 {
    2.5
}

fn default_rotary_axial_sample() -> ParamLength {
    ParamLength::from_mm(0.5)
}

/// Strategy parameters for **4th-Axis 2-Linear + Rotary 3D Surfacing** (`OperationKind::Rotary2Linear`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Rotary2LinearParams {
    #[serde(default)]
    pub pattern: RotaryPatternMode,
    #[serde(default)]
    pub cut_direction: RotaryCutDirection,
    /// Axial pitch per revolution (`ContinuousSpiral`) or circumferential surface stepover (`AxialParallel`).
    #[serde(default = "default_rotary_stepover")]
    pub stepover: ParamLength,
    /// Derive stepover automatically from `cusp_height` and cutter ball/corner radius.
    #[serde(default)]
    pub use_cusp_height: bool,
    #[serde(default = "default_finishing_cusp_height")]
    pub cusp_height: ParamLength,
    /// Start angular boundary (`deg`, used by `AxialParallel` or partial-sector spiral).
    #[serde(default)]
    pub start_angle_deg: f64,
    /// End angular boundary (`deg`, `360.0` default for full circumference).
    #[serde(default = "default_rotary_end_angle_deg")]
    pub end_angle_deg: f64,
    /// Inward trim margin (`mm`) from the `-u` stock end (e.g. to stay clear of the rotary chuck!).
    #[serde(default = "default_zero_length")]
    pub axial_margin_start: ParamLength,
    /// Inward trim margin (`mm`) from the `+u` stock end (e.g. to clear a live tailstock center).
    #[serde(default = "default_zero_length")]
    pub axial_margin_end: ParamLength,
    /// Angular drop-cutter sampling resolution (`deg`).
    #[serde(default = "default_rotary_angular_sample_deg")]
    pub angular_sample_deg: f64,
    /// Axial drop-cutter sampling pitch (`mm`).
    #[serde(default = "default_rotary_axial_sample")]
    pub axial_sample_step: ParamLength,
    /// Chord decimation tolerance (`mm`) keeping 4-axis G-code smooth and compact.
    #[serde(default = "default_finishing_smooth_tol")]
    pub smoothing_tolerance: ParamLength,
}

impl Default for Rotary2LinearParams {
    fn default() -> Self {
        Self {
            pattern: RotaryPatternMode::ContinuousSpiral,
            cut_direction: RotaryCutDirection::ZigZag,
            stepover: default_rotary_stepover(),
            use_cusp_height: false,
            cusp_height: default_finishing_cusp_height(),
            start_angle_deg: 0.0,
            end_angle_deg: 360.0,
            axial_margin_start: ParamLength::from_mm(0.0),
            axial_margin_end: ParamLength::from_mm(0.0),
            angular_sample_deg: default_rotary_angular_sample_deg(),
            axial_sample_step: default_rotary_axial_sample(),
            smoothing_tolerance: default_finishing_smooth_tol(),
        }
    }
}

impl Rotary2LinearParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.stepover.reevaluate(ctx);
        let _ = self.cusp_height.reevaluate(ctx);
        let _ = self.axial_margin_start.reevaluate(ctx);
        let _ = self.axial_margin_end.reevaluate(ctx);
        let _ = self.axial_sample_step.reevaluate(ctx);
        let _ = self.smoothing_tolerance.reevaluate(ctx);
    }
}

/// Toolpath traversal pattern for **Simultaneous 4-Axis Milling** (`OperationKind::Rotary4AxisSim`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Rotary4AxisPattern {
    /// Continuous 4-axis simultaneous helical spiral (`X + Y + Z + A` or `X + Y + Z + B`).
    #[default]
    SimultaneousSpiral,
    /// Axial/helical flowline strips stepped circumferentially around the workpiece.
    HelicalFlowline,
}

impl Rotary4AxisPattern {
    pub const ALL: [Self; 2] = [Self::SimultaneousSpiral, Self::HelicalFlowline];

    pub const fn label(self) -> &'static str {
        match self {
            Self::SimultaneousSpiral => "Simultaneous Helical Spiral",
            Self::HelicalFlowline => "Axial / Helical Flowline",
        }
    }
}

/// Off-centerline lateral tilt control mode for **Simultaneous 4-Axis Milling**.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RotaryTiltMode {
    /// Automatically computes smoothed off-centerline lateral cross-axis shift from local
    /// circumferential surface normals (engaging the side/ball flank on steep helical walls).
    #[default]
    SurfaceNormalAutoTilt,
    /// Applies a fixed cross-axis lateral offset (`lateral_offset`) off the rotary centerline.
    FixedLateralOffset,
}

impl RotaryTiltMode {
    pub const ALL: [Self; 2] = [Self::SurfaceNormalAutoTilt, Self::FixedLateralOffset];

    pub const fn label(self) -> &'static str {
        match self {
            Self::SurfaceNormalAutoTilt => "Auto Surface-Normal Tilt",
            Self::FixedLateralOffset => "Fixed Lateral Offset",
        }
    }
}

fn default_rotary4_lateral_offset() -> ParamLength {
    ParamLength::from_mm(2.5)
}

fn default_rotary4_max_tilt_deg() -> f64 {
    18.0
}

fn default_rotary4_tilt_smooth() -> u32 {
    3
}

/// Strategy parameters for **Simultaneous 4-Axis Milling (`X + Y + Z + A/B`)** (`OperationKind::Rotary4AxisSim`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Rotary4AxisSimParams {
    #[serde(default)]
    pub pattern: Rotary4AxisPattern,
    #[serde(default)]
    pub cut_direction: RotaryCutDirection,
    #[serde(default)]
    pub tilt_mode: RotaryTiltMode,
    /// Fixed cross-axis lateral offset (`mm`) used in [`RotaryTiltMode::FixedLateralOffset`].
    #[serde(default = "default_rotary4_lateral_offset")]
    pub lateral_offset: ParamLength,
    /// Maximum effective off-centerline tilt angle (`deg`, `0..40°`) in [`RotaryTiltMode::SurfaceNormalAutoTilt`].
    #[serde(default = "default_rotary4_max_tilt_deg")]
    pub max_tilt_deg: f64,
    /// Multi-pass grid smoothing radius keeping cross-axis `Y`/`X` and rotary motion smooth.
    #[serde(default = "default_rotary4_tilt_smooth")]
    pub tilt_smoothing_window: u32,
    #[serde(default = "default_rotary_stepover")]
    pub stepover: ParamLength,
    #[serde(default)]
    pub use_cusp_height: bool,
    #[serde(default = "default_finishing_cusp_height")]
    pub cusp_height: ParamLength,
    #[serde(default)]
    pub start_angle_deg: f64,
    #[serde(default = "default_rotary_end_angle_deg")]
    pub end_angle_deg: f64,
    #[serde(default = "default_zero_length")]
    pub axial_margin_start: ParamLength,
    #[serde(default = "default_zero_length")]
    pub axial_margin_end: ParamLength,
    #[serde(default = "default_rotary_angular_sample_deg")]
    pub angular_sample_deg: f64,
    #[serde(default = "default_rotary_axial_sample")]
    pub axial_sample_step: ParamLength,
    #[serde(default = "default_finishing_smooth_tol")]
    pub smoothing_tolerance: ParamLength,
}

impl Default for Rotary4AxisSimParams {
    fn default() -> Self {
        Self {
            pattern: Rotary4AxisPattern::SimultaneousSpiral,
            cut_direction: RotaryCutDirection::ZigZag,
            tilt_mode: RotaryTiltMode::SurfaceNormalAutoTilt,
            lateral_offset: default_rotary4_lateral_offset(),
            max_tilt_deg: default_rotary4_max_tilt_deg(),
            tilt_smoothing_window: default_rotary4_tilt_smooth(),
            stepover: default_rotary_stepover(),
            use_cusp_height: false,
            cusp_height: default_finishing_cusp_height(),
            start_angle_deg: 0.0,
            end_angle_deg: 360.0,
            axial_margin_start: ParamLength::from_mm(0.0),
            axial_margin_end: ParamLength::from_mm(0.0),
            angular_sample_deg: default_rotary_angular_sample_deg(),
            axial_sample_step: default_rotary_axial_sample(),
            smoothing_tolerance: default_finishing_smooth_tol(),
        }
    }
}

impl Rotary4AxisSimParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.lateral_offset.reevaluate(ctx);
        let _ = self.stepover.reevaluate(ctx);
        let _ = self.cusp_height.reevaluate(ctx);
        let _ = self.axial_margin_start.reevaluate(ctx);
        let _ = self.axial_margin_end.reevaluate(ctx);
        let _ = self.axial_sample_step.reevaluate(ctx);
        let _ = self.smoothing_tolerance.reevaluate(ctx);
    }
}
