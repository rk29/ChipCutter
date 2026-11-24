use chip_math::{EvalContext, ParamLength};
use serde::{Deserialize, Serialize};

use super::{
    common::{default_true, param_expr},
    milling_2d::AdaptiveClearingParams,
};

/// 2D XY machining containment boundary source for 3-axis surface operations (`Waterline3D`, `Adaptive3D`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MachiningBoundaryMode {
    #[default]
    StockBox,
    ModelBoundingBox,
    ModelSilhouette,
    SelectedRegions,
}

impl MachiningBoundaryMode {
    pub const ALL: [Self; 4] = [
        Self::StockBox,
        Self::ModelBoundingBox,
        Self::ModelSilhouette,
        Self::SelectedRegions,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::StockBox => "Stock Bounds",
            Self::ModelBoundingBox => "Model Bounds",
            Self::ModelSilhouette => "Model Silhouette",
            Self::SelectedRegions => "Selected 2D Contours",
        }
    }
}

/// How the cutter is constrained relative to the 2D machining boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolContainmentMode {
    #[default]
    ToolCenterOnBoundary,
    ToolInsideBoundary,
    ToolOutsideBoundary,
}

impl ToolContainmentMode {
    pub const ALL: [Self; 3] = [
        Self::ToolCenterOnBoundary,
        Self::ToolInsideBoundary,
        Self::ToolOutsideBoundary,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::ToolCenterOnBoundary => "Tool Center on Boundary",
            Self::ToolInsideBoundary => "Tool Inside Boundary",
            Self::ToolOutsideBoundary => "Tool Outside Boundary",
        }
    }
}

/// Contour ordering strategy across Z levels in [`OperationKind::Waterline3D`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WaterlineOrderMode {
    #[default]
    ByLevel,
    ByDepthRegion,
}

impl WaterlineOrderMode {
    pub const ALL: [Self; 2] = [Self::ByLevel, Self::ByDepthRegion];

    pub const fn label(self) -> &'static str {
        match self {
            Self::ByLevel => "Order by Level (Top-Down)",
            Self::ByDepthRegion => "Order by Depth / Island",
        }
    }
}

fn default_slope_max_deg() -> f64 {
    90.0
}

fn default_waterline_ramp_deg() -> f64 {
    3.0
}

fn default_flat_area_min_mm2() -> f64 {
    4.0
}

pub(super) fn default_zero_length() -> ParamLength {
    ParamLength::from_mm(0.0)
}

/// Shared 2D XY machining containment boundary configuration embedded via `#[serde(flatten)]`
/// across all 3-axis surfacing (`Waterline3D`, `Adaptive3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`),
/// `Undercut3D`, `Deburr3D`, and `Facing` parameter structs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MachiningBoundaryConfig {
    #[serde(default)]
    pub boundary_mode: MachiningBoundaryMode,
    #[serde(default)]
    pub tool_containment: ToolContainmentMode,
    #[serde(default = "default_zero_length")]
    pub boundary_offset: ParamLength,
    /// When `boundary_mode == ModelSilhouette`, fill interior through-holes so 3D passes machine
    /// through open voids instead of leaving uncut stock pillars inside them.
    #[serde(default = "default_true")]
    pub silhouette_fill_holes: bool,
    /// Signed `-X` side offset when `boundary_mode == ModelBoundingBox` (`+` expands outward toward `-X`, `-` insets inward toward `+X`).
    #[serde(default = "default_zero_length")]
    pub model_bounds_offset_x_neg: ParamLength,
    /// Signed `+X` side offset when `boundary_mode == ModelBoundingBox` (`+` expands outward toward `+X`, `-` insets inward toward `-X`).
    #[serde(default = "default_zero_length")]
    pub model_bounds_offset_x_pos: ParamLength,
    /// Signed `-Y` side offset when `boundary_mode == ModelBoundingBox` (`+` expands outward toward `-Y`, `-` insets inward toward `+Y`).
    #[serde(default = "default_zero_length")]
    pub model_bounds_offset_y_neg: ParamLength,
    /// Signed `+Y` side offset when `boundary_mode == ModelBoundingBox` (`+` expands outward toward `+Y`, `-` insets inward toward `-Y`).
    #[serde(default = "default_zero_length")]
    pub model_bounds_offset_y_pos: ParamLength,
}

impl Default for MachiningBoundaryConfig {
    fn default() -> Self {
        Self {
            boundary_mode: MachiningBoundaryMode::default(),
            tool_containment: ToolContainmentMode::default(),
            boundary_offset: default_zero_length(),
            silhouette_fill_holes: true,
            model_bounds_offset_x_neg: default_zero_length(),
            model_bounds_offset_x_pos: default_zero_length(),
            model_bounds_offset_y_neg: default_zero_length(),
            model_bounds_offset_y_pos: default_zero_length(),
        }
    }
}

impl MachiningBoundaryConfig {
    pub fn new(
        boundary_mode: MachiningBoundaryMode,
        tool_containment: ToolContainmentMode,
        boundary_offset_mm: f64,
    ) -> Self {
        Self {
            boundary_mode,
            tool_containment,
            boundary_offset: ParamLength::from_mm(boundary_offset_mm),
            silhouette_fill_holes: true,
            model_bounds_offset_x_neg: default_zero_length(),
            model_bounds_offset_x_pos: default_zero_length(),
            model_bounds_offset_y_neg: default_zero_length(),
            model_bounds_offset_y_pos: default_zero_length(),
        }
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.boundary_offset.reevaluate(ctx);
        let _ = self.model_bounds_offset_x_neg.reevaluate(ctx);
        let _ = self.model_bounds_offset_x_pos.reevaluate(ctx);
        let _ = self.model_bounds_offset_y_neg.reevaluate(ctx);
        let _ = self.model_bounds_offset_y_pos.reevaluate(ctx);
    }
}

/// Strategy parameters for **3D Waterline / Z-Level Machining** (`OperationKind::Waterline3D`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Waterline3DParams {
    #[serde(flatten)]
    pub boundary: MachiningBoundaryConfig,
    /// Minimum surface steepness (`0° = flat horizontal`, `90° = vertical wall`) machined by waterline passes.
    #[serde(default)]
    pub min_slope_deg: f64,
    /// Maximum surface steepness (`90°` default).
    #[serde(default = "default_slope_max_deg")]
    pub max_slope_deg: f64,
    /// Subdivide coarse Z levels on shallow slopes to keep surface cusp height below `max_cusp_height`.
    #[serde(default = "default_true")]
    pub adaptive_stepdown: bool,
    /// Minimum adaptive Z stepdown clamp (`mm`).
    #[serde(default = "default_waterline_min_stepdown")]
    pub min_stepdown: ParamLength,
    /// Target maximum scallop / cusp height (`mm`) driving adaptive intermediate Z slices.
    #[serde(default = "default_waterline_max_cusp")]
    pub max_cusp_height: ParamLength,
    /// Automatically inject exact Z-levels at detected horizontal flat plateaus (`+ stock_axial_mm`).
    #[serde(default = "default_true")]
    pub detect_flats: bool,
    #[serde(default)]
    pub order_mode: WaterlineOrderMode,
    #[serde(default = "default_true")]
    pub climb_milling: bool,
    #[serde(default = "default_waterline_ramp_deg")]
    pub ramp_angle_deg: f64,
}

fn default_waterline_min_stepdown() -> ParamLength {
    ParamLength::from_mm(0.35)
}

fn default_waterline_max_cusp() -> ParamLength {
    ParamLength::from_mm(0.05)
}

impl Default for Waterline3DParams {
    fn default() -> Self {
        Self {
            boundary: MachiningBoundaryConfig::new(
                MachiningBoundaryMode::ModelSilhouette,
                ToolContainmentMode::ToolOutsideBoundary,
                0.5,
            ),
            min_slope_deg: 0.0,
            max_slope_deg: 90.0,
            adaptive_stepdown: true,
            min_stepdown: default_waterline_min_stepdown(),
            max_cusp_height: default_waterline_max_cusp(),
            detect_flats: true,
            order_mode: WaterlineOrderMode::ByLevel,
            climb_milling: true,
            ramp_angle_deg: 3.0,
        }
    }
}

impl Waterline3DParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        self.boundary.reevaluate(ctx);
        let _ = self.min_stepdown.reevaluate(ctx);
        let _ = self.max_cusp_height.reevaluate(ctx);
    }
}

/// Strategy parameters for **3D Adaptive Clearing / Roughing** (`OperationKind::Adaptive3D`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Adaptive3DParams {
    #[serde(flatten)]
    pub boundary: MachiningBoundaryConfig,
    #[serde(default)]
    pub adaptive: AdaptiveClearingParams,
    /// Primary coarse roughing Z stepdown (`mm`).
    #[serde(default = "default_adaptive3d_rough_stepdown")]
    pub max_roughing_stepdown: ParamLength,
    /// Intermediate fine step-up height (`mm`) climbing sloped terraces between coarse levels.
    #[serde(default = "default_adaptive3d_fine_stepdown")]
    pub fine_stepdown: ParamLength,
    /// Automatically inject exact Z-levels at detected horizontal flat plateaus (`+ stock_axial_mm`).
    #[serde(default = "default_true")]
    pub detect_flats: bool,
    /// Minimum horizontal plateau area (`mm²`) considered for flat-level detection.
    #[serde(default = "default_flat_area_min_mm2")]
    pub flat_area_min_mm2: f64,
}

fn default_adaptive3d_rough_stepdown() -> ParamLength {
    ParamLength::from_mm(5.0)
}

fn default_adaptive3d_fine_stepdown() -> ParamLength {
    ParamLength::from_mm(1.5)
}

impl Default for Adaptive3DParams {
    fn default() -> Self {
        Self {
            boundary: MachiningBoundaryConfig::new(
                MachiningBoundaryMode::StockBox,
                ToolContainmentMode::ToolOutsideBoundary,
                1.0,
            ),
            adaptive: AdaptiveClearingParams::default(),
            max_roughing_stepdown: default_adaptive3d_rough_stepdown(),
            fine_stepdown: default_adaptive3d_fine_stepdown(),
            detect_flats: true,
            flat_area_min_mm2: 4.0,
        }
    }
}

impl Adaptive3DParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        self.boundary.reevaluate(ctx);
        self.adaptive.reevaluate(ctx);
        let _ = self.max_roughing_stepdown.reevaluate(ctx);
        let _ = self.fine_stepdown.reevaluate(ctx);
    }
}

/// Cutting traversal direction across parallel raster lines in [`OperationKind::Parallel3D`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParallelCutDirection {
    #[default]
    ZigZag,
    OneWayClimb,
    OneWayConventional,
}

impl ParallelCutDirection {
    pub const ALL: [Self; 3] = [Self::ZigZag, Self::OneWayClimb, Self::OneWayConventional];

    pub const fn label(self) -> &'static str {
        match self {
            Self::ZigZag => "Both Ways (Zig-Zag)",
            Self::OneWayClimb => "One-Way (Climb)",
            Self::OneWayConventional => "One-Way (Conventional)",
        }
    }
}

fn default_parallel3d_stepover() -> ParamLength {
    ParamLength::from_mm(0.8)
}

pub(super) fn default_finishing_cusp_height() -> ParamLength {
    ParamLength::from_mm(0.025)
}

fn default_parallel3d_sample_step() -> ParamLength {
    ParamLength::from_mm(0.35)
}

pub(super) fn default_finishing_smooth_tol() -> ParamLength {
    ParamLength::from_mm(0.005)
}

/// Strategy parameters for **3D Parallel / Raster Finishing** (`OperationKind::Parallel3D`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Parallel3DParams {
    #[serde(flatten)]
    pub boundary: MachiningBoundaryConfig,
    /// Explicit XY raster stepover (`mm`) when `use_cusp_height == false`.
    #[serde(default = "default_parallel3d_stepover")]
    pub stepover: ParamLength,
    /// Automatically compute stepover from `cusp_height` and cutter ball/corner radius.
    #[serde(default = "default_true")]
    pub use_cusp_height: bool,
    /// Target surface scallop / cusp height (`mm`) when `use_cusp_height == true`.
    #[serde(default = "default_finishing_cusp_height")]
    pub cusp_height: ParamLength,
    /// Raster scanline orientation angle in degrees (`0°` = parallel to +X, `90°` = parallel to +Y).
    #[serde(default)]
    pub pass_angle_deg: f64,
    /// Emit an additional perpendicular cross-finishing pass at `pass_angle_deg + 90°`.
    #[serde(default)]
    pub cross_pass: bool,
    #[serde(default)]
    pub cut_direction: ParallelCutDirection,
    /// Minimum surface slope angle (`0°` = horizontal flat, `90°` = vertical wall).
    #[serde(default)]
    pub min_slope_deg: f64,
    /// Maximum surface slope angle (`90°` default; e.g. `45°` for shallow finishing paired with Waterline).
    #[serde(default = "default_slope_max_deg")]
    pub max_slope_deg: f64,
    /// Along-pass Drop-Cutter sampling pitch (`mm`).
    #[serde(default = "default_parallel3d_sample_step")]
    pub sample_step: ParamLength,
    /// 3D Ramer–Douglas–Peucker chord decimation tolerance (`mm`).
    #[serde(default = "default_finishing_smooth_tol")]
    pub smoothing_tolerance: ParamLength,
}

impl Default for Parallel3DParams {
    fn default() -> Self {
        Self {
            boundary: MachiningBoundaryConfig::new(
                MachiningBoundaryMode::ModelSilhouette,
                ToolContainmentMode::ToolCenterOnBoundary,
                0.0,
            ),
            stepover: default_parallel3d_stepover(),
            use_cusp_height: true,
            cusp_height: default_finishing_cusp_height(),
            pass_angle_deg: 0.0,
            cross_pass: false,
            cut_direction: ParallelCutDirection::ZigZag,
            min_slope_deg: 0.0,
            max_slope_deg: 90.0,
            sample_step: default_parallel3d_sample_step(),
            smoothing_tolerance: default_finishing_smooth_tol(),
        }
    }
}

impl Parallel3DParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        self.boundary.reevaluate(ctx);
        let _ = self.stepover.reevaluate(ctx);
        let _ = self.cusp_height.reevaluate(ctx);
        let _ = self.sample_step.reevaluate(ctx);
        let _ = self.smoothing_tolerance.reevaluate(ctx);
    }
}

fn default_scallop3d_max_stepover() -> ParamLength {
    ParamLength::from_mm(1.5)
}

fn default_scallop3d_min_stepover() -> ParamLength {
    ParamLength::from_mm(0.15)
}

/// Strategy parameters for **3D Constant-Scallop Finishing** (`OperationKind::Scallop3D`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Scallop3DParams {
    #[serde(flatten)]
    pub boundary: MachiningBoundaryConfig,
    /// Target constant 3D scallop / cusp height (`mm`) across both shallow crests and steep walls.
    #[serde(default = "default_finishing_cusp_height")]
    pub cusp_height: ParamLength,
    /// Upper clamp on 3D surface arc-length stepover (`mm`).
    #[serde(default = "default_scallop3d_max_stepover")]
    pub max_stepover: ParamLength,
    /// Lower clamp on 3D surface arc-length stepover (`mm`).
    #[serde(default = "default_scallop3d_min_stepover")]
    pub min_stepover: ParamLength,
    #[serde(default = "default_true")]
    pub climb_milling: bool,
    #[serde(default)]
    pub min_slope_deg: f64,
    #[serde(default = "default_slope_max_deg")]
    pub max_slope_deg: f64,
    /// Traverse iso-contours from interior peaks outward (`true`) or boundary inward (`false`).
    #[serde(default)]
    pub inside_out: bool,
    /// 3D Ramer–Douglas–Peucker chord decimation tolerance (`mm`).
    #[serde(default = "default_finishing_smooth_tol")]
    pub smoothing_tolerance: ParamLength,
}

impl Default for Scallop3DParams {
    fn default() -> Self {
        Self {
            boundary: MachiningBoundaryConfig::new(
                MachiningBoundaryMode::ModelSilhouette,
                ToolContainmentMode::ToolCenterOnBoundary,
                0.0,
            ),
            cusp_height: default_finishing_cusp_height(),
            max_stepover: default_scallop3d_max_stepover(),
            min_stepover: default_scallop3d_min_stepover(),
            climb_milling: true,
            min_slope_deg: 0.0,
            max_slope_deg: 90.0,
            inside_out: false,
            smoothing_tolerance: default_finishing_smooth_tol(),
        }
    }
}

impl Scallop3DParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        self.boundary.reevaluate(ctx);
        let _ = self.cusp_height.reevaluate(ctx);
        let _ = self.max_stepover.reevaluate(ctx);
        let _ = self.min_stepover.reevaluate(ctx);
        let _ = self.smoothing_tolerance.reevaluate(ctx);
    }
}

fn default_pencil3d_ref_tool_dia() -> ParamLength {
    param_expr("tool_d * 2.0", 6.0)
}

fn default_pencil3d_valley_threshold() -> ParamLength {
    ParamLength::from_mm(0.08)
}

fn default_pencil3d_offset_stepover() -> ParamLength {
    ParamLength::from_mm(0.35)
}

/// Strategy parameters for **3D Pencil / Concave Fillet Cleanup** (`OperationKind::Pencil3D`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pencil3DParams {
    #[serde(flatten)]
    pub boundary: MachiningBoundaryConfig,
    /// Virtual larger reference ball diameter (`mm`) used to detect unmachined concave valleys & creases.
    #[serde(default = "default_pencil3d_ref_tool_dia")]
    pub reference_tool_diameter: ParamLength,
    /// Minimum residual stock height `z_drop(ref_ball) - z_drop(current_tool)` (`mm`) to trace a valley.
    #[serde(default = "default_pencil3d_valley_threshold")]
    pub valley_threshold: ParamLength,
    /// Number of extra lateral cleanup offset passes on each side of detected valley centerlines (`0..=4`).
    #[serde(default)]
    pub num_offset_passes: u32,
    /// Lateral stepover (`mm`) between extra multi-pass pencil offsets.
    #[serde(default = "default_pencil3d_offset_stepover")]
    pub offset_stepover: ParamLength,
    /// 3D Ramer–Douglas–Peucker chord decimation tolerance (`mm`).
    #[serde(default = "default_finishing_smooth_tol")]
    pub smoothing_tolerance: ParamLength,
}

impl Default for Pencil3DParams {
    fn default() -> Self {
        Self {
            boundary: MachiningBoundaryConfig::new(
                MachiningBoundaryMode::ModelSilhouette,
                ToolContainmentMode::ToolCenterOnBoundary,
                0.0,
            ),
            reference_tool_diameter: default_pencil3d_ref_tool_dia(),
            valley_threshold: default_pencil3d_valley_threshold(),
            num_offset_passes: 0,
            offset_stepover: default_pencil3d_offset_stepover(),
            smoothing_tolerance: default_finishing_smooth_tol(),
        }
    }
}

impl Pencil3DParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        self.boundary.reevaluate(ctx);
        let _ = self.reference_tool_diameter.reevaluate(ctx);
        let _ = self.valley_threshold.reevaluate(ctx);
        let _ = self.offset_stepover.reevaluate(ctx);
        let _ = self.smoothing_tolerance.reevaluate(ctx);
    }
}
