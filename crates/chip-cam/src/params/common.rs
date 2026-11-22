use chip_math::{DEFAULT_ARC_FIT_TOL_MM, DVec2, EvalContext, ParamLength};
use serde::{Deserialize, Serialize};

pub(super) fn default_one_u32() -> u32 {
    1
}

pub(super) fn param_expr(expr: &str, default_mm: f64) -> ParamLength {
    ParamLength {
        expr: Some(expr.to_string()),
        value_mm: default_mm,
    }
}

pub(super) fn default_true() -> bool {
    true
}

pub(super) fn default_tolerance_mm() -> f64 {
    DEFAULT_ARC_FIT_TOL_MM
}

pub(super) fn default_ramp_angle_deg() -> f64 {
    2.5
}

pub(super) fn default_lead_sweep_deg() -> f64 {
    90.0
}

/// 2.5D Rest Machining mode (`05-operations.md` §Rest Machining).
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum RestMachiningMode {
    #[default]
    Disabled,
    FromPreviousTool {
        #[serde(default = "default_prev_tool_dia")]
        prev_tool_diameter: ParamLength,
        #[serde(default = "default_zero_len")]
        prev_stock_to_leave: ParamLength,
    },
    FromPrecedingOperations {
        /// Optional explicit upstream `Operation::id` list within the same Setup;
        /// when empty, uses all enabled preceding operations in the Setup (`0..op_idx`).
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        source_op_ids: Vec<String>,
    },
}

fn default_prev_tool_dia() -> ParamLength {
    param_expr("tool_d * 2.0", 12.0)
}

fn default_rest_overlap() -> ParamLength {
    ParamLength::from_mm(0.25)
}

fn default_min_rest_thickness() -> ParamLength {
    ParamLength::from_mm(0.05)
}

/// Exact 2.5D Rest Machining configuration shared by Pocket/Adaptive and Contour (`05-operations.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RestMachiningConfig {
    #[serde(default)]
    pub mode: RestMachiningMode,
    #[serde(default = "default_rest_overlap")]
    pub rest_overlap: ParamLength,
    #[serde(default = "default_min_rest_thickness")]
    pub min_rest_thickness: ParamLength,
}

impl Default for RestMachiningConfig {
    fn default() -> Self {
        Self {
            mode: RestMachiningMode::Disabled,
            rest_overlap: default_rest_overlap(),
            min_rest_thickness: default_min_rest_thickness(),
        }
    }
}

impl RestMachiningConfig {
    pub fn is_disabled(&self) -> bool {
        matches!(self.mode, RestMachiningMode::Disabled)
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        if let RestMachiningMode::FromPreviousTool {
            prev_tool_diameter,
            prev_stock_to_leave,
        } = &mut self.mode
        {
            let _ = prev_tool_diameter.reevaluate(ctx);
            let _ = prev_stock_to_leave.reevaluate(ctx);
        }
        let _ = self.rest_overlap.reevaluate(ctx);
        let _ = self.min_rest_thickness.reevaluate(ctx);
    }
}

pub const DEFAULT_MIN_FEED_FRACTION: f64 = 0.35;
pub const DEFAULT_CORNER_ANGLE_THRESHOLD_DEG: f64 = 25.0;
pub const DEFAULT_MAX_THINNING_MULTIPLIER: f64 = 1.5;

fn default_min_feed_fraction() -> f64 {
    DEFAULT_MIN_FEED_FRACTION
}

fn default_corner_angle_threshold_deg() -> f64 {
    DEFAULT_CORNER_ANGLE_THRESHOLD_DEG
}

fn default_slowdown_distance() -> ParamLength {
    param_expr("tool_d * 0.5", 3.0)
}

fn default_max_thinning_multiplier() -> f64 {
    DEFAULT_MAX_THINNING_MULTIPLIER
}

/// Engagement-aware feedrate optimization settings (`M58`):
/// internal arc perimeter-to-centerline scaling, sharp-corner lookahead deceleration,
/// and optional constant-chipload radial chip-thinning boost.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FeedOptimizationConfig {
    /// Opt-in master toggle (`false` by default for 100% backward compatibility).
    #[serde(default)]
    pub enabled: bool,
    /// Decelerate approaching sharp linear corners and recover smoothly after exit.
    #[serde(default = "default_true")]
    pub corner_slowdown: bool,
    /// Minimum allowed feedrate fraction (`0.15..=1.0`) preventing cutter rubbing/melting.
    #[serde(default = "default_min_feed_fraction")]
    pub min_feed_fraction: f64,
    /// Deflection angle threshold (`5.0°..=120.0°`) triggering sharp-corner lookahead slowdown.
    #[serde(default = "default_corner_angle_threshold_deg")]
    pub corner_angle_threshold_deg: f64,
    /// Lookahead deceleration & recovery ramp distance around sharp corners (default `tool_d * 0.5`).
    #[serde(default = "default_slowdown_distance", alias = "slowdown_distance_mm")]
    pub slowdown_distance: ParamLength,
    /// Exact perimeter-to-centerline scaling `(R_arc - R_tool) / R_arc` on internal cutting arcs.
    #[serde(default = "default_true")]
    pub internal_arc_scaling: bool,
    /// Gently boost straight/low-curvature light-engagement passes to maintain constant chipload.
    #[serde(default)]
    pub chip_thinning_boost: bool,
    /// Maximum feedrate multiplier (`1.0..=2.5`) when `chip_thinning_boost` is enabled.
    #[serde(default = "default_max_thinning_multiplier")]
    pub max_thinning_multiplier: f64,
}

impl Default for FeedOptimizationConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            corner_slowdown: true,
            min_feed_fraction: DEFAULT_MIN_FEED_FRACTION,
            corner_angle_threshold_deg: DEFAULT_CORNER_ANGLE_THRESHOLD_DEG,
            slowdown_distance: default_slowdown_distance(),
            internal_arc_scaling: true,
            chip_thinning_boost: false,
            max_thinning_multiplier: DEFAULT_MAX_THINNING_MULTIPLIER,
        }
    }
}

impl FeedOptimizationConfig {
    /// Returns `true` when matching default disabled settings so untouched `.chip.toml`
    /// operations omit `feed_optimization` completely (zero diff on existing projects).
    pub fn is_default(&self) -> bool {
        !self.enabled
            && self.corner_slowdown
            && (self.min_feed_fraction - DEFAULT_MIN_FEED_FRACTION).abs() <= 1e-9
            && (self.corner_angle_threshold_deg - DEFAULT_CORNER_ANGLE_THRESHOLD_DEG).abs() <= 1e-9
            && self.slowdown_distance.expr.as_deref() == Some("tool_d * 0.5")
            && self.internal_arc_scaling
            && !self.chip_thinning_boost
            && (self.max_thinning_multiplier - DEFAULT_MAX_THINNING_MULTIPLIER).abs() <= 1e-9
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.slowdown_distance.reevaluate(ctx);
    }
}

/// Common multi-depth stepdown, stock-to-leave, Rest Machining, feed optimization, and tolerance parameters (`05-operations.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PassesParams {
    #[serde(default = "default_true")]
    pub multiple_depths: bool,
    #[serde(default = "default_max_stepdown")]
    pub max_stepdown: ParamLength,
    #[serde(default = "default_true")]
    pub even_stepdowns: bool,
    #[serde(default = "default_zero_len")]
    pub stock_to_leave_radial: ParamLength,
    #[serde(default = "default_zero_len")]
    pub stock_to_leave_axial: ParamLength,
    #[serde(default, skip_serializing_if = "RestMachiningConfig::is_disabled")]
    pub rest_machining: RestMachiningConfig,
    #[serde(default, skip_serializing_if = "FeedOptimizationConfig::is_default")]
    pub feed_optimization: FeedOptimizationConfig,
    #[serde(default = "default_tolerance_mm")]
    pub tolerance_mm: f64,
}

fn default_max_stepdown() -> ParamLength {
    param_expr("tool_d * 0.5", 3.0)
}

pub(super) fn default_zero_len() -> ParamLength {
    ParamLength::from_mm(0.0)
}

impl Default for PassesParams {
    fn default() -> Self {
        Self {
            multiple_depths: true,
            max_stepdown: default_max_stepdown(),
            even_stepdowns: true,
            stock_to_leave_radial: default_zero_len(),
            stock_to_leave_axial: default_zero_len(),
            rest_machining: RestMachiningConfig::default(),
            feed_optimization: FeedOptimizationConfig::default(),
            tolerance_mm: default_tolerance_mm(),
        }
    }
}

impl PassesParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.max_stepdown.reevaluate(ctx);
        let _ = self.stock_to_leave_radial.reevaluate(ctx);
        let _ = self.stock_to_leave_axial.reevaluate(ctx);
        self.rest_machining.reevaluate(ctx);
        self.feed_optimization.reevaluate(ctx);
    }
}

/// Safe Z retract policy between disconnected regions (`05-operations.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RetractPolicy {
    #[default]
    MinimumRetract,
    FullClearance,
}

impl RetractPolicy {
    pub const ALL: [Self; 2] = [Self::MinimumRetract, Self::FullClearance];

    pub const fn label(self) -> &'static str {
        match self {
            Self::MinimumRetract => "Minimum Retract Plane",
            Self::FullClearance => "Full Clearance Plane",
        }
    }
}

/// Closed-cavity / profile Z entry strategy (`05-operations.md`).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum EntryStrategy {
    #[default]
    HelicalRamp,
    LinearRamp,
    Plunge,
    PreDrilled {
        point_wcs: DVec2,
    },
}

impl EntryStrategy {
    pub const fn label(&self) -> &'static str {
        match self {
            Self::HelicalRamp => "Helical Ramp",
            Self::LinearRamp => "Linear Profile Ramp",
            Self::Plunge => "Direct Plunge",
            Self::PreDrilled { .. } => "Pre-Drilled Point",
        }
    }
}

/// Horizontal + vertical tangent lead-in / lead-out specification (`05-operations.md` §2).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LeadConfig {
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default = "default_lead_h_radius")]
    pub horizontal_radius: ParamLength,
    #[serde(default = "default_lead_sweep_deg")]
    pub horizontal_sweep_deg: f64,
    #[serde(default = "default_lead_linear")]
    pub linear_length: ParamLength,
    #[serde(default = "default_zero_len")]
    pub vertical_radius: ParamLength,
}

fn default_lead_h_radius() -> ParamLength {
    param_expr("tool_d * 0.25", 1.5)
}

fn default_lead_linear() -> ParamLength {
    param_expr("tool_d * 0.15", 0.9)
}

impl Default for LeadConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            horizontal_radius: default_lead_h_radius(),
            horizontal_sweep_deg: default_lead_sweep_deg(),
            linear_length: default_lead_linear(),
            vertical_radius: default_zero_len(),
        }
    }
}

impl LeadConfig {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.horizontal_radius.reevaluate(ctx);
        let _ = self.linear_length.reevaluate(ctx);
        let _ = self.vertical_radius.reevaluate(ctx);
    }
}

/// Common linking, retract, ramp, and lead-in/lead-out parameters (`05-operations.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LinkingParams {
    #[serde(default)]
    pub retract_policy: RetractPolicy,
    #[serde(default = "default_stay_down")]
    pub safe_stay_down_distance: ParamLength,
    #[serde(default)]
    pub entry_mode: EntryStrategy,
    #[serde(default = "default_ramp_angle_deg")]
    pub ramp_angle_deg: f64,
    #[serde(default = "default_min_ramp_dia")]
    pub min_ramp_diameter: ParamLength,
    #[serde(default)]
    pub lead_in: LeadConfig,
    #[serde(default = "default_true")]
    pub lead_out_same_as_in: bool,
    #[serde(default)]
    pub lead_out: LeadConfig,
    #[serde(default = "default_seam_overlap")]
    pub seam_overlap: ParamLength,
}

fn default_stay_down() -> ParamLength {
    param_expr("tool_d * 2.0", 12.0)
}

fn default_min_ramp_dia() -> ParamLength {
    param_expr("tool_d * 0.4", 2.4)
}

fn default_seam_overlap() -> ParamLength {
    ParamLength::from_mm(0.5)
}

impl Default for LinkingParams {
    fn default() -> Self {
        Self {
            retract_policy: RetractPolicy::MinimumRetract,
            safe_stay_down_distance: default_stay_down(),
            entry_mode: EntryStrategy::HelicalRamp,
            ramp_angle_deg: default_ramp_angle_deg(),
            min_ramp_diameter: default_min_ramp_dia(),
            lead_in: LeadConfig::default(),
            lead_out_same_as_in: true,
            lead_out: LeadConfig::default(),
            seam_overlap: default_seam_overlap(),
        }
    }
}

impl LinkingParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.safe_stay_down_distance.reevaluate(ctx);
        let _ = self.min_ramp_diameter.reevaluate(ctx);
        self.lead_in.reevaluate(ctx);
        self.lead_out.reevaluate(ctx);
        let _ = self.seam_overlap.reevaluate(ctx);
    }
}
